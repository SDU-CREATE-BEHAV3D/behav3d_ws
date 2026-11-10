"""Portable continuation parity, lean archives and independent startup defaults."""
from dataclasses import asdict, replace
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
ROOT, _ = paths()
from workbench.config import Parameters, SimulationOptions, DisplayOptions
from workbench.session import Session
from workbench.state import save_state, load_state, file_hash
from workbench.preferences import save_defaults, load_defaults, options_from_dict

PARAMETERS = replace(Parameters(), width_mode="fixed", width_mm=42, height_mm=20, count=3,
                     separation_mm=38, source_spacing_mm=3, clearance_mm=0)
VIEW = dict(display=asdict(DisplayOptions()), camera=dict(position=(1, -2, 1), focal_point=(0, 0, 0),
    up=(0, 0, 1), parallel_scale=0.4, parallel_projection=True, view_angle=30))


class SavedStateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.paths = dict(data_root=ROOT, output_root=self.root / "sessions")

    def session(self, **kwargs):
        return Session(**self.paths, options=SimulationOptions(voxel_mm=3, surface_step=2, checkpoint_every=0), **kwargs)

    def test_resume_matches_uninterrupted_next_cycle_without_replaying_history(self):
        s = self.session()
        for _ in range(2):
            s.accept(s.prepare(PARAMETERS).token)
        expected_field = s.result.implicit_field.copy()
        original_yaml = s.target_history.path.read_bytes()
        path = save_state(s, self.root / "run.b3dstate", PARAMETERS, VIEW)
        with zipfile.ZipFile(path) as bundle:
            names = bundle.namelist()
            self.assertEqual(names.count("dds.npz"), 1)
            self.assertFalse(any(name.endswith((".ply", ".png")) or "cache" in name for name in names))
            self.assertEqual(len([name for name in names if name.startswith("assets/")]), 3)
        loaded, parameters, view = load_state(path, self.paths)
        self.assertEqual(loaded.revision, 2)
        self.assertNotEqual(loaded.output, s.output)
        self.assertEqual(loaded.target_history.path.read_bytes(), original_yaml)
        self.assertEqual(view["camera"]["parallel_scale"], 0.4)
        np.testing.assert_array_equal(loaded.result.implicit_field, expected_field)
        self.assertFalse(np.shares_memory(loaded.result.implicit_field, loaded.simulator._implicit_field_cache))
        a, b = s.prepare(PARAMETERS), loaded.prepare(parameters)
        np.testing.assert_array_equal(a.targets.end_points, b.targets.end_points)
        s.accept(a.token)
        with patch("dds.simulator.accumulate_field", side_effect=AssertionError("Historical replay is unnecessary")):
            loaded.accept(b.token)
        self.assertEqual(loaded.revision, 3)
        np.testing.assert_array_equal(s.result.implicit_field, loaded.result.implicit_field)
        self.assertEqual(s.target_history.path.read_bytes(), loaded.target_history.path.read_bytes())

    def test_field_state_is_portable_after_original_inputs_are_removed(self):
        field = self.root / "goal.npz"
        scan = self.root / "scan.stl"
        width = self.root / "width.npz"
        shutil.copyfile(ROOT / "mesh/fields/field_state_init.npz", field)
        shutil.copyfile(ROOT / "mesh/ScanMesh.stl", scan)
        with np.load(field, allow_pickle=False) as data:
            np.savez_compressed(width, width_norm=1 - data["heat_norm"])
        s = self.session(field_path=field, scan_path=scan)
        p = replace(PARAMETERS, width_mode="field", width_field_path=str(width), width_min_mm=25, width_max_mm=45)
        s.accept(s.prepare(p).token)
        state = save_state(s, self.root / "portable.b3dstate", p, VIEW)
        for path in (field, scan, width):
            path.unlink()
        loaded, parameters, _ = load_state(state, self.paths)
        self.assertTrue(loaded.scan_path.exists())
        self.assertTrue(Path(parameters.width_field_path).exists())
        np.testing.assert_array_equal(loaded.result.implicit_field, s.result.implicit_field)
        preview = loaded.prepare(parameters, include_phantom=True)
        self.assertEqual(preview.parameters.width_mode, "field")
        self.assertFalse(preview.phantom_surface.is_empty)
        loaded.accept(preview.token)
        second = save_state(loaded, self.root / "again.b3dstate", parameters, VIEW)
        again, _, _ = load_state(second, self.paths)
        self.assertEqual(again.revision, 2)

    def test_zero_cycle_state_and_defaults_do_not_create_deposits(self):
        s = self.session()
        state = save_state(s, self.root / "empty.b3dstate", PARAMETERS, VIEW)
        loaded, parameters, _ = load_state(state, self.paths)
        self.assertIsNone(loaded.simulator)
        self.assertEqual(loaded.revision, 0)
        self.assertEqual(loaded.target_history.count, 0)
        settings = self.root / "defaults.json"
        changed = replace(PARAMETERS, count=5)
        display = replace(DisplayOptions(), target_axis_length_mm=18, camera_controls="VTK")
        save_defaults(settings, {"scan_mesh": str(s.scan_path)}, changed, s.options, display, VIEW["camera"])
        saved = load_defaults(settings)
        self.assertEqual(options_from_dict(Parameters, saved["parameters"]), changed)
        self.assertEqual(options_from_dict(DisplayOptions, saved["display"]), display)
        self.assertLess(settings.stat().st_size, 15000)
        self.assertEqual(s.revision, 0)
        settings.write_text("[]")
        with self.assertRaises(ValueError):
            load_defaults(settings)

    def test_failed_save_preserves_previous_file_and_oversized_load_is_rejected_early(self):
        s = self.session()
        s.accept(s.prepare(PARAMETERS).token)
        state = save_state(s, self.root / "safe.b3dstate", PARAMETERS, VIEW)
        original_hash = file_hash(state)
        with self.assertRaises(FileNotFoundError):
            save_state(s, state, replace(PARAMETERS, width_mode="field", width_field_path=str(self.root / "missing.npz")), VIEW)
        self.assertEqual(file_hash(state), original_hash)
        bad = self.root / "too-large.b3dstate"
        with zipfile.ZipFile(state) as source, zipfile.ZipFile(bad, "w", compression=zipfile.ZIP_DEFLATED) as target:
            metadata = json.loads(source.read("state.json"))
            metadata["simulation"]["max_voxels"] = 10
            target.writestr("state.json", json.dumps(metadata))
            for entry in source.infolist():
                if entry.filename != "state.json":
                    with source.open(entry) as incoming, target.open(entry.filename, "w") as outgoing:
                        shutil.copyfileobj(incoming, outgoing)
        with patch("workbench.state.SimulationResult.load", side_effect=AssertionError("Do not allocate this field")):
            with self.assertRaisesRegex(ValueError, "voxel limit"):
                load_state(bad, self.paths)
        self.assertEqual(s.revision, 1)
