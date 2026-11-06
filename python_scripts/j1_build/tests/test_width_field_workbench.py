"""Width-field integration, original-mesh selection and frozen deposition profiles."""
from dataclasses import replace
from pathlib import Path
import json
import sys
import subprocess
import unittest
import uuid

import numpy as np

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
ROOT, _ = paths()
from workbench.config import Parameters, SimulationOptions
from workbench.session import Session


class WidthFieldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.output = APP_ROOT / "workbench/outputs" / ("width-tests-" + uuid.uuid4().hex[:8])
        cls.output.mkdir(parents=True)
        original = ROOT / "mesh/fields/field_state_init.npz"
        with np.load(original, allow_pickle=False) as data:
            vertices = data["field_vertices_scaled"]
            cls.count = len(vertices)
            x = vertices[:, 0]
            values = (x - x.min()) / np.ptp(x)
        cls.map = cls.output / "width.npz"
        np.savez_compressed(cls.map, width_norm=values)
        cls.parameters = replace(Parameters(), width_mode="field", width_field_path=str(cls.map),
                                 width_min_mm=22, width_max_mm=50, overlap_mm=4)

    def session(self):
        return Session(ROOT, self.output, SimulationOptions(voxel_mm=3, surface_step=2, checkpoint_every=0))

    def test_original_mesh_profiles_phantom_and_history(self):
        s = self.session()
        fixed_count = len(s.field.heat_norm)
        p = s.prepare(self.parameters, include_phantom=True)
        self.assertEqual(s.field_path.name, "field_state_init.npz")
        self.assertEqual(len(s.field.heat_norm), self.count)
        self.assertGreater(fixed_count, self.count)
        self.assertGreater(p.targets.count, 1)
        self.assertGreater(np.ptp(p.bead_widths_m), 0.001)
        self.assertTrue(np.all((p.bead_widths_m >= 0.022) & (p.bead_widths_m <= 0.050)))
        expected = p.bead_widths_m.copy()
        self.assertFalse(p.phantom_surface.is_empty)
        self.assertEqual(s.revision, 0)
        record = s.accept(p.token)
        np.testing.assert_allclose([d.profile.width for d in s.simulator.deposits], expected)
        np.testing.assert_allclose(record["bead_widths_mm"], expected * 1000)
        self.assertEqual(len(p.phantom_surface.faces), len(s.surface.faces))
        self.assertTrue((s.output / record["inputs"]["width_snapshot"]).exists())
        self.assertIn("volume_mm3", (s.output / "step_0001/segments.yaml").read_text())
        old_deposits = s.simulator.deposits
        fixed = s.prepare(replace(self.parameters, width_mode="fixed"))
        self.assertEqual(len(s.field.heat_norm), fixed_count)
        self.assertFalse(fixed.geometry_cached)
        self.assertEqual(s.simulator.deposits, old_deposits)
        np.testing.assert_allclose(fixed.bead_widths_m, Parameters().width_mm * 0.001)

    def test_live_range_changes_reuse_geometry_and_freeze_preview(self):
        s = self.session()
        first = s.prepare(self.parameters)
        next_preview = s.prepare(replace(self.parameters, width_min_mm=30, width_max_mm=60))
        self.assertTrue(next_preview.geometry_cached)
        self.assertIs(first.pose, next_preview.pose)
        with self.assertRaises(ValueError):
            s.accept(first.token)
        s.accept(next_preview.token)
        np.testing.assert_allclose([d.profile.width for d in s.simulator.deposits], next_preview.bead_widths_m)

    def test_wrong_vertex_count_is_rejected_without_changing_goal(self):
        s = self.session()
        original_path = s.field_path
        bad = self.output / "wrong.npz"
        np.savez_compressed(bad, width_norm=np.zeros(self.count + 1))
        with self.assertRaisesRegex(ValueError, "length does not match"):
            s.prepare(replace(self.parameters, width_field_path=str(bad)))
        self.assertEqual(s.field_path, original_path)
        self.assertEqual(s.revision, 0)

    def test_matches_the_existing_field_mode_cli(self):
        p = replace(self.parameters, separation_mm=16, height_mm=20, count=7,
                    walk_distance_mm=12, walk_step_mm=1, walk_max_steps=32,
                    walk_start_fraction=0.25, mode="gradient_lift", tangent_sign=1,
                    clamp_to_cone=False, cone_deg=45, normal_continuity=True,
                    clearance_mm=0, deposit_mode="line", line_fraction=0.22,
                    source_spacing_mm=3)
        reference = self.output / "reference"
        scalar = ROOT / "scalar_field" if (ROOT / "scalar_field").exists() else ROOT / "src/behav3d_py/behav3d_py/scalar_field"
        args = [sys.executable, str(scalar / "field_state_scan_loop_simulator_dds_v2.py"),
                "--field-state", str(ROOT / "mesh/fields/field_state_init.npz"),
                "--scan-mesh", str(ROOT / "mesh/ScanMesh.stl"), "--scan-scale", "0.001", "--scan-yaw-deg", "180",
                "--candidate-width-mode", "field", "--width-field", str(self.map),
                "--bead-width-min-mm", "22", "--bead-width-max-mm", "50", "--bead-overlap-mm", "4",
                "--bead-height-mm", "20", "--dds-voxel-size-mm", "3", "--dds-surface-step-size", "2",
                "--dds-padding-mm", "60", "--dds-deposit-mode", "line", "--dds-line-fraction", "0.22",
                "--candidate-mode", "gradient_lift", "--no-vis", "--output-dir", str(reference)]
        proc = subprocess.run(args, input="n\nn\nq\n", text=True, capture_output=True, timeout=180)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        s = self.session()
        for cycle in (1, 2):
            s.accept(s.prepare(p).token)
            self.assertEqual((s.output / f"step_{cycle:04d}/segments.yaml").read_text(),
                             (reference / f"step_{cycle:02d}/segments.yaml").read_text())
        field = np.load(reference / "loop_sim_dds_bundle/implicit_field.npy")
        np.testing.assert_allclose(s.result.implicit_field, field, rtol=0, atol=1e-12)
