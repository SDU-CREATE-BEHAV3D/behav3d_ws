"""Behavioral parity against the original command-line loop and state safety."""
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import unittest
import uuid

import numpy as np

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
ROOT, _ = paths()
SCALAR = ROOT / "scalar_field" if (ROOT / "scalar_field").is_dir() else ROOT / "src/behav3d_py/behav3d_py/scalar_field"
sys.path.insert(0, str(SCALAR))
from workbench.config import Parameters, SimulationOptions
from workbench.session import Session
from lib_scalar.print_targets import write_line_targets_yaml

# Regression fixtures remain independent of editable startup preferences.
BASE = Parameters(mode="gradient_lift", source_spacing_mm=3, count=7,
    separation_mm=16, width_mm=42, height_mm=20, walk_distance_mm=12,
    walk_step_mm=1, walk_max_steps=32, walk_start_fraction=0.25, tangent_sign=1,
    clamp_to_cone=False, cone_deg=45, normal_continuity=True, clearance_mm=0,
    deposit_mode="line", line_fraction=0.22)


class SessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (ROOT / "mesh/fields/field_state_init_subdiv1.npz").exists():
            from prepare_field import prepare_field
            prepare_field(ROOT)
        cls.output = APP_ROOT / "workbench/outputs" / ("tests-" + uuid.uuid4().hex[:8])
        cls.reference = cls.output / "reference"
        cls.reference.mkdir(parents=True)
        cls.hashes = {p: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (SCALAR / "lib_scalar").glob("*.py")}
        args = [sys.executable, str(SCALAR / "field_state_scan_loop_simulator_dds_v2.py"),
            "--field-state", str(ROOT / "mesh/fields/field_state_init_subdiv1.npz"),
            "--scan-mesh", str(ROOT / "mesh/ScanMesh.stl"), "--scan-scale", "0.001", "--scan-yaw-deg", "180",
            "--candidate-mode", "gradient_lift", "--bead-width-mm", "42", "--bead-height-mm", "20",
            "--dds-deposit-mode", "line", "--dds-line-fraction", "0.22", "--dds-voxel-size-mm", "3",
            "--dds-surface-step-size", "2", "--no-vis", "--output-dir", str(cls.reference)]
        proc = subprocess.run(args, input="n\nn\nn\nq\n", text=True, capture_output=True, timeout=180)
        (cls.reference / "baseline.log").write_text(proc.stdout + proc.stderr, encoding="utf-8")
        if proc.returncode:
            raise RuntimeError(proc.stdout + proc.stderr)

    @classmethod
    def tearDownClass(cls):
        assert all(hashlib.sha256(p.read_bytes()).hexdigest() == checksum for p, checksum in cls.hashes.items())

    def session(self, **changes):
        return Session(ROOT, self.output, replace(SimulationOptions(voxel_mm=3, threshold=0.5,
            surface_step=2, padding_mm=24, checkpoint_every=0, max_voxels=30_000_000), **changes))

    def test_parity_with_original_targets_volume_and_surface(self):
        s = self.session()
        for cycle in range(1, 4):
            p = s.prepare(BASE)
            yaml = s.output / "preview.yaml"
            write_line_targets_yaml(yaml, p.targets, position_scale=1000.0)
            self.assertEqual(yaml.read_text(), (self.reference / f"step_{cycle:02d}/segments.yaml").read_text())
            s.accept(p.token)
        expected = np.load(self.reference / "loop_sim_dds_bundle/implicit_field.npy")
        np.testing.assert_allclose(s.result.implicit_field, expected, rtol=0, atol=1e-12)
        import open3d as o3d
        reference_mesh = o3d.io.read_triangle_mesh(str(self.reference / "loop_sim_dds_surface.ply"))
        self.assertEqual(len(s.surface.faces), len(reference_mesh.triangles))
        self.assertIsNone(s.result.coverage)

    def test_preview_is_non_destructive_and_reuses_geometry(self):
        s = self.session()
        p = s.prepare(BASE)
        updated = s.prepare(replace(BASE, source_spacing_mm=1.5, width_mm=50))
        self.assertIs(p.pose, updated.pose)
        self.assertTrue(updated.geometry_cached)
        self.assertGreater(len(updated.candidates.sampling_points), len(p.candidates.sampling_points))
        self.assertEqual(s.geometry_builds, 1)
        self.assertEqual(s.revision, 0)
        self.assertEqual(s.history, [])
        self.assertIsNone(s.simulator)
        s.prepare(replace(BASE, clearance_mm=1))
        self.assertEqual(s.geometry_builds, 2)

    def test_phantom_matches_the_accepted_first_batch_without_mutating_state(self):
        s = self.session()
        preview = s.prepare(BASE, include_phantom=True)
        ghost = preview.phantom_surface
        self.assertFalse(ghost.is_empty)
        self.assertIs(s.preview_phantom(preview.token), ghost)
        self.assertIsNone(s.simulator)
        self.assertEqual(s.revision, 0)
        self.assertFalse(list(s.output.glob("step_*")))
        s.accept(preview.token)
        self.assertEqual(len(ghost.faces), len(s.surface.faces))
        # Mesh ordering can differ because the phantom uses a cropped domain.
        def ordered(vertices):
            rounded = np.round(vertices, 10)
            return rounded[np.lexsort(rounded.T[::-1])]
        # Marching cubes uses float32 local coordinates; cropping can introduce
        # sub-micrometer rounding even on the same voxel lattice.
        np.testing.assert_allclose(ordered(ghost.vertices), ordered(s.surface.vertices), rtol=0, atol=1e-7)

    def test_phantom_after_deposition_preserves_accepted_voxels_and_history(self):
        s = self.session()
        s.accept(s.prepare(BASE).token)
        accepted = s.result
        deposits = s.simulator.deposits
        field = accepted.implicit_field.copy()
        current_scan = s.current_scan
        preview = s.prepare(replace(BASE, width_mm=55), include_phantom=True)
        self.assertFalse(preview.phantom_surface.is_empty)
        self.assertIs(s.result, accepted)
        self.assertIs(s.current_scan, current_scan)
        self.assertEqual(s.simulator.deposits, deposits)
        np.testing.assert_array_equal(s.result.implicit_field, field)
        self.assertEqual(len(s.history), 1)
        with self.assertRaises(ValueError):
            s.preview_phantom("stale-token")

    def test_stale_and_duplicate_accept_are_rejected(self):
        s = self.session()
        first = s.prepare(BASE)
        current = s.prepare(replace(BASE, count=2))
        with self.assertRaises(ValueError):
            s.accept(first.token)
        s.accept(current.token)
        with self.assertRaises(ValueError):
            s.accept(current.token)
        self.assertEqual(s.revision, 1)
        self.assertEqual(len(s.simulator.deposits), 2)

    def test_live_dimensions_do_not_rewrite_past_deposits_and_no_mesh_per_cycle(self):
        s = self.session()
        s.accept(s.prepare(BASE).token)
        s.accept(s.prepare(replace(BASE, width_mm=48, height_mm=25)).token)
        self.assertAlmostEqual(s.simulator.deposits[0].profile.width, 0.042)
        self.assertAlmostEqual(s.simulator.deposits[-1].profile.width, 0.048)
        self.assertEqual(s.history[0]["parameters"]["height_mm"], 20)
        self.assertEqual(s.history[1]["parameters"]["height_mm"], 25)
        self.assertFalse(list(s.output.rglob("*.ply")))
        self.assertTrue((s.output / "step_0002/segments.yaml").exists())
        self.assertEqual(len((s.output / "history.jsonl").read_text().splitlines()), 2)

    def test_domain_expansion_preserves_the_existing_voxel_lattice(self):
        s = self.session()
        s.prepare(BASE)
        planned = s.domain_for_display()
        first = s.accept(s.preview.token)
        self.assertEqual(first["domain_change"], "initialized")
        self.assertEqual(planned, s.domain_for_display())
        old_domain = s.simulator.domain
        old_field = s.result.implicit_field.copy()
        info = s.accept(s.prepare(replace(BASE, width_mm=150)).token)
        self.assertTrue(info["domain_rebuilt"])
        self.assertEqual(info["domain_change"], "expanded")
        self.assertTrue(info["domain"]["expanded_sides"])
        self.assertEqual(tuple(info["domain"]["grid_shape"]), s.simulator.domain.grid_shape)
        new_domain = s.simulator.domain
        shift = (np.array(old_domain.min_corner) - new_domain.min_corner) / new_domain.voxel_size
        np.testing.assert_allclose(shift, np.round(shift), atol=1e-10)
        start = np.round(shift).astype(int)
        slices = tuple(slice(a, a + n) for a, n in zip(start, old_domain.grid_shape))
        self.assertTrue(np.all(s.result.implicit_field[slices] >= old_field - 1e-10))

    def test_capacity_rejection_does_not_accept_or_modify_a_cycle(self):
        s = self.session(max_voxels=10)
        preview = s.prepare(BASE)
        with self.assertRaises(ValueError):
            s.accept(preview.token)
        self.assertEqual(s.revision, 0)
        self.assertEqual(s.history, [])
        self.assertIsNone(s.simulator)

    def test_export_checkpoint_round_trip(self):
        from dds import SimulationResult
        s = self.session(checkpoint_every=1)
        s.accept(s.prepare(BASE).token)
        self.assertTrue((s.output / "checkpoint-0001.npz").exists())
        folder = s.export()
        restored = SimulationResult.load(folder / "simulation.npz")
        np.testing.assert_array_equal(restored.implicit_field, s.result.implicit_field)
        self.assertEqual(len(restored.deposits), 7)
        self.assertEqual(json.loads((folder / "history.json").read_text())[0]["cycle"], 1)
        events = [json.loads(line) for line in (s.output / "performance.jsonl").read_text().splitlines()]
        self.assertTrue(any(e["operation"] == "checkpoint" and e["state"] == "finished" for e in events))


if __name__ == "__main__":
    unittest.main()
