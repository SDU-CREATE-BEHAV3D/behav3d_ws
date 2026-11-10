"""Check cumulative YAML fidelity and unlabeled physical-size XYZ frames."""
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
paths()
from field_scan_loop_simulator_dds_v2 import load_segment_targets_yaml
from lib_scalar.print_targets import OrientedLineTargets, write_line_targets_yaml
from workbench.target_history import TargetHistory, rebuild_history, load_frame_yaml
from workbench.target_frames import frame_glyphs, target_axes, select_target_frames


class TargetHistoryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def batch(self, cycle, volumes=None):
        path = self.root / f"step_{cycle:04d}/segments.yaml"
        starts = np.array([[0.1234567, -0.555555, 0.01], [0.15, -0.51, 0.03]])
        z = np.array([[0.1234, 0.0, 0.99236], [0, 0, 1]])
        write_line_targets_yaml(path, OrientedLineTargets(starts, starts + 0.005, z, z),
                                position_scale=1000, volumes_mm3=volumes)
        return path, load_segment_targets_yaml(path, position_scale=1000)

    def test_exact_planes_and_mixed_optional_volumes_are_preserved(self):
        history = TargetHistory(self.root)
        self.assertEqual(yaml.safe_load(history.path.read_text()), {"segments": []})
        expected = []
        applied_batches = []
        for cycle, volumes in [(1, None), (2, np.array([1234.567891, 3456.765432]))]:
            path, applied = self.batch(cycle, volumes)
            rows = yaml.safe_load(path.read_text())["segments"]
            before = path.read_bytes()
            history.add_batch(path, applied, cycle)
            self.assertEqual(path.read_bytes(), before)
            for row in rows:
                row["index"] = len(expected)
                expected.append(row)
            applied_batches.append(applied)
        self.assertEqual(yaml.safe_load(history.path.read_text())["segments"], expected)
        self.assertEqual(history.count, 4)
        restored = load_segment_targets_yaml(history.path, position_scale=1000)
        np.testing.assert_array_equal(restored.start_points, np.concatenate([a.start_points for a in applied_batches]))
        np.testing.assert_array_equal(history.points[0::2], restored.start_points)
        np.testing.assert_array_equal(history.points[1::2], restored.end_points)
        np.testing.assert_array_equal(history.z_dirs[1::2], restored.end_z_dirs)
        points, z_dirs = load_frame_yaml(history.path)
        np.testing.assert_array_equal(points, history.points)
        np.testing.assert_array_equal(z_dirs, history.z_dirs)
        original = history.path.read_bytes()
        with self.assertRaises(ValueError):
            history.add_batch(path, applied, 2)
        self.assertEqual(history.path.read_bytes(), original)
        # Old-session reconstruction yields exactly the same cumulative file.
        rebuilt = rebuild_history(self.root)
        self.assertEqual(rebuilt.path.read_bytes(), original)

    def test_frames_are_right_handed_and_keep_the_recorded_z_direction(self):
        z = np.array([[0, 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0], [0.2, 0.4, 0.8]])
        before = z.copy()
        axes = target_axes(z)
        np.testing.assert_array_equal(z, before)
        np.testing.assert_allclose(axes[:, 2], z / np.linalg.norm(z, axis=1)[:, None])
        np.testing.assert_allclose(np.cross(axes[:, 0], axes[:, 1]), axes[:, 2], atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(axes), 1, atol=1e-12)
        # Unit tubes must measure 14 mm in world units along each axis.
        meshes = frame_glyphs(np.zeros((1, 3)), np.array([[0, 0, 1]]), 14)
        for axis, mesh in enumerate(meshes):
            self.assertAlmostEqual(float(np.ptp(mesh.points[:, axis])), 0.014, places=8)
        self.assertTrue(all(mesh.n_points == 0 for mesh in frame_glyphs([], [], 14)))

    def test_endpoint_selection_and_thickness_do_not_modify_saved_frames(self):
        points = np.arange(18, dtype=float).reshape(6, 3)
        dirs = np.tile((0, 0, 1), (6, 1)).astype(float)
        before = points.copy()
        ends, end_dirs = select_target_frames(points, dirs)
        np.testing.assert_array_equal(ends, points[[1, 3, 5]])
        np.testing.assert_array_equal(end_dirs, dirs[[1, 3, 5]])
        starts, _ = select_target_frames(points, dirs, True, False)
        np.testing.assert_array_equal(starts, points[[0, 2, 4]])
        both, _ = select_target_frames(points, dirs, True, True)
        np.testing.assert_array_equal(both, before)
        hidden, _ = select_target_frames(points, dirs, False, False)
        self.assertEqual(len(hidden), 0)
        np.testing.assert_array_equal(points, before)
        thin = frame_glyphs([[0, 0, 0]], [[0, 0, 1]], 14, thickness=1)[2]
        thick = frame_glyphs([[0, 0, 0]], [[0, 0, 1]], 14, thickness=1.4)[2]
        self.assertGreater(np.ptp(thick.points[:, 0]), np.ptp(thin.points[:, 0]))
        self.assertAlmostEqual(float(np.ptp(thick.points[:, 2])), 0.014, places=8)
