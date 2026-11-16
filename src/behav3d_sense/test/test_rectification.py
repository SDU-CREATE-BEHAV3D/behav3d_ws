"""Numerical and mixed-camera-model regression checks; no ROS required."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import cv2

SPEC = importlib.util.spec_from_file_location("rectification", Path(__file__).parents[1] / "behav3d_sense/reconstruct/utils/rectification.py")
rectification = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rectification)


class RectificationTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[50., 0., 31.5], [0., 52., 23.5], [0., 0., 1.]])
        self.size = (64, 48)

    def test_depth_matches_analytic_radial_projection(self):
        yy, xx = np.mgrid[:48, :64]
        raw = (1000 + xx + 64 * yy).astype(np.uint16)
        before = raw.copy()
        d = np.array([.18, -.03, .001, -.002, .01, .02, 0., 0.])
        actual = rectification.rectify_images([raw], self.k, d, self.size, depth=True)[0]
        x, y = (xx - 31.5) / 50, (yy - 23.5) / 52
        r2 = x*x + y*y
        radial = (1 + d[0]*r2 + d[1]*r2**2 + d[4]*r2**3) / (1 + d[5]*r2)
        u = np.rint(50 * (x*radial + 2*d[2]*x*y + d[3]*(r2+2*x*x)) + 31.5).astype(int)
        v = np.rint(52 * (y*radial + d[2]*(r2+2*y*y) + 2*d[3]*x*y) + 23.5).astype(int)
        valid = (u >= 0) & (u < 64) & (v >= 0) & (v < 48)
        expected = np.zeros_like(raw)
        expected[valid] = raw[v[valid], u[valid]]
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(raw, before)

    def test_depth_edges_and_invalid_pixels_are_not_interpolated(self):
        raw = np.zeros((48, 64), np.uint16)
        raw[:, 10:32] = 300
        raw[:, 32:54] = 800
        out = rectification.rectify_images([raw], self.k, [.3, 0, 0, 0, 0], self.size, depth=True)[0]
        self.assertEqual(set(np.unique(out)), {0, 300, 800})

    def test_zero_distortion_is_identity(self):
        raw = np.arange(48*64, dtype=np.uint16).reshape(48, 64)
        np.testing.assert_array_equal(rectification.rectify_images([raw], self.k, np.zeros(8), self.size, depth=True)[0], raw)

    def test_invalid_calibration_and_image_fail(self):
        raw = np.ones((48, 64), np.uint16)
        for matrix, distortion, image in ((self.k, [np.nan]*5, raw), (self.k, [0]*3, raw), (self.k, [0]*5, raw[:-1]), (self.k, [0]*5, raw.astype(np.float32))):
            with self.assertRaises(ValueError):
                rectification.rectify_images([image], matrix, distortion, self.size, depth=True)

    def test_mixed_stale_and_incomplete_alignment_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "scan").mkdir()
            (root / "scan/manifest.yaml").write_text("captures: []")
            for name in ("depth", "color", "extrinsics"):
                (root / name).write_text(name)
            session = SimpleNamespace(path=root, _scan_folder="scan", depth_intrinsics_path=root/"depth", color_intrinsics_path=root/"color", _camera_extrinsics_path=root/"extrinsics")
            out = root / "aligned"
            rectification.validate_alignment_metadata(out, session, 3, rectified=False)
            with self.assertRaises(ValueError):
                rectification.validate_alignment_metadata(out, session, 3, rectified=True)
            rectification.write_alignment_metadata(out, session, 3, complete=False)
            with self.assertRaises(ValueError):
                rectification.validate_alignment_metadata(out, session, 3, rectified=True)
            for i in range(3):
                cv2.imwrite(str(out / f"color_in_depth_{i:04d}.png"), np.zeros((48,64,3),np.uint8))
            rectification.write_alignment_metadata(out, session, 3, complete=True)
            rectification.validate_alignment_metadata(out, session, 3, rectified=True)
            with self.assertRaises(ValueError):
                rectification.validate_alignment_metadata(out, session, 3, rectified=False)
            with self.assertRaises(ValueError):
                rectification.validate_alignment_metadata(out, session, 2, rectified=True)
            (root / "depth").write_text("different calibration")
            with self.assertRaises(ValueError):
                rectification.validate_alignment_metadata(out, session, 3, rectified=True)

    def test_failed_reload_invalidates_previous_rectified_alignment(self):
        sys.path.insert(0, str(Path(__file__).parents[1]))
        from behav3d_sense.reconstruct import color_to_depth
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "scan").mkdir()
            (root / "scan/manifest.yaml").write_text("captures: [{}, {}]")
            for name in ("depth", "color", "extrinsics"):
                (root / name).write_text(name)
            session = SimpleNamespace(path=root, _scan_folder="scan", depth_intrinsics_path=root/"depth", color_intrinsics_path=root/"color", _camera_extrinsics_path=root/"extrinsics")
            out = root / "scan/reconstruct/color_in_depth_rectified"
            out.mkdir(parents=True)
            for i in range(2):
                cv2.imwrite(str(out / f"color_in_depth_{i:04d}.png"), np.zeros((48,64,3),np.uint8))
            rectification.write_alignment_metadata(out, session, 2, complete=True)
            with patch.object(color_to_depth, "Session", return_value=session), patch.object(color_to_depth, "ColorToDepthAligner", side_effect=ValueError("unreadable color")):
                with self.assertRaisesRegex(ValueError, "unreadable color"):
                    color_to_depth.run(str(root), "scan", rectify_images=True)
            self.assertFalse(json.loads((out / rectification.ALIGNMENT_METADATA).read_text())["complete"])

    def test_missing_middle_and_duplicate_frame_indices_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            out=Path(temporary)
            for name in ('color_in_depth_0000.png','color_in_depth_0002.png'):
                (out/name).touch()
            with self.assertRaisesRegex(ValueError,'missing'):
                rectification.aligned_image_paths(out,3)
            (out/'color_in_depth_0001.png').touch()
            self.assertEqual(len(rectification.aligned_image_paths(out,3)),3)
            (out/'color_in_depth_1.png').touch()
            with self.assertRaisesRegex(ValueError,'extra'):
                rectification.aligned_image_paths(out,3)

    def test_failed_image_write_cannot_complete_alignment(self):
        from behav3d_sense.reconstruct.color_to_depth import ColorToDepthAligner
        with tempfile.TemporaryDirectory() as temporary:
            aligner=object.__new__(ColorToDepthAligner)
            aligner.output_dir=Path(temporary)
            aligner.depth_images=[np.ones((48,64),np.uint16)]
            aligner.color_images=[np.zeros((48,64,3),np.uint8)]
            with patch.object(aligner,'_align_color_to_depth',return_value=(aligner.color_images[0],None)), patch('cv2.imwrite',return_value=False):
                with self.assertRaises(OSError):
                    aligner.test_one_frame(0,visualize=False)
            self.assertFalse((aligner.output_dir/'color_in_depth_0000.png').exists())


if __name__ == "__main__":
    unittest.main()
