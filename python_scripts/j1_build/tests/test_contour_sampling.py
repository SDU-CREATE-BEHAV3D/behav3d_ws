"""Regression checks for arc sampling and disconnected contour topology."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scalar_field"))
from lib_scalar.contour_sampling import resample_contour
from lib_scalar.generate_print_points import _build_adjacency, _dijkstra_with_limit, generate_print_points


class ContourSamplingTests(unittest.TestCase):
    def sample(self, points_mm, lines, step_mm=3):
        return resample_contour(np.asarray(points_mm, float) * 0.001, lines, step_mm * 0.001)

    def test_sampling_independent_of_edge_partitions_and_segment_order(self):
        coarse = self.sample([[0, 0, 0], [10, 0, 0]], [[0, 1]])
        # Triangle extraction repeats edge crossings and may reverse segments.
        split = self.sample(
            [[0, 0, 0], [1, 0, 0], [1, 0, 0], [7, 0, 0], [7, 0, 0], [10, 0, 0]],
            [[5, 4], [2, 3], [1, 0]],
        )
        expected = np.array([0, 3, 6, 9, 10]) * 0.001
        np.testing.assert_allclose(np.sort(coarse.source_points[:, 0]), expected)
        np.testing.assert_allclose(np.sort(split.source_points[:, 0]), expected)

    def test_bend_keeps_arc_distance_but_corner_is_not_a_source(self):
        result = self.sample([[0, 0, 0], [2, 0, 0], [2, 5, 0]], [[0, 1], [1, 2]])
        expected = np.array([[0, 0, 0], [2, 1, 0], [2, 4, 0], [2, 5, 0]]) * 0.001
        for point in expected:
            self.assertTrue(np.any(np.linalg.norm(result.source_points - point, axis=1) < 1e-10))
        corner = np.argmin(np.linalg.norm(result.points - [0.002, 0, 0], axis=1))
        self.assertFalse(result.sample_mask[corner])
        origin = np.argmin(np.linalg.norm(result.points, axis=1))
        target = np.argmin(np.linalg.norm(result.points - [0.002, 0.001, 0], axis=1))
        distances = _dijkstra_with_limit(_build_adjacency(result.points, result.lines), origin, 0.02)
        self.assertAlmostEqual(distances[target], 0.003)

    def test_short_disconnected_lines_include_endpoints_without_bridges(self):
        result = self.sample([[0, 0, 0], [1, 0, 0], [10, 0, 0], [17, 0, 0]], [[0, 1], [2, 3]])
        np.testing.assert_allclose(np.sort(result.source_points[:, 0]) * 1000, [0, 1, 10, 13, 16, 17])
        for a, b in result.lines:
            self.assertEqual(result.points[a, 0] < 0.005, result.points[b, 0] < 0.005)

    def test_closed_contour_has_no_duplicate_seam_and_preserves_perimeter(self):
        result = self.sample([[0, 0, 0], [4, 0, 0], [4, 4, 0], [0, 4, 0]], [[2, 1], [0, 3], [2, 3], [1, 0]])
        self.assertEqual(len(result.source_points), 6)  # arc positions 0,3,6,9,12,15
        self.assertEqual(len(np.unique(result.source_points, axis=0)), 6)
        length = np.linalg.norm(result.points[result.lines[:, 1]] - result.points[result.lines[:, 0]], axis=1).sum()
        self.assertAlmostEqual(length, 0.016)

    def test_junctions_are_endpoints_and_sampled_once(self):
        result = self.sample([[0, 0, 0], [5, 0, 0], [0, 5, 0], [-5, 0, 0]], [[0, 1], [0, 2], [0, 3]])
        self.assertEqual(len(result.source_points), 7)
        self.assertEqual(sum(np.linalg.norm(result.source_points, axis=1) < 1e-10), 1)

    def test_selector_uses_only_samples(self):
        samples = self.sample([[0, 0, 0], [1, 0, 0], [10, 0, 0]], [[0, 1], [1, 2]])
        selected = generate_print_points(
            samples.points, samples.lines, samples.points, np.zeros(len(samples.points)),
            count=100, min_spacing=0, merge_tol=1e-10,
            point_valid_mask=samples.sample_mask,
        )
        self.assertEqual(len(selected.source_points), len(samples.source_points))
        self.assertFalse(np.any(np.isclose(selected.source_points[:, 0], 0.001)))

    def test_invalid_spacing_and_empty_contour(self):
        for step in [0, -1, np.nan, np.inf]:
            with self.assertRaises(ValueError):
                self.sample([[0, 0, 0], [1, 0, 0]], [[0, 1]], step)
        empty = resample_contour(np.empty((0, 3)), np.empty((0, 2)), 0.003)
        self.assertEqual(empty.source_points.shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
