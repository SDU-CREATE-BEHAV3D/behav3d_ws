"""Regression checks for recovered state and bounded offline planner alternatives."""
import hashlib
import json
from pathlib import Path
import unittest

import numpy as np

from replay import (ROOT, BOUNDS, field_identity, generate, historical_axes,
                    inside_bounds, point_comparison, current_targets)

OUTPUT = ROOT/'outputs/offline_planner_replay/recorded_260728'


class ComparisonTests(unittest.TestCase):
    def test_ordered_error_is_not_nearest_neighbor_error(self):
        points = np.array([[0., 0., 0.], [.01, 0., 0.]])
        result = point_comparison(points[::-1], points)
        self.assertEqual(result['ordered_max_mm'], 10.)
        self.assertEqual(result['symmetric_nearest_max_mm'], 0.)

    def test_missing_candidate_is_not_an_exact_match(self):
        points = np.array([[0., 0., 0.], [.01, 0., 0.]])
        result = point_comparison(points[:1], points)
        self.assertIsNone(result['ordered_max_mm'])
        self.assertEqual(result['symmetric_nearest_max_mm'], 10.)
        empty = point_comparison(np.empty((0, 3)), points)
        self.assertIsNone(empty['symmetric_nearest_max_mm'])

    def test_field_match_requires_order_and_visible_scalar_agreement(self):
        vertices = np.array([[0., 0., 0.], [0., 0., .1]])
        heat = np.array([0., .5])
        colors = np.array([[1., 1., 0.], [1., 128/255, 0.]])
        self.assertTrue(field_identity(vertices, heat, vertices.copy(), colors)['heat_within_color_rounding'])
        self.assertFalse(field_identity(vertices, heat, vertices[::-1], colors)['ordered_vertices_exact'])
        colors[1, 1] = .6
        self.assertFalse(field_identity(vertices, heat, vertices, colors)['heat_within_color_rounding'])
        colors[:] = .2
        self.assertFalse(field_identity(vertices, heat, vertices, colors)['heat_within_color_rounding'])

    def test_recovery_bounds_exclude_distant_or_rotated_substitutes(self):
        seed = {'top_m': [0., 0., 0.], 'axis': [0., 0., 1.]}
        for top, axis in [([.04, 0, 0], [0, 0, 1]), ([0, 0, .013], [0, 0, 1]),
                          ([0, 0, 0], [np.sin(np.radians(5)), 0, np.cos(np.radians(5))])]:
            self.assertFalse(inside_bounds({'top_m': top, 'axis': axis}, seed)[0])
        self.assertTrue(inside_bounds({'top_m': [.008, 0, .012], 'axis': [0, 0, 1]}, seed)[0])


class RecordedReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = json.loads((OUTPUT/'results.json').read_text())

    def test_portable_archived_batches_reproduce_through_real_functions(self):
        with np.load(OUTPUT/'baseline_fixture.npz', allow_pickle=False) as data:
            field = {key: data[f'field_{key}'] for key in ['vertices', 'faces', 'heat', 'offset']}
            for cycle in [1, 6, 19, 23]:
                with self.subTest(cycle=cycle):
                    points = generate(data[f'c{cycle}_contour'], data[f'c{cycle}_lines'], field)
                    error = point_comparison(points.points, data[f'c{cycle}_archived'])
                    self.assertIsNotNone(error['ordered_max_mm'])
                    self.assertLess(error['ordered_max_mm'], 1e-5)
                    self.assertLessEqual(float(np.max(np.abs(historical_axes(points, field) - data[f'c{cycle}_axes']))), .0050001)
                    targets, stats = current_targets(points, field)
                    self.assertEqual(targets.count, len(points.points) - stats['endpoint_spacing_removed'])

    def test_complete_baseline_does_not_claim_complete_runtime_state(self):
        result = self.results
        self.assertEqual(len(result['baseline']), 23)
        self.assertEqual(sum(c['saved_contour_replay']['reference_count'] for c in result['baseline']), 157)
        self.assertTrue(all(c['matches_archived_within_1e_5_mm'] for c in result['baseline']))
        self.assertEqual(result['profile']['status'], 'inferred_replay_profile')
        self.assertFalse(result['full_planner_state_complete'])
        self.assertFalse(result['width_map']['used'])
        self.assertFalse(result['width_map']['matches_field_vertex_count'])
        self.assertTrue(any(c['contour_comparison']['symmetric_nearest_max_mm'] > .01 for c in result['upstream']))

    def test_only_bounded_screened_candidates_can_be_selected(self):
        for trial in self.results['trials']:
            eligible = [c for c in trial['candidates'] if c['promising']]
            self.assertEqual(len(eligible), trial['promising_count'])
            for candidate in trial['candidates']:
                bounds = candidate['search_bounds']
                self.assertLessEqual(bounds['xy_change_mm'], BOUNDS['xy_radius_mm'] + 1e-8)
                self.assertLessEqual(bounds['z_change_mm'], BOUNDS['z_change_mm'] + 1e-8)
                self.assertLessEqual(bounds['axis_change_deg'], BOUNDS['axis_change_deg'] + 1e-8)
                self.assertEqual(candidate['promising'], candidate['current_endpoint_spacing_pass'] and candidate['check']['geometric_screen'] == 'passes_declared_screens')
                self.assertFalse(candidate['executable'])
            if trial['selected_id']:
                self.assertIn(trial['selected_id'], [c['id'] for c in eligible])
            else:
                self.assertEqual(eligible, [])
            self.assertFalse(trial['executable'])
            self.assertEqual(trial['robot_commands'], [])
        self.assertEqual(self.results['trials'][0]['decision'], 'use_normal_planner')

    def test_artifacts_and_repository_sources_are_hash_bound(self):
        manifest = json.loads((OUTPUT/'manifest.json').read_text())
        for name, expected in manifest['files_sha256'].items():
            self.assertEqual(hashlib.sha256((OUTPUT/name).read_bytes()).hexdigest(), expected, name)
        for name, expected in self.results['source_sha256'].items():
            # Original input paths are retained for audit; portable tests only
            # require repository files and the copied numerical fixture.
            marker = '/behav3d_ws/'
            if marker in name:
                path = ROOT/name.split(marker, 1)[1]
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected, name)

    def test_orientation_ablation_is_diagnostic_and_does_not_hide_support_failure(self):
        trial = next(t for t in self.results['trials'] if t['angle_deg'] == 8)
        self.assertEqual(trial['candidates'], [])
        self.assertIsNone(trial['selected_id'])
        self.assertEqual(len(trial['axis_limit_ablation']['candidates']), 6)
        for candidate in trial['axis_limit_ablation']['candidates']:
            self.assertGreater(candidate['search_bounds']['axis_change_deg'], BOUNDS['axis_change_deg'])
            self.assertIn('insufficient_nominal_base_support', candidate['check']['rejection_reasons'])
            self.assertEqual(candidate['check']['support_rays_within_gap'], 4)


if __name__ == '__main__':
    unittest.main()
