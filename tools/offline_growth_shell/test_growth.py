"""Constraints for observed-shell-driven, non-executable goal revisions."""
import hashlib
import json
from pathlib import Path
import unittest

import numpy as np

from growth import Frontier, fit_shell_offsets, predict_offset, revise_shell, mesh_quality, return_diagnostics

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT/'outputs/offline_growth_shell/scan19'


class ShellConstructionTests(unittest.TestCase):
    def setUp(self):
        self.frontier = Frontier(np.array([-.92, -.84]), np.array([-.04, -.04]))
        self.fit = {'z_reference_m': -.04, 'intercept_xy_m': [-.008, .002], 'slope_xy_per_m': [0., 0.]}

    def test_fit_recovers_corresponded_horizontal_displacement(self):
        top = np.column_stack([np.full(6, .24), np.full(6, -.88), np.linspace(-.075, -.025, 6)])
        expected = top-[0, 0, .008]
        delta = np.column_stack([.1*(top[:, 2]+.075), np.full(6, .002), np.full(6, .001)])
        result = fit_shell_offsets(top, expected, expected+delta)
        np.testing.assert_allclose(predict_offset(result, top[:, 2]), delta[:, :2], atol=1e-14)
        np.testing.assert_allclose(result['vertical_residual_range_mm'], [1, 1])

    def test_fit_refuses_sparse_or_height_degenerate_evidence(self):
        for points in [np.zeros((3, 3)), np.zeros((6, 3))]:
            with self.assertRaises(ValueError):
                fit_shell_offsets(points, points, points)

    def test_future_growth_keeps_z_and_monotonically_reduces_offset(self):
        points = np.column_stack([np.full(101, .24), np.full(101, -.88), np.linspace(-.04, .02, 101)])
        revised = revise_shell(points, self.frontier, self.fit, return_height_m=.04)
        np.testing.assert_array_equal(revised[:, 2], points[:, 2])
        error = np.linalg.norm(revised-points, axis=1)
        self.assertTrue(np.all(np.diff(error) <= 1e-12))
        self.assertTrue(np.all(np.diff(revised[:, 2]) > 0))
        np.testing.assert_allclose(revised[0, :2]-points[0, :2], [-.008, .002])
        np.testing.assert_array_equal(revised[points[:, 2] >= 0.], points[points[:, 2] >= 0.])

    def test_original_and_zero_offset_controls_remain_exact(self):
        points = np.array([[.24, -.88, -.04], [.24, -.88, .02]])
        np.testing.assert_array_equal(revise_shell(points, self.frontier, self.fit, return_height_m=None, mode='original'), points)
        fit = dict(self.fit, intercept_xy_m=[0, 0])
        np.testing.assert_array_equal(revise_shell(points, self.frontier, fit, return_height_m=.04), points)

    def test_lower_and_other_wall_vertices_are_protected_without_mutating_input(self):
        points = np.array([[.24, -.88, -.075], [.18, -.88, -.02], [.24, -.98, -.02]])
        original = points.copy()
        np.testing.assert_array_equal(revise_shell(points, self.frontier, self.fit, return_height_m=.04), original)
        np.testing.assert_array_equal(points, original)

    def test_excessive_revision_and_invalid_height_are_rejected(self):
        points = np.array([[.24, -.88, -.04]])
        with self.assertRaisesRegex(ValueError, 'bound'):
            revise_shell(points, self.frontier, dict(self.fit, intercept_xy_m=[-.02, 0]), return_height_m=.04)
        for h in [0, -1, float('nan'), None]:
            with self.assertRaises(ValueError):
                revise_shell(points, self.frontier, self.fit, return_height_m=h)

    def test_longer_return_is_not_reported_complete_when_goal_ends(self):
        vertices = np.array([[.24, y, z] for y in np.linspace(-.925, -.835, 100) for z in [-.04, 0]])
        short = return_diagnostics(vertices, self.frontier, self.fit, .02)
        long = return_diagnostics(vertices, self.frontier, self.fit, .06)
        self.assertEqual(short['complete_columns'], short['column_count'])
        self.assertEqual(long['complete_columns'], 0)
        self.assertGreater(long['maximum_residual_at_roof_mm'], 0)

    def test_mesh_face_reversal_is_detected(self):
        vertices = np.array([[0.,0,0], [1,0,0], [0,1,0]])
        faces = np.array([[0,1,2]])
        original = mesh_quality(vertices, vertices, faces)
        flipped = mesh_quality(vertices, vertices[[0,2,1]], faces)
        self.assertEqual(original['reversed_faces_relative_to_source'], 0)
        self.assertEqual(flipped['reversed_faces_relative_to_source'], 1)


class RecordedExperimentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = json.loads((OUTPUT/'results.json').read_text())
        cls.evidence = json.loads((OUTPUT/'shell_evidence.json').read_text())

    def test_recorded_pairs_preserve_age_and_registration_limitations(self):
        self.assertEqual(self.evidence['current_scan_pair_count'], 1)
        self.assertEqual(len(self.evidence['candidate_labels']), 7)
        self.assertEqual(self.evidence['registration_status'], 'unverified')
        self.assertFalse(self.evidence['past_observations_remeasured'])
        self.assertFalse(self.evidence['physical_failure_confirmed'])

    def test_four_angles_five_strategies_never_become_execution_authority(self):
        self.assertEqual([t['angle_deg'] for t in self.results['trials']], [0,4,8,12])
        for trial in self.results['trials']:
            self.assertEqual(len(trial['cases']), 5)
            self.assertFalse(trial['physical_failure_detected'])
            for case in trial['cases']:
                self.assertFalse(case['executable'])
                self.assertEqual(case['robot_commands'], [])
                self.assertIsNone(case['selected_for_execution'])
                self.assertIn('structural_stability', case['checks_pending'])
                for candidate in case['alternatives']:
                    self.assertEqual(candidate['passes_experimental_geometry'], not candidate['rejection_reasons'])
                    self.assertFalse(candidate['executable'])

    def test_proposed_fields_keep_z_topology_and_original_control(self):
        with np.load(OUTPUT/'proposed_fields.npz', allow_pickle=False) as fields:
            original = fields['original_vertices_m']
            for trial in self.results['trials']:
                for case in trial['cases']:
                    vertices = fields[case['id'].replace('-', '_')]
                    np.testing.assert_array_equal(vertices[:,2], original[:,2])
                    self.assertLessEqual(np.linalg.norm(vertices-original, axis=1).max(), .012+1e-12)
                    if case['key'] == 'original':
                        np.testing.assert_array_equal(vertices, original)
                    expected = hashlib.sha256(vertices.astype('<f8').tobytes()+fields['faces'].astype('<i4').tobytes()).hexdigest()
                    self.assertEqual(case['revised_goal_sha256'], expected)

    def test_artifact_and_repository_source_hashes_match(self):
        manifest = json.loads((OUTPUT/'manifest.json').read_text())
        for name, expected in manifest['files_sha256'].items():
            self.assertEqual(hashlib.sha256((OUTPUT/name).read_bytes()).hexdigest(), expected, name)
        for name, expected in self.results['source_sha256'].items():
            if '/behav3d_ws/' in name:
                path = ROOT/name.split('/behav3d_ws/', 1)[1]
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected, name)


if __name__ == '__main__':
    unittest.main()
