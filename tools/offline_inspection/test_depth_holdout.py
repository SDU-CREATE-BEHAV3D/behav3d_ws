import unittest
import numpy as np

from depth_holdout import (backproject_region, corrected_depth, crop_mask,
                           four_folds, order_heldout_views, subset_cloud)
from inspection_policy import qualify_requests


class DepthHoldoutTests(unittest.TestCase):
    def setUp(self):
        self.meta = {'depth_scale': 1000., 'depth_bias_m': -.0051, 'depth_max': 2.,
                     'center_crop_depth': True, 'center_crop_width': 2, 'center_crop_height': 2}

    def test_integer_depth_correction_preserves_unknown(self):
        actual = corrected_depth(np.array([0, 1, 1000], dtype=np.uint16), 1000., -.0051)
        self.assertTrue(np.isnan(actual[0]))
        np.testing.assert_allclose(actual[1:], [.001, .995])
        with self.assertRaises(ValueError): corrected_depth(np.array([1000.]), 1000., 0.)

    def test_crop_and_depth_limits_used_during_backprojection(self):
        mask = crop_mask((4, 4), self.meta)
        self.assertEqual(int(mask.sum()), 4)
        self.assertTrue(mask[1:3, 1:3].all())
        raw = np.full((4, 4), 1000, dtype=np.uint16)
        raw[1, 1] = 0
        camera = {'position_m': [0., 0., 0.], 'optical_axes_world': np.eye(3).tolist()}
        k = {'width': 4, 'height': 4, 'fx': 100., 'fy': 100., 'cx': 1.5, 'cy': 1.5}
        points, counts = backproject_region(raw, camera, k, self.meta, [0, 0, 1], np.eye(3), 36, 16, 8)
        self.assertEqual(len(points), 3)
        self.assertEqual(counts['valid_used_pixels'], 3)
        np.testing.assert_allclose(points[:, 2], .995)
        with self.assertRaises(ValueError):
            backproject_region(raw[:2], camera, k, self.meta, [0, 0, 1], np.eye(3), 36, 16, 8)

    def test_every_capture_held_out_once_and_subsets_do_not_leak_points(self):
        indices = list(range(16))
        clouds = {i: np.array([[float(i), 0, 0]]) for i in indices}
        folds = four_folds(indices)
        self.assertEqual(sorted(i for f in folds for i in f['held_out']), indices)
        for f in folds:
            self.assertEqual(len(f['training']), 12)
            self.assertFalse(set(f['training']) & set(f['held_out']))
            points, views = subset_cloud(clouds, f['training'])
            self.assertEqual(points[:, 0].tolist(), f['training'])
            self.assertFalse(set(views) & set(f['held_out']))
        with self.assertRaises(ValueError): subset_cloud(clouds, [0, 0])

    def test_direction_ranking_uses_pose_and_respects_crop(self):
        poses = [{'index': i, 'position_m': [x, 0, 1],
                  'optical_axes_world': [[1, 0, 0], [0, -1, 0], [0, 0, -1]]}
                 for i, x in enumerate([0., .01, .1, 2.])]
        k = {'width': 100, 'height': 100, 'fx': 50., 'fy': 50., 'cx': 50., 'cy': 50.}
        meta = dict(self.meta, center_crop_width=80, center_crop_height=80)
        self.assertEqual(order_heldout_views(poses, [0], [1, 2, 3], [0, 0, 0], k, meta), [2, 1])
        with self.assertRaises(ValueError): order_heldout_views(poses, [0], [0, 1], [0, 0, 0], k, meta)


class InspectionPolicyTests(unittest.TestCase):
    def setUp(self):
        self.request = {'request_id': 'test', 'candidate_labels': ['C1:T4'],
                        'evidence_ids': ['candidate:C1:T4'], 'action': 'inspect_dds_shape_size'}
        self.candidate = {'label': 'C1:T4', 'evidence_id': 'candidate:C1:T4',
                          'remeasured_in_current_scan': True, 'observation_age_cycles': 0,
                          'evaluation_status': 'within_provisional_position_rule',
                          'dds_status': 'within_tolerance', 'uncertainty_reasons': [],
                          'cap_reasons': [], 'paired_position_evidence_eligible': True}

    def qualify(self):
        return qualify_requests([self.request], self.candidate)

    def test_generic_review_is_not_a_scan_trigger(self):
        accepted, decisions = self.qualify()
        self.assertEqual(accepted, [])
        self.assertEqual(decisions[0]['disposition'], 'retain_sweep_no_specific_extra_scan_need')
        self.assertFalse(decisions[0]['inspection_resolved'])

    def test_specific_shape_discrepancy_requires_usable_evidence(self):
        self.candidate.update(dds_status='missing', evaluation_status='dds_shape_size_review')
        accepted, _ = self.qualify()
        self.assertEqual(accepted, [self.request])
        accepted[0]['candidate_labels'].append('wrong')
        self.assertEqual(self.request['candidate_labels'], ['C1:T4'])
        self.candidate['paired_position_evidence_eligible'] = False
        self.assertEqual(self.qualify()[0], [])

    def test_stale_or_identity_evidence_cannot_trigger_geometry_views(self):
        self.candidate.update(dds_status='missing', observation_age_cycles=1)
        self.assertEqual(self.qualify()[0], [])
        self.candidate.update(observation_age_cycles=0, evaluation_status='paired_identity_review')
        self.assertEqual(self.qualify()[0], [])

    def test_cap_quality_routes_to_depth_evidence(self):
        self.request['action'] = 'review_scan_coverage'
        self.candidate['cap_reasons'] = ['noncircular_or_fragmented_footprint']
        accepted, decisions = self.qualify()
        self.assertEqual(accepted, [])
        self.assertEqual(decisions[0]['disposition'], 'review_raw_depth_quality_before_pose_selection')

    def test_position_review_requires_supported_discrepancy(self):
        self.request['action'] = 'inspect_geometry'
        self.candidate.update(evaluation_status='position_deviation_review', quantitative_position_evidence_eligible=False)
        self.assertEqual(self.qualify()[0], [])
        self.candidate['quantitative_position_evidence_eligible'] = True
        self.assertEqual(self.qualify()[0], [self.request])

    def test_unbound_candidate_is_rejected(self):
        self.request['candidate_labels'] = ['C5:T7']
        with self.assertRaises(ValueError): self.qualify()


if __name__ == '__main__':
    unittest.main()
