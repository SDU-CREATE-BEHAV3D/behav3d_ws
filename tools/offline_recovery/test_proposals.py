import unittest

import numpy as np

from geometry import NominalBead
from build_inward_lean import SETTINGS
from proposals import adaptation_proposals, surface_fill_proposals, axis_options
from viability import Scene, collision_library


class ProposalTests(unittest.TestCase):
    def setUp(self):
        self.scene = Scene(np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0.]]),
                           np.array([[0, 1, 2], [0, 2, 3]]))
        self.lib = collision_library()
        self.models = {'test_tool': {'points_tcp': np.array([[0, 0, .1]])}}
        self.record = NominalBead('A', (0, 0, .016), (0, 0, 1), .036, .016).record()

    def test_accessible_deficit_produces_attached_first_bead(self):
        proposals = surface_fill_proposals(np.array([[0, 0, .008]]), self.record,
                                           self.scene, self.lib, self.models, SETTINGS)
        good = [p for p in proposals if p['metrics']['benefit'] and set(p['screens'].values()) == {'pass'}]
        self.assertTrue(good)
        self.assertEqual(good[0]['checks'][0]['support_rays_within_gap'], 9)
        self.assertFalse(good[0]['executable'])

    def test_buried_deficit_does_not_produce_buried_print_targets(self):
        proposals = surface_fill_proposals(np.array([[0, 0, -.025]]), self.record,
                                           self.scene, self.lib, self.models, SETTINGS)
        self.assertTrue(proposals)
        for p in proposals:
            self.assertAlmostEqual(p['targets'][0]['top_m'][2], .016, places=6)
            self.assertEqual(p['metrics']['benefit'], 0)

    def test_search_checks_spacing_against_reserved_candidates(self):
        peer = dict(self.record, label='B', top_m=[.015, 0, .016])
        proposals = surface_fill_proposals(np.array([[0, 0, .008]]), self.record,
                                           self.scene, self.lib, self.models, SETTINGS, peers=[peer])
        self.assertTrue(all(p['screens']['spacing'] == 'fail' for p in proposals))

    def test_bounded_search_preserves_the_seed_and_returns_all_checks(self):
        reference = dict(self.record, top_m=[0, 0, .026])
        proposals = adaptation_proposals(reference, reference, self.scene, self.lib, self.models, SETTINGS)
        self.assertEqual(len(proposals), 66)
        self.assertEqual(reference['top_m'][2], .026)
        for p in proposals[1:]:
            point = np.array(p['targets'][0]['top_m'])
            self.assertLessEqual(np.linalg.norm(point[:2]), .008 + 1e-12)
            self.assertEqual(len(p['checks']), 1)
        self.assertTrue(any(p['metrics']['benefit'] > 0 and p['screens']['support'] == 'pass' for p in proposals))

    def test_axis_search_keeps_declared_angle_bound(self):
        for axis in axis_options([.5, .1, .8]):
            reference = np.array([.5, .1, .8]); reference /= np.linalg.norm(reference)
            self.assertLessEqual(np.degrees(np.arccos(np.clip(np.dot(axis, reference), -1, 1))), 4 + 1e-8)


if __name__ == '__main__': unittest.main()
