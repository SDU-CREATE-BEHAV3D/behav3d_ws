import copy
import unittest

import numpy as np

from replay_frontier_cases import CASES, issued_bead


class RecordedCaseGeometryTests(unittest.TestCase):
    def setUp(self):
        self.entries = [{'index': 0, 'plane': 'O(-40,850,10) Z(-0.3,0,1)'},
                        {'index': 1, 'plane': 'O(-70,860,20) Z(0,0,1)'}]
        self.metrics = {'flip_x': True, 'flip_y': True,
                        'bead_width_m': .036, 'bead_height_m': .016}
        self.candidate = {'label': 'C5:T1', 'candidate_cycle': 5,
                          'position_m': [.04, -.85, .01],
                          'axis': (np.array([.3, 0, 1]) / np.linalg.norm([.3, 0, 1])).tolist(),
                          'width_mm': 36., 'height_mm': 16.}

    def test_issued_position_axis_and_dimensions_are_bound(self):
        bead = issued_bead(self.candidate, list(reversed(self.entries)), self.metrics)
        self.assertTrue(np.allclose(bead.top, self.candidate['position_m']))
        self.assertTrue(np.allclose(bead.direction, self.candidate['axis']))
        self.assertAlmostEqual(bead.width_m, .036)

    def test_wrong_sign_axis_dimensions_or_identity_rejected(self):
        changes = [{'position_m': [-.04, .85, .01]}, {'axis': [0, 0, 1]},
                   {'width_mm': 40.}, {'label': 'C4:T1'}, {'label': 'C5:T3'}]
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                issued_bead(dict(self.candidate, **change), self.entries, self.metrics)

    def test_duplicate_or_incomplete_plane_rejected(self):
        with self.assertRaises(ValueError):
            issued_bead(self.candidate, self.entries + [self.entries[0]], self.metrics)
        entries = copy.deepcopy(self.entries)
        entries[0]['plane'] = 'O(-40,850,10)'
        with self.assertRaises(ValueError):
            issued_bead(self.candidate, entries, self.metrics)
        with self.assertRaises(ValueError):
            issued_bead(self.candidate, self.entries, dict(self.metrics, flip_x='true'))

    def test_predeclared_batch_spans_stages_and_keeps_control_case(self):
        labels = [label for label, reason in CASES]
        self.assertEqual(len(labels), len(set(labels)))
        self.assertIn('C5:T1', labels)
        self.assertEqual({int(label.split(':')[0][1:]) + 1 for label in labels}, {2, 6, 12, 19, 23})


if __name__ == '__main__':
    unittest.main()
