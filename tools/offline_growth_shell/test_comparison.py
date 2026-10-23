"""Meaningful old/new association and exported-proposal checks."""
import json
from pathlib import Path
import unittest

import numpy as np

from comparison import candidate_delta, compare_batches

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT/'outputs/offline_growth_shell/scan19'


def candidate(name, x, axis=(0, 0, 1), support=4, gap=8.):
    return {'id': name, 'target': {'top_m': [x, 0., 0.], 'axis': list(axis)},
            'check': {'support_rays_within_gap': support, 'minimum_batch_spacing_mm': None,
                      'tool_models': {'test': {'minimum_sampled_clearance_mm': gap}}}}


class CandidateComparisonTests(unittest.TestCase):
    def test_signed_shift_and_direction_angle_are_independent(self):
        old = candidate('old', .1)
        new = candidate('new', .103, axis=(0, 2, 0), support=6, gap=7.)
        before = json.dumps(new)
        d = candidate_delta(new, old)
        np.testing.assert_allclose(d['top_delta_xyz_mm'], [3., 0., 0.])
        self.assertAlmostEqual(d['axis_change_deg'], 90.)
        self.assertEqual(d['support_ray_delta'], 2)
        self.assertEqual(d['sampled_tool_clearance_delta_mm'], -1.)
        self.assertIsNone(d['batch_spacing_delta_mm'])
        self.assertEqual(json.dumps(new), before)

    def test_reordering_does_not_become_a_position_change(self):
        old = [candidate('a', 0.), candidate('b', .04)]
        new = [candidate('n1', .04), candidate('n2', 0.)]
        result = compare_batches(new, old)
        self.assertEqual([(p['new_id'], p['old_id']) for p in result['pairs']], [('n1', 'b'), ('n2', 'a')])
        self.assertTrue(all(p['top_shift_mm'] == 0 for p in result['pairs']))
        self.assertFalse(result['identity_confirmed'])

    def test_global_association_does_not_reuse_old_target(self):
        old = [candidate('a', 0.), candidate('b', .01)]
        new = [candidate('n1', .006), candidate('n2', .011)]
        result = compare_batches(new, old, 10.)
        self.assertEqual([(p['new_id'], p['old_id']) for p in result['pairs']], [('n1', 'a'), ('n2', 'b')])

    def test_unmatched_candidates_are_preserved_in_both_directions(self):
        old = [candidate('a', 0.), candidate('b', .04)]
        new = [candidate('n1', .001), candidate('n2', .2)]
        result = compare_batches(new, old)
        self.assertEqual(result['unmatched_old_ids'], ['b'])
        self.assertEqual(result['unmatched_new_ids'], ['n2'])
        self.assertEqual(compare_batches([], old)['unmatched_old_ids'], ['a', 'b'])
        self.assertEqual(compare_batches(new, [])['unmatched_new_ids'], ['n1', 'n2'])

    def test_invalid_association_inputs_fail(self):
        with self.assertRaises(ValueError):
            compare_batches([], [], 0)
        with self.assertRaises(ValueError):
            compare_batches([candidate('a', 0), candidate('a', .01)], [])
        with self.assertRaises(ValueError):
            compare_batches([candidate('a', float('nan'))], [candidate('b', 0)])


class RecordedComparisonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = json.loads((OUTPUT/'results.json').read_text())
        cls.export = json.loads((OUTPUT/'candidate_comparison.json').read_text())

    def test_all_archived_targets_are_preserved_and_rescored_on_each_scan(self):
        old = json.loads((ROOT/'outputs/offline_recovery/inward_cycle19/results.json').read_text())
        expected = old['trials'][0]['plans'][0]['targets']
        for trial in self.results['trials']:
            candidates = trial['archived_candidates_on_same_scan']
            self.assertEqual([c['target'] for c in candidates], expected)
            self.assertEqual([c['check']['label'] for c in candidates], [c['label'] for c in expected])
        a, b = self.results['trials'][0], self.results['trials'][2]
        self.assertNotEqual(a['archived_candidates_on_same_scan'][-1]['check'],
                            b['archived_candidates_on_same_scan'][-1]['check'])

    def test_full_batches_and_comparisons_are_exported_without_execution(self):
        self.assertFalse(self.export['executable'])
        self.assertEqual(self.export['robot_commands'], [])
        self.assertEqual(len(self.export['trials']), 4)
        for trial, exported in zip(self.results['trials'], self.export['trials']):
            archived = trial['archived_candidates_on_same_scan']
            baseline = trial['cases'][0]['generated_batch']['candidates']
            for case, external in zip(trial['cases'], exported['cases']):
                batch = case['generated_batch']
                new = batch['candidates']
                self.assertEqual(external['generated_batch'], batch)
                self.assertEqual(len(new), case['normal_planner_current_count'])
                self.assertEqual(len(batch['secondary_rule_stats']['kept_indices']), len(new))
                self.assertEqual(case['batch_versus_archived'], compare_batches(new, archived))
                self.assertEqual(case['batch_versus_unchanged_goal'], compare_batches(new, baseline))
                for c in new:
                    self.assertFalse(c['executable'])
                    self.assertEqual(c['passes_experimental_geometry'], not c['rejection_reasons'])
                    self.assertAlmostEqual(np.linalg.norm(c['target']['axis']), 1.)
                    self.assertEqual(c['check']['label'], c['id'])
                    actual_spacing = min((np.linalg.norm(np.asarray(c['target']['top_m'])-other['target']['top_m'])*1000
                                          for other in new if other is not c), default=None)
                    self.assertAlmostEqual(c['check']['minimum_batch_spacing_mm'], actual_spacing)
            self.assertTrue(all(p['top_shift_mm'] == 0 and p['axis_change_deg'] < 1e-10
                                for p in trial['cases'][0]['batch_versus_unchanged_goal']['pairs']))

    def test_local_comparison_uses_same_reference_and_reserved_peers(self):
        for trial in self.results['trials']:
            archived = trial['archived_candidates_on_same_scan']
            old = next(c for c in archived if c['id'] == 'C19:T7')
            peers = [c['target']['top_m'] for c in archived if c is not old]
            for case in trial['cases']:
                for new in case['alternatives']:
                    self.assertEqual(new['versus_archived'], candidate_delta(new, old))
                    spacing = np.linalg.norm(np.asarray(peers)-new['target']['top_m'], axis=1).min()*1000
                    self.assertAlmostEqual(new['check']['minimum_batch_spacing_mm'], spacing)


if __name__ == '__main__':
    unittest.main()
