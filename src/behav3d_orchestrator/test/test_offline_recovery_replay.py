"""Regression checks on the recorded-scene policy replay and its provenance."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.offline_agent.recovery import decide_recovery

ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / 'outputs/offline_recovery/decisions'


class RecordedReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = json.loads((OUTPUT / 'results.json').read_text())
        cls.cases = {c['id']: c for c in cls.results['cases']}

    def test_recorded_results_reproduce_current_policy(self):
        for case in self.cases.values():
            with self.subTest(case=case['id']):
                self.assertEqual(decide_recovery(case['issue'], case['proposals'], case['planner_state']), case['decision'])

    def test_same_geometry_different_evidence_changes_response(self):
        expectations = {'lean-8-stable': 'validate_recovery_proposal', 'lean-8-unknown': 'inspect_region',
                        'lean-8-progressing': 'defer_affected_region', 'lean-8-restore': 'hold_for_contact_or_goal_study'}
        geometry = set()
        for name, action in expectations.items():
            case = self.cases[name]
            geometry.add(case['issue']['mesh_revision'])
            self.assertEqual(case['decision']['next_action'], action)
            self.assertEqual(case['issue']['evidence']['origin'], 'controlled_experiment')
        self.assertEqual(len(geometry), 1)

    def test_uncertain_recording_remains_uncertain(self):
        case = self.cases['recorded-deficit']
        self.assertEqual(case['issue']['evidence']['origin'], 'recorded')
        self.assertFalse(case['issue']['evidence']['failure_confirmed'])
        self.assertEqual(case['decision']['next_action'], 'inspect_region')
        self.assertIsNone(case['decision']['selected_proposal_id'])

    def test_buried_high_coverage_probe_loses_to_attached_candidate(self):
        case = self.cases['confirmed-deficit-hypothesis']
        by_id = {p['id']: p for p in case['proposals']}
        winner = by_id[case['decision']['selected_proposal_id']]
        buried = by_id['old-centroid-probe']
        self.assertGreater(buried['metrics']['benefit'], winner['metrics']['benefit'])
        self.assertEqual(buried['screens']['support'], 'fail')
        self.assertEqual(winner['screens']['support'], 'pass')

    def test_no_recorded_case_has_full_planner_validation(self):
        for case in self.cases.values():
            self.assertEqual(case['planner_state']['status'], 'incomplete')
            self.assertFalse(case['decision']['executable'])
            self.assertEqual(case['decision']['robot_commands'], [])
            self.assertTrue(case['decision']['checkpoint']['preserve_required_frontier_sweep'])
            self.assertTrue(all(not p['validated_for_review'] for p in case['decision']['alternatives']))

    def test_artifact_and_source_hashes_match(self):
        manifest = json.loads((OUTPUT / 'manifest.json').read_text())
        for name, sha in manifest['files_sha256'].items():
            self.assertEqual(hashlib.sha256((OUTPUT / name).read_bytes()).hexdigest(), sha, name)
        for name, sha in self.results['source_sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), sha, name)


if __name__ == '__main__': unittest.main()
