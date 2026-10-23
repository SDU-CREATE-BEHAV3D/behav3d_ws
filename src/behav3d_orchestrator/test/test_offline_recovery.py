"""Recovery policy tests use no ROS, geometry runtime or robot connection."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.offline_agent.recovery import (
    BINDING_KEYS, VALIDATION_GATES, decide_recovery, digest, planner_request)


def fixture():
    issue = {'schema': 'behav3d.recovery_issue.v1', 'id': 'issue-a', 'artifact_id': 'artifact-a',
             'mesh_revision': 'mesh-a', 'goal_revision': 'goal-a', 'frame': 'base', 'units': 'm',
             'observation_revision': 'scan-b', 'region_id': 'region-a', 'kind': 'lean', 'extent': 'local',
             'connection_status': 'intact', 'objective': 'continue_from_observed', 'benefit_basis': 'support',
             'attempt_count': 0, 'last_outcome': 'not_attempted',
             'evidence': {'origin': 'recorded', 'references': ['report-a'], 'identity_supported': True,
                          'registration_supported': True, 'coverage_sufficient': True, 'failure_confirmed': True,
                          'tolerance_sensitive': False, 'trend': 'stable', 'trend_observation_ids': ['scan-a', 'scan-b']}}
    proposal = {'id': 'candidate-a', 'strategy': 'adapt',
                'targets': [{'label': 'A', 'top_m': [0, 0, .016], 'axis': [0, 0, 1], 'width_m': .036, 'height_m': .016}],
                'screens': {'support': 'pass', 'spacing': 'pass', 'tool_access': 'pass', 'search_bounds': 'pass'},
                'metrics': {'benefit': 4, 'benefit_basis': 'support', 'deviation_mm': 2, 'material_mm3': 10}}
    snapshot = {'status': 'complete', 'snapshot_sha256': 'a' * 64,
                'binding': {k: issue[k] for k in BINDING_KEYS}}
    proposal['binding'] = deepcopy(snapshot['binding'])
    return issue, proposal, snapshot


def receipt(issue, proposal, snapshot):
    return {'schema': 'behav3d.recovery_validation_result.v1', 'validator': 'numerical-test-fixture',
            'request_sha256': digest(planner_request(issue, proposal, snapshot)),
            'gates': {g: {'status': 'pass', 'evidence': ['controlled test']} for g in VALIDATION_GATES}}


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.issue, self.proposal, self.snapshot = fixture()

    def decide(self, **kwargs):
        return decide_recovery(self.issue, [self.proposal], self.snapshot, **kwargs)

    def test_uncertain_deficit_requests_inspection_even_with_a_promising_candidate(self):
        self.issue.update(kind='missing_material', objective='repair_deficit')
        self.proposal['strategy'] = 'fill'
        self.issue['evidence'].update(failure_confirmed=False, tolerance_sensitive=True)
        decision = self.decide()
        self.assertEqual(decision['next_action'], 'inspect_region')
        self.assertIsNone(decision['selected_proposal_id'])

    def test_stable_lean_can_select_a_proposal_for_validation(self):
        self.snapshot['status'] = 'incomplete'
        decision = self.decide()
        self.assertEqual(decision['next_action'], 'validate_recovery_proposal')
        self.assertEqual(decision['selected_proposal_id'], 'candidate-a')
        self.assertTrue(decision['affected_region_deferred'])
        self.assertFalse(decision['executable'])

    def test_progressing_lean_or_compromised_connection_defers_growth(self):
        for key in ('trend', 'connection'):
            self.setUp()
            if key == 'trend': self.issue['evidence']['trend'] = 'progressing'
            else: self.issue['connection_status'] = 'compromised'
            self.assertEqual(self.decide()['next_action'], 'defer_affected_region')

    def test_unknown_trend_requests_an_observation(self):
        self.issue['evidence']['trend'] = 'unknown'
        self.issue['evidence']['trend_observation_ids'] = []
        self.assertEqual(self.decide()['next_action'], 'inspect_region')

    def test_one_observation_cannot_establish_stability(self):
        self.issue['evidence']['trend_observation_ids'] = ['a', 'a']
        with self.assertRaises(ValueError): self.decide()

    def test_restore_existing_shape_does_not_select_adaptive_deposition(self):
        self.issue['objective'] = 'restore_printed_shape'
        result = self.decide()
        self.assertEqual(result['next_action'], 'hold_for_contact_or_goal_study')
        self.assertIsNone(result['selected_proposal_id'])

    def test_failed_support_cannot_be_compensated_by_high_benefit(self):
        bad = deepcopy(self.proposal)
        bad['id'] = 'higher-benefit'
        bad['metrics']['benefit'] = 1000
        bad['screens']['support'] = 'fail'
        decision = decide_recovery(self.issue, [bad, self.proposal], self.snapshot)
        self.assertEqual(decision['selected_proposal_id'], 'candidate-a')

    def test_zero_benefit_has_no_recovery_winner(self):
        self.proposal['metrics']['benefit'] = 0
        self.assertEqual(self.decide()['next_action'], 'hold_and_reassess')

    def test_every_gate_required_and_no_execution_after_pass(self):
        result = receipt(self.issue, self.proposal, self.snapshot)
        decision = self.decide(receipts={'candidate-a': result})
        self.assertEqual(decision['next_action'], 'review_bounded_batch')
        self.assertEqual(decision['robot_commands'], [])
        self.assertFalse(decision['executable'])
        for gate in VALIDATION_GATES:
            altered = deepcopy(result)
            altered['gates'][gate]['status'] = 'unknown'
            self.assertEqual(self.decide(receipts={'candidate-a': altered})['next_action'], 'validate_recovery_proposal')

    def test_failed_planner_gate_removes_proposal(self):
        result = receipt(self.issue, self.proposal, self.snapshot)
        result['gates']['goal']['status'] = 'fail'
        self.assertIsNone(self.decide(receipts={'candidate-a': result})['selected_proposal_id'])

    def test_changed_candidate_cannot_reuse_an_earlier_receipt(self):
        result = receipt(self.issue, self.proposal, self.snapshot)
        self.proposal['targets'][0]['top_m'][0] += .001
        self.assertIn('planner_receipt_does_not_match_request',
                      self.decide(receipts={'candidate-a': result})['alternatives'][0]['validation_blockers'])

    def test_changed_issue_or_snapshot_invalidates_receipt(self):
        for change in ('issue', 'snapshot'):
            self.setUp()
            result = receipt(self.issue, self.proposal, self.snapshot)
            if change == 'issue': self.issue['observation_revision'] = 'new-scan'
            else: self.snapshot['snapshot_sha256'] = 'b' * 64
            self.assertFalse(self.decide(receipts={'candidate-a': result})['alternatives'][0]['validated_for_review'])

    def test_wrong_frame_or_mesh_snapshot_cannot_validate(self):
        for key in ('frame', 'mesh_revision', 'goal_revision'):
            self.setUp()
            self.snapshot['binding'][key] = 'wrong'
            self.assertIn('planner_state_revision_mismatch', self.decide()['alternatives'][0]['validation_blockers'])

    def test_no_unbounded_retries(self):
        for change in ({'attempt_count': 2}, {'last_outcome': 'unchanged'}, {'last_outcome': 'worsened'}):
            self.setUp()
            self.issue.update(change)
            self.assertEqual(self.decide()['next_action'], 'hold_and_reassess')

    def test_major_failure_requests_reassessment(self):
        self.issue['extent'] = 'extensive'
        self.assertEqual(self.decide()['next_action'], 'hold_and_reassess')

    def test_zero_control_preserves_normal_planner(self):
        self.issue['kind'] = 'none'
        self.issue['evidence']['failure_confirmed'] = False
        self.assertEqual(self.decide()['next_action'], 'use_normal_planner')

    def test_priority_needs_independence_and_objective_progress(self):
        self.proposal['strategy'] = 'prioritize'
        self.assertIn('independent_progress_not_verified', self.decide()['alternatives'][0]['rejection_reasons'])

    def test_metrics_with_different_meanings_cannot_be_ranked(self):
        self.proposal['metrics']['benefit_basis'] = 'unrelated_cost'
        with self.assertRaises(ValueError): self.decide()

    def test_result_does_not_mutate_inputs(self):
        before = deepcopy((self.issue, self.proposal, self.snapshot))
        decision = self.decide()
        decision['alternatives'][0]['metrics']['benefit'] = 42
        self.assertEqual(before, (self.issue, self.proposal, self.snapshot))

    def test_geometry_from_a_different_mesh_is_not_reused(self):
        self.proposal['binding']['mesh_revision'] = 'previous-mesh'
        with self.assertRaisesRegex(ValueError, 'revisions'): self.decide()

    def test_proposals_cannot_carry_commands_or_malformed_geometry(self):
        self.proposal['robot_commands'] = ['move']
        with self.assertRaisesRegex(ValueError, 'commands'): self.decide()
        self.proposal.pop('robot_commands')
        self.proposal['targets'][0]['axis'] = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, 'nonzero'): self.decide()


if __name__ == '__main__': unittest.main()
