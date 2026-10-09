"""No hardware imports: run with standard-library unittest discovery."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.offline_agent import ContractError, DryRunController


def fixture():
    packet = dict(
        schema='agent007.robot_review_packet.v1', artifact_id='artifact-a',
        evaluation_id='revision-1', scan_job_id='scan-1', candidate_cycle=0,
        scan_cycle=1, previous_revision=None, frame='ur20_base_link', units='mm',
        evidence_ids=['evidence-1'], summary={'uncertainty': 'Coverage incomplete'},
        suggestions=[dict(category='review_scan_coverage', evidence_ids=['evidence-1'],
                          executable=False, fabrication_authorized=False,
                          requires_operator_review=True)],
    )
    source = {key: copy.deepcopy(packet[key]) for key in (
        'artifact_id', 'scan_job_id', 'candidate_cycle', 'scan_cycle',
        'frame', 'units', 'evidence_ids')}
    return packet, source


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.controller = DryRunController(frame='ur20_base_link', units='mm')
        self.packet, self.source = fixture()

    def review(self, request='request-1'):
        return self.controller.review(self.packet, request_id=request, source=self.source)

    def test_decision_has_no_commands_and_ledger_is_detached(self):
        result = self.review()
        self.assertEqual(result['robot_commands'], [])
        self.assertFalse(result['proposal']['executable'])
        result['evidence_ids'].append('mutated')
        self.assertEqual(self.controller.decisions[0]['evidence_ids'], ['evidence-1'])

    def test_invalid_packets_leave_no_ledger_entry(self):
        mutations = [
            {'frame': 'world'}, {'units': 'cm'}, {'scan_cycle': True},
            {'candidate_cycle': -1}, {'previous_revision': 'unknown'},
            {'evidence_ids': ['unknown']}, {'evidence_ids': ['evidence-1'] * 2},
            {'summary': {}}, {'schema': 'unknown'}, {'suggestions': None},
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                self.packet, self.source = fixture()
                self.packet.update(mutation)
                with self.assertRaises(ContractError):
                    self.review()
                self.assertEqual(self.controller.decisions, [])

    def test_suggestions_cannot_grant_authority(self):
        for key in ('executable', 'fabrication_authorized', 'requires_operator_review'):
            self.packet, self.source = fixture()
            self.packet['suggestions'][0][key] = not self.packet['suggestions'][0][key]
            with self.assertRaises(ContractError):
                self.review()
        self.packet, self.source = fixture()
        self.packet['suggestions'][0]['evidence_ids'] = ['unknown']
        with self.assertRaises(ContractError):
            self.review()

    def test_duplicate_revision_and_request(self):
        self.review()
        with self.assertRaises(ContractError):
            self.review('new-request')
        self.packet['evaluation_id'] = 'revision-2'
        self.packet['previous_revision'] = 'revision-1'
        with self.assertRaises(ContractError):
            self.review()
        self.review('request-2')
        self.assertEqual(len(self.controller.decisions), 2)

    def test_stale_cycle_and_wrong_source(self):
        self.packet['candidate_cycle'] = self.source['candidate_cycle'] = 2
        self.packet['scan_cycle'] = self.source['scan_cycle'] = 3
        self.review()
        self.packet, self.source = fixture()
        self.packet.update(evaluation_id='revision-2', previous_revision='revision-1')
        with self.assertRaises(ContractError):
            self.review('request-2')
        self.source['artifact_id'] = 'wrong-artifact'
        with self.assertRaises(ContractError):
            self.review('request-3')

    def test_execution_fails_closed(self):
        with self.assertRaises(ContractError):
            self.controller.execute('capture')


if __name__ == '__main__':
    unittest.main()
