"""Standard-library tests: no ROS, model service or hardware imports."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.supervised_agent import Supervisor, SupervisorError


EVALUATION_1 = 'evaluation_' + '1' * 64
EVALUATION_2 = 'evaluation_' + '2' * 64
EVALUATION_EXTERNAL = 'evaluation_' + 'e' * 64


def request(request_id='request-1', scan_cycle=1, previous=''):
    return {
        'request_id': request_id, 'artifact_id': 'artifact-a',
        'observation_id': 'observation-' + str(scan_cycle),
        'event_type': 'deposition_readback', 'candidate_cycle': scan_cycle - 1,
        'scan_cycle': scan_cycle, 'previous_evaluation_id': previous,
        'frame_id': 'ur20_base_link', 'coordinate_units': 'm',
        'source_manifest_path': '/recorded/manifest.json',
        'source_manifest_sha256': 'a' * 64,
    }


def result(evaluation_id=EVALUATION_1, ok=True, request_id='request-1'):
    return {'ok': ok, 'kind': 'evaluate_observation', 'phase': 'plan',
            'planned_only': True,
            'error': None if ok else 'review failed',
            'metrics': {'evaluation_id': evaluation_id, 'request_id': request_id,
                        'status': 'REVIEW_REQUIRED', 'requires_review': True,
                        'fabrication_authorized': False}}


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'journal.jsonl'
        self.supervisor = Supervisor(self.path)
        self.addCleanup(lambda: self.supervisor.close())

    def proposed(self, value=None):
        return self.supervisor.propose('evaluate_observation', request() if value is None else value)

    def approved(self, value=None):
        self.supervisor.set_armed(True, 'operator-a')
        proposal = self.proposed(value)
        self.supervisor.approve(proposal['request_id'], proposal['proposal_sha256'], 'operator-a')
        return proposal

    def run_step(self, proposal, callback=None):
        return self.supervisor.execute(proposal['request_id'], proposal['proposal_sha256'],
                                       (lambda: result(request_id=proposal['request_id']))
                                       if callback is None else callback)

    def test_initial_disarmed_no_ui_does_not_authorize(self):
        self.assertEqual(self.supervisor.snapshot()['state'], 'DISARMED')
        with self.assertRaises(SupervisorError):
            self.proposed()
        self.supervisor.set_armed(True, 'operator-a')
        proposal = self.proposed()
        with self.assertRaises(SupervisorError):
            self.run_step(proposal)
        self.supervisor.set_armed(False, 'operator-a')
        with self.assertRaises(SupervisorError):
            self.supervisor.approve(proposal['request_id'], proposal['proposal_sha256'], 'operator-a')

    def test_only_review_operation_and_exact_request_are_allowed(self):
        for kind in ('scan', 'home', 'print', 'execute_recovery', 'evaluate_observation;print'):
            with self.assertRaises(SupervisorError):
                self.supervisor.propose(kind, request())
        for change in ({'scan_cycle': True}, {'scan_cycle': 3},
                       {'request_id': '../bad id'}, {'source_manifest_sha256': 'bad'},
                       {'event_type': 'environment_scan'}, {'coordinate_units': 'mm'},
                       {'previous_evaluation_id': 'short-id'},
                       {'source_manifest_path': 'relative/manifest.json'},
                       {'source_manifest_path': '/recorded/../manifest.json'}):
            value = request()
            value.update(change)
            with self.assertRaises(SupervisorError):
                self.proposed(value)
        value = request()
        value['robot_commands'] = ['move']
        with self.assertRaises(SupervisorError):
            self.proposed(value)

    def test_success_cached_before_stale_gate_even_after_restart(self):
        proposal = self.approved()
        calls = []
        self.assertTrue(self.run_step(proposal, lambda: calls.append(1) or result())['ok'])
        self.assertTrue(self.run_step(proposal, lambda: calls.append(2) or result())['ok'])
        self.assertEqual(calls, [1])
        self.supervisor.close()
        self.supervisor = Supervisor(self.path)
        self.assertFalse(self.supervisor.snapshot()['armed'])
        self.assertTrue(self.run_step(proposal, lambda: calls.append(3) or result())['ok'])
        self.assertEqual(self.proposed()['state'], 'COMPLETED')
        self.assertEqual(calls, [1])
        self.assertEqual(self.supervisor.receipt('request-1')['state'], 'COMPLETED')

    def test_identity_digest_and_revision_conflicts_are_rejected(self):
        proposal = self.approved()
        self.run_step(proposal)
        for value in (request('new-id', 1, EVALUATION_1), request('new-id', 2, EVALUATION_2)):
            with self.assertRaisesRegex(SupervisorError, 'stale'):
                self.proposed(value)
        changed = request()
        changed['source_manifest_sha256'] = 'b' * 64
        with self.assertRaisesRegex(SupervisorError, 'digest'):
            self.proposed(changed)
        with self.assertRaisesRegex(SupervisorError, 'digest'):
            self.supervisor.execute('request-1', 'f' * 64, lambda: result())
        next_proposal = self.proposed(request('request-2', 2, EVALUATION_1))
        self.assertEqual(next_proposal['state'], 'PENDING')

    def test_explicit_first_bootstrap_does_not_invent_history(self):
        proposal = self.approved(request('bootstrap', 20, EVALUATION_EXTERNAL))
        self.run_step(proposal)
        self.assertEqual(self.supervisor.snapshot()['heads']['artifact-a'],
                         {'evaluation_id': EVALUATION_1, 'scan_cycle': 20})

    def test_one_pending_step_deep_copies_and_single_use_approval(self):
        self.supervisor.set_armed(True, 'operator-a')
        original = request()
        proposal = self.proposed(original)
        original['frame_id'] = 'mutated'
        proposal['request']['frame_id'] = 'mutated'
        self.assertEqual(self.supervisor.snapshot()['records']['request-1']['request']['frame_id'],
                         'ur20_base_link')
        with self.assertRaises(SupervisorError):
            self.proposed(request('request-2', 2))
        proposal = self.supervisor.snapshot()['records']['request-1']
        self.supervisor.set_armed(True, 'operator-a')
        self.supervisor.approve('request-1', proposal['proposal_sha256'], 'operator-a')
        self.run_step(proposal)
        with self.assertRaises(SupervisorError):
            self.supervisor.approve('request-1', proposal['proposal_sha256'], 'operator-a')
        receipt = self.supervisor.receipt('request-1')
        receipt['result']['metrics']['evaluation_id'] = 'mutated'
        self.assertEqual(self.supervisor.receipt('request-1')['result']['metrics']['evaluation_id'], EVALUATION_1)

    def test_deny_cancel_invalidate_and_disarm_remove_authority(self):
        for transition in ('deny', 'cancel', 'invalidate', 'disarm'):
            with self.subTest(transition=transition):
                proposal = self.approved(request('step-' + transition))
                if transition == 'deny':
                    self.supervisor.cancel(proposal['request_id'], 'prepare explicit denial')
                    self.supervisor.set_armed(True, 'operator-a')
                    proposal = self.proposed(request('denied'))
                    self.supervisor.approve('denied', proposal['proposal_sha256'], 'operator-a', approve=False)
                elif transition == 'cancel':
                    self.supervisor.cancel(proposal['request_id'], 'operator cancelled')
                elif transition == 'invalidate':
                    self.supervisor.invalidate('source changed')
                else:
                    self.supervisor.set_armed(False, 'operator-a')
                with self.assertRaises(SupervisorError):
                    self.run_step(proposal)
                with self.assertRaises(SupervisorError):
                    self.proposed(proposal['request'])
                self.assertFalse(self.supervisor.snapshot()['armed'])

    def test_invalid_and_failed_command_results_do_not_advance_head(self):
        bad = [result(ok=1), {'ok': True}, dict(result(), metrics={'value': float('nan')}),
               dict(result(), fabrication_authorized=True), dict(result(), robot_commands=['move'])]
        for index, value in enumerate(bad + [result(ok=False)]):
            proposal = self.approved(request('bad-' + str(index)))
            returned = self.run_step(proposal, lambda: value)
            self.assertIs(returned['ok'], False)
            self.assertEqual(self.supervisor.snapshot()['heads'], {})
            self.assertFalse(self.supervisor.snapshot()['armed'])
            self.assertEqual(self.run_step(proposal, lambda: self.fail('failure retried')), returned)

    def test_callback_exception_fails_closed(self):
        proposal = self.approved()
        def fail():
            raise RuntimeError('do not treat exception as success')
        returned = self.run_step(proposal, fail)
        self.assertFalse(returned['ok'])
        self.assertEqual(self.supervisor.receipt('request-1')['state'], 'FAILED')
        self.assertFalse(self.supervisor.snapshot()['armed'])

    def test_realistic_command_result_and_strict_success_binding(self):
        proposal = self.approved()
        value = result()
        self.assertNotIn('fabrication_authorized', value)
        self.assertEqual(value['phase'], 'plan')
        self.assertEqual(self.run_step(proposal, lambda: value), value)
        for index, change in enumerate(({'status': 'COMPLETE'}, {'requires_review': False},
                                        {'request_id': 'wrong-request'},
                                        {'evaluation_id': 'evaluation-short'},
                                        {'fabrication_authorized': True})):
            value = result(request_id='bad-success-' + str(index))
            value['metrics'].update(change)
            proposal = self.approved(request('bad-success-' + str(index), 2, EVALUATION_1))
            self.assertFalse(self.run_step(proposal, lambda: value)['ok'])
            self.assertEqual(self.supervisor.snapshot()['heads']['artifact-a']['scan_cycle'], 1)
        value = result(request_id='conflicting-top-auth')
        value['fabrication_authorized'] = True
        proposal = self.approved(request('conflicting-top-auth', 2, EVALUATION_1))
        self.assertFalse(self.run_step(proposal, lambda: value)['ok'])

    def test_parent_directory_is_created_and_final_symlink_denied(self):
        nested = Path(self.temp.name) / 'new' / 'nested' / 'journal.jsonl'
        other = Supervisor(nested)
        self.assertTrue(nested.is_file())
        other.close()
        alias = Path(self.temp.name) / 'journal-alias.jsonl'
        alias.symlink_to(nested)
        with self.assertRaises(OSError):
            Supervisor(alias)

    def test_json_result_is_stable_across_restart_and_operator_is_audit_only(self):
        proposal = self.approved()
        value = result()
        value['metrics']['pairs'] = (1, 2)
        returned = self.run_step(proposal, lambda: value)
        self.assertEqual(returned['metrics']['pairs'], [1, 2])
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual(next(r for r in records if r['event'] == 'ARM')['operator_id'], 'operator-a')
        self.supervisor.close()
        self.supervisor = Supervisor(self.path)
        self.assertEqual(self.run_step(proposal), returned)
        self.assertFalse(self.supervisor.snapshot()['armed'])

    def test_start_is_durable_before_callback_and_write_failure_blocks_it(self):
        proposal = self.approved()
        called = []
        def operation():
            last = json.loads(self.path.read_text().splitlines()[-1])
            self.assertEqual(last['event'], 'START')
            self.assertIsNone(last['data']['records']['request-1']['approval'])
            called.append(1)
            return result()
        self.run_step(proposal, operation)
        self.assertEqual(called, [1])
        proposal = self.approved(request('request-2', 2, EVALUATION_1))
        with patch('behav3d_orchestrator.supervised_agent.supervisor.os.fsync', side_effect=OSError('disk failure')):
            with self.assertRaisesRegex(SupervisorError, 'journal write failed'):
                self.run_step(proposal, lambda: called.append(2) or result(EVALUATION_2, request_id='request-2'))
        self.assertEqual(called, [1])

    def running(self, proposal):
        entered, release = threading.Event(), threading.Event()
        returned = []
        def operation():
            entered.set()
            if not release.wait(5):
                raise RuntimeError('test timeout')
            return result(request_id=proposal['request_id'])
        worker = threading.Thread(target=lambda: returned.append(self.run_step(proposal, operation)))
        worker.start()
        self.assertTrue(entered.wait(5))
        return worker, release, returned

    def test_concurrent_execute_disallowed_and_cancelled_late_success_stays_hold(self):
        for transition in ('cancel', 'invalidate', 'disarm'):
            with self.subTest(transition=transition):
                proposal = self.approved(request('running-' + transition))
                worker, release, returned = self.running(proposal)
                try:
                    with self.assertRaises(SupervisorError):
                        self.run_step(proposal, lambda: self.fail('concurrent execution'))
                    if transition == 'cancel':
                        self.supervisor.cancel(proposal['request_id'], 'cancel during operation')
                    elif transition == 'invalidate':
                        self.supervisor.invalidate('source changed during operation')
                    else:
                        self.supervisor.set_armed(False, 'operator-a')
                    with self.assertRaises(SupervisorError):
                        self.supervisor.set_armed(True, 'operator-a')
                finally:
                    release.set()
                    worker.join(5)
                self.assertFalse(worker.is_alive())
                self.assertFalse(returned[0]['ok'])
                self.assertEqual(self.supervisor.snapshot()['state'], 'HOLD')
                self.assertEqual(self.supervisor.snapshot()['heads'], {})
                self.supervisor.cancel(proposal['request_id'], 'acknowledged uncertain outcome')

    def test_restart_discards_approval_and_holds_inflight_without_retry(self):
        proposal = self.approved()
        self.supervisor.close()
        self.supervisor = Supervisor(self.path)
        self.assertFalse(self.supervisor.snapshot()['armed'])
        self.assertEqual(self.supervisor.snapshot()['state'], 'HOLD')
        with self.assertRaises(SupervisorError):
            self.run_step(proposal)
        self.supervisor.cancel('request-1', 'restart acknowledged')
        proposal = self.approved(request('inflight'))
        worker, release, returned = self.running(proposal)
        crash_image = self.path.read_bytes()
        release.set()
        worker.join(5)
        self.supervisor.close()
        self.path.write_bytes(crash_image)
        self.supervisor = Supervisor(self.path)
        self.assertEqual(self.supervisor.snapshot()['state'], 'HOLD')
        self.assertFalse(self.supervisor.snapshot()['armed'])
        self.assertIsNone(self.supervisor.receipt('inflight'))
        with self.assertRaises(SupervisorError):
            self.run_step(proposal, lambda: self.fail('uncertain operation retried'))

    def test_corruption_truncation_and_competing_owner_fail_closed(self):
        with self.assertRaises(BlockingIOError):
            Supervisor(self.path)
        self.supervisor.close()
        original = self.path.read_bytes()
        for broken in (original[:-1], original.replace(b'DISARMED', b'BROKEN') if b'DISARMED' in original
                       else original.replace(b'BOOT', b'NOPE'), b'{"bad":NaN}\n'):
            self.path.write_bytes(broken)
            with self.assertRaises(SupervisorError):
                Supervisor(self.path)
        self.path.write_bytes(original)
        self.supervisor = Supervisor(self.path)
        self.assertFalse(self.supervisor.snapshot()['armed'])

    def test_journal_hash_chain_and_no_ros_import(self):
        proposal = self.approved()
        self.run_step(proposal)
        previous = '0' * 64
        for sequence, line in enumerate(self.path.read_text().splitlines(), 1):
            entry = json.loads(line)
            claimed = entry.pop('sha256')
            self.assertEqual(entry['sequence'], sequence)
            self.assertEqual(entry['previous_sha256'], previous)
            actual = hashlib.sha256(json.dumps(entry, sort_keys=True, separators=(',', ':'),
                                              ensure_ascii=True, allow_nan=False).encode()).hexdigest()
            self.assertEqual(claimed, actual)
            previous = claimed
        self.assertNotIn('rclpy', sys.modules)


if __name__ == '__main__':
    unittest.main()
