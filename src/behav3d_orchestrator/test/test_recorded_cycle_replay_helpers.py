"""Pure stdlib checks for recorded ROS replay preparation, no ROS or hardware."""
import json
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_recorded_cycle_replay as replay
import test_observation_worker as fixtures


class RecordedCycleReplayHelperTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.manifests, self.fixtures = [], []
        previous = None
        for scan in (5, 6, 7):
            fixture = fixtures.ObservationTests('test_exact_success_contract_and_no_source_mutation')
            fixture.setUp()
            self.addCleanup(fixture.doCleanups)
            fixture.bundle = self.root / f'cycle-{scan}'
            fixture.bundle.mkdir()
            fixture.evaluation['scan_cycle'] = scan
            fixture.evaluation['candidate_cycle'] = scan - 1
            fixture.evaluation['scan_job_id'] = f'scan-{scan}'
            fixture.evaluation['previous_evaluation_id'] = previous
            fixture.publish()
            previous = fixture.evaluation['evaluation_id']
            self.fixtures.append(fixture)
            self.manifests.append(fixture.bundle / 'manifest.json')

    def test_consecutive_metadata_retains_exact_recorded_lineage_and_paths(self):
        before = {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        sequence = replay.load_recorded_sequence(self.manifests, self.root)
        self.assertEqual([item['request']['scan_cycle'] for item in sequence], [5, 6, 7])
        self.assertEqual([item['request']['candidate_cycle'] for item in sequence], [4, 5, 6])
        for index, item in enumerate(sequence):
            self.assertEqual(item['evaluation_id'], self.fixtures[index].packet['evaluation_id'])
            self.assertEqual(item['request']['source_manifest_path'], str(self.manifests[index]))
            self.assertTrue(item['request']['request_id'].startswith('recorded-ros-'))
        self.assertEqual(sequence[1]['request']['previous_evaluation_id'], sequence[0]['evaluation_id'])
        self.assertEqual(sequence[2]['request']['previous_evaluation_id'], sequence[1]['evaluation_id'])
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()})
        self.assertEqual(len(replay.bundle_hashes(sequence)), 12)

    def test_order_gaps_duplicate_and_single_manifest_fail_closed(self):
        for manifests in (self.manifests[::-1], self.manifests[::2],
                          [self.manifests[0], self.manifests[0]], self.manifests[:1]):
            with self.subTest(manifests=manifests), self.assertRaises(ValueError):
                replay.load_recorded_sequence(manifests, self.root)

    def test_wrong_predecessor_artifact_or_frame_rejected(self):
        changes = ({'previous_evaluation_id': 'evaluation_' + 'f' * 64},
                   {'artifact_id': 'another-artifact'},)
        for change in changes:
            fixture = self.fixtures[1]
            original = deepcopy(fixture.evaluation)
            fixture.evaluation.update(change)
            fixture.publish()
            with self.assertRaises(ValueError):
                replay.load_recorded_sequence(self.manifests, self.root)
            fixture.evaluation = original
            fixture.publish()
        self.fixtures[1].evaluation['context']['frame_id'] = 'wrong-frame'
        self.fixtures[1].publish()
        with self.assertRaises(ValueError):
            replay.load_recorded_sequence(self.manifests, self.root)

    def test_unconfined_bundle_relative_path_and_tampered_packet_rejected(self):
        for manifests, root in ((self.manifests, self.fixtures[0].root),
                                ([Path('relative/manifest.json'), self.manifests[1]], self.root)):
            with self.subTest(root=root), self.assertRaises((ValueError, FileNotFoundError)):
                replay.load_recorded_sequence(manifests, root)
        path = self.manifests[0].parent / 'robot_review_packet.json'
        path.write_bytes(path.read_bytes() + b' ')
        with self.assertRaises(ValueError):
            replay.load_recorded_sequence(self.manifests, self.root)

    def test_unexpected_manifest_child_is_never_followed(self):
        path = self.manifests[0]
        value = json.loads(path.read_text())
        value['files']['../../private-source.json'] = '0' * 64
        path.write_text(json.dumps(value), encoding='utf-8')
        with self.assertRaises(ValueError):
            replay.load_recorded_sequence(self.manifests, self.root)

    def test_exact_result_binding_and_safety_flags(self):
        from behav3d_orchestrator.supervised_agent.observation import RESULT_FIELDS
        item = replay.load_recorded_sequence(self.manifests, self.root)[0]
        request = item['request']
        value = {name: '' for name in RESULT_FIELDS}
        value.update(success=True, status='REVIEW_REQUIRED', requires_review=True,
                     fabrication_authorized=False, request_id=request['request_id'],
                     evaluation_id=item['evaluation_id'], packet_path=item['packet_path'],
                     packet_sha256=item['packet_sha256'],
                     evidence_manifest_path=request['source_manifest_path'],
                     evidence_manifest_sha256=request['source_manifest_sha256'])
        self.assertEqual(replay.check_saved_result(value, item), value)
        for key, changed in (('fabrication_authorized', True), ('requires_review', False),
                             ('evaluation_id', 'evaluation_' + 'f' * 64),
                             ('packet_sha256', '0' * 64), ('request_id', 'wrong-id'),
                             ('evidence_manifest_sha256', '0' * 64)):
            with self.subTest(key=key), self.assertRaises(AssertionError):
                replay.check_saved_result(dict(value, **{key: changed}), item)

    def test_programmatic_approval_is_explicit_and_default_never_launches_ros(self):
        with patch.object(replay, 'require_isolation') as isolation:
            with self.assertRaises(ValueError):
                replay.run(manifests=self.manifests, evidence_root=self.root,
                           source_roots=[], output=self.root / 'should-not-exist')
            isolation.assert_not_called()
            self.assertFalse((self.root / 'should-not-exist').exists())

    def test_replay_requires_clean_child_exit_and_bounded_drained_logs(self):
        record = {'returncode': 0, 'logs_truncated': False,
                  'deadline_expired': False, 'reader_threads_stopped': True}
        self.assertTrue(replay.process_exit_ok(record))
        for key, value in (('returncode', 1), ('returncode', -9), ('returncode', None),
                           ('logs_truncated', True), ('deadline_expired', True),
                           ('reader_threads_stopped', False)):
            with self.subTest(key=key, value=value):
                self.assertFalse(replay.process_exit_ok(dict(record, **{key: value})))

    def test_output_cannot_overlap_readonly_inputs_or_existing_directory(self):
        kwargs = {'manifests': self.manifests, 'evidence_root': self.root, 'source_roots': [],
                  'offline_demonstration_approval': True}
        with patch.object(replay, 'require_isolation'):
            with self.assertRaises(ValueError):
                replay.run(**kwargs, output=self.root / 'inside-evidence')
            with self.assertRaises(ValueError):
                replay.run(**kwargs, output=Path('relative'))
            with tempfile.TemporaryDirectory() as output, self.assertRaises(FileExistsError):
                replay.run(**kwargs, output=Path(output).resolve())


if __name__ == '__main__':
    unittest.main()
