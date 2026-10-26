"""CLI reproduction with real isolated verification of synthetic saved evidence."""
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import supervised_review_demo as demo
import test_observation_worker as fixtures


class ReviewDemoTests(unittest.TestCase):
    def setUp(self):
        self.sources = fixtures.ObservationTests('test_exact_success_contract_and_no_source_mutation')
        self.sources.setUp()
        self.addCleanup(self.sources.doCleanups)
        self.output_temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.output_temp.cleanup)
        self.output = Path(self.output_temp.name).resolve() / 'review-demo'
        self.kwargs = {
            'manifest': self.sources.bundle / 'manifest.json',
            'evidence_root': self.sources.bundle,
            'source_roots': (self.sources.root,), 'output': self.output,
            'worker_python': Path(sys.executable).resolve(),
        }

    def test_default_disarmed_never_dispatches(self):
        report = demo.run_demo(**self.kwargs)
        self.assertTrue(report['disarmed_proposal_rejected'])
        self.assertEqual(report['initial_state'], 'DISARMED')
        self.assertEqual(report['final_state'], 'DISARMED')
        self.assertEqual(report['worker_invocations'], 0)
        self.assertEqual(report['worker_status'], 'NOT_DISPATCHED')
        self.assertFalse((self.output / 'receipt.json').exists())
        self.assertFalse(report['verified'])

    def test_real_isolated_worker_exact_retry_and_no_source_mutation(self):
        before = {p: p.read_bytes() for p in self.sources.root.rglob('*') if p.is_file()}
        report = demo.run_demo(**self.kwargs, offline_demonstration_approval=True)
        self.assertTrue(report['verified'])
        self.assertEqual(report['worker_status'], 'REVIEW_REQUIRED')
        self.assertEqual(report['worker_invocations'], 1)
        self.assertTrue(report['cached_retry_equal'])
        self.assertTrue(report['restart_cached_retry_equal'])
        self.assertEqual(report['restart_state'], 'DISARMED')
        self.assertFalse(report['fresh_analysis_performed'])
        self.assertFalse(report['ros_used'])
        self.assertFalse(report['model_calls_sent'])
        self.assertFalse(report['robot_commands_sent'])
        self.assertFalse(report['fabrication_authorized'])
        self.assertEqual(report['evaluation_id'], self.sources.packet['evaluation_id'])
        self.assertEqual(report['request']['observation_id'], self.sources.packet['scan_job_id'])
        self.assertEqual(report['request']['previous_evaluation_id'], '')
        self.assertEqual(before, {p: p.read_bytes() for p in self.sources.root.rglob('*') if p.is_file()})
        manifest = json.loads((self.output / 'manifest.json').read_text())
        for name, checksum in manifest['files'].items():
            self.assertEqual(hashlib.sha256((self.output / name).read_bytes()).hexdigest(), checksum)
        receipt = json.loads((self.output / 'receipt.json').read_text())
        self.assertEqual(receipt['proposal_sha256'], report['proposal_sha256'])
        self.assertTrue(receipt['result']['planned_only'])

    def test_stale_source_is_not_promoted_to_verified_evidence(self):
        self.sources.source.write_text('changed source', encoding='utf-8')
        report = demo.run_demo(**self.kwargs, offline_demonstration_approval=True)
        self.assertFalse(report['verified'])
        self.assertEqual(report['worker_status'], 'STALE_SOURCE')
        self.assertEqual(report['worker_invocations'], 1)
        self.assertEqual(report['evaluation_id'], '')
        self.assertEqual(report['final_state'], 'DISARMED')

    def test_fresh_output_and_read_only_root_separation(self):
        self.output.mkdir()
        with self.assertRaises(FileExistsError):
            demo.run_demo(**self.kwargs)
        for output in (self.sources.bundle / 'new-output', self.sources.root / 'new-output'):
            with self.assertRaises(ValueError):
                demo.run_demo(**dict(self.kwargs, output=output))
            self.assertFalse(output.exists())

    def test_metadata_tampering_and_relative_paths_fail_before_output(self):
        path = self.sources.bundle / 'robot_review_packet.json'
        path.write_bytes(path.read_bytes() + b' ')
        with self.assertRaises(ValueError):
            demo.run_demo(**self.kwargs)
        self.assertFalse(self.output.exists())
        with self.assertRaises(ValueError):
            demo.run_demo(**dict(self.kwargs, output=Path('relative')))

    def test_default_cli_summary_explicitly_says_not_dispatched(self):
        capture = io.StringIO()
        with redirect_stdout(capture):
            status = demo.main([
                '--manifest', str(self.kwargs['manifest']), '--evidence-root', str(self.sources.bundle),
                '--source-root', str(self.sources.root), '--output', str(self.output),
            ])
        summary = json.loads(capture.getvalue())
        self.assertEqual(status, 0)
        self.assertEqual(summary['worker_status'], 'NOT_DISPATCHED')
        self.assertFalse(summary['fabrication_authorized'])


if __name__ == '__main__':
    unittest.main()
