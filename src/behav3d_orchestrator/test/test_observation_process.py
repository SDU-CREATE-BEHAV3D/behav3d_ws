"""Adversarial POSIX worker tests; stdlib only, no ROS/model/hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.supervised_agent import process_runner
from behav3d_orchestrator.supervised_agent.observation import result_for_error
from behav3d_orchestrator.supervised_agent.process_runner import (
    WorkerError, run_observation_worker,
)


@unittest.skipUnless(os.name == "posix", "POSIX process-group isolation")
class ObservationProcessTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.script = self.root / "worker.py"
        self.request = {
            "request_id": "process-test", "artifact_id": "artifact-a",
            "observation_id": "scan-2", "event_type": "deposition_readback",
            "candidate_cycle": 1, "scan_cycle": 2, "previous_evaluation_id": "",
            "frame_id": "base", "coordinate_units": "m",
            "source_manifest_path": str(self.root / "manifest.json"),
            "source_manifest_sha256": "0" * 64,
        }
        self.result = result_for_error("process-test", "REJECTED", "Recorded test response.")

    def program(self, code):
        self.script.write_text(code, encoding="utf-8")

    def result_program(self, suffix=""):
        self.program("import json,sys\nrequest=json.load(sys.stdin)\n"
                     + "print(" + repr(json.dumps(self.result)) + ",flush=True)\n" + suffix)

    def run_worker(self, **kwargs):
        values = dict(python_executable=sys.executable, evidence_root=self.root,
                      timeout_s=2.0, worker_script=self.script)
        values.update(kwargs)
        return run_observation_worker(self.request, **values)

    def assert_worker_error(self, status, **kwargs):
        start = time.monotonic()
        with self.assertRaises(WorkerError) as caught:
            self.run_worker(**kwargs)
        self.assertEqual(caught.exception.status, status)
        self.assertLess(time.monotonic() - start, 2.5)

    def test_valid_result_and_request_eof(self):
        self.result_program()
        self.assertEqual(self.run_worker(), self.result)

    def test_fresh_policy_is_explicit_fixed_and_not_a_request_command(self):
        for changes in ({"backend_mode": "arbitrary"}, {"agent007_root": str(self.root)},
                        {"backend_mode": "fresh_geometry"},
                        {"backend_mode": "fresh_geometry", "agent007_root": str(self.root),
                         "dds_root": str(self.root), "output_root": str(self.root)}):
            with self.subTest(changes=changes), patch.object(process_runner.subprocess, 'Popen') as start:
                with self.assertRaises(ValueError):
                    self.run_worker(**changes)
                start.assert_not_called()

    def test_fixed_fresh_worker_receives_only_configured_roots(self):
        with patch.object(process_runner.subprocess, 'Popen', side_effect=OSError('not launched')) as start:
            with self.assertRaises(WorkerError):
                run_observation_worker(self.request, python_executable=sys.executable,
                    evidence_root=self.root, backend_mode='fresh_geometry',
                    agent007_root=self.root, dds_root=self.root, output_root=self.root)
        argv = start.call_args.args[0]
        self.assertEqual(Path(argv[2]).name, 'fresh_observation_worker.py')
        for option in ('--agent007-root', '--dds-root', '--output-root'):
            self.assertEqual(argv[argv.index(option) + 1], str(self.root))

    def test_oversized_stdout_and_stderr_fail_bounded(self):
        for target, count in (("stdout", 20000), ("stderr", 70000)):
            with self.subTest(target=target):
                self.program("import sys,time\n"
                             + "sys." + target + ".write('x'*" + str(count) + ")\n"
                             + "sys." + target + ".flush()\ntime.sleep(60)\n")
                self.assert_worker_error("FAILED")

    def test_duplicate_json_keys_rejected(self):
        encoded = json.dumps(self.result)
        encoded = '{"success":true,' + encoded[1:]
        self.program("import sys\nsys.stdin.read()\nprint(" + repr(encoded) + ")\n")
        self.assert_worker_error("FAILED")

    def test_multiple_json_documents_and_nonfinite_values_rejected(self):
        for encoded in (json.dumps(self.result) + "\n{}", json.dumps({**self.result, "message": float("nan")})):
            with self.subTest(encoded=encoded[:32]):
                self.program("import sys\nsys.stdin.read()\nprint(" + repr(encoded) + ")\n")
                self.assert_worker_error("FAILED")

    def test_result_cannot_grant_authority_or_change_request(self):
        for change in ({"fabrication_authorized": True}, {"request_id": "other"},
                       {"success": True, "status": "PASS"}, {"robot_commands": []}):
            with self.subTest(change=change):
                encoded = json.dumps({**self.result, **change})
                self.program("import sys\nsys.stdin.read()\nprint(" + repr(encoded) + ")\n")
                self.assert_worker_error("FAILED")

    def test_nonzero_exit_is_not_a_result(self):
        self.result_program("raise SystemExit(3)\n")
        self.assert_worker_error("FAILED")

    def test_no_input_reader_and_ignored_term_have_bounded_timeout(self):
        self.program("import signal,time\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\ntime.sleep(60)\n")
        self.assert_worker_error("TIMED_OUT", timeout_s=0.15)

    def test_saturated_stdin_does_not_block_deadline(self):
        self.program("import time\ntime.sleep(60)\n")
        original = subprocess.Popen

        def saturated(*args, **kwargs):
            process = original(*args, **kwargs)
            os.set_blocking(process.stdin.fileno(), False)
            while True:
                try:
                    os.write(process.stdin.fileno(), b"x" * 4096)
                except BlockingIOError:
                    break
            return process

        with patch.object(process_runner.subprocess, "Popen", side_effect=saturated):
            self.assert_worker_error("TIMED_OUT", timeout_s=0.15)

    def test_cancel_before_dispatch_never_starts_worker(self):
        self.result_program()
        with patch.object(process_runner.subprocess, "Popen") as start:
            self.assert_worker_error("CANCELLED", cancelled=lambda: True)
            start.assert_not_called()

    def test_cancel_while_worker_waits(self):
        self.program("import time\ntime.sleep(60)\n")
        start = time.monotonic()
        self.assert_worker_error("CANCELLED", cancelled=lambda: time.monotonic() - start > 0.08)

    def test_cancel_during_result_validation_discards_receipt(self):
        self.result_program()
        event = threading.Event()
        original = process_runner.validate_result

        def validate(value, request_id):
            result = original(value, request_id)
            event.set()
            return result

        with patch.object(process_runner, "validate_result", side_effect=validate):
            self.assert_worker_error("CANCELLED", cancelled=event.is_set)

    def test_deadline_during_result_validation_discards_receipt(self):
        self.result_program()
        original = process_runner.validate_result

        def validate(value, request_id):
            result = original(value, request_id)
            time.sleep(0.12)
            return result

        with patch.object(process_runner, "validate_result", side_effect=validate):
            self.assert_worker_error("TIMED_OUT", timeout_s=0.1)

    def test_leader_exit_with_child_holding_pipes_is_killed_bounded(self):
        heartbeat = self.root / "heartbeat"
        child_pid = self.root / "child.pid"
        code = (
            "import json,os,signal,sys,time\n"
            "sys.stdin.read()\n"
            "child=os.fork()\n"
            "if child==0:\n"
            " signal.signal(signal.SIGTERM,signal.SIG_IGN)\n"
            " counter=0\n"
            " while True:\n"
            "  counter+=1\n"
            "  with open(" + repr(str(heartbeat)) + ",'w') as stream: stream.write(str(counter))\n"
            "  time.sleep(.01)\n"
            "with open(" + repr(str(child_pid)) + ",'w') as stream:stream.write(str(child))\n"
            "print(" + repr(json.dumps(self.result)) + ",flush=True)\n"
            "os._exit(0)\n")
        self.program(code)
        try:
            self.assert_worker_error("TIMED_OUT", timeout_s=0.2)
            self.assertTrue(heartbeat.exists())
            before = heartbeat.read_text()
            time.sleep(0.1)
            self.assertEqual(heartbeat.read_text(), before)
        finally:
            if child_pid.exists():
                try:
                    os.kill(int(child_pid.read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_descendant_without_output_still_invalidates_result(self):
        child_pid = self.root / "child.pid"
        self.program(
            "import os,sys,time\nsys.stdin.read()\nchild=os.fork()\n"
            "if child==0:\n os.close(1)\n os.close(2)\n time.sleep(60)\n os._exit(0)\n"
            "with open(" + repr(str(child_pid)) + ",'w') as stream:stream.write(str(child))\n"
            "print(" + repr(json.dumps(self.result)) + ",flush=True)\nos._exit(0)\n")
        try:
            self.assert_worker_error("FAILED")
        finally:
            if child_pid.exists():
                try:
                    os.kill(int(child_pid.read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_server_config_and_request_options_do_not_leak_to_worker(self):
        self.result_program()
        with self.assertRaises(ValueError):
            self.run_worker(source_roots="/")
        with self.assertRaises(ValueError):
            self.run_worker(timeout_s=True)
        with self.assertRaises(ValueError):
            run_observation_worker({**self.request, "worker_script": str(self.script)},
                                   python_executable=sys.executable, evidence_root=self.root,
                                   worker_script=self.script)


    def test_selector_setup_failure_never_spawns_worker(self):
        self.result_program()
        with patch.object(process_runner.selectors, "DefaultSelector", side_effect=OSError), \
                patch.object(process_runner.subprocess, "Popen") as start:
            self.assert_worker_error("FAILED")
            start.assert_not_called()

    def test_spawn_failure_closes_preallocated_selector(self):
        self.result_program()
        selector = Mock()
        with patch.object(process_runner.selectors, "DefaultSelector", return_value=selector), \
                patch.object(process_runner.subprocess, "Popen", side_effect=PermissionError):
            self.assert_worker_error("FAILED")
        selector.close.assert_called_once_with()

    def test_writer_thread_start_failure_cleans_worker_without_joining_unstarted_thread(self):
        self.result_program()
        with patch.object(process_runner.threading.Thread, "start", side_effect=RuntimeError):
            self.assert_worker_error("FAILED")


if __name__ == "__main__":
    unittest.main()
