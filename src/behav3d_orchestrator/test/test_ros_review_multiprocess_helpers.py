"""Stdlib checks for the separate-process ROS validation harness helpers."""

import importlib.util
import json
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "ros_review_multiprocess", REPO / "scripts/ros_review_multiprocess.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MultiprocessHelpersTests(unittest.TestCase):
    def test_parameter_paths_are_single_quoted_argv_without_shell(self):
        arguments = MODULE.parameter_arguments(
            {
                "session_path": "/tmp/a space/session",
                "source_roots": ["/tmp/a", "/tmp/b"],
                "observation_timeout_s": 4.0,
            }
        )
        self.assertEqual(arguments[0], "--ros-args")
        self.assertIn('session_path:="/tmp/a space/session"', arguments)
        self.assertIn('source_roots:=["/tmp/a", "/tmp/b"]', arguments)
        for invalid in ("x y", "x;touch", ""):
            with self.assertRaises(ValueError):
                MODULE.parameter_arguments({invalid: "value"})

    def test_only_two_review_console_executables_are_allowed(self):
        for executable in ("print_move", "/bin/sh", "scan_sequence"):
            with self.assertRaises(ValueError):
                MODULE.executable_path(executable)

    def test_read_journal_ignores_only_partial_tail_and_counts_starts(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "journal.jsonl"
            self.assertEqual(MODULE.read_journal(path), [])
            entry = {"event": "START", "data": {"inflight_id": "request-a"}}
            path.write_bytes(json.dumps(entry).encode() + b'\n{"event":')
            entries = MODULE.read_journal(path)
            self.assertEqual(len(entries), 1)
            self.assertEqual(MODULE.start_count(entries), 1)
            self.assertEqual(MODULE.start_count(entries, "request-a"), 1)
            self.assertEqual(MODULE.start_count(entries, "request-b"), 0)

    def test_complete_corrupt_journal_line_is_not_silently_skipped(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "journal.jsonl"
            path.write_text("{bad}\n")
            with self.assertRaises(ValueError):
                MODULE.read_journal(path)

    @unittest.skipUnless(hasattr(signal, "SIGKILL"), "POSIX process groups required")
    def test_process_group_is_separate_logs_are_saved_and_sigkill_is_reaped(self):
        # Popen's actual executable is replaced only in this stdlib fixture.
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(MODULE, "executable_path", return_value=Path(sys.executable)),
            patch.object(
                MODULE,
                "parameter_arguments",
                return_value=[
                    "-u",
                    "-c",
                    'import time; print("ready"); time.sleep(30)',
                ],
            ),
        ):
            process = MODULE.RuntimeProcess(
                "test-child", "observation_bridge", {}, temporary, timeout_s=10
            ).start()
            try:
                self.assertNotEqual(process.pid, __import__("os").getpid())
                MODULE.eventually(lambda: "ready" in process.text(), timeout=3)
                process.healthy()
            finally:
                process.stop(kill=True)
            self.assertEqual(process.report()["returncode"], -signal.SIGKILL)
            self.assertTrue(process.report()["exit_matches_expectation"])
            self.assertFalse(process.report()["forced_kill"])
            self.assertTrue(process.report()["reader_threads_stopped"])
            self.assertIn(
                "ready", (Path(temporary) / "test-child.stdout.log").read_text()
            )

    @unittest.skipUnless(hasattr(signal, "SIGKILL"), "POSIX process groups required")
    def test_bounded_capture_still_drains_child_output(self):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(MODULE, "executable_path", return_value=Path(sys.executable)),
            patch.object(
                MODULE,
                "parameter_arguments",
                return_value=["-u", "-c", 'import sys; sys.stdout.write("x" * 300000)'],
            ),
        ):
            process = MODULE.RuntimeProcess(
                "test-overflow", "observation_bridge", {}, temporary, timeout_s=5
            ).start()
            try:
                process.process.wait(timeout=3)
            finally:
                process.stop()
            self.assertEqual(len(process.text()), MODULE.LOG_LIMIT)
            self.assertTrue(process.report()["logs_truncated"])
            self.assertEqual(process.report()["log_bytes_received"]["stdout"], 300000)
            self.assertTrue(process.report()["exit_matches_expectation"])

    @unittest.skipUnless(hasattr(signal, "SIGKILL"), "POSIX process groups required")
    def test_runtime_watchdog_kills_without_a_ros_client(self):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(MODULE, "executable_path", return_value=Path(sys.executable)),
            patch.object(
                MODULE,
                "parameter_arguments",
                return_value=["-c", "import time; time.sleep(30)"],
            ),
        ):
            process = MODULE.RuntimeProcess(
                "test-deadline", "observation_bridge", {}, temporary, timeout_s=0.15
            ).start()
            process.process.wait(timeout=3)
            process.stop()
            self.assertTrue(process.report()["deadline_expired"])
            with self.assertRaises(TimeoutError):
                process.healthy()

    @unittest.skipUnless(hasattr(signal, "SIGKILL"), "POSIX process groups required")
    def test_graceful_sigterm_requires_exit_zero_and_no_traceback(self):
        script = (
            "import signal, sys, time; "
            "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); "
            'print("ready", flush=True); time.sleep(30)'
        )
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(MODULE, "executable_path", return_value=Path(sys.executable)),
            patch.object(
                MODULE, "parameter_arguments", return_value=["-u", "-c", script]
            ),
        ):
            process = MODULE.RuntimeProcess(
                "test-graceful", "observation_bridge", {}, temporary, timeout_s=5
            ).start()
            try:
                MODULE.eventually(lambda: "ready" in process.text(), timeout=3)
            finally:
                process.stop()
            self.assertEqual(process.report()["returncode"], 0)
            self.assertTrue(process.report()["exit_matches_expectation"])
            self.assertFalse(process.report()["traceback_detected"])
            self.assertFalse(process.report()["forced_kill"])
            self.assertIsInstance(process.report()["stop_duration_s"], float)

    @unittest.skipUnless(hasattr(signal, "SIGKILL"), "POSIX process groups required")
    def test_sigterm_wait_uses_ros_drain_grace_and_forced_kill_stays_failure(self):
        from types import SimpleNamespace

        process = MODULE.RuntimeProcess("test-stuck", "observation_bridge", {}, "/tmp")
        # No process is launched: emulate the documented 12 s grace expiring.
        process.process = SimpleNamespace(
            pid=123456,
            wait=Mock(
                side_effect=[
                    MODULE.subprocess.TimeoutExpired("review", 12),
                    -signal.SIGKILL,
                ]
            ),
            poll=lambda: -signal.SIGKILL,
        )
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(MODULE.os, "killpg"),
        ):
            process.output = Path(temporary)
            process.stop()
        self.assertEqual(process.process.wait.call_args_list[0].kwargs["timeout"], 12.0)
        self.assertTrue(process.report()["forced_kill"])
        self.assertFalse(process.report()["exit_matches_expectation"])

    def test_no_runtime_node_classes_are_imported_into_client_script(self):
        source = (REPO / "scripts/ros_review_multiprocess.py").read_text()
        self.assertNotIn("import ObservationBridge", source)
        self.assertNotIn("import SupervisedAgentLoop", source)
        self.assertNotIn("coordinator._supervisor", source)
        self.assertNotIn("shell=True", source)


if __name__ == "__main__":
    unittest.main()
