"""Bounded fixed startup subprocess boundary; no scientific or ROS imports."""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from behav3d_orchestrator.supervised_agent import initial_process as runner
from behav3d_orchestrator.supervised_agent.process_runner import WorkerError
from test_initial_workflow import request, success


@unittest.skipUnless(os.name == "posix", "POSIX process groups required")
class InitialProcessTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.policy = {
            "python_executable": sys.executable,
            "input_roots": [self.root],
            "output_root": self.root,
            "agent007_root": self.root,
            "behav3d_root": self.root,
            "journal_path": self.root / "journal.jsonl",
            "timeout_s": 1.0,
        }

    def run_program(self, code, **changes):
        original = subprocess.Popen

        def start(argv, **kwargs):
            self.assertEqual(Path(argv[2]).name, "initial_worker.py")
            self.assertNotIn("--arbitrary-command", argv)
            return original([sys.executable, "-B", "-c", code], **kwargs)

        with patch.object(runner.subprocess, "Popen", start):
            return runner.run_initial_worker(request(), **dict(self.policy, **changes))

    def test_valid_exact_result_and_fixed_worker_argv(self):
        result = success(request())
        code = (
            "import sys\nsys.stdin.read()\nprint("
            + repr(json.dumps(result))
            + ",flush=True)\n"
        )
        self.assertEqual(self.run_program(code), result)

    def test_cancellation_before_spawn(self):
        with patch.object(runner.subprocess, "Popen") as start:
            with self.assertRaises(WorkerError) as caught:
                runner.run_initial_worker(
                    request(), **self.policy, cancelled=lambda: True
                )
            self.assertEqual(caught.exception.status, "CANCELLED")
            start.assert_not_called()

    def test_timeout_kills_ignoring_worker_with_bounded_cleanup(self):
        begin = time.monotonic()
        with self.assertRaises(WorkerError) as caught:
            self.run_program(
                "import signal,time\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\ntime.sleep(30)",
                timeout_s=0.1,
            )
        self.assertEqual(caught.exception.status, "TIMED_OUT")
        self.assertLess(time.monotonic() - begin, 2)

    def test_stdout_stderr_floods_are_rejected(self):
        for target, size in (("stdout", 17000), ("stderr", 66000)):
            with self.subTest(target=target), self.assertRaises(WorkerError):
                self.run_program(
                    "import sys\nsys.stdin.read()\nsys."
                    + target
                    + ".write('x'*"
                    + str(size)
                    + ")\nsys."
                    + target
                    + ".flush()"
                )

    def test_duplicate_json_or_authority_result_rejected(self):
        for raw in (
            '{"success":true,"success":false}',
            json.dumps(dict(success(request()), fabrication_authorized=True)),
        ):
            with self.subTest(raw=raw[:30]), self.assertRaises(WorkerError):
                self.run_program(
                    "import sys\nsys.stdin.read()\nprint(" + repr(raw) + ")"
                )

    def test_invalid_policy_and_request_never_spawn(self):
        with patch.object(runner.subprocess, "Popen") as start:
            for changes in (
                {"timeout_s": True},
                {"timeout_s": float("nan")},
                {"python_executable": "python"},
                {"input_roots": []},
                {"journal_path": "relative"},
            ):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    runner.run_initial_worker(request(), **dict(self.policy, **changes))
            with self.assertRaises(ValueError):
                runner.run_initial_worker(
                    dict(request(), command="print"), **self.policy
                )
            start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
