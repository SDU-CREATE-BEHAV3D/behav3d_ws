"""Isolated synthetic startup ROS validation; never approve a recorded scene.

The fixture is explicitly synthetic. All approvals are labelled software-test
operator decisions, never human approval or authorization to move/extrude.
"""

# ruff: noqa: BLE001 -- validation teardown must retain a failure report
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from copy import deepcopy
from pathlib import Path

from ros_review_multiprocess import (
    LOG_LIMIT,
    RuntimeProcess,
    check,
    eventually,
    future_value,
    parameter_arguments,
    read_journal,
    require_isolation,
)


def verify_preview(batch, planning, request):
    check(
        batch["selected"] == planning["selected"]
        and batch["frame_id"] == request["frame_id"],
        "Native generation changed approved placement/frame",
    )
    count, requested = batch["candidate_count"], batch["requested_count"]
    check(
        type(count) is int and type(requested) is int and 0 < count <= requested,
        "Native batch count invalid; missing targets must not be invented",
    )
    check(
        batch["partial_batch"] is (count < requested),
        "Partial-batch flag disagrees with actual count",
    )
    check(
        batch["requires_review"] is True and batch["fabrication_authorized"] is False,
        "Native batch leaked fabrication authority",
    )
    for key in ("scene_sha256", "geometry_sha256", "task_sha256", "proposal_id"):
        check(batch[key] == planning[key], "Native batch changed source/task identity")


def prepare_fixture(args):
    """Scientific child only; no ROS client, approvals or target generation."""
    sys.path[:0] = [
        str(args.agent007_root / "src"),
        str(args.agent007_root),
        str(args.behav3d_root / "src/behav3d_orchestrator"),
    ]
    location = args.behav3d_root / "tools/startup_validation/fixtures.py"
    spec = importlib.util.spec_from_file_location(
        "startup_validation_fixture", location
    )
    fixture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture)
    from behav3d_orchestrator.supervised_agent.fresh_observation_worker import (
        _analysis_stdout_to_stderr,
    )
    from ocsr.initial_scene import export_scene, file_digest

    source, scene, tasks = (args.output / name for name in ("source", "scene", "tasks"))
    with _analysis_stdout_to_stderr():
        fixture.create_source_scene(source, frame_id="startup_test_world")
        export_scene(source, scene, frame_id="startup_test_world")
    tasks.mkdir()
    task = tasks / "task.json"
    task.write_text(json.dumps(fixture.task_example(), sort_keys=True, indent=2) + "\n")
    request = {
        "phase": "PROPOSE",
        "request_id": "synthetic-startup-first",
        "artifact_id": "synthetic-startup-wall",
        "frame_id": "startup_test_world",
        "scene_manifest_path": str(scene / "manifest.json"),
        "scene_manifest_sha256": file_digest(scene / "manifest.json"),
        "task_path": str(task),
        "task_sha256": file_digest(task),
        "proposal_path": "",
        "proposal_sha256": "",
    }
    (args.output / "prepared.json").write_text(
        json.dumps(request, sort_keys=True, indent=2) + "\n"
    )
    return request


class StartupProcess(RuntimeProcess):
    """Reuse bounded logs/cleanup, resolving only the two startup executables."""

    def start(self):
        from ament_index_python.packages import get_package_prefix

        check(
            self.executable in {"initial_startup", "initial_wall_backend"},
            "Unexpected startup executable",
        )
        check(self.process is None, "RuntimeProcess instances are single-use")
        executable = (
            Path(get_package_prefix("behav3d_orchestrator"))
            / "lib/behav3d_orchestrator"
            / self.executable
        )
        check(
            executable.is_file() and os.access(executable, os.X_OK),
            "Installed startup executable missing",
        )
        self.output.mkdir(parents=True, exist_ok=True)
        self.process = subprocess.Popen(
            [str(executable), *parameter_arguments(self.parameters)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            start_new_session=True,
            env=dict(
                os.environ,
                PYTHONDONTWRITEBYTECODE="1",
                PYTHONUNBUFFERED="1",
                RCUTILS_LOGGING_USE_STDOUT="1",
            ),
        )
        self._started = time.monotonic()

        def drain(stream):
            pipe = getattr(self.process, stream)
            try:
                while True:
                    chunk = os.read(pipe.fileno(), 4096)
                    if not chunk:
                        break
                    self._totals[stream] += len(chunk)
                    remaining = LOG_LIMIT - len(self._buffers[stream])
                    self._buffers[stream].extend(chunk[: max(remaining, 0)])
                    if self._totals[stream] > LOG_LIMIT:
                        self._overflow.set()
            finally:
                pipe.close()

        for stream in ("stdout", "stderr"):
            thread = threading.Thread(target=drain, args=(stream,), daemon=True)
            thread.start()
            self._threads.append(thread)

        def expire():
            self._expired.set()
            self.stop(kill=True)

        self._timer = threading.Timer(self.timeout_s, expire)
        self._timer.daemon = True
        self._timer.start()
        return self


class StartupClient:
    def __enter__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String

        from behav3d_interfaces.action import InitialWall
        from behav3d_interfaces.srv import ApproveSupervisedStep, SetSupervisorArmed

        check(not rclpy.ok(), "Startup client owns its ROS context lifecycle")
        rclpy.init(args=[])
        self.node = rclpy.create_node("synthetic_initial_startup_validation_client")
        self.action = InitialWall
        self.review = ActionClient(
            self.node, InitialWall, "/behav3d/startup/review_wall"
        )
        self.backend = ActionClient(
            self.node, InitialWall, "/behav3d/startup/plan_wall"
        )
        self.arm_client = self.node.create_client(
            SetSupervisorArmed, "/behav3d/startup/set_armed"
        )
        self.approval_client = self.node.create_client(
            ApproveSupervisedStep, "/behav3d/startup/approve_placement"
        )
        self.status_value, self.errors, self.last_result = None, [], None
        self.lock = threading.Lock()
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.node.create_subscription(
            String, "/behav3d/startup/status", self._status, qos
        )
        self.control = self.node.create_publisher(String, "/behav3d/control_state", qos)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)

        def spin():
            try:
                self.executor.spin()
            except Exception as error:
                self.errors.append(type(error).__name__)

        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()
        return self

    def _status(self, message):
        with self.lock:
            self.status_value = json.loads(message.data)

    def status(self):
        with self.lock:
            return deepcopy(self.status_value)

    def ready(self):
        for client in (self.review, self.backend):
            check(
                client.wait_for_server(timeout_sec=10),
                "Startup action discovery failed",
            )
        for client in (self.arm_client, self.approval_client):
            check(
                client.wait_for_service(timeout_sec=10),
                "Startup service discovery failed",
            )
        eventually(
            lambda: self.status() is not None,
            timeout=10,
            description="fresh startup status",
        )

    def send(self, request, backend=False):
        goal = self.action.Goal()
        for name, value in request.items():
            setattr(goal, name, value)
        return future_value(
            (self.backend if backend else self.review).send_goal_async(goal), 10
        )

    def arm(self, armed=True):
        from behav3d_interfaces.srv import SetSupervisorArmed

        value = SetSupervisorArmed.Request()
        value.armed, value.operator_id = (
            armed,
            "synthetic-fixture-software-test-operator-not-human",
        )
        return future_value(self.arm_client.call_async(value), 10)

    def approve(self, request_id, checksum):
        from behav3d_interfaces.srv import ApproveSupervisedStep

        value = ApproveSupervisedStep.Request()
        value.request_id, value.proposal_sha256 = request_id, checksum
        value.approve, value.operator_id = (
            True,
            "synthetic-fixture-software-test-operator-not-human",
        )
        return future_value(self.approval_client.call_async(value), 10)

    def finish(self, handle, request, status, timeout):
        from action_msgs.msg import GoalStatus
        from behav3d_orchestrator.supervised_agent.initial_workflow import (
            RESULT_FIELDS,
            validate_result,
        )

        wrapped = future_value(handle.get_result_async(), timeout)
        values = validate_result(
            {name: getattr(wrapped.result, name) for name in RESULT_FIELDS}, request
        )
        self.last_result = deepcopy(values)
        check(
            values["status"] == status,
            "Unexpected startup typed status: "
            + values["status"]
            + ": "
            + values["message"],
        )
        if values["success"]:
            check(
                wrapped.status == GoalStatus.STATUS_SUCCEEDED,
                "Successful startup action did not succeed",
            )
        else:
            check(
                wrapped.status
                in {GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED},
                "Failed startup action invalid terminal state",
            )
        return values

    def pause(self, state):
        from std_msgs.msg import String

        eventually(
            lambda: self.control.get_subscription_count() > 0,
            timeout=5,
            description="shared pause subscriber",
        )
        self.control.publish(String(data=state))
        # A following reliable message is not an acknowledgement. Wait briefly
        # for the shared gate callback before submitting the test generation.
        time.sleep(0.15)

    def __exit__(self, *_args):
        import rclpy

        try:
            check(
                self.executor.shutdown(timeout_sec=3),
                "Startup client executor did not stop",
            )
            self.thread.join(timeout=2)
            check(not self.thread.is_alive(), "Startup client executor thread remained")
            self.node.destroy_node()
        finally:
            rclpy.try_shutdown()
        check(not self.errors, "Startup client executor failed: " + repr(self.errors))


def run(args):
    require_isolation()
    check(
        args.synthetic_software_test_approvals,
        "Explicit synthetic-only software test approval flag required",
    )
    check(
        args.output.is_absolute() and not args.output.exists(),
        "Use a new absolute validation output",
    )
    for path in (args.agent007_root, args.behav3d_root):
        check(path.is_absolute() and path.is_dir(), "Explicit code roots required")
    check(
        args.worker_python.is_absolute() and args.worker_python.is_file(),
        "Explicit scientific interpreter required",
    )
    check(
        30 <= args.timeout_s <= 600,
        "Startup scenario deadline must be in [30, 600] seconds",
    )
    args.output.mkdir(mode=0o700)
    started, processes = time.monotonic(), []
    report = {
        "schema": "behav3d.initial_startup_ros_validation.v1",
        "passed": False,
        "synthetic_fixture_only": True,
        "software_test_approvals_only": True,
        "real_recorded_scene_approved": False,
        "robot_commands_sent": False,
        "external_model_calls_sent": False,
        "fabrication_authorized": False,
        "client_pid": os.getpid(),
        "cases": [],
    }
    previous_handler = signal.getsignal(signal.SIGALRM)

    def expired(*_args):
        raise TimeoutError("Whole startup validation deadline exceeded")

    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, args.timeout_s)

    def record(name, **values):
        row = dict(name=name, passed=True, **values)
        report["cases"].append(row)
        print(json.dumps(row, sort_keys=True), flush=True)

    journal, artifacts = (
        args.output / "session/startup.jsonl",
        args.output / "artifacts",
    )
    artifacts.mkdir()

    def spawn(executable, generation):
        policy = {"journal_path": str(journal)}
        if executable == "initial_wall_backend":
            policy.update(
                input_roots=[
                    str(args.output / name) for name in ("source", "scene", "tasks")
                ],
                output_root=str(artifacts),
                agent007_root=str(args.agent007_root),
                behav3d_root=str(args.behav3d_root),
                worker_python=str(args.worker_python),
                worker_timeout_s=float(args.timeout_s - 20),
            )
        else:
            policy["operation_timeout_s"] = float(args.timeout_s - 15)
        process = StartupProcess(
            executable + "-" + str(generation),
            executable,
            policy,
            args.output / "logs",
            timeout_s=args.timeout_s + 30,
        ).start()
        processes.append(process)
        return process

    def batch_count():
        return len(list(artifacts.glob("batch-*")))

    try:
        command = [
            str(args.worker_python),
            "-B",
            str(Path(__file__).resolve()),
            "--prepare-fixture",
            "--agent007-root",
            str(args.agent007_root),
            "--behav3d-root",
            str(args.behav3d_root),
            "--output",
            str(args.output),
        ]
        prepared = subprocess.run(
            command,
            capture_output=True,
            timeout=min(40, args.timeout_s - 20),
            check=False,
        )
        (args.output / "prepare.stdout.log").write_bytes(prepared.stdout[:65536])
        (args.output / "prepare.stderr.log").write_bytes(prepared.stderr[:65536])
        check(
            prepared.returncode == 0
            and len(prepared.stdout) <= 65536
            and len(prepared.stderr) <= 65536,
            "Synthetic scientific fixture preparation failed; inspect prepare logs",
        )
        request = json.loads((args.output / "prepared.json").read_text())
        backend, coordinator = (
            spawn("initial_wall_backend", 1),
            spawn("initial_startup", 1),
        )
        with StartupClient() as client:
            client.ready()
            check(
                len({os.getpid(), backend.pid, coordinator.pid}) == 3,
                "Startup runtime processes not separate",
            )
            check(not client.send(request).accepted, "Disarmed startup accepted work")
            check(
                not list(artifacts.iterdir()), "Startup wrote artifacts while disarmed"
            )
            record("disarmed_rejects_new_startup")
            check(client.arm().success, "Startup review arm failed")
            handle = client.send(request)
            check(handle.accepted, "Synthetic placement PROPOSE rejected")
            proposed = client.finish(
                handle, request, "AWAITING_PLACEMENT_REVIEW", args.timeout_s
            )
            envelope = json.loads(Path(proposed["proposal_path"]).read_text())
            planning = envelope["planning"]
            check(
                planning["frame_id"] == "startup_test_world"
                and planning["fabrication_authorized"] is False,
                "Proposal changed frame or authority",
            )
            check(
                not list(artifacts.rglob("targets.yaml"))
                and not list(artifacts.rglob("*.npz")),
                "PROPOSE generated field or target batch",
            )
            record(
                "native_ros_propose_awaits_placement_review",
                proposal_id=planning["proposal_id"],
            )
            generation = dict(
                request,
                phase="GENERATE",
                proposal_path=proposed["proposal_path"],
                proposal_sha256=proposed["proposal_sha256"],
            )
            check(
                not client.send(generation, backend=True).accepted,
                "Direct backend bypassed durable approval gate",
            )
            check(
                not client.send(generation).accepted,
                "Coordinator generated before placement review",
            )
            check(
                not client.approve(request["request_id"], "0" * 64).success,
                "Wrong exact proposal hash approved",
            )
            check(batch_count() == 0, "Field/targets appeared before exact approval")
            record("no_generation_or_direct_backend_bypass_before_exact_approval")
            check(
                client.approve(
                    request["request_id"], proposed["proposal_sha256"]
                ).success,
                "Exact synthetic placement approval failed",
            )
            handle = client.send(generation)
            check(handle.accepted, "Approved first-batch preview rejected")
            generated = client.finish(
                handle, generation, "BATCH_PREVIEW_REVIEW_REQUIRED", args.timeout_s
            )
            batch = json.loads(Path(generated["batch_manifest_path"]).read_text())
            verify_preview(batch, planning, request)
            import yaml

            targets = yaml.safe_load(
                (
                    Path(generated["batch_manifest_path"]).parent / "targets.yaml"
                ).read_text()
            )
            check(
                len(targets["targets"]) == batch["candidate_count"]
                and batch_count() == 1,
                "Native YAML target count differs or first batch duplicated",
            )
            report["generated"] = generated
            record(
                "actual_production_kernels_first_batch_preview",
                candidate_count=batch["candidate_count"],
            )
            retry = client.send(generation)
            check(retry.accepted, "Completed preview retry rejected")
            cached = client.finish(
                retry, generation, "BATCH_PREVIEW_REVIEW_REQUIRED", 10
            )
            check(
                cached == generated and batch_count() == 1,
                "Cached retry regenerated or changed receipt",
            )
            record("cached_preview_retry_no_recomputation")
            # A new synthetic placement isolates pause/cancel from the cached
            # receipt path. No real recorded scene is approved by this harness.
            second = dict(request, request_id="synthetic-startup-pause-cancel")
            handle = client.send(second)
            check(handle.accepted, "Second synthetic PROPOSE rejected")
            second_proposal = client.finish(
                handle, second, "AWAITING_PLACEMENT_REVIEW", args.timeout_s
            )
            check(
                client.approve(
                    second["request_id"], second_proposal["proposal_sha256"]
                ).success,
                "Second synthetic exact approval failed",
            )
            client.pause("paused")
            second_generation = dict(
                second,
                phase="GENERATE",
                proposal_path=second_proposal["proposal_path"],
                proposal_sha256=second_proposal["proposal_sha256"],
            )
            handle = client.send(second_generation)
            check(handle.accepted, "Paused approved GENERATE rejected")
            eventually(
                lambda: (
                    read_journal(journal)[-1]["state"]["records"][second["request_id"]][
                        "state"
                    ]
                    == "GENERATING"
                ),
                timeout=5,
                description="generation held on shared pause gate",
            )
            check(
                batch_count() == 1,
                "Shared pause gate did not hold first-batch dispatch",
            )
            check(
                future_value(handle.cancel_goal_async(), 10).goals_canceling,
                "Paused startup cancellation not accepted",
            )
            client.finish(handle, second_generation, "CANCELLED", 10)
            check(
                batch_count() == 1, "Paused cancelled generation wrote a second batch"
            )
            client.pause("running")
            check(
                client.arm(False).success, "Explicit cancellation reconciliation failed"
            )
            record("shared_pause_and_cancel_prevent_backend_generation")
        coordinator.stop()
        coordinator = spawn("initial_startup", 2)
        with StartupClient() as client:
            client.ready()
            eventually(lambda: client.status()["state"] == "DISARMED", timeout=5)
            retry = client.send(generation)
            check(retry.accepted, "Restarted disarmed cached preview rejected")
            check(
                client.finish(retry, generation, "BATCH_PREVIEW_REVIEW_REQUIRED", 10)
                == generated
                and batch_count() == 1,
                "Restart recomputed or changed completed preview",
            )
            record("restart_disarmed_cached_receipt_no_approval_restoration")
            for field, value in (
                ("task_sha256", "f" * 64),
                ("frame_id", "unapproved_robot_alias"),
            ):
                check(client.arm().success, "Arm after reconciliation failed")
                invalid = dict(
                    request, request_id="synthetic-conflict-" + field, **{field: value}
                )
                handle = client.send(invalid)
                check(
                    handle.accepted,
                    "Typed source-conflict fixture rejected before backend test",
                )
                client.finish(handle, invalid, "STALE_SOURCE", 15)
                eventually(lambda: client.status()["state"] == "HOLD", timeout=5)
                check(batch_count() == 1, "Source/frame conflict generated new targets")
                check(
                    client.arm(False).success, "Source conflict reconciliation failed"
                )
            record("typed_task_and_frame_conflicts_hold_without_generation")
            eventually(lambda: client.status()["state"] == "DISARMED", timeout=5)
            report.update(
                passed=True, final_state="DISARMED", batch_count=batch_count()
            )
    except BaseException:
        report["error"] = traceback.format_exc(limit=16)
        if "client" in locals():
            report["last_result"] = client.last_result
        entries = read_journal(journal)
        if entries:
            report["last_journal_record"] = entries[-1]
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        for process in reversed(processes):
            try:
                process.stop()
            except Exception as error:
                report.setdefault("cleanup_errors", []).append(type(error).__name__)
                report["passed"] = False
        report["processes"] = [process.report() for process in processes]
        if any(
            not row["exit_matches_expectation"]
            or row["forced_kill"]
            or row["traceback_detected"]
            or row["logs_truncated"]
            or row["deadline_expired"]
            or not row["reader_threads_stopped"]
            for row in report["processes"]
        ):
            report["passed"] = False
        report["elapsed_s"] = round(time.monotonic() - started, 3)
        (args.output / "report.json").write_text(
            json.dumps(report, sort_keys=True, indent=2) + "\n"
        )
    check(report["passed"], "Startup validation cleanup failed; inspect report.json")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent007-root", type=Path, required=True)
    parser.add_argument("--behav3d-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--worker-python", type=Path, default=Path("/opt/behav3d_analysis/bin/python")
    )
    parser.add_argument("--timeout-s", type=float, default=120)
    parser.add_argument("--synthetic-software-test-approvals", action="store_true")
    parser.add_argument(
        "--prepare-fixture", action="store_true", help=argparse.SUPPRESS
    )
    args = parser.parse_args()
    if args.prepare_fixture:
        prepare_fixture(args)
    else:
        report = run(args)
        print(
            json.dumps(
                {
                    "passed": report["passed"],
                    "cases": len(report["cases"]),
                    "batch_count": report["batch_count"],
                }
            ),
            flush=True,
        )


if __name__ == "__main__":
    main()
