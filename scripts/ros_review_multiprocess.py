"""Three-process ROS review validation, with deliberate coordinator crashes.

Only a generated-interface test client lives here. Installed coordinator and
bridge executables run as separate Linux processes. This validates transport
and recovery of saved evidence, not physical safety or fresh scan accuracy.
Software approvals in this isolated test are not a human authentication system.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
import uuid
from copy import deepcopy
from pathlib import Path

from ros_review_smoke import (
    check,
    eventually,
    future_value,
    load_fixture,
    require_isolation,
)

LOG_LIMIT = 262144


def executable_path(executable):
    """Resolve only the two installed review entry points, not arbitrary code."""
    if executable not in {"observation_bridge", "supervised_agent_loop"}:
        raise ValueError("Only installed review executables are permitted")
    from ament_index_python.packages import get_package_prefix

    path = (
        Path(get_package_prefix("behav3d_orchestrator"))
        / "lib"
        / "behav3d_orchestrator"
        / executable
    )
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError("Installed review executable is unavailable: " + str(path))
    return path


def parameter_arguments(parameters):
    """ROS parameter values are argv entries; no shell is involved."""
    args = ["--ros-args"]
    for key, value in parameters.items():
        if not key or any(not (c.isalnum() or c == "_") for c in key):
            raise ValueError("Invalid parameter name")
        # JSON is a YAML subset, including safely quoted paths with spaces.
        args.extend(["-p", key + ":=" + json.dumps(value, allow_nan=False)])
    return args


def read_journal(path):
    """Read only complete durable lines; a concurrent append may be partial."""
    path = Path(path)
    if not path.exists():
        return []
    raw = path.read_bytes()
    if len(raw) > 16 * 1024 * 1024:
        raise ValueError("Test journal exceeds bounded read size")
    lines = raw.splitlines(keepends=True)
    return [json.loads(line) for line in lines if line.endswith(b"\n")]


def start_count(entries, request_id=None):
    return sum(
        entry["event"] == "START"
        and (request_id is None or entry["data"]["inflight_id"] == request_id)
        for entry in entries
    )


class RuntimeProcess:
    """Own one process group and bounded, continuously drained output logs."""

    def __init__(self, name, executable, parameters, output, timeout_s=180):
        if not name or any(not (c.isalnum() or c in "-_") for c in name):
            raise ValueError("Bounded log name must be an identifier")
        if not 0 < timeout_s <= 3600:
            raise ValueError("Runtime deadline must be in (0, 3600] seconds")
        self.name = name
        self.executable = executable
        self.parameters = dict(parameters)
        self.output = Path(output)
        self.timeout_s = timeout_s
        self.process = None
        self._buffers = {"stdout": bytearray(), "stderr": bytearray()}
        self._totals = {"stdout": 0, "stderr": 0}
        self._threads = []
        self._timer = None
        self._stop_lock = threading.RLock()
        self._overflow = threading.Event()
        self._expired = threading.Event()
        self._started = None
        self._stopped = False
        self._requested_stop = None
        self._stop_duration_s = None
        self._forced_kill = False

    @property
    def pid(self):
        return None if self.process is None else self.process.pid

    def start(self):
        if self.process is not None:
            raise RuntimeError("Create a new RuntimeProcess for each generation")
        self.output.mkdir(parents=True, exist_ok=True)
        command = [
            str(executable_path(self.executable)),
            *parameter_arguments(self.parameters),
        ]
        environment = dict(
            os.environ,
            PYTHONDONTWRITEBYTECODE="1",
            PYTHONUNBUFFERED="1",
            RCUTILS_LOGGING_USE_STDOUT="1",
        )
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            start_new_session=True,
            env=environment,
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
                    self._buffers[stream].extend(chunk[: max(0, remaining)])
                    if self._totals[stream] > LOG_LIMIT:
                        self._overflow.set()
            finally:
                pipe.close()

        for stream in ("stdout", "stderr"):
            thread = threading.Thread(
                target=drain, args=(stream,), daemon=True, name=self.name + "-" + stream
            )
            thread.start()
            self._threads.append(thread)

        def expire():
            self._expired.set()
            self.stop(kill=True)

        self._timer = threading.Timer(self.timeout_s, expire)
        self._timer.daemon = True
        self._timer.start()
        return self

    def text(self, stream="stdout"):
        if stream not in self._buffers:
            raise ValueError("Unknown output stream")
        return bytes(self._buffers[stream]).decode("utf-8", errors="replace")

    def healthy(self):
        if self._overflow.is_set():
            raise RuntimeError(self.name + " exceeded bounded log capture")
        if self._expired.is_set():
            raise TimeoutError(self.name + " exceeded process deadline")
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError(self.name + " exited before scenario completion")

    def stop(self, kill=False):
        with self._stop_lock:
            if self.process is None or self._stopped:
                return
            stop_started = time.monotonic()
            self._requested_stop = "SIGKILL" if kill else "SIGTERM"
            if self._timer is not None:
                self._timer.cancel()
            # The owned group may still contain children after its leader exits.
            # Never signal a reused unrelated process: this Popen created a new session.
            try:
                os.killpg(self.process.pid, signal.SIGKILL if kill else signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                # ROS nodes themselves allow 10 s for executor drainage. Do not
                # pre-empt that documented grace period from the test harness.
                self.process.wait(timeout=1.5 if kill else 12.0)
            except subprocess.TimeoutExpired:
                self._forced_kill = not kill
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
                # A leader that exited zero can still leave owned descendants.
                # Killing those is cleanup, not a successful graceful shutdown.
                self._forced_kill = self._forced_kill or not kill
            except ProcessLookupError:
                pass
            self.process.wait(timeout=1.5)
            for thread in self._threads:
                if thread is not threading.current_thread():
                    thread.join(timeout=1)
            for stream, data in self._buffers.items():
                (self.output / (self.name + "." + stream + ".log")).write_bytes(data)
            self._stop_duration_s = round(time.monotonic() - stop_started, 3)
            self._stopped = True

    def report(self):
        returncode = None if self.process is None else self.process.poll()
        expected = -signal.SIGKILL if self._requested_stop == "SIGKILL" else 0
        return {
            "name": self.name,
            "pid": self.pid,
            "executable": self.executable,
            "returncode": returncode,
            "requested_stop": self._requested_stop,
            "stop_duration_s": self._stop_duration_s,
            "forced_kill": self._forced_kill,
            "exit_matches_expectation": self.process is None or returncode == expected,
            "log_bytes_received": dict(self._totals),
            "log_limit_bytes": LOG_LIMIT,
            "logs_truncated": self._overflow.is_set(),
            "deadline_expired": self._expired.is_set(),
            "traceback_detected": any(
                "Traceback (most recent call last):" in self.text(stream)
                for stream in ("stdout", "stderr")
            ),
            "reader_threads_stopped": all(
                not thread.is_alive() for thread in self._threads
            ),
        }


class ReviewClient:
    """Real ROS client only; each restart replaces all discovery/status handles."""

    def __init__(self, timeout_s=15.0):
        self.timeout_s = timeout_s
        self.statuses = []
        self._lock = threading.Lock()
        self._errors = []
        self._node = None
        self._executor = None
        self._thread = None
        self._generation = 0

    def __enter__(self):
        import rclpy

        if rclpy.ok():
            raise RuntimeError("ReviewClient requires its own ROS context lifecycle")
        rclpy.init(args=[])
        self.new_generation()
        return self

    def _close_generation(self):
        if self._executor is not None:
            check(
                self._executor.shutdown(timeout_sec=2.0),
                "Client executor did not drain",
            )
        if self._thread is not None:
            self._thread.join(timeout=2)
            check(not self._thread.is_alive(), "Client executor thread remained active")
        if self._node is not None:
            self._action.destroy()
            self._node.destroy_node()
        self._executor = self._thread = self._node = None

    def new_generation(self):
        self._close_generation()
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String

        from behav3d_interfaces.action import EvaluateObservation
        from behav3d_interfaces.srv import ApproveSupervisedStep, SetSupervisorArmed

        self._generation += 1
        generation = self._generation
        self._node = Node("review_multiprocess_client_" + uuid.uuid4().hex[:12])
        self._action = ActionClient(
            self._node,
            EvaluateObservation,
            "/behav3d/supervised_agent/review_observation",
        )
        self._arm = self._node.create_client(
            SetSupervisorArmed, "/behav3d/supervised_agent/set_armed"
        )
        self._approval = self._node.create_client(
            ApproveSupervisedStep, "/behav3d/supervised_agent/approve_step"
        )
        self.statuses = []
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        def on_status(message):
            value = json.loads(message.data)
            check(
                value.get("fabrication_authorized") is False,
                "Status granted fabrication",
            )
            check(
                value.get("hardware_execution_available") is False,
                "Hardware execution exposed",
            )
            with self._lock:
                if generation == self._generation:
                    self.statuses.append(value)
                    del self.statuses[:-256]

        self._node.create_subscription(
            String, "/behav3d/supervised_agent/status", on_status, qos
        )
        self._control = self._node.create_publisher(
            String, "/behav3d/control_state", qos
        )
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._node)

        def spin():
            try:
                self._executor.spin()
            except BaseException as exc:  # noqa: BLE001 - surface every background executor failure.
                self._errors.append(repr(exc))

        self._thread = threading.Thread(
            target=spin, daemon=True, name="review-client-executor"
        )
        self._thread.start()
        return self

    def ready(self, timeout=10):
        check(
            self._action.wait_for_server(timeout_sec=timeout),
            "Review action unavailable",
        )
        check(
            self._arm.wait_for_service(timeout_sec=timeout), "Arm service unavailable"
        )
        check(
            self._approval.wait_for_service(timeout_sec=timeout),
            "Approval service unavailable",
        )
        eventually(
            lambda: self.status() is not None,
            timeout=timeout,
            description="fresh ROS status",
        )
        check(not self._errors, "Client executor failed: " + repr(self._errors))
        return self

    def status(self):
        with self._lock:
            return None if not self.statuses else deepcopy(self.statuses[-1])

    def arm(self, armed=True):
        from behav3d_interfaces.srv import SetSupervisorArmed

        request = SetSupervisorArmed.Request()
        request.armed = armed
        request.operator_id = "isolated-software-test-not-human"
        return future_value(self._arm.call_async(request), self.timeout_s)

    def send(self, values):
        from behav3d_interfaces.action import EvaluateObservation

        goal = EvaluateObservation.Goal()
        for key, value in values.items():
            setattr(goal, key, value)
        return future_value(self._action.send_goal_async(goal), self.timeout_s)

    def pending(self, request_id):
        found = []

        def locate():
            with self._lock:
                for status in reversed(self.statuses):
                    pending = status.get("pending")
                    if pending and pending.get("request_id") == request_id:
                        found.append(deepcopy(pending))
                        return True
            return False

        eventually(
            locate,
            timeout=self.timeout_s,
            description="pending exact proposal " + request_id,
        )
        return found[-1]

    def approve(self, request_id, digest=None, decision=True):
        from behav3d_interfaces.srv import ApproveSupervisedStep

        request = ApproveSupervisedStep.Request()
        request.request_id = request_id
        request.proposal_sha256 = (
            self.pending(request_id)["proposal_sha256"] if digest is None else digest
        )
        request.operator_id = "isolated-software-test-not-human"
        request.approve = decision
        return future_value(self._approval.call_async(request), self.timeout_s)

    def finish(self, handle, status="REVIEW_REQUIRED", timeout=20):
        from action_msgs.msg import GoalStatus

        wrapped = future_value(handle.get_result_async(), timeout)
        result = wrapped.result
        if status is not None:
            check(result.status == status, "Unexpected typed status: " + result.status)
        expected = (
            GoalStatus.STATUS_SUCCEEDED
            if result.success
            else GoalStatus.STATUS_CANCELED
            if wrapped.status == GoalStatus.STATUS_CANCELED
            else GoalStatus.STATUS_ABORTED
        )
        check(wrapped.status == expected, "Unexpected ROS terminal state")
        check(
            result.requires_review is True and result.fabrication_authorized is False,
            "Review result leaked fabrication authority",
        )
        check(
            (result.success and result.status == "REVIEW_REQUIRED")
            or (
                not result.success
                and result.status
                in {
                    "FAILED",
                    "REJECTED",
                    "CANCELLED",
                    "TIMED_OUT",
                    "UNSUPPORTED",
                    "STALE_SOURCE",
                }
            ),
            "Inconsistent success/status contract",
        )
        return result

    def publish_control(self, state):
        from std_msgs.msg import String

        if state not in {"paused", "running"}:
            raise ValueError("Only cooperative pause state is permitted")
        eventually(
            lambda: self._control.get_subscription_count() > 0,
            timeout=self.timeout_s,
            description="cooperative pause subscription",
        )
        self._control.publish(String(data=state))

    def __exit__(self, *exc):
        import rclpy

        try:
            self._close_generation()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        check(not self._errors, "Client executor failed: " + repr(self._errors))


def advance_fixture(fixture, previous_evaluation):
    """Create a new immutable synthetic cycle, retaining the old bundle."""
    fixture.bundle = fixture.root / "evidence-cycle-3"
    fixture.bundle.mkdir()
    fixture.shell = dict(fixture.shell, scan_cycle=3, candidate_through_cycle=2)
    fixture.evaluation.update(
        scan_cycle=3,
        candidate_cycle=2,
        scan_job_id="scan-3",
        previous_evaluation_id=previous_evaluation,
        shell=fixture.shell,
    )
    fixture.evaluation["ledger"].append(
        {
            "scan_cycle": 3,
            "scan_job_id": "scan-3",
            "source_sha256": fixture.evaluation["source_sha256"],
        }
    )
    fixture.evaluation["candidates"].append(
        dict(
            fixture.evaluation["candidates"][0],
            label="C2:T1",
            evidence_id="candidate:C2:T1",
            candidate_cycle=2,
            observation_cycle=3,
        )
    )
    fixture.evaluation["summary"]["accumulated_candidate_count"] = 2
    fixture.evaluation["suggestions"] = [
        dict(
            fixture.suggestion,
            suggestion_id="review:C2:T1",
            evidence_ids=["candidate:C2:T1"],
            candidate_labels=["C2:T1"],
        )
    ]
    fixture.request.update(
        scan_cycle=3,
        candidate_cycle=2,
        observation_id="scan-3",
        previous_evaluation_id=previous_evaluation,
        source_manifest_path=str(fixture.bundle / "manifest.json"),
    )
    fixture.publish()


def run(output, repo, timeout_s=140.0):
    require_isolation()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # Reserve < 40 s for client teardown and two 12 s runtime grace periods
    # beneath the documented outer 180 s hard deadline.
    check(0 < timeout_s <= 140, "Global scenario budget must be in (0, 140] seconds")
    fixture = load_fixture(repo)
    session = fixture.root / "session"
    session.mkdir()
    journal = session / "supervised_agent" / "review_journal.jsonl"
    processes = []
    started = time.monotonic()
    report = {
        "schema": "behav3d.ros_review_multiprocess.v1",
        "passed": False,
        "client_pid": os.getpid(),
        "real_rclpy": True,
        "generated_interfaces": True,
        "runtime_nodes_in_client": False,
        "synthetic_evidence": True,
        "robot_commands_sent": False,
        "physical_accuracy_validated": False,
        "software_test_approvals_only": True,
        "fabrication_authorized": False,
        "network_scope": "isolated_container_loopback_dds",
        "cases": [],
    }
    previous_handler = signal.getsignal(signal.SIGALRM)

    def expired(*_):
        raise TimeoutError("Overall multiprocess validation deadline exceeded")

    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, timeout_s)

    def record(name, **values):
        case = dict(name=name, passed=True, **values)
        report["cases"].append(case)
        print(json.dumps(case, sort_keys=True), flush=True)

    def spawn(role, generation):
        parameters = (
            {
                "evidence_root": str(fixture.root),
                "source_roots": [str(fixture.root)],
                "worker_python": sys.executable,
                "worker_timeout_s": 5.0,
            }
            if role == "bridge"
            else {
                "session_path": str(session),
                "approval_timeout_s": 10.0,
                       "observation_timeout_s": 8.0,
            }
        )
        executable = (
            "observation_bridge" if role == "bridge" else "supervised_agent_loop"
        )
        process = RuntimeProcess(
            role + "-" + str(generation),
            executable,
            parameters,
            output / "logs",
            timeout_s=min(170, timeout_s),
        ).start()
        processes.append(process)
        return process

    def entries():
        return read_journal(journal)

    def latest():
        value = entries()
        check(bool(value), "Durable journal unavailable")
        return value[-1]["data"]

    def wait_idle(client):
        eventually(
            lambda: client.status() is not None and client.status()["pending"] is None,
            description="idle status after result",
        )

    try:
        bridge = spawn("bridge", 1)
        coordinator = spawn("coordinator", 1)
        with ReviewClient() as client:
            client.ready()
            check(
                len({os.getpid(), bridge.pid, coordinator.pid}) == 3,
                "Client and runtime nodes are not three different processes",
            )
            check(
                not client.send(
                    dict(fixture.request, request_id="mp-disarmed")
                ).accepted,
                "New goal accepted while disarmed",
            )
            check(client.arm().success, "Could not explicitly arm software review")
            values = dict(fixture.request, request_id="mp-success")
            handle = client.send(values)
            check(handle.accepted, "Valid armed review rejected")
            check(
                not client.approve(values["request_id"], digest="f" * 64).success,
                "Wrong exact-proposal hash was accepted",
            )
            check(
                client.approve(values["request_id"]).success,
                "Exact software approval rejected",
            )
            result = client.finish(handle)
            wait_idle(client)
            check(
                result.evaluation_id == fixture.evaluation["evaluation_id"],
                "Worker identity changed",
            )
            record(
                "three_process_typed_ros_exact_approval",
                bridge_pid=bridge.pid,
                coordinator_pid=coordinator.pid,
                evaluation_id=result.evaluation_id,
            )

            before = start_count(entries())
            cached = client.send(values)
            check(cached.accepted, "Identical cached retry rejected")
            check(
                client.finish(cached).packet_sha256 == result.packet_sha256,
                "Cached packet changed",
            )
            wait_idle(client)
            check(start_count(entries()) == before, "Cached retry wrote another START")
            record("duplicate_cached_without_start")

            advance_fixture(fixture, result.evaluation_id)
            pending_values = dict(fixture.request, request_id="mp-crash-pending")
            pending_handle = client.send(pending_values)
            check(pending_handle.accepted, "Crash-before-approval proposal rejected")
            old_hash = client.pending(pending_values["request_id"])["proposal_sha256"]
            eventually(
                lambda: (
                    latest()["records"][pending_values["request_id"]]["state"]
                    == "PENDING"
                )
            )
            before = start_count(entries())
            coordinator.stop(kill=True)
            coordinator = spawn("coordinator", 2)
            client.new_generation().ready()
            eventually(
                lambda: client.status()["state"] == "HOLD",
                description="restarted coordinator HOLD status",
            )
            check(
                latest()["armed"] is False and client.status()["state"] == "HOLD",
                "Pending crash did not restart with disarmed HOLD authority",
            )
            check(
                not client.approve(
                    pending_values["request_id"], digest=old_hash
                ).success,
                "Pre-crash pending approval survived restart",
            )
            check(
                not client.send(pending_values).accepted,
                "Held request was automatically retryable",
            )
            check(
                start_count(entries()) == before, "Pending crash caused automatic START"
            )
            record("sigkill_pending_restarts_disarmed_hold_without_approval_or_start")

            # Explicit disarm reconciles a non-running held record. Nothing retries it.
            check(
                client.arm(False).success, "Could not explicitly reconcile pending HOLD"
            )
            cached = client.send(values)
            check(cached.accepted, "Completed receipt retry rejected after restart")
            check(
                client.finish(cached).packet_sha256 == result.packet_sha256,
                "Completed receipt changed after coordinator restart",
            )
            wait_idle(client)
            check(
                start_count(entries()) == before,
                "Restored completed receipt dispatched worker",
            )
            record("completed_receipt_cache_survives_coordinator_restart")

            check(client.arm().success, "Could not rearm after explicit reconciliation")
            client.publish_control("paused")
            eventually(
                lambda: "Global pause requested" in coordinator.text(),
                description="real shared pause reception in coordinator process",
            )
            running_values = dict(fixture.request, request_id="mp-crash-running-paused")
            running_handle = client.send(running_values)
            check(running_handle.accepted, "Paused crash proposal rejected")
            running_hash = client.pending(running_values["request_id"])[
                "proposal_sha256"
            ]
            check(
                client.approve(running_values["request_id"]).success,
                "Paused proposal approval rejected",
            )
            eventually(
                lambda: latest()["inflight_id"] == running_values["request_id"],
                description="durable START before shared pause",
            )
            eventually(
                lambda: "Paused before evaluate_observation" in coordinator.text(),
                description="actual shared gate blocking command dispatch",
            )
            check(
                latest()["records"][running_values["request_id"]]["receipt"] is None,
                "Paused operation produced a receipt before dispatch",
            )
            before = start_count(entries())
            coordinator.stop(kill=True)
            coordinator = spawn("coordinator", 3)
            client.new_generation().ready()
            eventually(
                lambda: client.status()["state"] == "HOLD",
                description="restarted in-flight coordinator HOLD status",
            )
            check(
                latest()["armed"] is False and latest()["inflight_id"] is None,
                "In-flight crash restored armed authority",
            )
            check(
                latest()["records"][running_values["request_id"]]["state"] == "HOLD",
                "Uncertain in-flight result not held",
            )
            check(
                not client.approve(
                    running_values["request_id"], digest=running_hash
                ).success,
                "Consumed in-flight approval became reusable",
            )
            check(
                not client.arm().success, "Unreconciled in-flight HOLD could be armed"
            )
            check(
                start_count(entries()) == before,
                "In-flight restart automatically redispatched",
            )
            record("sigkill_after_start_at_shared_pause_restarts_hold_without_dispatch")

            check(client.arm(False).success, "Explicit HOLD reconciliation failed")
            client.publish_control("running")
            # Stop the backend before approval. Typed transport must fail within
            # the command deadline, with no successful artifact head promotion.
            bridge.stop(kill=True)
            heads = deepcopy(latest()["heads"])
            check(client.arm().success, "Could not arm outage test")
            outage_values = dict(fixture.request, request_id="mp-bridge-outage")
            outage = client.send(outage_values)
            check(outage.accepted, "Outage proposal rejected before approval")
            check(
                client.approve(outage_values["request_id"]).success,
                "Outage approval rejected",
            )
            outage_started = time.monotonic()
            failure = client.finish(outage, status=None, timeout=15)
            check(
                not failure.success and failure.status in {"FAILED", "TIMED_OUT"},
                "Bridge outage did not return bounded failed review",
            )
            wait_idle(client)
            check(
                latest()["heads"] == heads, "Bridge outage advanced the artifact head"
            )
            check(latest()["armed"] is False, "Bridge outage left coordinator armed")
            record(
                "bridge_outage_bounded_failure_without_head_advance",
                status=failure.status,
                duration_s=round(time.monotonic() - outage_started, 3),
            )

            bridge = spawn("bridge", 2)
            eventually(lambda: 'Observation bridge: recorded' in bridge.text(), timeout=10,
                       description='new installed bridge process initialized')
            client.new_generation().ready()
            check(client.arm().success, "Explicit rearm after bridge recovery failed")
            recovered_values = dict(
                fixture.request, request_id="mp-recovery-new-request"
            )
            recovered = client.send(recovered_values)
            check(recovered.accepted, "Fresh recovery request rejected")
            check(
                client.approve(recovered_values["request_id"]).success,
                "Fresh recovery approval rejected",
            )
            verified = client.finish(recovered)
            wait_idle(client)
            check(
                verified.evaluation_id == fixture.evaluation["evaluation_id"],
                "Recovered review did not verify the new synthetic cycle",
            )
            check(
                latest()["heads"]["artifact-a"]["scan_cycle"] == 3,
                "Recovery head not advanced",
            )
            check(client.arm(False).success, "Final disarm failed")
            record(
                "bridge_restart_explicit_arm_new_request_recovers",
                evaluation_id=verified.evaluation_id,
            )
            bridge.healthy()
            coordinator.healthy()
            report["passed"] = True
    except BaseException:
        report["error"] = traceback.format_exc(limit=14)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        for process in reversed(processes):
            try:
                process.stop()
            except Exception as exc:  # noqa: BLE001 - still clean the remaining owned processes.
                report.setdefault("cleanup_errors", []).append(repr(exc))
        report["processes"] = [process.report() for process in processes]
        report["all_runtime_processes_stopped"] = all(
            process.process is None or process.process.poll() is not None
            for process in processes
        )
        report["journal"] = entries()
        report["duration_s"] = round(time.monotonic() - started, 3)
        if (
            report.get("cleanup_errors")
            or not report["all_runtime_processes_stopped"]
            or any(
                p["logs_truncated"]
                or p["deadline_expired"]
                or not p["reader_threads_stopped"]
                or not p["exit_matches_expectation"]
                or p["forced_kill"]
                or p["traceback_detected"]
                for p in report["processes"]
            )
        ):
            report["passed"] = False
        (output / "summary.json").write_text(
            json.dumps(report, sort_keys=True, indent=2) + "\n"
        )
        fixture.doCleanups()
    check(report["passed"], "Multiprocess cleanup failed; inspect summary.json")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--repo", type=Path, default=Path(__file__).resolve().parents[1]
    )
    parser.add_argument("--timeout-s", type=float, default=140.0)
    args = parser.parse_args()
    result = run(args.output.resolve(), args.repo.resolve(), args.timeout_s)
    print(
        json.dumps(
            {
                "passed": result["passed"],
                "case_count": len(result["cases"]),
                "report": str(args.output.resolve() / "summary.json"),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
