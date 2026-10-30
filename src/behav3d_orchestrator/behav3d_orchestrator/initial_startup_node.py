"""Native ROS startup proposal/review/first-batch preview. No robot bringup."""

# ruff: noqa: BLE001 -- ROS callback boundaries must retain a fail-closed result
from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from behav3d_interfaces.action import InitialWall
from behav3d_interfaces.srv import ApproveSupervisedStep, SetSupervisorArmed

from .src.initial_session import InitialWallSession
from .supervised_agent.initial_process import run_initial_worker
from .supervised_agent.initial_workflow import (
    REQUEST_FIELDS,
    RESULT_FIELDS,
    StartupError,
    StartupSupervisor,
    base_request,
    error_result,
    generation_authorized,
    validate_request,
    validate_result,
)
from .supervised_agent.process_runner import WorkerError


def request_from_goal(goal):
    return validate_request({name: getattr(goal, name) for name in REQUEST_FIELDS})


def action_result(values):
    result = InitialWall.Result()
    for name in RESULT_FIELDS:
        setattr(result, name, values[name])
    return result


class _StartupAction(Node):
    """Reserve one goal and discard cancelled/late scientific results."""

    def __init__(self, name, action_name):
        super().__init__(name)
        self._lock = threading.RLock()
        self._busy = False
        self._cancel = threading.Event()
        self._shutdown = threading.Event()
        self._ros_cancel_accepted = False
        self._server = ActionServer(
            self,
            InitialWall,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel_goal,
            callback_group=ReentrantCallbackGroup(),
        )

    def _goal(self, goal):
        try:
            request = request_from_goal(goal)
            with self._lock:
                if self._busy or self._shutdown.is_set() or not self._allowed(request):
                    return GoalResponse.REJECT
                self._busy = True
                self._cancel.clear()
                self._ros_cancel_accepted = False
            return GoalResponse.ACCEPT
        except (ValueError, TypeError, AttributeError):
            return GoalResponse.REJECT

    def _cancel_goal(self, _handle):
        with self._lock:
            self._ros_cancel_accepted = True
            self._cancel.set()
            self._invalidate("ROS startup cancellation")
        return CancelResponse.ACCEPT

    def _execute(self, handle):
        request = request_from_goal(handle.request)
        try:
            feedback = InitialWall.Feedback()
            feedback.stage, feedback.message = (
                request["phase"],
                "Review-only startup; no physical execution.",
            )
            handle.publish_feedback(feedback)
            result = self._operation(request)
            result = validate_result(result, request)
        except WorkerError as error:
            result = error_result(request["request_id"], error.status, str(error))
        except Exception:
            self._invalidate("Startup operation failed")
            result = error_result(
                request["request_id"],
                "FAILED",
                "Startup failed; inspect the journal and worker artifacts.",
            )
        try:
            deadline = time.monotonic() + 1
            while True:
                with self._lock:
                    await_cancel = (
                        self._ros_cancel_accepted
                        and not handle.is_cancel_requested
                        and time.monotonic() < deadline
                    )
                    if not await_cancel:
                        if (
                            self._cancel.is_set()
                            or self._shutdown.is_set()
                            or handle.is_cancel_requested
                        ):
                            result = error_result(
                                request["request_id"],
                                "CANCELLED",
                                "Late startup artifacts unaccepted after cancellation/disarm.",
                            )
                        response = action_result(result)
                        if handle.is_cancel_requested:
                            handle.canceled()
                        elif result["success"]:
                            handle.succeed()
                        else:
                            handle.abort()
                        return response
                time.sleep(0.001)
        finally:
            with self._lock:
                self._busy = False
            self._publish()

    def _invalidate(self, _reason):
        pass

    def _publish(self):
        pass

    def begin_shutdown(self):
        with self._lock:
            self._shutdown.set()
            self._cancel.set()
            self._invalidate("Startup node shutdown")

    def destroy_node(self):
        self.begin_shutdown()
        self._server.destroy()
        return super().destroy_node()


class InitialWallBackend(_StartupAction):
    def __init__(self):
        super().__init__("behav3d_initial_wall_backend", "/behav3d/startup/plan_wall")
        self._policy = {
            "python_executable": self.declare_parameter(
                "worker_python", sys.executable
            ).value,
            "input_roots": [
                value
                for value in self.declare_parameter("input_roots", [""]).value
                if value
            ],
            "output_root": self.declare_parameter("output_root", "").value,
            "agent007_root": self.declare_parameter("agent007_root", "").value,
            "behav3d_root": self.declare_parameter("behav3d_root", "").value,
            "journal_path": self.declare_parameter("journal_path", "").value,
            "timeout_s": self.declare_parameter("worker_timeout_s", 120.0).value,
        }

    def _allowed(self, request):
        paths = [
            *self._policy["input_roots"],
            *(
                self._policy[name]
                for name in ("output_root", "agent007_root", "behav3d_root")
            ),
        ]
        if not self._policy["input_roots"] or any(
            not value or not Path(value).is_absolute() or not Path(value).is_dir()
            for value in paths
        ):
            return False
        return request["phase"] == "PROPOSE" or generation_authorized(
            self._policy["journal_path"], request
        )

    def _operation(self, request):
        return run_initial_worker(
            request,
            **self._policy,
            cancelled=lambda: self._cancel.is_set() or self._shutdown.is_set(),
        )


class InitialStartupCoordinator(_StartupAction):
    def __init__(self):
        super().__init__("behav3d_initial_startup", "/behav3d/startup/review_wall")
        journal = self.declare_parameter("journal_path", "").value
        if not journal or not Path(journal).is_absolute():
            raise ValueError("Configure an explicit startup journal path")
        self.supervisor = StartupSupervisor(journal)
        self.session = InitialWallSession(self)
        self.timeout = self.declare_parameter("operation_timeout_s", 125.0).value
        group = ReentrantCallbackGroup()
        self._arm_service = self.create_service(
            SetSupervisorArmed,
            "/behav3d/startup/set_armed",
            self._arm,
            callback_group=group,
        )
        self._approve_service = self.create_service(
            ApproveSupervisedStep,
            "/behav3d/startup/approve_placement",
            self._approve,
            callback_group=group,
        )
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._status = self.create_publisher(String, "/behav3d/startup/status", qos)
        self._timer = self.create_timer(0.5, self._publish)
        self._publish()

    def _allowed(self, request):
        state = self.supervisor.snapshot()
        record = state["records"].get(request["request_id"])
        if record and record["binding"] != base_request(request):
            self.supervisor.invalidate("Changed startup request binding")
            return False
        if record and (
            (
                request["phase"] == "PROPOSE"
                and record["proposal"] is not None
                and record["state"] not in {"HOLD", "INVALIDATED"}
            )
            or (request["phase"] == "GENERATE" and record["state"] == "GENERATED")
        ):
            return True
        if not state["armed"]:
            return False
        return (
            request["phase"] == "PROPOSE"
            and record is None
            and state["active_id"] is None
        ) or (
            request["phase"] == "GENERATE"
            and record is not None
            and record["state"] == "APPROVED"
        )

    def _operation(self, request):
        def run():
            command = self.session.evaluate(
                request, timeout_s=self.timeout, cancel_event=self._cancel
            )
            if not command.get("ok"):
                metrics = command.get("metrics", {})
                if all(name in metrics for name in RESULT_FIELDS):
                    # A source-bound HOLD report is inspectable evidence, not
                    # an approvable placement or successful batch.
                    return validate_result(
                        {name: metrics[name] for name in RESULT_FIELDS}, request
                    )
                status = command.get("metrics", {}).get("status", "FAILED")
                if status not in {
                    "REJECTED",
                    "FAILED",
                    "HOLD",
                    "CANCELLED",
                    "TIMED_OUT",
                    "STALE_SOURCE",
                }:
                    status = "FAILED"
                return error_result(
                    request["request_id"],
                    status,
                    "Startup backend unavailable or failed; generation remains held.",
                )
            return {name: command["metrics"][name] for name in RESULT_FIELDS}

        result = self.supervisor.execute(request, run)
        self._publish()
        return result

    def _arm(self, request, response):
        try:
            if not request.armed:
                self._cancel.set()
            self.supervisor.set_armed(request.armed, request.operator_id)
            response.success, response.message = (
                True,
                "Startup review arm state recorded; no physical authority.",
            )
        except (ValueError, OSError):
            response.success, response.message = (
                False,
                "Startup arm change rejected; inspect journal.",
            )
        response.state = self._state_name()
        self._publish()
        return response

    def _approve(self, request, response):
        try:
            self.supervisor.approve(
                request.request_id,
                request.proposal_sha256,
                request.operator_id,
                approve=request.approve,
            )
            if not request.approve:
                self._cancel.set()
            response.success, response.message = (
                True,
                "Exact placement review recorded; generated batches remain previews.",
            )
        except (ValueError, OSError):
            response.success, response.message = False, "Placement approval rejected."
        response.state = self._state_name()
        self._publish()
        return response

    def _state_name(self):
        try:
            return self.supervisor.snapshot()["state"]
        except StartupError:
            return "HOLD"

    def _publish(self):
        if not hasattr(self, "_status"):
            return
        try:
            state = self.supervisor.snapshot()
            active = state["records"].get(state["active_id"])
            proposal = active["proposal"] if active else None
            values = {
                "schema": "behav3d.initial_startup_status.v1",
                "state": state["state"],
                "armed": state["armed"],
                "request_id": state["active_id"],
                "proposal_path": proposal["proposal_path"] if proposal else "",
                "proposal_sha256": proposal["proposal_sha256"] if proposal else "",
                "fabrication_authorized": False,
                "hardware_execution_available": False,
            }
        except StartupError:
            values = {
                "state": "HOLD",
                "fabrication_authorized": False,
                "hardware_execution_available": False,
            }
        self._status.publish(String(data=json.dumps(values)))

    def _invalidate(self, reason):
        if hasattr(self, "supervisor"):
            try:
                self.supervisor.invalidate(reason)
            except StartupError:
                pass

    def destroy_node(self):
        self.begin_shutdown()
        try:
            self.supervisor.close()
        except StartupError:
            pass
        return super().destroy_node()


def _main(node_type, args=None):
    rclpy.init(args=args)
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # Shutdown can invalidate the ROS context during WaitSet construction.
        # The same exception with an active context is a real failure.
        if rclpy.ok():
            raise
    finally:
        node.begin_shutdown()
        executor.shutdown(timeout_sec=10)
        node.destroy_node()
        rclpy.try_shutdown()


def main(args=None):
    _main(InitialStartupCoordinator, args)


def backend_main(args=None):
    _main(InitialWallBackend, args)
