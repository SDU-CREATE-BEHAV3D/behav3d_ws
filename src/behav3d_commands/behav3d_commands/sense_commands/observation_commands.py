#!/usr/bin/env python3
"""Opt-in, review-only observation action using the Session command contract."""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, Optional

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

try:
    from behav3d_interfaces.action import EvaluateObservation
except ImportError:
    from behav3d_interfaces.action._evaluate_observation import EvaluateObservation

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue


_STRING_FIELDS = (
    "request_id", "artifact_id", "observation_id", "event_type",
    "previous_evaluation_id", "frame_id", "coordinate_units",
    "source_manifest_path", "source_manifest_sha256",
)
_CYCLE_FIELDS = ("candidate_cycle", "scan_cycle")
_RESULT_STRING_FIELDS = (
    "status", "message", "request_id", "evaluation_id", "packet_path",
    "packet_sha256", "evidence_manifest_path", "evidence_manifest_sha256",
)
_RESULT_BOOL_FIELDS = ("success", "requires_review", "fabrication_authorized")
_SERVER_STATUSES = frozenset({
    "REVIEW_REQUIRED", "FAILED", "REJECTED", "CANCELLED", "TIMED_OUT",
    "UNSUPPORTED", "STALE_SOURCE",
})


def _validated_request(request: Dict[str, Any]) -> Dict[str, Any]:
    """Copy exact wire fields without lossy string or integer coercion."""
    if not isinstance(request, dict):
        raise ValueError("observation request must be a dict")
    if set(request) != set(_STRING_FIELDS + _CYCLE_FIELDS):
        raise ValueError("observation request must contain exactly the action goal fields")
    for name in _STRING_FIELDS:
        if not isinstance(request[name], str):
            raise ValueError(f"{name} must be a string")
        if name != "previous_evaluation_id" and not request[name].strip():
            raise ValueError(f"{name} must not be blank")
    for name in _CYCLE_FIELDS:
        if type(request[name]) is not int or not -(2 ** 63) <= request[name] < 2 ** 63:
            raise ValueError(f"{name} must be an int64, not a coerced value")
    return dict(request)


class ObservationCommands:
    """Evaluate observations without adding a robot-control transport.

    Registration is deliberately opt-in. A steady-clock timer bounds server
    discovery, goal acceptance and result completion together. Cancellation is
    best effort; a timeout always releases the command as failure and late
    accepted goals are cancelled. The caller must hold subsequent physical work
    after any failure, rather than assuming remote cancellation has completed.
    """

    def __init__(
        self, node: Node, *, queue: Optional[SessionQueue] = None,
        action_name: str = "/behav3d/evaluate_observation",
    ):
        self._node = node
        self._queue = queue
        self._action_client = ActionClient(node, EvaluateObservation, action_name)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

    def register(self, router) -> None:
        router.register("evaluate_observation", self._handle_evaluate_observation)

    def evaluate_observation(
        self, request: Dict[str, Any], *, timeout_s: float = 120.0,
        cancel_event: Optional[threading.Event] = None,
        on_done: OnCommandDone = None, enqueue: bool = True,
    ):
        """Build/enqueue a review-only command; cancel_event is never sent to ROS."""
        request_copy = _validated_request(request)
        if isinstance(timeout_s, bool):
            raise ValueError("timeout_s must be finite and positive")
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout_s must be finite and positive")
        if cancel_event is not None and not isinstance(cancel_event, threading.Event):
            raise ValueError("cancel_event must be a threading.Event")
        item = QueueItem(
            "evaluate_observation",
            {"request": request_copy, "timeout_s": timeout, "cancel_event": cancel_event},
            cmd_kind="evaluate_observation", on_done=on_done,
        )
        if not enqueue:
            return item
        if self._queue is None:
            raise RuntimeError("ObservationCommands requires a SessionQueue to enqueue items.")
        self._queue.enqueue(item)
        return None

    def _handle_evaluate_observation(self, payload: Dict[str, Any], cmd: Command) -> None:
        try:
            request = _validated_request(payload["request"])
            if isinstance(payload["timeout_s"], bool):
                raise ValueError("timeout_s must be finite and positive")
            timeout_s = float(payload["timeout_s"])
            if not math.isfinite(timeout_s) or timeout_s <= 0:
                raise ValueError("timeout_s must be finite and positive")
            cancel_event = payload.get("cancel_event")
            if cancel_event is not None and not isinstance(cancel_event, threading.Event):
                raise ValueError("cancel_event must be a threading.Event")
        except (KeyError, TypeError, ValueError) as exc:
            cmd.finish_flag(
                ok=False, phase="plan", planned_only=True,
                error=f"invalid observation command: {exc}",
                metrics={"status": "INVALID_REQUEST", "fabrication_authorized": False},
            )
            return

        deadline = time.monotonic() + timeout_s
        guard = threading.Lock()
        state = {"done": False, "sent": False, "handle": None, "timer": None}

        def interrupted():
            if cancel_event is not None and cancel_event.is_set():
                return "CANCELLED", "observation command cancelled by caller"
            if time.monotonic() >= deadline:
                return "TIMED_OUT", "observation action deadline exceeded"
            return None

        def cancel_handle(handle):
            if handle is None:
                return
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning(f"Observation cancellation failed: {exc}")

        def finish(ok, status, message, *, metrics=None, cancel=False):
            with guard:
                if state["done"]:
                    return
                stopped = interrupted()
                if stopped:
                    ok = False
                    status, message = stopped
                    cancel = True
                state["done"] = True
                timer, handle = state["timer"], state["handle"]
                state["timer"] = None
            if timer is not None:
                try:
                    timer.cancel()
                except Exception as exc:
                    self._node.get_logger().warning(f"Observation timer cancel failed: {exc}")
                try:
                    self._node.destroy_timer(timer)
                except Exception as exc:
                    self._node.get_logger().warning(f"Observation timer destroy failed: {exc}")
            if cancel:
                cancel_handle(handle)
            result_metrics = {
                "success": bool(ok), "status": status, "message": message,
                "request_id": request["request_id"], "evaluation_id": "",
                "packet_path": "", "packet_sha256": "",
                "evidence_manifest_path": "", "evidence_manifest_sha256": "",
                "requires_review": True, "fabrication_authorized": False,
            }
            if metrics:
                result_metrics.update(metrics)
            # Keep raw server evidence without presenting a late result as success.
            if metrics and (stopped or status == "INVALID_RESULT"):
                result_metrics["server_result"] = dict(metrics)
                result_metrics.update(success=False, status=status, message=message)
            result_metrics["command_status"] = status
            cmd.finish_flag(
                ok=bool(ok), phase="plan", planned_only=True,
                error=None if ok else message, metrics=result_metrics,
            )

        def on_result(future):
            with guard:
                if state["done"]:
                    return
            try:
                wrapped = future.result()
                result = wrapped.result
                metrics = {
                    name: getattr(result, name)
                    for name in _RESULT_STRING_FIELDS + _RESULT_BOOL_FIELDS
                }
                metrics["ros_goal_status"] = wrapped.status
                for name in _RESULT_STRING_FIELDS:
                    if not isinstance(metrics[name], str):
                        raise ValueError(f"result {name} is not a string")
                for name in _RESULT_BOOL_FIELDS:
                    if type(metrics[name]) is not bool:
                        raise ValueError(f"result {name} is not a bool")
            except Exception as exc:
                finish(False, "INVALID_RESULT", f"invalid observation result: {exc}", cancel=True)
                return
            if metrics["request_id"] != request["request_id"]:
                error = "observation result request_id does not match command"
            elif metrics["status"] not in _SERVER_STATUSES:
                error = "unknown observation result status"
            elif metrics["fabrication_authorized"] or not metrics["requires_review"]:
                error = "observation result must remain review-only and unauthorized"
            elif not metrics["success"] and metrics["status"] != "REVIEW_REQUIRED":
                finish(
                    False, metrics["status"],
                    metrics["message"] or "observation evaluation failed",
                    metrics=metrics, cancel=True,
                )
                return
            elif wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                error = "observation ROS action did not succeed"
            elif not metrics["success"] or metrics["status"] != "REVIEW_REQUIRED":
                failure_status = metrics["status"]
                if metrics["success"] or failure_status == "REVIEW_REQUIRED":
                    failure_status = "INVALID_RESULT"
                finish(
                    False, failure_status,
                    metrics["message"] or "observation evaluation failed",
                    metrics=metrics, cancel=True,
                )
                return
            else:
                finish(True, "REVIEW_REQUIRED", metrics["message"], metrics=metrics)
                return
            finish(False, "INVALID_RESULT", error, metrics=metrics, cancel=True)

        def on_goal(future):
            try:
                handle = future.result()
                if handle is None or not handle.accepted:
                    finish(False, "REJECTED", "observation action goal rejected")
                    return
            except Exception as exc:
                finish(False, "FAILED", f"observation goal response failed: {exc}")
                return
            with guard:
                late = state["done"]
                state["handle"] = handle
            if late:
                cancel_handle(handle)
                return
            stopped = interrupted()
            if stopped:
                finish(False, *stopped, cancel=True)
                return
            try:
                handle.get_result_async().add_done_callback(on_result)
            except Exception as exc:
                finish(False, "FAILED", f"observation result request failed: {exc}", cancel=True)

        def tick():
            with guard:
                if state["done"]:
                    return
            stopped = interrupted()
            if stopped:
                finish(False, *stopped, cancel=True)
                return
            with guard:
                if state["done"] or state["sent"]:
                    return
            try:
                ready = self._action_client.server_is_ready()
            except Exception as exc:
                finish(False, "UNAVAILABLE", f"observation server readiness failed: {exc}")
                return
            if not ready:
                return
            with guard:
                if state["done"] or state["sent"]:
                    return
                state["sent"] = True
            try:
                goal = EvaluateObservation.Goal()
                for name, value in request.items():
                    setattr(goal, name, value)
                self._action_client.send_goal_async(goal).add_done_callback(on_goal)
            except Exception as exc:
                finish(False, "FAILED", f"sending observation goal failed: {exc}", cancel=True)

        try:
            timer = self._node.create_timer(
                min(0.05, timeout_s), tick, clock=self._steady_clock,
            )
            with guard:
                already_done = state["done"]
                if not already_done:
                    state["timer"] = timer
            # Very short timers can run before create_timer returns its handle.
            if already_done:
                timer.cancel()
                self._node.destroy_timer(timer)
                return
        except Exception as exc:
            finish(False, "FAILED", f"observation deadline timer creation failed: {exc}")
            return
        tick()
