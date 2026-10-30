"""Opt-in initial-wall preview through Session/QueueItem/async ROS actions."""

# ruff: noqa: BLE001, S110 -- async cleanup must always finish the queue item
from __future__ import annotations

import math
import threading
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType

from behav3d_commands.queue import QueueItem
from behav3d_interfaces.action import InitialWall

FIELDS = (
    "phase",
    "request_id",
    "artifact_id",
    "frame_id",
    "scene_manifest_path",
    "scene_manifest_sha256",
    "task_path",
    "task_sha256",
    "proposal_path",
    "proposal_sha256",
)
RESULT_STRINGS = (
    "status",
    "message",
    "request_id",
    "proposal_path",
    "proposal_sha256",
    "batch_manifest_path",
    "batch_manifest_sha256",
)


class InitialWallCommands:
    def __init__(self, node, *, queue=None, action_name="/behav3d/startup/plan_wall"):
        self.node, self.queue = node, queue
        self.client = ActionClient(node, InitialWall, action_name)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)

    def register(self, router):
        router.register("initial_wall", self._handle)

    def initial_wall(
        self, request, *, timeout_s=120.0, cancel_event=None, enqueue=True, on_done=None
    ):
        if (
            type(request) is not dict
            or set(request) != set(FIELDS)
            or any(not isinstance(value, str) for value in request.values())
            or request["phase"] not in {"PROPOSE", "GENERATE"}
        ):
            raise ValueError("Exact typed initial-wall wire request required")
        if (
            type(timeout_s) not in (int, float)
            or not math.isfinite(timeout_s)
            or timeout_s <= 0
            or (
                cancel_event is not None
                and not isinstance(cancel_event, threading.Event)
            )
        ):
            raise ValueError("Finite timeout and optional cancellation event required")
        item = QueueItem(
            "initial_wall",
            {
                "request": dict(request),
                "timeout_s": timeout_s,
                "cancel_event": cancel_event,
            },
            cmd_kind="initial_wall",
            on_done=on_done,
        )
        if not enqueue:
            return item
        if self.queue is None:
            raise RuntimeError("A SessionQueue is required")
        self.queue.enqueue(item)

    def _handle(self, payload, command):
        request, timeout, event = (
            payload["request"],
            payload["timeout_s"],
            payload.get("cancel_event"),
        )
        deadline, lock = time.monotonic() + timeout, threading.RLock()
        state = {"done": False, "sent": False, "timer": None, "handle": None}

        def finish(ok, status, message, metrics=None):
            with lock:
                if state["done"]:
                    return
                if event is not None and event.is_set():
                    ok, status, message, metrics = (
                        False,
                        "CANCELLED",
                        "Startup command cancelled",
                        None,
                    )
                elif time.monotonic() >= deadline:
                    ok, status, message, metrics = (
                        False,
                        "TIMED_OUT",
                        "Startup action deadline exceeded",
                        None,
                    )
                state["done"] = True
                timer, handle = state["timer"], state["handle"]
            if timer is not None:
                try:
                    timer.cancel()
                    self.node.destroy_timer(timer)
                except Exception:
                    # Executor/context shutdown must not strand SessionQueue.
                    pass
            if not ok and handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            values = (
                {
                    "success": False,
                    "status": status,
                    "message": message,
                    "request_id": request["request_id"],
                    "proposal_path": "",
                    "proposal_sha256": "",
                    "batch_manifest_path": "",
                    "batch_manifest_sha256": "",
                    "requires_review": True,
                    "fabrication_authorized": False,
                }
                if metrics is None
                else metrics
            )
            command.finish_flag(
                ok=ok,
                phase="plan",
                planned_only=True,
                error=None if ok else message,
                metrics=values,
            )

        def result_ready(future):
            try:
                wrapped = future.result()
                result = wrapped.result
                values = {
                    name: getattr(result, name)
                    for name in RESULT_STRINGS
                    + ("success", "requires_review", "fabrication_authorized")
                }
                if (
                    any(not isinstance(values[name], str) for name in RESULT_STRINGS)
                    or any(
                        type(values[name]) is not bool
                        for name in (
                            "success",
                            "requires_review",
                            "fabrication_authorized",
                        )
                    )
                    or values["request_id"] != request["request_id"]
                    or values["requires_review"] is not True
                    or values["fabrication_authorized"] is not False
                ):
                    raise ValueError("Startup result identity or safety mismatch")
                expected = (
                    "AWAITING_PLACEMENT_REVIEW"
                    if request["phase"] == "PROPOSE"
                    else "BATCH_PREVIEW_REVIEW_REQUIRED"
                )
                ok = (
                    values["success"]
                    and values["status"] == expected
                    and wrapped.status == GoalStatus.STATUS_SUCCEEDED
                )
                if values["success"] and not ok:
                    raise ValueError("Inconsistent startup success")
                finish(ok, values["status"], values["message"], values)
            except Exception:
                finish(False, "FAILED", "Invalid startup action result")

        def accepted(future):
            try:
                handle = future.result()
                if not handle.accepted:
                    finish(False, "REJECTED", "Startup backend rejected goal")
                    return
                with lock:
                    state["handle"] = handle
                    late = state["done"]
                if late:
                    handle.cancel_goal_async()
                    return
                handle.get_result_async().add_done_callback(result_ready)
            except Exception:
                finish(False, "FAILED", "Startup action acceptance failed")

        def tick():
            with lock:
                if state["done"]:
                    return
            if (event is not None and event.is_set()) or time.monotonic() >= deadline:
                finish(False, "TIMED_OUT", "Startup command interrupted")
                return
            try:
                with lock:
                    if (
                        state["done"]
                        or state["sent"]
                        or not self.client.server_is_ready()
                    ):
                        return
                    state["sent"] = True
                goal = InitialWall.Goal()
                for name, value in request.items():
                    setattr(goal, name, value)
                self.client.send_goal_async(goal).add_done_callback(accepted)
            except Exception:
                finish(False, "FAILED", "Startup goal dispatch failed")

        try:
            timer = self.node.create_timer(min(0.05, timeout), tick, clock=self.clock)
            with lock:
                late = state["done"]
                if not late:
                    state["timer"] = timer
            if late:
                timer.cancel()
                self.node.destroy_timer(timer)
            else:
                tick()
        except Exception:
            finish(False, "FAILED", "Startup timer failed")
