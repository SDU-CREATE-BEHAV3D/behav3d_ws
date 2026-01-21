#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Optional

from rclpy.node import Node
from rclpy.action import ActionClient

from behav3d_interfaces.action import PrintTime, PrintSteps

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue


class ExtruderCommands:
    def __init__(self, node: Node, *, queue: Optional[SessionQueue] = None):
        self._queue = queue
        self._node = node
        self._print_ac = ActionClient(node, PrintTime, "print")
        self._print_steps_ac = ActionClient(node, PrintSteps, "print_steps")

    def register(self, router) -> None:
        router.register("print_time", self._handle_print_time)
        router.register("print_steps", self._handle_print_steps)
        router.register("print_time_delayed", self._handle_print_time_delayed)
        router.register("print_steps_delayed", self._handle_print_steps_delayed)

    def _queue_or_item(self, item: QueueItem, *, enqueue: bool):
        if enqueue:
            if self._queue is None:
                raise RuntimeError("ExtruderCommands requires a SessionQueue to enqueue items.")
            self._queue.enqueue(item)
            return None
        return item

    def print_time(
        self,
        *,
        secs: float,
        speed: int,
        offset_s: float = 0.0,
        use_previous_speed: bool = False,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        kind = "print_time_delayed" if float(offset_s) > 0.0 else "print_time"
        item = QueueItem(
            kind,
            {
                "secs": float(secs),
                "speed": int(speed),
                "use_prev": bool(use_previous_speed),
                "offset_s": float(offset_s),
            },
            cmd_kind="print",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def print_steps(
        self,
        *,
        steps: int,
        speed: int,
        offset_s: float = 0.0,
        use_previous_speed: bool = False,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        kind = "print_steps_delayed" if float(offset_s) > 0.0 else "print_steps"
        item = QueueItem(
            kind,
            {
                "steps": int(steps),
                "speed": int(speed),
                "use_prev": bool(use_previous_speed),
                "offset_s": float(offset_s),
            },
            cmd_kind="print_steps",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def _handle_print_time(self, payload: Dict[str, Any], cmd: Command) -> None:
        secs = float(payload.get("secs", 0.0))
        speed = int(payload.get("speed", 0))
        use_prev = bool(payload.get("use_prev", False))

        if not self._print_ac.wait_for_server(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="print action server not available")
            return

        goal = PrintTime.Goal()
        goal.duration.sec = int(secs)
        goal.duration.nanosec = int((secs - int(secs)) * 1e9)
        goal.speed = int(speed)
        goal.use_previous_speed = bool(use_prev)

        self._node.get_logger().info(
            f"PRINT: sending goal secs={secs:.2f} speed={speed} use_prev={use_prev}"
        )
        fut = self._print_ac.send_goal_async(goal)

        def _on_goal_response(gf):
            gh = gf.result()
            if not gh or not gh.accepted:
                cmd.finish_flag(ok=False, phase="exec", error="print goal rejected")
                return
            res_fut = gh.get_result_async()
            res_fut.add_done_callback(_on_result)

        def _on_result(rf):
            try:
                wrap = rf.result()
                result = wrap.result
                status = wrap.status
                ok = bool(result.success)
                metrics = {
                    "status": int(status),
                    "elapsed_ms": int(result.elapsed_ms),
                    "reason": str(result.reason),
                }
                cmd.finish_flag(
                    ok=ok,
                    phase="exec",
                    metrics=metrics,
                    error=None if ok else result.reason,
                )
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception waiting result: {exc}")

        fut.add_done_callback(_on_goal_response)

    def _handle_print_steps(self, payload: Dict[str, Any], cmd: Command) -> None:
        steps = int(payload.get("steps", 0))
        speed = int(payload.get("speed", 0))
        use_prev = bool(payload.get("use_prev", False))

        if not self._print_steps_ac.wait_for_server(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="print_steps action server not available")
            return

        goal = PrintSteps.Goal()
        goal.steps = int(steps)
        goal.speed = int(speed)
        goal.use_previous_speed = bool(use_prev)

        self._node.get_logger().info(
            f"PRINT_STEPS: sending goal steps={steps} speed={speed} use_prev={use_prev}"
        )
        fut = self._print_steps_ac.send_goal_async(goal)

        def _on_goal_response(gf):
            gh = gf.result()
            if not gh or not gh.accepted:
                cmd.finish_flag(ok=False, phase="exec", error="print_steps goal rejected")
                return
            res_fut = gh.get_result_async()
            res_fut.add_done_callback(_on_result)

        def _on_result(rf):
            try:
                wrap = rf.result()
                result = wrap.result
                status = wrap.status
                ok = bool(result.success)
                metrics = {
                    "status": int(status),
                    "accepted_steps": int(result.accepted_steps),
                    "reason": str(result.reason),
                }
                cmd.finish_flag(
                    ok=ok,
                    phase="exec",
                    metrics=metrics,
                    error=None if ok else result.reason,
                )
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception waiting result: {exc}")

        fut.add_done_callback(_on_goal_response)

    def _handle_print_time_delayed(self, payload: Dict[str, Any], cmd: Command) -> None:
        offset_s = float(payload.get("offset_s", 0.0))
        if offset_s <= 0.0:
            self._handle_print_time(payload, cmd)
            return

        t = None

        def _start():
            nonlocal t
            if t is not None:
                t.cancel()
                t = None
            self._handle_print_time(payload, cmd)

        self._node.get_logger().info(f"PRINT: delaying {offset_s:.2f} s before start")
        t = self._node.create_timer(offset_s, _start)

    def _handle_print_steps_delayed(self, payload: Dict[str, Any], cmd: Command) -> None:
        offset_s = float(payload.get("offset_s", 0.0))
        if offset_s <= 0.0:
            self._handle_print_steps(payload, cmd)
            return

        t = None

        def _start():
            nonlocal t
            if t is not None:
                t.cancel()
                t = None
            self._handle_print_steps(payload, cmd)

        self._node.get_logger().info(f"PRINT_STEPS: delaying {offset_s:.2f} s before start")
        t = self._node.create_timer(offset_s, _start)
