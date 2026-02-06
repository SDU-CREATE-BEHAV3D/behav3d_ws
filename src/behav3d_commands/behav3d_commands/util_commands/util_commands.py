#!/usr/bin/env python3
from __future__ import annotations

import sys
import threading
import time
from typing import Any, Callable, Dict, Optional, Sequence

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

try:
    from behav3d_interfaces.srv import PublishTargets, DeleteMarkers
except ImportError:
    # Some merged/partial installs may generate the srv modules but not re-export
    # them in behav3d_interfaces.srv.__init__.py yet.
    from behav3d_interfaces.srv._publish_targets import PublishTargets
    from behav3d_interfaces.srv._delete_markers import DeleteMarkers

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue


class UtilCommands:
    def __init__(self, node: Node, *, queue: Optional[SessionQueue] = None):
        self._queue = queue
        self._node = node
        self._publish_targets_cli = node.create_client(PublishTargets, "/behav3d/publish_targets")
        self._delete_markers_cli = node.create_client(DeleteMarkers, "/behav3d/delete_markers")

    def register(self, router) -> None:
        router.register("wait", self._handle_wait)
        router.register("wait_input", self._handle_wait_input)
        router.register("wait_until", self._handle_wait_until)
        router.register("publish_targets", self._handle_publish_targets)
        router.register("delete_markers", self._handle_delete_markers)

    def _queue_or_item(self, item: QueueItem, *, enqueue: bool):
        if enqueue:
            if self._queue is None:
                raise RuntimeError("UtilCommands requires a SessionQueue to enqueue items.")
            self._queue.enqueue(item)
            return None
        return item

    def wait(
        self,
        secs: float,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "wait",
            {"secs": float(secs)},
            cmd_kind="wait",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def wait_until(
        self,
        *,
        predicate: Callable[[], bool],
        period_s: float = 0.1,
        timeout_s: Optional[float] = None,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "wait_until",
            {"predicate": predicate, "period_s": float(period_s), "timeout_s": timeout_s},
            cmd_kind="wait_until",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def input(
        self,
        *,
        key: Optional[str] = None,
        prompt: Optional[str] = None,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "wait_input",
            {
                "key": (None if key is None else str(key)),
                "prompt": prompt,
            },
            cmd_kind="input",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def publish_targets(
        self,
        targets: Sequence[PoseStamped],
        *,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_before: bool = True,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "publish_targets",
            {
                "targets": list(targets),
                "axis_length": float(axis_length),
                "axis_radius": float(axis_radius),
                "clear_before": bool(clear_before),
            },
            cmd_kind="publish_targets",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def delete_markers(
        self,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "delete_markers",
            {},
            cmd_kind="delete_markers",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def _handle_wait(self, payload: Dict[str, Any], cmd: Command) -> None:
        secs = float(payload.get("secs", 0.0))
        self._node.get_logger().info(f"WAIT: delaying for {secs:.2f} s")

        t = None

        def _one_shot():
            nonlocal t
            if t is not None:
                t.cancel()
                t = None
            cmd.finish_flag(ok=True, phase="exec", metrics={"secs": secs})

        t = self._node.create_timer(secs, _one_shot)

    def _handle_wait_until(self, payload: Dict[str, Any], cmd: Command) -> None:
        predicate = payload.get("predicate")
        if not callable(predicate):
            cmd.finish_flag(ok=False, phase="exec", error="wait_until requires callable predicate")
            return

        period_s = float(payload.get("period_s", 0.1))
        timeout_s = payload.get("timeout_s")
        timeout_s = None if timeout_s is None else float(timeout_s)
        start = time.monotonic()

        t = None

        def _tick():
            nonlocal t
            try:
                if predicate():
                    if t is not None:
                        t.cancel()
                        t = None
                    elapsed = time.monotonic() - start
                    cmd.finish_flag(ok=True, phase="exec", metrics={"elapsed_s": elapsed})
                    return
            except Exception as exc:
                if t is not None:
                    t.cancel()
                    t = None
                cmd.finish_flag(ok=False, phase="exec", error=f"predicate exception: {exc}")
                return

            if timeout_s is not None and (time.monotonic() - start) >= timeout_s:
                if t is not None:
                    t.cancel()
                    t = None
                cmd.finish_flag(ok=False, phase="exec", error="wait_until timeout")

        t = self._node.create_timer(period_s, _tick)

    def _handle_wait_input(self, payload: Dict[str, Any], cmd: Command) -> None:
        key = payload.get("key", None)
        prompt = payload.get("prompt", None)
        if prompt is None:
            if key is None:
                prompt = "Press ENTER to continue..."
            else:
                prompt = f"Type '{key}' + ENTER to continue..."

        if not sys.stdin or not sys.stdin.isatty():
            self._node.get_logger().warn("WAIT_INPUT: no TTY detected; continuing automatically.")
            cmd.finish_flag(
                ok=True,
                phase="exec",
                metrics={"value": "", "reason": "no-tty-autocontinue"},
            )
            return

        self._node.get_logger().info(f"WAIT_INPUT: {prompt}")

        def _reader():
            try:
                text = input()
                value = text.strip()
                if key is not None:
                    ok = (value == str(key))
                    cmd.finish_flag(
                        ok=ok,
                        phase="exec",
                        metrics={"value": value},
                        error=None if ok else f"expected '{key}', got '{value}'",
                    )
                else:
                    cmd.finish_flag(ok=True, phase="exec", metrics={"value": value})
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"stdin exception: {exc}")

        t = threading.Thread(target=_reader, daemon=True)
        t.start()

    def _handle_publish_targets(self, payload: Dict[str, Any], cmd: Command) -> None:
        if not self._publish_targets_cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="publish_targets service not available")
            return

        targets = payload.get("targets", [])
        axis_length = float(payload.get("axis_length", 0.05))
        axis_radius = float(payload.get("axis_radius", 0.003))
        clear_before = bool(payload.get("clear_before", True))

        req = PublishTargets.Request()
        req.targets = list(targets)
        req.axis_length = float(axis_length)
        req.axis_radius = float(axis_radius)
        req.clear_before = bool(clear_before)

        self._node.get_logger().info(
            f"PUBLISH_TARGETS: count={len(req.targets)} clear_before={req.clear_before} "
            f"axis_length={req.axis_length:.3f} axis_radius={req.axis_radius:.3f}"
        )

        fut = self._publish_targets_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = getattr(resp, "message", "")
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={"message": msg},
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)

    def _handle_delete_markers(self, payload: Dict[str, Any], cmd: Command) -> None:
        _ = payload
        if not self._delete_markers_cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="delete_markers service not available")
            return

        req = DeleteMarkers.Request()
        fut = self._delete_markers_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = getattr(resp, "message", "")
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={"message": msg},
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)
