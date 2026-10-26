#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable, Dict, Iterable, Optional

from rclpy.node import Node

from .command import Command
from .queue import QueueItem, SessionQueue
from .motion_commands import MotionCommands
from .sense_commands import CameraCommands, FieldCommands
from .print_commands import ExtruderCommands
from .util_commands import UtilCommands


CommandHandler = Callable[[Dict[str, Any], Command], None]


class CommandRouter:
    def __init__(self, node: Node):
        self._node = node
        self._handlers: Dict[str, CommandHandler] = {}

    def register(self, kind: str, handler: CommandHandler) -> None:
        self._handlers[kind] = handler

    def dispatch(self, kind: str, payload: Dict[str, Any], cmd: Command) -> None:
        handler = self._handlers.get(kind)
        if handler is None:
            cmd.finish_flag(ok=False, phase="exec", error=f"unknown command kind '{kind}'")
            return
        try:
            handler(payload, cmd)
        except Exception as exc:
            self._node.get_logger().error(f"Command '{kind}' failed: {exc}")
            cmd.finish_flag(ok=False, phase="exec", error=str(exc))


class Session:
    """
    Orchestrates queued command execution across subsystem command sets.
    """

    def __init__(
        self,
        node: Node,
        *,
        controller_action: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        register_default_commands: bool = True,
        enabled_command_sets: Optional[Iterable[str]] = None,
    ):
        self.node = node
        self._router = CommandRouter(node)
        self._queue = SessionQueue(executor=self._execute_item)

        allowed = {'motion', 'camera', 'field', 'extruder', 'util'}
        enabled = allowed if enabled_command_sets is None else set(enabled_command_sets)
        if not enabled <= allowed:
            raise ValueError('Unknown command set: ' + ', '.join(sorted(enabled - allowed)))
        # Empty selection is an explicit evidence-only session: no hardware clients.
        self.motion = (MotionCommands(node, queue=self._queue, controller_action=controller_action)
                       if 'motion' in enabled else None)
        self.camera = CameraCommands(node, queue=self._queue) if 'camera' in enabled else None
        self.field = FieldCommands(node, queue=self._queue) if 'field' in enabled else None
        self.extruder = ExtruderCommands(node, queue=self._queue) if 'extruder' in enabled else None
        self.util = UtilCommands(node, queue=self._queue) if 'util' in enabled else None

        if register_default_commands:
            self._register_default_commands()

    def _register_default_commands(self) -> None:
        for commands in (self.motion, self.camera, self.field, self.extruder, self.util):
            if commands is not None:
                commands.register(self._router)

    def run_group(self, items: Iterable[QueueItem]) -> None:
        items_list = list(items)
        if any(item is None for item in items_list):
            raise ValueError("run_group received None; use enqueue=False to build items.")
        self._queue.enqueue_group(items_list)

    def run_sync(
        self,
        item: QueueItem,
        *,
        timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Enqueue a command and block until its on_done fires.

        NOTE: Do not call this from the ROS executor thread; use a worker thread
        or a multi-threaded executor.
        """
        import threading

        if item is None:
            raise ValueError("run_sync received None; use enqueue=False to build an item.")
        done = threading.Event()
        result: Dict[str, Any] = {}
        user_cb = item.on_done

        def _on_done(res: Dict[str, Any]) -> None:
            try:
                if user_cb:
                    user_cb(res)
            finally:
                result["res"] = res
                done.set()

        item_sync = replace(item, on_done=_on_done)
        self._queue.enqueue(item_sync)
        if not done.wait(timeout_s):
            raise TimeoutError("command timed out")
        return result["res"]

    def pause(self) -> None:
        self._queue.pause()
        self.node.get_logger().warn("[SESSION] Paused.")

    def resume(self) -> None:
        self._queue.resume()
        self.node.get_logger().info("[SESSION] Resumed.")

    @property
    def is_busy(self) -> bool:
        return self._queue.is_busy

    @property
    def is_paused(self) -> bool:
        return self._queue.is_paused

    @property
    def queue(self) -> SessionQueue:
        return self._queue

    def _execute_item(self, item: QueueItem, finish_cb: Callable[[], None]) -> None:
        cmd = Command(
            kind=item.cmd_kind or item.kind,
            on_done=item.on_done,
            queue_finish_callback=finish_cb,
        )
        self._router.dispatch(item.kind, item.payload, cmd)
