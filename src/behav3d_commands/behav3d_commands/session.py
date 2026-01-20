#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Callable, Dict, Iterable, Optional

from rclpy.node import Node

from .command import Command, OnCommandDone
from .session_queue import QueueItem, SessionQueue
from .motion_commands import MotionCommands
from .sense_commands import CameraCommands
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
    ):
        self.node = node
        self._router = CommandRouter(node)
        self._queue = SessionQueue(executor=self._execute_item)

        self.motion = MotionCommands(node, controller_action=controller_action)
        self.camera = CameraCommands(node)
        self.extruder = ExtruderCommands(node)
        self.util = UtilCommands(node)

        if register_default_commands:
            self._register_default_commands()

    def _register_default_commands(self) -> None:
        self.motion.register(self._router)
        self.camera.register(self._router)
        self.extruder.register(self._router)
        self.util.register(self._router)

    def build_item(
        self,
        kind: str,
        payload: Dict[str, Any],
        *,
        cmd_kind: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> QueueItem:
        return QueueItem(kind=kind, payload=payload, cmd_kind=cmd_kind, on_done=on_done)

    def enqueue(
        self,
        kind: str,
        payload: Dict[str, Any],
        *,
        cmd_kind: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self._queue.enqueue(self.build_item(kind, payload, cmd_kind=cmd_kind, on_done=on_done))

    def prepend(
        self,
        kind: str,
        payload: Dict[str, Any],
        *,
        cmd_kind: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self._queue.prepend(self.build_item(kind, payload, cmd_kind=cmd_kind, on_done=on_done))

    def run_simultaneously(self, items: Iterable[QueueItem]) -> None:
        self._queue.enqueue_group(items)

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

    def _execute_item(self, item: QueueItem, finish_cb: Callable[[], None]) -> None:
        cmd = Command(
            kind=item.cmd_kind or item.kind,
            on_done=item.on_done,
            queue_finish_callback=finish_cb,
        )
        self._router.dispatch(item.kind, item.payload, cmd)

    # ---------------- Convenience API ----------------

    def home(self, *, duration_s: float = 10.0, on_done: OnCommandDone = None) -> None:
        self.enqueue("home", {"duration_s": float(duration_s)}, cmd_kind="home", on_done=on_done)

    def goto(
        self,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        pose: Any = None,
        rx: Optional[float] = None,
        ry: Optional[float] = None,
        rz: Optional[float] = None,
        eef: Optional[str] = None,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        exec: bool = True,
        motion: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "goto",
            {
                "pose": pose,
                "x": x,
                "y": y,
                "z": z,
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "eef": eef,
                "vel_scale": vel_scale,
                "accel_scale": accel_scale,
                "exec": bool(exec),
                "motion": motion,
            },
            cmd_kind="goto",
            on_done=on_done,
        )

    def plan_motion(
        self,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        pose: Any = None,
        eef: Optional[str] = None,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        motion: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "plan_motion",
            {
                "pose": pose,
                "x": x,
                "y": y,
                "z": z,
                "eef": eef,
                "vel_scale": vel_scale,
                "accel_scale": accel_scale,
                "motion": motion,
            },
            cmd_kind="plan_motion",
            on_done=on_done,
        )

    def exec_motion(self, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("exec_motion", {}, cmd_kind="exec_motion", on_done=on_done)

    def setPTP(self, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("set_motion_mode", {"mode": "PTP"}, cmd_kind="setPTP", on_done=on_done)

    def setLIN(self, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("set_motion_mode", {"mode": "LIN"}, cmd_kind="setLIN", on_done=on_done)

    def setEef(self, name: str, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("set_eef", {"eef": str(name)}, cmd_kind="setEef", on_done=on_done)

    def setSpd(self, val: float, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("set_spd", {"vel_scale": float(val)}, cmd_kind="setSpd", on_done=on_done)

    def setAcc(self, val: float, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("set_acc", {"accel_scale": float(val)}, cmd_kind="setAcc", on_done=on_done)

    def wait(self, secs: float, *, on_done: OnCommandDone = None) -> None:
        self.enqueue("wait", {"secs": float(secs)}, cmd_kind="wait", on_done=on_done)

    def wait_until(
        self,
        *,
        predicate: Callable[[], bool],
        period_s: float = 0.1,
        timeout_s: Optional[float] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "wait_until",
            {"predicate": predicate, "period_s": float(period_s), "timeout_s": timeout_s},
            cmd_kind="wait_until",
            on_done=on_done,
        )

    def input(
        self,
        *,
        key: Optional[str] = None,
        prompt: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "wait_input",
            {
                "key": (None if key is None else str(key)),
                "prompt": prompt,
            },
            cmd_kind="input",
            on_done=on_done,
        )

    def capture(
        self,
        *,
        rgb: bool = False,
        depth: bool = False,
        ir: bool = False,
        pose: bool = False,
        folder: Optional[str] = None,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "capture",
            {
                "rgb": bool(rgb),
                "depth": bool(depth),
                "ir": bool(ir),
                "pose": bool(pose),
                "folder": folder,
            },
            cmd_kind="capture",
            on_done=on_done,
        )

    def get_pose(
        self,
        eef: str,
        base_frame: Optional[str] = "world",
        *,
        use_tf: bool = False,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "get_pose",
            {
                "link": str(eef),
                "base_frame": ("" if base_frame is None else str(base_frame)),
                "use_tf": bool(use_tf),
            },
            cmd_kind="get_pose",
            on_done=on_done,
        )

    def print_time(
        self,
        *,
        secs: float,
        speed: int,
        use_previous_speed: bool = False,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "print_time",
            {"secs": float(secs), "speed": int(speed), "use_prev": bool(use_previous_speed)},
            cmd_kind="print",
            on_done=on_done,
        )

    def print_steps(
        self,
        *,
        steps: int,
        speed: int,
        use_previous_speed: bool = False,
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "print_steps",
            {"steps": int(steps), "speed": int(speed), "use_prev": bool(use_previous_speed)},
            cmd_kind="print_steps",
            on_done=on_done,
        )

    def reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        on_done: OnCommandDone = None,
    ) -> None:
        self.enqueue(
            "reconstruct",
            {"use_latest": bool(use_latest), "session_path": (session_path or "")},
            cmd_kind="reconstruct",
            on_done=on_done,
        )
