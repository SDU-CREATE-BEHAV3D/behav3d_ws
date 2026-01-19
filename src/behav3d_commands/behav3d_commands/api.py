#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Optional, Callable

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from .queue import FifoQueue
from .motion_commands import MotionCommands
from .command import Command, OnCommandDone


class Commands_api:
    def __init__(self, node: Node):
        self._node = node
        self.motion = MotionCommands(node)
        self.queue = FifoQueue(executor=self._execute_item)


        self._motion_plan_jt = None
        self._motion_plan_meta = None
# ---------------- Public API ----------------

    # ---------- MOTION COMMANDS ----------

    def home(self, *, duration_s: float = 10.0, on_done: OnCommandDone = None) -> None:
        jt = self.motion.build_home_trajectory(duration_s=duration_s)
        self.queue.enqueue(
            "follow_traj",
            {
                "jt": jt,
                "cmd_kind": "home",
                "on_done": on_done,
            },
        )

    def plan_motion(
        self,
        *,
        x: float, y: float, z: float,
        eef: str | None = None,
        vel_scale: float | None = None,
        accel_scale: float | None = None,
        exec: bool = False,
        motion: str | None = None,
        on_done: OnCommandDone = None,
    ) -> None:
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0

        self.queue.enqueue(
            "plan_motion",
            {
                "pose": ps,
                "eef": eef,                 # can be None -> motion defaults
                "vel_scale": vel_scale,     # can be None -> motion defaults
                "accel_scale": accel_scale, # can be None -> motion defaults
                "exec": bool(exec),
                "motion": motion,           # can be None -> motion defaults
                "cmd_kind": "goto",         # label like before
                "on_done": on_done,
            },
        )

    def exec_motion(self, *, on_done: OnCommandDone = None) -> None:
        self.queue.enqueue(
            "exec_motion",
            {
                "cmd_kind": "exec_motion",
                "on_done": on_done,
            },
        )

    # ---------- UTILITY COMMANDS ----------

    def pause(self) -> None:
        self.queue.pause()
        self._node.get_logger().warn("[FIFO] Paused.")

    def resume(self) -> None:
        self.queue.resume()
        self._node.get_logger().info("[FIFO] Resumed.")


    def _execute_item(self, kind: str, payload: Dict[str, Any]) -> None:
        cmd = Command(
            kind=payload.get("cmd_kind", "command"),
            on_done=payload.get("on_done", None),
            queue_finish_callback=self.queue.finish_item,
        )

        if kind == "follow_traj":
            self.motion.exec_follow_traj(payload["jt"], cmd=cmd)
            return

        if kind == "plan_motion":
            def _store_plan(jt, meta):
                self._motion_plan_jt = jt
                self._motion_plan_meta = meta

            self.motion.plan_motion(
                ps=payload["pose"],
                eef=payload.get("eef") or self.motion._default_eef,
                vel_scale=payload.get("vel_scale") if payload.get("vel_scale") is not None else self.motion._default_vel_scale,
                accel_scale=payload.get("accel_scale") if payload.get("accel_scale") is not None else self.motion._default_accel_scale,
                motion=payload.get("motion") or self.motion._motion_mode,
                cmd=cmd,
                on_planned=_store_plan,   
            )
            return
        if kind == "exec_motion":
            jt = self._motion_plan_jt
            if jt is None:
                cmd.finish_flag(
                    ok=False,
                    phase="exec",
                    error="no stored motion plan (call plan_motion first)",
                )
                return

            self.motion.exec_follow_traj(jt, cmd=cmd)
            return



        cmd.finish_flag(ok=False, phase="exec", error=f"unknown queue kind: {kind}")