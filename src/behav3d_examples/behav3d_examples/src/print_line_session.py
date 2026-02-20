#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Optional, Sequence

from geometry_msgs.msg import PoseStamped

from behav3d_commands.session import Session


class PrintLineSession(Session):
    """
    Session helper for line-print behavior between two points.
    Home/motion defaults are intentionally managed by the sequence node.
    """

    def run_print_line(
        self,
        *,
        p1_xyz_m: Sequence[float],
        p2_xyz_m: Sequence[float],
        approach_z_offset_m: float = 0.50,
        extruder_speed: int = 800,
        timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        if len(p1_xyz_m) != 3 or len(p2_xyz_m) != 3:
            raise ValueError("p1_xyz_m and p2_xyz_m must be 3-element sequences: (x, y, z)")

        p1 = self._pose_world(float(p1_xyz_m[0]), float(p1_xyz_m[1]), float(p1_xyz_m[2]))
        p2 = self._pose_world(float(p2_xyz_m[0]), float(p2_xyz_m[1]), float(p2_xyz_m[2]))
        approach = self._pose_world(
            float(p1_xyz_m[0]),
            float(p1_xyz_m[1]),
            float(p1_xyz_m[2]) + float(approach_z_offset_m),
        )
        p2_exit = self._pose_world(
            float(p2_xyz_m[0]),
            float(p2_xyz_m[1]),
            float(p2_xyz_m[2]) + 0.05,
        )

        log = self.node.get_logger()
        approach_vel_scale = 0.1
        line_vel_scale = 0.01
        try:
            pub_res = self.run_sync(
                self.util.publish_targets(
                    [approach, p1, p2, p2_exit],
                    axis_length=0.05,
                    axis_radius=0.003,
                    clear_before=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not pub_res.get("ok", False):
                log.warn("[print_line] publish_targets failed; continuing without markers.")
        except TimeoutError:
            log.warn("[print_line] publish_targets timed out; continuing without markers.")

        try:
            log.info(
                "[print_line] Approach P1: "
                f"({approach.pose.position.x:.4f}, {approach.pose.position.y:.4f}, {approach.pose.position.z:.4f})"
            )
            res = self.run_sync(
                self.motion.goto(
                    pose=approach,
                    vel_scale=approach_vel_scale,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_p1", "error": res.get("error", "unknown")}

            log.info(
                "[print_line] Move to P1: "
                f"({p1.pose.position.x:.4f}, {p1.pose.position.y:.4f}, {p1.pose.position.z:.4f})"
            )
            res = self.run_sync(
                self.motion.goto(
                    pose=p1,
                    vel_scale=line_vel_scale,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "goto_p1", "error": res.get("error", "unknown")}

            log.info(
                "[print_line] Plan to P2: "
                f"({p2.pose.position.x:.4f}, {p2.pose.position.y:.4f}, {p2.pose.position.z:.4f})"
            )
            plan_res = self.run_sync(
                self.motion.plan(
                    pose=p2,
                    vel_scale=line_vel_scale,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not plan_res.get("ok", False):
                return {
                    "ok": False,
                    "stage": "plan_p2",
                    "error": plan_res.get("error", "unknown"),
                    "planned": False,
                }

            log.info(f"[print_line] Plan OK. Starting extruder at speed={int(extruder_speed)} for P1->P2 move.")
            self.run_group(
                [
                    self.motion.exec(enqueue=False),
                    self.extruder.setExtruder(True, speed=int(extruder_speed), enqueue=False),
                ]
            )
            # Barrier for run_group completion.
            self.run_sync(self.util.wait(0.01, enqueue=False), timeout_s=timeout_s)

            stop_res = self.run_sync(
                self.extruder.setExtruder(False, enqueue=False),
                timeout_s=timeout_s,
            )
            if not stop_res.get("ok", False):
                return {
                    "ok": False,
                    "stage": "stop_extruder",
                    "error": stop_res.get("error", "unknown"),
                    "planned": True,
                }

            log.info(
                "[print_line] Exit from P2: "
                f"({p2_exit.pose.position.x:.4f}, {p2_exit.pose.position.y:.4f}, {p2_exit.pose.position.z:.4f})"
            )
            exit_res = self.run_sync(
                self.motion.goto(
                    pose=p2_exit,
                    vel_scale=line_vel_scale,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not exit_res.get("ok", False):
                return {
                    "ok": False,
                    "stage": "goto_p2_exit",
                    "error": exit_res.get("error", "unknown"),
                    "planned": True,
                }

            return {"ok": True, "planned": True, "stage": "done"}
        finally:
            try:
                self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
            except TimeoutError:
                log.warn("[print_line] delete_markers timed out.")

    @staticmethod
    def _pose_world(x: float, y: float, z: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        return ps
