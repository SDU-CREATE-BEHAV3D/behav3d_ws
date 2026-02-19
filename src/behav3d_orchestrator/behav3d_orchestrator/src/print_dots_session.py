#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped

from .yaml_session import YamlSession


class PrintDotsSession(YamlSession):
    """
    Session helpers for dot-printing over YAML targets.
    Keeps YAML parsing in YamlSession and print behavior here.
    """

    def run_print_dots(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
        dot_steps: int = 5000,
        dot_speed: int = 1200,
        approach_z_offset_m: float = 0.40,
        dot_z_offset_m: float = 0.04,
        pre_dot_vel_scale: float = 0.10,
        dot_vel_scale: float = 0.03,
        dwell_s: float = 0.4,
        timeout_s: Optional[float] = None,
        pause_gate: Optional[Callable[[], None]] = None,
        publish_markers: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> dict:
        """
        Dot print behavior:
        1) Parse ordered poses from YAML.
        2) Move to first target + approach_z_offset_m at pre_dot_vel_scale.
        3) For each target:
           - go to target + dot_z_offset_m
           - go to target
           - extrude print_steps
           - wait dwell_s
           - lift back to target + dot_z_offset_m
        """
        targets = self.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
        if len(targets) < 1:
            raise ValueError("Need at least 1 target to print dots.")

        first = targets[0]
        first_approach = self._with_z_offset(first, float(approach_z_offset_m))

        if publish_markers:
            self.run_sync(
                self.util.publish_targets(
                    targets,
                    axis_length=float(axis_length),
                    axis_radius=float(axis_radius),
                    clear_before=bool(clear_markers_before),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )

        try:
            if callable(pause_gate):
                pause_gate()

            res = self.run_sync(
                self.motion.goto(
                    pose=first_approach,
                    motion="LIN",
                    vel_scale=float(pre_dot_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_first", "error": res.get("error", "unknown")}

            printed = 0
            for i, target in enumerate(targets):
                if callable(pause_gate):
                    pause_gate()

                above = self._with_z_offset(target, float(dot_z_offset_m))

                above_vel = float(pre_dot_vel_scale) if i == 0 else float(dot_vel_scale)
                res = self.run_sync(
                    self.motion.goto(
                        pose=above,
                        motion="LIN",
                        vel_scale=above_vel,
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"goto_above_{i}",
                        "error": res.get("error", "unknown"),
                    }

                res = self.run_sync(
                    self.motion.goto(
                        pose=target,
                        motion="LIN",
                        vel_scale=float(dot_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"goto_target_{i}",
                        "error": res.get("error", "unknown"),
                    }

                res = self.run_sync(
                    self.extruder.print_steps(
                        steps=int(dot_steps),
                        speed=int(dot_speed),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"extrude_{i}",
                        "error": res.get("error", "unknown"),
                    }

                if callable(pause_gate):
                    pause_gate()

                if float(dwell_s) > 0.0:
                    res = self.run_sync(
                        self.util.wait(float(dwell_s), enqueue=False),
                        timeout_s=timeout_s,
                    )
                    if not res.get("ok", False):
                        return {
                            "ok": False,
                            "stage": f"wait_{i}",
                            "error": res.get("error", "unknown"),
                        }

                res = self.run_sync(
                    self.motion.goto(
                        pose=above,
                        motion="LIN",
                        vel_scale=float(dot_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"lift_{i}",
                        "error": res.get("error", "unknown"),
                    }

                printed += 1

            return {"ok": True, "stage": "done", "targets": len(targets), "printed": printed}
        finally:
            if publish_markers:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    self.node.get_logger().warn("[print_dots] delete_markers timed out.")

    @staticmethod
    def _with_z_offset(ps: PoseStamped, z_offset_m: float) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = ps.header.frame_id
        out.header.stamp = ps.header.stamp
        out.pose.position.x = float(ps.pose.position.x)
        out.pose.position.y = float(ps.pose.position.y)
        out.pose.position.z = float(ps.pose.position.z) + float(z_offset_m)
        out.pose.orientation.x = float(ps.pose.orientation.x)
        out.pose.orientation.y = float(ps.pose.orientation.y)
        out.pose.orientation.z = float(ps.pose.orientation.z)
        out.pose.orientation.w = float(ps.pose.orientation.w)
        return out
