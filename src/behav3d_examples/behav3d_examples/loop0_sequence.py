#!/usr/bin/env python3
"""
Loop0 baseline: scan -> capture -> reconstruct -> pick point -> deposit -> repeat.
Minimal, synchronous, and uses existing ROS interfaces without refactoring.
"""

from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from .src.loop0_session import Loop0Session


class Loop0Node(Node):
    def __init__(self):
        super().__init__("loop0_baseline")

        # Parameters (minimal, but adjustable)
        self.declare_parameter("iterations", -1)  # -1 = forever
        self.declare_parameter("scan_pose", [0.0, 0.70, 0.50])
        self.declare_parameter("bbox_min", [-0.30, 0.45, -0.05])
        self.declare_parameter("bbox_max", [0.30, 1.10, 0.60])
        self.declare_parameter("ground_offset_m", 0.000)   # m above ground
        self.declare_parameter("safety_offset_mm", 15.0)    # safety Z offset in mm
        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("accel_scale", 0.05)
        self.declare_parameter("motion_timeout_s", 15.0)
        self.declare_parameter("capture_timeout_s", 10.0)
        self.declare_parameter("reconstruct_timeout_s", 5.0)
        self.declare_parameter("print_timeout_s", 10.0)
        self.declare_parameter("print_steps", 1500)
        self.declare_parameter("print_speed", 600)
        self.declare_parameter("reconstruct_points_frame", "ur20_base_link")

        self.session = Loop0Session(self)
        self._folder_fallback_idx = 0

        self._worker = threading.Thread(target=self._run_loop, daemon=True)
        self._worker.start()

    # ---------------- Helpers ----------------
    def _param_list(self, name: str, length: int) -> list:
        val = self.get_parameter(name).get_parameter_value()
        if val.string_array_value:
            arr = [float(x) for x in val.string_array_value]
        else:
            arr = [float(x) for x in val.double_array_value]
        if len(arr) != length:
            raise ValueError(f"Parameter '{name}' must have {length} elements.")
        return arr

    # ---------------- Loop ----------------
    def _run_loop(self):
        iterations = int(self.get_parameter("iterations").value)
        scan_pose = self._param_list("scan_pose", 3)

        # Optional default motion config    
        self.session.run_sync(self.session.motion.home(enqueue=False))
        self.session.run_sync(self.session.motion.setSpd(float(self.get_parameter("vel_scale").value), enqueue=False))
        self.session.run_sync(self.session.motion.setAcc(float(self.get_parameter("accel_scale").value), enqueue=False))
        self.session.run_sync(self.session.motion.setEef("extruder_tcp", enqueue=False))
        self.session.run_sync(self.session.motion.setLIN(enqueue=False))

        cycle = 0
        while rclpy.ok() and (iterations < 0 or cycle < iterations):
            self.get_logger().info(f"[Loop0] Cycle {cycle} start")
            scan_folder = self._next_scan_folder_name()
            capture_folder = f"@session/{scan_folder}"
            self.get_logger().info(f"[Loop0] Using capture folder: {capture_folder}")

            ok = self.session.run_loop0_cycle(
                scan_pose=scan_pose,
                bbox_min=self._param_list("bbox_min", 3),
                bbox_max=self._param_list("bbox_max", 3),
                capture_folder=capture_folder,
                ground_offset_m=float(self.get_parameter("ground_offset_m").value),
                safety_offset_mm=float(self.get_parameter("safety_offset_mm").value),
                motion_timeout_s=float(self.get_parameter("motion_timeout_s").value),
                capture_timeout_s=float(self.get_parameter("capture_timeout_s").value),
                reconstruct_timeout_s=float(self.get_parameter("reconstruct_timeout_s").value),
                print_timeout_s=float(self.get_parameter("print_timeout_s").value),
                print_steps=int(self.get_parameter("print_steps").value),
                print_speed=int(self.get_parameter("print_speed").value),
                reconstruct_points_frame=str(self.get_parameter("reconstruct_points_frame").value),
            )
            if not ok:
                self.get_logger().warn("[Loop0] Cycle failed; retrying next cycle.")
            cycle += 1

    def _next_scan_folder_name(self) -> str:
        session_dir = self._resolve_active_session_dir()
        if session_dir is not None and session_dir.exists():
            idx = 0
            while True:
                name = f"grid_sweep_{idx:02d}"
                if not (session_dir / name).exists():
                    return name
                idx += 1

        name = f"grid_sweep_{self._folder_fallback_idx:02d}"
        self._folder_fallback_idx += 1
        return name

    @staticmethod
    def _resolve_active_session_dir() -> Optional[Path]:
        active = os.environ.get("BEHAV3D_ACTIVE_SESSION", "").strip()
        if active:
            p = Path(active).expanduser()
            if p.exists():
                return p.resolve()

        root_env = os.environ.get("BEHAV3D_CAPTURES_ROOT", "").strip()
        captures_root = Path(root_env).expanduser() if root_env else (Path.home() / "behav3d_ws" / "captures")
        if not captures_root.exists():
            return None

        subdirs = [d for d in captures_root.iterdir() if d.is_dir()]
        if not subdirs:
            return None
        return max(subdirs, key=lambda d: d.stat().st_mtime).resolve()


def main(args=None):
    rclpy.init(args=args)
    node = Loop0Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
