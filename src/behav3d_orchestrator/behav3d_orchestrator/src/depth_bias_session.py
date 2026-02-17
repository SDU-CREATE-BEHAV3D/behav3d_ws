#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import behav3d_commands
from geometry_msgs.msg import PoseStamped


class DepthBiasSession(behav3d_commands.Session):
    """
    Session helper for depth-bias data collection at fixed XY and multiple heights.
    """

    def run_depth_bias_capture(
        self,
        *,
        center_x_mm: float,
        center_y_mm: float,
        center_z_mm: float,
        first_height_offset_m: float = 0.50,
        height_step_m: float = 0.30,
        num_steps: int = 3,
        captures_per_height: int = 15,
        folder_root: str = "@session/depth_bias",
        eef_link: str = "femto_ir_optical_calib",
        frame_id: str = "world",
        vel_scale: float = 0.05,
        accel_scale: float = 0.05,
        motion_timeout_s: Optional[float] = 30.0,
        capture_timeout_s: Optional[float] = 8.0,
        settle_s: float = 1.00,
        capture_wait_s: float = 0.50,
        home_before: bool = True,
        home_after: bool = False,
    ) -> Dict[str, object]:
        log = self.node.get_logger()
        if int(captures_per_height) <= 0:
            raise ValueError("captures_per_height must be > 0")
        if int(num_steps) <= 0:
            raise ValueError("num_steps must be > 0")

        cx = float(center_x_mm) / 1000.0
        cy = float(center_y_mm) / 1000.0
        cz = float(center_z_mm) / 1000.0

        heights_m = [
            cz + float(first_height_offset_m) + float(i) * float(height_step_m)
            for i in range(int(num_steps))
        ]

        if home_before:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=motion_timeout_s)

        self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=motion_timeout_s)
        self.run_sync(self.motion.setSpd(float(vel_scale), enqueue=False), timeout_s=motion_timeout_s)
        self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=motion_timeout_s)
        self.run_sync(self.motion.setEef(str(eef_link), enqueue=False), timeout_s=motion_timeout_s)

        qx, qy, qz, qw = self._get_current_eef_orientation(
            eef_link=str(eef_link),
            frame_id=str(frame_id or "world"),
            timeout_s=motion_timeout_s,
        )

        total_ok = 0
        total_fail = 0
        per_height: List[Dict[str, object]] = []

        for i, z in enumerate(heights_m, start=1):
            name = f"h{i}"
            folder = self._height_folder(folder_root, name)

            target = PoseStamped()
            target.header.frame_id = str(frame_id or "world")
            target.pose.position.x = float(cx)
            target.pose.position.y = float(cy)
            target.pose.position.z = float(z)
            target.pose.orientation.x = float(qx)
            target.pose.orientation.y = float(qy)
            target.pose.orientation.z = float(qz)
            target.pose.orientation.w = float(qw)

            log.info(
                f"[depth_bias] Moving to {name}: "
                f"x={target.pose.position.x:.3f} y={target.pose.position.y:.3f} z={target.pose.position.z:.3f}"
            )
            move_res = self.run_sync(
                self.motion.goto(
                    pose=target,
                    motion="LIN",
                    vel_scale=float(vel_scale),
                    accel_scale=float(accel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=motion_timeout_s,
            )
            if not move_res.get("ok", False):
                err = str(move_res.get("error", "unknown")).strip()
                raise RuntimeError(f"Move to {name} failed: {err}")

            if float(settle_s) > 0.0:
                self.run_sync(self.util.wait(float(settle_s), enqueue=False), timeout_s=motion_timeout_s)

            ok_count = 0
            fail_count = 0
            for j in range(int(captures_per_height)):
                folder_arg = folder if j == 0 else None
                cap_res = self.run_sync(
                    self.camera.capture(
                        depth=True,
                        pose=True,
                        folder=folder_arg,
                        enqueue=False,
                    ),
                    timeout_s=capture_timeout_s,
                )
                if cap_res.get("ok", False):
                    ok_count += 1
                else:
                    fail_count += 1
                    log.warn(
                        f"[depth_bias] Capture failed at {name} ({j + 1}/{captures_per_height}): "
                        f"{cap_res.get('error', 'unknown')}"
                    )
                if float(capture_wait_s) > 0.0:
                    self.run_sync(self.util.wait(float(capture_wait_s), enqueue=False), timeout_s=motion_timeout_s)

            total_ok += ok_count
            total_fail += fail_count
            per_height.append(
                {
                    "name": name,
                    "folder": folder,
                    "target_z_m": float(z),
                    "captures_total": int(captures_per_height),
                    "captures_ok": int(ok_count),
                    "captures_fail": int(fail_count),
                }
            )

        if home_after:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=motion_timeout_s)

        return {
            "ok": True,
            "frame_id": str(frame_id or "world"),
            "eef_link": str(eef_link),
            "center_xyz_m": [float(cx), float(cy), float(cz)],
            "first_height_offset_m": float(first_height_offset_m),
            "height_step_m": float(height_step_m),
            "num_steps": int(num_steps),
            "captures_per_height": int(captures_per_height),
            "heights": per_height,
            "captures_ok": int(total_ok),
            "captures_fail": int(total_fail),
        }

    @staticmethod
    def _height_folder(folder_root: str, name: str) -> str:
        root = str(folder_root or "").strip()
        if not root:
            return str(name)
        return f"{root.rstrip('/')}/{name}"

    def _get_current_eef_orientation(
        self,
        *,
        eef_link: str,
        frame_id: str,
        timeout_s: Optional[float],
    ) -> Tuple[float, float, float, float]:
        log = self.node.get_logger()
        try:
            res = self.run_sync(
                self.camera.get_pose(
                    eef=str(eef_link),
                    base_frame=str(frame_id or "world"),
                    use_tf=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if res.get("ok", False) and ("pose" in res):
                q = res["pose"].pose.orientation
                return (float(q.x), float(q.y), float(q.z), float(q.w))
        except TimeoutError:
            log.warn("[depth_bias] get_pose timeout; fallback to identity orientation.")

        log.warn("[depth_bias] Could not read current EEF orientation; using identity quaternion.")
        return (0.0, 0.0, 0.0, 1.0)
