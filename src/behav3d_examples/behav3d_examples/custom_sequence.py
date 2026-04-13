#!/usr/bin/env python3
# Demo: blocking sequencing with run_sync + run_group (queue-backed commands).

from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from .src.custom_session import MySession
from .src.grid_sweep_session import GridSweepSession

class CustomSequenceDemo(Node):
    def __init__(self):
        super().__init__("session_sync_demo")
        self.session = MySession(self)
        self.grid_session = GridSweepSession(self)
        self._folder_fallback_idx = 0
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()
        log.info("Starting loop: grid sweep (12 targets) -> color_to_depth -> tsdf_grid_sweep -> world mesh update.")

        cycle = 0
        while rclpy.ok():
            scan_folder = self._next_scan_folder_name()
            capture_folder = f"@session/{scan_folder}"
            cycle += 1

            log.info(f"[cycle {cycle}] Running capture folder: {capture_folder}")
            sweep_targets = self.grid_session.run_grid_sweep(
                width=0.9,
                height=0.40,
                center_x=0.0,
                center_y=0.8,
                center_z=0.2,
                z_off=0.5,
                nx=10,
                ny=5,
                debug=False,
                capture_folder=capture_folder,
                eef_link="femto_color_optical_calib",
                use_tf_orientation=True,
            )
            log.info(f"[cycle {cycle}] Grid sweep completed. Captured targets: {len(sweep_targets)}")

            if len(sweep_targets) != 12:
                log.warn(f"[cycle {cycle}] Expected 12 targets from grid sweep (4x3). Continuing anyway.")

            try:
                c2d_res = self.session.run_color_to_depth_grid_sweep_reconstruct(
                    use_latest=True,
                    session_path="@session",
                    scan_folder=scan_folder,
                    visualize=False,
                    timeout_s=10.0,
                    wait_for_outputs=True,
                    wait_timeout_s=60.0,
                )
            except TimeoutError:
                log.error(f"[cycle {cycle}] color_to_depth request timed out.")
                rclpy.shutdown()
                return

            if not c2d_res.get("ok", False):
                log.error(f"[cycle {cycle}] color_to_depth failed: {c2d_res.get('error')}")
                rclpy.shutdown()
                return

            try:
                tsdf_res = self.session.run_tsdf_grid_sweep_reconstruct(
                    use_latest=True,
                    session_path="@session",
                    scan_folder=scan_folder,
                    visualize=False,
                    device="CPU:0",
                    timeout_s=10.0,
                    wait_for_outputs=True,
                    wait_timeout_s=180.0,
                )
            except TimeoutError:
                log.error(f"[cycle {cycle}] tsdf_grid_sweep request timed out.")
                rclpy.shutdown()
                return

            if not tsdf_res.get("ok", False):
                log.error(f"[cycle {cycle}] tsdf_grid_sweep failed: {tsdf_res.get('error')}")
                rclpy.shutdown()
                return

            mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
            rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()

            try:
                mesh_res = self.session.run_update_world_mesh(
                    use_latest=True,
                    session_path="@session",
                    mesh_path=mesh_path,
                    ply_path=rgb_ply_path,
                    prefer="mesh",
                    wait_timeout_s=45.0,
                    timeout_s=55.0,
                )
            except TimeoutError:
                log.error(f"[cycle {cycle}] update_world_mesh request timed out.")
                rclpy.shutdown()
                return

            if not mesh_res.get("ok", False):
                log.error(f"[cycle {cycle}] update_world_mesh failed: {mesh_res.get('error')}")
                rclpy.shutdown()
                return

            published_path = mesh_res.get("metrics", {}).get("published_path", "")
            published_kind = mesh_res.get("metrics", {}).get("published_kind", "")
            log.info(f"[cycle {cycle}] World mesh updated ({published_kind}): {published_path}")
            log.info(f"[cycle {cycle}] Pipeline completed for scan folder '{scan_folder}'.")

            if not self._ask_continue():
                break

        rclpy.shutdown()

    def _ask_continue(self) -> bool:
        res = self.session.run_sync(
            self.session.util.input(
                prompt="Type 'c' + ENTER to capture another sequence, or ENTER to stop.",
                enqueue=False,
            )
        )
        value = str(res.get("metrics", {}).get("value", "")).strip().lower()
        return value == "c"

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
    rclpy.spin(CustomSequenceDemo())


if __name__ == "__main__":
    main()
