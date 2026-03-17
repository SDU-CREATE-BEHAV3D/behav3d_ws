#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_field_sequence
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import behav3d_commands
import rclpy
from behav3d_examples.src.grid_sweep_session import GridSweepSession
from rclpy.node import Node


class PrintFieldSequenceNode(Node):
    def __init__(self):
        super().__init__("print_field_sequence")

        # Global flow
        self.declare_parameter("home_before_scan", True)
        self.declare_parameter("home_after", False)
        self.declare_parameter("timeout_s", 0.0)

        # Grid sweep macro params
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("scan_eef_link", "femto_color_optical_calib")
        self.declare_parameter("scan_use_tf_orientation", True)
        self.declare_parameter("scan_width", 0.90)
        self.declare_parameter("scan_height", 0.60)
        self.declare_parameter("scan_center_x", 0.0)
        self.declare_parameter("scan_center_y", 0.80)
        self.declare_parameter("scan_center_z", 0.0)
        self.declare_parameter("scan_z_off", 0.60)
        self.declare_parameter("scan_nx", 9)
        self.declare_parameter("scan_ny", 6)
        self.declare_parameter("scan_row_major", False)
        self.declare_parameter("scan_capture_folder", "@session/grid_sweep")
        self.declare_parameter("scan_debug_prompt", False)
        self.declare_parameter("scan_vel_scale", 0.05)
        self.declare_parameter("scan_accel_scale", 0.05)
        self.declare_parameter("scan_publish_markers", True)
        self.declare_parameter("scan_axis_length", 0.05)
        self.declare_parameter("scan_axis_radius", 0.003)
        self.declare_parameter("scan_clear_markers_before", True)

        # Reconstruction + world mesh
        self.declare_parameter("run_reconstruction", True)
        self.declare_parameter("scan_folder", "grid_sweep")
        self.declare_parameter("reconstruct_device", "CPU:0")
        self.declare_parameter("reconstruct_request_timeout_s", 10.0)
        self.declare_parameter("wait_reconstruction_outputs", True)
        self.declare_parameter("color_to_depth_wait_timeout_s", 60.0)
        self.declare_parameter("tsdf_wait_timeout_s", 180.0)
        self.declare_parameter("mesh_prefer", "mesh")
        self.declare_parameter("mesh_update_wait_timeout_s", 45.0)
        self.declare_parameter("mesh_update_request_timeout_s", 55.0)

        # Field init command
        self.declare_parameter("run_field_init", True)
        self.declare_parameter("field_use_latest", True)
        self.declare_parameter("field_session_path", "@session")
        self.declare_parameter("field_scan_mesh_path", "")
        self.declare_parameter("field_mesh_path", "/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
        self.declare_parameter("field_state_output_dir", "@session/field_loop/init")
        self.declare_parameter("field_request_timeout_s", 120.0)

        # Same node, two sessions: macro + direct commands.
        self.session = behav3d_commands.Session(self)
        self.grid_macro = GridSweepSession(self)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        home_before_scan = bool(self.get_parameter("home_before_scan").value)
        home_after = bool(self.get_parameter("home_after").value)
        timeout_s = self._param_optional_float("timeout_s")

        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        scan_eef_link = str(self.get_parameter("scan_eef_link").value).strip() or "femto_color_optical_calib"
        scan_use_tf_orientation = bool(self.get_parameter("scan_use_tf_orientation").value)
        scan_width = float(self.get_parameter("scan_width").value)
        scan_height = float(self.get_parameter("scan_height").value)
        scan_center_x = float(self.get_parameter("scan_center_x").value)
        scan_center_y = float(self.get_parameter("scan_center_y").value)
        scan_center_z = float(self.get_parameter("scan_center_z").value)
        scan_z_off = float(self.get_parameter("scan_z_off").value)
        scan_nx = int(self.get_parameter("scan_nx").value)
        scan_ny = int(self.get_parameter("scan_ny").value)
        scan_row_major = bool(self.get_parameter("scan_row_major").value)
        scan_capture_folder = str(self.get_parameter("scan_capture_folder").value).strip() or "@session/grid_sweep"
        scan_debug_prompt = bool(self.get_parameter("scan_debug_prompt").value)
        scan_vel_scale = float(self.get_parameter("scan_vel_scale").value)
        scan_accel_scale = float(self.get_parameter("scan_accel_scale").value)
        scan_publish_markers = bool(self.get_parameter("scan_publish_markers").value)
        scan_axis_length = float(self.get_parameter("scan_axis_length").value)
        scan_axis_radius = float(self.get_parameter("scan_axis_radius").value)
        scan_clear_markers_before = bool(self.get_parameter("scan_clear_markers_before").value)

        run_reconstruction = bool(self.get_parameter("run_reconstruction").value)
        scan_folder = str(self.get_parameter("scan_folder").value).strip() or "grid_sweep"
        reconstruct_device = str(self.get_parameter("reconstruct_device").value).strip() or "CPU:0"
        reconstruct_request_timeout_s = float(self.get_parameter("reconstruct_request_timeout_s").value)
        wait_reconstruction_outputs = bool(self.get_parameter("wait_reconstruction_outputs").value)
        color_to_depth_wait_timeout_s = float(self.get_parameter("color_to_depth_wait_timeout_s").value)
        tsdf_wait_timeout_s = float(self.get_parameter("tsdf_wait_timeout_s").value)
        mesh_prefer = str(self.get_parameter("mesh_prefer").value).strip() or "mesh"
        mesh_update_wait_timeout_s = float(self.get_parameter("mesh_update_wait_timeout_s").value)
        mesh_update_request_timeout_s = float(self.get_parameter("mesh_update_request_timeout_s").value)

        run_field_init = bool(self.get_parameter("run_field_init").value)
        field_use_latest = bool(self.get_parameter("field_use_latest").value)
        field_session_path = str(self.get_parameter("field_session_path").value).strip()
        field_scan_mesh_path = str(self.get_parameter("field_scan_mesh_path").value).strip()
        field_mesh_path = str(self.get_parameter("field_mesh_path").value).strip()
        field_state_output_dir = str(self.get_parameter("field_state_output_dir").value).strip()
        field_request_timeout_s = float(self.get_parameter("field_request_timeout_s").value)

        log.info(
            "Starting print_field sequence: "
            f"home_before_scan={home_before_scan}, grid=({scan_nx}x{scan_ny}), "
            f"capture_folder='{scan_capture_folder}', run_reconstruction={run_reconstruction}, "
            f"run_field_init={run_field_init}"
        )

        mesh_path = ""
        rgb_ply_path = ""
        try:
            if home_before_scan:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )

            targets = self.grid_macro.run_grid_sweep(
                width=scan_width,
                height=scan_height,
                center_x=scan_center_x,
                center_y=scan_center_y,
                center_z=scan_center_z,
                z_off=scan_z_off,
                nx=scan_nx,
                ny=scan_ny,
                row_major=scan_row_major,
                frame_id=frame_id,
                eef_link=scan_eef_link,
                use_tf_orientation=scan_use_tf_orientation,
                debug=scan_debug_prompt,
                capture_folder=scan_capture_folder,
                do_home=False,
                vel_scale=scan_vel_scale,
                accel_scale=scan_accel_scale,
                timeout_s=timeout_s,
                publish_markers=scan_publish_markers,
                axis_length=scan_axis_length,
                axis_radius=scan_axis_radius,
                clear_markers_before=scan_clear_markers_before,
            )
            if not targets:
                raise RuntimeError("Grid sweep produced 0 targets.")
            log.info(f"[print_field] Grid sweep done. Targets={len(targets)}")

            if run_reconstruction:
                c2d_start_ts = time.time()
                c2d_res = self.session.run_sync(
                    self.session.camera.reconstruct_color_to_depth_grid_sweep(
                        use_latest=True,
                        session_path="@session",
                        scan_folder=scan_folder,
                        visualize=False,
                        enqueue=False,
                    ),
                    timeout_s=reconstruct_request_timeout_s,
                )
                if not c2d_res.get("ok", False):
                    raise RuntimeError(f"color_to_depth failed: {c2d_res.get('error')}")

                c2d_output = str(c2d_res.get("metrics", {}).get("output_path", "")).strip()
                if wait_reconstruction_outputs and c2d_output:
                    self._wait_for_fresh_alignment_output(
                        output_path=c2d_output,
                        start_ts=c2d_start_ts,
                        timeout_s=color_to_depth_wait_timeout_s,
                    )

                tsdf_start_ts = time.time()
                tsdf_res = self.session.run_sync(
                    self.session.camera.reconstruct_tsdf_grid_sweep(
                        use_latest=True,
                        session_path="@session",
                        scan_folder=scan_folder,
                        visualize=False,
                        device=reconstruct_device,
                        enqueue=False,
                    ),
                    timeout_s=reconstruct_request_timeout_s,
                )
                if not tsdf_res.get("ok", False):
                    raise RuntimeError(f"tsdf_grid_sweep failed: {tsdf_res.get('error')}")

                mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
                rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()
                if not rgb_ply_path:
                    rgb_ply_path = str(tsdf_res.get("metrics", {}).get("output_path", "")).strip()
                if wait_reconstruction_outputs and mesh_path:
                    self._wait_for_fresh_file_output(
                        output_path=mesh_path,
                        start_ts=tsdf_start_ts,
                        timeout_s=tsdf_wait_timeout_s,
                    )

                mesh_res = self.session.run_sync(
                    self.session.camera.update_world_mesh(
                        use_latest=True,
                        session_path="@session",
                        mesh_path=mesh_path,
                        ply_path=rgb_ply_path,
                        prefer=mesh_prefer,
                        wait_timeout_s=mesh_update_wait_timeout_s,
                        enqueue=False,
                    ),
                    timeout_s=mesh_update_request_timeout_s,
                )
                if not mesh_res.get("ok", False):
                    raise RuntimeError(f"update_world_mesh failed: {mesh_res.get('error')}")

                published_path = str(mesh_res.get("metrics", {}).get("published_path", "")).strip()
                published_kind = str(mesh_res.get("metrics", {}).get("published_kind", "")).strip()
                log.info(
                    f"[print_field] Scan mesh published in RViz ({published_kind}): {published_path}"
                )

            if run_field_init:
                scan_mesh_for_field = field_scan_mesh_path or mesh_path
                scan_mesh_paths = [scan_mesh_for_field] if scan_mesh_for_field else []
                init_res = self.session.run_sync(
                    self.session.field.init_field_from_scan(
                        use_latest=field_use_latest,
                        session_path=field_session_path,
                        scan_mesh_paths=scan_mesh_paths,
                        field_mesh_path=field_mesh_path,
                        state_output_dir=field_state_output_dir,
                        enqueue=False,
                    ),
                    timeout_s=field_request_timeout_s,
                )
                if not init_res.get("ok", False):
                    raise RuntimeError(f"init_field_from_scan failed: {init_res.get('error')}")
                metrics = init_res.get("metrics", {})
                log.info(
                    "[print_field] Field initialized: "
                    f"state='{metrics.get('field_state_path', '')}', "
                    f"debug_ply='{metrics.get('debug_field_ply_path', '')}', "
                    f"offset=({metrics.get('offset_x', 0.0):.4f}, "
                    f"{metrics.get('offset_y', 0.0):.4f}, "
                    f"{metrics.get('offset_z', 0.0):.4f})"
                )

            if home_after:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )
        except TimeoutError:
            log.error("[print_field] Sequence timed out.")
        except Exception as exc:
            log.error(f"[print_field] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _wait_for_fresh_alignment_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> None:
        out_dir = Path(output_path)

        def _fresh_alignment_exists() -> bool:
            if not out_dir.exists() or not out_dir.is_dir():
                return False
            for path in out_dir.glob("color_in_depth*.png"):
                try:
                    if path.stat().st_mtime >= (start_ts - 0.25):
                        return True
                except OSError:
                    continue
            return False

        self.session.run_sync(
            self.session.util.wait_until(
                predicate=_fresh_alignment_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def _wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> None:
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            try:
                if not out_path.exists() or not out_path.is_file():
                    return False
                st = out_path.stat()
                if st.st_size <= 0:
                    return False
                return st.st_mtime >= (start_ts - 0.25)
            except OSError:
                return False

        self.session.run_sync(
            self.session.util.wait_until(
                predicate=_fresh_file_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def _param_optional_float(self, name: str) -> Optional[float]:
        val = float(self.get_parameter(name).value)
        if val <= 0.0:
            return None
        return val


def main(args=None):
    rclpy.init(args=args)
    node = PrintFieldSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
