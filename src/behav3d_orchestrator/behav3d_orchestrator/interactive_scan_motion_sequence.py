#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator interactive_scan_motion_sequence
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from behav3d_examples.src.grid_sweep_session import GridSweepSession
from behav3d_utils import target_builder as tb


class InteractiveScanMotionSequenceNode(Node):
    def __init__(self):
        super().__init__("interactive_scan_motion_sequence")

        self.declare_parameter("home_on_startup", True)
        self.declare_parameter("home_duration_s", 10.0)
        self.declare_parameter("timeout_s", 0.0)

        # Grid sweep params mirrored from print_field_centered_sequence.py
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("scan_eef_link", "femto_color_optical_calib")
        self.declare_parameter("scan_use_tf_orientation", True)
        self.declare_parameter("scan_width", 0.50)
        self.declare_parameter("scan_height", 0.40)
        self.declare_parameter("scan_center_x", -0.20)
        self.declare_parameter("scan_center_y", 0.80)
        self.declare_parameter("scan_center_z", 0.0)
        self.declare_parameter("scan_z_off", 0.60)
        self.declare_parameter("scan_nx", 5)
        self.declare_parameter("scan_ny", 4)
        self.declare_parameter("scan_row_major", False)
        self.declare_parameter("scan_capture_folder", "@session/grid_sweep")
        self.declare_parameter("scan_debug_prompt", False)
        self.declare_parameter("scan_vel_scale", 0.05)
        self.declare_parameter("scan_accel_scale", 0.05)
        self.declare_parameter("scan_publish_markers", True)
        self.declare_parameter("scan_axis_length", 0.05)
        self.declare_parameter("scan_axis_radius", 0.003)
        self.declare_parameter("scan_clear_markers_before", True)
        self.declare_parameter("scan_do_home", True)

        # Reconstruction + RViz mesh visualization params mirrored from print_field_centered_sequence.py
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
        self.declare_parameter("tsdf_center_crop_enable", True)
        self.declare_parameter("tsdf_center_crop_width", 300)
        self.declare_parameter("tsdf_center_crop_height", 290)
        self.declare_parameter("tsdf_center_crop_apply_to_depth", True)
        self.declare_parameter("tsdf_aabb_crop_enable", False)
        self.declare_parameter("tsdf_aabb_crop_min", [-0.25, -1.10, -1.00])
        self.declare_parameter("tsdf_aabb_crop_max", [0.30, -0.65, 0.50])
        self.declare_parameter("tsdf_param_update_timeout_s", 8.0)

        # Planning-scene update params
        self.declare_parameter("planning_scene_object_id", "behav3d_reconstructed_mesh")
        self.declare_parameter("planning_scene_frame_id", "ur20_base_link")
        self.declare_parameter("planning_scene_wait_timeout_s", 30.0)
        self.declare_parameter("planning_scene_request_timeout_s", 35.0)

        # Manual move params
        self.declare_parameter("manual_eef_link", "extruder_tcp")
        self.declare_parameter("manual_use_tf_orientation", True)
        self.declare_parameter("manual_vel_scale", 0.05)
        self.declare_parameter("manual_accel_scale", 0.05)
        self.declare_parameter("manual_do_home", True)
        self.declare_parameter("manual_home_after", False)
        self.declare_parameter("manual_publish_markers", True)
        self.declare_parameter("manual_axis_length", 0.05)
        self.declare_parameter("manual_axis_radius", 0.003)
        self.declare_parameter("manual_clear_markers_before", True)

        self.session = GridSweepSession(self)
        self._last_mesh_path: str = ""
        self._last_rgb_ply_path: str = ""
        self._last_scan_folder: str = ""
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        if not sys.stdin or not sys.stdin.isatty():
            log.error("[interactive_scan_motion_sequence] No TTY detected; interactive prompt unavailable.")
            rclpy.shutdown()
            return

        timeout_s = self._param_optional_float("timeout_s")
        home_on_startup = bool(self.get_parameter("home_on_startup").value)

        try:
            if home_on_startup:
                self._do_home(timeout_s)

            log.info(
                "[interactive_scan_motion_sequence] Ready. "
                "Actions: s=grid sweep scan + reconstruct + visualize, "
                "p=push last mesh to planning scene, m=manual move sequence, h=home, q=quit."
            )
            log.info(
                "[interactive_scan_motion_sequence] Manual move sequence is hardcoded in cm as: "
                "(0, 80, 40) -> (0, 80, 10)."
            )

            while rclpy.ok():
                try:
                    choice = input(
                        "[interactive_scan_motion_sequence] Action [s/p/m/h/q]: "
                    ).strip().lower()
                except EOFError:
                    log.warn("[interactive_scan_motion_sequence] EOF on stdin; shutting down.")
                    break
                except Exception as exc:
                    log.error(f"[interactive_scan_motion_sequence] Input failed: {exc}")
                    break

                if choice == "q":
                    log.info("[interactive_scan_motion_sequence] Quit requested by user.")
                    break
                if choice == "h":
                    try:
                        self._do_home(timeout_s)
                    except Exception as exc:
                        log.error(f"[interactive_scan_motion_sequence] Home failed: {exc}")
                    continue
                if choice == "s":
                    try:
                        self._run_grid_sweep(timeout_s)
                    except Exception as exc:
                        log.error(f"[interactive_scan_motion_sequence] Grid sweep failed: {exc}")
                    continue
                if choice == "m":
                    try:
                        self._run_manual_motion_sequence(timeout_s)
                    except Exception as exc:
                        log.error(f"[interactive_scan_motion_sequence] Manual motion failed: {exc}")
                    continue
                if choice == "p":
                    try:
                        self._push_last_mesh_to_planning_scene()
                    except Exception as exc:
                        log.error(
                            f"[interactive_scan_motion_sequence] Planning-scene update failed: {exc}"
                        )
                    continue

                log.warn(
                    "[interactive_scan_motion_sequence] Unknown action. "
                    "Use s=scan, p=planning-scene update, m=manual move, h=home, q=quit."
                )
        finally:
            rclpy.shutdown()

    def _do_home(self, timeout_s: Optional[float]) -> None:
        duration_s = float(self.get_parameter("home_duration_s").value)
        self.get_logger().info(
            f"[interactive_scan_motion_sequence] Homing robot (duration_s={duration_s:.2f})."
        )
        res = self.session.run_sync(
            self.session.motion.home(duration_s=duration_s, enqueue=False),
            timeout_s=timeout_s,
        )
        if not res.get("ok", False):
            raise RuntimeError(res.get("error", "home failed"))

    def _run_grid_sweep(self, timeout_s: Optional[float]) -> None:
        log = self.get_logger()
        log.info("[interactive_scan_motion_sequence] Starting grid sweep scan.")

        targets = self.session.run_grid_sweep(
            width=float(self.get_parameter("scan_width").value),
            height=float(self.get_parameter("scan_height").value),
            center_x=float(self.get_parameter("scan_center_x").value),
            center_y=float(self.get_parameter("scan_center_y").value),
            center_z=float(self.get_parameter("scan_center_z").value),
            z_off=float(self.get_parameter("scan_z_off").value),
            nx=int(self.get_parameter("scan_nx").value),
            ny=int(self.get_parameter("scan_ny").value),
            row_major=bool(self.get_parameter("scan_row_major").value),
            frame_id=str(self.get_parameter("frame_id").value).strip() or "world",
            eef_link=str(self.get_parameter("scan_eef_link").value).strip() or "femto_color_optical_calib",
            use_tf_orientation=bool(self.get_parameter("scan_use_tf_orientation").value),
            debug=bool(self.get_parameter("scan_debug_prompt").value),
            capture_folder=str(self.get_parameter("scan_capture_folder").value).strip() or "@session/grid_sweep",
            do_home=bool(self.get_parameter("scan_do_home").value),
            vel_scale=float(self.get_parameter("scan_vel_scale").value),
            accel_scale=float(self.get_parameter("scan_accel_scale").value),
            timeout_s=timeout_s,
            publish_markers=bool(self.get_parameter("scan_publish_markers").value),
            axis_length=float(self.get_parameter("scan_axis_length").value),
            axis_radius=float(self.get_parameter("scan_axis_radius").value),
            clear_markers_before=bool(self.get_parameter("scan_clear_markers_before").value),
        )

        log.info(
            "[interactive_scan_motion_sequence] Grid sweep finished. "
            f"Traversed targets={len(targets)}."
        )

        if not targets:
            raise RuntimeError("Grid sweep produced 0 targets.")

        if bool(self.get_parameter("run_reconstruction").value):
            scan_folder = self._resolve_scan_folder()
            mesh_path, rgb_ply_path = self._run_reconstruction_for_scan(scan_folder=scan_folder)
            self._last_scan_folder = scan_folder
            self._last_mesh_path = mesh_path
            self._last_rgb_ply_path = rgb_ply_path
            log.info(
                "[interactive_scan_motion_sequence] Reconstruction + visualization finished. "
                f"mesh='{mesh_path}', rgb_ply='{rgb_ply_path}'."
            )

    def _push_last_mesh_to_planning_scene(self) -> None:
        log = self.get_logger()

        mesh_path = str(self._last_mesh_path).strip()
        if not mesh_path:
            raise RuntimeError("No reconstructed mesh available yet. Run 's' first.")

        object_id = str(self.get_parameter("planning_scene_object_id").value).strip() or "behav3d_reconstructed_mesh"
        frame_id = str(self.get_parameter("planning_scene_frame_id").value).strip()
        wait_timeout_s = float(self.get_parameter("planning_scene_wait_timeout_s").value)
        request_timeout_s = self._param_optional_float("planning_scene_request_timeout_s")

        res = self._run_checked(
            self.session.camera.update_planning_scene_mesh(
                use_latest=False,
                session_path="@session",
                mesh_path=mesh_path,
                object_id=object_id,
                frame_id=frame_id,
                wait_timeout_s=wait_timeout_s,
                enqueue=False,
            ),
            timeout_s=request_timeout_s,
            label="update_planning_scene_mesh",
        )

        metrics = res.get("metrics", {})
        log.info(
            "[interactive_scan_motion_sequence] Planning scene updated: "
            f"object_id='{metrics.get('applied_object_id', '')}', "
            f"frame='{metrics.get('applied_frame_id', '')}', "
            f"triangles={metrics.get('triangle_count', 0)}, "
            f"mesh='{metrics.get('resolved_mesh_path', '')}'."
        )

    def _run_manual_motion_sequence(self, timeout_s: Optional[float]) -> None:
        log = self.get_logger()

        manual_do_home = bool(self.get_parameter("manual_do_home").value)
        manual_home_after = bool(self.get_parameter("manual_home_after").value)
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        eef_link = str(self.get_parameter("manual_eef_link").value).strip() or "extruder_tcp"
        motion = "LIN"
        vel_scale = float(self.get_parameter("manual_vel_scale").value)
        accel_scale = float(self.get_parameter("manual_accel_scale").value)
        use_tf_orientation = bool(self.get_parameter("manual_use_tf_orientation").value)
        publish_markers = bool(self.get_parameter("manual_publish_markers").value)
        axis_length = float(self.get_parameter("manual_axis_length").value)
        axis_radius = float(self.get_parameter("manual_axis_radius").value)
        clear_markers_before = bool(self.get_parameter("manual_clear_markers_before").value)

        if manual_do_home:
            self._do_home(timeout_s)

        self._run_checked(
            self.session.motion.setEef(eef_link, enqueue=False),
            timeout_s=timeout_s,
            label="setEef",
        )
        self._run_checked(
            self.session.motion.setSpd(vel_scale, enqueue=False),
            timeout_s=timeout_s,
            label="setSpd",
        )
        self._run_checked(
            self.session.motion.setAcc(accel_scale, enqueue=False),
            timeout_s=timeout_s,
            label="setAcc",
        )
        self._run_checked(
            self.session.motion.setLIN(enqueue=False),
            timeout_s=timeout_s,
            label="setLIN",
        )

        reference_pose = self._resolve_reference_pose(
            eef_link=eef_link,
            frame_id=frame_id,
            use_tf_orientation=use_tf_orientation,
            timeout_s=timeout_s,
        )
        targets = [self._target_from_cm(reference_pose, *xyz_cm) for xyz_cm in self._manual_points_cm()]

        if publish_markers:
            self._run_checked(
                self.session.util.publish_targets(
                    targets,
                    axis_length=axis_length,
                    axis_radius=axis_radius,
                    clear_before=clear_markers_before,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
                label="publish manual targets",
            )

        log.info(
            "[interactive_scan_motion_sequence] Starting manual motion sequence: "
            f"eef='{eef_link}', motion='{motion}', targets={len(targets)}"
        )
        log.info(
            "[interactive_scan_motion_sequence] Manual target order (cm): "
            "(0, 80, 40) -> (0, 80, 10)"
        )

        for idx, target in enumerate(targets):
            log.info(
                "[interactive_scan_motion_sequence] Moving to target "
                f"{idx}: x={target.pose.position.x:.3f} y={target.pose.position.y:.3f} z={target.pose.position.z:.3f}"
            )
            plan_res = self._run_checked(
                self.session.motion.plan(
                    pose=target,
                    motion=motion,
                    vel_scale=vel_scale,
                    accel_scale=accel_scale,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
                label=f"plan target {idx}",
            )

            self._run_checked(
                self.session.motion.exec(enqueue=False),
                timeout_s=timeout_s,
                label=f"exec target {idx}",
            )

        log.info("[interactive_scan_motion_sequence] Manual motion sequence finished.")

        if manual_home_after:
            self._do_home(timeout_s)

    def _resolve_reference_pose(
        self,
        *,
        eef_link: str,
        frame_id: str,
        use_tf_orientation: bool,
        timeout_s: Optional[float],
    ) -> PoseStamped:
        log = self.get_logger()
        reference = tb.worldXY(0.0, 0.0, 0.0, frame_id)

        if not use_tf_orientation or not eef_link:
            return reference

        try:
            res = self.session.run_sync(
                self.session.camera.get_pose(
                    eef=eef_link,
                    base_frame=frame_id,
                    use_tf=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
        except TimeoutError:
            log.warn(
                "[interactive_scan_motion_sequence] TF orientation lookup timed out; "
                "using identity orientation."
            )
            return reference

        if res.get("ok", False) and "pose" in res:
            return res["pose"]

        log.warn(
            "[interactive_scan_motion_sequence] TF orientation not available; "
            "using identity orientation."
        )
        return reference

    @staticmethod
    def _manual_points_cm() -> Sequence[Tuple[float, float, float]]:
        return (
            (0.0, 80.0, 40.0),
            (0.0, 80.0, 10.0),
        )

    @staticmethod
    def _target_from_cm(reference: PoseStamped, x_cm: float, y_cm: float, z_cm: float) -> PoseStamped:
        return tb.setTargetOrigin(
            reference,
            float(x_cm) / 100.0,
            float(y_cm) / 100.0,
            float(z_cm) / 100.0,
        )

    def _param_optional_float(self, name: str) -> Optional[float]:
        value = float(self.get_parameter(name).value)
        return None if value <= 0.0 else value

    def _run_checked(self, item, *, timeout_s: Optional[float], label: str):
        res = self.session.run_sync(item, timeout_s=timeout_s)
        if not res.get("ok", False):
            raise RuntimeError(f"{label} failed: {res.get('error', 'unknown')}")
        return res

    def _resolve_scan_folder(self) -> str:
        scan_folder = str(self.get_parameter("scan_folder").value).strip()
        if scan_folder:
            return scan_folder

        capture_folder = str(self.get_parameter("scan_capture_folder").value).strip()
        if capture_folder.startswith("@session/"):
            return capture_folder[len("@session/") :]
        return "grid_sweep"

    def _run_reconstruction_for_scan(self, *, scan_folder: str) -> Tuple[str, str]:
        log = self.get_logger()

        reconstruct_request_timeout_s = float(self.get_parameter("reconstruct_request_timeout_s").value)
        wait_reconstruction_outputs = bool(self.get_parameter("wait_reconstruction_outputs").value)
        color_to_depth_wait_timeout_s = float(self.get_parameter("color_to_depth_wait_timeout_s").value)
        tsdf_wait_timeout_s = float(self.get_parameter("tsdf_wait_timeout_s").value)
        reconstruct_device = str(self.get_parameter("reconstruct_device").value).strip() or "CPU:0"
        mesh_prefer = str(self.get_parameter("mesh_prefer").value).strip() or "mesh"
        mesh_update_wait_timeout_s = float(self.get_parameter("mesh_update_wait_timeout_s").value)
        mesh_update_request_timeout_s = self._param_optional_float("mesh_update_request_timeout_s")
        tsdf_param_update_timeout_s = float(self.get_parameter("tsdf_param_update_timeout_s").value)

        c2d_start_ts = time.time()
        c2d_res = self._run_checked(
            self.session.camera.reconstruct_color_to_depth(
                use_latest=True,
                session_path="@session",
                scan_folder=scan_folder,
                visualize=False,
                enqueue=False,
            ),
            timeout_s=reconstruct_request_timeout_s,
            label="reconstruct_color_to_depth",
        )

        c2d_output = str(c2d_res.get("metrics", {}).get("output_path", "")).strip()
        if wait_reconstruction_outputs and c2d_output:
            self._wait_for_fresh_alignment_output(
                output_path=c2d_output,
                start_ts=c2d_start_ts,
                timeout_s=color_to_depth_wait_timeout_s,
            )

        try:
            self._set_tsdf_service_crop_params(
                center_crop_enable=bool(self.get_parameter("tsdf_center_crop_enable").value),
                center_crop_width=int(self.get_parameter("tsdf_center_crop_width").value),
                center_crop_height=int(self.get_parameter("tsdf_center_crop_height").value),
                center_crop_apply_to_depth=bool(self.get_parameter("tsdf_center_crop_apply_to_depth").value),
                aabb_crop_enable=bool(self.get_parameter("tsdf_aabb_crop_enable").value),
                aabb_crop_min=list(self.get_parameter("tsdf_aabb_crop_min").value),
                aabb_crop_max=list(self.get_parameter("tsdf_aabb_crop_max").value),
                timeout_s=tsdf_param_update_timeout_s,
            )
        except Exception as exc:
            log.warn(
                "[interactive_scan_motion_sequence] Failed to update TSDF crop params; "
                f"continuing with current service params. reason='{exc}'"
            )

        tsdf_start_ts = time.time()
        tsdf_res = self._run_checked(
            self.session.camera.reconstruct_tsdf_grid_sweep(
                use_latest=True,
                session_path="@session",
                scan_folder=scan_folder,
                visualize=False,
                device=reconstruct_device,
                enqueue=False,
            ),
            timeout_s=reconstruct_request_timeout_s,
            label="reconstruct_tsdf_grid_sweep",
        )

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

        mesh_res = self._run_checked(
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
            label="update_world_mesh",
        )

        log.info(
            "[interactive_scan_motion_sequence] Mesh published in RViz "
            f"({mesh_res.get('metrics', {}).get('published_kind', '')}): "
            f"{mesh_res.get('metrics', {}).get('published_path', '')}"
        )
        return (mesh_path, rgb_ply_path)

    def _wait_for_fresh_alignment_output(self, *, output_path: str, start_ts: float, timeout_s: float) -> None:
        out_dir = Path(output_path)
        deadline = time.time() + float(timeout_s)

        while rclpy.ok() and time.time() < deadline:
            if out_dir.exists() and out_dir.is_dir():
                for path in out_dir.glob("color_in_depth*.png"):
                    try:
                        if path.stat().st_mtime >= (start_ts - 0.25):
                            return
                    except OSError:
                        continue
            time.sleep(0.5)

        raise TimeoutError(f"Timed out waiting for fresh color_to_depth outputs in '{output_path}'")

    def _wait_for_fresh_file_output(self, *, output_path: str, start_ts: float, timeout_s: float) -> None:
        out_path = Path(output_path)
        deadline = time.time() + float(timeout_s)

        while rclpy.ok() and time.time() < deadline:
            try:
                if out_path.exists() and out_path.is_file():
                    st = out_path.stat()
                    if st.st_size > 0 and st.st_mtime >= (start_ts - 0.25):
                        return
            except OSError:
                pass
            time.sleep(0.5)

        raise TimeoutError(f"Timed out waiting for fresh file output '{output_path}'")

    def _set_tsdf_service_crop_params(
        self,
        *,
        center_crop_enable: bool,
        center_crop_width: int,
        center_crop_height: int,
        center_crop_apply_to_depth: bool,
        aabb_crop_enable: bool,
        aabb_crop_min: list[float],
        aabb_crop_max: list[float],
        timeout_s: float,
    ) -> None:
        timeout = max(0.1, float(timeout_s))
        crop_min = self._coerce_xyz_triplet(aabb_crop_min, fallback=(-0.25, -1.10, -1.00))
        crop_max = self._coerce_xyz_triplet(aabb_crop_max, fallback=(0.30, -0.65, 0.50))

        client = AsyncParameterClient(self, "/tsdf_cropped_service")
        if hasattr(client, "wait_for_services"):
            ready = bool(client.wait_for_services(timeout_sec=timeout))
        elif hasattr(client, "wait_for_service"):
            ready = bool(client.wait_for_service(timeout_sec=timeout))
        else:
            ready = bool(client.services_are_ready()) if hasattr(client, "services_are_ready") else False
        if not ready:
            raise RuntimeError("Parameter service for /tsdf_cropped_service not available.")

        params = [
            Parameter("center_crop_enable", value=bool(center_crop_enable)),
            Parameter("center_crop_width", value=int(center_crop_width)),
            Parameter("center_crop_height", value=int(center_crop_height)),
            Parameter("center_crop_apply_to_depth", value=bool(center_crop_apply_to_depth)),
            Parameter("aabb_crop_enable", value=bool(aabb_crop_enable)),
            Parameter("aabb_crop_min", value=[float(crop_min[0]), float(crop_min[1]), float(crop_min[2])]),
            Parameter("aabb_crop_max", value=[float(crop_max[0]), float(crop_max[1]), float(crop_max[2])]),
        ]

        fut = client.set_parameters(params)
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            raise TimeoutError("Timed out updating /tsdf_cropped_service parameters.")

        response = fut.result()
        if response is None:
            raise RuntimeError("No response from /tsdf_cropped_service parameter update.")

        if hasattr(response, "results"):
            results = list(response.results)
        elif isinstance(response, (list, tuple)):
            results = list(response)
        else:
            raise RuntimeError(
                "Unexpected response type while updating /tsdf_cropped_service parameters: "
                f"{type(response).__name__}"
            )

        failed = [str(item.reason) for item in results if not bool(getattr(item, "successful", False))]
        if failed:
            raise RuntimeError("; ".join(failed))

    @staticmethod
    def _coerce_xyz_triplet(values, *, fallback: Tuple[float, float, float]) -> Tuple[float, float, float]:
        if not isinstance(values, (list, tuple)) or len(values) < 3:
            return fallback
        try:
            return (float(values[0]), float(values[1]), float(values[2]))
        except (TypeError, ValueError):
            return fallback


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveScanMotionSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
