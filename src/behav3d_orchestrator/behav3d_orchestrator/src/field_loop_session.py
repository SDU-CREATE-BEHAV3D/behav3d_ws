#!/usr/bin/env python3
from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from behav3d_commands.session import Session
from behav3d_examples.src.grid_sweep_session import GridSweepSession
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class FieldLoopSession(Session):
    """
    Shared helpers for field-loop sequences.
    Keeps common scan/reconstruction/field-center logic out of sequence files.
    """

    def __init__(self, node):
        super().__init__(node)
        self._grid_macro = GridSweepSession(node)

    def run_grid_sweep(
        self,
        *,
        target: Optional[PoseStamped] = None,
        width: float,
        height: float,
        z_off: float,
        nx: int,
        ny: int,
        row_major: bool,
        frame_id: str,
        eef_link: str,
        use_tf_orientation: bool,
        debug: bool,
        capture_folder: str,
        do_home: bool = False,
        vel_scale: float,
        accel_scale: float,
        timeout_s: Optional[float],
        publish_markers: bool,
        axis_length: float,
        axis_radius: float,
        clear_markers_before: bool,
        center_x: float = 0.0,
        center_y: float = 0.0,
        center_z: float = 0.0,
    ) -> list[PoseStamped]:
        kwargs = {
            "target": target,
            "width": width,
            "height": height,
            "center_x": center_x,
            "center_y": center_y,
            "center_z": center_z,
            "z_off": z_off,
            "nx": nx,
            "ny": ny,
            "row_major": row_major,
            "frame_id": frame_id,
            "eef_link": eef_link,
            "use_tf_orientation": use_tf_orientation,
            "debug": debug,
            "capture_folder": capture_folder,
            "do_home": bool(do_home),
            "vel_scale": vel_scale,
            "accel_scale": accel_scale,
            "timeout_s": timeout_s,
            "publish_markers": publish_markers,
            "axis_length": axis_length,
            "axis_radius": axis_radius,
            "clear_markers_before": clear_markers_before,
        }
        return self._grid_macro.run_grid_sweep(**kwargs)

    def run_reconstruction_for_scan(
        self,
        *,
        scan_folder: str,
        reconstruct_device: str,
        reconstruct_request_timeout_s: float,
        wait_reconstruction_outputs: bool,
        color_to_depth_wait_timeout_s: float,
        tsdf_wait_timeout_s: float,
        mesh_prefer: str,
        mesh_update_wait_timeout_s: float,
        mesh_update_request_timeout_s: float,
        tsdf_center_crop_enable: bool,
        tsdf_center_crop_width: int,
        tsdf_center_crop_height: int,
        tsdf_center_crop_apply_to_depth: bool,
        tsdf_aabb_crop_enable: bool,
        tsdf_aabb_crop_min: list[float],
        tsdf_aabb_crop_max: list[float],
        tsdf_param_update_timeout_s: float,
    ) -> tuple[str, str]:
        log = self.node.get_logger()

        c2d_start_ts = time.time()
        c2d_res = self.run_sync(
            self.camera.reconstruct_color_to_depth(
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
            self.wait_for_fresh_alignment_output(
                output_path=c2d_output,
                start_ts=c2d_start_ts,
                timeout_s=color_to_depth_wait_timeout_s,
            )

        try:
            self.set_tsdf_service_crop_params(
                center_crop_enable=tsdf_center_crop_enable,
                center_crop_width=tsdf_center_crop_width,
                center_crop_height=tsdf_center_crop_height,
                center_crop_apply_to_depth=tsdf_center_crop_apply_to_depth,
                aabb_crop_enable=tsdf_aabb_crop_enable,
                aabb_crop_min=tsdf_aabb_crop_min,
                aabb_crop_max=tsdf_aabb_crop_max,
                timeout_s=tsdf_param_update_timeout_s,
            )
        except Exception as exc:
            log.warn(
                "[field_loop] Failed to update TSDF crop params; "
                f"continuing with current service params. reason='{exc}'"
            )

        tsdf_start_ts = time.time()
        tsdf_res = self.run_sync(
            self.camera.reconstruct_tsdf_grid_sweep(
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
            self.wait_for_fresh_file_output(
                output_path=mesh_path,
                start_ts=tsdf_start_ts,
                timeout_s=tsdf_wait_timeout_s,
            )

        mesh_res = self.run_sync(
            self.camera.update_world_mesh(
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
        log.info(f"[field_loop] Scan mesh published in RViz ({published_kind}): {published_path}")
        return mesh_path, rgb_ply_path

    def set_tsdf_service_crop_params(
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
        log = self.node.get_logger()
        timeout = max(0.1, float(timeout_s))
        crop_min = self.coerce_xyz_triplet(aabb_crop_min, fallback=(-0.25, -1.10, -1.00))
        crop_max = self.coerce_xyz_triplet(aabb_crop_max, fallback=(0.30, -0.65, 0.50))

        client = AsyncParameterClient(self.node, "/tsdf_cropped_service")
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
            raise TimeoutError("Timed out while setting TSDF crop parameters.")

        response = fut.result()
        if response is None:
            raise RuntimeError("Failed to set TSDF crop parameters: no response.")

        if hasattr(response, "results"):
            results = list(response.results)
        elif isinstance(response, (list, tuple)):
            results = list(response)
        else:
            raise RuntimeError(
                "Failed to set TSDF crop parameters: unexpected response type "
                f"{type(response).__name__}"
            )

        failed = [str(r.reason) for r in results if not bool(getattr(r, "successful", False))]
        if failed:
            raise RuntimeError(f"Failed to set TSDF crop parameters: {'; '.join(failed)}")

        log.info(
            "[field_loop] TSDF crop params set: "
            f"center_crop_enable={bool(center_crop_enable)} "
            f"w={int(center_crop_width)} h={int(center_crop_height)} "
            f"apply_to_depth={bool(center_crop_apply_to_depth)} "
            f"aabb_crop_enable={bool(aabb_crop_enable)} "
            f"aabb_min=({crop_min[0]:.3f}, {crop_min[1]:.3f}, {crop_min[2]:.3f}) "
            f"aabb_max=({crop_max[0]:.3f}, {crop_max[1]:.3f}, {crop_max[2]:.3f})"
        )

    @staticmethod
    def compute_field_center_from_state(field_state_path: str) -> tuple[float, float, float]:
        path = Path(str(field_state_path).strip()).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Field state file not found: {field_state_path}")

        with np.load(str(path), allow_pickle=False) as state:
            if "field_vertices_world" in state:
                vertices_world = np.asarray(state["field_vertices_world"], dtype=np.float64)
                if vertices_world.ndim == 2 and vertices_world.shape[1] == 3 and vertices_world.shape[0] > 0:
                    center = np.mean(vertices_world, axis=0)
                    return (float(center[0]), float(center[1]), float(center[2]))

            if "field_vertices_scaled" in state and "offset_xyz" in state:
                vertices_scaled = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
                offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
                if (
                    vertices_scaled.ndim == 2
                    and vertices_scaled.shape[1] == 3
                    and vertices_scaled.shape[0] > 0
                    and offset_arr.shape[0] >= 3
                ):
                    center = np.mean(vertices_scaled, axis=0) + offset_arr[:3]
                    return (float(center[0]), float(center[1]), float(center[2]))

            if "offset_xyz" in state:
                offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
                if offset_arr.shape[0] >= 3:
                    return (float(offset_arr[0]), float(offset_arr[1]), float(offset_arr[2]))

        raise RuntimeError(
            "Field state does not contain enough data to infer field center "
            "(expected field_vertices_world or field_vertices_scaled+offset_xyz or offset_xyz)."
        )

    @staticmethod
    def coerce_xyz_triplet(values: list[float], *, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
        try:
            arr = np.asarray(values, dtype=float).reshape(-1)
            if arr.shape[0] >= 3 and np.all(np.isfinite(arr[:3])):
                return (float(arr[0]), float(arr[1]), float(arr[2]))
        except Exception:
            pass
        return (float(fallback[0]), float(fallback[1]), float(fallback[2]))

    @staticmethod
    def map_field_center_xy(
        *,
        x: float,
        y: float,
        sign_x: float,
        sign_y: float,
        offset_x: float,
        offset_y: float,
    ) -> tuple[float, float]:
        return (
            float(sign_x) * float(x) + float(offset_x),
            float(sign_y) * float(y) + float(offset_y),
        )

    def wait_for_fresh_alignment_output(self, *, output_path: str, start_ts: float, timeout_s: float) -> None:
        out_dir = Path(output_path)

        def _fresh_alignment_exists() -> bool:
            if not out_dir.exists() or not out_dir.is_dir():
                return False
            # Keep compatibility with previous sequence behavior.
            for pattern in ("color_in_depth*.png", "color_in_depth*.jpg", "*.npy"):
                for p in out_dir.glob(pattern):
                    try:
                        if p.stat().st_mtime >= (float(start_ts) - 0.25):
                            return True
                    except OSError:
                        continue
            return False

        self.wait_for_fresh_file_output(
            output_path=str(out_dir),
            start_ts=start_ts,
            timeout_s=timeout_s,
            exists_fn=_fresh_alignment_exists,
        )

    def wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
        exists_fn=None,
    ) -> None:
        deadline = time.time() + max(0.1, float(timeout_s))
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            if exists_fn is not None:
                return bool(exists_fn())
            if not out_path.exists():
                return False
            try:
                return out_path.stat().st_mtime >= float(start_ts)
            except Exception:
                return True

        while rclpy.ok() and time.time() < deadline:
            if _fresh_file_exists():
                return
            time.sleep(0.1)

        raise TimeoutError(
            f"Timed out waiting for reconstruction output '{output_path}' "
            f"to be updated after start_ts={start_ts:.3f}."
        )
