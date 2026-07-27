import threading
from pathlib import Path
from typing import Any, Dict

import numpy as np
import rclpy
from rclpy.node import Node
from behav3d_interfaces.srv import ColorToDepth, TsdfCropped, TsdfObjectExtract

from . import color_to_depth
from . import TSDF_cpu_cropped
from . import TSDF_cpu_object_extract
from .service_utils import get_captures_root, resolve_session_path


_OUTPUT_FIELDS = ("output_path", "mesh_path", "rgb_ply_path", "confidence_ply_path")


def _normalized_scan_folder(scan_folder: str, default: str = "manual_caps") -> str:
    name = str(scan_folder or "").strip()
    return name if name else default


def _resolve_reconstruct_scan_dir(session_dir: Path, scan_folder: str) -> Path:
    return (session_dir / _normalized_scan_folder(scan_folder) / "reconstruct").resolve()


def _run_color_to_depth(session_dir: Path, scan_folder: str, visualize: bool, device: str = None):
    return color_to_depth.run(
        session_path=str(session_dir),
        scan_folder=scan_folder,
        visualize=visualize,
        device=device
    )


def _run_tsdf_cropped(
    session_dir: Path,
    scan_folder: str,
    visualize: bool,
    device: str = None,
    **kwargs,
):
    return TSDF_cpu_cropped.run(
        session_path=str(session_dir),
        scan_folder_override=scan_folder,
        visualize=visualize,
        device=device,
        **kwargs,
    )


def _run_tsdf_object_extract(session_dir: Path, scan_folder: str, visualize: bool, device: str = None):
    return TSDF_cpu_object_extract.run(
        session_path=str(session_dir),
        scan_folder_override=scan_folder,
        visualize=visualize,
        device=device
    )


def _resolve_color_to_depth_outputs(session_dir: Path, scan_folder: str) -> Dict[str, Any]:
    run_dir = _resolve_reconstruct_scan_dir(session_dir, scan_folder)
    return {
        "output_path": run_dir / "color_in_depth",
    }


def _resolve_tsdf_cropped_outputs(session_dir: Path, scan_folder: str) -> Dict[str, Any]:
    run_dir = _resolve_reconstruct_scan_dir(session_dir, scan_folder)
    return {
        # Keep legacy output_path mapped to RGB PLY for backward compatibility.
        "output_path": run_dir / "tsdf_surface_rgb_colored.ply",
        "mesh_path": run_dir / "tsdf_surface_mesh.stl",
        "rgb_ply_path": run_dir / "tsdf_surface_rgb_colored.ply",
        "confidence_ply_path": run_dir / "tsdf_surface_confidence_colored.ply",
    }


def _resolve_tsdf_object_extract_outputs(session_dir: Path, scan_folder: str) -> Dict[str, Any]:
    run_dir = _resolve_reconstruct_scan_dir(session_dir, scan_folder)
    return {
        # Legacy output_path remains the primary RGB point cloud output.
        "output_path": run_dir / "tsdf_surface_rgb_colored.ply",
        "mesh_path": "",
        "rgb_ply_path": run_dir / "tsdf_surface_rgb_colored.ply",
        "confidence_ply_path": run_dir / "tsdf_surface_confidence_colored.ply",
    }


class _BaseReconstructService(Node):
    def __init__(self, node_name, service_name, runner, output_resolver, srv_type):
        super().__init__(node_name)
        self._runner = runner
        self._output_resolver = output_resolver
        self._running = False
        self._captures_root = get_captures_root()

        self.scan_folder = self.declare_parameter('scan_folder', 'manual_caps').value
        self.visualize = self.declare_parameter('visualize', False).value
        self.device = self.declare_parameter('device', 'CPU:0').value

        self.srv = self.create_service(srv_type, service_name, self.handle_request)
        self.get_logger().info(f"Service active: {service_name}")
        self.get_logger().info(f"Capture root: {self._captures_root}")
        self.get_logger().info(f"Default device: {self.device}")

    @staticmethod
    def _apply_output_fields(response, outputs: Dict[str, Any]) -> None:
        for field in _OUTPUT_FIELDS:
            if not hasattr(response, field):
                continue
            value = outputs.get(field, "")
            if isinstance(value, Path):
                value = str(value)
            elif value is None:
                value = ""
            else:
                value = str(value)
            setattr(response, field, value)

    @staticmethod
    def _clear_output_fields(response) -> None:
        _BaseReconstructService._apply_output_fields(response, {})

    def handle_request(self, request, response):
        if self._running:
            response.success = False
            response.message = 'Reconstruction already running.'
            self._clear_output_fields(response)
            return response

        session_dir = resolve_session_path(
            request.session_path,
            request.use_latest,
            self._captures_root
        )

        scan_folder = request.scan_folder.strip() if request.scan_folder else self.scan_folder
        visualize = bool(request.visualize) if hasattr(request, "visualize") else bool(self.visualize)

        device = self.device
        if hasattr(request, "device"):
            device = request.device.strip() if request.device else device

        self.get_logger().info(
            "Reconstruct request: "
            f"session_dir={session_dir}, "
            f"scan_folder={scan_folder}, "
            f"visualize={visualize}, "
            f"device={device}, "
            f"use_latest={request.use_latest}"
        )

        if not session_dir.exists():
            response.success = False
            response.message = f"Session directory not found: {session_dir}"
            self._clear_output_fields(response)
            return response

        expected = self._output_resolver(session_dir, scan_folder) or {}
        runner_kwargs = self._build_runner_kwargs(request)
        self._running = True
        thread = threading.Thread(
            target=self._run_thread,
            args=(session_dir, scan_folder, visualize, device, runner_kwargs),
            daemon=True
        )
        thread.start()

        response.success = True
        response.message = f"Started reconstruction for {session_dir}"
        self._apply_output_fields(response, expected)
        return response

    def _run_thread(
        self,
        session_dir: Path,
        scan_folder: str,
        visualize: bool,
        device: str,
        runner_kwargs: Dict[str, Any],
    ):
        try:
            self.get_logger().info(f"Running reconstruction for {session_dir}")
            output = self._runner(session_dir, scan_folder, visualize, device, **runner_kwargs)
            if output is not None:
                self.get_logger().info(f"Reconstruction output: {output}")
        except Exception as exc:
            self.get_logger().error(f"Reconstruction failed: {exc}")
        finally:
            self._running = False

    def _build_runner_kwargs(self, request) -> Dict[str, Any]:
        _ = request
        return {}


class ColorToDepthService(_BaseReconstructService):
    def __init__(self):
        super().__init__(
            node_name='color_to_depth_service',
            service_name='/reconstruct/color_to_depth',
            runner=_run_color_to_depth,
            output_resolver=_resolve_color_to_depth_outputs,
            srv_type=ColorToDepth
        )


class TSDFCroppedService(_BaseReconstructService):
    def __init__(self):
        super().__init__(
            node_name='tsdf_cropped_service',
            service_name='/reconstruct/tsdf_cropped',
            runner=_run_tsdf_cropped,
            output_resolver=_resolve_tsdf_cropped_outputs,
            srv_type=TsdfCropped,
        )
        self.declare_parameter("center_crop_enable", bool(TSDF_cpu_cropped.C2D_CENTER_CROP_ENABLE))
        self.declare_parameter(
            "center_crop_width",
            int(TSDF_cpu_cropped.C2D_CENTER_CROP_WIDTH)
            if TSDF_cpu_cropped.C2D_CENTER_CROP_WIDTH is not None
            else 0,
        )
        self.declare_parameter(
            "center_crop_height",
            int(TSDF_cpu_cropped.C2D_CENTER_CROP_HEIGHT)
            if TSDF_cpu_cropped.C2D_CENTER_CROP_HEIGHT is not None
            else 0,
        )
        self.declare_parameter(
            "center_crop_apply_to_depth",
            bool(TSDF_cpu_cropped.C2D_CENTER_CROP_APPLY_TO_DEPTH),
        )
        self.declare_parameter("aabb_crop_enable", bool(TSDF_cpu_cropped.CROP_ENABLE))
        self.declare_parameter(
            "aabb_crop_min",
            [float(v) for v in np.asarray(TSDF_cpu_cropped.CROP_MIN, dtype=np.float64).reshape(-1)[:3]],
        )
        self.declare_parameter(
            "aabb_crop_max",
            [float(v) for v in np.asarray(TSDF_cpu_cropped.CROP_MAX, dtype=np.float64).reshape(-1)[:3]],
        )
        self.declare_parameter(
            "auto_object_crop_enable",
            bool(TSDF_cpu_cropped.AUTO_OBJECT_CROP_ENABLE),
        )
        self.declare_parameter(
            "auto_object_min_height_m",
            float(TSDF_cpu_cropped.AUTO_OBJECT_MIN_HEIGHT_M),
        )
        self.declare_parameter(
            "auto_object_cluster_eps_m",
            float(TSDF_cpu_cropped.AUTO_OBJECT_CLUSTER_EPS_M),
        )
        self.declare_parameter(
            "auto_object_cluster_min_points",
            int(TSDF_cpu_cropped.AUTO_OBJECT_CLUSTER_MIN_POINTS),
        )
        self.declare_parameter(
            "auto_object_neighbor_max_gap_m",
            float(TSDF_cpu_cropped.AUTO_OBJECT_NEIGHBOR_MAX_GAP_M),
        )
        self.declare_parameter(
            "auto_object_xy_margin_m",
            float(TSDF_cpu_cropped.AUTO_OBJECT_XY_MARGIN_M),
        )
        self.declare_parameter(
            "auto_object_top_margin_m",
            float(TSDF_cpu_cropped.AUTO_OBJECT_TOP_MARGIN_M),
        )
        self.declare_parameter(
            "auto_object_table_below_margin_m",
            float(TSDF_cpu_cropped.AUTO_OBJECT_TABLE_BELOW_MARGIN_M),
        )

    def _build_runner_kwargs(self, request) -> Dict[str, Any]:
        _ = request

        crop_min = list(self.get_parameter("aabb_crop_min").value)
        crop_max = list(self.get_parameter("aabb_crop_max").value)
        if len(crop_min) < 3:
            crop_min = [-0.25, -1.1, -1.0]
        if len(crop_max) < 3:
            crop_max = [0.3, -0.65, 0.5]

        return {
            "center_crop_enable": bool(self.get_parameter("center_crop_enable").value),
            "center_crop_width": int(self.get_parameter("center_crop_width").value),
            "center_crop_height": int(self.get_parameter("center_crop_height").value),
            "center_crop_apply_to_depth": bool(self.get_parameter("center_crop_apply_to_depth").value),
            "aabb_crop_enable": bool(self.get_parameter("aabb_crop_enable").value),
            "aabb_crop_min": [float(crop_min[0]), float(crop_min[1]), float(crop_min[2])],
            "aabb_crop_max": [float(crop_max[0]), float(crop_max[1]), float(crop_max[2])],
            "auto_object_crop_enable": bool(
                self.get_parameter("auto_object_crop_enable").value
            ),
            "auto_object_min_height_m": float(
                self.get_parameter("auto_object_min_height_m").value
            ),
            "auto_object_cluster_eps_m": float(
                self.get_parameter("auto_object_cluster_eps_m").value
            ),
            "auto_object_cluster_min_points": int(
                self.get_parameter("auto_object_cluster_min_points").value
            ),
            "auto_object_neighbor_max_gap_m": float(
                self.get_parameter("auto_object_neighbor_max_gap_m").value
            ),
            "auto_object_xy_margin_m": float(
                self.get_parameter("auto_object_xy_margin_m").value
            ),
            "auto_object_top_margin_m": float(
                self.get_parameter("auto_object_top_margin_m").value
            ),
            "auto_object_table_below_margin_m": float(
                self.get_parameter("auto_object_table_below_margin_m").value
            ),
        }


class TSDFObjectExtractService(_BaseReconstructService):
    def __init__(self):
        super().__init__(
            node_name='tsdf_object_extract_service',
            service_name='/reconstruct/tsdf_object_extract',
            runner=_run_tsdf_object_extract,
            output_resolver=_resolve_tsdf_object_extract_outputs,
            srv_type=TsdfObjectExtract,
        )


def _spin_node(node):
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_color_to_depth():
    rclpy.init()
    _spin_node(ColorToDepthService())


def main_tsdf_cropped():
    rclpy.init()
    _spin_node(TSDFCroppedService())


def main_tsdf_object_extract():
    rclpy.init()
    _spin_node(TSDFObjectExtractService())
