import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from behav3d_interfaces.srv import ColorToDepth, TsdfCropped, TsdfObjectExtract

from . import color_to_depth
from . import TSDF_cpu_cropped
from . import TSDF_cpu_object_extract
from .service_utils import get_captures_root, resolve_session_path


def _run_color_to_depth(session_dir: Path, scan_folder: str, visualize: bool):
    return color_to_depth.run(
        session_path=str(session_dir),
        scan_folder=scan_folder,
        visualize=visualize
    )


def _run_tsdf_cropped(session_dir: Path, scan_folder: str, visualize: bool):
    return TSDF_cpu_cropped.run(
        session_path=str(session_dir),
        scan_folder_override=scan_folder,
        visualize=visualize
    )


def _run_tsdf_object_extract(session_dir: Path, scan_folder: str, visualize: bool):
    return TSDF_cpu_object_extract.run(
        session_path=str(session_dir),
        scan_folder_override=scan_folder,
        visualize=visualize
    )


class _BaseReconstructService(Node):
    def __init__(self, node_name, service_name, runner, output_resolver, srv_type):
        super().__init__(node_name)
        self._runner = runner
        self._output_resolver = output_resolver
        self._running = False
        self._captures_root = get_captures_root()

        self.scan_folder = self.declare_parameter('scan_folder', 'manual_caps').value
        self.visualize = self.declare_parameter('visualize', False).value

        self.srv = self.create_service(srv_type, service_name, self.handle_request)
        self.get_logger().info(f"Service active: {service_name}")
        self.get_logger().info(f"Capture root: {self._captures_root}")

    def handle_request(self, request, response):
        if self._running:
            response.success = False
            response.message = 'Reconstruction already running.'
            response.output_path = ''
            return response

        session_dir = resolve_session_path(
            request.session_path,
            request.use_latest,
            self._captures_root
        )

        scan_folder = request.scan_folder.strip() if request.scan_folder else self.scan_folder
        visualize = bool(request.visualize) if hasattr(request, "visualize") else bool(self.visualize)

        self.get_logger().info(
            "Reconstruct request: "
            f"session_dir={session_dir}, "
            f"scan_folder={scan_folder}, "
            f"visualize={visualize}, "
            f"use_latest={request.use_latest}"
        )

        if not session_dir.exists():
            response.success = False
            response.message = f"Session directory not found: {session_dir}"
            response.output_path = ''
            return response

        expected = self._output_resolver(session_dir)
        self._running = True
        thread = threading.Thread(
            target=self._run_thread,
            args=(session_dir, scan_folder, visualize),
            daemon=True
        )
        thread.start()

        response.success = True
        response.message = f"Started reconstruction for {session_dir}"
        response.output_path = str(expected) if expected is not None else ''
        return response

    def _run_thread(self, session_dir: Path, scan_folder: str, visualize: bool):
        try:
            self.get_logger().info(f"Running reconstruction for {session_dir}")
            self._runner(session_dir, scan_folder, visualize)
        except Exception as exc:
            self.get_logger().error(f"Reconstruction failed: {exc}")
        finally:
            self._running = False


class ColorToDepthService(_BaseReconstructService):
    def __init__(self):
        super().__init__(
            node_name='color_to_depth_service',
            service_name='/reconstruct/color_to_depth',
            runner=_run_color_to_depth,
            output_resolver=lambda session: session / 'alignment_test',
            srv_type=ColorToDepth
        )


class TSDFCroppedService(_BaseReconstructService):
    def __init__(self):
        super().__init__(
            node_name='tsdf_cropped_service',
            service_name='/reconstruct/tsdf_cropped',
            runner=_run_tsdf_cropped,
            output_resolver=lambda session: session / 'tsdf_surface_rgb_colored.ply',
            srv_type=TsdfCropped
        )


class TSDFObjectExtractService(_BaseReconstructService):
    def __init__(self):
        super().__init__(
            node_name='tsdf_object_extract_service',
            service_name='/reconstruct/tsdf_object_extract',
            runner=_run_tsdf_object_extract,
            output_resolver=lambda session: session / 'tsdf_surface_rgb_colored.ply',
            srv_type=TsdfObjectExtract
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
