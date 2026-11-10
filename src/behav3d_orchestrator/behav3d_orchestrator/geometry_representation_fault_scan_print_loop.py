"""Run the geometry-representation loop with endpoint-selected underextrusion."""
import rclpy

from .geometry_representation_scan_print_loop import GeometryRepresentationScanPrintLoopNode
from .src.fault_region_controller import FaultRegionController


class GeometryRepresentationFaultScanPrintLoopNode(GeometryRepresentationScanPrintLoopNode):
    config_filename = "geometry_representation_fault_scan_print_loop_config.yaml"

    def __init__(self):
        self._fault_region_controller = None
        super().__init__("geometry_representation_fault_scan_print_loop")

    def _configure_print_execution(self, rt, config_path):
        if self._fault_region_controller is None:
            self._fault_region_controller = FaultRegionController(self)
        self._fault_region_controller.configure(
            rt.config, config_path=config_path, frame_id=rt.frame_id, print_session=self.print_session)


def main(args=None):
    rclpy.init(args=args)
    node = GeometryRepresentationFaultScanPrintLoopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
