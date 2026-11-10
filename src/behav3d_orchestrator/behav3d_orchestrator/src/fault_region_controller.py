"""Share fault-region configuration and RViz display across print workflows."""
from pathlib import Path

from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker

from .extrusion_fault_region import ExtrusionFaultRegion


class FaultRegionController:
    def __init__(self, node):
        self.node = node
        self.region = None
        self._key = None
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.publisher = node.create_publisher(Marker, "/behav3d/fault_region", qos)

    def configure(self, cfg, *, config_path, frame_id, print_session):
        mesh_path = cfg["fault_region_mesh_path"]
        if mesh_path is None or not str(mesh_path).strip():
            raise ValueError("fault_region_mesh_path must name a closed region mesh")
        path = Path(str(mesh_path).strip()).expanduser()
        if not path.is_absolute():
            path = Path(config_path).expanduser().resolve().parent / path
        key = (str(path.resolve()), float(cfg["fault_region_mesh_scale"]), frame_id)
        reduction = float(cfg["extrusion_reduction_pct"])
        if key != self._key:
            region = ExtrusionFaultRegion(key[0], scale=key[1], reduction_pct=reduction)
            self._publish_region(region, frame_id)
            self.region, self._key = region, key
        else:
            self.region.set_reduction(reduction)
        print_session.extrusion_fault_region = self.region

    def _publish_region(self, region, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "extrusion_fault_region"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.45, 0.1, 0.2
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                         for p in region.vertices[region.faces].reshape(-1, 3)]
        self.publisher.publish(marker)
