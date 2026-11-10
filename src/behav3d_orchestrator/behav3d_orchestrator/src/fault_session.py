"""Spatial underextrusion and its RViz region, shared by print workflows."""
import math
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker


class FaultSession:
    """Configure a closed region and reduce net material at selected endpoints."""

    def __init__(self, node=None):
        self.node = node
        self.publisher = None
        self.scene = None
        self._key = None

    def configure(self, cfg, *, config_path, frame_id):
        mesh_path = cfg["fault_region_mesh_path"]
        if mesh_path is None or not str(mesh_path).strip():
            raise ValueError("fault_region_mesh_path must name a closed region mesh")
        path = Path(str(mesh_path).strip()).expanduser()
        if not path.is_absolute():
            path = Path(config_path).expanduser().resolve().parent / path
        key = (str(path.resolve()), float(cfg["fault_region_mesh_scale"]), frame_id)
        reduction = float(cfg["extrusion_reduction_pct"])
        if key != self._key:
            self.load_region(key[0], scale=key[1], reduction_pct=reduction)
            self._publish_region(frame_id)
            self._key = key
        else:
            self.set_reduction(reduction)

    def load_region(self, mesh_path, *, scale=0.001, reduction_pct=40.0):
        """Load geometry without publishing, also usable independently of ROS IO."""
        import open3d as o3d
        path = Path(mesh_path).expanduser().resolve()
        scale = float(scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("fault_region_mesh_scale must be finite and positive")
        reduction = self._validate_reduction(reduction_pct)
        if not path.is_file():
            raise FileNotFoundError(f"Fault region mesh not found: {path}")
        mesh = o3d.io.read_triangle_mesh(str(path))
        mesh.remove_duplicated_vertices()
        mesh.remove_degenerate_triangles()
        if mesh.is_empty() or not mesh.is_watertight():
            raise ValueError("Fault region must be a nonempty watertight mesh")
        mesh.scale(scale, center=(0, 0, 0))
        vertices = np.asarray(mesh.vertices).copy()
        faces = np.asarray(mesh.triangles).copy()
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        self.path, self.scale, self.reduction_pct = path, scale, reduction
        self.vertices, self.faces, self.scene = vertices, faces, scene
        self.minimum, self.maximum = vertices.min(axis=0), vertices.max(axis=0)
        self._key = None
        return self

    @staticmethod
    def _validate_reduction(value):
        value = float(value)
        if not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError("extrusion_reduction_pct must be between 0 and 100")
        return value

    def set_reduction(self, reduction_pct):
        self.reduction_pct = self._validate_reduction(reduction_pct)

    def contains(self, point):
        import open3d as o3d
        if self.scene is None:
            raise RuntimeError("Load or configure the fault region before querying it")
        xyz = np.asarray(point, dtype=np.float32).reshape(1, 3)
        if not np.isfinite(xyz).all():
            raise ValueError("Fault query coordinates must be finite")
        tolerance = 1e-6
        if np.any(xyz[0] < self.minimum - tolerance) or np.any(xyz[0] > self.maximum + tolerance):
            return False
        query = o3d.core.Tensor(xyz)
        return bool(self.scene.compute_occupancy(query, nsamples=3).numpy()[0] > 0.5
                    or self.scene.compute_distance(query).numpy()[0] <= tolerance)

    def material_steps(self, nominal_steps, pose):
        p = pose.pose.position
        factor = 1.0 - self.reduction_pct / 100.0 if self.contains((p.x, p.y, p.z)) else 1.0
        return max(0, int(math.floor(int(nominal_steps) * factor + 0.5)))

    def _publish_region(self, frame_id):
        if self.node is None:
            return
        if self.publisher is None:
            qos = QoSProfile(depth=1)
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.publisher = self.node.create_publisher(Marker, "/behav3d/fault_region", qos)
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
                         for p in self.vertices[self.faces].reshape(-1, 3)]
        self.publisher.publish(marker)
