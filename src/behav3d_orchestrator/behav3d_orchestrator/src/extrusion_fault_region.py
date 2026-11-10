"""A closed mesh selects complete beads by their target endpoint."""
import math
from pathlib import Path

import numpy as np


class ExtrusionFaultRegion:
    def __init__(self, mesh_path, *, scale=0.001, reduction_pct=40.0):
        import open3d as o3d
        self.path = Path(mesh_path).expanduser().resolve()
        self.scale = float(scale)
        if not math.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("fault_region_mesh_scale must be finite and positive")
        if not self.path.is_file():
            raise FileNotFoundError(f"Fault region mesh not found: {self.path}")
        mesh = o3d.io.read_triangle_mesh(str(self.path))
        mesh.remove_duplicated_vertices()
        mesh.remove_degenerate_triangles()
        if mesh.is_empty() or not mesh.is_watertight():
            raise ValueError("Fault region must be a nonempty watertight mesh")
        mesh.scale(self.scale, center=(0, 0, 0))
        self.vertices = np.asarray(mesh.vertices).copy()
        self.faces = np.asarray(mesh.triangles).copy()
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        self.minimum, self.maximum = self.vertices.min(axis=0), self.vertices.max(axis=0)
        self.set_reduction(reduction_pct)

    def set_reduction(self, reduction_pct):
        value = float(reduction_pct)
        if not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError("extrusion_reduction_pct must be between 0 and 100")
        self.reduction_pct = value

    def contains(self, point):
        import open3d as o3d
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
