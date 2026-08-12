#!/usr/bin/env python3
"""Extruder-to-scan collision checks for generated scalar-field targets."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


DEFAULT_EXTRUDER_MESH = (
    Path(__file__).resolve().parents[4]
    / "custom_workcell"
    / "ur20_workcell"
    / "meshes"
    / "ram_extruder_v2_simplified.dae"
)

# Calibrated tool0 -> extruder_tcp transform from workcell_macro.xacro.
TOOL0_TCP_XYZ = (-0.00551, 0.47304, 0.0822)
TOOL0_TCP_RPY = (1.5641113421, 0.0, 0.0)


@dataclass(frozen=True)
class SegmentCollisionResult:
    """Minimum clearance and first colliding sample for one target segment."""

    collides: bool
    min_distance_mm: float
    hit_vertices: int
    sample_index: int | None
    alpha: float | None


def normalize(vector: np.ndarray) -> np.ndarray:
    """Return a unit vector, using world +Z for a degenerate input."""
    value = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(value))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return value / norm


def rotation_from_z_axis(z_axis: np.ndarray) -> np.ndarray:
    """Match the fixed-reference roll used by ``quat_from_z_axis`` at runtime."""
    z = normalize(z_axis)
    reference = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(z[0])) > 0.95:
        reference = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x = normalize(reference - float(np.dot(reference, z)) * z)
    y = normalize(np.cross(z, x))
    return np.column_stack((x, y, z))


def transform_from_xyz_rpy(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> np.ndarray:
    """Build a homogeneous transform using the URDF fixed-axis RPY convention."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a homogeneous transform to an ``(N, 3)`` point array."""
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotate_about_world_z(values: np.ndarray, yaw_deg: float) -> np.ndarray:
    """Rotate points or direction vectors around world Z at the origin."""
    yaw = np.deg2rad(float(yaw_deg))
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    rotation = np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return np.asarray(values, dtype=np.float64) @ rotation.T


def load_collada_mesh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load triangle arrays from the workcell COLLADA collision mesh."""
    import collada

    document = collada.Collada(str(path))
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for geometry in document.geometries:
        for primitive in geometry.primitives:
            if not hasattr(primitive, "vertex") or not hasattr(
                primitive, "vertex_index"
            ):
                continue
            primitive_vertices = np.asarray(primitive.vertex, dtype=np.float64)
            primitive_faces = np.asarray(
                primitive.vertex_index,
                dtype=np.int64,
            ).reshape((-1, 3))
            vertices.append(primitive_vertices)
            faces.append(primitive_faces + offset)
            offset += int(primitive_vertices.shape[0])
    if not vertices:
        raise ValueError(f"No triangle geometry found in COLLADA mesh: {path}")
    return np.vstack(vertices), np.vstack(faces)


def make_scan_scene(vertices: np.ndarray, faces: np.ndarray):
    """Build an Open3D raycasting scene from legacy mesh arrays."""
    import open3d as o3d

    mesh = o3d.t.geometry.TriangleMesh()
    mesh.vertex["positions"] = o3d.core.Tensor(
        np.asarray(vertices, dtype=np.float32)
    )
    mesh.triangle["indices"] = o3d.core.Tensor(
        np.asarray(faces, dtype=np.int32)
    )
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)
    return scene


class ExtruderCollisionChecker:
    """Check the extruder collision mesh at sampled target poses."""

    def __init__(
        self,
        *,
        scan_vertices: np.ndarray,
        scan_faces: np.ndarray,
        extruder_vertices_tool0: np.ndarray,
        threshold_mm: float,
        tcp_exclusion_radius_mm: float = 0.0,
        tool0_tcp_xyz: tuple[float, float, float] = TOOL0_TCP_XYZ,
        tool0_tcp_rpy: tuple[float, float, float] = TOOL0_TCP_RPY,
    ) -> None:
        threshold = float(threshold_mm)
        if not np.isfinite(threshold) or threshold < 0.0:
            raise ValueError(f"threshold_mm must be finite and >= 0, got {threshold_mm}")
        exclusion = float(tcp_exclusion_radius_mm)
        if not np.isfinite(exclusion) or exclusion < 0.0:
            raise ValueError(
                "tcp_exclusion_radius_mm must be finite and >= 0, "
                f"got {tcp_exclusion_radius_mm}"
            )

        vertices = np.asarray(extruder_vertices_tool0, dtype=np.float64)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or vertices.shape[0] == 0:
            raise ValueError(
                "extruder_vertices_tool0 must have non-empty shape (N, 3), "
                f"got {vertices.shape}"
            )
        if exclusion > 0.0:
            tcp_tool0 = np.asarray(tool0_tcp_xyz, dtype=np.float64)
            distances_mm = 1e3 * np.linalg.norm(vertices - tcp_tool0, axis=1)
            vertices = vertices[distances_mm > exclusion]
            if vertices.shape[0] == 0:
                raise ValueError("TCP exclusion radius removed every extruder vertex.")

        self._scene = make_scan_scene(scan_vertices, scan_faces)
        self._extruder_vertices_tool0 = vertices
        self._threshold_mm = threshold
        self._tcp_tool0 = np.linalg.inv(
            transform_from_xyz_rpy(tool0_tcp_xyz, tool0_tcp_rpy)
        )

    @classmethod
    def from_open3d_mesh(
        cls,
        scan_mesh,
        *,
        extruder_mesh_path: Path = DEFAULT_EXTRUDER_MESH,
        threshold_mm: float = 1.0,
        tcp_exclusion_radius_mm: float = 10.0,
        target_yaw_deg: float = 0.0,
    ) -> "ExtruderCollisionChecker":
        """Create a checker in the final target frame used by the YAML executor."""
        scan_vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
        scan_faces = np.asarray(scan_mesh.triangles, dtype=np.int32)
        if scan_vertices.size == 0 or scan_faces.size == 0:
            raise ValueError("Scan mesh must contain vertices and triangles.")
        scan_vertices = rotate_about_world_z(scan_vertices, float(target_yaw_deg))
        extruder_vertices, _extruder_faces = load_collada_mesh(extruder_mesh_path)
        return cls(
            scan_vertices=scan_vertices,
            scan_faces=scan_faces,
            extruder_vertices_tool0=extruder_vertices,
            threshold_mm=float(threshold_mm),
            tcp_exclusion_radius_mm=float(tcp_exclusion_radius_mm),
        )

    def check_pose(
        self,
        position: np.ndarray,
        z_axis: np.ndarray,
    ) -> tuple[bool, float, int]:
        """Check one target pose and return collision, clearance, and hit count."""
        import open3d as o3d

        world_tcp = np.eye(4, dtype=np.float64)
        world_tcp[:3, :3] = rotation_from_z_axis(z_axis)
        world_tcp[:3, 3] = np.asarray(position, dtype=np.float64)
        world_tool0 = world_tcp @ self._tcp_tool0
        vertices_world = transform_points(
            self._extruder_vertices_tool0,
            world_tool0,
        )
        distances_m = self._scene.compute_distance(
            o3d.core.Tensor(vertices_world.astype(np.float32))
        ).numpy()
        threshold_m = 1e-3 * self._threshold_mm
        hit_vertices = int(np.count_nonzero(distances_m <= threshold_m))
        return (
            bool(hit_vertices),
            float(np.min(distances_m) * 1e3),
            hit_vertices,
        )

    def check_segment(
        self,
        *,
        start: np.ndarray,
        end: np.ndarray,
        start_z: np.ndarray,
        end_z: np.ndarray,
        samples: int,
    ) -> SegmentCollisionResult:
        """Sample a target segment and reject it if any sampled pose collides."""
        sample_count = max(1, int(samples))
        if sample_count == 1:
            alphas = np.array([1.0], dtype=np.float64)
        else:
            alphas = np.linspace(0.0, 1.0, sample_count)

        min_distance_mm = float("inf")
        total_hit_vertices = 0
        first_sample: int | None = None
        first_alpha: float | None = None
        for sample_index, alpha_raw in enumerate(alphas):
            alpha = float(alpha_raw)
            position = (1.0 - alpha) * start + alpha * end
            z_axis = normalize((1.0 - alpha) * start_z + alpha * end_z)
            collides, distance_mm, hit_vertices = self.check_pose(position, z_axis)
            min_distance_mm = min(min_distance_mm, distance_mm)
            total_hit_vertices += hit_vertices
            if collides and first_sample is None:
                first_sample = int(sample_index)
                first_alpha = alpha

        return SegmentCollisionResult(
            collides=first_sample is not None,
            min_distance_mm=min_distance_mm,
            hit_vertices=total_hit_vertices,
            sample_index=first_sample,
            alpha=first_alpha,
        )
