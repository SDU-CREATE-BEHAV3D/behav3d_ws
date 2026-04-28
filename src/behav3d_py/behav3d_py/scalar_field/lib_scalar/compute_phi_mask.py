#!/usr/bin/env python3
"""Compute signed clearance (`phi`) and viability against scan geometry.

"Given a placed field, which points are free vs. buried relative to the scan"

Current phi model is z-up based:
    phi = z_field - z_scan - clearance
"""

from __future__ import annotations

import numpy as np
import open3d as o3d

from .types import PoseResult


def make_scan_scene(scan_mesh: o3d.geometry.TriangleMesh) -> tuple[o3d.t.geometry.RaycastingScene, float]:
    """Build raycasting acceleration structure for fast geometry queries."""
    vertices = np.asarray(scan_mesh.vertices)
    if vertices.size == 0:
        raise ValueError("Scan mesh has no vertices.")
    z_top = float(np.max(vertices[:, 2]) + 1e3)

    scene = o3d.t.geometry.RaycastingScene()
    scan_tmesh = o3d.t.geometry.TriangleMesh.from_legacy(scan_mesh)
    scene.add_triangles(scan_tmesh)
    return scene, z_top


def query_scan_z_with_vertical_rays(
    scene: o3d.t.geometry.RaycastingScene,
    points_world: np.ndarray,
    z_top: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate scan height z(x,y) by downward vertical raycasting."""
    n = points_world.shape[0]
    rays = np.zeros((n, 6), dtype=np.float32)
    rays[:, 0] = points_world[:, 0]
    rays[:, 1] = points_world[:, 1]
    rays[:, 2] = z_top
    rays[:, 5] = -1.0

    hits = scene.cast_rays(o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32))
    t_hit = hits["t_hit"].numpy()
    valid = np.isfinite(t_hit)

    z_scan = np.full(n, np.nan, dtype=np.float64)
    z_scan[valid] = z_top - t_hit[valid]
    return z_scan, valid


def compute_phi_mask(
    scene: o3d.t.geometry.RaycastingScene,
    z_top: float,
    field_vertices_world: np.ndarray,
    offset_xyz: tuple[float, float, float],
    clearance: float,
    iso_level: float,
) -> PoseResult:
    """Compute phi values and viable mask for a field already placed in world."""
    n = field_vertices_world.shape[0]
    z_scan, has_hit = query_scan_z_with_vertical_rays(scene, field_vertices_world, z_top=z_top)

    phi = np.full(n, -np.inf, dtype=np.float64)
    phi[has_hit] = field_vertices_world[has_hit, 2] - z_scan[has_hit] - float(clearance)
    viable = (phi > float(iso_level)) & has_hit

    base_world_z = float(np.min(field_vertices_world[:, 2]))
    base_dz = np.full(n, np.inf, dtype=np.float64)
    base_dz[has_hit] = field_vertices_world[has_hit, 2] - z_scan[has_hit]

    return PoseResult(
        offset_xyz=(float(offset_xyz[0]), float(offset_xyz[1]), float(offset_xyz[2])),
        field_vertices_world=field_vertices_world,
        z_scan=z_scan,
        has_hit=has_hit,
        phi=phi,
        viable=viable,
        base_dz=base_dz,
        base_world_z=base_world_z,
        hit_count=int(np.count_nonzero(has_hit)),
        viable_count=int(np.count_nonzero(viable)),
    )


def evaluate_fixed_pose(
    scene: o3d.t.geometry.RaycastingScene,
    z_top: float,
    field_vertices_scaled: np.ndarray,
    offset_xyz: tuple[float, float, float],
    clearance: float,
    iso_level: float,
) -> PoseResult:
    """Convenience wrapper: apply fixed offset and then compute phi mask."""
    world = field_vertices_scaled.copy()
    world[:, 0] += float(offset_xyz[0])
    world[:, 1] += float(offset_xyz[1])
    world[:, 2] += float(offset_xyz[2])
    return compute_phi_mask(
        scene=scene,
        z_top=z_top,
        field_vertices_world=world,
        offset_xyz=offset_xyz,
        clearance=clearance,
        iso_level=iso_level,
    )
