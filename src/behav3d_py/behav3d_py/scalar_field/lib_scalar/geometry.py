#!/usr/bin/env python3
"""Geometry input/preparation helpers for scalar-field pipeline.

This module ensures the rest of the pipeline receives:
- valid triangle meshes,
- consistent indexing,
- consistent coordinate transforms,
- common mesh-surface sampling utilities.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import open3d as o3d
import potpourri3d as pp3d

from .types import MeshData


def compact_triangle_mesh(vertices: np.ndarray, faces: np.ndarray) -> MeshData:
    """Remove unreferenced vertices and remap faces to compact indices."""
    used = np.unique(faces.reshape(-1))
    if used.size == vertices.shape[0]:
        return MeshData(vertices=vertices, faces=faces.astype(np.int32), dropped_vertices=0)

    remap = -np.ones(vertices.shape[0], dtype=np.int64)
    remap[used] = np.arange(used.size, dtype=np.int64)
    compact_vertices = vertices[used]
    compact_faces = remap[faces].astype(np.int32)
    dropped = int(vertices.shape[0] - compact_vertices.shape[0])
    return MeshData(vertices=compact_vertices, faces=compact_faces, dropped_vertices=dropped)


def load_triangle_mesh_arrays(path: Path) -> MeshData:
    """Load triangle mesh from file into numpy arrays and compact it."""
    if not path.is_file():
        raise FileNotFoundError(f"Mesh not found: {path}")

    vertices, faces = pp3d.read_mesh(str(path))
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError(f"Unexpected vertex array shape: {vertices.shape}")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"Mesh must be triangles, got faces shape: {faces.shape}")

    return compact_triangle_mesh(vertices, faces)


def load_triangle_mesh_legacy(path: Path) -> o3d.geometry.TriangleMesh:
    """Load Open3D legacy triangle mesh (useful for visualization/raycast scene)."""
    mesh = o3d.io.read_triangle_mesh(str(path))
    if not mesh.has_triangles():
        raise ValueError(f"Mesh has no triangles: {path}")
    return mesh


def apply_scale_and_offset(
    vertices: np.ndarray,
    scale: float,
    offset_xyz: tuple[float, float, float],
) -> np.ndarray:
    """Apply global scale and translation to mesh vertices."""
    if scale <= 0.0:
        raise ValueError(f"scale must be > 0, got {scale}")

    out = vertices.copy() * float(scale)
    out[:, 0] += float(offset_xyz[0])
    out[:, 1] += float(offset_xyz[1])
    out[:, 2] += float(offset_xyz[2])
    return out


def sample_vertex_scalar_on_surface(
    query_points: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    vertex_scalar: np.ndarray,
) -> np.ndarray:
    """Interpolate per-vertex scalar values at query points on/near mesh surface."""
    if query_points.ndim != 2 or query_points.shape[1] != 3:
        raise ValueError("query_points must have shape (N, 3)")
    if mesh_vertices.ndim != 2 or mesh_vertices.shape[1] != 3:
        raise ValueError("mesh_vertices must have shape (V, 3)")
    if mesh_faces.ndim != 2 or mesh_faces.shape[1] != 3:
        raise ValueError("mesh_faces must have shape (F, 3)")
    if vertex_scalar.ndim != 1 or vertex_scalar.shape[0] != mesh_vertices.shape[0]:
        raise ValueError("vertex_scalar must have shape (V,) matching mesh_vertices")

    n = query_points.shape[0]
    if n == 0:
        return np.zeros((0,), dtype=np.float64)

    v = o3d.core.Tensor(mesh_vertices.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    f = o3d.core.Tensor(mesh_faces.astype(np.int32), dtype=o3d.core.Dtype.Int32)
    tmesh = o3d.t.geometry.TriangleMesh(v, f)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tmesh)

    q = o3d.core.Tensor(query_points.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    out = scene.compute_closest_points(q)
    tri_ids = out["primitive_ids"].numpy().astype(np.int64).reshape(-1)
    uv = out["primitive_uvs"].numpy().astype(np.float64).reshape(-1, 2)

    sampled = np.full((n,), np.nan, dtype=np.float64)
    tri_valid = (
        (tri_ids >= 0)
        & (tri_ids < mesh_faces.shape[0])
        & np.isfinite(uv[:, 0])
        & np.isfinite(uv[:, 1])
    )
    if np.any(tri_valid):
        tri = mesh_faces[tri_ids[tri_valid]]
        u = uv[tri_valid, 0]
        vv = uv[tri_valid, 1]
        w = 1.0 - u - vv
        sampled[tri_valid] = (
            w * vertex_scalar[tri[:, 0]]
            + u * vertex_scalar[tri[:, 1]]
            + vv * vertex_scalar[tri[:, 2]]
        )

    invalid_idx = np.flatnonzero(~np.isfinite(sampled))
    for i in invalid_idx:
        d2 = np.sum((mesh_vertices - query_points[i]) ** 2, axis=1)
        sampled[i] = float(vertex_scalar[int(np.argmin(d2))])

    return sampled
