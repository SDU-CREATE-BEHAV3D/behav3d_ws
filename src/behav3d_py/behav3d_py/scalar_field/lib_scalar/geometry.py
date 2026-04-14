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


def _validate_mesh_arrays(
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
) -> None:
    if mesh_vertices.ndim != 2 or mesh_vertices.shape[1] != 3:
        raise ValueError("mesh_vertices must have shape (V, 3)")
    if mesh_faces.ndim != 2 or mesh_faces.shape[1] != 3:
        raise ValueError("mesh_faces must have shape (F, 3)")


def _closest_triangles_and_uv(
    query_points: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Find closest triangle id + barycentric UV for each query point."""
    if query_points.ndim != 2 or query_points.shape[1] != 3:
        raise ValueError("query_points must have shape (N, 3)")
    _validate_mesh_arrays(mesh_vertices, mesh_faces)

    v = o3d.core.Tensor(mesh_vertices.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    f = o3d.core.Tensor(mesh_faces.astype(np.int32), dtype=o3d.core.Dtype.Int32)
    tmesh = o3d.t.geometry.TriangleMesh(v, f)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tmesh)

    q = o3d.core.Tensor(query_points.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    out = scene.compute_closest_points(q)
    tri_ids = out["primitive_ids"].numpy().astype(np.int64).reshape(-1)
    uv = out["primitive_uvs"].numpy().astype(np.float64).reshape(-1, 2)
    return tri_ids, uv


def project_points_to_surface(
    query_points: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
) -> np.ndarray:
    """Project points to nearest points on mesh surface via raycasting scene."""
    if query_points.ndim != 2 or query_points.shape[1] != 3:
        raise ValueError("query_points must have shape (N, 3)")
    _validate_mesh_arrays(mesh_vertices, mesh_faces)

    n = query_points.shape[0]
    if n == 0:
        return np.zeros((0, 3), dtype=np.float64)

    v = o3d.core.Tensor(mesh_vertices.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    f = o3d.core.Tensor(mesh_faces.astype(np.int32), dtype=o3d.core.Dtype.Int32)
    tmesh = o3d.t.geometry.TriangleMesh(v, f)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tmesh)

    q = o3d.core.Tensor(query_points.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    out = scene.compute_closest_points(q)
    return out["points"].numpy().astype(np.float64).reshape(-1, 3)


def _normalize_rows(vectors: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    out = np.asarray(vectors, dtype=np.float64).copy()
    nrm = np.linalg.norm(out, axis=1)
    valid = nrm > float(eps)
    out[valid] /= nrm[valid, None]
    out[~valid] = 0.0
    return out


def clamp_vectors_to_cone(
    vectors: np.ndarray,
    *,
    max_tilt_deg: float,
    cone_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> np.ndarray:
    """Clamp direction vectors to a cone around a given axis.

    Input vectors are treated as directions and output vectors are unit-length.
    `max_tilt_deg` is the cone semi-angle (tilt from cone axis).
    """
    if vectors.ndim != 2 or vectors.shape[1] != 3:
        raise ValueError("vectors must have shape (N, 3)")

    axis = np.asarray(cone_axis, dtype=np.float64).reshape(3)
    axis_n = float(np.linalg.norm(axis))
    if axis_n <= 1e-12:
        raise ValueError("cone_axis norm must be > 0")
    axis /= axis_n

    max_tilt_rad = np.deg2rad(float(np.clip(max_tilt_deg, 0.0, 179.9)))
    cos_max = float(np.cos(max_tilt_rad))
    sin_max = float(np.sin(max_tilt_rad))

    out = np.zeros_like(vectors, dtype=np.float64)
    for i in range(vectors.shape[0]):
        v = np.asarray(vectors[i], dtype=np.float64)
        n = float(np.linalg.norm(v))
        if n <= 1e-12:
            out[i] = axis
            continue
        u = v / n
        c = float(np.dot(u, axis))
        if c >= cos_max:
            out[i] = u
            continue
        p = u - c * axis
        pn = float(np.linalg.norm(p))
        if pn <= 1e-12:
            # Deterministic fallback axis orthogonal to cone axis.
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            if abs(float(np.dot(ref, axis))) > 0.9:
                ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            p = np.cross(axis, ref)
            pn = float(np.linalg.norm(p))
        p_hat = p / max(pn, 1e-12)
        out[i] = cos_max * axis + sin_max * p_hat

    return _normalize_rows(out)


def _fallback_tangent_from_normals(normals: np.ndarray) -> np.ndarray:
    """Build deterministic tangent fallback orthogonal to normals."""
    n = _normalize_rows(normals)
    ref = np.tile(np.array([1.0, 0.0, 0.0], dtype=np.float64), (n.shape[0], 1))
    near_parallel = np.abs(np.sum(n * ref, axis=1)) > 0.9
    ref[near_parallel] = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    t = np.cross(ref, n)
    return _normalize_rows(t)


def compute_vertex_normals(
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
) -> np.ndarray:
    """Area-weighted per-vertex normals."""
    _validate_mesh_arrays(mesh_vertices, mesh_faces)
    v0 = mesh_vertices[mesh_faces[:, 0]]
    v1 = mesh_vertices[mesh_faces[:, 1]]
    v2 = mesh_vertices[mesh_faces[:, 2]]
    face_normals = np.cross(v1 - v0, v2 - v0)

    vert_normals = np.zeros((mesh_vertices.shape[0], 3), dtype=np.float64)
    for col in range(3):
        idx = mesh_faces[:, col]
        np.add.at(vert_normals, idx, face_normals)
    return _normalize_rows(vert_normals)


def compute_vertex_scalar_gradient(
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    vertex_scalar: np.ndarray,
) -> np.ndarray:
    """Area-weighted per-vertex gradient of a scalar defined on mesh vertices."""
    _validate_mesh_arrays(mesh_vertices, mesh_faces)
    if vertex_scalar.ndim != 1 or vertex_scalar.shape[0] != mesh_vertices.shape[0]:
        raise ValueError("vertex_scalar must have shape (V,) matching mesh_vertices")

    v0 = mesh_vertices[mesh_faces[:, 0]]
    v1 = mesh_vertices[mesh_faces[:, 1]]
    v2 = mesh_vertices[mesh_faces[:, 2]]

    n_unnorm = np.cross(v1 - v0, v2 - v0)
    two_area = np.linalg.norm(n_unnorm, axis=1)
    valid = two_area > 1e-12
    if not np.any(valid):
        raise ValueError("Degenerate mesh: all triangle areas are near zero.")

    n_hat = np.zeros_like(n_unnorm)
    n_hat[valid] = n_unnorm[valid] / two_area[valid, None]

    f0 = vertex_scalar[mesh_faces[:, 0]]
    f1 = vertex_scalar[mesh_faces[:, 1]]
    f2 = vertex_scalar[mesh_faces[:, 2]]

    face_grad = np.zeros((mesh_faces.shape[0], 3), dtype=np.float64)
    face_grad[valid] = (
        f0[valid, None] * np.cross(n_hat[valid], v2[valid] - v1[valid])
        + f1[valid, None] * np.cross(n_hat[valid], v0[valid] - v2[valid])
        + f2[valid, None] * np.cross(n_hat[valid], v1[valid] - v0[valid])
    ) / two_area[valid, None]

    face_area = 0.5 * two_area
    vert_grad = np.zeros((mesh_vertices.shape[0], 3), dtype=np.float64)
    vert_weight = np.zeros((mesh_vertices.shape[0],), dtype=np.float64)
    for col in range(3):
        idx = mesh_faces[:, col]
        np.add.at(vert_grad, idx, face_grad * face_area[:, None])
        np.add.at(vert_weight, idx, face_area)

    nonzero = vert_weight > 1e-12
    vert_grad[nonzero] /= vert_weight[nonzero, None]
    return vert_grad


def compute_vertex_tangent_axes_from_scalar(
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    vertex_scalar: np.ndarray,
    *,
    tangent_sign: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute local tangent axes from a scalar field.

    Returns:
    - tangent_t: projected gradient direction on tangent plane
    - orthogonal_b: 90-deg axis in tangent plane (cross(normal, tangent))
    - normal_n: surface normal
    """
    normals = compute_vertex_normals(mesh_vertices, mesh_faces)
    grad = compute_vertex_scalar_gradient(mesh_vertices, mesh_faces, vertex_scalar)

    sign = 1.0 if float(tangent_sign) >= 0.0 else -1.0
    t_raw = sign * grad
    t_proj = t_raw - np.sum(t_raw * normals, axis=1, keepdims=True) * normals
    t_norm = np.linalg.norm(t_proj, axis=1)
    bad = t_norm <= 1e-12
    t = _normalize_rows(t_proj)
    if np.any(bad):
        t_fallback = _fallback_tangent_from_normals(normals)
        t[bad] = t_fallback[bad]
        t = _normalize_rows(t)

    b = np.cross(normals, t)
    b = _normalize_rows(b)
    return t, b, normals


def sample_tangent_axes_on_surface_from_scalar(
    query_points: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    vertex_scalar: np.ndarray,
    *,
    tangent_sign: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample tangent/orthogonal/normal axes at arbitrary points on mesh surface."""
    if query_points.ndim != 2 or query_points.shape[1] != 3:
        raise ValueError("query_points must have shape (N, 3)")
    _validate_mesh_arrays(mesh_vertices, mesh_faces)
    if vertex_scalar.ndim != 1 or vertex_scalar.shape[0] != mesh_vertices.shape[0]:
        raise ValueError("vertex_scalar must have shape (V,) matching mesh_vertices")

    n_query = query_points.shape[0]
    if n_query == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty, empty

    t_v, b_v, n_v = compute_vertex_tangent_axes_from_scalar(
        mesh_vertices,
        mesh_faces,
        vertex_scalar,
        tangent_sign=tangent_sign,
    )

    tri_ids, uv = _closest_triangles_and_uv(query_points, mesh_vertices, mesh_faces)
    tri_valid = (
        (tri_ids >= 0)
        & (tri_ids < mesh_faces.shape[0])
        & np.isfinite(uv[:, 0])
        & np.isfinite(uv[:, 1])
    )

    t_q = np.zeros((n_query, 3), dtype=np.float64)
    n_q = np.zeros((n_query, 3), dtype=np.float64)

    if np.any(tri_valid):
        tri = mesh_faces[tri_ids[tri_valid]]
        u = uv[tri_valid, 0]
        v = uv[tri_valid, 1]
        w = 1.0 - u - v

        n_q[tri_valid] = (
            w[:, None] * n_v[tri[:, 0]]
            + u[:, None] * n_v[tri[:, 1]]
            + v[:, None] * n_v[tri[:, 2]]
        )
        n_q[tri_valid] = _normalize_rows(n_q[tri_valid])

        t_q[tri_valid] = (
            w[:, None] * t_v[tri[:, 0]]
            + u[:, None] * t_v[tri[:, 1]]
            + v[:, None] * t_v[tri[:, 2]]
        )
        # Reproject after interpolation to enforce tangency.
        t_q[tri_valid] = t_q[tri_valid] - np.sum(t_q[tri_valid] * n_q[tri_valid], axis=1, keepdims=True) * n_q[tri_valid]
        t_q[tri_valid] = _normalize_rows(t_q[tri_valid])

    invalid_idx = np.flatnonzero(~tri_valid)
    if invalid_idx.size > 0:
        for i in invalid_idx:
            d2 = np.sum((mesh_vertices - query_points[i]) ** 2, axis=1)
            k = int(np.argmin(d2))
            t_q[i] = t_v[k]
            n_q[i] = n_v[k]

    t_bad = np.linalg.norm(t_q, axis=1) <= 1e-12
    if np.any(t_bad):
        t_fb = _fallback_tangent_from_normals(n_q)
        t_q[t_bad] = t_fb[t_bad]
        t_q = _normalize_rows(t_q)

    b_q = np.cross(n_q, t_q)
    b_q = _normalize_rows(b_q)
    return t_q, b_q, n_q


def sample_vertex_scalar_on_surface(
    query_points: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    vertex_scalar: np.ndarray,
) -> np.ndarray:
    """Interpolate per-vertex scalar values at query points on/near mesh surface."""
    if query_points.ndim != 2 or query_points.shape[1] != 3:
        raise ValueError("query_points must have shape (N, 3)")
    _validate_mesh_arrays(mesh_vertices, mesh_faces)
    if vertex_scalar.ndim != 1 or vertex_scalar.shape[0] != mesh_vertices.shape[0]:
        raise ValueError("vertex_scalar must have shape (V,) matching mesh_vertices")

    n = query_points.shape[0]
    if n == 0:
        return np.zeros((0,), dtype=np.float64)

    tri_ids, uv = _closest_triangles_and_uv(query_points, mesh_vertices, mesh_faces)

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
