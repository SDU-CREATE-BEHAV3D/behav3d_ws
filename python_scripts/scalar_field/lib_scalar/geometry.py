#!/usr/bin/env python3
"""Geometry input/preparation helpers for scalar-field pipeline.

This module ensures the rest of the pipeline receives:
- valid triangle meshes,
- consistent indexing,
- consistent coordinate transforms.
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
