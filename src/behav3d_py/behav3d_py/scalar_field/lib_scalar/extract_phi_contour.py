#!/usr/bin/env python3
"""Extract `phi` contours and optional geodesic offsets on the field mesh.

This module is the contour-centric API:
- extract `phi = iso` boundary curves,
- optionally derive geodesic offset curves from that boundary.
"""

from __future__ import annotations

import numpy as np
import potpourri3d as pp3d


def extract_phi_contour(
    vertices: np.ndarray,
    faces: np.ndarray,
    scalar: np.ndarray,
    iso: float = 0.0,
    eps: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate phi crossings on triangle edges and return line segments."""
    seg_points: list[np.ndarray] = []
    seg_lines: list[list[int]] = []
    next_idx = 0

    for tri in faces:
        ids = [int(tri[0]), int(tri[1]), int(tri[2])]
        vals = np.array([scalar[ids[0]], scalar[ids[1]], scalar[ids[2]]], dtype=np.float64)
        pts = np.array([vertices[ids[0]], vertices[ids[1]], vertices[ids[2]]], dtype=np.float64)

        tri_cross: list[np.ndarray] = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            va = vals[a] - iso
            vb = vals[b] - iso
            if not np.isfinite(va) or not np.isfinite(vb):
                continue
            if abs(va) < eps and abs(vb) < eps:
                continue
            if (va > 0.0 and vb > 0.0) or (va < 0.0 and vb < 0.0):
                continue

            denom = va - vb
            if abs(denom) < eps:
                continue

            t = va / (va - vb)
            t = float(np.clip(t, 0.0, 1.0))
            p = pts[a] + t * (pts[b] - pts[a])
            tri_cross.append(p)

        if len(tri_cross) == 2:
            p0, p1 = tri_cross
            seg_points.append(p0)
            seg_points.append(p1)
            seg_lines.append([next_idx, next_idx + 1])
            next_idx += 2
        elif len(tri_cross) > 2:
            p0, p1 = tri_cross[0], tri_cross[1]
            seg_points.append(p0)
            seg_points.append(p1)
            seg_lines.append([next_idx, next_idx + 1])
            next_idx += 2

    if not seg_points:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 2), dtype=np.int32)
    return np.vstack(seg_points), np.asarray(seg_lines, dtype=np.int32)


def contour_seed_vertices_from_phi(
    faces: np.ndarray,
    phi: np.ndarray,
    iso_level: float = 0.0,
    eps: float = 1e-12,
) -> np.ndarray:
    """Collect mesh vertices adjacent to edges where `phi` crosses `iso_level`."""
    seeds: set[int] = set()
    shifted = phi - float(iso_level)

    for tri in faces:
        i0, i1, i2 = int(tri[0]), int(tri[1]), int(tri[2])
        ids = (i0, i1, i2)
        vals = (shifted[i0], shifted[i1], shifted[i2])

        for a, b in ((0, 1), (1, 2), (2, 0)):
            va = vals[a]
            vb = vals[b]
            if not (np.isfinite(va) and np.isfinite(vb)):
                continue

            same_zero = (abs(va) <= eps) and (abs(vb) <= eps)
            crosses = (va * vb) < 0.0
            touches = (abs(va) <= eps) != (abs(vb) <= eps)
            if same_zero or crosses or touches:
                seeds.add(ids[a])
                seeds.add(ids[b])

    if not seeds:
        return np.zeros((0,), dtype=np.int64)
    return np.asarray(sorted(seeds), dtype=np.int64)


def compute_geodesic_from_phi_contour(
    vertices: np.ndarray,
    faces: np.ndarray,
    phi: np.ndarray,
    iso_level: float = 0.0,
    t_coef: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute geodesic distance from the `phi=iso_level` contour seed set."""
    seed_vertices = contour_seed_vertices_from_phi(faces=faces, phi=phi, iso_level=iso_level)
    if seed_vertices.size == 0:
        return np.full(vertices.shape[0], np.inf, dtype=np.float64), seed_vertices

    solver = pp3d.MeshHeatMethodDistanceSolver(vertices, faces, t_coef=float(t_coef), use_robust=True)
    geod = solver.compute_distance_multisource(seed_vertices.tolist())
    return geod, seed_vertices


def extract_offset_phi_contour(
    vertices: np.ndarray,
    faces: np.ndarray,
    phi: np.ndarray,
    iso_level: float,
    offset_distance: float,
    toward_unprinted: bool = True,
    t_coef: float = 1.0,
    eps: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract geodesic offset contour from `phi=iso_level`.

    Side convention:
    - unprinted side: phi > iso_level
    - printed side:   phi <= iso_level

    Returns:
    - offset contour points
    - offset contour lines
    - signed geodesic scalar over vertices
    - contour seed vertices used for distance solve
    """
    d = float(offset_distance)
    if d < 0.0:
        raise ValueError(f"offset_distance must be >= 0, got {offset_distance}")

    geod, seed_vertices = compute_geodesic_from_phi_contour(
        vertices=vertices,
        faces=faces,
        phi=phi,
        iso_level=iso_level,
        t_coef=t_coef,
    )
    if seed_vertices.size == 0:
        return (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 2), dtype=np.int32),
            np.full(vertices.shape[0], np.nan, dtype=np.float64),
            seed_vertices,
        )

    side = np.where(phi > float(iso_level), 1.0, -1.0)
    signed_geod = geod * side
    iso_target = d if toward_unprinted else -d

    offset_points, offset_lines = extract_phi_contour(
        vertices=vertices,
        faces=faces,
        scalar=signed_geod,
        iso=iso_target,
        eps=eps,
    )
    return offset_points, offset_lines, signed_geod, seed_vertices


def extract_phi_contour_with_offset(
    vertices: np.ndarray,
    faces: np.ndarray,
    phi: np.ndarray,
    iso_level: float,
    offset_distance: float,
    *,
    toward_unprinted: bool = True,
    offset_t_coef: float = 1.0,
    eps: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """One-shot contour API: extract `phi=iso` and its geodesic offset contour.

    Returns:
    - contour points (phi=iso)
    - contour lines (phi=iso)
    - offset contour points
    - offset contour lines
    - signed geodesic scalar over vertices
    - contour seed vertices used for distance solve
    """
    contour_points, contour_lines = extract_phi_contour(
        vertices=vertices,
        faces=faces,
        scalar=phi,
        iso=float(iso_level),
        eps=eps,
    )
    offset_points, offset_lines, signed_geod, seed_vertices = extract_offset_phi_contour(
        vertices=vertices,
        faces=faces,
        phi=phi,
        iso_level=float(iso_level),
        offset_distance=float(offset_distance),
        toward_unprinted=bool(toward_unprinted),
        t_coef=float(offset_t_coef),
        eps=eps,
    )
    return contour_points, contour_lines, offset_points, offset_lines, signed_geod, seed_vertices
