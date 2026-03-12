#!/usr/bin/env python3
"""Build a geodesic offset curve from the phi contour on the field mesh.

Workflow:
1) Detect seed vertices along the original phi contour (iso crossing).
2) Compute geodesic distance from those seeds over the field mesh.
3) Sign the distance by side of phi (unprinted vs printed).
4) Extract an iso-curve at requested offset distance.
"""

from __future__ import annotations

import numpy as np
import potpourri3d as pp3d

from .extract_phi_contour import extract_phi_contour


def contour_seed_vertices_from_phi(
    faces: np.ndarray,
    phi: np.ndarray,
    iso_level: float = 0.0,
    eps: float = 1e-12,
) -> np.ndarray:
    """Collect mesh vertices adjacent to edges where phi crosses iso."""
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
    """Compute geodesic distance from the phi contour seed set."""
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
    """Extract geodesic offset curve from phi contour.

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
