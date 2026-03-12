#!/usr/bin/env python3
"""Compute base scalar preference on a field mesh using heat method.

This module answers:
"Where is the field stronger/weaker over the template geometry?"

It does not evaluate collision/viability against scan geometry.
"""

from __future__ import annotations

import numpy as np
import potpourri3d as pp3d

from .types import HeatField


def choose_seed_vertex(vertices: np.ndarray) -> int:
    """Pick a stable single-source seed near mesh centroid."""
    centroid = vertices.mean(axis=0)
    d2 = np.sum((vertices - centroid) ** 2, axis=1)
    return int(np.argmin(d2))


def choose_seed_vertices_below_level(vertices: np.ndarray, level: float) -> np.ndarray:
    """Pick multi-source seeds by thresholding vertex z-coordinate."""
    return np.flatnonzero(vertices[:, 2] < float(level)).astype(np.int64)


def normalize_scalar_field(values: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Normalize scalar values to [0,1] for scoring/visualization."""
    finite = np.isfinite(values)
    if not np.any(finite):
        raise ValueError("Scalar field has no finite values.")

    v_min = float(np.min(values[finite]))
    v_max = float(np.max(values[finite]))
    denom = max(v_max - v_min, 1e-12)

    norm = np.ones_like(values, dtype=np.float64)
    norm[finite] = (values[finite] - v_min) / denom
    return norm, v_min, v_max


def compute_heat_field(
    vertices: np.ndarray,
    faces: np.ndarray,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
) -> HeatField:
    """Run heat-method distance solve and package scalar field outputs.

    Returns both raw distances and normalized values plus descriptive stats.
    """
    solver = pp3d.MeshHeatMethodDistanceSolver(vertices, faces, t_coef=t_coef, use_robust=True)

    if seed_level is not None:
        seed_vertices = choose_seed_vertices_below_level(vertices, seed_level)
        if seed_vertices.size == 0:
            raise ValueError(
                f"No seed vertices found for z < {seed_level} (mesh z min/max: "
                f"{float(np.min(vertices[:, 2])):.6f}/{float(np.max(vertices[:, 2])):.6f})"
            )
        dist = solver.compute_distance_multisource(seed_vertices.tolist())
        seed_info = f"seed_level(z<{seed_level}): count={seed_vertices.size}"
    else:
        if seed is None:
            seed = choose_seed_vertex(vertices)
        if seed < 0 or seed >= vertices.shape[0]:
            raise ValueError(f"Seed index out of range: {seed} (num vertices={vertices.shape[0]})")
        dist = solver.compute_distance(seed)
        seed_info = f"seed_vertex: {seed}"

    if dist.shape[0] != vertices.shape[0]:
        raise RuntimeError(
            f"Distance field size mismatch: len(dist)={dist.shape[0]} vs vertices={vertices.shape[0]}"
        )

    norm, v_min, v_max = normalize_scalar_field(dist)
    return HeatField(
        dist=dist,
        norm=norm,
        min_value=v_min,
        max_value=v_max,
        mean_value=float(np.mean(dist)),
        seed_info=seed_info,
    )
