#!/usr/bin/env python3
"""Alternative print-point selection on existing polyline using heat minima.

Workflow:
1) Interpolate `heat.norm` on the source polyline using triangle barycentrics.
2) Apply spacing suppression on the polyline graph (same as default selector).
3) Lift selected points by +Z using a configured print height.
"""

from __future__ import annotations

import numpy as np

from .generate_print_points import generate_print_points
from .geometry import sample_vertex_scalar_on_surface
from .types import LiftedPrintPointSet


def generate_print_points_phi_lift(
    polyline_points: np.ndarray,
    polyline_lines: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    heat_norm: np.ndarray,
    *,
    print_height: float,
    count: int = 7,
    min_spacing: float = 0.016,
    merge_tol: float = 1e-6,
    point_valid_mask: np.ndarray | None = None,
) -> LiftedPrintPointSet:
    """Select heat minima on source polyline and lift them in +Z."""
    h = float(print_height)
    if h < 0.0:
        raise ValueError(f"print_height must be >= 0, got {print_height}")
    if heat_norm.shape[0] != field_vertices_world.shape[0]:
        raise ValueError("heat_norm length must match field_vertices_world length")

    polyline_heat = sample_vertex_scalar_on_surface(
        query_points=polyline_points,
        mesh_vertices=field_vertices_world,
        mesh_faces=field_faces,
        vertex_scalar=heat_norm,
    )

    selected = generate_print_points(
        polyline_points=polyline_points,
        polyline_lines=polyline_lines,
        # We already interpolated scalar at each candidate polyline point.
        # Reuse the existing selector by sampling those values back on polyline vertices.
        field_vertices_world=polyline_points,
        field_scalar=polyline_heat,
        count=int(count),
        min_spacing=float(min_spacing),
        merge_tol=float(merge_tol),
        point_valid_mask=point_valid_mask,
        extra_points=None,
    )

    lifted = selected.points.copy()
    if lifted.shape[0] > 0:
        lifted[:, 2] += h
        nearest_field_vertex_indices = np.zeros((selected.points.shape[0],), dtype=np.int32)
        for i in range(selected.points.shape[0]):
            d2 = np.sum((field_vertices_world - selected.points[i]) ** 2, axis=1)
            nearest_field_vertex_indices[i] = int(np.argmin(d2))
    else:
        nearest_field_vertex_indices = np.zeros((0,), dtype=np.int32)

    return LiftedPrintPointSet(
        source_points=selected.points,
        lifted_points=lifted,
        source_values=selected.scalar_values,
        polyline_indices=selected.polyline_indices,
        nearest_field_vertex_indices=nearest_field_vertex_indices,
        requested_count=selected.requested_count,
        min_spacing=selected.min_spacing,
        available_vertices=selected.available_vertices,
        lift_height=h,
        augmented_vertices=selected.augmented_vertices,
    )
