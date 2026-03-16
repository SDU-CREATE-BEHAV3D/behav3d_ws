#!/usr/bin/env python3
"""Reusable scalar-field pipeline library."""

from .extract_phi_contour import (
    compute_geodesic_from_phi_contour,
    contour_seed_vertices_from_phi,
    extract_offset_phi_contour,
    extract_phi_contour,
    extract_phi_contour_with_offset,
)
from .generate_print_points import generate_print_points
from .generate_print_points_phi_lift import generate_print_points_phi_lift
from .geometry import (
    apply_scale_and_offset,
    load_triangle_mesh_arrays,
    load_triangle_mesh_legacy,
    sample_vertex_scalar_on_surface,
)
from .compute_heat_field import compute_heat_field
from .compute_phi_mask import compute_phi_mask, evaluate_fixed_pose, make_scan_scene
from .position_field import default_xy_search_bounds, make_axis_samples, position_field
from .types import HeatField, LiftedPrintPointSet, MeshData, PoseResult, PrintPointSet
from .viz import compute_scene_bounds, make_line_set, make_point_cloud, yellow_to_red_colors

__all__ = [
    "MeshData",
    "HeatField",
    "PoseResult",
    "PrintPointSet",
    "LiftedPrintPointSet",
    "load_triangle_mesh_arrays",
    "load_triangle_mesh_legacy",
    "apply_scale_and_offset",
    "sample_vertex_scalar_on_surface",
    "compute_heat_field",
    "make_scan_scene",
    "compute_phi_mask",
    "evaluate_fixed_pose",
    "contour_seed_vertices_from_phi",
    "compute_geodesic_from_phi_contour",
    "extract_offset_phi_contour",
    "extract_phi_contour_with_offset",
    "generate_print_points",
    "generate_print_points_phi_lift",
    "make_axis_samples",
    "default_xy_search_bounds",
    "position_field",
    "extract_phi_contour",
    "yellow_to_red_colors",
    "make_point_cloud",
    "make_line_set",
    "compute_scene_bounds",
]
