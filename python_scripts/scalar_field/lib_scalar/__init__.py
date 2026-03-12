#!/usr/bin/env python3
"""Reusable scalar-field pipeline library."""

from .extract_phi_contour import extract_phi_contour
from .geometry import apply_scale_and_offset, load_triangle_mesh_arrays, load_triangle_mesh_legacy
from .compute_heat_field import compute_heat_field
from .compute_phi_mask import compute_phi_mask, evaluate_fixed_pose, make_scan_scene
from .position_field import default_xy_search_bounds, make_axis_samples, position_field
from .types import HeatField, MeshData, PoseResult
from .viz import compute_scene_bounds, make_line_set, make_point_cloud, yellow_to_red_colors

__all__ = [
    "MeshData",
    "HeatField",
    "PoseResult",
    "load_triangle_mesh_arrays",
    "load_triangle_mesh_legacy",
    "apply_scale_and_offset",
    "compute_heat_field",
    "make_scan_scene",
    "compute_phi_mask",
    "evaluate_fixed_pose",
    "make_axis_samples",
    "default_xy_search_bounds",
    "position_field",
    "extract_phi_contour",
    "yellow_to_red_colors",
    "make_point_cloud",
    "make_line_set",
    "compute_scene_bounds",
]
