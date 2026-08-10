#!/usr/bin/env python3
"""Reusable scalar-field pipeline library."""

from .agent_walk import AgentWalkConfig, AgentWalkResult, run_agent_walk
from .bead_profile import (
    WIDTH_MAP_KEY,
    load_normalized_width_map,
    minimum_center_distance,
    normalized_width_to_mm,
    rounded_cylinder_volume_mm3,
    scale_requested_volume_mm3,
)
from .extract_phi_contour import (
    compute_geodesic_from_phi_contour,
    contour_seed_vertices_from_phi,
    extract_offset_phi_contour,
    extract_phi_contour,
    extract_phi_contour_with_offset,
)
from .generate_print_points import generate_print_points
from .geometry import (
    apply_scale_and_offset,
    compute_vertex_normals,
    compute_vertex_scalar_gradient,
    compute_vertex_tangent_axes_from_scalar,
    load_triangle_mesh_arrays,
    load_triangle_mesh_legacy,
    sample_tangent_axes_on_surface_from_scalar,
    sample_vertex_scalar_on_surface,
)
from .loop_simulation import (
    apply_simulated_beads,
    compute_offset_contour_stage,
    generate_step_candidates,
    position_field_with_attempts,
)
from .print_targets import (
    OrientedLineTargets,
    build_candidate_segment_targets,
    offset_line_target_starts,
    resolve_target_output_mode,
    build_oriented_line_targets,
    orient_points_with_tangent,
    write_fixed_z_targets_yaml,
    write_line_targets_yaml,
    write_point_targets_yaml,
)
from .target_rules import (
    TARGET_NORMAL_FLIP_BEAD_HEIGHT_FRACTION,
    TARGET_NORMAL_FLIP_DOT_THRESHOLD,
    apply_secondary_target_rules,
    remove_close_endpoint_targets,
    replace_low_continuity_target_segments,
)
from .compute_heat_field import compute_heat_field
from .compute_phi_mask import compute_phi_mask, evaluate_fixed_pose, make_scan_scene
from .position_field import default_xy_search_bounds, make_axis_samples, position_field
from .types import HeatField, MeshData, PoseResult, PrintPointSet
from .viz import (
    compute_scene_bounds,
    make_line_set,
    make_point_cloud,
    make_segment_line_set,
    make_target_orientation_sticks,
    yellow_to_red_colors,
)

__all__ = [
    "MeshData",
    "HeatField",
    "PoseResult",
    "PrintPointSet",
    "AgentWalkConfig",
    "AgentWalkResult",
    "run_agent_walk",
    "WIDTH_MAP_KEY",
    "load_normalized_width_map",
    "normalized_width_to_mm",
    "minimum_center_distance",
    "rounded_cylinder_volume_mm3",
    "scale_requested_volume_mm3",
    "OrientedLineTargets",
    "orient_points_with_tangent",
    "build_candidate_segment_targets",
    "offset_line_target_starts",
    "resolve_target_output_mode",
    "build_oriented_line_targets",
    "write_point_targets_yaml",
    "write_fixed_z_targets_yaml",
    "write_line_targets_yaml",
    "TARGET_NORMAL_FLIP_DOT_THRESHOLD",
    "TARGET_NORMAL_FLIP_BEAD_HEIGHT_FRACTION",
    "apply_secondary_target_rules",
    "replace_low_continuity_target_segments",
    "remove_close_endpoint_targets",
    "load_triangle_mesh_arrays",
    "load_triangle_mesh_legacy",
    "apply_scale_and_offset",
    "compute_vertex_normals",
    "compute_vertex_scalar_gradient",
    "compute_vertex_tangent_axes_from_scalar",
    "sample_tangent_axes_on_surface_from_scalar",
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
    "position_field_with_attempts",
    "compute_offset_contour_stage",
    "generate_step_candidates",
    "apply_simulated_beads",
    "make_axis_samples",
    "default_xy_search_bounds",
    "position_field",
    "extract_phi_contour",
    "yellow_to_red_colors",
    "make_point_cloud",
    "make_line_set",
    "make_segment_line_set",
    "make_target_orientation_sticks",
    "compute_scene_bounds",
]
