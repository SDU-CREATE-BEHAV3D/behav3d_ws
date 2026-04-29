#!/usr/bin/env python3
"""Loop simulation helpers for scalar-field print planning.

Stages implemented here:
1) position field over current scan
2) extract phi contour and geodesic offset contour
3) select print points with local (intra-step) bead separation
4) update scan mesh with simulated beads (cylinders/spheres)
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np
import open3d as o3d

from .compute_phi_mask import make_scan_scene, query_scan_z_with_vertical_rays
from .extract_phi_contour import extract_phi_contour_with_offset
from .generate_print_points import generate_print_points
from .position_field import default_xy_search_bounds, make_axis_samples, position_field
from .types import PoseResult


@dataclass(frozen=True)
class ContourStage:
    """Result of contour extraction stage."""

    contour_points: np.ndarray
    contour_lines: np.ndarray
    offset_points: np.ndarray
    offset_lines: np.ndarray
    offset_distance_m: float
    z_valid_mask: np.ndarray


@dataclass(frozen=True)
class CandidateStage:
    """Result of candidate selection stage."""

    points: np.ndarray
    scalar_values: np.ndarray
    available_vertices: int
    z_valid_count: int
    source_points: np.ndarray | None = None
    segment_start_points: np.ndarray | None = None


def position_field_with_attempts(
    scan_mesh: o3d.geometry.TriangleMesh,
    field_vertices_scaled: np.ndarray,
    heat_norm: np.ndarray,
    *,
    clearance: float,
    iso_level: float,
    base_z_offset: float,
    search_step_x: float,
    search_step_y: float,
    positioning_attempts: int,
    search_max_candidates: int,
    require_full_hit: bool = True,
) -> PoseResult:
    """Position field using XY search with bounded retry attempts.

    Attempts expand search bounds by one step per retry around the default range.
    """

    attempts = max(1, int(positioning_attempts))
    scan_vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
    scene, z_top = make_scan_scene(scan_mesh)

    x_min0, x_max0, y_min0, y_max0 = default_xy_search_bounds(
        scan_vertices=scan_vertices,
        field_vertices_scaled=field_vertices_scaled,
        margin_x=0.0,
        margin_y=0.0,
    )

    best_pose: PoseResult | None = None
    best_score = -np.inf
    for attempt in range(attempts):
        grow_x = float(attempt) * float(search_step_x)
        grow_y = float(attempt) * float(search_step_y)
        x_values = make_axis_samples(x_min0 - grow_x, x_max0 + grow_x, float(search_step_x))
        y_values = make_axis_samples(y_min0 - grow_y, y_max0 + grow_y, float(search_step_y))

        total_candidates = int(x_values.size * y_values.size)
        if total_candidates > int(search_max_candidates):
            raise ValueError(
                f"Pose search grid too large on attempt {attempt + 1}: "
                f"{total_candidates} (limit={search_max_candidates})."
            )

        pose = position_field(
            scene=scene,
            z_top=z_top,
            field_vertices_scaled=field_vertices_scaled,
            heat_norm=heat_norm,
            x_values=x_values,
            y_values=y_values,
            clearance=float(clearance),
            iso_level=float(iso_level),
            base_z_offset=float(base_z_offset),
            require_full_hit=bool(require_full_hit),
            verbose=False,
        )

        viable_heat = float(pose.viable_heat if pose.viable_heat is not None else 0.0)
        score = viable_heat + 1e-9 * float(pose.viable_count)
        if score > best_score:
            best_pose = pose
            best_score = score

        if pose.viable_count > 0:
            break

    if best_pose is None:
        raise RuntimeError("No pose produced by position_field search.")
    return best_pose


def compute_offset_contour_stage(
    pose: PoseResult,
    field_faces: np.ndarray,
    *,
    iso_level: float,
    offset_distance_mm: float,
    offset_geodesic_delta_mm: float,
    offset_t_coef: float,
    scan_mesh: o3d.geometry.TriangleMesh,
) -> ContourStage:
    """Extract phi contour + geodesic offset and apply z-validity mask."""

    delta_mm = float(offset_geodesic_delta_mm)
    if delta_mm < 0.0:
        raise ValueError(f"offset_geodesic_delta_mm must be >= 0, got {offset_geodesic_delta_mm}")

    offset_distance_m = 1e-3 * float(offset_distance_mm)
    geodesic_m = 1e-3 * max(0.0, float(offset_distance_mm) - delta_mm)

    contour_points, contour_lines, offset_points, offset_lines, _, _ = extract_phi_contour_with_offset(
        vertices=pose.field_vertices_world,
        faces=field_faces,
        phi=pose.phi,
        iso_level=float(iso_level),
        offset_distance=geodesic_m,
        toward_unprinted=True,
        offset_t_coef=float(offset_t_coef),
    )

    if offset_points.shape[0] == 0:
        z_valid = np.zeros((0,), dtype=bool)
        return ContourStage(
            contour_points=contour_points,
            contour_lines=contour_lines,
            offset_points=offset_points,
            offset_lines=offset_lines,
            offset_distance_m=offset_distance_m,
            z_valid_mask=z_valid,
        )

    scene, z_top = make_scan_scene(scan_mesh)
    z_scan, has_hit = query_scan_z_with_vertical_rays(scene, offset_points, z_top=z_top)
    abs_dz = np.full(offset_points.shape[0], np.inf, dtype=np.float64)
    abs_dz[has_hit] = np.abs(offset_points[has_hit, 2] - z_scan[has_hit])
    z_valid = has_hit & (abs_dz <= offset_distance_m)

    return ContourStage(
        contour_points=contour_points,
        contour_lines=contour_lines,
        offset_points=offset_points,
        offset_lines=offset_lines,
        offset_distance_m=offset_distance_m,
        z_valid_mask=z_valid,
    )


def generate_step_candidates(
    contour: ContourStage,
    field_faces: np.ndarray,
    field_vertices_world: np.ndarray,
    heat_norm: np.ndarray,
    phi: np.ndarray | None = None,
    *,
    mode: str,
    beads_per_step: int,
    bead_separation_mm: float,
    bead_height_mm: float,
    walk_distance_mm: float = 12.0,
    walk_step_mm: float = 1.0,
    walk_max_steps: int = 32,
    walk_tangent_sign: float = 1.0,
    walk_start_fraction: float = 0.25,
    clamp_to_cone: bool = False,
    cone_max_tilt_deg: float = 45.0,
) -> CandidateStage:
    """Generate current-step print points.

    Modes:
    - geodesic: select on offset contour (with z-valid filter)
    - z_lift: select on phi=0 contour and lift +Z by bead height
    - gradient_lift: select on phi=0 contour and displace along scalar tangent
    - gradient_walk: select on phi=0 contour and walk along scalar tangent
    """

    sep_m = 1e-3 * float(bead_separation_mm)
    select_count = max(0, int(beads_per_step))
    mode_norm = str(mode).strip().lower()

    if mode_norm == "geodesic":
        pool = generate_print_points(
            polyline_points=contour.offset_points,
            polyline_lines=contour.offset_lines,
            field_vertices_world=field_vertices_world,
            field_scalar=heat_norm,
            count=select_count,
            min_spacing=sep_m,
            point_valid_mask=contour.z_valid_mask,
            extra_points=None,
        )
        return CandidateStage(
            points=pool.points,
            scalar_values=pool.scalar_values,
            available_vertices=int(pool.available_vertices),
            z_valid_count=int(np.count_nonzero(contour.z_valid_mask)),
            source_points=pool.source_points,
            segment_start_points=pool.segment_start_points,
        )

    if mode_norm == "z_lift":
        pool = generate_print_points(
            polyline_points=contour.contour_points,
            polyline_lines=contour.contour_lines,
            field_vertices_world=field_vertices_world,
            field_scalar=heat_norm,
            count=select_count,
            min_spacing=sep_m,
            point_valid_mask=None,
            extra_points=None,
            candidate_mode="z_lift",
            lift_height=1e-3 * float(bead_height_mm),
            field_faces=field_faces,
        )
        return CandidateStage(
            points=pool.points,
            scalar_values=pool.scalar_values,
            available_vertices=int(pool.available_vertices),
            z_valid_count=int(pool.available_vertices),
            source_points=pool.source_points,
            segment_start_points=pool.segment_start_points,
        )

    if mode_norm in ("gradient_lift", "gradient_walk"):
        pool = generate_print_points(
            polyline_points=contour.contour_points,
            polyline_lines=contour.contour_lines,
            field_vertices_world=field_vertices_world,
            field_scalar=heat_norm,
            count=select_count,
            min_spacing=sep_m,
            point_valid_mask=None,
            extra_points=None,
            candidate_mode=mode_norm,
            lift_height=1e-3 * float(bead_height_mm),
            field_faces=field_faces,
            walk_distance=1e-3 * float(walk_distance_mm),
            walk_step=1e-3 * float(walk_step_mm),
            walk_max_steps=int(walk_max_steps),
            walk_tangent_sign=float(walk_tangent_sign),
            walk_start_fraction=float(walk_start_fraction),
            clamp_to_cone=bool(clamp_to_cone),
            cone_max_tilt_deg=float(cone_max_tilt_deg),
            agent_phi_scalar=phi,
        )
        return CandidateStage(
            points=pool.points,
            scalar_values=pool.scalar_values,
            available_vertices=int(pool.available_vertices),
            z_valid_count=int(pool.available_vertices),
            source_points=pool.source_points,
            segment_start_points=pool.segment_start_points,
        )

    raise ValueError(
        f"Unknown candidate mode: {mode}. "
        "Use 'geodesic', 'z_lift', 'gradient_lift', or 'gradient_walk'."
    )


def apply_simulated_beads(
    scan_mesh: o3d.geometry.TriangleMesh,
    print_points: np.ndarray,
    *,
    bead_height_mm: float,
    bead_diameter_mm: float,
    bead_shape: str = "cylinder",
) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]:
    """Add bead solids to scan mesh and return updated mesh + solid centers.

    Bead model:
    - cylinder:
      - radius = bead_diameter / 2
      - height = bead_height
      - center = print_point - (0, 0, bead_height / 2)
    - sphere:
      - radius = bead_height / 2
      - center = print_point - (0, 0, bead_height / 2)
    """

    if print_points.shape[0] == 0:
        return copy.deepcopy(scan_mesh), np.zeros((0, 3), dtype=np.float64)

    height = 1e-3 * float(bead_height_mm)
    diameter = 1e-3 * float(bead_diameter_mm)
    if height <= 0.0:
        raise ValueError(f"bead_height_mm must be > 0, got {bead_height_mm}")
    if diameter <= 0.0:
        raise ValueError(f"bead_diameter_mm must be > 0, got {bead_diameter_mm}")

    centers = print_points.copy()
    centers[:, 2] -= 0.5 * height

    updated = copy.deepcopy(scan_mesh)
    shape = str(bead_shape).strip().lower()
    for c in centers:
        if shape == "cylinder":
            solid = o3d.geometry.TriangleMesh.create_cylinder(radius=0.5 * diameter, height=height, resolution=16)
            solid.translate(c)
        elif shape == "sphere":
            solid = o3d.geometry.TriangleMesh.create_sphere(radius=0.5 * height, resolution=12)
            solid.translate(c)
        else:
            raise ValueError(f"Unknown bead_shape: {bead_shape}. Use 'cylinder' or 'sphere'.")
        updated += solid
    return updated, centers
