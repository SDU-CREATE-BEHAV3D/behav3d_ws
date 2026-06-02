#!/usr/bin/env python3
"""Alternative pipeline: select print candidates on the phi contour.

Compared to the default geodesic-offset method:
- no geodesic contour offset is computed,
- candidates are chosen directly on the existing `phi=iso` contour
  using minimum `heat.norm` values interpolated on the triangle surface,
- selected points are post-processed by `generate_print_points` candidate modes.

Positioning modes:
- `manual` (default): uses `--field-offset-x/y/z`
- `search`: computes XY+Z placement from scan/phi constraints
  (equivalent to enabling legacy `--pose-search`)

Example command (manual positioning):
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_phi_polyline_lift.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-subdivide-iter 1 \
  --field-scale 0.001 \
  --field-offset-x -0.17 --field-offset-y -0.92 --field-offset-z -0.05 \
  --clearance 0.00 \
  --print-height-mm 12 --print-count 7 --print-min-spacing-mm 16 \
  --axis-size -1

Example command (auto positioning / search):
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_phi_polyline_lift.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-scale 0.001 \
  --positioning search \
  --base-z-offset 0.01 \
  --search-step-x 0.01 --search-step-y 0.01 \
  --clearance 0.00 \
  --print-height-mm 12 --print-count 7 --print-min-spacing-mm 16 \
  --axis-size -1
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.generate_print_points import generate_print_points
from lib_scalar.geometry import load_triangle_mesh_arrays, load_triangle_mesh_legacy
from lib_scalar.position_field import default_xy_search_bounds, make_axis_samples, position_field
from lib_scalar.print_targets import (
    build_oriented_line_targets,
    write_fixed_z_targets_yaml,
    write_line_targets_yaml,
)
from lib_scalar.viz import (
    compute_scene_bounds,
    make_line_set,
    make_point_cloud,
    make_segment_line_set,
    make_target_orientation_sticks,
    yellow_to_red_colors,
)


DEFAULT_FIELD_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
DEFAULT_SCAN_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def subdivide_field_mesh_loop(
    vertices: np.ndarray,
    faces: np.ndarray,
    iterations: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Optional Loop subdivision to increase field mesh resolution."""
    it = int(iterations)
    if it <= 0:
        return vertices, faces
    if it > 4:
        raise ValueError(f"field_subdivide_iter too large ({it}); use 0..4 to avoid excessive runtime.")

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    mesh_sub = mesh.subdivide_loop(number_of_iterations=it)
    v_sub = np.asarray(mesh_sub.vertices, dtype=np.float64)
    f_sub = np.asarray(mesh_sub.triangles, dtype=np.int32)
    return v_sub, f_sub


def run(
    field_mesh_path: Path,
    scan_mesh_path: Path,
    out_field_ply: Path | None,
    out_contour_ply: Path | None,
    out_print_ply: Path | None,
    out_targets_yaml: Path | None,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
    field_subdivide_iter: int,
    field_scale: float,
    field_offset: tuple[float, float, float],
    pose_search: bool,
    search_x_min: float | None,
    search_x_max: float | None,
    search_y_min: float | None,
    search_y_max: float | None,
    search_margin_x: float,
    search_margin_y: float,
    search_step_x: float,
    search_step_y: float,
    search_max_candidates: int,
    search_allow_partial_hit: bool,
    search_verbose: bool,
    base_z_offset: float,
    clearance: float,
    iso_level: float,
    candidate_mode: str,
    print_height_mm: float,
    walk_distance_mm: float,
    walk_step_mm: float,
    walk_max_steps: int,
    walk_tangent_sign: float,
    walk_start_fraction: float,
    clamp_to_cone: bool,
    cone_max_tilt_deg: float,
    print_count: int,
    print_min_spacing_mm: float,
    target_z_dir: tuple[float, float, float],
    target_position_scale: float,
    axis_size: float,
    visualize: bool,
) -> None:
    field_mesh = load_triangle_mesh_arrays(field_mesh_path)
    base_vertices = field_mesh.vertices
    base_faces = field_mesh.faces
    field_vertices, field_faces = subdivide_field_mesh_loop(
        vertices=base_vertices,
        faces=base_faces,
        iterations=int(field_subdivide_iter),
    )
    heat = compute_heat_field(
        vertices=field_vertices,
        faces=field_faces,
        seed=seed,
        seed_level=seed_level,
        t_coef=t_coef,
    )

    scale = float(field_scale)
    if scale <= 0.0:
        raise ValueError(f"field_scale must be > 0, got {field_scale}")
    field_vertices_scaled = field_vertices * scale

    scan_mesh_legacy = load_triangle_mesh_legacy(scan_mesh_path)
    scan_vertices = np.asarray(scan_mesh_legacy.vertices)
    scene, z_top = make_scan_scene(scan_mesh_legacy)

    search_meta: dict[str, float | int] = {}
    if pose_search:
        x_min, x_max, y_min, y_max = default_xy_search_bounds(
            scan_vertices=scan_vertices,
            field_vertices_scaled=field_vertices_scaled,
            margin_x=float(search_margin_x),
            margin_y=float(search_margin_y),
        )
        if search_x_min is not None:
            x_min = float(search_x_min)
        if search_x_max is not None:
            x_max = float(search_x_max)
        if search_y_min is not None:
            y_min = float(search_y_min)
        if search_y_max is not None:
            y_max = float(search_y_max)

        x_values = make_axis_samples(x_min, x_max, float(search_step_x))
        y_values = make_axis_samples(y_min, y_max, float(search_step_y))
        total_candidates = int(x_values.size * y_values.size)
        if total_candidates > int(search_max_candidates):
            raise ValueError(
                f"Pose search grid too large: {total_candidates} candidates "
                f"(limit={search_max_candidates}). Increase step or tighten bounds."
            )

        pose = position_field(
            scene=scene,
            z_top=z_top,
            field_vertices_scaled=field_vertices_scaled,
            heat_norm=heat.norm,
            x_values=x_values,
            y_values=y_values,
            clearance=float(clearance),
            iso_level=float(iso_level),
            base_z_offset=float(base_z_offset),
            require_full_hit=not bool(search_allow_partial_hit),
            verbose=bool(search_verbose),
        )
        pose_mode = "search_xy"
        search_meta = {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "step_x": float(search_step_x),
            "step_y": float(search_step_y),
            "grid_nx": int(x_values.size),
            "grid_ny": int(y_values.size),
            "tested": int(pose.tested or 0),
            "accepted": int(pose.accepted or 0),
            "viable_heat": float(pose.viable_heat or 0.0),
        }
    else:
        pose = evaluate_fixed_pose(
            scene=scene,
            z_top=z_top,
            field_vertices_scaled=field_vertices_scaled,
            offset_xyz=field_offset,
            clearance=float(clearance),
            iso_level=float(iso_level),
        )
        pose_mode = "manual"

    heat_colors = yellow_to_red_colors(heat.norm)
    masked_colors = heat_colors.copy()
    masked_colors[~pose.viable] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
    field_pcd = make_point_cloud(pose.field_vertices_world, masked_colors)

    contour_points, contour_lines = extract_phi_contour(
        vertices=pose.field_vertices_world,
        faces=field_faces,
        scalar=pose.phi,
        iso=float(iso_level),
    )
    contour_ls = make_line_set(contour_points, contour_lines, color=(0.0, 1.0, 1.0))

    print_height_m = 1e-3 * float(print_height_mm)
    walk_distance_m = 1e-3 * float(walk_distance_mm)
    walk_step_m = 1e-3 * float(walk_step_mm)
    print_min_spacing_m = 1e-3 * float(print_min_spacing_mm)
    mode = str(candidate_mode).strip().lower()
    selected = generate_print_points(
        polyline_points=contour_points,
        polyline_lines=contour_lines,
        field_vertices_world=pose.field_vertices_world,
        field_scalar=heat.norm,
        count=int(print_count),
        min_spacing=float(print_min_spacing_m),
        candidate_mode=mode,
        lift_height=float(print_height_m),
        field_faces=field_faces,
        walk_distance=float(walk_distance_m),
        walk_step=float(walk_step_m),
        walk_max_steps=int(walk_max_steps),
        walk_tangent_sign=float(walk_tangent_sign),
        walk_start_fraction=float(walk_start_fraction),
        clamp_to_cone=bool(clamp_to_cone),
        cone_max_tilt_deg=float(cone_max_tilt_deg),
        agent_phi_scalar=pose.phi,
    )
    candidate_points = selected.points
    source_points = selected.source_points
    if source_points is None:
        source_points = candidate_points.copy()
        if mode == "z_lift":
            source_points[:, 2] -= float(print_height_m)
    segment_start_points = selected.segment_start_points
    if segment_start_points is None or segment_start_points.shape != candidate_points.shape:
        segment_start_points = source_points.copy()

    if candidate_points.shape[0] > 0:
        print_colors = np.tile(np.array([1.0, 0.0, 1.0], dtype=np.float64), (candidate_points.shape[0], 1))
        print_pcd = make_point_cloud(candidate_points, print_colors)
    else:
        print_pcd = None
    if candidate_points.shape[0] > 0 and segment_start_points.shape == candidate_points.shape:
        segment_ls = make_segment_line_set(segment_start_points, candidate_points, color=(1.0, 0.55, 0.0))
        segment_start_colors = np.tile(
            np.array([1.0, 0.55, 0.0], dtype=np.float64),
            (segment_start_points.shape[0], 1),
        )
        segment_start_pcd = make_point_cloud(segment_start_points, segment_start_colors)
    else:
        segment_ls = None
        segment_start_pcd = None
    line_targets = None
    if mode == "gradient_walk":
        line_targets = build_oriented_line_targets(
            start_points=segment_start_points,
            end_points=candidate_points,
            field_vertices_world=pose.field_vertices_world,
            field_faces=field_faces,
            field_scalar=heat.norm,
            tangent_sign=float(walk_tangent_sign),
            clamp_to_cone=bool(clamp_to_cone),
            cone_max_tilt_deg=float(cone_max_tilt_deg),
        )

    print(f"field_mesh: {field_mesh_path}")
    print(f"scan_mesh: {scan_mesh_path}")
    print(f"field_scale: {scale}")
    print(f"pose_mode: {pose_mode}")
    print(
        "field_offset: "
        f"[{pose.offset_xyz[0]:.6f}, {pose.offset_xyz[1]:.6f}, {pose.offset_xyz[2]:.6f}]"
    )
    print(f"field vertices (used): {field_vertices.shape[0]}")
    if field_mesh.dropped_vertices > 0:
        print(f"dropped unreferenced field vertices: {field_mesh.dropped_vertices}")
    print(f"field faces: {field_faces.shape[0]}")
    if int(field_subdivide_iter) > 0:
        print(
            "field subdivision: "
            f"iter={int(field_subdivide_iter)} "
            f"base_v={base_vertices.shape[0]} base_f={base_faces.shape[0]}"
        )
    print(heat.seed_info)
    print(
        "heat stats: "
        f"min={heat.min_value:.6f}, max={heat.max_value:.6f}, mean={heat.mean_value:.6f}"
    )
    if np.any(pose.has_hit):
        phi_min = float(np.nanmin(pose.phi[pose.has_hit]))
        phi_max = float(np.nanmax(pose.phi[pose.has_hit]))
        contact_min = float(np.nanmin(pose.base_dz[pose.has_hit]))
        contact_max = float(np.nanmax(pose.base_dz[pose.has_hit]))
        base_local_z = float(np.min(field_vertices_scaled[:, 2]))
        bbox_diag = float(np.linalg.norm(np.max(field_vertices_scaled, axis=0) - np.min(field_vertices_scaled, axis=0)))
        base_tol = max(1e-9, 1e-6 * bbox_diag)
        base_mask = field_vertices_scaled[:, 2] <= (base_local_z + base_tol)
        base_hit = pose.has_hit & base_mask
        if np.any(base_hit):
            base_contact_min = float(np.nanmin(pose.base_dz[base_hit]))
            base_contact_max = float(np.nanmax(pose.base_dz[base_hit]))
        else:
            base_contact_min = float("nan")
            base_contact_max = float("nan")
    else:
        phi_min = float("nan")
        phi_max = float("nan")
        contact_min = float("nan")
        contact_max = float("nan")
        base_contact_min = float("nan")
        base_contact_max = float("nan")
    print(
        "phi stats (valid rays): "
        f"count={pose.hit_count}/{pose.phi.shape[0]}, min={phi_min:.6f}, max={phi_max:.6f}"
    )
    print(
        "contact dz (field_z - z_scan): "
        f"min={contact_min:.6f}, max={contact_max:.6f}"
    )
    print(
        "base contact dz (base_field_z - z_scan): "
        f"min={base_contact_min:.6f}, max={base_contact_max:.6f}"
    )
    print(f"viable count (phi>{iso_level}): {pose.viable_count}")
    if search_meta:
        print(
            "pose_search grid: "
            f"x=[{search_meta['x_min']:.6f},{search_meta['x_max']:.6f}] step={search_meta['step_x']:.6f} "
            f"(nx={search_meta['grid_nx']}), "
            f"y=[{search_meta['y_min']:.6f},{search_meta['y_max']:.6f}] step={search_meta['step_y']:.6f} "
            f"(ny={search_meta['grid_ny']})"
        )
        print(
            "pose_search stats: "
            f"tested={search_meta['tested']} accepted={search_meta['accepted']} "
            f"viable_heat={search_meta['viable_heat']:.6f}"
        )
    print(f"contour segments: {contour_lines.shape[0]}")
    print(
        "phi-polyline candidate config: "
        f"mode={mode} "
        f"print_height_mm={float(print_height_mm):.3f} "
        f"print_height_m={print_height_m:.6f} "
        f"walk_distance_mm={float(walk_distance_mm):.3f} "
        f"walk_step_mm={float(walk_step_mm):.3f} "
        f"walk_max_steps={int(walk_max_steps)} "
        f"walk_tangent_sign={float(walk_tangent_sign):+.1f} "
        f"walk_start_fraction={float(walk_start_fraction):.3f} "
        f"clamp_to_cone={bool(clamp_to_cone)} "
        f"cone_max_tilt_deg={float(cone_max_tilt_deg):.3f} "
        f"count={int(print_count)} "
        f"min_spacing_mm={float(print_min_spacing_mm):.3f} "
        f"min_spacing_m={print_min_spacing_m:.6f}"
    )
    print(
        "candidate point selection: "
        f"selected={candidate_points.shape[0]} "
        f"available_polyline_vertices={selected.available_vertices}"
    )
    if candidate_points.shape[0] > 0:
        print(
            "selected heat_norm stats: "
            f"min={float(np.min(selected.scalar_values)):.6f} "
            f"max={float(np.max(selected.scalar_values)):.6f}"
        )
        for i in range(candidate_points.shape[0]):
            p0 = source_points[i]
            p1 = segment_start_points[i]
            pf = candidate_points[i]
            v = float(selected.scalar_values[i])
            print(
                f"print_point[{i}]: "
                f"src=({float(p0[0]):.6f},{float(p0[1]):.6f},{float(p0[2]):.6f}) "
                f"start=({float(p1[0]):.6f},{float(p1[1]):.6f},{float(p1[2]):.6f}) "
                f"final=({float(pf[0]):.6f},{float(pf[1]):.6f},{float(pf[2]):.6f}) "
                f"heat_norm={v:.6f}"
            )

    if out_field_ply is not None:
        out_field_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_point_cloud(str(out_field_ply), field_pcd)
        if not ok:
            raise RuntimeError(f"Failed to write field masked point cloud: {out_field_ply}")
        print(f"saved field masked point cloud: {out_field_ply}")

    if out_contour_ply is not None and contour_lines.shape[0] > 0:
        out_contour_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_line_set(str(out_contour_ply), contour_ls)
        if not ok:
            raise RuntimeError(f"Failed to write contour line set: {out_contour_ply}")
        print(f"saved contour lines: {out_contour_ply}")

    if out_print_ply is not None and print_pcd is not None:
        out_print_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_point_cloud(str(out_print_ply), print_pcd)
        if not ok:
            raise RuntimeError(f"Failed to write candidate print points point cloud: {out_print_ply}")
        print(f"saved candidate print points: {out_print_ply}")

    if out_targets_yaml is not None:
        if line_targets is not None:
            write_line_targets_yaml(
                out_yaml=out_targets_yaml,
                targets=line_targets,
                position_scale=float(target_position_scale),
            )
            yaml_note = f"segments={line_targets.count}"
        else:
            write_fixed_z_targets_yaml(
                out_yaml=out_targets_yaml,
                points_world=candidate_points,
                z_dir=target_z_dir,
                position_scale=float(target_position_scale),
            )
            yaml_note = f"points={candidate_points.shape[0]}"
        print(
            "saved targets yaml: "
            f"{out_targets_yaml} "
            f"({yaml_note}, scale={float(target_position_scale):.2f})"
        )

    if visualize:
        bb_min, bb_max = compute_scene_bounds(pose.field_vertices_world, scan_vertices)
        bb_diag = float(np.linalg.norm(bb_max - bb_min))
        bb_center = 0.5 * (bb_min + bb_max)

        scan_wire = o3d.geometry.LineSet.create_from_triangle_mesh(scan_mesh_legacy)
        scan_wire.paint_uniform_color((0.5, 0.5, 0.5))
        geometries = [field_pcd, scan_wire]
        if contour_lines.shape[0] > 0:
            geometries.append(contour_ls)
        if print_pcd is not None:
            geometries.append(print_pcd)
        if segment_ls is not None:
            geometries.append(segment_ls)
        if segment_start_pcd is not None:
            geometries.append(segment_start_pcd)
        if line_targets is not None and line_targets.count > 0:
            target_points, target_z_dirs = line_targets.flattened_points_and_z_dirs()
            geometries.append(make_target_orientation_sticks(target_points, target_z_dirs))

        axis_size_val = float(axis_size)
        if axis_size_val == 0.0:
            axis_size_val = max(1e-4, 0.15 * bb_diag)
        if axis_size_val > 0.0:
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_size_val)
            axes.translate(bb_center)
            geometries.append(axes)

        print(f"visualize bounds min={bb_min.tolist()} max={bb_max.tolist()} diag={bb_diag:.6f}")
        if axis_size_val > 0.0:
            print(f"axis size used: {axis_size_val:.6f} (axis centered at scene bbox center)")
        else:
            print("axis disabled")
        if contour_lines.shape[0] > 0:
            print("phi contour shown in cyan")
        if print_pcd is not None:
            print("candidate print points shown in magenta")
        if segment_ls is not None:
            print("print target segments shown in orange")
        if line_targets is not None and line_targets.count > 0:
            print("target orientations shown as 8 mm green sticks")
        o3d.visualization.draw_geometries(geometries)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Alternative method: select print candidates on phi contour."
    )
    parser.add_argument("--field-mesh", type=Path, default=DEFAULT_FIELD_MESH)
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_SCAN_MESH)
    parser.add_argument(
        "--out-field-ply",
        type=Path,
        default=OUTPUT_DIR / "field_masked.ply",
    )
    parser.add_argument(
        "--out-contour-ply",
        type=Path,
        default=OUTPUT_DIR / "field_phi0_contour.ply",
    )
    parser.add_argument(
        "--out-print-ply",
        type=Path,
        default=OUTPUT_DIR / "field_print_points_phi_lift.ply",
    )
    parser.add_argument(
        "--out-targets-yaml",
        type=Path,
        default=Path("/home/lab/behav3d_ws/yaml/scalar_field_targets_phi_lift.yaml"),
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--seed-level", type=float, default=None)
    parser.add_argument("--t-coef", type=float, default=1.0)
    parser.add_argument(
        "--field-subdivide-iter",
        type=int,
        default=0,
        help="Loop-subdivide field mesh before heat/phi/selection (0 disables).",
    )
    parser.add_argument("--field-scale", type=float, default=1.0)
    parser.add_argument("--field-offset-x", type=float, default=0.0)
    parser.add_argument("--field-offset-y", type=float, default=0.0)
    parser.add_argument("--field-offset-z", type=float, default=0.0)
    parser.add_argument(
        "--positioning",
        type=str,
        choices=("manual", "search"),
        default="manual",
        help="Field placement mode: manual offsets or automatic search.",
    )
    parser.add_argument("--pose-search", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--search-x-min", type=float, default=None)
    parser.add_argument("--search-x-max", type=float, default=None)
    parser.add_argument("--search-y-min", type=float, default=None)
    parser.add_argument("--search-y-max", type=float, default=None)
    parser.add_argument("--search-margin-x", type=float, default=0.0)
    parser.add_argument("--search-margin-y", type=float, default=0.0)
    parser.add_argument("--search-step-x", type=float, default=0.01)
    parser.add_argument("--search-step-y", type=float, default=0.01)
    parser.add_argument("--search-max-candidates", type=int, default=20000)
    parser.add_argument("--search-allow-partial-hit", action="store_true")
    parser.add_argument("--search-verbose", action="store_true")
    parser.add_argument("--base-z-offset", type=float, default=1e-6)
    parser.add_argument("--clearance", type=float, default=0.0003)
    parser.add_argument("--iso-level", type=float, default=0.0)
    parser.add_argument(
        "--candidate-mode",
        type=str,
        choices=("z_lift", "gradient_lift", "gradient_walk"),
        default="z_lift",
        help="Post-process mode passed to generate_print_points.",
    )
    parser.add_argument(
        "--print-height-mm",
        type=float,
        default=12.0,
        help="Distance for z_lift/gradient_lift after selecting points on existing phi contour.",
    )
    parser.add_argument("--walk-distance-mm", type=float, default=12.0)
    parser.add_argument("--walk-step-mm", type=float, default=1.0)
    parser.add_argument("--walk-max-steps", type=int, default=32)
    parser.add_argument("--walk-tangent-sign", type=float, default=1.0)
    parser.add_argument("--walk-start-fraction", type=float, default=0.25)
    parser.add_argument("--clamp-to-cone", action="store_true")
    parser.add_argument("--cone-max-tilt-deg", type=float, default=45.0)
    parser.add_argument("--print-count", type=int, default=7)
    parser.add_argument("--print-min-spacing-mm", type=float, default=16.0)
    parser.add_argument("--target-zx", type=float, default=0.03)
    parser.add_argument("--target-zy", type=float, default=-0.01)
    parser.add_argument("--target-zz", type=float, default=1.00)
    parser.add_argument("--target-position-scale", type=float, default=1000.0)
    parser.add_argument("--axis-size", type=float, default=0.0)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    use_pose_search = bool(args.pose_search) or (str(args.positioning).strip().lower() == "search")

    run(
        field_mesh_path=args.field_mesh,
        scan_mesh_path=args.scan_mesh,
        out_field_ply=args.out_field_ply,
        out_contour_ply=args.out_contour_ply,
        out_print_ply=args.out_print_ply,
        out_targets_yaml=args.out_targets_yaml,
        seed=args.seed,
        seed_level=args.seed_level,
        t_coef=args.t_coef,
        field_subdivide_iter=args.field_subdivide_iter,
        field_scale=args.field_scale,
        field_offset=(args.field_offset_x, args.field_offset_y, args.field_offset_z),
        pose_search=use_pose_search,
        search_x_min=args.search_x_min,
        search_x_max=args.search_x_max,
        search_y_min=args.search_y_min,
        search_y_max=args.search_y_max,
        search_margin_x=args.search_margin_x,
        search_margin_y=args.search_margin_y,
        search_step_x=args.search_step_x,
        search_step_y=args.search_step_y,
        search_max_candidates=args.search_max_candidates,
        search_allow_partial_hit=args.search_allow_partial_hit,
        search_verbose=args.search_verbose,
        base_z_offset=args.base_z_offset,
        clearance=args.clearance,
        iso_level=args.iso_level,
        candidate_mode=args.candidate_mode,
        print_height_mm=args.print_height_mm,
        walk_distance_mm=args.walk_distance_mm,
        walk_step_mm=args.walk_step_mm,
        walk_max_steps=args.walk_max_steps,
        walk_tangent_sign=args.walk_tangent_sign,
        walk_start_fraction=args.walk_start_fraction,
        clamp_to_cone=args.clamp_to_cone,
        cone_max_tilt_deg=args.cone_max_tilt_deg,
        print_count=args.print_count,
        print_min_spacing_mm=args.print_min_spacing_mm,
        target_z_dir=(args.target_zx, args.target_zy, args.target_zz),
        target_position_scale=args.target_position_scale,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
