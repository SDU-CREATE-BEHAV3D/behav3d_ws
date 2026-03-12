#!/usr/bin/env python3
"""Compute phi mask between field and scan meshes, then extract phi=0 contour in 3D."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.geometry import load_triangle_mesh_arrays, load_triangle_mesh_legacy
from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.position_field import (
    default_xy_search_bounds,
    make_axis_samples,
    position_field,
)
from lib_scalar.viz import compute_scene_bounds, make_line_set, make_point_cloud, yellow_to_red_colors


DEFAULT_FIELD_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
DEFAULT_SCAN_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")


def run(
    field_mesh_path: Path,
    scan_mesh_path: Path,
    out_field_ply: Path | None,
    out_contour_ply: Path | None,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
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
    base_epsilon: float,
    clearance: float,
    iso_level: float,
    axis_size: float,
    visualize: bool,
) -> None:
    field_mesh = load_triangle_mesh_arrays(field_mesh_path)
    heat = compute_heat_field(
        vertices=field_mesh.vertices,
        faces=field_mesh.faces,
        seed=seed,
        seed_level=seed_level,
        t_coef=t_coef,
    )

    scale = float(field_scale)
    if scale <= 0.0:
        raise ValueError(f"field_scale must be > 0, got {field_scale}")
    field_vertices_scaled = field_mesh.vertices * scale

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
            base_epsilon=float(base_epsilon),
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
        faces=field_mesh.faces,
        scalar=pose.phi,
        iso=float(iso_level),
    )
    contour_ls = make_line_set(contour_points, contour_lines, color=(0.0, 1.0, 1.0))

    print(f"field_mesh: {field_mesh_path}")
    print(f"scan_mesh: {scan_mesh_path}")
    print(f"field_scale: {scale}")
    print(f"pose_mode: {pose_mode}")
    print(
        "field_offset: "
        f"[{pose.offset_xyz[0]:.6f}, {pose.offset_xyz[1]:.6f}, {pose.offset_xyz[2]:.6f}]"
    )
    print(f"field vertices (used): {field_mesh.vertices.shape[0]}")
    if field_mesh.dropped_vertices > 0:
        print(f"dropped unreferenced field vertices: {field_mesh.dropped_vertices}")
    print(f"field faces: {field_mesh.faces.shape[0]}")
    print(heat.seed_info)
    print(
        "heat stats: "
        f"min={heat.min_value:.6f}, max={heat.max_value:.6f}, mean={heat.mean_value:.6f}"
    )
    if np.any(pose.has_hit):
        phi_min = float(np.nanmin(pose.phi[pose.has_hit]))
        phi_max = float(np.nanmax(pose.phi[pose.has_hit]))
        base_max = float(np.nanmax(pose.base_dz[pose.has_hit]))
    else:
        phi_min = float("nan")
        phi_max = float("nan")
        base_max = float("nan")
    print(
        "phi stats (valid rays): "
        f"count={pose.hit_count}/{pose.phi.shape[0]}, min={phi_min:.6f}, max={phi_max:.6f}"
    )
    print(f"base constraint (base_z - z_scan <= 0): max={base_max:.6f}")
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

    if visualize:
        bb_min, bb_max = compute_scene_bounds(pose.field_vertices_world, scan_vertices)
        bb_diag = float(np.linalg.norm(bb_max - bb_min))
        bb_center = 0.5 * (bb_min + bb_max)

        scan_wire = o3d.geometry.LineSet.create_from_triangle_mesh(scan_mesh_legacy)
        scan_wire.paint_uniform_color((0.5, 0.5, 0.5))
        geometries = [field_pcd, scan_wire]
        if contour_lines.shape[0] > 0:
            geometries.append(contour_ls)

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
        o3d.visualization.draw_geometries(geometries)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute phi mask between field and scan meshes, then extract phi=0 contour in 3D."
    )
    parser.add_argument("--field-mesh", type=Path, default=DEFAULT_FIELD_MESH)
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_SCAN_MESH)
    parser.add_argument(
        "--out-field-ply",
        type=Path,
        default=Path("/home/lab/behav3d_ws/python_scripts/scalar_field/field_masked.ply"),
    )
    parser.add_argument(
        "--out-contour-ply",
        type=Path,
        default=Path("/home/lab/behav3d_ws/python_scripts/scalar_field/field_phi0_contour.ply"),
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--seed-level", type=float, default=None)
    parser.add_argument("--t-coef", type=float, default=1.0)
    parser.add_argument("--field-scale", type=float, default=1.0)
    parser.add_argument("--field-offset-x", type=float, default=0.0)
    parser.add_argument("--field-offset-y", type=float, default=0.0)
    parser.add_argument("--field-offset-z", type=float, default=0.0)
    parser.add_argument("--pose-search", action="store_true")
    parser.add_argument("--search-x-min", type=float, default=None)
    parser.add_argument("--search-x-max", type=float, default=None)
    parser.add_argument("--search-y-min", type=float, default=None)
    parser.add_argument("--search-y-max", type=float, default=None)
    parser.add_argument("--search-margin-x", type=float, default=0.0)
    parser.add_argument("--search-margin-y", type=float, default=0.0)
    parser.add_argument("--search-step-x", type=float, default=0.01)
    parser.add_argument("--search-step-y", type=float, default=0.01)
    parser.add_argument("--search-max-candidates", type=int, default=20000)
    parser.add_argument(
        "--search-allow-partial-hit",
        action="store_true",
        help="Allow poses where some field points do not intersect the scan raycast.",
    )
    parser.add_argument("--search-verbose", action="store_true")
    parser.add_argument(
        "--base-epsilon",
        type=float,
        default=1e-6,
        help="Small downward shift when enforcing base_z <= z_scan during pose search.",
    )
    parser.add_argument("--clearance", type=float, default=0.0003)
    parser.add_argument(
        "--iso-level",
        type=float,
        default=0.0,
        help="Contour level over phi (default 0 => boundary viable/non-viable).",
    )
    parser.add_argument(
        "--axis-size",
        type=float,
        default=0.0,
        help="Axis size for visualization. 0=auto from scene bounds, negative=disable axis.",
    )
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    run(
        field_mesh_path=args.field_mesh,
        scan_mesh_path=args.scan_mesh,
        out_field_ply=args.out_field_ply,
        out_contour_ply=args.out_contour_ply,
        seed=args.seed,
        seed_level=args.seed_level,
        t_coef=args.t_coef,
        field_scale=args.field_scale,
        field_offset=(args.field_offset_x, args.field_offset_y, args.field_offset_z),
        pose_search=args.pose_search,
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
        base_epsilon=args.base_epsilon,
        clearance=args.clearance,
        iso_level=args.iso_level,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
