#!/usr/bin/env python3
"""Compute phi mask, extract contours, and select print points on offset contour."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.extract_offset_phi_contour import extract_offset_phi_contour
from lib_scalar.generate_print_points import generate_print_points
from lib_scalar.geometry import load_triangle_mesh_arrays, load_triangle_mesh_legacy
from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.compute_phi_mask import (
    evaluate_fixed_pose,
    make_scan_scene,
    query_scan_z_with_vertical_rays,
)
from lib_scalar.position_field import (
    default_xy_search_bounds,
    make_axis_samples,
    position_field,
)
from lib_scalar.viz import compute_scene_bounds, make_line_set, make_point_cloud, yellow_to_red_colors


DEFAULT_FIELD_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
DEFAULT_SCAN_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")


def deduplicate_polyline(
    points: np.ndarray,
    lines: np.ndarray,
    merge_tol: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Merge near-equal polyline points and remap line indices."""
    if points.size == 0 or lines.size == 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 2), dtype=np.int32)

    tol = max(float(merge_tol), 1e-12)
    keys = np.round(points / tol).astype(np.int64)
    key_to_uid: dict[tuple[int, int, int], int] = {}
    uid_points: list[np.ndarray] = []
    old_to_uid = np.full(points.shape[0], -1, dtype=np.int32)

    for i in range(points.shape[0]):
        k = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
        uid = key_to_uid.get(k)
        if uid is None:
            uid = len(uid_points)
            key_to_uid[k] = uid
            uid_points.append(points[i])
        old_to_uid[i] = int(uid)

    edge_set: set[tuple[int, int]] = set()
    for e in lines:
        a = int(old_to_uid[int(e[0])])
        b = int(old_to_uid[int(e[1])])
        if a == b:
            continue
        u, v = (a, b) if a < b else (b, a)
        edge_set.add((u, v))

    if not uid_points or not edge_set:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 2), dtype=np.int32)
    return np.vstack(uid_points), np.asarray(sorted(edge_set), dtype=np.int32)


def polyline_endpoints(
    points: np.ndarray,
    lines: np.ndarray,
    merge_tol: float = 1e-6,
) -> np.ndarray:
    """Return endpoints (degree==1) of a polyline graph."""
    p_u, l_u = deduplicate_polyline(points=points, lines=lines, merge_tol=merge_tol)
    if p_u.shape[0] == 0 or l_u.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    deg = np.zeros(p_u.shape[0], dtype=np.int32)
    for e in l_u:
        deg[int(e[0])] += 1
        deg[int(e[1])] += 1
    idx = np.flatnonzero(deg == 1)
    if idx.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return p_u[idx]


def closest_points_on_mesh(
    query_points: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
) -> np.ndarray:
    """Project query points to closest points on triangle surface."""
    if query_points.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    v = o3d.core.Tensor(mesh_vertices.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    f = o3d.core.Tensor(mesh_faces.astype(np.int32), dtype=o3d.core.Dtype.Int32)
    tmesh = o3d.t.geometry.TriangleMesh(v, f)
    scene_local = o3d.t.geometry.RaycastingScene()
    scene_local.add_triangles(tmesh)
    q = o3d.core.Tensor(query_points.astype(np.float32), dtype=o3d.core.Dtype.Float32)
    out = scene_local.compute_closest_points(q)
    return out["points"].numpy().astype(np.float64)


def write_targets_yaml(
    out_yaml: Path,
    points_world: np.ndarray,
    z_dir: tuple[float, float, float],
    position_scale: float,
) -> None:
    """Write targets YAML with legacy plane-string format."""
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    zx, zy, zz = float(z_dir[0]), float(z_dir[1]), float(z_dir[2])
    scale = float(position_scale)

    lines: list[str] = ["targets:"]
    for i in range(points_world.shape[0]):
        p = points_world[i]
        ox = scale * float(p[0])
        oy = scale * float(p[1])
        oz = scale * float(p[2])
        plane = f'O({ox:.2f},{oy:.2f},{oz:.2f}) Z({zx:.2f},{zy:.2f},{zz:.2f})'
        lines.append(f"  - index: {i}")
        lines.append(f'    plane: "{plane}"')

    out_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(
    field_mesh_path: Path,
    scan_mesh_path: Path,
    out_field_ply: Path | None,
    out_contour_ply: Path | None,
    out_offset_ply: Path | None,
    out_print_ply: Path | None,
    out_targets_yaml: Path | None,
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
    offset_distance_mm: float,
    offset_t_coef: float,
    offset_toward_unprinted: bool,
    print_count: int,
    print_min_spacing_mm: float,
    target_z_dir: tuple[float, float, float],
    target_position_scale: float,
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

    offset_distance_m = 1e-3 * float(offset_distance_mm)
    offset_points, offset_lines, _, offset_seed_vertices = extract_offset_phi_contour(
        vertices=pose.field_vertices_world,
        faces=field_mesh.faces,
        phi=pose.phi,
        iso_level=float(iso_level),
        offset_distance=offset_distance_m,
        toward_unprinted=bool(offset_toward_unprinted),
        t_coef=float(offset_t_coef),
    )
    offset_ls = make_line_set(offset_points, offset_lines, color=(1.0, 0.0, 1.0))

    if offset_points.shape[0] > 0:
        offset_z_scan, offset_has_hit = query_scan_z_with_vertical_rays(scene, offset_points, z_top=z_top)
        offset_abs_dz = np.full(offset_points.shape[0], np.inf, dtype=np.float64)
        offset_abs_dz[offset_has_hit] = np.abs(offset_points[offset_has_hit, 2] - offset_z_scan[offset_has_hit])
        offset_valid_by_z = offset_has_hit & (offset_abs_dz <= offset_distance_m)
    else:
        offset_has_hit = np.zeros((0,), dtype=bool)
        offset_abs_dz = np.zeros((0,), dtype=np.float64)
        offset_valid_by_z = np.zeros((0,), dtype=bool)

    phi0_endpoints = polyline_endpoints(contour_points, contour_lines, merge_tol=1e-6)
    if phi0_endpoints.shape[0] > 0:
        phi0_proj = phi0_endpoints.copy()
        phi0_proj[:, 2] += offset_distance_m
        phi0_bridge = closest_points_on_mesh(
            query_points=phi0_proj,
            mesh_vertices=pose.field_vertices_world,
            mesh_faces=field_mesh.faces,
        )

        bridge_z_scan, bridge_has_hit = query_scan_z_with_vertical_rays(scene, phi0_bridge, z_top=z_top)
        bridge_abs_dz = np.full(phi0_bridge.shape[0], np.inf, dtype=np.float64)
        bridge_abs_dz[bridge_has_hit] = np.abs(phi0_bridge[bridge_has_hit, 2] - bridge_z_scan[bridge_has_hit])
        bridge_valid = bridge_has_hit & (bridge_abs_dz <= offset_distance_m)
        phi0_bridge_valid = phi0_bridge[bridge_valid]
    else:
        bridge_abs_dz = np.zeros((0,), dtype=np.float64)
        bridge_valid = np.zeros((0,), dtype=bool)
        phi0_bridge_valid = np.zeros((0, 3), dtype=np.float64)

    print_min_spacing_m = 1e-3 * float(print_min_spacing_mm)
    print_points = generate_print_points(
        polyline_points=offset_points,
        polyline_lines=offset_lines,
        field_vertices_world=pose.field_vertices_world,
        field_scalar=heat.norm,
        count=int(print_count),
        min_spacing=float(print_min_spacing_m),
        point_valid_mask=offset_valid_by_z,
        extra_points=phi0_bridge_valid,
    )
    if print_points.points.shape[0] > 0:
        print_colors = np.tile(np.array([0.0, 1.0, 0.0], dtype=np.float64), (print_points.points.shape[0], 1))
        print_pcd = make_point_cloud(print_points.points, print_colors)
    else:
        print_pcd = None

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

    print(
        "offset contour config: "
        f"distance_mm={offset_distance_mm:.3f} "
        f"distance_m={offset_distance_m:.6f} "
        f"side={'unprinted(phi>iso)' if offset_toward_unprinted else 'printed(phi<=iso)'} "
        f"t_coef={float(offset_t_coef):.6f}"
    )
    print(f"offset contour seed vertices: {offset_seed_vertices.shape[0]}")
    print(f"offset contour segments: {offset_lines.shape[0]}")
    print(
        "offset contour z-valid: "
        f"{int(np.count_nonzero(offset_valid_by_z))}/{offset_valid_by_z.shape[0]} "
        f"(abs(z_point-z_scan) <= {offset_distance_m:.6f} m)"
    )
    if offset_abs_dz.shape[0] > 0 and np.any(np.isfinite(offset_abs_dz)):
        finite_dz = offset_abs_dz[np.isfinite(offset_abs_dz)]
        print(
            "offset contour z-distance stats: "
            f"min={float(np.min(finite_dz)):.6f} max={float(np.max(finite_dz)):.6f}"
        )
    print(
        "phi0 endpoint bridges: "
        f"endpoints={phi0_endpoints.shape[0]} "
        f"valid={int(np.count_nonzero(bridge_valid))}/{bridge_valid.shape[0]} "
        f"(project +Z by {offset_distance_m:.6f} m, then snap to field)"
    )
    if bridge_abs_dz.shape[0] > 0 and np.any(np.isfinite(bridge_abs_dz)):
        finite_bridge_dz = bridge_abs_dz[np.isfinite(bridge_abs_dz)]
        print(
            "phi0 bridge z-distance stats: "
            f"min={float(np.min(finite_bridge_dz)):.6f} max={float(np.max(finite_bridge_dz)):.6f}"
        )
    print(
        "print point config: "
        f"count={int(print_count)} min_spacing_mm={float(print_min_spacing_mm):.3f} "
        f"min_spacing_m={print_min_spacing_m:.6f}"
    )
    print(
        "print point selection: "
        f"selected={print_points.points.shape[0]} "
        f"available_polyline_vertices={print_points.available_vertices} "
        f"endpoint_bridges_added={print_points.augmented_vertices}"
    )
    if print_points.points.shape[0] > 0:
        print(
            "print point scalar stats: "
            f"min={float(np.min(print_points.scalar_values)):.6f} "
            f"max={float(np.max(print_points.scalar_values)):.6f}"
        )
        for i in range(print_points.points.shape[0]):
            p = print_points.points[i]
            s = float(print_points.scalar_values[i])
            print(
                f"print_point[{i}]: "
                f"x={float(p[0]):.6f} y={float(p[1]):.6f} z={float(p[2]):.6f} scalar={s:.6f}"
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

    if out_offset_ply is not None and offset_lines.shape[0] > 0:
        out_offset_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_line_set(str(out_offset_ply), offset_ls)
        if not ok:
            raise RuntimeError(f"Failed to write offset contour line set: {out_offset_ply}")
        print(f"saved offset contour lines: {out_offset_ply}")

    if out_print_ply is not None and print_pcd is not None:
        out_print_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_point_cloud(str(out_print_ply), print_pcd)
        if not ok:
            raise RuntimeError(f"Failed to write print points point cloud: {out_print_ply}")
        print(f"saved print points: {out_print_ply}")

    if out_targets_yaml is not None:
        write_targets_yaml(
            out_yaml=out_targets_yaml,
            points_world=print_points.points,
            z_dir=target_z_dir,
            position_scale=float(target_position_scale),
        )
        print(
            "saved targets yaml: "
            f"{out_targets_yaml} "
            f"(points={print_points.points.shape[0]}, scale={float(target_position_scale):.2f})"
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
        if offset_lines.shape[0] > 0:
            geometries.append(offset_ls)
        if print_pcd is not None:
            geometries.append(print_pcd)

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
        if offset_lines.shape[0] > 0:
            print("offset contour shown in magenta")
        if print_pcd is not None:
            print("print points shown in green")
        o3d.visualization.draw_geometries(geometries)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute phi mask, extract contours, and select print points on offset contour."
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
    parser.add_argument(
        "--out-offset-ply",
        type=Path,
        default=Path("/home/lab/behav3d_ws/python_scripts/scalar_field/field_phi_offset_12mm.ply"),
        help="Output line set (.ply) for geodesic offset contour.",
    )
    parser.add_argument(
        "--out-print-ply",
        type=Path,
        default=Path("/home/lab/behav3d_ws/python_scripts/scalar_field/field_print_points.ply"),
        help="Output point cloud (.ply) for selected print points.",
    )
    parser.add_argument(
        "--out-targets-yaml",
        type=Path,
        default=Path("/home/lab/behav3d_ws/yaml/scalar_field_targets.yaml"),
        help="Output YAML with legacy targets/plane format.",
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
        "--offset-distance-mm",
        type=float,
        default=12.0,
        help="Geodesic offset distance from phi contour, in millimeters (default 12 mm).",
    )
    parser.add_argument(
        "--offset-t-coef",
        type=float,
        default=1.0,
        help="Heat-method t_coef used for geodesic offset solve.",
    )
    parser.add_argument(
        "--offset-toward-printed",
        action="store_true",
        help="Offset toward printed side (phi<=iso). Default is toward unprinted side (phi>iso).",
    )
    parser.add_argument(
        "--print-count",
        type=int,
        default=7,
        help="Number of print points to select from offset polyline (default 7).",
    )
    parser.add_argument(
        "--print-min-spacing-mm",
        type=float,
        default=16.0,
        help="Minimum spacing between selected print points along polyline (default 16 mm).",
    )
    parser.add_argument(
        "--target-zx",
        type=float,
        default=0.03,
        help="Fixed plane Z-direction x component for YAML export.",
    )
    parser.add_argument(
        "--target-zy",
        type=float,
        default=-0.01,
        help="Fixed plane Z-direction y component for YAML export.",
    )
    parser.add_argument(
        "--target-zz",
        type=float,
        default=1.00,
        help="Fixed plane Z-direction z component for YAML export.",
    )
    parser.add_argument(
        "--target-position-scale",
        type=float,
        default=1000.0,
        help="Scale factor applied to XYZ before YAML export (1000 => meters to millimeters).",
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
        out_offset_ply=args.out_offset_ply,
        out_print_ply=args.out_print_ply,
        out_targets_yaml=args.out_targets_yaml,
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
        offset_distance_mm=args.offset_distance_mm,
        offset_t_coef=args.offset_t_coef,
        offset_toward_unprinted=not args.offset_toward_printed,
        print_count=args.print_count,
        print_min_spacing_mm=args.print_min_spacing_mm,
        target_z_dir=(args.target_zx, args.target_zy, args.target_zz),
        target_position_scale=args.target_position_scale,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
