#!/usr/bin/env python3
"""Replay scalar-field candidate generation on a real captured cycle.

Default scenario:
- Scan mesh: cycle_0026 TSDF STL
- Field state: cycle_0000 initialized field state

Sample run:
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/test_real_cycle_replay.py \
  --no-vis
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.extract_phi_contour import (
    compute_geodesic_from_phi_contour,
    extract_offset_phi_contour,
    extract_phi_contour,
)
from lib_scalar.generate_print_points import generate_print_points
from lib_scalar.geometry import (
    clamp_vectors_to_cone,
    load_triangle_mesh_legacy,
    project_points_to_surface,
    sample_vertex_scalar_on_surface,
    sample_tangent_axes_on_surface_from_scalar,
)
from lib_scalar.viz import make_line_set, make_point_cloud, yellow_to_red_colors


DEFAULT_SCAN_MESH = Path(
    "/home/lab/behav3d_ws/captures/260318_125644/field_loop/cycle_0074/scan/reconstruct/tsdf_surface_mesh.stl"
)
DEFAULT_FIELD_STATE = Path(
    "/home/lab/behav3d_ws/captures/260318_125644/field_loop/cycle_0000/field_init/field_state_init.npz"
)
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "output" / "real_cycle_replay"


def _load_field_state(
    state_path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, tuple[float, float, float], float, float]:
    state = np.load(str(state_path), allow_pickle=False)
    if "field_faces" not in state:
        raise KeyError(f"Missing 'field_faces' in {state_path}")
    if "heat_norm" not in state:
        raise KeyError(f"Missing 'heat_norm' in {state_path}")
    if "offset_xyz" not in state:
        raise KeyError(f"Missing 'offset_xyz' in {state_path}")

    field_faces = np.asarray(state["field_faces"], dtype=np.int32)
    heat_norm = np.asarray(state["heat_norm"], dtype=np.float64).reshape(-1)
    if "heat_dist" in state:
        axis_scalar = np.asarray(state["heat_dist"], dtype=np.float64).reshape(-1)
    else:
        axis_scalar = heat_norm

    if "field_vertices_scaled" in state:
        field_vertices_scaled = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
    elif "field_vertices" in state:
        base = np.asarray(state["field_vertices"], dtype=np.float64)
        if "field_scale" in state:
            scale = float(np.asarray(state["field_scale"], dtype=np.float64).reshape(-1)[0])
        else:
            scale = 1.0
        field_vertices_scaled = base * scale
    else:
        raise KeyError(f"Missing 'field_vertices_scaled'/'field_vertices' in {state_path}")

    offset_xyz = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
    if offset_xyz.size < 3:
        raise ValueError(f"Invalid offset_xyz shape in {state_path}: {offset_xyz.shape}")

    if "clearance" in state:
        clearance = float(np.asarray(state["clearance"], dtype=np.float64).reshape(-1)[0])
    else:
        clearance = 0.0
    if "t_coef" in state:
        t_coef_state = float(np.asarray(state["t_coef"], dtype=np.float64).reshape(-1)[0])
    else:
        t_coef_state = 1.0

    if field_vertices_scaled.ndim != 2 or field_vertices_scaled.shape[1] != 3:
        raise ValueError(f"Invalid field_vertices_scaled shape: {field_vertices_scaled.shape}")
    if field_faces.ndim != 2 or field_faces.shape[1] != 3:
        raise ValueError(f"Invalid field_faces shape: {field_faces.shape}")
    if heat_norm.shape[0] != field_vertices_scaled.shape[0]:
        raise ValueError("heat_norm length does not match field vertices")
    if axis_scalar.shape[0] != field_vertices_scaled.shape[0]:
        raise ValueError("axis scalar length does not match field vertices")

    offset = (float(offset_xyz[0]), float(offset_xyz[1]), float(offset_xyz[2]))
    return field_vertices_scaled, field_faces, heat_norm, axis_scalar, offset, clearance, t_coef_state


def _normalize_scalar(values: np.ndarray) -> np.ndarray:
    finite = np.isfinite(values)
    if not np.any(finite):
        raise ValueError("Scalar field has no finite values.")
    v_min = float(np.min(values[finite]))
    v_max = float(np.max(values[finite]))
    denom = max(v_max - v_min, 1e-12)
    out = np.ones_like(values, dtype=np.float64)
    out[finite] = (values[finite] - v_min) / denom
    return out


def _align_tangent_frame_sign(
    t_dir: np.ndarray,
    b_dir: np.ndarray,
    order_idx: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Flip tangent/binormal signs for local consistency along polyline order."""
    if t_dir.shape[0] <= 1:
        return t_dir, b_dir
    t = np.asarray(t_dir, dtype=np.float64).copy()
    b = np.asarray(b_dir, dtype=np.float64).copy()
    order = np.asarray(order_idx, dtype=np.int64).reshape(-1)
    if order.shape[0] != t.shape[0]:
        return t, b

    seq = np.argsort(order)
    prev = int(seq[0])
    for k in range(1, seq.shape[0]):
        cur = int(seq[k])
        if float(np.dot(t[cur], t[prev])) < 0.0:
            t[cur] *= -1.0
            b[cur] *= -1.0
        prev = cur
    return t, b


def _build_axis_lines(
    points: np.ndarray,
    directions: np.ndarray,
    *,
    scale: float,
    color: tuple[float, float, float],
) -> o3d.geometry.LineSet:
    if points.shape[0] == 0:
        return make_line_set(
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 2), dtype=np.int32),
            color=color,
        )
    n = np.linalg.norm(directions, axis=1)
    valid = n > 1e-12
    p0 = points[valid]
    d = directions[valid] / n[valid, None]
    p1 = p0 + float(scale) * d
    if p0.shape[0] == 0:
        return make_line_set(
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 2), dtype=np.int32),
            color=color,
        )
    pts = np.vstack([p0, p1])
    m = p0.shape[0]
    lines = np.column_stack([np.arange(m, dtype=np.int32), np.arange(m, dtype=np.int32) + m])
    return make_line_set(pts, lines, color=color)


def run(
    scan_mesh_path: Path,
    field_state_path: Path,
    out_dir: Path,
    print_count: int,
    print_min_spacing_mm: float,
    print_height_mm: float,
    iso_level: float,
    clearance_override: float | None,
    recompute_field_from_contour: bool,
    recompute_seed_iso: float,
    recompute_t_coef: float | None,
    prelift_geodesic_offset_mm: float,
    candidate_mode: str,
    walk_distance_mm: float,
    walk_step_mm: float,
    walk_max_steps: int,
    walk_tangent_sign: float,
    clamp_to_cone: bool,
    cone_max_tilt_deg: float,
    tangent_sign: float,
    frame_scale_mm: float,
    axis_size: float,
    visualize: bool,
) -> None:
    if not scan_mesh_path.is_file():
        raise FileNotFoundError(f"scan mesh not found: {scan_mesh_path}")
    if not field_state_path.is_file():
        raise FileNotFoundError(f"field state not found: {field_state_path}")

    field_vertices_scaled, field_faces, heat_norm, axis_scalar, offset, clearance_state, t_coef_state = _load_field_state(
        field_state_path
    )

    scan_mesh = load_triangle_mesh_legacy(scan_mesh_path)
    scan_mesh.compute_vertex_normals()
    scan_mesh.paint_uniform_color(np.array([0.72, 0.72, 0.72], dtype=np.float64))
    scene, z_top = make_scan_scene(scan_mesh)

    clearance = float(clearance_state if clearance_override is None else clearance_override)
    pose = evaluate_fixed_pose(
        scene=scene,
        z_top=z_top,
        field_vertices_scaled=field_vertices_scaled,
        offset_xyz=offset,
        clearance=clearance,
        iso_level=float(iso_level),
    )

    contour_points, contour_lines = extract_phi_contour(
        vertices=pose.field_vertices_world,
        faces=field_faces,
        scalar=pose.phi,
        iso=float(iso_level),
    )

    candidate_scalar = heat_norm
    axis_scalar_used = axis_scalar
    recompute_seed_count = 0
    recompute_seed_iso_used = None
    recompute_used_t = None
    if bool(recompute_field_from_contour):
        t_used = float(t_coef_state if recompute_t_coef is None else recompute_t_coef)
        seed_iso = float(recompute_seed_iso)
        geod, seed_vertices = compute_geodesic_from_phi_contour(
            vertices=pose.field_vertices_world,
            faces=field_faces,
            phi=pose.phi,
            iso_level=seed_iso,
            t_coef=t_used,
        )

        if seed_vertices.size > 0 and np.any(np.isfinite(geod)):
            candidate_scalar = _normalize_scalar(geod)
            axis_scalar_used = np.asarray(geod, dtype=np.float64)
            recompute_seed_count = int(seed_vertices.size)
            recompute_seed_iso_used = seed_iso
            recompute_used_t = t_used

    mode = str(candidate_mode).strip().lower()
    selected = generate_print_points(
        polyline_points=contour_points,
        polyline_lines=contour_lines,
        field_vertices_world=pose.field_vertices_world,
        field_scalar=candidate_scalar,
        count=int(print_count),
        min_spacing=1e-3 * float(print_min_spacing_mm),
        candidate_mode=mode,
        lift_height=1e-3 * float(print_height_mm),
        field_faces=field_faces,
        walk_distance=1e-3 * float(walk_distance_mm),
        walk_step=1e-3 * float(walk_step_mm),
        walk_max_steps=int(walk_max_steps),
        walk_tangent_sign=float(walk_tangent_sign),
        # Cone clamp is applied at orientation stage (post point generation),
        # so point locations stay consistent across candidate modes.
        clamp_to_cone=False,
        cone_max_tilt_deg=float(cone_max_tilt_deg),
        agent_phi_scalar=pose.phi,
    )

    selected_surface_points = selected.surface_points
    if selected_surface_points is None:
        selected_surface_points = selected.source_points
    if selected_surface_points is None:
        selected_surface_points = selected.points.copy()
    prelift_offset_m = 1e-3 * float(prelift_geodesic_offset_mm)
    if prelift_offset_m > 0.0 and selected_surface_points.shape[0] > 0:
        offset_points, _offset_lines, _signed_geod, _seed_vertices = extract_offset_phi_contour(
            vertices=pose.field_vertices_world,
            faces=field_faces,
            phi=pose.phi,
            iso_level=float(iso_level),
            offset_distance=float(prelift_offset_m),
            toward_unprinted=True,
            t_coef=float(t_coef_state),
        )
        if offset_points.shape[0] > 0:
            d2 = np.sum(
                (selected_surface_points[:, None, :] - offset_points[None, :, :]) ** 2,
                axis=2,
            )
            nn = np.argmin(d2, axis=1).astype(np.int64)
            selected_surface_points = offset_points[nn]

    prelift_points = selected_surface_points.copy()
    if mode == "z_lift" and prelift_points.shape[0] > 0:
        prelift_points[:, 2] += 1e-3 * float(print_height_mm)

    projected_points = project_points_to_surface(
        query_points=prelift_points,
        mesh_vertices=pose.field_vertices_world,
        mesh_faces=field_faces,
    )

    t_dir, b_dir, n_dir = sample_tangent_axes_on_surface_from_scalar(
        query_points=projected_points,
        mesh_vertices=pose.field_vertices_world,
        mesh_faces=field_faces,
        vertex_scalar=axis_scalar_used,
        tangent_sign=float(tangent_sign),
    )
    t_dir, b_dir = _align_tangent_frame_sign(
        t_dir=t_dir,
        b_dir=b_dir,
        order_idx=np.asarray(selected.polyline_indices, dtype=np.int64),
    )
    if bool(clamp_to_cone):
        t_dir = clamp_vectors_to_cone(
            t_dir,
            max_tilt_deg=float(cone_max_tilt_deg),
            cone_axis=(0.0, 0.0, 1.0),
        )
        b_dir = clamp_vectors_to_cone(
            b_dir,
            max_tilt_deg=float(cone_max_tilt_deg),
            cone_axis=(0.0, 0.0, 1.0),
        )
    frame_scale = 1e-3 * float(frame_scale_mm)
    t_lines = _build_axis_lines(projected_points, t_dir, scale=frame_scale, color=(0.10, 0.95, 0.10))
    b_lines = _build_axis_lines(projected_points, b_dir, scale=frame_scale, color=(0.20, 0.60, 1.00))
    n_lines = _build_axis_lines(projected_points, n_dir, scale=frame_scale, color=(1.00, 0.30, 0.30))

    colors = yellow_to_red_colors(candidate_scalar)
    colors = np.asarray(colors, dtype=np.float64)
    colors[~pose.viable] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
    field_pcd = make_point_cloud(pose.field_vertices_world, colors)
    contour_ls = make_line_set(contour_points, contour_lines, color=(0.0, 1.0, 1.0))
    candidate_colors = np.tile(np.array([1.0, 0.0, 1.0], dtype=np.float64), (projected_points.shape[0], 1))
    candidates_pcd = make_point_cloud(projected_points, candidate_colors)
    lifted_debug_colors = np.tile(np.array([0.95, 0.95, 0.95], dtype=np.float64), (prelift_points.shape[0], 1))
    lifted_debug_pcd = make_point_cloud(prelift_points, lifted_debug_colors)

    out_dir.mkdir(parents=True, exist_ok=True)
    out_field = out_dir / "field_masked_replay_cycle26.ply"
    out_contour = out_dir / "field_phi0_contour_replay_cycle26.ply"
    out_candidates = out_dir / "field_print_candidates_replay_cycle26.ply"
    out_candidates_lifted = out_dir / "field_print_candidates_lifted_replay_cycle26.ply"
    out_candidates_surface = out_dir / "field_print_candidates_surface_samples_replay_cycle26.ply"
    out_t = out_dir / "field_candidate_t_axis_replay_cycle26.ply"
    out_b = out_dir / "field_candidate_b_axis_replay_cycle26.ply"
    out_n = out_dir / "field_candidate_n_axis_replay_cycle26.ply"

    o3d.io.write_point_cloud(str(out_field), field_pcd)
    if contour_lines.shape[0] > 0:
        o3d.io.write_line_set(str(out_contour), contour_ls)
    if projected_points.shape[0] > 0:
        o3d.io.write_point_cloud(str(out_candidates), candidates_pcd)
        o3d.io.write_point_cloud(str(out_candidates_lifted), lifted_debug_pcd)
        o3d.io.write_point_cloud(
            str(out_candidates_surface),
            make_point_cloud(
                selected_surface_points,
                np.tile(np.array([0.0, 1.0, 1.0], dtype=np.float64), (selected_surface_points.shape[0], 1)),
            ),
        )
        o3d.io.write_line_set(str(out_t), t_lines)
        o3d.io.write_line_set(str(out_b), b_lines)
        o3d.io.write_line_set(str(out_n), n_lines)

    print(f"scan_mesh: {scan_mesh_path}")
    print(f"field_state: {field_state_path}")
    print(f"offset_xyz: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}]")
    print(f"clearance used: {clearance:.6f}")
    print(f"iso_level: {float(iso_level):.6f}")
    print(f"viable vertices: {pose.viable_count}/{pose.field_vertices_world.shape[0]}")
    print(f"contour segments: {contour_lines.shape[0]}")
    print(f"print candidates: {projected_points.shape[0]}")
    print(f"candidate_mode: {mode}")
    if mode == "gradient_lift":
        print(
            "gradient_lift config: "
            f"distance_mm={float(print_height_mm):.3f} "
            f"tangent_sign={float(walk_tangent_sign):+.1f}"
        )
    if mode == "gradient_walk":
        print(
            "gradient_walk config: "
            f"distance_mm={float(walk_distance_mm):.3f} "
            f"step_mm={float(walk_step_mm):.3f} "
            f"max_steps={int(walk_max_steps)} "
            f"tangent_sign={float(walk_tangent_sign):+.1f}"
        )
    print(
        "orientation clamp config: "
        f"enabled={bool(clamp_to_cone)} "
        f"cone_max_tilt_deg={float(cone_max_tilt_deg):.3f}"
    )
    print(f"prelift_geodesic_offset_mm: {float(prelift_geodesic_offset_mm):.3f}")
    if selected_surface_points.shape[0] > 0:
        phi_sel = sample_vertex_scalar_on_surface(
            query_points=selected_surface_points,
            mesh_vertices=pose.field_vertices_world,
            mesh_faces=field_faces,
            vertex_scalar=pose.phi,
        )
        print(
            "phi at selected surface points before lift: "
            f"max_abs={float(np.max(np.abs(phi_sel))):.6f}"
        )
        print(
            "phi values before lift: "
            "[" + ", ".join(f"{float(v):.6f}" for v in phi_sel.tolist()) + "]"
        )
    if bool(recompute_field_from_contour) and selected_surface_points.shape[0] > 0:
        phi_before_lift = sample_vertex_scalar_on_surface(
            query_points=selected_surface_points,
            mesh_vertices=pose.field_vertices_world,
            mesh_faces=field_faces,
            vertex_scalar=pose.phi,
        )
        phi_str = ", ".join(f"{float(v):.6f}" for v in phi_before_lift.tolist())
        print(f"phi at source points before lift (recompute): [{phi_str}]")
        print(
            "phi|abs| summary before lift (recompute): "
            f"max_abs={float(np.max(np.abs(phi_before_lift))):.6f}"
        )
    if projected_points.shape[0] > 0:
        d_proj = np.linalg.norm(projected_points - prelift_points, axis=1)
        print(
            "candidate projection (lifted->surface) m: "
            f"min={float(np.min(d_proj)):.6f} "
            f"mean={float(np.mean(d_proj)):.6f} "
            f"max={float(np.max(d_proj)):.6f}"
        )
    src_points = selected.source_points
    if src_points is not None and projected_points.shape[0] > 0 and src_points.shape[0] == projected_points.shape[0]:
        d_src_final = np.linalg.norm(projected_points - src_points, axis=1)
        print(
            "candidate displacement (source->final) m: "
            f"min={float(np.min(d_src_final)):.6f} "
            f"mean={float(np.mean(d_src_final)):.6f} "
            f"max={float(np.max(d_src_final)):.6f}"
        )
    if recompute_used_t is None:
        print("recompute_field_from_contour: False (using stored heat field)")
    else:
        print(
            "recompute_field_from_contour: True "
            f"(seed_iso={float(recompute_seed_iso_used):.6f}, "
            f"seed_vertices={recompute_seed_count}, t_coef={recompute_used_t:.3f})"
        )
    print(f"output dir: {out_dir}")

    if visualize:
        geoms: list[o3d.geometry.Geometry] = [scan_mesh]
        scan_edges = o3d.geometry.LineSet.create_from_triangle_mesh(scan_mesh)
        scan_edges.paint_uniform_color((0.05, 0.05, 0.05))
        geoms.append(scan_edges)
        geoms.extend([field_pcd, contour_ls, candidates_pcd, t_lines, b_lines, n_lines])
        if float(axis_size) > 0.0:
            geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=float(axis_size)))
        o3d.visualization.draw_geometries(geoms, point_show_normal=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay scalar field candidate generation on a real cycle.")
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_SCAN_MESH)
    parser.add_argument("--field-state", type=Path, default=DEFAULT_FIELD_STATE)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--print-count", type=int, default=7)
    parser.add_argument("--print-min-spacing-mm", type=float, default=16.0)
    parser.add_argument("--print-height-mm", type=float, default=12.0)
    parser.add_argument(
        "--candidate-mode",
        type=str,
        choices=("z_lift", "gradient_lift", "gradient_walk"),
        default="z_lift",
        help="Candidate generation mode before projection/orientation.",
    )
    parser.add_argument("--walk-distance-mm", type=float, default=12.0, help="Gradient-walk target displacement.")
    parser.add_argument("--walk-step-mm", type=float, default=1.0, help="Gradient-walk step size.")
    parser.add_argument("--walk-max-steps", type=int, default=32, help="Gradient-walk max iterations.")
    parser.add_argument(
        "--walk-tangent-sign",
        type=float,
        default=1.0,
        help="Gradient-walk tangent sign (+1 or -1).",
    )
    parser.add_argument(
        "--clamp-to-cone",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Clamp final orientation vectors to a cone around world +Z.",
    )
    parser.add_argument(
        "--cone-max-tilt-deg",
        type=float,
        default=45.0,
        help="Cone semi-angle in degrees when --clamp-to-cone is enabled.",
    )
    parser.add_argument(
        "--iso-level",
        type=float,
        default=0.0,
        help="Iso-value used to extract candidate contour and viability threshold.",
    )
    parser.add_argument(
        "--clearance",
        type=float,
        default=None,
        help="Override clearance from state. If omitted, use saved state value.",
    )
    parser.add_argument(
        "--recompute-field-from-contour",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="If true, recompute scalar field each cycle using phi contour as heat seeds.",
    )
    parser.add_argument(
        "--recompute-t-coef",
        type=float,
        default=None,
        help="t_coef for recomputed contour-seeded heat field (default: use t_coef from state).",
    )
    parser.add_argument(
        "--recompute-seed-iso",
        type=float,
        default=0.0,
        help="Iso-value used as contour seed set for recomputed heat field.",
    )
    parser.add_argument(
        "--prelift-geodesic-offset-mm",
        type=float,
        default=0.0,
        help="Sample points at this geodesic offset from phi contour before z-lift (0 disables).",
    )
    parser.add_argument("--tangent-sign", type=float, default=1.0, help="Use +1 or -1 to flip tangent direction.")
    parser.add_argument("--frame-scale-mm", type=float, default=8.0, help="Length of t/b/n debug axes.")
    parser.add_argument("--axis-size", type=float, default=0.05)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    run(
        scan_mesh_path=args.scan_mesh,
        field_state_path=args.field_state,
        out_dir=args.out_dir,
        print_count=args.print_count,
        print_min_spacing_mm=args.print_min_spacing_mm,
        print_height_mm=args.print_height_mm,
        iso_level=args.iso_level,
        clearance_override=args.clearance,
        recompute_field_from_contour=bool(args.recompute_field_from_contour),
        recompute_seed_iso=args.recompute_seed_iso,
        recompute_t_coef=args.recompute_t_coef,
        prelift_geodesic_offset_mm=args.prelift_geodesic_offset_mm,
        candidate_mode=args.candidate_mode,
        walk_distance_mm=args.walk_distance_mm,
        walk_step_mm=args.walk_step_mm,
        walk_max_steps=args.walk_max_steps,
        walk_tangent_sign=args.walk_tangent_sign,
        clamp_to_cone=bool(args.clamp_to_cone),
        cone_max_tilt_deg=args.cone_max_tilt_deg,
        tangent_sign=args.tangent_sign,
        frame_scale_mm=args.frame_scale_mm,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
