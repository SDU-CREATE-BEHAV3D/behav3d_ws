#!/usr/bin/env python3
"""Simulate scalar-field print loop with bead accumulation on scan mesh.

Stage flow per loop iteration:
1) Position field over current scan (only first step)
2) Keep that field pose fixed and recompute phi vs updated scan
3) Generate print candidates (`geodesic` or `z_lift`)
4) Enforce bead separation only inside current step
5) Add selected beads as cylinders (or spheres) to scan mesh

Sample command:
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-subdivide-iter 1 --field-scale 0.001 \
  --candidate-mode z_lift \
  --beads-per-step 7 --bead-separation-mm 16 \
  --bead-height-mm 12 --bead-shape cylinder \
  --positioning-attempts 3 \
  --search-step-x 0.01 --search-step-y 0.01 \
  --offset-distance-mm 12 --offset-geodesic-delta-mm 0.6 \
  --axis-size -1

Controls:
- Open3D window: `N` = apply current step and go next, `Q`/`Esc` = stop loop
- `--no-vis` mode: type `n` to apply+next, `q` to stop
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.geometry import load_triangle_mesh_arrays, load_triangle_mesh_legacy
from lib_scalar.loop_simulation import (
    apply_simulated_beads,
    compute_offset_contour_stage,
    generate_step_candidates,
    position_field_with_attempts,
)
from lib_scalar.viz import compute_scene_bounds, make_line_set, make_point_cloud, yellow_to_red_colors


DEFAULT_FIELD_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
DEFAULT_SCAN_MESH = Path("/home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl")
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
ISO_LEVEL = 0.0
BASE_Z_OFFSET = 1e-6
SEARCH_MAX_CANDIDATES = 30000


def subdivide_field_mesh_loop(
    vertices: np.ndarray,
    faces: np.ndarray,
    iterations: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Optional Loop subdivision for higher contour/geodesic resolution."""
    it = int(iterations)
    if it <= 0:
        return vertices, faces
    if it > 4:
        raise ValueError(f"field_subdivide_iter too large ({it}); use 0..4.")

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    mesh_sub = mesh.subdivide_loop(number_of_iterations=it)
    v_sub = np.asarray(mesh_sub.vertices, dtype=np.float64)
    f_sub = np.asarray(mesh_sub.triangles, dtype=np.int32)
    return v_sub, f_sub


def save_step_outputs(
    output_dir: Path,
    step_index: int,
    field_vertices_world: np.ndarray,
    heat_colors: np.ndarray,
    viable_mask: np.ndarray,
    contour_points: np.ndarray,
    contour_lines: np.ndarray,
    offset_points: np.ndarray,
    offset_lines: np.ndarray,
    selected_points: np.ndarray,
) -> None:
    """Write debug artifacts for a single loop step."""
    output_dir.mkdir(parents=True, exist_ok=True)
    tag = f"step_{step_index:02d}"

    heat_pcd = make_point_cloud(field_vertices_world, heat_colors)
    o3d.io.write_point_cloud(str(output_dir / f"{tag}_field_heat.ply"), heat_pcd)

    masked_colors = heat_colors.copy()
    masked_colors[~viable_mask] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
    field_pcd = make_point_cloud(field_vertices_world, masked_colors)
    o3d.io.write_point_cloud(str(output_dir / f"{tag}_field_masked.ply"), field_pcd)

    if contour_lines.shape[0] > 0:
        contour_ls = make_line_set(contour_points, contour_lines, color=(0.0, 1.0, 1.0))
        o3d.io.write_line_set(str(output_dir / f"{tag}_phi_contour.ply"), contour_ls)

    if offset_lines.shape[0] > 0:
        offset_ls = make_line_set(offset_points, offset_lines, color=(1.0, 0.0, 1.0))
        o3d.io.write_line_set(str(output_dir / f"{tag}_offset_contour.ply"), offset_ls)

    if selected_points.shape[0] > 0:
        colors = np.tile(np.array([0.0, 1.0, 0.0], dtype=np.float64), (selected_points.shape[0], 1))
        selected_pcd = make_point_cloud(selected_points, colors)
        o3d.io.write_point_cloud(str(output_dir / f"{tag}_print_points.ply"), selected_pcd)


def wait_next_terminal(step_index: int) -> bool:
    """Wait for user next/quit command in terminal mode."""
    while True:
        ans = input(f"[loop] step {step_index} ready. type 'n' to apply+next or 'q' to stop: ").strip().lower()
        if ans in ("n", ""):
            return True
        if ans == "q":
            return False
        print("[loop] invalid input. use 'n' or 'q'.")


def show_step_window(
    step_index: int,
    field_vertices_world: np.ndarray,
    heat_colors: np.ndarray,
    viable_mask: np.ndarray,
    scan_mesh: o3d.geometry.TriangleMesh,
    contour_points: np.ndarray,
    contour_lines: np.ndarray,
    offset_points: np.ndarray,
    offset_lines: np.ndarray,
    selected_points: np.ndarray,
    axis_size: float,
) -> bool:
    """Visualize one loop step and wait for N/Q key.

    Viewer shows only viable field points with heat colors.
    """
    vis = o3d.visualization.VisualizerWithKeyCallback()
    ok = vis.create_window(window_name=f"scalar loop step {step_index}", width=1400, height=900)
    if not ok:
        raise RuntimeError("Failed to create Open3D visualization window.")

    state = {"next": False, "quit": False}

    def _next_cb(v: o3d.visualization.Visualizer) -> bool:
        state["next"] = True
        v.close()
        return False

    def _quit_cb(v: o3d.visualization.Visualizer) -> bool:
        state["quit"] = True
        v.close()
        return False

    vis.register_key_callback(ord("N"), _next_cb)
    vis.register_key_callback(ord("n"), _next_cb)
    vis.register_key_callback(ord("Q"), _quit_cb)
    vis.register_key_callback(ord("q"), _quit_cb)
    vis.register_key_callback(256, _quit_cb)  # Esc

    if np.any(viable_mask):
        heat_viable = make_point_cloud(field_vertices_world[viable_mask], heat_colors[viable_mask])
        vis.add_geometry(heat_viable)

    scan_wire = o3d.geometry.LineSet.create_from_triangle_mesh(scan_mesh)
    scan_wire.paint_uniform_color((0.5, 0.5, 0.5))
    vis.add_geometry(scan_wire)

    if contour_lines.shape[0] > 0:
        contour_ls = make_line_set(contour_points, contour_lines, color=(0.0, 1.0, 1.0))
        vis.add_geometry(contour_ls)

    if offset_lines.shape[0] > 0:
        offset_ls = make_line_set(offset_points, offset_lines, color=(1.0, 0.0, 1.0))
        vis.add_geometry(offset_ls)

    if selected_points.shape[0] > 0:
        colors = np.tile(np.array([0.0, 1.0, 0.0], dtype=np.float64), (selected_points.shape[0], 1))
        selected_pcd = make_point_cloud(selected_points, colors)
        vis.add_geometry(selected_pcd)

    bb_min, bb_max = compute_scene_bounds(field_vertices_world, np.asarray(scan_mesh.vertices, dtype=np.float64))
    bb_diag = float(np.linalg.norm(bb_max - bb_min))
    bb_center = 0.5 * (bb_min + bb_max)
    axis_size_val = float(axis_size)
    if axis_size_val == 0.0:
        axis_size_val = max(1e-4, 0.15 * bb_diag)
    if axis_size_val > 0.0:
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_size_val)
        axes.translate(bb_center)
        vis.add_geometry(axes)

    print("[viz] controls: N=apply+next, Q/Esc=stop")
    vis.run()
    vis.destroy_window()

    if (not state["next"]) and (not state["quit"]):
        return False
    return bool(state["next"])


def run(
    field_mesh_path: Path,
    scan_mesh_path: Path,
    output_dir: Path,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
    field_subdivide_iter: int,
    field_scale: float,
    clearance: float,
    candidate_mode: str,
    offset_distance_mm: float,
    offset_geodesic_delta_mm: float,
    beads_per_step: int,
    bead_separation_mm: float,
    bead_height_mm: float,
    bead_shape: str,
    positioning_attempts: int,
    search_step_x: float,
    search_step_y: float,
    axis_size: float,
    visualize: bool,
) -> None:
    field_mesh = load_triangle_mesh_arrays(field_mesh_path)
    base_vertices = field_mesh.vertices
    base_faces = field_mesh.faces
    field_vertices, field_faces = subdivide_field_mesh_loop(base_vertices, base_faces, int(field_subdivide_iter))
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
    heat_colors = yellow_to_red_colors(heat.norm)

    scan_mesh_current = load_triangle_mesh_legacy(scan_mesh_path)
    all_selected_points: list[np.ndarray] = []
    locked_offset_xyz: tuple[float, float, float] | None = None

    print(
        "[config] "
        f"candidate_mode={candidate_mode} "
        f"bead_shape={bead_shape} "
        f"bead_height_mm={float(bead_height_mm):.3f} "
        f"bead_diameter_mm={float(bead_separation_mm):.3f}"
    )

    step = 0
    while True:
        step_index = step + 1

        if locked_offset_xyz is None:
            pose = position_field_with_attempts(
                scan_mesh=scan_mesh_current,
                field_vertices_scaled=field_vertices_scaled,
                heat_norm=heat.norm,
                clearance=float(clearance),
                iso_level=float(ISO_LEVEL),
                base_z_offset=float(BASE_Z_OFFSET),
                search_step_x=float(search_step_x),
                search_step_y=float(search_step_y),
                positioning_attempts=int(positioning_attempts),
                search_max_candidates=int(SEARCH_MAX_CANDIDATES),
            )
            locked_offset_xyz = pose.offset_xyz
            print(
                "[pose] locked field offset from step 1: "
                f"x={locked_offset_xyz[0]:.6f} y={locked_offset_xyz[1]:.6f} z={locked_offset_xyz[2]:.6f}"
            )
        else:
            scene, z_top = make_scan_scene(scan_mesh_current)
            pose = evaluate_fixed_pose(
                scene=scene,
                z_top=z_top,
                field_vertices_scaled=field_vertices_scaled,
                offset_xyz=locked_offset_xyz,
                clearance=float(clearance),
                iso_level=float(ISO_LEVEL),
            )

        contour = compute_offset_contour_stage(
            pose=pose,
            field_faces=field_faces,
            iso_level=float(ISO_LEVEL),
            offset_distance_mm=float(offset_distance_mm),
            offset_geodesic_delta_mm=float(offset_geodesic_delta_mm),
            offset_t_coef=float(t_coef),
            scan_mesh=scan_mesh_current,
        )

        candidate = generate_step_candidates(
            contour=contour,
            field_faces=field_faces,
            field_vertices_world=pose.field_vertices_world,
            heat_norm=heat.norm,
            mode=str(candidate_mode),
            beads_per_step=int(beads_per_step),
            bead_separation_mm=float(bead_separation_mm),
            bead_height_mm=float(bead_height_mm),
        )

        print(
            f"[step {step_index}] "
            f"viable={pose.viable_count} "
            f"contour_segments={contour.contour_lines.shape[0]} "
            f"offset_segments={contour.offset_lines.shape[0]} "
            f"offset_z_valid={candidate.z_valid_count} "
            f"selected={candidate.points.shape[0]}"
        )

        save_step_outputs(
            output_dir=output_dir,
            step_index=step,
            field_vertices_world=pose.field_vertices_world,
            heat_colors=heat_colors,
            viable_mask=pose.viable,
            contour_points=contour.contour_points,
            contour_lines=contour.contour_lines,
            offset_points=contour.offset_points,
            offset_lines=contour.offset_lines,
            selected_points=candidate.points,
        )

        if visualize:
            apply_step = show_step_window(
                step_index=step_index,
                field_vertices_world=pose.field_vertices_world,
                heat_colors=heat_colors,
                viable_mask=pose.viable,
                scan_mesh=scan_mesh_current,
                contour_points=contour.contour_points,
                contour_lines=contour.contour_lines,
                offset_points=contour.offset_points,
                offset_lines=contour.offset_lines,
                selected_points=candidate.points,
                axis_size=axis_size,
            )
        else:
            apply_step = wait_next_terminal(step_index=step_index)

        if not apply_step:
            print("[loop] Stop requested by user.")
            break

        if candidate.points.shape[0] == 0:
            print("[loop] No valid candidates left; stopping early.")
            break

        scan_mesh_current, _ = apply_simulated_beads(
            scan_mesh=scan_mesh_current,
            print_points=candidate.points,
            bead_height_mm=float(bead_height_mm),
            bead_diameter_mm=float(bead_separation_mm+2),
            bead_shape=str(bead_shape),
        )
        all_selected_points.append(candidate.points)
        step += 1

    output_dir.mkdir(parents=True, exist_ok=True)
    final_scan_path = output_dir / "loop_sim_scan_with_beads.ply"
    if not o3d.io.write_triangle_mesh(str(final_scan_path), scan_mesh_current):
        raise RuntimeError(f"Failed to write final simulated scan mesh: {final_scan_path}")
    print(f"[done] saved final simulated scan mesh: {final_scan_path}")

    if all_selected_points:
        all_points = np.vstack(all_selected_points)
        colors = np.tile(np.array([0.0, 1.0, 0.0], dtype=np.float64), (all_points.shape[0], 1))
        all_pcd = make_point_cloud(all_points, colors)
        all_points_path = output_dir / "loop_sim_all_print_points.ply"
        if not o3d.io.write_point_cloud(str(all_points_path), all_pcd):
            raise RuntimeError(f"Failed to write accumulated print points: {all_points_path}")
        print(f"[done] saved all print points: {all_points_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simulate iterative scalar-field print loop with bead accumulation."
    )
    parser.add_argument("--field-mesh", type=Path, default=DEFAULT_FIELD_MESH)
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_SCAN_MESH)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR / "loop_sim")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--seed-level", type=float, default=None)
    parser.add_argument("--t-coef", type=float, default=2000.0)
    parser.add_argument("--field-subdivide-iter", type=int, default=1)
    parser.add_argument("--field-scale", type=float, default=0.001)
    parser.add_argument("--clearance", type=float, default=0.0)
    parser.add_argument(
        "--candidate-mode",
        type=str,
        choices=("geodesic", "z_lift"),
        default="geodesic",
    )
    parser.add_argument("--offset-distance-mm", type=float, default=12.0)
    parser.add_argument("--offset-geodesic-delta-mm", type=float, default=0.6)
    parser.add_argument("--beads-per-step", type=int, default=7)
    parser.add_argument("--bead-separation-mm", type=float, default=16.0)
    parser.add_argument("--bead-height-mm", type=float, default=12.0)
    parser.add_argument(
        "--bead-shape",
        type=str,
        choices=("cylinder", "sphere"),
        default="cylinder",
    )
    parser.add_argument("--positioning-attempts", type=int, default=3)
    parser.add_argument("--search-step-x", type=float, default=0.01)
    parser.add_argument("--search-step-y", type=float, default=0.01)
    parser.add_argument("--axis-size", type=float, default=0.0)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    run(
        field_mesh_path=args.field_mesh,
        scan_mesh_path=args.scan_mesh,
        output_dir=args.output_dir,
        seed=args.seed,
        seed_level=args.seed_level,
        t_coef=args.t_coef,
        field_subdivide_iter=args.field_subdivide_iter,
        field_scale=args.field_scale,
        clearance=args.clearance,
        candidate_mode=args.candidate_mode,
        offset_distance_mm=args.offset_distance_mm,
        offset_geodesic_delta_mm=args.offset_geodesic_delta_mm,
        beads_per_step=args.beads_per_step,
        bead_separation_mm=args.bead_separation_mm,
        bead_height_mm=args.bead_height_mm,
        bead_shape=args.bead_shape,
        positioning_attempts=args.positioning_attempts,
        search_step_x=args.search_step_x,
        search_step_y=args.search_step_y,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
