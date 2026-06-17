#!/usr/bin/env python3
"""Simulate scalar-field print loop with DDS bead accumulation.

Stage flow per loop iteration:
1) Position field over current scan (only first step)
2) Keep that field pose fixed and recompute phi vs updated scan
3) Generate print candidates (`geodesic`, `z_lift`, `gradient_lift`, or `gradient_walk`)
4) Enforce bead separation only inside current step
5) Add selected beads to a DDS Simulator
6) Extract a DDS implicit-surface proxy for the next scan intersection

Sample command:
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator_dds_v2.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-subdivide-iter 1 --field-scale 0.001 \
  --candidate-mode gradient_walk \
  --beads-per-step 7 --bead-separation-mm 16 \
  --bead-height-mm 12 --bead-shape cylinder \
  --positioning-attempts 3 \
  --search-step-x 0.01 --search-step-y 0.01 \
  --offset-distance-mm 12 --offset-geodesic-delta-mm 0.6 \
  --axis-size -1

  python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator_dds_v2.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --output-dir /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/output/loop_sim \
  --seed-level 1 \
  --t-coef 2000 \
  --field-subdivide-iter 1 \
  --field-scale 0.001 \
  --clearance 0.0 \
  --candidate-mode gradient_walk \
  --offset-distance-mm 12 \
  --offset-geodesic-delta-mm 0.6 \
  --beads-per-step 7 \
  --bead-separation-mm 16 \
  --bead-height-mm 12 \
  --bead-shape cylinder \
  --walk-distance-mm 12 \
  --walk-step-mm 1.0 \
  --walk-max-steps 32 \
  --walk-tangent-sign 1.0 \
  --walk-start-fraction 0.25 \
  --cone-max-tilt-deg 45 \
  --positioning-attempts 3 \
  --search-step-x 0.01 \
  --search-step-y 0.01 \
  --base-z-offset 0.000001 \
  --axis-size -1 \
  --dds-voxel-size-mm 2.0 \
  --dds-threshold 0.5 \
  --dds-padding-mm 24 \
  --dds-surface-step-size 1 \
  --dds-view-mode surface
Controls:
- DDS window: `N` = apply current step and go next, `Q`/`Esc` = stop loop
- `--no-vis` mode: type `n` to apply+next, `q` to stop
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.geometry import load_triangle_mesh_arrays, load_triangle_mesh_legacy
from lib_scalar.loop_simulation import (
    compute_offset_contour_stage,
    generate_step_candidates,
    position_field_with_attempts,
)
from lib_scalar.print_targets import build_oriented_line_targets
from lib_scalar.viz import (
    compute_scene_bounds,
    make_line_set,
    make_point_cloud,
    make_segment_line_set,
    make_target_orientation_sticks,
    yellow_to_red_colors,
)


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
    source_points: np.ndarray | None = None,
    segment_start_points: np.ndarray | None = None,
    target_points: np.ndarray | None = None,
    target_z_dirs: np.ndarray | None = None,
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

    if (
        source_points is not None
        and source_points.shape == selected_points.shape
        and selected_points.shape[0] > 0
    ):
        source_colors = np.tile(np.array([0.0, 1.0, 1.0], dtype=np.float64), (source_points.shape[0], 1))
        source_pcd = make_point_cloud(source_points, source_colors)
        o3d.io.write_point_cloud(str(output_dir / f"{tag}_print_sources.ply"), source_pcd)

        line_set = make_segment_line_set(source_points, selected_points, color=(1.0, 1.0, 0.0))
        o3d.io.write_line_set(str(output_dir / f"{tag}_print_source_to_candidate.ply"), line_set)

    if (
        segment_start_points is not None
        and segment_start_points.shape == selected_points.shape
        and selected_points.shape[0] > 0
    ):
        start_colors = np.tile(
            np.array([1.0, 0.55, 0.0], dtype=np.float64),
            (segment_start_points.shape[0], 1),
        )
        start_pcd = make_point_cloud(segment_start_points, start_colors)
        o3d.io.write_point_cloud(str(output_dir / f"{tag}_print_segment_starts.ply"), start_pcd)

        segment_set = make_segment_line_set(segment_start_points, selected_points, color=(1.0, 0.55, 0.0))
        o3d.io.write_line_set(str(output_dir / f"{tag}_print_segments.ply"), segment_set)

    if (
        target_points is not None
        and target_z_dirs is not None
        and target_points.shape == target_z_dirs.shape
        and target_points.shape[0] > 0
    ):
        orientation_mesh = make_target_orientation_sticks(target_points, target_z_dirs)
        if len(orientation_mesh.triangles) > 0:
            o3d.io.write_triangle_mesh(
                str(output_dir / f"{tag}_target_orientations.ply"),
                orientation_mesh,
            )


def wait_next_terminal(step_index: int) -> bool:
    """Wait for user next/quit command in terminal mode."""
    while True:
        ans = input(f"[loop] step {step_index} ready. type 'n' to apply+next or 'q' to stop: ").strip().lower()
        if ans in ("n", ""):
            return True
        if ans == "q":
            return False
        print("[loop] invalid input. use 'n' or 'q'.")


def build_dds_domain_from_scene(
    scan_mesh: o3d.geometry.TriangleMesh,
    field_vertices_world: np.ndarray,
    *,
    voxel_size_m: float,
    padding_m: float,
):
    """Build a fixed DDS domain covering scan, field, and bead support."""
    from dds import Domain

    scan_vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
    field_vertices = np.asarray(field_vertices_world, dtype=np.float64)
    bb_min, bb_max = compute_scene_bounds(scan_vertices, field_vertices)
    pad = max(float(padding_m), float(voxel_size_m))
    return Domain.from_bounds(
        xmin=float(bb_min[0] - pad),
        xmax=float(bb_max[0] + pad),
        ymin=float(bb_min[1] - pad),
        ymax=float(bb_max[1] + pad),
        zmin=float(bb_min[2] - pad),
        zmax=float(bb_max[2] + pad),
        voxel_size=float(voxel_size_m),
        length_unit="m",
    )


def _dds_target(point: np.ndarray, normal: np.ndarray):
    from dds import DepositionTarget

    p = np.asarray(point, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)
    n_norm = float(np.linalg.norm(n))
    if n_norm <= 1e-12:
        n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        n = n / n_norm
    return DepositionTarget(position=tuple(float(v) for v in p), normal=tuple(float(v) for v in n))


def build_dds_step_deposits(
    candidate_points: np.ndarray,
    *,
    profile,
) -> tuple[object, ...]:
    """Convert current scalar candidates into point deposits.

    The scalar gradient-walk segment is a tool-motion/debug overlay. For the DDS
    proxy bead state we only deposit the selected end/candidate point.
    """
    from dds import PointDeposit

    bead_normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    points = np.asarray(candidate_points, dtype=np.float64)
    deposits = []
    for point in points:
        target = _dds_target(point, bead_normal)
        deposits.append(PointDeposit(target=target, profile=profile))
    return tuple(deposits)


def dds_surface_to_open3d_mesh(surface_mesh) -> o3d.geometry.TriangleMesh:
    """Convert a DDS TriangleMesh to an Open3D mesh for raycasting."""
    mesh = o3d.geometry.TriangleMesh()
    if surface_mesh.is_empty:
        return mesh
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(surface_mesh.vertices, dtype=np.float64).copy())
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(surface_mesh.faces, dtype=np.int32).copy())
    mesh.compute_vertex_normals()
    return mesh


def compose_scan_with_dds_proxy(
    base_scan_mesh: o3d.geometry.TriangleMesh,
    dds_result,
    *,
    threshold: float,
    mesh_step_size: int,
) -> tuple[o3d.geometry.TriangleMesh, int, int]:
    """Merge the original scan mesh with the DDS occupancy surface proxy."""
    scan_with_proxy = copy.deepcopy(base_scan_mesh)
    surface_mesh = dds_result.analysis.surface_mesh(
        threshold=float(threshold),
        step_size=int(mesh_step_size),
    )
    proxy_mesh = dds_surface_to_open3d_mesh(surface_mesh)
    proxy_faces = int(np.asarray(proxy_mesh.triangles).shape[0])
    if proxy_faces > 0:
        scan_with_proxy += proxy_mesh
    occupied = int(np.count_nonzero(dds_result.analysis.occupancy(threshold=float(threshold))))
    return scan_with_proxy, occupied, proxy_faces


def show_step_window(
    step_index: int,
    viewer_state: dict[str, object],
    dds_simulator,
    dds_threshold: float,
    dds_view_mode: str,
    field_vertices_world: np.ndarray,
    heat_colors: np.ndarray,
    viable_mask: np.ndarray,
    scan_mesh: o3d.geometry.TriangleMesh,
    contour_points: np.ndarray,
    contour_lines: np.ndarray,
    offset_points: np.ndarray,
    offset_lines: np.ndarray,
    selected_points: np.ndarray,
    source_points: np.ndarray | None,
    segment_start_points: np.ndarray | None,
    target_points: np.ndarray | None,
    target_z_dirs: np.ndarray | None,
    axis_size: float,
) -> bool:
    """Visualize one loop step in the DDS workbench and wait for N/Q key."""
    import dds.viz
    from PySide6 import QtCore
    from dds.viz import ViewConfig
    from lib_scalar.viz_dds import (
        attach_viewer,
        add_colored_point_cloud,
        add_line_segments,
        add_vector_arrows,
        add_wire_mesh,
        remove_overlay,
    )

    view_mode = str(dds_view_mode).strip().lower()
    decision = {"next": False, "quit": False}
    viewer_state["decision"] = decision

    workbench = viewer_state.get("workbench")
    if workbench is None:
        scalar_field = None
        if view_mode == "implicit":
            scalar_field = "coverage"
        elif view_mode == "occupancy":
            scalar_field = "occupancy"
        workbench = dds.viz.show(
            dds_simulator,
            threshold=float(dds_threshold),
            initial_view=ViewConfig(
                view_mode=view_mode,
                scalar_field=scalar_field,
                show_toolpath=True,
                show_targets=False,
            ),
            off_screen=False,
        )
        overlay = attach_viewer(workbench.plotter)
        viewer_state["workbench"] = workbench
        viewer_state["overlay"] = overlay

        def _finish(next_step: bool) -> None:
            current = viewer_state.get("decision")
            if isinstance(current, dict):
                current["next"] = bool(next_step)
                current["quit"] = not bool(next_step)
            event_loop = viewer_state.get("event_loop")
            if not bool(next_step):
                viewer_state["closed"] = True
                workbench.close()
            if isinstance(event_loop, QtCore.QEventLoop):
                event_loop.quit()

        workbench.plotter.add_key_event("n", lambda: _finish(True))
        workbench.plotter.add_key_event("N", lambda: _finish(True))
        workbench.plotter.add_key_event("q", lambda: _finish(False))
        workbench.plotter.add_key_event("Q", lambda: _finish(False))
        workbench.plotter.add_key_event("Escape", lambda: _finish(False))
    else:
        workbench.refresh(dds_simulator)
        overlay = viewer_state["overlay"]

    workbench.setWindowTitle(f"BEHAV3D DDS Scalar Loop Step {step_index}")

    overlay.clear()
    for name in (
        "field_heat_masked",
        "scan_wire",
        "phi_contour",
        "offset_contour",
        "source_to_candidate",
        "print_segments",
        "target_orientations",
        "axis_x",
        "axis_y",
        "axis_z",
    ):
        remove_overlay(overlay, name)

    field_colors = heat_colors.copy()
    field_colors[~viable_mask] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
    add_colored_point_cloud(
        overlay,
        field_vertices_world,
        field_colors,
        name="field_heat_masked",
        point_size=3.0,
    )

    scan_vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
    scan_faces = np.asarray(scan_mesh.triangles, dtype=np.int64)
    add_wire_mesh(
        overlay,
        scan_vertices,
        scan_faces,
        name="scan_wire",
        color="#808080",
        line_width=1.0,
    )

    if contour_lines.shape[0] > 0:
        add_line_segments(
            overlay,
            contour_points,
            contour_lines,
            name="phi_contour",
            color="#00ffff",
            line_width=4.0,
        )

    if offset_lines.shape[0] > 0:
        add_line_segments(
            overlay,
            offset_points,
            offset_lines,
            name="offset_contour",
            color="#ff00ff",
            line_width=4.0,
        )

    if selected_points.shape[0] > 0:
        selected_colors = np.tile(
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            (selected_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            selected_points,
            selected_colors,
            name="selected_points",
            point_size=12.0,
            render_as_spheres=True,
        )

    if (
        source_points is not None
        and source_points.shape == selected_points.shape
        and selected_points.shape[0] > 0
    ):
        source_colors = np.tile(
            np.array([0.0, 1.0, 1.0], dtype=np.float64),
            (source_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            source_points,
            source_colors,
            name="source_points",
            point_size=8.0,
            render_as_spheres=True,
        )
        source_lines = np.column_stack(
            [
                np.arange(source_points.shape[0], dtype=np.int64),
                np.arange(source_points.shape[0], dtype=np.int64) + source_points.shape[0],
            ]
        )
        add_line_segments(
            overlay,
            np.vstack([source_points, selected_points]),
            source_lines,
            name="source_to_candidate",
            color="#ffff00",
            line_width=3.0,
        )

    if (
        segment_start_points is not None
        and segment_start_points.shape == selected_points.shape
        and selected_points.shape[0] > 0
    ):
        start_colors = np.tile(
            np.array([1.0, 0.55, 0.0], dtype=np.float64),
            (segment_start_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            segment_start_points,
            start_colors,
            name="segment_start_points",
            point_size=8.0,
            render_as_spheres=True,
        )
        print_segment_lines = np.column_stack(
            [
                np.arange(segment_start_points.shape[0], dtype=np.int64),
                np.arange(segment_start_points.shape[0], dtype=np.int64) + segment_start_points.shape[0],
            ]
        )
        add_line_segments(
            overlay,
            np.vstack([segment_start_points, selected_points]),
            print_segment_lines,
            name="print_segments",
            color="#ff8c00",
            line_width=4.0,
        )

    if (
        target_points is not None
        and target_z_dirs is not None
        and target_points.shape == target_z_dirs.shape
        and target_points.shape[0] > 0
    ):
        norms = np.linalg.norm(target_z_dirs, axis=1)
        valid = norms > 1e-12
        if np.any(valid):
            target_vectors = 0.008 * target_z_dirs[valid] / norms[valid, None]
            add_vector_arrows(
                overlay,
                target_points[valid],
                target_vectors,
                name="target_orientations",
                color="#21bf73",
            )

    bb_min, bb_max = compute_scene_bounds(field_vertices_world, scan_vertices)
    bb_diag = float(np.linalg.norm(bb_max - bb_min))
    bb_center = 0.5 * (bb_min + bb_max)
    axis_size_val = float(axis_size)
    if axis_size_val == 0.0:
        axis_size_val = max(1e-4, 0.15 * bb_diag)
    if axis_size_val > 0.0:
        axis_origin = bb_center.reshape(1, 3)
        add_vector_arrows(
            overlay,
            axis_origin,
            np.array([[axis_size_val, 0.0, 0.0]], dtype=np.float64),
            name="axis_x",
            color="#e74c3c",
        )
        add_vector_arrows(
            overlay,
            axis_origin,
            np.array([[0.0, axis_size_val, 0.0]], dtype=np.float64),
            name="axis_y",
            color="#27ae60",
        )
        add_vector_arrows(
            overlay,
            axis_origin,
            np.array([[0.0, 0.0, axis_size_val]], dtype=np.float64),
            name="axis_z",
            color="#2980b9",
        )

    print("[viz] DDS workbench controls: N=apply+next, Q/Esc=stop")
    event_loop = QtCore.QEventLoop()
    viewer_state["event_loop"] = event_loop
    event_loop.exec()
    viewer_state.pop("event_loop", None)

    if (not decision["next"]) and (not decision["quit"]):
        return False
    return bool(decision["next"])


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
    walk_distance_mm: float,
    walk_step_mm: float,
    walk_max_steps: int,
    walk_tangent_sign: float,
    walk_start_fraction: float,
    clamp_to_cone: bool,
    cone_max_tilt_deg: float,
    bead_shape: str,
    positioning_attempts: int,
    search_step_x: float,
    search_step_y: float,
    search_allow_partial_hit: bool,
    base_z_offset: float,
    axis_size: float,
    visualize: bool,
    dds_voxel_size_mm: float,
    dds_threshold: float,
    dds_padding_mm: float,
    dds_surface_step_size: int,
    dds_view_mode: str,
    save_dds_step_bundles: bool,
) -> None:
    from dds import BeadProfile, Simulator

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

    scan_mesh_base = load_triangle_mesh_legacy(scan_mesh_path)
    scan_mesh_current = copy.deepcopy(scan_mesh_base)
    all_selected_points: list[np.ndarray] = []
    locked_offset_xyz: tuple[float, float, float] | None = None
    dds_simulator: Simulator | None = None
    dds_result = None
    viewer_state: dict[str, object] = {}
    dds_profile = BeadProfile(
        width=1e-3 * float(bead_separation_mm + 2.0),
        height=1e-3 * float(bead_height_mm),
    )
    dds_voxel_size_m = 1e-3 * float(dds_voxel_size_mm)
    dds_padding_m = 1e-3 * float(dds_padding_mm)
    if dds_voxel_size_m <= 0.0:
        raise ValueError(f"dds_voxel_size_mm must be > 0, got {dds_voxel_size_mm}")
    if not 0.0 <= float(dds_threshold) <= 1.0:
        raise ValueError(f"dds_threshold must be in [0, 1], got {dds_threshold}")
    if int(dds_surface_step_size) < 1:
        raise ValueError(f"dds_surface_step_size must be >= 1, got {dds_surface_step_size}")

    print(
        "[config] "
        f"candidate_mode={candidate_mode} "
        f"proxy=dds_implicit_surface "
        f"legacy_bead_shape_arg={bead_shape} "
        f"bead_height_mm={float(bead_height_mm):.3f} "
        f"dds_bead_width_mm={float(bead_separation_mm + 2.0):.3f} "
        f"dds_voxel_size_mm={float(dds_voxel_size_mm):.3f} "
        f"dds_threshold={float(dds_threshold):.3f}"
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
                base_z_offset=float(base_z_offset),
                search_step_x=float(search_step_x),
                search_step_y=float(search_step_y),
                positioning_attempts=int(positioning_attempts),
                search_max_candidates=int(SEARCH_MAX_CANDIDATES),
                require_full_hit=not bool(search_allow_partial_hit),
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
            phi=pose.phi,
            mode=str(candidate_mode),
            beads_per_step=int(beads_per_step),
            bead_separation_mm=float(bead_separation_mm),
            bead_height_mm=float(bead_height_mm),
            walk_distance_mm=float(walk_distance_mm),
            walk_step_mm=float(walk_step_mm),
            walk_max_steps=int(walk_max_steps),
            walk_tangent_sign=float(walk_tangent_sign),
            walk_start_fraction=float(walk_start_fraction),
            clamp_to_cone=bool(clamp_to_cone),
            cone_max_tilt_deg=float(cone_max_tilt_deg),
        )

        print(
            f"[step {step_index}] "
            f"viable={pose.viable_count} "
            f"contour_segments={contour.contour_lines.shape[0]} "
            f"offset_segments={contour.offset_lines.shape[0]} "
            f"offset_z_valid={candidate.z_valid_count} "
            f"selected={candidate.points.shape[0]}"
        )

        target_points = np.zeros((0, 3), dtype=np.float64)
        target_z_dirs = np.zeros((0, 3), dtype=np.float64)
        if (
            str(candidate_mode).strip().lower() == "gradient_walk"
            and candidate.segment_start_points is not None
            and candidate.segment_start_points.shape == candidate.points.shape
            and candidate.points.shape[0] > 0
        ):
            line_targets = build_oriented_line_targets(
                start_points=candidate.segment_start_points,
                end_points=candidate.points,
                field_vertices_world=pose.field_vertices_world,
                field_faces=field_faces,
                field_scalar=heat.norm,
                tangent_sign=float(walk_tangent_sign),
                clamp_to_cone=bool(clamp_to_cone),
                cone_max_tilt_deg=float(cone_max_tilt_deg),
            )
            target_points, target_z_dirs = line_targets.flattened_points_and_z_dirs()

        if dds_simulator is None:
            dds_domain = build_dds_domain_from_scene(
                scan_mesh=scan_mesh_base,
                field_vertices_world=pose.field_vertices_world,
                voxel_size_m=dds_voxel_size_m,
                padding_m=max(
                    dds_padding_m,
                    1e-3 * float(offset_distance_mm),
                    1e-3 * float(bead_height_mm),
                    1e-3 * float(bead_separation_mm + 2.0),
                ),
            )
            dds_simulator = Simulator(dds_domain)
            print(
                "[dds] domain: "
                f"min={dds_domain.min_corner} max={dds_domain.max_corner} "
                f"grid_shape={dds_domain.grid_shape} voxel_size={dds_domain.voxel_size}"
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
            source_points=candidate.source_points,
            segment_start_points=candidate.segment_start_points,
            target_points=target_points,
            target_z_dirs=target_z_dirs,
        )

        if visualize:
            assert dds_simulator is not None
            apply_step = show_step_window(
                step_index=step_index,
                viewer_state=viewer_state,
                dds_simulator=dds_simulator,
                dds_threshold=float(dds_threshold),
                dds_view_mode=str(dds_view_mode),
                field_vertices_world=pose.field_vertices_world,
                heat_colors=heat_colors,
                viable_mask=pose.viable,
                scan_mesh=scan_mesh_current,
                contour_points=contour.contour_points,
                contour_lines=contour.contour_lines,
                offset_points=contour.offset_points,
                offset_lines=contour.offset_lines,
                selected_points=candidate.points,
                source_points=candidate.source_points,
                segment_start_points=candidate.segment_start_points,
                target_points=target_points,
                target_z_dirs=target_z_dirs,
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

        step_deposits = build_dds_step_deposits(
            candidate.points,
            profile=dds_profile,
        )

        assert dds_simulator is not None
        dds_simulator.add_deposits(step_deposits)
        dds_result = dds_simulator.result(
            include_coverage=True,
            threshold=float(dds_threshold),
        )
        scan_mesh_current, occupied_voxels, proxy_faces = compose_scan_with_dds_proxy(
            base_scan_mesh=scan_mesh_base,
            dds_result=dds_result,
            threshold=float(dds_threshold),
            mesh_step_size=int(dds_surface_step_size),
        )
        print(
            f"[dds] step {step_index}: "
            f"added_deposits={len(step_deposits)} "
            f"total_deposits={len(dds_simulator.deposits)} "
            f"occupied_voxels={occupied_voxels} "
            f"proxy_faces={proxy_faces} "
            f"implicit_max={float(dds_result.implicit_field.max()):.3f}"
        )
        if save_dds_step_bundles:
            bundle_dir = output_dir / f"step_{step:02d}_dds_bundle"
            dds_result.save(
                bundle_dir,
                metadata={
                    "step": step_index,
                    "candidate_mode": str(candidate_mode),
                    "threshold": float(dds_threshold),
                    "bead_width_m": float(dds_profile.width),
                    "bead_height_m": float(dds_profile.height),
                },
            )
            print(f"[dds] saved step bundle: {bundle_dir}")

        all_selected_points.append(candidate.points)
        step += 1

    workbench = viewer_state.get("workbench")
    if workbench is not None and not bool(viewer_state.get("closed", False)):
        workbench.close()

    output_dir.mkdir(parents=True, exist_ok=True)
    final_scan_path = output_dir / "loop_sim_scan_with_dds_proxy.ply"
    if not o3d.io.write_triangle_mesh(str(final_scan_path), scan_mesh_current):
        raise RuntimeError(f"Failed to write final scan + DDS proxy mesh: {final_scan_path}")
    print(f"[done] saved final scan + DDS proxy mesh: {final_scan_path}")

    if dds_result is not None:
        from dds.geometry import write_mesh

        final_bundle_dir = output_dir / "loop_sim_dds_bundle"
        dds_result.save(
            final_bundle_dir,
            metadata={
                "candidate_mode": str(candidate_mode),
                "threshold": float(dds_threshold),
                "bead_width_m": float(dds_profile.width),
                "bead_height_m": float(dds_profile.height),
            },
        )
        dds_surface = dds_result.analysis.surface_mesh(
            threshold=float(dds_threshold),
            step_size=int(dds_surface_step_size),
        )
        if not dds_surface.is_empty:
            surface_path = write_mesh(output_dir / "loop_sim_dds_surface.ply", dds_surface)
            print(f"[done] saved DDS surface mesh: {surface_path}")
        print(f"[done] saved DDS bundle: {final_bundle_dir}")

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
        description="Simulate iterative scalar-field print loop with DDS implicit bead proxy."
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
        choices=("geodesic", "z_lift", "gradient_lift", "gradient_walk"),
        default="geodesic",
    )
    parser.add_argument("--offset-distance-mm", type=float, default=12.0)
    parser.add_argument("--offset-geodesic-delta-mm", type=float, default=0.6)
    parser.add_argument("--beads-per-step", type=int, default=7)
    parser.add_argument("--bead-separation-mm", type=float, default=16.0)
    parser.add_argument("--bead-height-mm", type=float, default=12.0)
    parser.add_argument("--walk-distance-mm", type=float, default=12.0)
    parser.add_argument("--walk-step-mm", type=float, default=1.0)
    parser.add_argument("--walk-max-steps", type=int, default=32)
    parser.add_argument("--walk-tangent-sign", type=float, default=1.0)
    parser.add_argument("--walk-start-fraction", type=float, default=0.25)
    parser.add_argument("--clamp-to-cone", action="store_true")
    parser.add_argument("--cone-max-tilt-deg", type=float, default=45.0)
    parser.add_argument(
        "--bead-shape",
        type=str,
        choices=("cylinder", "sphere"),
        default="cylinder",
        help="Legacy compatibility argument; DDS proxy uses BeadProfile width/height.",
    )
    parser.add_argument("--positioning-attempts", type=int, default=3)
    parser.add_argument("--search-step-x", type=float, default=0.01)
    parser.add_argument("--search-step-y", type=float, default=0.01)
    parser.add_argument("--search-allow-partial-hit", action="store_true")
    parser.add_argument("--base-z-offset", type=float, default=BASE_Z_OFFSET)
    parser.add_argument("--axis-size", type=float, default=0.0)
    parser.add_argument(
        "--dds-voxel-size-mm",
        type=float,
        default=2.0,
        help="DDS dense-grid voxel size in mm for the bead proxy.",
    )
    parser.add_argument(
        "--dds-threshold",
        type=float,
        default=0.5,
        help="DDS implicit-field threshold used for occupancy and proxy surface extraction.",
    )
    parser.add_argument(
        "--dds-padding-mm",
        type=float,
        default=24.0,
        help="Minimum padding around scan/field bounds for the DDS domain, in mm.",
    )
    parser.add_argument(
        "--dds-surface-step-size",
        type=int,
        default=1,
        help="Marching-cubes step size for the DDS proxy surface.",
    )
    parser.add_argument(
        "--dds-view-mode",
        type=str,
        choices=("surface", "occupancy", "implicit"),
        default="surface",
        help="Initial DDS workbench representation.",
    )
    parser.add_argument(
        "--save-dds-step-bundles",
        action="store_true",
        help="Save a DDS bundle after every accepted loop step.",
    )
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
        walk_distance_mm=args.walk_distance_mm,
        walk_step_mm=args.walk_step_mm,
        walk_max_steps=args.walk_max_steps,
        walk_tangent_sign=args.walk_tangent_sign,
        walk_start_fraction=args.walk_start_fraction,
        clamp_to_cone=args.clamp_to_cone,
        cone_max_tilt_deg=args.cone_max_tilt_deg,
        bead_shape=args.bead_shape,
        positioning_attempts=args.positioning_attempts,
        search_step_x=args.search_step_x,
        search_step_y=args.search_step_y,
        search_allow_partial_hit=args.search_allow_partial_hit,
        base_z_offset=args.base_z_offset,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
        dds_voxel_size_mm=args.dds_voxel_size_mm,
        dds_threshold=args.dds_threshold,
        dds_padding_mm=args.dds_padding_mm,
        dds_surface_step_size=args.dds_surface_step_size,
        dds_view_mode=args.dds_view_mode,
        save_dds_step_bundles=args.save_dds_step_bundles,
    )


if __name__ == "__main__":
    main()
