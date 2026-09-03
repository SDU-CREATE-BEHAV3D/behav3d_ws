#!/usr/bin/env python3
"""Sample complete bead actions, collision-filter them, and visualize the pool."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np


TRIALS_ROOT = Path(__file__).resolve().parents[1]
if str(TRIALS_ROOT) not in sys.path:
    sys.path.insert(0, str(TRIALS_ROOT))
SCALAR_FIELD_ROOT = TRIALS_ROOT.parent
if str(SCALAR_FIELD_ROOT) not in sys.path:
    sys.path.insert(0, str(SCALAR_FIELD_ROOT))

from dds import BeadProfile, DepositionTarget, Domain, PointDeposit, simulate
from dds.viz import MeshStyle, PointCloudStyle, Viewer
from lib_scalar.bead_profile import normalized_width_to_mm
from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.extruder_collision import (
    ExtruderCollisionChecker,
    load_collada_mesh,
)
from lib_scalar.geometry import sample_vertex_scalar_on_surface
from lib_scalar.viz_dds import (
    add_colored_point_cloud,
    add_line_segments,
    attach_viewer,
)
from rl_trials.action_candidates import (
    ActionCandidateBatch,
    CandidateValidation,
    sample_action_candidates,
    validate_action_candidates,
)
from rl_trials.fixture_io import (
    load_configured_geometry,
    load_configured_goal_fields,
    load_experiment_config,
    resolve_workspace_path,
)
from rl_trials.goal_evaluator import evaluate_goal_with_vertical_rays
from rl_trials.visualization import EvaluationScene, build_evaluation_scene


if TYPE_CHECKING:
    from dds.geometry import TriangleMesh


VALID_COLOR = np.array([255, 193, 7], dtype=np.uint8)
COLLISION_COLOR = np.array([231, 76, 60], dtype=np.uint8)
CONTACT_INVALID_COLOR = np.array([52, 152, 219], dtype=np.uint8)
SELECTED_COLOR = np.array([0, 230, 118], dtype=np.uint8)


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def _optional_float(cli_value: float | None, configured: float) -> float:
    return float(configured if cli_value is None else cli_value)


def _candidate_segments(
    candidates: ActionCandidateBatch,
    mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    indices = np.flatnonzero(mask)
    if indices.size == 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 2), dtype=np.int32)
    count = int(indices.size)
    points = np.vstack(
        (candidates.source_points[indices], candidates.target_points[indices])
    )
    lines = np.column_stack(
        (np.arange(count, dtype=np.int32), np.arange(count, dtype=np.int32) + count)
    )
    return points, lines


def _add_candidate_group(
    viewer,
    candidates: ActionCandidateBatch,
    mask: np.ndarray,
    *,
    name: str,
    color: str,
    point_color: np.ndarray,
    line_width: float,
) -> None:
    points, lines = _candidate_segments(candidates, mask)
    if lines.shape[0] == 0:
        return
    add_line_segments(
        viewer,
        points,
        lines,
        name=f"{name}_axes",
        color=color,
        opacity=0.8,
        line_width=line_width,
    )
    targets = candidates.target_points[np.asarray(mask, dtype=bool)]
    colors = np.tile(point_color, (targets.shape[0], 1))
    add_colored_point_cloud(
        viewer,
        targets,
        colors,
        name=f"{name}_targets",
        point_size=7.0,
        render_as_spheres=True,
    )


def _add_candidate_overlays(
    viewer,
    scene: EvaluationScene,
    candidates: ActionCandidateBatch,
    validation: CandidateValidation,
    selected_index: int | None,
) -> None:
    add_line_segments(
        viewer,
        scene.phi_contour_points,
        scene.phi_contour_lines,
        name="phi_zero_contour",
        color="#ff2d95",
        line_width=6.0,
        render_as_tubes=True,
    )
    _add_candidate_group(
        viewer,
        candidates,
        validation.valid,
        name="valid",
        color="#ffc107",
        point_color=VALID_COLOR,
        line_width=2.0,
    )
    _add_candidate_group(
        viewer,
        candidates,
        validation.contact_valid & validation.collides,
        name="collision",
        color="#e74c3c",
        point_color=COLLISION_COLOR,
        line_width=3.0,
    )
    _add_candidate_group(
        viewer,
        candidates,
        ~validation.contact_valid,
        name="contact_invalid",
        color="#3498db",
        point_color=CONTACT_INVALID_COLOR,
        line_width=3.0,
    )
    if selected_index is not None:
        selected_mask = np.zeros(candidates.count, dtype=bool)
        selected_mask[selected_index] = True
        _add_candidate_group(
            viewer,
            candidates,
            selected_mask,
            name="selected",
            color="#00e676",
            point_color=SELECTED_COLOR,
            line_width=7.0,
        )
        source = candidates.source_points[selected_index].reshape(1, 3)
        add_colored_point_cloud(
            viewer,
            source,
            np.tile(SELECTED_COLOR, (1, 1)),
            name="selected_source",
            point_size=11.0,
            render_as_spheres=True,
        )


def _selected_deposit_and_mesh(
    candidates: ActionCandidateBatch,
    selected_index: int,
) -> tuple[PointDeposit, TriangleMesh]:
    deposit = PointDeposit(
        target=DepositionTarget(
            position=candidates.target_points[selected_index],
            normal=candidates.z_axes[selected_index],
        ),
        profile=BeadProfile(
            width=float(candidates.widths_m[selected_index]),
            height=float(candidates.heights_m[selected_index]),
        ),
    )
    domain = Domain.from_deposits(
        deposit,
        voxel_size=0.001,
        padding=0.002,
        length_unit="m",
    )
    result = simulate(domain, deposit, threshold=0.5)
    return deposit, result.analysis.surface_mesh(threshold=0.5, step_size=1)


def _legend(
    candidates: ActionCandidateBatch,
    validation: CandidateValidation,
    selected_index: int | None,
    attempts: int,
) -> str:
    selected = "none"
    if selected_index is not None:
        selected = (
            f"{selected_index}: h={1e3 * candidates.heights_m[selected_index]:.2f} mm, "
            f"w={1e3 * candidates.widths_m[selected_index]:.2f} mm, "
            f"heat={candidates.heat[selected_index]:.3f}"
        )
    return (
        "Action proposal pool (no gradient selection)\n"
        "magenta: interpolated phi=0 polyline\n"
        "yellow: valid candidate\n"
        "red: extruder collision\n"
        "blue: contact invalid\n"
        "green: random valid selection + DDS bead\n\n"
        f"pool={candidates.count}, valid={int(np.count_nonzero(validation.valid))}, "
        f"collision={int(np.count_nonzero(validation.collides))}, "
        f"contact invalid={int(np.count_nonzero(~validation.contact_valid))}\n"
        f"generation attempts={attempts}\n"
        f"selected={selected}"
    )


def _add_base_scene_to_plotter(
    plotter,
    scene: EvaluationScene,
    pv,
    *,
    hide_scan: bool,
) -> None:
    from dds.viz.converters import point_cloud_to_polydata, triangle_mesh_to_polydata

    if not hide_scan:
        plotter.add_mesh(
            triangle_mesh_to_polydata(scene.scan_mesh, pv),
            color="#95a5a6",
            opacity=0.27,
            smooth_shading=False,
        )
    plotter.add_mesh(
        triangle_mesh_to_polydata(scene.goal_mesh, pv),
        scalars="vertex_colors",
        rgb=True,
        opacity=0.70,
        show_edges=True,
        smooth_shading=False,
        show_scalar_bar=False,
    )
    if not scene.frontier_points.is_empty:
        plotter.add_mesh(
            point_cloud_to_polydata(scene.frontier_points, pv),
            scalars="point_colors",
            rgb=True,
            point_size=3.0,
            render_points_as_spheres=True,
            show_scalar_bar=False,
        )


def _render_screenshot(
    *,
    path: Path,
    scene: EvaluationScene,
    candidates: ActionCandidateBatch,
    validation: CandidateValidation,
    selected_index: int | None,
    selected_bead_mesh: TriangleMesh | None,
    attempts: int,
    hide_scan: bool,
) -> None:
    import pyvista as pv
    from dds.viz.converters import point_cloud_to_polydata, triangle_mesh_to_polydata

    plotter = pv.Plotter(off_screen=True, window_size=(1600, 1000))
    plotter.set_background("#f5f6f8")
    _add_base_scene_to_plotter(plotter, scene, pv, hide_scan=hide_scan)
    overlay = attach_viewer(plotter)
    _add_candidate_overlays(
        overlay,
        scene,
        candidates,
        validation,
        selected_index,
    )
    if selected_bead_mesh is not None:
        plotter.add_mesh(
            triangle_mesh_to_polydata(selected_bead_mesh, pv),
            color="#00a86b",
            opacity=0.55,
            show_edges=True,
            smooth_shading=True,
        )
    plotter.add_text(
        _legend(candidates, validation, selected_index, attempts),
        position="upper_left",
        font_size=9,
        color="#202124",
    )
    focus_points = np.vstack((candidates.source_points, candidates.target_points))
    focus_min = np.min(focus_points, axis=0) - 0.025
    focus_max = np.max(focus_points, axis=0) + 0.025
    focus_bounds = (
        focus_min[0],
        focus_max[0],
        focus_min[1],
        focus_max[1],
        focus_min[2],
        focus_max[2],
    )
    plotter.view_isometric(bounds=focus_bounds)
    plotter.reset_camera(bounds=focus_bounds)
    path.parent.mkdir(parents=True, exist_ok=True)
    plotter.show(screenshot=str(path), auto_close=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=TRIALS_ROOT / "configs" / "default_experiment.yaml",
    )
    parser.add_argument("--workspace-root", type=Path, default=workspace_root_from_script())
    parser.add_argument("--candidate-count", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--cone-max-tilt-deg", type=float, default=None)
    parser.add_argument("--collision-threshold-mm", type=float, default=None)
    parser.add_argument("--disable-collision-check", action="store_true")
    parser.add_argument("--hide-scan", action="store_true")
    parser.add_argument("--screenshot", type=Path, default=None)
    args = parser.parse_args()

    if args.screenshot is not None:
        os.environ["PYVISTA_OFF_SCREEN"] = "true"

    config = load_experiment_config(args.config)
    workspace = args.workspace_root.expanduser().resolve()
    goal_vertices, goal_faces, scan_vertices, scan_faces = load_configured_geometry(
        config,
        workspace,
    )
    heat_vertex, width_norm_vertex = load_configured_goal_fields(
        config,
        workspace,
        expected_count=goal_vertices.shape[0],
    )
    completion = config["completion"]
    evaluation = evaluate_goal_with_vertical_rays(
        goal_vertices,
        goal_faces,
        scan_vertices,
        scan_faces,
        fill_tolerance_m=1e-3 * float(completion["fill_tolerance_mm"]),
    )
    contour_points, contour_lines = extract_phi_contour(
        vertices=goal_vertices,
        faces=goal_faces,
        scalar=evaluation.phi,
        iso=float(config["intersection"]["iso_level_m"]),
    )
    if contour_lines.shape[0] == 0:
        raise RuntimeError("The current scan/goal state has no phi=0 contour")

    contour_heat = sample_vertex_scalar_on_surface(
        contour_points,
        goal_vertices,
        goal_faces,
        heat_vertex,
    )
    width_config = config["scalar_fields"]
    width_vertex_mm = normalized_width_to_mm(
        width_norm_vertex,
        width_min_mm=float(width_config["width_min_mm"]),
        width_max_mm=float(width_config["width_max_mm"]),
    )
    contour_widths_m = 1e-3 * sample_vertex_scalar_on_surface(
        contour_points,
        goal_vertices,
        goal_faces,
        width_vertex_mm,
    )

    collision_config = config["collision"]
    collision_checker = None
    collision_threshold_mm = _optional_float(
        args.collision_threshold_mm,
        collision_config["threshold_mm"],
    )
    if not args.disable_collision_check:
        extruder_path = resolve_workspace_path(
            workspace,
            str(config["paths"]["extruder_mesh"]),
        )
        extruder_vertices, _ = load_collada_mesh(extruder_path)
        collision_checker = ExtruderCollisionChecker(
            scan_vertices=scan_vertices,
            scan_faces=scan_faces,
            extruder_vertices_tool0=extruder_vertices,
            threshold_mm=collision_threshold_mm,
            tcp_exclusion_radius_mm=float(collision_config["tcp_exclusion_radius_mm"]),
        )

    pool_config = config["candidate_pool"]
    action_config = config["action"]
    candidate_count = int(
        pool_config["count"] if args.candidate_count is None else args.candidate_count
    )
    seed = int(config["random_seed"] if args.seed is None else args.seed)
    cone_degrees = _optional_float(
        args.cone_max_tilt_deg,
        action_config["cone_max_tilt_deg"],
    )
    rng = np.random.default_rng(seed)
    max_attempts = int(pool_config["max_generation_attempts"])
    candidates: ActionCandidateBatch | None = None
    validation: CandidateValidation | None = None
    attempts = 0
    for attempts in range(1, max_attempts + 1):
        candidates = sample_action_candidates(
            contour_points=contour_points,
            contour_lines=contour_lines,
            contour_heat=contour_heat,
            contour_widths_m=contour_widths_m,
            count=candidate_count,
            rng=rng,
            height_min_m=1e-3 * float(action_config["bead_height_min_mm"]),
            height_max_m=1e-3 * float(action_config["bead_height_max_mm"]),
            cone_max_tilt_deg=cone_degrees,
            width_delta_min_m=1e-3 * float(pool_config["width_delta_min_mm"]),
            width_delta_max_m=1e-3 * float(pool_config["width_delta_max_mm"]),
            width_min_m=1e-3 * float(action_config["bead_width_min_mm"]),
            width_max_m=1e-3 * float(action_config["bead_width_max_mm"]),
        )
        validation = validate_action_candidates(
            candidates=candidates,
            scan_vertices=scan_vertices,
            scan_faces=scan_faces,
            contact_distance_min_m=1e-3
            * float(action_config["contact_distance_min_mm"]),
            contact_distance_max_m=1e-3
            * float(action_config["contact_distance_max_mm"]),
            contact_source_tolerance_m=1e-3
            * float(pool_config["contact_source_tolerance_mm"]),
            collision_checker=collision_checker,
        )
        if np.any(validation.valid):
            break
    assert candidates is not None and validation is not None

    valid_indices = np.flatnonzero(validation.valid)
    selected_index = (
        int(valid_indices[int(rng.integers(valid_indices.size))])
        if valid_indices.size
        else None
    )
    selected_bead_mesh = None
    if selected_index is not None:
        _, selected_bead_mesh = _selected_deposit_and_mesh(candidates, selected_index)

    summary: dict[str, object] = {
        "seed": seed,
        "selection_mode": "random_valid_smoke_test",
        "candidate_count": candidates.count,
        "generation_attempts": attempts,
        "valid_count": int(np.count_nonzero(validation.valid)),
        "collision_count": int(np.count_nonzero(validation.collides)),
        "contact_invalid_count": int(np.count_nonzero(~validation.contact_valid)),
        "selected_index": selected_index,
    }
    if selected_index is not None:
        summary["selected"] = {
            "source_m": candidates.source_points[selected_index].tolist(),
            "target_O_m": candidates.target_points[selected_index].tolist(),
            "Z": candidates.z_axes[selected_index].tolist(),
            "height_mm": float(1e3 * candidates.heights_m[selected_index]),
            "width_map_mm": float(1e3 * candidates.width_map_m[selected_index]),
            "width_mm": float(1e3 * candidates.widths_m[selected_index]),
            "heat": float(candidates.heat[selected_index]),
            "contact_error_mm": float(
                1e3 * validation.contact_source_errors_m[selected_index]
            ),
            "extruder_clearance_mm": float(
                validation.extruder_clearance_mm[selected_index]
            ),
        }
    print(json.dumps(summary, indent=2, sort_keys=True))

    scene = build_evaluation_scene(
        goal_vertices=goal_vertices,
        goal_faces=goal_faces,
        scan_vertices=scan_vertices,
        scan_faces=scan_faces,
        phi_contour_points=contour_points,
        phi_contour_lines=contour_lines,
        evaluation=evaluation,
    )
    if args.screenshot is not None:
        screenshot = args.screenshot.expanduser().resolve()
        _render_screenshot(
            path=screenshot,
            scene=scene,
            candidates=candidates,
            validation=validation,
            selected_index=selected_index,
            selected_bead_mesh=selected_bead_mesh,
            attempts=attempts,
            hide_scan=bool(args.hide_scan),
        )
        print(f"Saved action-candidate screenshot: {screenshot}")
        return

    viewer = Viewer(title="RL Trials — Action Candidate Pool")
    with viewer.batch():
        if not args.hide_scan:
            viewer.add_mesh(
                scene.scan_mesh,
                name="initial_scan",
                style=MeshStyle(
                    color="#95a5a6",
                    opacity=0.27,
                    show_edges=False,
                    smooth_shading=False,
                ),
            )
        viewer.add_mesh(
            scene.goal_mesh,
            name="classified_goal",
            style=MeshStyle(
                color=None,
                opacity=0.70,
                show_edges=True,
                smooth_shading=False,
            ),
        )
        if not scene.frontier_points.is_empty:
            viewer.add_point_cloud(
                scene.frontier_points,
                name="vertex_frontier",
                style=PointCloudStyle(
                    color=None,
                    size=3.0,
                    render_as_spheres=True,
                ),
            )
        if selected_bead_mesh is not None:
            viewer.add_mesh(
                selected_bead_mesh,
                name="selected_dds_bead",
                style=MeshStyle(
                    color="#00a86b",
                    opacity=0.55,
                    show_edges=True,
                    smooth_shading=True,
                ),
            )

    _add_candidate_overlays(
        viewer,
        scene,
        candidates,
        validation,
        selected_index,
    )
    viewer.plotter.add_text(
        _legend(candidates, validation, selected_index, attempts),
        position="upper_left",
        font_size=9,
        color="#202124",
        name="action_pool_legend",
    )
    viewer.apply_camera_preset("perspective")
    raise SystemExit(viewer.run())


if __name__ == "__main__":
    main()
