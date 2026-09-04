#!/usr/bin/env python3
"""Decode, validate, and visualize one explicit continuous policy action."""

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

from dds import Domain, PointDeposit, simulate
from dds.viz import MeshStyle, Viewer
from lib_scalar.bead_profile import normalized_width_to_mm
from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.extruder_collision import ExtruderCollisionChecker, load_collada_mesh
from lib_scalar.geometry import sample_vertex_scalar_on_surface
from lib_scalar.viz_dds import add_colored_point_cloud, add_line_segments, attach_viewer

from rl_trials.action_decoder import DecodedAction, decode_continuous_action
from rl_trials.action_validation import ActionValidation, validate_decoded_action
from rl_trials.contour_parameterization import ContourParameterization
from rl_trials.dds_adapter import point_deposit_from_action
from rl_trials.fixture_io import (
    load_configured_geometry,
    load_configured_goal_fields,
    load_experiment_config,
    resolve_workspace_path,
)
from rl_trials.goal_evaluator import evaluate_goal_with_vertical_rays
from rl_trials.visualization import (
    EvaluationScene,
    add_evaluation_scene_to_plotter,
    add_evaluation_scene_to_viewer,
    build_evaluation_scene,
)

if TYPE_CHECKING:
    from dds.geometry import TriangleMesh


SOURCE_COLOR = np.array([0, 188, 212], dtype=np.uint8)
ACCEPTED_COLOR = np.array([0, 230, 118], dtype=np.uint8)
CONTACT_REJECTED_COLOR = np.array([52, 152, 219], dtype=np.uint8)
DOMAIN_REJECTED_COLOR = np.array([243, 156, 18], dtype=np.uint8)
COLLISION_REJECTED_COLOR = np.array([231, 76, 60], dtype=np.uint8)


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def _episode_domain(goal_vertices: np.ndarray, config: dict[str, object]) -> Domain:
    dds_config = config["dds"]
    assert isinstance(dds_config, dict)
    padding = 1e-3 * float(dds_config["padding_mm"])
    voxel_size = 1e-3 * float(dds_config["voxel_size_mm"])
    lower = np.min(goal_vertices, axis=0) - padding
    upper = np.max(goal_vertices, axis=0) + padding
    return Domain.from_bounds(
        xmin=float(lower[0]),
        xmax=float(upper[0]),
        ymin=float(lower[1]),
        ymax=float(upper[1]),
        zmin=float(lower[2]),
        zmax=float(upper[2]),
        voxel_size=voxel_size,
        length_unit="m",
    )


def _proposed_bead_mesh(deposit: PointDeposit) -> TriangleMesh:
    domain = Domain.from_deposits(
        deposit,
        voxel_size=0.001,
        padding=0.002,
        length_unit="m",
    )
    result = simulate(domain, deposit, threshold=0.5)
    return result.analysis.surface_mesh(threshold=0.5, step_size=1)


def _status_style(validation: ActionValidation) -> tuple[str, str, np.ndarray]:
    if validation.valid:
        return "accepted", "#00e676", ACCEPTED_COLOR
    if validation.collides:
        return "rejected: collision", "#e74c3c", COLLISION_REJECTED_COLOR
    if not validation.width_valid:
        return "rejected: width", "#f39c12", DOMAIN_REJECTED_COLOR
    if not validation.contact_valid:
        return "rejected: contact", "#3498db", CONTACT_REJECTED_COLOR
    return "rejected: DDS domain", "#f39c12", DOMAIN_REJECTED_COLOR


def _legend(action: DecodedAction, validation: ActionValidation) -> str:
    status, _, _ = _status_style(validation)
    reasons = (
        ", ".join(reason.value for reason in validation.rejection_reasons) or "none"
    )
    values = ", ".join(f"{value:.3f}" for value in action.normalized_action)
    return (
        "Direct deterministic policy action\n"
        "magenta: interpolated phi=0 contour\n"
        "cyan: decoded source S\n"
        "colored axis/target/bead: proposed action (not applied)\n\n"
        f"a=[{values}]\n"
        f"status={status}; reasons={reasons}\n"
        f"arc={100.0 * action.source_arc_fraction:.2f}%, "
        f"h={1e3 * action.height_m:.2f} mm, "
        f"w={1e3 * action.width_m:.2f} mm\n"
        f"tilt={action.tilt_deg:.2f} deg, heat={action.heat:.3f}"
    )


def _add_action_overlay(
    viewer,
    scene: EvaluationScene,
    action: DecodedAction,
    validation: ActionValidation,
) -> None:
    _, color, point_color = _status_style(validation)
    add_line_segments(
        viewer,
        scene.phi_contour_points,
        scene.phi_contour_lines,
        name="phi_zero_contour",
        color="#ff2d95",
        line_width=6.0,
        render_as_tubes=True,
    )
    action_points = np.vstack((action.source_point, action.target_point))
    add_line_segments(
        viewer,
        action_points,
        np.array([[0, 1]], dtype=np.int32),
        name="decoded_action_axis",
        color=color,
        line_width=7.0,
        render_as_tubes=True,
    )
    add_colored_point_cloud(
        viewer,
        action.source_point.reshape(1, 3),
        SOURCE_COLOR.reshape(1, 3),
        name="decoded_source",
        point_size=12.0,
        render_as_spheres=True,
    )
    add_colored_point_cloud(
        viewer,
        action.target_point.reshape(1, 3),
        point_color.reshape(1, 3),
        name="decoded_target",
        point_size=12.0,
        render_as_spheres=True,
    )


def _focus_bounds(action: DecodedAction) -> tuple[float, ...]:
    action_points = np.vstack((action.source_point, action.target_point))
    lower = np.min(action_points, axis=0) - 0.04
    upper = np.max(action_points, axis=0) + 0.04
    return (
        float(lower[0]),
        float(upper[0]),
        float(lower[1]),
        float(upper[1]),
        float(lower[2]),
        float(upper[2]),
    )


def _render_screenshot(
    *,
    path: Path,
    scene: EvaluationScene,
    action: DecodedAction,
    validation: ActionValidation,
    bead_mesh: TriangleMesh,
    hide_scan: bool,
) -> None:
    import pyvista as pv
    from dds.viz.converters import triangle_mesh_to_polydata

    plotter = pv.Plotter(off_screen=True, window_size=(1600, 1000))
    plotter.set_background("#f5f6f8")
    add_evaluation_scene_to_plotter(
        plotter,
        scene,
        pv,
        hide_scan=hide_scan,
        scan_opacity=0.27,
        goal_opacity=0.70,
        frontier_size=3.0,
    )
    overlay = attach_viewer(plotter)
    _add_action_overlay(overlay, scene, action, validation)
    _, bead_color, _ = _status_style(validation)
    plotter.add_mesh(
        triangle_mesh_to_polydata(bead_mesh, pv),
        color=bead_color,
        opacity=0.62,
        show_edges=True,
        smooth_shading=True,
    )
    plotter.add_text(
        _legend(action, validation),
        position="upper_left",
        font_size=9,
        color="#202124",
    )
    bounds = _focus_bounds(action)
    plotter.view_isometric(bounds=bounds)
    plotter.reset_camera(bounds=bounds)
    path.parent.mkdir(parents=True, exist_ok=True)
    plotter.show(screenshot=str(path), auto_close=True)


def _optional_finite(value: float) -> float | None:
    return float(value) if np.isfinite(value) else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=TRIALS_ROOT / "configs" / "default_experiment.yaml",
    )
    parser.add_argument(
        "--workspace-root", type=Path, default=workspace_root_from_script()
    )
    parser.add_argument(
        "--action",
        type=float,
        nargs=5,
        metavar=("SOURCE", "HEIGHT", "ORIENT_X", "ORIENT_Y", "WIDTH"),
        default=(0.0, 0.0, 0.0, 0.0, 0.0),
        help="Explicit normalized action in [-1, 1]^5 (default: all zeros).",
    )
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
    completion_config = config["completion"]
    evaluation = evaluate_goal_with_vertical_rays(
        goal_vertices,
        goal_faces,
        scan_vertices,
        scan_faces,
        fill_tolerance_m=1e-3 * float(completion_config["fill_tolerance_mm"]),
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
    scalar_config = config["scalar_fields"]
    width_vertex_mm = normalized_width_to_mm(
        width_norm_vertex,
        width_min_mm=float(scalar_config["width_min_mm"]),
        width_max_mm=float(scalar_config["width_max_mm"]),
    )
    contour_widths_m = 1e-3 * sample_vertex_scalar_on_surface(
        contour_points,
        goal_vertices,
        goal_faces,
        width_vertex_mm,
    )

    action_config = config["action"]
    contour = ContourParameterization.from_segments(contour_points, contour_lines)
    decoded = decode_continuous_action(
        normalized_action=np.asarray(args.action, dtype=np.float64),
        contour=contour,
        contour_heat=contour_heat,
        contour_widths_m=contour_widths_m,
        height_min_m=1e-3 * float(action_config["bead_height_min_mm"]),
        height_max_m=1e-3 * float(action_config["bead_height_max_mm"]),
        cone_max_tilt_deg=float(action_config["cone_max_tilt_deg"]),
        width_delta_min_m=1e-3 * float(action_config["width_delta_min_mm"]),
        width_delta_max_m=1e-3 * float(action_config["width_delta_max_mm"]),
        width_min_m=1e-3 * float(action_config["bead_width_min_mm"]),
        width_max_m=1e-3 * float(action_config["bead_width_max_mm"]),
    )

    collision_config = config["collision"]
    extruder_path = resolve_workspace_path(
        workspace, str(config["paths"]["extruder_mesh"])
    )
    extruder_vertices, _ = load_collada_mesh(extruder_path)
    collision_checker = ExtruderCollisionChecker(
        scan_vertices=scan_vertices,
        scan_faces=scan_faces,
        extruder_vertices_tool0=extruder_vertices,
        threshold_mm=float(collision_config["threshold_mm"]),
        tcp_exclusion_radius_mm=float(collision_config["tcp_exclusion_radius_mm"]),
    )
    validation = validate_decoded_action(
        action=decoded,
        scan_vertices=scan_vertices,
        scan_faces=scan_faces,
        contact_distance_min_m=1e-3 * float(action_config["contact_distance_min_mm"]),
        contact_distance_max_m=1e-3 * float(action_config["contact_distance_max_mm"]),
        contact_distance_epsilon_m=1e-3
        * float(action_config["contact_distance_numeric_epsilon_mm"]),
        domain=_episode_domain(goal_vertices, config),
        collision_checker=collision_checker,
    )
    deposit = point_deposit_from_action(decoded)
    bead_mesh = _proposed_bead_mesh(deposit)

    summary = {
        "normalized_action": decoded.normalized_action.tolist(),
        "contour_component_count": contour.component_count,
        "contour_total_length_mm": 1e3 * contour.total_length_m,
        "source_m": decoded.source_point.tolist(),
        "target_O_m": decoded.target_point.tolist(),
        "z_axis": decoded.z_axis.tolist(),
        "source_arc_fraction": decoded.source_arc_fraction,
        "source_arc_length_mm": 1e3 * decoded.source_arc_length_m,
        "contour_segment_index": decoded.contour_segment_index,
        "segment_fraction": decoded.segment_fraction,
        "height_mm": 1e3 * decoded.height_m,
        "width_map_mm": 1e3 * decoded.width_map_m,
        "width_delta_mm": 1e3 * decoded.width_delta_m,
        "width_mm": 1e3 * decoded.width_m,
        "width_valid": validation.width_valid,
        "tilt_deg": decoded.tilt_deg,
        "heat": decoded.heat,
        "valid": validation.valid,
        "rejection_reasons": [reason.value for reason in validation.rejection_reasons],
        "contact_distance_mm": 1e3 * _optional_finite(validation.contact_distance_m)
        if _optional_finite(validation.contact_distance_m) is not None
        else None,
        "contact_source_error_mm": 1e3
        * _optional_finite(validation.contact_source_error_m)
        if _optional_finite(validation.contact_source_error_m) is not None
        else None,
        "extruder_clearance_mm": _optional_finite(validation.extruder_clearance_mm),
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
            action=decoded,
            validation=validation,
            bead_mesh=bead_mesh,
            hide_scan=bool(args.hide_scan),
        )
        print(f"Saved direct-action screenshot: {screenshot}")
        return

    _, bead_color, _ = _status_style(validation)
    viewer = Viewer(title="RL Trials - Direct Policy Action")
    with viewer.batch():
        add_evaluation_scene_to_viewer(
            viewer,
            scene,
            hide_scan=bool(args.hide_scan),
            scan_opacity=0.27,
            goal_opacity=0.70,
            frontier_size=3.0,
        )
        viewer.add_mesh(
            bead_mesh,
            name="proposed_direct_action_bead",
            style=MeshStyle(
                color=bead_color,
                opacity=0.62,
                show_edges=True,
                smooth_shading=True,
            ),
        )
    _add_action_overlay(viewer, scene, decoded, validation)
    viewer.plotter.add_text(
        _legend(decoded, validation),
        position="upper_left",
        font_size=9,
        color="#202124",
        name="direct_action_legend",
    )
    bounds = _focus_bounds(decoded)
    viewer.plotter.view_isometric(bounds=bounds)
    viewer.plotter.reset_camera(bounds=bounds)
    raise SystemExit(viewer.run())


if __name__ == "__main__":
    main()
