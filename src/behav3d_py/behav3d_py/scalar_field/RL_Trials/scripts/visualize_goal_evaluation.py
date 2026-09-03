#!/usr/bin/env python3
"""Visualize scan, goal completion states, and frontier with DDS Viewer."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import TYPE_CHECKING

# Allow this file to be launched directly, without installing rl_trials or
# manually setting PYTHONPATH.
TRIALS_ROOT = Path(__file__).resolve().parents[1]
if str(TRIALS_ROOT) not in sys.path:
    sys.path.insert(0, str(TRIALS_ROOT))
SCALAR_FIELD_ROOT = TRIALS_ROOT.parent
if str(SCALAR_FIELD_ROOT) not in sys.path:
    sys.path.insert(0, str(SCALAR_FIELD_ROOT))

from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.viz_dds import add_line_segments, attach_viewer

from rl_trials.fixture_io import load_configured_geometry, load_experiment_config
from rl_trials.goal_evaluator import evaluate_goal_with_vertical_rays
from rl_trials.visualization import (
    add_evaluation_scene_to_plotter,
    add_evaluation_scene_to_viewer,
    build_evaluation_scene,
)

if TYPE_CHECKING:
    from rl_trials.goal_evaluator import GoalMetrics
    from rl_trials.visualization import EvaluationScene


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def legend_text(metrics: GoalMetrics) -> str:
    return (
        "Goal states\n"
        "orange: unfilled\n"
        "green: filled\n"
        "dark: unresolved\n"
        "cyan points: vertex frontier (auxiliary)\n"
        "magenta line: interpolated phi=0 intersection\n\n"
        f"completed area: {100.0 * metrics.completed_area_fraction:.3f}%"
    )


def save_screenshot(
    scene: EvaluationScene,
    metrics: GoalMetrics,
    path: Path,
    *,
    hide_scan: bool,
) -> None:
    """Render DDS geometry off-screen through the DDS PyVista converters."""

    import pyvista as pv

    plotter = pv.Plotter(off_screen=True, window_size=(1400, 900))
    plotter.set_background("#f5f6f8")
    add_evaluation_scene_to_plotter(plotter, scene, pv, hide_scan=hide_scan)
    if scene.phi_contour_lines.shape[0] > 0:
        overlay = attach_viewer(plotter)
        add_line_segments(
            overlay,
            scene.phi_contour_points,
            scene.phi_contour_lines,
            name="phi_zero_contour",
            color="#ff2d95",
            line_width=7.0,
            render_as_tubes=True,
        )
    plotter.add_text(
        legend_text(metrics),
        position="upper_left",
        font_size=10,
        color="#202124",
    )
    plotter.view_isometric()
    plotter.reset_camera()
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
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="Render off-screen to this image instead of opening an interactive window.",
    )
    parser.add_argument("--hide-scan", action="store_true")
    args = parser.parse_args()

    if args.screenshot is not None:
        os.environ["PYVISTA_OFF_SCREEN"] = "true"

    config = load_experiment_config(args.config)
    goal_vertices, goal_faces, scan_vertices, scan_faces = load_configured_geometry(
        config,
        args.workspace_root.resolve(),
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
        iso=0.0,
    )
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
        save_screenshot(
            scene,
            evaluation.metrics,
            screenshot,
            hide_scan=bool(args.hide_scan),
        )
        print(f"Saved DDS goal-evaluation screenshot: {screenshot}")
        return

    from dds.viz import Viewer

    viewer = Viewer(
        title="RL Trials — DDS Goal Evaluation",
        off_screen=args.screenshot is not None,
    )
    with viewer.batch():
        add_evaluation_scene_to_viewer(
            viewer,
            scene,
            hide_scan=bool(args.hide_scan),
        )

    if scene.phi_contour_lines.shape[0] > 0:
        add_line_segments(
            viewer,
            scene.phi_contour_points,
            scene.phi_contour_lines,
            name="phi_zero_contour",
            color="#ff2d95",
            line_width=7.0,
            render_as_tubes=True,
        )

    metrics = evaluation.metrics
    viewer.plotter.add_text(
        legend_text(metrics),
        position="upper_left",
        font_size=10,
        color="#202124",
        name="goal_state_legend",
    )
    viewer.apply_camera_preset("perspective")
    raise SystemExit(viewer.run())


if __name__ == "__main__":
    main()
