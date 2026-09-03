from __future__ import annotations

import numpy as np

from rl_trials.goal_evaluator import evaluate_goal_from_height_samples
from rl_trials.visualization import (
    FILLED_COLOR,
    FRONTIER_COLOR,
    OVERBUILT_COLOR,
    UNFILLED_COLOR,
    build_evaluation_scene,
)


def test_build_evaluation_scene_uses_dds_geometry_and_state_colors() -> None:
    goal_vertices = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    evaluation = evaluate_goal_from_height_samples(
        goal_vertices,
        faces,
        z_scan=np.array([1.010, 1.000, 0.980, 0.980]),
        has_hit=np.ones(4, dtype=bool),
        fill_tolerance_m=0.002,
        overbuild_tolerance_m=0.004,
    )
    scene = build_evaluation_scene(
        goal_vertices=goal_vertices,
        goal_faces=faces,
        scan_vertices=goal_vertices,
        scan_faces=faces,
        evaluation=evaluation,
    )

    colors = scene.goal_mesh.vertex_colors
    assert colors is not None
    assert np.array_equal(colors[0], OVERBUILT_COLOR)
    assert np.array_equal(colors[1], FILLED_COLOR)
    assert np.array_equal(colors[2], UNFILLED_COLOR)
    assert np.array_equal(colors[3], UNFILLED_COLOR)
    assert scene.frontier_points.n_points == 2
    assert scene.frontier_points.colors is not None
    assert np.all(scene.frontier_points.colors == FRONTIER_COLOR)
