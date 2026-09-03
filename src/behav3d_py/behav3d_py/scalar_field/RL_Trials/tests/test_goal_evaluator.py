from __future__ import annotations

import numpy as np
import pytest

from rl_trials.goal_evaluator import (
    evaluate_goal_from_height_samples,
    evaluate_goal_with_vertical_rays,
    vertex_area_weights,
)


@pytest.fixture
def square_goal() -> tuple[np.ndarray, np.ndarray]:
    vertices = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    return vertices, faces


def test_vertex_area_weights_sum_to_surface_area(square_goal) -> None:
    vertices, faces = square_goal
    weights = vertex_area_weights(vertices, faces)

    assert float(np.sum(weights)) == pytest.approx(1.0)
    assert weights.tolist() == pytest.approx([1.0 / 3.0, 1.0 / 6.0, 1.0 / 3.0, 1.0 / 6.0])
    assert not weights.flags.writeable


def test_ray_misses_are_unresolved_and_never_filled(square_goal) -> None:
    vertices, faces = square_goal
    result = evaluate_goal_from_height_samples(
        vertices,
        faces,
        z_scan=np.full(4, np.nan),
        has_hit=np.zeros(4, dtype=bool),
    )

    assert np.all(result.unresolved)
    assert np.all(result.unfilled)
    assert not np.any(result.filled)
    assert not np.any(result.frontier)
    assert np.all(np.isinf(result.gap_to_completion))
    assert result.metrics.resolved_area_fraction == pytest.approx(0.0)
    assert result.metrics.completed_area_fraction == pytest.approx(0.0)


def test_partial_fill_produces_mesh_frontier(square_goal) -> None:
    vertices, faces = square_goal
    result = evaluate_goal_from_height_samples(
        vertices,
        faces,
        z_scan=np.array([1.0, 1.0, 0.98, 0.98]),
        has_hit=np.ones(4, dtype=bool),
        fill_tolerance_m=0.002,
        overbuild_tolerance_m=0.004,
    )

    assert result.filled.tolist() == [True, True, False, False]
    assert result.frontier.tolist() == [False, False, True, True]
    assert result.metrics.filled_vertex_count == 2
    assert result.metrics.completed_area_fraction == pytest.approx(0.5)
    assert result.metrics.max_remaining_gap_m == pytest.approx(0.018)


def test_completed_goal_inside_positive_tolerance(square_goal) -> None:
    vertices, faces = square_goal
    result = evaluate_goal_from_height_samples(
        vertices,
        faces,
        z_scan=np.full(4, 0.999),
        has_hit=np.ones(4, dtype=bool),
        fill_tolerance_m=0.002,
    )

    assert np.all(result.filled)
    assert not np.any(result.unfilled)
    assert not np.any(result.frontier)
    assert result.metrics.completed_area_fraction == pytest.approx(1.0)
    assert result.metrics.mean_remaining_gap_m == pytest.approx(0.0)


def test_overbuilt_vertices_are_also_filled(square_goal) -> None:
    vertices, faces = square_goal
    result = evaluate_goal_from_height_samples(
        vertices,
        faces,
        z_scan=np.full(4, 1.010),
        has_hit=np.ones(4, dtype=bool),
        fill_tolerance_m=0.002,
        overbuild_tolerance_m=0.004,
    )

    assert np.all(result.filled)
    assert np.all(result.overbuilt)
    assert result.metrics.completed_area_fraction == pytest.approx(1.0)
    assert result.metrics.overbuilt_area_fraction == pytest.approx(1.0)
    assert result.metrics.mean_overbuild_beyond_tolerance_m == pytest.approx(0.006)


def test_vertical_raycast_backend_hits_planar_scan(square_goal) -> None:
    goal_vertices, goal_faces = square_goal
    scan_vertices = goal_vertices.copy()
    scan_vertices[:, 2] = 0.99
    scan_faces = goal_faces.copy()

    result = evaluate_goal_with_vertical_rays(
        goal_vertices,
        goal_faces,
        scan_vertices,
        scan_faces,
        fill_tolerance_m=0.002,
    )

    assert np.all(result.has_hit)
    assert result.phi.tolist() == pytest.approx([0.01] * 4, abs=1e-4)
    assert not np.any(result.filled)
