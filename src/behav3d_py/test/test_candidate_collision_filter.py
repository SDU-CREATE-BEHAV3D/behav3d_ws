import numpy as np
import pytest

from behav3d_py.scalar_field.lib_scalar.extruder_collision import (
    ExtruderCollisionChecker,
)
from behav3d_py.scalar_field.lib_scalar.generate_print_points import (
    generate_print_points,
)


def _straight_polyline(count: int = 5):
    points = np.column_stack(
        (
            0.010 * np.arange(count, dtype=np.float64),
            np.zeros(count),
            np.zeros(count),
        )
    )
    lines = np.column_stack(
        (np.arange(count - 1), np.arange(1, count))
    ).astype(np.int32)
    return points, lines


def test_rejected_candidate_does_not_consume_requested_count():
    points, lines = _straight_polyline()
    checked: list[int] = []

    def validator(source, endpoint, segment_start, polyline_index):
        checked.append(int(polyline_index))
        np.testing.assert_allclose(endpoint, source + np.array([0.0, 0.0, 0.005]))
        np.testing.assert_allclose(segment_start, source)
        return int(polyline_index) != 0

    selected = generate_print_points(
        polyline_points=points,
        polyline_lines=lines,
        field_vertices_world=points,
        field_scalar=np.arange(points.shape[0], dtype=np.float64),
        count=2,
        min_spacing=0.0,
        candidate_mode="z_lift",
        lift_height=0.005,
        candidate_validator=validator,
        validator_rejection_radius=0.0,
    )

    assert checked == [0, 1, 2]
    assert selected.polyline_indices.tolist() == [1, 2]
    assert selected.rejected_by_validator == 1


def test_rejected_candidate_removes_geodesic_polyline_neighborhood():
    points, lines = _straight_polyline()

    selected = generate_print_points(
        polyline_points=points,
        polyline_lines=lines,
        field_vertices_world=points,
        field_scalar=np.arange(points.shape[0], dtype=np.float64),
        count=1,
        min_spacing=0.0,
        candidate_validator=lambda _source, _end, _start, index: int(index) != 0,
        validator_rejection_radius=0.011,
    )

    assert selected.polyline_indices.tolist() == [2]
    assert selected.rejected_by_validator == 1


def test_extruder_checker_detects_near_scan_and_accepts_clear_pose():
    scan_vertices = np.array(
        [[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    scan_faces = np.array([[0, 1, 2]], dtype=np.int32)
    checker = ExtruderCollisionChecker(
        scan_vertices=scan_vertices,
        scan_faces=scan_faces,
        extruder_vertices_tool0=np.array([[0.0, 0.0, 0.0]], dtype=np.float64),
        threshold_mm=1.0,
        tool0_tcp_xyz=(0.0, 0.0, 0.0),
        tool0_tcp_rpy=(0.0, 0.0, 0.0),
    )

    colliding = checker.check_segment(
        start=np.array([0.0, 0.0, 0.0005]),
        end=np.array([0.0, 0.0, 0.0005]),
        start_z=np.array([0.0, 0.0, 1.0]),
        end_z=np.array([0.0, 0.0, 1.0]),
        samples=1,
    )
    clear = checker.check_segment(
        start=np.array([0.0, 0.0, 0.010]),
        end=np.array([0.0, 0.0, 0.010]),
        start_z=np.array([0.0, 0.0, 1.0]),
        end_z=np.array([0.0, 0.0, 1.0]),
        samples=1,
    )

    assert colliding.collides is True
    assert colliding.min_distance_mm == pytest.approx(0.5, abs=1e-3)
    assert clear.collides is False
    assert clear.min_distance_mm == pytest.approx(10.0, abs=1e-3)
