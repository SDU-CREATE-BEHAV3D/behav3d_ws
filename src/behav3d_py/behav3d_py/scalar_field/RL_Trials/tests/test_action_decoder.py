from __future__ import annotations

import numpy as np
import pytest

from rl_trials.action_decoder import decode_continuous_action
from rl_trials.contour_parameterization import ContourParameterization

POINTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [0.0, 0.0, 8.0],
        [2.0, 0.0, 8.0],
    ]
)
# Components and endpoint directions are intentionally not in canonical order.
LINES = np.array([[3, 2], [1, 0]], dtype=np.int32)
HEAT = np.array([0.0, 0.2, 0.8, 1.0])
WIDTH = np.array([0.020, 0.020, 0.030, 0.030])


def _decode(action, *, points=POINTS, lines=LINES, heat=HEAT, width=WIDTH):
    contour = ContourParameterization.from_segments(points, lines)
    return decode_continuous_action(
        normalized_action=np.asarray(action, dtype=np.float64),
        contour=contour,
        contour_heat=heat,
        contour_widths_m=width,
        height_min_m=0.010,
        height_max_m=0.016,
        cone_max_tilt_deg=30.0,
        width_delta_min_m=-0.004,
        width_delta_max_m=0.004,
        width_min_m=0.016,
        width_max_m=0.036,
    )


def test_zero_action_is_vertical_and_deterministic() -> None:
    first = _decode(np.zeros(5))
    second = _decode(np.zeros(5))

    # The exact 50% boundary enters the second canonical component.
    np.testing.assert_allclose(first.source_point, [0.0, 0.0, 8.0])
    np.testing.assert_allclose(first.source_point, second.source_point)
    np.testing.assert_allclose(first.target_point, second.target_point)
    np.testing.assert_allclose(first.z_axis, [0.0, 0.0, 1.0], atol=1e-12)
    assert first.source_arc_fraction == pytest.approx(0.5)
    assert first.source_arc_length_m == pytest.approx(2.0)
    assert first.tilt_deg == pytest.approx(0.0)
    assert first.height_m == pytest.approx(0.013)
    assert first.width_delta_m == pytest.approx(0.0)


def test_source_coordinate_addresses_disconnected_3d_components() -> None:
    action = _decode([-0.5, 0.0, 0.0, 0.0, 0.0])

    np.testing.assert_allclose(action.source_point, [1.0, 0.0, 0.0])
    assert action.source_arc_fraction == pytest.approx(0.25)
    assert action.source_arc_length_m == pytest.approx(1.0)
    assert action.heat == pytest.approx(0.1)
    assert action.width_map_m == pytest.approx(0.020)


def test_source_extremes_address_first_and_last_canonical_endpoints() -> None:
    first = _decode([-1.0, 0.0, 0.0, 0.0, 0.0])
    last = _decode([1.0, 0.0, 0.0, 0.0, 0.0])

    np.testing.assert_allclose(first.source_point, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(last.source_point, [2.0, 0.0, 8.0])


def test_connected_segment_soup_is_traced_as_one_continuous_curve() -> None:
    points = np.array(
        [
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]
    )
    lines = np.array([[0, 1], [2, 3], [4, 5]], dtype=np.int32)
    action = _decode(
        np.zeros(5),
        points=points,
        lines=lines,
        heat=np.linspace(0.0, 1.0, points.shape[0]),
        width=np.full(points.shape[0], 0.020),
    )

    np.testing.assert_allclose(action.source_point, [1.5, 0.0, 0.0])
    assert action.source_arc_length_m == pytest.approx(1.5)


def test_closed_loop_gets_a_canonical_start_and_direction() -> None:
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
    )
    action = _decode(
        np.zeros(5),
        points=points,
        lines=np.array([[2, 1], [0, 1], [3, 0], [2, 3]], dtype=np.int32),
        heat=np.zeros(4),
        width=np.full(4, 0.020),
    )

    np.testing.assert_allclose(action.source_point, [1.0, 1.0, 0.0])
    assert action.source_arc_length_m == pytest.approx(2.0)


def test_parameterization_is_independent_of_line_order_and_direction() -> None:
    action = [0.25, 0.0, 0.0, 0.0, 0.0]
    scrambled = _decode(action)
    reordered = _decode(
        action,
        lines=np.array([[0, 1], [2, 3]], dtype=np.int32),
    )

    np.testing.assert_allclose(scrambled.source_point, [0.5, 0.0, 8.0])
    np.testing.assert_allclose(reordered.source_point, scrambled.source_point)
    assert reordered.heat == pytest.approx(scrambled.heat)
    assert reordered.width_map_m == pytest.approx(scrambled.width_map_m)


def test_policy_controls_orientation_directly_inside_cone() -> None:
    vertical = _decode(np.zeros(5))
    tilted = _decode([0.0, 0.0, 1.0, 0.0, 0.0])

    assert vertical.tilt_deg == pytest.approx(0.0)
    assert tilted.tilt_deg == pytest.approx(30.0)
    assert tilted.z_axis[0] > 0.0
    assert tilted.z_axis[2] == pytest.approx(np.cos(np.deg2rad(30.0)))
    np.testing.assert_allclose(
        tilted.target_point,
        tilted.source_point + tilted.height_m * tilted.z_axis,
    )


def test_out_of_range_policy_action_is_rejected_not_clipped() -> None:
    with pytest.raises(ValueError, match=r"\[-1, 1\]"):
        _decode([1.1, 0.0, 0.0, 0.0, 0.0])


def test_seven_dimensional_query_action_is_rejected() -> None:
    with pytest.raises(ValueError, match=r"shape \(5,\)"):
        _decode(np.zeros(7))


def test_empty_contour_is_rejected() -> None:
    with pytest.raises(ValueError, match="empty phi contour"):
        _decode(np.zeros(5), lines=np.empty((0, 2), dtype=np.int32))


def test_degenerate_segments_are_ignored() -> None:
    action = _decode(
        np.zeros(5),
        lines=np.array([[0, 0], [3, 2]], dtype=np.int32),
    )

    np.testing.assert_allclose(action.source_point, [1.0, 0.0, 8.0])
    assert action.contour_segment_index == 1


def test_all_degenerate_segments_are_rejected() -> None:
    with pytest.raises(ValueError, match="no non-degenerate segments"):
        _decode(np.zeros(5), lines=np.array([[0, 0]], dtype=np.int32))


def test_branched_contour_is_rejected_explicitly() -> None:
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    with pytest.raises(ValueError, match="contains a branch"):
        _decode(
            np.zeros(5),
            points=points,
            lines=np.array([[0, 1], [0, 2], [0, 3]], dtype=np.int32),
            heat=np.zeros(4),
            width=np.full(4, 0.020),
        )


def test_width_lower_bound_is_preserved_without_clipping() -> None:
    valid = _decode([-0.5, 0.0, 0.0, 0.0, -1.0])
    assert valid.width_m == pytest.approx(0.016)
    assert valid.width_valid


def test_width_control_is_mapped_into_local_feasible_interval() -> None:
    action = _decode(
        [-1.0, 0.0, 0.0, 0.0, -1.0],
        width=np.full(POINTS.shape[0], 0.016),
    )

    assert action.width_m == pytest.approx(0.016)
    assert action.width_delta_m == pytest.approx(0.0)
    assert action.width_valid

    upper = _decode(
        [-1.0, 0.0, 0.0, 0.0, 1.0],
        width=np.full(POINTS.shape[0], 0.036),
    )
    assert upper.width_m == pytest.approx(0.036)
    assert upper.width_delta_m == pytest.approx(0.0)
    assert upper.width_valid
