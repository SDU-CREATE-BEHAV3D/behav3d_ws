from __future__ import annotations

import numpy as np
import pytest

from rl_trials.action_decoder import decode_continuous_action


POINTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [9.0, 2.0, 0.0],
    ]
)
LINES = np.array([[0, 1], [2, 3]], dtype=np.int32)
HEAT = np.array([0.0, 0.2, 0.8, 1.0])
WIDTH = np.array([0.020, 0.020, 0.030, 0.030])


def _decode(action):
    return decode_continuous_action(
        normalized_action=np.asarray(action, dtype=np.float64),
        contour_points=POINTS,
        contour_lines=LINES,
        contour_heat=HEAT,
        contour_widths_m=WIDTH,
        height_min_m=0.010,
        height_max_m=0.016,
        cone_max_tilt_deg=30.0,
        width_delta_min_m=-0.004,
        width_delta_max_m=0.004,
        width_min_m=0.016,
        width_max_m=0.036,
    )


def test_zero_action_is_vertical_and_deterministic() -> None:
    first = _decode([0.0, 0.0, 0.0, 0.0, 0.0])
    second = _decode([0.0, 0.0, 0.0, 0.0, 0.0])

    np.testing.assert_allclose(first.source_point, second.source_point)
    np.testing.assert_allclose(first.target_point, second.target_point)
    np.testing.assert_allclose(first.z_axis, [0.0, 0.0, 1.0], atol=1e-12)
    assert first.tilt_deg == pytest.approx(0.0)
    assert first.height_m == pytest.approx(0.013)
    assert first.width_delta_m == pytest.approx(0.0)


def test_source_coordinate_addresses_total_physical_arc_length() -> None:
    # Total contour length is 10 m. source_coord=-0.8 maps to 10% of it,
    # exactly the end of the first 1 m segment.
    action = _decode([-0.8, 0.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(action.source_point, [1.0, 0.0, 0.0])
    assert action.source_arc_fraction == pytest.approx(0.1)
    assert action.source_arc_length_m == pytest.approx(1.0)

    # source_coord=0 maps to 50% of total arc length: 4 m into the 9 m segment.
    action = _decode([0.0, 0.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(action.source_point, [4.0, 2.0, 0.0])


def test_policy_controls_orientation_directly_inside_cone() -> None:
    vertical = _decode([0.0, 0.0, 0.0, 0.0, 0.0])
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
    with pytest.raises(ValueError, match="\[-1, 1\]"):
        _decode([1.1, 0.0, 0.0, 0.0, 0.0])


def test_width_outside_physical_bounds_is_rejected_not_clipped() -> None:
    # The first contour has a 20 mm nominal width. -4 mm is valid at the lower bound.
    valid = _decode([-1.0, 0.0, 0.0, 0.0, -1.0])
    assert valid.width_m == pytest.approx(0.016)
