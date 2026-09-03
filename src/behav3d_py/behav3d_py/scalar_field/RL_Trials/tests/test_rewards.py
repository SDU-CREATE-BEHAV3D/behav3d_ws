from __future__ import annotations

import numpy as np
import pytest

from rl_trials.rewards import cantilever_ratio_cost, normalized_tilt_cost


def test_tilt_cost_prefers_vertical_and_is_one_at_limit() -> None:
    assert normalized_tilt_cost([0.0, 0.0, 1.0], max_tilt_deg=30.0) == pytest.approx(0.0)

    axis = [np.sin(np.deg2rad(30.0)), 0.0, np.cos(np.deg2rad(30.0))]
    assert normalized_tilt_cost(axis, max_tilt_deg=30.0) == pytest.approx(1.0)


def test_tilt_cost_is_quadratic() -> None:
    axis = [np.sin(np.deg2rad(15.0)), 0.0, np.cos(np.deg2rad(15.0))]
    assert normalized_tilt_cost(axis, max_tilt_deg=30.0) == pytest.approx(0.25)


def test_cantilever_cost_is_normalized_by_bead_width() -> None:
    # 10 mm lateral displacement with a 20 mm bead => ratio 0.5 => cost 0.25.
    cost = cantilever_ratio_cost(
        [0.0, 0.0, 0.0],
        [0.010, 0.0, 0.012],
        bead_width_m=0.020,
    )
    assert cost == pytest.approx(0.25)
