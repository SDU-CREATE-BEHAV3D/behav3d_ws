from __future__ import annotations

import numpy as np
import pytest

from rl_trials.action_candidates import (
    sample_action_candidates,
    validate_action_candidates,
)


def _sample(count: int = 2000):
    # Two disconnected segments with a 1:9 length ratio.
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [9.0, 2.0, 0.0],
        ]
    )
    lines = np.array([[0, 1], [2, 3]], dtype=np.int32)
    return sample_action_candidates(
        contour_points=points,
        contour_lines=lines,
        contour_heat=np.array([0.0, 0.2, 0.8, 1.0]),
        contour_widths_m=np.array([0.020, 0.020, 0.030, 0.030]),
        count=count,
        rng=np.random.default_rng(7),
        height_min_m=0.010,
        height_max_m=0.016,
        cone_max_tilt_deg=30.0,
        width_delta_min_m=-0.004,
        width_delta_max_m=0.004,
        width_min_m=0.016,
        width_max_m=0.036,
    )


def test_candidates_are_length_weighted_and_inside_action_bounds() -> None:
    candidates = _sample()

    long_fraction = np.mean(candidates.contour_segment_indices == 1)
    assert long_fraction == pytest.approx(0.9, abs=0.025)
    assert np.all(candidates.heights_m >= 0.010)
    assert np.all(candidates.heights_m <= 0.016)
    assert np.all(candidates.widths_m >= 0.016)
    assert np.all(candidates.widths_m <= 0.036)
    np.testing.assert_allclose(
        candidates.target_points,
        candidates.source_points + candidates.heights_m[:, None] * candidates.z_axes,
    )
    tilts = np.rad2deg(np.arccos(np.clip(candidates.z_axes[:, 2], -1.0, 1.0)))
    assert np.all(tilts <= 30.0 + 1e-10)


class _CollisionByX:
    def check_pose(
        self,
        position: np.ndarray,
        z_axis: np.ndarray,
    ) -> tuple[bool, float, int]:
        del z_axis
        collides = bool(position[0] > 4.5)
        return collides, 0.5 if collides else 5.0, 3 if collides else 0


def test_contact_and_collision_are_separate_validity_checks() -> None:
    candidates = _sample(count=64)
    # A large plane at z=0 contains every sampled source point.
    scan_vertices = np.array(
        [
            [-20.0, -20.0, 0.0],
            [20.0, -20.0, 0.0],
            [20.0, 20.0, 0.0],
            [-20.0, 20.0, 0.0],
        ]
    )
    scan_faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

    result = validate_action_candidates(
        candidates=candidates,
        scan_vertices=scan_vertices,
        scan_faces=scan_faces,
        contact_distance_min_m=0.010,
        contact_distance_max_m=0.0161,
        contact_source_tolerance_m=1e-5,
        collision_checker=_CollisionByX(),
    )

    assert np.all(result.contact_valid)
    assert np.all(result.collision_checked)
    assert np.any(result.collides)
    assert np.any(result.valid)
    assert np.all(result.valid == ~result.collides)
    np.testing.assert_allclose(
        result.contact_hit_points,
        candidates.source_points,
        atol=1e-6,
    )
