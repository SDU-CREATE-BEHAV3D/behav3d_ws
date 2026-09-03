from __future__ import annotations

import numpy as np
import pytest

from rl_trials.contour_observation import sample_contour_observation
from rl_trials.contour_parameterization import ContourParameterization

POINTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [0.0, 0.0, 8.0],
        [2.0, 0.0, 8.0],
    ]
)
LINES = np.array([[3, 2], [1, 0]], dtype=np.int32)


def test_parameterization_reports_3d_components_and_total_length() -> None:
    contour = ContourParameterization.from_segments(POINTS, LINES)

    assert contour.component_count == 2
    assert contour.total_length_m == pytest.approx(4.0)
    np.testing.assert_allclose(contour.component_lengths_m, [2.0, 2.0])
    assert not contour.points.flags.writeable
    assert not contour.lines.flags.writeable


def test_component_aware_sampling_covers_every_component() -> None:
    contour = ContourParameterization.from_segments(POINTS, LINES)
    samples = contour.sample(6)

    assert samples.count == 6
    np.testing.assert_array_equal(np.bincount(samples.component_indices), [3, 3])
    assert np.all(samples.valid_mask)
    assert np.all(np.diff(samples.source_coord) > 0.0)


def test_every_sample_round_trips_through_the_policy_source_coordinate() -> None:
    contour = ContourParameterization.from_segments(POINTS, LINES)
    samples = contour.sample(7)

    reconstructed = np.asarray(
        [contour.locate(source).point for source in samples.source_coord]
    )
    np.testing.assert_allclose(reconstructed, samples.points)


def test_sampling_refuses_to_silently_drop_a_disconnected_component() -> None:
    contour = ContourParameterization.from_segments(POINTS, LINES)

    with pytest.raises(ValueError, match="component count"):
        contour.sample(1)


def test_observation_fields_share_the_decoder_parameterization() -> None:
    contour = ContourParameterization.from_segments(POINTS, LINES)
    observation = sample_contour_observation(
        contour=contour,
        contour_heat=np.array([0.0, 0.2, 0.8, 1.0]),
        contour_widths_m=np.array([0.020, 0.020, 0.030, 0.030]),
        sample_count=4,
    )

    np.testing.assert_allclose(observation.heat, [0.05, 0.15, 0.85, 0.95])
    np.testing.assert_allclose(observation.widths_m, [0.020, 0.020, 0.030, 0.030])
    np.testing.assert_array_equal(observation.samples.component_indices, [0, 0, 1, 1])
    assert not observation.heat.flags.writeable
    assert not observation.widths_m.flags.writeable


def test_scalar_and_vector_fields_can_be_interpolated_at_samples() -> None:
    contour = ContourParameterization.from_segments(POINTS, LINES)
    samples = contour.sample(4)
    vectors = np.column_stack((np.arange(4), np.arange(4) * 10.0))

    interpolated = contour.interpolate(vectors, samples)

    assert interpolated.shape == (4, 2)
    np.testing.assert_allclose(interpolated[:, 1], 10.0 * interpolated[:, 0])
