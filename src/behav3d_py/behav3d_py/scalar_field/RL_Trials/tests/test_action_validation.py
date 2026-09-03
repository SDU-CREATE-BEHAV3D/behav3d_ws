from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from dds import Domain

from rl_trials.action_decoder import DecodedAction, decode_continuous_action
from rl_trials.action_validation import (
    ActionRejectionReason,
    validate_decoded_action,
)
from rl_trials.contour_parameterization import ContourParameterization
from rl_trials.dds_adapter import point_deposit_from_action

POINTS = np.array([[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
LINES = np.array([[0, 1]], dtype=np.int32)
SCAN_VERTICES = np.array(
    [
        [-2.0, -2.0, 0.0],
        [2.0, -2.0, 0.0],
        [2.0, 2.0, 0.0],
        [-2.0, 2.0, 0.0],
    ]
)
SCAN_FACES = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)


class _CollisionChecker:
    def __init__(self, collides: bool) -> None:
        self.collides = collides
        self.calls = 0

    def check_pose(
        self,
        position: np.ndarray,
        z_axis: np.ndarray,
    ) -> tuple[bool, float, int]:
        self.calls += 1
        return self.collides, 0.5 if self.collides else 10.0, int(self.collides)


def _decode() -> DecodedAction:
    contour = ContourParameterization.from_segments(POINTS, LINES)
    return decode_continuous_action(
        normalized_action=np.zeros(5),
        contour=contour,
        contour_heat=np.array([0.25, 0.75]),
        contour_widths_m=np.array([0.020, 0.020]),
        height_min_m=0.010,
        height_max_m=0.016,
        cone_max_tilt_deg=30.0,
        width_delta_min_m=-0.004,
        width_delta_max_m=0.004,
        width_min_m=0.016,
        width_max_m=0.036,
    )


def _domain(*, zmax: float = 1.0) -> Domain:
    return Domain.from_bounds(
        xmin=-2.0,
        xmax=2.0,
        ymin=-2.0,
        ymax=2.0,
        zmin=-1.0,
        zmax=zmax,
        voxel_size=0.001,
        length_unit="m",
    )


def _validate(
    action: DecodedAction,
    *,
    domain: Domain | None = None,
    collision_checker: _CollisionChecker | None = None,
):
    return validate_decoded_action(
        action=action,
        scan_vertices=SCAN_VERTICES,
        scan_faces=SCAN_FACES,
        contact_distance_min_m=0.010,
        contact_distance_max_m=0.016,
        contact_source_tolerance_m=1e-5,
        domain=_domain() if domain is None else domain,
        collision_checker=collision_checker,
    )


def test_decoded_action_converts_to_top_referenced_dds_deposit() -> None:
    action = _decode()
    deposit = point_deposit_from_action(action)

    np.testing.assert_allclose(deposit.target.position.to_array(), action.target_point)
    np.testing.assert_allclose(deposit.target.normal.to_array(), action.z_axis)
    assert deposit.profile.height == pytest.approx(action.height_m)
    assert deposit.profile.width == pytest.approx(action.width_m)


def test_valid_action_keeps_contact_domain_and_collision_results_separate() -> None:
    checker = _CollisionChecker(collides=False)
    result = _validate(_decode(), collision_checker=checker)

    assert result.valid
    assert result.rejection_reasons == ()
    assert result.contact_valid
    assert result.domain_valid
    assert result.collision_checked
    assert not result.collides
    assert checker.calls == 1
    np.testing.assert_allclose(result.contact_hit_point, [0.0, 0.0, 0.0], atol=1e-7)
    assert result.contact_distance_m == pytest.approx(0.013, abs=1e-6)


def test_collision_rejects_action_without_replacing_it() -> None:
    checker = _CollisionChecker(collides=True)
    result = _validate(_decode(), collision_checker=checker)

    assert not result.valid
    assert result.rejection_reasons == (ActionRejectionReason.COLLISION,)
    assert result.collides
    assert checker.calls == 1


def test_missing_contact_rejects_action_before_collision_check() -> None:
    checker = _CollisionChecker(collides=False)
    result = validate_decoded_action(
        action=_decode(),
        scan_vertices=SCAN_VERTICES + np.array([10.0, 0.0, 0.0]),
        scan_faces=SCAN_FACES,
        contact_distance_min_m=0.010,
        contact_distance_max_m=0.016,
        contact_source_tolerance_m=1e-5,
        domain=_domain(),
        collision_checker=checker,
    )

    assert not result.valid
    assert result.rejection_reasons == (ActionRejectionReason.NO_CONTACT,)
    assert not result.contact_valid
    assert not result.collision_checked
    assert checker.calls == 0


def test_contact_distance_outside_height_bounds_has_specific_reason() -> None:
    result = validate_decoded_action(
        action=_decode(),
        scan_vertices=SCAN_VERTICES,
        scan_faces=SCAN_FACES,
        contact_distance_min_m=0.014,
        contact_distance_max_m=0.016,
        contact_source_tolerance_m=1e-5,
        domain=_domain(),
        collision_checker=None,
    )

    assert not result.valid
    assert result.rejection_reasons == (ActionRejectionReason.CONTACT_DISTANCE,)


def test_contact_hit_must_return_to_decoded_source() -> None:
    action = _decode()
    inconsistent = replace(
        action,
        source_point=action.source_point + np.array([0.01, 0.0, 0.0]),
    )
    result = _validate(inconsistent)

    assert not result.valid
    assert result.rejection_reasons == (ActionRejectionReason.CONTACT_SOURCE_MISMATCH,)


def test_out_of_domain_action_skips_collision_check() -> None:
    checker = _CollisionChecker(collides=False)
    result = _validate(_decode(), domain=_domain(zmax=0.005), collision_checker=checker)

    assert not result.valid
    assert result.rejection_reasons == (ActionRejectionReason.OUT_OF_DOMAIN,)
    assert not result.domain_valid
    assert not result.collision_checked
    assert checker.calls == 0
