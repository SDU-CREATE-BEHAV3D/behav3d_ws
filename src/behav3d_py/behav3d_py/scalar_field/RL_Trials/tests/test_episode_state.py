from __future__ import annotations

from dataclasses import replace

import numpy as np
from dds import Domain

from rl_trials.action_decoder import DecodedAction, decode_continuous_action
from rl_trials.action_validation import ActionRejectionReason, ActionValidation
from rl_trials.contour_parameterization import ContourParameterization
from rl_trials.dds_adapter import point_deposit_from_action
from rl_trials.episode_state import DDSEpisodeState

SCAN_VERTICES = np.array(
    [
        [-0.03, -0.03, 0.0],
        [0.03, -0.03, 0.0],
        [0.03, 0.03, 0.0],
        [-0.03, 0.03, 0.0],
    ]
)
SCAN_FACES = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)


def _domain() -> Domain:
    return Domain.from_bounds(
        xmin=-0.05,
        xmax=0.05,
        ymin=-0.05,
        ymax=0.05,
        zmin=-0.02,
        zmax=0.05,
        voxel_size=0.002,
        length_unit="m",
    )


def _state() -> DDSEpisodeState:
    return DDSEpisodeState(
        domain=_domain(),
        initial_scan_vertices=SCAN_VERTICES,
        initial_scan_faces=SCAN_FACES,
        occupancy_threshold=0.5,
    )


def _action() -> DecodedAction:
    contour = ContourParameterization.from_segments(
        np.array([[-0.01, 0.0, 0.0], [0.01, 0.0, 0.0]]),
        np.array([[0, 1]], dtype=np.int32),
    )
    return decode_continuous_action(
        normalized_action=np.zeros(5),
        contour=contour,
        contour_heat=np.array([0.0, 1.0]),
        contour_widths_m=np.array([0.012, 0.012]),
        height_min_m=0.010,
        height_max_m=0.010,
        cone_max_tilt_deg=30.0,
        width_delta_min_m=-0.002,
        width_delta_max_m=0.002,
        width_min_m=0.010,
        width_max_m=0.014,
    )


def _valid_validation(action: DecodedAction) -> ActionValidation:
    deposit = point_deposit_from_action(action)
    support_min, support_max = deposit.support_bounds()
    return ActionValidation(
        width_valid=action.width_valid,
        contact_hit_point=action.source_point.copy(),
        contact_distance_m=action.height_m,
        contact_source_error_m=0.0,
        contact_valid=True,
        support_min=support_min.to_array(),
        support_max=support_max.to_array(),
        domain_valid=True,
        collision_checked=False,
        collides=False,
        extruder_clearance_mm=float("nan"),
        extruder_hit_vertices=0,
        valid=True,
        rejection_reasons=(),
    )


def test_rejected_action_does_not_mutate_dds_geometry() -> None:
    state = _state()
    action = _action()
    valid = _valid_validation(action)
    rejected = replace(
        valid,
        contact_valid=False,
        valid=False,
        rejection_reasons=(ActionRejectionReason.NO_CONTACT,),
    )
    before = state.result()

    transition = state.apply_validated_action(action, rejected)
    after = state.result()

    assert not transition.accepted
    assert transition.deposit_index is None
    assert state.attempted_steps == 1
    assert state.accepted_deposits == 0
    assert state.rejected_steps == 1
    assert state.geometry_revision == 0
    assert state.deposits == ()
    assert after is before
    np.testing.assert_array_equal(after.implicit_field, before.implicit_field)
    np.testing.assert_array_equal(after.coverage, before.coverage)


def test_accepted_action_updates_incremental_dds_fields_once() -> None:
    state = _state()
    action = _action()
    before = state.result()

    transition = state.apply_validated_action(action, _valid_validation(action))
    after = state.result()

    assert transition.accepted
    assert transition.deposit_index == 0
    assert transition.geometry_revision == 1
    assert state.attempted_steps == 1
    assert state.accepted_deposits == 1
    assert state.rejected_steps == 0
    assert len(state.deposits) == 1
    assert after is not before
    assert float(np.sum(after.implicit_field)) > 0.0
    assert after.coverage is not None
    assert float(np.sum(after.coverage)) > 0.0
    # Earlier DDS results are immutable snapshots.
    assert float(np.sum(before.implicit_field)) == 0.0


def test_reset_restores_initial_episode_and_reuses_state_object() -> None:
    state = _state()
    action = _action()
    state.apply_validated_action(action, _valid_validation(action))

    state.reset()
    result = state.result()

    assert state.attempted_steps == 0
    assert state.accepted_deposits == 0
    assert state.rejected_steps == 0
    assert state.geometry_revision == 0
    assert state.deposits == ()
    assert float(np.sum(result.implicit_field)) == 0.0
    assert result.coverage is not None
    assert float(np.sum(result.coverage)) == 0.0


def test_current_material_mesh_combines_initial_scan_and_dds_surface() -> None:
    state = _state()
    initial_vertices, initial_faces = state.current_material_mesh()
    action = _action()
    state.apply_validated_action(action, _valid_validation(action))

    vertices, faces = state.current_material_mesh()

    np.testing.assert_allclose(initial_vertices, SCAN_VERTICES)
    np.testing.assert_array_equal(initial_faces, SCAN_FACES)
    np.testing.assert_allclose(vertices[: SCAN_VERTICES.shape[0]], SCAN_VERTICES)
    np.testing.assert_array_equal(faces[: SCAN_FACES.shape[0]], SCAN_FACES)
    assert vertices.shape[0] > initial_vertices.shape[0]
    assert faces.shape[0] > initial_faces.shape[0]


def test_material_raycaster_is_reused_until_geometry_revision_changes() -> None:
    state = _state()
    initial = state.current_material_raycaster()

    assert state.current_material_raycaster() is initial

    action = _action()
    rejected = replace(
        _valid_validation(action),
        contact_valid=False,
        valid=False,
        rejection_reasons=(ActionRejectionReason.NO_CONTACT,),
    )
    state.apply_validated_action(action, rejected)
    assert state.current_material_raycaster() is initial

    state.apply_validated_action(action, _valid_validation(action))
    updated = state.current_material_raycaster()

    assert updated is not initial
    assert state.current_material_raycaster() is updated


def test_geometry_snapshot_recomputes_phi_and_shared_contour() -> None:
    state = _state()
    goal_vertices = np.array(
        [
            [-0.02, -0.02, -0.001],
            [0.02, -0.02, 0.010],
            [0.02, 0.02, 0.010],
            [-0.02, 0.02, -0.001],
        ]
    )
    goal_faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

    snapshot = state.geometry_snapshot(
        goal_vertices=goal_vertices,
        goal_faces=goal_faces,
        fill_tolerance_m=0.002,
        iso_level_m=0.0,
    )

    assert snapshot.geometry_revision == 0
    assert snapshot.evaluation.phi[0] < 0.0
    assert snapshot.evaluation.phi[1] > 0.0
    assert snapshot.contour_lines.shape[0] > 0
    assert snapshot.contour is not None
    assert snapshot.contour.total_length_m > 0.0
