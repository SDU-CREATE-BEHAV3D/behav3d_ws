"""Downstream safety validation for one deterministically decoded action."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

import numpy as np
import numpy.typing as npt

from .action_decoder import DecodedAction
from .dds_adapter import point_deposit_from_action
from .raycasting import MeshRaycaster

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]


class PoseCollisionChecker(Protocol):
    """Minimal interface implemented by the existing extruder checker."""

    def check_pose(
        self,
        position: np.ndarray,
        z_axis: np.ndarray,
    ) -> tuple[bool, float, int]: ...


class DomainBounds(Protocol):
    """DDS-domain subset needed for conservative support containment."""

    min_corner: tuple[float, float, float]
    max_corner: tuple[float, float, float]


class ActionRejectionReason(str, Enum):
    """Stable reason codes for rejected policy actions."""

    INVALID_WIDTH = "invalid_width"
    NO_CONTACT = "no_contact"
    CONTACT_DISTANCE = "contact_distance_out_of_bounds"
    OUT_OF_DOMAIN = "out_of_domain"
    COLLISION = "collision"


@dataclass(frozen=True)
class ActionValidation:
    """Contact, DDS-domain, and collision results for one decoded action."""

    width_valid: bool
    contact_hit_point: FloatArray
    contact_distance_m: float
    contact_source_error_m: float
    contact_valid: bool
    support_min: FloatArray
    support_max: FloatArray
    domain_valid: bool
    collision_checked: bool
    collides: bool
    extruder_clearance_mm: float
    extruder_hit_vertices: int
    valid: bool
    rejection_reasons: tuple[ActionRejectionReason, ...]


def _finite_bounds(lower: float, upper: float, name: str) -> tuple[float, float]:
    low = float(lower)
    high = float(upper)
    if not np.isfinite(low) or not np.isfinite(high) or high < low:
        raise ValueError(f"invalid {name} bounds: ({lower}, {upper})")
    return low, high


def _support_bounds(action: DecodedAction) -> tuple[FloatArray, FloatArray]:
    deposit = point_deposit_from_action(action)
    lower, upper = deposit.support_bounds()
    return lower.to_array(), upper.to_array()


def validate_decoded_action(
    *,
    action: DecodedAction,
    scan_vertices: npt.ArrayLike,
    scan_faces: npt.ArrayLike,
    contact_distance_min_m: float,
    contact_distance_max_m: float,
    contact_distance_epsilon_m: float,
    domain: DomainBounds | None,
    collision_checker: PoseCollisionChecker | None,
    raycaster: MeshRaycaster | None = None,
) -> ActionValidation:
    """Validate one policy action without correcting or replacing it.

    Collision is checked only after contact and domain containment succeed.
    Rejected actions are returned with typed reason codes; applying penalties
    and keeping episode state unchanged are responsibilities of ``env.step``.
    """

    if not isinstance(action, DecodedAction):
        raise TypeError("action must be a DecodedAction")
    distance_min, distance_max = _finite_bounds(
        contact_distance_min_m,
        contact_distance_max_m,
        "contact distance",
    )
    distance_epsilon = float(contact_distance_epsilon_m)
    if not np.isfinite(distance_epsilon) or distance_epsilon < 0.0:
        raise ValueError("contact_distance_epsilon_m must be finite and >= 0")

    width_valid = bool(action.width_valid)
    active_raycaster = (
        MeshRaycaster(scan_vertices, scan_faces) if raycaster is None else raycaster
    )
    if not isinstance(active_raycaster, MeshRaycaster):
        raise TypeError("raycaster must be a MeshRaycaster")
    hit_distance = active_raycaster.cast_distance(
        action.target_point,
        -action.z_axis,
    )
    has_hit = bool(np.isfinite(hit_distance))
    hit_point = np.full(3, np.nan, dtype=np.float64)
    source_error = float("inf")
    if has_hit:
        hit_point = action.target_point - hit_distance * action.z_axis
        source_error = float(np.linalg.norm(hit_point - action.source_point))
    distance_valid = bool(
        has_hit
        and distance_min - distance_epsilon
        <= hit_distance
        <= distance_max + distance_epsilon
    )
    contact_valid = distance_valid

    if action.width_m > 0.0:
        support_min, support_max = _support_bounds(action)
    else:
        support_min = np.full(3, np.nan, dtype=np.float64)
        support_max = np.full(3, np.nan, dtype=np.float64)
    domain_valid = True
    if domain is not None and action.width_m > 0.0:
        domain_min = np.asarray(domain.min_corner, dtype=np.float64)
        domain_max = np.asarray(domain.max_corner, dtype=np.float64)
        if domain_min.shape != (3,) or domain_max.shape != (3,):
            raise ValueError("domain bounds must each contain three values")
        if not np.all(np.isfinite(domain_min)) or not np.all(np.isfinite(domain_max)):
            raise ValueError("domain bounds must be finite")
        if np.any(domain_min >= domain_max):
            raise ValueError("domain bounds must be strictly increasing")
        domain_valid = bool(
            np.all(support_min >= domain_min) and np.all(support_max <= domain_max)
        )

    collision_checked = False
    collides = False
    clearance_mm = float("nan")
    hit_vertices = 0
    if (
        width_valid
        and contact_valid
        and domain_valid
        and collision_checker is not None
    ):
        collision_checked = True
        collision, clearance, hits = collision_checker.check_pose(
            action.target_point,
            action.z_axis,
        )
        collides = bool(collision)
        clearance_mm = float(clearance)
        hit_vertices = int(hits)

    reasons: list[ActionRejectionReason] = []
    if not width_valid:
        reasons.append(ActionRejectionReason.INVALID_WIDTH)
    if not has_hit:
        reasons.append(ActionRejectionReason.NO_CONTACT)
    elif not distance_valid:
        reasons.append(ActionRejectionReason.CONTACT_DISTANCE)
    if not domain_valid:
        reasons.append(ActionRejectionReason.OUT_OF_DOMAIN)
    if collides:
        reasons.append(ActionRejectionReason.COLLISION)

    return ActionValidation(
        width_valid=width_valid,
        contact_hit_point=hit_point,
        contact_distance_m=hit_distance,
        contact_source_error_m=source_error,
        contact_valid=contact_valid,
        support_min=support_min,
        support_max=support_max,
        domain_valid=domain_valid,
        collision_checked=collision_checked,
        collides=collides,
        extruder_clearance_mm=clearance_mm,
        extruder_hit_vertices=hit_vertices,
        valid=not reasons,
        rejection_reasons=tuple(reasons),
    )
