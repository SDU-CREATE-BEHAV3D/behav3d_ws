"""Downstream safety validation for one deterministically decoded action."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

import numpy as np
import numpy.typing as npt

from .action_decoder import DecodedAction
from .dds_adapter import point_deposit_from_action

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

    NO_CONTACT = "no_contact"
    CONTACT_DISTANCE = "contact_distance_out_of_bounds"
    CONTACT_SOURCE_MISMATCH = "contact_source_mismatch"
    OUT_OF_DOMAIN = "out_of_domain"
    COLLISION = "collision"


@dataclass(frozen=True)
class ActionValidation:
    """Contact, DDS-domain, and collision results for one decoded action."""

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


def _points(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _faces(values: npt.ArrayLike, vertex_count: int) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 3:
        raise ValueError(f"scan_faces must have shape (M, 3), got {raw.shape}")
    result = raw.astype(np.int32, copy=False)
    if result.size and (np.any(result < 0) or np.any(result >= vertex_count)):
        raise ValueError("scan_faces contains an invalid vertex index")
    return result


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
    contact_source_tolerance_m: float,
    domain: DomainBounds | None,
    collision_checker: PoseCollisionChecker | None,
) -> ActionValidation:
    """Validate one policy action without correcting or replacing it.

    Collision is checked only after contact and domain containment succeed.
    Rejected actions are returned with typed reason codes; applying penalties
    and keeping episode state unchanged are responsibilities of ``env.step``.
    """

    import open3d as o3d

    if not isinstance(action, DecodedAction):
        raise TypeError("action must be a DecodedAction")
    vertices = _points(scan_vertices, "scan_vertices")
    faces = _faces(scan_faces, vertices.shape[0])
    distance_min, distance_max = _finite_bounds(
        contact_distance_min_m,
        contact_distance_max_m,
        "contact distance",
    )
    tolerance = float(contact_source_tolerance_m)
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("contact_source_tolerance_m must be finite and >= 0")

    tensor_mesh = o3d.t.geometry.TriangleMesh()
    tensor_mesh.vertex["positions"] = o3d.core.Tensor(vertices.astype(np.float32))
    tensor_mesh.triangle["indices"] = o3d.core.Tensor(faces)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tensor_mesh)
    ray = np.concatenate((action.target_point, -action.z_axis)).astype(np.float32)
    hit_distance = float(
        scene.cast_rays(o3d.core.Tensor(ray.reshape(1, 6)))["t_hit"].numpy()[0]
    )
    has_hit = bool(np.isfinite(hit_distance))
    hit_point = np.full(3, np.nan, dtype=np.float64)
    source_error = float("inf")
    if has_hit:
        hit_point = action.target_point - hit_distance * action.z_axis
        source_error = float(np.linalg.norm(hit_point - action.source_point))
    distance_valid = has_hit and distance_min <= hit_distance <= distance_max
    source_valid = has_hit and source_error <= tolerance
    contact_valid = distance_valid and source_valid

    support_min, support_max = _support_bounds(action)
    domain_valid = True
    if domain is not None:
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
    if contact_valid and domain_valid and collision_checker is not None:
        collision_checked = True
        collision, clearance, hits = collision_checker.check_pose(
            action.target_point,
            action.z_axis,
        )
        collides = bool(collision)
        clearance_mm = float(clearance)
        hit_vertices = int(hits)

    reasons: list[ActionRejectionReason] = []
    if not has_hit:
        reasons.append(ActionRejectionReason.NO_CONTACT)
    else:
        if not distance_valid:
            reasons.append(ActionRejectionReason.CONTACT_DISTANCE)
        if not source_valid:
            reasons.append(ActionRejectionReason.CONTACT_SOURCE_MISMATCH)
    if not domain_valid:
        reasons.append(ActionRejectionReason.OUT_OF_DOMAIN)
    if collides:
        reasons.append(ActionRejectionReason.COLLISION)

    return ActionValidation(
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
