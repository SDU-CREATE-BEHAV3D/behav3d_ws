from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from . import half_cylinder
from .common import (
    TOOL_PLUS_Z_POINTS_OUTWARD,
    any_orthonormal,
    pose_from_position_rotation,
    rotation_from_z_axis,
    target_origin_rotation,
)


def build_targets(
    *,
    axis_start_xyz: Sequence[float],
    axis_end_xyz: Sequence[float],
    radius: float,
    angle_min_deg: float,
    angle_max_deg: float,
    n_angle: int,
    n_axis: int,
    frame_id: str = "world",
    row_major: bool = False,
    orientation_mode: str = "look_at",
    orientation_pose: PoseStamped | None = None,
    arc_center_direction: Sequence[float] = (0.0, 0.0, 1.0),
    roll_deg: float = 0.0,
    endcap_radius: float | None = None,
    endcap_angle_min_deg: float | None = None,
    endcap_angle_max_deg: float | None = None,
    endcap_n_angle: int | None = None,
    endcap_polar_min_deg: float = 15.0,
    endcap_polar_max_deg: float = 75.0,
    endcap_n_polar: int = 3,
    endcap_include_start: bool = True,
    endcap_include_end: bool = True,
    endcap_row_major: bool = False,
) -> list[PoseStamped]:
    """
    Build a line-axis half-cylinder scan plus hemispherical side-cap targets.

    The main half-cylinder scans along the line. The side caps scan from outside
    each end of the line, using spherical sectors around the start/end point.
    """
    p0, p1, axis, arc_mid, arc_side = _line_basis(
        axis_start_xyz=axis_start_xyz,
        axis_end_xyz=axis_end_xyz,
        arc_center_direction=arc_center_direction,
    )

    main_targets = half_cylinder.build_targets_around_line(
        axis_start_xyz=(float(p0[0]), float(p0[1]), float(p0[2])),
        axis_end_xyz=(float(p1[0]), float(p1[1]), float(p1[2])),
        radius=float(radius),
        angle_min_deg=float(angle_min_deg),
        angle_max_deg=float(angle_max_deg),
        n_angle=int(n_angle),
        n_axis=int(n_axis),
        frame_id=str(frame_id or "world"),
        row_major=bool(row_major),
        orientation_mode=str(orientation_mode or "look_at"),
        orientation_pose=orientation_pose,
        arc_center_direction=(float(arc_mid[0]), float(arc_mid[1]), float(arc_mid[2])),
        roll_deg=float(roll_deg),
    )

    cap_radius = float(radius if endcap_radius is None or float(endcap_radius) <= 0.0 else endcap_radius)
    cap_n_angle = int(n_angle if endcap_n_angle is None or int(endcap_n_angle) <= 0 else endcap_n_angle)
    cap_angle_min = float(angle_min_deg if endcap_angle_min_deg is None else endcap_angle_min_deg)
    cap_angle_max = float(angle_max_deg if endcap_angle_max_deg is None else endcap_angle_max_deg)
    cap_targets_start: list[PoseStamped] = []
    cap_targets_end: list[PoseStamped] = []

    if endcap_include_start:
        cap_targets_start = build_endcap_targets(
            endpoint=p0,
            outside_axis=-axis,
            line_axis=axis,
            arc_mid=arc_mid,
            arc_side=arc_side,
            radius=cap_radius,
            angle_min_deg=cap_angle_min,
            angle_max_deg=cap_angle_max,
            n_angle=cap_n_angle,
            polar_min_deg=float(endcap_polar_min_deg),
            polar_max_deg=float(endcap_polar_max_deg),
            n_polar=int(endcap_n_polar),
            frame_id=str(frame_id or "world"),
            row_major=bool(endcap_row_major),
            outside_to_inside=True,
            orientation_mode=str(orientation_mode or "look_at"),
            orientation_pose=orientation_pose,
            roll_deg=float(roll_deg),
        )

    if endcap_include_end:
        cap_targets_end = build_endcap_targets(
            endpoint=p1,
            outside_axis=axis,
            line_axis=axis,
            arc_mid=arc_mid,
            arc_side=arc_side,
            radius=cap_radius,
            angle_min_deg=cap_angle_min,
            angle_max_deg=cap_angle_max,
            n_angle=cap_n_angle,
            polar_min_deg=float(endcap_polar_min_deg),
            polar_max_deg=float(endcap_polar_max_deg),
            n_polar=int(endcap_n_polar),
            frame_id=str(frame_id or "world"),
            row_major=bool(endcap_row_major),
            outside_to_inside=False,
            orientation_mode=str(orientation_mode or "look_at"),
            orientation_pose=orientation_pose,
            roll_deg=float(roll_deg),
        )

    return cap_targets_start + main_targets + cap_targets_end


def build_endcap_targets(
    *,
    endpoint: np.ndarray,
    outside_axis: np.ndarray,
    line_axis: np.ndarray,
    arc_mid: np.ndarray,
    arc_side: np.ndarray,
    radius: float,
    angle_min_deg: float,
    angle_max_deg: float,
    n_angle: int,
    polar_min_deg: float,
    polar_max_deg: float,
    n_polar: int,
    frame_id: str = "world",
    row_major: bool = False,
    outside_to_inside: bool = False,
    orientation_mode: str = "look_at",
    orientation_pose: PoseStamped | None = None,
    roll_deg: float = 0.0,
) -> list[PoseStamped]:
    if int(n_angle) < 1 or int(n_polar) < 1:
        return []
    if float(radius) <= 0.0:
        raise ValueError("endcap radius must be > 0")
    if float(polar_min_deg) < 0.0 or float(polar_max_deg) > 90.0:
        raise ValueError("endcap polar angles must stay inside [0, 90] degrees")
    if float(polar_min_deg) > float(polar_max_deg):
        raise ValueError("endcap_polar_min_deg must be <= endcap_polar_max_deg")

    endpoint = np.asarray(endpoint, dtype=float).reshape(3)
    outside = np.asarray(outside_axis, dtype=float).reshape(3)
    outside = outside / max(np.linalg.norm(outside), 1e-12)
    axis = np.asarray(line_axis, dtype=float).reshape(3)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    arc_mid = np.asarray(arc_mid, dtype=float).reshape(3)
    arc_mid = arc_mid / max(np.linalg.norm(arc_mid), 1e-12)
    arc_side = np.asarray(arc_side, dtype=float).reshape(3)
    arc_side = arc_side / max(np.linalg.norm(arc_side), 1e-12)

    angles = np.linspace(math.radians(float(angle_min_deg)), math.radians(float(angle_max_deg)), int(n_angle))
    polar_values = list(
        np.linspace(math.radians(float(polar_min_deg)), math.radians(float(polar_max_deg)), int(n_polar))
    )
    if bool(outside_to_inside):
        polar_values.reverse()

    mode = str(orientation_mode or "look_at").strip().lower()
    fixed_rotation = None
    reference_rotation = None
    if mode in ("fixed", "current", "current_eef"):
        if orientation_pose is None:
            raise ValueError("orientation_pose is required for fixed/current_eef orientation mode")
        _, fixed_rotation = target_origin_rotation(orientation_pose)
    elif mode in ("look_at_current_roll", "look_at_keep_roll", "look_at_min_roll"):
        if orientation_pose is None:
            raise ValueError("orientation_pose is required for look_at_current_roll orientation mode")
        _, reference_rotation = target_origin_rotation(orientation_pose)

    out: list[PoseStamped] = []
    for ip, phi in enumerate(polar_values):
        row: list[PoseStamped] = []
        for theta in angles:
            radial = math.cos(float(theta)) * arc_mid + math.sin(float(theta)) * arc_side
            radial = radial / max(np.linalg.norm(radial), 1e-12)
            direction = math.cos(float(phi)) * radial + math.sin(float(phi)) * outside
            direction = direction / max(np.linalg.norm(direction), 1e-12)
            position = endpoint + float(radius) * direction
            z_axis = direction if TOOL_PLUS_Z_POINTS_OUTWARD else -direction
            if fixed_rotation is not None:
                R_w = fixed_rotation
            elif reference_rotation is not None:
                R_w = rotation_from_z_axis(z_axis, x_guess=reference_rotation.as_matrix()[:, 0])
            else:
                x_guess = axis
                if abs(float(np.dot(direction, axis))) > 0.92:
                    x_guess = arc_mid
                R_w = rotation_from_z_axis(z_axis, x_guess=x_guess)
            R_w = _apply_local_z_roll(R_w, roll_deg)
            row.append(pose_from_position_rotation(position, R_w, str(frame_id or "world")))
        if not row_major and ip % 2 == 1:
            row.reverse()
        out.extend(row)
    return out


def _line_basis(
    *,
    axis_start_xyz: Sequence[float],
    axis_end_xyz: Sequence[float],
    arc_center_direction: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    p0 = np.asarray(axis_start_xyz, dtype=float).reshape(3)
    p1 = np.asarray(axis_end_xyz, dtype=float).reshape(3)
    axis = p1 - p0
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        raise ValueError("side-cap scan axis line must have non-zero length")
    axis = axis / axis_norm

    arc_mid = np.asarray(arc_center_direction, dtype=float).reshape(3)
    arc_mid = arc_mid - np.dot(arc_mid, axis) * axis
    arc_mid_norm = np.linalg.norm(arc_mid)
    if arc_mid_norm < 1e-9:
        arc_mid = any_orthonormal(axis)
    else:
        arc_mid = arc_mid / arc_mid_norm
    arc_side = np.cross(axis, arc_mid)
    arc_side = arc_side / max(np.linalg.norm(arc_side), 1e-12)
    return p0, p1, axis, arc_mid, arc_side


def _apply_local_z_roll(rotation: R, roll_deg: float) -> R:
    roll = float(roll_deg)
    if abs(roll) < 1e-12:
        return rotation
    return rotation * R.from_euler("z", roll, degrees=True)
