from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from .common import (
    TOOL_PLUS_Z_POINTS_OUTWARD,
    any_orthonormal,
    pose_from_position_rotation,
    rotation_from_z_axis,
    target_origin_rotation,
)


def build_targets(
    *,
    center_x: float,
    center_y: float,
    center_z: float,
    radius: float,
    height: float,
    angle_min_deg: float,
    angle_max_deg: float,
    n_angle: int,
    n_height: int,
    frame_id: str = "world",
    row_major: bool = False,
    orientation_mode: str = "look_at",
    orientation_pose: PoseStamped | None = None,
    axis_start_xyz: tuple[float, float, float] | None = None,
    axis_end_xyz: tuple[float, float, float] | None = None,
    n_axis: int | None = None,
    arc_center_direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
    roll_deg: float = 0.0,
) -> list[PoseStamped]:
    if int(n_angle) < 2 or int(n_height) < 1:
        return []
    if float(radius) <= 0.0:
        raise ValueError("radius must be > 0")

    if axis_start_xyz is not None or axis_end_xyz is not None:
        if axis_start_xyz is None or axis_end_xyz is None:
            raise ValueError("axis_start_xyz and axis_end_xyz must be provided together")
        return build_targets_around_line(
            axis_start_xyz=axis_start_xyz,
            axis_end_xyz=axis_end_xyz,
            radius=radius,
            angle_min_deg=angle_min_deg,
            angle_max_deg=angle_max_deg,
            n_angle=n_angle,
            n_axis=int(n_axis if n_axis is not None else n_height),
            frame_id=frame_id,
            row_major=row_major,
            orientation_mode=orientation_mode,
            orientation_pose=orientation_pose,
            arc_center_direction=arc_center_direction,
            roll_deg=roll_deg,
        )

    angles = np.linspace(math.radians(float(angle_min_deg)), math.radians(float(angle_max_deg)), int(n_angle))
    if int(n_height) == 1:
        z_values = np.array([float(center_z)], dtype=float)
    else:
        z_values = np.linspace(
            float(center_z) - 0.5 * float(height),
            float(center_z) + 0.5 * float(height),
            int(n_height),
        )

    out: list[PoseStamped] = []
    center_xy = np.array([float(center_x), float(center_y)], dtype=float)
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

    for iz, z in enumerate(z_values):
        row: list[PoseStamped] = []
        for theta in angles:
            radial = np.array([math.cos(float(theta)), math.sin(float(theta)), 0.0], dtype=float)
            p = np.array(
                [
                    center_xy[0] + float(radius) * radial[0],
                    center_xy[1] + float(radius) * radial[1],
                    float(z),
                ],
                dtype=float,
            )
            z_axis = radial if TOOL_PLUS_Z_POINTS_OUTWARD else -radial
            if fixed_rotation is not None:
                R_w = fixed_rotation
            elif reference_rotation is not None:
                R_w = rotation_from_z_axis(z_axis, x_guess=reference_rotation.as_matrix()[:, 0])
            else:
                R_w = rotation_from_z_axis(z_axis, x_guess=np.array([0.0, 0.0, 1.0]))
            R_w = _apply_local_z_roll(R_w, roll_deg)
            row.append(pose_from_position_rotation(p, R_w, str(frame_id or "world")))
        if not row_major and iz % 2 == 1:
            row.reverse()
        out.extend(row)
    return out


def build_targets_around_line(
    *,
    axis_start_xyz: tuple[float, float, float],
    axis_end_xyz: tuple[float, float, float],
    radius: float,
    angle_min_deg: float,
    angle_max_deg: float,
    n_angle: int,
    n_axis: int,
    frame_id: str = "world",
    row_major: bool = False,
    orientation_mode: str = "look_at",
    orientation_pose: PoseStamped | None = None,
    arc_center_direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
    roll_deg: float = 0.0,
) -> list[PoseStamped]:
    if int(n_angle) < 2 or int(n_axis) < 1:
        return []
    if float(radius) <= 0.0:
        raise ValueError("radius must be > 0")

    p0 = np.asarray(axis_start_xyz, dtype=float).reshape(3)
    p1 = np.asarray(axis_end_xyz, dtype=float).reshape(3)
    axis = p1 - p0
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        raise ValueError("half-cylinder axis line must have non-zero length")
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

    if int(n_axis) == 1:
        axis_points = np.array([(p0 + p1) * 0.5], dtype=float)
    else:
        ts = np.linspace(0.0, 1.0, int(n_axis))
        axis_points = np.array([p0 + t * (p1 - p0) for t in ts], dtype=float)

    angles = np.linspace(math.radians(float(angle_min_deg)), math.radians(float(angle_max_deg)), int(n_angle))
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
    for ia, axis_point in enumerate(axis_points):
        row: list[PoseStamped] = []
        for theta in angles:
            radial = math.cos(float(theta)) * arc_mid + math.sin(float(theta)) * arc_side
            radial = radial / max(np.linalg.norm(radial), 1e-12)
            p = axis_point + float(radius) * radial
            z_axis = radial if TOOL_PLUS_Z_POINTS_OUTWARD else -radial
            if fixed_rotation is not None:
                R_w = fixed_rotation
            elif reference_rotation is not None:
                R_w = rotation_from_z_axis(z_axis, x_guess=reference_rotation.as_matrix()[:, 0])
            else:
                R_w = rotation_from_z_axis(z_axis, x_guess=axis)
            R_w = _apply_local_z_roll(R_w, roll_deg)
            row.append(pose_from_position_rotation(p, R_w, str(frame_id or "world")))
        if not row_major and ia % 2 == 1:
            row.reverse()
        out.extend(row)
    return out


def _apply_local_z_roll(rotation: R, roll_deg: float) -> R:
    roll = float(roll_deg)
    if abs(roll) < 1e-12:
        return rotation
    return rotation * R.from_euler("z", roll, degrees=True)
