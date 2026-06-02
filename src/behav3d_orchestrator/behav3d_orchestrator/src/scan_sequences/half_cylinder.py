from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import PoseStamped

from .common import TOOL_PLUS_Z_POINTS_OUTWARD, pose_from_position_rotation, rotation_from_z_axis


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
) -> list[PoseStamped]:
    if int(n_angle) < 2 or int(n_height) < 1:
        return []
    if float(radius) <= 0.0:
        raise ValueError("radius must be > 0")

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
            R_w = rotation_from_z_axis(z_axis, x_guess=np.array([0.0, 0.0, 1.0]))
            row.append(pose_from_position_rotation(p, R_w, str(frame_id or "world")))
        if not row_major and iz % 2 == 1:
            row.reverse()
        out.extend(row)
    return out
