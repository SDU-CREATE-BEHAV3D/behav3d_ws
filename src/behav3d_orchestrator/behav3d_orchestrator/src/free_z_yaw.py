#!/usr/bin/env python3
"""Resolve the free local-Z rotation of YAML planes with minimal twist.

The orientation of every target is selected from its Z axis, transporting the
previous frame by the smallest rotation. This preserves the Z normal while
avoiding arbitrary jumps in tool yaw. It does not optimize joint motion or IK.
"""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Sequence

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


def _rotation(pose: PoseStamped) -> Rotation:
    q = pose.pose.orientation
    return Rotation.from_quat([q.x, q.y, q.z, q.w])


def transport_free_z_yaw(
    poses: Sequence[PoseStamped],
    reference_pose: PoseStamped,
) -> list[PoseStamped]:
    """Align each tool Z to its target, transporting X/Y with minimum twist."""
    current = _rotation(reference_pose)
    result: list[PoseStamped] = []

    for target in poses:
        target_z = _rotation(target).apply([0.0, 0.0, 1.0])
        target_z = np.asarray(target_z, dtype=float)
        target_z /= np.linalg.norm(target_z)

        current_z = current.apply([0.0, 0.0, 1.0])
        axis = np.cross(current_z, target_z)
        axis_norm = float(np.linalg.norm(axis))
        alignment = float(np.clip(np.dot(current_z, target_z), -1.0, 1.0))

        if axis_norm > 1e-10:
            angle = math.atan2(axis_norm, alignment)
            delta = Rotation.from_rotvec(axis / axis_norm * angle)
            current = delta * current
        elif alignment < 0.0:
            # The normal reversed; choose the existing tool X as a stable axis.
            delta = Rotation.from_rotvec(current.apply([1.0, 0.0, 0.0]) * math.pi)
            current = delta * current

        pose = deepcopy(target)
        qx, qy, qz, qw = current.as_quat()
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        result.append(pose)

    return result
