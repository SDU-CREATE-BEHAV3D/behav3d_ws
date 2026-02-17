#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped


def pose_from_xyz_and_z_axis(
    *,
    xyz_m: Tuple[float, float, float],
    z_axis: Optional[Tuple[float, float, float]],
    frame_id: str,
) -> PoseStamped:
    """
    Build PoseStamped from XYZ (meters) and optional Z-axis normal.
    If z_axis is None, returns identity orientation.
    """
    ps = PoseStamped()
    ps.header.frame_id = str(frame_id or "world")
    ps.pose.position.x = float(xyz_m[0])
    ps.pose.position.y = float(xyz_m[1])
    ps.pose.position.z = float(xyz_m[2])

    if z_axis is None:
        ps.pose.orientation.w = 1.0
        return ps

    qx, qy, qz, qw = quat_from_z_axis(z_axis)
    ps.pose.orientation.x = float(qx)
    ps.pose.orientation.y = float(qy)
    ps.pose.orientation.z = float(qz)
    ps.pose.orientation.w = float(qw)
    return ps


def quat_from_z_axis(z_axis: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
    """
    Compute quaternion from desired tool Z axis.
    Uses a stable reference axis to resolve free roll around Z.
    """
    zx, zy, zz = float(z_axis[0]), float(z_axis[1]), float(z_axis[2])
    n = math.sqrt(zx * zx + zy * zy + zz * zz)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)

    zx /= n
    zy /= n
    zz /= n

    # Pick stable reference to define X around the provided Z normal.
    rx, ry, rz = (1.0, 0.0, 0.0)
    if abs(zx) > 0.95:
        rx, ry, rz = (0.0, 1.0, 0.0)

    dot = rx * zx + ry * zy + rz * zz
    xx = rx - dot * zx
    xy = ry - dot * zy
    xz = rz - dot * zz
    nx = math.sqrt(xx * xx + xy * xy + xz * xz)
    if nx < 1e-9:
        # Fallback orthonormal axis if projection degenerates.
        xx, xy, xz = (0.0, -zz, zy)
        nx = math.sqrt(xx * xx + xy * xy + xz * xz)
    xx /= nx
    xy /= nx
    xz /= nx

    yx = zy * xz - zz * xy
    yy = zz * xx - zx * xz
    yz = zx * xy - zy * xx

    return quat_from_rotmat(
        r00=xx, r01=yx, r02=zx,
        r10=xy, r11=yy, r12=zy,
        r20=xz, r21=yz, r22=zz,
    )


def quat_from_rotmat(
    *,
    r00: float,
    r01: float,
    r02: float,
    r10: float,
    r11: float,
    r12: float,
    r20: float,
    r21: float,
    r22: float,
) -> Tuple[float, float, float, float]:
    """
    Convert rotation matrix components to normalized quaternion (x,y,z,w).
    """
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s

    nq = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if nq < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / nq, qy / nq, qz / nq, qw / nq)

