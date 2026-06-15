from __future__ import annotations

from geometry_msgs.msg import PoseStamped

from behav3d_utils import target_builder as tb


def build_targets_from_center(
    *,
    center: PoseStamped,
    width: float,
    height: float,
    z_off: float,
    nx: int,
    ny: int,
    row_major: bool = False,
) -> list[PoseStamped]:
    out: list[PoseStamped] = []
    dx = float(width) / float(nx - 1)
    dy = float(height) / float(ny - 1)

    cx = float(center.pose.position.x)
    cy = float(center.pose.position.y)
    cz = float(center.pose.position.z)
    q = center.pose.orientation

    for j in range(int(ny)):
        row: list[PoseStamped] = []
        for i in range(int(nx)):
            row.append(
                tb.poseStamped(
                    cx - 0.5 * float(width) + i * dx,
                    cy - 0.5 * float(height) + j * dy,
                    cz + float(z_off),
                    float(q.x),
                    float(q.y),
                    float(q.z),
                    float(q.w),
                    center.header.frame_id or "world",
                )
            )
        if not row_major and j % 2 == 1:
            row.reverse()
        out.extend(row)
    return out
