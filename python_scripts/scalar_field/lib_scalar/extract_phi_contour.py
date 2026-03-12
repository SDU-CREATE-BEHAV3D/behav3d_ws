#!/usr/bin/env python3
"""Extract 3D contour lines where phi reaches a chosen iso-value.

This gives the boundary between classes such as:
- viable / non-viable (iso = 0)
- low-margin / high-margin (custom iso)
"""

from __future__ import annotations

import numpy as np


def extract_phi_contour(
    vertices: np.ndarray,
    faces: np.ndarray,
    scalar: np.ndarray,
    iso: float = 0.0,
    eps: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate phi crossings on triangle edges and return line segments."""
    seg_points: list[np.ndarray] = []
    seg_lines: list[list[int]] = []
    next_idx = 0

    for tri in faces:
        ids = [int(tri[0]), int(tri[1]), int(tri[2])]
        vals = np.array([scalar[ids[0]], scalar[ids[1]], scalar[ids[2]]], dtype=np.float64)
        pts = np.array([vertices[ids[0]], vertices[ids[1]], vertices[ids[2]]], dtype=np.float64)

        tri_cross: list[np.ndarray] = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            va = vals[a] - iso
            vb = vals[b] - iso
            if not np.isfinite(va) or not np.isfinite(vb):
                continue
            if abs(va) < eps and abs(vb) < eps:
                continue
            if (va > 0.0 and vb > 0.0) or (va < 0.0 and vb < 0.0):
                continue

            denom = va - vb
            if abs(denom) < eps:
                continue

            t = va / (va - vb)
            t = float(np.clip(t, 0.0, 1.0))
            p = pts[a] + t * (pts[b] - pts[a])
            tri_cross.append(p)

        if len(tri_cross) == 2:
            p0, p1 = tri_cross
            seg_points.append(p0)
            seg_points.append(p1)
            seg_lines.append([next_idx, next_idx + 1])
            next_idx += 2
        elif len(tri_cross) > 2:
            p0, p1 = tri_cross[0], tri_cross[1]
            seg_points.append(p0)
            seg_points.append(p1)
            seg_lines.append([next_idx, next_idx + 1])
            next_idx += 2

    if not seg_points:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 2), dtype=np.int32)
    return np.vstack(seg_points), np.asarray(seg_lines, dtype=np.int32)
