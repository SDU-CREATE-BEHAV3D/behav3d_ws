"""Pure reward components for RL bead deposition.

These functions return dimensionless *costs* in non-negative form. The
Gymnasium environment can log them separately and subtract weighted versions
from reward. Keeping them separate avoids hiding physical effects inside one
opaque reward scalar.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


def normalized_tilt_cost(
    z_axis: npt.ArrayLike,
    *,
    max_tilt_deg: float,
) -> float:
    """Quadratic preference for the minimum necessary deposition tilt.

    Returns 0 for vertical deposition and 1 at ``max_tilt_deg``. It is intended
    as a weak regularizer: progress, overlap, and collision consequences should
    be free to outweigh it when inclination is useful.
    """

    axis = np.asarray(z_axis, dtype=np.float64).reshape(-1)
    if axis.shape != (3,) or not np.all(np.isfinite(axis)):
        raise ValueError("z_axis must be one finite 3-vector")
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12:
        raise ValueError("z_axis must be non-zero")
    axis = axis / norm

    maximum = float(max_tilt_deg)
    if not np.isfinite(maximum) or maximum <= 0.0 or maximum > 90.0:
        raise ValueError("max_tilt_deg must be in (0, 90]")

    tilt = float(np.arccos(np.clip(axis[2], -1.0, 1.0)))
    max_tilt = float(np.deg2rad(maximum))
    return float((tilt / max_tilt) ** 2)


def cantilever_ratio_cost(
    source_point: npt.ArrayLike,
    target_point: npt.ArrayLike,
    *,
    bead_width_m: float,
) -> float:
    """Quadratic lateral cantilever cost normalized by bead width.

    The lateral XY shift between supported source ``S`` and top-referenced
    target ``O`` is divided by bead width before squaring. This distinguishes
    the physical support consequence of tilt from the tilt angle itself.
    """

    source = np.asarray(source_point, dtype=np.float64).reshape(-1)
    target = np.asarray(target_point, dtype=np.float64).reshape(-1)
    if source.shape != (3,) or target.shape != (3,):
        raise ValueError("source_point and target_point must each be 3-vectors")
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("source_point and target_point must be finite")

    width = float(bead_width_m)
    if not np.isfinite(width) or width <= 0.0:
        raise ValueError("bead_width_m must be finite and positive")

    lateral_shift = float(np.linalg.norm((target - source)[:2]))
    ratio = lateral_shift / width
    return float(ratio * ratio)
