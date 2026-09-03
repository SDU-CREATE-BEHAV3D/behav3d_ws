"""Deterministic decoder for direct continuous RL bead actions.

The policy emits ``[source, height, orient_x, orient_y, width]`` in
``[-1, 1]^5``. A shared :class:`ContourParameterization` maps ``source`` to
the current 3D contour; the remaining controls map to physical bead geometry.
No random candidate generation occurs between PPO and execution.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from .contour_parameterization import ContourParameterization

FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True)
class DecodedAction:
    """One policy-selected bead action in world coordinates."""

    normalized_action: FloatArray
    source_point: FloatArray
    target_point: FloatArray
    z_axis: FloatArray
    height_m: float
    width_map_m: float
    width_m: float
    heat: float
    source_arc_fraction: float
    source_arc_length_m: float
    contour_segment_index: int
    segment_fraction: float
    tilt_deg: float
    azimuth_rad: float
    width_delta_m: float


def _per_point(values: npt.ArrayLike, point_count: int, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if result.shape[0] != point_count:
        raise ValueError(f"{name} must contain one value per contour point")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _bounded(value: float, lower: float, upper: float, name: str) -> float:
    value = float(value)
    lower = float(lower)
    upper = float(upper)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if not np.isfinite(lower) or not np.isfinite(upper) or upper < lower:
        raise ValueError(f"invalid {name} bounds: ({lower}, {upper})")
    return lower + value * (upper - lower)


def _normalized_action(values: npt.ArrayLike) -> FloatArray:
    action = np.asarray(values, dtype=np.float64).reshape(-1)
    if action.shape != (5,):
        raise ValueError(f"normalized_action must have shape (5,), got {action.shape}")
    if not np.all(np.isfinite(action)):
        raise ValueError("normalized_action must contain only finite values")
    if np.any(action < -1.0) or np.any(action > 1.0):
        raise ValueError("normalized_action values must lie in [-1, 1]")
    return action


def _square_to_disk(x: float, y: float) -> tuple[float, float]:
    """Map the square to a disk while preserving the exact neutral origin."""

    if x == 0.0 and y == 0.0:
        return 0.0, 0.0
    if abs(x) > abs(y):
        radius = x
        angle = (np.pi / 4.0) * (y / x)
    else:
        radius = y
        angle = (np.pi / 2.0) - (np.pi / 4.0) * (x / y)
    return float(radius * np.cos(angle)), float(radius * np.sin(angle))


def decode_continuous_action(
    *,
    normalized_action: npt.ArrayLike,
    contour: ContourParameterization,
    contour_heat: npt.ArrayLike,
    contour_widths_m: npt.ArrayLike,
    height_min_m: float,
    height_max_m: float,
    cone_max_tilt_deg: float,
    width_delta_min_m: float,
    width_delta_max_m: float,
    width_min_m: float,
    width_max_m: float,
) -> DecodedAction:
    """Decode one PPO action directly into one geometric bead proposal."""

    action = _normalized_action(normalized_action)
    if not isinstance(contour, ContourParameterization):
        raise TypeError("contour must be a ContourParameterization")
    heat_at_points = _per_point(
        contour_heat,
        contour.points.shape[0],
        "contour_heat",
    )
    widths_at_points = _per_point(
        contour_widths_m,
        contour.points.shape[0],
        "contour_widths_m",
    )
    if np.any(widths_at_points <= 0.0):
        raise ValueError("contour_widths_m values must be positive")

    location = contour.locate(float(action[0]))
    line = contour.lines[location.segment_index]
    heat = float(
        (1.0 - location.segment_fraction) * heat_at_points[line[0]]
        + location.segment_fraction * heat_at_points[line[1]]
    )
    width_map_m = float(
        (1.0 - location.segment_fraction) * widths_at_points[line[0]]
        + location.segment_fraction * widths_at_points[line[1]]
    )

    height_alpha = 0.5 * (float(action[1]) + 1.0)
    height_m = _bounded(height_alpha, height_min_m, height_max_m, "height")

    cone_max_tilt_deg = float(cone_max_tilt_deg)
    if not np.isfinite(cone_max_tilt_deg) or not 0.0 <= cone_max_tilt_deg <= 90.0:
        raise ValueError("cone_max_tilt_deg must be in [0, 90]")
    disk_x, disk_y = _square_to_disk(float(action[2]), float(action[3]))
    disk_radius = float(np.hypot(disk_x, disk_y))
    tilt_rad = disk_radius * np.deg2rad(cone_max_tilt_deg)
    azimuth_rad = 0.0 if disk_radius <= 1e-15 else float(np.arctan2(disk_y, disk_x))
    sin_tilt = np.sin(tilt_rad)
    z_axis = np.array(
        [
            sin_tilt * np.cos(azimuth_rad),
            sin_tilt * np.sin(azimuth_rad),
            np.cos(tilt_rad),
        ],
        dtype=np.float64,
    )

    width_delta_alpha = 0.5 * (float(action[4]) + 1.0)
    width_delta_m = _bounded(
        width_delta_alpha,
        width_delta_min_m,
        width_delta_max_m,
        "width delta",
    )
    width_m = width_map_m + width_delta_m
    if width_m < float(width_min_m) or width_m > float(width_max_m):
        raise ValueError(
            "policy-selected width lies outside physical bead-width bounds; "
            "reject and penalize instead of clipping"
        )

    target = location.point + height_m * z_axis
    return DecodedAction(
        normalized_action=action.copy(),
        source_point=location.point.copy(),
        target_point=target,
        z_axis=z_axis,
        height_m=float(height_m),
        width_map_m=width_map_m,
        width_m=float(width_m),
        heat=heat,
        source_arc_fraction=location.arc_fraction,
        source_arc_length_m=location.arc_length_m,
        contour_segment_index=location.segment_index,
        segment_fraction=location.segment_fraction,
        tilt_deg=float(np.rad2deg(tilt_rad)),
        azimuth_rad=azimuth_rad,
        width_delta_m=float(width_delta_m),
    )
