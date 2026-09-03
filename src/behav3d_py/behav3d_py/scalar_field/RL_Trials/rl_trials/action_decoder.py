"""Deterministic decoder for direct continuous RL bead actions.

The policy owns the fabrication action. No random candidate pool sits between
PPO and the geometry. A normalized five-dimensional action in ``[-1, 1]^5`` is
mapped directly to a source point on the current ``phi=0`` contour, bead
height, deposition orientation, and width correction.

Action layout::

    [source_coord, height_ctrl, orient_x, orient_y, width_ctrl]

``source_coord`` addresses the complete current contour by cumulative physical
arc length. ``orient_x`` and ``orient_y`` are mapped from the square to a unit
disk and then to a tilt cone. This keeps the neutral policy output
``orient_x = orient_y = 0`` exactly vertical and avoids the singular/wrapped
``tilt + azimuth`` parameterization at zero tilt.

This module only decodes the requested action. Contact, collision, DDS-domain,
and robot-safety checks remain explicit downstream validators so an invalid
policy action can be rejected and penalized rather than silently replaced.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]


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


def _as_points(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _as_lines(values: npt.ArrayLike, point_count: int) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 2:
        raise ValueError(f"contour_lines must have shape (M, 2), got {raw.shape}")
    result = raw.astype(np.int32, copy=False)
    if result.size and (np.any(result < 0) or np.any(result >= point_count)):
        raise ValueError("contour_lines contains an invalid point index")
    return result


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
    """Shirley-Chiu concentric square-to-disk map.

    This deterministic map covers the complete unit disk without clipping.
    The origin maps exactly to the origin, which means a zero-centered policy
    initially requests a vertical deposition direction.
    """

    if x == 0.0 and y == 0.0:
        return 0.0, 0.0

    if abs(x) > abs(y):
        radius = x
        angle = (np.pi / 4.0) * (y / x)
    else:
        radius = y
        angle = (np.pi / 2.0) - (np.pi / 4.0) * (x / y)
    return float(radius * np.cos(angle)), float(radius * np.sin(angle))


def _source_on_contour(
    *,
    points: FloatArray,
    lines: IntArray,
    arc_fraction: float,
) -> tuple[FloatArray, int, float, float]:
    starts = points[lines[:, 0]]
    ends = points[lines[:, 1]]
    lengths = np.linalg.norm(ends - starts, axis=1)
    usable = lengths > 1e-12
    if not np.any(usable):
        raise ValueError("phi contour contains no non-degenerate segments")

    usable_indices = np.flatnonzero(usable)
    usable_lengths = lengths[usable]
    cumulative = np.cumsum(usable_lengths)
    total_length = float(cumulative[-1])
    distance = float(arc_fraction) * total_length

    if distance >= total_length:
        local_usable_index = usable_lengths.shape[0] - 1
        segment_fraction = 1.0
    else:
        local_usable_index = int(np.searchsorted(cumulative, distance, side="right"))
        previous = 0.0 if local_usable_index == 0 else float(cumulative[local_usable_index - 1])
        segment_fraction = (distance - previous) / float(usable_lengths[local_usable_index])

    segment_index = int(usable_indices[local_usable_index])
    start = starts[segment_index]
    end = ends[segment_index]
    source = (1.0 - segment_fraction) * start + segment_fraction * end
    return source, segment_index, float(segment_fraction), distance


def decode_continuous_action(
    *,
    normalized_action: npt.ArrayLike,
    contour_points: npt.ArrayLike,
    contour_lines: npt.ArrayLike,
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
    """Decode one PPO action directly into one geometric bead proposal.

    No random sampling is performed here. The same state and normalized action
    always produce the same requested bead action.
    """

    action = _normalized_action(normalized_action)
    points = _as_points(contour_points, "contour_points")
    lines = _as_lines(contour_lines, points.shape[0])
    if lines.shape[0] == 0:
        raise ValueError("cannot decode an action from an empty phi contour")
    heat_at_points = _per_point(contour_heat, points.shape[0], "contour_heat")
    widths_at_points = _per_point(
        contour_widths_m,
        points.shape[0],
        "contour_widths_m",
    )
    if np.any(widths_at_points <= 0.0):
        raise ValueError("contour_widths_m values must be positive")

    source_arc_fraction = 0.5 * (float(action[0]) + 1.0)
    source, segment_index, segment_fraction, source_arc_length_m = _source_on_contour(
        points=points,
        lines=lines,
        arc_fraction=source_arc_fraction,
    )
    line = lines[segment_index]
    heat = float(
        (1.0 - segment_fraction) * heat_at_points[line[0]]
        + segment_fraction * heat_at_points[line[1]]
    )
    width_map_m = float(
        (1.0 - segment_fraction) * widths_at_points[line[0]]
        + segment_fraction * widths_at_points[line[1]]
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

    target = source + height_m * z_axis
    return DecodedAction(
        normalized_action=action.copy(),
        source_point=source,
        target_point=target,
        z_axis=z_axis,
        height_m=float(height_m),
        width_map_m=width_map_m,
        width_m=float(width_m),
        heat=heat,
        source_arc_fraction=source_arc_fraction,
        source_arc_length_m=source_arc_length_m,
        contour_segment_index=segment_index,
        segment_fraction=segment_fraction,
        tilt_deg=float(np.rad2deg(tilt_rad)),
        azimuth_rad=azimuth_rad,
        width_delta_m=float(width_delta_m),
    )
