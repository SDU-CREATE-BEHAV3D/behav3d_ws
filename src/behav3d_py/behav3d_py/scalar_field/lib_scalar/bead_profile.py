#!/usr/bin/env python3
"""Nominal bead-profile mapping, spacing, and volume helpers."""

from __future__ import annotations

from pathlib import Path

import numpy as np


WIDTH_MAP_KEY = "width_norm"


def load_normalized_width_map(path: Path, *, expected_count: int) -> np.ndarray:
    """Load a normalized per-vertex width map from an NPZ file."""
    map_path = Path(path).expanduser().resolve()
    if not map_path.is_file():
        raise FileNotFoundError(f"Width field not found: {map_path}")

    with np.load(str(map_path), allow_pickle=False) as payload:
        if WIDTH_MAP_KEY not in payload:
            raise KeyError(
                f"Width field must contain '{WIDTH_MAP_KEY}': {map_path}"
            )
        values = np.asarray(payload[WIDTH_MAP_KEY], dtype=np.float64).reshape(-1)

    if values.shape[0] != int(expected_count):
        raise ValueError(
            "Width field length does not match field vertices: "
            f"{values.shape[0]} vs {int(expected_count)}"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("Width field contains non-finite values.")
    tolerance = 1e-6
    if np.any(values < -tolerance) or np.any(values > 1.0 + tolerance):
        raise ValueError(
            "Width field values must be normalized to [0, 1]. "
            f"Observed range=({float(np.min(values)):.6f}, {float(np.max(values)):.6f})"
        )
    return np.clip(values, 0.0, 1.0)


def normalized_width_to_mm(
    normalized: np.ndarray,
    *,
    width_min_mm: float,
    width_max_mm: float,
) -> np.ndarray:
    """Map normalized values linearly to nominal bead widths in millimeters."""
    width_min = float(width_min_mm)
    width_max = float(width_max_mm)
    if not np.isfinite(width_min) or not np.isfinite(width_max):
        raise ValueError("Bead width bounds must be finite.")
    if width_min <= 0.0 or width_max <= 0.0:
        raise ValueError("Bead width bounds must be positive.")
    if width_max < width_min:
        raise ValueError(
            f"width_max_mm must be >= width_min_mm, got {width_max} < {width_min}"
        )

    values = np.asarray(normalized, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("Normalized width values must be finite.")
    values = np.clip(values, 0.0, 1.0)

    # Keep the width law isolated here so a calibrated nonlinear relation can
    # replace this linear map without changing candidate generation.
    return width_min + values * (width_max - width_min)


def minimum_center_distance(
    width_a: np.ndarray | float,
    width_b: np.ndarray | float,
    *,
    overlap: float,
) -> np.ndarray:
    """Return pairwise center distance for two full widths in matching units."""
    overlap_value = float(overlap)
    if not np.isfinite(overlap_value) or overlap_value < 0.0:
        raise ValueError(f"overlap must be finite and >= 0, got {overlap}")
    a = np.asarray(width_a, dtype=np.float64)
    b = np.asarray(width_b, dtype=np.float64)
    if np.any(~np.isfinite(a)) or np.any(~np.isfinite(b)):
        raise ValueError("Bead widths must be finite.")
    if np.any(a <= 0.0) or np.any(b <= 0.0):
        raise ValueError("Bead widths must be positive.")
    return np.maximum(0.0, 0.5 * (a + b) - overlap_value)


def rounded_cylinder_volume_mm3(
    width_mm: np.ndarray | float,
    height_mm: np.ndarray | float,
) -> np.ndarray:
    """Return the analytic volume of the nominal DDS rounded cylinder."""
    width, height = np.broadcast_arrays(
        np.asarray(width_mm, dtype=np.float64),
        np.asarray(height_mm, dtype=np.float64),
    )
    if np.any(~np.isfinite(width)) or np.any(~np.isfinite(height)):
        raise ValueError("Bead width and height must be finite.")
    if np.any(width <= 0.0) or np.any(height <= 0.0):
        raise ValueError("Bead width and height must be positive.")

    outer_radius = 0.5 * width
    rounding_radius = 0.5 * np.minimum(width, height)
    core_radius = outer_radius - rounding_radius
    core_height = height - 2.0 * rounding_radius

    return (
        np.pi * outer_radius**2 * core_height
        + 2.0 * np.pi * core_radius**2 * rounding_radius
        + np.pi**2 * core_radius * rounding_radius**2
        + (4.0 / 3.0) * np.pi * rounding_radius**3
    )


def scale_requested_volume_mm3(
    volume_mm3: np.ndarray | float,
    *,
    factor: float,
) -> np.ndarray:
    """Scale positive requested volumes before target YAML serialization."""
    values = np.asarray(volume_mm3, dtype=np.float64)
    scale = float(factor)
    if np.any(~np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError("Requested volumes must be finite and positive.")
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"Volume factor must be finite and > 0, got {factor}")
    return values * scale
