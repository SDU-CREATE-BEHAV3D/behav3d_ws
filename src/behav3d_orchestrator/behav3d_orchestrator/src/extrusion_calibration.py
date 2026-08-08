#!/usr/bin/env python3
"""Extruder volume-to-step calibration helpers."""

from __future__ import annotations

import math


PISTON_DIAMETER_MM = 100.0
PISTON_REFERENCE_TRAVEL_MM = 10.0
PISTON_REFERENCE_STEPS = 64_000
MAX_PRINT_STEPS = (1 << 32) - 1

PISTON_AREA_MM2 = math.pi * (0.5 * PISTON_DIAMETER_MM) ** 2
THEORETICAL_STEPS_PER_MM3 = PISTON_REFERENCE_STEPS / (
    PISTON_AREA_MM2 * PISTON_REFERENCE_TRAVEL_MM
)


def volume_mm3_to_steps(
    volume_mm3: float,
    *,
    steps_per_mm3: float = THEORETICAL_STEPS_PER_MM3,
) -> int:
    """Convert requested material volume to a valid PrintSteps command."""
    volume = float(volume_mm3)
    calibration = float(steps_per_mm3)
    if not math.isfinite(volume) or volume <= 0.0:
        raise ValueError(f"volume_mm3 must be finite and > 0, got {volume_mm3}")
    if not math.isfinite(calibration) or calibration <= 0.0:
        raise ValueError(
            f"steps_per_mm3 must be finite and > 0, got {steps_per_mm3}"
        )

    steps = max(1, int(math.floor(volume * calibration + 0.5)))
    if steps > MAX_PRINT_STEPS:
        raise ValueError(
            f"Converted step command exceeds uint32: volume_mm3={volume:.6f} "
            f"steps={steps}"
        )
    return steps


def compensated_forward_steps(
    bead_steps: int,
    *,
    retract_steps: int = 0,
    retract_enabled: bool = True,
) -> int:
    """Return the forward command that delivers ``bead_steps`` after retracting."""
    net_steps = int(bead_steps)
    compensation = int(retract_steps) if bool(retract_enabled) else 0
    if net_steps <= 0:
        raise ValueError(f"bead_steps must be > 0, got {bead_steps}")
    if compensation < 0:
        raise ValueError(f"retract_steps must be >= 0, got {retract_steps}")

    forward_steps = net_steps + compensation
    if forward_steps > MAX_PRINT_STEPS:
        raise ValueError(
            "Compensated forward step command exceeds uint32: "
            f"bead_steps={net_steps} retract_steps={compensation} "
            f"forward_steps={forward_steps}"
        )
    return forward_steps
