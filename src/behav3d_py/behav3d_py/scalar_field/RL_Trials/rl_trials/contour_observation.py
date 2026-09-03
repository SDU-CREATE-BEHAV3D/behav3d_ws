"""Deterministic contour samples for future policy observations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from .contour_parameterization import ContourParameterization, ContourSamples

FloatArray = npt.NDArray[np.float64]


def _readonly(values: npt.ArrayLike) -> FloatArray:
    result = np.array(values, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ContourObservation:
    """Geometry and goal fields aligned to canonical contour coordinates."""

    samples: ContourSamples
    heat: FloatArray
    widths_m: FloatArray

    def __post_init__(self) -> None:
        if not isinstance(self.samples, ContourSamples):
            raise TypeError("samples must be ContourSamples")
        heat = _readonly(self.heat).reshape(-1)
        widths = _readonly(self.widths_m).reshape(-1)
        if heat.shape[0] != self.samples.count or widths.shape[0] != self.samples.count:
            raise ValueError("observation fields must match the contour sample count")
        if np.any(widths <= 0.0):
            raise ValueError("sampled contour widths must be positive")
        object.__setattr__(self, "heat", heat)
        object.__setattr__(self, "widths_m", widths)


def sample_contour_observation(
    *,
    contour: ContourParameterization,
    contour_heat: npt.ArrayLike,
    contour_widths_m: npt.ArrayLike,
    sample_count: int,
) -> ContourObservation:
    """Build fixed-budget samples without generating candidate actions."""

    if not isinstance(contour, ContourParameterization):
        raise TypeError("contour must be a ContourParameterization")
    samples = contour.sample(sample_count)
    heat = contour.interpolate(contour_heat, samples, name="contour_heat")
    widths = contour.interpolate(
        contour_widths_m,
        samples,
        name="contour_widths_m",
    )
    return ContourObservation(samples=samples, heat=heat, widths_m=widths)
