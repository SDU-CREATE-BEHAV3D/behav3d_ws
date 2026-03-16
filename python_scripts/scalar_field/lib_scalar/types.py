#!/usr/bin/env python3
"""Typed data contracts shared across scalar-field pipeline stages.

These dataclasses make each stage explicit and composable:
- geometry in,
- scalar field out,
- pose/phi evaluation out.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class MeshData:
    """Prepared triangle-mesh arrays after load/compaction stage."""
    vertices: np.ndarray
    faces: np.ndarray
    dropped_vertices: int = 0


@dataclass(frozen=True)
class HeatField:
    """Heat scalar output with normalized values and summary statistics."""
    dist: np.ndarray
    norm: np.ndarray
    min_value: float
    max_value: float
    mean_value: float
    seed_info: str


@dataclass(frozen=True)
class PoseResult:
    """Result of field placement + phi evaluation against scan geometry."""
    offset_xyz: Tuple[float, float, float]
    field_vertices_world: np.ndarray
    z_scan: np.ndarray
    has_hit: np.ndarray
    phi: np.ndarray
    viable: np.ndarray
    base_dz: np.ndarray
    base_world_z: float
    hit_count: int
    viable_count: int
    viable_heat: Optional[float] = None
    tested: Optional[int] = None
    accepted: Optional[int] = None


@dataclass(frozen=True)
class PrintPointSet:
    """Selected print points over polyline and selection metadata."""
    points: np.ndarray
    scalar_values: np.ndarray
    polyline_indices: np.ndarray
    nearest_field_vertex_indices: np.ndarray
    requested_count: int
    min_spacing: float
    available_vertices: int
    augmented_vertices: int = 0


@dataclass(frozen=True)
class LiftedPrintPointSet:
    """Print points selected on a source polyline and lifted along +Z."""
    source_points: np.ndarray
    lifted_points: np.ndarray
    source_values: np.ndarray
    polyline_indices: np.ndarray
    nearest_field_vertex_indices: np.ndarray
    requested_count: int
    min_spacing: float
    available_vertices: int
    lift_height: float
    augmented_vertices: int = 0
