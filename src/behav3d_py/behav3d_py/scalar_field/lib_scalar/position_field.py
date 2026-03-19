#!/usr/bin/env python3
"""Position field mesh over scan mesh with constrained XY search.

This module builds feasible pose candidates and chooses a local best candidate.

Base rule enforced here:
    base_world_z - z_scan <= 0
with:
    base_world_z = min(field_vertices_world[:, 2])

In practice:
- Candidates are sampled in XY.
- For each candidate, Z is solved from the base rule.
- Viability metrics are computed from phi.

The global score of your full loop can still be defined outside this module.
"""

from __future__ import annotations

import numpy as np

from .compute_phi_mask import query_scan_z_with_vertical_rays
from .types import PoseResult


def make_axis_samples(vmin: float, vmax: float, step: float) -> np.ndarray:
    """Create inclusive 1D grid samples for search axis."""
    if step <= 0.0:
        raise ValueError(f"Search step must be > 0, got {step}")
    lo = float(min(vmin, vmax))
    hi = float(max(vmin, vmax))
    count = int(np.floor((hi - lo) / step)) + 1
    values = lo + step * np.arange(max(count, 1), dtype=np.float64)
    if values.size == 0:
        values = np.array([lo], dtype=np.float64)
    if values[-1] < (hi - 0.5 * step):
        values = np.append(values, hi)
    return values


def default_xy_search_bounds(
    scan_vertices: np.ndarray,
    field_vertices_scaled: np.ndarray,
    margin_x: float,
    margin_y: float,
) -> tuple[float, float, float, float]:
    """Compute conservative XY search bounds from scan/field bounding boxes."""
    scan_min = np.min(scan_vertices, axis=0)
    scan_max = np.max(scan_vertices, axis=0)
    field_min = np.min(field_vertices_scaled, axis=0)
    field_max = np.max(field_vertices_scaled, axis=0)

    x_min = float(scan_min[0] - field_max[0] - margin_x)
    x_max = float(scan_max[0] - field_min[0] + margin_x)
    y_min = float(scan_min[1] - field_max[1] - margin_y)
    y_max = float(scan_max[1] - field_min[1] + margin_y)
    return x_min, x_max, y_min, y_max


def position_field(
    scene,
    z_top: float,
    field_vertices_scaled: np.ndarray,
    heat_norm: np.ndarray,
    x_values: np.ndarray,
    y_values: np.ndarray,
    clearance: float,
    iso_level: float,
    base_z_offset: float,
    require_full_hit: bool,
    verbose: bool,
) -> PoseResult:
    """Search XY candidates and compute a feasible Z from base min-z.

    Candidate generation:
    - Each candidate is an XY offset `(ox, oy)` from the search grid.
    - We raycast scan heights `z_scan` at those XY positions.
    - If `require_full_hit=True`, every field point must hit scan geometry.

    Base-feasible Z solve:
    - `base_local_z = min(field_vertices_scaled[:, 2])`
    - `z_offset = min(z_scan_hit - base_local_z) - base_z_offset`
    - This guarantees base non-penetration on hit points.

    Local ranking used in this method:
    - `phi = z_field - z_scan - clearance`
    - `viable = phi > iso_level`
    - sort key: `(viable_count, viable_heat, hit_count, z_offset)`

    Note:
    - This is only the local pose-search ranking.
    - If you use a pipeline-wide objective, maximize that outside this method.
    """
    n = field_vertices_scaled.shape[0]
    base_local_z = float(np.min(field_vertices_scaled[:, 2]))
    best_result: PoseResult | None = None
    best_key = None
    tested = 0
    accepted = 0

    for ox in x_values:
        for oy in y_values:
            tested += 1

            probe = field_vertices_scaled.copy()
            probe[:, 0] += float(ox)
            probe[:, 1] += float(oy)
            z_scan, has_hit = query_scan_z_with_vertical_rays(scene, probe, z_top=z_top)
            hit_count = int(np.count_nonzero(has_hit))
            if hit_count == 0:
                continue
            if require_full_hit and hit_count < n:
                continue

            # Highest Z that keeps the base under (or touching) scan on all hit points.
            z_offset = float(np.min(z_scan[has_hit] - base_local_z) - float(base_z_offset))
            field_world = probe.copy()
            field_world[:, 2] = field_vertices_scaled[:, 2] + z_offset

            phi = np.full(n, -np.inf, dtype=np.float64)
            phi[has_hit] = field_world[has_hit, 2] - z_scan[has_hit] - float(clearance)
            viable = (phi > float(iso_level)) & has_hit
            viable_count = int(np.count_nonzero(viable))
            viable_heat = float(np.sum(heat_norm[viable]))

            base_world_z = float(base_local_z + z_offset)
            base_dz = np.full(n, np.inf, dtype=np.float64)
            base_dz[has_hit] = base_world_z - z_scan[has_hit]
            base_ok = bool(np.all(base_dz[has_hit] <= 1e-9))
            if not base_ok:
                continue
            accepted += 1

            # Local ranking inside this search routine.
            key = (viable_count, viable_heat, hit_count, z_offset)
            if best_key is None or key > best_key:
                best_key = key
                best_result = PoseResult(
                    offset_xyz=(float(ox), float(oy), z_offset),
                    field_vertices_world=field_world,
                    z_scan=z_scan,
                    has_hit=has_hit,
                    phi=phi,
                    viable=viable,
                    base_dz=base_dz,
                    base_world_z=base_world_z,
                    hit_count=hit_count,
                    viable_count=viable_count,
                    viable_heat=viable_heat,
                    tested=tested,
                    accepted=accepted,
                )
                if verbose:
                    print(
                        "pose_search update: "
                        f"offset=({ox:.6f},{oy:.6f},{z_offset:.6f}) "
                        f"viable={viable_count} hit={hit_count}/{n} heat={viable_heat:.6f}"
                    )

    if best_result is None:
        raise RuntimeError(
            "Pose search found no feasible pose. "
            "Try wider XY bounds, smaller step, or disable full-hit requirement."
        )

    return PoseResult(
        offset_xyz=best_result.offset_xyz,
        field_vertices_world=best_result.field_vertices_world,
        z_scan=best_result.z_scan,
        has_hit=best_result.has_hit,
        phi=best_result.phi,
        viable=best_result.viable,
        base_dz=best_result.base_dz,
        base_world_z=best_result.base_world_z,
        hit_count=best_result.hit_count,
        viable_count=best_result.viable_count,
        viable_heat=best_result.viable_heat,
        tested=tested,
        accepted=accepted,
    )
