#!/usr/bin/env python3
"""Secondary rules for post-processing generated target segments."""

from __future__ import annotations

import numpy as np

from .print_targets import OrientedLineTargets


TARGET_NORMAL_FLIP_DOT_THRESHOLD = 0.5
TARGET_NORMAL_FLIP_BEAD_HEIGHT_FRACTION = 0.5


def _normalize_target_dirs(dirs: np.ndarray) -> np.ndarray:
    out = np.asarray(dirs, dtype=np.float64).copy()
    if out.ndim != 2 or out.shape[1] != 3:
        raise ValueError(f"dirs must have shape (N, 3), got {out.shape}")
    norms = np.linalg.norm(out, axis=1)
    valid = norms > 1e-12
    out[valid] /= norms[valid, None]
    out[~valid] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return out


def replace_low_continuity_target_segments(
    targets: OrientedLineTargets,
    *,
    bead_height_m: float,
    dot_threshold: float = TARGET_NORMAL_FLIP_DOT_THRESHOLD,
    bead_height_fraction: float = TARGET_NORMAL_FLIP_BEAD_HEIGHT_FRACTION,
) -> tuple[OrientedLineTargets, int]:
    """Replace segments whose target Z directions have a large discontinuity."""
    if targets.count == 0:
        return targets, 0

    starts = np.asarray(targets.start_points, dtype=np.float64).copy()
    ends = np.asarray(targets.end_points, dtype=np.float64).copy()
    start_z = _normalize_target_dirs(targets.start_z_dirs)
    end_z = _normalize_target_dirs(targets.end_z_dirs)

    dots = np.sum(start_z * end_z, axis=1)
    replace_mask = dots < float(dot_threshold)
    replace_count = int(np.count_nonzero(replace_mask))
    if replace_count == 0:
        return OrientedLineTargets(starts, ends, start_z, end_z), 0

    midpoints = 0.5 * (starts[replace_mask] + ends[replace_mask])
    average_dirs = start_z[replace_mask] + end_z[replace_mask]
    average_norms = np.linalg.norm(average_dirs, axis=1)
    replacement_dirs = np.zeros_like(average_dirs)

    average_valid = average_norms > 1e-12
    replacement_dirs[average_valid] = (
        average_dirs[average_valid] / average_norms[average_valid, None]
    )

    if np.any(~average_valid):
        fallback_dirs = ends[replace_mask][~average_valid] - starts[replace_mask][~average_valid]
        fallback_norms = np.linalg.norm(fallback_dirs, axis=1)
        fallback_valid = fallback_norms > 1e-12
        fallback_out = np.zeros_like(fallback_dirs)
        fallback_out[fallback_valid] = (
            fallback_dirs[fallback_valid] / fallback_norms[fallback_valid, None]
        )
        fallback_out[~fallback_valid] = end_z[replace_mask][~average_valid][~fallback_valid]
        replacement_dirs[~average_valid] = fallback_out

    replacement_dirs = _normalize_target_dirs(replacement_dirs)
    ends[replace_mask] = (
        midpoints
        + float(bead_height_fraction) * float(bead_height_m) * replacement_dirs
    )
    start_z[replace_mask] = replacement_dirs
    end_z[replace_mask] = replacement_dirs

    return OrientedLineTargets(starts, ends, start_z, end_z), replace_count


def remove_close_endpoint_targets(
    targets: OrientedLineTargets,
    *,
    min_distance_m: float,
) -> tuple[OrientedLineTargets, int]:
    """Remove later targets whose end point is too close to an earlier kept end."""
    if targets.count == 0:
        return targets, 0

    min_distance = max(0.0, float(min_distance_m))
    if min_distance <= 0.0:
        return targets, 0

    starts = np.asarray(targets.start_points, dtype=np.float64)
    ends = np.asarray(targets.end_points, dtype=np.float64)
    start_z = _normalize_target_dirs(targets.start_z_dirs)
    end_z = _normalize_target_dirs(targets.end_z_dirs)

    keep_indices: list[int] = []
    for idx, end in enumerate(ends):
        if not keep_indices:
            keep_indices.append(int(idx))
            continue
        kept_ends = ends[np.asarray(keep_indices, dtype=np.int32)]
        distances = np.linalg.norm(kept_ends - end, axis=1)
        if np.any(distances < min_distance):
            continue
        keep_indices.append(int(idx))

    removed = int(targets.count - len(keep_indices))
    if removed == 0:
        return OrientedLineTargets(starts.copy(), ends.copy(), start_z, end_z), 0

    keep = np.asarray(keep_indices, dtype=np.int32)
    return (
        OrientedLineTargets(
            start_points=starts[keep].copy(),
            end_points=ends[keep].copy(),
            start_z_dirs=start_z[keep].copy(),
            end_z_dirs=end_z[keep].copy(),
        ),
        removed,
    )


def apply_secondary_target_rules(
    targets: OrientedLineTargets,
    *,
    candidate_mode: str,
    bead_height_m: float,
    bead_separation_m: float,
    normal_continuity_rule: bool,
    endpoint_spacing_rule: bool,
) -> tuple[OrientedLineTargets, dict[str, int]]:
    """Apply optional post-generation target rules before visualization/YAML/DDS."""
    stats = {
        "normal_continuity_replaced": 0,
        "endpoint_spacing_removed": 0,
    }
    mode = str(candidate_mode).strip().lower()

    if bool(normal_continuity_rule) and mode == "gradient_lift":
        targets, replaced = replace_low_continuity_target_segments(
            targets,
            bead_height_m=float(bead_height_m),
        )
        stats["normal_continuity_replaced"] = int(replaced)

    if bool(endpoint_spacing_rule):
        targets, removed = remove_close_endpoint_targets(
            targets,
            min_distance_m=float(bead_separation_m),
        )
        stats["endpoint_spacing_removed"] = int(removed)

    return targets, stats
