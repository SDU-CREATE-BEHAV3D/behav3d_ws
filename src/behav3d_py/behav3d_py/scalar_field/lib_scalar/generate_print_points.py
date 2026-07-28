#!/usr/bin/env python3
"""Generate print points from an offset polyline using scalar-field minima.

Selection rule:
1) Pick the polyline vertex with minimum scalar value.
2) Remove polyline vertices within min_spacing of that selected point
   (distance measured along the polyline graph).
3) Repeat until K points are selected or no valid vertices remain.
"""

from __future__ import annotations

import heapq

import numpy as np

from .agent_walk import AgentWalkConfig, run_agent_walk
from .geometry import (
    project_points_to_surface,
    sample_tangent_axes_on_surface_from_scalar,
    sample_vertex_scalar_on_surface,
)
from .types import PrintPointSet


def _deduplicate_polyline_points(
    points: np.ndarray,
    lines: np.ndarray,
    merge_tol: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if points.size == 0 or lines.size == 0:
        return (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 2), dtype=np.int32),
            np.zeros((0,), dtype=np.int32),
        )

    tol = max(float(merge_tol), 1e-12)
    keys = np.round(points / tol).astype(np.int64)

    key_to_uid: dict[tuple[int, int, int], int] = {}
    uid_points: list[np.ndarray] = []
    uid_first_old: list[int] = []
    old_to_uid = np.full(points.shape[0], -1, dtype=np.int32)

    for i in range(points.shape[0]):
        k = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
        uid = key_to_uid.get(k)
        if uid is None:
            uid = len(uid_points)
            key_to_uid[k] = uid
            uid_points.append(points[i])
            uid_first_old.append(i)
        old_to_uid[i] = int(uid)

    edge_set: set[tuple[int, int]] = set()
    for e in lines:
        a = int(old_to_uid[int(e[0])])
        b = int(old_to_uid[int(e[1])])
        if a == b:
            continue
        u, v = (a, b) if a < b else (b, a)
        edge_set.add((u, v))

    if not uid_points or not edge_set:
        return (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 2), dtype=np.int32),
            np.zeros((0,), dtype=np.int32),
        )
    return (
        np.vstack(uid_points),
        np.asarray(sorted(edge_set), dtype=np.int32),
        np.asarray(uid_first_old, dtype=np.int32),
    )


def _build_adjacency(
    points: np.ndarray,
    lines: np.ndarray,
) -> list[list[tuple[int, float]]]:
    n = points.shape[0]
    adj: list[list[tuple[int, float]]] = [[] for _ in range(n)]
    for e in lines:
        i = int(e[0])
        j = int(e[1])
        w = float(np.linalg.norm(points[i] - points[j]))
        if not np.isfinite(w):
            continue
        adj[i].append((j, w))
        adj[j].append((i, w))
    return adj


def _augment_with_endpoint_z_projection(
    points: np.ndarray,
    lines: np.ndarray,
    valid_mask: np.ndarray,
    extra_points: np.ndarray | None,
    merge_tol: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Add extra bridge points and connect each to nearest existing graph vertex."""
    if extra_points is None or extra_points.size == 0:
        return points, lines, valid_mask, 0

    points_list = [points[i].copy() for i in range(points.shape[0])]
    valid_list = [bool(v) for v in valid_mask.tolist()]
    lines_list = [[int(e[0]), int(e[1])] for e in lines.tolist()]
    edge_set: set[tuple[int, int]] = set()
    for e in lines_list:
        u, v = (e[0], e[1]) if e[0] < e[1] else (e[1], e[0])
        edge_set.add((u, v))

    tol2 = max(float(merge_tol), 1e-12) ** 2
    added = 0
    for i in range(extra_points.shape[0]):
        p_extra = extra_points[i]
        if not np.all(np.isfinite(p_extra)):
            continue

        if len(points_list) > 0:
            existing = np.asarray(points_list, dtype=np.float64)
            d2_existing = np.sum((existing - p_extra) ** 2, axis=1)
            near_idx = int(np.argmin(d2_existing))
        else:
            d2_existing = np.array([], dtype=np.float64)
            near_idx = -1

        if d2_existing.size > 0 and float(d2_existing[near_idx]) <= tol2:
            new_idx = near_idx
            valid_list[new_idx] = True
        else:
            new_idx = len(points_list)
            points_list.append(p_extra.copy())
            valid_list.append(True)
            added += 1

        if new_idx < 0:
            continue
        if len(points_list) <= 1:
            continue
        if new_idx < len(points_list) - 1:
            # Point already existed; keep graph unchanged.
            continue

        # Connect new bridge node to nearest previous graph vertex.
        existing_prev = np.asarray(points_list[:-1], dtype=np.float64)
        d2_prev = np.sum((existing_prev - points_list[new_idx]) ** 2, axis=1)
        anchor = int(np.argmin(d2_prev))
        u, v = (anchor, new_idx) if anchor < new_idx else (new_idx, anchor)
        if (u, v) in edge_set:
            continue
        edge_set.add((u, v))
        lines_list.append([u, v])

    return (
        np.asarray(points_list, dtype=np.float64),
        np.asarray(lines_list, dtype=np.int32),
        np.asarray(valid_list, dtype=bool),
        added,
    )


def _sample_scalar_nearest_vertex(
    sample_points: np.ndarray,
    field_vertices_world: np.ndarray,
    field_scalar: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if sample_points.shape[0] == 0:
        return np.zeros((0,), dtype=np.float64), np.zeros((0,), dtype=np.int32)
    if field_vertices_world.shape[0] != field_scalar.shape[0]:
        raise ValueError("field_vertices_world and field_scalar must have same length")

    nearest_idx = np.zeros((sample_points.shape[0],), dtype=np.int32)
    scalar = np.zeros((sample_points.shape[0],), dtype=np.float64)
    for i in range(sample_points.shape[0]):
        d2 = np.sum((field_vertices_world - sample_points[i]) ** 2, axis=1)
        j = int(np.argmin(d2))
        nearest_idx[i] = j
        scalar[i] = float(field_scalar[j])
    return scalar, nearest_idx


def _nearest_indices(
    sample_points: np.ndarray,
    field_vertices_world: np.ndarray,
) -> np.ndarray:
    if sample_points.shape[0] == 0:
        return np.zeros((0,), dtype=np.int32)
    idx = np.zeros((sample_points.shape[0],), dtype=np.int32)
    for i in range(sample_points.shape[0]):
        d2 = np.sum((field_vertices_world - sample_points[i]) ** 2, axis=1)
        idx[i] = int(np.argmin(d2))
    return idx


def _dijkstra_with_limit(
    adj: list[list[tuple[int, float]]],
    source: int,
    limit: float,
) -> dict[int, float]:
    dist: dict[int, float] = {int(source): 0.0}
    pq: list[tuple[float, int]] = [(0.0, int(source))]
    lim = float(limit)

    while pq:
        d, u = heapq.heappop(pq)
        if d > lim:
            break
        if d > dist.get(u, np.inf):
            continue
        for v, w in adj[u]:
            nd = d + w
            if nd > lim:
                continue
            if nd < dist.get(v, np.inf):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return dist


def _gradient_lift_points(
    source_points: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    field_scalar: np.ndarray,
    *,
    lift_height: float,
    tangent_sign: float,
) -> np.ndarray:
    if source_points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)

    h = float(lift_height)
    if h < 0.0:
        raise ValueError(f"lift_height must be >= 0, got {lift_height}")
    if h <= 0.0:
        return source_points.copy()

    src_surface = project_points_to_surface(source_points, field_vertices_world, field_faces)
    t_dir, _b_dir, _n_dir = sample_tangent_axes_on_surface_from_scalar(
        query_points=src_surface,
        mesh_vertices=field_vertices_world,
        mesh_faces=field_faces,
        vertex_scalar=field_scalar,
        tangent_sign=float(tangent_sign),
    )
    return source_points + h * t_dir


def generate_print_points(
    polyline_points: np.ndarray,
    polyline_lines: np.ndarray,
    field_vertices_world: np.ndarray,
    field_scalar: np.ndarray,
    *,
    count: int = 7,
    min_spacing: float = 0.016,
    merge_tol: float = 1e-6,
    point_valid_mask: np.ndarray | None = None,
    extra_points: np.ndarray | None = None,
    candidate_mode: str = "polyline",
    lift_height: float = 0.0,
    field_faces: np.ndarray | None = None,
    walk_distance: float = 0.0,
    walk_step: float = 1e-3,
    walk_max_steps: int = 32,
    walk_tangent_sign: float = 1.0,
    walk_start_fraction: float = 0.25,
    clamp_to_cone: bool = False,
    cone_max_tilt_deg: float = 45.0,
    agent_phi_scalar: np.ndarray | None = None,
) -> PrintPointSet:
    """Select print points and optionally post-process candidate locations.

    Base selection:
    - pick scalar minima on polyline with spacing suppression.

    Candidate modes:
    - `polyline`: output selected polyline points.
    - `z_lift`: output selected points with +Z `lift_height`.
    - `gradient_lift`: output selected points displaced by `lift_height`
      in local tangent gradient direction (`walk_tangent_sign` controls sign).
    - `gradient_walk`: walk on field surface along scalar tangent direction
      until euclidean displacement from source reaches `walk_distance`
      (or `walk_max_steps` is reached).
      The simple agent walk can reject final points with `phi < 4 mm` when
      `agent_phi_scalar` is passed. Final endpoint spacing is handled by the
      shared secondary target rules after target construction.
      It also exposes a projected segment start at `walk_start_fraction`
      along `source -> output`.
      Optional clamp:
      - if `clamp_to_cone=True`, the final displacement `source -> output`
        is clamped to a cone around world `+Z` (semi-angle `cone_max_tilt_deg`)
        and re-projected to the field surface.
    """
    k = int(count)
    if k < 0:
        raise ValueError(f"count must be >= 0, got {count}")
    spacing = max(0.0, float(min_spacing))
    mode = str(candidate_mode).strip().lower()
    if mode not in ("polyline", "z_lift", "gradient_lift", "gradient_walk"):
        raise ValueError(
            f"Unknown candidate_mode: {candidate_mode}. "
            "Use 'polyline', 'z_lift', 'gradient_lift', or 'gradient_walk'."
        )

    unique_points, unique_lines, unique_to_old = _deduplicate_polyline_points(
        points=polyline_points,
        lines=polyline_lines,
        merge_tol=merge_tol,
    )
    if unique_points.shape[0] == 0 or k == 0:
        return PrintPointSet(
            points=np.zeros((0, 3), dtype=np.float64),
            scalar_values=np.zeros((0,), dtype=np.float64),
            polyline_indices=np.zeros((0,), dtype=np.int32),
            nearest_field_vertex_indices=np.zeros((0,), dtype=np.int32),
            requested_count=k,
            min_spacing=spacing,
            available_vertices=int(unique_points.shape[0]),
            augmented_vertices=0,
            source_points=np.zeros((0, 3), dtype=np.float64),
            surface_points=np.zeros((0, 3), dtype=np.float64),
            segment_start_points=np.zeros((0, 3), dtype=np.float64),
        )

    if point_valid_mask is not None:
        if point_valid_mask.shape[0] != polyline_points.shape[0]:
            raise ValueError("point_valid_mask must match polyline_points length")
        unique_valid = point_valid_mask[unique_to_old].astype(bool)
    else:
        unique_valid = np.ones(unique_points.shape[0], dtype=bool)

    unique_points, unique_lines, unique_valid, added_vertices = _augment_with_endpoint_z_projection(
        points=unique_points,
        lines=unique_lines,
        valid_mask=unique_valid,
        extra_points=extra_points,
        merge_tol=float(merge_tol),
    )

    use_surface_sampling = (
        field_faces is not None
        and field_vertices_world.ndim == 2
        and field_vertices_world.shape[1] == 3
        and field_scalar.ndim == 1
        and field_scalar.shape[0] == field_vertices_world.shape[0]
    )
    if use_surface_sampling:
        scalar_vals = sample_vertex_scalar_on_surface(
            query_points=unique_points,
            mesh_vertices=field_vertices_world,
            mesh_faces=np.asarray(field_faces, dtype=np.int32),
            vertex_scalar=field_scalar,
        )
        nearest_idx = _nearest_indices(unique_points, field_vertices_world)
    else:
        scalar_vals, nearest_idx = _sample_scalar_nearest_vertex(
            sample_points=unique_points,
            field_vertices_world=field_vertices_world,
            field_scalar=field_scalar,
        )
    adj = _build_adjacency(unique_points, unique_lines)

    valid = unique_valid.copy()
    selected: list[int] = []

    while len(selected) < k:
        valid_idx = np.flatnonzero(valid)
        if valid_idx.size == 0:
            break

        local_min_i = int(valid_idx[np.argmin(scalar_vals[valid_idx])])
        selected.append(local_min_i)
        valid[local_min_i] = False

        if spacing > 0.0:
            blocked = _dijkstra_with_limit(adj, local_min_i, limit=spacing)
            for idx in blocked.keys():
                valid[int(idx)] = False

    selected_arr = np.asarray(selected, dtype=np.int32)
    source_points = unique_points[selected_arr].copy()
    out_points = source_points.copy()
    surface_points = source_points.copy()
    segment_start_points = source_points.copy()
    if mode == "z_lift":
        h = float(lift_height)
        if h < 0.0:
            raise ValueError(f"lift_height must be >= 0, got {lift_height}")
        out_points[:, 2] += h
    elif mode == "gradient_lift":
        if field_faces is None:
            raise ValueError("field_faces is required for candidate_mode='gradient_lift'")
        out_points = _gradient_lift_points(
            source_points=source_points,
            field_vertices_world=field_vertices_world,
            field_faces=np.asarray(field_faces, dtype=np.int32),
            field_scalar=field_scalar,
            lift_height=float(lift_height),
            tangent_sign=float(walk_tangent_sign),
        )
        surface_points = out_points.copy()
    elif mode == "gradient_walk":
        if field_faces is None:
            raise ValueError("field_faces is required for candidate_mode='gradient_walk'")
        agent_result = run_agent_walk(
            source_points=source_points,
            field_vertices_world=field_vertices_world,
            field_faces=np.asarray(field_faces, dtype=np.int32),
            field_scalar=field_scalar,
            config=AgentWalkConfig(
                target_distance=float(walk_distance),
                step_size=float(walk_step),
                max_steps=int(walk_max_steps),
                tangent_sign=float(walk_tangent_sign),
                segment_start_fraction=float(walk_start_fraction),
                clamp_to_cone=bool(clamp_to_cone),
                cone_max_tilt_deg=float(cone_max_tilt_deg),
            ),
            phi_scalar=agent_phi_scalar,
        )
        selected_arr = selected_arr[agent_result.accepted_indices]
        source_points = agent_result.source_points
        segment_start_points = agent_result.segment_start_points
        out_points = agent_result.points
        surface_points = out_points.copy()

    return PrintPointSet(
        points=out_points,
        scalar_values=scalar_vals[selected_arr],
        polyline_indices=selected_arr,
        nearest_field_vertex_indices=nearest_idx[selected_arr],
        requested_count=k,
        min_spacing=spacing,
        available_vertices=int(unique_points.shape[0]),
        augmented_vertices=int(added_vertices),
        source_points=source_points,
        surface_points=surface_points,
        segment_start_points=segment_start_points,
    )
