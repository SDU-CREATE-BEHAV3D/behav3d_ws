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
) -> PrintPointSet:
    """Select print points on polyline via scalar-min + spacing suppression."""
    k = int(count)
    if k < 0:
        raise ValueError(f"count must be >= 0, got {count}")
    spacing = max(0.0, float(min_spacing))

    unique_points, unique_lines, unique_to_old = _deduplicate_polyline_points(
        points=polyline_points,
        lines=polyline_lines,
        merge_tol=merge_tol,
    )
    if unique_points.shape[0] == 0 or unique_lines.shape[0] == 0 or k == 0:
        return PrintPointSet(
            points=np.zeros((0, 3), dtype=np.float64),
            scalar_values=np.zeros((0,), dtype=np.float64),
            polyline_indices=np.zeros((0,), dtype=np.int32),
            nearest_field_vertex_indices=np.zeros((0,), dtype=np.int32),
            requested_count=k,
            min_spacing=spacing,
            available_vertices=int(unique_points.shape[0]),
        )

    if point_valid_mask is not None:
        if point_valid_mask.shape[0] != polyline_points.shape[0]:
            raise ValueError("point_valid_mask must match polyline_points length")
        unique_valid = point_valid_mask[unique_to_old].astype(bool)
    else:
        unique_valid = np.ones(unique_points.shape[0], dtype=bool)

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
    return PrintPointSet(
        points=unique_points[selected_arr],
        scalar_values=scalar_vals[selected_arr],
        polyline_indices=selected_arr,
        nearest_field_vertex_indices=nearest_idx[selected_arr],
        requested_count=k,
        min_spacing=spacing,
        available_vertices=int(unique_points.shape[0]),
    )
