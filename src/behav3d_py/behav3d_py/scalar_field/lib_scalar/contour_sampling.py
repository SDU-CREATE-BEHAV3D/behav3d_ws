"""Sample connected contours by arc length, independently of mesh edge spacing.

Original corners remain in the distance graph but are eligible for selection
only when they coincide with a sample. This preserves contour arc distances
without reintroducing mesh crossings into the candidate pool.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ContourSamples:
    points: np.ndarray
    lines: np.ndarray
    sample_mask: np.ndarray

    @property
    def source_points(self) -> np.ndarray:
        return self.points[self.sample_mask]


def resample_contour(
    points: np.ndarray,
    lines: np.ndarray,
    spacing: float,
    merge_tol: float = 1e-6,
) -> ContourSamples:
    """Sample every ``spacing`` meters and include each open contour endpoint.

    Unordered triangle segments are welded and traced into connected chains.
    Junctions terminate chains; closed contours use one seam sample without
    duplicating it. The final arc interval may be shorter than ``spacing``.
    """
    step = float(spacing)
    if not np.isfinite(step) or step <= 0:
        raise ValueError("Contour sample spacing must be finite and > 0.")
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    segs = np.asarray(lines, dtype=np.int64).reshape(-1, 2)
    if not np.all(np.isfinite(pts)):
        raise ValueError("Contour points must be finite.")
    if not len(segs):
        return ContourSamples(np.empty((0, 3)), np.empty((0, 2), dtype=np.int32), np.zeros(0, bool))
    if segs.min() < 0 or segs.max() >= len(pts):
        raise ValueError("Contour lines contain invalid indices.")
    tol = max(float(merge_tol), 1e-12)
    # np.unique sorts spatial keys, so tracing does not depend on triangle order.
    _, first, inverse = np.unique(
        np.round(pts / tol).astype(np.int64), axis=0,
        return_index=True, return_inverse=True,
    )
    vertices = pts[first]
    edges = {tuple(sorted((int(inverse[a]), int(inverse[b])))) for a, b in segs
             if inverse[a] != inverse[b]}
    neighbors = [set() for _ in vertices]
    for a, b in edges:
        neighbors[a].add(b)
        neighbors[b].add(a)
    remaining = edges.copy()

    def trace(start: int, neighbor: int) -> list[int]:
        chain = [start]
        previous, current = start, neighbor
        while True:
            remaining.remove(tuple(sorted((previous, current))))
            chain.append(current)
            if current == start or len(neighbors[current]) != 2:
                return chain
            following = next(node for node in neighbors[current] if node != previous)
            if tuple(sorted((current, following))) not in remaining:
                return chain
            previous, current = current, following

    chains = []
    for start, adjacent in enumerate(neighbors):
        if len(adjacent) == 2:
            continue
        for neighbor in sorted(adjacent):
            if tuple(sorted((start, neighbor))) in remaining:
                chains.append(trace(start, neighbor))
    while remaining:
        start, neighbor = min(remaining)
        chains.append(trace(start, neighbor))

    graph_points = list(vertices.copy())
    sample_mask = [False] * len(vertices)
    graph_lines = []
    for chain in chains:
        coords = vertices[chain]
        lengths = np.linalg.norm(np.diff(coords, axis=0), axis=1)
        cumulative = np.r_[0.0, np.cumsum(lengths)]
        total = float(cumulative[-1])
        positions = np.arange(int(np.floor(total / step)) + 1, dtype=float) * step
        # Include the exact endpoint, avoiding a near-duplicate final sample.
        if total - positions[-1] > 1e-10:
            positions = np.r_[positions, total]
        else:
            positions[-1] = total
        if chain[0] == chain[-1]:
            positions = positions[:-1]
        inserted = [[] for _ in lengths]
        for distance in positions:
            edge = min(int(np.searchsorted(cumulative, distance, side="right") - 1), len(lengths) - 1)
            fraction = float(np.clip((distance - cumulative[edge]) / lengths[edge], 0, 1))
            if fraction * lengths[edge] <= 1e-10:
                index = chain[edge]
            elif (1 - fraction) * lengths[edge] <= 1e-10:
                index = chain[edge + 1]
            else:
                index = len(graph_points)
                graph_points.append(coords[edge] + fraction * (coords[edge + 1] - coords[edge]))
                sample_mask.append(False)
                inserted[edge].append(index)
            sample_mask[index] = True
        for edge, additions in enumerate(inserted):
            nodes = [chain[edge], *additions, chain[edge + 1]]
            graph_lines.extend(zip(nodes[:-1], nodes[1:]))
    return ContourSamples(
        np.asarray(graph_points, dtype=np.float64).reshape(-1, 3),
        np.asarray(graph_lines, dtype=np.int32).reshape(-1, 2),
        np.asarray(sample_mask, dtype=bool),
    )
