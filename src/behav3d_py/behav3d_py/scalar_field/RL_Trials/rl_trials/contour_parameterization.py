"""Canonical 3D contour addressing shared by policy decoding and observation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]
BoolArray = npt.NDArray[np.bool_]

DEFAULT_WELD_TOLERANCE_M = 1e-9


def _readonly(values: npt.ArrayLike, dtype: npt.DTypeLike) -> np.ndarray:
    result = np.array(values, dtype=dtype, copy=True)
    result.setflags(write=False)
    return result


def _as_points(values: npt.ArrayLike) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError(f"contour_points must have shape (N, 3), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError("contour_points must contain only finite values")
    return result


def _as_lines(values: npt.ArrayLike, point_count: int) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 2:
        raise ValueError(f"contour_lines must have shape (M, 2), got {raw.shape}")
    if not np.issubdtype(raw.dtype, np.integer) and not np.all(
        np.equal(raw, np.floor(raw))
    ):
        raise ValueError("contour_lines must contain integer indices")
    result = raw.astype(np.int32, copy=False)
    if result.size and (np.any(result < 0) or np.any(result >= point_count)):
        raise ValueError("contour_lines contains an invalid point index")
    return result


@dataclass(frozen=True)
class ContourLocation:
    """One exact location addressed on a canonical contour parameterization."""

    source_coord: float
    arc_fraction: float
    arc_length_m: float
    point: FloatArray
    segment_index: int
    segment_fraction: float
    component_index: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "point", _readonly(self.point, np.float64))


@dataclass(frozen=True)
class ContourSamples:
    """Fixed-size deterministic samples suitable for a policy observation."""

    source_coord: FloatArray
    arc_fraction: FloatArray
    arc_length_m: FloatArray
    points: FloatArray
    segment_indices: IntArray
    segment_fractions: FloatArray
    component_indices: IntArray
    valid_mask: BoolArray

    def __post_init__(self) -> None:
        for name, dtype in (
            ("source_coord", np.float64),
            ("arc_fraction", np.float64),
            ("arc_length_m", np.float64),
            ("points", np.float64),
            ("segment_indices", np.int32),
            ("segment_fractions", np.float64),
            ("component_indices", np.int32),
            ("valid_mask", np.bool_),
        ):
            object.__setattr__(self, name, _readonly(getattr(self, name), dtype))

    @property
    def count(self) -> int:
        return int(self.source_coord.shape[0])


@dataclass(frozen=True)
class ContourParameterization:
    """Canonical cumulative arc-length map over 3D contour components."""

    points: FloatArray
    lines: IntArray
    ordered_segment_indices: IntArray
    ordered_forward: BoolArray
    ordered_lengths_m: FloatArray
    ordered_component_indices: IntArray
    component_lengths_m: FloatArray
    weld_tolerance_m: float

    def __post_init__(self) -> None:
        for name, dtype in (
            ("points", np.float64),
            ("lines", np.int32),
            ("ordered_segment_indices", np.int32),
            ("ordered_forward", np.bool_),
            ("ordered_lengths_m", np.float64),
            ("ordered_component_indices", np.int32),
            ("component_lengths_m", np.float64),
        ):
            object.__setattr__(self, name, _readonly(getattr(self, name), dtype))

    @classmethod
    def from_segments(
        cls,
        contour_points: npt.ArrayLike,
        contour_lines: npt.ArrayLike,
        *,
        weld_tolerance_m: float = DEFAULT_WELD_TOLERANCE_M,
    ) -> ContourParameterization:
        """Build a face-order-independent map from an unordered segment soup."""

        points = _as_points(contour_points)
        lines = _as_lines(contour_lines, points.shape[0])
        if lines.shape[0] == 0:
            raise ValueError("cannot parameterize an empty phi contour")
        tolerance = float(weld_tolerance_m)
        if not np.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("weld_tolerance_m must be finite and > 0")

        starts = points[lines[:, 0]]
        ends = points[lines[:, 1]]
        lengths = np.linalg.norm(ends - starts, axis=1)
        quantized = np.rint(points / tolerance).astype(np.int64)
        _, node_ids = np.unique(quantized, axis=0, return_inverse=True)
        edge_nodes = node_ids[lines]
        usable = (lengths > 1e-12) & (edge_nodes[:, 0] != edge_nodes[:, 1])
        if not np.any(usable):
            raise ValueError("phi contour contains no non-degenerate segments")

        original_indices = np.flatnonzero(usable)
        usable_nodes = edge_nodes[usable]
        usable_lengths = lengths[usable]
        node_count = int(np.max(usable_nodes)) + 1
        adjacency: list[list[int]] = [[] for _ in range(node_count)]
        for edge_index, (node_a, node_b) in enumerate(usable_nodes):
            adjacency[int(node_a)].append(edge_index)
            adjacency[int(node_b)].append(edge_index)

        if any(len(incident) > 2 for incident in adjacency):
            raise ValueError(
                "phi contour contains a branch; scalar arc-length addressing "
                "requires non-branching components"
            )

        unvisited = set(range(usable_nodes.shape[0]))
        components: list[tuple[int, list[int], list[bool], list[float]]] = []
        while unvisited:
            seed_edge = min(unvisited)
            stack = [seed_edge]
            component_edges: set[int] = set()
            component_nodes: set[int] = set()
            while stack:
                edge_index = stack.pop()
                if edge_index in component_edges:
                    continue
                component_edges.add(edge_index)
                node_a, node_b = usable_nodes[edge_index]
                component_nodes.update((int(node_a), int(node_b)))
                for node in (int(node_a), int(node_b)):
                    stack.extend(
                        adjacent
                        for adjacent in adjacency[node]
                        if adjacent not in component_edges
                    )

            endpoints = [node for node in component_nodes if len(adjacency[node]) == 1]
            if len(endpoints) not in (0, 2):
                raise ValueError(
                    "phi contour component is neither an open path nor a loop"
                )
            current_node = min(endpoints) if endpoints else min(component_nodes)
            unused_component_edges = set(component_edges)
            ordered_indices: list[int] = []
            ordered_forward: list[bool] = []
            ordered_lengths: list[float] = []

            while unused_component_edges:
                candidates = [
                    edge_index
                    for edge_index in adjacency[current_node]
                    if edge_index in unused_component_edges
                ]
                if not candidates:
                    raise ValueError("failed to trace a continuous contour component")
                ranked_candidates: list[tuple[int, int]] = []
                for candidate in candidates:
                    candidate_a, candidate_b = usable_nodes[candidate]
                    candidate_next = (
                        int(candidate_b)
                        if int(candidate_a) == current_node
                        else int(candidate_a)
                    )
                    ranked_candidates.append((candidate_next, candidate))
                edge_index = min(ranked_candidates)[1]
                node_a, node_b = usable_nodes[edge_index]
                forward = int(node_a) == current_node
                ordered_indices.append(int(original_indices[edge_index]))
                ordered_forward.append(forward)
                ordered_lengths.append(float(usable_lengths[edge_index]))
                unused_component_edges.remove(edge_index)
                unvisited.remove(edge_index)
                current_node = int(node_b) if forward else int(node_a)

            components.append(
                (
                    min(component_nodes),
                    ordered_indices,
                    ordered_forward,
                    ordered_lengths,
                )
            )

        components.sort(key=lambda component: component[0])
        segment_indices: list[int] = []
        segment_forward: list[bool] = []
        segment_lengths: list[float] = []
        segment_components: list[int] = []
        component_lengths: list[float] = []
        for component_index, (_, indices, forward, values) in enumerate(components):
            segment_indices.extend(indices)
            segment_forward.extend(forward)
            segment_lengths.extend(values)
            segment_components.extend([component_index] * len(indices))
            component_lengths.append(float(np.sum(values)))

        return cls(
            points=points,
            lines=lines,
            ordered_segment_indices=np.asarray(segment_indices, dtype=np.int32),
            ordered_forward=np.asarray(segment_forward, dtype=np.bool_),
            ordered_lengths_m=np.asarray(segment_lengths, dtype=np.float64),
            ordered_component_indices=np.asarray(segment_components, dtype=np.int32),
            component_lengths_m=np.asarray(component_lengths, dtype=np.float64),
            weld_tolerance_m=tolerance,
        )

    @property
    def total_length_m(self) -> float:
        return float(np.cumsum(self.ordered_lengths_m)[-1])

    @property
    def component_count(self) -> int:
        return int(self.component_lengths_m.shape[0])

    def _locate_distance(self, distance_m: float) -> ContourLocation:
        cumulative = np.cumsum(self.ordered_lengths_m)
        total_length = float(cumulative[-1])
        requested_distance = float(distance_m)
        if (
            not np.isfinite(requested_distance)
            or requested_distance < 0.0
            or requested_distance > total_length
        ):
            raise ValueError("arc distance must lie in [0, total_length_m]")
        selection_distance = requested_distance

        boundary_tolerance = 8.0 * np.finfo(np.float64).eps * max(1.0, total_length)
        boundary_offsets = np.abs(cumulative - selection_distance)
        nearest_boundary = int(np.argmin(boundary_offsets))
        if boundary_offsets[nearest_boundary] <= boundary_tolerance:
            selection_distance = float(cumulative[nearest_boundary])

        if selection_distance >= total_length:
            ordered_index = self.ordered_lengths_m.shape[0] - 1
            traversal_fraction = 1.0
        else:
            ordered_index = int(
                np.searchsorted(cumulative, selection_distance, side="right")
            )
            previous = (
                0.0 if ordered_index == 0 else float(cumulative[ordered_index - 1])
            )
            traversal_fraction = (selection_distance - previous) / float(
                self.ordered_lengths_m[ordered_index]
            )

        segment_index = int(self.ordered_segment_indices[ordered_index])
        segment_fraction = (
            traversal_fraction
            if self.ordered_forward[ordered_index]
            else 1.0 - traversal_fraction
        )
        line = self.lines[segment_index]
        point = (1.0 - segment_fraction) * self.points[
            line[0]
        ] + segment_fraction * self.points[line[1]]
        arc_fraction = requested_distance / total_length
        return ContourLocation(
            source_coord=2.0 * arc_fraction - 1.0,
            arc_fraction=arc_fraction,
            arc_length_m=requested_distance,
            point=point,
            segment_index=segment_index,
            segment_fraction=float(segment_fraction),
            component_index=int(self.ordered_component_indices[ordered_index]),
        )

    def locate(self, source_coord: float) -> ContourLocation:
        """Map one normalized policy coordinate to an exact contour point."""

        source = float(source_coord)
        if not np.isfinite(source) or source < -1.0 or source > 1.0:
            raise ValueError("source_coord must be finite and lie in [-1, 1]")
        return self._locate_distance(0.5 * (source + 1.0) * self.total_length_m)

    def sample(self, sample_count: int) -> ContourSamples:
        """Sample every component deterministically with a fixed total budget.

        At least one sample is assigned to every disconnected component. The
        remaining budget is distributed by component arc length. Samples lie
        at cell midpoints, avoiding ambiguous concatenation boundaries.
        """

        if isinstance(sample_count, bool) or not isinstance(sample_count, int):
            raise TypeError("sample_count must be an integer")
        if sample_count < self.component_count:
            raise ValueError("sample_count must be >= the contour component count")

        allocations = np.ones(self.component_count, dtype=np.int32)
        remaining = sample_count - self.component_count
        if remaining:
            quotas = remaining * self.component_lengths_m / self.total_length_m
            whole = np.floor(quotas).astype(np.int32)
            allocations += whole
            leftover = remaining - int(np.sum(whole))
            remainders = quotas - whole
            order = sorted(
                range(self.component_count),
                key=lambda index: (-float(remainders[index]), index),
            )
            for component_index in order[:leftover]:
                allocations[component_index] += 1

        locations: list[ContourLocation] = []
        component_offset = 0.0
        for component_index, count in enumerate(allocations):
            component_length = float(self.component_lengths_m[component_index])
            local_distances = (np.arange(int(count)) + 0.5) * (
                component_length / int(count)
            )
            locations.extend(
                self._locate_distance(component_offset + float(local_distance))
                for local_distance in local_distances
            )
            component_offset += component_length

        return ContourSamples(
            source_coord=np.asarray(
                [location.source_coord for location in locations],
                dtype=np.float64,
            ),
            arc_fraction=np.asarray(
                [location.arc_fraction for location in locations],
                dtype=np.float64,
            ),
            arc_length_m=np.asarray(
                [location.arc_length_m for location in locations],
                dtype=np.float64,
            ),
            points=np.asarray([location.point for location in locations]),
            segment_indices=np.asarray(
                [location.segment_index for location in locations],
                dtype=np.int32,
            ),
            segment_fractions=np.asarray(
                [location.segment_fraction for location in locations],
                dtype=np.float64,
            ),
            component_indices=np.asarray(
                [location.component_index for location in locations],
                dtype=np.int32,
            ),
            valid_mask=np.ones(sample_count, dtype=np.bool_),
        )

    def interpolate(
        self,
        point_values: npt.ArrayLike,
        samples: ContourSamples,
        *,
        name: str = "point_values",
    ) -> np.ndarray:
        """Interpolate per-contour-point scalar or vector values at samples."""

        if not isinstance(samples, ContourSamples):
            raise TypeError("samples must be ContourSamples")
        values = np.asarray(point_values, dtype=np.float64)
        if values.ndim < 1 or values.shape[0] != self.points.shape[0]:
            raise ValueError(f"{name} must contain one value per contour point")
        if not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must contain only finite values")
        lines = self.lines[samples.segment_indices]
        start = values[lines[:, 0]]
        end = values[lines[:, 1]]
        weight_shape = (samples.count,) + (1,) * (values.ndim - 1)
        weights = samples.segment_fractions.reshape(weight_shape)
        return _readonly((1.0 - weights) * start + weights * end, np.float64)
