"""Area-weighted completion metrics for an open 2.5D goal surface.

This module is the first, deliberately narrow goal evaluator for RL trials. It
matches the scalar pipeline's vertical-ray convention on resolved samples:

    phi = z_goal - z_scan - clearance

Ray misses remain explicitly unresolved. They are never treated as completed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]
BoolArray = npt.NDArray[np.bool_]
IntArray = npt.NDArray[np.int32]


@dataclass(frozen=True)
class GoalMetrics:
    """Scalar summary of one goal evaluation."""

    vertex_count: int
    resolved_vertex_count: int
    filled_vertex_count: int
    unfilled_vertex_count: int
    frontier_vertex_count: int
    total_area_m2: float
    resolved_area_fraction: float
    completed_area_fraction: float
    unfilled_area_fraction: float
    frontier_area_fraction: float
    mean_remaining_gap_m: float
    p95_remaining_gap_m: float
    max_remaining_gap_m: float

    def to_dict(self) -> dict[str, int | float]:
        """Return a serialization-friendly metrics mapping."""

        return asdict(self)


@dataclass(frozen=True)
class GoalEvaluation:
    """Per-vertex goal state and its area-weighted summary."""

    phi: FloatArray
    gap_to_completion: FloatArray
    has_hit: BoolArray
    filled: BoolArray
    unfilled: BoolArray
    unresolved: BoolArray
    frontier: BoolArray
    vertex_area_weights: FloatArray
    edges: IntArray
    metrics: GoalMetrics


def _vertices(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {result.shape}")
    if result.shape[0] == 0:
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _faces(values: npt.ArrayLike, vertex_count: int) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 3:
        raise ValueError(f"goal_faces must have shape (M, 3), got {raw.shape}")
    if raw.shape[0] == 0:
        raise ValueError("goal_faces must not be empty")
    if not np.issubdtype(raw.dtype, np.integer):
        if not np.all(np.equal(raw, np.floor(raw))):
            raise ValueError("goal_faces must contain integer indices")
    result = raw.astype(np.int32, copy=False)
    if np.any(result < 0) or np.any(result >= int(vertex_count)):
        raise ValueError("goal_faces contains an out-of-range vertex index")
    return result


def _nonnegative_finite(value: float, name: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and >= 0, got {value}")
    return result


def _readonly(values: npt.NDArray, dtype: npt.DTypeLike) -> npt.NDArray:
    result = np.asarray(values, dtype=dtype).copy()
    result.setflags(write=False)
    return result


def vertex_area_weights(
    goal_vertices: npt.ArrayLike,
    goal_faces: npt.ArrayLike,
) -> FloatArray:
    """Assign one third of each incident triangle's area to each vertex."""

    vertices = _vertices(goal_vertices, "goal_vertices")
    faces = _faces(goal_faces, vertices.shape[0])
    triangles = vertices[faces]
    triangle_areas = 0.5 * np.linalg.norm(
        np.cross(
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 0],
        ),
        axis=1,
    )
    if not np.all(np.isfinite(triangle_areas)):
        raise ValueError("goal mesh produced non-finite triangle areas")

    weights = np.zeros(vertices.shape[0], dtype=np.float64)
    contribution = triangle_areas / 3.0
    for corner in range(3):
        np.add.at(weights, faces[:, corner], contribution)
    if float(np.sum(weights)) <= 1e-15:
        raise ValueError("goal mesh has zero total surface area")
    return _readonly(weights, np.float64)


def goal_mesh_edges(
    goal_faces: npt.ArrayLike,
    *,
    vertex_count: int,
) -> IntArray:
    """Return unique undirected edges from triangular goal faces."""

    faces = _faces(goal_faces, vertex_count)
    edges = np.concatenate(
        (faces[:, (0, 1)], faces[:, (1, 2)], faces[:, (2, 0)]),
        axis=0,
    )
    edges = np.sort(edges, axis=1)
    edges = edges[edges[:, 0] != edges[:, 1]]
    edges = np.unique(edges, axis=0).astype(np.int32, copy=False)
    return _readonly(edges, np.int32)


def frontier_from_fill_state(
    filled: npt.ArrayLike,
    edges: npt.ArrayLike,
) -> BoolArray:
    """Mark unfilled vertices having at least one filled mesh neighbor."""

    filled_array = np.asarray(filled, dtype=bool).reshape(-1)
    edge_array = np.asarray(edges, dtype=np.int32)
    if edge_array.ndim != 2 or edge_array.shape[1] != 2:
        raise ValueError(f"edges must have shape (E, 2), got {edge_array.shape}")
    if edge_array.size and (
        np.any(edge_array < 0) or np.any(edge_array >= filled_array.shape[0])
    ):
        raise ValueError("edges contains an out-of-range vertex index")

    filled_neighbor_count = np.zeros(filled_array.shape[0], dtype=np.int32)
    if edge_array.size:
        np.add.at(
            filled_neighbor_count,
            edge_array[:, 0],
            filled_array[edge_array[:, 1]].astype(np.int32),
        )
        np.add.at(
            filled_neighbor_count,
            edge_array[:, 1],
            filled_array[edge_array[:, 0]].astype(np.int32),
        )
    frontier = (~filled_array) & (filled_neighbor_count > 0)
    return _readonly(frontier, np.bool_)


def _weighted_quantile(
    values: FloatArray,
    weights: FloatArray,
    quantile: float,
) -> float:
    if values.size == 0 or float(np.sum(weights)) <= 0.0:
        return 0.0
    order = np.argsort(values)
    sorted_values = values[order]
    sorted_weights = weights[order]
    cumulative = np.cumsum(sorted_weights)
    target = float(quantile) * float(cumulative[-1])
    index = min(int(np.searchsorted(cumulative, target, side="left")), values.size - 1)
    return float(sorted_values[index])


def evaluate_goal_from_height_samples(
    goal_vertices_world: npt.ArrayLike,
    goal_faces: npt.ArrayLike,
    z_scan: npt.ArrayLike,
    has_hit: npt.ArrayLike,
    *,
    clearance_m: float = 0.0,
    fill_tolerance_m: float = 0.002,
) -> GoalEvaluation:
    """Evaluate completion from vertical scan-height samples.

    `gap_to_completion` is zero inside the accepted fill band and positive
    above it. Ray misses receive infinite gap and are explicitly unresolved.
    Negative phi remains completed goal in this first evaluator. It is not
    labeled as overbuild: real overbuild will later be defined from DDS
    material outside the permitted goal envelope.
    """

    vertices = _vertices(goal_vertices_world, "goal_vertices_world")
    faces = _faces(goal_faces, vertices.shape[0])
    scan_heights = np.asarray(z_scan, dtype=np.float64).reshape(-1)
    hit = np.asarray(has_hit, dtype=bool).reshape(-1)
    if scan_heights.shape[0] != vertices.shape[0] or hit.shape[0] != vertices.shape[0]:
        raise ValueError("z_scan and has_hit must have one value per goal vertex")
    if np.any(hit & ~np.isfinite(scan_heights)):
        raise ValueError("z_scan must be finite wherever has_hit is true")

    clearance = float(clearance_m)
    if not np.isfinite(clearance):
        raise ValueError("clearance_m must be finite")
    fill_tolerance = _nonnegative_finite(fill_tolerance_m, "fill_tolerance_m")

    phi = np.full(vertices.shape[0], np.nan, dtype=np.float64)
    phi[hit] = vertices[hit, 2] - scan_heights[hit] - clearance
    filled = hit & (phi <= fill_tolerance)
    unresolved = ~hit
    unfilled = ~filled

    gap_to_completion = np.full(vertices.shape[0], np.inf, dtype=np.float64)
    gap_to_completion[hit] = np.maximum(phi[hit] - fill_tolerance, 0.0)

    weights = vertex_area_weights(vertices, faces)
    edges = goal_mesh_edges(faces, vertex_count=vertices.shape[0])
    frontier = frontier_from_fill_state(filled, edges)
    total_area = float(np.sum(weights))

    def area_fraction(mask: BoolArray) -> float:
        return float(np.sum(weights[mask]) / total_area)

    resolved_weights = weights[hit]
    resolved_gaps = gap_to_completion[hit]
    resolved_area = float(np.sum(resolved_weights))
    mean_gap = (
        float(np.sum(resolved_weights * resolved_gaps) / resolved_area)
        if resolved_area > 0.0
        else 0.0
    )
    metrics = GoalMetrics(
        vertex_count=int(vertices.shape[0]),
        resolved_vertex_count=int(np.count_nonzero(hit)),
        filled_vertex_count=int(np.count_nonzero(filled)),
        unfilled_vertex_count=int(np.count_nonzero(unfilled)),
        frontier_vertex_count=int(np.count_nonzero(frontier)),
        total_area_m2=total_area,
        resolved_area_fraction=area_fraction(hit),
        completed_area_fraction=area_fraction(filled),
        unfilled_area_fraction=area_fraction(unfilled),
        frontier_area_fraction=area_fraction(frontier),
        mean_remaining_gap_m=mean_gap,
        p95_remaining_gap_m=_weighted_quantile(
            resolved_gaps,
            resolved_weights,
            0.95,
        ),
        max_remaining_gap_m=(
            float(np.max(resolved_gaps)) if resolved_gaps.size else 0.0
        ),
    )

    return GoalEvaluation(
        phi=_readonly(phi, np.float64),
        gap_to_completion=_readonly(gap_to_completion, np.float64),
        has_hit=_readonly(hit, np.bool_),
        filled=_readonly(filled, np.bool_),
        unfilled=_readonly(unfilled, np.bool_),
        unresolved=_readonly(unresolved, np.bool_),
        frontier=frontier,
        vertex_area_weights=weights,
        edges=edges,
        metrics=metrics,
    )


def vertical_raycast_scan_heights(
    goal_vertices_world: npt.ArrayLike,
    scan_vertices_world: npt.ArrayLike,
    scan_faces: npt.ArrayLike,
    *,
    z_top: float | None = None,
) -> tuple[FloatArray, BoolArray]:
    """Cast world -Z rays using the current scalar-pipeline convention."""

    import open3d as o3d

    goal_vertices = _vertices(goal_vertices_world, "goal_vertices_world")
    scan_vertices = _vertices(scan_vertices_world, "scan_vertices_world")
    faces = _faces(scan_faces, scan_vertices.shape[0])
    top = float(np.max(scan_vertices[:, 2]) + 1e3) if z_top is None else float(z_top)
    if not np.isfinite(top) or top <= float(np.max(scan_vertices[:, 2])):
        raise ValueError("z_top must be finite and above the scan mesh")

    tensor_mesh = o3d.t.geometry.TriangleMesh()
    tensor_mesh.vertex["positions"] = o3d.core.Tensor(scan_vertices.astype(np.float32))
    tensor_mesh.triangle["indices"] = o3d.core.Tensor(faces.astype(np.int32))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tensor_mesh)

    rays = np.zeros((goal_vertices.shape[0], 6), dtype=np.float32)
    rays[:, :2] = goal_vertices[:, :2]
    rays[:, 2] = top
    rays[:, 5] = -1.0
    t_hit = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
    hit = np.isfinite(t_hit)
    z_scan = np.full(goal_vertices.shape[0], np.nan, dtype=np.float64)
    z_scan[hit] = top - t_hit[hit]
    return _readonly(z_scan, np.float64), _readonly(hit, np.bool_)


def evaluate_goal_with_vertical_rays(
    goal_vertices_world: npt.ArrayLike,
    goal_faces: npt.ArrayLike,
    scan_vertices_world: npt.ArrayLike,
    scan_faces: npt.ArrayLike,
    *,
    clearance_m: float = 0.0,
    fill_tolerance_m: float = 0.002,
    z_top: float | None = None,
) -> GoalEvaluation:
    """Raycast the scan and evaluate the goal in one call."""

    z_scan, has_hit = vertical_raycast_scan_heights(
        goal_vertices_world,
        scan_vertices_world,
        scan_faces,
        z_top=z_top,
    )
    return evaluate_goal_from_height_samples(
        goal_vertices_world,
        goal_faces,
        z_scan,
        has_hit,
        clearance_m=clearance_m,
        fill_tolerance_m=fill_tolerance_m,
    )
