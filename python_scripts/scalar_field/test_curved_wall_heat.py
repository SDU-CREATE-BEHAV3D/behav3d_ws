#!/usr/bin/env python3
"""Quick potpourri3d heat-method test on a mesh file."""
# Run with: python3 /home/lab/behav3d_ws/python_scripts/scalar_field/test_curved_wall_heat.py
# Also  python3 /home/lab/behav3d_ws/python_scripts/scalar_field/test_curved_wall_heat.py --seed-level 1 --t-coef 2000


from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import potpourri3d as pp3d


DEFAULT_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")


def choose_seed_vertex(vertices: np.ndarray) -> int:
    """Pick a stable seed near the mesh centroid."""
    centroid = vertices.mean(axis=0)
    d2 = np.sum((vertices - centroid) ** 2, axis=1)
    return int(np.argmin(d2))


def choose_seed_vertices_below_level(vertices: np.ndarray, level: float) -> np.ndarray:
    """Pick all seed vertices with z-coordinate below a scalar level."""
    return np.flatnonzero(vertices[:, 2] < float(level)).astype(np.int64)


def compact_triangle_mesh(
    vertices: np.ndarray,
    faces: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Remove unreferenced vertices and remap faces to a compact index set."""
    used = np.unique(faces.reshape(-1))
    if used.size == vertices.shape[0]:
        return vertices, faces.astype(np.int32), 0

    remap = -np.ones(vertices.shape[0], dtype=np.int64)
    remap[used] = np.arange(used.size, dtype=np.int64)
    compact_vertices = vertices[used]
    compact_faces = remap[faces].astype(np.int32)
    dropped = int(vertices.shape[0] - compact_vertices.shape[0])
    return compact_vertices, compact_faces, dropped


def normalize_scalar_field(values: np.ndarray) -> tuple[np.ndarray, float, float]:
    finite = np.isfinite(values)
    if not np.any(finite):
        raise ValueError("Scalar field has no finite values.")

    v_min = float(np.min(values[finite]))
    v_max = float(np.max(values[finite]))
    denom = max(v_max - v_min, 1e-12)

    norm = np.ones_like(values, dtype=np.float64)
    norm[finite] = (values[finite] - v_min) / denom
    return norm, v_min, v_max


def yellow_to_red_colors(norm_scalar: np.ndarray) -> np.ndarray:
    norm = np.clip(norm_scalar, 0.0, 1.0)
    colors = np.zeros((norm.shape[0], 3), dtype=np.float64)
    colors[:, 0] = 1.0          # R stays at max
    colors[:, 1] = 1.0 - norm   # G fades from 1 (yellow) to 0 (red)
    colors[:, 2] = 0.0          # B stays at 0
    return colors


def build_colored_point_cloud(vertices: np.ndarray, colors: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def run(
    mesh_path: Path,
    out_ply: Path | None,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
    visualize: bool,
) -> None:
    if not mesh_path.is_file():
        raise FileNotFoundError(f"Mesh not found: {mesh_path}")

    vertices_raw, faces_raw = pp3d.read_mesh(str(mesh_path))
    if vertices_raw.ndim != 2 or vertices_raw.shape[1] != 3:
        raise ValueError(f"Unexpected vertex array shape: {vertices_raw.shape}")
    if faces_raw.ndim != 2 or faces_raw.shape[1] != 3:
        raise ValueError(
            f"Heat method expects triangles; got faces shape {faces_raw.shape} from {mesh_path}"
        )

    vertices, faces, dropped_vertices = compact_triangle_mesh(vertices_raw, faces_raw)

    solver = pp3d.MeshHeatMethodDistanceSolver(vertices, faces, t_coef=t_coef, use_robust=True)
    if seed_level is not None:
        seed_vertices = choose_seed_vertices_below_level(vertices, seed_level)
        if seed_vertices.size == 0:
            raise ValueError(
                f"No seed vertices found for z < {seed_level} (mesh z min/max: "
                f"{float(np.min(vertices[:, 2])):.6f}/{float(np.max(vertices[:, 2])):.6f})"
            )
        dist = solver.compute_distance_multisource(seed_vertices.tolist())
        seed_info = f"seed_level(z<{seed_level}): count={seed_vertices.size}"
    else:
        if seed is None:
            seed = choose_seed_vertex(vertices)
        if seed < 0 or seed >= vertices.shape[0]:
            raise ValueError(f"Seed index out of range: {seed} (num vertices={vertices.shape[0]})")
        dist = solver.compute_distance(seed)
        seed_info = f"seed_vertex: {seed}"

    if dist.shape[0] != vertices.shape[0]:
        raise RuntimeError(
            f"Distance field size mismatch: len(dist)={dist.shape[0]} vs vertices={vertices.shape[0]}"
        )
    norm_dist, d_min, d_max = normalize_scalar_field(dist)
    colors = yellow_to_red_colors(norm_dist)
    colored_pcd = build_colored_point_cloud(vertices, colors)

    print(f"mesh: {mesh_path}")
    print(f"vertices (raw): {vertices_raw.shape[0]}")
    print(f"vertices (used): {vertices.shape[0]}")
    print(
        "z-range (used vertices): "
        f"[{float(np.min(vertices[:, 2])):.6f}, {float(np.max(vertices[:, 2])):.6f}]"
    )
    if dropped_vertices > 0:
        print(f"dropped unreferenced vertices: {dropped_vertices}")
    print(f"faces: {faces.shape[0]}")
    print(seed_info)
    print(
        "distance stats: "
        f"min={d_min:.6f}, "
        f"max={d_max:.6f}, "
        f"mean={float(np.mean(dist)):.6f}"
    )

    if out_ply is not None:
        out_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_point_cloud(str(out_ply), colored_pcd)
        if not ok:
            raise RuntimeError(f"Failed to write colored point cloud PLY: {out_ply}")
        print(f"saved colored point cloud: {out_ply}")

    if visualize:
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        o3d.visualization.draw_geometries([colored_pcd, axes], point_show_normal=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Test heat-method scalar field on curved wall mesh.")
    parser.add_argument(
        "--mesh",
        type=Path,
        default=DEFAULT_MESH,
        help=f"Triangle mesh path (default: {DEFAULT_MESH})",
    )
    parser.add_argument(
        "--out-ply",
        type=Path,
        default=Path("/home/lab/behav3d_ws/python_scripts/scalar_field/curved_wall_heat_yellow_red.ply"),
        help="Output .ply path for colored point cloud (yellow -> red gradient).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional seed vertex index. If omitted, choose centroid-nearest vertex.",
    )
    parser.add_argument(
        "--seed-level",
        type=float,
        default=None,
        help=(
            "Use all vertices with z < level as multi-source seeds "
            "(level in mesh coordinate units)."
        ),
    )
    parser.add_argument(
        "--t-coef",
        type=float,
        default=1.0,
        help="Heat-method time coefficient.",
    )
    parser.add_argument(
        "--no-vis",
        action="store_true",
        help="Disable Open3D visualization window.",
    )
    args = parser.parse_args()

    run(
        mesh_path=args.mesh,
        out_ply=args.out_ply,
        seed=args.seed,
        seed_level=args.seed_level,
        t_coef=args.t_coef,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
