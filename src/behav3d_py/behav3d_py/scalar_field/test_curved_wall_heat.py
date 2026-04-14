#!/usr/bin/env python3
"""Quick potpourri3d heat-method test on a mesh file.
Sample run:
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/test_curved_wall_heat.py \
  --mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --seed-level 1 --t-coef 2000 --vector-kind both

"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.geometry import (
    compute_vertex_tangent_axes_from_scalar,
    load_triangle_mesh_arrays,
    sample_tangent_axes_on_surface_from_scalar,
)
from lib_scalar.viz import make_point_cloud, yellow_to_red_colors


DEFAULT_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def _sample_surface_points_uniform(
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    count: int,
    *,
    seed: int = 0,
) -> np.ndarray:
    """Sample approximately-uniform random points over triangle areas."""
    if count <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    v0 = mesh_vertices[mesh_faces[:, 0]]
    v1 = mesh_vertices[mesh_faces[:, 1]]
    v2 = mesh_vertices[mesh_faces[:, 2]]
    face_area2 = np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    probs = face_area2 / max(float(np.sum(face_area2)), 1e-12)
    rng = np.random.default_rng(int(seed))
    f_idx = rng.choice(mesh_faces.shape[0], size=int(count), replace=True, p=probs)

    r1 = rng.random(int(count))
    r2 = rng.random(int(count))
    sr1 = np.sqrt(r1)
    w0 = 1.0 - sr1
    w1 = sr1 * (1.0 - r2)
    w2 = sr1 * r2

    tri = mesh_faces[f_idx]
    pts = (
        w0[:, None] * mesh_vertices[tri[:, 0]]
        + w1[:, None] * mesh_vertices[tri[:, 1]]
        + w2[:, None] * mesh_vertices[tri[:, 2]]
    )
    return pts


def _sample_vectors(
    points: np.ndarray,
    vectors: np.ndarray,
    *,
    step: int,
    scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Downsample vectors and return start/end points."""
    step = max(1, int(step))
    idx = np.arange(0, points.shape[0], step, dtype=np.int64)
    p0 = points[idx]
    v = vectors[idx]
    n = np.linalg.norm(v, axis=1)
    valid = n > 1e-12

    p0 = p0[valid]
    v = v[valid] / n[valid, None]
    p1 = p0 + float(scale) * v

    return p0, p1


def _rotation_from_z(direction: np.ndarray) -> np.ndarray:
    """Rotation matrix that maps +Z to a target unit direction."""
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    d = np.asarray(direction, dtype=np.float64)
    d /= max(np.linalg.norm(d), 1e-12)
    c = float(np.dot(z, d))
    if c > 1.0 - 1e-12:
        return np.eye(3, dtype=np.float64)
    if c < -1.0 + 1e-12:
        return np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float64,
        )

    axis = np.cross(z, d)
    s = max(np.linalg.norm(axis), 1e-12)
    axis /= s
    ax = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + ax * s + (ax @ ax) * (1.0 - c)


def _build_vector_tubes(
    start: np.ndarray,
    end: np.ndarray,
    *,
    radius: float,
    color: tuple[float, float, float],
) -> o3d.geometry.TriangleMesh:
    """Build thicker vector glyphs using short cylinders."""
    mesh = o3d.geometry.TriangleMesh()
    if start.shape[0] == 0:
        return mesh

    r = max(1e-9, float(radius))
    for i in range(start.shape[0]):
        p0 = start[i]
        p1 = end[i]
        d = p1 - p0
        length = float(np.linalg.norm(d))
        if length <= 1e-12:
            continue
        tube = o3d.geometry.TriangleMesh.create_cylinder(radius=r, height=length, resolution=12, split=1)
        tube.paint_uniform_color(color)
        tube.rotate(_rotation_from_z(d), center=np.zeros(3, dtype=np.float64))
        tube.translate(0.5 * (p0 + p1))
        mesh += tube

    return mesh


def run(
    mesh_path: Path,
    out_ply: Path | None,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
    vector_kind: str,
    vector_step: int,
    vector_scale_ratio: float,
    vector_radius_ratio: float,
    vector_sample_mode: str,
    vector_surface_count: int,
    visualize: bool,
) -> None:
    mesh_data = load_triangle_mesh_arrays(mesh_path)
    heat = compute_heat_field(
        vertices=mesh_data.vertices,
        faces=mesh_data.faces,
        seed=seed,
        seed_level=seed_level,
        t_coef=t_coef,
    )
    colors = yellow_to_red_colors(heat.norm)
    colored_pcd = make_point_cloud(mesh_data.vertices, colors)
    bbox_diag = float(np.linalg.norm(np.max(mesh_data.vertices, axis=0) - np.min(mesh_data.vertices, axis=0)))
    vec_scale = max(1e-9, float(vector_scale_ratio) * bbox_diag)
    vec_radius = max(1e-9, float(vector_radius_ratio) * bbox_diag)

    print(f"mesh: {mesh_path}")
    print(f"vertices (used): {mesh_data.vertices.shape[0]}")
    if mesh_data.dropped_vertices > 0:
        print(f"dropped unreferenced vertices: {mesh_data.dropped_vertices}")
    print(f"faces: {mesh_data.faces.shape[0]}")
    print(heat.seed_info)
    print(
        "distance stats: "
        f"min={heat.min_value:.6f}, "
        f"max={heat.max_value:.6f}, "
        f"mean={heat.mean_value:.6f}"
    )
    print(
        "z-range (used vertices): "
        f"[{float(np.min(mesh_data.vertices[:, 2])):.6f}, "
        f"{float(np.max(mesh_data.vertices[:, 2])):.6f}]"
    )

    if out_ply is not None:
        out_ply.parent.mkdir(parents=True, exist_ok=True)
        ok = o3d.io.write_point_cloud(str(out_ply), colored_pcd)
        if not ok:
            raise RuntimeError(f"Failed to write colored point cloud PLY: {out_ply}")
        print(f"saved colored point cloud: {out_ply}")

    if visualize:
        geoms = [colored_pcd]
        need_tangent_frame = vector_kind in ("gradient", "orthogonal", "both")
        if need_tangent_frame:
            sample_mode = str(vector_sample_mode).strip().lower()
            if sample_mode == "surface":
                vec_points = _sample_surface_points_uniform(
                    mesh_data.vertices,
                    mesh_data.faces,
                    int(vector_surface_count),
                    seed=0,
                )
                t_dir, b_dir, _ = sample_tangent_axes_on_surface_from_scalar(
                    vec_points,
                    mesh_data.vertices,
                    mesh_data.faces,
                    heat.dist,
                    tangent_sign=1.0,
                )
            else:
                vec_points = mesh_data.vertices
                t_dir, b_dir, _ = compute_vertex_tangent_axes_from_scalar(
                    mesh_data.vertices,
                    mesh_data.faces,
                    heat.dist,
                    tangent_sign=1.0,
                )

            if vector_kind in ("gradient", "both"):
                g0, g1 = _sample_vectors(
                    vec_points,
                    t_dir,
                    step=vector_step,
                    scale=vec_scale,
                )
                grad_tubes = _build_vector_tubes(g0, g1, radius=vec_radius, color=(0.10, 0.95, 0.10))
                geoms.append(grad_tubes)
                print(
                    f"gradient tangent vectors (green): mode={sample_mode}, "
                    f"step={max(1, int(vector_step))}"
                )

            if vector_kind in ("orthogonal", "both"):
                b0, b1 = _sample_vectors(
                    vec_points,
                    b_dir,
                    step=vector_step,
                    scale=vec_scale,
                )
                orth_tubes = _build_vector_tubes(b0, b1, radius=vec_radius, color=(0.20, 0.60, 1.00))
                geoms.append(orth_tubes)
                print(
                    f"orthogonal tangent vectors (blue): mode={sample_mode}, "
                    f"step={max(1, int(vector_step))}"
                )

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geoms.append(axes)
        o3d.visualization.draw_geometries(geoms, point_show_normal=False)


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
        default=OUTPUT_DIR / "curved_wall_heat.ply",
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
        "--vector-kind",
        type=str,
        choices=("gradient", "orthogonal", "both", "none"),
        default="both",
        help="Which tangent vector directions to overlay in Open3D.",
    )
    parser.add_argument(
        "--vector-step",
        type=int,
        default=35,
        help="Draw one vector every N vertices.",
    )
    parser.add_argument(
        "--vector-scale-ratio",
        type=float,
        default=0.02,
        help="Vector length as ratio of mesh AABB diagonal.",
    )
    parser.add_argument(
        "--vector-radius-ratio",
        type=float,
        default=0.0015,
        help="Vector tube radius as ratio of mesh AABB diagonal.",
    )
    parser.add_argument(
        "--vector-sample-mode",
        type=str,
        choices=("vertices", "surface"),
        default="surface",
        help="Sample vectors at mesh vertices or interpolated arbitrary surface points.",
    )
    parser.add_argument(
        "--vector-surface-count",
        type=int,
        default=2400,
        help="Number of random surface points when --vector-sample-mode=surface.",
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
        vector_kind=str(args.vector_kind).strip().lower(),
        vector_step=args.vector_step,
        vector_scale_ratio=args.vector_scale_ratio,
        vector_radius_ratio=args.vector_radius_ratio,
        vector_sample_mode=str(args.vector_sample_mode).strip().lower(),
        vector_surface_count=args.vector_surface_count,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
