#!/usr/bin/env python3
"""Quick potpourri3d heat-method test on a mesh file.
Sample run:
python3 /home/lab/behav3d_ws/python_scripts/scalar_field/field_scan_phi_contour.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh.stl \
  --seed-level 1 --t-coef 2000 \
  --field-scale 0.001 \
  --pose-search --search-step-x 0.01 --search-step-y 0.01 \
  --clearance 0.001 --axis-size 0

"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.geometry import load_triangle_mesh_arrays
from lib_scalar.compute_heat_field import compute_heat_field
from lib_scalar.viz import make_point_cloud, yellow_to_red_colors


DEFAULT_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
OUTPUT_DIR = Path("/home/lab/behav3d_ws/python_scripts/scalar_field/output")


def run(
    mesh_path: Path,
    out_ply: Path | None,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
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
