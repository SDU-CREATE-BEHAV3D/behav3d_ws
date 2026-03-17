#!/usr/bin/env python3
"""Manual-pose version of scalar-field + phi-contour pipeline.

This wrapper keeps the same outputs, but always uses user-provided
field offsets (no XY pose search).
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_phi_contour_manual.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-subdivide-iter 1 \
  --field-scale 0.001 \
  --field-offset-x -0.17 --field-offset-y -0.92 --field-offset-z -0.05 \
  --clearance 0.00 \
  --offset-distance-mm 12 \
  --offset-geodesic-delta-mm 1.0 \
  --print-count 7 --print-min-spacing-mm 16 \
  --axis-size -1

"""

from __future__ import annotations

import argparse
from pathlib import Path

from field_scan_phi_contour import DEFAULT_FIELD_MESH, DEFAULT_SCAN_MESH, run

OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Manual-pose pipeline: phi contour, geodesic offset contour, print points, YAML targets."
    )
    parser.add_argument("--field-mesh", type=Path, default=DEFAULT_FIELD_MESH)
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_SCAN_MESH)
    parser.add_argument(
        "--out-field-ply",
        type=Path,
        default=OUTPUT_DIR / "field_masked.ply",
    )
    parser.add_argument(
        "--out-contour-ply",
        type=Path,
        default=OUTPUT_DIR / "field_phi0_contour.ply",
    )
    parser.add_argument(
        "--out-offset-ply",
        type=Path,
        default=OUTPUT_DIR / "field_phi_offset_12mm.ply",
    )
    parser.add_argument(
        "--out-print-ply",
        type=Path,
        default=OUTPUT_DIR / "field_print_points.ply",
    )
    parser.add_argument(
        "--out-targets-yaml",
        type=Path,
        default=Path("/home/lab/behav3d_ws/yaml/scalar_field_targets.yaml"),
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--seed-level", type=float, default=None)
    parser.add_argument("--t-coef", type=float, default=1.0)
    parser.add_argument("--field-subdivide-iter", type=int, default=0)
    parser.add_argument("--field-scale", type=float, default=1.0)
    parser.add_argument("--field-offset-x", type=float, default=0.0)
    parser.add_argument("--field-offset-y", type=float, default=0.0)
    parser.add_argument("--field-offset-z", type=float, default=0.0)
    parser.add_argument("--base-epsilon", type=float, default=1e-6)
    parser.add_argument("--clearance", type=float, default=0.0003)
    parser.add_argument("--iso-level", type=float, default=0.0)
    parser.add_argument("--offset-distance-mm", type=float, default=12.0)
    parser.add_argument("--offset-geodesic-delta-mm", type=float, default=0.0)
    parser.add_argument("--offset-t-coef", type=float, default=1.0)
    parser.add_argument("--offset-toward-printed", action="store_true")
    parser.add_argument("--print-count", type=int, default=7)
    parser.add_argument("--print-min-spacing-mm", type=float, default=16.0)
    parser.add_argument("--target-zx", type=float, default=0.03)
    parser.add_argument("--target-zy", type=float, default=-0.01)
    parser.add_argument("--target-zz", type=float, default=1.00)
    parser.add_argument("--target-position-scale", type=float, default=1000.0)
    parser.add_argument("--axis-size", type=float, default=0.0)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    run(
        field_mesh_path=args.field_mesh,
        scan_mesh_path=args.scan_mesh,
        out_field_ply=args.out_field_ply,
        out_contour_ply=args.out_contour_ply,
        out_offset_ply=args.out_offset_ply,
        out_print_ply=args.out_print_ply,
        out_targets_yaml=args.out_targets_yaml,
        seed=args.seed,
        seed_level=args.seed_level,
        t_coef=args.t_coef,
        field_subdivide_iter=args.field_subdivide_iter,
        field_scale=args.field_scale,
        field_offset=(args.field_offset_x, args.field_offset_y, args.field_offset_z),
        pose_search=False,
        search_x_min=None,
        search_x_max=None,
        search_y_min=None,
        search_y_max=None,
        search_margin_x=0.0,
        search_margin_y=0.0,
        search_step_x=0.01,
        search_step_y=0.01,
        search_max_candidates=1,
        search_allow_partial_hit=False,
        search_verbose=False,
        base_epsilon=args.base_epsilon,
        clearance=args.clearance,
        iso_level=args.iso_level,
        offset_distance_mm=args.offset_distance_mm,
        offset_geodesic_delta_mm=args.offset_geodesic_delta_mm,
        offset_t_coef=args.offset_t_coef,
        offset_toward_unprinted=not args.offset_toward_printed,
        print_count=args.print_count,
        print_min_spacing_mm=args.print_min_spacing_mm,
        target_z_dir=(args.target_zx, args.target_zy, args.target_zz),
        target_position_scale=args.target_position_scale,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
    )


if __name__ == "__main__":
    main()
