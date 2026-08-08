#!/usr/bin/env python3
"""Run the DDS field/scan loop from a pre-initialized scalar field state.

This variant is intentionally thin: it reuses the DDS v2 loop, candidate
generation, target rules, YAML output, and visualization, but skips the
field-mesh heat computation and scan-based field positioning stage.

Sample command:
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_state_scan_loop_simulator_dds_v2.py \
  --field-state /home/lab/behav3d_ws/mesh/fields/field_state_init.npz \
  --scan-mesh /home/lab/behav3d_ws/mesh/ScanMesh.stl \
  --scan-scale 0.001 \
  --output-dir /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/output/loop_sim_field_state \
  --candidate-mode gradient_lift \
  --offset-distance-mm 12 \
  --offset-geodesic-delta-mm 0.6 \
  --beads-per-step 7 \
  --bead-separation-mm 16 \
  --bead-width-mm 18 \
  --bead-height-mm 12 \
  --dds-deposit-mode line \
  --dds-line-fraction 0.22
"""

from __future__ import annotations

import argparse
from pathlib import Path

from field_scan_loop_simulator_dds_v2 import (
    DEFAULT_FIELD_MESH,
    DEFAULT_FIELD_STATE,
    DEFAULT_FIELD_STATE_SCAN_MESH,
    OUTPUT_DIR,
    TARGET_POSITION_SCALE,
    run,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Simulate iterative DDS scalar-field loop from an existing field_state_init.npz."
        )
    )
    parser.add_argument("--field-state", type=Path, default=DEFAULT_FIELD_STATE)
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_FIELD_STATE_SCAN_MESH)
    parser.add_argument(
        "--scan-scale",
        type=float,
        default=0.001,
        help="Uniform scale applied to the scan mesh. Default converts mesh/ScanMesh.stl from mm to m.",
    )
    parser.add_argument(
        "--scan-yaw-deg",
        type=float,
        default=0.0,
        help="Rotate the loaded scan mesh about world Z at the origin before simulation.",
    )
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR / "loop_sim_field_state")
    parser.add_argument("--clearance", type=float, default=0.0)
    parser.add_argument("--t-coef", type=float, default=2000.0)
    parser.add_argument(
        "--candidate-mode",
        type=str,
        choices=("geodesic", "z_lift", "gradient_lift", "gradient_walk"),
        default="geodesic",
    )
    parser.add_argument("--offset-distance-mm", type=float, default=12.0)
    parser.add_argument("--offset-geodesic-delta-mm", type=float, default=0.6)
    parser.add_argument("--beads-per-step", type=int, default=7)
    parser.add_argument("--bead-separation-mm", type=float, default=16.0)
    parser.add_argument(
        "--candidate-width-mode",
        choices=("fixed", "field"),
        default="fixed",
    )
    parser.add_argument(
        "--bead-width-mm",
        type=float,
        default=None,
        help="DDS width in fixed mode. Required when --candidate-width-mode fixed.",
    )
    parser.add_argument(
        "--width-field",
        type=Path,
        default=None,
        help="NPZ containing normalized per-field-vertex values under width_norm.",
    )
    parser.add_argument("--bead-width-min-mm", type=float, default=20.0)
    parser.add_argument("--bead-width-max-mm", type=float, default=40.0)
    parser.add_argument("--bead-overlap-mm", type=float, default=4.0)
    parser.add_argument("--bead-height-mm", type=float, default=12.0)
    parser.add_argument("--walk-distance-mm", type=float, default=12.0)
    parser.add_argument("--walk-step-mm", type=float, default=1.0)
    parser.add_argument("--walk-max-steps", type=int, default=32)
    parser.add_argument("--walk-tangent-sign", type=float, default=1.0)
    parser.add_argument("--walk-start-fraction", type=float, default=0.25)
    parser.add_argument("--clamp-to-cone", action="store_true")
    parser.add_argument("--cone-max-tilt-deg", type=float, default=45.0)
    parser.add_argument(
        "--disable-normal-continuity-rule",
        action="store_true",
        help="Disable secondary replacement of low start/end Z-continuity segments.",
    )
    parser.add_argument(
        "--bead-shape",
        type=str,
        choices=("cylinder", "sphere"),
        default="cylinder",
        help="Legacy compatibility argument; DDS proxy uses BeadProfile width/height.",
    )
    parser.add_argument("--axis-size", type=float, default=0.0)
    parser.add_argument(
        "--target-position-scale",
        type=float,
        default=TARGET_POSITION_SCALE,
        help="Scale used when writing step segment YAML positions. Default 1000 writes meters as mm.",
    )
    parser.add_argument("--dds-voxel-size-mm", type=float, default=2.0)
    parser.add_argument(
        "--dds-domain-source",
        type=str,
        choices=("field", "scene"),
        default="field",
    )
    parser.add_argument(
        "--dds-deposit-mode",
        type=str,
        choices=("dot", "line"),
        default="dot",
    )
    parser.add_argument("--dds-line-fraction", type=float, default=1.0)
    parser.add_argument("--dds-threshold", type=float, default=0.5)
    parser.add_argument("--dds-padding-mm", type=float, default=24.0)
    parser.add_argument("--dds-surface-step-size", type=int, default=1)
    parser.add_argument(
        "--dds-view-mode",
        type=str,
        choices=("surface", "occupancy", "implicit"),
        default="surface",
    )
    parser.add_argument("--save-dds-step-bundles", action="store_true")
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    run(
        field_mesh_path=DEFAULT_FIELD_MESH,
        scan_mesh_path=args.scan_mesh,
        output_dir=args.output_dir,
        field_state_path=args.field_state,
        scan_scale=args.scan_scale,
        scan_yaw_deg=args.scan_yaw_deg,
        seed=None,
        seed_level=None,
        t_coef=args.t_coef,
        field_subdivide_iter=0,
        field_scale=1.0,
        clearance=args.clearance,
        candidate_mode=args.candidate_mode,
        offset_distance_mm=args.offset_distance_mm,
        offset_geodesic_delta_mm=args.offset_geodesic_delta_mm,
        beads_per_step=args.beads_per_step,
        bead_separation_mm=args.bead_separation_mm,
        bead_width_mm=args.bead_width_mm,
        candidate_width_mode=args.candidate_width_mode,
        width_field_path=args.width_field,
        bead_width_min_mm=args.bead_width_min_mm,
        bead_width_max_mm=args.bead_width_max_mm,
        bead_overlap_mm=args.bead_overlap_mm,
        bead_height_mm=args.bead_height_mm,
        walk_distance_mm=args.walk_distance_mm,
        walk_step_mm=args.walk_step_mm,
        walk_max_steps=args.walk_max_steps,
        walk_tangent_sign=args.walk_tangent_sign,
        walk_start_fraction=args.walk_start_fraction,
        clamp_to_cone=args.clamp_to_cone,
        cone_max_tilt_deg=args.cone_max_tilt_deg,
        normal_continuity_rule=not args.disable_normal_continuity_rule,
        bead_shape=args.bead_shape,
        positioning_attempts=0,
        position_target_xy=None,
        search_step_x=0.0,
        search_step_y=0.0,
        search_allow_partial_hit=True,
        base_z_offset=0.0,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
        dds_voxel_size_mm=args.dds_voxel_size_mm,
        dds_domain_source=args.dds_domain_source,
        dds_deposit_mode=args.dds_deposit_mode,
        dds_line_fraction=args.dds_line_fraction,
        dds_threshold=args.dds_threshold,
        dds_padding_mm=args.dds_padding_mm,
        dds_surface_step_size=args.dds_surface_step_size,
        dds_view_mode=args.dds_view_mode,
        save_dds_step_bundles=args.save_dds_step_bundles,
        target_position_scale=args.target_position_scale,
    )


if __name__ == "__main__":
    main()
