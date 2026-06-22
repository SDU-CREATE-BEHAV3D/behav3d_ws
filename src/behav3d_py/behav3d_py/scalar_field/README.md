# scalar_field

Prototype scripts for scalar-field print planning, scan/field inspection, and
DDS-based bead accumulation experiments.

This folder is intentionally script-oriented. The reusable implementation lives
in `lib_scalar/`; scripts here wire those stages into visual tests, YAML export,
or full loop simulations.

## Current Main Scripts

### `field_scan_loop_simulator_dds_v2.py`

Current DDS-based loop simulator. This is the preferred workbench for testing
scalar policies with DDS bead representation.

Loop flow:

1. Load field mesh and scan mesh.
2. Compute heat scalar on the field mesh.
3. Position the field over the scan on the first step.
4. Keep that field pose fixed for later steps.
5. Recompute `phi` against the current scan proxy.
6. Extract `phi=0` contour and geodesic offset contour.
7. Generate candidates with the selected mode.
8. Add selected candidate end points to a DDS `Simulator` as point deposits.
9. Extract the DDS implicit surface as a mesh proxy.
10. Merge that proxy with the original scan for the next scalar iteration.

Important behavior:

- DDS receives only the selected candidate end points as deposits.
- `gradient_walk` segments are visualization/debug tool paths; they do not
  become DDS bead geometry.
- The magenta geodesic offset line is a diagnostic/reference line. In
  `gradient_walk`, candidates are selected from the `phi=0` contour and walked
  over the scalar field, so the magenta line is not the print path.
- The simulator does not read YAML segments. Its candidates are generated at
  runtime from the scan, field, heat scalar, and selected candidate mode.
- `--bead-shape` is kept only as a legacy compatibility argument. DDS v2 uses
  the DDS `BeadProfile` width/height and point deposits.

Recommended command:

```bash
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator_dds_v2.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-subdivide-iter 1 --field-scale 0.001 \
  --candidate-mode gradient_walk \
  --beads-per-step 7 --bead-separation-mm 16 \
  --bead-height-mm 12 \
  --positioning-attempts 3 \
  --position-target-x 0.0 --position-target-y -0.75 \
  --search-step-x 0.01 --search-step-y 0.01 \
  --offset-distance-mm 12 --offset-geodesic-delta-mm 0.6 \
  --dds-domain-source field \
  --axis-size -1
```

Workbench controls:

- `N`: accept current step, add DDS deposits, and advance.
- `Q` or `Esc`: stop the loop.
- DDS native sidebar controls still manage DDS representation.
- The `BEHAV3D Scalar` overlay checkboxes toggle scalar debug layers:
  - scalar field points,
  - scan wire mesh,
  - `phi=0` contour,
  - geodesic offset line,
  - print targets,
  - source points,
  - walk segments,
  - target normals,
  - debug axes.

Useful DDS/debug flags:

- `--dds-view-mode surface|occupancy|implicit`
- `--dds-domain-source field|scene`
- `--dds-voxel-size-mm 2.0`
- `--dds-threshold 0.5`
- `--dds-surface-step-size 1`
- `--save-dds-step-bundles`
- `--no-vis` for terminal stepping with `n`/`q`

Performance knobs:

- `--dds-domain-source field` builds one fixed DDS grid from the positioned
  field and initial print envelope. This is the default and avoids allocating
  voxels over unrelated regions of the scan.
- `--dds-domain-source scene` restores the legacy scan+field domain.
- `--field-subdivide-iter 0` reduces field/contour resolution.
- `--dds-voxel-size-mm 3.0` reduces DDS grid density.
- `--dds-surface-step-size 2` reduces marching-cubes proxy cost.
- `--walk-step-mm 2.0` and `--walk-max-steps 16` reduce agent-walk work.

Position preference:

- DDS v2 defaults to a preferred positioned field vertex centroid at
  `X=0.0 m`, `Y=-0.75 m`.
- The pose search still maximizes viable field vertices first. Among poses with
  equal viability, it minimizes the XY distance from the field vertex centroid
  to the preferred target before
  comparing heat/hit/Z metrics.
- Change it with `--position-target-x` and `--position-target-y` (meters).
- Use `--no-position-target` to restore the legacy ranking.

### `field_scan_loop_simulator.py`

Legacy Open3D loop simulator. It keeps the older bead proxy path based on mesh
solids, such as cylinders. Leave this script as the reference baseline while
DDS v2 evolves.

### `field_scan_phi_contour.py`

Single-shot field/scan contour inspection. It positions or applies a fixed field
pose, computes `phi`, extracts contours, selects print points, and visualizes
with the DDS viewer path.

Use it when debugging:

- field pose,
- `phi=0` contour,
- geodesic offset contour,
- selected points before running the full loop.

### `field_scan_phi_contour_manual.py`

Manual-offset version of the contour test. It is useful for reproducing older
hand-tuned alignments.

### `test_curved_wall_heat.py`

Heat-field and field visualization test for the curved wall mesh. Useful for
checking scalar seed/t-coef behavior before scan interaction.

## Outputs

Most scripts write debug artifacts under:

```text
src/behav3d_py/behav3d_py/scalar_field/output/
```

Typical DDS v2 outputs:

- `step_##_field_masked.ply`
- `step_##_phi_contour.ply`
- `step_##_offset_contour.ply`
- `step_##_print_points.ply`
- `step_##_print_segments.ply`
- `loop_sim_scan_with_dds_proxy.ply`
- `loop_sim_dds_surface.ply`
- `loop_sim_dds_bundle/`
- `loop_sim_all_print_points.ply`

These files are debugging artifacts and can be regenerated.

## Conceptual Notes

- `phi = z_field - z_scan - clearance`.
- Viable field points are usually those with `phi <= iso`.
- The cyan contour is `phi=0`.
- The magenta contour is a geodesic offset over the field mesh from the cyan
  contour. It is not a Euclidean offset in free space.
- In `gradient_walk`, candidate source points come from the cyan contour, then
  walk along the scalar tangent field. The orange segment visualizes that walk.
- DDS v2 intersects future scalar iterations against `original scan + DDS proxy
  surface mesh`. It currently does not directly sample DDS occupancy for scalar
  candidate filtering.
- The DDS domain is created once after the first field pose/candidate stage and
  remains fixed so deposits and dense fields can be updated incrementally. New
  deposits are checked against the domain support bounds; increase
  `--dds-padding-mm` if a future bead falls outside the print envelope.

## Relationship to YAML/ROS

The DDS v2 simulator is a prototyping track and does not consume print YAML.
YAML segment execution is handled elsewhere in the orchestrator stack. This
separation is intentional while scalar-policy design and ROS execution evolve in
parallel.
