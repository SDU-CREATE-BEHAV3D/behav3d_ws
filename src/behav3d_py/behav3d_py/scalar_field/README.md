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
8. Write the accepted step using the selected dot or segment YAML contract.
9. Read that same segment YAML back and add DDS deposits from the segment
   targets. `dot` mode uses each `end`; `line` mode sweeps `start -> end`.
10. Extract the DDS implicit surface as a mesh proxy.
11. Merge that proxy with the original scan for the next scalar iteration.

Important behavior:

- Accepted steps are saved as `step_##/segments.yaml` using the same nested
  segment contract as the print path. DDS deposits are built from that YAML,
  not the raw in-memory candidate arrays.
- `--dds-deposit-mode dot` keeps the legacy behavior: DDS receives only each
  segment `end` target as a point deposit. `--dds-deposit-mode line` sweeps a
  DDS line deposit from each segment `start` to `end`.
- `--dds-line-fraction` limits line deposits to the final fraction of each
  segment. `1.0` is the full segment, `0.5` is the final half, and `0.0`
  collapses to the end point. The saved target YAML remains unchanged.
- `gradient_walk` and `gradient_lift` keep generated target positions and write
  tangent-based target Z directions; modes without explicit target orientation
  fall back to world `+Z`.
- Segment orientation is sampled only at `start`; the same normalized `Z(...)`
  is copied to `end`, so segment execution keeps one constant tool orientation.
- In the ROS field loops, `candidate_mode` controls candidate geometry while
  `print_mode` independently selects flat dots or start/end segments.
  `print_mode=auto` retains the legacy mapping where only `gradient_walk`
  selects segments automatically.
- For `gradient_lift` segments, `candidate_segment_start_offset_mm` moves the
  start toward the end before secondary rules and YAML serialization. It is
  ignored for dots and `gradient_walk`.
- Secondary target rules run after candidate generation and before
  visualization/YAML/DDS.
- The low-continuity replacement helper remains available for externally built
  targets, but generated segments now share their start orientation at both
  endpoints and therefore do not trigger it.
- The endpoint-spacing rule removes later targets whose `end` point violates
  the active fixed or variable spacing rule.
- `--candidate-width-mode fixed` preserves fixed candidate spacing and requires
  `--bead-width-mm` for DDS geometry.
- `--candidate-width-mode field` loads an NPZ passed through `--width-field`.
  It must contain `width_norm`, one normalized value per field vertex in the
  same order as the heat field. `--bead-width-min-mm` and
  `--bead-width-max-mm` define the linear mapping, while
  `--bead-overlap-mm` defines within-cycle neighbor overlap.
- In field mode, candidate suppression and secondary endpoint pruning use
  `(width_i + width_j) / 2 - overlap`; DDS receives one `BeadProfile` per
  accepted candidate and reserves domain padding for the maximum width.
- Heat and width are sampled on contour source points before lift/walk. Heat
  orders the greedy selection; spacing suppresses neighbors after each pick.
  See `lib_scalar/README.md` for the detailed two-stage spacing algorithm.
- ROS variable-width generation writes analytic `volume_mm3` for dot or segment
  targets and can scale it with `candidate_volume_factor` immediately before
  YAML serialization.
- The magenta geodesic offset line is a diagnostic/reference line. In
  `gradient_walk`, candidates are selected from the `phi=0` contour and walked
  over the scalar field, so the magenta line is not the print path.
- The simulator still generates candidates at runtime from the scan, field,
  heat scalar, and selected candidate mode. After a step is accepted, it
  serializes those candidates as segment YAML and uses that YAML contract to
  build DDS deposits.
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
  --bead-width-mm 18 --bead-height-mm 12 \
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
  - heat seed points,
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

Variable-width arguments:

```bash
  --candidate-width-mode field \
  --width-field /home/lab/behav3d_ws/mesh/fields/width_field.npz \
  --bead-width-min-mm 20 \
  --bead-width-max-mm 40 \
  --bead-overlap-mm 4
```

### `field_state_scan_loop_simulator_dds_v2.py`

Variant of the DDS v2 loop that starts from a pre-initialized field state
instead of a field mesh. It expects a `.npz` state with
`field_vertices_scaled`, `field_faces`, `heat_norm`, and `offset_xyz`, such as
the `field_state_init.npz` produced by the ROS field init path.

This skips only the initial field-mesh heat computation and scan-based field
positioning. Candidate generation, secondary rules, segment YAML, DDS deposits,
viewer overlays, and output layout are shared with `field_scan_loop_simulator_dds_v2.py`.

Example:

```bash
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_state_scan_loop_simulator_dds_v2.py \
  --field-state /home/lab/behav3d_ws/mesh/fields/field_state_init.npz \
  --scan-mesh /home/lab/behav3d_ws/mesh/ScanMesh.stl \
  --scan-scale 0.001 \
  --scan-yaw-deg 0 \
  --output-dir /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/output/loop_sim_field_state \
  --candidate-mode gradient_lift \
  --offset-distance-mm 12 --offset-geodesic-delta-mm 0.6 \
  --beads-per-step 7 --bead-separation-mm 16 \
  --bead-width-mm 18 --bead-height-mm 12 \
  --dds-deposit-mode line --dds-line-fraction 0.22
```

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

- `step_##/segments.yaml`
- `step_##/scan_with_dds_proxy.ply`
- `step_##/dds_bundle/` when `--save-dds-step-bundles` is enabled
- `loop_sim_scan_with_dds_proxy.ply`
- `loop_sim_dds_surface.ply`
- `loop_sim_dds_bundle/`
- `loop_sim_all_print_points.ply`

Per-step PLY debug overlays are visualized in the DDS workbench but are not
written by default.

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

The DDS v2 simulator remains a prototyping track and writes/reads its own
per-step `segments.yaml` before creating DDS deposits. The ROS YAML and print
contracts are documented centrally in `command_format.md` and
`src/behav3d_orchestrator/README.md`. Width maps and DDS objects remain outside
the ROS print service contract.
