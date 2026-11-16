# behav3d_orchestrator

ROS 2 sequence nodes that compose motion, scanning, field candidate generation,
and printing through the shared session classes in `behav3d_orchestrator/src/`.

## Scan, undistort, reconstruct and inspect in RViz

Use the existing `scan_yaml_targets_sequence` on branch `J1_Build`.
It runs the configured scan targets, waits for
complete color alignment, reconstructs the saved depth, and updates the RViz
mesh through `/behav3d/update_world_mesh`. Both color and depth use the session's
saved distortion calibration. Bringup enables this with
`reconstruct_rectify_images:=true`; all three reconstruction services also
default to `rectify_images=true` when started individually.

On the lab PC, stop the old bringup, update `J1_Build`, and rebuild
the processing, orchestration, launch and RViz packages together:

```bash
cd ~/behav3d_ws
git fetch origin
git switch J1_Build
git pull --ff-only origin J1_Build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select behav3d_sense behav3d_orchestrator \
  behav3d_bringup ur20_workcell_moveit_config --executor sequential
source install/setup.bash
```

Start the usual bringup using the lab's existing robot address (replace
`YOUR_EXISTING_ROBOT_IP`):

```bash
ros2 launch behav3d_bringup print_move.launch.py \
  use_mock_hardware:=false robot_ip:=YOUR_EXISTING_ROBOT_IP \
  orbbec_enable:=true enable_reconstruct_services:=true \
  reconstruct_rectify_images:=true world_mesh_frame_id:=ur20_base_link \
  world_mesh_use_colors:=true
```

In a second sourced terminal, check the running nodes report `True` for both
processing stages. Display only the latest reconstruction during this test:

```bash
cd ~/behav3d_ws
source install/setup.bash
ros2 param get /color_to_depth_service rectify_images
ros2 param get /tsdf_cropped_service rectify_images
ros2 param set /behav3d_world mesh_accumulate false
```

Select the existing scan trajectory through `yaml_path` in
`src/behav3d_orchestrator/config/scan_yaml_targets_sequence_config.yaml`. The
checked-in path is the November 13 linear-barrel scan. The existing config
already has `run_reconstruction`, `wait_reconstruction_outputs` and
`update_world_mesh` enabled, with center/AABB/adaptive crops disabled. It keeps
the normal 1.953125 mm TSDF grid and threshold 3. No experimental 1 mm grid,
angle rejection or pose refinement is enabled. Motion settings and target
coordinates remain those of the existing scan config.

Run the existing sequence when ready to execute that scan:

```bash
ros2 run behav3d_orchestrator scan_yaml_targets_sequence --ros-args \
  -p config_path:="$PWD/src/behav3d_orchestrator/config/scan_yaml_targets_sequence_config.yaml"
```

This command executes robot motion and capture. Wait for the sequence to finish,
including the configured `home_after`, before manually moving toward the object.
The corrected mesh appears after reconstruction finishes; it is a fixed scan
result while the robot model follows live joint states. This is not continuous
live meshing. RViz should use fixed frame `world`, with the Marker display on
`/visualization_marker`, namespace `reconstructed_mesh`, enabled. The supplied
RViz configuration uses Transient Local durability to receive the latest marker
after reconnecting. Mesh coordinates are metres in `ur20_base_link`; the
world-to-base transform places them alongside the robot. The mesh marker is a
visual overlay, not a MoveIt collision object.

The processing console reports `Saved-calibration rectification: True`.
Alignment is written under
`<session>/scan_yaml_targets/reconstruct/color_in_depth_rectified/`; its
`alignment.json` must report `rectified: true` and `complete: true`. The final
colored mesh is `tsdf_surface_mesh.ply` in the same `reconstruct/` directory;
an STL copy is also exported. The scan
sequence reports `Reconstruction complete`, and the world node logs
`Published world mesh` with the actual source path.

Local validation covers distortion correction, complete-image handoff and
service dispatch, plus saved-scan service replays. Live ROS transport, RViz and
robot alignment still require this lab test.

### Changes from original J1 (`e0d2512`)

| Stage | Original J1 | Updated ROS workflow |
|---|---|---|
| Camera distortion | Coefficients loaded but unused | Saved color/depth calibration applied by default |
| Alignment handoff | First fresh image could trigger TSDF | Complete indexed frame set, matching calibration and file hashes required |
| Mesh display | Uniform STL | RGB vertex-colored PLY triangles; STL remains available |
| Detached fragments | Components below 50 triangles removed | Adjustable cutoff of 100; duplicate/degenerate cleanup ordered correctly |
| RViz marker | Zero quaternion; volatile display subscription | Identity rotation; retained marker received on reconnect |

Scan targets, capture settings, robot poses, calibration files, voxel size and
fusion confidence threshold are unchanged. Experimental angle/pose filters and
the 1 mm grid are not enabled. The complete study and its reproducible tools
remain on the [reconstruction-quality branch](https://github.com/SDU-CREATE-BEHAV3D/behav3d_ws/tree/codex/j1-reconstruction-quality).

The focused offline checks can be run from the workspace root in an environment
with Open3D, OpenCV, NumPy, SciPy, PyYAML and pytest installed:

```bash
PYTHONPATH=src/behav3d_sense python3 -m pytest \
  src/behav3d_sense/test/test_capture_validation.py \
  src/behav3d_sense/test/test_rectification.py \
  src/behav3d_sense/test/test_reconstruction_workflow.py \
  src/behav3d_sense/test/test_colored_mesh.py \
  src/behav3d_sense/test/test_mesh_cleanup.py -q
```

### Camera-colored meshes

The same scan command now publishes a triangle mesh with measured RGB vertex
colors. The reconstruction service returns the colored PLY, and the world node
renders its triangles even when point-cloud display is otherwise preferred.
Colors come from the aligned camera images with the existing depth-visibility
check and minimum of three observations. Vertices with insufficient color
support remain neutral gray. Their geometry is retained.

`world_mesh_use_colors:=true` is the bringup default. Launch with
`world_mesh_use_colors:=false` for the uniform-gray comparison. Both modes use
the same saved vertices and faces. Reconstruct older sessions again to produce
vertex colors; their existing STL files do not contain RGB data. Colored point
clouds remain available through `mesh_prefer: ply`; keep `mesh_prefer: mesh` for
the filled triangle surface used by this test.

The default `ply_marker_max_triangles=0` displays the full mesh. Large scans can
take longer to publish and render. If needed, starting the world node with a
positive `ply_marker_max_triangles` applies display-only quadric simplification;
the saved reconstruction remains unchanged. Keep zero for geometry comparisons.

### Detached-fragment cleanup

The mesh export removes disconnected components with fewer than 100 triangles
before saving either PLY or STL. The earlier 50-triangle cutoff let the visible
62- and 63-triangle floating patches survive. This operation keeps the exact
vertices and faces of retained components, including attached thin details;
it does not smooth the surface. Fusion confidence remains at 3.

The separate RGB point cloud also has statistical outlier filtering, which does
not by itself clean the exported triangle mesh. Noise connected to a retained
surface is unaffected by the component-size filter.

To change the component cutoff for subsequent reconstructions:

```bash
ros2 param set /tsdf_cropped_service mesh_min_component_triangles 100
```

Use 50 to reproduce the earlier cutoff or 0 to retain all disconnected parts.
Small genuine detached parts below the selected size are also removed, so lower
the cutoff when those are intentional features of the object being scanned.

## YAML Fault Print and Scan

`print_yaml_fault_and_scan_sequence` reuses the YAML-print sequence, scan and
reconstruction flow. Both YAML sequence nodes print `points_per_cycle` beads,
then scan and optionally reconstruct before the next group when `enable_scan`
is true. The last, possibly smaller group also gets a scan; there is no extra
final scan. `enable_scan: false` skips all scans. A failed scan/reconstruction
stops the sequence. `print_mode: dots` deposits once at each
segment end or flat target; `segments` prints the saved start-to-end motion;
`auto` selects segments for segment YAML and dots otherwise. Omitted mode keeps
the fault sequence's previous `auto` behavior. Segment starts are not flattened
into additional dots, and segment volumes remain attached to their beads. It
never rewrites the input target YAML.

Configure `config/print_yaml_fault_and_scan_sequence_config.yaml`:

- `yaml_path`: original target/segment YAML.
- `print_mode`: `dots`, `segments`, or `auto`, matching the geometry loop.
- `fault_region_mesh_path`: a closed, watertight STL or other Open3D triangle
  mesh. Relative paths are resolved from the config directory.
- `fault_region_mesh_scale`: `0.001` for mesh coordinates authored in mm,
  or `1.0` for meters. Scaled vertices must already be in `frame_id`, the
  same coordinate frame as the target origins. No implicit yaw is applied.
- `extrusion_reduction_pct`: material reduction from 0 to 100 percent.
  For example, 40 requests 60 percent of nominal net material inside the region.

The endpoint selects the whole bead, including for segments. Points on the
region boundary (within 0.001 mm) count as inside. The region is always active;
there is no cycle schedule. Requested net steps are reduced before retract
compensation is added. Nominal segment motion duration and speed are retained;
only forward extrusion steps and the corresponding step rate change. For dots,
the nominal extrusion duration is retained to the nearest supported integer
step rate. A 100 percent reduction sends no forward or retract command for that
bead. A valid region mesh is required, including for a zero-reduction baseline.

The node publishes the scaled region as an orange triangle marker with opacity
0.2 on `/behav3d/fault_region`. Add a Marker display for this topic in RViz.
Target colors retain the existing display behavior.

`src/fault_session.py` provides the shared `FaultSession`: it owns region
loading, inside/outside queries, material reduction and the RViz marker. Each
fault node configures this session and attaches it to its existing
`PrintSession`. It does not own a separate command queue or execute motion.

```bash
ros2 run behav3d_orchestrator print_yaml_fault_and_scan_sequence
```

The default config is read from the source workspace config folder, matching the
existing YAML sequence. A different file can be selected explicitly:

```bash
ros2 run behav3d_orchestrator print_yaml_fault_and_scan_sequence --ros-args -p config_path:=/absolute/path/to/experiment.yaml
```

## Bead Execution Report

`PrintSession` writes each YAML-print/field-loop cycle to
`<session>/<scope>/cycle_xxxx/print_manifest.yaml`. The active cycle is cached in
memory; bead checkpoints atomically replace only this small file, using the C
safe YAML backend when available. Earlier cycles are never reread or rewritten
for a bead checkpoint. This is independent of optional joint-current logging.
Existing target-generation, current/event and contour-offset outputs retain
their formats.

After each print group, the cycle gets `print_status` and a `summary`: attempted,
successful, failed and unknown bead counts, status counts, known net steps and
volume, missing-value counts, and the volume from successful beads. Known
material includes acknowledged extrusion even when segment motion failed;
unknown amounts are excluded from sums and counted explicitly. A print group
can finish while individual beads remain failed/unknown, so inspect the summary
as well as `print_status`.

At orderly run exit, `<session>/print_manifest.yaml` is generated once as a
consolidated snapshot for existing analysis tools. It is not updated bead by
bead and can be stale after a crash. Cycle files are authoritative;
`PrintSession.load_print_manifest(session_root)` merges them with any legacy
root manifest, preferring cycle files. Restart numbering checks cycle
directories as well as legacy history, so an interrupted cycle is not
overwritten. Existing captures are not migrated until explicitly resumed or
consolidated by a run.

Manifest updates, command-result tracking and capture-session path resolution
live in `src/print_session.py`; there is no separate bead logger or session-path
module. `JointCurrentLogger` continues to handle its existing current samples
and print-event stream. The capture manifest in `behav3d_sense` remains specific
to camera captures.

Each cycle contains `attempted_beads` and `successful_beads`. Entries record
the original target index, endpoint pose, segment start when applicable,
nominal/requested net material steps and volumes, forward and retract counts,
the bead's `extrusion_steps_per_mm3`, outcome, and `net_volume_mm3`. An attempt is persisted before approach/planning; a pending
unknown-volume record is persisted before requesting extrusion. Updates replace
the report atomically, retaining earlier cycles and repeat attempts.

Manifest schema 3 uses cycle storage and retains the schema 2 bead accounting:
the `step_accounting` legend and `record_schema_version: 2` entries. Counts
distinguish requested commands from replies:

| Field | Meaning |
| --- | --- |
| `nominal_net_steps` | Configured or volume-derived net material, before fault reduction. |
| `requested_net_steps` | Net material after fault reduction, excluding retract compensation. |
| `commanded_forward_steps` | Requested net material **plus** retract compensation. |
| `commanded_retract_steps` | Requested reverse count, or zero if disabled/suppressed. |
| `accepted_forward_steps`, `accepted_retract_steps` | Counts from successful PrintSteps responses. |
| `accepted_net_steps` | Forward reply minus retract reply, clamped to zero. |
| `net_volume_mm3` | Accepted net steps divided by this bead's conversion constant. |

For example, 19,002 net steps with 4,000 retract steps produce an advance of
23,002 and a reverse of 4,000: the net remains **19,002**. The advance includes
retract compensation and must not be interpreted as net deposited material.
`commanded_*` describes requested counts, including attempts that failed before
extrusion; `accepted_*` is zero when no command ran and null when its reply is
missing or failed. Net counts/volume remain null if a required reply is unknown.

`volume_mm3` remains a compatibility alias for `net_volume_mm3`. Both represent
the net balance of completed PrintSteps responses, not measured material flow.
Existing bead records are retained unchanged when an older manifest is read;
missing historical retract/calibration values are not inferred from current
settings. Successful beads require a positive confirmed net
amount, confirmed retract when enabled and confirmed segment motion. A failed
lift after completed deposition does not erase that completed bead.

- Planning/approach failures before extrusion: `extruded: false`, volume zero.
- Confirmed completed deposition: `extruded: true`, estimated volume in mm3.
- Missing/failed extrusion or retract responses: `extruded: null`, volume null.
- Failed/unconfirmed segment motion: retained in attempted beads, excluded from
  successful beads even if a forward-step volume is known.

An operator stop before a target does not create an attempt. YAML-sequence
restarts use the next report cycle number rather than replacing earlier cycles.
The active capture session is resolved once for a run; config reloads do not
move its execution report or cycle scans into a different active session.

Both YAML sequence nodes reload config before selecting each print group,
immediately before printing (after the debug gate), before each bead and after
its print confirmation, before the scan prompt, and after scan confirmation.
Steps, speeds, retract and fault settings can change between beads. Each bead
keeps its settings through extrusion, retract and lift. Scan edits made during
printing apply to the scan following that group; `points_per_cycle` changes
apply when selecting the next group. A reload that
fails to parse or has an invalid group size, timeout, or mode keeps the previous
config. Target YAML contents, `yaml_path`, and `frame_id` require a restart.

## Field Print Loops

- `print_field_oriented_sequence_v2`: scan, reconstruct, initialize/resume a
  scalar field, generate candidates, and print each cycle.
- `geometry_representation_scan_print_loop`: equivalent iterative print path
  starting from an existing field-state representation.

Their runtime settings are installed from:

- `config/print_field_oriented_sequence_config.yaml`
- `config/geometry_representation_scan_print_loop_config.yaml`

Both synchronize candidate-width parameters with `/behav3d_fields` before
calling `/behav3d/generate_print_candidates`.

## Geometry Representation Fault Loop

`geometry_representation_fault_scan_print_loop` inherits the complete
`geometry_representation_scan_print_loop` workflow, including its configured
initial scan, candidate generation, printing, scan/reconstruction for the next
cycle, runtime config reloads and operator rescan option. It adds only the same
execution fault used by `print_yaml_fault_and_scan_sequence`; region loading
and RViz display are shared by both nodes.

Edit `config/geometry_representation_fault_scan_print_loop_config.yaml`. Its
scan, field, candidate and printing settings start from the original loop
configuration. The additional settings are `fault_region_mesh_path`,
`fault_region_mesh_scale` and `extrusion_reduction_pct`, with the same semantics
as the YAML fault workflow above. Set the mesh path before running. Relative
mesh paths use the config folder; the default scale is 0.001 for mm-authored
meshes and the initial reduction is 40 percent.

The region vertices must be in the final execution frame (`world`), matching
the generated target YAML after `oriented_base_to_world_yaw_deg` is applied.
The region does not inherit that yaw automatically from the source field.
Endpoints inside the region select the whole bead for reduced extrusion.
Generated targets, width rules and source volumes remain unchanged.

The region is validated before initial motion and refreshed before each print
batch using the latest runtime config. Changing the percentage takes effect
on the next batch. Geometry is reused until the configured mesh path or scale
changes; restarting also reloads the mesh. RViz receives the same orange marker
at opacity 0.2 on `/behav3d/fault_region`; target colors remain unchanged.

```bash
ros2 run behav3d_orchestrator geometry_representation_fault_scan_print_loop
```

Use `--ros-args -p config_path:=/absolute/path/to/experiment.yaml` to select a
different config. Per-cycle config snapshots remain under
`field_loop/cycle_XXXX/config/`. Attempted and successful beads and their
command-confirmed volume estimates remain in each cycle's `print_manifest.yaml` and the final session snapshot at the
capture-session root. The existing loop's original config and default print
behavior are retained.

## Candidate Width Modes

`candidate_width_mode: fixed` uses:

- `candidate_bead_separation_mm` for center-to-center candidate spacing.
- `candidate_bead_width_mm` as the nominal fixed width.
- `dot_steps` or `segment_steps` as the net material request, according to the
  selected print mode, because fixed-mode target YAML omits `volume_mm3`.

`candidate_width_mode: field` uses:

- `candidate_width_field_path`, an NPZ containing `width_norm` in `[0, 1]`, one
  value per scalar-field vertex in the same vertex order.
- `candidate_bead_width_min_mm` and `candidate_bead_width_max_mm` for the current
  linear width mapping.
- `candidate_bead_overlap_mm` for pair spacing:
  `(width_a + width_b) / 2 - overlap`.
- Analytic rounded-cylinder volume for dot and segment targets. The resulting
  volume is multiplied by `candidate_volume_factor` before `volume_mm3` is
  written, then converted with `extrusion_steps_per_mm3` during field-loop
  printing.

The detailed heat-priority and spacing algorithm belongs to
`behav3d_py/behav3d_py/scalar_field/lib_scalar/README.md`.

Candidate construction and target serialization are configured independently:

- `candidate_mode` selects `z_lift`, `gradient_lift`, or `gradient_walk`.
- `print_mode: dots` writes flat targets; `print_mode: segments` writes
  start/end segments; `print_mode: auto` keeps the legacy mode-based mapping.
- `candidate_segment_start_offset_mm` moves a `gradient_lift` segment start
  toward its end to provide initial tool clearance. It does not affect dots or
  `gradient_walk`.

## Contour Offset Estimation

Set `contour_offset_estimation: true` to compare each printed target with the
direct phi contour generated by the following cycle. It does not modify
candidates, motion, or extrusion.

Each printed cycle writes `contour_offset/contour_offset.json` and
`contour_offset/targets.csv` below its `cycle_xxxx` directory. These reports
contain only the consecutive direct phi contours, target geometry, required
frame/height settings, and calculated offsets. Scan, field-state, target-YAML,
candidate-metric, volume, and extrusion-step references are intentionally not
duplicated here. No geodesic-offset contour participates in the estimate.

When the following cycle generates its phi contour, the estimator maps targets
back into contour coordinates, reconstructs the original gradient-lift source,
and intersects each target axis with the local post-print contour. It records
`expected_gain_mm`, `observed_gain_mm`, `scan_offset_mm`, lateral matching error,
dispersion, and confidence per target. Negative `scan_offset_mm` means the
observed contour remained below the expected target along its print direction.
The cycle summary reports the median offset, MAD, percentile range, spatial
consistency, and the opposite-sign suggested correction. A final cycle without
a following scan and contour remains pending.

## Dot Extrusion Contract

`print_yaml_and_scan_sequence` selects execution with `print_mode`, keeping the
input YAML unchanged. `dots` deposits once at each segment's `end` pose (or each
flat target); `segments` prints along each start-to-end path and requires segment
YAML; `auto` selects segments for segment YAML and dots otherwise. Omitted mode
defaults to `dots` for existing offline configs. The mode is read each cycle.
Segment indices and optional `volume_mm3` are preserved in both modes. In dots
mode, the segment's `plane_type` selects the N/E fallback steps and defaults to N.
Segment mode uses `segment_steps`, `segment_steps_per_second`, and the
`segment_*` motion and retract parameters, as in the geometry loop config.

`dot_steps` always means net bead material. Do not add retract steps in a
configuration file.

For every dot, `PrintSession` resolves net bead steps as follows:

1. Use `dot_steps` when the YAML target has no `volume_mm3`.
2. Otherwise compute `round(volume_mm3 * extrusion_steps_per_mm3)`.
3. If retract is enabled, command
   `forward_steps = net_bead_steps + post_dot_retract_steps`.
4. Deposit the dot, then command the configured reverse retract.

Retract is enabled only when both `post_dot_retract_steps` and
`post_dot_retract_speed` are positive. `extrusion_steps_per_mm3` therefore affects
only targets carrying `volume_mm3`; it has no effect in normal fixed-step mode.
Keep it as the physical unit conversion. Tune variable-width over/under-
extrusion with `candidate_volume_factor`, not the conversion constant.

## Segment Extrusion Contract

`segment_steps` is the net material fallback for segment YAML without
`volume_mm3`. Variable-volume segments use
`round(volume_mm3 * extrusion_steps_per_mm3)` instead.

For each segment, `PrintSession` computes:

1. Use `post_segment_retract_steps` when retract steps and speed are positive.
2. `forward_steps = material_steps + retract_steps`.
3. `target_duration_s = forward_steps / segment_steps_per_second`.
4. `target_tcp_speed_mm_s = segment_length_mm / target_duration_s`.

The planner iteratively tunes its velocity and acceleration scales to that
duration. Once the trajectory is fixed, `PrintSession` adjusts the commanded
step rate to `forward_steps / planned_duration_s`, then starts finite forward
`PrintSteps` and trajectory execution as one parallel command group. The cycle
waits for both operations before continuing. Therefore `segment_print_vel_scale`
is an initial planner seed, not the extrusion amount. After both operations
finish, retract runs as an explicit reverse `print_steps` command.
Theoretical `extrusion_steps_per_mm3` is a unit conversion; tune requested
material upstream with `candidate_volume_factor`.

The target schemas are documented in the workspace `command_format.md`.

## Verification

Focused contracts are covered under `test/`:

- `test_extrusion_calibration.py`
- `test_contour_target_offset_estimator.py`
- `test_contour_offset_logger.py`
- `test_yaml_volume_contract.py`

