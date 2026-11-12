# behav3d_orchestrator

ROS 2 sequence nodes that compose motion, scanning, field candidate generation,
and printing through the shared session classes in `behav3d_orchestrator/src/`.

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

The existing `PrintSession` writes `print_manifest.yaml` for YAML-print and field-loop nodes in
their capture-session root. This is independent of optional joint-current
logging. Existing target-generation, current/event and contour-offset outputs
retain their formats.

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

Schema 2 writes a `step_accounting` legend and marks new beads with
`record_schema_version: 2`. Counts distinguish requested commands from replies:

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
Existing bead records are retained unchanged when an older manifest is reopened;
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
command-confirmed volume estimates remain in `print_manifest.yaml` at the
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

