# behav3d_orchestrator

ROS 2 sequence nodes that compose motion, scanning, field candidate generation,
and printing through the shared session classes in `behav3d_orchestrator/src/`.

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

## Dot Extrusion Contract

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
duration before enabling the extruder and executing the segment. Therefore
`segment_print_vel_scale` is an initial planner seed, not the extrusion amount.
After printing, retract runs as an explicit reverse `print_steps` command.
Theoretical `extrusion_steps_per_mm3` is a unit conversion; tune requested
material upstream with `candidate_volume_factor`.

The target schemas are documented in the workspace `command_format.md`.

## Verification

Focused contracts are covered under `test/`:

- `test_extrusion_calibration.py`
- `test_yaml_volume_contract.py`
