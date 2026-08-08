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
- `dot_steps` as the net material request because fixed-mode target YAML omits
  `volume_mm3`.

`candidate_width_mode: field` uses:

- `candidate_width_field_path`, an NPZ containing `width_norm` in `[0, 1]`, one
  value per scalar-field vertex in the same vertex order.
- `candidate_bead_width_min_mm` and `candidate_bead_width_max_mm` for the current
  linear width mapping.
- `candidate_bead_overlap_mm` for pair spacing:
  `(width_a + width_b) / 2 - overlap`.
- Analytic rounded-cylinder volume for flat dot targets. The resulting
  `volume_mm3` is converted with `dot_steps_per_mm3` during printing.

The detailed heat-priority and spacing algorithm belongs to
`behav3d_py/behav3d_py/scalar_field/lib_scalar/README.md`.

## Dot Extrusion Contract

`dot_steps` always means net bead material. Do not add retract steps in a
configuration file.

For every dot, `PrintSession` resolves net bead steps as follows:

1. Use `dot_steps` when the YAML target has no `volume_mm3`.
2. Otherwise compute `round(volume_mm3 * dot_steps_per_mm3)`.
3. If retract is enabled, command
   `forward_steps = net_bead_steps + post_dot_retract_steps`.
4. Deposit the dot, then command the configured reverse retract.

Retract is enabled only when both `post_dot_retract_steps` and
`post_dot_retract_speed` are positive. `dot_steps_per_mm3` therefore affects
only targets carrying `volume_mm3`; it has no effect in normal fixed-step mode.

The target schemas are documented in the workspace `command_format.md`.

## Verification

Focused contracts are covered under `test/`:

- `test_extrusion_calibration.py`
- `test_yaml_volume_contract.py`
