# Command and YAML formats

The former generic YAML command list consumed by
`behav3d_py.SequenceParser` has been retired. `behav3d_py` no longer exports
`Commands` or `SequenceParser`, so old lists containing commands such as
`setPTP`, `goto`, `printTime`, and `capture` are not executable through that
package.

## Current command API

Python orchestration uses `behav3d_commands.Session` and its subsystem command
sets. Active sessions in `behav3d_orchestrator` derive from
`ControlAwareSession` and use `run_sync` and `run_group` for sequencing.

See:

- `agent_prompt.md` for the current command surface and orchestration model.
- `src/behav3d_commands/README.md` for the Session API.
- `src/behav3d_orchestrator/config/` for runtime sequence parameters.

## Current YAML inputs

YAML is still used as data by specific workflows, not as a generic command
language:

- `print_yaml_and_scan_sequence` reads configured target or segment data.
- `polyline_motion_sequence` reads `polylines:` data.
- `print_field_oriented_sequence_v2` consumes generated field target/segment
  files through `PrintSession`.
- `scan_yaml_targets_sequence` reads indexed `targets:` containing plane strings
  such as `O(0,80,30) Z(0,0.01,1)`; origins are expressed in millimetres.

Each workflow owns its parser and expected schema. Do not send retired generic
command-list YAML to these nodes.

## Dot target contract

Generated and user-authored dot targets use a flat `targets:` list:

```yaml
targets:
  - index: 0
    plane: "O(100.0,-800.0,120.0) Z(0.0,0.0,1.0)"
    volume_mm3: 2929.187507
```

- `index` controls execution order.
- `plane` uses millimetres for `O(...)`; `Z(...)` is the target normal.
- `volume_mm3` is optional and must be finite and positive when present.
- If `volume_mm3` is absent, dot printing uses the configured net `dot_steps`.
- If it is present, `PrintSession` converts it with `dot_steps_per_mm3`.
- Do not add width, DDS profile, deposition, or retract fields to this contract.

Variable-width field generation adds `volume_mm3` to flat dot targets. Fixed
width generation omits it. Nested `segments:` retain their `start`/`end` plane
contract and do not carry per-dot volume.
