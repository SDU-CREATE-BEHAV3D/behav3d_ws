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

Each workflow owns its parser and expected schema. Do not send retired generic
command-list YAML to these nodes.
