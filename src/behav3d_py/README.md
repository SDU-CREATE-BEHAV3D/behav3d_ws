# behav3d_py (ROS 2 package)

High-level Python API and demo nodes for motion, printing, and capture workflows. This package is the ROS 2 entry point; the Python modules live in `src/behav3d_py/behav3d_py/`.

## Entry points (nodes)
- `run_yaml_test`: execute a YAML command list via `SequenceParser`.
- `print_test`: interactive print action test (keyboard control).
- `modbus_test`: low-level Modbus coil/register test node.

Run with:
```
ros2 run behav3d_py <node_name>
```

## Library modules
- `behav3d_py/commands.py`: command FIFO + service/action wrappers.
- `behav3d_py/yaml_parser.py`: YAML-to-Commands mapper.
- `behav3d_py/macros.py`: higher-level scanning macros (fib-scan).

For function-level documentation, see `src/behav3d_py/behav3d_py/README.md`.

Note: `move_and_print_test` and `handeye_capture_sequence` moved to `behav3d_examples`.

## Actionable improvements
- Convert demo scripts into launchable examples with ROS params instead of in-file constants.
- Add a minimal integration test that exercises `Commands` with mocked services/actions.
- Document the expected TF frames and default `eef` names in one place.
- Provide a sample YAML for `run_yaml_test` under a `examples/` folder.
- Ensure `setup.py` exposes all demo nodes via `console_scripts`.
- Add parameter YAMLs for each demo node (speed, frames, IPs, default folders).
- Add docstrings for demo nodes outlining dependencies and expected running stack.
