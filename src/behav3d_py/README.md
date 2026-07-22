# behav3d_py

Python package containing scalar-field research utilities and low-level print
diagnostic nodes.

Motion, sensing, printing, and orchestration use `behav3d_commands` and
`behav3d_orchestrator`. The former `Commands` FIFO, YAML parser, macros, and
their demo nodes have been removed.

## Entry point

- `modbus_test`: direct Modbus TCP coil/register test for the extruder
  controller.
- `print_test`: interactive test of the print action and its speed/status
  services. Requires `behav3d_print` to be running.

Run it with:

```bash
ros2 run behav3d_py <modbus_test|print_test>
```

This is a hardware diagnostic. Check its ROS parameters and controller address
before connecting it to the real extruder.

## Scalar-field utilities

The `behav3d_py/scalar_field/` tree contains standalone field construction,
contour extraction, candidate generation, simulation, visualization, and
collision-probe scripts. Its reusable algorithms live in
`scalar_field/lib_scalar/`.

See [`behav3d_py/scalar_field/README.md`](behav3d_py/scalar_field/README.md) for
the current scripts, inputs, and generated artifacts.
