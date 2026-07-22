# behav3d_py Python modules

This module namespace contains three independent areas:

- `modbus_test.py`: low-level ROS 2 diagnostic node for the Modbus extruder
  controller.
- `print_test.py`: interactive client for the print action and its
  configuration/status services.
- `scalar_field/`: standalone scalar-field experiments and the reusable
  `lib_scalar` algorithms used by the field-processing workflow.

The active robot command API is `behav3d_commands.Session`; active scan and
print flows live in `behav3d_orchestrator`. This package no longer provides the
legacy `Commands`, `SequenceParser`, or capture macros.

## Modbus diagnostic node

`ModbusTest` connects directly to Modbus TCP and exposes parameters for the
extruder coil and speed register. Run it with:

```bash
ros2 run behav3d_py modbus_test
```

Treat it as a hardware diagnostic rather than an orchestration interface.

## Interactive print test

`print_test.py` sends a timed extrusion goal to `behav3d_print`. During the
test, `u` and `d` adjust speed and Enter cancels the goal. Start the print node
first, then run:

```bash
ros2 run behav3d_py print_test
```

## Scalar-field code

The scalar-field scripts cover field positioning, heat/phi evaluation, contour
and candidate extraction, loop simulation, visualization, target YAML output,
and extruder collision probing.

Detailed documentation is in
[`scalar_field/README.md`](scalar_field/README.md), with library-level notes in
[`scalar_field/lib_scalar/README.md`](scalar_field/lib_scalar/README.md).
