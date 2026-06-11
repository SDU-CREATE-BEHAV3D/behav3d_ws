# Arduino CLI notes

This folder contains Arduino sketches used for extruder firmware/debugging.

## Serial permission workaround

The `lab` user is a member of `dialout`, but some terminal/Codex sessions may
start without the refreshed group list. If `/dev/ttyUSB0` is not writable, run
Arduino commands inside a `newgrp dialout` shell.

Check the current session:

```bash
groups
ls -l /dev/ttyUSB0
test -w /dev/ttyUSB0 && echo writable || echo not-writable
```

If `dialout` is missing or the port says `not-writable`, use:

```bash
newgrp dialout
groups
test -w /dev/ttyUSB0 && echo writable || echo not-writable
```

For one-off commands from scripts/Codex:

```bash
newgrp dialout <<'EOF'
arduino-cli board list
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ino_files/stepper_pot_simple
EOF
```

Use the correct FQBN for the board:

```bash
arduino-cli board listall arduino:avr
```

Common examples:

```bash
arduino:avr:uno
arduino:avr:nano
arduino:avr:mega
```

## Compile examples

```bash
arduino-cli compile --fqbn arduino:avr:uno ino_files/stepper_pot_simple
arduino-cli compile --fqbn arduino:avr:mega ino_files/stepper_pot_simple
```

## Upload examples

```bash
newgrp dialout <<'EOF'
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ino_files/stepper_pot_simple
EOF
```

For Arduino Nano clones, the old bootloader may be needed:

```bash
newgrp dialout <<'EOF'
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old ino_files/stepper_pot_simple
EOF
```

## Ram extruder Nano setup

The current ram extruder debug board is an Arduino Nano clone on `/dev/ttyUSB0`
using the old bootloader:

```bash
arduino:avr:nano:cpu=atmega328old
```

Compile/upload the Timer1 + limit-switch sketch with:

```bash
newgrp dialout <<'EOF'
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old ino_files/stepper_pot_switch
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old ino_files/stepper_pot_switch
EOF
```

Current wiring:

```text
D3 -> STEP
D4 -> DIR
A0 -> potentiometer wiper
D2 -> NC limit switch sense
```

NC limit switch wiring with external pull-up:

```text
          +5V
           |
         5.6k
           |
D2 --------+-------- NC switch -------- GND
           |
        100 nF
           |
          GND
```

With this wiring:

```text
D2 LOW  = switch normal/closed
D2 HIGH = limit active, disconnected switch, or broken wire
```

Current mechanical assumptions in `stepper_pot_switch`:

```text
Stepper driver microstepping: 1600 pulses / motor rev
Gear reduction:              16:1
Lead screw:                  4 mm / screw rev
Linear travel per motor rev: 4 / 16 = 0.25 mm
Steps per mm:                1600 / 0.25 = 6400 steps/mm
Limit backoff distance:      4 mm = 25,600 steps
Limit backoff speed:         240 motor RPM ~= 1 mm/s
```

The limit input currently uses software debounce:

```text
LIMIT_DEBOUNCE_US = 10000 us = 10 ms
```

This filters short HIGH spikes on D2 before the firmware accepts the limit as
active. It was tested successfully even with the limit signal sharing a 6-core
cable bundle near the stepper wiring, where the raw signal had previously
caused false limit triggers.

For the prototype, the current working stack is:

```text
NC switch
5.6k external pull-up to +5V
100 nF capacitor between D2 and GND
10 ms software debounce
```

For the final machine, prefer separate wiring for motor and signals:

```text
Stepper motor: its own cable, with coil pairs kept together/twisted if possible
Limit switch: D2 + GND as a twisted pair, shielded if the run is long/noisy
Potentiometer: +5V + A0 + GND away from motor power wiring
```
The direction constants currently match the tested wiring:

```cpp
const bool DIR_CLOCKWISE = LOW;
const bool DIR_COUNTERCLOCKWISE = HIGH;
```

If the backoff moves toward the switch instead of away from it, swap those two
constants and re-upload.

## Ram extruder Controllino setup

`ino_files/StepperModbus_ram` is a Controllino MAXI Automation variant of the
existing `StepperModbus` sketch for the ram extruder.

It keeps the same ROS/Modbus contract as `StepperModbus`:

```text
Coils 0..4   -> D00..D04
Coil 10      -> EXTRUDE_MAN
Coil 11      -> PRG_STATE
Coil 12      -> EXTRUDE_PRG
Coil 13      -> QUEUE_PUSH
Coil 14      -> CANCEL_ALL
HR1          -> step_rate_hz
HR2          -> cfsMultiplier
HR3|HR4      -> 32-bit requested steps
IR0|IR1      -> 32-bit remaining steps (steps_accum)
```

Current Controllino wiring for the ram variant:

```text
D07 -> STEP
D04 -> DIR
D05 -> ENA (active LOW)
R8  -> driver power relay
DI0 -> NC limit switch input
```

Recommended limit-switch wiring on the Controllino MAXI:

```text
+24V ---- NC switch ---- DI0
```

With that wiring:

```text
DI0 HIGH = switch normal/closed
DI0 LOW  = limit active, open switch, or broken wire
```

The current `StepperModbus_ram` behavior is:

```text
Debounce:                10 ms software debounce
Limit backoff distance:  4 mm
Soft zero:               set after the 4 mm backoff completes
Total soft travel:       305 mm from soft zero
Steps per mm:            6400
Soft limit max steps:    305 * 6400 = 1,952,000
```

Use this conversion when commanding `print_steps` from ROS:

```text
1 mm  = 6,400 steps
2 mm  = 12,800 steps
4 mm  = 25,600 steps
10 mm = 64,000 steps
```

The soft-limit direction assumption matches the tested Nano wiring:

```text
DIR LOW  = move away from the limit switch
DIR HIGH = move toward the limit switch
```

If the physical direction is inverted on the Controllino build, update:

```cpp
DIR_AWAY_FROM_LIMIT_STATE
DIR_TOWARD_LIMIT_STATE
```

in `ino_files/StepperModbus_ram/ModbusFunc.ino`.
