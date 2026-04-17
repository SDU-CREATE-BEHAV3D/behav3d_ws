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

The direction constants currently match the tested wiring:

```cpp
const bool DIR_CLOCKWISE = LOW;
const bool DIR_COUNTERCLOCKWISE = HIGH;
```

If the backoff moves toward the switch instead of away from it, swap those two
constants and re-upload.
