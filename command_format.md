**YAML snippets** (one per command/config) matching current API and behaviors. Keep as **list items** under a sequence.

### Motion mode & config can be used at the start(stateful “setters”)

```yaml
- PTP                 # set default motion planner to PTP
- LIN                 # set default motion planner to LIN
- EEF: extruder_tcp   # set default end-effector
- SPD: 0.10           # default velocity_scale (0..1)
- ACC: 0.10           # default accel_scale (0..1)
- TOTG: on            #TODO (placeholder if you wire a time-optimizer toggle)
```

### Home

```yaml
- home: {duration_s: 10.0}
```

### Wait (queue delay)

```yaml
- wait: 2.5           # seconds
```

### printTime (queued in FIFO)

Runs a timed extrusion via the `print` action. The FIFO will only continue after the action finishes.

```yaml
- printTime: {secs: 1.2, speed: 900}
```

**Fields**

* `secs` (float): Duration in seconds.
* `speed` (int): Extrusion speed (implementation-specific units).
* `use_previous_speed` (bool, optional, default: false).

---

### printSteps (queued in FIFO)

Runs a step-count extrusion via the `print_steps` action. The FIFO will only continue after the action finishes.

```yaml
- printSteps: {steps: 6000, speed: 900}
```

**Fields**

* `steps` (int): Number of steps to extrude.
* `speed` (int): Extrusion speed (implementation-specific units).
* `use_previous_speed` (bool, optional, default: false).

---

### goto (absolute XYZ, optional RPY in radians)

Plans and (optionally) executes a robot motion in the `world` frame.

```yaml
- goto: {x: -0.0500, y: 1.30, z: 0.31, exec: true}
```

#### goto with orientation (any subset of rx/ry/rz; others default to 0)

```yaml
- goto: {x: 0.50, y: 1.50, z: 0.30, ry: 1.5708, exec: true}
```

#### goto overriding motion mode for this call

```yaml
- goto: {x: -0.0500, y: 1.00, z: 0.31, motion: LIN, exec: true}
```

#### goto with non-default planning scales and eef

```yaml
- goto: {x: 0.50, y: 1.00, z: 0.31, eef: extruder_tcp, vel_scale: 0.1, accel_scale: 0.1, exec: true}
```

#### goto (plan-only, does not execute)

```yaml
- goto: {x: 0.25, y: 0.80, z: 0.40, exec: false}
```

---

### goto with inline print (concurrent to motion)

Starts extrusion when the motion goal is **accepted** (does not block the FIFO; runs in parallel to the trajectory). Use either **time** or **steps** inside `start_print`.

**Time-based inline print**

```yaml
- goto:
    x: 0.50
    y: 1.00
    z: 0.31
    exec: true
    start_print: {secs: 1.5, speed: 1200, offset_s: 0.0, sync: accept}
```

**Steps-based inline print**

```yaml
- goto:
    x: 0.50
    y: 1.00
    z: 0.36
    exec: true
    start_print: {steps: 1800, speed: 600, offset_s: 0.0, sync: accept}
```

**start_print fields**

* Use **either** `secs` (float) **or** `steps` (int).
* `speed` (int): Extrusion speed (implementation-specific units).
* `offset_s` (float, optional, default: 0.0): Delay after goal acceptance before starting extrusion.
* `sync` (string, optional, default: `accept`): Currently supports `accept` (trigger at motion goal acceptance).

---

### Pose (queued; query EEF/link pose)

```yaml
- pose: extruder_tcp
# Queries pose of 'extruder_tcp' in 'world' using MoveIt (default).
```

#### Pose with explicit base frame (MoveIt service)

```yaml
- pose: { eef: extruder_tcp, base: ur20_base_link }
# Express pose of 'extruder_tcp' in 'ur20_base_link' (MoveIt service).
```

#### Pose via TF (latest transform)

```yaml
- pose: { eef: extruder_tcp, base: ur20_base_link, tf: true }
# Uses TF2 instead of MoveIt. If 'base' is omitted/empty, defaults to 'world'.
```

#### Pose via TF with default base (world)

```yaml
- pose: { eef: extruder_tcp, tf: true }
# Equivalent to base='world' in TF path.
```

---

## Notes for `pose`

* **Queued by default:** `pose` respects the FIFO, executes exactly where you place it in the sequence.
* **Backends:**

  * `tf: false` (default) → uses your MoveIt pose service (`GetLinkPose`); `base: null` means “planning frame”.
  * `tf: true` → uses TF2 (`lookup_transform`) at **latest** time; if `base` is empty/omitted, it defaults to `'world'` (adjust in code if your base is different).
* **Keys:**

  * Scalar form: `- pose: extruder_tcp` is shorthand for `{ eef: extruder_tcp, base: world, tf: false }`.
  * Mapping form supports: `eef` (or `link`), `base`, and `tf`.
* **Result:** your callback receives a payload with `pose` (PoseStamped), `base_frame`, `link`, and `metrics.source` (`moveit` or `tf`).
* **Common uses:** sanity checks after a move, logging end effector drift, verifying tool offsets.

---

## Example mini-sequence with `pose`

```yaml
- LIN
- EEF: extruder_tcp
- SPD: 0.10
- ACC: 0.10

- home: {duration_s: 2.0}

- goto: {x: -0.1965, y: 0.9550, z: -0.0440, exec: true}
- pose: extruder_tcp                      # MoveIt, in 'world'

- goto: {x: -0.1965, y: 0.9550, z: 0.0000, exec: true}
- wait: 1.0
- pose: { eef: extruder_tcp, base: ur20_base_link }  # MoveIt, in base link

- goto: {x: -0.2050, y: 0.9550, z: 0.0000, exec: true}
- wait: 1.0
- pose: { eef: extruder_tcp, base: ur20_base_link, tf: true }  # TF, latest

- home: {duration_s: 4.0}
```
---

### Input (pause sequence until ENTER)

```yaml
- input
# Waits for user to press ENTER before continuing.
```

#### Input with custom prompt

```yaml
- input: { prompt: "Press ENTER to continue..." }
# Displays the given message, then waits for ENTER.
```

---

### Capture (image or data acquisition)

#### Capture with selected streams

```yaml
- capture: { rgb: true, depth: true, ir: true }
# Captures RGB, depth, and IR images using the configured service.
```

#### Capture including pose

```yaml
- capture: { rgb: true, depth: true, ir: true, pose: true }
# Captures RGB/depth/IR and records the current pose.
```

#### Capture with folder change

```yaml
- capture: { folder: "/data/folder_name" }
# Sets the active capture folder before running the capture.
```

---

### Notes

* `input` pauses the FIFO until ENTER is pressed (no key mapping).
* `capture` calls the capture service; omitted keys default to `false`.

---


## Notes / defaults

* `goto` units: meters for XYZ; **radians** for `rx/ry/rz` (unless `degrees: true` is present).
* If `rx/ry/rz` are omitted → identity orientation (`w=1`).
* `motion` accepts `PTP` or `LIN`. If omitted, current mode set by `- PTP` / `- LIN` is used.
* `EEF/SPD/ACC/TOTG` lines set persistent defaults for subsequent `goto` calls (can be overridden per-call).
* `print` outside `goto` is queued in the FIFO; `start_print` inside `goto` runs **concurrently** at motion start.
* `wait` is one-shot and holds the FIFO before the next command.


