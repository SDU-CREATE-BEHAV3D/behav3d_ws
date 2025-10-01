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

### Print (queued in FIFO)

```yaml
- print: {secs: 1.2, speed: 900}
```

### Goto (absolute XYZ, optional RPY in radians)

```yaml
- goto: {x: -0.0500, y: 1.30, z: 0.31, exec: true}
```

#### Goto with orientation (any subset of rx/ry/rz; others default to 0)

```yaml
- goto: {x: 0.50, y: 1.50, z: 0.30, ry: 1.5708, exec: true}
```

#### Goto overriding motion mode just for this call

```yaml
- goto: {x: -0.0500, y: 1.00, z: 0.31, motion: LIN, exec: true}
```

#### Goto with non-default planning scales and eef

```yaml
- goto: {x: 0.50, y: 1.00, z: 0.31, eef: extruder_tcp, vel_scale: 0.1, accel_scale: 0.1, exec: true}
```

#### Goto (plan-only)

```yaml
- goto: {x: 0.25, y: 0.80, z: 0.40, exec: false}
```

### Goto with inline print (starts when motion goal is accepted)

```yaml
- goto:
    x: 0.50
    y: 1.00
    z: 0.31
    exec: true
    start_print: {secs: 1.5, speed: 1200, offset_s: 0.0, sync: accept}
```

---


## Notes / defaults

* `goto` units: meters for XYZ; **radians** for `rx/ry/rz` (unless `degrees: true` is present).
* If `rx/ry/rz` are omitted → identity orientation (`w=1`).
* `motion` accepts `PTP` or `LIN`. If omitted, current mode set by `- PTP` / `- LIN` is used.
* `EEF/SPD/ACC/TOTG` lines set persistent defaults for subsequent `goto` calls (can be overridden per-call).
* `print` outside `goto` is queued in the FIFO; `start_print` inside `goto` runs **concurrently** at motion start.
* `wait` is one-shot and holds the FIFO before the next command.


