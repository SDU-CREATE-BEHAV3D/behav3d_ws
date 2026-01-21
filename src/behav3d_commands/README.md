# behav3d_commands architecture

## Overview
`behav3d_commands` provides the low-level command layer and a queue-driven
orchestration surface for Behav3D systems. The main entry point is `Session`,
which owns a `SessionQueue`, a `CommandRouter`, and subsystem command sets
(`MotionCommands`, `CameraCommands`, `ExtruderCommands`, `UtilCommands`).

Higher-level orchestration and example sessions live in `behav3d_examples`.

## Core building blocks

### Session
`Session` wires the command sets to a shared queue and routes queue items to
their handlers. It exposes:
- `run_group(items)` to schedule a parallel group.
- `run_sync(item, timeout_s=None)` to block until a single item's `on_done` fires.
- `pause()` / `resume()` and `is_busy` / `is_paused`.
- `queue` for direct access to the `SessionQueue` when needed.

`run_sync` must not be called from the ROS executor thread; use a worker thread
or a multi-threaded executor.

### SessionQueue
`SessionQueue` is a FIFO queue with optional parallel groups:
- `enqueue(item)` adds one item.
- `prepend(item)` pushes one item to the head.
- `enqueue_group([...])` starts a set of items at the same time.

Groups are best-effort concurrency. There is no resource locking yet, so avoid
conflicting actions in the same group unless you know they are safe.

### QueueItem and Command
`QueueItem` is the unit of work:
```
QueueItem(kind, payload, cmd_kind=None, on_done=None)
```

- `kind` maps to a handler registered in `CommandRouter`.
- `cmd_kind` is the user-facing label in callbacks (defaults to `kind`).
- `on_done` receives a result dict when the command finishes.

`Command.finish_flag(...)` enforces the result contract and guarantees the queue
advance even if the callback raises.

## Command sets
Each `*Commands` class:
- Builds a `QueueItem` for its public methods.
- Enqueues by default, or returns a `QueueItem` when `enqueue=False`.
- Registers internal handlers with `CommandRouter`.

Example (build items, run in a group):
```python
session = Session(node)
move = session.motion.goto(x=0.5, y=1.0, z=0.3, enqueue=False)
print_item = session.extruder.print_steps(steps=2000, speed=500, enqueue=False)
session.run_group([move, print_item])
```

Example (blocking call):
```python
plan_item = session.motion.plan(x=0.5, y=1.0, z=0.3, enqueue=False)
result = session.run_sync(plan_item, timeout_s=10.0)
```

## Result contract
`on_done` receives a dict with:
- `ok` (bool), `kind` (str), `phase` (str), `error` (str or None)
- `metrics` (dict) and optional extras for command-specific data

## Extending the system
To add a new subsystem:
1. Create a new `*Commands` class.
2. Implement handlers (`_handle_*`) that accept `(payload, Command)`.
3. Add public methods that return `QueueItem` (and optionally enqueue).
4. Register handlers in `register(...)`, and wire the class in `Session`.

## Examples
`ScanSession` and `PrintSession` are example subclasses in `behav3d_examples`.
Example nodes that use them live in `behav3d_py`:
- `move_and_print_test`
- `handeye_capture_sequence`

## Related packages
`behav3d_utils` contains geometry helpers (Python port of `target_builder.cpp`).
Import from `behav3d_utils.target_builder` when you need pose/transform helpers.
