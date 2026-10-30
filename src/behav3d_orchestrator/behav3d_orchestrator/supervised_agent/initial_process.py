"""Bounded POSIX startup worker runner, independent of observation wire types."""

import json
import math
import os
import selectors
import subprocess
import time
from pathlib import Path

from .initial_workflow import validate_request, validate_result
from .observation import _decode
from .process_runner import WorkerError, _guard, _stop


def run_initial_worker(
    request,
    *,
    python_executable,
    input_roots,
    output_root,
    agent007_root,
    behav3d_root,
    journal_path,
    timeout_s=120.0,
    cancelled=lambda: False,
):
    request = validate_request(request)
    if (
        os.name != "posix"
        or type(timeout_s) not in (int, float)
        or not math.isfinite(timeout_s)
        or not 0 < timeout_s <= 3600
    ):
        raise ValueError("POSIX and finite bounded startup timeout required")
    python = Path(python_executable)
    if not python.is_absolute() or not python.is_file():
        raise ValueError("Explicit scientific Python executable required")
    if not isinstance(input_roots, (tuple, list)) or not 0 < len(input_roots) <= 32:
        raise ValueError("Bounded startup input roots required")
    argv = [str(python), "-B", str(Path(__file__).with_name("initial_worker.py"))]
    for name, value in [("--input-root", root) for root in input_roots] + [
        ("--output-root", output_root),
        ("--agent007-root", agent007_root),
        ("--behav3d-root", behav3d_root),
    ]:
        path = Path(value)
        if not path.is_absolute() or not path.is_dir():
            raise ValueError("Explicit existing startup directories required")
        argv.extend([name, str(path)])
    journal = Path(journal_path)
    if not journal.is_absolute():
        raise ValueError("Explicit startup journal path required")
    argv.extend(["--journal-path", str(journal)])
    payload = json.dumps(request, allow_nan=False).encode()
    deadline = time.monotonic() + timeout_s
    _guard(cancelled, deadline)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)
    selector = selectors.DefaultSelector()
    try:
        process = subprocess.Popen(
            argv,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            close_fds=True,
            shell=False,
            start_new_session=True,
            env=env,
        )
    except (OSError, ValueError) as error:
        selector.close()
        raise WorkerError("FAILED", "Startup worker could not start") from error
    buffers = [bytearray(), bytearray()]
    position = 0
    clean = False
    try:
        for pipe in (process.stdin, process.stdout, process.stderr):
            os.set_blocking(pipe.fileno(), False)
        selector.register(process.stdin, selectors.EVENT_WRITE, -1)
        selector.register(process.stdout, selectors.EVENT_READ, 0)
        selector.register(process.stderr, selectors.EVENT_READ, 1)
        while selector.get_map() or process.poll() is None:
            _guard(cancelled, deadline)
            if process.poll() not in (None, 0):
                raise WorkerError("FAILED", "Startup worker exited unsuccessfully")
            events = selector.select(max(0, min(0.02, deadline - time.monotonic())))
            for key, _mask in events:
                pipe, channel = key.fileobj, key.data
                try:
                    if channel == -1:
                        count = os.write(
                            pipe.fileno(), payload[position : position + 4096]
                        )
                        position += count
                        if position == len(payload):
                            selector.unregister(pipe)
                            pipe.close()
                    else:
                        chunk = os.read(pipe.fileno(), 8192)
                        if not chunk:
                            selector.unregister(pipe)
                            pipe.close()
                        else:
                            buffers[channel].extend(chunk)
                            if len(buffers[channel]) > (
                                16384 if channel == 0 else 65536
                            ):
                                raise WorkerError(
                                    "FAILED", "Startup worker output exceeds limits"
                                )
                except (BlockingIOError, InterruptedError):
                    continue
        _guard(cancelled, deadline)
        try:
            result = validate_result(_decode(bytes(buffers[0]), 16384), request)
        except (ValueError, TypeError, KeyError) as error:
            raise WorkerError("FAILED", "Invalid startup worker result") from error
    except OSError as error:
        raise WorkerError("FAILED", "Startup worker I/O failed") from error
    finally:
        clean = _stop(process)
        selector.close()
        for pipe in (process.stdin, process.stdout, process.stderr):
            if not pipe.closed:
                pipe.close()
    _guard(cancelled, deadline)
    if not clean:
        raise WorkerError("FAILED", "Startup worker cleanup incomplete")
    return result
