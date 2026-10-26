"""Bounded POSIX observation worker isolation; no ROS or hardware imports."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import select
import selectors
import signal
import subprocess
import threading
import time

from .observation import MAX_REQUEST_BYTES, MAX_RESULT_BYTES, validate_request, validate_result

POLL_SECONDS = 0.02
CLEANUP_GRACE_SECONDS = 0.2
MAX_STDERR_BYTES = 65536


class WorkerError(RuntimeError):
    """An isolated worker failed without producing an acceptable result."""

    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def _guard(cancelled, deadline):
    if cancelled():
        raise WorkerError("CANCELLED", "Observation cancelled; late output discarded")
    if time.monotonic() >= deadline:
        raise WorkerError("TIMED_OUT", "Observation worker deadline exceeded")


def _group_alive(group):
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _signal_group(group, value):
    try:
        os.killpg(group, value)
        return True
    except ProcessLookupError:
        return True
    except OSError:
        return False


def _stop(process):
    """Bound cleanup of our new process group even after its leader exits.

    This is process-group cleanup, not a security sandbox. The configured worker
    must not daemonize or escape its session. No unrelated process group is used.
    A producer that leaves descendants after returning a result fails closed.
    """
    leader_exited = process.poll() is not None
    lingering = leader_exited and _group_alive(process.pid)
    clean = _signal_group(process.pid, signal.SIGTERM)
    end = time.monotonic() + CLEANUP_GRACE_SECONDS
    while _group_alive(process.pid) and time.monotonic() < end:
        process.poll()
        time.sleep(min(POLL_SECONDS, max(0.0, end - time.monotonic())))
    if _group_alive(process.pid):
        clean = _signal_group(process.pid, signal.SIGKILL) and clean
    try:
        process.wait(timeout=CLEANUP_GRACE_SECONDS)
    except subprocess.TimeoutExpired:
        clean = False
    return clean and not lingering


def _duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate worker JSON key")
        result[key] = value
    return result


def _nonfinite(_value):
    raise ValueError("Nonfinite worker JSON number")


def _decode_result(raw, request_id):
    """Reject ambiguous JSON before validating the exact result contract."""
    if len(raw) > MAX_RESULT_BYTES:
        raise ValueError("Worker result exceeds byte limit")
    value = json.loads(raw.decode("utf-8"), object_pairs_hook=_duplicate_keys,
                       parse_constant=_nonfinite)
    return validate_result(value, request_id)


def _write_request(pipe, payload, stopped, done, failed, deadline):
    """Nonblocking bounded writes; the main deadline never waits on stdin."""
    try:
        fd = pipe.fileno()
        position = 0
        while position < len(payload):
            if stopped.is_set() or time.monotonic() >= deadline:
                return
            try:
                _readable, writable, _errors = select.select([], [fd], [], POLL_SECONDS)
                if not writable:
                    continue
                count = os.write(fd, payload[position:position + 4096])
                if count <= 0:
                    failed.set()
                    return
                position += count
            except (BlockingIOError, InterruptedError):
                continue
    except (OSError, ValueError):
        if not stopped.is_set():
            failed.set()
    finally:
        # Popen uses unbuffered FileIO, so this has no buffered-reader lock.
        try:
            pipe.close()
        except (OSError, ValueError):
            pass
        done.set()


def run_observation_worker(request, *, python_executable, evidence_root,
                           source_roots=(), timeout_s=120.0,
                           cancelled=lambda: False, worker_script=None,
                           backend_mode="recorded", agent007_root="", dds_root="",
                           output_root=""):
    """Run one configured producer through bounded pipes.

    Worker executable/script/roots are deployment policy, never request fields.
    Request writes happen in a nonblocking writer thread; stdout and stderr are
    read by a bounded selector loop. Deadline/cancellation include result parsing
    and are rechecked after cleanup. No buffered reader threads can retain locks.
    POSIX session/process-group semantics are required; other platforms fail closed.
    """
    request = validate_request(request)
    if os.name != "posix":
        raise WorkerError("UNSUPPORTED", "POSIX observation worker isolation is required")
    if (type(timeout_s) not in (int, float) or not math.isfinite(timeout_s)
            or not 0 < timeout_s <= 3600):
        raise ValueError("worker timeout must be finite and in (0, 3600]")
    executable = Path(python_executable)
    root = Path(evidence_root)
    if not executable.is_absolute() or not executable.is_file():
        raise ValueError("explicit absolute worker Python executable required")
    if not root.is_absolute() or not root.is_dir():
        raise ValueError("explicit existing evidence_root required")
    if backend_mode not in {"recorded", "fresh_geometry"}:
        raise ValueError("Unsupported observation backend mode")
    policy_arguments = []
    if backend_mode == "fresh_geometry":
        if worker_script is not None:
            raise ValueError("Fresh analysis uses the fixed installed worker")
        for option, value in (("--agent007-root", agent007_root),
                              ("--dds-root", dds_root), ("--output-root", output_root)):
            path = Path(value)
            if not value or not path.is_absolute() or not path.is_dir():
                raise ValueError("Fresh analysis roots must be explicit existing directories")
            policy_arguments.extend([option, str(path)])
        script = Path(__file__).with_name("fresh_observation_worker.py")
    else:
        if any((agent007_root, dds_root, output_root)):
            raise ValueError("Fresh configuration requires explicit fresh_geometry mode")
        script = (Path(worker_script) if worker_script is not None
                  else Path(__file__).with_name("observation_worker.py"))
    if not script.is_absolute() or not script.is_file():
        raise ValueError("explicit existing worker script required")
    if not isinstance(source_roots, (tuple, list)) or len(source_roots) > 32:
        raise ValueError("source roots must be a bounded configured list")
    argv = [str(executable), "-B", str(script), "--evidence-root", str(root)]
    argv.extend(policy_arguments)
    for value in source_roots:
        source = Path(value)
        if not source.is_absolute() or not source.is_dir():
            raise ValueError("source roots must be explicit existing directories")
        argv.extend(["--source-root", str(source)])
    payload = json.dumps(request, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_REQUEST_BYTES:
        raise ValueError("observation request exceeds byte limit")
    deadline = time.monotonic() + timeout_s
    _guard(cancelled, deadline)
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    # The explicitly configured Python environment owns its dependencies.
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    stopped, writer_done, writer_failed = (threading.Event() for _ in range(3))
    writer = None
    buffers = [bytearray(), bytearray()]
    try:
        selector = selectors.DefaultSelector()
    except OSError as exc:
        raise WorkerError("FAILED", "Observation worker I/O setup failed") from exc
    try:
        process = subprocess.Popen(
            argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=environment, shell=False, start_new_session=True, bufsize=0, close_fds=True)
    except (OSError, ValueError) as exc:
        selector.close()
        raise WorkerError("FAILED", "Observation worker could not start") from exc
    clean = True
    try:
        for pipe in (process.stdin, process.stdout, process.stderr):
            os.set_blocking(pipe.fileno(), False)
        for index, pipe in enumerate((process.stdout, process.stderr)):
            selector.register(pipe, selectors.EVENT_READ, index)
        writer = threading.Thread(
            target=_write_request,
            args=(process.stdin, payload, stopped, writer_done, writer_failed, deadline),
            name="behav3d-observation-stdin", daemon=True)
        try:
            writer.start()
        except RuntimeError as exc:
            raise WorkerError("FAILED", "Observation input writer could not start") from exc
        while True:
            _guard(cancelled, deadline)
            if writer_failed.is_set():
                raise WorkerError("FAILED", "Observation worker closed its input")
            returncode = process.poll()
            if returncode is not None and returncode != 0:
                raise WorkerError("FAILED", "Observation worker exited unsuccessfully")
            if returncode is not None and writer_done.is_set() and not selector.get_map():
                break
            wait = max(0.0, min(POLL_SECONDS, deadline - time.monotonic()))
            if not selector.get_map():
                time.sleep(wait)
                continue
            try:
                events = selector.select(wait)
            except InterruptedError:
                continue
            for key, _mask in events:
                pipe, index = key.fileobj, key.data
                try:
                    chunk = os.read(pipe.fileno(), 8192)
                except (BlockingIOError, InterruptedError):
                    continue
                if not chunk:
                    selector.unregister(pipe)
                    pipe.close()
                    continue
                limit = MAX_RESULT_BYTES if index == 0 else MAX_STDERR_BYTES
                if len(buffers[index]) + len(chunk) > limit:
                    raise WorkerError("FAILED", "Observation worker exceeded output bounds")
                buffers[index].extend(chunk)
        _guard(cancelled, deadline)
        try:
            result = _decode_result(bytes(buffers[0]), request["request_id"])
        except (ValueError, TypeError, UnicodeError, RecursionError, OverflowError) as exc:
            raise WorkerError("FAILED", "Invalid observation worker result") from exc
        _guard(cancelled, deadline)
    except OSError as exc:
        raise WorkerError("FAILED", "Observation worker I/O failed") from exc
    finally:
        stopped.set()
        clean = _stop(process)
        if writer is not None and writer.ident is not None:
            writer.join(timeout=CLEANUP_GRACE_SECONDS)
            clean = clean and not writer.is_alive()
        selector.close()
        for pipe in (process.stdin, process.stdout, process.stderr):
            if pipe is not None and not pipe.closed:
                try:
                    pipe.close()
                except (OSError, ValueError):
                    clean = False
    _guard(cancelled, deadline)
    if not clean:
        raise WorkerError("FAILED", "Observation worker cleanup incomplete or descendants remained")
    return result
