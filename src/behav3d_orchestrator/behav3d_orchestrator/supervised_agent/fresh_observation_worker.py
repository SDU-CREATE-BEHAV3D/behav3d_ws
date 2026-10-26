"""Fixed local fresh-analysis worker; one bounded request and exact result.

All code roots/output policy are configured by the operator, not supplied in
the request. The outer POSIX runner provides the deadline and group cleanup.
No ROS, robot commands, external models or fabrication authorization is used.
"""
import argparse
import ctypes
import json
import os
import signal
import sys
from contextlib import contextmanager

if __package__:
    from .fresh_observation import evaluate_fresh_observation
    from .observation import MAX_REQUEST_BYTES, decode_request, result_for_error
else:
    from fresh_observation import evaluate_fresh_observation
    from observation import MAX_REQUEST_BYTES, decode_request, result_for_error


class _Parser(argparse.ArgumentParser):
    def error(self, _message):
        raise ValueError("Invalid fresh worker CLI configuration")


@contextmanager
def _analysis_stdout_to_stderr():
    """Reserve fd 1 for JSON, including C/C++ geometry-library progress."""
    # Native stdio may buffer progress until process exit. Flush before each
    # descriptor change, otherwise old buffered bytes can escape onto JSON fd1.
    native_flush = ctypes.CDLL(None).fflush
    native_flush.argtypes = [ctypes.c_void_p]
    native_flush.restype = ctypes.c_int

    def flush():
        sys.stdout.flush()
        sys.stderr.flush()
        if native_flush(None) != 0:
            raise OSError("Native analysis output could not be flushed")

    flush()
    saved = os.dup(1)
    try:
        os.dup2(2, 1)
        yield
    finally:
        try:
            flush()
        finally:
            os.dup2(saved, 1)
            os.close(saved)


def main(argv=None):
    parser = _Parser(description=__doc__, add_help=False)
    for name in ("evidence-root", "agent007-root", "dds-root", "output-root"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--source-root", action="append", default=[])
    try:
        args = parser.parse_args(argv)
    except ValueError:
        result = result_for_error("", "FAILED", "Invalid fresh worker CLI configuration.")
        sys.stdout.write(json.dumps(result, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        return 0
    stopped = [False]

    def cancel(_signal, _frame):
        stopped[0] = True

    handlers = {}
    for name in ("SIGTERM", "SIGINT"):
        if hasattr(signal, name):
            value = getattr(signal, name)
            handlers[value] = signal.signal(value, cancel)
    try:
        try:
            request = decode_request(sys.stdin.buffer.read(MAX_REQUEST_BYTES + 1))
        except (ValueError, TypeError):
            result = result_for_error("", "REJECTED", "Invalid bounded JSON observation request.")
        else:
            print(json.dumps({"stage": "analysing_fresh_geometry", "message": "Verifying frozen inputs and generating local geometry evidence."}),
                  file=sys.stderr, flush=True)
            with _analysis_stdout_to_stderr():
                result = evaluate_fresh_observation(
                    request, args.evidence_root, cancelled=lambda: stopped[0], source_roots=args.source_root,
                    agent007_root=args.agent007_root, dds_root=args.dds_root, output_root=args.output_root)
        sys.stdout.write(json.dumps(result, allow_nan=False, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        return 0
    finally:
        for value, handler in handlers.items():
            signal.signal(value, handler)


if __name__ == "__main__":
    raise SystemExit(main())
