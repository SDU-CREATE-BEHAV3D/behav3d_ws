"""One bounded JSON request on stdin and one exact result on stdout.

Run with a configured isolated Python executable and --evidence-root. Optional
repeatable --source-root values are server policy, never values from the request.
No ROS, model, network, subprocess, hardware or source-file writes are performed.
"""
import argparse
import json
import signal
import sys

if __package__:
    from .observation import (
        MAX_REQUEST_BYTES, decode_request, evaluate_recorded_observation, result_for_error,
    )
else:
    from observation import (
        MAX_REQUEST_BYTES, decode_request, evaluate_recorded_observation, result_for_error,
    )


class _Parser(argparse.ArgumentParser):
    def error(self, _message):
        raise ValueError("Invalid worker CLI configuration")


def main(argv=None):
    parser = _Parser(description=__doc__, add_help=False)
    parser.add_argument("--evidence-root", required=True)
    parser.add_argument("--source-root", action="append", default=[])
    try:
        args = parser.parse_args(argv)
    except ValueError:
        result = result_for_error("", "FAILED", "Invalid worker CLI configuration.")
        sys.stdout.write(json.dumps(result, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        return 0
    stopped = [False]

    def cancel(_signal, _frame):
        stopped[0] = True

    previous_handlers = {}
    for name in ("SIGTERM", "SIGINT"):
        if hasattr(signal, name):
            value = getattr(signal, name)
            previous_handlers[value] = signal.signal(value, cancel)
    try:
        raw = sys.stdin.buffer.read(MAX_REQUEST_BYTES + 1)
        try:
            request = decode_request(raw)
        except (ValueError, TypeError):
            result = result_for_error("", "REJECTED", "Invalid bounded JSON observation request.")
        else:
            print(json.dumps({"stage": "verifying", "message": "Checking recorded evidence and source identities."}),
                  file=sys.stderr, flush=True)
            result = evaluate_recorded_observation(
                request, args.evidence_root, cancelled=lambda: stopped[0], source_roots=args.source_root)
        sys.stdout.write(json.dumps(result, allow_nan=False, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        return 0
    finally:
        for value, handler in previous_handlers.items():
            signal.signal(value, handler)


if __name__ == "__main__":
    raise SystemExit(main())
