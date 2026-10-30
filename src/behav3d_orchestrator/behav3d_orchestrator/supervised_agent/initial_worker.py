"""Fixed scientific startup worker: bounded JSON, no ROS or physical clients."""

import argparse
import json
import signal
import sys
from pathlib import Path

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from behav3d_orchestrator.supervised_agent.fresh_observation_worker import (
    _analysis_stdout_to_stderr,
)
from behav3d_orchestrator.supervised_agent.initial_workflow import (
    error_result,
    evaluate_initial_wall,
    generation_authorized,
    validate_request,
)
from behav3d_orchestrator.supervised_agent.observation import _decode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", action="append", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--agent007-root", required=True)
    parser.add_argument("--behav3d-root", required=True)
    parser.add_argument("--journal-path", required=True)
    args = parser.parse_args(argv)
    stopped = [False]

    def cancel(*_args):
        stopped[0] = True

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, cancel)
    request_id = ""
    try:
        request = validate_request(_decode(sys.stdin.buffer.read(16385), 16384))
        request_id = request["request_id"]
        if request["phase"] == "GENERATE" and not generation_authorized(
            args.journal_path, request
        ):
            raise ValueError("No durable exact-placement approval is in flight")

        def cancelled():
            return stopped[0] or (
                request["phase"] == "GENERATE"
                and not generation_authorized(args.journal_path, request)
            )

        with _analysis_stdout_to_stderr():
            result = evaluate_initial_wall(
                request,
                input_roots=args.input_root,
                output_root=args.output_root,
                agent007_root=args.agent007_root,
                cancelled=cancelled,
                behav3d_root=args.behav3d_root,
                approval_journal_path=args.journal_path,
            )
    except (OSError, ValueError, TypeError, KeyError):
        result = error_result(
            request_id,
            "REJECTED",
            "Invalid startup request or no durable generation approval.",
        )
    sys.stdout.write(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n")
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
