"""Review an initial wall placement offline; never approve or generate targets.

Run with the configured scientific interpreter. Real ROS transport is provided
by initial_startup.launch.py; this small entry point is for recorded-scene review.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--frame", required=True)
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--artifact-id", required=True)
    parser.add_argument("--input-root", type=Path, action="append", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--agent007-root", type=Path, required=True)
    parser.add_argument("--behav3d-root", type=Path, required=True)
    args = parser.parse_args()
    checkout = Path(__file__).resolve().parents[1]
    if args.behav3d_root != checkout:
        parser.error("--behav3d-root must name this explicit canonical checkout")
    sys.path.insert(0, str(checkout / "src/behav3d_orchestrator"))
    from behav3d_orchestrator.supervised_agent.initial_workflow import (
        evaluate_initial_wall,
    )

    request = {
        "phase": "PROPOSE", "request_id": args.request_id,
        "artifact_id": args.artifact_id, "frame_id": args.frame,
        "scene_manifest_path": str(args.scene_manifest),
        "scene_manifest_sha256": hashlib.sha256(args.scene_manifest.read_bytes()).hexdigest(),
        "task_path": str(args.task),
        "task_sha256": hashlib.sha256(args.task.read_bytes()).hexdigest(),
        "proposal_path": "", "proposal_sha256": "",
    }
    result = evaluate_initial_wall(
        request, input_roots=[str(path) for path in args.input_root],
        output_root=str(args.output_root), agent007_root=str(args.agent007_root),
        behav3d_root=str(args.behav3d_root))
    print(json.dumps(result, indent=2))
    return 0 if result["success"] or result["status"] == "HOLD" else 1


if __name__ == "__main__":
    raise SystemExit(main())
