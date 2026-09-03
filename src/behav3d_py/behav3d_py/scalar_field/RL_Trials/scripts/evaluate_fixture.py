#!/usr/bin/env python3
"""Evaluate the selected field_state_init2 RL fixture."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Allow this file to be launched directly, without installing rl_trials or
# manually setting PYTHONPATH.
TRIALS_ROOT = Path(__file__).resolve().parents[1]
if str(TRIALS_ROOT) not in sys.path:
    sys.path.insert(0, str(TRIALS_ROOT))

from rl_trials.fixture_io import load_configured_geometry, load_experiment_config
from rl_trials.goal_evaluator import evaluate_goal_with_vertical_rays


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def main() -> None:
    default_config = TRIALS_ROOT / "configs" / "default_experiment.yaml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--workspace-root", type=Path, default=workspace_root_from_script())
    args = parser.parse_args()

    config = load_experiment_config(args.config)
    workspace = args.workspace_root.resolve()
    goal_vertices, goal_faces, scan_vertices, scan_faces = load_configured_geometry(
        config,
        workspace,
    )
    completion = config["completion"]
    result = evaluate_goal_with_vertical_rays(
        goal_vertices,
        goal_faces,
        scan_vertices,
        scan_faces,
        fill_tolerance_m=1e-3 * float(completion["fill_tolerance_mm"]),
    )
    print(json.dumps(result.metrics.to_dict(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
