#!/usr/bin/env python3
"""Run deterministic or random headless Gymnasium DDS steps."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

TRIALS_ROOT = Path(__file__).resolve().parents[1]
if str(TRIALS_ROOT) not in sys.path:
    sys.path.insert(0, str(TRIALS_ROOT))
SCALAR_FIELD_ROOT = TRIALS_ROOT.parent
if str(SCALAR_FIELD_ROOT) not in sys.path:
    sys.path.insert(0, str(SCALAR_FIELD_ROOT))

from rl_trials.gym_env import DirectBeadDepositionEnv


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=TRIALS_ROOT / "configs" / "default_experiment.yaml",
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=workspace_root_from_script(),
    )
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--action",
        type=float,
        nargs=5,
        metavar=("SOURCE", "HEIGHT", "ORIENT_X", "ORIENT_Y", "WIDTH"),
        default=None,
        help="Repeat one explicit normalized action; default is the neutral action.",
    )
    parser.add_argument(
        "--random",
        action="store_true",
        help="Sample actions from the Box instead of repeating a fixed action.",
    )
    parser.add_argument(
        "--disable-collision",
        action="store_true",
        help="Skip the extruder proxy for a faster geometry-only smoke test.",
    )
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be > 0")
    if args.random and args.action is not None:
        parser.error("--random and --action are mutually exclusive")

    env = DirectBeadDepositionEnv.from_config(
        args.config,
        workspace_root=args.workspace_root,
        enable_collision=not args.disable_collision,
    )
    env.action_space.seed(args.seed)
    _, reset_info = env.reset(seed=args.seed)
    print(json.dumps({"event": "reset", **_jsonable(reset_info)}, sort_keys=True))

    fixed_action = np.asarray(
        np.zeros(5) if args.action is None else args.action,
        dtype=np.float32,
    )
    try:
        for step_index in range(args.steps):
            action = env.action_space.sample() if args.random else fixed_action
            _, reward, terminated, truncated, info = env.step(action)
            payload = {
                "event": "step",
                "step": step_index,
                "action": action,
                "reward": reward,
                "terminated": terminated,
                "truncated": truncated,
                **info,
            }
            print(json.dumps(_jsonable(payload), sort_keys=True))
            if terminated or truncated:
                break
    finally:
        env.close()


if __name__ == "__main__":
    main()
