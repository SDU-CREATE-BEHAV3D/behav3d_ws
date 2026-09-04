#!/usr/bin/env python3
"""Evaluate a saved PPO policy in deterministic headless DDS episodes."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

TRIALS_ROOT = Path(__file__).resolve().parents[1]
if str(TRIALS_ROOT) not in sys.path:
    sys.path.insert(0, str(TRIALS_ROOT))
SCALAR_FIELD_ROOT = TRIALS_ROOT.parent
if str(SCALAR_FIELD_ROOT) not in sys.path:
    sys.path.insert(0, str(SCALAR_FIELD_ROOT))

from stable_baselines3 import PPO

from rl_trials.gym_env import DirectBeadDepositionEnv


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def _to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
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
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Optional evaluation cap; the environment limits still apply.",
    )
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Sample the policy instead of evaluating its deterministic action.",
    )
    parser.add_argument(
        "--disable-collision",
        action="store_true",
        help="Geometry-only debugging mode; keep collision enabled for comparison.",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if args.episodes <= 0:
        parser.error("--episodes must be > 0")
    if args.max_steps is not None and args.max_steps <= 0:
        parser.error("--max-steps must be > 0")
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        parser.error(f"model does not exist: {model_path}")

    env = DirectBeadDepositionEnv.from_config(
        args.config,
        workspace_root=args.workspace_root,
        enable_collision=not args.disable_collision,
    )
    try:
        model = PPO.load(model_path, env=env, device=args.device)
        if not model.policy.squash_output:
            parser.error("model does not use the required squashed action policy")

        episodes: list[dict[str, Any]] = []
        for episode_index in range(args.episodes):
            observation, reset_info = env.reset(seed=args.seed + episode_index)
            total_reward = 0.0
            accepted = 0
            rejection_counts: Counter[str] = Counter()
            final_info: dict[str, Any] = reset_info
            terminated = False
            truncated = False
            step_count = 0
            while not (terminated or truncated):
                if args.max_steps is not None and step_count >= args.max_steps:
                    break
                action, _ = model.predict(
                    observation,
                    deterministic=not args.stochastic,
                )
                observation, reward, terminated, truncated, final_info = env.step(action)
                total_reward += float(reward)
                accepted += int(bool(final_info.get("accepted", False)))
                rejection_counts.update(final_info.get("rejection_reasons", ()))
                step_count += 1

            episodes.append(
                {
                    "episode": episode_index,
                    "seed": args.seed + episode_index,
                    "steps": step_count,
                    "total_reward": total_reward,
                    "accepted": accepted,
                    "rejected": step_count - accepted,
                    "rejection_counts": dict(rejection_counts),
                    "completed_area_fraction": final_info.get(
                        "completed_area_fraction"
                    ),
                    "terminated": terminated,
                    "truncated": truncated,
                    "termination_reason": final_info.get("termination_reason"),
                    "truncation_reasons": final_info.get("truncation_reasons", ()),
                    "stopped_by_max_steps": bool(
                        not (terminated or truncated)
                        and args.max_steps is not None
                        and step_count >= args.max_steps
                    ),
                }
            )
    finally:
        env.close()

    result = {
        "model": str(model_path),
        "deterministic": not args.stochastic,
        "collision_enabled": not args.disable_collision,
        "episodes": episodes,
    }
    rendered = json.dumps(_to_jsonable(result), indent=2, sort_keys=True)
    if args.output is not None:
        output_path = args.output.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
