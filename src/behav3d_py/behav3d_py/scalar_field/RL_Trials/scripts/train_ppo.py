#!/usr/bin/env python3
"""Train a from-scratch PPO policy on direct DDS bead actions."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

TRIALS_ROOT = Path(__file__).resolve().parents[1]
if str(TRIALS_ROOT) not in sys.path:
    sys.path.insert(0, str(TRIALS_ROOT))
SCALAR_FIELD_ROOT = TRIALS_ROOT.parent
if str(SCALAR_FIELD_ROOT) not in sys.path:
    sys.path.insert(0, str(SCALAR_FIELD_ROOT))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from rl_trials.gym_env import DirectBeadDepositionEnv
from rl_trials.ppo_training import (
    DDSMetricsCallback,
    build_ppo_model,
    validate_ppo_batching,
)


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[6]


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _default_run_name() -> str:
    return datetime.now().strftime("ppo_%Y%m%d_%H%M%S")


def _prepare_run_directory(root: Path, name: str) -> Path:
    if not name or name in {".", ".."} or Path(name).name != name:
        raise ValueError("run name must be one non-empty path component")
    run_dir = root.expanduser().resolve() / name
    if run_dir.exists() and any(run_dir.iterdir()):
        raise FileExistsError(f"training run directory is not empty: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


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
    parser.add_argument(
        "--output-root",
        type=Path,
        default=TRIALS_ROOT / "outputs" / "training",
    )
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--total-timesteps", type=int, default=10_240)
    parser.add_argument("--n-steps", type=int, default=128)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--n-epochs", type=int, default=10)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument("--ent-coef", type=float, default=0.0)
    parser.add_argument("--vf-coef", type=float, default=0.5)
    parser.add_argument(
        "--action-log-std-init",
        type=float,
        nargs=5,
        metavar=("SOURCE", "HEIGHT", "ORIENT_X", "ORIENT_Y", "WIDTH"),
        default=(-1.5, -2.5, -2.0, -2.0, -2.5),
        help="Initial gSDE log standard deviation for each action dimension.",
    )
    parser.add_argument(
        "--sde-sample-freq",
        type=int,
        default=1,
        help="Draw new state-dependent exploration noise every N environment steps.",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--checkpoint-freq", type=int, default=1_024)
    parser.add_argument("--resume", type=Path, default=None)
    parser.add_argument(
        "--disable-collision",
        action="store_true",
        help="Geometry-only debugging mode; do not use for a final safety policy.",
    )
    args = parser.parse_args()

    if args.n_epochs <= 0:
        parser.error("--n-epochs must be > 0")
    if args.checkpoint_freq <= 0:
        parser.error("--checkpoint-freq must be > 0")
    run_name = args.run_name or _default_run_name()
    try:
        run_dir = _prepare_run_directory(args.output_root, run_name)
    except (ValueError, FileExistsError) as exc:
        parser.error(str(exc))

    base_env = DirectBeadDepositionEnv.from_config(
        args.config,
        workspace_root=args.workspace_root,
        enable_collision=not args.disable_collision,
    )
    env = Monitor(
        base_env,
        filename=str(run_dir / "monitor.csv"),
        info_keywords=(
            "completed_area_fraction",
            "accepted_deposits",
            "rejected_steps",
        ),
    )

    if args.resume is None:
        try:
            validate_ppo_batching(
                total_timesteps=args.total_timesteps,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
            )
            model = build_ppo_model(
                env,
                learning_rate=args.learning_rate,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                n_epochs=args.n_epochs,
                gamma=args.gamma,
                gae_lambda=args.gae_lambda,
                clip_range=args.clip_range,
                ent_coef=args.ent_coef,
                vf_coef=args.vf_coef,
                action_log_std_init=tuple(args.action_log_std_init),
                sde_sample_freq=args.sde_sample_freq,
                seed=args.seed,
                device=args.device,
                verbose=1,
            )
        except ValueError as exc:
            env.close()
            parser.error(str(exc))
        reset_num_timesteps = True
    else:
        model_path = args.resume.expanduser().resolve()
        if not model_path.is_file():
            env.close()
            parser.error(f"resume model does not exist: {model_path}")
        model = PPO.load(model_path, env=env, device=args.device)
        if not model.policy.squash_output:
            env.close()
            parser.error("resume model does not use the required squashed action policy")
        if args.total_timesteps <= 0 or args.total_timesteps % model.n_steps != 0:
            env.close()
            parser.error("additional timesteps must be a positive multiple of model.n_steps")
        reset_num_timesteps = False

    model.set_logger(configure(str(run_dir / "logs"), ["stdout", "csv"]))
    checkpoint_callback = CheckpointCallback(
        save_freq=args.checkpoint_freq,
        save_path=str(run_dir / "checkpoints"),
        name_prefix="ppo_dds",
    )
    callbacks = CallbackList([checkpoint_callback, DDSMetricsCallback()])

    metadata = {
        "arguments": _jsonable(vars(args)),
        "run_directory": str(run_dir),
        "policy": "MultiInputPolicy",
        "squash_output": bool(model.policy.squash_output),
        "use_sde": bool(model.use_sde),
        "collision_enabled": not args.disable_collision,
        "resolved_ppo": {
            "n_steps": int(model.n_steps),
            "batch_size": int(model.batch_size),
            "n_epochs": int(model.n_epochs),
            "gamma": float(model.gamma),
            "gae_lambda": float(model.gae_lambda),
            "action_log_std_mean": (
                model.policy.log_std.detach().mean(dim=0).cpu().tolist()
            ),
            "sde_sample_freq": int(model.sde_sample_freq),
        },
    }
    (run_dir / "run_config.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    shutil.copy2(args.config, run_dir / "experiment_config.yaml")

    started = time.perf_counter()
    interrupted = False
    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callbacks,
            reset_num_timesteps=reset_num_timesteps,
            progress_bar=False,
        )
    except KeyboardInterrupt:
        interrupted = True
    finally:
        model_path = run_dir / ("interrupted_model" if interrupted else "final_model")
        model.save(model_path)
        env.close()

    summary = {
        "elapsed_seconds": time.perf_counter() - started,
        "interrupted": interrupted,
        "model_path": str(model_path.with_suffix(".zip")),
        "num_timesteps": int(model.num_timesteps),
    }
    (run_dir / "training_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
