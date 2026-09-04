"""Stable-Baselines3 PPO construction for the direct DDS Gym environment."""

from __future__ import annotations

from collections import Counter
import math
from typing import Any

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
import torch


class DDSMetricsCallback(BaseCallback):
    """Log geometry outcomes and raw reward terms once per PPO rollout."""

    def __init__(self) -> None:
        super().__init__(verbose=0)
        self.last_rollout_metrics: dict[str, float] = {}
        self._transition_count = 0
        self._accepted_count = 0
        self._rejection_counts: Counter[str] = Counter()
        self._reward_sums: Counter[str] = Counter()
        self._height_samples_mm: list[float] = []
        self._width_samples_mm: list[float] = []
        self._contact_distance_samples_mm: list[float] = []
        self._contact_source_error_samples_mm: list[float] = []

    def _on_rollout_start(self) -> None:
        self._transition_count = 0
        self._accepted_count = 0
        self._rejection_counts.clear()
        self._reward_sums.clear()
        self._height_samples_mm.clear()
        self._width_samples_mm.clear()
        self._contact_distance_samples_mm.clear()
        self._contact_source_error_samples_mm.clear()

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", ()):
            if "accepted" not in info:
                continue
            self._transition_count += 1
            self._accepted_count += int(bool(info["accepted"]))
            self._rejection_counts.update(info.get("rejection_reasons", ()))
            for name, value in info.get("reward_components", {}).items():
                self._reward_sums[name] += float(value)
            decoded = info.get("decoded_action", {})
            if "height_m" in decoded:
                self._height_samples_mm.append(1e3 * float(decoded["height_m"]))
            if "width_m" in decoded:
                self._width_samples_mm.append(1e3 * float(decoded["width_m"]))
            contact = info.get("contact", {})
            hit_distance = float(contact.get("hit_distance_m", float("nan")))
            source_error = float(contact.get("source_error_m", float("nan")))
            if math.isfinite(hit_distance):
                self._contact_distance_samples_mm.append(1e3 * hit_distance)
            if math.isfinite(source_error):
                self._contact_source_error_samples_mm.append(1e3 * source_error)
        return True

    def _on_rollout_end(self) -> None:
        if self._transition_count == 0:
            return
        denominator = float(self._transition_count)
        self.last_rollout_metrics = {
            "geometry/acceptance_rate": self._accepted_count / denominator,
        }
        for reason, count in sorted(self._rejection_counts.items()):
            self.last_rollout_metrics[f"geometry/rejection_rate_{reason}"] = (
                count / denominator
            )
        for name, value in sorted(self._reward_sums.items()):
            self.last_rollout_metrics[f"reward_components/mean_{name}"] = (
                value / denominator
            )
        for name, samples in (
            ("height_mm", self._height_samples_mm),
            ("width_mm", self._width_samples_mm),
            ("contact_distance_mm", self._contact_distance_samples_mm),
            ("contact_source_error_mm", self._contact_source_error_samples_mm),
        ):
            if samples:
                self.last_rollout_metrics[f"actions/min_{name}"] = min(samples)
                self.last_rollout_metrics[f"actions/mean_{name}"] = sum(samples) / len(
                    samples
                )
                self.last_rollout_metrics[f"actions/max_{name}"] = max(samples)
        for name, value in self.last_rollout_metrics.items():
            self.logger.record(name, value)


def validate_ppo_batching(
    *,
    total_timesteps: int,
    n_steps: int,
    batch_size: int,
) -> None:
    """Reject ambiguous rollout sizes before an expensive geometry run."""

    for value, name in (
        (total_timesteps, "total_timesteps"),
        (n_steps, "n_steps"),
        (batch_size, "batch_size"),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
    if n_steps <= 1:
        raise ValueError("n_steps must be > 1 for advantage normalization")
    if total_timesteps < n_steps or total_timesteps % n_steps != 0:
        raise ValueError("total_timesteps must be a positive multiple of n_steps")
    if n_steps % batch_size != 0:
        raise ValueError("batch_size must divide n_steps for the single training env")


def build_ppo_model(
    env: gym.Env,
    *,
    learning_rate: float = 3e-4,
    n_steps: int = 128,
    batch_size: int = 64,
    n_epochs: int = 10,
    gamma: float = 0.99,
    gae_lambda: float = 0.95,
    clip_range: float = 0.2,
    ent_coef: float = 0.0,
    vf_coef: float = 0.5,
    log_std_init: float = -2.0,
    action_log_std_init: tuple[float, ...] | None = None,
    sde_sample_freq: int = 1,
    seed: int = 0,
    device: str = "cpu",
    verbose: int = 1,
    policy_kwargs: dict[str, Any] | None = None,
) -> PPO:
    """Create PPO with a correctly squashed continuous policy distribution.

    SB3's default diagonal Gaussian is clipped after sampling. Here gSDE plus
    ``squash_output`` applies the tanh transform inside the distribution, so
    actions reach the geometric decoder inside ``[-1, 1]`` without external
    clipping or incorrect log probabilities.
    """

    if n_steps <= 1 or batch_size <= 0 or n_steps % batch_size != 0:
        raise ValueError("n_steps must be > 1 and divisible by batch_size")
    if n_epochs <= 0:
        raise ValueError("n_epochs must be > 0")
    if not math.isfinite(float(log_std_init)):
        raise ValueError("log_std_init must be finite")
    if isinstance(sde_sample_freq, bool) or int(sde_sample_freq) <= 0:
        raise ValueError("sde_sample_freq must be a positive integer")
    action_log_std: tuple[float, ...] | None = None
    if action_log_std_init is not None:
        action_log_std = tuple(float(value) for value in action_log_std_init)
        action_count = int(env.action_space.shape[0])
        if len(action_log_std) != action_count:
            raise ValueError(
                f"action_log_std_init must contain {action_count} values"
            )
        if not all(math.isfinite(value) for value in action_log_std):
            raise ValueError("action_log_std_init values must be finite")
    resolved_policy_kwargs: dict[str, Any] = {
        "squash_output": True,
        "net_arch": {"pi": [256, 128], "vf": [256, 128]},
        "log_std_init": float(log_std_init),
        "use_expln": True,
    }
    if policy_kwargs:
        resolved_policy_kwargs.update(policy_kwargs)
    if not bool(resolved_policy_kwargs.get("squash_output")):
        raise ValueError("direct DDS actions require squash_output=True")

    model = PPO(
        "MultiInputPolicy",
        env,
        learning_rate=float(learning_rate),
        n_steps=int(n_steps),
        batch_size=int(batch_size),
        n_epochs=int(n_epochs),
        gamma=float(gamma),
        gae_lambda=float(gae_lambda),
        clip_range=float(clip_range),
        ent_coef=float(ent_coef),
        vf_coef=float(vf_coef),
        use_sde=True,
        sde_sample_freq=int(sde_sample_freq),
        policy_kwargs=resolved_policy_kwargs,
        seed=int(seed),
        device=device,
        verbose=int(verbose),
    )
    if action_log_std is not None:
        log_std = model.policy.log_std
        values = log_std.new_tensor(action_log_std)
        with torch.no_grad():
            if log_std.ndim == 1 and log_std.shape == values.shape:
                log_std.copy_(values)
            elif log_std.ndim == 2 and log_std.shape[1] == values.shape[0]:
                log_std.copy_(values.unsqueeze(0).expand_as(log_std))
            else:
                raise RuntimeError(
                    "unexpected gSDE log_std shape for per-action initialization: "
                    f"{tuple(log_std.shape)}"
                )
    if not model.policy.squash_output:
        raise RuntimeError("PPO policy did not enable squashed actions")
    return model
