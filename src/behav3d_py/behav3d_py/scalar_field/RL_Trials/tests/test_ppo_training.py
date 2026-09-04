from __future__ import annotations

import numpy as np
import pytest

try:
    import gymnasium as gym
    from gymnasium import spaces
    import stable_baselines3  # noqa: F401
except ModuleNotFoundError:
    gym = None
    spaces = None


pytestmark = pytest.mark.skipif(
    gym is None or spaces is None,
    reason="Gymnasium and Stable-Baselines3 are optional training dependencies",
)


if gym is not None and spaces is not None:
    from stable_baselines3.common.callbacks import CallbackList

    from rl_trials.ppo_training import (
        DDSMetricsCallback,
        build_ppo_model,
        validate_ppo_batching,
    )

    class TinyDirectEnv(gym.Env):
        metadata = {"render_modes": []}

        def __init__(self) -> None:
            self.action_space = spaces.Box(-1.0, 1.0, shape=(5,), dtype=np.float32)
            self.observation_space = spaces.Dict(
                {
                    "contour": spaces.Box(
                        -1.0,
                        1.0,
                        shape=(4, 8),
                        dtype=np.float32,
                    ),
                    "global": spaces.Box(
                        -1.0,
                        1.0,
                        shape=(12,),
                        dtype=np.float32,
                    ),
                }
            )
            self.steps = 0

        def _observation(self) -> dict[str, np.ndarray]:
            return {
                "contour": np.zeros((4, 8), dtype=np.float32),
                "global": np.zeros(12, dtype=np.float32),
            }

        def reset(self, *, seed=None, options=None):
            super().reset(seed=seed)
            self.steps = 0
            return self._observation(), {}

        def step(self, action):
            assert self.action_space.contains(np.asarray(action, dtype=np.float32))
            self.steps += 1
            reward = float(1.0 - np.square(action).mean())
            info = {
                "accepted": True,
                "rejection_reasons": (),
                "reward_components": {"total": reward},
                "decoded_action": {"height_m": 0.013, "width_m": 0.024},
                "contact": {
                    "hit_distance_m": 0.012,
                    "source_error_m": 0.004,
                    "valid": True,
                },
            }
            return self._observation(), reward, False, self.steps >= 4, info


def test_validate_ppo_batching_rejects_partial_rollouts() -> None:
    with pytest.raises(ValueError, match="multiple of n_steps"):
        validate_ppo_batching(total_timesteps=10, n_steps=8, batch_size=4)
    with pytest.raises(ValueError, match="divide n_steps"):
        validate_ppo_batching(total_timesteps=16, n_steps=8, batch_size=3)


def test_squashed_ppo_predicts_bounded_actions_and_learns() -> None:
    env = TinyDirectEnv()
    model = build_ppo_model(
        env,
        n_steps=8,
        batch_size=4,
        n_epochs=1,
        action_log_std_init=(-1.5, -2.5, -2.0, -2.0, -2.5),
        seed=7,
        verbose=0,
        policy_kwargs={"net_arch": {"pi": [16], "vf": [16]}},
    )
    np.testing.assert_allclose(
        model.policy.log_std.detach().mean(dim=0).numpy(),
        [-1.5, -2.5, -2.0, -2.0, -2.5],
    )
    assert model.sde_sample_freq == 1
    observation, _ = env.reset(seed=7)
    for _ in range(8):
        action, _ = model.predict(observation, deterministic=False)
        assert env.action_space.contains(action)
    metrics = DDSMetricsCallback()
    model.learn(total_timesteps=8, callback=CallbackList([metrics]))
    assert model.num_timesteps == 8
    assert metrics.last_rollout_metrics["geometry/acceptance_rate"] == 1.0
    assert metrics.last_rollout_metrics["reward_components/mean_total"] == 1.0
    assert metrics.last_rollout_metrics["actions/min_height_mm"] == 13.0
    assert metrics.last_rollout_metrics["actions/max_height_mm"] == 13.0
    assert metrics.last_rollout_metrics["actions/mean_contact_distance_mm"] == 12.0
    assert metrics.last_rollout_metrics["actions/mean_contact_source_error_mm"] == 4.0
