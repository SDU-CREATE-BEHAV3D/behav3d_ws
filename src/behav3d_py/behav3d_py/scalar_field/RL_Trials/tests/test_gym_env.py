from __future__ import annotations

import numpy as np
import pytest
from dds import Domain

try:
    import gymnasium  # noqa: F401
except ModuleNotFoundError:
    gymnasium = None

if gymnasium is not None:
    from gymnasium.utils.env_checker import check_env

    from rl_trials.gym_env import DirectBeadDepositionEnv
else:
    check_env = None
    DirectBeadDepositionEnv = None

pytestmark = pytest.mark.skipif(
    gymnasium is None,
    reason="gymnasium is not installed; use RL_Trials/.venv",
)


GOAL_VERTICES = np.array(
    [
        [-0.02, -0.02, -0.001],
        [0.02, -0.02, 0.010],
        [0.02, 0.02, 0.010],
        [-0.02, 0.02, -0.001],
    ]
)
GOAL_FACES = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
SCAN_VERTICES = np.array(
    [
        [-0.03, -0.03, 0.0],
        [0.03, -0.03, 0.0],
        [0.03, 0.03, 0.0],
        [-0.03, 0.03, 0.0],
    ]
)
SCAN_FACES = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)


def _config() -> dict[str, object]:
    return {
        "action": {
            "bead_height_min_mm": 10.0,
            "bead_height_max_mm": 10.0,
            "cone_max_tilt_deg": 30.0,
            "width_delta_min_mm": -4.0,
            "width_delta_max_mm": 4.0,
            "bead_width_min_mm": 10.0,
            "bead_width_max_mm": 14.0,
            "contact_distance_min_mm": 9.5,
            "contact_distance_max_mm": 10.5,
            "contact_distance_numeric_epsilon_mm": 0.001,
            "contact_source_error_role": "diagnostic_only",
        },
        "observation": {"contour_sample_count": 16},
        "completion": {
            "fill_tolerance_mm": 0.5,
            "success_completed_area_fraction": 0.99,
        },
        "dds": {"occupancy_threshold": 0.5},
        "reward": {
            "progress_scale": 100.0,
            "invalid_action_penalty": 1.0,
            "collision_penalty": 10.0,
            "collision_terminates_episode": False,
            "tilt_weight": 0.01,
            "cantilever_weight": 0.01,
        },
        "episode": {
            "max_deposits": 5,
            "max_attempted_steps": 8,
            "max_consecutive_invalid_actions": 2,
            "max_consecutive_no_progress": 4,
            "min_progress_area_fraction": 1e-8,
        },
        "intersection": {"iso_level_m": 0.0},
    }


def _env(*, collision_checker_factory=None) -> DirectBeadDepositionEnv:
    domain = Domain.from_bounds(
        xmin=-0.05,
        xmax=0.05,
        ymin=-0.05,
        ymax=0.05,
        zmin=-0.02,
        zmax=0.04,
        voxel_size=0.002,
        length_unit="m",
    )
    return DirectBeadDepositionEnv(
        config=_config(),
        domain=domain,
        goal_vertices=GOAL_VERTICES,
        goal_faces=GOAL_FACES,
        initial_scan_vertices=SCAN_VERTICES,
        initial_scan_faces=SCAN_FACES,
        goal_heat=np.linspace(0.0, 1.0, GOAL_VERTICES.shape[0]),
        goal_widths_m=np.full(GOAL_VERTICES.shape[0], 0.012),
        collision_checker_factory=collision_checker_factory,
    )


def test_env_passes_gymnasium_contract_check() -> None:
    env = _env()
    check_env(env, skip_render_check=True)
    env.close()


def test_reset_returns_fixed_observation_and_contour_metadata() -> None:
    env = _env()
    observation, info = env.reset(seed=7)

    assert env.observation_space.contains(observation)
    assert observation["contour"].shape == (16, 8)
    assert observation["global"].shape == (12,)
    assert np.all(observation["contour"][:, 7] == 1.0)
    assert info["contour_component_count"] >= 1
    assert env.state.attempted_steps == 0


def test_valid_action_applies_one_dds_deposit() -> None:
    env = _env()
    env.reset()

    observation, reward, terminated, truncated, info = env.step(
        np.zeros(5, dtype=np.float32)
    )

    assert env.observation_space.contains(observation)
    assert info["accepted"]
    assert info["rejection_reasons"] == ()
    assert info["reward_components"]["invalid_action_cost"] == 0.0
    assert env.state.accepted_deposits == 1
    assert env.state.geometry_revision == 1
    assert np.isfinite(reward)
    assert not (terminated and truncated)


def test_width_extreme_is_mapped_to_a_valid_physical_width() -> None:
    env = _env()
    env.reset()
    action = np.array([0.0, 0.0, 0.0, 0.0, -1.0], dtype=np.float32)

    _, reward, terminated, truncated, info = env.step(action)

    assert info["accepted"]
    assert info["rejection_reasons"] == ()
    assert info["decoded_action"]["width_m"] == pytest.approx(0.010)
    assert info["reward_components"]["invalid_action_cost"] == 0.0
    assert env.state.accepted_deposits == 1
    assert env.state.geometry_revision == 1
    assert np.isfinite(reward)
    assert not terminated
    assert not truncated


def test_consecutive_invalid_actions_truncate_episode() -> None:
    class _CollidingChecker:
        def check_pose(self, position, z_axis):
            return True, 0.0, 1

    def factory(vertices, faces):
        return _CollidingChecker()

    env = _env(collision_checker_factory=factory)
    env.reset()
    action = np.zeros(5, dtype=np.float32)

    env.step(action)
    _, _, terminated, truncated, info = env.step(action)

    assert not terminated
    assert truncated
    assert "max_consecutive_invalid_actions" in info["truncation_reasons"]
    with pytest.raises(RuntimeError, match=r"reset\(\)"):
        env.step(action)


def test_collision_gets_strong_penalty_and_keeps_dds_unchanged() -> None:
    builds = 0

    class _CollidingChecker:
        def check_pose(self, position, z_axis):
            return True, 0.0, 1

    def factory(vertices, faces):
        nonlocal builds
        builds += 1
        return _CollidingChecker()

    env = _env(collision_checker_factory=factory)
    env.reset()

    _, reward, terminated, truncated, info = env.step(np.zeros(5))

    assert builds == 1
    assert reward == pytest.approx(-11.0)
    assert info["rejection_reasons"] == ("collision",)
    assert info["reward_components"]["collision_cost"] == pytest.approx(10.0)
    assert env.state.geometry_revision == 0
    assert not terminated
    assert not truncated


def test_collision_checker_build_is_lazy_and_revision_scoped() -> None:
    builds = 0

    class _ClearChecker:
        def check_pose(self, position, z_axis):
            return False, 100.0, 0

    def factory(vertices, faces):
        nonlocal builds
        builds += 1
        return _ClearChecker()

    env = _env(collision_checker_factory=factory)
    env.reset()
    assert builds == 0
    env.step(np.zeros(5))
    assert builds == 1
    env.step(np.zeros(5))
    assert builds == 2
