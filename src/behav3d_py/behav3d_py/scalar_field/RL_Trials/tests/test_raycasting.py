from __future__ import annotations

from pathlib import Path

import numpy as np

from rl_trials.fixture_io import load_experiment_config
from rl_trials.raycasting import MeshRaycaster


def test_mesh_raycaster_returns_physical_distance_for_non_unit_directions() -> None:
    vertices = np.array(
        [
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
        ]
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    raycaster = MeshRaycaster(vertices, faces)

    distances = raycaster.cast_distances(
        origins=np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]),
        directions=np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -2.0]]),
    )

    np.testing.assert_allclose(distances, [1.0, 1.0])


def test_default_config_uses_positive_penalty_magnitudes() -> None:
    config_path = Path(__file__).resolve().parents[1] / "configs/default_experiment.yaml"
    config = load_experiment_config(config_path)

    assert float(config["reward"]["collision_penalty"]) > 0.0
