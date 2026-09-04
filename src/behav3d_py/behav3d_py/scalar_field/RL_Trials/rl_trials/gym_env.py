"""Gymnasium environment for direct continuous DDS bead deposition."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import numpy.typing as npt
from dds import Domain
from gymnasium import spaces

from .action_decoder import DecodedAction, decode_continuous_action
from .action_validation import (
    ActionRejectionReason,
    ActionValidation,
    PoseCollisionChecker,
    validate_decoded_action,
)
from .contour_observation import sample_contour_observation
from .episode_state import DDSEpisodeState, EpisodeGeometrySnapshot
from .fixture_io import (
    load_configured_geometry,
    load_configured_goal_fields,
    load_experiment_config,
    resolve_workspace_path,
)
from .rewards import cantilever_ratio_cost, normalized_tilt_cost

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]
CollisionCheckerFactory = Callable[
    [FloatArray, IntArray],
    PoseCollisionChecker,
]

CONTOUR_FEATURES = (
    "source_coord",
    "position_x",
    "position_y",
    "position_z",
    "heat",
    "target_width",
    "component_index",
    "valid_mask",
)
GLOBAL_FEATURES = (
    "completed_area_fraction",
    "accepted_deposit_fraction",
    "attempted_step_fraction",
    "consecutive_invalid_fraction",
    "consecutive_no_progress_fraction",
    "has_contour",
    "component_count_fraction",
    "previous_source",
    "previous_height",
    "previous_orient_x",
    "previous_orient_y",
    "previous_width",
)


def _section(config: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = config.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"experiment config requires mapping section '{name}'")
    return value


def _points(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or result.shape[0] == 0:
        raise ValueError(f"{name} must have non-empty shape (N, 3)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return np.array(result, copy=True)


def _faces(values: npt.ArrayLike, vertex_count: int, name: str) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 3 or raw.shape[0] == 0:
        raise ValueError(f"{name} must have non-empty shape (M, 3)")
    if not np.issubdtype(raw.dtype, np.integer) and not np.all(
        np.equal(raw, np.floor(raw))
    ):
        raise ValueError(f"{name} must contain integer indices")
    result = raw.astype(np.int32, copy=True)
    if np.any(result < 0) or np.any(result >= vertex_count):
        raise ValueError(f"{name} contains an invalid vertex index")
    return result


def _per_vertex(values: npt.ArrayLike, count: int, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {count} finite values")
    return np.array(result, copy=True)


def _positive_float(value: Any, name: str, *, allow_zero: bool = False) -> float:
    result = float(value)
    valid = result >= 0.0 if allow_zero else result > 0.0
    if not np.isfinite(result) or not valid:
        comparator = ">= 0" if allow_zero else "> 0"
        raise ValueError(f"{name} must be finite and {comparator}")
    return result


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    result = int(value)
    if result <= 0:
        raise ValueError(f"{name} must be > 0")
    return result


def _domain_from_goal(
    goal_vertices: FloatArray,
    config: Mapping[str, Any],
) -> Domain:
    dds_config = _section(config, "dds")
    padding = 1e-3 * _positive_float(dds_config["padding_mm"], "dds.padding_mm")
    voxel_size = 1e-3 * _positive_float(
        dds_config["voxel_size_mm"],
        "dds.voxel_size_mm",
    )
    lower = np.min(goal_vertices, axis=0) - padding
    upper = np.max(goal_vertices, axis=0) + padding
    return Domain.from_bounds(
        xmin=float(lower[0]),
        xmax=float(upper[0]),
        ymin=float(lower[1]),
        ymax=float(upper[1]),
        zmin=float(lower[2]),
        zmax=float(upper[2]),
        voxel_size=voxel_size,
        length_unit="m",
    )


class _LazyCollisionChecker:
    """Delay the expensive distance scene until validation actually needs it."""

    def __init__(
        self,
        factory: CollisionCheckerFactory,
        vertices: FloatArray,
        faces: IntArray,
    ) -> None:
        self._factory = factory
        self._vertices = vertices
        self._faces = faces
        self._checker: PoseCollisionChecker | None = None

    def check_pose(
        self,
        position: np.ndarray,
        z_axis: np.ndarray,
    ) -> tuple[bool, float, int]:
        if self._checker is None:
            self._checker = self._factory(self._vertices, self._faces)
        return self._checker.check_pose(position, z_axis)


class DirectBeadDepositionEnv(gym.Env[dict[str, np.ndarray], np.ndarray]):
    """One direct policy action per DDS point deposit or typed rejection.

    This first Gymnasium integration intentionally uses a compact contour and
    global observation. Full per-goal-vertex occupancy and the final
    heat/overlap reward remain later modeling stages.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        config: Mapping[str, Any],
        domain: Domain,
        goal_vertices: npt.ArrayLike,
        goal_faces: npt.ArrayLike,
        initial_scan_vertices: npt.ArrayLike,
        initial_scan_faces: npt.ArrayLike,
        goal_heat: npt.ArrayLike,
        goal_widths_m: npt.ArrayLike,
        collision_checker_factory: CollisionCheckerFactory | None = None,
        render_mode: None = None,
    ) -> None:
        super().__init__()
        if render_mode is not None:
            raise ValueError("DirectBeadDepositionEnv is currently headless")
        self.render_mode = render_mode
        if not isinstance(domain, Domain):
            raise TypeError("domain must be a dds.Domain")

        self.config = dict(config)
        self.goal_vertices = _points(goal_vertices, "goal_vertices")
        self.goal_faces = _faces(
            goal_faces,
            self.goal_vertices.shape[0],
            "goal_faces",
        )
        scan_vertices = _points(initial_scan_vertices, "initial_scan_vertices")
        scan_faces = _faces(
            initial_scan_faces,
            scan_vertices.shape[0],
            "initial_scan_faces",
        )
        self.goal_heat = _per_vertex(
            goal_heat,
            self.goal_vertices.shape[0],
            "goal_heat",
        )
        if np.any(self.goal_heat < 0.0) or np.any(self.goal_heat > 1.0):
            raise ValueError("goal_heat must be normalized to [0, 1]")
        self.goal_widths_m = _per_vertex(
            goal_widths_m,
            self.goal_vertices.shape[0],
            "goal_widths_m",
        )
        if np.any(self.goal_widths_m <= 0.0):
            raise ValueError("goal_widths_m must be positive")

        action_config = _section(config, "action")
        observation_config = _section(config, "observation")
        completion_config = _section(config, "completion")
        dds_config = _section(config, "dds")
        reward_config = _section(config, "reward")
        episode_config = _section(config, "episode")
        intersection_config = _section(config, "intersection")

        self.height_min_m = 1e-3 * _positive_float(
            action_config["bead_height_min_mm"],
            "action.bead_height_min_mm",
        )
        self.height_max_m = 1e-3 * _positive_float(
            action_config["bead_height_max_mm"],
            "action.bead_height_max_mm",
        )
        if self.height_max_m < self.height_min_m:
            raise ValueError("bead height maximum must be >= minimum")
        self.cone_max_tilt_deg = _positive_float(
            action_config["cone_max_tilt_deg"],
            "action.cone_max_tilt_deg",
        )
        if self.cone_max_tilt_deg > 90.0:
            raise ValueError("action.cone_max_tilt_deg must be <= 90")
        self.width_delta_min_m = 1e-3 * float(action_config["width_delta_min_mm"])
        self.width_delta_max_m = 1e-3 * float(action_config["width_delta_max_mm"])
        if (
            not np.isfinite(self.width_delta_min_m)
            or not np.isfinite(self.width_delta_max_m)
            or self.width_delta_max_m < self.width_delta_min_m
        ):
            raise ValueError("invalid action width-delta bounds")
        self.width_min_m = 1e-3 * _positive_float(
            action_config["bead_width_min_mm"],
            "action.bead_width_min_mm",
        )
        self.width_max_m = 1e-3 * _positive_float(
            action_config["bead_width_max_mm"],
            "action.bead_width_max_mm",
        )
        if self.width_max_m < self.width_min_m:
            raise ValueError("bead width maximum must be >= minimum")
        self.contact_distance_min_m = 1e-3 * _positive_float(
            action_config["contact_distance_min_mm"],
            "action.contact_distance_min_mm",
            allow_zero=True,
        )
        self.contact_distance_max_m = 1e-3 * _positive_float(
            action_config["contact_distance_max_mm"],
            "action.contact_distance_max_mm",
        )
        if self.contact_distance_max_m < self.contact_distance_min_m:
            raise ValueError("contact distance maximum must be >= minimum")
        self.contact_distance_epsilon_m = 1e-3 * _positive_float(
            action_config["contact_distance_numeric_epsilon_mm"],
            "action.contact_distance_numeric_epsilon_mm",
            allow_zero=True,
        )
        self.contour_sample_count = _positive_int(
            observation_config["contour_sample_count"],
            "observation.contour_sample_count",
        )
        self.fill_tolerance_m = 1e-3 * _positive_float(
            completion_config["fill_tolerance_mm"],
            "completion.fill_tolerance_mm",
            allow_zero=True,
        )
        self.success_fraction = float(
            completion_config["success_completed_area_fraction"]
        )
        if not 0.0 <= self.success_fraction <= 1.0:
            raise ValueError("success_completed_area_fraction must lie in [0, 1]")
        self.iso_level_m = float(intersection_config["iso_level_m"])
        if not np.isfinite(self.iso_level_m):
            raise ValueError("intersection.iso_level_m must be finite")

        self.max_deposits = _positive_int(
            episode_config["max_deposits"],
            "episode.max_deposits",
        )
        self.max_attempted_steps = _positive_int(
            episode_config.get("max_attempted_steps", 2 * self.max_deposits),
            "episode.max_attempted_steps",
        )
        self.max_consecutive_invalid = _positive_int(
            episode_config["max_consecutive_invalid_actions"],
            "episode.max_consecutive_invalid_actions",
        )
        self.max_consecutive_no_progress = _positive_int(
            episode_config["max_consecutive_no_progress"],
            "episode.max_consecutive_no_progress",
        )
        self.min_progress_fraction = _positive_float(
            episode_config.get("min_progress_area_fraction", 1e-8),
            "episode.min_progress_area_fraction",
            allow_zero=True,
        )

        self.progress_scale = _positive_float(
            reward_config.get("progress_scale", 100.0),
            "reward.progress_scale",
            allow_zero=True,
        )
        self.invalid_action_penalty = _positive_float(
            reward_config.get("invalid_action_penalty", 1.0),
            "reward.invalid_action_penalty",
            allow_zero=True,
        )
        self.collision_penalty = _positive_float(
            reward_config["collision_penalty"],
            "reward.collision_penalty",
            allow_zero=True,
        )
        self.tilt_weight = _positive_float(
            reward_config.get("tilt_weight", 0.01),
            "reward.tilt_weight",
            allow_zero=True,
        )
        self.cantilever_weight = _positive_float(
            reward_config.get("cantilever_weight", 0.01),
            "reward.cantilever_weight",
            allow_zero=True,
        )
        collision_terminates = reward_config.get(
            "collision_terminates_episode",
            False,
        )
        if not isinstance(collision_terminates, bool):
            raise TypeError("reward.collision_terminates_episode must be boolean")
        self.collision_terminates_episode = collision_terminates

        self._goal_min = np.min(self.goal_vertices, axis=0)
        self._goal_max = np.max(self.goal_vertices, axis=0)
        self._goal_center = 0.5 * (self._goal_min + self._goal_max)
        self._goal_half_extent = 0.5 * (self._goal_max - self._goal_min)
        self._goal_half_extent[self._goal_half_extent <= 1e-12] = 1.0

        self._state = DDSEpisodeState(
            domain=domain,
            initial_scan_vertices=scan_vertices,
            initial_scan_faces=scan_faces,
            occupancy_threshold=float(dds_config["occupancy_threshold"]),
        )
        self._collision_checker_factory = collision_checker_factory
        self._collision_revision = -1
        self._collision_checker: _LazyCollisionChecker | None = None
        self._snapshot: EpisodeGeometrySnapshot | None = None
        self._contour_heat: FloatArray | None = None
        self._contour_widths_m: FloatArray | None = None
        self._previous_action = np.zeros(5, dtype=np.float64)
        self._consecutive_invalid = 0
        self._consecutive_no_progress = 0
        self._needs_reset = True

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(5,),
            dtype=np.float32,
        )
        self.observation_space = spaces.Dict(
            {
                "contour": spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(self.contour_sample_count, len(CONTOUR_FEATURES)),
                    dtype=np.float32,
                ),
                "global": spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(len(GLOBAL_FEATURES),),
                    dtype=np.float32,
                ),
            }
        )

    @classmethod
    def from_config(
        cls,
        config_path: Path,
        *,
        workspace_root: Path,
        enable_collision: bool = True,
    ) -> DirectBeadDepositionEnv:
        """Load the selected goal/scan/fields and construct the headless env."""

        from lib_scalar.bead_profile import normalized_width_to_mm

        config = load_experiment_config(Path(config_path))
        workspace = Path(workspace_root).expanduser().resolve()
        goal_vertices, goal_faces, scan_vertices, scan_faces = (
            load_configured_geometry(config, workspace)
        )
        heat, width_norm = load_configured_goal_fields(
            config,
            workspace,
            expected_count=goal_vertices.shape[0],
        )
        scalar_config = _section(config, "scalar_fields")
        widths_m = 1e-3 * normalized_width_to_mm(
            width_norm,
            width_min_mm=float(scalar_config["width_min_mm"]),
            width_max_mm=float(scalar_config["width_max_mm"]),
        )
        domain = _domain_from_goal(goal_vertices, config)

        collision_factory: CollisionCheckerFactory | None = None
        if enable_collision:
            from lib_scalar.extruder_collision import (
                ExtruderCollisionChecker,
                load_collada_mesh,
            )

            paths = _section(config, "paths")
            extruder_path = resolve_workspace_path(
                workspace,
                str(paths["extruder_mesh"]),
            )
            extruder_vertices, _ = load_collada_mesh(extruder_path)
            collision_config = _section(config, "collision")
            threshold_mm = float(collision_config["threshold_mm"])
            exclusion_mm = float(collision_config["tcp_exclusion_radius_mm"])

            def collision_factory(
                material_vertices: FloatArray,
                material_faces: IntArray,
            ) -> PoseCollisionChecker:
                return ExtruderCollisionChecker(
                    scan_vertices=material_vertices,
                    scan_faces=material_faces,
                    extruder_vertices_tool0=extruder_vertices,
                    threshold_mm=threshold_mm,
                    tcp_exclusion_radius_mm=exclusion_mm,
                )

        return cls(
            config=config,
            domain=domain,
            goal_vertices=goal_vertices,
            goal_faces=goal_faces,
            initial_scan_vertices=scan_vertices,
            initial_scan_faces=scan_faces,
            goal_heat=heat,
            goal_widths_m=widths_m,
            collision_checker_factory=collision_factory,
        )

    @property
    def state(self) -> DDSEpisodeState:
        """Expose the headless DDS state for diagnostics and visualization."""

        return self._state

    def _refresh_snapshot(self) -> None:
        from lib_scalar.geometry import sample_vertex_scalar_on_surface

        self._snapshot = self._state.geometry_snapshot(
            goal_vertices=self.goal_vertices,
            goal_faces=self.goal_faces,
            fill_tolerance_m=self.fill_tolerance_m,
            iso_level_m=self.iso_level_m,
        )
        contour = self._snapshot.contour
        if contour is None:
            self._contour_heat = None
            self._contour_widths_m = None
            return
        self._contour_heat = sample_vertex_scalar_on_surface(
            contour.points,
            self.goal_vertices,
            self.goal_faces,
            self.goal_heat,
        )
        self._contour_widths_m = sample_vertex_scalar_on_surface(
            contour.points,
            self.goal_vertices,
            self.goal_faces,
            self.goal_widths_m,
        )

    def _current_collision_checker(self) -> PoseCollisionChecker | None:
        if self._collision_checker_factory is None:
            return None
        revision = self._state.geometry_revision
        if self._collision_checker is None or self._collision_revision != revision:
            vertices, faces = self._state.current_material_mesh()
            self._collision_checker = _LazyCollisionChecker(
                self._collision_checker_factory,
                vertices,
                faces,
            )
            self._collision_revision = revision
        return self._collision_checker

    def _decode(self, action: np.ndarray) -> DecodedAction:
        if self._snapshot is None or self._snapshot.contour is None:
            raise RuntimeError("cannot decode an action without a current contour")
        if self._contour_heat is None or self._contour_widths_m is None:
            raise RuntimeError("current contour fields have not been sampled")
        return decode_continuous_action(
            normalized_action=action,
            contour=self._snapshot.contour,
            contour_heat=self._contour_heat,
            contour_widths_m=self._contour_widths_m,
            height_min_m=self.height_min_m,
            height_max_m=self.height_max_m,
            cone_max_tilt_deg=self.cone_max_tilt_deg,
            width_delta_min_m=self.width_delta_min_m,
            width_delta_max_m=self.width_delta_max_m,
            width_min_m=self.width_min_m,
            width_max_m=self.width_max_m,
        )

    def _validate(self, action: DecodedAction) -> ActionValidation:
        if self._snapshot is None:
            raise RuntimeError("environment has no current geometry snapshot")
        return validate_decoded_action(
            action=action,
            scan_vertices=self._snapshot.material_vertices,
            scan_faces=self._snapshot.material_faces,
            contact_distance_min_m=self.contact_distance_min_m,
            contact_distance_max_m=self.contact_distance_max_m,
            contact_distance_epsilon_m=self.contact_distance_epsilon_m,
            domain=self._state.domain,
            collision_checker=self._current_collision_checker(),
            raycaster=self._state.current_material_raycaster(),
        )

    def _observation(self) -> dict[str, np.ndarray]:
        if self._snapshot is None:
            raise RuntimeError("environment has no current geometry snapshot")
        contour_features = np.zeros(
            (self.contour_sample_count, len(CONTOUR_FEATURES)),
            dtype=np.float32,
        )
        contour = self._snapshot.contour
        if contour is not None:
            if self._contour_heat is None or self._contour_widths_m is None:
                raise RuntimeError("current contour fields have not been sampled")
            sampled = sample_contour_observation(
                contour=contour,
                contour_heat=self._contour_heat,
                contour_widths_m=self._contour_widths_m,
                sample_count=self.contour_sample_count,
            )
            normalized_positions = (
                sampled.samples.points - self._goal_center
            ) / self._goal_half_extent
            heat = 2.0 * np.clip(sampled.heat, 0.0, 1.0) - 1.0
            if self.width_max_m > self.width_min_m:
                width = 2.0 * (
                    (sampled.widths_m - self.width_min_m)
                    / (self.width_max_m - self.width_min_m)
                ) - 1.0
            else:
                width = np.zeros(self.contour_sample_count, dtype=np.float64)
            if contour.component_count > 1:
                component = 2.0 * (
                    sampled.samples.component_indices / (contour.component_count - 1)
                ) - 1.0
            else:
                component = np.zeros(self.contour_sample_count, dtype=np.float64)
            contour_features[:, 0] = sampled.samples.source_coord
            contour_features[:, 1:4] = np.clip(normalized_positions, -1.0, 1.0)
            contour_features[:, 4] = heat
            contour_features[:, 5] = np.clip(width, -1.0, 1.0)
            contour_features[:, 6] = component
            contour_features[:, 7] = sampled.samples.valid_mask.astype(np.float32)

        metrics = self._snapshot.evaluation.metrics
        component_count = 0 if contour is None else contour.component_count
        global_features = np.concatenate(
            (
                np.asarray(
                    [
                        metrics.completed_area_fraction,
                        self._state.accepted_deposits / self.max_deposits,
                        self._state.attempted_steps / self.max_attempted_steps,
                        self._consecutive_invalid / self.max_consecutive_invalid,
                        self._consecutive_no_progress
                        / self.max_consecutive_no_progress,
                        float(contour is not None),
                        component_count / self.contour_sample_count,
                    ],
                    dtype=np.float64,
                ),
                self._previous_action,
            )
        )
        observation = {
            "contour": contour_features,
            "global": np.clip(global_features, -1.0, 1.0).astype(np.float32),
        }
        if not self.observation_space.contains(observation):
            raise RuntimeError("generated observation lies outside observation_space")
        return observation

    def _base_info(self) -> dict[str, Any]:
        if self._snapshot is None:
            raise RuntimeError("environment has no current geometry snapshot")
        contour = self._snapshot.contour
        return {
            "geometry_revision": self._state.geometry_revision,
            "attempted_steps": self._state.attempted_steps,
            "accepted_deposits": self._state.accepted_deposits,
            "rejected_steps": self._state.rejected_steps,
            "completed_area_fraction": (
                self._snapshot.evaluation.metrics.completed_area_fraction
            ),
            "contour_component_count": 0 if contour is None else contour.component_count,
            "contour_total_length_m": 0.0 if contour is None else contour.total_length_m,
        }

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        super().reset(seed=seed)
        if options is not None and not isinstance(options, dict):
            raise TypeError("reset options must be a dictionary or None")
        self._state.reset()
        self._collision_revision = -1
        self._collision_checker = None
        self._previous_action = np.zeros(5, dtype=np.float64)
        self._consecutive_invalid = 0
        self._consecutive_no_progress = 0
        self._refresh_snapshot()
        self._needs_reset = False
        info = self._base_info()
        info["contour_features"] = CONTOUR_FEATURES
        info["global_features"] = GLOBAL_FEATURES
        return self._observation(), info

    def step(
        self,
        action: npt.ArrayLike,
    ) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        if self._needs_reset:
            raise RuntimeError("reset() must be called before step()")
        if self._snapshot is None:
            raise RuntimeError("environment has no current geometry snapshot")
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError("Gym action must contain five finite values")
        if np.any(values < -1.0) or np.any(values > 1.0):
            raise ValueError("Gym action values must lie in [-1, 1]")

        before_fraction = self._snapshot.evaluation.metrics.completed_area_fraction
        if before_fraction >= self.success_fraction:
            self._needs_reset = True
            info = self._base_info()
            info["termination_reason"] = "success"
            return self._observation(), 0.0, True, False, info
        if self._snapshot.contour is None:
            self._needs_reset = True
            info = self._base_info()
            info["termination_reason"] = "no_contour"
            return self._observation(), 0.0, True, False, info

        decoded = self._decode(values)
        validation = self._validate(decoded)
        transition = self._state.apply_validated_action(decoded, validation)
        if transition.accepted:
            self._refresh_snapshot()
        if self._snapshot is None:
            raise RuntimeError("environment lost its geometry snapshot")

        after_fraction = self._snapshot.evaluation.metrics.completed_area_fraction
        progress = after_fraction - before_fraction
        if transition.accepted:
            self._consecutive_invalid = 0
        else:
            self._consecutive_invalid += 1
        if transition.accepted and progress > self.min_progress_fraction:
            self._consecutive_no_progress = 0
        else:
            self._consecutive_no_progress += 1
        self._previous_action = values.copy()

        tilt_cost = normalized_tilt_cost(
            decoded.z_axis,
            max_tilt_deg=self.cone_max_tilt_deg,
        )
        cantilever_cost = cantilever_ratio_cost(
            decoded.source_point,
            decoded.target_point,
            bead_width_m=max(decoded.width_m, np.finfo(np.float64).eps),
        )
        invalid_cost = self.invalid_action_penalty if not transition.accepted else 0.0
        collision = ActionRejectionReason.COLLISION in transition.rejection_reasons
        collision_cost = self.collision_penalty if collision else 0.0
        tilt_penalty = self.tilt_weight * tilt_cost if transition.accepted else 0.0
        cantilever_penalty = (
            self.cantilever_weight * cantilever_cost if transition.accepted else 0.0
        )
        progress_reward = self.progress_scale * progress
        reward = (
            progress_reward
            - invalid_cost
            - collision_cost
            - tilt_penalty
            - cantilever_penalty
        )

        success = after_fraction >= self.success_fraction
        no_contour = self._snapshot.contour is None
        terminated = bool(
            success
            or no_contour
            or (collision and self.collision_terminates_episode)
        )
        truncation_reasons: list[str] = []
        if self._state.attempted_steps >= self.max_attempted_steps:
            truncation_reasons.append("max_attempted_steps")
        if self._state.accepted_deposits >= self.max_deposits:
            truncation_reasons.append("max_deposits")
        if self._consecutive_invalid >= self.max_consecutive_invalid:
            truncation_reasons.append("max_consecutive_invalid_actions")
        if self._consecutive_no_progress >= self.max_consecutive_no_progress:
            truncation_reasons.append("max_consecutive_no_progress")
        truncated = bool(truncation_reasons) and not terminated

        info = self._base_info()
        info.update(
            {
                "accepted": transition.accepted,
                "rejection_reasons": tuple(
                    reason.value for reason in transition.rejection_reasons
                ),
                "decoded_action": {
                    "source_point": decoded.source_point.copy(),
                    "target_point": decoded.target_point.copy(),
                    "z_axis": decoded.z_axis.copy(),
                    "height_m": decoded.height_m,
                    "width_m": decoded.width_m,
                    "heat": decoded.heat,
                    "component_index": decoded.source_component_index,
                },
                "contact": {
                    "hit_distance_m": validation.contact_distance_m,
                    "source_error_m": validation.contact_source_error_m,
                    "valid": validation.contact_valid,
                },
                "reward_components": {
                    "progress_area_fraction": progress,
                    "progress_reward": progress_reward,
                    "invalid_action_cost": invalid_cost,
                    "collision_cost": collision_cost,
                    "normalized_tilt_cost": tilt_cost,
                    "tilt_penalty": tilt_penalty,
                    "cantilever_ratio_cost": cantilever_cost,
                    "cantilever_penalty": cantilever_penalty,
                    "total": reward,
                },
            }
        )
        if success:
            info["termination_reason"] = "success"
        elif no_contour:
            info["termination_reason"] = "no_contour"
        elif collision and self.collision_terminates_episode:
            info["termination_reason"] = "collision"
        if truncation_reasons:
            info["truncation_reasons"] = tuple(truncation_reasons)

        observation = self._observation()
        self._needs_reset = terminated or truncated
        return observation, float(reward), terminated, truncated, info

    def render(self) -> None:
        """The first training environment is deliberately headless."""

        return None

    def close(self) -> None:
        self._collision_checker = None
