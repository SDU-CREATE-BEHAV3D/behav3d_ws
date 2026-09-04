"""Mutable headless DDS episode state for direct policy actions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt
from dds import Domain, Simulator

from .action_decoder import DecodedAction
from .action_validation import ActionRejectionReason, ActionValidation
from .contour_parameterization import ContourParameterization
from .dds_adapter import point_deposit_from_action
from .goal_evaluator import GoalEvaluation, evaluate_goal_with_vertical_rays
from .raycasting import MeshRaycaster

if TYPE_CHECKING:
    from dds import Deposit, SimulationResult

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]


def _readonly(values: npt.ArrayLike, dtype: npt.DTypeLike) -> np.ndarray:
    result = np.array(values, dtype=dtype, copy=True)
    result.setflags(write=False)
    return result


def _points(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or result.shape[0] == 0:
        raise ValueError(f"{name} must have non-empty shape (N, 3)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _faces(values: npt.ArrayLike, vertex_count: int, name: str) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 3 or raw.shape[0] == 0:
        raise ValueError(f"{name} must have non-empty shape (M, 3)")
    if not np.issubdtype(raw.dtype, np.integer) and not np.all(
        np.equal(raw, np.floor(raw))
    ):
        raise ValueError(f"{name} must contain integer indices")
    result = raw.astype(np.int32, copy=False)
    if np.any(result < 0) or np.any(result >= vertex_count):
        raise ValueError(f"{name} contains an invalid vertex index")
    return result


@dataclass(frozen=True)
class EpisodeTransition:
    """Bookkeeping result of accepting or rejecting one attempted action."""

    attempted_step: int
    accepted: bool
    deposit_index: int | None
    rejection_reasons: tuple[ActionRejectionReason, ...]
    geometry_revision: int


@dataclass(frozen=True)
class EpisodeGeometrySnapshot:
    """Current raycast geometry, goal evaluation, and derived contour."""

    geometry_revision: int
    material_vertices: FloatArray
    material_faces: IntArray
    evaluation: GoalEvaluation
    contour_points: FloatArray
    contour_lines: IntArray
    contour: ContourParameterization | None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "material_vertices",
            _readonly(self.material_vertices, np.float64),
        )
        object.__setattr__(
            self,
            "material_faces",
            _readonly(self.material_faces, np.int32),
        )
        object.__setattr__(
            self,
            "contour_points",
            _readonly(self.contour_points, np.float64),
        )
        object.__setattr__(
            self,
            "contour_lines",
            _readonly(self.contour_lines, np.int32),
        )


class DDSEpisodeState:
    """Own the mutable DDS deposit field and immutable initial scan.

    The DDS field contains only policy-applied deposits. For raycasting and
    goal evaluation, :meth:`current_material_mesh` combines the immutable
    initial scan with the current DDS iso-surface as one triangle soup. This is
    sufficient for current vertical ray queries but is not a boolean mesh union.
    """

    def __init__(
        self,
        *,
        domain: Domain,
        initial_scan_vertices: npt.ArrayLike,
        initial_scan_faces: npt.ArrayLike,
        occupancy_threshold: float = 0.5,
        surface_step_size: int = 1,
    ) -> None:
        if not isinstance(domain, Domain):
            raise TypeError("domain must be a dds.Domain")
        vertices = _points(initial_scan_vertices, "initial_scan_vertices")
        faces = _faces(initial_scan_faces, vertices.shape[0], "initial_scan_faces")
        threshold = float(occupancy_threshold)
        if not np.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
            raise ValueError("occupancy_threshold must lie in [0, 1]")
        if isinstance(surface_step_size, bool) or not isinstance(
            surface_step_size, int
        ):
            raise TypeError("surface_step_size must be an integer")
        if surface_step_size < 1:
            raise ValueError("surface_step_size must be >= 1")

        self.domain = domain
        self.initial_scan_vertices = _readonly(vertices, np.float64)
        self.initial_scan_faces = _readonly(faces, np.int32)
        self.occupancy_threshold = threshold
        self.surface_step_size = surface_step_size
        self._simulator = Simulator(domain)
        self._attempted_steps = 0
        self._accepted_deposits = 0
        self._rejected_steps = 0
        self._geometry_revision = 0
        self._last_rejection_reasons: tuple[ActionRejectionReason, ...] = ()
        # Warm both arrays once so later add/reset operations are incremental.
        self._result_cache: SimulationResult | None = self._simulator.result(
            include_coverage=True,
            threshold=self.occupancy_threshold,
        )
        self._material_mesh_cache: tuple[FloatArray, IntArray] | None = (
            self.initial_scan_vertices,
            self.initial_scan_faces,
        )
        self._raycaster_cache: MeshRaycaster | None = None

    @property
    def attempted_steps(self) -> int:
        return self._attempted_steps

    @property
    def accepted_deposits(self) -> int:
        return self._accepted_deposits

    @property
    def rejected_steps(self) -> int:
        return self._rejected_steps

    @property
    def geometry_revision(self) -> int:
        return self._geometry_revision

    @property
    def last_rejection_reasons(self) -> tuple[ActionRejectionReason, ...]:
        return self._last_rejection_reasons

    @property
    def deposits(self) -> tuple[Deposit, ...]:
        return self._simulator.deposits

    def result(self) -> SimulationResult:
        """Return an immutable DDS snapshot including additive coverage."""

        if self._result_cache is None:
            self._result_cache = self._simulator.result(
                include_coverage=True,
                threshold=self.occupancy_threshold,
            )
        return self._result_cache

    def reset(self) -> None:
        """Restore the initial physical state while retaining allocated arrays."""

        self._simulator.clear_deposits()
        self._attempted_steps = 0
        self._accepted_deposits = 0
        self._rejected_steps = 0
        self._geometry_revision = 0
        self._last_rejection_reasons = ()
        self._result_cache = None
        self._material_mesh_cache = (
            self.initial_scan_vertices,
            self.initial_scan_faces,
        )
        self._raycaster_cache = None

    def apply_validated_action(
        self,
        action: DecodedAction,
        validation: ActionValidation,
    ) -> EpisodeTransition:
        """Apply one accepted action or record a mutation-free rejection."""

        if not isinstance(action, DecodedAction):
            raise TypeError("action must be a DecodedAction")
        if not isinstance(validation, ActionValidation):
            raise TypeError("validation must be an ActionValidation")

        attempted_step = self._attempted_steps
        self._attempted_steps += 1
        if not validation.valid:
            self._rejected_steps += 1
            self._last_rejection_reasons = validation.rejection_reasons
            return EpisodeTransition(
                attempted_step=attempted_step,
                accepted=False,
                deposit_index=None,
                rejection_reasons=validation.rejection_reasons,
                geometry_revision=self._geometry_revision,
            )

        deposit = point_deposit_from_action(action)
        deposit_index = self._accepted_deposits
        self._simulator.add_deposit(deposit)
        self._accepted_deposits += 1
        self._geometry_revision += 1
        self._last_rejection_reasons = ()
        self._result_cache = None
        self._material_mesh_cache = None
        self._raycaster_cache = None
        return EpisodeTransition(
            attempted_step=attempted_step,
            accepted=True,
            deposit_index=deposit_index,
            rejection_reasons=(),
            geometry_revision=self._geometry_revision,
        )

    def current_material_mesh(self) -> tuple[FloatArray, IntArray]:
        """Return the initial scan plus the accumulated DDS surface mesh."""

        if self._material_mesh_cache is not None:
            return self._material_mesh_cache
        dds_mesh = self.result().analysis.surface_mesh(
            threshold=self.occupancy_threshold,
            step_size=self.surface_step_size,
        )
        if dds_mesh.is_empty:
            self._material_mesh_cache = (
                self.initial_scan_vertices,
                self.initial_scan_faces,
            )
            return self._material_mesh_cache
        deposited_vertices = np.asarray(dds_mesh.vertices, dtype=np.float64)
        deposited_faces = np.asarray(dds_mesh.faces, dtype=np.int32)
        vertices = np.vstack((self.initial_scan_vertices, deposited_vertices))
        faces = np.vstack(
            (
                self.initial_scan_faces,
                deposited_faces + self.initial_scan_vertices.shape[0],
            )
        )
        self._material_mesh_cache = (
            _readonly(vertices, np.float64),
            _readonly(faces, np.int32),
        )
        return self._material_mesh_cache

    def current_material_raycaster(self) -> MeshRaycaster:
        """Return one reusable raycasting scene for the geometry revision."""

        if self._raycaster_cache is None:
            vertices, faces = self.current_material_mesh()
            self._raycaster_cache = MeshRaycaster(vertices, faces)
        return self._raycaster_cache

    def geometry_snapshot(
        self,
        *,
        goal_vertices: npt.ArrayLike,
        goal_faces: npt.ArrayLike,
        fill_tolerance_m: float,
        clearance_m: float = 0.0,
        iso_level_m: float = 0.0,
    ) -> EpisodeGeometrySnapshot:
        """Re-evaluate the goal and contour against current physical geometry."""

        from lib_scalar.extract_phi_contour import extract_phi_contour

        material_vertices, material_faces = self.current_material_mesh()
        raycaster = self.current_material_raycaster()
        evaluation = evaluate_goal_with_vertical_rays(
            goal_vertices,
            goal_faces,
            material_vertices,
            material_faces,
            clearance_m=clearance_m,
            fill_tolerance_m=fill_tolerance_m,
            raycaster=raycaster,
        )
        contour_points, contour_lines = extract_phi_contour(
            vertices=goal_vertices,
            faces=goal_faces,
            scalar=evaluation.phi,
            iso=float(iso_level_m),
        )
        contour = (
            None
            if contour_lines.shape[0] == 0
            else ContourParameterization.from_segments(contour_points, contour_lines)
        )
        return EpisodeGeometrySnapshot(
            geometry_revision=self._geometry_revision,
            material_vertices=material_vertices,
            material_faces=material_faces,
            evaluation=evaluation,
            contour_points=contour_points,
            contour_lines=contour_lines,
            contour=contour,
        )
