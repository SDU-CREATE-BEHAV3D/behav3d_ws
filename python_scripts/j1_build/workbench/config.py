"""Startup defaults for simulation panels and display. Restart after editing."""
from dataclasses import asdict, dataclass, field
import math


@dataclass(frozen=True)
class Parameters:
    mode: str = "gradient_lift"
    source_spacing_mm: float = 3.0
    count: int = 7
    separation_mm: float = 38.0
    width_mm: float = 42.0
    height_mm: float = 20.0
    walk_distance_mm: float = 12.0
    walk_step_mm: float = 1.0
    walk_max_steps: int = 32
    walk_start_fraction: float = 0.25
    tangent_sign: float = 1.0
    clamp_to_cone: bool = False
    cone_deg: float = 45.0
    normal_continuity: bool = True
    clearance_mm: float = 0.0
    deposit_mode: str = "line"
    line_fraction: float = 0.22

    def validate(self):
        if self.mode not in {"z_lift", "gradient_lift", "gradient_walk"}:
            raise ValueError("Unsupported candidate mode")
        for name, value in asdict(self).items():
            if isinstance(value, (float, int)) and not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        for name in ("source_spacing_mm", "width_mm", "height_mm", "walk_step_mm"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")
        if self.count < 1 or self.walk_max_steps < 1 or self.separation_mm < 0 or self.walk_distance_mm < 0:
            raise ValueError("Invalid count, walk length or spacing")
        if not 0 <= self.line_fraction <= 1 or not 0 <= self.walk_start_fraction <= 1:
            raise ValueError("Fractions must be between 0 and 1")
        if not 0 <= self.cone_deg <= 90 or self.deposit_mode not in {"line", "dot"}:
            raise ValueError("Invalid cone or deposit mode")
        if self.tangent_sign not in {-1.0, 1.0}:
            raise ValueError("Tangent sign must be -1 or +1")


@dataclass(frozen=True)
class SimulationOptions:
    voxel_mm: float = 3.5
    threshold: float = 0.5
    surface_step: int = 1
    padding_mm: float = 24.0
    checkpoint_every: int = 10
    max_voxels: int = 30_000_000

    def validate(self):
        if not math.isfinite(self.voxel_mm) or self.voxel_mm <= 0:
            raise ValueError("Voxel size must be finite and > 0")
        if not math.isfinite(self.threshold) or not 0 < self.threshold < 1:
            raise ValueError("Threshold must be between 0 and 1")
        if self.surface_step < 1 or self.checkpoint_every < 0 or not math.isfinite(self.padding_mm) or self.padding_mm < 0:
            raise ValueError("Invalid surface/checkpoint options")


@dataclass(frozen=True)
class DisplayOptions:
    """Display-only defaults; these never change simulation geometry."""
    run_cycles: int = 10
    goal_opacity: float = 0.25
    goal_lighting: bool = False
    light_intensity: float = 1.2
    key_light_intensity: float = 0.85
    fill_light_intensity: float = 0.36
    rim_light_intensity: float = 0.18
    surface_ambient: float = 0.2
    surface_diffuse: float = 0.7
    surface_specular: float = 0.12
    surface_specular_power: float = 28.0
    phantom_color: str = "#9ec9e5"
    phantom_opacity: float = 0.80
    bead_material: str = "Warm white"
    goal_edge_opacity: float = 0.1
    bead_edge_opacity: float = 0.02
    contact_shading: bool = False
    ssao_radius_mm: float = 12.0
    ssao_bias_mm: float = 0.4
    ssao_kernel_size: int = 64
    layers: dict = field(default_factory=lambda: {
        "goal": True, "goal_edges": True, "environment": True,
        "deposits": True, "bead_edges": True, "contour": True,
        "phantom": True, "sources": True, "candidates": True,
        "paths": True, "last": True, "offset": False, "axes": False,
    })
