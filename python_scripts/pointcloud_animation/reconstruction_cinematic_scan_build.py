#!/usr/bin/env python3
"""Animate multiple reconstructed scan folders as a cumulative build sequence.

This script reuses the same core loading and camera/frustum projection concept
as `reconstruction_cinematic_stage1.py`, but changes the animation unit from
capture groups within one scan to whole `print_scan_*` folders across a session.

Each selected scan contributes its reconstructed point cloud as one animated
projection event:
- choose one representative camera pose for that scan
- start that scan's points on the representative camera frustum plane
- animate the points into their final world-space positions
- keep settled points visible so the geometry accumulates scan by scan

Coordinate-system assumptions:
- Each `.ply` is already in the world/base frame.
- `T_base_tool0` from each manifest is converted to camera poses using the same
  explicit extrinsic convention as the existing Stage 1 script.
- The representative camera is only a cinematic source point for the reveal. It
  is not meant to claim that the full scan physically came from one exposure.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from reconstruction_cinematic_stage1 import (
    CapturePose,
    IntrinsicsData,
    SequenceData,
    Stage1Error,
    ValidationError,
    apply_frustum_plane_noise,
    blend_rgb,
    capture_export_frame,
    compute_default_side_offset_direction,
    compute_preview_frustum_intensity,
    compute_projected_start_positions,
    compute_scene_extent,
    configure_logging,
    create_camera_frustum,
    create_marker_sphere,
    create_visual_point_cloud,
    discover_sequence_directories,
    easing_value,
    encode_exported_frames_to_video,
    ensure_directory_exists,
    ensure_file_exists,
    load_extrinsic_transform,
    load_intrinsics,
    load_sequence,
    parse_color,
    prepare_export_frames_directory,
    print_dataset_summary,
    print_sequence_summary,
    require_numpy,
    require_open3d,
    resolve_optional_path_arg,
    sample_point_launch_delays,
    sequence_color,
    set_line_set_uniform_color,
    set_mesh_uniform_color,
    set_preview_filming_camera,
    summarize_intrinsics,
    interpolate_scalar,
    direction_to_yaw_elevation,
    yaw_elevation_to_direction,
)


@dataclass(frozen=True)
class ScriptInputs:
    """Editable defaults for running the script directly from VS Code."""

    dataset_root: str = "~/Downloads/260227_160709"
    extrinsics_path: str = "~/Downloads/260227_160709/config/extrinsics.yaml"
    intrinsics_path: str | None = "~/Downloads/260227_160709/config/ir_intrinsics.yaml"
    extrinsic_convention: str = "T_tool0_camera"
    extrinsic_key: str | None = "frames.T_tool0_ir"
    expected_captures: int = 12
    skip_invalid_sequences: bool = True
    scan_index_start: int | None = None
    scan_index_end: int | None = None
    animation_start_scan_index: int | None = None
    animation_start_folder: str | None = "print_scan_077"
    point_size: float = 4.0
    use_point_cloud_colors: bool = True
    representative_capture_mode: str = "centroid_nearest"
    representative_capture_index: int | None = None
    show_representative_camera_positions: bool = True
    show_representative_camera_frustums: bool = True
    camera_marker_scale: float = 0.0075
    camera_frustum_scale: float = 0.18
    camera_frustum_plane_scale: float = 0.7
    camera_frustum_color: str = "0.68,0.88,1.0"
    animate_sequence_build: bool = True
    sequence_speed_multiplier: float = 1.0
    sequence_overlap_fraction: float = 0.5
    sequence_projection_start_scale: float = 1.0
    sequence_animation_frames: int = 50
    sequence_hold_frames: int = 60
    sequence_fps: float = 30.0
    sequence_easing: str = "ease_out_cubic"
    sequence_motion_style: str = "tornado"
    sequence_point_limit: int | None = None
    sequence_delay_fraction: float = 0.5
    sequence_start_noise_scale: float = 0.009
    sequence_frustum_fade_frames: int = 6
    sequence_random_seed: int = 7
    sequence_tornado_turns: float = 3.0
    sequence_tornado_radius_scale: float = 0.09
    sequence_tornado_radius_decay_power: float = 1.25
    sequence_tornado_wander_fraction: float = 0.72
    sequence_tornado_wander_scale: float = 1.9
    sequence_source_focus_scale: float = 0.45
    sequence_start_opacity: float = 0.5
    film_camera_mode: str = "top_to_side_view"
    film_target_mode: str = "final_centroid"
    film_motion_easing: str = "linear"
    film_top_elevation_start_deg: float = 88.0
    film_top_elevation_end_deg: float = 12.0
    film_top_yaw_start_deg: float = 0.0
    film_top_yaw_end_deg: float = 0.0
    film_yaw_start_deg: float = -65.0
    film_yaw_end_deg: float = 35.0
    film_vertical_offset_deg: float = 50.0
    film_frame_vertical_offset_scale: float = 0.0
    film_zoom: float = 0.3
    film_yaw_start_deg_final: float = -65.0
    film_yaw_end_deg_final: float = 55.0
    film_vertical_offset_deg_final: float = 4.0
    film_frame_vertical_offset_scale_final: float = 0.0
    film_zoom_final: float = 0.4
    export_enabled: bool = False
    export_frames_dir: str = "~/Downloads/260227_160709/exports/scan_build_frames"
    export_video_path: str | None = "~/Downloads/260227_160709/exports/scan_build_preview.mp4"
    export_fps: float | None = None
    export_keep_frames: bool = False
    background_color: str = "0.02,0.02,0.03"
    window_width: int = 2400
    window_height: int = 1600
    no_visualization: bool = False
    verbose: bool = False


SCRIPT_INPUTS = ScriptInputs(
    # Input data and calibration
    dataset_root="~/Downloads/260227_160709",  # Session root containing multiple print_scan_* folders.
    extrinsics_path="~/Downloads/260227_160709/config/extrinsics.yaml",  # Tool-to-camera extrinsics file.
    intrinsics_path="~/Downloads/260227_160709/config/ir_intrinsics.yaml",  # Camera intrinsics file used for frustums/projected starts.
    extrinsic_convention="T_tool0_camera",  # Whether the extrinsics file stores T_tool0_camera or T_camera_tool0.
    extrinsic_key="frames.T_tool0_ir",  # Dotted key used when the extrinsics file contains multiple transforms.
    expected_captures=12,  # Expected number of manifest captures per valid scan.
    skip_invalid_sequences=True,  # Skip scans with bad manifests/calibration/empty geometry instead of stopping the whole session load.
    scan_index_start=None,  # Optional first numeric scan index to include.
    scan_index_end=None,  # Optional last numeric scan index to include.
    animation_start_scan_index=None,  # Optional numeric scan index where animation starts; earlier loaded scans start already settled.
    animation_start_folder="print_scan_035",  # Folder name or numeric suffix for the first scan that should animate.

    # Point cloud look
    point_size=3.5,  # Render size of the accumulated point clouds in Open3D.
    use_point_cloud_colors=True,  # Use RGB stored in each .ply when available.
    sequence_point_limit=None,  # Optional per-scan point cap for lighter previews/exports; None keeps all points.

    # Representative camera per scan
    representative_capture_mode="centroid_nearest",  # How each scan chooses its one cinematic source camera.
    representative_capture_index=None,  # Optional explicit capture index override for every scan.
    show_representative_camera_positions=True,  # Show a sphere at the active representative camera.
    show_representative_camera_frustums=True,  # Show a frustum for the active representative camera.
    camera_marker_scale=0.006,  # Sphere marker size at the representative camera position.
    camera_frustum_scale=0.14,  # Representative frustum depth as a fraction of scene extent.
    camera_frustum_plane_scale=0.7,  # Representative frustum front rectangle width/height multiplier.
    camera_frustum_color="0.68,0.88,1.0",  # Color of representative camera spheres and frustums.

    # Sequence build timing
    animate_sequence_build=True,  # Run the scan-by-scan build animation instead of only showing the final static view.
    sequence_speed_multiplier=1.3,  # Main speed knob: larger values make the sequence build play faster.
    sequence_overlap_fraction=0.8,  # When the next scan starts, as a fraction of the previous scan's duration; 0.5 starts the next scan midway.
    sequence_projection_start_scale=1.0,  # 1.0 starts exactly on the frustum plane; smaller values start closer to the camera.
    sequence_animation_frames=40,  # Frame count for one scan cloud to travel from frustum to final points.
    sequence_hold_frames=20,  # Extra settled frames shown after all scans are in place.
    sequence_fps=30.0,  # Base playback FPS before the speed multiplier is applied.
    sequence_easing="ease_out_cubic",  # Motion easing curve for each scan reveal.
    sequence_motion_style="tornado",  # Motion style for scan points: straight-line or source-to-final randomized waypoint interpolation.
    sequence_delay_fraction=0.5,  # Random per-point launch delay fraction within each scan reveal.
    sequence_start_noise_scale=0.009,  # Frustum-plane jitter to avoid a rigid sheet of points.
    sequence_frustum_fade_frames=6,  # How many frames the active representative frustum takes to fade after its scan settles.
    sequence_random_seed=7,  # Seed so subsampling, noise, and delays stay repeatable.
    sequence_tornado_turns=6.5,  # Number of swirl turns each point makes before settling.
    sequence_tornado_radius_scale=0.09,  # Base tornado radius as a fraction of scene extent.
    sequence_tornado_radius_decay_power=4.25,  # How quickly the vortex radius collapses as points settle.
    sequence_tornado_wander_fraction=0.72,  # Fraction of each reveal spent in looser free-flight before the points home in.
    sequence_tornado_wander_scale=1.9,  # Strength of the off-path wander while points are still flying around.
    sequence_source_focus_scale=0.45,  # Smaller values keep launch points and early waypoints tighter around the source frustum.
    sequence_start_opacity=0.5,  # Simulated starting opacity for points by blending their colors toward the background.

    # Filming camera
    film_camera_mode="top_to_side_view",  # Virtual filming camera mode.
    film_target_mode="final_centroid",  # What the filming camera looks at during the build.
    film_motion_easing="linear",  # Easing used for the filming camera motion; linear starts moving immediately.
    film_top_elevation_start_deg=60.0,  # Starting absolute camera elevation for top-view mode.
    film_top_elevation_end_deg=12.0,  # Final absolute camera elevation for top-to-side mode.
    film_top_yaw_start_deg=0.0,  # Starting yaw offset relative to the scene's base side direction for top-view mode.
    film_top_yaw_end_deg=55.0,  # Final yaw offset relative to the scene's base side direction; keep equal for a pure top-to-side move.
    film_yaw_start_deg=-65.0,  # Initial starting yaw offset in degrees relative to the default side view.
    film_yaw_end_deg=35.0,  # Initial ending yaw offset in degrees relative to the default side view.
    film_vertical_offset_deg=50.0,  # Initial vertical filming-camera offset in degrees relative to the default side view.
    film_frame_vertical_offset_scale=0.0,  # Initial vertical framing offset; keep near zero for a clean top-view start.
    film_zoom=0.4,  # Initial Open3D zoom for the filming camera.
    film_yaw_start_deg_final=-65.0,  # Final starting yaw offset in degrees relative to the default side view.
    film_yaw_end_deg_final=55.0,  # Final ending yaw offset in degrees relative to the default side view.
    film_vertical_offset_deg_final=-14.0,  # Final vertical filming-camera offset in degrees relative to the default side view.
    film_frame_vertical_offset_scale_final=0.0,  # Final vertical framing offset for the filming camera.
    film_zoom_final=0.4,  # Final Open3D zoom for the filming camera.

    # Export
    export_enabled=True,  # Export the scan-build animation to frames/video instead of only previewing interactively.
    export_frames_dir="~/Downloads/260227_160709/exports/scan_build_frames",  # Folder where numbered PNG frames are written during export.
    export_video_path="~/Downloads/260227_160709/exports/scan_build_preview.mp4",  # Output .mp4 path for the encoded scan-build video.
    export_fps=None,  # Output video FPS. None uses sequence_fps * sequence_speed_multiplier.
    export_keep_frames=False,  # Keep the PNG frame sequence after video encoding finishes.

    # Viewer / utility
    background_color="0.02,0.02,0.03",  # Viewer background color as r,g,b in 0-1 or 0-255.
    window_width=3600,  # Viewer/export width in pixels.
    window_height=2800,  # Viewer/export height in pixels.
    no_visualization=False,  # Validate and summarize only; do not open the viewer.
    verbose=False,  # Print extra debug logging while loading and animating.
)


LOGGER = logging.getLogger("reconstruction_cinematic_scan_build")


@dataclass(frozen=True)
class SequenceProjectionData:
    sequence: SequenceData
    order_index: int
    representative_capture_index: int
    representative_capture: CapturePose
    final_points: Any
    start_points: Any
    colors: Any
    launch_delays: Any
    swirl_basis_u: Any
    swirl_basis_v: Any
    swirl_phase_offsets: Any
    swirl_radius_scales: Any
    path_directions: Any
    swirl_turn_multipliers: Any
    swirl_direction_signs: Any
    swirl_secondary_phase_offsets: Any
    swirl_secondary_radius_scales: Any
    swirl_turbulence_frequencies: Any
    swirl_turbulence_phases: Any
    swirl_axial_scales: Any
    waypoint_1: Any
    waypoint_2: Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Animate multiple print_scan_* folders as a cumulative scan-by-scan build.",
    )
    parser.add_argument("--dataset-root", default=SCRIPT_INPUTS.dataset_root)
    parser.add_argument("--extrinsics-path", default=SCRIPT_INPUTS.extrinsics_path)
    parser.add_argument("--intrinsics-path", default=SCRIPT_INPUTS.intrinsics_path)
    parser.add_argument(
        "--extrinsic-convention",
        choices=("T_tool0_camera", "T_camera_tool0"),
        default=SCRIPT_INPUTS.extrinsic_convention,
    )
    parser.add_argument("--extrinsic-key", type=str, default=SCRIPT_INPUTS.extrinsic_key)
    parser.add_argument("--expected-captures", type=int, default=SCRIPT_INPUTS.expected_captures)
    parser.add_argument(
        "--skip-invalid-sequences",
        action="store_true",
        default=SCRIPT_INPUTS.skip_invalid_sequences,
    )
    parser.add_argument(
        "--fail-on-invalid-sequence",
        action="store_false",
        dest="skip_invalid_sequences",
    )
    parser.add_argument("--scan-index-start", type=int, default=SCRIPT_INPUTS.scan_index_start)
    parser.add_argument("--scan-index-end", type=int, default=SCRIPT_INPUTS.scan_index_end)
    parser.add_argument(
        "--animation-start-scan-index",
        type=int,
        default=SCRIPT_INPUTS.animation_start_scan_index,
    )
    parser.add_argument(
        "--animation-start-folder",
        type=str,
        default=SCRIPT_INPUTS.animation_start_folder,
    )
    parser.add_argument("--point-size", type=float, default=SCRIPT_INPUTS.point_size)
    parser.add_argument(
        "--use-point-cloud-colors",
        action="store_true",
        default=SCRIPT_INPUTS.use_point_cloud_colors,
    )
    parser.add_argument(
        "--hide-point-cloud-colors",
        action="store_false",
        dest="use_point_cloud_colors",
    )
    parser.add_argument(
        "--representative-capture-mode",
        choices=("centroid_nearest", "first_capture", "middle_capture", "last_capture"),
        default=SCRIPT_INPUTS.representative_capture_mode,
    )
    parser.add_argument(
        "--representative-capture-index",
        type=int,
        default=SCRIPT_INPUTS.representative_capture_index,
    )
    parser.add_argument(
        "--show-representative-camera-positions",
        action="store_true",
        default=SCRIPT_INPUTS.show_representative_camera_positions,
    )
    parser.add_argument(
        "--hide-representative-camera-positions",
        action="store_false",
        dest="show_representative_camera_positions",
    )
    parser.add_argument(
        "--show-representative-camera-frustums",
        action="store_true",
        default=SCRIPT_INPUTS.show_representative_camera_frustums,
    )
    parser.add_argument(
        "--hide-representative-camera-frustums",
        action="store_false",
        dest="show_representative_camera_frustums",
    )
    parser.add_argument("--camera-marker-scale", type=float, default=SCRIPT_INPUTS.camera_marker_scale)
    parser.add_argument("--camera-frustum-scale", type=float, default=SCRIPT_INPUTS.camera_frustum_scale)
    parser.add_argument(
        "--camera-frustum-plane-scale",
        type=float,
        default=SCRIPT_INPUTS.camera_frustum_plane_scale,
    )
    parser.add_argument("--camera-frustum-color", type=str, default=SCRIPT_INPUTS.camera_frustum_color)
    parser.add_argument(
        "--animate-sequence-build",
        action="store_true",
        default=SCRIPT_INPUTS.animate_sequence_build,
    )
    parser.add_argument(
        "--static-view",
        action="store_false",
        dest="animate_sequence_build",
    )
    parser.add_argument(
        "--sequence-speed-multiplier",
        type=float,
        default=SCRIPT_INPUTS.sequence_speed_multiplier,
    )
    parser.add_argument(
        "--sequence-overlap-fraction",
        type=float,
        default=SCRIPT_INPUTS.sequence_overlap_fraction,
    )
    parser.add_argument(
        "--sequence-projection-start-scale",
        type=float,
        default=SCRIPT_INPUTS.sequence_projection_start_scale,
    )
    parser.add_argument(
        "--sequence-animation-frames",
        type=int,
        default=SCRIPT_INPUTS.sequence_animation_frames,
    )
    parser.add_argument(
        "--sequence-hold-frames",
        type=int,
        default=SCRIPT_INPUTS.sequence_hold_frames,
    )
    parser.add_argument("--sequence-fps", type=float, default=SCRIPT_INPUTS.sequence_fps)
    parser.add_argument(
        "--sequence-easing",
        choices=("linear", "ease_out_cubic", "ease_in_out_cubic"),
        default=SCRIPT_INPUTS.sequence_easing,
    )
    parser.add_argument(
        "--sequence-motion-style",
        choices=("linear", "tornado"),
        default=SCRIPT_INPUTS.sequence_motion_style,
    )
    parser.add_argument("--sequence-point-limit", type=int, default=SCRIPT_INPUTS.sequence_point_limit)
    parser.add_argument(
        "--sequence-delay-fraction",
        type=float,
        default=SCRIPT_INPUTS.sequence_delay_fraction,
    )
    parser.add_argument(
        "--sequence-start-noise-scale",
        type=float,
        default=SCRIPT_INPUTS.sequence_start_noise_scale,
    )
    parser.add_argument(
        "--sequence-frustum-fade-frames",
        type=int,
        default=SCRIPT_INPUTS.sequence_frustum_fade_frames,
    )
    parser.add_argument("--sequence-random-seed", type=int, default=SCRIPT_INPUTS.sequence_random_seed)
    parser.add_argument(
        "--sequence-tornado-turns",
        type=float,
        default=SCRIPT_INPUTS.sequence_tornado_turns,
    )
    parser.add_argument(
        "--sequence-tornado-radius-scale",
        type=float,
        default=SCRIPT_INPUTS.sequence_tornado_radius_scale,
    )
    parser.add_argument(
        "--sequence-tornado-radius-decay-power",
        type=float,
        default=SCRIPT_INPUTS.sequence_tornado_radius_decay_power,
    )
    parser.add_argument(
        "--sequence-tornado-wander-fraction",
        type=float,
        default=SCRIPT_INPUTS.sequence_tornado_wander_fraction,
    )
    parser.add_argument(
        "--sequence-tornado-wander-scale",
        type=float,
        default=SCRIPT_INPUTS.sequence_tornado_wander_scale,
    )
    parser.add_argument(
        "--sequence-source-focus-scale",
        type=float,
        default=SCRIPT_INPUTS.sequence_source_focus_scale,
    )
    parser.add_argument(
        "--sequence-start-opacity",
        type=float,
        default=SCRIPT_INPUTS.sequence_start_opacity,
    )
    parser.add_argument(
        "--film-camera-mode",
        choices=("static_side", "orbit_side", "top_to_oblique_orbit", "top_to_side_view"),
        default=SCRIPT_INPUTS.film_camera_mode,
    )
    parser.add_argument(
        "--film-target-mode",
        choices=("final_centroid", "visible_centroid"),
        default=SCRIPT_INPUTS.film_target_mode,
    )
    parser.add_argument(
        "--film-motion-easing",
        choices=("linear", "ease_out_cubic", "ease_in_out_cubic"),
        default=SCRIPT_INPUTS.film_motion_easing,
    )
    parser.add_argument(
        "--film-top-elevation-start-deg",
        type=float,
        default=SCRIPT_INPUTS.film_top_elevation_start_deg,
    )
    parser.add_argument(
        "--film-top-elevation-end-deg",
        type=float,
        default=SCRIPT_INPUTS.film_top_elevation_end_deg,
    )
    parser.add_argument(
        "--film-top-yaw-start-deg",
        type=float,
        default=SCRIPT_INPUTS.film_top_yaw_start_deg,
    )
    parser.add_argument(
        "--film-top-yaw-end-deg",
        type=float,
        default=SCRIPT_INPUTS.film_top_yaw_end_deg,
    )
    parser.add_argument("--film-yaw-start-deg", type=float, default=SCRIPT_INPUTS.film_yaw_start_deg)
    parser.add_argument("--film-yaw-end-deg", type=float, default=SCRIPT_INPUTS.film_yaw_end_deg)
    parser.add_argument(
        "--film-vertical-offset-deg",
        type=float,
        default=SCRIPT_INPUTS.film_vertical_offset_deg,
    )
    parser.add_argument(
        "--film-frame-vertical-offset-scale",
        type=float,
        default=SCRIPT_INPUTS.film_frame_vertical_offset_scale,
    )
    parser.add_argument("--film-zoom", type=float, default=SCRIPT_INPUTS.film_zoom)
    parser.add_argument(
        "--film-yaw-start-deg-final",
        type=float,
        default=SCRIPT_INPUTS.film_yaw_start_deg_final,
    )
    parser.add_argument(
        "--film-yaw-end-deg-final",
        type=float,
        default=SCRIPT_INPUTS.film_yaw_end_deg_final,
    )
    parser.add_argument(
        "--film-vertical-offset-deg-final",
        type=float,
        default=SCRIPT_INPUTS.film_vertical_offset_deg_final,
    )
    parser.add_argument(
        "--film-frame-vertical-offset-scale-final",
        type=float,
        default=SCRIPT_INPUTS.film_frame_vertical_offset_scale_final,
    )
    parser.add_argument("--film-zoom-final", type=float, default=SCRIPT_INPUTS.film_zoom_final)
    parser.add_argument("--export-enabled", action="store_true", default=SCRIPT_INPUTS.export_enabled)
    parser.add_argument("--preview-only", action="store_false", dest="export_enabled")
    parser.add_argument("--export-frames-dir", type=str, default=SCRIPT_INPUTS.export_frames_dir)
    parser.add_argument("--export-video-path", type=str, default=SCRIPT_INPUTS.export_video_path)
    parser.add_argument("--export-fps", type=float, default=SCRIPT_INPUTS.export_fps)
    parser.add_argument(
        "--export-keep-frames",
        action="store_true",
        default=SCRIPT_INPUTS.export_keep_frames,
    )
    parser.add_argument(
        "--discard-export-frames",
        action="store_false",
        dest="export_keep_frames",
    )
    parser.add_argument("--background-color", type=str, default=SCRIPT_INPUTS.background_color)
    parser.add_argument("--window-width", type=int, default=SCRIPT_INPUTS.window_width)
    parser.add_argument("--window-height", type=int, default=SCRIPT_INPUTS.window_height)
    parser.add_argument("--no-visualization", action="store_true", default=SCRIPT_INPUTS.no_visualization)
    parser.add_argument("--verbose", action="store_true", default=SCRIPT_INPUTS.verbose)
    return parser.parse_args()


def filter_scan_directories(
    discovered: Sequence[tuple[int, Path]],
    scan_index_start: int | None,
    scan_index_end: int | None,
) -> list[tuple[int, Path]]:
    filtered: list[tuple[int, Path]] = []
    for scan_index, sequence_path in discovered:
        if scan_index_start is not None and scan_index < scan_index_start:
            continue
        if scan_index_end is not None and scan_index > scan_index_end:
            continue
        filtered.append((scan_index, sequence_path))
    return filtered


def resolve_animation_start_scan_index(
    sequences: Sequence[SequenceData],
    animation_start_scan_index: int | None,
    animation_start_folder: str | None,
) -> int | None:
    if animation_start_folder is None or not str(animation_start_folder).strip():
        return animation_start_scan_index

    folder_name = Path(str(animation_start_folder).strip()).name
    if folder_name.startswith("print_scan_"):
        suffix = folder_name[len("print_scan_") :]
    else:
        suffix = folder_name

    if not suffix.isdigit():
        raise ValidationError(
            f"Animation start folder '{animation_start_folder}' is invalid. "
            "Expected a folder name like 'print_scan_035' or a numeric suffix like '35'."
        )

    resolved_index = int(suffix)
    available_scan_indices = [sequence.scan_index for sequence in sequences]
    if resolved_index not in available_scan_indices:
        raise ValidationError(
            f"Animation start folder '{animation_start_folder}' resolved to scan {resolved_index}, "
            f"but that scan was not loaded. Available scan indices: {available_scan_indices}"
        )

    return resolved_index


def split_sequences_for_initial_state(
    sequences: Sequence[SequenceData],
    animation_start_scan_index: int | None,
) -> tuple[list[SequenceData], list[SequenceData]]:
    if not sequences:
        raise ValidationError("At least one loaded scan is required for sequence-build animation.")

    if animation_start_scan_index is None:
        return [], list(sequences)

    prebuilt_sequences = [
        sequence for sequence in sequences if sequence.scan_index < animation_start_scan_index
    ]
    animated_sequences = [
        sequence for sequence in sequences if sequence.scan_index >= animation_start_scan_index
    ]
    if not any(sequence.scan_index == animation_start_scan_index for sequence in sequences):
        available_scan_indices = [sequence.scan_index for sequence in sequences]
        raise ValidationError(
            f"Animation start scan index {animation_start_scan_index} was not loaded. "
            f"Available scan indices: {available_scan_indices}"
        )
    if not animated_sequences:
        raise ValidationError(
            f"No scans remain to animate from start scan index {animation_start_scan_index}."
        )

    return prebuilt_sequences, animated_sequences


def select_representative_capture(
    sequence: SequenceData,
    representative_capture_mode: str,
    representative_capture_index: int | None,
) -> tuple[int, CapturePose]:
    np = require_numpy()

    if representative_capture_index is not None:
        if representative_capture_index < 0 or representative_capture_index >= len(sequence.captures):
            raise ValidationError(
                f"Representative capture index {representative_capture_index} is out of range "
                f"for {sequence.sequence_path.name}; valid range is "
                f"[0, {len(sequence.captures) - 1}]."
            )
        return representative_capture_index, sequence.captures[representative_capture_index]

    if representative_capture_mode == "first_capture":
        capture_index = 0
    elif representative_capture_mode == "middle_capture":
        capture_index = len(sequence.captures) // 2
    elif representative_capture_mode == "last_capture":
        capture_index = len(sequence.captures) - 1
    elif representative_capture_mode == "centroid_nearest":
        offsets = np.asarray(sequence.camera_positions, dtype=float) - np.asarray(
            sequence.sequence_centroid,
            dtype=float,
        )
        distances_sq = np.sum(offsets * offsets, axis=1)
        capture_index = int(np.argmin(distances_sq))
    else:
        raise ValidationError(
            f"Unsupported representative capture mode: {representative_capture_mode}."
        )

    return capture_index, sequence.captures[capture_index]


def load_sequences_with_optional_skips(
    dataset_root: Path,
    expected_captures: int,
    T_extrinsic: Any,
    convention: str,
    scan_index_start: int | None,
    scan_index_end: int | None,
    skip_invalid_sequences: bool,
) -> list[SequenceData]:
    discovered = discover_sequence_directories(dataset_root)
    filtered = filter_scan_directories(
        discovered=discovered,
        scan_index_start=scan_index_start,
        scan_index_end=scan_index_end,
    )
    if not filtered:
        raise ValidationError(
            f"No scan folders remain after applying scan range "
            f"start={scan_index_start}, end={scan_index_end} in {dataset_root}."
        )

    sequences: list[SequenceData] = []
    for scan_index, sequence_path in filtered:
        try:
            sequence = load_sequence(
                scan_index=scan_index,
                sequence_path=sequence_path,
                expected_captures=expected_captures,
                T_extrinsic=T_extrinsic,
                convention=convention,
            )
            if sequence.point_count <= 0:
                raise ValidationError(
                    f"Skipping {sequence.sequence_path.name} because its reconstructed point cloud is empty."
                )
            print_sequence_summary(sequence)
            sequences.append(sequence)
        except Stage1Error as exc:
            if not skip_invalid_sequences:
                raise
            LOGGER.warning("Skipping %s | %s", sequence_path.name, exc)

    if not sequences:
        raise ValidationError(
            "No valid scan folders were loaded after filtering and validation."
        )

    return sequences


def sample_sequence_points(
    sequence: SequenceData,
    order_index: int,
    total_sequence_count: int,
    use_point_cloud_colors: bool,
    point_limit: int | None,
) -> tuple[Any, Any]:
    np = require_numpy()

    points = np.asarray(sequence.point_cloud.points, dtype=float)
    if use_point_cloud_colors and sequence.has_colors:
        colors = np.asarray(sequence.point_cloud.colors, dtype=float)
    else:
        fallback_color = np.asarray(sequence_color(order_index, total_sequence_count), dtype=float)
        colors = np.tile(fallback_color, (points.shape[0], 1))

    if point_limit is not None and points.shape[0] > point_limit:
        sample_indices = np.linspace(0, points.shape[0] - 1, num=point_limit, dtype=int)
        points = points[sample_indices]
        colors = colors[sample_indices]

    return points, colors


def create_sampled_visual_point_cloud(
    sequence: SequenceData,
    order_index: int,
    total_sequence_count: int,
    use_point_cloud_colors: bool,
    point_limit: int | None,
) -> Any:
    o3d = require_open3d()

    points, colors = sample_sequence_points(
        sequence=sequence,
        order_index=order_index,
        total_sequence_count=total_sequence_count,
        use_point_cloud_colors=use_point_cloud_colors,
        point_limit=point_limit,
    )
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def build_tornado_swirl_components(
    start_points: Any,
    final_points: Any,
    scene_extent: float,
    tornado_radius_scale: float,
    rng: Any,
) -> tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any, Any, Any, Any]:
    np = require_numpy()

    start_points = np.asarray(start_points, dtype=float)
    final_points = np.asarray(final_points, dtype=float)
    travel_vectors = final_points - start_points
    travel_norms = np.linalg.norm(travel_vectors, axis=1, keepdims=True)
    safe_travel_norms = np.maximum(travel_norms, 1e-9)
    path_directions = travel_vectors / safe_travel_norms

    reference_up = np.asarray([0.0, 0.0, 1.0], dtype=float)
    reference_side = np.asarray([0.0, 1.0, 0.0], dtype=float)
    basis_u = np.cross(path_directions, reference_up[None, :])
    basis_u_norms = np.linalg.norm(basis_u, axis=1, keepdims=True)
    fallback_mask = basis_u_norms[:, 0] <= 1e-6
    if np.any(fallback_mask):
        basis_u[fallback_mask] = np.cross(
            path_directions[fallback_mask],
            reference_side[None, :],
        )
        basis_u_norms = np.linalg.norm(basis_u, axis=1, keepdims=True)
    basis_u /= np.maximum(basis_u_norms, 1e-9)
    basis_v = np.cross(path_directions, basis_u)
    basis_v /= np.maximum(np.linalg.norm(basis_v, axis=1, keepdims=True), 1e-9)

    base_radius = scene_extent * tornado_radius_scale
    random_radius_factors = rng.uniform(0.55, 1.15, size=(start_points.shape[0],))
    path_radius_factors = np.clip(travel_norms[:, 0] / max(scene_extent, 1e-9), 0.35, 1.2)
    swirl_radius_scales = base_radius * random_radius_factors * path_radius_factors
    swirl_phase_offsets = rng.uniform(0.0, 2.0 * math.pi, size=(start_points.shape[0],))
    swirl_turn_multipliers = rng.uniform(0.7, 1.8, size=(start_points.shape[0],))
    swirl_direction_signs = rng.choice(
        np.asarray([-1.0, 1.0], dtype=float),
        size=(start_points.shape[0],),
        p=(0.3, 0.7),
    )
    swirl_secondary_phase_offsets = rng.uniform(0.0, 2.0 * math.pi, size=(start_points.shape[0],))
    swirl_secondary_radius_scales = (
        swirl_radius_scales * rng.uniform(0.18, 0.55, size=(start_points.shape[0],))
    )
    swirl_turbulence_frequencies = rng.uniform(1.4, 3.8, size=(start_points.shape[0],))
    swirl_turbulence_phases = rng.uniform(0.0, 2.0 * math.pi, size=(start_points.shape[0],))
    swirl_axial_scales = np.minimum(
        swirl_radius_scales * rng.uniform(0.12, 0.4, size=(start_points.shape[0],)),
        (0.12 * travel_norms[:, 0]) + (0.02 * scene_extent),
    )
    return (
        basis_u,
        basis_v,
        swirl_phase_offsets,
        swirl_radius_scales,
        path_directions,
        swirl_turn_multipliers,
        swirl_direction_signs,
        swirl_secondary_phase_offsets,
        swirl_secondary_radius_scales,
        swirl_turbulence_frequencies,
        swirl_turbulence_phases,
        swirl_axial_scales,
    )


def build_random_air_waypoints(
    start_points: Any,
    final_points: Any,
    path_directions: Any,
    basis_u: Any,
    basis_v: Any,
    swirl_radius_scales: Any,
    swirl_secondary_radius_scales: Any,
    swirl_axial_scales: Any,
    wander_scale: float,
    source_focus_scale: float,
    rng: Any,
) -> tuple[Any, Any]:
    np = require_numpy()

    start_points = np.asarray(start_points, dtype=float)
    final_points = np.asarray(final_points, dtype=float)
    travel_vectors = final_points - start_points
    point_count = start_points.shape[0]
    focus_scale = float(np.clip(source_focus_scale, 0.05, 2.0))
    early_progress_scale = 0.35 + (0.65 * focus_scale)
    early_radius_scale = 0.2 + (0.8 * focus_scale)
    late_radius_scale = 0.55 + (0.45 * focus_scale)
    axial_scale = 0.35 + (0.65 * focus_scale)

    progress_1 = rng.uniform(0.14, 0.30, size=(point_count, 1)) * early_progress_scale
    progress_2 = rng.uniform(0.56, 0.82, size=(point_count, 1)) * (0.82 + (0.18 * focus_scale))

    angle_1 = rng.uniform(0.0, 2.0 * math.pi, size=(point_count,))
    angle_2 = angle_1 + rng.uniform(0.65 * math.pi, 1.65 * math.pi, size=(point_count,))

    radius_1 = swirl_radius_scales * wander_scale * early_radius_scale * rng.uniform(
        1.05,
        1.85,
        size=(point_count,),
    )
    radius_2 = swirl_secondary_radius_scales * wander_scale * late_radius_scale * rng.uniform(
        0.95,
        1.85,
        size=(point_count,),
    )

    axial_1 = swirl_axial_scales * axial_scale * rng.uniform(-0.55, 0.55, size=(point_count,))
    axial_2 = swirl_axial_scales * axial_scale * rng.uniform(-0.45, 0.45, size=(point_count,))

    waypoint_1 = (
        start_points
        + (progress_1 * travel_vectors)
        + (np.cos(angle_1) * radius_1)[:, None] * basis_u
        + (np.sin(angle_1) * radius_1)[:, None] * basis_v
        + axial_1[:, None] * path_directions
    )
    waypoint_2 = (
        start_points
        + (progress_2 * travel_vectors)
        + (np.cos(angle_2) * radius_2)[:, None] * basis_u
        + (np.sin(angle_2) * radius_2)[:, None] * basis_v
        + axial_2[:, None] * path_directions
    )
    return waypoint_1, waypoint_2


def evaluate_catmull_rom_segment(
    P0: Any,
    P1: Any,
    P2: Any,
    P3: Any,
    u: Any,
) -> Any:
    np = require_numpy()

    u = np.asarray(u, dtype=float)[:, None]
    u2 = u * u
    u3 = u2 * u
    return 0.5 * (
        (2.0 * P1)
        + ((-P0 + P2) * u)
        + ((2.0 * P0 - (5.0 * P1) + (4.0 * P2) - P3) * u2)
        + ((-P0 + (3.0 * P1) - (3.0 * P2) + P3) * u3)
    )


def evaluate_waypoint_path(
    projection: SequenceProjectionData,
    path_t: Any,
    wander_fraction: float,
) -> Any:
    np = require_numpy()

    path_t = np.asarray(path_t, dtype=float)
    segment_1_end = min(0.18 + (0.22 * wander_fraction), 0.40)
    segment_2_end = min(0.62 + (0.18 * wander_fraction), 0.86)
    boundaries = np.asarray([0.0, segment_1_end, segment_2_end, 1.0], dtype=float)

    control_points = [
        projection.start_points,
        projection.waypoint_1,
        projection.waypoint_2,
        projection.final_points,
    ]
    segment_triplets = [
        (control_points[0], control_points[0], control_points[1], control_points[2]),
        (control_points[0], control_points[1], control_points[2], control_points[3]),
        (control_points[1], control_points[2], control_points[3], control_points[3]),
    ]

    current_points = np.empty_like(projection.start_points)
    for segment_index, (P0, P1, P2, P3) in enumerate(segment_triplets):
        start_t = boundaries[segment_index]
        end_t = boundaries[segment_index + 1]
        if segment_index == len(segment_triplets) - 1:
            segment_mask = (path_t >= start_t) & (path_t <= end_t)
        else:
            segment_mask = (path_t >= start_t) & (path_t < end_t)
        if not np.any(segment_mask):
            continue
        local_u = np.clip(
            (path_t[segment_mask] - start_t) / max(end_t - start_t, 1e-6),
            0.0,
            1.0,
        )
        current_points[segment_mask] = evaluate_catmull_rom_segment(
            P0[segment_mask],
            P1[segment_mask],
            P2[segment_mask],
            P3[segment_mask],
            local_u,
        )
    return current_points


def set_scan_build_filming_camera(
    vis: Any,
    base_offset_direction: Any,
    lookat: Any,
    scene_extent: float,
    film_camera_mode: str,
    film_top_elevation_start_deg: float,
    film_top_elevation_end_deg: float,
    film_top_yaw_start_deg: float,
    film_top_yaw_end_deg: float,
    film_yaw_start_deg: float,
    film_yaw_end_deg: float,
    film_vertical_offset_deg: float,
    film_frame_vertical_offset_scale: float,
    film_zoom: float,
    film_yaw_start_deg_final: float,
    film_yaw_end_deg_final: float,
    film_vertical_offset_deg_final: float,
    film_frame_vertical_offset_scale_final: float,
    film_zoom_final: float,
    motion_t: float,
) -> None:
    np = require_numpy()

    if film_camera_mode not in ("top_to_oblique_orbit", "top_to_side_view"):
        set_preview_filming_camera(
            vis=vis,
            base_offset_direction=base_offset_direction,
            lookat=lookat,
            scene_extent=scene_extent,
            film_camera_mode=film_camera_mode,
            film_yaw_start_deg=film_yaw_start_deg,
            film_yaw_end_deg=film_yaw_end_deg,
            film_vertical_offset_deg=film_vertical_offset_deg,
            film_frame_vertical_offset_scale=film_frame_vertical_offset_scale,
            film_zoom=film_zoom,
            film_yaw_start_deg_final=film_yaw_start_deg_final,
            film_yaw_end_deg_final=film_yaw_end_deg_final,
            film_vertical_offset_deg_final=film_vertical_offset_deg_final,
            film_frame_vertical_offset_scale_final=film_frame_vertical_offset_scale_final,
            film_zoom_final=film_zoom_final,
            motion_t=motion_t,
        )
        return

    base_yaw, _ = direction_to_yaw_elevation(base_offset_direction)
    current_elevation_deg = interpolate_scalar(
        film_top_elevation_start_deg,
        film_top_elevation_end_deg,
        motion_t,
    )
    current_yaw_deg = interpolate_scalar(
        film_top_yaw_start_deg,
        film_top_yaw_end_deg,
        motion_t,
    )
    current_frame_vertical_offset_scale = interpolate_scalar(
        film_frame_vertical_offset_scale,
        film_frame_vertical_offset_scale_final,
        motion_t,
    )
    current_zoom = interpolate_scalar(
        film_zoom,
        film_zoom_final,
        motion_t,
    )

    elevation = math.radians(max(-80.0, min(89.0, current_elevation_deg)))
    yaw = base_yaw + math.radians(current_yaw_deg)
    offset_direction = yaw_elevation_to_direction(yaw, elevation)
    front = offset_direction

    world_up = np.asarray([0.0, 0.0, 1.0], dtype=float)
    reference_up = (
        np.asarray([0.0, 1.0, 0.0], dtype=float)
        if abs(float(np.dot(front, world_up))) > 0.98
        else world_up
    )
    right = np.cross(reference_up, front)
    right_norm = np.linalg.norm(right)
    if right_norm <= 1e-9:
        right = np.asarray([1.0, 0.0, 0.0], dtype=float)
        right_norm = np.linalg.norm(right)
    right /= max(right_norm, 1e-9)
    up = np.cross(front, right)
    up_norm = np.linalg.norm(up)
    if up_norm <= 1e-9:
        up = np.asarray([0.0, 1.0, 0.0], dtype=float)
        up_norm = np.linalg.norm(up)
    up /= max(up_norm, 1e-9)

    adjusted_lookat = np.asarray(lookat, dtype=float) + np.asarray(
        [0.0, 0.0, scene_extent * current_frame_vertical_offset_scale],
        dtype=float,
    )
    view_control = vis.get_view_control()
    view_control.set_lookat(adjusted_lookat)
    view_control.set_front(front)
    view_control.set_up(up)
    view_control.set_zoom(float(current_zoom))


def prepare_sequence_projection_data(
    sequences: Sequence[SequenceData],
    intrinsics: IntrinsicsData,
    representative_capture_mode: str,
    representative_capture_index: int | None,
    use_point_cloud_colors: bool,
    point_limit: int | None,
    camera_frustum_depth: float,
    camera_frustum_plane_scale: float,
    sequence_projection_start_scale: float,
    sequence_start_noise_scale: float,
    sequence_delay_fraction: float,
    random_seed: int,
    sequence_tornado_radius_scale: float,
    sequence_tornado_wander_scale: float,
    sequence_source_focus_scale: float,
) -> list[SequenceProjectionData]:
    np = require_numpy()

    scene_extent = compute_scene_extent(sequences)
    rng = np.random.default_rng(random_seed)
    prepared: list[SequenceProjectionData] = []

    for order_index, sequence in enumerate(sequences):
        capture_index, capture = select_representative_capture(
            sequence=sequence,
            representative_capture_mode=representative_capture_mode,
            representative_capture_index=representative_capture_index,
        )
        final_points, colors = sample_sequence_points(
            sequence=sequence,
            order_index=order_index,
            total_sequence_count=len(sequences),
            use_point_cloud_colors=use_point_cloud_colors,
            point_limit=point_limit,
        )
        start_points = compute_projected_start_positions(
            points_world=final_points,
            T_base_camera=capture.T_base_camera,
            intrinsics=intrinsics,
            frustum_depth=camera_frustum_depth,
            frustum_plane_scale=camera_frustum_plane_scale,
            frustum_point_scale=sequence_projection_start_scale,
        )
        start_points = apply_frustum_plane_noise(
            start_points_world=start_points,
            T_base_camera=capture.T_base_camera,
            noise_scale_world=scene_extent * sequence_start_noise_scale * sequence_source_focus_scale,
            rng=rng,
        )
        launch_delays = sample_point_launch_delays(
            point_count=final_points.shape[0],
            delay_fraction=sequence_delay_fraction,
            rng=rng,
        )
        (
            swirl_basis_u,
            swirl_basis_v,
            swirl_phase_offsets,
            swirl_radius_scales,
            path_directions,
            swirl_turn_multipliers,
            swirl_direction_signs,
            swirl_secondary_phase_offsets,
            swirl_secondary_radius_scales,
            swirl_turbulence_frequencies,
            swirl_turbulence_phases,
            swirl_axial_scales,
        ) = (
            build_tornado_swirl_components(
                start_points=start_points,
                final_points=final_points,
                scene_extent=scene_extent,
                tornado_radius_scale=sequence_tornado_radius_scale,
                rng=rng,
            )
        )
        waypoint_1, waypoint_2 = build_random_air_waypoints(
            start_points=start_points,
            final_points=final_points,
            path_directions=path_directions,
            basis_u=swirl_basis_u,
            basis_v=swirl_basis_v,
            swirl_radius_scales=swirl_radius_scales,
            swirl_secondary_radius_scales=swirl_secondary_radius_scales,
            swirl_axial_scales=swirl_axial_scales,
            wander_scale=sequence_tornado_wander_scale,
            source_focus_scale=sequence_source_focus_scale,
            rng=rng,
        )
        prepared.append(
            SequenceProjectionData(
                sequence=sequence,
                order_index=order_index,
                representative_capture_index=capture_index,
                representative_capture=capture,
                final_points=final_points,
                start_points=start_points,
                colors=colors,
                launch_delays=launch_delays,
                swirl_basis_u=swirl_basis_u,
                swirl_basis_v=swirl_basis_v,
                swirl_phase_offsets=swirl_phase_offsets,
                swirl_radius_scales=swirl_radius_scales,
                path_directions=path_directions,
                swirl_turn_multipliers=swirl_turn_multipliers,
                swirl_direction_signs=swirl_direction_signs,
                swirl_secondary_phase_offsets=swirl_secondary_phase_offsets,
                swirl_secondary_radius_scales=swirl_secondary_radius_scales,
                swirl_turbulence_frequencies=swirl_turbulence_frequencies,
                swirl_turbulence_phases=swirl_turbulence_phases,
                swirl_axial_scales=swirl_axial_scales,
                waypoint_1=waypoint_1,
                waypoint_2=waypoint_2,
            )
        )

    return prepared


def evaluate_sequence_frame_points(
    projection: SequenceProjectionData,
    sequence_frame_index: int,
    sequence_frame_count: int,
    easing: str,
    motion_style: str,
    tornado_turns: float,
    tornado_radius_decay_power: float,
    tornado_wander_fraction: float,
    tornado_wander_scale: float,
    start_opacity: float,
    background_color: Sequence[float],
) -> tuple[Any, Any]:
    np = require_numpy()

    if sequence_frame_index < 0:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)
    if sequence_frame_index >= sequence_frame_count:
        return projection.final_points, projection.colors
    if sequence_frame_count <= 1:
        return projection.final_points, projection.colors

    t = sequence_frame_index / float(sequence_frame_count - 1)
    local_t = np.clip(
        (t - projection.launch_delays) / np.maximum(1.0 - projection.launch_delays, 1e-6),
        0.0,
        1.0,
    )
    eased_t = np.asarray([easing_value(value, easing) for value in local_t], dtype=float)
    if motion_style == "linear":
        linear_points = projection.start_points + eased_t[:, None] * (
            projection.final_points - projection.start_points
        )
        current_points = linear_points
    elif motion_style == "tornado":
        wander_fraction = float(np.clip(tornado_wander_fraction, 0.0, 0.95))
        path_points = evaluate_waypoint_path(
            projection=projection,
            path_t=eased_t,
            wander_fraction=wander_fraction,
        )
        remaining_t = np.maximum(1.0 - eased_t, 0.0)
        homing_t = np.clip(
            (eased_t - wander_fraction) / max(1.0 - wander_fraction, 1e-6),
            0.0,
            1.0,
        )
        turbulence_decay = np.power(1.0 - homing_t, max(0.85, tornado_radius_decay_power * 0.45))
        micro_radius_scale = tornado_wander_scale * (
            0.18 + (0.82 * np.power(np.maximum(1.0 - homing_t, 0.0), 1.15))
        )
        primary_turns = (
            tornado_turns
            * projection.swirl_turn_multipliers
            * projection.swirl_direction_signs
        )
        primary_angles = projection.swirl_phase_offsets + (2.0 * math.pi * primary_turns * eased_t)
        secondary_turns = tornado_turns * projection.swirl_turbulence_frequencies
        secondary_angles = projection.swirl_secondary_phase_offsets + (
            2.0 * math.pi * secondary_turns * eased_t
        )
        turbulence_angles = projection.swirl_turbulence_phases + (
            2.0
            * math.pi
            * projection.swirl_turbulence_frequencies
            * eased_t
        )
        micro_offsets = (
            (
                np.cos(primary_angles)
                * projection.swirl_secondary_radius_scales
                * 0.42
                * micro_radius_scale
                * turbulence_decay
            )[:, None]
            * projection.swirl_basis_u
            + (
                np.sin(secondary_angles)
                * projection.swirl_secondary_radius_scales
                * 0.38
                * micro_radius_scale
                * turbulence_decay
            )[:, None]
            * projection.swirl_basis_v
            + (
                np.sin((0.65 * turbulence_angles) + projection.swirl_phase_offsets)
                * projection.swirl_axial_scales
                * 0.55
                * micro_radius_scale
                * turbulence_decay
            )[:, None]
            * projection.path_directions
        )
        current_points = path_points + micro_offsets
    else:
        raise ValidationError(
            f"Unsupported sequence motion style: {motion_style}. "
            "Expected 'linear' or 'tornado'."
        )
    visible_mask = local_t > 0.0
    if not np.any(visible_mask):
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)
    visible_opacity = start_opacity + ((1.0 - start_opacity) * local_t[visible_mask])
    visible_background = np.asarray(background_color, dtype=float)[None, :]
    visible_colors = (
        (projection.colors[visible_mask] * visible_opacity[:, None])
        + (visible_background * (1.0 - visible_opacity[:, None]))
    )
    return current_points[visible_mask], visible_colors


def compute_sequence_build_target_point(
    projections: Sequence[SequenceProjectionData],
    visible_point_sets: Sequence[Any],
    target_mode: str,
) -> Any:
    np = require_numpy()

    all_final_points = np.vstack([projection.final_points for projection in projections])
    if target_mode == "final_centroid":
        return all_final_points.mean(axis=0)

    if target_mode == "visible_centroid":
        visible_non_empty = [
            np.asarray(points, dtype=float)
            for points in visible_point_sets
            if np.asarray(points).size > 0
        ]
        if visible_non_empty:
            return np.vstack(visible_non_empty).mean(axis=0)
        return all_final_points.mean(axis=0)

    raise ValidationError(f"Unsupported filming camera target mode: {target_mode}.")


def show_static_sequence_view(
    sequences: Sequence[SequenceData],
    intrinsics: IntrinsicsData | None,
    point_size: float,
    background_color: Sequence[float],
    window_width: int,
    window_height: int,
    camera_marker_scale: float,
    camera_frustum_scale: float,
    camera_frustum_plane_scale: float,
    camera_frustum_color: Sequence[float],
    use_point_cloud_colors: bool,
    representative_capture_mode: str,
    representative_capture_index: int | None,
    show_representative_camera_positions: bool,
    show_representative_camera_frustums: bool,
) -> None:
    o3d = require_open3d()
    np = require_numpy()

    scene_extent = compute_scene_extent(sequences)
    marker_radius = max(scene_extent * camera_marker_scale, scene_extent * 0.0025)
    frustum_depth = max(scene_extent * camera_frustum_scale, scene_extent * 0.02)

    geometries: list[Any] = [
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15 * scene_extent)
    ]
    for order_index, sequence in enumerate(sequences):
        geometries.append(
            create_visual_point_cloud(
                sequence=sequence,
                fallback_color=sequence_color(order_index, len(sequences)),
                use_point_cloud_colors=use_point_cloud_colors,
            )
        )
        rep_index, rep_capture = select_representative_capture(
            sequence=sequence,
            representative_capture_mode=representative_capture_mode,
            representative_capture_index=representative_capture_index,
        )
        LOGGER.info(
            "Static view representative camera for scan %03d -> capture %d",
            sequence.scan_index,
            rep_index,
        )
        if show_representative_camera_positions:
            geometries.append(
                create_marker_sphere(
                    position=rep_capture.camera_position,
                    radius=marker_radius,
                    color=camera_frustum_color,
                )
            )
        if show_representative_camera_frustums:
            if intrinsics is None:
                raise ValidationError(
                    "Representative camera frustums require intrinsics in static view."
                )
            geometries.append(
                create_camera_frustum(
                    T_base_camera=rep_capture.T_base_camera,
                    intrinsics=intrinsics,
                    depth=frustum_depth,
                    plane_scale=camera_frustum_plane_scale,
                    color=camera_frustum_color,
                )
            )

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Cinematic Scan Build - Static View",
        width=window_width,
        height=window_height,
    )
    try:
        for geometry in geometries:
            vis.add_geometry(geometry)
        render_option = vis.get_render_option()
        render_option.point_size = float(point_size)
        render_option.background_color = np.asarray(background_color, dtype=float)
        render_option.mesh_show_back_face = True
        vis.poll_events()
        vis.update_renderer()
        vis.run()
    finally:
        vis.destroy_window()


def animate_sequence_build(
    sequences: Sequence[SequenceData],
    intrinsics: IntrinsicsData | None,
    point_size: float,
    background_color: Sequence[float],
    window_width: int,
    window_height: int,
    use_point_cloud_colors: bool,
    animation_start_scan_index: int | None,
    representative_capture_mode: str,
    representative_capture_index: int | None,
    show_representative_camera_positions: bool,
    show_representative_camera_frustums: bool,
    camera_marker_scale: float,
    camera_frustum_scale: float,
    camera_frustum_plane_scale: float,
    camera_frustum_color: Sequence[float],
    sequence_speed_multiplier: float,
    sequence_overlap_fraction: float,
    sequence_projection_start_scale: float,
    sequence_animation_frames: int,
    sequence_hold_frames: int,
    sequence_fps: float,
    sequence_easing: str,
    sequence_motion_style: str,
    sequence_point_limit: int | None,
    sequence_delay_fraction: float,
    sequence_start_noise_scale: float,
    sequence_frustum_fade_frames: int,
    sequence_random_seed: int,
    sequence_tornado_turns: float,
    sequence_tornado_radius_scale: float,
    sequence_tornado_radius_decay_power: float,
    sequence_tornado_wander_fraction: float,
    sequence_tornado_wander_scale: float,
    sequence_source_focus_scale: float,
    sequence_start_opacity: float,
    film_camera_mode: str,
    film_target_mode: str,
    film_motion_easing: str,
    film_top_elevation_start_deg: float,
    film_top_elevation_end_deg: float,
    film_top_yaw_start_deg: float,
    film_top_yaw_end_deg: float,
    film_yaw_start_deg: float,
    film_yaw_end_deg: float,
    film_vertical_offset_deg: float,
    film_frame_vertical_offset_scale: float,
    film_zoom: float,
    film_yaw_start_deg_final: float,
    film_yaw_end_deg_final: float,
    film_vertical_offset_deg_final: float,
    film_frame_vertical_offset_scale_final: float,
    film_zoom_final: float,
    export_enabled: bool,
    export_frames_dir: Path | None,
    export_video_path: Path | None,
    export_fps: float | None,
    export_keep_frames: bool,
) -> None:
    o3d = require_open3d()
    np = require_numpy()

    if intrinsics is None:
        raise ValidationError(
            "Sequence-build animation requires intrinsics so points can start on representative frustum planes."
        )
    if sequence_speed_multiplier <= 0.0:
        raise ValidationError("--sequence-speed-multiplier must be positive.")
    if not (0.0 < sequence_projection_start_scale <= 1.0):
        raise ValidationError("--sequence-projection-start-scale must be in (0, 1].")
    if not (0.0 <= sequence_overlap_fraction <= 1.0):
        raise ValidationError("--sequence-overlap-fraction must be in [0, 1].")
    if sequence_animation_frames <= 1:
        raise ValidationError("--sequence-animation-frames must be greater than 1.")
    if sequence_hold_frames < 0:
        raise ValidationError("--sequence-hold-frames must be non-negative.")
    if sequence_fps <= 0.0:
        raise ValidationError("--sequence-fps must be positive.")
    if sequence_motion_style not in ("linear", "tornado"):
        raise ValidationError("--sequence-motion-style must be 'linear' or 'tornado'.")
    if sequence_point_limit is not None and sequence_point_limit <= 0:
        raise ValidationError("--sequence-point-limit must be positive when provided.")
    if not (0.0 <= sequence_delay_fraction < 1.0):
        raise ValidationError("--sequence-delay-fraction must be in [0, 1).")
    if sequence_start_noise_scale < 0.0:
        raise ValidationError("--sequence-start-noise-scale must be non-negative.")
    if sequence_frustum_fade_frames < 0:
        raise ValidationError("--sequence-frustum-fade-frames must be non-negative.")
    if sequence_tornado_turns < 0.0:
        raise ValidationError("--sequence-tornado-turns must be non-negative.")
    if sequence_tornado_radius_scale < 0.0:
        raise ValidationError("--sequence-tornado-radius-scale must be non-negative.")
    if sequence_tornado_radius_decay_power <= 0.0:
        raise ValidationError("--sequence-tornado-radius-decay-power must be positive.")
    if not (0.0 <= sequence_tornado_wander_fraction < 1.0):
        raise ValidationError("--sequence-tornado-wander-fraction must be in [0, 1).")
    if sequence_tornado_wander_scale < 0.0:
        raise ValidationError("--sequence-tornado-wander-scale must be non-negative.")
    if sequence_source_focus_scale <= 0.0:
        raise ValidationError("--sequence-source-focus-scale must be positive.")
    if not (0.0 <= sequence_start_opacity <= 1.0):
        raise ValidationError("--sequence-start-opacity must be in [0, 1].")
    if export_fps is not None and export_fps <= 0.0:
        raise ValidationError("--export-fps must be positive when provided.")
    if export_enabled and export_frames_dir is None:
        raise ValidationError("Sequence-build export is enabled, but --export-frames-dir is not configured.")

    prebuilt_sequences, animated_sequences = split_sequences_for_initial_state(
        sequences=sequences,
        animation_start_scan_index=animation_start_scan_index,
    )
    scene_extent = compute_scene_extent(sequences)
    camera_marker_radius = max(scene_extent * camera_marker_scale, scene_extent * 0.0025)
    camera_frustum_depth = max(scene_extent * camera_frustum_scale, scene_extent * 0.02)
    projections = prepare_sequence_projection_data(
        sequences=animated_sequences,
        intrinsics=intrinsics,
        representative_capture_mode=representative_capture_mode,
        representative_capture_index=representative_capture_index,
        use_point_cloud_colors=use_point_cloud_colors,
        point_limit=sequence_point_limit,
        camera_frustum_depth=camera_frustum_depth,
        camera_frustum_plane_scale=camera_frustum_plane_scale,
        sequence_projection_start_scale=sequence_projection_start_scale,
        sequence_start_noise_scale=sequence_start_noise_scale,
        sequence_delay_fraction=sequence_delay_fraction,
        random_seed=sequence_random_seed,
        sequence_tornado_radius_scale=sequence_tornado_radius_scale,
        sequence_tornado_wander_scale=sequence_tornado_wander_scale,
        sequence_source_focus_scale=sequence_source_focus_scale,
    )
    for projection in projections:
        LOGGER.info(
            "Scan %03d representative camera -> capture %d | animated_points=%d",
            projection.sequence.scan_index,
            projection.representative_capture_index,
            projection.final_points.shape[0],
        )
    LOGGER.info(
        "Initial settled scans: %s | animated scans: %s",
        [sequence.scan_index for sequence in prebuilt_sequences],
        [projection.sequence.scan_index for projection in projections],
    )

    filming_base_offset_direction = compute_default_side_offset_direction(
        projections[0].representative_capture.T_base_camera
    )
    prebuilt_point_sets = [
        sample_sequence_points(
            sequence=sequence,
            order_index=order_index,
            total_sequence_count=len(sequences),
            use_point_cloud_colors=use_point_cloud_colors,
            point_limit=sequence_point_limit,
        )[0]
        for order_index, sequence in enumerate(prebuilt_sequences)
    ]
    all_final_point_sets = [*prebuilt_point_sets, *[projection.final_points for projection in projections]]
    all_final_points = np.vstack(all_final_point_sets)
    effective_sequence_fps = sequence_fps * sequence_speed_multiplier
    output_video_fps = export_fps if export_fps is not None else effective_sequence_fps
    frame_output_dir = export_frames_dir
    if export_enabled:
        assert frame_output_dir is not None
        prepare_export_frames_directory(frame_output_dir)

    preview_markers: list[Any] = []
    preview_frustums: list[Any] = []
    preview_clouds: list[Any] = []

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Cinematic Scan Build",
        width=window_width,
        height=window_height,
    )
    try:
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15 * scene_extent)
        vis.add_geometry(coordinate_frame)
        for order_index, sequence in enumerate(prebuilt_sequences):
            vis.add_geometry(
                create_sampled_visual_point_cloud(
                    sequence=sequence,
                    order_index=order_index,
                    total_sequence_count=len(sequences),
                    use_point_cloud_colors=use_point_cloud_colors,
                    point_limit=sequence_point_limit,
                )
            )

        if show_representative_camera_positions:
            for projection in projections:
                marker = create_marker_sphere(
                    position=projection.representative_capture.camera_position,
                    radius=camera_marker_radius,
                    color=background_color,
                )
                preview_markers.append(marker)
                vis.add_geometry(marker)

        if show_representative_camera_frustums:
            for projection in projections:
                frustum = create_camera_frustum(
                    T_base_camera=projection.representative_capture.T_base_camera,
                    intrinsics=intrinsics,
                    depth=camera_frustum_depth,
                    plane_scale=camera_frustum_plane_scale,
                    color=background_color,
                )
                preview_frustums.append(frustum)
                vis.add_geometry(frustum)

        for _ in projections:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=float))
            cloud.colors = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=float))
            preview_clouds.append(cloud)
            vis.add_geometry(cloud)

        render_option = vis.get_render_option()
        render_option.point_size = float(point_size)
        render_option.background_color = np.asarray(background_color, dtype=float)
        render_option.mesh_show_back_face = True

        initial_target_point = compute_sequence_build_target_point(
            projections=projections,
            visible_point_sets=all_final_point_sets,
            target_mode=film_target_mode,
        )
        set_scan_build_filming_camera(
            vis=vis,
            base_offset_direction=filming_base_offset_direction,
            lookat=initial_target_point,
            scene_extent=scene_extent,
            film_camera_mode=film_camera_mode,
            film_top_elevation_start_deg=film_top_elevation_start_deg,
            film_top_elevation_end_deg=film_top_elevation_end_deg,
            film_top_yaw_start_deg=film_top_yaw_start_deg,
            film_top_yaw_end_deg=film_top_yaw_end_deg,
            film_yaw_start_deg=film_yaw_start_deg,
            film_yaw_end_deg=film_yaw_end_deg,
            film_vertical_offset_deg=film_vertical_offset_deg,
            film_frame_vertical_offset_scale=film_frame_vertical_offset_scale,
            film_zoom=film_zoom,
            film_yaw_start_deg_final=film_yaw_start_deg_final,
            film_yaw_end_deg_final=film_yaw_end_deg_final,
            film_vertical_offset_deg_final=film_vertical_offset_deg_final,
            film_frame_vertical_offset_scale_final=film_frame_vertical_offset_scale_final,
            film_zoom_final=film_zoom_final,
            motion_t=0.0,
        )
        vis.poll_events()
        vis.update_renderer()

        LOGGER.info(
            "Animating %d scan folders in numeric order | speed_multiplier=%.2f | "
            "overlap_fraction=%.2f | motion_style=%s | start_scan=%s | "
            "representative_capture_mode=%s | export_enabled=%s",
            len(projections),
            sequence_speed_multiplier,
            sequence_overlap_fraction,
            sequence_motion_style,
            animation_start_scan_index if animation_start_scan_index is not None else projections[0].sequence.scan_index,
            representative_capture_mode if representative_capture_index is None else f"fixed_index_{representative_capture_index}",
            export_enabled,
        )
        if export_enabled:
            LOGGER.info(
                "Sequence-build export enabled | frames_dir=%s | output_video=%s | output_fps=%.3f",
                frame_output_dir,
                export_video_path if export_video_path is not None else "<frames-only>",
                output_video_fps,
            )

        frame_interval = 1.0 / effective_sequence_fps
        sequence_start_stride = int(round((sequence_animation_frames - 1) * sequence_overlap_fraction))
        sequence_start_frames = [
            projection_index * sequence_start_stride for projection_index in range(len(projections))
        ]
        total_animation_frames = max(
            start_frame + sequence_animation_frames + sequence_frustum_fade_frames
            for start_frame in sequence_start_frames
        )
        animation_start_time = time.perf_counter()
        exported_frame_index = 0

        for frame_idx in range(total_animation_frames):
            visible_point_sets: list[Any] = [*prebuilt_point_sets]
            for projection_index, (projection, cloud, start_frame) in enumerate(
                zip(projections, preview_clouds, sequence_start_frames)
            ):
                current_points, current_colors = evaluate_sequence_frame_points(
                    projection=projection,
                    sequence_frame_index=frame_idx - start_frame,
                    sequence_frame_count=sequence_animation_frames,
                    easing=sequence_easing,
                    motion_style=sequence_motion_style,
                    tornado_turns=sequence_tornado_turns,
                    tornado_radius_decay_power=sequence_tornado_radius_decay_power,
                    tornado_wander_fraction=sequence_tornado_wander_fraction,
                    tornado_wander_scale=sequence_tornado_wander_scale,
                    start_opacity=sequence_start_opacity,
                    background_color=background_color,
                )
                cloud.points = o3d.utility.Vector3dVector(current_points)
                cloud.colors = o3d.utility.Vector3dVector(current_colors)
                vis.update_geometry(cloud)
                visible_point_sets.append(current_points)

                frustum_intensity = compute_preview_frustum_intensity(
                    group_frame_index=frame_idx - start_frame,
                    group_frame_count=sequence_animation_frames,
                    fade_frame_count=sequence_frustum_fade_frames,
                )
                dynamic_camera_color = blend_rgb(
                    source_color=background_color,
                    target_color=camera_frustum_color,
                    blend_t=frustum_intensity,
                )
                if show_representative_camera_positions:
                    set_mesh_uniform_color(preview_markers[projection_index], dynamic_camera_color)
                    vis.update_geometry(preview_markers[projection_index])
                if show_representative_camera_frustums:
                    set_line_set_uniform_color(preview_frustums[projection_index], dynamic_camera_color)
                    vis.update_geometry(preview_frustums[projection_index])

            camera_motion_t = (
                1.0 if total_animation_frames <= 1 else frame_idx / float(total_animation_frames - 1)
            )
            camera_motion_t = easing_value(camera_motion_t, film_motion_easing)
            target_point = compute_sequence_build_target_point(
                projections=projections,
                visible_point_sets=visible_point_sets,
                target_mode=film_target_mode,
            )
            set_scan_build_filming_camera(
                vis=vis,
                base_offset_direction=filming_base_offset_direction,
                lookat=target_point,
                scene_extent=scene_extent,
                film_camera_mode=film_camera_mode,
                film_top_elevation_start_deg=film_top_elevation_start_deg,
                film_top_elevation_end_deg=film_top_elevation_end_deg,
                film_top_yaw_start_deg=film_top_yaw_start_deg,
                film_top_yaw_end_deg=film_top_yaw_end_deg,
                film_yaw_start_deg=film_yaw_start_deg,
                film_yaw_end_deg=film_yaw_end_deg,
                film_vertical_offset_deg=film_vertical_offset_deg,
                film_frame_vertical_offset_scale=film_frame_vertical_offset_scale,
                film_zoom=film_zoom,
                film_yaw_start_deg_final=film_yaw_start_deg_final,
                film_yaw_end_deg_final=film_yaw_end_deg_final,
                film_vertical_offset_deg_final=film_vertical_offset_deg_final,
                film_frame_vertical_offset_scale_final=film_frame_vertical_offset_scale_final,
                film_zoom_final=film_zoom_final,
                motion_t=camera_motion_t,
            )
            vis.poll_events()
            vis.update_renderer()
            if export_enabled:
                capture_export_frame(vis, frame_output_dir / f"frame_{exported_frame_index:06d}.png")
                exported_frame_index += 1
            else:
                target_time = animation_start_time + (frame_idx + 1) * frame_interval
                remaining = target_time - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)

        final_target_point = compute_sequence_build_target_point(
            projections=projections,
            visible_point_sets=all_final_point_sets,
            target_mode=film_target_mode,
        )
        hold_start_time = time.perf_counter()
        for hold_idx in range(sequence_hold_frames):
            for marker in preview_markers:
                set_mesh_uniform_color(marker, background_color)
                vis.update_geometry(marker)
            for frustum in preview_frustums:
                set_line_set_uniform_color(frustum, background_color)
                vis.update_geometry(frustum)
            for cloud in preview_clouds:
                vis.update_geometry(cloud)
            set_scan_build_filming_camera(
                vis=vis,
                base_offset_direction=filming_base_offset_direction,
                lookat=final_target_point,
                scene_extent=scene_extent,
                film_camera_mode=film_camera_mode,
                film_top_elevation_start_deg=film_top_elevation_start_deg,
                film_top_elevation_end_deg=film_top_elevation_end_deg,
                film_top_yaw_start_deg=film_top_yaw_start_deg,
                film_top_yaw_end_deg=film_top_yaw_end_deg,
                film_yaw_start_deg=film_yaw_start_deg,
                film_yaw_end_deg=film_yaw_end_deg,
                film_vertical_offset_deg=film_vertical_offset_deg,
                film_frame_vertical_offset_scale=film_frame_vertical_offset_scale,
                film_zoom=film_zoom,
                film_yaw_start_deg_final=film_yaw_start_deg_final,
                film_yaw_end_deg_final=film_yaw_end_deg_final,
                film_vertical_offset_deg_final=film_vertical_offset_deg_final,
                film_frame_vertical_offset_scale_final=film_frame_vertical_offset_scale_final,
                film_zoom_final=film_zoom_final,
                motion_t=1.0,
            )
            vis.poll_events()
            vis.update_renderer()
            if export_enabled:
                capture_export_frame(vis, frame_output_dir / f"frame_{exported_frame_index:06d}.png")
                exported_frame_index += 1
            else:
                target_time = hold_start_time + (hold_idx + 1) * frame_interval
                remaining = target_time - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)

        if export_enabled:
            if export_video_path is not None:
                encode_exported_frames_to_video(
                    frames_dir=frame_output_dir,
                    output_video_path=export_video_path,
                    output_fps=output_video_fps,
                )
                LOGGER.info(
                    "Sequence-build video export complete | video=%s | frames=%d",
                    export_video_path,
                    exported_frame_index,
                )
                if not export_keep_frames:
                    for frame_path in frame_output_dir.glob("frame_*.png"):
                        frame_path.unlink()
                    LOGGER.info(
                        "Deleted exported frame sequence after encoding | frames_dir=%s",
                        frame_output_dir,
                    )
            else:
                LOGGER.info(
                    "Sequence-build frame export complete without video encoding | frames_dir=%s | frames=%d",
                    frame_output_dir,
                    exported_frame_index,
                )
            return

        vis.run()
    finally:
        vis.destroy_window()


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)

    try:
        dataset_root = resolve_optional_path_arg(args.dataset_root)
        extrinsics_path = resolve_optional_path_arg(args.extrinsics_path)
        intrinsics_path = resolve_optional_path_arg(args.intrinsics_path)
        export_frames_dir = resolve_optional_path_arg(args.export_frames_dir)
        export_video_path = resolve_optional_path_arg(args.export_video_path)
        background_color = parse_color(args.background_color)
        camera_frustum_color = parse_color(args.camera_frustum_color)

        if dataset_root is None:
            raise ValidationError(
                "Dataset root is not configured. Edit SCRIPT_INPUTS.dataset_root at the top of the script or pass --dataset-root."
            )
        if extrinsics_path is None:
            raise ValidationError(
                "Extrinsics path is not configured. Edit SCRIPT_INPUTS.extrinsics_path at the top of the script or pass --extrinsics-path."
            )
        if args.expected_captures <= 0:
            raise ValidationError("--expected-captures must be a positive integer.")
        if args.scan_index_start is not None and args.scan_index_start < 0:
            raise ValidationError("--scan-index-start must be non-negative when provided.")
        if args.scan_index_end is not None and args.scan_index_end < 0:
            raise ValidationError("--scan-index-end must be non-negative when provided.")
        if args.animation_start_scan_index is not None and args.animation_start_scan_index < 0:
            raise ValidationError("--animation-start-scan-index must be non-negative when provided.")
        if (
            args.scan_index_start is not None
            and args.scan_index_end is not None
            and args.scan_index_end < args.scan_index_start
        ):
            raise ValidationError("--scan-index-end must be greater than or equal to --scan-index-start.")
        if args.export_enabled and args.no_visualization:
            raise ValidationError("--export-enabled cannot be combined with --no-visualization.")
        if args.export_enabled and not args.animate_sequence_build:
            raise ValidationError(
                "Sequence-build export currently supports only the animated path. Disable --export-enabled or keep --animate-sequence-build enabled."
            )
        if args.export_enabled and export_frames_dir is None:
            raise ValidationError("Sequence-build export is enabled, but --export-frames-dir is not configured.")

        ensure_directory_exists(dataset_root, "Dataset root")
        ensure_file_exists(extrinsics_path, "Extrinsics file")
        if intrinsics_path is not None:
            ensure_file_exists(intrinsics_path, "Intrinsics file")

        T_extrinsic, resolved_key = load_extrinsic_transform(
            extrinsics_path=extrinsics_path,
            convention=args.extrinsic_convention,
            extrinsic_key=args.extrinsic_key,
        )
        LOGGER.info(
            "Extrinsics loaded from %s | convention=%s | resolved_key=%s",
            extrinsics_path,
            args.extrinsic_convention,
            resolved_key,
        )

        intrinsics = load_intrinsics(intrinsics_path) if intrinsics_path is not None else None
        summarize_intrinsics(intrinsics)

        sequences = load_sequences_with_optional_skips(
            dataset_root=dataset_root,
            expected_captures=args.expected_captures,
            T_extrinsic=T_extrinsic,
            convention=args.extrinsic_convention,
            scan_index_start=args.scan_index_start,
            scan_index_end=args.scan_index_end,
            skip_invalid_sequences=args.skip_invalid_sequences,
        )
        print_dataset_summary(sequences, dataset_root)
        resolved_animation_start_scan_index = resolve_animation_start_scan_index(
            sequences=sequences,
            animation_start_scan_index=args.animation_start_scan_index,
            animation_start_folder=args.animation_start_folder,
        )

        if not args.no_visualization:
            if args.animate_sequence_build:
                animate_sequence_build(
                    sequences=sequences,
                    intrinsics=intrinsics,
                    point_size=args.point_size,
                    background_color=background_color,
                    window_width=args.window_width,
                    window_height=args.window_height,
                    use_point_cloud_colors=args.use_point_cloud_colors,
                    animation_start_scan_index=resolved_animation_start_scan_index,
                    representative_capture_mode=args.representative_capture_mode,
                    representative_capture_index=args.representative_capture_index,
                    show_representative_camera_positions=args.show_representative_camera_positions,
                    show_representative_camera_frustums=args.show_representative_camera_frustums,
                    camera_marker_scale=args.camera_marker_scale,
                    camera_frustum_scale=args.camera_frustum_scale,
                    camera_frustum_plane_scale=args.camera_frustum_plane_scale,
                    camera_frustum_color=camera_frustum_color,
                    sequence_speed_multiplier=args.sequence_speed_multiplier,
                    sequence_overlap_fraction=args.sequence_overlap_fraction,
                    sequence_projection_start_scale=args.sequence_projection_start_scale,
                    sequence_animation_frames=args.sequence_animation_frames,
                    sequence_hold_frames=args.sequence_hold_frames,
                    sequence_fps=args.sequence_fps,
                    sequence_easing=args.sequence_easing,
                    sequence_motion_style=args.sequence_motion_style,
                    sequence_point_limit=args.sequence_point_limit,
                    sequence_delay_fraction=args.sequence_delay_fraction,
                    sequence_start_noise_scale=args.sequence_start_noise_scale,
                    sequence_frustum_fade_frames=args.sequence_frustum_fade_frames,
                    sequence_random_seed=args.sequence_random_seed,
                    sequence_tornado_turns=args.sequence_tornado_turns,
                    sequence_tornado_radius_scale=args.sequence_tornado_radius_scale,
                    sequence_tornado_radius_decay_power=args.sequence_tornado_radius_decay_power,
                    sequence_tornado_wander_fraction=args.sequence_tornado_wander_fraction,
                    sequence_tornado_wander_scale=args.sequence_tornado_wander_scale,
                    sequence_source_focus_scale=args.sequence_source_focus_scale,
                    sequence_start_opacity=args.sequence_start_opacity,
                    film_camera_mode=args.film_camera_mode,
                    film_target_mode=args.film_target_mode,
                    film_motion_easing=args.film_motion_easing,
                    film_top_elevation_start_deg=args.film_top_elevation_start_deg,
                    film_top_elevation_end_deg=args.film_top_elevation_end_deg,
                    film_top_yaw_start_deg=args.film_top_yaw_start_deg,
                    film_top_yaw_end_deg=args.film_top_yaw_end_deg,
                    film_yaw_start_deg=args.film_yaw_start_deg,
                    film_yaw_end_deg=args.film_yaw_end_deg,
                    film_vertical_offset_deg=args.film_vertical_offset_deg,
                    film_frame_vertical_offset_scale=args.film_frame_vertical_offset_scale,
                    film_zoom=args.film_zoom,
                    film_yaw_start_deg_final=args.film_yaw_start_deg_final,
                    film_yaw_end_deg_final=args.film_yaw_end_deg_final,
                    film_vertical_offset_deg_final=args.film_vertical_offset_deg_final,
                    film_frame_vertical_offset_scale_final=args.film_frame_vertical_offset_scale_final,
                    film_zoom_final=args.film_zoom_final,
                    export_enabled=args.export_enabled,
                    export_frames_dir=export_frames_dir,
                    export_video_path=export_video_path,
                    export_fps=args.export_fps,
                    export_keep_frames=args.export_keep_frames,
                )
            else:
                show_static_sequence_view(
                    sequences=sequences,
                    intrinsics=intrinsics,
                    point_size=args.point_size,
                    background_color=background_color,
                    window_width=args.window_width,
                    window_height=args.window_height,
                    camera_marker_scale=args.camera_marker_scale,
                    camera_frustum_scale=args.camera_frustum_scale,
                    camera_frustum_plane_scale=args.camera_frustum_plane_scale,
                    camera_frustum_color=camera_frustum_color,
                    use_point_cloud_colors=args.use_point_cloud_colors,
                    representative_capture_mode=args.representative_capture_mode,
                    representative_capture_index=args.representative_capture_index,
                    show_representative_camera_positions=args.show_representative_camera_positions,
                    show_representative_camera_frustums=args.show_representative_camera_frustums,
                )

        return 0
    except Stage1Error as exc:
        LOGGER.error("%s", exc)
        return 1
    except Exception as exc:  # pragma: no cover - defensive top-level fallback
        LOGGER.exception("Unexpected failure: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
