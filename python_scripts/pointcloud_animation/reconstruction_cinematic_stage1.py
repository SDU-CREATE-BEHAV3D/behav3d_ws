#!/usr/bin/env python3
"""Stage 1 loader and static visualizer for cinematic reconstruction previews.

This script validates a dataset of reconstructed point clouds plus capture
manifests. It does not perform reconstruction. The `.ply` files are assumed to
already be expressed in the same world/base frame used by the robot poses.

Coordinate-system assumptions:
- `T_base_tool0` in the manifest is interpreted as the pose of `tool0`
  expressed in the `base` frame.
- The camera extrinsic convention is explicit and never guessed silently:
  - `T_tool0_camera`: `T_base_camera = T_base_tool0 @ T_tool0_camera`
  - `T_camera_tool0`: `T_base_camera = T_base_tool0 @ inverse(T_camera_tool0)`
- Intrinsics are loaded for validation and later extensions, but Stage 1 only
  visualizes point clouds and camera positions.
"""

from __future__ import annotations

import argparse
import colorsys
import json
import logging
import math
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


@dataclass(frozen=True)
class ScriptInputs:
    """Editable defaults for running the script directly from VS Code."""

    dataset_root: str = "~/Downloads/260227_160709/print_scan_049"
    extrinsics_path: str = "~/Downloads/260227_160709/config/extrinsics.yaml"
    intrinsics_path: str | None = "~/Downloads/260227_160709/config/ir_intrinsics.yaml"
    extrinsic_convention: str = "T_tool0_camera"
    extrinsic_key: str | None = "frames.T_tool0_ir"
    expected_captures: int = 12
    point_size: float = 3.0
    camera_marker_scale: float = 0.0075
    camera_frustum_scale: float = 0.18
    camera_frustum_plane_scale: float = 0.7
    camera_frustum_color: str = "0.68,0.88,1.0"
    use_point_cloud_colors: bool = True
    show_all_camera_positions: bool = True
    show_all_camera_frustums: bool = True
    frustum_start_index: int = 0
    frustum_end_index: int | None = 11
    animate_first_group_preview: bool = True
    preview_speed_multiplier: float = 1
    preview_second_group_start_fraction: float = 0.2
    preview_projection_start_scale: float = 1.0
    preview_animation_frames: int = 40
    preview_hold_frames: int = 45
    preview_fps: float = 30.0
    preview_easing: str = "ease_out_cubic"
    preview_point_limit: int | None = None
    preview_grouping_mode: str = "randomized_balanced"
    preview_capture_order_mode: str = "index_order"
    preview_delay_fraction: float = 0.5
    preview_start_noise_scale: float = 0.009
    preview_frustum_fade_frames: int = 6
    preview_random_seed: int = 7
    film_camera_mode: str = "orbit_side"
    film_target_mode: str = "final_centroid"
    film_motion_easing: str = "ease_in_out_cubic"
    film_yaw_start_deg: float = -65.0
    film_yaw_end_deg: float = 35.0
    film_vertical_offset_deg: float = 50.0
    film_frame_vertical_offset_scale: float = 0.35
    film_zoom: float = 0.3
    film_yaw_start_deg_final: float = -65.0
    film_yaw_end_deg_final: float = 55.0
    film_vertical_offset_deg_final: float = 4.0
    film_frame_vertical_offset_scale_final: float = 0.35
    film_zoom_final: float = 0.4
    export_enabled: bool = True
    export_frames_dir: str = "~/Downloads/260227_160709/exports/print_scan_049_projection_frames"
    export_video_path: str | None = "~/Downloads/260227_160709/exports/print_scan_049_projection_preview.mp4"
    export_fps: float | None = None
    export_keep_frames: bool = False
    background_color: str = "0.02,0.02,0.03"
    window_width: int = 2400
    window_height: int = 1600
    no_visualization: bool = False
    verbose: bool = False


# Edit the SCRIPT_INPUTS block below if you want to run the script from the
# VS Code Run button without passing CLI arguments. CLI arguments still
# override these defaults. The dataclass field values above are constructor
# defaults, but the active Run-button configuration comes from SCRIPT_INPUTS.
# This session's config folder provides `frames.T_tool0_ir` and `frames.T_tool0_color`.
# The default below targets one validated scan first and uses the IR calibration
# so the extrinsics and intrinsics match.
SCRIPT_INPUTS = ScriptInputs(
    # Input data and calibration
    dataset_root="~/Downloads/260227_160709/print_scan_049",  # Session root or one print_scan_* folder to load.
    extrinsics_path="~/Downloads/260227_160709/config/extrinsics.yaml",  # Tool-to-camera extrinsics file.
    intrinsics_path="~/Downloads/260227_160709/config/ir_intrinsics.yaml",  # Camera intrinsics file used for frustums/projected starts.
    extrinsic_convention="T_tool0_camera",  # Whether the extrinsics file stores T_tool0_camera or T_camera_tool0.
    extrinsic_key="frames.T_tool0_ir",  # Dotted key used when the extrinsics file contains multiple transforms.
    expected_captures=12,  # Expected number of manifest captures per scan.

    # General scene rendering
    point_size=4,  # Render size of the animated/static point cloud in Open3D.
    background_color="0.02,0.02,0.03",  # Viewer background color as r,g,b in 0-1 or 0-255.
    window_width=2400,  # Viewer window width in pixels.
    window_height=1600,  # Viewer window height in pixels.

    # Camera markers and frustums
    camera_marker_scale=0.0075,  # Sphere marker size at each camera position, as a fraction of scene extent.
    camera_frustum_scale=0.18,  # Frustum length/depth, as a fraction of scene extent.
    camera_frustum_plane_scale=0.7,  # Frustum front-rectangle width/height multiplier.
    camera_frustum_color="0.68,0.88,1.0",  # Color of camera spheres and frustum wireframes.
    show_all_camera_positions=True,  # Show camera-position sphere markers.
    show_all_camera_frustums=True,  # Show camera frustum wireframes.
    frustum_start_index=0,  # First capture index whose marker/frustum is shown and whose point group is animated in preview mode.
    frustum_end_index=11,  # Last capture index whose marker/frustum is shown and whose point group is animated in preview mode.

    # Point-cloud coloring
    use_point_cloud_colors=True,  # Use RGB stored in the .ply instead of a fallback tint.

    # Preview mode selection
    animate_first_group_preview=True,  # Run the projection preview instead of only the static viewer.

    # Preview timing and speed
    preview_speed_multiplier=1,  # Main speed knob: larger values make the projection play faster.
    preview_second_group_start_fraction=0.2,  # When each next capture group starts, as a fraction of the previous group's duration.
    preview_animation_frames=40,  # Interpolation frame count for one group's travel from frustum to final points.
    preview_hold_frames=45,  # Extra settled frames shown before handing control to the interactive viewer.
    preview_fps=30.0,  # Base playback FPS before the speed multiplier is applied.
    preview_easing="ease_out_cubic",  # Motion easing curve for point travel.

    # Preview projection look
    preview_projection_start_scale=1.0,  # 1.0 starts exactly on the frustum plane; smaller values start closer to the camera.
    preview_point_limit=None,  # Optional max number of animated points; None uses the full assigned group.
    preview_grouping_mode="randomized_balanced",  # How points are assigned to capture groups for the preview.
    preview_capture_order_mode="index_order",  # Whether the selected capture groups animate in index order or a randomized order.
    preview_delay_fraction=0.5,  # Random per-point launch delay fraction; lower values make the group feel snappier.
    preview_start_noise_scale=0.009,  # Random jitter on the frustum plane to avoid a perfectly rigid sheet of points.
    preview_frustum_fade_frames=6,  # How many frames each capture frustum takes to fade out after its point group finishes.
    preview_random_seed=7,  # Seed so the preview grouping/noise stays repeatable.
    film_camera_mode="orbit_side",  # Virtual filming camera mode: static side view or an orbit around that side view.
    film_target_mode="final_centroid",  # What the filming camera looks at: a fixed final centroid or the currently visible geometry centroid.
    film_motion_easing="ease_in_out_cubic",  # Easing used for the filming camera motion itself.
    film_yaw_start_deg=-65.0,  # Initial starting yaw offset in degrees relative to the default side view.
    film_yaw_end_deg=35.0,  # Initial ending yaw offset in degrees relative to the default side view.
    film_vertical_offset_deg=50.0,  # Initial vertical filming-camera offset in degrees relative to the default side view.
    film_frame_vertical_offset_scale=0.35,  # Initial vertical framing offset; positive values move the previewed scene lower in frame.
    film_zoom=0.3,  # Initial Open3D zoom for the filming camera.
    film_yaw_start_deg_final=-65.0,  # Final starting yaw offset in degrees relative to the default side view.
    film_yaw_end_deg_final=55.0,  # Final ending yaw offset in degrees relative to the default side view.
    film_vertical_offset_deg_final=4.0,  # Final vertical filming-camera offset in degrees relative to the default side view.
    film_frame_vertical_offset_scale_final=0.35,  # Final vertical framing offset; positive values move the previewed scene lower in frame.
    film_zoom_final=0.4,  # Final Open3D zoom for the filming camera.
    export_enabled=False,  # Export the preview animation directly to frames/video instead of leaving it only as an interactive preview.
    export_frames_dir="~/Downloads/260227_160709/exports/print_scan_049_projection_frames",  # Folder where numbered PNG frames are written during export.
    export_video_path="~/Downloads/260227_160709/exports/print_scan_049_projection_preview.mp4",  # Output .mp4 path for the encoded preview video.
    export_fps=None,  # Output video FPS. None uses preview_fps * preview_speed_multiplier so exported speed matches the preview speed.
    export_keep_frames=False,  # Keep the PNG frame sequence after video encoding finishes.

    # Utility flags
    no_visualization=False,  # Validate and print summaries only; do not open the viewer.
    verbose=False,  # Print extra debug logging while loading and validating data.
)


LOGGER = logging.getLogger("reconstruction_cinematic_stage1")
SCAN_DIR_PATTERN = re.compile(r"^print_scan_(\d+)$")


class Stage1Error(RuntimeError):
    """Base exception for Stage 1 validation and loading errors."""


class DependencyError(Stage1Error):
    """Raised when a required runtime dependency is missing."""


class ValidationError(Stage1Error):
    """Raised when an input file or parameter is invalid."""


class ManifestValidationError(ValidationError):
    """Raised when a manifest is missing required content."""


class CalibrationParseError(ValidationError):
    """Raised when intrinsics or extrinsics cannot be parsed."""


@dataclass(frozen=True)
class IntrinsicsData:
    path: Path
    width: int
    height: int
    camera_matrix: Any
    distortion_coefficients: Any


@dataclass(frozen=True)
class CapturePose:
    capture_index: int
    timestamp: Any
    color_path: str | None
    depth_path: str | None
    ir_path: str | None
    T_base_tool0: Any
    T_base_camera: Any
    camera_position: Any


@dataclass
class SequenceData:
    scan_index: int
    sequence_path: Path
    manifest_path: Path
    reconstruct_path: Path
    ply_path: Path
    manifest_keys: list[str]
    captures: list[CapturePose]
    point_cloud: Any
    point_count: int
    bbox_min: Any
    bbox_max: Any
    bbox_extent: Any
    camera_positions: Any
    sequence_centroid: Any
    has_colors: bool


@dataclass(frozen=True)
class PreviewGroupData:
    capture_index: int
    capture: CapturePose
    final_points: Any
    start_points: Any
    colors: Any
    launch_delays: Any


def require_numpy():
    try:
        import numpy as np
    except ModuleNotFoundError as exc:
        raise DependencyError(
            "Missing dependency 'numpy'. Install numpy before running this script."
        ) from exc
    return np


def require_open3d():
    try:
        import open3d as o3d
    except ModuleNotFoundError as exc:
        raise DependencyError(
            "Missing dependency 'open3d'. Install open3d before running this script."
        ) from exc
    return o3d


def require_yaml():
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise DependencyError(
            "Missing dependency 'pyyaml'. Install pyyaml before running this script."
        ) from exc
    return yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=(
            "Stage 1 validation and static visualization for reconstructed "
            "print_scan_* datasets."
        )
    )
    parser.add_argument(
        "--dataset-root",
        default=SCRIPT_INPUTS.dataset_root,
        help="Session root containing print_scan_* folders, or a single print_scan_* folder.",
    )
    parser.add_argument(
        "--extrinsics-path",
        default=SCRIPT_INPUTS.extrinsics_path,
        help="Path to the tool/camera extrinsics file (YAML, YML, or JSON).",
    )
    parser.add_argument(
        "--intrinsics-path",
        default=SCRIPT_INPUTS.intrinsics_path,
        help=(
            "Optional path to a camera intrinsics file. If provided, it is "
            "validated and summarized for later stages."
        ),
    )
    parser.add_argument(
        "--extrinsic-convention",
        choices=("T_tool0_camera", "T_camera_tool0"),
        default=SCRIPT_INPUTS.extrinsic_convention,
        help=(
            "Convention used by the provided extrinsic transform. This is "
            "never inferred silently."
        ),
    )
    parser.add_argument(
        "--extrinsic-key",
        type=str,
        default=SCRIPT_INPUTS.extrinsic_key,
        help=(
            "Optional dotted path to the transform inside the extrinsics file, "
            "for example 'frames.T_tool0_ir'. Required if the file contains "
            "multiple transforms and no unambiguous default exists."
        ),
    )
    parser.add_argument(
        "--expected-captures",
        type=int,
        default=SCRIPT_INPUTS.expected_captures,
        help="Expected number of captures per sequence. Defaults to 12.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=SCRIPT_INPUTS.point_size,
        help="Rendered point size for the Open3D static visualization.",
    )
    parser.add_argument(
        "--camera-marker-scale",
        type=float,
        default=SCRIPT_INPUTS.camera_marker_scale,
        help=(
            "Camera marker radius as a fraction of the scene extent for the "
            "static visualization."
        ),
    )
    parser.add_argument(
        "--camera-frustum-scale",
        type=float,
        default=SCRIPT_INPUTS.camera_frustum_scale,
        help="Camera-frustum depth as a fraction of the scene extent.",
    )
    parser.add_argument(
        "--camera-frustum-plane-scale",
        type=float,
        default=SCRIPT_INPUTS.camera_frustum_plane_scale,
        help="Scale factor for the frustum rectangle width and height at the image plane.",
    )
    parser.add_argument(
        "--camera-frustum-color",
        type=str,
        default=SCRIPT_INPUTS.camera_frustum_color,
        help="Camera frustum and camera marker color as 'r,g,b' or '#RRGGBB'.",
    )
    parser.add_argument(
        "--show-all-camera-frustums",
        action="store_true",
        default=SCRIPT_INPUTS.show_all_camera_frustums,
        help="Show a frustum for every camera pose in the sequence.",
    )
    parser.add_argument(
        "--hide-all-camera-frustums",
        action="store_false",
        dest="show_all_camera_frustums",
        help="Hide all camera frustums.",
    )
    parser.add_argument(
        "--use-point-cloud-colors",
        action="store_true",
        default=SCRIPT_INPUTS.use_point_cloud_colors,
        help="Use RGB colors stored in the reconstructed .ply when available.",
    )
    parser.add_argument(
        "--hide-point-cloud-colors",
        action="store_false",
        dest="use_point_cloud_colors",
        help="Ignore stored .ply colors and fall back to sequence tinting.",
    )
    parser.add_argument(
        "--show-all-camera-positions",
        action="store_true",
        default=SCRIPT_INPUTS.show_all_camera_positions,
        help="Show camera position markers.",
    )
    parser.add_argument(
        "--hide-all-camera-positions",
        action="store_false",
        dest="show_all_camera_positions",
        help="Hide per-capture camera position markers.",
    )
    parser.add_argument(
        "--frustum-start-index",
        type=int,
        default=SCRIPT_INPUTS.frustum_start_index,
        help="Inclusive capture index where camera marker/frustum rendering starts.",
    )
    parser.add_argument(
        "--frustum-end-index",
        type=int,
        default=SCRIPT_INPUTS.frustum_end_index,
        help="Inclusive capture index where camera marker/frustum rendering stops.",
    )
    parser.add_argument(
        "--animate-first-group-preview",
        action="store_true",
        default=SCRIPT_INPUTS.animate_first_group_preview,
        help="Animate the preview point groups in the selected capture range instead of showing only the static view.",
    )
    parser.add_argument(
        "--static-view",
        action="store_false",
        dest="animate_first_group_preview",
        help="Disable the capture-range animation preview and show the static view only.",
    )
    parser.add_argument(
        "--preview-speed-multiplier",
        type=float,
        default=SCRIPT_INPUTS.preview_speed_multiplier,
        help="Speed multiplier for the preview animation. Values above 1.0 make it faster.",
    )
    parser.add_argument(
        "--preview-second-group-start-fraction",
        type=float,
        default=SCRIPT_INPUTS.preview_second_group_start_fraction,
        help="Fraction of one group's animation duration after which the next capture group begins.",
    )
    parser.add_argument(
        "--preview-projection-start-scale",
        type=float,
        default=SCRIPT_INPUTS.preview_projection_start_scale,
        help="Scale applied to the projected point location on the frustum plane. Use 1.0 to start exactly on the frustum.",
    )
    parser.add_argument(
        "--preview-animation-frames",
        type=int,
        default=SCRIPT_INPUTS.preview_animation_frames,
        help="Frame count used for one capture group's projection animation.",
    )
    parser.add_argument(
        "--preview-hold-frames",
        type=int,
        default=SCRIPT_INPUTS.preview_hold_frames,
        help="How many frames to hold the settled preview after all selected groups have animated.",
    )
    parser.add_argument(
        "--preview-fps",
        type=float,
        default=SCRIPT_INPUTS.preview_fps,
        help="Preview playback frame rate.",
    )
    parser.add_argument(
        "--preview-easing",
        type=str,
        choices=("linear", "ease_out_cubic", "ease_in_out_cubic"),
        default=SCRIPT_INPUTS.preview_easing,
        help="Easing applied to each capture group's interpolation.",
    )
    parser.add_argument(
        "--preview-point-limit",
        type=int,
        default=SCRIPT_INPUTS.preview_point_limit,
        help="Optional cap on animated points for the preview. If omitted, animate the whole group.",
    )
    parser.add_argument(
        "--preview-grouping-mode",
        type=str,
        choices=("nearest_pose", "randomized_nearest_counts", "randomized_balanced"),
        default=SCRIPT_INPUTS.preview_grouping_mode,
        help=(
            "How preview point groups are assigned to captures. "
            "'nearest_pose' keeps spatial slices, "
            "'randomized_nearest_counts' preserves nearest-pose group sizes but shuffles points, "
            "and 'randomized_balanced' distributes points evenly across captures."
        ),
    )
    parser.add_argument(
        "--preview-capture-order-mode",
        type=str,
        choices=("index_order", "randomized_order"),
        default=SCRIPT_INPUTS.preview_capture_order_mode,
        help="Whether the selected capture groups animate in their index order or a randomized order.",
    )
    parser.add_argument(
        "--preview-delay-fraction",
        type=float,
        default=SCRIPT_INPUTS.preview_delay_fraction,
        help="Random per-point launch delay as a fraction of the total animation duration.",
    )
    parser.add_argument(
        "--preview-start-noise-scale",
        type=float,
        default=SCRIPT_INPUTS.preview_start_noise_scale,
        help="Random frustum-plane jitter scale as a fraction of the scene extent.",
    )
    parser.add_argument(
        "--preview-frustum-fade-frames",
        type=int,
        default=SCRIPT_INPUTS.preview_frustum_fade_frames,
        help="How many frames a capture frustum takes to fade out after its point group finishes.",
    )
    parser.add_argument(
        "--preview-random-seed",
        type=int,
        default=SCRIPT_INPUTS.preview_random_seed,
        help="Seed for deterministic preview jitter and delay sampling.",
    )
    parser.add_argument(
        "--film-camera-mode",
        type=str,
        choices=("static_side", "orbit_side"),
        default=SCRIPT_INPUTS.film_camera_mode,
        help="Virtual filming camera mode for the preview animation.",
    )
    parser.add_argument(
        "--film-target-mode",
        type=str,
        choices=("final_centroid", "visible_centroid"),
        default=SCRIPT_INPUTS.film_target_mode,
        help="What point the virtual filming camera should look at during the preview.",
    )
    parser.add_argument(
        "--film-motion-easing",
        type=str,
        choices=("linear", "ease_out_cubic", "ease_in_out_cubic"),
        default=SCRIPT_INPUTS.film_motion_easing,
        help="Easing applied to the virtual filming camera motion.",
    )
    parser.add_argument(
        "--film-yaw-start-deg",
        type=float,
        default=SCRIPT_INPUTS.film_yaw_start_deg,
        help="Starting yaw offset in degrees relative to the default side view.",
    )
    parser.add_argument(
        "--film-yaw-end-deg",
        type=float,
        default=SCRIPT_INPUTS.film_yaw_end_deg,
        help="Ending yaw offset in degrees relative to the default side view.",
    )
    parser.add_argument(
        "--film-vertical-offset-deg",
        type=float,
        default=SCRIPT_INPUTS.film_vertical_offset_deg,
        help="Vertical filming-camera offset in degrees relative to the default side view.",
    )
    parser.add_argument(
        "--film-frame-vertical-offset-scale",
        type=float,
        default=SCRIPT_INPUTS.film_frame_vertical_offset_scale,
        help="Vertical framing offset as a fraction of scene extent. Positive values move the scene lower in frame.",
    )
    parser.add_argument(
        "--film-zoom",
        type=float,
        default=SCRIPT_INPUTS.film_zoom,
        help="Open3D zoom value used by the virtual filming camera.",
    )
    parser.add_argument(
        "--film-yaw-start-deg-final",
        type=float,
        default=SCRIPT_INPUTS.film_yaw_start_deg_final,
        help="Final starting yaw offset in degrees relative to the default side view.",
    )
    parser.add_argument(
        "--film-yaw-end-deg-final",
        type=float,
        default=SCRIPT_INPUTS.film_yaw_end_deg_final,
        help="Final ending yaw offset in degrees relative to the default side view.",
    )
    parser.add_argument(
        "--film-vertical-offset-deg-final",
        type=float,
        default=SCRIPT_INPUTS.film_vertical_offset_deg_final,
        help="Final vertical filming-camera offset in degrees relative to the default side view.",
    )
    parser.add_argument(
        "--film-frame-vertical-offset-scale-final",
        type=float,
        default=SCRIPT_INPUTS.film_frame_vertical_offset_scale_final,
        help="Final vertical framing offset as a fraction of scene extent.",
    )
    parser.add_argument(
        "--film-zoom-final",
        type=float,
        default=SCRIPT_INPUTS.film_zoom_final,
        help="Final Open3D zoom value for the virtual filming camera.",
    )
    parser.add_argument(
        "--export-enabled",
        action="store_true",
        default=SCRIPT_INPUTS.export_enabled,
        help="Export the preview animation to numbered frames and an encoded video.",
    )
    parser.add_argument(
        "--preview-only",
        action="store_false",
        dest="export_enabled",
        help="Disable video export and keep the animation in interactive preview mode.",
    )
    parser.add_argument(
        "--export-frames-dir",
        type=str,
        default=SCRIPT_INPUTS.export_frames_dir,
        help="Directory where numbered PNG frames are written during preview export.",
    )
    parser.add_argument(
        "--export-video-path",
        type=str,
        default=SCRIPT_INPUTS.export_video_path,
        help="Optional output video path. If omitted, only the PNG frame sequence is kept.",
    )
    parser.add_argument(
        "--export-fps",
        type=float,
        default=SCRIPT_INPUTS.export_fps,
        help="Optional output video FPS. If omitted, use preview_fps * preview_speed_multiplier.",
    )
    parser.add_argument(
        "--export-keep-frames",
        action="store_true",
        default=SCRIPT_INPUTS.export_keep_frames,
        help="Keep the numbered PNG frames after successful video encoding.",
    )
    parser.add_argument(
        "--discard-export-frames",
        action="store_false",
        dest="export_keep_frames",
        help="Delete numbered PNG frames after successful video encoding.",
    )
    parser.add_argument(
        "--background-color",
        type=str,
        default=SCRIPT_INPUTS.background_color,
        help="Background color as 'r,g,b' in 0-1 or 0-255, or '#RRGGBB'.",
    )
    parser.add_argument(
        "--window-width",
        type=int,
        default=SCRIPT_INPUTS.window_width,
        help="Static visualization window width in pixels.",
    )
    parser.add_argument(
        "--window-height",
        type=int,
        default=SCRIPT_INPUTS.window_height,
        help="Static visualization window height in pixels.",
    )
    parser.add_argument(
        "--no-visualization",
        action="store_true",
        default=SCRIPT_INPUTS.no_visualization,
        help="Validate and summarize only; do not open the static viewer.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=SCRIPT_INPUTS.verbose,
        help="Enable debug logging.",
    )
    return parser.parse_args()


def configure_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="[%(levelname)s] %(message)s",
    )


def resolve_optional_path_arg(path_value: Path | str | None) -> Path | None:
    if path_value is None:
        return None

    if isinstance(path_value, Path):
        raw_value = str(path_value)
    else:
        raw_value = str(path_value)

    if not raw_value.strip():
        return None

    return Path(raw_value).expanduser().resolve()


def prepare_export_frames_directory(frames_dir: Path) -> None:
    frames_dir.mkdir(parents=True, exist_ok=True)
    for frame_path in frames_dir.glob("frame_*.png"):
        frame_path.unlink()


def cleanup_exported_frames(frames_dir: Path) -> None:
    for frame_path in frames_dir.glob("frame_*.png"):
        frame_path.unlink()


def resolve_ffmpeg_executable() -> Path | None:
    ffmpeg_path = shutil.which("ffmpeg")
    if ffmpeg_path:
        return Path(ffmpeg_path).resolve()

    python_bin_dir = Path(sys.executable).resolve().parent
    candidate = python_bin_dir / "ffmpeg"
    if candidate.exists() and candidate.is_file():
        return candidate

    return None


def capture_export_frame(vis: Any, frame_path: Path) -> None:
    vis.capture_screen_image(str(frame_path), do_render=True)
    if not frame_path.exists():
        raise ValidationError(f"Open3D did not write the expected frame image: {frame_path}")


def encode_exported_frames_to_video(
    frames_dir: Path,
    output_video_path: Path,
    output_fps: float,
) -> None:
    ffmpeg_path = resolve_ffmpeg_executable()
    if ffmpeg_path is None:
        raise ValidationError(
            "Could not find 'ffmpeg' on PATH or next to the active Python executable. "
            f"Frames were written to {frames_dir}, but video encoding could not start."
        )

    output_video_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(ffmpeg_path),
        "-y",
        "-framerate",
        f"{output_fps:.6f}",
        "-i",
        str(frames_dir / "frame_%06d.png"),
        "-vf",
        "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_video_path),
    ]
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        stderr_tail = "\n".join(result.stderr.strip().splitlines()[-12:])
        raise ValidationError(
            "ffmpeg failed while encoding the exported frames.\n"
            f"ffmpeg: {ffmpeg_path}\n"
            f"frames_dir: {frames_dir}\n"
            f"output_video_path: {output_video_path}\n"
            f"stderr_tail:\n{stderr_tail}"
        )


def ensure_file_exists(path: Path, label: str) -> None:
    if not path.exists():
        raise ValidationError(f"{label} does not exist: {path}")
    if not path.is_file():
        raise ValidationError(f"{label} is not a file: {path}")


def ensure_directory_exists(path: Path, label: str) -> None:
    if not path.exists():
        raise ValidationError(f"{label} does not exist: {path}")
    if not path.is_dir():
        raise ValidationError(f"{label} is not a directory: {path}")


def discover_sequence_directories(dataset_root: Path) -> list[tuple[int, Path]]:
    ensure_directory_exists(dataset_root, "Dataset root")

    direct_match = SCAN_DIR_PATTERN.fullmatch(dataset_root.name)
    if direct_match:
        return [(int(direct_match.group(1)), dataset_root)]

    matches: list[tuple[int, Path]] = []
    for child in dataset_root.iterdir():
        if not child.is_dir():
            continue
        match = SCAN_DIR_PATTERN.fullmatch(child.name)
        if match:
            matches.append((int(match.group(1)), child))

    if not matches:
        raise ValidationError(
            f"No sequence folders matching 'print_scan_*' were found in {dataset_root}. "
            "Pass either the session root or a specific print_scan_* directory."
        )

    matches.sort(key=lambda item: item[0])
    return matches


def sanitize_yaml_text(text: str) -> str:
    cleaned_lines = []
    for line in text.splitlines():
        if line.startswith("%YAML:"):
            continue
        cleaned_lines.append(line.replace("!!opencv-matrix", ""))
    return "\n".join(cleaned_lines)


def load_structured_file(path: Path) -> Any:
    yaml = require_yaml()
    ensure_file_exists(path, "Structured input")
    raw_text = path.read_text(encoding="utf-8")
    if not raw_text.strip():
        raise ValidationError(f"Input file is empty: {path}")

    suffix = path.suffix.lower()
    try:
        if suffix == ".json":
            return json.loads(raw_text)
        return yaml.safe_load(sanitize_yaml_text(raw_text))
    except (json.JSONDecodeError, yaml.YAMLError) as exc:
        raise ValidationError(f"Failed to parse {path}: {exc}") from exc


def parse_float_vector(
    value: Any,
    expected_length: int,
    field_name: str,
    file_path: Path,
) -> Any:
    np = require_numpy()

    if isinstance(value, dict):
        axis_sets = {
            3: ("x", "y", "z"),
            4: ("x", "y", "z", "w"),
        }
        axes = axis_sets.get(expected_length)
        if axes and all(axis in value for axis in axes):
            value = [value[axis] for axis in axes]

    if not isinstance(value, (list, tuple)):
        raise ValidationError(
            f"Malformed {field_name} in {file_path}: expected a list of "
            f"{expected_length} numeric values."
        )

    if len(value) != expected_length:
        raise ValidationError(
            f"Malformed {field_name} in {file_path}: expected length "
            f"{expected_length}, got {len(value)}."
        )

    try:
        return np.asarray(value, dtype=float).reshape(expected_length)
    except (TypeError, ValueError) as exc:
        raise ValidationError(
            f"Malformed {field_name} in {file_path}: could not parse numeric values."
        ) from exc


def parse_flat_numeric_array(
    value: Any,
    field_name: str,
    file_path: Path,
) -> Any:
    np = require_numpy()

    if isinstance(value, dict) and {"rows", "cols", "data"}.issubset(value.keys()):
        value = value["data"]

    try:
        return np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValidationError(
            f"Malformed {field_name} in {file_path}: expected numeric array data."
        ) from exc


def parse_matrix(
    value: Any,
    expected_shape: tuple[int, int],
    field_name: str,
    file_path: Path,
) -> Any:
    np = require_numpy()

    if isinstance(value, dict) and {"rows", "cols", "data"}.issubset(value.keys()):
        rows = int(value["rows"])
        cols = int(value["cols"])
        data = value["data"]
        value = np.asarray(data, dtype=float).reshape(rows, cols)
    else:
        try:
            value = np.asarray(value, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValidationError(
                f"Malformed {field_name} in {file_path}: expected a numeric matrix."
            ) from exc

    if value.shape != expected_shape:
        raise ValidationError(
            f"Malformed {field_name} in {file_path}: expected shape "
            f"{expected_shape}, got {value.shape}."
        )

    return value


def quaternion_xyzw_to_rotation_matrix(quaternion_xyzw: Sequence[float]) -> list[list[float]]:
    if len(quaternion_xyzw) != 4:
        raise ValueError("Quaternion must contain exactly 4 values in [x, y, z, w] order.")

    x, y, z, w = [float(v) for v in quaternion_xyzw]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be non-zero.")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def compose_transform_from_position_quaternion(
    position: Sequence[float],
    quaternion_xyzw: Sequence[float],
    context: str,
) -> Any:
    np = require_numpy()
    try:
        rotation = quaternion_xyzw_to_rotation_matrix(quaternion_xyzw)
    except ValueError as exc:
        raise ValidationError(f"{context}: {exc}") from exc

    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(rotation, dtype=float)
    T[:3, 3] = np.asarray(position, dtype=float)
    return T


def invert_rigid_transform(T: Any) -> Any:
    np = require_numpy()
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=float)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -(R.T @ t)
    return T_inv


def compute_camera_pose(T_base_tool0: Any, T_extrinsic: Any, convention: str) -> Any:
    if convention == "T_tool0_camera":
        return T_base_tool0 @ T_extrinsic
    if convention == "T_camera_tool0":
        return T_base_tool0 @ invert_rigid_transform(T_extrinsic)
    raise ValueError(f"Unsupported extrinsic convention: {convention}")


def get_nested_key(data: Any, dotted_key: str, file_path: Path) -> Any:
    current = data
    traversed: list[str] = []
    for token in dotted_key.split("."):
        traversed.append(token)
        if not isinstance(current, dict) or token not in current:
            raise CalibrationParseError(
                f"Could not resolve extrinsic key '{dotted_key}' in {file_path}. "
                f"Missing path segment: {'.'.join(traversed)}"
            )
        current = current[token]
    return current


def is_transform_like(value: Any) -> bool:
    if isinstance(value, (list, tuple)):
        if len(value) == 4 and all(isinstance(row, (list, tuple)) for row in value):
            return True
        return len(value) == 16

    if not isinstance(value, dict):
        return False

    if {"rows", "cols", "data"}.issubset(value.keys()):
        return True

    matrix_keys = {"matrix", "transform", "T"}
    if matrix_keys.intersection(value.keys()):
        return True

    translation_keys = {"xyz", "position", "translation"}
    quaternion_keys = {"quat_xyzw", "orientation_xyzw", "quaternion_xyzw", "rotation_xyzw"}
    return bool(translation_keys.intersection(value.keys())) and bool(
        quaternion_keys.intersection(value.keys())
    )


def resolve_extrinsic_node(
    data: Any,
    file_path: Path,
    convention: str,
    extrinsic_key: str | None,
) -> tuple[Any, str]:
    if extrinsic_key:
        return get_nested_key(data, extrinsic_key, file_path), extrinsic_key

    if is_transform_like(data):
        return data, "<root>"

    if not isinstance(data, dict):
        raise CalibrationParseError(
            f"Could not parse extrinsics from {file_path}. Expected a transform-like "
            "mapping, 4x4 matrix, or a dotted --extrinsic-key."
        )

    exact_candidates: list[tuple[str, Any]] = []
    if convention in data and is_transform_like(data[convention]):
        exact_candidates.append((convention, data[convention]))
    if isinstance(data.get("frames"), dict) and convention in data["frames"]:
        candidate = data["frames"][convention]
        if is_transform_like(candidate):
            exact_candidates.append((f"frames.{convention}", candidate))

    if len(exact_candidates) == 1:
        return exact_candidates[0][1], exact_candidates[0][0]
    if len(exact_candidates) > 1:
        raise CalibrationParseError(
            f"Extrinsics file {file_path} contains multiple candidates for "
            f"{convention}: {[name for name, _ in exact_candidates]}. "
            "Pass --extrinsic-key to disambiguate."
        )

    candidate_sets: list[tuple[str, Any]] = []
    for key, value in data.items():
        if is_transform_like(value):
            candidate_sets.append((key, value))

    frames = data.get("frames")
    if isinstance(frames, dict):
        for key, value in frames.items():
            if is_transform_like(value):
                candidate_sets.append((f"frames.{key}", value))

    unique_names = {name for name, _ in candidate_sets}
    if len(unique_names) == 1 and candidate_sets:
        name, value = candidate_sets[0]
        return value, name

    raise CalibrationParseError(
        f"Could not resolve a unique extrinsic transform from {file_path}. "
        "Expected one transform-like entry or an explicit --extrinsic-key."
    )


def parse_transform_node(node: Any, file_path: Path, parameter_name: str) -> Any:
    if isinstance(node, dict):
        for nested_key in ("matrix", "transform", "T"):
            if nested_key in node:
                return parse_transform_node(
                    node[nested_key],
                    file_path=file_path,
                    parameter_name=f"{parameter_name}.{nested_key}",
                )

        translation_value = None
        for key in ("xyz", "position", "translation"):
            if key in node:
                translation_value = node[key]
                break

        quaternion_value = None
        for key in ("quat_xyzw", "orientation_xyzw", "quaternion_xyzw", "rotation_xyzw"):
            if key in node:
                quaternion_value = node[key]
                break

        if translation_value is None and quaternion_value is not None:
            raise CalibrationParseError(
                f"Failed to parse transform '{parameter_name}' in {file_path}. "
                "Missing translation field. Expected one of: xyz, position, translation."
            )

        if translation_value is not None and quaternion_value is None:
            raise CalibrationParseError(
                f"Failed to parse transform '{parameter_name}' in {file_path}. "
                "Missing quaternion field. Expected one of: quat_xyzw, orientation_xyzw, "
                "quaternion_xyzw, rotation_xyzw."
            )

        if translation_value is not None and quaternion_value is not None:
            position = parse_float_vector(
                translation_value,
                expected_length=3,
                field_name=f"{parameter_name}.position",
                file_path=file_path,
            )
            quaternion = parse_float_vector(
                quaternion_value,
                expected_length=4,
                field_name=f"{parameter_name}.orientation_xyzw",
                file_path=file_path,
            )
            return compose_transform_from_position_quaternion(
                position,
                quaternion,
                context=f"{file_path}:{parameter_name}",
            )

    try:
        return parse_matrix(
            node,
            expected_shape=(4, 4),
            field_name=parameter_name,
            file_path=file_path,
        )
    except ValidationError as exc:
        raise CalibrationParseError(
            f"Failed to parse transform '{parameter_name}' in {file_path}. "
            "Expected either a 4x4 matrix or translation plus quaternion in "
            "[x, y, z, w] order."
        ) from exc


def load_extrinsic_transform(
    extrinsics_path: Path,
    convention: str,
    extrinsic_key: str | None,
) -> tuple[Any, str]:
    data = load_structured_file(extrinsics_path)
    node, resolved_key = resolve_extrinsic_node(
        data=data,
        file_path=extrinsics_path,
        convention=convention,
        extrinsic_key=extrinsic_key,
    )
    T_extrinsic = parse_transform_node(
        node=node,
        file_path=extrinsics_path,
        parameter_name=resolved_key,
    )
    return T_extrinsic, resolved_key


def load_intrinsics(intrinsics_path: Path) -> IntrinsicsData:
    data = load_structured_file(intrinsics_path)
    if not isinstance(data, dict):
        raise CalibrationParseError(
            f"Failed to parse intrinsics from {intrinsics_path}. Expected a top-level mapping "
            "with image dimensions and camera_matrix."
        )

    width = data.get("image_width", data.get("width"))
    height = data.get("image_height", data.get("height"))
    if width is None or height is None:
        raise CalibrationParseError(
            f"Failed to parse intrinsics from {intrinsics_path}. Missing 'image_width'/'image_height' "
            "or 'width'/'height'."
        )

    camera_matrix_node = None
    for key in ("camera_matrix", "K", "intrinsic_matrix"):
        if key in data:
            camera_matrix_node = data[key]
            break
    if camera_matrix_node is None:
        raise CalibrationParseError(
            f"Failed to parse intrinsics from {intrinsics_path}. Missing required "
            "'camera_matrix' or equivalent 3x3 matrix field."
        )

    distortion_node = None
    for key in ("distortion_coefficients", "distortion", "D"):
        if key in data:
            distortion_node = data[key]
            break
    if distortion_node is None:
        raise CalibrationParseError(
            f"Failed to parse intrinsics from {intrinsics_path}. Missing "
            "'distortion_coefficients' or equivalent distortion field."
        )

    try:
        width_int = int(width)
        height_int = int(height)
    except (TypeError, ValueError) as exc:
        raise CalibrationParseError(
            f"Failed to parse intrinsics from {intrinsics_path}. Image dimensions must be integers."
        ) from exc

    camera_matrix = parse_matrix(
        camera_matrix_node,
        expected_shape=(3, 3),
        field_name="camera_matrix",
        file_path=intrinsics_path,
    )

    distortion = parse_flat_numeric_array(
        distortion_node,
        field_name="distortion_coefficients",
        file_path=intrinsics_path,
    )

    return IntrinsicsData(
        path=intrinsics_path,
        width=width_int,
        height=height_int,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion,
    )


def load_manifest_captures(manifest_path: Path, expected_captures: int) -> tuple[list[dict[str, Any]], list[str]]:
    try:
        manifest = load_structured_file(manifest_path)
    except ValidationError as exc:
        raise ManifestValidationError(
            f"Failed to parse manifest {manifest_path}: {exc}. "
            "Top-level keys are unavailable because the file could not be parsed."
        ) from exc

    if not isinstance(manifest, dict):
        raise ManifestValidationError(
            f"Manifest {manifest_path} must contain a top-level mapping. Parsed type: {type(manifest).__name__}"
        )

    manifest_keys = sorted(str(key) for key in manifest.keys())
    captures = manifest.get("captures")
    if not isinstance(captures, list):
        raise ManifestValidationError(
            f"Manifest {manifest_path} is missing a top-level 'captures' list. "
            f"Top-level keys: {manifest_keys}"
        )

    if len(captures) != expected_captures:
        raise ManifestValidationError(
            f"Manifest {manifest_path} has {len(captures)} captures, expected {expected_captures}. "
            f"Top-level keys: {manifest_keys}"
        )

    return captures, manifest_keys


def parse_capture_pose(capture: dict[str, Any], manifest_path: Path, list_index: int) -> tuple[int, Any, Any]:
    if not isinstance(capture, dict):
        raise ManifestValidationError(
            f"Manifest {manifest_path} capture entry {list_index} is not a mapping."
        )

    if "index" not in capture:
        raise ManifestValidationError(
            f"Manifest {manifest_path} capture entry {list_index} is missing 'index'."
        )

    try:
        capture_index = int(capture["index"])
    except (TypeError, ValueError) as exc:
        raise ManifestValidationError(
            f"Manifest {manifest_path} capture entry {list_index} has a non-integer index: {capture['index']}"
        ) from exc

    pose = capture.get("T_base_tool0")
    if not isinstance(pose, dict):
        raise ManifestValidationError(
            f"Manifest {manifest_path} capture index {capture_index} is missing 'T_base_tool0'."
        )

    try:
        position = parse_float_vector(
            pose.get("position"),
            expected_length=3,
            field_name=f"capture[{capture_index}].T_base_tool0.position",
            file_path=manifest_path,
        )
        quaternion = parse_float_vector(
            pose.get("orientation_xyzw"),
            expected_length=4,
            field_name=f"capture[{capture_index}].T_base_tool0.orientation_xyzw",
            file_path=manifest_path,
        )
        T_base_tool0 = compose_transform_from_position_quaternion(
            position,
            quaternion,
            context=f"{manifest_path}:capture[{capture_index}].T_base_tool0",
        )
    except ValidationError as exc:
        raise ManifestValidationError(
            f"Failed to extract pose for capture index {capture_index} in {manifest_path}: {exc}"
        ) from exc

    return capture_index, position, T_base_tool0


def locate_reconstructed_ply(sequence_path: Path) -> tuple[Path, Path]:
    reconstruct_path = sequence_path / "reconstruct"
    ensure_directory_exists(reconstruct_path, f"Reconstruct directory for {sequence_path.name}")

    ply_files = sorted(
        path for path in reconstruct_path.iterdir() if path.is_file() and path.suffix.lower() == ".ply"
    )
    if len(ply_files) != 1:
        found = [path.name for path in ply_files]
        raise ValidationError(
            f"Expected exactly one .ply inside {reconstruct_path}, found {len(ply_files)}: {found}"
        )
    return reconstruct_path, ply_files[0]


def load_point_cloud(ply_path: Path) -> tuple[Any, int, Any, Any, Any, bool]:
    o3d = require_open3d()
    np = require_numpy()

    point_cloud = o3d.io.read_point_cloud(str(ply_path))
    points = np.asarray(point_cloud.points)
    point_count = int(points.shape[0])
    has_colors = bool(point_cloud.has_colors()) and len(np.asarray(point_cloud.colors)) == point_count

    if point_count == 0:
        LOGGER.warning("Point cloud is empty: %s", ply_path)
        bbox_min = np.zeros(3, dtype=float)
        bbox_max = np.zeros(3, dtype=float)
        bbox_extent = np.zeros(3, dtype=float)
    else:
        aabb = point_cloud.get_axis_aligned_bounding_box()
        bbox_min = np.asarray(aabb.get_min_bound(), dtype=float)
        bbox_max = np.asarray(aabb.get_max_bound(), dtype=float)
        bbox_extent = bbox_max - bbox_min

    return point_cloud, point_count, bbox_min, bbox_max, bbox_extent, has_colors


def load_sequence(
    scan_index: int,
    sequence_path: Path,
    expected_captures: int,
    T_extrinsic: Any,
    convention: str,
) -> SequenceData:
    np = require_numpy()

    manifest_path = sequence_path / "manifest.yaml"
    ensure_file_exists(manifest_path, f"Manifest for {sequence_path.name}")
    reconstruct_path, ply_path = locate_reconstructed_ply(sequence_path)
    captures_raw, manifest_keys = load_manifest_captures(manifest_path, expected_captures)

    parsed_captures: list[tuple[int, dict[str, Any], Any]] = []
    seen_indices: set[int] = set()
    for list_index, capture in enumerate(captures_raw):
        capture_index, _, T_base_tool0 = parse_capture_pose(capture, manifest_path, list_index)
        if capture_index in seen_indices:
            raise ManifestValidationError(
                f"Manifest {manifest_path} contains duplicate capture index {capture_index}."
            )
        seen_indices.add(capture_index)
        parsed_captures.append((capture_index, capture, T_base_tool0))

    parsed_captures.sort(key=lambda item: item[0])
    expected_indices = list(range(expected_captures))
    actual_indices = [item[0] for item in parsed_captures]
    if actual_indices != expected_indices:
        raise ManifestValidationError(
            f"Manifest {manifest_path} capture indices are {actual_indices}, expected {expected_indices}."
        )

    captures: list[CapturePose] = []
    for capture_index, capture, T_base_tool0 in parsed_captures:
        T_base_camera = compute_camera_pose(T_base_tool0, T_extrinsic, convention)
        camera_position = np.asarray(T_base_camera[:3, 3], dtype=float)
        captures.append(
            CapturePose(
                capture_index=capture_index,
                timestamp=capture.get("timestamp"),
                color_path=capture.get("color"),
                depth_path=capture.get("depth"),
                ir_path=capture.get("ir"),
                T_base_tool0=T_base_tool0,
                T_base_camera=T_base_camera,
                camera_position=camera_position,
            )
        )

    point_cloud, point_count, bbox_min, bbox_max, bbox_extent, has_colors = load_point_cloud(ply_path)
    camera_positions = np.vstack([capture.camera_position for capture in captures])

    if point_count > 0:
        sequence_centroid = (bbox_min + bbox_max) * 0.5
    else:
        sequence_centroid = camera_positions.mean(axis=0)

    return SequenceData(
        scan_index=scan_index,
        sequence_path=sequence_path,
        manifest_path=manifest_path,
        reconstruct_path=reconstruct_path,
        ply_path=ply_path,
        manifest_keys=manifest_keys,
        captures=captures,
        point_cloud=point_cloud,
        point_count=point_count,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        bbox_extent=bbox_extent,
        camera_positions=camera_positions,
        sequence_centroid=sequence_centroid,
        has_colors=has_colors,
    )


def format_vector(vector: Any, precision: int = 4) -> str:
    np = require_numpy()
    arr = np.asarray(vector, dtype=float).reshape(-1)
    return "[" + ", ".join(f"{value:.{precision}f}" for value in arr) + "]"


def summarize_intrinsics(intrinsics: IntrinsicsData | None) -> None:
    if intrinsics is None:
        LOGGER.info("Intrinsics: not provided for Stage 1.")
        return

    LOGGER.info(
        "Intrinsics loaded from %s | size=%dx%d | fx=%.4f fy=%.4f cx=%.4f cy=%.4f | distortion_len=%d",
        intrinsics.path,
        intrinsics.width,
        intrinsics.height,
        intrinsics.camera_matrix[0, 0],
        intrinsics.camera_matrix[1, 1],
        intrinsics.camera_matrix[0, 2],
        intrinsics.camera_matrix[1, 2],
        len(intrinsics.distortion_coefficients),
    )


def print_sequence_summary(sequence: SequenceData) -> None:
    np = require_numpy()

    camera_mean = sequence.camera_positions.mean(axis=0)
    camera_min = sequence.camera_positions.min(axis=0)
    camera_max = sequence.camera_positions.max(axis=0)
    camera_first = sequence.captures[0].camera_position
    camera_last = sequence.captures[-1].camera_position

    LOGGER.info(
        "Scan %03d | points=%d | captures=%d | has_colors=%s | bbox_min=%s | bbox_max=%s | bbox_extent=%s",
        sequence.scan_index,
        sequence.point_count,
        len(sequence.captures),
        sequence.has_colors,
        format_vector(sequence.bbox_min),
        format_vector(sequence.bbox_max),
        format_vector(sequence.bbox_extent),
    )
    LOGGER.info(
        "Scan %03d camera summary | first=%s | last=%s | mean=%s | min=%s | max=%s",
        sequence.scan_index,
        format_vector(camera_first),
        format_vector(camera_last),
        format_vector(camera_mean),
        format_vector(camera_min),
        format_vector(camera_max),
    )
    LOGGER.debug(
        "Scan %03d manifest keys: %s",
        sequence.scan_index,
        sequence.manifest_keys,
    )
    LOGGER.debug(
        "Scan %03d capture indices: %s",
        sequence.scan_index,
        [capture.capture_index for capture in sequence.captures],
    )


def compute_scene_extent(sequences: Sequence[SequenceData]) -> float:
    np = require_numpy()
    all_points: list[Any] = []

    for sequence in sequences:
        if sequence.point_count > 0:
            all_points.append(sequence.bbox_min)
            all_points.append(sequence.bbox_max)
        all_points.append(sequence.camera_positions.min(axis=0))
        all_points.append(sequence.camera_positions.max(axis=0))

    if not all_points:
        return 1.0

    bounds = np.vstack(all_points)
    extent = np.max(bounds.max(axis=0) - bounds.min(axis=0))
    return float(extent) if extent > 0.0 else 1.0


def sequence_color(order_index: int, count: int) -> tuple[float, float, float]:
    if count <= 0:
        return 0.8, 0.8, 0.8
    hue = (order_index / max(count, 1)) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.65, 0.95)
    return r, g, b


def build_line_set(points: Any, lines: list[list[int]], color: Sequence[float]) -> Any | None:
    o3d = require_open3d()
    np = require_numpy()

    if len(lines) == 0:
        return None

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=int))
    line_colors = np.tile(np.asarray(color, dtype=float), (len(lines), 1))
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    return line_set


def create_marker_sphere(position: Any, radius: float, color: Sequence[float]) -> Any:
    o3d = require_open3d()
    np = require_numpy()

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=12)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(np.asarray(color, dtype=float))
    sphere.translate(np.asarray(position, dtype=float))
    return sphere


def create_camera_frustum(
    T_base_camera: Any,
    intrinsics: IntrinsicsData,
    depth: float,
    plane_scale: float,
    color: Sequence[float],
) -> Any:
    o3d = require_open3d()
    np = require_numpy()

    fx = float(intrinsics.camera_matrix[0, 0])
    fy = float(intrinsics.camera_matrix[1, 1])
    cx = float(intrinsics.camera_matrix[0, 2])
    cy = float(intrinsics.camera_matrix[1, 2])
    width = float(intrinsics.width - 1)
    height = float(intrinsics.height - 1)

    if fx <= 0.0 or fy <= 0.0:
        raise ValidationError(
            f"Invalid intrinsics for frustum visualization in {intrinsics.path}: "
            f"fx={fx}, fy={fy}"
        )

    def pixel_to_camera(u: float, v: float) -> Any:
        return np.asarray(
            [
                plane_scale * (u - cx) * depth / fx,
                plane_scale * (v - cy) * depth / fy,
                depth,
            ],
            dtype=float,
        )

    corners_camera = [
        pixel_to_camera(0.0, 0.0),
        pixel_to_camera(width, 0.0),
        pixel_to_camera(width, height),
        pixel_to_camera(0.0, height),
    ]
    center_camera = pixel_to_camera(cx, cy)
    camera_points = np.vstack([np.zeros(3, dtype=float), *corners_camera, center_camera])

    R_base_camera = np.asarray(T_base_camera[:3, :3], dtype=float)
    t_base_camera = np.asarray(T_base_camera[:3, 3], dtype=float)
    world_points = (R_base_camera @ camera_points.T).T + t_base_camera

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(world_points)
    line_set.lines = o3d.utility.Vector2iVector(
        np.asarray(
            [
                [0, 1],
                [0, 2],
                [0, 3],
                [0, 4],
                [1, 2],
                [2, 3],
                [3, 4],
                [4, 1],
                [0, 5],
            ],
            dtype=int,
        )
    )
    line_colors = np.tile(np.asarray(color, dtype=float), (9, 1))
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    return line_set


def create_visual_point_cloud(
    sequence: SequenceData,
    fallback_color: Sequence[float],
    use_point_cloud_colors: bool,
) -> Any:
    o3d = require_open3d()
    np = require_numpy()

    vis_cloud = o3d.geometry.PointCloud()
    vis_cloud.points = o3d.utility.Vector3dVector(
        np.asarray(sequence.point_cloud.points, dtype=float)
    )

    if use_point_cloud_colors and sequence.has_colors:
        vis_cloud.colors = o3d.utility.Vector3dVector(
            np.asarray(sequence.point_cloud.colors, dtype=float)
        )
    else:
        vis_cloud.paint_uniform_color(np.asarray(fallback_color, dtype=float))

    return vis_cloud


def assign_points_to_nearest_cameras(sequence: SequenceData) -> tuple[Any, Any]:
    np = require_numpy()

    points = np.asarray(sequence.point_cloud.points, dtype=float)
    camera_positions = np.asarray(sequence.camera_positions, dtype=float)
    if points.shape[0] == 0:
        assignments = np.zeros((0,), dtype=int)
        counts = np.zeros((len(sequence.captures),), dtype=int)
        return assignments, counts

    distances_sq = np.sum(
        (points[:, None, :] - camera_positions[None, :, :]) ** 2,
        axis=2,
    )
    assignments = np.argmin(distances_sq, axis=1).astype(int)
    counts = np.bincount(assignments, minlength=len(sequence.captures))

    if assignments.shape[0] != points.shape[0]:
        raise ValidationError(
            f"Nearest-pose assignment failed for {sequence.sequence_path.name}: "
            f"{assignments.shape[0]} assignments for {points.shape[0]} points."
        )

    return assignments, counts


def normalize_capture_index_range(
    capture_count: int,
    start_index: int,
    end_index: int | None,
    label: str,
) -> tuple[int, int]:
    if capture_count <= 0:
        raise ValidationError(f"{label} has no captures available.")
    if start_index < 0 or start_index >= capture_count:
        raise ValidationError(
            f"Capture start index must be in [0, {capture_count - 1}] for {label}, "
            f"got {start_index}."
        )

    resolved_end_index = capture_count - 1 if end_index is None else end_index
    if resolved_end_index < 0 or resolved_end_index >= capture_count:
        raise ValidationError(
            f"Capture end index must be in [0, {capture_count - 1}] for {label}, "
            f"got {resolved_end_index}."
        )
    if resolved_end_index < start_index:
        raise ValidationError(
            f"Capture end index must be greater than or equal to start index for {label}: "
            f"start={start_index}, end={resolved_end_index}."
        )

    return int(start_index), int(resolved_end_index)


def compute_balanced_group_counts(point_count: int, capture_count: int) -> Any:
    np = require_numpy()

    if point_count < 0:
        raise ValidationError("Point count for preview grouping must be non-negative.")
    if capture_count <= 0:
        raise ValidationError("Capture count for preview grouping must be positive.")

    counts = np.full((capture_count,), point_count // capture_count, dtype=int)
    counts[: point_count % capture_count] += 1
    return counts


def randomize_assignments_from_counts(target_counts: Any, rng: Any) -> Any:
    np = require_numpy()

    target_counts = np.asarray(target_counts, dtype=int)
    if target_counts.ndim != 1:
        raise ValidationError("Preview grouping counts must be a 1D vector.")
    if np.any(target_counts < 0):
        raise ValidationError("Preview grouping counts must be non-negative.")

    assignments = np.repeat(np.arange(target_counts.shape[0], dtype=int), target_counts)
    rng.shuffle(assignments)
    return assignments


def assign_points_to_preview_groups(
    sequence: SequenceData,
    grouping_mode: str,
    random_seed: int,
) -> tuple[Any, Any]:
    np = require_numpy()

    if grouping_mode == "nearest_pose":
        return assign_points_to_nearest_cameras(sequence)

    base_assignments, base_counts = assign_points_to_nearest_cameras(sequence)
    point_count = int(base_assignments.shape[0])
    capture_count = len(sequence.captures)
    rng = np.random.default_rng(random_seed)

    if grouping_mode == "randomized_nearest_counts":
        target_counts = np.asarray(base_counts, dtype=int)
    elif grouping_mode == "randomized_balanced":
        target_counts = compute_balanced_group_counts(point_count, capture_count)
    else:
        raise ValidationError(
            "Unsupported preview grouping mode: "
            f"{grouping_mode}. Expected one of "
            "'nearest_pose', 'randomized_nearest_counts', 'randomized_balanced'."
        )

    assignments = randomize_assignments_from_counts(target_counts=target_counts, rng=rng)
    counts = np.bincount(assignments, minlength=capture_count)
    if assignments.shape[0] != point_count:
        raise ValidationError(
            f"Preview grouping produced {assignments.shape[0]} assignments for "
            f"{point_count} points in {sequence.sequence_path.name}."
        )
    if int(counts.sum()) != point_count:
        raise ValidationError(
            f"Preview grouping count mismatch for {sequence.sequence_path.name}: "
            f"{int(counts.sum())} assigned vs {point_count} expected."
        )
    return assignments.astype(int), counts.astype(int)


def easing_value(t: float, easing: str) -> float:
    t = max(0.0, min(1.0, float(t)))
    if easing == "linear":
        return t
    if easing == "ease_out_cubic":
        return 1.0 - (1.0 - t) ** 3
    if easing == "ease_in_out_cubic":
        if t < 0.5:
            return 4.0 * t * t * t
        return 1.0 - ((-2.0 * t + 2.0) ** 3) / 2.0
    raise ValidationError(f"Unsupported easing mode: {easing}")


def compute_projected_start_positions(
    points_world: Any,
    T_base_camera: Any,
    intrinsics: IntrinsicsData,
    frustum_depth: float,
    frustum_plane_scale: float,
    frustum_point_scale: float,
) -> Any:
    np = require_numpy()

    points_world = np.asarray(points_world, dtype=float)
    R_base_camera = np.asarray(T_base_camera[:3, :3], dtype=float)
    t_base_camera = np.asarray(T_base_camera[:3, 3], dtype=float)
    points_camera = (R_base_camera.T @ (points_world - t_base_camera).T).T

    fx = float(intrinsics.camera_matrix[0, 0])
    fy = float(intrinsics.camera_matrix[1, 1])
    cx = float(intrinsics.camera_matrix[0, 2])
    cy = float(intrinsics.camera_matrix[1, 2])
    width = float(intrinsics.width - 1)
    height = float(intrinsics.height - 1)

    z = points_camera[:, 2]
    valid = z > 1e-6

    u = np.full(points_camera.shape[0], cx, dtype=float)
    v = np.full(points_camera.shape[0], cy, dtype=float)
    u[valid] = fx * points_camera[valid, 0] / z[valid] + cx
    v[valid] = fy * points_camera[valid, 1] / z[valid] + cy
    u = np.clip(u, 0.0, width)
    v = np.clip(v, 0.0, height)

    start_camera = np.column_stack(
        [
            frustum_point_scale * frustum_plane_scale * (u - cx) * frustum_depth / fx,
            frustum_point_scale * frustum_plane_scale * (v - cy) * frustum_depth / fy,
            np.full(points_camera.shape[0], frustum_depth, dtype=float),
        ]
    )
    return (R_base_camera @ start_camera.T).T + t_base_camera


def apply_frustum_plane_noise(
    start_points_world: Any,
    T_base_camera: Any,
    noise_scale_world: float,
    rng: Any,
) -> Any:
    np = require_numpy()

    if noise_scale_world <= 0.0:
        return np.asarray(start_points_world, dtype=float)

    start_points_world = np.asarray(start_points_world, dtype=float)
    camera_right = np.asarray(T_base_camera[:3, 0], dtype=float)
    camera_up = np.asarray(T_base_camera[:3, 1], dtype=float)

    plane_noise = rng.normal(
        loc=0.0,
        scale=noise_scale_world,
        size=(start_points_world.shape[0], 2),
    )
    return (
        start_points_world
        + plane_noise[:, [0]] * camera_right[None, :]
        + plane_noise[:, [1]] * camera_up[None, :]
    )


def sample_point_launch_delays(
    point_count: int,
    delay_fraction: float,
    rng: Any,
) -> Any:
    np = require_numpy()

    if delay_fraction <= 0.0:
        return np.zeros((point_count,), dtype=float)
    return rng.uniform(0.0, delay_fraction, size=point_count)


def resolve_preview_capture_indices(
    counts: Any,
    sequence_name: str,
    grouping_mode: str,
    allowed_start_index: int,
    allowed_end_index: int,
) -> list[int]:
    np = require_numpy()

    counts = np.asarray(counts, dtype=int)
    preview_capture_indices = [
        int(capture_index)
        for capture_index in range(allowed_start_index, allowed_end_index + 1)
        if counts[capture_index] > 0
    ]
    skipped_capture_indices = [
        int(capture_index)
        for capture_index in range(allowed_start_index, allowed_end_index + 1)
        if counts[capture_index] <= 0
    ]

    if skipped_capture_indices:
        LOGGER.info(
            "Skipping empty preview groups for %s under grouping mode '%s': %s",
            sequence_name,
            grouping_mode,
            skipped_capture_indices,
        )

    if not preview_capture_indices:
        raise ValidationError(
            f"No non-empty preview groups exist within frustum range "
            f"[{allowed_start_index}, {allowed_end_index}] in {sequence_name}."
        )

    return preview_capture_indices


def order_preview_capture_indices(
    preview_capture_indices: Sequence[int],
    order_mode: str,
    random_seed: int,
) -> list[int]:
    np = require_numpy()

    ordered_indices = [int(capture_index) for capture_index in preview_capture_indices]
    if order_mode == "index_order":
        return ordered_indices
    if order_mode == "randomized_order":
        rng = np.random.default_rng(random_seed)
        rng.shuffle(ordered_indices)
        return ordered_indices

    raise ValidationError(
        f"Unsupported preview capture order mode: {order_mode}. "
        "Expected 'index_order' or 'randomized_order'."
    )


def prepare_preview_group_data(
    sequence: SequenceData,
    assignments: Any,
    capture_index: int,
    intrinsics: IntrinsicsData,
    camera_frustum_depth: float,
    camera_frustum_plane_scale: float,
    preview_projection_start_scale: float,
    preview_point_limit: int | None,
    preview_start_noise_scale: float,
    camera_frustum_color: Sequence[float],
    scene_extent: float,
    rng: Any,
    preview_delay_fraction: float,
) -> PreviewGroupData:
    np = require_numpy()

    group_mask = assignments == capture_index
    if not np.any(group_mask):
        raise ValidationError(
            f"Preview group for capture {capture_index} is empty in {sequence.sequence_path.name}."
        )

    final_points = np.asarray(sequence.point_cloud.points, dtype=float)[group_mask]
    if sequence.has_colors:
        colors = np.asarray(sequence.point_cloud.colors, dtype=float)[group_mask]
    else:
        colors = np.tile(
            np.asarray(camera_frustum_color, dtype=float),
            (final_points.shape[0], 1),
        )

    if preview_point_limit is not None and final_points.shape[0] > preview_point_limit:
        sample_indices = np.linspace(
            0,
            final_points.shape[0] - 1,
            num=preview_point_limit,
            dtype=int,
        )
        final_points = final_points[sample_indices]
        colors = colors[sample_indices]

    capture = sequence.captures[capture_index]
    start_points = compute_projected_start_positions(
        points_world=final_points,
        T_base_camera=capture.T_base_camera,
        intrinsics=intrinsics,
        frustum_depth=camera_frustum_depth,
        frustum_plane_scale=camera_frustum_plane_scale,
        frustum_point_scale=preview_projection_start_scale,
    )
    start_points = apply_frustum_plane_noise(
        start_points_world=start_points,
        T_base_camera=capture.T_base_camera,
        noise_scale_world=scene_extent * preview_start_noise_scale,
        rng=rng,
    )
    launch_delays = sample_point_launch_delays(
        point_count=final_points.shape[0],
        delay_fraction=preview_delay_fraction,
        rng=rng,
    )

    return PreviewGroupData(
        capture_index=capture_index,
        capture=capture,
        final_points=final_points,
        start_points=start_points,
        colors=colors,
        launch_delays=launch_delays,
    )


def evaluate_group_frame_points(
    group: PreviewGroupData,
    group_frame_index: int,
    group_frame_count: int,
    easing: str,
    hide_before_start: bool,
) -> tuple[Any, Any]:
    np = require_numpy()

    if group_frame_index < 0:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)

    if group_frame_index >= group_frame_count:
        return group.final_points, group.colors

    if group_frame_count <= 1:
        return group.final_points, group.colors

    t = group_frame_index / float(group_frame_count - 1)
    local_t = np.clip(
        (t - group.launch_delays) / np.maximum(1.0 - group.launch_delays, 1e-6),
        0.0,
        1.0,
    )
    eased_t = np.asarray([easing_value(value, easing) for value in local_t], dtype=float)
    current_points = group.start_points + eased_t[:, None] * (group.final_points - group.start_points)

    if hide_before_start:
        visible_mask = local_t > 0.0
        if not np.any(visible_mask):
            return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)
        return current_points[visible_mask], group.colors[visible_mask]

    return current_points, group.colors


def compute_default_side_offset_direction(T_base_camera: Any) -> Any:
    np = require_numpy()

    R_base_camera = np.asarray(T_base_camera[:3, :3], dtype=float)
    camera_right = R_base_camera[:, 0]
    world_up = np.asarray([0.0, 0.0, 1.0], dtype=float)

    side_offset = camera_right + 0.25 * world_up
    side_norm = np.linalg.norm(side_offset)
    if side_norm <= 1e-9:
        side_offset = np.asarray([1.0, 0.0, 0.2], dtype=float)
        side_norm = np.linalg.norm(side_offset)
    return side_offset / side_norm


def direction_to_yaw_elevation(direction: Any) -> tuple[float, float]:
    np = require_numpy()

    direction = np.asarray(direction, dtype=float)
    direction_norm = np.linalg.norm(direction)
    if direction_norm <= 1e-9:
        raise ValidationError("Cannot compute filming camera angles from a zero-length direction.")
    direction = direction / direction_norm

    yaw = math.atan2(direction[1], direction[0])
    elevation = math.atan2(direction[2], np.linalg.norm(direction[:2]))
    return float(yaw), float(elevation)


def yaw_elevation_to_direction(yaw: float, elevation: float) -> Any:
    np = require_numpy()

    cos_elevation = math.cos(elevation)
    return np.asarray(
        [
            cos_elevation * math.cos(yaw),
            cos_elevation * math.sin(yaw),
            math.sin(elevation),
        ],
        dtype=float,
    )


def interpolate_scalar(start_value: float, end_value: float, t: float) -> float:
    t = max(0.0, min(1.0, float(t)))
    return (1.0 - t) * float(start_value) + t * float(end_value)


def compute_preview_target_point(
    preview_groups: Sequence[PreviewGroupData],
    visible_point_sets: Sequence[Any],
    target_mode: str,
) -> Any:
    np = require_numpy()

    all_final_points = np.vstack([group.final_points for group in preview_groups])
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

    raise ValidationError(
        f"Unsupported filming camera target mode: {target_mode}. "
        "Expected 'final_centroid' or 'visible_centroid'."
    )


def set_preview_filming_camera(
    vis: Any,
    base_offset_direction: Any,
    lookat: Any,
    scene_extent: float,
    film_camera_mode: str,
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

    base_yaw, base_elevation = direction_to_yaw_elevation(base_offset_direction)
    current_yaw_start_deg = interpolate_scalar(
        film_yaw_start_deg,
        film_yaw_start_deg_final,
        motion_t,
    )
    current_yaw_end_deg = interpolate_scalar(
        film_yaw_end_deg,
        film_yaw_end_deg_final,
        motion_t,
    )
    current_vertical_offset_deg = interpolate_scalar(
        film_vertical_offset_deg,
        film_vertical_offset_deg_final,
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

    if film_camera_mode == "static_side":
        yaw_offset_deg = current_yaw_start_deg
    elif film_camera_mode == "orbit_side":
        yaw_offset_deg = current_yaw_start_deg + motion_t * (
            current_yaw_end_deg - current_yaw_start_deg
        )
    else:
        raise ValidationError(
            f"Unsupported filming camera mode: {film_camera_mode}. "
            "Expected 'static_side' or 'orbit_side'."
        )

    yaw = base_yaw + math.radians(yaw_offset_deg)
    elevation = base_elevation + math.radians(current_vertical_offset_deg)
    elevation = max(math.radians(-80.0), min(math.radians(80.0), elevation))
    offset_direction = yaw_elevation_to_direction(yaw, elevation)

    front = -offset_direction
    world_up = np.asarray([0.0, 0.0, 1.0], dtype=float)
    up = world_up - np.dot(world_up, front) * front
    up_norm = np.linalg.norm(up)
    if up_norm <= 1e-9:
        up = np.asarray([0.0, 1.0, 0.0], dtype=float)
        up_norm = np.linalg.norm(up)
    up /= max(up_norm, 1e-9)

    view_control = vis.get_view_control()
    adjusted_lookat = np.asarray(lookat, dtype=float) + np.asarray(
        [0.0, 0.0, scene_extent * current_frame_vertical_offset_scale],
        dtype=float,
    )
    view_control.set_lookat(adjusted_lookat)
    view_control.set_front(front)
    view_control.set_up(up)
    view_control.set_zoom(float(current_zoom))


def blend_rgb(
    source_color: Sequence[float],
    target_color: Sequence[float],
    blend_t: float,
) -> Any:
    np = require_numpy()

    blend_t = max(0.0, min(1.0, float(blend_t)))
    source = np.asarray(source_color, dtype=float)
    target = np.asarray(target_color, dtype=float)
    return (1.0 - blend_t) * source + blend_t * target


def set_line_set_uniform_color(line_set: Any, color: Sequence[float]) -> None:
    o3d = require_open3d()
    np = require_numpy()

    line_count = len(line_set.lines)
    line_colors = np.tile(np.asarray(color, dtype=float), (line_count, 1))
    line_set.colors = o3d.utility.Vector3dVector(line_colors)


def set_mesh_uniform_color(mesh: Any, color: Sequence[float]) -> None:
    np = require_numpy()

    mesh.paint_uniform_color(np.asarray(color, dtype=float))


def compute_preview_frustum_intensity(
    group_frame_index: int,
    group_frame_count: int,
    fade_frame_count: int,
) -> float:
    if group_frame_index < 0:
        return 0.0
    if group_frame_index < group_frame_count:
        return 1.0
    if fade_frame_count <= 0:
        return 0.0

    fade_progress = (group_frame_index - group_frame_count + 1) / float(fade_frame_count)
    if fade_progress >= 1.0:
        return 0.0
    return 1.0 - easing_value(fade_progress, "ease_out_cubic")


def build_visualization_geometries(
    sequences: Sequence[SequenceData],
    intrinsics: IntrinsicsData | None,
    camera_marker_radius: float,
    camera_frustum_depth: float,
    camera_frustum_plane_scale: float,
    camera_frustum_color: Sequence[float],
    use_point_cloud_colors: bool,
    show_all_camera_positions: bool,
    show_all_camera_frustums: bool,
    frustum_start_index: int,
    frustum_end_index: int | None,
    include_sequence_point_clouds: bool = True,
) -> list[Any]:
    o3d = require_open3d()
    np = require_numpy()

    geometries: list[Any] = []
    centroid_positions: list[Any] = []
    centroid_lines: list[list[int]] = []

    scene_extent = compute_scene_extent(sequences)
    frame_size = 0.15 * scene_extent

    geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size))

    for order_index, sequence in enumerate(sequences):
        color = sequence_color(order_index, len(sequences))
        active_frustum_start, active_frustum_end = normalize_capture_index_range(
            capture_count=len(sequence.captures),
            start_index=frustum_start_index,
            end_index=frustum_end_index,
            label=sequence.sequence_path.name,
        )

        if include_sequence_point_clouds:
            geometries.append(
                create_visual_point_cloud(
                    sequence=sequence,
                    fallback_color=color,
                    use_point_cloud_colors=use_point_cloud_colors,
                )
            )

        camera_points = sequence.camera_positions
        if show_all_camera_positions:
            for position in camera_points[active_frustum_start : active_frustum_end + 1]:
                geometries.append(
                    create_marker_sphere(position, camera_marker_radius, camera_frustum_color)
                )

        if show_all_camera_frustums:
            if intrinsics is not None:
                for capture in sequence.captures[active_frustum_start : active_frustum_end + 1]:
                    geometries.append(
                        create_camera_frustum(
                            T_base_camera=capture.T_base_camera,
                            intrinsics=intrinsics,
                            depth=camera_frustum_depth,
                            plane_scale=camera_frustum_plane_scale,
                            color=camera_frustum_color,
                        )
                    )
            else:
                LOGGER.warning(
                    "Intrinsics were not provided, so camera frustums for %s cannot be shown.",
                    sequence.sequence_path.name,
                )
        centroid_positions.append(np.asarray(sequence.sequence_centroid, dtype=float))

    if centroid_positions:
        centroid_lines = [[idx, idx + 1] for idx in range(len(centroid_positions) - 1)]
        order_path = build_line_set(centroid_positions, centroid_lines, (0.95, 0.95, 0.95))
        if order_path is not None:
            geometries.append(order_path)

    return geometries


def preview_first_group_projection_animation(
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
    show_all_camera_positions: bool,
    show_all_camera_frustums: bool,
    frustum_start_index: int,
    frustum_end_index: int | None,
    preview_speed_multiplier: float,
    preview_second_group_start_fraction: float,
    preview_projection_start_scale: float,
    preview_animation_frames: int,
    preview_hold_frames: int,
    preview_fps: float,
    preview_easing: str,
    preview_point_limit: int | None,
    preview_grouping_mode: str,
    preview_capture_order_mode: str,
    preview_delay_fraction: float,
    preview_start_noise_scale: float,
    preview_frustum_fade_frames: int,
    preview_random_seed: int,
    film_camera_mode: str,
    film_target_mode: str,
    film_motion_easing: str,
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

    if len(sequences) != 1:
        raise ValidationError(
            "First-group animation preview currently supports exactly one loaded sequence. "
            f"Loaded {len(sequences)} sequences."
        )

    if preview_speed_multiplier <= 0.0:
        raise ValidationError("--preview-speed-multiplier must be positive.")
    if not (0.0 <= preview_second_group_start_fraction <= 1.0):
        raise ValidationError("--preview-second-group-start-fraction must be in [0, 1].")
    if not (0.0 < preview_projection_start_scale <= 1.0):
        raise ValidationError("--preview-projection-start-scale must be in (0, 1].")
    if preview_animation_frames <= 1:
        raise ValidationError("--preview-animation-frames must be greater than 1.")
    if preview_hold_frames < 0:
        raise ValidationError("--preview-hold-frames must be non-negative.")
    if preview_fps <= 0.0:
        raise ValidationError("--preview-fps must be positive.")
    if preview_point_limit is not None and preview_point_limit <= 0:
        raise ValidationError("--preview-point-limit must be positive when provided.")
    if preview_grouping_mode not in (
        "nearest_pose",
        "randomized_nearest_counts",
        "randomized_balanced",
    ):
        raise ValidationError(
            "Unsupported --preview-grouping-mode. Expected "
            "'nearest_pose', 'randomized_nearest_counts', or 'randomized_balanced'."
        )
    if preview_capture_order_mode not in ("index_order", "randomized_order"):
        raise ValidationError(
            "--preview-capture-order-mode must be 'index_order' or 'randomized_order'."
        )
    if not (0.0 <= preview_delay_fraction < 1.0):
        raise ValidationError("--preview-delay-fraction must be in [0, 1).")
    if preview_start_noise_scale < 0.0:
        raise ValidationError("--preview-start-noise-scale must be non-negative.")
    if preview_frustum_fade_frames < 0:
        raise ValidationError("--preview-frustum-fade-frames must be non-negative.")
    if film_camera_mode not in ("static_side", "orbit_side"):
        raise ValidationError(
            "--film-camera-mode must be 'static_side' or 'orbit_side'."
        )
    if film_target_mode not in ("final_centroid", "visible_centroid"):
        raise ValidationError(
            "--film-target-mode must be 'final_centroid' or 'visible_centroid'."
        )
    if film_motion_easing not in ("linear", "ease_out_cubic", "ease_in_out_cubic"):
        raise ValidationError(
            "--film-motion-easing must be one of: linear, ease_out_cubic, ease_in_out_cubic."
        )
    if film_zoom <= 0.0:
        raise ValidationError("--film-zoom must be positive.")
    if film_zoom_final <= 0.0:
        raise ValidationError("--film-zoom-final must be positive.")
    if export_fps is not None and export_fps <= 0.0:
        raise ValidationError("--export-fps must be positive when provided.")
    if intrinsics is None:
        raise ValidationError(
            "The first-group animation preview requires intrinsics so the points can start on the frustum plane."
        )
    if export_enabled and export_frames_dir is None:
        raise ValidationError(
            "Preview export is enabled, but --export-frames-dir is not configured."
        )

    sequence = sequences[0]
    active_frustum_start, active_frustum_end = normalize_capture_index_range(
        capture_count=len(sequence.captures),
        start_index=frustum_start_index,
        end_index=frustum_end_index,
        label=sequence.sequence_path.name,
    )

    assignments, counts = assign_points_to_preview_groups(
        sequence=sequence,
        grouping_mode=preview_grouping_mode,
        random_seed=preview_random_seed,
    )
    LOGGER.info(
        "Preview grouping for scan %03d | mode=%s | counts=%s",
        sequence.scan_index,
        preview_grouping_mode,
        counts.tolist(),
    )
    scene_extent = compute_scene_extent(sequences)
    camera_frustum_depth = max(scene_extent * camera_frustum_scale, scene_extent * 0.02)
    preview_rng = np.random.default_rng(preview_random_seed + 1)
    preview_capture_indices = resolve_preview_capture_indices(
        counts=counts,
        sequence_name=sequence.sequence_path.name,
        grouping_mode=preview_grouping_mode,
        allowed_start_index=active_frustum_start,
        allowed_end_index=active_frustum_end,
    )
    preview_capture_indices = order_preview_capture_indices(
        preview_capture_indices=preview_capture_indices,
        order_mode=preview_capture_order_mode,
        random_seed=preview_random_seed + 2,
    )
    preview_groups = [
        prepare_preview_group_data(
            sequence=sequence,
            assignments=assignments,
            capture_index=capture_index,
            intrinsics=intrinsics,
            camera_frustum_depth=camera_frustum_depth,
            camera_frustum_plane_scale=camera_frustum_plane_scale,
            preview_projection_start_scale=preview_projection_start_scale,
            preview_point_limit=preview_point_limit,
            preview_start_noise_scale=preview_start_noise_scale,
            camera_frustum_color=camera_frustum_color,
            scene_extent=scene_extent,
            rng=preview_rng,
            preview_delay_fraction=preview_delay_fraction,
        )
        for capture_index in preview_capture_indices
    ]
    filming_base_offset_direction = compute_default_side_offset_direction(
        preview_groups[0].capture.T_base_camera
    )
    all_final_points = np.vstack([group.final_points for group in preview_groups])

    camera_marker_radius = max(scene_extent * camera_marker_scale, scene_extent * 0.0025)
    static_geometries = build_visualization_geometries(
        sequences=sequences,
        intrinsics=intrinsics,
        camera_marker_radius=camera_marker_radius,
        camera_frustum_depth=camera_frustum_depth,
        camera_frustum_plane_scale=camera_frustum_plane_scale,
        camera_frustum_color=camera_frustum_color,
        use_point_cloud_colors=True,
        show_all_camera_positions=False,
        show_all_camera_frustums=False,
        frustum_start_index=active_frustum_start,
        frustum_end_index=active_frustum_end,
        include_sequence_point_clouds=False,
    )
    preview_markers: list[Any] = []
    if show_all_camera_positions:
        for group in preview_groups:
            marker = create_marker_sphere(
                position=group.capture.camera_position,
                radius=camera_marker_radius,
                color=background_color,
            )
            preview_markers.append(marker)
    preview_frustums: list[Any] = []
    if show_all_camera_frustums:
        for group in preview_groups:
            frustum = create_camera_frustum(
                T_base_camera=group.capture.T_base_camera,
                intrinsics=intrinsics,
                depth=camera_frustum_depth,
                plane_scale=camera_frustum_plane_scale,
                color=background_color,
            )
            preview_frustums.append(frustum)

    preview_clouds: list[Any] = []
    for _ in preview_groups:
        group_cloud = o3d.geometry.PointCloud()
        group_cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=float))
        group_cloud.colors = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=float))
        preview_clouds.append(group_cloud)

    effective_preview_fps = preview_fps * preview_speed_multiplier
    output_video_fps = export_fps if export_fps is not None else effective_preview_fps
    frame_output_dir = export_frames_dir
    if export_enabled:
        assert frame_output_dir is not None
        prepare_export_frames_directory(frame_output_dir)

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=f"Capture Range Projection Preview - Scan {sequence.scan_index:03d}",
        width=window_width,
        height=window_height,
    )
    try:
        for geometry in static_geometries:
            vis.add_geometry(geometry)
        for marker in preview_markers:
            vis.add_geometry(marker)
        for frustum in preview_frustums:
            vis.add_geometry(frustum)
        for group_cloud in preview_clouds:
            vis.add_geometry(group_cloud)

        render_option = vis.get_render_option()
        render_option.point_size = float(point_size)
        render_option.background_color = np.asarray(background_color, dtype=float)
        render_option.mesh_show_back_face = True
        initial_target_point = compute_preview_target_point(
            preview_groups=preview_groups,
            visible_point_sets=[all_final_points],
            target_mode=film_target_mode,
        )
        set_preview_filming_camera(
            vis=vis,
            base_offset_direction=filming_base_offset_direction,
            lookat=initial_target_point,
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
            motion_t=0.0,
        )
        vis.poll_events()
        vis.update_renderer()

        LOGGER.info(
            "Animating scan %03d captures %s in sequence | grouping_mode=%s | "
            "capture_order_mode=%s | "
            "speed_multiplier=%.2f | next_group_start_fraction=%.2f | "
            "film_camera_mode=%s | film_target_mode=%s | "
            "delay_fraction=%.3f | start_noise_scale=%.4f | frustum_fade_frames=%d | seed=%d",
            sequence.scan_index,
            preview_capture_indices,
            preview_grouping_mode,
            preview_capture_order_mode,
            preview_speed_multiplier,
            preview_second_group_start_fraction,
            film_camera_mode,
            film_target_mode,
            preview_delay_fraction,
            preview_start_noise_scale,
            preview_frustum_fade_frames,
            preview_random_seed,
        )
        if export_enabled:
            LOGGER.info(
                "Preview export enabled | frames_dir=%s | output_video=%s | output_fps=%.3f",
                frame_output_dir,
                export_video_path if export_video_path is not None else "<frames-only>",
                output_video_fps,
            )

        frame_interval = 1.0 / effective_preview_fps
        group_start_stride = int(
            round((preview_animation_frames - 1) * preview_second_group_start_fraction)
        )
        group_start_frames = [
            group_index * group_start_stride for group_index in range(len(preview_groups))
        ]
        total_animation_frames = max(
            start_frame + preview_animation_frames + preview_frustum_fade_frames
            for start_frame in group_start_frames
        )
        animation_start_time = time.perf_counter()
        exported_frame_index = 0

        for frame_idx in range(total_animation_frames):
            visible_point_sets: list[Any] = []
            for group_index, (group, group_cloud, start_frame) in enumerate(zip(
                preview_groups,
                preview_clouds,
                group_start_frames,
            )):
                group_points, group_colors = evaluate_group_frame_points(
                    group=group,
                    group_frame_index=frame_idx - start_frame,
                    group_frame_count=preview_animation_frames,
                    easing=preview_easing,
                    hide_before_start=start_frame > 0,
                )
                group_cloud.points = o3d.utility.Vector3dVector(group_points)
                group_cloud.colors = o3d.utility.Vector3dVector(group_colors)
                vis.update_geometry(group_cloud)
                visible_point_sets.append(group_points)
                frustum_intensity = compute_preview_frustum_intensity(
                    group_frame_index=frame_idx - start_frame,
                    group_frame_count=preview_animation_frames,
                    fade_frame_count=preview_frustum_fade_frames,
                )
                dynamic_camera_color = blend_rgb(
                    source_color=background_color,
                    target_color=camera_frustum_color,
                    blend_t=frustum_intensity,
                )
                if show_all_camera_positions:
                    set_mesh_uniform_color(preview_markers[group_index], dynamic_camera_color)
                    vis.update_geometry(preview_markers[group_index])
                if show_all_camera_frustums:
                    set_line_set_uniform_color(
                        preview_frustums[group_index],
                        dynamic_camera_color,
                    )
                    vis.update_geometry(preview_frustums[group_index])
            camera_motion_t = (
                1.0 if total_animation_frames <= 1 else frame_idx / float(total_animation_frames - 1)
            )
            camera_motion_t = easing_value(camera_motion_t, film_motion_easing)
            target_point = compute_preview_target_point(
                preview_groups=preview_groups,
                visible_point_sets=visible_point_sets,
                target_mode=film_target_mode,
            )
            set_preview_filming_camera(
                vis=vis,
                base_offset_direction=filming_base_offset_direction,
                lookat=target_point,
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
                motion_t=camera_motion_t,
            )
            vis.poll_events()
            vis.update_renderer()
            if export_enabled:
                capture_export_frame(
                    vis,
                    frame_output_dir / f"frame_{exported_frame_index:06d}.png",
                )
                exported_frame_index += 1
            else:
                target_time = animation_start_time + (frame_idx + 1) * frame_interval
                remaining = target_time - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)

        final_target_point = compute_preview_target_point(
            preview_groups=preview_groups,
            visible_point_sets=[all_final_points],
            target_mode=film_target_mode,
        )
        hold_start_time = time.perf_counter()
        for hold_idx in range(preview_hold_frames):
            for marker in preview_markers:
                set_mesh_uniform_color(marker, background_color)
                vis.update_geometry(marker)
            for frustum in preview_frustums:
                set_line_set_uniform_color(frustum, background_color)
                vis.update_geometry(frustum)
            for group_cloud in preview_clouds:
                vis.update_geometry(group_cloud)
            set_preview_filming_camera(
                vis=vis,
                base_offset_direction=filming_base_offset_direction,
                lookat=final_target_point,
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
                motion_t=1.0,
            )
            vis.poll_events()
            vis.update_renderer()
            if export_enabled:
                capture_export_frame(
                    vis,
                    frame_output_dir / f"frame_{exported_frame_index:06d}.png",
                )
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
                    "Preview video export complete | video=%s | frames=%d",
                    export_video_path,
                    exported_frame_index,
                )
                if not export_keep_frames:
                    cleanup_exported_frames(frame_output_dir)
                    LOGGER.info(
                        "Deleted exported frame sequence after encoding | frames_dir=%s",
                        frame_output_dir,
                    )
            else:
                LOGGER.info(
                    "Preview frame export complete without video encoding | frames_dir=%s | frames=%d",
                    frame_output_dir,
                    exported_frame_index,
                )
            return

        vis.run()
    finally:
        vis.destroy_window()


def parse_color(color_text: str) -> tuple[float, float, float]:
    color_text = color_text.strip()
    if color_text.startswith("#"):
        hex_text = color_text[1:]
        if len(hex_text) != 6:
            raise argparse.ArgumentTypeError(
                f"Invalid background color '{color_text}'. Expected '#RRGGBB'."
            )
        try:
            values = [int(hex_text[i : i + 2], 16) / 255.0 for i in (0, 2, 4)]
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid background color '{color_text}'."
            ) from exc
        return tuple(values)  # type: ignore[return-value]

    try:
        values = [float(part.strip()) for part in color_text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Invalid background color '{color_text}'. Expected 'r,g,b'."
        ) from exc

    if len(values) != 3:
        raise argparse.ArgumentTypeError(
            f"Invalid background color '{color_text}'. Expected exactly 3 components."
        )

    if any(value > 1.0 for value in values):
        values = [value / 255.0 for value in values]

    if any(value < 0.0 or value > 1.0 for value in values):
        raise argparse.ArgumentTypeError(
            f"Invalid background color '{color_text}'. Components must be in [0, 1] or [0, 255]."
        )

    return tuple(values)  # type: ignore[return-value]


def show_visualization(
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
    show_all_camera_positions: bool,
    show_all_camera_frustums: bool,
    frustum_start_index: int,
    frustum_end_index: int | None,
) -> None:
    o3d = require_open3d()
    np = require_numpy()

    scene_extent = compute_scene_extent(sequences)
    camera_marker_radius = max(scene_extent * camera_marker_scale, scene_extent * 0.0025)
    camera_frustum_depth = max(scene_extent * camera_frustum_scale, scene_extent * 0.02)
    geometries = build_visualization_geometries(
        sequences=sequences,
        intrinsics=intrinsics,
        camera_marker_radius=camera_marker_radius,
        camera_frustum_depth=camera_frustum_depth,
        camera_frustum_plane_scale=camera_frustum_plane_scale,
        camera_frustum_color=camera_frustum_color,
        use_point_cloud_colors=use_point_cloud_colors,
        show_all_camera_positions=show_all_camera_positions,
        show_all_camera_frustums=show_all_camera_frustums,
        frustum_start_index=frustum_start_index,
        frustum_end_index=frustum_end_index,
    )

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Cinematic Reconstruction Stage 1",
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


def print_dataset_summary(sequences: Sequence[SequenceData], dataset_root: Path) -> None:
    np = require_numpy()
    total_points = sum(sequence.point_count for sequence in sequences)
    all_bounds: list[Any] = []
    for sequence in sequences:
        if sequence.point_count > 0:
            all_bounds.extend([sequence.bbox_min, sequence.bbox_max])
        all_bounds.extend([sequence.camera_positions.min(axis=0), sequence.camera_positions.max(axis=0)])

    if all_bounds:
        bounds = np.vstack(all_bounds)
        global_min = bounds.min(axis=0)
        global_max = bounds.max(axis=0)
        global_extent = global_max - global_min
    else:
        global_min = np.zeros(3, dtype=float)
        global_max = np.zeros(3, dtype=float)
        global_extent = np.zeros(3, dtype=float)

    LOGGER.info(
        "Loaded %d sequences from %s | total_points=%d | global_min=%s | global_max=%s | global_extent=%s",
        len(sequences),
        dataset_root,
        total_points,
        format_vector(global_min),
        format_vector(global_max),
        format_vector(global_extent),
    )

    for order_index, sequence in enumerate(sequences, start=1):
        LOGGER.info(
            "Sequence order %02d -> scan %03d -> %s",
            order_index,
            sequence.scan_index,
            sequence.sequence_path.name,
        )


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)

    try:
        dataset_root = resolve_optional_path_arg(args.dataset_root)
        extrinsics_path = resolve_optional_path_arg(args.extrinsics_path)
        intrinsics_path = resolve_optional_path_arg(args.intrinsics_path)
        export_frames_dir = resolve_optional_path_arg(args.export_frames_dir)
        export_video_path = resolve_optional_path_arg(args.export_video_path)
        try:
            background_color = parse_color(args.background_color)
            camera_frustum_color = parse_color(args.camera_frustum_color)
        except argparse.ArgumentTypeError as exc:
            raise ValidationError(str(exc)) from exc

        if dataset_root is None:
            raise ValidationError(
                "Dataset root is not configured. Edit SCRIPT_INPUTS.dataset_root at the "
                "top of the script or pass --dataset-root."
            )

        if extrinsics_path is None:
            raise ValidationError(
                "Extrinsics path is not configured. Edit SCRIPT_INPUTS.extrinsics_path at "
                "the top of the script or pass --extrinsics-path."
            )

        if args.expected_captures <= 0:
            raise ValidationError("--expected-captures must be a positive integer.")
        if args.export_enabled and args.no_visualization:
            raise ValidationError(
                "--export-enabled cannot be combined with --no-visualization."
            )
        if args.export_enabled and not args.animate_first_group_preview:
            raise ValidationError(
                "Preview export currently supports only the animated preview path. "
                "Disable --export-enabled or keep --animate-first-group-preview enabled."
            )
        if args.export_enabled and export_frames_dir is None:
            raise ValidationError(
                "Preview export is enabled, but --export-frames-dir is not configured."
            )

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

        sequences: list[SequenceData] = []
        for scan_index, sequence_path in discover_sequence_directories(dataset_root):
            sequence = load_sequence(
                scan_index=scan_index,
                sequence_path=sequence_path,
                expected_captures=args.expected_captures,
                T_extrinsic=T_extrinsic,
                convention=args.extrinsic_convention,
            )
            print_sequence_summary(sequence)
            sequences.append(sequence)

        print_dataset_summary(sequences, dataset_root)

        if not args.no_visualization:
            if args.animate_first_group_preview:
                preview_first_group_projection_animation(
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
                    show_all_camera_positions=args.show_all_camera_positions,
                    show_all_camera_frustums=args.show_all_camera_frustums,
                    frustum_start_index=args.frustum_start_index,
                    frustum_end_index=args.frustum_end_index,
                    preview_speed_multiplier=args.preview_speed_multiplier,
                    preview_second_group_start_fraction=args.preview_second_group_start_fraction,
                    preview_projection_start_scale=args.preview_projection_start_scale,
                    preview_animation_frames=args.preview_animation_frames,
                    preview_hold_frames=args.preview_hold_frames,
                    preview_fps=args.preview_fps,
                    preview_easing=args.preview_easing,
                    preview_point_limit=args.preview_point_limit,
                    preview_grouping_mode=args.preview_grouping_mode,
                    preview_capture_order_mode=args.preview_capture_order_mode,
                    preview_delay_fraction=args.preview_delay_fraction,
                    preview_start_noise_scale=args.preview_start_noise_scale,
                    preview_frustum_fade_frames=args.preview_frustum_fade_frames,
                    preview_random_seed=args.preview_random_seed,
                    film_camera_mode=args.film_camera_mode,
                    film_target_mode=args.film_target_mode,
                    film_motion_easing=args.film_motion_easing,
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
                show_visualization(
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
                    show_all_camera_positions=args.show_all_camera_positions,
                    show_all_camera_frustums=args.show_all_camera_frustums,
                    frustum_start_index=args.frustum_start_index,
                    frustum_end_index=args.frustum_end_index,
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
