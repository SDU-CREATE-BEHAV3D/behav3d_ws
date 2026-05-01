#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_field_oriented_sequence_v2

Runtime YAML (hot-reload each cycle):
ros2 run behav3d_orchestrator print_field_oriented_sequence_v2 --ros-args \
  -p runtime_config_path:=/home/lab/behav3d_ws/src/behav3d_orchestrator/config/print_field_oriented_sequence_config.yaml
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from behav3d_orchestrator.src.field_loop_session import FieldLoopSession
from behav3d_orchestrator.src.print_session import PrintSession
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


class PrintFieldOrientedSequenceV2Node(Node):
    """
    Field-oriented print loop v2.

    v2 keeps the scan/reconstruction/field loop flow in this sequence, but
    delegates print execution to PrintSession:
    - flat `targets:` YAML prints as dots,
    - nested `segments:` YAML prints as line paths.
    """

    def __init__(self):
        super().__init__("print_field_oriented_sequence_v2")

        # Runtime config control
        self.declare_parameter("runtime_config_path", "")

        # Same node, dedicated field-loop session + print/YAML session.
        self.session = FieldLoopSession(self)
        self.print_session = PrintSession(self)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        runtime_config_path = self._resolve_runtime_config_path(
            str(self.get_parameter("runtime_config_path").value).strip()
        )
        cfg = self._load_runtime_config(runtime_config_path)

        home_before_scan = self._cfg_bool(cfg, "home_before_scan")
        home_after = self._cfg_bool(cfg, "home_after")
        timeout_s = self._cfg_optional_timeout(cfg, "timeout_s")

        frame_id = self._cfg_str(cfg, "frame_id")
        scan_eef_link = self._cfg_str(cfg, "scan_eef_link")
        scan_use_tf_orientation = self._cfg_bool(cfg, "scan_use_tf_orientation")
        scan_width = self._cfg_float(cfg, "scan_width")
        scan_height = self._cfg_float(cfg, "scan_height")
        scan_center_x = self._cfg_float(cfg, "scan_center_x")
        scan_center_y = self._cfg_float(cfg, "scan_center_y")
        scan_center_z = self._cfg_float(cfg, "scan_center_z")
        scan_z_off = self._cfg_float(cfg, "scan_z_off")
        scan_nx = self._cfg_int(cfg, "scan_nx")
        scan_ny = self._cfg_int(cfg, "scan_ny")
        scan_row_major = self._cfg_bool(cfg, "scan_row_major")
        scan_capture_folder = self._cfg_str(cfg, "scan_capture_folder")
        scan_debug_prompt = self._cfg_bool(cfg, "scan_debug_prompt")
        scan_vel_scale = self._cfg_float(cfg, "scan_vel_scale")
        scan_accel_scale = self._cfg_float(cfg, "scan_accel_scale")
        scan_publish_markers = self._cfg_bool(cfg, "scan_publish_markers")
        scan_axis_length = self._cfg_float(cfg, "scan_axis_length")
        scan_axis_radius = self._cfg_float(cfg, "scan_axis_radius")
        scan_clear_markers_before = self._cfg_bool(cfg, "scan_clear_markers_before")

        layer_scan_width = self._cfg_float(cfg, "layer_scan_width")
        layer_scan_height = self._cfg_float(cfg, "layer_scan_height")
        layer_scan_nx = self._cfg_int(cfg, "layer_scan_nx")
        layer_scan_ny = self._cfg_int(cfg, "layer_scan_ny")
        layer_scan_row_major = self._cfg_bool(cfg, "layer_scan_row_major")
        layer_scan_first_z_off = self._cfg_float(cfg, "layer_scan_first_z_off")
        layer_scan_z_margin_m = self._cfg_float(cfg, "layer_scan_z_margin_mm") / 1000.0
        field_center_sign_x = self._cfg_float(cfg, "field_center_sign_x")
        field_center_sign_y = self._cfg_float(cfg, "field_center_sign_y")
        field_center_offset_x = self._cfg_float(cfg, "field_center_offset_x")
        field_center_offset_y = self._cfg_float(cfg, "field_center_offset_y")

        run_reconstruction = self._cfg_bool(cfg, "run_reconstruction")
        reconstruct_device = self._cfg_str(cfg, "reconstruct_device")
        reconstruct_request_timeout_s = self._cfg_float(cfg, "reconstruct_request_timeout_s")
        wait_reconstruction_outputs = self._cfg_bool(cfg, "wait_reconstruction_outputs")
        color_to_depth_wait_timeout_s = self._cfg_float(cfg, "color_to_depth_wait_timeout_s")
        tsdf_wait_timeout_s = self._cfg_float(cfg, "tsdf_wait_timeout_s")
        mesh_prefer = self._cfg_str(cfg, "mesh_prefer")
        mesh_update_wait_timeout_s = self._cfg_float(cfg, "mesh_update_wait_timeout_s")
        mesh_update_request_timeout_s = self._cfg_float(cfg, "mesh_update_request_timeout_s")
        tsdf_center_crop_enable = self._cfg_bool(cfg, "tsdf_center_crop_enable")
        tsdf_center_crop_width = self._cfg_int(cfg, "tsdf_center_crop_width")
        tsdf_center_crop_height = self._cfg_int(cfg, "tsdf_center_crop_height")
        tsdf_center_crop_apply_to_depth = self._cfg_bool(cfg, "tsdf_center_crop_apply_to_depth")
        tsdf_aabb_crop_enable = self._cfg_bool(cfg, "tsdf_aabb_crop_enable")
        tsdf_aabb_crop_min = self._cfg_float_list(cfg, "tsdf_aabb_crop_min")
        tsdf_aabb_crop_max = self._cfg_float_list(cfg, "tsdf_aabb_crop_max")
        tsdf_param_update_timeout_s = self._cfg_float(cfg, "tsdf_param_update_timeout_s")

        run_field_init = self._cfg_bool(cfg, "run_field_init")
        field_use_latest = self._cfg_bool(cfg, "field_use_latest")
        field_session_path = self._cfg_str(cfg, "field_session_path")
        field_scan_mesh_path = self._cfg_str_allow_empty(cfg, "field_scan_mesh_path")
        field_mesh_path = self._cfg_str(cfg, "field_mesh_path")
        field_state_output_dir = self._cfg_str(cfg, "field_state_output_dir")
        field_request_timeout_s = self._cfg_float(cfg, "field_request_timeout_s")
        skip_bootstrap_scan_and_init = self._cfg_bool(cfg, "skip_bootstrap_scan_and_init")
        resume_field_state_path = self._cfg_str_allow_empty(cfg, "resume_field_state_path")
        resume_scan_mesh_path = self._cfg_str_allow_empty(cfg, "resume_scan_mesh_path")
        scan_before_generate_first_cycle = self._cfg_bool(cfg, "scan_before_generate_first_cycle")

        run_generate_candidates = self._cfg_bool(cfg, "run_generate_candidates")
        candidate_use_latest = self._cfg_bool(cfg, "candidate_use_latest")
        candidate_session_path = self._cfg_str(cfg, "candidate_session_path")
        candidate_field_state_path = self._cfg_str_allow_empty(cfg, "candidate_field_state_path")
        candidate_scan_mesh_path = self._cfg_str_allow_empty(cfg, "candidate_scan_mesh_path")
        candidate_output_dir = self._cfg_str(cfg, "candidate_output_dir")
        candidate_request_timeout_s = self._cfg_float(cfg, "candidate_request_timeout_s")
        candidate_mode = self._cfg_str(cfg, "candidate_mode")
        candidate_beads_per_step = self._cfg_int(cfg, "candidate_beads_per_step")
        candidate_bead_separation_mm = self._cfg_float(cfg, "candidate_bead_separation_mm")
        candidate_bead_height_mm = self._cfg_float(cfg, "candidate_bead_height_mm")
        candidate_walk_distance_mm = self._cfg_float(cfg, "candidate_walk_distance_mm")
        candidate_walk_step_mm = self._cfg_float(cfg, "candidate_walk_step_mm")
        candidate_walk_max_steps = self._cfg_int(cfg, "candidate_walk_max_steps")
        candidate_walk_start_fraction = self._cfg_float(cfg, "candidate_walk_start_fraction")
        candidate_target_zx = self._cfg_float(cfg, "candidate_target_zx")
        candidate_target_zy = self._cfg_float(cfg, "candidate_target_zy")
        candidate_target_zz = self._cfg_float(cfg, "candidate_target_zz")
        candidate_target_position_scale = self._cfg_float(cfg, "candidate_target_position_scale")
        candidate_visualize_field_ply = self._cfg_bool(cfg, "candidate_visualize_field_ply")
        candidate_restore_scan_mesh_after_field_ply = self._cfg_bool(
            cfg, "candidate_restore_scan_mesh_after_field_ply"
        )
        candidate_publish_markers = self._cfg_bool(cfg, "candidate_publish_markers")
        candidate_marker_axis_length = self._cfg_float(cfg, "candidate_marker_axis_length")
        candidate_marker_axis_radius = self._cfg_float(cfg, "candidate_marker_axis_radius")
        candidate_marker_clear_before = self._cfg_bool(cfg, "candidate_marker_clear_before")
        oriented_targets_enable = self._cfg_bool(cfg, "oriented_targets_enable")
        oriented_tangent_sign = self._cfg_float(cfg, "oriented_tangent_sign")
        oriented_clamp_to_cone = self._cfg_bool(cfg, "oriented_clamp_to_cone")
        oriented_cone_max_tilt_deg = self._cfg_float(cfg, "oriented_cone_max_tilt_deg")
        oriented_base_to_world_yaw_deg = self._cfg_float(cfg, "oriented_base_to_world_yaw_deg")
        segment_print_speed = self._cfg_int(cfg, "segment_print_speed")
        segment_approach_z_offset_m = self._cfg_float(cfg, "segment_approach_z_offset_m")
        segment_travel_z_offset_m = self._cfg_float(cfg, "segment_travel_z_offset_m")
        segment_approach_vel_scale = self._cfg_float(cfg, "segment_approach_vel_scale")
        segment_travel_vel_scale = self._cfg_float(cfg, "segment_travel_vel_scale")
        segment_print_vel_scale = self._cfg_float(cfg, "segment_print_vel_scale")
        segment_accel_scale = self._cfg_float(cfg, "segment_accel_scale")
        segment_target_print_speed_mm_s = self._cfg_float(cfg, "segment_target_print_speed_mm_s")
        max_cycles = self._cfg_int(cfg, "max_cycles")
        prompt_before_next_cycle = self._cfg_bool(cfg, "prompt_before_next_cycle")

        log.info(
            "Starting print_field_oriented sequence: "
            f"initial_grid=({scan_nx}x{scan_ny}), "
            f"layer_grid=({layer_scan_nx}x{layer_scan_ny}), "
            f"run_reconstruction={run_reconstruction}, run_field_init={run_field_init}, "
            f"run_generate_candidates={run_generate_candidates}, max_cycles={max_cycles}, "
            f"candidate_mode={candidate_mode}, "
            f"walk_distance_mm={candidate_walk_distance_mm:.1f}, "
            f"segment_print_speed={segment_print_speed}, segment_print_v={segment_print_vel_scale:.3f}, "
            f"segment_target_print_speed_mm_s={segment_target_print_speed_mm_s:.3f}, "
            f"oriented_targets_enable={oriented_targets_enable}, "
            f"clamp_to_cone={oriented_clamp_to_cone}, cone_max_tilt_deg={oriented_cone_max_tilt_deg:.1f}, "
            f"skip_bootstrap_scan_and_init={skip_bootstrap_scan_and_init}, "
            f"scan_before_generate_first_cycle={scan_before_generate_first_cycle}, "
            f"runtime_config_path='{runtime_config_path}'"
        )

        mesh_path = ""
        rgb_ply_path = ""
        field_state_path = str(candidate_field_state_path).strip()
        field_center_xyz: Optional[tuple[float, float, float]] = None
        last_printed_targets: List[PoseStamped] = []
        reference_scan_orientation: Optional[tuple[float, float, float, float]] = None

        try:
            if home_before_scan:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )

            eef = str(scan_eef_link).strip()
            if not eef:
                log.warn(
                    "[print_field_oriented] scan_eef_link is empty; cannot capture TF scan orientation."
                )
            else:
                try:
                    pose_res = self.session.run_sync(
                        self.session.camera.get_pose(
                            eef=eef,
                            base_frame=str(frame_id).strip() or "world",
                            use_tf=True,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                    if pose_res.get("ok", False) and "pose" in pose_res:
                        pose = pose_res["pose"]
                        reference_scan_orientation = (
                            float(pose.pose.orientation.x),
                            float(pose.pose.orientation.y),
                            float(pose.pose.orientation.z),
                            float(pose.pose.orientation.w),
                        )
                        log.info(
                            "[print_field_oriented] Initial scan reference orientation captured from TF."
                        )
                    else:
                        err = str(pose_res.get("error", "unknown"))
                        log.warn(
                            "[print_field_oriented] Could not capture initial scan reference orientation from TF. "
                            f"Will rely on bootstrap/pre-scan orientation if available. error='{err}'"
                        )
                except TimeoutError:
                    log.warn(
                        "[print_field_oriented] Timed out while reading scanner TF pose for "
                        "reference orientation."
                    )

            # -----------------------------------------------------------------
            # Bootstrap cycle: table scan + reconstruction + field init.
            # Optional resume mode: skip bootstrap and continue from existing
            # field state (plus optional existing scan mesh path).
            # -----------------------------------------------------------------
            if skip_bootstrap_scan_and_init:
                field_state_path = (resume_field_state_path or field_state_path).strip()
                if not field_state_path:
                    raise RuntimeError(
                        "skip_bootstrap_scan_and_init=True requires a field state path via "
                        "resume_field_state_path or candidate_field_state_path."
                    )

                field_center_raw = self.session.compute_field_center_from_state(field_state_path)
                field_center_xy_mapped = self.session.map_field_center_xy(
                    x=float(field_center_raw[0]),
                    y=float(field_center_raw[1]),
                    sign_x=field_center_sign_x,
                    sign_y=field_center_sign_y,
                    offset_x=field_center_offset_x,
                    offset_y=field_center_offset_y,
                )
                field_center_xyz = (
                    float(field_center_xy_mapped[0]),
                    float(field_center_xy_mapped[1]),
                    float(field_center_raw[2]),
                )

                mesh_path = (
                    resume_scan_mesh_path or candidate_scan_mesh_path or field_scan_mesh_path or mesh_path
                ).strip()

                if mesh_path:
                    mesh_res = self.session.run_sync(
                        self.session.camera.update_world_mesh(
                            use_latest=False,
                            session_path=candidate_session_path or field_session_path or "@session",
                            mesh_path=mesh_path,
                            ply_path="",
                            prefer="mesh",
                            wait_timeout_s=mesh_update_wait_timeout_s,
                            enqueue=False,
                        ),
                        timeout_s=mesh_update_request_timeout_s,
                    )
                    if not mesh_res.get("ok", False):
                        raise RuntimeError(
                            f"update_world_mesh(resume_scan_mesh) failed: {mesh_res.get('error')}"
                        )
                    published_path = str(mesh_res.get("metrics", {}).get("published_path", "")).strip()
                    published_kind = str(mesh_res.get("metrics", {}).get("published_kind", "")).strip()
                    log.info(
                        f"[print_field_oriented] Resume scan mesh published in RViz ({published_kind}): "
                        f"{published_path}"
                    )

                log.info(
                    "[print_field_oriented] Skipping bootstrap scan/init. "
                    f"Using existing field state='{field_state_path}', "
                    f"field_center_raw=({field_center_raw[0]:.4f}, {field_center_raw[1]:.4f}, {field_center_raw[2]:.4f}), "
                    f"field_center_scan=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}), "
                    f"xy_map=(x*{field_center_sign_x:.3f}+{field_center_offset_x:.3f}, "
                    f"y*{field_center_sign_y:.3f}+{field_center_offset_y:.3f}), "
                    f"scan_mesh='{mesh_path}'"
                )

                if scan_before_generate_first_cycle:
                    cycle_tag = "cycle_0001"
                    cycle_root = f"@session/field_loop/{cycle_tag}"
                    cycle_scan_capture_folder = f"{cycle_root}/scan"
                    cycle_scan_folder = f"field_loop/{cycle_tag}/scan"
                    cycle_scan_z_off = float(layer_scan_first_z_off) + float(layer_scan_z_margin_m)
                    abs_scan_z = float(field_center_xyz[2]) + float(cycle_scan_z_off)
                    pre_scan_use_tf_orientation = bool(scan_use_tf_orientation)

                    log.info(
                        f"[print_field_oriented] ===== pre_scan_for_{cycle_tag} ===== "
                        f"scan_capture='{cycle_scan_capture_folder}' scan_folder='{cycle_scan_folder}' "
                        f"center=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}) "
                        f"grid={layer_scan_nx}x{layer_scan_ny} size={1000.0*layer_scan_width:.1f}x"
                        f"{1000.0*layer_scan_height:.1f}mm "
                        f"z_off={cycle_scan_z_off:.4f}m => abs_scan_z={abs_scan_z:.4f}m "
                        f"use_tf_orientation={pre_scan_use_tf_orientation}"
                    )

                    pre_scan_targets = self.session.run_grid_sweep(
                        target=None,
                        width=layer_scan_width,
                        height=layer_scan_height,
                        center_x=float(field_center_xyz[0]),
                        center_y=float(field_center_xyz[1]),
                        center_z=float(field_center_xyz[2]),
                        z_off=cycle_scan_z_off,
                        nx=layer_scan_nx,
                        ny=layer_scan_ny,
                        row_major=layer_scan_row_major,
                        eef_link=scan_eef_link,
                        use_tf_orientation=pre_scan_use_tf_orientation,
                        debug=scan_debug_prompt,
                        capture_folder=cycle_scan_capture_folder,
                        do_home=False,
                        vel_scale=scan_vel_scale,
                        accel_scale=scan_accel_scale,
                        timeout_s=timeout_s,
                        publish_markers=scan_publish_markers,
                        axis_length=scan_axis_length,
                        axis_radius=scan_axis_radius,
                        clear_markers_before=scan_clear_markers_before,
                        frame_id=frame_id,
                    )
                    if not pre_scan_targets:
                        raise RuntimeError(f"Grid sweep produced 0 targets for pre_scan_for_{cycle_tag}.")
                    reference_scan_orientation = (
                        float(pre_scan_targets[0].pose.orientation.x),
                        float(pre_scan_targets[0].pose.orientation.y),
                        float(pre_scan_targets[0].pose.orientation.z),
                        float(pre_scan_targets[0].pose.orientation.w),
                    )

                    if run_reconstruction:
                        mesh_path, rgb_ply_path = self.session.run_reconstruction_for_scan(
                            scan_folder=cycle_scan_folder,
                            reconstruct_device=reconstruct_device,
                            reconstruct_request_timeout_s=reconstruct_request_timeout_s,
                            wait_reconstruction_outputs=wait_reconstruction_outputs,
                            color_to_depth_wait_timeout_s=color_to_depth_wait_timeout_s,
                            tsdf_wait_timeout_s=tsdf_wait_timeout_s,
                            mesh_prefer=mesh_prefer,
                            mesh_update_wait_timeout_s=mesh_update_wait_timeout_s,
                            mesh_update_request_timeout_s=mesh_update_request_timeout_s,
                            tsdf_center_crop_enable=tsdf_center_crop_enable,
                            tsdf_center_crop_width=tsdf_center_crop_width,
                            tsdf_center_crop_height=tsdf_center_crop_height,
                            tsdf_center_crop_apply_to_depth=tsdf_center_crop_apply_to_depth,
                            tsdf_aabb_crop_enable=tsdf_aabb_crop_enable,
                            tsdf_aabb_crop_min=tsdf_aabb_crop_min,
                            tsdf_aabb_crop_max=tsdf_aabb_crop_max,
                            tsdf_param_update_timeout_s=tsdf_param_update_timeout_s,
                        )
            else:
                bootstrap_tag = "cycle_0000"
                bootstrap_root = f"@session/field_loop/{bootstrap_tag}"
                bootstrap_scan_capture_folder = f"{bootstrap_root}/scan"
                bootstrap_scan_folder = f"field_loop/{bootstrap_tag}/scan"
                bootstrap_field_state_output_dir = f"{bootstrap_root}/field_init"

                log.info(
                    f"[print_field_oriented] ===== {bootstrap_tag} (bootstrap) ===== "
                    f"scan_capture='{bootstrap_scan_capture_folder}' "
                    f"scan_folder='{bootstrap_scan_folder}'"
                )

                bootstrap_targets = self.session.run_grid_sweep(
                    target=None,
                    width=scan_width,
                    height=scan_height,
                    center_x=scan_center_x,
                    center_y=scan_center_y,
                    center_z=scan_center_z,
                    z_off=scan_z_off,
                    nx=scan_nx,
                    ny=scan_ny,
                    row_major=scan_row_major,
                    eef_link=scan_eef_link,
                    use_tf_orientation=scan_use_tf_orientation,
                    debug=scan_debug_prompt,
                    capture_folder=bootstrap_scan_capture_folder,
                    do_home=False,
                    vel_scale=scan_vel_scale,
                    accel_scale=scan_accel_scale,
                    timeout_s=timeout_s,
                    publish_markers=scan_publish_markers,
                    axis_length=scan_axis_length,
                    axis_radius=scan_axis_radius,
                    clear_markers_before=scan_clear_markers_before,
                    frame_id=frame_id,
                )
                if not bootstrap_targets:
                    raise RuntimeError("Bootstrap grid sweep produced 0 targets.")

                reference_scan_orientation = (
                    float(bootstrap_targets[0].pose.orientation.x),
                    float(bootstrap_targets[0].pose.orientation.y),
                    float(bootstrap_targets[0].pose.orientation.z),
                    float(bootstrap_targets[0].pose.orientation.w),
                )

                if run_reconstruction:
                    mesh_path, rgb_ply_path = self.session.run_reconstruction_for_scan(
                        scan_folder=bootstrap_scan_folder,
                        reconstruct_device=reconstruct_device,
                        reconstruct_request_timeout_s=reconstruct_request_timeout_s,
                        wait_reconstruction_outputs=wait_reconstruction_outputs,
                        color_to_depth_wait_timeout_s=color_to_depth_wait_timeout_s,
                        tsdf_wait_timeout_s=tsdf_wait_timeout_s,
                        mesh_prefer=mesh_prefer,
                        mesh_update_wait_timeout_s=mesh_update_wait_timeout_s,
                        mesh_update_request_timeout_s=mesh_update_request_timeout_s,
                        tsdf_center_crop_enable=tsdf_center_crop_enable,
                        tsdf_center_crop_width=tsdf_center_crop_width,
                        tsdf_center_crop_height=tsdf_center_crop_height,
                        tsdf_center_crop_apply_to_depth=tsdf_center_crop_apply_to_depth,
                        tsdf_aabb_crop_enable=tsdf_aabb_crop_enable,
                        tsdf_aabb_crop_min=tsdf_aabb_crop_min,
                        tsdf_aabb_crop_max=tsdf_aabb_crop_max,
                        tsdf_param_update_timeout_s=tsdf_param_update_timeout_s,
                    )

                if run_field_init:
                    scan_mesh_for_field = field_scan_mesh_path or mesh_path
                    scan_mesh_paths = [scan_mesh_for_field] if scan_mesh_for_field else []
                    init_res = self.session.run_sync(
                        self.session.field.init_field_from_scan(
                            use_latest=field_use_latest,
                            session_path=field_session_path,
                            scan_mesh_paths=scan_mesh_paths,
                            field_mesh_path=field_mesh_path,
                            state_output_dir=bootstrap_field_state_output_dir,
                            enqueue=False,
                        ),
                        timeout_s=field_request_timeout_s,
                    )
                    if not init_res.get("ok", False):
                        raise RuntimeError(f"init_field_from_scan failed: {init_res.get('error')}")

                    metrics = init_res.get("metrics", {})
                    field_state_path = str(metrics.get("field_state_path", "")).strip()
                    if not field_state_path:
                        raise RuntimeError("init_field_from_scan did not return field_state_path")

                    field_center_raw = self.session.compute_field_center_from_state(field_state_path)
                    field_center_xy = self.session.map_field_center_xy(
                        x=float(field_center_raw[0]),
                        y=float(field_center_raw[1]),
                        sign_x=field_center_sign_x,
                        sign_y=field_center_sign_y,
                        offset_x=field_center_offset_x,
                        offset_y=field_center_offset_y,
                    )
                    field_center_xyz = (
                        float(field_center_xy[0]),
                        float(field_center_xy[1]),
                        float(field_center_raw[2]),
                    )
                    log.info(
                        "[print_field_oriented] Field initialized: "
                        f"state='{field_state_path}', "
                        f"debug_ply='{metrics.get('debug_field_ply_path', '')}', "
                        f"offset=({metrics.get('offset_x', 0.0):.4f}, "
                        f"{metrics.get('offset_y', 0.0):.4f}, "
                        f"{metrics.get('offset_z', 0.0):.4f}), "
                        f"field_center_raw=({field_center_raw[0]:.4f}, {field_center_raw[1]:.4f}, {field_center_raw[2]:.4f}), "
                        f"field_center_scan=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}), "
                        f"xy_map=(x*{field_center_sign_x:.3f}+{field_center_offset_x:.3f}, "
                        f"y*{field_center_sign_y:.3f}+{field_center_offset_y:.3f})"
                    )

                    debug_field_ply_path = str(metrics.get("debug_field_ply_path", "")).strip()
                    if debug_field_ply_path:
                        field_vis_res = self.session.run_sync(
                            self.session.camera.update_world_mesh(
                                use_latest=False,
                                session_path=field_session_path or "@session",
                                mesh_path="",
                                ply_path=debug_field_ply_path,
                                prefer="ply",
                                wait_timeout_s=mesh_update_wait_timeout_s,
                                enqueue=False,
                            ),
                            timeout_s=mesh_update_request_timeout_s,
                        )
                        if not field_vis_res.get("ok", False):
                            raise RuntimeError(
                                f"update_world_mesh(field_ply) failed: {field_vis_res.get('error')}"
                            )
                        field_vis_path = str(field_vis_res.get("metrics", {}).get("published_path", "")).strip()
                        field_vis_kind = str(field_vis_res.get("metrics", {}).get("published_kind", "")).strip()
                        log.info(
                            f"[print_field_oriented] Field PLY published in RViz ({field_vis_kind}): {field_vis_path}"
                        )
                else:
                    if not field_state_path:
                        raise RuntimeError(
                            "run_field_init=False and candidate_field_state_path is empty. "
                            "Provide an existing field state path."
                        )
                    field_center_raw = self.session.compute_field_center_from_state(field_state_path)
                    field_center_xy = self.session.map_field_center_xy(
                        x=float(field_center_raw[0]),
                        y=float(field_center_raw[1]),
                        sign_x=field_center_sign_x,
                        sign_y=field_center_sign_y,
                        offset_x=field_center_offset_x,
                        offset_y=field_center_offset_y,
                    )
                    field_center_xyz = (
                        float(field_center_xy[0]),
                        float(field_center_xy[1]),
                        float(field_center_raw[2]),
                    )
                    log.info(
                        "[print_field_oriented] Using provided field state: "
                        f"state='{field_state_path}', "
                        f"field_center_raw=({field_center_raw[0]:.4f}, {field_center_raw[1]:.4f}, {field_center_raw[2]:.4f}), "
                        f"field_center_scan=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}), "
                        f"xy_map=(x*{field_center_sign_x:.3f}+{field_center_offset_x:.3f}, "
                        f"y*{field_center_sign_y:.3f}+{field_center_offset_y:.3f})"
                    )

            if field_center_xyz is None:
                raise RuntimeError("Field center not available after bootstrap.")

            if not run_generate_candidates:
                log.warn("[print_field_oriented] run_generate_candidates=False; stopping after bootstrap.")
                return

            # -----------------------------------------------------------------
            # Print cycles:
            # - cycle_0001 candidates come from bootstrap scan mesh.
            # - after each print, run a centered layer scan for the next cycle.
            # -----------------------------------------------------------------
            print_cycle_index = 0
            while rclpy.ok():
                try:
                    cfg = self._load_runtime_config(runtime_config_path)
                except Exception as exc:
                    log.warn(
                        "[print_field_oriented] Runtime config reload failed; keeping previous config. "
                        f"path='{runtime_config_path}' error='{exc}'"
                    )

                max_cycles = self._cfg_int(cfg, "max_cycles")
                prompt_before_next_cycle = self._cfg_bool(cfg, "prompt_before_next_cycle")
                candidate_use_latest = self._cfg_bool(cfg, "candidate_use_latest")
                candidate_session_path = self._cfg_str(cfg, "candidate_session_path")
                candidate_field_state_path = self._cfg_str_allow_empty(cfg, "candidate_field_state_path")
                candidate_scan_mesh_path = self._cfg_str_allow_empty(cfg, "candidate_scan_mesh_path")
                candidate_request_timeout_s = self._cfg_float(cfg, "candidate_request_timeout_s")
                candidate_mode = self._cfg_str(cfg, "candidate_mode")
                candidate_beads_per_step = self._cfg_int(cfg, "candidate_beads_per_step")
                candidate_bead_separation_mm = self._cfg_float(cfg, "candidate_bead_separation_mm")
                candidate_bead_height_mm = self._cfg_float(cfg, "candidate_bead_height_mm")
                candidate_walk_distance_mm = self._cfg_float(cfg, "candidate_walk_distance_mm")
                candidate_walk_step_mm = self._cfg_float(cfg, "candidate_walk_step_mm")
                candidate_walk_max_steps = self._cfg_int(cfg, "candidate_walk_max_steps")
                candidate_walk_start_fraction = self._cfg_float(cfg, "candidate_walk_start_fraction")
                candidate_target_zx = self._cfg_float(cfg, "candidate_target_zx")
                candidate_target_zy = self._cfg_float(cfg, "candidate_target_zy")
                candidate_target_zz = self._cfg_float(cfg, "candidate_target_zz")
                candidate_target_position_scale = self._cfg_float(cfg, "candidate_target_position_scale")
                candidate_visualize_field_ply = self._cfg_bool(cfg, "candidate_visualize_field_ply")
                candidate_restore_scan_mesh_after_field_ply = self._cfg_bool(
                    cfg, "candidate_restore_scan_mesh_after_field_ply"
                )
                candidate_publish_markers = self._cfg_bool(cfg, "candidate_publish_markers")
                candidate_marker_axis_length = self._cfg_float(cfg, "candidate_marker_axis_length")
                candidate_marker_axis_radius = self._cfg_float(cfg, "candidate_marker_axis_radius")
                candidate_marker_clear_before = self._cfg_bool(cfg, "candidate_marker_clear_before")
                oriented_targets_enable = self._cfg_bool(cfg, "oriented_targets_enable")
                oriented_tangent_sign = self._cfg_float(cfg, "oriented_tangent_sign")
                oriented_clamp_to_cone = self._cfg_bool(cfg, "oriented_clamp_to_cone")
                oriented_cone_max_tilt_deg = self._cfg_float(cfg, "oriented_cone_max_tilt_deg")
                oriented_base_to_world_yaw_deg = self._cfg_float(cfg, "oriented_base_to_world_yaw_deg")
                segment_print_speed = self._cfg_int(cfg, "segment_print_speed")
                segment_approach_z_offset_m = self._cfg_float(cfg, "segment_approach_z_offset_m")
                segment_travel_z_offset_m = self._cfg_float(cfg, "segment_travel_z_offset_m")
                segment_approach_vel_scale = self._cfg_float(cfg, "segment_approach_vel_scale")
                segment_travel_vel_scale = self._cfg_float(cfg, "segment_travel_vel_scale")
                segment_print_vel_scale = self._cfg_float(cfg, "segment_print_vel_scale")
                segment_accel_scale = self._cfg_float(cfg, "segment_accel_scale")
                segment_target_print_speed_mm_s = self._cfg_float(cfg, "segment_target_print_speed_mm_s")
                layer_scan_width = self._cfg_float(cfg, "layer_scan_width")
                layer_scan_height = self._cfg_float(cfg, "layer_scan_height")
                layer_scan_nx = self._cfg_int(cfg, "layer_scan_nx")
                layer_scan_ny = self._cfg_int(cfg, "layer_scan_ny")
                layer_scan_row_major = self._cfg_bool(cfg, "layer_scan_row_major")
                layer_scan_first_z_off = self._cfg_float(cfg, "layer_scan_first_z_off")
                layer_scan_z_margin_m = self._cfg_float(cfg, "layer_scan_z_margin_mm") / 1000.0
                scan_debug_prompt = self._cfg_bool(cfg, "scan_debug_prompt")
                scan_vel_scale = self._cfg_float(cfg, "scan_vel_scale")
                scan_accel_scale = self._cfg_float(cfg, "scan_accel_scale")
                scan_publish_markers = self._cfg_bool(cfg, "scan_publish_markers")
                scan_axis_length = self._cfg_float(cfg, "scan_axis_length")
                scan_axis_radius = self._cfg_float(cfg, "scan_axis_radius")
                scan_clear_markers_before = self._cfg_bool(cfg, "scan_clear_markers_before")
                scan_eef_link = self._cfg_str(cfg, "scan_eef_link")

                if max_cycles > 0 and print_cycle_index >= max_cycles:
                    log.info(f"[print_field_oriented] Reached max_cycles={max_cycles}.")
                    break

                cycle_tag = f"cycle_{print_cycle_index + 1:04d}"
                cycle_root = f"@session/field_loop/{cycle_tag}"
                cycle_candidate_output_dir = f"{cycle_root}/candidates"

                scan_mesh_for_candidates = candidate_scan_mesh_path or field_scan_mesh_path or mesh_path
                scan_mesh_paths = [scan_mesh_for_candidates] if scan_mesh_for_candidates else []
                field_state_for_candidates = candidate_field_state_path or field_state_path
                if not field_state_for_candidates:
                    raise RuntimeError(
                        "Missing field state for candidate generation. "
                        "Run field init or provide candidate_field_state_path."
                    )

                cand_res = self.session.run_sync(
                        self.session.field.generate_print_candidates(
                            use_latest=candidate_use_latest,
                            session_path=candidate_session_path,
                            field_state_path=field_state_for_candidates,
                            scan_mesh_paths=scan_mesh_paths,
                            output_dir=cycle_candidate_output_dir,
                            candidate_mode=candidate_mode,
                            beads_per_step=candidate_beads_per_step,
                            bead_separation_mm=candidate_bead_separation_mm,
                            bead_height_mm=candidate_bead_height_mm,
                            walk_distance_mm=candidate_walk_distance_mm,
                            walk_step_mm=candidate_walk_step_mm,
                            walk_max_steps=candidate_walk_max_steps,
                            walk_start_fraction=candidate_walk_start_fraction,
                            orient_with_tangent=oriented_targets_enable,
                            tangent_sign=oriented_tangent_sign,
                            clamp_to_cone=oriented_clamp_to_cone,
                            cone_max_tilt_deg=oriented_cone_max_tilt_deg,
                            base_to_world_yaw_deg=oriented_base_to_world_yaw_deg,
                            target_zx=candidate_target_zx,
                            target_zy=candidate_target_zy,
                            target_zz=candidate_target_zz,
                            target_position_scale=candidate_target_position_scale,
                            enqueue=False,
                    ),
                    timeout_s=candidate_request_timeout_s,
                )
                if not cand_res.get("ok", False):
                    raise RuntimeError(f"generate_print_candidates failed: {cand_res.get('error')}")

                cand_metrics = cand_res.get("metrics", {})
                log.info(
                    "[print_field_oriented] Candidates generated: "
                    f"count={cand_metrics.get('candidate_count', 0)} "
                    f"viable={cand_metrics.get('viable_count', 0)} "
                    f"contour_segments={cand_metrics.get('contour_segment_count', 0)} "
                    f"yaml='{cand_metrics.get('targets_yaml_path', '')}' "
                    f"cycle_dir='{cand_metrics.get('cycle_output_dir', '')}'"
                )

                targets_yaml_path = str(cand_metrics.get("targets_yaml_path", "")).strip()
                preview_targets: List[PoseStamped] = []
                preview_segments = []
                if targets_yaml_path:
                    preview_segments = self.print_session.parse_yaml_segments(
                        yaml_path=targets_yaml_path,
                        frame_id=frame_id,
                    )
                    preview_targets = self.print_session.parse_yaml_targets(
                        yaml_path=targets_yaml_path,
                        frame_id=frame_id,
                    )

                if candidate_visualize_field_ply:
                    cycle_field_ply = str(cand_metrics.get("debug_field_ply_path", "")).strip()
                    if cycle_field_ply:
                        cycle_vis_res = self.session.run_sync(
                            self.session.camera.preview_field_ply(
                                use_latest=False,
                                session_path=candidate_session_path or "@session",
                                field_ply_path=cycle_field_ply,
                                restore_mesh_path=(
                                    scan_mesh_for_candidates if candidate_restore_scan_mesh_after_field_ply else ""
                                ),
                                wait_timeout_s=mesh_update_wait_timeout_s,
                                enqueue=False,
                            ),
                            timeout_s=mesh_update_request_timeout_s,
                        )
                        if not cycle_vis_res.get("ok", False):
                            raise RuntimeError(
                                f"preview_field_ply failed: {cycle_vis_res.get('error')}"
                            )

                        cycle_vis_path = str(
                            cycle_vis_res.get("metrics", {}).get("field_ply_published_path", "")
                        ).strip()
                        cycle_vis_kind = str(
                            cycle_vis_res.get("metrics", {}).get("field_ply_published_kind", "ply")
                        ).strip()
                        log.info(
                            f"[print_field_oriented] Cycle field PLY published in RViz ({cycle_vis_kind}): "
                            f"{cycle_vis_path}"
                        )

                        if candidate_restore_scan_mesh_after_field_ply and scan_mesh_for_candidates:
                            restored_path = str(
                                cycle_vis_res.get("metrics", {}).get("restore_mesh_published_path", "")
                            ).strip()
                            restored_kind = str(
                                cycle_vis_res.get("metrics", {}).get("restore_mesh_published_kind", "mesh")
                            ).strip()
                            log.info(
                                f"[print_field_oriented] Scan mesh restored in RViz ({restored_kind}): "
                                f"{restored_path}"
                            )

                if candidate_publish_markers and preview_targets:
                    marker_res = self.session.run_sync(
                        self.session.util.publish_targets(
                            preview_targets,
                            axis_length=float(candidate_marker_axis_length),
                            axis_radius=float(candidate_marker_axis_radius),
                            clear_before=bool(candidate_marker_clear_before),
                            enqueue=False,
                        ),
                        timeout_s=mesh_update_request_timeout_s,
                    )
                    if not marker_res.get("ok", False):
                        raise RuntimeError(
                            f"publish_targets(candidates) failed: {marker_res.get('error')}"
                        )
                    log.info(
                        f"[print_field_oriented] Candidate markers published: {len(preview_targets)} targets "
                        f"(yaml='{targets_yaml_path}', frame='{frame_id}')"
                    )

                if not preview_targets:
                    log.warn("[print_field_oriented] No candidate targets generated; stopping loop.")
                    break

                gate_res = self.session.run_sync(
                    self.session.util.input(
                        prompt=(
                            f"[print_field_oriented:{cycle_tag}] Press ENTER to print targets "
                            "(type 'q' + ENTER to stop)."
                        ),
                        enqueue=False,
                    ),
                    timeout_s=None,
                )
                gate_value = str(gate_res.get("metrics", {}).get("value", "")).strip().lower()
                if gate_value == "q":
                    log.warn("[print_field_oriented] Print execution stopped by user.")
                    break

                if preview_segments:
                    print_res = self.print_session.run_print_segments(
                        segments=preview_segments,
                        print_speed=segment_print_speed,
                        approach_z_offset_m=segment_approach_z_offset_m,
                        travel_z_offset_m=segment_travel_z_offset_m,
                        approach_vel_scale=segment_approach_vel_scale,
                        travel_vel_scale=segment_travel_vel_scale,
                        print_vel_scale=segment_print_vel_scale,
                        accel_scale=segment_accel_scale,
                        target_print_speed_mm_s=segment_target_print_speed_mm_s,
                        timeout_s=timeout_s,
                    )
                else:
                    print_res = self.print_session.run_print_dots_targets(
                        targets=preview_targets,
                        timeout_s=timeout_s,
                    )
                if not print_res.get("ok", False):
                    log.warn(
                        "[print_field_oriented] Print stage returned non-fatal failure: "
                        f"stage={print_res.get('stage')} error={print_res.get('error', 'unknown')}"
                    )
                    break

                printed_count = int(print_res.get("printed", 0))
                skipped_count = int(print_res.get("skipped", 0))
                if preview_segments:
                    log.info(
                        "[print_field_oriented] Print segments completed: "
                        f"printed={printed_count} skipped={skipped_count} / segments={print_res.get('segments', 0)}"
                    )
                else:
                    log.info(
                        "[print_field_oriented] Print dots completed: "
                        f"printed={printed_count} skipped={skipped_count} / targets={print_res.get('targets', 0)}"
                    )
                printed_targets_res = print_res.get("printed_targets", [])
                if isinstance(printed_targets_res, list) and printed_targets_res:
                    last_printed_targets = list(printed_targets_res)
                elif preview_segments:
                    last_printed_targets = list(preview_targets[: 2 * printed_count])
                else:
                    last_printed_targets = list(preview_targets[:printed_count])

                print_cycle_index += 1
                if max_cycles > 0 and print_cycle_index >= max_cycles:
                    log.info(f"[print_field_oriented] Reached max_cycles={max_cycles}.")
                    break

                if prompt_before_next_cycle:
                    next_res = self.session.run_sync(
                        self.session.util.input(
                            prompt=(
                                f"[print_field_oriented:{cycle_tag}] Press ENTER for next cycle "
                                f"(cycle_{print_cycle_index + 1:04d}) or type 'q' + ENTER to stop."
                            ),
                            enqueue=False,
                        ),
                        timeout_s=None,
                    )
                    next_value = str(next_res.get("metrics", {}).get("value", "")).strip().lower()
                    if next_value == "q":
                        log.info("[print_field_oriented] Stopped by user before next cycle.")
                        break

                next_cycle_tag = f"cycle_{print_cycle_index + 1:04d}"
                next_cycle_root = f"@session/field_loop/{next_cycle_tag}"
                next_cycle_scan_capture_folder = f"{next_cycle_root}/scan"
                next_cycle_scan_folder = f"field_loop/{next_cycle_tag}/scan"

                center_target = PoseStamped()
                center_target.header.frame_id = frame_id
                center_target.pose.position.x = float(field_center_xyz[0])
                center_target.pose.position.y = float(field_center_xyz[1])
                center_target.pose.position.z = float(field_center_xyz[2])
                if reference_scan_orientation is not None:
                    center_target.pose.orientation.x = float(reference_scan_orientation[0])
                    center_target.pose.orientation.y = float(reference_scan_orientation[1])
                    center_target.pose.orientation.z = float(reference_scan_orientation[2])
                    center_target.pose.orientation.w = float(reference_scan_orientation[3])
                else:
                    center_target.pose.orientation.w = 1.0

                if last_printed_targets:
                    highest_prev_z = max(float(t.pose.position.z) for t in last_printed_targets)
                    abs_scan_z_from_print = highest_prev_z + float(layer_scan_z_margin_m)
                    min_abs_scan_z = (
                        float(field_center_xyz[2])
                        + float(layer_scan_first_z_off)
                        + float(layer_scan_z_margin_m)
                    )
                    abs_scan_z = max(abs_scan_z_from_print, min_abs_scan_z)
                    cycle_scan_z_off = abs_scan_z - float(field_center_xyz[2])
                    z_msg = (
                        f"highest_prev_z={highest_prev_z:.4f}m + margin={layer_scan_z_margin_m:.4f}m "
                        f"=> from_print={abs_scan_z_from_print:.4f}m, "
                        f"min_from_center={min_abs_scan_z:.4f}m "
                        f"(first_z_off+margin) => abs_scan_z={abs_scan_z:.4f}m "
                        f"=> z_off={cycle_scan_z_off:.4f}m"
                    )
                else:
                    cycle_scan_z_off = float(layer_scan_first_z_off) + float(layer_scan_z_margin_m)
                    abs_scan_z = float(field_center_xyz[2]) + float(cycle_scan_z_off)
                    z_msg = (
                        f"fallback(no printed targets): z_off={cycle_scan_z_off:.4f}m "
                        f"=> abs_scan_z={abs_scan_z:.4f}m"
                    )

                log.info(
                    f"[print_field_oriented] ===== scan_for_{next_cycle_tag} ===== "
                    f"scan_capture='{next_cycle_scan_capture_folder}' "
                    f"scan_folder='{next_cycle_scan_folder}' "
                    f"center=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}) "
                    f"grid={layer_scan_nx}x{layer_scan_ny} size={1000.0*layer_scan_width:.1f}x"
                    f"{1000.0*layer_scan_height:.1f}mm {z_msg}"
                )

                scan_targets = self.session.run_grid_sweep(
                    target=center_target,
                    width=layer_scan_width,
                    height=layer_scan_height,
                    z_off=cycle_scan_z_off,
                    nx=layer_scan_nx,
                    ny=layer_scan_ny,
                    row_major=layer_scan_row_major,
                    eef_link=scan_eef_link,
                    use_tf_orientation=False,
                    debug=scan_debug_prompt,
                    capture_folder=next_cycle_scan_capture_folder,
                    do_home=False,
                    vel_scale=scan_vel_scale,
                    accel_scale=scan_accel_scale,
                    timeout_s=timeout_s,
                    publish_markers=scan_publish_markers,
                    axis_length=scan_axis_length,
                    axis_radius=scan_axis_radius,
                    clear_markers_before=scan_clear_markers_before,
                    frame_id=center_target.header.frame_id,
                )
                if not scan_targets:
                    raise RuntimeError(f"Grid sweep produced 0 targets for scan_for_{next_cycle_tag}.")

                if run_reconstruction:
                    mesh_path, rgb_ply_path = self.session.run_reconstruction_for_scan(
                        scan_folder=next_cycle_scan_folder,
                        reconstruct_device=reconstruct_device,
                        reconstruct_request_timeout_s=reconstruct_request_timeout_s,
                        wait_reconstruction_outputs=wait_reconstruction_outputs,
                        color_to_depth_wait_timeout_s=color_to_depth_wait_timeout_s,
                        tsdf_wait_timeout_s=tsdf_wait_timeout_s,
                        mesh_prefer=mesh_prefer,
                        mesh_update_wait_timeout_s=mesh_update_wait_timeout_s,
                        mesh_update_request_timeout_s=mesh_update_request_timeout_s,
                        tsdf_center_crop_enable=tsdf_center_crop_enable,
                        tsdf_center_crop_width=tsdf_center_crop_width,
                        tsdf_center_crop_height=tsdf_center_crop_height,
                        tsdf_center_crop_apply_to_depth=tsdf_center_crop_apply_to_depth,
                        tsdf_aabb_crop_enable=tsdf_aabb_crop_enable,
                        tsdf_aabb_crop_min=tsdf_aabb_crop_min,
                        tsdf_aabb_crop_max=tsdf_aabb_crop_max,
                        tsdf_param_update_timeout_s=tsdf_param_update_timeout_s,
                    )
                else:
                    log.warn(
                        "[print_field_oriented] run_reconstruction=False after scan; "
                        "next cycle will reuse previous scan mesh unless candidate_scan_mesh_path is set."
                    )

            if home_after:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )

        except TimeoutError:
            log.error("[print_field_oriented] Sequence timed out.")
        except Exception as exc:
            log.error(f"[print_field_oriented] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _load_runtime_config(self, config_path: str) -> Dict[str, Any]:
        if yaml is None:
            raise RuntimeError("PyYAML is not available; cannot load runtime YAML config.")

        path = Path(str(config_path).strip()).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"Runtime config file not found: {path}")

        try:
            raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise RuntimeError(f"Failed to read runtime config '{path}': {exc}") from exc

        params_map = self._extract_runtime_param_map(raw)
        if not params_map:
            raise ValueError(
                f"Runtime config '{path}' does not contain a valid parameter map "
                "(expected ros__parameters block or plain key/value map)."
            )
        return dict(params_map)

    def _resolve_runtime_config_path(self, config_path: str) -> str:
        path_text = str(config_path).strip()
        if path_text:
            path = Path(path_text).expanduser()
            if not path.is_absolute():
                path = (Path.cwd() / path).resolve()
            return str(path)
        return self._default_runtime_config_path()

    @staticmethod
    def _default_runtime_config_path() -> str:
        return "/home/lab/behav3d_ws/src/behav3d_orchestrator/config/print_field_oriented_sequence_config.yaml"

    def _extract_runtime_param_map(self, raw: Any) -> Dict[str, Any]:
        if not isinstance(raw, dict):
            return {}

        node_name = str(self.get_name()).strip()
        node_keys = [node_name, f"/{node_name}"]
        if node_name == "print_field_oriented_sequence_v2":
            node_keys.extend(["print_field_oriented_sequence", "/print_field_oriented_sequence"])
        for key in node_keys:
            block = raw.get(key)
            if isinstance(block, dict):
                ros_params = block.get("ros__parameters")
                if isinstance(ros_params, dict):
                    return dict(ros_params)

        ros_params = raw.get("ros__parameters")
        if isinstance(ros_params, dict):
            return dict(ros_params)

        return dict(raw)

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if value is None:
            raise ValueError("expected bool, got null")
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text in ("1", "true", "yes", "y", "on"):
            return True
        if text in ("0", "false", "no", "n", "off"):
            return False
        raise ValueError(f"expected bool-like value, got '{value}'")

    @staticmethod
    def _as_int(value: Any) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            raise ValueError(f"expected int, got '{value}'")

    @staticmethod
    def _as_float(value: Any) -> float:
        try:
            out = float(value)
            if np.isfinite(out):
                return out
        except (TypeError, ValueError):
            pass
        raise ValueError(f"expected finite float, got '{value}'")

    @staticmethod
    def _as_str(value: Any) -> str:
        if value is None:
            raise ValueError("expected non-empty string, got null")
        text = str(value).strip()
        if not text:
            raise ValueError("expected non-empty string, got empty")
        return text

    @staticmethod
    def _as_float_list(value: Any) -> List[float]:
        if isinstance(value, (list, tuple)):
            out: List[float] = []
            for x in value:
                try:
                    fx = float(x)
                except (TypeError, ValueError):
                    raise ValueError(f"expected float list, got '{value}'")
                if not np.isfinite(fx):
                    raise ValueError(f"expected finite float list, got '{value}'")
                out.append(float(fx))
            if not out:
                raise ValueError("expected non-empty float list")
            return out
        raise ValueError(f"expected float list, got '{value}'")

    @classmethod
    def _optional_timeout(cls, value: Any) -> Optional[float]:
        val = cls._as_float(value)
        if val <= 0.0:
            return None
        return val

    @staticmethod
    def _require(cfg: Dict[str, Any], key: str) -> Any:
        if key not in cfg:
            raise KeyError(f"Missing required config key: '{key}'")
        return cfg[key]

    @classmethod
    def _cfg_bool(cls, cfg: Dict[str, Any], key: str) -> bool:
        return cls._as_bool(cls._require(cfg, key))

    @classmethod
    def _cfg_int(cls, cfg: Dict[str, Any], key: str) -> int:
        return cls._as_int(cls._require(cfg, key))

    @classmethod
    def _cfg_float(cls, cfg: Dict[str, Any], key: str) -> float:
        return cls._as_float(cls._require(cfg, key))

    @classmethod
    def _cfg_str(cls, cfg: Dict[str, Any], key: str) -> str:
        return cls._as_str(cls._require(cfg, key))

    @classmethod
    def _cfg_str_allow_empty(cls, cfg: Dict[str, Any], key: str) -> str:
        value = cls._require(cfg, key)
        if value is None:
            raise ValueError(f"expected string, got null for key '{key}'")
        return str(value).strip()

    @classmethod
    def _cfg_float_list(cls, cfg: Dict[str, Any], key: str) -> List[float]:
        return cls._as_float_list(cls._require(cfg, key))

    @classmethod
    def _cfg_optional_timeout(cls, cfg: Dict[str, Any], key: str) -> Optional[float]:
        return cls._optional_timeout(cls._require(cfg, key))

def main(args=None):
    rclpy.init(args=args)
    node = PrintFieldOrientedSequenceV2Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
