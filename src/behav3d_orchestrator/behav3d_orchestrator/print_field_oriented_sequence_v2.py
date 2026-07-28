#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_field_oriented_sequence_v2
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from behav3d_orchestrator.src.field_loop_session import FieldLoopSession
from behav3d_orchestrator.src.print_session import PrintSession
from behav3d_orchestrator.src.scan_session import ScanSession
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

CONFIG_FILENAME = "print_field_oriented_sequence_config.yaml"
SRC_CONFIG_PATH = f"/home/lab/behav3d_ws/src/behav3d_orchestrator/config/{CONFIG_FILENAME}"


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

        # Field logic stays in FieldLoopSession; scan execution uses ScanSession.
        self.session = FieldLoopSession(self)
        self.print_session = PrintSession(self)
        self.scan_session = ScanSession(self)
        self.control_pause = self.session.control_pause

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _wait_if_control_paused(self, label: str) -> bool:
        return self.control_pause.wait(label)

    def _run(self):
        log = self.get_logger()

        runtime_config_path = self._default_runtime_config_path()
        cfg = self._load_runtime_config(runtime_config_path)
        run_session_path = self._latest_capture_session_path()
        run_session_arg = str(run_session_path)
        first_print_cycle_number = self._first_available_cycle_number(run_session_path)

        rt = self._parse_runtime_state(cfg, run_session_path)

        def refresh_runtime_config(reason: str) -> None:
            new_cfg = self._reload_runtime_config_or_keep(
                current=rt.config,
                config_path=runtime_config_path,
                reason=reason,
            )
            rt.__dict__.update(
                self._parse_runtime_state(new_cfg, run_session_path).__dict__
            )

        log.info(
            "Starting print_field_oriented sequence: "
            f"scan_type={rt.scan_type}, enable_scan={rt.enable_scan}, debug_mode={rt.debug_mode}, "
            f"prompt_before_scan={rt.prompt_before_scan}, "
            f"initial_grid=({rt.scan_nx}x{rt.scan_ny}), "
            f"cycle_grid=({rt.grid_nx}x{rt.grid_ny}), "
            f"run_reconstruction={rt.run_reconstruction}, run_field_init={rt.run_field_init}, "
            f"run_generate_candidates={rt.run_generate_candidates}, max_cycles={rt.max_cycles}, "
            f"candidate_mode={rt.candidate_mode}, "
            f"field_position_target_enabled={rt.field_use_position_target}, "
            f"field_seed_level={rt.field_seed_level:.4f}, "
            f"field_require_full_hit={rt.field_require_full_hit}, "
            f"field_base_z_offset={rt.field_base_z_offset:.4f}, "
            f"field_position_target=({rt.field_position_target_x:.3f},{rt.field_position_target_y:.3f}), "
            f"print_mode={rt.print_mode}, dot_steps={rt.dot_steps}, "
            f"walk_distance_mm={rt.candidate_walk_distance_mm:.1f}, "
            f"segment_print_speed={rt.segment_print_speed}, segment_print_v={rt.segment_print_vel_scale:.3f}, "
            f"segment_target_print_speed_mm_s={rt.segment_target_print_speed_mm_s:.3f}, "
            f"post_segment_wait_s={rt.post_segment_wait_s:.3f}, "
            f"post_segment_retract_s={rt.post_segment_retract_s:.3f}, "
            f"post_segment_retract_speed={rt.post_segment_retract_speed}, "
            f"oriented_targets_enable={rt.oriented_targets_enable}, "
            f"clamp_to_cone={rt.oriented_clamp_to_cone}, cone_max_tilt_deg={rt.oriented_cone_max_tilt_deg:.1f}, "
            f"skip_bootstrap_scan_and_init={rt.skip_bootstrap_scan_and_init}, "
            f"scan_before_generate_first_cycle={rt.scan_before_generate_first_cycle}, "
            f"run_session='{run_session_arg}', first_cycle={first_print_cycle_number:04d}, "
            f"runtime_config_path='{runtime_config_path}'"
        )

        mesh_path = ""
        rgb_ply_path = ""
        field_state_path = str(rt.candidate_field_state_path).strip()
        field_center_xyz: Optional[tuple[float, float, float]] = None
        last_generated_targets: List[PoseStamped] = []
        reference_scan_orientation: Optional[tuple[float, float, float, float]] = None

        try:
            if rt.home_before_scan:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=rt.timeout_s,
                )
                if not self._wait_if_control_paused("after home_before_scan"):
                    return

            eef = str(rt.scan_eef_link).strip()
            if not eef:
                log.warn(
                    "[print_field_oriented] scan_eef_link is empty; cannot capture TF scan orientation."
                )
            else:
                try:
                    pose_res = self.session.run_sync(
                        self.session.camera.get_pose(
                            eef=eef,
                            base_frame=str(rt.frame_id).strip() or "world",
                            use_tf=True,
                            enqueue=False,
                        ),
                        timeout_s=rt.timeout_s,
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
            if rt.skip_bootstrap_scan_and_init:
                field_state_path = (rt.resume_field_state_path or field_state_path).strip()
                if not field_state_path:
                    raise RuntimeError(
                        "skip_bootstrap_scan_and_init=True requires a field state path via "
                        "resume_field_state_path or candidate_field_state_path."
                    )

                field_center_raw = self.session.compute_field_center_from_state(field_state_path)
                field_center_xy_mapped = self.session.map_field_center_xy(
                    x=float(field_center_raw[0]),
                    y=float(field_center_raw[1]),
                    sign_x=rt.field_center_sign_x,
                    sign_y=rt.field_center_sign_y,
                    offset_x=rt.field_center_offset_x,
                    offset_y=rt.field_center_offset_y,
                )
                field_center_xyz = (
                    float(field_center_xy_mapped[0]),
                    float(field_center_xy_mapped[1]),
                    float(field_center_raw[2]),
                )

                mesh_path = (
                    rt.resume_scan_mesh_path or rt.candidate_scan_mesh_path or rt.field_scan_mesh_path or mesh_path
                ).strip()

                if mesh_path:
                    mesh_res = self.session.run_sync(
                        self.session.camera.update_world_mesh(
                            use_latest=False,
                            session_path=rt.candidate_session_path or rt.field_session_path or run_session_arg,
                            mesh_path=mesh_path,
                            ply_path="",
                            prefer="mesh",
                            wait_timeout_s=rt.mesh_update_wait_timeout_s,
                            enqueue=False,
                        ),
                        timeout_s=rt.mesh_update_request_timeout_s,
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
                    if not self._wait_if_control_paused("after resume scan mesh publish"):
                        return

                log.info(
                    "[print_field_oriented] Skipping bootstrap scan/init. "
                    f"Using existing field state='{field_state_path}', "
                    f"field_center_raw=({field_center_raw[0]:.4f}, {field_center_raw[1]:.4f}, {field_center_raw[2]:.4f}), "
                    f"field_center_scan=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}), "
                    f"xy_map=(x*{rt.field_center_sign_x:.3f}+{rt.field_center_offset_x:.3f}, "
                    f"y*{rt.field_center_sign_y:.3f}+{rt.field_center_offset_y:.3f}), "
                    f"scan_mesh='{mesh_path}'"
                )

                if rt.scan_before_generate_first_cycle:
                    refresh_runtime_config(f"before pre_scan_for_cycle_{first_print_cycle_number:04d}")
                    cycle_tag = f"cycle_{first_print_cycle_number:04d}"
                    cycle_root = self._run_session_child(run_session_path, f"field_loop/{cycle_tag}")
                    cycle_scan_capture_folder = f"{cycle_root}/scan"
                    cycle_scan_folder = f"field_loop/{cycle_tag}/scan"
                    cycle_scan_z_off = float(rt.grid_z_off)
                    abs_scan_z = float(rt.grid_center_z) + float(cycle_scan_z_off)
                    pre_scan_use_tf_orientation = bool(rt.grid_use_tf_orientation)

                    log.info(
                        f"[print_field_oriented] ===== pre_scan_for_{cycle_tag} ===== "
                        f"scan_capture='{cycle_scan_capture_folder}' scan_folder='{cycle_scan_folder}' "
                        f"center=({rt.grid_center_x:.4f}, {rt.grid_center_y:.4f}, {rt.grid_center_z:.4f}) "
                        f"grid={rt.grid_nx}x{rt.grid_ny} size={1000.0*rt.grid_width:.1f}x"
                        f"{1000.0*rt.grid_height:.1f}mm "
                        f"z_off={cycle_scan_z_off:.4f}m => abs_scan_z={abs_scan_z:.4f}m "
                        f"use_tf_orientation={pre_scan_use_tf_orientation}"
                    )

                    if not rt.enable_scan:
                        log.info(
                            f"[print_field_oriented] pre_scan_for_{cycle_tag} skipped because enable_scan=false."
                        )
                    else:
                        if not self._prompt_before_scan_if_enabled(
                            enabled=rt.prompt_before_scan,
                            label=f"pre_scan_for_{cycle_tag}",
                        ):
                            log.info("[print_field_oriented] Stopped by user before pre-scan.")
                            return

                        pre_scan_targets = self._run_configured_scan(
                            cfg=rt.config,
                            scan_type=rt.scan_type,
                            target=None,
                            width=rt.grid_width,
                            height=rt.grid_height,
                            center_x=float(rt.grid_center_x),
                            center_y=float(rt.grid_center_y),
                            center_z=float(rt.grid_center_z),
                            z_off=cycle_scan_z_off,
                            nx=rt.grid_nx,
                            ny=rt.grid_ny,
                            row_major=rt.grid_row_major,
                            use_tf_orientation=pre_scan_use_tf_orientation,
                            capture_folder=cycle_scan_capture_folder,
                            timeout_s=rt.timeout_s,
                            frame_id=rt.frame_id,
                        )
                        if not pre_scan_targets:
                            raise RuntimeError(f"Grid sweep produced 0 targets for pre_scan_for_{cycle_tag}.")
                        if not self._wait_if_control_paused(f"after pre_scan_for_{cycle_tag}"):
                            return
                        reference_scan_orientation = (
                            float(pre_scan_targets[0].pose.orientation.x),
                            float(pre_scan_targets[0].pose.orientation.y),
                            float(pre_scan_targets[0].pose.orientation.z),
                            float(pre_scan_targets[0].pose.orientation.w),
                        )

                    if rt.enable_scan and rt.run_reconstruction:
                        mesh_path, rgb_ply_path = self._run_reconstruction_for_scan(
                            session_path=run_session_arg,
                            scan_folder=cycle_scan_folder,
                            reconstruct_device=rt.reconstruct_device,
                            reconstruct_request_timeout_s=rt.reconstruct_request_timeout_s,
                            wait_reconstruction_outputs=rt.wait_reconstruction_outputs,
                            color_to_depth_wait_timeout_s=rt.color_to_depth_wait_timeout_s,
                            tsdf_wait_timeout_s=rt.tsdf_wait_timeout_s,
                            mesh_prefer=rt.mesh_prefer,
                            mesh_update_wait_timeout_s=rt.mesh_update_wait_timeout_s,
                            mesh_update_request_timeout_s=rt.mesh_update_request_timeout_s,
                            tsdf_center_crop_enable=rt.tsdf_center_crop_enable,
                            tsdf_center_crop_width=rt.tsdf_center_crop_width,
                            tsdf_center_crop_height=rt.tsdf_center_crop_height,
                            tsdf_center_crop_apply_to_depth=rt.tsdf_center_crop_apply_to_depth,
                            tsdf_aabb_crop_enable=rt.tsdf_aabb_crop_enable,
                            tsdf_aabb_crop_min=rt.tsdf_aabb_crop_min,
                            tsdf_aabb_crop_max=rt.tsdf_aabb_crop_max,
                            tsdf_param_update_timeout_s=rt.tsdf_param_update_timeout_s,
                        )
                        if not self._wait_if_control_paused(f"after reconstruction for pre_scan_for_{cycle_tag}"):
                            return
            else:
                bootstrap_tag = "cycle_0000"
                bootstrap_root = self._run_session_child(run_session_path, f"field_loop/{bootstrap_tag}")
                bootstrap_scan_capture_folder = f"{bootstrap_root}/scan"
                bootstrap_scan_folder = f"field_loop/{bootstrap_tag}/scan"
                bootstrap_field_state_output_dir = f"{bootstrap_root}/field_init"

                log.info(
                    f"[print_field_oriented] ===== {bootstrap_tag} (bootstrap) ===== "
                    f"scan_capture='{bootstrap_scan_capture_folder}' "
                    f"scan_folder='{bootstrap_scan_folder}'"
                )

                bootstrap_targets = self._run_configured_scan(
                    cfg=rt.config,
                    scan_type=rt.scan_type,
                    target=None,
                    width=rt.scan_width,
                    height=rt.scan_height,
                    center_x=rt.scan_center_x,
                    center_y=rt.scan_center_y,
                    center_z=rt.scan_center_z,
                    z_off=rt.scan_z_off,
                    nx=rt.scan_nx,
                    ny=rt.scan_ny,
                    row_major=rt.scan_row_major,
                    use_tf_orientation=rt.scan_use_tf_orientation,
                    capture_folder=bootstrap_scan_capture_folder,
                    timeout_s=rt.timeout_s,
                    frame_id=rt.frame_id,
                )
                if not bootstrap_targets:
                    raise RuntimeError("Bootstrap grid sweep produced 0 targets.")
                if not self._wait_if_control_paused("after bootstrap scan"):
                    return

                reference_scan_orientation = (
                    float(bootstrap_targets[0].pose.orientation.x),
                    float(bootstrap_targets[0].pose.orientation.y),
                    float(bootstrap_targets[0].pose.orientation.z),
                    float(bootstrap_targets[0].pose.orientation.w),
                )

                if rt.run_reconstruction:
                    mesh_path, rgb_ply_path = self._run_reconstruction_for_scan(
                        session_path=run_session_arg,
                        scan_folder=bootstrap_scan_folder,
                        reconstruct_device=rt.reconstruct_device,
                        reconstruct_request_timeout_s=rt.reconstruct_request_timeout_s,
                        wait_reconstruction_outputs=rt.wait_reconstruction_outputs,
                        color_to_depth_wait_timeout_s=rt.color_to_depth_wait_timeout_s,
                        tsdf_wait_timeout_s=rt.tsdf_wait_timeout_s,
                        mesh_prefer=rt.mesh_prefer,
                        mesh_update_wait_timeout_s=rt.mesh_update_wait_timeout_s,
                        mesh_update_request_timeout_s=rt.mesh_update_request_timeout_s,
                        tsdf_center_crop_enable=rt.tsdf_center_crop_enable,
                        tsdf_center_crop_width=rt.tsdf_center_crop_width,
                        tsdf_center_crop_height=rt.tsdf_center_crop_height,
                        tsdf_center_crop_apply_to_depth=rt.tsdf_center_crop_apply_to_depth,
                        tsdf_aabb_crop_enable=rt.tsdf_aabb_crop_enable,
                        tsdf_aabb_crop_min=rt.tsdf_aabb_crop_min,
                        tsdf_aabb_crop_max=rt.tsdf_aabb_crop_max,
                        tsdf_param_update_timeout_s=rt.tsdf_param_update_timeout_s,
                    )
                    if not self._wait_if_control_paused("after bootstrap reconstruction"):
                        return

                if rt.run_field_init:
                    self._set_fields_node_positioning_params(
                        fields_node_name=rt.fields_node_name,
                        seed_level=rt.field_seed_level,
                        require_full_hit=rt.field_require_full_hit,
                        base_z_offset=rt.field_base_z_offset,
                        use_position_target=rt.field_use_position_target,
                        position_target_x=rt.field_position_target_x,
                        position_target_y=rt.field_position_target_y,
                        timeout_s=rt.field_position_param_timeout_s,
                    )
                    scan_mesh_for_field = rt.field_scan_mesh_path or mesh_path
                    scan_mesh_paths = [scan_mesh_for_field] if scan_mesh_for_field else []
                    init_res = self.session.run_sync(
                        self.session.field.init_field_from_scan(
                            use_latest=rt.field_use_latest,
                            session_path=rt.field_session_path,
                            scan_mesh_paths=scan_mesh_paths,
                            field_mesh_path=rt.field_mesh_path,
                            state_output_dir=bootstrap_field_state_output_dir,
                            enqueue=False,
                        ),
                        timeout_s=rt.field_request_timeout_s,
                    )
                    if not init_res.get("ok", False):
                        raise RuntimeError(f"init_field_from_scan failed: {init_res.get('error')}")
                    if not self._wait_if_control_paused("after field init"):
                        return

                    metrics = init_res.get("metrics", {})
                    field_state_path = str(metrics.get("field_state_path", "")).strip()
                    if not field_state_path:
                        raise RuntimeError("init_field_from_scan did not return field_state_path")

                    field_center_raw = self.session.compute_field_center_from_state(field_state_path)
                    field_center_xy = self.session.map_field_center_xy(
                        x=float(field_center_raw[0]),
                        y=float(field_center_raw[1]),
                        sign_x=rt.field_center_sign_x,
                        sign_y=rt.field_center_sign_y,
                        offset_x=rt.field_center_offset_x,
                        offset_y=rt.field_center_offset_y,
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
                        f"xy_map=(x*{rt.field_center_sign_x:.3f}+{rt.field_center_offset_x:.3f}, "
                        f"y*{rt.field_center_sign_y:.3f}+{rt.field_center_offset_y:.3f})"
                    )

                    debug_field_ply_path = str(metrics.get("debug_field_ply_path", "")).strip()
                    if debug_field_ply_path:
                        field_vis_res = self.session.run_sync(
                            self.session.camera.update_world_mesh(
                                use_latest=False,
                                session_path=rt.field_session_path or run_session_arg,
                                mesh_path="",
                                ply_path=debug_field_ply_path,
                                prefer="ply",
                                wait_timeout_s=rt.mesh_update_wait_timeout_s,
                                enqueue=False,
                            ),
                            timeout_s=rt.mesh_update_request_timeout_s,
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
                        if not self._wait_if_control_paused("after field PLY publish"):
                            return
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
                        sign_x=rt.field_center_sign_x,
                        sign_y=rt.field_center_sign_y,
                        offset_x=rt.field_center_offset_x,
                        offset_y=rt.field_center_offset_y,
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
                        f"xy_map=(x*{rt.field_center_sign_x:.3f}+{rt.field_center_offset_x:.3f}, "
                        f"y*{rt.field_center_sign_y:.3f}+{rt.field_center_offset_y:.3f})"
                    )

            if field_center_xyz is None:
                raise RuntimeError("Field center not available after bootstrap.")

            if not rt.run_generate_candidates:
                log.warn("[print_field_oriented] run_generate_candidates=False; stopping after bootstrap.")
                return

            # -----------------------------------------------------------------
            # Print cycles:
            # - cycle_0000 candidates come from the bootstrap scan mesh.
            # - after each print, run a centered layer scan for the next cycle.
            # -----------------------------------------------------------------
            print_cycle_index = 0
            while rclpy.ok():
                refresh_runtime_config(
                    f"cycle loop start {first_print_cycle_number + print_cycle_index:04d}"
                )

                if rt.max_cycles > 0 and print_cycle_index >= rt.max_cycles:
                    log.info(f"[print_field_oriented] Reached max_cycles={rt.max_cycles}.")
                    break

                cycle_number = first_print_cycle_number + print_cycle_index
                cycle_tag = f"cycle_{cycle_number:04d}"
                cycle_root = self._run_session_child(run_session_path, f"field_loop/{cycle_tag}")
                cycle_candidate_output_dir = f"{cycle_root}/candidates"

                if not self._wait_if_control_paused(f"before candidate generation for {cycle_tag}"):
                    break

                scan_mesh_for_candidates = rt.candidate_scan_mesh_path or rt.field_scan_mesh_path or mesh_path
                scan_mesh_paths = [scan_mesh_for_candidates] if scan_mesh_for_candidates else []
                field_state_for_candidates = rt.candidate_field_state_path or field_state_path
                if not field_state_for_candidates:
                    raise RuntimeError(
                        "Missing field state for candidate generation. "
                        "Run field init or provide candidate_field_state_path."
                    )

                cand_res = self.session.run_sync(
                        self.session.field.generate_print_candidates(
                            use_latest=rt.candidate_use_latest,
                            session_path=rt.candidate_session_path,
                            field_state_path=field_state_for_candidates,
                            scan_mesh_paths=scan_mesh_paths,
                            output_dir=cycle_candidate_output_dir,
                            candidate_mode=rt.candidate_mode,
                            beads_per_step=rt.candidate_beads_per_step,
                            bead_separation_mm=rt.candidate_bead_separation_mm,
                            bead_height_mm=rt.candidate_bead_height_mm,
                            walk_distance_mm=rt.candidate_walk_distance_mm,
                            walk_step_mm=rt.candidate_walk_step_mm,
                            walk_max_steps=rt.candidate_walk_max_steps,
                            walk_start_fraction=rt.candidate_walk_start_fraction,
                            orient_with_tangent=rt.oriented_targets_enable,
                            tangent_sign=rt.oriented_tangent_sign,
                            clamp_to_cone=rt.oriented_clamp_to_cone,
                            cone_max_tilt_deg=rt.oriented_cone_max_tilt_deg,
                            base_to_world_yaw_deg=rt.oriented_base_to_world_yaw_deg,
                            target_zx=rt.candidate_target_zx,
                            target_zy=rt.candidate_target_zy,
                            target_zz=rt.candidate_target_zz,
                            target_position_scale=rt.candidate_target_position_scale,
                            enqueue=False,
                    ),
                    timeout_s=rt.candidate_request_timeout_s,
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
                if not self._wait_if_control_paused(f"after candidate generation for {cycle_tag}"):
                    break

                targets_yaml_path = str(cand_metrics.get("targets_yaml_path", "")).strip()
                preview_targets: List[PoseStamped] = []
                preview_segments = []
                if targets_yaml_path:
                    preview_segments = self.print_session.parse_yaml_segments(
                        yaml_path=targets_yaml_path,
                        frame_id=rt.frame_id,
                    )
                    preview_targets = self.print_session.parse_yaml_targets(
                        yaml_path=targets_yaml_path,
                        frame_id=rt.frame_id,
                    )

                if rt.candidate_visualize_field_ply:
                    cycle_field_ply = str(cand_metrics.get("debug_field_ply_path", "")).strip()
                    if cycle_field_ply:
                        cycle_vis_res = self.session.run_sync(
                            self.session.camera.preview_field_ply(
                                use_latest=False,
                                session_path=rt.candidate_session_path or run_session_arg,
                                field_ply_path=cycle_field_ply,
                                restore_mesh_path=(
                                    scan_mesh_for_candidates if rt.candidate_restore_scan_mesh_after_field_ply else ""
                                ),
                                wait_timeout_s=rt.mesh_update_wait_timeout_s,
                                enqueue=False,
                            ),
                            timeout_s=rt.mesh_update_request_timeout_s,
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

                        if rt.candidate_restore_scan_mesh_after_field_ply and scan_mesh_for_candidates:
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

                if rt.candidate_publish_markers and preview_targets:
                    marker_res = self.session.run_sync(
                        self.session.util.publish_targets(
                            preview_targets,
                            axis_length=float(rt.candidate_marker_axis_length),
                            axis_radius=float(rt.candidate_marker_axis_radius),
                            clear_before=bool(rt.candidate_marker_clear_before),
                            enqueue=False,
                        ),
                        timeout_s=rt.mesh_update_request_timeout_s,
                    )
                    if not marker_res.get("ok", False):
                        raise RuntimeError(
                            f"publish_targets(candidates) failed: {marker_res.get('error')}"
                        )
                    log.info(
                        f"[print_field_oriented] Candidate markers published: {len(preview_targets)} targets "
                        f"(yaml='{targets_yaml_path}', frame='{rt.frame_id}')"
                    )
                    if not self._wait_if_control_paused(f"after candidate markers for {cycle_tag}"):
                        break

                if not preview_targets:
                    log.warn("[print_field_oriented] No candidate targets generated; stopping loop.")
                    break

                refresh_runtime_config(f"before print prompt for {cycle_tag}")
                if not self._wait_if_control_paused(f"before print for {cycle_tag}"):
                    break

                if rt.debug_mode or rt.prompt_before_print:
                    gate_res = self.session.run_sync(
                        self.session.util.input(
                            prompt=(
                                f"[print_field_oriented:{cycle_tag}] Debug gate: press ENTER to print targets "
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

                print_mode_norm = str(rt.print_mode or "auto").strip().lower()
                if print_mode_norm not in ("auto", "segments", "dots"):
                    raise RuntimeError("print_mode must be one of: auto, segments, dots")

                use_segments = bool(preview_segments) if print_mode_norm == "auto" else (print_mode_norm == "segments")
                if use_segments and not preview_segments:
                    raise RuntimeError("print_mode=segments but generated YAML has no segments.")

                if use_segments:
                    log.info(
                        f"[print_field_oriented] Printing {len(preview_segments)} segments "
                        f"(print_mode={print_mode_norm})."
                    )
                    print_res = self.print_session.run_print_segments(
                        segments=preview_segments,
                        print_speed=rt.segment_print_speed,
                        approach_z_offset_m=rt.segment_approach_z_offset_m,
                        travel_z_offset_m=rt.segment_travel_z_offset_m,
                        approach_vel_scale=rt.segment_approach_vel_scale,
                        travel_vel_scale=rt.segment_travel_vel_scale,
                        print_vel_scale=rt.segment_print_vel_scale,
                        accel_scale=rt.segment_accel_scale,
                        target_print_speed_mm_s=rt.segment_target_print_speed_mm_s,
                        post_segment_wait_s=rt.post_segment_wait_s,
                        post_segment_retract_s=rt.post_segment_retract_s,
                        post_segment_retract_speed=rt.post_segment_retract_speed,
                        timeout_s=rt.timeout_s,
                    )
                else:
                    log.info(
                        f"[print_field_oriented] Printing {len(preview_targets)} dots "
                        f"(print_mode={print_mode_norm}, dot_steps={rt.dot_steps}, "
                        f"retract_steps={rt.post_dot_retract_steps})."
                    )
                    print_res = self.print_session.run_print_dots_targets(
                        targets=preview_targets,
                        dot_steps=rt.dot_steps,
                        dot_speed=rt.dot_speed,
                        approach_z_offset_m=rt.dot_approach_z_offset_m,
                        dot_z_offset_m=rt.dot_z_offset_m,
                        pre_dot_vel_scale=rt.dot_pre_vel_scale,
                        dot_vel_scale=rt.dot_print_vel_scale,
                        accel_scale=rt.dot_accel_scale,
                        dwell_s=rt.dot_dwell_s,
                        post_dot_retract_steps=rt.post_dot_retract_steps,
                        post_dot_retract_speed=rt.post_dot_retract_speed,
                        eef_link=rt.dot_eef_link,
                        timeout_s=rt.timeout_s,
                    )
                if not print_res.get("ok", False):
                    log.warn(
                        "[print_field_oriented] Print stage returned non-fatal failure: "
                        f"stage={print_res.get('stage')} error={print_res.get('error', 'unknown')}"
                    )
                    break
                if not self._wait_if_control_paused(f"after print for {cycle_tag}"):
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
                last_generated_targets = list(preview_targets)

                print_cycle_index += 1

                refresh_runtime_config(f"before scan prompt after {cycle_tag}")

                if rt.max_cycles > 0 and print_cycle_index >= rt.max_cycles:
                    log.info(f"[print_field_oriented] Reached max_cycles={rt.max_cycles}.")
                    break

                if rt.prompt_before_scan and rt.prompt_before_next_cycle:
                    next_res = self.session.run_sync(
                        self.session.util.input(
                            prompt=(
                                f"[print_field_oriented:{cycle_tag}] Press ENTER for next cycle "
                                f"(cycle_{first_print_cycle_number + print_cycle_index:04d}) "
                                "or type 'q' + ENTER to stop."
                            ),
                            enqueue=False,
                        ),
                        timeout_s=None,
                    )
                    next_value = str(next_res.get("metrics", {}).get("value", "")).strip().lower()
                    if next_value == "q":
                        log.info("[print_field_oriented] Stopped by user before next cycle.")
                        break

                next_cycle_number = first_print_cycle_number + print_cycle_index
                next_cycle_tag = f"cycle_{next_cycle_number:04d}"
                next_cycle_root = self._run_session_child(run_session_path, f"field_loop/{next_cycle_tag}")
                next_cycle_scan_capture_folder = f"{next_cycle_root}/scan"
                next_cycle_scan_folder = f"field_loop/{next_cycle_tag}/scan"

                if last_generated_targets:
                    highest_prev_z = max(float(t.pose.position.z) for t in last_generated_targets)
                    abs_scan_z = highest_prev_z + float(rt.grid_z_off)
                    cycle_scan_z_off = abs_scan_z - float(rt.grid_center_z)
                    z_msg = (
                        f"highest_generated_z={highest_prev_z:.4f}m + grid_z_off={rt.grid_z_off:.4f}m "
                        f"=> abs_scan_z={abs_scan_z:.4f}m, "
                        f"center_z={rt.grid_center_z:.4f}m => applied_z_off={cycle_scan_z_off:.4f}m"
                    )
                else:
                    cycle_scan_z_off = float(rt.grid_z_off)
                    abs_scan_z = float(rt.grid_center_z) + float(cycle_scan_z_off)
                    z_msg = (
                        f"fallback(no printed targets): z_off={cycle_scan_z_off:.4f}m "
                        f"=> abs_scan_z={abs_scan_z:.4f}m"
                    )

                log.info(
                    f"[print_field_oriented] ===== scan_for_{next_cycle_tag} ===== "
                    f"scan_capture='{next_cycle_scan_capture_folder}' "
                    f"scan_folder='{next_cycle_scan_folder}' "
                    f"center=({rt.grid_center_x:.4f}, {rt.grid_center_y:.4f}, {rt.grid_center_z:.4f}) "
                    f"grid={rt.grid_nx}x{rt.grid_ny} size={1000.0*rt.grid_width:.1f}x"
                    f"{1000.0*rt.grid_height:.1f}mm use_tf_orientation={rt.grid_use_tf_orientation} {z_msg}"
                )

                if not rt.enable_scan:
                    log.info(
                        f"[print_field_oriented] scan_for_{next_cycle_tag} skipped because enable_scan=false."
                    )
                    scan_targets = []
                else:
                    if not self._prompt_before_scan_if_enabled(
                        enabled=rt.prompt_before_scan,
                        label=f"scan_for_{next_cycle_tag}",
                    ):
                        log.info("[print_field_oriented] Stopped by user before scan.")
                        break

                    scan_targets = self._run_configured_scan(
                        cfg=rt.config,
                        scan_type=rt.scan_type,
                        target=None,
                        width=rt.grid_width,
                        height=rt.grid_height,
                        center_x=float(rt.grid_center_x),
                        center_y=float(rt.grid_center_y),
                        center_z=float(rt.grid_center_z),
                        z_off=cycle_scan_z_off,
                        nx=rt.grid_nx,
                        ny=rt.grid_ny,
                        row_major=rt.grid_row_major,
                        use_tf_orientation=rt.grid_use_tf_orientation,
                        capture_folder=next_cycle_scan_capture_folder,
                        timeout_s=rt.timeout_s,
                        frame_id=rt.frame_id,
                    )
                    if not scan_targets:
                        raise RuntimeError(f"Scan produced 0 targets for scan_for_{next_cycle_tag}.")
                    if not self._wait_if_control_paused(f"after scan_for_{next_cycle_tag}"):
                        break

                if rt.enable_scan and rt.run_reconstruction:
                    mesh_path, rgb_ply_path = self._run_reconstruction_for_scan(
                        session_path=run_session_arg,
                        scan_folder=next_cycle_scan_folder,
                        reconstruct_device=rt.reconstruct_device,
                        reconstruct_request_timeout_s=rt.reconstruct_request_timeout_s,
                        wait_reconstruction_outputs=rt.wait_reconstruction_outputs,
                        color_to_depth_wait_timeout_s=rt.color_to_depth_wait_timeout_s,
                        tsdf_wait_timeout_s=rt.tsdf_wait_timeout_s,
                        mesh_prefer=rt.mesh_prefer,
                        mesh_update_wait_timeout_s=rt.mesh_update_wait_timeout_s,
                        mesh_update_request_timeout_s=rt.mesh_update_request_timeout_s,
                        tsdf_center_crop_enable=rt.tsdf_center_crop_enable,
                        tsdf_center_crop_width=rt.tsdf_center_crop_width,
                        tsdf_center_crop_height=rt.tsdf_center_crop_height,
                        tsdf_center_crop_apply_to_depth=rt.tsdf_center_crop_apply_to_depth,
                        tsdf_aabb_crop_enable=rt.tsdf_aabb_crop_enable,
                        tsdf_aabb_crop_min=rt.tsdf_aabb_crop_min,
                        tsdf_aabb_crop_max=rt.tsdf_aabb_crop_max,
                        tsdf_param_update_timeout_s=rt.tsdf_param_update_timeout_s,
                    )
                    if not self._wait_if_control_paused(f"after reconstruction for scan_for_{next_cycle_tag}"):
                        break
                else:
                    log.warn(
                        "[print_field_oriented] run_reconstruction=False after scan; "
                        "next cycle will reuse previous scan mesh unless candidate_scan_mesh_path is set."
                    )

            if rt.home_after:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=rt.timeout_s,
                )
                self._wait_if_control_paused("after home_after")

        except TimeoutError:
            log.error("[print_field_oriented] Sequence timed out.")
        except Exception as exc:
            log.error(f"[print_field_oriented] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _run_configured_scan(
        self,
        *,
        cfg: Dict[str, Any],
        scan_type: str,
        target: Optional[PoseStamped],
        width: float,
        height: float,
        center_x: float,
        center_y: float,
        center_z: float,
        z_off: float,
        nx: int,
        ny: int,
        row_major: bool,
        use_tf_orientation: bool,
        capture_folder: str,
        timeout_s: Optional[float],
        frame_id: str,
    ) -> List[PoseStamped]:
        scan_type_norm = str(scan_type or "grid_sweep").strip().lower()
        common = self._scan_common_kwargs(cfg=cfg, timeout_s=timeout_s)

        if scan_type_norm in ("grid", "grid_sweep"):
            res = self.scan_session.run_grid_scan(
                target=target,
                capture_folder=capture_folder,
                width=float(width),
                height=float(height),
                center_x=float(center_x),
                center_y=float(center_y),
                center_z=float(center_z),
                z_off=float(z_off),
                nx=int(nx),
                ny=int(ny),
                row_major=bool(row_major),
                frame_id=str(frame_id or "world"),
                use_tf_orientation=bool(use_tf_orientation),
                **common,
            )
        elif scan_type_norm in ("half_cylinder", "half-cylinder", "half"):
            orientation_mode = self._cfg_str(cfg, "half_orientation_mode")
            orientation_pose = None
            if orientation_mode.strip().lower() in (
                "fixed",
                "current",
                "current_eef",
                "look_at_current_roll",
                "look_at_keep_roll",
                "look_at_min_roll",
            ):
                orientation_pose = self._current_eef_pose(cfg=cfg, frame_id=frame_id, timeout_s=timeout_s)

            axis_start_xyz = None
            axis_end_xyz = None
            n_axis = None
            if self._cfg_bool(cfg, "half_use_line_axis"):
                axis_start_xyz = (
                    self._cfg_float(cfg, "half_axis_start_x"),
                    self._cfg_float(cfg, "half_axis_start_y"),
                    self._cfg_float(cfg, "half_axis_start_z"),
                )
                axis_end_xyz = (
                    self._cfg_float(cfg, "half_axis_end_x"),
                    self._cfg_float(cfg, "half_axis_end_y"),
                    self._cfg_float(cfg, "half_axis_end_z"),
                )
                n_axis = self._cfg_int(cfg, "half_n_axis")

            res = self.scan_session.run_half_cylinder_scan(
                capture_folder=capture_folder,
                center_x=self._cfg_float(cfg, "half_center_x"),
                center_y=self._cfg_float(cfg, "half_center_y"),
                center_z=self._cfg_float(cfg, "half_center_z"),
                radius=self._cfg_float(cfg, "half_radius"),
                height=self._cfg_float(cfg, "half_height"),
                angle_min_deg=self._cfg_float(cfg, "half_angle_min_deg"),
                angle_max_deg=self._cfg_float(cfg, "half_angle_max_deg"),
                n_angle=self._cfg_int(cfg, "half_n_angle"),
                n_height=self._cfg_int(cfg, "half_n_height"),
                frame_id=str(frame_id or "world"),
                row_major=self._cfg_bool(cfg, "half_row_major"),
                orientation_mode=orientation_mode,
                orientation_pose=orientation_pose,
                axis_start_xyz=axis_start_xyz,
                axis_end_xyz=axis_end_xyz,
                n_axis=n_axis,
                arc_center_direction=(
                    self._cfg_float(cfg, "half_arc_center_dx"),
                    self._cfg_float(cfg, "half_arc_center_dy"),
                    self._cfg_float(cfg, "half_arc_center_dz"),
                ),
                roll_deg=self._cfg_float(cfg, "half_roll_deg"),
                **common,
            )
        elif scan_type_norm in (
            "half_cylinder_side_caps",
            "half-cylinder-side-caps",
            "half_cylinder_caps",
            "half-cylinder-caps",
            "side_caps",
        ):
            if not self._cfg_bool(cfg, "half_use_line_axis"):
                raise ValueError("half_cylinder_side_caps requires half_use_line_axis=true")
            orientation_mode = self._cfg_str(cfg, "half_orientation_mode")
            orientation_pose = None
            if orientation_mode.strip().lower() in (
                "fixed",
                "current",
                "current_eef",
                "look_at_current_roll",
                "look_at_keep_roll",
                "look_at_min_roll",
            ):
                orientation_pose = self._current_eef_pose(cfg=cfg, frame_id=frame_id, timeout_s=timeout_s)

            endcap_radius = self._cfg_float(cfg, "endcap_radius")
            endcap_n_angle = self._cfg_int(cfg, "endcap_n_angle")
            res = self.scan_session.run_half_cylinder_side_caps_scan(
                capture_folder=capture_folder,
                axis_start_xyz=(
                    self._cfg_float(cfg, "half_axis_start_x"),
                    self._cfg_float(cfg, "half_axis_start_y"),
                    self._cfg_float(cfg, "half_axis_start_z"),
                ),
                axis_end_xyz=(
                    self._cfg_float(cfg, "half_axis_end_x"),
                    self._cfg_float(cfg, "half_axis_end_y"),
                    self._cfg_float(cfg, "half_axis_end_z"),
                ),
                radius=self._cfg_float(cfg, "half_radius"),
                angle_min_deg=self._cfg_float(cfg, "half_angle_min_deg"),
                angle_max_deg=self._cfg_float(cfg, "half_angle_max_deg"),
                n_angle=self._cfg_int(cfg, "half_n_angle"),
                n_axis=self._cfg_int(cfg, "half_n_axis"),
                frame_id=str(frame_id or "world"),
                row_major=self._cfg_bool(cfg, "half_row_major"),
                orientation_mode=orientation_mode,
                orientation_pose=orientation_pose,
                arc_center_direction=(
                    self._cfg_float(cfg, "half_arc_center_dx"),
                    self._cfg_float(cfg, "half_arc_center_dy"),
                    self._cfg_float(cfg, "half_arc_center_dz"),
                ),
                roll_deg=self._cfg_float(cfg, "half_roll_deg"),
                endcap_radius=None if endcap_radius <= 0.0 else endcap_radius,
                endcap_angle_min_deg=self._cfg_float(cfg, "endcap_angle_min_deg"),
                endcap_angle_max_deg=self._cfg_float(cfg, "endcap_angle_max_deg"),
                endcap_n_angle=None if endcap_n_angle <= 0 else endcap_n_angle,
                endcap_polar_min_deg=self._cfg_float(cfg, "endcap_polar_min_deg"),
                endcap_polar_max_deg=self._cfg_float(cfg, "endcap_polar_max_deg"),
                endcap_n_polar=self._cfg_int(cfg, "endcap_n_polar"),
                endcap_include_start=self._cfg_bool(cfg, "endcap_include_start"),
                endcap_include_end=self._cfg_bool(cfg, "endcap_include_end"),
                endcap_row_major=self._cfg_bool(cfg, "endcap_row_major"),
                **common,
            )
        else:
            raise ValueError("scan_type must be one of: grid_sweep, half_cylinder, half_cylinder_side_caps")

        if not res.get("ok", False):
            raise RuntimeError(
                f"{scan_type_norm} scan failed at stage={res.get('stage')}: "
                f"{res.get('error', 'unknown')}"
            )

        targets = list(res.get("targets", []))
        self.get_logger().info(
            f"[print_field_oriented] Scan complete: type={scan_type_norm}, "
            f"targets={len(targets)}, planned={res.get('plan_ok', 0)}, "
            f"executed={res.get('exec_ok', 0)}, captures={res.get('captures_ok', 0)}"
        )
        return targets

    def _prompt_before_scan_if_enabled(self, *, enabled: bool, label: str) -> bool:
        if not bool(enabled):
            return True
        res = self.session.run_sync(
            self.session.util.input(
                prompt=(
                    f"[print_field_oriented:{label}] Press ENTER to scan "
                    "(type 'q' + ENTER to stop)."
                ),
                enqueue=False,
            ),
            timeout_s=None,
        )
        value = str(res.get("metrics", {}).get("value", "")).strip().lower()
        return value != "q"

    def _scan_common_kwargs(self, *, cfg: Dict[str, Any], timeout_s: Optional[float]) -> Dict[str, Any]:
        return {
            "motion": self._cfg_str(cfg, "scan_motion"),
            "eef_link": self._cfg_str(cfg, "scan_eef_link"),
            "do_home": False,
            "vel_scale": self._cfg_float(cfg, "scan_vel_scale"),
            "accel_scale": self._cfg_float(cfg, "scan_accel_scale"),
            "timeout_s": timeout_s,
            "settle_s": self._cfg_float(cfg, "scan_settle_s"),
            "prompt": self._cfg_str_allow_empty(cfg, "scan_prompt") or None,
            "debug": self._cfg_bool(cfg, "scan_debug") or self._cfg_bool(cfg, "scan_debug_prompt"),
            "rgb": self._cfg_bool(cfg, "scan_rgb"),
            "depth": self._cfg_bool(cfg, "scan_depth"),
            "ir": self._cfg_bool(cfg, "scan_ir"),
            "pose": self._cfg_bool(cfg, "scan_pose"),
            "publish_markers": self._cfg_bool(cfg, "scan_publish_markers"),
            "axis_length": self._cfg_float(cfg, "scan_axis_length"),
            "axis_radius": self._cfg_float(cfg, "scan_axis_radius"),
            "clear_markers_before": self._cfg_bool(cfg, "scan_clear_markers_before"),
            "clear_markers_after": self._cfg_bool(cfg, "scan_clear_markers_after"),
        }

    def _run_reconstruction_for_scan(
        self,
        *,
        session_path: str,
        scan_folder: str,
        reconstruct_device: str,
        reconstruct_request_timeout_s: float,
        wait_reconstruction_outputs: bool,
        color_to_depth_wait_timeout_s: float,
        tsdf_wait_timeout_s: float,
        mesh_prefer: str,
        mesh_update_wait_timeout_s: float,
        mesh_update_request_timeout_s: float,
        tsdf_center_crop_enable: bool,
        tsdf_center_crop_width: int,
        tsdf_center_crop_height: int,
        tsdf_center_crop_apply_to_depth: bool,
        tsdf_aabb_crop_enable: bool,
        tsdf_aabb_crop_min: List[float],
        tsdf_aabb_crop_max: List[float],
        tsdf_param_update_timeout_s: float,
    ) -> tuple[str, str]:
        rec = self.scan_session.run_reconstruction_for_scan(
            scan_folder=scan_folder,
            session_path=session_path,
            reconstruct_device=reconstruct_device,
            reconstruct_request_timeout_s=reconstruct_request_timeout_s,
            wait_reconstruction_outputs=wait_reconstruction_outputs,
            color_to_depth_wait_timeout_s=color_to_depth_wait_timeout_s,
            tsdf_wait_timeout_s=tsdf_wait_timeout_s,
            mesh_prefer=mesh_prefer,
            mesh_update_wait_timeout_s=mesh_update_wait_timeout_s,
            mesh_update_request_timeout_s=mesh_update_request_timeout_s,
            update_world_mesh=True,
            tsdf_center_crop_enable=tsdf_center_crop_enable,
            tsdf_center_crop_width=tsdf_center_crop_width,
            tsdf_center_crop_height=tsdf_center_crop_height,
            tsdf_center_crop_apply_to_depth=tsdf_center_crop_apply_to_depth,
            tsdf_aabb_crop_enable=tsdf_aabb_crop_enable,
            tsdf_aabb_crop_min=tsdf_aabb_crop_min,
            tsdf_aabb_crop_max=tsdf_aabb_crop_max,
            tsdf_param_update_timeout_s=tsdf_param_update_timeout_s,
        )
        if not rec.get("ok", False):
            raise RuntimeError(
                f"reconstruction failed at stage={rec.get('stage')}: {rec.get('error', 'unknown')}"
            )
        return str(rec.get("mesh_path", "")).strip(), str(rec.get("rgb_ply_path", "")).strip()

    def _set_fields_node_positioning_params(
        self,
        *,
        fields_node_name: str,
        seed_level: float,
        require_full_hit: bool,
        base_z_offset: float,
        use_position_target: bool,
        position_target_x: float,
        position_target_y: float,
        timeout_s: float,
    ) -> None:
        node_name = str(fields_node_name or "/behav3d_fields").strip()
        if not node_name.startswith("/"):
            node_name = f"/{node_name}"
        timeout = max(0.1, float(timeout_s))
        client = AsyncParameterClient(self, node_name)
        if hasattr(client, "wait_for_services"):
            ready = bool(client.wait_for_services(timeout_sec=timeout))
        elif hasattr(client, "wait_for_service"):
            ready = bool(client.wait_for_service(timeout_sec=timeout))
        else:
            ready = bool(client.services_are_ready()) if hasattr(client, "services_are_ready") else False
        if not ready:
            raise RuntimeError(f"Parameter service for {node_name} not available.")

        params = [
            Parameter("seed_level", value=float(seed_level)),
            Parameter("require_full_hit", value=bool(require_full_hit)),
            Parameter("base_z_offset", value=float(base_z_offset)),
            Parameter("use_position_target", value=bool(use_position_target)),
            Parameter("position_target_x", value=float(position_target_x)),
            Parameter("position_target_y", value=float(position_target_y)),
        ]
        fut = client.set_parameters(params)
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            raise TimeoutError(f"Timed out while setting field positioning params on {node_name}.")

        response = fut.result()
        if response is None:
            raise RuntimeError(f"Failed to set field positioning params on {node_name}: no response.")
        if hasattr(response, "results"):
            results = list(response.results)
        elif isinstance(response, (list, tuple)):
            results = list(response)
        else:
            raise RuntimeError(
                "Failed to set field positioning params: unexpected response type "
                f"{type(response).__name__}"
            )
        failed = [str(r.reason) for r in results if not bool(getattr(r, "successful", False))]
        if failed:
            raise RuntimeError(f"Failed to set field positioning params: {'; '.join(failed)}")

        self.get_logger().info(
            "[print_field_oriented] Field positioning params set: "
            f"node={node_name} use_target={bool(use_position_target)} "
            f"seed_level={float(seed_level):.6f} "
            f"require_full_hit={bool(require_full_hit)} "
            f"base_z_offset={float(base_z_offset):.6f} "
            f"target=({float(position_target_x):.6f}, {float(position_target_y):.6f})"
        )

    def _current_eef_pose(self, *, cfg: Dict[str, Any], frame_id: str, timeout_s: Optional[float]):
        res = self.scan_session.run_sync(
            self.scan_session.camera.get_pose(
                eef=self._cfg_str(cfg, "scan_eef_link"),
                base_frame=str(frame_id or "world"),
                use_tf=True,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if res.get("ok", False) and "pose" in res:
            return res["pose"]
        self.get_logger().warn("[print_field_oriented] Current EEF pose unavailable for scan orientation.")
        return None

    def _parse_runtime_state(self, cfg: Dict[str, Any], run_session_path: Path) -> SimpleNamespace:
        home_before_scan = self._cfg_bool(cfg, "home_before_scan")
        home_after = self._cfg_bool(cfg, "home_after")
        timeout_s = self._cfg_optional_timeout(cfg, "timeout_s")
        debug_mode = self._cfg_bool(cfg, "debug_mode")
        prompt_before_print = self._cfg_bool(cfg, "prompt_before_print")
        prompt_before_scan = self._cfg_bool(cfg, "prompt_before_scan")
        enable_scan = self._cfg_bool(cfg, "enable_scan")

        frame_id = self._cfg_str(cfg, "frame_id")
        scan_eef_link = self._cfg_str(cfg, "scan_eef_link")
        scan_type = self._cfg_str(cfg, "scan_type")
        scan_motion = self._cfg_str(cfg, "scan_motion")
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
        scan_debug = self._cfg_bool(cfg, "scan_debug")
        scan_vel_scale = self._cfg_float(cfg, "scan_vel_scale")
        scan_accel_scale = self._cfg_float(cfg, "scan_accel_scale")
        scan_settle_s = self._cfg_float(cfg, "scan_settle_s")
        scan_prompt = self._cfg_str_allow_empty(cfg, "scan_prompt")
        scan_rgb = self._cfg_bool(cfg, "scan_rgb")
        scan_depth = self._cfg_bool(cfg, "scan_depth")
        scan_ir = self._cfg_bool(cfg, "scan_ir")
        scan_pose = self._cfg_bool(cfg, "scan_pose")
        scan_publish_markers = self._cfg_bool(cfg, "scan_publish_markers")
        scan_axis_length = self._cfg_float(cfg, "scan_axis_length")
        scan_axis_radius = self._cfg_float(cfg, "scan_axis_radius")
        scan_clear_markers_before = self._cfg_bool(cfg, "scan_clear_markers_before")
        scan_clear_markers_after = self._cfg_bool(cfg, "scan_clear_markers_after")

        grid_width = self._cfg_float(cfg, "grid_width")
        grid_height = self._cfg_float(cfg, "grid_height")
        grid_center_x = self._cfg_float(cfg, "grid_center_x")
        grid_center_y = self._cfg_float(cfg, "grid_center_y")
        grid_center_z = self._cfg_float(cfg, "grid_center_z")
        grid_z_off = self._cfg_float(cfg, "grid_z_off")
        grid_nx = self._cfg_int(cfg, "grid_nx")
        grid_ny = self._cfg_int(cfg, "grid_ny")
        grid_row_major = self._cfg_bool(cfg, "grid_row_major")
        grid_use_tf_orientation = self._cfg_bool(cfg, "grid_use_tf_orientation")
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
        field_session_path = self._resolve_run_session_token(self._cfg_str(cfg, "field_session_path"), run_session_path)
        field_scan_mesh_path = self._cfg_str_allow_empty(cfg, "field_scan_mesh_path")
        field_mesh_path = self._cfg_str(cfg, "field_mesh_path")
        field_state_output_dir = self._resolve_run_session_token(self._cfg_str(cfg, "field_state_output_dir"), run_session_path)
        field_request_timeout_s = self._cfg_float(cfg, "field_request_timeout_s")
        fields_node_name = self._cfg_str(cfg, "fields_node_name")
        field_position_param_timeout_s = self._cfg_float(cfg, "field_position_param_timeout_s")
        field_seed_level = self._cfg_float(cfg, "field_seed_level")
        field_require_full_hit = self._cfg_bool(cfg, "field_require_full_hit")
        field_base_z_offset = self._cfg_float(cfg, "field_base_z_offset")
        field_use_position_target = self._cfg_bool(cfg, "field_use_position_target")
        field_position_target_x = self._cfg_float(cfg, "field_position_target_x")
        field_position_target_y = self._cfg_float(cfg, "field_position_target_y")
        skip_bootstrap_scan_and_init = self._cfg_bool(cfg, "skip_bootstrap_scan_and_init")
        resume_field_state_path = self._cfg_str_allow_empty(cfg, "resume_field_state_path")
        resume_scan_mesh_path = self._cfg_str_allow_empty(cfg, "resume_scan_mesh_path")
        scan_before_generate_first_cycle = self._cfg_bool(cfg, "scan_before_generate_first_cycle")

        run_generate_candidates = self._cfg_bool(cfg, "run_generate_candidates")
        candidate_use_latest = self._cfg_bool(cfg, "candidate_use_latest")
        candidate_session_path = self._resolve_run_session_token(self._cfg_str(cfg, "candidate_session_path"), run_session_path)
        candidate_field_state_path = self._cfg_str_allow_empty(cfg, "candidate_field_state_path")
        candidate_scan_mesh_path = self._cfg_str_allow_empty(cfg, "candidate_scan_mesh_path")
        candidate_output_dir = self._resolve_run_session_token(self._cfg_str(cfg, "candidate_output_dir"), run_session_path)
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
        print_mode = self._cfg_str(cfg, "print_mode")
        dot_steps = self._cfg_int(cfg, "dot_steps")
        dot_speed = self._cfg_int(cfg, "dot_speed")
        dot_approach_z_offset_m = self._cfg_float(cfg, "dot_approach_z_offset_m")
        dot_z_offset_m = self._cfg_float(cfg, "dot_z_offset_m")
        dot_pre_vel_scale = self._cfg_float(cfg, "dot_pre_vel_scale")
        dot_print_vel_scale = self._cfg_float(cfg, "dot_print_vel_scale")
        dot_accel_scale = self._cfg_float(cfg, "dot_accel_scale")
        dot_dwell_s = self._cfg_float(cfg, "dot_dwell_s")
        post_dot_retract_steps = self._cfg_int(cfg, "post_dot_retract_steps")
        post_dot_retract_speed = self._cfg_int(cfg, "post_dot_retract_speed")
        dot_eef_link = self._cfg_str(cfg, "dot_eef_link")
        segment_print_speed = self._cfg_int(cfg, "segment_print_speed")
        segment_approach_z_offset_m = self._cfg_float(cfg, "segment_approach_z_offset_m")
        segment_travel_z_offset_m = self._cfg_float(cfg, "segment_travel_z_offset_m")
        segment_approach_vel_scale = self._cfg_float(cfg, "segment_approach_vel_scale")
        segment_travel_vel_scale = self._cfg_float(cfg, "segment_travel_vel_scale")
        segment_print_vel_scale = self._cfg_float(cfg, "segment_print_vel_scale")
        segment_accel_scale = self._cfg_float(cfg, "segment_accel_scale")
        segment_target_print_speed_mm_s = self._cfg_float(cfg, "segment_target_print_speed_mm_s")
        post_segment_wait_s = self._cfg_float(cfg, "post_segment_wait_s")
        post_segment_retract_s = self._cfg_float(cfg, "post_segment_retract_s")
        post_segment_retract_speed = self._cfg_int(cfg, "post_segment_retract_speed")
        max_cycles = self._cfg_int(cfg, "max_cycles")
        prompt_before_next_cycle = self._cfg_bool(cfg, "prompt_before_next_cycle")
        scope = locals().copy()
        values = {
            name: scope[name]
            for name in (
            'home_before_scan',
            'home_after',
            'timeout_s',
            'debug_mode',
            'prompt_before_print',
            'prompt_before_scan',
            'enable_scan',
            'frame_id',
            'scan_eef_link',
            'scan_type',
            'scan_motion',
            'scan_use_tf_orientation',
            'scan_width',
            'scan_height',
            'scan_center_x',
            'scan_center_y',
            'scan_center_z',
            'scan_z_off',
            'scan_nx',
            'scan_ny',
            'scan_row_major',
            'scan_capture_folder',
            'scan_debug_prompt',
            'scan_debug',
            'scan_vel_scale',
            'scan_accel_scale',
            'scan_settle_s',
            'scan_prompt',
            'scan_rgb',
            'scan_depth',
            'scan_ir',
            'scan_pose',
            'scan_publish_markers',
            'scan_axis_length',
            'scan_axis_radius',
            'scan_clear_markers_before',
            'scan_clear_markers_after',
            'grid_width',
            'grid_height',
            'grid_center_x',
            'grid_center_y',
            'grid_center_z',
            'grid_z_off',
            'grid_nx',
            'grid_ny',
            'grid_row_major',
            'grid_use_tf_orientation',
            'field_center_sign_x',
            'field_center_sign_y',
            'field_center_offset_x',
            'field_center_offset_y',
            'run_reconstruction',
            'reconstruct_device',
            'reconstruct_request_timeout_s',
            'wait_reconstruction_outputs',
            'color_to_depth_wait_timeout_s',
            'tsdf_wait_timeout_s',
            'mesh_prefer',
            'mesh_update_wait_timeout_s',
            'mesh_update_request_timeout_s',
            'tsdf_center_crop_enable',
            'tsdf_center_crop_width',
            'tsdf_center_crop_height',
            'tsdf_center_crop_apply_to_depth',
            'tsdf_aabb_crop_enable',
            'tsdf_aabb_crop_min',
            'tsdf_aabb_crop_max',
            'tsdf_param_update_timeout_s',
            'run_field_init',
            'field_use_latest',
            'field_session_path',
            'field_scan_mesh_path',
            'field_mesh_path',
            'field_state_output_dir',
            'field_request_timeout_s',
            'fields_node_name',
            'field_position_param_timeout_s',
            'field_seed_level',
            'field_require_full_hit',
            'field_base_z_offset',
            'field_use_position_target',
            'field_position_target_x',
            'field_position_target_y',
            'skip_bootstrap_scan_and_init',
            'resume_field_state_path',
            'resume_scan_mesh_path',
            'scan_before_generate_first_cycle',
            'run_generate_candidates',
            'candidate_use_latest',
            'candidate_session_path',
            'candidate_field_state_path',
            'candidate_scan_mesh_path',
            'candidate_output_dir',
            'candidate_request_timeout_s',
            'candidate_mode',
            'candidate_beads_per_step',
            'candidate_bead_separation_mm',
            'candidate_bead_height_mm',
            'candidate_walk_distance_mm',
            'candidate_walk_step_mm',
            'candidate_walk_max_steps',
            'candidate_walk_start_fraction',
            'candidate_target_zx',
            'candidate_target_zy',
            'candidate_target_zz',
            'candidate_target_position_scale',
            'candidate_visualize_field_ply',
            'candidate_restore_scan_mesh_after_field_ply',
            'candidate_publish_markers',
            'candidate_marker_axis_length',
            'candidate_marker_axis_radius',
            'candidate_marker_clear_before',
            'oriented_targets_enable',
            'oriented_tangent_sign',
            'oriented_clamp_to_cone',
            'oriented_cone_max_tilt_deg',
            'oriented_base_to_world_yaw_deg',
            'print_mode',
            'dot_steps',
            'dot_speed',
            'dot_approach_z_offset_m',
            'dot_z_offset_m',
            'dot_pre_vel_scale',
            'dot_print_vel_scale',
            'dot_accel_scale',
            'dot_dwell_s',
            'post_dot_retract_steps',
            'post_dot_retract_speed',
            'dot_eef_link',
            'segment_print_speed',
            'segment_approach_z_offset_m',
            'segment_travel_z_offset_m',
            'segment_approach_vel_scale',
            'segment_travel_vel_scale',
            'segment_print_vel_scale',
            'segment_accel_scale',
            'segment_target_print_speed_mm_s',
            'post_segment_wait_s',
            'post_segment_retract_s',
            'post_segment_retract_speed',
            'max_cycles',
            'prompt_before_next_cycle'
            )
        }
        values["config"] = cfg
        return SimpleNamespace(**values)

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

    def _reload_runtime_config_or_keep(
        self,
        *,
        current: Dict[str, Any],
        config_path: str,
        reason: str,
    ) -> Dict[str, Any]:
        try:
            new_cfg = self._load_runtime_config(config_path)
        except Exception as exc:
            self.get_logger().warn(
                f"[print_field_oriented] Runtime config reload failed ({reason}); "
                f"keeping previous config. error={exc}"
            )
            return current

        self.get_logger().info(
            f"[print_field_oriented] Runtime config reloaded ({reason}): {config_path}"
        )
        return new_cfg

    @staticmethod
    def _default_runtime_config_path() -> str:
        return SRC_CONFIG_PATH

    @staticmethod
    def _captures_root() -> Path:
        env_root = os.environ.get("BEHAV3D_CAPTURES_ROOT", "").strip()
        if env_root:
            return Path(env_root).expanduser().resolve()
        return (Path.home() / "behav3d_ws" / "captures").resolve()

    def _latest_capture_session_path(self) -> Path:
        captures_root = self._captures_root()
        captures_root.mkdir(parents=True, exist_ok=True)
        dirs = [p for p in captures_root.iterdir() if p.is_dir()]
        if not dirs:
            return captures_root

        def _is_timestamp_dir(path: Path) -> bool:
            name = path.name
            return (
                len(name) == 13
                and name[6] == "_"
                and name[:6].isdigit()
                and name[7:].isdigit()
            )

        timestamp_dirs = [p for p in dirs if _is_timestamp_dir(p)]
        return max(timestamp_dirs or dirs, key=lambda p: p.name).resolve()

    @staticmethod
    def _resolve_run_session_token(value: str, run_session_path: Path) -> str:
        text = str(value or "").strip()
        if text == "@session":
            return str(run_session_path)
        if text.startswith("@session/"):
            return str((run_session_path / text[len("@session/") :]).resolve())
        return text

    @staticmethod
    def _run_session_child(run_session_path: Path, relative_path: str) -> str:
        return str((run_session_path / str(relative_path).lstrip("/")).resolve())

    @staticmethod
    def _first_available_cycle_number(run_session_path: Path) -> int:
        field_loop = run_session_path / "field_loop"
        if not field_loop.is_dir():
            return 0

        used: list[int] = []
        for path in field_loop.iterdir():
            if not path.is_dir() or not path.name.startswith("cycle_"):
                continue
            suffix = path.name[len("cycle_") :]
            if suffix.isdigit():
                used.append(int(suffix))
        if not used:
            return 0
        return max(used) + 1

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
