#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_field_centered_sequence
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import List, Optional

import behav3d_commands
import numpy as np
import rclpy
from behav3d_examples.src.grid_sweep_session import GridSweepSession
from behav3d_orchestrator.src.yaml_session import YamlSession
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class PrintFieldCenteredSequenceNode(Node):
    def __init__(self):
        super().__init__("print_field_centered_sequence")

        # Global flow
        self.declare_parameter("home_before_scan", True)
        self.declare_parameter("home_after", False)
        self.declare_parameter("timeout_s", 0.0)

        # Initial table grid sweep params (same behavior as print_field_sequence.py)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("scan_eef_link", "femto_color_optical_calib")
        self.declare_parameter("scan_use_tf_orientation", True)
        self.declare_parameter("scan_width", 0.50)
        self.declare_parameter("scan_height", 0.40)
        self.declare_parameter("scan_center_x", -0.20)
        self.declare_parameter("scan_center_y", 0.80)
        self.declare_parameter("scan_center_z", 0.0)
        self.declare_parameter("scan_z_off", 0.60)
        self.declare_parameter("scan_nx", 5)
        self.declare_parameter("scan_ny", 4)
        self.declare_parameter("scan_row_major", False)
        self.declare_parameter("scan_capture_folder", "@session/grid_sweep")
        self.declare_parameter("scan_debug_prompt", False)
        self.declare_parameter("scan_vel_scale", 0.05)
        self.declare_parameter("scan_accel_scale", 0.05)
        self.declare_parameter("scan_publish_markers", True)
        self.declare_parameter("scan_axis_length", 0.05)
        self.declare_parameter("scan_axis_radius", 0.003)
        self.declare_parameter("scan_clear_markers_before", True)

        # Layer scan params (used for cycle_0001 and later)
        self.declare_parameter("layer_scan_width", 0.30)
        self.declare_parameter("layer_scan_height", 0.30)
        self.declare_parameter("layer_scan_nx", 3)
        self.declare_parameter("layer_scan_ny", 3)
        self.declare_parameter("layer_scan_row_major", False)
        self.declare_parameter("layer_scan_first_z_off", 0.06)
        self.declare_parameter("layer_scan_z_margin_mm", 550.0)
        self.declare_parameter("field_center_sign_x", -1.0)
        self.declare_parameter("field_center_sign_y", -1.0)
        self.declare_parameter("field_center_offset_x", 0.0)
        self.declare_parameter("field_center_offset_y", 0.0)

        # Reconstruction + world mesh
        self.declare_parameter("run_reconstruction", True)
        self.declare_parameter("scan_folder", "grid_sweep")
        self.declare_parameter("reconstruct_device", "CPU:0")
        self.declare_parameter("reconstruct_request_timeout_s", 10.0)
        self.declare_parameter("wait_reconstruction_outputs", True)
        self.declare_parameter("color_to_depth_wait_timeout_s", 60.0)
        self.declare_parameter("tsdf_wait_timeout_s", 180.0)
        self.declare_parameter("mesh_prefer", "mesh")
        self.declare_parameter("mesh_update_wait_timeout_s", 45.0)
        self.declare_parameter("mesh_update_request_timeout_s", 55.0)
        self.declare_parameter("tsdf_center_crop_enable", True)
        self.declare_parameter("tsdf_center_crop_width", 300)
        self.declare_parameter("tsdf_center_crop_height", 290)
        self.declare_parameter("tsdf_center_crop_apply_to_depth", True)
        self.declare_parameter("tsdf_aabb_crop_enable", False)
        self.declare_parameter("tsdf_aabb_crop_min", [-0.25, -1.10, -1.00])
        self.declare_parameter("tsdf_aabb_crop_max", [0.30, -0.65, 0.50])
        self.declare_parameter("tsdf_param_update_timeout_s", 8.0)

        # Field init command
        self.declare_parameter("run_field_init", True)
        self.declare_parameter("field_use_latest", True)
        self.declare_parameter("field_session_path", "@session")
        self.declare_parameter("field_scan_mesh_path", "")
        self.declare_parameter("field_mesh_path", "/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
        self.declare_parameter("field_state_output_dir", "@session/field_loop/init")
        self.declare_parameter("field_request_timeout_s", 120.0)

        # Candidate generation + print
        self.declare_parameter("run_generate_candidates", True)
        self.declare_parameter("candidate_use_latest", True)
        self.declare_parameter("candidate_session_path", "@session")
        self.declare_parameter("candidate_field_state_path", "")
        self.declare_parameter("candidate_scan_mesh_path", "")
        self.declare_parameter("candidate_output_dir", "@session/field_loop/cycle_0001")
        self.declare_parameter("candidate_request_timeout_s", 120.0)
        self.declare_parameter("candidate_beads_per_step", 7)
        self.declare_parameter("candidate_bead_separation_mm", 16.0)
        self.declare_parameter("candidate_bead_height_mm", 12.0)
        self.declare_parameter("candidate_target_zx", 0.03)
        self.declare_parameter("candidate_target_zy", -0.01)
        self.declare_parameter("candidate_target_zz", 1.00)
        self.declare_parameter("candidate_target_position_scale", 1000.0)
        self.declare_parameter("candidate_visualize_field_ply", True)
        self.declare_parameter("candidate_restore_scan_mesh_after_field_ply", True)
        self.declare_parameter("candidate_publish_markers", True)
        self.declare_parameter("candidate_marker_axis_length", 0.05)
        self.declare_parameter("candidate_marker_axis_radius", 0.003)
        self.declare_parameter("candidate_marker_clear_before", True)
        self.declare_parameter("max_cycles", 0)
        self.declare_parameter("prompt_before_next_cycle", True)

        # Same node, two sessions: macro + direct commands.
        self.session = behav3d_commands.Session(self)
        self.grid_macro = GridSweepSession(self)
        self.yaml_parser = YamlSession(self, register_default_commands=False)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        home_before_scan = bool(self.get_parameter("home_before_scan").value)
        home_after = bool(self.get_parameter("home_after").value)
        timeout_s = self._param_optional_float("timeout_s")

        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        scan_eef_link = str(self.get_parameter("scan_eef_link").value).strip() or "femto_color_optical_calib"
        scan_use_tf_orientation = bool(self.get_parameter("scan_use_tf_orientation").value)
        scan_width = float(self.get_parameter("scan_width").value)
        scan_height = float(self.get_parameter("scan_height").value)
        scan_center_x = float(self.get_parameter("scan_center_x").value)
        scan_center_y = float(self.get_parameter("scan_center_y").value)
        scan_center_z = float(self.get_parameter("scan_center_z").value)
        scan_z_off = float(self.get_parameter("scan_z_off").value)
        scan_nx = int(self.get_parameter("scan_nx").value)
        scan_ny = int(self.get_parameter("scan_ny").value)
        scan_row_major = bool(self.get_parameter("scan_row_major").value)
        scan_capture_folder = str(self.get_parameter("scan_capture_folder").value).strip() or "@session/grid_sweep"
        scan_debug_prompt = bool(self.get_parameter("scan_debug_prompt").value)
        scan_vel_scale = float(self.get_parameter("scan_vel_scale").value)
        scan_accel_scale = float(self.get_parameter("scan_accel_scale").value)
        scan_publish_markers = bool(self.get_parameter("scan_publish_markers").value)
        scan_axis_length = float(self.get_parameter("scan_axis_length").value)
        scan_axis_radius = float(self.get_parameter("scan_axis_radius").value)
        scan_clear_markers_before = bool(self.get_parameter("scan_clear_markers_before").value)

        layer_scan_width = float(self.get_parameter("layer_scan_width").value)
        layer_scan_height = float(self.get_parameter("layer_scan_height").value)
        layer_scan_nx = int(self.get_parameter("layer_scan_nx").value)
        layer_scan_ny = int(self.get_parameter("layer_scan_ny").value)
        layer_scan_row_major = bool(self.get_parameter("layer_scan_row_major").value)
        layer_scan_first_z_off = float(self.get_parameter("layer_scan_first_z_off").value)
        layer_scan_z_margin_m = float(self.get_parameter("layer_scan_z_margin_mm").value) / 1000.0
        field_center_sign_x = float(self.get_parameter("field_center_sign_x").value)
        field_center_sign_y = float(self.get_parameter("field_center_sign_y").value)
        field_center_offset_x = float(self.get_parameter("field_center_offset_x").value)
        field_center_offset_y = float(self.get_parameter("field_center_offset_y").value)

        run_reconstruction = bool(self.get_parameter("run_reconstruction").value)
        reconstruct_device = str(self.get_parameter("reconstruct_device").value).strip() or "CPU:0"
        reconstruct_request_timeout_s = float(self.get_parameter("reconstruct_request_timeout_s").value)
        wait_reconstruction_outputs = bool(self.get_parameter("wait_reconstruction_outputs").value)
        color_to_depth_wait_timeout_s = float(self.get_parameter("color_to_depth_wait_timeout_s").value)
        tsdf_wait_timeout_s = float(self.get_parameter("tsdf_wait_timeout_s").value)
        mesh_prefer = str(self.get_parameter("mesh_prefer").value).strip() or "mesh"
        mesh_update_wait_timeout_s = float(self.get_parameter("mesh_update_wait_timeout_s").value)
        mesh_update_request_timeout_s = float(self.get_parameter("mesh_update_request_timeout_s").value)
        tsdf_center_crop_enable = bool(self.get_parameter("tsdf_center_crop_enable").value)
        tsdf_center_crop_width = int(self.get_parameter("tsdf_center_crop_width").value)
        tsdf_center_crop_height = int(self.get_parameter("tsdf_center_crop_height").value)
        tsdf_center_crop_apply_to_depth = bool(self.get_parameter("tsdf_center_crop_apply_to_depth").value)
        tsdf_aabb_crop_enable = bool(self.get_parameter("tsdf_aabb_crop_enable").value)
        tsdf_aabb_crop_min = list(self.get_parameter("tsdf_aabb_crop_min").value)
        tsdf_aabb_crop_max = list(self.get_parameter("tsdf_aabb_crop_max").value)
        tsdf_param_update_timeout_s = float(self.get_parameter("tsdf_param_update_timeout_s").value)

        run_field_init = bool(self.get_parameter("run_field_init").value)
        field_use_latest = bool(self.get_parameter("field_use_latest").value)
        field_session_path = str(self.get_parameter("field_session_path").value).strip()
        field_scan_mesh_path = str(self.get_parameter("field_scan_mesh_path").value).strip()
        field_mesh_path = str(self.get_parameter("field_mesh_path").value).strip()
        field_state_output_dir = str(self.get_parameter("field_state_output_dir").value).strip()
        field_request_timeout_s = float(self.get_parameter("field_request_timeout_s").value)

        run_generate_candidates = bool(self.get_parameter("run_generate_candidates").value)
        candidate_use_latest = bool(self.get_parameter("candidate_use_latest").value)
        candidate_session_path = str(self.get_parameter("candidate_session_path").value).strip()
        candidate_field_state_path = str(self.get_parameter("candidate_field_state_path").value).strip()
        candidate_scan_mesh_path = str(self.get_parameter("candidate_scan_mesh_path").value).strip()
        candidate_output_dir = str(self.get_parameter("candidate_output_dir").value).strip()
        candidate_request_timeout_s = float(self.get_parameter("candidate_request_timeout_s").value)
        candidate_beads_per_step = int(self.get_parameter("candidate_beads_per_step").value)
        candidate_bead_separation_mm = float(self.get_parameter("candidate_bead_separation_mm").value)
        candidate_bead_height_mm = float(self.get_parameter("candidate_bead_height_mm").value)
        candidate_target_zx = float(self.get_parameter("candidate_target_zx").value)
        candidate_target_zy = float(self.get_parameter("candidate_target_zy").value)
        candidate_target_zz = float(self.get_parameter("candidate_target_zz").value)
        candidate_target_position_scale = float(self.get_parameter("candidate_target_position_scale").value)
        candidate_visualize_field_ply = bool(self.get_parameter("candidate_visualize_field_ply").value)
        candidate_restore_scan_mesh_after_field_ply = bool(
            self.get_parameter("candidate_restore_scan_mesh_after_field_ply").value
        )
        candidate_publish_markers = bool(self.get_parameter("candidate_publish_markers").value)
        candidate_marker_axis_length = float(self.get_parameter("candidate_marker_axis_length").value)
        candidate_marker_axis_radius = float(self.get_parameter("candidate_marker_axis_radius").value)
        candidate_marker_clear_before = bool(self.get_parameter("candidate_marker_clear_before").value)
        max_cycles = int(self.get_parameter("max_cycles").value)
        prompt_before_next_cycle = bool(self.get_parameter("prompt_before_next_cycle").value)

        log.info(
            "Starting print_field_centered sequence: "
            f"initial_grid=({scan_nx}x{scan_ny}), "
            f"layer_grid=({layer_scan_nx}x{layer_scan_ny}), "
            f"run_reconstruction={run_reconstruction}, run_field_init={run_field_init}, "
            f"run_generate_candidates={run_generate_candidates}, max_cycles={max_cycles}"
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

            # -----------------------------------------------------------------
            # Bootstrap cycle: table scan + reconstruction + field init.
            # -----------------------------------------------------------------
            bootstrap_tag = "cycle_0000"
            bootstrap_root = f"@session/field_loop/{bootstrap_tag}"
            bootstrap_scan_capture_folder = f"{bootstrap_root}/scan"
            bootstrap_scan_folder = f"field_loop/{bootstrap_tag}/scan"
            bootstrap_field_state_output_dir = f"{bootstrap_root}/field_init"

            log.info(
                f"[print_field_centered] ===== {bootstrap_tag} (bootstrap) ===== "
                f"scan_capture='{bootstrap_scan_capture_folder}' "
                f"scan_folder='{bootstrap_scan_folder}'"
            )

            bootstrap_targets = self.grid_macro.run_grid_sweep(
                width=scan_width,
                height=scan_height,
                center_x=scan_center_x,
                center_y=scan_center_y,
                center_z=scan_center_z,
                z_off=scan_z_off,
                nx=scan_nx,
                ny=scan_ny,
                row_major=scan_row_major,
                frame_id=frame_id,
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
                mesh_path, rgb_ply_path = self._run_reconstruction_for_scan(
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

                field_center_raw = self._compute_field_center_from_state(field_state_path)
                field_center_xy = self._map_field_center_xy(
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
                    "[print_field_centered] Field initialized: "
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
                        f"[print_field_centered] Field PLY published in RViz ({field_vis_kind}): {field_vis_path}"
                    )
            else:
                if not field_state_path:
                    raise RuntimeError(
                        "run_field_init=False and candidate_field_state_path is empty. "
                        "Provide an existing field state path."
                    )
                field_center_raw = self._compute_field_center_from_state(field_state_path)
                field_center_xy = self._map_field_center_xy(
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
                    "[print_field_centered] Using provided field state: "
                    f"state='{field_state_path}', "
                    f"field_center_raw=({field_center_raw[0]:.4f}, {field_center_raw[1]:.4f}, {field_center_raw[2]:.4f}), "
                    f"field_center_scan=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}), "
                    f"xy_map=(x*{field_center_sign_x:.3f}+{field_center_offset_x:.3f}, "
                    f"y*{field_center_sign_y:.3f}+{field_center_offset_y:.3f})"
                )

            if field_center_xyz is None:
                raise RuntimeError("Field center not available after bootstrap.")

            if not run_generate_candidates:
                log.warn("[print_field_centered] run_generate_candidates=False; stopping after bootstrap.")
                return

            # -----------------------------------------------------------------
            # Print cycles:
            # - cycle_0001 candidates come from bootstrap scan mesh.
            # - after each print, run a centered layer scan for the next cycle.
            # -----------------------------------------------------------------
            print_cycle_index = 0
            while rclpy.ok():
                if max_cycles > 0 and print_cycle_index >= max_cycles:
                    log.info(f"[print_field_centered] Reached max_cycles={max_cycles}.")
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
                        beads_per_step=candidate_beads_per_step,
                        bead_separation_mm=candidate_bead_separation_mm,
                        bead_height_mm=candidate_bead_height_mm,
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
                    "[print_field_centered] Candidates generated: "
                    f"count={cand_metrics.get('candidate_count', 0)} "
                    f"viable={cand_metrics.get('viable_count', 0)} "
                    f"contour_segments={cand_metrics.get('contour_segment_count', 0)} "
                    f"yaml='{cand_metrics.get('targets_yaml_path', '')}' "
                    f"cycle_dir='{cand_metrics.get('cycle_output_dir', '')}'"
                )

                targets_yaml_path = str(cand_metrics.get("targets_yaml_path", "")).strip()
                preview_targets: List[PoseStamped] = []
                if targets_yaml_path:
                    preview_targets = self.yaml_parser.parse_yaml_targets(
                        yaml_path=targets_yaml_path,
                        frame_id=frame_id,
                    )

                if candidate_visualize_field_ply:
                    cycle_field_ply = str(cand_metrics.get("debug_field_ply_path", "")).strip()
                    if cycle_field_ply:
                        cycle_vis_res = self.session.run_sync(
                            self.session.camera.update_world_mesh(
                                use_latest=False,
                                session_path=candidate_session_path or "@session",
                                mesh_path="",
                                ply_path=cycle_field_ply,
                                prefer="ply",
                                wait_timeout_s=mesh_update_wait_timeout_s,
                                enqueue=False,
                            ),
                            timeout_s=mesh_update_request_timeout_s,
                        )
                        if not cycle_vis_res.get("ok", False):
                            raise RuntimeError(
                                f"update_world_mesh(cycle_field_ply) failed: {cycle_vis_res.get('error')}"
                            )

                        cycle_vis_path = str(cycle_vis_res.get("metrics", {}).get("published_path", "")).strip()
                        cycle_vis_kind = str(cycle_vis_res.get("metrics", {}).get("published_kind", "")).strip()
                        log.info(
                            f"[print_field_centered] Cycle field PLY published in RViz ({cycle_vis_kind}): "
                            f"{cycle_vis_path}"
                        )

                        if candidate_restore_scan_mesh_after_field_ply and scan_mesh_for_candidates:
                            scan_restore_res = self.session.run_sync(
                                self.session.camera.update_world_mesh(
                                    use_latest=False,
                                    session_path=candidate_session_path or "@session",
                                    mesh_path=scan_mesh_for_candidates,
                                    ply_path="",
                                    prefer="mesh",
                                    wait_timeout_s=mesh_update_wait_timeout_s,
                                    enqueue=False,
                                ),
                                timeout_s=mesh_update_request_timeout_s,
                            )
                            if not scan_restore_res.get("ok", False):
                                raise RuntimeError(
                                    f"update_world_mesh(restore_scan_mesh) failed: "
                                    f"{scan_restore_res.get('error')}"
                                )
                            restored_path = str(
                                scan_restore_res.get("metrics", {}).get("published_path", "")
                            ).strip()
                            restored_kind = str(
                                scan_restore_res.get("metrics", {}).get("published_kind", "")
                            ).strip()
                            log.info(
                                f"[print_field_centered] Scan mesh restored in RViz ({restored_kind}): "
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
                        f"[print_field_centered] Candidate markers published: {len(preview_targets)} targets "
                        f"(yaml='{targets_yaml_path}', frame='{frame_id}')"
                    )

                if not preview_targets:
                    log.warn("[print_field_centered] No candidate targets generated; stopping loop.")
                    break

                gate_res = self.session.run_sync(
                    self.session.util.input(
                        prompt=(
                            f"[print_field_centered:{cycle_tag}] Press ENTER to print targets "
                            "(type 'q' + ENTER to stop)."
                        ),
                        enqueue=False,
                    ),
                    timeout_s=None,
                )
                gate_value = str(gate_res.get("metrics", {}).get("value", "")).strip().lower()
                if gate_value == "q":
                    log.warn("[print_field_centered] Print execution stopped by user.")
                    break

                print_res = self._run_print_dots_targets(
                    targets=preview_targets,
                    timeout_s=timeout_s,
                )
                if not print_res.get("ok", False):
                    raise RuntimeError(
                        f"print targets failed at stage={print_res.get('stage')}: "
                        f"{print_res.get('error', 'unknown')}"
                    )

                printed_count = int(print_res.get("printed", 0))
                log.info(
                    "[print_field_centered] Print dots completed: "
                    f"printed={printed_count} / targets={print_res.get('targets', 0)}"
                )
                last_printed_targets = list(preview_targets[:printed_count])

                print_cycle_index += 1
                if max_cycles > 0 and print_cycle_index >= max_cycles:
                    log.info(f"[print_field_centered] Reached max_cycles={max_cycles}.")
                    break

                if prompt_before_next_cycle:
                    next_res = self.session.run_sync(
                        self.session.util.input(
                            prompt=(
                                f"[print_field_centered:{cycle_tag}] Press ENTER for next cycle "
                                f"(cycle_{print_cycle_index + 1:04d}) or type 'q' + ENTER to stop."
                            ),
                            enqueue=False,
                        ),
                        timeout_s=None,
                    )
                    next_value = str(next_res.get("metrics", {}).get("value", "")).strip().lower()
                    if next_value == "q":
                        log.info("[print_field_centered] Stopped by user before next cycle.")
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
                    f"[print_field_centered] ===== scan_for_{next_cycle_tag} ===== "
                    f"scan_capture='{next_cycle_scan_capture_folder}' "
                    f"scan_folder='{next_cycle_scan_folder}' "
                    f"center=({field_center_xyz[0]:.4f}, {field_center_xyz[1]:.4f}, {field_center_xyz[2]:.4f}) "
                    f"grid={layer_scan_nx}x{layer_scan_ny} size={1000.0*layer_scan_width:.1f}x"
                    f"{1000.0*layer_scan_height:.1f}mm {z_msg}"
                )

                scan_targets = self.grid_macro.run_grid_sweep(
                    target=center_target,
                    width=layer_scan_width,
                    height=layer_scan_height,
                    z_off=cycle_scan_z_off,
                    nx=layer_scan_nx,
                    ny=layer_scan_ny,
                    row_major=layer_scan_row_major,
                    frame_id=center_target.header.frame_id,
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
                )
                if not scan_targets:
                    raise RuntimeError(f"Grid sweep produced 0 targets for scan_for_{next_cycle_tag}.")

                if run_reconstruction:
                    mesh_path, rgb_ply_path = self._run_reconstruction_for_scan(
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
                        "[print_field_centered] run_reconstruction=False after scan; "
                        "next cycle will reuse previous scan mesh unless candidate_scan_mesh_path is set."
                    )

            if home_after:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )

        except TimeoutError:
            log.error("[print_field_centered] Sequence timed out.")
        except Exception as exc:
            log.error(f"[print_field_centered] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _run_reconstruction_for_scan(
        self,
        *,
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
        tsdf_aabb_crop_min: list[float],
        tsdf_aabb_crop_max: list[float],
        tsdf_param_update_timeout_s: float,
    ) -> tuple[str, str]:
        log = self.get_logger()

        c2d_start_ts = time.time()
        c2d_res = self.session.run_sync(
            self.session.camera.reconstruct_color_to_depth(
                use_latest=True,
                session_path="@session",
                scan_folder=scan_folder,
                visualize=False,
                enqueue=False,
            ),
            timeout_s=reconstruct_request_timeout_s,
        )
        if not c2d_res.get("ok", False):
            raise RuntimeError(f"color_to_depth failed: {c2d_res.get('error')}")

        c2d_output = str(c2d_res.get("metrics", {}).get("output_path", "")).strip()
        if wait_reconstruction_outputs and c2d_output:
            self._wait_for_fresh_alignment_output(
                output_path=c2d_output,
                start_ts=c2d_start_ts,
                timeout_s=color_to_depth_wait_timeout_s,
            )

        try:
            self._set_tsdf_service_crop_params(
                center_crop_enable=tsdf_center_crop_enable,
                center_crop_width=tsdf_center_crop_width,
                center_crop_height=tsdf_center_crop_height,
                center_crop_apply_to_depth=tsdf_center_crop_apply_to_depth,
                aabb_crop_enable=tsdf_aabb_crop_enable,
                aabb_crop_min=tsdf_aabb_crop_min,
                aabb_crop_max=tsdf_aabb_crop_max,
                timeout_s=tsdf_param_update_timeout_s,
            )
        except Exception as exc:
            log.warn(
                "[print_field_centered] Failed to update TSDF crop params; "
                f"continuing with current service params. reason='{exc}'"
            )

        tsdf_start_ts = time.time()
        tsdf_res = self.session.run_sync(
            self.session.camera.reconstruct_tsdf_grid_sweep(
                use_latest=True,
                session_path="@session",
                scan_folder=scan_folder,
                visualize=False,
                device=reconstruct_device,
                enqueue=False,
            ),
            timeout_s=reconstruct_request_timeout_s,
        )
        if not tsdf_res.get("ok", False):
            raise RuntimeError(f"tsdf_grid_sweep failed: {tsdf_res.get('error')}")

        mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
        rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()
        if not rgb_ply_path:
            rgb_ply_path = str(tsdf_res.get("metrics", {}).get("output_path", "")).strip()

        if wait_reconstruction_outputs and mesh_path:
            self._wait_for_fresh_file_output(
                output_path=mesh_path,
                start_ts=tsdf_start_ts,
                timeout_s=tsdf_wait_timeout_s,
            )

        mesh_res = self.session.run_sync(
            self.session.camera.update_world_mesh(
                use_latest=True,
                session_path="@session",
                mesh_path=mesh_path,
                ply_path=rgb_ply_path,
                prefer=mesh_prefer,
                wait_timeout_s=mesh_update_wait_timeout_s,
                enqueue=False,
            ),
            timeout_s=mesh_update_request_timeout_s,
        )
        if not mesh_res.get("ok", False):
            raise RuntimeError(f"update_world_mesh failed: {mesh_res.get('error')}")

        published_path = str(mesh_res.get("metrics", {}).get("published_path", "")).strip()
        published_kind = str(mesh_res.get("metrics", {}).get("published_kind", "")).strip()
        log.info(
            f"[print_field_centered] Scan mesh published in RViz ({published_kind}): {published_path}"
        )
        return mesh_path, rgb_ply_path

    def _set_tsdf_service_crop_params(
        self,
        *,
        center_crop_enable: bool,
        center_crop_width: int,
        center_crop_height: int,
        center_crop_apply_to_depth: bool,
        aabb_crop_enable: bool,
        aabb_crop_min: list[float],
        aabb_crop_max: list[float],
        timeout_s: float,
    ) -> None:
        log = self.get_logger()
        timeout = max(0.1, float(timeout_s))
        crop_min = self._coerce_xyz_triplet(aabb_crop_min, fallback=(-0.25, -1.10, -1.00))
        crop_max = self._coerce_xyz_triplet(aabb_crop_max, fallback=(0.30, -0.65, 0.50))

        client = AsyncParameterClient(self, "/tsdf_cropped_service")
        if hasattr(client, "wait_for_services"):
            ready = bool(client.wait_for_services(timeout_sec=timeout))
        elif hasattr(client, "wait_for_service"):
            ready = bool(client.wait_for_service(timeout_sec=timeout))
        else:
            ready = bool(client.services_are_ready()) if hasattr(client, "services_are_ready") else False
        if not ready:
            raise RuntimeError("Parameter service for /tsdf_cropped_service not available.")

        params = [
            Parameter("center_crop_enable", value=bool(center_crop_enable)),
            Parameter("center_crop_width", value=int(center_crop_width)),
            Parameter("center_crop_height", value=int(center_crop_height)),
            Parameter("center_crop_apply_to_depth", value=bool(center_crop_apply_to_depth)),
            Parameter("aabb_crop_enable", value=bool(aabb_crop_enable)),
            Parameter("aabb_crop_min", value=[float(crop_min[0]), float(crop_min[1]), float(crop_min[2])]),
            Parameter("aabb_crop_max", value=[float(crop_max[0]), float(crop_max[1]), float(crop_max[2])]),
        ]

        fut = client.set_parameters(params)
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.05)

        if not fut.done():
            raise TimeoutError("Timed out while setting TSDF crop parameters.")

        response = fut.result()
        if response is None:
            raise RuntimeError("Failed to set TSDF crop parameters: no response.")

        if hasattr(response, "results"):
            results = list(response.results)
        elif isinstance(response, (list, tuple)):
            results = list(response)
        else:
            raise RuntimeError(
                "Failed to set TSDF crop parameters: unexpected response type "
                f"{type(response).__name__}"
            )

        failed = [str(r.reason) for r in results if not bool(getattr(r, "successful", False))]
        if failed:
            raise RuntimeError(f"Failed to set TSDF crop parameters: {'; '.join(failed)}")

        log.info(
            "[print_field_centered] TSDF crop params set: "
            f"center_crop_enable={bool(center_crop_enable)} "
            f"w={int(center_crop_width)} h={int(center_crop_height)} "
            f"apply_to_depth={bool(center_crop_apply_to_depth)} "
            f"aabb_crop_enable={bool(aabb_crop_enable)} "
            f"aabb_min=({crop_min[0]:.3f}, {crop_min[1]:.3f}, {crop_min[2]:.3f}) "
            f"aabb_max=({crop_max[0]:.3f}, {crop_max[1]:.3f}, {crop_max[2]:.3f})"
        )

    @staticmethod
    def _compute_field_center_from_state(field_state_path: str) -> tuple[float, float, float]:
        path = Path(str(field_state_path).strip()).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Field state file not found: {field_state_path}")

        with np.load(str(path), allow_pickle=False) as state:
            if "field_vertices_world" in state:
                vertices_world = np.asarray(state["field_vertices_world"], dtype=np.float64)
                if vertices_world.ndim == 2 and vertices_world.shape[1] == 3 and vertices_world.shape[0] > 0:
                    center = np.mean(vertices_world, axis=0)
                    return (float(center[0]), float(center[1]), float(center[2]))

            if "field_vertices_scaled" in state and "offset_xyz" in state:
                vertices_scaled = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
                offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
                if (
                    vertices_scaled.ndim == 2
                    and vertices_scaled.shape[1] == 3
                    and vertices_scaled.shape[0] > 0
                    and offset_arr.shape[0] >= 3
                ):
                    center = np.mean(vertices_scaled, axis=0) + offset_arr[:3]
                    return (float(center[0]), float(center[1]), float(center[2]))

            if "offset_xyz" in state:
                offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
                if offset_arr.shape[0] >= 3:
                    return (float(offset_arr[0]), float(offset_arr[1]), float(offset_arr[2]))

        raise ValueError(
            f"Unable to infer field center from state file '{path}'. "
            "Expected field_vertices_world or (field_vertices_scaled + offset_xyz)."
        )

    @staticmethod
    def _coerce_xyz_triplet(values: list[float], *, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
        try:
            arr = np.asarray(values, dtype=float).reshape(-1)
            if arr.shape[0] >= 3:
                return (float(arr[0]), float(arr[1]), float(arr[2]))
        except Exception:
            pass
        return (float(fallback[0]), float(fallback[1]), float(fallback[2]))

    @staticmethod
    def _map_field_center_xy(
        *,
        x: float,
        y: float,
        sign_x: float,
        sign_y: float,
        offset_x: float,
        offset_y: float,
    ) -> tuple[float, float]:
        return (
            float(sign_x) * float(x) + float(offset_x),
            float(sign_y) * float(y) + float(offset_y),
        )

    def _wait_for_fresh_alignment_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> None:
        out_dir = Path(output_path)

        def _fresh_alignment_exists() -> bool:
            if not out_dir.exists() or not out_dir.is_dir():
                return False
            for path in out_dir.glob("color_in_depth*.png"):
                try:
                    if path.stat().st_mtime >= (start_ts - 0.25):
                        return True
                except OSError:
                    continue
            return False

        self.session.run_sync(
            self.session.util.wait_until(
                predicate=_fresh_alignment_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def _wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> None:
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            try:
                if not out_path.exists() or not out_path.is_file():
                    return False
                st = out_path.stat()
                if st.st_size <= 0:
                    return False
                return st.st_mtime >= (start_ts - 0.25)
            except OSError:
                return False

        self.session.run_sync(
            self.session.util.wait_until(
                predicate=_fresh_file_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def _param_optional_float(self, name: str) -> Optional[float]:
        val = float(self.get_parameter(name).value)
        if val <= 0.0:
            return None
        return val

    def _run_print_dots_targets(
        self,
        *,
        targets: list[PoseStamped],
        timeout_s: Optional[float],
    ) -> dict:
        if len(targets) < 1:
            return {"ok": False, "stage": "parse_targets", "error": "Need at least 1 target."}

        pre_dot_vel_scale = 0.1
        dot_vel_scale = 0.05
        accel_scale = 0.05
        approach_z_offset_m = 0.40
        dot_z_offset_m = 0.04
        dot_steps = 11000
        dot_speed = 1200
        dwell_s = 0.4

        self.session.run_sync(self.session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
        self.session.run_sync(self.session.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
        self.session.run_sync(self.session.motion.setSpd(float(dot_vel_scale), enqueue=False), timeout_s=timeout_s)
        self.session.run_sync(self.session.motion.setEef("extruder_tcp", enqueue=False), timeout_s=timeout_s)

        first_approach = self._with_z_offset(targets[0], float(approach_z_offset_m))
        res = self.session.run_sync(
            self.session.motion.goto(
                pose=first_approach,
                motion="LIN",
                vel_scale=float(pre_dot_vel_scale),
                exec=True,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not res.get("ok", False):
            return {"ok": False, "stage": "approach_first", "error": res.get("error", "unknown")}

        printed = 0
        for i, target in enumerate(targets):
            above = self._with_z_offset(target, float(dot_z_offset_m))

            above_vel = float(pre_dot_vel_scale) if i == 0 else float(dot_vel_scale)
            res = self.session.run_sync(
                self.session.motion.goto(
                    pose=above,
                    motion="LIN",
                    vel_scale=above_vel,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": f"goto_above_{i}", "error": res.get("error", "unknown")}

            res = self.session.run_sync(
                self.session.motion.goto(
                    pose=target,
                    motion="LIN",
                    vel_scale=float(dot_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": f"goto_target_{i}", "error": res.get("error", "unknown")}

            res = self.session.run_sync(
                self.session.extruder.print_steps(
                    steps=int(dot_steps),
                    speed=int(dot_speed),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": f"extrude_{i}", "error": res.get("error", "unknown")}

            if float(dwell_s) > 0.0:
                res = self.session.run_sync(
                    self.session.util.wait(float(dwell_s), enqueue=False),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {"ok": False, "stage": f"wait_{i}", "error": res.get("error", "unknown")}

            res = self.session.run_sync(
                self.session.motion.goto(
                    pose=above,
                    motion="LIN",
                    vel_scale=float(dot_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": f"lift_{i}", "error": res.get("error", "unknown")}

            printed += 1

        return {"ok": True, "stage": "done", "targets": len(targets), "printed": printed}

    @staticmethod
    def _with_z_offset(ps: PoseStamped, z_offset_m: float) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = ps.header.frame_id
        out.header.stamp = ps.header.stamp
        out.pose.position.x = float(ps.pose.position.x)
        out.pose.position.y = float(ps.pose.position.y)
        out.pose.position.z = float(ps.pose.position.z) + float(z_offset_m)
        out.pose.orientation.x = float(ps.pose.orientation.x)
        out.pose.orientation.y = float(ps.pose.orientation.y)
        out.pose.orientation.z = float(ps.pose.orientation.z)
        out.pose.orientation.w = float(ps.pose.orientation.w)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PrintFieldCenteredSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
