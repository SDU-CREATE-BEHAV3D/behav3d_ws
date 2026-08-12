#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator geometry_representation_scan_print_loop
"""

from __future__ import annotations

import os
import shutil
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from behav3d_orchestrator.src.field_loop_session import FieldLoopSession
from behav3d_orchestrator.src.contour_offset_logger import ContourOffsetLogger
from behav3d_orchestrator.src.print_session import PrintSession
from behav3d_orchestrator.src.remote_parameters import set_remote_parameters
from behav3d_orchestrator.src.scan_session import ScanSession
from behav3d_utils.config_snapshot import snapshot_cycle_config
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

CONFIG_FILENAME = "geometry_representation_scan_print_loop_config.yaml"
SRC_CONFIG_PATH = f"/home/lab/behav3d_ws/src/behav3d_orchestrator/config/{CONFIG_FILENAME}"

SCAN_FRAME_ID = "world"
SCAN_EEF_LINK = "femto_color_optical_calib"
SCAN_MOTION = "LIN"

CANDIDATE_VISUALIZE_FIELD_PLY = True
CANDIDATE_RESTORE_SCAN_MESH_AFTER_FIELD_PLY = True
CANDIDATE_PUBLISH_MARKERS = True
CANDIDATE_MARKER_AXIS_LENGTH = 0.05
CANDIDATE_MARKER_AXIS_RADIUS = 0.003
CANDIDATE_MARKER_CLEAR_BEFORE = True
FIELDS_NODE_NAME = "/behav3d_fields"
CANDIDATE_PARAM_TIMEOUT_S = 8.0


class GeometryRepresentationScanPrintLoopNode(Node):
    """
    Geometry-representation scan and print loop.

    Keeps the scan/reconstruction/field loop flow in this sequence, but
    delegates print execution to PrintSession:
    - flat `targets:` YAML prints as dots,
    - nested `segments:` YAML prints as line paths.
    """

    def __init__(self):
        super().__init__("geometry_representation_scan_print_loop")

        # Field logic stays in FieldLoopSession; scan execution uses ScanSession.
        self.session = FieldLoopSession(self)
        self.print_session = PrintSession(self)
        self.scan_session = ScanSession(self)
        self.contour_offset_logger = ContourOffsetLogger()
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
            "Starting geometry_representation_scan_print_loop sequence: "
            f"scan_type={rt.scan_type}, enable_scan={rt.enable_scan}, debug_mode={rt.debug_mode}, "
            f"prompt_before_scan={rt.prompt_before_scan}, "
            f"scan_motion={SCAN_MOTION}, scan_vel_scale={rt.scan_vel_scale:.3f}, "
            f"scan_accel_scale={rt.scan_accel_scale:.3f}, scan_settle_s={rt.scan_settle_s:.3f}, "
            f"grid=({rt.grid_nx}x{rt.grid_ny}), "
            f"run_reconstruction={rt.run_reconstruction}, "
            f"max_cycles={rt.max_cycles}, "
            f"candidate_mode={rt.candidate_mode}, "
            f"candidate_width_mode={rt.candidate_width_mode}, "
            f"use_collision={rt.use_collision}, "
            f"contour_offset_estimation={rt.contour_offset_estimation}, "
            f"print_mode={rt.print_mode}, log_joint_currents={rt.log_joint_currents}, "
            f"dot_steps={rt.dot_steps}, extrusion_steps_per_mm3={rt.extrusion_steps_per_mm3:.9f}, "
            f"segment_steps={rt.segment_steps}, "
            f"segment_steps_per_second={rt.segment_steps_per_second}, "
            f"segment_print_v={rt.segment_print_vel_scale:.3f}, "
            f"post_segment_wait_s={rt.post_segment_wait_s:.3f}, "
            f"post_segment_retract_steps={rt.post_segment_retract_steps}, "
            f"post_segment_retract_speed={rt.post_segment_retract_speed}, "
            f"oriented_targets_enable={rt.oriented_targets_enable}, "
            f"clamp_to_cone={rt.oriented_clamp_to_cone}, cone_max_tilt_deg={rt.oriented_cone_max_tilt_deg:.1f}, "
            f"scan_before_generate_first_cycle={rt.scan_before_generate_first_cycle}, "
            f"run_session='{run_session_arg}', first_cycle={first_print_cycle_number:04d}, "
            f"runtime_config_path='{runtime_config_path}'"
        )

        mesh_path = ""
        field_state_path = ""
        field_center_xyz: Optional[tuple[float, float, float]] = None
        last_generated_targets: List[PoseStamped] = []
        pending_contour_offset_record: Optional[Path] = None

        try:
            if rt.home_before_scan:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=rt.timeout_s,
                )
                if not self._wait_if_control_paused("after home_before_scan"):
                    return

            field_state_path, field_center_raw, field_center_xyz = (
                self._load_configured_field_state(rt)
            )

            mesh_path = str(rt.resume_scan_mesh_path).strip()
            if mesh_path:
                mesh_res = self.session.run_sync(
                    self.session.camera.update_world_mesh(
                        use_latest=False,
                        session_path=rt.candidate_session_path or run_session_arg,
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
                if not self._wait_if_control_paused("after resume scan mesh publish"):
                    return

            log.info(
                "[geometry_representation_scan_print_loop] Using known field state: "
                f"state='{field_state_path}', "
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

                if not rt.enable_scan:
                    log.info(
                        f"[geometry_representation_scan_print_loop] pre_scan_for_{cycle_tag} skipped because enable_scan=false."
                    )
                else:
                    if not self._prompt_before_scan_if_enabled(
                        enabled=rt.prompt_before_scan,
                        label=f"pre_scan_for_{cycle_tag}",
                    ):
                        log.info("[geometry_representation_scan_print_loop] Stopped by user before pre-scan.")
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
                        use_tf_orientation=rt.grid_use_tf_orientation,
                        capture_folder=cycle_scan_capture_folder,
                        timeout_s=rt.timeout_s,
                        frame_id=rt.frame_id,
                        vel_scale=rt.scan_vel_scale,
                        accel_scale=rt.scan_accel_scale,
                        settle_s=rt.scan_settle_s,
                    )
                    if not pre_scan_targets:
                        raise RuntimeError(f"Scan produced 0 targets for pre_scan_for_{cycle_tag}.")
                    if not self._wait_if_control_paused(f"after pre_scan_for_{cycle_tag}"):
                        return

                if rt.enable_scan and rt.run_reconstruction:
                    mesh_path, _ = self._run_reconstruction_for_scan(
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

            # -----------------------------------------------------------------
            # Print cycles:
            # - first-cycle candidates use a fresh or resumed scan mesh.
            # - after each print, run a centered layer scan for the next cycle.
            # -----------------------------------------------------------------
            print_cycle_index = 0
            while rclpy.ok():
                refresh_runtime_config(
                    f"cycle loop start {first_print_cycle_number + print_cycle_index:04d}"
                )
                (
                    configured_field_state_path,
                    configured_field_center_raw,
                    configured_field_center_xyz,
                ) = self._load_configured_field_state(rt)
                if (
                    configured_field_state_path != field_state_path
                    or configured_field_center_xyz != field_center_xyz
                ):
                    log.warn(
                        "[geometry_representation_scan_print_loop] Runtime field state "
                        f"updated: '{field_state_path}' -> "
                        f"'{configured_field_state_path}', "
                        f"center_raw=({configured_field_center_raw[0]:.4f}, "
                        f"{configured_field_center_raw[1]:.4f}, "
                        f"{configured_field_center_raw[2]:.4f}), "
                        f"center_scan=({configured_field_center_xyz[0]:.4f}, "
                        f"{configured_field_center_xyz[1]:.4f}, "
                        f"{configured_field_center_xyz[2]:.4f})"
                    )
                field_state_path = configured_field_state_path
                field_center_xyz = configured_field_center_xyz

                if rt.max_cycles > 0 and print_cycle_index >= rt.max_cycles:
                    log.info(f"[geometry_representation_scan_print_loop] Reached max_cycles={rt.max_cycles}.")
                    break

                cycle_number = first_print_cycle_number + print_cycle_index
                cycle_tag = f"cycle_{cycle_number:04d}"
                cycle_root = self._run_session_child(run_session_path, f"field_loop/{cycle_tag}")
                cycle_candidate_output_dir = f"{cycle_root}/candidates"

                if not self._wait_if_control_paused(f"before candidate generation for {cycle_tag}"):
                    break

                if not mesh_path:
                    raise RuntimeError(
                        "No scan mesh is available for candidate generation. "
                        "Enable scan_before_generate_first_cycle or set resume_scan_mesh_path."
                    )
                scan_mesh_paths = [mesh_path]
                field_state_for_candidates = field_state_path
                if not field_state_for_candidates:
                    raise RuntimeError(
                        "Missing field state for candidate generation."
                    )

                self._set_fields_node_candidate_params(rt)

                cand_res = self.session.run_sync(
                    self.session.field.generate_print_candidates(
                        use_latest=False,
                        session_path=rt.candidate_session_path,
                        field_state_path=field_state_for_candidates,
                        scan_mesh_paths=scan_mesh_paths,
                        output_dir=cycle_candidate_output_dir,
                        candidate_mode=rt.candidate_mode,
                        beads_per_step=rt.candidate_beads_per_step,
                        bead_separation_mm=rt.candidate_bead_separation_mm,
                        bead_height_mm=rt.candidate_bead_height_mm,
                        orient_with_tangent=rt.oriented_targets_enable,
                        tangent_sign=rt.oriented_tangent_sign,
                        clamp_to_cone=rt.oriented_clamp_to_cone,
                        cone_max_tilt_deg=rt.oriented_cone_max_tilt_deg,
                        base_to_world_yaw_deg=rt.oriented_base_to_world_yaw_deg,
                        enqueue=False,
                    ),
                    timeout_s=rt.candidate_request_timeout_s,
                )
                if not cand_res.get("ok", False):
                    raise RuntimeError(f"generate_print_candidates failed: {cand_res.get('error')}")

                cand_metrics = cand_res.get("metrics", {})
                if pending_contour_offset_record is not None:
                    post_contour_path = str(
                        cand_metrics.get("debug_contour_ply_path", "")
                    ).strip()
                    try:
                        offset_result = self.contour_offset_logger.evaluate_cycle(
                            pending_contour_offset_record,
                            post_phi_contour_path=post_contour_path,
                        )
                        log.info(
                            "[contour_offset] Evaluated: "
                            f"valid={offset_result.get('valid_target_count', 0)}/"
                            f"{offset_result.get('target_count', 0)} "
                            f"median_mm={offset_result.get('scan_offset_median_mm')} "
                            f"mad_mm={offset_result.get('scan_offset_mad_mm')} "
                            f"confidence={offset_result.get('confidence', 0.0):.3f}"
                        )
                    except Exception as exc:
                        log.warn(
                            "[contour_offset] Evaluation failed; "
                            f"loop is unaffected. error='{exc}'"
                        )
                    finally:
                        pending_contour_offset_record = None
                log.info(
                    "[geometry_representation_scan_print_loop] Candidates generated: "
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
                preview_volumes_mm3: List[Optional[float]] = []
                preview_segments = []
                if targets_yaml_path:
                    preview_segments = self.print_session.parse_yaml_segments(
                        yaml_path=targets_yaml_path,
                        frame_id=rt.frame_id,
                    )
                    if preview_segments:
                        preview_targets = self.print_session.parse_yaml_targets(
                            yaml_path=targets_yaml_path,
                            frame_id=rt.frame_id,
                        )
                        preview_volumes_mm3 = [None] * len(preview_targets)
                    else:
                        dot_targets = self.print_session.parse_yaml_dot_targets(
                            yaml_path=targets_yaml_path,
                            frame_id=rt.frame_id,
                        )
                        preview_targets = [target.pose for target in dot_targets]
                        preview_volumes_mm3 = [
                            target.volume_mm3 for target in dot_targets
                        ]

                if CANDIDATE_VISUALIZE_FIELD_PLY:
                    cycle_field_ply = str(cand_metrics.get("debug_field_ply_path", "")).strip()
                    if cycle_field_ply:
                        cycle_vis_res = self.session.run_sync(
                            self.session.camera.preview_field_ply(
                                use_latest=False,
                                session_path=rt.candidate_session_path or run_session_arg,
                                field_ply_path=cycle_field_ply,
                                restore_mesh_path=(
                                    mesh_path
                                    if CANDIDATE_RESTORE_SCAN_MESH_AFTER_FIELD_PLY
                                    else ""
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

                        cycle_vis_metrics = cycle_vis_res.get("metrics", {})
                        cycle_vis_path = str(
                            cycle_vis_metrics.get("field_ply_published_path", "")
                        ).strip()
                        cycle_vis_kind = str(
                            cycle_vis_metrics.get("field_ply_published_kind", "ply")
                        ).strip()
                        log.info(
                            "[geometry_representation_scan_print_loop] "
                            f"Cycle field PLY published in RViz ({cycle_vis_kind}): "
                            f"{cycle_vis_path}"
                        )

                        if CANDIDATE_RESTORE_SCAN_MESH_AFTER_FIELD_PLY and mesh_path:
                            restored_path = str(
                                cycle_vis_metrics.get("restore_mesh_published_path", "")
                            ).strip()
                            restored_kind = str(
                                cycle_vis_metrics.get(
                                    "restore_mesh_published_kind", "mesh"
                                )
                            ).strip()
                            log.info(
                                "[geometry_representation_scan_print_loop] "
                                f"Scan mesh restored in RViz ({restored_kind}): "
                                f"{restored_path}"
                            )

                if CANDIDATE_PUBLISH_MARKERS and preview_targets:
                    marker_res = self.session.run_sync(
                        self.session.util.publish_targets(
                            preview_targets,
                            axis_length=CANDIDATE_MARKER_AXIS_LENGTH,
                            axis_radius=CANDIDATE_MARKER_AXIS_RADIUS,
                            clear_before=CANDIDATE_MARKER_CLEAR_BEFORE,
                            enqueue=False,
                        ),
                        timeout_s=rt.mesh_update_request_timeout_s,
                    )
                    if not marker_res.get("ok", False):
                        raise RuntimeError(
                            f"publish_targets(candidates) failed: {marker_res.get('error')}"
                        )
                    log.info(
                        "[geometry_representation_scan_print_loop] "
                        f"Candidate markers published: {len(preview_targets)} targets "
                        f"(yaml='{targets_yaml_path}', frame='{rt.frame_id}')"
                    )
                    if not self._wait_if_control_paused(
                        f"after candidate markers for {cycle_tag}"
                    ):
                        break

                if not preview_targets:
                    log.warn("[geometry_representation_scan_print_loop] No candidate targets generated; stopping loop.")
                    break

                refresh_runtime_config(f"before print prompt for {cycle_tag}")
                config_snapshot_path, config_snapshot_created = snapshot_cycle_config(
                    runtime_config_path,
                    cycle_root,
                )
                log.info(
                    "[geometry_representation_scan_print_loop] Cycle config "
                    f"{'saved' if config_snapshot_created else 'already exists'}: "
                    f"'{config_snapshot_path}'"
                )
                if not self._wait_if_control_paused(f"before print for {cycle_tag}"):
                    break

                if rt.debug_mode or rt.prompt_before_print:
                    gate_res = self.session.run_sync(
                        self.session.util.input(
                            prompt=(
                                f"[geometry_representation_scan_print_loop:{cycle_tag}] "
                                "Debug gate: press ENTER to print targets, "
                                "'r' + ENTER to rescan this cycle, or "
                                "'q' + ENTER to stop."
                            ),
                            enqueue=False,
                        ),
                        timeout_s=None,
                    )
                    gate_value = str(gate_res.get("metrics", {}).get("value", "")).strip().lower()
                    if gate_value == "q":
                        log.warn("[geometry_representation_scan_print_loop] Print execution stopped by user.")
                        break
                    if gate_value == "r":
                        rescan_cfg = self._load_runtime_config(runtime_config_path)
                        rt.__dict__.update(
                            self._parse_runtime_state(
                                rescan_cfg,
                                run_session_path,
                            ).__dict__
                        )
                        (
                            rescan_field_state_path,
                            rescan_field_center_raw,
                            rescan_field_center_xyz,
                        ) = self._load_configured_field_state(rt)
                        log.info(
                            "[geometry_representation_scan_print_loop] "
                            f"Runtime config reloaded for rescan of {cycle_tag}: "
                            f"{runtime_config_path}; field_state='"
                            f"{rescan_field_state_path}', center_raw=("
                            f"{rescan_field_center_raw[0]:.4f}, "
                            f"{rescan_field_center_raw[1]:.4f}, "
                            f"{rescan_field_center_raw[2]:.4f})"
                        )
                        mesh_path = self._rescan_cycle(
                            rt=rt,
                            runtime_config_path=runtime_config_path,
                            run_session_path=run_session_path,
                            run_session_arg=run_session_arg,
                            cycle_root=cycle_root,
                            cycle_tag=cycle_tag,
                            last_generated_targets=last_generated_targets,
                        )
                        field_state_path = rescan_field_state_path
                        field_center_xyz = rescan_field_center_xyz
                        log.info(
                            "[geometry_representation_scan_print_loop] "
                            f"Rescan complete for {cycle_tag}; regenerating candidates."
                        )
                        continue

                print_mode_norm = str(rt.print_mode or "auto").strip().lower()
                if print_mode_norm not in ("auto", "segments", "dots"):
                    raise RuntimeError("print_mode must be one of: auto, segments, dots")

                use_segments = bool(preview_segments) if print_mode_norm == "auto" else (print_mode_norm == "segments")
                if use_segments and not preview_segments:
                    raise RuntimeError("print_mode=segments but generated YAML has no segments.")

                if use_segments:
                    log.info(
                        f"[geometry_representation_scan_print_loop] Printing {len(preview_segments)} segments "
                        f"(print_mode={print_mode_norm})."
                    )
                    print_res = self.print_session.run_print_segments(
                        segments=preview_segments,
                        segment_steps=rt.segment_steps,
                        steps_per_mm3=rt.extrusion_steps_per_mm3,
                        steps_per_second=rt.segment_steps_per_second,
                        approach_z_offset_m=rt.segment_approach_z_offset_m,
                        travel_z_offset_m=rt.segment_travel_z_offset_m,
                        approach_vel_scale=rt.segment_approach_vel_scale,
                        travel_vel_scale=rt.segment_travel_vel_scale,
                        print_vel_scale=rt.segment_print_vel_scale,
                        accel_scale=rt.segment_accel_scale,
                        post_segment_wait_s=rt.post_segment_wait_s,
                        post_segment_retract_steps=rt.post_segment_retract_steps,
                        post_segment_retract_speed=rt.post_segment_retract_speed,
                        timeout_s=rt.timeout_s,
                        joint_current_log_dir=(
                            f"{cycle_root}/joint_current" if rt.log_joint_currents else None
                        ),
                        joint_current_log_label=f"{cycle_tag}:segments",
                    )
                else:
                    log.info(
                        f"[geometry_representation_scan_print_loop] Printing {len(preview_targets)} dots "
                        f"(print_mode={print_mode_norm}, net_dot_steps={rt.dot_steps}, "
                        f"retract_steps={rt.post_dot_retract_steps})."
                    )
                    print_res = self.print_session.run_print_dots_targets(
                        targets=preview_targets,
                        volumes_mm3=preview_volumes_mm3,
                        dot_steps=rt.dot_steps,
                        steps_per_mm3=rt.extrusion_steps_per_mm3,
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
                        joint_current_log_dir=(
                            f"{cycle_root}/joint_current" if rt.log_joint_currents else None
                        ),
                        joint_current_log_label=f"{cycle_tag}:dots",
                    )
                if not print_res.get("ok", False):
                    log.warn(
                        "[geometry_representation_scan_print_loop] Print stage returned non-fatal failure: "
                        f"stage={print_res.get('stage')} error={print_res.get('error', 'unknown')}"
                    )
                    break
                printed_count = int(print_res.get("printed", 0))
                skipped_count = int(print_res.get("skipped", 0))
                if preview_segments:
                    log.info(
                        "[geometry_representation_scan_print_loop] Print segments completed: "
                        f"printed={printed_count} skipped={skipped_count} / segments={print_res.get('segments', 0)}"
                    )
                else:
                    log.info(
                        "[geometry_representation_scan_print_loop] Print dots completed: "
                        f"printed={printed_count} skipped={skipped_count} / targets={print_res.get('targets', 0)}"
                    )

                if rt.contour_offset_estimation:
                    try:
                        pending_contour_offset_record = self.contour_offset_logger.begin_cycle(
                            cycle_root=cycle_root,
                            cycle_number=cycle_number,
                            phi_contour_path=str(
                                cand_metrics.get("debug_contour_ply_path", "")
                            ),
                            candidate_settings={
                                "candidate_mode": rt.candidate_mode,
                                "bead_height_mm": rt.candidate_bead_height_mm,
                                "target_output_yaw_deg": (
                                    rt.oriented_base_to_world_yaw_deg
                                ),
                            },
                            dot_targets=preview_targets if not use_segments else (),
                            segments=preview_segments if use_segments else (),
                        )
                        log.info(
                            "[contour_offset] Cycle opened: "
                            f"'{pending_contour_offset_record}' phi_contour='"
                            f"{cand_metrics.get('debug_contour_ply_path', '')}'"
                        )
                    except Exception as exc:
                        pending_contour_offset_record = None
                        log.warn(
                            "[contour_offset] Failed to open cycle; printing is unaffected. "
                            f"error='{exc}'"
                        )
                last_generated_targets = list(preview_targets)
                if not self._wait_if_control_paused(f"after print for {cycle_tag}"):
                    break

                print_cycle_index += 1

                refresh_runtime_config(f"before scan prompt after {cycle_tag}")

                if rt.max_cycles > 0 and print_cycle_index >= rt.max_cycles:
                    log.info(f"[geometry_representation_scan_print_loop] Reached max_cycles={rt.max_cycles}.")
                    break

                next_cycle_number = first_print_cycle_number + print_cycle_index
                next_cycle_tag = f"cycle_{next_cycle_number:04d}"
                next_cycle_root = self._run_session_child(run_session_path, f"field_loop/{next_cycle_tag}")
                next_cycle_scan_capture_folder = f"{next_cycle_root}/scan"
                next_cycle_scan_folder = f"field_loop/{next_cycle_tag}/scan"

                cycle_scan_z_off, z_msg = self._cycle_scan_height(
                    rt,
                    last_generated_targets,
                )

                log.info(
                    f"[geometry_representation_scan_print_loop] ===== scan_for_{next_cycle_tag} ===== "
                    f"scan_capture='{next_cycle_scan_capture_folder}' "
                    f"scan_folder='{next_cycle_scan_folder}' "
                    f"center=({rt.grid_center_x:.4f}, {rt.grid_center_y:.4f}, {rt.grid_center_z:.4f}) "
                    f"grid={rt.grid_nx}x{rt.grid_ny} size={1000.0*rt.grid_width:.1f}x"
                    f"{1000.0*rt.grid_height:.1f}mm use_tf_orientation={rt.grid_use_tf_orientation} {z_msg}"
                )

                if not rt.enable_scan:
                    log.info(
                        f"[geometry_representation_scan_print_loop] scan_for_{next_cycle_tag} skipped because enable_scan=false."
                    )
                    scan_targets = []
                else:
                    if not self._prompt_before_scan_if_enabled(
                        enabled=rt.prompt_before_scan,
                        label=f"scan_for_{next_cycle_tag}",
                    ):
                        log.info("[geometry_representation_scan_print_loop] Stopped by user before scan.")
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
                        vel_scale=rt.scan_vel_scale,
                        accel_scale=rt.scan_accel_scale,
                        settle_s=rt.scan_settle_s,
                    )
                    if not scan_targets:
                        raise RuntimeError(f"Scan produced 0 targets for scan_for_{next_cycle_tag}.")
                    if not self._wait_if_control_paused(f"after scan_for_{next_cycle_tag}"):
                        break

                if rt.enable_scan and rt.run_reconstruction:
                    mesh_path, _ = self._run_reconstruction_for_scan(
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
                        "[geometry_representation_scan_print_loop] run_reconstruction=False after scan; "
                        "next cycle will reuse the previous scan mesh."
                    )
                    if pending_contour_offset_record is not None:
                        try:
                            self.contour_offset_logger.close_without_post_contour(
                                pending_contour_offset_record,
                                reason="scan or reconstruction disabled",
                            )
                        except Exception as exc:
                            log.warn(
                                "[contour_offset] Failed to close unavailable observation. "
                                f"error='{exc}'"
                            )
                        finally:
                            pending_contour_offset_record = None

            if rt.home_after:
                self.session.run_sync(
                    self.session.motion.home(enqueue=False),
                    timeout_s=rt.timeout_s,
                )
                self._wait_if_control_paused("after home_after")

        except TimeoutError:
            log.error("[geometry_representation_scan_print_loop] Sequence timed out.")
        except Exception as exc:
            log.error(f"[geometry_representation_scan_print_loop] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    @staticmethod
    def _cycle_scan_height(
        rt: SimpleNamespace,
        last_generated_targets: List[PoseStamped],
    ) -> tuple[float, str]:
        if last_generated_targets:
            highest_prev_z = max(
                float(target.pose.position.z)
                for target in last_generated_targets
            )
            abs_scan_z = highest_prev_z + float(rt.grid_z_off)
            cycle_scan_z_off = abs_scan_z - float(rt.grid_center_z)
            message = (
                f"highest_generated_z={highest_prev_z:.4f}m + "
                f"grid_z_off={rt.grid_z_off:.4f}m "
                f"=> abs_scan_z={abs_scan_z:.4f}m, "
                f"center_z={rt.grid_center_z:.4f}m "
                f"=> applied_z_off={cycle_scan_z_off:.4f}m"
            )
            return cycle_scan_z_off, message

        cycle_scan_z_off = float(rt.grid_z_off)
        abs_scan_z = float(rt.grid_center_z) + cycle_scan_z_off
        message = (
            f"fallback(no printed targets): z_off={cycle_scan_z_off:.4f}m "
            f"=> abs_scan_z={abs_scan_z:.4f}m"
        )
        return cycle_scan_z_off, message

    def _load_configured_field_state(
        self,
        rt: SimpleNamespace,
    ) -> tuple[str, tuple[float, float, float], tuple[float, float, float]]:
        field_state_path = str(rt.resume_field_state_path).strip()
        if not field_state_path:
            raise RuntimeError("resume_field_state_path is required.")

        field_center_raw = self.session.compute_field_center_from_state(
            field_state_path
        )
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
        return field_state_path, field_center_raw, field_center_xyz

    def _rescan_cycle(
        self,
        *,
        rt: SimpleNamespace,
        runtime_config_path: str,
        run_session_path: Path,
        run_session_arg: str,
        cycle_root: str,
        cycle_tag: str,
        last_generated_targets: List[PoseStamped],
    ) -> str:
        if not rt.enable_scan:
            raise RuntimeError(
                f"Cannot rescan {cycle_tag}: enable_scan=false in the reloaded config."
            )
        if not rt.run_reconstruction:
            raise RuntimeError(
                f"Cannot rescan {cycle_tag}: run_reconstruction=false in the reloaded "
                "config; regenerated candidates require a fresh mesh."
            )

        cycle_root_path = Path(cycle_root).expanduser().resolve()
        field_loop_root = (run_session_path / "field_loop").resolve()
        try:
            relative_cycle = cycle_root_path.relative_to(field_loop_root)
        except ValueError as exc:
            raise RuntimeError(
                f"Refusing to replace rescan files outside {field_loop_root}: "
                f"{cycle_root_path}"
            ) from exc
        if relative_cycle.parts != (cycle_tag,):
            raise RuntimeError(
                "Refusing to replace rescan files because the cycle path does not "
                f"match {cycle_tag}: {cycle_root_path}"
            )

        scan_path = cycle_root_path / "scan"
        candidates_path = cycle_root_path / "candidates"
        config_snapshot_path = cycle_root_path / "config" / Path(
            runtime_config_path
        ).name
        for output_path in (scan_path, candidates_path):
            if output_path.exists():
                shutil.rmtree(output_path)
        if config_snapshot_path.exists():
            config_snapshot_path.unlink()

        capture_folder = f"{cycle_root_path}/scan"
        scan_folder = f"field_loop/{cycle_tag}/scan"
        cycle_scan_z_off, z_message = self._cycle_scan_height(
            rt,
            last_generated_targets,
        )
        self.get_logger().warn(
            "[geometry_representation_scan_print_loop] "
            f"Replacing scan and candidate outputs for {cycle_tag}: "
            f"scan='{scan_path}', candidates='{candidates_path}', {z_message}"
        )

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
            capture_folder=capture_folder,
            timeout_s=rt.timeout_s,
            frame_id=rt.frame_id,
            vel_scale=rt.scan_vel_scale,
            accel_scale=rt.scan_accel_scale,
            settle_s=rt.scan_settle_s,
        )
        if not scan_targets:
            raise RuntimeError(f"Rescan produced 0 targets for {cycle_tag}.")
        if not self._wait_if_control_paused(f"after rescan for {cycle_tag}"):
            raise RuntimeError(f"Rescan interrupted after capture for {cycle_tag}.")

        mesh_path, _ = self._run_reconstruction_for_scan(
            session_path=run_session_arg,
            scan_folder=scan_folder,
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
        if not self._wait_if_control_paused(
            f"after reconstruction for rescan of {cycle_tag}"
        ):
            raise RuntimeError(f"Rescan interrupted after reconstruction for {cycle_tag}.")
        return mesh_path

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
        vel_scale: float,
        accel_scale: float,
        settle_s: float,
    ) -> List[PoseStamped]:
        scan_type_norm = str(scan_type or "grid_sweep").strip().lower()
        common = self._scan_common_kwargs(
            timeout_s=timeout_s,
            vel_scale=vel_scale,
            accel_scale=accel_scale,
            settle_s=settle_s,
        )

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
            f"[geometry_representation_scan_print_loop] Scan complete: type={scan_type_norm}, "
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
                    f"[geometry_representation_scan_print_loop:{label}] Press ENTER to scan "
                    "(type 'q' + ENTER to stop)."
                ),
                enqueue=False,
            ),
            timeout_s=None,
        )
        value = str(res.get("metrics", {}).get("value", "")).strip().lower()
        return value != "q"

    @staticmethod
    def _scan_common_kwargs(
        *,
        timeout_s: Optional[float],
        vel_scale: float,
        accel_scale: float,
        settle_s: float,
    ) -> Dict[str, Any]:
        return {
            "motion": SCAN_MOTION,
            "eef_link": SCAN_EEF_LINK,
            "do_home": False,
            "vel_scale": float(vel_scale),
            "accel_scale": float(accel_scale),
            "timeout_s": timeout_s,
            "settle_s": float(settle_s),
            "prompt": None,
            "debug": False,
            "rgb": True,
            "depth": True,
            "ir": True,
            "pose": True,
            "publish_markers": True,
            "axis_length": 0.05,
            "axis_radius": 0.003,
            "clear_markers_before": True,
            "clear_markers_after": True,
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

    def _current_eef_pose(self, *, cfg: Dict[str, Any], frame_id: str, timeout_s: Optional[float]):
        res = self.scan_session.run_sync(
            self.scan_session.camera.get_pose(
                eef=SCAN_EEF_LINK,
                base_frame=str(frame_id or "world"),
                use_tf=True,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if res.get("ok", False) and "pose" in res:
            return res["pose"]
        self.get_logger().warn("[geometry_representation_scan_print_loop] Current EEF pose unavailable for scan orientation.")
        return None

    def _set_fields_node_candidate_params(self, rt: SimpleNamespace) -> None:
        params = [
            Parameter("target_output_mode", value=rt.print_mode),
            Parameter("candidate_width_mode", value=rt.candidate_width_mode),
            Parameter("candidate_bead_width_mm", value=rt.candidate_bead_width_mm),
            Parameter("candidate_width_field_path", value=rt.candidate_width_field_path),
            Parameter(
                "candidate_bead_width_min_mm",
                value=rt.candidate_bead_width_min_mm,
            ),
            Parameter(
                "candidate_bead_width_max_mm",
                value=rt.candidate_bead_width_max_mm,
            ),
            Parameter(
                "candidate_bead_overlap_mm",
                value=rt.candidate_bead_overlap_mm,
            ),
            Parameter(
                "candidate_volume_factor",
                value=rt.candidate_volume_factor,
            ),
            Parameter(
                "candidate_segment_start_offset_mm",
                value=rt.candidate_segment_start_offset_mm,
            ),
            Parameter("use_collision", value=rt.use_collision),
            Parameter(
                "collision_exclusion_radius_mm",
                value=rt.collision_exclusion_radius_mm,
            ),
            Parameter(
                "collision_threshold_mm",
                value=rt.collision_threshold_mm,
            ),
            Parameter(
                "collision_samples_per_segment",
                value=rt.collision_samples_per_segment,
            ),
            Parameter(
                "collision_tcp_exclusion_radius_mm",
                value=rt.collision_tcp_exclusion_radius_mm,
            ),
            Parameter(
                "collision_extruder_mesh_path",
                value=rt.collision_extruder_mesh_path,
            ),
        ]
        node_name = set_remote_parameters(
            self,
            remote_node_name=FIELDS_NODE_NAME,
            params=params,
            timeout_s=CANDIDATE_PARAM_TIMEOUT_S,
            label="candidate width",
        )
        self.get_logger().info(
            "[geometry_representation_scan_print_loop] Candidate width params set: "
            f"node={node_name} mode={rt.candidate_width_mode} "
            f"fixed_width_mm={rt.candidate_bead_width_mm:.3f} "
            f"range_mm=({rt.candidate_bead_width_min_mm:.3f}, "
            f"{rt.candidate_bead_width_max_mm:.3f}) "
            f"overlap_mm={rt.candidate_bead_overlap_mm:.3f} "
            f"volume_factor={rt.candidate_volume_factor:.6f} "
            f"target_output_mode={rt.print_mode} "
            f"segment_start_offset_mm={rt.candidate_segment_start_offset_mm:.3f} "
            f"use_collision={rt.use_collision} "
            f"collision_exclusion_radius_mm={rt.collision_exclusion_radius_mm:.3f} "
            f"collision_threshold_mm={rt.collision_threshold_mm:.3f} "
            f"collision_samples={rt.collision_samples_per_segment} "
            f"collision_tcp_exclusion_radius_mm={rt.collision_tcp_exclusion_radius_mm:.3f} "
            f"field='{rt.candidate_width_field_path}'"
        )

    def _parse_runtime_state(self, cfg: Dict[str, Any], run_session_path: Path) -> SimpleNamespace:
        home_before_scan = self._cfg_bool(cfg, "home_before_scan")
        home_after = self._cfg_bool(cfg, "home_after")
        timeout_s = self._cfg_optional_timeout(cfg, "timeout_s")
        debug_mode = self._cfg_bool(cfg, "debug_mode")
        prompt_before_print = self._cfg_bool(cfg, "prompt_before_print")
        prompt_before_scan = self._cfg_bool(cfg, "prompt_before_scan")
        enable_scan = self._cfg_bool(cfg, "enable_scan")
        contour_offset_estimation = self._cfg_bool(cfg, "contour_offset_estimation")

        frame_id = SCAN_FRAME_ID
        scan_eef_link = SCAN_EEF_LINK
        scan_type = self._cfg_str(cfg, "scan_type").strip().lower()
        if scan_type not in ("grid_sweep", "half_cylinder", "half_cylinder_side_caps"):
            raise ValueError(
                "scan_type must be one of: grid_sweep, half_cylinder, "
                "half_cylinder_side_caps"
            )
        scan_vel_scale = self._cfg_float(cfg, "scan_vel_scale")
        scan_accel_scale = self._cfg_float(cfg, "scan_accel_scale")
        scan_settle_s = self._cfg_float(cfg, "scan_settle_s")

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

        resume_field_state_path = self._cfg_str_allow_empty(cfg, "resume_field_state_path")
        resume_scan_mesh_path = self._cfg_str_allow_empty(cfg, "resume_scan_mesh_path")
        scan_before_generate_first_cycle = self._cfg_bool(cfg, "scan_before_generate_first_cycle")

        candidate_session_path = self._resolve_run_session_token(self._cfg_str(cfg, "candidate_session_path"), run_session_path)
        candidate_output_dir = self._resolve_run_session_token(self._cfg_str(cfg, "candidate_output_dir"), run_session_path)
        candidate_request_timeout_s = self._cfg_float(cfg, "candidate_request_timeout_s")
        candidate_mode = self._cfg_str(cfg, "candidate_mode").strip().lower()
        if candidate_mode not in ("z_lift", "gradient_lift"):
            raise ValueError(
                "geometry_representation_scan_print_loop supports candidate_mode "
                "'z_lift' or 'gradient_lift'."
            )
        candidate_beads_per_step = self._cfg_int(cfg, "candidate_beads_per_step")
        candidate_bead_separation_mm = self._cfg_float(cfg, "candidate_bead_separation_mm")
        candidate_bead_height_mm = self._cfg_float(cfg, "candidate_bead_height_mm")
        candidate_width_mode = self._cfg_str(cfg, "candidate_width_mode").strip().lower()
        if candidate_width_mode not in ("fixed", "field"):
            raise ValueError("candidate_width_mode must be 'fixed' or 'field'")
        candidate_bead_width_mm = self._cfg_float(cfg, "candidate_bead_width_mm")
        candidate_width_field_path = self._cfg_str_allow_empty(
            cfg,
            "candidate_width_field_path",
        )
        candidate_bead_width_min_mm = self._cfg_float(
            cfg,
            "candidate_bead_width_min_mm",
        )
        candidate_bead_width_max_mm = self._cfg_float(
            cfg,
            "candidate_bead_width_max_mm",
        )
        candidate_bead_overlap_mm = self._cfg_float(
            cfg,
            "candidate_bead_overlap_mm",
        )
        candidate_volume_factor = self._cfg_float(
            cfg,
            "candidate_volume_factor",
        )
        if candidate_volume_factor <= 0.0:
            raise ValueError("candidate_volume_factor must be > 0")
        candidate_segment_start_offset_mm = self._cfg_float(
            cfg,
            "candidate_segment_start_offset_mm",
        )
        if candidate_segment_start_offset_mm < 0.0:
            raise ValueError("candidate_segment_start_offset_mm must be >= 0")
        use_collision = self._cfg_bool(cfg, "use_collision")
        collision_exclusion_radius_mm = self._cfg_float(
            cfg,
            "collision_exclusion_radius_mm",
        )
        collision_threshold_mm = self._cfg_float(cfg, "collision_threshold_mm")
        collision_samples_per_segment = self._cfg_int(
            cfg,
            "collision_samples_per_segment",
        )
        collision_tcp_exclusion_radius_mm = self._cfg_float(
            cfg,
            "collision_tcp_exclusion_radius_mm",
        )
        collision_extruder_mesh_path = self._cfg_str(
            cfg,
            "collision_extruder_mesh_path",
        )
        if collision_exclusion_radius_mm < 0.0:
            raise ValueError("collision_exclusion_radius_mm must be >= 0")
        if collision_threshold_mm < 0.0:
            raise ValueError("collision_threshold_mm must be >= 0")
        if collision_samples_per_segment < 1:
            raise ValueError("collision_samples_per_segment must be >= 1")
        if collision_tcp_exclusion_radius_mm < 0.0:
            raise ValueError("collision_tcp_exclusion_radius_mm must be >= 0")
        if candidate_width_mode == "fixed" and candidate_bead_width_mm <= 0.0:
            raise ValueError("candidate_bead_width_mm must be > 0 in fixed mode")
        if candidate_width_mode == "field":
            if not candidate_width_field_path:
                raise ValueError(
                    "candidate_width_field_path is required in field width mode"
                )
            if (
                candidate_bead_width_min_mm <= 0.0
                or candidate_bead_width_max_mm < candidate_bead_width_min_mm
            ):
                raise ValueError(
                    "candidate bead width range must satisfy 0 < min <= max"
                )
            if candidate_bead_overlap_mm < 0.0:
                raise ValueError("candidate_bead_overlap_mm must be >= 0")
        oriented_targets_enable = self._cfg_bool(cfg, "oriented_targets_enable")
        oriented_tangent_sign = self._cfg_float(cfg, "oriented_tangent_sign")
        oriented_clamp_to_cone = self._cfg_bool(cfg, "oriented_clamp_to_cone")
        oriented_cone_max_tilt_deg = self._cfg_float(cfg, "oriented_cone_max_tilt_deg")
        oriented_base_to_world_yaw_deg = self._cfg_float(cfg, "oriented_base_to_world_yaw_deg")
        print_mode = self._cfg_str(cfg, "print_mode")
        log_joint_currents = self._cfg_bool(cfg, "log_joint_currents")
        dot_steps = self._cfg_int(cfg, "dot_steps")
        extrusion_steps_per_mm3 = self._cfg_float(cfg, "extrusion_steps_per_mm3")
        if extrusion_steps_per_mm3 <= 0.0:
            raise ValueError("extrusion_steps_per_mm3 must be > 0")
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
        segment_steps = self._cfg_int(cfg, "segment_steps")
        if segment_steps <= 0:
            raise ValueError("segment_steps must be > 0")
        segment_steps_per_second = self._cfg_int(cfg, "segment_steps_per_second")
        if segment_steps_per_second <= 0:
            raise ValueError("segment_steps_per_second must be > 0")
        segment_approach_z_offset_m = self._cfg_float(cfg, "segment_approach_z_offset_m")
        segment_travel_z_offset_m = self._cfg_float(cfg, "segment_travel_z_offset_m")
        segment_approach_vel_scale = self._cfg_float(cfg, "segment_approach_vel_scale")
        segment_travel_vel_scale = self._cfg_float(cfg, "segment_travel_vel_scale")
        segment_print_vel_scale = self._cfg_float(cfg, "segment_print_vel_scale")
        segment_accel_scale = self._cfg_float(cfg, "segment_accel_scale")
        post_segment_wait_s = self._cfg_float(cfg, "post_segment_wait_s")
        post_segment_retract_steps = self._cfg_int(cfg, "post_segment_retract_steps")
        post_segment_retract_speed = self._cfg_int(cfg, "post_segment_retract_speed")
        max_cycles = self._cfg_int(cfg, "max_cycles")
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
            'contour_offset_estimation',
            'frame_id',
            'scan_eef_link',
            'scan_type',
            'scan_vel_scale',
            'scan_accel_scale',
            'scan_settle_s',
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
            'resume_field_state_path',
            'resume_scan_mesh_path',
            'scan_before_generate_first_cycle',
            'candidate_session_path',
            'candidate_output_dir',
            'candidate_request_timeout_s',
            'candidate_mode',
            'candidate_beads_per_step',
            'candidate_bead_separation_mm',
            'candidate_bead_height_mm',
            'candidate_width_mode',
            'candidate_bead_width_mm',
            'candidate_width_field_path',
            'candidate_bead_width_min_mm',
            'candidate_bead_width_max_mm',
            'candidate_bead_overlap_mm',
            'candidate_volume_factor',
            'candidate_segment_start_offset_mm',
            'use_collision',
            'collision_exclusion_radius_mm',
            'collision_threshold_mm',
            'collision_samples_per_segment',
            'collision_tcp_exclusion_radius_mm',
            'collision_extruder_mesh_path',
            'oriented_targets_enable',
            'oriented_tangent_sign',
            'oriented_clamp_to_cone',
            'oriented_cone_max_tilt_deg',
            'oriented_base_to_world_yaw_deg',
            'print_mode',
            'log_joint_currents',
            'dot_steps',
            'extrusion_steps_per_mm3',
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
            'segment_steps',
            'segment_steps_per_second',
            'segment_approach_z_offset_m',
            'segment_travel_z_offset_m',
            'segment_approach_vel_scale',
            'segment_travel_vel_scale',
            'segment_print_vel_scale',
            'segment_accel_scale',
            'post_segment_wait_s',
            'post_segment_retract_steps',
            'post_segment_retract_speed',
            'max_cycles'
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
                f"[geometry_representation_scan_print_loop] Runtime config reload failed ({reason}); "
                f"keeping previous config. error={exc}"
            )
            return current

        self.get_logger().info(
            f"[geometry_representation_scan_print_loop] Runtime config reloaded ({reason}): {config_path}"
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
    node = GeometryRepresentationScanPrintLoopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
