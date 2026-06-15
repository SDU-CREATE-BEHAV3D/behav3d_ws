#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_yaml_and_scan_sequence
"""

from __future__ import annotations

import math
import threading
from pathlib import Path, PurePosixPath
from typing import Any, Dict, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .src.print_session import PrintSession
from .src.scan_session import ScanSession

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

CONFIG_FILENAME = "print_yaml_and_scan_sequence_config.yaml"
SRC_CONFIG_PATH = f"/home/lab/behav3d_ws/src/behav3d_orchestrator/config/{CONFIG_FILENAME}"


class PrintYamlAndScanSequenceNode(Node):
    """
    Print ordered YAML targets in fixed-size chunks, then run a grid-sweep scan
    and optional reconstruction after each chunk.
    """

    def __init__(self):
        super().__init__("print_yaml_and_scan_sequence")

        self.print_session = PrintSession(self)
        self.scan_session = ScanSession(self)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self) -> None:
        log = self.get_logger()
        config_path = self._default_config_path()

        try:
            cfg = self._load_config(config_path)
        except Exception as exc:
            log.error(f"[print_yaml_and_scan] Failed to load config '{config_path}': {exc}")
            rclpy.shutdown()
            return

        yaml_path = self._cfg_str(cfg, "yaml_path")
        frame_id = self._cfg_str(cfg, "frame_id")
        points_per_cycle = self._cfg_int(cfg, "points_per_cycle")
        timeout_s = self._cfg_optional_timeout(cfg, "timeout_s")
        scan_timeout_s = self._cfg_optional_timeout(cfg, "scan_timeout_s")
        if scan_timeout_s is None:
            scan_timeout_s = timeout_s

        if points_per_cycle <= 0:
            log.error("Config key 'points_per_cycle' must be > 0.")
            rclpy.shutdown()
            return

        try:
            targets = self.print_session.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
            if not targets:
                raise ValueError(f"No valid targets found in YAML: {yaml_path}")

            log.info(
                "[print_yaml_and_scan] Starting sequence: "
                f"config='{config_path}', yaml='{yaml_path}', frame='{frame_id}', "
                f"targets={len(targets)}, points_per_cycle={points_per_cycle}, "
                f"scan_enabled={self._cfg_bool(cfg, 'enable_scan')}, "
                f"run_reconstruction={self._cfg_bool(cfg, 'run_reconstruction')}"
            )
            log.info(
                "[print_yaml_and_scan] YAML target preview: "
                f"{self._format_target_preview(targets, max_items=min(points_per_cycle, 7))}"
            )

            if self._cfg_bool(cfg, "home_before"):
                self.print_session.run_sync(
                    self.print_session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )

            total_printed = 0
            total_skipped = 0
            cycle_count = 0
            next_target_idx = 0
            stopped_by_user = False

            while rclpy.ok() and next_target_idx < len(targets):
                try:
                    new_cfg = self._load_config(config_path)
                    new_timeout_s = self._cfg_optional_timeout(new_cfg, "timeout_s")
                    new_scan_timeout_s = self._cfg_optional_timeout(new_cfg, "scan_timeout_s")
                    if new_scan_timeout_s is None:
                        new_scan_timeout_s = new_timeout_s
                    new_points_per_cycle = self._cfg_int(new_cfg, "points_per_cycle")
                    if new_points_per_cycle <= 0:
                        raise ValueError("Config key 'points_per_cycle' must be > 0.")
                    cfg = new_cfg
                    timeout_s = new_timeout_s
                    scan_timeout_s = new_scan_timeout_s
                    points_per_cycle = new_points_per_cycle
                except Exception as exc:
                    log.warn(
                        "[print_yaml_and_scan] Runtime config reload failed; keeping previous config. "
                        f"path='{config_path}' error='{exc}'"
                    )

                cfg_yaml_path = self._cfg_str(cfg, "yaml_path")
                cfg_frame_id = self._cfg_str(cfg, "frame_id")
                if cfg_yaml_path != yaml_path or cfg_frame_id != frame_id:
                    log.warn(
                        "[print_yaml_and_scan] Runtime yaml_path/frame_id changes are ignored "
                        "after targets are loaded. Restart the node to use them. "
                        f"loaded_yaml='{yaml_path}', loaded_frame='{frame_id}', "
                        f"current_yaml='{cfg_yaml_path}', current_frame='{cfg_frame_id}'"
                    )

                if not rclpy.ok():
                    break

                start_idx = next_target_idx
                end_idx = min(start_idx + points_per_cycle, len(targets))
                cycle_count += 1
                cycle_tag = f"cycle_{cycle_count:04d}"
                chunk = targets[start_idx:end_idx]
                next_target_idx = end_idx

                log.info(
                    f"[print_yaml_and_scan] ===== {cycle_tag} print ===== "
                    f"targets={start_idx + 1}..{end_idx}/{len(targets)} "
                    f"chunk_size={len(chunk)} "
                    f"preview={self._format_target_preview(chunk, max_items=len(chunk))}"
                )
                self._publish_yaml_targets(targets, cfg=cfg, timeout_s=timeout_s)

                if self._cfg_bool(cfg, "debug_mode") or self._cfg_bool(cfg, "prompt_before_print"):
                    if self._operator_requested_stop(
                        f"[print_yaml_and_scan:{cycle_tag}] Debug gate: press ENTER to continue this cycle "
                        "(type 'q' + ENTER to stop)."
                    ):
                        stopped_by_user = True
                        break

                print_res = self._print_chunk(chunk, cfg=cfg, timeout_s=timeout_s)
                if not print_res.get("ok", False):
                    log.error(
                        f"[print_yaml_and_scan] Print failed in {cycle_tag} "
                        f"stage={print_res.get('stage')}: {print_res.get('error', 'unknown')}"
                    )
                    break

                printed = int(print_res.get("printed", 0))
                skipped = int(print_res.get("skipped", 0))
                printed_targets = print_res.get("printed_targets", [])
                total_printed += printed
                total_skipped += skipped
                log.info(
                    f"[print_yaml_and_scan] {cycle_tag} print complete: "
                    f"printed={printed}, skipped={skipped}, requested={len(chunk)}, "
                    f"total_printed={total_printed}"
                )

                has_more_targets = end_idx < len(targets)
                should_scan = self._cfg_bool(cfg, "enable_scan") and (
                    has_more_targets or self._cfg_bool(cfg, "scan_after_final")
                )
                if not should_scan:
                    if self._cfg_bool(cfg, "enable_scan"):
                        log.info(
                            f"[print_yaml_and_scan] {cycle_tag} is final chunk; "
                            "scan skipped because scan_after_final=false."
                        )
                    continue

                if self._cfg_bool(cfg, "prompt_before_scan"):
                    if self._operator_requested_stop(
                        f"[print_yaml_and_scan:{cycle_tag}] Press ENTER to scan after this chunk "
                        "(type 'q' + ENTER to stop)."
                    ):
                        stopped_by_user = True
                        break

                scan_ok = self._run_scan_cycle(
                    cycle_tag=cycle_tag,
                    cfg=cfg,
                    timeout_s=scan_timeout_s,
                    printed_targets=printed_targets if isinstance(printed_targets, list) else [],
                )
                if not scan_ok:
                    break

            if stopped_by_user:
                log.warn("[print_yaml_and_scan] Sequence stopped by user.")

            log.info(
                "[print_yaml_and_scan] Sequence finished: "
                f"cycles={cycle_count}, printed={total_printed}, skipped={total_skipped}, "
                f"targets={len(targets)}"
            )

            if self._cfg_bool(cfg, "home_after"):
                self.print_session.run_sync(
                    self.print_session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )

        except TimeoutError:
            log.error("[print_yaml_and_scan] Sequence timed out.")
        except Exception as exc:
            log.error(f"[print_yaml_and_scan] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _print_chunk(
        self,
        targets: Sequence[PoseStamped],
        *,
        cfg: Dict[str, Any],
        timeout_s: Optional[float],
    ) -> dict:
        log = self.get_logger()
        target_list = list(targets)
        if not target_list:
            return {"ok": False, "stage": "parse_targets", "error": "Need at least 1 target."}

        print_eef = self._cfg_str(cfg, "print_eef_link")
        dot_steps = self._cfg_int(cfg, "dot_steps")
        dot_speed = self._cfg_int(cfg, "dot_speed")
        retract_steps = self._cfg_int(cfg, "post_dot_retract_steps")
        retract_speed = self._cfg_int(cfg, "post_dot_retract_speed")
        approach_z_offset_m = self._cfg_float(cfg, "approach_z_offset_m")
        dot_z_offset_m = self._cfg_float(cfg, "dot_z_offset_m")
        pre_dot_vel_scale = self._cfg_float(cfg, "pre_dot_vel_scale")
        dot_vel_scale = self._cfg_float(cfg, "dot_vel_scale")
        accel_scale = self._cfg_float(cfg, "print_accel_scale")
        dwell_s = self._cfg_float(cfg, "dwell_s")

        res = self.print_session.run_sync(self.print_session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
        if not res.get("ok", False):
            return {"ok": False, "stage": "set_lin", "error": res.get("error", "unknown")}

        res = self.print_session.run_sync(
            self.print_session.motion.setAcc(accel_scale, enqueue=False),
            timeout_s=timeout_s,
        )
        if not res.get("ok", False):
            return {"ok": False, "stage": "set_acc", "error": res.get("error", "unknown")}

        res = self.print_session.run_sync(
            self.print_session.motion.setSpd(dot_vel_scale, enqueue=False),
            timeout_s=timeout_s,
        )
        if not res.get("ok", False):
            return {"ok": False, "stage": "set_spd", "error": res.get("error", "unknown")}

        res = self.print_session.run_sync(
            self.print_session.motion.setEef(print_eef, enqueue=False),
            timeout_s=timeout_s,
        )
        if not res.get("ok", False):
            return {"ok": False, "stage": "set_print_eef", "error": res.get("error", "unknown")}

        log.info(
            f"[print_yaml_and_scan] Print context ready: eef='{print_eef}', "
            f"targets={len(target_list)}, retract_steps={retract_steps}, retract_speed={retract_speed}"
        )

        first_approach = self._with_z_offset(target_list[0], approach_z_offset_m)
        res = self.print_session.run_sync(
            self.print_session.motion.goto(
                pose=first_approach,
                eef=print_eef,
                motion="LIN",
                vel_scale=pre_dot_vel_scale,
                accel_scale=accel_scale,
                exec=True,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not res.get("ok", False):
            log.warn(
                "[print_yaml_and_scan] Initial print approach failed; continuing target-by-target. "
                f"error='{res.get('error', 'unknown')}'"
            )

        printed = 0
        skipped = 0
        failed_targets: list[dict[str, Any]] = []
        printed_targets: list[PoseStamped] = []
        for i, target in enumerate(target_list):
            above = self._with_z_offset(target, dot_z_offset_m)
            above_vel = pre_dot_vel_scale if i == 0 else dot_vel_scale

            res = self.print_session.run_sync(
                self.print_session.motion.goto(
                    pose=above,
                    eef=print_eef,
                    motion="LIN",
                    vel_scale=above_vel,
                    accel_scale=accel_scale,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                skipped += 1
                err = str(res.get("error", "unknown"))
                failed_targets.append({"index": i, "stage": "goto_above", "error": err})
                log.warn(f"[print_yaml_and_scan] Skip print target {i}: goto_above failed. error='{err}'")
                continue

            res = self.print_session.run_sync(
                self.print_session.motion.goto(
                    pose=target,
                    eef=print_eef,
                    motion="LIN",
                    vel_scale=dot_vel_scale,
                    accel_scale=accel_scale,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                skipped += 1
                err = str(res.get("error", "unknown"))
                failed_targets.append({"index": i, "stage": "goto_target", "error": err})
                log.warn(f"[print_yaml_and_scan] Skip print target {i}: goto_target failed. error='{err}'")
                continue

            res = self.print_session.run_sync(
                self.print_session.extruder.print_steps(
                    steps=dot_steps,
                    speed=dot_speed,
                    reverse=False,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                skipped += 1
                err = str(res.get("error", "unknown"))
                failed_targets.append({"index": i, "stage": "extrude", "error": err})
                log.warn(f"[print_yaml_and_scan] Skip print target {i}: extrude failed. error='{err}'")
                continue

            if retract_steps > 0:
                res = self.print_session.run_sync(
                    self.print_session.extruder.print_steps(
                        steps=retract_steps,
                        speed=retract_speed,
                        reverse=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    err = str(res.get("error", "unknown"))
                    failed_targets.append({"index": i, "stage": "retract", "error": err})
                    log.warn(f"[print_yaml_and_scan] Target {i} printed but retract failed. error='{err}'")

            if dwell_s > 0.0:
                self.print_session.run_sync(
                    self.print_session.util.wait(dwell_s, enqueue=False),
                    timeout_s=timeout_s,
                )

            lift_res = self.print_session.run_sync(
                self.print_session.motion.goto(
                    pose=above,
                    eef=print_eef,
                    motion="LIN",
                    vel_scale=dot_vel_scale,
                    accel_scale=accel_scale,
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not lift_res.get("ok", False):
                err = str(lift_res.get("error", "unknown"))
                failed_targets.append({"index": i, "stage": "lift", "error": err})
                log.warn(f"[print_yaml_and_scan] Target {i} printed but lift failed. error='{err}'")

            printed += 1
            printed_targets.append(target)

        return {
            "ok": True,
            "stage": "done",
            "targets": len(target_list),
            "printed": printed,
            "skipped": skipped,
            "failed_targets": failed_targets,
            "printed_targets": printed_targets,
        }

    def _publish_yaml_targets(
        self,
        targets: Sequence[PoseStamped],
        *,
        cfg: Dict[str, Any],
        timeout_s: Optional[float],
    ) -> None:
        if not self._cfg_bool(cfg, "print_publish_markers"):
            return
        self.print_session.run_sync(
            self.print_session.util.publish_targets(
                list(targets),
                axis_length=self._cfg_float(cfg, "print_axis_length"),
                axis_radius=self._cfg_float(cfg, "print_axis_radius"),
                clear_before=self._cfg_bool(cfg, "print_clear_markers_before"),
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        self.get_logger().info(
            f"[print_yaml_and_scan] Published YAML target markers: targets={len(targets)}"
        )

    def _run_scan_cycle(
        self,
        *,
        cycle_tag: str,
        cfg: Dict[str, Any],
        timeout_s: Optional[float],
        printed_targets: Sequence[PoseStamped],
    ) -> bool:
        log = self.get_logger()
        scan_root = self._cfg_str(cfg, "scan_root_folder")
        scan_capture_folder = f"{scan_root.rstrip('/')}/{cycle_tag}/scan"
        scan_folder = self._scan_folder_from_capture_folder(scan_capture_folder)

        grid_width = self._cfg_float(cfg, "grid_width")
        grid_height = self._cfg_float(cfg, "grid_height")
        grid_center_x = self._cfg_float(cfg, "grid_center_x")
        grid_center_y = self._cfg_float(cfg, "grid_center_y")
        grid_center_z = self._cfg_float(cfg, "grid_center_z")
        grid_z_off = self._cfg_float(cfg, "grid_z_off")
        grid_nx = self._cfg_int(cfg, "grid_nx")
        grid_ny = self._cfg_int(cfg, "grid_ny")
        scan_abs_z = grid_center_z + grid_z_off
        if self._cfg_bool(cfg, "scan_follow_printed_z"):
            if printed_targets:
                highest_printed_z = max(float(ps.pose.position.z) for ps in printed_targets)
                grid_center_z = highest_printed_z
                scan_abs_z = grid_center_z + grid_z_off
                z_msg = (
                    f"adaptive_z=true highest_printed_z={highest_printed_z:.4f}m "
                    f"=> abs_scan_z={scan_abs_z:.4f}m"
                )
            else:
                z_msg = (
                    "adaptive_z=true but no printed targets reported; "
                    f"using configured abs_scan_z={scan_abs_z:.4f}m"
                )
        else:
            z_msg = f"adaptive_z=false abs_scan_z={scan_abs_z:.4f}m"

        log.info(
            f"[print_yaml_and_scan] ===== scan_for_{cycle_tag} ===== "
            f"scan_capture='{scan_capture_folder}' scan_folder='{scan_folder}' "
            f"grid={grid_nx}x{grid_ny} size={1000.0 * grid_width:.1f}x"
            f"{1000.0 * grid_height:.1f}mm "
            f"center=({grid_center_x:.4f}, {grid_center_y:.4f}, {grid_center_z:.4f}) "
            f"z_off={grid_z_off:.4f}m "
            f"use_tf_orientation={self._cfg_bool(cfg, 'grid_use_tf_orientation')} "
            f"{z_msg}"
        )

        if not self._prepare_scan_context(timeout_s=timeout_s):
            return False

        scan_res = self.scan_session.run_grid_scan(
            capture_folder=scan_capture_folder,
            width=grid_width,
            height=grid_height,
            center_x=grid_center_x,
            center_y=grid_center_y,
            center_z=grid_center_z,
            z_off=grid_z_off,
            nx=grid_nx,
            ny=grid_ny,
            row_major=self._cfg_bool(cfg, "grid_row_major"),
            frame_id=self._cfg_str(cfg, "frame_id"),
            use_tf_orientation=self._cfg_bool(cfg, "grid_use_tf_orientation"),
            motion=self._cfg_str(cfg, "scan_motion"),
            eef_link=self._cfg_str(cfg, "scan_eef_link"),
            do_home=self._cfg_bool(cfg, "scan_do_home"),
            vel_scale=self._cfg_float(cfg, "scan_vel_scale"),
            accel_scale=self._cfg_float(cfg, "scan_accel_scale"),
            timeout_s=timeout_s,
            settle_s=self._cfg_float(cfg, "scan_settle_s"),
            prompt=self._cfg_str_allow_empty(cfg, "scan_prompt") or None,
            debug=self._cfg_bool(cfg, "scan_debug"),
            rgb=self._cfg_bool(cfg, "scan_rgb"),
            depth=self._cfg_bool(cfg, "scan_depth"),
            ir=self._cfg_bool(cfg, "scan_ir"),
            pose=self._cfg_bool(cfg, "scan_pose"),
            publish_markers=self._cfg_bool(cfg, "scan_publish_markers"),
            axis_length=self._cfg_float(cfg, "scan_axis_length"),
            axis_radius=self._cfg_float(cfg, "scan_axis_radius"),
            clear_markers_before=self._cfg_bool(cfg, "scan_clear_markers_before"),
            clear_markers_after=self._cfg_bool(cfg, "scan_clear_markers_after"),
        )
        if not scan_res.get("ok", False):
            log.error(
                f"[print_yaml_and_scan] Scan failed for {cycle_tag} "
                f"stage={scan_res.get('stage')}: {scan_res.get('error', 'unknown')}"
            )
            return False

        log.info(
            f"[print_yaml_and_scan] {cycle_tag} scan complete: "
            f"targets={len(scan_res.get('targets', []))}, "
            f"planned={scan_res.get('plan_ok', 0)}, "
            f"executed={scan_res.get('exec_ok', 0)}, "
            f"captures={scan_res.get('captures_ok', 0)}"
        )

        if not self._cfg_bool(cfg, "run_reconstruction"):
            return True

        rec = self.scan_session.run_reconstruction_for_scan(
            scan_folder=scan_folder,
            session_path="@session",
            reconstruct_device=self._cfg_str(cfg, "reconstruct_device"),
            reconstruct_request_timeout_s=self._cfg_float(cfg, "reconstruct_request_timeout_s"),
            wait_reconstruction_outputs=self._cfg_bool(cfg, "wait_reconstruction_outputs"),
            color_to_depth_wait_timeout_s=self._cfg_float(cfg, "color_to_depth_wait_timeout_s"),
            tsdf_wait_timeout_s=self._cfg_float(cfg, "tsdf_wait_timeout_s"),
            update_world_mesh=self._cfg_bool(cfg, "update_world_mesh"),
            tsdf_center_crop_enable=self._cfg_bool(cfg, "tsdf_center_crop_enable"),
            tsdf_center_crop_width=self._cfg_int(cfg, "tsdf_center_crop_width"),
            tsdf_center_crop_height=self._cfg_int(cfg, "tsdf_center_crop_height"),
            tsdf_center_crop_apply_to_depth=self._cfg_bool(cfg, "tsdf_center_crop_apply_to_depth"),
            tsdf_aabb_crop_enable=self._cfg_bool(cfg, "tsdf_aabb_crop_enable"),
            tsdf_aabb_crop_min=self._cfg_float_list(cfg, "tsdf_aabb_crop_min"),
            tsdf_aabb_crop_max=self._cfg_float_list(cfg, "tsdf_aabb_crop_max"),
            tsdf_param_update_timeout_s=self._cfg_float(cfg, "tsdf_param_update_timeout_s"),
        )
        if not rec.get("ok", False):
            log.error(
                f"[print_yaml_and_scan] Reconstruction failed for {cycle_tag} "
                f"stage={rec.get('stage')}: {rec.get('error', 'unknown')}"
            )
            return False

        log.info(
            f"[print_yaml_and_scan] {cycle_tag} reconstruction complete: "
            f"mesh='{rec.get('mesh_path', '')}', rgb_ply='{rec.get('rgb_ply_path', '')}'"
        )
        return True

    def _operator_requested_stop(self, prompt: str) -> bool:
        res = self.print_session.run_sync(
            self.print_session.util.input(prompt=prompt, enqueue=False),
            timeout_s=None,
        )
        value = str(res.get("metrics", {}).get("value", "")).strip().lower()
        return value == "q"

    def _prepare_scan_context(self, *, timeout_s: Optional[float]) -> bool:
        off_res = self.print_session.run_sync(
            self.print_session.extruder.setExtruder(
                False,
                reverse=False,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not off_res.get("ok", False):
            self.get_logger().error(
                "[print_yaml_and_scan] Refusing to scan because extruder OFF failed: "
                f"{off_res.get('error', 'unknown')}"
            )
            return False

        self.get_logger().info("[print_yaml_and_scan] Scan context ready: extruder OFF.")
        return True

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

    @staticmethod
    def _format_target_preview(targets: Sequence[PoseStamped], *, max_items: int = 7) -> str:
        parts = []
        for i, ps in enumerate(list(targets)[: max(0, int(max_items))]):
            p = ps.pose.position
            parts.append(f"{i}:({p.x:.4f},{p.y:.4f},{p.z:.4f})")
        if len(targets) > len(parts):
            parts.append(f"... +{len(targets) - len(parts)}")
        return "[" + ", ".join(parts) + "]"

    def _default_config_path(self) -> str:
        return SRC_CONFIG_PATH

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        if yaml is None:
            raise RuntimeError("PyYAML is not available; cannot load sequence config.")

        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"Config file not found: {path}")

        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        params = self._extract_config_params(raw)
        if not params:
            raise ValueError(
                "Config does not contain parameters. Expected "
                "'print_yaml_and_scan_sequence: ros__parameters: ...'."
            )
        return dict(params)

    def _extract_config_params(self, raw: Any) -> Dict[str, Any]:
        if not isinstance(raw, dict):
            return {}

        node_name = str(self.get_name()).strip()
        for key in (node_name, f"/{node_name}"):
            block = raw.get(key)
            if isinstance(block, dict):
                params = block.get("ros__parameters")
                if isinstance(params, dict):
                    return dict(params)

        params = raw.get("ros__parameters")
        if isinstance(params, dict):
            return dict(params)

        return dict(raw)

    @staticmethod
    def _scan_folder_from_capture_folder(capture_folder: str) -> str:
        name = str(capture_folder or "").strip()
        if name in ("@session", "@"):
            return ""
        if name.startswith("@session/"):
            name = name[len("@session/") :]
        elif name.startswith("@/"):
            name = name[2:]
        elif name.startswith("./"):
            name = name[2:]
        return PurePosixPath(name).as_posix()

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
    def _cfg_float_list(cls, cfg: Dict[str, Any], key: str) -> list[float]:
        return cls._as_float_list(cls._require(cfg, key))

    @classmethod
    def _cfg_optional_timeout(cls, cfg: Dict[str, Any], key: str) -> Optional[float]:
        val = cls._cfg_float(cfg, key)
        if val <= 0.0:
            return None
        return val

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
        except (TypeError, ValueError):
            raise ValueError(f"expected finite float, got '{value}'")
        if not math.isfinite(out):
            raise ValueError(f"expected finite float, got '{value}'")
        return out

    @staticmethod
    def _as_str(value: Any) -> str:
        if value is None:
            raise ValueError("expected non-empty string, got null")
        text = str(value).strip()
        if not text:
            raise ValueError("expected non-empty string, got empty")
        return text

    @classmethod
    def _as_float_list(cls, value: Any) -> list[float]:
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"expected float list, got '{value}'")
        out = [cls._as_float(item) for item in value]
        if not out:
            raise ValueError("expected non-empty float list")
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PrintYamlAndScanSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
