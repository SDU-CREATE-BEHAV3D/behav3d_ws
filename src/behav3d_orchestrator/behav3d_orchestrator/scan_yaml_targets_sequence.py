#!/usr/bin/env python3
"""Scan explicit YAML planes, capture at each reachable target, then reconstruct."""

from __future__ import annotations

import math
import threading
from pathlib import Path, PurePosixPath
from typing import Any, Dict, Optional, Sequence

import rclpy
from behav3d_utils import target_builder as tb
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .src.scan_session import STALE_FRAME_ERROR_PREFIX, ScanSession
from .src.yaml_session import YamlSession

try:
    import yaml
except ImportError:  # pragma: no cover - reported clearly at runtime
    yaml = None


CONFIG_FILENAME = "scan_yaml_targets_sequence_config.yaml"


class ScanYamlTargetsSequenceNode(Node):
    """Execute LIN scan targets loaded from a plane-oriented YAML file."""

    def __init__(self) -> None:
        super().__init__("scan_yaml_targets_sequence")
        self.declare_parameter("config_path", self._default_config_path())

        # Both sessions share the node's single /behav3d/control_state gate.
        self.yaml_session = YamlSession(self)
        self.scan_session = ScanSession(self)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self) -> None:
        log = self.get_logger()
        config_path = str(self.get_parameter("config_path").value).strip()
        latest_cfg: Dict[str, Any] = {}
        publish_markers = False
        timeout_s: Optional[float] = None
        captures_ok = 0

        try:
            latest_cfg = self._load_config(config_path)
            timeout_s = self._cfg_optional_timeout(latest_cfg, "timeout_s")

            yaml_path = self._cfg_str(latest_cfg, "yaml_path")
            frame_id = self._cfg_str(latest_cfg, "frame_id")
            capture_folder = self._cfg_str(latest_cfg, "capture_folder")
            configured_scan_folder = self._cfg_str_allow_empty(latest_cfg, "scan_folder")
            scan_folder = configured_scan_folder or self._scan_folder_from_capture_folder(capture_folder)
            session_path = self._cfg_str(latest_cfg, "session_path")
            eef_link = self._cfg_str(latest_cfg, "eef_link")

            targets = self.yaml_session.parse_yaml_targets(
                yaml_path=yaml_path,
                frame_id=frame_id,
            )
            if not targets:
                raise ValueError(f"No valid targets found in YAML: {yaml_path}")

            log.info(
                "[scan_yaml_targets] Starting sequence: "
                f"config='{config_path}', yaml='{yaml_path}', frame='{frame_id}', "
                f"targets={len(targets)}, capture_folder='{capture_folder}', "
                f"scan_folder='{scan_folder}'"
            )

            if self._cfg_bool(latest_cfg, "home_before"):
                home_res = self.scan_session.run_sync(
                    self.scan_session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )
                if not home_res.get("ok", False):
                    raise RuntimeError(f"home_before failed: {home_res.get('error', 'unknown')}")

            reference_pose = self._current_eef_pose(
                eef_link=eef_link,
                frame_id=frame_id,
                timeout_s=timeout_s,
            )
            targets = self._match_target_z_to_eef_orientation(
                targets,
                reference_pose=reference_pose,
            )
            log.info(
                "[scan_yaml_targets] Target roll matched to current EEF orientation: "
                f"eef='{eef_link}', frame='{frame_id}'."
            )

            publish_markers = self._cfg_bool(latest_cfg, "publish_markers")
            if publish_markers:
                marker_res = self.scan_session.run_sync(
                    self.scan_session.util.publish_targets(
                        targets,
                        axis_length=self._cfg_float(latest_cfg, "axis_length"),
                        axis_radius=self._cfg_float(latest_cfg, "axis_radius"),
                        clear_before=self._cfg_bool(latest_cfg, "clear_markers_before"),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not marker_res.get("ok", False):
                    log.warn(
                        "[scan_yaml_targets] Target marker publication failed; continuing. "
                        f"error='{marker_res.get('error', 'unknown')}'"
                    )

            plan_ok = 0
            exec_ok = 0
            skipped = 0
            failed_targets: list[dict[str, Any]] = []
            stopped_by_user = False

            for seq_idx, target in enumerate(targets):
                if not rclpy.ok():
                    break

                latest_cfg = self._reload_config_or_keep(config_path, latest_cfg)
                timeout_s = self._cfg_optional_timeout(latest_cfg, "timeout_s")
                self._warn_immutable_changes(
                    latest_cfg,
                    yaml_path=yaml_path,
                    frame_id=frame_id,
                    capture_folder=capture_folder,
                    scan_folder=configured_scan_folder,
                    session_path=session_path,
                    eef_link=eef_link,
                )

                target_label = f"target {seq_idx + 1}/{len(targets)}"
                if self._cfg_bool(latest_cfg, "prompt_before_next_plane"):
                    prompt_res = self.scan_session.run_sync(
                        self.scan_session.util.input(
                            prompt=(
                                f"[scan_yaml_targets] {target_label}. Press ENTER to plan "
                                "(type 'q' + ENTER to finish with captures collected so far)."
                            ),
                            enqueue=False,
                        ),
                        timeout_s=None,
                    )
                    value = str(prompt_res.get("metrics", {}).get("value", "")).strip().lower()
                    if value == "q":
                        stopped_by_user = True
                        log.warn("[scan_yaml_targets] Operator requested an early finish.")
                        break

                vel_scale = self._cfg_float(latest_cfg, "vel_scale")
                accel_scale = self._cfg_float(latest_cfg, "accel_scale")

                context_error = self._prepare_linear_context(
                    eef_link=eef_link,
                    vel_scale=vel_scale,
                    accel_scale=accel_scale,
                    timeout_s=timeout_s,
                )
                if context_error:
                    skipped += 1
                    failed_targets.append(
                        {"index": seq_idx, "stage": "motion_context", "error": context_error}
                    )
                    log.warn(
                        f"[scan_yaml_targets] Skipping {target_label}: "
                        f"motion context failed. error='{context_error}'"
                    )
                    continue

                log.info(
                    f"[scan_yaml_targets] Planning LIN {target_label}: "
                    f"xyz=({target.pose.position.x:.4f}, {target.pose.position.y:.4f}, "
                    f"{target.pose.position.z:.4f}) vel={vel_scale:.3f} accel={accel_scale:.3f}"
                )
                try:
                    plan_res = self.scan_session.run_sync(
                        self.scan_session.motion.plan(
                            pose=target,
                            eef=eef_link,
                            motion="LIN",
                            vel_scale=vel_scale,
                            accel_scale=accel_scale,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                except TimeoutError:
                    plan_res = {"ok": False, "error": "timeout"}

                if not plan_res.get("ok", False):
                    skipped += 1
                    error = str(plan_res.get("error", "unknown"))
                    failed_targets.append({"index": seq_idx, "stage": "plan", "error": error})
                    log.warn(
                        f"[scan_yaml_targets] LIN plan failed for {target_label}; skipping it. "
                        f"error='{error}'"
                    )
                    continue
                plan_ok += 1

                try:
                    exec_res = self.scan_session.run_sync(
                        self.scan_session.motion.exec(enqueue=False),
                        timeout_s=timeout_s,
                    )
                except TimeoutError:
                    exec_res = {"ok": False, "error": "timeout"}

                if not exec_res.get("ok", False):
                    skipped += 1
                    error = str(exec_res.get("error", "unknown"))
                    failed_targets.append({"index": seq_idx, "stage": "exec", "error": error})
                    log.warn(
                        f"[scan_yaml_targets] Execution failed for {target_label}; "
                        f"skipping dwell/capture. error='{error}'"
                    )
                    continue
                exec_ok += 1

                dwell_s = self._cfg_float(latest_cfg, "dwell_s")
                if dwell_s > 0.0:
                    try:
                        dwell_res = self.scan_session.run_sync(
                            self.scan_session.util.wait(dwell_s, enqueue=False),
                            timeout_s=timeout_s,
                        )
                        if not dwell_res.get("ok", False):
                            log.warn(
                                f"[scan_yaml_targets] Dwell failed for {target_label}; "
                                "continuing to capture."
                            )
                    except TimeoutError:
                        log.warn(
                            f"[scan_yaml_targets] Dwell timed out for {target_label}; "
                            "continuing to capture."
                        )

                try:
                    capture_res = self.scan_session.run_sync(
                        self.scan_session.camera.capture(
                            rgb=self._cfg_bool(latest_cfg, "rgb"),
                            depth=self._cfg_bool(latest_cfg, "depth"),
                            ir=self._cfg_bool(latest_cfg, "ir"),
                            pose=self._cfg_bool(latest_cfg, "pose"),
                            folder=capture_folder,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                except TimeoutError:
                    capture_res = {"ok": False, "error": "timeout"}

                if not capture_res.get("ok", False):
                    error = str(capture_res.get("error", "unknown"))
                    failed_targets.append({"index": seq_idx, "stage": "capture", "error": error})
                    if error.startswith(STALE_FRAME_ERROR_PREFIX):
                        raise RuntimeError(
                            "Frozen camera frame detected; stopping scan immediately. " + error
                        )
                    log.warn(f"[scan_yaml_targets] Capture failed for {target_label}: {error}")
                    continue

                captures_ok += 1
                log.info(f"[scan_yaml_targets] Capture complete for {target_label}.")

            log.info(
                "[scan_yaml_targets] Motion/capture pass complete: "
                f"targets={len(targets)}, planned={plan_ok}, executed={exec_ok}, "
                f"captures={captures_ok}, skipped={skipped}, "
                f"stopped_by_user={stopped_by_user}"
            )
            if failed_targets:
                log.warn(f"[scan_yaml_targets] Failed target details: {failed_targets}")

            latest_cfg = self._reload_config_or_keep(config_path, latest_cfg)
            timeout_s = self._cfg_optional_timeout(latest_cfg, "timeout_s")
            if self._cfg_bool(latest_cfg, "run_reconstruction"):
                if captures_ok <= 0:
                    log.error(
                        "[scan_yaml_targets] Reconstruction not started because no capture completed."
                    )
                else:
                    reconstruction = self.scan_session.run_reconstruction_for_scan(
                        scan_folder=scan_folder,
                        session_path=session_path,
                        reconstruct_device=self._cfg_str(latest_cfg, "reconstruct_device"),
                        reconstruct_request_timeout_s=self._cfg_float(
                            latest_cfg, "reconstruct_request_timeout_s"
                        ),
                        wait_reconstruction_outputs=self._cfg_bool(
                            latest_cfg, "wait_reconstruction_outputs"
                        ),
                        color_to_depth_wait_timeout_s=self._cfg_float(
                            latest_cfg, "color_to_depth_wait_timeout_s"
                        ),
                        tsdf_wait_timeout_s=self._cfg_float(latest_cfg, "tsdf_wait_timeout_s"),
                        mesh_prefer=self._cfg_str(latest_cfg, "mesh_prefer"),
                        mesh_update_wait_timeout_s=self._cfg_float(
                            latest_cfg, "mesh_update_wait_timeout_s"
                        ),
                        mesh_update_request_timeout_s=self._cfg_float(
                            latest_cfg, "mesh_update_request_timeout_s"
                        ),
                        update_world_mesh=self._cfg_bool(latest_cfg, "update_world_mesh"),
                        tsdf_center_crop_enable=self._cfg_bool(
                            latest_cfg, "tsdf_center_crop_enable"
                        ),
                        tsdf_center_crop_width=self._cfg_int(
                            latest_cfg, "tsdf_center_crop_width"
                        ),
                        tsdf_center_crop_height=self._cfg_int(
                            latest_cfg, "tsdf_center_crop_height"
                        ),
                        tsdf_center_crop_apply_to_depth=self._cfg_bool(
                            latest_cfg, "tsdf_center_crop_apply_to_depth"
                        ),
                        tsdf_aabb_crop_enable=self._cfg_bool(
                            latest_cfg, "tsdf_aabb_crop_enable"
                        ),
                        tsdf_aabb_crop_min=self._cfg_float_list(
                            latest_cfg, "tsdf_aabb_crop_min", expected_length=3
                        ),
                        tsdf_aabb_crop_max=self._cfg_float_list(
                            latest_cfg, "tsdf_aabb_crop_max", expected_length=3
                        ),
                        tsdf_auto_object_crop_enable=self._cfg_bool(
                            latest_cfg, "tsdf_auto_object_crop_enable"
                        ),
                        tsdf_auto_object_min_height_m=self._cfg_float(
                            latest_cfg, "tsdf_auto_object_min_height_m"
                        ),
                        tsdf_auto_object_cluster_eps_m=self._cfg_float(
                            latest_cfg, "tsdf_auto_object_cluster_eps_m"
                        ),
                        tsdf_auto_object_cluster_min_points=self._cfg_int(
                            latest_cfg, "tsdf_auto_object_cluster_min_points"
                        ),
                        tsdf_auto_object_neighbor_max_gap_m=self._cfg_float(
                            latest_cfg, "tsdf_auto_object_neighbor_max_gap_m"
                        ),
                        tsdf_auto_object_xy_margin_m=self._cfg_float(
                            latest_cfg, "tsdf_auto_object_xy_margin_m"
                        ),
                        tsdf_auto_object_top_margin_m=self._cfg_float(
                            latest_cfg, "tsdf_auto_object_top_margin_m"
                        ),
                        tsdf_auto_object_table_below_margin_m=self._cfg_float(
                            latest_cfg, "tsdf_auto_object_table_below_margin_m"
                        ),
                        tsdf_param_update_timeout_s=self._cfg_float(
                            latest_cfg, "tsdf_param_update_timeout_s"
                        ),
                    )
                    if not reconstruction.get("ok", False):
                        log.error(
                            "[scan_yaml_targets] Reconstruction failed: "
                            f"stage={reconstruction.get('stage')} "
                            f"error='{reconstruction.get('error', 'unknown')}'"
                        )
                    else:
                        log.info(
                            "[scan_yaml_targets] Reconstruction complete: "
                            f"mesh='{reconstruction.get('mesh_path', '')}', "
                            f"rgb_ply='{reconstruction.get('rgb_ply_path', '')}'"
                        )

            if self._cfg_bool(latest_cfg, "home_after"):
                home_res = self.scan_session.run_sync(
                    self.scan_session.motion.home(enqueue=False),
                    timeout_s=timeout_s,
                )
                if not home_res.get("ok", False):
                    log.warn(
                        "[scan_yaml_targets] home_after failed: "
                        f"{home_res.get('error', 'unknown')}"
                    )

        except Exception as exc:
            log.error(f"[scan_yaml_targets] Sequence failed: {exc}")
        finally:
            if publish_markers and latest_cfg and self._cfg_bool(latest_cfg, "clear_markers_after"):
                try:
                    self.scan_session.run_sync(
                        self.scan_session.util.delete_markers(enqueue=False),
                        timeout_s=timeout_s,
                    )
                except Exception as exc:
                    log.warn(f"[scan_yaml_targets] Failed to clear target markers: {exc}")
            if rclpy.ok():
                rclpy.shutdown()

    def _prepare_linear_context(
        self,
        *,
        eef_link: str,
        vel_scale: float,
        accel_scale: float,
        timeout_s: Optional[float],
    ) -> str:
        commands = (
            self.scan_session.motion.setLIN(enqueue=False),
            self.scan_session.motion.setEef(eef_link, enqueue=False),
            self.scan_session.motion.setSpd(vel_scale, enqueue=False),
            self.scan_session.motion.setAcc(accel_scale, enqueue=False),
        )
        for item in commands:
            try:
                result = self.scan_session.run_sync(item, timeout_s=timeout_s)
            except TimeoutError:
                return f"{item.kind} timeout"
            if not result.get("ok", False):
                return f"{item.kind}: {result.get('error', 'unknown')}"
        return ""

    def _current_eef_pose(
        self,
        *,
        eef_link: str,
        frame_id: str,
        timeout_s: Optional[float],
    ) -> PoseStamped:
        result = self.scan_session.run_sync(
            self.scan_session.camera.get_pose(
                eef=eef_link,
                base_frame=frame_id,
                use_tf=True,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        pose = result.get("pose")
        if not result.get("ok", False) or not isinstance(pose, PoseStamped):
            raise RuntimeError(
                "Current camera orientation is required to resolve target roll: "
                f"eef='{eef_link}', frame='{frame_id}', "
                f"error='{result.get('error', 'pose unavailable')}'"
            )
        return pose

    @staticmethod
    def _match_target_z_to_eef_orientation(
        targets: Sequence[PoseStamped],
        *,
        reference_pose: PoseStamped,
    ) -> list[PoseStamped]:
        """Align each target Z while preserving the EEF's current roll reference."""
        aligned_targets: list[PoseStamped] = []
        for target in targets:
            desired_z = tb.z_axis(target)
            aligned = tb.adjust_target(reference_pose, desired_z)
            aligned.header.frame_id = target.header.frame_id
            aligned.header.stamp = target.header.stamp
            aligned.pose.position.x = float(target.pose.position.x)
            aligned.pose.position.y = float(target.pose.position.y)
            aligned.pose.position.z = float(target.pose.position.z)
            aligned_targets.append(aligned)
        return aligned_targets

    def _reload_config_or_keep(
        self,
        config_path: str,
        previous: Dict[str, Any],
    ) -> Dict[str, Any]:
        try:
            current = self._load_config(config_path)
            self._validate_config(current)
            return current
        except Exception as exc:
            self.get_logger().warn(
                "[scan_yaml_targets] Runtime config reload failed; keeping previous values. "
                f"path='{config_path}' error='{exc}'"
            )
            return previous

    def _warn_immutable_changes(
        self,
        cfg: Dict[str, Any],
        *,
        yaml_path: str,
        frame_id: str,
        capture_folder: str,
        scan_folder: str,
        session_path: str,
        eef_link: str,
    ) -> None:
        loaded = {
            "yaml_path": yaml_path,
            "frame_id": frame_id,
            "capture_folder": capture_folder,
            "scan_folder": scan_folder,
            "session_path": session_path,
            "eef_link": eef_link,
        }
        changed = []
        for key, original in loaded.items():
            current = self._cfg_str_allow_empty(cfg, key)
            if current != original:
                changed.append(f"{key}='{current}'")
        if changed:
            self.get_logger().warn(
                "[scan_yaml_targets] Path/frame changes are ignored during an active run; "
                "restart to apply them: " + ", ".join(changed)
            )

    def _validate_config(self, cfg: Dict[str, Any]) -> None:
        vel = self._cfg_float(cfg, "vel_scale")
        accel = self._cfg_float(cfg, "accel_scale")
        dwell = self._cfg_float(cfg, "dwell_s")
        if not 0.0 < vel <= 1.0:
            raise ValueError(f"vel_scale must be in (0, 1], got {vel}")
        if not 0.0 < accel <= 1.0:
            raise ValueError(f"accel_scale must be in (0, 1], got {accel}")
        if dwell < 0.0:
            raise ValueError(f"dwell_s must be >= 0, got {dwell}")

        # Validate the complete file so a partial save during live editing
        # falls back to the previous known-good configuration.
        self._cfg_optional_timeout(cfg, "timeout_s")
        for key in ("yaml_path", "frame_id", "session_path", "capture_folder", "eef_link"):
            self._cfg_str(cfg, key)
        self._cfg_str_allow_empty(cfg, "scan_folder")

        for key in (
            "home_before",
            "home_after",
            "prompt_before_next_plane",
            "rgb",
            "depth",
            "ir",
            "pose",
            "publish_markers",
            "clear_markers_before",
            "clear_markers_after",
            "run_reconstruction",
            "wait_reconstruction_outputs",
            "update_world_mesh",
            "tsdf_center_crop_enable",
            "tsdf_center_crop_apply_to_depth",
            "tsdf_aabb_crop_enable",
            "tsdf_auto_object_crop_enable",
        ):
            self._cfg_bool(cfg, key)

        axis_length = self._cfg_float(cfg, "axis_length")
        axis_radius = self._cfg_float(cfg, "axis_radius")
        if axis_length <= 0.0 or axis_radius <= 0.0:
            raise ValueError("axis_length and axis_radius must be > 0")

        self._cfg_str(cfg, "reconstruct_device")
        mesh_prefer = self._cfg_str(cfg, "mesh_prefer").lower()
        if mesh_prefer not in ("mesh", "ply"):
            raise ValueError(f"mesh_prefer must be 'mesh' or 'ply', got '{mesh_prefer}'")
        for key in (
            "reconstruct_request_timeout_s",
            "color_to_depth_wait_timeout_s",
            "tsdf_wait_timeout_s",
            "mesh_update_wait_timeout_s",
            "mesh_update_request_timeout_s",
            "tsdf_param_update_timeout_s",
        ):
            if self._cfg_float(cfg, key) <= 0.0:
                raise ValueError(f"{key} must be > 0")

        if self._cfg_int(cfg, "tsdf_center_crop_width") <= 0:
            raise ValueError("tsdf_center_crop_width must be > 0")
        if self._cfg_int(cfg, "tsdf_center_crop_height") <= 0:
            raise ValueError("tsdf_center_crop_height must be > 0")
        crop_min = self._cfg_float_list(cfg, "tsdf_aabb_crop_min", expected_length=3)
        crop_max = self._cfg_float_list(cfg, "tsdf_aabb_crop_max", expected_length=3)
        if any(low >= high for low, high in zip(crop_min, crop_max)):
            raise ValueError("each tsdf_aabb_crop_min component must be below crop_max")
        if self._cfg_float(cfg, "tsdf_auto_object_min_height_m") < 0.0:
            raise ValueError("tsdf_auto_object_min_height_m must be >= 0")
        if self._cfg_float(cfg, "tsdf_auto_object_cluster_eps_m") <= 0.0:
            raise ValueError("tsdf_auto_object_cluster_eps_m must be > 0")
        if self._cfg_int(cfg, "tsdf_auto_object_cluster_min_points") <= 0:
            raise ValueError("tsdf_auto_object_cluster_min_points must be > 0")
        if self._cfg_float(cfg, "tsdf_auto_object_neighbor_max_gap_m") < 0.0:
            raise ValueError("tsdf_auto_object_neighbor_max_gap_m must be >= 0")
        for key in (
            "tsdf_auto_object_xy_margin_m",
            "tsdf_auto_object_top_margin_m",
            "tsdf_auto_object_table_below_margin_m",
        ):
            if self._cfg_float(cfg, key) < 0.0:
                raise ValueError(f"{key} must be >= 0")

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
                "'scan_yaml_targets_sequence: ros__parameters: ...'."
            )
        cfg = dict(params)
        self._validate_config(cfg)
        return cfg

    def _extract_config_params(self, raw: Any) -> Dict[str, Any]:
        if not isinstance(raw, dict):
            return {}
        for key in (self.get_name(), f"/{self.get_name()}"):
            block = raw.get(key)
            if isinstance(block, dict) and isinstance(block.get("ros__parameters"), dict):
                return dict(block["ros__parameters"])
        if isinstance(raw.get("ros__parameters"), dict):
            return dict(raw["ros__parameters"])
        return dict(raw)

    @staticmethod
    def _default_config_path() -> str:
        # Prefer the editable workspace copy. With a regular colcon build this
        # module runs from install/, so __file__ alone cannot locate src/.
        source_candidates = [
            Path(__file__).resolve().parents[1] / "config" / CONFIG_FILENAME,
            Path.cwd() / "src" / "behav3d_orchestrator" / "config" / CONFIG_FILENAME,
        ]
        for source_path in source_candidates:
            if source_path.is_file():
                return str(source_path)

        try:
            from ament_index_python.packages import get_package_share_directory

            share_path = Path(get_package_share_directory("behav3d_orchestrator")).resolve()
            for parent in share_path.parents:
                if parent.name != "install":
                    continue
                source_path = (
                    parent.parent
                    / "src"
                    / "behav3d_orchestrator"
                    / "config"
                    / CONFIG_FILENAME
                )
                if source_path.is_file():
                    return str(source_path)
            return str(share_path / "config" / CONFIG_FILENAME)
        except Exception:
            return str(source_candidates[0])

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
        value = cls._require(cfg, key)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text in ("1", "true", "yes", "y", "on"):
            return True
        if text in ("0", "false", "no", "n", "off"):
            return False
        raise ValueError(f"expected bool for '{key}', got '{value}'")

    @classmethod
    def _cfg_int(cls, cfg: Dict[str, Any], key: str) -> int:
        value = cls._require(cfg, key)
        try:
            return int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"expected int for '{key}', got '{value}'") from exc

    @classmethod
    def _cfg_float(cls, cfg: Dict[str, Any], key: str) -> float:
        value = cls._require(cfg, key)
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"expected float for '{key}', got '{value}'") from exc
        if not math.isfinite(result):
            raise ValueError(f"expected finite float for '{key}', got '{value}'")
        return result

    @classmethod
    def _cfg_str(cls, cfg: Dict[str, Any], key: str) -> str:
        result = cls._cfg_str_allow_empty(cfg, key)
        if not result:
            raise ValueError(f"expected non-empty string for '{key}'")
        return result

    @classmethod
    def _cfg_str_allow_empty(cls, cfg: Dict[str, Any], key: str) -> str:
        value = cls._require(cfg, key)
        if value is None:
            raise ValueError(f"expected string for '{key}', got null")
        return str(value).strip()

    @classmethod
    def _cfg_float_list(
        cls,
        cfg: Dict[str, Any],
        key: str,
        *,
        expected_length: Optional[int] = None,
    ) -> list[float]:
        value = cls._require(cfg, key)
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"expected float list for '{key}', got '{value}'")
        result = [float(item) for item in value]
        if expected_length is not None and len(result) != expected_length:
            raise ValueError(
                f"expected {expected_length} values for '{key}', got {len(result)}"
            )
        if not all(math.isfinite(item) for item in result):
            raise ValueError(f"expected finite values for '{key}', got '{value}'")
        return result

    @classmethod
    def _cfg_optional_timeout(cls, cfg: Dict[str, Any], key: str) -> Optional[float]:
        value = cls._cfg_float(cfg, key)
        return None if value <= 0.0 else value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanYamlTargetsSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
