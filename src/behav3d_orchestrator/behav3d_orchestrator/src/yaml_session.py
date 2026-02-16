#!/usr/bin/env python3
from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

import behav3d_commands
from geometry_msgs.msg import PoseStamped

try:
    import yaml
except Exception:  # pragma: no cover - handled explicitly at runtime
    yaml = None


class YamlSession(behav3d_commands.Session):
    """
    Session helpers for target sequences loaded from YAML.
    """

    def run_yaml_target_sequence(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
        debug: bool = False,
        vel_scale: float = 0.05,
        accel_scale: float = 0.05,
        timeout_s: Optional[float] = None,
        publish_markers: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> List[PoseStamped]:
        """
        Sequence:
        1) home
        2) parse targets from YAML
        3) visualize targets
        4) plan+exec for each target (optional debug ENTER gate)
        """
        log = self.node.get_logger()

        self.run_sync(self.motion.home(enqueue=False), timeout_s=timeout_s)
        # Give mock/planning state a brief moment to settle after home.
        self.run_sync(self.util.wait(0.5, enqueue=False), timeout_s=timeout_s)

        targets = self.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
        if not targets:
            raise ValueError(f"No valid targets found in YAML: {yaml_path}")

        self.run_sync(self.motion.setSpd(float(vel_scale), enqueue=False), timeout_s=timeout_s)
        self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
        self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)

        if publish_markers:
            self.run_sync(
                self.util.publish_targets(
                    targets,
                    axis_length=float(axis_length),
                    axis_radius=float(axis_radius),
                    clear_before=bool(clear_markers_before),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )

        for seq_idx, ps in enumerate(targets):
            if debug:
                res = self.run_sync(
                    self.util.input(
                        prompt=(
                            f"[yaml_sequence] Target {seq_idx + 1}/{len(targets)}. "
                            "Press ENTER to move (or type 'q' + ENTER to stop)."
                        ),
                        enqueue=False,
                    ),
                    timeout_s=None,
                )
                value = str(res.get("metrics", {}).get("value", "")).strip().lower()
                if value == "q":
                    log.warn("[yaml_sequence] User requested stop. Ending sequence early.")
                    break

            # Safety approach only for the first target: same XY, 40 cm above target Z.
            if seq_idx == 0:
                approach_ps = PoseStamped()
                approach_ps.header.frame_id = ps.header.frame_id
                approach_ps.header.stamp = ps.header.stamp
                approach_ps.pose.position.x = float(ps.pose.position.x)
                approach_ps.pose.position.y = float(ps.pose.position.y)
                approach_ps.pose.position.z = float(ps.pose.position.z)
                approach_ps.pose.orientation.x = float(ps.pose.orientation.x)
                approach_ps.pose.orientation.y = float(ps.pose.orientation.y)
                approach_ps.pose.orientation.z = float(ps.pose.orientation.z)
                approach_ps.pose.orientation.w = float(ps.pose.orientation.w)
                approach_ps.pose.position.z = float(ps.pose.position.z) + 0.40

                approach_plan_res = self.run_sync(
                    self.motion.plan(
                        pose=approach_ps,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not approach_plan_res.get("ok", False):
                    err = str(approach_plan_res.get("error", "unknown")).strip()
                    log.warn(
                        f"[yaml_sequence] Approach plan failed at target {seq_idx} "
                        f"({approach_ps.pose.position.x:.3f}, {approach_ps.pose.position.y:.3f}, {approach_ps.pose.position.z:.3f}). "
                        f"Error='{err}'. Skipping target."
                    )
                    continue

                approach_exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
                if not approach_exec_res.get("ok", False):
                    err = str(approach_exec_res.get("error", "unknown")).strip()
                    log.warn(
                        f"[yaml_sequence] Approach exec failed at target {seq_idx}. "
                        f"Error='{err}'. Skipping target."
                    )
                    continue

            plan_res = self.run_sync(
                self.motion.plan(
                    pose=ps,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )

            if not plan_res.get("ok", False):
                err = str(plan_res.get("error", "unknown")).strip()
                log.warn(
                    f"[yaml_sequence] Plan failed at target {seq_idx} "
                    f"({ps.pose.position.x:.3f}, {ps.pose.position.y:.3f}, {ps.pose.position.z:.3f}). "
                    f"Error='{err}'. Skipping exec."
                )
                continue

            exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
            if not exec_res.get("ok", False):
                log.warn(f"[yaml_sequence] Exec failed at target {seq_idx}. Continuing.")

        return targets

    def run_yaml_print_path(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
        print_speed: int = 800,
        approach_z_offset_m: float = 0.40,
        approach_vel_scale: float = 0.10,
        print_vel_scale: float = 0.01,
        timeout_s: Optional[float] = None,
        publish_markers: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> dict:
        """
        Print path behavior:
        1) Parse ordered poses from YAML (supports xyz and plane strings).
        2) Move to approach pose above first point.
        3) Move to first point.
        4) Turn extruder on and execute path segments to the last point.
        5) Turn extruder off at the end.
        """
        log = self.node.get_logger()
        targets = self.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
        if len(targets) < 2:
            raise ValueError(f"Need at least 2 targets to print a path. Got {len(targets)}")

        first = targets[0]
        approach = PoseStamped()
        approach.header.frame_id = first.header.frame_id
        approach.pose.position.x = float(first.pose.position.x)
        approach.pose.position.y = float(first.pose.position.y)
        approach.pose.position.z = float(first.pose.position.z) + float(approach_z_offset_m)
        approach.pose.orientation.x = float(first.pose.orientation.x)
        approach.pose.orientation.y = float(first.pose.orientation.y)
        approach.pose.orientation.z = float(first.pose.orientation.z)
        approach.pose.orientation.w = float(first.pose.orientation.w)

        marker_targets = [approach] + targets

        extruder_on = False
        try:
            if publish_markers:
                self.run_sync(
                    self.util.publish_targets(
                        marker_targets,
                        axis_length=float(axis_length),
                        axis_radius=float(axis_radius),
                        clear_before=bool(clear_markers_before),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )

            log.info(
                "[yaml_print_path] Approach first target: "
                f"({approach.pose.position.x:.3f}, {approach.pose.position.y:.3f}, {approach.pose.position.z:.3f})"
            )
            res = self.run_sync(
                self.motion.goto(
                    pose=approach,
                    motion="LIN",
                    vel_scale=float(approach_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_first", "error": res.get("error", "unknown")}

            log.info(
                "[yaml_print_path] Move to first print target: "
                f"({first.pose.position.x:.3f}, {first.pose.position.y:.3f}, {first.pose.position.z:.3f})"
            )
            res = self.run_sync(
                self.motion.goto(
                    pose=first,
                    motion="LIN",
                    vel_scale=float(print_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "goto_first", "error": res.get("error", "unknown")}

            on_res = self.run_sync(
                self.extruder.setExtruder(True, speed=int(print_speed), enqueue=False),
                timeout_s=timeout_s,
            )
            if not on_res.get("ok", False):
                return {"ok": False, "stage": "extruder_on", "error": on_res.get("error", "unknown")}
            extruder_on = True

            for i in range(1, len(targets)):
                ps = targets[i]
                plan_res = self.run_sync(
                    self.motion.plan(
                        pose=ps,
                        motion="LIN",
                        vel_scale=float(print_vel_scale),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not plan_res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"plan_segment_{i}",
                        "error": plan_res.get("error", "unknown"),
                    }

                exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
                if not exec_res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"exec_segment_{i}",
                        "error": exec_res.get("error", "unknown"),
                    }

            off_res = self.run_sync(
                self.extruder.setExtruder(False, enqueue=False),
                timeout_s=timeout_s,
            )
            extruder_on = False
            if not off_res.get("ok", False):
                return {"ok": False, "stage": "extruder_off", "error": off_res.get("error", "unknown")}

            return {"ok": True, "stage": "done", "targets": len(targets)}
        finally:
            if extruder_on:
                try:
                    self.run_sync(self.extruder.setExtruder(False, enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[yaml_print_path] Failed to force extruder OFF after error/timeout.")
            if publish_markers:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[yaml_print_path] delete_markers timed out.")

    def parse_yaml_targets(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
    ) -> List[PoseStamped]:
        """
        Parse YAML into PoseStamped targets sorted by `index` when available.
        Supported target item shapes:
        - {index: 0, xyz: [x, y, z]}
        - {index: 0, x: ..., y: ..., z: ...}
        - {index: 0, plane: "O(x,y,z) Z(i,j,k)"}  # O in mm, Z as orientation normal
        - [x, y, z]
        """
        if yaml is None:
            raise RuntimeError("PyYAML is not available. Install python3-yaml.")

        path = self._resolve_yaml_path(yaml_path)
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        items = self._extract_target_items(data)

        ordered: List[Tuple[int, int, PoseStamped]] = []
        for pos, item in enumerate(items):
            idx, ps = self._parse_target_item(
                item=item,
                fallback_index=pos,
                frame_id=str(frame_id or "world"),
            )
            order_idx = int(idx) if idx is not None else 1_000_000 + pos
            ordered.append((order_idx, pos, ps))

        ordered.sort(key=lambda row: (row[0], row[1]))
        return [ps for _, _, ps in ordered]

    @staticmethod
    def _resolve_yaml_path(yaml_path: str) -> Path:
        if not str(yaml_path).strip():
            raise ValueError("yaml_path is empty")

        candidate = Path(yaml_path).expanduser()
        if candidate.is_absolute() and candidate.is_file():
            return candidate.resolve()

        cwd_candidate = (Path.cwd() / candidate).resolve()
        if cwd_candidate.is_file():
            return cwd_candidate

        ws_candidate = (Path(__file__).resolve().parents[4] / candidate).resolve()
        if ws_candidate.is_file():
            return ws_candidate

        raise FileNotFoundError(f"YAML target file not found: {yaml_path}")

    @staticmethod
    def _extract_target_items(data: Any) -> List[Any]:
        if isinstance(data, dict):
            if isinstance(data.get("targets"), list):
                return list(data["targets"])

            # Allow direct map with index-like keys.
            if all(YamlSession._is_int_like(k) for k in data.keys()):
                out: List[Any] = []
                for k, v in data.items():
                    idx = int(k)
                    if isinstance(v, dict):
                        row = dict(v)
                        row.setdefault("index", idx)
                        out.append(row)
                    else:
                        out.append({"index": idx, "xyz": v})
                return out

        if isinstance(data, list):
            return list(data)

        raise ValueError("YAML format unsupported. Use 'targets: [...]' or a list of targets.")

    @staticmethod
    def _parse_target_item(
        *,
        item: Any,
        fallback_index: int,
        frame_id: str,
    ) -> Tuple[Optional[int], PoseStamped]:
        if isinstance(item, dict):
            idx_raw = item.get("index", fallback_index)
            idx = int(idx_raw) if idx_raw is not None else None

            if "plane" in item:
                xyz_m, z_axis = YamlSession._parse_plane_entry(str(item["plane"]))
                ps = YamlSession._pose_from_xyz_and_z_axis(xyz_m=xyz_m, z_axis=z_axis, frame_id=frame_id)
            elif "xyz" in item:
                xyz = YamlSession._coerce_xyz(item["xyz"])
                ps = YamlSession._pose_from_xyz_and_z_axis(
                    xyz_m=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                    z_axis=None,
                    frame_id=frame_id,
                )
            elif all(k in item for k in ("x", "y", "z")):
                ps = YamlSession._pose_from_xyz_and_z_axis(
                    xyz_m=(float(item["x"]), float(item["y"]), float(item["z"])),
                    z_axis=None,
                    frame_id=frame_id,
                )
            else:
                raise ValueError(f"Target entry missing supported fields (xyz/x,y,z/plane): {item}")
            return idx, ps

        if isinstance(item, (list, tuple)):
            xyz = YamlSession._coerce_xyz(item)
            ps = YamlSession._pose_from_xyz_and_z_axis(
                xyz_m=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                z_axis=None,
                frame_id=frame_id,
            )
            return fallback_index, ps

        raise ValueError(f"Unsupported target entry type: {type(item).__name__}")

    @staticmethod
    def _coerce_xyz(value: Any) -> Tuple[float, float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"xyz must be a 3-element list/tuple. Got: {value}")
        return (float(value[0]), float(value[1]), float(value[2]))

    @staticmethod
    def _parse_plane_entry(plane: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        pattern = (
            r"O\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\)\s*"
            r"Z\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\)"
        )
        match = re.search(pattern, str(plane))
        if match is None:
            raise ValueError(f"Invalid plane format. Expected 'O(x,y,z) Z(i,j,k)'. Got: {plane}")

        ox_mm, oy_mm, oz_mm, zx, zy, zz = [float(match.group(i)) for i in range(1, 7)]
        xyz_m = (ox_mm / 1000.0, oy_mm / 1000.0, oz_mm / 1000.0)
        z_axis = (float(zx), float(zy), float(zz))
        return xyz_m, z_axis

    @staticmethod
    def _pose_from_xyz_and_z_axis(
        *,
        xyz_m: Tuple[float, float, float],
        z_axis: Optional[Tuple[float, float, float]],
        frame_id: str,
    ) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = str(frame_id or "world")
        ps.pose.position.x = float(xyz_m[0])
        ps.pose.position.y = float(xyz_m[1])
        ps.pose.position.z = float(xyz_m[2])

        if z_axis is None:
            ps.pose.orientation.w = 1.0
            return ps

        qx, qy, qz, qw = YamlSession._quat_from_z_axis(z_axis)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        return ps

    @staticmethod
    def _quat_from_z_axis(z_axis: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
        zx, zy, zz = float(z_axis[0]), float(z_axis[1]), float(z_axis[2])
        n = math.sqrt(zx * zx + zy * zy + zz * zz)
        if n < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)

        zx /= n
        zy /= n
        zz /= n

        # Pick a stable reference to define X around the provided Z normal.
        rx, ry, rz = (1.0, 0.0, 0.0)
        if abs(zx) > 0.95:
            rx, ry, rz = (0.0, 1.0, 0.0)

        dot = rx * zx + ry * zy + rz * zz
        xx = rx - dot * zx
        xy = ry - dot * zy
        xz = rz - dot * zz
        nx = math.sqrt(xx * xx + xy * xy + xz * xz)
        if nx < 1e-9:
            # Fallback orthonormal axis if projection degenerates.
            xx, xy, xz = (0.0, -zz, zy)
            nx = math.sqrt(xx * xx + xy * xy + xz * xz)
        xx /= nx
        xy /= nx
        xz /= nx

        yx = zy * xz - zz * xy
        yy = zz * xx - zx * xz
        yz = zx * xy - zy * xx

        return YamlSession._quat_from_rotmat(
            r00=xx, r01=yx, r02=zx,
            r10=xy, r11=yy, r12=zy,
            r20=xz, r21=yz, r22=zz,
        )

    @staticmethod
    def _quat_from_rotmat(
        *,
        r00: float,
        r01: float,
        r02: float,
        r10: float,
        r11: float,
        r12: float,
        r20: float,
        r21: float,
        r22: float,
    ) -> Tuple[float, float, float, float]:
        tr = r00 + r11 + r22
        if tr > 0.0:
            s = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r21 - r12) / s
            qy = (r02 - r20) / s
            qz = (r10 - r01) / s
        elif r00 > r11 and r00 > r22:
            s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
            qw = (r21 - r12) / s
            qx = 0.25 * s
            qy = (r01 + r10) / s
            qz = (r02 + r20) / s
        elif r11 > r22:
            s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
            qw = (r02 - r20) / s
            qx = (r01 + r10) / s
            qy = 0.25 * s
            qz = (r12 + r21) / s
        else:
            s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
            qw = (r10 - r01) / s
            qx = (r02 + r20) / s
            qy = (r12 + r21) / s
            qz = 0.25 * s

        nq = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if nq < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (qx / nq, qy / nq, qz / nq, qw / nq)

    @staticmethod
    def _is_int_like(value: Any) -> bool:
        try:
            int(value)
            return True
        except Exception:
            return False
