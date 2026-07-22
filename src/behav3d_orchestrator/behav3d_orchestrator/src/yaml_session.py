#!/usr/bin/env python3
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

from behav3d_utils import pose_from_xyz_and_z_axis
from geometry_msgs.msg import PoseStamped

from .control_session import ControlAwareSession

try:
    import yaml
except Exception:  # pragma: no cover - handled explicitly at runtime
    yaml = None


@dataclass(frozen=True)
class TargetSegment:
    index: int
    start: PoseStamped
    end: PoseStamped


@dataclass(frozen=True)
class TargetPolyline:
    index: int
    poses: List[PoseStamped]


class YamlSession(ControlAwareSession):
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
        data = self._load_yaml_data(path)

        if isinstance(data, dict) and isinstance(data.get("polylines"), list):
            return self._flatten_polylines(
                self._parse_polylines_data(data, frame_id=str(frame_id or "world"))
            )

        if isinstance(data, dict) and isinstance(data.get("path"), list):
            return self._parse_path_data(data, frame_id=str(frame_id or "world"))

        if isinstance(data, dict) and isinstance(data.get("segments"), list):
            segments = self._parse_segments_data(data, frame_id=str(frame_id or "world"))
            out: List[PoseStamped] = []
            for seg in segments:
                out.append(seg.start)
                out.append(seg.end)
            return out

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

    def parse_yaml_polylines(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
    ) -> List[TargetPolyline]:
        """
        Parse indexed YAML polylines into ordered pose lists.

        Canonical shape:
        polylines:
          - index: 0
            planes:
              - "O(x,y,z) Z(i,j,k)"
              - "O(x,y,z) Z(i,j,k)"

        Each indexed item is one polyline. A `path: [...]` list is also
        accepted inside each item when per-point maps are preferred.
        """
        if yaml is None:
            raise RuntimeError("PyYAML is not available. Install python3-yaml.")

        path = self._resolve_yaml_path(yaml_path)
        data = self._load_yaml_data(path)
        if not isinstance(data, dict):
            return []
        if isinstance(data.get("polylines"), list):
            return self._parse_polylines_data(data, frame_id=str(frame_id or "world"))
        if isinstance(data.get("path"), list):
            return [TargetPolyline(index=0, poses=self._parse_path_data(data, frame_id=str(frame_id or "world")))]
        return []

    def parse_yaml_path(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
    ) -> List[PoseStamped]:
        """
        Parse a YAML polyline file into a flat ordered pose list.

        Prefer `parse_yaml_polylines` when the caller needs to preserve the
        indexed polyline grouping.
        """
        polylines = self.parse_yaml_polylines(yaml_path=yaml_path, frame_id=frame_id)
        return self._flatten_polylines(polylines)

    def parse_yaml_segments(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
    ) -> List[TargetSegment]:
        """
        Parse `segments` YAML into ordered start/end PoseStamped pairs.

        Supported segment item shapes:
        - {index: 0, start: {plane: "..."}, end: {plane: "..."}}
        - {index: 0, start_index: 0, end_index: 1} with a sibling `targets` list
        """
        if yaml is None:
            raise RuntimeError("PyYAML is not available. Install python3-yaml.")

        path = self._resolve_yaml_path(yaml_path)
        data = self._load_yaml_data(path)
        if not isinstance(data, dict) or not isinstance(data.get("segments"), list):
            return []
        return self._parse_segments_data(data, frame_id=str(frame_id or "world"))

    @staticmethod
    def _load_yaml_data(path: Path) -> Any:
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    @staticmethod
    def _parse_path_data(data: dict, frame_id: str) -> List[PoseStamped]:
        ordered: List[Tuple[int, int, PoseStamped]] = []
        for pos, item in enumerate(data.get("path", [])):
            idx, ps = YamlSession._parse_target_item(
                item=item,
                fallback_index=pos,
                frame_id=frame_id,
            )
            order_idx = int(idx) if idx is not None else 1_000_000 + pos
            ordered.append((order_idx, pos, ps))

        ordered.sort(key=lambda row: (row[0], row[1]))
        return [ps for _, _, ps in ordered]

    @staticmethod
    def _parse_polylines_data(data: dict, frame_id: str) -> List[TargetPolyline]:
        ordered: List[Tuple[int, int, TargetPolyline]] = []
        for pos, item in enumerate(data.get("polylines", [])):
            if not isinstance(item, dict):
                raise ValueError(f"Polyline entry must be a map. Got: {item}")

            idx_raw = item.get("index", pos)
            idx = int(idx_raw) if idx_raw is not None else pos

            if isinstance(item.get("planes"), list):
                points = [{"plane": plane} if isinstance(plane, str) else plane for plane in item["planes"]]
            elif isinstance(item.get("path"), list):
                points = list(item["path"])
            else:
                raise ValueError("Polyline entry missing supported fields. Use 'planes' or 'path'.")

            poses = YamlSession._parse_polyline_points(points, frame_id=frame_id)
            if len(poses) < 2:
                raise ValueError(f"Polyline index {idx} requires at least 2 planes/poses. Got {len(poses)}.")

            ordered.append((idx, pos, TargetPolyline(index=idx, poses=poses)))

        ordered.sort(key=lambda row: (row[0], row[1]))
        return [polyline for _, _, polyline in ordered]

    @staticmethod
    def _parse_polyline_points(points: Sequence[Any], frame_id: str) -> List[PoseStamped]:
        ordered: List[Tuple[int, int, PoseStamped]] = []
        for pos, item in enumerate(points):
            idx, ps = YamlSession._parse_target_item(
                item=item,
                fallback_index=pos,
                frame_id=frame_id,
            )
            order_idx = int(idx) if idx is not None else 1_000_000 + pos
            ordered.append((order_idx, pos, ps))

        ordered.sort(key=lambda row: (row[0], row[1]))
        return [ps for _, _, ps in ordered]

    @staticmethod
    def _flatten_polylines(polylines: Sequence[TargetPolyline]) -> List[PoseStamped]:
        out: List[PoseStamped] = []
        for polyline in polylines:
            out.extend(polyline.poses)
        return out

    @staticmethod
    def _parse_segments_data(data: dict, frame_id: str) -> List[TargetSegment]:
        target_map: dict[int, PoseStamped] = {}
        if isinstance(data.get("targets"), list):
            for pos, item in enumerate(data["targets"]):
                idx, ps = YamlSession._parse_target_item(
                    item=item,
                    fallback_index=pos,
                    frame_id=frame_id,
                )
                if idx is not None:
                    target_map[int(idx)] = ps

        ordered: List[Tuple[int, int, TargetSegment]] = []
        for pos, item in enumerate(data.get("segments", [])):
            if not isinstance(item, dict):
                raise ValueError(f"Segment entry must be a map. Got: {item}")

            idx_raw = item.get("index", pos)
            idx = int(idx_raw) if idx_raw is not None else pos
            if "start" in item and "end" in item:
                _start_idx, start = YamlSession._parse_target_item(
                    item=item["start"],
                    fallback_index=2 * pos,
                    frame_id=frame_id,
                )
                _end_idx, end = YamlSession._parse_target_item(
                    item=item["end"],
                    fallback_index=2 * pos + 1,
                    frame_id=frame_id,
                )
            elif "start_index" in item and "end_index" in item:
                start_key = int(item["start_index"])
                end_key = int(item["end_index"])
                if start_key not in target_map or end_key not in target_map:
                    raise ValueError(
                        "Segment start_index/end_index references require matching entries in 'targets'. "
                        f"Missing start={start_key} or end={end_key}."
                    )
                start = target_map[start_key]
                end = target_map[end_key]
            else:
                raise ValueError(
                    "Segment entry missing supported fields. Use start/end targets or start_index/end_index."
                )

            ordered.append((idx, pos, TargetSegment(index=idx, start=start, end=end)))

        ordered.sort(key=lambda row: (row[0], row[1]))
        return [seg for _, _, seg in ordered]

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
                ps = pose_from_xyz_and_z_axis(xyz_m=xyz_m, z_axis=z_axis, frame_id=frame_id)
            elif "xyz" in item:
                xyz = YamlSession._coerce_xyz(item["xyz"])
                ps = pose_from_xyz_and_z_axis(
                    xyz_m=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                    z_axis=None,
                    frame_id=frame_id,
                )
            elif all(k in item for k in ("x", "y", "z")):
                ps = pose_from_xyz_and_z_axis(
                    xyz_m=(float(item["x"]), float(item["y"]), float(item["z"])),
                    z_axis=None,
                    frame_id=frame_id,
                )
            else:
                raise ValueError(f"Target entry missing supported fields (xyz/x,y,z/plane): {item}")
            return idx, ps

        if isinstance(item, (list, tuple)):
            xyz = YamlSession._coerce_xyz(item)
            ps = pose_from_xyz_and_z_axis(
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
    def _is_int_like(value: Any) -> bool:
        try:
            int(value)
            return True
        except Exception:
            return False
