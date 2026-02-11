#!/usr/bin/env python3
from __future__ import annotations

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
        motion_mode: str = "LIN",
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

        mode = str(motion_mode or "LIN").strip().upper()
        if mode not in ("LIN", "PTP"):
            raise ValueError(f"Unsupported motion_mode '{motion_mode}'. Use LIN or PTP.")

        self.run_sync(self.motion.setSpd(float(vel_scale), enqueue=False), timeout_s=timeout_s)
        self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
        if mode == "LIN":
            self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)
        else:
            self.run_sync(self.motion.setPTP(enqueue=False), timeout_s=timeout_s)

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

            plan_attempts = 2 if seq_idx == 0 else 1
            plan_res = None
            for attempt in range(plan_attempts):
                plan_res = self.run_sync(
                    self.motion.plan(
                        pose=ps,
                        motion=mode,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if plan_res.get("ok", False):
                    break
                if seq_idx == 0 and attempt == 0:
                    err = str(plan_res.get("error", "unknown")).strip()
                    log.warn(
                        "[yaml_sequence] First target plan failed once "
                        f"('{err}'). Waiting 0.5s and retrying."
                    )
                    self.run_sync(self.util.wait(0.5, enqueue=False), timeout_s=timeout_s)

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
            idx, xyz = self._parse_target_item(item=item, fallback_index=pos)
            ps = PoseStamped()
            ps.header.frame_id = str(frame_id or "world")
            ps.pose.position.x = float(xyz[0])
            ps.pose.position.y = float(xyz[1])
            ps.pose.position.z = float(xyz[2])
            ps.pose.orientation.w = 1.0

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
    def _parse_target_item(item: Any, fallback_index: int) -> Tuple[Optional[int], Sequence[float]]:
        if isinstance(item, dict):
            idx_raw = item.get("index", fallback_index)
            idx = int(idx_raw) if idx_raw is not None else None

            if "xyz" in item:
                xyz = YamlSession._coerce_xyz(item["xyz"])
            elif all(k in item for k in ("x", "y", "z")):
                xyz = (
                    float(item["x"]),
                    float(item["y"]),
                    float(item["z"]),
                )
            else:
                raise ValueError(f"Target entry missing xyz fields: {item}")
            return idx, xyz

        if isinstance(item, (list, tuple)):
            xyz = YamlSession._coerce_xyz(item)
            return fallback_index, xyz

        raise ValueError(f"Unsupported target entry type: {type(item).__name__}")

    @staticmethod
    def _coerce_xyz(value: Any) -> Tuple[float, float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"xyz must be a 3-element list/tuple. Got: {value}")
        return (float(value[0]), float(value[1]), float(value[2]))

    @staticmethod
    def _is_int_like(value: Any) -> bool:
        try:
            int(value)
            return True
        except Exception:
            return False
