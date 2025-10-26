#!/usr/bin/env python3
from typing import Any, Dict, List, Callable
import yaml

class SequenceParser:
    """
    Maps YAML items to Commands API.
    Option A: config setters (EEF/SPD/ACC/PTP/LIN) are applied immediately (not queued).
    """

    def __init__(self, cmd, *, logger=None):
        self.cmd = cmd
        self.log = logger or getattr(cmd.node, "get_logger", lambda: None)()  # optional

        # Dispatch tables
        self._config_handlers: Dict[str, Callable[[Any], None]] = {
            "EEF": lambda v: self.cmd.EEF(str(v)),
            "SPD": lambda v: self.cmd.SPD(float(v)),
            "ACC": lambda v: self.cmd.ACC(float(v)),
            "PTP": lambda v=None: self.cmd.PTP(),
            "LIN": lambda v=None: self.cmd.LIN(),
        }

        # --- pose parser helper ---
        def _pose_handler(v: Any):
            params = self._parse_pose(v)
            # Always queued: respects FIFO inside Commands
            self.cmd.getPose(**params)

        self._queue_handlers: Dict[str, Callable[[Any], None]] = {
            "home":  lambda v: self.cmd.home(**self._ensure_dict(v)),
            "goto":  lambda v: self.cmd.goto(**self._ensure_dict(v)),
            "print": lambda v: self.cmd.print(**self._ensure_dict(v)),
            "wait":  lambda v: self.cmd.wait(float(v)),
            "input":   lambda v: self._handle_input(v),
            "capture": lambda v: self._handle_capture(v),
            # ---  pose/getPose map to the same handler ---
            "pose": _pose_handler,
            "getPose": _pose_handler,
        }

    # Optional: default callback for pose to print a short line
    def _on_pose_log(self, res: Dict[str, Any]):
        log = self.log
        if not log:
            return
        if not res.get("ok", False):
            log.error(f"POSE failed: {res.get('error')}")
            return
        ps = res["pose"]
        base = res.get("base_frame") or "(planning frame)"
        link = res.get("link", "?")
        src = res.get("metrics", {}).get("source", "?")
        p, q = ps.pose.position, ps.pose.orientation
        log.info(
            f"[{src}] {link} in {base}: "
            f"p=({p.x:.5f},{p.y:.5f},{p.z:.5f}) "
            f"q=({q.x:.5f},{q.y:.5f},{q.z:.5f},{q.w:.5f})"
        )

    def run_file(self, path: str):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, list):
            raise ValueError("Top-level YAML must be a list of commands.")

        for i, item in enumerate(data, start=1):

            # Scalar config commands like "- LIN" or "- PTP"
            if isinstance(item, str):
                key = item.strip()
                if key in self._config_handlers:
                    self._config_handlers[key](None)
                    if self.log:
                        self.log.info(f"CFG: {key}")
                    continue
                raise ValueError(f"Unknown scalar command '{key}' at item #{i}")

            # Mapping items
            if not isinstance(item, dict) or len(item) != 1:
                raise ValueError(f"Item #{i} must be a single-key mapping, got: {item!r}")

            key, value = next(iter(item.items()))

            # Config (immediate)
            if key in self._config_handlers:
                self._config_handlers[key](value)
                if self.log:
                    self.log.info(f"CFG: {key} -> {value}")
                continue

            # Queued commands (FIFO)
            if key in self._queue_handlers:
                self._queue_handlers[key](value)
                if self.log:
                    self.log.info(f"ENQ: {key} {value}")
                continue

            raise ValueError(f"Unknown YAML command key: {key!r} at item #{i}")

    @staticmethod
    def _ensure_dict(val: Any) -> Dict[str, Any]:
        if val is None:
            return {}
        if isinstance(val, dict):
            return val
        raise ValueError(f"Expected mapping/dict, got: {type(val).__name__}")

    def _parse_pose(self, val: Any) -> Dict[str, Any]:
        """
        Accepts:
          - scalar: 'extruder_tcp'
          - mapping: { eef: 'extruder_tcp', base: 'tool0', tf: true }
        Produces kwargs for Commands.getPose(eef, base_frame, use_tf, on_done=None)
        """
        if isinstance(val, str):
            return {"eef": val, "base_frame": "world", "use_tf": False, "on_done": getattr(self, "_on_pose_log", None)}
        if isinstance(val, dict):
            eef = str(val.get("eef") or val.get("link") or val.get("EEF") or val.get("LINK"))
            if not eef:
                raise ValueError("pose: missing 'eef' (or 'link') field")
            base = val.get("base")
            # None -> MoveIt planning frame (empty string), otherwise string
            base_frame = None if base is None else str(base)
            use_tf = bool(val.get("tf", False))
            return {
                "eef": eef,
                "base_frame": base_frame if base_frame is not None else None,
                "use_tf": use_tf,
                "on_done": getattr(self, "_on_pose_log", None),
            }
        raise ValueError(f"pose: expected scalar or mapping, got {type(val).__name__}")

    def _handle_input(self, val: Any):
        """
        YAML forms:
        - input
        - input: null
        - input: "q" NOT WORKING DUE TO YAML PARSING
        - input: { key: "q", prompt: "Type 'q' + ENTER to shutdown..." }
        """
        if val is None:
            self.cmd.input()  # wait for ENTER
            return

        if isinstance(val, str):
            self.cmd.input(key=val)
            return

        if isinstance(val, dict):
            key = val.get("key", None)
            prompt = val.get("prompt", None)
            self.cmd.input(key=key, prompt=prompt)
            return

        raise ValueError(f"input: expected null, string, or mapping; got {type(val).__name__}")

    def _handle_capture(self, val: Any):
        """
        YAML forms:
        - capture                      # no streams, no folder change
        - capture: { rgb: true, depth: true, ir: true }      # pose omitted -> False
        - capture: { rgb: true, depth: true, ir: true, pose: true }
        - capture: { folder: "" }       # set_folder=True, folder=""
        - capture: { folder: "/data/run_042" }  # set_folder=True to given path
        - capture: { rgb: true, folder: "/tmp" }             # mix flags + folder
        """
        if val is None:
            self.cmd.capture()  # all False, no folder change
            return

        if not isinstance(val, dict):
            raise ValueError(f"capture: expected mapping, got {type(val).__name__}")

        rgb   = bool(val.get("rgb", False))
        depth = bool(val.get("depth", False))
        ir    = bool(val.get("ir", False))
        pose  = bool(val.get("pose", False))

        # folder: None -> do not change; "" -> clear; "path" -> set to that path
        folder = val.get("folder", None)
        if folder is not None:
            folder = str(folder)

        self.cmd.capture(rgb=rgb, depth=depth, ir=ir, pose=pose, folder=folder)
