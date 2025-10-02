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

        self._queue_handlers: Dict[str, Callable[[Any], None]] = {
            "home": lambda v: self.cmd.home(**self._ensure_dict(v)),
            "goto": lambda v: self.cmd.goto(**self._ensure_dict(v)),
            "print": lambda v: self.cmd.print(**self._ensure_dict(v)),
            "wait": lambda v: self.cmd.wait(float(v)),
        }

    def run_file(self, path: str):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, list):
            raise ValueError("Top-level YAML must be a list of commands.")

        for i, item in enumerate(data, start=1):

            # --- NEW: scalar config commands like "- LIN" or "- PTP"
            if isinstance(item, str):
                key = item.strip()
                if key in self._config_handlers:
                    self._config_handlers[key](None)
                    if self.log:
                        self.log.info(f"CFG: {key}")
                    continue
                raise ValueError(f"Unknown scalar command '{key}' at item #{i}")

            # --- existing: mapping items ---
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
