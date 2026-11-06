"""Accumulate accepted segment YAML without re-rounding its target planes."""
from pathlib import Path
import re
import tempfile

import numpy as np
import yaml


def load_frame_yaml(path):
    """Read start/end planes for display without changing simulation state."""
    from field_scan_loop_simulator_dds_v2 import load_segment_targets_yaml
    targets = load_segment_targets_yaml(Path(path), position_scale=1000.0)
    points = np.stack((targets.start_points, targets.end_points), axis=1).reshape(-1, 3)
    z_dirs = np.stack((targets.start_z_dirs, targets.end_z_dirs), axis=1).reshape(-1, 3)
    return points, z_dirs


class TargetHistory:
    def __init__(self, output):
        self.path = Path(output) / "targets_history.yaml"
        self.blocks = []
        self.count = self.last_cycle = 0
        self.points = np.empty((0, 3))
        self.z_dirs = np.empty((0, 3))
        self._write([])

    def _write(self, blocks):
        header = "# Accepted targets in cycle order. Positions: mm; Z: direction vectors.\n"
        content = header + ("segments:\n" + "".join(blocks) if blocks else "segments: []\n")
        pending = self.path.with_name(".targets_history.pending.yaml")
        pending.write_text(content, encoding="utf-8")
        pending.replace(self.path)

    def add_batch(self, path, applied, cycle):
        if cycle != self.last_cycle + 1:
            raise ValueError("Target history batches must be accepted once in consecutive cycle order")
        text = Path(path).read_text(encoding="utf-8")
        rows = yaml.safe_load(text)["segments"]
        if len(rows) != applied.count or not rows:
            raise ValueError("Target history must match a nonempty accepted batch")
        lines = text.splitlines(keepends=True)
        if lines[0].strip() != "segments:":
            raise ValueError("Expected the canonical segments YAML writer format")
        index = self.count
        body = []
        for line in lines[1:]:
            if re.match(r"^  - index: \d+\s*$", line):
                line = f"  - index: {index}\n"
                index += 1
            body.append(line)
        if index != self.count + applied.count:
            raise ValueError("Unexpected target indices in accepted YAML")
        # Keep exact rounded plane strings and optional per-segment volumes.
        blocks = [*self.blocks, f"  # Cycle {cycle}\n" + "".join(body)]
        self._write(blocks)
        points = np.stack((applied.start_points, applied.end_points), axis=1).reshape(-1, 3)
        z_dirs = np.stack((applied.start_z_dirs, applied.end_z_dirs), axis=1).reshape(-1, 3)
        self.points = np.concatenate((self.points, points))
        self.z_dirs = np.concatenate((self.z_dirs, z_dirs))
        self.points.setflags(write=False)
        self.z_dirs.setflags(write=False)
        self.blocks, self.count, self.last_cycle = blocks, index, cycle


def rebuild_history(directory):
    """Rebuild an older session from its accepted per-step files only."""
    from field_scan_loop_simulator_dds_v2 import load_segment_targets_yaml
    directory = Path(directory)
    with tempfile.TemporaryDirectory(prefix=".target-history-", dir=directory) as temporary:
        history = TargetHistory(temporary)
        for path in sorted(directory.glob("step_*/segments.yaml")):
            cycle = int(path.parent.name.removeprefix("step_"))
            applied = load_segment_targets_yaml(path, position_scale=1000.0)
            history.add_batch(path, applied, cycle)
        destination = directory / history.path.name
        history.path.replace(destination)
        history.path = destination
    return history
