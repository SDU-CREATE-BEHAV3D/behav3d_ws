#!/usr/bin/env python3
from __future__ import annotations

import shutil
from pathlib import Path


def snapshot_cycle_config(source_path: str | Path, cycle_root: str | Path) -> tuple[Path, bool]:
    """Copy one immutable runtime-config snapshot into a cycle directory."""
    source = Path(source_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Runtime config file not found: {source}")

    output_dir = Path(cycle_root).expanduser().resolve() / "config"
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / source.name
    if destination.exists():
        return destination, False

    shutil.copy2(source, destination)
    return destination, True
