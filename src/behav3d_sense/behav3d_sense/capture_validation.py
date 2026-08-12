#!/usr/bin/env python3
"""Helpers for validating newly written camera capture files."""

from pathlib import Path
from typing import Mapping, Optional


IMAGE_FIELDS = ("color", "depth", "ir")


def files_are_identical(first: Path, second: Path, chunk_size: int = 1024 * 1024) -> bool:
    """Return True only when two regular files contain exactly the same bytes."""
    try:
        if not first.is_file() or not second.is_file():
            return False
        if first.stat().st_size != second.stat().st_size:
            return False

        with first.open("rb") as first_file, second.open("rb") as second_file:
            while True:
                first_chunk = first_file.read(chunk_size)
                second_chunk = second_file.read(chunk_size)
                if first_chunk != second_chunk:
                    return False
                if not first_chunk:
                    return True
    except OSError:
        return False


def find_identical_capture_files(
    capture_dir: Path,
    previous_capture: Optional[Mapping[str, object]],
    current_capture: Mapping[str, object],
) -> dict[str, tuple[Path, Path]]:
    """Find image fields whose current file exactly matches the previous capture."""
    if not previous_capture:
        return {}

    duplicates: dict[str, tuple[Path, Path]] = {}
    for field in IMAGE_FIELDS:
        previous_relative = previous_capture.get(field)
        current_relative = current_capture.get(field)
        if not isinstance(previous_relative, str) or not isinstance(current_relative, str):
            continue
        if not previous_relative or not current_relative:
            continue

        previous_path = capture_dir / previous_relative
        current_path = capture_dir / current_relative
        if files_are_identical(previous_path, current_path):
            duplicates[field] = (previous_path, current_path)

    return duplicates
