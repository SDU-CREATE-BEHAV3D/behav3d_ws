from pathlib import Path

from behav3d_sense.capture_validation import (
    files_are_identical,
    find_identical_capture_files,
)


def _write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def test_files_are_identical_compares_content_exactly(tmp_path):
    first = tmp_path / "first.png"
    same = tmp_path / "same.png"
    different = tmp_path / "different.png"
    other_size = tmp_path / "other_size.png"
    _write(first, b"exact camera frame")
    _write(same, b"exact camera frame")
    _write(different, b"exact camera FRAME")
    _write(other_size, b"short")

    assert files_are_identical(first, same)
    assert not files_are_identical(first, different)
    assert not files_are_identical(first, other_size)
    assert not files_are_identical(first, tmp_path / "missing.png")


def test_find_identical_capture_files_reports_each_matching_modality(tmp_path):
    _write(tmp_path / "color_raw/color_i0.png", b"same color")
    _write(tmp_path / "color_raw/color_i1.png", b"same color")
    _write(tmp_path / "depth_raw/depth_i0.png", b"old depth")
    _write(tmp_path / "depth_raw/depth_i1.png", b"new depth")

    duplicates = find_identical_capture_files(
        tmp_path,
        {
            "color": "color_raw/color_i0.png",
            "depth": "depth_raw/depth_i0.png",
            "ir": None,
        },
        {
            "color": "color_raw/color_i1.png",
            "depth": "depth_raw/depth_i1.png",
            "ir": None,
        },
    )

    assert set(duplicates) == {"color"}
    assert duplicates["color"] == (
        tmp_path / "color_raw/color_i0.png",
        tmp_path / "color_raw/color_i1.png",
    )


def test_first_capture_has_nothing_to_compare(tmp_path):
    _write(tmp_path / "color_raw/color_i0.png", b"frame")

    assert find_identical_capture_files(
        tmp_path,
        None,
        {"color": "color_raw/color_i0.png"},
    ) == {}
