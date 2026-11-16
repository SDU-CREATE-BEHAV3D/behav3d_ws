"""Rectify calibrated raw images onto the original pinhole grid.

Depth uses nearest-neighbor sampling: interpolating across object boundaries
would invent depths. Keeping K and image dimensions preserves the downstream
TSDF projection and makes cropping operate in rectified pixel coordinates.
"""
import hashlib
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np

ALIGNMENT_METADATA = "alignment.json"
LEGACY_RECTIFIED_METADATA = "rectified_images.json"


def rectify_images(images, matrix, distortion, size, *, depth=False):
    matrix = np.asarray(matrix, dtype=np.float64)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    width, height = size
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("Rectification requires a finite 3x3 camera matrix")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
        raise ValueError("Invalid pinhole camera matrix")
    if distortion.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
        raise ValueError("Unsupported OpenCV distortion coefficients")
    if width <= 0 or height <= 0:
        raise ValueError("Rectification requires positive image dimensions")
    maps = cv2.initUndistortRectifyMap(matrix, distortion, None, matrix, (width, height), cv2.CV_32FC1)
    result = []
    for image in images:
        if image is None or image.shape[:2] != (height, width):
            raise ValueError("Image dimensions do not match the saved calibration")
        if depth and (image.ndim != 2 or image.dtype != np.uint16):
            raise ValueError("TSDF rectification requires single-channel uint16 depth")
        result.append(cv2.remap(image, *maps,
                                cv2.INTER_NEAREST if depth else cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0))
    return result


def alignment_signature(session):
    paths = {
        "manifest": Path(session.path) / session._scan_folder / "manifest.yaml",
        "depth_intrinsics": session.depth_intrinsics_path,
        "color_intrinsics": session.color_intrinsics_path,
        "extrinsics": session._camera_extrinsics_path,
    }
    return {key: hashlib.sha256(Path(path).read_bytes()).hexdigest() for key, path in paths.items()}


def aligned_image_paths(directory, frame_count):
    """Require one correctly indexed image per capture; never silently truncate."""
    directory = Path(directory)
    expected = [directory / f"color_in_depth_{i:04d}.png" for i in range(frame_count)]
    found = set(directory.glob("color_in_depth*.png"))
    if frame_count <= 0 or found != set(expected):
        missing = sorted(p.name for p in set(expected) - found)
        extra = sorted(p.name for p in found - set(expected))
        raise ValueError(f"Incomplete/misindexed color alignment: missing={missing}, extra={extra}; rerun color_to_depth")
    return expected


def write_alignment_metadata(directory, session, frame_count, *, complete,
                             rectified=True, run_id=None, started_at=None,
                             input_signature=None):
    path = Path(directory) / ALIGNMENT_METADATA
    path.parent.mkdir(parents=True, exist_ok=True)
    signature = alignment_signature(session)
    if input_signature is not None and signature != input_signature:
        raise ValueError("Manifest/calibration changed during color alignment; rerun color_to_depth")
    files = []
    if complete:
        for image_path in aligned_image_paths(directory, frame_count):
            if cv2.imread(str(image_path), cv2.IMREAD_COLOR) is None:
                raise ValueError(f"Unreadable aligned image: {image_path}")
            files.append(dict(name=image_path.name, sha256=hashlib.sha256(image_path.read_bytes()).hexdigest()))
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(dict(schema=2, rectified=bool(rectified), complete=complete,
                                         run_id=run_id or uuid.uuid4().hex,
                                         started_at=time.time() if started_at is None else started_at,
                                         completed_at=time.time() if complete else None,
                                         frame_count=frame_count, files=files,
                                         input_signature=signature), indent=2) + "\n")
    temporary.replace(path)


def validate_alignment_metadata(directory, session, frame_count, *, rectified):
    path = Path(directory) / ALIGNMENT_METADATA
    if not path.exists():
        path = Path(directory) / LEGACY_RECTIFIED_METADATA
    if not path.exists():
        if rectified:
            raise ValueError("Run color_to_depth with --rectify-images before rectified TSDF reconstruction")
        return  # Existing legacy color-in-depth folders remain supported.
    data = json.loads(path.read_text())
    if data.get("rectified") != bool(rectified):
        raise ValueError("Aligned color and TSDF use different rectification settings; rerun color_to_depth with the matching setting")
    if (data.get("schema") not in (1, 2)
            or data.get("complete") is not True or data.get("frame_count") != frame_count
            or data.get("input_signature") != alignment_signature(session)):
        raise ValueError("Color alignment is incomplete or its manifest/calibration has changed; rerun color_to_depth")
    if data.get("schema") == 2:
        expected = aligned_image_paths(directory, frame_count)
        files = data.get("files", [])
        if len(files) != frame_count or any(
                row.get("name") != p.name or row.get("sha256") != hashlib.sha256(p.read_bytes()).hexdigest()
                for row, p in zip(files, expected)):
            raise ValueError("Aligned image files changed after completion; rerun color_to_depth")
