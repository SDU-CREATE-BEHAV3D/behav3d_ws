#!/usr/bin/env python3
"""Audit saved captures and meshes without changing them."""
import argparse
from collections import Counter
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from experiment import digest, intrinsics, mesh_metrics, write_json

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--data", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
args = p.parse_args()
result = []
for root in sorted(args.data.glob("26????_??????")):
    if not (root / "config/depth_intrinsics.yaml").exists():
        continue
    k, d, size = intrinsics(root / "config/depth_intrinsics.yaml")
    config_hashes = {str(f): digest(f) for f in (root / "config").glob("*.yaml")}
    for manifest in sorted(root.rglob("manifest.yaml")):
        captures = yaml.safe_load(manifest.read_text()).get("captures", [])
        hashes = dict(config_hashes)
        hashes[str(manifest)] = digest(manifest)
        missing, invalid_poses, depths, depth_hashes = [], [], [], []
        for position, c in enumerate(captures):
            pose = c.get("T_base_tool0")
            if not isinstance(pose, dict) or len(pose.get("position", [])) != 3 or len(pose.get("orientation_xyzw", [])) != 4:
                invalid_poses.append(position)
            for kind in ("depth", "color", "ir"):
                path = manifest.parent / str(c.get(kind, "MISSING"))
                if not path.is_file():
                    missing.append(str(path))
                    continue
                hashes[str(path)] = digest(path)
                if kind == "depth":
                    depth_hashes.append(hashes[str(path)])
                    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                    if image is None:
                        raise ValueError(f"Unreadable depth {path}")
                    valid = (image > 0) & (image < 900)
                    depths.append(dict(shape=list(image.shape), dtype=str(image.dtype), valid_under_900mm_fraction=float(valid.mean()), median_nonzero_mm=float(np.median(image[image > 0]))))
        output = manifest.parent / "reconstruct/tsdf_surface_mesh.stl"
        metric = mesh_metrics(output) if output.exists() else None
        if output.exists():
            hashes[str(output)] = digest(output)
        row = dict(session=root.name, scan=str(manifest.parent.relative_to(root)), frames=len(captures),
                   missing=missing, invalid_poses=invalid_poses, depth_images=depths,
                   duplicate_depth_files=len(depth_hashes)-len(set(depth_hashes)),
                   depth_intrinsics=dict(matrix=k.tolist(), distortion=d.tolist(), size=list(size)),
                   mesh=metric, source_sha256=hashes)
        result.append(row)
    print(root.name, "audited", flush=True)
write_json(args.output, result)
