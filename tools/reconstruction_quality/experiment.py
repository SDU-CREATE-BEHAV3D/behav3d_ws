#!/usr/bin/env python3
"""Replay immutable saved captures through a Git-pinned production TSDF pipeline."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
import open3d as o3d
import yaml

REVISION = "e0d25121105883fb6958c00a690329e839132c7b"
CASES = {
    "june_c1": ("260625_110936", "print_yaml_scan/cycle_0001/scan"),
    "july17_c0": ("260717_151744", "field_loop/cycle_0000/scan"),
    "orbit": ("260727_163002", "scan_yaml_targets"),
    "july28_c6": ("260728_133013", "field_loop/cycle_0006/scan"),
    "july28_c19": ("260728_133013", "field_loop/cycle_0019/scan"),
    "july29_c0": ("260729_165741", "field_loop/cycle_0000/scan"),
    "july29_c20": ("260729_165741", "field_loop/cycle_0020/scan"),
}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def intrinsics(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    try:
        return (fs.getNode("camera_matrix").mat(),
                fs.getNode("distortion_coefficients").mat().ravel(),
                (int(fs.getNode("image_width").real()), int(fs.getNode("image_height").real())))
    finally:
        fs.release()


def source_snapshot(repo, destination, revision, working_tree=False):
    prefix = "src/behav3d_sense/behav3d_sense/reconstruct/"
    files = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", revision, "--", prefix], cwd=repo, text=True).splitlines()
    if working_tree:
        files = sorted(str(p.relative_to(repo)) for p in (repo / prefix).rglob("*.py"))
    hashes = {}
    for name in files:
        if not name.endswith(".py"):
            continue
        path = destination / "reconstruct" / name[len(prefix):]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((repo / name).read_bytes() if working_tree else subprocess.check_output(["git", "show", f"{revision}:{name}"], cwd=repo))
        hashes[name] = digest(path)
    return hashes


def stage(source, scan_name, destination, selected, rectify):
    """Copy input files; never link writable experiment files to original captures."""
    (destination / "config").mkdir(parents=True)
    scan = destination / "scan"
    scan.mkdir()
    original = yaml.safe_load((source / scan_name / "manifest.yaml").read_text())
    provenance = {str(source / scan_name / "manifest.yaml"): digest(source / scan_name / "manifest.yaml")}
    for file in (source / "config").glob("*.yaml"):
        shutil.copy2(file, destination / "config" / file.name)
        provenance[str(file)] = digest(file)
    captures = original["captures"]
    selected = list(range(len(captures))) if selected is None else selected
    maps = {}
    if rectify:
        for kind in ("depth", "color"):
            k, d, size = intrinsics(source / "config" / f"{kind}_intrinsics.yaml")
            maps[kind] = cv2.initUndistortRectifyMap(k, d, None, k, size, cv2.CV_32FC1)
    staged = []
    for index in selected:
        capture = captures[index]
        if not isinstance(capture.get("T_base_tool0"), dict):
            raise ValueError(f"Missing pose: {index}")
        entry = dict(capture)
        for kind in ("depth", "color", "ir"):
            if not capture.get(kind):
                continue
            path = (source / scan_name / capture[kind]).resolve()
            if not path.is_relative_to((source / scan_name).resolve()):
                raise ValueError(f"Image escapes scan folder: {path}")
            target = scan / capture[kind]
            target.parent.mkdir(parents=True, exist_ok=True)
            provenance[str(path)] = digest(path)
            if rectify and kind in maps:
                image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                if image is None:
                    raise ValueError(f"Unreadable image: {path}")
                method = cv2.INTER_NEAREST if kind == "depth" else cv2.INTER_LINEAR
                image = cv2.remap(image, *maps[kind], method, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                if not cv2.imwrite(str(target), image):
                    raise OSError(f"Could not write {target}")
            else:
                shutil.copy2(path, target)
        staged.append(entry)
    original["captures"] = staged
    # Production ignores D, but explicitly describe the staged pinhole images.
    if rectify:
        for kind in ("depth", "color"):
            path = destination / "config" / f"{kind}_intrinsics.yaml"
            data = yaml.safe_load(path.read_text().replace("%YAML:1.0", "").replace("!!opencv-matrix", ""))
            data["distortion_coefficients"]["data"] = [0.0] * len(data["distortion_coefficients"]["data"])
            lines = ["%YAML:1.0", "---"]
            for key, value in data.items():
                if isinstance(value, dict) and "rows" in value:
                    lines.extend([f"{key}: !!opencv-matrix", f"   rows: {value['rows']}", f"   cols: {value['cols']}", "   dt: d", f"   data: {value['data']}"])
                else:
                    lines.append(f"{key}: {value}")
            path.write_text("\n".join(lines) + "\n")
    (scan / "manifest.yaml").write_text(yaml.safe_dump(original, sort_keys=False))
    return provenance, selected


def mesh_metrics(path):
    mesh = o3d.io.read_triangle_mesh(str(path))
    mesh.remove_duplicated_vertices()
    vertices, tri = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    if len(tri) == 0:
        return {"vertices": len(vertices), "triangles": 0}
    _, sizes, areas = mesh.cluster_connected_triangles()
    sizes, areas = np.asarray(sizes), np.asarray(areas)
    edges = np.sort(np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]), axis=1)
    unique_edges, counts = np.unique(edges, axis=0, return_counts=True)
    boundary = unique_edges[counts == 1]
    return dict(vertices=len(vertices), triangles=len(tri), components=len(sizes),
                tiny_triangles=int(sizes[sizes < 50].sum()), area_m2=float(areas.sum()),
                largest_component_area_fraction=float(areas.max() / areas.sum()),
                boundary_edges=len(boundary), nonmanifold_edges=int((counts > 2).sum()),
                boundary_length_m=float(np.linalg.norm(vertices[boundary[:, 0]] - vertices[boundary[:, 1]], axis=1).sum()),
                bounds_m=[vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()])


def run(args):
    source = args.data / CASES[args.case][0]
    scan_name = CASES[args.case][1]
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to replace experiment: {output}")
    if output.is_relative_to(source.resolve()):
        raise ValueError("Experiment output must not be inside source captures")
    output.mkdir(parents=True)
    repo = Path(__file__).resolve().parents[2]
    hashes = source_snapshot(repo, output / "source", args.revision, args.working_tree)
    t0 = time.perf_counter()
    selected = None if args.frames is None else [int(x) for x in args.frames.split(",")]
    provenance, selected = stage(source, scan_name, output / "session", selected, args.variant == "rectified")
    stage_seconds = time.perf_counter() - t0
    sys.path.insert(0, str(output / "source"))
    from reconstruct import color_to_depth, TSDF_cpu_cropped as tsdf
    # Explicit experimental parameters; the production implementation stays pinned.
    # Keep truncation in meters constant when testing resolution independently.
    original_class = tsdf.TSDF_Integration

    class GridParameters:
        def __init__(self, grid):
            self.grid = grid

        def __getattr__(self, key):
            return getattr(self.grid, key)

        def compute_unique_block_coordinates(self, *values):
            return self.grid.compute_unique_block_coordinates(*values, trunc_voxel_multiplier=args.truncation_m / args.voxel_size)

        def integrate(self, *values):
            return self.grid.integrate(*values, trunc_voxel_multiplier=args.truncation_m / args.voxel_size)

        def extract_triangle_mesh(self):
            return self.grid.extract_triangle_mesh(weight_threshold=args.weight_threshold)

    class ExperimentalTSDF(original_class):
        def __init__(self, *values, **kwargs):
            super().__init__(*values, voxel_size=args.voxel_size, **kwargs)
            self.vbg = GridParameters(self.vbg)

    if (args.voxel_size, args.truncation_m, args.weight_threshold) != (1/512, 8/512, 3.0):
        tsdf.TSDF_Integration = ExperimentalTSDF
    o3d.utility.random.seed(42)
    cv2.setNumThreads(1)
    settings = dict(center_crop_enable=True, center_crop_width=args.crop_width,
                    center_crop_height=args.crop_height, center_crop_apply_to_depth=True,
                    aabb_crop_enable=False, auto_object_crop_enable=False)
    if args.crop_width == 0:
        settings.update(center_crop_enable=False, center_crop_apply_to_depth=False)
    record = dict(case=args.case, source_session=str(source), source_scan=scan_name,
                  revision=args.revision, working_tree_snapshot=args.working_tree, source_sha256=hashes, input_sha256=provenance,
                  selected_manifest_positions=selected, variant=args.variant, crop=settings,
                  seed=42, device="CPU:0", python=sys.version, platform=platform.platform(),
                  executable=sys.executable, threads={k: os.environ.get(k) for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS")},
                  packages={p: importlib.metadata.version(p) for p in ("open3d", "numpy", "scipy", "opencv-python", "PyYAML", "matplotlib")},
                  defaults=dict(voxel_size_m=args.voxel_size, depth_max_m=0.9, depth_scale=1000.0,
                                depth_bias_mm=-5.1, effective_integer_bias_mm=-5,
                                trunc_voxel_multiplier=args.truncation_m / args.voxel_size, extraction_weight_threshold=args.weight_threshold,
                                mesh_min_component_triangles=50, pointcloud_min_observations=3,
                                pointcloud_z_band_m=0.02, pointcloud_outlier_neighbors=35,
                                pointcloud_outlier_std_ratio=1.5),
                  stage_seconds=stage_seconds,
                  runner_sha256=digest(Path(__file__)),
                  command=sys.argv)
    shutil.copy2(__file__, output / "experiment_runner.py")
    write_json(output / "run.json", record)
    t0 = time.perf_counter()
    extra = {"rectify_images": True} if args.variant == "production_rectified" else {}
    color_to_depth.run(session_path=str(output / "session"), scan_folder="scan", visualize=False, **extra)
    record["color_alignment_seconds"] = time.perf_counter() - t0
    t0 = time.perf_counter()
    tsdf.run(session_path=str(output / "session"), scan_folder_override="scan", visualize=False, device="CPU:0", **settings, **extra)
    record["tsdf_seconds"] = time.perf_counter() - t0
    products = output / "session/scan/reconstruct"
    record["mesh_metrics"] = mesh_metrics(products / "tsdf_surface_mesh.ply")
    record["output_sha256"] = {str(p.relative_to(output)): digest(p) for p in products.rglob("*") if p.is_file()}
    record["source_inputs_unchanged"] = all(digest(p) == h for p, h in provenance.items())
    if not record["source_inputs_unchanged"]:
        raise RuntimeError("Source inputs changed during replay")
    write_json(output / "run.json", record)
    print(json.dumps({k: record[k] for k in ("case", "variant", "tsdf_seconds", "mesh_metrics", "source_inputs_unchanged")}))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--case", choices=CASES, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--revision", default=REVISION)
    p.add_argument("--variant", choices=("baseline", "rectified", "production_rectified"), default="baseline")
    p.add_argument("--working-tree", action="store_true", help="Snapshot local reconstruction Python files, recording their hashes")
    p.add_argument("--crop-width", type=int, default=270)
    p.add_argument("--crop-height", type=int, default=290)
    p.add_argument("--frames", help="Comma-separated zero-based manifest positions; all frames by default")
    p.add_argument("--voxel-size", type=float, default=1/512)
    p.add_argument("--truncation-m", type=float, default=8/512)
    p.add_argument("--weight-threshold", type=float, default=3.0)
    run(p.parse_args())
