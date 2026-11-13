#!/usr/bin/env python3
"""Fixed oblique orthographic mesh renders and common depth-difference maps."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from experiment import write_json

# Frozen from saved-output inspection; same camera for every algorithm.
VIEWS = {
    "orbit": dict(center=[-.075, -.875, -.100], width=.39, height=.30),
    "july28_c19": dict(center=[.24, -.89, -.09], width=.42, height=.33),
    "july29_c20": dict(center=[.19, -.89, -.05], width=.39, height=.35),
}


def cast(mesh, camera):
    forward = -np.array([.7, -1., .65])
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    x, y = np.meshgrid(np.linspace(-camera["width"]/2, camera["width"]/2, 900), np.linspace(-camera["height"]/2, camera["height"]/2, 700))
    origin = np.array(camera["center"]) - forward
    origins = origin + x[..., None]*right + y[..., None]*up
    rays = np.concatenate([origins, np.broadcast_to(forward, origins.shape)], axis=-1).astype(np.float32)
    scene = o3d.t.geometry.RaycastingScene(nthreads=4)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    hits = scene.cast_rays(o3d.core.Tensor(rays), nthreads=4)
    distance = hits["t_hit"].numpy()
    normal = hits["primitive_normals"].numpy()
    valid = np.isfinite(distance)
    light = np.array([-.4, -.6, .8])
    light /= np.linalg.norm(light)
    intensity = .28 + .72*np.abs(normal @ light)
    color = np.array([.58, .72, .77])[None, None, :]*intensity[..., None]
    color[~valid] = .96
    return color, distance


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--runs", type=Path, nargs=2, required=True)
parser.add_argument("--labels", nargs=2, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--cases", nargs="+", default=list(VIEWS), choices=list(VIEWS))
args = parser.parse_args()
fig, axes = plt.subplots(len(args.cases), 3, figsize=(14, 3.5*len(args.cases)+.5), squeeze=False)
details = {}
for row, case in enumerate(args.cases):
    camera = VIEWS[case]
    surfaces = []
    for col, run in enumerate(args.runs):
        mesh = o3d.io.read_triangle_mesh(str(run / case / "session/scan/reconstruct/tsdf_surface_mesh.ply"))
        color, distance = cast(mesh, camera)
        surfaces.append(distance)
        axes[row, col].imshow(color, origin="lower")
        axes[row, col].set_title(f"{case} · {args.labels[col]}")
    common = np.isfinite(surfaces[0]) & np.isfinite(surfaces[1])
    change = np.full_like(surfaces[0], np.nan)
    change[common] = (surfaces[1][common] - surfaces[0][common])*1000
    im = axes[row, 2].imshow(np.ma.masked_invalid(change), origin="lower", cmap="coolwarm", vmin=-5, vmax=5)
    axes[row, 2].set_title("Candidate − baseline view depth (mm)")
    axes[row, 2].set_facecolor("#eeeeee")
    fig.colorbar(im, ax=axes[row, 2], fraction=.04, pad=.02)
    details[case] = dict(camera=camera, common_hits=int(common.sum()),
                         median_abs_view_change_mm=float(np.median(np.abs(change[common]))),
                         p95_abs_view_change_mm=float(np.percentile(np.abs(change[common]), 95)),
                         note="Difference is geometric change, not accuracy. Both table and object are visible.")
for ax in axes.flat:
    ax.set_axis_off()
fig.suptitle("Identical cameras, lighting and full triangle meshes\nNo smoothing, hole filling or visual mesh decimation", fontsize=14)
fig.tight_layout(rect=[0, 0, 1, .90 if len(args.cases) == 1 else .95])
fig.savefig(args.output, dpi=170)
write_json(args.output.with_suffix(".json"), details)
print(args.output)
