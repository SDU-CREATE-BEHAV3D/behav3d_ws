#!/usr/bin/env python3
"""Render fixed-axis height maps of saved meshes and replay variants."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from experiment import CASES

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--data", type=Path, required=True)
parser.add_argument("--runs", type=Path, nargs="+", required=True)
parser.add_argument("--cases", nargs="+", default=["june_c1", "orbit", "july28_c19", "july29_c20"])
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
fig, axes = plt.subplots(len(args.cases), len(args.runs) + 1, figsize=(5 * (len(args.runs) + 1), 3.7 * len(args.cases)), squeeze=False)
for row, case in enumerate(args.cases):
    session, scan = CASES[case]
    reference = args.data / session / scan / "reconstruct"
    paths = [reference / "tsdf_surface_mesh.stl"] + [r / case / "session/scan/reconstruct/tsdf_surface_mesh.ply" for r in args.runs]
    titles = ["Saved output (settings unrecorded)"] + [r.name for r in args.runs]
    meshes = [o3d.io.read_triangle_mesh(str(p)) for p in paths]
    clouds = [np.asarray(mesh.vertices) for mesh in meshes]
    plane = np.array(json.loads((reference / "table_plane.json").read_text())["plane_model"])
    # Fixed object/table footprint from original mesh; identical axes and colors in each row.
    lower, upper = np.min(np.concatenate(clouds), axis=0), np.max(np.concatenate(clouds), axis=0)
    heights = [c @ plane[:3] + plane[3] for c in clouds]
    vmax = max(25, float(np.percentile(np.concatenate(heights), 99.5) * 1000))
    xs = np.linspace(lower[0] - .01, upper[0] + .01, 720)
    ys = np.linspace(lower[1] - .01, upper[1] + .01, 720)
    xx, yy = np.meshgrid(xs, ys)
    top = upper[2] + .05
    rays = np.stack([xx, yy, np.full_like(xx, top), np.zeros_like(xx), np.zeros_like(xx), -np.ones_like(xx)], axis=-1).astype(np.float32)
    for col, (mesh, title) in enumerate(zip(meshes, titles)):
        ax = axes[row, col]
        scene = o3d.t.geometry.RaycastingScene(nthreads=4)
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        hit = scene.cast_rays(o3d.core.Tensor(rays), nthreads=4)["t_hit"].numpy()
        height = (xx * plane[0] + yy * plane[1] + (top - hit) * plane[2] + plane[3]) * 1000
        height = np.ma.masked_invalid(height)
        im = ax.imshow(height, origin="lower", extent=[xs[0]*1000, xs[-1]*1000, ys[0]*1000, ys[-1]*1000], cmap="viridis", vmin=-5, vmax=vmax, interpolation="nearest")
        ax.set(xlim=(lower[0] * 1000 - 10, upper[0] * 1000 + 10), ylim=(lower[1] * 1000 - 10, upper[1] * 1000 + 10), title=f"{case} · {title}", xlabel="Base X (mm)", ylabel="Base Y (mm)")
        ax.set_aspect("equal")
        ax.set_facecolor("#f0f0f0")
    fig.colorbar(im, ax=axes[row, :].tolist(), label="Height above saved fitted plane (mm)", fraction=.015, pad=.02)
fig.suptitle("Saved reconstruction vs controlled replays\nFixed coordinates and height scale within each row; blank regions are missing surface", fontsize=15)
fig.subplots_adjust(top=.90, bottom=.04, left=.06, right=.87, hspace=.38, wspace=.23)
fig.savefig(args.output, dpi=170)
print(args.output)
