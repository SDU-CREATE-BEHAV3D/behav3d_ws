#!/usr/bin/env python3
"""Compare full indexed meshes independent of Open3D vertex/triangle ordering."""
import argparse
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from experiment import write_json


def compare(first, second):
    a = o3d.io.read_triangle_mesh(str(first))
    b = o3d.io.read_triangle_mesh(str(second))
    av, bv = np.asarray(a.vertices), np.asarray(b.vertices)
    da, ia = cKDTree(bv).query(av)
    db, _ = cKDTree(av).query(bv)
    maximum = float(max(da.max(), db.max()))
    at = np.sort(ia[np.asarray(a.triangles)], axis=1)
    bt = np.sort(np.asarray(b.triangles), axis=1)
    at = at[np.lexsort(at.T[::-1])]
    bt = bt[np.lexsort(bt.T[::-1])]
    equal = len(av) == len(bv) and maximum < 1e-7 and np.array_equal(at, bt)
    return dict(first=str(first), second=str(second), first_vertices=len(av), second_vertices=len(bv),
                first_triangles=len(at), second_triangles=len(bt), max_vertex_difference_mm=maximum*1000,
                same_triangle_connectivity=bool(np.array_equal(at, bt)), equivalent=bool(equal))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("first", type=Path)
    p.add_argument("second", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    results = {}
    for path in sorted(args.first.glob("*/session/scan/reconstruct/tsdf_surface_mesh.ply")):
        other = args.second / path.relative_to(args.first)
        if other.exists():
            results[path.parents[3].name] = compare(path, other)
    if not results:
        raise ValueError("No matching completed runs")
    write_json(args.output, results)
    print(json.dumps({k: {x: v[x] for x in ("equivalent", "max_vertex_difference_mm", "same_triangle_connectivity")} for k,v in results.items()}, indent=2))
    if not all(v["equivalent"] for v in results.values()):
        raise SystemExit(1)
