#!/usr/bin/env python3
"""Render paired November meshes and table-aligned cube dimensions in millimetres.

Boxes enclose all cube-region vertices above a fixed three-voxel table exclusion.
Their base is the same saved support plane in both variants. No cube size is
assumed, and no smoothing, face fitting, or removal of cube artefacts is applied.
"""
from __future__ import annotations
import argparse
from itertools import product
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from experiment import digest, write_json
from november_results import render


CASES = {
    '261113_145255': ('Cube', 'Linear barrel'),
    '261113_150006': ('Cube', 'Oriented barrel'),
    '261113_150432': ('Cube', 'Planar'),
    '261113_150709': ('Cone', 'Linear barrel'),
    '261113_151101': ('Cone', 'Oriented barrel'),
    '261113_151435': ('Cone', 'Planar'),
    '261113_151637': ('Sphere', 'Linear barrel'),
    '261113_152024': ('Sphere', 'Oriented barrel'),
    '261113_152345': ('Sphere', 'Planar'),
}
VOXEL = 1 / 512
FLOOR = 3 * VOXEL
CORNER_BITS = np.array(list(product((0, 1), repeat=3)))
EDGES = [(i, j) for i in range(8) for j in range(i+1, 8)
         if np.abs(CORNER_BITS[i]-CORNER_BITS[j]).sum() == 1]
FORWARD = -np.array([.7, -1., .8])
FORWARD /= np.linalg.norm(FORWARD)
RIGHT = np.cross(FORWARD, [0., 0., 1.])
RIGHT /= np.linalg.norm(RIGHT)
UP = np.cross(RIGHT, FORWARD)
VARIANTS = ('uncorrected', 'corrected')
TITLES = ('Without distortion correction', 'With distortion correction')


def reference_frame(plane):
    plane = np.asarray(plane, float) / np.linalg.norm(plane[:3])
    if plane[2] < 0:
        plane = -plane
    z = plane[:3]
    x = np.array([1., 0., 0.])
    x -= (x @ z) * z
    x /= np.linalg.norm(x)
    axes = np.column_stack((x, np.cross(z, x), z))
    origin = -plane[3] * z
    assert np.allclose(axes.T @ axes, np.eye(3), atol=1e-12)
    return origin, axes


def measure(vertices, plane, target, floor=FLOOR):
    origin, axes = reference_frame(plane)
    local = (vertices-origin) @ axes
    center = (np.asarray(target)-origin) @ axes
    # Broad neighbourhood locates the cube, with no nominal side-length prior.
    region = ((np.abs(local[:, 0]-center[0]) < .10)
              & (np.abs(local[:, 1]-center[1]) < .10)
              & (local[:, 2] > floor) & (local[:, 2] < .15))
    points = local[region]
    if len(points) < 100:
        raise ValueError('Too few isolated cube vertices')
    low, high = points.min(axis=0), points.max(axis=0)
    if np.any(np.abs(points[:, :2]-center[:2]).max(axis=0) > .095) or high[2] > .145:
        raise ValueError('Cube-region geometry approaches the location crop; enlarge and recheck')
    low[2] = 0.  # Full height is referenced to the same support plane in both versions.
    corners_local = low + CORNER_BITS * (high-low)
    corners_world = corners_local @ axes.T + origin
    tolerance = 1e-10
    assert np.all(points >= low-tolerance) and np.all(points <= high+tolerance)
    assert np.allclose((corners_world-origin) @ axes, corners_local, atol=tolerance)
    for i, j in EDGES:
        direction = np.flatnonzero(CORNER_BITS[i] != CORNER_BITS[j])[0]
        assert np.isclose(np.linalg.norm(corners_world[i]-corners_world[j]), high[direction]-low[direction])
    return dict(dimensions_mm=((high-low)*1000).tolist(), selected_vertices=int(region.sum()),
                minimum_local_m=low.tolist(), maximum_local_m=high.tolist(),
                corners_base_m=corners_world.tolist(), frame_origin_base_m=origin.tolist(),
                frame_axes_columns=axes.tolist(), table_exclusion_mm=floor*1000,
                support_plane=np.asarray(plane).tolist())


def project(points, center):
    delta = np.asarray(points)-center
    return np.stack((delta @ RIGHT, delta @ UP), axis=-1)*1000


def dimension_lines(bounds):
    low, high = np.array(bounds['minimum_local_m']), np.array(bounds['maximum_local_m'])
    result = []
    # Dimension lines are displaced outside the box, retaining actual edge length.
    for axis, p, q in (
        (0, [low[0], low[1]-.010, 0.], [high[0], low[1]-.010, 0.]),
        (1, [high[0]+.010, low[1], 0.], [high[0]+.010, high[1], 0.]),
        (2, [low[0]-.010, low[1]-.006, 0.], [low[0]-.010, low[1]-.006, high[2]])):
        local = np.array([p, q])
        world = local @ np.array(bounds['frame_axes_columns']).T + bounds['frame_origin_base_m']
        result.append((axis, world))
    return result


def mesh_image(mesh, center, extent, width=1000):
    xmin, xmax, ymin, ymax = np.array(extent)/1000
    height = int(round(width*(ymax-ymin)/(xmax-xmin)))
    x, y = np.meshgrid(np.linspace(xmin, xmax, width), np.linspace(ymin, ymax, height))
    origins = center-FORWARD + x[..., None]*RIGHT + y[..., None]*UP
    rays = np.concatenate([origins, np.broadcast_to(FORWARD, origins.shape)], axis=-1).astype(np.float32)
    scene = o3d.t.geometry.RaycastingScene(nthreads=4)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    hits = scene.cast_rays(o3d.core.Tensor(rays), nthreads=4)
    light = np.array([-.4, -.6, .8]); light /= np.linalg.norm(light)
    intensity = .25 + .75*np.abs(hits['primitive_normals'].numpy() @ light)
    color = np.array([.58, .72, .77])[None, None] * intensity[..., None]
    color[~np.isfinite(hits['t_hit'].numpy())] = .97
    return color


def draw_bounds(ax, image, bounds, center, extent):
    ax.imshow(image, origin='lower', extent=extent)
    corners = project(bounds['corners_base_m'], center)
    for i, j in EDGES:
        bits = CORNER_BITS[[i,j]]
        hidden = ((bits[:,1] == 1).all() and (bits[:,2] == 0).all()
                  or (bits[:,0] == 0).all() and (bits[:,2] == 0).all()
                  or (bits[:,0] == 0).all() and (bits[:,1] == 1).all())
        ax.plot(corners[[i,j],0], corners[[i,j],1], color='#e36c09',
                lw=1.4, linestyle='--' if hidden else '-', alpha=.75 if hidden else .95)
    middle = corners.mean(axis=0)
    for axis, points in dimension_lines(bounds):
        a, b = project(points, center)
        color = ('#a83e00', '#6d28d9', '#006b42')[axis]
        ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='<->', color=color, lw=1.6, shrinkA=0, shrinkB=0))
        direction = b-a
        normal = np.array([-direction[1], direction[0]])/np.linalg.norm(direction)
        if normal @ ((a+b)/2-middle) < 0:
            normal = -normal
        location = (a+b)/2 + 3.5*normal
        angle = np.rad2deg(np.arctan2(direction[1], direction[0]))
        if angle > 90: angle -= 180
        if angle < -90: angle += 180
        ax.text(*location, f"{('X','Y','H')[axis]} {bounds['dimensions_mm'][axis]:.1f} mm",
                fontsize=10, fontweight='bold', color=color, ha='center', va='center', rotation=angle,
                bbox=dict(boxstyle='round,pad=.22', facecolor='white', edgecolor='none', alpha=.94))
    ax.set_xlim(extent[:2]); ax.set_ylim(extent[2:])
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values(): spine.set_visible(False)


def self_check():
    # An independently specified tilted 40 x 50 x 60 mm cuboid exercises units,
    # support-plane anchoring, axis transforms, and exclusion of the bottom band.
    plane = np.array([.08, -.05, 1., .18]); plane /= np.linalg.norm(plane[:3])
    origin, axes = reference_frame(plane)
    x, y, z = np.meshgrid(np.linspace(-.020,.020,11), np.linspace(-.025,.025,11), np.linspace(0,.060,11))
    local = np.c_[x.ravel(), y.ravel(), z.ravel()]
    world = local @ axes.T + origin
    measurement = measure(world, plane, origin)
    assert np.allclose(measurement['dimensions_mm'], [40.,50.,60.], atol=1e-8)


def main(audit, root):
    self_check()
    prior = json.loads((audit / 'object_summary.json').read_text())
    figures = root/'figures'; figures.mkdir(exist_ok=True)
    cubes = [name for name, (shape, _) in CASES.items() if shape == 'Cube']
    measurements, meshes, centers, projected = {}, {}, {}, []
    for name in cubes:
        source = Path(prior[name]['source'])
        plane = np.array(json.loads((source/'scan_yaml_targets/reconstruct/table_plane.json').read_text())['plane_model'])
        center = np.array(prior[name]['camera_center'])
        center[2] = -(center[:2] @ plane[:2]+plane[3])/plane[2]+.025
        centers[name] = center
        measurements[name] = dict(scan_pattern=CASES[name][1], variants={})
        for variant in VARIANTS:
            path = root/f'runs/{name}/{variant}/session/scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply'
            mesh = o3d.io.read_triangle_mesh(str(path))
            if not len(mesh.triangles): raise ValueError(f'Empty mesh: {path}')
            meshes[name,variant] = mesh
            vertices = np.asarray(mesh.vertices)
            bounds = measure(vertices, plane, center)
            bounds['mesh_path'] = str(path)
            bounds['mesh_sha256'] = digest(path)
            bounds['table_band_sensitivity'] = {str(n): measure(vertices, plane, center, floor=n*VOXEL)['dimensions_mm'] for n in (2,3,4)}
            measurements[name]['variants'][variant] = bounds
            p = project(bounds['corners_base_m'], center)
            projected.append(p)
            projected.extend(project(points, center) for _,points in dimension_lines(bounds))
            lines = o3d.geometry.LineSet()
            lines.points = o3d.utility.Vector3dVector(bounds['corners_base_m'])
            lines.lines = o3d.utility.Vector2iVector(EDGES)
            lines.paint_uniform_color([.89,.42,.035])
            if not o3d.io.write_line_set(str(path.parents[3]/'cube_bounding_box.ply'), lines):
                raise OSError('Could not export bounding box')
    p = np.concatenate(projected)
    xmin,ymin = p.min(axis=0)-14.; xmax,ymax = p.max(axis=0)+14.
    extent = [float(xmin),float(xmax),float(ymin),float(ymax)]
    combined, axes = plt.subplots(3,2,figsize=(12,12))
    for row,name in enumerate(cubes):
        pair, pair_axes = plt.subplots(1,2,figsize=(12,5))
        for col,variant in enumerate(VARIANTS):
            bounds = measurements[name]['variants'][variant]
            image = mesh_image(meshes[name,variant],centers[name],extent)
            for ax in (axes[row,col],pair_axes[col]):
                draw_bounds(ax,image,bounds,centers[name],extent)
                ax.set_title(TITLES[col],fontsize=12,pad=10)
            if col == 0:
                axes[row,col].set_ylabel(CASES[name][1],fontsize=13,labelpad=12)
            individual,ax=plt.subplots(figsize=(7,6))
            draw_bounds(ax,image,bounds,centers[name],extent)
            ax.set_title(f'{CASES[name][1]} · {TITLES[col]}',pad=12)
            individual.text(.5,.02,'Estimated mesh bounds · X / Y / height in mm · threshold 3',ha='center',fontsize=10)
            individual.tight_layout(rect=[0,.04,1,1]);individual.savefig(figures/f'{name}_{variant}_dimensions.png',dpi=180);plt.close(individual)
        pair.suptitle(f'Cube / {CASES[name][1]} · {name} · threshold 3 in both',fontsize=14)
        pair.text(.5,.01,'Same scale and view. Table-aligned mesh bounds include reconstruction artefacts; height is measured from the saved table plane.',ha='center',fontsize=9)
        pair.tight_layout(rect=[0,.04,1,.95]);pair.savefig(figures/f'{name}_dimensions_pair.png',dpi=180);plt.close(pair)
    combined.suptitle('November 13 cube scans · distortion correction OFF / ON\nThreshold 3 in both · identical view and scale',fontsize=15,y=.995)
    combined.text(.5,.01,'X / Y = table-aligned extents; H = height above the shared support plane.\nTable band excluded below 5.9 mm. Boxes include cube artefacts; these are reconstructed dimensions.',ha='center',fontsize=10)
    combined.tight_layout(rect=[0,.045,1,.96]);combined.savefig(figures/'cube_dimensions_comparison.png',dpi=170);plt.close(combined)
    # Paired images for every non-cube scan complete the requested folder comparison.
    for name,(shape,pattern) in CASES.items():
        if shape=='Cube': continue
        fig,axs=plt.subplots(1,2,figsize=(10,4.5))
        for ax,variant,title in zip(axs,VARIANTS,TITLES):
            path=root/f'runs/{name}/{variant}/session/scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply'
            mesh=o3d.io.read_triangle_mesh(str(path))
            ax.imshow(render(mesh,prior[name]['camera_center'],width=.12,height=.09),origin='lower');ax.set_title(title);ax.axis('off')
        fig.suptitle(f'{shape} / {pattern} · {name} · threshold 3 in both')
        fig.tight_layout();fig.savefig(figures/f'{name}_pair.png',dpi=160);plt.close(fig)
    result = dict(source_drive='https://drive.google.com/drive/folders/16YXI5viCtLaFw5RlyM0jDAW7WdZNkFMb',
                  units='millimetres', method='Table-aligned X/Y bounding extents of all cube-region mesh vertices above three voxels (5.859375 mm); bottom anchored to the shared saved table plane. Includes cube artefacts. No nominal dimensions, smoothing, or cuboid fitting.',
                  table_band_sensitivity_units='millimetres; keys are voxel multiples',
                  common_projection_extent_mm=extent, dimensional_self_check_passed=True, cases=measurements)
    write_json(root/'cube_dimensions.json',result)
    for name,case in measurements.items():
        print(name,{v:np.round(b['dimensions_mm'],1).tolist() for v,b in case['variants'].items()},flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root',type=Path,required=True)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    main(args.audit_root,args.root)
