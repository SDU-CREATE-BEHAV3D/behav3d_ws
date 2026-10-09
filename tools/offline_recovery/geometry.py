"""Numerical recovery experiments; no ROS or execution interfaces."""
from pathlib import Path
import re
import sys

import numpy as np
import yaml

INSPECTION = Path(__file__).resolve().parents[1] / 'offline_inspection'
sys.path.insert(0, str(INSPECTION))
from local_scene import NominalBead
from visibility import MeshRays


def bead(record):
    return NominalBead(record['label'], tuple(record['top_m']), tuple(record['axis']),
                       record['width_m'], record['height_m'])


def issued_targets(path, cycle, width, height, *, flip_x, flip_y):
    if type(flip_x) is not bool or type(flip_y) is not bool:
        raise ValueError('explicit recorded sign convention required')
    signs = np.array([-1 if flip_x else 1, -1 if flip_y else 1, 1])
    result = []
    for row in yaml.safe_load(Path(path).read_text())['targets']:
        origin = re.search(r'O\(([^)]+)\)', row['plane'])
        axis = re.search(r'Z\(([^)]+)\)', row['plane'])
        if origin is None or axis is None:
            raise ValueError('target plane missing')
        top = np.fromstring(origin[1], sep=',') * .001 * signs
        direction = np.fromstring(axis[1], sep=',') * signs
        item = NominalBead(f'C{cycle}:T{row["index"] + 1}', tuple(top),
                           tuple(direction), width, height).record()
        result.append(item)
    if len({r['label'] for r in result}) != len(result):
        raise ValueError('duplicate target')
    return result


def ply_points(path):
    """Read the saved Open3D binary point/line files without importing Open3D."""
    raw = Path(path).read_bytes()
    header, body = raw.split(b'end_header\n', 1)
    lines = header.decode('ascii').splitlines()
    if 'format binary_little_endian 1.0' not in lines:
        raise ValueError('expected binary little-endian PLY')
    types = {'double': '<f8', 'float': '<f4', 'uchar': 'u1', 'int': '<i4'}
    props, count, active = [], None, False
    for line in lines:
        words = line.split()
        if words[:1] == ['element']:
            active = words[1] == 'vertex'
            if active:
                count = int(words[2])
        elif words[:1] == ['property'] and active:
            props.append((words[2], types[words[1]]))
    vertices = np.frombuffer(body, dtype=np.dtype(props), count=count)
    return np.column_stack([vertices[k] for k in ('x', 'y', 'z')])


def centers(indices, domain):
    indices = np.asarray(indices, dtype=float).reshape(-1, 3)
    return np.asarray(domain['origin_m']) + (indices + .5) * np.asarray(domain['voxel_size_m'])


def split_distance_evidence(distances_mm, tolerance_mm, saved_missing_count):
    """Preserve uncertainty from DDS's distance export rounded to 0.001 mm."""
    distances = np.asarray(distances_mm, dtype=float)
    if not np.isfinite(distances).all():
        raise ValueError('finite exported distances required')
    missing = distances > tolerance_mm + .0005
    boundary = np.abs(distances-tolerance_mm) <= .0005
    if not int(missing.sum()) <= saved_missing_count <= int(missing.sum()+boundary.sum()):
        raise ValueError('distance export cannot reproduce saved count even with rounding uncertainty')
    return missing, boundary


def smoothstep(value):
    x = np.clip(value, 0, 1)
    return x*x*(3-2*x)


def lean(points, degrees, config):
    """Anchored smooth shear of a measured region; a controlled geometric edit.

    Full-weight vertical lines acquire the specified lean toward +X. This is not
    rigid-body motion, a deformation mechanics model, or a simulated observation.
    """
    p = np.asarray(points, dtype=float)
    if p.shape[-1] != 3 or not np.isfinite(p).all() or not np.isfinite(degrees) or abs(degrees) > 20:
        raise ValueError('finite geometry and bounded lean required')
    if not config['x_full_m'] > config['x_start_m'] or not config['y_outer_m'] > config['y_inner_m']:
        raise ValueError('invalid transition bounds')
    wx = smoothstep((p[..., 0]-config['x_start_m'])/(config['x_full_m']-config['x_start_m']))
    wy = 1-smoothstep((np.abs(p[..., 1]-config['y_center_m'])-config['y_inner_m']) /
                     (config['y_outer_m']-config['y_inner_m']))
    out = p.copy()
    out[..., 0] += wx*wy*np.maximum(p[..., 2]-config['pivot_z_m'], 0)*np.tan(np.radians(degrees))
    return out


def adapt_targets(records, degrees, config):
    result = []
    for record in records:
        b = bead(record)
        base = np.asarray(b.top)-b.height_m*b.direction
        moved_base, moved_top = lean(np.array([base, b.top]), degrees, config)
        axis = moved_top-moved_base
        axis /= np.linalg.norm(axis)
        # Keep the commanded bead dimensions; attach its base to the transformed
        # archived base. This is a proposal, not a regenerated goal-field plan.
        top = moved_base + b.height_m*axis
        item = NominalBead(b.label, tuple(top), tuple(axis), b.width_m, b.height_m).record()
        item['proposal_basis'] = 'base attached to controlled deformation; dimensions preserved'
        result.append(item)
    return result


def union_contains(points, records, tolerance=0):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    inside = np.zeros(len(points), dtype=bool)
    for record in records:
        inside |= bead(record).sdf(points) <= tolerance
    return inside


def repair_probe(missing_points, original):
    """One unchanged-size bead centered on the recorded discrepancy samples."""
    if len(missing_points) == 0:
        raise ValueError('missing evidence required')
    b = bead(original)
    top = np.mean(missing_points, axis=0) + b.height_m/2*b.direction
    item = NominalBead('repair-probe', tuple(top), tuple(b.direction), b.width_m, b.height_m).record()
    item['proposal_basis'] = 'recorded missing-reference centroid; issued dimensions and axis'
    return item


def footprint_gaps(records, triangles):
    """Ray diagnostics from the whole mesh. No robot or field validity claim."""
    rays = MeshRays(triangles)
    start_z = float(triangles[..., 2].max()+.1)
    result = []
    for record in records:
        b = bead(record)
        base = np.asarray(b.top)-b.height_m*b.direction
        hit = rays.first_hit([base[0], base[1], start_z], [0, 0, -1])
        result.append({'label': b.label, 'base_m': base.tolist(),
                       'base_to_upper_surface_mm': None if hit is None else float((base[2]-(start_z-hit))*1000),
                       'ray_hit': hit is not None})
    return result


def priority_order(records, missing_points):
    """Reorder only the archived batch by direct nominal reach of the evidence."""
    centroid = np.mean(missing_points, axis=0)
    return sorted(records, key=lambda r: (-int(union_contains(missing_points, [r]).sum()),
                                          float(np.linalg.norm(bead(r).center-centroid)), r['label']))


def probe_metrics(records, missing, observed, intended):
    """Reference-sample diagnostics, not material volume or print success."""
    outside = total = 0
    for record in records:
        b = bead(record)
        half = b.width_m/2 + b.height_m/2
        axis = np.arange(-half, half+.0001, .002)
        xyz = np.stack(np.meshgrid(axis, axis, axis, indexing='ij'), axis=-1).reshape(-1, 3)+b.center
        solid = xyz[b.sdf(xyz) <= 0]
        total += len(solid)
        outside += int((~union_contains(solid, intended)).sum())
    return {'candidate_count': len(records),
            'missing_reference_samples_inside_proposed_solids': int(union_contains(missing, records).sum()),
            'missing_reference_sample_count': len(missing),
            'observed_reference_samples_inside_proposed_solids': int(union_contains(observed, records).sum()),
            'sampled_solid_outside_issued_cycle5_fraction': None if not total else outside/total,
            'outside_basis': 'outside prior-cycle issued beads, NOT outside final product goal',
            'sample_basis': 'DDS surface-band reference samples, NOT missing material volume'}


def evidence_decision(*, identity_supported, tolerance_sensitive, observation_fraction, minimum=.8):
    if not identity_supported or observation_fraction < minimum:
        return 'review_evidence_before_recovery'
    if tolerance_sensitive:
        return 'compare_recovery_hypotheses_keep_inspection_open'
    return 'compare_recovery_hypotheses'
