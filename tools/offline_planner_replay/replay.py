"""Numerical planner replay. Imports no ROS nodes or robot clients."""
from pathlib import Path
import sys

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/behav3d_py/behav3d_py/scalar_field'))
from lib_scalar.compute_phi_mask import make_scan_scene, compute_phi_mask
from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.generate_print_points import generate_print_points
from lib_scalar.geometry import sample_tangent_axes_on_surface_from_scalar, clamp_vectors_to_cone
from lib_scalar.print_targets import build_candidate_segment_targets
from lib_scalar.target_rules import apply_secondary_target_rules

# Inferred by fitting saved contours/candidates; not a recovered ROS parameter log.
PROFILE = {'count': 7, 'min_spacing_m': .036, 'lift_height_m': .016,
           'candidate_mode': 'gradient_lift', 'tangent_sign': 1., 'cone_deg': 30.,
           'clearance_m': 0., 'iso_level_m': 0., 'status': 'inferred_replay_profile'}
BOUNDS = {'xy_radius_mm': 8., 'z_change_mm': 12., 'axis_change_deg': 4.}


def point_comparison(actual, reference):
    """Keep ordered error distinct from unordered nearest-neighbor agreement."""
    actual, reference = np.asarray(actual), np.asarray(reference)
    equal = len(actual) == len(reference)
    if not len(actual) or not len(reference):
        return {'actual_count': len(actual), 'reference_count': len(reference),
                'ordered_max_mm': 0. if equal else None,
                'symmetric_nearest_max_mm': 0. if equal else None}
    ordered = np.linalg.norm(actual - reference, axis=1) * 1000 if equal else None
    forward = cKDTree(reference).query(actual)[0] * 1000
    backward = cKDTree(actual).query(reference)[0] * 1000
    return {'actual_count': len(actual), 'reference_count': len(reference),
            'ordered_max_mm': None if ordered is None else float(ordered.max()),
            'symmetric_nearest_max_mm': float(max(forward.max(), backward.max()))}


def field_identity(vertices, heat, recorded_vertices, recorded_colors):
    """Colors verify scalar values only where the export marks viable vertices."""
    same = vertices.shape == recorded_vertices.shape and np.array_equal(vertices, recorded_vertices)
    if not same:
        return {'ordered_vertices_exact': False, 'heat_within_color_rounding': False,
                'compared_heat_vertices': 0, 'maximum_heat_color_error': None}
    visible = recorded_colors[:, 0] > .9
    errors = np.abs((1 - recorded_colors[visible, 1]) - heat[visible])
    return {'ordered_vertices_exact': True, 'heat_within_color_rounding': bool(len(errors) and np.all(errors <= 1/510 + 1e-12)),
            'compared_heat_vertices': int(visible.sum()),
            'maximum_heat_color_error': float(errors.max()) if len(errors) else None}


def generate(contour, lines, field, *, mask=None, count=None):
    return generate_print_points(contour, lines, field['vertices'], field['heat'],
        count=PROFILE['count'] if count is None else count,
        min_spacing=PROFILE['min_spacing_m'], point_valid_mask=mask,
        candidate_mode=PROFILE['candidate_mode'], lift_height=PROFILE['lift_height_m'],
        field_faces=field['faces'], walk_tangent_sign=PROFILE['tangent_sign'])


def historical_axes(points, field):
    axes = sample_tangent_axes_on_surface_from_scalar(query_points=points.surface_points,
        mesh_vertices=field['vertices'], mesh_faces=field['faces'], vertex_scalar=field['heat'])[0]
    return clamp_vectors_to_cone(axes, max_tilt_deg=PROFILE['cone_deg'])


def current_targets(points, field):
    targets = build_candidate_segment_targets(candidate_points=points.points,
        segment_start_points=points.segment_start_points, candidate_mode=PROFILE['candidate_mode'],
        field_vertices_world=field['vertices'], field_faces=field['faces'], field_scalar=field['heat'],
        tangent_sign=PROFILE['tangent_sign'], clamp_to_cone=True, cone_max_tilt_deg=PROFILE['cone_deg'],
        orient_with_tangent=True, fixed_z_dir=(0., 0., 1.))
    return apply_secondary_target_rules(targets, candidate_mode=PROFILE['candidate_mode'],
        bead_height_m=PROFILE['lift_height_m'], bead_separation_m=PROFILE['min_spacing_m'],
        normal_continuity_rule=True)


def contour_from_mesh(vertices, faces, field):
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    scene, z_top = make_scan_scene(mesh)
    state = compute_phi_mask(scene, z_top, field['vertices'], tuple(field['offset']),
                             PROFILE['clearance_m'], PROFILE['iso_level_m'])
    points, lines = extract_phi_contour(field['vertices'], field['faces'], state.phi)
    return points, lines, state


def inside_bounds(record, seed):
    delta = np.asarray(record['top_m']) - seed['top_m']
    a, b = np.asarray(record['axis']), np.asarray(seed['axis'])
    angle = float(np.degrees(np.arccos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))
    metrics = {'xy_change_mm': float(np.linalg.norm(delta[:2]) * 1000),
               'z_change_mm': float(abs(delta[2]) * 1000), 'axis_change_deg': angle}
    return (metrics['xy_change_mm'] <= BOUNDS['xy_radius_mm'] + 1e-8
            and metrics['z_change_mm'] <= BOUNDS['z_change_mm'] + 1e-8
            and angle <= BOUNDS['axis_change_deg'] + 1e-8), metrics


def local_goal_candidates(contour, lines, field, seed):
    """Enumerate existing contour vertices, then apply unchanged production math.

    The agent changes the selection region, not the field or candidate direction.
    Candidates outside the previously declared local recovery bounds are excluded.
    Alternatives are single-bead choices, never a jointly issued batch.
    """
    roi = ((contour[:, 0] > .19) & (np.abs(contour[:, 1] + .875) < .095) & (contour[:, 2] > -.075))
    output, axis_ablation, seen, generated_count = [], [], set(), 0
    excluded = {'xy': 0, 'z': 0, 'axis': 0}
    for index in np.flatnonzero(roi):
        mask = np.zeros(len(contour), dtype=bool)
        mask[index] = True
        points = generate(contour, lines, field, mask=mask, count=1)
        if not len(points.points):
            continue
        targets, _ = current_targets(points, field)
        key = tuple(np.round(np.r_[targets.end_points[0], targets.end_z_dirs[0]], 10))
        if key in seen:
            continue
        seen.add(key)
        generated_count += 1
        record = dict(seed, label=f'goal-contour-{index}', top_m=targets.end_points[0].tolist(),
                      axis=targets.end_z_dirs[0].tolist())
        eligible, limits = inside_bounds(record, seed)
        failures = {'xy': limits['xy_change_mm'] > BOUNDS['xy_radius_mm'] + 1e-8,
                    'z': limits['z_change_mm'] > BOUNDS['z_change_mm'] + 1e-8,
                    'axis': limits['axis_change_deg'] > BOUNDS['axis_change_deg'] + 1e-8}
        for name, fail in failures.items():
            excluded[name] += int(fail)
        proposal = {'id': record['label'], 'target': record, 'source_m': points.source_points[0].tolist(),
                    'search_bounds': limits, 'executable': False}
        if eligible:
            output.append(proposal)
        elif not failures['xy'] and not failures['z']:
            # Diagnostic only: preserve production's cone but remove the extra
            # local 4-degree constraint. Never mix these with strict eligibility.
            axis_ablation.append(proposal)
    return output, {'regional_contour_vertices': int(roi.sum()), 'unique_generated_targets': generated_count,
                    'within_local_bounds': len(output), 'outside_local_bounds': generated_count - len(output),
                    'bounds_failure_counts_nonexclusive': excluded}, axis_ablation
