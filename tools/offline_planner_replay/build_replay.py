"""Audit archived candidate replay and bounded fixed-goal inward-lean proposals."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import sys

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/offline_planner_replay/recorded_260728')
    parser.add_argument('--dependency-path', type=Path, help='Optional directory containing numerical dependencies')
    args = parser.parse_args()
    if args.dependency_path:
        sys.path.insert(0, str(args.dependency_path.resolve()))
    import numpy as np
    import open3d as o3d
    import yaml
    from replay import (PROFILE, BOUNDS, field_identity, point_comparison, generate, historical_axes,
                        current_targets, contour_from_mesh, local_goal_candidates)
    sys.path.insert(0, str(ROOT/'tools/offline_recovery'))
    from build_inward_lean import deform, LEAN, SETTINGS
    from geometry import adapt_targets
    from viability import Scene, tool_models, evaluate_candidates, COLLISION_SOURCE, TOOL_MESH, WORKCELL

    output = args.output.resolve()
    recording = args.recording.resolve()
    if output == recording or recording in output.parents:
        raise ValueError('output must not modify the recording')
    output.mkdir(parents=True, exist_ok=True)
    paths = [*Path(__file__).parent.glob('*.py'), Path(__file__).with_name('viewer.html'),
             * (ROOT/'src/behav3d_py/behav3d_py/scalar_field/lib_scalar').glob('*.py'),
             ROOT/'src/behav3d_sense/behav3d_sense/fields_node.py', COLLISION_SOURCE, TOOL_MESH, WORKCELL,
             * (ROOT/'tools/offline_recovery').glob('*.py'),
             ROOT/'tools/offline_inspection/local_scene.py', ROOT/'tools/offline_inspection/visibility.py']
    source_hashes = {}

    def track(path):
        path = Path(path).resolve()
        source_hashes[str(path)] = sha(path)
        return path

    for path in paths:
        track(path)
    field_path = track(ROOT/'mesh/fields/field_state_init1.npz')
    with np.load(field_path, allow_pickle=False) as state:
        field = {'vertices': state['field_vertices_world'], 'faces': state['field_faces'],
                 'heat': state['heat_norm'], 'offset': state['offset_xyz']}
    if not all(np.isfinite(value).all() for value in field.values()):
        raise ValueError('nonfinite numerical field input')
    snapshots = {f'field_{key}': value for key, value in field.items()}
    baseline, field_audit = [], []
    first_cloud = o3d.io.read_point_cloud(str(track(recording/'field_loop/cycle_0001/candidates/field_masked_cycle.ply')))
    for name in ['field_state_init.npz', 'field_state_init1.npz', 'field_state_init2.npz']:
        with np.load(track(ROOT/'mesh/fields'/name), allow_pickle=False) as state:
            result = field_identity(state['field_vertices_world'], state['heat_norm'],
                                    np.asarray(first_cloud.points), np.asarray(first_cloud.colors))
            field_audit.append({'file': name, 'vertex_count': len(state['field_vertices_world']), **result})
    with np.load(track(ROOT/'mesh/fields/width_field.npz'), allow_pickle=False) as widths:
        width_count = len(widths['width_norm'])

    for cycle in range(1, 24):
        folder = recording/f'field_loop/cycle_{cycle:04d}'
        capture = yaml.safe_load(track(folder/'scan/manifest.yaml').read_text())
        if not capture['captures'] or any(c['T_base_tool0']['parent'] != 'ur20_base_link' for c in capture['captures']):
            raise ValueError('recorded frame mismatch')
        saved_field = o3d.io.read_point_cloud(str(track(folder/'candidates/field_masked_cycle.ply')))
        identity = field_identity(field['vertices'], field['heat'], np.asarray(saved_field.points), np.asarray(saved_field.colors))
        if not identity['ordered_vertices_exact'] or not identity['heat_within_color_rounding']:
            raise ValueError(f'field identity mismatch at cycle {cycle}')
        saved = o3d.io.read_line_set(str(track(folder/'candidates/field_phi0_contour_cycle.ply')))
        contour, lines = np.asarray(saved.points), np.asarray(saved.lines)
        archived = np.asarray(o3d.io.read_point_cloud(str(track(folder/'candidates/field_print_candidates_cycle.ply'))).points)
        rows = yaml.safe_load(track(folder/'candidates/targets.yaml').read_text())['targets']
        axes = np.array([np.fromstring(re.search(r'Z\(([^)]+)\)', row['plane'])[1], sep=',') for row in rows]) * [-1, -1, 1]
        origins = np.array([np.fromstring(re.search(r'O\(([^)]+)\)', row['plane'])[1], sep=',') for row in rows]) * [-.001, -.001, .001]
        if origins.shape != archived.shape or np.max(np.abs(origins - archived)) > .00000501:
            raise ValueError('YAML frame/units do not reproduce PLY within per-component rounding')
        points = generate(contour, lines, field)
        comparison = point_comparison(points.points, archived)
        predicted_axes = historical_axes(points, field)
        axis_error = float(np.abs(predicted_axes - axes).max()) if axes.shape == predicted_axes.shape else None
        targets, stats = current_targets(points, field)
        baseline.append({'cycle': cycle, 'field_identity': identity, 'saved_contour_replay': comparison,
                         'axis_component_max_error': axis_error,
                         'matches_archived_within_1e_5_mm': bool(comparison['ordered_max_mm'] is not None and comparison['ordered_max_mm'] < 1e-5
                                                                and axis_error is not None and axis_error <= .0050001),
                         'current_rules_count': targets.count, 'current_rules_stats': {k: v.tolist() if hasattr(v, 'tolist') else v for k,v in stats.items()}})
        for key, value in {'contour': contour, 'lines': lines, 'archived': archived, 'axes': axes,
                           'viable': np.asarray(saved_field.colors)[:, 0] > .9}.items():
            snapshots[f'c{cycle}_{key}'] = value
    print('Saved-contour replay:', sum(r['matches_archived_within_1e_5_mm'] for r in baseline), '/23 batches', flush=True)

    upstream, meshes = [], {}
    for cycle, name in [(6, 'recorded_cycle6'), (19, 'inward_cycle19')]:
        folder = ROOT/'outputs/offline_recovery'/name
        manifest = json.loads(track(folder/'manifest.json').read_text())
        mesh_path = track(folder/'recorded_mesh.npz')
        if sha(mesh_path) != manifest['files_sha256']['recorded_mesh.npz']:
            raise ValueError('stored mesh hash mismatch')
        with np.load(mesh_path, allow_pickle=False) as mesh:
            vertices, faces = mesh['vertices_m'], mesh['faces']
        meshes[cycle] = (vertices, faces)
        contour, lines, pose = contour_from_mesh(vertices, faces, field)
        points = generate(contour, lines, field)
        upstream.append({'cycle': cycle, 'contour_comparison': point_comparison(contour, snapshots[f'c{cycle}_contour']),
                         'viability_mask_mismatches': int(np.sum(pose.viable != snapshots[f'c{cycle}_viable'])),
                         'candidate_comparison': point_comparison(points.points, snapshots[f'c{cycle}_archived']),
                         'status': 'upstream_drift_unresolved'})

    old = json.loads(track(ROOT/'outputs/offline_recovery/inward_cycle19/results.json').read_text())
    archived = old['trials'][0]['plans'][0]['targets']
    original = next(r for r in archived if r['label'] == 'C19:T7')
    peers = [r for r in archived if r['label'] != 'C19:T7']
    decisions = json.loads(track(ROOT/'outputs/offline_recovery/decisions/results.json').read_text())
    lib, models, tool_meta = tool_models()
    # Keep old support/tool proxy settings for a fair comparison; separately report
    # the stricter current endpoint-distance rule inferred at 36 mm.
    trials, scenes = [], {}
    vertices, faces = meshes[19]
    for angle in [0, 4, 8, 12]:
        print(f'Testing fixed goal at {angle} degrees inward lean', flush=True)
        moved = deform(vertices, angle)
        scene = Scene(moved, faces)
        contour, lines, _ = contour_from_mesh(moved, faces, field)
        ordinary = generate(contour, lines, field)
        ordinary_targets, ordinary_stats = current_targets(ordinary, field)
        ordinary_records = [dict(original, label=f'ordinary-{i+1}', top_m=p.tolist(), axis=a.tolist())
                            for i, (p, a) in enumerate(zip(ordinary_targets.end_points, ordinary_targets.end_z_dirs))]
        seed = adapt_targets([original], -angle, LEAN)[0]
        candidates, counts, axis_ablation = local_goal_candidates(contour, lines, field, seed)
        unchanged = evaluate_candidates([original, *peers], scene, lib, models, SETTINGS)[0]

        def evaluate(record):
            check = evaluate_candidates([record, *peers], scene, lib, models, SETTINGS)[0]
            check['field_goal_validity'] = 'generated_from_matched_field_under_inferred_profile'
            spacing = check['minimum_batch_spacing_mm']
            return check, spacing is None or spacing >= PROFILE['min_spacing_m'] * 1000

        for candidate in [*candidates, *axis_ablation]:
            check, spacing_pass = evaluate(candidate['target'])
            candidate.update(check=check, current_endpoint_spacing_pass=spacing_pass,
                             promising=check['geometric_screen'] == 'passes_declared_screens' and spacing_pass,
                             displacement_from_archived_mm=float(np.linalg.norm(np.asarray(candidate['target']['top_m']) - original['top_m']) * 1000))
        eligible = [c for c in candidates if c['promising']]
        selected = min(eligible, key=lambda c: (-c['check']['support_rays_within_gap'], c['displacement_from_archived_mm'], c['id']), default=None)
        old_case = next(c for c in decisions['cases'] if c['id'] == f'lean-{angle}-stable')
        old_selected = next((p for p in old_case['proposals'] if p['id'] == old_case['decision']['selected_proposal_id']), None)
        old_comparison = None
        if old_selected:
            old_check = evaluate_candidates([old_selected['targets'][0], *peers], scene, lib, models, SETTINGS)[0]
            old_comparison = {'id': old_selected['id'], 'target': old_selected['targets'][0], 'check': old_check,
                              'goal_status': 'local_surface_proposal_not_generated_by_goal_planner', 'executable': False}
        trial = {'angle_deg': angle, 'ordinary_raw_count': len(ordinary.points), 'ordinary_current_count': ordinary_targets.count,
                 'ordinary_targets': ordinary_records, 'local_search': counts, 'candidates': candidates,
                 'axis_limit_ablation': {'scope': 'diagnostic only: keep XY/Z bounds and production 30-degree cone; omit extra 4-degree seed-relative bound',
                                        'candidates': axis_ablation, 'passes_other_screens': sum(c['promising'] for c in axis_ablation)},
                 'selected_id': None if selected is None else selected['id'], 'promising_count': len(eligible),
                 'archived_affected_target': original, 'archived_affected_check': unchanged, 'previous_local_proposal': old_comparison,
                 'decision': 'use_normal_planner' if angle == 0 else ('proposal_requires_further_validation' if selected else 'no_bounded_fixed_goal_candidate_passed'),
                 'executable': False, 'robot_commands': []}
        trials.append(trial)
        display = moved[(moved[:, 0] > 0) & (moved[:, 0] < .34) & (moved[:, 1] > -.99)
                        & (moved[:, 1] < -.78) & (moved[:, 2] > -.17)]
        if len(display) > 9000:
            display = display[np.linspace(0, len(display)-1, 9000, dtype=int)]
        scenes[str(angle)] = {'points': display.tolist(), 'contour': contour.tolist()}
        print(f"  {counts['within_local_bounds']} bounded alternatives, {len(eligible)} pass declared screens", flush=True)

    results = {'schema': 'behav3d.fixed_goal_replay.v1', 'artifact_id': 'recording-260728_133013',
               'frame': 'ur20_base_link', 'units': 'm', 'field_sha256': sha(field_path),
               'profile': PROFILE, 'field_audit': field_audit, 'baseline': baseline, 'upstream': upstream,
               'width_map': {'vertex_count': width_count, 'matches_field_vertex_count': width_count == len(field['vertices']),
                             'used': False, 'reason': 'repository width map has a different vertex count; no recorded width map recovered'},
               'lean_parameters': LEAN, 'search_bounds': BOUNDS, 'screen_settings': SETTINGS,
               'tool_model': tool_meta, 'trials': trials,
               'history': {'pre_recording_source': '8fa8e6f', 'later_secondary_rule_source': '6214cdb',
                           'interpretation': 'raw candidate stage matches recording; current endpoint filters are a separate experiment'},
               'limitations': ['Replay settings are inferred, not logged runtime settings.',
                   'Matching field vertices and scalar colors do not independently verify archived field topology.',
                   'Mesh-to-contour drift remains unresolved; fixed-goal trials are conditional on this implementation.',
                   '36 x 16 mm bead proxy dimensions remain borrowed; 16 mm planner lift is not bead-height calibration.',
                   'The goal is a surface field; its ray/contour construction is not a volumetric tolerance certificate.',
                   'Avoidance state, recorded tool calibration, IK, mechanics and future deposition are not validated.',
                   'Lean, stable trend and intact connections are controlled assumptions, not Agent007 detections.',
                   'Preserve the full frontier sweep before requesting the next normal batch.'],
               'full_planner_state_complete': False, 'source_sha256': source_hashes,
               'runtime': {'python': platform.python_version(), **{n: importlib.metadata.version(n) for n in ['numpy', 'scipy', 'open3d', 'potpourri3d']}},
               'executable': False, 'robot_commands': []}
    if any(sha(name) != value for name, value in source_hashes.items()):
        raise ValueError('source changed during replay')
    np.savez_compressed(output/'baseline_fixture.npz', **snapshots)
    dump(output/'results.json', results)
    data = {'results': results, 'scenes': scenes}
    html = Path(__file__).with_name('viewer.html').read_text().replace('/*DATA*/', json.dumps(data, allow_nan=False).replace('<', '\\u003c'))
    (output/'viewer.html').write_text(html)
    dump(output/'manifest.json', {'files_sha256': {name: sha(output/name) for name in ['results.json', 'baseline_fixture.npz', 'viewer.html']},
                                 'source_hashes_unchanged': True, 'executable': False})
    print('Wrote', output, flush=True)


if __name__ == '__main__':
    main()
