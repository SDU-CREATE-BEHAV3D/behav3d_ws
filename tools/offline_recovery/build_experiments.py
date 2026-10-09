"""Two reproducible, non-executable experiments on recorded cycle-6 geometry."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

from geometry import (adapt_targets, bead, centers, evidence_decision, footprint_gaps,
                      issued_targets, lean, ply_points, priority_order, probe_metrics,
                      repair_probe, split_distance_evidence)
from build_grid_supplement import validate_assessment
from build_frontier_scan import load_bundle
from visibility import load_binary_stl

ROOT = Path(__file__).resolve().parents[2]
LEAN = {'x_start_m': .19, 'x_full_m': .23, 'y_center_m': -.87,
        'y_inner_m': .065, 'y_outer_m': .095, 'pivot_z_m': -.175}
ANGLES = [0, 4, 8, 12]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def snapshot_sources(paths):
    return {str(Path(p).resolve()): digest(p) for p in sorted(set(paths), key=str)}


def strategy(sid, name, records, triangles, *, basis, inspection):
    return {'id': sid, 'name': name, 'targets': deepcopy(records), 'basis': basis,
            'upper_surface_diagnostics': footprint_gaps(records, triangles),
            'inspection_criterion': inspection,
            'field_goal_validity': 'not_replayed', 'avoidance_validity': 'not_replayed',
            'robot_feasibility': 'unverified', 'mechanical_effect': 'unvalidated',
            'executable': False, 'robot_commands': []}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local', type=Path, default=ROOT/'outputs/offline_inspection/frontier_batch/c5-t7/local_scene.json')
    parser.add_argument('--agent-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/offline_recovery/recorded_cycle6')
    args = parser.parse_args()
    local = json.loads(args.local.read_text())
    validate_assessment(local)
    if (local['label'], local['frame'], local['units'], local['scan_cycle']) != ('C5:T7', 'ur20_base_link', 'm', 6):
        raise ValueError('this protocol is bound to C5:T7 in recorded scan 6')
    bundle = args.agent_root/'outputs/integrated_evaluation_260728_133013_final/cycle_0006/evaluation'
    packet, evaluation = load_bundle(bundle)
    review_path = next(Path(p) for p in local['sources'] if p.endswith('/cycle_0006/review.json'))
    if evaluation['source_sha256'].get(str(review_path)) != local['sources'][str(review_path)]:
        raise ValueError('Agent007/local revision mismatch')
    review = json.loads(review_path.read_text())
    candidate = next(c for c in review['assessed_candidates'] if c['label'] == local['label'])
    evaluated = next(c for c in evaluation['candidates'] if c['label'] == local['label'])
    metrics_path = Path(candidate['source_metrics'])
    metrics = json.loads(metrics_path.read_text())
    overview_path = Path(metrics['target_voxel_overview_data_json_path'])
    overview = json.loads(overview_path.read_text())
    target = next(t for t in overview['targets'] if t['target_number'] == 7)
    ti = overview['default_tolerance_index']
    if target['summaries'][ti] != candidate['evidence'] or not np.allclose(target['position_m'], local['target']['top_m']):
        raise ValueError('voxel display export does not reproduce Agent007 evidence')
    if overview['tolerances_mm'][ti] != metrics['defect_tolerance_m']*1000:
        raise ValueError('DDS tolerance mismatch')
    expected = centers(target['expected_core_indices'], overview['domain'])
    distances = np.asarray(target['expected_core_distance_mm'])
    missing_mask, boundary_mask = split_distance_evidence(distances, overview['tolerances_mm'][ti],
                                                          candidate['evidence']['missing_voxels'])
    missing = expected[missing_mask]
    boundary = expected[boundary_mask]
    observed = centers(target['observed_core_indices'], overview['domain'])
    unknown = centers(target['expected_unknown_indices'], overview['domain'])
    # The plotting export can be capped. Require exact retained counts, including
    # saved counts to be consistent with export rounding before using coordinates.
    if len(expected) != candidate['evidence']['expected_within_tolerance_voxels']+candidate['evidence']['missing_voxels']:
        raise ValueError('truncated expected sample export')
    if len(observed) != candidate['evidence']['observed_within_tolerance_voxels']+candidate['evidence']['extra_voxels']:
        raise ValueError('truncated observed sample export')
    for tolerance, summary in zip(overview['tolerances_mm'], target['summaries']):
        split_distance_evidence(distances, tolerance, summary['missing_voxels'])
    mesh_path = next(Path(p) for p in local['sources'] if p.endswith('/tsdf_surface_mesh.stl'))
    scan = mesh_path.parents[1]
    manifest_path = scan/'manifest.yaml'
    manifest = yaml.safe_load(manifest_path.read_text())
    if any(c['T_base_tool0']['parent'] != local['frame'] for c in manifest['captures']):
        raise ValueError('recorded camera frame mismatch')
    next_dir = scan.parent/'candidates'
    next_path = next_dir/'targets.yaml'
    next_ply = next_dir/'field_print_candidates_cycle.ply'
    next_contour = next_dir/'field_phi0_contour_cycle.ply'
    next_field = next_dir/'field_masked_cycle.ply'
    baseline = issued_targets(next_path, 6, local['target']['width_m'], local['target']['height_m'],
                              flip_x=metrics['flip_x'], flip_y=metrics['flip_y'])
    if not np.allclose(np.array([r['top_m'] for r in baseline]), ply_points(next_ply), rtol=0, atol=.00001):
        raise ValueError('issued next batch sign/position does not match saved base-frame point cloud')
    intended = [local['target'], *local['neighbors']]
    source_paths = [args.local, review_path, metrics_path, overview_path, mesh_path, manifest_path,
                    next_path, next_ply, next_contour, next_field, *bundle.glob('*.json')]
    source_paths += [Path(p) for p in local['sources']]
    source_paths += list(Path(__file__).parent.glob('*.py'))+[Path(__file__).with_name('viewer.html')]
    sources = snapshot_sources(source_paths)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    triangles = load_binary_stl(mesh_path)
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3).astype(np.int32)
    np.savez_compressed(output/'recorded_mesh.npz', vertices_m=vertices, faces=faces)
    np.savez_compressed(output/'recorded_evidence.npz', expected_known_m=expected, observed_m=observed,
                        expected_unknown_m=unknown, missing_reference_m=missing,
                        rounding_boundary_reference_m=boundary)
    print('Bound recorded evidence:', len(missing), 'missing reference samples;', len(vertices), 'mesh vertices', flush=True)
    protocol = {'schema': 'behav3d.recorded_recovery_protocol.v1', 'label': local['label'],
                'frame': local['frame'], 'units': 'm', 'scan_cycle': 6, 'candidate_cycle': 5,
                'evaluation_id': packet['evaluation_id'], 'sources_sha256': sources,
                'hypothesis_source': 'user-requested experiments, not new Agent007 failure classifications',
                'missing_case': 'unmodified recorded DDS evidence, tolerance-sensitive, physical failure unconfirmed',
                'distance_export_precision_mm': .001,
                'missing_sample_selection': 'strictly above tolerance plus half the export rounding unit; boundary samples excluded from reach scoring',
                'saved_missing_count': candidate['evidence']['missing_voxels'],
                'spatially_unambiguous_missing_count': len(missing), 'rounding_boundary_count': len(boundary),
                'lean_case': 'controlled regional shear of the full recorded mesh; no new sensor observations',
                'lean_parameters': LEAN, 'lean_angles_deg': ANGLES, 'default_lean_deg': 8,
                'comparison_batch_budget': 2,
                'repair_probe': 'one bead at missing-reference centroid, same width/height/axis as C5:T7',
                'next_batch_geometry': 'recorded C6 positions/axes; proxy dimensions borrowed from C5 (C6 YAML has no dimensions)',
                'prioritization': 'reorder archived C6 batch by nominal containment of missing samples, then centroid distance',
                'available_goal_data': 'archived field point cloud and contour; original field faces/scalars, avoidance state and runtime settings absent from this recording',
                'planner_scope': 'archived rule-generated batch replay and geometric proposals; full planner regeneration not performed',
                'required_sweep': 'preserve all 16 recorded poses; extra inspection remains a separate proposal',
                'uses_future_scan': False, 'execution_authorized': False, 'robot_commands': []}
    dump(output/'protocol.json', protocol)
    inspection = 'Reassess the same region with fixed registration/tolerances; track missing and extra evidence and unknown coverage.'
    plans = [strategy('normal2', 'Normal: first two archived candidates', baseline[:2], triangles,
                      basis='unchanged archived order', inspection=inspection),
             strategy('priority2', 'Prioritize the discrepancy region: two candidates', priority_order(baseline, missing)[:2], triangles,
                      basis='same archived candidate pool; changed order only', inspection=inspection),
             strategy('repair1', 'Local repair probe: one recorded-size bead', [repair_probe(missing, local['target'])], triangles,
                      basis='new geometry proposal; not generated by the field planner', inspection=inspection),
             strategy('normal7', 'Reference: full archived batch of seven', baseline, triangles,
                      basis='cost reference, not equal budget', inspection=inspection),
             strategy('inspect', 'Hold deposition and inspect the discrepancy', [], triangles,
                      basis='recorded tolerance sensitivity remains unresolved', inspection=inspection)]
    for plan in plans:
        plan['metrics'] = probe_metrics(plan['targets'], missing, observed, intended)
    missing_case = {'id': 'missing', 'title': 'Recorded discrepancy · C5:T7',
                    'evidence': candidate['evidence'], 'tolerance_sensitive': candidate['tolerance_sensitive'],
                    'tolerance_sweep': [dict(tolerance_mm=t, **s) for t,s in zip(overview['tolerances_mm'],target['summaries'])],
                    'physical_failure_confirmed': False, 'strategies': plans,
                    'spatially_unambiguous_missing_count': len(missing), 'rounding_boundary_count': len(boundary),
                    'decision': evidence_decision(identity_supported=evaluated.get('paired_position_evidence_eligible') is True,
                                                  tolerance_sensitive=candidate['tolerance_sensitive'],
                                                  observation_fraction=candidate['evidence']['observation_fraction']),
                    'decision_note': 'Geometric reach is insufficient to select deposition. Recorded evidence remains provisional.'}
    lean_trials = []
    for angle in ANGLES:
        print('Comparing controlled lean:', angle, 'degrees', flush=True)
        moved = lean(vertices, angle, LEAN)
        moved_triangles = moved[faces]
        adapted = adapt_targets(baseline, angle, LEAN)
        # Region membership is fixed from the declared perturbation, not chosen
        # after observing a favorable outcome at one severity.
        nominal_top = np.array([r['top_m'] for r in baseline])
        affected = np.linalg.norm(lean(nominal_top, 8, LEAN)-nominal_top, axis=1) > 1e-6
        deferred = [r for i,r in enumerate(baseline) if not affected[i]]
        trial_plans = [strategy('continue', 'Continue the archived batch', baseline, moved_triangles,
                               basis='unchanged candidates against changed mesh', inspection='Track regional displacement and its change after a bounded batch.'),
                       strategy('defer', 'Defer growth in the edited region', deferred, moved_triangles,
                                basis='remove region candidates from this batch; no stabilizing effect assumed', inspection='Check whether the region continues to move before resuming growth.'),
                       strategy('adapt', 'Adapt nearby candidate bases to the edited geometry', adapted, moved_triangles,
                                basis='counterfactual candidate adaptation; a revised goal field still needs construction', inspection='Compare the resulting geometry with both original and proposed continuation targets.'),
                       strategy('inspect', 'Hold deposition and inspect the whole affected region', [], moved_triangles,
                                basis='mechanical stability unresolved', inspection='Measure region pose, deformation, attachment and temporal change.')]
        displacement = np.linalg.norm(moved-vertices, axis=1)
        for plan in trial_plans:
            orig = {r['label']: r for r in baseline}
            shifts = [np.linalg.norm(np.array(r['top_m'])-orig[r['label']]['top_m'])*1000 for r in plan['targets']]
            plan['metrics'] = {'candidate_count': len(plan['targets']), 'max_target_change_mm': max(shifts, default=0),
                               'affected_region_candidates': sum(bool(affected[i]) for i,r in enumerate(baseline)
                                                                 if r['label'] in {t['label'] for t in plan['targets']}),
                               'stability_improvement': None}
        trial = {'angle_deg': angle, 'modified_vertex_count': int((displacement > 1e-12).sum()),
                 'affected_candidate_labels': [r['label'] for i,r in enumerate(baseline) if affected[i]],
                 'maximum_vertex_displacement_mm': float(displacement.max()*1000),
                 'max_archived_target_displacement_mm': float(np.linalg.norm(lean(nominal_top, angle, LEAN)-nominal_top, axis=1).max()*1000),
                 'strategies': trial_plans, 'decision': 'compare_geometry_require_stability_evidence',
                 'physical_failure_confirmed': False, 'mechanics_simulated': False}
        lean_trials.append(trial)
        if angle == 8:
            np.savez_compressed(output/'controlled_lean_8deg_mesh.npz', vertices_m=moved, faces=faces,
                                source_vertex_indices=np.arange(len(vertices)))
    results = {'schema': 'behav3d.recorded_recovery_results.v1', 'frame': local['frame'], 'units': 'm',
               'missing_case': missing_case, 'lean_trials': lean_trials, 'execution_authorized': False,
               'robot_commands': [], 'protocol_sha256': digest(output/'protocol.json')}
    dump(output/'results.json', results)
    # Only the viewer is sampled. Every mesh triangle was retained in ray checks.
    display_mask = ((vertices[:,0] > -.02)&(vertices[:,0] < .36)&(vertices[:,1] > -1.0)&
                    (vertices[:,1] < -.76)&(vertices[:,2] > -.181))
    display = vertices[display_mask]
    if len(display) > 14000:
        display = display[np.linspace(0, len(display)-1, 14000, dtype=int)]
    field = ply_points(next_field)
    display_field = field[::max(1, len(field)//2500)]
    payload = {'results': results, 'protocol': {k:v for k,v in protocol.items() if k != 'sources_sha256'},
               'recorded': display.tolist(), 'lean_scenes': {str(a): lean(display,a,LEAN).tolist() for a in ANGLES},
               'intended': intended, 'field': display_field.tolist(), 'contour': ply_points(next_contour).tolist(),
               'expected': expected.tolist(), 'missing': missing.tolist(), 'observed': observed.tolist(),
               'unknown': unknown.tolist(), 'rounding_boundary': boundary.tolist()}
    template = Path(__file__).with_name('viewer.html').read_text()
    (output/'viewer.html').write_text(template.replace('/*DATA*/', json.dumps(payload, allow_nan=False)))
    # Verify that building the fixture did not mutate any source, including the
    # external recorded dataset, review outputs, or experiment implementation.
    if snapshot_sources(source_paths) != sources:
        raise ValueError('a source changed during the experiment')
    artifacts = {p.name: digest(p) for p in output.iterdir() if p.name in {
        'recorded_mesh.npz','recorded_evidence.npz','controlled_lean_8deg_mesh.npz','protocol.json','results.json','viewer.html'}}
    dump(output/'manifest.json', {'schema':'behav3d.recovery_artifacts.v1','files_sha256':artifacts,
                                  'source_hashes_unchanged':True,'execution_authorized':False})
    print(json.dumps({'output':str(output),'missing_samples':len(missing),
                      'lean_trials':len(lean_trials),'source_hashes_unchanged':True}), flush=True)


if __name__ == '__main__':
    main()
