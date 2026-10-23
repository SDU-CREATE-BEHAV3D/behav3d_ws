"""Replay the recovery decision policy using the committed experimental scenes."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

import numpy as np

from geometry import adapt_targets
from build_inward_lean import deform, LEAN, SETTINGS
from planner_state import incomplete_snapshot, sha
from proposals import adaptation_proposals, surface_fill_proposals, make_proposal
from viability import Scene, tool_models, COLLISION_SOURCE, TOOL_MESH, WORKCELL

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/behav3d_orchestrator'))
from behav3d_orchestrator.offline_agent.recovery import (BINDING_KEYS, decide_recovery,
                                                       planner_request, digest)


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def verify_artifacts(folder):
    manifest = read(folder / 'manifest.json')
    for name, expected in manifest['files_sha256'].items():
        if sha(folder / name) != expected: raise ValueError('recorded artifact changed: ' + name)


def issue_fixture(case_id, mesh_revision, *, kind, origin, confirmed, trend='unknown', angle=None):
    return {'schema': 'behav3d.recovery_issue.v1', 'id': case_id,
            'artifact_id': 'recording-260728_133013', 'mesh_revision': mesh_revision,
            'goal_revision': 'archived-goal-state-unavailable', 'frame': 'ur20_base_link', 'units': 'm',
            'observation_revision': 'scan6' if kind == 'missing_material' else f'controlled-scan19-{angle}deg',
            'region_id': 'C5:T7' if kind == 'missing_material' else 'upper-right-C19:T7',
            'kind': kind, 'extent': 'local', 'connection_status': 'intact' if origin == 'controlled_experiment' else 'unknown',
            'objective': 'repair_deficit' if kind == 'missing_material' else 'continue_from_observed',
            'benefit_basis': ('missing_reference_samples_reached_not_volume' if kind == 'missing_material' else
                              'additional_base_rays_within_experimental_gap_not_stability'),
            'attempt_count': 0, 'last_outcome': 'not_attempted',
            'evidence': {'origin': origin, 'references': ['recorded_cycle6/protocol.json'] if kind == 'missing_material'
                         else ['inward_cycle19/protocol.json', f'controlled-deformation-{angle}deg'],
                         'identity_supported': True, 'registration_supported': True, 'coverage_sufficient': True,
                         'failure_confirmed': confirmed, 'tolerance_sensitive': origin == 'recorded',
                         'trend': trend, 'trend_observation_ids': [] if trend == 'unknown' else
                         [case_id + '/assumed-observation-a', case_id + '/assumed-observation-b'],
                         'assumptions': ('Controlled policy inputs on recorded geometry; confirmation, intact connections and '
                                         'any stable/progressing trend are test assumptions, not Agent007 detections.')}}


def display_points(vertices):
    vertices = vertices[(vertices[:, 0] > -.02) & (vertices[:, 0] < .36)
                        & (vertices[:, 1] > -1) & (vertices[:, 1] < -.76) & (vertices[:, 2] > -.19)]
    if len(vertices) > 7000: vertices = vertices[np.linspace(0, len(vertices) - 1, 7000, dtype=int)]
    return vertices.tolist()


def build(output):
    root = ROOT / 'outputs/offline_recovery'
    missing_dir, inward_dir = root / 'recorded_cycle6', root / 'inward_cycle19'
    verify_artifacts(missing_dir)
    verify_artifacts(inward_dir)
    inputs = [*missing_dir.glob('*.json'), missing_dir / 'recorded_mesh.npz', missing_dir / 'recorded_evidence.npz',
              *inward_dir.glob('*.json'), inward_dir / 'recorded_mesh.npz',
              ROOT / 'outputs/offline_inspection/frontier_batch/c5-t7/local_scene.json',
              COLLISION_SOURCE, TOOL_MESH, WORKCELL, *Path(__file__).parent.glob('*.py'),
              Path(__file__).with_name('decision_viewer.html'),
              ROOT / 'src/behav3d_orchestrator/behav3d_orchestrator/offline_agent/recovery.py']
    source_hashes = {str(p.relative_to(ROOT)): sha(p) for p in inputs}
    lib, models, tool_meta = tool_models()
    cases, scenes = [], {}

    def add(title, issue, proposals, scene_key, note):
        snapshot = incomplete_snapshot({k: issue[k] for k in BINDING_KEYS}, ['full mesh', 'archived candidate batch'])
        proposals = deepcopy(proposals)
        for proposal in proposals:
            proposal['binding'] = {k: issue[k] for k in BINDING_KEYS}
        decision = decide_recovery(issue, proposals, snapshot)
        selected = next((p for p in proposals if p['id'] == decision['selected_proposal_id']), None)
        cases.append({'id': issue['id'], 'title': title, 'issue': issue, 'note': note,
                      'scene_key': scene_key, 'planner_state': snapshot, 'proposals': proposals, 'decision': decision,
                      'validation_request': planner_request(issue, selected, snapshot) if selected else None})

    print('Replaying recorded missing-region evidence', flush=True)
    with np.load(missing_dir / 'recorded_mesh.npz', allow_pickle=False) as mesh:
        vertices, faces = mesh['vertices_m'], mesh['faces']
    scene = Scene(vertices, faces)
    with np.load(missing_dir / 'recorded_evidence.npz', allow_pickle=False) as evidence:
        missing = evidence['missing_reference_m']
    scenes['scan6'] = {'points': display_points(vertices), 'missing': missing.tolist()}
    old = read(missing_dir / 'results.json')['missing_case']
    reference = read(ROOT / 'outputs/offline_inspection/frontier_batch/c5-t7/local_scene.json')['target']
    archived = next(p for p in old['strategies'] if p['id'] == 'normal7')['targets']
    proposals = surface_fill_proposals(missing, reference, scene, lib, models, SETTINGS)
    probe = next(p for p in old['strategies'] if p['id'] == 'repair1')['targets'][0]
    proposals.append(make_proposal('old-centroid-probe', 'fill', probe, scene, lib, models, SETTINGS,
                                   reference=reference, missing=missing))
    for target in archived:
        proposals.append(make_proposal('archived-' + target['label'], 'fill', target, scene, lib, models, SETTINGS,
                                       reference=reference, missing=missing))
    recorded = issue_fixture('recorded-deficit', sha(missing_dir / 'recorded_mesh.npz'), kind='missing_material',
                             origin='recorded', confirmed=False)
    add('Recorded deficit · evidence unresolved', recorded, proposals, 'scan6',
        'The recorded discrepancy is tolerance-sensitive. Unknown and rounding-boundary samples are excluded from fill benefit.')
    confirmed = deepcopy(recorded)
    confirmed['id'] = 'confirmed-deficit-hypothesis'
    confirmed['evidence'].update(origin='controlled_experiment', failure_confirmed=True, tolerance_sensitive=False)
    add('Same deficit · assume confirmation', confirmed, proposals, 'scan6',
        'Only the evidence assumption changes. This does not turn the original recording into a confirmed failure.')

    with np.load(inward_dir / 'recorded_mesh.npz', allow_pickle=False) as mesh:
        vertices, faces = mesh['vertices_m'], mesh['faces']
    old = read(inward_dir / 'results.json')
    baseline = next(t for t in old['trials'] if t['angle_deg'] == 0)['plans'][0]['targets']
    original = next(r for r in baseline if r['label'] == 'C19:T7')
    peers = [r for r in baseline if r['label'] != 'C19:T7']
    for angle in (0, 4, 8, 12):
        print(f'Evaluating bounded candidates at {angle} degrees', flush=True)
        edited = deform(vertices, angle)
        scene = Scene(edited, faces)
        scene_key = f'scan19-{angle}'
        scenes[scene_key] = {'points': display_points(edited), 'missing': []}
        seed = adapt_targets([original], -angle, LEAN)[0]
        proposals = adaptation_proposals(seed, original, scene, lib, models, SETTINGS, peers=peers)
        mesh_revision = digest({'recorded_mesh_sha256': sha(inward_dir / 'recorded_mesh.npz'), 'lean': LEAN, 'angle_deg': angle})
        issue = issue_fixture(f'lean-{angle}-stable', mesh_revision, kind='none' if angle == 0 else 'lean',
                              origin='controlled_experiment', confirmed=angle != 0, trend='stable', angle=angle)
        add('Unmodified control' if not angle else f'Inward lean {angle}° · assume stable', issue, proposals, scene_key,
            'Stable trend and intact connections are explicit test assumptions. Geometry is from the recorded scan with the declared edit.')
        if angle == 8:
            for trend in ('unknown', 'progressing'):
                variant = deepcopy(issue)
                variant['id'] = f'lean-8-{trend}'
                variant['evidence']['trend'] = trend
                variant['evidence']['trend_observation_ids'] = ([] if trend == 'unknown' else
                    ['controlled-earlier-lean', 'controlled-later-lean'])
                add(f'Same 8° lean · trend {trend}', variant, proposals, scene_key,
                    'A policy counterfactual on the same edited geometry; no new measured observations are claimed.')
            variant = deepcopy(issue)
            variant['id'] = 'lean-8-restore'
            variant['objective'] = 'restore_printed_shape'
            add('Same 8° lean · restore existing shape', variant, proposals, scene_key,
                'Changing the objective changes the strategy: future deposition does not move the printed artifact back.')

    if any(sha(ROOT / name) != expected for name, expected in source_hashes.items()):
        raise ValueError('source changed during replay')
    output.mkdir(parents=True, exist_ok=True)
    results = {'schema': 'behav3d.recovery_policy_replay.v1', 'cases': cases,
               'tool_model': tool_meta, 'screen_settings': SETTINGS,
               'source_sha256': source_hashes, 'executable': False, 'robot_commands': []}
    dump(output / 'results.json', results)
    # Display data shares scenes, while all queried meshes retain every triangle.
    data = {'cases': cases, 'scenes': scenes}
    html = Path(__file__).with_name('decision_viewer.html').read_text()
    (output / 'viewer.html').write_text(html.replace('/*DATA*/', json.dumps(data, allow_nan=False).replace('<', '\\u003c')))
    requests = [c['validation_request'] for c in cases if c['validation_request']]
    dump(output / 'validation_requests.json', requests)
    dump(output / 'manifest.json', {'files_sha256': {name: sha(output / name) for name in
                                                   ('results.json', 'viewer.html', 'validation_requests.json')},
                                   'source_hashes_unchanged': True, 'executable': False})
    for case in cases:
        decision = case['decision']
        print(case['id'], decision['next_action'], decision['selected_proposal_id'], flush=True)
    print('Wrote', output, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/offline_recovery/decisions')
    build(parser.parse_args().output)
