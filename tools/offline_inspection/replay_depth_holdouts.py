"""Re-evaluate three recorded cases from raw depth, with held-out captures.

Loads only Agent007's audited numerical cap estimator, with its saved code hash.
No worker, service, reconstruction or robot module is imported. Source bytecode
writes are disabled. The recorded sweep remains mandatory in scan proposals;
12/13/14-view subsets here are diagnostic ablations, never replacement scans.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
from html import escape
import importlib.util
from itertools import combinations
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml

from build_frontier_scan import (file_digest, inspection_requests, load_bundle, recorded_baseline)
from build_grid_supplement import canonical_digest, validate_assessment
from depth_holdout import (backproject_region, four_folds, order_heldout_views, subset_cloud)
from frontier_scan import plan_frontier_scan
from inspection_policy import qualify_requests

CASES = ('C1:T4', 'C5:T7', 'C11:T4')


def load_estimator(agent_root, expected_hash):
    path = agent_root / 'src/ocsr/printed_artifacts/candidate_caps.py'
    if file_digest(path) != expected_hash:
        raise ValueError('cap estimator differs from recorded producer; inspect revision before replay')
    # This file was audited: only numpy/scipy/skimage and dataclass definitions.
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location('recorded_cap_estimator', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module, path


def cap_summary(cap):
    fit = cap['diagnostic_fit'] or {}
    return {'status': cap['status'], 'passes_provisional_cap_gates': cap['status'] == 'observed_cap_estimate',
            'reasons': cap['reasons'], 'raw_point_count': cap.get('raw_point_count', 0),
            'multi_view_cells': cap.get('multi_view_cell_count', 0),
            'cap_cells': cap.get('cap_cell_count', 0), 'angular_bins': fit.get('angular_bins'),
            'complete_views': len(fit.get('per_view_estimates', [])),
            'outline_residual_p90_mm': fit.get('circle_boundary_residual_p90_mm'),
            'neighbor_ambiguity_fraction': fit.get('neighbour_ambiguity_fraction'),
            'view_spread_mm': fit.get('maximum_xy_view_spread_mm'),
            'centre_m': fit.get('centre_m')}


def assert_reference_reproduced(actual, saved):
    a, b = cap_summary(actual), cap_summary(saved)
    for key in ('status', 'reasons', 'raw_point_count', 'multi_view_cells', 'cap_cells', 'angular_bins', 'complete_views'):
        if a[key] != b[key]:
            raise ValueError('full-depth replay differs from saved cap: ' + key)
    for key in ('outline_residual_p90_mm', 'neighbor_ambiguity_fraction', 'view_spread_mm', 'centre_m'):
        if (a[key] is None) != (b[key] is None) or (a[key] is not None and not np.allclose(a[key], b[key], atol=1e-5, rtol=1e-6)):
            raise ValueError('full-depth replay differs numerically from saved cap: ' + key)


def raw_inputs(local, evaluation, target, estimator, cfg):
    baseline, provenance = recorded_baseline(local)
    manifest_path = Path(provenance['manifest_path'])
    scan = manifest_path.parent
    captures = yaml.safe_load(manifest_path.read_text())['captures']
    metrics = json.loads(Path(target['source_metrics']).read_text())
    metadata_path = Path(metrics['tsdf_export_path'])
    with np.load(metadata_path, allow_pickle=False) as bundle:
        meta_bytes = bytes(bundle['_meta'])
    meta = json.loads(meta_bytes)
    intr_path = next(Path(p) for p in local['sources'] if p.endswith('/config/depth_intrinsics.yaml'))
    intr = yaml.safe_load(intr_path.read_text().replace('%YAML:1.0', '').replace('!!opencv-matrix', ''))
    matrix = np.array(intr['camera_matrix']['data']).reshape(3, 3)
    k = {'width': intr['image_width'], 'height': intr['image_height'], 'fx': matrix[0, 0],
         'fy': matrix[1, 1], 'cx': matrix[0, 2], 'cy': matrix[1, 2]}
    if (Path(meta['scan_folder']).resolve() != scan.resolve() or meta['batch']
            or meta['extrinsic_frame_key'] != 'T_tool0_ir'
            or not np.allclose(meta['depth_intrinsics']['matrix'], matrix)
            or meta['depth_intrinsics']['width'] != k['width'] or meta['depth_intrinsics']['height'] != k['height']):
        raise ValueError('depth processing metadata does not match this recorded scan')
    top, chart = estimator.candidate_chart(target)
    clouds, per_view, sources = {}, [], {}
    for capture, pose in zip(captures, baseline['poses'], strict=True):
        path = (scan / capture['depth']).resolve()
        if not path.is_relative_to(scan.resolve()):
            raise ValueError('depth path must belong to this scan')
        digest = file_digest(path)
        if evaluation['source_sha256'].get(str(path)) != digest:
            raise ValueError('raw depth is not bound to the evaluation')
        sources[str(path)] = digest
        with Image.open(path) as im:
            raw = np.array(im)
        cloud, detail = backproject_region(raw, pose, k, meta, top, chart,
                                           target['width_mm'], target['height_mm'], cfg.search_padding_mm)
        clouds[capture['index']] = cloud
        per_view.append(dict(detail, capture_index=capture['index']))
    return baseline, clouds, per_view, k, meta, {
        'raw_depth_sha256': sources, 'metadata_path': str(metadata_path),
        'metadata_sha256': hashlib.sha256(meta_bytes).hexdigest(),
        'depth_processing': {key: meta[key] for key in ('depth_scale', 'depth_bias_m', 'depth_max',
                              'center_crop_depth', 'center_crop_width', 'center_crop_height', 'extrinsic_frame_key')},
        'projection_basis': 'same ideal-pinhole depth convention as saved Agent007 cap replay; no new calibration claim'}


def run_case(label, agent_root, batch_root, cap_root, estimator):
    folder = batch_root / label.lower().replace(':', '-')
    local = json.loads((folder / 'local_scene.json').read_text())
    validate_assessment(local)
    cycle = local['scan_cycle']
    bundle = agent_root / f'outputs/integrated_evaluation_260728_133013_final/cycle_{cycle:04d}/evaluation'
    packet, evaluation = load_bundle(bundle)
    review_path = next(Path(p) for p in local['sources'] if p.endswith('/review.json'))
    review = json.loads(review_path.read_text())
    target = next(c for c in review['assessed_candidates'] if c['label'] == label)
    candidate = next(c for c in evaluation['candidates'] if c['label'] == label)
    cap_path = cap_root / f'cycle_{cycle:04d}/candidate_caps.json'
    caps = json.loads(cap_path.read_text())
    if caps['scan_cycle'] != cycle:
        raise ValueError('saved cap cycle mismatch')
    for path, digest in caps['source_sha256'].items():
        if file_digest(path) != digest:
            raise ValueError('saved cap source changed: ' + path)
    saved = next(c for c in caps['candidates'] if c['label'] == label)['cap']
    cfg = estimator.CapSettings(**saved['settings'])
    baseline, clouds, views, k, meta, provenance = raw_inputs(local, evaluation, target, estimator, cfg)
    indices = [p['index'] for p in baseline['poses']]
    folds = four_folds(indices)
    # Freeze these orders before running any held-out fit. Only poses/target/crop enter.
    for fold in folds:
        fold['ranked_held_out_before_depth'] = order_heldout_views(baseline['poses'], fold['training'],
            fold['held_out'], target['position_m'], k, meta)
    cache = {}
    def estimate(ids):
        ordered = [i for i in indices if i in set(ids)]
        key = tuple(ordered)
        if key not in cache:
            points, ids_array = subset_cloud(clouds, ordered)
            cap, _ = estimator.estimate_candidate_cap(points, ids_array, target, review['assessed_candidates'], cfg)
            cache[key] = cap
        return cache[key]
    full = estimate(indices)
    assert_reference_reproduced(full, saved)
    for fold in folds:
        fold['training_result'] = cap_summary(estimate(fold['training']))
        fold['selected_additions'] = []
        for count in (1, 2):
            chosen = fold['ranked_held_out_before_depth'][:count]
            fold['selected_additions'].append({'budget': count, 'actual_added_views': chosen,
                                               'result': cap_summary(estimate(fold['training'] + chosen))})
        # Enumerate all restorations as an explicitly retrospective diagnostic,
        # never as the selected scan policy or an unseen-depth prediction.
        fold['all_restorations_retrospective'] = []
        for count in (1, 2):
            for combo in combinations(fold['held_out'], count):
                fold['all_restorations_retrospective'].append({'added_views': list(combo),
                    'result': cap_summary(estimate(fold['training'] + list(combo)))})
    requests, deferred, binding = inspection_requests(packet, evaluation, local)
    qualified, decisions = qualify_requests(requests, candidate)
    plans = [plan_frontier_scan(baseline, local['patterns']['direction_pool'], local['nominal_surface'], qualified,
                               frame=local['frame'], units='m', max_extra=n) for n in (0, 1, 2)]
    for plan in plans:
        if plan['baseline_poses'] != baseline['poses']:
            raise AssertionError('full mandatory sweep changed')
    return {'label': label, 'candidate_cycle': cycle-1, 'scan_cycle': cycle,
            'source_binding': binding, 'reference_reproduced': True,
            'full_recorded_cap': cap_summary(full), 'cap_settings': asdict(cfg),
            'raw_view_diagnostics': views, 'folds': folds, 'qualification_decisions': decisions,
            'geometry_budget_plans': plans, 'deferred_reviews': deferred,
            'sources': {str(cap_path): file_digest(cap_path), str(folder / 'local_scene.json'): file_digest(folder / 'local_scene.json'),
                        str(bundle / 'robot_review_packet.json'): file_digest(bundle / 'robot_review_packet.json'),
                        str(bundle / 'evaluation.json'): file_digest(bundle / 'evaluation.json')},
            'raw_provenance': provenance, 'execution_authorized': False, 'robot_commands': []}


def report_html(result):
    def metric(r):
        residual = r['outline_residual_p90_mm']
        return ('Pass' if r['passes_provisional_cap_gates'] else 'Review') + f" · {r['complete_views']} complete views · " + (f'{residual:.3f} mm' if residual is not None else 'no fit')
    sections = []
    for case in result['cases']:
        rows = []
        for fold in case['folds']:
            values = [str(fold['fold']+1), ', '.join(map(str, fold['held_out'])), metric(fold['training_result']),
                      *[metric(a['result']) for a in fold['selected_additions']]]
            rows.append('<tr>' + ''.join('<td>'+escape(v)+'</td>' for v in values) + '</tr>')
        gate = ''.join('<li>'+escape(d['reason'])+'</li>' for d in case['qualification_decisions'])
        plans = []
        for p in case['geometry_budget_plans']:
            req = p['requests']
            opportunity = (f"{req[0]['combined_opportunity_fraction']:.1%}" if req else 'No qualified area objective')
            plans.append(f"<tr><td>{p['max_extra_views']}</td><td>{p['baseline_capture_count']} + {p['extra_capture_count']}</td><td>{opportunity}</td><td>{p['append_order_metrics']['position_order_length_m']:.2f} m</td></tr>")
        sections.append(f'''<h2>{escape(case['label'])} · scan {case['scan_cycle']}</h2><ul>{gate}</ul>
<p>Full 16-view raw-depth replay: <strong>{escape(metric(case['full_recorded_cap']))}</strong>. Saved Agent007 result reproduced.</p>
<div class="table"><table><thead><tr><th>Fold</th><th>Held-out capture IDs</th><th>12-view subset</th><th>Restore up to 1</th><th>Restore up to 2</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>
<p>“Pass” means the provisional cap gates pass. It does not clear a shape discrepancy, establish material identity, or validate printing.</p>
<div class="table"><table><thead><tr><th>Extra-view budget</th><th>Required sweep + proposed additions</th><th>Modeled target opportunity</th><th>Added position order</th></tr></thead><tbody>{''.join(plans)}</tbody></table></div>''')
    return '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Depth evidence and scan decisions</title>
<style>body{font:16px/1.5 system-ui;max-width:1120px;margin:32px auto;padding:0 22px;background:#121c26;color:#edf3f6}h1{font-size:28px}h2{font-size:21px;margin-top:36px}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:12px;border-bottom:1px solid #405366;text-align:left}.table{overflow:auto}a{color:#88dbe0}.note{padding:16px;background:#213747}</style>
<h1>Does another recorded view improve the evidence?</h1><p class="note">Three recorded cases, four fixed holdout folds each. The operational sweep remains 16 views. Removing captures here is an offline sensitivity test, not a proposal to shorten the sweep.</p>
<p>Held-out views are ranked from recorded camera directions and commanded target geometry before their depth is assessed. Each subset is fitted afresh using the unchanged, source-hashed Agent007 cap estimator. Full-scan meshes and fitted patches do not enter view selection.</p>''' + ''.join(sections) + '''
<h2>What this test establishes</h2><p>It measures how existing grid captures affect the provisional cap evidence. The proposed oblique views have no recorded depth: their benefits remain geometric predictions. A threshold crossing after dropping observations is sensitivity, not proof that a bead improved.</p>
<p><a href="depth_holdout_results.json">All results, held-out alternatives and source identities</a></p></html>'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent-root', type=Path, required=True)
    ap.add_argument('--batch', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    cap_root = args.agent_root / 'outputs/candidate_caps_260728_133013_prefix14_validated'
    cap_run = json.loads((cap_root / 'run.json').read_text())
    estimator, estimator_path = load_estimator(args.agent_root, cap_run['code_sha256']['estimator'])
    args.output.mkdir(parents=True, exist_ok=True)
    protocol = {'cases': list(CASES), 'folds': 'hold out every fourth manifest capture; four fixed folds',
                'ranking': 'new view direction, then camera displacement; no held-out depth or full mesh',
                'settings': asdict(estimator.CapSettings()),
                'estimator_source': str(estimator_path), 'estimator_sha256': file_digest(estimator_path),
                'local_code_sha256': {str(p.resolve()): file_digest(p) for p in (
                    Path(__file__), Path(__file__).with_name('depth_holdout.py'), Path(__file__).with_name('inspection_policy.py'))}}
    (args.output / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    result = {'schema': 'behav3d.depth_holdout_replay.v1', 'protocol': protocol,
              'cases': [], 'physical_improvement_validated': False,
              'execution_authorized': False, 'robot_commands': []}
    for label in CASES:
        print('Replaying raw depth', label, flush=True)
        case = run_case(label, args.agent_root, args.batch, cap_root, estimator)
        result['cases'].append(case)
        (args.output / 'depth_holdout_results.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        (args.output / 'report.html').write_text(report_html(result))
        print(json.dumps({'label': label, 'full': case['full_recorded_cap'],
                          'qualification': case['qualification_decisions'],
                          'fold_statuses': [[f['training_result']['status'], *[a['result']['status'] for a in f['selected_additions']]] for f in case['folds']]}), flush=True)


if __name__ == '__main__':
    main()
