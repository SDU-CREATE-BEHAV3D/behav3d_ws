"""Portable, hash-bound inputs for future recovery validation by the planner.

This packages supplied state; it cannot reconstruct omitted historical fields or
certify a candidate. Completeness means inputs are present, not that rules passed.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
REQUIRED = ('field_state', 'scan_mesh', 'settings', 'tool_geometry', 'tool_calibration')
BINDING_KEYS = ('artifact_id', 'mesh_revision', 'goal_revision', 'frame', 'units')
PARAMETERS = ('candidate_mode_default', 'beads_per_step', 'bead_separation_mm', 'bead_height_mm',
              'candidate_width_mode', 'candidate_bead_width_mm', 'candidate_volume_factor',
              'candidate_bead_overlap_mm', 'clearance', 'iso_level', 'use_collision')
FIELDS_NODE = ROOT / 'src/behav3d_sense/behav3d_sense/fields_node.py'
REQUEST_SCHEMA = ROOT / 'src/behav3d_interfaces/srv/GeneratePrintCandidates.srv'


def required_settings():
    """List actual node parameter reads and request fields without importing ROS."""
    tree = ast.parse(FIELDS_NODE.read_text())
    params = {node.args[0].value for node in ast.walk(tree)
              if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
              and node.func.attr == 'get_parameter' and node.args
              and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str)}
    request = {}
    for line in REQUEST_SCHEMA.read_text().split('---', 1)[0].splitlines():
        words = line.split('#', 1)[0].split()
        if len(words) >= 2: request[words[1]] = words[0]
    return {'effective_parameters': sorted(params | set(PARAMETERS)), 'effective_request': list(request),
            'request_types': request}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _mesh(vertices, faces):
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
            or not np.isfinite(vertices).all()):
        raise ValueError('finite nonempty mesh vertices required')
    if (faces.ndim != 2 or faces.shape[1] != 3 or not len(faces)
            or not np.issubdtype(faces.dtype, np.integer)
            or faces.min() < 0 or faces.max() >= len(vertices)):
        raise ValueError('valid triangle indices required')


def _validate_inputs(paths, spec):
    binding = spec['binding']
    if set(binding) != set(BINDING_KEYS) or binding['units'] != 'm' or not all(isinstance(v, str) and v for v in binding.values()):
        raise ValueError('complete revision/frame binding in metres required')
    avoidance = spec['avoidance']
    if avoidance['mode'] not in ('mesh', 'explicitly_none') or not avoidance.get('revision'):
        raise ValueError('avoidance state must be explicit and versioned')
    if avoidance['mode'] == 'explicitly_none' and (not avoidance.get('reason') or 'avoidance_map' in paths):
        raise ValueError('explicit absence needs a reason and cannot also contain a map')
    with np.load(paths['scan_mesh'], allow_pickle=False) as data:
        _mesh(data['vertices_m'], data['faces'])
    with np.load(paths['field_state'], allow_pickle=False) as data:
        vertices, faces = data['field_vertices_scaled'], data['field_faces']
        _mesh(vertices, faces)
        heat, offset = data['heat_norm'], data['offset_xyz']
        if heat.shape != (len(vertices),) or not np.isfinite(heat).all():
            raise ValueError('one finite heat scalar per field vertex required')
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError('explicit finite field offset required')
        if 'field_vertices_world' in data and not np.allclose(data['field_vertices_world'], vertices + offset):
            raise ValueError('field world/scaled coordinates disagree')
        vertex_count = len(vertices)
    settings = json.loads(paths['settings'].read_text())
    for key in ('effective_parameters', 'effective_request', 'field_to_scene', 'source_revision'):
        if key not in settings: raise ValueError('missing settings: ' + key)
    params = settings['effective_parameters']
    required = required_settings()
    missing = set(required['effective_parameters']) - params.keys()
    if missing: raise ValueError('missing resolved planner settings: ' + ', '.join(sorted(missing)))
    missing = set(required['effective_request']) - settings['effective_request'].keys()
    if missing: raise ValueError('missing captured request fields: ' + ', '.join(sorted(missing)))
    for key, kind in required['request_types'].items():
        value = settings['effective_request'][key]
        valid = ((kind == 'bool' and type(value) is bool)
                 or (kind == 'string' and isinstance(value, str))
                 or (kind == 'string[]' and isinstance(value, list) and all(isinstance(v, str) for v in value))
                 or (kind == 'int32' and type(value) is int)
                 or (kind == 'float64' and type(value) in (float, int) and np.isfinite(value)))
        if not valid: raise ValueError('invalid or unresolved request field: ' + key)
    for key in ('bead_separation_mm', 'bead_height_mm', 'candidate_bead_width_mm', 'candidate_volume_factor'):
        if type(params[key]) not in (int, float) or not np.isfinite(params[key]) or params[key] <= 0:
            raise ValueError('positive resolved dimension/setting required: ' + key)
    if type(params['beads_per_step']) is not int or params['beads_per_step'] < 1:
        raise ValueError('positive resolved batch size required')
    if type(params['use_collision']) is not bool: raise ValueError('explicit collision mode required')
    if params['candidate_mode_default'] not in ('z_lift', 'gradient_lift', 'gradient_walk'):
        raise ValueError('unsupported captured planner mode')
    # This transform makes frame conversion explicit instead of guessing historical flips.
    transform = np.asarray(settings['field_to_scene'], dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1)):
        raise ValueError('field_to_scene must be a rigid 4x4 transform in metres')
    if params['candidate_width_mode'] not in ('fixed', 'field'):
        raise ValueError('invalid width mode')
    if params['candidate_width_mode'] == 'field':
        if 'width_map' not in paths: raise ValueError('field width mode requires the actual width map')
        with np.load(paths['width_map'], allow_pickle=False) as data:
            width = data['width_norm']
            if width.shape != (vertex_count,) or not np.isfinite(width).all() or np.any((width < 0) | (width > 1)):
                raise ValueError('width map must match field vertices and contain values in [0,1]')
        for key in ('candidate_bead_width_min_mm', 'candidate_bead_width_max_mm'):
            if key not in params: raise ValueError('missing mapped-width parameter: ' + key)
    if spec['avoidance']['mode'] == 'mesh':
        with np.load(paths['avoidance_map'], allow_pickle=False) as data:
            _mesh(data['vertices_m'], data['faces'])
    calibration = json.loads(paths['tool_calibration'].read_text())
    for key in ('revision', 'tool0_to_tcp_xyz_m', 'tool0_to_tcp_rpy_rad', 'status'):
        if key not in calibration: raise ValueError('missing tool calibration field: ' + key)
    for key in ('tool0_to_tcp_xyz_m', 'tool0_to_tcp_rpy_rad'):
        values = np.asarray(calibration[key], dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all(): raise ValueError('invalid tool transform')
    if calibration['status'] not in ('verified', 'unverified'):
        raise ValueError('explicit calibration status required')
    json_digest(settings)
    json_digest(calibration)


def pack_snapshot(spec, output):
    """Freeze explicit file inputs. No default field, avoidance or settings."""
    if spec['schema'] != 'behav3d.planner_snapshot_spec.v1':
        raise ValueError('unsupported snapshot specification')
    binding = spec['binding']
    if set(binding) != set(BINDING_KEYS) or binding['units'] != 'm' or not all(isinstance(v, str) and v for v in binding.values()):
        raise ValueError('complete revision/frame binding in metres required')
    paths = {role: Path(path).resolve() for role, path in spec['files'].items()}
    missing = set(REQUIRED) - paths.keys()
    if missing: raise ValueError('missing planner inputs: ' + ', '.join(sorted(missing)))
    if not set(paths) <= set(REQUIRED) | {'avoidance_map', 'width_map'}:
        raise ValueError('unsupported input role')
    avoidance = spec['avoidance']
    if avoidance['mode'] not in ('mesh', 'explicitly_none') or not avoidance.get('revision'):
        raise ValueError('avoidance state must be explicit and versioned')
    if avoidance['mode'] == 'mesh' and 'avoidance_map' not in paths:
        raise ValueError('avoidance mesh missing')
    if avoidance['mode'] == 'explicitly_none' and (not avoidance.get('reason') or 'avoidance_map' in paths):
        raise ValueError('explicit absence needs a reason and cannot also contain a map')
    _validate_inputs(paths, spec)
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()): raise ValueError('snapshot destination must be empty')
    hashes = {role: sha(path) for role, path in paths.items()}
    output.mkdir(parents=True, exist_ok=True)
    assets = {}
    for role, path in paths.items():
        filename = role + path.suffix
        shutil.copyfile(path, output / filename)
        if sha(output / filename) != hashes[role] or sha(path) != hashes[role]:
            raise ValueError('source changed while snapshotting: ' + role)
        assets[role] = {'path': filename, 'sha256': hashes[role]}
    sources = [FIELDS_NODE, REQUEST_SCHEMA]
    sources += sorted((ROOT / 'src/behav3d_py/behav3d_py/scalar_field/lib_scalar').glob('*.py'))
    manifest = {'schema': 'behav3d.planner_snapshot.v1', 'binding': binding,
                'avoidance': avoidance, 'assets': assets,
                'planner_source_sha256': {str(p.relative_to(ROOT)): sha(p) for p in sources},
                'scope': 'Captured inputs only. Complete pipeline replay and candidate validation are separate.',
                'executable': False}
    (output / 'snapshot.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    return inspect_snapshot(output / 'snapshot.json')


def inspect_snapshot(path):
    path = Path(path).resolve()
    manifest = json.loads(path.read_text())
    if manifest['schema'] != 'behav3d.planner_snapshot.v1': raise ValueError('unsupported snapshot')
    assets = {}
    for role, record in manifest['assets'].items():
        asset = (path.parent / record['path']).resolve()
        if asset.parent != path.parent or sha(asset) != record['sha256']:
            raise ValueError('missing, modified or external snapshot asset: ' + role)
        assets[role] = asset
    if set(REQUIRED) - assets.keys(): raise ValueError('snapshot lacks required assets')
    _validate_inputs(assets, manifest)
    if not manifest['planner_source_sha256']: raise ValueError('planner revision missing')
    for name, expected in manifest['planner_source_sha256'].items():
        source = (ROOT / name).resolve()
        if not source.is_relative_to(ROOT) or sha(source) != expected:
            raise ValueError('planner source changed; replay needs an explicit new revision')
    return {'status': 'complete', 'binding': manifest['binding'],
            'snapshot_sha256': json_digest(manifest), 'snapshot_path': str(path),
            'validation_status': 'not_run', 'executable': False}


def incomplete_snapshot(binding, available):
    return {'status': 'incomplete', 'binding': binding, 'snapshot_sha256': None,
            'available': available,
            'missing': ['full goal field faces/scalars', 'effective planner settings',
                        'explicit avoidance state', 'historical bead dimensions/width map',
                        'verified tool calibration'],
            'validation_status': 'not_run', 'executable': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--spec', type=Path)
    group.add_argument('--inspect', type=Path)
    group.add_argument('--requirements', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.spec and not args.output: parser.error('--spec requires --output')
    result = (required_settings() if args.requirements else
              pack_snapshot(json.loads(args.spec.read_text()), args.output) if args.spec
              else inspect_snapshot(args.inspect))
    print(json.dumps(result, indent=2))
