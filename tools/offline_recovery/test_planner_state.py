from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from planner_state import pack_snapshot, inspect_snapshot, required_settings


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        vertices = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0.]])
        faces = np.array([[0, 1, 2], [0, 2, 3]])
        np.savez(self.root / 'field.npz', field_vertices_scaled=vertices, field_faces=faces,
                 heat_norm=np.arange(4) / 3, offset_xyz=np.zeros(3))
        np.savez(self.root / 'scan.npz', vertices_m=vertices, faces=faces)
        required = required_settings()
        params = dict.fromkeys(required['effective_parameters'], 0)
        params.update(candidate_mode_default='z_lift', beads_per_step=1, bead_separation_mm=32,
                      bead_height_mm=16, candidate_width_mode='fixed', candidate_bead_width_mm=36,
                      candidate_volume_factor=1, candidate_bead_overlap_mm=4, clearance=0, iso_level=0, use_collision=False)
        request = {key: {'bool': False, 'string': '', 'string[]': [], 'int32': 0, 'float64': 0.}[kind]
                   for key, kind in required['request_types'].items()}
        self.settings = {'effective_parameters': params, 'effective_request': request,
                         'field_to_scene': np.eye(4).tolist(), 'source_revision': 'controlled-test'}
        self.write_settings()
        (self.root / 'tool.dae').write_text('<COLLADA/>')
        (self.root / 'calibration.json').write_text(json.dumps({'revision': 'test', 'status': 'unverified',
            'tool0_to_tcp_xyz_m': [0, 0, .1], 'tool0_to_tcp_rpy_rad': [0, 0, 0]}))
        self.spec = {'schema': 'behav3d.planner_snapshot_spec.v1',
                     'binding': {'artifact_id': 'test', 'mesh_revision': 'scan-a', 'goal_revision': 'goal-a',
                                 'frame': 'base', 'units': 'm'},
                     'files': {role: str(self.root / name) for role, name in
                               [('field_state', 'field.npz'), ('scan_mesh', 'scan.npz'), ('settings', 'settings.json'),
                                ('tool_geometry', 'tool.dae'), ('tool_calibration', 'calibration.json')]},
                     'avoidance': {'mode': 'explicitly_none', 'revision': 'test-a', 'reason': 'Controlled empty scene'}}

    def write_settings(self):
        (self.root / 'settings.json').write_text(json.dumps(self.settings))

    def test_snapshot_freezes_files_but_does_not_validate_candidates(self):
        result = pack_snapshot(self.spec, self.root / 'bundle')
        self.assertEqual(result['status'], 'complete')
        self.assertEqual(result['validation_status'], 'not_run')
        self.assertFalse(result['executable'])
        self.assertEqual(inspect_snapshot(result['snapshot_path'])['snapshot_sha256'], result['snapshot_sha256'])

    def test_tampered_mesh_is_rejected(self):
        result = pack_snapshot(self.spec, self.root / 'bundle')
        (self.root / 'bundle/scan_mesh.npz').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'modified'): inspect_snapshot(result['snapshot_path'])

    def test_absent_avoidance_cannot_be_silently_inferred(self):
        self.spec['avoidance']['mode'] = 'unknown'
        with self.assertRaisesRegex(ValueError, 'explicit'): pack_snapshot(self.spec, self.root / 'bundle')

    def test_field_width_mode_requires_matching_map(self):
        self.settings['effective_parameters']['candidate_width_mode'] = 'field'
        self.write_settings()
        with self.assertRaisesRegex(ValueError, 'width map'): pack_snapshot(self.spec, self.root / 'bundle')

    def test_invalid_field_scalar_count_is_rejected(self):
        np.savez(self.root / 'field.npz', field_vertices_scaled=np.eye(3), field_faces=np.array([[0, 1, 2]]),
                 heat_norm=np.zeros(2), offset_xyz=np.zeros(3))
        with self.assertRaisesRegex(ValueError, 'scalar'): pack_snapshot(self.spec, self.root / 'bundle')

    def test_settings_must_include_all_actual_parameters_and_request_fields(self):
        for section, key in [('effective_parameters', 'walk_step_mm'), ('effective_request', 'target_position_scale')]:
            with self.subTest(section=section):
                original = deepcopy(self.settings)
                del self.settings[section][key]
                self.write_settings()
                with self.assertRaisesRegex(ValueError, 'missing'): pack_snapshot(self.spec, self.root / 'bundle')
                self.settings = original

    def test_non_rigid_field_transform_is_rejected(self):
        self.settings['field_to_scene'][0][0] = 1000
        self.write_settings()
        with self.assertRaisesRegex(ValueError, 'rigid'): pack_snapshot(self.spec, self.root / 'bundle')

    def test_null_request_values_are_incomplete_state(self):
        self.settings['effective_request']['target_position_scale'] = None
        self.write_settings()
        with self.assertRaisesRegex(ValueError, 'unresolved'): pack_snapshot(self.spec, self.root / 'bundle')

    def test_existing_snapshot_cannot_be_overwritten(self):
        pack_snapshot(self.spec, self.root / 'bundle')
        with self.assertRaisesRegex(ValueError, 'empty'): pack_snapshot(self.spec, self.root / 'bundle')


if __name__ == '__main__': unittest.main()
