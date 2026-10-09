import copy
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from build_frontier_scan import inspection_requests, load_bundle, recorded_baseline, file_digest
from frontier_scan import plan_frontier_scan
from test_grid_supplement import group, pose


class FrontierScanTests(unittest.TestCase):
    def setUp(self):
        self.surface = {'points_m': [[0, 0, 0], [.01, 0, 0], [.02, 0, 0]],
                        'regions': ['top', 'top', 'side'], 'area_weights_m2': [1., 1., 1.]}
        self.baseline = group([(pose(0, 0), [1, 0, 0]), (pose(1, .1), [1, 0, 0])])
        self.pool = group([(pose(0, .2), [0, 1, 0]), (pose(1, .5), [0, 0, 1])])
        self.request = {'request_id': 'frontier-gap', 'origin': 'geometry_development',
                        'reason': 'frontier region hidden in baseline',
                        'evidence_ids': ['geometry:test-wall'], 'sample_indices': [0, 1]}

    def plan(self, requests=None, **kwargs):
        return plan_frontier_scan(self.baseline, self.pool, self.surface,
                                  [self.request] if requests is None else requests,
                                  frame='test', units='m', **kwargs)

    def test_no_inspection_need_preserves_whole_sweep_without_additions(self):
        original = copy.deepcopy(self.baseline)
        result = self.plan([])
        self.assertEqual(result['decision'], 'keep_frontier_sweep')
        self.assertEqual(result['baseline_poses'], original['poses'])
        self.assertEqual([x['pose'] for x in result['capture_order']], original['poses'])
        self.assertEqual(result['extra_capture_count'], 0)
        self.assertEqual(self.baseline, original)

    def test_blocked_frontier_request_adds_only_relevant_pose_and_keeps_prefix(self):
        result = self.plan()
        self.assertEqual([p['index'] for p in result['supplemental_poses']], [0])
        self.assertEqual(result['baseline_poses'], self.baseline['poses'])
        self.assertEqual([x['pose'] for x in result['capture_order'][:2]], self.baseline['poses'])
        self.assertAlmostEqual(result['append_order_metrics']['position_order_length_m'], .1)
        self.assertEqual(result['requests'][0]['combined_opportunity_fraction'], 1.)
        self.assertFalse(result['requests'][0]['inspection_resolved'])
        self.assertEqual(result['robot_commands'], [])
        self.assertFalse(result['execution_authorized'])

    def test_cap_review_does_not_reward_unrequested_side_area(self):
        self.request.update(sample_indices=[0], origin='agent007', reason='cap_quality_review')
        result = self.plan()
        self.assertEqual(result['extra_capture_count'], 0)
        self.assertEqual(result['requests'][0]['status'], 'unresolved_no_new_geometric_opportunity_in_pool')
        self.assertFalse(result['requests'][0]['inspection_resolved'])

    def test_shape_request_can_select_side_view(self):
        self.request.update(sample_indices=[2], origin='agent007', reason='dds_shape_size_review')
        result = self.plan()
        self.assertEqual([p['index'] for p in result['supplemental_poses']], [1])

    def test_mesh_obstruction_drives_an_addition_without_replacing_baseline(self):
        import numpy as np
        from local_scene import NominalBead, ObservedScene, assess_views
        from behav3d_orchestrator.offline_agent.inspection import basis
        wall = np.array([[[.2, -.1, -.1], [.2, .1, -.1], [.2, .1, .2]],
                         [[.2, -.1, -.1], [.2, .1, .2], [.2, -.1, .2]]])
        cameras = [dict(pose(i, 0), position_m=position,
                        optical_axes_world=basis([-v for v in position]))
                   for i, position in enumerate(([.4, 0, .1], [.4, .4, .1]))]
        surface = {'points_m': [[0, 0, 0]], 'normals': [[1, 0, 0]],
                   'regions': ['side'], 'area_weights_m2': [1.]}
        target = NominalBead('target', (0, 0, 0), (0, 0, 1), .036, .016)
        calibration = {'width': 640, 'height': 480, 'fx': 400, 'fy': 400, 'cx': 320, 'cy': 240}
        envelope = {'basis': 'synthetic sphere', 'margin_m': .001,
                    'spheres': [{'name': 'body', 'center_optical_m': [0, 0, -.01], 'radius_m': .02}]}
        rows = assess_views(cameras, surface, calibration, ObservedScene(wall), target, [], envelope)
        self.assertEqual(rows[0]['outcomes'], ['unassigned_observed_scene'])
        result = plan_frontier_scan({'poses': cameras[:1], 'assessments': rows[:1]},
                                   {'poses': cameras[1:], 'assessments': rows[1:]}, surface,
                                   [dict(self.request, sample_indices=[0])], frame='test', units='m')
        self.assertEqual(result['baseline_poses'], cameras[:1])
        self.assertEqual(result['supplemental_poses'], cameras[1:])

    def test_budget_and_clearance_never_remove_baseline(self):
        self.assertEqual(self.plan(max_extra=0)['requests'][0]['status'], 'unresolved_within_extra_view_budget')
        self.pool['assessments'][0]['eligible_for_offline_selection'] = False
        self.assertEqual(self.plan()['extra_capture_count'], 0)
        self.baseline['assessments'][0]['eligible_for_offline_selection'] = False
        result = self.plan()
        self.assertEqual(result['decision'], 'hold_baseline_assessment_rejected')
        self.assertEqual(result['baseline_poses'], self.baseline['poses'])
        self.assertEqual(result['extra_capture_count'], 0)

    def test_overlapping_requests_are_not_double_counted_and_output_is_detached(self):
        first = self.plan()
        other = dict(self.request, request_id='another-review')
        second = self.plan([self.request, other])
        self.assertEqual(first['supplemental_poses'], second['supplemental_poses'])
        first['baseline_poses'][0]['position_m'][0] = 999
        self.assertEqual(self.baseline['poses'][0]['position_m'][0], 0)
        self.assertEqual(second, self.plan([self.request, other]))

    def test_invalid_inputs_rejected(self):
        for changes in ({'sample_indices': [99]}, {'evidence_ids': []}, {'reason': ''}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.plan([dict(self.request, **changes)])
        with self.assertRaises(ValueError):
            self.plan([self.request, self.request])
        with self.assertRaises(ValueError):
            self.plan(max_extra=5)
        self.pool['poses'][0]['frame'] = 'another-frame'
        with self.assertRaises(ValueError):
            self.plan()


class RecordedAdapterTests(unittest.TestCase):
    def setUp(self):
        self.local = {'label': 'C5:T1', 'candidate_cycle': 5, 'scan_cycle': 6,
                      'frame': 'robot_base', 'units': 'm', 'target': {'top_m': [0, 0, 0]},
                      'sources': {'/review.json': 'same-hash'},
                      'nominal_surface': {'regions': ['top', 'side']}}
        suggestion = {'suggestion_id': 'review:C5:T1', 'action': 'review_scan_coverage',
                      'candidate_labels': ['C5:T1'], 'evidence_ids': ['candidate:C5:T1'],
                      'reason': 'cap_quality_review', 'executable': False,
                      'fabrication_authorized': False, 'requires_operator_review': True}
        self.packet = {'schema': 'agent007.robot_review_packet.v1', 'artifact_id': 'recording',
                       'evaluation_id': 'revision', 'scan_job_id': 'scan6',
                       'candidate_cycle': 5, 'scan_cycle': 6, 'previous_evaluation_id': 'previous',
                       'frame_id': 'world/base', 'coordinate_units': 'm', 'summary': {},
                       'suggestions': [suggestion], 'suggestion_count': 1, 'suggestions_truncated': False,
                       'future_evidence_used': False, 'fabrication_authorized': False,
                       'robot_commands_sent': False}
        self.evaluation = copy.deepcopy(self.packet)
        self.evaluation.update(context={'frame_id': 'world/base'}, source_sha256={'/review.json': 'same-hash'},
                               candidates=[{'label': 'C5:T1', 'evidence_id': 'candidate:C5:T1',
                                            'candidate_cycle': 5, 'observation_cycle': 6,
                                            'expected_top_m': [0, 0, 0], 'uncertainty_reasons': ['partial_angular_coverage']}])

    def bind(self):
        return inspection_requests(self.packet, self.evaluation, self.local)

    def test_real_field_names_bind_ids_without_aliasing_frames(self):
        requests, deferred, binding = self.bind()
        self.assertEqual(requests[0]['sample_indices'], [0])
        self.assertEqual(requests[0]['uncertainty_reasons'], ['partial_angular_coverage'])
        self.assertEqual(deferred, [])
        self.assertEqual(binding['packet_frame_metadata'], 'world/base')
        self.assertEqual(binding['planning_frame'], 'robot_base')
        self.assertFalse(binding['coordinate_transform_applied'])

    def test_registration_is_not_automatically_a_scan_request(self):
        for obj in (self.packet, self.evaluation):
            obj['suggestions'][0].update(action='review_registration', candidate_labels=[],
                                          evidence_ids=['context:registration'])
        requests, deferred, _ = self.bind()
        self.assertEqual(requests, [])
        self.assertEqual(deferred[0]['disposition'], 'evidence_review_before_scan_change')

    def test_source_cycle_target_and_packet_revision_mismatch_rejected(self):
        for mutate in (
                lambda: self.packet.update(evaluation_id='different'),
                lambda: self.local.update(scan_cycle=7),
                lambda: self.local['target'].update(top_m=[1, 0, 0]),
                lambda: self.local['sources'].update({'/review.json': 'changed'}),
                lambda: self.packet.update(suggestions_truncated=True),
                lambda: self.packet.update(coordinate_units='mm')):
            with self.subTest(mutate=mutate):
                self.setUp()
                mutate()
                with self.assertRaises(ValueError): self.bind()

    def test_unbound_evidence_and_execution_flags_rejected(self):
        for change in ({'evidence_ids': ['candidate:wrong']}, {'executable': True}):
            self.setUp()
            for obj in (self.packet, self.evaluation): obj['suggestions'][0].update(change)
            with self.assertRaises(ValueError): self.bind()

    def test_changed_bundle_content_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / 'evaluation.json').write_text(json.dumps({'source_sha256': {}}))
            (folder / 'robot_review_packet.json').write_text('{}')
            manifest = {'schema': 'agent007.evaluation_bundle.v1', 'files': {
                name: file_digest(folder / name) for name in ('evaluation.json', 'robot_review_packet.json')}}
            (folder / 'manifest.json').write_text(json.dumps(manifest))
            load_bundle(folder)
            (folder / 'robot_review_packet.json').write_text('{"changed":true}')
            with self.assertRaises(ValueError): load_bundle(folder)

    def test_recorded_baseline_rejects_crop_reorder_and_pose_change(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            manifest = folder / 'cycle_0006/scan/manifest.yaml'
            ext = folder / 'config/extrinsics.yaml'
            manifest.parent.mkdir(parents=True); ext.parent.mkdir()
            captures = [{'index': i, 'T_base_tool0': {'parent': 'robot_base',
                         'position': [i, 0, 0], 'orientation_xyzw': [0, 0, 0, 1]}} for i in range(2)]
            manifest.write_text(yaml.safe_dump({'captures': captures}))
            ext.write_text(yaml.safe_dump({'frames': {'T_tool0_ir': {'xyz': [0, 0, .1], 'quat_xyzw': [0, 0, 0, 1]}}}))
            local = dict(self.local, sources={str(manifest): file_digest(manifest), str(ext): file_digest(ext)},
                         patterns={'recorded': {'poses': [{'index': i, 'position_m': [i, 0, .1],
                              'optical_axes_world': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]} for i in range(2)]}})
            baseline, _ = recorded_baseline(local)
            self.assertEqual(len(baseline['poses']), 2)
            for mutate in (lambda g: g['poses'].pop(), lambda g: g['poses'].reverse(),
                           lambda g: g['poses'][0]['position_m'].__setitem__(0, 10)):
                bad = copy.deepcopy(local); mutate(bad['patterns']['recorded'])
                with self.assertRaises(ValueError): recorded_baseline(bad)


if __name__ == '__main__':
    unittest.main()
