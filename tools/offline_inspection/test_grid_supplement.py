import copy
import json
from pathlib import Path
import tempfile
import unittest
from grid_supplement import plan_supplement, shortest_append_order, rotation_change_deg
from build_grid_supplement import canonical_digest, validate_assessment

YES='inspection_opportunity_unknown_space'
NO='unassigned_observed_scene'


def pose(index,x,axes=None):
    return {'index':index,'position_m':[x,0,1],'frame':'test',
            'optical_axes_world':axes or [[1,0,0],[0,-1,0],[0,0,-1]]}


def group(items):
    return {'poses':[p for p,mask in items],
            'assessments':[{'index':p['index'],'outcomes':[YES if v else NO for v in mask],
                            'eligible_for_offline_selection':True} for p,mask in items]}


class GridSupplementTests(unittest.TestCase):
    def setUp(self):
        self.surface={'points_m':[[0,0,0],[.01,0,0],[.02,0,0],[.03,0,0]],
                      'regions':['side']*4,'area_weights_m2':[1.]*4}
        self.grid=group([(pose(0,0),[1,0,0,0]),(pose(1,.1),[1,0,0,0])])
        self.pool=group([(pose(10,.2),[1,1,0,0]),(pose(11,.4),[0,0,1,0]),
                         (pose(12,.6),[0,0,0,1]),(pose(13,.8),[1,0,0,0])])

    def plan(self,**kwargs):
        return plan_supplement(self.grid,self.pool,self.surface,frame='test',units='m',**kwargs)

    def test_new_coverage_is_relative_to_grid_and_stops(self):
        original=copy.deepcopy(self.grid)
        result=self.plan()
        self.assertEqual(result['recommended']['extra_views'],3)
        self.assertEqual(result['recommended']['new_side_area_fraction'],.75)
        self.assertEqual(result['curves'][4]['stop_reason'],'no_further_new_side_area_in_pool')
        self.assertNotIn(13,result['recommended']['selected_pool_indices'])
        self.assertEqual(self.grid,original)
        self.assertFalse(result['execution_authorized']);self.assertEqual(result['robot_commands'],[])
        self.assertEqual(result,self.plan())

    def test_shortest_order_includes_last_grid_pose(self):
        ordered,cost=shortest_append_order(pose(0,0),[pose(9,3),pose(5,1),pose(7,2)])
        self.assertEqual([p['index'] for p in ordered],[5,7,9])
        self.assertAlmostEqual(cost['position_order_length_m'],3)
        self.assertAlmostEqual(cost['summed_orientation_change_deg'],0)

    def test_same_coverage_prefers_less_travel(self):
        self.pool=group([(pose(10,2),[0,1,0,0]),(pose(11,.2),[0,1,0,0])])
        result=self.plan()
        self.assertEqual(result['recommended']['selected_pool_indices'],[11])
        self.assertEqual(result['recommended']['extra_views'],1)

    def test_no_gain_or_zero_budget_adds_no_views(self):
        self.pool=group([(pose(10,.2),[1,0,0,0])])
        self.assertEqual(self.plan()['status'],'no_supplement_adds_new_side_area')
        self.assertEqual(self.plan(max_extra=0)['recommended']['extra_views'],0)

    def test_ineligible_pool_and_invalid_baseline(self):
        self.pool['assessments'][2]['eligible_for_offline_selection']=False
        self.assertNotIn(12,self.plan()['recommended']['selected_pool_indices'])
        self.grid['assessments'][0]['eligible_for_offline_selection']=False
        result=self.plan()
        self.assertEqual(result['status'],'hold_baseline_grid_has_rejected_views')
        self.assertIsNone(result['recommended'])

    def test_unique_pose_ids_frames_and_shapes(self):
        for mutate in (lambda p:p['poses'][1].update(index=10),
                       lambda p:p['poses'][0].update(frame='world'),
                       lambda p:p['assessments'][0].update(outcomes=[YES])):
            with self.subTest(mutate=mutate):
                prior=copy.deepcopy(self.pool);mutate(self.pool)
                with self.assertRaises(ValueError):self.plan()
                self.pool=prior
        with self.assertRaises(ValueError):self.plan(max_extra=5)
        self.pool['poses'][0]['optical_axes_world']=[[0,0,0]]*3
        with self.assertRaises(ValueError):self.plan()

    def test_orientation_change_uses_full_rotation(self):
        a=pose(0,0);b=pose(1,0,[[0,-1,0],[-1,0,0],[0,0,-1]])
        self.assertAlmostEqual(rotation_change_deg(a,b),90)

    def test_content_revision_and_changed_source_rejected(self):
        import hashlib
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'source.json';path.write_text('{}')
            assessment={'candidate_cycle':5,'scan_cycle':6,'sources':{str(path):hashlib.sha256(path.read_bytes()).hexdigest()}}
            assessment['assessment_sha256']=canonical_digest(assessment)
            validate_assessment(assessment)
            bad=copy.deepcopy(assessment);bad['candidate_cycle']=4
            with self.assertRaises(ValueError):validate_assessment(bad)
            path.write_text('{"changed":true}')
            with self.assertRaises(ValueError):validate_assessment(assessment)
