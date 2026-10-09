import unittest
import numpy as np
from local_scene import (NominalBead, ObservedScene, assess_views, coverage,
                         segment_envelopes, select_views, spatial_relation)
from assess_local_scene import directional_pool


def plane(z, x=0, half=.02):
    return np.array([[[x-half,-half,z],[x+half,-half,z],[x+half,half,z]],
                     [[x-half,-half,z],[x+half,half,z],[x-half,half,z]]])


def surface(points, normals=None):
    return {'points_m':points,'normals':normals or [[0,0,1]]*len(points),
            'regions':['side']*len(points),'area_weights_m2':[1.]*len(points)}


def camera(x=0,z=.4,index=0):
    return {'index':index,'position_m':[x,0,z],
            'optical_axes_world':[[1,0,0],[0,-1,0],[0,0,-1]]}


class LocalSceneTests(unittest.TestCase):
    def setUp(self):
        self.target=NominalBead('target',(0,0,0),(0,0,1),.036,.016)
        self.k={'width':640,'height':480,'fx':400,'fy':400,'cx':320,'cy':240}
        self.envelope={'basis':'test sphere','margin_m':.001,
                       'spheres':[{'name':'body','center_optical_m':[0,0,-.01],'radius_m':.02}]}

    def assess(self,triangles,neighbors=(),poses=None):
        return assess_views(poses or [camera()],surface([[0,0,0]]),self.k,
                            ObservedScene(triangles),self.target,neighbors,self.envelope)

    def test_target_region_occluder_is_retained(self):
        rows=self.assess(plane(.006))
        self.assertEqual(rows[0]['outcomes'],['target_region_possible_self_obstruction'])
        self.assertFalse(rows[0]['blocker_relations'][0]['identity_verified'])
        self.assertAlmostEqual(rows[0]['blocker_points_m'][0][2],.006)

    def test_neighbor_and_unassigned_obstruction(self):
        neighbor=NominalBead('neighbor',(0,0,.21),(0,0,1),.036,.02)
        self.assertEqual(self.assess(plane(.2),[neighbor])[0]['outcomes'],['neighbor_region_identity_unverified'])
        self.assertEqual(self.assess(plane(.2))[0]['outcomes'],['unassigned_observed_scene'])

    def test_ambiguous_overlap_is_not_given_an_owner(self):
        neighbor=NominalBead('neighbor',(.01,0,0),(0,0,1),.036,.016)
        relation=spatial_relation([0,0,0],self.target,[neighbor],.008)
        self.assertEqual(relation['kind'],'overlapping_target_neighbor_regions')
        self.assertEqual(relation['neighbor_labels'],['neighbor'])

    def test_unobserved_intended_neighbor_is_uncertain(self):
        neighbor=NominalBead('neighbor',(0,0,.21),(0,0,1),.036,.02)
        row=self.assess(np.empty((0,3,3)),[neighbor])[0]
        self.assertEqual(row['outcomes'],['possible_intended_neighbor_obstruction'])
        self.assertEqual(row['possible_neighbor_labels'],[['neighbor']])
        self.assertIsNone(row['blocker_points_m'][0])
        self.assertFalse(row['eligible_for_offline_selection'])

    def test_empty_scene_does_not_prove_clearance(self):
        row=self.assess(np.empty((0,3,3)))[0]
        self.assertEqual(row['outcomes'],['inspection_opportunity_unknown_space'])
        self.assertEqual(row['clearance']['status'],'unknown_no_observed_geometry')
        self.assertFalse(row['eligible_for_offline_selection'])

    def test_triangle_interior_distance_and_clearance(self):
        scene=ObservedScene(plane(0,half=10))
        self.assertAlmostEqual(scene.nearest_surface([0,0,1])['distance_m'],1)
        self.assertTrue(np.allclose(scene.nearest_surface([0,0,1])['point_m'],[0,0,0]))
        clearance=scene.envelope_clearance(camera(z=.005),self.envelope)
        self.assertEqual(clearance['status'],'intersects_recorded_surface')
        self.assertFalse(clearance['full_tool_clearance_verified'])

    def test_degenerate_triangle_and_edge_distance(self):
        scene=ObservedScene(np.array([[[0,0,0],[1,0,0],[1,0,0]]]))
        self.assertAlmostEqual(scene.nearest_surface([.5,1,0])['distance_m'],1)
        self.assertAlmostEqual(scene.nearest_surface([2,1,0])['distance_m'],2**.5)

    def test_camera_envelope_rejects_possible_intended_neighbor_contact(self):
        neighbor=NominalBead('near_camera',(0,0,.42),(0,0,1),.036,.016)
        row=self.assess(plane(0),[neighbor])[0]
        self.assertEqual(row['clearance']['status'],'no_recorded_surface_intersection_unknown_space')
        self.assertTrue(row['clearance']['possible_intended_neighbor_contacts'])
        self.assertFalse(row['eligible_for_offline_selection'])

    def test_missing_envelope_fails_closed(self):
        result=ObservedScene(plane(0)).envelope_clearance(camera(),None)
        self.assertEqual(result['status'],'unverified_missing_envelope')

    def test_intended_envelopes_behind_camera_do_not_block(self):
        n=NominalBead('behind',(0,0,1),(0,0,1),.036,.016)
        self.assertEqual(segment_envelopes([0,0,.4],[0,0,0],[n],.008),[])

    def test_selection_is_deterministic_and_filters_clearance(self):
        poses=[camera(index=i) for i in range(3)]
        yes='inspection_opportunity_unknown_space';no='unassigned_observed_scene'
        rows=[{'index':0,'outcomes':[yes,no],'eligible_for_offline_selection':True},
              {'index':1,'outcomes':[no,yes],'eligible_for_offline_selection':True},
              {'index':2,'outcomes':[yes,yes],'eligible_for_offline_selection':False}]
        s=surface([[0,0,0],[.01,0,0]])
        result=select_views(poses,rows,s,2)
        self.assertEqual(result['selected_pool_indices'],[0,1])
        self.assertEqual(result,select_views(poses,rows,s,2))
        self.assertFalse(result['execution_authorized'])
        self.assertEqual(result['robot_commands'],[])
        for r in rows:r['eligible_for_offline_selection']=False
        self.assertEqual(select_views(poses,rows,s,2)['status'],'hold_no_supported_proposal')

    def test_surrounding_wall_changes_selected_direction(self):
        from behav3d_orchestrator.offline_agent.inspection import basis
        wall=np.array([[[.2,-.1,-.1],[.2,.1,-.1],[.2,.1,.2]],
                       [[.2,-.1,-.1],[.2,.1,.2],[.2,-.1,.2]]])
        poses=[]
        for i,position in enumerate(([.4,0,.1],[.4,.4,.1])):
            poses.append(dict(camera(index=i),position_m=position,
                              optical_axes_world=basis([-v for v in position])))
        s=surface([[0,0,0]],[[1,0,0]])
        rows=assess_views(poses,s,self.k,ObservedScene(wall),self.target,[],self.envelope)
        self.assertEqual(rows[0]['outcomes'],['unassigned_observed_scene'])
        self.assertEqual(rows[1]['outcomes'],['inspection_opportunity_unknown_space'])
        selected=select_views(poses,rows,s,1)
        self.assertEqual(selected['selected_pool_indices'],[1])

    def test_directional_pool_and_invalid_settings(self):
        settings={'distance_m':.4,'pool_tilts_deg':[25,50,75],'azimuth_samples':8}
        poses=directional_pool(self.target,'test_frame',settings)
        self.assertEqual(len(poses),24)
        self.assertEqual(poses,directional_pool(self.target,'test_frame',settings))
        for p in poses:
            delta=np.array(p['position_m'])-self.target.top
            self.assertAlmostEqual(np.linalg.norm(delta),.4)
            self.assertAlmostEqual(np.dot(delta,p['optical_axes_world'][2]),-.4)
        with self.assertRaises(ValueError): directional_pool(self.target,'test_frame',dict(settings,pool_tilts_deg=[float('nan')]))
