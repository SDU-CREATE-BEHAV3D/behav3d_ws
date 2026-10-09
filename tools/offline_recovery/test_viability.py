import unittest

import numpy as np

from geometry import NominalBead
from build_inward_lean import deform,LEAN,SETTINGS
from viability import (Scene,TOOL_MESH,collada_vertices_faces,collision_library,
                       evaluate_candidates,seat_targets,interpolate_path,
                       swept_clearance,propose_contact,tool_models,search_supported_targets)


class ViabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vertices=np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)
        cls.faces=np.array([[0,1,2],[0,2,3]])
        cls.lib=collision_library()
        cls.models={name:{'points_tcp':np.array([[0,0,.1]])} for name in ['urdf','checker_defaults']}

    def target(self,label='A',x=0,z=.016):
        return NominalBead(label,(x,0,z),(0,0,1),.036,.016).record()

    def test_higher_anchor_and_inward_direction(self):
        points=np.array([[.24,-.875,-.10],[.24,-.875,-.075],[.24,-.875,-.025],[.15,-.875,0]])
        shifted=deform(points,8)
        np.testing.assert_array_equal(shifted[[0,1,3]],points[[0,1,3]])
        self.assertLess(shifted[2,0],points[2,0])
        self.assertAlmostEqual(points[2,0]-shifted[2,0],.05*np.tan(np.radians(8)))
        np.testing.assert_array_equal(deform(points,0),points)

    def test_repo_collada_mesh_has_valid_geometry(self):
        vertices,faces=collada_vertices_faces(TOOL_MESH)
        self.assertEqual(vertices.shape,(1874,3));self.assertEqual(faces.shape,(2564,3))
        self.assertTrue(np.isfinite(vertices).all());self.assertLess(faces.max(),len(vertices))

    def test_both_real_tool_transforms_are_retained(self):
        _,models,meta=tool_models()
        self.assertEqual(set(models),{'urdf','checker_defaults'})
        self.assertGreater(meta['tool_offset_difference_mm'],10)
        self.assertFalse(np.allclose(models['urdf']['xyz_m'],models['checker_defaults']['xyz_m']))

    def test_supported_plane_pass_is_still_non_executable(self):
        scene=Scene(self.vertices,self.faces)
        result=evaluate_candidates([self.target()],scene,self.lib,self.models,SETTINGS)[0]
        self.assertEqual(result['support_rays_within_gap'],9)
        self.assertEqual(result['geometric_screen'],'passes_declared_screens')
        self.assertFalse(result['executable'])
        self.assertEqual(result['field_goal_validity'],'not_replayed')

    def test_unsupported_base_is_rejected_even_with_clear_tool(self):
        scene=Scene(self.vertices,self.faces)
        result=evaluate_candidates([self.target(z=.03)],scene,self.lib,self.models,SETTINGS)[0]
        self.assertIn('insufficient_nominal_base_support',result['rejection_reasons'])
        self.assertTrue(all(t['screen']=='no_sampled_conflict' for t in result['tool_models'].values()))

    def test_spacing_checks_the_current_batch(self):
        scene=Scene(self.vertices,self.faces)
        results=evaluate_candidates([self.target(),self.target('B',x=.02)],scene,self.lib,self.models,SETTINGS)
        self.assertTrue(all('nominal_batch_spacing' in r['rejection_reasons'] for r in results))

    def test_tool_offset_disagreement_does_not_silently_pass(self):
        scene=Scene(self.vertices,self.faces)
        models={'urdf':{'points_tcp':np.array([[0,0,.1]])},
                'checker_defaults':{'points_tcp':np.array([[0,0,-.016]])}}
        result=evaluate_candidates([self.target()],scene,self.lib,models,SETTINGS)[0]
        self.assertTrue(result['tcp_model_disagreement'])
        self.assertIn('sampled_extruder_approach_conflict',result['rejection_reasons'])

    def test_path_is_checked_between_endpoints(self):
        scene=Scene(self.vertices,self.faces)
        path=interpolate_path([[0,0,.02],[0,0,-.02]],.001)
        check=swept_clearance(scene,[[0,0,0]],path)
        self.assertEqual(check['screen'],'conflict')
        self.assertFalse(check['continuous_collision_verified'])
        self.assertFalse(check['inside_outside_verified'])

    def test_surface_fit_uses_scene_and_does_not_change_other_candidates(self):
        scene=Scene(self.vertices,self.faces)
        records=[self.target(z=.03),self.target('B',x=.2,z=.03)]
        result=seat_targets(records,['A'],scene,self.lib)
        self.assertAlmostEqual(result[0]['top_m'][2],.016,places=6)
        self.assertEqual(result[1],records[1]);self.assertEqual(records[0]['top_m'][2],.03)

    def test_no_push_proposal_for_zero_control(self):
        scene=Scene(self.vertices,self.faces)
        result=propose_contact(scene,scene,deform,0,self.lib,self.models,SETTINGS)
        self.assertEqual(result['proposals'],[]);self.assertIsNone(result['selected_id'])
        self.assertFalse(result['executable'])

    def test_local_search_can_find_support_without_relaxing_gap(self):
        v=np.array([[0,-1,0],[.04,-1,0],[.04,1,0],[0,1,0]],float)
        scene=Scene(v,self.faces)
        original=self.target(x=-.004,z=.03)
        records=search_supported_targets([original],['A'],scene,self.lib,SETTINGS)
        self.assertLessEqual(np.linalg.norm(np.array(records[0]['top_m'][:2])-original['top_m'][:2]),.008+1e-12)
        check=evaluate_candidates(records,scene,self.lib,self.models,SETTINGS)[0]
        self.assertGreaterEqual(check['support_rays_within_gap'],6)
        self.assertFalse(check['executable']);self.assertEqual(original['top_m'][0],-.004)


if __name__=='__main__':unittest.main()
