import unittest
import numpy as np
from visibility import MeshRays, classify, project, evaluate


class VisibilityTests(unittest.TestCase):
    def setUp(self):
        self.pose={'index':0,'position_m':[0,0,1],'optical_axes_world':[[1,0,0],[0,-1,0],[0,0,-1]]}
        self.k={'fx':100,'fy':100,'cx':100,'cy':100,'width':200,'height':200}
        self.target=[0,0,0]
        self.floor=np.array([[[-1,-1,0],[1,-1,0],[1,1,0]],[[-1,-1,0],[1,1,0],[-1,1,0]]])

    def test_exposed_target_supported(self):
        self.assertEqual(classify(self.target,self.pose,self.k,MeshRays(self.floor),.004)[0],'endpoint_supported_in_mesh')

    def test_ridge_blocks_top_but_side_view_resolves(self):
        ridge=np.array([[[-.1,-.2,.5],[.1,-.2,.5],[.1,.2,.5]],[[-.1,-.2,.5],[.1,.2,.5],[-.1,.2,.5]]])
        rays=MeshRays(np.concatenate([self.floor,ridge]))
        self.assertEqual(classify(self.target,self.pose,self.k,rays,.004)[0],'blocked_in_mesh')
        from pathlib import Path
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/behav3d_orchestrator'))
        from behav3d_orchestrator.offline_agent.inspection import basis
        side=dict(self.pose,position_m=[1,0,1],optical_axes_world=basis([-1,0,-1]))
        self.assertEqual(classify(self.target,side,self.k,rays,.004)[0],'endpoint_supported_in_mesh')

    def test_empty_or_missing_geometry_stays_unknown(self):
        rays=MeshRays(np.empty((0,3,3)))
        self.assertEqual(classify(self.target,self.pose,self.k,rays,.004)[0],'unknown_mesh_support')
        self.assertEqual(evaluate([self.pose],[self.target],self.k,rays)['endpoint_supported_in_at_least_one_view'],0)

    def test_behind_and_outside_image(self):
        self.assertIsNone(project([0,0,2],self.pose,self.k))
        self.assertEqual(classify([5,0,0],self.pose,self.k,MeshRays(self.floor),.004)[0],'outside_image')

    def test_reversed_triangle_winding_still_blocks(self):
        self.assertAlmostEqual(MeshRays(self.floor[:,::-1]).first_hit([0,0,1],[0,0,-1]),1)

    def test_endpoint_tolerance_sensitivity(self):
        rays=MeshRays(self.floor)
        self.assertEqual(classify([0,0,-.003],self.pose,self.k,rays,.004)[0],'endpoint_supported_in_mesh')
        self.assertEqual(classify([0,0,-.003],self.pose,self.k,rays,.002)[0],'blocked_in_mesh')

    def test_bad_inputs(self):
        for t in (0,-1,float('nan')):
            with self.assertRaises(ValueError): classify(self.target,self.pose,self.k,MeshRays(self.floor),t)
        with self.assertRaises(ValueError): project(self.target,dict(self.pose,optical_axes_world=np.zeros((3,3))),self.k)


if __name__=='__main__': unittest.main()
