import unittest
import numpy as np
from bead_surface import nominal_surface, surface_views
from visibility import MeshRays,evaluate


class BeadSurfaceTests(unittest.TestCase):
    def test_dimensions_normals_and_top_anchor(self):
        s=nominal_surface([0,0,0],[0,0,1],.036,.016)
        p=np.array(s['points_m']);n=np.array(s['normals'])
        self.assertTrue(np.allclose(np.linalg.norm(n,axis=1),1))
        self.assertLessEqual(np.max(np.linalg.norm(p[:,:2],axis=1)),.018)
        self.assertLessEqual(p[:,2].max(),0)
        self.assertGreaterEqual(p[:,2].min(),-.016)
        self.assertTrue(all(p[i,2]==0 for i,r in enumerate(s['regions']) if r=='top'))
        # Confirm sampled sides lie on the DDS rounded-cylinder zero isosurface.
        radial=np.linalg.norm(p[:,:2],axis=1);axial=p[:,2]+.008
        q=np.stack([radial-.018+.008,np.abs(axial)-.008+.008],axis=1)
        sdf=np.minimum(np.max(q,axis=1),0)+np.linalg.norm(np.maximum(q,0),axis=1)-.008
        self.assertTrue(np.allclose(sdf,0,atol=1e-10))

    def test_back_and_grazing_sides_are_not_counted(self):
        s={'points_m':[[0,0,0],[0,0,0]],'normals':[[0,0,-1],[1,0,0]],'regions':['side','side'],'area_weights_m2':[1,1]}
        pose={'index':0,'position_m':[0,0,1],'optical_axes_world':[[1,0,0],[0,-1,0],[0,0,-1]]}
        k={'fx':100,'fy':100,'cx':100,'cy':100,'width':200,'height':200}
        r=surface_views([pose],s,k,MeshRays(np.empty((0,3,3))),evaluate)
        self.assertEqual(r['regions']['side']['opportunity_area_fraction_once'],0)

    def test_bad_dimensions(self):
        with self.assertRaises(ValueError):nominal_surface([0,0,0],[0,0,1],-1,.01)
