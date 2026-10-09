import copy
import unittest

import numpy as np

from geometry import (NominalBead, adapt_targets, bead, centers, evidence_decision,
                      footprint_gaps, lean, priority_order, repair_probe, union_contains,
                      split_distance_evidence)


CONFIG = {'x_start_m': .1, 'x_full_m': .2, 'y_center_m': 0.,
          'y_inner_m': .1, 'y_outer_m': .2, 'pivot_z_m': 0.}


class RecoveryGeometryTests(unittest.TestCase):
    def test_zero_lean_is_exact_control_and_input_is_unchanged(self):
        points = np.array([[.3, 0, .1], [.15, .15, .06], [.3, 0, -.1]])
        original = points.copy()
        np.testing.assert_array_equal(lean(points, 0, CONFIG), original)
        lean(points, 12, CONFIG)
        np.testing.assert_array_equal(points, original)

    def test_anchor_and_neighbors_stay_fixed(self):
        points = np.array([[.3, 0, 0], [.3, 0, -.01], [.09, 0, .1], [.3, .21, .1]])
        np.testing.assert_array_equal(lean(points, 12, CONFIG), points)

    def test_full_weight_vertical_line_has_declared_lean(self):
        points = np.array([[.3, 0, .02], [.3, 0, .12]])
        for angle in (4, 8, 12):
            a, b = lean(points, angle, CONFIG)
            measured = np.degrees(np.arctan2(b[0]-a[0], b[2]-a[2]))
            self.assertAlmostEqual(measured, angle)
            np.testing.assert_array_equal(np.array([a,b])[:,1:], points[:,1:])

    def test_invalid_lean_is_rejected(self):
        for angle in (float('nan'), 25):
            with self.assertRaises(ValueError):
                lean(np.array([[.3,0,.1]]), angle, CONFIG)

    def test_adaptation_preserves_dimensions_and_attaches_base(self):
        original = NominalBead('A', (.3,0,.1), (0,0,1), .036,.016).record()
        before = copy.deepcopy(original)
        adapted = adapt_targets([original], 8, CONFIG)[0]
        b = bead(adapted)
        np.testing.assert_allclose(np.array(b.top)-b.height_m*b.direction,
                                   lean(np.array([[.3,0,.084]]),8,CONFIG)[0], atol=1e-12)
        self.assertEqual((b.width_m,b.height_m),(.036,.016))
        self.assertEqual(original,before)

    def test_voxel_coordinates_are_centers_and_keep_frame(self):
        p = centers([0,1,2], {'origin_m':[.1,-.9,-.2], 'voxel_size_m':[.002]*3})
        np.testing.assert_allclose(p,[[.101,-.897,-.195]])

    def test_rounded_distance_boundary_is_not_invented_missing_evidence(self):
        missing,boundary=split_distance_evidence([9.9,10,10,10.1],10,2)
        np.testing.assert_array_equal(missing,[False,False,False,True])
        np.testing.assert_array_equal(boundary,[False,True,True,False])
        with self.assertRaises(ValueError): split_distance_evidence([9.9,10,10,10.1],10,4)

    def test_priority_is_only_a_reordering(self):
        a = NominalBead('A', (.3,0,.1),(0,0,1),.036,.016).record()
        b = NominalBead('B', (.1,0,.1),(0,0,1),.036,.016).record()
        rows=[b,a]; before=copy.deepcopy(rows)
        result=priority_order(rows,np.array([[.3,0,.092]]))
        self.assertEqual([r['label'] for r in result],['A','B'])
        self.assertEqual(rows,before)
        self.assertEqual(result[0],a)

    def test_empty_deposition_cannot_claim_reach(self):
        self.assertFalse(union_contains(np.array([[0,0,0]]),[]).any())

    def test_probe_needs_evidence_and_keeps_issued_dimensions(self):
        original=NominalBead('A',(.3,0,.1),(0,0,1),.036,.016).record()
        with self.assertRaises(ValueError): repair_probe([],original)
        points=np.array([[.31,0,.08],[.31,0,.084]])
        probe=bead(repair_probe(points,original))
        np.testing.assert_allclose(probe.center,points.mean(0))
        self.assertEqual((probe.width_m,probe.height_m),(.036,.016))

    def test_whole_mesh_ray_diagnostic_distinguishes_buried_and_clear(self):
        triangles=np.array([[[-1,-1,0],[1,-1,0],[1,1,0]],[[-1,-1,0],[1,1,0],[-1,1,0]]],float)
        records=[NominalBead('above',(.1,0,.03),(0,0,1),.036,.016).record(),
                 NominalBead('below',(.1,0,.01),(0,0,1),.036,.016).record(),
                 NominalBead('unknown',(2,0,.03),(0,0,1),.036,.016).record()]
        rows=footprint_gaps(records,triangles)
        self.assertAlmostEqual(rows[0]['base_to_upper_surface_mm'],14)
        self.assertAlmostEqual(rows[1]['base_to_upper_surface_mm'],-6)
        self.assertIsNone(rows[2]['base_to_upper_surface_mm'])

    def test_uncertain_evidence_cannot_become_recovery_clearance(self):
        self.assertEqual(evidence_decision(identity_supported=False,tolerance_sensitive=False,observation_fraction=1),
                         'review_evidence_before_recovery')
        self.assertEqual(evidence_decision(identity_supported=True,tolerance_sensitive=False,observation_fraction=.3),
                         'review_evidence_before_recovery')
        self.assertEqual(evidence_decision(identity_supported=True,tolerance_sensitive=True,observation_fraction=.94),
                         'compare_recovery_hypotheses_keep_inspection_open')


if __name__=='__main__': unittest.main()
