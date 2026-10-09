import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.offline_agent.inspection import generate_patterns, dot, cross
from behav3d_orchestrator.offline_agent import ContractError


class InspectionTests(unittest.TestCase):
    def setUp(self):
        self.args = dict(focus_m=[.04,-.86,-.11], frame='ur20_base_link', units='m',
                         distance_m=.4, width_m=.12, height_m=.12, nx=3, ny=3,
                         samples=9, cap_deg=60, outward_axis=[.24,-.02,.97])

    def test_deterministic_and_no_execution(self):
        a = generate_patterns(**self.args)
        self.assertEqual(a, generate_patterns(**self.args))
        self.assertEqual(a['robot_commands'], [])
        self.assertFalse(a['executable'])

    def test_grid_spacing_and_serpentine(self):
        grid = generate_patterns(**self.args)['grid']
        self.assertEqual(len(grid), 9)
        self.assertAlmostEqual(grid[1]['position_m'][0]-grid[0]['position_m'][0], .06)
        self.assertAlmostEqual(grid[3]['position_m'][0], grid[2]['position_m'][0])
        self.assertLess(grid[4]['position_m'][0],grid[3]['position_m'][0])
        for p in grid:
            self.assertAlmostEqual(p['position_m'][2], .29)

    def test_cap_radius_bounds_and_look_at(self):
        plan = generate_patterns(**self.args)
        axis = self.args['outward_axis']; n = math.sqrt(dot(axis,axis)); axis = [v/n for v in axis]
        for p in plan['cap']:
            delta = [a-b for a,b in zip(p['position_m'],plan['focus_m'])]
            self.assertAlmostEqual(math.sqrt(dot(delta,delta)), .4)
            self.assertGreaterEqual(dot(delta,axis)/.4, .5-1e-12)
            x,y,z = p['optical_axes_world']
            self.assertAlmostEqual(dot(z,delta)/.4,-1)
            for v in (x,y,z): self.assertAlmostEqual(dot(v,v),1)
            self.assertAlmostEqual(dot(x,y),0)
            self.assertAlmostEqual(dot(cross(x,y),z),1)

    def test_invalid_requests(self):
        for key,value in [('frame',''),('units','mm'),('nx',1),('samples',True),
                          ('distance_m',-1),('cap_deg',91),('outward_axis',[0,0,0]),
                          ('focus_m',[float('nan'),0,0])]:
            with self.subTest(key=key), self.assertRaises(ContractError):
                generate_patterns(**dict(self.args, **{key:value}))
