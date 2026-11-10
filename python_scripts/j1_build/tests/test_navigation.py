"""Exercise the actual VTK mouse states and camera changes without Qt."""
from pathlib import Path
import sys
import unittest

import numpy as np
import pyvista as pv

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workbench.navigation import capture_camera, configure_navigation, restore_camera


class NavigationTests(unittest.TestCase):
    def test_orbit_can_leave_an_exact_top_view_without_a_degenerate_basis(self):
        plotter = pv.Plotter(off_screen=True, window_size=(500, 400))
        try:
            plotter.add_mesh(pv.Cube())
            plotter.enable_parallel_projection()
            plotter.view_xy()
            plotter.show(auto_close=False, interactive=False)
            configure_navigation(plotter, "Rhino")
            iren = plotter.iren.interactor
            iren.SetEventInformation(250, 200, 0, 0)
            iren.InvokeEvent("RightButtonPressEvent")
            iren.SetEventInformation(290, 225, 0, 0)
            iren.InvokeEvent("MouseMoveEvent")
            iren.InvokeEvent("RightButtonReleaseEvent")
            c = plotter.renderer.camera
            self.assertTrue(np.isfinite(c.position).all())
            self.assertGreater(np.linalg.norm(np.cross(c.direction, c.up)), 0.015)
        finally:
            plotter.close()

    def test_snapshot_does_not_trigger_pyvista_lazy_camera_reset(self):
        plotter = pv.Plotter(off_screen=True)
        try:
            plotter.add_mesh(pv.Cube())
            plotter.enable_parallel_projection()
            plotter.view_isometric()
            raw = plotter.renderer.camera
            self.assertFalse(raw.is_set)
            raw.Azimuth(37)
            raw.Elevation(15)
            raw.SetParallelScale(raw.GetParallelScale() * 0.61)
            expected = (raw.GetPosition(), raw.GetFocalPoint(), raw.GetViewUp(), raw.GetParallelScale())
            snapshot = capture_camera(plotter)
            self.assertEqual(snapshot.GetPosition(), expected[0])
            self.assertFalse(raw.is_set, "Capturing must not invoke the lazy Plotter.camera getter")
            raw.Azimuth(-70)
            restore_camera(plotter, snapshot)
            # Deliberately access the getter afterwards to verify it no longer
            # reinitializes the restored native-mouse view.
            camera = plotter.camera
            self.assertEqual((camera.position, camera.focal_point, camera.up, camera.parallel_scale), expected)
            self.assertTrue(camera.is_set)
        finally:
            plotter.close()

    def test_right_drag_gestures_and_switch_back_to_vtk(self):
        plotter = pv.Plotter(off_screen=True, window_size=(500, 400))
        try:
            plotter.add_mesh(pv.Cube())
            plotter.enable_parallel_projection()
            plotter.view_isometric()
            plotter.show(auto_close=False, interactive=False)
            configure_navigation(plotter, "Rhino")
            iren = plotter.iren.interactor
            style = plotter.iren.style
            for control, shift, state in [(0, 0, 1), (0, 1, 2), (1, 0, 4), (1, 1, 1)]:
                before = plotter.camera.copy()
                iren.SetEventInformation(250, 200, control, shift)
                iren.InvokeEvent("RightButtonPressEvent")
                self.assertEqual(style.GetState(), state)
                iren.SetEventInformation(280, 220, control, shift)
                iren.InvokeEvent("MouseMoveEvent")
                iren.InvokeEvent("RightButtonReleaseEvent")
                self.assertEqual(style.GetState(), 0)
                if state == 1:
                    self.assertFalse(np.allclose(before.position, plotter.camera.position))
                    np.testing.assert_array_equal(plotter.renderer.camera.up, (0, 0, 1))
                elif state == 2:
                    self.assertFalse(np.allclose(before.focal_point, plotter.camera.focal_point))
                    np.testing.assert_allclose(np.subtract(before.position, before.focal_point),
                        np.subtract(plotter.camera.position, plotter.camera.focal_point))
                else:
                    self.assertNotEqual(before.parallel_scale, plotter.camera.parallel_scale)
            camera = plotter.renderer.camera
            focal_point, distance = camera.focal_point, camera.distance
            # Repeated diagonal drags used to progressively tilt the horizon.
            for _ in range(20):
                iren.SetEventInformation(240, 190, 0, 0)
                iren.InvokeEvent("RightButtonPressEvent")
                iren.SetEventInformation(300, 240, 0, 0)
                iren.InvokeEvent("MouseMoveEvent")
                iren.InvokeEvent("RightButtonReleaseEvent")
                np.testing.assert_array_equal(camera.up, (0, 0, 1))
                np.testing.assert_allclose(camera.focal_point, focal_point, atol=1e-12)
                self.assertAlmostEqual(camera.distance, distance, places=12)
                direction = np.array(camera.direction)
                right = np.cross(direction, camera.up)
                self.assertGreater(np.linalg.norm(right), 0.015)
            scale = camera.parallel_scale
            iren.InvokeEvent("MouseWheelForwardEvent")
            self.assertLess(camera.parallel_scale, scale)
            configure_navigation(plotter, "VTK")
            iren.SetEventInformation(250, 200, 0, 0)
            iren.InvokeEvent("RightButtonPressEvent")
            self.assertEqual(plotter.iren.style.GetState(), 4)
            iren.InvokeEvent("RightButtonReleaseEvent")
        finally:
            plotter.close()
