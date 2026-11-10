"""Camera snapshots and world-Z orbit gestures for the retained VTK viewport."""
from vtkmodules.vtkRenderingCore import vtkCamera


def capture_camera(plotter):
    # Plotter.camera is not a passive getter: it resets a camera whose is_set
    # flag is false. Mouse input operates on VTK directly and can leave it false.
    snapshot = vtkCamera()
    snapshot.DeepCopy(plotter.renderer.camera)
    return snapshot


def restore_camera(plotter, snapshot):
    camera = plotter.renderer.camera
    camera.DeepCopy(snapshot)
    camera.is_set = True
    plotter.reset_camera_clipping_range()


def configure_navigation(plotter, convention):
    if plotter.iren is None:
        return
    if convention == "VTK":
        plotter.enable_trackball_style()
        return
    if convention != "Rhino":
        raise ValueError(f"Unknown camera controls: {convention}")
    # Terrain rotation preserves a natural up vector and prevents crossing the
    # poles, unlike trackball rotation which progressively rolls the horizon.
    plotter.enable_terrain_style(mouse_wheel_zooms=True, shift_pans=True)
    style = plotter.iren.style
    interactor = plotter.iren.interactor

    def press(_style, _event):
        control, shift = interactor.GetControlKey(), interactor.GetShiftKey()
        camera = plotter.renderer.camera
        camera.is_set = True
        if control and not shift:
            style.OnRightButtonDown()
        elif shift and not control:
            style.OnMiddleButtonDown()
        else:
            # A top view points exactly down Z. Tilt off the pole before
            # selecting world Z as the up vector, avoiding a degenerate basis.
            if abs(camera.GetDirectionOfProjection()[2]) > 0.99985:
                camera.Elevation(1.0)
            camera.SetViewUp(0, 0, 1)
            style.OnLeftButtonDown()

    def release(_style, _event):
        # Match the native release handler to the action so VTK also releases
        # its mouse focus. Ending the state first would skip that cleanup.
        state = style.GetState()
        if state == 1:
            style.OnLeftButtonUp()
        elif state == 2:
            style.OnMiddleButtonUp()
        elif state == 4:
            style.OnRightButtonUp()

    style.add_observer("RightButtonPressEvent", press)
    style.add_observer("RightButtonReleaseEvent", release)
