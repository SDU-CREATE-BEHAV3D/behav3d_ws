"""Exercise save/defaults/open/continue and orthographic views in real Qt."""
from dataclasses import asdict
from pathlib import Path
import os
import sys
import uuid
import numpy as np

native = "--native" in sys.argv
if not native:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
root, _ = paths()
from PySide6 import QtCore, QtWidgets
from workbench.ui import Workbench
from workbench.preferences import camera_settings, load_defaults

out = APP_ROOT / "workbench/outputs" / ("persistence-ui-" + uuid.uuid4().hex[:8])
out.mkdir(parents=True)
defaults = out / "defaults.json"
state = out / "advanced.b3dstate"
app = QtWidgets.QApplication([])
app.setQuitOnLastWindowClosed(False)
kwargs = dict(paths=dict(data_root=root, output_root=out), off_screen=not native, preferences_path=defaults)
w = Workbench(**kwargs)
w.setWindowTitle("BEHAV3D - Persistence and views QA")
w.controls["checkpoint_every"].setValue(0)
w.controls["voxel_mm"].setValue(3)
w.controls["surface_step"].setValue(2)
w.show()
phase = 0
saved_camera = None

def fail(result):
    print(result, flush=True)
    os._exit(1)

w.worker.failed.connect(fail)

def poll():
    global phase, saved_camera, w
    if w.busy or not w.initialized or w.preview_revision != w.settings_revision:
        return
    try:
        if phase == 0:
            w.controls["count"].setValue(2)
            w.goal_opacity.setValue(0.35)
            w.target_axis_length.setValue(17)
            w.front_button.click()
            np.testing.assert_allclose(w.plotter.renderer.camera.direction, (0, 1, 0), atol=1e-12)
            w.top_button.click()
            np.testing.assert_allclose(w.plotter.renderer.camera.direction, (0, 0, -1), atol=1e-12)
            saved_camera = camera_settings(w.plotter)
            phase = 1
        elif phase == 1:
            w.save_defaults_button.click()
            assert load_defaults(defaults)["parameters"]["count"] == 2
            w.accept_preview()
            phase = 2
        elif phase == 2 and w.latest["cycle"] == 1:
            w.submit("save_state", state_path=str(state), view=dict(display=asdict(w.display_settings()), camera=saved_camera))
            phase = 3
        elif phase == 3 and w.latest["job"]["action"] == "save_state":
            assert state.exists()
            w.goal_opacity.setValue(0.7)
            w.controls["count"].setValue(1)
            w.submit("new")
            phase = 4
        elif phase == 4 and w.latest["cycle"] == 0:
            # Even invalid current batch inputs must not block opening a save.
            w.controls["width_mode"].setCurrentText("field")
            w.submit("open_state", state_path=str(state))
            phase = 5
        elif phase == 5 and w.latest["cycle"] == 1:
            assert w.parameters().count == 2 and w.parameters().width_mode == "fixed"
            assert abs(w.goal_opacity.value() - 0.35) < 1e-12
            assert w.target_axis_length.value() == 17
            assert w.options().voxel_mm == 3 and w.options().checkpoint_every == 0
            assert camera_settings(w.plotter) == saved_camera
            assert len(w.latest["target_history_points"]) == 4
            assert w.accept_button.isEnabled()
            if native:
                w.front_button.click()
                w.plotter.render_window.MakeCurrent()
                w.plotter.screenshot(str(out / "front.png"))
                w.top_button.click()
                w.plotter.render_window.MakeCurrent()
                w.plotter.screenshot(str(out / "top.png"))
            w.accept_preview()
            phase = 6
        elif phase == 6 and w.latest["cycle"] == 2:
            w.close()
            w = Workbench(**kwargs)
            w.worker.failed.connect(fail)
            w.show()
            phase = 7
        elif phase == 7:
            assert w.latest["cycle"] == 0
            assert w.parameters().count == 2
            assert w.target_axis_length.value() == 17
            assert abs(w.goal_opacity.value() - 0.35) < 1e-12
            assert camera_settings(w.plotter) == saved_camera
            print("PASS: defaults across windows, save/open/continue, restored history/controls/camera, Front/Top views; " + str(out), flush=True)
            phase = 8
            timer.stop()
            w.close()
            app.quit()
    except Exception:
        import traceback
        traceback.print_exc()
        os._exit(1)

timer = QtCore.QTimer()
timer.timeout.connect(poll)
timer.start(100)
QtCore.QTimer.singleShot(180000, lambda: os._exit(2))
app.exec()
assert phase == 8
