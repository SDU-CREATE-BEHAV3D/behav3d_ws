"""Check conditional width controls, goal replacement and camera persistence."""
from pathlib import Path
import os
import sys
import uuid
import numpy as np

os.environ["QT_QPA_PLATFORM"] = "offscreen"
APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
root, _ = paths()
from PySide6 import QtCore, QtWidgets
from workbench.ui import Workbench

out = APP_ROOT / "workbench/outputs" / ("width-ui-" + uuid.uuid4().hex[:8])
out.mkdir(parents=True)
with np.load(root / "mesh/fields/field_state_init.npz", allow_pickle=False) as data:
    count = len(data["heat_norm"])
width_map = out / "width.npz"
np.savez_compressed(width_map, width_norm=np.linspace(0, 1, count))
app = QtWidgets.QApplication([])
w = Workbench(dict(data_root=root, output_root=out), off_screen=True, preferences_path=out / "defaults.json")
w.controls["checkpoint_every"].setValue(0)
w.show()
phase = 0
saved = None
initial_count = None

def camera():
    c = w.plotter.renderer.camera
    return c.position, c.focal_point, c.up, c.parallel_scale

def poll():
    global phase, saved, initial_count
    if w.busy or not w.initialized or w.preview_revision != w.settings_revision:
        return
    try:
        if phase == 0:
            initial_count = w.meshes["goal"].n_points
            assert w.fixed_width_group.isEnabled() and not w.field_width_group.isEnabled()
            w.plotter.renderer.camera.Azimuth(27)
            saved = camera()
            w.width_field_input.setText(str(width_map))
            w.controls["width_mode"].setCurrentText("field")
            assert not w.accept_button.isEnabled()
            assert w.field_width_group.isEnabled() and not w.fixed_width_group.isEnabled()
            assert "FIELD ACTIVE" in w.width_note.text()
            phase = 1
        elif phase == 1:
            assert w.meshes["goal"].n_points == count < initial_count
            assert camera() == saved
            assert not w.preview.phantom_surface.is_empty
            assert len(w.preview.bead_widths_m) == w.preview.targets.count
            w.accept_preview()
            phase = 2
        elif phase == 2 and w.latest["cycle"] == 1:
            assert camera() == saved
            assert w.latest["history"][0]["inputs"]["width_mode"] == "field"
            w.controls["width_mode"].setCurrentText("fixed")
            phase = 3
        elif phase == 3:
            assert w.meshes["goal"].n_points == initial_count
            assert w.latest["cycle"] == 1
            assert "deposits" in w.actors
            assert camera() == saved
            print("PASS: conditional width inputs, original goal in field mode, phantom, accept and mode switches preserve camera/deposits", flush=True)
            phase = 4
            timer.stop()
            w.close()
            app.quit()
    except Exception:
        import traceback
        traceback.print_exc()
        os._exit(1)

w.worker.failed.connect(lambda result: (print(result, flush=True), os._exit(1)))
timer = QtCore.QTimer()
timer.timeout.connect(poll)
timer.start(100)
QtCore.QTimer.singleShot(120000, lambda: os._exit(2))
app.exec()
assert phase == 4
