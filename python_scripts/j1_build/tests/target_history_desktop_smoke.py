"""Check imported/accepted/preview frames are independent display-only layers."""
from pathlib import Path
import os
import sys
import uuid
import numpy as np
import yaml

os.environ["QT_QPA_PLATFORM"] = "offscreen"
APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
from workbench.app import paths
root, _ = paths()
from PySide6 import QtCore, QtWidgets
from workbench.ui import Workbench
from lib_scalar.print_targets import OrientedLineTargets, write_line_targets_yaml

out = APP_ROOT / "workbench/outputs" / ("target-ui-" + uuid.uuid4().hex[:8])
out.mkdir(parents=True)
source = out / "previous_targets.yaml"
points = np.array([[0.1, -0.8, 0], [0.15, -0.8, 0]])
z_dirs = np.array([[0, 0, 1], [0.2, 0, 0.98]])
write_line_targets_yaml(source, OrientedLineTargets(points, points + 0.005, z_dirs, z_dirs), position_scale=1000)
source_bytes = source.read_bytes()
app = QtWidgets.QApplication([])
w = Workbench(dict(data_root=root, output_root=out), off_screen=True)
w.controls["checkpoint_every"].setValue(0)
w.show()
phase = 0
saved_camera = saved_token = old_output = None

def camera():
    c = w.plotter.renderer.camera
    return c.position, c.focal_point, c.up, c.parallel_scale

def poll():
    global phase, saved_camera, saved_token, old_output
    if w.busy or not w.initialized or w.preview_revision != w.settings_revision:
        return
    try:
        if phase == 0:
            saved_camera, saved_token = camera(), w.preview.token
            w.submit("load_target_yaml", target_yaml_path=str(source))
            phase = 1
        elif phase == 1:
            assert len(w.loaded_targets["points"]) == 4
            assert w.latest["cycle"] == 0 and camera() == saved_camera
            assert w.preview.token == saved_token
            assert yaml.safe_load((w.latest["output"] / "targets_history.yaml").read_text())["segments"] == []
            assert all(w.actors[f"target_imported_{a}"].GetVisibility() for a in "xyz")
            assert not w.target_starts.isChecked() and w.target_endpoints.isChecked()
            assert w.target_axis_thickness.value() == 1.4
            end_vertex_count = w.actors["target_imported_x"].mapper.dataset.n_points
            w.target_starts.setChecked(True)
            assert w.actors["target_imported_x"].mapper.dataset.n_points == 2 * end_vertex_count
            w.target_endpoints.setChecked(False)
            assert w.actors["target_imported_x"].mapper.dataset.n_points == end_vertex_count
            w.target_starts.setChecked(False)
            assert "target_imported_x" not in w.actors
            w.target_endpoints.setChecked(True)
            w.target_axis_thickness.setValue(1.8)
            assert w.frame_sources["target_imported"][-1] == 1.8
            w.target_axis_thickness.setValue(1.4)
            assert w.preview.token == saved_token and camera() == saved_camera
            w.target_axis_length.setValue(20)
            assert w.frame_sources["target_imported"][2] == 20
            w.layers["target_imported"].setChecked(False)
            assert not w.actors["target_imported_x"].GetVisibility()
            w.target_axis_length.setValue(14)
            w.layers["target_imported"].setChecked(True)
            assert w.frame_sources["target_imported"][2] == 14
            w.accept_preview()
            phase = 2
        elif phase == 2 and w.latest["cycle"] == 1:
            assert all(w.actors[f"target_history_{a}"].GetVisibility() for a in "xyz")
            assert all(w.actors[f"target_preview_{a}"].GetVisibility() for a in "xyz")
            assert len(w.loaded_targets["points"]) == 4
            old_output = w.latest["output"]
            assert len(yaml.safe_load((old_output / "targets_history.yaml").read_text())["segments"]) > 0
            w.submit("new")
            phase = 3
        elif phase == 3 and w.latest["cycle"] == 0:
            assert w.latest["output"] != old_output
            assert len(w.latest["target_history_points"]) == 0
            assert "target_history_x" not in w.actors
            assert w.actors["target_imported_x"].GetVisibility()
            assert source.read_bytes() == source_bytes
            assert len(yaml.safe_load((old_output / "targets_history.yaml").read_text())["segments"]) > 0
            print("PASS: YAML loading preserves simulation, preview and camera; independent frame layers/length; new-session history reset", flush=True)
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
