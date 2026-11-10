"""Verify three native viewport captures and camera restoration on failure."""
from pathlib import Path
import os
import sys
import uuid
from unittest.mock import patch

import numpy as np

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))
os.environ["MPLCONFIGDIR"] = str(APP_ROOT / "workbench/.cache/matplotlib")
from workbench.app import paths
root, _ = paths()
from PySide6 import QtCore, QtGui, QtWidgets
from workbench.ui import Workbench
from workbench.preferences import camera_settings

out = APP_ROOT / "workbench/outputs" / ("captures-ui-" + uuid.uuid4().hex[:8])
app = QtWidgets.QApplication([])
w = Workbench(dict(data_root=root, output_root=out), preferences_path=out / "defaults.json")
w.setWindowTitle("BEHAV3D - Capture views QA")
w.controls["count"].setValue(2)
w.controls["checkpoint_every"].setValue(0)
w.show()
phase = 0


def fail(result):
    print(result, flush=True)
    os._exit(1)


w.worker.failed.connect(fail)


def poll():
    global phase
    if w.busy or not w.initialized or w.preview_revision != w.settings_revision:
        return
    try:
        if phase == 0:
            assert w.captures_button.isEnabled()
            assert w.layers["deposits"].text() == "Bead surface"
            assert list(w.layers).index("bead_edges") == list(w.layers).index("deposits") + 1
            w.accept_preview()
            assert not w.captures_button.isEnabled()
            phase = 1
            return
        camera = w.plotter.renderer.camera
        camera.Azimuth(27)
        camera.Elevation(13)
        camera.SetParallelScale(camera.GetParallelScale() * 0.8)
        before = camera_settings(w.plotter)
        layers = {key: widget.isChecked() for key, widget in w.layers.items()}
        directions = []
        screenshot = w.plotter.screenshot

        def capture(*args, **kwargs):
            directions.append(tuple(w.plotter.renderer.camera.direction))
            return screenshot(*args, **kwargs)

        with patch.object(w.plotter, "screenshot", side_effect=capture):
            w.captures_button.click()
        folder = next((w.latest["output"] / "captures").iterdir())
        assert {p.name for p in folder.iterdir()} == {"paper.png", "front.png", "top.png"}
        for path in folder.iterdir():
            image = QtGui.QImage(str(path))
            assert not image.isNull() and image.width() > 100 and image.height() > 100
        np.testing.assert_allclose(directions[1], (0, 1, 0), atol=1e-12)
        np.testing.assert_allclose(directions[2], (0, 0, -1), atol=1e-12)
        assert camera_settings(w.plotter) == before
        assert layers == {key: widget.isChecked() for key, widget in w.layers.items()}
        assert w.latest["cycle"] == 1
        again = w.save_captures()
        assert again != folder and (again / "paper.png").is_file()
        with patch.object(w.plotter, "screenshot", side_effect=OSError("Capture QA write failure")):
            assert w.save_captures() is None
        assert camera_settings(w.plotter) == before
        assert "Capture QA write failure" in w.log.toPlainText()
        assert w.captures_button.isEnabled()
        print("PASS: three PNG views, native rendering, layer order, unique sets, camera/layer preservation and error recovery", flush=True)
        print(f"OUTPUT: {folder}", flush=True)
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
QtCore.QTimer.singleShot(120000, lambda: os._exit(2))
app.exec()
