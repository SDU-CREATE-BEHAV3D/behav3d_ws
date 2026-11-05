"""Exercise the real Qt panels and worker, with no visible desktop window."""
from pathlib import Path
import os
import sys
os.environ["QT_QPA_PLATFORM"] = "offscreen"
app_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(app_root))
from workbench.app import paths
root, _ = paths()
if not (root / "mesh/fields/field_state_init_subdiv1.npz").exists():
    from prepare_field import prepare_field
    prepare_field(root)
os.environ["MPLCONFIGDIR"] = str(app_root / "workbench/.cache/matplotlib")
from PySide6 import QtCore, QtWidgets
from workbench.ui import Workbench

app = QtWidgets.QApplication([])
window = Workbench(dict(data_root=root, output_root=app_root / "workbench/outputs"), off_screen=True)
window.controls["voxel_mm"].setValue(3)
window.controls["surface_step"].setValue(2)
window.controls["checkpoint_every"].setValue(0)
window.show()
phase = 0
ticks = 0
errors = []
static_actor = None

def heartbeat():
    global ticks
    ticks += 1

def fail(result):
    errors.append(result)
    print(result, flush=True)

window.worker.failed.connect(fail)
timer = QtCore.QTimer()
timer.timeout.connect(heartbeat)
timer.start(50)

def poll():
    global phase, static_actor
    try:
        if errors:
            raise AssertionError(errors)
        if window.busy or not window.initialized or window.preview_revision != window.settings_revision:
            return
        if phase == 0:
            assert len(window.plotter.renderer.lights) == 3
            from workbench.config import Parameters, SimulationOptions, DisplayOptions
            assert window.parameters() == Parameters()
            assert window.options() == SimulationOptions(voxel_mm=3, surface_step=2, checkpoint_every=0)
            assert window.goal_opacity.value() == DisplayOptions().goal_opacity
            assert 0 < window.preview.targets.count <= Parameters().count
            assert "phantom" in window.actors
            assert window.layers["phantom"].isChecked()
            assert not window.preview.phantom_surface.is_empty
            assert not window.layers["axes"].isChecked()
            static_actor = window.actors["goal"]
            window.controls["source_spacing_mm"].setValue(2)
            assert not window.accept_button.isEnabled()
            phase = 1
        elif phase == 1:
            assert len(window.preview.candidates.sampling_points) > 202
            assert window.actors["goal"] is static_actor
            assert window.preview.geometry_cached
            window.accept_button.click()
            phase = 2
        elif phase == 2 and window.latest["cycle"] == 1:
            assert window.latest["history"][0]["parameters"]["source_spacing_mm"] == 2
            settings_revision = window.settings_revision
            preview_token = window.preview.token
            assert not window.actors["goal"].prop.lighting
            window.light_intensity.setValue(1.5)
            from workbench.config import DisplayOptions
            expected_light = DisplayOptions().key_light_intensity * 1.5
            assert abs(window.plotter.renderer.lights[0].intensity - expected_light) < 1e-12
            assert not window.actors["goal"].prop.lighting
            window.goal_lighting.setChecked(True)
            assert window.actors["goal"].prop.lighting
            window.goal_lighting.setChecked(False)
            window.light_intensity.setValue(DisplayOptions().light_intensity)
            assert window.bead_material.currentText() == "Warm white"
            from workbench.appearance import PHANTOM_COLOR
            import pyvista as pv
            assert window.actors["phantom"].prop.color == pv.Color(PHANTOM_COLOR)
            window.contact_shading.setChecked(False)
            window.contact_shading.setChecked(True)
            window.phantom_opacity.setValue(0.25)
            assert abs(window.actors["phantom"].GetProperty().GetOpacity() - 0.25) < 1e-12
            window.layers["phantom"].setChecked(False)
            assert not window.actors["phantom"].GetVisibility()
            window.layers["phantom"].setChecked(True)
            assert window.controls["width_mm"].locale().name() == "en_US"
            window.layers["goal_edges"].setChecked(True)
            window.layers["bead_edges"].setChecked(True)
            window.edge_opacity["goal_edges"].setValue(0.08)
            window.edge_opacity["bead_edges"].setValue(0.42)
            assert abs(window.actors["goal_edges"].GetProperty().GetOpacity() - 0.08) < 1e-12
            assert abs(window.actors["bead_edges"].GetProperty().GetOpacity() - 0.42) < 1e-12
            window.layers["goal"].setChecked(False)
            assert not window.actors["goal"].GetVisibility()
            assert window.actors["goal_edges"].GetVisibility()
            window.layers["goal"].setChecked(True)
            window.bead_material.setCurrentText("Graphite")
            assert window.settings_revision == settings_revision
            assert window.preview.token == preview_token
            assert not window.busy
            window.run_count.setValue(2)
            window.run_button.click()
            phase = 3
        elif phase == 3 and window.latest["cycle"] == 3 and not window.running:
            assert len(window.plotter.renderer.lights) == 3
            assert window.actors["goal"] is static_actor
            assert window.actors["bead_edges"].GetVisibility()
            assert window.edge_sources["bead_edges"] is window.meshes["deposits"]
            assert abs(window.actors["bead_edges"].GetProperty().GetOpacity() - 0.42) < 1e-12
            assert ticks > 5
            window.export_button.click()
            phase = 4
        elif phase == 4 and window.latest["job"]["action"] == "export":
            out = app_root / "workbench/outputs/ui-check"
            out.mkdir(exist_ok=True)
            # Qt's offscreen platform has no native OpenGL context here.
            # Native rendering and viewport export are tested separately via CUA.
            window.grab().save(str(out / "window.png"))
            assert (window.latest["extra"] / "simulation.npz").is_file()
            assert len(window.latest["history"]) == 3
            print(f"PASS: live sampling, stale-preview guard, accept/run/pause/export, independent edge layers and opacities, display-only material changes; {ticks} UI heartbeats", flush=True)
            print(f"OUTPUT: {window.latest['output']}", flush=True)
            phase = 5
            poller.stop()
            window.close()
            app.quit()
    except Exception:
        import traceback
        traceback.print_exc()
        os._exit(1)

poller = QtCore.QTimer()
poller.timeout.connect(poll)
poller.start(100)
QtCore.QTimer.singleShot(180000, lambda: os._exit(2))
app.exec()
assert phase == 5
