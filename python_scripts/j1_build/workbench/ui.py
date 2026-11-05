"""Native Qt panels and the existing PyVista/DDS drawing stack."""
from dataclasses import asdict
import traceback

import numpy as np
import pyvista as pv
from PySide6 import QtCore, QtWidgets
from pyvistaqt import QtInteractor

from lib_scalar.viz_dds import attach_viewer, add_colored_point_cloud, add_line_segments
from .config import Parameters, SimulationOptions, DisplayOptions
from .session import Session
from .appearance import PHANTOM_COLOR, PHANTOM_OPACITY, SURFACE_MATERIAL, configure_studio_lighting, set_contact_shading


class Worker(QtCore.QObject):
    done = QtCore.Signal(object)
    failed = QtCore.Signal(object)

    def __init__(self, paths):
        super().__init__()
        self.paths = paths
        self.session = None

    @QtCore.Slot(object)
    def dispatch(self, job):
        try:
            action = job["action"]
            extra = None
            if action == "new":
                session = Session(**self.paths, options=job["options"])
                preview = session.prepare(job["parameters"], include_phantom=job.get("phantom", True))
                self.session = session
            elif action == "preview":
                preview = self.session.prepare(job["parameters"], include_phantom=job.get("phantom", True))
            elif action == "accept":
                extra = self.session.accept(job["token"])
                preview = None
            elif action == "export":
                extra = self.session.export()
                preview = self.session.preview
            elif action == "offset":
                extra = self.session.offset_diagnostic()
                preview = self.session.preview
            elif action == "phantom":
                extra = self.session.preview_phantom(job["token"])
                preview = self.session.preview
            else:
                raise ValueError(action)
            s = self.session
            self.done.emit({"job": job, "preview": preview, "extra": extra,
                "cycle": s.revision, "surface": s.surface,
                "field_vertices": s.field.field_vertices_scaled + np.asarray(s.field.offset_xyz),
                "field_faces": s.field.field_faces, "heat": s.field.heat_norm,
                "scan_vertices": np.asarray(s.base.vertices), "scan_faces": np.asarray(s.base.triangles),
                "last_targets": s.last_targets, "history": list(s.history), "output": s.output,
                "grid": s.simulator.domain.grid_shape if s.simulator else None,
                "geometry_builds": s.geometry_builds})
        except Exception as exc:
            self.failed.emit({"job": job, "error": str(exc), "traceback": traceback.format_exc()})


def polydata(vertices, faces):
    return pv.PolyData(vertices, np.c_[np.full(len(faces), 3), faces].ravel())


class Workbench(QtWidgets.QMainWindow):
    requested = QtCore.Signal(object)

    def __init__(self, paths, off_screen=False):
        super().__init__()
        self.setLocale(QtCore.QLocale("en_US"))
        self.setWindowTitle("BEHAV3D · J1 Workbench")
        self.resize(1440, 920)
        self.busy = False
        self.settings_revision = 0
        self.preview_revision = -1
        self.preview = self.latest = None
        self.running = False
        self.remaining = 0
        self.actors = {}
        self.meshes = {}
        self.edge_sources = {}
        self.initialized = False
        self.controls = {}
        self.layers = {}
        self.parameter_defaults = Parameters()
        self.simulation_defaults = SimulationOptions()
        self.display_defaults = DisplayOptions()
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #f5f6f8; color: #26323e; font-size: 12px; }
            QGroupBox { background: white; border: 1px solid #dce1e7; border-radius: 6px; margin-top: 12px; padding-top: 14px; font-weight: 600; }
            QPushButton { background: white; border: 1px solid #cbd4df; border-radius: 5px; padding: 7px 10px; }
            QPushButton:hover { background: #e4eef8; }
            QPushButton:disabled { color: #9ca8b4; background: #eef1f5; }
            QSpinBox, QDoubleSpinBox, QComboBox { background: white; padding: 3px; border: 1px solid #cbd4df; border-radius: 3px; }
            QPlainTextEdit { background: white; border: 1px solid #dce1e7; font-family: Consolas; font-size: 11px; }
        """)
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        bar = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("BEHAV3D  /  J1")
        title.setStyleSheet("font-size: 20px; font-weight: 700; padding: 8px;")
        bar.addWidget(title)
        bar.addStretch()
        self.preview_button = self.button("Preview", self.regenerate, bar)
        self.accept_button = self.button("Accept batch", self.accept_preview, bar)
        self.run_button = self.button("Run", self.run_cycles, bar)
        self.run_count = QtWidgets.QSpinBox()
        self.run_count.setRange(1, 1000)
        self.run_count.setValue(self.display_defaults.run_cycles)
        self.run_count.setSuffix(" cycles")
        bar.addWidget(self.run_count)
        self.pause_button = self.button("Pause", self.pause, bar)
        self.export_button = self.button("Export", lambda: self.submit("export"), bar)
        layout.addLayout(bar)
        self.status = QtWidgets.QLabel("Loading session…")
        self.status.setStyleSheet("padding: 7px; background: #e8edf3; border-radius: 4px;")
        layout.addWidget(self.status)
        self.plotter = QtInteractor(container, off_screen=off_screen, lighting="light kit")
        self.plotter.set_background("#fafafa")
        configure_studio_lighting(self.plotter)
        self.overlay = attach_viewer(self.plotter)
        layout.addWidget(self.plotter.interactor, stretch=1)
        self.setCentralWidget(container)
        self._panels()
        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setInterval(350)
        self.timer.timeout.connect(self.regenerate)
        self.thread = QtCore.QThread(self)
        self.worker = Worker(paths)
        self.worker.moveToThread(self.thread)
        self.requested.connect(self.worker.dispatch)
        self.worker.done.connect(self.completed)
        self.worker.failed.connect(self.failed)
        self.thread.finished.connect(self.worker.deleteLater)
        self.thread.start()
        self.plotter.add_key_event("n", self.accept_preview)
        self.plotter.add_key_event("space", self.pause)
        QtCore.QTimer.singleShot(0, lambda: self.submit("new"))

    def button(self, label, callback, layout):
        button = QtWidgets.QPushButton(label)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    def _group(self, parent, title):
        box = QtWidgets.QGroupBox(title)
        form = QtWidgets.QFormLayout(box)
        parent.addWidget(box)
        return form

    def _number(self, form, label, name, value, low, high, decimals=2, structural=False):
        defaults = self.simulation_defaults if structural else self.parameter_defaults
        value = getattr(defaults, name, value)
        integer = isinstance(value, int)
        widget = QtWidgets.QSpinBox() if integer else QtWidgets.QDoubleSpinBox()
        if not integer:
            widget.setDecimals(decimals)
            widget.setSingleStep(0.5 if high > 1 else 0.05)
        widget.setRange(low, high)
        widget.setValue(value)
        widget.setObjectName(name)
        form.addRow(label, widget)
        self.controls[name] = widget
        if not structural:
            widget.valueChanged.connect(self.parameters_changed)
        return widget

    def _check(self, form, label, name, value):
        value = getattr(self.parameter_defaults, name, value)
        widget = QtWidgets.QCheckBox(label)
        widget.setChecked(value)
        widget.stateChanged.connect(self.parameters_changed)
        self.controls[name] = widget
        form.addRow(widget)

    def _combo(self, form, label, name, values, current):
        default = getattr(self.parameter_defaults, name, current)
        current = format(default, "g") if isinstance(default, float) else str(default)
        widget = QtWidgets.QComboBox()
        widget.addItems(values)
        widget.setCurrentText(current)
        widget.currentTextChanged.connect(self.parameters_changed)
        self.controls[name] = widget
        form.addRow(label, widget)

    def _panels(self):
        dock = QtWidgets.QDockWidget("Parameters and layers", self)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable | QtWidgets.QDockWidget.DockWidgetFloatable)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        panel = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(panel)
        note = QtWidgets.QLabel("Changes update the preview.\nOnly Accept batch adds material.")
        note.setWordWrap(True)
        column.addWidget(note)
        form = self._group(column, "Next batch · mm")
        self._combo(form, "Mode", "mode", ["gradient_lift", "gradient_walk", "z_lift"], "gradient_lift")
        for label, name, val, low, high in [
            ("Source spacing", "source_spacing_mm", 3.0, 0.1, 50),
            ("Candidates", "count", 7, 1, 100),
            ("Separation", "separation_mm", 16.0, 0, 200),
            ("Bead width", "width_mm", 42.0, 1, 150),
            ("Bead height", "height_mm", 20.0, 1, 100),
            ("Clearance", "clearance_mm", 0.0, -50, 50)]:
            self._number(form, label, name, val, low, high)
        self._combo(form, "DDS deposit", "deposit_mode", ["line", "dot"], "line")
        self._number(form, "Line fraction", "line_fraction", 0.22, 0, 1)
        form = self._group(column, "Walk and orientation")
        for label, name, val, low, high in [
            ("Travel (mm)", "walk_distance_mm", 12.0, 0, 100),
            ("Step (mm)", "walk_step_mm", 1.0, 0.1, 20),
            ("Maximum steps", "walk_max_steps", 32, 1, 500),
            ("Segment start", "walk_start_fraction", 0.25, 0, 1),
            ("Cone (degrees)", "cone_deg", 45.0, 0, 90)]:
            self._number(form, label, name, val, low, high)
        self._combo(form, "Gradient direction", "tangent_sign", ["1", "-1"], "1")
        self._check(form, "Clamp orientation to cone", "clamp_to_cone", False)
        self._check(form, "Normal continuity rule", "normal_continuity", True)
        form = self._group(column, "Layers · display only")
        for key, label, visible in [
            ("goal", "Goal and scalar field", True), ("goal_edges", "Goal edges", False),
            ("environment", "Initial environment", True), ("deposits", "DDS surface", True),
            ("bead_edges", "Bead edges", False), ("contour", "Phi contour", True),
            ("phantom", "DDS phantom", True),
            ("sources", "Source samples", True), ("candidates", "Candidates", True),
            ("paths", "Paths", False), ("last", "Last batch · targets", True),
            ("offset", "Magenta offset (diagnostic)", False), ("axes", "Axes", False)]:
            widget = QtWidgets.QCheckBox(label)
            visible = self.display_defaults.layers.get(key, visible)
            widget.setChecked(visible)
            widget.toggled.connect(lambda checked, k=key: self.layer_changed(k, checked))
            self.layers[key] = widget
            form.addRow(widget)
        self.goal_opacity = QtWidgets.QDoubleSpinBox()
        self.goal_opacity.setRange(0, 1)
        self.goal_opacity.setSingleStep(0.1)
        self.goal_opacity.setValue(self.display_defaults.goal_opacity)
        self.goal_opacity.valueChanged.connect(self.opacity_changed)
        form.addRow("Goal opacity", self.goal_opacity)
        self.goal_lighting = QtWidgets.QCheckBox("Shade goal")
        self.goal_lighting.setChecked(self.display_defaults.goal_lighting)
        self.goal_lighting.setToolTip("Apply scene lighting to the goal. Off preserves unlit scalar colors; SSAO is a global screen-space effect.")
        self.goal_lighting.toggled.connect(self.goal_lighting_changed)
        form.addRow(self.goal_lighting)
        self.light_intensity = QtWidgets.QDoubleSpinBox()
        self.light_intensity.setRange(0, 4)
        self.light_intensity.setSingleStep(0.1)
        self.light_intensity.setValue(self.display_defaults.light_intensity)
        self.light_intensity.setToolTip("Multiplier for key, fill and rim lights. Does not affect an unlit goal.")
        self.light_intensity.valueChanged.connect(self.light_intensity_changed)
        form.addRow("Light intensity", self.light_intensity)
        self.phantom_opacity = QtWidgets.QDoubleSpinBox()
        self.phantom_opacity.setRange(0, 1)
        self.phantom_opacity.setSingleStep(0.05)
        self.phantom_opacity.setValue(PHANTOM_OPACITY)
        self.phantom_opacity.valueChanged.connect(self.phantom_opacity_changed)
        self.phantom_opacity.setToolTip("Proposed batch only. Does not modify accepted material.")
        form.addRow("Phantom opacity", self.phantom_opacity)
        self.edge_opacity = {}
        for key, label, default in [("goal_edges", "Goal edge opacity", self.display_defaults.goal_edge_opacity), ("bead_edges", "Bead edge opacity", self.display_defaults.bead_edge_opacity)]:
            opacity = QtWidgets.QDoubleSpinBox()
            opacity.setRange(0, 1)
            opacity.setSingleStep(0.05)
            opacity.setDecimals(2)
            opacity.setValue(default)
            opacity.setObjectName(key + "_opacity")
            opacity.valueChanged.connect(lambda value, k=key: self.edge_opacity_changed(k, value))
            opacity.setToolTip("Display opacity only. Does not change simulation geometry.")
            self.edge_opacity[key] = opacity
            form.addRow(label, opacity)
        self.bead_material = QtWidgets.QComboBox()
        self.material_colors = {"Warm white": "#f1f0e9", "Slate": "#a6b6c6", "Blue": "#73a8ce", "Graphite": "#677583"}
        self.bead_material.addItems(self.material_colors)
        self.bead_material.setCurrentText(self.display_defaults.bead_material)
        self.bead_material.currentTextChanged.connect(self.material_changed)
        form.addRow("Bead material", self.bead_material)
        self.contact_shading = QtWidgets.QCheckBox("Contact shading (SSAO)")
        self.contact_shading.setChecked(self.display_defaults.contact_shading)
        self.contact_shading.setToolTip("Soft contact shading and crevice darkening. Display only; disable for lighter GPU rendering.")
        self.contact_shading.toggled.connect(self.shading_changed)
        form.addRow(self.contact_shading)
        set_contact_shading(self.plotter, self.display_defaults.contact_shading)
        self.paper_button = QtWidgets.QPushButton("Paper view / frame goal")
        self.paper_button.clicked.connect(self.paper_view)
        form.addRow(self.paper_button)
        self._structural_group = QtWidgets.QGroupBox("New session")
        form = QtWidgets.QFormLayout(self._structural_group)
        column.addWidget(self._structural_group)
        self._number(form, "Voxel (mm)", "voxel_mm", 2.0, 0.5, 10, structural=True)
        self._number(form, "Physical threshold", "threshold", 0.5, 0.05, 0.95, structural=True)
        self._number(form, "Surface step", "surface_step", 1, 1, 4, structural=True)
        self._number(form, "Checkpoint every N", "checkpoint_every", 10, 0, 100, structural=True)
        label = QtWidgets.QLabel("Applied only when creating a new session.\nPrevious session files are retained.")
        label.setWordWrap(True)
        form.addRow(label)
        self.new_button = QtWidgets.QPushButton("Create new session")
        self.new_button.clicked.connect(lambda: self.submit("new"))
        form.addRow(self.new_button)
        column.addStretch()
        scroll.setWidget(panel)
        dock.setWidget(scroll)
        dock.setMinimumWidth(300)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea, dock)
        bottom = QtWidgets.QDockWidget("Activity and performance", self)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        bottom.setWidget(self.log)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea, bottom)
        bottom.setMaximumHeight(160)

    def parameters(self):
        values = {}
        for name in asdict(Parameters()):
            widget = self.controls[name]
            if isinstance(widget, QtWidgets.QCheckBox):
                value = widget.isChecked()
            elif isinstance(widget, QtWidgets.QComboBox):
                value = widget.currentText()
            else:
                value = widget.value()
            values[name] = float(value) if name == "tangent_sign" else value
        return Parameters(**values)

    def options(self):
        return SimulationOptions(**{key: self.controls[key].value()
            for key in ("voxel_mm", "threshold", "surface_step", "checkpoint_every")})

    def parameters_changed(self, *_):
        self.pause()
        self.settings_revision += 1
        self.preview_revision = -1
        self._draw_phantom(None)
        self.accept_button.setEnabled(False)
        if hasattr(self, "timer"):
            self.timer.start()

    def regenerate(self):
        if not self.busy and self.initialized:
            self.timer.stop()
            self.submit("preview")

    def submit(self, action):
        if self.busy:
            return
        if action in {"new", "export"}:
            self.pause()
        try:
            parameters = self.parameters()
            parameters.validate()
            job = {"action": action, "parameters": parameters, "options": self.options(),
                   "phantom": self.layers["phantom"].isChecked(),
                   "settings_revision": self.settings_revision}
            if action in {"accept", "phantom"}:
                if self.preview is None or self.preview_revision != self.settings_revision:
                    return
                job["token"] = self.preview.token
            if action == "accept":
                self._draw_phantom(None)
            self.busy = True
            self.set_buttons()
            self.status.setText({"new": "Loading session…", "preview": "Computing preview…",
                "accept": "Adding batch and updating surface…", "export": "Exporting…",
                "offset": "Computing diagnostic…", "phantom": "Computing DDS phantom…"}[action])
            self.requested.emit(job)
        except Exception as exc:
            self.log.appendPlainText(str(exc))

    def accept_preview(self):
        if self.accept_button.isEnabled():
            self.submit("accept")

    def run_cycles(self):
        if not self.initialized:
            return
        self.remaining = self.run_count.value()
        self.running = True
        self.set_buttons()
        if not self.busy:
            if self.preview_revision == self.settings_revision and self.preview is not None:
                self.accept_preview()
            else:
                self.regenerate()

    def pause(self):
        self.running = False
        if hasattr(self, "pause_button"):
            self.set_buttons()
            if self.busy and self.initialized:
                self.status.setText("Pause requested · finishing the current operation before stopping")

    def set_buttons(self):
        valid = self.preview is not None and self.preview_revision == self.settings_revision and self.preview.targets.count > 0
        self.preview_button.setEnabled(not self.busy and self.initialized)
        self.accept_button.setEnabled(not self.busy and valid)
        self.run_button.setEnabled(self.initialized and not self.running and not self.busy)
        self.pause_button.setEnabled(self.running or self.busy)
        self.new_button.setEnabled(not self.busy)
        self.export_button.setEnabled(not self.busy and self.initialized)

    @QtCore.Slot(object)
    def completed(self, result):
        self.busy = False
        job = result["job"]
        action = job["action"]
        self.latest = result
        if action == "new":
            self.initialized = True
            self._static_scene(result)
            self.log.appendPlainText(f"Session: {result['output']}")
        if action in {"new", "accept"}:
            self._surface_scene(result)
        if action == "accept":
            self.preview = None
            self.preview_revision = -1
            self._clear_preview()
            self.remaining = max(0, self.remaining - 1)
            self.log.appendPlainText(f"Cycle {result['cycle']} · {result['extra']['targets']} deposits · "
                f"update {result['extra']['timings']['accept_s']:.2f} s")
            if not self.remaining:
                self.running = False
        elif action == "export":
            try:
                self.plotter.render_window.MakeCurrent()
                self.plotter.screenshot(str(result["extra"] / "viewport.png"))
            except Exception as exc:
                self.log.appendPlainText(f"Screenshot: {exc}")
            self.log.appendPlainText(f"Exported: {result['extra']}")
        elif action == "offset" and job["settings_revision"] == self.settings_revision:
            c = result["extra"]
            self._lines("offset", "offset", c.offset_points, c.offset_lines, "#e34bd3")
        elif action == "phantom" and job["settings_revision"] == self.settings_revision:
            if self.preview is not None and self.preview.token == job["token"]:
                self._draw_phantom(result["extra"])
        if result["preview"] is not None and action in {"new", "preview"}:
            if job["settings_revision"] == self.settings_revision:
                self.preview = result["preview"]
                self.preview_revision = self.settings_revision
                self._preview_scene(self.preview)
                p = self.preview
                self.log.appendPlainText(f"Preview · {len(p.candidates.sampling_points)} sources → {p.targets.count} targets · "
                    f"{p.timings['preview_s']:.2f} s · contour {'cached' if p.geometry_cached else 'recomputed'}")
                if not p.targets.count:
                    self.running = False
            else:
                self.preview_revision = -1
        total = len(result["history"])
        n = self.preview.targets.count if self.preview and self.preview_revision == self.settings_revision else 0
        grid = result["grid"]
        voxels = f" · {np.prod(grid):,} voxels" if grid else ""
        self.status.setText(f"Cycle {total} · Next batch: {n} targets{voxels}")
        self.set_buttons()
        self.plotter.render()
        if action == "new":
            self.paper_view()
        if action == "accept" or self.preview_revision != self.settings_revision:
            QtCore.QTimer.singleShot(0, self.regenerate)
        elif self.running and action in {"new", "preview", "offset", "phantom"}:
            QtCore.QTimer.singleShot(100, lambda: self.accept_preview() if self.running else None)

    @QtCore.Slot(object)
    def failed(self, result):
        self.busy = False
        self.running = False
        self.status.setText("Operation failed: " + result["error"])
        self.log.appendPlainText(result["traceback"])
        self.set_buttons()

    def _static_scene(self, result):
        self.plotter.clear()
        # PyVista clear() removes lights as well as actors. Reinstall them here.
        configure_studio_lighting(self.plotter, self.light_intensity.value())
        set_contact_shading(self.plotter, self.contact_shading.isChecked())
        self.overlay = attach_viewer(self.plotter)
        self.actors.clear()
        self.meshes.clear()
        self.edge_sources.clear()
        goal = polydata(result["field_vertices"], result["field_faces"])
        goal["priority"] = result["heat"]
        self.meshes["goal"] = goal
        self.actors["goal"] = self.plotter.add_mesh(goal, scalars="priority", cmap="autumn",
            opacity=self.goal_opacity.value(), lighting=self.goal_lighting.isChecked(),
            show_scalar_bar=False, name="goal", reset_camera=False)
        self.actors["environment"] = self.plotter.add_mesh(polydata(result["scan_vertices"], result["scan_faces"]),
            color="#d6d5d0", smooth_shading=False, name="environment", reset_camera=False)
        self.plotter.add_axes()
        self.plotter.hide_axes()
        for key in ("goal", "environment"):
            self.layer_changed(key, self.layers[key].isChecked())
        self.layer_changed("goal_edges", self.layers["goal_edges"].isChecked())
        self.layer_changed("axes", self.layers["axes"].isChecked())

    def _surface_scene(self, result):
        surface = result["surface"]
        self.plotter.remove_actor("bead_edges", render=False)
        self.actors.pop("bead_edges", None)
        self.edge_sources.pop("bead_edges", None)
        if not surface.is_empty:
            self.meshes["deposits"] = polydata(surface.vertices, surface.faces)
            self.actors["deposits"] = self.plotter.add_mesh(self.meshes["deposits"],
                color=self.material_colors[self.bead_material.currentText()], smooth_shading=True,
                **SURFACE_MATERIAL,
                name="deposits", reset_camera=False)
            self.layer_changed("deposits", self.layers["deposits"].isChecked())
            self.layer_changed("bead_edges", self.layers["bead_edges"].isChecked())
        self._cloud("last", "last", result["last_targets"], "#73a8ce", 9)

    def _cloud(self, name, layer, points, color, size):
        if points is None or not len(points):
            try:
                self.overlay.get(name).remove()
            except KeyError:
                pass
            self.actors.pop(name, None)
            return
        self.actors[name] = add_colored_point_cloud(self.overlay, points, name=name,
            color=color, point_size=size, render_as_spheres=True)
        self.layer_changed(layer, self.layers[layer].isChecked())

    def _lines(self, name, layer, points, lines, color):
        actor = add_line_segments(self.overlay, points, lines, name=name, color=color, line_width=3)
        self.actors[name] = actor
        self.layer_changed(layer, self.layers[layer].isChecked())

    def _preview_scene(self, preview):
        self._draw_phantom(preview.phantom_surface)
        c = preview.contour
        self._lines("contour", "contour", c.contour_points, c.contour_lines, "#00b9cb")
        self._cloud("sources", "sources", preview.candidates.sampling_points, "#00c5d4", 6)
        self._cloud("candidates", "candidates", preview.targets.end_points, "#2686ce", 11)
        points, _ = preview.targets.flattened_points_and_z_dirs()
        self._lines("paths", "paths", points, np.arange(len(points)).reshape(-1, 2), "#e8a029")
        self._lines("offset", "offset", np.empty((0, 3)), np.empty((0, 2), int), "#e34bd3")
        if self.layers["offset"].isChecked():
            QtCore.QTimer.singleShot(0, lambda: self.submit("offset"))

    def _clear_preview(self):
        self._draw_phantom(None)
        for name in ("sources", "candidates"):
            self._cloud(name, name, np.empty((0, 3)), "#ffffff", 1)
        for name in ("contour", "paths", "offset"):
            self._lines(name, name, np.empty((0, 3)), np.empty((0, 2), int), "#ffffff")

    def layer_changed(self, key, checked):
        if key in {"goal_edges", "bead_edges"} and checked:
            self._ensure_edges(key)
        if key == "axes":
            self.plotter.show_axes() if checked else self.plotter.hide_axes()
        else:
            actor = self.actors.get(key)
            if actor is not None:
                if hasattr(actor, "set_visible"):
                    actor.set_visible(checked)
                else:
                    actor.SetVisibility(checked)
        if key == "offset" and checked and self.preview and not self.busy and self.actors.get("offset") is None:
            self.submit("offset")
        if key == "phantom" and checked and self.preview and not self.busy and self.actors.get("phantom") is None:
            self.submit("phantom")
        self.plotter.render()

    def _draw_phantom(self, surface):
        self.plotter.remove_actor("phantom", render=False)
        self.actors.pop("phantom", None)
        if surface is not None and not surface.is_empty:
            actor = self.plotter.add_mesh(polydata(surface.vertices, surface.faces),
                name="phantom", color=PHANTOM_COLOR, opacity=self.phantom_opacity.value(),
                smooth_shading=True, **SURFACE_MATERIAL,
                pickable=False, reset_camera=False, render=False)
            actor.SetVisibility(self.layers["phantom"].isChecked())
            self.actors["phantom"] = actor

    def phantom_opacity_changed(self, value):
        actor = self.actors.get("phantom")
        if actor is not None:
            actor.GetProperty().SetOpacity(value)
            self.plotter.render()

    def _ensure_edges(self, key):
        """Reuse the surface mesh for an independently translucent wire overlay."""
        mesh = self.meshes.get("goal" if key == "goal_edges" else "deposits")
        if mesh is None or not mesh.n_cells:
            return
        if self.edge_sources.get(key) is mesh:
            return
        actor = self.plotter.add_mesh(mesh, style="wireframe", color="#344653",
            opacity=self.edge_opacity[key].value(), line_width=1,
            lighting=False, show_scalar_bar=False, name=key, reset_camera=False,
            pickable=False, render=False)
        actor.mapper.scalar_visibility = False
        # Offset lines slightly in depth to avoid flickering over their own faces.
        actor.mapper.SetResolveCoincidentTopologyToPolygonOffset()
        actor.mapper.SetRelativeCoincidentTopologyLineOffsetParameters(0, -2)
        self.actors[key] = actor
        self.edge_sources[key] = mesh

    def edge_opacity_changed(self, key, value):
        actor = self.actors.get(key)
        if actor is not None:
            actor.GetProperty().SetOpacity(value)
            self.plotter.render()

    def material_changed(self, name):
        actor = self.actors.get("deposits")
        if actor is not None:
            actor.prop.color = self.material_colors[name]
            self.plotter.render()

    def shading_changed(self, enabled):
        set_contact_shading(self.plotter, enabled)
        self.plotter.render()

    def light_intensity_changed(self, value):
        configure_studio_lighting(self.plotter, value)
        self.plotter.render()

    def goal_lighting_changed(self, enabled):
        actor = self.actors.get("goal")
        if actor is not None:
            actor.prop.lighting = enabled
            self.plotter.render()

    def opacity_changed(self, value):
        actor = self.actors.get("goal")
        if actor is not None:
            actor.GetProperty().SetOpacity(value)
            self.plotter.render()

    def paper_view(self):
        self.plotter.enable_parallel_projection()
        self.plotter.view_isometric()
        if "goal" in self.actors:
            bounds = np.array(self.actors["goal"].GetBounds()).reshape(3, 2)
            bounds[:, 0] -= 0.04
            bounds[:, 1] += 0.04
            self.plotter.reset_camera(bounds=bounds.ravel())
        else:
            self.plotter.reset_camera()
        self.plotter.render()

    def closeEvent(self, event):
        self.pause()
        if self.busy:
            self.status.setText("Wait for the current operation to finish before closing.")
            event.ignore()
            return
        self.thread.quit()
        self.thread.wait(3000)
        self.plotter.close()
        event.accept()
