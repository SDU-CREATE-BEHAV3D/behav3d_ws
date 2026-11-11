"""Native Qt panels and the existing PyVista/DDS drawing stack."""
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path
import tempfile
import traceback
from time import perf_counter

import numpy as np
import pyvista as pv
from PySide6 import QtCore, QtWidgets
from pyvistaqt import QtInteractor

from lib_scalar.viz_dds import attach_viewer, add_colored_point_cloud, add_line_segments
from .config import Parameters, SimulationOptions, DisplayOptions
from .session import Session
from .appearance import configure_studio_lighting, set_contact_shading, surface_material
from .navigation import capture_camera, configure_navigation, restore_camera
from .telemetry import event
from .target_frames import AXIS_COLORS, frame_glyphs, select_target_frames
from .target_history import load_frame_yaml
from .preferences import (DEFAULTS_PATH, load_defaults, options_from_dict, save_defaults,
                          camera_settings, apply_camera_settings)
from .state import save_state, load_state


class Worker(QtCore.QObject):
    done = QtCore.Signal(object)
    failed = QtCore.Signal(object)

    def __init__(self, paths):
        super().__init__()
        self.paths = paths
        self.session = None

    @QtCore.Slot(object)
    def dispatch(self, job):
        start = perf_counter()
        output = self.session.output if self.session is not None and job["action"] != "new" else None
        try:
            action = job["action"]
            extra = None
            if output is not None:
                event(output, action, "started", cycle=self.session.revision)
            if action == "new":
                session = Session(**self.paths, options=job["options"])
                output = session.output
                event(output, action, "started", cycle=0)
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
            elif action == "load_target_yaml":
                points, z_dirs = load_frame_yaml(job["target_yaml_path"])
                extra = {"path": job["target_yaml_path"], "points": points, "z_dirs": z_dirs}
                preview = self.session.preview
            elif action == "save_state":
                extra = save_state(self.session, job["state_path"], job["parameters"], job["view"])
                preview = self.session.preview
            elif action == "open_state":
                session, parameters, view = load_state(job["state_path"], self.paths)
                preview = session.prepare(parameters, include_phantom=view["display"]["layers"].get("phantom", True))
                self.session = session
                self.paths.update(field_path=session.fixed_field_path, original_path=session.original_field_path,
                                  scan_path=session.scan_path, scan_scale=session.scan_scale, scan_yaw=session.scan_yaw)
                output = session.output
                extra = dict(parameters=asdict(parameters), simulation=asdict(session.options), view=view)
            else:
                raise ValueError(action)
            s = self.session
            domain = s.domain_for_display()
            worker_seconds = perf_counter() - start
            event(output, action, "finished", cycle=s.revision, seconds=worker_seconds)
            self.done.emit({"job": job, "preview": preview, "extra": extra,
                "cycle": s.revision, "surface": s.surface,
                "field_vertices": s.field.field_vertices_scaled + np.asarray(s.field.offset_xyz),
                "field_path": str(s.field_path),
                "field_faces": s.field.field_faces, "heat": s.field.heat_norm,
                "scan_vertices": np.asarray(s.base.vertices), "scan_faces": np.asarray(s.base.triangles),
                "last_targets": s.last_targets, "history": list(s.history), "output": s.output,
                "target_history_points": s.target_history.points,
                "target_history_z_dirs": s.target_history.z_dirs,
                "grid": s.simulator.domain.grid_shape if s.simulator else None,
                "domain": domain,
                "domain_planned": s.simulator is None,
                "session_inputs": dict(scan_mesh=str(s.scan_path.resolve()), scan_scale=s.scan_scale,
                    scan_yaw=s.scan_yaw, field_state=str(s.fixed_field_path.resolve()),
                    original_field_state=str(s.original_field_path.resolve())),
                "worker_s": worker_seconds,
                "geometry_builds": s.geometry_builds})
        except Exception as exc:
            if output is not None:
                event(output, job["action"], "failed", seconds=perf_counter() - start, error=str(exc))
            self.failed.emit({"job": job, "error": str(exc), "traceback": traceback.format_exc()})


def polydata(vertices, faces):
    return pv.PolyData(vertices, np.c_[np.full(len(faces), 3), faces].ravel())


class Workbench(QtWidgets.QMainWindow):
    requested = QtCore.Signal(object)

    def __init__(self, paths, off_screen=False, preferences_path=DEFAULTS_PATH):
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
        self.frame_sources = {}
        self.loaded_targets = None
        self.initialized = False
        self.scene_field_path = None
        self.controls = {}
        self.layers = {}
        self.preferences_path = Path(preferences_path)
        self.preferences_error = None
        try:
            saved = load_defaults(self.preferences_path)
        except (ValueError, OSError, TypeError) as exc:
            saved = {}
            self.preferences_error = str(exc)
        self.parameter_defaults = options_from_dict(Parameters, saved.get("parameters", {}))
        self.simulation_defaults = options_from_dict(SimulationOptions, saved.get("simulation", {}))
        self.display_defaults = options_from_dict(DisplayOptions, saved.get("display", {}))
        self.startup_camera = saved.get("camera")
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
        storage = QtWidgets.QHBoxLayout()
        self.save_defaults_button = self.button("Save defaults", self.save_current_defaults, storage)
        self.save_state_button = self.button("Save state…", self.choose_save_state, storage)
        self.open_state_button = self.button("Open state…", self.choose_open_state, storage)
        self.captures_button = self.button("Save captures", self.save_captures, storage)
        self.captures_button.setToolTip("Save Paper, Front and Top PNGs in the session captures folder, keeping current layers and appearance.")
        storage.addStretch()
        layout.addLayout(storage)
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
        if self.preferences_error:
            self.log.appendPlainText("Saved defaults ignored; using config.py: " + self.preferences_error)
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
            ("Bead height", "height_mm", 20.0, 1, 100),
            ("Clearance", "clearance_mm", 0.0, -50, 50)]:
            self._number(form, label, name, val, low, high)
        self._combo(form, "DDS deposit", "deposit_mode", ["line", "dot"], "line")
        self._number(form, "Line fraction", "line_fraction", 0.22, 0, 1)
        form = self._group(column, "Width control")
        self._combo(form, "Width mode", "width_mode", ["fixed", "field"], "fixed")
        self.width_note = QtWidgets.QLabel()
        self.width_note.setWordWrap(True)
        form.addRow(self.width_note)
        fixed_box = QtWidgets.QGroupBox("Fixed mode only · mm")
        column.addWidget(fixed_box)
        fixed_form = QtWidgets.QFormLayout(fixed_box)
        self._number(fixed_form, "Bead width", "width_mm", 42.0, 1, 150)
        self._number(fixed_form, "Separation", "separation_mm", 38.0, 0, 200)
        field_box = QtWidgets.QGroupBox("Width field mode only · mm")
        column.addWidget(field_box)
        field_form = QtWidgets.QFormLayout(field_box)
        self.width_field_input = QtWidgets.QLineEdit(self.parameter_defaults.width_field_path)
        self.width_field_input.setPlaceholderText("NPZ containing width_norm on the original goal")
        self.width_field_input.setToolTip("One normalized value per original goal vertex, in the same order. No extra goal subdivision.")
        self.width_field_input.textChanged.connect(self.parameters_changed)
        self.controls["width_field_path"] = self.width_field_input
        field_form.addRow("Width field NPZ", self.width_field_input)
        browse = QtWidgets.QPushButton("Load width field…")
        browse.clicked.connect(self.choose_width_field)
        field_form.addRow(browse)
        self._number(field_form, "Minimum bead width", "width_min_mm", 20.0, 0.1, 150)
        self._number(field_form, "Maximum bead width", "width_max_mm", 40.0, 0.1, 150)
        self._number(field_form, "Within-batch overlap", "overlap_mm", 4.0, 0, 150)
        self.fixed_width_group, self.field_width_group = fixed_box, field_box
        self.controls["width_mode"].currentTextChanged.connect(self.sync_width_controls)
        self.sync_width_controls()
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
            ("environment", "Initial environment", True),
            ("deposits", "Bead surface", True), ("bead_edges", "Bead edges", False),
            ("domain", "DDS domain", False), ("contour", "Phi contour", True),
            ("phantom", "DDS phantom", True),
            ("sources", "Source samples", True), ("candidates", "Candidates", True),
            ("paths", "Paths", False), ("last", "Last batch · targets", True),
            ("target_history", "Accepted targets · XYZ", True),
            ("target_preview", "Preview targets · XYZ", True),
            ("target_imported", "Loaded target YAML · XYZ", True),
            ("offset", "Magenta offset (diagnostic)", False), ("axes", "Axes", False)]:
            widget = QtWidgets.QCheckBox(label)
            visible = self.display_defaults.layers.get(key, visible)
            widget.setChecked(visible)
            widget.toggled.connect(lambda checked, k=key: self.layer_changed(k, checked))
            self.layers[key] = widget
            form.addRow(widget)
        for key in ("target_history", "target_preview", "target_imported"):
            self.layers[key].setToolTip("Selected start/endpoint frames. X red, Y green, Z blue. Z follows the target; X/Y complete a stable display frame because YAML stores O and Z only.")
        self.target_starts = QtWidgets.QCheckBox("Target starts · XYZ")
        self.target_starts.setChecked(self.display_defaults.target_show_starts)
        self.target_endpoints = QtWidgets.QCheckBox("Target endpoints · XYZ (DDS)")
        self.target_endpoints.setChecked(self.display_defaults.target_show_endpoints)
        for widget in (self.target_starts, self.target_endpoints):
            widget.setToolTip("Select which planes to display in all three target layers. Display only; YAML retains both start and end.")
            widget.toggled.connect(self.target_axis_length_changed)
            form.addRow(widget)
        self.target_axis_thickness = QtWidgets.QDoubleSpinBox()
        self.target_axis_thickness.setRange(0.5, 4)
        self.target_axis_thickness.setSingleStep(0.1)
        self.target_axis_thickness.setSuffix(" ×")
        self.target_axis_thickness.setValue(self.display_defaults.target_axis_thickness)
        self.target_axis_thickness.valueChanged.connect(self.target_axis_length_changed)
        form.addRow("Target line thickness", self.target_axis_thickness)
        self.target_axis_length = QtWidgets.QDoubleSpinBox()
        self.target_axis_length.setRange(1, 100)
        self.target_axis_length.setSingleStep(1)
        self.target_axis_length.setValue(self.display_defaults.target_axis_length_mm)
        self.target_axis_length.setToolTip("Physical length of each XYZ tube. Display only; no arrowheads or target labels.")
        self.target_axis_length.valueChanged.connect(self.target_axis_length_changed)
        form.addRow("Target axis length (mm)", self.target_axis_length)
        self.load_target_button = QtWidgets.QPushButton("Load target YAML…")
        self.load_target_button.clicked.connect(self.choose_target_yaml)
        self.load_target_button.setToolTip("View a previous targets_history.yaml or segments.yaml. Does not replay deposits or change the current session.")
        form.addRow(self.load_target_button)
        self.loaded_target_label = QtWidgets.QLabel("No target YAML loaded")
        self.loaded_target_label.setWordWrap(True)
        form.addRow(self.loaded_target_label)
        self.goal_opacity = QtWidgets.QDoubleSpinBox()
        self.goal_opacity.setRange(0, 1)
        self.goal_opacity.setSingleStep(0.1)
        self.goal_opacity.setValue(self.display_defaults.goal_opacity)
        self.goal_opacity.valueChanged.connect(self.opacity_changed)
        form.addRow("Goal opacity", self.goal_opacity)
        self.domain_opacity = QtWidgets.QDoubleSpinBox()
        self.domain_opacity.setRange(0, 1)
        self.domain_opacity.setSingleStep(0.05)
        self.domain_opacity.setValue(self.display_defaults.domain_opacity)
        self.domain_opacity.setToolTip("Unlit domain bounds. Planned before the first batch; active afterwards.")
        self.domain_opacity.valueChanged.connect(self.domain_opacity_changed)
        form.addRow("Domain opacity", self.domain_opacity)
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
        self.phantom_opacity.setValue(self.display_defaults.phantom_opacity)
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
        set_contact_shading(self.plotter, self.display_defaults.contact_shading, self.display_defaults)
        self.camera_controls = QtWidgets.QComboBox()
        self.camera_controls.addItems(["Rhino", "VTK"])
        self.camera_controls.setCurrentText(self.display_defaults.camera_controls)
        self.camera_controls.setToolTip("Rhino: right drag rotates; Shift + right pans; Ctrl + right or wheel zooms; Ctrl + Shift + right rotates.")
        self.camera_controls.currentTextChanged.connect(lambda value: configure_navigation(self.plotter, value))
        configure_navigation(self.plotter, self.camera_controls.currentText())
        form.addRow("Camera controls", self.camera_controls)
        self.paper_button = QtWidgets.QPushButton("Paper view / frame goal")
        self.paper_button.clicked.connect(self.paper_view)
        form.addRow(self.paper_button)
        views = QtWidgets.QHBoxLayout()
        self.front_button = self.button("Front", lambda: self.standard_view((0, -1, 0), (0, 0, 1)), views)
        self.top_button = self.button("Top", lambda: self.standard_view((0, 0, 1), (0, 1, 0)), views)
        form.addRow(views)
        self._structural_group = QtWidgets.QGroupBox("New session")
        form = QtWidgets.QFormLayout(self._structural_group)
        column.addWidget(self._structural_group)
        self._number(form, "Voxel (mm)", "voxel_mm", 2.0, 0.5, 10, structural=True)
        self._number(form, "Physical threshold", "threshold", 0.5, 0.05, 0.95, structural=True)
        self._number(form, "Surface step", "surface_step", 1, 1, 4, structural=True)
        self._number(form, "Domain padding (mm)", "padding_mm", 60.0, 0, 500, structural=True).setToolTip(
            "Initial margin in X, Y and Z. The effective margin is at least the bead width and height.")
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

    def choose_width_field(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load width field", "", "NumPy width field (*.npz)")
        if path:
            self.width_field_input.setText(path)

    def sync_width_controls(self, *_):
        active = self.controls["width_mode"].currentText() == "field"
        self.fixed_width_group.setEnabled(not active)
        self.field_width_group.setEnabled(active)
        self.width_note.setText(
            "FIELD ACTIVE · Original goal mesh; no extra subdivision. Width range and overlap control the next batch. Height and 3D source spacing remain shared."
            if active else "FIXED ACTIVE · Fixed width and separation. Uses the configured goal mesh with its existing subdivision.")

    def parameters(self):
        values = {}
        for name in asdict(Parameters()):
            widget = self.controls[name]
            if isinstance(widget, QtWidgets.QCheckBox):
                value = widget.isChecked()
            elif isinstance(widget, QtWidgets.QComboBox):
                value = widget.currentText()
            elif isinstance(widget, QtWidgets.QLineEdit):
                value = widget.text().strip()
            else:
                value = widget.value()
            values[name] = float(value) if name == "tangent_sign" else value
        return Parameters(**values)

    def options(self):
        return replace(self.simulation_defaults, **{key: self.controls[key].value()
            for key in ("voxel_mm", "threshold", "surface_step", "padding_mm", "checkpoint_every")})

    def display_settings(self):
        return replace(self.display_defaults, run_cycles=self.run_count.value(),
            goal_opacity=self.goal_opacity.value(), goal_lighting=self.goal_lighting.isChecked(),
            domain_opacity=self.domain_opacity.value(), light_intensity=self.light_intensity.value(),
            phantom_opacity=self.phantom_opacity.value(), bead_material=self.bead_material.currentText(),
            goal_edge_opacity=self.edge_opacity["goal_edges"].value(), bead_edge_opacity=self.edge_opacity["bead_edges"].value(),
            contact_shading=self.contact_shading.isChecked(), camera_controls=self.camera_controls.currentText(),
            target_axis_length_mm=self.target_axis_length.value(), target_axis_thickness=self.target_axis_thickness.value(),
            target_show_starts=self.target_starts.isChecked(), target_show_endpoints=self.target_endpoints.isChecked(),
            layers={key: widget.isChecked() for key, widget in self.layers.items()})

    def save_current_defaults(self):
        if self.busy or not self.initialized:
            return
        try:
            save_defaults(self.preferences_path, self.latest["session_inputs"], self.parameters(), self.options(),
                          self.display_settings(), camera_settings(self.plotter))
            self.log.appendPlainText(f"Startup defaults saved: {self.preferences_path}")
            self.status.setText("Defaults saved for the next launch")
        except Exception as exc:
            self.log.appendPlainText("Could not save defaults: " + str(exc))

    def choose_save_state(self):
        if self.busy or not self.initialized:
            return
        self.pause()
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save simulation state", "simulation.b3dstate", "BEHAV3D state (*.b3dstate)")
        if path:
            if not path.lower().endswith(".b3dstate"):
                path += ".b3dstate"
            self.submit("save_state", state_path=path, view={"display": asdict(self.display_settings()),
                        "camera": camera_settings(self.plotter)})

    def choose_open_state(self):
        if self.busy:
            return
        self.pause()
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open simulation state", "", "BEHAV3D state (*.b3dstate)")
        if path:
            self.submit("open_state", state_path=path)

    def apply_state_controls(self, extra):
        self.display_defaults = options_from_dict(DisplayOptions, extra["view"]["display"])
        self.simulation_defaults = options_from_dict(SimulationOptions, extra["simulation"])
        values = {**extra["parameters"], **extra["simulation"]}
        display = self.display_defaults
        pairs = [(self.controls[key], value) for key, value in values.items() if key in self.controls]
        pairs += [(self.layers[key], value) for key, value in display.layers.items() if key in self.layers]
        pairs += [(getattr(self, key), getattr(display, key)) for key in (
            "goal_opacity", "goal_lighting", "domain_opacity", "light_intensity", "phantom_opacity",
            "bead_material", "contact_shading", "camera_controls")]
        pairs += [(self.edge_opacity["goal_edges"], display.goal_edge_opacity),
                  (self.edge_opacity["bead_edges"], display.bead_edge_opacity), (self.run_count, display.run_cycles),
                  (self.target_axis_length, display.target_axis_length_mm), (self.target_axis_thickness, display.target_axis_thickness),
                  (self.target_starts, display.target_show_starts), (self.target_endpoints, display.target_show_endpoints)]
        blockers = [QtCore.QSignalBlocker(widget) for widget, _ in pairs]
        try:
            for widget, value in pairs:
                if isinstance(widget, QtWidgets.QCheckBox):
                    widget.setChecked(value)
                elif isinstance(widget, QtWidgets.QComboBox):
                    widget.setCurrentText(format(value, "g") if isinstance(value, float) else str(value))
                elif isinstance(widget, QtWidgets.QLineEdit):
                    widget.setText(str(value))
                else:
                    widget.setValue(value)
        finally:
            del blockers
        self.sync_width_controls()
        configure_navigation(self.plotter, self.camera_controls.currentText())
        self.loaded_targets = None
        self.loaded_target_label.setText("No target YAML loaded")

    def parameters_changed(self, *_):
        self.pause()
        self.settings_revision += 1
        self.preview_revision = -1
        self._draw_phantom(None)
        if hasattr(self, "target_axis_length"):
            self._target_frames("target_preview", np.empty((0, 3)), np.empty((0, 3)))
        self.accept_button.setEnabled(False)
        if hasattr(self, "timer"):
            self.timer.start()

    def regenerate(self):
        if not self.busy and self.initialized:
            self.timer.stop()
            self.submit("preview")

    def choose_target_yaml(self):
        if self.busy or not self.initialized:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load target YAML", "", "Target YAML (*.yaml *.yml)")
        if path:
            self.pause()
            self.submit("load_target_yaml", target_yaml_path=path)

    def submit(self, action, **details):
        if self.busy:
            return
        if action in {"new", "export", "save_state", "open_state"}:
            self.pause()
        if action == "open_state":
            self.timer.stop()
        try:
            parameters = self.parameters()
            if action not in {"load_target_yaml", "open_state"}:
                parameters.validate()
            job = {"action": action, "parameters": parameters, "options": self.options(),
                   "phantom": self.layers["phantom"].isChecked(),
                   "settings_revision": self.settings_revision}
            job.update(details)
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
                "offset": "Computing diagnostic…", "phantom": "Computing DDS phantom…",
                "load_target_yaml": "Loading target YAML for display…", "save_state": "Saving compressed simulation state…",
                "open_state": "Opening simulation state and rebuilding its surface…"}[action])
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
        self.load_target_button.setEnabled(not self.busy and self.initialized)
        self.save_defaults_button.setEnabled(not self.busy and self.initialized)
        self.save_state_button.setEnabled(not self.busy and self.initialized)
        self.open_state_button.setEnabled(not self.busy)
        self.captures_button.setEnabled(not self.busy and self.initialized)

    @QtCore.Slot(object)
    def completed(self, result):
        start = perf_counter()
        event(result["output"], "viewport", "started", cycle=result["cycle"], action=result["job"]["action"])
        # Read the live camera when the worker finishes, so navigation during a
        # calculation is retained as well as navigation between batches.
        camera = capture_camera(self.plotter) if result["job"]["action"] not in {"new", "open_state"} else None
        suppressed = self.plotter.suppress_rendering
        self.plotter.suppress_rendering = True
        try:
            self._complete_scene(result)
        finally:
            if camera is not None:
                restore_camera(self.plotter, camera)
            self.plotter.suppress_rendering = suppressed
        scene_seconds = perf_counter() - start
        event(result["output"], "viewport_scene", "finished", cycle=result["cycle"], seconds=scene_seconds)
        render_start = perf_counter()
        event(result["output"], "viewport_render", "started", cycle=result["cycle"])
        self.plotter.render()
        render_seconds = perf_counter() - render_start
        event(result["output"], "viewport_render", "finished", cycle=result["cycle"], seconds=render_seconds)
        event(result["output"], "viewport", "finished", cycle=result["cycle"],
              action=result["job"]["action"], scene_s=scene_seconds,
              render_s=render_seconds, ui_s=perf_counter() - start)

    def _complete_scene(self, result):
        self.busy = False
        job = result["job"]
        action = job["action"]
        self.latest = result
        if action == "open_state":
            self.apply_state_controls(result["extra"])
            self.settings_revision += 1
            job["settings_revision"] = self.settings_revision
            self.preview = None
            self.preview_revision = -1
        if action == "load_target_yaml":
            self.loaded_targets = result["extra"]
            self.loaded_target_label.setText(f"{len(self.loaded_targets['points'])} target frames · " + str(self.loaded_targets["path"]).replace("\\", "/").rsplit("/", 1)[-1])
            self.loaded_target_label.setToolTip(str(self.loaded_targets["path"]))
            self.layers["target_imported"].setChecked(True)
        scene_changed = self.scene_field_path != result["field_path"]
        if action in {"new", "open_state"} or scene_changed:
            self.initialized = True
            self._static_scene(result)
            self.scene_field_path = result["field_path"]
        if action in {"new", "open_state"}:
            self.log.appendPlainText(f"Session: {result['output']}")
        elif scene_changed:
            self.log.appendPlainText(f"Goal input changed: {result['field_path']} · accepted deposits retained")
        if action in {"new", "accept", "open_state"} or scene_changed:
            self._surface_scene(result)
        self._target_frames("target_history", result["target_history_points"], result["target_history_z_dirs"])
        if self.loaded_targets is not None:
            self._target_frames("target_imported", self.loaded_targets["points"], self.loaded_targets["z_dirs"])
        self._domain_scene(result)
        if action == "accept":
            self.preview = None
            self.preview_revision = -1
            self._clear_preview()
            self.remaining = max(0, self.remaining - 1)
            self.log.appendPlainText(f"Cycle {result['cycle']} · {result['extra']['targets']} deposits · "
                f"update {result['worker_s']:.2f} s · domain {result['extra']['domain_change']}")
            if result["extra"]["domain_change"] == "expanded":
                self.log.appendPlainText("Domain expanded: " + ", ".join(result["extra"]["domain"]["expanded_sides"]))
            if not self.remaining:
                self.running = False
        elif action == "export":
            try:
                self.plotter.render_window.MakeCurrent()
                self.plotter.screenshot(str(result["extra"] / "viewport.png"))
            except Exception as exc:
                self.log.appendPlainText(f"Screenshot: {exc}")
            self.log.appendPlainText(f"Exported: {result['extra']}")
        elif action == "save_state":
            self.log.appendPlainText(f"Simulation state saved: {result['extra']}")
        if action == "open_state":
            self.log.appendPlainText(f"State restored at cycle {result['cycle']} · ready to continue")
        elif action == "offset" and job["settings_revision"] == self.settings_revision:
            c = result["extra"]
            self._lines("offset", "offset", c.offset_points, c.offset_lines, "#e34bd3")
        elif action == "phantom" and job["settings_revision"] == self.settings_revision:
            if self.preview is not None and self.preview.token == job["token"]:
                self._draw_phantom(result["extra"])
        if result["preview"] is not None and action in {"new", "preview", "open_state"}:
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
            if self.startup_camera:
                try:
                    apply_camera_settings(self.plotter, self.startup_camera)
                except (ValueError, KeyError, TypeError):
                    self.paper_view()
                self.startup_camera = None
            else:
                self.paper_view()
        elif action == "open_state":
            apply_camera_settings(self.plotter, result["extra"]["view"]["camera"])
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
        configure_studio_lighting(self.plotter, self.light_intensity.value(), self.display_defaults)
        set_contact_shading(self.plotter, self.contact_shading.isChecked(), self.display_defaults)
        self.overlay = attach_viewer(self.plotter)
        self.actors.clear()
        self.meshes.clear()
        self.edge_sources.clear()
        self.frame_sources.clear()
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
                **surface_material(self.display_defaults),
                name="deposits", reset_camera=False)
            self.layer_changed("deposits", self.layers["deposits"].isChecked())
            self.layer_changed("bead_edges", self.layers["bead_edges"].isChecked())
        self._cloud("last", "last", result["last_targets"], "#73a8ce", 9)

    def _domain_scene(self, result):
        domain = result["domain"]
        if domain is None:
            return
        state = "Planned initial domain" if result["domain_planned"] else "Active DDS domain"
        low = ", ".join(f"{v * 1000:.1f}" for v in domain.min_corner)
        high = ", ".join(f"{v * 1000:.1f}" for v in domain.max_corner)
        self.layers["domain"].setToolTip(f"{state}\nGrid: {domain.grid_shape}\nMin XYZ (mm): {low}\nMax XYZ (mm): {high}")
        bounds = tuple(value for pair in zip(domain.min_corner, domain.max_corner) for value in pair)
        if self.meshes.get("domain_bounds") == bounds:
            return
        self.meshes["domain_bounds"] = bounds
        actor = self.plotter.add_mesh(pv.Box(bounds=bounds), name="domain",
            color="#8abacb", opacity=self.domain_opacity.value(), lighting=False,
            show_edges=True, edge_color="#558ca3", pickable=False,
            reset_camera=False, render=False)
        actor.SetUseBounds(False)
        actor.SetVisibility(self.layers["domain"].isChecked())
        self.actors["domain"] = actor

    def domain_opacity_changed(self, value):
        actor = self.actors.get("domain")
        if actor is not None:
            actor.GetProperty().SetOpacity(value)
            self.plotter.render()

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
        points, z_dirs = preview.targets.flattened_points_and_z_dirs()
        self._target_frames("target_preview", points, z_dirs)
        self._lines("paths", "paths", points, np.arange(len(points)).reshape(-1, 2), "#e8a029")
        self._lines("offset", "offset", np.empty((0, 3)), np.empty((0, 2), int), "#e34bd3")
        if self.layers["offset"].isChecked():
            QtCore.QTimer.singleShot(0, lambda: self.submit("offset"))

    def _clear_preview(self):
        self._draw_phantom(None)
        self._target_frames("target_preview", np.empty((0, 3)), np.empty((0, 3)))
        for name in ("sources", "candidates"):
            self._cloud(name, name, np.empty((0, 3)), "#ffffff", 1)
        for name in ("contour", "paths", "offset"):
            self._lines(name, name, np.empty((0, 3)), np.empty((0, 2), int), "#ffffff")

    def layer_changed(self, key, checked):
        if key in {"target_history", "target_preview", "target_imported"}:
            if checked:
                if key == "target_history" and self.latest is not None:
                    self._target_frames(key, self.latest["target_history_points"], self.latest["target_history_z_dirs"])
                elif key == "target_preview" and self.preview is not None and self.preview_revision == self.settings_revision:
                    points, z_dirs = self.preview.targets.flattened_points_and_z_dirs()
                    self._target_frames(key, points, z_dirs)
                elif key == "target_imported" and self.loaded_targets is not None:
                    self._target_frames(key, self.loaded_targets["points"], self.loaded_targets["z_dirs"])
            for axis in "xyz":
                actor = self.actors.get(f"{key}_{axis}")
                if actor is not None:
                    actor.SetVisibility(checked)
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

    def _target_frames(self, key, points, z_dirs):
        length = self.target_axis_length.value()
        style = (self.target_starts.isChecked(), self.target_endpoints.isChecked(), self.target_axis_thickness.value())
        source = self.frame_sources.get(key)
        if source is not None and source[0] is points and source[1] is z_dirs and source[2] == length and source[3:] == style:
            return
        if len(points) and not self.layers[key].isChecked():
            # Retain the prior hidden actors, rebuilding lazily on re-enable.
            return
        for axis in "xyz":
            name = f"{key}_{axis}"
            self.plotter.remove_actor(name, render=False)
            self.actors.pop(name, None)
        self.frame_sources[key] = (points, z_dirs, length, *style)
        selected_points, selected_dirs = select_target_frames(points, z_dirs, style[0], style[1])
        if not len(selected_points):
            return
        meshes = frame_glyphs(selected_points, selected_dirs, length, thickness=style[2])
        for axis, color, mesh in zip("xyz", AXIS_COLORS, meshes):
            name = f"{key}_{axis}"
            actor = self.plotter.add_mesh(mesh, name=name, color=color, lighting=False,
                pickable=False, reset_camera=False, render=False, show_scalar_bar=False)
            actor.mapper.scalar_visibility = False
            actor.SetUseBounds(False)
            actor.SetVisibility(self.layers[key].isChecked())
            self.actors[name] = actor

    def target_axis_length_changed(self, _value):
        if self.latest is not None:
            self._target_frames("target_history", self.latest["target_history_points"], self.latest["target_history_z_dirs"])
        if self.preview is not None and self.preview_revision == self.settings_revision:
            points, z_dirs = self.preview.targets.flattened_points_and_z_dirs()
            self._target_frames("target_preview", points, z_dirs)
        if self.loaded_targets is not None:
            self._target_frames("target_imported", self.loaded_targets["points"], self.loaded_targets["z_dirs"])
        self.plotter.render()

    def _draw_phantom(self, surface):
        self.plotter.remove_actor("phantom", render=False)
        self.actors.pop("phantom", None)
        if surface is not None and not surface.is_empty:
            actor = self.plotter.add_mesh(polydata(surface.vertices, surface.faces),
                name="phantom", color=self.display_defaults.phantom_color, opacity=self.phantom_opacity.value(),
                smooth_shading=True, **surface_material(self.display_defaults),
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
        set_contact_shading(self.plotter, enabled, self.display_defaults)
        self.plotter.render()

    def light_intensity_changed(self, value):
        configure_studio_lighting(self.plotter, value, self.display_defaults)
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

    def save_captures(self):
        if self.busy or not self.initialized:
            return
        camera = capture_camera(self.plotter)
        destination = None
        try:
            root = Path(self.latest["output"]) / "captures"
            root.mkdir(exist_ok=True)
            prefix = f"cycle_{self.latest['cycle']:04d}_{datetime.now():%Y%m%d-%H%M%S}_"
            destination = Path(tempfile.mkdtemp(prefix=prefix, dir=root))
            views = (("paper", self.paper_view),
                     ("front", lambda: self.standard_view((0, -1, 0), (0, 0, 1))),
                     ("top", lambda: self.standard_view((0, 0, 1), (0, 1, 0))))
            for name, apply_view in views:
                apply_view()
                self.plotter.render_window.MakeCurrent()
                self.plotter.screenshot(str(destination / f"{name}.png"), return_img=False)
            self.log.appendPlainText(f"Saved Paper, Front and Top captures: {destination}")
            return destination
        except Exception as exc:
            self.log.appendPlainText(f"Capture failed: {exc}" + (f" · folder: {destination}" if destination else ""))
        finally:
            restore_camera(self.plotter, camera)
            self.plotter.render()

    def paper_view(self):
        self.standard_view((1, -1, 0.8), (0, 0, 1))

    def standard_view(self, direction, up):
        self.plotter.enable_parallel_projection()
        self.plotter.view_vector(direction, viewup=up, render=False)
        if "goal" in self.actors:
            bounds = np.asarray(self.actors["goal"].GetBounds()).reshape(3, 2)
            self.plotter.reset_camera(bounds=(bounds + (-0.04, 0.04)).ravel(), render=False)
        self.plotter.renderer.camera.is_set = True
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
