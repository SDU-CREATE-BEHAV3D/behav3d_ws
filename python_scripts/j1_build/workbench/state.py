"""Portable resumable snapshots without saved surface meshes or render caches."""
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import zipfile

import numpy as np
from dds import Simulator, SimulationResult

from .config import Parameters, SimulationOptions, DisplayOptions
from .preferences import options_from_dict, validate_camera_settings


def file_hash(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_state(session, destination, parameters, view):
    """Stream inputs and one DDS checkpoint into an atomically replaced archive."""
    parameters.validate()
    options_from_dict(DisplayOptions, view["display"])
    validate_camera_settings(view["camera"])
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    files = {}
    def asset(path):
        path = Path(path).resolve()
        key = f"assets/{file_hash(path)}{path.suffix.lower()}"
        files[key] = path
        return key
    assets = dict(scan=asset(session.scan_path), fixed_goal=asset(session.fixed_field_path),
                  original_goal=asset(session.original_field_path))
    width_path = Path(parameters.width_field_path).expanduser()
    if parameters.width_field_path and width_path.is_file():
        assets["current_width"] = asset(width_path)
    elif parameters.width_mode == "field":
        raise FileNotFoundError(f"Width field not found: {width_path}")
    history_inputs = {}
    for record in session.history:
        relative = record.get("inputs", {}).get("width_snapshot")
        if relative:
            history_inputs[relative] = asset(session.output / relative)
    metadata = dict(version=1, cycle=session.revision, simulation=asdict(session.options),
                    parameters=asdict(parameters), view=view, assets=assets,
                    history_inputs=history_inputs, history=session.history,
                    active_goal="original_goal" if session.field_path == session.original_field_path else "fixed_goal",
                    scan_scale=session.scan_scale, scan_yaw=session.scan_yaw)
    # A temporary sibling makes the final replacement atomic and leaves any
    # previous save intact if compression or input reading fails.
    with tempfile.TemporaryDirectory(prefix=".state-save-", dir=destination.parent) as directory:
        temporary = Path(directory)
        archive = temporary / "snapshot.b3dstate"
        with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=3) as bundle:
            bundle.writestr("state.json", json.dumps(metadata, allow_nan=False))
            for key, path in files.items():
                bundle.write(path, key)
            for cycle in range(1, session.revision + 1):
                key = f"step_{cycle:04d}/segments.yaml"
                bundle.write(session.output / key, key)
            if session.result is not None:
                session.result.checkpoint(temporary / "dds.npz")
                # The NPZ is already compressed; do not compress it twice.
                bundle.write(temporary / "dds.npz", "dds.npz", compress_type=zipfile.ZIP_STORED)
        archive.replace(destination)
    return destination


def load_state(source, paths):
    """Restore into a new output folder, retaining all original session files."""
    from .session import Session, load_field_state_npz, dds_surface_to_open3d_mesh, load_segment_targets_yaml
    output_root = Path(paths["output_root"])
    output_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".state-open-", dir=output_root) as directory:
        temporary = Path(directory)
        with zipfile.ZipFile(source) as bundle:
            metadata = json.loads(bundle.read("state.json"))
            if metadata.get("version") != 1:
                raise ValueError("Unsupported simulation state version")
            options = options_from_dict(SimulationOptions, metadata["simulation"])
            metadata["view"]["display"] = asdict(options_from_dict(DisplayOptions, metadata["view"]["display"]))
            validate_camera_settings(metadata["view"]["camera"])
            for entry in bundle.infolist():
                relative = PurePosixPath(entry.filename)
                if relative.is_absolute() or ".." in relative.parts or "\\" in entry.filename or ":" in entry.filename:
                    raise ValueError("Invalid path in simulation state")
                target = temporary.joinpath(*relative.parts)
                if entry.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with bundle.open(entry) as incoming, target.open("wb") as outgoing:
                        shutil.copyfileobj(incoming, outgoing, length=128 * 1024)
        assets = metadata["assets"]
        for relative in assets.values():
            if relative not in bundle.namelist():
                raise ValueError("State is missing an input asset")
        cycle = metadata["cycle"]
        if cycle != len(metadata["history"]):
            raise ValueError("State cycle does not match accepted history")
        result = None
        if cycle:
            with np.load(temporary / "dds.npz", allow_pickle=False) as stored:
                dds_metadata = json.loads(stored["_meta"].tobytes().decode("utf-8"))
                if np.prod(dds_metadata["domain"]["grid_shape"], dtype=np.int64) > options.max_voxels:
                    raise ValueError("State domain exceeds its voxel limit")
            result = SimulationResult.load(temporary / "dds.npz")
            if np.prod(result.domain.grid_shape, dtype=np.int64) > options.max_voxels:
                raise ValueError("State domain exceeds its voxel limit")
            if len(result.deposits) != metadata["history"][-1]["deposits"]:
                raise ValueError("State deposits do not match history")
        # Extracted assets are moved, not copied into a second full input tree.
        s = Session(data_root=paths["data_root"], output_root=output_root, options=options,
                    field_path=temporary / assets["fixed_goal"], original_path=temporary / assets["original_goal"],
                    scan_path=temporary / assets["scan"], scan_scale=metadata["scan_scale"], scan_yaw=metadata["scan_yaw"])
        asset_destination = (s.output / "assets").resolve()
        if not asset_destination.is_relative_to(output_root.resolve()):
            raise ValueError("Restored asset directory must stay within the output root")
        shutil.move(str(temporary / "assets"), str(asset_destination))
        s.fixed_field_path = s.output / assets["fixed_goal"]
        s.original_field_path = s.output / assets["original_goal"]
        s.scan_path = s.output / assets["scan"]
        s.field_path = s.output / assets[metadata["active_goal"]]
        s.field = load_field_state_npz(s.field_path)
        for relative, stored in metadata.get("history_inputs", {}).items():
            target = (s.output / relative).resolve()
            if not target.is_relative_to(s.output.resolve()):
                raise ValueError("Invalid historical input path")
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(s.output / stored, target)
        s.history = metadata["history"]
        for i, record in enumerate(s.history, 1):
            folder = s.output / f"step_{i:04d}"
            folder.mkdir()
            shutil.move(str(temporary / f"step_{i:04d}/segments.yaml"), str(folder / "segments.yaml"))
            applied = load_segment_targets_yaml(folder / "segments.yaml", position_scale=1000.0)
            if record["cycle"] != i or record["targets"] != applied.count:
                raise ValueError("State target files do not match accepted history")
            s.target_history.add_batch(folder / "segments.yaml", applied, i)
            (folder / "parameters.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
            s.last_targets = applied.end_points.copy()
        (s.output / "history.jsonl").write_text("".join(json.dumps(r) + "\n" for r in s.history), encoding="utf-8")
        s.revision = cycle
        if result is not None:
            s.result = result
            s.simulator = Simulator(result.domain, result.deposits)
            # DDS at the pinned version has no public from-result constructor.
            # Warm only its mutable union cache; immutable snapshots remain separate.
            if not hasattr(s.simulator, "_implicit_field_cache"):
                raise RuntimeError("This DDS version cannot restore its incremental field cache")
            s.simulator._implicit_field_cache = result.implicit_field.copy()
            s.surface = result.analysis.surface_mesh(threshold=options.threshold, step_size=options.surface_step)
            s.current_scan = s.base + dds_surface_to_open3d_mesh(s.surface)
        parameters = options_from_dict(Parameters, metadata["parameters"])
        parameters = replace(parameters, width_field_path=str(s.output / assets["current_width"]) if "current_width" in assets else "")
        parameters.validate()
        session_meta = json.loads((s.output / "session.json").read_text())
        session_meta.update(field=str(s.fixed_field_path), scan=str(s.scan_path), restored_from=str(Path(source).resolve()),
                            restored_cycle=cycle)
        (s.output / "session.json").write_text(json.dumps(session_meta, indent=2), encoding="utf-8")
        return s, parameters, metadata["view"]
