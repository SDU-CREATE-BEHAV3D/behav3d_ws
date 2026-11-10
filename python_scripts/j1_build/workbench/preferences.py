"""Small local startup preferences; config.py remains the built-in fallback."""
from dataclasses import asdict, fields
import json
from pathlib import Path
import math

from .config import InputOptions, Parameters, SimulationOptions, DisplayOptions

DEFAULTS_PATH = Path(__file__).with_name("user_defaults.json")


def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(path.name + ".pending")
    pending.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
    pending.replace(path)


def options_from_dict(cls, values):
    if not isinstance(values, dict):
        raise ValueError("Settings must be a JSON object")
    allowed = {item.name for item in fields(cls)}
    values = {key: value for key, value in values.items() if key in allowed}
    builtins = asdict(cls())
    for key, value in values.items():
        default = builtins[key]
        if isinstance(default, bool):
            valid = isinstance(value, bool)
        elif isinstance(default, (int, float)):
            valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            if isinstance(default, int):
                valid = valid and int(value) == value
        elif isinstance(default, str):
            valid = isinstance(value, str)
        else:
            valid = isinstance(value, dict) and all(isinstance(v, bool) for v in value.values())
        if not valid:
            raise ValueError(f"Invalid setting: {key}")
    if cls is DisplayOptions and "layers" in values:
        values["layers"] = {**DisplayOptions().layers, **values["layers"]}
    result = cls(**values)
    if hasattr(result, "validate"):
        result.validate()
    return result


def load_defaults(path=DEFAULTS_PATH):
    path = Path(path)
    if not path.exists():
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("version") != 1:
        raise ValueError("Unsupported defaults version")
    for key, cls in (("inputs", InputOptions), ("parameters", Parameters),
                     ("simulation", SimulationOptions), ("display", DisplayOptions)):
        options_from_dict(cls, payload.get(key, {}))
    if payload.get("camera"):
        validate_camera_settings(payload["camera"])
    return payload


def save_defaults(path, inputs, parameters, simulation, display, camera):
    parameters.validate()
    simulation.validate()
    write_json(path, dict(version=1, inputs=inputs, parameters=asdict(parameters),
                         simulation=asdict(simulation), display=asdict(display), camera=camera))


def camera_settings(plotter):
    c = plotter.renderer.camera
    return dict(position=c.position, focal_point=c.focal_point, up=c.up,
                parallel_scale=c.parallel_scale, parallel_projection=c.parallel_projection,
                view_angle=c.view_angle)


def validate_camera_settings(values):
    if not isinstance(values, dict) or not all(key in values for key in (
        "position", "focal_point", "up", "parallel_scale", "parallel_projection", "view_angle")):
        raise ValueError("Incomplete saved camera")
    for name in ("position", "focal_point", "up"):
        if len(values[name]) != 3 or not all(math.isfinite(v) for v in values[name]):
            raise ValueError("Invalid saved camera")
    if values["parallel_scale"] <= 0 or not 0 < values["view_angle"] < 180:
        raise ValueError("Invalid saved camera scale or field of view")
    if not math.isfinite(values["parallel_scale"]) or sum(v*v for v in values["up"]) < 1e-12:
        raise ValueError("Invalid saved camera up vector or scale")
    if sum((a-b)**2 for a, b in zip(values["position"], values["focal_point"])) < 1e-20:
        raise ValueError("Saved camera position must differ from its target")


def apply_camera_settings(plotter, values):
    validate_camera_settings(values)
    c = plotter.renderer.camera
    for key in ("position", "focal_point", "up", "parallel_scale", "parallel_projection", "view_angle"):
        setattr(c, key, values[key])
    c.is_set = True
    plotter.reset_camera_clipping_range()
