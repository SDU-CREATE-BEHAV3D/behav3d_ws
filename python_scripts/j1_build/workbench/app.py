"""Launch the persistent desktop workbench with the existing Python stack."""
from pathlib import Path
import argparse
import os
import sys


def paths():
    here = Path(__file__).resolve().parent
    if (here.parent / "scalar_field").is_dir():
        data = here.parent
        scalar = data / "scalar_field"
    else:
        data = next(parent for parent in here.parents if (parent / "src/behav3d_py/behav3d_py/scalar_field").is_dir())
        scalar = data / "src/behav3d_py/behav3d_py/scalar_field"
    sys.path.insert(0, str(scalar))
    sys.path.insert(0, str(here.parent))
    for name, folder in (("MPLCONFIGDIR", "matplotlib"), ("TEMP", "tmp"), ("TMP", "tmp")):
        directory = here / ".cache" / folder
        directory.mkdir(parents=True, exist_ok=True)
        os.environ[name] = str(directory)
    return data, here / "outputs"


def main():
    data, output = paths()
    from workbench.config import InputOptions
    from workbench.preferences import load_defaults, options_from_dict
    try:
        saved = load_defaults()
    except (ValueError, OSError, TypeError) as exc:
        print(f"Saved defaults could not be loaded; using config.py: {exc}")
        saved = {}
    inputs = options_from_dict(InputOptions, saved.get("inputs", {}))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--field-state", type=Path)
    parser.add_argument("--scan-mesh", type=Path, help="Override InputOptions.scan_mesh in config.py")
    parser.add_argument("--scan-scale", type=float, default=inputs.scan_scale,
                        help="Override InputOptions.scan_scale in config.py")
    parser.add_argument("--scan-yaw", type=float, default=inputs.scan_yaw,
                        help="Override InputOptions.scan_yaw in config.py (degrees around Z)")
    parser.add_argument("--off-screen", action="store_true")
    args = parser.parse_args()
    original_path = Path(inputs.original_field_state).expanduser() if inputs.original_field_state and args.field_state is None else None
    if original_path is not None and not original_path.is_absolute():
        original_path = data / original_path
    if args.field_state is None and inputs.field_state:
        args.field_state = Path(inputs.field_state).expanduser()
        if not args.field_state.is_absolute():
            args.field_state = data / args.field_state
    scan_path = args.scan_mesh
    if scan_path is None:
        scan_path = Path(inputs.scan_mesh).expanduser()
        if not scan_path.is_absolute():
            scan_path = data / scan_path
    if not args.field_state and not (data / "mesh/fields/field_state_init_subdiv1.npz").exists():
        sys.path.insert(0, str(data / "python_scripts/j1_build"))
        from prepare_field import prepare_field
        prepare_field(data)
    from PySide6 import QtCore, QtWidgets
    QtCore.QLocale.setDefault(QtCore.QLocale("en_US"))
    from workbench.ui import Workbench
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    window = Workbench(dict(data_root=data, output_root=output, field_path=args.field_state,
        scan_path=scan_path, scan_scale=args.scan_scale, scan_yaw=args.scan_yaw,
        original_path=original_path), off_screen=args.off_screen)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
