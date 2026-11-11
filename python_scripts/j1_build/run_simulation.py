"""Run the pinned scalar-field simulation using this Python environment."""
from datetime import datetime
from pathlib import Path
import os
import subprocess
import sys

script_dir = Path(__file__).resolve().parent
root = script_dir.parents[1]
from workbench.config import InputOptions
from workbench.preferences import load_defaults, options_from_dict

try:
    inputs = options_from_dict(InputOptions, load_defaults().get("inputs", {}))
except (ValueError, OSError, TypeError) as exc:
    print(f"Saved defaults could not be loaded; using config.py: {exc}")
    inputs = InputOptions()
field_path = Path(inputs.field_state or InputOptions().field_state).expanduser()
scan_path = Path(inputs.scan_mesh).expanduser()
if not field_path.is_absolute():
    field_path = root / field_path
if not scan_path.is_absolute():
    scan_path = root / scan_path
for variable, folder in [("MPLCONFIGDIR", "matplotlib"), ("TEMP", "tmp"), ("TMP", "tmp")]:
    cache = script_dir / ".cache" / folder
    cache.mkdir(parents=True, exist_ok=True)
    os.environ[variable] = str(cache)
output = script_dir / "outputs" / datetime.now().strftime("simulation-%Y%m%d-%H%M%S")
command = [
    sys.executable, "-u",
    str(root / "src/behav3d_py/behav3d_py/scalar_field/field_state_scan_loop_simulator_dds_v2.py"),
    "--field-state", str(field_path),
    "--scan-mesh", str(scan_path),
    "--scan-scale", str(inputs.scan_scale),
    "--scan-yaw-deg", str(inputs.scan_yaw),
    "--output-dir", str(output),
    "--candidate-mode", "gradient_lift",
    "--contour-sample-spacing-mm", "3",
    "--offset-distance-mm", "12",
    "--offset-geodesic-delta-mm", "0.6",
    "--beads-per-step", "7",
    "--bead-separation-mm", "16",
    "--bead-width-mm", "42",
    "--bead-height-mm", "20",
    "--dds-deposit-mode", "line",
    "--dds-line-fraction", "0.22",
    *sys.argv[1:],
]
raise SystemExit(subprocess.call(command, cwd=root))
