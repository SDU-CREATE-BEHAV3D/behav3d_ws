"""Run the pinned scalar-field simulation using this Python environment."""
from datetime import datetime
from pathlib import Path
import os
import subprocess
import sys

workbench = Path(__file__).resolve().parent
root = workbench.parents[1]
for variable, folder in [("MPLCONFIGDIR", "matplotlib"), ("TEMP", "tmp"), ("TMP", "tmp")]:
    cache = workbench / ".cache" / folder
    cache.mkdir(parents=True, exist_ok=True)
    os.environ[variable] = str(cache)
output = workbench / "outputs" / datetime.now().strftime("simulation-%Y%m%d-%H%M%S")
if not any(arg == "--field-state" or arg.startswith("--field-state=") for arg in sys.argv[1:]):
    from prepare_field import prepare_field
    prepare_field(root)

command = [
    sys.executable, "-u",
    str(root / "src" / "behav3d_py" / "behav3d_py" / "scalar_field" / "field_state_scan_loop_simulator_dds_v2.py"),
    "--field-state", str(root / "mesh" / "fields" / "field_state_init_subdiv1.npz"),
    "--scan-mesh", str(root / "mesh" / "ScanMesh.stl"),
    "--scan-scale", "0.001",
    "--scan-yaw-deg", "180",
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
