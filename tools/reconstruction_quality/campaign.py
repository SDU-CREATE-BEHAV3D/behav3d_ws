#!/usr/bin/env python3
"""Run sequential, logged cases; never overwrite a previous experiment."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--data", required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--variant", default="baseline")
parser.add_argument("--cases", nargs="+", default=["june_c1", "july17_c0", "orbit", "july28_c6", "july28_c19", "july29_c0", "july29_c20"])
parser.add_argument("--crop-width", type=int, default=270)
parser.add_argument("--crop-height", type=int, default=290)
parser.add_argument("--working-tree", action="store_true")
parser.add_argument("--voxel-size", type=float, default=1/512)
parser.add_argument("--truncation-m", type=float, default=8/512)
parser.add_argument("--weight-threshold", type=float, default=3.0)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
env = dict(os.environ, OMP_NUM_THREADS="4", OPENBLAS_NUM_THREADS="1", MPLBACKEND="Agg", MPLCONFIGDIR="/tmp/j1-matplotlib", PYTHONDONTWRITEBYTECODE="1")
for case in args.cases:
    dest = args.output / case
    with (args.output / f"{case}.log").open("x") as log:
        command = [sys.executable, "-u", "-B", str(Path(__file__).with_name("experiment.py")), "--data", args.data, "--case", case, "--output", str(dest), "--variant", args.variant, "--crop-width", str(args.crop_width), "--crop-height", str(args.crop_height)]
        if args.working_tree:
            command.append("--working-tree")
        command.extend(["--voxel-size", str(args.voxel_size), "--truncation-m", str(args.truncation_m), "--weight-threshold", str(args.weight_threshold)])
        print("START", case, args.variant, flush=True)
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
        print("DONE", case, flush=True)
