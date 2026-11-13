#!/usr/bin/env python3
"""Sequential paired runtime trials, with immutable outputs for every repetition."""
import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
from experiment import write_json

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--data", required=True)
p.add_argument("--output", type=Path, required=True)
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
env = dict(os.environ, OMP_NUM_THREADS="4", OPENBLAS_NUM_THREADS="1", MPLBACKEND="Agg", MPLCONFIGDIR="/tmp/j1-matplotlib", PYTHONDONTWRITEBYTECODE="1")
results = {}
for case in ("orbit", "july29_c20"):
    results[case] = {}
    for trial in range(3):
        # Alternate order to reduce a systematic warm-cache advantage.
        variants = ("baseline", "production_rectified") if trial % 2 == 0 else ("production_rectified", "baseline")
        for variant in variants:
            output = args.output / f"{case}_{variant}_{trial}"
            command = [sys.executable, "-u", "-B", str(Path(__file__).with_name("experiment.py")), "--data", args.data, "--case", case, "--output", str(output), "--variant", variant, "--crop-width", "0", "--crop-height", "0"]
            if variant == "production_rectified":
                command.append("--working-tree")
            with output.with_suffix(".log").open("x") as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
            run = json.loads((output / "run.json").read_text())
            timing = {key: run[key] for key in ("color_alignment_seconds", "tsdf_seconds")}
            timing["pipeline_seconds"] = sum(timing.values())
            results[case].setdefault(variant, []).append(timing)
            print(case, variant, trial, round(timing["pipeline_seconds"], 3), flush=True)
summary = {case: {variant: {key: statistics.median(t[key] for t in trials) for key in trials[0]} for variant, trials in values.items()} for case, values in results.items()}
write_json(args.output / "timings.json", dict(trials=results, median=summary,
    note="CPU, 4 OpenMP threads, 1 OpenBLAS thread; sequential paired trials. Excludes imports, staging, hashing and evaluation. Includes image processing, TSDF, point cloud filtering and output writes. Other host activity is uncontrolled."))
