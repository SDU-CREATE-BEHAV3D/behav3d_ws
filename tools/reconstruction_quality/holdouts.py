#!/usr/bin/env python3
"""Fixed interleaved-view ablation; an exploratory consistency check, not ground truth."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import yaml
from experiment import CASES, write_json

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--data", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--crop-width", type=int, default=350)
p.add_argument("--crop-height", type=int, default=350)
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
env = dict(os.environ, OMP_NUM_THREADS="4", OPENBLAS_NUM_THREADS="1", MPLBACKEND="Agg", MPLCONFIGDIR="/tmp/j1-matplotlib", PYTHONDONTWRITEBYTECODE="1")
protocol = {}
for case, (session, scan) in CASES.items():
    count = len(yaml.safe_load((args.data / session / scan / "manifest.yaml").read_text())["captures"])
    test = list(range(3, count, 4))
    protocol[case] = dict(training=[i for i in range(count) if i not in test], heldout=test)
write_json(args.output / "protocol.json", dict(split="Every fourth manifest position starting at 3; frozen before subset replay", crop=[args.crop_width, args.crop_height], cases=protocol, limitation="Exploratory ablation; all-view summaries were inspected before this check. No pose fitting or ground truth."))
for case, split in protocol.items():
    for variant in ("baseline", "rectified"):
        dest = args.output / variant / case
        dest.parent.mkdir(exist_ok=True)
        with (dest.parent / f"{case}.log").open("x") as log:
            command = [sys.executable, "-u", "-B", str(Path(__file__).with_name("experiment.py")), "--data", str(args.data), "--case", case, "--output", str(dest), "--variant", variant, "--crop-width", str(args.crop_width), "--crop-height", str(args.crop_height), "--frames", ",".join(map(str, split["training"]))]
            print("START", case, variant, flush=True)
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
            subprocess.run([sys.executable, "-u", "-B", str(Path(__file__).with_name("evaluate.py")), str(dest), "--frames", ",".join(map(str, split["heldout"]))], stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
        print("DONE", case, variant, flush=True)
