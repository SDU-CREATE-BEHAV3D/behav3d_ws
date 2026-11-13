#!/usr/bin/env python3
"""Generate a compact, inspectable summary from completed experiment records."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from experiment import CASES, REVISION, write_json

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--artifacts", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
(args.output / "figures").mkdir(exist_ok=True)
read = lambda path: json.loads(path.read_text())
profiles = ["baseline_v1", "rectified_v1", "crop350_v1", "rectified350_v1", "baseline_full", "rectified_full", "rectified350_weight2", "rectified350_voxel1", "rectified350_trunc8"]
summary = dict(baseline_commit=REVISION, artifact_root=str(args.artifacts), cases=CASES, profiles={}, holdouts={}, final_runs={})
for profile in profiles:
    summary["profiles"][profile] = {}
    for case in CASES:
        path = args.artifacts / profile / case / "evaluation.json"
        if path.exists():
            result = read(path)
            # Per-frame detail remains in the original evaluation file.
            for region in ("depth", "depth_full_frame"):
                for model in result.get(region, {}).values():
                    model.pop("per_frame", None)
            summary["profiles"][profile][case] = result
for profile in ("holdout350_v1", "holdout_full_v1"):
    summary["holdouts"][profile] = {}
    for variant in ("baseline", "rectified"):
        summary["holdouts"][profile][variant] = {}
        for case in CASES:
            result = read(args.artifacts / profile / variant / case / "heldout_evaluation.json")
            for region in ("depth", "depth_full_frame"):
                for model in result.get(region, {}).values():
                    model.pop("per_frame", None)
            summary["holdouts"][profile][variant][case] = result
for case in CASES:
    summary["final_runs"][case] = read(args.artifacts / "final_candidate_v2" / case / "run.json")
summary["validation"] = {name: read(args.artifacts / f"{name}.json") for name in ("baseline_reproducibility", "production_equivalence", "final_equivalence")}
summary["runtime"] = read(args.artifacts / "runtime_trials/timings.json")
inventory = read(args.artifacts / "saved_inventory.json")
summary["inventory"] = [{key: row[key] for key in ("session", "scan", "frames", "missing", "invalid_poses", "duplicate_depth_files", "mesh")} for row in inventory]
write_json(args.output / "summary.json", summary)

lines = ["# Reconstruction comparison results", "", f"Baseline: `{REVISION}`. Same saved inputs, calibration and poses in each paired comparison.", "", "## Matched full-frame optical-model comparison", "", "Both columns use full-frame depth, 1.953125 mm voxels, 15.625 mm truncation and weight 3. Only distortion correction changes. Object regions are fixed raw-depth proxies 15–500 mm above the saved table plane. These are sensor-consistency measures, not ground-truth errors.", "", "| Case | Object p90 absolute residual, J1 → corrected (mm) | Object rays within 5 mm, J1 → corrected | Interior hit coverage, J1 → corrected |", "|---|---:|---:|---:|"]
for case in CASES:
    a,b=[summary["profiles"][v][case]["depth"]["saved_distortion"] for v in ("baseline_full", "rectified_full")]
    x,y=a["object_interior"],b["object_interior"]
    lines.append(f"| {case} | {x['p90_abs_mm']:.2f} → {y['p90_abs_mm']:.2f} | {100*x['within_5mm_fraction']:.1f}% → {100*y['within_5mm_fraction']:.1f}% | {100*a['interior']['hit_fraction']:.1f}% → {100*b['interior']['hit_fraction']:.1f}% |")
lines += ["", "## Withheld-view comparison", "", "Every fourth view (positions 3, 7, …) is omitted from fusion and used only for evaluation. The table reports object-region rays in the fixed central ROI. This is exploratory validation on already inspected sessions, not a blinded external test.", "", "| Case | Held-out object p90, J1 → corrected (mm) | Held-out object rays within 5 mm, J1 → corrected |", "|---|---:|---:|"]
for case in CASES:
    a,b=[summary["holdouts"]["holdout_full_v1"][v][case]["depth"]["saved_distortion"]["object_interior"] for v in ("baseline", "rectified")]
    lines.append(f"| {case} | {a['p90_abs_mm']:.2f} → {b['p90_abs_mm']:.2f} | {100*a['within_5mm_fraction']:.1f}% → {100*b['within_5mm_fraction']:.1f}% |")
lines += ["", "![Object-region held-out metrics](figures/metrics.png)", "", "## Coverage versus the original default crop", "", "This comparison combines two changes: distortion correction and removal of the early image crop. The matched tables above isolate the correction. Hit coverage is measured on the same raw rays; missing hits count against it.", "", "| Case | J1 default 270×290 interior hits | Corrected full-frame interior hits |", "|---|---:|---:|"]
for case in CASES:
    a,b=[summary["profiles"][v][case]["depth"]["saved_distortion"]["interior"] for v in ("baseline_v1", "rectified_full")]
    lines.append(f"| {case} | {100*a['hit_fraction']:.1f}% | {100*b['hit_fraction']:.1f}% |")
lines += ["", "## Other controlled settings", "", "The rows below retain rectification and the 350×350 crop. Smaller voxel tests keep the physical truncation distance constant. Lower residuals on surviving hits must be weighed against deleted coverage and fragmentation.", "", "| Case | Setting | Interior p90 (mm) | Interior hit coverage | Components | Triangles |", "|---|---|---:|---:|---:|---:|"]
for case in ("orbit", "july28_c19", "july29_c20"):
    for profile,label in [("rectified350_v1","reference"),("rectified350_weight2","weight 2"),("rectified350_voxel1","1 mm voxels"),("rectified350_trunc8","7.8125 mm truncation")]:
        r=summary["profiles"][profile][case];d=r["depth"]["saved_distortion"]["interior"]
        lines.append(f"| {case} | {label} | {d['p90_abs_mm']:.2f} | {100*d['hit_fraction']:.1f}% | {r['mesh']['components']} | {r['mesh']['triangles']:,} |")
lines += ["", "## Runtime", "", "Median of three sequential paired runs, alternating order, CPU with 4 OpenMP / 1 OpenBLAS / 1 OpenCV threads. Both profiles use full-frame images. Timing includes alignment, rectification where enabled, TSDF, filtering and output writes; excludes interpreter import, input copying, hashing and evaluation. Other host activity is uncontrolled.", "", "| Case | J1 pipeline (s) | Corrected pipeline (s) | Added time (s) |", "|---|---:|---:|---:|"]
for case, values in summary["runtime"]["median"].items():
    a,b=values["baseline"]["pipeline_seconds"],values["production_rectified"]["pipeline_seconds"]
    lines.append(f"| {case} | {a:.3f} | {b:.3f} | {b-a:.3f} |")
lines += ["", "## Validation and remaining uncertainty", "", "- All seven legacy 350×350 replays match their baseline geometry exactly.", "- All seven final production candidates match the independent full-frame rectification experiment exactly: identical vertices and triangle connectivity after index remapping.", "- Nine focused capture/rectification tests pass. Full ROS/ament lint and live integration were not available on macOS.", "- Fine detail, absolute dimensions, hand–eye accuracy, depth bias, pose latency and color accuracy are not independently verified. Residuals can share calibration/systematic errors with the inputs.", "- The orbit held-out object p90 remains above 6 mm. Sparse-view surfaces retain occlusions and holes; no hole filling or smoothing is used to conceal them.", "- Both pinhole and saved-distortion ray models, full-frame evaluation, mesh topology and plane-fit metrics remain in `summary.json` and the detailed run records.", "", "No merge or live-system default change has been made. See [the workflow and investigation](INVESTIGATION.md) and [reproduction instructions](../../tools/reconstruction_quality/README.md).", ""]
(args.output / "RESULTS.md").write_text("\n".join(lines))

fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
order=list(CASES)
for ax, key, title, unit in [(axes[0],"p90_abs_mm","Held-out object residual (lower is better)","p90 absolute residual (mm)"),(axes[1],"within_5mm_fraction","Held-out observed support (higher is better)","Valid object rays with mesh within 5 mm (%)")]:
    for variant,offset,color,label in [("baseline",-.17,"#8293a6","J1 full frame"),("rectified",.17,"#137b78","Distortion corrected")]:
        values=[summary["holdouts"]["holdout_full_v1"][variant][case]["depth"]["saved_distortion"]["object_interior"][key] for case in order]
        if key.endswith("fraction"): values=np.array(values)*100
        ax.barh(np.arange(len(order))+offset, values, height=.31, color=color, label=label)
    ax.set(yticks=np.arange(len(order)), yticklabels=order, title=title, xlabel=unit)
    ax.invert_yaxis(); ax.spines[["top","right"]].set_visible(False); ax.grid(axis="x",alpha=.15); ax.set_axisbelow(True)
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.18), ncol=2, fontsize=8)
fig.suptitle("Same inputs and poses · complete frames · 1.95 mm voxels · no ground truth",fontsize=11)
fig.savefig(args.output / "figures/metrics.png", dpi=180)
print(args.output)
