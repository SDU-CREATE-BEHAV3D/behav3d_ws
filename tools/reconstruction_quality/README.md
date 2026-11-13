# Offline reconstruction quality investigation

Baseline: `e0d25121105883fb6958c00a690329e839132c7b` (`J1_Build`, verified 2026-11-13).

The experiment runner preserves source captures and writes only to a new output
directory. It runs the production reconstruction modules without ROS or robot
connections. Original saved outputs are references, not ground truth. See the
[investigation](../../reports/reconstruction_quality/INVESTIGATION.md) and
[results](../../reports/reconstruction_quality/RESULTS.md).

## Tested environment

Python 3.11.14 on macOS; Open3D 0.19.0, NumPy 2.4.6, SciPy 1.17.1,
opencv-python 4.13.0.92, PyYAML 6.0.3, Matplotlib 3.11.0. Existing environment:
`/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python`.
No dependencies were installed or changed there. CPU trials use four OpenMP
threads, one OpenBLAS thread and one OpenCV thread. Every `run.json` records the
platform, interpreter, package versions and source/input/output hashes.

## Reproduce

Run from the repository root using that interpreter (or a matching environment).
Choose a new output path for every command; previous experiment directories are
never overwritten. These commands **copy** data from Downloads before running
reconstruction. Do not point the low-level production CLI at original captures
if their previous reconstruction outputs must be preserved.

```sh
python -B tools/reconstruction_quality/campaign.py \
  --data /Users/josephnamar/Downloads --output /path/to/new-default-baseline

python -B tools/reconstruction_quality/campaign.py \
  --data /Users/josephnamar/Downloads --output /path/to/new-full-baseline \
  --crop-width 0 --crop-height 0

python -B tools/reconstruction_quality/campaign.py \
  --data /Users/josephnamar/Downloads --output /path/to/new-candidate \
  --variant production_rectified --working-tree --crop-width 0 --crop-height 0

OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 python -B \
  tools/reconstruction_quality/evaluate.py /path/to/new-candidate/*/

python -B tools/reconstruction_quality/holdouts.py \
  --data /Users/josephnamar/Downloads --output /path/to/new-heldout-check \
  --crop-width 0 --crop-height 0
```

`experiment.py` defaults to Git-pinned source from the verified J1 commit.
`--working-tree` explicitly snapshots local reconstruction modules instead, with
hashes; it does not imply the branch is committed. `--variant rectified` rectifies
copied depth/RGB inputs, zeroes their staged distortion coefficients, then runs
unchanged J1 source. `production_rectified` uses the new production option with
untouched copied raw inputs. The two approaches are compared independently.

`--voxel-size`, `--truncation-m`, and `--weight-threshold` are harness-only
experimental overrides, recorded in `run.json`. Voxel-size experiments hold the
physical truncation distance constant. They do not change production defaults.
`--frames` selects original manifest positions for ablations, never by renaming
the original files. Edit `CASES` explicitly to add new capture sessions.

Additional tools:

- `inventory.py`: all saved manifests, file hashes, missing files and mesh topology.
- `evaluate.py`: fixed-ray coverage/residuals, object/table proxies, topology and table fits.
- `verify_geometry.py`: vertex and triangle equivalence independent of vertex ordering.
- `render.py`: fixed top-down height renders from complete triangle meshes.
- `render_detail.py`: fixed oblique renders and depth-change maps.
- `benchmark.py`: sequential, alternating-order runtime trials.
- `summarize.py`: compact review tables, evidence JSON and a metric chart.

The first default-baseline attempt failed before reconstruction due to a runner
keyword typo. Use `baseline_v1`, not the incomplete `baseline` directory in the
retained artifacts. Baseline reconstruction source itself was never edited.

## Focused tests

```sh
PYTHONPATH=src/behav3d_sense PYTHONDONTWRITEBYTECODE=1 python -B -m pytest \
  -p no:cacheprovider src/behav3d_sense/test/test_capture_validation.py \
  src/behav3d_sense/test/test_rectification.py -q
```

The ROS/ament lint suite needs the Linux ROS environment and was not available
on this Mac. No robot or ROS service should be launched for this offline study.
## November 2026 confidence audit

The follow-up findings are in `reports/reconstruction_quality/NOVEMBER_CONFIDENCE.md`.
These scripts inspect the November 4/13 experiment-folder layout. They do not
modify production reconstruction settings or the source Drive files. The local
artifact root used for the recorded results is:

```
/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence
```

Use the same Open3D/Python environment recorded in `integrity.json`, with
`OMP_NUM_THREADS=4`, `OPENBLAS_NUM_THREADS=1`, `PYTHONDONTWRITEBYTECODE=1`, and a
writable `MPLCONFIGDIR`. Set `NOVEMBER_SOURCE` to the original `E01 - Scan Test`
folder and `NOVEMBER_ROOT` to a **new** artifact directory. Run from the repository
root with the configured Python interpreter:

```sh
python -B tools/reconstruction_quality/november_inventory.py "$NOVEMBER_SOURCE" "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_confidence.py --data "$NOVEMBER_ROOT/data" --output "$NOVEMBER_ROOT/experiments" --variant legacy
python -B tools/reconstruction_quality/november_confidence.py --data "$NOVEMBER_ROOT/data" --output "$NOVEMBER_ROOT/experiments" --variant rectified
python -B tools/reconstruction_quality/november_contact_sheets.py "$NOVEMBER_ROOT/data" "$NOVEMBER_ROOT/contact_sheets"
python -B tools/reconstruction_quality/november_results.py "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_pcd_probe.py "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_prefix_replay.py "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_holdout.py "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_raw_geometry.py "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_review.py "$NOVEMBER_ROOT"
python -B tools/reconstruction_quality/november_holdout.py "$NOVEMBER_ROOT" --profiles rectified_trunc8 rectified_trunc4 rectified_fine8 --sessions 261113_150006 261113_150432 --experiment heldout_tuning
python -B tools/reconstruction_quality/november_integrity.py "$NOVEMBER_ROOT"
```

The full-volume audit extracts weights 3/2/1 from the same integrated grid. Its
observation instrumentation asserts equality against the production colorizer.
The prefix probe is parameter inference; the subsequent production replay is
the stronger verification. Unmatched scans are explicitly marked unresolved.
Holdout reconstruction excludes every fourth view before fusion. Geometry
comparisons use fixed raw-depth rays and retain missing-ray denominators. The
additional cube profiles are exploratory comparisons on that same holdout,
not a second independent validation set. Saved meshes and input hashes remain
available beside the experimental outputs.
