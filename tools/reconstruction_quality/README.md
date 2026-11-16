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
  src/behav3d_sense/test/test_rectification.py \
  src/behav3d_sense/test/test_reconstruction_workflow.py \
  src/behav3d_sense/test/test_colored_mesh.py -q
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

## November 13 distortion pairs and cube dimensions

`november_pair_run.py` performs fresh production reconstructions of all nine
November 13 scans with correction off/on, retaining extraction weight 3 and all
other reconstruction settings. It verifies the 381 original and snapshot input
files against the earlier inventory, waits for every aligned image, and writes
18 separate reconstruction folders. The original captures are read only.

```sh
python -B tools/reconstruction_quality/november_pair_run.py \
  --audit-root "$NOVEMBER_ROOT" --output /path/to/new-pairs
python -B tools/reconstruction_quality/november_cube_bounds.py \
  --audit-root "$NOVEMBER_ROOT" --root /path/to/new-pairs
```

The bounds script renders actual triangle meshes with matching views and scale.
It measures the three cube scans in a table-aligned coordinate frame: X follows
projected robot-base X, Y lies along the table, and height follows its normal.
Both versions use the same saved table plane. A broad 200 mm square region
locates the cube; points below three voxels (5.859375 mm) are excluded to remove
the table band. The box bottom is anchored to the table, and the remaining
vertices determine its X/Y limits and top. No nominal cube dimensions, smoothing,
or shape fitting are used. Protrusions contribute to the reported dimensions;
these are estimated mesh bounds, not independent physical measurements.

`cube_dimensions.json` records exact dimensions, bounding corners in meters,
mesh hashes, and sensitivity to two/four-voxel table exclusion. The script checks
the coordinate transforms using a known tilted cuboid and verifies that every
selected vertex is enclosed. Separate `cube_bounding_box.ply` line sets accompany
the six cube meshes. The figures folder contains all nine comparisons and six
individual dimensioned cube images.

Recorded run: `reconstruction_quality/november_cube_dimensions` beside the
November audit artifact folder above. `complete.json` confirms all 18 runs
finished and the snapshot inputs and production source stayed unchanged.

## ROS workflow verification and further refinement (16 November)

The final section of `reports/reconstruction_quality/NOVEMBER_CONFIDENCE.md`
records the J1 workflow trace, the new ROS defaults, and the refinement results.
`j1_workflow_verification.json` identifies the verified remote J1 commit and
pinned Orbbec driver sources. `refinement_summary.json` contains all nine scans.

The new ROS service default is rectification enabled. Standalone Python/CLI
rectification remains explicit. Both alignment modes publish `alignment.json`
only after every indexed image has been written successfully; the orchestrator
waits for that run's complete record. TSDF rejects incomplete or mismatched
alignment instead of silently using fewer frames. The completion protocol and
service parameter routing are covered by focused tests without a live ROS graph.

For reproduction, `PAIRS_ROOT` is the completed November distortion-pair folder,
`REFINEMENT_ROOT` is a new output root, and `NOVEMBER_ROOT` is the original audit
folder. Use the interpreter/thread settings documented above. Source captures
are read only; new output directories must not already exist.

```sh
python -B tools/reconstruction_quality/november_ros_replay.py \
  --audit "$NOVEMBER_ROOT" --pairs "$PAIRS_ROOT" \
  --output "$REFINEMENT_ROOT/ros_replay"

python -B tools/reconstruction_quality/november_refine.py \
  --audit "$NOVEMBER_ROOT" --pairs "$PAIRS_ROOT" \
  --output "$REFINEMENT_ROOT/cube_discovery" \
  --sessions 261113_145255 261113_150006 261113_150432

python -B tools/reconstruction_quality/november_refine.py \
  --audit "$NOVEMBER_ROOT" --pairs "$PAIRS_ROOT" \
  --output "$REFINEMENT_ROOT/all_scan_validation" \
  --sessions 261113_145255 261113_150006 261113_150432 261113_150709 261113_151101 261113_151435 261113_151637 261113_152024 261113_152345 \
  --profiles baseline bias_minus6 bias_minus7 support5x3 pose_edge20
```

Every retained study includes `provenance.json` with its exact command/source
hashes and `runner.py` matching the recorded runner hash. Those commands also
cover the additional depth/support sweep, alternate cube split, fine-grid sweep,
and full-scan review meshes. By default every fourth view (index 3 onwards) is
withheld. `--offset 1` changes the split for a robustness check. `--full` fuses all
views for visual review; its scores are training-view scores, not validation.
Pose refinement and cross-view support use training frames only. All candidates
are scored against the same raw rays and original poses/calibration, including
the fixed -5 mm scoring bias. The trials therefore do not calibrate absolute
depth bias. Experimental filters/pose changes are confined to this harness.

The retained output root is `reconstruction_quality/november_refinement` beside
the earlier audit. Once the six studies and ROS replay are complete, regenerate
the compact results and full-scan figures with:

```sh
python -B tools/reconstruction_quality/november_refinement_results.py \
  --root "$REFINEMENT_ROOT" --audit "$NOVEMBER_ROOT" --pairs "$PAIRS_ROOT"
```

`november_ros_replay.py` invokes actual service runner functions and production
processing, with ROS imports stubbed. It does not validate ROS transport, launch
startup, hardware, or Linux/ament integration. Nine full-frame replays matched
the earlier corrected meshes; a separate object-extraction replay also passed.

## Processing-only angle selection

Further work is scoped to processing the existing saved data. The corrected
baseline keeps its original 1.953125 mm voxel size and weight threshold 3.
`november_angles.py` tests binary selection by surface incidence angle without
altering calibration, poses, or surviving depth values. It estimates normals
from 5x5 camera-space PCA neighbourhoods; discontinuities and uncertain normals
are retained. Synthetic frontal/tilted planes and a depth step check the normal
estimation and preservation of input values before the campaign starts.

```sh
python -B tools/reconstruction_quality/november_angles.py \
  --audit "$NOVEMBER_ROOT" --pairs "$PAIRS_ROOT" \
  --output /path/to/new-angle-trial --angles 55 70
```

This runs the baseline and both angle cutoffs on all nine November scans with
the same held-out views/rays. The retained trial is
`reconstruction_quality/november_processing_angles`, with results in
`reports/reconstruction_quality/processing_angles.json`. Both cutoffs worsen
coverage and the within-5-mm rate; the method remains experimental. These tests
evaluate hard rejection, not continuous observation weighting during fusion.
