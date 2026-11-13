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
