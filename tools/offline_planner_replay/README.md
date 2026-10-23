# Fixed-goal offline planner replay

This experiment answers whether the previously proposed recovery placements can
be replaced by candidates generated through the existing scalar-field planner.
It uses recording `260728_133013`, all 23 archived candidate batches and the same
full scan-19 mesh with controlled 0°/4°/8°/12° inward lean. It imports numerical
`lib_scalar` functions directly; no ROS node, service, robot or extrusion client is
started. All outputs remain non-executable.

Open `outputs/offline_planner_replay/recorded_260728/viewer.html`. The report keeps
three different questions separate:

1. Can the saved field and contours reproduce the archived candidates?
2. Can today's numerical implementation reproduce those contours from the mesh?
3. Can a bounded alternative on the edited mesh satisfy the goal-generation rules
   and the previously declared geometric screens?

## Results

`mesh/fields/field_state_init1.npz` matches all **5,188 field vertices exactly, in
order, in every one of the 23 recorded exports**. Its heat scalar matches the
visible color encoding within 8-bit rounding. The other two saved field states
do not match. The repository width map has 5,721 values and is explicitly excluded.
Field topology comes from the NPZ; the point exports cannot independently verify it.

One inferred profile reproduces **23/23 batches, 157/157 ordered candidate positions**:
gradient lift 16 mm, graph spacing 36 mm, count 7, positive gradient direction and
a 30° orientation cone. Maximum ordered error is **0.000001322 mm**; all axes match
the two-decimal YAML within 0.005 per component after the documented XY sign change.
These parameters were fitted to archived outputs and then verified across all 23
batches. This is reproduction, not an independent prediction benchmark. It does
not prove those were the complete runtime settings or that bead height was 16 mm.

The historical raw-selection behavior matches the recording. Later/current
endpoint-spacing rules retain fewer candidates in many batches. Those rules are
reported separately; they must not be silently applied when claiming historical
reproduction. Source history inspected: `8fa8e6f` before the run and `6214cdb`
afterward. The current numerical functions are imported unchanged.

Full mesh-to-contour replay is close but not exact:

| Recorded scan | Symmetric nearest contour error | Viable-mask mismatches | Maximum ordered candidate error |
| --- | --- | --- | --- |
| 6 | 0.314 mm | 0 | 1.167 mm |
| 19 | 0.266 mm | 1 | 0.073 mm |

The cause is unresolved. Contour point order can also change: scan 19 has a large
ordered contour error despite its small symmetric nearest-neighbor error. The
report does not treat these two measures as interchangeable. Production ray
origins and contour code remain unchanged.

## Fixed goal against inward lean

The goal stays fixed while the same controlled deformation changes the scan.
Candidate endpoints and axes come from the existing gradient-lift and current
target-rule functions. Search enumerates unique eligible **contour vertices**;
it is not an exhaustive continuous search along edges or all possible repairs.

For a local alternative to C19:T7, preserve the previous recovery bounds relative
to the deformation-following seed: 8 mm XY, 12 mm Z and 4° axis change. Other six
archived endpoints reserve space; they are not issued or recertified. Evaluate
each alternative separately with the borrowed 36 × 16 mm bead proxy, 6/9 support
rays within ±3 mm, 2 mm sampled tool margin and both repository TCP transforms.
Additionally require 36 mm endpoint spacing, while retaining the earlier 32 mm
proxy-screen result for comparison.

| Inward lean | Alternatives within all local bounds | Passing alternatives | Orientation-only ablation |
| --- | --- | --- | --- |
| 0° control | 6 | 0 | No extra candidates |
| 4° | 5 | 0 | No extra candidates |
| 8° | 0 | 0 | 6 fit XY/Z but exceed 4°; all have 4/9 support |
| 12° | 0 | 0 | 5 fit XY/Z but exceed 4°; all have 4/9 support |

The ablation removes only the extra seed-relative 4° bound, retaining the planner's
30° cone. It still finds no passing candidate. At 8°, those axes differ from the
seed by approximately 5.6–5.8°. The prior surface-fitted `adapt-2-12` proposal still
has 8/9 support, but was not generated through this goal-field construction.
Finding the field does not retroactively validate it.

**The control is a material limitation:** the archived C19:T7 itself gets only
3/9 support under this nominal model on the unmodified scan. This experiment
therefore cannot establish that the real placement is unprintable, that the goal
is impossible to follow, or that lean causes all screen failures. It demonstrates
that the existing goal rule and the current local-recovery screen do not yet agree.
The control remains `use_normal_planner`, not a fabricated artifact-failure case.

The useful next development is to calibrate the nominal support/bead construction
against archived ordinary placements, then specify an explicit bounded goal-offset
strategy if continued growth from the observed surface is desired. Changing the
goal, reprioritizing a distant area, or contact correction are distinct strategies;
none is silently substituted for a passing local candidate here. Contact mechanics,
avoidance, calibrated tool transforms, IK, process response and future continuation
remain unresolved. Preserve the full-frontier inspection checkpoint.

## Reproduce and test

Numerical dependencies used: Python 3.11.14, NumPy 2.4.6, SciPy 1.17.1,
Open3D 0.19.0, potpourri3d 1.4.0 and PyYAML. No new ROS environment is needed.
The existing DDS environment supplied everything except potpourri3d, installed
only into `/private/tmp/behav3d_planner_dependencies` with `--no-deps --target`.
That temporary directory is not part of the repository. On another machine use a
numerical environment with these packages; omit `--dependency-path` if installed.

```sh
/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  tools/offline_planner_replay/build_replay.py \
  --recording /Users/josephnamar/Downloads/260728_133013 \
  --dependency-path /private/tmp/behav3d_planner_dependencies

PYTHONPATH=/private/tmp/behav3d_planner_dependencies \
  /Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  -m unittest discover -s tools/offline_planner_replay -p 'test_*.py' -v
```

`baseline_fixture.npz` copies the matched field and recorded contour/candidate
arrays for portable regression tests without the Downloads dataset. It contains
neither an invented avoidance map nor a fabricated runtime settings log. Lean
queries use complete committed mesh arrays from the earlier recovery experiments;
only the HTML display is downsampled. Source hashes are checked before and after
the build; results record dependency versions and the output manifest binds all
three generated artifacts. Earlier frozen recovery artifacts remain unchanged.
