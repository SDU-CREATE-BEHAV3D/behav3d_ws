**November 4 and 13, 2026 — confidence and reconstruction audit**

Confidence is rejecting recoverable colored-cloud points because reconstruction appears to start with an incomplete set of aligned images. The mesh has an additional geometry problem: reducing its extraction threshold does little to repair the objects, while applying the saved distortion calibration helps substantially. Cube sides remain damaged after that correction.

This audit inspected 14 complete reconstruction scans containing 252 depth/color pairs: five scans on November 4 and nine on November 13. The folder also contains a 15-frame calibration capture and a config-only session, excluded from the reconstruction comparisons. All 252 reconstruction depth frames have distinct hashes, all 252 pose records are distinct, and every complete scan eventually has the expected depth, color and aligned-image count. A separate snapshot contains 883 selected files (630 MB). Original Drive files were only read. Final SHA-256 checks verified all 883 selected originals and their snapshot copies unchanged; all frozen production-source hashes also remain unchanged.

Source: [shared experiment folder](https://drive.google.com/drive/folders/1fohHZ1mIbK39buiSANj6SX4hXxn0WKRm). Full artifacts, input hashes and diagnostic images: [November audit folder](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence). Production code was frozen from local commit `de476296ecf4b04b74cb29d7ca7d4a798e40de55`, whose default reconstruction behavior follows J1. This follow-up changes analysis tools and reports only; it does not change production defaults, ROS services, or the original checkout.

**The strongest finding is incomplete color availability, not an excessively high minimum observation count.** I swept confidence bands, crops, frame strides, minimum observations, and prefixes of available aligned frames. I then replayed the inferred prefix through the actual production colorization and statistical outlier filter, using the unfiltered extracted mesh. All nine November 13 saved point clouds are reproduced to more than 99.99% of points within 0.1 mm in both directions; point counts differ by at most eight. Three November 4 clouds also match. Two older clouds do not have a close match and remain unresolved.

The original runtime logs/settings are not present, so the loaded-frame counts below are inferred, not read from logs. The near-exact point sets and the code path make early loading a strong causal explanation. Full-frame replay changes only aligned-image availability; it keeps `min_obs=3`, the 20 mm depth agreement band, and the same outlier filter.

| November 13 scan | Session | Inferred aligned frames used / available | Saved colored points | Complete replay points | Increase |
|---|---|---:|---:|---:|---:|
| Cube / linear | `261113_145255` | 9 / 20 | 122,033 | 236,811 | 94.1% |
| Cube / oriented | `261113_150006` | 12 / 20 | 198,048 | 217,986 | 10.1% |
| Cube / planar | `261113_150432` | 10 / 16 | 173,636 | 207,008 | 19.2% |
| Cone / linear | `261113_150709` | 11 / 20 | 130,663 | 232,229 | 77.7% |
| Cone / oriented | `261113_151101` | 12 / 20 | 193,866 | 213,239 | 10.0% |
| Cone / planar | `261113_151435` | 11 / 16 | 179,519 | 205,261 | 14.3% |
| Sphere / linear | `261113_151637` | 11 / 20 | 132,797 | 235,325 | 77.2% |
| Sphere / oriented | `261113_152024` | 10 / 20 | 185,185 | 217,502 | 17.5% |
| Sphere / planar | `261113_152345` | 11 / 16 | 181,711 | 208,009 | 14.5% |

Across these nine clouds, complete replay retains **1,973,370 rather than 1,497,458 points (+31.8%)**. These are whole-scene counts, including the table; they are not a claim of the same percentage increase on the object. The cube linear scan rises from 122,033 to 236,811 points with the confidence threshold unchanged.

![Color availability comparison](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/figures/november_color_availability.png)

The relevant code chain is:

- [The reconstruction service starts a background worker and returns before completion](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/src/behav3d_sense/behav3d_sense/reconstruct/reconstruct_services.py:164).
- [The orchestrator considers any one fresh aligned image sufficient](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/src/behav3d_orchestrator/behav3d_orchestrator/src/scan_session.py:792).
- [TSDF loads the currently available aligned files and only warns on a frame-count mismatch](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/src/behav3d_sense/behav3d_sense/reconstruct/TSDF_cpu_cropped.py:338).
- [Color confidence loops over the minimum of depth, pose and loaded-color counts](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/src/behav3d_sense/behav3d_sense/reconstruct/TSDF_cpu_cropped.py:526). Late color files are never incorporated into that in-memory list.

For the three older clouds with a close match:

| November 4 session | Inferred frames used / available | Saved points | Complete replay points |
|---|---:|---:|---:|
| `261104_145134` | 9 / 18 | 168,445 | 223,683 |
| `261104_150201` | 8 / 9 | 129,846 | 134,556 |
| `261104_150619` | 7 / 19 | 89,350 | 248,112 |

`261104_151209` and `261104_151736` were not reproduced by this prefix model. Do not assume their original filter settings or loaded inputs are known.

**Mesh confidence and point-cloud confidence are separate.** Open3D extraction uses weight threshold 3 before the colored cloud is computed. The later `min_obs=3`/color-mask/SOR operations filter the point cloud. The exported STL/PLY mesh is still the original extracted mesh, followed by optional world cropping and removal of components smaller than 50 triangles. Point-cloud confidence therefore cannot directly explain holes already present in `tsdf_surface_mesh.ply`.

I held each integrated volume fixed and extracted meshes at weights 3, 2, and 1, with and without rectification, for all 14 scans. Lower thresholds add substantial background surface but do not reconstruct the missing cone or sphere correctly. In the November 13 planar cone, the object-region triangle count remains 368 at all three thresholds. Rectification at the original threshold gives 1,003 triangles and a much more coherent cone. For six of nine rectified November 13 objects, lowering the threshold from 3 to 1 adds no object-region triangles at all.

With every aligned image loaded, the point-cloud confidence gate removes zero object-region vertices in five of nine legacy November 13 cases. The exceptions are cube linear (401/4,657), cube oriented (22/3,127), cube planar (4/3,179), and sphere linear (188/3,092). With rectification, seven of nine have zero loss; cube linear loses 96/3,131 and cube oriented 3/3,058. These counts are before statistical outlier removal and use the fixed region defined below.

![Controlled mesh comparison](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/figures/november_mesh_comparison.png)

**The correction improves views that were not used for fusion, but does not solve every defect.** Every fourth original view (positions 3, 7, 11, 15, 19 when present) was withheld from integration. Both profiles use full depth frames, 1.953125 mm voxels, 15.625 mm TSDF truncation, extraction weight 3, effective depth bias −5 mm, and depth maximum 0.9 m. Only the saved K/D camera model correction changes. Depth uses nearest-neighbor rectification and color uses linear sampling.

The table below compares meshes against the same held-out raw object-region rays under the saved distortion model. P90 is the 90th percentile absolute axial depth residual among mesh hits. The last two columns count rays within 5 mm over **all** valid reference rays, so a missed surface is penalized rather than hidden. Values measure consistency with saved sensor observations, not physical accuracy against an independent target.

| November 13 scan | Legacy P90 (mm) | Rectified P90 (mm) | Legacy within 5 mm | Rectified within 5 mm |
|---|---:|---:|---:|---:|
| Cube / linear | 49.13 | 36.81 | 57.8% | 71.6% |
| Cube / oriented | 38.40 | 36.02 | 70.4% | 70.2% |
| Cube / planar | 43.97 | 35.37 | 43.3% | 63.6% |
| Cone / linear | 45.41 | 15.26 | 41.1% | 84.7% |
| Cone / oriented | 17.71 | 17.64 | 82.9% | 82.9% |
| Cone / planar | 16.49 | 9.59 | 56.7% | 82.9% |
| Sphere / linear | 36.53 | 14.64 | 47.5% | 82.9% |
| Sphere / oriented | 10.01 | 8.94 | 82.9% | 83.4% |
| Sphere / planar | 33.49 | 21.44 | 53.1% | 74.2% |

P90 decreases in all nine cases, although the oriented cube changes little and its within-5-mm fraction decreases slightly. The large remaining cube residuals and visible side damage prevent calling this a finished high-quality reconstruction. Both pinhole and saved-distortion scoring are retained in the machine-readable evidence.

I also tested shorter truncation bands (8 mm and 4 mm) and 1 mm voxels with an 8 mm band on the oriented and planar cubes. These were exploratory tests, assessed on the same held-out views; they are not an additional independent validation set. None provided a consistent improvement over the conservative rectified profile. Shorter bands worsened P90 and often lost coverage; the finer grid also worsened P90. No resolution or truncation change is recommended from these tests.

| Cube scan / profile | Held-out P90 (mm) | Within 5 mm | Mesh-hit fraction |
|---|---:|---:|---:|
| Cube / oriented / Rectified / original grid | 36.02 | 70.2% | 100.0% |
| Cube / oriented / rectified_trunc8_w3 | 46.42 | 72.8% | 98.8% |
| Cube / oriented / rectified_trunc4_w3 | 54.01 | 64.1% | 93.3% |
| Cube / oriented / rectified_fine8_w3 | 45.08 | 73.6% | 99.1% |
| Cube / planar / Rectified / original grid | 35.37 | 63.6% | 98.3% |
| Cube / planar / rectified_trunc8_w3 | 38.42 | 61.8% | 90.3% |
| Cube / planar / rectified_trunc4_w3 | 40.97 | 59.6% | 83.9% |
| Cube / planar / rectified_fine8_w3 | 37.58 | 62.1% | 92.3% |

Raw-view table fits still disagree after rectification: the predicted table height at the object center spans 2.45–3.66 mm across November 13 views, with maximum normal differences of about 0.35–0.53°. This establishes residual inter-view inconsistency. It does not uniquely identify hand–eye calibration, depth bias, robot timing, or sensor noise as the cause. The oriented scans can have smaller table-height spread without rectification even though object residuals slightly favor rectification; one metric should not be treated as a universal accuracy score.

**The November 4 input contains scene changes.** In `261104_145134`, frames 0–8 show a wooden cube, and frames 9–17 show different objects. There is a 197-second gap between frames 8 and 9. Fusing that manifest treats incompatible scenes as one static object. In `261104_150619`, a hand is visibly placing an object at frame 2. These captures should be split or excluded from static-scene comparisons. They are retained unchanged in the audit.

[All frames from the mixed session](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/contact_sheets/261104_145134.png) · [Object placement during capture](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/contact_sheets/261104_150619.png). The November 13 RGB sequences have consistent objects. In linear barrel scans the object is outside the RGB view in the first five captures; this limits useful color support. The oriented paths keep it visible throughout. RGB visibility alone is not used to assert missing depth, because the two cameras have different fields of view.

The immediate engineering priority is to wait for **complete alignment**, with a run-specific completion marker written only after successful output writes, and to reject missing/misindexed color frames instead of silently using a shortened list. Re-running colorization after all files are ready recovers the colored-cloud points demonstrated above. This timing fix does not by itself repair STL geometry. For that, the evidence supports the existing opt-in rectification path first, followed by investigation of the remaining view disagreement and cube side coverage; lowering confidence globally is not supported.

For review, the nine November 13 meshes have been exported with full-frame rectification and the unchanged weight threshold 3. They preserve the complete scene and robot-base coordinates in **meters**; only the figures use a tighter viewing window.

[Review meshes in PLY and STL](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/review_meshes) · [All-image colored-cloud replay, cube linear](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/experiments/legacy/261113_145255/complete_rgb.ply) · [All-image colored-cloud replay, sphere linear](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/experiments/legacy/261113_151637/complete_rgb.ply).

The comparison region is fixed across candidates: base X in (−0.15, 0.15) m, Y in (−1.15, −0.65) m, and height 12–150 mm above the saved table plane. It is an object-region proxy, not a semantic ground-truth mask. Ray evaluation samples every second pixel and excludes immediate depth discontinuities using a 3×3 depth range greater than 20 mm. Both camera-model interpretations are saved. Confidence instrumentation was independently checked against production colorization with exact retained-point and observation-count equality for all 28 full-volume replays.

Baseline replays closely match the saved meshes but are not bit-identical across platforms. The integrity record includes vertex/triangle counts and bidirectional nearest-neighbor distributions; rare differences reach about one voxel. Prefix replay results likewise report the small residual differences rather than claiming exact geometry. No runtime capture configuration, sensor-confidence map, independent dimensions, original software build record, or ground-truth mesh was supplied. The original IR files and duplicate STL copies were not needed for this depth/mesh audit.

Reproducibility: [input inventory](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/inventory.json), [integrity and runtime](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/integrity.json), [prefix verification](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/prefix_replay.json), [full-volume object metrics](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/object_summary.json), [held-out metrics](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/heldout_summary.json), [additional cube tests](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/heldout_tuning_summary.json), and [per-view table fits](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_confidence/raw_table_consistency.json). Commands are documented in the repository analysis-tools README.


**16 November follow-up — ROS correction and further refinement**

The checked J1 workflow did not apply the saved lens-distortion coefficients.
GitHub was rechecked on 16 November: `J1_Build` still points to
`e0d25121105883fb6958c00a690329e839132c7b`. Its [color alignment](https://github.com/SDU-CREATE-BEHAV3D/behav3d_ws/blob/e0d25121105883fb6958c00a690329e839132c7b/src/behav3d_sense/behav3d_sense/reconstruct/color_to_depth.py#L65)
loads `Dd`/`Dc` but uses pinhole equations, and its
[cropped TSDF](https://github.com/SDU-CREATE-BEHAV3D/behav3d_ws/blob/e0d25121105883fb6958c00a690329e839132c7b/src/behav3d_sense/behav3d_sense/reconstruct/TSDF_cpu_cropped.py#L228)
loads `D` but supplies only `K` to fusion. The ROS wrappers call those paths;
capture saves raw image topics without undistortion. In the pinned
[Orbbec v2.4.5 driver](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2.4.5/orbbec_camera/src/ob_camera_node.cpp#L1720),
optional color undistortion defaults to false and publishes a separate
`color/image_undistorted` topic. This verifies the checked source path; no
historical deployed binary or complete runtime override log was supplied.

This branch now defaults all three ROS reconstruction services to
`rectify_images=true`, including object extraction. `reconstruct_services.launch.py`
passes the same setting to all nodes; `print_move.launch.py` exposes it as
`reconstruct_rectify_images`. Offline Python/CLI entry points retain explicit
opt-in behavior. No voxel size, extraction threshold, depth bias, or crop default
was changed by this ROS integration.

Alignment now writes images atomically and publishes a run-specific
`alignment.json` only when every image is present, readable, and indexed
correctly. The record binds image hashes to the manifest/calibration and records
start/completion times. The orchestrator waits for a fresh complete record;
TSDF rejects incomplete, mixed-camera-model, changed, or misindexed sets. A single
fresh PNG can no longer start the downstream reconstruction. Both packages must
be rebuilt together because old alignment workers do not publish this protocol.

Nineteen focused tests passed. All nine November 13 sessions were replayed
through the actual service runner functions with correction enabled by default.
All full frame sets passed the completion handoff; bidirectional mesh vertex
distances to the prior corrected replays were zero for every scan. The separate
object-extraction service runner also completed on the planar cube. These are
real image/geometry replays with ROS imports stubbed, not a live ROS transport or
hardware test. Linux/ament/launch startup remains untested on this Mac.

Further experiments performed 114 held-out reconstructions, followed by nine
full-scan experimental meshes for visual review. The profiles test 10/20 mm depth
jump rejection, 3/5 mm cross-view depth agreement with two/three total supporting
views, bounded pose refinement, several depth biases, and 1 mm voxels with an
8 mm truncation band. Pose fitting uses only training views, a fixed anchor,
object/table balancing, and a robot-pose prior; per-axis corrections are bounded
to 3 mm and 1 degree. These experimental changes remain in the analysis harness.

The primary comparison below uses all nine scan types with every fourth view
held out starting at index 3. P90 is the arithmetic mean of nine per-scan P90
values, not a pooled percentile. The within-5-mm and coverage percentages pool
12,085 valid reference rays, counting missing surfaces against the denominator.
Raw scoring rays, original poses, calibration and the scoring depth bias (-5 mm)
are fixed across candidates. These are sensor-consistency measures, not measured
physical accuracy or an independent calibration of the depth bias.

| Profile | Mean scan P90 (mm) | Within 5 mm | Mesh-hit coverage |
|---|---:|---:|---:|
| Corrected baseline | 21.75 | 74.30% | 99.72% |
| Depth bias -6 mm | 16.15 | 73.46% | 99.63% |
| Depth bias -7 mm | 12.33 | 71.49% | 99.61% |
| Three-view depth support | 21.12 | 74.06% | 99.59% |
| Pose + depth-edge rejection | 20.04 | 73.54% | 99.63% |
| 1 mm voxels / 8 mm band | 28.65 | 75.11% | 98.27% |
| 1 mm + pose / edge rejection | 26.54 | 74.93% | 98.50% |

No further candidate improves both reported agreement measures and coverage
consistently across the nine scans. A -6 mm bias reduces mean scan P90 by about
26%, while reducing the within-5-mm rate from 74.30% to 73.46%. At -7 mm the tail
improves further, but the within-5-mm rate falls to 71.49%. The stronger 10 mm edge
mask removes substantial planar-cube surface (held-out coverage 98.3% to 82.9%).
The 1 mm grid exposes finer structure and more noise, raises the mean P90, and
loses coverage. None is enabled as a new production default.

The alternate cube split (held-out indices 1, 5, 9, ...) is a robustness check,
not an independent unseen dataset. The 1 mm pose/edge combination raises the
within-5-mm rate from 69.1% to 75.7% on the linear cube and 69.3% to 74.2% on the
oriented cube, but worsens both tail errors; on the planar cube coverage falls
from 99.4% to 96.8%. Its artifact-inclusive box is 46.5 x 48.5 x 50.9 mm (linear),
46.6 x 47.6 x 50.2 mm (oriented), and 49.0 x 82.3 x 50.5 mm (planar). Those extents
use the same fixed 5.859375 mm table band/support plane as the earlier bounds.
The planar extent catches an unwanted protrusion; fitting an ideal cube would
hide that failure. Physical dimensions are still unknown.

The full-scan figures below show actual meshes, with matched view/scale and no
visual smoothing. Both columns use distortion correction and threshold 3.
The right-hand mesh additionally uses the experimental 1 mm grid, 8 mm band,
pose refinement, and 20 mm depth-edge rejection. These figures use all views
for fusion; their same-view scores are not used as validation evidence.

![All nine refinement comparisons](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/figures/all_nine_refinement.png)

![Held-out refinement measurements](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/figures/refinement_metrics.png)

[Full trials, meshes, provenance and integrity](/Users/josephnamar/.codex/visualizations/2026/11/13/01a1d591-ef53-71a1-b5d1-4895a2ba80d2/reconstruction_quality/november_refinement) ·
[Compact per-scan results](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/refinement_summary.json) ·
[J1 source verification](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/j1_workflow_verification.json).

All 381 original November 13 input files and their snapshot copies were verified
unchanged after processing. All 123 experimental mesh hashes and all six frozen
experiment-runner hashes were checked. The nine service-replay source/output
hashes were also verified. Production now includes calibrated rectification and
the complete-alignment handoff; further numerical refinements remain experimental.
Further work is scoped to processing the saved captures, keeping acquisition,
camera calibration and robot poses fixed. Physical accuracy remains unverified
without independent measurements.

## Processing-only follow-up, 2026-11-16

All nine November scans were reconstructed with the corrected baseline and
55-degree / 70-degree incidence-angle rejection: 27 reconstructions. The
baseline retains 1.953125 mm voxels and extraction weight 3. Reliable surface
normals are estimated from 5x5 camera-space PCA neighbourhoods; uncertain
normals and discontinuity neighbourhoods are retained. Surviving depth values,
calibration and poses are unchanged. This evaluates hard rejection, not soft
observation weighting.

| Profile | Mean scan P90 (mm) | Within 5 mm | Mesh-hit coverage |
|---|---:|---:|---:|
| Corrected baseline | 21.75 | 74.30% | 99.72% |
| Reject incidence above 55 degrees | 42.74 | 45.75% | 78.73% |
| Reject incidence above 70 degrees | 29.44 | 64.38% | 90.57% |

Both cutoffs reduce the within-5-mm rate on every scan and lose surface coverage.
Neither is adopted. The reference rays, held-out split and scoring convention
match the previous study; all nine baseline metric dictionaries are identical.
All 27 mesh hashes, the frozen runner hash and all 381 original/snapshot input
file hashes were verified. Results: `processing_angles.json`; reproducible
runner: `tools/reconstruction_quality/november_angles.py`.

## Summer school source audit, 2026-11-16

GitHub reports `summer_school_26` at
`8290731231fa46f850643962351641333248deb8` (2026-08-18). The local summer-school
branch remains at `2ba8bf293171156716cfd3f00cd112d947b4fb44` (2026-08-14,
"last day summerschool"). Both have the exact same `src/behav3d_sense` Git tree
as J1 `e0d25121105883fb6958c00a690329e839132c7b`:
`065de1bb7ccb08cea9bdedbbbaf9d626be39ce94`.

The launched reconstruction services therefore share J1's omission: color
alignment loads depth/color distortion coefficients but projects with only K;
TSDF loads D but integrates raw depth with a pinhole model. Bringup enables
those services; neither their launch nor service API applies rectification.

An older `behav3d_sense/reconstruction.py` does undistort color at line 165,
using hard-coded coefficients. It does not correct depth and is not registered
as a console entry point or referenced by the branch's launch files. Its
presence does not establish correction in the launched reconstruction path.
This audit verifies checked-in source, not historical deployed binaries or
runtime overrides. No summer-school branch files or references were changed.
Exact references and source links are recorded in `j1_workflow_verification.json`.

## Colored RViz mesh validation, 2026-11-16

The existing corrected pipeline now exports measured RGB vertex colors on
`tsdf_surface_mesh.ply`; the ROS cropped-TSDF service returns that mesh for RViz.
Unsupported vertices remain gray. Point-only confidence/outlier filters do not
remove mesh geometry. The world node publishes triangles with per-vertex colors
and retains every face by default; STL and RGB point-cloud exports remain.

Replayed oriented-cube scan `261113_150006` using its completed rectified image
alignment and the production TSDF service runner. All 230,947 vertices and
457,748 triangles match the earlier corrected reconstruction, allowing for
Open3D's output ordering; maximum vertex displacement is 0.0 mm. All 45 original
and snapshot input files for the scan were verified unchanged. Full-size marker
generation produced 1,373,244 positions and colors without geometry decimation.
24 focused checks passed, including RGB channel order/visibility, gray fallback,
mesh-versus-point routing, opacity, and the uniform-color display option.

ROS messages and transport were stubbed locally; live RViz rendering and robot
alignment still require the lab test. Colors improve appearance without changing
dimensional accuracy. Details are in `colored_mesh_validation.json`.

![Actual mesh with and without vertex colors (offline preview)](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/figures/colored_mesh_preview.png)

## Detached mesh fragments, 2026-11-16

The floating patches in the colored oriented-cube preview are two disconnected
components with 62 and 63 triangles, approximately 6.8 and 7.9 mm from the main
surface. They survived the previous 50-triangle mesh cutoff. Statistical outlier
filtering of the RGB point cloud did not remove these exported mesh triangles.

The production cutoff is now 100 triangles, adjustable through the cropped-TSDF
parameter `mesh_min_component_triangles` (50 restores the old cutoff; 0 retains
all components). Cleanup also now merges duplicate vertices before removing
duplicate/degenerate triangles, preventing invalid faces from surviving that
merge. There is no smoothing or movement of retained vertices. Confidence stays
at 3 and the grid at 1.953125 mm.

Validated all nine full-scan meshes and their nine held-out baseline meshes.
The full-scan outputs lose 64 small components containing 4,289 triangles, plus
eight degenerate faces from the cleanup-order fix. Every retained valid face and
its vertex colors match exactly, allowing for indexing/order changes. The main
connected surface's valid faces are retained in every case. All 18 source meshes
were checked unchanged. Thirty-two focused checks pass, including attached thin
details, separate valid objects, threshold boundaries, and ROS parameter routing.

Held-out mesh-hit coverage remains 99.7187%, unchanged on each scan. The pooled
within-5-mm rate changes from 74.2987% to 74.3815%; mean scan P90 changes from
21.7466 to 21.7714 mm. This is targeted artifact removal, not evidence of a broad
accuracy gain. Small genuine separate parts below the cutoff can also be removed;
artifacts connected to a larger surface remain. Live ROS/RViz is untested here.
Full results: `mesh_cleanup_validation.json`.

![Floating fragments before and after cleanup (offline preview)](/Users/josephnamar/.codex/worktrees/j1-reconstruction-quality/behav3d_ws/reports/reconstruction_quality/figures/mesh_cleanup_preview.png)
