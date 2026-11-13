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
