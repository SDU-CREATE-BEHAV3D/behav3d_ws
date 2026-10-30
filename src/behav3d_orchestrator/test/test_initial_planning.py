"""Bounded synthetic initial placement and actual native-kernel integration tests.

No ROS, models, robot commands or physical/material validation are involved.
"""
import hashlib
import re
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.supervised_agent import initial_planning as planning


def task(**changes):
    result = {"task_id": "wall-300-review-v1", "width_m": .3, "height_m": .3,
              "thickness_m": .03, "bead_spacing_m": .036, "bead_height_m": .016,
              "bead_diameter_m": .03, "count": 7, "yaw_degrees": [0., 90.],
              "search_step_m": .05, "max_pose_evaluations": 1000,
              "min_confidence": .6, "min_perception_score": .6, "clearance_m": .02,
              "footprint_margin_m": .01, "max_duplicate_layer_separation_m": .005,
              "max_support_triangle_height_span_m": .01, "base_z_offset_m": .005,
              "support_contact_tolerance_m": .005, "max_frontier_height_span_m": .016,
              "heat_seed_level_m": .005, "heat_t_coef": 2000.,
              "iso_level_m": 0., "normal_continuity_rule": True}
    result.update(changes)
    return result


def fixture(width=.6, depth=.6, z=.735):
    xs = np.arange(-width / 2 + .005, width / 2, .01)
    ys = np.arange(-depth / 2 + .005, depth / 2, .01)
    points = np.array([[x, y, z] for x in xs for y in ys])
    n = len(points)
    packet = {"schema": "agent007.initial_scene_packet.v1", "scene_id": "synthetic-scene",
              "frame_id": "synthetic_world", "coordinate_units": "m", "cell_size_m": .01,
              "primary_surface_id": "substrate",
              "surface_cell_indexing": {"origin_m": [0, 0], "horizontal_axes": [0, 1], "vertical_axis": 2},
              "registration_status": "unverified", "class_values": {"CLEAR_CANDIDATE": 1},
              "objects": [], "supports": [], "regions": [{"region_id": "region-0", "region_index": 0,
              "eligible": True, "surface_id": "substrate"}]}
    arrays = {"points": points, "surface_cells": np.floor(points[:, :2] / .01 + 1e-6).astype(np.int64),
              "review_class": np.ones(n, dtype=np.int32), "confidence": np.full(n, .9),
              "perception_score": np.full(n, .9), "clearance_m": np.full(n, np.inf),
              "suitability_region_index": np.zeros(n, dtype=np.int32),
              "object_points": np.empty((0, 3)), "object_owner_index": np.empty(0, dtype=np.int32),
              "object_ids": np.empty(0, dtype="U1")}
    return packet, arrays


def remove_rows(arrays, keep):
    for name in ("points", "surface_cells", "review_class", "confidence", "perception_score", "clearance_m", "suitability_region_index"):
        arrays[name] = arrays[name][keep]


def add_review_board(packet, arrays, *, width=.4, depth=.4, z=.755):
    """Explicit semantic membership plus independent measured owner geometry."""
    xs = np.arange(-width / 2 + .005, width / 2, .01)
    ys = np.arange(-depth / 2 + .005, depth / 2, .01)
    board = np.asarray([[x, y, z] for x in xs for y in ys])
    table = arrays["points"].copy()
    arrays.update(object_points=np.concatenate((table, board)), object_ids=np.asarray(["aa_table", "zz_board"]),
        object_owner_index=np.concatenate((np.zeros(len(table), dtype=np.int32), np.ones(len(board), dtype=np.int32))))
    packet["objects"] = [{"object_id": object_id, "label": label, "semantic_role": "support",
        "track_status": "confirmed", "geometry_eligible": True, "intent_confirmed": False}
        for object_id, label in (("aa_table", "work_table"), ("zz_board", "plywood_board"))]
    packet["supports"] = [{"surface_id": "substrate", "source_object_id": "aa_table", "print_candidate_allowed": True,
                           "owner": {"object_id": "aa_table", "semantic_role": "support"}}]
    packet["support_grid_provenance"] = {"augmentations": {"semantic_support_augmentation": {"objects": [
        {"object_id": "aa_table", "semantic_role": "support", "input_point_count": len(table)},
        {"object_id": "zz_board", "semantic_role": "support", "input_point_count": len(board)}]}}}
    return board


class InitialPlanningTests(unittest.TestCase):
    def setUp(self):
        self.packet, self.arrays = fixture()
        self.task = task()

    def propose(self):
        return planning.propose_wall(self.packet, self.arrays, self.task)

    def test_explicit_task_no_hidden_profile(self):
        for key in self.task:
            bad = dict(self.task)
            del bad[key]
            with self.subTest(key=key), self.assertRaises(ValueError):
                planning.validate_task(bad)
        with self.assertRaises(ValueError):
            planning.validate_task(dict(self.task, imaginary_default=True))

    def test_bounded_invalid_tasks(self):
        for change in ({"width_m": float("nan")}, {"count": True}, {"max_pose_evaluations": 0},
                       {"bead_diameter_m": .04}, {"yaw_degrees": [float("inf")]},
                       {"search_step_m": .000001}, {"normal_continuity_rule": 1}, {"max_pose_evaluations": 25001}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                planning.validate_task(task(**change))

    def test_deterministic_300mm_proposal_and_support_abstention(self):
        result = self.propose()
        self.assertEqual(result, self.propose())
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED")
        self.assertAlmostEqual(result["selected"]["center_m"][2], .730, places=6)
        corners = np.asarray(result["selected"]["wall_corners_m"])
        self.assertAlmostEqual(np.linalg.norm(corners[1] - corners[0]), .3)
        self.assertAlmostEqual(np.linalg.norm(corners[2] - corners[1]), .3)
        self.assertIsNone(result["selected"]["support_object_id"])
        self.assertEqual(result["support_assessment"]["decision"], "ABSTAIN")
        self.assertFalse(result["fabrication_authorized"])
        self.assertFalse(result["selected"]["checks"]["ik_checked"])
        self.assertTrue(result["requires_review"])

    def test_yaw_search_finds_only_fitting_orientation(self):
        self.packet, self.arrays = fixture(width=.16, depth=.5)
        result = self.propose()
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED")
        self.assertEqual(result["selected"]["yaw_deg"], 90.)
        self.assertTrue(all(pose["yaw_deg"] == 90 for pose in result["alternatives"]))

    def test_missing_internal_cells_not_bbox_area(self):
        self.packet, self.arrays = fixture(width=.34, depth=.08)
        keep = np.abs(self.arrays["points"][:, 0]) > .011
        remove_rows(self.arrays, keep)
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["rejected_reason_counts"].get("unobserved_footprint", 0), 0)

    def test_unknown_cells_veto_whole_footprint(self):
        self.packet, self.arrays = fixture(width=.34, depth=.08)
        self.arrays["review_class"][np.abs(self.arrays["points"][:, 0] - .05) < .011] = 6
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["rejected_reason_counts"].get("footprint_not_clear", 0), 0)

    def test_region_crossing_rejected(self):
        self.packet, self.arrays = fixture(width=.34, depth=.08)
        self.arrays["suitability_region_index"][self.arrays["points"][:, 0] > 0] = 1
        self.packet["regions"].append({"region_id": "region-1", "region_index": 1, "eligible": True, "surface_id": "substrate"})
        self.assertEqual(self.propose()["status"], "HOLD")

    def test_ineligible_regions_not_used(self):
        self.packet["regions"][0]["eligible"] = False
        self.assertEqual(self.propose()["status"], "HOLD")

    def test_low_confidence_or_score_not_used(self):
        for name in ("confidence", "perception_score"):
            self.packet, self.arrays = fixture()
            self.arrays[name][:] = .1
            with self.subTest(name=name):
                self.assertEqual(self.propose()["status"], "HOLD")

    def test_clearance_and_discontinuous_support_checks(self):
        self.arrays["clearance_m"][:] = .005
        self.assertEqual(self.propose()["status"], "HOLD")
        self.packet, self.arrays = fixture(width=.34, depth=.08)
        self.arrays["points"][:, 2] += (self.arrays["points"][:, 0] > 0) * .02
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["initialization_rejected_reason_counts"].get("native_position_no_full_hit", 0), 0)

    def test_full_wall_height_unknown_obstacles_not_only_base(self):
        points = self.arrays["points"].copy()
        points[:, 2] += .2
        self.arrays.update(object_points=points, object_ids=np.asarray(["unknown-object"]),
                           object_owner_index=np.zeros(len(points), dtype=np.int32))
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["rejected_reason_counts"].get("full_height_obstacle", 0), 0)

    def test_obstacle_above_wall_not_false_collision(self):
        points = self.arrays["points"].copy()
        points[:, 2] += .5
        self.arrays.update(object_points=points, object_ids=np.asarray(["overhead"]),
                           object_owner_index=np.zeros(len(points), dtype=np.int32))
        self.assertEqual(self.propose()["status"], "PROPOSED_REVIEW_REQUIRED")

    def test_same_cell_second_height_or_unknown_not_silently_deduplicated(self):
        for unknown in (False, True):
            self.packet, self.arrays = fixture(width=.34, depth=.08)
            indices = np.flatnonzero(np.abs(self.arrays["points"][:, 0] - .05) < .011)
            for name in ("points", "surface_cells", "review_class", "confidence", "perception_score", "clearance_m", "suitability_region_index"):
                extra = self.arrays[name][indices].copy()
                if name == "points" and not unknown:
                    extra[:, 2] += .05
                if name == "review_class" and unknown:
                    extra[:] = 6
                self.arrays[name] = np.concatenate((self.arrays[name], extra))
            result = self.propose()
            with self.subTest(unknown=unknown):
                self.assertEqual(result["status"], "HOLD")
                self.assertGreater(result["rejected_reason_counts"].get("ambiguous_surface_cell", 0), 0)

    def test_agreeing_duplicate_rows_are_bound_and_conservatively_usable(self):
        for name in ("points", "surface_cells", "review_class", "confidence", "perception_score", "clearance_m", "suitability_region_index"):
            self.arrays[name] = np.concatenate((self.arrays[name], self.arrays[name][:1]))
        self.assertEqual(self.propose()["status"], "PROPOSED_REVIEW_REQUIRED")

    def test_jittered_cell_representatives_preserve_coverage(self):
        self.arrays["points"][:, :2] += .002
        self.assertEqual(self.propose()["status"], "PROPOSED_REVIEW_REQUIRED")

    def test_frame_units_and_up_axis_are_never_aliased(self):
        for mutate in (lambda p: p.update(coordinate_units="mm"),
                       lambda p: p["surface_cell_indexing"].update(vertical_axis=1),
                       lambda p: p["surface_cell_indexing"].update(origin_m=[1, 1])):
            packet = deepcopy(self.packet)
            mutate(packet)
            with self.assertRaises(ValueError):
                planning.propose_wall(packet, self.arrays, self.task)

    def test_bound_inputs_and_exact_pose_cannot_change_after_proposal(self):
        proposal = self.propose()
        self.assertTrue(planning.validate_proposal(self.packet, self.arrays, self.task, proposal)["valid"])
        moved = deepcopy(proposal)
        moved["selected"]["center_m"][0] += .01
        with self.assertRaises(ValueError):
            planning.validate_proposal(self.packet, self.arrays, self.task, moved)
        self.arrays["confidence"][0] -= .01
        with self.assertRaises(ValueError):
            planning.validate_proposal(self.packet, self.arrays, self.task, proposal)

    def test_bounded_search_and_no_candidate_generation_on_proposal(self):
        self.task["max_pose_evaluations"] = 1
        result = self.propose()
        self.assertEqual(result["tested_pose_count"], 1)
        self.assertTrue(result["search_truncated"])
        self.assertNotIn("files", result)

    def test_optional_existing_support_assessed_but_not_automatically_used(self):
        self.packet["supports"] = [{"surface_id": "substrate", "source_object_id": "table", "intent_confirmed": False,
                                    "print_candidate_allowed": True,
                                    "owner": {"object_id": "table", "semantic_role": "support"}}]
        self.arrays.update(object_points=self.arrays["points"].copy(), object_ids=np.asarray(["table"]),
                           object_owner_index=np.zeros(len(self.arrays["points"]), dtype=np.int32))
        result = self.propose()
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED")
        self.assertEqual(result["support_assessment"]["existing_candidates"][0]["object_id"], "table")
        self.assertFalse(result["support_assessment"]["existing_candidates"][0]["selected"])
        self.assertIsNone(result["selected"]["support_object_id"])

    def test_explicit_semantic_surface_rejection_vetoes_clear_geometry(self):
        for rejection in ({"print_candidate_allowed": False}, {"semantic_class": "NON_PRINTABLE_SURFACE"},
                          {"owner": {"semantic_role": "obstacle"}}):
            self.packet["supports"] = [{"surface_id": "substrate", **rejection}]
            result = self.propose()
            with self.subTest(rejection=rejection):
                self.assertEqual(result["status"], "HOLD")
                self.assertGreater(result["rejected_reason_counts"].get("surface_explicitly_rejected", 0), 0)

    def test_invalid_center_pruning_does_not_spend_pose_budget(self):
        points = self.arrays["points"]
        self.arrays["review_class"][(np.abs(points[:, 0]) < .011) & (np.abs(points[:, 1]) < .011)] = 6
        self.task["max_pose_evaluations"] = 1
        result = self.propose()
        self.assertEqual(result["tested_pose_count"], 1)
        self.assertGreater(result["center_pruned_count"], 0)

    def test_rotated_footprint_cell_sliver_cannot_escape_dense_sampling(self):
        current_task = task(yaw_degrees=[37])
        center = np.array([.0007, .0013])
        u, v = planning._axes(37)
        samples = planning._sample_rectangle(center, u, v, .16, .025, .01)
        cells = np.floor(samples / .01 + 1e-6).astype(int)
        missing = (-8, -10)
        self.assertFalse(np.any(np.all(cells == missing, axis=1)))
        data, normalized = planning._inputs(self.packet, self.arrays, current_task)
        region = data["_regions"][0]
        valid, reason = planning._evaluate_pose(self.packet, data, normalized, region, center, 37)
        self.assertIsNotNone(valid)
        self.assertIsNone(reason)
        keep = ~np.all(self.arrays["surface_cells"] == missing, axis=1)
        remove_rows(self.arrays, keep)
        data, normalized = planning._inputs(self.packet, self.arrays, current_task)
        invalid, reason = planning._evaluate_pose(self.packet, data, normalized, region, center, 37)
        self.assertIsNone(invalid)
        self.assertEqual(reason, "unobserved_footprint")

    def test_linux_edge_candidate_cannot_pass_smaller_preselection_reserve(self):
        # Actual Linux failure: +1000 m rays moved a contour vertex to this X.
        # It passed a 1 um premask but failed the 10 um final rounding guard.
        contour = np.asarray([[.13499313354492187, 0., 0.], [.13498, 0., 0.],
                              [-.13499313354492187, 0., 0.]])
        mask = planning._candidate_width_mask(contour, np.asarray([1., 0.]), self.task)
        self.assertEqual(mask.tolist(), [False, True, False])
        self.assertGreaterEqual(planning.TARGET_ROUNDING_RESERVE_M, np.sqrt(2) * .000005)

    def test_large_world_translation_is_conditioned_without_moving_approved_pose(self):
        translation = np.asarray([123.45, -789.12, 12.345])
        self.arrays["points"] += translation
        self.arrays["surface_cells"] = np.floor(self.arrays["points"][:, :2] / .01 + 1e-6).astype(np.int64)
        proposal = self.propose()
        approved = deepcopy(proposal["selected"])
        with tempfile.TemporaryDirectory() as directory:
            result = planning.generate_first_batch(self.packet, self.arrays, self.task, proposal, Path(directory) / "batch")
            self.assertEqual(result["status"], "GENERATED_REVIEW_REQUIRED", result)
            self.assertEqual(result["selected"], approved)
            conditioning = result["planner"]["numeric_conditioning"]
            self.assertEqual(conditioning["world_origin_m"][:2], approved["center_m"][:2])
            self.assertEqual(conditioning["rotation"], "identity")
            self.assertLess(conditioning["ray_start_height_local_m"], 1.)
            self.assertLess(conditioning["z_scan_world_max_m"] - conditioning["z_scan_world_min_m"], 1e-6)
            self.assertAlmostEqual(conditioning["z_scan_world_max_m"], approved["center_m"][2] + .005, places=6)
            relative = np.asarray(result["candidate_points_m"]) - np.asarray(approved["center_m"])
            u, _ = planning._axes(approved["yaw_deg"])
            self.assertTrue(np.all(np.abs(relative[:, :2] @ u) + .015 + planning.TARGET_ROUNDING_RESERVE_M <= .15 + 1e-12))

    def test_actual_production_first_batch_preserves_pose_and_contract(self):
        self.task["yaw_degrees"] = [90.]
        proposal = self.propose()
        original_pose = deepcopy(proposal["selected"])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "first"
            result = planning.generate_first_batch(self.packet, self.arrays, self.task, proposal, output)
            self.assertEqual(result["status"], "GENERATED_REVIEW_REQUIRED")
            self.assertGreater(result["candidate_count"], 0)
            self.assertEqual(result["selected"], original_pose)
            self.assertEqual(proposal["selected"], original_pose)
            self.assertFalse(result["fabrication_authorized"])
            self.assertFalse(result["selected"]["checks"]["ik_checked"])
            self.assertEqual(result["planner"]["target_frame_id"], "synthetic_world")
            self.assertEqual(result["planner"]["target_origin_units"], "mm")
            self.assertEqual(result["planner"]["base_to_world_yaw_deg"], 0.)
            for name, record in result["files"].items():
                self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), record["sha256"])
            with np.load(output / "field_state_init.npz", allow_pickle=False) as field:
                self.assertTrue(np.allclose(field["field_vertices_world"], field["field_vertices_scaled"] + field["offset_xyz"]))
                heat = field["heat_norm"]
                z = field["field_vertices_scaled"][:, 2]
                native = planning._production()["compute_heat_field"].compute_heat_field(
                    field["field_vertices_scaled"], field["field_faces"], seed=None, seed_level=.005, t_coef=2000.)
                self.assertTrue(np.array_equal(heat, native.norm))
                self.assertTrue(np.array_equal(field["heat_dist"], native.dist))
                self.assertAlmostEqual(float(np.min(z)), 0.)
                self.assertAlmostEqual(float(np.max(z)), .3)
                self.assertGreater(len(np.unique(z)), 30)
                self.assertLess(np.ptp(field["z_scan_world"]), 1e-6)
                self.assertTrue(np.allclose(field["z_scan_world"], original_pose["center_m"][2] + .005, atol=1e-6))
                self.assertTrue(np.array_equal(field["numeric_origin_world"][:2], original_pose["center_m"][:2]))
                self.assertGreater(len(field["frontier_edges"]), 0)
                self.assertTrue(np.any(field["phi"] < 0))
                self.assertTrue(np.any(field["phi"] > 0))
            yaml = (output / "targets.yaml").read_text()
            self.assertNotIn("volume_mm3", yaml)
            self.assertNotIn("width", yaml)
            self.assertEqual(len(re.findall(r"  - index:", yaml)), result["candidate_count"])
            written = np.asarray([[float(value) for value in values] for values in re.findall(r'O\(([^,]+),([^,]+),([^\)]+)\)', yaml)])
            self.assertTrue(np.allclose(written / 1000, result["candidate_points_m"], atol=.0000051))
            self.assertTrue(np.all(written[:, 2] > 735))
            import open3d as o3d
            goal = o3d.io.read_triangle_mesh(str(output / "goal_mesh.ply"))
            vertices = np.asarray(goal.vertices)
            self.assertAlmostEqual(np.ptp(vertices[:, 2]), .3)
            self.assertTrue(np.allclose(vertices, np.asarray(proposal["initialization"]["goal_vertices_m"]), atol=1e-6))
            self.assertAlmostEqual(float(np.min(vertices[:, 2])), original_pose["center_m"][2], places=6)
            self.assertAlmostEqual(np.linalg.norm(vertices[-1, :2] - vertices[0, :2]), .3, places=5)
            with self.assertRaises(ValueError):
                planning.generate_first_batch(self.packet, self.arrays, self.task, proposal, output)

    def set_owner_cloud(self, height_offset, *, foreign=False):
        self.packet["supports"] = [{"surface_id": "substrate", "source_object_id": "table",
                                    "print_candidate_allowed": True,
                                    "owner": {"object_id": "table", "semantic_role": "support"}}]
        points = self.arrays["points"].copy()
        points[:, 2] += height_offset
        self.arrays.update(object_points=points, object_ids=np.asarray(["foreign_support" if foreign else "table"]),
                           object_owner_index=np.zeros(len(points), dtype=np.int32))

    def test_owning_table_noise_within_explicit_contact_band_is_not_collision(self):
        self.set_owner_cloud(.004)
        result = self.propose()
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED", result)
        self.assertGreater(result["selected"]["metrics"]["owning_substrate_contact_sample_count"], 0)
        self.assertEqual(result["selected"]["metrics"]["support_contact_tolerance_m"], .005)

    def test_owner_true_protrusion_above_band_remains_obstacle(self):
        self.set_owner_cloud(.008)
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["rejected_reason_counts"].get("full_height_obstacle", 0), 0)

    def test_explicit_printing_plane_owner_gets_only_bounded_contact(self):
        self.set_owner_cloud(.004)
        self.packet["supports"][0]["owner"]["semantic_role"] = "printing_plane"
        self.assertEqual(self.propose()["status"], "PROPOSED_REVIEW_REQUIRED")
        self.arrays["object_points"][:, 2] += .004
        self.assertEqual(self.propose()["status"], "HOLD")

    def test_native_shifted_upper_clearance_volume_is_rechecked(self):
        # Outside-centreline low support controls the provisional footprint base,
        # but the native centreline solve places the actual base ~8mm higher.
        # An overhead object in only that shifted upper clearance band must veto.
        self.task["yaw_degrees"] = [0.]
        self.arrays["points"][:, 2] -= (abs(self.arrays["points"][:, 1]) > .01) * .008
        self.arrays.update(object_points=np.asarray([[0., 0., 1.046]]), object_ids=np.asarray(["overhead"]),
                           object_owner_index=np.zeros(1, dtype=np.int32))
        data, normalized = planning._inputs(self.packet, self.arrays, self.task)
        site, reason = planning._evaluate_pose(self.packet, data, normalized, data["_regions"][0], np.asarray([0., 0.]), 0.)
        self.assertIsNone(reason)
        with self.assertRaisesRegex(ValueError, "native_full_height_obstacle"):
            planning._initialize_pose(self.packet, data, normalized, site)

    def test_same_primary_ground_contact_outside_clear_region_is_not_unknown(self):
        self.set_owner_cloud(.004)
        point = np.asarray([[.175, .005, .739]])
        self.arrays.update(object_points=point, object_owner_index=np.zeros(1, dtype=np.int32))
        outside = np.all(self.arrays["surface_cells"] == [17, 0], axis=1)
        self.arrays["suitability_region_index"][outside] = -1
        data, normalized = planning._inputs(self.packet, self.arrays, self.task)
        site, reason = planning._evaluate_pose(self.packet, data, normalized, data["_regions"][0], np.asarray([0., 0.]), 0.)
        self.assertIsNone(reason)
        self.assertEqual(site["metrics"]["owning_substrate_contact_sample_count"], 1)
        self.arrays["review_class"][outside] = 6
        data, normalized = planning._inputs(self.packet, self.arrays, self.task)
        site, reason = planning._evaluate_pose(self.packet, data, normalized, data["_regions"][0], np.asarray([0., 0.]), 0.)
        self.assertIsNone(site)
        self.assertEqual(reason, "full_height_obstacle")

    def test_high_owner_hump_cannot_self_authorize_from_its_own_high_cell(self):
        self.set_owner_cloud(.02)
        self.arrays.update(object_points=np.asarray([[.175, .005, .755]]), object_owner_index=np.zeros(1, dtype=np.int32))
        outside = np.all(self.arrays["surface_cells"] == [17, 0], axis=1)
        self.arrays["suitability_region_index"][outside] = -1
        self.arrays["points"][outside, 2] = .755
        data, normalized = planning._inputs(self.packet, self.arrays, self.task)
        site, reason = planning._evaluate_pose(self.packet, data, normalized, data["_regions"][0], np.asarray([0., 0.]), 0.)
        self.assertIsNone(site)
        self.assertEqual(reason, "full_height_obstacle")

    def test_foreign_support_and_unknowns_do_not_inherit_owner_contact_permission(self):
        self.set_owner_cloud(.001, foreign=True)
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["rejected_reason_counts"].get("full_height_obstacle", 0), 0)

    def test_tilted_support_uses_actual_frontier_not_global_5mm_flatness(self):
        self.packet, self.arrays = fixture(width=.34, depth=.08)
        self.arrays["points"][:, 2] += self.arrays["points"][:, 0] * .04
        self.task["yaw_degrees"] = [0.]
        result = self.propose()
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED", result)
        metrics = result["selected"]["initialization_metrics"]
        self.assertGreater(metrics["global_site_height_span_m"], .005)
        self.assertGreater(metrics["frontier_height_span_m"], .01)
        self.assertLess(metrics["frontier_height_span_m"], .016)
        self.assertAlmostEqual(metrics["minimum_base_overlap_m"], .005, places=6)
        self.assertTrue(result["selected"]["checks"]["native_goal_scan_intersection"])
        self.assertNotIn("files", result)

    def test_frontier_exceeding_explicit_one_bead_profile_holds(self):
        self.packet, self.arrays = fixture(width=.34, depth=.08)
        self.arrays["points"][:, 2] += self.arrays["points"][:, 0] * .08
        self.task["yaw_degrees"] = [0.]
        result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertGreater(result["initialization_rejected_reason_counts"].get("native_frontier_height_span", 0), 0)

    def test_overlap_changes_actual_goal_not_hidden_scalar_extension(self):
        self.task["base_z_offset_m"] = .007
        result = self.propose()
        vertices = np.asarray(result["initialization"]["goal_vertices_m"])
        self.assertAlmostEqual(float(vertices[:, 2].min()), .728, places=6)
        self.assertAlmostEqual(float(np.ptp(vertices[:, 2])), .3)
        self.assertEqual(result["initialization"]["metrics"]["field_guard_row_below_base_m"], 0.)
        self.assertAlmostEqual(result["initialization"]["metrics"]["minimum_base_overlap_m"], .007, places=6)

    def test_reviewed_native_evidence_cannot_be_resealed_to_change_geometry(self):
        proposal = self.propose()
        proposal["initialization"]["goal_vertices_m"][0][2] += .001
        proposal["proposal_id"] = "proposal-" + planning._digest({k: v for k, v in proposal.items() if k != "proposal_id"})
        with self.assertRaisesRegex(ValueError, "native goal initialization changed"):
            planning.validate_proposal(self.packet, self.arrays, self.task, proposal)

    def test_only_distant_unknown_object_preserves_empty_nearby_query(self):
        self.arrays.update(object_points=np.asarray([[2., 2., .8]]), object_ids=np.asarray(["unknown"]),
                           object_owner_index=np.zeros(1, dtype=np.int32))
        result = self.propose()
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED")
        self.assertEqual(result["selected"]["metrics"]["full_height_obstacle_sample_count"], 0)

    def test_goal_ply_round_trips_exact_world_geometry_not_six_digit_approximation(self):
        import open3d as o3d
        vertices = np.asarray([[.02689289238692094, -1.2323457951956047, -.1948091068],
                               [.32689289238692094, -1.2323457951956047, -.1948091068],
                               [.32689289238692094, -1.2323457951956047, .1051908932]])
        faces = np.asarray([[0, 1, 2]], dtype=np.int32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "exact.ply"
            planning._write_exact_goal_mesh(path, vertices, faces)
            mesh = o3d.io.read_triangle_mesh(str(path))
            self.assertTrue(np.array_equal(np.asarray(mesh.vertices), vertices))
            self.assertTrue(np.array_equal(np.asarray(mesh.triangles), faces))

    def test_centrality_precedes_quality_without_relaxing_hard_gates(self):
        points = self.arrays["points"]
        self.arrays["confidence"][:] = .95
        self.arrays["confidence"][(abs(points[:, 0]) < .16) & (abs(points[:, 1]) < .03)] = .7
        result = self.propose()
        self.assertEqual(result["placement_policy"]["ranking"], "orientation_then_semantic_review_then_centrality_then_evidence")
        self.assertLess(result["selected"]["metrics"]["placement_center_distance_m"], 1e-10)
        self.assertEqual(result["selected"]["metrics"]["minimum_confidence"], .7)
        self.assertEqual(len(result["placement_policy"]["reference_center_m"]), 2)

    def test_explicit_board_is_selected_centrally_with_its_own_geometry(self):
        add_review_board(self.packet, self.arrays)
        result = self.propose()
        self.assertEqual(result["status"], "PROPOSED_REVIEW_REQUIRED", result)
        selected = result["selected"]
        self.assertEqual(selected["substrate_object_id"], "zz_board")
        self.assertEqual(selected["substrate_label"], "plywood_board")
        self.assertEqual(selected["substrate_geometry_source"], "owner_only_observed_cell_top_envelope")
        self.assertTrue(selected["substrate_review_required"])
        self.assertIsNone(selected["support_object_id"])
        self.assertLess(selected["metrics"]["placement_center_distance_m"], 1e-10)
        support = np.asarray(result["initialization"]["support_vertices_m"])
        self.assertTrue(np.all(support[:, 2] == .755))
        self.assertAlmostEqual(selected["center_m"][2], .75, places=6)
        self.assertTrue(planning.validate_proposal(self.packet, self.arrays, self.task, result)["valid"])
        with tempfile.TemporaryDirectory() as directory:
            generated = planning.generate_first_batch(self.packet, self.arrays, self.task, result, Path(directory) / "board")
            self.assertEqual(generated["status"], "GENERATED_REVIEW_REQUIRED")
            self.assertEqual(generated["selected"]["substrate_object_id"], "zz_board")
            self.assertEqual(generated["geometry_sha256"], result["geometry_sha256"])

    def test_multiple_substrates_share_central_search_budget_fairly(self):
        add_review_board(self.packet, self.arrays)
        self.task.update(max_pose_evaluations=2, yaw_degrees=[0.])
        result = self.propose()
        self.assertEqual(result["tested_pose_count"], 2)
        self.assertEqual(result["selected"]["substrate_object_id"], "zz_board")
        self.assertEqual(result["substrate_search_diagnostics"]["aa_table"]["tested_pose_count"], 1)
        self.assertEqual(result["substrate_search_diagnostics"]["zz_board"]["tested_pose_count"], 1)

    def test_nonmember_unconfirmed_unknown_or_explicitly_rejected_support_cannot_become_substrate(self):
        for change in ("nonmember", "unconfirmed", "unknown", "rejected"):
            packet, arrays = fixture()
            add_review_board(packet, arrays)
            if change == "nonmember":
                packet["support_grid_provenance"]["augmentations"]["semantic_support_augmentation"]["objects"].pop()
            elif change == "unconfirmed":
                packet["objects"][1]["track_status"] = "tentative"
            elif change == "unknown":
                packet["objects"][1]["semantic_role"] = "unknown_object"
            else:
                packet["supports"].append({"surface_id": "board_surface", "source_object_id": "zz_board", "print_candidate_allowed": False})
            data, normalized = planning._inputs(packet, arrays, self.task)
            views = planning._substrate_views(packet, data, normalized)
            with self.subTest(change=change):
                self.assertNotIn("zz_board", [view["_view_metadata"]["substrate_object_id"] for view in views])

    def test_board_holes_and_nonclear_mixed_cells_cannot_use_table_as_filler(self):
        for change in ("hole", "mixed_class"):
            packet, arrays = fixture()
            add_review_board(packet, arrays, width=.34, depth=.08)
            if change == "hole":
                keep = (arrays["object_owner_index"] == 0) | (abs(arrays["object_points"][:, 0]) > .011)
                arrays["object_points"] = arrays["object_points"][keep]
                arrays["object_owner_index"] = arrays["object_owner_index"][keep]
                packet["support_grid_provenance"]["augmentations"]["semantic_support_augmentation"]["objects"][1]["input_point_count"] = int(np.count_nonzero(arrays["object_owner_index"] == 1))
            else:
                arrays["review_class"][abs(arrays["points"][:, 0]) < .011] = 6
            current_task = task(yaw_degrees=[0.])
            result = planning.propose_wall(packet, arrays, current_task)
            diagnostic = result["substrate_search_diagnostics"]["zz_board"]
            with self.subTest(change=change):
                self.assertEqual(diagnostic["native_initialized_pose_count"], 0)
                self.assertNotEqual((result.get("selected") or {}).get("substrate_object_id"), "zz_board")

    def test_substrate_eligibility_metadata_change_invalidates_exact_review(self):
        add_review_board(self.packet, self.arrays)
        result = self.propose()
        self.packet["objects"][1]["intent_confirmed"] = True
        with self.assertRaisesRegex(ValueError, "scene, geometry, frame or task changed"):
            planning.validate_proposal(self.packet, self.arrays, self.task, result)

    def test_owner_top_hump_is_not_discarded_to_make_board_fit(self):
        add_review_board(self.packet, self.arrays, width=.34, depth=.08)
        board = self.arrays["object_owner_index"] == 1
        stripe = board & (abs(self.arrays["object_points"][:, 0]) < .011)
        self.arrays["object_points"][stripe, 2] += .03
        self.task["yaw_degrees"] = [0.]
        result = self.propose()
        diagnostic = result["substrate_search_diagnostics"]["zz_board"]
        self.assertEqual(diagnostic["native_initialized_pose_count"], 0)
        self.assertNotEqual((result.get("selected") or {}).get("substrate_object_id"), "zz_board")

    def test_explicit_yaw_order_is_preserved_and_no_zero_is_injected(self):
        normalized = planning.validate_task(task(yaw_degrees=[90, 0, 270, -180]))
        self.assertEqual(normalized["yaw_degrees"], [90., 0.])
        self.task["yaw_degrees"] = [90., 0.]
        result = self.propose()
        self.assertEqual(result["selected"]["yaw_deg"], 90.)
        self.assertEqual(result["orientation_diagnostics"][1]["tested_pose_count"], 0)
        self.task["yaw_degrees"] = [45.]
        result = self.propose()
        self.assertEqual(result["selected"]["yaw_deg"], 45.)
        self.assertEqual(result["orientation_policy"]["yaw_order_degrees"], [45.])
        self.assertEqual(result["orientation_policy"]["wall_up_axis"], "+Z")
        self.assertEqual(result["orientation_policy"]["frame_id"], self.packet["frame_id"])

    def test_valid_unrotated_away_from_centre_beats_rotated_exact_centre(self):
        point = self.arrays["points"]
        self.arrays["review_class"][(abs(point[:, 0] - .125) < .006) & (abs(point[:, 1]) < .006)] = 6
        data, normalized = planning._inputs(self.packet, self.arrays, self.task)
        center_rotated, reason = planning._evaluate_pose(self.packet, data, normalized, data["_regions"][0], np.asarray([0., 0.]), 90.)
        self.assertIsNone(reason)
        self.assertIsNotNone(center_rotated)
        result = self.propose()
        self.assertEqual(result["selected"]["yaw_deg"], 0.)
        self.assertGreater(result["selected"]["metrics"]["placement_center_distance_m"], .02)
        self.assertEqual(result["orientation_diagnostics"][1]["status"], "NOT_RUN_PRIOR_STAGE_SELECTED")

    def test_rotation_requires_complete_failed_preferred_orientation(self):
        self.packet, self.arrays = fixture(width=.16, depth=.5)
        result = self.propose()
        self.assertEqual(result["selected"]["yaw_deg"], 90.)
        first = result["orientation_diagnostics"][0]
        self.assertEqual(first["status"], "NO_VALID_POSE_COMPLETE")
        self.assertTrue(first["full_pose_grid_exhausted"])
        self.assertTrue(first["full_native_candidates_exhausted"])
        self.assertEqual(first["native_valid_pose_count"], 0)
        self.assertTrue(planning.validate_proposal(self.packet, self.arrays, self.task, result)["valid"])
        result["orientation_diagnostics"][0]["full_native_candidates_exhausted"] = False
        result["proposal_id"] = "proposal-" + planning._digest({k: v for k, v in result.items() if k != "proposal_id"})
        with self.assertRaisesRegex(ValueError, "preferred orientation remains unresolved"):
            planning.validate_proposal(self.packet, self.arrays, self.task, result)

    def test_native_budget_uncertainty_cannot_trigger_rotation(self):
        with patch.object(planning, "MAX_NATIVE_INITIALIZATIONS", 1), patch.object(
                planning, "_initialize_pose", side_effect=ValueError("controlled_native_failure: test only")) as kernel:
            result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertEqual(result["orientation_search_status"], "UNRESOLVED_BUDGET")
        self.assertEqual(result["orientation_diagnostics"][0]["status"], "UNRESOLVED_NATIVE_BUDGET")
        self.assertEqual(result["orientation_diagnostics"][1]["status"], "NOT_RUN_PRIOR_STAGE_UNRESOLVED")
        self.assertEqual(kernel.call_count, 1)

    def test_pose_budget_uncertainty_cannot_trigger_rotation(self):
        self.task["max_pose_evaluations"] = 1
        with patch.object(planning, "_initialize_pose", side_effect=ValueError("controlled_native_failure: test only")):
            result = self.propose()
        self.assertEqual(result["status"], "HOLD")
        self.assertEqual(result["orientation_diagnostics"][0]["status"], "UNRESOLVED_POSE_BUDGET")
        self.assertEqual(result["orientation_diagnostics"][1]["tested_pose_count"], 0)

    def add_material_review(self):
        add_review_board(self.packet, self.arrays)
        self.packet["semantic_context"] = {"upstream_required_checks": ["material", "reachability", "collision"],
                                            "material_review_required": True}
        self.packet["objects"][1]["semantic_review"] = {
            "schema": "agent007.object_semantic_review.v1",
            "material": {"value": "wood", "confidence": .95, "source": "source_model_hypothesis"},
            "operational_treatment": {"value": "secondary_surface_review", "source": "resolved_policy"},
            "role_hypotheses": [{"role": "print_on_candidate", "confidence": .8, "evidence": ["visible flat board"]}],
            "perception_affordances": ["POTENTIALLY_PRINTABLE_ON", "REQUIRES_SURFACE_REVIEW"],
            "evidence_gaps": ["adhesion untested"], "operator_review_requested": True,
            "task_intent_confirmed": False, "material_compatibility_verified": False,
            "source_files": {"object/semantic.json": "a" * 64}, "llm_interpretation": {}}

    def test_structured_material_preference_propagates_review_not_physical_permission(self):
        self.add_material_review()
        result = self.propose()
        review = result["selected"]["semantic_placement_review"]
        self.assertEqual(review["recommendation_class"], "preferred_review_candidate")
        self.assertEqual(review["material_label"], "wood")
        self.assertEqual(review["material_confidence"], .95)
        self.assertTrue(review["operator_review_required"])
        self.assertFalse(review["task_intent_confirmed"])
        self.assertFalse(review["material_compatibility_verified"])
        self.assertFalse(result["fabrication_authorized"])
        for check in ("material", "reachability", "collision", "adhesion untested"):
            self.assertTrue(any(check in item for item in review["restrictions"]))
        self.assertEqual(result["selected"]["yaw_deg"], 0.)
        self.assertTrue(planning.validate_proposal(self.packet, self.arrays, self.task, result)["valid"])

    def test_known_label_alone_or_missing_material_does_not_invent_preferred_printability(self):
        self.add_material_review()
        review = self.packet["objects"][1]["semantic_review"]
        review["material"] = {}
        derived = planning._semantic_placement_review(self.packet, "zz_board")
        self.assertEqual(derived["recommendation_class"], "material_unknown_review")
        review["material"] = {"value": "wood"}
        review["role_hypotheses"] = []
        review["perception_affordances"] = []
        derived = planning._semantic_placement_review(self.packet, "zz_board")
        self.assertEqual(derived["recommendation_class"], "possible_support_review")

    def test_explicit_upstream_negative_policy_beats_positive_material(self):
        for key, value in (("requires_rescan", True), ("candidate_class", "AVOID_OBJECT")):
            self.add_material_review()
            self.packet["objects"][1][key] = value
            self.assertEqual(planning._semantic_placement_review(self.packet, "zz_board")["recommendation_class"], "blocked_upstream")
            data, normalized = planning._inputs(self.packet, self.arrays, self.task)
            self.assertNotIn("zz_board", [view["_view_metadata"]["substrate_object_id"] for view in planning._substrate_views(self.packet, data, normalized)])
        self.add_material_review()
        self.packet["objects"][1]["semantic_review"]["operational_treatment"]["value"] = "avoid_until_reviewed"
        self.assertEqual(planning._semantic_placement_review(self.packet, "zz_board")["recommendation_class"], "blocked_upstream")

    def test_raw_model_avoidance_and_display_fallback_do_not_override_resolved_policy(self):
        self.add_material_review()
        obj = self.packet["objects"][1]
        obj["operational_treatment"] = "avoid_until_reviewed"
        obj["semantic_review"]["llm_interpretation"] = {"recommended_role": "AVOID_OBJECT"}
        derived = planning._semantic_placement_review(self.packet, "zz_board")
        self.assertEqual(derived["recommendation_class"], "preferred_review_candidate")
        self.assertTrue(any("Earlier raw model avoidance" in item for item in derived["restrictions"]))
        obj["semantic_review"]["operational_treatment"] = {}
        self.assertNotEqual(planning._semantic_placement_review(self.packet, "zz_board")["recommendation_class"], "blocked_upstream")

    def test_blocked_primary_owner_cannot_reappear_through_legacy_fallback_cells(self):
        self.add_material_review()
        self.packet["objects"][0]["semantic_review"] = {"material": {},
            "operational_treatment": {"value": "avoid_until_reviewed", "source": "explicit_source_policy"}}
        data, normalized = planning._inputs(self.packet, self.arrays, self.task)
        views = planning._substrate_views(self.packet, data, normalized)
        self.assertNotIn("aa_table", [view["_view_metadata"]["substrate_object_id"] for view in views])
        result = self.propose()
        self.assertEqual(result["selected"]["substrate_object_id"], "zz_board")
        self.assertGreater(result["rejected_reason_counts"].get("substrate_semantically_rejected", 0), 0)
        self.assertEqual(result["semantic_placement_policy"]["substrate_reviews"]["aa_table"]["recommendation_class"], "blocked_upstream")

    def test_malformed_positive_hypothesis_confidence_does_not_invent_printability(self):
        self.add_material_review()
        review = self.packet["objects"][1]["semantic_review"]
        review["perception_affordances"] = []
        for confidence in (1.5, float("inf"), float("nan"), 0., -1., True):
            review["role_hypotheses"][0]["confidence"] = confidence
            with self.subTest(confidence=confidence):
                result = planning._semantic_placement_review(self.packet, "zz_board")
                self.assertEqual(result["recommendation_class"], "possible_support_review")


if __name__ == "__main__":
    unittest.main()
