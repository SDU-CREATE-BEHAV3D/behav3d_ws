"""Scientific two-agent startup integration, real exporter and native kernels.

Opt in with BEHAV3D_AGENT007_ROOT pointing to the actual Agent007 checkout and
run using the scientific Python environment. This is deliberately separate from
the lightweight mocked workflow tests. No ROS, model, camera or robot calls.
"""
import hashlib
import json
import os
import re
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.supervised_agent import initial_workflow as startup

from tools.startup_validation.fixtures import create_source_scene, task_example


def checksum(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


@unittest.skipUnless(os.environ.get("BEHAV3D_AGENT007_ROOT"),
                     "scientific joint test requires explicit BEHAV3D_AGENT007_ROOT")
class InitialJointScientificTests(unittest.TestCase):
    def setUp(self):
        self.agent = Path(os.environ["BEHAV3D_AGENT007_ROOT"]).resolve()
        if not (self.agent / "src/ocsr/initial_scene.py").is_file():
            self.fail("BEHAV3D_AGENT007_ROOT must be an actual Agent007 source checkout")
        sys.path[:0] = [str(self.agent / "src"), str(self.agent)]
        from ocsr import initial_scene
        self.handoff = initial_scene
        self.temporary = tempfile.TemporaryDirectory(prefix="behav3d-startup-joint-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.inputs = self.root / "inputs"
        self.inputs.mkdir()
        self.output = self.root / "outputs"
        self.output.mkdir()
        self.source = create_source_scene(self.inputs / "source_scene")
        self.task = task_example()
        self.task_path = self.inputs / "task.json"
        write_json(self.task_path, self.task)
        self.freeze_count = 0
        self.freeze()
        self.journal = self.root / "control/journal.jsonl"
        self.supervisor = startup.StartupSupervisor(self.journal)
        self.addCleanup(self.close_supervisor)
        self.calls = []

    def close_supervisor(self):
        self.supervisor.close()

    def freeze(self):
        self.freeze_count += 1
        self.frozen = self.inputs / ("frozen-" + str(self.freeze_count))
        self.handoff.export_scene(self.source, self.frozen, frame_id="startup_test_world")
        self.loaded = self.handoff.load_scene(self.frozen, frame_id="startup_test_world")
        self.request = {"phase": "PROPOSE", "request_id": "synthetic-initial-wall",
                        "artifact_id": "synthetic-control-wall-300", "frame_id": "startup_test_world",
                        "scene_manifest_path": str(self.frozen / "manifest.json"),
                        "scene_manifest_sha256": checksum(self.frozen / "manifest.json"),
                        "task_path": str(self.task_path), "task_sha256": checksum(self.task_path),
                        "proposal_path": "", "proposal_sha256": ""}

    def operation(self, request):
        self.calls.append(request["phase"])
        return startup.evaluate_initial_wall(request, input_roots=(self.inputs,), output_root=self.output,
            agent007_root=self.agent, behav3d_root=ROOT, approval_journal_path=self.journal)

    def propose(self):
        self.supervisor.set_armed(True, "synthetic-control-test-operator-not-human")
        result = self.supervisor.execute(self.request, lambda: self.operation(self.request))
        self.assertEqual(result["status"], "AWAITING_PLACEMENT_REVIEW", result)
        self.assertTrue(result["success"])
        self.assertEqual(checksum(result["proposal_path"]), result["proposal_sha256"])
        return result

    def generation_request(self, proposal):
        return dict(self.request, phase="GENERATE", proposal_path=proposal["proposal_path"],
                    proposal_sha256=proposal["proposal_sha256"])

    def assert_no_generated_products(self):
        self.assertEqual(list(self.output.rglob("targets.yaml")), [])
        self.assertEqual(list(self.output.rglob("field_state_init.npz")), [])
        self.assertEqual(list(self.output.rglob("goal_mesh.ply")), [])

    def test_full_real_exporter_supervisor_and_native_first_batch_loop(self):
        original = self.handoff._inventory(self.source)
        self.assertEqual(original, self.loaded["manifest"]["source_inventory"])
        self.assertEqual(self.loaded["packet"]["frame_id"], "startup_test_world")
        self.assertEqual(len(self.loaded["geometry"]["points"]), 3600)
        self.assertEqual(len(self.loaded["packet"]["objects"]), 2)
        self.assertFalse(self.loaded["packet"]["fabrication_authorized"])
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(self.request, lambda: self.fail("disarmed callback must not run"))
        proposal = self.propose()
        envelope = json.loads(Path(proposal["proposal_path"]).read_text())
        self.assertEqual(envelope["planning"]["status"], "PROPOSED_REVIEW_REQUIRED")
        selected = deepcopy(envelope["planning"]["selected"])
        self.assertIsNone(selected["support_object_id"])
        self.assert_no_generated_products()
        generate = self.generation_request(proposal)
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(generate, lambda: self.fail("unapproved callback must not run"))
        direct = self.operation(generate)
        self.assertEqual(direct["status"], "REJECTED")
        self.assert_no_generated_products()
        with self.assertRaises(startup.StartupError):
            self.supervisor.approve(self.request["request_id"], "f" * 64, "synthetic-test")
        self.supervisor.approve(self.request["request_id"], proposal["proposal_sha256"], "synthetic-test-not-human")

        def generate_operation():
            self.assertTrue(startup.generation_authorized(self.journal, generate))
            latest = json.loads(self.journal.read_text().splitlines()[-1])
            self.assertEqual(latest["event"], "START_GENERATE")
            self.assertIsNone(latest["state"]["records"][generate["request_id"]]["approval"])
            return self.operation(generate)

        result = self.supervisor.execute(generate, generate_operation)
        self.assertEqual(result["status"], "BATCH_PREVIEW_REVIEW_REQUIRED", result)
        self.assertTrue(result["success"])
        self.assertFalse(result["fabrication_authorized"])
        self.assertEqual(result["proposal_sha256"], proposal["proposal_sha256"])
        self.assertEqual(checksum(result["batch_manifest_path"]), result["batch_manifest_sha256"])
        batch = json.loads(Path(result["batch_manifest_path"]).read_text())
        self.assertEqual(batch["selected"], selected)
        self.assertGreater(batch["candidate_count"], 0)
        self.assertLessEqual(batch["candidate_count"], self.task["count"])
        self.assertEqual(batch["requested_count"], self.task["count"])
        self.assertEqual(batch["partial_batch"], batch["candidate_count"] < self.task["count"])
        self.assertFalse(batch["fabrication_authorized"])
        self.assertTrue(batch["requires_review"])
        for product in batch["files"].values():
            self.assertEqual(checksum(product["path"]), product["sha256"])
        field_path = batch["files"]["field_state_init.npz"]["path"]
        with np.load(field_path, allow_pickle=False) as field:
            self.assertTrue(np.array_equal(field["offset_xyz"], selected["center_m"]))
            self.assertTrue(np.allclose(field["field_vertices_world"], field["field_vertices_scaled"] + field["offset_xyz"]))
            self.assertGreater(len(np.unique(field["field_vertices_scaled"][:, 2])), 30)
        import open3d as o3d
        mesh = o3d.io.read_triangle_mesh(batch["files"]["goal_mesh.ply"]["path"])
        vertices = np.asarray(mesh.vertices)
        self.assertAlmostEqual(np.ptp(vertices[:, 2]), .3)
        self.assertAlmostEqual(np.linalg.norm(vertices[-1, :2] - vertices[0, :2]), .3)
        yaml = Path(batch["files"]["targets.yaml"]["path"]).read_text()
        self.assertEqual(len(re.findall(r"  - index:", yaml)), batch["candidate_count"])
        self.assertNotIn("volume_mm3", yaml)
        written = np.asarray([[float(value) for value in row] for row in re.findall(r'O\(([^,]+),([^,]+),([^\)]+)\)', yaml)])
        self.assertTrue(np.allclose(written / 1000, batch["candidate_points_m"], atol=.0000051))
        self.assertEqual(batch["planner"]["target_frame_id"], "startup_test_world")
        self.assertFalse(startup.generation_authorized(self.journal, generate))
        self.assertEqual(self.handoff._inventory(self.source), original)
        products_before = {str(path): checksum(path) for path in self.output.rglob("*") if path.is_file()}
        self.supervisor.close()
        self.supervisor = startup.StartupSupervisor(self.journal)
        self.assertFalse(self.supervisor.snapshot()["armed"])
        cached = self.supervisor.execute(generate, lambda: self.fail("cached generation must not rerun"))
        self.assertEqual(cached, result)
        cached_proposal = self.supervisor.execute(self.request, lambda: self.fail("cached proposal must not rerun"))
        self.assertEqual(cached_proposal, proposal)
        self.assertEqual(products_before, {str(path): checksum(path) for path in self.output.rglob("*") if path.is_file()})
        self.assertEqual(self.calls, ["PROPOSE", "GENERATE", "GENERATE"])

    def test_source_mutation_after_review_fails_closed_before_generation(self):
        proposal = self.propose()
        metadata = self.source / "object_0000/metadata.json"
        document = json.loads(metadata.read_text())
        document["synthetic_mutation"] = True
        write_json(metadata, document)
        self.assert_stale_generation(proposal)

    def add_explicit_secondary_board_source(self):
        """Extend only this synthetic source, then freeze through the real exporter.

        The primary decision grid deliberately spans the table and board, as in
        semantic support fusion. Owner-specific native geometry must instead be
        derived solely from the explicit board's retained points. The table is
        20 mm below the board, so no foreign-owner underlay exemption is needed.
        This is an offline software control, not measured wood/adhesion evidence.
        """
        from tools.startup_validation.fixtures import SYNTHETIC_CONTROL, _points
        board_id = "object_0002"
        cells = np.asarray([(x, y) for x in range(-20, 20) for y in range(-20, 20)], dtype=np.int32)
        points = np.column_stack(((cells + .5) * .01, np.full(len(cells), .755)))
        folder = self.source / board_id
        (folder / "perception").mkdir(parents=True)
        _points(folder / "points.ply", points)
        np.savez_compressed(folder / "cloud_attributes.npz", confidence=np.full(len(points), .9))
        write_json(folder / "metadata.json", {"object_id": board_id, "provisional_label": "plywood_board",
            "semantic_role": "support", "track_status": "confirmed", "point_count": len(points),
            "points_file": "points.ply", "cloud_attributes_file": "cloud_attributes.npz",
            "intent_confirmed": False, "synthetic_control": SYNTHETIC_CONTROL,
            "perception_files": {"semantic": "perception/semantic.json"}})
        write_json(folder / "perception/semantic.json", {"object_id": board_id,
            "semantic_role": "support", "scene_class": "SECONDARY_PRINT_SURFACE",
            "operational_geometry_eligible": True,
            "perception_affordances": ["POTENTIALLY_PRINTABLE_ON", "REQUIRES_SURFACE_REVIEW"],
            "ontology_v2": {"format": "ocsr.environment_ontology.v2", "object_id": board_id,
                "physical_identity": {"label": "plywood_board", "confidence": .95,
                    "source": "synthetic_visual_semantic_control", "evidence": ["controlled wood-grain identity evidence"]},
                "material": {"value": "wood", "confidence": .95, "source": "synthetic_visual_semantic_control",
                    "evidence": ["controlled visual wood hypothesis; not an adhesion measurement"]},
                "task_role": {"value": "secondary_substrate", "confidence": .95,
                    "source": "existing_semantic_role", "confirmed_intent": False, "candidate_role": None,
                    "hypotheses": [{"role": "secondary_substrate", "confidence": .85,
                                    "evidence": ["controlled support-role hypothesis"]},
                                   {"role": "print_on_candidate", "confidence": .8,
                                    "evidence": ["controlled clean flat board hypothesis"]}]},
                "operational_treatment": {"value": "secondary_surface_review", "source": "synthetic_conservative_policy"},
                "requires_operator_confirmation": False,
                "evidence_gaps": ["specific deposition boundary on board is not operator-confirmed"]},
            "synthetic_control": SYNTHETIC_CONTROL})
        interpretation = self.source / "interpretation"
        interpretation.mkdir()
        write_json(interpretation / "llm_interpretation.json", {"entities": [{"object_id": board_id,
            "recommended_role": "secondary_substrate", "operational_treatment": "secondary_surface_review",
            "ambiguities": ["specific deposition boundary on board is not operator-confirmed"],
            "requires_operator_confirmation": True}], "synthetic_control": SYNTHETIC_CONTROL})
        write_json(self.source / "environment_interpretation.json", {
            "cache_file": "interpretation/llm_interpretation.json",
            "environment_summary": {"description": "Controlled synthetic table and secondary plywood-board review fixture",
                                    "overall_uncertainties": ["task intent and physical contact unconfirmed"]},
            "support_interpretations": [], "synthetic_control": SYNTHETIC_CONTROL})
        context_path = self.source / "decision_context.json"
        context = json.loads(context_path.read_text())
        context.update(material_confirmed=False, material_review_required=True)
        write_json(context_path, context)
        write_json(self.source / "decision_recommendations.json", {
            "format": "ocsr.decision_recommendations.v1", "decision_status": "CONDITIONALLY_READY",
            "required_checks": ["confirm substrate material", "check robot reachability",
                                "check complete robot collision geometry"],
            "fabrication_authorized": False, "synthetic_control": SYNTHETIC_CONTROL})
        for name in ("scene.json", "semantic_scene.json"):
            path = self.source / name
            document = json.loads(path.read_text())
            document["objects"].append({"object_id": board_id, "metadata_file": board_id + "/metadata.json"})
            if "object_count" in document:
                document["object_count"] = len(document["objects"])
            write_json(path, document)
        topography_path = self.source / "support_topography/support_topography.json"
        topography = json.loads(topography_path.read_text())
        topography["semantic_support_augmentation"] = {
            "object_count": 1, "added_sample_count": len(points),
            "objects": [{"object_id": board_id, "semantic_role": "support",
                         "input_point_count": len(points), "added_sample_count": len(points)}],
            "purpose": "synthetic owner-provenance control; no physical or material certification"}
        write_json(topography_path, topography)
        # Simulate the existing merged support grid without adding ambiguous
        # duplicate rows. Its values cannot establish ownership of the board.
        for name in ("decision_context_grid.npz", "support_topography/surface_0001/topography_grid.npz"):
            path = self.source / name
            with np.load(path, allow_pickle=False) as source:
                arrays = {key: source[key] for key in source.files}
            inside = np.all(np.abs(arrays["points"][:, :2]) < .2, axis=1)
            arrays["points"][inside, 2] = .755
            np.savez_compressed(path, **arrays)
        self.freeze()
        return board_id

    def test_explicit_secondary_board_real_export_approval_and_native_batch_bindings(self):
        board_id = self.add_explicit_secondary_board_source()
        original_inventory = self.handoff._inventory(self.source)
        board = next(row for row in self.loaded["packet"]["objects"] if row["object_id"] == board_id)
        self.assertTrue(board["geometry_eligible"])
        self.assertFalse(board["intent_confirmed"])
        review = board["semantic_review"]
        self.assertEqual(review["material"]["value"], "wood")
        self.assertEqual(review["material"]["confidence"], .95)
        self.assertEqual(review["material"]["source"], "synthetic_visual_semantic_control")
        self.assertIn("not an adhesion measurement", review["material"]["evidence"][0])
        self.assertEqual(review["role_hypotheses"][1]["role"], "print_on_candidate")
        self.assertEqual(review["role_hypotheses"][1]["confidence"], .8)
        self.assertTrue(review["llm_interpretation"]["requires_operator_confirmation"])
        self.assertFalse(board["requires_intent_confirmation"])
        self.assertTrue(review["operator_review_requested"])
        self.assertFalse(review["task_intent_confirmed"])
        self.assertFalse(review["material_compatibility_verified"])
        context = self.loaded["packet"]["semantic_context"]
        self.assertTrue(context["material_review_required"])
        self.assertFalse(context["material_compatibility_verified"])
        self.assertEqual(context["upstream_decision_status"], "CONDITIONALLY_READY")
        self.assertEqual(context["upstream_required_checks"], ["confirm substrate material", "check robot reachability",
                                                            "check complete robot collision geometry"])
        for name, expected in {**review["source_files"], **context["source_files"]}.items():
            self.assertEqual(checksum(self.source / name), expected)
            self.assertEqual(self.loaded["manifest"]["source_inventory"][name], expected)
        proposal = self.propose()
        envelope = json.loads(Path(proposal["proposal_path"]).read_text())
        planning = envelope["planning"]
        selected = deepcopy(planning["selected"])
        self.assertEqual(selected["substrate_object_id"], board_id)
        self.assertEqual(selected["substrate_label"], "plywood_board")
        self.assertEqual(selected["substrate_geometry_source"], "owner_only_observed_cell_top_envelope")
        self.assertTrue(selected["substrate_review_required"])
        self.assertEqual(selected["yaw_deg"], 0.)
        self.assertEqual(planning["orientation_policy"]["yaw_order_degrees"][0], 0.)
        self.assertEqual(planning["orientation_diagnostics"][0]["status"], "SELECTED_REVIEW_REQUIRED")
        self.assertTrue(all(stage["tested_pose_count"] == 0 for stage in planning["orientation_diagnostics"][1:]))
        policy = planning["semantic_placement_policy"]
        self.assertEqual(policy["substrate_reviews"][board_id], selected["semantic_placement_review"])
        selected_review = selected["semantic_placement_review"]
        self.assertEqual(selected_review["recommendation_class"], "preferred_review_candidate")
        self.assertEqual(selected_review["material_label"], "wood")
        self.assertEqual(selected_review["material_confidence"], .95)
        self.assertEqual(selected_review["material_source"], "synthetic_visual_semantic_control")
        self.assertFalse(selected_review["material_compatibility_verified"])
        self.assertFalse(selected_review["task_intent_confirmed"])
        self.assertTrue(selected_review["operator_review_required"])
        self.assertTrue(any("print_on_candidate" in item for item in selected_review["positive_basis"]))
        restrictions = "\n".join(selected_review["restrictions"])
        for text in ("not physically verified", "not confirmed", "explicitly requests operator review",
                     "requires material review", *context["upstream_required_checks"],
                     "specific deposition boundary on board is not operator-confirmed"):
            self.assertIn(text, restrictions)
        self.assertEqual(selected_review["source_files"], review["source_files"])
        self.assertIsNone(selected["support_object_id"])
        self.assertFalse(selected["checks"]["material_adhesion_verified"])
        self.assertFalse(selected["checks"]["structural_capacity_verified"])
        self.assertLess(np.linalg.norm(selected["center_m"][:2]), .04)
        evidence = planning["initialization"]
        self.assertEqual(evidence["metrics"]["field_guard_row_below_base_m"], 0.)
        self.assertEqual(evidence["metrics"]["exact_pose_obstacle_sample_count"], 0)
        self.assertGreater(evidence["metrics"]["frontier_segment_count"], 0)
        self.assertTrue(np.allclose(np.asarray(evidence["support_vertices_m"])[:, 2], .755))
        self.assertTrue(np.isclose(selected["center_m"][2], .75, atol=1e-6))
        self.assert_no_generated_products()
        generate = self.generation_request(proposal)
        direct = self.operation(generate)
        self.assertEqual(direct["status"], "REJECTED")
        self.assert_no_generated_products()
        self.supervisor.approve(self.request["request_id"], proposal["proposal_sha256"],
                                "synthetic-board-software-test-reviewer-not-human")
        generated = self.supervisor.execute(generate, lambda: self.operation(generate))
        self.assertEqual(generated["status"], "BATCH_PREVIEW_REVIEW_REQUIRED", generated)
        self.assertFalse(generated["fabrication_authorized"])
        self.assertTrue(generated["requires_review"])
        self.assertEqual(checksum(generated["batch_manifest_path"]), generated["batch_manifest_sha256"])
        batch = json.loads(Path(generated["batch_manifest_path"]).read_text())
        self.assertEqual(batch["selected"], selected)
        self.assertEqual(batch["proposal_id"], planning["proposal_id"])
        for name in ("scene_id", "scene_sha256", "geometry_sha256", "task_sha256"):
            self.assertEqual(batch[name], planning[name])
        self.assertGreater(batch["candidate_count"], 0)
        self.assertEqual(batch["partial_batch"], batch["candidate_count"] < batch["requested_count"])
        for product in batch["files"].values():
            self.assertEqual(checksum(product["path"]), product["sha256"])
        with np.load(batch["files"]["field_state_init.npz"]["path"], allow_pickle=False) as field:
            self.assertTrue(np.array_equal(field["field_vertices_world"], evidence["goal_vertices_m"]))
            self.assertTrue(np.array_equal(field["field_faces"], evidence["goal_faces"]))
            self.assertTrue(np.array_equal(field["heat_norm"], evidence["heat_norm"]))
            self.assertTrue(np.array_equal(field["phi"], evidence["phi_m"]))
            self.assertTrue(np.array_equal(field["support_vertices_world_m"], evidence["support_vertices_m"]))
        import open3d as o3d
        goal = o3d.io.read_triangle_mesh(batch["files"]["goal_mesh.ply"]["path"])
        self.assertTrue(np.array_equal(np.asarray(goal.vertices), evidence["goal_vertices_m"]))
        self.assertTrue(np.array_equal(np.asarray(goal.triangles), evidence["goal_faces"]))
        targets = Path(batch["files"]["targets.yaml"]["path"]).read_text()
        written = np.asarray([[float(v) for v in row] for row in re.findall(r'O\(([^,]+),([^,]+),([^\)]+)\)', targets)])
        self.assertTrue(np.allclose(written / 1000, batch["candidate_points_m"], atol=.0000051, rtol=0))
        self.assertEqual(self.handoff._inventory(self.source), original_inventory)

    def test_secondary_board_owner_source_mutation_invalidates_exact_review(self):
        board_id = self.add_explicit_secondary_board_source()
        proposal = self.propose()
        selected = json.loads(Path(proposal["proposal_path"]).read_text())["planning"]["selected"]
        self.assertEqual(selected["substrate_object_id"], board_id)
        path = self.source / board_id / "points.ply"
        with path.open("a", encoding="ascii") as stream:
            stream.write("\n# software-test mutation of frozen owner evidence\n")
        self.assert_stale_generation(proposal)

    def test_secondary_board_semantic_source_mutation_invalidates_exact_review(self):
        board_id = self.add_explicit_secondary_board_source()
        proposal = self.propose()
        path = self.source / board_id / "perception/semantic.json"
        document = json.loads(path.read_text())
        document["ontology_v2"]["material"]["value"] = "changed_material_control"
        write_json(path, document)
        self.assert_stale_generation(proposal)

    def test_task_mutation_after_review_fails_closed_before_generation(self):
        proposal = self.propose()
        write_json(self.task_path, dict(self.task, width_m=.31))
        self.assert_stale_generation(proposal)

    def test_proposal_bytes_mutation_after_review_fails_closed(self):
        proposal = self.propose()
        path = Path(proposal["proposal_path"])
        envelope = json.loads(path.read_text())
        envelope["planning"]["selected"]["center_m"][0] += .01
        write_json(path, envelope)
        self.assert_stale_generation(proposal)

    def assert_stale_generation(self, proposal):
        self.supervisor.approve(self.request["request_id"], proposal["proposal_sha256"], "synthetic-test")
        generate = self.generation_request(proposal)
        result = self.supervisor.execute(generate, lambda: self.operation(generate))
        self.assertEqual(result["status"], "STALE_SOURCE", result)
        self.assertFalse(result["success"])
        self.assertFalse(self.supervisor.snapshot()["armed"])
        self.assert_no_generated_products()

    def test_frame_change_invalidates_gate_without_calling_backend(self):
        proposal = self.propose()
        self.supervisor.approve(self.request["request_id"], proposal["proposal_sha256"], "synthetic-test")
        generate = dict(self.generation_request(proposal), frame_id="ur20_base_link")
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(generate, lambda: self.fail("frame alias must not reach backend"))
        self.assertFalse(self.supervisor.snapshot()["armed"])
        self.assert_no_generated_products()

    def test_no_fit_preserves_source_bound_hold_without_generating(self):
        # .7 m can fit diagonally on a .6 m square; 1 m cannot at any supplied yaw.
        self.task["width_m"] = 1.0
        write_json(self.task_path, self.task)
        self.request["task_sha256"] = checksum(self.task_path)
        self.assert_hold()

    def test_occluded_grid_cannot_use_object_cloud_to_invent_support(self):
        grid = self.source / "decision_context_grid.npz"
        with np.load(grid, allow_pickle=False) as data:
            arrays = {key: data[key] for key in data.files}
        # Missing 1 cm strips every 8 cm in both axes: no 32 cm footprint fits
        # at any supplied yaw, although the retained object's table cloud exists.
        keep = np.all(arrays["surface_cells"] % 8 != 0, axis=1)
        np.savez_compressed(grid, **{key: value[keep] for key, value in arrays.items()})
        support = self.source / "support_topography/surface_0001/topography_grid.npz"
        with np.load(support, allow_pickle=False) as data:
            support_arrays = {key: data[key] for key in data.files}
        np.savez_compressed(support, **{key: value[keep] for key, value in support_arrays.items()})
        suitability = self.source / "surface_suitability.json"
        document = json.loads(suitability.read_text())
        document["regions"][0]["cell_count"] = int(np.count_nonzero(keep))
        write_json(suitability, document)
        self.freeze()
        self.assert_hold()

    def test_explicit_semantic_surface_veto_beats_clear_geometric_grid(self):
        path = self.source / "surface_ownership.json"
        document = json.loads(path.read_text())
        document["surfaces"][0]["print_candidate_allowed"] = False
        write_json(path, document)
        self.freeze()
        result = self.assert_hold()
        report = json.loads(Path(result["proposal_path"]).read_text())
        self.assertGreater(report["planning"]["rejected_reason_counts"].get("surface_explicitly_rejected", 0), 0)

    def assert_hold(self):
        self.supervisor.set_armed(True, "synthetic-control-test")
        result = self.supervisor.execute(self.request, lambda: self.operation(self.request))
        self.assertEqual(result["status"], "HOLD", result)
        self.assertFalse(result["success"])
        self.assertTrue(Path(result["proposal_path"]).is_file())
        self.assertEqual(checksum(result["proposal_path"]), result["proposal_sha256"])
        self.assert_no_generated_products()
        self.assertFalse(self.supervisor.snapshot()["armed"])
        with self.assertRaises(startup.StartupError):
            self.supervisor.approve(self.request["request_id"], result["proposal_sha256"], "synthetic-test")
        return result


if __name__ == "__main__":
    unittest.main()
