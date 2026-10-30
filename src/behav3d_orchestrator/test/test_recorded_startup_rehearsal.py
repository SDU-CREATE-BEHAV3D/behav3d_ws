"""Recorded rehearsal control tests: real durable gate, fake numerical backend.

These prove sequencing and receipts only; scientific recorded/native integration
is exercised by the dedicated offline runner, not by the fake used here.
"""
import importlib.util
import io
import json
import sys
import tempfile
import types
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/behav3d_orchestrator"))
from behav3d_orchestrator.supervised_agent import initial_workflow as startup

SPEC = importlib.util.spec_from_file_location("recorded_startup_rehearsal", ROOT / "scripts/rehearse_initial_startup.py")
rehearsal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rehearsal)


class RecordedStartupRehearsalTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="behav3d-recorded-rehearsal-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.inputs, self.agent = self.root / "inputs", self.root / "agent"
        self.inputs.mkdir()
        (self.agent / "src/ocsr").mkdir(parents=True)
        (self.agent / "src/ocsr/initial_scene.py").write_text("# fake source, no geometry\n")
        self.manifest = self.inputs / "manifest.json"
        rehearsal.write_json(self.manifest, {"schema": "agent007.initial_scene.v1", "frame_id": "recorded_test_world"})
        self.task = self.inputs / "task.json"
        rehearsal.write_json(self.task, {"task_id": "fake-numerical-task"})
        self.run = self.root / "run"
        self.config = rehearsal.prepare_run(output=self.run, scene_manifest=self.manifest,
            task=self.task, frame_id="recorded_test_world", request_id="recorded-test-1",
            artifact_id="wall-review", input_roots=[self.inputs], agent007_root=self.agent,
            behav3d_root=ROOT)
        self.calls, self.generation_gates, self.emitted = [], [], []
        self.workflow = types.SimpleNamespace(StartupSupervisor=startup.StartupSupervisor,
            StartupError=startup.StartupError, evaluate_initial_wall=self.fake_backend)

    def fake_backend(self, request, **config):
        self.calls.append(request["phase"])
        if request["phase"] == "GENERATE":
            authorized = startup.generation_authorized(config["approval_journal_path"], request)
            self.generation_gates.append(authorized)
            if not authorized:
                return startup.error_result(request["request_id"], "REJECTED", "test gate rejection")
            batch = self.run / "products/batch"
            batch.mkdir()
            path = batch / "manifest.json"
            rehearsal.write_json(path, {"fake_numerical_backend": True})
            (batch / "targets.yaml").write_text("targets: []\n")
            return {"success": True, "status": "BATCH_PREVIEW_REVIEW_REQUIRED", "message": "fake numerical preview",
                "request_id": request["request_id"], "proposal_path": request["proposal_path"],
                "proposal_sha256": request["proposal_sha256"], "batch_manifest_path": str(path),
                "batch_manifest_sha256": rehearsal.checksum(path), "requires_review": True,
                "fabrication_authorized": False}
        folder = self.run / "products/proposal"
        folder.mkdir()
        path = folder / "proposal.json"
        rehearsal.write_json(path, {"fake_numerical_backend": True})
        return {"success": True, "status": "AWAITING_PLACEMENT_REVIEW", "message": "fake geometric proposal",
            "request_id": request["request_id"], "proposal_path": str(path), "proposal_sha256": rehearsal.checksum(path),
            "batch_manifest_path": "", "batch_manifest_sha256": "", "requires_review": True,
            "fabrication_authorized": False}

    def execute(self, **kwargs):
        return rehearsal.run_rehearsal(self.config, workflow=self.workflow,
            proposal_audit=lambda *args: {"passed": True, "scope": "fake numerical test only"},
            batch_audit=lambda *args: {"passed": True, "scope": "fake numerical test only"},
            emit=self.emitted.append, **kwargs)

    def events(self):
        return [json.loads(line) for line in (self.run / "events.jsonl").read_text().splitlines()]

    def test_prepare_has_no_approval_or_products_and_cannot_overwrite(self):
        self.assertTrue((self.run / "configuration.json").is_file())
        self.assertEqual(list((self.run / "products").iterdir()), [])
        self.assertFalse((self.run / "startup.jsonl").exists())
        self.assertEqual(rehearsal.load_run(self.run), self.config)
        with self.assertRaises(FileExistsError):
            rehearsal.prepare_run(output=self.run, scene_manifest=self.manifest, task=self.task,
                frame_id="recorded_test_world", request_id="other", artifact_id="wall-review",
                input_roots=[self.inputs], agent007_root=self.agent, behav3d_root=ROOT)

    def test_automatic_uses_real_gate_and_never_records_human_approval(self):
        summary = self.execute(software_test=True)
        self.assertTrue(summary["passed"], summary)
        self.assertEqual(summary["status"], "SOFTWARE_TEST_PASSED")
        self.assertTrue(summary["end_to_end_preview_validated"])
        self.assertFalse(summary["hold_handled_safely"])
        self.assertEqual(summary["final_state"], "DISARMED")
        self.assertFalse(summary["human_placement_approved"])
        self.assertFalse(summary["human_batch_preview_accepted"])
        self.assertFalse(summary["fabrication_authorized"])
        self.assertEqual(self.generation_gates, [False, True])
        labels = [row["event"] for row in self.events()]
        for name in ("DISARMED_REQUEST_BLOCKED", "UNAPPROVED_GENERATION_BLOCKED",
                     "DIRECT_BACKEND_BYPASS_BLOCKED", "WRONG_HASH_APPROVAL_BLOCKED", "SOFTWARE_TEST_APPROVAL"):
            self.assertIn(name, labels)
        self.assertNotIn("HUMAN_PLACEMENT_APPROVAL", labels)
        self.assertTrue(all(not row["robot_commands_sent"] for row in self.events()))
        self.assertEqual(rehearsal.read_json(self.run / "summary.json"), summary)

    def test_human_reject_stops_before_generation(self):
        summary = self.execute(decision=lambda *_: "reject")
        self.assertEqual(summary["status"], "HUMAN_REJECTED")
        self.assertFalse(summary["end_to_end_preview_validated"])
        self.assertTrue(summary["no_candidates_before_approval"])
        self.assertEqual(self.calls, ["PROPOSE"])
        self.assertEqual(rehearsal.generated_products(self.run / "products"), [])

    def test_human_placement_and_batch_review_are_separate(self):
        stages = []
        def decide(stage, checksum, events):
            stages.append((stage, checksum))
            return "approve" if stage == "placement" else "reject"
        summary = self.execute(decision=decide)
        self.assertEqual([stage for stage, _ in stages], ["placement", "batch"])
        self.assertTrue(summary["human_placement_approved"])
        self.assertFalse(summary["human_batch_preview_accepted"])
        self.assertEqual(self.calls, ["PROPOSE", "GENERATE"])
        self.assertFalse(summary["robot_commands_sent"])

    def test_proposal_hold_is_safe_but_not_end_to_end_success(self):
        self.workflow.evaluate_initial_wall = lambda request, **kwargs: startup.error_result(
            request["request_id"], "HOLD", "no validated geometric placement")
        summary = self.execute(software_test=True)
        self.assertEqual(summary["status"], "HOLD")
        self.assertFalse(summary["passed"])
        self.assertTrue(summary["hold_handled_safely"])
        self.assertFalse(summary["end_to_end_preview_validated"])
        self.assertFalse(summary["human_placement_approved"])
        self.assertEqual(rehearsal.generated_products(self.run / "products"), [])
        self.assertEqual(summary["final_state"], "DISARMED")

    def test_invalid_proposal_audit_fails_closed_before_approval(self):
        def fail(*args):
            raise ValueError("test independent geometry audit rejected")
        summary = rehearsal.run_rehearsal(self.config, software_test=True, workflow=self.workflow,
            proposal_audit=fail, emit=self.emitted.append)
        self.assertEqual(summary["status"], "FAILED")
        self.assertFalse(summary["passed"])
        self.assertFalse(summary["human_placement_approved"])
        self.assertEqual(self.calls, ["PROPOSE"])
        self.assertEqual(summary["final_state"], "DISARMED")

    def test_existing_run_cannot_restore_approval_or_replay_automatically(self):
        self.execute(decision=lambda *_: "abort")
        with self.assertRaisesRegex(ValueError, "already started"):
            rehearsal.load_run(self.run)
        resumed = startup.StartupSupervisor(self.run / "startup.jsonl")
        self.addCleanup(resumed.close)
        self.assertFalse(resumed.snapshot()["armed"])

    def test_stdin_eof_aborts_and_wrong_hash_never_approves(self):
        events = rehearsal.Events(self.root / "stdin-events.jsonl", mode="human_review", emit=lambda *_: None)
        self.addCleanup(events.close)
        expected = "a" * 64
        with patch.object(sys, "stdin", io.StringIO("approve " + "b" * 64 + "\n")):
            self.assertEqual(rehearsal.stdin_decision("placement", expected, events), "abort")
        with patch.object(sys, "stdin", io.StringIO("approve " + expected + "\n")):
            self.assertEqual(rehearsal.stdin_decision("placement", expected, events), "approve")
        with patch.object(sys, "stdin", io.StringIO("accept-preview " + expected + "\n")):
            self.assertEqual(rehearsal.stdin_decision("batch", expected, events), "approve")

    def test_cli_automatic_requires_explicit_software_test_flag(self):
        with self.assertRaises(SystemExit) as raised:
            rehearsal.main(["automatic", "--run", str(self.run)])
        self.assertEqual(raised.exception.code, 2)
        self.assertFalse((self.run / "events.jsonl").exists())


@unittest.skipUnless(importlib.util.find_spec("numpy"), "numerical initialization audit requires NumPy")
class NativeInitializationAuditTests(unittest.TestCase):
    def setUp(self):
        self.planning = {"task": {"width_m": .3, "height_m": .3, "iso_level_m": 0.},
            "selected": {"yaw_deg": 0., "center_m": [0., 0., 0.]},
            "initialization": {"schema": "behav3d.initial_goal_initialization.v1",
                "goal_vertices_m": [[-.15, 0., 0.], [.15, 0., 0.], [-.15, 0., .3], [.15, 0., .3]],
                "goal_faces": [[0, 1, 2], [1, 3, 2]], "phi_m": [-.005, -.005, .295, .295],
                "heat_norm": [0., 0., 1., 1.],
                "frontier_points_m": [[-.15, 0., .005], [.15, 0., .005]], "frontier_lines": [[0, 1]],
                "metrics": {"field_guard_row_below_base_m": 0., "ray_hit_count": 4, "goal_vertex_count": 4,
                    "buried_vertex_count": 2, "viable_vertex_count": 2, "frontier_segment_count": 1,
                    "base_z_offset_m": .005, "minimum_base_overlap_m": .005,
                    "maximum_base_overlap_m": .005, "heat_seed_level_m": .005, "heat_t_coef": 2000.}}}

    def test_actual_lowered_goal_heat_and_intersection_pass(self):
        audit = rehearsal.audit_initialization(self.planning)
        self.assertTrue(audit["passed"])
        self.assertEqual(audit["frontier_segment_count"], 1)
        self.assertEqual(audit["field_guard_row_below_base_m"], 0)

    def test_nonintersecting_mesh_or_hidden_guard_cannot_pass(self):
        for changes in ({"phi_m": [.001, .001, .3, .3]},
                        {"metrics": {**self.planning["initialization"]["metrics"], "field_guard_row_below_base_m": .016}},
                        {"frontier_lines": []}, {"heat_norm": [0., 0., 0., 0.]}):
            planning = deepcopy(self.planning)
            planning["initialization"].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                rehearsal.audit_initialization(planning)

    def test_different_goal_dimensions_or_base_cannot_pass(self):
        for width, height, base in ((.4, .3, 0.), (.3, .4, 0.), (.3, .3, .01)):
            planning = deepcopy(self.planning)
            planning["task"].update(width_m=width, height_m=height)
            planning["selected"]["center_m"][2] = base
            with self.subTest(width=width, height=height, base=base), self.assertRaises(ValueError):
                rehearsal.audit_initialization(planning)

    def test_incomplete_raycast_cannot_pass(self):
        self.planning["initialization"]["metrics"]["ray_hit_count"] = 3
        with self.assertRaisesRegex(ValueError, "missing support"):
            rehearsal.audit_initialization(self.planning)


if __name__ == "__main__":
    unittest.main()
