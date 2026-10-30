"""Startup gate/integrity tests; no ROS, numerical libraries or hardware."""

import hashlib
import json
import sys
import tempfile
import threading
import types
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.supervised_agent import initial_workflow as startup


def request(phase="PROPOSE", **changes):
    result = {
        "phase": phase,
        "request_id": "startup-a",
        "artifact_id": "wall-a",
        "frame_id": "recorded-world",
        "scene_manifest_path": "/inputs/manifest.json",
        "scene_manifest_sha256": "a" * 64,
        "task_path": "/inputs/task.json",
        "task_sha256": "b" * 64,
        "proposal_path": "",
        "proposal_sha256": "",
    }
    if phase == "GENERATE":
        result.update(proposal_path="/outputs/proposal.json", proposal_sha256="c" * 64)
    result.update(changes)
    return result


def success(value):
    return {
        "success": True,
        "status": startup.SUCCESS[value["phase"]],
        "message": "Review only",
        "request_id": value["request_id"],
        "proposal_path": value["proposal_path"] or "/outputs/proposal.json",
        "proposal_sha256": value["proposal_sha256"] or "c" * 64,
        "batch_manifest_path": "/outputs/batch/manifest.json"
        if value["phase"] == "GENERATE"
        else "",
        "batch_manifest_sha256": "d" * 64 if value["phase"] == "GENERATE" else "",
        "requires_review": True,
        "fabrication_authorized": False,
    }


class StartupSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.journal = Path(self.temporary.name).resolve() / "journal.jsonl"
        self.supervisor = startup.StartupSupervisor(self.journal)
        self.addCleanup(self.cleanup)
        self.calls = []

    def cleanup(self):
        self.supervisor.close()

    def proposal(self):
        self.supervisor.set_armed(True, "clearly-labelled-offline-test-operator")
        value = request()
        return self.supervisor.execute(
            value, lambda: self.calls.append("propose") or success(value)
        )

    def approve(self):
        proposal = self.proposal()
        self.supervisor.approve(
            "startup-a", proposal["proposal_sha256"], "offline-test-operator"
        )
        return proposal

    def test_disarmed_boot_denies_all_work(self):
        self.assertFalse(self.supervisor.snapshot()["armed"])
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(request(), lambda: self.fail("must not run"))
        self.assertFalse(
            startup.generation_authorized(self.journal, request("GENERATE"))
        )

    def test_proposal_waits_without_generating(self):
        result = self.proposal()
        self.assertEqual(result["status"], "AWAITING_PLACEMENT_REVIEW")
        self.assertEqual(
            self.supervisor.snapshot()["state"], "AWAITING_PLACEMENT_REVIEW"
        )
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(
                request("GENERATE"), lambda: self.fail("not approved")
            )
        self.assertEqual(self.calls, ["propose"])

    def test_wrong_hash_and_unknown_id_do_not_approve(self):
        self.proposal()
        for identifier, checksum in (("startup-a", "f" * 64), ("other", "c" * 64)):
            with self.assertRaises(startup.StartupError):
                self.supervisor.approve(identifier, checksum, "operator")
        self.assertEqual(
            self.supervisor.snapshot()["state"], "AWAITING_PLACEMENT_REVIEW"
        )

    def test_backend_gate_rejects_partial_trailing_journal_record(self):
        self.approve()
        proposal = self.supervisor.snapshot()["records"]["startup-a"]["proposal"]
        value = request(
            "GENERATE",
            proposal_path=proposal["proposal_path"],
            proposal_sha256=proposal["proposal_sha256"],
        )

        def operation():
            self.assertTrue(startup.generation_authorized(self.journal, value))
            with self.journal.open("ab") as stream:
                stream.write(b'{"interrupted_disarm":')
            self.assertFalse(startup.generation_authorized(self.journal, value))
            return success(value)

        self.supervisor.execute(value, operation)

    def test_exact_approval_consumed_before_generation_callback(self):
        self.approve()
        value = request("GENERATE")

        def operation():
            self.assertTrue(startup.generation_authorized(self.journal, value))
            row = json.loads(self.journal.read_text().splitlines()[-1])
            self.assertEqual(row["event"], "START_GENERATE")
            self.assertIsNone(row["state"]["records"]["startup-a"]["approval"])
            return success(value)

        result = self.supervisor.execute(value, operation)
        self.assertEqual(result["status"], "BATCH_PREVIEW_REVIEW_REQUIRED")
        self.assertFalse(startup.generation_authorized(self.journal, value))
        self.assertFalse(result["fabrication_authorized"])

    def test_denial_disarms_and_invalidates(self):
        self.proposal()
        self.supervisor.approve("startup-a", "c" * 64, "operator", approve=False)
        self.assertFalse(self.supervisor.snapshot()["armed"])
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(request("GENERATE"), lambda: self.fail("denied"))

    def test_changed_task_scene_or_frame_invalidates_approval(self):
        for field in ("task_sha256", "scene_manifest_sha256", "frame_id"):
            with self.subTest(field=field):
                if self.supervisor.snapshot()["records"]:
                    self.supervisor.close()
                    self.journal = self.journal.parent / (field + ".jsonl")
                    self.supervisor = startup.StartupSupervisor(self.journal)
                self.approve()
                value = request(
                    "GENERATE",
                    **{field: "f" * 64 if field.endswith("sha256") else "wrong-frame"},
                )
                with self.assertRaises(startup.StartupError):
                    self.supervisor.execute(value, lambda: self.fail("changed"))
                self.assertFalse(self.supervisor.snapshot()["armed"])

    def test_restart_never_restores_approved_authority(self):
        self.approve()
        self.supervisor.close()
        self.supervisor = startup.StartupSupervisor(self.journal)
        state = self.supervisor.snapshot()
        self.assertFalse(state["armed"])
        self.assertEqual(state["state"], "HOLD")
        self.assertIsNone(state["records"]["startup-a"]["approval"])
        with self.assertRaises(startup.StartupError):
            self.supervisor.set_armed(True, "operator")
        self.supervisor.set_armed(False, "explicit-reconciliation")
        self.supervisor.set_armed(True, "operator")
        with self.assertRaises(startup.StartupError):
            self.supervisor.execute(request("GENERATE"), lambda: self.fail("restart"))

    def test_completed_receipts_reuse_without_recomputing_even_after_restart(self):
        self.approve()
        value = request("GENERATE")
        result = self.supervisor.execute(value, lambda: success(value))
        self.supervisor.close()
        self.supervisor = startup.StartupSupervisor(self.journal)
        self.assertEqual(
            self.supervisor.execute(value, lambda: self.fail("cached")), result
        )
        self.assertEqual(
            self.supervisor.execute(request(), lambda: self.fail("cached proposal"))[
                "status"
            ],
            "AWAITING_PLACEMENT_REVIEW",
        )

    def test_cancel_running_discards_late_generation(self):
        self.approve()
        value = request("GENERATE")

        def operation():
            self.supervisor.invalidate("cancelled")
            self.assertFalse(startup.generation_authorized(self.journal, value))
            return success(value)

        result = self.supervisor.execute(value, operation)
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], "HOLD")
        self.assertFalse(self.supervisor.snapshot()["armed"])
        self.assertIsNone(self.supervisor.snapshot()["records"]["startup-a"]["batch"])

    def test_invalid_success_cannot_authorize_or_advance(self):
        self.supervisor.set_armed(True, "test")
        result = success(request())
        result["fabrication_authorized"] = True
        returned = self.supervisor.execute(request(), lambda: result)
        self.assertFalse(returned["success"])
        self.assertEqual(self.supervisor.snapshot()["state"], "HOLD")

    def test_failure_reason_retained_in_journal_without_approval(self):
        self.supervisor.set_armed(True, "software-test")
        failure = startup.error_result(
            "startup-a", "HOLD", "Native planner HOLD: measured support incomplete"
        )
        self.assertEqual(self.supervisor.execute(request(), lambda: failure), failure)
        record = self.supervisor.snapshot()["records"]["startup-a"]
        self.assertEqual(record["last_result"], failure)
        self.assertIsNone(record["approval"])
        self.assertIsNone(record["proposal"])
        self.supervisor.close()
        self.supervisor = startup.StartupSupervisor(self.journal)
        self.assertEqual(
            self.supervisor.snapshot()["records"]["startup-a"]["last_result"], failure
        )

    def test_single_owner_and_tampered_or_truncated_journal_fail_closed(self):
        with self.assertRaises((BlockingIOError, OSError)):
            startup.StartupSupervisor(self.journal)
        self.supervisor.close()
        raw = self.journal.read_bytes()
        self.journal.write_bytes(raw[:-1])
        with self.assertRaises(startup.StartupError):
            startup.StartupSupervisor(self.journal)
        self.journal.write_bytes(raw.replace(b'"BOOT"', b'"FAIL"'))
        with self.assertRaises(startup.StartupError):
            startup.StartupSupervisor(self.journal)

    def test_fsync_failure_prevents_callback(self):
        self.supervisor.set_armed(True, "test")
        with (
            patch.object(startup.os, "fsync", side_effect=OSError),
            self.assertRaises(startup.StartupError),
        ):
            self.supervisor.execute(request(), lambda: self.fail("not durable"))

    def test_unknown_commands_extra_fields_and_nonabsolute_paths_rejected(self):
        for changed in (
            {"phase": "PRINT"},
            {"robot_commands": []},
            {"task_path": "relative"},
            {"proposal_path": "/unapproved.json"},
            {"task_sha256": "A" * 64},
        ):
            with (
                self.subTest(changed=changed),
                self.assertRaises((ValueError, TypeError)),
            ):
                startup.validate_request(dict(request(), **changed))


class InitialWorkerBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.scene, self.original, self.tasks, self.output, self.agent, self.behav = [
            self.root / name
            for name in ("scene", "original", "tasks", "output", "agent", "behav")
        ]
        for path in (
            self.scene,
            self.original,
            self.tasks,
            self.output,
            self.agent,
            self.behav,
        ):
            path.mkdir()
        scalar = self.behav / "src/behav3d_py/behav3d_py/scalar_field/lib_scalar"
        scalar.mkdir(parents=True)
        (scalar / "__init__.py").write_text("# frozen test kernels\n")
        self.sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
        (self.original / "input.json").write_text("{}")
        (self.scene / "sources").mkdir()
        (self.scene / "sources/input.json").write_text("{}")
        (self.scene / "scene_packet.json").write_text("{}")
        (self.scene / "geometry.npz").write_bytes(b"synthetic test geometry")
        manifest = {
            "schema": "agent007.initial_scene.v1",
            "frame_id": "recorded-world",
            "coordinate_units": "m",
            "registration_status": "unverified",
            "source_root": str(self.original),
            "source_inventory": {"input.json": self.sha(self.original / "input.json")},
            "files": {
                name: self.sha(self.scene / name)
                for name in ("scene_packet.json", "geometry.npz")
            },
        }
        (self.scene / "manifest.json").write_text(json.dumps(manifest))
        (self.tasks / "task.json").write_text("{}")
        modulefile = self.agent / "src/ocsr/initial_scene.py"
        modulefile.parent.mkdir(parents=True)
        modulefile.write_text("# configured test adapter")
        for name in (
            "perception_scene.py",
            "scene_validation.py",
            "support_topography.py",
        ):
            (modulefile.parent / name).write_text("# configured scene producer")
        hashes = {
            name: self.sha(modulefile.parent / name)
            for name in (
                "initial_scene.py",
                "perception_scene.py",
                "scene_validation.py",
                "support_topography.py",
            )
        }
        manifest["producer"] = {"sha256": startup.digest(hashes), "files": hashes}
        (self.scene / "manifest.json").write_text(json.dumps(manifest))
        scene_module = types.ModuleType("ocsr.initial_scene")
        scene_module.__file__ = str(modulefile)
        scene_module.load_scene = lambda *args, **kwargs: {
            "manifest": manifest,
            "packet": {},
            "geometry": {},
        }
        package = types.ModuleType("ocsr")
        package.__path__ = []
        self.planner = types.ModuleType(
            "behav3d_orchestrator.supervised_agent.initial_planning"
        )
        self.planner.propose_wall = lambda *_: {
            "status": "PROPOSED_REVIEW_REQUIRED",
            "requires_review": True,
            "fabrication_authorized": False,
            "proposal_id": "proposal-test",
            "selected": {"center_m": [0, 0, 0]},
        }
        self.planner.validate_proposal = lambda *_: {
            "status": "VALIDATED_REVIEW_REQUIRED",
            "valid": True,
        }
        self.generated = []

        def generate(packet, geometry, task, proposal, output):
            self.generated.append(output)
            output.mkdir()
            (output / "targets.yaml").write_text("targets: []")
            (output / "field_state_init.npz").write_bytes(b"synthetic approved field")
            (output / "goal_mesh.ply").write_bytes(b"synthetic approved goal")
            document = {
                "schema": "behav3d.initial_wall_batch.v1",
                "status": "GENERATED_REVIEW_REQUIRED",
                "frame_id": "recorded-world",
                "coordinate_units": "m",
                "requires_review": True,
                "fabrication_authorized": False,
                "proposal_id": proposal["proposal_id"],
                "selected": proposal["selected"],
                "files": {
                    name: {
                        "path": str(output / name),
                        "sha256": self.sha(output / name),
                    }
                    for name in (
                        "targets.yaml",
                        "field_state_init.npz",
                        "goal_mesh.ply",
                    )
                },
            }
            (output / "manifest.json").write_text(json.dumps(document))
            return {
                "status": "GENERATED_REVIEW_REQUIRED",
                "requires_review": True,
                "fabrication_authorized": False,
            }

        self.planner.generate_first_batch = generate
        scalar_module = types.ModuleType("lib_scalar")
        scalar_module.__file__ = str(scalar / "__init__.py")
        self.patch = patch.dict(
            sys.modules,
            {
                "ocsr": package,
                "ocsr.initial_scene": scene_module,
                "lib_scalar": scalar_module,
                "behav3d_orchestrator.supervised_agent.initial_planning": self.planner,
            },
        )
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.value = request(
            scene_manifest_path=str(self.scene / "manifest.json"),
            scene_manifest_sha256=self.sha(self.scene / "manifest.json"),
            task_path=str(self.tasks / "task.json"),
            task_sha256=self.sha(self.tasks / "task.json"),
        )
        self.policy = {
            "input_roots": [self.scene, self.original, self.tasks],
            "output_root": self.output,
            "agent007_root": self.agent,
            "behav3d_root": self.behav,
        }

    def run_phase(self, value=None, **policy):
        return startup.evaluate_initial_wall(
            self.value if value is None else value, **dict(self.policy, **policy)
        )

    def test_propose_writes_only_review_artifact(self):
        result = self.run_phase()
        self.assertTrue(result["success"], result)
        self.assertEqual(self.generated, [])
        self.assertFalse(list(self.output.rglob("targets.yaml")))
        self.assertEqual(list(self.output.rglob("*.npz")), [])
        envelope = json.loads(Path(result["proposal_path"]).read_text())
        self.assertEqual(envelope["binding"], startup.base_request(self.value))
        self.assertFalse(envelope["fabrication_authorized"])

    def test_generation_without_durable_exact_approval_is_rejected(self):
        proposal = self.run_phase()
        value = dict(
            self.value,
            phase="GENERATE",
            proposal_path=proposal["proposal_path"],
            proposal_sha256=proposal["proposal_sha256"],
        )
        self.assertEqual(self.run_phase(value)["status"], "REJECTED")
        self.assertEqual(self.generated, [])

    def test_full_exact_approved_preview_and_changed_producer_hold(self):
        supervisor = startup.StartupSupervisor(self.root / "journal.jsonl")
        try:
            supervisor.set_armed(True, "explicit-offline-test-operator")
            proposal = supervisor.execute(self.value, self.run_phase)
            value = dict(
                self.value,
                phase="GENERATE",
                proposal_path=proposal["proposal_path"],
                proposal_sha256=proposal["proposal_sha256"],
            )
            supervisor.approve(
                value["request_id"],
                value["proposal_sha256"],
                "explicit-offline-test-operator",
            )
            result = supervisor.execute(
                value,
                lambda: self.run_phase(value, approval_journal_path=supervisor.path),
            )
            self.assertTrue(result["success"], result)
            self.assertEqual(len(self.generated), 1)
        finally:
            supervisor.close()

    def test_changed_planner_kernels_after_approval_prevent_first_batch(self):
        supervisor = startup.StartupSupervisor(self.root / "journal.jsonl")
        try:
            supervisor.set_armed(True, "offline-test-operator")
            proposal = supervisor.execute(self.value, self.run_phase)
            value = dict(
                self.value,
                phase="GENERATE",
                proposal_path=proposal["proposal_path"],
                proposal_sha256=proposal["proposal_sha256"],
            )
            supervisor.approve(
                value["request_id"], value["proposal_sha256"], "offline-test-operator"
            )
            kernel = (
                self.behav
                / "src/behav3d_py/behav3d_py/scalar_field/lib_scalar/__init__.py"
            )
            kernel.write_text("# producer changed after placement review")
            result = supervisor.execute(
                value,
                lambda: self.run_phase(value, approval_journal_path=supervisor.path),
            )
            self.assertFalse(result["success"])
            self.assertEqual(self.generated, [])
        finally:
            supervisor.close()

    def test_modified_proposal_bytes_after_approval_prevent_first_batch(self):
        supervisor = startup.StartupSupervisor(self.root / "journal.jsonl")
        try:
            supervisor.set_armed(True, "offline-test-operator")
            proposal = supervisor.execute(self.value, self.run_phase)
            value = dict(
                self.value,
                phase="GENERATE",
                proposal_path=proposal["proposal_path"],
                proposal_sha256=proposal["proposal_sha256"],
            )
            supervisor.approve(
                value["request_id"], value["proposal_sha256"], "offline-test-operator"
            )
            path = Path(proposal["proposal_path"])
            path.write_text(path.read_text() + " ")
            result = supervisor.execute(
                value,
                lambda: self.run_phase(value, approval_journal_path=supervisor.path),
            )
            self.assertFalse(result["success"])
            self.assertEqual(self.generated, [])
        finally:
            supervisor.close()

    def test_planner_cannot_publish_unsafe_success(self):
        self.planner.propose_wall = lambda *_: {
            "status": "PROPOSED_REVIEW_REQUIRED",
            "requires_review": True,
            "fabrication_authorized": True,
        }
        self.assertFalse(self.run_phase()["success"])
        self.assertEqual(list(self.output.iterdir()), [])

    def test_scene_task_frame_and_manifest_tampering_fail_before_generation(self):
        for change in (
            {"frame_id": "robot-base"},
            {"task_sha256": "f" * 64},
            {"scene_manifest_sha256": "f" * 64},
        ):
            with self.subTest(change=change):
                result = self.run_phase(dict(self.value, **change))
                self.assertFalse(result["success"])
                self.assertEqual(self.generated, [])

    def test_changed_original_scene_sources_rejected(self):
        (self.original / "input.json").write_text("changed")
        self.assertFalse(self.run_phase()["success"])
        self.assertEqual(list(self.output.iterdir()), [])

    def test_output_root_cannot_overlap_frozen_inputs(self):
        self.assertFalse(self.run_phase(output_root=self.scene)["success"])

    def test_planner_hold_does_not_create_artifacts(self):
        self.planner.propose_wall = lambda *_: {"status": "HOLD"}
        self.assertEqual(self.run_phase()["status"], "HOLD")
        self.assertTrue(list(self.output.rglob("HOLD.json")))
        self.assertFalse(list(self.output.rglob("targets.yaml")))
        self.assertFalse(list(self.output.rglob("*.npz")))

    def test_cancelled_before_work_and_late_proposal_are_unaccepted(self):
        self.assertEqual(self.run_phase(cancelled=lambda: True)["status"], "CANCELLED")
        event = threading.Event()
        original = self.planner.propose_wall

        def late(*args):
            result = original(*args)
            event.set()
            return result

        self.planner.propose_wall = late
        self.assertEqual(self.run_phase(cancelled=event.is_set)["status"], "CANCELLED")
        self.assertTrue(list(self.output.rglob("UNACCEPTED.json")))

    def test_changed_task_after_planning_rejected(self):
        original = self.planner.propose_wall

        def change(*args):
            result = original(*args)
            (self.tasks / "task.json").write_text('{"changed":true}')
            return result

        self.planner.propose_wall = change
        self.assertFalse(self.run_phase()["success"])

    def test_hold_cannot_publish_stale_source_report(self):
        def changed_hold(*_args):
            (self.tasks / "task.json").write_text('{"changed_during_hold":true}')
            return {"status": "HOLD", "reason": "No supported placement"}

        self.planner.propose_wall = changed_hold
        result = self.run_phase()
        self.assertEqual(result["status"], "STALE_SOURCE")
        self.assertEqual(result["proposal_path"], "")
        self.assertFalse(list(self.output.rglob("HOLD.json")))

    def test_hold_rechecks_producer_after_report_write(self):
        self.planner.propose_wall = lambda *_args: {"status": "HOLD"}
        original = startup._write

        def changed(path, value):
            original(path, value)
            if path.name == "HOLD.json":
                kernel = (
                    self.behav
                    / "src/behav3d_py/behav3d_py/scalar_field/lib_scalar/__init__.py"
                )
                kernel.write_text("# producer changed after HOLD serialization")

        with patch.object(startup, "_write", changed):
            result = self.run_phase()
        self.assertEqual(result["status"], "STALE_SOURCE")
        self.assertEqual(result["proposal_path"], "")
        self.assertTrue(list(self.output.rglob("UNACCEPTED.json")))

    def test_request_detached_and_proposal_hash_matches_bytes(self):
        before = deepcopy(self.value)
        result = self.run_phase()
        self.assertEqual(self.value, before)
        self.assertEqual(
            self.sha(Path(result["proposal_path"])), result["proposal_sha256"]
        )


if __name__ == "__main__":
    unittest.main()
