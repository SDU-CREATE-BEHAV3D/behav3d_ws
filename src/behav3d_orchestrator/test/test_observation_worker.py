"""Pure-stdlib synthetic evidence tests. No ROS, model or physical actions."""
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behav3d_orchestrator.supervised_agent.observation import (
    REQUEST_FIELDS, RESULT_FIELDS, decode_request, evaluate_recorded_observation,
    request_from_goal, result_for_error, validate_request, validate_result,
)
from behav3d_orchestrator.supervised_agent import observation_worker


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.bundle = self.root / "evidence"
        self.bundle.mkdir()
        self.source = self.root / "source.txt"
        self.source.write_text("immutable source", encoding="utf-8")
        self.shell = {
            "schema": "ocsr.planned_observed_shell.v1", "scan_cycle": 2,
            "candidate_through_cycle": 1, "future_evidence_used": False,
            "physical_accuracy_validated": False, "robot_commands_sent": False,
            "expected_reference_uses_observations": False,
            "observed_middle_is_measured_volume_centre": False,
        }
        self.suggestion = {
            "suggestion_id": "review:C1:T1", "action": "review_scan_coverage",
            "evidence_ids": ["candidate:C1:T1"], "candidate_labels": ["C1:T1"],
            "reason": "cap_quality_review", "requires_operator_review": True,
            "executable": False, "fabrication_authorized": False,
        }
        sources = {str(self.source): file_hash(self.source)}
        self.evaluation = {
            "schema": "agent007.integrated_evaluation.v1",
            "artifact_id": "artifact-a", "scan_job_id": "scan-2",
            "scan_cycle": 2, "candidate_cycle": 1, "previous_evaluation_id": None,
            "history_start_scan": 2,
            "context": {"frame_id": "base", "geometry_profile_sha256": "a" * 64},
            "source_sha256": sources,
            "ledger": [{"scan_cycle": 2, "scan_job_id": "scan-2", "source_sha256": sources}],
            "shell": self.shell,
            "candidates": [{"label": "C1:T1", "evidence_id": "candidate:C1:T1",
                            "candidate_cycle": 1, "observation_cycle": 2,
                            "physical_defect_confirmed": False}],
            "summary": {"current_candidate_count": 1, "accumulated_candidate_count": 1},
            "suggestions": [self.suggestion], "limitations": ["Physical accuracy unvalidated."],
            "future_evidence_used": False, "physical_accuracy_validated": False,
            "robot_commands_sent": False, "fabrication_authorized": False,
        }
        self.request = {
            "request_id": "request-a", "artifact_id": "artifact-a", "observation_id": "scan-2",
            "event_type": "deposition_readback", "candidate_cycle": 1, "scan_cycle": 2,
            "previous_evaluation_id": "", "frame_id": "base", "coordinate_units": "m",
            "source_manifest_path": str(self.bundle / "manifest.json"),
            "source_manifest_sha256": "0" * 64,
        }
        self.publish()

    def publish(self):
        self.evaluation.pop("evaluation_id", None)
        self.evaluation["evaluation_id"] = "evaluation_" + digest(self.evaluation)
        value = self.evaluation
        self.packet = {
            "schema": "agent007.robot_review_packet.v1", "artifact_id": value["artifact_id"],
            "evaluation_id": value["evaluation_id"], "scan_job_id": value["scan_job_id"],
            "scan_cycle": value["scan_cycle"], "candidate_cycle": value["candidate_cycle"],
            "previous_evaluation_id": value["previous_evaluation_id"], "frame_id": value["context"]["frame_id"],
            "coordinate_units": "m", "summary": value["summary"], "registration_status": "unverified",
            "suggestions": value["suggestions"][:100], "suggestion_count": len(value["suggestions"]),
            "suggestions_truncated": len(value["suggestions"]) > 100, "limitations": value["limitations"],
            "future_evidence_used": False, "physical_accuracy_validated": False,
            "structural_evaluation": "not_performed", "fabrication_authorized": False,
            "robot_commands_sent": False,
        }
        docs = {"evaluation.json": value, "shell.json": self.shell,
                "robot_review_packet.json": self.packet}
        for name, document in docs.items():
            (self.bundle / name).write_text(json.dumps(document, allow_nan=False), encoding="utf-8")
        manifest = {"schema": "agent007.evaluation_bundle.v1",
                    "files": {name: file_hash(self.bundle / name) for name in docs}}
        (self.bundle / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        self.request["source_manifest_sha256"] = file_hash(self.bundle / "manifest.json")

    def evaluate(self, **kwargs):
        return evaluate_recorded_observation(self.request, self.root, **kwargs)

    def test_exact_success_contract_and_no_source_mutation(self):
        before = {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        result = self.evaluate()
        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "REVIEW_REQUIRED")
        self.assertEqual(set(result), set(RESULT_FIELDS))
        self.assertTrue(result["requires_review"])
        self.assertFalse(result["fabrication_authorized"])
        self.assertEqual(validate_result(result, "request-a"), result)
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()})

    def test_goal_copy_and_detached_validation(self):
        request = request_from_goal(SimpleNamespace(**self.request))
        self.assertEqual(tuple(request), REQUEST_FIELDS)
        request["request_id"] = "other"
        self.assertEqual(self.request["request_id"], "request-a")
        self.assertEqual(validate_request(self.request), self.request)

    def test_request_fields_types_cycles_and_hashes_fail_closed(self):
        changes = ({"robot_commands": []}, {"candidate_cycle": True}, {"scan_cycle": 5},
                   {"coordinate_units": "mm"}, {"frame_id": ""}, {"previous_evaluation_id": None},
                   {"source_manifest_path": "../manifest.json"}, {"source_manifest_sha256": "A" * 64},
                   {"event_type": "execute"}, {"source_roots": [str(self.root)]})
        for change in changes:
            with self.subTest(change=change):
                request = {**self.request, **change}
                self.assertFalse(evaluate_recorded_observation(request, self.root)["success"])
                with self.assertRaises((ValueError, TypeError)):
                    validate_request(request)

    def test_duplicate_nonfinite_and_oversized_json_rejected(self):
        for raw in (b'{"request_id":"a","request_id":"b"}', b'{"x":NaN}',
                    b'{"x":1e999}', b" " * 17000):
            with self.subTest(raw=raw[:32]):
                with self.assertRaises(ValueError):
                    decode_request(raw)

    def test_unsupported_events_never_open_bundle(self):
        self.request["source_manifest_path"] = str(self.root / "not-there" / "manifest.json")
        for event in ("inspection", "intervention_readback", "recovery_verification"):
            self.request["event_type"] = event
            self.request["candidate_cycle"] = -1
            self.request["scan_cycle"] = 2
            self.assertEqual(validate_request(self.request)["candidate_cycle"], -1)
            self.assertEqual(self.evaluate()["status"], "UNSUPPORTED")

    def test_manifest_file_and_revision_tampering_rejected(self):
        for filename in ("manifest.json", "evaluation.json", "shell.json", "robot_review_packet.json"):
            self.publish()
            path = self.bundle / filename
            path.write_bytes(path.read_bytes() + b" ")
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        self.publish()
        self.evaluation["evaluation_id"] = "evaluation_" + "f" * 64
        path = self.bundle / "evaluation.json"
        path.write_text(json.dumps(self.evaluation))
        manifest = json.loads((self.bundle / "manifest.json").read_text())
        manifest["files"]["evaluation.json"] = file_hash(path)
        (self.bundle / "manifest.json").write_text(json.dumps(manifest))
        self.request["source_manifest_sha256"] = file_hash(self.bundle / "manifest.json")
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_exact_identity_frame_previous_and_observation_required(self):
        for change in ({"artifact_id": "different"}, {"observation_id": "scan-3"},
                       {"frame_id": "world/base"}, {"previous_evaluation_id": "evaluation_" + "a" * 64}):
            with self.subTest(change=change):
                result = evaluate_recorded_observation({**self.request, **change}, self.root)
                self.assertEqual(result["status"], "STALE_SOURCE")

    def test_all_safety_flags_and_reference_contracts_checked(self):
        original = deepcopy(self.evaluation)
        for field in ("fabrication_authorized", "robot_commands_sent", "physical_accuracy_validated",
                      "future_evidence_used"):
            self.evaluation = deepcopy(original)
            self.evaluation[field] = True
            self.publish()
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        self.evaluation = deepcopy(original)
        self.evaluation["suggestions"][0]["executable"] = True
        self.publish()
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        self.evaluation = deepcopy(original)
        self.evaluation["suggestions"][0]["evidence_ids"] = ["candidate:unknown"]
        self.publish()
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_duplicate_candidate_and_suggestion_ids_rejected(self):
        original = deepcopy(self.evaluation)
        self.evaluation["candidates"].append(deepcopy(self.evaluation["candidates"][0]))
        self.evaluation["summary"]["accumulated_candidate_count"] = 2
        self.publish()
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        self.evaluation = deepcopy(original)
        self.evaluation["suggestions"].append(deepcopy(self.evaluation["suggestions"][0]))
        self.publish()
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_current_and_historical_sources_checked(self):
        self.source.write_text("changed")
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        self.publish()
        self.evaluation["source_sha256"] = {str(self.bundle / "not-there"): "a" * 64}
        self.publish()
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_source_roots_are_explicit_server_policy(self):
        result = evaluate_recorded_observation(self.request, self.bundle)
        self.assertEqual(result["status"], "STALE_SOURCE")
        self.assertTrue(evaluate_recorded_observation(self.request, self.bundle,
                                                      source_roots=(self.root,))["success"])
        outside = self.root / "outside"
        outside.mkdir()
        self.assertEqual(evaluate_recorded_observation(self.request, outside,
                                                       source_roots=(self.root,))["status"], "STALE_SOURCE")

    def test_bundle_traversal_and_symlink_escape_rejected(self):
        manifest = json.loads((self.bundle / "manifest.json").read_text())
        manifest["files"]["../outside.json"] = manifest["files"].pop("shell.json")
        (self.bundle / "manifest.json").write_text(json.dumps(manifest))
        self.request["source_manifest_sha256"] = file_hash(self.bundle / "manifest.json")
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        self.publish()
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory).resolve() / "shell.json"
            target.write_bytes((self.bundle / "shell.json").read_bytes())
            (self.bundle / "shell.json").unlink()
            (self.bundle / "shell.json").symlink_to(target)
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_cancellation_checked_before_and_after_sources(self):
        self.assertEqual(self.evaluate(cancelled=lambda: True)["status"], "CANCELLED")
        calls = [0]
        def cancelled():
            calls[0] += 1
            return calls[0] >= 12
        self.assertEqual(self.evaluate(cancelled=cancelled)["status"], "CANCELLED")

    def test_bounded_file_and_total_hash_reads(self):
        from behav3d_orchestrator.supervised_agent import observation
        with patch.object(observation, "MAX_JSON_BYTES", 10):
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        with patch.object(observation, "MAX_TOTAL_HASH_BYTES", 10):
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
        with patch.object(observation, "MAX_FILE_BYTES", 2):
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_result_validation_rejects_authority_extra_fields_and_false_pass(self):
        result = self.evaluate()
        for change in ({"fabrication_authorized": True}, {"requires_review": False},
                       {"status": "PASS"}, {"request_id": "another"}, {"packet_sha256": "wrong"},
                       {"robot_commands": []}):
            with self.subTest(change=change):
                with self.assertRaises(ValueError):
                    validate_result({**result, **change}, "request-a")
        self.assertEqual(result_for_error("request-a", "TIMED_OUT", "Worker timeout.")["status"], "TIMED_OUT")

    def test_worker_stdout_is_one_final_json_and_stderr_is_bounded_feedback(self):
        stdin = io.TextIOWrapper(io.BytesIO(json.dumps(self.request).encode()))
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch("sys.stdin", stdin), patch("sys.stdout", stdout), patch("sys.stderr", stderr):
            code = observation_worker.main(["--evidence-root", str(self.root)])
        self.assertEqual(code, 0)
        result = json.loads(stdout.getvalue())
        self.assertTrue(result["success"])
        self.assertEqual(len(stdout.getvalue().splitlines()), 1)
        self.assertLess(len(stderr.getvalue()), 1024)
        self.assertEqual(json.loads(stderr.getvalue())["stage"], "verifying")

    def test_worker_rejects_duplicate_keys_without_echoing_input(self):
        stdin = io.TextIOWrapper(io.BytesIO(b'{"request_id":"private","request_id":"secret"}'))
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch("sys.stdin", stdin), patch("sys.stdout", stdout), patch("sys.stderr", stderr):
            observation_worker.main(["--evidence-root", str(self.root)])
        self.assertEqual(json.loads(stdout.getvalue())["status"], "REJECTED")
        self.assertNotIn("private", stdout.getvalue())
        self.assertNotIn("secret", stdout.getvalue())


    def test_historical_source_not_in_current_map_is_verified(self):
        historical = self.root / "older-source.txt"
        historical.write_text("older capture")
        self.evaluation["ledger"][0]["source_sha256"] = {
            str(historical): file_hash(historical)}
        self.publish()
        self.assertTrue(self.evaluate()["success"])
        historical.write_text("changed older capture")
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_source_change_between_hash_passes_is_rejected(self):
        from behav3d_orchestrator.supervised_agent import observation
        original = observation._file
        mutated = [False]
        def read(*args, **kwargs):
            result = original(*args, **kwargs)
            if result[0] == self.source and not mutated[0]:
                self.source.write_text("changed between passes")
                mutated[0] = True
            return result
        with patch.object(observation, "_file", side_effect=read):
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_nested_protocol_numbers_cannot_be_booleans_or_floats(self):
        for key in ("current_candidate_count", "accumulated_candidate_count"):
            original = deepcopy(self.evaluation)
            self.evaluation["summary"][key] = True
            self.publish()
            self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")
            self.evaluation = original
        self.publish()
        self.shell["scan_cycle"] = 2.0
        self.publish()
        self.assertEqual(self.evaluate()["status"], "STALE_SOURCE")

    def test_validation_helpers_always_reject_unhashable_protocol_values(self):
        with self.assertRaises(ValueError):
            validate_request({**self.request, "event_type": []})
        result = self.evaluate()
        with self.assertRaises(ValueError):
            validate_result({**result, "status": []}, self.request["request_id"])

    def test_worker_invalid_cli_is_one_failed_result(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch("sys.stdout", stdout), patch("sys.stderr", stderr):
            observation_worker.main([])
        self.assertEqual(json.loads(stdout.getvalue())["status"], "FAILED")
        self.assertEqual(stderr.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
