"""Synthetic fresh-analysis boundaries; no ROS, models or geometry dependencies."""
import hashlib
import io
import json
import os
import subprocess
import sys
import tempfile
import threading
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_observation_worker as saved_fixtures
from behav3d_orchestrator.supervised_agent import fresh_observation as fresh
from behav3d_orchestrator.supervised_agent import fresh_observation_worker as worker
from behav3d_orchestrator.supervised_agent.observation import validate_result
from behav3d_orchestrator.supervised_agent.supervisor import Supervisor


def checksum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FreshTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.evidence = self.root / "submitted"
        self.output = self.root / "generated"
        self.agent = self.root / "agent007"
        self.dds = self.root / "dds"
        for directory in (self.evidence, self.output, self.agent, self.dds):
            directory.mkdir()
        self.adapter = self.agent / fresh.ADAPTER_RELATIVE
        self.adapter.parent.mkdir(parents=True)
        self.adapter.write_text("# configured producer\n")
        self.folder = self.evidence / "submission"
        (self.folder / "inputs").mkdir(parents=True)
        (self.folder / "inputs/raw.depth").write_bytes(b"raw depth pixels")
        (self.folder / "inputs/provenance.json").write_text("{}\n")
        self.fixture = saved_fixtures.ObservationTests("test_exact_success_contract_and_no_source_mutation")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.request = dict(self.fixture.request, source_manifest_path=str(self.folder / "manifest.json"))
        self.document = {"schema": fresh.INPUT_SCHEMA,
                         "binding": {name: self.request[name] for name in fresh.BINDING_FIELDS},
                         "files": {str(path.relative_to(self.folder)): checksum(path)
                                   for path in sorted((self.folder / "inputs").rglob("*")) if path.is_file()},
                         "previous_job_directory": None, "previous_manifest_sha256": None,
                         "adapter_sha256": checksum(self.adapter)}
        self.publish()
        self.calls = []

    def publish(self):
        (self.folder / "manifest.json").write_text(json.dumps(self.document))
        self.request["source_manifest_sha256"] = checksum(self.folder / "manifest.json")

    def analyse(self, folder, **kwargs):
        self.calls.append((folder, kwargs))
        document = json.loads((folder / "manifest.json").read_text())
        binding = document["binding"]
        previous = kwargs["previous_folder"]
        cycle = binding["candidate_cycle"]
        label = "C" + str(cycle) + ":T1"
        row = {"label": label, "evidence_id": "candidate:" + label,
               "candidate_cycle": cycle, "observation_cycle": cycle + 1,
               "physical_defect_confirmed": False}
        suggestion = deepcopy(self.fixture.suggestion)
        suggestion.update(suggestion_id="review:" + label, evidence_ids=["candidate:" + label],
                          candidate_labels=[label])
        sources = {str(folder / name): expected for name, expected in document["files"].items()}
        evaluation = deepcopy(self.fixture.evaluation)
        evaluation.update(artifact_id=binding["artifact_id"], scan_job_id=binding["observation_id"],
                          scan_cycle=binding["scan_cycle"], candidate_cycle=cycle,
                          previous_evaluation_id=binding["previous_evaluation_id"] or None,
                          source_sha256=sources, suggestions=[suggestion], candidates=[row])
        ledger = [{"scan_cycle": binding["scan_cycle"], "scan_job_id": binding["observation_id"],
                   "source_sha256": sources}]
        if previous is not None:
            old = json.loads((previous / "evaluation/evaluation.json").read_text())
            evaluation["candidates"] = old["candidates"] + [row]
            ledger = old["ledger"] + ledger
        evaluation.update(ledger=ledger, history_start_scan=ledger[0]["scan_cycle"],
                          summary={"current_candidate_count": 1,
                                   "accumulated_candidate_count": len(evaluation["candidates"])})
        shell = deepcopy(self.fixture.shell)
        shell.update(scan_cycle=binding["scan_cycle"], candidate_through_cycle=cycle)
        evaluation["shell"] = shell
        self.fixture.bundle = folder / "evaluation"
        self.fixture.bundle.mkdir()
        self.fixture.evaluation, self.fixture.shell = evaluation, shell
        self.fixture.publish()
        return evaluation

    def run_fresh(self, analyser=None, **kwargs):
        policy = {"agent007_root": self.agent, "dds_root": self.dds, "output_root": self.output}
        policy.update(kwargs)
        with patch.object(fresh, "_analyse", self.analyse if analyser is None else analyser):
            return fresh.evaluate_fresh_observation(self.request, self.evidence, **policy)

    def failed(self, status="STALE_SOURCE", **kwargs):
        result = self.run_fresh(**kwargs)
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], status)
        self.assertEqual(validate_result(result, self.request["request_id"]), result)
        return result

    def markers(self):
        return [json.loads(path.read_text()) for path in self.output.glob("*/analysis_status.json")]

    def test_success_generates_new_bundle_without_mutating_submission_or_request(self):
        original_request = deepcopy(self.request)
        original_files = {path: path.read_bytes() for path in self.folder.rglob("*") if path.is_file()}
        result = self.run_fresh()
        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "REVIEW_REQUIRED")
        self.assertEqual(validate_result(result, self.request["request_id"]), result)
        self.assertNotEqual(result["evidence_manifest_path"], self.request["source_manifest_path"])
        self.assertEqual(self.request, original_request)
        self.assertEqual(original_files, {path: path.read_bytes() for path in self.folder.rglob("*") if path.is_file()})
        self.assertEqual(len(self.calls), 1)
        generated = self.calls[0][0]
        self.assertEqual(json.loads((generated / "job.json").read_text()),
                         {"artifact_id": "artifact-a", "job_id": "scan-2", "previous_scan_job_id": None})
        self.assertEqual(self.markers()[0]["status"], "COMPLETE_REVIEW_REQUIRED")
        self.assertEqual(self.markers()[0]["supervisor_acceptance"], "not_recorded")
        self.assertFalse(self.markers()[0]["fabrication_authorized"])

    def test_fresh_job_never_impersonates_saved_bundle(self):
        self.document["schema"] = "agent007.evaluation_bundle.v1"
        self.publish()
        self.failed()
        self.assertEqual(list(self.output.iterdir()), [])

    def test_extra_manifest_fields_and_extra_binding_fields_rejected(self):
        for kind in ("root", "binding"):
            with self.subTest(kind=kind):
                original = deepcopy(self.document)
                (self.document if kind == "root" else self.document["binding"])["command"] = "print"
                self.publish()
                self.failed()
                self.document = original
        self.assertEqual(self.calls, [])

    def test_binding_identity_frame_units_and_previous_mismatch_rejected(self):
        for name, value in (("artifact_id", "other"), ("observation_id", "other"),
                            ("frame_id", "world"), ("coordinate_units", "mm"),
                            ("candidate_cycle", 0), ("scan_cycle", 3),
                            ("previous_evaluation_id", "evaluation_" + "1" * 64)):
            with self.subTest(name=name):
                original = self.document["binding"][name]
                self.document["binding"][name] = value
                self.publish()
                self.failed()
                self.document["binding"][name] = original

    def test_binding_numbers_cannot_be_booleans_or_floats(self):
        for name, value in (("candidate_cycle", True), ("scan_cycle", 2.0)):
            with self.subTest(name=name):
                original = self.document["binding"][name]
                self.document["binding"][name] = value
                self.publish()
                self.failed()
                self.document["binding"][name] = original

    def test_manifest_hash_mismatch_rejected_before_output(self):
        self.request["source_manifest_sha256"] = "0" * 64
        self.failed()
        self.assertEqual(list(self.output.iterdir()), [])

    def test_duplicate_keys_and_nonfinite_json_rejected(self):
        for raw in ('{"schema":1,"schema":2}', '{"binding":NaN}'):
            with self.subTest(raw=raw):
                (self.folder / "manifest.json").write_text(raw)
                self.request["source_manifest_sha256"] = checksum(self.folder / "manifest.json")
                self.failed()

    def test_missing_changed_and_extra_inputs_rejected(self):
        original = (self.folder / "inputs/raw.depth").read_bytes()
        (self.folder / "inputs/raw.depth").write_bytes(b"changed")
        self.failed()
        (self.folder / "inputs/raw.depth").unlink()
        self.failed()
        (self.folder / "inputs/raw.depth").write_bytes(original)
        (self.folder / "inputs/unlisted.depth").write_bytes(b"extra")
        self.failed()
        self.assertEqual(list(self.output.iterdir()), [])

    def test_unsafe_relative_input_paths_rejected(self):
        for name in ("../outside", "/absolute", "inputs/../outside", "inputs//raw.depth", "outputs/x", "inputs/./raw.depth"):
            with self.subTest(name=name):
                original = self.document["files"]
                self.document["files"] = {name: "a" * 64}
                self.publish()
                self.failed()
                self.document["files"] = original

    def test_input_file_symlinks_and_directory_symlinks_rejected(self):
        original = self.folder / "inputs/raw.depth"
        target = self.root / "outside.depth"
        target.write_bytes(original.read_bytes())
        original.unlink()
        original.symlink_to(target)
        self.failed()
        original.unlink()
        original.write_bytes(target.read_bytes())
        (self.folder / "inputs/linked").symlink_to(self.dds, target_is_directory=True)
        self.failed()

    @unittest.skipUnless(hasattr(os, "mkfifo"), "POSIX FIFO required")
    def test_special_input_file_rejected_without_blocking(self):
        os.mkfifo(self.folder / "inputs/fifo")
        self.failed()

    def test_manifest_outside_evidence_root_and_symlink_alias_rejected(self):
        self.request["source_manifest_path"] = str(self.fixture.bundle / "manifest.json")
        self.failed()
        alias = self.evidence / "alias.json"
        alias.symlink_to(self.folder / "manifest.json")
        self.request["source_manifest_path"] = str(alias)
        self.failed()

    def test_invalid_policy_never_creates_output(self):
        for policy in ({"agent007_root": "relative"}, {"dds_root": self.root / "missing"},
                       {"output_root": self.evidence}, {"output_root": self.folder},
                       {"source_roots": [self.root]}):
            with self.subTest(policy=policy):
                self.failed("FAILED", **policy)
        self.assertEqual(list(self.output.iterdir()), [])

    def test_output_root_symlink_rejected(self):
        alias = self.root / "output-alias"
        alias.symlink_to(self.output, target_is_directory=True)
        self.failed("FAILED", output_root=alias)

    def test_adapter_hash_change_rejected_before_analysis(self):
        self.adapter.write_text("# different producer")
        self.failed()
        self.assertEqual(list(self.output.iterdir()), [])

    def test_cancel_before_dispatch_creates_nothing(self):
        self.failed("CANCELLED", cancelled=lambda: True)
        self.assertEqual(list(self.output.iterdir()), [])
        self.assertEqual(self.calls, [])

    def test_cancel_after_copy_never_calls_analysis(self):
        event = threading.Event()
        original = fresh._copy_inputs
        def copy(*args):
            original(*args)
            event.set()
        with patch.object(fresh, "_copy_inputs", copy):
            self.failed("CANCELLED", cancelled=event.is_set)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.markers()[0]["status"], "UNACCEPTED")

    def test_cancel_after_analysis_discards_generated_result(self):
        event = threading.Event()
        def analyse(*args, **kwargs):
            self.analyse(*args, **kwargs)
            event.set()
        self.failed("CANCELLED", analyser=analyse, cancelled=event.is_set)
        self.assertEqual(self.markers()[0]["status"], "UNACCEPTED")

    def test_runtime_analysis_exception_is_failed_without_evidence(self):
        def fail(*_args, **_kwargs):
            raise RuntimeError("private detail must not be exposed")
        result = self.failed("FAILED", analyser=fail)
        self.assertNotIn("private", result["message"])
        self.assertEqual(self.markers()[0]["status"], "UNACCEPTED")
        diagnostic = self.markers()[0]["diagnostic"]
        self.assertEqual(diagnostic["exception_type"], "RuntimeError")
        self.assertEqual(diagnostic["stage"], "local_geometry_analysis")
        self.assertTrue(diagnostic["frames"])
        self.assertNotIn("private detail must not be exposed", json.dumps(diagnostic))

    def test_missing_generated_bundle_rejected(self):
        self.failed(analyser=lambda *_args, **_kwargs: None)
        self.assertEqual(self.markers()[0]["status"], "UNACCEPTED")

    def test_input_or_producer_mutation_during_analysis_rejected(self):
        for destination in ("original", "copy", "adapter"):
            with self.subTest(destination=destination):
                def mutate(folder, destination=destination, **kwargs):
                    self.analyse(folder, **kwargs)
                    path = {"original": self.folder / "inputs/raw.depth", "copy": folder / "inputs/raw.depth",
                            "adapter": self.adapter}[destination]
                    path.write_bytes(path.read_bytes() + b"changed")
                original_data = (self.folder / "inputs/raw.depth").read_bytes()
                original_code = self.adapter.read_bytes()
                self.failed(analyser=mutate)
                (self.folder / "inputs/raw.depth").write_bytes(original_data)
                self.adapter.write_bytes(original_code)

    def test_generated_authority_and_wrong_observation_rejected(self):
        for field, value in (("fabrication_authorized", True), ("scan_job_id", "wrong")):
            with self.subTest(field=field):
                def bad(folder, field=field, value=value, **kwargs):
                    self.analyse(folder, **kwargs)
                    self.fixture.evaluation[field] = value
                    self.fixture.publish()
                self.failed(analyser=bad)

    def test_generated_sources_must_stay_inside_server_output_root(self):
        def bad(folder, **kwargs):
            self.analyse(folder, **kwargs)
            sources = {str(self.folder / "inputs/raw.depth"): checksum(self.folder / "inputs/raw.depth")}
            self.fixture.evaluation["source_sha256"] = sources
            self.fixture.evaluation["ledger"][-1]["source_sha256"] = sources
            self.fixture.publish()
        self.failed(analyser=bad)

    def test_output_directory_collision_never_overwrites(self):
        class Fixed:
            hex = "a" * 32
        collision = self.output / ("fresh-" + Fixed.hex)
        collision.mkdir()
        sentinel = collision / "retain"
        sentinel.write_text("user content")
        with patch.object(fresh.uuid, "uuid4", return_value=Fixed()):
            self.failed()
        self.assertEqual(sentinel.read_text(), "user content")
        self.assertEqual(self.calls, [])

    def test_initial_request_cannot_invent_previous_job(self):
        self.document.update(previous_job_directory=str(self.output / "old"), previous_manifest_sha256="1" * 64)
        self.publish()
        self.failed()

    def test_previous_job_chain_verified_before_next_analysis(self):
        first = self.run_fresh()
        self.assertTrue(first["success"])
        previous = self.calls[-1][0]
        self.request.update(request_id="request-b", observation_id="scan-3", candidate_cycle=2, scan_cycle=3,
                            previous_evaluation_id=first["evaluation_id"])
        self.document["binding"] = {name: self.request[name] for name in fresh.BINDING_FIELDS}
        self.document.update(previous_job_directory=str(previous), previous_manifest_sha256=first["evidence_manifest_sha256"])
        self.publish()
        second = self.run_fresh()
        self.assertTrue(second["success"], second)
        self.assertEqual(self.calls[-1][1]["previous_folder"], previous)
        self.assertEqual(json.loads((self.calls[-1][0] / "job.json").read_text())["previous_scan_job_id"], "scan-2")

    def test_previous_job_hash_incomplete_and_escape_rejected(self):
        first = self.run_fresh()
        previous = self.calls[-1][0]
        self.request.update(request_id="request-b", observation_id="scan-3", candidate_cycle=2, scan_cycle=3,
                            previous_evaluation_id=first["evaluation_id"])
        self.document["binding"] = {name: self.request[name] for name in fresh.BINDING_FIELDS}
        self.document.update(previous_job_directory=str(previous), previous_manifest_sha256="0" * 64)
        self.publish()
        self.failed()
        self.document["previous_manifest_sha256"] = first["evidence_manifest_sha256"]
        marker_path = previous / "analysis_status.json"
        marker = json.loads(marker_path.read_text())
        marker["status"] = "INCOMPLETE"
        marker_path.write_text(json.dumps(marker))
        self.publish()
        self.failed()
        self.document["previous_job_directory"] = str(self.folder)
        self.publish()
        self.failed()
        self.assertEqual(len(self.calls), 1)

    def test_request_extensions_and_unsupported_event_do_not_open_inputs(self):
        self.request["backend"] = "arbitrary"
        self.failed("REJECTED")
        del self.request["backend"]
        self.request.update(event_type="inspection", candidate_cycle=-1, source_manifest_path=str(self.root / "absent"))
        self.failed("UNSUPPORTED")
        self.assertEqual(list(self.output.iterdir()), [])

    def test_supervisor_receipt_retries_do_not_generate_again_after_restart(self):
        journal = self.root / "journal/review.jsonl"
        supervisor = Supervisor(journal)
        supervisor.set_armed(True, "offline-test")
        proposal = supervisor.propose("evaluate_observation", self.request)
        supervisor.approve(self.request["request_id"], proposal["proposal_sha256"], "offline-test")
        def operation():
            result = self.run_fresh()
            return {"ok": result["success"], "kind": "evaluate_observation", "phase": "plan",
                    "planned_only": True, "error": None, "metrics": result}
        first = supervisor.execute(self.request["request_id"], proposal["proposal_sha256"], operation)
        self.assertTrue(first["ok"])
        self.assertEqual(supervisor.execute(self.request["request_id"], proposal["proposal_sha256"], operation), first)
        supervisor.close()
        supervisor = Supervisor(journal)
        try:
            self.assertFalse(supervisor.snapshot()["armed"])
            self.assertEqual(supervisor.execute(self.request["request_id"], proposal["proposal_sha256"], operation), first)
        finally:
            supervisor.close()
        self.assertEqual(len(self.calls), 1)

    def test_worker_invalid_cli_and_duplicate_input_return_one_result(self):
        output = io.StringIO()
        with patch.object(sys, "stdout", output):
            self.assertEqual(worker.main([]), 0)
        self.assertEqual(json.loads(output.getvalue())["status"], "FAILED")
        output = io.StringIO()
        stdin = type("Input", (), {"buffer": io.BytesIO(b'{"request_id":"a","request_id":"b"}')})()
        args = ["--evidence-root", str(self.evidence), "--agent007-root", str(self.agent),
                "--dds-root", str(self.dds), "--output-root", str(self.output)]
        with patch.object(sys, "stdin", stdin), patch.object(sys, "stdout", output):
            self.assertEqual(worker.main(args), 0)
        self.assertEqual(json.loads(output.getvalue())["status"], "REJECTED")

    def test_c_library_stdout_redirect_preserves_single_json_channel(self):
        # A separate process proves fd 1, not just Python's file object, moves.
        root = str(Path(worker.__file__).parent.parent.parent)
        program = (
            "import ctypes,os,sys\n"
            "sys.path.insert(0," + repr(root) + ")\n"
            "from behav3d_orchestrator.supervised_agent.fresh_observation_worker import _analysis_stdout_to_stderr\n"
            "with _analysis_stdout_to_stderr():\n"
            " os.write(1,b'C++ progress\\n')\n"
            " ctypes.CDLL(None).printf(b'buffered native progress\\n')\n"
            " print('Python progress',flush=True)\n"
            "print('{}')\n")
        result = subprocess.run([sys.executable, "-B", "-c", program], capture_output=True, timeout=5, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, b"{}\n")
        self.assertIn(b"C++ progress", result.stderr)
        self.assertIn(b"buffered native progress", result.stderr)
        self.assertIn(b"Python progress", result.stderr)


if __name__ == "__main__":
    unittest.main()
