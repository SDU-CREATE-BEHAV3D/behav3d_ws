"""Recorded-scene startup rehearsal; never starts ROS or physical commands.

Prepare a new run, then choose a labelled automatic software-test review or a
single long-lived human review process. Startup approvals deliberately do not
survive restart. A closed/interrupted run must be prepared again with a new ID.
The shared native planner and durable StartupSupervisor remain the gate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = "behav3d.recorded_startup_rehearsal.v1"
CHECKOUT = Path(__file__).resolve().parents[1]


def emit_line(value):
    print(value, flush=True)


def checksum(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError("Rehearsal JSON exceeds limit")
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def canonical(path, *, exists=True):
    path = Path(path)
    if not path.is_absolute() or path.resolve(strict=exists) != path or path.is_symlink():
        raise ValueError("Explicit canonical absolute paths required")
    return path


def prepare_run(*, output, scene_manifest, task, frame_id, request_id,
                artifact_id, input_roots, agent007_root, behav3d_root):
    """Bind existing recorded evidence; no planning, approval or generation."""
    output = canonical(output, exists=False)
    scene_manifest, task = canonical(scene_manifest), canonical(task)
    agent007_root, behav3d_root = canonical(agent007_root), canonical(behav3d_root)
    roots = [canonical(root) for root in input_roots]
    if behav3d_root != CHECKOUT:
        raise ValueError("BEHAV root must name this explicit checkout")
    if not roots or not (agent007_root / "src/ocsr/initial_scene.py").is_file():
        raise ValueError("Configured source roots are incomplete")
    for root in (*roots, agent007_root, behav3d_root):
        if output.is_relative_to(root) or root.is_relative_to(output):
            raise ValueError("Rehearsal output must be separate from immutable inputs/code")
    for path in (scene_manifest, task):
        if not any(path.is_relative_to(root) for root in roots):
            raise ValueError("Input artifact lies outside explicit allowed roots")
    manifest = read_json(scene_manifest)
    if (manifest.get("schema") != "agent007.initial_scene.v1"
            or manifest.get("frame_id") != frame_id):
        raise ValueError("Recorded scene/frame binding mismatch")
    request = {"phase": "PROPOSE", "request_id": request_id, "artifact_id": artifact_id,
               "frame_id": frame_id, "scene_manifest_path": str(scene_manifest),
               "scene_manifest_sha256": checksum(scene_manifest), "task_path": str(task),
               "task_sha256": checksum(task), "proposal_path": "", "proposal_sha256": ""}
    sys.path.insert(0, str(behav3d_root / "src/behav3d_orchestrator"))
    from behav3d_orchestrator.supervised_agent.initial_workflow import validate_request
    validate_request(request)
    output.mkdir(mode=0o700, exist_ok=False)
    (output / "products").mkdir(mode=0o700)
    config = {"schema": SCHEMA, "run_path": str(output), "request": request,
              "input_roots": [str(root) for root in roots],
              "agent007_root": str(agent007_root), "behav3d_root": str(behav3d_root),
              "recorded_evidence_only": True, "robot_commands_sent": False,
              "fabrication_authorized": False}
    write_json(output / "configuration.json", config)
    return config


def load_run(path):
    run = canonical(path)
    config = read_json(run / "configuration.json")
    if (config.get("schema") != SCHEMA or config.get("run_path") != str(run)
            or config.get("recorded_evidence_only") is not True
            or config.get("robot_commands_sent") is not False
            or config.get("fabrication_authorized") is not False
            or canonical(config["behav3d_root"]) != CHECKOUT):
        raise ValueError("Prepared rehearsal identity/safety mismatch")
    for name in ("events.jsonl", "summary.json", "startup.jsonl"):
        if (run / name).exists():
            raise ValueError("Run already started; prepare a new run and review again")
    for name in ("agent007_root", "behav3d_root"):
        canonical(config[name])
    for root in config["input_roots"]:
        canonical(root)
    products = canonical(run / "products")
    if products.parent != run:
        raise ValueError("Products must remain confined to this rehearsal")
    for root in (*config["input_roots"], config["agent007_root"], config["behav3d_root"]):
        root = Path(root)
        if run.is_relative_to(root) or root.is_relative_to(run):
            raise ValueError("Rehearsal output overlaps immutable roots")
    return config


class Events:
    """Durable inspectable process receipts, separate from the approval journal."""
    def __init__(self, path, *, mode, emit=emit_line):
        self.stream = Path(path).open("x", encoding="utf-8")  # noqa: SIM115 -- owned until close()
        self.sequence, self.mode, self.emit = 0, mode, emit

    def add(self, event, **fields):
        self.sequence += 1
        row = {"sequence": self.sequence, "utc": datetime.now(timezone.utc).isoformat(),
               "mode": self.mode, "event": event, "fabrication_authorized": False,
               "robot_commands_sent": False, **fields}
        text = json.dumps(row, separators=(",", ":"), allow_nan=False)
        self.stream.write(text + "\n")
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.emit(text)
        return row

    def close(self):
        self.stream.close()


def generated_products(root):
    return sorted(str(path) for name in ("targets.yaml", "field_state_init.npz", "goal_mesh.ply")
                  for path in Path(root).rglob(name))


def audit_proposal(config, result):
    """Recheck the exact selected geometry; never search or move it."""
    request = config["request"]
    path = Path(result["proposal_path"])
    if checksum(path) != result["proposal_sha256"]:
        raise ValueError("Proposal bytes changed before review")
    envelope = read_json(path)
    from behav3d_orchestrator.supervised_agent.initial_planning import validate_proposal
    from behav3d_orchestrator.supervised_agent.initial_workflow import (
        _producer,
        base_request,
    )
    from ocsr.initial_scene import load_scene
    if (envelope.get("schema") != "behav3d.initial_wall_proposal.v1"
            or envelope.get("binding") != base_request(request)
            or envelope.get("producer_sha256") != _producer(config["behav3d_root"])
            or envelope.get("requires_review") is not True
            or envelope.get("fabrication_authorized") is not False):
        raise ValueError("Proposal bindings/safety failed")
    for path_name, hash_name in (("scene_manifest_path", "scene_manifest_sha256"), ("task_path", "task_sha256")):
        if checksum(request[path_name]) != request[hash_name]:
            raise ValueError("Recorded scene/task changed before review")
    loaded = load_scene(Path(request["scene_manifest_path"]).parent, frame_id=request["frame_id"])
    checked = validate_proposal(loaded["packet"], loaded["geometry"], read_json(request["task_path"]),
                                envelope["planning"])
    if checked.get("valid") is not True:
        raise ValueError("Selected pose no longer validates")
    initialization = audit_initialization(envelope["planning"])
    return {"passed": True, "proposal_id": envelope["planning"]["proposal_id"],
            "selected": envelope["planning"]["selected"],
            "native_initialization_audit": initialization,
            "registration_status": loaded["manifest"]["registration_status"],
            "scope": "Offline geometric consistency only; not robot or physical safety approval"}


def audit_initialization(planning):
    """Require the actual lowered goal, native heat and nonempty intersection."""
    import numpy as np
    evidence, task, selected = planning["initialization"], planning["task"], planning["selected"]
    if evidence.get("schema") != "behav3d.initial_goal_initialization.v1":
        raise ValueError("Native goal initialization evidence is required")
    vertices = np.asarray(evidence["goal_vertices_m"], dtype=float)
    faces = np.asarray(evidence["goal_faces"], dtype=int)
    phi, heat = np.asarray(evidence["phi_m"]), np.asarray(evidence["heat_norm"])
    frontier = np.asarray(evidence["frontier_points_m"], dtype=float)
    lines = np.asarray(evidence["frontier_lines"], dtype=int)
    if (vertices.ndim != 2 or vertices.shape[1:] != (3,) or not np.isfinite(vertices).all()
            or faces.ndim != 2 or faces.shape[1:] != (3,) or not len(faces)
            or faces.min() < 0 or faces.max() >= len(vertices)
            or phi.shape != (len(vertices),) or heat.shape != phi.shape
            or not np.isfinite(phi).all() or not np.isfinite(heat).all()
            or not float(phi.min()) <= task["iso_level_m"] < float(phi.max())
            or float(heat.min()) < 0 or float(heat.max()) > 1 or np.ptp(heat) <= 0):
        raise ValueError("Actual goal mesh/heat/intersection evidence is invalid")
    if (frontier.ndim != 2 or frontier.shape[1:] != (3,) or not np.isfinite(frontier).all()
            or lines.ndim != 2 or lines.shape[1:] != (2,) or not len(lines)
            or lines.min() < 0 or lines.max() >= len(frontier)
            or not np.any(np.linalg.norm(frontier[lines[:, 1]] - frontier[lines[:, 0]], axis=1) > 1e-9)):
        raise ValueError("Native goal/support frontier must contain nonzero intersection segments")
    angle = np.deg2rad(selected["yaw_deg"])
    tangent = np.asarray([np.cos(angle), np.sin(angle)])
    if (not np.isclose(np.ptp(vertices[:, :2] @ tangent), task["width_m"], atol=1e-8, rtol=0)
            or not np.isclose(np.ptp(vertices[:, 2]), task["height_m"], atol=1e-8, rtol=0)
            or not np.isclose(vertices[:, 2].min(), selected["center_m"][2], atol=1e-8, rtol=0)):
        raise ValueError("Goal mesh dimensions/base differ from the reviewed placement")
    metrics = evidence["metrics"]
    if (metrics["field_guard_row_below_base_m"] != 0
            or metrics["ray_hit_count"] != len(vertices)
            or metrics["goal_vertex_count"] != len(vertices)
            or metrics["buried_vertex_count"] <= 0 or metrics["viable_vertex_count"] <= 0
            or metrics["frontier_segment_count"] != len(lines)):
        raise ValueError("Native frontier relies on missing support or a hidden scalar guard")
    return {"passed": True, "goal_vertex_count": len(vertices), "frontier_segment_count": len(lines),
            "goal_width_m": float(np.ptp(vertices[:, :2] @ tangent)),
            "goal_height_m": float(np.ptp(vertices[:, 2])),
            "frontier_height_span_m": float(np.ptp(frontier[:, 2])),
            "base_z_offset_m": metrics["base_z_offset_m"],
            "minimum_base_overlap_m": metrics["minimum_base_overlap_m"],
            "maximum_base_overlap_m": metrics["maximum_base_overlap_m"],
            "heat_seed_level_m": metrics["heat_seed_level_m"], "heat_t_coef": metrics["heat_t_coef"],
            "field_guard_row_below_base_m": 0}


def audit_batch(config, proposal, result):
    """Independent product, exact-pose, YAML units and field-intersection checks."""
    import numpy as np
    manifest_path = Path(result["batch_manifest_path"])
    if checksum(manifest_path) != result["batch_manifest_sha256"]:
        raise ValueError("Batch receipt changed")
    batch, envelope = read_json(manifest_path), read_json(proposal["proposal_path"])
    planning = envelope["planning"]
    native_initialization = audit_initialization(planning)
    if (batch.get("schema") != "behav3d.initial_wall_batch.v1"
            or batch.get("selected") != planning["selected"]
            or batch.get("proposal_id") != planning["proposal_id"]
            or batch.get("frame_id") != config["request"]["frame_id"]
            or batch.get("requires_review") is not True
            or batch.get("fabrication_authorized") is not False
            or result["proposal_sha256"] != proposal["proposal_sha256"]
            or set(batch.get("files", {})) != {"targets.yaml", "goal_mesh.ply", "field_state_init.npz"}):
        raise ValueError("Batch does not match the exact reviewed pose/scene")
    for name in ("scene_id", "scene_sha256", "geometry_sha256", "task_sha256"):
        if batch.get(name) != planning.get(name):
            raise ValueError("Batch scene/geometry/task binding changed")
    for name, row in batch["files"].items():
        path = canonical(row["path"])
        if path != manifest_path.parent / name or checksum(path) != row["sha256"]:
            raise ValueError("Generated product identity changed")
    points = np.asarray(batch["candidate_points_m"], dtype=float)
    count, requested = batch["candidate_count"], batch["requested_count"]
    if (type(count) is not int or type(requested) is not int or not 0 < count <= requested
            or points.shape != (count, 3) or not np.isfinite(points).all()
            or batch["partial_batch"] != (count < requested)):
        raise ValueError("Invalid first-batch count/positions")
    text = Path(batch["files"]["targets.yaml"]["path"]).read_text()
    rows = re.findall(r'O\(([^,]+),([^,]+),([^\)]+)\)', text)
    written = np.asarray([[float(value) for value in row] for row in rows])
    if written.shape != points.shape or not np.allclose(written / 1000, points, atol=.0000051, rtol=0):
        raise ValueError("YAML millimetre targets differ from the candidate preview")
    if len(re.findall(r"\s+- index:", text)) != count:
        raise ValueError("YAML count differs from batch manifest")
    with np.load(batch["files"]["field_state_init.npz"]["path"], allow_pickle=False) as field:
        phi, heat = np.asarray(field["phi"]), np.asarray(field["heat_norm"])
        evidence = planning["initialization"]
        if (not np.allclose(field["field_vertices_world"], evidence["goal_vertices_m"], atol=1e-8, rtol=0)
                or not np.array_equal(field["field_faces"], evidence["goal_faces"])
                or not np.allclose(heat, evidence["heat_norm"], atol=1e-8, rtol=0)
                or not np.allclose(phi, evidence["phi_m"], atol=1e-8, rtol=0)):
            raise ValueError("Saved field differs from the exact initialized goal/heat/frontier")
        if (not len(phi) or not np.isfinite(phi).all() or not np.isfinite(heat).all()
                or len(phi) != len(heat) or np.ptp(heat) <= 0):
            raise ValueError("Sequence field is missing, nonfinite or constant")
        iso = planning["task"]["iso_level_m"]
        if not float(phi.min()) <= iso < float(phi.max()):
            raise ValueError("Goal and scanned support have no lower-frontier intersection")
        intersection = {"phi_min_m": float(phi.min()), "phi_max_m": float(phi.max()),
                        "iso_level_m": iso, "heat_min": float(heat.min()), "heat_max": float(heat.max())}
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(batch["files"]["goal_mesh.ply"]["path"])
    if (not np.allclose(np.asarray(mesh.vertices), planning["initialization"]["goal_vertices_m"], atol=1e-6, rtol=0)
            or not np.array_equal(np.asarray(mesh.triangles), planning["initialization"]["goal_faces"])):
        raise ValueError("Exported goal mesh differs from the exact initialized goal")
    return {"passed": True, "candidate_count": count, "requested_count": requested,
            "native_initialization_audit": native_initialization,
            "partial_batch": batch["partial_batch"], "intersection": intersection,
            "scope": "Native first-candidate preview consistency, not deposition or robot reachability"}


def stdin_decision(stage, checksum_value, events):
    expected = "approve " + checksum_value if stage == "placement" else "accept-preview " + checksum_value
    events.add("AWAITING_HUMAN_" + stage.upper() + "_REVIEW", exact_sha256=checksum_value,
               expected_command=expected, alternatives=["reject", "abort"],
               note="No physical execution exists. EOF aborts; no TTY auto-continue.")
    while True:
        line = sys.stdin.readline()
        if not line:
            return "abort"
        line = line.strip()
        if line == expected:
            return "approve"
        if line in {"reject", "abort"}:
            return line
        events.add("REVIEW_INPUT_REJECTED", reason="Expected exact current hash command, reject or abort")


def run_rehearsal(config, *, software_test=False, decision=None, emit=emit_line,
                  workflow=None, proposal_audit=audit_proposal, batch_audit=audit_batch):
    """One process owns the durable gate until review completes or is aborted."""
    if workflow is None:
        from behav3d_orchestrator.supervised_agent import initial_workflow as workflow
    run = Path(config["run_path"])
    mode = "software_test_reviewer_not_human" if software_test else "human_review"
    events = Events(run / "events.jsonl", mode=mode, emit=emit)
    journal = run / "startup.jsonl"
    supervisor = workflow.StartupSupervisor(journal)
    request, products = config["request"], run / "products"
    summary = {"schema": SCHEMA, "mode": mode, "recorded_evidence_only": True,
               "human_placement_approved": False, "human_batch_preview_accepted": False,
               "software_test_approval_only": software_test, "robot_commands_sent": False,
               "fabrication_authorized": False, "passed": False, "status": "FAILED",
               "hold_handled_safely": False, "end_to_end_preview_validated": False,
               "request": request, "configuration_sha256": checksum(run / "configuration.json")}
    operator = "software-test-reviewer-not-human" if software_test else "human-offline-review"
    def operation(value):
        return workflow.evaluate_initial_wall(value, input_roots=config["input_roots"],
            output_root=str(products), agent007_root=config["agent007_root"],
            behav3d_root=config["behav3d_root"], approval_journal_path=str(journal))
    def must_reject(label, callback):
        try:
            callback()
        except workflow.StartupError:
            events.add(label, passed=True)
            return
        raise ValueError(label + " unexpectedly bypassed approval")
    try:
        events.add("RECORDED_SCENE_READY", scene_manifest_path=request["scene_manifest_path"],
                   scene_manifest_sha256=request["scene_manifest_sha256"],
                   note="Saved perception replay, not a new camera capture or LLM inference")
        if software_test:
            must_reject("DISARMED_REQUEST_BLOCKED", lambda: supervisor.execute(request,
                lambda: (_ for _ in ()).throw(AssertionError("disarmed callback ran"))))
        supervisor.set_armed(True, operator)
        events.add("OBSERVATION_TO_ROBOT_SCENE_HANDOFF", request=request)
        proposal = supervisor.execute(request, lambda: operation(request))
        summary["proposal"] = proposal
        events.add("PLACEMENT_RESULT", result=proposal)
        if generated_products(products):
            raise ValueError("PROPOSE wrote candidate products before any approval")
        summary["no_candidates_before_approval"] = True
        if not proposal["success"]:
            summary.update(status=proposal["status"], passed=False,
                           hold_handled_safely=proposal["status"] == "HOLD")
            return summary
        summary["proposal_audit"] = proposal_audit(config, proposal)
        events.add("EXACT_POSE_AUDIT", **summary["proposal_audit"])
        generate = dict(request, phase="GENERATE", proposal_path=proposal["proposal_path"],
                        proposal_sha256=proposal["proposal_sha256"])
        if software_test:
            must_reject("UNAPPROVED_GENERATION_BLOCKED", lambda: supervisor.execute(generate,
                lambda: (_ for _ in ()).throw(AssertionError("unapproved callback ran"))))
            direct = operation(generate)
            if direct["success"] or direct["status"] != "REJECTED" or generated_products(products):
                raise ValueError("Direct backend bypass was not rejected")
            events.add("DIRECT_BACKEND_BYPASS_BLOCKED", passed=True, result=direct)
            wrong = "f" * 64 if proposal["proposal_sha256"] != "f" * 64 else "e" * 64
            must_reject("WRONG_HASH_APPROVAL_BLOCKED", lambda: supervisor.approve(
                request["request_id"], wrong, operator))
            review = "approve"
        else:
            review = (decision or stdin_decision)("placement", proposal["proposal_sha256"], events)
        if review != "approve":
            events.add("PLACEMENT_REJECTED" if review == "reject" else "REHEARSAL_ABORTED")
            summary.update(status="HUMAN_REJECTED" if review == "reject" else "ABORTED", passed=True)
            return summary
        supervisor.approve(request["request_id"], proposal["proposal_sha256"], operator)
        summary["human_placement_approved"] = not software_test
        events.add("SOFTWARE_TEST_APPROVAL" if software_test else "HUMAN_PLACEMENT_APPROVAL",
                   proposal_sha256=proposal["proposal_sha256"], operator_id=operator,
                   scope="First-candidate preview generation only; not printing")
        result = supervisor.execute(generate, lambda: operation(generate))
        summary["generated"] = result
        events.add("FIRST_BATCH_RESULT", result=result)
        if not result["success"]:
            summary.update(status=result["status"], passed=False,
                           hold_handled_safely=result["status"] == "HOLD")
            return summary
        summary["batch_audit"] = batch_audit(config, proposal, result)
        summary["end_to_end_preview_validated"] = True
        events.add("FIRST_BATCH_AUDIT", **summary["batch_audit"])
        if not software_test:
            review = (decision or stdin_decision)("batch", result["batch_manifest_sha256"], events)
            summary["human_batch_preview_accepted"] = review == "approve"
            events.add("HUMAN_BATCH_PREVIEW_REVIEW", decision=review,
                       scope="Offline assessment only. No print execution follows.")
        summary.update(status="SOFTWARE_TEST_PASSED" if software_test else "OFFLINE_REVIEW_COMPLETE", passed=True)
        return summary
    except Exception as error:  # noqa: BLE001 -- persist failed offline run; never execute hardware
        summary.update(status="FAILED", passed=False, error=str(error))
        events.add("FAILED_CLOSED", error=str(error))
        return summary
    finally:
        supervisor.set_armed(False, operator + "-ended-disarmed")
        summary["final_state"] = supervisor.snapshot()["state"]
        supervisor.close()
        events.add("REHEARSAL_ENDED", status=summary["status"], passed=summary["passed"],
                   final_state=summary["final_state"])
        write_json(run / "summary.json", summary)
        events.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="Bind existing evidence into a new exclusive run")
    for name in ("output", "scene-manifest", "task", "agent007-root", "behav3d-root"):
        prepare.add_argument("--" + name, type=Path, required=True)
    for name in ("frame-id", "request-id", "artifact-id"):
        prepare.add_argument("--" + name, required=True)
    prepare.add_argument("--input-root", type=Path, action="append", required=True)
    automatic = commands.add_parser("automatic", help="Labelled software-test review, never human approval")
    automatic.add_argument("--run", type=Path, required=True)
    automatic.add_argument("--software-test-reviewer", action="store_true", required=True)
    interactive = commands.add_parser("interactive", help="Wait for exact approval text on stdin; EOF aborts")
    interactive.add_argument("--run", type=Path, required=True)
    interactive.add_argument("--human-review-stdin", action="store_true", required=True)
    args = parser.parse_args(argv)
    if args.command == "prepare":
        config = prepare_run(output=args.output, scene_manifest=args.scene_manifest, task=args.task,
            frame_id=args.frame_id, request_id=args.request_id, artifact_id=args.artifact_id,
            input_roots=args.input_root, agent007_root=args.agent007_root, behav3d_root=args.behav3d_root)
        print(json.dumps({"status": "PREPARED_NO_APPROVAL", "configuration": str(Path(config["run_path"]) / "configuration.json")}))
        return 0
    config = load_run(args.run)
    root, agent = Path(config["behav3d_root"]), Path(config["agent007_root"])
    sys.path[:0] = [str(root / "src/behav3d_orchestrator"), str(agent / "src"), str(agent)]
    summary = run_rehearsal(config, software_test=args.command == "automatic")
    print(json.dumps(summary, indent=2, allow_nan=False))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
