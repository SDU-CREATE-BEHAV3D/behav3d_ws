"""Opt-in local geometry analysis of a strict, frozen RGB-D submission.

This module does not grant robot authority or provide an operating-system
sandbox. Only operator-configured source code is imported, inside the isolated
worker process. Request manifests contain data and hashes, never executable
names, import paths, model settings, or writable output directories.
"""
from __future__ import annotations

import hashlib
import importlib
import json
import os
import stat
import sys
import traceback
import uuid
from contextlib import redirect_stdout
from pathlib import Path

try:
    from .observation import (
        MAX_DEPTH,
        MAX_JSON_BYTES,
        MAX_SOURCE_FILES,
        _Budget,
        _Cancelled,
        _check_cancel,
        _confined,
        _decode,
        _EvidenceError,
        _file,
        _hash,
        _open_regular,
        _roots,
        _text,
        evaluate_recorded_observation,
        result_for_error,
        validate_request,
        validate_result,
    )
except ImportError:  # Direct fixed-script execution by process_runner.
    from observation import (
        MAX_DEPTH,
        MAX_JSON_BYTES,
        MAX_SOURCE_FILES,
        _Budget,
        _Cancelled,
        _check_cancel,
        _confined,
        _decode,
        _EvidenceError,
        _file,
        _hash,
        _open_regular,
        _roots,
        _text,
        evaluate_recorded_observation,
        result_for_error,
        validate_request,
        validate_result,
    )

INPUT_SCHEMA = "agent007.fresh_observation_input.v1"
INPUT_FIELDS = frozenset(("schema", "binding", "files", "previous_job_directory",
                          "previous_manifest_sha256", "adapter_sha256"))
BINDING_FIELDS = frozenset(("artifact_id", "observation_id", "candidate_cycle",
                            "scan_cycle", "previous_evaluation_id", "frame_id",
                            "coordinate_units"))
STATUS_SCHEMA = "behav3d.fresh_analysis_status.v1"
ADAPTER_RELATIVE = Path("src/ocsr/printed_artifacts/ros_fresh_analysis.py")


def _directory(value):
    path = Path(value)
    if not path.is_absolute() or not path.is_dir() or path.is_symlink():
        raise ValueError("Explicit existing non-symlink directory required")
    resolved = path.resolve(strict=True)
    if resolved != path:
        raise ValueError("Canonical directory required")
    return resolved


def _policy(evidence_root, source_roots, agent007_root, dds_root, output_root):
    evidence, sources = _roots(evidence_root, source_roots)
    agent = _directory(agent007_root)
    dds = _directory(dds_root)
    output = _directory(output_root)
    for source in (*sources, agent, dds):
        if output == source or output.is_relative_to(source) or source.is_relative_to(output):
            raise ValueError("Writable output root must be separate from read-only roots")
    adapter = agent / ADAPTER_RELATIVE
    if adapter.resolve(strict=True) != adapter or not adapter.is_file():
        raise ValueError("Configured Agent007 adapter must be a regular canonical file")
    return evidence, sources, agent, dds, output, adapter


def _inventory(folder, cancelled):
    """Enumerate all inputs without following links or accepting special files."""
    inputs = folder / "inputs"
    if not inputs.is_dir() or inputs.is_symlink():
        raise _EvidenceError("Missing regular inputs directory")
    pending, names, entries = [(inputs, 1)], set(), 0
    while pending:
        directory, depth = pending.pop()
        if depth > MAX_DEPTH:
            raise _EvidenceError("Input directory depth exceeds limit")
        _check_cancel(cancelled)
        with os.scandir(directory) as children:
            for child in children:
                _check_cancel(cancelled)
                entries += 1
                if entries > MAX_SOURCE_FILES * 4:
                    raise _EvidenceError("Input inventory exceeds limit")
                value = child.stat(follow_symlinks=False)
                if stat.S_ISDIR(value.st_mode):
                    pending.append((Path(child.path), depth + 1))
                elif stat.S_ISREG(value.st_mode):
                    names.add(Path(child.path).relative_to(folder).as_posix())
                    if len(names) > MAX_SOURCE_FILES:
                        raise _EvidenceError("Too many input files")
                else:
                    raise _EvidenceError("Input links and special files are prohibited")
    return names


def _manifest(request, evidence_root, budget, cancelled):
    path, checksum, raw = _file(request["source_manifest_path"], (evidence_root,),
                                budget, cancelled, collect=True)
    if (path.name != "manifest.json" or path != Path(request["source_manifest_path"])
            or checksum != request["source_manifest_sha256"]):
        raise _EvidenceError("Fresh input manifest identity mismatch")
    document = _decode(raw, MAX_JSON_BYTES)
    if (type(document) is not dict or set(document) != INPUT_FIELDS
            or document["schema"] != INPUT_SCHEMA
            or type(document["binding"]) is not dict
            or set(document["binding"]) != BINDING_FIELDS
            or document["binding"] != {name: request[name] for name in BINDING_FIELDS}):
        raise _EvidenceError("Fresh manifest schema or observation binding mismatch")
    # Equality alone accepts True == 1 and 2.0 == 2; retain exact wire types.
    validate_request(dict(request, **document["binding"]))
    files = document["files"]
    if type(files) is not dict or not 0 < len(files) <= MAX_SOURCE_FILES:
        raise _EvidenceError("Invalid bounded input file inventory")
    for name, expected in files.items():
        _text(name, 4096)
        relative = Path(name)
        if (relative.is_absolute() or relative.as_posix() != name
                or relative.parts[0] != "inputs" or len(relative.parts) < 2
                or any(part in (".", "..") for part in relative.parts)):
            raise _EvidenceError("Invalid relative frozen input path")
        _hash(expected)
    _hash(document["adapter_sha256"])
    previous, previous_hash = document["previous_job_directory"], document["previous_manifest_sha256"]
    if request["previous_evaluation_id"]:
        if not isinstance(previous, str) or not Path(_text(previous, 4096)).is_absolute():
            raise _EvidenceError("Explicit previous generated job required")
        _hash(previous_hash)
    elif previous is not None or previous_hash is not None:
        raise _EvidenceError("Initial evaluation may not claim previous job evidence")
    _verify_inputs(path.parent, files, (evidence_root,), budget, cancelled)
    return path, raw, document


def _verify_inputs(folder, files, roots, budget, cancelled):
    if _inventory(folder, cancelled) != set(files):
        raise _EvidenceError("Frozen input inventory is incomplete or has extra files")
    for name, expected in files.items():
        path = folder / name
        resolved, _root = _confined(path, roots)
        if resolved != path or _file(path, roots, budget, cancelled)[1] != expected:
            raise _EvidenceError("Frozen input identity changed")


def _json_file(path, root, budget, cancelled):
    _path, _checksum, raw = _file(path, (root,), budget, cancelled, collect=True)
    return _decode(raw, MAX_JSON_BYTES)


def _previous(document, request, output, budget, cancelled):
    if not request["previous_evaluation_id"]:
        return None, None
    previous = Path(document["previous_job_directory"])
    if (previous.parent != output or previous.resolve(strict=True) != previous
            or previous.is_symlink() or not previous.is_dir()
            or not previous.name.startswith("fresh-")):
        raise _EvidenceError("Previous job is outside the configured generated-job root")
    marker = _json_file(previous / "analysis_status.json", output, budget, cancelled)
    if (type(marker) is not dict or marker.get("schema") != STATUS_SCHEMA
            or marker.get("status") != "COMPLETE_REVIEW_REQUIRED"
            or marker.get("fabrication_authorized") is not False
            or marker.get("supervisor_acceptance") != "not_recorded"):
        raise _EvidenceError("Previous job has no complete analysis artifact")
    old = _json_file(previous / "manifest.json", output, budget, cancelled)
    if (type(old) is not dict or set(old) != INPUT_FIELDS or old.get("schema") != INPUT_SCHEMA
            or type(old.get("binding")) is not dict or set(old["binding"]) != BINDING_FIELDS):
        raise _EvidenceError("Previous generated job manifest is invalid")
    binding = old["binding"]
    if (binding["artifact_id"] != request["artifact_id"]
            or type(binding["scan_cycle"]) is not int
            or binding["scan_cycle"] != request["scan_cycle"] - 1
            or binding["frame_id"] != request["frame_id"] or binding["coordinate_units"] != "m"):
        raise _EvidenceError("Previous generated observation is not consecutive")
    prior_request = dict(request, **binding)
    prior_request.update(source_manifest_path=str(previous / "evaluation/manifest.json"),
                         source_manifest_sha256=document["previous_manifest_sha256"])
    result = evaluate_recorded_observation(prior_request, output, cancelled=cancelled)
    if not result["success"] or result["evaluation_id"] != request["previous_evaluation_id"]:
        raise _EvidenceError("Previous verified evaluation does not match requested history")
    job = _json_file(previous / "job.json", output, budget, cancelled)
    if (type(job) is not dict or set(job) != {"artifact_id", "job_id", "previous_scan_job_id"}
            or job["artifact_id"] != request["artifact_id"] or job["job_id"] != binding["observation_id"]):
        raise _EvidenceError("Previous job identity is inconsistent")
    return previous, job["job_id"]


def _write_json(path, document):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, allow_nan=False, sort_keys=True, separators=(",", ":"))
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _status(folder, request, status, diagnostic=None):
    """Analysis completion is not a Supervisor receipt or operator approval."""
    document = {"schema": STATUS_SCHEMA, "request_id": request["request_id"],
                "source_manifest_sha256": request["source_manifest_sha256"],
                "status": status, "fabrication_authorized": False,
                "supervisor_acceptance": "not_recorded"}
    if diagnostic is not None:
        document["diagnostic"] = diagnostic
    temporary = folder / (".status-" + uuid.uuid4().hex + ".json")
    _write_json(temporary, document)
    os.replace(temporary, folder / "analysis_status.json")


def _diagnostic(error, stage):
    """Keep bounded local failure locations, not exception messages or locals.

    Backend exceptions may contain raw data or credentials. File/line/function
    and exception type are sufficient to identify the failed implementation
    without copying arbitrary exception text into evidence or ROS results.
    """
    return {"stage": stage, "exception_type": type(error).__name__[:128],
            "frames": [{"file": frame.filename[-512:], "line": frame.lineno,
                        "function": frame.name[:128]}
                       for frame in traceback.extract_tb(error.__traceback__)[-16:]]}


def _copy_inputs(source, destination, files, evidence, budget, cancelled):
    for name, expected in files.items():
        _check_cancel(cancelled)
        path = source / name
        resolved, root = _confined(path, (evidence,))
        if resolved != path:
            raise _EvidenceError("Frozen source path changed")
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        checksum = hashlib.sha256()
        with _open_regular(path, root) as stream, target.open("xb") as output:
            before = os.fstat(stream.fileno())
            total = 0
            while True:
                _check_cancel(cancelled)
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                total += len(chunk)
                if total > before.st_size:
                    raise _EvidenceError("Frozen source grew while copying")
                budget.consume(len(chunk))
                checksum.update(chunk)
                output.write(chunk)
            after = os.fstat(stream.fileno())
            identity = lambda value: (value.st_dev, value.st_ino, value.st_size,
                                      value.st_mtime_ns, value.st_ctime_ns)
            if (total != before.st_size or identity(before) != identity(after)
                    or checksum.hexdigest() != expected):
                raise _EvidenceError("Frozen source changed while copying")
            output.flush()
            os.fsync(output.fileno())


def _analyse(folder, *, agent007_root, dds_root, previous_folder):
    """Import only the configured adapter, inside this isolated worker process."""
    expected = agent007_root / ADAPTER_RELATIVE
    sys.path[:0] = [str(agent007_root / "src"), str(agent007_root)]
    adapter = importlib.import_module("ocsr.printed_artifacts.ros_fresh_analysis")
    if Path(adapter.__file__).resolve(strict=True) != expected:
        raise _EvidenceError("Configured adapter import resolved to another producer")
    return adapter.analyse_prepared_job(folder, dds_root=dds_root, previous_folder=previous_folder)


def evaluate_fresh_observation(request, evidence_root, cancelled=lambda: False, *,
                               source_roots=(), agent007_root, dds_root, output_root):
    """Generate and verify new local geometry evidence, never operate a robot.

    The outer process runner owns the single end-to-end monotonic deadline and
    kills this worker's process group on timeout. Analysis libraries must not
    daemonize or launch detached sessions. All effective inputs are frozen and
    verified before/after generation. Output artifacts are not accepted receipts.
    """
    request_id, folder, diagnostic, stage = "", None, None, "input_validation"
    try:
        if isinstance(request, dict):
            try:
                request_id = _text(request.get("request_id"), empty=True)
            except ValueError:
                pass
        try:
            request = validate_request(request)
        except (ValueError, TypeError, KeyError):
            return result_for_error(request_id, "REJECTED", "Invalid observation request contract.")
        request_id = request["request_id"]
        _check_cancel(cancelled)
        if request["event_type"] != "deposition_readback":
            return result_for_error(request_id, "UNSUPPORTED", "Fresh backend only supports deposition readback.")
        try:
            evidence, _sources, agent, dds, output, adapter = _policy(
                evidence_root, source_roots, agent007_root, dds_root, output_root)
        except (OSError, ValueError, TypeError):
            return result_for_error(request_id, "FAILED", "Invalid fresh-analysis server configuration.")
        budget = _Budget()
        manifest_path, raw, document = _manifest(request, evidence, budget, cancelled)
        adapter_hash = _file(adapter, (agent,), budget, cancelled)[1]
        if adapter_hash != document["adapter_sha256"]:
            raise _EvidenceError("Configured analysis adapter changed since submission")
        previous, previous_id = _previous(document, request, output, budget, cancelled)
        _check_cancel(cancelled)
        folder = output / ("fresh-" + uuid.uuid4().hex)
        folder.mkdir(mode=0o700)
        _status(folder, request, "INCOMPLETE")
        with (folder / "manifest.json").open("xb") as stream:
            stream.write(raw)
        _copy_inputs(manifest_path.parent, folder, document["files"], evidence, budget, cancelled)
        _write_json(folder / "job.json", {"artifact_id": request["artifact_id"],
                                         "job_id": request["observation_id"],
                                         "previous_scan_job_id": previous_id})
        _verify_inputs(folder, document["files"], (output,), budget, cancelled)
        _check_cancel(cancelled)
        stage = "local_geometry_analysis"
        # Geometry libraries may print progress; stdout is reserved for the
        # worker's single exact result document.
        with redirect_stdout(sys.stderr):
            _analyse(folder, agent007_root=agent, dds_root=dds, previous_folder=previous)
        stage = "generated_evidence_verification"
        _check_cancel(cancelled)
        _verify_inputs(folder, document["files"], (output,), budget, cancelled)
        _verify_inputs(manifest_path.parent, document["files"], (evidence,), budget, cancelled)
        if (_file(manifest_path, (evidence,), budget, cancelled)[1] != request["source_manifest_sha256"]
                or _file(folder / "manifest.json", (output,), budget, cancelled)[1] != request["source_manifest_sha256"]
                or _file(adapter, (agent,), budget, cancelled)[1] != adapter_hash):
            raise _EvidenceError("Submission or configured producer changed during analysis")
        generated = folder / "evaluation/manifest.json"
        generated_hash = _file(generated, (output,), budget, cancelled)[1]
        detached = dict(request, source_manifest_path=str(generated), source_manifest_sha256=generated_hash)
        result = evaluate_recorded_observation(detached, output, cancelled=cancelled)
        if not result["success"]:
            raise _EvidenceError("Generated evidence failed independent bundle verification")
        _check_cancel(cancelled)
        _status(folder, request, "COMPLETE_REVIEW_REQUIRED")
        _check_cancel(cancelled)
        result["message"] = "Fresh local RGB-D geometry analysis generated and verified; registration and physical accuracy remain unvalidated. Operator review required."
        return validate_result(result, request_id)
    except _Cancelled:
        result = result_for_error(request_id, "CANCELLED", "Fresh analysis cancelled; no action authorized.")
    except (OSError, ValueError, TypeError, KeyError, RecursionError, OverflowError) as error:
        diagnostic = _diagnostic(error, stage)
        result = result_for_error(request_id, "STALE_SOURCE", "Fresh input integrity, observation binding or generated evidence verification failed.")
    except Exception as error:  # noqa: BLE001 -- untrusted backend boundary must fail closed
        diagnostic = _diagnostic(error, stage)
        result = result_for_error(request_id, "FAILED", "Fresh analysis failed closed.")
    if folder is not None:
        try:
            _status(folder, request, "UNACCEPTED", diagnostic)
        except (OSError, ValueError):
            pass
    return result
