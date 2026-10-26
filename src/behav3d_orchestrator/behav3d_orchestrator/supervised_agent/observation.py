"""Bounded recorded Agent007 evidence verification, without ROS or model imports.

The bridge owns request freshness/idempotency and operator approval. This module
only reads an existing immutable bundle; it cannot evaluate a new scan, change a
plan, execute commands, or establish physical safety. Configured roots are
server policy, never request fields. Source roots are additional read-only
allowlists; they do not authorize request-controlled bundle paths.
"""
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat

REQUEST_FIELDS = (
    "request_id", "artifact_id", "observation_id", "event_type",
    "candidate_cycle", "scan_cycle", "previous_evaluation_id", "frame_id",
    "coordinate_units", "source_manifest_path", "source_manifest_sha256",
)
RESULT_FIELDS = (
    "success", "status", "message", "request_id", "evaluation_id", "packet_path",
    "packet_sha256", "evidence_manifest_path", "evidence_manifest_sha256",
    "requires_review", "fabrication_authorized",
)
EVENT_TYPES = frozenset(("deposition_readback", "inspection",
                         "intervention_readback", "recovery_verification"))
STATUSES = frozenset(("REVIEW_REQUIRED", "FAILED", "REJECTED", "CANCELLED",
                      "TIMED_OUT", "UNSUPPORTED", "STALE_SOURCE"))
MAX_REQUEST_BYTES = 16 * 1024
MAX_RESULT_BYTES = 16 * 1024
MAX_JSON_BYTES = 32 * 1024 * 1024
MAX_FILE_BYTES = 512 * 1024 * 1024
MAX_TOTAL_HASH_BYTES = 4 * 1024 * 1024 * 1024
MAX_SOURCE_FILES = 4096
MAX_ITEMS = 10000
MAX_DEPTH = 48
MAX_NODES = 2_000_000
HASH_RE = re.compile(r"[a-f0-9]{64}\Z")
EVALUATION_RE = re.compile(r"evaluation_[a-f0-9]{64}\Z")
BUNDLE_FILES = frozenset(("evaluation.json", "shell.json",
                          "robot_review_packet.json"))
ACTIONS = frozenset(("review_registration", "review_scan_coverage",
                     "review_surface_identity", "inspect_geometry",
                     "inspect_dds_shape_size"))
SUGGESTION_FIELDS = frozenset((
    "suggestion_id", "action", "evidence_ids", "candidate_labels", "reason",
    "requires_operator_review", "executable", "fabrication_authorized",
))
PACKET_FIELDS = frozenset((
    "schema", "artifact_id", "evaluation_id", "scan_job_id", "scan_cycle",
    "candidate_cycle", "previous_evaluation_id", "frame_id", "coordinate_units",
    "summary", "registration_status", "suggestions", "suggestion_count",
    "suggestions_truncated", "limitations", "future_evidence_used",
    "physical_accuracy_validated", "structural_evaluation",
    "fabrication_authorized", "robot_commands_sent",
))


class _EvidenceError(ValueError):
    pass


class _Cancelled(Exception):
    pass


def _text(value, maximum=256, empty=False):
    if not isinstance(value, str) or len(value) > maximum:
        raise ValueError("Expected bounded text")
    if (not value and not empty) or value != value.strip() or any(ord(c) < 32 for c in value):
        raise ValueError("Expected nonempty text without control characters")
    return value


def _hash(value):
    if not isinstance(value, str) or HASH_RE.fullmatch(value) is None:
        raise ValueError("Expected lowercase SHA256")
    return value


def _evaluation_id(value, nullable=False):
    if nullable and value is None:
        return None
    if not isinstance(value, str) or EVALUATION_RE.fullmatch(value) is None:
        raise ValueError("Expected evaluation revision")
    return value


def _cycle(value):
    if type(value) is not int or not 0 <= value <= 2_147_483_647:
        raise ValueError("Expected bounded integer cycle")
    return value


def _finite_tree(value):
    pending = [(value, 0)]
    count = 0
    while pending:
        item, depth = pending.pop()
        count += 1
        if depth > MAX_DEPTH or count > MAX_NODES:
            raise ValueError("JSON structure exceeds limits")
        if isinstance(item, dict):
            if not all(isinstance(key, str) for key in item):
                raise ValueError("JSON object keys must be strings")
            pending.extend((child, depth + 1) for child in item.values())
        elif isinstance(item, list):
            pending.extend((child, depth + 1) for child in item)
        elif type(item) is float:
            if not math.isfinite(item):
                raise ValueError("Nonfinite JSON number")
        elif item is not None and type(item) not in (str, bool, int):
            raise ValueError("Unsupported JSON value")


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _constant(_value):
    raise ValueError("Nonfinite JSON number")


def _integer(value):
    if len(value) > 128:
        raise ValueError("JSON integer exceeds digit limit")
    return int(value)


def _number(value):
    if len(value) > 128:
        raise ValueError("JSON number exceeds digit limit")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Nonfinite JSON number")
    return number


def _decode(raw, maximum):
    if not isinstance(raw, bytes) or len(raw) > maximum:
        raise ValueError("JSON byte limit exceeded")
    try:
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=_pairs,
                           parse_constant=_constant, parse_int=_integer,
                           parse_float=_number)
        _finite_tree(value)
        return value
    except (UnicodeError, RecursionError, OverflowError) as exc:
        raise ValueError("Invalid bounded JSON") from exc


def validate_request(request):
    """Return a detached exact-field request or raise ValueError.

    This does not accept server policy, executable names, source roots, or ROS
    commands. A recorded deposition readback must bind plan N to scan N+1.
    Other recognized events require candidate_cycle=-1 (no deposition plan)
    and validate but the recorded backend refuses them.
    """
    if type(request) is not dict or set(request) != set(REQUEST_FIELDS):
        raise ValueError("Observation request fields do not match the contract")
    result = {key: request[key] for key in REQUEST_FIELDS}
    for key in ("request_id", "artifact_id", "observation_id", "frame_id"):
        _text(result[key])
    if not isinstance(result["event_type"], str) or result["event_type"] not in EVENT_TYPES:
        raise ValueError("Unsupported observation event type")
    if result["coordinate_units"] != "m":
        raise ValueError("Observation coordinates must use metres")
    _cycle(result["scan_cycle"])
    if result["event_type"] == "deposition_readback":
        _cycle(result["candidate_cycle"])
        if result["scan_cycle"] != result["candidate_cycle"] + 1:
            raise ValueError("Recorded plan N must bind scan N+1")
    elif type(result["candidate_cycle"]) is not int or result["candidate_cycle"] != -1:
        raise ValueError("Non-deposition observations require the no-plan cycle sentinel")
    previous = result["previous_evaluation_id"]
    if previous != "":
        _evaluation_id(previous)
    path = _text(result["source_manifest_path"], 4096)
    if not Path(path).is_absolute():
        raise ValueError("Manifest path must be explicit and absolute")
    _hash(result["source_manifest_sha256"])
    if len(json.dumps(result, allow_nan=False).encode("utf-8")) > MAX_REQUEST_BYTES:
        raise ValueError("Observation request exceeds byte limit")
    return result


def request_from_goal(goal):
    """Copy a ROS-shaped goal without importing ROS; reject malformed fields."""
    try:
        request = {key: getattr(goal, key) for key in REQUEST_FIELDS}
    except (AttributeError, TypeError) as exc:
        raise ValueError("Malformed observation goal") from exc
    return validate_request(request)


def decode_request(raw):
    """Decode one bounded JSON request, rejecting duplicate keys and NaN."""
    return validate_request(_decode(raw, MAX_REQUEST_BYTES))


def result_for_error(request_id, status, message):
    """Construct a bounded fail-closed result; never echo exception payloads."""
    _text(request_id, empty=True)
    if status not in STATUSES or status == "REVIEW_REQUIRED":
        raise ValueError("Expected failure status")
    _text(message, 1024)
    return {
        "success": False, "status": status, "message": message,
        "request_id": request_id, "evaluation_id": "", "packet_path": "",
        "packet_sha256": "", "evidence_manifest_path": "",
        "evidence_manifest_sha256": "", "requires_review": True,
        "fabrication_authorized": False,
    }


def validate_result(result, request_id):
    """Validate worker output before a ROS bridge exposes it to orchestration."""
    if type(result) is not dict or set(result) != set(RESULT_FIELDS):
        raise ValueError("Observation result fields do not match the contract")
    _text(request_id, empty=True)
    if result["request_id"] != request_id or not isinstance(result["status"], str) or result["status"] not in STATUSES:
        raise ValueError("Observation result identity or status mismatch")
    _text(result["message"], 1024)
    if type(result["success"]) is not bool or result["fabrication_authorized"] is not False:
        raise ValueError("Observation result must not authorize fabrication")
    if result["requires_review"] is not True:
        raise ValueError("Recorded results always require review")
    evidence_fields = ("evaluation_id", "packet_path", "packet_sha256",
                       "evidence_manifest_path", "evidence_manifest_sha256")
    if result["success"]:
        if result["status"] != "REVIEW_REQUIRED":
            raise ValueError("Recorded success must require review")
        _evaluation_id(result["evaluation_id"])
        for key in ("packet_sha256", "evidence_manifest_sha256"):
            _hash(result[key])
        for key in ("packet_path", "evidence_manifest_path"):
            if not Path(_text(result[key], 4096)).is_absolute():
                raise ValueError("Result evidence path must be absolute")
    elif result["status"] == "REVIEW_REQUIRED" or any(result[key] != "" for key in evidence_fields):
        raise ValueError("Failed result must not publish evidence")
    _finite_tree(result)
    if len(json.dumps(result, allow_nan=False).encode("utf-8")) > MAX_RESULT_BYTES:
        raise ValueError("Observation result exceeds byte limit")
    return dict(result)


def _check_cancel(cancelled):
    if cancelled():
        raise _Cancelled()


def _roots(evidence_root, source_roots):
    root = Path(evidence_root)
    if not root.is_absolute():
        raise ValueError("Evidence root must be absolute")
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError("Evidence root must be a directory")
    if not isinstance(source_roots, (tuple, list)) or len(source_roots) > 32:
        raise ValueError("Expected bounded configured source roots")
    extras = []
    for value in source_roots:
        extra = Path(value)
        if not extra.is_absolute():
            raise ValueError("Source roots must be absolute")
        extra = extra.resolve(strict=True)
        if not extra.is_dir():
            raise ValueError("Source roots must be directories")
        extras.append(extra)
    return root, tuple(dict.fromkeys((root, *extras)))


def _confined(path, roots):
    value = Path(_text(str(path), 4096))
    if not value.is_absolute():
        raise _EvidenceError("Evidence paths must be explicit and absolute")
    resolved = value.resolve(strict=True)
    for root in roots:
        try:
            resolved.relative_to(root)
            if resolved == root:
                raise _EvidenceError("Expected a regular evidence file")
            return resolved, root
        except ValueError:
            continue
    raise _EvidenceError("Evidence path is outside configured roots")


@contextmanager
def _open_regular(path, root):
    """Traverse canonical descendants with no-follow directory descriptors.

    Resolving an allowlisted path alone is insufficient: a directory could
    otherwise be replaced by a symlink between confinement and opening.
    """
    if not hasattr(os, "O_NOFOLLOW") or not hasattr(os, "O_DIRECTORY"):
        raise _EvidenceError("This worker requires no-follow filesystem support")
    directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    directory = os.open(str(root), directory_flags)
    opened = None
    try:
        parts = path.relative_to(root).parts
        for part in parts[:-1]:
            child = os.open(part, directory_flags, dir_fd=directory)
            os.close(directory)
            directory = child
        opened = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK,
                         dir_fd=directory)
        if not stat.S_ISREG(os.fstat(opened).st_mode):
            raise _EvidenceError("Expected a regular evidence file")
        with os.fdopen(opened, "rb") as stream:
            opened = None
            yield stream
    finally:
        if opened is not None:
            os.close(opened)
        os.close(directory)


class _Budget:
    def __init__(self):
        self.remaining = MAX_TOTAL_HASH_BYTES

    def consume(self, count):
        self.remaining -= count
        if self.remaining < 0:
            raise _EvidenceError("Total evidence byte limit exceeded")


def _file(path, roots, budget, cancelled, collect=False):
    _check_cancel(cancelled)
    path, root = _confined(path, roots)
    maximum = MAX_JSON_BYTES if collect else MAX_FILE_BYTES
    checksum = hashlib.sha256()
    contents = []
    total = 0
    with _open_regular(path, root) as stream:
        before = os.fstat(stream.fileno())
        if before.st_size > maximum:
            raise _EvidenceError("Evidence file exceeds byte limit")
        while True:
            _check_cancel(cancelled)
            chunk = stream.read(min(1024 * 1024, maximum - total + 1))
            if not chunk:
                break
            total += len(chunk)
            budget.consume(len(chunk))
            if total > maximum:
                raise _EvidenceError("Evidence file exceeds byte limit")
            checksum.update(chunk)
            if collect:
                contents.append(chunk)
        after = os.fstat(stream.fileno())
        identity = lambda entry: (entry.st_dev, entry.st_ino, entry.st_size,
                                  entry.st_mtime_ns, entry.st_ctime_ns)
        if identity(before) != identity(after) or total != before.st_size:
            raise _EvidenceError("Evidence changed while being read")
    _check_cancel(cancelled)
    return path, checksum.hexdigest(), b"".join(contents) if collect else None


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode("utf-8")).hexdigest()


def _false_flags(document, names):
    if any(document.get(name) is not False for name in names):
        raise _EvidenceError("Recorded evidence has unsupported safety claims")


def _validated_bundle(documents, request):
    evaluation = documents["evaluation.json"]
    shell = documents["shell.json"]
    packet = documents["robot_review_packet.json"]
    if not all(type(value) is dict for value in (evaluation, shell, packet)):
        raise _EvidenceError("Invalid bundle documents")
    if evaluation.get("schema") != "agent007.integrated_evaluation.v1":
        raise _EvidenceError("Unsupported evaluation schema")
    if shell.get("schema") != "ocsr.planned_observed_shell.v1":
        raise _EvidenceError("Unsupported shell schema")
    if packet.get("schema") != "agent007.robot_review_packet.v1" or set(packet) != PACKET_FIELDS:
        raise _EvidenceError("Unsupported packet schema")
    identity = _evaluation_id(evaluation.get("evaluation_id"))
    if identity != "evaluation_" + _digest({k: v for k, v in evaluation.items() if k != "evaluation_id"}):
        raise _EvidenceError("Evaluation revision hash mismatch")
    for key, expected in (
        ("artifact_id", request["artifact_id"]),
        ("scan_job_id", request["observation_id"]),
        ("scan_cycle", request["scan_cycle"]),
        ("candidate_cycle", request["candidate_cycle"]),
        ("previous_evaluation_id", request["previous_evaluation_id"] or None),
    ):
        if evaluation.get(key) != expected or packet.get(key) != expected:
            raise _EvidenceError("Recorded evidence does not match requested observation")
    _cycle(evaluation["scan_cycle"])
    _cycle(evaluation["candidate_cycle"])
    _cycle(packet["scan_cycle"])
    _cycle(packet["candidate_cycle"])
    _cycle(shell.get("scan_cycle"))
    _cycle(shell.get("candidate_through_cycle"))
    if type(packet["suggestion_count"]) is not int or type(packet["suggestions_truncated"]) is not bool:
        raise _EvidenceError("Invalid packet count types")
    _evaluation_id(evaluation["previous_evaluation_id"], nullable=True)
    context = evaluation.get("context")
    if type(context) is not dict or context.get("frame_id") != request["frame_id"]:
        raise _EvidenceError("Evaluation frame does not match request")
    _hash(context.get("geometry_profile_sha256"))
    if packet["frame_id"] != request["frame_id"] or packet["coordinate_units"] != "m":
        raise _EvidenceError("Packet frame or units do not match request")
    if packet["evaluation_id"] != identity or packet["registration_status"] != "unverified":
        raise _EvidenceError("Unsupported packet identity or registration claim")
    _false_flags(evaluation, ("future_evidence_used", "physical_accuracy_validated",
                              "robot_commands_sent", "fabrication_authorized"))
    _false_flags(shell, ("future_evidence_used", "physical_accuracy_validated",
                         "robot_commands_sent", "expected_reference_uses_observations",
                         "observed_middle_is_measured_volume_centre"))
    _false_flags(packet, ("future_evidence_used", "physical_accuracy_validated",
                          "robot_commands_sent", "fabrication_authorized"))
    if packet["structural_evaluation"] != "not_performed":
        raise _EvidenceError("Unsupported structural claim")
    if evaluation.get("shell") != shell or shell.get("scan_cycle") != request["scan_cycle"] or \
            shell.get("candidate_through_cycle") != request["candidate_cycle"]:
        raise _EvidenceError("Shell products disagree")
    candidates = evaluation.get("candidates")
    if type(candidates) is not list or not 0 < len(candidates) <= MAX_ITEMS:
        raise _EvidenceError("Invalid bounded candidate records")
    by_label, by_evidence = {}, {}
    for row in candidates:
        if type(row) is not dict:
            raise _EvidenceError("Invalid candidate record")
        label = _text(row.get("label"))
        evidence_id = _text(row.get("evidence_id"))
        if evidence_id != "candidate:" + label or label in by_label or evidence_id in by_evidence:
            raise _EvidenceError("Duplicate or inconsistent candidate evidence")
        _cycle(row.get("candidate_cycle"))
        _cycle(row.get("observation_cycle"))
        if row["observation_cycle"] != row["candidate_cycle"] + 1 or \
                row["observation_cycle"] > request["scan_cycle"]:
            raise _EvidenceError("Candidate cycle binding is invalid")
        if row.get("physical_defect_confirmed") is not False:
            raise _EvidenceError("Unsupported confirmed-defect claim")
        by_label[label], by_evidence[evidence_id] = row, row
    summary = evaluation.get("summary")
    current = [r for r in candidates if r["observation_cycle"] == request["scan_cycle"]]
    if type(summary) is not dict or type(summary.get("current_candidate_count")) is not int or \
            type(summary.get("accumulated_candidate_count")) is not int or \
            summary["current_candidate_count"] != len(current) or \
            summary["accumulated_candidate_count"] != len(candidates):
        raise _EvidenceError("Candidate summary does not match evidence")
    suggestions = evaluation.get("suggestions")
    if type(suggestions) is not list or not 0 < len(suggestions) <= MAX_ITEMS + 1:
        raise _EvidenceError("Invalid bounded suggestions")
    seen = set()
    for suggestion in suggestions:
        if type(suggestion) is not dict or set(suggestion) != SUGGESTION_FIELDS:
            raise _EvidenceError("Invalid suggestion contract")
        suggestion_id = _text(suggestion["suggestion_id"])
        if suggestion_id in seen or suggestion["action"] not in ACTIONS:
            raise _EvidenceError("Duplicate or unsupported suggestion")
        seen.add(suggestion_id)
        _text(suggestion["reason"], 1024)
        if suggestion["requires_operator_review"] is not True:
            raise _EvidenceError("Suggestions must require review")
        _false_flags(suggestion, ("executable", "fabrication_authorized"))
        references, labels = suggestion["evidence_ids"], suggestion["candidate_labels"]
        if type(references) is not list or type(labels) is not list or \
                not 0 < len(references) <= 100 or len(labels) > 100:
            raise _EvidenceError("Invalid bounded suggestion references")
        if not all(isinstance(value, str) for value in (*references, *labels)) or \
                len(set(references)) != len(references) or len(set(labels)) != len(labels):
            raise _EvidenceError("Duplicate or invalid evidence references")
        if suggestion["action"] == "review_registration":
            if references != ["context:registration"] or labels:
                raise _EvidenceError("Invalid registration reference")
        else:
            if not labels or set(references) != {"candidate:" + label for label in labels}:
                raise _EvidenceError("Suggestion references do not match candidate labels")
            if any(label not in by_label or by_label[label]["observation_cycle"] != request["scan_cycle"]
                   for label in labels):
                raise _EvidenceError("Suggestion references unknown or historical candidate")
    limitations = evaluation.get("limitations")
    if type(limitations) is not list or not 0 < len(limitations) <= 100:
        raise _EvidenceError("Missing bounded limitations")
    for limitation in limitations:
        _text(limitation, 2048)
    expected = {
        "schema": "agent007.robot_review_packet.v1",
        "artifact_id": evaluation["artifact_id"], "evaluation_id": identity,
        "scan_job_id": evaluation["scan_job_id"], "scan_cycle": evaluation["scan_cycle"],
        "candidate_cycle": evaluation["candidate_cycle"],
        "previous_evaluation_id": evaluation["previous_evaluation_id"],
        "frame_id": context["frame_id"], "coordinate_units": "m", "summary": summary,
        "registration_status": "unverified", "suggestions": suggestions[:100],
        "suggestion_count": len(suggestions), "suggestions_truncated": len(suggestions) > 100,
        "limitations": limitations, "future_evidence_used": False,
        "physical_accuracy_validated": False, "structural_evaluation": "not_performed",
        "fabrication_authorized": False, "robot_commands_sent": False,
    }
    if packet != expected:
        raise _EvidenceError("Packet does not reproduce evaluation")
    ledger = evaluation.get("ledger")
    if type(ledger) is not list or not 0 < len(ledger) <= MAX_ITEMS:
        raise _EvidenceError("Missing bounded evaluation history")
    first_scan = _cycle(evaluation.get("history_start_scan"))
    if first_scan < 1 or first_scan + len(ledger) - 1 != request["scan_cycle"]:
        raise _EvidenceError("Invalid evaluation history range")
    for index, row in enumerate(ledger):
        if type(row) is not dict or type(row.get("scan_cycle")) is not int or \
                row["scan_cycle"] != first_scan + index:
            raise _EvidenceError("Invalid evaluation history linkage")
        _text(row.get("scan_job_id"))
    if ledger[-1]["scan_job_id"] != request["observation_id"]:
        raise _EvidenceError("History head observation mismatch")
    return evaluation, [evaluation.get("source_sha256"),
                        *(entry.get("source_sha256") for entry in ledger)]


def _source_files(maps, roots, cancelled):
    expected = {}
    references = 0
    for sources in maps:
        _check_cancel(cancelled)
        if type(sources) is not dict or not 0 < len(sources) <= MAX_SOURCE_FILES:
            raise _EvidenceError("Missing or excessive source identities")
        for name, checksum in sources.items():
            _check_cancel(cancelled)
            references += 1
            if references > 65536:
                raise _EvidenceError("Source reference limit exceeded")
            _hash(checksum)
            path, _root = _confined(name, roots)
            if path in expected and expected[path] != checksum:
                raise _EvidenceError("Conflicting source identities")
            expected[path] = checksum
            if len(expected) > MAX_SOURCE_FILES:
                raise _EvidenceError("Source count limit exceeded")
    return expected


def evaluate_recorded_observation(request, evidence_root, cancelled=lambda: False, *, source_roots=()):
    """Read and verify one recorded bundle, never evaluate a new physical scan.

    Request paths are confined to evidence_root; historical/current source files
    may also use explicitly configured source_roots. All unique ledger/current
    source hashes and every bundle file are verified before and after reading.
    This API has no persistent freshness ledger: its caller owns the expected
    previous revision and request-id deduplication.
    """
    request_id = ""
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
            return result_for_error(request_id, "UNSUPPORTED",
                                    "Recorded backend only verifies deposition readback; no new evaluation was produced.")
        try:
            root, source_allowlist = _roots(evidence_root, source_roots)
        except (OSError, ValueError, TypeError):
            return result_for_error(request_id, "FAILED", "Invalid server evidence-root configuration.")
        budget = _Budget()
        manifest_path, manifest_hash, raw = _file(
            request["source_manifest_path"], (root,), budget, cancelled, collect=True)
        if manifest_path.name != "manifest.json" or manifest_hash != request["source_manifest_sha256"]:
            raise _EvidenceError("Evidence manifest identity mismatch")
        manifest = _decode(raw, MAX_JSON_BYTES)
        if type(manifest) is not dict or set(manifest) != {"schema", "files"} or \
                manifest["schema"] != "agent007.evaluation_bundle.v1" or \
                type(manifest["files"]) is not dict or set(manifest["files"]) != BUNDLE_FILES:
            raise _EvidenceError("Invalid confined bundle manifest")
        documents, bundle_paths = {}, {}
        for name, expected in manifest["files"].items():
            _hash(expected)
            child = manifest_path.parent / name
            if child.is_symlink():
                raise _EvidenceError("Bundle children must not be symbolic links")
            path, checksum, raw = _file(child, (root,), budget, cancelled, collect=True)
            if path.parent != manifest_path.parent or path.name != name:
                raise _EvidenceError("Bundle child escaped its manifest directory")
            if checksum != expected:
                raise _EvidenceError("Bundle file hash mismatch")
            documents[name], bundle_paths[name] = _decode(raw, MAX_JSON_BYTES), path
        evaluation, source_maps = _validated_bundle(documents, request)
        expected_sources = _source_files(source_maps, source_allowlist, cancelled)
        for _pass in range(2):
            for path, expected in expected_sources.items():
                if _file(path, source_allowlist, budget, cancelled)[1] != expected:
                    raise _EvidenceError("Recorded source changed")
        for name, path in bundle_paths.items():
            if _file(path, (root,), budget, cancelled)[1] != manifest["files"][name]:
                raise _EvidenceError("Bundle changed after verification")
        if _file(manifest_path, (root,), budget, cancelled)[1] != manifest_hash:
            raise _EvidenceError("Manifest changed after verification")
        _check_cancel(cancelled)
        result = {
            "success": True, "status": "REVIEW_REQUIRED",
            "message": "Recorded evidence verified; physical accuracy and registration remain unvalidated. Operator review required.",
            "request_id": request_id, "evaluation_id": evaluation["evaluation_id"],
            "packet_path": str(bundle_paths["robot_review_packet.json"]),
            "packet_sha256": manifest["files"]["robot_review_packet.json"],
            "evidence_manifest_path": str(manifest_path),
            "evidence_manifest_sha256": manifest_hash,
            "requires_review": True, "fabrication_authorized": False,
        }
        return validate_result(result, request_id)
    except _Cancelled:
        return result_for_error(request_id, "CANCELLED", "Observation verification cancelled; no action authorized.")
    except (OSError, ValueError, TypeError, KeyError, RecursionError, OverflowError):
        return result_for_error(request_id, "STALE_SOURCE",
                                "Evidence integrity, source confinement, safety flags or observation binding failed.")
    except Exception:
        return result_for_error(request_id, "FAILED", "Observation worker failed closed.")
