"""Disarmed, human-reviewed initial wall planning. No physical executor.

PROPOSE creates review evidence only. GENERATE writes a first-batch preview
only after an exact persisted-proposal hash approval. Neither phase authorizes
motion or extrusion. Operator labels and the journal are not authentication.
"""

# ruff: noqa: BLE001 -- worker/callback trust boundaries must fail closed
from __future__ import annotations

import fcntl
import hashlib
import importlib
import json
import os
import stat
import sys
import threading
import uuid
from copy import deepcopy
from pathlib import Path

from .observation import (
    MAX_JSON_BYTES,
    _Budget,
    _Cancelled,
    _check_cancel,
    _decode,
    _file,
    _hash,
    _roots,
    _text,
)

REQUEST_FIELDS = (
    "phase",
    "request_id",
    "artifact_id",
    "frame_id",
    "scene_manifest_path",
    "scene_manifest_sha256",
    "task_path",
    "task_sha256",
    "proposal_path",
    "proposal_sha256",
)
RESULT_FIELDS = (
    "success",
    "status",
    "message",
    "request_id",
    "proposal_path",
    "proposal_sha256",
    "batch_manifest_path",
    "batch_manifest_sha256",
    "requires_review",
    "fabrication_authorized",
)
SUCCESS = {
    "PROPOSE": "AWAITING_PLACEMENT_REVIEW",
    "GENERATE": "BATCH_PREVIEW_REVIEW_REQUIRED",
}
ERRORS = frozenset(
    ("REJECTED", "FAILED", "HOLD", "CANCELLED", "TIMED_OUT", "STALE_SOURCE")
)


def digest(value):
    return hashlib.sha256(
        json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()


def validate_request(request):
    if type(request) is not dict or set(request) != set(REQUEST_FIELDS):
        raise ValueError("Exact initial-wall request required")
    result = dict(request)
    if result["phase"] not in SUCCESS:
        raise ValueError("Only PROPOSE and GENERATE are supported")
    for name in ("request_id", "artifact_id", "frame_id"):
        _text(result[name])
    for path_name, hash_name in (
        ("scene_manifest_path", "scene_manifest_sha256"),
        ("task_path", "task_sha256"),
        ("proposal_path", "proposal_sha256"),
    ):
        if path_name == "proposal_path" and result["phase"] == "PROPOSE":
            if result[path_name] != "" or result[hash_name] != "":
                raise ValueError("PROPOSE cannot supply an existing proposal")
            continue
        path = Path(_text(result[path_name], 4096))
        if (
            not path.is_absolute()
            or str(path) != result[path_name]
            or ".." in path.parts
        ):
            raise ValueError("Explicit normalized absolute artifact paths required")
        _hash(result[hash_name])
    if len(json.dumps(result).encode()) > 16384:
        raise ValueError("Initial-wall request exceeds limit")
    return result


def error_result(request_id, status, message):
    if status not in ERRORS:
        raise ValueError("Invalid startup failure status")
    _text(request_id, empty=True)
    _text(message, 1024)
    return {
        "success": False,
        "status": status,
        "message": message,
        "request_id": request_id,
        "proposal_path": "",
        "proposal_sha256": "",
        "batch_manifest_path": "",
        "batch_manifest_sha256": "",
        "requires_review": True,
        "fabrication_authorized": False,
    }


def validate_result(result, request):
    if type(result) is not dict or set(result) != set(RESULT_FIELDS):
        raise ValueError("Exact initial-wall result required")
    if (
        type(result["success"]) is not bool
        or result["request_id"] != request["request_id"]
        or result["requires_review"] is not True
        or result["fabrication_authorized"] is not False
    ):
        raise ValueError("Startup result identity or safety contract failed")
    _text(result["message"], 1024)
    if not result["success"]:
        inspectable_hold = result["status"] == "HOLD" and request["phase"] == "PROPOSE"
        if (
            result["status"] not in ERRORS
            or result["batch_manifest_path"]
            or result["batch_manifest_sha256"]
            or (
                not inspectable_hold
                and (result["proposal_path"] or result["proposal_sha256"])
            )
        ):
            raise ValueError("Failed startup result may not expose usable artifacts")
        if inspectable_hold and (result["proposal_path"] or result["proposal_sha256"]):
            _hash(result["proposal_sha256"])
            if not Path(_text(result["proposal_path"], 4096)).is_absolute():
                raise ValueError("Absolute source-bound HOLD report required")
    else:
        if result["status"] != SUCCESS[request["phase"]]:
            raise ValueError("Unexpected successful startup phase")
        _hash(result["proposal_sha256"])
        if not Path(_text(result["proposal_path"], 4096)).is_absolute():
            raise ValueError("Absolute proposal artifact required")
        if request["phase"] == "GENERATE":
            _hash(result["batch_manifest_sha256"])
            if not Path(_text(result["batch_manifest_path"], 4096)).is_absolute():
                raise ValueError("Absolute batch artifact required")
            if (
                result["proposal_path"] != request["proposal_path"]
                or result["proposal_sha256"] != request["proposal_sha256"]
            ):
                raise ValueError("Generated batch changed the reviewed placement")
        elif result["batch_manifest_path"] or result["batch_manifest_sha256"]:
            raise ValueError("PROPOSE may not return generated field or targets")
    if len(json.dumps(result, allow_nan=False).encode()) > 16384:
        raise ValueError("Initial-wall result exceeds limit")
    return deepcopy(result)


def base_request(request):
    return {
        name: value
        for name, value in request.items()
        if name not in ("phase", "proposal_path", "proposal_sha256")
    }


def generation_authorized(journal_path, request):
    """Read-only check of the coordinator's durable in-flight generation gate.

    This is deployment-local integrity, not authentication or robot authority.
    The backend never accepts a request-selected journal or approval token.
    """
    try:
        path = Path(journal_path)
        if not path.is_absolute() or path.is_symlink():
            return False
        with path.open("rb") as stream:
            raw = stream.read(16 * 1024 * 1024 + 1)
        if len(raw) > 16 * 1024 * 1024 or (raw and not raw.endswith(b"\n")):
            return False
        previous, sequence, latest = "0" * 64, 0, None
        for line in raw.splitlines(keepends=True):
            if not line.endswith(b"\n"):
                return False
            row = _decode(line, 1024 * 1024)
            claimed = row.pop("sha256")
            if (
                row["schema"] != "behav3d.startup_journal.v1"
                or row["sequence"] != sequence + 1
                or row["previous_sha256"] != previous
                or digest(row) != claimed
            ):
                return False
            previous, sequence, latest = claimed, row["sequence"], row["state"]
        if (
            not latest
            or not latest["armed"]
            or latest["active_id"] != request["request_id"]
        ):
            return False
        record = latest["records"][request["request_id"]]
        return (
            record["state"] == "GENERATING"
            and record["binding"] == base_request(request)
            and record["proposal"]["proposal_path"] == request["proposal_path"]
            and record["proposal"]["proposal_sha256"] == request["proposal_sha256"]
        )
    except (OSError, ValueError, TypeError, KeyError):
        return False


def _write(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, separators=(",", ":"), allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _relative(value):
    path = Path(_text(value, 4096))
    if path.is_absolute() or str(path) != value or ".." in path.parts or not path.parts:
        raise ValueError("Confined relative scene inventory required")
    return path


def _verify_batch(folder, planning, request, output, budget, cancelled):
    path, checksum, raw = _file(
        folder / "manifest.json", (output,), budget, cancelled, collect=True
    )
    if path != folder / "manifest.json":
        raise ValueError("Generated batch manifest escaped its exclusive directory")
    document = _decode(raw, MAX_JSON_BYTES)
    if (
        type(document) is not dict
        or document.get("schema") != "behav3d.initial_wall_batch.v1"
        or document.get("status") != "GENERATED_REVIEW_REQUIRED"
        or document.get("requires_review") is not True
        or document.get("fabrication_authorized") is not False
        or document.get("frame_id") != request["frame_id"]
        or document.get("coordinate_units") != "m"
        or document.get("proposal_id") != planning.get("proposal_id")
        or document.get("selected") != planning.get("selected")
        or set(document.get("files", {}))
        != {"field_state_init.npz", "goal_mesh.ply", "targets.yaml"}
    ):
        raise ValueError(
            "Generated batch differs from the reviewed pose or safety contract"
        )
    for name, row in document["files"].items():
        if (
            type(row) is not dict
            or set(row) != {"path", "sha256"}
            or row["path"] != str(folder / name)
        ):
            raise ValueError("Generated batch file reference escaped its directory")
        _hash(row["sha256"])
        actual, file_hash, _raw = _file(folder / name, (output,), budget, cancelled)
        if actual != folder / name or file_hash != row["sha256"]:
            raise ValueError("Generated batch file changed")
    if _file(path, (output,), budget, cancelled)[1] != checksum:
        raise ValueError("Generated batch manifest changed after verification")
    return str(path), checksum


def _producer(behav3d_root):
    """Bind the installed planning implementation and its local package code."""
    here = Path(__file__).resolve().parent.parent
    files = sorted(path for path in here.rglob("*.py") if path.is_file())
    scalar = Path(behav3d_root) / "src/behav3d_py/behav3d_py/scalar_field"
    if (
        not scalar.is_dir()
        or scalar.resolve() != scalar
        or not (scalar / "lib_scalar/__init__.py").is_file()
    ):
        raise ValueError("Configured production scalar kernels are unavailable")
    files.extend(sorted(scalar.rglob("*.py")))
    spec = importlib.util.find_spec("behav3d_utils")
    if spec and spec.submodule_search_locations:
        for root in spec.submodule_search_locations:
            files.extend(sorted(Path(root).rglob("*.py")))
    if not 0 < len(files) <= 4096:
        raise ValueError("Planner code inventory exceeds limits")
    return digest(
        {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    )


def _load_inputs(request, input_roots, agent007_root, cancelled):
    if not isinstance(input_roots, (tuple, list)) or not input_roots:
        raise ValueError("Explicit startup input roots required")
    first, roots = _roots(input_roots[0], input_roots[1:])
    del first
    agent = Path(agent007_root)
    if not agent.is_absolute() or not agent.is_dir() or agent.resolve() != agent:
        raise ValueError("Explicit canonical Agent007 code root required")
    budget = _Budget()
    manifest_path, checksum, raw = _file(
        request["scene_manifest_path"], roots, budget, cancelled, collect=True
    )
    if checksum != request["scene_manifest_sha256"] or manifest_path != Path(
        request["scene_manifest_path"]
    ):
        raise ValueError("Initial scene manifest changed")
    manifest = _decode(raw, MAX_JSON_BYTES)
    if (
        type(manifest) is not dict
        or manifest.get("schema") != "agent007.initial_scene.v1"
        or manifest.get("frame_id") != request["frame_id"]
        or manifest.get("coordinate_units") != "m"
        or manifest.get("registration_status") != "unverified"
    ):
        raise ValueError("Initial scene frame, units or schema mismatch")
    checks = {manifest_path: request["scene_manifest_sha256"]}
    if set(manifest.get("files", {})) != {"scene_packet.json", "geometry.npz"}:
        raise ValueError("Initial scene products incomplete")
    inventory = manifest.get("source_inventory")
    if type(inventory) is not dict or not 0 < len(inventory) <= 4096:
        raise ValueError("Initial scene source inventory incomplete")
    source_root = Path(_text(manifest.get("source_root"), 4096))
    if not source_root.is_absolute():
        raise ValueError("Explicit original scene root required")
    for name, expected in manifest["files"].items():
        checks[manifest_path.parent / _relative(name)] = _hash(expected)
    for name, expected in inventory.items():
        relative = _relative(name)
        checks[manifest_path.parent / "sources" / relative] = _hash(expected)
        checks[source_root / relative] = expected
    for path, expected in checks.items():
        if _file(path, roots, budget, cancelled)[1] != expected:
            raise ValueError("Initial scene sources changed")
    task_path, task_hash, task_raw = _file(
        request["task_path"], roots, budget, cancelled, collect=True
    )
    if task_hash != request["task_sha256"] or task_path != Path(request["task_path"]):
        raise ValueError("Task specification changed")
    task = _decode(task_raw, MAX_JSON_BYTES)
    if type(task) is not dict:
        raise ValueError("Task must be a bounded JSON object")
    checks[task_path] = task_hash
    producer = manifest.get("producer")
    expected_modules = {
        "initial_scene.py",
        "perception_scene.py",
        "scene_validation.py",
        "support_topography.py",
    }
    if (
        type(producer) is not dict
        or set(producer) != {"sha256", "files"}
        or type(producer["files"]) is not dict
        or set(producer["files"]) != expected_modules
        or producer["sha256"] != digest(producer["files"])
    ):
        raise ValueError("Initial scene producer identity is incomplete")
    for name, expected in producer["files"].items():
        _hash(expected)
        path = agent / "src/ocsr" / name
        if _file(path, (agent,), budget, cancelled)[1] != expected:
            raise ValueError("Initial scene producer changed")
        checks[path] = expected
    roots = (*roots, agent)
    sys.path[:0] = [str(agent / "src"), str(agent)]
    module = importlib.import_module("ocsr.initial_scene")
    if Path(module.__file__).resolve() != agent / "src/ocsr/initial_scene.py":
        raise ValueError("Configured scene adapter import mismatch")
    scene = module.load_scene(manifest_path.parent, frame_id=request["frame_id"])
    _check_cancel(cancelled)
    return scene, task, checks, roots, budget


def evaluate_initial_wall(
    request,
    *,
    input_roots,
    output_root,
    agent007_root,
    behav3d_root,
    cancelled=lambda: False,
    approval_journal_path=None,
):
    """Trusted worker API; PROPOSE computes review evidence, not batch products.

    Goal positioning, heat and intersection evidence may be evaluated in memory
    before approval. Field-state, goal-mesh and target batch files are written
    only by the approved GENERATE phase.

    GENERATE also requires the configured coordinator's durable in-flight gate.
    The numerical module is not an authorization boundary; this workflow is.
    """
    request_id, folder = "", None
    try:
        if isinstance(request, dict):
            try:
                request_id = _text(request.get("request_id"), empty=True)
            except ValueError:
                pass
        request = validate_request(request)
        if request["phase"] == "GENERATE":
            if approval_journal_path is None or not generation_authorized(
                approval_journal_path, request
            ):
                return error_result(
                    request_id,
                    "REJECTED",
                    "Durable exact-placement approval required before generation.",
                )
            caller_cancelled = cancelled
            cancelled = lambda: (
                caller_cancelled()
                or not generation_authorized(approval_journal_path, request)
            )
        _check_cancel(cancelled)
        output = Path(output_root)
        if (
            not output.is_absolute()
            or not output.is_dir()
            or output.resolve() != output
        ):
            raise ValueError("Explicit canonical existing startup output root required")
        for root in (*input_roots, agent007_root, behav3d_root):
            root = Path(root).resolve()
            if output.is_relative_to(root) or root.is_relative_to(output):
                raise ValueError("Startup output must be separate from immutable roots")
        scene, task, checks, roots, budget = _load_inputs(
            request, input_roots, agent007_root, cancelled
        )
        producer = _producer(behav3d_root)

        def verify_sources():
            for path, expected in checks.items():
                if _file(path, roots, budget, cancelled)[1] != expected:
                    raise ValueError("Scene or task changed during planning")
            if _producer(behav3d_root) != producer:
                raise ValueError("Planning implementation changed during operation")
            _check_cancel(cancelled)

        scalar_root = Path(behav3d_root) / "src/behav3d_py/behav3d_py/scalar_field"
        sys.path.insert(0, str(scalar_root))
        scalar_module = importlib.import_module("lib_scalar")
        if (
            Path(scalar_module.__file__).resolve()
            != scalar_root / "lib_scalar/__init__.py"
        ):
            raise ValueError("Configured scalar kernel import mismatch")
        from .initial_planning import (
            generate_first_batch,
            propose_wall,
            validate_proposal,
        )

        if request["phase"] == "PROPOSE":
            planning = propose_wall(scene["packet"], scene["geometry"], task)
            if planning.get("status") != "PROPOSED_REVIEW_REQUIRED":
                verify_sources()
                folder = output / ("proposal-" + uuid.uuid4().hex)
                folder.mkdir(mode=0o700)
                _write(
                    folder / "HOLD.json",
                    {
                        "schema": "behav3d.initial_wall_proposal.v1",
                        "status": "HOLD",
                        "planning": planning,
                        "binding": base_request(request),
                        "producer_sha256": producer,
                        "requires_review": True,
                        "fabrication_authorized": False,
                    },
                )
                result = error_result(
                    request_id,
                    "HOLD",
                    "No supported wall placement; inspect source-bound HOLD report.",
                )
                result.update(
                    proposal_path=str(folder / "HOLD.json"),
                    proposal_sha256=_file(
                        folder / "HOLD.json", (output,), budget, cancelled
                    )[1],
                )
                verify_sources()
                return validate_result(result, request)
            if (
                planning.get("requires_review") is not True
                or planning.get("fabrication_authorized") is not False
            ):
                raise ValueError("Planning output must remain review-only")
            envelope = {
                "schema": "behav3d.initial_wall_proposal.v1",
                "binding": base_request(request),
                "producer_sha256": producer,
                "planning": planning,
                "requires_review": True,
                "fabrication_authorized": False,
            }
            folder = output / ("proposal-" + uuid.uuid4().hex)
            folder.mkdir(mode=0o700)
            _write(folder / "proposal.json", envelope)
            proposal_path = folder / "proposal.json"
            proposal_hash = _file(proposal_path, (output,), budget, cancelled)[1]
            batch_path, batch_hash = "", ""
        else:
            proposal_path, proposal_hash, raw = _file(
                request["proposal_path"], (output,), budget, cancelled, collect=True
            )
            if proposal_hash != request["proposal_sha256"] or proposal_path != Path(
                request["proposal_path"]
            ):
                raise ValueError("Reviewed proposal bytes changed")
            envelope = _decode(raw, MAX_JSON_BYTES)
            if (
                type(envelope) is not dict
                or set(envelope)
                != {
                    "schema",
                    "binding",
                    "producer_sha256",
                    "planning",
                    "requires_review",
                    "fabrication_authorized",
                }
                or envelope["schema"] != "behav3d.initial_wall_proposal.v1"
                or envelope["binding"] != base_request(request)
                or envelope["producer_sha256"] != producer
                or envelope["requires_review"] is not True
                or envelope["fabrication_authorized"] is not False
            ):
                raise ValueError("Reviewed scene, task or planning producer changed")
            planning = envelope["planning"]
            verified = validate_proposal(
                scene["packet"], scene["geometry"], task, planning
            )
            if (
                verified.get("valid") is not True
                or verified.get("status") != "VALIDATED_REVIEW_REQUIRED"
            ):
                return error_result(
                    request_id,
                    "HOLD",
                    "Reviewed placement HOLD: "
                    + str(verified.get("reason", "No longer validates"))[:600],
                )
            folder = output / ("batch-" + uuid.uuid4().hex)
            # Numerical API owns its new exclusive output directory.
            _check_cancel(cancelled)
            generated = generate_first_batch(
                scene["packet"], scene["geometry"], task, planning, folder
            )
            if generated.get("status") != "GENERATED_REVIEW_REQUIRED":
                return error_result(
                    request_id,
                    "HOLD",
                    "First batch HOLD: "
                    + str(generated.get("reason", "No viable native candidates"))[:600],
                )
            if (
                generated.get("requires_review") is not True
                or generated.get("fabrication_authorized") is not False
            ):
                raise ValueError("Generated output must remain review-only")
            batch_path, batch_hash = _verify_batch(
                folder, planning, request, output, budget, cancelled
            )
            if _file(proposal_path, (output,), budget, cancelled)[1] != proposal_hash:
                raise ValueError("Reviewed proposal changed during batch generation")
        verify_sources()
        return validate_result(
            {
                "success": True,
                "status": SUCCESS[request["phase"]],
                "message": "Initial-wall review artifact ready; no motion, extrusion or safety authorization.",
                "request_id": request_id,
                "proposal_path": str(proposal_path),
                "proposal_sha256": proposal_hash,
                "batch_manifest_path": batch_path,
                "batch_manifest_sha256": batch_hash,
                "requires_review": True,
                "fabrication_authorized": False,
            },
            request,
        )
    except _Cancelled:
        result = error_result(
            request_id,
            "CANCELLED",
            "Startup planning cancelled; late artifacts unaccepted.",
        )
    except (OSError, ValueError, TypeError, KeyError):
        result = error_result(
            request_id,
            "STALE_SOURCE",
            "Startup source, frame, task, proposal or producer verification failed.",
        )
    except Exception:
        result = error_result(request_id, "FAILED", "Startup planning failed closed.")
    if folder is not None and folder.is_dir():
        try:
            _write(
                folder / "UNACCEPTED.json",
                {"status": result["status"], "fabrication_authorized": False},
            )
        except OSError:
            pass
    return result


class StartupError(ValueError):
    pass


class StartupSupervisor:
    """Single-owner durable startup gate; approvals never survive restart."""

    def __init__(self, path):
        self._lock = threading.RLock()
        self._closed = False
        self._running = False
        self._faulted = False
        self.path = Path(path)
        if not self.path.is_absolute():
            raise StartupError("Explicit journal path required")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fd = os.open(
            self.path, os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600
        )
        try:
            if not stat.S_ISREG(os.fstat(self._fd).st_mode):
                raise StartupError("Regular startup journal required")
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._sequence, self._last = 0, "0" * 64
            self._state = {"armed": False, "active_id": None, "records": {}}
            with os.fdopen(os.dup(self._fd), "rb") as stream:
                raw = stream.read(16 * 1024 * 1024 + 1)
            if len(raw) > 16 * 1024 * 1024 or (raw and not raw.endswith(b"\n")):
                raise StartupError(
                    "Truncated or excessive startup journal; reconciliation required"
                )
            for line in raw.splitlines():
                row = _decode(line, 16 * 1024 * 1024)
                claimed = row.pop("sha256")
                if (
                    set(row)
                    != {"schema", "sequence", "previous_sha256", "event", "state"}
                    or row["schema"] != "behav3d.startup_journal.v1"
                    or row["sequence"] != self._sequence + 1
                    or row["previous_sha256"] != self._last
                    or claimed != digest(row)
                ):
                    raise StartupError("Startup journal integrity failure")
                self._state = row["state"]
                self._sequence, self._last = row["sequence"], claimed
            boot = deepcopy(self._state)
            boot["armed"] = False
            if boot["active_id"]:
                record = boot["records"][boot["active_id"]]
                record.update(state="HOLD", approval=None)
            self._commit("BOOT", boot)
            directory = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except BaseException:
            os.close(self._fd)
            self._closed = True
            raise

    def _ensure(self):
        if self._closed or self._faulted:
            raise StartupError("Startup journal unavailable; HOLD")

    def _commit(self, event, state):
        self._ensure()
        row = {
            "schema": "behav3d.startup_journal.v1",
            "sequence": self._sequence + 1,
            "previous_sha256": self._last,
            "event": event,
            "state": deepcopy(state),
        }
        row["sha256"] = digest(row)
        raw = (
            json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False)
            + "\n"
        ).encode()
        if len(raw) > 1024 * 1024:
            raise StartupError("Startup state limit reached")
        try:
            remaining = memoryview(raw)
            while remaining:
                count = os.write(self._fd, remaining)
                if count <= 0:
                    raise OSError("Short startup journal write")
                remaining = remaining[count:]
            os.fsync(self._fd)
        except OSError as error:
            self._faulted = True
            self._state["armed"] = False
            raise StartupError("Startup journal write failed; HOLD") from error
        self._state, self._sequence, self._last = (
            deepcopy(state),
            row["sequence"],
            row["sha256"],
        )

    def snapshot(self):
        with self._lock:
            self._ensure()
            state = deepcopy(self._state)
            active = state["records"].get(state["active_id"])
            state["state"] = (
                active["state"] if active else "ARMED" if state["armed"] else "DISARMED"
            )
            state.update(
                fabrication_authorized=False, hardware_execution_available=False
            )
            return state

    def set_armed(self, armed, operator_id):
        if type(armed) is not bool:
            raise StartupError("Explicit arm state required")
        _text(operator_id)
        with self._lock:
            self._ensure()
            state = deepcopy(self._state)
            active = state["records"].get(state["active_id"])
            if armed and active and active["state"] == "HOLD":
                raise StartupError("Explicit disarm reconciliation required")
            if not armed and active:
                active.update(
                    state="HOLD" if self._running else "INVALIDATED", approval=None
                )
                if not self._running:
                    state["active_id"] = None
            state["armed"] = armed
            self._commit(
                "ARM:" + operator_id if armed else "DISARM:" + operator_id, state
            )
            return self.snapshot()

    def invalidate(self, reason="cancelled"):
        return self.set_armed(False, reason)

    def approve(self, request_id, proposal_sha256, operator_id, approve=True):
        _text(operator_id)
        _hash(proposal_sha256)
        if type(approve) is not bool:
            raise StartupError("Explicit approval decision required")
        with self._lock:
            self._ensure()
            record = self._state["records"].get(request_id)
            if (
                not record
                or self._state["active_id"] != request_id
                or record["state"] != "AWAITING_PLACEMENT_REVIEW"
                or not self._state["armed"]
                or record["proposal"]["proposal_sha256"] != proposal_sha256
            ):
                raise StartupError(
                    "Only exact current proposed placement can be reviewed"
                )
            if not approve:
                return self.set_armed(False, operator_id)
            state = deepcopy(self._state)
            state["records"][request_id].update(state="APPROVED", approval=operator_id)
            self._commit("APPROVE:" + operator_id, state)
            return self.snapshot()

    def execute(self, request, operation):
        request = validate_request(request)
        request_id, phase = request["request_id"], request["phase"]
        binding = base_request(request)
        with self._lock:
            self._ensure()
            state = deepcopy(self._state)
            record = state["records"].get(request_id)
            if record and record["binding"] != binding:
                self.set_armed(False, "changed startup source/task binding")
                raise StartupError(
                    "Startup request identity conflict invalidated approval"
                )
            if (
                record
                and phase == "PROPOSE"
                and record["proposal"] is not None
                and record["state"] not in {"HOLD", "INVALIDATED"}
            ):
                return deepcopy(record["proposal"])
            if (
                record
                and phase == "GENERATE"
                and record["batch"] is not None
                and record["state"] == "GENERATED"
            ):
                if (
                    request["proposal_path"] != record["proposal"]["proposal_path"]
                    or request["proposal_sha256"]
                    != record["proposal"]["proposal_sha256"]
                ):
                    raise StartupError("Cached placement identity conflict")
                return deepcopy(record["batch"])
            if not state["armed"] or self._running:
                raise StartupError("Startup is disarmed or already running")
            if phase == "PROPOSE":
                if record or state["active_id"] is not None:
                    raise StartupError(
                        "Use a new reviewed startup request after reconciliation"
                    )
                record = {
                    "binding": binding,
                    "state": "PROPOSING",
                    "approval": None,
                    "proposal": None,
                    "batch": None,
                }
                state["records"][request_id] = record
                state["active_id"] = request_id
            elif (
                not record
                or state["active_id"] != request_id
                or record["state"] != "APPROVED"
                or request["proposal_path"] != record["proposal"]["proposal_path"]
                or request["proposal_sha256"] != record["proposal"]["proposal_sha256"]
            ):
                raise StartupError(
                    "Exact placement approval required before first-batch writes"
                )
            record.update(
                state="PROPOSING" if phase == "PROPOSE" else "GENERATING", approval=None
            )
            self._commit("START_" + phase, state)
            self._running = True
        try:
            result = validate_result(operation(), request)
        except Exception:
            result = error_result(
                request_id, "FAILED", "Startup operation failed closed."
            )
        with self._lock:
            self._running = False
            self._ensure()
            state = deepcopy(self._state)
            record = state["records"][request_id]
            if not state["armed"] or record["state"] not in {"PROPOSING", "GENERATING"}:
                result = error_result(
                    request_id,
                    "HOLD",
                    "Late startup result unaccepted after invalidation.",
                )
            # Validated diagnostics are retained for operator reconciliation;
            # they are not an approval and cannot make a HOLD executable.
            record["last_result"] = deepcopy(result)
            if not result["success"]:
                record.update(state="HOLD", approval=None)
                state["armed"] = False
            elif phase == "PROPOSE":
                record.update(
                    state="AWAITING_PLACEMENT_REVIEW", proposal=deepcopy(result)
                )
            else:
                record.update(state="GENERATED", batch=deepcopy(result))
                state["active_id"] = None
            self._commit(
                "COMPLETE_" + phase if result["success"] else "HOLD_" + phase, state
            )
            return result

    def close(self):
        with self._lock:
            if self._running:
                raise StartupError(
                    "Cannot release journal while startup callback is running"
                )
            if not self._closed:
                os.close(self._fd)
                self._closed = True
