"""Persist attempted beads and command-confirmed bead volumes by cycle."""
from contextlib import contextmanager
from copy import deepcopy
import math
from pathlib import Path
import threading

import yaml


def pose_record(target):
    pose = target.pose
    return dict(frame_id=target.header.frame_id,
                position_m=[float(getattr(pose.position, a)) for a in "xyz"],
                orientation_xyzw=[float(getattr(pose.orientation, a)) for a in "xyzw"])


class BeadExecutionLogger:
    """Small atomic session report; never interpret missing replies as zero flow."""

    def __init__(self, session_root):
        self.path = Path(session_root).expanduser().resolve() / "print_manifest.yaml"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()

    def update(self, cycle, attempt_id, record):
        with self.lock:
            data = yaml.safe_load(self.path.read_text(encoding="utf-8")) if self.path.exists() else {
                "schema_version": 1,
                "volume_basis": "Completed PrintSteps responses divided by steps_per_mm3; not a physical flow measurement. Unknown responses remain null.",
                "cycles": {},
            }
            bucket = data["cycles"].setdefault(str(cycle), {"attempted_beads": [], "successful_beads": []})
            attempts = bucket["attempted_beads"]
            if attempt_id is None:
                attempt_id = len(attempts)
                attempts.append(deepcopy(record))
            else:
                attempts[attempt_id] = deepcopy(record)
            bucket["successful_beads"] = [deepcopy(row) for row in attempts if row["status"] == "success"]
            pending = self.path.with_suffix(".yaml.pending")
            pending.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
            pending.replace(self.path)
            return attempt_id

    def next_cycle_number(self, prefix):
        if not self.path.exists():
            return 1
        data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        numbers = [int(key.rsplit("cycle_", 1)[1]) for key in data["cycles"]
                   if key.startswith(prefix + "/cycle_") and key.rsplit("cycle_", 1)[1].isdigit()]
        return max(numbers, default=0) + 1


class BeadAttempt:
    def __init__(self, logger, cycle, *, index, target, nominal_steps, material_steps,
                 retract_steps, steps_per_mm3, start=None):
        calibration = float(steps_per_mm3)
        if not math.isfinite(calibration) or calibration <= 0:
            raise ValueError("steps_per_mm3 must be finite and positive")
        self.logger, self.cycle, self.attempt_id = logger, cycle, None
        self.calibration, self.retract_steps = calibration, retract_steps
        self.started = False
        self.forward_result = self.retract_result = None
        self.motion_ok = True
        self.record = dict(index=int(index), target=pose_record(target),
                           nominal_volume_mm3=int(nominal_steps) / calibration,
                           requested_volume_mm3=int(material_steps) / calibration,
                           commanded_forward_steps=int(material_steps) + int(retract_steps),
                           volume_mm3=0.0, extruded=False, status="attempted")
        if start is not None:
            self.record["start"] = pose_record(start)
        self.persist()

    def persist(self):
        if self.logger is not None:
            self.attempt_id = self.logger.update(self.cycle, self.attempt_id, self.record)

    def forward_started(self):
        self.started = True
        self.record.update(volume_mm3=None, extruded=None, status="unknown")
        self.persist()

    def observe(self, result, *, reverse=False, motion=False):
        if motion:
            self.motion_ok = bool(result.get("ok", False))
        elif reverse:
            self.retract_result = result
        else:
            self.forward_result = result
        if not result.get("ok", False):
            self.record["error"] = str(result.get("error") or result.get("stage") or "command failed")

    def finish(self, error=None):
        if error is not None:
            self.record["error"] = str(error)
        forward = self.forward_result or {}
        reverse = self.retract_result or {}
        accepted = forward.get("metrics", {}).get("accepted_steps")
        retracted = reverse.get("metrics", {}).get("accepted_steps") if self.retract_steps else 0
        if not self.started:
            self.record.update(volume_mm3=0.0, extruded=False, status="not_extruded")
        elif (forward.get("ok", False) and accepted is not None
              and (not self.retract_steps or reverse.get("ok", False) and retracted is not None)):
            net = max(0, int(accepted) - int(retracted))
            outcome = "motion_unknown" if self.motion_ok is None else "success" if self.motion_ok else "motion_failed"
            self.record.update(volume_mm3=net / self.calibration, extruded=net > 0,
                               status=outcome if net else "not_extruded")
        else:
            self.record.update(volume_mm3=None, extruded=None, status="unknown")
        self.persist()


@contextmanager
def track_bead(session, **details):
    attempt = BeadAttempt(getattr(session, "bead_logger", None),
                          getattr(session, "bead_cycle", "unassigned"), **details)
    session._active_bead = attempt
    try:
        yield attempt
    except BaseException as exc:
        attempt.finish(exc)
        raise
    else:
        attempt.finish()
    finally:
        session._active_bead = None
