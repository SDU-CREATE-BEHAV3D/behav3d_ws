#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import re
import threading
import time
from pathlib import Path
from typing import Optional

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointCurrentLogger:
    """Record UR joint currents and print events for one print invocation."""

    def __init__(
        self,
        node,
        *,
        topic: str = "/joint_states",
        max_sample_rate_hz: float = 100.0,
    ) -> None:
        self._node = node
        self._topic = str(topic)
        self._max_sample_rate_hz = max(1.0, float(max_sample_rate_hz))
        self._sample_period_s = 1.0 / self._max_sample_rate_hz
        self._lock = threading.Lock()
        self._active = False
        self._warned_missing_effort = False
        self._start_monotonic = 0.0
        self._output_dir: Optional[Path] = None
        self._current_file = None
        self._current_writer = None
        self._event_file = None
        self._event_writer = None
        self._joint_names: list[str] = []
        self._extruder_on = False
        self._item_kind = ""
        self._item_index = -1
        self._sample_count = 0
        self._next_sample_monotonic = 0.0
        self._subscription = node.create_subscription(
            JointState,
            self._topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._active)

    def start(self, output_dir: str | Path, *, label: str = "print") -> Path:
        path = Path(output_dir).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=True)

        with self._lock:
            if self._active:
                raise RuntimeError("Joint current logger is already active.")

            self._output_dir = path
            self._start_monotonic = time.monotonic()
            self._joint_names = []
            self._extruder_on = False
            self._item_kind = ""
            self._item_index = -1
            self._sample_count = 0
            self._next_sample_monotonic = 0.0
            self._warned_missing_effort = False

            self._current_file = (path / "joint_currents.csv").open(
                "w", newline="", encoding="utf-8"
            )
            self._event_file = (path / "print_events.csv").open(
                "w", newline="", encoding="utf-8"
            )
            self._event_writer = csv.writer(self._event_file)
            self._event_writer.writerow(
                ["ros_time_s", "elapsed_s", "event", "extruder_on", "item_kind", "item_index"]
            )
            self._active = True
            self._write_event_locked("session_start", label=label)
            self._event_file.flush()

        self._node.get_logger().info(
            f"[joint_current_logger] Recording '{self._topic}' in '{path}' "
            f"at up to {self._max_sample_rate_hz:.1f} Hz."
        )
        return path

    def mark_event(
        self,
        event: str,
        *,
        extruder_on: Optional[bool] = None,
        item_kind: Optional[str] = None,
        item_index: Optional[int] = None,
    ) -> None:
        with self._lock:
            if not self._active:
                return
            if extruder_on is not None:
                state_changed = bool(extruder_on) != self._extruder_on
                self._extruder_on = bool(extruder_on)
                if state_changed:
                    # Allow the next message through immediately so the sampled
                    # stream captures the boundary with at most one ROS-message delay.
                    self._next_sample_monotonic = 0.0
            if item_kind is not None:
                self._item_kind = str(item_kind)
            if item_index is not None:
                self._item_index = int(item_index)
            self._write_event_locked(str(event))
            self._event_file.flush()

    def stop(self, *, make_plot: bool = True) -> dict:
        with self._lock:
            if not self._active:
                return {"ok": False, "error": "logger_not_active"}

            self._extruder_on = False
            self._write_event_locked("session_stop")
            output_dir = self._output_dir
            sample_count = int(self._sample_count)
            self._active = False

            if self._current_file is not None:
                self._current_file.flush()
                self._current_file.close()
            if self._event_file is not None:
                self._event_file.flush()
                self._event_file.close()
            self._current_file = None
            self._current_writer = None
            self._event_file = None
            self._event_writer = None

        result = {
            "ok": sample_count > 0,
            "samples": sample_count,
            "output_dir": str(output_dir) if output_dir is not None else "",
            "csv_path": str(output_dir / "joint_currents.csv") if output_dir is not None else "",
            "events_path": str(output_dir / "print_events.csv") if output_dir is not None else "",
            "plot_path": "",
            "summary_path": "",
        }
        if make_plot and output_dir is not None and sample_count > 1:
            try:
                analysis_paths = self._make_plot(output_dir)
                result["plot_path"] = str(analysis_paths["plot_path"])
                result["summary_path"] = str(analysis_paths["summary_path"])
            except Exception as exc:  # Plot failure must never fail a print cleanup.
                self._node.get_logger().warn(
                    f"[joint_current_logger] Could not create plot: {exc}"
                )

        if sample_count == 0:
            self._node.get_logger().warn(
                "[joint_current_logger] No joint-current samples were recorded. "
                "Check /joint_states.effort and use_mock_hardware."
            )
        else:
            self._node.get_logger().info(
                f"[joint_current_logger] Saved {sample_count} samples in '{output_dir}'."
            )
        return result

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            if not self._active:
                return
            if not msg.name or len(msg.effort) != len(msg.name):
                if not self._warned_missing_effort:
                    self._warned_missing_effort = True
                    self._node.get_logger().warn(
                        "[joint_current_logger] /joint_states has no complete effort array; "
                        "waiting for UR actual_current samples."
                    )
                return

            callback_monotonic = time.monotonic()
            if (
                self._next_sample_monotonic > 0.0
                and callback_monotonic < self._next_sample_monotonic
            ):
                return
            if self._next_sample_monotonic <= 0.0:
                self._next_sample_monotonic = callback_monotonic + self._sample_period_s
            else:
                periods_late = max(
                    0,
                    int(
                        (callback_monotonic - self._next_sample_monotonic)
                        / self._sample_period_s
                    ),
                )
                self._next_sample_monotonic += (periods_late + 1) * self._sample_period_s

            if not self._joint_names:
                self._joint_names = [str(name) for name in msg.name]
                safe_names = [self._safe_column_name(name) for name in self._joint_names]
                header = [
                    "ros_time_s",
                    "elapsed_s",
                    "extruder_on",
                    "item_kind",
                    "item_index",
                ]
                header += [f"{name}_current_a" for name in safe_names]
                header += [f"{name}_position_rad" for name in safe_names]
                header += [f"{name}_velocity_rad_s" for name in safe_names]
                self._current_writer = csv.writer(self._current_file)
                self._current_writer.writerow(header)

            index_by_name = {str(name): idx for idx, name in enumerate(msg.name)}
            if any(name not in index_by_name for name in self._joint_names):
                return

            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            ros_time_s = (
                1e-9 * float(stamp_ns)
                if stamp_ns > 0
                else 1e-9 * float(self._node.get_clock().now().nanoseconds)
            )
            elapsed_s = callback_monotonic - self._start_monotonic
            indices = [index_by_name[name] for name in self._joint_names]
            currents = [self._finite_or_nan(msg.effort[idx]) for idx in indices]
            positions = [
                self._finite_or_nan(msg.position[idx]) if idx < len(msg.position) else math.nan
                for idx in indices
            ]
            velocities = [
                self._finite_or_nan(msg.velocity[idx]) if idx < len(msg.velocity) else math.nan
                for idx in indices
            ]
            self._current_writer.writerow(
                [
                    f"{ros_time_s:.9f}",
                    f"{elapsed_s:.9f}",
                    int(self._extruder_on),
                    self._item_kind,
                    self._item_index,
                    *currents,
                    *positions,
                    *velocities,
                ]
            )
            self._sample_count += 1

    def _write_event_locked(self, event: str, *, label: str = "") -> None:
        ros_time_s = 1e-9 * float(self._node.get_clock().now().nanoseconds)
        elapsed_s = time.monotonic() - self._start_monotonic
        event_text = str(event) if not label else f"{event}:{label}"
        self._event_writer.writerow(
            [
                f"{ros_time_s:.9f}",
                f"{elapsed_s:.9f}",
                event_text,
                int(self._extruder_on),
                self._item_kind,
                self._item_index,
            ]
        )

    @staticmethod
    def _safe_column_name(value: str) -> str:
        return re.sub(r"[^A-Za-z0-9_]+", "_", str(value)).strip("_") or "joint"

    @staticmethod
    def _finite_or_nan(value) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return math.nan
        return number if math.isfinite(number) else math.nan

    def _make_plot(self, output_dir: Path) -> dict[str, Path]:
        # Imports stay optional so CSV logging still works without plotting packages.
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np

        csv_path = output_dir / "joint_currents.csv"
        with csv_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)
            fieldnames = list(reader.fieldnames or [])
        if len(rows) < 2:
            raise RuntimeError("not enough samples")

        current_columns = [name for name in fieldnames if name.endswith("_current_a")]
        if not current_columns:
            raise RuntimeError("no current columns in CSV")

        elapsed = np.asarray([float(row["elapsed_s"]) for row in rows], dtype=float)
        extrusion = np.asarray([int(row["extruder_on"]) != 0 for row in rows], dtype=bool)
        currents = np.column_stack(
            [[float(row[name]) for row in rows] for name in current_columns]
        )

        # Local baseline removes slow gravity/posture drift. The score is exploratory:
        # it highlights fast current deviations, not calibrated external torque.
        dt = float(np.median(np.diff(elapsed)))
        window = max(3, min(501, int(round(0.20 / max(dt, 1e-4)))))
        if window % 2 == 0:
            window += 1
        baseline = np.column_stack(
            [self._centered_moving_average(currents[:, idx], window) for idx in range(currents.shape[1])]
        )
        residual = np.abs(currents - baseline)
        reference = residual[extrusion] if np.any(extrusion) else residual
        median = np.nanmedian(reference, axis=0)
        mad = np.nanmedian(np.abs(reference - median), axis=0)
        scale = np.maximum(1.4826 * mad, 1e-6)
        score = np.maximum(0.0, np.nanmax((residual - median) / scale, axis=1))
        peak_mask = extrusion & np.isfinite(score) & (score >= 6.0)
        peak_indices = self._separated_peak_indices(score, peak_mask, elapsed, min_gap_s=0.05)
        summary_path = self._write_bead_summary(
            output_dir=output_dir,
            rows=rows,
            elapsed=elapsed,
            extrusion=extrusion,
            currents=currents,
            current_columns=current_columns,
            score=score,
        )
        plot_step = max(1, int(math.ceil(len(elapsed) / 50_000)))
        plot_indices = np.arange(0, len(elapsed), plot_step, dtype=int)

        fig, (ax_current, ax_score) = plt.subplots(
            2, 1, figsize=(14, 8), sharex=True, gridspec_kw={"height_ratios": [2.0, 1.0]}
        )
        for idx, column in enumerate(current_columns):
            label = column[: -len("_current_a")]
            ax_current.plot(
                elapsed[plot_indices],
                currents[plot_indices, idx],
                linewidth=0.35,
                label=label,
            )
        self._shade_extrusion(ax_current, elapsed, extrusion)
        ax_current.set_ylabel("Joint current [A]")
        ax_current.set_title("UR joint currents during the print session")
        ax_current.grid(alpha=0.25)
        ax_current.legend(loc="upper right", ncol=2, fontsize=8)

        ax_score.plot(
            elapsed[plot_indices],
            score[plot_indices],
            color="black",
            linewidth=0.5,
            label="robust transient score",
        )
        ax_score.axhline(
            6.0,
            color="tab:red",
            linestyle="--",
            linewidth=0.7,
            label="exploratory threshold",
        )
        if peak_indices:
            ax_score.scatter(
                elapsed[peak_indices],
                score[peak_indices],
                color="tab:red",
                s=10,
                zorder=3,
                label="candidate peak",
            )
        self._shade_extrusion(ax_score, elapsed, extrusion)
        ax_score.set_xlabel("Elapsed time [s]")
        ax_score.set_ylabel("Robust deviation [MAD scale]")
        ax_score.grid(alpha=0.25)
        ax_score.legend(loc="upper right", fontsize=8)

        fig.tight_layout()
        plot_path = output_dir / "joint_currents.png"
        fig.savefig(plot_path, dpi=160)
        plt.close(fig)
        return {"plot_path": plot_path, "summary_path": summary_path}

    @staticmethod
    def _write_bead_summary(
        *,
        output_dir: Path,
        rows,
        elapsed,
        extrusion,
        currents,
        current_columns,
        score,
    ) -> Path:
        import numpy as np

        groups: dict[tuple[str, int], list[int]] = {}
        for idx, row in enumerate(rows):
            if not extrusion[idx]:
                continue
            kind = str(row.get("item_kind", "")).strip() or "unknown"
            try:
                item_index = int(row.get("item_index", -1))
            except (TypeError, ValueError):
                item_index = -1
            groups.setdefault((kind, item_index), []).append(idx)

        summary_path = output_dir / "bead_current_summary.csv"
        joint_labels = [name[: -len("_current_a")] for name in current_columns]
        header = [
            "item_kind",
            "item_index",
            "start_elapsed_s",
            "duration_s",
            "samples",
            "max_robust_transient_score",
        ]
        for label in joint_labels:
            header.extend(
                [
                    f"{label}_baseline_a",
                    f"{label}_median_a",
                    f"{label}_peak_abs_delta_a",
                    f"{label}_p95_abs_delta_a",
                ]
            )

        with summary_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for (kind, item_index), indices in sorted(
                groups.items(), key=lambda item: float(elapsed[item[1][0]])
            ):
                start_idx = indices[0]
                stop_idx = indices[-1]
                start_s = float(elapsed[start_idx])
                baseline_mask = (
                    (elapsed >= start_s - 0.25)
                    & (elapsed < start_s)
                    & (~extrusion)
                )
                if np.any(baseline_mask):
                    baseline = np.nanmedian(currents[baseline_mask], axis=0)
                else:
                    baseline = currents[start_idx].copy()
                active_values = currents[indices]
                delta_abs = np.abs(active_values - baseline)

                output_row = [
                    kind,
                    item_index,
                    f"{start_s:.9f}",
                    f"{max(0.0, float(elapsed[stop_idx]) - start_s):.9f}",
                    len(indices),
                    f"{float(np.nanmax(score[indices])):.6f}",
                ]
                for joint_idx in range(currents.shape[1]):
                    output_row.extend(
                        [
                            f"{float(baseline[joint_idx]):.9g}",
                            f"{float(np.nanmedian(active_values[:, joint_idx])):.9g}",
                            f"{float(np.nanmax(delta_abs[:, joint_idx])):.9g}",
                            f"{float(np.nanpercentile(delta_abs[:, joint_idx], 95.0)):.9g}",
                        ]
                    )
                writer.writerow(output_row)
        return summary_path

    @staticmethod
    def _centered_moving_average(values, window: int):
        import numpy as np

        array = np.asarray(values, dtype=float)
        if window <= 1 or len(array) < 3:
            return array.copy()
        half = window // 2
        padded = np.pad(array, (half, half), mode="edge")
        cumulative = np.concatenate(([0.0], np.cumsum(padded)))
        return (cumulative[window:] - cumulative[:-window]) / float(window)

    @staticmethod
    def _separated_peak_indices(score, mask, elapsed, *, min_gap_s: float) -> list[int]:
        candidates = [
            idx
            for idx in range(1, len(score) - 1)
            if mask[idx] and score[idx] >= score[idx - 1] and score[idx] >= score[idx + 1]
        ]
        selected: list[int] = []
        for idx in candidates:
            if not selected or float(elapsed[idx]) - float(elapsed[selected[-1]]) >= min_gap_s:
                selected.append(idx)
            elif float(score[idx]) > float(score[selected[-1]]):
                selected[-1] = idx
        if len(selected) > 500:
            selected = sorted(
                sorted(selected, key=lambda item: float(score[item]), reverse=True)[:500]
            )
        return selected

    @staticmethod
    def _shade_extrusion(axis, elapsed, extrusion) -> None:
        import numpy as np

        padded = np.concatenate(([False], extrusion, [False])).astype(int)
        changes = np.diff(padded)
        starts = np.flatnonzero(changes == 1)
        stops = np.flatnonzero(changes == -1) - 1
        for pos, (start, stop) in enumerate(zip(starts, stops)):
            axis.axvspan(
                elapsed[start],
                elapsed[stop],
                color="tab:orange",
                alpha=0.12,
                label="extruder active" if pos == 0 else None,
            )
