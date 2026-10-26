"""ROS review bridge to isolated saved-evidence or opt-in geometry analysis."""
from __future__ import annotations

from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from behav3d_interfaces.action import EvaluateObservation

from .supervised_agent.observation import (
    RESULT_FIELDS, request_from_goal, validate_request)
from .supervised_agent.process_runner import WorkerError, run_observation_worker


def action_result(values):
    result = EvaluateObservation.Result()
    for field in RESULT_FIELDS:
        setattr(result, field, values[field])
    return result


def failed_result(request_id, status, message):
    return dict(success=False, status=status, message=message,
                request_id=request_id, evaluation_id='', packet_path='',
                packet_sha256='', evidence_manifest_path='',
                evidence_manifest_sha256='', requires_review=True,
                fabrication_authorized=False)


class ObservationBridge(Node):
    """One concurrent review goal; no hardware or model backend is registered."""

    def __init__(self):
        super().__init__('behav3d_observation_bridge')
        self._evidence_root = self.declare_parameter('evidence_root', '').value
        self._source_roots = self.declare_parameter('source_roots', ['']).value
        self._source_roots = [value for value in self._source_roots if value]
        self._python = self.declare_parameter('worker_python', sys.executable).value
        self._timeout = self.declare_parameter('worker_timeout_s', 120.0).value
        self._backend_mode = self.declare_parameter('backend_mode', 'recorded').value
        if self._backend_mode not in {'recorded', 'fresh_geometry'}:
            raise ValueError('backend_mode must be recorded or fresh_geometry')
        self._fresh_roots = {
            'agent007_root': self.declare_parameter('agent007_root', '').value,
            'dds_root': self.declare_parameter('dds_root', '').value,
            'output_root': self.declare_parameter('analysis_output_root', '').value,
        }
        self._busy = False
        self._request_id = None
        self._cancel_event = threading.Event()
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._server = ActionServer(
            self, EvaluateObservation, '/behav3d/evaluate_observation',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            'Observation bridge: ' + self._backend_mode + '; no fabrication authority.')

    def _goal(self, goal):
        try:
            request = validate_request(request_from_goal(goal))
            if (not self._evidence_root
                    or not Path(self._evidence_root).is_absolute()
                    or not Path(self._evidence_root).is_dir()):
                raise ValueError('Configure an explicit evidence_root first')
            if self._backend_mode == 'fresh_geometry':
                for value in self._fresh_roots.values():
                    if not value or not Path(value).is_absolute() or not Path(value).is_dir():
                        raise ValueError('Configure explicit fresh-analysis roots first')
            elif any(self._fresh_roots.values()):
                raise ValueError('Fresh roots require explicit fresh_geometry mode')
            with self._lock:
                if self._busy or self._shutdown.is_set():
                    return GoalResponse.REJECT
                self._busy = True
                self._request_id = request['request_id']
                self._cancel_event.clear()
            return GoalResponse.ACCEPT
        except (TypeError, ValueError):
            return GoalResponse.REJECT

    def _cancel(self, handle):
        with self._lock:
            if not self._busy or self._request_id != handle.request.request_id:
                return CancelResponse.REJECT
            # rclpy changes the handle to CANCELING only after this returns.
            self._cancel_event.set()
        return CancelResponse.ACCEPT

    def _execute(self, handle):
        request_id = handle.request.request_id
        try:
            request = request_from_goal(handle.request)
            feedback = EvaluateObservation.Feedback()
            fresh = self._backend_mode == 'fresh_geometry'
            feedback.stage = 'ANALYSING_FRESH_GEOMETRY' if fresh else 'VERIFYING_RECORDED_EVIDENCE'
            feedback.message = ('Reconstructing supplied scans and computing review evidence.' if fresh
                                else 'Checking source identities and immutable evidence.')
            handle.publish_feedback(feedback)
            policy = dict(backend_mode=self._backend_mode)
            if fresh:
                policy.update(self._fresh_roots)
            result = run_observation_worker(
                request, python_executable=self._python,
                evidence_root=self._evidence_root, source_roots=self._source_roots,
                timeout_s=self._timeout,
                cancelled=lambda: self._cancel_event.is_set() or handle.is_cancel_requested
                    or self._shutdown.is_set(),
                **policy)
        except WorkerError as exc:
            result = failed_result(request_id, exc.status, str(exc))
        except Exception:
            result = failed_result(request_id, 'FAILED',
                                   'Observation worker could not complete')
        cancel_deadline = time.monotonic() + 1.0
        try:
            while True:
                with self._lock:
                    await_transition = (self._cancel_event.is_set() and not handle.is_cancel_requested
                                        and time.monotonic() < cancel_deadline)
                    if not await_transition:
                        if (self._cancel_event.is_set() or handle.is_cancel_requested
                                or self._shutdown.is_set()):
                            result = failed_result(request_id, 'CANCELLED',
                                                   'Late result discarded after cancellation')
                        response = action_result(result)
                        if handle.is_cancel_requested:
                            handle.canceled()
                        elif result['success']:
                            handle.succeed()
                        else:
                            handle.abort()
                        return response
                # Do not hold the lock while rclpy completes the cancel transition.
                time.sleep(0.001)
        finally:
            with self._lock:
                self._busy = False
                self._request_id = None

    def destroy_node(self):
        self._shutdown.set()
        self._server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObservationBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM may already have closed the default ROS context.
        pass
    except _rclpy.RCLError:
        # Context shutdown can race WaitSet construction in the executor.
        # Suppress that shutdown-only ROS error, never an active-context error.
        if rclpy.ok():
            raise
    finally:
        node._shutdown.set()
        executor.shutdown(timeout_sec=10.0)
        node.destroy_node()
        rclpy.try_shutdown()
