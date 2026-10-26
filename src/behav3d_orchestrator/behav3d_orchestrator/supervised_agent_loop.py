"""Human-approved ROS observation review; physical execution is not registered."""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from behav3d_interfaces.action import EvaluateObservation
from behav3d_interfaces.srv import ApproveSupervisedStep, SetSupervisorArmed

from .observation_bridge import action_result, failed_result
from .src.observation_session import ObservationSession
from .supervised_agent.observation import (
    RESULT_FIELDS, request_from_goal, validate_request, validate_result)
from .supervised_agent.supervisor import Supervisor, SupervisorError


class SupervisedAgentLoop(Node):
    """An opt-in review coordinator, disarmed until an operator arms it.

    ROS carries lifecycle and approval. Large evidence remains source-bound
    files. This node never submits motion, scanning or extrusion commands.
    """

    def __init__(self):
        super().__init__('behav3d_supervised_agent_loop')
        session_path = self.declare_parameter('session_path', '').value
        self._approval_timeout = self.declare_parameter('approval_timeout_s', 300.0).value
        self._observation_timeout = self.declare_parameter('observation_timeout_s', 125.0).value
        session = Path(session_path)
        if (not session_path or not session.is_absolute() or not session.is_dir()):
            raise ValueError('session_path must name an explicit existing capture session')
        for value in (self._approval_timeout, self._observation_timeout):
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not 0 < value <= 3600):
                raise ValueError('timeouts must be finite and in (0, 3600]')
        self._supervisor = Supervisor(session / 'supervised_agent' / 'review_journal.jsonl')
        self._session = ObservationSession(self)
        self._lock = threading.Lock()
        self._pending = None
        self._interruption = None
        self._cancel_event = threading.Event()
        self._approval_event = threading.Event()
        self._shutdown = threading.Event()
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self._status = self.create_publisher(String, '/behav3d/supervised_agent/status', qos)
        group = ReentrantCallbackGroup()
        self._arm_service = self.create_service(
            SetSupervisorArmed, '/behav3d/supervised_agent/set_armed', self._arm,
            callback_group=group)
        self._approve_service = self.create_service(
            ApproveSupervisedStep, '/behav3d/supervised_agent/approve_step', self._approve,
            callback_group=group)
        self._server = ActionServer(
            self, EvaluateObservation, '/behav3d/supervised_agent/review_observation',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=self._cancel, callback_group=group)
        self._status_timer = self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            'DISARMED: recorded observation review only; no hardware executor.')

    def _publish_state(self):
        state = self._state()
        with self._lock:
            pending = None if self._pending is None else {
                'request_id': self._pending['request']['request_id'],
                'proposal_sha256': self._pending['proposal_sha256']}
        message = String()
        message.data = json.dumps({
            'schema': 'behav3d.supervised_review_status.v1',
            'state': state['state'], 'pending': pending,
            'fabrication_authorized': False, 'hardware_execution_available': False},
            allow_nan=False)
        self._status.publish(message)

    def _arm(self, request, response):
        try:
            with self._lock:
                if self._shutdown.is_set():
                    raise SupervisorError('Node is shutting down')
                if not request.armed:
                    self._interruption = 'Operator disarmed review'
                    self._cancel_event.set()
                    self._approval_event.set()
                self._supervisor.set_armed(request.armed, request.operator_id)
            response.success = True
            response.message = 'Armed for observation review only' if request.armed else 'Disarmed'
        except (SupervisorError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        response.state = self._state()['state']
        self._publish_state()
        return response

    def _approve(self, request, response):
        try:
            with self._lock:
                if self._shutdown.is_set():
                    raise SupervisorError('Node is shutting down')
                self._supervisor.approve(
                    request.request_id, request.proposal_sha256,
                    request.operator_id, approve=request.approve)
                if not request.approve:
                    self._interruption = 'Operator denied review'
                    self._cancel_event.set()
                self._approval_event.set()
            response.success = True
            response.message = 'Decision recorded for exact proposal'
        except (SupervisorError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        response.state = self._state()['state']
        self._publish_state()
        return response

    def _goal(self, goal):
        try:
            request = validate_request(request_from_goal(goal))
            with self._lock:
                if self._pending is not None or self._shutdown.is_set():
                    return GoalResponse.REJECT
                proposal = self._supervisor.propose('evaluate_observation', request)
                self._cancel_event.clear()
                self._approval_event.clear()
                self._interruption = None
                self._pending = {'request': request,
                                 'proposal_sha256': proposal['proposal_sha256']}
            self._publish_state()
            return GoalResponse.ACCEPT
        except (SupervisorError, ValueError, TypeError):
            return GoalResponse.REJECT

    def _cancel(self, handle):
        with self._lock:
            if (self._pending is None or
                    self._pending['request']['request_id'] != handle.request.request_id):
                return CancelResponse.REJECT
            self._interruption = 'ROS goal cancelled'
            self._pending['ros_cancel_accepted'] = True
            self._cancel_event.set()
            self._approval_event.set()
            try:
                self._supervisor.cancel(handle.request.request_id, 'ROS goal cancelled')
            except SupervisorError:
                # Cancellation remains effective even if the journal fails closed.
                pass
        return CancelResponse.ACCEPT

    def _execute(self, handle):
        request = request_from_goal(handle.request)
        with self._lock:
            proposal_hash = self._pending['proposal_sha256']
        try:
            if self._supervisor.receipt(request['request_id']) is None:
                feedback = EvaluateObservation.Feedback()
                feedback.stage = 'WAITING_APPROVAL'
                feedback.message = 'Approve the exact proposal hash on the ROS approval service.'
                handle.publish_feedback(feedback)
                deadline = time.monotonic() + self._approval_timeout
                while not self._approval_event.wait(0.1):
                    if handle.is_cancel_requested or self._shutdown.is_set():
                        self._cancel_event.set()
                        break
                    if time.monotonic() >= deadline:
                        self._supervisor.cancel(request['request_id'], 'Approval deadline exceeded')
                        raise TimeoutError('Approval deadline exceeded')
            if (self._cancel_event.is_set() or self._shutdown.is_set()
                    or handle.is_cancel_requested):
                raise InterruptedError('Review cancelled or disarmed')

            def evaluate():
                return self._session.evaluate(
                    request, timeout_s=self._observation_timeout,
                    cancel_event=self._cancel_event)

            command = self._supervisor.execute(request['request_id'], proposal_hash, evaluate)
            if command.get('ok') is not True:
                status = command.get('metrics', {}).get('status', 'FAILED')
                if status not in {'FAILED', 'REJECTED', 'CANCELLED', 'TIMED_OUT',
                                  'UNSUPPORTED', 'STALE_SOURCE'}:
                    status = 'FAILED'
                result = failed_result(request['request_id'],
                                       status,
                                       command.get('error') or 'Observation review failed')
            else:
                result = validate_result({field: command['metrics'][field]
                                          for field in RESULT_FIELDS}, request['request_id'])
        except TimeoutError:
            self._cancel_event.set()
            self._invalidate('Review deadline exceeded')
            result = failed_result(request['request_id'], 'TIMED_OUT', 'Review deadline exceeded')
        except InterruptedError:
            result = failed_result(request['request_id'], 'CANCELLED', 'Review cancelled or disarmed')
        except Exception:
            self._cancel_event.set()
            self._invalidate('Review failed; operator reconciliation required')
            result = failed_result(request['request_id'], 'FAILED', 'Review failed; inspect supervisor journal')
        try:
            cancel_state_deadline = None
            while True:
                with self._lock:
                    # rclpy invokes cancel_callback before changing the goal to
                    # CANCELING. Waking the approval waiter inside _cancel can
                    # therefore race that state transition. Let its callback
                    # return without holding our lock before terminalizing.
                    await_ros_cancel = (
                        self._pending.get('ros_cancel_accepted', False)
                        and not handle.is_cancel_requested)
                    if await_ros_cancel:
                        if cancel_state_deadline is None:
                            cancel_state_deadline = time.monotonic() + 1.0
                        await_ros_cancel = time.monotonic() < cancel_state_deadline
                    if not await_ros_cancel:
                        if (handle.is_cancel_requested or self._interruption is not None
                                or self._shutdown.is_set()):
                            result = failed_result(request['request_id'], 'CANCELLED',
                                                   'Late result discarded after cancellation or disarm')
                        response = action_result(result)
                        if handle.is_cancel_requested:
                            handle.canceled()
                        elif result['success']:
                            handle.succeed()
                        else:
                            # A missing ROS transition fails closed as ABORTED;
                            # canceled() is invalid before the CANCELING state.
                            handle.abort()
                        return response
                time.sleep(0.001)
        finally:
            with self._lock:
                self._pending = None
            self._publish_state()

    def _state(self):
        try:
            return self._supervisor.snapshot()
        except SupervisorError:
            return {'state': 'HOLD'}

    def _invalidate(self, reason):
        try:
            self._supervisor.invalidate(reason)
        except SupervisorError:
            self.get_logger().error('Supervisor journal unavailable; review remains held')

    def begin_shutdown(self):
        with self._lock:
            if self._shutdown.is_set():
                return
            self._shutdown.set()
            self._interruption = 'Node shutting down'
            self._cancel_event.set()
            self._approval_event.set()
            self._invalidate('Node shutdown; pending approvals invalidated')

    def destroy_node(self):
        self.begin_shutdown()
        try:
            self._supervisor.close()
        except SupervisorError:
            # Retain exclusive journal ownership until the in-flight callback exits.
            self.get_logger().error('Review still in flight; journal lock retained until process exit')
        self._server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SupervisedAgentLoop()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Signal handlers can close the context before executor.spin returns.
        pass
    except _rclpy.RCLError:
        # A signal can invalidate the context during executor WaitSet creation.
        if rclpy.ok():
            raise
    finally:
        node.begin_shutdown()
        executor.shutdown(timeout_sec=10.0)
        node.destroy_node()
        rclpy.try_shutdown()
