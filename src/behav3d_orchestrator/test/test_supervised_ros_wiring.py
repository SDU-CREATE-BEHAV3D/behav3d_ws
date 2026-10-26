"""Mocked ROS-shaped wiring tests, not a ROS integration/hardware test.

Real coordinator, supervisor, observation validation, Session/router and pause
code run against isolated in-memory ROS/interface stand-ins. No ROS servers,
executors, camera, model calls, external processes or robot connections start.
"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[3]
ORCHESTRATOR = ROOT / 'src/behav3d_orchestrator/behav3d_orchestrator'
COMMANDS = ROOT / 'src/behav3d_commands/behav3d_commands'
INTERFACES = ROOT / 'src/behav3d_interfaces'


def typed_message(name, definitions):
    fields = []
    defaults = {}
    for line in definitions.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        kind, field = line.split()
        fields.append(field)
        defaults[field] = '' if kind == 'string' else False if kind == 'bool' else 0

    def init(self, **values):
        for field in fields:
            setattr(self, field, values.get(field, defaults[field]))

    return type(name, (), {'__slots__': fields, '__init__': init})


def module(name, directory=None):
    value = types.ModuleType(name)
    value.__path__ = [] if directory is None else [str(directory)]
    return value


def request(request_id='request-1', scan_cycle=1, previous=''):
    return {
        'request_id': request_id, 'artifact_id': 'wall',
        'observation_id': 'scan-' + str(scan_cycle),
        'event_type': 'deposition_readback', 'candidate_cycle': scan_cycle - 1,
        'scan_cycle': scan_cycle, 'previous_evaluation_id': previous,
        'frame_id': 'ur20_base_link', 'coordinate_units': 'm',
        'source_manifest_path': '/recorded/source_manifest.json',
        'source_manifest_sha256': 'a' * 64,
    }


def action_result(request_id='request-1'):
    return {
        'success': True, 'status': 'REVIEW_REQUIRED', 'message': 'Recorded review only',
        'request_id': request_id, 'evaluation_id': 'evaluation_' + '1' * 64,
        'packet_path': '/recorded/packet.json', 'packet_sha256': 'b' * 64,
        'evidence_manifest_path': '/recorded/evidence.json',
        'evidence_manifest_sha256': 'c' * 64,
        'requires_review': True, 'fabrication_authorized': False,
    }


def command_result(request_id='request-1'):
    return {
        'ok': True, 'kind': 'evaluate_observation', 'phase': 'plan',
        'planned_only': True, 'error': None, 'metrics': action_result(request_id),
    }


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeHandle:
    def __init__(self, goal):
        self.request = goal
        self.is_cancel_requested = False
        self.feedback = []
        self.feedback_event = threading.Event()
        self.terminals = []

    def publish_feedback(self, value):
        self.feedback.append(value)
        self.feedback_event.set()

    def succeed(self):
        self.terminals.append('SUCCEEDED')

    def abort(self):
        self.terminals.append('ABORTED')

    def canceled(self):
        if not self.is_cancel_requested:
            raise RuntimeError('CANCELED requires a ROS cancellation request')
        self.terminals.append('CANCELLED')


def load_wiring(parameters):
    """Restore sys.modules completely, including pre-existing real packages."""
    physical_constructions = []

    class FakeNode:
        def __init__(self, name='test'):
            self.name = name
            self.subscriptions = []
            self.services = []
            self.publishers = []
            self.timers = []
            self.destroyed = False

        def declare_parameter(self, name, default):
            return types.SimpleNamespace(value=parameters.get(name, default))

        def get_logger(self):
            return types.SimpleNamespace(
                info=lambda _: None, warning=lambda _: None,
                warn=lambda _: None, error=lambda _: None)

        def create_subscription(self, message, topic, callback, qos):
            self.subscriptions.append((topic, callback))
            return types.SimpleNamespace()

        def create_publisher(self, message, topic, qos):
            publisher = types.SimpleNamespace(topic=topic, messages=[])
            publisher.publish = publisher.messages.append
            self.publishers.append(publisher)
            return publisher

        def create_service(self, service, name, callback, **kwargs):
            self.services.append((service, name, callback))
            return types.SimpleNamespace()

        def create_timer(self, period, callback, **kwargs):
            timer = FakeTimer(callback)
            self.timers.append(timer)
            return timer

        def destroy_timer(self, timer):
            timer.cancel()

        def destroy_node(self):
            self.destroyed = True
            return True

    class FakeActionServer:
        def __init__(self, node, action, name, **callbacks):
            self.name = name
            self.callbacks = callbacks
            self.destroyed = False

        def destroy(self):
            self.destroyed = True

    class FakeActionClient:
        def __init__(self, node, action, name):
            self.name = name

        def server_is_ready(self):
            return False

        def send_goal_async(self, goal):
            raise AssertionError('No ROS action may be dispatched by wiring fixtures')

    class FakeSession:
        def __init__(self, node):
            self.calls = []
            self.handler = lambda value, **_: command_result(value['request_id'])

        def evaluate(self, value, **kwargs):
            self.calls.append((dict(value), kwargs))
            return self.handler(value, **kwargs)

    class PhysicalCommands:
        def __init__(self, *args, **kwargs):
            physical_constructions.append(type(self).__name__)

        def register(self, router):
            router.register('PHYSICAL_HANDLER_MUST_NOT_BE_REGISTERED', lambda *args: None)

    class Qos:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    modules = {
        name: module(name) for name in (
            'rclpy', 'rclpy.action', 'rclpy.clock', 'rclpy.node',
            'rclpy.callback_groups', 'rclpy.executors', 'rclpy.qos',
            'rclpy.impl', 'rclpy.impl.implementation_singleton',
            'std_msgs', 'std_msgs.msg', 'action_msgs', 'action_msgs.msg',
            'behav3d_interfaces', 'behav3d_interfaces.action', 'behav3d_interfaces.srv',
        )
    }
    modules.update({
        'behav3d_orchestrator': module('behav3d_orchestrator', ORCHESTRATOR),
        'behav3d_orchestrator.src': module('behav3d_orchestrator.src', ORCHESTRATOR / 'src'),
        'behav3d_orchestrator.supervised_agent': module(
            'behav3d_orchestrator.supervised_agent', ORCHESTRATOR / 'supervised_agent'),
        'behav3d_commands': module('behav3d_commands', COMMANDS),
        'behav3d_commands.sense_commands': module('behav3d_commands.sense_commands'),
        'behav3d_commands.motion_commands': module('behav3d_commands.motion_commands'),
        'behav3d_commands.print_commands': module('behav3d_commands.print_commands'),
        'behav3d_commands.util_commands': module('behav3d_commands.util_commands'),
    })
    modules['rclpy'].ok = lambda: True
    modules['rclpy.node'].Node = FakeNode
    modules['rclpy.action'].ActionServer = FakeActionServer
    modules['rclpy.action'].ActionClient = FakeActionClient
    modules['rclpy.action'].GoalResponse = types.SimpleNamespace(ACCEPT='ACCEPT', REJECT='REJECT')
    modules['rclpy.action'].CancelResponse = types.SimpleNamespace(ACCEPT='ACCEPT', REJECT='REJECT')
    modules['rclpy.clock'].Clock = lambda **kwargs: types.SimpleNamespace(**kwargs)
    modules['rclpy.clock'].ClockType = types.SimpleNamespace(STEADY_TIME=1)
    modules['rclpy.callback_groups'].ReentrantCallbackGroup = object
    modules['rclpy.executors'].MultiThreadedExecutor = object
    modules['rclpy.executors'].ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
    modules['rclpy.impl.implementation_singleton'].rclpy_implementation = types.SimpleNamespace(
        RCLError=type('RCLError', (RuntimeError,), {}))
    modules['rclpy.qos'].QoSProfile = Qos
    modules['rclpy.qos'].QoSDurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    modules['rclpy.qos'].QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    modules['std_msgs.msg'].String = typed_message('String', 'string data')
    modules['action_msgs.msg'].GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)
    goal, result, feedback = (INTERFACES / 'action/EvaluateObservation.action').read_text().split('---')
    action = types.SimpleNamespace(
        Goal=typed_message('Goal', goal), Result=typed_message('Result', result),
        Feedback=typed_message('Feedback', feedback))
    modules['behav3d_interfaces.action'].EvaluateObservation = action
    for name in ('SetSupervisorArmed', 'ApproveSupervisedStep'):
        req, resp = (INTERFACES / ('srv/' + name + '.srv')).read_text().split('---')
        setattr(modules['behav3d_interfaces.srv'], name, types.SimpleNamespace(
            Request=typed_message(name + 'Request', req),
            Response=typed_message(name + 'Response', resp)))
    for package, names in (
        ('motion_commands', ('MotionCommands',)),
        ('sense_commands', ('CameraCommands', 'FieldCommands')),
        ('print_commands', ('ExtruderCommands',)), ('util_commands', ('UtilCommands',)),
    ):
        for name in names:
            setattr(modules['behav3d_commands.' + package], name, PhysicalCommands)

    loaded = {}
    with patch.dict(sys.modules, modules):
        def load(name, path):
            spec = importlib.util.spec_from_file_location(name, path)
            value = importlib.util.module_from_spec(spec)
            sys.modules[name] = value
            spec.loader.exec_module(value)
            loaded[name] = value
            return value

        for suffix in ('observation', 'supervisor', 'process_runner'):
            load('behav3d_orchestrator.supervised_agent.' + suffix,
                 ORCHESTRATOR / 'supervised_agent' / (suffix + '.py'))
        bridge = load('behav3d_orchestrator.observation_bridge', ORCHESTRATOR / 'observation_bridge.py')
        fake_session_module = module('behav3d_orchestrator.src.observation_session')
        fake_session_module.ObservationSession = FakeSession
        sys.modules[fake_session_module.__name__] = fake_session_module
        coordinator = load('behav3d_orchestrator.supervised_agent_loop',
                           ORCHESTRATOR / 'supervised_agent_loop.py')
        for suffix in ('command', 'queue', 'session'):
            load('behav3d_commands.' + suffix, COMMANDS / (suffix + '.py'))
        load('behav3d_commands.sense_commands.observation_commands',
             COMMANDS / 'sense_commands/observation_commands.py')
        pause = load('behav3d_orchestrator.src.control_session',
                     ORCHESTRATOR / 'src/control_session.py')
        observation_session = load('behav3d_orchestrator.src.observation_session',
                                   ORCHESTRATOR / 'src/observation_session.py')
    return types.SimpleNamespace(
        coordinator=coordinator, bridge=bridge, pause=pause,
        observation_session=observation_session, node=FakeNode, action=action,
        physical_constructions=physical_constructions,
        session=loaded['behav3d_commands.session'],
        arm_service=modules['behav3d_interfaces.srv'].SetSupervisorArmed,
        approve_service=modules['behav3d_interfaces.srv'].ApproveSupervisedStep)


class SupervisedWiringTests(unittest.TestCase):
    def test_node_mains_only_ignore_rcl_error_after_context_shutdown(self):
        for target, constructor in ((self.wiring.bridge, 'ObservationBridge'),
                                    (self.wiring.coordinator, 'SupervisedAgentLoop')):
            for active in (True, False):
                node, executor = Mock(), Mock()
                executor.spin.side_effect = target._rclpy.RCLError('invalid wait set context')
                with self.subTest(node=constructor, active=active), \
                        patch.object(target, constructor, return_value=node), \
                        patch.object(target, 'MultiThreadedExecutor', return_value=executor), \
                        patch.object(target.rclpy, 'init', create=True), \
                        patch.object(target.rclpy, 'ok', return_value=active), \
                        patch.object(target.rclpy, 'try_shutdown', create=True) as shutdown:
                    if active:
                        with self.assertRaises(target._rclpy.RCLError):
                            target.main()
                    else:
                        target.main()
                    node.destroy_node.assert_called_once_with()
                    shutdown.assert_called_once_with()

    def test_bridge_cancel_reservation_wins_rclpy_transition_race(self):
        bridge = self.wiring.bridge.ObservationBridge()
        handle = FakeHandle(self.goal())
        transition = None
        try:
            self.assertEqual(bridge._goal(handle.request), 'ACCEPT')
            def late_result(*args, **kwargs):
                nonlocal transition
                self.assertEqual(bridge._cancel(handle), 'ACCEPT')
                self.assertTrue(kwargs['cancelled']())
                transition = threading.Timer(0.02, lambda: setattr(handle, 'is_cancel_requested', True))
                transition.start()
                return action_result()
            with patch.object(self.wiring.bridge, 'run_observation_worker', late_result):
                response = bridge._execute(handle)
            self.assertFalse(response.success)
            self.assertEqual(response.status, 'CANCELLED')
            self.assertEqual(handle.terminals, ['CANCELLED'])
        finally:
            if transition is not None:
                transition.join(timeout=1)
            bridge.destroy_node()

    def test_bridge_rejects_noncurrent_cancel(self):
        bridge = self.wiring.bridge.ObservationBridge()
        try:
            handle = FakeHandle(self.goal())
            self.assertEqual(bridge._cancel(handle), 'REJECT')
            self.assertEqual(bridge._goal(handle.request), 'ACCEPT')
            self.assertEqual(bridge._cancel(FakeHandle(self.goal(request_id='old-request'))), 'REJECT')
            self.assertFalse(bridge._cancel_event.is_set())
        finally:
            bridge.destroy_node()

    def test_unknown_bridge_backend_mode_is_rejected(self):
        self.parameters['backend_mode'] = 'arbitrary_program'
        with self.assertRaises(ValueError):
            self.wiring.bridge.ObservationBridge()

    def test_fresh_bridge_requires_explicit_roots_and_opt_in(self):
        self.parameters['backend_mode'] = 'fresh_geometry'
        bridge = self.wiring.bridge.ObservationBridge()
        try:
            self.assertEqual(bridge._goal(self.goal()), 'REJECT')
        finally:
            bridge.destroy_node()
        self.parameters['backend_mode'] = 'recorded'
        self.parameters['agent007_root'] = self.temp.name
        bridge = self.wiring.bridge.ObservationBridge()
        try:
            self.assertEqual(bridge._goal(self.goal()), 'REJECT')
        finally:
            bridge.destroy_node()

    def test_fresh_bridge_routes_fixed_policy_and_labels_new_analysis(self):
        self.parameters.update(backend_mode='fresh_geometry', agent007_root=self.temp.name,
                               dds_root=self.temp.name, analysis_output_root=self.temp.name)
        bridge = self.wiring.bridge.ObservationBridge()
        handle = FakeHandle(self.goal())
        try:
            self.assertEqual(bridge._goal(handle.request), 'ACCEPT')
            with patch.object(self.wiring.bridge, 'run_observation_worker', return_value=action_result()) as worker:
                response = bridge._execute(handle)
            self.assertTrue(response.success)
            self.assertEqual(handle.feedback[0].stage, 'ANALYSING_FRESH_GEOMETRY')
            self.assertEqual(worker.call_args.kwargs['backend_mode'], 'fresh_geometry')
            self.assertEqual(worker.call_args.kwargs['output_root'], self.temp.name)
            self.assertEqual(handle.terminals, ['SUCCEEDED'])
        finally:
            bridge.destroy_node()

    def test_both_node_mains_cleanup_after_signal_or_normal_spin(self):
        for target, constructor in ((self.wiring.bridge, 'ObservationBridge'),
                                    (self.wiring.coordinator, 'SupervisedAgentLoop')):
            for interruption in (None, KeyboardInterrupt(), target.ExternalShutdownException()):
                with self.subTest(node=constructor, interruption=type(interruption).__name__):
                    node = Mock()
                    executor = Mock()
                    executor.spin.side_effect = interruption
                    with patch.object(target, constructor, return_value=node), \
                            patch.object(target, 'MultiThreadedExecutor', return_value=executor), \
                            patch.object(target.rclpy, 'init', create=True) as initialize, \
                            patch.object(target.rclpy, 'try_shutdown', create=True) as shutdown:
                        target.main(args=[])
                        initialize.assert_called_once_with(args=[])
                        executor.shutdown.assert_called_once_with(timeout_sec=10.0)
                        node.destroy_node.assert_called_once_with()
                        shutdown.assert_called_once_with()
                        if constructor == 'ObservationBridge':
                            node._shutdown.set.assert_called_once_with()
                        else:
                            node.begin_shutdown.assert_called_once_with()

    def test_node_mains_do_not_hide_unexpected_spin_failure(self):
        for target, constructor in ((self.wiring.bridge, 'ObservationBridge'),
                                    (self.wiring.coordinator, 'SupervisedAgentLoop')):
            node, executor = Mock(), Mock()
            executor.spin.side_effect = RuntimeError('unexpected executor failure')
            with self.subTest(node=constructor), \
                    patch.object(target, constructor, return_value=node), \
                    patch.object(target, 'MultiThreadedExecutor', return_value=executor), \
                    patch.object(target.rclpy, 'init', create=True), \
                    patch.object(target.rclpy, 'try_shutdown', create=True) as shutdown, \
                    self.assertRaisesRegex(RuntimeError, 'unexpected executor failure'):
                target.main()
            node.destroy_node.assert_called_once_with()
            shutdown.assert_called_once_with()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.parameters = {
            'session_path': self.temp.name, 'evidence_root': self.temp.name,
            'source_roots': [self.temp.name], 'approval_timeout_s': 2.0,
            'observation_timeout_s': 2.0,
        }
        self.wiring = load_wiring(self.parameters)
        self.node = self.wiring.coordinator.SupervisedAgentLoop()
        self.addCleanup(self.node.destroy_node)

    def arm(self, value=True):
        service = self.wiring.arm_service
        return self.node._arm(service.Request(armed=value, operator_id='operator-a'),
                              service.Response())

    def goal(self, **changes):
        return self.wiring.action.Goal(**dict(request(), **changes))

    def approve(self, digest=None, approve=True):
        pending = self.node._pending
        service = self.wiring.approve_service
        return self.node._approve(service.Request(
            request_id=pending['request']['request_id'],
            proposal_sha256=pending['proposal_sha256'] if digest is None else digest,
            operator_id='operator-a', approve=approve), service.Response())

    def accepted(self):
        self.assertTrue(self.arm().success)
        goal = self.goal()
        self.assertEqual(self.node._goal(goal), 'ACCEPT')
        return FakeHandle(goal)

    def run_thread(self, handle):
        results = []
        errors = []

        def execute():
            try:
                results.append(self.node._execute(handle))
            except BaseException as exc:
                errors.append(exc)

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()
        return thread, results, errors

    def finish_thread(self, thread, errors):
        thread.join(timeout=3)
        self.assertFalse(thread.is_alive(), 'Mocked review must terminate promptly')
        self.assertEqual(errors, [])

    def test_disarmed_malformed_and_busy_goals_are_rejected(self):
        self.assertEqual(self.node._goal(self.goal()), 'REJECT')
        self.assertTrue(self.arm().success)
        self.assertEqual(self.node._goal(self.goal(coordinate_units='mm')), 'REJECT')
        self.assertEqual(self.node._goal(self.goal(scan_cycle=9)), 'REJECT')
        self.assertEqual(self.node._goal(self.goal()), 'ACCEPT')
        self.assertEqual(self.node._goal(self.goal(request_id='request-2')), 'REJECT')
        self.assertEqual(self.node._session.calls, [])

    def test_exact_approval_dispatches_one_standard_command(self):
        handle = self.accepted()
        self.assertFalse(self.approve('f' * 64).success)
        self.assertFalse(self.node._approval_event.is_set())
        self.assertTrue(self.approve().success)
        response = self.node._execute(handle)
        self.assertTrue(response.success)
        self.assertEqual(response.status, 'REVIEW_REQUIRED')
        self.assertTrue(response.requires_review)
        self.assertFalse(response.fabrication_authorized)
        self.assertEqual(handle.terminals, ['SUCCEEDED'])
        self.assertEqual(len(self.node._session.calls), 1)
        sent, options = self.node._session.calls[0]
        self.assertEqual(sent, request())
        self.assertIs(options['cancel_event'], self.node._cancel_event)
        self.assertEqual(options['timeout_s'], 2.0)
        receipt = self.node._supervisor.receipt('request-1')
        self.assertEqual(receipt['result']['kind'], 'evaluate_observation')
        self.assertEqual(receipt['result']['planned_only'], True)

    def test_cancel_waiting_for_approval_does_not_dispatch(self):
        handle = self.accepted()
        thread, results, errors = self.run_thread(handle)
        self.assertTrue(handle.feedback_event.wait(1))
        handle.is_cancel_requested = True
        self.assertEqual(self.node._cancel(handle), 'ACCEPT')
        self.finish_thread(thread, errors)
        self.assertFalse(results[0].success)
        self.assertEqual(results[0].status, 'CANCELLED')
        self.assertEqual(self.node._session.calls, [])
        self.assertEqual(handle.terminals, ['CANCELLED'])

    def test_disarm_after_approval_prevents_dispatch(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        self.assertTrue(self.arm(False).success)
        result = self.node._execute(handle)
        self.assertFalse(result.success)
        self.assertEqual(result.status, 'CANCELLED')
        self.assertEqual(self.node._session.calls, [])

    def test_cancel_callback_wakes_before_ros_canceling_transition(self):
        handle = self.accepted()
        reached_transition_wait, release = threading.Event(), threading.Event()

        def await_ros_transition(_):
            self.assertFalse(self.node._lock.locked())
            reached_transition_wait.set()
            if not release.wait(2):
                raise AssertionError('ROS cancellation transition was not released')

        with patch.object(self.wiring.coordinator.time, 'sleep', await_ros_transition):
            thread, results, errors = self.run_thread(handle)
            try:
                self.assertTrue(handle.feedback_event.wait(1))
                # ActionServer has not yet made is_cancel_requested true when
                # it calls the application's cancellation callback.
                self.assertFalse(handle.is_cancel_requested)
                self.assertEqual(self.node._cancel(handle), 'ACCEPT')
                self.assertTrue(reached_transition_wait.wait(1))
                self.assertIsNotNone(self.node._pending)
                self.assertEqual(handle.terminals, [])
                handle.is_cancel_requested = True
            finally:
                release.set()
            self.finish_thread(thread, errors)
        self.assertEqual(results[0].status, 'CANCELLED')
        self.assertEqual(handle.terminals, ['CANCELLED'])
        self.assertEqual(self.node._session.calls, [])

    def test_missing_ros_canceling_transition_is_bounded_and_fails_closed(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        self.assertEqual(self.node._cancel(handle), 'ACCEPT')
        with patch.object(self.wiring.coordinator.time, 'monotonic',
                          side_effect=[100, 100, 102]):
            response = self.node._execute(handle)
        self.assertFalse(response.success)
        self.assertEqual(response.status, 'CANCELLED')
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertIsNone(self.node._pending)
        self.assertEqual(self.node._session.calls, [])

    def test_disarm_discards_a_late_successful_command_result(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        entered, release = threading.Event(), threading.Event()

        def delayed(value, **kwargs):
            entered.set()
            if not release.wait(2):
                raise AssertionError('Fixture was not released')
            return command_result(value['request_id'])

        self.node._session.handler = delayed
        thread, results, errors = self.run_thread(handle)
        try:
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.arm(False).success)
        finally:
            release.set()
        self.finish_thread(thread, errors)
        self.assertFalse(results[0].success)
        self.assertEqual(results[0].status, 'CANCELLED')
        self.assertEqual(handle.terminals, ['ABORTED'])
        receipt = self.node._supervisor.receipt('request-1')
        self.assertFalse(receipt['accepted'])
        self.assertFalse(receipt['result']['ok'])

    def test_noncurrent_cancel_does_not_poison_current_review(self):
        handle = self.accepted()
        unrelated = FakeHandle(self.goal(request_id='old-request'))
        self.assertEqual(self.node._cancel(unrelated), 'REJECT')
        self.assertFalse(self.node._cancel_event.is_set())
        self.assertTrue(self.approve().success)
        self.assertTrue(self.node._execute(handle).success)

    def test_missing_configuration_is_fail_closed(self):
        self.parameters['session_path'] = ''
        with self.assertRaises(ValueError):
            self.wiring.coordinator.SupervisedAgentLoop()
        self.parameters['evidence_root'] = ''
        bridge = self.wiring.bridge.ObservationBridge()
        try:
            self.assertEqual(bridge._goal(self.goal()), 'REJECT')
        finally:
            bridge.destroy_node()

    def test_bridge_goal_reservation_and_late_cancelled_worker(self):
        bridge = self.wiring.bridge.ObservationBridge()
        handle = FakeHandle(self.goal())
        try:
            self.assertEqual(bridge._goal(handle.request), 'ACCEPT')
            self.assertEqual(bridge._goal(handle.request), 'REJECT')

            def late_worker(value, **kwargs):
                handle.is_cancel_requested = True
                return action_result(value['request_id'])

            with patch.object(self.wiring.bridge, 'run_observation_worker', late_worker):
                response = bridge._execute(handle)
            self.assertFalse(response.success)
            self.assertEqual(response.status, 'CANCELLED')
            self.assertEqual(handle.terminals, ['CANCELLED'])
        finally:
            bridge.destroy_node()

    def test_observation_session_registers_only_evidence_handler(self):
        node = self.wiring.node()
        session = self.wiring.observation_session.ObservationSession(node)
        other = self.wiring.observation_session.ObservationSession(node)
        self.assertEqual(set(session._router._handlers), {'evaluate_observation'})
        self.assertEqual(self.wiring.physical_constructions, [])
        self.assertIs(session.control_pause, other.control_pause)
        self.assertEqual([topic for topic, _ in node.subscriptions], ['/behav3d/control_state'])

    def test_cancelled_shared_pause_returns_without_dispatch(self):
        node = self.wiring.node()
        gate = self.wiring.pause.get_control_pause_gate(node)
        gate._on_state(types.SimpleNamespace(data='paused'))
        cancel = threading.Event()
        returned = []
        thread = threading.Thread(target=lambda: returned.append(
            gate.wait('evaluate_observation', cancel_event=cancel)), daemon=True)
        thread.start()
        cancel.set()
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(returned, [False])

    def test_cancel_during_resume_is_not_lost(self):
        node = self.wiring.node()
        gate = self.wiring.pause.get_control_pause_gate(node)
        gate._paused = True
        cancel = threading.Event()

        class ResumeAndCancel:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def wait(self, **kwargs):
                gate._paused = False
                cancel.set()

        gate._condition = ResumeAndCancel()
        self.assertFalse(gate.wait('evaluate_observation', cancel_event=cancel))

    def test_observation_session_rechecks_cancel_after_pause(self):
        node = self.wiring.node()
        session = self.wiring.observation_session.ObservationSession(node)
        cancel = threading.Event()

        def resumed(*args, **kwargs):
            cancel.set()
            return True

        session.control_pause.wait = resumed
        with patch.object(self.wiring.session.Session, 'run_sync') as dispatch:
            result = session.evaluate(request(), timeout_s=1, cancel_event=cancel)
        dispatch.assert_not_called()
        self.assertFalse(result['ok'])
        self.assertTrue(result['planned_only'])
        self.assertEqual(result['metrics']['status'], 'CANCELLED')
        self.assertFalse(result['metrics']['fabrication_authorized'])

    def test_isolated_import_restores_existing_modules(self):
        sentinel = types.ModuleType('rclpy')
        with patch.dict(sys.modules, {'rclpy': sentinel}):
            load_wiring(self.parameters)
            self.assertIs(sys.modules['rclpy'], sentinel)

    def test_begin_shutdown_discards_inflight_success_and_holds_receipt(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        entered, release = threading.Event(), threading.Event()

        def delayed(value, **kwargs):
            entered.set()
            if not release.wait(2):
                raise AssertionError('Fixture was not released')
            return command_result(value['request_id'])

        self.node._session.handler = delayed
        thread, results, errors = self.run_thread(handle)
        try:
            self.assertTrue(entered.wait(1))
            self.node.begin_shutdown()
            self.assertTrue(self.node._shutdown.is_set())
            self.assertTrue(self.node._cancel_event.is_set())
            self.assertFalse(self.arm().success)
            self.assertEqual(self.node._goal(self.goal(request_id='new-request')), 'REJECT')
        finally:
            release.set()
        self.finish_thread(thread, errors)
        self.assertFalse(results[0].success)
        self.assertEqual(results[0].status, 'CANCELLED')
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertEqual(len(self.node._session.calls), 1)
        receipt = self.node._supervisor.receipt('request-1')
        self.assertEqual(receipt['state'], 'HOLD')
        self.assertFalse(receipt['accepted'])
        self.assertFalse(receipt['result']['ok'])
        self.assertTrue(receipt['unaccepted_operation_result']['ok'])
        self.assertFalse(self.node._supervisor.snapshot()['armed'])

    def test_begin_shutdown_before_dispatch_invalidates_approval(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        self.node.begin_shutdown()
        response = self.node._execute(handle)
        self.assertFalse(response.success)
        self.assertEqual(response.status, 'CANCELLED')
        self.assertEqual(self.node._session.calls, [])
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertIsNone(self.node._supervisor.receipt('request-1'))

    def test_coordinator_reservation_covers_conversion_and_terminal_transition(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        phases = []
        original_converter = self.wiring.coordinator.action_result
        original_succeed = handle.succeed

        def assert_reserved(phase):
            phases.append(phase)
            self.assertIsNotNone(self.node._pending)
            self.assertEqual(self.node._pending['request']['request_id'], 'request-1')
            self.assertTrue(self.node._lock.locked())

        def convert(values):
            assert_reserved('conversion')
            return original_converter(values)

        def succeed():
            assert_reserved('terminal')
            original_succeed()

        handle.succeed = succeed
        with patch.object(self.wiring.coordinator, 'action_result', convert):
            response = self.node._execute(handle)
        self.assertTrue(response.success)
        self.assertEqual(phases, ['conversion', 'terminal'])
        self.assertIsNone(self.node._pending)

    def test_bridge_reservation_covers_conversion_and_terminal_transition(self):
        bridge = self.wiring.bridge.ObservationBridge()
        handle = FakeHandle(self.goal())
        phases = []
        original_converter = self.wiring.bridge.action_result
        original_succeed = handle.succeed

        def assert_reserved(phase):
            phases.append(phase)
            self.assertTrue(bridge._busy)
            self.assertTrue(bridge._lock.locked())

        def convert(values):
            assert_reserved('conversion')
            return original_converter(values)

        def succeed():
            assert_reserved('terminal')
            original_succeed()

        handle.succeed = succeed
        try:
            self.assertEqual(bridge._goal(handle.request), 'ACCEPT')
            with patch.object(self.wiring.bridge, 'run_observation_worker',
                              lambda value, **kwargs: action_result(value['request_id'])), \
                    patch.object(self.wiring.bridge, 'action_result', convert):
                response = bridge._execute(handle)
            self.assertTrue(response.success)
            self.assertEqual(phases, ['conversion', 'terminal'])
            self.assertFalse(bridge._busy)
        finally:
            bridge.destroy_node()

    def test_internal_failure_statuses_are_normalized_on_the_action_wire(self):
        for internal_status in ('HOLD', 'UNAVAILABLE', 'INVALID_RESULT'):
            with self.subTest(internal_status=internal_status):
                request_id = 'request-' + internal_status
                self.assertTrue(self.arm().success)
                goal = self.goal(request_id=request_id)
                self.assertEqual(self.node._goal(goal), 'ACCEPT')
                self.assertTrue(self.approve().success)

                def failed(value, **kwargs):
                    response = command_result(value['request_id'])
                    response.update(ok=False, error='Review remains held')
                    response['metrics'].update(success=False, status=internal_status)
                    return response

                self.node._session.handler = failed
                handle = FakeHandle(goal)
                response = self.node._execute(handle)
                self.assertFalse(response.success)
                self.assertEqual(response.status, 'FAILED')
                self.assertEqual(response.request_id, request_id)
                self.assertEqual(response.evaluation_id, '')
                self.assertTrue(response.requires_review)
                self.assertFalse(response.fabrication_authorized)
                self.assertEqual(handle.terminals, ['ABORTED'])

    def test_journal_fault_publishes_hold_and_rejects_arming(self):
        self.node._supervisor._faulted = True
        response = self.arm()
        self.assertFalse(response.success)
        self.assertEqual(response.state, 'HOLD')
        self.node._publish_state()
        published = json.loads(self.node._status.messages[-1].data)
        self.assertEqual(published['state'], 'HOLD')
        self.assertFalse(published['fabrication_authorized'])
        self.assertFalse(published['hardware_execution_available'])
        self.assertEqual(self.node._goal(self.goal()), 'REJECT')
        self.assertEqual(self.node._session.calls, [])

    def test_journal_fault_rejects_approval_without_releasing_waiter(self):
        self.accepted()
        self.node._supervisor._faulted = True
        response = self.approve()
        self.assertFalse(response.success)
        self.assertEqual(response.state, 'HOLD')
        self.assertFalse(self.node._approval_event.is_set())
        self.assertEqual(self.node._session.calls, [])

    def test_invalid_denial_hash_does_not_interrupt_current_review(self):
        handle = self.accepted()
        self.assertFalse(self.approve('f' * 64, approve=False).success)
        self.assertFalse(self.node._approval_event.is_set())
        self.assertFalse(self.node._cancel_event.is_set())
        self.assertIsNone(self.node._interruption)
        self.assertTrue(self.approve().success)
        self.assertTrue(self.node._execute(handle).success)

    def test_valid_denial_prevents_dispatch_and_reports_cancellation(self):
        handle = self.accepted()
        self.assertTrue(self.approve(approve=False).success)
        response = self.node._execute(handle)
        self.assertFalse(response.success)
        self.assertEqual(response.status, 'CANCELLED')
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertEqual(self.node._session.calls, [])

    def test_approval_deadline_is_timeout_not_cancellation(self):
        handle = self.accepted()
        with patch.object(self.wiring.coordinator.time, 'monotonic', side_effect=[100, 103]):
            response = self.node._execute(handle)
        self.assertFalse(response.success)
        self.assertEqual(response.status, 'TIMED_OUT')
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertEqual(self.node._session.calls, [])
        self.assertIsNone(self.node._interruption)

    def test_coordinator_timeout_exception_remains_timeout(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        with patch.object(self.node._supervisor, 'execute', side_effect=TimeoutError('deadline')):
            response = self.node._execute(handle)
        self.assertFalse(response.success)
        self.assertEqual(response.status, 'TIMED_OUT')
        self.assertTrue(self.node._cancel_event.is_set())
        self.assertIsNone(self.node._interruption)
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertEqual(self.node._session.calls, [])

    def test_coordinator_unexpected_exception_remains_failed(self):
        handle = self.accepted()
        self.assertTrue(self.approve().success)
        with patch.object(self.node._supervisor, 'execute', side_effect=RuntimeError('failure')):
            response = self.node._execute(handle)
        self.assertFalse(response.success)
        self.assertEqual(response.status, 'FAILED')
        self.assertTrue(self.node._cancel_event.is_set())
        self.assertIsNone(self.node._interruption)
        self.assertEqual(handle.terminals, ['ABORTED'])
        self.assertEqual(self.node._session.calls, [])

    def test_bridge_local_shutdown_aborts_without_invalid_cancel_transition(self):
        bridge = self.wiring.bridge.ObservationBridge()
        handle = FakeHandle(self.goal())
        try:
            self.assertEqual(bridge._goal(handle.request), 'ACCEPT')

            def late_shutdown(value, **kwargs):
                bridge._shutdown.set()
                return action_result(value['request_id'])

            with patch.object(self.wiring.bridge, 'run_observation_worker', late_shutdown):
                response = bridge._execute(handle)
            self.assertFalse(response.success)
            self.assertEqual(response.status, 'CANCELLED')
            self.assertFalse(handle.is_cancel_requested)
            self.assertEqual(handle.terminals, ['ABORTED'])
        finally:
            bridge.destroy_node()


if __name__ == '__main__':
    unittest.main()
