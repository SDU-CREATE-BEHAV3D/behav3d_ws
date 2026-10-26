"""Real ROS 2, headless review smoke test for an isolated no-device container.

This is not a robot test. It starts only the two review nodes and a test client,
uses generated rosidl interfaces, DDS loopback, the real queue and subprocess
worker, and synthetic immutable evidence. No ROS mocks or model calls are used.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback


def require_isolation():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if (os.environ.get('BEHAV3D_REVIEW_CONTAINER') != '1'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or not domain.isdigit() or not 1 <= int(domain) <= 232
            or not (Path('/.dockerenv').exists() or Path('/run/.containerenv').exists())):
        raise RuntimeError(
            'Run only in the no-device review container with BEHAV3D_REVIEW_CONTAINER=1, '
            'ROS_LOCALHOST_ONLY=1 and an explicit isolated nonzero ROS_DOMAIN_ID.')


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def eventually(predicate, timeout=10.0, description='condition'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise TimeoutError('Timed out waiting for ' + description)


def future_value(future, timeout=15.0):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise TimeoutError('ROS future exceeded headless test deadline')
    return future.result()


def load_fixture(repo):
    path = repo / 'src/behav3d_orchestrator/test/test_observation_worker.py'
    spec = importlib.util.spec_from_file_location('review_smoke_evidence_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fixture = module.ObservationTests('test_exact_success_contract_and_no_source_mutation')
    fixture.setUp()
    return fixture


def run(output, repo):
    require_isolation()
    import rclpy
    from action_msgs.msg import GoalStatus
    from behav3d_interfaces.action import EvaluateObservation
    from behav3d_interfaces.srv import ApproveSupervisedStep, SetSupervisorArmed
    from behav3d_orchestrator.observation_bridge import ObservationBridge
    from behav3d_orchestrator.supervised_agent_loop import SupervisedAgentLoop
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import String

    fixture = load_fixture(repo)
    report = {
        'schema': 'behav3d.real_ros_review_smoke.v1', 'passed': False,
        'real_rclpy': True, 'generated_interfaces': True, 'synthetic_evidence': True,
        'physical_accuracy_validated': False, 'robot_commands_sent': False,
        'network_scope': 'isolated_container_loopback_dds',
        'ros_distro': os.environ.get('ROS_DISTRO'), 'domain': os.environ['ROS_DOMAIN_ID'],
        'python': sys.version, 'cases': [],
    }
    executor = None
    thread = None
    nodes = []
    coordinator = None
    spin_errors = []
    started = time.monotonic()

    def record(name, **values):
        report['cases'].append({'name': name, 'passed': True, **values})
        print(json.dumps(report['cases'][-1], sort_keys=True), flush=True)

    try:
        # CLI parameters apply only to these freshly-created test nodes.
        session = fixture.root / 'session'
        session.mkdir()
        rclpy.init(args=[
            '--ros-args', '-p', 'session_path:=' + str(session),
            '-p', 'evidence_root:=' + str(fixture.root),
            '-p', 'source_roots:=[' + str(fixture.root) + ']',
            '-p', 'worker_python:=' + sys.executable,
            '-p', 'worker_timeout_s:=10.0',
            '-p', 'approval_timeout_s:=3.0',
            '-p', 'observation_timeout_s:=15.0',
        ])
        bridge = ObservationBridge()
        nodes.append(bridge)
        coordinator = SupervisedAgentLoop()
        nodes.append(coordinator)
        client = Node('behav3d_review_smoke_client')
        nodes.append(client)
        executor = MultiThreadedExecutor(num_threads=8)
        for node in nodes:
            executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except BaseException as exc:
                spin_errors.append(repr(exc))

        thread = threading.Thread(target=spin, name='real-ros-review-executor', daemon=True)
        thread.start()
        action = ActionClient(client, EvaluateObservation,
                              '/behav3d/supervised_agent/review_observation')
        arm_client = client.create_client(SetSupervisorArmed,
                                          '/behav3d/supervised_agent/set_armed')
        approval_client = client.create_client(ApproveSupervisedStep,
                                               '/behav3d/supervised_agent/approve_step')
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        statuses = []
        status_lock = threading.Lock()

        def on_status(message):
            value = json.loads(message.data)
            with status_lock:
                statuses.append(value)

        client.create_subscription(
            String, '/behav3d/supervised_agent/status', on_status, qos)
        pause_publisher = client.create_publisher(String, '/behav3d/control_state', qos)
        check(action.wait_for_server(timeout_sec=10), 'Coordinator action not discoverable')
        check(arm_client.wait_for_service(timeout_sec=10), 'Arm service not discoverable')
        check(approval_client.wait_for_service(timeout_sec=10), 'Approval service not discoverable')
        eventually(lambda: bool(statuses), description='transient-local status')
        check(coordinator._session.motion is None and coordinator._session.camera is None
              and coordinator._session.extruder is None, 'Physical command set constructed')
        check(set(coordinator._session._router._handlers) == {'evaluate_observation'},
              'Physical handler registered')

        def goal_values(request_id, artifact=None):
            value = dict(fixture.request, request_id=request_id)
            if artifact is not None:
                value['artifact_id'] = artifact
            return value

        def send(values):
            goal = EvaluateObservation.Goal()
            for key, value in values.items():
                setattr(goal, key, value)
            return future_value(action.send_goal_async(goal))

        def arm(armed=True):
            value = SetSupervisorArmed.Request()
            value.armed = armed
            value.operator_id = 'container-test-operator'
            response = future_value(arm_client.call_async(value))
            check(response.success, 'Arm/disarm rejected: ' + response.message)
            return response

        def pending(request_id):
            found = []

            def locate():
                with status_lock:
                    for value in reversed(statuses):
                        candidate = value.get('pending')
                        if candidate and candidate['request_id'] == request_id:
                            found.append(candidate)
                            return True
                return False

            eventually(locate, description='pending proposal status for ' + request_id)
            return found[-1]

        def approve(request_id, *, decision=True, digest=None):
            value = ApproveSupervisedStep.Request()
            value.request_id = request_id
            value.proposal_sha256 = pending(request_id)['proposal_sha256'] if digest is None else digest
            value.operator_id = 'container-test-operator'
            value.approve = decision
            return future_value(approval_client.call_async(value))

        def finish(handle, status, ros_status):
            wrapped = future_value(handle.get_result_async(), timeout=20)
            check(wrapped.status == ros_status, 'Unexpected real ROS goal terminal state')
            check(wrapped.result.status == status, 'Unexpected typed review status: ' + wrapped.result.status)
            check(wrapped.result.requires_review, 'Review gate omitted')
            check(not wrapped.result.fabrication_authorized, 'Fabrication authority leaked')
            return wrapped.result

        check(not send(goal_values('case-disarmed')).accepted, 'Disarmed goal accepted')
        record('default_disarmed_rejection')
        arm()
        malformed = goal_values('case-malformed')
        malformed['coordinate_units'] = 'mm'
        check(not send(malformed).accepted, 'Malformed goal accepted')
        record('typed_malformed_goal_rejection')

        values = goal_values('case-success')
        handle = send(values)
        check(handle.accepted, 'Armed valid goal rejected')
        pending('case-success')
        check(not approve('case-success', decision=False, digest='f' * 64).success,
              'Incorrect hash denial accepted')
        check(approve('case-success').success, 'Exact hash approval rejected')
        response = finish(handle, 'REVIEW_REQUIRED', GoalStatus.STATUS_SUCCEEDED)
        check(response.success, 'Real isolated worker did not verify synthetic bundle')
        check(response.evaluation_id == fixture.evaluation['evaluation_id'], 'Evaluation identity changed')
        record('approved_real_worker_review', evaluation_id=response.evaluation_id,
               packet_sha256=response.packet_sha256)
        journal = session / 'supervised_agent/review_journal.jsonl'

        def start_count():
            return sum(json.loads(line)['event'] == 'START' for line in journal.read_text().splitlines())

        before = start_count()
        duplicate = send(values)
        check(duplicate.accepted, 'Completed identical retry rejected')
        cached = finish(duplicate, 'REVIEW_REQUIRED', GoalStatus.STATUS_SUCCEEDED)
        check(cached.packet_sha256 == response.packet_sha256, 'Cached receipt changed')
        check(start_count() == before, 'Duplicate request dispatched another worker')
        record('duplicate_cached_retry_without_redispatch')

        cancel = send(goal_values('case-cancel', artifact='cancel-case'))
        check(cancel.accepted, 'Cancellation test goal rejected')
        pending('case-cancel')
        cancellation = future_value(cancel.cancel_goal_async())
        check(bool(cancellation.goals_canceling), 'Real ROS cancellation rejected')
        finish(cancel, 'CANCELLED', GoalStatus.STATUS_CANCELED)
        record('real_ros_client_cancellation_before_approval')

        arm()
        denied = send(goal_values('case-denied', artifact='denial-case'))
        check(denied.accepted, 'Denial test goal rejected')
        check(approve('case-denied', decision=False).success, 'Exact operator denial rejected')
        finish(denied, 'CANCELLED', GoalStatus.STATUS_ABORTED)
        record('operator_denial_is_aborted_not_invalid_cancel_transition')

        arm()
        timeout = send(goal_values('case-timeout', artifact='timeout-case'))
        check(timeout.accepted, 'Deadline test goal rejected')
        finish(timeout, 'TIMED_OUT', GoalStatus.STATUS_ABORTED)
        record('approval_deadline_without_dispatch')

        arm()
        eventually(lambda: pause_publisher.get_subscription_count() > 0,
                   description='shared pause subscription discovery')
        pause_publisher.publish(String(data='paused'))
        eventually(lambda: coordinator._session.control_pause._paused,
                   description='real DDS cooperative pause reception')
        disarmed = send(goal_values('case-disarmed-paused', artifact='paused-case'))
        check(disarmed.accepted, 'Paused review goal rejected')
        check(approve('case-disarmed-paused').success, 'Paused review approval rejected')
        eventually(lambda: coordinator._supervisor.snapshot().get('inflight_id')
                   == 'case-disarmed-paused', description='approved operation entering pause gate')
        arm(False)
        finish(disarmed, 'CANCELLED', GoalStatus.STATUS_ABORTED)
        receipt = coordinator._supervisor.receipt('case-disarmed-paused')
        check(receipt['state'] == 'HOLD' and not receipt['accepted'],
              'Late disarmed operation was durably accepted')
        record('disarm_during_real_shared_pause_holds_unaccepted_result')

        check(not spin_errors, 'Executor failed: ' + repr(spin_errors))
        report['journal_records'] = len(journal.read_text().splitlines())
        report['passed'] = True
    except BaseException:
        report['error'] = traceback.format_exc(limit=12)
        raise
    finally:
        if coordinator is not None:
            coordinator.begin_shutdown()
        if executor is not None:
            drained = executor.shutdown(timeout_sec=10.0)
            report['executor_drained'] = bool(drained)
        if thread is not None:
            thread.join(timeout=10)
            report['executor_thread_stopped'] = not thread.is_alive()
        for node in reversed(nodes):
            try:
                node.destroy_node()
            except Exception as exc:
                report.setdefault('cleanup_errors', []).append(repr(exc))
        if rclpy.ok():
            rclpy.shutdown()
        fixture.doCleanups()
        report['duration_s'] = round(time.monotonic() - started, 3)
        report['executor_errors'] = spin_errors
        if (report.get('cleanup_errors') or spin_errors
                or report.get('executor_drained') is False
                or report.get('executor_thread_stopped') is False):
            report['passed'] = False
        output.mkdir(parents=True, exist_ok=True)
        (output / 'summary.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    check(report['passed'], 'Real ROS smoke cleanup failed; inspect summary.json')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    result = run(args.output.resolve(), args.repo.resolve())
    print(json.dumps({'passed': result['passed'], 'case_count': len(result['cases']),
                      'report': str(args.output.resolve() / 'summary.json')}, sort_keys=True))


if __name__ == '__main__':
    main()
