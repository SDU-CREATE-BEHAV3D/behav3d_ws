"""Replay consecutive saved evaluations through separate ROS review processes.

This is a no-device software protocol experiment, not fresh scan analysis. The
explicit offline approval flag is NOT human approval and never authorizes robot
operation. Evidence roots are read only; output is a new, separate directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import sys
import time
import traceback
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import supervised_review_demo as demo
from ros_review_multiprocess import (
    ReviewClient,
    RuntimeProcess,
    check,
    eventually,
    read_journal,
    require_isolation,
    start_count,
)


def load_recorded_sequence(manifests, evidence_root):
    """Load bounded metadata in supplied order without trusting it as analysis.

    A single shared coordinator head is meaningful only for one artifact with
    consecutive cycles and exact content-addressed predecessor identities.
    The isolated worker still verifies every bundle and underlying source.
    """
    if len(manifests) < 2:
        raise ValueError('At least two explicit consecutive manifests are required')
    root = demo._absolute(evidence_root, directory=True)
    sequence, seen = [], set()
    for manifest in manifests:
        request = demo.request_from_bundle(manifest, root)
        request['request_id'] = 'recorded-ros-' + request['source_manifest_sha256'][:24]
        path = Path(request['source_manifest_path'])
        packet, checksum = demo._read_json(path.parent / 'robot_review_packet.json')
        saved, _ = demo._read_json(path)
        if set(saved['files']) != {'evaluation.json', 'shell.json', 'robot_review_packet.json'}:
            raise ValueError('Exact saved evaluation bundle files required')
        identity = packet.get('evaluation_id')
        if (not isinstance(identity, str)
                or re.fullmatch(r'evaluation_[0-9a-f]{64}', identity) is None):
            raise ValueError('Full content-addressed evaluation identity required')
        if request['request_id'] in seen:
            raise ValueError('Duplicate manifests cannot establish consecutive progress')
        if sequence:
            previous = sequence[-1]
            if (request['artifact_id'] != previous['request']['artifact_id']
                    or request['frame_id'] != previous['request']['frame_id']
                    or request['scan_cycle'] != previous['request']['scan_cycle'] + 1
                    or request['candidate_cycle'] != previous['request']['candidate_cycle'] + 1
                    or request['previous_evaluation_id'] != previous['evaluation_id']):
                raise ValueError('Manifests must be one exact consecutive artifact chain')
        seen.add(request['request_id'])
        sequence.append({
            'request': request, 'evaluation_id': identity,
            'packet_path': str(path.parent / 'robot_review_packet.json'),
            'packet_sha256': checksum,
            'bundle_files': dict(saved['files']),
        })
    return sequence


def result_fields(result):
    """Detach only the generated typed result fields into report JSON."""
    from behav3d_orchestrator.supervised_agent.observation import RESULT_FIELDS
    if isinstance(result, dict):
        return {field: result[field] for field in RESULT_FIELDS}
    return {field: getattr(result, field) for field in RESULT_FIELDS}


def check_saved_result(result, item):
    value = result_fields(result)
    request = item['request']
    expected = {
        'success': True, 'status': 'REVIEW_REQUIRED', 'requires_review': True,
        'fabrication_authorized': False, 'request_id': request['request_id'],
        'evaluation_id': item['evaluation_id'], 'packet_path': item['packet_path'],
        'packet_sha256': item['packet_sha256'],
        'evidence_manifest_path': request['source_manifest_path'],
        'evidence_manifest_sha256': request['source_manifest_sha256'],
    }
    check(all(value[key] == expected_value for key, expected_value in expected.items()),
          'Typed result did not bind the exact saved evaluation, packet and manifest')
    return value


def bundle_hashes(sequence):
    """Snapshot bundle metadata only; workers verify all underlying source files."""
    result = {}
    for item in sequence:
        manifest = Path(item['request']['source_manifest_path'])
        for name in ('manifest.json', *item['bundle_files']):
            path = manifest.parent / name
            _, result[str(path)] = demo._read_json(path)
    return result


def process_exit_ok(record):
    """All replay children must shut down cleanly; this scenario crashes none."""
    return (record.get('returncode') == 0
            and record.get('logs_truncated') is False
            and record.get('deadline_expired') is False
            and record.get('reader_threads_stopped') is True)


def _write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def _latest_state(client, expected):
    eventually(lambda: client.status() is not None
               and client.status()['state'] == expected,
               description='recorded replay coordinator state ' + expected)


def run(*, manifests, evidence_root, source_roots, output,
        offline_demonstration_approval=False, timeout_s=150.0):
    if offline_demonstration_approval is not True:
        raise ValueError('Explicit --offline-demonstration-approval is required')
    if not 30 <= timeout_s <= 150:
        raise ValueError('Replay scenario timeout must be between 30 and 150 seconds')
    require_isolation()
    root = demo._absolute(evidence_root, directory=True)
    sources = tuple(demo._absolute(path, directory=True) for path in source_roots)
    sequence = load_recorded_sequence(manifests, root)
    destination = Path(output)
    if not destination.is_absolute():
        raise ValueError('Output must be an explicit absolute new directory')
    destination = destination.resolve(strict=False)
    if any(demo._inside(destination, source) for source in (root, *sources)):
        raise ValueError('Output must be outside all read-only input roots')
    destination.mkdir(mode=0o700, parents=False, exist_ok=False)
    logs = destination / 'process-logs'
    logs.mkdir()
    session = destination / 'session'
    session.mkdir()
    negative_session = destination / 'negative-session'
    negative_session.mkdir()
    journal = session / 'supervised_agent/review_journal.jsonl'
    negative_journal = negative_session / 'supervised_agent/review_journal.jsonl'
    before = bundle_hashes(sequence)
    deadline = time.monotonic() + timeout_s
    report = {
        'schema': 'behav3d.recorded_cycle_ros_replay.v1', 'passed': False,
        'scope': 'Consecutive SAVED evaluation verification, not new scan analysis.',
        'approval_label': 'Programmatic offline protocol test, NOT human approval.',
        'offline_demonstration_approval': True, 'real_rclpy': True,
        'separate_runtime_processes': True, 'generated_interfaces': True,
        'client_pid': os.getpid(), 'runtime_nodes_in_client': False,
        'synthetic_evidence': False, 'fresh_analysis_performed': False,
        'model_calls_sent': False, 'robot_commands_sent': False,
        'fabrication_authorized': False, 'physical_accuracy_validated': False,
        'structural_evaluation': 'not_performed',
        'network_scope': 'isolated_container_loopback_dds',
        'replay_start_scan': sequence[0]['request']['scan_cycle'],
        'initial_previous_evaluation_id': sequence[0]['request']['previous_evaluation_id'],
        'prior_history_replayed_in_this_session': False,
        'cases': [], 'reviews': [], 'processes': [],
        'domain': os.environ.get('ROS_DOMAIN_ID'),
    }
    processes, client = [], None
    coordinator = None
    previous_handler = signal.getsignal(signal.SIGALRM)

    def expired(*_):
        raise TimeoutError('Overall recorded ROS replay deadline exceeded')

    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, timeout_s)

    def record(name, **values):
        check(time.monotonic() < deadline, 'Recorded replay exceeded its total deadline')
        item = {'name': name, 'passed': True, **values}
        report['cases'].append(item)
        print(json.dumps(item, sort_keys=True), flush=True)

    def arm(armed):
        response = client.arm(armed)
        check(response.success, 'Offline review arm/disarm service rejected: ' + response.message)
        return response

    def start_coordinator(name, capture_session):
        process = RuntimeProcess(name, 'supervised_agent_loop', {
            'session_path': str(capture_session), 'approval_timeout_s': 12.0,
            'observation_timeout_s': 55.0,
        }, logs, timeout_s=timeout_s)
        processes.append(process)
        return process.start()

    def reject_without_dispatch(request, name):
        count = start_count(read_journal(journal))
        handle = client.send(request)
        check(not handle.accepted, name + ' unexpectedly accepted')
        check(start_count(read_journal(journal)) == count,
              name + ' unexpectedly dispatched a worker')
        record(name, request_id=request['request_id'], worker_dispatched=False)

    try:
        bridge = RuntimeProcess('recorded-observation-bridge', 'observation_bridge', {
            'evidence_root': str(root), 'source_roots': [str(path) for path in sources],
            'worker_python': sys.executable, 'worker_timeout_s': 45.0,
        }, logs, timeout_s=timeout_s)
        processes.append(bridge)
        bridge.start()
        coordinator = start_coordinator('recorded-coordinator-1', session)
        client = ReviewClient(timeout_s=5.0)
        client.__enter__()
        client.ready()
        _latest_state(client, 'DISARMED')
        check(len({os.getpid(), bridge.pid, coordinator.pid}) == 3,
              'Client, bridge and coordinator are not separate OS processes')
        reject_without_dispatch(sequence[0]['request'], 'default_disarmed_rejection')
        arm(True)
        for item in sequence:
            request = item['request']
            handle = client.send(request)
            check(handle.accepted, 'Consecutive saved observation was rejected')
            proposal = client.pending(request['request_id'])
            check(client.approve(request['request_id']).success,
                  'Exact offline demonstration approval rejected')
            result = check_saved_result(client.finish(handle, timeout=60), item)
            entries = read_journal(journal)
            expected_head = {'evaluation_id': item['evaluation_id'],
                             'scan_cycle': request['scan_cycle']}
            check(entries[-1]['data']['heads'][request['artifact_id']] == expected_head,
                  'Shared artifact head did not advance to the accepted evaluation')
            check(start_count(entries, request['request_id']) == 1,
                  'Observation was evaluated more than once')
            report['reviews'].append({'request': request,
                                      'proposal_sha256': proposal['proposal_sha256'],
                                      'result': result, 'head': expected_head})
            record('consecutive_recorded_cycle', scan_cycle=request['scan_cycle'],
                   candidate_cycle=request['candidate_cycle'], evaluation_id=item['evaluation_id'])
        check(start_count(read_journal(journal)) == len(sequence),
              'Consecutive run has an unexpected dispatch count')
        first, last = sequence[0], sequence[-1]
        count = start_count(read_journal(journal))
        duplicate = client.send(first['request'])
        check(duplicate.accepted, 'Exact completed retry rejected after later cycles')
        check_saved_result(client.finish(duplicate), first)
        check(start_count(read_journal(journal)) == count,
              'Duplicate before restart invoked another worker')
        record('cached_retry_after_later_cycles_without_redispatch')

        # New IDs are not cached retries: stale order and predecessor conflicts
        # must be rejected before any worker starts.
        stale = dict(first['request'], request_id='recorded-ros-stale-order')
        reject_without_dispatch(stale, 'stale_cycle_order_rejection')
        wrong_previous = dict(last['request'], request_id='recorded-ros-wrong-previous',
                              candidate_cycle=last['request']['candidate_cycle'] + 1,
                              scan_cycle=last['request']['scan_cycle'] + 1,
                              previous_evaluation_id='evaluation_' + '0' * 64)
        reject_without_dispatch(wrong_previous, 'previous_evaluation_conflict_rejection')
        conflict = dict(last['request'], source_manifest_sha256='0' * 64)
        reject_without_dispatch(conflict, 'same_request_source_hash_conflict_rejection')
        arm(False)
        coordinator.stop()
        client.new_generation()
        coordinator = start_coordinator('recorded-coordinator-2', session)
        client.ready()
        _latest_state(client, 'DISARMED')
        duplicate = client.send(last['request'])
        check(duplicate.accepted, 'Completed retry rejected after process restart')
        check_saved_result(client.finish(duplicate), last)
        entries = read_journal(journal)
        check(start_count(entries) == count, 'Restart retry invoked another worker')
        check(entries[-1]['data']['heads'][last['request']['artifact_id']] ==
              report['reviews'][-1]['head'], 'Restart changed the shared artifact head')
        record('restart_disarmed_cached_retry_without_redispatch')
        arm(False)
        coordinator.stop()

        # Negative evidence tests get an independent journal. They intentionally
        # do not alter any evidence bytes or poison the successful artifact head.
        client.new_generation()
        coordinator = start_coordinator('recorded-negative-coordinator', negative_session)
        client.ready()
        _latest_state(client, 'DISARMED')
        failures = (
            ('wrong_frame_backend_rejection', dict(first['request'],
                request_id='recorded-ros-wrong-frame', frame_id='wrong_frame_for_protocol_test')),
            ('wrong_manifest_hash_backend_rejection', dict(first['request'],
                request_id='recorded-ros-wrong-source-hash', source_manifest_sha256='0' * 64)),
        )
        for name, request in failures:
            arm(True)
            handle = client.send(request)
            check(handle.accepted, 'Negative binding test never reached approval boundary')
            check(client.approve(request['request_id']).success,
                  'Negative binding test approval rejected')
            result = result_fields(client.finish(handle, status='STALE_SOURCE', timeout=60))
            _latest_state(client, 'DISARMED')
            entries = read_journal(negative_journal)
            check(not entries[-1]['data']['heads'], 'Failed binding advanced the artifact head')
            check(start_count(entries, request['request_id']) == 1,
                  'Negative binding did not exercise the actual worker exactly once')
            check(not result['success'] and not result['evaluation_id'],
                  'Invalid binding was promoted into an evaluation identity')
            record(name, status=result['status'], head_advanced=False, final_state='DISARMED')
        arm(False)
        _latest_state(client, 'DISARMED')
        report.update(passed=True, final_state='DISARMED',
                      saved_cycles_verified=len(sequence),
                      main_worker_dispatches=start_count(read_journal(journal)),
                      negative_worker_dispatches=start_count(read_journal(negative_journal)))
    except BaseException as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        report['traceback'] = traceback.format_exc()
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        if client is not None:
            try:
                client.arm(False)
            except Exception as exc:  # noqa: BLE001 - finish cleanup after ROS transport failure.
                report.setdefault('cleanup_errors', []).append(type(exc).__name__)
                report['passed'] = False
            try:
                client.__exit__(None, None, None)
            except Exception as exc:  # noqa: BLE001 - always stop owned runtime processes.
                report.setdefault('cleanup_errors', []).append(type(exc).__name__)
                report['passed'] = False
        for process in reversed(processes):
            try:
                process.stop()
            except Exception as exc:  # noqa: BLE001 - one failed cleanup cannot skip other processes.
                report.setdefault('cleanup_errors', []).append(type(exc).__name__)
                report['passed'] = False
            report['processes'].append(process.report())
        report['all_child_processes_cleanly_stopped'] = all(
            process_exit_ok(item) for item in report['processes'])
        if not report['all_child_processes_cleanly_stopped']:
            report['passed'] = False
        try:
            report['input_bundle_metadata_unchanged'] = bundle_hashes(sequence) == before
            if not report['input_bundle_metadata_unchanged']:
                report['passed'] = False
        except (OSError, ValueError, KeyError):
            report['input_bundle_metadata_unchanged'] = False
            report['passed'] = False
        report['elapsed_s'] = round(timeout_s - (deadline - time.monotonic()), 3)
        _write(destination / 'requests.json', [item['request'] for item in sequence])
        _write(destination / 'report.json', report)
        _write(destination / 'manifest.json', {
            'schema': 'behav3d.recorded_cycle_ros_replay_bundle.v1',
            'files': {str(path.relative_to(destination)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in destination.rglob('*') if path.is_file()},
            'fabrication_authorized': False,
        })
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, action='append', required=True,
                        help='Repeat in consecutive scan order; explicit absolute manifest paths')
    parser.add_argument('--evidence-root', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=150.0)
    parser.add_argument('--offline-demonstration-approval', action='store_true')
    args = parser.parse_args(argv)
    try:
        report = run(manifests=args.manifest, evidence_root=args.evidence_root,
                     source_roots=args.source_root, output=args.output, timeout_s=args.timeout_s,
                     offline_demonstration_approval=args.offline_demonstration_approval)
    except (OSError, ValueError, KeyError, RuntimeError, AssertionError):
        parser.exit(2, 'Recorded ROS replay failed closed; inspect the retained report and logs.\n')
    print(json.dumps({key: report[key] for key in (
        'passed', 'saved_cycles_verified', 'main_worker_dispatches',
        'negative_worker_dispatches', 'final_state')}, sort_keys=True))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
