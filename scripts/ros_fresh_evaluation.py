"""Offline fresh RGB-D -> DDS -> cap/pair/shell analysis through real ROS.

Runs no robot, inventory model or external service. The explicit demonstration
flag provides software approvals, not human authorization. Native workcell use
must use the normal ROS review interface and a human approval client.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
import traceback
from pathlib import Path

from ros_review_multiprocess import (
    ReviewClient,
    RuntimeProcess,
    check,
    eventually,
    read_journal,
    require_isolation,
    start_count,
)


def run(args):
    check(args.offline_demonstration_approval, 'Explicit software demonstration approval required')
    require_isolation()
    for path in (args.agent007_root, args.dds_root, args.dataset, args.profile, args.worker_python):
        check(path.is_absolute() and path.exists(), 'Explicit existing absolute input paths required')
    check(args.output.is_absolute() and not args.output.exists(), 'Output must be a new absolute directory')
    check(30 <= args.timeout_s <= 1800, 'Scenario timeout must be in [30, 1800] seconds')
    cycles = args.candidate_cycle or [4, 5]
    check(cycles == list(range(cycles[0], cycles[0] + len(cycles))) and cycles[0] >= 0,
          'Candidate cycles must be nonnegative and consecutive')
    sys.path[:0] = [str(args.agent007_root / 'src'), str(args.agent007_root)]
    from ocsr.printed_artifacts.integrated_evaluation import (
        file_digest,
        load_evaluation,
    )
    from ocsr.printed_artifacts.ros_fresh_analysis import prepare_input

    import yaml

    args.output.mkdir(mode=0o700)
    submitted, jobs, session = (args.output / name for name in ('submitted', 'jobs', 'session'))
    for path in (submitted, jobs, session):
        path.mkdir()
    journal = session / 'supervised_agent/review_journal.jsonl'
    processes = []
    started = time.monotonic()
    report = {
        'schema': 'behav3d.fresh_ros_validation.v1', 'passed': False,
        'client_pid': os.getpid(), 'separate_runtime_processes': True,
        'software_test_approvals_only': True, 'fresh_analysis_performed': True,
        'external_model_calls_sent': False, 'robot_commands_sent': False,
        'fabrication_authorized': False, 'physical_accuracy_validated': False,
        'cases': [], 'cycles': [], 'dataset': str(args.dataset),
    }
    previous_handler = signal.getsignal(signal.SIGALRM)

    def expired(*_):
        raise TimeoutError('Fresh ROS validation exceeded the whole-scenario deadline')

    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, args.timeout_s)

    def record(name, **values):
        row = dict(name=name, passed=True, **values)
        report['cases'].append(row)
        print(json.dumps(row, sort_keys=True), flush=True)

    try:
        bridge = RuntimeProcess('fresh-bridge', 'observation_bridge', {
            'backend_mode': 'fresh_geometry', 'evidence_root': str(submitted),
            'agent007_root': str(args.agent007_root), 'dds_root': str(args.dds_root),
            'analysis_output_root': str(jobs), 'worker_python': str(args.worker_python),
            'worker_timeout_s': float(args.timeout_s - 15),
        }, args.output / 'logs', timeout_s=args.timeout_s + 20).start()
        processes.append(bridge)
        coordinator = RuntimeProcess('fresh-coordinator', 'supervised_agent_loop', {
            'session_path': str(session), 'approval_timeout_s': 30.0,
            'observation_timeout_s': float(args.timeout_s - 10),
        }, args.output / 'logs', timeout_s=args.timeout_s + 20).start()
        processes.append(coordinator)
        previous = None
        with ReviewClient(timeout_s=5.0) as client:
            client.ready()
            check(len({os.getpid(), bridge.pid, coordinator.pid}) == 3, 'Runtime processes not separate')
            for cycle in cycles:
                cycle_root = args.dataset / f'field_loop/cycle_{cycle:04d}'
                targets = cycle_root / 'candidates/targets.yaml'
                selected = yaml.safe_load(targets.read_text())['targets']
                check(isinstance(selected, list) and bool(selected), 'Explicit flat issued target list required')
                folder = submitted / f'cycle_{cycle + 1:04d}'
                manifest = prepare_input(folder,
                    scan=args.dataset / f'field_loop/cycle_{cycle + 1:04d}/scan',
                    baseline_scan=cycle_root / 'scan', calibration=args.dataset / 'config',
                    targets=targets, profile_path=args.profile,
                    pipeline_config=Path(__file__).resolve().parents[1] /
                        'src/behav3d_orchestrator/config/fresh_observation_pipeline.example.yaml',
                    dds_root=args.dds_root, artifact_id='fresh-recorded-artifact',
                    observation_id=f'fresh-recorded-scan-{cycle + 1}', candidate_cycle=cycle,
                    frame_id='world/base', expected_target_count=len(selected),
                    previous_job_directory=previous)
                request = dict(manifest['binding'], event_type='deposition_readback',
                    source_manifest_path=str(folder / 'manifest.json'),
                    source_manifest_sha256=file_digest(folder / 'manifest.json'))
                request['request_id'] = 'fresh-' + request['source_manifest_sha256'][:24]
                if previous is None:
                    check(not client.send(request).accepted, 'Fresh work accepted while disarmed')
                    check(not list(jobs.iterdir()), 'Fresh analysis ran before arming and approval')
                    record('default_disarmed_no_analysis')
                    check(client.arm().success, 'Explicit review arm failed')
                handle = client.send(request)
                check(handle.accepted, 'Fresh observation rejected')
                client.pending(request['request_id'])
                before_jobs = set(jobs.iterdir())
                check(not client.approve(request['request_id'], digest='0' * 64).success,
                      'Wrong exact-proposal hash was accepted')
                check(set(jobs.iterdir()) == before_jobs, 'Analysis ran without exact approval')
                check(client.approve(request['request_id']).success, 'Exact software approval rejected')
                result = client.finish(handle, timeout=args.timeout_s)
                product = Path(result.evidence_manifest_path).parent
                evaluation = load_evaluation(product)
                check(evaluation['context']['input_mode'] == 'fresh_raw_depth_scan_job',
                      'Saved replay was presented as newly generated geometry')
                check(evaluation['scan_job_id'] == request['observation_id']
                      and evaluation['scan_cycle'] == cycle + 1, 'Fresh output binding changed')
                previous = product.parent
                check((previous / 'dds/observed/dds_tsdf_export.npz').is_file(), 'Fresh TSDF missing')
                check((previous / 'dds/comparison/target_local_metrics.json').is_file(), 'Fresh DDS evidence missing')
                entries = read_journal(journal)
                check(start_count(entries, request['request_id']) == 1, 'Fresh request dispatched more than once')
                check(entries[-1]['data']['heads'][request['artifact_id']]['evaluation_id']
                      == result.evaluation_id, 'Fresh result did not become the accepted review head')
                before = start_count(entries)
                retry = client.send(request)
                check(retry.accepted, 'Exact completed fresh retry rejected')
                cached = client.finish(retry)
                check(cached.evaluation_id == result.evaluation_id and cached.packet_sha256 == result.packet_sha256,
                      'Fresh cached receipt changed')
                check(start_count(read_journal(journal)) == before and len(set(jobs.iterdir()) - before_jobs) == 1,
                      'Fresh retry recomputed the geometry')
                report['cycles'].append({
                    'request': request, 'evaluation_id': result.evaluation_id,
                    'evaluation_manifest': result.evidence_manifest_path,
                    'summary': evaluation['summary'], 'suggestions': evaluation['suggestions'],
                    'input_mode': evaluation['context']['input_mode'],
                })
                record('fresh_rgbd_dds_caps_pair_shell_and_cached_retry', scan_cycle=cycle + 1,
                       candidate_count=len(selected), evaluation_id=result.evaluation_id)
            check(client.arm(False).success, 'Final disarm failed')
            eventually(lambda: client.status()['state'] == 'DISARMED')
            report.update(passed=True, final_state='DISARMED',
                          worker_dispatches=start_count(read_journal(journal)))
    except BaseException:
        report['error'] = traceback.format_exc(limit=15)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        for process in reversed(processes):
            try:
                process.stop()
            except Exception as exc:  # noqa: BLE001 - always attempt remaining child cleanup.
                report.setdefault('cleanup_errors', []).append(repr(exc))
                report['passed'] = False
        report['processes'] = [process.report() for process in processes]
        if any(not row['exit_matches_expectation'] or row['forced_kill'] or row['traceback_detected']
               or row['logs_truncated'] or row['deadline_expired'] or not row['reader_threads_stopped']
               for row in report['processes']):
            report['passed'] = False
        report['elapsed_s'] = round(time.monotonic() - started, 3)
        (args.output / 'report.json').write_text(json.dumps(report, sort_keys=True, indent=2) + '\n')
    check(report['passed'], 'Fresh ROS validation cleanup failed; inspect report.json')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('agent007-root', 'dds-root', 'dataset', 'profile', 'output', 'worker-python'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--candidate-cycle', type=int, action='append')
    parser.add_argument('--timeout-s', type=float, default=900.0)
    parser.add_argument('--offline-demonstration-approval', action='store_true')
    report = run(parser.parse_args())
    print(json.dumps({'passed': report['passed'], 'cycles': len(report['cycles']),
                      'worker_dispatches': report['worker_dispatches']}, sort_keys=True))


if __name__ == '__main__':
    main()
