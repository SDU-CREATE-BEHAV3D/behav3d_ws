#!/usr/bin/env python3
"""Reproduce the recorded-evidence approval boundary without ROS or hardware.

This verifies an existing Agent007 bundle, not a new scan or model analysis.
The explicit --offline-demonstration-approval flag performs software-only arm
and approval calls to exercise the protocol. It is NOT evidence that a human
approved a robot operation. No robot operation exists in this demonstration.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/behav3d_orchestrator'))
from behav3d_orchestrator.supervised_agent import Supervisor, SupervisorError
from behav3d_orchestrator.supervised_agent.observation import (
    MAX_JSON_BYTES, _decode, result_for_error, validate_request,
)
from behav3d_orchestrator.supervised_agent.process_runner import (
    WorkerError, run_observation_worker,
)


def _absolute(value, *, directory=False):
    path = Path(value)
    if not path.is_absolute():
        raise ValueError('Explicit absolute paths are required')
    path = path.resolve(strict=True)
    if directory and not path.is_dir():
        raise ValueError('Configured root must be an existing directory')
    return path


def _inside(path, root):
    return path == root or root in path.parents


def _read_json(path):
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0)
    descriptor = os.open(path, flags)
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise ValueError('Bundle metadata must be a regular file')
        with os.fdopen(descriptor, 'rb') as stream:
            descriptor = None
            raw = stream.read(MAX_JSON_BYTES + 1)
    finally:
        if descriptor is not None:
            os.close(descriptor)
    return _decode(raw, MAX_JSON_BYTES), hashlib.sha256(raw).hexdigest()


def request_from_bundle(manifest, evidence_root):
    """Read bounded metadata to construct a request; the worker verifies it."""
    root = _absolute(evidence_root, directory=True)
    manifest = _absolute(manifest)
    if manifest.name != 'manifest.json' or not _inside(manifest, root):
        raise ValueError('Manifest must be under the explicit evidence root')
    saved, checksum = _read_json(manifest)
    if (not isinstance(saved, dict) or saved.get('schema') != 'agent007.evaluation_bundle.v1'
            or not isinstance(saved.get('files'), dict)):
        raise ValueError('Expected an Agent007 evaluation bundle manifest')
    packet_path = (manifest.parent / 'robot_review_packet.json').resolve(strict=True)
    if packet_path.parent != manifest.parent or not _inside(packet_path, root):
        raise ValueError('Packet escaped its evaluation bundle')
    packet, packet_checksum = _read_json(packet_path)
    if saved['files'].get('robot_review_packet.json') != packet_checksum:
        raise ValueError('Packet does not match the supplied manifest')
    if not isinstance(packet, dict) or packet.get('schema') != 'agent007.robot_review_packet.v1':
        raise ValueError('Expected a recorded Agent007 robot review packet')
    request = {
        'request_id': 'offline-review-' + checksum[:24],
        'artifact_id': packet['artifact_id'], 'observation_id': packet['scan_job_id'],
        'event_type': 'deposition_readback', 'candidate_cycle': packet['candidate_cycle'],
        'scan_cycle': packet['scan_cycle'],
        'previous_evaluation_id': packet['previous_evaluation_id'] or '',
        'frame_id': packet['frame_id'], 'coordinate_units': packet['coordinate_units'],
        'source_manifest_path': str(manifest), 'source_manifest_sha256': checksum,
    }
    return validate_request(request)


def _write_json(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def run_demo(*, manifest, evidence_root, source_roots=(), output,
             worker_python=None, timeout_s=120.0, offline_demonstration_approval=False):
    """Verify saved evidence once, then prove exact retry does not reevaluate.

    Only explicit server policy roots are passed to the isolated stdlib worker.
    Output must be new and outside every read-only input root. The default run
    proves disarmed rejection and stops without dispatching any worker.
    """
    if type(offline_demonstration_approval) is not bool:
        raise ValueError('Explicit boolean offline demonstration flag required')
    root = _absolute(evidence_root, directory=True)
    sources = tuple(_absolute(path, directory=True) for path in source_roots)
    destination = Path(output)
    if not destination.is_absolute():
        raise ValueError('Output must be an explicit absolute new directory')
    destination = destination.resolve(strict=False)
    if any(_inside(destination, source) for source in (root, *sources)):
        raise ValueError('Output must be outside all read-only evidence/source roots')
    if destination.exists():
        raise FileExistsError('Refusing to reuse an existing output directory')
    request = request_from_bundle(manifest, root)
    python = _absolute(sys.executable if worker_python is None else worker_python)
    if not python.is_file():
        raise ValueError('Worker Python must be an explicit existing executable file')
    destination.mkdir(mode=0o700, parents=False, exist_ok=False)
    supervisor = Supervisor(destination / 'review_journal.jsonl')
    calls = 0
    report = {
        'schema': 'behav3d.offline_supervised_review_demo.v1',
        'scope': 'Recorded bundle verification only; not fresh analysis or an autonomous agent experiment.',
        'request': request, 'initial_state': supervisor.snapshot()['state'],
        'offline_demonstration_approval': offline_demonstration_approval,
        'approval_label': 'Programmatic offline protocol demonstration, NOT human or hardware approval.',
        'worker_invocations': 0, 'fresh_analysis_performed': False,
        'ros_used': False, 'model_calls_sent': False, 'robot_commands_sent': False,
        'fabrication_authorized': False, 'physical_accuracy_validated': False,
        'structural_evaluation': 'not_performed',
    }
    files = ['request.json', 'review_journal.jsonl', 'report.json']
    try:
        try:
            supervisor.propose('evaluate_observation', request)
        except SupervisorError:
            report['disarmed_proposal_rejected'] = True
        else:
            raise RuntimeError('Default-disarmed boundary was not enforced')
        _write_json(destination / 'request.json', request)
        if offline_demonstration_approval:
            supervisor.set_armed(True, 'offline-demonstration-not-human-approval')
            proposal = supervisor.propose('evaluate_observation', request)
            digest = proposal['proposal_sha256']
            supervisor.approve(request['request_id'], digest,
                               'offline-demonstration-not-human-approval')

            def operation():
                nonlocal calls
                calls += 1
                try:
                    observed = run_observation_worker(
                        request, python_executable=str(python), evidence_root=str(root),
                        source_roots=[str(path) for path in sources], timeout_s=timeout_s)
                except WorkerError as exc:
                    observed = result_for_error(request['request_id'], exc.status, str(exc))
                return {
                    'ok': observed['success'], 'kind': 'evaluate_observation',
                    'phase': 'plan', 'planned_only': True,
                    'error': None if observed['success'] else observed['message'],
                    'metrics': observed,
                }

            first = supervisor.execute(request['request_id'], digest, operation)

            def forbidden_repeat():
                raise RuntimeError('Cached retry unexpectedly attempted another worker')

            cached = supervisor.execute(request['request_id'], digest, forbidden_repeat)
            if first != cached or calls != 1:
                raise RuntimeError('Exact retry did not preserve its receipt')
            _write_json(destination / 'result.json', first)
            _write_json(destination / 'receipt.json', supervisor.receipt(request['request_id']))
            files.extend(['result.json', 'receipt.json'])
            report.update(proposal_sha256=digest, worker_invocations=calls,
                          cached_retry_equal=True, cached_retry_reverified_sources=False,
                          verified=first['ok'], worker_status=first['metrics'].get('status', 'FAILED'))
            supervisor.close()
            supervisor = Supervisor(destination / 'review_journal.jsonl')
            report['restart_state'] = supervisor.snapshot()['state']
            again = supervisor.execute(request['request_id'], digest, forbidden_repeat)
            if again != first or calls != 1:
                raise RuntimeError('Restart retry did not preserve its receipt')
            report['restart_cached_retry_equal'] = True
            report['evaluation_id'] = first['metrics'].get('evaluation_id', '')
        else:
            report.update(verified=False, worker_status='NOT_DISPATCHED',
                          reason='No offline demonstration approval flag: remain disarmed.')
        report['final_state'] = supervisor.snapshot()['state']
    finally:
        supervisor.close()
    _write_json(destination / 'report.json', report)
    _write_json(destination / 'manifest.json', {
        'schema': 'behav3d.offline_review_demo_bundle.v1',
        'files': {name: hashlib.sha256((destination / name).read_bytes()).hexdigest()
                  for name in files},
        'fabrication_authorized': False,
    })
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--evidence-root', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker-python', type=Path)
    parser.add_argument('--timeout-s', type=float, default=120.0)
    parser.add_argument('--offline-demonstration-approval', action='store_true',
                        help='Exercise software arm/approval only; never a human or robot approval')
    args = parser.parse_args(argv)
    try:
        report = run_demo(
            manifest=args.manifest, evidence_root=args.evidence_root,
            source_roots=args.source_root, output=args.output,
            worker_python=args.worker_python, timeout_s=args.timeout_s,
            offline_demonstration_approval=args.offline_demonstration_approval)
    except (OSError, ValueError, KeyError, RuntimeError):
        parser.exit(2, 'Recorded review demo failed closed; check explicit paths and source integrity.\n')
    print(json.dumps({key: report[key] for key in (
        'worker_status', 'worker_invocations', 'final_state', 'fresh_analysis_performed',
        'fabrication_authorized')}, sort_keys=True))
    return 0 if report['worker_status'] in ('REVIEW_REQUIRED', 'NOT_DISPATCHED') else 1


if __name__ == '__main__':
    raise SystemExit(main())
