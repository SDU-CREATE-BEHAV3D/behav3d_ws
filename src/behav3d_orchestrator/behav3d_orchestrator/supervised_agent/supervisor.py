"""Durable, explicitly approved observation review, never a robot controller.

The only supported operation is ``evaluate_observation``. The supplied callable
must be a trusted zero-argument review adapter, not agent-authored code. Python
callables cannot be sandboxed here. No operation name, request text, operator ID,
timestamp or restored journal record grants authority to move or print.

One instance exclusively owns a journal until ``close()``. Each transition is a
hash-chained JSONL state record, flushed with fsync before in-memory promotion.
The journal is an integrity/recovery log, not an authentication mechanism.
"""

from copy import deepcopy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import threading


class SupervisorError(ValueError):
    """The requested review transition cannot be accepted safely."""


REQUEST_FIELDS = frozenset({
    'request_id', 'artifact_id', 'observation_id', 'event_type',
    'candidate_cycle', 'scan_cycle', 'previous_evaluation_id', 'frame_id',
    'coordinate_units', 'source_manifest_path', 'source_manifest_sha256',
})
RECORD_STATES = frozenset({
    'PENDING', 'APPROVED', 'RUNNING', 'COMPLETED', 'FAILED',
    'CANCELLED', 'INVALIDATED', 'HOLD',
})
EVENTS = frozenset({
    'BOOT', 'ARM', 'DISARM', 'PROPOSE', 'APPROVE', 'REJECT', 'START',
    'COMPLETE', 'FAIL', 'CANCEL', 'INVALIDATE', 'LATE_RESULT',
})


def _json(value):
    try:
        return json.dumps(value, sort_keys=True, separators=(',', ':'),
                          ensure_ascii=True, allow_nan=False)
    except (TypeError, ValueError, OverflowError) as exc:
        raise SupervisorError('finite JSON data required') from exc


def _digest(value):
    return hashlib.sha256(_json(value).encode('utf-8')).hexdigest()


def _identifier(value, *, empty=False):
    if empty and value == '':
        return value
    if (not isinstance(value, str) or not value or len(value) > 256
            or any(not (c.isascii() and (c.isalnum() or c in '_./:-'))
                   for c in value)):
        raise SupervisorError('explicit bounded ASCII identifier required')
    return value


def _text(value):
    if (not isinstance(value, str) or not value.strip() or len(value) > 4096
            or any(ord(c) < 32 or ord(c) == 127 for c in value)):
        raise SupervisorError('nonempty single-line audit text required')
    return value


def _sha(value):
    if (not isinstance(value, str) or len(value) != 64
            or any(c not in '0123456789abcdef' for c in value)):
        raise SupervisorError('lowercase SHA-256 required')
    return value


def _evaluation_id(value, *, empty=False):
    if empty and value == '':
        return value
    if not isinstance(value, str) or not value.startswith('evaluation_'):
        raise SupervisorError('full evaluation_<SHA-256> identity required')
    _sha(value[len('evaluation_'):])
    return value


def _request(kind, value):
    if kind != 'evaluate_observation':
        raise SupervisorError('only observation review is implemented; hardware operations are disabled')
    if not isinstance(value, dict) or set(value) != REQUEST_FIELDS:
        raise SupervisorError('exact observation request fields required')
    _json(value)
    for key in ('request_id', 'artifact_id', 'observation_id', 'frame_id'):
        _identifier(value[key])
    _evaluation_id(value['previous_evaluation_id'], empty=True)
    if value['event_type'] != 'deposition_readback':
        raise SupervisorError('unsupported observation event')
    if value['coordinate_units'] != 'm':
        raise SupervisorError('explicit metre coordinates required')
    if any(type(value[key]) is not int or value[key] < 0
           for key in ('candidate_cycle', 'scan_cycle')):
        raise SupervisorError('nonnegative integer cycles required')
    if value['scan_cycle'] != value['candidate_cycle'] + 1:
        raise SupervisorError('candidate N must link to scan N+1')
    _text(value['source_manifest_path'])
    manifest_path = Path(value['source_manifest_path'])
    if (not manifest_path.is_absolute() or '..' in manifest_path.parts
            or str(manifest_path) != value['source_manifest_path']):
        raise SupervisorError('normalized absolute source manifest path required')
    _sha(value['source_manifest_sha256'])
    return json.loads(_json(value))


def _command_result(value, request_id):
    if not isinstance(value, dict):
        raise SupervisorError('Command result dictionary required')
    if type(value.get('ok')) is not bool:
        raise SupervisorError('Command result ok must be an exact boolean')
    if (value.get('kind') != 'evaluate_observation'
            or value.get('phase') not in ('plan', 'exec')):
        raise SupervisorError('unexpected Command result kind or phase')
    if value.get('planned_only') is not True:
        raise SupervisorError('review result must remain non-fabricating and planned-only')
    if (not isinstance(value.get('metrics'), dict)
            or 'error' not in value
            or (value['error'] is not None and not isinstance(value['error'], str))):
        raise SupervisorError('standard Command error and metrics required')
    if (value['metrics'].get('fabrication_authorized') is not False
            or value.get('fabrication_authorized', False) is not False):
        raise SupervisorError('review metrics must explicitly deny fabrication')
    if value.get('robot_commands', []) != []:
        raise SupervisorError('review results may not contain robot commands')
    _json(value)
    if value['ok']:
        if (value['metrics'].get('status') != 'REVIEW_REQUIRED'
                or value['metrics'].get('requires_review') is not True
                or value['metrics'].get('request_id') != request_id):
            raise SupervisorError('successful review must bind this request and require review')
        _evaluation_id(value['metrics'].get('evaluation_id'))
    return json.loads(_json(value))


def _failure(reason):
    return {
        'ok': False, 'kind': 'evaluate_observation', 'phase': 'exec',
        'planned_only': True,
        'error': reason, 'metrics': {'status': 'HOLD', 'requires_review': True,
                                   'fabrication_authorized': False},
    }


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise SupervisorError('duplicate journal JSON key')
        result[key] = value
    return result


class Supervisor:
    """Single-step review supervisor with durable, single-use approval.

    ``execute`` calls the trusted operation outside the lock and returns its
    standard Command result. ``receipt`` returns the durable wrapper. Repeating
    a completed request/digest returns its cached result, including after restart;
    pending approvals are never restored. Cancelled/invalidated/held IDs cannot
    be reused. After an uncertain in-flight result an operator must explicitly
    cancel/invalidate that record before a new request can be proposed.
    """

    def __init__(self, journal_path):
        self._lock = threading.RLock()
        self._faulted = False
        self._closed = False
        self._callback_running = False
        self._seq = 0
        self._last_sha = '0' * 64
        self._path = Path(journal_path).absolute()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        flags = os.O_RDWR | os.O_CREAT | os.O_APPEND
        flags |= getattr(os, 'O_NOFOLLOW', 0)
        self._fd = os.open(self._path, flags, 0o600)
        try:
            if not stat.S_ISREG(os.fstat(self._fd).st_mode):
                raise SupervisorError('journal must be a regular file')
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._data = self._load()
            boot = deepcopy(self._data)
            boot['armed'] = False
            active = boot['active_id']
            if active is not None:
                record = boot['records'][active]
                record['state'] = 'HOLD'
                record['approval'] = None
                record['hold_reason'] = 'restart requires explicit review; no automatic retry'
            boot['inflight_id'] = None
            self._commit('BOOT', boot)
            directory = os.open(self._path.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except BaseException:
            os.close(self._fd)
            self._closed = True
            raise

    def _load(self):
        os.lseek(self._fd, 0, os.SEEK_SET)
        with os.fdopen(os.dup(self._fd), 'rb') as stream:
            raw = stream.read()
        if raw and not raw.endswith(b'\n'):
            raise SupervisorError('truncated journal; reconciliation required')
        data = {'armed': False, 'active_id': None, 'inflight_id': None,
                'records': {}, 'heads': {}}
        for line in raw.splitlines():
            try:
                entry = json.loads(line.decode('utf-8'), object_pairs_hook=_pairs,
                                   parse_constant=lambda _: (_ for _ in ()).throw(
                                       SupervisorError('nonfinite journal data')))
                claimed = entry.pop('sha256')
                if (set(entry) != {'schema', 'sequence', 'previous_sha256', 'event', 'data', 'operator_id'}
                        or entry['schema'] != 'behav3d.supervised_journal.v1'
                        or type(entry['sequence']) is not int
                        or entry['sequence'] != self._seq + 1
                        or entry['previous_sha256'] != self._last_sha
                        or entry['event'] not in EVENTS or claimed != _digest(entry)):
                    raise SupervisorError('journal chain/schema mismatch')
                self._validate_state(entry['data'])
                data = entry['data']
                self._seq += 1
                self._last_sha = claimed
            except (AttributeError, KeyError, TypeError, ValueError, UnicodeError) as exc:
                raise SupervisorError('corrupt journal; no action is authorized') from exc
        return data

    @staticmethod
    def _validate_state(data):
        if (not isinstance(data, dict)
                or set(data) != {'armed', 'active_id', 'inflight_id', 'records', 'heads'}
                or type(data['armed']) is not bool
                or not isinstance(data['records'], dict) or not isinstance(data['heads'], dict)):
            raise SupervisorError('invalid journal state')
        for request_id, record in data['records'].items():
            _identifier(request_id)
            request = _request(record['kind'], record['request'])
            if (request_id != request['request_id'] or record['request_id'] != request_id
                    or record['state'] not in RECORD_STATES
                    or record['proposal_sha256'] != _digest({'kind': record['kind'], 'request': request})):
                raise SupervisorError('invalid journal proposal')
            if record['receipt'] is not None:
                receipt = record['receipt']
                if (receipt['schema'] != 'behav3d.supervised_receipt.v1'
                        or receipt['request_id'] != request_id
                        or receipt['proposal_sha256'] != record['proposal_sha256']
                        or type(receipt['accepted']) is not bool):
                    raise SupervisorError('invalid journal receipt')
                _command_result(receipt['result'], request_id)
        active = data['active_id']
        inflight = data['inflight_id']
        if active is not None and (active not in data['records']
                                   or data['records'][active]['state'] not in
                                   {'PENDING', 'APPROVED', 'RUNNING', 'HOLD'}):
            raise SupervisorError('invalid active journal record')
        if inflight is not None and (inflight != active or
                                      data['records'][inflight]['state'] not in {'RUNNING', 'HOLD'}):
            raise SupervisorError('invalid in-flight journal record')
        pending = [r['request_id'] for r in data['records'].values()
                   if r['state'] in {'PENDING', 'APPROVED', 'RUNNING', 'HOLD'}]
        if pending != ([] if active is None else [active]):
            raise SupervisorError('more than one active step in journal')
        for artifact, head in data['heads'].items():
            _identifier(artifact)
            _evaluation_id(head['evaluation_id'])
            if type(head['scan_cycle']) is not int or head['scan_cycle'] < 0:
                raise SupervisorError('invalid successful head')
        _json(data)

    def _ensure(self):
        if self._closed or self._faulted:
            raise SupervisorError('supervisor unavailable; journal reconciliation required')

    def _commit(self, event, data, operator_id=None):
        self._ensure()
        self._validate_state(data)
        entry = {'schema': 'behav3d.supervised_journal.v1',
                 'sequence': self._seq + 1, 'previous_sha256': self._last_sha,
                 'event': event, 'data': deepcopy(data), 'operator_id': operator_id}
        entry['sha256'] = _digest(entry)
        raw = (_json(entry) + '\n').encode('utf-8')
        try:
            remaining = memoryview(raw)
            while remaining:
                written = os.write(self._fd, remaining)
                if written <= 0:
                    raise OSError('short journal write')
                remaining = remaining[written:]
            os.fsync(self._fd)
        except OSError as exc:
            self._faulted = True
            self._data['armed'] = False
            raise SupervisorError('journal write failed; operation denied') from exc
        self._data = deepcopy(data)
        self._seq += 1
        self._last_sha = entry['sha256']

    def snapshot(self):
        with self._lock:
            self._ensure()
            result = deepcopy(self._data)
            active = result['active_id']
            result['state'] = ('HOLD' if active and result['records'][active]['state'] == 'HOLD'
                               else 'ARMED' if result['armed'] else 'DISARMED')
            result.update(sequence=self._seq, review_only=True,
                          hardware_operations_enabled=False)
            return result

    def _record(self, request_id, proposal_sha256=None):
        self._ensure()
        _identifier(request_id)
        if request_id not in self._data['records']:
            raise SupervisorError('unknown request')
        record = self._data['records'][request_id]
        if proposal_sha256 is not None:
            _sha(proposal_sha256)
            if record['proposal_sha256'] != proposal_sha256:
                raise SupervisorError('proposal digest conflict')
        return record

    def receipt(self, request_id):
        with self._lock:
            return deepcopy(self._record(request_id)['receipt'])

    def set_armed(self, armed, operator_id):
        if type(armed) is not bool:
            raise SupervisorError('explicit boolean arm state required')
        _text(operator_id)
        with self._lock:
            self._ensure()
            if not armed:
                return self._invalidate('explicit disarm', 'DISARM', operator_id)
            active = self._data['active_id']
            if self._data['inflight_id'] is not None or (active and
                    self._data['records'][active]['state'] == 'HOLD'):
                raise SupervisorError('in-flight/held step requires reconciliation')
            data = deepcopy(self._data)
            data['armed'] = True
            self._commit('ARM', data, operator_id)
            return self.snapshot()

    def _check_head(self, request):
        head = self._data['heads'].get(request['artifact_id'])
        if head is not None and (request['previous_evaluation_id'] != head['evaluation_id']
                                 or request['scan_cycle'] <= head['scan_cycle']):
            raise SupervisorError('stale/conflicting observation revision')

    def propose(self, kind, request):
        request = _request(kind, request)
        proposal_sha256 = _digest({'kind': kind, 'request': request})
        request_id = request['request_id']
        with self._lock:
            self._ensure()
            if request_id in self._data['records']:
                record = self._record(request_id, proposal_sha256)
                if record['state'] in {'CANCELLED', 'INVALIDATED', 'HOLD'}:
                    raise SupervisorError('request was invalidated; use a new reviewed request')
                return deepcopy(record)
            if not self._data['armed']:
                raise SupervisorError('supervisor is disarmed; new review proposal denied')
            if self._data['active_id'] is not None:
                raise SupervisorError('only one pending/running review is allowed')
            self._check_head(request)
            record = {'request_id': request_id, 'kind': kind, 'request': request,
                      'proposal_sha256': proposal_sha256, 'state': 'PENDING',
                      'approval': None, 'receipt': None, 'hold_reason': None}
            data = deepcopy(self._data)
            data['records'][request_id] = record
            data['active_id'] = request_id
            self._commit('PROPOSE', data)
            return deepcopy(record)

    def approve(self, request_id, proposal_sha256, operator_id, approve=True):
        _text(operator_id)
        if type(approve) is not bool:
            raise SupervisorError('explicit boolean approval required')
        with self._lock:
            record = self._record(request_id, proposal_sha256)
            if record['state'] != 'PENDING' or self._data['active_id'] != request_id:
                raise SupervisorError('only the current pending proposal can be approved')
            if not approve:
                return self._invalidate('operator rejected proposal', 'REJECT', operator_id)
            if not self._data['armed']:
                raise SupervisorError('supervisor is disarmed')
            self._check_head(record['request'])
            data = deepcopy(self._data)
            changed = data['records'][request_id]
            changed['state'] = 'APPROVED'
            changed['approval'] = {'operator_id': operator_id,
                                   'proposal_sha256': proposal_sha256}
            self._commit('APPROVE', data, operator_id)
            return deepcopy(changed)

    def _invalidate(self, reason, event, operator_id=None):
        _text(reason)
        self._ensure()
        data = deepcopy(self._data)
        data['armed'] = False
        active = data['active_id']
        if active is not None:
            record = data['records'][active]
            running = data['inflight_id'] == active
            record['state'] = 'HOLD' if running else ('CANCELLED' if event in {'CANCEL', 'REJECT'} else 'INVALIDATED')
            record['approval'] = None
            record['hold_reason'] = reason
            record['invalidated_by'] = operator_id
            if not running:
                data['active_id'] = None
        self._commit(event, data, operator_id)
        return self.snapshot()

    def invalidate(self, reason):
        with self._lock:
            return self._invalidate(reason, 'INVALIDATE')

    def cancel(self, request_id, reason):
        with self._lock:
            record = self._record(request_id)
            if self._data['active_id'] != request_id:
                raise SupervisorError('only the current active step can be cancelled')
            if record['state'] not in {'PENDING', 'APPROVED', 'RUNNING', 'HOLD'}:
                raise SupervisorError('completed steps cannot be cancelled')
            return self._invalidate(reason, 'CANCEL')

    def execute(self, request_id, proposal_sha256, operation_callable):
        with self._lock:
            record = self._record(request_id, proposal_sha256)
            if record['state'] in {'COMPLETED', 'FAILED'}:
                return deepcopy(record['receipt']['result'])
            if not callable(operation_callable):
                raise SupervisorError('trusted observation operation required')
            if (not self._data['armed'] or record['state'] != 'APPROVED'
                    or self._data['active_id'] != request_id
                    or self._data['inflight_id'] is not None):
                raise SupervisorError('current, armed, explicit single-use approval required')
            self._check_head(record['request'])
            data = deepcopy(self._data)
            data['records'][request_id]['state'] = 'RUNNING'
            data['records'][request_id]['approval'] = None
            data['inflight_id'] = request_id
            self._commit('START', data)
            self._callback_running = True
        interrupted = None
        try:
            result = _command_result(operation_callable(), request_id)
        except BaseException as exc:
            result = _failure('review operation failed: ' + type(exc).__name__)
            if not isinstance(exc, Exception):
                interrupted = exc
        with self._lock:
            self._callback_running = False
            self._ensure()
            data = deepcopy(self._data)
            record = data['records'][request_id]
            accepted = record['state'] == 'RUNNING' and data['armed'] and interrupted is None
            original_result = deepcopy(result)
            if not accepted:
                result = _failure('review completed after invalidation; reconciliation required')
                record['state'] = 'HOLD'
                record['hold_reason'] = result['error']
                data['armed'] = False
            else:
                record['state'] = 'COMPLETED' if result['ok'] else 'FAILED'
                data['active_id'] = None
                if result['ok']:
                    request = record['request']
                    data['heads'][request['artifact_id']] = {
                        'evaluation_id': result['metrics']['evaluation_id'],
                        'scan_cycle': request['scan_cycle'],
                    }
                else:
                    data['armed'] = False
            data['inflight_id'] = None
            record['receipt'] = {
                'schema': 'behav3d.supervised_receipt.v1', 'request_id': request_id,
                'proposal_sha256': proposal_sha256, 'state': record['state'],
                'accepted': accepted, 'result': deepcopy(result),
            }
            if not accepted:
                record['receipt']['unaccepted_operation_result'] = original_result
            self._commit('LATE_RESULT' if not accepted else 'COMPLETE' if result['ok'] else 'FAIL', data)
        if interrupted is not None:
            raise interrupted
        return deepcopy(result)

    def close(self):
        """Release exclusive journal ownership only when no callback is running."""
        with self._lock:
            if self._closed:
                return
            if self._callback_running:
                raise SupervisorError('cannot close while a review callback is in flight')
            os.close(self._fd)
            self._closed = True
