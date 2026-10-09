"""Provisional local review envelope, not an Agent007 transport implementation."""
import copy
import hashlib
import json


class ContractError(ValueError):
    """A request cannot be accepted safely."""


CATEGORIES = frozenset({
    'review_registration', 'review_scan_coverage', 'review_surface_identity',
    'inspect_geometry', 'inspect_dds_shape_size',
})


def _id(value):
    if not isinstance(value, str) or not value.strip():
        raise ContractError('IDs and frame must be nonempty strings')
    return value


def _cycle(value):
    if type(value) is not int or value < 0:
        raise ContractError('cycles must be nonnegative integers')
    return value


class DryRunController:
    """Single-process review ledger; never constructs executable commands.

    Caller must supply a trusted source manifest separately from the packet.
    Reuse this instance for replay protection. Persistence is a later milestone.
    """

    def __init__(self, *, frame, units):
        self.frame = _id(frame)
        if units not in ('m', 'mm'):
            raise ContractError('units must be m or mm')
        self.units = units
        self._heads = {}
        self._seen = set()
        self._requests = set()
        self._decisions = []

    @property
    def decisions(self):
        return copy.deepcopy(self._decisions)

    def execute(self, *args, **kwargs):
        raise ContractError('execution is unsupported in the offline controller')

    def review(self, packet, *, request_id, source):
        """Validate normalized packet against an explicitly supplied manifest.

        Exact provisional fields are documented in docs/OFFLINE_ROBOT_AGENT.md.
        Validation completes before any ledger mutation.
        """
        try:
            p = copy.deepcopy(packet)
            request_id = _id(request_id)
            if request_id in self._requests:
                raise ContractError('duplicate request')
            if p['schema'] != 'agent007.robot_review_packet.v1':
                raise ContractError('unsupported packet schema')
            for key in ('artifact_id', 'evaluation_id', 'scan_job_id'):
                _id(p[key])
            if p['frame'] != self.frame or p['units'] != self.units:
                raise ContractError('frame/units mismatch; no implicit conversion')
            for key in ('artifact_id', 'scan_job_id', 'candidate_cycle', 'scan_cycle',
                        'frame', 'units'):
                if p[key] != source[key]:
                    raise ContractError('source manifest mismatch: ' + key)
            cycle = _cycle(p['candidate_cycle'])
            if _cycle(p['scan_cycle']) != cycle + 1:
                raise ContractError('expected candidate N and scan N+1')
            artifact = p['artifact_id']
            head = self._heads.get(artifact)
            if p['evaluation_id'] in self._seen:
                raise ContractError('replayed evaluation')
            previous = p['previous_revision']
            if previous is not None:
                _id(previous)
            if previous != (head[0] if head else None):
                raise ContractError('revision does not extend current head')
            if head and cycle < head[1]:
                raise ContractError('stale cycle')
            evidence = p['evidence_ids']
            if not isinstance(evidence, list) or not evidence:
                raise ContractError('evidence IDs required')
            ids = {_id(e) for e in evidence}
            if len(ids) != len(evidence) or not ids <= set(source['evidence_ids']):
                raise ContractError('duplicate or unbound evidence')
            if not isinstance(p['summary'], dict) or 'uncertainty' not in p['summary']:
                raise ContractError('explicit uncertainty required')
            if not isinstance(p['summary']['uncertainty'], str) or not p['summary']['uncertainty'].strip():
                raise ContractError('uncertainty must be a nonempty description')
            if not isinstance(p['suggestions'], list):
                raise ContractError('suggestions must be a list')
            for suggestion in p['suggestions']:
                if suggestion['category'] not in CATEGORIES:
                    raise ContractError('unsupported suggestion category')
                for key, expected in (('executable', False),
                                      ('requires_operator_review', True),
                                      ('fabrication_authorized', False)):
                    if suggestion[key] is not expected:
                        raise ContractError('suggestion cannot authorize execution')
                refs = suggestion['evidence_ids']
                if not isinstance(refs, list) or not refs or not {_id(e) for e in refs} <= ids:
                    raise ContractError('suggestion evidence is unbound')
            digest = hashlib.sha256(json.dumps(p, sort_keys=True, allow_nan=False).encode()).hexdigest()
        except (KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, ContractError):
                raise
            raise ContractError('malformed review envelope') from exc
        result = {
            'schema': 'behav3d.dry_run_decision.v1', 'request_id': request_id,
            'artifact_id': artifact, 'evaluation_id': p['evaluation_id'],
            'packet_sha256': digest, 'candidate_cycle': cycle,
            'scan_cycle': p['scan_cycle'], 'frame': self.frame, 'units': self.units,
            'evidence_ids': evidence, 'uncertainty': p['summary']['uncertainty'],
            'decision': 'hold_for_operator_review',
            'reason': 'Evaluation evidence requires review; it grants no execution authority.',
            'proposal': {'action': 'review_recorded_evidence',
                         'categories': [s['category'] for s in p['suggestions']],
                         'preconditions': ['Operator reviews source identity and uncertainty'],
                         'executable': False, 'fabrication_authorized': False,
                         'requires_operator_review': True},
            'robot_commands': [],
        }
        self._heads[artifact] = (p['evaluation_id'], cycle)
        self._seen.add(p['evaluation_id'])
        self._requests.add(request_id)
        self._decisions.append(copy.deepcopy(result))
        return result
