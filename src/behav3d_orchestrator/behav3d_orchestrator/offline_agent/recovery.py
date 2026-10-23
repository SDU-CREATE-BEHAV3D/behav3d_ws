"""Deterministic recovery decisions, with no ROS or execution interface.

Agent007 does not yet produce this normalized issue contract. Adapters must state
whether an issue is observed or is an explicitly controlled experiment.
"""
from copy import deepcopy
import hashlib
import json
import math


SCHEMA = 'behav3d.recovery_issue.v1'
STRATEGIES = ('continue', 'fill', 'adapt', 'prioritize', 'support', 'contact', 'revise_goal')
VALIDATION_GATES = ('goal', 'avoidance', 'support', 'spacing', 'tool_access',
                    'robot', 'process', 'future_continuation')
SCREEN_GATES = ('support', 'spacing', 'tool_access', 'search_bounds')
BINDING_KEYS = ('artifact_id', 'mesh_revision', 'goal_revision', 'frame', 'units')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _text(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('nonempty text required')
    return value


def validate_issue(issue):
    if issue['schema'] != SCHEMA or issue['units'] != 'm':
        raise ValueError('unsupported issue schema or units')
    for key in (*BINDING_KEYS, 'id', 'region_id', 'observation_revision'):
        _text(issue[key])
    if issue['kind'] not in ('none', 'missing_material', 'lean', 'deviation', 'major_failure'):
        raise ValueError('unsupported issue kind')
    if issue['objective'] not in ('repair_deficit', 'continue_from_observed', 'restore_printed_shape'):
        raise ValueError('explicit recovery objective required')
    if issue['extent'] not in ('local', 'extensive'):
        raise ValueError('explicit failure extent required')
    evidence = issue['evidence']
    if evidence['origin'] not in ('recorded', 'controlled_experiment'):
        raise ValueError('explicit evidence origin required')
    if not evidence['references'] or not all(isinstance(r, str) and r for r in evidence['references']):
        raise ValueError('evidence references required')
    for key in ('identity_supported', 'registration_supported', 'coverage_sufficient',
                'failure_confirmed', 'tolerance_sensitive'):
        if type(evidence[key]) is not bool:
            raise ValueError('explicit evidence boolean required: ' + key)
    if evidence['trend'] not in ('stable', 'progressing', 'unknown'):
        raise ValueError('invalid trend')
    observations = evidence['trend_observation_ids']
    if not isinstance(observations, list) or any(not isinstance(x, str) or not x for x in observations):
        raise ValueError('invalid trend observations')
    if evidence['trend'] != 'unknown' and len(set(observations)) < 2:
        raise ValueError('stable/progressing requires two distinct observation revisions')
    if evidence['origin'] == 'controlled_experiment':
        _text(evidence['assumptions'])
    if issue['connection_status'] not in ('intact', 'compromised', 'unknown'):
        raise ValueError('invalid connection status')
    if type(issue['attempt_count']) is not int or issue['attempt_count'] < 0:
        raise ValueError('invalid attempt count')
    if issue['last_outcome'] not in ('not_attempted', 'improved', 'unchanged', 'worsened'):
        raise ValueError('invalid last outcome')
    # Validate the complete payload, including non-finite nested measurements.
    digest(issue)


def strategy_options(issue):
    """Eligibility describes strategy relevance, not permission to print."""
    kind, objective = issue['kind'], issue['objective']
    unstable = issue['evidence']['trend'] == 'progressing' or issue['connection_status'] == 'compromised'
    rules = {
        'continue': (kind == 'none', 'Ordinary rule-based planning remains the baseline.'),
        'fill': (kind == 'missing_material' and not unstable and issue['extent'] == 'local',
                 'Build from an accessible attachment surface toward a supported deficit.'),
        'adapt': (kind in ('lean', 'deviation') and objective == 'continue_from_observed'
                  and not unstable and issue['extent'] == 'local',
                  'Fit future deposition to the observed frontier within the allowed goal.'),
        'prioritize': (kind != 'none' and issue['extent'] == 'local',
                       'Continue elsewhere only after independence and access checks; does not repair the affected region.'),
        'support': (False, 'Needs an explicit support goal, attachment sequence and mechanical benefit model.'),
        'contact': (False, 'Needs a calibrated contact tool and characterized force/material response.'),
        'revise_goal': (False, 'Needs a versioned remaining goal and allowed design deviations.'),
    }
    return [{'strategy': key, 'eligible_for_comparison': allowed, 'reason': reason}
            for key, (allowed, reason) in rules.items()]


def planner_request(issue, proposal, snapshot):
    """Bind validation to this exact proposal and immutable planner state."""
    return {'schema': 'behav3d.recovery_validation_request.v1',
            'binding': {k: issue[k] for k in BINDING_KEYS},
            'issue_id': issue['id'], 'issue_sha256': digest(issue),
            'proposal_id': proposal['id'], 'proposal_sha256': digest(proposal),
            'snapshot_sha256': snapshot.get('snapshot_sha256'),
            'snapshot_status': snapshot['status'], 'strategy': proposal['strategy'],
            'targets': deepcopy(proposal['targets']), 'required_gates': list(VALIDATION_GATES),
            'instruction': 'Run existing rules with the captured settings. Unknown checks remain unknown.',
            'executable': False, 'robot_commands': []}


def _validation(issue, proposal, snapshot, receipt):
    unknown = {g: 'unknown' for g in VALIDATION_GATES}
    blockers = []
    if snapshot['status'] != 'complete':
        return unknown, ['planner_state_incomplete']
    state_hash = snapshot.get('snapshot_sha256', '')
    if not isinstance(state_hash, str) or len(state_hash) != 64 or any(c not in '0123456789abcdef' for c in state_hash):
        return unknown, ['planner_state_digest_missing_or_invalid']
    if snapshot.get('binding') != {k: issue[k] for k in BINDING_KEYS}:
        return unknown, ['planner_state_revision_mismatch']
    if not receipt:
        return unknown, ['planner_validation_not_run']
    request = planner_request(issue, proposal, snapshot)
    if (receipt.get('schema') != 'behav3d.recovery_validation_result.v1'
            or receipt.get('request_sha256') != digest(request)):
        return unknown, ['planner_receipt_does_not_match_request']
    _text(receipt['validator'])
    gates = {}
    for gate in VALIDATION_GATES:
        record = receipt.get('gates', {}).get(gate, {})
        status = record.get('status', 'unknown')
        if status not in ('pass', 'fail', 'unknown'):
            raise ValueError('invalid validation result')
        if status != 'unknown' and not record.get('evidence'):
            raise ValueError('gate result needs validation evidence')
        gates[gate] = status
        if status != 'pass':
            blockers.append(gate + ':' + status)
    return gates, blockers


def decide_recovery(issue, proposals, snapshot, *, receipts=None, max_attempts=2):
    """Choose a next step and keep diagnostic promise separate from validation.

    Receipts come from a trusted caller's numerical validator, never the agent's
    own scoring output. Even a fully validated result cannot execute a command.
    """
    validate_issue(issue)
    if type(max_attempts) is not int or max_attempts < 1:
        raise ValueError('positive attempt limit required')
    if snapshot['status'] not in ('complete', 'incomplete'):
        raise ValueError('invalid planner state status')
    receipts = receipts or {}
    options = strategy_options(issue)
    eligible = {r['strategy'] for r in options if r['eligible_for_comparison']}
    candidates = []
    seen = set()
    for proposal in proposals:
        pid = _text(proposal['id'])
        if pid in seen:
            raise ValueError('duplicate proposal ID')
        seen.add(pid)
        if proposal['strategy'] not in STRATEGIES:
            raise ValueError('unsupported proposal strategy')
        if proposal.get('executable', False) is not False or proposal.get('robot_commands', []):
            raise ValueError('offline proposal cannot contain execution authority or robot commands')
        if proposal.get('binding') != {k: issue[k] for k in BINDING_KEYS}:
            raise ValueError('proposal geometry checks do not match issue revisions/frame')
        if not isinstance(proposal['targets'], list):
            raise ValueError('proposal targets must be a list')
        labels = set()
        for target in proposal['targets']:
            label = _text(target['label'])
            if label in labels: raise ValueError('duplicate target label')
            labels.add(label)
            for key in ('top_m', 'axis'):
                v = target[key]
                if (not isinstance(v, (list, tuple)) or len(v) != 3
                        or any(not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x) for x in v)):
                    raise ValueError('finite XYZ target vector required')
            if sum(x*x for x in target['axis']) < 1e-12: raise ValueError('nonzero target axis required')
            for key in ('width_m', 'height_m'):
                if type(target[key]) not in (int, float) or not math.isfinite(target[key]) or target[key] <= 0:
                    raise ValueError('positive target dimensions required')
        digest(proposal)
        metrics = proposal['metrics']
        for key in ('benefit', 'deviation_mm', 'material_mm3'):
            if type(metrics[key]) not in (int, float) or not math.isfinite(metrics[key]) or metrics[key] < 0:
                raise ValueError('finite nonnegative proposal metric required: ' + key)
        if metrics['benefit_basis'] != issue['benefit_basis']:
            raise ValueError('cannot compare benefits with different meanings')
        states = {g: proposal['screens'].get(g, 'unknown') for g in SCREEN_GATES}
        if any(s not in ('pass', 'fail', 'unknown') for s in states.values()):
            raise ValueError('invalid geometric screen status')
        reasons = [g + ':failed_screen' for g, status in states.items() if status == 'fail']
        if not proposal['targets']:
            reasons.append('no_deposition_candidates')
        if metrics['benefit'] <= 0:
            reasons.append('no_demonstrated_objective_progress')
        if proposal['strategy'] not in eligible:
            reasons.append('strategy_not_applicable_to_objective_or_condition')
        if proposal['strategy'] == 'prioritize' and proposal.get('independence_verified') is not True:
            reasons.append('independent_progress_not_verified')
        gates, blockers = _validation(issue, proposal, snapshot, receipts.get(pid))
        reasons.extend(g + ':validation_failed' for g, status in gates.items() if status == 'fail')
        geometry = ('rejected' if any(s == 'fail' for s in states.values()) else
                    'passes_experimental_screens' if all(s == 'pass' for s in states.values()) else 'incomplete')
        screen = ('rejected' if reasons else 'promising' if all(s == 'pass' for s in states.values())
                  else 'incomplete')
        candidates.append({'id': pid, 'strategy': proposal['strategy'], 'metrics': deepcopy(metrics),
                           'geometric_screen': geometry, 'comparison_status': screen, 'rejection_reasons': reasons,
                           'validation_gates': gates, 'validation_blockers': blockers,
                           'validated_for_review': screen == 'promising' and not blockers,
                           'proposal_sha256': digest(proposal), 'executable': False})
    # Feasibility precedes ranking. Never let a high benefit cancel a failed gate.
    ranked = sorted((r for r in candidates if r['comparison_status'] == 'promising'),
                    key=lambda r: (not r['validated_for_review'], -r['metrics']['benefit'],
                                   r['metrics']['deviation_mm'], r['metrics']['material_mm3'], r['id']))
    evidence = issue['evidence']
    evidence_gaps = [key for key in ('identity_supported', 'registration_supported', 'coverage_sufficient')
                     if not evidence[key]]
    if issue['kind'] != 'none' and not evidence['failure_confirmed']:
        evidence_gaps.append('failure_unconfirmed')
    if evidence['tolerance_sensitive']:
        evidence_gaps.append('tolerance_sensitive_discrepancy')
    if issue['kind'] in ('lean', 'deviation'):
        if evidence['trend'] == 'unknown': evidence_gaps.append('deformation_trend_unknown')
        if issue['connection_status'] == 'unknown': evidence_gaps.append('connections_unverified')
    selected = None
    if issue['attempt_count'] >= max_attempts or issue['last_outcome'] in ('unchanged', 'worsened'):
        action, reason = 'hold_and_reassess', 'Recovery retry limit or unsuccessful readback reached.'
    elif issue['extent'] == 'extensive' or issue['kind'] == 'major_failure':
        action, reason = 'hold_and_reassess', 'A revised construction or salvage objective is needed.'
    elif evidence['trend'] == 'progressing' or issue['connection_status'] == 'compromised':
        action, reason = 'defer_affected_region', 'Further deposition requires stability or connection recovery evidence.'
    elif evidence_gaps:
        action, reason = 'inspect_region', 'Resolve: ' + ', '.join(evidence_gaps) + '.'
    elif issue['kind'] == 'none':
        action, reason = 'use_normal_planner', 'No supported recovery issue; preserve normal rule-based generation.'
    elif issue['objective'] == 'restore_printed_shape':
        action, reason = 'hold_for_contact_or_goal_study', 'New deposition does not restore the existing printed shape.'
    elif ranked:
        selected = ranked[0]
        action = 'review_bounded_batch' if selected['validated_for_review'] else 'validate_recovery_proposal'
        reason = ('Best feasible comparison under the declared objective; ' +
                  ('all required validation gates passed.' if selected['validated_for_review'] else
                   'complete the missing planner/process checks before considering deposition.'))
    else:
        action, reason = 'hold_and_reassess', 'No proposal demonstrates objective progress while passing the geometric screens.'
    return {'schema': 'behav3d.recovery_decision.v1', 'issue_id': issue['id'],
            'issue_sha256': digest(issue), 'evidence_origin': evidence['origin'],
            'next_action': action, 'reason': reason,
            'selected_proposal_id': None if selected is None else selected['id'],
            'diagnostic_best_id': None if not ranked else ranked[0]['id'],
            'affected_region_deferred': action != 'use_normal_planner',
            'evidence_gaps': evidence_gaps, 'strategies': options, 'alternatives': candidates,
            'ranking_order': ['validated feasibility', 'objective progress', 'least displacement', 'least material'],
            'checkpoint': {'preserve_required_frontier_sweep': True, 'region_id': issue['region_id'],
                           'compare_to_observation': issue['observation_revision'],
                           'measure': (['deficit extent and visibility', 'attachment and neighbor changes']
                                       if issue['kind'] == 'missing_material' else
                                       ['residual displacement', 'change in lean', 'connections and neighbor clearance']),
                           'before_next_batch': True, 'max_attempts': max_attempts,
                           'stop_if': ['new or worsening deviation', 'lost attachment or clearance',
                                       'no measured improvement', 'unresolved identity or registration'],
                           'success_requires': 'Issue-specific calibrated tolerances and a new evidence revision; no automatic resume.'},
            'executable': False, 'robot_commands': []}
