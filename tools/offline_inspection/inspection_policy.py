"""Qualify a named inspection need before spending supplemental scan views.

Review decisions remain proposals. A nominal-area gain alone cannot justify
another capture or clear an Agent007 finding.
"""
from copy import deepcopy


def qualify_requests(requests, candidate):
    accepted, decisions = [], []
    for request in requests:
        if request['candidate_labels'] != [candidate['label']] or candidate['evidence_id'] not in request['evidence_ids']:
            raise ValueError('inspection request is not bound to this candidate')
        action = request['action']
        if (candidate.get('remeasured_in_current_scan') is not True
                or candidate.get('observation_age_cycles') != 0):
            disposition = 'review_stale_evidence_before_scan_change'
            reason = 'The requested region lacks a current candidate evaluation.'
        elif candidate.get('evaluation_status') == 'paired_identity_review':
            disposition = 'review_identity_before_scan_change'
            reason = 'Another geometric opportunity does not establish which material produced the returns.'
        elif action == 'review_scan_coverage':
            disposition = 'review_raw_depth_quality_before_pose_selection'
            reason = 'Cap quality requires depth/outline/view evidence; nominal visible area alone is insufficient.'
        elif action == 'inspect_dds_shape_size':
            if (candidate.get('dds_status') in {'missing', 'extra', 'missing_and_extra'}
                    and candidate.get('paired_position_evidence_eligible') is True
                    and not candidate.get('cap_reasons')):
                disposition = 'propose_targeted_geometry_views'
                reason = 'A specific provisional shape discrepancy remains with usable cap and pairing evidence.'
            elif (candidate.get('dds_status') == 'within_tolerance'
                  and candidate.get('evaluation_status') == 'within_provisional_position_rule'
                  and not candidate.get('uncertainty_reasons')
                  and not candidate.get('cap_reasons')):
                disposition = 'retain_sweep_no_specific_extra_scan_need'
                reason = 'Current position and DDS evidence are within their provisional rules; the generic suggestion supplies no unresolved scan objective.'
            else:
                disposition = 'review_shape_evidence_before_pose_selection'
                reason = 'Clarify observation support or association before turning a shape review into scan poses.'
        elif action == 'inspect_geometry':
            if (candidate.get('evaluation_status') == 'position_deviation_review'
                    and candidate.get('quantitative_position_evidence_eligible') is True):
                disposition = 'propose_targeted_geometry_views'
                reason = 'A supported provisional positional discrepancy provides a specific inspection objective.'
            else:
                disposition = 'review_position_evidence_before_pose_selection'
                reason = 'No supported positional discrepancy has been supplied for this request.'
        else:
            raise ValueError('unsupported inspection objective')
        if disposition == 'propose_targeted_geometry_views':
            accepted.append(deepcopy(request))
        decisions.append({'request_id': request['request_id'], 'candidate_label': candidate['label'],
                          'evidence_ids': list(request['evidence_ids']),
                          'disposition': disposition, 'reason': reason,
                          'inspection_resolved': False})
    return accepted, decisions
