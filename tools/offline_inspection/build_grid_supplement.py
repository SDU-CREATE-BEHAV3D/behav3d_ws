"""Reuse source-bound local-scene assessments; no additional scene reconstruction."""
import argparse
import hashlib
import json
from pathlib import Path
from grid_supplement import plan_supplement


def canonical_digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_assessment(local):
    payload=dict(local);expected=payload.pop('assessment_sha256')
    if canonical_digest(payload)!=expected:
        raise ValueError('local assessment content differs from its saved revision')
    if local['scan_cycle']!=local['candidate_cycle']+1:
        raise ValueError('invalid candidate/scan cycle linkage')
    for path,digest in local['sources'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
            raise ValueError('source changed since local assessment: '+path)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--demo',type=Path,required=True)
    args=ap.parse_args();path=args.demo/'local_scene.json'
    local=json.loads(path.read_text());validate_assessment(local)
    display=json.loads((args.demo/'scene.json').read_text())
    if display['local_scene']!=local or display['plan']['grid']!=local['patterns']['grid']['poses']:
        raise ValueError('viewer/assessment grid or revision mismatch')
    result=plan_supplement(local['patterns']['grid'],local['patterns']['direction_pool'],
                           local['nominal_surface'],frame=local['frame'],units=local['units'])
    result.update(label=local['label'],candidate_cycle=local['candidate_cycle'],scan_cycle=local['scan_cycle'],
                  source_assessment_sha256=local['assessment_sha256'],
                  source_path=str(path.resolve()),source_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    result['proposal_sha256']=canonical_digest(result)
    (args.demo/'grid_supplement.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    display['grid_supplement']=result
    (args.demo/'scene.json').write_text(json.dumps(display,allow_nan=False)+'\n')
    template=Path(__file__).with_name('viewer.html').read_text()
    (args.demo/'viewer.html').write_text(template.replace('/*DATA*/',json.dumps(display,allow_nan=False)))
    for row in result['curves']:
        print(json.dumps({k:row[k] for k in ('budget','extra_views','new_side_area_fraction','selected_pool_indices','append_order_metrics')}))


if __name__=='__main__':main()
