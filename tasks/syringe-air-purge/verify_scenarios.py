"""Verify recorded controls, their source snapshots and the intended failure cause.
This checks evidence consistency; inspect the videos separately for scene validity.
Usage: python verify_scenarios.py OUTPUT_ROOT
"""
import hashlib
import json
from pathlib import Path
import sys

SYRINGE={'full':1.,'horizontal':0.,'overpush':0.,'partial':.3,'retract_repurge':1.,'no_return':0.,'no_release':0.,'unsupported':0.,'none':0.}
DENTAL={'full':1.,'fixed_site':1.,'point':0.,'static':.25,'off':0.,'late_suction':0.,'no_withdraw':.75,'obstruct':0.,'none':0.}

def verify(case,kind):
    result=json.loads((case/'result.json').read_text())
    trace=json.loads((case/'trace.json').read_text())
    mode=result['mode']; expected=(SYRINGE if kind=='syringe' else DENTAL)[mode]
    assert result['task_score']==expected, 'unexpected score'
    assert 0<result['steps']<=(1800 if kind=='syringe' else 1200), 'action budget'
    assert len(trace)==result['steps'], 'incomplete trace'
    assert result['termination']!='invalid_physics', 'invalid physics'
    assert result['blank_frames']==0, 'blank frames or unrecorded run'
    assert result.get('recorded_frames_per_camera')==result['steps']+1, 'missing initial or trial frames'
    counts=result.get('blank_frames_by_camera')
    assert counts and set(counts)=={'top_cam','left_cam','right_cam'} and all(x==0 for x in counts.values()), 'per-camera blank check missing/failed'
    assert (case/'demo.mp4').stat().st_size>0 and (case/'setup.png').stat().st_size>0, 'missing media'
    source=case.parent/'source'; manifest=json.loads((source/'manifest.json').read_text())
    for name,digest in manifest['sha256'].items():
        assert hashlib.sha256((source/name).read_bytes()).hexdigest()==digest, 'snapshot hash mismatch'
    m=result['task_metrics']
    assert m==trace[-1], 'result/trace mismatch'
    assert abs(m['score']-expected)<1e-8, 'trace score differs'
    if kind=='syringe':
        quality=m['air_ml']<=.05 and m['sample_loss_ml']<=.30
        if mode in {'full','retract_repurge','no_return','no_release'}: assert quality,'purge itself did not succeed; endpoint/control cause not isolated'
        if mode in {'full','retract_repurge'}:
            assert all(m[x] for x in ['lifted','parked','released']) and not m['failed']
            assert m['endpoint_hold_s']>=1., 'full endpoint not held for one second'
        if mode in {'horizontal','overpush'}:
            assert m['sample_loss_ml']>.50 and m['failed'],'expected sample-loss failure absent'
            assert all(m[x] for x in ['lifted','parked','released']),'endpoint failure confounds sample-loss comparison'
        if mode=='retract_repurge':assert m['aspirated_ml']>.20 and not m['failed'],'corrective aspiration did not recover'
        if mode=='no_return':assert not m['parked'],'return omission not demonstrated'
        if mode=='no_release':assert m['parked'] and not m['released'],'release omission not isolated'
        if mode=='unsupported':assert m['failed'],'removing support did not produce the expected physical failure'
    else:
        if mode in {'full','fixed_site','static','late_suction','no_withdraw'}:assert m['coverage']==[9,9,9],'coverage failed too; intended contrast not isolated'
        if mode in {'full','fixed_site','point','no_withdraw'}:
            assert all(v>=1.8 for v in m['captured']) and m['lost']<=.30,'recovery failed too'
            assert not m['failed'],'unintended guard failure'
        if mode=='point':assert m['coverage']==[1,1,1] and m['held'] and m['clear'],'point-rinse comparison not isolated'
        if mode=='static':assert m['captured'][0]>=1.8 and m['captured'][1]<1.8 and m['captured'][2]<1.8 and not m['failed']
        if mode=='late_suction':
            assert all(v>=1.8 for v in m['delivered']) and m['lost']>.30 and m['peak_pool']>.45
            assert not m['failed'] and m['held'] and m['clear'],'guard/endpoint failure confounds delayed-suction comparison'
        if mode=='no_withdraw':assert m['held'] and not m['clear'],'withdrawal omission not isolated'
        if mode=='obstruct':assert m['failed'] and m.get('obstruction_s',0)>.25,'expected obstruction failure absent'
        assert abs(m['balance_error'])<1e-6,'water imbalance'
    return {'mode':mode,'score':expected,'evidence_level':'recorded_rigid_body_simulation','automated_checks':'pass','visual_inspection':'required'}

def main():
    root=Path(sys.argv[1]);kind='syringe' if (Path(__file__).parent/'fluid.py').exists() else 'dental'
    rows=[];failed=False
    for case in sorted(root.iterdir()):
        if not case.is_dir() or case.name=='source':continue
        try:row=verify(case,kind)
        except Exception as e:row={'directory':case.name,'automated_checks':'failed','reason':str(e)};failed=True
        rows.append(row)
    if not rows:raise SystemExit('No recorded scenario directories found; no evidence verified.')
    print(json.dumps(rows,indent=2))
    raise SystemExit(1 if failed else 0)

if __name__=='__main__':main()
