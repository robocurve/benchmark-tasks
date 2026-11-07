"""Validate evidence, not a substitute for inspecting real images and task design."""
import argparse,json
from pathlib import Path
import numpy as np

def validate(run):
    logs=list(run.glob('adhoc_*.json'));assert len(logs)==1
    result=json.loads(logs[0].read_text());assert result['status']=='success' and result['error'] is None
    assert result['results']['errored_trials']==0
    n=result['stats']['total_steps'];assert n>0
    trace=json.loads((run/'evaluation_trace.json').read_text())
    cfg=json.loads((run/'configuration.json').read_text())
    assert trace['monitor_ready'] and trace['monitor_error'] is None
    assert trace['contact_samples']>=12*n
    assert len(trace['steps'])==n and all(s['valid'] for s in trace['steps'])
    score=result['results']['metrics']['task_score'];assert score==trace['steps'][-1]['score']
    actions=list(run.rglob('scene-0-e0.jsonl'));assert len(actions)==1
    rows=[json.loads(line) for line in actions[0].open()]
    assert sum('action' in row for row in rows)==n
    assert all(len(row['action'])==14 and np.isfinite(row['action']).all() for row in rows if 'action' in row)
    counts={}
    from guide_task.image_health import ImageMonitor
    health=ImageMonitor()
    all_frames={}
    for cam in ('top_cam','left_cam','right_cam'):
        frames=sorted((run/'frames').rglob(f'*-e0_{cam}_*.npy'));assert len(frames)==n+1
        all_frames[cam]=frames
        for f in (frames[0],frames[-1]):
            rgb=np.load(f);assert rgb.shape==(176,320,3) and rgb.dtype==np.uint8
        counts[cam]=len(frames)
    for i in range(n+1):
        health.update({cam:np.load(paths[i]) for cam,paths in all_frames.items()})
    if score==1:
        assert trace['steps'][-1]['complete'] and trace['steps'][-1]['hold_s']>=1
        assert not any(s['contact_latched'] for s in trace['steps'])
    from replay_score import replay
    from guide_task.rules import VERSION
    recomputed=replay(run)
    assert all(s.get('version')==VERSION for s in trace['steps'])
    for saved,checked in zip(trace['steps'],recomputed):
        for key in ('score','complete','valid','hold_s','threaded','goal_supported','released','contact_latched','contact_witness'):
            assert saved[key]==checked[key],(saved['step'],key)
    assert cfg['scoring_version']==VERSION and not cfg['config_only']
    assert n<=cfg['steps'] and result['results']['total_trials']==1
    assert result['samples'][0]['instruction']==cfg['canonical_instruction']
    if (run/'process.exit').exists():assert int((run/'process.exit').read_text())==0
    mode=cfg['mode']
    if mode=='demo':
        assert score==1,'scripted demo did not complete'
        assert n/20>=cfg['demo_record_until_s']
        control=json.loads((run/'controller_trace.json').read_text())
        assert len(control)==n and any(row['rgb_view'] is not None for row in control)
        assert not trace['violations'] and cfg['controller_installed']
    elif mode in ('setup','stability'):
        assert score==0 and n==cfg['steps'] and not trace['violations']
        positions=np.array([row['pose'][:3] for row in trace['steps']])
        assert np.linalg.norm(positions-positions[0],axis=1).max()<.001
    elif mode in ('negative','lift_probe'):
        assert score==0 and trace['violations'] and trace['steps'][-1]['contact_latched']
    elif mode=='dream':
        assert cfg.get('image_integrity_guard')
        assert not cfg['controller_installed'] and not (run/'controller_trace.json').exists()
        assert cfg['demo_record_until_s'] is None
        assert result['eval']['policy']=='dream'
        transcript=result['samples'][0]['policy_transcripts'][0]
        assert transcript['requests'] and not transcript['dry_run']
        assert transcript['model']=='dreamlabs/dream-1-beta'
        assert transcript['control_hz']==20 and transcript['rows_per_request']==48
        assert score==1 or n==cfg['steps'],'Dream stopped before its declared budget'
    report=dict(evidence_valid=True,scorer_version=VERSION,raw_replay_matches=True,score=score,actions=n,frames=counts,
                monitor_samples=trace['contact_samples'],prohibited_reports=len(trace['violations']),
                contact_latched=any(s['contact_latched'] for s in trace['steps']),
                final_hold_s=trace['steps'][-1]['hold_s'],final_error_m=trace['steps'][-1]['distance_m'],
                max_progress=max(s['progress'] for s in trace['steps']),peak_grasp_force_n=trace['peak_grasp_force_n'],
                controller=cfg['controller'],mode=mode,formal_dream=mode=='dream' and cfg['steps']==2400,
                contact_witness=trace['steps'][-1]['contact_witness'],
                image_integrity_valid=True,minimum_overview_spatial_std=health.report()['minimum_spatial_std'],
                submission_ready=False)
    (run/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    try:report=validate(a.run)
    except Exception as error:
        report=dict(evidence_valid=False,error_type=type(error).__name__)
        if isinstance(error,AssertionError):report['check']=str(error)
        (a.run/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['evidence_valid'] else 1)
