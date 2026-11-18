"""Check complete original-camera evidence, source identity and native score replay."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent

def replay(run):
    from surface_material_models import WetFilm,SpreadFilm
    from surface_scoring import ScoreState
    c=json.loads((run/'contact-contract.json').read_text());cfg=json.loads((run/'configuration.json').read_text());kind=cfg['film_response']
    if kind=='stencil_paste':
        from stencil_paste_model import StencilPaste
        m=StencilPaste(json.loads((ROOT/'board-geometry.json').read_text()))
    else:m=WetFilm() if kind=='wet_mount' else SpreadFilm()
    z=np.load(run/'physics.npz');poses=z['poses'];vel=z['velocities'];rc=z['robot_contact_counts'];oc=z['object_contact_counts'];force=z['object_contact_forces'];hz=int(z['physics_hz']);s=ScoreState(kind,c['paths'],c['filters'],c['sensors'],m)
    observations=z['observation_samples'];recorded=json.loads((run/'material-trace.json').read_text());boundaries=set((observations+1).tolist());values={};start=0
    for i in range(len(poses)):
        s.step(poses[i],vel[i],rc[i],oc[i],1/hz)
        end=i+1
        if end-start==12 or end in boundaries or end==len(poses):
            if kind=='wet_mount':m.step(poses[i,0],(end-start)/hz)
            elif kind=='spread_plate':
                for a in range(start,end,2):
                    b=min(a+2,end);m.step(poses[b-1,0],poses[b-1,2],np.linalg.norm(force[a:b,2,c['filters'].index(c['paths'][0])],axis=-1).mean(),(b-a)/hz)
            else:
                n=Rotation.from_quat(poses[i,0,3:]).as_matrix()[:,2];f=np.abs(force[start:end,1,c['filters'].index(c['paths'][0])]@n).mean();m.step(poses[i,0],poses[i,1],f,(end-start)/hz,board_force=np.linalg.norm(force[start:end,1,c['filters'].index('/World/envs/env_0/pcb')],axis=-1).mean())
            s.update();start=end
        if end in boundaries:values[end-1]=(s.value,dict(s.metrics))
    errors=[]
    for idx,at in enumerate(observations):
        actual,metrics=values[int(at)]
        if idx>=len(recorded) or abs(actual-recorded[idx]['score'])>1e-8:errors.append(idx)
    assert not errors,('Native replay score mismatch',errors[:20])
    return dict(passed=True,final_score=s.value,checked_frames=len(observations),metrics=s.metrics)

def validate(run,do_replay=True):
    cfg=json.loads((run/'configuration.json').read_text());assert not cfg.get('config_only');steps=cfg.get('steps',len(json.loads((run/'plan.json').read_text())['steps']) if (run/'plan.json').exists() else 0)
    reports=[f for f in run.glob('adhoc*.json') if '.live.' not in f.name];assert len(reports)==1
    report=json.loads(reports[0].read_text());assert report['status']=='success' and not report['results'].get('errored_trials')
    files={k:sorted((run/'frames').rglob(f'*_{k}_*.npy')) for k in ['top_cam','left_cam','right_cam']};health={}
    for k,fs in files.items():
        assert len(fs)==steps+1,(k,len(fs),steps)
        low=[]
        for f in fs:
            im=np.load(f);assert im.shape==(176,320,3) and im.dtype==np.uint8
            low.append(float(im.astype(np.float32).std(axis=(0,1)).mean()))
        if k=='top_cam':assert min(low)>=2.
        health[k]=dict(frames=len(fs),minimum_spatial_std=min(low))
    z=np.load(run/'physics.npz');assert z['joint_pos'].shape==(steps+1,14)
    for n in z.files:
        if z[n].dtype.kind in 'fiu':assert np.isfinite(z[n]).all(),n
    assert len(z['poses'])>=steps*(int(z['physics_hz'])//20)
    source=json.loads((run/'source-manifest.json').read_text());runtime_names=[n for n in source if n.endswith('.py') or n in ('scene.json','demo-plan.json','board-geometry.json','runtime-lock.json')]
    changed=[n for n in runtime_names if not (ROOT/n).is_file() or hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=source[n]]
    if do_replay:assert not changed,('Use exact archived task source for score replay',changed)
    rr=replay(run) if do_replay else None
    official=float(report['results']['metrics']['task_score'])
    recorded=float(json.loads((run/'material-trace.json').read_text())[-1]['score'])
    assert abs(official-recorded)<1e-8
    if rr is not None:assert abs(rr['final_score']-official)<1e-8
    result=dict(passed=True,steps=steps,simulated_s=steps/20,cameras=health,source_match=not changed,raw_score_replay=rr)
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--no-replay',action='store_true');a=p.parse_args();print(json.dumps(validate(a.run,not a.no_replay),indent=2))
