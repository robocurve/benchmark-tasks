"""Evidence validator, never a task scorer or proof of paperboard fidelity."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('run',type=Path)
p.add_argument('--log',type=Path)
a=p.parse_args();out=a.run
if a.log is None:a.log=out.parent/(out.name+'.console.log')
import subprocess,sys
if subprocess.call([sys.executable,str(Path(__file__).resolve().with_name('replay_score.py')),str(out)],stdout=subprocess.DEVNULL)!=0:
    raise SystemExit('Raw scoring replay failed')
contract=json.loads((out/('mechanism-contract.json' if (out/'mechanism-contract.json').exists() else 'parameters.json')).read_text())
z=np.load(out/'physics-substeps.npz');q=z['poses'];v=z['velocities']
paths=[str(x) for x in z['paths']];names=[x.rsplit('/',1)[-1] for x in paths]
centres=contract['centres'];errors=[];hinges=[]
finite=bool(np.isfinite(q).all() and np.isfinite(v).all())
if not finite:errors.append('Nonfinite physics')
norms=np.linalg.norm(q[:,:,3:],axis=2)
quat_ok=bool(np.max(abs(norms-1))<1e-3)
if not quat_ok:errors.append('Invalid quaternion')
sdk_errors=[line[:500] for line in a.log.read_text(errors='replace').splitlines() if '[Error]' in line or 'Traceback (most recent call last)' in line]
errors.extend(sdk_errors)
expected=round(contract['seconds']*240) if 'seconds' in contract else None
if expected is not None and len(q)!=expected:errors.append(f'Incomplete run: {len(q)}/{expected} samples')
if (out/'run-state.json').exists():
    state=json.loads((out/'run-state.json').read_text())
    if not state['completed']:errors.append('Runtime failure: '+str(state['error']))
capacity=int(z['contact_capacity']) if 'contact_capacity' in z else 2048
if 'contact_counts' in z and np.max(z['contact_counts'],initial=0)>=capacity:errors.append('Contact recording capacity exhausted')
if 'contact_counts' in z and (not np.isfinite(z['contact_counts']).all() or np.any(z['contact_counts']<0)):
    errors.append('Invalid robot contact counts')
image=json.loads((out/'image_health.json').read_text())
if not image['valid']:errors.append('Invalid original camera rendering')
if 'Incomplete contact data' in a.log.read_text(errors='replace'):errors.append('SDK reported incomplete contacts')
if 'robot_contact_forces' in z:
    rf=z['robot_contact_forces']
    if rf.shape!=(len(q),18,len(paths),3):errors.append('Incomplete original robot/carton contact matrix')
    if not np.isfinite(rf).all():errors.append('Nonfinite robot contacts')
    if len(z['contact_counts'])!=len(q):errors.append('Incomplete robot contact counts')
    cc=json.loads((out/'contact-contract.json').read_text())
    if len(cc['sensors'])!=18 or any(row!=paths for row in cc['filters']):errors.append('Robot contact filter mapping mismatch')
environment_coverage=False
if (out/'environment-contact-contract.json').exists():
    ec=json.loads((out/'environment-contact-contract.json').read_text())
    if ec['robot_sensors']!=cc['sensors'] or ec['carton_sensors']!=paths:
        errors.append('Environment contact sensor mapping mismatch')
    if any(row!=cc['sensors']+[ec['table']] for row in ec['robot_filters']):
        errors.append('Environment robot filter mapping mismatch')
    if any(row!=[ec['table']] for row in ec['carton_filters']):
        errors.append('Support filter mapping mismatch')
    for prefix,shape in (('environment',(18,19)),('support',(len(paths),1))):
        f=z[prefix+'_contact_forces'];counts=z[prefix+'_contact_counts']
        if f.shape!=(len(q),*shape,3) or counts.shape!=(len(q),*shape):
            errors.append('Incomplete '+prefix+' contacts')
        if not np.isfinite(f).all() or not np.isfinite(counts).all() or np.any(counts<0) or np.any(counts.sum(axis=(1,2))>=ec['capacity']):
            errors.append('Invalid or saturated '+prefix+' contacts')
    environment_coverage=not errors
official=None
if (out/'configuration.json').exists():
    cfg=json.loads((out/'configuration.json').read_text());argv=cfg['argv']
    expected_actions=int(argv[argv.index('--max-steps')+1])
    files=[f for f in out.glob('adhoc*.json') if '.live.' not in f.name]
    if len(files)!=1:errors.append('Missing or ambiguous official run result')
    else:
        r=json.loads(files[0].read_text());frames=Path(r['stats']['frames_dir'])
        counts={n:len(list(frames.glob('*_'+n+'_*.npy'))) for n in ('top_cam','left_cam','right_cam')}
        actual_actions=r['stats']['total_steps']
        achieved=r.get('results',{}).get('metrics',{}).get('task_score')==1.
        success_termination=(len(r.get('samples',[]))==1 and r['samples'][0].get('termination_reasons')==['success'])
        allowed_early=cfg.get('scoring_implemented',False) and achieved and success_termination and 0<actual_actions<=expected_actions
        official=dict(status=r['status'],error=r.get('error'),steps=actual_actions,expected_actions=expected_actions,frames=counts,full_score=achieved,
                      score=r.get('results',{}).get('metrics',{}).get('task_score'))
        # Pinned station reset performs 0.5s settling before the action loop.
        expected=120+12*actual_actions
        if len(q)!=expected:errors.append(f'Incomplete reset/action substeps: {len(q)}/{expected}')
        if r['status']!='success' or r.get('error') or (actual_actions!=expected_actions and not allowed_early):errors.append('Official rollout incomplete or failed')
        if any(c!=actual_actions+1 for c in counts.values()):errors.append('Original camera stream incomplete')
        if len(q)<actual_actions*12:errors.append('Missing physical substeps')
def point(name,local):
    i=names.index(name)
    return q[:,i,:3]+Rotation.from_quat(q[:,i,3:]).apply(np.tile(np.array(local)-centres[name],(len(q),1)))
if finite and quat_ok:
    for h in contract['hinges']:
        errs=[np.linalg.norm(point(h['bodies'][0],e)-point(h['bodies'][1],e),axis=1)*1000 for e in h['edge']]
        hinges.append(dict(name=h['name'],peak_mm=float(np.max(errs)),final_mm=float(np.max(np.array(errs)[:,-1]))))
    apex=point('carton_spout',[0.,0.,contract['width_m']/2])
    body_origin=point('carton_body',[0,0,0]);i=names.index('carton_body')
    local=Rotation.from_quat(q[:,i,3:]).inv().apply(apex-body_origin)
    sample_indices=range(0,len(local),240)
    apex_sampled_mm=[dict(t_s=k/240,xyz=(local[k]*1000).tolist()) for k in sample_indices]
    opened_apex=np.array([-contract['width_m'],0.,contract['width_m']/2])
    open_error_mm=float(np.linalg.norm(local[-1]-opened_apex)*1000)
else:apex_sampled_mm=[];open_error_mm=None
pairs=[]
if 'contact_forces' in z:
    f=np.linalg.norm(z['contact_forces'],axis=-1)
    if not np.isfinite(f).all():errors.append('Nonfinite contacts')
    for i in range(len(names)):
        for j in range(i+1,len(names)):
            peak=float(max(f[:,i,j].max(),f[:,j,i].max()))
            if peak>1e-6:pairs.append(dict(bodies=[names[i],names[j]],peak_N=peak))
    pairs.sort(key=lambda x:-x['peak_N'])
maxgap=max((h['peak_mm'] for h in hinges),default=None)
score_agreement=False
if official and (out/'live-score.json').exists() and (out/'score-replay.json').exists():
    live=json.loads((out/'live-score.json').read_text())
    replay=json.loads((out/'score-replay.json').read_text())
    manifest=json.loads((out/'source_manifest.json').read_text())
    score_agreement=(live['final']['score']==replay['score']==official['score']
        and replay['samples']==len(q) and live['final']['samples']==len(q)
        and replay['scorer_sha256']==manifest['carton_probe/scoring.py'])
    if not score_agreement:errors.append('Official/live/raw scoring or scorer-source mismatch')
full_verified=bool(not errors and environment_coverage and score_agreement and official['full_score'] and maxgap is not None and maxgap<.2)
result=dict(samples=len(q),physics_hz=240,finite=finite,quaternion_norm_ok=quat_ok,
    sdk_errors=sdk_errors,validation_errors=errors,expected_samples=expected,recording_valid=not errors,hinge_closure=hinges,
    peak_crease_endpoint_error_mm=maxgap,endpoint_accuracy_under_0_2mm=bool(maxgap is not None and maxgap<.2),
    apex_sampled_mm=apex_sampled_mm,final_apex_90deg_reference_error_mm=open_error_mm,
    apex_reference_warning='90 degrees is an overfolded closed mouth, NOT a functional opening target.',
    last_second_peak_linear_speed_m_s=float(np.max(abs(v[-240:,:,:3]))),
    last_second_peak_angular_speed_rad_s=float(np.max(abs(v[-240:,:,3:]))),
    native_break_events=json.loads((out/'break-events.json').read_text()),contact_pairs=pairs,official_run=official,
    environment_contact_coverage=environment_coverage,
    official_live_replay_agreement=score_agreement,fullscore_demonstrated=full_verified,
    robot_opening_demonstrated=bool(full_verified and np.any(z['contact_counts']>0)),material_fidelity_passed=False,
    limitation='Full score, when verified, establishes this mechanical surrogate and known-layout controller only; it does not validate paperboard material fidelity.')
(out/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('hinge_closure','apex_sampled_mm','sdk_errors')},indent=2))
raise SystemExit(0 if not errors else 2)
