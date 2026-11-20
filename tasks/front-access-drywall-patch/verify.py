"""Replay a front-only repair from native poses, contact impulses and recorded fastening sites."""
import argparse, json
from itertools import groupby
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

from media import export_video, image_at
from grading import backing_rows


def audit(run,media=None):
    schema=json.loads((run/'contact-contract.json').read_text())
    archive=json.loads((run/'repair-result.json').read_text())
    cfg=json.loads((run/'configuration.json').read_text())
    step_count=cfg.get('steps')
    if step_count is None:
        step_count=len(json.loads((run/'plan.json').read_text())['steps'])
    official=list(run.glob('adhoc_*.json'))
    result=json.loads(next(f for f in official if '.live.' not in f.name).read_text())
    raw=np.load(run/'physics.npz')
    ids={p.split('/')[-1]:i for i,p in enumerate(schema['paths'])}
    hz=int(raw['physics_hz']);obs=raw['observation_samples'];pose=raw['poses'];vel=raw['velocities']
    f=raw['object_contact_forces'];robot=raw['robot_contact_counts']
    sensors=schema['sensors'];filters=schema['filters'];targets=[filters.index(schema['paths'][ids[n]]) for n in ('strip','spare_strip','patch')]
    samples=len(pose);assert np.isfinite(pose).all() and len(obs)==step_count+1
    assert result['status']=='success' and not result['results'].get('errored_trials')
    parts={name:pose[:,i] for name,i in ids.items()}
    transfer={}
    for name in ('strip','spare_strip','patch','driver','carrier'):
        index=filters.index(schema['paths'][ids[name]])
        holds={}
        for side in ('LeftArm','RightArm'):
            tips=[i for i,n in enumerate(sensors) if '/'+side+'/' in n and '/tip_' in n]
            assert len(tips)==2
            holds[side]=float(np.all(robot[:,tips,index]>0,axis=1).sum()/hz)
        transfer[name]=dict(initial=parts[name][0,:3].tolist(),final=parts[name][-1,:3].tolist(),two_finger_contact_s=holds)
    workspace=(run/'scene.usda').read_text()
    loose_start=('test_only_clamp_' not in workspace and
                 all(np.linalg.norm(parts[n][0,:3]-[.371,0,.9195])>.075 for n in ('strip','spare_strip','patch')))
    fasteners=archive['fasteners'];checkshots=[];mount_residual=[];retained_offsets=[]
    for shot in fasteners:
        end=shot['sample']
        i=end-1;head=np.array(shot['head']);axis=np.array(shot['axis'])
        sh=parts['shoe'][i];shell=parts['driver'][i]
        stroke=float((shell[:3]-sh[:3])@axis)
        wood=parts[shot['wood']][i];samples_in_wood=R.from_quat(wood[3:]).inv().apply(head+np.arange(.015,.031,.001)[:,None]*axis-wood[:3])
        bite=int(np.all(abs(samples_in_wood)<[.008,.1255-.003,.0125-.002],axis=1).sum())
        wrot=R.from_quat(wood[3:]); wn=wrot.apply([1.,0,0]);wn=wn if wn@axis>=0 else -wn;wf=wood[:3]-wn*.008
        substrate_gap=float(((wf-head)@wn)/(axis@wn)-.0127)
        index=filters.index(schema['paths'][ids['patch']]) if shot['front']=='patch' else filters.index('/World/envs/env_0/'+shot['front'])
        load=float(-f[i-11:i+1,ids['shoe'],index].mean(axis=0)@axis)
        # Recover cumulative roll relative to the casing from every native
        # spindle pose, including the true start of this individual shoe stroke.
        begin=max(0,i-8*hz)
        sh_hist=parts['shoe'][begin:i+1];h_hist=parts['driver'][begin:i+1];m=R.from_quat(h_hist[:,3:])
        axes=m.apply(np.repeat([[1.,0,0]],len(h_hist),axis=0))
        compression=np.sum((h_hist[:,:3]-sh_hist[:,:3])*axes,axis=1)
        relaxed=np.flatnonzero(compression<.001)
        begin+=int(relaxed[-1]) if len(relaxed) else 0
        relative=R.from_quat(parts['driver'][begin:i+1,3:]).inv()*R.from_quat(parts['spindle'][begin:i+1,3:])
        turns=float((relative[:-1].inv()*relative[1:]).as_rotvec()[:,0].sum()/(2*np.pi))
        local_anchor=wrot.inv().apply(head-wood[:3])
        final_wood=parts[shot['wood']][-hz:]
        current=final_wood[:,:3]+R.from_quat(final_wood[:,3:]).apply(local_anchor)
        reference=head
        if shot['front']=='patch':
            initial_panel=parts['patch'][i]
            a=R.from_quat(initial_panel[3:]).inv().apply(head-initial_panel[:3])
            final_panel=parts['patch'][-hz:]
            reference=final_panel[:,:3]+R.from_quat(final_panel[:,3:]).apply(a)
        offsets=np.linalg.norm(current-reference,axis=1)
        retained_offsets.append(offsets)
        mount_residual.append(float(np.max(offsets)))
        work=float(shot['wood_work_J'])
        ok=work>=1. and stroke>=.0305 and bite>=8 and load>.35 and turns>=9.7 and -.0015<=substrate_gap<=.0025
        checkshots.append(dict(wood=shot['wood'],front=shot['front'],action=int(np.searchsorted(obs,i)),stroke_mm=1000*stroke,wood_depth_samples_mm=bite,backing_gap_mm=1000*substrate_gap,pressure_N=load,native_spindle_turns=turns,recorded_wood_work_J=work,valid=bool(ok)))
    tail=parts['patch'][-hz:];rot=R.from_quat(tail[:,3:]).as_matrix()
    corners=np.array([[-.0127/2,y,z] for y in (-.0985,.0985) for z in (-.0985,.0985)])
    edges=tail[:,:3,None]+rot@corners.T
    flush=float(np.max(np.abs(edges[:,0]-.35)))
    seam_lo=float(min(np.min(.1-np.abs(edges[:,1])),np.min(.1-np.abs(edges[:,2]-.9195))))
    seam_hi=float(max(np.max(.1-np.abs(edges[:,1])),np.max(.1-np.abs(edges[:,2]-.9195))))
    drift=float(np.max(np.linalg.norm(edges-edges[0],axis=1)))
    clear=not np.any(robot[-hz:,:,targets])
    release=archive.get('key_release')
    tool_clear=release is not None and release['sample']<=samples-hz
    for n in ('driver','carrier','shoe','spindle'):
        if np.any(f[-hz:, [ids['strip'],ids['spare_strip'],ids['patch']],filters.index(schema['paths'][ids[n]])]):tool_clear=False
    proof=archive.get('proof_substeps',[])
    proof_windows=[[step for _,step in group] for _,group in groupby(enumerate(proof),key=lambda item:item[1]-item[0])]
    delivery={r['sample']:r for r in archive.get('delivered_proof',[])}
    proof_verified=False
    loaded_drift=None
    response_mm=None
    delivered_range=None
    for window in reversed(proof_windows):
        if len(window)!=hz:
            continue
        start=window[0]-hz//2
        stop=window[-1]+1+hz
        if start<0 or stop>len(pose):
            continue
        panel=parts['patch'][start:stop]
        full_edges=panel[:,:3,None]+R.from_quat(panel[:,3:]).as_matrix()@corners.T
        d=float(np.max(np.linalg.norm(full_edges-full_edges[0],axis=1)))
        if np.any(robot[start:stop,:,targets]):
            continue
        outside=release is not None and release['sample']<=start
        metal_contact=any(np.any(f[start:stop,[ids[n] for n in ('strip','spare_strip','patch')],filters.index(schema['paths'][ids[t]])]) for t in ('driver','carrier','shoe','spindle'))
        if not all(step in delivery for step in window):
            continue
        applied=np.array([np.subtract(delivery[i]['submitted_N'],delivery[i]['fasteners_N']) for i in window])
        if not np.allclose(applied,[2.,0.,0.],rtol=0.,atol=2.e-6):
            continue
        before=parts['patch'][window[0]-hz//4:window[0],0].mean(dtype=float)
        loaded=parts['patch'][window[-1]-hz//4+1:window[-1]+1,0].mean(dtype=float)
        after=parts['patch'][stop-hz//4:stop,0].mean(dtype=float)
        if outside and not metal_contact and d<=.001:
            proof_verified=True;loaded_drift=1000*d
            delivered_range=[float(applied[:,0].min()),float(applied[:,0].max())]
            response_mm=dict(under_load=float(1000*(loaded-before)),after_unload=float(1000*(after-before)))
            break
    mounts=archive['connections']
    # Basic backing credit is an end-state measurement. The tightening system
    # measures its anchor separation immediately before the final physics step.
    # Partial panel credit additionally requires the whole final second.
    def coverage(sample):
        s=samples-hz+sample
        sites=[]
        mounts_at_sample=[]
        for shot,m,offsets in zip(fasteners,mounts,retained_offsets,strict=True):
            if shot['sample']<=s+1:
                sites.append(shot)
                mounts_at_sample.append(dict(broken=m['broken'],gap=offsets[sample]))
        return backing_rows(sites,mounts_at_sample,.9195)
    rows,panel_rows,complete=coverage(hz-2)
    panel_retained=all(coverage(i)[:2]==(2,2) for i in range(0,hz,12))
    grade=float(result['results']['metrics']['task_score'])
    taped_fit=flush<.002 and seam_lo>=-.0005 and seam_hi<=.0035
    independent=.25*rows
    if rows==2 and panel_rows==2 and panel_retained and clear and tool_clear and taped_fit:
        independent=.75
        if complete and proof_verified:
            independent=1.
    verified=bool(loose_start and 0<step_count<=2400 and all(c['valid'] for c in checkshots)
                  and independent==grade and grade==archive['metrics']['score'] and len(obs)==step_count+1
                  and hz==240 and np.all(np.diff(obs)==12))
    if not verified:
        raise ValueError(f"Independent grade {independent} disagrees with reported grade {grade}, or evidence invalid")
    report=dict(actions=step_count,seconds=step_count/20,physics_hz=hz,native_score=grade,independently_verified=verified,replay_score=independent,backing_rows=rows,loose_stock_reset=loose_start,final_mount_residuals_mm=[1000*x[-2] for x in retained_offsets],peak_last_second_mount_residuals_mm=[1000*x for x in mount_residual],transfers=transfer,fastenings=checkshots,maximum_face_step_mm=1000*flush,edge_gap_range_mm=[1000*seam_lo,1000*seam_hi],last_1s_drift_mm=1000*drift,released=clear and tool_clear,proof_load_s=len(proof)/hz,complete_released_proof_window=proof_verified,peak_drift_across_proof_mm=loaded_drift,verified_net_added_x_force_range_N=delivered_range,mean_centre_x_response_mm=response_mm)
    if media:
        media.mkdir(parents=True,exist_ok=True)
        if cfg.get('mode')=='demo' and grade!=1.:raise ValueError('Full-score demonstration did not achieve 1.0')
        health={}
        for camera in ('top','left','right'):
            files=sorted((run/'frames').rglob(f'*_{camera}_cam_*.npy'))
            if len(files)!=len(obs):
                raise ValueError(f'{camera}: incomplete image sequence')
            lowest=float('inf')
            for index,path in enumerate(files):
                if not path.name.endswith(f'_{index:06d}.npy'):
                    raise ValueError(f'{camera}: missing frame {index}')
                im=np.load(path)
                if im.shape!=(176,320,3) or im.dtype!=np.uint8:
                    raise ValueError(f'{camera}: unexpected frame format')
                lowest=min(lowest,float(im.std()))
            if camera=='top' and lowest<=4:
                raise ValueError(f'{camera}: near-uniform camera frame')
            health[camera]=dict(frames=len(files),minimum_rgb_std=lowest)
        report['original_cameras']=health
        export_video(run,media/('dream.mp4' if cfg.get('mode')=='dream' else 'fullscore.mp4'),len(obs))
        image_at(run,0).save(media/'setup.png');image_at(run,len(obs)-1).save(media/'finished.png')
    (run/'repair-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--media',type=Path)
    a=p.parse_args();print(json.dumps(audit(a.run,a.media),indent=2))
