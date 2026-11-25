"""Parallel native search. Candidate rollouts need a separate single-scene scored replay."""
import json
import time
from pathlib import Path

import numpy as np
import torch
from yam_arena.urdf import quat_xyzw_to_matrix

from config import CLIP_Z, PLUG_X, SERVICE, TARGET, USED, port
from feedback import retaining_face
from pinch import make_policy


def cpu(tensor):
    tensor = tensor.torch if hasattr(tensor,'torch') else tensor
    return tensor.detach().cpu().numpy()


def observations(env, origins):
    world=env._base.arena_world
    names=[f'plug_{i}' for i in USED]+[f'clip_{SERVICE}']
    parts=[]
    for name in names:
        t=world.get_pose_w(name)
        parts.append(t.torch if hasattr(t,'torch') else t)
    poses=cpu(torch.stack(parts, dim=1)).astype(float)
    poses[:,:,:3]-=origins[:,None,:]
    joints=np.stack([cpu(env._base.scene[side+'_robot'].data.joint_pos) for side in ('left','right')],axis=1)
    return joints,poses


def run_trials(env, out, specs):
    root=Path(out)
    origins=cpu(env._base.scene.env_origins)
    batch=len(origins)
    for first in range(0,len(specs),batch):
        if (root.parent/'stop-tuning').exists():
            break
        env._env.reset()
        env._t=0.
        chunk=specs[first:first+batch]
        workers=[make_policy(env,spec) for spec in chunk]
        actions=[[] for _ in chunk]
        signals=[dict(peak_mm=0.,withdrawn_mm=0.,neighbour_mm=0.,target_hold_ticks=0,released=False,pulled=False) for _ in chunk]
        last=[None for _ in chunk]
        active=set(range(len(chunk)))
        start=time.monotonic()
        for i,spec in enumerate(chunk):
            folder=root/f'trial-{first+i:03d}'
            folder.mkdir()
            (folder/'plan.json').write_text(json.dumps(spec))
        joint,poses=observations(env,origins)
        with (root/f'wave-{first:03d}.jsonl').open('w', buffering=1) as stream:
            for tick in range(2400):
                cmd=np.tile(workers[0].ready,(batch,1))
                reports={}
                for i in active:
                    action,report=workers[i].command(tick/20,(joint[i] if chunk[i].get('style')=='boot_press' else joint[i,1],poses[i,0],poses[i,-1]))
                    cmd[i]=action
                    actions[i].append(np.asarray(action,dtype=np.float32))
                    reports[i]=report
                env._env.step(torch.as_tensor(cmd,device=env._base.device,dtype=torch.float32))
                env._t=(tick+1)/20
                joint,poses=observations(env,origins)
                for i in list(active):
                    report=reports[i];s=signals[i]
                    plug,clip=poses[i,0],poses[i,-1]
                    rot=quat_xyzw_to_matrix(plug[3:])
                    travel=CLIP_Z-(rot.T@(clip[:3]-plug[:3]))[2]
                    tip=plug[:3]+rot[:,0]*.012
                    s['peak_mm']=max(s['peak_mm'],1000*travel)
                    s['withdrawn_mm']=max(s['withdrawn_mm'],1000*(PLUG_X-plug[0]))
                    s['neighbour_mm']=max(s['neighbour_mm'],1000*(PLUG_X-poses[i,1:-1,0].min()))
                    s['released']|=bool(travel>=.0018 and tip[0]>.4045 and np.linalg.norm(plug[1:3]-np.array(port(SERVICE))[1:3])<.007)
                    s['pulled']|=bool(s['released'] and tip[0]<.3965)
                    retained=retaining_face(clip,TARGET)['retained']
                    speed=0 if last[i] is None else 20*np.linalg.norm(last[i]-plug[:3])
                    last[i]=plug[:3].copy()
                    s['target_hold_ticks']=s['target_hold_ticks']+1 if (s['pulled'] and retained and travel<.0003 and np.linalg.norm(plug[1:3]-np.array(port(TARGET))[1:3])<.001 and speed<.01 and report['phase']=='rest') else 0
                    event=dict(trial=first+i,t=env._t,phase=report['phase'],failure=report['failure'],travel_mm=1000*travel,
                        plug_pose=plug.tolist(),clip_pose=clip.tolist(),tcp=report['tcp'],target=report['target'],
                        opening_mm=report['opening_command_mm'],jaws_mm=report['jaws_mm'],retained=bool(retained),**s)
                    if travel>.0026 or s['neighbour_mm']>8. or plug[2]<.7275:
                        event['failure']='native overtravel, drop or disconnected neighbour'
                    done=event['failure'] or report['complete'] or tick==2399
                    if tick%5==0 or done:
                        stream.write(json.dumps(event)+'\n')
                    if done:
                        active.remove(i)
                        event.update(elapsed_s=time.monotonic()-start,action_count=len(actions[i]),candidate_for_replay=bool(s['target_hold_ticks']>=40 and not event['failure']),plan=chunk[i])
                        folder=root/f'trial-{first+i:03d}'
                        (folder/'trial.json').write_text(json.dumps(event,indent=2)+'\n')
                        np.save(folder/'actions.npy',np.stack(actions[i]))
                        if event['candidate_for_replay']:
                            (root/'candidate.json').write_text(json.dumps({'trial':folder.name,'plan':chunk[i]},indent=2))
                if not active:
                    break
        if (root/'candidate.json').exists():
            break
