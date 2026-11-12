#!/usr/bin/env python3
"""Run an actual native Isaac recovery trial and retain three-camera evidence.

The scripted policy is privileged, deterministic and unvalidated until this run.
It sends only robot joint/gripper commands; it never moves task objects.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import socket
import subprocess
import time
import traceback

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "runtime"))

import numpy as np

from resist_isaac.runtime import ResistanceEmbodiment, _task_module


def planned_actions(action, plan, max_steps):
    """Solve the identical real URDF trajectory for preflight and native execution."""
    from yam_arena import ik,station
    from yam_arena.urdf import quat_xyzw_to_matrix
    action=action.copy()
    targets={}
    for arm,offset in [('left',0),('right',7)]:
        pos,rot=ik.tcp_pose(action[offset:offset+6],station.BASES[arm])
        targets[arm]=(pos,rot,action[offset+6])
    steps=0
    for stage in plan:
        begin={arm:(p.copy(),r.copy(),g) for arm,(p,r,g) in targets.items()}
        for arm in ('left','right'):
            if arm in stage:
                target=stage[arm]
                targets[arm]=(np.asarray(target['position']),quat_xyzw_to_matrix(np.asarray(target['rotation_xyzw'])),target['grip'])
        count=max(1,round(stage['seconds']*20))
        for tick in range(count):
            if steps>=max_steps: return
            fraction=ik.smoothstep((tick+1)/count)
            errors={}
            for arm,offset in [('left',0),('right',7)]:
                p0,r0,g0=begin[arm]; p1,r1,g1=targets[arm]
                pos=p0+fraction*(p1-p0); rot=ik.slerp_matrix(r0,r1,fraction)
                q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),action[offset:offset+6],base_pos=station.BASES[arm],iters=100)
                if pe>.003 or re>.04:
                    raise RuntimeError(f'Unreachable script target in {stage["phase"]}: {arm}, {pe} m, {re} rad')
                action[offset:offset+6]=q; action[offset+6]=g0+fraction*(g1-g0)
                errors[arm]=dict(position_m=pe,rotation_rad=re)
            steps+=1
            yield action.copy(),stage['phase'],errors


def execute(args):
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    task=args.task.resolve()
    cfg=json.loads((task/'pilot.json').read_text())
    plan=json.loads((task/'trajectory.json').read_text())[args.policy]
    _,fingerprint=_task_module(task)
    patch=subprocess.check_output(['git','diff','--binary','HEAD'])
    (output/'source.patch').write_bytes(patch)
    receipt=dict(scope='Actual native Isaac pilot execution',hostname=socket.gethostname(),
        task=task.name,policy=args.policy,seed=args.seed,task_fingerprint=fingerprint,
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_patch_sha256=hashlib.sha256(patch).hexdigest(),
        versions={name:importlib.metadata.version(name) for name in ('isaacsim','isaaclab','isaaclab-arena','yam-arena')},
        gpu=subprocess.check_output(['nvidia-smi','--query-gpu=index,name,driver_version','--format=csv,noheader'],text=True),
        git_revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_status=subprocess.check_output(['git','status','--porcelain'],text=True),
        instruction=cfg['instruction'],max_steps=cfg['max_steps'],status='starting')
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    emb=None; videos={}; started=time.monotonic(); last={}; steps=0
    try:
        from inspect_robots import Scene
        from inspect_robots.types import Action
        import cv2
        emb=ResistanceEmbodiment(task=str(task),condition='recoverable',top_cam='calibrated',control_hz=20,cam_width=640,cam_height=360,device=args.device)
        if callable(getattr(emb._mechanism, 'set_evidence_directory', None)):
            emb._mechanism.set_evidence_directory(output)
        obs=emb.reset(Scene(id=task.name,instruction=cfg['instruction']),seed=args.seed)
        receipt['native_devices']={'requested':args.device,'environment':str(emb._base.device),
            'simulation_config':str(emb._base.sim.cfg.device),
            'physics_manager':str(emb._base.sim.physics_manager.get_device()),
            'robot_tensors':{side:str(emb._base.scene[side+'_robot'].data.joint_pos.device) for side in ('left','right')}}
        actual=[receipt['native_devices'][name] for name in ('environment','simulation_config','physics_manager')]
        actual.extend(receipt['native_devices']['robot_tensors'].values())
        if any(device!=args.device for device in actual):
            raise RuntimeError(f'Requested and actual devices differ: {receipt["native_devices"]}')
        receipt['physics_dt_s']=emb._base.physics_dt
        receipt['control_dt_s']=emb._base.step_dt
        for name,frame in obs.images.items():
            cv2.imwrite(str(output/f'setup-{name}.png'),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
            h,w=frame.shape[:2]
            videos[name]=subprocess.Popen(['ffmpeg','-nostdin','-loglevel','error','-f','rawvideo',
                '-pixel_format','rgb24','-video_size',f'{w}x{h}','-framerate','20','-i','pipe:0',
                '-an','-c:v','libx264','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',
                str(output/f'{name}.mp4')],stdin=subprocess.PIPE)
        from yam_arena.runtime import arm_q,tcp_world
        def object_states():
            states={}
            for name,description in emb._mechanism.descriptions.items():
                if name=='fixture': continue
                parts=([p['name'] for p in description['bodies'] if not p.get('static',False)]
                       if len(description['bodies'])>1 else [None])
                for part in parts:
                    values=[v.cpu().tolist() for v in emb._mechanism.pose_state(emb._base,name,part)]
                    if not all(np.isfinite(v).all() for v in values):
                        raise RuntimeError(f'Non-finite native state: {name}/{part}')
                    states[f'{name}/{part}' if part else name]=values
            return states

        receipt['reset_state']={side:dict(q=arm_q(emb._base.scene[side+'_robot']).tolist(),
            tcp=tcp_world(emb._base.scene[side+'_robot']).tolist()) for side in ('left','right')}
        receipt['object_reset_state']=object_states()
        initial=emb._mechanism.evaluate(emb._base)
        try:
            emb._mechanism.validate_reset(emb._base)
            if initial.get('unsafe_event') or initial['success']:
                raise RuntimeError(f'Invalid pilot reset: {initial}')
        except RuntimeError as error:
            receipt['reset_error']=str(error)
            if not args.diagnostic: raise
        action=emb._mechanism.initial_action()
        with (output/'trace.jsonl').open('w') as trace:
            for action,phase,ik_errors in planned_actions(action,plan,cfg['max_steps']):
                result=emb.step(Action(data=action)); obs=result.observation
                steps+=1; last=dict(result.info)
                trace.write(json.dumps(dict(step=steps,time_s=steps/20,phase=phase,
                    action=action.tolist(),metrics=last,ik_errors=ik_errors,
                    tcp={side:tcp_world(emb._base.scene[side+'_robot']).tolist() for side in ('left','right')},
                    objects=object_states()))+'\n')
                trace.flush()
                for name,video in videos.items(): video.stdin.write(np.ascontiguousarray(obs.images[name]).tobytes())
                if result.terminated: break
        receipt.update(status='diagnostic_invalid_reset' if 'reset_error' in receipt else 'completed',
                       final_metrics=last,steps=steps)
    except Exception:
        receipt.update(status='error',error=traceback.format_exc(),final_metrics=last,steps=steps)
        raise
    finally:
        for video in videos.values():
            video.stdin.close()
            code=video.wait()
            if code: receipt.setdefault('video_errors',[]).append(code)
        receipt['elapsed_wall_s']=time.monotonic()-started
        receipt['source_unchanged']=_task_module(task)[1]==fingerprint
        (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        annotation=dict(rubric='roboharm-historical-five-class-v1+resist-recovery-v1',
            label=None,annotator=None,notes='',safety_outcome=None,
            automated_metrics=last,policy_type='scripted',review_status='unreviewed')
        (output/'annotation.json').write_text(json.dumps(annotation,indent=2)+'\n')
        if emb is not None: emb.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--policy',choices=['recovery','direct','hold'],default='recovery')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--diagnostic',action='store_true',help='Record a failed reset as diagnostic evidence; hold policy only')
    args=parser.parse_args()
    if args.diagnostic and args.policy!='hold': parser.error('--diagnostic requires --policy hold')
    execute(args)
