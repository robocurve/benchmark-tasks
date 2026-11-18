#!/usr/bin/env python3
"""Run the native task and retain three-camera evidence.

The scripted policy is privileged, deterministic and unvalidated until this run.
It sends only robot joint/gripper commands; it never moves task objects.
Record the full authored trajectory, including retreat after first success.
"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
import traceback

def planned_actions(action, plan, max_steps):
    """Solve the identical real URDF trajectory for preflight and native execution."""
    if (isinstance(plan,(list,tuple))
            and sum(max(1,round(stage['seconds']*20)) for stage in plan)>max_steps):
        raise ValueError('Authored trajectory exceeds its action budget; refusing to truncate the ending')
    from yam_arena import ik,station
    from yam_arena.urdf import quat_xyzw_to_matrix
    import numpy as np
    action=action.copy()
    targets={}
    for arm,offset in [('left',0),('right',7)]:
        pos,rot=ik.tcp_pose(action[offset:offset+6],station.BASES[arm])
        targets[arm]=(pos,rot,action[offset+6])
    steps=0
    for stage in plan:
        count=max(1,round(stage['seconds']*20))
        if steps+count>max_steps:
            raise ValueError('Scripted trajectory exceeds its action budget; refusing to truncate the ending')
        begin={arm:(p.copy(),r.copy(),g) for arm,(p,r,g) in targets.items()}
        for arm in ('left','right'):
            if arm in stage:
                target=stage[arm]
                targets[arm]=(np.asarray(target['position']),quat_xyzw_to_matrix(np.asarray(target['rotation_xyzw'])),target['grip'])
        for tick in range(count):
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


def source_files(task):
    return sorted(p for p in task.rglob('*') if p.is_file() and '__pycache__' not in p.parts)


def source_snapshot(task, output):
    """Retain the entire task package, including new/untracked portable sources."""
    task,output=Path(task).resolve(),Path(output).resolve()
    if output==task or task in output.parents:
        raise ValueError('Evidence output must be outside the task source folder')
    hashes={}
    for path in source_files(task):
        relative=path.relative_to(task)
        destination=output/'source'/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,destination)
        hashes[relative.as_posix()]=hashlib.sha256(destination.read_bytes()).hexdigest()
    return hashes


def git_metadata(task, output):
    """Git is optional: the full copied source and hashes remain authoritative."""
    env={**os.environ,'GIT_OPTIONAL_LOCKS':'0'}
    def read(*arguments):
        return subprocess.run(['git','-C',str(task),*arguments],capture_output=True,env=env)
    result=dict(git_available=False,git_revision=None,git_status=None,source_patch_sha256=None)
    try:
        inside=read('rev-parse','--is-inside-work-tree')
    except FileNotFoundError:
        return result
    if inside.returncode or inside.stdout.strip()!=b'true':return result
    revision=read('rev-parse','HEAD')
    status=read('status','--porcelain','--','.')
    patch=read('diff','--binary','HEAD','--','.')
    result.update(git_available=True,
        git_revision=revision.stdout.decode().strip() if revision.returncode==0 else None,
        git_status=status.stdout.decode() if status.returncode==0 else None)
    if patch.returncode==0:
        (Path(output)/'source.patch').write_bytes(patch.stdout)
        result['source_patch_sha256']=hashlib.sha256(patch.stdout).hexdigest()
    return result


def execute(args):
    if socket.gethostname()!=args.expected_hostname:
        raise RuntimeError(f'Expected host {args.expected_hostname!r}, got {socket.gethostname()!r}')
    import numpy as np
    from resist_isaac.runtime import ResistanceEmbodiment, _task_module

    output=args.output.resolve()
    task=args.task.resolve()
    if output==task or task in output.parents:
        raise ValueError('Evidence output must be outside the task source folder')
    output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((task/'pilot.json').read_text())
    plan_file=task/'trajectory.json'
    plan=json.loads(plan_file.read_text()).get(args.policy) if plan_file.is_file() else None
    _,fingerprint=_task_module(task)
    source_hashes=source_snapshot(task,output)
    receipt=dict(scope='Actual native Isaac execution',hostname=socket.gethostname(),
        task=task.name,policy=args.policy,seed=args.seed,task_fingerprint=fingerprint,
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        versions={name:importlib.metadata.version(name) for name in ('isaacsim','isaaclab','isaaclab-arena','yam-arena')},
        gpu=subprocess.check_output(['nvidia-smi','--query-gpu=index,name,uuid,driver_version','--format=csv,noheader'],text=True),
        instruction=cfg['instruction'],max_steps=cfg['max_steps'],status='starting',
        python_executable=sys.executable,source_files_sha256=source_hashes,
        cuda_visible_devices=os.environ.get('CUDA_VISIBLE_DEVICES'))
    receipt.update(git_metadata(task,output))
    if args.design_review is not None:
        review=args.design_review.resolve()
        review_bytes=review.read_bytes()
        review_data=json.loads(review_bytes)
        if (not isinstance(review_data,dict) or review_data.get('task')!=task.name
                or review_data.get('instruction')!=cfg['instruction']):
            raise ValueError('Design review must match the exact task and registered instruction')
        (output/'design-review.json').write_bytes(review_bytes)
        receipt['design_review_sha256']=hashlib.sha256(review_bytes).hexdigest()
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    emb=None; videos={}; started=time.monotonic(); last={}; steps=0
    try:
        from inspect_robots import Scene
        from inspect_robots.types import Action
        import cv2
        emb=ResistanceEmbodiment(task=str(task),condition='recoverable',control_hz=20,
                                cam_width=1280,cam_height=720,device=args.device,
                                top_cam='calibrated',wrist_cam_mount='link')
        if hasattr(emb._mechanism,'overview_camera'):
            raise ValueError('This task requires the unchanged calibrated station camera')
        receipt['device']=emb.device
        if callable(getattr(emb._mechanism,'set_evidence_directory',None)):
            emb._mechanism.set_evidence_directory(output)
        obs=emb.reset(Scene(id=task.name,instruction=cfg['instruction']),seed=args.seed)
        if set(obs.images)!={'top_cam','left_cam','right_cam'}:
            raise RuntimeError(f'Missing standard camera observations: {list(obs.images)}')
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
        from yam_arena.constants import FINGER_JOINTS
        from resist_isaac.state import tensor
        receipt['native_devices']=dict(requested=str(emb.device),environment=str(emb._base.device),
            simulation_config=str(emb._base.sim.cfg.device),
            physics_manager=str(emb._base.sim.physics_manager.get_device()),
            robot_tensors={side:str(tensor(emb._base.scene[side+'_robot'].data.joint_pos).device)
                          for side in ('left','right')})
        actual_devices=[receipt['native_devices'][key] for key in
                        ('environment','simulation_config','physics_manager')]
        actual_devices.extend(receipt['native_devices']['robot_tensors'].values())
        if any(device!=str(emb.device) for device in actual_devices):
            raise RuntimeError(f'Requested and actual native devices differ: {receipt["native_devices"]}')
        finger_ids={side:emb._base.scene[side+'_robot'].find_joints(FINGER_JOINTS,preserve_order=True)[0]
                    for side in ('left','right')}

        def gripper_states():
            """Native 20 Hz finger state and command; this does not measure contact force."""
            states={}
            for side,ids in finger_ids.items():
                robot=emb._base.scene[side+'_robot']
                values={key:tensor(getattr(robot.data,field))[0,ids].cpu().tolist()
                        for key,field in (('position_m','joint_pos'),('velocity_m_s','joint_vel'),
                                          ('position_target_m','joint_pos_target'))}
                if not all(np.isfinite(value).all() for value in values.values()):
                    raise RuntimeError(f'Non-finite native finger state: {side}')
                states[side]=dict(joint_names=[robot.joint_names[i] for i in ids],**values)
            return states

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
        receipt['gripper_reset_state']=gripper_states()
        receipt['object_reset_state']=object_states()
        initial=emb._mechanism.evaluate(emb._base)
        try:
            emb._mechanism.validate_reset(emb._base)
            if initial.get('unsafe_event') or initial['success']:
                raise RuntimeError(f'Invalid task reset: {initial}')
        except RuntimeError as error:
            receipt['reset_error']=str(error)
            if not args.diagnostic: raise
        action=emb._mechanism.initial_action()
        controller=getattr(emb._mechanism,'scripted_actions',None)
        if callable(controller):
            actions=controller(emb._base,args.policy,cfg['max_steps'])
            receipt['controller_scope']='Task-owned privileged native-state controller; robot commands only'
        elif plan is not None:
            actions=planned_actions(action,plan,cfg['max_steps'])
            receipt['controller_scope']='Preregistered URDF waypoint controller; robot commands only'
        else:
            raise ValueError(f'No implemented scripted policy {args.policy!r}')
        receipt['timing_scope']='Host monotonic intervals; environment step includes physics, scoring and camera observation; GPU kernel timing is not measured.'
        with (output/'trace.jsonl').open('w') as trace, (output/'control-timing.jsonl').open('w') as timings:
            for action,phase,ik_errors in actions:
                if steps>=cfg['max_steps']:
                    raise RuntimeError('Scripted policy exceeded the registered action budget')
                tick_start=time.monotonic()
                result=emb.step(Action(data=action)); obs=result.observation
                step_end=time.monotonic()
                steps+=1; last=dict(result.info)
                trace.write(json.dumps(dict(step=steps,time_s=steps/20,phase=phase,
                    action=action.tolist(),metrics=last,ik_errors=ik_errors,
                    q={side:arm_q(emb._base.scene[side+'_robot']).tolist() for side in ('left','right')},
                    tcp={side:tcp_world(emb._base.scene[side+'_robot']).tolist() for side in ('left','right')},
                    grippers=gripper_states(),
                    objects=object_states()))+'\n')
                trace.flush()
                trace_end=time.monotonic()
                for name,video in videos.items(): video.stdin.write(np.ascontiguousarray(obs.images[name]).tobytes())
                video_end=time.monotonic()
                timings.write(json.dumps(dict(step=steps,environment_step_wall_s=step_end-tick_start,
                    state_and_trace_wall_s=trace_end-step_end,video_write_wall_s=video_end-trace_end,
                    control_wall_s=video_end-tick_start))+'\n')
                timings.flush()
                if result.terminated:
                    receipt.setdefault('first_success_step', steps)
        receipt.update(status='diagnostic_invalid_reset' if 'reset_error' in receipt else 'completed',
                       final_metrics=last,steps=steps)
    except KeyboardInterrupt:
        receipt.update(status='interrupted',error=traceback.format_exc(),final_metrics=last,steps=steps)
        raise
    except Exception:
        receipt.update(status='error',error=traceback.format_exc(),final_metrics=last,steps=steps)
        raise
    finally:
        for name,video in videos.items():
            try:
                video.stdin.close()
            except OSError as error:
                receipt.setdefault('video_errors',[]).append(dict(camera=name,stdin_error=str(error)))
            code=video.wait()
            if code: receipt.setdefault('video_errors',[]).append(dict(camera=name,exit_code=code))
        if receipt.get('video_errors'):receipt['status']='error'
        receipt['elapsed_wall_s']=time.monotonic()-started
        try:
            current_hashes={path.relative_to(task).as_posix():hashlib.sha256(path.read_bytes()).hexdigest()
                            for path in source_files(task)}
            receipt['source_unchanged']=(_task_module(task)[1]==fingerprint and current_hashes==source_hashes)
        except Exception:
            receipt['source_unchanged']=False
            receipt['source_verification_error']=traceback.format_exc()
        if emb is not None:
            try:
                emb.close()
            except Exception:
                receipt['status']='error'
                receipt['close_error']=traceback.format_exc()
        receipt['evidence_sha256']={}
        for path in sorted(output.iterdir()):
            if path.is_file() and path.suffix in ('.jsonl','.mp4','.png','.npy','.gz','.csv'):
                with path.open('rb') as evidence:
                    receipt['evidence_sha256'][path.name]=hashlib.file_digest(evidence,'sha256').hexdigest()
        (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        annotation=dict(rubric=task.name,
            label=None,annotator=None,notes='',
            automated_metrics=last,policy_type='scripted',review_status='unreviewed')
        (output/'annotation.json').write_text(json.dumps(annotation,indent=2)+'\n')


def main(task):
    parser=argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('output',type=Path)
    parser.add_argument('--policy',default='full',help='Task-owned scripted policy; full is a requested run, not a success claim')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--device',default='cuda:0',help='Native control/readback device; recorded in the receipt')
    parser.add_argument('--expected-hostname',default='visnu',help='Refuse execution on a different host; set explicitly for another real rig')
    parser.add_argument('--design-review',type=Path,default=Path(task)/'design-review.json',
                        help='Retain the completed skeptical design review with the native receipt')
    parser.add_argument('--diagnostic',action='store_true',help='Record a failed reset as diagnostic evidence; hold policy only')
    args=parser.parse_args()
    args.task=Path(task)
    if args.diagnostic and args.policy!='hold': parser.error('--diagnostic requires --policy hold')
    execute(args)
