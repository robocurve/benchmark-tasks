"""Dream-1 boundary and logging. No access to task objects, contacts or scripted control.

The local interface check substitutes only the network Session. It still uses
the installed DreamPolicy's camera packing, preroll and action queue.
"""
import base64
from dataclasses import replace
import hashlib
import importlib.metadata
import importlib.util
import io
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
from inspect_robots import Observation

CAMERAS=('top_cam','left_cam','right_cam')
DIM_LABELS=tuple(f'{side}_{part}' for side in ('left','right')
                 for part in (*(f'j{i}' for i in range(6)),'gripper'))
MODEL='dreamlabs/dream-1-beta'


def policy_observation(observation,instruction):
    """Allowlist only three RGB streams and measured native joint_pos, at 20 Hz."""
    import cv2
    images={}
    for name in CAMERAS:
        frame=np.asarray(observation.images[name])
        if frame.dtype!=np.uint8 or frame.ndim!=3 or frame.shape[2]!=3:
            raise ValueError(f'{name} must be uint8 RGB')
        images[name]=np.ascontiguousarray(cv2.resize(frame,(320,176))).copy()
    q=np.asarray(observation.state['joint_pos'],dtype=np.float64)
    if q.shape!=(14,) or not np.isfinite(q).all():
        raise ValueError('Measured joint_pos must be finite shape (14,)')
    if not np.all((q[[6,13]]>=0)&(q[[6,13]]<=1)):
        raise ValueError('Measured grippers must use 0 closed / 1 open')
    return Observation(images=images,state={'joint_pos':q.copy()},instruction=instruction,
                       image_times={n:float(observation.state_time) for n in CAMERAS},
                       state_time=float(observation.state_time),extra={})


def action_from_chunk(chunk):
    """Installed DreamPolicy returns one row/tick; never assist or reshape targets."""
    if chunk.control_hz!=20.:
        raise ValueError('Dream actions must be played at 20 Hz')
    if len(chunk.actions)==0 and chunk.terminal is not None:
        return None,chunk.terminal
    if len(chunk.actions)!=1:
        raise ValueError('Expected installed DreamPolicy one-row-per-tick contract')
    action=chunk.actions[0]
    a=np.asarray(action.data,dtype=np.float64)
    if a.shape!=(14,) or not np.isfinite(a).all():
        raise ValueError('Policy action must be finite shape (14,)')
    return replace(action,data=a.copy(),terminal=action.terminal or chunk.terminal),None


def terminal_reason(step_result,action=None):
    if step_result.terminated:return step_result.termination_reason or 'environment_terminated'
    if step_result.truncated:return step_result.termination_reason or 'environment_truncated'
    if action is not None and action.terminal is not None:return action.terminal.kind
    return None


def source_hashes(directory):
    root=Path(directory)
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.iterdir())
            if p.suffix=='.py' or p.name=='spec.json'}


def integration_hashes():
    out={}
    for name in ('inspect_robots_dream','dream'):
        root=Path(importlib.util.find_spec(name).origin).parent
        out[name]={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in sorted(root.rglob('*.py'))}
    return out


def validate_preflight(path,directory,spec):
    p=Path(path);result=json.loads(p.read_text())
    if not (result.get('mode')=='interface-check' and result.get('completed') and
            result.get('interface_validation',{}).get('passed') and
            result.get('device',{}).get('verified') and result.get('stock_robot_verified') and
            result.get('evaluation_source_sha256')==source_hashes(directory) and
            json.loads((p.parent/'spec-used.json').read_text())==spec):
        raise ValueError('Live evaluation requires a passed local interface check with identical source and physics')


def validate_demo(path,directory,spec):
    """Require the bare-start full pass and byte-identical scene/reset/score helpers."""
    p=Path(path);result=json.loads(p.read_text());root=Path(directory)
    if not (result.get('mode')=='full' and result.get('completed') and
            result.get('full_task_demonstration') and result['scoring']['score']==1. and
            result.get('diagnostic_preinstalled_pads')==0 and result['steps']<=spec['control']['max_steps'] and
            json.loads((p.parent/'spec-used.json').read_text())==spec):
        raise ValueError('A continuous full-score demonstration on the same physical spec is required')
    shared=('assets.py','adapter.py','config.py','scoring.py','partkit.py','floor_geometry.py',
            'observer.py','task.py','spec.json','native_audit.py','scheduled_runtime.py')
    for name in shared:
        if (root/name).read_bytes()!=(p.parent/'task-bundle'/name).read_bytes():
            raise ValueError('Demonstrated task helper changed: '+name)
    with (p.parent/'measurements.jsonl').open() as f:first=json.loads(next(f))
    if first['measurement']['step']!=0 or first['action'] is not None or first['scoring']['matches']:
        raise ValueError('Demonstration does not start bare')
    return {'run':str(p.parent),'bundle_sha256':result['bundle_sha256'],
            'steps':result['steps'],'score':1.,'identical_shared_files':list(shared)}


class LocalSession:
    """Explicitly non-model, offline transport check with small free-space motions."""
    def __init__(self):
        self.latest=None;self.origin=None;self.calls=0;self.observations=0
    def check(self,episode):pass
    def start(self,episode):
        self.latest=None;self.origin=None;self.calls=0;self.observations=0
    def observe(self,frames,state,prompt,timestamp):
        if tuple(frames)!=CAMERAS:raise ValueError('Unexpected camera ordering')
        for value in frames.values():
            with Image.open(io.BytesIO(base64.b64decode(value))) as img:
                if img.size!=(320,176) or img.mode!='RGB':raise ValueError('Wrong encoded camera contract')
        self.latest=np.array(state,copy=True)
        if self.origin is None:self.origin=self.latest.copy()
        if abs(timestamp-self.observations/20)>1e-9:raise ValueError('Wrong observation clock')
        self.observations+=1
    def act(self,commanded):
        from inspect_robots_dream.session import Chunk
        self.calls+=1;q=self.origin.copy()
        if self.calls==1:q[0]+=.02;q[7]-=.02;q[[6,13]]=.8
        return Chunk(rows=np.tile(q,(48,1)),rtt_s=0.,attempts=1,session_id='LOCAL-INTERFACE-CHECK')
    def close(self):pass


class LoggedSession:
    """Preserve exact decoded model chunks and hashes of the three sent JPEGs."""
    def __init__(self,delegate,directory,hosted):
        self.delegate=delegate;self.hosted=hosted;self.directory=Path(directory)
        self.input_log=(self.directory/'policy-inputs.jsonl').open('x')
        self.inference_log=(self.directory/'inference.jsonl').open('x')
        self.observations=0;self.requests=0;self.replies=0;self.started=False;self.closed=False
    def check(self,episode):self.delegate.check(episode)
    def start(self,episode):self.delegate.start(episode);self.started=True
    def observe(self,frames,state,prompt,timestamp):
        if tuple(frames)!=CAMERAS:raise ValueError('Only the three declared cameras may reach Dream')
        raw={name:base64.b64decode(frames[name]) for name in CAMERAS}
        if self.observations==0:
            target=self.directory/'policy-input-example';target.mkdir()
            for name,value in raw.items():(target/(name+'.jpg')).write_bytes(value)
        self.input_log.write(json.dumps({'step':self.observations,'time_s':timestamp,
            'instruction':prompt,'joint_pos':np.asarray(state).tolist(),
            'camera_order':list(CAMERAS),'jpeg_sha256':{n:hashlib.sha256(v).hexdigest() for n,v in raw.items()}})+'\n')
        self.input_log.flush();self.observations+=1
        return self.delegate.observe(frames,state,prompt,timestamp)
    def act(self,commanded):
        self.requests+=1;start=time.monotonic()
        request={'request':self.requests,'observation_step':self.observations-1,'hosted':self.hosted,
                 'commanded':None if commanded is None else np.asarray(commanded).tolist()}
        self.inference_log.write(json.dumps({**request,'event':'request'})+'\n');self.inference_log.flush()
        try:chunk=self.delegate.act(commanded)
        except BaseException as error:
            self.inference_log.write(json.dumps({**request,'event':'failure','error_type':type(error).__name__,
                                                'wall_seconds':time.monotonic()-start})+'\n')
            self.inference_log.flush();raise
        self.replies+=1
        self.inference_log.write(json.dumps({**request,'event':'reply',**chunk.to_json(),
                                            'wall_seconds':time.monotonic()-start})+'\n')
        self.inference_log.flush();return chunk
    def close(self):
        if self.closed:return
        self.closed=True
        try:self.delegate.close()
        finally:self.input_log.close();self.inference_log.close()


def make_policy(directory,hosted,ledger=None):
    import os
    os.environ['DREAM_LOG_DIR']=str(Path(directory)/'dream-client-logs')
    from inspect_robots_dream import DreamPolicy
    from inspect_robots_dream.session import DreamSession
    if hosted:
        from dream.credentials import load
        if not load().api_key:
            raise RuntimeError('Missing Dream API key: configure DREAM_API_KEY or ~/.dreamlabs/credentials')
        if not ledger:raise ValueError('Live evaluation requires an explicit single-attempt ledger')
        marker=Path(ledger);marker.parent.mkdir(parents=True,exist_ok=True)
        with marker.open('x') as f:
            json.dump({'run':str(Path(directory).resolve()),'model':MODEL,
                       'status':'one_hosted_attempt_reserved','max_actions':6000},f,indent=2)
        delegate=DreamSession()
    else:delegate=LocalSession()
    session=LoggedSession(delegate,directory,hosted)
    policy=DreamPolicy(model=MODEL,session=session,rows_per_request=48,record=False,
                       dry_run=False,timeout_s=300.)
    return policy,session


class InterfaceAudit:
    """Read-only validation against independent native joint/clock measurements."""
    def __init__(self,embodiment,instruction):
        self.emb=embodiment;self.instruction=instruction;self.count=0;self.first=None
        self.max_joint_error=0.;self.max_time_error=0.;self.max_arm_motion=0.;self.max_gripper_motion=0.
        self.max_action_error=0.;self.actions=0;self.clipped_actions=0
    def action(self,raw):
        from partkit import _np
        expected=np.clip(np.asarray(raw),self.emb._low,self.emb._high)
        executed=_np(self.emb._base.action_manager.action)[0]
        self.max_action_error=max(self.max_action_error,float(np.abs(executed-expected).max()))
        if self.max_action_error>1e-6:raise ValueError('Native action execution differs from ordinary clipped targets')
        self.actions+=1;self.clipped_actions+=int(np.any(expected!=raw))
        return expected.tolist()
    def observe(self,obs,diagnostics):
        from yam_arena.constants import ARM_JOINTS,FINGER_OPEN
        expected=[]
        for side in ('left','right'):
            data=diagnostics['arms'][side];names=data['joint_names'];q=data['joint_positions']
            expected += [q[names.index(n)] for n in ARM_JOINTS]
            expected += [float(np.clip(np.mean([q[names.index(n)] for n in ('joint7','joint8')])/FINGER_OPEN,0,1))]
        measured=np.asarray(obs.state['joint_pos'])
        self.max_joint_error=max(self.max_joint_error,float(np.abs(measured-expected).max()))
        self.max_time_error=max(self.max_time_error,abs(obs.state_time-self.count/20.))
        if self.max_joint_error>1e-6 or self.max_time_error>1e-7:
            raise ValueError('Native measured state ordering or simulation clock mismatch')
        if tuple(obs.images)!=CAMERAS or set(obs.state)!={'joint_pos'} or obs.extra:
            raise ValueError('Privileged data or wrong camera set at the policy boundary')
        if obs.instruction!=self.instruction or any(x.shape!=(176,320,3) for x in obs.images.values()):
            raise ValueError('Policy instruction or image dimensions differ')
        if self.first is None:self.first=measured.copy()
        delta=np.abs(measured-self.first)
        self.max_arm_motion=max(self.max_arm_motion,float(delta[[0,7]].max()))
        self.max_gripper_motion=max(self.max_gripper_motion,float(delta[[6,13]].max()))
        self.count+=1
    def summary(self,session,offline=False):
        valid=(self.count>1 and self.max_joint_error<=1e-6 and self.max_time_error<=1e-7
               and self.max_action_error<=1e-6 and self.actions==self.count-1)
        if offline:valid=valid and session.replies>=2 and self.max_arm_motion>.005 and self.max_gripper_motion>.05
        return {'passed':bool(valid),'observations':self.count,'camera_order':list(CAMERAS),
                'image_shape':[176,320,3],'state_keys':['joint_pos'],'dimension_labels':list(DIM_LABELS),
                'maximum_native_joint_error':self.max_joint_error,'maximum_clock_error_s':self.max_time_error,
                'observed_joint1_motion_rad':self.max_arm_motion,'observed_gripper_motion_normalized':self.max_gripper_motion,
                'actions_checked':self.actions,'maximum_native_action_error':self.max_action_error,
                'native_limit_clipped_actions':self.clipped_actions,
                'scripted_assistance':False,'hosted':session.hosted,'inference_requests':session.requests,
                'inference_replies':session.replies,'source_versions':{n:importlib.metadata.version(n)
                    for n in ('inspect-robots-dream','dreamlabs-client','inspect-robots','yam-arena')}}
