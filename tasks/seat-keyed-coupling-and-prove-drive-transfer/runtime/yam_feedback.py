"""Privileged scripted expert using measured prop poses and joint state.

Only joint targets are commanded. No prop pose or physics state is written.
This expert is for feasibility demonstrations. Dream receives camera frames and joints.
"""
import numpy as np
import yam_articulated_base as common
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import os,json
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.scripted import YamScriptedPolicy
from yam_arena import ik
from yam_arena.constants import PAD_TIP_PAST_TCP as D,FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix

class FeedbackExpert(YamScriptedPolicy):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.info=replace(self.info,name='yam_feedback')
    def act(self,observation):
        t=float(observation.state_time)+self._dt
        module=self._task.module
        states=getattr(common,'EXPERT_STATES',getattr(module,'_expert_states',{}))
        joints=getattr(common,'EXPERT_JOINTS',getattr(module,'_expert_joints',{}))
        if not states:
            telemetry=os.environ.get('YAM_TELEMETRY')
            if telemetry and Path(telemetry).exists():
                with Path(telemetry).open('rb') as stream:
                    stream.seek(0,2);stream.seek(max(0,stream.tell()-65536))
                    lines=stream.read().splitlines()
                for line in reversed(lines):
                    try:row=json.loads(line)
                    except (ValueError,UnicodeDecodeError):continue
                    states={name:SimpleNamespace(pos=np.asarray(value['pos']),quat_xyzw=np.asarray(value['quat'])) for name,value in row['objects'].items()}
                    joints=row['joints'];break
        if states and not hasattr(self,'_initial_poses'):
            self._initial_poses={name:np.asarray(value.pos).copy() for name,value in states.items()}
        if t>1. and not states:raise RuntimeError('Scripted feedback has no measured state receipt')
        slug=self._task.path.parent.name
        values=[]
        for arm in ['left','right']:
            plan=self._plans[arm]
            target,rot,finger,label=plan.target(t)
            obj=None;goal=None;active=False
            if slug=='split_shaft_collar' and 3.8<=t<=12.3:
                sign=1 if arm=='left' else -1
                obj=states.get(arm+'_half')
                goal=target+np.array([0,-sign*.0375,-D]);active=True
            elif arm=='right' and 6.1<=t<=13.6 and slug=='retaining_fork':
                obj=states.get('fork');goal=target+np.array([0,.0675,-.0025-D]);active=True
            elif arm=='right' and 6.1<=t<=13.8 and slug=='keyed_drive_coupling':
                obj=states.get('coupling');goal=target+np.array([0,0,-.012-D]);active=True
            elif arm=='right' and 6.1<=t<=14.6 and slug=='captive_channel_nut':
                obj=states.get('nut');goal=target+np.array([0,0,-.0055-D]);active=True
            elif arm=='left' and 3.0<=t<17.8 and slug=='retaining_fork':
                obj=states.get('fixture');goal=getattr(module,'_reference',getattr(self,'_initial_poses',{}).get('fixture'));active=True
            elif arm=='left' and 3.0<=t<18.8 and slug=='captive_channel_nut':
                obj=states.get('channel');goal=getattr(module,'_reference',getattr(self,'_initial_poses',{}).get('fixture'));active=True
            if active and obj is not None and goal is not None and arm+'_robot' in joints:
                data=joints[arm+'_robot'];by=dict(zip(data['names'],data['q']))
                measured=np.array([by['joint'+str(i)] for i in range(1,7)])
                ee,ee_rot=ik.tcp_pose(measured,self._bases[arm])
                object_rot=quat_xyzw_to_matrix(np.asarray(obj.quat_xyzw))
                # Rigid grasp servo from actual geometry. Bounded corrections avoid forceful jumps.
                desired=np.asarray(goal)+object_rot.T@(ee-np.asarray(obj.pos))
                delta=desired-target;length=np.linalg.norm(delta)
                if length>.025:delta*=.025/length
                target=target+delta
                corrected=object_rot.T@ee_rot
                rot=ik.slerp_matrix(rot,corrected,.7)
            q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rot),plan.q,base_pos=self._bases[arm],iters=80)
            plan.q=q
            grip=float(np.clip(finger/FINGER_OPEN,0.,1.))
            values.append(np.r_[q,grip])
        meta={'request_stop':True} if t>self._end+self.hold_s else {}
        return ActionChunk(actions=[Action(data=pack(*values),meta=meta)],control_hz=1/self._dt)

def policy(**kwargs):return FeedbackExpert(**kwargs)
