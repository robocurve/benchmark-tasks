"""Privileged pose-feedback demo. Only robot joint targets are commanded."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack, split
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import quat_xyzw_to_matrix

class PlacementPlan(ArmPlan):
    def object_goal(self,pos,rot,index,phase):
        desired=rot@grip_rotation(self.task.grasp_yaw(int(index)-1)).T
        center=pos-desired@np.r_[self.task.grasp_offset(int(index)-1),self.task.GRASP_Z_OFFSET]
        if int(index)>1 and phase!='lift':
            anchor=self.task.demo_states.get('square_api_piece_1')
            if anchor is not None:
                goal,anchor_rotation=self.task.assembly_pose(int(index)-1,anchor)
                if int(index)==4:
                    if not hasattr(self,'corner_fit'):
                        self.corner_fit=self.task.fit_final_corner(self.task.demo_states,goal,anchor_rotation)
                    goal=np.asarray(self.corner_fit['xy'])
                    yaw=self.corner_fit['yaw'];c,s=np.cos(yaw),np.sin(yaw)
                    anchor_rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
                center[:2]+=goal-np.asarray(self.task.GOAL[int(index)-1])
                desired=anchor_rotation@desired
        return center,desired

    def may_advance(self):
        pos,rot,_,label=ArmPlan.target(self,self.clock)
        if ':' not in label or label.split(':',1)[1]!='lower':return True
        index,phase=label.split(':',1)
        obj=self.task.demo_states.get(f'square_api_piece_{index}')
        if obj is None:return False
        center,desired=self.object_goal(pos,rot,index,phase)
        actual=quat_xyzw_to_matrix(obj.quat_xyzw)
        angle=np.arccos(np.clip((np.trace(desired@actual.T)-1)/2,-1.,1.))
        xy_tolerance=.0001 if int(index)==4 else .00025
        angle_tolerance=.1 if int(index)==4 else .35
        return bool(np.linalg.norm(center[:2]-obj.pos[:2])<xy_tolerance and
                    abs(center[2]-obj.pos[2])<.001 and angle<np.deg2rad(angle_tolerance))

    def target(self,t):
        pos,rot,finger,label=super().target(t)
        if ':' not in label:return pos,rot,finger,label
        index,phase=label.split(':',1)
        obj=self.task.demo_states.get(f'square_api_piece_{index}')
        if obj is None:return pos,rot,finger,label
        tcp,tcp_rot=self.actual_pose
        if phase in ('lift','carry','align','lower','seat'):
            if getattr(self,'piece',None)!=index:
                self.piece=index;self.bias=np.zeros(3);self.rotation_bias=np.eye(3)
            obj_rot=quat_xyzw_to_matrix(obj.quat_xyzw)
            center,desired=self.object_goal(pos,rot,index,phase)
            correction=desired@obj_rot.T
            if phase in ('align','seat','lower'):
                gain=.04 if phase=='lower' else .06
                self.bias=np.clip(self.bias+gain*(center-obj.pos),-.012,.012)
                self.bias[2]=np.clip(self.bias[2],-.004,.004)
                self.rotation_bias=ik.slerp_matrix(np.eye(3),correction,.04)@self.rotation_bias
                q=ik.matrix_to_quat_xyzw(self.rotation_bias)
                angle=2*np.arccos(np.clip(abs(q[3]),0.,1.))
                if angle>np.deg2rad(6):
                    self.rotation_bias=ik.slerp_matrix(np.eye(3),self.rotation_bias,np.deg2rad(6)/angle)
            rot=self.rotation_bias@correction@tcp_rot
            pos=center+correction@(tcp-obj.pos)+self.bias
            self.last_command=(pos.copy(),rot.copy())
            if phase in ('lower','seat') and obj_rot[2,2]<np.cos(np.deg2rad(15)):
                self.failed=True
        elif phase in ('settle','release') and hasattr(self,'last_command'):
            pos,rot=self.last_command
        elif phase=='retreat' and hasattr(self,'last_command'):
            previous,rot=self.last_command
            pos=np.array([previous[0],previous[1],pos[2]])
        return pos,rot,finger,label

class SquarePolicy(YamScriptedPolicy):
    def __init__(self,trace_path=None,**kwargs):
        super().__init__(**kwargs)
        self.info=replace(self.info,name='square_scripted')
        self.trace_path=Path(trace_path) if trace_path else None
    def reset(self,scene):
        plans=self.plan_for(scene.instruction,scene.target)
        self._plans={side:PlacementPlan(list(waypoints),self._bases[side]) for side,waypoints in plans.items()}
        self.trace_step=0
        for side,p in self._plans.items():
            p.task=self._task.module;p.clock=0.;p.blocked=0;p.label='start'
            p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases[side])
    def act(self,observation):
        self.trace_step+=1
        for side,q in zip(('left','right'),split(observation.state['joint_pos'])):
            if side in self._plans:self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
        actions=[]
        for side in ('left','right'):
            p=self._plans.get(side)
            if p is None:
                actions.append(np.r_[READY,1.]);continue
            advance=p.may_advance()
            if side=='right' and 'left' in self._plans:
                wait=p.waypoints[0]
                if wait.label=='wait clear of left arm' and p.clock+self._dt>=wait.duration:
                    advance=advance and self._plans['left'].clock>=self._plans['left'].duration
            if advance:p.clock+=self._dt;p.blocked=0
            else:p.blocked+=1
            if p.blocked>200 and ':' in p.label:p.failed=True
            action,p.label=p.action7(p.clock);actions.append(action)
        finished=all(p.clock>p.duration+self.hold_s for p in self._plans.values())
        failed=any(getattr(p,'failed',False) for p in self._plans.values())
        if self.trace_path and self.trace_step%10==0:
            row={'step':self.trace_step,'objects':{k:{'pos':v.pos.tolist(),'quat':v.quat_xyzw.tolist()} for k,v in self._task.module.demo_states.items() if k.startswith('square_api_piece_')}}
            row['arms']={side:{'plan_time':p.clock,'label':p.label,'blocked_steps':p.blocked,'corner_fit':getattr(p,'corner_fit',None),'tcp':p.actual_pose[0].tolist(),'tcp_rotation':p.actual_pose[1].tolist(),'command_pos':p.last_command[0].tolist() if hasattr(p,'last_command') else None,'command_rotation':p.last_command[1].tolist() if hasattr(p,'last_command') else None} for side,p in self._plans.items()}
            with self.trace_path.open('a') as f:f.write(json.dumps(row)+'\n')
        meta={'request_stop':True} if finished or failed else {}
        return ActionChunk(actions=[Action(data=pack(*actions),meta=meta)],control_hz=1/self._dt)

def make_policy(**kwargs):return SquarePolicy(**kwargs)
