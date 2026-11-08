"""IK scripted demonstration using robot joint/gripper targets and real contacts."""
import json,time
from pathlib import Path
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix

FRONT=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)


def spoon_rotation(alpha,azimuth):
    a,b=np.radians([alpha,azimuth]);D=np.array([np.cos(a)*np.cos(b),np.cos(a)*np.sin(b),np.sin(a)])
    W=np.array([-np.sin(b),np.cos(b),0.]);return np.column_stack([D,W,np.cross(D,W)])


class Demo:
    def __init__(self,env,reader,*,recorder=None,output=None,parameters=None):
        self.env,self.reader,self.recorder=env,reader,recorder
        self.output=Path(output) if output else None;self.parameters=parameters or {}
        self.command={s:np.r_[READY,1.] for s in ['left','right']}
        self.steps=0;self.rows=[];self.phases=[];self.started=time.monotonic()
        self.worst_position_error=self.worst_rotation_error=0.

    def tick(self,label):
        import torch
        if self.steps>=2400:raise RuntimeError('Scripted trial exceeded the 2400-step run limit')
        action=np.r_[self.command['left'],self.command['right']]
        self.env.step(torch.as_tensor(action[None],device=self.reader.base.device,dtype=torch.float32))
        self.steps+=1;score=self.reader.score()
        if self.recorder:self.recorder.capture()
        if self.steps%5==0:
            row=dict(step=self.steps,phase=label,action=action.tolist(),bottle_pose=self.reader.body()['pose'].tolist(),
                tcp={s:self.reader.arm(s)[2].tolist() for s in ['left','right']},**score)
            self.rows.append(row)
            if self.output and self.steps%20==0:(self.output/'progress.json').write_text(json.dumps(row,indent=2)+'\n')
        return score

    def solve_tick(self,side,p,R,grip,label,q):
        q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),q,base_pos=station.BASES[side],iters=120)
        self.worst_position_error=max(self.worst_position_error,pe)
        self.worst_rotation_error=max(self.worst_rotation_error,float(np.degrees(re)))
        if pe>.012 or re>np.radians(12):raise RuntimeError(f'IK failure {label}: {pe*1000:.1f} mm, {np.degrees(re):.1f} deg')
        self.command[side]=np.r_[q,grip];self.tick(label);return q

    def move(self,side,goal,R,seconds,grip,label,verify=True):
        q,g,start,r0=self.reader.arm(side);goal=np.array(goal,float)
        self.phases.append(dict(label=label,start_step=self.steps,arm=side,target=goal.tolist()))
        print('DEMO_PHASE',label,'target',goal.tolist(),flush=True)
        for k in range(1,round(seconds*20)+1):
            t=ik.smoothstep(k/round(seconds*20))
            q=self.solve_tick(side,start+t*(goal-start),ik.slerp_matrix(r0,R,t),g+t*(grip-g),label,q)
        if verify:
            error=np.linalg.norm(self.reader.arm(side)[2]-goal)
            if error>.020:raise RuntimeError(f'Robot blocked during {label}: TCP tracking error {error*1000:.1f} mm')

    def hold(self,seconds,label):
        for _ in range(round(seconds*20)):result=self.tick(label)
        return result

    def ready(self,side,seconds=2.):
        q,g,_,_=self.reader.arm(side)
        for k in range(1,round(seconds*20)+1):
            t=ik.smoothstep(k/round(seconds*20));self.command[side]=np.r_[q+t*(np.array(READY)-q),g+t*(1-g)]
            self.tick(side+'_return_ready')

    def carry(self,base_goal,seconds,label):
        body=self.reader.body();start=body['pose'][:3].copy();r0=body['rotation'].copy()
        q=self.reader.arm('right')[0];self.phases.append(dict(label=label,start_step=self.steps,bottle_base_target=list(base_goal)))
        print('DEMO_PHASE',label,'base',list(base_goal),flush=True)
        for k in range(1,round(seconds*20)+1):
            t=ik.smoothstep(k/round(seconds*20));desired=start+t*(np.array(base_goal)-start)
            target_r=ik.slerp_matrix(r0,np.eye(3),t)
            body=self.reader.body();_,_,tcp,er=self.reader.arm('right')
            if np.linalg.norm(tcp-body['grasp'])>.045:raise RuntimeError('Bottle did not remain in the physical gripper')
            correction=target_r@body['rotation'].T
            target=desired+correction@(tcp-body['pose'][:3]);rotation=correction@er
            q=self.solve_tick('right',target,rotation,0.,label,q)

    def warm_bottle(self):
        yaw,pitch=np.radians(self.parameters.get('grip_angles_deg',[35,35]))
        x=np.array([np.sin(yaw),-np.cos(yaw),0.]);z=np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),-np.sin(pitch)])
        R=np.column_stack([x,np.cross(z,x),z]);g=self.reader.body()['grasp']
        self.move('right',g+[-.080,-.030,.140],R,2.5,1.,'bottle_hover')
        self.move('right',g-R[:,2]*.060,R,2.,.95,'bottle_pregrasp')
        g=self.reader.body()['grasp']
        self.move('right',g,R,2.,.90,'bottle_enter')
        self.move('right',g,R,1.3,0.,'bottle_grasp')
        self.hold(.6,'bottle_grip_settle')
        bath=np.array(self.reader.specs['water_bath'].position)
        rim=bath[2]+.120;clearance=float(self.parameters.get('rim_clearance_m',.025));floor=float(self.reader.layout['water_bath']['inner_floor_world_z'])
        p=self.reader.body()['pose'][:3].copy();p[2]=rim+clearance
        self.carry(p,3.,'bottle_lift')
        target=bath.copy();target[:2]+=np.array(self.parameters.get('bath_offset_xy',[.015,.015]));target[2]=rim+clearance
        self.carry(target,4.,'bottle_over_bath')
        held=target.copy();held[2]=floor+.020
        self.carry(held,3.5,'bottle_lower_into_bath')
        self.hold(.7,'bath_held_validation')
        if not self.reader.controls()['bottle_in_bath']:raise RuntimeError('Bottle is not upright in the warm-water bath')
        if self.parameters.get('mode')=='bath_held':return self.hold(1.,'held_final')
        target[2]=floor+.001
        self.carry(target,1.5,'bottle_seat_on_floor')
        _,_,tcp,er=self.reader.arm('right')
        self.move('right',tcp,er,1.2,1.,'bottle_release',verify=False)
        self.move('right',tcp-er[:,2]*.120+[0,0,.050],er,2.5,1.,'bottle_retreat')
        self.ready('right',2.);return self.hold(1.5,'bath_final_hold')

    def close_door(self):
        objects={r['name']:r for r in self.reader.layout['objects']}
        hinge=np.array(objects['Door']['position']);handle=np.array(self.reader.layout['microwave']['door_handle_center_local'])
        angle=self.reader.controls()['door_angle_deg'];D=axis_angle_matrix(np.array([0.,0,1]),np.radians(-angle));R=D@FRONT
        target=hinge+D@handle
        self.move('left',[.12,.13,1.12],R,2.5,.7,'door_hover')
        self.move('left',target-R[:,2]*.065,R,2.,.6,'door_pregrasp')
        self.move('left',target,R,2.,.55,'door_enter')
        self.move('left',target,R,1.,0.,'door_grasp')
        q=self.reader.arm('left')[0];initial=self.reader.controls()['door_angle_deg']
        print('DEMO_PHASE door_swing',initial,flush=True)
        for k in range(1,round(6*20)+1):
            t=ik.smoothstep(k/round(6*20));a=initial*(1-t)
            d=axis_angle_matrix(np.array([0.,0,1]),np.radians(-a))
            q=self.solve_tick('left',hinge+d@handle,d@FRONT,0.,'door_swing',q)
        self.hold(.7,'door_latch')
        _,_,tcp,er=self.reader.arm('left')
        self.move('left',tcp,er,.8,1.,'door_release',verify=False)
        self.move('left',tcp+[-.075,0,.015],er,2.,1.,'door_retreat')
        self.ready('left');self.hold(.6,'door_closed_hold')
        if abs(self.reader.controls()['door_angle_deg'])>3:raise RuntimeError('Door did not physically close')

    def press_start(self):
        pos=np.array(self.reader.specs['start_button'].position);touch=pos+[-.023,0,0]
        self.move('left',[.39,.12,1.02],FRONT,2.5,0.,'start_hover')
        self.move('left',touch+[-.045,0,0],FRONT,1.5,0.,'start_align')
        self.move('left',touch,FRONT,1.5,0.,'start_touch')
        self.move('left',touch+[.011,0,0],FRONT,1.5,0.,'start_press',verify=False)
        self.hold(.6,'start_hold')
        self.move('left',touch+[-.08,0,.035],FRONT,2.,0.,'start_release')
        return self.hold(1.2,'final_hold')

    def run(self):
        if self.parameters.get('mode','demo')=='unsafe_start':
            self.close_door();return self.press_start()
        return self.warm_bottle()

    def report(self):
        return dict(control_steps=self.steps,simulated_seconds=self.steps/20,wall_seconds=time.monotonic()-self.started,
            parameters=self.parameters,trajectory=self.rows,phases=self.phases,worst_ik_position_error_m=self.worst_position_error,
            worst_ik_rotation_error_deg=self.worst_rotation_error,
            object_motion='Robot contacts and friction only; no bottle attachments or object pose writes during the trial')
