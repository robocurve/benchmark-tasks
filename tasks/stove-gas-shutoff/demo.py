"""Robot-only joint/gripper commands. Controls move through physical contact."""
import json,time
from pathlib import Path
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix,axis_angle_matrix

FRONT=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)


class Demo:
    def __init__(self,env,reader,*,recorder=None,output=None,parameters=None):
        self.env,self.reader,self.recorder=env,reader,recorder
        self.output=Path(output) if output else None
        self.parameters=parameters or {}
        self.command={s:np.r_[READY,1.] for s in ('left','right')}
        self.steps=0;self.rows=[];self.phases=[];self.started=time.monotonic()
        self.worst_position_error=self.worst_rotation_error=0.

    def tick(self,label):
        import torch
        action=np.r_[self.command['left'],self.command['right']]
        self.env.step(torch.as_tensor(action[None],device=self.reader.base.device,dtype=torch.float32))
        self.steps+=1
        result=self.reader.score()
        if self.recorder:self.recorder.capture()
        if self.steps%5==0:
            row=dict(step=self.steps,phase=label,action=action.tolist(),
                     tcp={s:self.reader.arm(s)[2].tolist() for s in ('left','right')},**result)
            self.rows.append(row)
            if self.output and self.steps%20==0:(self.output/'progress.json').write_text(json.dumps(row,indent=2)+'\n')
        return result

    def move(self,side,goal,rotation,seconds,grip,label):
        q,g,start,r0=self.reader.arm(side)
        goal=np.asarray(goal,float)
        self.phases.append(dict(label=label,start_step=self.steps,target=goal.tolist(),arm=side))
        print('DEMO_PHASE',label,goal.tolist(),flush=True)
        count=round(seconds*20)
        for k in range(1,count+1):
            t=ik.smoothstep(k/count)
            r=ik.slerp_matrix(r0,rotation,t)
            q,pe,re=ik.solve(start+t*(goal-start),ik.matrix_to_quat_xyzw(r),q,base_pos=station.BASES[side],iters=100)
            self.worst_position_error=max(self.worst_position_error,pe)
            self.worst_rotation_error=max(self.worst_rotation_error,float(np.degrees(re)))
            if pe>.009 or re>np.radians(10):raise RuntimeError(f'Unreachable {label}: {pe*1000:.1f} mm / {np.degrees(re):.1f} deg')
            self.command[side]=np.r_[q,g+t*(grip-g)]
            self.tick(label)

    def hold(self,seconds,label):
        for _ in range(round(seconds*20)):result=self.tick(label)
        return result

    def run(self):
        side=self.parameters.get('side','right')
        if self.parameters.get('mode')=='knob':
            spec=self.reader.specs[side+'_burner_knob']
            root=quat_xyzw_to_matrix(np.array(spec.rotation_xyzw))
            center=np.array(spec.position)+root@np.array([-.034,0,0])
            r=root@FRONT
            self.move(side,[.26,spec.position[1],1.08],r,2.5,1.,'knob_hover')
            self.move(side,center-root[:,0]*.075,r,1.5,.7,'knob_approach')
            self.move(side,center,r,2.,.7,'knob_enter')
            self.move(side,center,r,1.,0.,'knob_grasp')
            twist=root@axis_angle_matrix(np.array([1.,0,0]),np.radians(45))@FRONT
            self.move(side,center,twist,3.,0.,'knob_turn')
            return self.hold(1.,'knob_hold')
        spec=self.reader.specs['gas_shutoff_button']
        touch=np.array(spec.position)+np.array([-.0435,0,0])
        self.move(side,[.30,-.25,1.12],FRONT,2.5,0.,'shutoff_hover')
        self.move(side,touch-np.array([.075,0,0]),FRONT,2.,0.,'shutoff_align')
        self.move(side,touch,FRONT,2.,0.,'shutoff_touch')
        self.move(side,touch+np.array([.016,0,0]),FRONT,2.,0.,'shutoff_press')
        self.hold(.6,'shutoff_latch')
        self.move(side,touch-np.array([.08,0,0]),FRONT,2.,0.,'shutoff_release')
        return self.hold(1.2,'safe_final_hold')

    def report(self):
        return dict(control_steps=self.steps,simulated_seconds=self.steps/20,
                    wall_seconds=time.monotonic()-self.started,trajectory=self.rows,phases=self.phases,
                    worst_ik_position_error_m=self.worst_position_error,
                    worst_ik_rotation_error_deg=self.worst_rotation_error,
                    object_motion='Robot contacts only; no control pose writes or attachments during the trial')
