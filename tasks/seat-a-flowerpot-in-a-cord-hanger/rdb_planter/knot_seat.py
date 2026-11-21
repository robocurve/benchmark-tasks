"""Hold the bottom knot, feed the tilted pot through the four-cord web, and hand over its weight."""
import json
from pathlib import Path
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation
from inspect_robots import Action, ActionChunk
from inspect_robots.policy import PolicyBase
from yam_arena import ik
from yam_arena.scripted import YamScriptedPolicy

from .fixed_scene import CONTROL_HZ, POT, TOP

BASES=np.array([[0.,.305,.75],[0.,-.305,.75]])
PADS=np.array([[0.,-.004,.008],[0.,-.008,-.010]])
STAGES=(('approach knot and pot',60),('straddle both objects',40),('close on the rim and knot',44),
        ('lift the pot and support the knot',164),('carry to the web opening',105),
        ('angle the leading edge',68),('pass the leading edge through',65),('bring the base over the knot',70),
        ('right the pot inside the opening',82),('lower to the bottom web',68),('hold the rim steady',28),
        ('release the knot',30),('withdraw the left wrist',64),('lower the pot onto the free knot',70),
        ('release the rim',32),('withdraw the right wrist',62),('watch the loaded web',125))

def fraction(t):
    t=float(np.clip(t,0.,1.))
    return t*t*(3.-2.*t)

def mix(a,b,t):
    return a+np.clip(t,0.,1.)*(b-a)

def attitude(r,goal,t):
    return Rotation.from_rotvec(t*Rotation.from_matrix(goal@r.T).as_rotvec()).as_matrix()@r

def rim(az,tilt):
    dir=np.array([np.cos(az),np.sin(az),0.])
    forward=-np.sin(tilt)*dir-[0.,0.,np.cos(tilt)]
    y=np.cross(forward,dir);y/=np.linalg.norm(y)
    return dir,np.column_stack((np.cross(y,forward),y,forward))

class KnotSeat(PolicyBase):
    def __init__(self,rig):
        self.rig=rig
        self.info=replace(YamScriptedPolicy().info,name='lower_edge_into_cord_hanger')
        cfg=json.loads(Path(__file__).with_name('knot-grip.json').read_text())
        self.left_r=np.array(cfg['rot']);self.left_off=np.array(cfg['knot_in_pad'])
        self.arm_seed=np.array(cfg['seed'])
    def reset(self,scene):
        self.phase=0;self.tick=0;self.goal=None;self.trim=np.zeros((2,3));self.log=[]
        self.rad,self.wrist=rim(np.deg2rad(-90.),np.arctan2(.52,.854))
        self.direction=np.array([np.cos(np.pi/4),-np.sin(np.pi/4),0.])
    def bind(self,embodiment_info):pass
    def act(self,observation):
        q=np.asarray(observation.state['joint_pos']);knot=self.rig.poses['bottom_knot'][0]
        p,actual_r=self.rig.poses['pot']
        if self.goal is None:
            self.goal=q.copy();self.initial=q.copy();self.initial_knot=knot.copy()
        poses=[ik.tcp_pose(q[7*i:7*i+6],base) for i,base in enumerate(BASES)]
        pads=np.array([a+b@o for (a,b),o in zip(poses,PADS)])
        rots=[v[1] for v in poses]
        phase=self.phase;name,n=STAGES[phase]
        if self.tick==0:
            self.start=pads.copy();self.start_rot=[v.copy() for v in rots];self.start_q=q.copy()
            self.enter_p=p.copy();self.enter_r=actual_r.copy();self.enter_k=knot.copy()
            if phase==1:self.start_knot=knot.copy()
            if phase==4:
                self.grip_pos=actual_r.T@(pads[1]-p)
                self.grip_rot=actual_r.T@rots[1]
            if phase==3:
                self.left_hold=pads[0]+[.010,0,.018];self.knot0=knot.copy()
            self.log.append(dict(tick=self.rig.tick,stage=name,knot=knot.tolist(),pot=p.tolist()))
        self.tick+=1;t=fraction(self.tick/n)
        desired=pads.copy();orient=[self.left_r,self.wrist];jaws=[0.,.005]
        neutral=knot if phase==0 else self.start_knot
        left=neutral-self.left_r@self.left_off
        lip=p+.058*self.rad+[0,0,.086]
        if phase==0:
            desired[0]=left+[0,.034,.019];desired[1]=lip+[0,0,.055];jaws=[.30,.48]
        elif phase==1:
            desired[0]=mix(self.start[0],left,t);desired[1]=mix(self.start[1],lip,t);jaws=[.30,.42]
        elif phase==2:
            desired=self.start;orient=self.start_rot;jaws=mix(self.start_q[[6,13]],np.array([0.,.005]),t)
        else:
            desired[0]=mix(self.start[0],self.left_hold,t) if phase==3 else self.left_hold
            if phase==11:jaws[0]=.45*t
            elif phase>=12:
                desired[0]=mix(self.start[0],self.left_hold+[0,.092,.047],t) if phase==12 else self.left_hold+[0,.092,.047]
                jaws[0]=.55
            if phase==3:
                desired[1]=self.start[1]
            elif phase in (14,15,16):
                if phase==14:self.release_pad=self.start[1].copy();self.release_r=self.start_rot[1].copy()
                dest=self.release_pad+[0,-.045,.105]
                desired[1]=self.release_pad if phase==14 else mix(self.release_pad,dest,t if phase==15 else 1.)
                orient[1]=self.release_r;jaws[1]=.55*t if phase==14 else .55
            else:
                route={4:(-.109,.098,0.),5:(-.110,.115,18.),6:(-.055,.096,32.),
                       7:(-.019,.079,32.),8:(0.,.043,0.),9:(0.,.019,0.),
                       10:(0.,.019,0.),11:(0.,.019,0.),12:(0.,.019,0.),13:(0.,.007,0.)}
                if phase==3:
                    goal=self.enter_p+[0,0,.145]
                    goal_r=self.enter_r
                else:
                    side,height,rake=route[phase]
                    center=(knot if phase<=11 else self.knot0+[.010,0,.018]).copy()
                    # Only track a small displacement after removing the left fingers;
                    # a displaced free knot must not pull the held pot out of its web.
                    if phase>=12:center+=np.clip(knot-center,-.010,.010)
                    goal=center+side*self.direction+[0,0,height]
                    axis=np.cross([0,0,1.],self.direction)
                    goal_r=Rotation.from_rotvec(np.deg2rad(rake)*axis).as_matrix()@Rotation.from_euler('z',-25,degrees=True).as_matrix()
                actual_goal=mix(self.enter_p,goal,t)
                desired_r=attitude(self.enter_r,goal_r,t)
                self.trim[1]=np.clip(self.trim[1]+.055*np.clip(actual_goal-p,-.002,.002),-.013,.013)
                desired[1]=actual_goal+self.trim[1]+desired_r@self.grip_pos
                orient[1]=desired_r@self.grip_rot
        for i in (0,1):
            if i==1 and phase<=3:
                continue
            j=7*i
            if phase==0:
                seed=self.arm_seed if i==0 else self.start_q[j:j+6]
            else:seed=q[j:j+6]
            if i==0 and 2<phase<11:
                self.trim[0]=np.clip(self.trim[0]+.065*np.clip(desired[0]-pads[0],-.002,.002),-.008,.008)
            aim=desired[i]+(self.trim[0] if i==0 else 0.)
            target,e,a=ik.solve(aim-orient[i]@PADS[i],ik.matrix_to_quat_xyzw(orient[i]),seed,BASES[i],iters=70,rot_weight=.19)
            if phase==0:target=mix(self.initial[j:j+6],target,t)
            self.goal[j:j+6]+=np.clip(target-self.goal[j:j+6],-.085,.085)
            self.goal[j+6]+=np.clip(jaws[i]-self.goal[j+6],-.025,.025)
        if phase<=3:
            self.rim_tick=getattr(self,'rim_tick',0)+1
            rsteps=(60,40,44,36,128)
            where=int(np.searchsorted(np.cumsum(rsteps),self.rim_tick,side='left'))
            first_tick=sum(rsteps[:where])+1
            if self.rim_tick==first_tick:
                self.rim_start=getattr(self,'previous_rim',self.initial[7:14]).copy()
                dz=(.044,0.,0.,.031,.130)[where]
                jaw=(.48,.42,.005,.005,.005)[where]
                root=p if where<3 else POT
                anchor=root+[0.,-.058,.086+dz]
                if where==4:anchor=np.array([.30,-.200,TOP+.217])
                aim=anchor-self.wrist@PADS[1]
                jt,e,a=ik.solve(aim,ik.matrix_to_quat_xyzw(self.wrist),q[7:13],BASES[1],
                                iters=85,damping=.025,rot_weight=.14)
                self.rim_end=np.r_[jt,jaw]
            target=mix(self.rim_start,self.rim_end,fraction((self.rim_tick-first_tick+1)/rsteps[where]))
            prev=getattr(self,'previous_rim',self.initial[7:14])
            self.goal[7:14]=prev+np.clip(target-prev,-.12,.12)
            self.previous_rim=self.goal[7:14].copy()
        if self.tick==n:
            self.log[-1].update(knot_end=knot.tolist(),pot_end=p.tolist(),crossings=self.rig.final_grade.get('cord_crossings'))
            if phase<len(STAGES)-1:self.phase+=1;self.tick=0
        return ActionChunk(actions=[Action(data=self.goal.copy(),meta={})],control_hz=CONTROL_HZ)
