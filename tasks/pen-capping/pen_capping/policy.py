"""Privileged pose feedback commands robot joints; assembly motion stays physical."""
from dataclasses import dataclass
import math
import numpy as np
from inspect_robots_yam.packing import split
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,READY
from yam_arena.demo import ArmPlan,Waypoint,grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import axis_angle_matrix
from pen_capping.scoring import SEATED_X
ASSEMBLY_AXIS=(-math.sqrt(.5),-math.sqrt(.5),0.)

@dataclass
class ToolWaypoint(Waypoint):
    object_name: str | None = None
    axis: tuple = ASSEMBLY_AXIS
    follow_pen: int | None = None
    depth: float | None = None
    release: bool = False
    pick_name: str | None = None
    pick_x: float = 0.
    pick_height: float = 0.
    pick_yaw_offset: float = 0.

def make_plan(top,pens,caps,colors):
    plans={'left':[],'right':[]};counts={'red':0,'blue':0}
    park={'left':(.13,.37,top+.20),'right':(.13,-.40,top+.20)}
    def tcp(d,p,f,l,yaw=0):return ToolWaypoint(d,tuple(p),f,l,yaw)
    def root(d,name,p,f,l,axis=ASSEMBLY_AXIS,follow=None,depth=None,release=False):return ToolWaypoint(d,tuple(p),f,l,0.,name,axis,follow,depth,release)
    for i,(pen,cap,color) in enumerate(zip(pens,caps,colors)):
        pn,cn=f'pen_{i}',f'cap_{i}';x,y,z,a=pen;cx,cy,cz,ca=cap
        pg=np.array([x+.055*math.cos(a),y+.055*math.sin(a),z+.010]);cg=np.array([cx,cy,cz+.010])
        right=plans['right'];left=plans['left']
        for points,g,yaw in ((right,pg,a+math.pi),(left,cg,ca+math.pi)):
            points += [tcp(2,g+[0,0,.10],O,'approach',yaw),tcp(2,g,O,'lower',yaw),tcp(1,g,C,'grasp',yaw),tcp(2,g+[0,0,.10],C,'lift',yaw)]
            for w,height in zip(points[-4:],(.110,.010,.010,.110)):
                w.pick_name=pn if points is right else cn;w.pick_x=.055 if points is right else 0.;w.pick_height=height;w.pick_yaw_offset=math.pi
        body=(.27,-.02,top+.10);cap_above=tuple(np.array(body)-.16*np.array(ASSEMBLY_AXIS))
        right += [root(4,pn,body,C,'carry body'),root(2,pn,body,C,'align body'),root(6,pn,body,C,'hold insertion'),root(3,pn,body,C,'hold seat')]
        left += [root(4,cn,cap_above,C,'carry cap'),root(2,cn,cap_above,C,'align cap',follow=i,depth=-.16),root(6,cn,cap_above,C,'insert',follow=i,depth=SEATED_X),root(3,cn,cap_above,C,'seat',follow=i,depth=SEATED_X)]
        placer='left' if color=='red' else 'right';other='right' if placer=='left' else 'left';held=cn if placer=='left' else pn;other_name=pn if placer=='left' else cn
        plans[other] += [root(1,other_name,body,O,'release supporting hand',release=True),tcp(2,park[other],O,'retreat supporting hand')]
        plans[placer] += [root(1,held,body,C,'hold assembled'),root(2,held,body,C,'hold assembled')]
        # The left arm holds the cap for red pens; the right holds the blue body.
        # Reuse the last reached pose in the handover hold, rather than moving it.
        for w in plans[placer][-2:]:w.label='hold reached'
        index=counts[color];counts[color]+=1;dest=np.array([.30,(.19+.08*index)*(1 if color=='red' else -1),top+.10]);axis=np.array([-1.,0.,0.])
        target=dest+(axis*SEATED_X if placer=='left' else 0)
        low=target.copy();low[2]=top+.017
        plans[placer] += [root(5,held,target,C,'carry to destination',tuple(axis)),root(3,held,low,C,'lower assembly',tuple(axis)),root(2,held,low,O,'release assembly',tuple(axis),release=True),root(1,held,low,O,'wait fully open',tuple(axis),release=True)]
        grip_offset=0. if placer=='left' else .055;hover=low+axis*grip_offset+[0,0,.13]
        if placer=='left':
            exit_point=low+axis*.11+[0,0,.010]
            plans[placer].append(tcp(2,exit_point,O,'withdraw through open mouth'))
            hover=exit_point+[0,0,.12]
        plans[placer] += [tcp(2,hover,O,'retreat vertically'),tcp(2,park[placer],O,'park'),tcp(2,park[placer],O,'rest')]
        plans[other] += [tcp(19 if placer=='left' else 17,park[other],O,'wait for placement')]
    return plans

class FeedbackPlan(ArmPlan):
    def __init__(self,points,base,side):
        super().__init__(points,base);self.side=side;self.actual_pose=ik.tcp_pose(np.asarray(READY),base);self.active=-1;self.follow_index=None;self.follow_bias=np.zeros(3)
    def target(self,t):
        from pen_capping.embodiment import FEEDBACK
        elapsed=t;index=0
        for index,w in enumerate(self.waypoints):
            if elapsed<w.duration:break
            elapsed-=w.duration
        else:return self.last
        tcp,trot=self.actual_pose;poses=FEEDBACK['poses'];rots=FEEDBACK['rotations'];s=ik.smoothstep(elapsed/w.duration)
        if index!=self.active:
            print('PEN_PHASE',self.side,round(t,2),w.label,'object',w.object_name,flush=True)
            self.active=index;self.start_tcp=tcp.copy();self.start_rot=trot.copy();self.start_finger=self.finger_start if index==0 else self.waypoints[index-1].finger
            if w.pick_name:
                axis=rots[w.pick_name][:,0];self.pick_goal=poses[w.pick_name][:3]+axis*w.pick_x+np.array([0.,0.,w.pick_height]);yaw=math.atan2(axis[1],axis[0])+w.pick_yaw_offset;self.pick_rotation=grip_rotation(yaw)
                print('MEASURED_PICK',self.side,w.label,w.pick_name,'root',poses[w.pick_name][:3].tolist(),'target',self.pick_goal.tolist(),'yaw',math.degrees(yaw),flush=True)
            if w.object_name:
                self.start_root=poses[w.object_name][:3].copy()
                if w.follow_pen is not None:
                    if self.follow_index!=w.follow_pen:self.follow_index=w.follow_pen;self.follow_bias=np.zeros(3)
                    pn=f'pen_{w.follow_pen}';self.start_depth=float((rots[pn].T@(poses[w.object_name][:3]-poses[pn][:3]))[0])
        finger=self.start_finger+s*(w.finger-self.start_finger)
        if w.release or w.label=='hold reached':pos=self.start_tcp;rot=self.start_rot
        elif w.object_name:
            goal_axis=np.array(w.axis);desired=self.start_root+s*(np.array(w.pos)-self.start_root)
            if w.follow_pen is not None:
                pn=f'pen_{w.follow_pen}';depth=self.start_depth+s*(w.depth-self.start_depth);desired=poses[pn][:3]+rots[pn]@np.array([depth,0.,0.]);goal_axis=rots[pn][:,0]
            actual=poses[w.object_name][:3];error=desired-actual
            if w.follow_pen is not None:
                self.follow_bias=np.clip(self.follow_bias+.02*error,-.003,.003);error=error+self.follow_bias
            pos=tcp+np.clip(error,-.004,.004)
            axis=rots[w.object_name][:,0];cross=np.cross(axis,goal_axis);sine=np.linalg.norm(cross);angle=math.atan2(sine,float(axis@goal_axis))
            rot=trot if sine<1e-8 else axis_angle_matrix(cross/sine,angle)@trot
        else:
            goal=self.pick_goal if w.pick_name else np.array(w.pos);goal_rot=self.pick_rotation if w.pick_name else grip_rotation(w.yaw)
            pos=self.start_tcp+s*(goal-self.start_tcp);rot=ik.slerp_matrix(self.start_rot,goal_rot,s)
        self.last=(pos,rot,finger,w.label);return self.last

class PenPolicy(YamScriptedPolicy):
    def reset(self,scene):
        self._plans={side:FeedbackPlan(points,self._bases[side],side) for side,points in self.plan_for(scene.instruction,scene.target).items()};self._end=max(p.duration for p in self._plans.values());self._seat_checks=[]
        elapsed=0.
        for w in self._plans['left'].waypoints:
            elapsed+=w.duration
            if w.label=='seat':self._seat_checks.append((elapsed,w.follow_pen))
    def act(self,obs):
        for side,q in zip(('left','right'),split(obs.state['joint_pos'])):self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
        result=super().act(obs)
        from pen_capping.embodiment import FEEDBACK
        for deadline,index in self._seat_checks:
            if float(obs.state_time)>deadline+.1 and index not in FEEDBACK.get('pairs',{}):
                result.actions[0].meta['request_stop']=True
                print('DEMO_STOP_MISSING_RETENTION',index,flush=True);break
        return result
def make_policy(**kwargs):
    kwargs.setdefault('hold_s',5.)
    return PenPolicy(**kwargs)
