"""Finite physical gripper-only attempts; these are baselines, not a proof of impossibility."""
import numpy as np
from functools import lru_cache
from yam_arena.demo import ArmPlan,Waypoint as W,grip_rotation
from yam_arena import ik,YAM_URDF
from yam_arena.constants import ARM_JOINTS,READY,TCP_OFFSET
from yam_arena.urdf import forward_kinematics,axis_angle_matrix
from fixture_geometry import FILM_Z,TABLE_Z

SCENARIOS=('top','single_front','single_back','angled_front','angled_back',
           'side_edge','far_edge','lift_carrier','push_carrier')


@lru_cache(maxsize=1)
def collision_parts():
    from audit_geometry import load_parts
    return load_parts()[0]


class TiltPlan(ArmPlan):
    def __init__(self,waypoints,base,tilt):
        self.tilt=float(tilt)
        super().__init__(waypoints,base)

    def target(self,t):
        previous=self.start_pos,self.start_rot,self.finger_start
        for w in self.waypoints:
            pos=np.asarray(w.pos)
            rot=axis_angle_matrix(np.array([1.,0,0]),self.tilt)@grip_rotation(w.yaw)
            if t<w.duration:
                s=ik.smoothstep(t/w.duration)
                return (previous[0]+s*(pos-previous[0]),ik.slerp_matrix(previous[1],rot,s),
                        previous[2]+s*(w.finger-previous[2]),w.label)
            t-=w.duration;previous=pos,rot,w.finger
        return *previous,'done'


def anchored_tcp(target,anchor,finger,yaw,tilt):
    """Place the chosen finger's lowest 3 mm band at a target, using its collision mesh."""
    parts=collision_parts()
    q=dict(zip(ARM_JOINTS,READY));q.update(joint7=finger,joint8=finger)
    fk=forward_kinematics(YAM_URDF,q);g=fk['gripper']
    rot=axis_angle_matrix(np.array([1.,0,0]),tilt)@grip_rotation(yaw)
    points=[]
    for name in ('tip_left','tip_right'):
        if anchor!='both' and name!=anchor:continue
        tf=np.linalg.inv(g)@fk[name]
        for s in parts[name]:points.append((s.vertices if hasattr(s,'vertices') else s.v)@tf[:3,:3].T+tf[:3,3])
    world=np.vstack(points)@rot.T
    bottom=world[:,2].min();end=world[world[:,2]<bottom+.003]
    offset=np.asarray(target)-[end[:,0].mean(),end[:,1].mean(),bottom]
    return offset+rot@np.asarray(TCP_OFFSET)


def make_plan(name,base):
    if name not in SCENARIOS:raise ValueError(name)
    yaw=np.pi/2 if name=='side_edge' else 0.
    tilt=np.deg2rad(30 if name=='angled_front' else -15 if name=='angled_back' else 0)
    anchor='tip_left' if name in ('single_front','angled_front') else 'tip_right' if name in ('single_back','angled_back') else 'both'
    finger=-.0067
    target=(.28,-.052,FILM_Z-.007)
    if name=='side_edge':target=(.247,-.12,FILM_Z-.001)
    if name=='far_edge':target=(.28,-.185,FILM_Z-.002)
    if name=='lift_carrier':target=(.28,.137,TABLE_Z+.006);finger=-.020
    if name=='push_carrier':target=(.35,.070,TABLE_Z+.005);finger=0.
    tcp=anchored_tcp(target,anchor,finger,yaw,tilt)
    moves=[W(4,tuple(tcp+[0,0,.065]),finger,'approach '+name,yaw),
           W(4,tuple(tcp),finger,'enter '+name,yaw),
           W(2,tuple(tcp),0.,'close or press',yaw)]
    if name=='push_carrier':
        moves.append(W(4,tuple(tcp+[-.055,0,0]),0.,'push carrier',yaw))
    else:
        moves.extend([W(3,tuple(tcp+[0,0,.030]),0.,'lift or hook',yaw),
                      W(4,tuple(tcp+[0,-.050,.030]),0.,'pull',yaw)])
    moves.append(W(2,tuple(tcp+[0,-.05,.070]),-.020,'withdraw',yaw))
    return TiltPlan(moves,base,tilt)


def decoy_waypoints():
    """The scripted demo's right-arm path, but taking and returning the paddle-tip tweezers."""
    from dataclasses import replace
    from motion import demo
    from tool_geometry import TOOLS
    fine,paddle=TOOLS['fine']['stand'],TOOLS['paddle']['stand']
    return [replace(w,pos=(*paddle,w.pos[2])) if np.allclose(w.pos[:2],fine) else w for w in demo()['right']]
