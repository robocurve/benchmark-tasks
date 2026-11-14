"""Guarded Cartesian waypoints executed exclusively through normal robot actions."""
from dataclasses import dataclass
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
@dataclass
class Target:
    seconds:float
    pos:np.ndarray
    rot:np.ndarray
    grip:float
    label:str
class Plan:
    def __init__(self,targets,side,q=None,grip=1.):
        self.targets=targets;self.base=station.BASES[side];self.q=np.array(READY if q is None else q);self.start_p,self.start_r=ik.tcp_pose(self.q,self.base);self.start_g=grip;self.duration=sum(t.seconds for t in targets)
    def action7(self,time):
        p0,r0,g0=self.start_p,self.start_r,self.start_g;label='done'
        for t in self.targets:
            if time<t.seconds:
                s=ik.smoothstep(time/t.seconds);p=p0+s*(t.pos-p0);r=ik.slerp_matrix(r0,t.rot,s);g=g0+s*(t.grip-g0);label=t.label;break
            time-=t.seconds;p0,r0,g0=t.pos,t.rot,t.grip
        else:p,r,g=p0,r0,g0
        self.q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),self.q,base_pos=self.base,iters=100)
        if pe>.002 or re>.02:raise RuntimeError(f'IK guard {label}: position={pe}, rotation={re}')
        return np.r_[self.q,g],label

def rz(a):
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
