"""Stock-URDF Cartesian control through bounded 14-D joint position targets."""
import numpy as np
from scipy.optimize import least_squares
from yam_arena import ik
from config import HZ


def rot_from(closing,approach):
    x=np.asarray(closing,float); x/=np.linalg.norm(x)
    z=np.asarray(approach,float); z-=x*(x@z); z/=np.linalg.norm(z)
    return np.column_stack((x,np.cross(z,x),z))


def solve(p,r,q,base):
    def residual(v):
        pos,rot=ik.tcp_pose(v,base)
        return np.r_[pos-p,.25*ik._rot_error(r,rot)]
    fit=least_squares(residual,np.clip(q,ik.LOWER+1e-7,ik.UPPER-1e-7),bounds=(ik.LOWER,ik.UPPER),
                      max_nfev=65,xtol=1e-9,ftol=1e-9,gtol=1e-9)
    error=residual(fit.x)
    if np.linalg.norm(error[:3])>.001 or np.linalg.norm(error[3:])/.25>np.radians(1):
        raise RuntimeError(f'IK failed at {np.asarray(p).round(4).tolist()}: {np.linalg.norm(error[:3])*1000:.2f} mm, '
                           f'{np.degrees(np.linalg.norm(error[3:])/.25):.2f} deg')
    return fit.x


class Arm:
    def __init__(self,side,q):
        self.side=side
        self.base=np.array((0,.305 if side=='left' else -.305,.75))
        self.q=np.asarray(q,float).copy(); self.grip=1.; self.load_N=0.
        self.pos,self.rot=ik.tcp_pose(self.q,self.base)

    def aim(self,p,r):
        self.q=solve(np.asarray(p),r,self.q,self.base)
        self.pos,self.rot=np.asarray(p).copy(),r.copy()

    def command(self):
        q=self.q.copy()
        if self.load_N:
            jz=[]
            p,_=ik.tcp_pose(q,self.base)
            for i in range(6):
                v=q.copy(); v[i]+=1e-5
                jz.append((ik.tcp_pose(v,self.base)[0][2]-p[2])/1e-5)
            # Actuator K is stock. Payload torque becomes a position error offset.
            offset=np.asarray(jz)*self.load_N/np.array([80,80,80,10,10,10])
            if abs(offset).max()>np.radians(12): raise RuntimeError('Payload compensation exceeds 12 degree bound')
            q+=offset
        if np.any(q<ik.LOWER) or np.any(q>ik.UPPER): raise RuntimeError('Action exceeds stock joint limits')
        return np.r_[q,self.grip]

    def move(self,p,r,duration,grip=None):
        p0,r0,g0=self.pos.copy(),self.rot.copy(),self.grip
        n=max(1,round(duration*HZ))
        for k in range(1,n+1):
            s=ik.smoothstep(k/n)
            if grip is not None: self.grip=g0+s*(grip-g0)
            self.aim(p0+s*(np.asarray(p)-p0),ik.slerp_matrix(r0,r,s))
            yield

    def move_q(self,q,duration,grip=None):
        q0,g0=self.q.copy(),self.grip
        n=max(1,round(duration*HZ))
        for k in range(1,n+1):
            s=ik.smoothstep(k/n)
            self.q=q0+s*(np.asarray(q)-q0)
            self.pos,self.rot=ik.tcp_pose(self.q,self.base)
            if grip is not None: self.grip=g0+s*(grip-g0)
            yield


def wait(seconds):
    for _ in range(max(1,round(seconds*HZ))): yield
