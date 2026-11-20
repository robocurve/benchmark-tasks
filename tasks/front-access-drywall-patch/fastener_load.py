"""Compliant screw connections acting on native bodies. No pose or velocity writes.

A driven head and the engaged wood share an elastic connection at their actual
intersection. Solve the small coupled Kelvin-Voigt system implicitly so a light
strip and a heavier panel can bear several closely spaced screws at 240 Hz.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from surface_process_probe import WORLD

NAMES=('strip','spare_strip','patch')
MASS=(.049,.049,.39)
DIMS=((.016,.251,.025),(.016,.251,.025),(.0127,.197,.197))
STIFF=np.array([9000.,10000.,10000.,12.,6.4,6.4])
CLAMP_N=12.
DAMP=np.array([4.,4.,4.,.035,.035,.035])


def skew(r):
    x,y,z=r
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


class Connections:
    def __init__(self):
        self.mounts=[]
        self.max_error=0.

    def insert(self,front,wood,point,poses):
        child=NAMES.index(wood)
        parent=2 if front=='patch' else None
        c=poses[wood];rc=R.from_quat(c[3:])
        a=rc.inv().apply(point-c[:3])
        if parent is None:
            b=point.copy();rest=rc
        else:
            p=poses['patch'];rp=R.from_quat(p[3:])
            b=rp.inv().apply(point-p[:3]);rest=rp.inv()*rc
        self.mounts.append(dict(child=child,parent=parent,a=a,b=b,rest=rest,overload_s=0.,broken=False,max_gap=0.,gap=0.,peak_force=0.,peak_moment=0.))

    def forces(self,poses,vel,dt):
        if not self.mounts:return None
        rs=R.from_quat(poses[:,3:]);r=rs.as_matrix()
        mass=np.zeros((18,18))
        for i,(m,d) in enumerate(zip(MASS,DIMS)):
            k=6*i;sq=np.square(d)
            mass[k:k+3,k:k+3]=np.eye(3)*m
            mass[k+3:k+6,k+3:k+6]=r[i]@np.diag(m*(sq.sum()-sq)/12)@r[i].T
        js=[];es=[];used=[]
        for mount in self.mounts:
            if mount['broken']:continue
            c,p=mount['child'],mount['parent']
            ac=r[c]@mount['a'];world_c=poses[c,:3]+ac
            jac=np.zeros((6,18))
            jac[:,6*c:6*c+6]=np.eye(6);jac[:3,6*c+3:6*c+6]=-skew(ac)
            if p is None: world_p=mount['b'];desired=mount['rest']
            else:
                ap=r[p]@mount['b'];world_p=poses[p,:3]+ap;desired=rs[p]*mount['rest']
                jac[:,6*p:6*p+6]=-np.eye(6);jac[:3,6*p+3:6*p+6]=skew(ap)
            error=np.r_[world_c-world_p,(rs[c]*desired.inv()).as_rotvec()]
            # Tightening a seated head draws the timber towards the board and
            # preloads their physical interface. Preserve the measured gap
            # separately from the elastic strain associated with screw tension.
            mount['gap']=float(np.linalg.norm(error[:3]))
            normal=np.array([1.,0.,0.]) if p is None else r[p][:,0]
            error[:3]+=CLAMP_N*normal/STIFF[:3]
            js.append(jac);es.append(error);used.append(mount)
        if not js:return None
        j=np.vstack(js);err=np.concatenate(es)
        k=np.tile(STIFF,len(js));d=np.tile(DAMP,len(js))
        kdt=d+dt*k
        v=np.linalg.solve(mass+dt*(j.T*kdt)@j,mass@vel.reshape(-1)-dt*j.T@(k*err))
        reactions=-(k*err+kdt*(j@v)).reshape(-1,6)
        for index,(m,s) in enumerate(zip(used,reactions)):
            gap=m['gap']
            force=float(np.linalg.norm(s[:3]));moment=float(np.linalg.norm(s[3:]))
            m['max_gap']=max(m['max_gap'],gap)
            m['gap']=gap
            m['peak_force']=max(m['peak_force'],force);m['peak_moment']=max(m['peak_moment'],moment)
            m['overload_s']=m['overload_s']+dt if force>120. or moment>2.2 else 0.
            if m['overload_s']>=.05:
                m['broken']=True;s[:]=0.
        return (j.T@reactions.reshape(-1)).reshape(3,6)

    def bind(self,sim,external_wrench,on_apply,hz=240):
        import torch
        from isaaclab_physx.physics.physx_manager import PhysxManager
        view=sim.create_rigid_body_view([WORLD+'/'+n for n in NAMES])
        order=[str(n).rsplit('/',1)[-1] for n in view.prim_paths]
        reorder=[order.index(n) for n in NAMES];undo=np.argsort(reorder)
        device=view.get_transforms().device
        inds=torch.arange(3,device=device,dtype=torch.int32)
        base=PhysxManager.step
        sample=0
        def step(cls):
            nonlocal sample
            external=external_wrench()
            internal=None
            if self.mounts:
                pos=view.get_transforms().cpu().numpy()[reorder]
                vel=view.get_velocities().cpu().numpy()[reorder]
                internal=self.forces(pos,vel,1/hz)
            if internal is not None or external is not None:
                summed=np.zeros((len(NAMES),6)) if internal is None else internal.copy()
                if external is not None:
                    summed+=external
                f=torch.tensor(summed[undo,:3],device=device,dtype=torch.float32).contiguous()
                t=torch.tensor(summed[undo,3:],device=device,dtype=torch.float32).contiguous()
                view.apply_forces_and_torques_at_position(f,t,None,inds,True)
                on_apply(sample,internal,summed,f[order.index('patch')])
            base()
            sample+=1
        PhysxManager.step=classmethod(step)
