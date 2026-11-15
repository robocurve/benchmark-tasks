"""Passive thread bending candidate, independent of task state or contacts.

For dual segment length h, k=B/h and U=0.5*k*theta**2. Native D6 swing
drives implement two bending components; axial twist stays free. The starting
candidate B=1.2e-9 N m^2 is the order of a measured 0.236 mm polyester thread
(Amir et al., 2020, DOI 10.1080/00405000.2019.1646076, Table 1). Applying it
to the coarser 0.8 mm collision strand is an explicit low-stiffness engineering
approximation, not calibration of this thread. Zero preserves the old model.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class ThreadBending:
    rigidity_nm2: float = 1.2e-9
    damping_ratio: float = .1

    def gains(self,lengths,radius,density):
        lengths=np.asarray(lengths,float)
        if (lengths.ndim!=1 or len(lengths)<3 or not np.isfinite(lengths).all()
                or np.any(lengths<=0) or not np.isfinite((self.rigidity_nm2,self.damping_ratio,radius,density)).all()
                or self.rigidity_nm2<0 or self.damping_ratio<0 or radius<=0 or density<=0):
            raise ValueError('Finite physical thread bending parameters required')
        dual=(lengths+np.roll(lengths,1))/2
        stiffness=self.rigidity_nm2/dual
        masses=np.pi*radius**2*lengths*density
        # Cylinder transverse inertia is only the damping estimate. Actual
        # native capsule inertia is read back by the diagnostic, not replaced.
        inertia=masses*(lengths**2/12+radius**2/4)
        previous=np.roll(inertia,1)
        reduced=inertia*previous/(inertia+previous)
        damping=2*self.damping_ratio*np.sqrt(stiffness*reduced)
        return stiffness,damping


def planar_frames(nodes):
    """Continuous material frames for planar loops; physical X axis unchanged."""
    nodes=np.asarray(nodes,float);direction=np.roll(nodes,-1,axis=0)-nodes
    length=np.linalg.norm(direction,axis=1)
    if nodes.ndim!=2 or nodes.shape[1]!=3 or np.any(length<=0):
        raise ValueError('Non-degenerate closed thread polygon required')
    tangent=direction/length[:,None]
    crosses=np.cross(tangent,np.roll(tangent,-1,axis=0))
    normal=crosses[np.argmax(np.linalg.norm(crosses,axis=1))]
    if np.linalg.norm(normal)<1e-9:raise ValueError('Thread reset must have a plane')
    normal/=np.linalg.norm(normal)
    if np.max(np.abs((nodes-nodes[0])@normal))>1e-9:
        raise ValueError('Passive-bending reset requires a planar thread loop')
    z=np.broadcast_to(normal,tangent.shape)
    matrix=np.stack((tangent,np.cross(z,tangent),z),axis=2)
    return Rotation.from_matrix(matrix).as_quat()


def circular_loop(lengths):
    """Cyclic polygon with exactly the original segment lengths and mass.

    Used only as a free-loop equilibrium diagnostic; never changes the seam
    reset. There is no fixture, attachment or external shape-maintaining force.
    """
    from scipy.optimize import brentq
    lengths=np.asarray(lengths,float)
    if len(lengths)<3 or np.any(lengths<=0) or 2*max(lengths)>=sum(lengths):
        raise ValueError('Lengths must close into a non-degenerate cyclic polygon')
    def residual(r):return np.sum(2*np.arcsin(np.clip(lengths/(2*r),-1,1)))-2*np.pi
    radius=brentq(residual,max(lengths)*.500000001,sum(lengths),xtol=1e-15)
    angle=np.r_[0.,np.cumsum(2*np.arcsin(lengths/(2*radius)))[:-1]]
    points=np.column_stack((radius*np.cos(angle),np.zeros(len(angle)),radius*np.sin(angle)))
    return points-points.mean(0)


def elastic_diagnostics(poses,velocity,mass,inertia,lengths,rigidity):
    """Read-only kinetic energy and rotation-invariant bending-energy proxy.

    The centreline angle proxy agrees with the isotropic small-angle law. It is
    not asserted to equal PhysX's finite-angle D6 swing-coordinate potential.
    Therefore it is reported as evidence, never used as a pass/fail gate.
    """
    poses=np.asarray(poses,float);velocity=np.asarray(velocity,float)
    rotations=Rotation.from_quat(poses[:,3:]).as_matrix()
    tangent=rotations[:,:,0]
    angle=np.arccos(np.clip(np.sum(tangent*np.roll(tangent,1,axis=0),axis=1),-1,1))
    k=float(rigidity)/((np.asarray(lengths)+np.roll(lengths,1))/2)
    mass=np.asarray(mass,float);comv=np.average(velocity[:,:3],axis=0,weights=mass)
    relative=velocity[:,:3]-comv
    angular_local=np.einsum('fji,fj->fi',rotations,velocity[:,3:])
    kinetic=.5*np.sum(mass*np.sum(relative**2,axis=1))+.5*np.einsum('fi,fij,fj->',angular_local,inertia,angular_local)
    return dict(bending_energy_proxy_j=float(.5*np.sum(k*angle**2)),
                relative_kinetic_energy_j=float(kinetic),maximum_bend_rad=float(angle.max()),
                centre_velocity_m_s=comv.tolist())
