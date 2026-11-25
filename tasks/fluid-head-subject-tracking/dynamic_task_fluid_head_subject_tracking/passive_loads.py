"""Disclosed cutting resistance: a passive material response, never a pose driver."""
from dataclasses import dataclass
import math
import numpy as np
from dynamic_task_fluid_head_subject_tracking.scoring import _rotate


@dataclass(frozen=True)
class PlaningResistance:
    """Nominal forward cutting load at the actual blade point on fixed stock.

    Only native contact, an upright forward-facing blade, an in-stock working
    point and positive cutting velocity activate it. Its power at the point of
    application is always nonpositive. The geometry is fixed in world space;
    this class is not suitable for a moving workpiece without a frame extension.
    """
    point_local: tuple[float,float,float]
    bounds_low: tuple[float,float,float]
    bounds_high: tuple[float,float,float]
    contact_pair: str
    force_N: float = 25.
    speed_scale_m_s: float = .005
    minimum_normal_N: float = .1

    def __post_init__(self):
        vectors=(self.point_local,self.bounds_low,self.bounds_high)
        if any(len(v)!=3 or not np.isfinite(v).all() for v in vectors):
            raise ValueError('cutting geometry must contain three finite coordinates')
        if not np.all(np.asarray(self.bounds_high)>self.bounds_low):
            raise ValueError('cutting bounds must have positive extent')
        if (not self.contact_pair or not all(math.isfinite(v) and v>0 for v in
                (self.force_N,self.speed_scale_m_s,self.minimum_normal_N))):
            raise ValueError('cutting load needs a contact pair and finite positive parameters')

    def evaluate(self,pose,velocity,com_local,normal_N,*,dt=None,inverse_mass=None,inverse_inertia_local=None):
        """Return world force, world application point and its actual velocity.

        PhysX reports COM linear velocity; its lever arm must start at the
        native COM, rather than at the visual or actor origin.
        """
        pose=np.asarray(pose,dtype=float);velocity=np.asarray(velocity,dtype=float)
        com_local=np.asarray(com_local,dtype=float)
        if (pose.shape!=(7,) or velocity.shape!=(6,) or com_local.shape!=(3,)
                or not np.isfinite(np.r_[pose,velocity,com_local,normal_N]).all()):
            raise ValueError('invalid native state for passive cutting load')
        norm=np.linalg.norm(pose[3:]);
        if norm<1e-8:raise ValueError('invalid native body quaternion')
        q=pose[3:]/norm
        lever=_rotate(tuple(np.asarray(self.point_local)-com_local),tuple(q))
        point=pose[:3]+_rotate(self.point_local,tuple(q))
        speed=velocity[:3]+np.cross(velocity[3:],lever)
        force=np.zeros(3)
        upright=_rotate((0,0,1),tuple(q))[2]>=math.cos(math.radians(8))
        forward=_rotate((0,1,0),tuple(q))[1]>.7
        inside=np.all(point>=self.bounds_low) and np.all(point<=self.bounds_high)
        if inside and upright and forward and abs(normal_N)>=self.minimum_normal_N and speed[1]>0:
            magnitude=self.force_N*math.tanh(speed[1]/self.speed_scale_m_s)
            discrete=(dt,inverse_mass,inverse_inertia_local)
            if any(v is not None for v in discrete):
                if any(v is None for v in discrete):raise ValueError('incomplete native mass/time data')
                inertia=np.asarray(inverse_inertia_local,dtype=float)
                if (not math.isfinite(dt) or dt<=0 or not math.isfinite(inverse_mass) or inverse_mass<=0
                        or inertia.shape!=(3,3) or not np.isfinite(inertia).all()
                        or not np.allclose(inertia,inertia.T,atol=1e-6)
                        or np.linalg.eigvalsh(inertia).min()<-1e-7):
                    raise ValueError('invalid native mass/time data')
                R=np.column_stack([_rotate(tuple(v),tuple(q)) for v in np.eye(3)])
                moment=np.cross(lever,(0.,1.,0.))
                inverse_effective_mass=inverse_mass+float(moment@(R@inertia@R.T)@moment)
                # An explicit friction impulse must not reverse the blade by
                # itself and add kinetic energy at low speed. Include both
                # translation and the off-COM torque in its stopping bound.
                magnitude=min(magnitude,speed[1]/(dt*inverse_effective_mass))
            force[1]=-magnitude
        if np.dot(force,speed)>1e-9:raise RuntimeError('cutting resistance supplied positive power')
        return force,point,speed
