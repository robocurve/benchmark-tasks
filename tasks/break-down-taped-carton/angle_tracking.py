"""Read-only, atomic native-substep continuity evidence. Angles are degrees.

No modulo-selected physical branch, maxima latch, body writes or 20 Hz fallback.
Native integration and schema-rate enforcement still require a native checkpoint.
"""
import math
import numpy as np

VERSION='carton-continuous-bottom-v1'
WALLS=('north','south','east','west')
BODIES=tuple(n for wall in WALLS for n in (wall,wall+'_bottom'))
PHYSICS_DT=1./240.
SUBSTEPS=12
NUMERIC_RAD=1e-5  # float32 quaternion/readback roundoff, not a physical allowance


def rotation(quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.)>1e-4:
        raise ValueError('invalid native xyzw quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def principal_delta(now,previous):
    return (now-previous+180.)%360.-180.


class ContinuousBottomAngles:
    def __init__(self,caps_deg_s,physics_dt=PHYSICS_DT):
        self.failure=None;self.initialized=False;self.counter=None;self.time=None
        self.angles=None;self.previous=None;self.raw=None;self.radius=None
        self.dt=float(physics_dt);self.caps=dict(caps_deg_s)
        if self.dt!=PHYSICS_DT or set(self.caps)!=set(BODIES):
            raise ValueError('unverified timestep or body rate-cap topology')
        for n in BODIES:
            if not math.isfinite(self.caps[n]) or self.caps[n]<=0:
                raise ValueError('unproven native angular rate cap: '+n)
        for n in WALLS:
            if math.radians((self.caps[n]+self.caps[n+'_bottom'])*self.dt)>=math.pi:
                raise ValueError('substep rate bound cannot uniquely unwrap')

    def invalidate(self,reason):
        if self.failure is None:self.failure=str(reason)
        return False

    def _read(self,poses,velocities):
        if set(poses)!=set(BODIES) or set(velocities)!=set(BODIES):
            raise ValueError('missing/extra native body')
        matrices={}
        for n in BODIES:
            p=np.asarray(poses[n],dtype=float);v=np.asarray(velocities[n],dtype=float)
            if p.shape!=(7,) or v.shape!=(6,) or not np.isfinite(p).all() or not np.isfinite(v).all():
                raise ValueError('nonfinite/malformed native state: '+n)
            if np.linalg.norm(v[3:])>math.radians(self.caps[n])+NUMERIC_RAD:
                raise ValueError('native angular velocity exceeds schema cap: '+n)
            matrices[n]=rotation(p[3:])
        raw=[];radius=[]
        for n in WALLS:
            rel=matrices[n].T@matrices[n+'_bottom']
            raw.append(math.degrees(math.atan2(rel[2,1],rel[1,1])))
            radius.append(math.hypot(rel[1,1],rel[2,1]))
        return matrices,np.array(raw),np.array(radius)

    def initialize(self,counter,time_s,poses,velocities):
        if self.failure:return False
        if self.initialized:return self.invalidate('unexpected reseed/reset')
        try:
            if type(counter) is not int or counter!=0 or time_s!=0.:
                raise ValueError('only verified normal-reset counter/time zero can initialize')
            matrices,raw,radius=self._read(poses,velocities)
            if np.max(np.abs(raw))>1e-3 or np.min(radius)<1.-1e-5:
                raise ValueError('normal reset is not authored bottom-hinge zero')
        except (ValueError,TypeError) as e:return self.invalidate(e)
        self.initialized=True;self.counter=counter;self.time=float(time_s)
        self.previous=matrices;self.raw=raw;self.radius=radius;self.angles=np.zeros(4)
        return True

    def advance(self,counter,time_s,poses,velocities):
        if self.failure:return False
        try:
            if not self.initialized:raise ValueError('no verified normal-reset initialization')
            if type(counter) is not int or counter!=self.counter+1:
                raise ValueError('native substep gap, duplicate or reset')
            if not math.isfinite(time_s) or abs(time_s-self.time-self.dt)>1e-9:
                raise ValueError('native substep time gap/reset')
            matrices,raw,radius=self._read(poses,velocities)
            for n in BODIES:
                relative=self.previous[n].T@matrices[n]
                angle=math.acos(float(np.clip((np.trace(relative)-1.)/2.,-1.,1.)))
                if angle>math.radians(self.caps[n])*self.dt+NUMERIC_RAD:
                    raise ValueError('native body SO3 increment exceeds schema bound: '+n)
            for i,n in enumerate(WALLS):
                b=math.radians((self.caps[n]+self.caps[n+'_bottom'])*self.dt)
                if self.radius[i]<=2.*math.sin(b/2.)+NUMERIC_RAD:
                    raise ValueError('projected-angle branch ambiguous: '+n)
            angles=self.angles+np.array([principal_delta(a,b) for a,b in zip(raw,self.raw)])
        except (ValueError,TypeError) as e:return self.invalidate(e)
        # Commit all four only after every body/joint has passed validation.
        self.counter=counter;self.time=float(time_s);self.previous=matrices
        self.raw=raw;self.radius=radius;self.angles=angles
        return True

    def evidence(self,control_index,substeps):
        return dict(version=VERSION,valid=bool(self.initialized and not self.failure),failure=self.failure,
                    initialized_from_authored_zero=self.initialized,initial_counter=0,
                    physics_dt_s=self.dt,control_index=int(control_index),counter=self.counter,
                    time_s=self.time,substeps_in_control=int(substeps),
                    angles_deg=None if self.angles is None else self.angles.tolist())


def evidence_error(evidence,angles,time_s,expected_control):
    """Fail closed on old logs, missing first history, stale/mixed-angle evidence."""
    try:
        e=evidence
        if not isinstance(e,dict) or e.get('version')!=VERSION:return 'missing continuous angle evidence'
        if e.get('valid') is not True or e.get('failure') is not None:return 'invalid continuous angle history'
        if e.get('initialized_from_authored_zero') is not True or e.get('initial_counter')!=0:return 'unknown angle prehistory'
        if e.get('physics_dt_s')!=PHYSICS_DT or e.get('substeps_in_control')!=SUBSTEPS:return 'incomplete native angle cadence'
        if type(e.get('control_index')) is not int or e['control_index']!=expected_control:return 'missing angle control interval'
        if type(e.get('counter')) is not int or e['counter']!=SUBSTEPS*expected_control:return 'wrong angle native counter'
        if not math.isfinite(time_s) or abs(time_s-expected_control*.05)>1e-8 or abs(e['time_s']-time_s)>1e-8:return 'stale angle evidence time'
        a=np.asarray(angles,dtype=float);b=np.asarray(e['angles_deg'],dtype=float)
        if a.shape!=(4,) or b.shape!=(4,) or not np.isfinite(a).all() or not np.isfinite(b).all() or np.max(np.abs(a-b))>1e-9:return 'mixed angle evidence'
    except (ValueError,TypeError,KeyError):return 'malformed continuous angle evidence'
    return None
