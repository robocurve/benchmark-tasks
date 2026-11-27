"""Bounded axis feedback for the privileged scripted controller, NumPy only."""
import math
import numpy as np


def bounded(vector, limit):
    vector = np.asarray(vector, dtype=float)
    length = float(np.linalg.norm(vector))
    return vector * min(1., limit / max(length, 1e-12))


def rotation_vector(axis, goal):
    axis = np.asarray(axis, dtype=float)
    goal = np.asarray(goal, dtype=float)
    axis = axis / np.linalg.norm(axis)
    goal = goal / np.linalg.norm(goal)
    cross = np.cross(axis, goal)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(axis @ goal, -1., 1.))
    if sine < 1e-10:
        if cosine >= 0: return np.zeros(3)
        helper = np.eye(3)[int(np.argmin(np.abs(axis)))]
        cross = np.cross(axis, helper)
        return cross / np.linalg.norm(cross) * math.pi
    return cross / sine * math.atan2(sine, cosine)


def matrix(vector):
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12: return np.eye(3)
    x, y, z = vector / angle
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)


class AxisServo:
    """Integral correction addresses persistent tracking error, capped at 3°.

    This controls robot targets; it never changes object states or fit limits.
    Integral accumulation is confined to the final 8° of alignment to avoid
    winding up on large carry rotations. Commands are bounded to 4° per step.
    """
    def __init__(self): self.integral = np.zeros(3)

    def correction(self, axis, goal, dt):
        error = rotation_vector(axis, goal)
        if np.linalg.norm(error) < math.radians(8):
            self.integral = bounded(self.integral + .7 * dt * error, math.radians(3))
        else:
            self.integral *= max(0., 1. - 2. * dt)
        # Suppress irrelevant roll: only perpendicular rotations align axes.
        goal = np.asarray(goal, dtype=float)
        goal = goal / np.linalg.norm(goal)
        self.integral -= goal * float(self.integral @ goal)
        return matrix(bounded(error + self.integral, math.radians(4)))


def rotation_about_object(tcp, object_position, rotation):
    """TCP translation that keeps the held object's root fixed during rotation."""
    return (np.eye(3) - rotation) @ (np.asarray(object_position) - np.asarray(tcp))


def safe_depth(requested, axis_error_rad, lateral_error, current_depth=-.100):
    """Align before barrel enters the mouth; this is a controller guard only."""
    ready = axis_error_rad < math.radians(.75) and lateral_error < .0005
    return requested if ready else min(requested, max(-.100, current_depth))


class RetainedReleaseClock:
    """Shared plan clock waits for retention before timed support release.

    Each pair may add at most 6s. Combined with the 261s maximum plan/hold this bounds
    total planned control time to 285s under the 300s action budget. A pair needs
    0.5s observed retention before its release boundary may be crossed.
    """
    def __init__(self,deadlines):
        self.deadlines=list(deadlines);self.waited={};self.retained_steps={};self.failed=None

    def advance(self,time,dt,pairs):
        for _,i in self.deadlines:
            self.retained_steps[i]=self.retained_steps.get(i,0)+1 if i in pairs else 0
        candidate=time+dt
        for deadline,i in self.deadlines:
            if time<deadline and self.retained_steps[i]>=10:
                # Both hands preserve their last commands after engagement.
                # Skip unused insertion/seating dwell after the same 0.5s hold.
                return deadline+dt
        for deadline,i in self.deadlines:
            if time<deadline<=candidate and self.retained_steps[i]<10:
                self.waited[i]=self.waited.get(i,0.)+dt
                if self.waited[i]>=6.-1e-9:self.failed=i
                return deadline-1e-6
        return candidate


def follow_bias_update(previous,error,axis,pivot,blocked):
    """002 integral with guard-aware axial projection and saturation antiwindup.

    Project only the integral; proportional axial error is left to restore a
    held depth. Keep the world-frame lateral component. Integral magnitude is
    bounded by the original component box's maximum norm (sqrt(3)*3mm).
    """
    previous=np.asarray(previous,dtype=float);error=np.asarray(error,dtype=float)
    axis=np.asarray(axis,dtype=float);axis=axis/np.linalg.norm(axis)
    base=previous.copy()
    if blocked:base-=axis*float(base@axis)
    proposed=np.clip(base+.02*error,-.003,.003)
    if blocked:proposed-=axis*float(proposed@axis)
    raw=error+proposed+np.asarray(pivot)
    outward=(np.abs(raw)>.004)&(error*raw>0)
    proposed=np.where(outward,base,proposed)
    if blocked:proposed-=axis*float(proposed@axis)
    return bounded(proposed,math.sqrt(3)*.003)


class LateralDamping:
    """Brake measured relative motion, without changing target fit criteria.

    013 showed lateral motion before renewed backing contact and roughly200ms
    axial command lag. Filter velocity over100ms;150ms of braking lead is
    bounded to1mm. These are controller choices, not a calibrated plant model.
    Axial motion is excluded so the insertion/restoring P term is preserved.
    """
    def __init__(self):
        self.previous=None;self.velocity=np.zeros(3);self.raw_velocity=np.zeros(3)

    def observe(self,relative,pen_rotation,dt):
        relative=np.asarray(relative,dtype=float)
        if self.previous is None:self.previous=relative.copy()
        self.raw_velocity=(relative-self.previous)/dt
        self.raw_velocity[0]=0.;self.previous=relative.copy()
        self.velocity+=dt/(.100+dt)*(self.raw_velocity-self.velocity)
        return np.asarray(pen_rotation)@bounded(-.150*self.velocity,.001)


class InsertionAdvance:
    """Bound near-mouth axial speed and lead without changing fit criteria.

    The final 20mm advances at no more than 3mm/s with at most 2.5mm lead
    over the measured cap root. Coarse free-space approach stays unchanged.
    """
    def __init__(self): self.reference = None

    def step(self, requested, actual, dt, ready=True):
        if actual < -.102:
            self.reference = None
            return min(requested, -.100)
        if self.reference is None: self.reference = actual
        if ready:self.reference = min(requested, self.reference + .003 * dt, actual + .0025)
        return self.reference
