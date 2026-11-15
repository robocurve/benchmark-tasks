"""Optional robot joint-state tracking for ordinary 14-D YAM commands.

No object pose, contact signal, task material or score is used. The controller
adds bounded integral offsets during slow/held arm targets and respects the
same published joint-position and command-speed limits. It cannot raise any
native actuator effort limit and does not alter gripper commands. Optional
velocity feedforward uses the existing native drive's damping/stiffness ratio:
K*(target + D/K*target_velocity - q) - D*q_velocity. This supplies the missing
desired-velocity term through an ordinary position target; it never changes
the native drive or applies an external torque.
"""
import numpy as np


class JointTracker:
    def __init__(self, ready, lower, upper, maximum_step, *, dt=.05, gain=2., offset_limit=.12,
                 damping_over_stiffness_s=None):
        self.previous=np.asarray(ready,float).copy()
        self.previous_target=self.previous.copy()
        self.lower=np.asarray(lower,float).copy();self.upper=np.asarray(upper,float).copy()
        self.maximum_step=np.asarray(maximum_step,float).copy()
        if any(a.shape!=(14,) or not np.isfinite(a).all() for a in (self.previous,self.lower,self.upper,self.maximum_step)):
            raise ValueError('Finite 14-D robot limits and initial state required')
        if not np.isfinite([dt,gain,offset_limit]).all() or dt<=0 or gain<0 or offset_limit<0:
            raise ValueError('Finite positive timestep and nonnegative tracking gains required')
        if np.any(self.lower>=self.upper) or np.any(self.maximum_step<=0) or np.any(self.previous<self.lower) or np.any(self.previous>self.upper):
            raise ValueError('Inconsistent robot limits')
        self.dt=float(dt);self.gain=float(gain);self.offset_limit=float(offset_limit)
        self.offset=np.zeros(14);self.last=None
        self.arm=np.ones(14,bool);self.arm[[6,13]]=False
        self.damping_over_stiffness_s=np.zeros(14) if damping_over_stiffness_s is None else np.asarray(damping_over_stiffness_s,float).copy()
        if (self.damping_over_stiffness_s.shape!=(14,) or
            not np.isfinite(self.damping_over_stiffness_s).all() or
            np.any(self.damping_over_stiffness_s<0) or
            np.any(self.damping_over_stiffness_s[~self.arm]!=0)):
            raise ValueError('Finite nonnegative arm damping/stiffness ratios with zero gripper terms required')

    def command(self, target, measured_joint_positions):
        target=np.asarray(target,float);actual=np.asarray(measured_joint_positions,float)
        if target.shape!=(14,) or actual.shape!=(14,) or not np.isfinite(target).all() or not np.isfinite(actual).all():
            raise ValueError('Finite ordinary joint targets and measurements required')
        if np.any(target<self.lower) or np.any(target>self.upper):raise ValueError('Nominal target exceeds robot limits')
        # Do not accumulate normal trajectory lag during fast motion. Preserve
        # established gravity/load compensation while following the route.
        desired_velocity=(target-self.previous_target)/self.dt
        slow=np.abs(desired_velocity)<=.15
        integrate=self.arm&slow
        self.offset[integrate]+=self.gain*self.dt*(target-actual)[integrate]
        self.offset=np.clip(self.offset,-self.offset_limit,self.offset_limit)
        self.offset=np.clip(self.offset,self.lower-target,self.upper-target)
        self.offset[~self.arm]=0.
        feedforward=self.damping_over_stiffness_s*desired_velocity
        # Keep the same total correction envelope as integral-only tracking.
        # Abrupt pose-servo corrections therefore cannot bypass the original
        # position-offset or command-speed bounds.
        correction=np.clip(self.offset+feedforward,-self.offset_limit,self.offset_limit)
        correction=np.clip(correction,self.lower-target,self.upper-target)
        desired=target+correction
        command=np.clip(desired,self.previous-self.maximum_step,self.previous+self.maximum_step)
        command=np.clip(command,self.lower,self.upper)
        self.last={'offset_rad':self.offset.tolist(),'maximum_arm_error_rad':float(np.max(np.abs(target-actual)[self.arm])),
                   'velocity_feedforward_rad':feedforward.tolist(),'total_correction_rad':correction.tolist(),
                   'rate_limited_joints':np.flatnonzero(np.abs(command-desired)>1e-10).tolist()}
        self.previous[:]=command;self.previous_target[:]=target
        return command.copy()
