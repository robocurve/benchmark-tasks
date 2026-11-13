"""Optional demo joint tracking using only ordinary proprioception.

This policy component never reads task-object poses, material state or scores.
It adds bounded PI tracking to the precompiled joint reference through exactly
normal action targets; native motors, effort limits and physics are unchanged.
The fixed reference remains available for independent replay and inspection.
"""
from __future__ import annotations
import numpy as np


class JointTracking:
    def __init__(self, *, proportional=1.5, integral=2., bias_limit=.12, slew_limit=.006):
        self.kp=float(proportional);self.ki=float(integral)
        self.limit=float(bias_limit);self.slew=float(slew_limit)
        self.integral=np.zeros(12);self.bias=np.zeros(12)
        self.indices=np.r_[np.arange(6),np.arange(7,13)]

    def action(self, reference, observed, *, enabled):
        ref=np.asarray(reference,float);obs=np.asarray(observed,float)
        if ref.shape!=(14,) or obs.shape!=(14,) or not np.isfinite(ref).all() or not np.isfinite(obs).all():
            raise ValueError('Expected finite ordinary14-element joint observation and action')
        if not enabled:
            self.integral[:]=0.;self.bias[:]=0.;return ref.copy()
        error=ref[self.indices]-obs[self.indices]
        self.integral=np.clip(self.integral+self.ki*.05*error,-self.limit,self.limit)
        wanted=np.clip(self.kp*error+self.integral,-self.limit,self.limit)
        self.bias+=np.clip(wanted-self.bias,-self.slew,self.slew)
        result=ref.copy();result[self.indices]+=self.bias
        return result
