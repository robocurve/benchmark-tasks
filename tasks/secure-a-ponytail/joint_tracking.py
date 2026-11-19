"""Bounded arm tracking from ordinary joint observations only.

Native motors, effort limits and contacts are unchanged. No prop, cloth,
assessor or score data enters this controller. The runner additionally applies
normal joint bounds and speed limits to every requested command.
"""
import numpy as np


class JointTracking:
    def __init__(self,proportional=1.5,integral=2.,bias_limit=.12,slew_limit=.006,side='right'):
        if side not in ('left','right'):raise ValueError('Unknown robot arm')
        self.indices=slice(0,6) if side=='left' else slice(7,13)
        self.kp=float(proportional);self.ki=float(integral);self.limit=float(bias_limit);self.slew=float(slew_limit)
        self.integral=np.zeros(6);self.bias=np.zeros(6)

    def action(self,reference,observed,*,enabled):
        ref=np.asarray(reference,float);obs=np.asarray(observed,float)
        if ref.shape!=(14,) or obs.shape!=(14,) or not np.isfinite(ref).all() or not np.isfinite(obs).all():
            raise ValueError('Expected finite ordinary14-element joint action and observation')
        if not enabled:
            self.integral[:]=0.;self.bias[:]=0.;return ref.copy()
        error=ref[self.indices]-obs[self.indices]
        self.integral=np.clip(self.integral+self.ki*.05*error,-self.limit,self.limit)
        wanted=np.clip(self.kp*error+self.integral,-self.limit,self.limit)
        self.bias+=np.clip(wanted-self.bias,-self.slew,self.slew)
        result=ref.copy();result[self.indices]+=self.bias
        return result
