"""Read-only interception of an already verified single-physics-step function.

The native binding must verify runtime source, counters, raw views and rate caps.
This module performs no physical state writes and never substitutes a step.
"""
from angle_tracking import SUBSTEPS


class AngleStepObserver:
    def __init__(self,tracker,original_step,read_state,read_counter,read_time):
        self.tracker=tracker;self.original=original_step;self.read_state=read_state
        self.read_counter=read_counter;self.read_time=read_time
        self.control_index=0;self.in_control=False;self.samples=[]

    def initialize(self):
        poses,velocities=self.read_state()
        self.initial_state=dict(counter=self.read_counter(),time_s=self.read_time(),poses=poses,velocities=velocities)
        return self.tracker.initialize(self.initial_state['counter'],self.initial_state['time_s'],poses,velocities)

    def begin_control(self):
        if self.in_control or self.tracker.failure:
            self.tracker.invalidate('overlapping interval or failed tracker')
            raise RuntimeError(self.tracker.failure)
        if self.read_counter()!=self.tracker.counter:
            self.tracker.invalidate('unobserved native step before control')
            raise RuntimeError(self.tracker.failure)
        self.in_control=True;self.samples=[]

    def step(self,*args,**kwargs):
        if not self.in_control:
            self.tracker.invalidate('native step outside control interval')
            raise RuntimeError(self.tracker.failure)
        if len(self.samples)>=SUBSTEPS:
            self.tracker.invalidate('extra native substep')
            raise RuntimeError(self.tracker.failure)
        # Call once, preserve arguments/return, then observe completed physics.
        try:
            result=self.original(*args,**kwargs)
            poses,velocities=self.read_state()
            if not self.tracker.advance(self.read_counter(),self.read_time(),poses,velocities):
                raise RuntimeError(self.tracker.failure)
        except Exception:
            self.tracker.invalidate('native step/read failed; continuity unknown')
            raise
        self.samples.append(dict(counter=self.tracker.counter,time_s=self.tracker.time,
                                 poses=poses,velocities=velocities,
                                 angles_deg=self.tracker.angles.tolist()))
        return result

    def end_control(self):
        self.in_control=False;self.control_index+=1
        if len(self.samples)!=SUBSTEPS or self.read_counter()!=self.tracker.counter:
            self.tracker.invalidate('incomplete native control interval')
        elif not self.tracker.failure:
            poses,velocities=self.read_state()
            # A reset or unobserved state change after the last substep cannot
            # borrow the preceding interval's continuity evidence.
            if poses!=self.samples[-1]['poses'] or velocities!=self.samples[-1]['velocities']:
                self.tracker.invalidate('native endpoint changed after last substep')
        return self.tracker.evidence(self.control_index,len(self.samples))
