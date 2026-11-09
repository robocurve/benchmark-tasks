"""Choose between checked routes using recent native RGB feature evidence.

This is a specialized view-quality decision, not a grasp-pose planner or an
occlusion classifier. It does not read object state, scene truth or arm poses.
"""
from collections import deque
import numpy as np
from .vision import alignment


class ViewDecision:
    def __init__(self):
        self.history=deque(maxlen=20)
        self.choice=None

    def update(self,rgb):
        measurement=alignment(rgb)
        available=(rgb is not None and np.asarray(rgb).shape==(176,320,3)
                   and np.isfinite(rgb).all())
        if available:
            available=float(np.asarray(rgb,dtype=float).std(axis=(0,1)).mean())>=6.
        if not available:
            self.history.clear()
            return dict(choice=None,reason='missing or nearly uniform wrist image',
                        frames=0,valid_frames=0,alignment=measurement)
        self.history.append(bool(measurement['valid']))
        count=sum(self.history)
        if len(self.history)==20:
            recent=list(self.history)[-3:]
            if count>=16 and all(recent):self.choice='continue'
            elif count<=2 and not any(recent):self.choice='recover'
        return dict(choice=self.choice,reason='native RGB alignment-feature history',
                    frames=len(self.history),valid_frames=count,alignment=measurement)
