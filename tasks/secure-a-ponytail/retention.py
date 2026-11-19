"""Evaluator's gentle axial pull: force only, never a band pose constraint.

After three seconds with clear hands, ramp a 1 N downward pull over 0.5 s,
hold three seconds, then unload over 0.5 s. Real equivalent: a spring scale
hooked through the elastic, pulled towards the hair ends. Count displacement
from the unloaded starting position; a loose band sliding over 10 mm fails.
"""
import numpy as np

def fresh():
    return dict(elapsed=0., origin=None, maximum_slip_mm=0., completed=False)

def advance(state, centre, released, moved_from_rest, dt=.05):
    state=dict(state)
    if released < 3.-1e-8 or not moved_from_rest:
        return fresh(), 0.
    if state['origin'] is None:
        state['origin']=np.asarray(centre).tolist()
    state['maximum_slip_mm']=max(state['maximum_slip_mm'],
        float(np.linalg.norm(np.asarray(centre)-state['origin'])*1000))
    state['elapsed']=min(4.,state['elapsed']+dt)
    t=state['elapsed']
    state['completed']=t>=4.-1e-8
    force=min(1.,t/.5,max(0.,(4.-t)/.5))
    return state,force
