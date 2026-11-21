"""Grade the measured suspension topology and only credit a disturbed, hands-off result."""
import math
import numpy as np
from . import fixed_scene as s

def measurements(p,rot,knot,cords,hands):
    local=(knot-p)@rot
    tilt=math.degrees(math.acos(float(np.clip(rot[2,2],-1,1))))
    angles=[]
    for pts in cords:
        strand=(np.asarray(pts)-p)@rot
        z=strand[:,2]-.05
        crossings=np.flatnonzero(z[:-1]*z[1:]<=0)
        if len(crossings)!=1:continue
        i=int(crossings[0])
        alpha=-z[i]/(z[i+1]-z[i]) if abs(z[i+1]-z[i])>1e-5 else 0.
        xy=(strand[i]+alpha*(strand[i+1]-strand[i]))[:2]
        if .045<=np.linalg.norm(xy)<=.080:angles.append(np.arctan2(xy[1],xy[0]))
    max_gap=None
    wrapped=False
    if len(angles)==4:
        a=np.sort(angles);max_gap=float(np.max(np.diff(np.r_[a,a[0]+2*np.pi])));wrapped=max_gap<np.pi
    supported=bool(wrapped and np.linalg.norm(local[:2])<=s.BASE_RADIUS and -.025<=local[2]<=.008)
    hands=np.asarray(hands);distance=np.min(np.linalg.norm(hands-(p+[0,0,.047]),axis=1))
    cord_clear=min(np.linalg.norm(hands[:,None,:]-np.asarray(c)[None,:,:],axis=2).min() for c in cords)
    return dict(seated=supported,hanging=bool(p[2]>s.TOP+.05),hands_clear=bool(distance>.13 and cord_clear>.055),
                strand_max_gap_deg=math.degrees(max_gap) if max_gap is not None else None,knot_local_m=local.tolist(),tilt_deg=tilt,bottom_offset_m=float(np.linalg.norm(local[:2])),cord_crossings=len(angles))

class Grader:
    def __init__(self):
        self.clear=0;self.pulse_steps=0;self.failed_after_pulse=False
    def update(self,m,dt):
        eligible=m['seated'] and m['hanging'] and m['hands_clear'] and m['tilt_deg']<=10.
        self.clear=self.clear+1 if eligible else 0
        if self.pulse_steps and not eligible:self.failed_after_pulse=True
        full=self.pulse_steps==3 and self.clear>=106 and not self.failed_after_pulse
        return dict(m,clear_steps=self.clear,pulse_steps=self.pulse_steps,score=1. if full else .5 if m['seated'] else 0.)

class Scorer:
    name=s.SCORER
    def __call__(self,record,target):
        from inspect_robots.scorer import Score
        m=record.steps[-1].result.info['fit'] if record.steps else {}
        return Score(value=m.get('score',0.),explanation=str(m))
def make_scorer():return Scorer()
