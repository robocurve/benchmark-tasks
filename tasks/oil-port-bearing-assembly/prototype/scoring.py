"""Outcome-only local rubric. No arm, grasp, route, or camera-use requirement."""
from collections import deque
import math
import numpy as np
from . import layout as L

SCORING_REVISION='released-seated-full-10mm-port-v3'

class Evaluator:
    def __init__(self):
        self.window=deque(maxlen=21)
        self.last_step=None

    def update(self,row):
        """One pose at 20 Hz; 21 consecutive samples span a real 1.0 second."""
        result=dict(score=0.,complete=False,stable_seconds=0.,distance_m=1.,
                    seated=False,released=False,passage_clear=False,valid=False)
        try:
            step=row['step'];state=row['objects']['sleeve'];contact=row['contact_interval']
            passage=row['passage'];p=np.asarray(state['pose_xyzw'],dtype=float)
            velocity=np.r_[state['linear_velocity'],state['angular_velocity']].astype(float)
            if p.shape!=(7,) or velocity.shape!=(6,) or not np.isfinite(np.r_[p,velocity]).all():raise ValueError('invalid state')
            q=p[3:];norm=float(np.linalg.norm(q))
            if abs(norm-1)>.001:raise ValueError('invalid quaternion')
            q=q/norm
            if not contact['valid'] or contact['samples']<12 or not passage['valid']:raise ValueError('invalid measurement')
            if self.last_step is not None and step!=self.last_step+1:self.window.clear()
            radial=float(np.linalg.norm(p[:2]-np.asarray(L.SEATED)[:2]));axial=abs(float(p[2]-L.SEATED[2]))
            tilt=math.acos(min(1.,abs(1-2*(q[0]**2+q[1]**2))))
            seated=radial<=.0008 and axial<=.0005 and tilt<=math.radians(1)
            released=not contact['robot_sleeve_contact'] and contact['supported_all_substeps']
            slow=np.linalg.norm(velocity[:3])<=.003 and np.linalg.norm(velocity[3:])<=.05
            result.update(valid=True,seated=bool(seated),released=bool(released),passage_clear=bool(passage['clear']),
                          distance_m=float(np.linalg.norm(p[:3]-L.SEATED)),radial_error_m=radial,axial_error_m=axial,tilt_deg=math.degrees(tilt))
            if seated and released and slow:
                self.window.append((step,p.copy(),bool(passage['clear'])))
            else:self.window.clear()
            if self.window:
                result['stable_seconds']=(step-self.window[0][0])/20
            if len(self.window)==21:
                # Prevent slow drift from accumulating over the entire hold window.
                reference=self.window[0][1]
                displacement=max(np.linalg.norm(t[1][:3]-reference[:3]) for t in self.window)
                rotation=max(2*math.acos(min(1.,abs(float(np.dot(t[1][3:]/np.linalg.norm(t[1][3:]),reference[3:]/np.linalg.norm(reference[3:])))))) for t in self.window)
                if displacement<=.0005 and rotation<=math.radians(1):
                    result['score']=1. if all(t[2] for t in self.window) else .5
                    result['complete']=result['score']==1.
        except (KeyError,ValueError,TypeError,IndexError):
            self.window.clear()
        self.last_step=row.get('step')
        return result
