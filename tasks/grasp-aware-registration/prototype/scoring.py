"""Provisional physical-outcome rubric; no view, grasp identity or arm-role reward."""
from collections import deque
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from . import layout as L

SCORING_REVISION = "outcome-v2"
NUMERIC_EPS = 1e-12  # roundoff only: far below physical measurement resolution

def measure(pose):
    p=np.asarray(pose,dtype=float)
    if p.shape!=(7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1)>.001:
        raise ValueError('invalid rigid-body pose')
    rotation=quat_xyzw_to_matrix(p[3:])
    windows=np.array([[x,y,-.009] for x,y in L.WINDOWS])@rotation.T+p[:3]
    targets=np.array(L.crosses());targets[:,2]=L.BOARD_TOP
    # The two identical crosses may exchange identities under a 180-degree turn.
    error=min(float(np.linalg.norm(windows[:,:2]-t[:,:2],axis=1).max()) for t in [targets,targets[::-1]])
    height=float(np.max(np.abs(windows[:,2]-L.BOARD_TOP)))
    tilt=float(np.arccos(np.clip(rotation[2,2],-1,1)))
    return dict(pose=p.tolist(),distance_m=error,height_error_m=height,tilt_rad=tilt)

class Evaluator:
    def __init__(self):self.reset()
    def reset(self):self.history=deque(maxlen=21);self.last_event=0;self.last_sample=0
    def update(self,pose,monitor):
        values=measure(pose);events=monitor.events[self.last_event:];self.last_event=len(monitor.events)
        previous_sample=self.last_sample
        samples=range(previous_sample+1,monitor.samples+1);self.last_sample=monitor.samples
        supported={e['physics_sample'] for e in events if 'plate' in (e['a'].rsplit('/',1)[-1],e['b'].rsplit('/',1)[-1])
                   and ('/goal_board/' in e['a'] or '/goal_board/' in e['b'] or e['a'].endswith('/goal_board') or e['b'].endswith('/goal_board'))}
        held_samples={e['physics_sample'] for e in events if (e['a'].endswith('/plate') and any(s in e['b'] for s in ['/LeftArm/','/RightArm/'])) or
                 (e['b'].endswith('/plate') and any(s in e['a'] for s in ['/LeftArm/','/RightArm/']))}
        valid=bool(monitor.ready and not monitor.error and len(samples)>0 and
                   (previous_sample==0 or len(samples)==12))
        release_support=bool(valid and not held_samples and all(n in supported for n in samples))
        at_pose=bool(valid and monitor.samples in supported and monitor.samples not in held_samples)
        self.history.append(dict(**values,valid=valid,release_support=release_support,at_pose=at_pose))
        full_window=len(self.history)==21 and all(x['valid'] for x in self.history)
        still=False
        if full_window:
            poses=np.array([x['pose'] for x in self.history]);xyz=poses[:,:3]
            linear=float(np.linalg.norm(xyz[:,None,:]-xyz[None,:,:],axis=2).max())
            quats=poses[:,3:]/np.linalg.norm(poses[:,3:],axis=1,keepdims=True)
            angles=2*np.arccos(np.clip(np.abs(quats@quats.T),0,1))
            still=bool(linear<=.001+NUMERIC_EPS and float(angles.max())<=np.deg2rad(1.)+NUMERIC_EPS)
        score=0.
        # 21 poses span 1 second. Include the first pose contact and the next
        # 20 complete physics intervals, not an extra preceding 50 ms.
        history=list(self.history)
        released_window=bool(full_window and history[0]['at_pose'] and all(x['release_support'] for x in history[1:]))
        if still and released_window and all(x['height_error_m']<=.002+NUMERIC_EPS and x['tilt_rad']<=np.deg2rad(2.)+NUMERIC_EPS for x in self.history):
            worst=max(x['distance_m'] for x in self.history)
            score=1. if worst<=.003+NUMERIC_EPS else .5 if worst<=.012+NUMERIC_EPS else 0.
        return dict(**values,score=score,complete=score==1.,monitor_valid=valid,
                    still=still,release_support=release_support,hold_window_s=1. if full_window else max(0,(len(self.history)-1)/20),
                    prototype=True,contact_safety_certified=False,scoring_revision=SCORING_REVISION)
