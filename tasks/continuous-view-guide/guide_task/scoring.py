"""Ending-state outcome rubric with whole-episode contact and integrity latches."""
from collections import deque
import math
import numpy as np
from . import rules as R
from .measurements import pose_measurements
from .contact import arm

EPS=1e-10

def collar_pair(a,b):
    return b if a.endswith('/collar') else a if b.endswith('/collar') else None

class Evaluator:
    def __init__(self):self.reset()
    def reset(self):
        self.contact_latched=False;self.first_contact_witness=None
        self.processed_events=0;self.processed_violations=0;self.last_sample=None
        self.invalid_reason=None;self.window=deque()
    def invalid(self,reason):
        self.invalid_reason=self.invalid_reason or reason;self.window.clear()
        return dict(version=R.VERSION,score=0.,complete=False,valid=False,
                    invalid_reason=self.invalid_reason,threaded=False,at_goal=False,
                    contact_latched=self.contact_latched,stable=False,released=False,
                    goal_supported=False,hold_s=0.,distance_m=None,progress=0.,pose=None,
                    angle_deg=None,tilt_rad=None)
    def update(self,pose,history):
        if self.invalid_reason:return self.invalid(self.invalid_reason)
        if not history.ready or history.error is not None:return self.invalid('contact monitor unavailable')
        try:
            sample=history.samples
            if isinstance(sample,bool) or not isinstance(sample,(int,np.integer)) or sample<R.SAMPLES_PER_ACTION:
                return self.invalid('invalid physics sample counter')
            previous=sample-R.SAMPLES_PER_ACTION if self.last_sample is None else self.last_sample
            if sample-previous!=R.SAMPLES_PER_ACTION:return self.invalid('missing, stale or duplicated contact interval')
            if len(history.events)<self.processed_events or len(history.violations)<self.processed_violations:
                return self.invalid('contact history truncated')
            m=pose_measurements(pose)
        except (AttributeError,TypeError,ValueError,OverflowError) as exc:
            return self.invalid(str(exc))
        new_events=history.events[self.processed_events:];new_bad=history.violations[self.processed_violations:]
        all_new=[*new_events,*new_bad]
        if any(not isinstance(e.get('physics_sample'),(int,np.integer)) or not 0<e['physics_sample']<=sample for e in all_new):
            return self.invalid('invalid contact timestamp')
        if self.last_sample is not None and any(e['physics_sample']<=previous for e in all_new):
            return self.invalid('late or repeated contact report')
        if self.last_sample is None and any(e['physics_sample']<=previous for e in new_bad):
            return self.invalid('prohibited contact during reset')
        # Ideal contact-transfer witness: one forbidden contact leaves a permanent
        # record, including brief touches. There is no duration or force allowance.
        if new_bad and not self.contact_latched:
            first=min(new_bad,key=lambda event:event['physics_sample'])
            self.first_contact_witness={k:first[k] for k in ('physics_sample','a','b')}
            self.contact_latched=True
        recent=[e for e in new_events if e['physics_sample']>previous]
        pairs=[(e,collar_pair(e['a'],e['b'])) for e in recent]
        robot_contact=any(other is not None and arm(other) is not None for _,other in pairs)
        support_samples={e['physics_sample'] for e,other in pairs if other is not None and '/dock_goal/' in other}
        supported=len(support_samples)==R.SAMPLES_PER_ACTION
        at_goal=(m['distance_m']<=R.POSITION_TOL_M+EPS and m['alignment_deg']<=R.YAW_TOL_DEG+EPS
                 and m['tilt_rad']<=math.radians(R.UPRIGHT_TOL_DEG)+EPS)
        self.window.append(dict(sample=int(sample),centre=m['centre'],quaternion=m['quaternion'],
                                threaded=m['threaded'],at_goal=at_goal,robot_contact=robot_contact,supported=supported))
        while self.window and sample-self.window[0]['sample']>round(R.HOLD_S*R.PHYSICS_HZ):self.window.popleft()
        window=list(self.window);span=(sample-window[0]['sample'])/R.PHYSICS_HZ
        centres=np.array([x['centre'] for x in window]);quats=np.array([x['quaternion'] for x in window])
        position_envelope=float(np.linalg.norm(centres[:,None]-centres[None,:],axis=2).max())
        rotation_envelope=math.degrees(2*math.acos(float(np.clip(np.abs(quats@quats.T).min(),0,1))))
        quiet=position_envelope<=R.POSITION_ENVELOPE_M+EPS and rotation_envelope<=R.ROTATION_ENVELOPE_DEG+EPS
        stable=span>=R.HOLD_S-EPS and quiet
        threaded_window=all(x['threaded'] for x in window);goal_window=all(x['at_goal'] for x in window)
        # The first pose marks the beginning; subsequent intervals span exactly1s.
        free_window=all(not x['robot_contact'] and x['supported'] for x in window[1:])
        hold=span if quiet and threaded_window and goal_window and free_window and not self.contact_latched else 0.
        score=0.
        if stable and threaded_window and not self.contact_latched:
            score=(1. if goal_window and free_window else .75 if goal_window else
                   .5 if m['progress']>=R.PARTIAL_FRACTIONS[1]-EPS else
                   .25 if m['progress']>=R.PARTIAL_FRACTIONS[0]-EPS else 0.)
        self.processed_events=len(history.events);self.processed_violations=len(history.violations);self.last_sample=int(sample)
        return dict(version=R.VERSION,score=score,complete=score==1.,valid=True,invalid_reason=None,
                    contact_latched=self.contact_latched,contact_witness=self.first_contact_witness,
                    threaded=bool(m['threaded']),at_goal=bool(at_goal),
                    stable=bool(stable),released=not robot_contact,goal_supported=supported,
                    hold_s=float(hold),stationary_window_s=span,position_envelope_m=position_envelope,
                    rotation_envelope_deg=rotation_envelope,distance_m=m['distance_m'],progress=m['progress'],
                    centre=m['centre'].tolist(),pose=m['pose'],angle_deg=m['angle_deg'],
                    alignment_deg=m['alignment_deg'],tilt_rad=m['tilt_rad'])
