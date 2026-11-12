"""One bounded unload/recenter retry for reported barrel/backing contact stalls.

Only controller targets change. There is no pose projection or fit relaxation.
"""
from collections import deque
import math
import numpy as np
from pen_capping.retention import engagement,BARREL_START,MOUTH_X

WITHDRAW_X=-.100


class PrecontactBias:
    """Restore only compensation learned before the first reported interval.

    Delivery may lag an interval: keep bounded history and exclude its command
    as well as later commands. This is controller memory, not force estimation.
    """
    def __init__(self):
        self.history=deque(maxlen=256);self.seen=False
        self.saved=np.zeros(3);self.saved_time=None;self.first_interval=None

    def observe(self,now,bias,axis,report):
        if self.seen:return
        if report is not None:
            self.seen=True;self.first_interval=report.get('first_report_interval_start_s')
            if self.first_interval is not None:
                candidates=[row for row in self.history if row[0]<self.first_interval-1e-9]
                if candidates:self.saved_time,self.saved=candidates[-1][0],candidates[-1][1].copy()
            return
        axis=np.asarray(axis,dtype=float);axis=axis/np.linalg.norm(axis)
        bias=np.asarray(bias,dtype=float)
        self.history.append((float(now),bias-axis*float(bias@axis)))

    def restore(self,axis):
        axis=np.asarray(axis,dtype=float);axis=axis/np.linalg.norm(axis)
        return self.saved-axis*float(self.saved@axis)


def barrel_mouth_gap(relative,rotation):
    """Conservative axial gap from barrel front to outer cap mouth rim."""
    cosine=float(np.clip(rotation[0,0],-1.,1.))
    mouth_front=relative[0]+MOUTH_X*cosine+.0164*math.sqrt(max(0.,1.-cosine*cosine))
    return BARREL_START-mouth_front


class ContactRecovery:
    def __init__(self):
        self.state='normal';self.retries=0;self.entered=0.;self.samples=deque()
        self.withdraw_lateral=None;self.reference=WITHDRAW_X;self.centered=0;self.reason=None

    def _stall(self,now,x,requested,contact,eligible):
        self.samples.append((now,x))
        while len(self.samples)>1 and now-self.samples[1][0]>=.5-1e-9:self.samples.popleft()
        span=now-self.samples[0][0];progress=x-self.samples[0][1]
        return (contact and not eligible and x>BARREL_START-MOUTH_X-.001
                and requested-x>.001 and span>=.5-1e-9 and progress<.0003)

    def _enter(self,state,now,reason):
        self.state=state;self.entered=now;self.reason=reason;self.samples.clear()

    def step(self,now,relative,rotation,requested,contact,dt=.05):
        relative=np.asarray(relative,dtype=float);rotation=np.asarray(rotation,dtype=float)
        metrics=engagement(relative,rotation);clear_bias=False;transition=None
        report=contact or {};reported=bool(report.get('loaded') or report.get('recent_loaded')) if 'loaded' in report else bool(report.get('active') or report.get('recent'))
        if self.state in ('normal','advance') and self._stall(now,float(relative[0]),requested,reported,metrics['eligible']):
            if self.retries>=1:
                self._enter('failed',now,'backing_contact_stall_after_one_retry');transition=self.state
            else:
                self.retries+=1;self.withdraw_lateral=relative[1:].copy();self.centered=0
                self._enter('withdraw',now,'reported_backing_contact_and_less_than_0.3mm_progress_over_0.5s_with_more_than_1mm_command_gap')
                clear_bias=True;transition=self.state
        if self.state=='withdraw':
            if now-self.entered>2.5:
                self._enter('failed',now,'withdraw_timeout');transition=self.state
            elif abs(relative[0]-WITHDRAW_X)<.0015 and barrel_mouth_gap(relative,rotation)>=.010 and not reported:
                self.reference=WITHDRAW_X;self._enter('center',now,'barrel_clear_and_contact_reports_clear');transition=self.state
        if self.state=='center':
            prospective=engagement(np.array([requested,relative[1],relative[2]]),rotation)
            ready=(barrel_mouth_gap(relative,rotation)>=.010 and not reported
                   and prospective.get('eccentricity_m') is not None
                   and prospective['eccentricity_m']<=.00025 and prospective.get('axis_angle_deg',180)<=.5)
            self.centered=self.centered+1 if ready else 0
            if self.centered>=3 and now-self.entered<=5.:
                self.reference=float(relative[0]);self._enter('advance',now,'three_clear_centered_observations');transition=self.state
            elif now-self.entered>5.:
                self._enter('failed',now,'clear_centering_timeout');transition=self.state
        if self.state=='advance':
            if now-self.entered>8.:
                self._enter('failed',now,'reinsert_timeout');transition=self.state
            elif math.hypot(relative[1],relative[2])<.0005 and metrics.get('axis_angle_deg',180)<.75:
                self.reference=min(requested,self.reference+.003*dt)
        depth=None;lateral=np.zeros(2)
        if self.state=='withdraw':depth=WITHDRAW_X;lateral=self.withdraw_lateral.copy()
        elif self.state=='center':depth=WITHDRAW_X
        elif self.state=='advance':depth=self.reference
        return dict(state=self.state,retries=self.retries,reason=self.reason,transition=transition,
                    depth=depth,lateral=lateral.tolist(),clear_bias=clear_bias,
                    failed=self.state=='failed',centered_steps=self.centered,
                    barrel_mouth_gap_m=float(barrel_mouth_gap(relative,rotation)),reported_backing=reported)
