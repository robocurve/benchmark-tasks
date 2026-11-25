"""Observable connector checks a person can repeat with a ruler and a luggage scale.

Every condition maps to a physical check described in task.json:
  * depth: how far the plug front sits inside the socket face. In a real trial the
    equivalent reading is the plug's rear-shell protrusion from the switch face,
    compared with the neighbouring plugs (ruler, 1 mm resolution).
  * tug: after the hands-off hold, the scorer (not the policy) pulls the orange boot
    straight back with TUG_N newtons for TUG_S seconds, as a tester would with a
    luggage scale. A latched plug does not move; an unlatched one slides out.
No hidden clip depression or hook-overlap value contributes to the score.
"""
import math
from config import *

HOME_TOLERANCE=.0015   # orange plug at most 1.5 mm shallower than the median neighbour
NEIGHBOUR_LIMIT=.002   # any neighbour pulled out by more than 2 mm is disconnected
CLEAR_M=.070           # both gripper centres at least 70 mm from the orange plug
HOLD_S=2.
TUG_N=3.
TUG_S=1.
TUG_MOVE=.002          # plug may move at most 2 mm during the tug (latch play is about 1 mm)
LINGER_S=1.            # keep recording after a full score so the final frames show it


class EthernetScorer:
 def __init__(self):
  self.value=0.;self.time=None;self.out=False;self.hold=0.;self.invalid=None
  self.tug_start=None;self.tug_depth0=None;self.tug_min=None;self.passed_tug=False;self.complete_time=None;self.tug_attempts=0

 @property
 def tug_active(self):return self.tug_start is not None and not self.passed_tug

 def update(self,s):
  dt=0. if self.time is None else s['time']-self.time;self.time=s['time']
  keys=['time','clip_travel','source_insertion','target_insertion','target_lateral_error','target_angle','height','max_neighbour_withdrawal','neighbour_median_depth']
  if not all(math.isfinite(s[k]) for k in keys):self.invalid='nonfinite state'
  if not 0<=dt<=.051:self.invalid='missing samples'
  if s['clip_travel']>CLIP_TRAVEL+.0002:self.invalid='orange tab bent past its stop (broken tab)'
  if s['height']<TOP-.02:self.invalid='orange plug dropped below the work surface'
  if s['max_neighbour_withdrawal']>NEIGHBOUR_LIMIT:self.invalid='a neighbouring cable was pulled out'
  if s['source_insertion']<-.002:self.out=True
  home=(self.out and s['target_insertion']>=s['neighbour_median_depth']-HOME_TOLERANCE
        and s['target_lateral_error']<.002 and s['target_angle']<math.radians(5))
  clear=s['hands_clear']
  if self.passed_tug:
   pass
  elif self.tug_start is not None:
   self.tug_min=min(self.tug_min,s['target_insertion'])
   if self.tug_depth0-self.tug_min>TUG_MOVE or not home:
    self.tug_start=None;self.hold=0.          # plug moved: not latched; hold must restart
   elif s['time']-self.tug_start>=TUG_S-1e-9:
    self.passed_tug=True;self.complete_time=s['time']
  else:
   self.hold=self.hold+dt if (home and clear) else 0.
   if self.hold>=HOLD_S:
    self.tug_start=s['time'];self.tug_depth0=s['target_insertion'];self.tug_min=s['target_insertion'];self.tug_attempts+=1
  if self.passed_tug and not home:self.passed_tug=False;self.complete_time=None;self.hold=0.;self.tug_start=None
  self.value=(0. if self.invalid else 1. if self.passed_tug else .75 if (self.hold>=HOLD_S or self.tug_start is not None)
              else .5 if home else .25 if self.out else 0.)
  return {'score':self.value,'invalid':self.invalid,'source_cleared':self.out,'home':home,'hands_clear':clear,
          'hands_off_hold_s':self.hold,'tug_active':self.tug_active,'tug_attempts':self.tug_attempts,
          'tug_movement_m':None if self.tug_depth0 is None else self.tug_depth0-self.tug_min,'tug_passed':self.passed_tug}

 def finished(self,time):
  return self.passed_tug and time-self.complete_time>=LINGER_S-1e-9
