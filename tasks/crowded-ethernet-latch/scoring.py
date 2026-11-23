"""Measured native connector transitions and released latch retention.

Probe forces are never allowed to call this scorer for a scored episode.
"""
import math
from config import *
class EthernetScorer:
 def __init__(self):self.value=0.;self.time=None;self.released=False;self.disconnected=False;self.hold=0.;self.invalid=None
 def update(self,s):
  dt=0. if self.time is None else s['time']-self.time;self.time=s['time']
  if not all(math.isfinite(v) for v in [s['time'],s['clip_travel'],s['source_insertion'],s['target_insertion'],s['target_lateral_error'],s['target_angle'],s['speed'],s['height'],s['max_neighbour_withdrawal']]):self.invalid='nonfinite state'
  if not 0<=dt<=.051:self.invalid='missing samples'
  if s['clip_travel']>CLIP_TRAVEL+.0002:self.invalid='clip constraint damaged'
  if s['height']<TOP-.02:self.invalid='plug dropped below work surface'
  if s['max_neighbour_withdrawal']>.008:self.invalid='neighbour connector disconnected'
  if s['source_insertion']>.006 and s['clip_travel']>=RELEASE_TRAVEL:self.released=True
  if s['source_insertion']<-.002:
   if not self.released:self.invalid='withdrawal without first depressing clip'
   else:self.disconnected=True
  seated=(self.disconnected and s['target_insertion']>.008 and s['target_lateral_error']<.001 and s['target_angle']<math.radians(5) and s['clip_travel']<.0003 and s['hook_retained'])
  ready=seated and s['hands_clear'] and s['speed']<.01
  self.hold=self.hold+dt if ready else 0.
  self.value=0. if self.invalid else 1. if self.hold>=2. else .75 if self.hold>=1. else .5 if seated else .25 if self.disconnected else 0.
  return {'score':self.value,'invalid':self.invalid,'released_before_pull':self.released,'disconnected':self.disconnected,'retention_hold_s':self.hold}
