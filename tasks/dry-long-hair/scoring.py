"""Measured surface dryness, fixed-head integrity and released finish."""
import math
from config import HEAD,HEAD_QUAT,MAX_TEMPERATURE,conjugate,multiply

MAX_HEAD_TRANSLATION_M=.002
MAX_HEAD_ROTATION_DEG=1.


def measured_head_motion(pose):
 """Native root XYZ/XYZW displacement from declared HEAD/HEAD_QUAT, not reset pose."""
 invalid={'head_pose_valid':False,'head_translation_m':None,'head_rotation_deg':None}
 try:
  if pose is None or len(pose)!=7 or not all(math.isfinite(v) for v in pose):return invalid
  norm=math.hypot(*pose[3:])
  if norm==0.:return invalid
  translation=math.dist(pose[:3],HEAD)
  # Compare to the declared physical rest frame, never rebase to the first
  # observation. q and -q remain equivalent under native float32 transport.
  relative=multiply(conjugate(HEAD_QUAT),tuple(v/norm for v in pose[3:]))
  rotation=math.degrees(2.*math.atan2(math.hypot(*relative[:3]),abs(relative[3])))
  if not all(math.isfinite(v) for v in (translation,rotation)):return invalid
 except (TypeError,ValueError,OverflowError):return invalid
 return {'head_pose_valid':True,'head_translation_m':translation,'head_rotation_deg':rotation}


class HairScorer:
 def __init__(self):self.value=0.;self.time=None;self.invalid=None;self.hold=0.;self.operated=False
 def update(self,s):
  dt=0. if self.time is None else s['time']-self.time;self.time=s['time']
  nums=[s['time'],s['root_separation'],s['min_height'],s['dry_fraction'],s['worst_remaining_water_fraction'],s['max_temperature_C'],s['dryer_speed']]
  if not all(math.isfinite(v) for v in nums):self.invalid='nonfinite measured state'
  if not 0<=dt<=.051:self.invalid='missing control samples'
  if s['root_separation']>.005:self.invalid='detached hair or broken lock'
  if s['min_height']<-.03:self.invalid='dropped dryer or hair'
  if s['max_temperature_C']>MAX_TEMPERATURE:self.invalid='task temperature limit exceeded'
  head=measured_head_motion(s.get('head_pose'))
  if not head['head_pose_valid']:self.invalid='missing or nonfinite measured head pose'
  elif head['head_translation_m']>MAX_HEAD_TRANSLATION_M:self.invalid='head translated beyond fixed-stand tolerance'
  elif head['head_rotation_deg']>MAX_HEAD_ROTATION_DEG:self.invalid='head rotated beyond fixed-stand tolerance'
  self.operated|=s['power_on'] and s['water_removed_step_kg']>0.
  ready=(s['worst_remaining_water_fraction']<=.10 and not s['power_on'] and s['dryer_parked'] and s['dryer_speed']<.02 and s['hands_clear'])
  self.hold=self.hold+dt if ready else 0.
  self.value=0. if self.invalid else 1. if self.hold>=1. else .75 if s['dry_fraction']>=.9 else .5 if s['dry_fraction']>=.5 else .25 if self.operated else 0.
  return {'score':self.value,'invalid':self.invalid,'release_hold_s':self.hold,**head}
