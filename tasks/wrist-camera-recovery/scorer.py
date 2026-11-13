"""Observable progress; simulator sensor-pose consistency is not a score condition."""
from dataclasses import dataclass
import math
@dataclass
class RecoveryScorer:
 lift_time:float=0.
 alignment_time:float=0.
 release_time:float=0.
 ever_lifted:bool=False
 invalid:bool=False
 def update(self,*,lifted,held,latched,position_error_m,rotation_error_rad,other_hand_clear,live_image,damaged=False,off_table=False,dt=.05):
  self.invalid |= damaged or off_table
  aligned=position_error_m<=.005 and rotation_error_rad<=math.radians(5)
  self.lift_time=self.lift_time+dt if lifted and held else 0.
  self.ever_lifted |= self.lift_time>=1.-1e-8
  self.alignment_time=self.alignment_time+dt if aligned and (held or latched) else 0.
  self.release_time=self.release_time+dt if latched and aligned and other_hand_clear and live_image else 0.
  score=0.25 if self.ever_lifted else 0.
  if self.alignment_time>=1.-1e-8:score=max(score,.5)
  if latched and aligned:score=.75
  if self.release_time>=2.-1e-8:score=1.
  if self.invalid:score=0.
  return dict(score=score,aligned=aligned,latched=latched,live_image=live_image,lift_time=self.lift_time,alignment_time=self.alignment_time,release_time=self.release_time,invalid=self.invalid)
