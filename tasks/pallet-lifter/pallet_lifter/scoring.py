from dataclasses import dataclass
import numpy as np
@dataclass
class History:
 tool_contact:bool=False
 lifted:bool=False
 invalid:bool=False
 open_clear:bool=False
 def contact(self,a,b):
  names=(a.lower(),b.lower())
  if not any('/pallet/' in n or n.endswith('/pallet') for n in names):return
  if any('/lifter/' in n or n.endswith('/lifter') for n in names):self.tool_contact=True
  if any(any(robot in n for robot in ('left_robot','right_robot','leftarm','rightarm')) for n in names):self.invalid=True
 def update(self,o,top):
  if self.tool_contact and o['pallet'].low[2]>top+.060:self.lifted=True
  for key in ('pallet','lifter'):
   s=o[key]
   if not np.all(np.isfinite(s.pos)) or s.low[2]<top-.02 or not -.1<s.pos[0]<.95 or abs(s.pos[1])>.62:self.invalid=True

def seated(s,top):
 q=s.quat_xyzw/np.linalg.norm(s.quat_xyzw)
 return bool(np.linalg.norm(s.pos[:2]-(.37,-.12))<.012 and abs(s.low[2]-(top+.015))<.004 and s.high[2]-s.low[2]<.042 and abs(q[3])>np.cos(np.radians(8)/2) and s.resting())
def parked(s,top):
 return bool(np.linalg.norm(s.pos[:2]-(.2575,.28))<.025 and abs(s.low[2]-(top+.035))<.006 and s.high[2]-s.low[2]<.035 and s.resting())
def evaluate(o,h,top):
 if h.invalid or not h.lifted:return 0.
 if not seated(o['pallet'],top):return .25 if o['pallet'].low[2]>top+.060 else 0.
 return 1. if parked(o['lifter'],top) and h.open_clear else .75
