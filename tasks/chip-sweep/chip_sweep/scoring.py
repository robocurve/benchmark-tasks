"""Numeric end-state score and measured contact provenance."""
from dataclasses import dataclass,field
import numpy as np
@dataclass
class History:
 touched:set=field(default_factory=set)
 chip_edges:set=field(default_factory=set)
 invalid:bool=False
 open_clear:bool=False
 def contact(self,a,b):
  names=(a.lower(),b.lower())
  chips={f'chip_{i}' for i in range(5) if any(f'/chip_{i}/' in n or n.endswith(f'/chip_{i}') for n in names)}
  if not chips:return
  if len(chips)==2:self.chip_edges.add(tuple(sorted(chips)))
  if any('/rake/' in n or n.endswith('/rake') for n in names):self.touched.update(chips)
  if any(any(robot in n for robot in ('left_robot','right_robot','leftarm','rightarm')) for n in names):self.invalid=True
  # Physical chip clusters inherit provenance through measured contact edges.
  changed=True
  while changed:
   before=len(self.touched)
   for edge in self.chip_edges:
    if self.touched.intersection(edge):self.touched.update(edge)
   changed=len(self.touched)>before
 def update(self,objects,top):
  for k in ['rake']+[f'chip_{i}' for i in range(5)]:
   s=objects[k]
   if not np.all(np.isfinite(s.pos)) or s.low[2]<top-.02 or not -.1<s.pos[0]<.95 or abs(s.pos[1])>.62:self.invalid=True

def collected(s,top):
 return bool(.185<s.low[0] and s.high[0]<.305 and .045<s.low[1] and s.high[1]<.215 and abs(s.low[2]-(top+.012))<.003 and s.resting())
def parked(s,top):
 return bool(np.linalg.norm(s.pos[:2]-(.22875,.27))<.025 and abs(s.low[2]-(top+.028))<.006 and s.high[2]-s.low[2]<.105 and s.resting())
def evaluate(objects,h,top):
 if h.invalid:return 0.
 count=sum(f'chip_{i}' in h.touched and collected(objects[f'chip_{i}'],top) for i in range(5))
 return count*.16+(.2 if count==5 and parked(objects['rake'],top) and h.open_clear else 0.)
