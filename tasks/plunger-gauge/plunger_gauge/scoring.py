from dataclasses import dataclass,field
import numpy as np
@dataclass
class History:
 touched:set=field(default_factory=set)
 tested:set=field(default_factory=set)
 held:dict=field(default_factory=dict)
 invalid:bool=False
 open_clear:bool=False
 def contact(self,a,b):
  names=(a.lower(),b.lower())
  for i in range(2):
   key=f'plunger_{i}'
   if not any('/'+key+'/' in n or n.endswith('/'+key) for n in names):continue
   if any('/punch/' in n or n.endswith('/punch') for n in names):self.touched.add(key)
   if any(any(robot in n for robot in ('left_robot','right_robot','leftarm','rightarm')) for n in names):self.invalid=True
 def update(self,objects,top):
  for i in range(2):
   key=f'plunger_{i}';s=objects[key]
   if key in self.touched and s.pos[2]<=top+.0221:self.held[key]=self.held.get(key,0)+1
   else:self.held[key]=0
   if self.held[key]>=6:self.tested.add(key)
  s=objects['punch']
  if not np.all(np.isfinite(s.pos)) or s.low[2]<top-.02 or not -.1<s.pos[0]<.95 or abs(s.pos[1])>.62:self.invalid=True

def parked(s,top):
 return bool(np.linalg.norm(s.pos[:2]-(.20,.27))<.006 and abs(s.low[2]-(top+.010))<.003 and s.high[2]-s.low[2]<.084 and s.resting())
def evaluate(objects,h,top):
 if h.invalid:return 0.
 count=sum(f'plunger_{i}' in h.tested and abs(objects[f'plunger_{i}'].pos[2]-(top+.0301))<.0012 and objects[f'plunger_{i}'].resting() for i in range(2))
 return .4*count+(.2 if count==2 and parked(objects['punch'],top) and h.open_clear else 0.)
