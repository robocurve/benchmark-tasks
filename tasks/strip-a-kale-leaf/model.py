import numpy as np
from native_model import NativeModel
from geometry import JOINTS,TABLE
class Model(NativeModel):
 def material(self,hands,dt):
  self.failed |= any(n.startswith('stem_') for n in self.broken)
 def measure(self):
  attachments=[j for j in JOINTS if j['name'].startswith('attach_')]
  leaf_names=[n for n in self.names if n.startswith('leaf_')];stalk=np.array([self.p(n) for n in self.names if n.startswith('stalk_')])
  detached=[j for j in attachments if j['name'] in self.broken]
  # Area weighting prevents a narrow tip counting as much as broad leaf tissue.
  areas={b['name']:b['shapes'][0]['size'][0]*b['shapes'][0]['size'][1] for b in __import__('geometry').BODIES if b['name'] in leaf_names}
  total=sum(areas.values());recovered=sum(areas[j['b']] for j in detached if abs(self.p(j['b'])[0]-.34)<.16 and abs(self.p(j['b'])[1])<.17 and TABLE+.015<self.p(j['b'])[2]<TABLE+.06)
  detached_area=sum(areas[j['b']] for j in detached)
  distance=min(np.min(np.linalg.norm(stalk-self.p(n),axis=1)) for n in leaf_names)
  return dict(detached_fraction=detached_area/total,leaf_on_board_fraction=recovered/total,stalk_separation_mm=float(distance*1000),stalk_intact=not self.failed,released_seconds=self.release,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed() or (any(n.startswith('stem_') for n in self.broken))
 def score(self):
  self.failed |= self.integrity_failed()
  m=self.measure()
  if self.failed:return 0.
  f=m['leaf_on_board_fraction']
  if f>=.90 and m['stalk_separation_mm']>=20 and self.release>=3:return 1.
  return .5 if f>=.75 else .25 if f>=.5 else 0.
