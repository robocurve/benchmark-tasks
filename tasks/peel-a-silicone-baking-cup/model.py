import numpy as np
from native_model import NativeModel
from geometry import TABLE,N
class Model(NativeModel):
 def material(self,hands,dt):
  self.failed |= any(n.startswith(('base_','wall_','ring_','tab_seam_','crumb_bond_')) for n in self.broken)
 def measure(self):
  detached=sum('adhesion_'+str(i) in self.broken for i in range(N));muffin=self.p('muffin');cup=self.p('cup_bottom')
  # No success on a nominal bond event while the muffin remains inside the cup.
  clear=np.linalg.norm(muffin-cup)>.075
  on_tray=all(abs(p[0]-.34)<.14 and abs(p[1])<.12 and TABLE+.003<p[2]<TABLE+.085 for p in [muffin,cup])
  return dict(peeled_fraction=detached/N,bottom_released='bottom_adhesion' in self.broken,muffin_clear=bool(clear),both_on_tray=bool(on_tray),cup_intact=not self.failed,released_seconds=self.release,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed() or (any(n.startswith(('base_','wall_','ring_','tab_seam_','crumb_bond_')) for n in self.broken))
 def score(self):
  self.failed |= self.integrity_failed()
  m=self.measure()
  if self.failed:return 0.
  if m['peeled_fraction']>=.95 and m['bottom_released'] and m['muffin_clear'] and m['both_on_tray'] and self.release>=3:return 1.
  return .5 if m['peeled_fraction']>=.75 else .25 if m['peeled_fraction']>=.5 else 0.
