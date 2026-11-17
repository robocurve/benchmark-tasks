import numpy as np
from native_model import NativeModel
from geometry import TABLE
class Model(NativeModel):
 def material(self,hands,dt):
  self.failed |= any(n.startswith('flesh_bond_') for n in self.broken)
 def measure(self):
  p=self.p('stone');d=self.p('dish');R=self.orientation('dish');local=R.T@(p-d)
  fruit=self.p('fruit');on_board=abs(fruit[0]-.34)<.13 and abs(fruit[1])<.10 and TABLE+.019<fruit[2]<TABLE+.06
  return dict(fruit_on_board=bool(on_board),stone_removed='stone_bond' in self.broken,separation_mm=float(np.linalg.norm(p-self.p('fruit'))*1000),stone_in_dish=bool(np.linalg.norm(local[:2])<.032 and .015<local[2]<.038),flesh_intact=not self.failed,released_seconds=self.release,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed() or (any(n.startswith('flesh_bond_') for n in self.broken))
 def score(self):
  self.failed |= self.integrity_failed()
  m=self.measure()
  if self.failed or not m['stone_removed']:return 0.
  if m['stone_in_dish'] and m['fruit_on_board'] and self.release>=3:return 1.
  return .5 if m['separation_mm']>=45 else 0.
