"""Native loose-leaf transport; scoring does not collect or move leaves."""
import numpy as np
from native_model import NativeModel
from geometry import *
class Model(NativeModel):
 def __init__(self):super().__init__();self.spilled=False
 def configure_station(self):pass
 def reset_valid(self):return self.measure()['leaves_in_bucket']==0 and not self.failed
 def material(self,hands,dt):
  # A leaf lost off the work surface is visible failure, not successful disposal.
  self.spilled |= any(self.p(n)[2]<TABLE-.04 for row in LEAVES for n in row)
 def measure(self):
  complete=0
  for row in LEAVES:
   points=[]
   for j,n in enumerate(row):
    half=[.013,.026,.026,.013][j]/2
    for x in [-half,half]:
     for y in [-.0075,.0075]:points.append(self.p(n,[x,y,0]))
   q=(np.array(points)-self.p('bucket'))@self.orientation('bucket')
   complete+=bool(np.all(np.linalg.norm(q[:,:2],axis=1)<BUCKET_R-.004) and np.all((q[:,2]>-.074)&(q[:,2]<.075)))
  upright=bool(self.orientation('bucket')[2,2]>.95 and np.linalg.norm(self.p('bucket')[:2]-BUCKET[:2])<.04)
  return dict(leaves_in_bucket=complete,total_leaves=len(LEAVES),bucket_upright=upright,spilled_off_surface=self.spilled,released_seconds=self.release,failed=bool(self.failed or self.broken))
 def score(self):
  q=self.measure()
  if self.failed or self.broken or self.spilled or not q['bucket_upright'] or self.release<1.:return 0.
  return q['leaves_in_bucket']/len(LEAVES)
 def state(self):return dict(super().state(),spilled=self.spilled)
