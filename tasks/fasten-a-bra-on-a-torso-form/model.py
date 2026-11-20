import numpy as np
from native_model import NativeModel
from geometry import BODIES,JOINTS,X,Z,TABLE,BAND_ANCHORS,BAND_REST
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.retained=0.;self.retained_any=0.;self.max_extension=0.;self.max_strain=1.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,['left_tab','right_tab'],.0003,.001,.003,(-.006,.004),points={'left_tab':[0,.010,0],'right_tab':[0,-.010,0]})
 def hooks(self):
  # Eye crossbars must pass through each actual ungated hook pocket.
  left=self.orientation('left_tab');right=self.orientation('right_tab');count=0
  aligned=np.dot(left[:,2],right[:,2])>.94 and np.dot(left[:,0],right[:,0])>.94
  for z in [-.008,.008]:
   eye=self.p('right_tab',[-.002,.013,z]);q=(eye-self.p('left_tab'))@left
   count+=bool(aligned and .0005<q[0]<.0035 and -.0116<q[1]<-.0084 and abs(q[2]-z)<.0020)
  return count
 def untwisted(self):
  return all(self.orientation(n)[2,2]>0. for n in self.names if "_wing_" in n or n.endswith("_tab"))
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.pinch.update(hands)
  lengths={side:np.linalg.norm(self.p(side+'_tab',[0,sign*.015,0])-BAND_ANCHORS[side]) for side,sign in [('left',1),('right',-1)]}
  self.max_extension=max(self.max_extension,max(lengths[side]-BAND_REST[side] for side in lengths))
  self.max_strain=max(self.max_strain,max(lengths[side]/BAND_REST[side] for side in lengths))
  self.failed |= self.integrity_failed()
  clear=all(h['opening']>.4 and min(np.linalg.norm(h['tcp']-self.p(n)) for n in ['left_tab','right_tab'])>.055 for h in hands.values())
  self.release=self.release+dt if clear else 0.
  self.retained=self.retained+dt if clear and self.hooks()==2 and self.untwisted() and not self.failed else 0.
  self.retained_any=self.retained_any+dt if clear and self.hooks()>=1 and self.untwisted() and not self.failed else 0.
 def integrity_failed(self):return super().integrity_failed() or bool(self.broken) or self.max_strain>1.6
 def reset_valid(self):return self.hooks()==0 and not self.failed
 def measure(self):return dict(engaged_hooks=self.hooks(),total_hooks=2,released_seconds=self.release,retained_seconds=self.retained,one_or_more_retained_seconds=self.retained_any,band_untwisted=self.untwisted(),maximum_local_extension_mm=1000*self.max_extension,maximum_band_length_ratio=self.max_strain,intact=not self.failed,failed=self.failed)
 def score(self):
  if self.failed or self.release<3 or not self.untwisted():return 0.
  return 1. if self.retained>=3 else .5 if self.retained_any>=3 else 0.
 def draw(self,d):
  from geometry import WIDTH
  for side,sign in [('left',1),('right',-1)]:
   start=BAND_ANCHORS[side];end=self.p(side+'_tab',[0,sign*.015,0]);c1=start+[-.015,-sign*.010,0];c2=end+[0,sign*.015,0]
   ts=np.linspace(0,1,12);centres=np.array([(1-t)**3*start+3*(1-t)**2*t*c1+3*(1-t)*t*t*c2+t**3*end for t in ts])
   width=np.array([(1-t)*np.array([0,0,1.])+t*self.orientation(side+'_tab')[:,2] for t in ts]);width/=np.maximum(np.linalg.norm(width,axis=1)[:,None],1e-9)
   vertices=np.stack([centres-width*WIDTH/2,centres+width*WIDTH/2],axis=1).reshape(-1,3)
   faces=[(2*i,2*i+2,2*i+3,2*i+1) for i in range(len(ts)-1)]
   d.mesh(side+'_elastic_ribbon',vertices,faces,(.20,.28,.45))
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,retained=self.retained,retained_any=self.retained_any,max_extension=self.max_extension,max_strain=self.max_strain)
