"""Native, self-colliding PhysX cord; measured topology and released straightness."""
import json
from pathlib import Path
import numpy as np
from topology import analyze_open_rope
class Model:
 def __init__(self):
  self.initial=np.array(json.loads(Path(__file__).with_name('initial-nodes.json').read_text()));self.points=self.initial.copy();self.events=[];self.time=0.;self.release=0.;self.failed=False;self.initial_valid=False;self.gap=0.;self.record={};self.kind='unmeasured';self.base=None
 def bind_native(self,base):
  self.base=base;self.read();r=analyze_open_rope(self.points,.010);self.initial_valid=r['status']=='overhand_candidate';self.kind=r['status'];self.events.append(dict(event='settled_initial_topology',status=self.kind,valid=self.initial_valid))
 def read(self):
  from rope import rope_state
  s=rope_state(self.base,'cord');self.record=s.to_record();self.points=s.polyline();self.gap=s.max_joint_gap
 def update(self,hands,dt=.05):
  if self.base is None:raise RuntimeError('This cord requires native PhysX, not the CPU preview')
  self.time+=dt;self.read();self.failed=self.failed or self.gap>.005 or not np.isfinite(self.points).all()
  if round(self.time*20)%20==0:
   r=analyze_open_rope(self.points,.010);self.kind=r['status']
  clear=all(h['opening']>.6 and np.min(np.linalg.norm(self.points-h['tcp'],axis=1))>.05 for h in hands.values())
  self.release=self.release+dt if clear else 0.
 def crossings(self):
  p=self.points[:,:2];a=p[:-1];u=np.diff(p,axis=0);i,j=np.triu_indices(len(u),k=3);v=u[j];w=a[j]-a[i];x=u[i];den=x[:,0]*v[:,1]-x[:,1]*v[:,0];good=abs(den)>1e-10
  t=np.divide(w[:,0]*v[:,1]-w[:,1]*v[:,0],den,out=np.zeros(len(den)),where=good);q=np.divide(w[:,0]*x[:,1]-w[:,1]*x[:,0],den,out=np.zeros(len(den)),where=good)
  return int(np.sum(good&(t>0)&(t<1)&(q>0)&(q<1)))
 def measure(self):return dict(top_view_crossings=self.crossings(),initial_overhand=self.initial_valid,topology=self.kind,end_separation_m=float(np.linalg.norm(self.points[-1]-self.points[0])),table_clearance_m=float(np.max(self.points[:,2]-.7475-.005)),released_seconds=self.release,max_joint_gap_mm=self.gap*1000,failed=self.failed)
 def score(self):
  m=self.measure()
  if self.failed or not self.initial_valid:return 0.
  # Whole cord above the 1.10 x 1.30 m station tabletop, allowing 2 mm contact tolerance.
  on_table=np.all(self.points>=[-.185,-.645,.7505]) and np.all(self.points[:,:2]<=[.905,.645])
  if self.kind=='unknot' and m['top_view_crossings']==0 and self.release>=3 and m['table_clearance_m']<.025 and on_table:return 1.
  return .5 if self.kind=='unknot' and self.release>=3 else 0.
 def state(self):return self.record
 def draw(self,d):pass
