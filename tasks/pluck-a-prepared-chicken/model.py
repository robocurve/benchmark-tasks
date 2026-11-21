import numpy as np
from native_model import NativeModel
from geometry import BODIES,JOINTS,C,N,SKIN,AXIS,TABLE
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.extracted=set();self.peak_skin_gap=0.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  from finite_pinch import FinitePinch
  self.pinches=[FinitePinch(self,[f'feather_{i}'],.0006,.002,.009,(-.010,.004)) for i in range(N)]
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;held={}
  for pinch in self.pinches:
   pinch.update(hands)
   for side,g in self.grasp_state.items():held.setdefault(side,[]).append(g)
  self.grasp_state=held
  for i in range(N):
   name=f'follicle_{i}';j=next(j for j in JOINTS if j['name']==name)
   axis=self.orientation(j['a'])@AXIS
   if i not in self.extracted and np.dot(self.joint_gap(name),axis)>.0065:
    self.disable(name,'calamus_fully_extracted');self.extracted.add(i);self.restore_extracted_contacts(i)
  for name in SKIN:
   joint='skin_bond_'+name;gap=float(np.linalg.norm(self.joint_gap(joint)));self.peak_skin_gap=max(self.peak_skin_gap,gap)
   if gap>=.004:self.disable(joint,'skin_torn')
  dynamic=[i for i,b in enumerate(BODIES) if not b.get('fixed')]
  clear=all(h['opening']>.4 and np.min(np.linalg.norm(self.poses[dynamic,:3]-h['tcp'],axis=1))>.05 for h in hands.values())
  self.release=self.release+dt if clear else 0.;self.failed |= self.integrity_failed()
 def restore_extracted_contacts(self,i):
  from pxr import UsdPhysics
  feathers={f'quill_{i}',f'feather_{i}'};skin=set(SKIN+['carcass'])
  for p in self.stage.Traverse():
   if '/materials/' not in str(p.GetPath()) or not p.HasAPI(UsdPhysics.FilteredPairsAPI):continue
   source=p.GetName();rel=UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel()
   for target in rel.GetTargets():
    other=str(target).rsplit('/',1)[-1]
    if (source in feathers and other in skin) or (source in skin and other in feathers):rel.RemoveTarget(target)
 def integrity_failed(self):
  return super().integrity_failed() or bool(self.broken) or self.peak_skin_gap>=.004
 def reset_valid(self):return not self.extracted and not self.failed
 def within_tray(self,name):
  # Whole horizontal geometry, with a 1 mm tolerance at the outer tray rim.
  # Carcass height/upright checks are in measure(); mesh-floor penetration is
  # a contact-solver detail, not an out-of-tray outcome. Feather floor checks remain.
  b=BODIES[self.index[name]];points=[];physical=[]
  for shape in b['shapes']:
   if shape['kind']=='mesh':v=np.asarray(shape['points'])+shape['pos']
   elif shape['kind']=='cylinder':
    r=shape['radius'];h=shape['length']/2
    v=np.array([[x,y,z] for x in [-r,r] for y in [-r,r] for z in [-h,h]])+shape['pos']
   else:continue
   w=[self.p(name,p) for p in v];points.extend(w)
   if shape.get('collision',True):physical.extend(w)
  p=np.asarray(points);c=np.asarray(physical)
  return bool(len(p) and np.all(abs(p[:,0]-.36)<=.211) and np.all(abs(p[:,1])<=.191) and len(c) and (name=='carcass' or np.all(c[:,2]>=TABLE+.004)))
 def measure(self):
  carcass=self.p('carcass');on_tray=bool(self.within_tray('carcass') and TABLE+.050<carcass[2]<TABLE+.090 and self.orientation('carcass')[2,2]>.85)
  contained=all(self.within_tray(name) for i in self.extracted for name in [f'feather_{i}',f'quill_{i}'])
  return dict(extracted_feathers_in_tray=bool(contained),whole_feathers_extracted=len(self.extracted),total_feathers=N,skin_intact=not self.failed,maximum_skin_displacement_mm=self.peak_skin_gap*1000,carcass_in_tray=on_tray,released_seconds=self.release,failed=self.failed)
 def score(self):
  if self.failed or self.release<3 or not self.measure()['carcass_in_tray'] or not self.measure()['extracted_feathers_in_tray']:return 0.
  return len(self.extracted)/N
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,extracted=sorted(self.extracted),peak_skin_gap=self.peak_skin_gap)
 def restore(self,state):
  state=dict(state);self.extracted=set(state.pop('extracted'));super().restore(state)
