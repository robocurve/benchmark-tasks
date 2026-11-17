"""Cutting resistance is applied to native blades, with equal/opposite reactions.

Only blade–cap collision is replaced: the analytic material contact permits a
sharp edge to traverse the cap under load. The rest of the cutter and cigar
retain native collisions. A light touch or merely closing empty handles cannot
cut. No body pose is written by the material model.
"""
import numpy as np
from native_model import NativeModel
from geometry import RADIUS,TABLE
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.cut=False;self.work=np.zeros(2);self.depth=np.zeros(2);self.loaded=np.zeros(2,dtype=bool);self.force_apis={};self.crush=0.;self.cut_depth=None
 def bind_native(self,base):
  super().bind_native(base)
  from pxr import PhysxSchema,Gf
  for name in ['cap','blade_left','blade_right']:
   paths=[p for p in self.stage.Traverse() if p.GetName()==name and '/materials/' in str(p.GetPath())];assert len(paths)==1
   self.force_apis[name]=PhysxSchema.PhysxForceAPI(paths[0])
 def material(self,hands,dt):
  from pxr import Gf
  cutterR=self.orientation('cutter');local=cutterR.T@(self.p('cap')-self.p('cutter'));capaxis=self.orientation('cap')[:,2]
  alignment=abs(float(capaxis@cutterR[:,2]));valid=alignment>np.cos(np.radians(8)) and abs(local[2])<.0007 and abs(local[1])<.004
  forces={n:np.zeros(3) for n in self.force_apis};torques={n:np.zeros(3) for n in self.force_apis}
  for i,(side,name) in enumerate([(-1,'blade_left'),(1,'blade_right')]):
   slider=cutterR.T@(self.p(name)-self.p('cutter'))
   # Inner cutting edge starts at ±12 mm and crosses the cap's diameter.
   edge=slider[0]-side*.022;penetration=np.clip(RADIUS-side*(edge-local[0]),0,2*RADIUS)
   delta=max(0.,penetration-self.depth[i]);inside=valid and 0<penetration<2*RADIUS and not self.cut
   if inside:
    # 10 N per blade is an uncalibrated engineering approximation. Motion is
    # resisted in PhysX and reacts on the cigar, rather than just recorded.
    force=cutterR[:,0]*side*10.;point=self.p('cutter')+cutterR@np.array([edge,local[1],0.])
    forces[name]+=force;forces['cap']-=force
    torques[name]+=np.cross(point-self.p(name),force);torques['cap']+=np.cross(point-self.p('cap'),-force)
    if self.loaded[i] and delta<=.003:self.work[i]+=10.*delta
   self.loaded[i]=inside;self.depth[i]=penetration
  # Both real blades must traverse nearly to the axis while doing loaded work.
  if valid and min(self.depth)>=RADIUS-.0005 and min(self.work)>=.060 and not self.cut:
   self.disable('cap_bond','cap_severed_after_loaded_blade_travel');self.cut=True;self.cut_depth=float(local[2])
  for name,api in self.force_apis.items():
   api.GetForceAttr().Set(Gf.Vec3f(*forces[name]));api.GetTorqueAttr().Set(Gf.Vec3f(*torques[name]))
  for i,t in enumerate(np.arange(4)*np.pi/2):
   radial=self.orientation('cigar')@np.array([np.cos(t),np.sin(t),0]);self.crush=max(self.crush,max(0.,-float(self.joint_gap('wrapper_spring_'+str(i))@radial)))
  self.failed |= self.crush>.001 or bool(self.broken)
 def measure(self):
  cigar=self.p('cigar');ends=np.array([self.p('cigar',[0,0,z]) for z in [-.067,.067]])
  on_surface=bool(np.all((ends[:,0]>.01)&(ends[:,0]<.80)&(abs(ends[:,1])<.50)) and TABLE+.004<cigar[2]<TABLE+.045)
  return dict(cigar_on_work_surface=on_surface,cap_cut=self.cut,cap_separate=bool(np.linalg.norm(self.p('cap')-self.p('cigar',[0,0,.068]))>.020),wrapper_intact=not self.failed,wrapper_compression_mm=self.crush*1000,cutting_work_J=self.work.tolist(),released_seconds=self.release,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed() or (self.crush>.001 or bool(self.broken))
 def score(self):
  self.failed |= self.integrity_failed()
  m=self.measure()
  if self.failed:return 0.
  if self.cut and m['cap_separate'] and m['cigar_on_work_surface'] and self.release>=3:return 1.
  return .5 if self.cut and m['cap_separate'] else 0.
 def state(self):return dict(**super().state(),cut=self.cut,work=self.work,depth=self.depth,loaded=self.loaded,crush=self.crush,cut_depth=self.cut_depth)
