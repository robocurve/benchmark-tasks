"""Loaded carving contact; no driven chips, knife poses or stick poses."""
import numpy as np
from native_model import NativeModel
from geometry import *
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.work=np.zeros(5);self.progress=np.zeros(5);self.previous=np.zeros(5);self.loaded=np.zeros(5,dtype=bool);self.split=False;self.force_apis={}
 def configure_station(self):pass
 def reset_valid(self):return not self.disabled and not self.failed
 def bind_native(self,base):
  super().bind_native(base)
  from pxr import PhysxSchema
  for n in ['stick','edge']:
   matches=[p for p in self.stage.Traverse() if p.GetName()==n and '/materials/' in str(p.GetPath())];assert len(matches)==1
   self.force_apis[n]=PhysxSchema.PhysxForceAPI(matches[0])
 def material(self,hands,dt):
  from pxr import Gf
  R=self.orientation('stick');e=R.T@(self.p('edge')-self.p('stick'));er=R.T@self.orientation('edge');force=np.zeros(3)
  for i in range(5):
   F=np.eye(3);p=e;edgeaxis=er[:,0];travel=-p[1]-.045;surface=.006-(i+1)*2./45*travel
   if any('grain_'+str(j) not in self.disabled for j in range(i)):continue
   if 'grain_'+str(i) in self.disabled:continue
   aligned=abs(edgeaxis[0])>np.cos(np.radians(12)) and abs(p[0])<.004
   cutting=aligned and 0<travel<.046 and surface-.0008<p[2]<surface+.0008 and 'grain_'+str(i) not in self.disabled
   delta=travel-self.previous[i]
   if cutting:
    # Uncalibrated engineering approximation: 4 N cutting resistance.
    # Applies equal and opposite native forces, so speed alone cannot sever it.
    f=R@F@np.array([0,4.,.45]);force+=f
    if self.loaded[i] and 0<delta<.004:
     self.work[i]+=4.*delta;self.progress[i]+=delta
   if aligned and 0<travel<.046 and p[2]<surface-.002:self.split=True
   if cutting and self.progress[i]>.040 and self.work[i]>.16 and travel>.042:
    self.disable('grain_'+str(i),'chip_severed_after_loaded_cut')
    # Once severed, the real fragment must collide with the blade again.
    from pxr import UsdPhysics,Sdf
    edge=next(p for p in self.stage.Traverse() if p.GetName()=='edge' and '/materials/' in str(p.GetPath()))
    chip_path=edge.GetPath().GetParentPath().AppendChild('chip_'+str(i))
    UsdPhysics.FilteredPairsAPI(edge).GetFilteredPairsRel().RemoveTarget(chip_path)
   self.previous[i]=travel;self.loaded[i]=cutting
  for n,sgn in [('edge',1),('stick',-1)]:
   api=self.force_apis[n];api.GetForceAttr().Set(Gf.Vec3f(*(sgn*force)));api.GetTorqueAttr().Set(Gf.Vec3f(*np.cross(self.p('edge')-self.p(n),sgn*force)))
  self.failed |= bool(self.broken) or self.split
 def measure(self):
  cut=[i for i in range(5) if 'grain_'+str(i) in self.disabled and np.linalg.norm(self.p('chip_'+str(i))-self.p('stick'))>.01]
  on_tray=bool(abs(self.p('stick')[0]-TRAY[0])<.14 and abs(self.p('stick')[1])<.18 and TABLE+.009<self.p('stick')[2]<TABLE+.035)
  knife_down=bool(abs(self.p('knife')[0]-TRAY[0])<.14 and abs(self.p('knife')[1])<.18 and TABLE+.006<self.p('knife')[2]<TABLE+.04)
  return dict(shavings_removed=len(cut),tip_thickness_mm=12.-2.*len(cut),taper_length_mm=45.,stick_intact=not self.failed,stick_on_tray=on_tray,knife_on_tray=knife_down,released_seconds=self.release,cutting_work_J=self.work.tolist(),failed=self.failed)
 def score(self):
  m=self.measure()
  if self.failed:return 0.
  if m['shavings_removed']==5:return 1. if m['stick_on_tray'] and m['knife_on_tray'] and self.release>=2. else .75
  return .5 if m['shavings_removed']>=3 else .25 if m['shavings_removed']>=1 else 0.
 def state(self):return dict(super().state(),work=self.work,progress=self.progress,previous=self.previous,loaded=self.loaded,split=self.split)
