"""Loaded directional scraping removes scale groups; native skin compression can fail."""
import numpy as np
from native_model import NativeModel
from geometry import *
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.work=np.zeros(len(PATCHES));self.removed=set();self.peak_compression=0.;self.previous_tool=None;self.maximum_load=0.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  from finite_pinch import FinitePinch
  self.tail_pinch=FinitePinch(self,['fish'],.008,.001,.003,(-.006,.004),points={'fish':[0,.103,0]});self.tool_pinch=FinitePinch(self,['scaler'],.006,.001,.003,(-.006,.004))
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.tail_pinch.update(hands);self.tool_pinch.update(hands)
  self.grasp_state={**{s:dict(body=g['body']) for s,g in self.tail_pinch.held.items()},**{s:dict(body=g['body']) for s,g in self.tool_pinch.held.items()}}
  tool=self.p('scaler',HEAD_OFFSET);local_tool=(tool-self.p('fish'))@self.orientation('fish');velocity=np.zeros(3) if self.previous_tool is None else (local_tool-self.previous_tool)/dt;self.previous_tool=local_tool.copy()
  for i in PATCHES:
   normal=self.orientation('skin_'+str(i))[:,2];compression=max(0.,-np.dot(self.joint_gap('skin_bond_'+str(i)),normal));self.peak_compression=max(self.peak_compression,compression);load=700*compression;self.maximum_load=max(self.maximum_load,load)
   point=self.p('skin_'+str(i));q=(point-self.p('scaler'))@self.orientation('scaler')
   contact=abs(q[0])<.013 and .044<q[1]<.076 and abs(q[2]-TOOTH_Z)<.0015
   # Stroke must be tail-to-head, under load, across the actual teeth.
   if i not in self.removed and contact and load>=.07:
    self.work[i]+=load*max(0.,-velocity[1])*dt
    if self.work[i]>=.0025:
     self.disable('scale_retention_'+str(i),'scales_scraped_free');self.removed.add(i);self.restore_scale_collision(i)
  self.failed |= self.integrity_failed()
  clear=all(h['opening']>.4 and min(np.linalg.norm(h['tcp']-self.p(n)) for n in ['fish','scaler'])>.065 for h in hands.values())
  self.release=self.release+dt if clear else 0.
 def restore_scale_collision(self,i):
  from pxr import UsdPhysics
  scale='scales_'+str(i);others={'fish','skin_'+str(i)}
  for prim in self.stage.Traverse():
   if '/materials/' not in str(prim.GetPath()) or not prim.HasAPI(UsdPhysics.FilteredPairsAPI):continue
   source=prim.GetName();rel=UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
   for target in rel.GetTargets():
    other=str(target).rsplit('/',1)[-1]
    if source==scale and other in others or source in others and other==scale:rel.RemoveTarget(target)
 def integrity_failed(self):return super().integrity_failed() or bool(self.broken) or self.peak_compression>.002
 def reset_valid(self):return not self.removed and not self.failed
 def measure(self):
  p=self.p('fish');upright=self.orientation('fish')[2,2]>.8;on_board=abs(p[0]-.38)<.09 and abs(p[1])<.05 and TABLE+.012<p[2]<TABLE+.045
  return dict(descaled_area_fraction=len(self.removed)/len(PATCHES),scale_groups_removed=len(self.removed),scale_groups_total=len(PATCHES),skin_intact=not self.failed,maximum_skin_compression_mm=1000*self.peak_compression,maximum_patch_normal_load_N=self.maximum_load,fish_on_board=bool(on_board and upright),released_seconds=self.release,failed=self.failed)
 def score(self):return 0. if self.failed or self.release<3 or not self.measure()['fish_on_board'] else len(self.removed)/len(PATCHES)
 def draw(self,d):
  # Gill line and dorsal speckling follow the native fish rather than a timeline.
  for sign in [-1,1]:
   pts=np.array([[x,-.067+abs(x)*.25,sign*.012] for x in np.linspace(-.021,.021,15)])
   d.tube('gill_'+str(sign).replace('-','n'),pts@self.orientation('fish').T+self.p('fish'),.0006,(.30,.34,.27),5)
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,work=self.work,removed=sorted(self.removed),peak_compression=self.peak_compression,previous_tool=self.previous_tool,maximum_load=self.maximum_load)
 def restore(self,state):
  s=dict(state);self.removed=set(s.pop('removed'));self.previous_tool=None if s['previous_tool'] is None else np.asarray(s.pop('previous_tool'));s.pop('previous_tool',None);super().restore(s)
