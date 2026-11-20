import numpy as np
from native_model import NativeModel
from geometry import BODIES,JOINTS,C,N,FLESH,AXIS,TABLE
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.extracted=set();self.peak_flesh_gap=0.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  from pxr import UsdGeom
  for name in FLESH:
   path=self.joints['flesh_bond_'+name].GetBody1Rel().GetTargets()[0]
   UsdGeom.Imageable(self.stage.GetPrimAtPath(path)).MakeInvisible()
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,[f'bone_tip_{i}' for i in range(N)],.0006,.001,.003,(-.006,.004))
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.pinch.update(hands)
  for i in range(N):
   name=f'bone_socket_{i}';gap=self.joint_gap(name);j=next(j for j in JOINTS if j['name']==name)
   axis=self.orientation(j['a'])@AXIS
   if i not in self.extracted and np.dot(gap,axis)>.023:
    self.disable(name,'bone_fully_extracted');self.extracted.add(i);self.restore_extracted_contacts(i)
  for name in FLESH:
   joint='flesh_bond_'+name;gap=float(np.linalg.norm(self.joint_gap(joint)))
   self.peak_flesh_gap=max(self.peak_flesh_gap,gap)
   if gap>=.005:self.disable(joint,'flesh_torn_or_crushed')
  dynamic=[j for j,b in enumerate(BODIES) if not b.get('fixed')]
  clear=all(h['opening']>.4 and np.min(np.linalg.norm(self.poses[dynamic,:3]-h['tcp'],axis=1))>.05 for h in hands.values())
  self.release=self.release+dt if clear else 0.;self.failed |= self.integrity_failed()
 def restore_extracted_contacts(self,i):
  from pxr import UsdPhysics
  bones={f'bone_base_{i}',f'bone_tip_{i}'};flesh=set(FLESH+['skin'])
  for p in self.stage.Traverse():
   if '/materials/' not in str(p.GetPath()) or not p.HasAPI(UsdPhysics.FilteredPairsAPI):continue
   source=p.GetName();rel=UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel()
   for target in rel.GetTargets():
    other=str(target).rsplit('/',1)[-1]
    if (source in bones and other in flesh) or (source in flesh and other in bones):rel.RemoveTarget(target)
 def integrity_failed(self):
  return super().integrity_failed() or bool(self.broken) or self.peak_flesh_gap>=.005
 def reset_valid(self):return not self.extracted and not self.failed
 def measure(self):
  dish=self.p('dish');R=self.orientation('dish');count=0
  for i in range(N):
   endpoints=[self.p(f'bone_base_{i}',[0,0,-.011]),self.p(f'bone_tip_{i}',[0,0,.003])]
   local=(np.asarray(endpoints)-dish)@R
   count+=i in self.extracted and np.all(np.linalg.norm(local[:,:2],axis=1)<.041) and np.all(local[:,2]>0) and np.all(local[:,2]<.030)
  skin=self.p('skin');on_board=bool(abs(skin[0]-.36)<.09 and abs(skin[1])<.055 and TABLE+.011<skin[2]<TABLE+.03)
  dish_ready=bool(R[2,2]>=np.cos(np.deg2rad(15)) and abs(dish[2]-(TABLE+.002))<=.005)
  return dict(dish_upright_on_table=dish_ready,fillet_on_board=on_board,extracted_bones=len(self.extracted),whole_bones_in_dish=count,total_bones=N,flesh_intact=not self.failed,maximum_flesh_displacement_mm=self.peak_flesh_gap*1000,released_seconds=self.release,failed=self.failed)
 def score(self):
  if self.failed or self.release<3 or not self.measure()['fillet_on_board'] or not self.measure()['dish_upright_on_table']:return 0.
  return self.measure()['whole_bones_in_dish']/N
 def draw(self,d):
  # The visible surface interpolates the small gaps between intact native
  # sectors. A damaged sector separates visually with its actual body pose.
  faces=[(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)]
  for row in range(4):
   for col in range(3):
    name=f'flesh_{row}_{col}';top=[];bottom=[]
    for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]:
     gx=col+(sx==1);gy=row+(sy==1);samples=[]
     intact='flesh_bond_'+name not in self.disabled and 'flesh_bond_'+name not in self.broken
     for rr in [gy-1,gy]:
      for cc in [gx-1,gx]:
       if not 0<=rr<4 or not 0<=cc<3:continue
       other=f'flesh_{rr}_{cc}'
       if not intact or 'flesh_bond_'+other in self.disabled or 'flesh_bond_'+other in self.broken:continue
       samples.append(self.p(other,[(1 if gx>cc else -1)*.0147,(1 if gy>rr else -1)*.01825,.011]))
     own=self.p(name,[sx*.0147,sy*.01825,.011])
     top.append(np.mean(samples,axis=0) if samples else own)
     bottom.append(self.p(name,[sx*.0147,sy*.01825,-.011]))
    d.mesh(name+'_surface',np.array(top+bottom),faces,(.90,.28,.15))
  # Pale fat lines follow the physical tissue; they do not hide damage.
  for name in FLESH:
   R=self.orientation(name);p=self.p(name)
   for k,y in enumerate([-.010,.002,.014]):
    d.tube(name+'_stripe_'+str(k),np.array([[-.014,y-.003,.0112],[0,y,.0112],[.014,y+.003,.0112]])@R.T+p,.0006,(.98,.79,.65),sides=4)
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,extracted=sorted(self.extracted),peak_flesh_gap=self.peak_flesh_gap)
 def restore(self,state):
  state=dict(state);self.extracted=set(state.pop('extracted'));super().restore(state)
