"""Contact/dwell adhesion on a passive native sheet; leak-path proxy, not CFD."""
import numpy as np
from scipy.ndimage import binary_dilation,label
from native_model import NativeModel
from geometry import *
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.dwell=np.zeros((len(CELLS),len(SAMPLES)));self.bonded=np.zeros_like(self.dwell,dtype=bool);self.bond_cells=set();self.peak_gap=0.;self.roller_load=0.;self.sealed=np.zeros(96,dtype=bool)
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  # Environment reset restores body visibility; hide only the native collision
  # shapes after reset. The translucent mesh below follows their measured poses.
  from pxr import UsdGeom
  root=str(self.joints['roller_compliance'].GetBody0Rel().GetTargets()[0]).rsplit('/',1)[0]
  for name in CELLS:UsdGeom.Imageable(self.stage.GetPrimAtPath(root+'/'+name)).CreateVisibilityAttr().Set('invisible')
  from finite_pinch import FinitePinch
  self.film_pinch=FinitePinch(self,[f'film_{NX//2}_{j}' for j in [0,NY-1]],THICKNESS/2,.001,.003,(-.004,.004),force_limit=.4,torque_limit=.001)
  self.tool_pinch=FinitePinch(self,['roller_handle'],.005,.001,.003,(-.006,.004))
 def sample_positions(self):return np.array([SAMPLES@self.orientation(n).T+self.p(n) for n in CELLS])
 def create_bond(self,index):
  from pxr import UsdPhysics,Sdf,Gf
  from scipy.spatial.transform import Rotation
  name=CELLS[index];root=str(self.joints['roller_compliance'].GetBody0Rel().GetTargets()[0]).rsplit('/',1)[0];path=root+'/joint_adhesive_'+name
  joint=UsdPhysics.Joint.Define(self.stage,path);joint.CreateJointEnabledAttr(False);joint.CreateExcludeFromArticulationAttr(True)
  joint.CreateBody0Rel().SetTargets([Sdf.Path(root+'/plate')]);joint.CreateBody1Rel().SetTargets([Sdf.Path(root+'/'+name)])
  # One finite point bond per film sector, anchored on an actually pressed rim.
  # Free rotation preserves the sheet's bending instead of freezing a rigid tile.
  sample=int(np.flatnonzero(self.bonded[index])[0]);anchor=SAMPLES[sample];world=self.p(name,anchor)
  q=Rotation.from_matrix(self.orientation('plate').T@self.orientation(name)).as_quat();local=(world-self.p('plate'))@self.orientation('plate')
  joint.CreateLocalPos0Attr(Gf.Vec3f(*local));joint.CreateLocalPos1Attr(Gf.Vec3f(*anchor));joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])));joint.CreateLocalRot1Attr(Gf.Quatf(1,Gf.Vec3f(0,0,0)))
  for axis in ['transX','transY','transZ']:
   lim=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis);lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
  joint.CreateCollisionEnabledAttr(True);joint.CreateBreakForceAttr(1.);joint.GetJointEnabledAttr().Set(True)
  self.bond_cells.add(index);self.events.append(dict(event='pressure_bond',cell=index,sample=sample,point_world=world.tolist(),roller_load_N=self.roller_load,time=self.time))
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.film_pinch.update(hands);self.tool_pinch.update(hands)
  self.grasp_state={**{s:dict(body=g['body']) for s,g in self.film_pinch.held.items()},**{s:dict(body=g['body']) for s,g in self.tool_pinch.held.items()}}
  R=self.orientation('plate');p=self.sample_positions();local=(p-self.p('plate'))@R
  head=self.p('roller_head');hr=self.orientation('roller_handle');delta=np.dot(self.joint_gap('roller_compliance'),hr[:,2]);self.roller_load=max(0.,800*delta)
  # Rim lands, not the open holes, carry the adhesive. The padded wheel's
  # native compression supplies the load; hovering or unloaded touch cannot seal.
  land=(abs((local[:,:,0]+.054+.0045)%.009-.0045)<.0010)|(abs((local[:,:,1]+.036+.0045)%.009-.0045)<.0010)
  over=(abs(local[:,:,0])<.055)&(abs(local[:,:,1])<.037)&land
  near=abs(local[:,:,2]-.017)<.0008
  wheel=(p-head)@hr
  pressed=(abs(wheel[:,:,0])<.025)&(abs(wheel[:,:,1])<.004)&(abs(wheel[:,:,2]+.012)<.0015)&(self.roller_load>=.30)
  good=over&near&pressed
  self.dwell=np.where(good,self.dwell+dt,np.maximum(0,self.dwell-dt*.25));self.bonded|=self.dwell>=.10
  for i in np.flatnonzero(np.any(self.bonded,axis=1)):
   if i not in self.bond_cells:self.create_bond(int(i))
   if 'adhesive_'+CELLS[i] in self.broken:self.bonded[i]=False
  self.peak_gap=max(self.peak_gap,max(float(np.linalg.norm(self.joint_gap(j['name']))) for j in JOINTS if j['name']!='roller_compliance'))
  self.failed |= self.integrity_failed() or R[2,2]<np.cos(np.deg2rad(35))
  clear=all(h['opening']>.4 and min(np.linalg.norm(h['tcp']-self.p(n)) for n in CELLS+['plate','roller_handle'])>.05 for h in hands.values())
  self.release=self.release+dt if clear else 0.;self.sealed=self.leak_check()
 def leak_check(self):
  # Flood fill the unbonded interface at 0.75 mm resolution. A well must have
  # no route to the outside or any other well. Its film must still cover it.
  R=self.orientation('plate');p=(self.sample_positions()-self.p('plate'))@R
  good=self.bonded&(abs(p[:,:,2]-.017)<.0010)
  points=p[good];grid=np.zeros((201,141),bool);dx=.00075
  if len(points):
   ix=np.rint(points[:,:2]/dx+[100,70]).astype(int);ok=(ix[:,0]>=0)&(ix[:,0]<201)&(ix[:,1]>=0)&(ix[:,1]<141);grid[ix[ok,0],ix[ok,1]]=True
  grid=binary_dilation(grid,iterations=1);labels,_=label(~grid)
  outside=set(np.r_[labels[0,:],labels[-1,:],labels[:,0],labels[:,-1]])
  cells=np.rint(WELLS[:,:2]/dx+[100,70]).astype(int);regions=labels[cells[:,0],cells[:,1]]
  centres=np.array([self.p(n) for n in CELLS]);rotations=np.array([self.orientation(n) for n in CELLS]);world=WELLS@R.T+self.p('plate')
  delta=world[:,None,:]-centres[None,:,:];q=np.einsum('wci,cij->wcj',delta,rotations)
  coverage=np.any((abs(q[:,:,0])<DX/2)&(abs(q[:,:,1])<DY/2)&(abs(q[:,:,2])<.0015),axis=1)
  return np.array([bool(cover and region and region not in outside and np.sum(regions==region)==1) for cover,region in zip(coverage,regions)])

 def integrity_failed(self):return super().integrity_failed() or any(not n.startswith('adhesive_') for n in self.broken) or self.peak_gap>.001
 def reset_valid(self):return not self.bonded.any() and not self.failed
 def measure(self):return dict(sealed_wells=int(self.sealed.sum()),total_wells=96,bonded_samples=int(self.bonded.sum()),roller_normal_load_N=self.roller_load,maximum_sheet_joint_gap_mm=self.peak_gap*1000,released_seconds=self.release,failed=self.failed)
 def score(self):return 0. if self.failed or self.release<3 else float(self.sealed.mean())
 def draw(self,d):
  faces=[(0,1,2,3)]
  for name in CELLS:
   local=np.array([[-DX/2,-DY/2,.00008],[DX/2,-DY/2,.00008],[DX/2,DY/2,.00008],[-DX/2,DY/2,.00008]])
   d.mesh(name+'_visible',local@self.orientation(name).T+self.p(name),faces,(.62,.78,.83,.35))
  for i,q in enumerate(WELLS):
   d.box('sample_'+str(i),self.p('plate',q+[0,0,-.011]),(.0073,.0073,.008),(.13,.40,.70) if (i//8+i%8)%2 else (.78,.86,.90),self.orientation('plate'))
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,dwell=self.dwell,bonded=self.bonded,bond_cells=sorted(self.bond_cells),peak_gap=self.peak_gap,roller_load=self.roller_load,sealed=self.sealed)
 def restore(self,state):
  s=dict(state)
  for key in ['dwell','bonded','sealed']:setattr(self,key,np.asarray(s.pop(key)))
  self.bond_cells=set(s.pop('bond_cells'));super().restore(s)
