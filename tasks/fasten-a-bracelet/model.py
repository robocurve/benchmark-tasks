"""Read native bracelet dynamics and score physical pin insertion/retention."""
import numpy as np
from common import rotation
from bracelet_geometry import ANCHOR0,ANCHOR1
WRIST=np.array([.34,0.,.80]);N=41
class Model:
 def __init__(self):
  self.base=None;self.points=np.zeros((N,3));self.poses=np.zeros((N,7));self.velocities=np.zeros((N,6));self.events=[]
  self.latched=False;self.failed=False;self.native_catch_broken=False;self.time=0.;self.release=0.;self.entered=0.;self.depth=0.;self.radial=0.;self.angle=0.;self.max_gap=0.
 def bind_native(self,base):
  self.base=base
  import omni.usd
  from pxr import UsdPhysics,PhysicsSchemaTools
  stage=omni.usd.get_context().get_stage()
  paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='spring_catch']
  assert len(paths)==1,paths
  self.catch=UsdPhysics.SphericalJoint.Get(stage,paths[0]);self.catch.CreateJointEnabledAttr(False)
  from omni.physx import get_physx_interface
  from omni.physx.bindings._physx import SimulationEvent
  def native_event(event):
   if event.type==int(SimulationEvent.JOINT_BREAK):
    path=str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath']))
    if path==paths[0]:
     self.native_catch_broken=True;self.failed=True
     self.events.append(dict(event='native_catch_broke',time=self.time))
  self._native_event_subscription=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(native_event)
  self.read()
 def read(self):
  asset=self.base.scene['bracelet'];data=asset.data
  names=list(getattr(asset,'body_names',getattr(asset,'object_names',[])))
  ids=[names.index(f'link_{i:03d}') for i in range(N)]
  def arr(*fields):
   value=next(getattr(data,k) for k in fields if hasattr(data,k))
   return getattr(value,'torch',value).detach().cpu().numpy()[0,ids].copy()
  p=arr('body_link_pos_w','object_link_pos_w');q=arr('body_link_quat_w','object_link_quat_w')
  origin=self.base.scene.env_origins;origin=getattr(origin,'torch',origin).detach().cpu().numpy()[0]
  self.points=p-origin;self.poses=np.c_[self.points,q]
  self.velocities=np.c_[arr('body_link_lin_vel_w','object_link_lin_vel_w'),arr('body_link_ang_vel_w','object_link_ang_vel_w')]
  R=np.array([rotation(q) for q in self.poses[:,3:]])
  self.axes=-R[:,:,0]
  ends_a=self.points[:-1]+np.einsum('nij,nj->ni',R[:-1],ANCHOR0);ends_b=self.points[1:]+np.einsum('nij,nj->ni',R[1:],ANCHOR1)
  self.max_gap=float(np.max(np.linalg.norm(ends_a-ends_b,axis=1)))
  f,m=self.points[[0,-1]];axis=self.axes[0];tip=m+self.axes[-1]*.011;entry=f-axis*.005
  rel=tip-entry;depth=float(rel@axis);self.radial=float(np.linalg.norm(rel-depth*axis))
  self.angle=float(np.degrees(np.arccos(np.clip(axis@self.axes[-1],-1,1))))
  self.depth=depth
  self.entered=depth if self.radial<=.00055 and self.angle<=8 else 0.
 def update(self,hands,dt=.05):
  if self.base is None:raise RuntimeError('Bracelet requires native contact physics; CPU preview cannot demonstrate it.')
  self.time+=dt;self.read()
  if not self.latched and .0025<=self.entered<=.0035:
   # Reduced spring detent only after measured physical penetration into the bore.
   from pxr import Gf
   from scipy.spatial.transform import Rotation
   Rf,Rm=[rotation(q) for q in self.poses[[0,-1],3:]]
   anchor=Rm.T@(self.points[0]+Rf@np.array([.005,0.,0.])-self.points[-1])
   q=Rotation.from_matrix(Rm.T@Rf).as_quat()
   self.catch.GetLocalPos1Attr().Set(Gf.Vec3f(*anchor))
   self.catch.CreateLocalRot1Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
   self.catch.GetJointEnabledAttr().Set(True);self.latched=True
   self.events.append(dict(event='pin_spring_caught',time=self.time,depth_m=self.entered,radial_m=self.radial))
  if self.latched and (self.depth<.001 or self.depth>.004 or self.radial>.0015 or self.angle>15):self.failed=True
  self.failed |= self.native_catch_broken or self.max_gap>.004 or not np.isfinite(self.points).all()
  clear=all(h['opening']>.5 and np.min(np.linalg.norm(self.points-h['tcp'],axis=1))>.035 for h in hands.values())
  self.release=self.release+dt if clear else 0.
 def measure(self):
  p=self.points[:,1:]-WRIST[1:];angle=np.arctan2(p[:,1],p[:,0]);d=np.diff(np.r_[angle,angle[0]])
  turns=abs(np.sum((d+np.pi)%(2*np.pi)-np.pi))/(2*np.pi)
  return dict(clasp_closed=self.latched,inserted_mm=max(0.,self.entered)*1000,radial_error_mm=self.radial*1000,axis_error_deg=self.angle,
              around_wrist=bool(turns>.8),axial_offset_mm=float(np.max(abs(self.points[:,0]-.34))*1000),released_seconds=self.release,max_joint_gap_mm=self.max_gap*1000,failed=bool(self.failed))
 def score(self):
  m=self.measure()
  if m['failed'] or not m['around_wrist'] or m['axial_offset_mm']>50:return 0.
  if self.latched:return 1. if self.release>=3. else .5
  return .25 if self.entered>=.001 else 0.
 def state(self):return dict(poses=self.poses,velocities=self.velocities,latched=self.latched,native_catch_broken=self.native_catch_broken)
 def draw(self,d):pass
