"""Contact-and-compression-triggered local ideal adhesion.
Only engaged rigid strip patches are constrained; adjoining elastic joints
retain dynamics. Adhesion is resolved with the other joints by PhysX.
"""
import numpy as np
from scipy.spatial.transform import Rotation
class VelcroContact:
 def __init__(self,physics,hardware):
  self.h=hardware;self.bond_index=None;self.bonded_indices=[];self.local={};self.hold={};self.T=None;self.subscription=None
  self.view=physics.create_rigid_body_view('/World/envs/env_0/service_flap/segment_*')
  self.contacts=physics.create_rigid_contact_view('/World/envs/env_0/service_flap/segment_*',filter_patterns=[hardware.CREATED['links']['link3']])
 def matches(self,poses,T):
  B=T[:3,:3]@self.h.B;c=(T@np.r_[self.h.MOUNT,1])[:3];out=[]
  for i,row in enumerate(poses):
   R=Rotation.from_quat(row[3:]).as_matrix();p=(row[:3]+R[:,2]*.00115-c)@B
   # <=1.5mm numerical contact envelope, true pad radius17mm.
   if abs(p[0]-.0355)<=.0015 and np.linalg.norm(p[1:])<.017 and R[:,2]@B[:,0]<-.866:out.append(i)
  return out
 def bind(self):
  import omni.usd
  self.stage=omni.usd.get_context().get_stage()
 def update(self,poses,T,dt):
  self.T=T.copy()
  if len(self.bonded_indices)==len(poses):return False
  f=self.contacts.get_contact_force_matrix(dt);f=f.numpy() if hasattr(f,'numpy') else f.cpu().numpy()
  forces=np.linalg.norm(f.reshape(len(poses),-1,3),axis=2).sum(axis=1);self.last_forces=forces.tolist()
  ids=self.matches(poses,T)
  for i in range(len(poses)):self.hold[i]=self.hold.get(i,0)+dt if i in ids and i not in self.local and forces[i]>.025 else 0.
  active=[i for i in ids if self.hold[i]>=.10-1e-8]
  if not active:return False
  # Pick the strongest actually pressed patch, not the entire strip.
  i=max(active,key=lambda i:forces[i]);row=poses[i]
  from pxr import Gf, Sdf, UsdPhysics
  p=(row[:3]-T[:3,3])@T[:3,:3]
  R=T[:3,:3].T@Rotation.from_quat(row[3:]).as_matrix()
  q=Rotation.from_matrix(R).as_quat()
  root=self.h.CREATED['root']
  joint=UsdPhysics.FixedJoint.Define(self.stage,f'{root}/pad_bond_{i:03d}')
  joint.CreateBody0Rel().SetTargets([Sdf.Path(self.h.CREATED['links']['link3'])])
  joint.CreateBody1Rel().SetTargets([Sdf.Path(f'{root}/segment_{i:03d}')])
  joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)))
  joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
  joint.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
  joint.CreateLocalRot1Attr(Gf.Quatf(1.,0.,0.,0.))
  joint.CreateExcludeFromArticulationAttr(True)
  joint.CreateCollisionEnabledAttr(True)
  joint.CreateJointEnabledAttr(True)
  self.local[i]=(p,R)
  self.bonded_indices.append(i)
  if self.bond_index is None:self.bond_index=i
  return True
 def maintain(self,T):self.T=T.copy()
