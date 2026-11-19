import numpy as np
from native_model import NativeModel
from geometry import HEAD,POINTS,EDGES,REST_LENGTHS,STRANDS,SEGMENTS,M,BAND_ATTACHMENTS,ELASTIC,BAND_CONTACT_INDICES
from retention import fresh,advance
class Model(NativeModel):
 def __init__(self):super().__init__();self.grasp_state={};self.nodes=POINTS.copy();self.node_velocities=np.zeros_like(self.nodes);self.peak_stretch=1.;self.retention=fresh();self.proof_force_n=0.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  self.surface=None;super().bind_native(base)
  from isaaclab_physx.physics import PhysxManager
  self.surface=PhysxManager.get_physics_sim_view().create_surface_deformable_body_view('/World/envs/env_*/materials/elastic')
  if self.surface.count!=1:raise RuntimeError('Expected one native elastic surface')
  from pxr import PhysxSchema
  self.proof_force_apis=[PhysxSchema.PhysxForceAPI(self.stage.GetPrimAtPath('/World/envs/env_0/materials/band_segment_%03d'%i)) for i in BAND_CONTACT_INDICES]
  if any(not a for a in self.proof_force_apis):raise RuntimeError('Retention force API missing')
  self.read()
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,[n for n in self.names if n.startswith('band_segment_')],.0015,.003,.004,(-.025,.012))
 def read(self):
  super().read()
  if getattr(self,'surface',None) is not None:
   self.nodes=self.surface.get_simulation_nodal_positions().numpy()[0,:len(POINTS)].copy()
   self.node_velocities=self.surface.get_simulation_nodal_velocities().numpy()[0,:len(POINTS)].copy()
 def rim_gap(self):return max(float(np.linalg.norm(self.nodes[ids]-(local@self.orientation(name).T+self.p(name)),axis=1).max()) for name,ids,local in BAND_ATTACHMENTS)
 def release_points(self):return np.vstack([self.nodes,self.poses[[i for i,n in enumerate(self.names) if n.startswith('hair_')],:3]])
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.pinch.update(hands);p=self.release_points();clear=all(h['opening']>.4 and np.min(np.linalg.norm(p-h['tcp'],axis=1))>.05 for h in hands.values());self.release=self.release+dt if clear else 0.
  centre=self.nodes[M:2*M].mean(axis=0)
  self.retention,self.proof_force_n=advance(self.retention,centre,self.release,np.linalg.norm(centre-ELASTIC)>.05,dt)
  from pxr import Gf
  for api in self.proof_force_apis:api.GetForceAttr().Set(Gf.Vec3f(0.,0.,-self.proof_force_n/len(self.proof_force_apis)))
  self.peak_stretch=max(self.peak_stretch,float(np.max(np.linalg.norm(self.nodes[EDGES[:,1]]-self.nodes[EDGES[:,0]],axis=1)/REST_LENGTHS)));self.failed |= self.integrity_failed()
 def integrity_failed(self):return self.rim_gap()>.005 or super().integrity_failed() or not np.isfinite(self.nodes).all() or self.peak_stretch>5. or bool(self.broken)
 def measure(self):
  ring=self.nodes[M:2*M];centre=ring.mean(axis=0);_,_,axes=np.linalg.svd(ring-centre,full_matrices=False);normal=axes[2];crossings=[];held=0
  for s in range(STRANDS):
   points=np.array([self.p(f'hair_{s}_{i}') for i in range(SEGMENTS)]);signed=(points-centre)@normal
   cross=np.flatnonzero(signed[:-1]*signed[1:]<=0)
   inside=False
   for i in cross:
    u=signed[i]/(signed[i]-signed[i+1]+1e-12);c=points[i]+u*(points[i+1]-points[i]);local=(ring-c)@axes[:2].T;a=np.arctan2(local[:,1],local[:,0]);d=np.diff(np.r_[a,a[0]]);w=abs(np.sum((d+np.pi)%(2*np.pi)-np.pi))/(2*np.pi)
    if w>.8:inside=True
   held+=inside
  # Retention around a common gathering point, below the scalp and above the
  # last 60 mm of hair. Passing the loop over only the tips does not finish.
  usable=bool(HEAD[2]-.16<centre[2]<HEAD[2]-.025 and centre[0]<HEAD[0]-.055)
  return dict(secured_locks=held,total_locks=STRANDS,elastic_at_ponytail_base=usable,elastic_diameter_mm=float(np.ptp((ring-centre)@axes[:2].T,axis=0).max()*1000),intact=not self.failed,released_seconds=self.release,failed=self.failed,retention_completed=self.retention['completed'],retention_slip_mm=self.retention['maximum_slip_mm'],proof_force_n=self.proof_force_n)
 def reset_valid(self):
  from common import TABLE
  return self.nodes[:,2].min()>TABLE+.035 and self.measure()['secured_locks']==0
 def score(self):
  if self.failed:return 0.
  m=self.measure()
  if not m['retention_completed'] or m['retention_slip_mm']>10.:return 0.
  if m['secured_locks']>=11 and m['elastic_at_ponytail_base'] and m['elastic_diameter_mm']<=50 and self.release>=3:return 1.
  if m['elastic_at_ponytail_base'] and m['secured_locks']>=9:return .5
  return .25 if m['elastic_at_ponytail_base'] and m['secured_locks']>=6 else 0.
 def draw(self,d):
  from hair_visual import draw
  draw(self,d)
  ring=self.nodes[M:2*M];d.tube("plain_elastic",np.vstack([ring,ring[0]]),.0015,(.64,.12,.27),sides=10)
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,nodes=self.nodes,node_velocities=self.node_velocities,peak_stretch=self.peak_stretch,retention=self.retention,proof_force_n=self.proof_force_n)
 def restore(self,state):
  s=dict(state);self.nodes=np.asarray(s.pop('nodes'),dtype=np.float32);self.node_velocities=np.asarray(s.pop('node_velocities'),dtype=np.float32);self.peak_stretch=s.pop('peak_stretch');super().restore(s)
