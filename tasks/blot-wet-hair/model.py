import numpy as np
from native_model import NativeModel
from geometry import POINTS,EDGES,REST_LENGTHS,STRANDS,SEGMENTS,TRIS,PATCHES,BODIES
from absorption import distances,advance
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.nodes=POINTS.copy();self.node_velocities=np.zeros_like(self.nodes);self.grasp_state={};self.peak_stretch=1.;self.water=np.full(STRANDS*SEGMENTS,18./(STRANDS*SEGMENTS));self.uptake=0.;self.contact=np.zeros(STRANDS*SEGMENTS,dtype=bool)
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  self.surface=None;super().bind_native(base)
  from isaaclab_physx.physics import PhysxManager
  self.surface=PhysxManager.get_physics_sim_view().create_surface_deformable_body_view('/World/envs/env_*/materials/towel')
  if self.surface.count!=1:raise RuntimeError('Expected one native towel')
  self.read()
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,[n for n,i in PATCHES],.0015,.003,.012,(-.025,.002))
 def read(self):
  super().read()
  if getattr(self,'surface',None) is not None:
   self.nodes=self.surface.get_simulation_nodal_positions().numpy()[0,:len(POINTS)].copy();self.node_velocities=self.surface.get_simulation_nodal_velocities().numpy()[0,:len(POINTS)].copy()
 def contacts(self):
  names=[f'hair_{s}_{i}' for s in range(STRANDS) for i in range(SEGMENTS)]
  lengths={b['name']:b['shapes'][0].get('length',0) for b in BODIES if b['name'].startswith('hair_')}
  # Three samples along each capsule avoid contact only at its endpoint being
  # mistaken for a dry centre. Distance is to cloth triangles, not mesh vertices.
  points=np.array([self.p(n,[0,0,t*lengths[n]]) for n in names for t in [-.4,0,.4]])
  return distances(points,self.nodes,TRIS).reshape(-1,3).min(axis=1)<=.0034
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.pinch.update(hands)
  self.contact=self.contacts()
  if self.time>=0:self.water,self.uptake=advance(self.water,self.uptake,self.contact,dt)
  self.peak_stretch=max(self.peak_stretch,float(np.max(np.linalg.norm(self.nodes[EDGES[:,1]]-self.nodes[EDGES[:,0]],axis=1)/REST_LENGTHS)))
  points=np.vstack([self.nodes,self.poses[[i for i,n in enumerate(self.names) if n.startswith('hair_')],:3]])
  clear=all(h['opening']>.4 and np.min(np.linalg.norm(points-h['tcp'],axis=1))>.05 for h in hands.values());self.release=self.release+dt if clear else 0.
  self.failed |= self.integrity_failed()
 def integrity_failed(self):
  return super().integrity_failed() or bool(self.broken) or not np.isfinite(self.nodes).all() or self.peak_stretch>1.3 or any(np.linalg.norm(self.p(n)-self.nodes[i])>.005 for n,i in PATCHES)
 def reset_valid(self):return self.uptake<1e-8 and not self.failed
 def measure(self):
  return dict(water_transferred_g=self.uptake,water_remaining_g=float(self.water.sum()),fraction_removed=self.uptake/18.,intact=not self.failed,released_seconds=self.release,failed=self.failed)
 def score(self):
  if self.failed:return 0.
  f=self.uptake/18.
  return 1. if f>=.7 and self.release>=3 else .5 if f>=.5 else .25 if f>=.25 else 0.
 def draw(self,d):
  from hair_visual import draw
  draw(self,d)
 def state(self):return dict(super().state(),nodes=self.nodes,node_velocities=self.node_velocities,grasp_state=self.grasp_state,peak_stretch=self.peak_stretch,water=self.water,uptake=self.uptake,contact=self.contact)
 def restore(self,state):
  s=dict(state)
  for key in ['nodes','node_velocities','water','contact']:setattr(self,key,np.asarray(s.pop(key)))
  self.peak_stretch=s.pop('peak_stretch');super().restore(s)
