import numpy as np
from native_model import NativeModel
from geometry import HEAD,HEADROT,RADII,POINTS,TRIS,RIM,EDGES,REST_LENGTHS
def surface_gap(points,tris,target):
 # Distance to the actual sheet, independent of which material point is its apex.
 t=np.asarray(points)[tris];q=np.asarray(target);a,b,c=t[:,0],t[:,1],t[:,2]
 ab=b-a;ac=c-a;aq=q-a
 d00=np.sum(ab*ab,axis=1);d01=np.sum(ab*ac,axis=1);d11=np.sum(ac*ac,axis=1)
 d20=np.sum(aq*ab,axis=1);d21=np.sum(aq*ac,axis=1);den=d00*d11-d01*d01
 v=np.divide(d11*d20-d01*d21,den,out=np.full_like(den,-1.),where=den>1e-16)
 w=np.divide(d00*d21-d01*d20,den,out=np.full_like(den,-1.),where=den>1e-16)
 inside=(v>=0)&(w>=0)&(v+w<=1)&(den>1e-16)
 projected=a+v[:,None]*ab+w[:,None]*ac
 distances=np.where(inside,np.linalg.norm(projected-q,axis=1),np.inf)
 for x,y in [(a,b),(b,c),(c,a)]:
  e=y-x;u=np.clip(np.sum((q-x)*e,axis=1)/np.maximum(np.sum(e*e,axis=1),1e-20),0,1)
  distances=np.minimum(distances,np.linalg.norm(x+u[:,None]*e-q,axis=1))
 return float(distances.min())

class Model(NativeModel):
 def __init__(self):super().__init__();self.grasp_state={};self.nodes=POINTS.copy();self.node_velocities=np.zeros_like(self.nodes);self.peak_stretch=1.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  self.surface=None;super().bind_native(base)
  from isaaclab_physx.physics import PhysxManager as SimulationManager
  view=SimulationManager.get_physics_sim_view();self.surface=view.create_surface_deformable_body_view('/World/envs/env_*/materials/cap')
  if self.surface.count!=1:raise RuntimeError('Expected one native cap surface')
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,[n for n in self.names if n.startswith('rim_contact_')],0.0015,0.003,0.012,(-0.078, 0.002))
  self.read()
 def read(self):
  super().read()
  if getattr(self,'surface',None) is not None:
   self.nodes=self.surface.get_simulation_nodal_positions().numpy()[0,:len(POINTS)].copy()
   self.node_velocities=self.surface.get_simulation_nodal_velocities().numpy()[0,:len(POINTS)].copy()
 def release_points(self):return self.nodes
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.pinch.update(hands)
  clear=all(h['opening']>.4 and np.min(np.linalg.norm(self.nodes-h['tcp'],axis=1))>.05 for h in hands.values())
  self.release=self.release+dt if clear else 0.
  self.peak_stretch=max(self.peak_stretch,float(np.max(np.linalg.norm(self.nodes[EDGES[:,1]]-self.nodes[EDGES[:,0]],axis=1)/REST_LENGTHS)))
  self.failed |= self.integrity_failed()
 def rim_gap(self):return max(float(np.linalg.norm(self.nodes[node]-self.p(f'rim_contact_{i:03d}'))) for i,node in enumerate(RIM))
 def integrity_failed(self):return self.rim_gap()>.005 or super().integrity_failed() or not np.isfinite(self.nodes).all() or self.peak_stretch>3.
 def measure(self):
  rim=self.nodes[RIM];local=(rim-HEAD)@HEADROT
  # The rim must encircle the head below the crown, while its front arc
  # stays above the eyes. The normalised radial check excludes a cap beside it.
  angle=np.arctan2(local[:,1],local[:,0]);d=np.diff(np.r_[angle,angle[0]]);winding=abs(np.sum((d+np.pi)%(2*np.pi)-np.pi))/(2*np.pi)
  front=local[:,0]<-.045
  radial=np.sqrt((local[:,0]/RADII[0])**2+(local[:,1]/RADII[1])**2)
  crown=surface_gap(self.nodes,TRIS,HEAD+HEADROT@np.array([0,0,RADII[2]]))
  fitted=bool(winding>.9 and radial.min()>.55 and radial.max()<1.35 and local[:,2].max()<.075 and local[:,2].min()>-.055)
  progressing=bool(winding>.9 and radial.min()>.55 and radial.max()<1.35 and local[:,2].max()<.100 and local[:,2].min()>-.055)
  clear=bool(front.any() and local[front,2].min()>=.030)
  return dict(rim_attachment_gap_mm=self.rim_gap()*1000,crown_gap_mm=crown*1000,rim_encircles_head=fitted,rim_partly_lowered=progressing,face_clear=clear,intact=not self.failed,released_seconds=self.release,failed=self.failed)
 def score(self):
  self.failed |= self.integrity_failed();m=self.measure()
  if self.failed:return 0.
  if m['rim_encircles_head'] and m['face_clear'] and m['crown_gap_mm']<=20 and self.release>=3:return 1.
  if m['rim_encircles_head'] and m['face_clear']:return .5
  return .25 if m['rim_partly_lowered'] and m['face_clear'] else 0.
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,nodes=self.nodes,node_velocities=self.node_velocities,peak_stretch=self.peak_stretch)
 def restore(self,state):
  s=dict(state);self.nodes=np.asarray(s.pop('nodes'));self.node_velocities=np.asarray(s.pop('node_velocities'));self.peak_stretch=s.pop('peak_stretch');super().restore(s)
