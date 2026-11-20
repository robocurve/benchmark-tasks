"""Read-only native state plus disclosed, object-to-object material transitions."""
import numpy as np
from common import rotation
from geometry import BODIES,JOINTS

class NativeModel:
 def __init__(self):
  self.base=None;self.events=[];self.broken=set();self.disabled=set();self.enabled=set();self.failed=False;self.time=0.;self.release=0.
  self.names=[b['name'] for b in BODIES];self.index={n:i for i,n in enumerate(self.names)}
  self.poses=np.array([b['pos']+b.get('quat',[0,0,0,1]) for b in BODIES]);self.velocities=np.zeros((len(BODIES),6));self.R=np.array([rotation(p[3:]) for p in self.poses])
 def subscribe_events(self):
  from pxr import PhysicsSchemaTools
  from omni.physx import get_physx_interface
  from omni.physx.bindings._physx import SimulationEvent
  def native_event(event):
   if event.type==int(SimulationEvent.JOINT_BREAK):
    path=str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath']));name=path.rsplit('/',1)[-1]
    if name.startswith('joint_') and '/materials/' in path:
     name=name[6:];self.broken.add(name);self.events.append(dict(event='native_joint_break',joint=name,time=self.time))
  self.subscription=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(native_event)
 def bind_native(self,base):
  self.base=base
  import omni.usd
  from pxr import UsdPhysics
  self.stage=omni.usd.get_context().get_stage()
  self.joints={p.GetName()[6:]:UsdPhysics.Joint.Get(self.stage,str(p.GetPath())) for p in self.stage.Traverse() if p.GetName().startswith('joint_') and '/materials/' in str(p.GetPath())}
  self.read()
 def read(self):
  if self.base is None:raise RuntimeError('Native contact simulation required; a CPU plan is not task evidence.')
  asset=self.base.scene['materials'];data=asset.data;names=list(getattr(asset,'body_names',getattr(asset,'object_names',[])));ids=[names.index(n) for n in self.names]
  def arr(*fields):
   a=next(getattr(data,k) for k in fields if hasattr(data,k));return getattr(a,'torch',a).detach().cpu().numpy()[0,ids].copy()
  p=arr('body_link_pos_w','object_link_pos_w');q=arr('body_link_quat_w','object_link_quat_w')
  origin=self.base.scene.env_origins;origin=getattr(origin,'torch',origin).detach().cpu().numpy()[0]
  self.poses=np.c_[p-origin,q];self.R=np.array([rotation(q) for q in q]);self.velocities=np.c_[arr('body_link_lin_vel_w','object_link_lin_vel_w'),arr('body_link_ang_vel_w','object_link_ang_vel_w')]
  if not np.isfinite(self.poses).all():self.failed=True
 def p(self,name,local=(0,0,0)):
  i=self.index[name];return self.poses[i,:3]+self.R[i]@np.asarray(local)
 def orientation(self,name):return self.R[self.index[name]]
 def joint_gap(self,name):
  j=next(j for j in JOINTS if j['name']==name);return self.p(j['b'],j.get('p1',(0,0,0)))-self.p(j['a'],j.get('p0',(0,0,0)))
 def exclude_collision(self,a,b):
  from pxr import UsdPhysics,Sdf
  roots=[p for p in self.stage.Traverse() if p.GetName()==a and '/materials/' in str(p.GetPath())]
  assert len(roots)==1
  target=str(roots[0].GetPath()).rsplit('/',1)[0]+'/'+b
  UsdPhysics.FilteredPairsAPI.Apply(roots[0]).CreateFilteredPairsRel().AddTarget(Sdf.Path(target))
 def disable(self,name,reason):
  if name in self.disabled:return
  self.joints[name].GetJointEnabledAttr().Set(False);self.disabled.add(name);self.events.append(dict(event=reason,joint=name,time=self.time))
 def enable(self,name,reason):
  if name in self.enabled:return
  self.joints[name].GetJointEnabledAttr().Set(True);self.enabled.add(name);self.events.append(dict(event=reason,joint=name,time=self.time))
 def update(self,hands,dt=.05):
  self.read();self.time+=dt
  dynamic=[i for i,b in enumerate(BODIES) if not b.get('fixed')]
  clear=all(h['opening']>.4 and np.min(np.linalg.norm(self.poses[dynamic,:3]-h['tcp'],axis=1))>.05 for h in hands.values())
  self.release=self.release+dt if clear else 0.
  self.material(hands,dt)
  self.failed |= self.integrity_failed()
 def integrity_failed(self):return not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all())
 def restore(self,state):
  """Restore recorded observations for scoring only; never writes simulator state."""
  arrays={'poses','velocities','work','depth','loaded'};sets={'broken','disabled','enabled','lost'};maps={'holes','contact_uv','transferred'}
  for key,value in state.items():
   if not hasattr(self,key):raise ValueError('Unexpected recorded state field: '+key)
   if key in arrays:value=np.asarray(value)
   elif key in sets:value=set(value)
   elif key in maps:value={k:np.asarray(v) for k,v in value.items()}
   setattr(self,key,value)
  if self.poses.shape!=(len(self.names),7) or self.velocities.shape!=(len(self.names),6):raise ValueError('Incomplete body state')
  if not np.allclose(np.linalg.norm(self.poses[:,3:],axis=1),1.,atol=.002):raise ValueError('Invalid quaternion')
  self.R=np.array([rotation(q) for q in self.poses[:,3:]])
  self.failed |= self.integrity_failed()
 def material(self,hands,dt):pass
 def draw(self,d):pass
 def state(self):return dict(poses=self.poses,velocities=self.velocities,broken=sorted(self.broken),disabled=sorted(self.disabled),enabled=sorted(self.enabled),release=self.release,failed=self.failed)
