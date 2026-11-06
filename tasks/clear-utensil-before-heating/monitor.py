"""Fresh PhysX reads each 1/240 s; observation and irreversible safety history only."""
import numpy as np
import scenekit,geometry
from scoring import Trial
trial=Trial();completed_physics_samples=0;monitor_error=None;_base=None;_callback=None;_last_heat=None;_lamps=[]
CENTER=np.array((.36,0.));KNOB_XY=np.array((.21,0.));TABLE=.7475
mesh=np.load(__import__('pathlib').Path(__file__).parent/'visual-mesh.npz');V=mesh['vertices'];F=mesh['faces']
def is_energized(angle):
 return bool(np.isfinite(angle) and angle>np.radians(30))
# Every affine surface extremum lies on the convex hull; all original faces retained for contact.
from scipy.spatial import ConvexHull
H=V[ConvexHull(V).vertices]
def array(x):
 if hasattr(x,'torch'):x=x.torch
 return x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy()
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def _physics_step(*_):
 global _last_heat
 if _base is None:return
 origin=array(_base.scene.env_origins)[0]
 wp=array(_base.scene['wood'].root_view.get_transforms())[0];pp=array(_base.scene['pan'].root_view.get_transforms())[0]
 q=float(array(_base.scene['knob'].root_view.get_dof_positions())[0,0])
 w=H@rotation(wp[3:]).T+wp[:3]-origin
 pan=geometry.PAN_TRIANGLES.reshape(-1,3)@rotation(pp[3:]).T+pp[:3]-origin
 # Broad phase is only a sound exclusion; exact full boundary+solid follows near contact.
 near=(w[:,2].min()<=TABLE+geometry.BURNER_TOP+.002 and w[:,2].max()>=TABLE+geometry.BURNER_TOP-.002 and np.all(w[:,:2].min(0)<=CENTER+geometry.BURNER_RADIUS) and np.all(w[:,:2].max(0)>=CENTER-geometry.BURNER_RADIUS))
 if is_energized(q) and near and not trial.failed:
  triangles=(V@rotation(wp[3:]).T+wp[:3]-origin)[F]
 # Separate always-on whole rigid extrema checks from expensive contact.
 trial.physics_steps+=1
 if not np.isfinite(np.r_[wp,pp,q,w.reshape(-1),pan.reshape(-1)]).all():trial.fail('invalid_state')
 hp=array(_base.scene['hotplate'].root_view.get_transforms())[0];kp=array(_base.scene['knob'].root_view.get_root_transforms())[0]
 if not np.isfinite(np.r_[hp,kp]).all():trial.fail('invalid_state')
 for pose,target in ((hp,np.r_[CENTER,TABLE]),(kp,np.r_[KNOB_XY,TABLE+.004])):
  if np.linalg.norm(pose[:3]-origin-target)>.001 or rotation(pose[3:])[2,2]<np.cos(np.radians(1)):trial.fail('fixture_moved')
 if w[:,2].min()<TABLE-.05 or pan[:,2].min()<TABLE-.05:trial.fail('object_fell')
 if is_energized(q) and near and not trial.failed:
  from scoring import mesh_touches_contact_disk
  if mesh_touches_contact_disk(triangles,CENTER,geometry.BURNER_RADIUS,TABLE+geometry.BURNER_TOP,.002):trial.fail('heated_with_utensil_contact')
 heating=is_energized(q)
 if heating!=_last_heat:
  from pxr import Gf
  for attr in _lamps:attr.Set(Gf.Vec3f(*((1.,.025,.005) if heating else (.04,.003,.002))))
  _last_heat=heating
def physics_step(*args):
 global completed_physics_samples,monitor_error
 if monitor_error is not None:return
 try:
  _physics_step(*args)
  if _base is not None:completed_physics_samples+=1
 except Exception as exc:
  monitor_error=repr(exc);trial.fail('physics_monitor_error')
  print('FATAL PHYSICS MONITOR ERROR',monitor_error,flush=True)
@scenekit.on_reset
def reset(environment):
 global _base,_callback,_last_heat,_lamps,completed_physics_samples,monitor_error
 _base=environment.unwrapped;trial.reset();_last_heat=None;completed_physics_samples=0;monitor_error=None
 _lamps=[prim.GetAttribute('inputs:diffuseColor') for prim in _base.sim.stage.Traverse() if '/indicator/' in str(prim.GetPath()) and prim.GetAttribute('inputs:diffuseColor')]
 if _callback is None:
  from isaaclab_physx.physics import PhysxManager
  from isaaclab_physx.physics.physx_manager import IsaacEvents
  _callback=PhysxManager.register_callback(physics_step,IsaacEvents.POST_PHYSICS_STEP,order=100,name='utensil_heat_history',wrap_weak_ref=False)
 print('MONITOR reset hz',1/_base.physics_dt,'knob_degrees',np.degrees(array(_base.scene['knob'].root_view.get_dof_positions())[0]).tolist(),flush=True)
