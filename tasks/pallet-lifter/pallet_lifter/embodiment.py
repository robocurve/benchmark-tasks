"""Task-scoped mesh spawning and physical contact reports."""
from pathlib import Path
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
DEMO_STATES={}
class PalletEmbodiment(YamArenaEmbodiment):
 def _ensure_sim(self,goal):
  import os,atexit
  from yam_arena import inspect_embodiment as base
  if base._APP is None:
   from isaaclab.app import AppLauncher
   os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
   base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app
   atexit.register(base._close_app_at_exit)
  from yam_arena import scene
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  import isaaclab.sim as sim
  original=scene.task_object
  def make_object(spec):
   if spec.name in ('lifter','pallet','rests'):
    addon={'physics_material':scene._grippy(),'collision_props':sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.)}
    if spec.free:addon.update(rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=32,solver_velocity_iteration_count=4),mass_props={'':[sim.MassCfg(mass=spec.mass)]},mass_props_create_if_missing=True)
    return Object(name=spec.name,usd_path=str(Path(__file__).resolve().parent.parent/'assets'/(spec.name+'.usdc')),object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(spec.pos)),spawn_cfg_addon=addon)
   obj=original(spec)
   if spec.name=='pallet':
    obj.spawner_cfg.activate_contact_sensors=True;obj.object_cfg=obj._init_object_cfg()
   return obj
  scene.task_object=make_object
  try:super()._ensure_sim(goal)
  finally:scene.task_object=original
  self._base.sim.set_setting('/physics/disableContactProcessing',False)
  if not hasattr(self,'_contacts_subscription'):
   import omni.physx
   self._contacts_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._contacts)
 def _contacts(self,headers,data):
  from pxr import PhysicsSchemaTools
  for h in headers:
   if h.num_contact_data:self.task.module.HISTORY.contact(str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
 def reset(self,scene,*,seed=None):
  from pallet_lifter.scoring import History
  self.task.module.HISTORY=History();obs=super().reset(scene,seed=seed)
  self._base.sim.set_setting('/physics/disableContactProcessing',False);self._states();return obs
 def _states(self):
  from yam_arena.scene import object_states
  global DEMO_STATES
  DEMO_STATES=object_states(self._env,self.task.names);return DEMO_STATES
 def _task_step(self):
  from yam_arena import ik,station
  from yam_arena.constants import FINGER_OPEN
  states=self._states();h=self.task.module.HISTORY;h.update(states,self.task.module.TOP)
  clear=True
  for side,key in (('left','left_robot'),('right','right_robot')):
   arm_ids,finger_ids=self._ids[key];q=self._base.scene[key].data.joint_pos.torch[0]
   tcp,_=ik.tcp_pose(q[arm_ids].cpu().numpy(),station.BASES[side])
   clear &= q[finger_ids].mean().item()/FINGER_OPEN>.75 and all(np.linalg.norm(tcp-states[k].pos)>.09 for k in ('lifter','pallet'))
  h.open_clear=bool(clear);result=super()._task_step()
  if round(self._t*self.info.control_hz)%20==0:
   print('PALLET_STATE',round(self._t,2),{k:np.round(s.pos,4).tolist() for k,s in states.items() if k in ('lifter','pallet')},'lifter_quat',states['lifter'].quat_xyzw.tolist(),'pallet_quat',states['pallet'].quat_xyzw.tolist(),'history',str(vars(h)),'score',result.info.get('score'),flush=True)
  return result

def make_embodiment(**kwargs):return PalletEmbodiment(**kwargs)
