"""Mounted full-size cam nut: native contacting ramps, spring and thread flanks."""
from pathlib import Path
import json,numpy as np
NAME='anti-backlashnut-preload';HZ=20;MAX_STEPS=2400
from mechanism import ASSET,P,E,U,REAR_ZERO
INSTRUCTION=json.loads(Path(__file__).with_name('task.json').read_text())['instruction']
OBJECT_NAMES=('cam','rear')
ORIGIN=np.array([.30,0.,.7475+.09])
SETUP_CAMERA_EYE=(.86,.69,1.25)
SETUP_CAMERA_TARGET=(.30,0.,.865)

def make_environment(embodiment,device,control_hz=HZ):
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.scene.scene import Scene
 from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
 from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
 from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
 from isaaclab_arena.utils.pose import Pose
 from yam_arena.scene import task_room,env_cfg_callback
 objs=task_room('station',embodiment.rig.top_cam)
 # BASE intentionally spawns an original USD with independent native rigid
 # bodies. Soft thread limits are not supported on articulation joints.
 objs.append(Object(name='assembly',usd_path=str(ASSET/'assembly.usda'),object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(ORIGIN))))
 default=env_cfg_callback(control_hz)
 def cb(cfg):
  from isaaclab_physx.physics import PhysxCfg
  default(cfg);cfg.sim.dt=1/1000;cfg.decimation=50;cfg.sim.render_interval=50;cfg.sim.physics=PhysxCfg(enable_ccd=True,solver_type=1,enable_external_forces_every_iteration=False);cfg.sim.device='cpu'
  from isaaclab.sensors import ContactSensorCfg
  cfg.scene.task_contact_probe=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/assembly/cam',update_period=0.,history_length=0)
  return cfg
 cfg=IsaacLabArenaEnvironment(name='yam_cam_nut_preload',scene=Scene(objs),embodiment=embodiment,env_cfg_callback=cb)
 return ArenaEnvBuilder(cfg,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device='cpu')).make_registered()

def _observer(env):
 if not hasattr(env.unwrapped,'_cam_nut_observer'):
  from dynamics import Observer
  env.unwrapped._cam_nut_observer=Observer(env)
 return env.unwrapped._cam_nut_observer

def make_dynamics(env):
 _require_contact_reporting()
 return _observer(env)

def read_object_states(env):
 from yam_arena.task_file import ObjectState
 obs=_observer(env);obs.ensure_view();tf=obs.bodies.get_transforms();vel=obs.bodies.get_velocities();out={};poses={}
 for n in OBJECT_NAMES:
  i=obs.indices[n];p=np.asarray(tf[i,:3],float);q=np.asarray(tf[i,3:],float);v=np.asarray(vel[i],float)
  half=np.array([.0208,.0208,.04763 if n=='cam'else .018])/2
  centre=p+np.array([0,0,.0238 if n=='cam'else -.003])
  out[n]=ObjectState(centre,q,v[:3],v[3:],centre-half,centre+half);poses[n]=np.r_[p,q]
 return out,poses

def make_scorer():
 from scoring import Scorer
 return Scorer()

def demo():return {}

def _require_contact_reporting():
 from isaaclab.app.settings_manager import get_settings_manager
 if get_settings_manager().get('/physics/disableContactProcessing',None)is not False:
  raise RuntimeError('Native contact processing is disabled or unavailable; refusing silent zero-contact observations')
