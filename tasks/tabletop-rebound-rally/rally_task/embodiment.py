"""Native ball contacts and the stock calibrated station, including its table and cameras."""
from pathlib import Path
from dataclasses import replace
import hashlib,math,numpy as np
from yam_arena import ik,station
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import *
from .scoring import apply_contacts,score
ROOT=Path(__file__).resolve().parents[1];FEEDBACK={}
class RallyEmbodiment(YamArenaEmbodiment):
 def __init__(self,**kw):
  for k,v in dict(task=str(ROOT/'task.py'),device='cpu',control_hz=20,cam_width=640,cam_height=360,object_jitter=0.).items():kw.setdefault(k,v)
  super().__init__(**kw);assert self.rig==station.rig('calibrated')
  self.info=replace(self.info,name='yam_tabletop_rally',environment_revision=hashlib.sha256(b''.join(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes() for p in sorted(ROOT.rglob('*')) if p.is_file() and (p.suffix in ('.py','.usda','.png','.toml') or p.name=='physical_design.json'))).hexdigest());self.active=False;self.trace=[];self.score=0.
 def _ensure_sim(self,goal):
  if self._env is not None:return
  from yam_arena import inspect_embodiment as base
  if base._APP is None:
   import atexit
   from isaaclab.app import AppLauncher
   base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app;atexit.register(base._close_app_at_exit)
  from yam_arena import scene,embodiment as robots
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  import isaaclab.sim as sim
  old_spawn,old_cb,old_robot=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment
  def spawn(spec):
   mat=sim.RigidBodyMaterialCfg(static_friction=.12,dynamic_friction=.10,restitution=.9,restitution_combine_mode='max',friction_combine_mode='min')
   if spec.name=='paddle':return Object(name='paddle',usd_path=str(ROOT/'assets/paddle.usda'),object_type=ObjectType.RIGID,initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=tuple(map(float,ik.matrix_to_quat_xyzw(PADDLE_R)))),spawn_cfg_addon=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=8,max_depenetration_velocity=.3),physics_material=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.6,restitution=.9)))
   if spec.name=='ball':return Object(name='ball',usd_path=str(ROOT/'assets/ball.usda'),object_type=ObjectType.RIGID,initial_pose=Pose(position_xyz=spec.pos),spawn_cfg_addon=dict(rigid_props=sim.RigidBodyPropertiesCfg(kinematic_enabled=True,solver_position_iteration_count=64,solver_velocity_iteration_count=8,max_depenetration_velocity=.3),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),physics_material=mat))
   if spec.name=='board':return Object(name='board',object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=spec.pos),spawner_cfg=sim.CuboidCfg(size=spec.size,collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.6,restitution=.9),visual_material=sim.PreviewSurfaceCfg(diffuse_color=spec.color)))
   return old_spawn(spec)
  def callback(hz=None):
   parent=old_cb(hz)
   def cb(cfg):
    cfg=parent(cfg);cfg.sim.dt=1/480;cfg.decimation=24;cfg.sim.render_interval=24;cfg.sim.physics.enable_ccd=True;cfg.episode_length_s=35.;return cfg
   return cb
  class ReadyRobot(old_robot):
   def __init__(robot,**opts):
    opts['robot_contact_offset']=.0002;super().__init__(**opts);cfg=robot.scene_config.left_robot;cfg.init_state.joint_pos={**{f'joint{i+1}':float(v) for i,v in enumerate(Q0)},'joint[7-8]':-.014}
  scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=spawn,callback,ReadyRobot
  try:super()._ensure_sim(goal)
  finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=old_spawn,old_cb,old_robot
  import omni.physx
  from pxr import PhysxSchema
  self.native=omni.physx.get_physx_interface();self.physics=omni.physx.get_physx_simulation_interface();self._base.sim.set_setting('/physics/disableContactProcessing',False);self.subscription=self.physics.subscribe_full_contact_report_events(lambda *args:None)
  for n in ('ball','paddle'):PhysxSchema.PhysxContactReportAPI.Apply(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n)).CreateThresholdAttr(0.)
  raw=self._base.sim.step
  def tick(*args,**kw):
   ans=raw(*args,**kw)
   if self.active:self.observe()
   return ans
  self._base.sim.step=tick
 def pose(self,n):
  v=self.native.get_rigidbody_transformation('/World/envs/env_0/'+n);return np.array(v['position']),quat_xyzw_to_matrix(np.array(v['rotation']))
 def observe(self):
  from pxr import PhysicsSchemaTools,UsdPhysics
  self.tick+=1;t=self.tick/480
  if not self.launched and t>=2.:
   UsdPhysics.RigidBodyAPI(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/ball')).GetKinematicEnabledAttr().Set(False);self._base.scene['ball'].write_root_velocity_to_sim_index(root_velocity=self._torch.tensor([[-1.2,0.,-.5,0.,0.,0.]],device=self._base.device));self.launched=True;self.last_ball=None
  p,R=self.pose('ball')
  if self.last_ball is not None:self.velocity=(p-self.last_ball)*480
  self.last_ball=p;self.ball=p
  if not self.launched or self.done:return
  headers,contacts,_=self.physics.get_full_contact_report();pairs={};contact_details={};blade_contacts=[]
  for h in headers:
   if not h.num_contact_data:continue
   names=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.actor0,h.actor1)]
   if any(n.endswith('/paddle') for n in names):
    for tip in self.grip:
     if any('/LeftArm/' in n and n.endswith('/'+tip) for n in names):self.grip[tip]=self.tick
   if not any(n.endswith('/ball') for n in names):continue
   other=next(n for n in names if not n.endswith('/ball'));kind='table' if '/table/' in other or other.endswith('/table') else ('wall' if '/board/' in other or other.endswith('/board') else ('paddle' if other.endswith('/paddle') else 'other'))
   pairs.setdefault(kind,[]).append(other)
   contact_details.setdefault(kind,[]).append(dict(actors=names,colliders=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.collider0,h.collider1)],points=[dict(position=list(c.position),normal=list(c.normal),separation=float(c.separation),impulse=str(c.impulse)) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]]))
   if kind=='paddle':blade_contacts.append(any(str(PhysicsSchemaTools.intToSdfPath(int(v))).rsplit('/',1)[-1] in ('Blade','RubberFront','RubberBack') for v in (h.collider0,h.collider1)))
  entered=set(pairs)-self.previous
  for kind in sorted(entered):
   self.events.append(dict(t=t,event=kind,ball=p.tolist(),velocity=self.velocity.tolist(),grip_age={n:self.tick-v for n,v in self.grip.items()},phase_before=self.phase,contacts=contact_details.get(kind,[])))
  robot_contact=any('/LeftArm/' in actor or '/RightArm/' in actor for actor in pairs.get('other',[]))
  apply_contacts(self,entered,p,table=TABLE,board=BOARD,robot_contact=robot_contact,blade=bool(blade_contacts) and all(blade_contacts),held=all(self.tick-v<=2 for v in self.grip.values()))
  self.previous=set(pairs)
  if p[2]<TABLE-.05 or p[0]<-.1 or abs(p[1])>.62:self.done=True
  self.score=score(self)
 def reset(self,scene,*,seed=None):
  self.active=False;self._ensure_sim(None)
  from pxr import UsdPhysics
  UsdPhysics.RigidBodyAPI(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/ball')).GetKinematicEnabledAttr().Set(True)
  self._ready=self._torch.tensor([[*Q0,.10,*self._ready[0,7:].cpu().numpy()]],device=self._base.device,dtype=self._torch.float32);obs=super().reset(scene,seed=seed)
  self.tick=0;self.launched=False;self.last_ball=None;self.velocity=np.zeros(3);self.ball=np.array([.68,.25,1.0]);self.grip={n:-10000 for n in ('tip_left','tip_right')};self.previous=set();self.phase='table';self.hits=0;self.cycles=0;self.invalid=False;self.done=False;self.score=0.;self.events=[];self.trace=[];FEEDBACK.clear();self.active=True
  from runtime_audit import write_reset
  write_reset(self,obs,['paddle', 'ball'])
  return obs
 def close(self):
  from runtime_audit import write_episode
  write_episode(self)
  return super().close()
 def _task_step(self):
  from inspect_robots.types import StepResult
  paddle,PR=self.pose('paddle');omega=self._base.scene['ball'].data.root_ang_vel_w.torch[0].cpu().numpy();row=dict(t=self._t,score=self.score,phase=self.phase,hits=self.hits,cycles=self.cycles,invalid=self.invalid,done=self.done,ball=self.ball.tolist(),velocity=self.velocity.tolist(),angular_velocity=omega.tolist(),paddle=paddle.tolist(),paddle_rotation=PR.tolist(),grip_age={n:self.tick-v for n,v in self.grip.items()});self.trace.append(row);FEEDBACK.clear();FEEDBACK.update(row)
  return StepResult(observation=self._observation(),terminated=self.done,termination_reason='success' if self.score==1. else ('miss' if self.done else None),info={'score':self.score,'success':self.score==1.})
def make_embodiment(**kw):return RallyEmbodiment(**kw)
