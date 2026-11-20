"""Original Arena adapter; lazy simulator imports and read-only runtime observers.
Integration pattern informed by MIT PR2036; no pose writer or temporary attach.
"""
from dataclasses import dataclass,field
import importlib.abc,importlib.util,sys
from pathlib import Path
import numpy as np
from yam_arena.task_file import Box,ObjectState
import geometry as g
@dataclass(frozen=True)
class Item(Box):
 kind:str='card'
@dataclass(frozen=True)
class State(ObjectState):
 bodies:dict=field(default_factory=dict)
 joints:dict=field(default_factory=dict)
STEP=[];RESET=[];SPECS={}
def arr(v):
 v=getattr(v,'torch',v)
 return np.asarray(v.detach().cpu().numpy()if hasattr(v,'detach')else v,dtype=np.float32)
def spawn(path,cfg,translation=None,orientation=None,**kwargs):
 from isaaclab.sim.utils.stage import get_current_stage
 from build_usd import author
 return author(get_current_stage(),path,SPECS[cfg.item_name].kind,translation,orientation)
def asset(spec):
 from isaaclab.actuators import ImplicitActuatorCfg
 from isaaclab.assets import ArticulationCfg
 from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
 from isaaclab.sim.utils import clone
 from isaaclab.utils import configclass
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 @configclass
 class Spawner(SpawnerCfg):
  func:object=clone(spawn)
  item_name:str=''
 class Case(Object):
  def _generate_articulation_cfg(self):
   cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,
    actuators={'spring':ImplicitActuatorCfg(joint_names_expr=['follower_slide'],stiffness=g.SPRING_K,damping=.2)})
   cfg=self._add_initial_pose_to_cfg(cfg);cfg.init_state.joint_pos={'follower_slide':0.}
   return cfg
 cls=Case if spec.kind=='case'else Object
 return cls(name=spec.name,object_type=ObjectType.ARTICULATION if spec.kind=='case'else ObjectType.RIGID,
  spawner_cfg=Spawner(item_name=spec.name),initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=(0.,0.,0.,1.)))
def state(env,spec):
 scene=env.unwrapped.scene;a=scene[spec.name];origin=arr(scene.env_origins)[0]
 p=arr(a.data.root_link_pose_w)[0];v=arr(a.data.root_vel_w)[0]
 bodies={};joints={}
 if spec.kind=='case':
  for n,b in zip(a.body_names,arr(a.data.body_link_pose_w)[0]):bodies[n]={'pos':(b[:3]-origin).tolist(),'quat_xyzw':b[3:7].tolist()}
  joints=dict(zip(a.joint_names,map(float,arr(a.data.joint_pos)[0])))
 return State(p[:3]-origin,p[3:7],v[:3],v[3:6],p[:3]-origin-np.array(spec.size)/2,p[:3]-origin+np.array(spec.size)/2,bodies,joints)
def patch_scene(m):
 if getattr(m,'_cardcase_patch',False):return
 old_obj,old_states=m.task_object,m.object_states
 def obj(spec):
  if isinstance(spec,Item):SPECS[spec.name]=spec;return asset(spec)
  return old_obj(spec)
 def states(env,names):
  rest=[n for n in names if n not in SPECS];out=old_states(env,rest)if rest else{}
  out.update({n:state(env,SPECS[n])for n in names if n in SPECS});return {n:out[n]for n in names}
 m.task_object,m.object_states=obj,states;m._cardcase_patch=True
def patch_embodiment(m):
 cls=m.YamArenaEmbodiment
 if getattr(cls,'_cardcase_patch',False):return
 old_step,old_reset,old_ensure=cls._task_step,cls.reset,cls._ensure_sim
 def spring_target(environment):
  import torch
  for name,s in SPECS.items():
   if s.kind=='case':
    a=environment.unwrapped.scene[name]
    target=torch.zeros_like(getattr(a.data.joint_pos,'torch',a.data.joint_pos))+g.SPRING_REST
    a.set_joint_position_target(target)
 def step(self,*args,**kwargs):
  spring_target(self._env)
  for f in STEP:f(self._env)
  return old_step(self,*args,**kwargs)
 def ensure(self,*args,**kwargs):
  result=old_ensure(self,*args,**kwargs)
  if not getattr(self._env,'_cardcase_spring_reset',False):
   original=self._env.reset
   def environment_reset(*a,**kw):
    value=original(*a,**kw);spring_target(self._env);return value
   self._env.reset=environment_reset;self._env._cardcase_spring_reset=True
  return result
 def reset(self,*args,**kwargs):
  result=old_reset(self,*args,**kwargs)
  for f in RESET:f(self._env)
  return result
 cls._task_step,cls.reset,cls._ensure_sim=step,reset,ensure;cls._cardcase_patch=True

def patch_scripted(m):
 cls=m.YamScriptedPolicy
 if getattr(cls,'_cardcase_patch',False):return
 old_reset,old_act=cls.reset,cls.act
 def own(self):return self._task is not None and self._task.path.parent.resolve()==Path(__file__).resolve().parent
 def reset(self,scene):
  self._cardcase_active=own(self)
  if self._cardcase_active:
   if abs(self._dt-.05)>1e-8:raise ValueError('Card reference requires control_hz=20')
   self._cardcase_iterator=iter(self._task.demo());self._cardcase_last=None
  else:return old_reset(self,scene)
 def act(self,observation):
  if not getattr(self,'_cardcase_active',False):return old_act(self,observation)
  from inspect_robots.types import Action,ActionChunk
  stop=False
  try:self._cardcase_last=next(self._cardcase_iterator)
  except StopIteration:stop=True
  if self._cardcase_last is None:raise RuntimeError('Empty reference plan')
  return ActionChunk(actions=[Action(data=self._cardcase_last,meta={'request_stop':True}if stop else{})],control_hz=1./self._dt)
 cls.reset,cls.act=reset,act;cls._cardcase_patch=True

class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.inspect_embodiment':patch_embodiment,'yam_arena.scripted':patch_scripted}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if spec is None or spec.loader is None:return None
  old=spec.loader.exec_module;patch=self.targets[fullname]
  def execute(module):old(module);patch(module)
  spec.loader.exec_module=execute;return spec
for n,p in Finder.targets.items():
 if n in sys.modules:p(sys.modules[n])
sys.meta_path.insert(0,Finder())
