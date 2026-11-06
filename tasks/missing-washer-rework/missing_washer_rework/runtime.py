# SPDX-License-Identifier: MIT
"""Pinned YAM runtime: ordinary rigid physics and 14 joint/gripper actions.

Construction hooks are scoped to initial scene creation. Measured object poses
are available only in the explicitly enabled demonstration observation stream.
"""
from __future__ import annotations
import atexit
import hashlib
import json
import math
import os
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from . import task_spec as spec
from .scoring import ReworkScorer, config_dict

def _numpy(value):
 value=value.torch if hasattr(value,'torch') else value
 return value.detach().cpu().numpy().copy()

def source_manifest():
 root=Path(__file__).parent
 files={str(f.relative_to(root)).replace('\\','/'):hashlib.sha256(f.read_bytes()).hexdigest()
        for f in sorted([*root.glob('*.py'),*root.glob('assets/*.usda')])}
 digest=hashlib.sha256()
 for name,checksum in files.items():digest.update(name.encode()+b'\0'+checksum.encode()+b'\n')
 return dict(schema='missing-washer-rework-source-v1',revision=digest.hexdigest(),files=files)

class MissingWasherReworkEmbodiment(YamArenaEmbodiment):
 def __init__(self,*,output_dir=None,demo_observations=False,terminate_on_success=True,**kwargs):
  self.demo_observations=bool(demo_observations)
  self.terminate_on_success=bool(terminate_on_success)
  kwargs.setdefault('task',str(Path(spec.__file__).resolve()))
  kwargs.setdefault('object_jitter',0.)
  super().__init__(**kwargs)
  self._manifest=source_manifest()
  self.info=replace(self.info,environment_revision=self._manifest['revision'])
  self.output_dir=Path(output_dir).resolve() if output_dir else None
  self._trial=-1;self._records=[];self._pending=[];self._metrics={'score':0.,'success':False}
  self._scorer=ReworkScorer()
  self._physical_settings={}

 def _ensure_sim(self,goal):
  if self._env is not None and goal==self._goal:return
  import yam_arena.inspect_embodiment as upstream
  if upstream._APP is None:
   from isaaclab.app import AppLauncher
   os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
   options=dict(headless=self.headless,device=self.device,enable_cameras=True)
   if not self.headless:
    options['visualizer']=['kit'];os.environ['DISPLAY']=upstream.window_display(os.environ)
   upstream._APP=AppLauncher(**options).app
   atexit.register(upstream._close_app_at_exit)
  import isaaclab.sim as sim_utils
  import yam_arena.scene as scene_module
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg
  original_object=scene_module.task_object
  original_callback=scene_module.env_cfg_callback
  def task_object(obj):
   if obj.name not in spec.COMPOUND_ASSETS:return original_object(obj)
   cfg=sim_utils.UsdFileCfg(usd_path=spec.COMPOUND_ASSETS[obj.name])
   if obj.name in spec.FIXED_ASSETS:
    cfg.rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
   return Object(name=obj.name,prim_path='{ENV_REGEX_NS}/'+obj.name,
    object_type=ObjectType.RIGID,spawner_cfg=cfg,
    initial_pose=Pose(position_xyz=tuple(obj.pos),rotation_xyzw=(0,0,math.sin(obj.yaw/2),math.cos(obj.yaw/2))))
  def callback(control_hz=None):
   base_callback=original_callback(control_hz)
   def configure(cfg):
    cfg=base_callback(cfg)
    cfg.sim.physics.solver_type=0
    cfg.sim.physics.min_position_iteration_count=spec.POSITION_ITERATIONS
    cfg.sim.physics.min_velocity_iteration_count=spec.VELOCITY_ITERATIONS
    return cfg
   return configure
  with ExitStack() as stack:
   stack.enter_context(patch.object(scene_module,'task_object',task_object))
   stack.enter_context(patch.object(scene_module,'env_cfg_callback',callback))
   super()._ensure_sim(goal)
  stage=self._base.sim.stage
  physics=stage.GetPrimAtPath(self._base.cfg.sim.physics_prim_path)
  self._physical_settings=dict(
   solver=physics.GetAttribute('physxScene:solverType').Get(),
   min_position_iterations=physics.GetAttribute('physxScene:minPositionIterationCount').Get(),
   min_velocity_iterations=physics.GetAttribute('physxScene:minVelocityIterationCount').Get(),
   rigid_bodies={})
  expected=(spec.PHYSICS_SOLVER,spec.POSITION_ITERATIONS,spec.VELOCITY_ITERATIONS)
  actual=tuple(self._physical_settings[k] for k in ('solver','min_position_iterations','min_velocity_iterations'))
  if actual!=expected:raise RuntimeError(f'Unexpected physics solver readback: {actual}, expected {expected}')
  for name in (*spec.DYNAMIC_NAMES,*spec.REFERENCE_NAMES):
   body=stage.GetPrimAtPath(self._base.scene.env_prim_paths[0]+'/'+name)
   counts=dict(position_iterations=body.GetAttribute('physxRigidBody:solverPositionIterationCount').Get(),
               velocity_iterations=body.GetAttribute('physxRigidBody:solverVelocityIterationCount').Get())
   if tuple(counts.values())!=expected[1:]:raise RuntimeError(f'Unexpected {name} solver iterations: {counts}')
   self._physical_settings['rigid_bodies'][name]=counts

 def _pose(self,name):return _numpy(self._base.scene[name].data.root_link_pose_w)[0]

 def _observation(self):
  observation=super()._observation()
  if self.demo_observations:
   tools={name+'_pose':self._pose(name).tolist() for name in spec.DYNAMIC_NAMES}
   return replace(observation,extra={**observation.extra,'demo_tools':tools})
  return observation

 def _measure(self,observation):
  from inspect_robots_yam.packing import STATE_KEY,split
  from yam_arena import ik,station
  joints=np.asarray(observation.state[STATE_KEY],dtype=float)
  left,right=split(joints)
  tcp={}
  for arm,q in [('left',left),('right',right)]:
   pos,rot=ik.tcp_pose(q[:6],station.BASES[arm])
   tcp[arm]=[*pos.tolist(),*ik.matrix_to_quat_xyzw(rot).tolist()]
  names=(*spec.DYNAMIC_NAMES,*spec.REFERENCE_NAMES)
  record=dict(t=float(self._t),step=len(self._records),
   poses={name:self._pose(name).tolist() for name in names},
   velocities={name:_numpy(self._base.scene[name].data.root_com_vel_w)[0].tolist() for name in spec.DYNAMIC_NAMES},
   tcp=tcp,grippers=dict(left=float(left[6]),right=float(right[6])),joints=joints.tolist())
  if not all(np.isfinite(record['poses'][n]).all() for n in names):
   raise RuntimeError('Nonfinite measured object pose')
  self._records.append(record);self._pending.append(record)
  self._metrics=self._scorer.update(record)
  return record

 def reset(self,scene,*,seed=None):
  if self._trial>=0:self._export()
  self._trial+=1
  if self.output_dir:
   while (self.output_dir/f'trial-{self._trial:04d}').exists():self._trial+=1
  self._records=[];self._pending=[];self._scorer=ReworkScorer()
  self._metrics={'score':0.,'success':False}
  observation=super().reset(scene,seed=seed)
  self._measure(observation)
  self._export(label='setup')
  return observation

 def _task_step(self):
  from inspect_robots.types import StepResult
  observation=self._observation();self._measure(observation)
  self._flush();self._write_metrics()
  done=bool(self._metrics['success']) and self.terminate_on_success
  return StepResult(observation=observation,terminated=done,
   termination_reason='success' if done else None,info=dict(self._metrics))

 def _trial_path(self,name):
  folder=self.output_dir/f'trial-{self._trial:04d}';folder.mkdir(parents=True,exist_ok=True)
  return folder/name

 def _flush(self):
  if self.output_dir and self._pending:
   with self._trial_path('measurements.jsonl').open('a',encoding='utf-8') as stream:
    for record in self._pending:stream.write(json.dumps(record,allow_nan=False)+'\n')
  self._pending.clear()

 def _write_metrics(self):
  if self.output_dir:self._trial_path('metrics.json').write_text(json.dumps(self._metrics,indent=2,allow_nan=False)+'\n',encoding='utf-8')

 def _export(self,label='result'):
  self._flush();self._write_metrics()
  if not self.output_dir:return
  self._trial_path('source_manifest.json').write_text(json.dumps(self._manifest,indent=2)+'\n',encoding='utf-8')
  config=dict(schema=spec.TRACE_SCHEMA,demo_observations=self.demo_observations,
   terminate_on_success=self.terminate_on_success,control_hz=self.info.control_hz,
   environment_revision=self.info.environment_revision,object_jitter=self.object_jitter,thresholds=config_dict(),physics=self._physical_settings)
  self._trial_path('run_config.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
  if self._base is not None:
   from PIL import Image
   for name in ('top_cam','left_cam','right_cam'):
    rgb=_numpy(self._base.scene[name].data.output['rgb'])[0,...,:3]
    Image.fromarray(rgb.astype(np.uint8)).save(self._trial_path(label+'_'+name+'.png'))

 def close(self):
  if self._trial>=0:self._export()
  super().close()

def make_embodiment(**kwargs):return MissingWasherReworkEmbodiment(**kwargs)
