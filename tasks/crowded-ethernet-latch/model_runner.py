"""Real Dream or clock-only policy on the native Ethernet scene; no bench forces."""
import argparse,atexit,hashlib,json,os,shutil
from pathlib import Path


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--policy',choices=('dream','recorded'),default='dream');p.add_argument('--out',type=Path,required=True);p.add_argument('--max-steps',type=int,default=2400);p.add_argument('--state-only',action='store_true');a=p.parse_args()
 if not 1<=a.max_steps<=2400:p.error('max-steps must be1..2400')
 if a.state_only and a.policy!='recorded':p.error('Dream requires real RGB camera observations')
 out=a.out.expanduser().resolve();out.mkdir(parents=True,exist_ok=False);(out/'source').mkdir()
 for src in [*Path(__file__).parent.glob('*.py'),*Path(__file__).parent.glob('*.json')]:shutil.copy2(src,out/'source'/src.name)
 (out/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (out/'source').iterdir()},indent=2)+'\n')
 (out/'mode.json').write_text(json.dumps(dict(policy=a.policy,state_only=a.state_only,external_bench_forces=False,normal_reset=True,native_rgb=not a.state_only,offline_replay_requires_trace_audit=True),indent=2)+'\n')
 from inspect_robots.cli import main as cli
 from inspect_robots.registry import register
 from yam_arena.inspect_embodiment import YamArenaEmbodiment
 from native_episode import NativeEpisode,joint_state
 import numpy as np

 class NativeEthernet(YamArenaEmbodiment):
  def __init__(self,*args,**kwargs):
   super().__init__(*args,**kwargs)
   if a.state_only:
    from dataclasses import replace
    from inspect_robots_yam.config import observation_space
    self.info=replace(self.info,observation_space=observation_space(self.cam_height,self.cam_width,()))
  def _ensure_sim(self,goal):
   if self._env is not None:return
   from yam_arena import inspect_embodiment as adapter
   if adapter._APP is None:
    from isaaclab.app import AppLauncher
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
    adapter._APP=AppLauncher(headless=True,device=self.device,enable_cameras=not a.state_only).app
    atexit.register(adapter._close_app_at_exit)
   from yam_arena import scene as sc
   import isaaclab.sim as sim
   from yam_arena.embodiment import BimanualYamEmbodiment
   from native import export,install_joints
   from config import USED,LINKS
   assets={s.name:export(out/(s.name+'.usda'),s.name) for s in self.task.module.OBJECTS if s.name!='mat'};original=sc.task_object;original_init=BimanualYamEmbodiment.__init__
   def make(spec):
    if spec.name not in assets:return original(spec)
    cfg=sim.UsdFileCfg(usd_path=assets[spec.name])
    if spec.name==f'wire_{USED[-1]}_{LINKS-1}':
     spawn=cfg.func
     def joints(path,cfg,translation=None,orientation=None,**kw):
      prim=spawn(path,cfg,translation=translation,orientation=orientation,**kw);install_joints(prim.GetStage(),str(prim.GetPath()).rsplit('/',1)[0]);return prim
     cfg.func=joints
    return sc.Object(name=spec.name,object_type=sc.ObjectType.BASE if spec.name=='switch' else sc.ObjectType.RIGID,spawner_cfg=cfg,initial_pose=sc.Pose(position_xyz=spec.pos,rotation_xyzw=(0.,0.,0.,1.)))
   def init(instance,*args,**kwargs):kwargs['enable_cameras']=not a.state_only;original_init(instance,*args,**kwargs)
   sc.task_object=make;BimanualYamEmbodiment.__init__=init
   try:super()._ensure_sim(goal)
   finally:sc.task_object=original;BimanualYamEmbodiment.__init__=original_init
   from physics_manifest import manifest
   from yam_arena import YAM_URDF
   physical=manifest(self._base.scene.stage,assets,YAM_URDF,sc.PHYSICS_DT,20)
   (out/'physics-manifest.json').write_text(json.dumps(physical,indent=2)+'\n')
   if a.state_only:self._base.sim.render=lambda *args,**kwargs:None
  def reset(self,scene,*,seed=None):
   if hasattr(self,'episode'):raise RuntimeError('one fresh episode per process')
   self._ensure_sim(None);self._env.reset();self._instruction=scene.instruction;self._t=0.;self._held=0;self._best_score=0.
   self.episode=NativeEpisode(self,out);self.task.module._tracker=self.episode.scorer;self.task.module._diagnostic=False
   return self._observation()
  def _observation(self):
   if not a.state_only:return super()._observation()
   from inspect_robots.types import Observation
   from inspect_robots_yam.packing import STATE_KEY
   return Observation(images={},state={STATE_KEY:joint_state(self)},instruction=self._instruction,image_times={},state_time=self._t)
  def step(self,action):
   self.applied=np.asarray(np.clip(np.asarray(action.data,dtype=float).reshape(-1),self._low,self._high),dtype=np.float32)
   return super().step(action)
  def _task_step(self):
   from inspect_robots.types import StepResult
   row=self.episode.step(self.applied);scorer=self.episode.scorer
   # Scorer-side tug check (the policy never applies forces): a constant pull on the
   # orange boot along its withdrawal axis while the tug window is open.
   import torch
   from scoring import TUG_N
   from yam_arena.urdf import quat_xyzw_to_matrix
   plug=np.asarray(row['poses']['plug_0']);axis=-quat_xyzw_to_matrix(plug[3:])[:,0]
   f=axis*TUG_N if scorer.tug_active else np.zeros(3)
   self._base.scene['plug_0'].permanent_wrench_composer.set_forces_and_torques_index(forces=torch.as_tensor(f[None,None,:],device=self._base.device,dtype=torch.float32),torques=torch.zeros((1,1,3),device=self._base.device,dtype=torch.float32),is_global=True)
   row['tug_force_N']=f.tolist()
   success=scorer.finished(self._t)
   return StepResult(observation=self._observation(),terminated=success,termination_reason='success' if success else None,info={'score':row['metrics']['score'],'success':success})
  def close(self):
   if hasattr(self,'episode'):self.episode.close()
   super().close()

 register('embodiment','crowded_ethernet_native')(NativeEthernet)
 policy='dream'
 if a.policy=='recorded':
  from clock_policy import register_policy
  policy=register_policy()
 instruction='Press the orange plug clip, withdraw it from the lower marked port, insert and latch it in the upper marked empty port, then release both hands. Keep the six other cables connected.'
 return cli(['run','--embodiment','crowded_ethernet_native','--policy',policy,'-E',f'task={Path(__file__).with_name("task.py")}','-E','object_jitter=0','-E','control_hz=20','-E','cam_width=320','-E','cam_height=176','--instruction',instruction,'--scorer','task_score','--max-steps',str(a.max_steps),'--seed','0','--no-prompt','--grader','none',*([] if a.state_only else ['--store-frames']),'--log-dir',str(out)])

if __name__=='__main__':main()
