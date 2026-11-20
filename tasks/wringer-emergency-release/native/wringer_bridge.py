"""In-process adapter for one YAM task on Isaac Lab Arena. No installed files are edited."""
import atexit
import json
import os
from pathlib import Path
from unittest.mock import patch
import numpy as np
from measurements import Trial

def numpy_view(value):
    value=getattr(value,'torch',value)
    if hasattr(value,'detach'):value=value.detach().cpu()
    if hasattr(value,'numpy'):value=value.numpy()
    return np.asarray(value)

class Observer:
    def __init__(self,base,entry,seed):
        import omni.usd
        from pxr import Usd,UsdPhysics
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        self.base=base;self.dt=float(base.sim.get_physics_dt())
        self.decimation=round(float(base.step_dt)/self.dt)
        if self.decimation<1 or abs(self.decimation*self.dt-base.step_dt)>1e-6:raise ValueError('Unexpected YAM clock')
        self.trial=Trial(); self.tick=0; self.sub=0; self.release_seconds=0.
        self.bilateral_samples = {"LeftArm": 0, "RightArm": 0}
        self.lift_demand_N = 0.
        stage=omni.usd.get_context().get_stage()
        self.sv=PhysxManager.get_physics_sim_view()
        root='/World/envs/env_0/wringer_trainer'
        self.bodies={n:self.sv.create_rigid_body_view(root+'/'+n) for n in ('frame','yoke','arm','sleeve')}
        if any(v.count!=1 for v in self.bodies.values()):raise ValueError('Fixture mount or rigid link is unresolved')
        arm_roots=[str(base.scene[k].cfg.prim_path).replace('env_.*','env_0').replace('env_[^/]+','env_0') for k in ('left_robot','right_robot')]
        paths=asset_link_paths()
        self.hands={r+paths[k] for r in arm_roots for k in ('tip_left','tip_right','gripper')}
        self.links=sorted(str(p.GetPath()) for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.RigidBodyAPI) and any(str(p.GetPath()).startswith(r+'/') for r in arm_roots))
        if not self.hands.issubset(self.links) or len(self.hands)!=6:raise ValueError('Both YAM grippers must be identifiable')
        self.hand_views={p:self.sv.create_rigid_body_view(p) for p in sorted(self.hands)}
        if any(v.count!=1 for v in self.hand_views.values()):raise ValueError('Gripper diagnostic view is unresolved')
        self.contacts={}
        for n in self.bodies:
            v=self.sv.create_rigid_contact_view([root+'/'+n],filter_patterns=[self.links],max_contact_data_count=256)
            if v.filter_count!=len(self.links):raise ValueError('Invalid contact tensor filter count')
            self.contacts[n]=v
        log=os.environ.get('WRINGER_TRACE')
        self.fp=Path(log).open('x',encoding='utf8') if log else None
        self.record(dict(kind='reset',dt=self.dt,substeps=self.decimation,seed=seed,filters=self.links,hands=sorted(self.hands)))
        self.measure(initial=True)
    def record(self,row):
        if self.fp:self.fp.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
    def measure(self,initial=False):
        poses={}
        for n,v in self.bodies.items():
            a=numpy_view(v.get_transforms()).reshape(7)
            poses[n]=(a[:3].tolist(),a[3:].tolist())
        pairs={}
        for name,view in self.contacts.items():
            forces=numpy_view(view.get_contact_force_matrix(dt=self.dt)).reshape(-1,len(self.links),3)
            if not len(forces):raise ValueError('Empty contact tensor')
            if not np.isfinite(forces).all():raise ValueError('Nonfinite contact readings')
            magnitudes=np.linalg.norm(forces,axis=2).max(axis=0)
            pairs[name]={link:float(f) for link,f in zip(self.links,magnitudes) if f>.04}
        release=bool(self.hands.intersection(pairs['yoke']));arm=bool(self.hands.intersection(pairs['arm']))
        # Demonstrator bookkeeping from the existing link-filtered contact readings.
        # Each count belongs to the current action, even when an arm briefly loses contact.
        for side in self.bilateral_samples:
            if self.sub == 0:
                self.bilateral_samples[side] = 0
            tips = {p.rsplit('/', 1)[-1] for p in pairs['yoke'] if '/' + side + '/' in p}
            if {'tip_left', 'tip_right'} <= tips:
                self.bilateral_samples[side] += 1
        self.trial.observe(poses,self.dt,release,arm,any(pairs.values()))
        self.release_seconds=self.release_seconds+self.dt if release and self.trial.last['gap_mm']>=9 else 0.
        sample=dict(kind='sample',tick=self.tick,sub=self.sub,poses=poses,pairs=pairs,score=self.trial.score)
        if self.fp and self.sub==0:
            sample['hand_link_poses_m_xyzw']={p:numpy_view(v.get_transforms()).reshape(7).tolist() for p,v in self.hand_views.items()}
        self.record(sample)
        self.sub+=1
    def close(self):
        if self.fp and not self.fp.closed:self.fp.flush();self.fp.close()

def install(entry):
    from yam_arena import inspect_embodiment as bridge
    cls=bridge.YamArenaEmbodiment
    if getattr(cls,'_wringer_entry',None)==entry:return
    ensure,reset,step,close=cls._ensure_sim,cls.reset,cls.step,cls.close
    def selected(bot):return getattr(bot,'task',None) is not None and bot.task.path==entry
    def create(bot,goal):
        if not selected(bot) or bot._env is not None:return ensure(bot,goal)
        if bridge._APP is None:
            from isaaclab.app import AppLauncher
            bridge._APP=AppLauncher(headless=bot.headless,enable_cameras=True,device=bot.device).app
            atexit.register(bridge._close_app_at_exit)
        from yam_arena import scene
        import isaaclab.sim as sim
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        fallback=scene.task_object
        def spawn(marker):
            if marker.name!='wringer_trainer':return fallback(marker)
            return Object(name=marker.name,object_type=ObjectType.BASE,spawner_cfg=sim.UsdFileCfg(usd_path=str(entry.with_name('wringer.usda'))),
                          initial_pose=Pose(position_xyz=tuple(marker.pos),rotation_xyzw=(0.,0.,0.,1.)))
        with patch.object(scene,'task_object',spawn):return ensure(bot,goal)
    def start(bot,scene,*,seed=None):
        if not selected(bot):return reset(bot,scene,seed=seed)
        if getattr(bot,'_wringer',None) is not None:
            # Rebuild the compound. The arena only auto-resets top-level free objects.
            bot._wringer.close(); bot._env.close();bot._env=bot._base=None
        obs=reset(bot,scene,seed=seed)
        bot._wringer=Observer(bot._base,entry,seed)
        bot.task.module.monitor=bot._wringer
        return obs
    def advance(bot,action):
        if not selected(bot):return step(bot,action)
        from yam_arena import scene
        m=bot._wringer;m.tick+=1;m.sub=0
        engine=bot._base.sim;advance_physics=engine.step
        def instrument(*args,**kwargs):
            result=advance_physics(*args,**kwargs);m.measure();return result
        # The public task score reads the live compound-body tensor measurements.
        original_states=scene.object_states
        def states(env,names):
            if env is bot._env and tuple(names)==('wringer_trainer',):return {}
            return original_states(env,names)
        with patch.object(engine,'step',instrument), patch.object(scene,'object_states',states):
            result=step(bot,action)
        if m.sub!=m.decimation:raise RuntimeError(f'Missing physics substeps: expected {m.decimation}, measured {m.sub}')
        if m.fp:m.fp.flush()
        return result
    def finish(bot):
        if selected(bot) and getattr(bot,'_wringer',None):bot._wringer.close()
        return close(bot)
    cls._ensure_sim=create;cls.reset=start;cls.step=advance;cls.close=finish;cls._wringer_entry=entry
