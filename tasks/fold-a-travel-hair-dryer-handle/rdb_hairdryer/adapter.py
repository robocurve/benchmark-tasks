"""Free folding dryer with a native passive hinge and an attached loose cable."""
from dataclasses import replace
from pathlib import Path
import numpy as np
from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from . import fixed_scene as F,layout as L
from .grading import DryerGrade
CONTACT_CAPACITY=8192
CURRENT=None

def array(value):
    t=value.torch if hasattr(value,'torch') else value
    return t.detach().cpu().numpy().copy() if hasattr(t,'detach') else np.asarray(t).copy()

class DryerArena(YamArenaEmbodiment):
    def __init__(self,*,media_dir=None):
        global CURRENT
        super().__init__(scene='station',object_jitter=0,control_hz=F.CONTROL_HZ,cam_width=320,cam_height=176)
        self.info=replace(self.info,name='yam_hairdryer_fold',environment_id='isaaclab-arena-0.3.1-yam-hairdryer-fold',supported_target_kinds=frozenset(),docs='Three stock RGB cameras and 14 joint/jaw readings; Fold the travel hair dryer handle.')
        self.media_dir=Path(media_dir) if media_dir else None
        self.recorder=None;self.demo_state=None;self.final_grade=None;self.diagnostics=[];self.tick=0
        self._raw_sim_step=None;self._collect_physics=False;CURRENT=self
    def _ensure_sim(self,goal):
        if self._env is not None:return
        from .recording import review_camera
        def make_env(embodiment,*,device,control_hz,**kwargs):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene
            objects=scenes.task_room('station',embodiment.rig.top_cam)
            objects.append(Object(name='hairdryer',object_type=ObjectType.BASE,spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(F.USD))))
            def configure(cfg):
                cfg=scenes.env_cfg_callback(control_hz)(cfg);cfg.sim.dt=1/240;cfg.decimation=12;cfg.sim.render_interval=12;cfg.sim.physics.enable_ccd=True;return cfg
            env=IsaacLabArenaEnvironment(name='yam_hairdryer_fold',scene=Scene(objects),embodiment=embodiment,env_cfg_callback=configure)
            return ArenaEnvBuilder(env,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device=device)).make_registered()
        with review_camera():
            from yam_arena import scene as scenes
            original=scenes.make_env;scenes.make_env=make_env
            try:super()._ensure_sim(None)
            finally:scenes.make_env=original
    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths
        root='/World/envs/env_0';stage=omni.usd.get_context().get_stage()
        roots=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='Actors' and str(p.GetPath()).startswith(root+'/')]
        if len(roots)!=1:raise RuntimeError(f'Unexpected actor roots {roots}')
        self.physics=tensors.create_simulation_view('torch');self.physics.set_subspace_roots('/')
        self.body_view=self.physics.create_rigid_body_view(roots[0]+'/*')
        self.actor_paths=[roots[0]+'/'+n for n in L.LINKS]
        if list(self.body_view.prim_paths)!=self.actor_paths:raise RuntimeError('Unexpected dryer actor order')
        self.body_contacts=self.physics.create_rigid_contact_view(self.actor_paths,[self.actor_paths]*L.N,max_contact_data_count=CONTACT_CAPACITY)
        links=asset_link_paths();self.link_names=list(links)
        self.robot_links=[root+'/'+a.title()+'Arm'+p for a in F.ARMS for p in links.values()]
        self.native_tips=self.physics.create_rigid_body_view([root+'/RightArm'+links[n] for n in ('tip_left','tip_right')])
        self.native_grippers=self.physics.create_rigid_body_view([root+'/'+a.title()+'Arm'+links['gripper'] for a in F.ARMS])
        self.robot_tools=self.physics.create_rigid_contact_view(self.robot_links,[self.actor_paths]*len(self.robot_links),max_contact_data_count=CONTACT_CAPACITY)
        self.physics_dt=float(self._base.physics_dt)
    def _read(self):
        self.poses=array(self.body_view.get_transforms());self.poses[:,2]-=F.TOP
        self.velocities=array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):raise RuntimeError('Nonfinite native state')
    def contacts(self,poses):
        counts=array(self.body_contacts.get_contact_data(dt=self.physics_dt)[4]).reshape(L.N,L.N)
        forces=array(self.body_contacts.get_contact_force_matrix(dt=self.physics_dt)).reshape(L.N,L.N,3)
        rc=array(self.robot_tools.get_contact_data(dt=self.physics_dt)[4]).reshape(len(self.robot_links),L.N)
        if counts.sum()>=CONTACT_CAPACITY or rc.sum()>=CONTACT_CAPACITY:raise RuntimeError('Native contact recording capacity exhausted')
        return dict(body_contact_counts=counts.tolist(),body_contact_forces=forces.tolist(),robot_contact_counts=rc.tolist())
    def _publish(self):
        self.demo_state=dict(native_tips=array(self.native_tips.get_transforms()),link_names=self.link_names,native_grippers=array(self.native_grippers.get_transforms()),robot_forces=array(self.robot_tools.get_contact_force_matrix(dt=self.physics_dt)).reshape(len(self.robot_links),L.N,3),poses=self.poses.copy(),velocities=self.velocities.copy(),contacts=self.last_contacts.copy(),grade=self.final_grade)
    def reset(self,scene,*,seed=None):
        from .recording import Recorder
        if self.recorder is not None:self.recorder.close();self.recorder=None
        self._collect_physics=False;self._ensure_sim(None);self._physics_views()
        t=self._torch;pose=t.as_tensor(L.START_POSES,dtype=self.body_view.get_transforms().dtype,device=self._base.device).clone();pose[:,2]+=F.TOP
        ids=t.arange(L.N,dtype=t.int32,device=self._base.device)
        # Only actor-state writes: episode reset. No writes during step/policy.
        self.body_view.set_transforms(pose,ids);self.body_view.set_velocities(t.zeros_like(self.body_view.get_velocities()),ids)
        observation=super().reset(scene,seed=seed)
        self._read()
        drift=np.linalg.norm(self.poses[:,:3]-L.START_POSES[:,:3],axis=1)
        if drift[:2].max()>.04 or drift[2:].max()>.08:raise RuntimeError(f'Dryer reset drift {drift.tolist()} poses={self.poses.tolist()}')
        if not np.allclose(array(self.body_view.get_masses()).reshape(-1),L.MASSES,rtol=.01):raise RuntimeError('Mass mismatch')
        if set(observation.images)!={'top_cam','left_cam','right_cam'} or set(observation.state)!={'joint_pos'}:raise RuntimeError('Unexpected observation fields')
        self.grading=DryerGrade(sample_hz=1/self.physics_dt);self.final_grade=None;self.tick=0;self.diagnostics=[]
        self.last_contacts=self.contacts(self.poses);self._publish();self._install_sampler()
        self.recorder=Recorder(self,self.media_dir) if self.media_dir else None
        if self.recorder is not None:self.recorder.start()
        return observation
    def _install_sampler(self):
        if self._raw_sim_step is not None:return
        if getattr(self._base,'_physics_handles_decimation',False):raise RuntimeError('Scoring requires visible native substeps')
        self._raw_sim_step=self._base.sim.step
        def sampled(*args,**kwargs):
            result=self._raw_sim_step(*args,**kwargs)
            if self._collect_physics:
                p=array(self.body_view.get_transforms());p[:,2]-=F.TOP;v=array(self.body_view.get_velocities());contacts=self.contacts(p)
                self._latest=self.grading.update(p,v,**contacts);self._sample_count+=1
                if self.recorder is not None:self.recorder.physics_sample(p,v,contacts,self._latest)
            return result
        self._base.sim.step=sampled
    def step(self,action):
        self._sample_count=0;self._latest=None;self._collect_physics=True
        try:observation=super().step(action).observation
        finally:self._collect_physics=False
        if self._sample_count!=self._base.cfg.decimation:raise RuntimeError(f'Wrong physics sample count {self._sample_count}')
        self._read();self.tick+=1;self.last_contacts=self.contacts(self.poses);self.final_grade=self._latest;self._publish()
        if self.tick%5==0:self.diagnostics.append(dict(tick=self.tick,**self.final_grade,poses=self.poses.tolist(),velocities=self.velocities.tolist(),contacts=self.last_contacts))
        terminal=bool(self.final_grade['finished'])
        if self.recorder is not None:self.recorder.step(self.tick,observation,self.final_grade,terminal=terminal or self.tick==F.MAX_STEPS)
        return StepResult(observation=observation,terminated=terminal,termination_reason='success' if self.final_grade['score']==1. else self.final_grade['void'] if terminal else None,info={'dryer_grade':self.final_grade})
    def close(self):
        global CURRENT
        if self.recorder is not None:self.recorder.close();self.recorder=None
        if self._raw_sim_step is not None and self._base is not None:self._base.sim.step=self._raw_sim_step;self._raw_sim_step=None
        if CURRENT is self:CURRENT=None
        super().close()

class DryerScripted(PolicyBase):
    def bind(self,embodiment_info):pass
    def reset(self,scene):
        from .scripted import DryerReference
        self.reference=DryerReference();self.actions=self.reference.sequence()
    def act(self,observation):
        self.reference.observe(observation.state['joint_pos'],CURRENT.demo_state if CURRENT else None)
        return next(self.actions)
def make_embodiment(**kwargs):return DryerArena(**kwargs)
def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy
    p=DryerScripted();p.info=replace(YamScriptedPolicy().info,name='hairdryer_fold_scripted');return p
