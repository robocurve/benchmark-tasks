"""Native full-size keyboard tilt-foot setup."""
from dataclasses import replace
from pathlib import Path
import numpy as np
from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
from yam_arena.constants import PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from . import fixed_scene as F,layout as L
from .grading import KeyboardGrade
CURRENT=None


def array(value):
    tensor=value.torch if hasattr(value,'torch') else value
    return tensor.detach().cpu().numpy().copy() if hasattr(tensor,'detach') else np.asarray(tensor).copy()


class KeyboardArena(YamArenaEmbodiment):
    def __init__(self,*,media_dir=None):
        global CURRENT
        super().__init__(scene='station',device='cpu',object_jitter=0,control_hz=F.CONTROL_HZ,cam_width=320,cam_height=176)
        self.info=replace(self.info,name='yam_keyboard_feet',environment_id='isaaclab-arena-0.3.1-yam-keyboard-feet',
            supported_target_kinds=frozenset(),docs='Stock calibrated YAM station and keyboard and two passive flip feet. Three stock RGB cameras and 14 joint/jaw readings.')
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
            objects.append(Object(name='retrieval_fixture',object_type=ObjectType.BASE,spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(F.USD))))
            def configure(cfg):
                cfg=scenes.env_cfg_callback(control_hz)(cfg)
                # Use native CPU contact reporting against the stock static
                # tabletop's actual collision prim, with unchanged geometry.
                cfg.sim.device='cpu'
                cfg.sim.dt=1/L.PHYSICS_HZ
                cfg.decimation=round(L.PHYSICS_HZ/control_hz)
                cfg.sim.render_interval=cfg.decimation
                cfg.sim.physics.bounce_threshold_velocity=L.BOUNCE_THRESHOLD
                return cfg
            env=IsaacLabArenaEnvironment(name='yam_keyboard_feet',scene=Scene(objects),embodiment=embodiment,env_cfg_callback=configure)
            return ArenaEnvBuilder(env,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device=device)).make_registered()
        with review_camera():
            from yam_arena import scene as scenes
            stock=scenes.make_env;scenes.make_env=make_env
            try:super()._ensure_sim(None)
            finally:scenes.make_env=stock

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        import carb.settings
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        from yam_arena.robot import asset_link_paths
        root='/World/envs/env_0';stage=omni.usd.get_context().get_stage()
        roots=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='Actors' and str(p.GetPath()).startswith(root+'/')]
        if len(roots)!=1:raise RuntimeError(f'Expected one keyboard-feet fixture: {roots}')
        scene_prim=stage.GetPrimAtPath(self._base.cfg.sim.physics_prim_path)
        self.native_physics=dict(device=self._base.cfg.sim.device,gpu_dynamics=bool(scene_prim.GetAttribute('physxScene:enableGPUDynamics').Get()))
        if self.native_physics['gpu_dynamics']:raise RuntimeError('Desk-force filtering requires native CPU PhysX')
        from isaaclab.sim.utils.stage import get_current_stage_id
        self.physics=tensors.create_simulation_view('torch',stage_id=get_current_stage_id());self.physics.set_subspace_roots('/')
        self.actor_paths=[roots[0]+'/'+name for name in L.LINKS]
        self.body_view=self.physics.create_rigid_body_view(roots[0]+'/*')
        if list(self.body_view.prim_paths)!=self.actor_paths:raise RuntimeError(f'Unexpected body order {self.body_view.prim_paths}')
        links=asset_link_paths();self.link_names=list(links)
        self.robot_links=[root+'/'+a.title()+'Arm'+suffix for a in F.ARMS for suffix in links.values()]
        self.robot_task=self.physics.create_rigid_contact_view(self.robot_links,[self.actor_paths]*len(self.robot_links),max_contact_data_count=16384)
        table_path=root+'/table'
        if not stage.GetPrimAtPath(table_path):
            matches=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='table' and str(p.GetPath()).startswith(root+'/')]
            if len(matches)!=1:raise RuntimeError(f'Table collider not found: {matches}')
            table_path=matches[0]
        from pxr import Usd,UsdPhysics
        # Isaac Lab's stock cuboid spawner places collision on the mesh under
        # table/geometry, not the table Xform. Filtering that Xform silently
        # yielded no static-table forces, including the resting notebook.
        table_colliders=[str(p.GetPath()) for p in stage.Traverse(Usd.TraverseInstanceProxies())
            if str(p.GetPath()).startswith(table_path+'/') and p.HasAPI(UsdPhysics.CollisionAPI)]
        if len(table_colliders)!=1:raise RuntimeError(f'Expected one stock tabletop collider: {table_colliders}')
        self.native_physics['table_contact_filter']=table_colliders
        self.body_floor=self.physics.create_rigid_contact_view(self.actor_paths,[table_colliders]*len(L.LINKS),max_contact_data_count=8192)
        self.body_book=self.physics.create_rigid_contact_view(self.actor_paths[:3],[[self.actor_paths[3]]]*3,max_contact_data_count=1024)
        self.all_ids=self._torch.arange(len(L.LINKS),dtype=self._torch.int32,device=self._base.device)
        self.physics_dt=float(getattr(self._base,'physics_dt',0.) or PHYSICS_DT)

    def _read(self):
        self.poses=array(self.body_view.get_transforms());self.poses[:,2]-=F.TOP
        self.velocities=array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):raise RuntimeError('Nonfinite native retrieval state')

    def _publish_demo_state(self):
        self.demo_state=dict(poses=self.poses.copy(),velocities=self.velocities.copy(),robot_counts=self.contact_counts.copy(),angles=L.angles(self.poses),desk_counts=getattr(self,'desk_counts',np.zeros(4)).copy())

    def reset(self,scene,*,seed=None):
        from .recording import Recorder
        if self.recorder is not None:self.recorder.close();self.recorder=None
        self._collect_physics=False;self._ensure_sim(None);self._physics_views()
        torch=self._torch
        pose=torch.as_tensor(L.START_POSES,dtype=self.body_view.get_transforms().dtype,device=self._base.device).clone();pose[:,2]+=F.TOP
        ids=torch.arange(len(L.LINKS),dtype=torch.int32,device=self._base.device)
        # The only task-body writes are this episode reset.
        self.body_view.set_transforms(pose,ids);self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()),ids)
        observation=super().reset(scene,seed=seed)
        self._read();error=np.linalg.norm(self.poses[:,:3]-L.START_POSES[:,:3],axis=1)
        if error.max()>.006:raise RuntimeError(f'Body reset drift {error*1000} mm')
        if not np.allclose(array(self.body_view.get_masses()).reshape(-1),L.MASSES,rtol=.01):raise RuntimeError('Body mass mismatch')
        if set(observation.images)!={'top_cam','left_cam','right_cam'} or set(observation.state)!={'joint_pos'}:raise RuntimeError('Unexpected observation fields')
        self.grading=KeyboardGrade(sample_hz=1/self.physics_dt);self.final_grade=None;self.tick=0;self.diagnostics=[]
        self.contact_wrenches={}
        self.contact_counts=np.zeros((len(self.robot_links),len(L.LINKS)),dtype=int)
        self.joint_pos=np.asarray(observation.state['joint_pos'],dtype=float).copy()
        self._publish_demo_state();self._install_substep_sampler()
        self.recorder=Recorder(self,self.media_dir) if self.media_dir else None
        if self.recorder is not None:self.recorder.start()
        return observation

    def _install_substep_sampler(self):
        if self._raw_sim_step is not None:return
        if getattr(self._base,'_physics_handles_decimation',False):raise RuntimeError('Retrieval scoring requires individual substeps')
        self._raw_sim_step=self._base.sim.step
        def sampled(*args,**kwargs):
            result=self._raw_sim_step(*args,**kwargs)
            if self._collect_physics:
                poses=array(self.body_view.get_transforms());poses[:,2]-=F.TOP
                velocities=array(self.body_view.get_velocities())
                counts=array(self.robot_task.get_contact_data(dt=self.physics_dt)[4]).reshape(len(self.robot_links),len(L.LINKS))
                self._substep_counts=np.maximum(self._substep_counts,counts)
                loads=self._support_loads(poses)
                book_counts=array(self.body_book.get_contact_data(dt=self.physics_dt)[4])
                self.contact_wrenches=dict(support_loads_n=loads.tolist(),notebook_contacts=book_counts.tolist(),desk_contact_counts=self.desk_counts.tolist(),desk_forces_n=self.desk_forces.tolist())
                self._latest=self.grading.update(poses,velocities,robot_counts=counts,support_loads_n=loads,notebook_contacts=book_counts)
                if self.recorder is not None:self.recorder.physics_sample(poses,velocities,counts,self.contact_wrenches,self._latest)
                self._sample_count+=1
            return result
        self._base.sim.step=sampled

    def step(self,action):
        self.commanded_joint_pos=np.asarray(action.data,dtype=float).copy()
        self._substep_counts=np.zeros((len(self.robot_links),len(L.LINKS)),dtype=int);self._sample_count=0;self._latest=None;self._collect_physics=True
        try:observation=super().step(action).observation
        finally:self._collect_physics=False
        if self._sample_count!=self._base.cfg.decimation:raise RuntimeError(f'Wrong physics sample count {self._sample_count}')
        self._read();self.tick+=1;self.joint_pos=np.asarray(observation.state['joint_pos'],dtype=float).copy()
        if self.tick==1 and abs(self.desk_forces[3,2])<.5:
            raise RuntimeError('Actual initial notebook/table load was not reported by the native contact sensor')
        self.contact_counts=self._substep_counts.copy()
        self.final_grade=self._latest;self._publish_demo_state()
        if self.tick%5==0:self.diagnostics.append(self._diagnose())
        terminal=bool(self.final_grade['finished'])
        if self.recorder is not None:self.recorder.step(self.tick,observation,self.final_grade,terminal=terminal or self.tick==F.MAX_STEPS)
        return StepResult(observation=observation,terminated=terminal,
            termination_reason='success' if self.final_grade['score']==1. else self.final_grade['void'] if terminal else None,
            info={'keyboard_grade':self.final_grade})

    def _support_loads(self,poses):
        data=self.body_floor.get_contact_data(dt=self.physics_dt)
        force,points,normals,distances,counts,starts=[array(x) for x in data]
        counts=counts.reshape(len(L.LINKS));starts=starts.reshape(len(L.LINKS))
        self.desk_counts=counts.copy()
        total=array(self.body_floor.get_contact_force_matrix(dt=self.physics_dt)).reshape(len(L.LINKS),3)
        self.desk_forces=total.copy()
        loads=np.zeros(4);loads[:2]=np.abs(total[1:3,2])
        i=int(starts[0]);n=int(counts[0])
        if n:
            world=points[i:i+n].reshape(-1,3).copy();world[:,2]-=F.TOP
            local=(world-poses[0,:3])@L.quat_matrix(poses[0,3:])
            f=np.abs(force[i:i+n].reshape(-1)*normals[i:i+n].reshape(-1,3)[:,2])
            for j,side in enumerate((1,-1)):
                mask=(local[:,0]<-.050)&(side*local[:,1]>.150)
                loads[2+j]=float(f[mask].sum())
        return loads

    def _diagnose(self):
        from yam_arena import ik
        robots={}
        for i,a in enumerate(F.ARMS):
            p,r=ik.tcp_pose(self.joint_pos[i*7:i*7+6],F.BASES[a]);wanted,_=ik.tcp_pose(self.commanded_joint_pos[i*7:i*7+6],F.BASES[a])
            robots[a]=dict(tcp_pos=p.tolist(),tcp_rotation=r.tolist(),jaw=float(self.joint_pos[i*7+6]),commanded_tcp_pos=wanted.tolist())
        return dict(tick=self.tick,**self.final_grade,robot=robots,poses=self.poses.tolist(),velocities=self.velocities.tolist(),
            contact_counts=self.contact_counts.tolist(),contact_wrenches=self.contact_wrenches)

    def close(self):
        global CURRENT
        if self.recorder is not None:self.recorder.close();self.recorder=None
        if self._raw_sim_step is not None and self._base is not None:self._base.sim.step=self._raw_sim_step;self._raw_sim_step=None
        if CURRENT is self:CURRENT=None
        super().close()


class KeyboardScripted(PolicyBase):
    def bind(self,embodiment_info):pass
    def reset(self,scene):
        from .scripted import KeyboardReference
        self.reference=KeyboardReference();self.actions=self.reference.sequence()
    def act(self,observation):
        self.reference.observe(observation.state['joint_pos'],CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):return KeyboardArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy
    policy=KeyboardScripted();policy.info=replace(YamScriptedPolicy().info,name='keyboard_feet_scripted');return policy
