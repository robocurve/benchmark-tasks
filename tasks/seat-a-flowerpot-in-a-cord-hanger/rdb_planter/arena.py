"""Native YAM station. All prop motion is produced by PhysX contact and free joints."""
from dataclasses import replace
from pathlib import Path

import numpy as np
from inspect_robots import StepResult
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena import ik
from . import fixed_scene as s
from .grading import measurements, Grader

def array(t):
    return getattr(t,'torch',t).detach().cpu().numpy().copy()

class PlanterStation(YamArenaEmbodiment):
    def __init__(self, out):
        super().__init__(scene='station',object_jitter=0.,control_hz=s.CONTROL_HZ,
                         cam_width=320,cam_height=176)
        self.info=replace(self.info,name='yam_planter',environment_id='arena-yam-planter',
                          supported_target_kinds=frozenset(),docs=s.INSTRUCTION)
        self.out=Path(out)
        self.tick=0;self.live=False;self.recorder=None;self.reset_once=False
    def _ensure_sim(self,goal):
        if self._env is not None:return
        from .recording import review_camera
        def make_env(embodiment, *, device, control_hz,**unused):
            import isaaclab.sim as su
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene
            objects=scenes.task_room('station',embodiment.rig.top_cam)
            objects.append(Object(name='training_fixture',object_type=ObjectType.BASE,
                                  spawner_cfg=su.UsdFileCfg(usd_path=str(s.USD))))
            env=IsaacLabArenaEnvironment(name='yam_planter',scene=Scene(objects),embodiment=embodiment,
                                         env_cfg_callback=scenes.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device=device)).make_registered()
        with review_camera():
            from yam_arena import scene as scenes
            stock=scenes.make_env;scenes.make_env=make_env
            try:super()._ensure_sim(None)
            finally:scenes.make_env=stock

    def views(self):
        import omni.physics.tensors as tensors
        from yam_arena.robot import asset_link_paths
        physics=tensors.create_simulation_view('torch');physics.set_subspace_roots('/')
        self.physics=physics;self.tips=[]
        paths=asset_link_paths()
        links={a:['/World/envs/env_0/'+a+'Arm'+v for v in paths.values()] for a in ('Left','Right')}
        self.arm_contacts=physics.create_rigid_contact_view(links['Left'],[links['Right']]*len(links['Left']))
        self.peak_interarm_contact_n=None
        self.interarm_samples=0
        for arm in ('Left','Right'):
            for link in ('tip_left','tip_right','gripper'):
                p='/World/envs/env_0/'+arm+'Arm'+paths[link]
                self.tips.append(physics.create_rigid_body_view(p))

        # PhysX requires one exact rigid body per filter and sensor.
        strands = [f'/World/envs/env_0/training_fixture/parts/cord_{i}_{j:02d}'
                   for i in range(4) for j in range(18)]
        self.cord_contacts = [physics.create_rigid_contact_view(
            ['/World/envs/env_0/'+arm+'Arm'+paths[t]], [strands])
            for arm in ('Left','Right') for t in ('tip_left','tip_right')]
        pot='/World/envs/env_0/training_fixture/parts/pot'
        self.pot_contacts=[physics.create_rigid_contact_view(
            ['/World/envs/env_0/RightArm'+paths[t]],[[pot]]) for t in ('tip_left','tip_right')]
        self.parts=physics.create_rigid_body_view('/World/envs/env_0/training_fixture/parts/*')
        self.rows={p.rsplit('/',1)[-1]:i for i,p in enumerate(self.parts.prim_paths)}
        self.pot_view=physics.create_rigid_body_view('/World/envs/env_0/training_fixture/parts/pot')

    def read(self):
        x=array(self.parts.get_transforms())
        self.poses={n:(x[i,:3],quat_xyzw_to_matrix(x[i,3:])) for n,i in self.rows.items()}
        self.cords=[[self.poses[f'cord_{i}_{j:02d}'][0] for j in range(18)] for i in range(4)]

    def section(self,z):
        points=[]
        for strand in self.cords:
            pts=np.asarray(strand)
            spans=np.flatnonzero((pts[:-1,2]>=z)&(pts[1:,2]<z))
            if len(spans)!=1:
                return None
            i=spans[0]
            t=(z-pts[i,2])/(pts[i+1,2]-pts[i,2])
            points.append(pts[i]+t*(pts[i+1]-pts[i]))
        return np.array(points)

    def reset(self,scene,*,seed=None):
        if self.reset_once:raise RuntimeError('Use a fresh process for each native trial')
        self.reset_once=True;self.grader=Grader()
        self._ensure_sim(None);self.views()
        ob=super().reset(scene,seed=seed);self.read()
        self.policy_input_shapes={key:list(np.asarray(rgb).shape) for key,rgb in ob.images.items()}
        self.policy_state_keys=sorted(ob.state)
        if (set(self.policy_input_shapes)!={'top_cam','left_cam','right_cam'}
                or any(shape!=[176,320,3] for shape in self.policy_input_shapes.values())
                or self.policy_state_keys!=['joint_pos'] or np.asarray(ob.state['joint_pos']).shape!=(14,)):
            raise ValueError('Unexpected YAM policy input at reset')
        m=self.metrics(ob)
        if m['seated'] or self.poses['pot'][0][2]<s.TOP-.015:
            raise RuntimeError(f'Invalid hanger reset: {m}')
        from .recording import Recorder
        self.recorder=Recorder(self,self.out/'media');self.recorder.start()
        return ob

    def metrics(self,ob):
        q=np.asarray(ob.state['joint_pos'])
        centers=[array(v.get_transforms())[0,:3] for v in self.tips]
        centers += [ik.tcp_pose(q[i:i+6],(0,.305*sgn,.75))[0] for i,sgn in [(0,1),(7,-1)]]
        return measurements(*self.poses['pot'],self.poses['bottom_knot'][0],self.cords,centers)

    def step(self,action):
        # Validation pulse is the only external force; the policy never applies it.
        import torch
        if self.grader.clear >= 6 and self.grader.pulse_steps < 3:
            sim = self._base.sim
            advance = sim.step
            dt = float(sim.get_physics_dt())
            count = 0
            force = torch.tensor([[.28, 0., 0.]], device=self._base.device)
            indices = torch.tensor([0], dtype=torch.int32, device=self._base.device)

            def advance_with_load(*args, **kwargs):
                nonlocal count
                self.pot_view.apply_forces_and_torques_at_position(force, None, None, indices, True)
                count += 1
                return advance(*args, **kwargs)

            sim.step = advance_with_load
            try:
                ob = super().step(action).observation
            finally:
                sim.step = advance
            if abs(count * dt - 1. / s.CONTROL_HZ) > 1e-5:
                raise RuntimeError(f'Invalid applied horizontal load duration: {count} physics steps at {dt} s')
            self.grader.pulse_steps += 1
        else:
            ob = super().step(action).observation
        self.tick+=1;self.read()
        forces=array(self.arm_contacts.get_contact_force_matrix(dt=float(self._base.sim.get_physics_dt())))
        if not forces.size or not np.isfinite(forces).all():
            raise ValueError('Missing native inter-arm contact data')
        peak=float(np.linalg.norm(forces,axis=-1).max())
        self.peak_interarm_contact_n=peak if self.peak_interarm_contact_n is None else max(peak,self.peak_interarm_contact_n)
        self.interarm_samples+=1
        self.final_grade=self.grader.update(self.metrics(ob),1./s.CONTROL_HZ)
        section=self.section(self.poses['bottom_knot'][0][2]+.115)
        dt=float(self._base.sim.get_physics_dt())
        cord_loads=[np.linalg.norm(array(view.get_contact_force_matrix(dt=dt)).reshape(-1,3),axis=1).tolist()
                    for view in self.cord_contacts]
        pot_forces=[float(np.linalg.norm(array(v.get_contact_force_matrix(dt=dt)).reshape(-1,3),axis=1).max()) for v in self.pot_contacts]
        pot_pos,pot_rotation=self.poses['pot']
        local_sections=[]
        for z in (.024,.05,.09):
            crossings=[]
            for k,strand in enumerate(self.cords):
                # Intersect each actual strand with a horizontal slice in pot coordinates.
                local=(np.asarray(strand)-pot_pos)@pot_rotation
                hi=local[:,2]-z
                cuts=np.flatnonzero(hi[:-1]*hi[1:]<0.)
                candidates=[]
                for i in cuts:
                    t=-hi[i]/(hi[i+1]-hi[i])
                    candidates.append(((1.-t)*local[i,:2]+t*local[i+1,:2]).tolist())
                crossings.append(candidates)
            local_sections.append(dict(height_m=z,cord_xy_m=crossings))
        self.final_grade.update(right_pad_pot_normal_n=pot_forces,pot_local_sections=local_sections, left_finger_cord_normals_n=cord_loads[:2],
                               right_finger_peak_by_cord_n=[np.round(np.max(np.reshape(a,(4,18)),axis=1),3).tolist() for a in cord_loads[2:]],cradle_section_m=section.tolist() if section is not None else None,
                                far_pair_width_m=float(np.linalg.norm(self.poses['cord_0_10'][0]-self.poses['cord_1_10'][0])),
                                pot_base_m=self.poses['pot'][0].tolist(),
                                cradle_knot_m=self.poses['bottom_knot'][0].tolist(),
                                grasped_strand_m=self.poses['cord_0_10'][0].tolist(),
                                peak_interarm_contact_n=self.peak_interarm_contact_n)
        if self.tick%8==0:
            from json import dumps
            if not hasattr(self,'wire_log'):
                self.wire_log=(self.out/'media'/'material-pose.jsonl').open('x',buffering=1)
            self.wire_log.write(dumps(dict(tick=self.tick,cords=np.asarray(self.cords).round(6).tolist(),
                pot_rotation=pot_rotation.round(6).tolist(),pot_base=pot_pos.tolist(),
                joints=np.asarray(ob.state['joint_pos']).tolist()))+'\n')
        self.recorder.step(self.tick,ob,self.final_grade,terminal=self.final_grade['score']==1. or self.tick>=s.MAX_STEPS)
        passed=self.final_grade['score']==1.
        return StepResult(observation=ob,terminated=passed,termination_reason='success' if passed else None,
                          info={'fit':self.final_grade})

    def close(self):
        self.live=False
        if hasattr(self,'wire_log'):self.wire_log.close()
        if self.recorder:self.recorder.close()
        super().close()

def make_embodiment(out=None):
    import tempfile
    from . import reference
    rig=PlanterStation(out or Path(tempfile.mkdtemp(prefix="planter-review-")))
    reference.ACTIVE=rig
    return rig
