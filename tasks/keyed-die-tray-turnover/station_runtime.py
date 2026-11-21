"""Install the free jig, private native scorer and optional image-gated demonstration."""
import atexit
import json
import numpy as np
from layout import PHYSICS_HZ


def install(out, mode, plan):
    import grading
    from yam_arena import inspect_embodiment as official

    cls = official.YamArenaEmbodiment
    ensure0, reset0, step0 = cls._ensure_sim, cls.reset, cls._task_step
    poses, counts, scores, key_reads = [], [], [], []
    sim = view = contact = None
    filters, names = [], []

    def ensure(self, goal):
        nonlocal sim, view, contact, names, filters
        if sim is not None:
            raise RuntimeError('Use one new process per fixed-scene episode')
        if self.rig != official.station.rig('calibrated') or (self.cam_width,self.cam_height,self.info.control_hz,self.object_jitter) != (320,176,20,0):
            raise ValueError('Unexpected station configuration')
        from isaaclab.app import AppLauncher
        official._APP = AppLauncher(headless=True, device=self.device, enable_cameras=True, multi_gpu=False,
                                    kit_args='--/UJITSO/geometry=false').app
        atexit.register(official._close_app_at_exit)
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab.sim.utils import clone
        from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
        from isaaclab.sim.utils.stage import get_current_stage_id
        from pxr import Usd, UsdPhysics
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena import scene
        import mechanism
        import edge_catch
        compose0 = ArenaEnvBuilder.compose_manager_cfg
        def configure(builder):
            cfg, k = compose0(builder)
            cfg.sim.dt = 1 / PHYSICS_HZ
            cfg.decimation = cfg.sim.render_interval = PHYSICS_HZ // 20
            phys = cfg.sim.physics
            phys.solve_articulation_contact_last = True
            phys.friction_correlation_distance = .0001
            phys.friction_offset_threshold = .00005
            phys.gpu_max_rigid_contact_count = 262144
            phys.gpu_max_rigid_patch_count = 65536
            return cfg, k
        ArenaEnvBuilder.compose_manager_cfg = configure
        obj0 = scene.task_object
        def task_object(spec):
            obj = obj0(spec)
            if spec.name == 'build_anchor':
                @clone
                def spawn(path, cfg, *args, **kwargs):
                    prim = spawn_cuboid.__wrapped__(path, cfg, *args, **kwargs)
                    mechanism.build(prim.GetStage())
                    edge_catch.add(prim.GetStage(), mechanism.ROOT)
                    return prim
                obj.spawner_cfg.func = spawn
                obj.object_cfg.spawn = obj.spawner_cfg
            return obj
        scene.task_object = task_object
        ensure0(self, goal)
        sim = tensors.create_simulation_view('torch', stage_id=get_current_stage_id())
        sim.set_subspace_roots('/')
        bodies = ['source','receiver','source_frame','receiver_frame','source_knob','receiver_knob',
                  'd0','d1','d2','d3','roller0','roller2']
        view = sim.create_rigid_body_view([mechanism.ROOT+'/'+n for n in bodies])
        names = [p.rsplit('/',1)[-1] for p in view.prim_paths]
        stage = omni.usd.get_context().get_stage()
        robot = sorted(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath('/World/envs/env_0'))
                       if any('/'+a+'/' in str(p.GetPath()) for a in ('LeftArm','RightArm'))
                       and p.HasAPI(UsdPhysics.RigidBodyAPI) and 'PhysxContactReportAPI' in p.GetAppliedSchemas())
        if len(robot) != 18 or set(names) != set(bodies):
            raise ValueError(f'Missing native bodies: {len(robot)} robot links and {names}')
        filters = [mechanism.ROOT+'/'+n for n in bodies[:4]] + robot
        contact = sim.create_rigid_contact_view(list(view.prim_paths), [filters]*view.count, max_contact_data_count=16384)
        grading.CURRENT = grading.Grader(names, filters)

    def sample():
        p = view.get_transforms().cpu().numpy().copy()
        c = contact.get_contact_data(dt=1/PHYSICS_HZ)[4].cpu().numpy().copy()
        poses.append(p)
        counts.append(c)
        scores.append(grading.CURRENT.update(p,c))
        if len(scores)%100 == 1:
            (out/'progress.json').write_text(json.dumps(dict(action=len(scores)-1,score=scores[-1])))

    def reset(self, *args, **kwargs):
        obs = reset0(self,*args,**kwargs)
        sample()
        return obs

    def task_step(self):
        sample()
        return step0(self)

    cls._ensure_sim, cls.reset, cls._task_step = ensure, reset, task_step
    if mode != 'dream':
        from yam_arena.scripted import YamScriptedPolicy
        from yam_arena.constants import READY
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack
        from vision_key import classify
        index = branch = 0
        def act(self, obs):
            nonlocal index, branch
            if mode == 'setup':
                a = [*READY, 1., *READY, 1.]
            else:
                if index and plan['labels'][index-1] == 'read_key':
                    vote, pixels = classify(obs.images['right_cam'], plan['eye'],plan['roll'])
                    key_reads.append(dict(action=index,heading=vote,pixels=pixels))
                    if plan['labels'][index] != 'read_key':
                        votes = [row['heading'] for row in key_reads[-6:]]
                        eligible = [v for v in (0,1) if votes.count(v)>=5]
                        if len(eligible) != 1:
                            raise RuntimeError('Inconclusive wrist observation')
                        branch = eligible[0]
                a = plan['a'+str(branch)][index]
            index += 1
            return ActionChunk(actions=[Action(data=pack(a[:7],a[7:]))],control_hz=20)
        YamScriptedPolicy.act = act

    def save():
        if poses:
            np.savez_compressed(out/'physics.npz',poses=poses,counts=counts,names=names,filters=filters,scores=scores)
        if key_reads:
            (out/'wrist-read.json').write_text(json.dumps(key_reads,indent=2))
    return save
