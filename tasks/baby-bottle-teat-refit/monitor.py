"""Install the free objects and read-only physical score on the calibrated YAM station."""
import atexit
import json
import numpy as np
from scipy.spatial import cKDTree

import geometry
from live_score import AssemblyScore

_active = None


def end_score():
    if _active is None:
        raise RuntimeError("Use run.py to install the physical assembly monitor")
    return _active.observe()


class Monitor:
    def __init__(self, embodiment):
        self.embodiment = embodiment
        from isaaclab.sim.utils.stage import get_current_stage_id
        import omni.physics.tensors as tensors
        import omni.usd
        from pxr import UsdPhysics

        sv = tensors.create_simulation_view("torch", stage_id=get_current_stage_id())
        sv.set_subspace_roots("/")
        self.view = sv
        stage = omni.usd.get_context().get_stage()
        links = sorted(str(p.GetPath()) for p in stage.Traverse()
                       if p.HasAPI(UsdPhysics.RigidBodyAPI)
                       and 'PhysxContactReportAPI' in p.GetAppliedSchemas()
                       and any(name in str(p.GetPath()) for name in ('/LeftArm/', '/RightArm/')))
        fingers = [p for p in links if p.split('/')[-1] in ('tip_left','tip_right')]
        if len(links) != 18 or len(fingers) != 4:
            raise RuntimeError("Unexpected YAM collision body inventory")
        rigid_paths = [geometry.ROOT+'/'+s for s in ('collar','bottle')]
        self.rigid = sv.create_rigid_body_view(rigid_paths)
        self.arm_bodies = sv.create_rigid_body_view(links)
        self.contacts = sv.create_rigid_contact_view(list(self.rigid.prim_paths),[links]*2,
                                                    max_contact_data_count=8192)
        self.soft = sv.create_volume_deformable_body_view(geometry.ROOT+'/teat')
        self.load = sv.create_rigid_body_view(geometry.ROOT+'/water_load')
        if self.soft.count != 1 or not np.isclose(self.load.get_masses().cpu().item(),.020):
            raise RuntimeError("Missing deformable teat or 20 g water ballast")
        self.local, self.tets, self.grid = geometry.specimen()
        rest = self.soft.get_rest_nodal_positions()[0].cpu().numpy()
        distances, self.order = cKDTree(rest).query(self.local + geometry.TEAT)
        if distances.max() >= .00001 or len(np.unique(self.order)) != len(self.local):
            raise ValueError("Unexpected material node order")
        self.objects = list(self.rigid.prim_paths)
        self.arm_paths = list(self.arm_bodies.prim_paths)
        names = [s.split('/')[-1] for s in self.objects]
        self.scorer = AssemblyScore(names,self.arm_paths,self.grid,self.tets)
        self.poses, self.arm_poses, self.nodes, self.counts, self.scores = [], [], [], [], []
        self.pending = []
        self.sim = embodiment._base.sim
        self.original_step = self.sim.step
        self.sim.step = self.physics_step
        self.observe(initial=True)

    def physics_step(self, *args, **kwargs):
        result = self.original_step(*args, **kwargs)
        self.pending.append(self.contacts.get_contact_data(dt=1/240)[4].clone())
        return result

    def observe(self, initial=False):
        import torch
        poses = self.rigid.get_transforms().cpu().numpy().copy()
        arms = self.arm_bodies.get_transforms().cpu().numpy().copy()
        nodes = self.soft.get_simulation_nodal_positions()[0].cpu().numpy()[self.order]
        counts = None
        if not initial:
            if len(self.pending) != 12:
                raise RuntimeError(f"Expected twelve substeps, observed {len(self.pending)}")
            counts = torch.stack(self.pending).cpu().numpy()
            if np.any(counts.sum(axis=tuple(range(1, counts.ndim))) >= 8192):
                raise RuntimeError("Rigid contact buffer saturated")
            self.counts.extend(counts)
        self.pending.clear()
        self.poses.append(poses)
        self.arm_poses.append(arms)
        self.nodes.append(nodes)
        value = self.scorer.observe(poses,nodes,arms,counts)
        if not initial:
            self.scores.append(value)
        return value

    def save(self, out):
        np.savez_compressed(out/'physics.npz',nodes=self.nodes,poses=self.poses,arm_poses=self.arm_poses,
                            robotcounts=self.counts,objects=self.objects,
                            arm_paths=self.arm_paths,grid=self.grid,tets=self.tets)
        (out/'score-trace.json').write_text(json.dumps(self.scores)+'\n')


def install():
    from isaaclab.app import AppLauncher
    from yam_arena import inspect_embodiment as yam

    yam._APP = AppLauncher(headless=True,device='cuda:0',enable_cameras=True,
                           multi_gpu=False,kit_args='--/UJITSO/geometry=false').app
    atexit.register(yam._close_app_at_exit)
    from yam_arena import scene, station
    from isaaclab.sim.utils import clone
    from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
    original_object = scene.task_object
    def task_object(spec):
        obj = original_object(spec)
        if spec.name == 'build_anchor':
            @clone
            def spawn(path,cfg,*args,**kwargs):
                prim = spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
                geometry.build(prim.GetStage())
                return prim
            obj.spawner_cfg.func = spawn
            obj.object_cfg.spawn = obj.spawner_cfg
        return obj
    scene.task_object = task_object
    original_reset = yam.YamArenaEmbodiment.reset
    def reset(emb, *args, **kwargs):
        global _active
        if _active is not None:
            _active.sim.step = _active.original_step
        obs = original_reset(emb,*args,**kwargs)
        if emb.rig != station.rig('calibrated') or emb._base.cfg.decimation != 12 or not np.isclose(emb._base.cfg.sim.dt,1/240):
            raise RuntimeError("Unexpected station or physics rate")
        _active = Monitor(emb)
        return obs
    yam.YamArenaEmbodiment.reset = reset


def save(out):
    if _active is None or not _active.scores:
        raise RuntimeError("No scored actions were recorded")
    _active.save(out)
