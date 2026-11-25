"""Attach the declarative fixture to the stock dual-arm YAM environment."""
import atexit
from dataclasses import replace
import math
from pathlib import Path
import tempfile

import numpy as np
from inspect_robots import StepResult
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.station import BASES, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from .grading import Grade
from .scene import compile_layer, load_fixture
from .scripted import make_scripted


active = None


def array(tensor):
    native = tensor.torch if hasattr(tensor, "torch") else tensor
    return native.detach().cpu().numpy().copy()


def metric(spec, frames, contacts):
    if spec["kind"] == "contact_n":
        return contacts[(spec["a"], spec["b"])]
    a, b = frames[spec["a"]], frames[spec["b"]]
    p, q = np.array(spec["pa"]), np.array(spec["pb"])
    kind = spec["kind"]
    if kind == "angle_deg":
        u, v = a[1] @ p, b[1] @ q
        cosine = np.dot(u, v)/(np.linalg.norm(u)*np.linalg.norm(v))
        return math.degrees(math.acos(float(np.clip(cosine, -1., 1.))))
    if kind == "speed":
        return float(np.linalg.norm(a[2][:3]))
    if kind == "spin_deg_s":
        return float(np.degrees(np.linalg.norm(a[2][3:])))
    delta = (a[0] + a[1] @ p) - (b[0] + b[1] @ q)
    if kind == "local_dx":
        return float(delta @ (b[1] @ spec.get("axis", (1.,0.,0.))))
    if kind == "distance":
        return float(np.linalg.norm(delta))
    if kind == "planar":
        return float(np.linalg.norm(delta[:2]))
    return float(delta[{"dx": 0, "dy": 1, "dz": 2}[kind]])


class PhysicalTaskArena(YamArenaEmbodiment):
    def __init__(self):
        global active
        self.spec = load_fixture()
        from .coverage import LensCoverage
        actors = {a["name"]: a for a in self.spec["actors"]}
        self.lens_coverage = {m["name"]: LensCoverage(m, actors) for m in self.spec["metrics"]
                              if m["kind"] == "lens_coverage"}
        super().__init__(scene="station", object_jitter=0, control_hz=20, cam_width=320, cam_height=176)
        self.info = replace(self.info, name="tabletop_task", supported_target_kinds=frozenset())
        self.assets = tempfile.TemporaryDirectory(prefix="robot-fixture-")
        self.layer = Path(self.assets.name) / "fixture.usda"
        self.layer.write_text(compile_layer(self.spec), encoding="utf-8")
        active = self

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        import yam_arena.inspect_embodiment as runtime
        if runtime._APP is None:
            import ctypes
            ctypes.CDLL("libnvidia-ngx.so.1")
            from isaaclab.app import AppLauncher
            runtime._APP = AppLauncher(headless=True, device="cuda:0", enable_cameras=True,
                renderer="RealTimePathTracing", anti_aliasing=4,
                kit_args="--/persistent/rtx/modes/rt2/enabled=true --/persistent/rtx/modes/rt/enabled=false "
                         "--/persistent/rtx/modes/pt/enabled=false").app
            atexit.register(runtime._close_app_at_exit)
        from yam_arena import scene as station

        def add_fixture(embodiment, *, device, control_hz, **kwargs):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene
            objects = station.task_room("station", embodiment.rig.top_cam)
            objects.append(Object(name="task_fixture", object_type=ObjectType.BASE,
                spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(self.layer))))
            env = IsaacLabArenaEnvironment(name="yam_tabletop_task", scene=Scene(objects), embodiment=embodiment,
                                          env_cfg_callback=station.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()

        original = station.make_env
        station.make_env = add_fixture
        try:
            super()._ensure_sim(None)
        finally:
            station.make_env = original

    def _bind_bodies(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths
        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()
        actor_roots = [str(p.GetPath()) for p in stage.Traverse()
                       if p.GetName() == "Actors" and str(p.GetPath()).startswith(root+"/")]
        if len(actor_roots) != 1:
            raise RuntimeError(f"Expected exactly one fixture, got {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.bodies = self.physics.create_rigid_body_view(actors+"/*")
        paths = list(self.bodies.prim_paths)
        self.rows = {actor["name"]: paths.index(actors+"/"+actor["name"]) for actor in self.spec["actors"]}
        self.contacts = {}
        self.pad_bodies={}
        links=asset_link_paths()
        pad_paths={arm+"_pad_"+side:root+"/"+arm.title()+"Arm"+links["tip_"+side]
                   for arm in ("left","right") for side in ("left","right")}
        for spec in self.spec["metrics"]:
            if spec["kind"] == "contact_n":
                arm, obj = spec["a"], spec["b"]
                if arm in pad_paths:
                    sensors=[pad_paths[arm]]
                elif arm in self.rows:
                    sensors=[actors+"/"+arm]
                else:
                    sensors = [root+"/"+arm.removesuffix("_tcp").title()+"Arm"+suffix for suffix in links.values()]
                self.contacts[arm, obj] = self.physics.create_rigid_contact_view(sensors, [[actors+"/"+obj]]*len(sensors))
            else:
                for frame in (spec["a"],spec["b"]):
                    if frame in pad_paths and frame not in self.pad_bodies:
                        self.pad_bodies[frame]=self.physics.create_rigid_body_view(pad_paths[frame])

    def _measure(self, observation):
        from yam_arena.constants import PHYSICS_DT
        poses = array(self.bodies.get_transforms())
        velocity = array(self.bodies.get_velocities())
        table = np.array([0., 0., float(TABLE_TOP)])
        self.frames = {"table": (np.zeros(3), np.eye(3), np.zeros(6))}
        for name, row in self.rows.items():
            self.frames[name] = (poses[row,:3]-table, quat_xyzw_to_matrix(poses[row,3:]), velocity[row])
        for name,view in self.pad_bodies.items():
            pose=array(view.get_transforms())[0]
            self.frames[name]=(pose[:3]-table,quat_xyzw_to_matrix(pose[3:]),array(view.get_velocities())[0])
        joints = np.asarray(observation.state["joint_pos"])
        for arm, offset in [("left", 0), ("right", 7)]:
            pos, rot = ik.tcp_pose(joints[offset:offset+6], np.asarray(BASES[arm])-table)
            self.frames[arm+"_tcp"] = (np.asarray(pos), np.asarray(rot), np.zeros(6))
        contacts = {key: float(np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum())
                    for key, view in self.contacts.items()}
        return {s["name"]: (self.lens_coverage[s["name"]].measure(self.frames)
                            if s["kind"] == "lens_coverage" else metric(s, self.frames, contacts))
                for s in self.spec["metrics"]}

    def reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        self._bind_bodies()
        poses = self.bodies.get_transforms().clone()
        for actor in self.spec["actors"]:
            x, y, z = actor["position"]
            poses[self.rows[actor["name"]]] = self._torch.tensor((x,y,float(TABLE_TOP)+z,0.,0.,0.,1.),
                                                                device=poses.device, dtype=poses.dtype)
        ids = self._torch.arange(len(poses), dtype=self._torch.int32, device=poses.device)
        self.bodies.set_transforms(poses, ids)
        self.bodies.set_velocities(self._torch.zeros_like(self.bodies.get_velocities()), ids)
        observation = super().reset(scene, seed=seed)
        self.grading = Grade(self.spec)
        self.demo_command = None
        self.grading.prime(self._measure(observation))
        return observation

    def step(self, action):
        observation = super().step(action).observation
        grade = self.grading.update(self._measure(observation))
        if self.demo_command is not None:
            # Diagnostics are written to the grader's trace; never passed into a model observation.
            grade["demo"] = {**self.demo_command, "tcp_m": {arm: np.round(self.frames[arm+"_tcp"][0],4).tolist()
                                                          for arm in ("left","right")}}
        done = False  # Inspect evaluates the final observation at the declared horizon.
        return StepResult(observation=observation, terminated=done,
                          termination_reason="success" if done else None, info={"grade": grade})

    def close(self):
        try:
            super().close()
        finally:
            self.assets.cleanup()


def make_embodiment():
    return PhysicalTaskArena()
