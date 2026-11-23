"""`yam_bike_cage`: the stock yam_arena embodiment plus the bike's front triangle with its bottle cage (one fixed-base
articulation, built by bike_cage.mech in place of the task file's placeholder box "bike") and the bottle (a free rigid
body, in place of the placeholder "bottle"). Created 11_16_26 (America/New_York).

The scorer gets, besides the stock objects: `left_tcp` and `right_tcp` (each gripper's fingertip midpoint), `bottle`
(the middle of its base, its orientation and velocities) and `cage_arm` (low[0] = the sprung arm's opening angle in
degrees, 0 = closed on its stop). After each control step the task's note_step runs. Robot, cameras, action and
observations are the stock ones."""

import math
import os

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

DEBUG = os.environ.get("BIKE_DEBUG") == "1"
BIKE = "bike"
BOTTLE = "bottle"
EXTRA = ("left_tcp", "right_tcp", BOTTLE, "cage_arm")


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


def _np(a):
    if hasattr(a, "numpy"):
        try:
            return np.array(a.numpy(), dtype=np.float32)
        except TypeError:
            pass
    return np.array(a.detach().cpu().numpy() if hasattr(a, "detach") else a, dtype=np.float32)


class BikeCageEmbodiment(YamArenaEmbodiment):
    _view = None
    _path = None
    _dof = None

    def reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        self.task.module.reset_latch()
        self._close_arm()
        return super().reset(scene, seed=seed)

    def _ensure_sim(self, goal):
        import atexit

        from yam_arena import inspect_embodiment as base

        if self._env is not None:
            return super()._ensure_sim(goal)
        if base._APP is None:  # Kit must be up before Arena is imported
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            base._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene

        original = scene.task_object
        owner = self

        def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            from . import mech

            stage = omni.usd.get_context().get_stage()
            _place(stage, prim_path, translation, orientation)
            name = prim_path.rstrip("/").rsplit("/", 1)[-1]
            if name == BIKE:
                owner._path = prim_path
                return mech.build_bike(stage, prim_path)
            return mech.build_bottle(stage, prim_path)

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            if spec.name not in (BIKE, BOTTLE):
                return original(spec)
            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            kind = ObjectType.BASE if spec.name == BIKE else ObjectType.RIGID
            return Object(name=spec.name, object_type=kind, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    # -- the cage arm, through the PhysX tensor view ------------------------------------------------------------------
    def _art(self):
        if self._view is None:
            import omni.physics.tensors as tensors

            sim_view = None
            try:
                from isaaclab.sim import SimulationContext

                sim_view = SimulationContext.instance().physics_sim_view
            except Exception:  # noqa: BLE001
                sim_view = None
            if sim_view is None:
                sim_view = tensors.create_simulation_view("torch")
                sim_view.set_subspace_roots("/")
            self._view = sim_view.create_articulation_view(self._path)
            names = list(self._view.shared_metatype.dof_names)
            self._dof = names.index("arm_joint")
            if DEBUG:
                print("BIKE dofs", names, flush=True)
        return self._view

    def _close_arm(self):
        v = self._art()
        like = v.get_dof_positions()
        pos = _np(like) * 0.0
        idx = np.array([0], dtype=np.int32)
        for setter, getter in ((v.set_dof_positions, v.get_dof_positions), (v.set_dof_velocities, v.get_dof_velocities)):
            like = getter()
            if type(like).__module__.startswith("warp"):
                import warp as wp

                setter(wp.from_numpy(pos, dtype=wp.float32, device=like.device), wp.from_numpy(idx, dtype=wp.int32, device=like.device))
            elif hasattr(like, "detach"):
                import torch

                setter(torch.tensor(pos, dtype=torch.float32, device=like.device), torch.tensor(idx, dtype=torch.int32, device=like.device))
            else:
                setter(pos, idx)

    def _extra_states(self):
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.task_file import ObjectState

        base = self._base
        origin = base.scene.env_origins[0].cpu().numpy().astype(float)
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        zero, still = np.zeros(3), np.array((0.0, 0.0, 0.0, 1.0))
        out = {}
        for arm, robot in (("left", "left_robot"), ("right", "right_robot")):
            arm_ids, _ = self._ids[robot]
            q = arr(base.scene[robot].data.joint_pos.torch[0, arm_ids])
            p = np.asarray(ik.tcp_pose(q, np.array(BASES[arm]))[0], float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=p, high=p)
        world = base.arena_world
        pose = arr(world.get_pose_w(BOTTLE)[0])
        p = pose[:3] - origin
        out[BOTTLE] = ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=arr(world.get_root_linear_velocity_w(BOTTLE)[0]),
                                  ang_vel=arr(world.get_root_angular_velocity_w(BOTTLE)[0]), low=p, high=p)
        ang = math.degrees(float(_np(self._art().get_dof_positions())[0, self._dof]))
        a = np.array((ang, 0.0, 0.0))
        out["cage_arm"] = ObjectState(pos=a, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=a, high=a)
        return out

    def _task_step(self):
        from yam_arena import scene

        original = scene.object_states
        m = self.task.module
        last = {}

        def object_states(env, names):
            names = [n for n in names if n not in EXTRA and n != BIKE]
            last["s"] = {**(original(env, names) if names else {}), **self._extra_states()}
            return last["s"]

        scene.object_states = object_states
        try:
            result = super()._task_step()
        finally:
            scene.object_states = original
        if "s" in last:
            m.note_step(last["s"], 1.0 / self.info.control_hz)
            if DEBUG:
                print(m.debug_line(last["s"], self._t), flush=True)
        return result


def make_embodiment(**kwargs):
    return BikeCageEmbodiment(**kwargs)
