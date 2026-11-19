"""`yam_plasma`: the stock yam_arena embodiment plus the cutting stand, the plate's offcut, the plasma torch and the
straightedge (plasma_cut.mech). The scorer also gets `left_tcp` and `right_tcp` (each gripper's fingertip midpoint) and
`torch`, `guide` and `offcut` in their own frames. After each control step the task's note_step runs; the offcut's joints it
reports as cut are switched off, and the kerf pieces and arc glow it names are shown. The harness is unchanged."""

import math
import os
import re

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

DEBUG = os.environ.get("PLASMA_DEBUG") == "1"
BENCH = "bench"
LOOSE = ("torch", "guide", "offcut")
FX = re.compile(r"^(kerf|dross|gouge)_\d+$")


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


class PlasmaEmbodiment(YamArenaEmbodiment):
    _fx = None
    _joints = None
    _cut = None

    def reset(self, scene, *, seed=None):
        self.task.module.reset_latch()
        self._restore()
        out = super().reset(scene, seed=seed)
        self._show(set())
        return out

    def _restore(self):
        """Put the offcut back and re-tie every joint (only matters from the second episode on)."""
        if getattr(self, "_base", None) is None or self._joints is None:
            return
        from pxr import UsdPhysics

        obj = self._base.scene["offcut"]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] += self._base.scene.env_origins
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=self._torch.zeros_like(obj.data.default_root_vel.torch))
        for prim in self._joints.values():  # off and on again: this also re-makes a joint that broke
            UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
        for prim in self._joints.values():
            UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(True)
        self._cut = set()

    def _ensure_sim(self, goal):
        import atexit

        from yam_arena import inspect_embodiment as base

        if base._APP is None:  # Kit must be up before Arena is imported (its scene module needs PhysxSchema)
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            base._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene

        original = scene.task_object

        def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            from . import mech

            stage = omni.usd.get_context().get_stage()
            _place(stage, prim_path, translation, orientation)
            name = prim_path.rstrip("/").rsplit("/", 1)[-1]
            if name == BENCH:
                return mech.build_bench(stage, prim_path)
            if name == "torch":
                return mech.build_torch(stage, prim_path)
            if name == "guide":
                return mech.build_guide(stage, prim_path)
            return mech.build_offcut(stage, prim_path, prim_path.rstrip("/").rsplit("/", 1)[0] + "/" + BENCH)

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            if spec.name != BENCH and spec.name not in LOOSE:
                return original(spec)
            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            kind = ObjectType.BASE if spec.name == BENCH else ObjectType.RIGID
            return Object(name=spec.name, object_type=kind, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                            rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def _prims(self):
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        self._fx, self._joints, self._cut = {}, {}, set()
        for prim in stage.Traverse():
            n, p = prim.GetName(), str(prim.GetPath())
            if ("/bench/" in p and FX.match(n)) or "/torch/fx_" in p:
                self._fx[n] = [UsdGeom.Imageable(prim), None]
            elif "/offcut/joint_" in p:
                self._joints[int(n.rsplit("_", 1)[1])] = prim

    def _show(self, on):
        if getattr(self, "_base", None) is None:
            return
        if self._fx is None:
            self._prims()
        for n, rec in self._fx.items():
            want = n in on
            if rec[1] != want:
                rec[0].MakeVisible() if want else rec[0].MakeInvisible()
                rec[1] = want

    def _sever(self, cut):
        from pxr import UsdPhysics

        if self._joints is None:
            self._prims()
        for i in sorted(set(cut) - self._cut):
            UsdPhysics.Joint(self._joints[i]).GetJointEnabledAttr().Set(False)
            self._cut.add(i)

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
            p = np.asarray(ik.tcp_pose(arr(base.scene[robot].data.joint_pos.torch[0, arm_ids]), np.array(BASES[arm]))[0], float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=p, high=p)
        world = base.arena_world
        for name in LOOSE:
            pose = arr(world.get_pose_w(name)[0])
            p = pose[:3] - origin
            out[name] = ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=arr(world.get_root_linear_velocity_w(name)[0]),
                                    ang_vel=arr(world.get_root_angular_velocity_w(name)[0]), low=p, high=p)
        return out

    def _task_step(self):
        from yam_arena import scene

        original = scene.object_states
        last = {}
        m = self.task.module

        def object_states(env, names):
            names = [n for n in names if n != BENCH and n not in LOOSE]  # these come from _extra_states
            last["s"] = {**(original(env, names) if names else {}), **self._extra_states()}
            return last["s"]

        scene.object_states = object_states
        try:
            st0 = object_states(self._env, [])
            m.note_step(st0, 1.0 / self.info.control_hz)  # the cut's bookkeeping first, so score() reads this step's record
            result = super()._task_step()
        finally:
            scene.object_states = original
        self._sever(m.severed())
        self._show(m.shown())
        if DEBUG:
            print(m.debug_line(st0, self._t), flush=True)
        return result


def make_embodiment(**kwargs):
    return PlasmaEmbodiment(**kwargs)
