"""`yam_tack`: the stock yam_arena embodiment plus the fit-up bench, the coupon and the TIG torch (tack_t_joint.mech). The
scorer also gets `left_tcp` and `right_tcp` (each gripper's fingertip midpoint), `left_grip` and `right_grip` (the contact force between the coupon and each of that gripper's two fingers, read from a PhysX contact view), and `coupon` and `torch` in their own
frames (the coupon's centre, the torch's grip centre). After each control step the task's note_step runs; for each tack
it reports, a joint between the world and the coupon is set to the coupon's pose at that moment and switched on, and a
tack it reports cracked is switched off. The robot, cameras, action and observations are the stock ones."""

import math
import os

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

DEBUG = os.environ.get("TACK_DEBUG") == "1"
BENCH = "bench"
PARTS = ("coupon", "torch")


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


class TackEmbodiment(YamArenaEmbodiment):
    _joints = None
    _fx = None
    _contact = None
    _recent = None  # the last few control steps' finger forces on the coupon

    def reset(self, scene, *, seed=None):
        self.task.module.reset_latch()
        if self._joints is not None:
            self._release_all()
        out = super().reset(scene, seed=seed)
        self._make_joints()
        self._release_all()
        return out

    def _ensure_sim(self, goal):
        import atexit

        from yam_arena import inspect_embodiment as base

        if base._APP is None:  # Kit must be up before Arena is imported
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
            return {BENCH: mech.build_bench, "coupon": mech.build_coupon, "torch": mech.build_torch}[name](stage, prim_path)

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            if spec.name != BENCH and spec.name not in PARTS:
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

    # -- the tacks ---------------------------------------------------------------------------------------------------
    def _stage(self):
        import omni.usd

        return omni.usd.get_context().get_stage()

    def _make_joints(self):
        if self._joints is not None:
            return
        from pxr import UsdGeom

        from . import mech

        stage = self._stage()
        coupon = next(str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "coupon" and "/envs/" in str(p.GetPath()))
        self._joints = mech.build_tack_joints(stage, coupon)
        # the coupon's contact force against each finger of each gripper
        from isaaclab_physx.physics import PhysxManager

        self._fingers = []
        for arm, robot in (("left", "left_robot"), ("right", "right_robot")):
            paths = self._base.scene[robot].root_physx_view.link_paths
            for p in (paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths):
                if p.rsplit("/", 1)[-1] in ("tip_left", "tip_right"):
                    self._fingers.append((arm, p))
        assert len(self._fingers) == 4, self._fingers
        self._contact = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            [coupon], filter_patterns=[[p for _, p in self._fingers]], max_contact_data_count=64)
        self._fx = {}
        for prim in stage.Traverse():
            n, p = prim.GetName(), str(prim.GetPath())
            if "/torch/fx_" in p or ("/bench/tack" in p and n.startswith("tack")):
                self._fx[n] = [UsdGeom.Imageable(prim), None, prim]

    def _finger_forces(self):
        """Per arm, the force (N) between the coupon and each of its two fingers: the largest over the last 5 control steps
        (0.25 s), so one physics step of chatter in a steady grip does not read as letting go."""
        fm = self._contact.get_contact_force_matrix(dt=float(self._base.physics_dt))
        fm = fm.detach().cpu().numpy() if hasattr(fm, "detach") else np.asarray(fm.numpy() if hasattr(fm, "numpy") else fm)
        now = np.linalg.norm(np.asarray(fm, float).reshape(-1, 3), axis=1)
        self._recent = ([now] if self._recent is None else self._recent + [now])[-5:]
        peak = np.max(self._recent, axis=0)
        return {arm: [float(peak[i]) for i, (a, _) in enumerate(self._fingers) if a == arm] for arm in ("left", "right")}

    def _release_all(self):
        self._recent = None
        for j in self._joints:
            j.GetJointEnabledAttr().Set(False)
        self._show({})

    def _show(self, on):
        for n, rec in self._fx.items():
            want = n in on
            if rec[1] != want:
                rec[0].MakeVisible() if want else rec[0].MakeInvisible()
                rec[1] = want

    def _apply(self, commands):
        from pxr import Gf

        from . import mech

        origin = self._base.scene.env_origins[0].cpu().numpy().astype(float)
        m = self.task.module
        for cmd in commands:
            slot = cmd["slot"]
            if slot >= len(self._joints):
                continue
            if cmd["do"] == "lock":
                mech.lock_tack(self._joints[slot], np.asarray(cmd["anchor"]) + origin, np.asarray(cmd["pos"]) + origin, cmd["quat"])
                bead = self._fx.get(f"tack{slot}")
                if bead is not None:
                    a = np.asarray(cmd["anchor"], float) - (0.0, 0.0, m.TOP)
                    bead[2].GetAttribute("xformOp:translate").Set(Gf.Vec3d(float(a[0]), float(a[1]), float(a[2])))
            else:
                self._joints[slot].GetJointEnabledAttr().Set(False)

    def _extra_states(self):
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.task_file import ObjectState

        base = self._base
        origin = base.scene.env_origins[0].cpu().numpy().astype(float)
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        zero, still = np.zeros(3), np.array((0.0, 0.0, 0.0, 1.0))
        out = {}
        forces = self._finger_forces()
        for arm, robot in (("left", "left_robot"), ("right", "right_robot")):
            arm_ids, _ = self._ids[robot]
            p = np.asarray(ik.tcp_pose(arr(base.scene[robot].data.joint_pos.torch[0, arm_ids]), np.array(BASES[arm]))[0], float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=p, high=p)
            g = np.array((*forces[arm], 0.0))
            out[f"{arm}_grip"] = ObjectState(pos=g, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=g, high=g)  # N on each finger
        world = base.arena_world
        for name in PARTS:
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
            names = [n for n in names if n != BENCH and n not in PARTS]  # these come from _extra_states
            last["s"] = {**original(env, names), **self._extra_states()}
            return last["s"]

        scene.object_states = object_states
        try:
            result = super()._task_step()
        finally:
            scene.object_states = original
        if "s" in last:
            st = last["s"]
            m.note_step(st, 1.0 / self.info.control_hz)
            self._apply(m.take_commands())
            self._show(m.shown())
            if DEBUG:
                print(f"TACK t={self._t:6.2f} " + m.debug_line(st), flush=True)
        return result


def make_embodiment(**kwargs):
    return TackEmbodiment(**kwargs)
