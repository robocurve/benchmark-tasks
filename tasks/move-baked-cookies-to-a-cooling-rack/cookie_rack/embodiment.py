"""`yam_cookie`: the stock yam_arena embodiment plus the pan, cookies, rack, turner and rest (cookie_rack.parts).

Created 11_15_26 18:40 EST. The scorer gets every loose object in its own frame (see parts), and `left_tcp` / `right_tcp`
(each gripper's fingertip midpoint). Every contact force on every cookie is read from PhysX after every physics step (see `Loads`):
the robot's force on it (a touch: a contact force, not a distance) and the loads that break it go to the task's
note_step per cookie. The harness is unchanged; `plan.install()` only lets the stock `yam_scripted` policy play a ready
joint plan.
"""

import atexit
import math
import os

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import plan

DEBUG = os.environ.get("COOKIE_DEBUG") == "1"
ARMS = (("left", "left_robot"), ("right", "right_robot"))

plan.install()


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


class Loads:
    """What PhysX reports pressing on each cookie, polled after every physics step: every contact's point, direction and
    force (a rigid contact view from each cookie to the pan, the rack, the turner, the table, the other cookies and
    every robot link). From these: the robot's force on the cookie (a touch), and the bending, squeeze and press loads
    of parts.loads(), averaged over parts.LOAD_WINDOW steps; and each cookie's downward speed (a fall). take() gives each one's peak since the last call."""

    KEYS = ("touch", "bend", "squeeze", "press", "fall")

    def __init__(self):
        self.view = None
        self.error = None
        self.last = None

    def bind(self, names):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        from . import parts

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        robot, bodies, cookies = [], {}, {}
        for prim in get_current_stage().Traverse():
            p = prim.GetPath().pathString
            if "/env_0/" not in p:
                continue
            name = prim.GetName()
            if name == "table" and p.endswith("/env_0/table"):
                bodies["table"] = p
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if name in names:
                cookies[name] = p
            elif name in ("pan", "rack", "turner"):
                bodies[name] = p
            elif "/LeftArm/" in p or "/RightArm/" in p:
                robot.append(p)
        if len(robot) < 10 or len(cookies) != len(names) or len(bodies) < 3:
            raise RuntimeError(f"load monitor: found {len(robot)} robot links, {sorted(cookies)}, {sorted(bodies)}")
        self.names = list(names)
        paths = [cookies[n] for n in self.names]
        self.parts = parts
        physics = PhysxManager.get_physics_sim_view()
        for keys in (sorted(bodies), sorted(k for k in bodies if k != "table")):  # the table is a static collider: leave it out if PhysX won't take it as a filter
            others = [bodies[k] for k in keys]
            filters = [others + robot + [q for q in paths if q != p] for p in paths]
            self.robot = slice(len(others), len(others) + len(robot))
            self.view = physics.create_rigid_contact_view(paths, filter_patterns=filters, max_contact_data_count=256 * len(paths))
            if self.view is not None and getattr(self.view, "filter_count", len(filters[0])) == len(filters[0]):
                self.with_table = "table" in keys
                break
        self.pose = physics.create_rigid_body_view(paths)
        self.window = {n: [] for n in self.names}
        self.peak = {n: dict.fromkeys(self.KEYS, 0.0) for n in self.names}
        self.sign = float(os.environ.get("COOKIE_NORMAL_SIGN", "1"))
        if not getattr(PhysxManager.step, "_cookie_rack", False):
            original = PhysxManager.step.__func__
            me = self

            def step(cls):
                original(cls)
                me.poll(cls)

            step._cookie_rack = True
            PhysxManager.step = classmethod(step)

    @staticmethod
    def _np(x):
        return x.detach().cpu().numpy().astype(float) if hasattr(x, "detach") else np.asarray(x.numpy() if hasattr(x, "numpy") else x, float)

    def poll(self, manager):
        if self.view is None:
            return
        try:
            f, pts, nrm, _, cnt, start = (self._np(x) for x in self.view.get_contact_data(dt=manager.get_physics_dt()))
            f, pts, nrm = f.reshape(-1), pts.reshape(-1, 3), nrm.reshape(-1, 3)
            cnt, start = cnt.reshape(len(self.names), -1).astype(int), start.reshape(len(self.names), -1).astype(int)
            tf = self._np(self.pose.get_transforms()).reshape(len(self.names), 7)
            vel = self._np(self.pose.get_velocities()).reshape(len(self.names), 6)
            for i, name in enumerate(self.names):
                idx = np.concatenate([np.arange(s, s + c) for s, c in zip(start[i], cnt[i])] or [np.zeros(0, int)]).astype(int)
                rid = np.concatenate([np.arange(s, s + c) for s, c in zip(start[i][self.robot], cnt[i][self.robot])] or [np.zeros(0, int)]).astype(int)
                touch = float(np.abs(f[rid]).sum())
                rot = _rot(tf[i, 3:7])
                force = self.sign * f[idx, None] * nrm[idx]
                bend, squeeze, press = self.parts.loads((pts[idx] - tf[i, :3]) @ rot, force @ rot, weight_up=rot[2, 2])
                w = self.window[name]
                w.append((bend, squeeze, press))
                del w[: -self.parts.LOAD_WINDOW]
                now = {"touch": touch, "bend": float(np.mean([x[0] for x in w], axis=0).max()),
                       "squeeze": float(np.mean([x[1] for x in w], axis=0).max()), "press": float(np.mean([x[2] for x in w])), "fall": max(0.0, -float(vel[i, 2]))}
                if not all(math.isfinite(v) for v in now.values()):
                    raise ValueError("nonfinite contact load")
                pk = self.peak[name]
                for k in self.KEYS:
                    pk[k] = max(pk[k], now[k])
                self.last = (name, force.sum(axis=0) if len(idx) else np.zeros(3)) if i == 0 else self.last
        except Exception as exc:  # reported at the next control step
            import traceback

            self.error = traceback.format_exc() + repr(exc)

    def take(self):
        """{cookie: {"touch": N, "bend": N m, "squeeze": N, "press": N, "fall": m/s}}: peaks since the last call."""
        if self.error:
            raise RuntimeError("load monitor failed: " + self.error)
        out = {n: dict(v) for n, v in self.peak.items()}
        for v in self.peak.values():
            for k in v:
                v[k] = 0.0
        return out


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


class CookieEmbodiment(YamArenaEmbodiment):
    _contacts = None
    _cracks = None
    _episodes = 0

    def _ensure_sim(self, goal):
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

            from . import parts

            stage = omni.usd.get_context().get_stage()
            _place(stage, prim_path, translation, orientation)
            name = prim_path.rstrip("/").rsplit("/", 1)[-1]
            if name == "pan":
                return parts.build_pan(stage, prim_path)
            if name == "rack":
                return parts.build_rack(stage, prim_path)
            if name == "turner":
                return parts.build_turner(stage, prim_path)
            if name == "rest":
                return parts.build_rest(stage, prim_path)
            return parts.build_cookie(stage, prim_path)

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            kind = ObjectType.BASE if spec.name == "rest" else ObjectType.RIGID
            return Object(name=spec.name, object_type=kind, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                            rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def _show_cracks(self, broken):
        """Show the crack on each broken cookie (only those that change)."""
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        if self._cracks is None:
            self._cracks = {}
            for prim in stage.Traverse():
                p = prim.GetPath().pathString
                if prim.GetName() == "crack" and "/cookie_" in p:
                    self._cracks[p.rsplit("/", 2)[-2]] = [UsdGeom.Imageable(prim), False]
        for name, rec in self._cracks.items():
            want = name in broken
            if rec[1] != want:
                rec[0].MakeVisible() if want else rec[0].MakeInvisible()
                rec[1] = want

    def reset(self, scene, *, seed=None):
        m = self.task.module
        m.reset_trial()
        out = super().reset(scene, seed=seed)
        if self._contacts is None:
            self._contacts = Loads()
            self._contacts.bind(m.COOKIE_NAMES)
        if self._episodes:  # a later episode in the same process: put everything back, mend the cookies, settle again
            base, torch = self._base, self._torch
            for name in self.task.free_objects:
                obj = base.scene[name]
                pose = obj.data.default_root_pose.torch.clone()
                pose[:, :3] += base.scene.env_origins
                obj.write_root_pose_to_sim_index(root_pose=pose)
                obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
            for _ in range(max(1, round(0.5 / base.step_dt))):
                self._env.step(self._ready)
            base.sim.render()
            out = self._observation()
        self._episodes += 1
        self._contacts.take()
        m.reset_trial()
        self._show_cracks(set())
        return out

    def _states(self):
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.task_file import ObjectState

        base = self._base
        origin = base.scene.env_origins[0].cpu().numpy().astype(float)
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        zero, still = np.zeros(3), np.array((0.0, 0.0, 0.0, 1.0))
        out = {}
        for arm, robot in ARMS:
            arm_ids, _ = self._ids[robot]
            p = np.asarray(ik.tcp_pose(arr(base.scene[robot].data.joint_pos.torch[0, arm_ids]), np.array(BASES[arm]))[0], float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=p, high=p)
        world = base.arena_world
        for name in self.task.free_objects:
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
            last["s"] = self._states()
            m.note_step(last["s"], 1.0 / self.info.control_hz, self._contacts.take())
            return last["s"]

        scene.object_states = object_states
        try:
            result = super()._task_step()
        finally:
            scene.object_states = original
        st = last.get("s")
        if st is not None:
            self._show_cracks(m.broken_names())
        if st is not None and DEBUG and round(self._t * self.info.control_hz) % 4 == 0:
            last = self._contacts.last
            print(f"COOKIE t={self._t:6.2f} " + m.debug_line(st) + (f" F1 {np.round(last[1], 3).tolist()} table {int(self._contacts.with_table)}" if last else ""), flush=True)
        return result


def make_embodiment(**kwargs):
    return CookieEmbodiment(**kwargs)
