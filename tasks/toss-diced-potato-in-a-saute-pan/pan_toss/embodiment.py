"""`yam_pan`: the stock yam_arena embodiment plus the saute pan, the potato dice and the burner (pan_toss.pan).

The scorer gets `pan` and each `die_NN` in their own frames (pan: centre of the cooking surface; die: cube centre), and
`left_tcp` / `right_tcp` (each gripper's fingertip midpoint). Contact between any robot link and any die is read from
PhysX itself after every physics step (a rigid contact view filtered to the dice: a nonzero contact force, not a
distance), and latched into the task's record. The harness is unchanged; `plan.install()` only lets the stock
`yam_scripted` policy play a ready joint plan.
"""

import atexit
import math
import os

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import plan

DEBUG = os.environ.get("PAN_DEBUG") == "1"
LOG = os.environ.get("PAN_LOG")  # dev: write every step's poses, joints and torques to this .npz
FORCE_EPS = 1e-4  # N: any contact force above numerical noise is a touch
ARMS = (("left", "left_robot"), ("right", "right_robot"))

plan.install()


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


class _Contacts:
    """Robot-on-food contact straight from PhysX, polled after every physics step."""

    def __init__(self):
        self.view = None
        self.touched = False
        self.peak = 0.0
        self.error = None

    def bind(self):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        robot, dice = [], []
        for prim in get_current_stage().Traverse():
            p = prim.GetPath().pathString
            if "/env_0/" not in p or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if ("/LeftArm/" in p or "/RightArm/" in p) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
                robot.append(p)
            elif prim.GetName().startswith("die_"):
                dice.append(p)
        if len(robot) < 2 or not dice:
            raise RuntimeError(f"contact monitor: found {len(robot)} robot links and {len(dice)} dice")
        physics = PhysxManager.get_physics_sim_view()
        self.view = physics.create_rigid_contact_view(robot, filter_patterns=[dice] * len(robot), max_contact_data_count=64 * len(robot))
        self.links, self.dice = robot, dice
        if not getattr(PhysxManager.step, "_pan_toss", False):
            original = PhysxManager.step.__func__
            me = self

            def step(cls):
                original(cls)
                me.poll(cls)

            step._pan_toss = True
            PhysxManager.step = classmethod(step)

    def poll(self, manager):
        if self.view is None:
            return
        try:
            f = self.view.get_contact_force_matrix(dt=manager.get_physics_dt())
            f = f.detach().cpu().numpy() if hasattr(f, "detach") else np.asarray(f.numpy() if hasattr(f, "numpy") else f)
            peak = float(np.abs(f).max()) if f.size else 0.0
            if not math.isfinite(peak):
                raise ValueError("nonfinite contact force")
            self.peak = max(self.peak, peak)
            if peak > FORCE_EPS:
                self.touched = True
        except Exception as exc:  # reported at the next control step
            self.error = repr(exc)

    def take(self):
        """Whether a robot link touched a die since the last call."""
        if self.error:
            raise RuntimeError("contact monitor failed: " + self.error)
        out, self.touched = self.touched, False
        return out


class PanEmbodiment(YamArenaEmbodiment):
    _contacts = None
    _episodes = 0
    _log = None
    log_path = LOG

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

            from . import pan

            stage = omni.usd.get_context().get_stage()
            _place(stage, prim_path, translation, orientation)
            name = prim_path.rstrip("/").rsplit("/", 1)[-1]
            if name == "burner":
                return pan.build_burner(stage, prim_path)
            if name == "pan":
                return pan.build_pan(stage, prim_path)
            return pan.build_die(stage, prim_path)

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            kind = ObjectType.BASE if spec.name == "burner" else ObjectType.RIGID
            return Object(name=spec.name, object_type=kind, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                            rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def reset(self, scene, *, seed=None):
        m = self.task.module
        m.reset_trial()
        out = super().reset(scene, seed=seed)
        if self._contacts is None:
            self._contacts = _Contacts()
            self._contacts.bind()
        if self._episodes:  # a later episode in the same process: put the pan and dice back and settle again
            base = self._base
            torch = self._torch
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
        self._contacts.peak = 0.0
        m.reset_trial()
        self.flush_log()
        self._log = {} if self.log_path else None
        return out

    def flush_log(self):
        if self._log and self.log_path:
            np.savez(self.log_path, **{k: np.array(v) for k, v in self._log.items()})

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
            m.note_step(last["s"], 1.0 / self.info.control_hz, touched=self._contacts.take())
            return last["s"]

        scene.object_states = object_states
        try:
            result = super()._task_step()
        finally:
            scene.object_states = original
        st = last.get("s")
        if st is not None and (DEBUG or self._log is not None):
            self._record(st, m)
        return result

    def _record(self, st, m):
        base = self._base
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        robot = base.scene["left_robot"]
        q = arr(robot.data.joint_pos.torch[0])
        tau = arr(robot.data.applied_torque.torch[0])
        body = robot.find_bodies("gripper")[0][0]
        grip = arr(robot.data.body_link_pose_w.torch[0, body])
        c = m.counts(st)
        if DEBUG and round(self._t * self.info.control_hz) % 5 == 0:
            p = st["pan"]
            print(f"PAN t={self._t:6.2f} pan {np.round(1000 * p.pos, 0).tolist()} tilt {m.tilt_deg(p):5.1f} turned {c['turned']} in {c['inside']} "
                  f"lost {c['lost']} touched {int(m.trial()['touched'])} home {int(m.pan_home(st))} clear {int(m.hands_clear(st))} "
                  f"still {m.trial()['still']:.2f} tau {np.round(tau[:6], 1).tolist()} score {m.score(st):.2f}", flush=True)
        if self._log is not None:
            names = sorted(n for n in st if n.startswith("die_"))
            row = {"t": self._t, "pan": np.r_[st["pan"].pos, st["pan"].quat_xyzw], "dice": np.array([np.r_[st[n].pos, st[n].quat_xyzw] for n in names]),
                   "q": q, "tau": tau, "gripper": grip, "turned": c["turned"], "inside": c["inside"], "lost": c["lost"],
                   "peak": self._contacts.peak, "score": m.score(st)}
            for k, v in row.items():
                self._log.setdefault(k, []).append(v)
            if len(self._log["t"]) % 20 == 0:
                self.flush_log()


def make_embodiment(**kwargs):
    return PanEmbodiment(**kwargs)
