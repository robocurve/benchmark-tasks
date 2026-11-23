"""Task-local compound USD spawner and read-only native substep measurement."""

import atexit
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
from unittest.mock import patch

import numpy as np
from isaac.camera import configure, floor_stand
from isaac.readout import BODIES, LINKS, Readout
from task.run import lift_moment
from task.scoring import score
from scipy.spatial.transform import Rotation

ROOT = "/World/envs/env_0/"
RIG = ROOT + "heel_fixture/"


def array(a):
    a = getattr(a, "torch", a)
    if hasattr(a, "detach"):
        a = a.detach().cpu().numpy()
    return np.asarray(a.numpy() if hasattr(a, "numpy") else a)


class Observer:
    def __init__(self, base, seed):
        import carb.settings
        from isaaclab_physx.physics import PhysxManager
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        self.dt = float(base.sim.get_physics_dt())
        if abs(self.dt - 1 / 240) > 1e-7 or abs(base.step_dt - 0.05) > 1e-6:
            raise ValueError("Expected 240 Hz physics, 20 Hz action")
        physics = PhysxManager.get_physics_sim_view()
        self.views = {n: physics.create_rigid_body_view(RIG + n) for n in BODIES}
        self.arms = {}
        for side in ("left", "right"):
            arm = base.scene[side + "_robot"]
            view = physics.create_articulation_view(ROOT + side.title() + "Arm")
            self.arms[side] = view, [arm.body_names.index(n) for n in LINKS]
        if any(
            v.count != 1
            for v in [*self.views.values(), *(x[0] for x in self.arms.values())]
        ):
            raise ValueError("Missing native actor")
        self.settings = json.loads(os.environ["HEEL_CONDITION"])
        self.read = Readout(self.settings)
        self.tick = 0
        self.rows = []
        self.heel_sum = 0.0
        self.assist = 0.0
        self.error = None
        self.decode = PhysicsSchemaTools.intToSdfPath
        self.config = carb.settings.get_settings()
        self.key = "/physics/disableContactProcessing"
        self.old = self.config.get(self.key)
        self.config.set_bool(self.key, False)
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(
            self.contacts
        )
        self.trace = gzip.open(os.environ["HEEL_TRACE"], "xt", compresslevel=2)
        self.trace.write(
            json.dumps(
                dict(
                    kind="reset",
                    seed=seed,
                    settings=self.settings,
                    dt=self.dt,
                    fixture_sha256=hashlib.sha256(
                        Path(os.environ["HEEL_USD"]).read_bytes()
                    ).hexdigest(),
                )
            )
            + "\n"
        )
        self.score = 0.0

    def contacts(self, headers, points):
        try:
            for h in headers:
                if not h.num_contact_data:
                    continue
                shapes = [
                    str(self.decode(getattr(h, k))) for k in ("collider0", "collider1")
                ]
                if RIG + "shin/heel" not in shapes:
                    continue
                for i in range(
                    h.contact_data_offset, h.contact_data_offset + h.num_contact_data
                ):
                    p = points[i]
                    imp = np.asarray(p.impulse).ravel()
                    self.heel_sum += (
                        abs(
                            float(imp[0])
                            if len(imp) == 1
                            else float(imp @ np.asarray(p.normal))
                        )
                        / self.dt
                    )
        except Exception as exc:
            self.error = exc

    def posture(self):
        legs = [
            array(self.views[n].get_transforms()).reshape(7)
            for n in ("upper_leg", "shin")
        ]
        rot = [Rotation.from_quat(p[3:]) for p in legs]
        angle = float((rot[0].inv() * rot[1]).as_rotvec()[0])
        rates = [
            float(array(self.views[n].get_velocities()).reshape(6)[3])
            for n in ("upper_leg", "shin")
        ]
        return angle, rates[1] - rates[0]

    def capture(self):
        poses = {
            n: array(v.get_transforms()).reshape(7).tolist()
            for n, v in self.views.items()
        }
        for s, (v, ids) in self.arms.items():
            pp = array(v.get_link_transforms())[0]
            poses.update({s + "/" + n: pp[i].tolist() for n, i in zip(LINKS, ids)})
        return dict(
            poses=poses,
            heel_n=self.heel_sum / 12,
            assist_nm=self.assist,
            knee_rate=self.posture()[1],
        )

    def initialize(self):
        self.initial = self.capture()
        _, self.obs = self.read.sample(self.initial)
        self.trace.write(json.dumps({"initial": self.initial}) + "\n")

    def advance_worker(self):
        angle, rate = self.posture()
        self.assist = lift_moment(
            angle, rate, self.tick * self.dt, self.settings.get("delay", 0)
        )
        if self.assist:
            for n, sign in (("shin", 1), ("upper_leg", -1)):
                view = self.views[n]
                raw = view.get_transforms()
                torque = np.array([[sign * self.assist, 0, 0]], dtype=np.float32)
                ids = np.array([0], dtype=np.int32)
                backend = type(raw).__module__.split(".")[0]
                if backend == "torch":
                    import torch

                    torque, ids = torch.from_numpy(torque), torch.from_numpy(ids)
                elif backend == "warp":
                    import warp as wp

                    torque, ids = (
                        wp.array(torque, device="cpu"),
                        wp.array(ids, device="cpu"),
                    )
                view.apply_forces_and_torques_at_position(None, torque, None, ids, True)

    def sample(self):
        if self.error:
            raise self.error
        self.tick += 1
        if self.tick % 12:
            return
        raw = self.capture()
        self.heel_sum = 0
        w, self.obs = self.read.sample(raw)
        row = dict(tick=self.tick // 12 - 1, witness=w, action=self.action)
        self.rows.append(row)
        self.trace.write(
            json.dumps(dict(raw=raw, **row), allow_nan=False, separators=(",", ":"))
            + "\n"
        )
        if len(self.rows) == 280:
            self.score = score(self.rows)[0]

    def close(self):
        self.sub = None
        self.trace.close()
        if self.old is None:
            self.config.destroy_item(self.key)
        else:
            self.config.set_bool(self.key, self.old)


def install(entry):
    from yam_arena import inspect_embodiment as api

    cls = api.YamArenaEmbodiment
    if getattr(cls, "_heel_path", None) == entry:
        return
    original, reset, step, close = cls._ensure_sim, cls.reset, cls.step, cls.close

    def selected(bot):
        return bot.task is not None and bot.task.path == entry

    def create(bot, goal):
        if not selected(bot) or bot._env is not None:
            return original(bot, goal)
        if api._APP is None:
            from isaaclab.app import AppLauncher

            api._APP = AppLauncher(
                headless=bot.headless, enable_cameras=True, device=bot.device
            ).app
            atexit.register(api._close_app_at_exit)
        import isaaclab.sim as sim
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from yam_arena import scene

        fallback = scene.task_object

        def spawn(mark):
            if mark.name != "heel_fixture":
                return fallback(mark)
            return Object(
                name=mark.name,
                object_type=ObjectType.BASE,
                spawner_cfg=sim.UsdFileCfg(usd_path=os.environ["HEEL_USD"]),
                initial_pose=Pose(
                    position_xyz=tuple(mark.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
                ),
            )

        bot.rig = configure(bot.rig)
        # The bed replaces the station table. Bases and both calibrated wrist cameras remain on the station rig.
        room = scene.task_room

        def bedside_room(*args, **kwargs):
            return [item for item in room(*args, **kwargs) if item.name != "table"]

        stand = scene.station.camera_stand
        with (
            patch.object(scene, "task_object", spawn),
            patch.object(scene, "task_room", bedside_room),
            patch.object(
                scene.station, "camera_stand", lambda cam: floor_stand(cam, stand)
            ),
            patch.object(bot, "device", "cpu"),
        ):
            return original(bot, goal)

    def start(bot, scene, *, seed=None):
        if not selected(bot):
            return reset(bot, scene, seed=seed)
        if getattr(bot, "_heel", None):
            bot._heel.close()
            bot._env.close()
            bot._env = bot._base = None
        obs = reset(bot, scene, seed=seed)
        bot._heel = Observer(bot._base, seed)
        bot._heel.initialize()
        bot.task.module.monitor = bot._heel
        return obs

    def advance(bot, action):
        if not selected(bot):
            return step(bot, action)
        from yam_arena import scene

        m = bot._heel
        engine = bot._base.sim
        saved = engine.step
        prior = scene.object_states

        def sampled(*args, **kw):
            m.advance_worker()
            result = saved(*args, **kw)
            m.sample()
            return result

        def states(env, names):
            return (
                {}
                if env is bot._env and tuple(names) == ("heel_fixture",)
                else prior(env, names)
            )

        target = np.asarray(action.data, dtype=float).reshape(14)
        m.action = np.clip(target, bot._low, bot._high).tolist()
        before = m.tick
        with (
            patch.object(engine, "step", sampled),
            patch.object(scene, "object_states", states),
        ):
            result = step(bot, action)
        if m.tick - before != 12:
            raise RuntimeError("Lost physics substeps")
        return result

    def finish(bot):
        if selected(bot) and getattr(bot, "_heel", None):
            bot._heel.close()
            bot._heel = None
        return close(bot)

    cls._ensure_sim = create
    cls.reset = start
    cls.step = advance
    cls.close = finish
    cls._heel_path = entry
