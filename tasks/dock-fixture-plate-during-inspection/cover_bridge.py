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
from cover_camera import configure, floor_stand
from cover_geometry import (
    BODIES,
    SIDES,
    ORIGIN,
    CARRIAGE,
    drive_travel,
    INSPECTION_PROFILES,
)
from cover_scoring import Trial
from cover_observations import measure

ROOT = "/World/envs/env_0/"
RIG = ROOT + "cover_fixture/"


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
        from yam_arena.robot import asset_link_paths

        device = str(base.device)
        if device != "cpu":
            raise RuntimeError(f"Expected CPU PhysX, received {device}")
        self.dt = float(base.sim.get_physics_dt())
        if abs(self.dt - 1 / 240) > 1e-7 or abs(float(base.step_dt) - 0.05) > 1e-6:
            raise RuntimeError("Expected 240/20 Hz")
        physics = PhysxManager.get_physics_sim_view()
        self.views = {n: physics.create_rigid_body_view(RIG + n) for n in BODIES}
        if any(v.count != 1 for v in self.views.values()):
            raise RuntimeError("Missing rigid body")
        self.arms = []
        self.paths = {RIG + n: n for n in BODIES}
        for side in SIDES:
            arm = base.scene[side + "_robot"]
            arm_view = physics.create_articulation_view(ROOT + side.title() + "Arm")
            if arm_view.count != 1:
                raise RuntimeError("Missing YAM articulation " + side)
            self.arms.append(
                (
                    side,
                    arm_view,
                    {
                        n: arm.body_names.index(n)
                        for n in ("link5", "gripper", "tip_left", "tip_right")
                    },
                )
            )
            for link, path in asset_link_paths().items():
                self.paths[ROOT + side.title() + "Arm" + path] = side + "/" + link
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        self.lamps = [
            UsdGeom.Imageable(stage.GetPrimAtPath(RIG + "frame/flag" + n))
            for n in ("Left", "Right", "Skin", "Transfer")
        ]
        self.drive_view = self.views["stageDrive"]
        self.drive_backend = type(self.drive_view.get_transforms()).__module__.split(
            "."
        )[0]
        self.drive_offset = None
        self.inspection_profile = os.environ.get("DOCK_INSPECTION_PROFILE", "nominal")
        self.inspection_end = INSPECTION_PROFILES[self.inspection_profile]
        self.trial = Trial()
        self.events = []
        self.error = None
        self.decode = PhysicsSchemaTools.intToSdfPath
        self.setting = "/physics/disableContactProcessing"
        self.settings = carb.settings.get_settings()
        self.old = self.settings.get(self.setting)
        self.settings.set_bool(self.setting, False)
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(
            self.contacts
        )
        f = os.environ.get("COVER_TRACE")
        self.trace = gzip.open(f, "xt", compresslevel=2) if f else None
        if self.trace:
            self.trace.write(
                json.dumps(
                    dict(
                        kind="reset",
                        inspection_profile=self.inspection_profile,
                        seed=seed,
                        dt=self.dt,
                        physics=device,
                        fixture_sha256=hashlib.sha256(
                            Path(__file__).with_name("cover.usda").read_bytes()
                        ).hexdigest(),
                    )
                )
                + "\n"
            )

    def contacts(self, headers, points):
        try:
            for h in headers:
                if not h.num_contact_data:
                    continue
                a, b, sa, sb = [
                    str(self.decode(getattr(h, k)))
                    for k in ("actor0", "actor1", "collider0", "collider1")
                ]
                if a not in self.paths and b not in self.paths:
                    continue
                start = h.contact_data_offset
                end = start + h.num_contact_data
                if not 0 <= start <= end <= len(points):
                    raise ValueError("Invalid contact point range")
                impulse = 0.0
                for i in range(start, end):
                    p = points[i]
                    v = p.impulse
                    try:
                        v = tuple(v)
                    except TypeError:
                        v = (float(v),)
                    mag = (
                        abs(float(v[0]))
                        if len(v) == 1
                        else abs(
                            sum(
                                float(a) * float(b)
                                for a, b in zip(v, p.normal, strict=True)
                            )
                        )
                    )
                    if not math.isfinite(mag):
                        raise ValueError("Nonfinite impulse")
                    impulse += mag
                self.events.append(
                    (
                        self.paths.get(a, "environment"),
                        self.paths.get(b, "environment"),
                        sa.split("/")[-1],
                        sb.split("/")[-1],
                        impulse / self.dt,
                    )
                )
        except Exception as exc:
            self.error = exc

    def advance_worker(self):
        offset = drive_travel((self.trial.tick + 1) * self.dt, self.inspection_end)
        if offset == self.drive_offset:
            return
        xyz = [a + b for a, b in zip(ORIGIN, CARRIAGE)]
        xyz[0] += offset
        data = np.array([[*xyz, 0, 0, 0, 1]], dtype=np.float32)
        index = np.array([0], dtype=np.int32)
        if self.drive_backend == "torch":
            import torch

            data, index = torch.from_numpy(data), torch.from_numpy(index)
        elif self.drive_backend == "warp":
            import warp as wp

            data, index = (
                wp.array(data, dtype=wp.float32, device="cpu"),
                wp.array(index, dtype=wp.int32, device="cpu"),
            )
        self.drive_view.set_kinematic_targets(data, index)
        self.drive_offset = offset

    def sample(self):
        if self.error:
            raise RuntimeError("Native contact decoding failed") from self.error
        distal = {}
        for side, arm, indices in self.arms:
            tf = array(arm.get_link_transforms())[0]
            distal.update({side + "/" + n: tf[i].tolist() for n, i in indices.items()})
        raw = dict(
            tick=self.trial.tick + 1,
            poses={
                n: array(v.get_transforms()).reshape(7).tolist()
                for n, v in self.views.items()
            },
            twist=array(self.views["cover"].get_velocities()).reshape(6).tolist(),
            distal=distal,
            contacts=self.events,
        )
        before = self.trial.flags.copy()
        self.trial.observe(measure(raw))
        for lamp, old, new in zip(self.lamps, before, self.trial.flags):
            if new and not old:
                lamp.MakeVisible()
        if self.trace:
            self.trace.write(
                json.dumps(
                    dict(raw=raw, score=self.trial.score, reason=self.trial.reason),
                    allow_nan=False,
                    separators=(",", ":"),
                )
                + "\n"
            )
        self.events = []

    def close(self):
        self.sub = None
        if self.trace:
            self.trace.close()
        if self.old is None:
            self.settings.destroy_item(self.setting)
        else:
            self.settings.set_bool(self.setting, self.old)


def install(entry):
    from yam_arena import inspect_embodiment as api

    cls = api.YamArenaEmbodiment
    if getattr(cls, "_cover_path", None) == entry:
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
            if mark.name != "cover_fixture":
                return fallback(mark)
            return Object(
                name=mark.name,
                object_type=ObjectType.BASE,
                spawner_cfg=sim.UsdFileCfg(usd_path=str(entry.with_name("cover.usda"))),
                initial_pose=Pose(
                    position_xyz=tuple(mark.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
                ),
            )

        bot.rig = configure(bot.rig)
        stand = scene.station.camera_stand
        with (
            patch.object(scene, "task_object", spawn),
            patch.object(
                scene.station, "camera_stand", lambda cam: floor_stand(cam, stand)
            ),
            patch.object(bot, "device", "cpu"),
        ):
            return original(bot, goal)

    def start(bot, scene, *, seed=None):
        if not selected(bot):
            return reset(bot, scene, seed=seed)
        if getattr(bot, "_cover", None):
            bot._cover.close()
            bot._env.close()
            bot._env = bot._base = None
        obs = reset(bot, scene, seed=seed)
        bot._cover = Observer(bot._base, seed)
        bot.task.module.monitor = bot._cover
        return obs

    def advance(bot, action):
        if not selected(bot):
            return step(bot, action)
        from yam_arena import scene

        m = bot._cover
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
                if env is bot._env and tuple(names) == ("cover_fixture",)
                else prior(env, names)
            )

        before = m.trial.tick
        with (
            patch.object(engine, "step", sampled),
            patch.object(scene, "object_states", states),
        ):
            result = step(bot, action)
        if m.trial.tick - before != 12:
            raise RuntimeError("Lost physics substeps")
        return result

    def finish(bot):
        if selected(bot) and getattr(bot, "_cover", None):
            bot._cover.close()
            bot._cover = None
        return close(bot)

    cls._ensure_sim = create
    cls.reset = start
    cls.step = advance
    cls.close = finish
    cls._cover_path = entry
