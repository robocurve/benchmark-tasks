"""Scenario-scoped adapter for Inspect Robots and IsaacLab-Arena-YAM."""

import atexit
import json
import os
from pathlib import Path
from unittest.mock import patch

import numpy as np

from evd.camera_clearance import SIDES, WristClearance
from evd.cameras import configure
from evd.identity import source_id
from evd.instrument import Monitor
from evd.layout import LINKS, MODES, NAMES, ROOT


def host_array(value):
    value = getattr(value, "torch", value)
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return value.numpy() if hasattr(value, "numpy") else np.asarray(value)


def live_stage(sim):
    stage = getattr(sim, "stage", None)
    if stage is None:
        # Older runtimes expose the active stage through Kit after app launch.
        from omni.usd import get_context

        stage = get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath("/World/envs/env_0/water_trainer").IsValid():
        raise RuntimeError("The live USD stage does not contain the active water trainer")
    return stage


class NativeMonitor:
    def __init__(self, base, mode, out):
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdGeom, UsdPhysics

        self.mode = mode
        self.base = base
        case, style = MODES[mode]
        self.case = case
        if (
            abs(base.sim.get_physics_dt() - 1 / 240) > 1e-7
            or abs(base.step_dt - 0.05) > 1e-6
        ):
            raise RuntimeError("Expected 240/20 Hz")
        prefix = "/World/envs/env_0/water_trainer"
        sim = PhysxManager.get_physics_sim_view()
        self.views = {n: sim.create_rigid_body_view(prefix + "/" + n) for n in NAMES}
        if any(v.count != 1 for v in self.views.values()):
            raise RuntimeError("Incomplete trainer body set")
        # Resolve the real articulation links in the composed stage, then read the
        # same low-level rigid-body tensors used by the fixture at every substep.
        from pxr import Usd

        stage = live_stage(base.sim)
        self.arm = {}
        for side in ("left", "right"):
            root = stage.GetPrimAtPath("/World/envs/env_0/" + side.title() + "Arm")
            links = {
                p.GetName(): p
                for p in Usd.PrimRange(root)
                if p.HasAPI(UsdPhysics.RigidBodyAPI)
            }
            views = [
                sim.create_rigid_body_view(str(links[name].GetPath())) for name in LINKS
            ]
            if any(v.count != 1 for v in views):
                raise RuntimeError("Missing live YAM link on " + side)
            self.arm[side] = views
        self.drive = UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(prefix + "/tapHinge"), "angular"
        ).GetTargetPositionAttr()
        self.is_open = False
        self.liquid_ops = UsdGeom.Xformable(
            stage.GetPrimAtPath(prefix + "/mast/waterColumn")
        ).GetOrderedXformOps()
        self.water = UsdGeom.Xformable(
            stage.GetPrimAtPath(prefix + "/mast/volumeFloat")
        ).GetOrderedXformOps()[0]
        self.tube = UsdGeom.BasisCurves.Get(
            stage, prefix + "/mast/flexibleLoop"
        ).GetPointsAttr()
        offsets = {s: base.scene[s + "_cam"].cfg.offset.pos for s in SIDES}
        self.wrist_clearance = WristClearance(ROOT / "assets" / (case + ".usda"), offsets)
        self.out = out
        self.core = Monitor(
            case, style == "blocked", out / "trace.jsonl.gz" if out else None
        )
        self.capture_contacts(base, stage)
        try:
            self.step()
        except Exception:
            self.close()
            raise

    def capture_contacts(self, base, stage):
        import carb.settings
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        from evd.pairs import ContactBook

        self.book = ContactBook(PhysicsSchemaTools.intToSdfPath)
        self.settings = carb.settings.get_settings()
        key = "/physics/disableContactProcessing"
        self.old = self.settings.get(key)
        self.settings.set_bool(key, False)
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(
            self.book.event
        )

    def step(self):
        from pxr import Gf

        poses = {
            n: host_array(v.get_transforms()).reshape(7) for n, v in self.views.items()
        }
        arms = {
            side: np.stack([host_array(v.get_transforms()).reshape(7) for v in views])
            for side, views in self.arm.items()
        }
        f = self.core.sample(self.core.tick + 1, poses, arms)
        if (not self.is_open and f.tap_deg > 60) or (self.is_open and f.tap_deg < 30):
            self.is_open = not self.is_open
            self.drive.Set(-90.0 if self.is_open else 0.0)
        self.book.advance(f.tick)
        self.wrist_clearance.sample(f.tick, poses, arms)
        if f.tick % 12 == 0:
            from evd.visual import column_geometry, loop_points

            center, size, surface = column_geometry(self.core.volume)
            self.liquid_ops[0].Set(Gf.Vec3d(*center))
            self.liquid_ops[1].Set(Gf.Vec3f(*size))
            self.water.Set(Gf.Vec3d(*surface))

            # USD Python bindings require built-in floats at the PhysX tensor boundary.
            self.tube.Set(
                [
                    Gf.Vec3f(*map(float, p))
                    for p in loop_points(poses["patient"][2], poses["carriage"][2])
                ]
            )
            if self.out:
                row = {
                    **self.core.report(),
                    "source_id": source_id_cached(),
                    "case": self.case,
                    "mode": self.mode,
                    "contacts": self.book.summary(),
                    "wrist_envelopes": self.wrist_clearance.report(),
                }
                import os

                temp = self.out / "progress.tmp"
                temp.write_text(json.dumps(row))
                os.replace(temp, self.out / "progress.json")
                self.core.trace.flush()

    def close(self):
        self.sub = None
        if self.old is None:
            self.settings.destroy_item("/physics/disableContactProcessing")
        else:
            self.settings.set_bool("/physics/disableContactProcessing", self.old)
        self.core.close()


from functools import lru_cache

source_id_cached = lru_cache(maxsize=1)(source_id)


def install(entry):
    from yam_arena import inspect_embodiment as bridge
    from yam_arena.scripted import YamScriptedPolicy

    cls = bridge.YamArenaEmbodiment
    if getattr(cls, "_evd39_entry", None) == entry:
        return
    ensure, reset, step, close = cls._ensure_sim, cls.reset, cls.step, cls.close

    def selected(bot):
        return getattr(bot, "task", None) is not None and bot.task.path == entry

    def mode():
        return os.environ.get("EVD_TRIAL", "reference")

    def create(bot, goal):
        if not selected(bot) or bot._env is not None:
            return ensure(bot, goal)
        if bridge._APP is None:
            from isaaclab.app import AppLauncher

            bridge._APP = AppLauncher(
                headless=bot.headless, enable_cameras=True, device=bot.device
            ).app
            atexit.register(bridge._close_app_at_exit)
        from isaaclab import sim
        from isaaclab.scene.interactive_scene import InteractiveScene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from yam_arena import scene

        fallback = scene.task_object
        old_init = InteractiveScene.__init__

        def spawn(mark):
            if mark.name != "water_trainer":
                return fallback(mark)
            usd = ROOT / "assets" / (MODES[mode()][0] + ".usda")
            return Object(
                name=mark.name,
                object_type=ObjectType.BASE,
                spawner_cfg=sim.UsdFileCfg(usd_path=str(usd)),
                initial_pose=Pose(
                    position_xyz=mark.pos, rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
                ),
            )

        def configured(inst, cfg, *args, **kw):
            configure(cfg)
            for side in ("left", "right"):
                getattr(cfg, side + "_robot").spawn.activate_contact_sensors = True
            return old_init(inst, cfg, *args, **kw)

        with (
            patch.object(scene, "task_object", spawn),
            patch.object(InteractiveScene, "__init__", configured),
            patch.object(bot, "device", "cpu"),
        ):
            return ensure(bot, goal)

    def start(bot, scene, *, seed=None):
        if not selected(bot):
            return reset(bot, scene, seed=seed)
        if getattr(bot, "_evd39", None):
            bot._evd39.close()
            bot._env.close()
            bot._env = bot._base = None
        obs = reset(bot, scene, seed=seed)
        out = Path(os.environ["EVD_OUTPUT"]) if "EVD_OUTPUT" in os.environ else None
        bot._evd39 = NativeMonitor(bot._base, mode(), out)
        bot.task.module.monitor = bot._evd39
        return obs

    def advance(bot, action):
        if not selected(bot):
            return step(bot, action)
        from yam_arena import scene

        m = bot._evd39
        engine = bot._base.sim
        orig = engine.step
        object_states = scene.object_states

        def sampled(*a, **kw):
            result = orig(*a, **kw)
            m.step()
            return result

        def states(env, names):
            return (
                {}
                if env is bot._env and tuple(names) == ("water_trainer",)
                else object_states(env, names)
            )

        before = m.core.tick
        with (
            patch.object(engine, "step", sampled),
            patch.object(scene, "object_states", states),
        ):
            result = step(bot, action)
        if m.core.tick != before + 12:
            raise RuntimeError("Native substep count changed")
        return result

    def finish(bot):
        if selected(bot) and getattr(bot, "_evd39", None):
            bot._evd39.close()
            bot._evd39 = None
        return close(bot)

    cls._ensure_sim = create
    cls.reset = start
    cls.step = advance
    cls.close = finish
    cls._evd39_entry = entry
    old_reset, old_act = YamScriptedPolicy.reset, YamScriptedPolicy.act

    def plan(policy, scene):
        if policy._task is None or policy._task.path != entry:
            return old_reset(policy, scene)
        from evd.policy import make_plan

        policy.evd_plan = make_plan(mode())

    def act(policy, obs):
        if policy._task is None or policy._task.path != entry:
            return old_act(policy, obs)
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack

        from evd.policy import action

        return ActionChunk(
            actions=[
                Action(
                    data=pack(
                        *action(policy.evd_plan, float(obs.state_time) + policy._dt)
                    )
                )
            ],
            control_hz=1 / policy._dt,
        )

    YamScriptedPolicy.reset = plan
    YamScriptedPolicy.act = act
