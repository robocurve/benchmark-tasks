"""Native Isaac adapter. Fixture and robot measurements are sampled at 240 Hz."""

import atexit, hashlib, json, math, os
from pathlib import Path
from unittest.mock import patch
import numpy as np
from native.fixture import ROOT, CASES, BASES, OPEN_DEG, withdrawal_target
from native.qualification import Qualification
from native.contact_pairs import PairMonitor
from native.station import configure, readback

RIG = "/World/envs/env_0/cart_fixture"
from native.measurements import BODIES, observations
PUBLIC = sorted(p.relative_to(ROOT).as_posix() for p in (ROOT/'native').rglob('*')
                if p.is_file() and p.suffix in ('.py','.usda','.png'))


def digest(p):
    with p.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def source_id():
    return hashlib.sha256(
        json.dumps({s: digest(ROOT / s) for s in PUBLIC}, sort_keys=True).encode()
    ).hexdigest()


def asset():
    c = os.environ.get("TASK20_SCENARIO", "open")
    if c not in CASES:
        raise ValueError("Unsupported fixture")
    return ROOT / "native/assets" / f"{c}.usda"


def array(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


class Monitor:
    def __init__(self, bot, seed):
        import omni.usd, carb.settings
        from pxr import PhysicsSchemaTools, UsdPhysics
        from isaaclab_physx.physics import PhysxManager
        from omni.physx import get_physx_simulation_interface

        self.dt = float(bot.sim.get_physics_dt())
        if abs(self.dt - 1 / 240) > 1e-7 or abs(bot.step_dt - 0.05) > 1e-6:
            raise RuntimeError("Expected 240/20 Hz")
        station = readback()
        view = PhysxManager.get_physics_sim_view()
        self.views = {n: view.create_rigid_body_view(RIG + "/" + n) for n in BODIES}
        if any(v.count != 1 for v in self.views.values()):
            raise RuntimeError("Missing fixture body")
        self.robots = {}
        for s in BASES:
            arm = bot.scene[s + "_robot"]
            self.robots[s] = (
                arm,
                [
                    list(arm.body_names).index(n)
                    for n in ("gripper", "tip_left", "tip_right")
                ],
                [list(arm.joint_names).index(n) for n in ("joint7", "joint8")],
            )
        self.tick = 0
        self.target = 0.0
        self.was_open = False
        self.experiment = os.environ.get("TASK20_MECH_TEST", "")
        if self.experiment: raise ValueError("Native episodes use the unmodified occupied reset")
        self.qualifier = Qualification(not self.experiment)
        self.qualification_score = 0
        self.control = os.environ.get("TASK20_CONTROL", "policy")
        self.contacts = PairMonitor(PhysicsSchemaTools.intToSdfPath)
        stage = omni.usd.get_context().get_stage()
        self.drive = UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(RIG + "/ramSlide"), "linear"
        ).GetTargetPositionAttr()
        self.worker_drive = UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(RIG + "/workerSlide"), "linear").GetTargetPositionAttr()
        self.settings = carb.settings.get_settings()
        self.key = "/physics/disableContactProcessing"
        self.old = self.settings.get(self.key)
        self.settings.set_bool(self.key, False)
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(
            self.contacts.on_contacts
        )
        output = os.environ.get("TASK20_NATIVE_TRACE")
        from gzip import open as gzip_open
        self.trace = gzip_open(output,"xt",compresslevel=2) if output else None
        if self.trace:
            self.trace.write(
                json.dumps(
                    dict(
                        kind="header",
                        version=22,
                        dt=self.dt,
                        seed=seed,
                        station=station,
                        source_id=source_id(),
                        scene_sha256=digest(asset()),
                        case=asset().stem,
                        experiment=self.experiment,
                        control=os.environ.get("TASK20_CONTROL", "policy"),
                    )
                )
                + "\n"
            )

    def before_step(self):
        self.worker_drive.Set(withdrawal_target(self.tick,self.control))
        if self.was_open:
            self.target = max(-0.235, self.target - 0.045 * self.dt)
        self.drive.Set(self.target)

    def measure(self):
        self.tick += 1
        p = {
            n: array(v.get_transforms()).reshape(7).tolist()
            for n, v in self.views.items()
        }
        r = observations(p)
        self.was_open = r["relief_angle_deg"] >= OPEN_DEG
        snap = self.contacts.drain()
        hands = {}
        jaw = {}
        speeds = {}
        for s, (arm, ids, joints) in self.robots.items():
            fingers = array(arm.data.joint_pos)[0, joints]
            jaw[s] = -float(np.sum(fingers))
            speeds[s] = float(np.max(np.abs(array(arm.data.joint_vel)[0, :6])))
            for n, xyz in zip(
                ("gripper", "tip_left", "tip_right"),
                array(arm.data.body_pos_w)[0, ids],
                strict=True,
            ):
                hands[s + "_" + n] = xyz.tolist()
        vertical, worker = 0., 0.
        for (a,b),(vertical_impulse,total_impulse) in snap.impulses.items():
            for obj,other in ((a,b),(b,a)):
                if obj == RIG+"/worker" and ("Arm/" in other or other in (RIG+"/deck",RIG+"/crate")):
                    worker += total_impulse / self.dt
                if obj.startswith(RIG+"/") and obj.rsplit("/",1)[-1] not in ("frame","worker","valve_lever","valve_knob") and "Arm/" in other:
                    vertical += vertical_impulse / self.dt
        r.update(
            tick=self.tick,
            poses=p,
            hand_points_w=hands,
            jaw_aperture_m=jaw,
            external_vertical_n=vertical, worker_force_n=worker,
            collider_pairs=sorted(snap.collider_pairs),
            contact_pairs=sorted(snap.pairs),
            max_joint_speed_rad_s=speeds,
            ram_target_m=self.target,
            worker_drive_target_m=withdrawal_target(self.tick-1,self.control),
        )
        verdict = self.qualifier.observe(r)
        self.qualification_score = verdict["score"]
        if self.trace:
            self.trace.write(
                json.dumps(
                    dict(**r, qualification=verdict, score=self.qualification_score),
                    allow_nan=False,
                    separators=(",", ":"),
                )
                + "\n"
            )

    def close(self):
        self.sub = None
        if self.old is None:
            self.settings.destroy_item(self.key)
        else:
            self.settings.set_bool(self.key, self.old)
        if self.trace:
            self.trace.close()


def install(entry):
    from yam_arena import inspect_embodiment as bridge

    cls = bridge.YamArenaEmbodiment
    if getattr(cls, "_task20_entry", None) == entry:
        return
    ensure, reset, step, close = cls._ensure_sim, cls.reset, cls.step, cls.close

    def selected(bot):
        return getattr(bot, "task", None) is not None and bot.task.path == entry

    def create(bot, goal):
        if not selected(bot) or bot._env is not None:
            return ensure(bot, goal)
        if bridge._APP is None:
            from isaaclab.app import AppLauncher

            bridge._APP = AppLauncher(
                headless=bot.headless, enable_cameras=True, device=bot.device
            ).app
            atexit.register(bridge._close_app_at_exit)
        from yam_arena import scene
        import isaaclab.sim as sim
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose

        fallback = scene.task_object

        def spawn(mark):
            if mark.name != "cart_fixture":
                return fallback(mark)
            selected_asset = asset()
            if not selected_asset.is_file():
                raise FileNotFoundError(selected_asset)
            return Object(
                name=mark.name,
                object_type=ObjectType.BASE,
                spawner_cfg=sim.UsdFileCfg(usd_path=str(selected_asset)),
                initial_pose=Pose(
                    position_xyz=tuple(mark.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
                ),
            )

        # Task20 station layout must be corrected BEFORE Isaac/PhysX spawns
        # the camera stand; changing USD transforms after simulation begins
        # would not reliably update PhysX collision actors.
        from isaaclab.scene.interactive_scene import InteractiveScene

        orig_scene_init = InteractiveScene.__init__
        captured = []

        def init_with_clear_camera_stand(inst, cfg, *args, **kwargs):
            if captured:
                raise RuntimeError(
                    "Unexpected multiple Task20 YAM scene initializations"
                )
            layout = configure(cfg)
            captured.append(layout)
            return orig_scene_init(inst, cfg, *args, **kwargs)

        with (
            patch.object(scene, "task_object", spawn),
            patch.object(bot, "device", "cpu"),
            patch.object(InteractiveScene, "__init__", init_with_clear_camera_stand),
        ):
            result = ensure(bot, goal)
        if len(captured) != 1 or getattr(bot, "_base", None) is None:
            raise RuntimeError(
                "Task20 camera stand native scene hook did not spawn exactly once"
            )
        bot._base._task20_station_layout = captured[0]
        print(
            "TASK20_CAMERA_STAND_NATIVE_SPAWN: "
            + json.dumps(captured[0], sort_keys=True),
            flush=True,
        )
        return result

    def start(bot, scene, *, seed=None):
        if not selected(bot):
            return reset(bot, scene, seed=seed)
        if getattr(bot, "_task20", None):
            bot._task20.close()
            bot._env.close()
            bot._env = bot._base = None
        obs = reset(bot, scene, seed=seed)
        bot._task20 = Monitor(bot._base, seed)
        bot.task.module.monitor = bot._task20
        return obs

    def advance(bot, action):
        if not selected(bot):
            return step(bot, action)
        from yam_arena import scene

        m = bot._task20
        engine = bot._base.sim
        tick = engine.step
        states = scene.object_states

        def sampled(*a, **kw):
            m.before_step()
            result = tick(*a, **kw)
            m.measure()
            return result

        def get(env, names):
            return (
                {}
                if env is bot._env and tuple(names) == ("cart_fixture",)
                else states(env, names)
            )

        before = m.tick
        with (
            patch.object(engine, "step", sampled),
            patch.object(scene, "object_states", get),
        ):
            result = step(bot, action)
        if m.tick - before != 12:
            raise RuntimeError("Native substeps missing: required 12 per action")
        if m.trace:
            m.trace.flush()
        return result

    def finish(bot):
        if selected(bot) and getattr(bot, "_task20", None):
            bot._task20.close()
            bot._task20 = None
        return close(bot)

    cls._ensure_sim = create
    cls.reset = start
    cls.step = advance
    cls.close = finish
    cls._task20_entry = entry
    # YAM ScriptedPolicy independently stores base frames for IK. Patch it only
    # for this task; no effect on Dream or other benchmark tasks.
    from native.scripted_guard import install_task20_guard

    install_task20_guard(entry)
