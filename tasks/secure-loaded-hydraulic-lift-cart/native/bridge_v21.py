"""Native Isaac adapter. Fixture and robot measurements are sampled at 240 Hz."""

import atexit, hashlib, json, math, os
from pathlib import Path
from unittest.mock import patch
import numpy as np
from native.fixture_v21 import ROOT, CASES, BASES, Y, HEIGHT, HALF, GRIP_Y, GRIP_Z
from native.qualification_v21 import Qualification
from native.contact_pairs_v21 import PairMonitor
from native.station_v21 import configure, readback

RIG = "/World/envs/env_0/cart_fixture"
BODIES = (
    "frame",
    "deck",
    "crate",
    "left_support",
    "right_support",
    "valve_lever",
    "ram_plunger",
    "crosshead",
    "left_grip",
    "right_grip",
    "valve_knob",
)
PUBLIC = [
    "native/" + s
    for s in (
        "entry.py",
        "bridge_v21.py",
        "fixture_v21.py",
        "qualification_v21.py",
        "station_v21.py",
        "cart_motion_v21.py",
        "scripted_guard_v21.py",
        "entry_layout.py",
        "contact_pairs_v21.py",
        "v21_scenarios/open.usda",
        "v21_scenarios/workwear_weave.png",
    )
] + ["model/__init__.py", "model/human_safety.py"]


def digest(p):
    with p.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def source_id():
    return hashlib.sha256(
        json.dumps({s: digest(ROOT / s) for s in PUBLIC}, sort_keys=True).encode()
    ).hexdigest()


def asset():
    c = os.environ.get("TASK20_SCENARIO", "open")
    exp = os.environ.get("TASK20_MECH_TEST", "")
    if c not in CASES or exp not in ("", "retract") or (c != "open" and not exp):
        raise ValueError("Unsupported fixture")
    return ROOT / "native/v21_scenarios" / f"{c}.usda"


def array(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def observations(p):
    from scipy.spatial.transform import Rotation

    if set(p) != set(BODIES) or any(
        len(x) != 7
        or not np.all(np.isfinite(x))
        or abs(np.linalg.norm(x[3:]) - 1) > 0.002
        for x in p.values()
    ):
        raise ValueError("Invalid rigid-body pose")

    def basis(n):
        return Rotation.from_quat(p[n][3:]).as_matrix()

    frame = basis("frame").T
    beam = basis("crosshead")
    v = frame @ basis("valve_lever")[:, 0]
    knob_center = np.array(p["valve_lever"][:3]) + basis("valve_lever") @ np.array(
        [0.074, 0, 0.080]
    )
    if np.linalg.norm(knob_center - p["valve_knob"][:3]) > 0.006:
        raise ValueError("Disconnected valve knob")
    angles = {}
    gaps = {}
    for s, sign in [("left", 1), ("right", -1)]:
        n = s + "_support"
        top = basis(n)[:, 2]
        axle = np.array(p[n][:3]) + basis(n) @ np.array([0, sign * GRIP_Y, GRIP_Z])
        if np.linalg.norm(axle - p[s + "_grip"][:3]) > 0.006:
            raise ValueError("Disconnected prop bearing: " + s)
        local = frame @ top
        angles[s] = math.degrees(math.atan2(local[0], local[2]))
        pt = beam.T @ (np.array(p[n][:3]) + top * HEIGHT - np.array(p["crosshead"][:3]))
        gaps[s] = (
            float(-HALF - pt[2])
            if abs(pt[0]) <= 0.024 and abs(pt[1] - sign * Y) <= 0.045
            else 1.0
        )
    return dict(
        prop_angle_deg=angles,
        relief_angle_deg=math.degrees(math.atan2(v[1], v[0])),
        bearing_gaps_m=gaps,
        deck_world_z_m=p["deck"][2],
        ram_relative_z_m=p["ram_plunger"][2] - p["frame"][2] - 0.6,
        crate_relative_m=(np.array(p["crate"][:3]) - p["deck"][:3]).tolist(),
        crosshead_roll_deg=math.degrees(math.atan2(beam[2, 1], beam[2, 2])),
    )


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
        self.previous = None
        self.unsafe = None
        self.target = 0.0
        self.was_open = False
        self.experiment = os.environ.get("TASK20_MECH_TEST", "")
        self.qualifier = Qualification(not self.experiment)
        self.qualification_score = 0
        self.contacts = PairMonitor(PhysicsSchemaTools.intToSdfPath)
        stage = omni.usd.get_context().get_stage()
        self.drive = UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(RIG + "/ramSlide"), "linear"
        ).GetTargetPositionAttr()
        self.settings = carb.settings.get_settings()
        self.key = "/physics/disableContactProcessing"
        self.old = self.settings.get(self.key)
        self.settings.set_bool(self.key, False)
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(
            self.contacts.on_contacts
        )
        output = os.environ.get("TASK20_NATIVE_TRACE")
        self.trace = Path(output).open("x") if output else None
        if self.trace:
            self.trace.write(
                json.dumps(
                    dict(
                        kind="header",
                        version=21,
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
        self.proof_before = self.qualifier.proof_command
        if (
            self.was_open
            or self.proof_before
            or (self.experiment == "retract" and self.tick >= 240)
        ):
            self.target = max(-0.235, self.target - 0.045 * self.dt)
        self.drive.Set(self.target)

    def measure(self):
        self.tick += 1
        p = {
            n: array(v.get_transforms()).reshape(7).tolist()
            for n, v in self.views.items()
        }
        r = observations(p)
        self.was_open = r["relief_angle_deg"] >= 45
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
        if self.tick % 12 == 0:
            if (
                self.previous
                and self.tick <= 1680
                and (
                    max(speeds.values()) > 1.2
                    or any(
                        math.dist(hands[s + "_gripper"], self.previous[s + "_gripper"])
                        / 0.05
                        > 0.45
                        for s in BASES
                    )
                )
            ):
                self.unsafe = self.unsafe or "startup overspeed"
            self.previous = hands
        external = False
        for a, b in snap.pairs:
            for load, other in ((a, b), (b, a)):
                if load in [
                    RIG + "/" + n for n in ("deck", "crate", "crosshead", "ram_plunger")
                ] and not other.startswith(RIG + "/"):
                    external = True
                if "/LeftArm/" in load and "/RightArm/" in other:
                    self.unsafe = self.unsafe or "inter-arm collision"
        r.update(
            tick=self.tick,
            poses=p,
            hand_points_w=hands,
            jaw_aperture_m=jaw,
            unsafe_observed=self.unsafe,
            external_load_contact=external,
            collider_pairs=sorted(snap.collider_pairs),
            contact_pairs=sorted(snap.pairs),
            max_joint_speed_rad_s=speeds,
            ram_target_m=self.target,
            independent_release_command=self.proof_before,
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
    from native.scripted_guard_v21 import install_task20_guard

    install_task20_guard(entry)
