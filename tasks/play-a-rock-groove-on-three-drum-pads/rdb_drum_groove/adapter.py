"""Add the three drum pads and the two sticks to the stock Inspect Robots YAM station; put a stick between each
gripper's finger pads at reset (held there by friction only once the fingers have closed); run the pads' trigger on
every physics step (240 Hz); and score the trial from its hits.

Pattern: ``rdb_spark_plug/adapter.py`` (fixture USD added to the station, physics tensor views, contact views, the
review camera). New here: ``sim.step`` is wrapped, so a callback runs before and after every physics substep (Isaac
Lab's ManagerBasedEnv calls it once per substep for PhysX: ``_physics_handles_decimation`` is False).

Per physics step (``_post_physics``): each stick's pose and velocity, its contact force on each of the five trigger
zones, the finger pads' force on it (is it held), and every other robot part's force on the pads (a gripper or arm
striking a pad, over 2 N). Per control step (``step``): the end condition (both sticks held, both tips at least 50 mm
from every pad), and the score (``judge_midi.judge`` over the hits so far) when the end condition has held for 1 s
and at the last step.
"""

from dataclasses import replace
import math
from pathlib import Path

from inspect_robots import StepResult
import numpy as np
from yam_arena.constants import TCP_OFFSET
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as F
from . import grading
from . import hits as H
from . import layout as L
from . import poses

CURRENT = None
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def segment_gap(a0, a1, b0, b1):
    """Shortest distance between segments a0-a1 and b0-b1."""
    u, v, w = a1 - a0, b1 - b0, a0 - b0
    a, b, c, d, e = u @ u, u @ v, v @ v, u @ w, v @ w
    den = a * c - b * b
    s = 0. if den < 1e-12 else min(max((b * e - c * d) / den, 0.), 1.)
    t = min(max((b * s + e) / c, 0.), 1.) if c > 1e-12 else 0.
    s = min(max((b * t - d) / a, 0.), 1.) if a > 1e-12 else 0.
    return float(np.linalg.norm(w + s * u - t * v))


class DrumArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None, headless=True, review=True, step_limit=F.MAX_STEPS):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=F.CONTROL_HZ,
                         cam_width=320, cam_height=176, headless=headless)
        self.info = replace(self.info, name="yam_drum_groove",
                            environment_id="isaaclab-arena-0.3.1-yam-drum-groove",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with three Alesis Nitro Max pads at real size (a 10 in "
                                 "hi-hat pad in front of the left arm, a 10 in mesh snare and an 8 in mesh tom in front "
                                 "of the right arm) and a Vic Firth 5A drumstick held by friction in each gripper, its "
                                 "finger pads on the stick's upper and lower sides. Observations are overhead RGB, both "
                                 "wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.review = review
        self.step_limit = step_limit
        self.recorder = None
        self.hooked = False
        self.hold = False
        self.trial_on = False
        self.tick = 0
        self.k = 0
        self.final_grade = None
        self.demo_state = None
        self.state_after_reset = None
        CURRENT = self

    # -- Isaac ------------------------------------------------------------------------------------------------------
    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        from contextlib import nullcontext
        import os

        import yam_arena.inspect_embodiment as inspect_yam

        from .recording import review_camera

        def make_env(embodiment, *, device, control_hz, **kwargs):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene

            objects = scenes.task_room("station", embodiment.rig.top_cam)
            objects.append(Object(name="drum_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(F.USD))))
            env = IsaacLabArenaEnvironment(name="yam_drum_groove", scene=Scene(objects),
                                           embodiment=embodiment, env_cfg_callback=scenes.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=device)).make_registered()

        if not self.review and inspect_yam._APP is None:   # start Kit as the runtime does (no review camera)
            import atexit

            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            inspect_yam._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(inspect_yam._close_app_at_exit)
        with (review_camera(headless=self.headless) if self.review else nullcontext()):
            from yam_arena import scene as scenes

            stock = scenes.make_env
            scenes.make_env = make_env
            try:
                super()._ensure_sim(None)
            finally:
                scenes.make_env = stock

    def physics_settings(self):
        """The physics scene's settings that matter here (written into RESULT.json)."""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        out = {}
        for prim in stage.Traverse():
            if prim.GetTypeName() == "PhysicsScene":
                for a in prim.GetAttributes():
                    n = a.GetName()
                    if any(k in n for k in ("CCD", "bounce", "GPUDynamics", "broadphase", "solverType", "Determinism",
                                            "timeStepsPerSecond")):
                        v = a.Get()
                        out[n] = v if isinstance(v, (int, float, str, bool)) or v is None else str(v)
            elif str(prim.GetPath()).endswith("/" + F.STICKS["right"]):
                for n in ("physxRigidBody:enableCCD", "physxRigidBody:enableSpeculativeCCD"):
                    a = prim.GetAttribute(n)
                    out["stick:" + n] = a.Get() if a else None
        return out

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()

        def find(name):
            found = [str(p.GetPath()) for p in stage.Traverse()
                     if p.GetName() == name and str(p.GetPath()).startswith(root + "/")]
            if len(found) != 1:
                raise RuntimeError(f"Expected one {name}, found {found}")
            return found[0]

        kit, sticks = find(F.KIT), find(F.ACTORS)
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.stick_paths = [f"{sticks}/{F.STICKS[arm]}" for arm in F.ARMS]
        self.stick_view = self.physics.create_rigid_body_view(self.stick_paths)
        self.stick_ids = self._torch.arange(2, dtype=self._torch.int32, device=self._base.device)
        self.zone_paths = [f"{kit}/{z}" for z in F.SURFACES]
        self.stick_contacts = self.physics.create_rigid_contact_view(self.stick_paths, [self.zone_paths] * 2)
        links = asset_link_paths()
        arm_root = {arm: root + "/" + arm.title() + "Arm" for arm in F.ARMS}
        self.gripper_view = self.physics.create_rigid_body_view([arm_root[arm] + links["gripper"] for arm in F.ARMS])
        fingers = [[arm_root[arm] + links["tip_left"], arm_root[arm] + links["tip_right"]] for arm in F.ARMS]
        self.finger_contacts = self.physics.create_rigid_contact_view(self.stick_paths, fingers)
        hand = [arm_root[arm] + links[k] for arm in F.ARMS for k in HAND_LINKS]
        self.hand_names = [f"{arm} {k}" for arm in F.ARMS for k in HAND_LINKS]
        self.hand_contacts = self.physics.create_rigid_contact_view(hand, [self.zone_paths] * len(hand))
        self.cross_contacts = self.physics.create_rigid_contact_view([self.stick_paths[0]],
                                                                     [[self.stick_paths[1]] + hand[len(HAND_LINKS):]])
        self.zone_frames = {}
        for z in F.SURFACES:
            c, n = F.pad_frame(F.ZONE_PAD[z])
            self.zone_frames[z] = (np.array(c), np.array(n))

    def _install_hook(self):
        if self.hooked:
            return
        sim = self._base.sim
        original = sim.step

        def step(*args, **kwargs):
            self._pre_physics()
            out = original(*args, **kwargs)
            self._post_physics()
            return out

        sim.step = step
        self.hooked = True

    # -- per physics step ---------------------------------------------------------------------------------------------
    def gripper_poses(self):
        g = array(self.gripper_view.get_transforms())
        return {arm: (g[i, :3] - (0., 0., F.TOP), quat_xyzw_to_matrix(g[i, 3:7])) for i, arm in enumerate(F.ARMS)}

    def held_pose(self, arm, gp):
        p, R = gp[arm]
        return p + R @ np.array(TCP_OFFSET) - poses.GRIP_DEPTH * R[:, 2], R @ poses.stick_in_gripper()

    def _pre_physics(self):
        if not self.hold:
            return
        from yam_arena.ik import matrix_to_quat_xyzw

        torch = self._torch
        gp = self.gripper_poses()
        pose = self.stick_view.get_transforms().clone()
        for i, arm in enumerate(F.ARMS):
            pos, Rs = self.held_pose(arm, gp)
            q = matrix_to_quat_xyzw(Rs)
            pose[i] = torch.tensor((pos[0], pos[1], pos[2] + F.TOP, *q), device=pose.device, dtype=pose.dtype)
        self.stick_view.set_transforms(pose, self.stick_ids)
        self.stick_view.set_velocities(torch.zeros_like(self.stick_view.get_velocities()), self.stick_ids)

    def _post_physics(self):
        if self.hold or not self.trial_on:
            return
        dt = F.PHYSICS_DT
        self.k += 1
        t = self.k * dt
        tr = array(self.stick_view.get_transforms())
        ve = array(self.stick_view.get_velocities())
        forces = np.linalg.norm(array(self.stick_contacts.get_contact_force_matrix(dt=dt)), axis=-1)   # (2, zones)
        grip = np.linalg.norm(array(self.finger_contacts.get_contact_force_matrix(dt=dt)), axis=-1)     # (2, 2)
        hand = np.linalg.norm(array(self.hand_contacts.get_contact_force_matrix(dt=dt)), axis=-1)       # (10, zones)
        cross = np.linalg.norm(array(self.cross_contacts.get_contact_force_matrix(dt=dt)), axis=-1)
        gp = self.gripper_poses()
        impulses, approach, heights, held = {}, {}, {}, {}
        for i, arm in enumerate(F.ARMS):
            stick = F.STICKS[arm]
            R = quat_xyzw_to_matrix(tr[i, 3:7])
            p = tr[i, :3] - (0., 0., F.TOP)
            tip = p + R[:, 0] * (L.TIP_OUT - L.TIP_R)
            com = p + R[:, 0] * (L.BALANCE_FROM_BUTT - L.GRIP_FROM_BUTT)
            v_tip = ve[i, :3] + np.cross(ve[i, 3:6], tip - com)
            hp, hR = self.held_pose(arm, gp)
            a_g = gp[arm][1].T @ R[:, 0]                       # the stick's axis in the gripper frame
            in_plane = math.degrees(math.atan2(a_g[2], a_g[1]))   # turned in the finger pads' plane
            out_plane = math.degrees(math.asin(max(-1., min(1., float(a_g[0])))))
            axial = float((p - hp) @ hR[:, 0])
            turn = math.degrees(math.acos(max(-1., min(1., float(R[:, 0] @ hR[:, 0])))))
            held[stick] = bool(grip[i].min() >= L.HELD_FORCE and abs(axial) < L.HELD_OFFSET and turn < L.HELD_TURN_DEG)
            self.sticks_now[arm] = {"tip": tip, "in_plane": in_plane, "out_plane": out_plane, "axial": axial,
                                    "grip": grip[i].copy(), "held": held[stick], "p": p, "axis": R[:, 0]}
            touching = False
            on_pad = set()
            for j, z in enumerate(F.SURFACES):
                c, n = self.zone_frames[z]
                approach[(stick, z)] = float(-(v_tip @ n))
                heights[(stick, z)] = float((tip - c) @ n - L.TIP_R) if not z.endswith("rim") else None
                if forces[i, j] > 0.:
                    impulses[(stick, z)] = float(forces[i, j]) * dt
                    touching = True
                    on_pad.add(F.ZONE_PAD[z])
            # resting: on a pad (touching it, or its tip within 2 mm of it, so a stick chattering on the head counts)
            # without a break for 0.5 s or more
            for pad in F.PADS:
                if pad not in on_pad and F.tip_clearance(tip, pad) >= L.REARM_GAP:
                    continue
                key = (stick, pad)
                run = self.rest_run.setdefault(key, [self.k, self.k])
                if run[1] < self.k - 1:                       # a new stay on the pad
                    run[0] = self.k
                run[1] = self.k
                lasted = (run[1] - run[0] + 1) * dt
                mark = (key, run[0])
                if lasted >= L.REST_S - 1e-9:
                    if mark not in self.rest_marked:
                        self.rest_marked[mark] = {"t": round(run[0] * dt, 4), "stick": arm, "pad": pad}
                        self.events["rests"].append(self.rest_marked[mark])
                    self.rest_marked[mark]["duration_s"] = round(lasted, 4)
            if touching:
                self.last_touch[stick] = self.k
            if not held[stick] and self.held_before.get(stick, True):
                self.events["dropped"].append({"t": round(t, 4), "stick": arm})
            self.held_before[stick] = held[stick]
        new = self.trigger.step(self.k, impulses, approach, heights, held=held)
        for h in new:
            self.note_ons.append((h["t"], L.NOTE_OF_PAD[h["note"]], h["velocity"]))
            self.events["hits"].append({"t": h["t"], "note": L.NOTE_OF_PAD[h["note"]], "stick": h["stick"].split("_")[1],
                                        "zone": h["zone"], "speed": h["speed"]})
            self.last_hit_t = h["t"]
            self.clear_since = None
            self.clean_from = None
        for a, name in enumerate(self.hand_names):              # gripper or arm strikes, one per contact over 2 N
            for z, zone in enumerate(F.SURFACES):
                key = (name, zone)
                if hand[a, z] > L.ARM_CONTACT_N:
                    if key not in self.arm_touching:
                        self.events["arm_contacts"].append({"t": round(t, 4), "part": name, "pad": F.ZONE_PAD[zone],
                                                            "force_n": round(float(hand[a, z]), 2)})
                    self.arm_touching[key] = self.k
                elif key in self.arm_touching and self.k - self.arm_touching[key] > 12:
                    del self.arm_touching[key]
                self.hand_peak = max(self.hand_peak, float(hand[a, z]))
        if cross.max() > 0:
            self.cross_steps += 1
        segs = [(s["p"] - s["axis"] * L.GRIP_FROM_BUTT, s["p"] + s["axis"] * L.TIP_OUT) for s in
                (self.sticks_now["left"], self.sticks_now["right"])]
        self.min_stick_gap = min(self.min_stick_gap, segment_gap(*segs[0], *segs[1]) - 2 * L.STICK_R)
        for arm in F.ARMS:
            self.grip_peak[arm] = max(self.grip_peak[arm], float(self.sticks_now[arm]["grip"].max()))

    # -- episode ----------------------------------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        self._install_hook()
        torch = self._torch
        self.trial_on = False
        self.hold = True
        q = [*F.START_Q["left"], 1., *F.START_Q["right"], 1.]
        self._ready = torch.tensor([q], dtype=torch.float32, device=self._base.device)
        observation = super().reset(scene, seed=seed)    # READY, then 0.5 s to the start pose, the sticks held there
        closed = self._ready.clone()
        closed[0, 6] = 0.
        closed[0, 13] = 0.
        for _ in range(12):                              # close on the sticks, still holding them in place
            self._env.step(closed)
        self.hold = False
        for _ in range(10):                              # let go: from here on only the fingers' friction holds them
            self._env.step(closed)
        for _ in range(4):
            self._base.sim.render()
        observation = self._observation()
        self.settings = self.physics_settings()
        self.trigger = H.Trigger({z: (F.ZONE_PAD[z], F.ZONE_NOTE[z]) for z in F.SURFACES},
                                 [F.STICKS[a] for a in F.ARMS], F.PHYSICS_DT)
        self.note_ons, self.events = [], {"hits": [], "arm_contacts": [], "rests": [], "dropped": [], "end": {}}
        self.rest_run, self.rest_marked, self.arm_touching = {}, {}, {}
        self.last_touch, self.held_before, self.sticks_now = {}, {}, {}
        self.last_hit_t, self.clear_since, self.clean_from = None, None, None
        self.cross_steps, self.min_stick_gap, self.hand_peak = 0, math.inf, 0.
        self.grip_peak = {arm: 0. for arm in F.ARMS}
        self.k, self.tick = 0, 0
        self.final_grade, self.judged_at = None, None
        self.diagnostics = []
        self.trial_on = True
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        gp = self.gripper_poses()
        tr = array(self.stick_view.get_transforms())
        self.state_after_reset = {}
        for i, arm in enumerate(F.ARMS):
            hp, hR = self.held_pose(arm, gp)
            R = quat_xyzw_to_matrix(tr[i, 3:7])
            p = tr[i, :3] - (0., 0., F.TOP)
            tip = p + R[:, 0] * (L.TIP_OUT - L.TIP_R)
            self.state_after_reset[arm] = {
                "stick_vs_grip_deg": round(math.degrees(math.acos(max(-1., min(1., float(R[:, 0] @ hR[:, 0]))))), 3),
                "axial_mm": round(1000 * float((p - hp) @ hR[:, 0]), 3),
                "tip_mm": [round(1000 * float(v), 1) for v in tip],
                "tip_clearance_mm": {k: round(1000 * F.tip_clearance(tip, k), 1) for k in F.PADS}}
            if self.state_after_reset[arm]["stick_vs_grip_deg"] > 2. or abs(self.state_after_reset[arm]["axial_mm"]) > 3.:
                raise RuntimeError(f"The {arm} stick did not settle in its grip: {self.state_after_reset[arm]}")
        masses = array(self.stick_view.get_masses()).reshape(-1)
        self.state_after_reset["stick_masses_kg"] = [round(float(m), 4) for m in masses]
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"k": self.k, "last_touch": dict(self.last_touch)}

    def end_ok(self):
        """Both sticks held and both tips at least 50 mm from every pad."""
        return all(self.sticks_now[arm]["held"] and min(F.tip_clearance(self.sticks_now[arm]["tip"], k)
                                                         for k in F.PADS) >= L.END_CLEAR for arm in F.ARMS)

    def grade(self):
        """The score of the hits so far (``judge_midi``), with this trial's events."""
        from .judge_midi import judge

        events = dict(self.events, end={"clean_from": self.clean_from})
        return judge(self.note_ons, events)

    def step(self, action):
        observation = super().step(action).observation
        self.tick += 1
        t = self.k * F.PHYSICS_DT
        if self.end_ok() and self.last_hit_t is not None:
            if self.clear_since is None:
                self.clear_since = t
            if self.clean_from is None and t - self.clear_since >= L.END_HOLD_S - 1e-9:
                self.clean_from = self.clear_since
        else:
            self.clear_since = None
        light = {"score": None, "hits": len(self.note_ons), "arm_contacts": len(self.events["arm_contacts"]),
                 "rests": len(self.events["rests"]), "clean_from": self.clean_from}
        last = self.tick >= self.step_limit
        result = light
        if (self.clean_from is not None and self.judged_at != self.clean_from) or last:
            self.judged_at = self.clean_from
            result = self.grade()
            self.final_grade = result
        over = result.get("score") == 1. and self.clean_from is not None
        if over or last:
            self.events["end"] = {"clean_from": self.clean_from}
        self._publish_demo_state()
        if self.tick % 2 == 0 or over or last:
            self.diagnostics.append(self._diagnose())
        grade_info = grading.summary(result) if result is not light else light
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade_info, terminal=over or last)
        return StepResult(observation=observation, terminated=over, termination_reason="success" if over else None,
                          info={"drum_grade": grade_info})

    def _diagnose(self):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        row = {"tick": self.tick, "hits": len(self.note_ons)}
        for arm in F.ARMS:
            s = self.sticks_now.get(arm)
            if s is None:
                continue
            row[arm] = {"in_plane_deg": round(s["in_plane"], 3), "out_plane_deg": round(s["out_plane"], 3),
                        "axial_mm": round(1000 * s["axial"], 2), "grip_n": [round(float(v), 1) for v in s["grip"]],
                        "held": s["held"], "tip_mm": [round(1000 * float(v), 1) for v in s["tip"]]}
        return row

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


def make_embodiment(**kwargs):
    return DrumArena(**kwargs)


from inspect_robots.policy import PolicyBase  # noqa: E402


class DrumScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import DrumReference

        self.reference = DrumReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration reads the robot's joint readings and the trigger's record of each stick's last contact
        # with a pad (simulator state, which the review rules allow for a scripted demonstration).
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = DrumScripted()
    policy.info = replace(YamScriptedPolicy().info, name="drum_groove_scripted")
    return policy
