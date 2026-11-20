"""Add the ukulele, on its side in two foam V-blocks with a clip-on tuner on its headstock, to the stock Inspect
Robots YAM station; run the pitch, pluck and tuner rules every control step and feed the scorer.

The fixed-base articulation is read through one articulation view (``create_articulation_view`` on the
articulation's Xform); the buttons' revolute joints have no end stops, so their readings are unwrapped (``_read``);
the tuner's lamps are drawn prims shown and hidden as the display changes (``skin.py``).
"""

from dataclasses import replace
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as fixture
from . import layout as L
from . import tuning
from .grading import TuningGrade

CURRENT = None                                       # the live embodiment, for the scripted demonstration
TOUCH_N = .05                                        # an arm touches a part above this contact force
BASES = {arm: np.asarray(fixture.BASES[arm], dtype=float) for arm in fixture.ARMS}
MOVING = (*fixture.BUTTON_LINKS.values(), fixture.PLUCK_LINK, fixture.NEEDLE_LINK)


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def needle_joint(displayed_deg):
    """The needle joint's angle (radians) that shows ``displayed_deg`` (positive: sharp, to the right)."""
    return float(np.radians(L.NEEDLE_SIGN * displayed_deg))


class UkuleleArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None, headless=True):
        global CURRENT
        # headless=False shows the Isaac Sim window on DISPLAY or the runtime's browser desktop
        # (yam_arena/inspect_embodiment.py, -E headless=false); it does not change the trial.
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176, headless=headless)
        self.info = replace(self.info, name="yam_ukulele_tuning",
                            environment_id="isaaclab-arena-0.3.1-yam-ukulele-tuning",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a soprano ukulele standing on its side in playing "
                                 "position, soundboard toward the robot, held in two foam V-blocks, and a clip-on "
                                 "tuner on its headstock. Observations are overhead RGB, both "
                                 "wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.skin = None
        self.skin_error = None
        self.tick = 0
        self.final_grade = None
        self.diagnostics = []
        self.pluck_events = []
        self.state_after_reset = None
        self.needle_target = needle_joint(L.NEEDLE_REST_DEG)
        CURRENT = self

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
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
            objects.append(Object(name="ukulele_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_ukulele_tuning", scene=Scene(objects),
                                           embodiment=embodiment, env_cfg_callback=scenes.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=device)).make_registered()

        with review_camera(headless=self.headless):
            from yam_arena import scene as scenes

            stock = scenes.make_env
            scenes.make_env = make_env
            try:
                super()._ensure_sim(None)
            finally:
                scenes.make_env = stock

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()
        found = {name: [str(p.GetPath()) for p in stage.Traverse()
                        if p.GetName() == name and str(p.GetPath()).startswith(root + "/")]
                 for name in (fixture.INSTRUMENT, fixture.LAMP_SCOPE)}
        if len(found[fixture.INSTRUMENT]) != 1 or len(found[fixture.LAMP_SCOPE]) != 1:
            raise RuntimeError(f"Expected one ukulele fixture, found {found}")
        ukulele = found[fixture.INSTRUMENT][0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        # One fixed-base articulation: the instrument, welded to the world, with the four buttons, the G string's
        # pluckable stretch and the tuner's needle.
        self.ukulele = self.physics.create_articulation_view(ukulele)
        names = list(self.ukulele.shared_metatype.dof_names)
        self.dofs = {name: names.index(name) for name in fixture.JOINTS}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.link_count = len(links)
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * self.link_count)
        # One contact view per arm: each arm link against every ukulele body (its own column); dt passed on read.
        targets = [f"{ukulele}/{name}" for name in fixture.LINKS]
        self.arm_contacts = {arm: self.physics.create_rigid_contact_view(arm_links[arm], [targets] * self.link_count)
                             for arm in fixture.ARMS}
        if self.skin is None:
            from .skin import LampSkin
            self.skin = LampSkin(stage, found[fixture.LAMP_SCOPE][0])

    # -- state ----------------------------------------------------------------------------------------------------
    def _read(self):
        raw = array(self.ukulele.get_dof_positions()).reshape(-1)
        if not np.isfinite(raw).all():
            raise RuntimeError("Nonfinite ukulele state")
        # The buttons have no end stops: continue their readings past half a turn.
        self.buttons = {name: tuning.unwrap(self.buttons[name], float(raw[self.dofs[joint]]))
                        for name, joint in fixture.BUTTON_JOINTS.items()}
        self.pluck = float(raw[self.dofs[fixture.PLUCK_JOINT]])
        self.needle = float(raw[self.dofs[fixture.NEEDLE_JOINT]])

    def _targets(self):
        """Drive targets, every step: the needle's from the display; the G string's
        stretch springs back to 0; the buttons' drives hold no position (stiffness 0), so their targets do nothing."""
        torch = self._torch
        one = torch.arange(1, dtype=torch.int32, device=self._base.device)
        targets = torch.zeros_like(self.ukulele.get_dof_positions())
        targets[0, self.dofs[fixture.NEEDLE_JOINT]] = self.needle_target
        self.ukulele.set_dof_position_targets(targets, one)

    def arm_forces(self):
        """Contact force (N) of each arm on each ukulele body: {arm: {link: N}}."""
        out = {}
        for arm, view in self.arm_contacts.items():
            matrix = np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum(axis=0)
            out[arm] = {name: float(matrix[i]) for i, name in enumerate(fixture.LINKS)}
        return out

    def _show(self):
        try:
            self.skin.update(self.shown["lamps"])
        except Exception as error:   # the lamps are display only; never let them stop a trial
            self.skin_error = repr(error)

    # -- episode --------------------------------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        one = torch.arange(1, dtype=torch.int32, device=self._base.device)
        self.needle_target = needle_joint(L.NEEDLE_REST_DEG)
        positions = torch.zeros_like(self.ukulele.get_dof_positions())
        positions[0, self.dofs[fixture.NEEDLE_JOINT]] = self.needle_target
        self.ukulele.set_dof_positions(positions, one)
        self.ukulele.set_dof_velocities(torch.zeros_like(positions), one)
        self._targets()
        observation = super().reset(scene, seed=seed)
        self.buttons = dict.fromkeys(L.STRINGS, 0.)
        self._read()
        self.state_after_reset = {"buttons_deg": {k: round(float(np.degrees(v)), 3) for k, v in self.buttons.items()},
                                  "pluck_mm": round(1000 * self.pluck, 3),
                                  "needle_deg": round(float(np.degrees(self.needle)), 2)}
        if max(abs(v) for v in self.buttons.values()) > np.radians(1.):
            raise RuntimeError(f"Buttons are not at rest after reset: {self.state_after_reset}")
        if abs(self.pluck) > .0003:
            raise RuntimeError(f"The G string's stretch is not at rest after reset: {self.state_after_reset}")
        if abs(self.needle - self.needle_target) > np.radians(3.):
            raise RuntimeError(f"The tuner's needle is not at rest after reset: {self.state_after_reset}")
        self._check_masses()
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.pitches = {name: tuning.Pitch(name, angle=self.buttons[name]) for name in L.STRINGS}
        self.detector = tuning.PluckDetector(fixture.CONTROL_HZ)
        self.display = tuning.TunerDisplay(fixture.CONTROL_HZ)
        self.shown = tuning.blank()
        self.grading = TuningGrade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak = {"arm_on_button_g_n": 0., "arm_on_button_c_n": 0., "arm_on_buttons_e_a_n": 0., "arm_on_pluck_n": 0.,
                     "arm_on_instrument_n": 0., "pluck_mm": 0., "needle_error_deg": 0.}
        self.final_grade = None
        self.diagnostics = []
        self.pluck_events = []
        self._show()
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _check_masses(self):
        """The moving bodies' masses as declared, read through the articulation view (get_masses,
        shared_metatype.link_names, as Isaac Lab's articulation data does). If that API is missing the check is
        recorded as skipped, not failed."""
        try:
            masses = array(self.ukulele.get_masses()).reshape(-1)
            names = list(self.ukulele.shared_metatype.link_names)
            found = {name: float(masses[names.index(name)]) for name in MOVING}
        except Exception as error:
            self.state_after_reset["mass_check"] = f"skipped: {error!r}"
            return
        declared = dict(zip(fixture.LINKS, fixture.LINK_MASSES))
        for name, mass in found.items():
            if not np.isclose(mass, declared[name], rtol=.02, atol=1e-6):
                raise RuntimeError(f"{name} mass {mass} differs from the declared {declared[name]}")
        self.state_after_reset["mass_check"] = "passed"

    def _publish_demo_state(self, cents=None):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        cents = cents if cents is not None else {name: self.pitches[name].cents for name in L.STRINGS}
        self.demo_state = {"buttons": dict(self.buttons), "cents": dict(cents), "play_deg": self.pitches[L.TUNED].play,
                           "pluck": self.pluck, "plucks": self.detector.count, "tuner": dict(self.shown)}

    def tool_points(self, joints):
        return [ik.tcp_pose(joints[i:i + 6], BASES[arm]) for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]

    def step(self, action):
        self._targets()
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = self.arm_forces()
        touched = any(forces[arm][fixture.PLUCK_LINK] > TOUCH_N for arm in fixture.ARMS)
        cents = {name: self.pitches[name].update(self.buttons[name]) for name in L.STRINGS}
        self.tick += 1
        plucked = self.detector.update(self.tick, self.pluck, touched)
        if plucked:
            self.pluck_events.append({"tick": self.tick, "peak_mm": round(1000 * self.detector.last_peak, 2),
                                      "g_cents": round(cents[L.TUNED], 2)})
        self.shown = self.display.update(self.tick, plucked, cents[L.TUNED])
        self.needle_target = needle_joint(self.shown["needle_deg"])
        self._show()
        tools = self.tool_points(joints)
        # the final check needs this step's pluck and the G button's angle (tightening degrees from rest)
        grade = self.grading.update(cents, [fixture.to_instrument(point) for point, _ in tools], plucked,
                                    self.pitches[L.TUNED].button)
        grade = {**grade, "plucks": self.detector.count, "lamps_lit": [k for k, on in self.shown["lamps"].items() if on],
                 "tuner": {k: self.shown[k] for k in ("active", "note", "cents_from_note", "needle_deg")}}
        self.final_grade = grade
        self._publish_demo_state(cents)
        self._peaks(forces)
        if self.tick % 5 == 0 or plucked:
            self.diagnostics.append(self._diagnose(joints, tools, forces, cents, grade))
        over = self.grading.finished
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=over, termination_reason="success" if over else None,
                          info={"tuning_grade": grade})

    def _peaks(self, forces):
        both = {name: forces["left"][name] + forces["right"][name] for name in fixture.LINKS}
        for key, value in (("arm_on_button_g_n", both["button_g"]), ("arm_on_button_c_n", both["button_c"]),
                           ("arm_on_buttons_e_a_n", both["button_e"] + both["button_a"]),
                           ("arm_on_pluck_n", both[fixture.PLUCK_LINK]),
                           ("arm_on_instrument_n", both[fixture.ROOT_LINK]), ("pluck_mm", 1000 * abs(self.pluck)),
                           ("needle_error_deg", float(np.degrees(abs(self.needle - self.needle_target))))):
            self.peak[key] = max(self.peak[key], float(value))

    def _diagnose(self, joints, tools, forces, cents, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        return {
            "tick": self.tick,
            "tool_mm": [[round(1000 * float(v), 1) for v in fixture.to_instrument(point)] for point, _ in tools],   # instrument frame
            "gripper_opening": [round(float(joints[6]), 4), round(float(joints[13]), 4)],
            "joint6_deg": [round(float(np.degrees(joints[5])), 1), round(float(np.degrees(joints[12])), 1)],
            "buttons_deg": {k: round(float(np.degrees(v)), 2) for k, v in self.buttons.items()},
            "cents": {k: round(v, 2) for k, v in cents.items()},
            "play_deg": round(self.pitches[L.TUNED].play, 2),
            "pluck_mm": round(1000 * self.pluck, 2), "plucks": self.detector.count,
            "tuner": {k: self.shown[k] for k in ("active", "note", "cents_from_note", "needle_deg")},
            "needle_deg_measured": round(float(np.degrees(L.NEEDLE_SIGN * self.needle)), 1),
            "contact_n": {arm: {k: round(v, 2) for k, v in forces[arm].items() if v > .01} for arm in fixture.ARMS},
            "clearance_mm": grade["clearance_mm"], "final_check": grade["final_check"], "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class UkuleleScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import TuningReference

        self.reference = TuningReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (button angles, computed
        # pitches, the free play's state, the string's displacement, the pluck count, the tuner's display). Only the
        # scripted demonstration gets this; no policy observation contains it.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return UkuleleArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = UkuleleScripted()
    policy.info = replace(YamScriptedPolicy().info, name="ukulele_tuning_scripted")
    return policy
