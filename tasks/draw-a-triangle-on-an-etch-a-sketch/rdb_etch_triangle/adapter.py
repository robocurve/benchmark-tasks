"""Add the Etch A Sketch to the stock Inspect Robots YAM station, draw its line and feed the scorer."""

from dataclasses import replace
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import PHYSICS_DT
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as fixture
from . import layout
from .grading import TriangleGrade
from .stylus import Stylus, unwrap

CURRENT = None                                       # the live embodiment, for the scripted demonstration
TRACE_EVERY = 1                                      # contact trace: every control step
CONTACT_TRACE_COLUMNS = ("tick", "left_arm_on_left_knob_n", "right_arm_on_right_knob_n", "left_arm_on_frame_n",
                         "right_arm_on_frame_n", "left_knob_deg", "right_knob_deg", "stylus_x_mm", "stylus_y_mm")
PATH_STEP_M = .0002                                  # the drawn path keeps a point when the stylus has moved this far


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def yaw_quat_xyzw(yaw):
    return (0., 0., float(np.sin(yaw / 2)), float(np.cos(yaw / 2)))


class EtchArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_etch_triangle",
                            environment_id="isaaclab-arena-0.3.1-yam-etch-triangle",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with an Etch A Sketch lying screen up. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and jaw "
                                 "positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.skin = None
        self.tick = 0
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self.state_after_reset = None
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
            objects.append(Object(name="etch_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_etch_triangle", scene=Scene(objects),
                                           embodiment=embodiment, env_cfg_callback=scenes.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=device)).make_registered()

        with review_camera():
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
        actor_roots = [str(p.GetPath()) for p in stage.Traverse()
                       if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(actor_roots) != 1:
            raise RuntimeError(f"Expected one Etch A Sketch fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.LINKS}
        self.toy = self.physics.create_articulation_view(actors + "/frame")
        names = list(self.toy.shared_metatype.dof_names)
        self.dofs = {name: names.index(name) for name in fixture.JOINTS}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.link_count = len(links)
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * self.link_count)
        # One contact view per arm, each link against the three toy bodies (its own columns), dt passed on read.
        targets = [f"{actors}/{name}" for name in fixture.LINKS]
        self.arm_contacts = {arm: self.physics.create_rigid_contact_view(arm_links[arm], [targets] * self.link_count)
                             for arm in fixture.ARMS}
        if self.skin is None:
            from .skin import LineSkin
            self.skin = LineSkin(stage, actors.rsplit("/", 1)[0] + "/DrawnLine")

    def _read(self):
        poses = array(self.body_view.get_transforms())
        poses[:, 2] -= fixture.TOP
        wrapped = array(self.toy.get_dof_positions()).reshape(-1)
        if not (np.isfinite(poses).all() and np.isfinite(wrapped).all()):
            raise RuntimeError("Nonfinite Etch A Sketch state")
        self.poses = poses
        raw = np.array([wrapped[self.dofs[name]] for name in fixture.JOINTS])
        self.knob_angles = np.array([unwrap(a, b) for a, b in zip(self.knob_angles, raw, strict=True)])

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def rotation(self, name):
        return quat_xyzw_to_matrix(self.poses[self.rows[name], 3:])

    def knob_top(self, name):
        return self.pos(name) + self.rotation(name)[:, 2] * (layout.KNOB_TOP - layout.BODY_H)

    def arm_forces(self):
        """Contact force (N) of each arm on each toy body: {arm: {body: N}}."""
        out = {}
        for arm, view in self.arm_contacts.items():
            matrix = np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
            out[arm] = {name: float(matrix[:, i].sum()) for i, name in enumerate(fixture.LINKS)}
        return out

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        one = torch.arange(1, dtype=torch.int32, device=self._base.device)
        root = self.toy.get_root_transforms().clone()
        root[0] = torch.tensor((*fixture.TOY_XY, fixture.TOP, *yaw_quat_xyzw(fixture.TOY_YAW)),
                               device=root.device, dtype=root.dtype)
        self.toy.set_root_transforms(root, one)
        self.toy.set_root_velocities(torch.zeros_like(self.toy.get_root_velocities()), one)
        zero = torch.zeros_like(self.toy.get_dof_positions())
        self.toy.set_dof_positions(zero, one)
        self.toy.set_dof_velocities(zero, one)
        self.toy.set_dof_position_targets(zero, one)
        self.knob_angles = np.zeros(2)
        observation = super().reset(scene, seed=seed)
        self.knob_angles = np.zeros(2)
        self._read()
        frame = self.pos("frame")
        self.state_after_reset = {"frame_mm": [round(1000 * float(v), 2) for v in frame],
                                  "knob_deg": [round(float(np.degrees(a)), 3) for a in self.knob_angles],
                                  "knob_tops_mm": {n: [round(1000 * float(v), 2) for v in self.knob_top(n)]
                                                   for n in fixture.JOINTS}}
        if not np.allclose(frame, fixture.TOY_START, atol=.003, rtol=0):
            raise RuntimeError(f"Etch A Sketch is off its marks after reset: {frame}")
        if np.abs(self.knob_angles).max() > np.radians(1.):
            raise RuntimeError(f"Knobs are not at zero after reset: {np.degrees(self.knob_angles)}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        declared = {"frame": layout.MASS - 2 * layout.KNOB_MASS, "left_knob": layout.KNOB_MASS,
                    "right_knob": layout.KNOB_MASS}
        for name, mass in declared.items():
            if not np.isclose(masses[self.rows[name]], mass, rtol=.01, atol=0):
                raise RuntimeError(f"{name} mass {masses[self.rows[name]]} differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        # The stylus starts on the dot with both strings' slack centred; the dot is the first point of the line.
        self.stylus = Stylus()
        self.stylus.update(self.knob_angles)
        self.path = [np.asarray(layout.START, dtype=float)]
        self.grading = TriangleGrade(start_xy=tuple(frame[:2]), start_yaw=fixture.TOY_YAW)
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak_forces = {"left_arm_on_left_knob": 0., "right_arm_on_right_knob": 0., "arms_on_frame": 0.,
                            "left_arm_on_right_knob": 0., "right_arm_on_left_knob": 0.}
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self._draw()
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _draw(self):
        point = np.asarray(layout.START) + self.stylus.point
        if np.linalg.norm(point - self.path[-1]) >= PATH_STEP_M:
            self.path.append(point)
        try:
            self.skin.update(self.path, self.pos("frame"), self.rotation("frame"), fixture.TOP)
        except Exception as error:   # the drawing is display only; never let it stop a trial
            self.skin_error = repr(error)

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {
            "stylus": self.stylus.point.copy(), "knobs": self.knob_angles.copy(),
            "engaged": [(self.stylus.engaged(i, +1), self.stylus.engaged(i, -1)) for i in range(2)],
            "knob_tops": [self.knob_top(name) for name in fixture.JOINTS]}

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = self.arm_forces()
        touching = any(v > .05 for arm in forces.values() for v in arm.values())
        self.stylus.update(self.knob_angles)
        self._draw()
        grade = self.grading.update(self.stylus.point, self.pos("frame"), self.rotation("frame"), touching)
        self.final_grade = grade
        self.tick += 1
        self._publish_demo_state()
        own = (forces["left"]["left_knob"], forces["right"]["right_knob"])
        for key, value in (("left_arm_on_left_knob", own[0]), ("right_arm_on_right_knob", own[1]),
                           ("arms_on_frame", forces["left"]["frame"] + forces["right"]["frame"]),
                           ("left_arm_on_right_knob", forces["left"]["right_knob"]),
                           ("right_arm_on_left_knob", forces["right"]["left_knob"])):
            self.peak_forces[key] = max(self.peak_forces[key], value)
        if self.tick % TRACE_EVERY == 0:
            self.contact_trace.append([self.tick, round(own[0], 2), round(own[1], 2),
                                       round(forces["left"]["frame"], 2), round(forces["right"]["frame"], 2),
                                       *(round(float(np.degrees(a)), 2) for a in self.knob_angles),
                                       *(round(1000 * float(v), 2) for v in self.stylus.point)])
        if self.tick % 5 == 0:
            self.diagnostics.append(self._diagnose(joints, forces, grade))
        success = grade["score"] == 1.
        over = success or self.grading.finished
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        reason = "success" if success else grade["void"] if over else None
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"triangle_grade": grade})

    def _diagnose(self, joints, forces, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        tips, wrist = [], []
        for arm, i, knob in zip(fixture.ARMS, (0, 7), fixture.JOINTS, strict=True):
            point, _ = ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])
            tips.append([round(1000 * float(v), 1) for v in point - self.knob_top(knob)])
            wrist.append(round(float(np.degrees(joints[i + 5])), 1))
        return {
            "tick": self.tick,
            "knob_deg": [round(float(np.degrees(a)), 2) for a in self.knob_angles],
            "drum_deg": [round(float(np.degrees(a)), 2) for a in self.stylus.drums],
            "stylus_mm": [round(1000 * float(v), 2) for v in self.stylus.point],
            "tcp_from_knob_top_mm": tips,                      # left, right; from joint readings
            "joint6_deg": wrist,
            "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],
            "arm_on_own_knob_n": [round(forces["left"]["left_knob"], 2), round(forces["right"]["right_knob"], 2)],
            "arm_on_frame_n": [round(forces["left"]["frame"], 2), round(forces["right"]["frame"], 2)],
            "toy_offset_mm": grade["toy_offset_mm"], "toy_turned_deg": grade["toy_turned_deg"],
            "toy_highest_corner_mm": grade["toy_highest_corner_mm"],
            "covered_mm": grade["covered_mm"], "longest_step_mm": grade["longest_step_mm"], "stray": grade["stray"],
            "tier": grade["tier"], "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class EtchScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import EtchReference

        self.reference = EtchReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (stylus point, knob
        # angles and slack, knob positions), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return EtchArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = EtchScripted()
    policy.info = replace(YamScriptedPolicy().info, name="etch_triangle_scripted")
    return policy
