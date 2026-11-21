"""Add the bottle, its cap, the tablets and the medicine cup to the stock Inspect Robots YAM station."""

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
from .grading import HELD_N, SUPPORT_N, TabletGrade, in_bottle

BODIES = fixture.BODIES
N = layout.N_TABLETS
TOUCH_N = .005                                       # a gripper link pressing on a tablet with at least this force
FINGERTIPS = ("tip_left", "tip_right")              # the gripper's two finger links
CURRENT = None                                       # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class TabletArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_tablets_cap",
                            environment_id="isaaclab-arena-0.3.1-yam-tablets-cap",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a safety-capped bottle of tablets, its cap and a "
                                 "medicine cup. Observations are overhead RGB, both wrist RGB views, and robot joint "
                                 "and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
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
            objects.append(Object(name="tablets_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_tablets_cap", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        self.collar_view = self.physics.create_rigid_body_view(actors.rsplit("/", 1)[0] + "/Closure/spring_collar")
        paths = list(self.body_view.prim_paths)
        self.rows = np.array([paths.index(f"{actors}/{name}") for name in BODIES])
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        self.link_names = list(links)
        targets = [f"{actors}/{name}" for name in BODIES]
        # Contact force of every arm link on each of the scene's bodies, columns in BODIES order.
        self.arm_contacts = self.physics.create_rigid_contact_view(all_links, [targets] * len(all_links),
                                                                   max_contact_data_count=8192)
        self.all_ids = self._torch.arange(len(paths), dtype=self._torch.int32, device=self._base.device)
        # Which rows of arm_contacts are the two fingertips of each gripper (for "the cap is held").
        count = len(self.link_names)
        missing = [name for name in FINGERTIPS if name not in self.link_names]
        if missing:
            raise RuntimeError(f"Fingertip links {missing} not among the arm links {self.link_names}")
        self.tip_rows = {arm: [i * count + self.link_names.index(name) for name in FINGERTIPS]
                         for i, arm in enumerate(fixture.ARMS)}
        # Whether the bottle or the cup pushes on the cap (the cap resting on one of them). Its own view, so that
        # no other pair's force can be credited to it.
        self.cap_support = self.physics.create_rigid_contact_view(
            [f"{actors}/cap"], [[f"{actors}/bottle", f"{actors}/cup"]], max_contact_data_count=1024)

    def _read(self):
        poses = array(self.body_view.get_transforms())[self.rows]
        poses[:, 2] -= fixture.TOP
        if not np.isfinite(poses).all():
            raise RuntimeError("Nonfinite scene state")
        self.poses = poses
        self.velocities = array(self.body_view.get_velocities())[self.rows]

    def pose(self, index):
        return self.poses[index, :3].copy(), quat_xyzw_to_matrix(self.poses[index, 3:])

    def grippers(self, joints):
        return [ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        # Everything back where it starts: bottle, cap and cup on their marks, the tablets in the bottle.
        poses = self.body_view.get_transforms().clone()
        for row, name in zip(self.rows, BODIES, strict=True):
            x, y, z = fixture.START[name]
            orientation = (1., 0., 0., 0.) if name == "cap" else (0., 0., 0., 1.)
            poses[row] = torch.tensor((x, y, z + fixture.TOP + .0003, *orientation), device=poses.device, dtype=poses.dtype)
        self.body_view.set_transforms(poses, self.all_ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.all_ids)
        collar = self.collar_view.get_transforms().clone()
        collar[0] = poses[self.rows[0]]
        collar[0, 2] += layout.COLLAR_Z
        self.collar_view.set_transforms(collar, self.all_ids[:1])
        self.collar_view.set_velocities(torch.zeros_like(self.collar_view.get_velocities()), self.all_ids[:1])
        observation = super().reset(scene, seed=seed)
        self._read()
        for index, name in enumerate(BODIES[:3]):
            if not np.allclose(self.poses[index, :3], fixture.START[name], atol=.004, rtol=0):
                raise RuntimeError(f"{name} is off its mark after reset: {self.poses[index, :3]}")
        masses = array(self.body_view.get_masses()).reshape(-1)[self.rows]
        if not np.allclose(masses, fixture.MASSES, rtol=.01, atol=0):
            raise RuntimeError("A body's mass differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        if self.pose(1)[1][2, 2] > -.99:
            raise RuntimeError("Cap did not reset closed over the neck")
        self.grading = TabletGrade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak_arm_on = dict.fromkeys(("bottle", "cap", "cup", "tablets"), 0.)
        self.final_grade = None
        self.joint_pos = np.asarray(observation.state["joint_pos"], dtype=float).copy()
        self.diagnostics = []
        self.start_report = {"tablets_in_bottle_at_start": int(np.sum(in_bottle(self.pose(0), self.poses[3:, :3]))),
            "safety_cap_initial_height_mm": round(1000 * float(self.poses[1, 2]-self.poses[0, 2]), 2),
            "tablet_height_range_mm": [round(1000 * float(self.poses[3:, 2].min()), 1), round(1000 * float(self.poses[3:, 2].max()), 1)]}
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"bottle": self.pose(0), "cap": self.pose(1), "cup": self.pose(2),
                           "tablets": self.poses[3:, :3].copy(), "opening_stage": self.grading.opening.stage}

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = np.linalg.norm(array(self.arm_contacts.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        touching = forces[:, 3:].max(axis=0) > TOUCH_N
        for i, name in enumerate(("bottle", "cap", "cup")):
            self.peak_arm_on[name] = max(self.peak_arm_on[name], float(forces[:, i].sum()))
        self.peak_arm_on["tablets"] = max(self.peak_arm_on["tablets"], float(forces[:, 3:].max()))
        self.joint_pos = joints.copy()
        cap_column = BODIES.index("cap")
        tips_on_cap = {arm: [float(forces[row, cap_column]) for row in rows] for arm, rows in self.tip_rows.items()}
        cap_held = any(min(values) >= HELD_N for values in tips_on_cap.values())
        bottle_held = {arm: min(forces[row, 0] for row in rows) >= HELD_N for arm, rows in self.tip_rows.items()}
        bimanual = any(min(tips_on_cap[arm]) >= HELD_N and bottle_held[other]
                       for arm, other in (("left", "right"), ("right", "left")))
        support = np.linalg.norm(array(self.cap_support.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).reshape(-1)
        self.tips_on_cap, self.cap_support_n = tips_on_cap, [float(v) for v in support]
        grade = self.grading.update(self.poses[3:, :3], self.pose(0), self.pose(1), self.pose(2), touching,
                                    self.grippers(joints), cap_held=cap_held,
                                    cap_supported=bool(support.max() >= SUPPORT_N), opening_bimanual=bimanual)
        self.final_grade = grade
        self.tick += 1
        self._publish_demo_state()
        if self.tick % 5 == 0:
            self.diagnostics.append(self._diagnose(joints, forces, grade))
        success = grade["score"] == 1.
        over = success or self.grading.finished
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        reason = "success" if success else grade["void"] if over else None
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"tablet_grade": grade})

    def _diagnose(self, joints, forces, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        count = len(forces) // 2
        tips = [[round(1000 * float(v), 1) for v in point] for point in self.grippers(joints)]
        bottle, bottle_r = self.pose(0)
        cap, cap_r = self.pose(1)
        cup, cup_r = self.pose(2)
        lowest = self.poses[3:, 2].min()
        return {
            "tick": self.tick,
            "gripper_centre_mm": tips,                                  # left, right; from the joint readings
            "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],
            "bottle_mm": [round(1000 * float(v), 1) for v in bottle],
            "bottle_tilt_deg": round(float(np.degrees(np.arccos(np.clip(bottle_r[2, 2], -1., 1.)))), 1),
            "cap_mm": [round(1000 * float(v), 1) for v in cap],
            "cap_tilt_deg": round(float(np.degrees(np.arccos(np.clip(cap_r[2, 2], -1., 1.)))), 1),
            "cup_mm": [round(1000 * float(v), 1) for v in cup],
            "cup_tilt_deg": round(float(np.degrees(np.arccos(np.clip(cup_r[2, 2], -1., 1.)))), 1),
            "arm_on_bottle_n": [round(float(forces[:count, 0].sum()), 2), round(float(forces[count:, 0].sum()), 2)],
            "arm_on_cap_n": [round(float(forces[:count, 1].sum()), 2), round(float(forces[count:, 1].sum()), 2)],
            "arm_on_cup_n": [round(float(forces[:count, 2].sum()), 2), round(float(forces[count:, 2].sum()), 2)],
            "arm_on_tablets_n": round(float(forces[:, 3:].max()), 3),
            "fingertips_on_cap_n": {arm: [round(v, 2) for v in values] for arm, values in self.tips_on_cap.items()},
            "bottle_cup_on_cap_n": [round(v, 3) for v in self.cap_support_n],
            "fastest_tablet_m_s": round(float(np.linalg.norm(self.velocities[3:, :3], axis=1).max()), 3),
            "lowest_tablet_mm": round(1000 * float(lowest), 1),
            **{key: grade[key] for key in ("in_bottle", "in_cap", "in_cup", "strays", "lay_in_cap", "max_touch_ticks",
                                           "released", "still", "outcome", "score", "cap_lowest_mm", "cap_held", "cap_supported",
                                           "opening_stage", "opening_bimanual", "cap_press_mm", "cap_turn_deg",
                                           "cap_lift_mm", "void")},
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class TabletScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import TabletReference

        self.reference = TabletReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (poses of the bottle,
        # the cap and the cup, positions of the tablets), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return TabletArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = TabletScripted()
    policy.info = replace(YamScriptedPolicy().info, name="tablets_cap_scripted")
    return policy
