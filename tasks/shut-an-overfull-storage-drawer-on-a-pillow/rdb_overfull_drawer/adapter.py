"""Add the storage drawer and the pillow to the stock Inspect Robots YAM station."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import grading
from . import layout
from .scripted import fingertips

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

BODIES = fixture.ACTORS
PAD_ROWS = [[layout.pad_name(row, col) for col in range(layout.COLS)] for row in range(layout.ROWS)]
CURRENT = None                                       # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class DrawerArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_overfull_drawer",
                            environment_id="isaaclab-arena-0.3.1-yam-overfull-drawer",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a plastic storage drawer, pulled part of "
                                 "the way out, and a pillow in it that stands above the drawer's opening. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and "
                                 "jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.skin = None
        self.skin_error = None
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
            objects.append(Object(name="drawer_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_overfull_drawer", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one drawer fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in BODIES}
        self.pad_rows = np.array([[self.rows[name] for name in row] for row in PAD_ROWS])
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        self.link_names = list(links)
        targets = [f"{actors}/{name}" for name in BODIES]
        # Rows: every link of the left arm, then of the right arm. Columns: the bodies in BODIES order.
        self.fixture_contacts = self.physics.create_rigid_contact_view(all_links, [targets] * len(all_links))
        # Display only: the pillow drawn as one fabric surface over its pads (the pads' boxes are hidden).
        if self.skin is None:
            try:
                from .skin import PillowSkin

                self.skin = PillowSkin(stage, actors.rsplit("/", 1)[0], fixture.TOP)
            except Exception as error:      # the drawing is not part of the physics or the score
                self.skin_error = repr(error)

    def _draw_pillow(self, base_position=None, base_rotation=None, pad_tops=None):
        if self.skin is None:
            return
        try:
            if base_position is None:
                base = self.poses[self.rows["pillow_base"]]
                base_position, base_rotation, pad_tops = base[:3], quat_xyzw_to_matrix(base[3:]), self.pad_tops()
            self.skin.update(base_position, base_rotation, pad_tops)
        except Exception as error:
            if self.skin_error is None:
                self.skin_error = repr(error)

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):
            raise RuntimeError("Nonfinite drawer or pillow state")

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def drawer_open(self):
        return layout.FRAME_FRONT_X - float(self.pos("drawer")[0])

    def pad_tops(self):
        """Height above the table of the top of each pad, rows from the drawer front, columns left and right."""
        return self.poses[self.pad_rows, 2].copy()

    def pad_leads(self):
        """Tabletop x of each row's leading (far) edge."""
        return self.poses[self.pad_rows, 0].max(axis=1) + layout.PAD_SIZE[0] / 2

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        # Every body back where the scene file has it: the drawer pulled out, the pillow in it, pads at rest
        # (those under the frame's top pressed down to just below it). Nothing depends on a previous trial.
        poses = self.body_view.get_transforms().clone()
        for name, (x, y, z) in fixture.WORLD_START.items():
            poses[self.rows[name]] = torch.tensor((x, y, z, 0., 0., 0., 1.), device=poses.device, dtype=poses.dtype)
        ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        # Draw the pillow at its start pose before the first frame is rendered.
        start_tops = [[fixture.START[name][2] for name in row] for row in PAD_ROWS]
        self._draw_pillow(np.asarray(fixture.START["pillow_base"]), np.eye(3), start_tops)
        observation = super().reset(scene, seed=seed)
        self._read()
        self._draw_pillow()
        if abs(self.drawer_open() - layout.OPEN_START) > .004:
            raise RuntimeError(f"Drawer is not at its start opening after reset: {self.drawer_open()}")
        if not np.allclose(self.pos("pillow_base"), fixture.START["pillow_base"], atol=.006, rtol=0):
            raise RuntimeError(f"Pillow is off its start place after reset: {self.pos('pillow_base')}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        if not np.allclose([masses[self.rows[name]] for name in BODIES], layout.MASSES, rtol=.01, atol=0):
            raise RuntimeError("A body's mass differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = grading.DrawerGrade()
        self.tick = 0
        # Diagnostics only; none of these enters the score.
        self.peak_interarm_n = 0.
        self.peak_arm_on_pillow_n = [0., 0.]
        self.peak_arm_on_drawer_n = [0., 0.]
        self.peak_pad_above_opening_under_frame_mm = 0.
        self.pad_tops_after_reset_mm = (1000. * self.pad_tops()).round(1).tolist()
        self.final_grade = None
        self.diagnostics = []
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"open": self.drawer_open(), "pad_top": self.pad_tops(), "pad_lead": self.pad_leads(),
                           "base": self.pos("pillow_base")}

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        self._draw_pillow()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        # Contact force of every arm link on each body, columns in BODIES order (last physics step).
        forces = np.linalg.norm(array(self.fixture_contacts.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        forces = forces.reshape(-1, len(BODIES))
        count = len(forces) // 2
        on_drawer = [float(forces[:count, 0].sum()), float(forces[count:, 0].sum())]
        on_pillow = [float(forces[:count, 1:].sum()), float(forces[count:, 1:].sum())]
        touching = bool(forces.max() > grading.TOUCH_N)
        tcp = np.array([ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        base = self.poses[self.rows["pillow_base"]]
        tops = self.pad_tops()
        open_m = self.drawer_open()
        grade = self.grading.update(open_m, base[:3], quat_xyzw_to_matrix(base[3:])[:, 2], tops.ravel(), tcp, touching)
        for i in range(2):
            self.peak_arm_on_pillow_n[i] = max(self.peak_arm_on_pillow_n[i], on_pillow[i])
            self.peak_arm_on_drawer_n[i] = max(self.peak_arm_on_drawer_n[i], on_drawer[i])
        leads = self.pad_leads()
        under = leads > layout.RAIL_X + .002
        if under.any():       # a pad under the frame's top cannot be above its underside unless contact gave way
            self.peak_pad_above_opening_under_frame_mm = max(
                self.peak_pad_above_opening_under_frame_mm, 1000. * float(tops[under].max() - layout.RAIL_Z))
        self.final_grade = grade
        self.tick += 1
        self._publish_demo_state()
        if self.tick % 5 == 0 or grade["score"] == 1.:
            self.diagnostics.append(self._diagnose(joints, tops, leads, on_drawer, on_pillow, grade))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"drawer_grade": grade})

    def _diagnose(self, joints, tops, leads, on_drawer, on_pillow, grade):
        """One row for the run log, in millimetres and newtons. Written to RESULT.json only; no policy receives it."""
        base = self.pos("pillow_base")
        centre = layout.drawer_x(self.drawer_open()) + layout.FRONT_T + layout.DRAWER_INNER[0] / 2
        return {
            "tick": self.tick, "open_mm": grade["open_mm"],
            "drawer_closing_speed_mm_s": round(1000. * float(self.velocities[self.rows["drawer"], 0]), 1),
            # per row from the drawer front: the higher of its two pads, relative to the underside of the frame's top
            "row_above_opening_mm": (1000. * (tops.max(axis=1) - layout.RAIL_Z)).round(1).tolist(),
            "row_left_minus_right_mm": (1000. * (tops[:, 0] - tops[:, 1])).round(1).tolist(),
            # per row: how far its leading edge is past the frame's front edge (negative: still in front of it)
            "row_lead_past_frame_mm": (1000. * (leads - layout.RAIL_X)).round(1).tolist(),
            "pillow_shift_in_drawer_mm": [round(1000. * float(base[0] - centre), 1), round(1000. * float(base[1]), 1),
                                          round(1000. * float(base[2] - layout.FLOOR_Z), 1)],
            "fingertips_mm": [[round(1000. * float(v), 1) for v in fingertips(joints, arm)] for arm in fixture.ARMS],
            "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],
            "arm_on_pillow_n": [round(v, 2) for v in on_pillow],
            "arm_on_drawer_n": [round(v, 2) for v in on_drawer],
            "gripper_clearance_mm": grade["gripper_clearance_mm"], "hands_clear": grade["hands_clear"],
            "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class DrawerScripted(PolicyBase):
    def __init__(self, variant="full"):
        self.variant = variant

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import DrawerReference

        self.reference = DrawerReference(self.variant)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the drawer's opening
        # and the height of the pillow's pads), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return DrawerArena(**kwargs)


def make_scripted(variant=None):
    from yam_arena.scripted import YamScriptedPolicy

    policy = DrawerScripted(variant or os.environ.get("OVERFULL_DRAWER_DEMO", "full"))
    policy.info = replace(YamScriptedPolicy().info, name="overfull_drawer_scripted")
    return policy
