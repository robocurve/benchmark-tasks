"""Add the bag, the carton and the pack to the stock Inspect Robots YAM station, grade every step and log
diagnostics."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import bag_model as m
from . import fixed_scene as fixture
from . import grading

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

PHYSICS_DT = 1. / fixture.PHYSICS_HZ          # PhysX reports contact impulses of the last 240 Hz substep
CURRENT = None                                # the live embodiment, for the scripted demonstration
BODIES = (*m.LINKS, *m.ITEMS)
TIPS = ("tip_left", "tip_right")
FELT = ("carton", "pack", *m.HANDLES.values())            # bodies whose fingertip contact forces are logged
START_ITEMS = {"carton": (*m.CARTON_XY, m.CARTON[2] / 2), "pack": (*m.PACK_XY, m.PACK[2] / 2)}


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def plain(value):
    """JSON-safe copy: numpy scalars and arrays become Python numbers and lists."""
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def mm(values):
    return [round(1000 * float(v), 1) for v in values]


class BagArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_bag_load",
                            environment_id="isaaclab-arena-0.3.1-yam-bag-load",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with an empty limp tote bag collapsed on its base "
                                 "board, a box of crackers and a four-roll pack of toilet tissue. Observations are "
                                 "overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.tick = 0
        self.diagnostics = []
        self.final_grade = None
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
            objects.append(Object(name="bag_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_bag_load", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one bag fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in BODIES}
        device = self._base.device
        self.item_ids = self._torch.as_tensor([self.rows[name] for name in m.ITEMS], dtype=self._torch.int32, device=device)
        # The bag: one floating articulation rooted on its base board.
        self.bag = self.physics.create_articulation_view(actors + "/base")
        names = list(self.bag.shared_metatype.dof_names)
        self.dofs = {name: names.index(name) for name in m.JOINTS}
        links = asset_link_paths()
        self.arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                          for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            self.arm_links["left"], [self.arm_links["right"]] * len(self.arm_links["left"]))
        # Every arm link against every body of the fixture (rows: arm links; columns: BODIES), one view per arm.
        targets = [f"{actors}/{name}" for name in BODIES]
        self.arm_contacts = {arm: self.physics.create_rigid_contact_view(sensors, [targets] * len(sensors))
                             for arm, sensors in self.arm_links.items()}
        link_names = list(links)
        self.tip_rows = [link_names.index(name) for name in TIPS]

    # --- reading the simulator ------------------------------------------------------------------------
    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        self.q = array(self.bag.get_dof_positions()).reshape(-1)
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all() and np.isfinite(self.q).all()):
            raise RuntimeError("Nonfinite bag state")

    def pose(self, name):
        row = self.poses[self.rows[name]]
        return row[:3].copy(), quat_xyzw_to_matrix(row[3:])

    def bag_state(self):
        """Poses of the base board, wall panels, handles and items, joint angles and rim heights: tabletop frame."""
        uppers = {wall: self.pose(m.UPPER[wall]) for wall in m.WALLS}
        handles = {wall: self.pose(m.HANDLES[wall]) for wall in m.HANDLES}
        return {"base": self.pose("base"), "uppers": uppers, "handles": handles,
                "lowers": {wall: self.pose(m.LOWER[wall]) for wall in m.WALLS},
                "grips": {wall: m.grip_centre(*handles[wall]) for wall in m.HANDLES},
                "items": {name: self.pose(name) for name in m.ITEMS},
                "rims": {wall: m.rim_height(wall, *uppers[wall]) for wall in m.WALLS},
                "angles": {name: float(self.q[i]) for name, i in self.dofs.items()},
                "speed": float(np.abs(self.velocities[:, :3]).max())}

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        device = self._base.device
        one = torch.arange(1, dtype=torch.int32, device=device)
        # The bag on its marks, collapsed; its creases sprung toward their folded rest shape.
        root = self.bag.get_root_transforms().clone()
        root[0] = torch.tensor((*m.BAG_XY, fixture.TOP, 0., 0., 0., 1.), device=device, dtype=root.dtype)
        self.bag.set_root_transforms(root, one)
        self.bag.set_root_velocities(torch.zeros_like(self.bag.get_root_velocities()), one)
        positions = torch.zeros_like(self.bag.get_dof_positions())
        rest = torch.zeros_like(positions)
        for name, value in m.START.items():
            positions[0, self.dofs[name]] = float(value)
        for wall in m.WALLS:
            rest[0, self.dofs[m.LEAN[wall]]] = float(m.LEAN_REST[wall])
            rest[0, self.dofs[m.FOLD[wall]]] = float(m.FOLD_REST[wall])
        self.bag.set_dof_positions(positions, one)
        self.bag.set_dof_velocities(torch.zeros_like(positions), one)
        self.bag.set_dof_position_targets(rest, one)
        poses = self.body_view.get_transforms().clone()
        for name, (x, y, z) in START_ITEMS.items():
            poses[self.rows[name]] = torch.tensor((x, y, z + fixture.TOP + .0005, 0., 0., 0., 1.), device=device, dtype=poses.dtype)
        self.body_view.set_transforms(poses, self.item_ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.item_ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        state = self.bag_state()
        self.state_after_reset = self._state_row(state)
        # Fail loudly, with the numbers, if the scene did not come up as declared.
        off = max(abs(state["angles"][name] - value) for name, value in m.START.items())
        if off > np.radians(6.):
            raise RuntimeError(f"The bag is not at its start shape after reset: {self.state_after_reset}")
        for name, place in START_ITEMS.items():
            if np.linalg.norm(state["items"][name][0] - place) > .004:
                raise RuntimeError(f"The {name} is off its start place after reset: {self.state_after_reset}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        declared = {"base": m.BASE_MASS, "carton": m.CARTON_MASS, "pack": m.PACK_MASS,
                    **{m.HANDLES[w]: m.HANDLE_MASS for w in m.HANDLES},
                    **{names[w]: m.PANEL_MASS[w] for names in (m.LOWER, m.UPPER) for w in m.WALLS}}
        for name, mass in declared.items():
            if not np.isclose(masses[self.rows[name]], mass, rtol=.02, atol=1e-6):
                raise RuntimeError(f"{name} mass {masses[self.rows[name]]} differs from the declared {mass}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = grading.BagGrade()
        self.tick = 0
        self.peak_interarm_n = 0.       # diagnostics; none of these enters the score
        self.peaks = {}
        self.final_grade = None
        self.diagnostics = []
        self.demo_state = state
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _forces(self):
        """Contact force (N, last physics step) of each arm on the fixture: the whole arm on the bag and on each
        item, and each finger on the carton, the pack and the two handles."""
        out, touching = {}, False
        bag = [BODIES.index(name) for name in m.LINKS]
        for arm, view in self.arm_contacts.items():
            matrix = np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
            touching = touching or bool(matrix.max() > grading.TOUCH_N)
            out[f"{arm}_on_bag"] = float(matrix[:, bag].sum())
            for name in m.ITEMS:
                out[f"{arm}_on_{name}"] = float(matrix[:, BODIES.index(name)].sum())
            tips = matrix[self.tip_rows]
            for name in FELT:
                out[f"{arm}_tips_on_{name}"] = [round(float(v), 2) for v in tips[:, BODIES.index(name)]]
        return out, touching

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces, touching = self._forces()
        for key, value in forces.items():
            self.peaks[key] = max(self.peaks.get(key, 0.), float(np.max(value)))
        tcp = np.array([ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        state = self.bag_state()
        state["forces"] = forces
        grade = plain(self.grading.update(base=state["base"], uppers=state["uppers"], handles=state["handles"],
                                          items=state["items"], tcp=tcp, touching=touching))
        self.final_grade = grade
        self.tick += 1
        self.demo_state = state
        if self.tick % 5 == 0 or grade["score"] == 1.:
            self.diagnostics.append({"tick": self.tick, **self._state_row(state),
                                     "gripper_mm": [mm(p) for p in tcp],
                                     "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],
                                     "forces_n": {k: v for k, v in forces.items() if np.max(v) > 0.},
                                     "score": grade["score"], "upright": grade.get("upright"),
                                     "lifted": grade["lifted"], "hands_clear": grade.get("hands_clear"),
                                     "hold_ticks": grade["hold_ticks"]})
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"bag_load_grade": grade})

    def _state_row(self, state):
        base_pos, base_rot = state["base"]
        return {"base_mm": mm(base_pos), "base_tilt_deg": round(grading.tilt_deg(base_rot), 2),
                "rims_mm": {wall: round(1000 * v, 1) for wall, v in state["rims"].items()},
                "grips_mm": {wall: mm(p) for wall, p in state["grips"].items()},
                "carton_mm": mm(state["items"]["carton"][0]), "pack_mm": mm(state["items"]["pack"][0]),
                "carton_tilt_deg": round(grading.tilt_deg(state["items"]["carton"][1]), 1),
                "pack_tilt_deg": round(grading.tilt_deg(state["items"]["pack"][1]), 1),
                "angles_deg": {name: round(float(np.degrees(v)), 1) for name, v in state["angles"].items()}}

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class BagScripted(PolicyBase):
    def __init__(self, variant="full"):
        self.variant = variant

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import BagReference

        self.reference = BagReference(self.variant)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (where the bag's walls
        # and handles and the two items are), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return BagArena(**kwargs)


def make_scripted(variant=None):
    from yam_arena.scripted import YamScriptedPolicy

    policy = BagScripted(variant or os.environ.get("BAG_LOAD_DEMO", "full"))
    policy.info = replace(YamScriptedPolicy().info, name="bag_load_scripted")
    return policy
