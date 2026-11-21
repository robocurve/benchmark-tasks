"""Add the TLC bench (plate on its bar, spotters in their rack, sample vials, the developing tank and its lid) to the stock
Inspect Robots YAM station; run the filling, spotting, gouging and breaking rules every control step and feed the
scorer.

Pattern: the ukulele package's adapter (``rdb_ukulele_tuning/adapter.py``: the review camera, the window, the
diagnostics), with the loose-body views of the filter-clamp package (``rdb_filter_clamp/adapter.py``: one rigid-body
view of the ``Actors`` scope, poses set at reset) and its explicit contact columns.
"""

from dataclasses import replace
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as fixture
from . import layout as L
from . import tlc
from .grading import TLCGrade

CURRENT = None                                       # the live embodiment, for the scripted demonstration
BASES = {arm: np.asarray(fixture.BASES[arm], dtype=float) for arm in fixture.ARMS}
PAD_SPAN = 2 * abs(FINGER_OPEN)                      # pads' separation at a gripper reading of 1.0 (93.9 mm)


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class TLCArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None, headless=True):
        global CURRENT
        # headless=False shows the Isaac Sim window on DISPLAY or the runtime's browser desktop
        # (yam_arena/inspect_embodiment.py, -E headless=false); it does not change the trial.
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176, headless=headless)
        self.info = replace(self.info, name="yam_tlc_spotting",
                            environment_id="isaaclab-arena-0.3.1-yam-tlc-spotting",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a glass-backed TLC plate on a narrow bar, three "
                                 "empty glass capillary spotters in a rack, three vials of dyed water and a "
                                 "rectangular glass TLC developing tank (115 x 77 mm inside) with 5 mm of water and its lid. Observations are overhead RGB, both "
                                 "wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.skin = None
        self.skin_error = None
        self.tick = 0
        self.final_grade = None
        self.diagnostics = []
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
            objects.append(Object(name="tlc_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_tlc_spotting", scene=Scene(objects),
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
        found = [str(p.GetPath()) for p in stage.Traverse()
                 if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(found) != 1:
            raise RuntimeError(f"Expected one TLC fixture, found {found}")
        actors = found[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.ACTORS}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()] for arm in fixture.ARMS}
        self.fingers = [p for arm in fixture.ARMS for p in arm_links[arm] if "tip" in p.rsplit("/", 1)[-1]]
        if len(self.fingers) != 4:
            raise RuntimeError(f"Expected four finger links, found {self.fingers}")
        spotters = [f"{actors}/{name}" for name in fixture.SPOTTERS]
        plate = f"{actors}/plate"
        # Each spotter against the plate (column 0) and the four fingers; the net force gives the rest.
        self.spot_view = self.physics.create_rigid_contact_view(spotters, [[plate] + self.fingers] * 3)
        # The plate against the fingers, the tank and the lid (diagnostics only).
        others = [f"{actors}/tank", f"{actors}/lid"]
        self.plate_view = self.physics.create_rigid_contact_view([plate], [self.fingers + others])
        self.lid_view = self.physics.create_rigid_contact_view([f"{actors}/lid"], [self.fingers + [f"{actors}/tank"]])
        self.interarm = self.physics.create_rigid_contact_view(arm_links["left"], [arm_links["right"]] * len(links))
        if self.skin is None:
            from .skin import MarkSkin
            self.skin = MarkSkin(stage, actors)

    # -- state ----------------------------------------------------------------------------------------------------
    def _read(self):
        raw = array(self.body_view.get_transforms()).astype(np.float64)     # float32 does not go into JSON
        if not np.isfinite(raw).all():
            raise RuntimeError("Nonfinite TLC bench state")
        raw[:, 2] -= fixture.TOP
        self.poses = {name: (raw[row, :3].copy(), raw[row, 3:7].copy()) for name, row in self.rows.items()}

    def forces(self):
        """Each spotter's force from the plate and from anything but the pads and the plate (N), the pads' grip on
        each spotter, and the plate's and the lid's contacts (diagnostics)."""
        matrix = array(self.spot_view.get_contact_force_matrix(dt=PHYSICS_DT))          # (3, 5, 3)
        net = array(self.spot_view.get_net_contact_forces(dt=PHYSICS_DT)).reshape(3, 3)
        plate = np.linalg.norm(matrix[:, 0], axis=-1)
        rest = np.linalg.norm(net - matrix.sum(axis=1), axis=-1)
        pads = np.linalg.norm(matrix[:, 1:], axis=-1).sum(axis=-1)
        pm = np.linalg.norm(array(self.plate_view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).reshape(-1)
        wm = np.linalg.norm(array(self.lid_view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).reshape(-1)
        return {"plate": plate.tolist(), "other": rest.tolist(), "pads": pads.tolist(),
                "plate_fingers": [float(pm[0] + pm[1]), float(pm[2] + pm[3])], "plate_tank": float(pm[4]),
                "plate_lid": float(pm[5]), "lid_fingers": [float(wm[0] + wm[1]), float(wm[2] + wm[3])],
                "lid_tank": float(wm[4])}

    def _show(self, grade):
        try:
            self.skin.update(grade)
        except Exception as error:   # the marks are display only; never let them stop a trial
            self.skin_error = repr(error)

    # -- episode --------------------------------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        poses = self.body_view.get_transforms().clone()
        for name, ((x, y, z), q) in fixture.START.items():
            poses[self.rows[name]] = torch.tensor((x, y, z + fixture.TOP, *q), device=poses.device, dtype=poses.dtype)
        ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        self.state_after_reset = {}
        for name, (start, _) in fixture.START.items():
            p, q = self.poses[name]
            tilt = tlc.tilt_deg((p, q))
            self.state_after_reset[name] = {"mm": [round(1000 * float(v), 2) for v in p], "tilt_deg": round(tilt, 2)}
            if np.linalg.norm(p - np.asarray(start)) > .003 or tilt > 3.:
                raise RuntimeError(f"{name} is off its start pose after reset: {self.state_after_reset[name]}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        self.state_after_reset["masses_g"] = {n: round(1000 * float(masses[r]), 3) for n, r in self.rows.items()}
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = TLCGrade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak = {"spotter_plate_n": [0., 0., 0.], "spotter_other_n": [0., 0., 0.], "spotter_pads_n": [0., 0., 0.],
                     "plate_fingers_n": [0., 0.], "plate_tank_n": 0., "lid_fingers_n": [0., 0.]}
        self.final_grade = None
        self.diagnostics = []
        self.last_forces = None
        self._publish_demo_state(None, None)
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self, forces, grade):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"poses": {n: (p.copy(), q.copy()) for n, (p, q) in self.poses.items()},
                           "forces": forces, "grade": grade}

    def tool_points(self, joints):
        return [ik.tcp_pose(joints[i:i + 6], BASES[arm]) for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = self.forces()
        tools = self.tool_points(joints)
        self.tick += 1
        grade = self.grading.update(self.poses, forces["plate"], forces["other"], [p for p, _ in tools], forces["pads"])
        self.final_grade = grade
        self._show(grade)
        self._publish_demo_state(forces, grade)
        self._peaks(forces)
        if self.tick % 5 == 0 or any(f > .02 for f in forces["plate"]):
            self.diagnostics.append(self._diagnose(joints, tools, forces, grade))
        over = self.grading.finished
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, {k: grade[k] for k in (
                "score", "points", "lanes", "standing", "leaning", "covered", "plate", "lid", "racked", "loads",
                "spots", "gouges", "void")}, terminal=over or self.tick == fixture.MAX_STEPS)
        reason = None if not over else ("success" if grade["score"] == 1. else "void")
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"tlc_grade": grade})

    def _peaks(self, forces):
        for key, name in (("spotter_plate_n", "plate"), ("spotter_other_n", "other"), ("spotter_pads_n", "pads")):
            self.peak[key] = [max(a, float(b)) for a, b in zip(self.peak[key], forces[name])]
        self.peak["plate_fingers_n"] = [max(a, b) for a, b in zip(self.peak["plate_fingers_n"], forces["plate_fingers"])]
        self.peak["lid_fingers_n"] = [max(a, b) for a, b in zip(self.peak["lid_fingers_n"], forces["lid_fingers"])]
        self.peak["plate_tank_n"] = max(self.peak["plate_tank_n"], forces["plate_tank"])

    def _diagnose(self, joints, tools, forces, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        r = lambda v, k=1: [round(1000 * float(x), k) for x in v]
        return {
            "tick": self.tick, "score": grade["score"], "points": grade["points"],
            "tool_mm": [r(p) for p, _ in tools],
            "gripper_mm": [round(float(joints[6]) * PAD_SPAN * 1000, 2), round(float(joints[13]) * PAD_SPAN * 1000, 2)],
            "tips_mm": [r(tlc.spotter_tip(self.poses[n]), 2) for n in fixture.SPOTTERS],
            "plate_mm": r(self.poses["plate"][0], 2), "plate_quat": [round(float(v), 4) for v in self.poses["plate"][1]],
            "tank_mm": r(self.poses["tank"][0], 2), "lid_mm": r(self.poses["lid"][0], 2),
            "forces_n": {k: ([round(float(x), 3) for x in v] if isinstance(v, list) else round(float(v), 3))
                         for k, v in forces.items()},
            "plate_state": grade["plate"], "lid": grade["lid"], "loads": grade["loads"],
            "lanes": grade["lanes"], "racked": grade["racked"], "clearance_mm": grade["clearance_mm"],
            "void": grade["void"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class TLCScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import TLCReference

        self.reference = TLCReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the bodies' poses, the
        # spotters' contact forces, what the rules have recorded), which the review rules allow for a scripted
        # demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return TLCArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = TLCScripted()
    policy.info = replace(YamScriptedPolicy().info, name="tlc_spotting_scripted")
    return policy
