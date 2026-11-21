"""Add the loaded sit-stand desk converter to the stock Inspect Robots YAM station and grade every control step."""

from dataclasses import replace
import math
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import layout as L
from .grading import TOUCH_N, make_grade

CURRENT = None                                       # the live embodiment, for the scripted demonstration
PHYSICS_DT = 1. / fixture.PHYSICS_HZ                 # contact impulses are per physics step; pass dt on every read
TRACE_COLUMNS = ("tick", "setting", "latch_l_mm", "latch_r_mm", "pad_on_paddle_l_n", "pad_on_paddle_r_n",
                 "pads_on_top_l_n", "pads_on_top_r_n", "pin_on_teeth_l_n", "pin_on_teeth_r_n", "arm_touch_l_n",
                 "arm_touch_r_n", "mimic_err_deg", "top_path_err_mm")


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def plain(value):
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


class ConverterArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_desk_converter",
                            environment_id="isaaclab-arena-0.3.1-yam-desk-converter",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a loaded sit-stand desk converter (monitor on "
                                 "the top, keyboard on the tray). Observations are overhead RGB, both wrist RGB views, "
                                 "and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.final_grade = None
        self.tick = 0
        self.peak_interarm_n = 0.
        self.samples = []
        self.contact_trace = []
        self.joint_trace = []
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
            objects.append(Object(name="desk_converter_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_desk_converter", scene=Scene(objects),
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

        def find(name):
            found = [str(p.GetPath()) for p in stage.Traverse()
                     if p.GetName() == name and str(p.GetPath()).startswith(root + "/")]
            if len(found) != 1:
                raise RuntimeError(f"Expected one {name}, found {found}")
            return found[0]

        conv, loads = find(fixture.ROOT_NAME), find("Loads")
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(conv + "/Bodies/*")
        self.load_view = self.physics.create_rigid_body_view(loads + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{conv}/Bodies/{name}") for name in fixture.BODIES}
        load_paths = list(self.load_view.prim_paths)
        self.load_rows = {name: load_paths.index(f"{loads}/{name}") for name in fixture.LOADS}
        self.art = self.physics.create_articulation_view(conv)
        names = list(self.art.shared_metatype.dof_names)
        if sorted(names) != sorted(fixture.JOINTS):
            raise RuntimeError(f"Unexpected converter joints: {names}")
        self.dofs = {name: names.index(name) for name in fixture.JOINTS}
        links = asset_link_paths()
        self.link_names = list(links)
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()] for arm in fixture.ARMS}
        tips = {arm: [root + "/" + arm.title() + "Arm" + links[n] for n in ("tip_left", "tip_right")] for arm in fixture.ARMS}
        body = {n: f"{conv}/Bodies/{n}" for n in fixture.BODIES}
        load = {n: f"{loads}/{n}" for n in fixture.LOADS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        # Every contact pair that matters has its own view (a shared view once mislabelled a force for us).
        self.views = {}
        for arm in fixture.ARMS:
            self.views[f"pad_on_paddle_{arm}"] = self.physics.create_rigid_contact_view(tips[arm], [[body[f"latch_{arm}"]]] * 2)
            self.views[f"pads_on_top_{arm}"] = self.physics.create_rigid_contact_view(tips[arm], [[body["top"]]] * 2)
            others = [body["top"], body["latch_left"], body["latch_right"], body["lift_arm"], body["base"],
                      load["monitor"], load["keyboard"]]
            self.views[f"arm_touch_{arm}"] = self.physics.create_rigid_contact_view(
                arm_links[arm], [others] * len(arm_links[arm]))
            self.views[f"pin_on_teeth_{arm}"] = self.physics.create_rigid_contact_view([body[f"latch_{arm}"]],
                                                                                       [[body["lift_arm"]]])
        self.one = self._torch.arange(1, dtype=self._torch.int32, device=self._base.device)

    # -- reading ---------------------------------------------------------------------------------------------
    def _read(self):
        poses = array(self.body_view.get_transforms())
        poses[:, 2] -= fixture.TOP
        loads = array(self.load_view.get_transforms())
        loads[:, 2] -= fixture.TOP
        dof = array(self.art.get_dof_positions()).reshape(-1)
        if not (np.isfinite(poses).all() and np.isfinite(loads).all() and np.isfinite(dof).all()):
            raise RuntimeError("Nonfinite converter state")
        self.poses, self.loads, self.dof = poses, loads, dof
        self.theta = fixture.START_THETA + float(dof[self.dofs[fixture.LIFT]])
        self.latch = {arm: float(dof[self.dofs[fixture.LATCH[arm]]]) for arm in fixture.ARMS}

    def pose(self, name):
        if name in self.load_rows:
            row = self.loads[self.load_rows[name]]
        else:
            row = self.poses[self.rows[name]]
        return row[:3].copy(), quat_xyzw_to_matrix(row[3:])

    def force(self, key):
        return float(np.linalg.norm(array(self.views[key].get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum())

    # -- episode -----------------------------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        zero = torch.zeros_like(self.art.get_dof_positions())
        self.art.set_dof_positions(zero, self.one)
        self.art.set_dof_velocities(zero, self.one)
        targets = zero.clone()
        targets[0, self.dofs[fixture.LIFT]] = float(fixture.LIFT_TARGET)
        for arm in fixture.ARMS:
            targets[0, self.dofs[fixture.LATCH[arm]]] = -float(fixture.LATCH_PRELOAD)
        self.art.set_dof_position_targets(targets, self.one)
        transforms = self.load_view.get_transforms().clone()
        for name, start in (("monitor", fixture.MONITOR_START), ("keyboard", fixture.KEYBOARD_START)):
            transforms[self.load_rows[name]] = torch.tensor((start[0], start[1], start[2] + fixture.TOP, 0., 0., 0., 1.),
                                                            device=transforms.device, dtype=transforms.dtype)
        ids = torch.arange(len(fixture.LOADS), dtype=torch.int32, device=self._base.device)
        self.load_view.set_transforms(transforms, ids)
        self.load_view.set_velocities(torch.zeros_like(self.load_view.get_velocities()), ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        top, _ = self.pose("top")
        self.state_after_reset = {
            "setting": round(float(L.setting_of(self.theta)), 4), "latch_mm": {a: round(1000 * v, 3) for a, v in self.latch.items()},
            "top_mm": [round(1000 * float(v), 2) for v in top],
            "top_expected_mm": [round(1000 * float(v), 2) for v in fixture.TOP_START],
            "monitor_mm": [round(1000 * float(v), 2) for v in self.pose("monitor")[0]],
            "keyboard_mm": [round(1000 * float(v), 2) for v in self.pose("keyboard")[0]],
            "dof": {n: round(float(self.dof[i]), 5) for n, i in self.dofs.items()},
        }
        start_setting = L.setting_of(fixture.START_THETA)
        self.state_after_reset["seated_start_setting"] = round(float(start_setting), 4)
        if (abs(L.setting_of(self.theta) - start_setting) > fixture.RESET_SETTING_TOL
                or np.linalg.norm(top - fixture.TOP_START) > fixture.RESET_TOP_TOL
                or max(self.latch.values()) > fixture.RESET_LATCH_TOL):
            raise RuntimeError(f"Converter is not locked at setting 4 after reset: {self.state_after_reset}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        masses = array(self.body_view.get_masses()).reshape(-1)
        self.masses = {n: round(float(masses[r]), 4) for n, r in self.rows.items()}
        lmasses = array(self.load_view.get_masses()).reshape(-1)
        self.masses.update({n: round(float(lmasses[r]), 4) for n, r in self.load_rows.items()})
        self.grading = make_grade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peaks = {k: 0. for k in self.views}
        self.final_grade = None
        self.samples, self.contact_trace, self.joint_trace = [], [], []
        self.last_command = None
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"theta": self.theta, "latch": dict(self.latch)}

    def step(self, action):
        command = np.asarray(action.data, dtype=float).reshape(-1)
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = {k: self.force(k) for k in self.views}
        for k, v in forces.items():
            self.peaks[k] = max(self.peaks[k], v)
        touching = any(forces[f"arm_touch_{a}"] > TOUCH_N for a in fixture.ARMS)
        tcps = {arm: ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)}
        top_pos, top_rot = self.pose("top")
        self.tick += 1
        grade = self.grading.update(self.theta, self.latch, top_pos, top_rot, self.pose("monitor"), self.pose("keyboard"),
                                    tcps, touching)
        self.final_grade = grade
        mimic_err = math.degrees(float(self.dof[self.dofs[fixture.LEVEL]] + self.dof[self.dofs[fixture.LIFT]]))
        path_err = 1000 * float(np.linalg.norm(top_pos - L.top_origin(self.theta)))
        # commanded vs measured joints, every step (the YAM wrist lags about 0.15 s, the shoulder about 0.07 s)
        self.joint_trace.append([self.tick, [round(float(v), 4) for v in command], [round(float(v), 4) for v in joints]])
        self.contact_trace.append([self.tick, grade["setting"], grade["latch_mm"]["left"], grade["latch_mm"]["right"],
                                   *(round(forces[k], 2) for k in ("pad_on_paddle_left", "pad_on_paddle_right",
                                                                   "pads_on_top_left", "pads_on_top_right",
                                                                   "pin_on_teeth_left", "pin_on_teeth_right",
                                                                   "arm_touch_left", "arm_touch_right")),
                                   round(mimic_err, 3), round(path_err, 2)])
        if self.tick % 5 == 0:
            self.samples.append(self._diagnose(joints, command, forces, grade, tcps, top_pos, top_rot, mimic_err, path_err))
        self._publish_demo_state()
        success = grade["score"] == 1. and self.grading.finished
        over = success
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=over, termination_reason="success" if success else None,
                          info={"desk_converter_grade": grade})

    def _diagnose(self, joints, command, forces, grade, tcps, top_pos, top_rot, mimic_err, path_err):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        rear_err = math.degrees(float(self.dof[self.dofs[fixture.REAR]] - self.dof[self.dofs[fixture.LIFT]]))
        roll = math.degrees(math.asin(float(np.clip(top_rot[2, 1], -1., 1.))))
        pitch = math.degrees(math.asin(float(np.clip(-top_rot[2, 0], -1., 1.))))
        row = {
            "tick": self.tick, "score": grade["score"], "tier": grade["tier"], "setting": grade["setting"],
            "theta_deg": round(math.degrees(self.theta), 3), "latch_mm": grade["latch_mm"], "seated": grade["seated"],
            "net_push_model_n": round(L.net_push(self.theta), 2),
            "mimic_err_deg": round(mimic_err, 3), "rear_err_deg": round(rear_err, 3),
            "top_path_err_mm": round(path_err, 2), "top_roll_deg": round(roll, 3), "top_pitch_deg": round(pitch, 3),
            "top_mm": [round(1000 * float(v), 1) for v in top_pos],
            "tcp_mm": {a: [round(1000 * float(v), 1) for v in tcps[a]] for a in fixture.ARMS},
            "tcp_from_grasp_mm": {a: [round(1000 * float(v), 1) for v in tcps[a] - L.grasp_point(a, self.theta)]
                                  for a in fixture.ARMS},
            "jaw_measured": [round(float(joints[6]), 4), round(float(joints[13]), 4)],
            "jaw_command": [round(float(command[6]), 4), round(float(command[13]), 4)],
            "contact_n": {k: round(v, 2) for k, v in forces.items()},
            "monitor_moved_mm": grade["monitor_moved_mm"], "keyboard_moved_mm": grade["keyboard_moved_mm"],
            "monitor_tilt_deg": grade["monitor_tilt_deg"], "touching": grade["touching"], "clear": grade["clear"],
        }
        return row

    def diagnostics(self, reference=None):
        phase_ends = []
        if reference is not None:
            starts = reference.phases + [(self.tick + 1, "end")]
            for (first, name), (after, _) in zip(starts, starts[1:]):
                rows = [r for r in self.samples if first < r["tick"] <= after]
                if rows:
                    phase_ends.append({"phase": name, **rows[-1]})
        return plain({
            "state_after_reset": self.state_after_reset, "body_masses_kg": getattr(self, "masses", None),
            "peak_contact_n": {k: round(v, 2) for k, v in getattr(self, "peaks", {}).items()},
            "phase_ends": phase_ends, "samples": self.samples,
            "contact_trace_columns": list(TRACE_COLUMNS), "contact_trace": self.contact_trace,
            "joint_trace_columns": ["tick", "commanded_14", "measured_14"], "joint_trace": self.joint_trace,
        })

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class ConverterScripted(PolicyBase):
    variant = None

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import ConverterReference

        self.reference = ConverterReference(self.variant)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the lift arm's angle and
        # the two latches' heights), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return ConverterArena(**kwargs)


def make_scripted(variant=None):
    from yam_arena.scripted import YamScriptedPolicy

    policy = ConverterScripted()
    policy.variant = variant
    policy.info = replace(YamScriptedPolicy().info, name="desk_converter_scripted")
    return policy
