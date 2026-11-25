"""Add the filter holder on its flask, the loose funnel and the spring clamp to the stock Inspect Robots YAM station."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import grading
from . import layout as L

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

CURRENT = None                                       # the live embodiment, for the scripted demonstration
PAD_SPAN_MM = 2000 * abs(FINGER_OPEN)                # pads' separation at a gripper reading of 1.0 (93.9 mm)
# Every step from this tick to the end of the clamp's fitting is logged in full (contacts and poses);
# other ticks every 5. Diagnostics only.
TRACE_EVERY = int(os.environ.get("FILTER_CLAMP_TRACE_EVERY", "5") or 5)


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class FilterClampArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_filter_clamp",
                            environment_id="isaaclab-arena-0.3.0-yam-filter-clamp",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a 47 mm glass vacuum filter holder on a 1 L "
                                 "filtering flask, its loose 300 mL funnel and its aluminium spring clamp. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
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
            objects.append(Object(name="filter_clamp_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_filter_clamp", scene=Scene(objects),
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

        actors, clamp = find("Actors"), find("Clamp")
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.ACTORS}
        self.clamp_art = self.physics.create_articulation_view(clamp)
        self.clamp_bodies = self.physics.create_rigid_body_view(clamp + "/Bodies/*")
        cpaths = list(self.clamp_bodies.prim_paths)
        self.clamp_rows = {name: cpaths.index(f"{clamp}/Bodies/{name}") for name in fixture.CLAMP_BODIES}
        self.dof_names = list(self.clamp_art.shared_metatype.dof_names)
        if sorted(self.dof_names) != sorted(fixture.CLAMP_JOINTS):
            raise RuntimeError(f"Unexpected clamp joints: {self.dof_names}")
        self.dof = [self.dof_names.index(n) for n in fixture.CLAMP_JOINTS]
        links = asset_link_paths()
        self.link_names = list(links)
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()] for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        # Every arm link against every task body: one column per body (see lesson: one view, explicit columns).
        self.targets = ("holder", "funnel", "knuckle", "jawL", "jawR")
        target_paths = [f"{actors}/holder", f"{actors}/funnel"] + [f"{clamp}/Bodies/{n}" for n in fixture.CLAMP_BODIES]
        self.arm_contacts = self.physics.create_rigid_contact_view(all_links, [target_paths] * len(all_links))
        # Glass and clamp against each other, each pair its own view so that no force is mislabelled.
        self.pair_views = {}
        for a, b in (("funnel", "holder"), ("jawL", "funnel"), ("jawL", "holder"), ("jawR", "funnel"), ("jawR", "holder"),
                     ("knuckle", "funnel")):
            pa = f"{actors}/{a}" if a in fixture.ACTORS else f"{clamp}/Bodies/{a}"
            pb = f"{actors}/{b}" if b in fixture.ACTORS else f"{clamp}/Bodies/{b}"
            self.pair_views[f"{a}_on_{b}"] = self.physics.create_rigid_contact_view([pa], [[pb]])
        self.tips = [i for i, n in enumerate(self.link_names) if "tip" in n]
        self.one = self._torch.arange(1, dtype=self._torch.int32, device=self._base.device)

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        root = array(self.clamp_art.get_root_transforms()).reshape(-1)
        root[2] -= fixture.TOP
        self.clamp_root = root
        q = array(self.clamp_art.get_dof_positions()).reshape(-1)
        self.jaw_q = np.array([q[self.dof[0]], q[self.dof[1]]])
        if not (np.isfinite(self.poses).all() and np.isfinite(root).all() and np.isfinite(q).all()):
            raise RuntimeError("Nonfinite glassware or clamp state")

    def pose(self, name):
        if name == "clamp":
            return self.clamp_root[:3].copy(), quat_xyzw_to_matrix(self.clamp_root[3:])
        row = self.poses[self.rows[name]]
        return row[:3].copy(), quat_xyzw_to_matrix(row[3:])

    def jaw_open_deg(self):
        return [float(np.degrees(self.jaw_q[0])), float(-np.degrees(self.jaw_q[1]))]

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        # Every body back where the scene file has it; the clamp shut on its stop, lying flat.
        poses = self.body_view.get_transforms().clone()
        for name, start in (("holder", fixture.HOLDER_START), ("funnel", fixture.FUNNEL_START)):
            x, y, z = start + (0., 0., fixture.TOP + .0002)
            poses[self.rows[name]] = torch.tensor((x, y, z, 0., 0., 0., 1.), device=poses.device, dtype=poses.dtype)
        ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        rootpose = self.clamp_art.get_root_transforms().clone()
        x, y, z = fixture.CLAMP_START + (0., 0., fixture.TOP + .0002)
        rootpose[0] = torch.tensor((x, y, z, 0., 0., 0., 1.), device=rootpose.device, dtype=rootpose.dtype)
        self.clamp_art.set_root_transforms(rootpose, self.one)
        self.clamp_art.set_root_velocities(torch.zeros_like(self.clamp_art.get_root_velocities()), self.one)
        q = torch.zeros_like(self.clamp_art.get_dof_positions())
        q[0, self.dof[0]], q[0, self.dof[1]] = float(L.THETA_MIN), float(-L.THETA_MIN)
        self.clamp_art.set_dof_positions(q, self.one)
        self.clamp_art.set_dof_velocities(torch.zeros_like(q), self.one)
        observation = super().reset(scene, seed=seed)
        self._read()
        for name, start in (("holder", fixture.HOLDER_START), ("funnel", fixture.FUNNEL_START), ("clamp", fixture.CLAMP_START)):
            p, r = self.pose(name)
            if np.linalg.norm(p - start) > .004 or L.tilt_deg(r) > 2.:
                raise RuntimeError(f"{name} is off its start pose after reset: {p}, tilt {L.tilt_deg(r):.2f} deg")
        if max(self.jaw_open_deg()) > 1.:
            raise RuntimeError(f"Clamp is not shut after reset: {self.jaw_open_deg()}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        masses = array(self.body_view.get_masses()).reshape(-1)
        cm = array(self.clamp_bodies.get_masses()).reshape(-1)
        self.masses = {**{n: round(float(masses[r]), 5) for n, r in self.rows.items()},
                       **{n: round(float(cm[r]), 5) for n, r in self.clamp_rows.items()}}
        self.grading = grading.FilterClampGrade(holder_start=tuple(fixture.HOLDER_START), clamp_start=tuple(fixture.CLAMP_START))
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peaks = {}
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self.after_reset = {n: [round(1000 * float(v), 2) for v in self.pose(n)[0]] for n in ("holder", "funnel", "clamp")}
        self.after_reset["jaw_open_deg"] = [round(v, 2) for v in self.jaw_open_deg()]
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {name: self.pose(name) for name in ("holder", "funnel", "clamp")}
        self.demo_state["jaw_open_deg"] = self.jaw_open_deg()

    def _grippers(self, joints):
        tcps, rots, openings = [], [], []
        for i in (0, 7):
            p, r = ik.tcp_pose(joints[i:i + 6], fixture.BASES[fixture.ARMS[i // 7]])
            tcps.append(p)
            rots.append(r)
            openings.append(float(joints[i + 6]) * PAD_SPAN_MM)
        return tcps, rots, openings

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.tick += 1
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        tcps, rots, openings = self._grippers(joints)
        hp, hr = self.pose("holder")
        fp, fr = self.pose("funnel")
        cp, cr = self.pose("clamp")
        contacts = self._contacts()
        grade = self.grading.update({"holder_pos": hp, "holder_rot": hr, "funnel_pos": fp, "funnel_rot": fr,
                                     "clamp_pos": cp, "clamp_rot": cr, "jaw_open_deg": self.jaw_open_deg(),
                                     "tcp": tcps, "contacts": contacts})
        self.final_grade = grade
        self._publish_demo_state()
        self.demo_state["funnel_on_holder_n"] = round(contacts["funnel_on_holder"], 2)     # for the demo's log only
        for key, value in contacts.items():
            peak = float(np.max(np.abs(value))) if np.size(value) else 0.
            self.peaks[key] = round(max(self.peaks.get(key, 0.), peak), 2)
        busy = grade["assembled"] or grade["seated"] and self.tick % 2 == 0
        if busy or self.tick % TRACE_EVERY == 0:
            self.contact_trace.append(self._trace_row(grade, contacts, fp, cp, openings))
        if self.tick % 5 == 0:
            self.diagnostics.append(self._diagnose(joints, grade, tcps, rots, openings, contacts))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success, termination_reason="success" if success else None,
                          info={"filter_clamp_grade": grade})

    def _contacts(self):
        """Normal contact forces (N) at the last physics step of the control interval."""
        forces = np.linalg.norm(array(self.arm_contacts.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        count = len(forces) // 2
        col = {n: i for i, n in enumerate(self.targets)}
        out = {}
        for arm, rows in (("left", slice(0, count)), ("right", slice(count, 2 * count))):
            block = forces[rows]
            for n in self.targets:
                out[f"{arm}_arm_on_{n}"] = float(block[:, col[n]].sum())
            out[f"{arm}_tips_on_funnel"] = [float(block[i, col["funnel"]]) for i in self.tips]
            out[f"{arm}_tips_on_jaws"] = [float(block[i, col["jawL"]] + block[i, col["jawR"]]) for i in self.tips]
        for key, view in self.pair_views.items():
            out[key] = float(np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT))))
        return out

    def _trace_row(self, grade, contacts, fp, cp, openings):
        r = lambda v: round(float(v), 2)
        return {"tick": self.tick, "score": grade["score"], "jaw_deg": grade["jaw_open_deg"],
                "funnel_mm": [r(1000 * v) for v in fp], "clamp_mm": [r(1000 * v) for v in cp],
                "funnel_offset_mm": grade["funnel_offset_mm"], "funnel_gap_mm": grade["funnel_gap_mm"],
                "foot_clearance_mm": grade["foot_clearance_mm"], "clamp_off_axis_mm": grade["clamp_off_axis_mm"],
                "clamp_height_mm": grade["clamp_height_mm"], "upper_support": grade["upper_support"], "opening_mm": [r(o) for o in openings],
                "left_tips_on_funnel_n": [r(v) for v in contacts["left_tips_on_funnel"]],
                "right_tips_on_jaws_n": [r(v) for v in contacts["right_tips_on_jaws"]],
                **{f"{k}_n": r(v) for k, v in contacts.items() if "_on_" in k and k.split("_on_")[0] in ("funnel", "jawL", "jawR", "knuckle")}}

    def _diagnose(self, joints, grade, tcps, rots, openings, contacts):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        fp, fr = self.pose("funnel")
        cp, cr = self.pose("clamp")
        hp, hr = self.pose("holder")
        rnd = lambda v, k=1: [round(1000 * float(x), k) for x in v]
        row = {"tick": self.tick, **grade,
               "holder_mm": rnd(hp), "funnel_mm": rnd(fp), "clamp_mm": rnd(cp),
               "tcp_mm": [rnd(p) for p in tcps], "opening_mm": [round(o, 2) for o in openings],
               "funnel_in_left_hand_mm": rnd(rots[0].T @ (fp - tcps[0])),
               "clamp_in_right_hand_mm": rnd(rots[1].T @ (cp - tcps[1])),
               "clamp_axis_in_right_hand": [round(float(v), 3) for v in rots[1].T @ cr[:, 2]]}
        row.update({k: ([round(x, 2) for x in v] if isinstance(v, list) else round(v, 2)) for k, v in contacts.items()})
        return row

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class FilterClampScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import FilterClampReference

        self.reference = FilterClampReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the funnel's, the
        # holder's and the clamp's poses and the jaws' angles), which the review rules allow for a scripted
        # demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return FilterClampArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = FilterClampScripted()
    policy.info = replace(YamScriptedPolicy().info, name="filter_clamp_scripted")
    return policy
