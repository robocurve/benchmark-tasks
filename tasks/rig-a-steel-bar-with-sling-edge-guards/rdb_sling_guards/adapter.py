"""Add the rigging fixture (``rigging.usda``: the bar on its blocks, the web sling, four printed magnetic edge guards in
two trays, the double hook on the hoist) to the stock Inspect Robots YAM station; pull each guard toward the bar at
every 240 Hz physics substep; measure the scene every control step for the scorer; run the operator's test (the hoist
takes up the slack and holds 50 N by force control) when the scorer calls for it.

Pattern: ``rdb_spark_plug/adapter.py`` (the fixture, physics views, diagnostics) with the per-substep forces of
``rdb_window_wall/adapter.py`` (``scene.write_data_to_sim`` runs before each PhysX step of a control period).
"""

from dataclasses import replace
import math
from pathlib import Path
import time

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as F
from . import grading
from . import hook as HK
from . import layout as L
from . import magnet

if grading.CONTROL_HZ != F.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

CURRENT = None                                       # the live embodiment, for the scripted demonstration
BASES = {arm: np.asarray(F.BASES[arm], dtype=float) for arm in F.ARMS}
PAD_SPAN_MM = 2000 * abs(FINGER_OPEN)
EDGES = grading.EDGE_NAMES
EDGE_YZ = {e: np.array(L.EDGES[e][0]) for e in EDGES}
REST_TO_O = np.array((0., 0., -(L.EYE_TOP + L.PEND_L)))  # the hook frame's origin from the pendulum's pivot, at rest


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def quat_R(q):
    """Rotation matrix from (x, y, z, w)."""
    return np.array(grading.quat_matrix(tuple(float(v) for v in q)))


class RiggingArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None, headless=True):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=F.CONTROL_HZ,
                         cam_width=320, cam_height=176, headless=headless)
        self.info = replace(self.info, name="yam_sling_guards",
                            environment_id="isaaclab-arena-0.3.1-yam-sling-guards",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a 4 x 4 x 12 in steel bar on two blocks, a 3 ft "
                                 "two-ply web sling with triangle fittings laid under it, four magnetic edge guards in "
                                 "two trays, and a latched double hook on a chain hoist over the bar. Observations are "
                                 "overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.tick = 0
        self.final_grade = None
        self.diagnostics = []
        self.state_after_reset = None
        self.magnets_on = False
        self.magnets_near = True         # some guard is near enough to the bar for its discs to pull (checked each step)
        self.timing = {}                 # wall seconds spent per part of a control step (diagnostics)
        self.start = None
        self.magnet_gap = {}
        self.stop_check = None           # run.py's Dream trial: () -> a termination reason once its requests are spent
        CURRENT = self

    # -- the simulation ---------------------------------------------------------------------------------------------
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
            objects.append(Object(name="rigging_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(F.USD))))
            env = IsaacLabArenaEnvironment(name="yam_sling_guards", scene=Scene(objects),
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
        # the guards' magnets act at every physics substep: Isaac Lab calls write_data_to_sim() once before each of the
        # PhysX steps inside one control period (rdb_window_wall/adapter.py)
        scene = self._base.scene
        stock_write = scene.write_data_to_sim

        def write_data_to_sim(*args, **kwargs):
            stock_write(*args, **kwargs)
            if self.magnets_on and self.magnets_near:
                self.apply_magnets()

        scene.write_data_to_sim = write_data_to_sim

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

        sling, rig, guards, bar = find(F.SLING), find(F.RIG), find(F.GUARDS), find(F.BAR)
        self.paths = {"sling": sling, "rig": rig, "guards": guards, "bar": bar}
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.sling = self.physics.create_articulation_view(f"{sling}/{F.SLING_ROOT}")
        self.rig = self.physics.create_articulation_view(rig)
        self.guards = self.physics.create_rigid_body_view(guards + "/*")
        gpaths = list(self.guards.prim_paths)
        self.grow = {n: gpaths.index(f"{guards}/{n}") for n in F.GUARD_NAMES}
        self.gids = self._torch.arange(len(gpaths), dtype=self._torch.int32, device=self._base.device)
        self.one = self._torch.arange(1, dtype=self._torch.int32, device=self._base.device)
        self.sling_links = list(self.sling.shared_metatype.link_names)
        self.sling_dofs = list(self.sling.shared_metatype.dof_names)
        self.rig_links = list(self.rig.shared_metatype.link_names)
        self.rig_dofs = list(self.rig.shared_metatype.dof_names)
        if sorted(self.rig_dofs) != sorted(F.RIG_JOINTS):
            raise RuntimeError(f"Unexpected hook-rig joints: {self.rig_dofs}")
        links = [f"{sling}/{n}" for n in F.LINKS]
        fits = [f"{sling}/{n}" for n in F.FITS]
        hook = [f"{rig}/hook", f"{rig}/latchL", f"{rig}/latchR"]
        # each pair its own view, so that no force is mislabelled (rdb_filter_clamp/adapter.py)
        self.web_bar = self.physics.create_rigid_contact_view(links, [[bar]] * len(links), max_contact_data_count=4096)
        self.web_guard = {n: self.physics.create_rigid_contact_view(links, [[f"{guards}/{n}"]] * len(links),
                                                                    max_contact_data_count=2048) for n in F.GUARD_NAMES}
        self.fit_hook = self.physics.create_rigid_contact_view(fits, [hook] * 2, max_contact_data_count=512)
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in asset_link_paths().values()]
                     for arm in F.ARMS}
        self.task_bodies = hook + fits + links + [f"{guards}/{n}" for n in F.GUARD_NAMES]   # "clear of the sling" too
        all_arm = arm_links["left"] + arm_links["right"]
        self.arm_contacts = self.physics.create_rigid_contact_view(all_arm, [self.task_bodies] * len(all_arm))
        self.arm_link_count = len(arm_links["left"])

    # -- magnets ----------------------------------------------------------------------------------------------------
    NEAR = L.MAG_RANGE + L.GUARD_FL + .030       # a guard's corner this far from the bar: its discs may come in range
                                                 # within one control period

    def check_magnets_near(self):
        """Once per control step: whether any guard's corner is within NEAR of the bar. When none is, every disc is
        beyond MAG_RANGE for the whole control period and the per-substep pull (zero) is skipped."""
        poses = array(self.guards.get_transforms())[:, :3] - (0., 0., F.TOP)
        gap = np.linalg.norm(poses - np.clip(poses, magnet.BAR_LO, magnet.BAR_HI), axis=1)
        self.magnets_near = bool((gap < self.NEAR).any())

    def apply_magnets(self):
        t0 = time.perf_counter()
        poses = array(self.guards.get_transforms())
        forces = np.zeros((len(poses), 3), dtype=np.float32)
        torques = np.zeros_like(forces)
        for n, row in self.grow.items():
            p = poses[row, :3] - (0., 0., F.TOP)
            f, tq, gap = magnet.on_guard(p, quat_R(poses[row, 3:]), self.coms[row])
            forces[row], torques[row] = f, tq
            self.magnet_gap[n] = gap
        torch = self._torch
        self.guards.apply_forces_and_torques_at_position(torch.as_tensor(forces, device=self._base.device),
                                                         torch.as_tensor(torques, device=self._base.device), None,
                                                         self.gids, True)
        self.timing["magnets"] = self.timing.get("magnets", 0.) + time.perf_counter() - t0

    # -- measurements -----------------------------------------------------------------------------------------------
    def _read(self):
        self.g_poses = array(self.guards.get_transforms())
        self.s_poses = array(self.sling.get_link_transforms())[0]
        self.r_poses = array(self.rig.get_link_transforms())[0]
        self.r_dofs = array(self.rig.get_dof_positions())[0]
        if not (np.isfinite(self.g_poses).all() and np.isfinite(self.s_poses).all() and np.isfinite(self.r_poses).all()):
            raise RuntimeError("Nonfinite scene state")

    def hook_frame(self):
        """(origin, rotation) of the hook's own frame (midway between the bowl centres; y across, z up) in the
        tabletop frame."""
        pose = self.r_poses[self.rig_links.index("hook")]
        R = quat_R(pose[3:])
        pivot = pose[:3] - (0., 0., F.TOP)
        return pivot + R @ REST_TO_O, R

    def fittings(self):
        """Each fitting's base-bar centre and apex-bar centre (tabletop) and its base-to-apex direction."""
        out = {}
        for name, sgn in (("fitA", -1.), ("fitB", 1.)):
            pose = self.s_poses[self.sling_links.index(name)]
            R = quat_R(pose[3:])
            base = pose[:3] - (0., 0., F.TOP)
            apex = base + R @ np.array((0., sgn * L.FIT_H, 0.))
            out[name] = {"base": base, "apex": apex, "u": (apex - base) / L.FIT_H, "bar": R[:, 0]}
        return out

    def latch_deg(self):
        return {s: math.degrees(float(self.r_dofs[self.rig_dofs.index(f"latch{s}_hinge")])) for s in ("L", "R")}

    def pull(self):
        mat = array(self.fit_hook.get_contact_force_matrix(dt=PHYSICS_DT))
        return abs(float(mat.reshape(-1, 3).sum(axis=0)[2]))

    def edge_contacts(self, seated):
        """Web on bare steel within 15 mm of each edge (contact points from the web-bar view), and web on the guard
        seated at each edge (N)."""
        forces, points, _, _, counts, starts = self.web_bar.get_contact_data(dt=PHYSICS_DT)
        forces, points = array(forces).reshape(-1), array(points).reshape(-1, 3)
        counts, starts = array(counts).reshape(len(F.LINKS), -1), array(starts).reshape(len(F.LINKS), -1)
        bare = dict.fromkeys(EDGES, 0.)
        for i in range(len(F.LINKS)):
            for k in range(int(starts[i, 0]), int(starts[i, 0]) + int(counts[i, 0])):
                yz = points[k, 1:] - (0., F.TOP)
                for e in EDGES:
                    if np.linalg.norm(yz - EDGE_YZ[e]) <= L.EDGE_ZONE:
                        bare[e] += float(forces[k])
        on_guard = {}
        for e in EDGES:
            name = seated.get(e)
            if name is None:
                on_guard[e] = 0.
            else:
                on_guard[e] = float(np.linalg.norm(array(self.web_guard[name].get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum())
        return bare, on_guard

    def arm_forces(self):
        mat = np.linalg.norm(array(self.arm_contacts.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        n = self.arm_link_count
        return {"left": float(mat[:n].sum()), "right": float(mat[n:].sum())}

    def tool_points(self, joints):
        return {arm: ik.tcp_pose(joints[i:i + 6], BASES[arm])[0] for arm, i in zip(F.ARMS, (0, 7))}

    def measure(self, joints):
        O, Rh = self.hook_frame()
        fits = self.fittings()
        apex = {n: tuple(float(v) for v in (Rh.T @ (f["apex"] - O))[1:]) for n, f in fits.items()}
        apex_x = {n: float((Rh.T @ (f["apex"] - O))[0]) for n, f in fits.items()}    # off the hook's plane
        tcps = self.tool_points(joints)
        forces = self.arm_forces()
        near = min(min(np.linalg.norm(t - O), min(np.linalg.norm(t - f["apex"]) for f in fits.values()))
                   for t in tcps.values())
        clear = max(forces.values()) < .2 and near > L.CLEAR_DIST
        web = self.s_poses[[self.sling_links.index(n) for n in F.LINKS], :3] - (0., 0., F.TOP)
        under = bool(np.any((np.abs(web[:, 0] - L.XS) < .06) & (np.abs(web[:, 1]) < L.HALF - .005) & (web[:, 2] < L.Z_BOT)))
        guards = {n: (tuple(float(v) for v in self.g_poses[r, :3] - (0., 0., F.TOP)), tuple(float(v) for v in self.g_poses[r, 3:]))
                  for n, r in self.grow.items()}
        seated = {}
        for n, (p, q) in guards.items():
            edge, _ = grading.guard_seat(p, q)
            if edge is not None:
                seated[edge] = n
        testing = self.grade is not None and self.grade.test in ("lifting", "holding")
        bare, on_guard = self.edge_contacts(seated) if testing else (dict.fromkeys(EDGES, 0.), dict.fromkeys(EDGES, 0.))
        return {"guards": guards, "apex": apex, "apex_x": apex_x, "latch_deg": self.latch_deg(), "hands_clear": clear, "under_bar": under,
                "pull_n": self.pull(), "bare_n": bare, "guard_n": on_guard}, {"tcps": tcps, "arm_n": forces, "near_m": near,
                                                                             "hook_O": O, "hook_R": Rh, "fits": fits}

    # -- the operator's hoist -----------------------------------------------------------------------------------------
    def _drive_hoist(self):
        cmd = self.grade.command if self.grade is not None else "rest"
        lift = self.rig_dofs.index("lift")
        if cmd == "lift":
            self.hoist_target += L.LIFT_RATE / F.CONTROL_HZ
        elif cmd == "hold":
            self.hoist_target += float(np.clip(L.PULL_KI * (L.PULL_N - self.last_pull), -.02, .02)) / F.CONTROL_HZ
        self.hoist_target = float(np.clip(self.hoist_target, -.05, .38))
        targets = self.rig.get_dof_position_targets().clone()
        targets[0, lift] = self.hoist_target
        self.rig.set_dof_position_targets(targets, self.one)

    # -- episode ----------------------------------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        if self.start is None:              # the scene file's start state, kept for every later reset
            self.start = {"guards": self.guards.get_transforms().clone(), "sling_root": self.sling.get_root_transforms().clone(),
                          "sling_q": self.sling.get_dof_positions().clone(), "rig_targets": self.rig.get_dof_position_targets().clone()}
        self.coms = array(self.guards.get_coms())[:, :3]
        self.guards.set_transforms(self.start["guards"].clone(), self.gids)
        self.guards.set_velocities(torch.zeros_like(self.guards.get_velocities()), self.gids)
        self.sling.set_root_transforms(self.start["sling_root"].clone(), self.one)
        self.sling.set_root_velocities(torch.zeros_like(self.sling.get_root_velocities()), self.one)
        bends = self.start["sling_q"].clone()
        for i, a in enumerate(F.sling_start_bends()):        # the folded-back fittings and the raised end links
            bends[0, self.sling_dofs.index(f"hinge_{i:02d}")] = a
        self.sling.set_dof_positions(bends, self.one)
        self.sling.set_dof_velocities(torch.zeros_like(self.sling.get_dof_velocities()), self.one)
        zero = torch.zeros_like(self.rig.get_dof_positions())
        self.rig.set_dof_positions(zero, self.one)
        self.rig.set_dof_velocities(zero, self.one)
        self.rig.set_dof_position_targets(self.start["rig_targets"].clone(), self.one)   # keeps the latch preloads
        self.hoist_target = L.HOIST_PRELOAD     # holds the hook at its rest height against its weight
        self.last_pull = 0.
        self.grade = None
        self.magnets_on = True
        observation = super().reset(scene, seed=seed)
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        m, extra = self.measure(joints)
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        masses = array(self.guards.get_masses()).reshape(-1)
        self.masses = {n: round(float(masses[r]), 4) for n, r in self.grow.items()}
        if any(abs(v - L.GUARD_MASS) > 2e-3 for v in self.masses.values()):
            raise RuntimeError(f"Guard masses differ from the scene file: {self.masses}")
        if not m["under_bar"]:
            raise RuntimeError("The sling does not lie under the bar after reset")
        if any(abs(a) > 1. for a in m["latch_deg"].values()):
            raise RuntimeError(f"A latch is not closed after reset: {m['latch_deg']}")
        self.state_after_reset = {"guards_mm": {n: [round(1000 * v, 1) for v in p] for n, (p, _) in m["guards"].items()},
                                  "apex_hook_frame_mm": {n: [round(1000 * v, 1) for v in yz] for n, yz in m["apex"].items()},
                                  "latch_deg": {k: round(v, 3) for k, v in m["latch_deg"].items()},
                                  "hook_origin_mm": [round(1000 * v, 1) for v in extra["hook_O"]], "masses": self.masses}
        self.grade = grading.RiggingGrade()
        self.tick = 0
        self.peaks = {}
        self.final_grade = None
        self.diagnostics = []
        self._publish_demo_state(m, extra)
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self, m, extra):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"guards": m["guards"], "fits": {n: {k: np.asarray(v).copy() for k, v in f.items()}
                                                            for n, f in extra["fits"].items()},
                           "hook_O": extra["hook_O"].copy(), "hook_R": extra["hook_R"].copy(),
                           "latch_deg": m["latch_deg"], "magnet_gap_mm": {k: 1000 * v for k, v in self.magnet_gap.items()},
                           "grade": self.final_grade}

    def step(self, action):
        t0 = time.perf_counter()
        self._drive_hoist()
        self.check_magnets_near()
        t1 = time.perf_counter()
        observation = super().step(action).observation
        t2 = time.perf_counter()
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.tick += 1
        m, extra = self.measure(joints)
        t3 = time.perf_counter()
        for key, dt in (("hoist_and_magnet_check", t1 - t0), ("simulate_and_render", t2 - t1), ("measure", t3 - t2)):
            self.timing[key] = self.timing.get(key, 0.) + dt
        self.last_pull = m["pull_n"]
        grade = self.grade.update(m)
        self.final_grade = grade
        self._publish_demo_state(m, extra)
        for arm, v in extra["arm_n"].items():
            key = f"{arm}_arm_on_task_bodies_n"
            self.peaks[key] = round(max(self.peaks.get(key, 0.), v), 2)
        if self.tick % 5 == 0 or grade["test"] in ("lifting", "holding"):
            self.diagnostics.append(self._diagnose(joints, m, extra, grade))
        over = self.grade.finished
        reason = ("success" if grade["score"] == 1. else "test_done") if over else None
        if not over and self.stop_check is not None:
            reason = self.stop_check()   # the Dream trial's last request is used up: it ends here, scored as it stands
            over = reason is not None
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == F.MAX_STEPS)
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"rigging_grade": grade})

    def _diagnose(self, joints, m, extra, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        return {"tick": self.tick,
                "tcp_mm": {a: [round(1000 * v, 1) for v in p] for a, p in extra["tcps"].items()},
                "pads_mm": [round(float(joints[6]) * PAD_SPAN_MM, 1), round(float(joints[13]) * PAD_SPAN_MM, 1)],
                "apex_hook_frame_mm": {n: [round(1000 * v, 1) for v in yz] for n, yz in m["apex"].items()},
                "apex_off_plane_mm": {n: round(1000 * v, 1) for n, v in m["apex_x"].items()},
                "latch_deg": {k: round(v, 2) for k, v in m["latch_deg"].items()},
                "guards": grade["guards"], "magnet_gap_mm": {k: round(1000 * v, 2) for k, v in self.magnet_gap.items()},
                "hook_origin_mm": [round(1000 * v, 1) for v in extra["hook_O"]], "hoist_mm": round(1000 * self.hoist_target, 2),
                "pull_n": round(m["pull_n"], 2), "bare_n": {e: round(v, 3) for e, v in m["bare_n"].items()},
                "guard_n": {e: round(v, 3) for e, v in m["guard_n"].items()}, "arm_n": {a: round(v, 2) for a, v in extra["arm_n"].items()},
                "near_mm": round(1000 * extra["near_m"], 1), "under_bar": m["under_bar"],
                "score": grade["score"], "why": grade["why"], "test": grade["test"]}

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class RiggingScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import RiggingReference

        self.reference = RiggingReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (guard, fitting and hook
        # poses); it commands the robot and moves nothing in the scene itself.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        t0 = time.perf_counter()
        action = next(self.actions)
        if CURRENT is not None:
            CURRENT.timing["policy"] = CURRENT.timing.get("policy", 0.) + time.perf_counter() - t0
        return action


def make_embodiment(**kwargs):
    return RiggingArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = RiggingScripted()
    policy.info = replace(YamScriptedPolicy().info, name="sling_guards_scripted")
    return policy
