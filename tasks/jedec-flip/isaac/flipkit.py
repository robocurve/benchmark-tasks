"""flipkit: the JEDEC tray-flip bench for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the ESD riser block (kinematic) the loaded tray sits on, so both tray ends overhang it;
* tray A (loaded) and tray B (empty): free rigid bodies, each made of ~45 convex pieces (walls with the pin-1 chamfer,
  stacking lip and rebate, floor web with two end handling slots, pocket walls on top, cell walls underneath);
* 21 packages (35 x 35 mm lidded FCBGA): free rigid bodies of three pieces each (ball field, substrate, lid).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
control step the scorer reads the tray and package poses and the finger pad tips; after every physics step the finger
contact forces on the trays are read for the grip log. Nothing here writes a pose or velocity during a trial;
``Runtime.restore_authored`` puts trays and packages back only on an environment reset. The scripted demo READS poses
through ``bench_state()``.
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import layout as L  # noqa: E402
from scoring import measure, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("JFLIP_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("JFLIP_CONTACT_OFFSET", ".0003"))  # below the 0.4 mm rib gaps
SETUP_BASE = int(os.environ.get("JFLIP_SETUP", "0"))
FORCE_NAMED = "JFLIP_SETUP" in os.environ
DEBUG = bool(os.environ.get("JFLIP_DEBUG"))
DEV_PRESTACK = bool(os.environ.get("JFLIP_DEV_PRESTACK"))

N_CHIPS = 21
CHIPS = tuple(f"chip_{i:02d}" for i in range(N_CHIPS))
FREE = ("trayA", "trayB") + CHIPS
SUB_GREEN, BALL_GREY, LID_NICKEL, LABEL_WHITE = (  # ball side reads dark, lid side bright (inspection cue)
    (0.12, 0.30, 0.18),
    (0.26, 0.27, 0.28),
    (0.86, 0.86, 0.84),
    (0.93, 0.93, 0.90),
)


class TrayBench(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quat(r):
    from scipy.spatial.transform import Rotation

    q = Rotation.from_matrix(np.asarray(r, float)).as_quat()  # xyzw
    return q / np.linalg.norm(q)


def authored(setup):
    """Reset pose of every free body for a setup: {name: (pos, rot)} in the env frame."""
    s = L.start_poses(setup)
    out = {"trayA": s["A"], "trayB": s["B"]}
    if DEV_PRESTACK:  # development probe only: B starts stacked on A
        pa, ra = s["A"]
        out["trayB"] = (pa + ra @ np.array([0.0, 0.0, L.STACK_DZ + 0.0001]), ra.copy())
    for n, pr in zip(CHIPS, s["chips"]):
        out[n] = pr
    return out


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from scipy.spatial import ConvexHull
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass=None, kinematic=False, inertia=None, com=None):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass if mass is not None else 1.0))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction):
        v = np.asarray(v, float)
        h = ConvexHull(v)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        m.CreateFaceVertexCountsAttr([3] * len(h.simplices))
        faces = []
        c = v.mean(axis=0)
        for s in h.simplices:
            a, b, d = v[s[0]], v[s[1]], v[s[2]]
            n = np.cross(b - a, d - a)
            faces.append(s if n @ (a - c) > 0 else s[::-1])
        m.CreateFaceVertexIndicesAttr([int(j) for f in faces for j in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))])
        prim = m.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        hc.CreateHullVertexLimitAttr(64)
        hc.CreateMinThicknessAttr(0.0001)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        count["n"] += 1
        return m

    a0 = authored(SETUP_BASE)
    # ---------------------------------------------------------------- riser (kinematic)
    rp = L.RISER + np.array([0.0, 0.0, L.RISER_SIZE[2] / 2])
    xform(f"{prim_path}/riser", rp, np.eye(3))
    rigid(f"{prim_path}/riser", 3.0, kinematic=True)
    hx, hy, hz = (s / 2 for s in L.RISER_SIZE)
    hull(f"{prim_path}/riser/block", L.box_v(-hx, hx, -hy, hy, -hz, hz), L.RISER_COLOR, L.TRAY_FRICTION)
    # ---------------------------------------------------------------- trays
    pieces = L.tray_pieces()
    for name in ("trayA", "trayB"):
        xform(f"{prim_path}/{name}", *a0[name])
        rigid(f"{prim_path}/{name}", L.TRAY_MASS, inertia=L.tray_inertia(), com=L.TRAY_COM)
        for pn, v in pieces:
            hull(f"{prim_path}/{name}/{pn}", v, L.TRAY_COLOR, L.TRAY_FRICTION)
        # pin-1 marker (visual only): a white triangle on the wall top next to the chamfer
        tri = UsdGeom.Mesh.Define(stage, f"{prim_path}/{name}/pin1_mark")
        x0, y0, z = L.TRAY_L / 2 - 0.030, L.TRAY_W / 2 - 0.0035, L.TRAY_H + 0.0002
        tri.CreatePointsAttr([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x0 + 0.014, y0, z), Gf.Vec3f(x0 + 0.007, y0 - 0.0001, z)])
        tri.CreateFaceVertexCountsAttr([3])
        tri.CreateFaceVertexIndicesAttr([0, 1, 2])
        bind_visual_material(str(tri.GetPrim().GetPath()), look(LABEL_WHITE), stage=stage)
    # ---------------------------------------------------------------- packages
    colors = {"balls": BALL_GREY, "sub": SUB_GREEN, "lid": LID_NICKEL}
    for name in CHIPS:
        xform(f"{prim_path}/{name}", *a0[name])
        rigid(f"{prim_path}/{name}", L.CHIP_MASS, inertia=L.chip_inertia(), com=L.CHIP_COM)
        for pn, v in L.chip_pieces():
            hull(f"{prim_path}/{name}/{pn}", v, colors[pn], L.CHIP_FRICTION)
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in FREE + ("riser",)}
    _SPEC["colliders"] = count["n"]
    return root.GetPrim()


def _arena_object(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BenchSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bench)

    return Object(
        name=spec.name,
        object_type=ObjectType.BASE,
        spawner_cfg=BenchSpawnerCfg(),
        initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None)


def finger_tips(environment):
    """Pad-tip centres of all four fingers (env frame)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[list(robot.body_names).index("gripper")]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Runtime:
    """Reads the finger contact forces every physics step, scores every control step."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/bench"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in FREE])
        fingers = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            links = list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
            fingers += [p for p in links if p.endswith("tip_left") or p.endswith("tip_right")]
        self.fingers = fingers
        self.cv_grip = self.simview.create_rigid_contact_view(
            [P("trayA"), P("trayB")], filter_patterns=[fingers, fingers], max_contact_data_count=4096
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.grip_now = [0.0] * 4
        self.grip_peak = [0.0] * 4
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        poses = {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(FREE)}
        return dict(A=poses["trayA"], B=poses["trayB"], chips=[poses[n] for n in CHIPS])

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return v[:, :3], v[:, 3:6]

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        if self.trial_active:
            f = self.cv_grip.get_contact_force_matrix(dt=dt).detach().cpu().numpy()  # (2 trays, 4 fingers, 3)
            per = np.linalg.norm(f, axis=-1).sum(axis=0) if f.size else np.zeros(4)
            self.grip_now = [float(x) for x in per]
            self.grip_peak = [max(a, b) for a, b in zip(self.grip_peak, self.grip_now)]
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): trays and packages back to the setup's start, at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(FREE):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.arange(len(FREE), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        lin, ang = self.velocities()
        if not (np.all(np.isfinite(lin)) and np.all(np.isfinite(ang))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        m = measure(poses, tips)
        level = self.scorer.step(
            m, float(np.linalg.norm(lin, axis=1).max()), math.degrees(float(np.linalg.norm(ang, axis=1).max()))
        )
        STATE.update(
            score=level,
            level=self.scorer.level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                grip_n=[round(x, 2) for x in self.grip_now],
                grip_peak_n=[round(x, 2) for x in self.grip_peak],
                a_pos_mm=np.round(poses["A"][0] * 1000, 2).tolist(),
                b_pos_mm=np.round(poses["B"][0] * 1000, 2).tolist(),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'A', 'B', 'chips'} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def _patch_scene(scene) -> None:
    if getattr(scene, "_jflip_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, TrayBench):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "bench"]
        out = orig_states(environment, plain) if plain else {}
        if "bench" in names:
            rt = _RT["rt"]
            p, r = rt.read()["A"] if rt else L.start_poses(SETUP_BASE)["A"]
            z = np.zeros(3)
            out["bench"] = ObjectState(
                pos=np.asarray(p),
                quat_xyzw=_quat(r),
                lin_vel=z,
                ang_vel=z,
                low=p - 0.2,
                high=p + 0.2,
            )
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, TrayBench) for o in task.objects):
            previous = environment.env_cfg_callback

            def configure(cfg):
                from yam_arena.constants import CONTROL_HZ

                cfg = previous(cfg)
                cfg.sim.dt = PHYSICS_DT
                cfg.decimation = round(1.0 / (PHYSICS_DT * (control_hz or CONTROL_HZ)))
                cfg.sim.render_interval = cfg.decimation
                return cfg

            environment.env_cfg_callback = configure
        return environment

    scene.build_task_environment = build
    scene.task_object = task_object
    scene.object_states = object_states
    scene._jflip_patched = True


def _has_bench(emb):
    return any(
        isinstance(o, TrayBench)
        for o in (emb.task.objects if emb.task is not None else ())
    )


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_jflip_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_bench(self):
            _wrap(self._env)
            _RT["rt"] = Runtime(self._env)
        return out

    def _task_step(self, *a, **k):
        if _RT["rt"] is not None:
            _RT["rt"].control(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        rt = _RT["rt"]
        seed = k.get("seed")
        if seed is not None and not FORCE_NAMED:
            setup = ("seed", int(seed))
        else:
            setup = (SETUP_BASE + _RT["trials"]) % len(L.SETUPS)
        if rt is not None:
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup:  # first reset of a fresh env
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._jflip_patched = True


def _wrap(environment) -> None:
    base = environment.unwrapped
    sim = base.sim
    if _RT["sim"] is sim:
        return
    orig = sim.step

    def step(*a, **k):
        out = orig(*a, **k)
        if _RT["rt"] is not None:
            _RT["rt"].physics(base.physics_dt)
        return out

    sim.step = step
    _RT["sim"] = sim


def install() -> None:
    import importlib.abc
    import importlib.util

    class _Finder(importlib.abc.MetaPathFinder):
        targets = {
            "yam_arena.scene": _patch_scene,
            "yam_arena.inspect_embodiment": _patch_embodiment,
        }

        def find_spec(self, fullname, path, target=None):
            if fullname not in self.targets:
                return None
            sys.meta_path.remove(self)
            try:
                spec = importlib.util.find_spec(fullname)
            finally:
                sys.meta_path.insert(0, self)
            if spec is None or spec.loader is None:
                return None
            loader, patch = spec.loader, self.targets[fullname]
            orig_exec = loader.exec_module

            def exec_module(module):
                orig_exec(module)
                patch(module)

            loader.exec_module = exec_module
            return spec

    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(
        type(f).__name__ == "_Finder"
        and getattr(f, "targets", None)
        and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
