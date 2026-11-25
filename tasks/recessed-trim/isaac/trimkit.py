"""trimkit: the ceiling section with its recessed-light housing, the torsion-spring baffle trim and its carton, for
yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the ceiling (1/2 in. drywall panel with its 6-3/4 in. cut-out, on two posts and a header) as a kinematic body;
* the housing (can wall, can top, lamp socket and BR30 lamp, two torsion-spring brackets with slotted tabs) as one
  kinematic body, turned about its axis by the trial's bracket angle phi at reset;
* the trim's carton (kinematic);
* the trim (flange + baffle cone + the two spring coils) as a free rigid body and the four spring legs as free rigid
  bodies on revolute joints at the coils (hard joint limits well outside the working range).
Each leg's spring torque k (theta_free - theta) - c omega (plus a stiff stop where the squeezed legs meet) is computed
from the measured joint angle every physics step
and applied to the leg and, equal and opposite, to the trim (drives on these maximal joints do not act in this PhysX
build, so the spring is a computed torque). Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py``.

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
physics step the runtime applies the spring torques and reads the robot's contact force on the trim and legs; after
every control step the scorer reads the poses. Nothing here writes a pose or a velocity of the trim or its legs during
a trial; ``Runtime.restore_authored`` puts them back on the carton only on an environment reset. The scripted demo
READS poses through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("TRIM_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("TRIM_CONTACT_OFFSET", "0.0004"))
SETUP_BASE = int(os.environ.get("TRIM_SETUP", "0"))  # named setup of the first unseeded trial; later trials cycle
# Trials reset WITH a seed (every inspect-robots rollout passes one) draw phi, the trim's yaw and the carton offset from
# the seed (layout.setup_spec); TRIM_SETUP set in the environment forces the named setups instead.
FORCE_NAMED = "TRIM_SETUP" in os.environ
DEBUG = bool(os.environ.get("TRIM_DEBUG"))

DRYWALL, WOOD, STEEL, WHITE, LAMP, CARDBOARD, WIRE, BRASS = (
    (0.90, 0.89, 0.86),
    (0.72, 0.58, 0.40),
    (0.60, 0.62, 0.64),
    (0.97, 0.97, 0.96),
    (0.95, 0.93, 0.85),
    (0.70, 0.56, 0.38),
    (0.25, 0.26, 0.28),
    (0.75, 0.62, 0.30),
)
WORK_LIGHT_POS = (0.10, 0.0, 0.95)  # station frame (the rig root is at the origin)
WORK_LIGHT_R = 0.03
WORK_LIGHT_INTENSITY = float(os.environ.get("TRIM_WORK_LIGHT", "40000"))
KINEMATIC = ("ceiling", "housing", "carton")
DYNAMIC = ("trim",) + tuple(f"leg{n}" for n in L.LEGS)
BODIES = KINEMATIC + DYNAMIC


class TrimRig(Box):
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
    """Authored (reset) pose of the moving bodies for a setup: {name: (pos, rot)} in the env frame."""
    sp = L.setup_spec(setup)
    tp, tr = L.trim_start_pose(setup)
    out = {
        "trim": (tp, tr),
        "housing": (L.H.copy(), L.rz(sp["phi"])),
        "carton": (L.carton_pos(setup), np.eye(3)),
        "ceiling": (L.H.copy(), np.eye(3)),
    }
    for leg in L.LEGS:
        out[f"leg{leg}"] = (tp + tr @ L.leg_root(leg), tr @ L.leg_rot(leg, L.THETA_FREE))
    return out


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
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

    def hull(path, v, color, friction=L.TRIM_FRICTION, collide=True):
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
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(CONTACT_OFFSET)
            cp.CreateRestOffsetAttr(0.0)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(64)
            hc.CreateMinThicknessAttr(0.0001)
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def box(path, center, size, color, friction=L.TRIM_FRICTION, rot=None, collide=True):
        hx, hy, hz = (s / 2 for s in size)
        corners = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        r = np.eye(3) if rot is None else np.asarray(rot, float)
        return hull(path, corners @ r.T + np.asarray(center, float), color, friction, collide)

    # ---------------------------------------------------------------- ceiling (kinematic, housing frame, no rotation)
    cp = f"{prim_path}/ceiling"
    xform(cp, L.H, np.eye(3))
    rigid(cp, 20.0, kinematic=True)
    for i, v in enumerate(L.ceiling_pieces()):
        hull(f"{cp}/gyp_{i:03d}", v, DRYWALL, friction=0.5)
    # two 2x4 posts at the far corners and a header under the panel's far edge (cosmetic support, colliders)
    x1 = L.CEIL_X[1] - L.H[0]
    z_floor = L.TOP - L.H[2]
    for k, y in enumerate((L.CEIL_Y[0] + 0.03, L.CEIL_Y[1] - 0.03)):
        box(f"{cp}/post_{k}", (x1 - 0.03, y - L.H[1], (z_floor + 0.0) / 2), (0.038, 0.089, -z_floor), WOOD)
    box(f"{cp}/header", (x1 - 0.03, 0.0, -0.045), (0.038, L.CEIL_Y[1] - L.CEIL_Y[0], 0.089), WOOD)
    # joists above the drywall (2x6 on edge, cosmetic + collider), clear of the housing
    # the bay's two joists along the panel's long edges (2x6 on edge, on top of the drywall)
    for k, y in enumerate((L.CEIL_Y[0] + 0.019, L.CEIL_Y[1] - 0.019)):
        box(f"{cp}/joist_{k}", ((L.CEIL_X[0] + L.CEIL_X[1]) / 2 - L.H[0], y - L.H[1], L.DRYWALL_T + 0.070),
            (L.CEIL_X[1] - L.CEIL_X[0], 0.038, 0.140), WOOD)
    # ---------------------------------------------------------------- housing (kinematic, turned by phi at reset)
    hp = f"{prim_path}/housing"
    xform(hp, *authored(SETUP_BASE)["housing"])
    rigid(hp, 2.0, kinematic=True)
    for i, c in enumerate(L.annulus_cells(L.CAN_R[0], L.CAN_R[1], 48)):
        hull(f"{hp}/can_{i:02d}", L.prism(c, L.CAN_Z[0], L.CAN_Z[1]), STEEL, friction=0.4)
    hull(f"{hp}/can_top", L.cylinder(L.CAN_R[1], L.CAN_Z[1], L.CAN_Z[1] + 0.0015, n=48), STEEL)
    # aperture ring (frame flange on the drywall's top face) - cosmetic
    for i, c in enumerate(L.annulus_cells(L.CAN_R[1], 0.110, 24)):
        hull(f"{hp}/frame_{i:02d}", L.prism(c, L.DRYWALL_T, L.DRYWALL_T + 0.001), STEEL, collide=False)
    hull(f"{hp}/socket", L.cylinder(0.020, L.LAMP_Z[1], L.CAN_Z[1], n=24), WHITE)
    # BR30: neck + bulged body + face (three hulls)
    hull(f"{hp}/lamp_neck", L.cylinder(0.016, L.LAMP_Z[1] - 0.030, L.LAMP_Z[1], n=24), WHITE)
    body = np.r_[L.cylinder(0.016, L.LAMP_Z[1] - 0.030, L.LAMP_Z[1] - 0.030, n=24)[:24],
                 L.cylinder(L.LAMP_R, L.LAMP_Z[0] + 0.012, L.LAMP_Z[0] + 0.012, n=32)[:32]]
    hull(f"{hp}/lamp_body", body, WHITE)
    hull(f"{hp}/lamp_face", L.cylinder(L.LAMP_R - 0.002, L.LAMP_Z[0], L.LAMP_Z[0] + 0.012, n=32), LAMP)
    for name, beta in (("A", 90.0), ("B", -90.0)):  # bracket angles in the housing frame (phi turns the body)
        for i, (c, s, yaw) in enumerate(L.tab_boxes(beta)):
            box(f"{hp}/bracket{name}_{i}", c, s, STEEL, friction=L.WIRE_FRICTION, rot=L.rz(yaw))
    # ---------------------------------------------------------------- carton (kinematic)
    kp = f"{prim_path}/carton"
    a0 = authored(SETUP_BASE)
    xform(kp, *a0["carton"])
    rigid(kp, 0.3, kinematic=True)
    cs = L.CARTON_SIZE
    box(f"{kp}/box", (0.0, 0.0, cs[2] / 2), cs, CARDBOARD, friction=0.5)
    # ---------------------------------------------------------------- trim (free body) and its four spring legs
    tp = f"{prim_path}/trim"
    xform(tp, *a0["trim"])
    m_trim = L.TRIM_MASS - 4 * L.LEG_MASS
    r_avg = 0.08
    rigid(tp, m_trim, inertia=(0.6 * m_trim * r_avg**2, 0.6 * m_trim * r_avg**2, 0.9 * m_trim * r_avg**2),
          com=(0.0, 0.0, 0.020))
    for i, v in enumerate(L.flange_pieces()):
        hull(f"{tp}/flange_{i:02d}", v, WHITE)
    for i, v in enumerate(L.baffle_pieces()):
        hull(f"{tp}/baffle_{i:02d}", v, WHITE)
    for s in ("A", "B"):
        c, rad, tan = L.spring_frame(s)
        r_wall = L.baffle_r_at(L.COIL_Z)
        rr = np.column_stack([rad, tan, [0.0, 0.0, 1.0]])
        # mounting clip from the baffle wall out to the coil, and the coil (7 mm diameter, 5 mm long, radial axis)
        box(f"{tp}/clip{s}", rad * (r_wall + L.LEG_R - 0.0035) / 2 + np.array([0, 0, L.COIL_Z]),
            (L.LEG_R - 0.0035 - r_wall + 0.001, 0.004, 0.006), STEEL, rot=rr)
        box(f"{tp}/coil{s}", c, (0.005, 2 * L.ROOT_U - 0.001, 0.007), WIRE, rot=rr)
    jpaths = {}
    for leg in L.LEGS:
        lp = f"{prim_path}/leg{leg}"
        xform(lp, *a0[f"leg{leg}"])
        rigid(lp, L.LEG_MASS, inertia=(L.LEG_MASS * L.LEG_L**2 / 3, L.LEG_MASS * L.LEG_L**2 / 3, 2e-9),
              com=(0.0, 0.0, L.LEG_L / 2))
        w = L.WIRE
        box(f"{lp}/wire", (0.0, 0.0, L.LEG_L / 2), (w, w, L.LEG_L), WIRE, friction=L.WIRE_FRICTION)
        hull(f"{lp}/hook", L.hook_hull(leg), WIRE, friction=L.WIRE_FRICTION)
        # revolute joint at the root: frame0 in the trim = leg frame at theta 0; axis X (radial)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{lp}/pivot")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(tp)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(lp)])
        q0 = _quat(L.leg_rot(leg, 0.0))
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, L.leg_root(leg))))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]), *map(float, q0[:3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        sg = L.leg_axis_sign(leg)
        lo, hi = sorted((sg * math.degrees(L.THETA_HARD), sg * math.degrees(L.THETA_MAX)))
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(lp)).CreateFilteredPairsRel().AddTarget(tp)
        jpaths[leg] = str(j.GetPath())
    # the two legs of one spring share the coil: no contact between them
    for s in ("A", "B"):
        UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath(f"{prim_path}/leg{s}p")).CreateFilteredPairsRel().AddTarget(
            f"{prim_path}/leg{s}m")
    # work light on the bench under the ceiling (cosmetic, no collider): the station's lights are above the ceiling,
    # so without it the trim's underside and the housing are in deep shadow on every camera
    from pxr import UsdLux

    lt = UsdLux.SphereLight.Define(stage, f"{prim_path}/work_light")
    lt.CreateRadiusAttr(WORK_LIGHT_R)
    lt.CreateIntensityAttr(WORK_LIGHT_INTENSITY)
    lt.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
    UsdGeom.Xformable(lt.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*WORK_LIGHT_POS))
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in BODIES}
    _SPEC["joints"] = jpaths
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
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[
            list(robot.body_names).index("gripper")]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Runtime:
    """Applies the leg springs and reads the robot contact every physics step, scores every control step."""

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
        root = "/World/envs/env_0/rig"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in DYNAMIC])
        self.kview = self.simview.create_rigid_body_view([P(n) for n in KINEMATIC])
        self.index = {n: i for i, n in enumerate(DYNAMIC)}
        self.idx = torch.arange(len(DYNAMIC), dtype=torch.int32, device=self.view.get_transforms().device)
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_robot = self.simview.create_rigid_contact_view(
            [P(n) for n in DYNAMIC], filter_patterns=[robot_links] * len(DYNAMIC), max_contact_data_count=8192)
        self.cv_env = self.simview.create_rigid_contact_view(
            [P("trim")], filter_patterns=[[P("ceiling"), P("housing"), P("carton")]], max_contact_data_count=4096)
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        dev = self.view.get_transforms().device
        n = len(DYNAMIC)
        self.f = torch.zeros((n, 3), dtype=torch.float32, device=dev)
        self.tq = torch.zeros((n, 3), dtype=torch.float32, device=dev)
        self.pp = torch.zeros((n, 3), dtype=torch.float32, device=dev)
        self.setup = SETUP_BASE
        self.phi = L.setup_spec(self.setup)["phi"]
        self.trial_active = False
        self.extra = {}  # development probes only: body -> (force, at point, torque) world; empty in every trial
        self.probe = None  # development probes only: callable(rt, poses, vel) run before the forces are applied
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.steps = 0
        self.substep = 0
        self.t = 0.0
        self.robot_n = 0.0  # max robot contact force on trim + legs since the last control step
        self.robot_peak = 0.0
        self.theta = {leg: L.THETA_FREE for leg in L.LEGS}
        self.theta_min = {leg: L.THETA_FREE for leg in L.LEGS}
        self.squeeze_peak_nm = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        out = {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.index.items()}
        kt = self.kview.get_transforms().detach().cpu().numpy().astype(float)
        for i, n in enumerate(KINEMATIC):
            out[n] = (kt[i, :3] - self.origin, _rot(kt[i, 3:7]))
        return out

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for n, i in self.index.items()}

    def legs(self, poses):
        return {leg: poses[f"leg{leg}"] for leg in L.LEGS}

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        self.substep += 1
        poses = self.read()
        vel = self.velocities()
        tp, tr = poses["trim"]
        w_trim = vel["trim"][1]
        f = np.zeros((len(DYNAMIC), 3))
        tq = np.zeros((len(DYNAMIC), 3))
        pp = np.zeros((len(DYNAMIC), 3))
        for n, i in self.index.items():
            pp[i] = poses[n][0] + self.origin
        it = self.index["trim"]
        for leg in L.LEGS:
            lp, lr = poses[f"leg{leg}"]
            th = L.leg_theta(tr, lr, leg)
            sg = L.leg_axis_sign(leg)
            axis = lr[:, 0]  # radial pivot axis (leg frame x), world
            om = sg * float((vel[f"leg{leg}"][1] - w_trim) @ axis)
            tau = L.spring_torque(th, om)
            il = self.index[f"leg{leg}"]
            tq[il] += sg * tau * axis
            tq[it] -= sg * tau * axis
            self.theta[leg] = th
            self.theta_min[leg] = min(self.theta_min[leg], th)
            self.squeeze_peak_nm = max(self.squeeze_peak_nm, tau)
        if self.probe is not None:  # development probes only
            self.probe(self, poses, vel)
        for n, (fw, at, tw) in self.extra.items():  # development probes only
            i = self.index[n]
            f[i] += fw
            tq[i] += tw
            pp[i] = np.asarray(at) + self.origin
        torch = self.torch
        self.f.copy_(torch.as_tensor(f, dtype=torch.float32))
        self.tq.copy_(torch.as_tensor(tq, dtype=torch.float32))
        self.pp.copy_(torch.as_tensor(pp, dtype=torch.float32))
        self.view.apply_forces_and_torques_at_position(
            force_data=self.f, torque_data=self.tq, position_data=self.pp, indices=self.idx, is_global=True)
        if self.trial_active:
            fr = self.cv_robot.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            fn = float(np.linalg.norm(fr, axis=-1).max()) if fr.size else 0.0
            self.robot_n = max(self.robot_n, fn)
            self.robot_peak = max(self.robot_peak, fn)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): housing turned to the setup's phi, carton placed, trim and
        legs back on the carton at rest."""
        torch = self.torch
        self.setup = setup
        self.phi = L.setup_spec(setup)["phi"]
        self.trial_active = False
        a = authored(setup)
        kt = self.kview.get_transforms().clone()
        for i, n in enumerate(KINEMATIC):
            pos, rot = a[n]
            kt[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=kt.dtype)
            kt[i, 3:7] = torch.tensor(_quat(rot), dtype=kt.dtype)
        kidx = torch.arange(len(KINEMATIC), dtype=torch.int32, device=kt.device)
        try:
            self.kview.set_kinematic_targets(kt, kidx)
        except Exception:  # noqa: BLE001
            pass
        self.kview.set_transforms(kt, kidx)
        tf = self.view.get_transforms().clone()
        for n, i in self.index.items():
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        self.view.set_transforms(tf, self.idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), self.idx)
        self._init_state()
        hr = self.read()["housing"][1]
        print(f"TRIMKIT reset setup={setup} phi={self.phi:.3f} housing_yaw={math.degrees(math.atan2(hr[1, 0], hr[0, 0])):.3f}",
              flush=True)

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        vel = self.velocities()
        tp, tr = poses["trim"]
        lin, ang = vel["trim"]
        if not (np.all(np.isfinite(tp)) and np.all(np.isfinite(tr)) and np.all(np.isfinite(lin))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        m = measure(tp, tr, self.legs(poses), self.phi, self.robot_n, self.theta)
        level = self.scorer.step(m, float(np.linalg.norm(lin)), math.degrees(float(np.linalg.norm(ang))))
        fe = self.cv_env.get_contact_force_matrix(dt=PHYSICS_DT).detach().cpu().numpy()
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                robot_n=round(self.robot_n, 3),
                robot_peak_n=round(self.robot_peak, 2),
                squeeze_peak_nm=round(self.squeeze_peak_nm, 4),
                theta_min_deg={k: round(math.degrees(v), 2) for k, v in self.theta_min.items()},
                trim_pos_mm=np.round(tp * 1000, 2).tolist(),
                env_n=[round(float(np.linalg.norm(x)), 3) for x in fe[0]] if fe.size else None,
                setup=self.setup,
                phi=round(self.phi, 3),
                hook_wall_s=round(self.hook_s, 2),
            ),
        )
        self.robot_n = 0.0


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {body: (pos, rot)} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def _patch_scene(scene) -> None:
    if getattr(scene, "_trim_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, TrimRig):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "rig"]
        out = orig_states(environment, plain) if plain else {}
        if "rig" in names:
            rt = _RT["rt"]
            p, r = rt.read()["trim"] if rt else authored(SETUP_BASE)["trim"]
            z = np.zeros(3)
            out["rig"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.2,
                                     high=p + 0.2)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, TrimRig) for o in task.objects):
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
    scene._trim_patched = True


def _has_rig(emb):
    return any(isinstance(o, TrimRig) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_trim_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_rig(self):
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
        fresh = rt is None
        rt = _RT["rt"]
        if rt is not None:
            if fresh or rt.setup != setup:  # first reset of a fresh env
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._trim_patched = True


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
        targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

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
    if not any(type(f).__name__ == "_Finder" and getattr(f, "targets", None) and "yam_arena.scene" in f.targets
               for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
