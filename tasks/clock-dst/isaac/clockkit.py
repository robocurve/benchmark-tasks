"""clockkit: the wall section, the nail and the quartz wall clock (with its setting knob and hands) for yam_arena
(Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the wall section (a 1/2 in gypsum board on a bench stand) and the nail (6d common nail, 10 mm proud) as kinematic
  rigid bodies, so PhysX contact views can report the forces on them;
* the clock as a floating PhysX articulation: the case (root link: bezel ring, crystal, back shell with its keyhole
  hanger and knob opening, movement body; 0.50 kg) with two revolute child links on the hand axis: the setting knob,
  which carries the minute hand (they turn together 1:1, as on a quartz movement whose knob is on the minute arbor),
  and the hour hand. The knob joint has the movement's friction-clutch drag: an elasto-plastic friction anchor (stiff
  position drive whose force is capped at the breakaway torque, its target moved when it slips; same model as earlier
  verified kits). The hour-hand joint is a position drive that follows the knob at 1:12 (the motion work's gear ratio).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the runtime updates the
clutch anchor and the hour-hand target and reads the contact forces (nail on clock, robot on clock) and the clock's
vertical speed; after every control step the scorer reads the clock pose and the knob angle. Nothing here writes the
clock's pose, velocity or joint angles during a trial; ``Runtime.restore_authored`` puts the clock back on its nail
(and the nail at the setup's position) only on an environment reset. The scripted demo READS state through
``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("CLOCK_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("CLOCK_CONTACT_OFFSET", "0.0003"))  # below the 0.57 mm slot play a side
SETUP_BASE = int(os.environ.get("CLOCK_SETUP", "0"))
FORCE_NAMED = "CLOCK_SETUP" in os.environ
DEBUG = bool(os.environ.get("CLOCK_DEBUG"))

KNOB_STIFF = 2.0  # N m / rad: clutch anchor drive stiffness (elastic play 0.23 deg at the 8 mN m breakaway)
KNOB_DAMP = 2e-4
KNOB_ARMATURE = 2e-7  # kg m^2 felt at the knob joint (wheel train, stabilises the tiny link)
HOUR_STIFF = 0.5
HOUR_DAMP = 2e-4

BLACK, DARK, WHITE, STEEL, WALLC, RED, GREY = (
    (0.06, 0.06, 0.07),
    (0.16, 0.16, 0.18),
    (0.95, 0.95, 0.93),
    (0.62, 0.63, 0.66),
    (0.88, 0.86, 0.80),
    (0.80, 0.08, 0.06),
    (0.45, 0.45, 0.47),
)
KINEMATIC = ("wall", "nail")
LINKS = ("clock", "knob", "hourhand")


class ClockSet(Box):
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


# joint frame of the hand axis in K: joint X along -z_K (positive = clockwise seen from the front)
R_JOINT = np.column_stack([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])


def authored(setup):
    """Authored (reset) state for a setup: clock K pose, nail position, knob and hour joint positions."""
    sp = L.setup_spec(setup)
    cp, cr = L.clock_hung_pose(setup)
    q0 = L.q_start(sp["start_min"])
    return dict(
        clock=(cp, cr),
        nail=(L.nail_point(setup), np.eye(3)),
        knob_q=q0,
        hour_q=L.hour_angle(sp["start_min"]),
        start_min=sp["start_min"],
    )


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
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*(translation or (0.0, 0.0, 0.0)))
    )
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(ow, ox, oy, oz)
    )
    looks, mats = {}, {}

    def look(color, rough=0.5):
        key = (color, rough)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(color), roughness=rough
            )
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(
                static_friction=friction, dynamic_friction=friction
            )
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def gfq(r):
        q = _quat(r)
        return float(q[3]), float(q[0]), float(q[1]), float(q[2])

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*map(float, pos))
        )
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(*gfq(rot))
        )
        return x

    def rigid(path, mass, kinematic=False, inertia=None, com=None, iters=32):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
            m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.3)
        px.CreateSolverPositionIterationCountAttr(iters)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    count = {"n": 0}

    def hull(path, v, color, friction, visible=True, torsion=0.0):
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
        m.CreateExtentAttr(
            [Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))]
        )
        prim = m.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        if torsion:
            cp.CreateTorsionalPatchRadiusAttr(float(torsion))
            cp.CreateMinTorsionalPatchRadiusAttr(float(torsion))
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        hc.CreateHullVertexLimitAttr(255)
        hc.CreateMinThicknessAttr(0.0001)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        if visible:
            bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        else:
            UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
        count["n"] += 1
        return m

    def box_pts(center, size, rot=np.eye(3)):
        hx, hy, hz = (s / 2 for s in size)
        corners = np.array(
            [[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        )
        return corners @ np.asarray(rot).T + np.asarray(center, float)

    vis_n = {"n": 0}

    def vbox(parent, center, size, color, rot=np.eye(3), rough=0.5):
        vis_n["n"] += 1
        g = UsdGeom.Cube.Define(stage, f"{parent}/v{vis_n['n']}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        g.AddOrientOp().Set(Gf.Quatf(*gfq(rot)))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(str(g.GetPrim().GetPath()), look(color, rough), stage=stage)

    def vcyl(parent, center, radius, height, color, axis="Z", rough=0.5):
        vis_n["n"] += 1
        g = UsdGeom.Cylinder.Define(stage, f"{parent}/v{vis_n['n']}")
        g.CreateAxisAttr(axis)
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(height))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        bind_visual_material(str(g.GetPrim().GetPath()), look(color, rough), stage=stage)

    def cyl_pts(r, z0, z1, n=32, center=(0.0, 0.0)):
        return L.prism(L.circle_pts(r, n, center), z0, z1)

    A = authored(SETUP_BASE)
    # ---------------------------------------------------------------- wall section (kinematic)
    wpath = f"{prim_path}/wall"
    xform(wpath, (L.WALL_X, 0.0, L.TOP), np.eye(3))
    rigid(wpath, 5.0, kinematic=True)
    hull(
        f"{wpath}/board",
        box_pts((L.WALL_T / 2, 0.0, L.WALL_H / 2 + 0.001), (L.WALL_T, L.WALL_W, L.WALL_H)),
        WALLC,
        L.WALL_FRICTION,
    )
    for s in (-1.0, 1.0):  # stand feet behind the board
        hull(
            f"{wpath}/foot_{'l' if s > 0 else 'r'}",
            box_pts((L.WALL_T + 0.06, s * (L.WALL_W / 2 - 0.03), 0.0105), (0.12, 0.04, 0.02)),
            GREY,
            0.5,
        )
    # ---------------------------------------------------------------- nail (kinematic; frame at the wall-face point)
    npath = f"{prim_path}/nail"
    xform(npath, A["nail"][0], np.eye(3))
    rigid(npath, 0.01, kinematic=True)
    rx_ = L.ry(-math.pi / 2)  # cylinder axis z -> -x (out of the wall)
    shank = L.prism(L.circle_pts(L.NAIL_SHANK_R, 16), -0.004, L.NAIL_PROUD - L.NAIL_HEAD_T) @ rx_.T
    head = L.prism(L.circle_pts(L.NAIL_HEAD_R, 24), L.NAIL_PROUD - L.NAIL_HEAD_T, L.NAIL_PROUD) @ rx_.T
    hull(f"{npath}/shank", shank, STEEL, L.NAIL_FRICTION)
    hull(f"{npath}/head", head, STEEL, L.NAIL_FRICTION)
    # ---------------------------------------------------------------- clock case (articulation root, floating)
    cp, cr = A["clock"]
    cpath = f"{prim_path}/clock"
    xform(cpath, cp, cr)
    prim = rigid(
        cpath,
        L.CLOCK_MASS - L.KNOB_MASS - 0.001,
        inertia=L.clock_inertia(),
        com=(0.0, 0.0, L.CLOCK_COM_Z),
        iters=64,
    )
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(64)
    ar.CreateSolverVelocityIterationCountAttr(4)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    for i, cell in enumerate(L.annulus_cells(L.BEZEL_RI, L.CLOCK_R, 6.0)):
        hull(f"{cpath}/bezel_{i:02d}", L.prism(cell, *L.BEZEL_Z), BLACK, L.CASE_FRICTION, torsion=0.004)
    for i, cell in enumerate(L.annulus_cells(L.SHELL_R[0], L.SHELL_R[1], 6.0)):
        hull(f"{cpath}/shell_{i:02d}", L.prism(cell, *L.SHELL_Z), BLACK, L.CASE_FRICTION)
    for i, cell in enumerate(L.back_panel_cells()):
        hull(f"{cpath}/back_{i:03d}", L.prism(cell, 0.0, L.BACK_T), BLACK, L.CASE_FRICTION)
    # crystal (glass): collider, rendered faintly; dial plate behind it (visual)
    hull(f"{cpath}/crystal", cyl_pts(L.CRYSTAL_R, *L.CRYSTAL_Z, n=48), WHITE, L.CASE_FRICTION, visible=False)
    # dial support ring between the shell and the bezel (inside), so the fingers' back pad has a face behind the lip
    hull(f"{cpath}/movement", box_pts((0.0, 0.0, sum(L.MOVE_Z) / 2), L.MOVE_SIZE), DARK, 0.5)
    vcyl(cpath, (0.0, 0.0, sum(L.DIAL_Z) / 2), L.BEZEL_RI, L.DIAL_Z[1] - L.DIAL_Z[0], WHITE, rough=0.8)
    zt = L.DIAL_Z[1] + 0.0003
    for k in range(60):  # minute track and hour bars (printed on the dial)
        a = math.radians(6 * k)
        rr = L.ry(0.0) @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        big = k % 5 == 0
        ln = 0.011 if big else 0.004
        w = 0.0032 if big else 0.0011
        rad = 0.104 - ln / 2
        # tick along the radial direction at clockwise angle 6k from 12: K position (sin, cos)
        pos = (rad * math.sin(a), rad * math.cos(a), zt)
        rot = np.array([[math.cos(-a), -math.sin(-a), 0], [math.sin(-a), math.cos(-a), 0], [0, 0, 1]])
        vbox(cpath, pos, (w, ln, 0.0004), BLACK, rot)
        if k == 0:  # double bar at 12
            vbox(cpath, (0.0045, rad, zt), (w, ln, 0.0004), BLACK)
            vbox(cpath, (-0.0045, rad, zt), (w, ln, 0.0004), BLACK)
        del rr
    # stopped second hand (battery removed for the trial) and centre cap
    vbox(cpath, (0.0, 0.035, L.HAND_Z["second"]), (0.0008, 0.11, 0.0004), RED)
    vcyl(cpath, (0.0, 0.0, L.HAND_Z["second"] + 0.0006), 0.0025, 0.0012, RED)
    # battery cell in the movement (visual, on the back face) and a hanger label plate
    vcyl(cpath, (0.0, -0.016, L.MOVE_Z[0] - 0.0001), 0.007, 0.0002, (0.75, 0.62, 0.15))
    vbox(cpath, (0.0, -0.016, L.MOVE_Z[0] - 0.0005), (0.050, 0.014, 0.0008), (0.75, 0.62, 0.15), L.rx(0.0))
    # ---------------------------------------------------------------- knob link (with the minute hand)
    kpath = f"{prim_path}/knob"
    xform(kpath, cp, cr)
    rigid(kpath, L.KNOB_MASS, inertia=(2e-7, 2e-7, 1.2e-7), com=(0.0, 0.0, 0.006), iters=64)
    hull(f"{kpath}/wheel", cyl_pts(L.KNOB_R, *L.KNOB_Z, n=24), WHITE, L.KNOB_FRICTION, torsion=0.003)
    vbox(kpath, (0.0, L.KNOB_R * 0.55, L.KNOB_Z[0] - 0.0002), (0.0015, L.KNOB_R * 0.8, 0.0004), BLACK)
    vcyl(kpath, (0.0, 0.0, (L.KNOB_Z[1] + L.HAND_Z["minute"]) / 2), 0.0012, L.HAND_Z["minute"] - L.KNOB_Z[1], STEEL)
    # minute hand pointing at 6 (-y_K) when q = 0 (see layout.Q_OFFSET)
    vbox(kpath, (0.0, -L.MINUTE_LEN / 2 + 0.012, L.HAND_Z["minute"]), (0.0028, L.MINUTE_LEN + 0.024, 0.0006), BLACK)
    # ---------------------------------------------------------------- hour hand link
    hpath = f"{prim_path}/hourhand"
    xform(hpath, cp, cr)
    rigid(hpath, 0.001, inertia=(1e-7, 1e-7, 1e-7), com=(0.0, 0.0, L.HAND_Z["hour"]), iters=64)
    vbox(hpath, (0.0, L.HOUR_LEN / 2 - 0.010, L.HAND_Z["hour"]), (0.0050, L.HOUR_LEN + 0.020, 0.0006), BLACK)

    def revolute(name, body1, stiff, damp, maxf, target_rad):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{body1}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(cpath)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateAxisAttr("X")
        q = Gf.Quatf(*gfq(R_JOINT))
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(q)
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(q)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiff * math.pi / 180))  # USD angular drives are per degree
        d.CreateDampingAttr(float(damp * math.pi / 180))
        d.CreateMaxForceAttr(float(maxf))
        d.CreateTargetPositionAttr(float(math.degrees(target_rad)))
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateArmatureAttr(KNOB_ARMATURE)
        return str(j.GetPrim().GetPath())

    revolute("arbor", kpath, KNOB_STIFF, KNOB_DAMP, L.CLUTCH_NM, 0.0)
    revolute("hourpipe", hpath, HOUR_STIFF, HOUR_DAMP, 1.0, 0.0)
    _SPEC["paths"] = dict(
        wall=wpath, nail=npath, clock=cpath, knob=kpath, hourhand=hpath
    )
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
        initial_pose=Pose(
            position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
        ),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(
    score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None
)


def finger_tips(environment):
    """Pad-tip centres of all four fingers (env frame)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = (
            robot.data.body_link_pose_w[0]
            .detach()
            .cpu()
            .numpy()
            .astype(float)[list(robot.body_names).index("gripper")]
        )
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Friction:
    """Elasto-plastic friction anchor for an articulation dof driven by a stiff position drive capped at ``cap``."""

    def __init__(self, stiff, q0):
        self.stiff = stiff
        self.anchor = q0

    def update(self, q, cap):
        lim = cap / self.stiff
        if q - self.anchor > lim:
            self.anchor = q - lim
        elif self.anchor - q > lim:
            self.anchor = q + lim
        return self.anchor


Q_TARGET_MAX = 2 * math.pi - 0.2  # PhysX revolute drive targets must stay within +-2 pi


class Runtime:
    """Clutch anchor + hour follower every physics step, contact forces, scoring every control step."""

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
        root = "/World/envs/env_0/clockset"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.kin = self.simview.create_rigid_body_view([P("wall"), P("nail")])
        self.links = self.simview.create_rigid_body_view([P("clock"), P("knob"), P("hourhand")])
        self.art = self.simview.create_articulation_view(P("clock"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        self.i_knob = self.dof_names.index("arbor")
        self.i_hour = self.dof_names.index("hourpipe")
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv = self.simview.create_rigid_contact_view(
            [P("clock"), P("knob")],
            filter_patterns=[[P("nail"), P("wall")] + robot_links] * 2,
            max_contact_data_count=8192,
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.dev = self.art.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        self.setup = SETUP_BASE
        self.trial_active = False
        # the spawned joints are at 0: put the knob / hour hand (and the clock) at the base setup's start state now,
        # because the first reset of a fresh env with the base setup does not call restore_authored
        self.restore_authored(SETUP_BASE)

    def _init_state(self):
        a = authored(self.setup)
        self.start_min = a["start_min"]
        self.q0 = a["knob_q"]
        self.hour0 = a["hour_q"]
        self.clutch = Friction(KNOB_STIFF, self.q0)
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.vz_min = 0.0
        self.nail_n = 0.0
        self.robot_n = 0.0
        self.peak_robot_n = 0.0
        self.peak_knob_torque = 0.0
        self.clamped = False
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.links.get_transforms().detach().cpu().numpy().astype(float)
        kt = self.kin.get_transforms().detach().cpu().numpy().astype(float)
        return {
            "clock": (tf[0, :3] - self.origin, _rot(tf[0, 3:7])),
            "knob": (tf[1, :3] - self.origin, _rot(tf[1, 3:7])),
            "nail": (kt[1, :3] - self.origin, _rot(kt[1, 3:7])),
        }

    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        v = self.art.get_dof_velocities().detach().cpu().numpy()[0].astype(float)
        return dict(knob=float(q[self.i_knob]), hour=float(q[self.i_hour]), knob_v=float(v[self.i_knob]))

    def clock_velocity(self):
        v = self.links.get_velocities().detach().cpu().numpy().astype(float)
        return v[0, :3], v[0, 3:6]

    def shown(self, q=None):
        q = self.dofs()["knob"] if q is None else q
        return L.shown_minutes(self.start_min, q, self.q0)

    # ------------------------------------------------------------------ physics
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        torch = self.torch
        d = self.dofs()
        anchor = self.clutch.update(d["knob"], L.CLUTCH_NM)
        a_cl = float(np.clip(anchor, -Q_TARGET_MAX, Q_TARGET_MAX))
        self.clamped = self.clamped or a_cl != anchor
        hour_t = self.hour0 + (d["knob"] - self.q0) / 12.0
        tg = self.art.get_dof_position_targets().clone()
        tg[0, self.i_knob] = a_cl
        tg[0, self.i_hour] = float(np.clip(hour_t, -Q_TARGET_MAX, Q_TARGET_MAX))
        self.art.set_dof_position_targets(tg, self.ai)
        self.peak_knob_torque = max(
            self.peak_knob_torque, min(L.CLUTCH_NM, KNOB_STIFF * abs(d["knob"] - anchor))
        )
        if self.trial_active:
            f = self.cv.get_contact_force_matrix(dt=dt).detach().cpu().numpy()  # (2 links, filters, 3)
            nail = f[:, 0].sum(axis=0)
            self.nail_n = float(np.linalg.norm(nail))
            rob = np.linalg.norm(f[:, 2:], axis=-1) if f.shape[1] > 2 else np.zeros((1, 1))
            self.robot_n = float(rob.max()) if rob.size else 0.0
            self.peak_robot_n = max(self.peak_robot_n, self.robot_n)
            lin, _ = self.clock_velocity()
            self.vz_min = min(self.vz_min, float(lin[2]))
            if DEBUG and int(round(self.t / dt)) % 48 == 0:
                print("PHYS", dict(knob=round(d["knob"], 4), anchor=round(anchor, 4), nail_n=round(self.nail_n, 3),
                                   robot_n=round(self.robot_n, 3), vz=round(float(lin[2]), 4)), flush=True)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): the nail at the setup's position, the clock hanging level on
        it at rest, the knob and hour hand at the setup's start time."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        kt = self.kin.get_transforms().clone()
        kt[1, :3] = torch.tensor(np.asarray(a["nail"][0], float) + self.origin, dtype=kt.dtype)
        kt[1, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=kt.dtype)
        idx = torch.tensor([1], dtype=torch.int32, device=kt.device)
        try:
            self.kin.set_kinematic_targets(kt, idx)
        except Exception:  # noqa: BLE001
            pass
        self.kin.set_transforms(kt, idx)
        cp, cr = a["clock"]
        rt = self.art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(np.asarray(cp, float) + self.origin, dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(cr), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.ai)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.ai)
        q = self.art.get_dof_positions().clone()
        q[0, self.i_knob] = a["knob_q"]
        q[0, self.i_hour] = a["hour_q"]
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q, self.ai)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        pc, rc = poses["clock"]
        lin, ang = self.clock_velocity()
        if not (np.all(np.isfinite(pc)) and np.all(np.isfinite(rc)) and np.all(np.isfinite(lin))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        d = self.dofs()
        m = measure(
            pc,
            rc,
            L.nail_point(self.setup),
            self.shown(d["knob"]),
            self.nail_n,
            self.robot_n,
            float(np.linalg.norm(lin)),
            math.degrees(float(np.linalg.norm(ang))),
        )
        level = self.scorer.step(m, self.vz_min)
        self.vz_min = 0.0
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                knob_q=round(d["knob"], 5),
                knob_turned_deg=round(math.degrees(d["knob"] - self.q0), 2),
                hour_q=round(d["hour"], 5),
                clock_pos_mm=np.round(pc * 1000, 2).tolist(),
                start=L.fmt(self.start_min),
                setup=self.setup,
                peak_robot_n=round(self.peak_robot_n, 2),
                peak_knob_nm=round(self.peak_knob_torque, 5),
                target_clamped=self.clamped,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'clock', 'knob', 'nail'} plus the knob angle and the shown time, for the scripted demo
    (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    d = rt.dofs()
    out["knob_q"] = d["knob"]
    out["shown_min"] = rt.shown(d["knob"])
    out["start_min"] = rt.start_min
    out["nail_point"] = L.nail_point(rt.setup)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_clock_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, ClockSet):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "clockset"]
        out = orig_states(environment, plain) if plain else {}
        if "clockset" in names:
            rt = _RT["rt"]
            p, r = rt.read()["clock"] if rt else authored(SETUP_BASE)["clock"]
            z = np.zeros(3)
            out["clockset"] = ObjectState(
                pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.15, high=p + 0.15
            )
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, ClockSet) for o in task.objects):
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
    scene._clock_patched = True


def _has_bench(emb):
    return any(isinstance(o, ClockSet) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_clock_patched", False):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.NAMED_STARTS)
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
    cls._clock_patched = True


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
        type(f).__name__ == "_Finder" and getattr(f, "targets", None) and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
