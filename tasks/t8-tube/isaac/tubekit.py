"""tubekit: the relamping bench for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Bodies (custom-spawned; yam_arena task-file primitives are single boxes):
* ``fixture``: fixed-base root of one articulation (a 2-ft one-lamp T8 strip fixture screwed under a wall shelf: the
  shelf board, its back rail and brackets, the strip channel and the two rotor-lock G13 lampholders ("tombstones"),
  all colliders on the root link). The ``lamp`` (dead F17T8) hangs off it through a chain of joints: slides ``tx``,
  ``ty``, ``tz`` along the fixture frame, then rotations ``tyaw`` (about z), ``tpitch`` (about x) and ``spin`` (the
  lamp's own axis). Massless-in-effect links (1 g) carry the chain. Once the lamp is out of the lampholders all six
  joints are free (zero drive force): the lamp is an ordinary loose body that the hands carry, colliding with the
  fixture, shelf, carton, table and robot (articulation self-collision is on).
* ``carton``: free rigid body, open-top corrugated lamp-recycling carton on the bench.

Lampholder model (runtime, every physics step, through articulation drives; nothing is teleported). The pins are
drawn but not colliders; their seat in the rotors and their path down the slots are this model:
* ``locked``/turning: the pins sit in the two rotors. The lamp's position and tilt are held at the seat by stiff
  drives whose force caps are the lampholders' strength (a sustained load past F_BREAK per lampholder breaks it: veto).
  ``spin`` (the rotors) has a detent: within 12 deg of locked the contacts snap it back with up to 0.2 N m per
  lampholder; between there and open the contacts drag (elasto-plastic anchor, 0.08 N m per lampholder); the rotors
  stop at +-90 deg.
* ``slot``: at +-90 deg (within the 2.6 deg slot clearance) the pin pairs line up with the slots, which open
  DOWNWARD: nothing holds the lamp up any more (one-sided stop at the rotor centre only). While an end's pins are in
  its slot, the slot walls hold the lamp's x, yaw and spin; it slides down freely.
* ``out``: both pin pairs below the slots' bottom edges: the lamp is free (the end caps still meet the lampholder
  faces as colliders until they are below them).

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the lampholder model and
the drop monitor; after every control step the scorer (robot-on-part contact from PhysX contact forces between the
robot's links and the lamp / carton: collider identity). ``Runtime.restore_authored`` places the parts only on an
environment reset. The scripted demo READS state through ``bench_state()``.
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
from scoring import Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("TUBE_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.001
SETUP_BASE = int(os.environ.get("TUBE_SETUP", "0"))
FORCE_NAMED = "TUBE_SETUP" in os.environ

K_LAT, D_LAT = 30000.0, 30.0  # tx, ty, tz hold (N/m): 2 mm of give at the 60 N cap
K_TILT, D_TILT = 3000.0, 2.0  # tyaw, tpitch hold (N m / rad)
K_SPIN, D_SPIN = 8.0, 0.004  # spin detent / friction anchor (N m / rad): the click cap is reached in ~3 deg
CHAIN = ("s_x", "s_y", "s_z", "r_yaw", "r_pitch")
MOUNT_CHAIN = ("m_x", "m_y", "m_z")
MDOFS = ("mx", "my", "mz", "myaw")
K_MOUNT, D_MOUNT, F_MOUNT = 1.0e6, 2.0e3, 1.0e5  # wall mount: stiff slides (N/m), only moved on a reset
K_MOUNT_R, D_MOUNT_R, T_MOUNT = 1.0e5, 2.0e2, 1.0e4
DOFS = ("tx", "ty", "tz", "tyaw", "tpitch", "spin")
BODIES = ("fixture", "lamp", "carton")

GLASS, ALU, PIN, DARK, BLACK, WHITE, STEEL, WOOD, CARD, GREY, PRINT = (
    (0.95, 0.95, 0.93),
    (0.78, 0.79, 0.80),
    (0.72, 0.62, 0.35),
    (0.20, 0.20, 0.21),
    (0.04, 0.04, 0.04),
    (0.97, 0.97, 0.96),
    (0.90, 0.90, 0.88),
    (0.72, 0.58, 0.40),
    (0.70, 0.55, 0.36),
    (0.55, 0.56, 0.58),
    (0.10, 0.25, 0.55),
)


class Bench(Box):
    """The whole bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


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


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


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

    def look(color, emissive=None):
        key = (color, emissive)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            if emissive is not None:
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color),
                                                  emissive_color=tuple(8.0 * v for v in linear_rgb(emissive)),
                                                  roughness=0.5)
            else:
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

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

    def rigid(path, mass, com=None, inertia=None, kinematic=False, depen=0.3):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
            m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(depen)
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    count = {"n": 0, "k": 0}

    def mesh(path, v, color, collide=True, friction=0.5, emissive=None):
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
            hc.CreateMinThicknessAttr(0.0005)
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color, emissive), stage=stage)
        count["k"] += 1
        return m

    spec = L.setup_spec(SETUP_BASE)
    fp, fr = L.fixture_pose(spec)

    # ------------------------------------------------------------------ mount (fixed-base articulation root) + fixture
    # The root is an invisible mount fixed to the world at the nominal fixture spot; the fixture (shelf, channel,
    # lampholders) hangs off it through three slides and a yaw joint held by very stiff drives (a rigid wall mount;
    # 1e6 N/m), which only serve to place the fixture for each start state on a reset.
    m0 = np.array([L.NOMINAL["fx"], L.NOMINAL["fy"], L.TOP + L.NOMINAL["fz"]])
    Mt = f"{prim_path}/mount"
    xform(Mt, m0, np.eye(3))
    mprim = rigid(Mt, 5.0, inertia=(0.1, 0.1, 0.1))
    UsdPhysics.ArticulationRootAPI.Apply(mprim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(mprim)
    ar.CreateEnabledSelfCollisionsAttr(True)
    ar.CreateSolverPositionIterationCountAttr(32)
    ar.CreateSolverVelocityIterationCountAttr(4)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/mount_fix")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(Mt)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, m0)))
    fj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    fp, fr = m0, np.eye(3)  # every link is authored at the mount pose (all joint positions zero)
    for nme in MOUNT_CHAIN:
        xform(f"{prim_path}/{nme}", fp, fr)
        rigid(f"{prim_path}/{nme}", 0.05, inertia=(1e-4, 1e-4, 1e-4))
    Fx = f"{prim_path}/fixture"
    xform(Fx, fp, fr)
    rigid(Fx, 5.0, com=(0.10, 0.0, 0.08), inertia=(0.4, 0.05, 0.4))
    ff = L.FRICTION["fixture"]
    ch_bot = L.DROP
    shelf_bot = L.DROP + L.CH_H
    # shelf board, back rail, two brackets
    mesh(f"{Fx}/shelf", L.box_xyz((L.SHELF_FRONT, L.SHELF_FRONT + L.SHELF_D), (-L.SHELF_W / 2, L.SHELF_W / 2),
                                  (shelf_bot, shelf_bot + L.SHELF_T)), WOOD, friction=ff)
    xb = L.SHELF_FRONT + L.SHELF_D
    mesh(f"{Fx}/rail", L.box_xyz((xb, xb + 0.018), (-L.SHELF_W / 2, L.SHELF_W / 2), (shelf_bot - 0.10, shelf_bot + 0.12)),
         WOOD, friction=ff)
    for k, yy in enumerate((-0.38, 0.38)):
        pts = [[xb, yy - 0.012, shelf_bot], [xb, yy + 0.012, shelf_bot], [xb - 0.16, yy - 0.012, shelf_bot],
               [xb - 0.16, yy + 0.012, shelf_bot], [xb, yy - 0.012, shelf_bot - 0.09], [xb, yy + 0.012, shelf_bot - 0.09]]
        mesh(f"{Fx}/bracket_{k}", np.array(pts), DARK, friction=ff)
    # strip channel (one convex body: the lamp never goes inside it) with its end plates
    mesh(f"{Fx}/channel", L.box_xyz((-L.CH_W / 2, L.CH_W / 2), (-L.CH_L / 2, L.CH_L / 2), (ch_bot, shelf_bot)), WHITE,
         friction=ff)
    mesh(f"{Fx}/label", L.box_xyz((-L.CH_W / 2 - 0.0008, -L.CH_W / 2), (-0.06, 0.06), (ch_bot + 0.008, ch_bot + 0.026)),
         GREY, collide=False)
    # lampholders: inner faces FACE_GAP from the base faces; slot (dark stripe) from the bottom edge up to the rotor
    for s, nm in ((1.0, "ts_left"), (-1.0, "ts_right")):
        y0 = s * (L.HALF + L.FACE_GAP)
        y1 = s * (L.HALF + L.FACE_GAP + L.TS_T)
        mesh(f"{Fx}/{nm}", L.box_xyz((-L.TS_W / 2, L.TS_W / 2), tuple(sorted((y0, y1))), (-L.SLOT_L, ch_bot)), WHITE,
             friction=ff)
        yf = y0 - s * 0.0004
        ys = tuple(sorted((yf, yf - s * 0.0006)))
        mesh(f"{Fx}/{nm}_slot", L.box_xyz((-L.SLOT_W / 2, L.SLOT_W / 2), ys, (-L.SLOT_L, 0.0)), BLACK, collide=False)
        mesh(f"{Fx}/{nm}_rotor", L.cyl_y((0.0, (ys[0] + ys[1]) / 2, 0.0), L.ROTOR_D / 2, 0.0005, n=24), DARK,
             collide=False)
        mesh(f"{Fx}/{nm}_rotor_slot", L.box_xyz((-L.ROTOR_D / 2, L.ROTOR_D / 2), tuple(sorted((yf - s * 0.0008,
                                                                                               yf - s * 0.0012))),
                                                (-L.SLOT_W / 2, L.SLOT_W / 2)), BLACK, collide=False)

    # ------------------------------------------------------------------ lamp chain (all links at the seat)
    links = CHAIN + ("lamp",)
    for nme in links:
        xform(f"{prim_path}/{nme}", fp, fr)
    for nme in CHAIN:
        rigid(f"{prim_path}/{nme}", 0.001, inertia=(1e-7, 1e-7, 1e-7))
    Lp = f"{prim_path}/lamp"
    ixx = L.LAMP_MASS * (3 * L.TUBE_R ** 2 + L.LAMP_L ** 2) / 12
    iyy = L.LAMP_MASS * L.TUBE_R ** 2  # thin-walled tube about its axis
    rigid(Lp, L.LAMP_MASS, com=(0.0, 0.0, 0.0), inertia=(ixx, iyy, ixx))
    fl = L.FRICTION["lamp"]
    glass_l = L.LAMP_L - 2 * L.CAP_L
    mesh(f"{Lp}/glass", L.cyl_y((0, 0, 0), L.TUBE_R, glass_l, n=24), GLASS, friction=fl)
    for s in (1.0, -1.0):
        yc = s * (L.HALF - L.CAP_L / 2)
        tag = "l" if s > 0 else "r"
        mesh(f"{Lp}/cap_{tag}", L.cyl_y((0, yc, 0), L.CAP_R, L.CAP_L, n=24), ALU, friction=fl)
        for k, px in enumerate((L.PIN_SPACING / 2, -L.PIN_SPACING / 2)):  # pins: pair horizontal when locked
            mesh(f"{Lp}/pin_{tag}{k}", L.cyl_y((px, s * (L.HALF + L.PIN_L / 2), 0), L.PIN_D / 2, L.PIN_L, n=10), PIN,
                 collide=False)
        # blackened ends of a burnt-out lamp
        yb = s * (L.HALF - L.CAP_L - 0.020)
        mesh(f"{Lp}/burn_{tag}", L.cyl_y((0, yb, 0), L.TUBE_R + 0.0003, 0.030, n=24), DARK, collide=False)
    # printed lamp code on the front of the glass (turns with the lamp: shows the rotor angle)
    mesh(f"{Lp}/print", L.arc_patch_y(L.TUBE_R + 0.0002, L.TUBE_R + 0.0006, math.radians(170), math.radians(190),
                                      -0.10, 0.10), PRINT, collide=False)
    mesh(f"{Lp}/print_dot", L.arc_patch_y(L.TUBE_R + 0.0002, L.TUBE_R + 0.0006, math.radians(160), math.radians(200),
                                          0.15, 0.17), PRINT, collide=False)
    fixture_path = Sdf.Path(Fx)

    def joint(kind, name, parent, child, axis):
        cls = UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{prim_path}/{child}/{name}")
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        return j

    def drive(j, kind, stiff, damp):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "rev" else "linear")
        d.CreateTypeAttr("force")
        scale = math.pi / 180 if kind == "rev" else 1.0  # USD angular drives are per degree
        d.CreateStiffnessAttr(float(stiff * scale))
        d.CreateDampingAttr(float(damp * scale))
        d.CreateMaxForceAttr(0.0)
        d.CreateTargetPositionAttr(0.0)

    P = lambda n: Sdf.Path(f"{prim_path}/{n}")  # noqa: E731
    jm = [joint("pri", "mx", P("mount"), "m_x", "X"), joint("pri", "my", P("m_x"), "m_y", "Y"),
          joint("pri", "mz", P("m_y"), "m_z", "Z"), joint("rev", "myaw", P("m_z"), "fixture", "Z")]
    for j in jm[:3]:
        j.CreateLowerLimitAttr(-0.2)
        j.CreateUpperLimitAttr(0.2)
        drive(j, "pri", K_MOUNT, D_MOUNT)
        UsdPhysics.DriveAPI.Get(j.GetPrim(), "linear").GetMaxForceAttr().Set(F_MOUNT)
    jm[3].CreateLowerLimitAttr(-30.0)
    jm[3].CreateUpperLimitAttr(30.0)
    drive(jm[3], "rev", K_MOUNT_R, D_MOUNT_R)
    UsdPhysics.DriveAPI.Get(jm[3].GetPrim(), "angular").GetMaxForceAttr().Set(T_MOUNT)
    jx = joint("pri", "tx", fixture_path, "s_x", "X")
    jy = joint("pri", "ty", P("s_x"), "s_y", "Y")
    jz = joint("pri", "tz", P("s_y"), "s_z", "Z")
    for j in (jx, jy, jz):
        j.CreateLowerLimitAttr(-1.2)
        j.CreateUpperLimitAttr(1.2)
    jyaw = joint("rev", "tyaw", P("s_z"), "r_yaw", "Z")
    jyaw.CreateLowerLimitAttr(-170.0)
    jyaw.CreateUpperLimitAttr(170.0)
    jp = joint("rev", "tpitch", P("r_yaw"), "r_pitch", "X")
    jp.CreateLowerLimitAttr(-80.0)
    jp.CreateUpperLimitAttr(80.0)
    js = joint("rev", "spin", P("r_pitch"), "lamp", "Y")
    for j, k in ((jx, "pri"), (jy, "pri"), (jz, "pri")):
        drive(j, k, K_LAT, D_LAT)
    drive(jyaw, "rev", K_TILT, D_TILT)
    drive(jp, "rev", K_TILT, D_TILT)
    drive(js, "rev", K_SPIN, D_SPIN)

    # ------------------------------------------------------------------ carton (free rigid body)
    Cp = f"{prim_path}/carton"
    bp, br = L.carton_pose(spec)
    xform(Cp, bp + np.array([0.0, 0.0, 0.0005]), br)
    hx, hy, hz = L.BOX_IN[0] / 2, L.BOX_IN[1] / 2, L.BOX_IN[2]
    w = L.BOX_WALL
    rigid(Cp, L.BOX_MASS, com=(0.0, 0.0, 0.02), inertia=(0.011, 0.0006, 0.011))
    fc = L.FRICTION["carton"]
    mesh(f"{Cp}/bottom", L.box_xyz((-hx - w, hx + w), (-hy - w, hy + w), (0.0, w)), CARD, friction=fc)
    mesh(f"{Cp}/front", L.box_xyz((-hx - w, -hx), (-hy - w, hy + w), (w, w + hz)), CARD, friction=fc)
    mesh(f"{Cp}/back", L.box_xyz((hx, hx + w), (-hy - w, hy + w), (w, w + hz)), CARD, friction=fc)
    mesh(f"{Cp}/end_l", L.box_xyz((-hx, hx), (hy, hy + w), (w, w + hz)), CARD, friction=fc)
    mesh(f"{Cp}/end_r", L.box_xyz((-hx, hx), (-hy - w, -hy), (w, w + hz)), CARD, friction=fc)
    mesh(f"{Cp}/mark", L.box_xyz((-hx - w - 0.0008, -hx - w), (-0.09, 0.09), (w + 0.012, w + 0.034)), (0.10, 0.45, 0.20),
         collide=False)

    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in BODIES}
    _SPEC["colliders"] = count["n"]
    _SPEC["meshes"] = count["k"]
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

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=BenchSpawnerCfg(),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# ---------------------------------------------------------------------------------------------- runtime state


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None)


class Friction:
    """Elasto-plastic friction anchor for an articulation dof driven by a stiff position drive capped at ``cap``."""

    def __init__(self, stiff, q0=0.0):
        self.stiff = stiff
        self.anchor = q0

    def update(self, q, cap):
        lim = cap / self.stiff
        if q - self.anchor > lim:
            self.anchor = q - lim
        elif self.anchor - q > lim:
            self.anchor = q + lim
        return self.anchor


class Holder:
    """State machine of the lamp in its two lampholders (see the module docstring)."""

    def __init__(self):
        self.state = "locked"
        self.side = 0.0  # +1 / -1: the direction it was turned open
        self.anchor = Friction(K_SPIN)
        self.events = []
        self.over = {}  # dof -> seconds at its force cap (break monitor)
        self.max_spin_deg = 0.0
        self.broke = None


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


CAP_LAT = 2 * L.F_BREAK  # two lampholders share a side / pull load on the lamp
CAP_TILT = L.F_BREAK * L.LAMP_L  # a tilting couple loads each lampholder with torque / lamp length


class Runtime:
    """Lampholder model every physics step; scorer every control step."""

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
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.art = self.simview.create_articulation_view(P("mount"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        print("tubekit: dofs", self.dof_names, flush=True)
        self.idof = {n: self.dof_names.index(n) for n in DOFS}
        self.imdof = {n: self.dof_names.index(n) for n in MDOFS}
        self.view = self.simview.create_rigid_body_view([P(n) for n in BODIES])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv = self.simview.create_rigid_contact_view([P(n) for n in BODIES],
                                                         filter_patterns=[robot_links] * len(BODIES),
                                                         max_contact_data_count=4096)
        self.dev = self.art.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.mf_now = None
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.holder = Holder()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.contact_step = {n: 0.0 for n in BODIES}
        self.contact_peak = {n: 0.0 for n in BODIES}
        self.robot_link_peak = {}
        self.peaks = dict(lat_n=0.0, tilt_nm=0.0, spin_nm=0.0, v_free=0.0)
        self.dropped = None
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        return {n: float(q[i]) for n, i in self.idof.items()}

    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(BODIES)}

    def lamp_speed(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return float(np.linalg.norm(v[BODIES.index("lamp"), :3]))

    @staticmethod
    def end_z(d):
        """Heights (fixture frame) of the two pin pairs' centres relative to the rotor centres."""
        return [d["tz"] + s * L.HALF * math.sin(d["tpitch"]) * math.cos(d["tyaw"]) for s in (1.0, -1.0)]

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        h = self.holder
        d = self.dofs()
        sp = d["spin"]
        h.max_spin_deg = max(h.max_spin_deg, abs(math.degrees(sp))) if h.state in ("locked", "slot") else h.max_spin_deg
        mf = np.zeros(6)
        tg = np.zeros(6)
        open_rad = math.radians(L.OPEN_DEG - L.OPEN_TOL_DEG)
        if h.state == "locked" and abs(sp) >= open_rad:
            h.state, h.side = "slot", math.copysign(1.0, sp)
            h.events.append(dict(t=round(self.t, 3), ev="open (pins line up with the slots)", spin=round(math.degrees(sp), 1)))
        elif h.state == "slot":
            ez = self.end_z(d)
            if d["tz"] > -0.001 and abs(sp) < open_rad - math.radians(1.0):
                h.state = "locked"
                h.events.append(dict(t=round(self.t, 3), ev="turned back into the rotors"))
            elif min(ez) < -L.SLOT_L and max(ez) < -L.SLOT_L:
                h.state = "out"
                h.events.append(dict(t=round(self.t, 3), ev="out of the lampholders"))
        if h.state == "locked":
            mf[:5] = (CAP_LAT, CAP_LAT, CAP_LAT, CAP_TILT, CAP_TILT)
            a = abs(math.degrees(sp))
            if a < L.DETENT_DEG:
                tg[5], mf[5] = 0.0, 2 * L.T_CLICK
                h.anchor.anchor = sp
            elif a <= L.OPEN_DEG:
                mf[5] = 2 * L.T_RUN
                tg[5] = h.anchor.update(sp, mf[5])
            else:
                mf[5] = 2 * L.T_STOP
                tg[5] = math.copysign(math.radians(L.OPEN_DEG), sp)
        elif h.state == "slot":
            in_slot = [z >= -L.SLOT_L for z in self.end_z(d)]
            mf[0] = CAP_LAT  # slot walls: across the slot
            mf[1] = CAP_LAT  # lampholder faces: along the lamp (the end caps also meet them as colliders)
            if d["tz"] > 0.0:
                mf[2] = CAP_LAT  # rotor centre: the top of the slot (one-sided)
            if all(in_slot):
                mf[3] = CAP_TILT  # both ends in their slots: no yaw
            mf[5] = 2 * L.T_STOP  # pins in the 3.2 mm slot cannot turn
            tg[5] = h.side * math.radians(L.OPEN_DEG)
        if h.state in ("locked", "slot"):
            k = (K_LAT, K_LAT, K_LAT, K_TILT, K_TILT)
            for i in range(5):
                if mf[i] <= 0.0:
                    continue
                f = k[i] * abs(d[DOFS[i]] - tg[i])
                if i < 3:
                    self.peaks["lat_n"] = max(self.peaks["lat_n"], min(f, mf[i]))
                else:
                    self.peaks["tilt_nm"] = max(self.peaks["tilt_nm"], min(f, mf[i]))
                if f >= 0.98 * mf[i]:
                    h.over[DOFS[i]] = h.over.get(DOFS[i], 0.0) + dt
                    if h.over[DOFS[i]] >= L.BREAK_HOLD_S and self.trial_active:
                        h.state, h.broke = "broken", DOFS[i]
                        h.events.append(dict(t=round(self.t, 3), ev=f"lampholder broken ({DOFS[i]} load at its cap)"))
                        mf[:] = 0.0
                        break
                else:
                    h.over[DOFS[i]] = 0.0
            self.peaks["spin_nm"] = max(self.peaks["spin_nm"], min(mf[5], K_SPIN * abs(sp - tg[5])))
        targets = self.art.get_dof_position_targets().clone()
        for k2, n in enumerate(DOFS):
            targets[0, self.idof[n]] = float(tg[k2])
        self.art.set_dof_position_targets(targets, self.ai)
        if self.mf_now is None or np.max(np.abs(mf - self.mf_now)) > 1e-4 * (1.0 + np.max(np.abs(mf))):
            m = self.art.get_dof_max_forces().clone()
            for k2, n in enumerate(DOFS):
                m[0, self.idof[n]] = float(mf[k2])
            self.art.set_dof_max_forces(m, self.ai_cpu)
            self.mf_now = mf.copy()
        if self.trial_active:
            fm = self.cv.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            per = np.zeros(len(BODIES))
            if fm.size:
                nl = np.linalg.norm(fm, axis=-1)
                per = nl.sum(axis=-1)
                for i, n in enumerate(BODIES):
                    self.contact_step[n] = max(self.contact_step[n], float(per[i]))
                    self.contact_peak[n] = max(self.contact_peak[n], float(per[i]))
                    j = int(np.argmax(nl[i]))
                    if nl[i, j] > self.robot_link_peak.get(n, (0.0, ""))[0]:
                        self.robot_link_peak[n] = (float(nl[i, j]), "/".join(self.robot_links[j].split("/")[-2:]))
            if h.state in ("out", "slot", "broken"):
                v = self.lamp_speed()
                if per[BODIES.index("lamp")] < 0.05:
                    self.peaks["v_free"] = max(self.peaks["v_free"], v)
                    if v > L.V_DROP and self.dropped is None:
                        p, _ = self.read()["lamp"]
                        self.dropped = dict(t=round(self.t, 3), v=round(v, 3), z=round(float(p[2] - L.TOP), 4))
                        h.events.append(dict(t=round(self.t, 3), ev="lamp fell (no hand on it)", v=round(v, 2)))
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): fixture (with the lamp locked in it) and carton placed for
        the setup, at rest; lampholder drives at the locked hold."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        spec = L.setup_spec(setup)
        mq = (spec["fx"] - L.NOMINAL["fx"], spec["fy"] - L.NOMINAL["fy"], spec["fz"] - L.NOMINAL["fz"],
              math.radians(spec["fyaw"]))
        q = torch.zeros_like(self.art.get_dof_positions())
        for k2, n in enumerate(MDOFS):
            q[0, self.imdof[n]] = float(mq[k2])
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q.clone(), self.ai)
        m = self.art.get_dof_max_forces().clone()
        for k2, n in enumerate(MDOFS):
            m[0, self.imdof[n]] = float((F_MOUNT, F_MOUNT, F_MOUNT, T_MOUNT)[k2])
        for k2, n in enumerate(DOFS):
            m[0, self.idof[n]] = float((CAP_LAT, CAP_LAT, CAP_LAT, CAP_TILT, CAP_TILT, 2 * L.T_CLICK)[k2])
        self.art.set_dof_max_forces(m, self.ai_cpu)
        self.mf_now = None
        bp, br = L.carton_pose(spec)
        tf = self.view.get_transforms().clone()
        k = BODIES.index("carton")
        tf[k, :3] = torch.tensor(bp + np.array([0.0, 0.0, 0.0005]) + self.origin, dtype=tf.dtype)
        tf[k, 3:7] = torch.tensor(_quat(br), dtype=tf.dtype)
        ids = torch.tensor([k], dtype=torch.int32, device=self.dev)
        self.view.set_transforms(tf, ids)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), ids)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def measure(self):
        poses = self.read()
        h = self.holder
        lp, lr = poses["lamp"]
        cp, cr = poses["carton"]
        fp, fr = poses["fixture"]
        spec = L.setup_spec(self.setup)
        gp, gr = L.fixture_pose(spec)
        in_box, on_bench = L.resting_surfaces(lp, lr, (cp, cr))
        vel = self.view.get_velocities()[BODIES.index("lamp")].detach().cpu().numpy()
        lever = L.HALF * lr[:, 1]
        end_velocity = np.cross(vel[3:], lever)
        end_speed = max(np.linalg.norm(vel[:3] + end_velocity), np.linalg.norm(vel[:3] - end_velocity))
        surface_speed = float(end_speed + L.CAP_R * np.linalg.norm(vel[3:]))
        return dict(poses=poses, state=h.state, in_carton=L.in_carton(lp, lr, (cp, cr)),
                    on_bottom=in_box, on_bench=on_bench, speed=surface_speed,
                    fixture_moved_mm=float(np.linalg.norm(fp - gp)) * 1e3,
                    lamp_z=float(lp[2] - L.TOP), carton_shift_mm=float(np.linalg.norm(cp[:2] - L.carton_pose(spec)[0][:2])) * 1e3)

    def control(self, environment, dt=0.05):
        if not self.trial_active:
            return
        self.steps += 1
        m = self.measure()
        if not all(np.all(np.isfinite(p)) for p, _ in m["poses"].values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        contact = {n: self.contact_step[n] > 0.05 for n in BODIES}
        h = self.holder
        d = self.dofs()
        level = self.scorer.step(dt, m["state"], m["on_bottom"], m["speed"], contact, dropped=self.dropped is not None,
                                 broken=h.state == "broken", on_bench=m["on_bench"])
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                holder=h.state,
                spin_deg=round(math.degrees(d["spin"]), 2),
                max_spin_deg=round(h.max_spin_deg, 2),
                lamp_dz_mm=round(d["tz"] * 1e3, 2),
                lamp_z=round(m["lamp_z"], 4),
                speed=round(m["speed"], 3),
                dropped=self.dropped,
                broke=h.broke,
                contact_n={n: round(v, 2) for n, v in self.contact_step.items()},
                contact_peak_n={n: round(v, 1) for n, v in self.contact_peak.items()},
                robot_link_peak={k: [round(v[0], 1), v[1]] for k, v in self.robot_link_peak.items()},
                peaks={k: round(v, 3) for k, v in self.peaks.items()},
                carton_shift_mm=round(m["carton_shift_mm"], 1),
                fixture_moved_mm=round(m["fixture_moved_mm"], 2),
                events=h.events[-6:],
                setup=self.setup,
            ),
        )
        self.contact_step = {n: 0.0 for n in BODIES}


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'fixture', 'lamp', 'carton'} plus the lampholder state, for the scripted demo (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    m = rt.measure()
    out = dict(m["poses"])
    out.update(holder=rt.holder.state, in_carton=m["in_carton"], speed=m["speed"], dofs=rt.dofs())
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_tube_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Bench):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "bench"]
        out = orig_states(environment, plain) if plain else {}
        if "bench" in names:
            rt = _RT["rt"]
            if rt:
                p, r = rt.read()["fixture"]
            else:
                p, r = L.fixture_pose(L.setup_spec(SETUP_BASE))
            z = np.zeros(3)
            out["bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.3,
                                       high=p + 0.3)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Bench) for o in task.objects):
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
    scene._tube_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_tube_patched", False):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.NAMED_SETUPS)
        if rt is not None:
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            spec = L.setup_spec(setup)
            moved = float(np.linalg.norm(rt.read()["fixture"][0] - L.fixture_pose(spec)[0])) > 0.001
            if rt.setup != setup or moved or _RT["trials"] == 0:
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._tube_patched = True


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
