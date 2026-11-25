"""capkit: the solvent-waste bench for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Bodies (custom-spawned; yam_arena task-file primitives are single boxes):
* ``bottle``: floating root of one articulation (500 mL HDPE bottle: hollow wall, shoulder and 53 mm neck built from
  convex pieces; the collected waste is a rigid mass low in the bottle). The ``cap`` hangs off it through a chain of
  joints: slides ``cx``, ``cy``, ``cz`` along the bottle axes, then rotations ``crx``, ``cry`` and ``crz`` (the cap's own
  axis). Massless-in-effect links (1 g) carry the chain. While the cap is off the thread all six joints are free
  (zero drive force): the cap is an ordinary loose body that the hands pick up, carry and set down, colliding with the
  bottle, table and robot (articulation self-collision is on).
* ``funnel``: free rigid body (75 mm polypropylene funnel with a hanging tab on its rim) standing in the neck.
* static: lab hot plate (switched on: indicator and HOT lamp lit), dedicated four-sided drip tray on a riser.

Thread model (runtime, every physics step, through articulation drives; nothing is teleported):
* capture: the cap's axis within 1.6 mm and 5 deg of the neck's axis and its liner face at or below the thread start
  (4.23 mm above the lip; the 1 mm-clearance skirt over the neck keeps it centred): the cap rests on the thread start
  (one-sided support on ``cz``), free to turn;
* turning clockwise (seen from above) by phi from there lowers the support by 4.23 mm per turn; past 20 deg the
  flanks overlap: ``cx``/``cy``/``crx``/``cry`` are held at the axis and ``cz`` follows the helix both ways (the cap no
  longer lifts off); turning back unscrews it;
* ``crz`` has the thread's running friction (an elasto-plastic anchor capped at T_RUN); after one turn the liner meets
  the lip (seat): the friction rises by K_SEAT per rad (liner compression) up to a hard stop 30 deg later.

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the thread model; after
every control step the scorer (robot-on-part contact from PhysX contact forces between the robot's links and the cap /
funnel / bottle: collider identity). ``Runtime.restore_authored`` places the parts only on an environment reset. The
scripted demo READS state through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("CAP_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.001
SETUP_BASE = int(os.environ.get("CAP_SETUP", "0"))
FORCE_NAMED = "CAP_SETUP" in os.environ

Z0 = 0.30  # chain origin this far above the lip (cz = z_rel - Z0), so zero joint positions park the cap out of the way
K_LAT, D_LAT, F_LAT = 20000.0, 20.0, 300.0  # cx, cy (engaged)
K_AX, D_AX, F_AX = 30000.0, 30.0, 400.0  # cz (thread support)
K_TILT, D_TILT, T_TILT = 40.0, 0.05, 6.0  # crx, cry (engaged), N m / rad
K_ROT, D_ROT = 20.0, 0.002  # crz friction anchor (elastic play T_RUN / 20 rad)
CHAIN = ("s_x", "s_y", "s_z", "r_x", "r_y")
DOFS = ("cx", "cy", "cz", "crx", "cry", "crz")
BODIES = ("bottle", "cap", "funnel")

HDPE, CAPC, FUNNELC, WHITE, RED, BLACK, GREY, DARK, CERAMIC, ORANGE, TRAY_BLUE = (
    (0.86, 0.86, 0.82),
    (0.93, 0.93, 0.95),
    (0.70, 0.78, 0.86),
    (0.96, 0.96, 0.95),
    (0.85, 0.08, 0.06),
    (0.03, 0.03, 0.03),
    (0.55, 0.56, 0.58),
    (0.18, 0.18, 0.20),
    (0.80, 0.80, 0.78),
    (1.00, 0.45, 0.05),
    (0.30, 0.45, 0.70),
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
    bp, br = L.bottle_pose(spec)
    origin = L.lip_station(spec) + br @ np.array([0.0, 0.0, Z0])  # chain origin (cap parked here at zero dofs)

    # ------------------------------------------------------------------ bottle (articulation root)
    B = f"{prim_path}/bottle"
    xform(B, bp, br)
    bprim = rigid(B, L.BOTTLE_MASS, com=L.BOTTLE_COM, inertia=(0.78e-3, 0.78e-3, 0.32e-3))
    UsdPhysics.ArticulationRootAPI.Apply(bprim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(bprim)
    ar.CreateEnabledSelfCollisionsAttr(True)
    ar.CreateSolverPositionIterationCountAttr(32)
    ar.CreateSolverVelocityIterationCountAttr(4)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    fb = L.FRICTION["bottle"]
    mesh(f"{B}/base", L.cyl_z((0, 0, 0.0025), L.BODY_R, 0.005, n=32), HDPE, friction=fb)
    for i, v in enumerate(L.ring_segments(L.BODY_R - L.WALL_T, L.BODY_R, 0.004, L.BODY_H, n=24)):
        mesh(f"{B}/wall_{i}", v, HDPE, friction=fb)
    # shoulder: sloped annulus from the wall top to the neck root
    n = 24
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            pts += [[L.BODY_R * c, L.BODY_R * s, L.BODY_H], [L.NECK_R_OUT * c, L.NECK_R_OUT * s, L.BODY_H + L.SHOULDER_H],
                    [(L.BODY_R - L.WALL_T) * c, (L.BODY_R - L.WALL_T) * s, L.BODY_H - 0.002],
                    [L.NECK_R_IN * c, L.NECK_R_IN * s, L.BODY_H + L.SHOULDER_H - 0.002]]
        mesh(f"{B}/shoulder_{i}", np.array(pts), HDPE, friction=fb)
    for i, v in enumerate(L.ring_segments(L.NECK_R_IN, L.NECK_R_OUT, L.BODY_H + L.SHOULDER_H - 0.002, L.H_LIP, n=32)):
        mesh(f"{B}/neck_{i}", v, HDPE, friction=fb)
    # visual thread ridges on the neck (simplified circumferential ridges) and the waste label
    for k in range(3):
        z = L.H_LIP - 0.004 - k * L.PITCH
        mesh(f"{B}/ridge_{k}", L.cyl_z((0, 0, z), L.NECK_R_OUT + 0.0006, 0.0012, n=32), HDPE, collide=False)
    th0 = math.radians(180.0 - L.LABEL_HALF_ANGLE)
    th1 = math.radians(180.0 + L.LABEL_HALF_ANGLE)
    nl = 8
    for i in range(nl):
        a0 = th0 + (th1 - th0) * i / nl
        a1 = th0 + (th1 - th0) * (i + 1) / nl
        for name, (z0, z1), col in (("lab", L.LABEL_Z, WHITE), ("band", (0.089, 0.097), RED)):
            r0, r1 = L.BODY_R + 0.0004, L.BODY_R + (0.0010 if name == "lab" else 0.0016)
            pts = []
            for a in (a0, a1):
                c, s = math.cos(a), math.sin(a)
                pts += [[r0 * c, r0 * s, z0], [r0 * c, r0 * s, z1], [r1 * c, r1 * s, z0], [r1 * c, r1 * s, z1]]
            mesh(f"{B}/label_{name}_{i}", np.array(pts), col, collide=False)
    # Black lettering on the curved side facing the robot. Each ink pixel is one
    # quad on the same visual mesh, so label details add no contact geometry.
    letters = {"S": ("111","100","111","001","111"), "O": ("111","101","101","101","111"),
               "L": ("100","100","100","100","111"), "V": ("101","101","101","101","010"),
               "E": ("111","100","110","100","111"), "N": ("1001","1101","1011","1001","1001"),
               "T": ("111","010","010","010","010"), "W": ("10001","10001","10101","10101","01010"),
               "A": ("010","101","111","101","101")}
    letters.update({
        "F": ("111","100","110","100","100"), "M": ("10001","11011","10101","10001","10001"),
        "B": ("110","101","110","101","110"), "U": ("101","101","101","101","111"),
        "D": ("110","101","101","101","110"), " ": ("0","0","0","0","0"),
    })

    def lettering(path, lines, px, py, surface, color):
        vertices = []
        for word, top in lines:
            width = sum(len(letters[ch][0]) + 1 for ch in word) - 1
            column = -width / 2
            for ch in word:
                for row, glyph in enumerate(letters[ch]):
                    for ix, mark in enumerate(glyph):
                        if mark == "1":
                            for dx, dy in ((0, 0), (1, 0), (1, 1), (0, 1)):
                                vertices.append(Gf.Vec3f(*surface((column + ix + dx)*px, top-(row+dy)*py)))
                column += len(letters[ch][0]) + 1
        ink = UsdGeom.Mesh.Define(stage, path)
        ink.CreatePointsAttr(vertices)
        ink.CreateFaceVertexCountsAttr([4] * (len(vertices) // 4))
        ink.CreateFaceVertexIndicesAttr(list(range(len(vertices))))
        ink.CreateDoubleSidedAttr(True)
        ink.CreateSubdivisionSchemeAttr("none")
        bind_visual_material(str(ink.GetPath()), look(color), stage=stage)

    def bottle_surface(u, z):
        radius = L.BODY_R + 0.002
        angle = math.pi + u / radius
        return radius*math.cos(angle), radius*math.sin(angle), z

    lettering(f"{B}/flammable_waste_text", [("FLAMMABLE", .085), ("WASTE", .060)],
              .0017, .0033, bottle_surface, BLACK)

    # ------------------------------------------------------------------ cap chain
    links = CHAIN + ("cap",)
    for nme in links:
        xform(f"{prim_path}/{nme}", origin, br)
    for nme in CHAIN:
        rigid(f"{prim_path}/{nme}", 0.001, inertia=(1e-7, 1e-7, 1e-7))
    C = f"{prim_path}/cap"
    rigid(C, L.CAP_MASS, com=(0.0, 0.0, -0.006), inertia=(4.5e-6, 4.5e-6, 7.5e-6))
    fc = L.FRICTION["cap"]
    mesh(f"{C}/top", L.cyl_z((0, 0, L.CAP_TOP_T / 2), L.CAP_R_OUT, L.CAP_TOP_T, n=40), CAPC, friction=fc)
    for i, v in enumerate(L.ring_segments(L.CAP_R_IN, L.CAP_R_OUT, -L.CAP_DEPTH, 0.0005, n=36,
                                          r_in_bottom=L.CAP_R_IN + L.CAP_CHAMFER, chamfer=L.CAP_CHAMFER)):
        mesh(f"{C}/skirt_{i}", v, CAPC, friction=fc)
    for i in range(18):  # knurl ribs (visual)
        a = 2 * math.pi * i / 18
        c, s = math.cos(a), math.sin(a)
        mesh(f"{C}/rib_{i}", L.box((L.CAP_R_OUT * c, L.CAP_R_OUT * s, -L.CAP_DEPTH / 2 + 0.001),
                                   (0.0012, 0.0012, L.CAP_DEPTH - 0.003)) , GREY, collide=False)
    body = Sdf.Path(B)

    def joint(kind, name, parent, child, axis, pos0):
        cls = UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{prim_path}/{child}/{name}")
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
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

    o_b = br.T @ (origin - bp)  # chain origin in the bottle frame
    jx = joint("pri", "cx", body, "s_x", "X", o_b)
    jy = joint("pri", "cy", Sdf.Path(f"{prim_path}/s_x"), "s_y", "Y", (0, 0, 0))
    jz = joint("pri", "cz", Sdf.Path(f"{prim_path}/s_y"), "s_z", "Z", (0, 0, 0))
    for j in (jx, jy):
        j.CreateLowerLimitAttr(-0.6)
        j.CreateUpperLimitAttr(0.6)
    jz.CreateLowerLimitAttr(-Z0 - L.H_LIP - 0.05)
    jz.CreateUpperLimitAttr(0.3)
    jrx = joint("rev", "crx", Sdf.Path(f"{prim_path}/s_z"), "r_x", "X", (0, 0, 0))
    jry = joint("rev", "cry", Sdf.Path(f"{prim_path}/r_x"), "r_y", "Y", (0, 0, 0))
    jrz = joint("rev", "crz", Sdf.Path(f"{prim_path}/r_y"), "cap", "Z", (0, 0, 0))
    for j in (jrx, jry):
        j.CreateLowerLimitAttr(-175.0)
        j.CreateUpperLimitAttr(175.0)
    drive(jx, "pri", K_LAT, D_LAT)
    drive(jy, "pri", K_LAT, D_LAT)
    drive(jz, "pri", K_AX, D_AX)
    drive(jrx, "rev", K_TILT, D_TILT)
    drive(jry, "rev", K_TILT, D_TILT)
    drive(jrz, "rev", K_ROT, D_ROT)

    # ------------------------------------------------------------------ funnel (free rigid body)
    Fp = f"{prim_path}/funnel"
    fpos, frot = L.funnel_start(spec)
    xform(Fp, fpos, frot)
    rigid(Fp, L.F_MASS, com=(0.0, 0.0, L.F_COM_Z), inertia=(1.6e-5, 1.6e-5, 1.8e-5))
    ff = L.FRICTION["funnel"]
    for i, v in enumerate(L.cone_segments(16)):
        mesh(f"{Fp}/cone_{i}", v, FUNNELC, friction=ff)
    mesh(f"{Fp}/stem", L.cyl_z((0, 0, -L.F_STEM_L / 2), L.F_STEM_R, L.F_STEM_L, n=16), FUNNELC, friction=ff)
    tl, tw, tt = L.F_TAB
    mesh(f"{Fp}/tab", L.box((L.F_R_TOP + tl / 2 - 0.002, 0.0, L.F_CONE_H - tt / 2), (tl + 0.004, tw, tt)), FUNNELC,
         friction=ff)
    mesh(f"{Fp}/tab_hole", L.cyl_z((L.F_R_TOP + tl * 0.6, 0.0, L.F_CONE_H + 0.0002), 0.004, 0.0008, n=12), DARK,
         collide=False)

    # ------------------------------------------------------------------ hot plate (static, switched on)
    S = f"{prim_path}/static"
    xform(S, (0.0, 0.0, 0.0), np.eye(3))
    hx, hy = L.HP_CENTER
    dx, dy, dz = L.HP_SIZE
    T = L.TOP
    mesh(f"{S}/hp_body", L.box((hx, hy, T + dz / 2), (dx, dy, dz)), WHITE)
    tx, ty = L.HP_TOP_SIZE
    mesh(f"{S}/hp_top", L.box((hx + 0.03, hy, T + dz + 0.003), (tx, ty, 0.006)), CERAMIC)
    mesh(f"{S}/hp_panel", L.box((hx - dx / 2 + 0.035, hy, T + dz * 0.55), (0.07, dy - 0.01, dz * 0.9)), DARK,
         collide=False)
    fx = hx - dx / 2 - 0.0015
    mesh(f"{S}/hp_display", L.box((fx, hy + 0.035, T + 0.070), (0.003, 0.040, 0.016)), ORANGE, collide=False,
         emissive=ORANGE)
    mesh(f"{S}/hp_heat_led", L.box((fx, hy - 0.010, T + 0.072), (0.003, 0.010, 0.010)), RED, collide=False,
         emissive=RED)
    mesh(f"{S}/hp_hot_lamp", L.box((fx, hy - 0.045, T + 0.072), (0.003, 0.030, 0.012)), RED, collide=False,
         emissive=RED)
    for k, yy in enumerate((hy + 0.045, hy - 0.050)):  # knobs: heat (right), stir (left)
        mesh(f"{S}/hp_knob_{k}", L.box((fx - 0.010, yy, T + 0.032), (0.020, 0.030, 0.030)), GREY)
    mesh(f"{S}/hp_knob_mark", L.box((fx - 0.021, hy - 0.050, T + 0.040), (0.002, 0.004, 0.012)), ORANGE,
         collide=False, emissive=ORANGE)
    # A removable, continuous catch basin stands on a fixed pedestal. The four
    # inner wall faces bound the same region used to check the resting funnel.
    (x0,x1),(y0,y1),(floor,rim),w = L.TRAY_X,L.TRAY_Y,L.TRAY_Z,L.TRAY_WALL
    floor, rim = T+floor, T+rim
    mesh(f"{S}/tray_riser", L.box_xyz((x0+.012,x1-.012),(y0+.012,y1-.012),(T,floor-.006)), DARK)
    mesh(f"{S}/tray_base", L.box_xyz((x0-w,x1+w),(y0-w,y1+w),(floor-.006,floor)), TRAY_BLUE)
    for name,xb,yb in (("front",(x0-w,x0),(y0-w,y1+w)), ("back",(x1,x1+w),(y0-w,y1+w)),
                       ("left",(x0,x1),(y1,y1+w)), ("right",(x0,x1),(y0-w,y0))):
        mesh(f"{S}/tray_{name}", L.box_xyz(xb,yb,(floor,rim)), TRAY_BLUE)

    # Bench markings face the overhead camera. The waste placard is beside
    # the bottle, and the tray marking sits at the dedicated basin's front.
    def marker(name, x, y, width, height, lines, px, py, ink):
        mesh(f"{S}/{name}_panel", L.box((x,y,T+.001), (height,width,.002)), WHITE, collide=False)
        lettering(f"{S}/{name}_ink", lines, px, py, lambda u,v: (x+v,y-u,T+.0022), ink)

    # The larger waste placard preserves the icon and word shape in the
    # 320-pixel policy view. All coordinates below are in its tabletop plane.
    sx, sy = .52, -.065
    mesh(f"{S}/waste_sign_panel", L.box((sx,sy,T+.001),(.154,.370,.002)), WHITE, collide=False)
    lettering(f"{S}/waste_sign_ink", [("FLAMMABLE",.061),("WASTE",-.006)], .0067,.0092,
              lambda u,v: (sx+v,sy-u-.038,T+.0023), RED)

    def symbol(path, outline, color, height):
        surface = UsdGeom.Mesh.Define(stage, f"{S}/{path}")
        surface.CreatePointsAttr([Gf.Vec3f(sx+v,sy+.138-u,T+height) for u,v in outline])
        surface.CreateFaceVertexCountsAttr([len(outline)])
        surface.CreateFaceVertexIndicesAttr(list(range(len(outline))))
        surface.CreateDoubleSidedAttr(True)
        surface.CreateSubdivisionSchemeAttr("none")
        bind_visual_material(str(surface.GetPath()),look(color),stage=stage)

    for name, size, color, height in (("hazard_border",.044,RED,.0024),("hazard_field",.035,WHITE,.0027)):
        symbol(name,[(0.,size),(size,0.),(0.,-size),(-size,0.)],color,height)
    symbol("flame", [(-.016,-.010),(-.016,-.002),(-.010,.009),(-.009,.001),
                     (-.003,.013),(-.003,.024),(.005,.015),(.008,.005),(.013,.010),
                     (.017,0.),(.017,-.009),(.010,-.017),(-.006,-.018)],BLACK,.0030)
    symbol("flame_base",[(-.017,-.020),(.017,-.020),(.017,-.023),(-.017,-.023)],BLACK,.0030)
    marker("tray_sign", .129,-.240,.150,.051, [("USED",.021),("FUNNEL",-.004)],
           .0040,.0035,BLACK)

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


class Thread:
    """State machine of the cap on the neck (see the module docstring)."""

    def __init__(self):
        self.state = "free"
        self.phi = 0.0  # clockwise turn (rad) from the thread start
        self.ref = 0.0  # crz at the thread start
        self.rz_prev = None
        self.rz_unwrapped = 0.0
        self.events = []
        self.anchor = Friction(K_ROT)
        self.t_peak = 0.0

    def unwrap(self, rz):
        if self.rz_prev is None:
            self.rz_prev = rz
            self.rz_unwrapped = rz
        self.rz_unwrapped += _wrap_pi(rz - self.rz_prev)
        self.rz_prev = rz
        return self.rz_unwrapped

    def friction_cap(self):
        past = self.phi - L.PHI_SEAT
        if past <= 0.0:
            return L.T_RUN
        if past >= math.radians(L.SEAT_STOP_DEG):
            return L.T_STOP
        return L.T_RUN + L.K_SEAT * past


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
    """Thread model every physics step; scorer every control step."""

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
        self.art = self.simview.create_articulation_view(P("bottle"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        print("capkit: dofs", self.dof_names, flush=True)
        self.idof = {n: self.dof_names.index(n) for n in DOFS}
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
        self.thread = Thread()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.contact_step = {n: 0.0 for n in BODIES}
        self.contact_peak = {n: 0.0 for n in BODIES}
        self.robot_link_peak = {}
        self.peaks = dict(ax_n=0.0, lat_n=0.0, rot_nm=0.0, tilt_nm=0.0)
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

    def cap_rel(self, d=None):
        """Cap relative to the lip in the bottle frame: (x, y, z_rel, tilt_deg)."""
        d = d or self.dofs()
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(d["crx"]) * math.cos(d["cry"])))))
        return d["cx"], d["cy"], d["cz"] + Z0, tilt

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        th = self.thread
        d = self.dofs()
        x, y, z, tilt = self.cap_rel(d)
        r = math.hypot(x, y)
        rz = th.unwrap(d["crz"])
        if th.state == "free":
            if r < L.CAPTURE_R and tilt < L.CAPTURE_TILT_DEG and L.TRAVEL - 0.004 < z < L.TRAVEL + 0.0008:
                th.state, th.ref, th.phi = "captured", rz, 0.0
                th.anchor.anchor = rz
                th.events.append(dict(t=round(self.t, 3), ev="captured", r_mm=round(r * 1e3, 2), tilt=round(tilt, 2)))
        if th.state != "free":
            th.phi = -(rz - th.ref)
            if th.state == "captured":
                if th.phi < 0.0:  # turned back: rides on the thread crest
                    th.ref, th.phi = rz, 0.0
                if r > 1.5 * L.CAPTURE_R or tilt > 8.0 or z > L.TRAVEL + 0.002:
                    th.state = "free"
                    th.events.append(dict(t=round(self.t, 3), ev="lifted off", z_mm=round(z * 1e3, 2)))
                elif th.phi > math.radians(L.ENGAGE_DEG):
                    th.state = "engaged"
                    th.events.append(dict(t=round(self.t, 3), ev="engaged"))
            elif th.state == "engaged" and th.phi < math.radians(L.ENGAGE_DEG / 2):
                th.state = "captured"
                th.events.append(dict(t=round(self.t, 3), ev="disengaged (unscrewed)"))
        h = L.h_thread(max(th.phi, 0.0)) if th.state != "free" else 0.0
        mf = np.zeros(6)
        tg = np.zeros(6)
        rot_cap = 0.0
        if th.state == "captured":
            if z < h + 0.0003:
                mf[2] = F_AX
            tg[2] = h - Z0
            rot_cap = L.T_RUN
        elif th.state == "engaged":
            mf[:] = (F_LAT, F_LAT, F_AX, T_TILT, T_TILT, 0.0)
            tg[2] = h - Z0
            rot_cap = th.friction_cap()
        if th.state != "free":
            mf[5] = rot_cap
            # friction anchor kept in unwrapped coordinates; the drive target is given in the joint's raw coordinate
            raw_target = th.anchor.update(rz, rot_cap) + (d["crz"] - rz)
            # At a wrapped revolution boundary the friction anchor can lie a
            # fraction of a degree past PhysX's valid drive-target range.
            # Saturate locally so the friction drive cannot reverse direction.
            tg[5] = max(-math.tau, min(math.tau, raw_target))
        targets = self.art.get_dof_position_targets().clone()
        for k, n in enumerate(DOFS):
            targets[0, self.idof[n]] = float(tg[k])
        self.art.set_dof_position_targets(targets, self.ai)
        if self.mf_now is None or np.max(np.abs(mf - self.mf_now)) > 1e-4 * (1.0 + np.max(np.abs(mf))):
            m = self.art.get_dof_max_forces().clone()
            for k, n in enumerate(DOFS):
                m[0, self.idof[n]] = float(mf[k])
            self.art.set_dof_max_forces(m, self.ai_cpu)
            self.mf_now = mf.copy()
        if self.trial_active:
            pk = self.peaks
            if mf[2] > 0:
                pk["ax_n"] = max(pk["ax_n"], min(F_AX, K_AX * abs(h - z)))
            if th.state == "engaged":
                pk["lat_n"] = max(pk["lat_n"], min(F_LAT, K_LAT * r))
                pk["tilt_nm"] = max(pk["tilt_nm"], min(T_TILT, K_TILT * math.radians(tilt)))
            if rot_cap > 0:
                pk["rot_nm"] = max(pk["rot_nm"], min(rot_cap, K_ROT * abs(rz - th.anchor.anchor)))
            fm = self.cv.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if fm.size:
                nl = np.linalg.norm(fm, axis=-1)
                per = nl.sum(axis=-1)
                for i, n in enumerate(BODIES):
                    self.contact_step[n] = max(self.contact_step[n], float(per[i]))
                    self.contact_peak[n] = max(self.contact_peak[n], float(per[i]))
                    j = int(np.argmax(nl[i]))
                    if nl[i, j] > self.robot_link_peak.get(n, (0.0, ""))[0]:
                        self.robot_link_peak[n] = (float(nl[i, j]), "/".join(self.robot_links[j].split("/")[-2:]))
        th.t_peak = max(th.t_peak, rot_cap)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): bottle, cap (lying top-up on the bench) and funnel (in the
        neck) placed for the setup, at rest; all thread drives off."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        spec = L.setup_spec(setup)
        bp, br = L.bottle_pose(spec)
        rt = self.art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(bp + self.origin + np.array([0.0, 0.0, 0.0003]), dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(br), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.ai)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.ai)
        c = L.cap_chain_start(spec)
        q = torch.zeros_like(self.art.get_dof_positions())
        for k, n in enumerate(DOFS):
            q[0, self.idof[n]] = float(c[k] - (Z0 if n == "cz" else 0.0))
        q[0, self.idof["cz"]] += 0.0003
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q.clone(), self.ai)
        m = self.art.get_dof_max_forces().clone()
        m[:] = 0.0
        self.art.set_dof_max_forces(m, self.ai_cpu)
        self.mf_now = np.zeros(6)
        fpos, frot = L.funnel_start(spec)
        tf = self.view.get_transforms().clone()
        k = BODIES.index("funnel")
        tf[k, :3] = torch.tensor(np.asarray(fpos, float) + self.origin, dtype=tf.dtype)
        tf[k, 3:7] = torch.tensor(_quat(frot), dtype=tf.dtype)
        ids = torch.tensor([k], dtype=torch.int32, device=self.dev)
        self.view.set_transforms(tf, ids)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), ids)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def measure(self):
        poses = self.read()
        th = self.thread
        x, y, z, tilt = self.cap_rel()
        bp, br = poses["bottle"]
        fp, fr = poses["funnel"]
        lip = bp + br @ np.array([0.0, 0.0, L.H_LIP])
        f_com = fp + fr @ np.array([0.0, 0.0, L.F_COM_Z])
        stem_top = fp
        in_neck = float(np.linalg.norm(stem_top - (lip - br @ np.array([0.0, 0.0, L.F_REST_Z])))) < 0.03
        on_bench = (not in_neck) and f_com[2] > L.TOP - 0.01
        btilt = math.degrees(math.acos(max(-1.0, min(1.0, float(br[2, 2])))))
        velocity = self.view.get_velocities()[BODIES.index("funnel")].detach().cpu().numpy()
        rim_speed = np.linalg.norm(velocity[:3]) + (L.F_R_TOP + L.F_CONE_H) * np.linalg.norm(velocity[3:])
        return dict(poses=poses, funnel_still=bool(rim_speed < 0.01), gap_mm=(z * 1e3 if th.state != "free" else None), engaged=th.state == "engaged",
                    in_tray=L.in_tray(fp, fr), on_bench=on_bench, in_neck=in_neck, bottle_tilt=btilt,
                    cap_r_mm=math.hypot(x, y) * 1e3, cap_tilt=tilt, cap_z_mm=z * 1e3, f_com=f_com)

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
        th = self.thread
        level = self.scorer.step(dt, m["gap_mm"], m["engaged"], m["in_tray"], m["on_bench"], contact,
                                 m["bottle_tilt"], funnel_still=m["funnel_still"], phi_deg=math.degrees(th.phi) if th.state != "free" else 0.0)
        bp, br = m["poses"]["bottle"]
        spec = L.setup_spec(self.setup)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                thread=th.state,
                turns=round(th.phi / (2 * math.pi), 4) if th.state != "free" else None,
                cap_r_mm=round(m["cap_r_mm"], 3),
                cap_z_mm=round(m["cap_z_mm"], 3),
                cap_tilt_deg=round(m["cap_tilt"], 2),
                funnel_in_neck=m["in_neck"],
                funnel_com=[round(v, 4) for v in m["f_com"]],
                bottle_shift_mm=round(float(np.linalg.norm(bp[:2] - np.array([spec["bx"], spec["by"]]))) * 1e3, 2),
                bottle_yaw_deg=round(math.degrees(math.atan2(br[1, 0], br[0, 0])), 2),
                contact_n={n: round(v, 2) for n, v in self.contact_step.items()},
                contact_peak_n={n: round(v, 1) for n, v in self.contact_peak.items()},
                robot_link_peak={k: [round(v[0], 1), v[1]] for k, v in self.robot_link_peak.items()},
                peaks={k: round(v, 3) for k, v in self.peaks.items()},
                events=th.events[-6:],
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )
        self.contact_step = {n: 0.0 for n in BODIES}


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'bottle', 'cap', 'funnel'} plus the thread state, for the scripted demo (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    m = rt.measure()
    out = dict(m["poses"])
    th = rt.thread
    out.update(thread=th.state, phi=th.phi, cap_z=m["cap_z_mm"] / 1e3, cap_r=m["cap_r_mm"] / 1e3,
               cap_tilt=m["cap_tilt"], funnel_in_neck=m["in_neck"], funnel_in_tray=m["in_tray"],
               dofs=rt.dofs())
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_cap_patched", False):
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
                p, r = rt.read()["bottle"]
            else:
                p, r = L.bottle_pose(L.setup_spec(SETUP_BASE))
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
    scene._cap_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_cap_patched", False):
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
            moved = float(np.linalg.norm(rt.read()["bottle"][0][:2] - np.array([spec["bx"], spec["by"]]))) > 0.001
            if rt.setup != setup or moved or _RT["trials"] == 0:
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._cap_patched = True


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
