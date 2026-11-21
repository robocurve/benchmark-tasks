"""drivekit: a single-speed bicycle drivetrain on a bench stand, with a real roller chain.

Real sizes (a 14 to 16 inch child's bike with a freewheel): 1/2 in x 1/8 in chain (12.7 mm pitch, 7.75 mm rollers),
26-tooth chainring, 14-tooth cog, 80 mm cranks, 56 links. Every link is its own rigid body on a pin joint; the
chain is driven only by the teeth pushing its rollers. Nothing couples the crank to the cog except the chain.

Frame of the drivetrain (its origin is the bottom bracket's centre on the frame's centre line):
x along the chainstay toward the rear cog, y along the axles (the chain is on the +y side), z up.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import ObjectState


def _k(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


PITCH = 0.0127
ROLLER_R = 0.003875
N_RING, N_COG, N_LINKS = 26, 14, 56
R_RING = PITCH / (2 * math.sin(math.pi / N_RING))  # pitch radius, 52.7 mm
R_COG = PITCH / (2 * math.sin(math.pi / N_COG))  # 28.5 mm
SPAN_LINKS = 18
SLACK = _k("DRIVE_SLACK", 0.0006)  # m the rear axle sits closer than a taut chain needs
CENTRES = math.sqrt((SPAN_LINKS * PITCH) ** 2 - (R_RING - R_COG) ** 2) - SLACK  # bottom bracket to rear axle
AXLE_H = 0.0  # the cranks' axle is the origin of the drivetrain's own frame (the "rig")
BB_H = 0.21  # the cranks' axle above the table: a 14 inch bike standing on its wheels
WHEEL_R = 0.178  # a 14 x 1.75 wheel with its tyre
TILT = math.asin((BB_H - WHEEL_R) / CENTRES)  # the chainstay runs down to the rear axle (the wheel's centre): 8 degrees
YAW = math.radians(_k("DRIVE_YAW", 90.0))  # the bike across the table: its rear toward +y (the left arm), its chain
# side toward -x (the robot)
TYRE_W = 0.040
CHAINLINE = 0.042  # chain plane from the frame's centre line
CRANK = 0.080  # crank length
ARM_Y = 0.062  # crank arms' centre from the centre line
PEDAL = (0.050, 0.080, 0.018)  # along the crank's travel, along the axle, thick
PEDAL_Y = ARM_Y + 0.008 + PEDAL[1] / 2
TOOTH_T = 0.0024  # sprocket thickness (a 1/8 in chain is 3.3 mm between its inner plates)
PLATE_T = 0.0010
INNER_Y, OUTER_Y = 0.0023, 0.0034  # plate centres from the chain's middle: inner links, outer links
PLATE_H = 0.0080
PLATE_L = PITCH + 0.0068
LINK_MASS = _k("DRIVE_LINK_MASS", 0.0045)  # kg: a 1/8 in chain weighs about 4.5 g a link
CRANK_MASS, COG_MASS, PEDAL_MASS = 0.55, 0.20, 0.12
PIN_DAMP = _k("DRIVE_PIN_DAMP", 2.0e-5)  # N m s/rad at each chain pin (a dry, slightly stiff chain)
BB_DAMP = _k("DRIVE_BB_DAMP", 0.01)  # N m s/rad: the bottom bracket's drag
HUB_DAMP = _k("DRIVE_HUB_DAMP", 0.004)  # the freewheel's drag
PEDAL_DAMP = 0.001
FRICTION = _k("DRIVE_FRICTION", 0.15)  # steel on steel, oily
ROLLER_FRICTION = _k("DRIVE_ROLLER_FRICTION", 0.05)  # a real roller turns on its bushing: it rolls on the tooth
LINK_INERTIA = _k("DRIVE_LINK_INERTIA", 4.0e-7)  # kg m2 on every axis: about 6x a real link's, which keeps a loop
# of very light bodies on joints steady next to the 550 g cranks (the usual fix for small masses in a chain drive)
DEPEN = _k("DRIVE_DEPEN", 0.2)  # m/s: how fast overlapping parts are pushed apart (low: no kicks)
ITERS = int(_k("DRIVE_ITERS", 32))
SKIN = _k("DRIVE_SKIN", 0.0004)  # contact offset of the chain and teeth

STEEL = (0.62, 0.63, 0.65)
DARK = (0.10, 0.10, 0.11)
FRAME = (0.80, 0.16, 0.12)
RUBBER = (0.05, 0.05, 0.05)
LUBE_COLOR = (1.0, 0.72, 0.05)  # a wet link is drawn with a bright amber dab


def chain_pins() -> np.ndarray:
    """The 56 pin centres at the start, in the chain's plane: (x, z) from the bottom bracket's centre."""
    pts = []
    for k in range(14):  # round the back of the chainring, from its top to its bottom
        a = math.radians(90) + k * 2 * math.pi / N_RING
        pts.append((R_RING * math.cos(a), R_RING * math.sin(a)))
    cog = [(CENTRES + R_COG * math.cos(a), R_COG * math.sin(a))
           for a in (math.radians(-90) + j * 2 * math.pi / N_COG for j in range(8))]  # bottom to top round the cog
    lower = np.linspace(pts[-1], cog[0], SPAN_LINKS + 1)[1:-1]
    upper = np.linspace(cog[-1], pts[0], SPAN_LINKS + 1)[1:-1]
    out = np.array(pts + [tuple(p) for p in lower] + cog + [tuple(p) for p in upper])
    assert len(out) == N_LINKS
    return out


def ring_teeth() -> list[float]:
    return [math.radians(90) + (k + 0.5) * 2 * math.pi / N_RING for k in range(N_RING)]


def cog_teeth() -> list[float]:
    return [math.radians(-90) + (j + 0.5) * 2 * math.pi / N_COG for j in range(N_COG)]


@dataclass(frozen=True)
class DriveState(ObjectState):
    """``pins`` (56, 3) the chain's pin centres (world); ``quats`` (56, 4) xyzw; ``crank`` and ``rear`` the crank's
    and the cog's angles (rad, 0 at the start; positive is pedalling backwards); ``pedal`` (3,) the turned pedal's
    centre."""

    pins: np.ndarray | None = None
    quats: np.ndarray | None = None
    crank: float = 0.0
    rear: float = 0.0
    pedal: np.ndarray | None = None
    lubed: np.ndarray | None = None  # (56,) drops landed on each link this trial
    spills: int = 0  # drops that missed the chain
    drops: int = 0  # every drop released this trial
    crank_speed: float = 0.0  # rad/s, how fast the cranks are turning now


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


R_BIKE = _rz(YAW)  # the bike's frame (x toward its rear, y toward its chain side, z up) in the world
R_RIG = R_BIKE @ _ry(TILT)  # the drivetrain's frame: x from the cranks' axle to the rear axle


def to_world(origin, p_rig) -> np.ndarray:
    """A point (or points) given in the drivetrain's frame, from the cranks' axle, in the world. ``origin`` is the
    point on the table under the cranks' axle."""
    return np.asarray(origin, float) + np.array([0.0, 0.0, BB_H]) + np.asarray(p_rig, float) @ R_RIG.T


def to_bike(origin, p_world) -> np.ndarray:
    """World point(s) in the bike's frame (x along it toward the rear, y across it, z up from the table)."""
    return (np.asarray(p_world, float) - np.asarray(origin, float)) @ R_BIKE


def start_pins(origin) -> np.ndarray:
    p = chain_pins()
    return to_world(origin, np.stack([p[:, 0], np.full(len(p), CHAINLINE), p[:, 1]], axis=1))


def state(origin, pins=None, quats=None, crank=0.0, rear=0.0, pedal=None, vel=None) -> DriveState:
    o = np.asarray(origin, float)
    pins = start_pins(o) if pins is None else np.asarray(pins, float)
    quats = np.tile([0, 0, 0, 1.0], (len(pins), 1)) if quats is None else np.asarray(quats, float)
    if pedal is None:
        pedal = to_world(o, [0.0, PEDAL_Y, -CRANK])
    lin = np.zeros(3) if vel is None else np.asarray(vel, float)
    return DriveState(pos=o + np.array([0.0, 0.0, BB_H]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=lin,
                      ang_vel=np.zeros(3), low=pins.min(0) - 0.12, high=pins.max(0) + 0.12, pins=pins, quats=quats,
                      crank=float(crank), rear=float(rear), pedal=np.asarray(pedal, float),
                      lubed=LUBE["lubed"].copy(), spills=int(LUBE["spills"]), drops=int(LUBE["drops"]),
                      crank_speed=float(LUBE.get("crank_speed", 0.0)))


SIZE = (0.46, 1.02, 0.60)  # the whole bike's bounding box (checks only)


# -- authoring -----------------------------------------------------------------------------------------------------------


def _q_from_matrix(R):
    """(w, x, y, z) of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def _q_z_to(d):
    """(w, x, y, z) turning the z axis onto the unit vector ``d``."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    c = float(z @ d)
    if c < -0.999999:
        return (0.0, 1.0, 0.0, 0.0)
    ax = np.cross(z, d)
    w = 1.0 + c
    n = math.sqrt(w * w + ax @ ax)
    return (w / n, ax[0] / n, ax[1] / n, ax[2] / n)


def _quat_y(angle: float):
    """(w, x, y, z) of a turn by ``angle`` about y."""
    return (math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)


def build(stage, root: str) -> None:
    """Author the stand, the cranks, the cog and the chain under ``root`` (an Xform at the drivetrain's origin)."""
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    outer = root
    root = f"{outer}/rig"  # the drivetrain's own frame: at the cranks' axle, turned with the bike and tilted down its chainstay
    rig = UsdGeom.Xform.Define(stage, root)
    rig.AddTranslateOp().Set(Gf.Vec3d(0, 0, BB_H))
    rq = _q_from_matrix(R_RIG)
    rig.AddOrientOp().Set(Gf.Quatf(*(float(v) for v in rq)))

    def look(name, color, metallic=0.0, rough=0.5):
        m = UsdShade.Material.Define(stage, f"{root}/looks/{name}")
        sh = UsdShade.Shader.Define(stage, f"{root}/looks/{name}/surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m

    looks = {"steel": look("steel", STEEL, 0.9, 0.35), "dark": look("dark", DARK, 0.6, 0.5),
             "frame": look("frame", FRAME, 0.1, 0.4), "rubber": look("rubber", RUBBER, 0.0, 0.9),
             "lube": look("lube", LUBE_COLOR, 0.0, 0.15)}

    def phys(name, friction):
        m = UsdShade.Material.Define(stage, f"{root}/looks/{name}")
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(friction)
        api.CreateDynamicFrictionAttr(friction)
        api.CreateRestitutionAttr(0.0)
        return m

    metal, grip, rolling = phys("p_metal", FRICTION), phys("p_grip", 1.0), phys("p_roller", ROLLER_FRICTION)

    def collide(prim, mat, skin):
        UsdPhysics.CollisionAPI.Apply(prim)
        c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        c.CreateContactOffsetAttr(skin)
        c.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    def box(path, at, size, color, mat=metal, quat=None, skin=SKIN, solid=True):
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
        if quat is not None:
            g.AddOrientOp().Set(Gf.Quatf(*(float(v) for v in quat)))
        g.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in size)))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(looks[color])
        if solid:
            collide(g.GetPrim(), mat, skin)

    def cyl(path, at, radius, length, color, mat=metal, skin=SKIN, solid=True):
        g = UsdGeom.Cylinder.Define(stage, path)  # along y
        g.CreateAxisAttr("Y")
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(length))
        g.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(looks[color])
        if solid:
            collide(g.GetPrim(), mat, skin)

    def ball(path, at, radius, color, mat=metal, skin=SKIN):
        g = UsdGeom.Sphere.Define(stage, path)
        g.CreateRadiusAttr(float(radius))
        g.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(looks[color])
        collide(g.GetPrim(), mat, skin)

    def body(path, at, mass, quat=None, iters=ITERS, inertia=None):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
        xf.AddOrientOp().Set(Gf.Quatf(*(quat if quat is not None else (1.0, 0.0, 0.0, 0.0))))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
            m.CreateCenterOfMassAttr(Gf.Vec3f(PITCH / 2, 0, 0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(iters)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateMaxDepenetrationVelocityAttr(DEPEN)
        rb.CreateSleepThresholdAttr(0.0)
        return prim

    def hinge(path, b0, b1, at0, at1, damping, quat1=None):
        """A pin about y between ``b0`` (None: the stand, at0 then in the root's frame) and ``b1``."""
        j = UsdPhysics.RevoluteJoint.Define(stage, path)
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0 if b0 else f"{root}/stand")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in at0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(float(v) for v in at1)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(*(quat1 if quat1 is not None else (1.0, 0.0, 0.0, 0.0))))
        j.CreateCollisionEnabledAttr(False)
        dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        dr.CreateTypeAttr("force")
        dr.CreateStiffnessAttr(0.0)
        dr.CreateDampingAttr(float(damping * math.pi / 180))  # USD angular damping is per degree/s
        dr.CreateMaxForceAttr(1.0e6)
        return j

    # -- the stand: a foot on the table, a post under the bottom bracket, the chainstay, a post at the rear axle.
    # A kinematic body (it never moves) so the joints have a body to hang from.
    stand = body(f"{root}/stand", (0, 0, 0), 5.0)
    UsdPhysics.RigidBodyAPI(stand).CreateKinematicEnabledAttr(True)
    cyl(f"{root}/stand/bb_shell", (0, 0, 0), 0.020, 0.068, "frame", grip, skin=0.002)
    cyl(f"{root}/stand/dropout", (CENTRES, 0.0, 0), 0.011, 0.050, "frame", grip, skin=0.002)

    # -- the cranks: axle, chainring, two arms, one body
    def sprocket(base, n_teeth, r_pitch, teeth, y):
        root_r = r_pitch - ROLLER_R
        cyl(f"{base}/root", (0, y, 0), root_r, TOOTH_T, "steel")
        for i, a in enumerate(teeth):
            c, s = math.cos(a), math.sin(a)
            q = _quat_y(-a)  # local x along the radius
            for tag, r0, r1, w in (("a", root_r - 0.001, r_pitch + 0.0008, 0.0044), ("b", r_pitch + 0.0008, r_pitch + 0.0034, 0.0022)):
                rm = (r0 + r1) / 2
                box(f"{base}/tooth_{i:02d}{tag}", (rm * c, y, rm * s), (r1 - r0, TOOTH_T, w), "steel", quat=q)

    crank = f"{root}/crank"
    body(crank, (0, 0, AXLE_H), CRANK_MASS)
    cyl(f"{crank}/axle", (0, 0, 0), 0.008, 2 * ARM_Y, "steel", solid=False)
    sprocket(crank, N_RING, R_RING, ring_teeth(), CHAINLINE)
    for side, ys, zs in (("turned", 1.0, -1.0), ("far", -1.0, 1.0)):  # the turned pedal is the chain side's (the robot's
        # side): its arm points down at the start, the other up
        box(f"{crank}/arm_{side}", (0, ys * ARM_Y, zs * CRANK / 2), (0.024, 0.012, CRANK + 0.024), "dark", grip, skin=0.002)
    hinge(f"{root}/joints/bb", None, crank, (0, 0, AXLE_H), (0, 0, 0), BB_DAMP)

    for side, ys, zs in (("turned", 1.0, -1.0), ("far", -1.0, 1.0)):
        p = f"{root}/pedal_{side}"
        body(p, (0, ys * PEDAL_Y, zs * CRANK), PEDAL_MASS)
        box(f"{p}/block", (0, 0, 0), PEDAL, "rubber", grip, skin=0.002)
        hinge(f"{root}/joints/pedal_{side}", crank, p, (0, ys * PEDAL_Y, zs * CRANK), (0, 0, 0), PEDAL_DAMP)

    cog = f"{root}/cog"
    body(cog, (CENTRES, 0, AXLE_H), COG_MASS)
    cyl(f"{cog}/hub", (0, CHAINLINE / 2 + 0.010, 0), 0.014, CHAINLINE - 0.024, "steel", solid=False)
    sprocket(cog, N_COG, R_COG, cog_teeth(), CHAINLINE)
    hinge(f"{root}/joints/hub", None, cog, (CENTRES, 0, AXLE_H), (0, 0, 0), HUB_DAMP)

    # -- the chain: link i runs from pin i to pin i+1; its body's x axis points along it
    pins = chain_pins()
    n = len(pins)
    angs = []
    for i in range(n):
        d = pins[(i + 1) % n] - pins[i]
        angs.append(math.atan2(d[1], d[0]))  # the link's direction in (x, z): a turn of -angle about y
    links = []
    for i in range(n):
        p = f"{root}/link_{i:02d}"
        body(p, (pins[i][0], CHAINLINE, AXLE_H + pins[i][1]), LINK_MASS, quat=_quat_y(-angs[i]), inertia=LINK_INERTIA)
        ball(f"{p}/roller", (0, 0, 0), ROLLER_R, "steel", rolling)
        yy = OUTER_Y if i % 2 == 0 else INNER_Y
        for tag, sgn in (("l", 1.0), ("r", -1.0)):
            box(f"{p}/plate_{tag}", (PITCH / 2, sgn * yy, 0), (PLATE_L, PLATE_T, PLATE_H), "steel" if i % 2 else "dark")
        dab = UsdGeom.Sphere.Define(stage, f"{p}/lube")  # drawn only: shown once a drop has landed on this link
        dab.CreateRadiusAttr(0.0052)
        dab.AddTranslateOp().Set(Gf.Vec3d(PITCH / 2, 0, 0))
        UsdShade.MaterialBindingAPI.Apply(dab.GetPrim()).Bind(looks["lube"])
        UsdGeom.Imageable(dab.GetPrim()).MakeInvisible()
        links.append(p)
    for i in range(n):
        j = (i + 1) % n
        rel = angs[j] - angs[i]  # link j's frame seen from link i: a turn of -rel about y
        hinge(f"{root}/joints/pin_{i:02d}", links[i], links[j], (PITCH, 0, 0), (0, 0, 0), PIN_DAMP, quat1=_quat_y(rel))
    for i in range(n):  # neighbours two apart never touch: skip the pair
        f = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(links[i]))
        f.CreateFilteredPairsRel().AddTarget(Sdf.Path(links[(i + 2) % n]))

    # -- the rest of the bike, fixed: frame, fork, wheels, bars, saddle and a wheel stand. Its frame: x toward the rear,
    # y toward the chain side, z up from the table, the cranks' axle above its origin. Fixed colliders (no body).
    bike = f"{outer}/bike"
    bx = UsdGeom.Xform.Define(stage, bike)
    bx.AddOrientOp().Set(Gf.Quatf(*(float(v) for v in _q_from_matrix(R_BIKE))))
    n_tube = [0]

    def tube(p, q, radius, color, solid=True):
        p, q = np.asarray(p, float), np.asarray(q, float)
        g = UsdGeom.Cylinder.Define(stage, f"{bike}/tube_{n_tube[0]:02d}")
        n_tube[0] += 1
        g.CreateAxisAttr("Z")
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(np.linalg.norm(q - p)))
        g.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in (p + q) / 2)))
        g.AddOrientOp().Set(Gf.Quatf(*(float(v) for v in _q_z_to(q - p))))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(looks[color])
        if solid:
            collide(g.GetPrim(), grip, 0.002)

    rear = np.array([CENTRES * math.cos(TILT), 0.0, WHEEL_R])
    bb = np.array([0.0, 0.0, BB_H])
    seat, head_top, head_low = np.array([0.075, 0.0, 0.40]), np.array([-0.335, 0.0, 0.44]), np.array([-0.322, 0.0, 0.36])
    front = np.array([-0.45, 0.0, WHEEL_R])
    side = np.array([0.0, 0.030, 0.0])
    tube(bb, seat, 0.014, "frame"); tube(seat, head_top, 0.014, "frame"); tube(bb + [-0.012, 0, 0.012], head_low, 0.016, "frame")
    tube(head_low, head_top + [-0.004, 0, 0.03], 0.017, "frame")
    tube(seat, seat + [0.018, 0, 0.05], 0.011, "steel")  # seat post
    tube(head_top + [-0.004, 0, 0.03], [-0.345, 0.0, 0.55], 0.010, "steel")  # stem
    tube([-0.345, -0.21, 0.55], [-0.345, 0.21, 0.55], 0.010, "dark")  # handlebars
    for sgn in (1.0, -1.0):
        tube(bb + sgn * side * 0.9 + [0.02, 0, -0.004], rear + sgn * side, 0.006, "frame")  # chain stays
        tube(seat + sgn * side * 0.5, rear + sgn * side, 0.006, "frame")  # seat stays
        tube(head_low + sgn * side, front + sgn * side, 0.008, "frame")  # fork blades
        tube([-0.345, sgn * 0.21, 0.55], [-0.345, sgn * 0.13, 0.55], 0.014, "rubber")  # grips
    box(f"{bike}/saddle", seat + [0.03, 0, 0.062], (0.20, 0.10, 0.025), "rubber", grip, skin=0.002)
    for name, c in (("rear", rear), ("front", front)):
        cyl(f"{bike}/tyre_{name}", c, WHEEL_R, TYRE_W, "rubber", grip, skin=0.002)
        cyl(f"{bike}/rim_{name}", c, WHEEL_R - 0.030, TYRE_W + 0.002, "steel", solid=False)
        cyl(f"{bike}/hub_{name}", c, 0.016, 0.075, "dark", solid=False)
    for name, x in (("rear", rear[0]), ("front", front[0])):  # a wheel stand's two cheeks at each wheel
        for sgn in (1.0, -1.0):
            box(f"{bike}/stand_{name}_{'l' if sgn > 0 else 'r'}", (x, sgn * (TYRE_W / 2 + 0.014), 0.035), (0.11, 0.022, 0.07), "dark",
                grip, skin=0.002)
        box(f"{bike}/stand_{name}_foot", (x, 0.0, 0.004), (0.13, 0.16, 0.008), "dark", grip, skin=0.002)


# -- reading ------------------------------------------------------------------------------------------------------------

_VIEWS: dict = {}
_TURNS: dict = {}
_HOME: dict = {}


def _copy(t):
    if hasattr(t, "clone"):
        return t.clone()
    import warp as wp

    return wp.clone(t)


def _indices(t):
    n = int(t.shape[0])
    if hasattr(t, "clone"):
        import torch

        return torch.arange(n, device=t.device, dtype=torch.int32)
    import warp as wp

    return wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device=t.device)


def _zeros_like_vel(t):
    n = int(t.shape[0])
    if hasattr(t, "clone"):
        import torch

        return torch.zeros((n, 6), device=t.device, dtype=t.dtype)
    import warp as wp

    return wp.zeros((n, 6), dtype=wp.float32, device=t.device)


def _np(t):
    t = getattr(t, "torch", t)
    if hasattr(t, "detach"):
        return t.detach().cpu().numpy().astype(float)
    if hasattr(t, "numpy"):
        return np.asarray(t.numpy(), float)
    return np.asarray(t, float)


def _turn_about_own_y(q0, q) -> float:
    """How far a body on a pin along its own y has turned from ``q0`` to ``q`` (xyzw), -pi..pi."""
    x0, y0, z0, w0 = q0
    x, y, z, w = q
    # conj(q0) * q, its y and w parts
    ry = w0 * y - y0 * w - z0 * x + x0 * z
    rw = w0 * w + x0 * x + y0 * y + z0 * z
    return 2 * math.atan2(ry, rw)


def _angle_y(q) -> float:
    """A body's turn about y from its xyzw quaternion (a body on a y pin)."""
    return 2 * math.atan2(q[1], q[3])


def read(environment, origin, prim_root: str) -> DriveState:
    base = environment.unwrapped
    key = (id(base), prim_root)
    if key not in _VIEWS:
        view = base.sim.physics_manager.get_physics_sim_view()
        _VIEWS[key] = {n: view.create_rigid_body_view(f"{prim_root}/rig/{pat}")
                       for n, pat in (("links", "link_*"), ("crank", "crank"), ("cog", "cog"), ("pedal", "pedal_turned"))}
        _VIEWS[key]["pedal2"] = view.create_rigid_body_view(f"{prim_root}/rig/pedal_far")
        _TURNS[key] = {"crank": [0.0, 0.0], "cog": [0.0, 0.0]}
        _HOME[key] = {n: _copy(b.get_transforms()) for n, b in _VIEWS[key].items()}  # where every part starts
    v = _VIEWS[key]
    env0 = _np(base.scene.env_origins[0])
    tf = _np(v["links"].get_transforms()).reshape(-1, 7)
    vel = _np(v["links"].get_velocities()).reshape(-1, 6)
    out = {}
    for name in ("crank", "cog"):  # unwrap: count whole turns
        a = _turn_about_own_y(_np(_HOME[key][name]).reshape(-1, 7)[0, 3:7], _np(v[name].get_transforms()).reshape(-1, 7)[0, 3:7])
        last, total = _TURNS[key][name]
        total += (a - last + math.pi) % (2 * math.pi) - math.pi
        _TURNS[key][name] = [a, total]
        out[name] = total
    pedal = _np(v["pedal"].get_transforms()).reshape(-1, 7)[0, :3] - env0
    LUBE["crank_speed"] = float(_np(v["crank"].get_velocities()).reshape(-1, 6)[0, 3:] @ R_RIG[:, 1])  # about its axle
    _drip(environment, origin, tf[:, :3] - env0, prim_root)
    fastest = vel[np.argmax(np.linalg.norm(vel[:, :3], axis=1)), :3]
    return state(origin, tf[:, :3] - env0, tf[:, 3:7], out["crank"], out["cog"], pedal, fastest)


def reset_turns() -> None:
    """At every reset: put every part of the drivetrain back where it started, at rest (labkit resets only its own
    kinds), and zero the turn counters."""
    for key, views in _VIEWS.items():
        for name, view in views.items():
            home = _HOME[key][name]
            try:
                idx = _indices(home)
                view.set_transforms(home, idx)
                view.set_velocities(_zeros_like_vel(home), idx)
            except Exception as e:  # the stream must not die on a reset: say so and carry on
                print(f"drivekit: could not reset {name}: {type(e).__name__}: {e}", flush=True)
    for t in _TURNS.values():
        for name in t:
            t[name] = [0.0, 0.0]
    LUBE.update(lubed=np.zeros(N_LINKS, int), spills=0, drops=0, squeezed_s=0.0, last_t=None, hide=True)


# -- the lube bottle and its drops -----------------------------------------------------------------------------------------
# A 4 oz (118 ml) squeeze bottle: 38 mm across, 12 cm tall with its nozzle, about 130 g full. No liquid is simulated.
# Its wall is four sprung pads round a rigid core (two opposite pairs), so a grip squeezes it; a drop leaves the
# nozzle only while the bottle points at least 20 degrees below level and one pair of pads is pressed in past
# SQUEEZE_AT each (about 4.5 N a side). A drop falls straight down and wets the first chain link under the tip.
BOTTLE_R, CORE_R, BOTTLE_H = 0.019, 0.013, 0.120
RIM_R = 0.0138  # the rigid base and shoulder sit inside the wall's full travel: a finger always lands on the wall (20 Nov:
# with rims as wide as the wall one finger rested on a rim and only the other side of the wall gave)
PAD_T, PAD_W, PAD_H, PAD_Z = 0.004, 0.024, 0.062, 0.043
PAD_TRAVEL = 0.005
PAD_K = _k("LUBE_PAD_K", 6000.0)  # N/m: the wall's spring. 20 Nov: at 1,500 any keyboard grip (up to 35 N a finger, in
# steps of about 12 N) pressed the wall to its stop, so the bottle dripped whenever it was tipped. At 6,000 one notch of
# grip past contact holds it dry (about 12 N, wall in 1 mm) and a held grip squeezes it (18 N and up)
PAD_PRELOAD = 0.001  # m: the pads rest against their outer stop with 6 N, so a light hold doesn't move them
SQUEEZE_AT = _k("LUBE_SQUEEZE_AT", 0.003)  # m each side of the wall is pressed in: (1 + 3) mm x 6,000 N/m = 24 N. A firm
# carrying grip (18 N, the wall in 2 mm) stays dry
POINT_DOWN = math.sin(math.radians(_k("LUBE_POINT_DOWN_DEG", 20.0)))  # a squeezed, nearly full bottle drips once its
# nozzle points 20 degrees below level
DROP_EVERY = _k("LUBE_DROP_EVERY", 0.45)  # s of squeezing per drop
DROP_HALF_WIDTH = 0.006  # m either side of the chain's middle that a falling drop still lands on a link
BOTTLE_GREEN, CAP_RED = (0.10, 0.45, 0.25), (0.80, 0.10, 0.08)
BOTTLE_SIZE = (0.040, 0.040, 0.124)

LUBE = {"lubed": np.zeros(N_LINKS, int), "spills": 0, "drops": 0, "squeezed_s": 0.0, "last_t": None, "hide": False,
        "tip": None, "axis": None, "squeeze": 0.0, "flowing": False}


@dataclass(frozen=True)
class BottleState(ObjectState):
    """``tip`` the nozzle's tip (world); ``squeeze`` (m) how far the better pair of wall pads is pressed in;
    ``flowing`` whether a drop can leave it now."""

    tip: np.ndarray | None = None
    squeeze: float = 0.0
    flowing: bool = False


def bottle_state(pos, quat=(0, 0, 0, 1.0), squeeze=0.0, lin=(0, 0, 0), ang=(0, 0, 0)) -> BottleState:
    pos, quat = np.asarray(pos, float), np.asarray(quat, float)
    R = _xyzw_to_matrix(quat)
    tip, axis = pos + R @ np.array([0, 0, BOTTLE_H]), R[:, 2]
    flowing = bool(axis[2] <= -POINT_DOWN and squeeze >= SQUEEZE_AT)
    LUBE.update(tip=tip, axis=axis, squeeze=float(squeeze), flowing=flowing)
    mid = pos + R @ np.array([0, 0, BOTTLE_H / 2])
    return BottleState(pos=pos, quat_xyzw=quat, lin_vel=np.asarray(lin, float), ang_vel=np.asarray(ang, float),
                       low=mid - BOTTLE_H / 2, high=mid + BOTTLE_H / 2, tip=tip, squeeze=float(squeeze), flowing=flowing)


def _xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def build_bottle(stage, root: str) -> None:
    """Author the bottle under ``root`` (an Xform at the centre of its base, standing upright)."""
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    def look(name, color, rough=0.4):
        m = UsdShade.Material.Define(stage, f"{root}/looks/{name}")
        sh = UsdShade.Shader.Define(stage, f"{root}/looks/{name}/surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m

    green, red = look("green", BOTTLE_GREEN), look("red", CAP_RED)
    pm = UsdShade.Material.Define(stage, f"{root}/looks/p_wall")
    api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    api.CreateStaticFrictionAttr(1.0)
    api.CreateDynamicFrictionAttr(0.9)
    api.CreateRestitutionAttr(0.0)

    def collide(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        c.CreateContactOffsetAttr(0.002)
        c.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")

    def body(path, at, mass, yaw=0.0):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
        xf.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(16)
        rb.CreateSolverVelocityIterationCountAttr(2)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateSleepThresholdAttr(0.0)
        return prim

    def cyl(path, z, radius, height, mat):
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateAxisAttr("Z")
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(height))
        g.AddTranslateOp().Set(Gf.Vec3d(0, 0, float(z)))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
        collide(g.GetPrim())

    core = f"{root}/core"
    body(core, (0, 0, 0), 0.11)
    cyl(f"{core}/base", 0.005, RIM_R, 0.010, green)
    cyl(f"{core}/inner", 0.0425, CORE_R, 0.085, green)
    cyl(f"{core}/shoulder", 0.080, RIM_R, 0.010, green)
    cyl(f"{core}/neck", 0.091, 0.011, 0.012, red)
    cyl(f"{core}/nozzle", 0.1085, 0.0045, 0.023, red)
    r0 = BOTTLE_R - PAD_T / 2
    for i in range(4):
        a = i * math.pi / 2
        p = f"{root}/pad_{i}"
        body(p, (r0 * math.cos(a), r0 * math.sin(a), PAD_Z), 0.005, yaw=a)  # the pad's x points outward
        g = UsdGeom.Cube.Define(stage, f"{p}/wall")
        g.CreateSizeAttr(1.0)
        g.AddScaleOp().Set(Gf.Vec3f(PAD_T, PAD_W, PAD_H))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(green)
        collide(g.GetPrim())
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/joints/pad_{i}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(core)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(p)])
        j.CreateLocalPos0Attr(Gf.Vec3f(r0 * math.cos(a), r0 * math.sin(a), PAD_Z))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-PAD_TRAVEL)
        j.CreateUpperLimitAttr(0.0)
        j.CreateCollisionEnabledAttr(False)
        dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        dr.CreateTypeAttr("force")
        dr.CreateStiffnessAttr(PAD_K)
        dr.CreateDampingAttr(4.0)
        dr.CreateTargetPositionAttr(PAD_PRELOAD)
        dr.CreateMaxForceAttr(1.0e3)
    for i in range(4):  # neighbouring pads never press on each other
        f = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{root}/pad_{i}"))
        f.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{root}/pad_{(i + 1) % 4}"))


def read_bottle(environment, prim_root: str) -> BottleState:
    base = environment.unwrapped
    key = (id(base), prim_root)
    if key not in _VIEWS:
        view = base.sim.physics_manager.get_physics_sim_view()
        _VIEWS[key] = {"core": view.create_rigid_body_view(f"{prim_root}/core"),
                       "pads": view.create_rigid_body_view(f"{prim_root}/pad_*")}
        _HOME[key] = {n: _copy(b.get_transforms()) for n, b in _VIEWS[key].items()}
    v = _VIEWS[key]
    env0 = _np(base.scene.env_origins[0])
    core = _np(v["core"].get_transforms()).reshape(-1, 7)[0]
    vel = _np(v["core"].get_velocities()).reshape(-1, 6)[0]
    pads = _np(v["pads"].get_transforms()).reshape(-1, 7)[:, :3]
    R = _xyzw_to_matrix(core[3:7])
    local = (pads - core[:3]) @ R  # each pad's centre in the bottle's frame
    pressed = (BOTTLE_R - PAD_T / 2) - np.linalg.norm(local[:, :2], axis=1)  # m each pad is in from its rest
    squeeze = float(np.clip(pressed, 0.0, None).sum()) / 2  # the whole wall's give, per side: a bottle that has turned
    # in the hand has its fingers across two pads a side, and still reads the same
    if os.environ.get("LUBE_LOG"):
        from labkit import trial_time

        t = trial_time()
        if int(t * 20 + 0.5) % 10 == 0:
            print(f"drivekit: bottle t={t:.1f} pos={np.round(core[:3] - env0, 3).tolist()} axis_z={R[2, 2]:.2f} "
                  f"pads_mm={np.round(pressed * 1000, 2).tolist()} squeeze_mm={squeeze * 1000:.2f} "
                  f"tip={np.round(core[:3] - env0 + R @ np.array([0, 0, BOTTLE_H]), 3).tolist()}", flush=True)
    return bottle_state(core[:3] - env0, core[3:7], squeeze, vel[:3], vel[3:])


def land(tip, pins):
    """Which link a drop falling straight down from ``tip`` lands on (its index), or None: a miss."""
    best, best_z = None, -math.inf
    n = len(pins)
    for i in range(n):
        a, b = pins[i], pins[(i + 1) % n]
        if abs(tip[1] - (a[1] + b[1]) / 2) > DROP_HALF_WIDTH:
            continue
        lo, hi = min(a[0], b[0]), max(a[0], b[0])
        if not (lo - 0.0005 <= tip[0] <= hi + 0.0005):
            continue
        f = 0.5 if hi - lo < 1e-9 else (tip[0] - a[0]) / (b[0] - a[0])
        z = a[2] + min(1.0, max(0.0, f)) * (b[2] - a[2])
        if z < tip[2] and z > best_z:
            best, best_z = i, z
    return best


def drip(t: float, pins, origin=(0.0, 0.0, 0.0)) -> list:
    """Advance the drops to trial time ``t``: one drop per DROP_EVERY s of squeezing with the bottle pointing down.
    Returns the links newly wetted."""
    last, LUBE["last_t"] = LUBE["last_t"], t
    if last is None or t <= last or not LUBE["flowing"] or LUBE["tip"] is None:
        return []
    LUBE["squeezed_s"] += t - last
    new = []
    while LUBE["squeezed_s"] >= DROP_EVERY:
        LUBE["squeezed_s"] -= DROP_EVERY
        LUBE["drops"] += 1
        i = land(to_bike(origin, LUBE["tip"]), to_bike(origin, pins))  # in the bike's frame: x along the chain
        print(f"drivekit: drop {LUBE['drops']} at t={t:.2f}: " + ("missed the chain" if i is None else f"link {i}"), flush=True)
        if i is None:
            LUBE["spills"] += 1
        else:
            if LUBE["lubed"][i] == 0:
                new.append(i)
            LUBE["lubed"][i] += 1
    return new


def _drip(environment, origin, pins, prim_root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from labkit import trial_time
    from pxr import UsdGeom

    stage = get_current_stage()
    if LUBE["hide"]:  # a new trial: every link dry again
        LUBE["hide"] = False
        for i in range(N_LINKS):
            prim = stage.GetPrimAtPath(f"{prim_root}/rig/link_{i:02d}/lube")
            if prim.IsValid():
                UsdGeom.Imageable(prim).MakeInvisible()
    for i in drip(trial_time(), pins, origin):
        prim = stage.GetPrimAtPath(f"{prim_root}/rig/link_{i:02d}/lube")
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeVisible()
