"""shoekit: a practice lower leg in an open-toe post-op shoe, as one PhysX articulation, for a yam_arena task.

Tree (fixed base): stand (fixed to the world at the knee pivot) -> knee (sideways swing, sprung and damped) -> leg
(flexion, sprung and damped; the shin, the foot and the shoe's sole, toe, heel, medial quarter and the closed rear part
of the lateral quarter) -> the lateral wing of the opening, a flap on a sprung hinge along the shoe (it carries the
strap's plastic D-ring), and the shoe's instep strap, a chain of thin plates on sprung hinges, anchored on the medial
quarter. The code keeps per-strap lists (tag "a"), so the model is written for any number of straps.

Hook and loop: the strap's tail carries the hook patch and its first run the loop patch, both on the strap's outer face.
For every (hook plate, loop plate) pair a point joint (a D6 with free rotations and stiff translational drives) is
authored disabled, outside the articulation. :class:`Fastener` switches one on, at the pose the two plates have at
that moment, only when the contact force PhysX reports between those two plates reaches the press threshold with the
faces opposed; it switches it off again when the joint's spring force exceeds the peel (normal) or shear limit.
The pair joints act at plate centres and leave rotation free, so they do not add stiffness to the strap's hinges.

Pure Python at import time; everything that needs Isaac Sim is imported lazily. Importing this module hooks
``yam_arena.scene`` without changing yam_arena.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

# -- geometry (shoe frame: origin at the middle of the sole's underside, x toward the heel, y lateral (outer side of
# this right shoe), z up; metres) ------------------------------------------------------------------------------------

FOOT_YZ = ((-0.050, 0.030), (-0.047, 0.058), (-0.029, 0.079), (-0.008, 0.088), (0.002, 0.087), (0.012, 0.074),
           (0.030, 0.053), (0.044, 0.031))  # cross-section of foot + tongue + medial quarter under the straps
FOOT_X = (-0.100, 0.050)
MEDIAL_EDGE = np.array((-0.008, 0.088))  # medial eyestay edge (y, z)
HINGE = np.array((0.048, 0.032))  # lateral flap hinge line (y, z)
EDGE_CLOSED = np.array((0.000, 0.088))  # lateral flap's edge when fully closed
FLAP_T = 0.003
RING_ALONG, RING_OUT = 0.030, 0.0065  # loop centre: along the flap from the hinge, and out from its surface
RING_IN_W, RING_IN_H, RING_BAR = 0.029, 0.0075, 0.0040  # loop opening (across the strap, through it) and bar size
ANCHOR = np.array((-0.0515, 0.050))  # strap anchor on the medial quarter (y, z)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


FLAP_LEN = float(np.linalg.norm(EDGE_CLOSED - HINGE))
FLAP_D = _unit(EDGE_CLOSED - HINGE)  # hinge -> edge, closed
FLAP_N = np.array((FLAP_D[1], -FLAP_D[0]))  # outward normal of the closed flap


def rot2(a: float) -> np.ndarray:
    """Rotation of (y, z) about +x by a (positive closes the lateral flap)."""
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s), (s, c)))


@dataclass(frozen=True)
class StrapGeom:
    x: float  # strap centre along the shoe
    flap_x: tuple[float, float]


@dataclass(frozen=True)
class Shoe(Box):
    """``pos`` is the shoe frame's origin in the world; the shoe frame is the world frame shifted (no rotation)."""

    straps: tuple[StrapGeom, ...] = (StrapGeom(-0.040, (-0.066, -0.014)),)  # one strap over the instep
    n_links: int = 34
    link_len: float = 0.0056
    strap_w: float = 0.025
    strap_t: float = 0.0018
    link_mass: float = 0.0004
    bend_k: float = 0.004  # N m/rad per hinge (spring toward the strap's start shape)
    bend_c: float = 2e-4
    armature: float = 1e-5
    open_deg: float = 26.0  # flaps start splayed this far open
    flap_k: float = 0.35  # N m/rad, toward the splayed (relaxed) pose
    flap_c: float = 0.01
    flap_mass: float = 0.015
    knee_x: float = 0.20  # knee pivot (shoe frame): behind the ankle, the shin sloping back as when seated
    knee_z: float = 0.53
    ankle: tuple[float, float, float] = (0.085, 0.0, 0.15)  # the shin's lower end (shoe frame)
    shin_r: float = 0.040
    leg_mass: float = 3.5
    side_k: float = 5.0  # N m/rad: sideways swing spring at the pivot
    side_c: float = 3.0
    flex_k: float = 18.0
    flex_c: float = 3.0
    hook: tuple[int, int] = (21, 34)  # plates with the hook patch (half-open range)
    loop: tuple[int, int] = (6, 22)  # plates with the loop patch
    pair_k: float = 3000.0  # N/m, pair joint stiffness
    pair_c: float = 2.0
    friction: float = 0.3
    sole_friction: float = 0.4  # rubber outsole on the table
    tab_links: int = 3  # the last plates form a rubberised pull tab
    bead_t: float = 0.008  # the bead across the tab's end
    stiff_from: int = 31  # hinges from here to the tab are stiffer (the hook tape's backing)
    hook_bend_k: float = 0.006
    tab_bend_k: float = 0.08
    tab_t: float = 0.004
    tab_mass: float = 0.0004  # kg per tab plate (moulded rubber)
    tab_friction: float = 1.0
    ring_friction: float = 0.15  # moulded acetal loop on nylon webbing
    tab_torsion: float = 0.004  # m, torsional friction patch on the pull tab (resists turning in a pinch)
    contact_offset: float = 0.0006
    solver_pos_iters: int = 32
    tab_drive: bool = True  # a world drive on each strap's last plate: the scorer's spring scale, off until its pull test

    @property
    def knee(self) -> tuple[float, float, float]:
        return (self.knee_x, 0.0, self.knee_z)

    @property
    def open_rad(self) -> float:
        return math.radians(self.open_deg)

    def flap_edge(self, angle_from_open: float = 0.0) -> np.ndarray:
        """Flap edge (y, z), shoe frame, at a flap joint angle (0 = splayed start, open_rad = closed)."""
        return HINGE + rot2(angle_from_open - self.open_rad) @ (EDGE_CLOSED - HINGE)

    def ring_local(self) -> np.ndarray:
        """Loop centre relative to the hinge in the flap's closed frame (y, z)."""
        return RING_ALONG * FLAP_D + RING_OUT * FLAP_N

    def strap_path(self) -> np.ndarray:
        """Start (splayed) strap centre line (y, z), anchor to tab end."""
        R = rot2(-self.open_rad)
        d, n = R @ FLAP_D, R @ FLAP_N
        ring = HINGE + R @ self.ring_local()
        edge = self.flap_edge(0.0)
        off = self.strap_t / 2 + 0.0008
        pts = [ANCHOR]
        foot = np.array(FOOT_YZ)
        for k in (1, 2, 3):  # medial side and crest, offset outward
            a, b = foot[k - 1], foot[k + 1]
            t = _unit(b - a)
            pts.append(foot[k] + off * np.array((t[1], -t[0])) * -1)
        crest = np.array((0.000, 0.088 + off))
        pts.append(crest)
        pts.append(edge + (off + 0.0005) * n - 0.004 * d)
        pts.append(ring + 0.012 * d)
        pts.append(ring)
        exit_ = ring - 0.006 * d
        pts.append(exit_)
        foot_ = np.array((exit_[0] + 0.012, exit_[1]))  # the opened tail lies out from the loop ...
        pts.append(foot_)
        pts.append(foot_ + np.array((0.0, 0.045)))  # ... and its stiff pull tab stands up, proud of the upper
        return np.array(pts)

    def strap_links(self) -> tuple[np.ndarray, np.ndarray]:
        """Link centres (y, z) and headings (rad, in the y-z plane) for the start pose."""
        pts = resample(self.strap_path(), self.link_len)
        need = self.n_links + 1
        if len(pts) < need:
            d = _unit(pts[-1] - pts[-2])
            extra = [pts[-1] + d * self.link_len * (k + 1) for k in range(need - len(pts))]
            pts = np.vstack((pts, extra))
        pts = pts[:need]
        seg = np.diff(pts, axis=0)
        return (pts[:-1] + pts[1:]) / 2, np.arctan2(seg[:, 1], seg[:, 0])


def resample(points, step: float) -> np.ndarray:
    p = np.asarray(points, float)
    seg = np.diff(p, axis=0)
    lens = np.linalg.norm(seg, axis=1)
    cum = np.concatenate(([0.0], np.cumsum(lens)))
    out = [p[0]]
    # chord resampling: next point on the polyline at distance `step` from the last one
    s_idx, cur = 0, p[0]
    while True:
        found = False
        for k in range(s_idx, len(seg)):
            a, b = p[k], p[k + 1]
            # solve |a + t (b - a) - cur| = step for t in [0, 1], the largest root beyond the current point
            d = b - a
            A = d @ d
            B = 2 * d @ (a - cur)
            C = (a - cur) @ (a - cur) - step * step
            disc = B * B - 4 * A * C
            if disc < 0:
                continue
            t = (-B + math.sqrt(disc)) / (2 * A)
            if 0.0 <= t <= 1.0 and (k > s_idx or t * math.sqrt(A) + 1e-12 >= 0):
                nxt = a + t * d
                if np.linalg.norm(nxt - cur) > step * 0.99:
                    out.append(nxt)
                    cur, s_idx, found = nxt, k, True
                    break
        if not found:
            break
    del cum
    return np.asarray(out)


def link_quat(phi: float) -> tuple[float, float, float, float]:
    """xyzw quaternion of a strap plate whose local x runs along heading phi in the (y, z) plane, local y along -x
    (across the strap) and local z its outer face."""
    ex = np.array((0.0, math.cos(phi), math.sin(phi)))
    ey = np.array((-1.0, 0.0, 0.0))
    ez = np.cross(ex, ey)
    return quat_from_matrix(np.stack((ex, ey, ez), axis=1))


def quat_from_matrix(m) -> tuple[float, float, float, float]:
    m = np.asarray(m, float)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array((x, y, z, w))
    return tuple((q / np.linalg.norm(q)).tolist())


def rotmat(q) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = np.moveaxis(q, -1, 0)
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qinv(a):
    return (-a[0], -a[1], -a[2], a[3])


def qx(angle: float):
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


# -- state -------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class ShoeState(ObjectState):
    leg_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))  # shoe-frame origin, world
    leg_quat: np.ndarray = field(default_factory=lambda: np.array((0.0, 0.0, 0.0, 1.0)))
    flap_angle: tuple = ()  # rad from splayed, per strap
    gap: tuple = ()  # m, per strap: flap edge to medial edge
    links: tuple = ()  # per strap: (n, 3) centres, world
    quats: tuple = ()  # per strap: (n, 4)
    ring: tuple = ()  # per strap: (centre (3,), rotation (3, 3)) world
    pairs: tuple = ()  # per strap: tuple of engaged (hook, loop) pairs


# -- Isaac Sim side ------------------------------------------------------------------------------------------------

_SPECS: dict[str, Shoe] = {}
PATHS: dict[str, str] = {}


def _spawn_shoe(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, Vt

    from yam_arena.station import linear_rgb

    spec: Shoe = _SPECS[cfg.shoe_name]
    PATHS[spec.name] = prim_path
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    # Isaac Lab places the articulation's root link (the stand, at the knee pivot) at the object's initial pose, so the
    # object is posed at the knee and the shoe frame sits one knee offset below it.
    T = np.array(translation or (0.0, 0.0, 0.0), dtype=float) - np.array(spec.knee)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*T))

    looks = {}

    def look(name, color, rough=0.6):
        if name not in looks:
            p = f"{prim_path}/Looks/{name}"
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough)
            m.func(p, m)
            looks[name] = p
        return looks[name]

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction, restitution=0.0)
    pm.func(phys, pm)
    grip_phys = f"{prim_path}/Looks/tab_physics"
    gm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.tab_friction, dynamic_friction=spec.tab_friction,
                                        restitution=0.0)
    gm.func(grip_phys, gm)
    ring_phys = f"{prim_path}/Looks/ring_physics"
    rm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.ring_friction, dynamic_friction=spec.ring_friction,
                                        restitution=0.0)
    rm.func(ring_phys, rm)

    def body(path, pos, quat_xyzw, mass, com=None, inertia=None, report=False, maxv=None):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        x, y, z, w = quat_xyzw
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        ma = UsdPhysics.MassAPI.Apply(prim)
        ma.CreateMassAttr(mass)
        if com is not None:
            ma.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateSleepThresholdAttr(0.0)
        rb.CreateLinearDampingAttr(0.05)
        rb.CreateAngularDampingAttr(0.1)
        if maxv is not None:
            rb.CreateMaxLinearVelocityAttr(maxv)
        if report:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    def collider(prim, offset=None):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset or spec.contact_offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)

    def box(path, centre, size, color_look, quat_xyzw=None, collide=True):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if quat_xyzw is not None:
            x, y, z, w = quat_xyzw
            cube.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        cube.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        if collide:
            collider(cube.GetPrim())
        if color_look:
            bind_visual_material(path, color_look, stage=stage)
        else:
            UsdGeom.Imageable(cube.GetPrim()).CreatePurposeAttr("guide")
        return cube

    def prism(path, poly_yz, x0, x1, color_look, collide=True, along_y=False):
        """A convex prism along x with cross-section poly_yz (body frame); with ``along_y``, a prism along y (x0..x1
        are then y limits) with cross-section poly_yz read as (x, z)."""
        poly = [tuple(map(float, p)) for p in poly_yz]
        n = len(poly)
        if along_y:
            pts = [Gf.Vec3f(a, x0, b) for a, b in poly] + [Gf.Vec3f(a, x1, b) for a, b in poly]
        else:
            pts = [Gf.Vec3f(x0, y, z) for y, z in poly] + [Gf.Vec3f(x1, y, z) for y, z in poly]
        counts, idx = [n, n], list(range(n))[::-1] + list(range(n, 2 * n))
        for k in range(n):
            j = (k + 1) % n
            counts.append(4)
            idx += [k, j, n + j, n + k]
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateSubdivisionSchemeAttr("none")
        if collide:
            collider(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        if color_look:
            bind_visual_material(path, color_look, stage=stage)
        return mesh

    white = look("canvas", (0.13, 0.17, 0.30), 0.9)  # the post-op shoe's navy padded canvas upper
    grey = look("foam", (0.36, 0.38, 0.44), 0.9)  # padded foam lining and tabs
    sole_l = look("sole", (0.20, 0.20, 0.21), 0.8)  # rigid flat rubber outsole
    gauze = look("gauze", (0.93, 0.92, 0.88), 0.95)  # the practice foot's padded dressing, seen at the open toe
    dark = look("strap", (0.12, 0.12, 0.13))
    hookl = look("hook", (0.30, 0.30, 0.33))
    skin = look("skin", (0.80, 0.66, 0.56))
    ring_l = look("ring", (0.05, 0.05, 0.05), 0.4)
    metal = look("metal", (0.55, 0.57, 0.60), 0.4)

    K = np.array(spec.knee)
    ident = (0.0, 0.0, 0.0, 1.0)
    # stand (fixed to the world) and knee link
    stand = f"{prim_path}/stand"
    body(stand, K, ident, 1.0, inertia=(1e-2, 1e-2, 1e-2))
    box(f"{stand}/bracket", (0.03, 0, 0.0), (0.05, 0.03, 0.03), metal, collide=False)
    knee = f"{prim_path}/knee"
    body(knee, K, ident, 0.1, inertia=(1e-4, 1e-4, 1e-4))
    leg = f"{prim_path}/leg"
    body(leg, (0, 0, 0), ident, spec.leg_mass, report=True)  # mass properties from the colliders
    # shin (visual and collider), ankle
    A = np.array(spec.ankle)
    axis = K - A
    shin_len = float(np.linalg.norm(axis))
    tilt = math.atan2(axis[0], axis[2])  # about y, from vertical toward +x
    cap = UsdGeom.Capsule.Define(stage, f"{leg}/shin")
    cap.CreateRadiusAttr(spec.shin_r)
    cap.CreateHeightAttr(shin_len)
    cap.CreateAxisAttr("Z")
    mid = (A + K) / 2
    cap.AddTranslateOp().Set(Gf.Vec3d(*map(float, mid)))
    cap.AddOrientOp().Set(Gf.Quatf(math.cos(tilt / 2), 0.0, math.sin(tilt / 2), 0.0))
    collider(cap.GetPrim(), 0.002)
    bind_visual_material(f"{leg}/shin", skin, stage=stage)
    wrap = UsdGeom.Capsule.Define(stage, f"{leg}/dressing")  # the padded dressing round the ankle (visual)
    wrap.CreateRadiusAttr(spec.shin_r + 0.004)
    wrap.CreateHeightAttr(0.05)
    wrap.CreateAxisAttr("Z")
    wrap.AddTranslateOp().Set(Gf.Vec3d(*map(float, A + 0.045 * axis / shin_len)))
    wrap.AddOrientOp().Set(Gf.Quatf(math.cos(tilt / 2), 0.0, math.sin(tilt / 2), 0.0))
    bind_visual_material(f"{leg}/dressing", gauze, stage=stage)
    # rigid post-op sole, 29 cm long, 10.7 cm wide at the forefoot, 3 cm thick: flat under the foot, with a rocker
    # (the toe rises 12 mm over its last 4 cm, the heel 6 mm over its last 2 cm)
    sole_poly_f = ((-0.050, 0.0), (0.055, 0.0), (0.057, 0.030), (-0.052, 0.030))
    prism(f"{leg}/sole_front", sole_poly_f, -0.105, -0.010, sole_l)
    toe_rock = ((-0.105, 0.0), (-0.105, 0.030), (-0.145, 0.030), (-0.145, 0.012), (-0.126, 0.003))
    prism(f"{leg}/sole_toe", toe_rock, -0.050, 0.055, sole_l, along_y=True)
    sole_poly_r = ((-0.042, 0.0), (0.045, 0.0), (0.047, 0.030), (-0.044, 0.030))
    prism(f"{leg}/sole_rear", sole_poly_r, -0.010, 0.125, sole_l)
    heel_rock = ((0.125, 0.0), (0.145, 0.006), (0.145, 0.030), (0.125, 0.030))
    prism(f"{leg}/sole_heel", heel_rock, -0.042, 0.045, sole_l, along_y=True)
    sole_phys = f"{prim_path}/Looks/sole_physics"
    sm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.sole_friction, dynamic_friction=spec.sole_friction,
                                        restitution=0.0)
    sm.func(sole_phys, sm)
    for part in ("sole_front", "sole_toe", "sole_rear", "sole_heel"):
        bind_physics_material(f"{leg}/{part}", sole_phys, stage=stage)
    # foot / tongue / medial quarter under the straps
    prism(f"{leg}/instep", FOOT_YZ, FOOT_X[0], FOOT_X[1], white)
    # open toe: the dressed forefoot and toes of the practice foot stand on the sole past the upper
    toe = ((-0.050, 0.030), (-0.046, 0.055), (-0.025, 0.068), (0.020, 0.068), (0.045, 0.055), (0.054, 0.030))
    prism(f"{leg}/toe", toe, -0.132, FOOT_X[0], gauze)
    tip = ((-0.040, 0.030), (-0.030, 0.050), (0.030, 0.050), (0.044, 0.030))
    prism(f"{leg}/toe_tip", tip, -0.140, -0.132, gauze)
    # the forefoot strap, already fastened before the trial (visual): a band over the dressed forefoot
    band = [(1.06 * y, 0.030 + 1.06 * (z - 0.030)) for y, z in toe]
    prism(f"{leg}/toe_strap", band, -0.128, -0.103, look("strap", (0.12, 0.12, 0.13)), collide=False)
    # lateral quarter behind the flaps, collar and heel counter
    rear = ((-0.044, 0.030), (-0.042, 0.080), (-0.030, 0.092), (0.030, 0.092), (0.044, 0.080), (0.046, 0.030))
    prism(f"{leg}/heel", rear, FOOT_X[1], 0.140, white)
    prism(f"{leg}/heel_tab", ((-0.025, 0.060), (-0.025, 0.120), (0.025, 0.120), (0.025, 0.060)), 0.140, 0.148, grey)
    # the closed rear part of the lateral quarter, from behind the opening's wing to the heel (visual: the instep
    # prism under it is the collider)
    rx0 = max(sg.flap_x[1] for sg in spec.straps) + 0.002 if spec.straps else FOOT_X[0]
    q_closed = (FLAP_LEN / 2) * FLAP_D - (FLAP_T / 2) * FLAP_N + HINGE
    box(f"{leg}/lateral_quarter", ((rx0 + FOOT_X[1]) / 2, q_closed[0], q_closed[1]), (FOOT_X[1] - rx0, FLAP_LEN, FLAP_T),
        white, quat_xyzw=qx(math.atan2(FLAP_D[1], FLAP_D[0])), collide=False)
    # tongue top (suede, visual)
    box(f"{leg}/tongue", (-0.020, 0.0, 0.0885), (0.120, 0.016, 0.002), grey, collide=False)

    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(stand))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(stand))
    art.CreateEnabledSelfCollisionsAttr(True)
    art.CreateSolverPositionIterationCountAttr(spec.solver_pos_iters)
    art.CreateSolverVelocityIterationCountAttr(4)
    art.CreateSleepThresholdAttr(0.0)
    art.CreateStabilizationThresholdAttr(0.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fixed")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(stand)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, T + K)))
    fj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    def revolute(path, b0, b1, p0, q0, p1, q1, axis, k=0.0, c=0.0):
        j = UsdPhysics.RevoluteJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalRot0Attr(Gf.Quatf(q0[3], q0[0], q0[1], q0[2]))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p1)))
        j.CreateLocalRot1Attr(Gf.Quatf(q1[3], q1[0], q1[1], q1[2]))
        j.CreateAxisAttr(axis)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateTargetPositionAttr(0.0)
        d.CreateStiffnessAttr(float(k) * math.pi / 180)
        d.CreateDampingAttr(float(c) * math.pi / 180)
        return j

    revolute(f"{knee}/knee_side", stand, knee, (0, 0, 0), ident, (0, 0, 0), ident, "X", spec.side_k, spec.side_c)
    revolute(f"{leg}/knee_flex", knee, leg, (0, 0, 0), ident, K, ident, "Y", spec.flex_k, spec.flex_c)

    L = spec.link_len
    hl = L / 2
    for si, sg in enumerate(spec.straps):
        tag = "ab"[si]
        # flap: body frame = the closed flap's frame at the hinge, posed splayed
        flap = f"{prim_path}/flap_{tag}"
        q_open = qx(-spec.open_rad)
        body(flap, (sum(sg.flap_x) / 2, HINGE[0], HINGE[1]), q_open, spec.flap_mass, inertia=(2e-5, 2e-5, 2e-5))
        fw = sg.flap_x[1] - sg.flap_x[0]
        mid = (FLAP_LEN / 2) * FLAP_D - (FLAP_T / 2) * FLAP_N
        ang = math.atan2(FLAP_D[1], FLAP_D[0])
        box(f"{flap}/plate", (0.0, mid[0], mid[1]), (fw, FLAP_LEN, FLAP_T), white, quat_xyzw=qx(ang))
        # the loop (rigid plastic), centred on the strap's x
        dx = sg.x - sum(sg.flap_x) / 2
        c = (np.array((0.0, 0.0)) + RING_ALONG * FLAP_D + RING_OUT * FLAP_N)
        hb = RING_BAR / 2
        for k, s in enumerate((-1, 1)):  # bars across the strap, inside and outside
            cc = c + s * (RING_IN_H / 2 + hb) * FLAP_N
            if s > 0:  # the outer bar, round: the strap turns back round it
                cap = UsdGeom.Capsule.Define(stage, f"{flap}/ring_bar_{k}")
                cap.CreateRadiusAttr(RING_BAR / 2)
                cap.CreateHeightAttr(RING_IN_W + RING_BAR)
                cap.CreateAxisAttr("X")
                cap.AddTranslateOp().Set(Gf.Vec3d(float(dx), float(cc[0]), float(cc[1])))
                collider(cap.GetPrim())
                bind_physics_material(str(cap.GetPath()), ring_phys, stage=stage)
                bind_visual_material(str(cap.GetPath()), ring_l, stage=stage)
                continue
            rb_ = box(f"{flap}/ring_bar_{k}", (dx, cc[0], cc[1]), (RING_IN_W + 2 * RING_BAR, RING_BAR, RING_BAR),
                      ring_l, quat_xyzw=qx(ang))
            bind_physics_material(str(rb_.GetPath()), ring_phys, stage=stage)
        for k, s in enumerate((-1, 1)):  # ends
            re_ = box(f"{flap}/ring_end_{k}", (dx + s * (RING_IN_W / 2 + hb), c[0], c[1]),
                      (RING_BAR, RING_BAR, RING_IN_H + 2 * RING_BAR), ring_l, quat_xyzw=qx(ang))
            bind_physics_material(str(re_.GetPath()), ring_phys, stage=stage)
        box(f"{flap}/ring_tab", (dx, (c - 0.004 * FLAP_N - 0.006 * FLAP_D)[0], (c - 0.004 * FLAP_N - 0.006 * FLAP_D)[1]),
            (RING_IN_W * 0.8, 0.014, 0.0015), grey, quat_xyzw=qx(ang), collide=False)
        hinge_pos = (sum(sg.flap_x) / 2, HINGE[0], HINGE[1])
        j = revolute(f"{flap}/flap_{tag}_hinge", leg, flap, hinge_pos, q_open, (0, 0, 0), ident, "X", spec.flap_k, spec.flap_c)
        j.CreateLowerLimitAttr(-25.0)
        j.CreateUpperLimitAttr(float(spec.open_deg))
        # strap
        cen, phi = spec.strap_links()
        prev, prev_q = leg, ident
        for i in range(spec.n_links):
            p = f"{prim_path}/strap_{tag}_{i:02d}"
            q = link_quat(float(phi[i]))
            m_i = spec.tab_mass if i >= spec.n_links - spec.tab_links else spec.link_mass
            body(p, (sg.x, cen[i][0], cen[i][1]), q, m_i, report=True, maxv=3.0)
            hk = spec.hook[0] <= i < spec.hook[1]
            lp = spec.loop[0] <= i < spec.loop[1]
            if i >= spec.n_links - spec.tab_links:  # the rubberised pull tab at the end
                g = box(f"{p}/geom", (0, 0, 0), (L * 0.97, spec.strap_w, spec.tab_t), look("tab", (0.08, 0.08, 0.09), 0.8))
                bind_physics_material(str(g.GetPath()), grip_phys, stage=stage)
                cpa = PhysxSchema.PhysxCollisionAPI(g.GetPrim())
                cpa.CreateTorsionalPatchRadiusAttr(spec.tab_torsion)
                cpa.CreateMinTorsionalPatchRadiusAttr(0.5 * spec.tab_torsion)
                if i == spec.n_links - 1:  # the moulded bead across the tab's end
                    bd = box(f"{p}/bead", (L / 2 - 0.0015, 0, 0), (0.003, spec.strap_w, spec.bead_t),
                             look("tab", (0.08, 0.08, 0.09), 0.8))
                    bind_physics_material(str(bd.GetPath()), grip_phys, stage=stage)
            else:
                box(f"{p}/geom", (0, 0, 0), (L * 0.97, spec.strap_w, spec.strap_t), dark)
            if hk:  # hook patch, slightly inset on the outer face (visual)
                box(f"{p}/hook", (0, 0, spec.strap_t / 2 + 0.0001), (L * 0.98, spec.strap_w * 0.92, 0.0002), hookl,
                    collide=False)
            if lp:
                box(f"{p}/loopface", (0, 0, spec.strap_t / 2 + 0.0001), (L * 0.98, spec.strap_w * 0.96, 0.0002),
                    look("loopf", (0.20, 0.20, 0.22)), collide=False)
            if i == 0:
                pos0 = (sg.x, ANCHOR[0], ANCHOR[1])
                jq = q
                revolute(f"{p}/strap_{tag}_bend_{i:02d}", leg, p, pos0, jq, (-hl, 0, 0), ident, "Y", spec.bend_k, spec.bend_c)
            else:
                rel = qmul(qinv(prev_q), q)
                kk = spec.bend_k if i < spec.stiff_from else (  # the hinges inside the moulded tab are stiff; the
                    spec.hook_bend_k if i <= spec.n_links - spec.tab_links else spec.tab_bend_k)  # tab joins as webbing
                revolute(f"{p}/strap_{tag}_bend_{i:02d}", prev, p, (hl, 0, 0), rel, (-hl, 0, 0), ident, "Y", kk,
                         max(spec.bend_c, kk * 0.05))
            prev, prev_q = p, q
        # hook-and-loop pair joints (disabled; Fastener enables them)
        for h in range(*spec.hook):
            for lo in range(*spec.loop):
                if lo >= h:
                    continue
                jp = f"{prim_path}/pair_{tag}_{h:02d}_{lo:02d}"
                pj = UsdPhysics.Joint.Define(stage, jp)
                pj.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/strap_{tag}_{lo:02d}")])
                pj.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/strap_{tag}_{h:02d}")])
                pj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
                pj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
                pj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                pj.CreateExcludeFromArticulationAttr(True)
                pj.CreateJointEnabledAttr(False)
                pj.CreateCollisionEnabledAttr(True)
                for ax in ("transX", "transY", "transZ"):
                    d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), ax)
                    d.CreateTypeAttr("force")
                    d.CreateStiffnessAttr(spec.pair_k)
                    d.CreateDampingAttr(spec.pair_c)
                    d.CreateTargetPositionAttr(0.0)
        if spec.tab_drive:
            last = f"{prim_path}/strap_{tag}_{spec.n_links - 1:02d}"
            w0 = T + np.array((sg.x, cen[-1][0], cen[-1][1]))
            dj = UsdPhysics.Joint.Define(stage, f"{prim_path}/pull_gauge_{tag}")
            dj.CreateBody1Rel().SetTargets([Sdf.Path(last)])
            dj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, w0)))
            dj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            dj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            dj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            dj.CreateExcludeFromArticulationAttr(True)
            for ax in ("transX", "transY", "transZ"):
                d = UsdPhysics.DriveAPI.Apply(dj.GetPrim(), ax)
                d.CreateTypeAttr("force")
                d.CreateStiffnessAttr(0.0)
                d.CreateDampingAttr(0.0)
                d.CreateMaxForceAttr(0.0)
                d.CreateTargetPositionAttr(0.0)
    return root.GetPrim()


def _actuators(spec: Shoe):
    from isaaclab.actuators import ImplicitActuatorCfg
    out = {"side": ImplicitActuatorCfg(joint_names_expr=["knee_side"], stiffness=spec.side_k, damping=spec.side_c),
           "flex": ImplicitActuatorCfg(joint_names_expr=["knee_flex"], stiffness=spec.flex_k, damping=spec.flex_c)}
    if spec.straps:
        out["flaps"] = ImplicitActuatorCfg(joint_names_expr=["flap_.*"], stiffness=spec.flap_k, damping=spec.flap_c)
    if spec.straps:
        groups = {"strap": (range(0, spec.stiff_from), spec.bend_k),
                  "strap_hook": (range(spec.stiff_from, spec.n_links - spec.tab_links + 1), spec.hook_bend_k),
                  "strap_tab": (range(spec.n_links - spec.tab_links + 1, spec.n_links), spec.tab_bend_k)}
        for name, (idx, k) in groups.items():
            if len(idx) == 0:
                continue
            out[name] = ImplicitActuatorCfg(joint_names_expr=[f"strap_[ab]_bend_{i:02d}" for i in idx], stiffness=k,
                                            damping=max(spec.bend_c, k * 0.05), armature=spec.armature)
    return out


def _arena_object(spec: Shoe):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class ShoeSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_shoe)
        shoe_name: str = ""

    class ShoeObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path,
                spawn=self.spawner_cfg,
                actuators=_actuators(spec) or {
                    "side": ImplicitActuatorCfg(joint_names_expr=["knee_side"], stiffness=spec.side_k,
                                                damping=spec.side_c),
                    "flex": ImplicitActuatorCfg(joint_names_expr=["knee_flex"], stiffness=spec.flex_k,
                                                damping=spec.flex_c),
                    "flaps": ImplicitActuatorCfg(joint_names_expr=["flap_.*"], stiffness=spec.flap_k,
                                                 damping=spec.flap_c),
                    "strap": ImplicitActuatorCfg(joint_names_expr=["strap_.*"], stiffness=spec.bend_k,
                                                 damping=spec.bend_c, armature=spec.armature),
                },
            )
            return self._add_initial_pose_to_cfg(cfg)

    return ShoeObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=ShoeSpawnerCfg(shoe_name=spec.name),
                      initial_pose=Pose(position_xyz=tuple(float(v) + float(k) for v, k in zip(spec.pos, spec.knee)),
                                        rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _t(x):
    return getattr(x, "torch", x)


def as_np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy(), dtype=np.float64)


def articulation(env, name: str):
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    return base.scene[name]


def _origin(env):
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    return _t(base.scene.env_origins)[0].detach().cpu().numpy()


def raw_state(env, name: str) -> dict:
    """Body poses (world, env origin removed) by body name, and joint positions by joint name."""
    art = articulation(env, name)
    o = _origin(env)
    pos = _t(art.data.body_link_pos_w)[0].detach().cpu().numpy().astype(np.float64) - o
    quat = _t(art.data.body_link_quat_w)[0].detach().cpu().numpy().astype(np.float64)  # xyzw in this Isaac Lab
    q = _t(art.data.joint_pos)[0].detach().cpu().numpy().astype(np.float64)
    return {"pos": dict(zip(art.body_names, pos)), "quat": dict(zip(art.body_names, quat)),
            "joint": dict(zip(art.joint_names, q))}


def state_from_raw(spec: Shoe, raw: dict, pairs=()) -> ShoeState:
    """Pure NumPy: raw poses to a ShoeState (used live and offline)."""
    lp, lq = raw["pos"]["leg"], raw["quat"]["leg"]
    Rl = rotmat(lq[None])[0]
    gaps, angs, links, quats, rings = [], [], [], [], []
    for si, sg in enumerate(spec.straps):
        tag = "ab"[si]
        fp, fq = raw["pos"][f"flap_{tag}"], raw["quat"][f"flap_{tag}"]
        Rf = rotmat(fq[None])[0]
        e_loc = np.array((0.0, *(EDGE_CLOSED - HINGE)))
        edge_w = fp + Rf @ e_loc
        med_w = lp + Rl @ np.array((sg.x, *MEDIAL_EDGE))
        # gap in the leg's y-z plane
        d = Rl.T @ (edge_w - med_w)
        gaps.append(float(math.hypot(d[1], d[2])))
        angs.append(float(raw["joint"].get(f"flap_{tag}_hinge", 0.0)))
        links.append(np.array([raw["pos"][f"strap_{tag}_{i:02d}"] for i in range(spec.n_links)]))
        quats.append(np.array([raw["quat"][f"strap_{tag}_{i:02d}"] for i in range(spec.n_links)]))
        c_loc = np.array((sg.x - sum(sg.flap_x) / 2, *(RING_ALONG * FLAP_D + RING_OUT * FLAP_N)))
        rings.append((fp + Rf @ c_loc, Rf))
    lo = np.min([lk.min(0) for lk in links] + [lp + np.array((-0.15, -0.06, 0.0))], axis=0)
    hi = np.max([lk.max(0) for lk in links] + [lp + np.array((0.15, 0.06, 0.10))], axis=0)
    return ShoeState(pos=lp, quat_xyzw=lq, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=lo, high=hi, leg_pos=lp,
                     leg_quat=lq, flap_angle=tuple(angs), gap=tuple(gaps), links=tuple(links), quats=tuple(quats),
                     ring=tuple(rings), pairs=tuple(pairs) if pairs else tuple(() for _ in spec.straps))


FASTENERS: dict[str, "Fastener"] = {}


def shoe_state(env, name: str) -> ShoeState:
    spec = _SPECS[name]
    f = FASTENERS.get(name)
    pairs = tuple(tuple(sorted(f.on[t].keys())) for t in "ab"[:len(spec.straps)]) if f else ()
    return state_from_raw(spec, raw_state(env, name), pairs)


# -- hook and loop -------------------------------------------------------------------------------------------------------

@dataclass
class FastenRule:
    press_n: float = 2.0  # N of contact force between a hook plate and a loop plate to engage
    face_dot: float = -0.8  # faces opposed: hook outer normal . loop outer normal below this
    peel_n: float = 2.5  # N per plate pulling the faces apart (1.0 N/cm x 2.5 cm)
    shear_n: float = 17.0  # N per plate along the faces (7 N/cm2 x 2.5 cm x 1.1 cm, rounded)


def engage_ok(rule: FastenRule, force: float, z_hook, z_loop, p_hook, p_loop) -> bool:
    """Pure: may this pair engage now?"""
    if force < rule.press_n:
        return False
    if float(np.dot(z_hook, z_loop)) > rule.face_dot:
        return False
    return float(np.dot(np.asarray(p_hook) - np.asarray(p_loop), z_loop)) > 0.0  # hook on the loop's outer side


def pair_load(k: float, R_loop, p_loop, p_hook, target) -> tuple[float, float]:
    """Pure: (peel, shear) force of an engaged pair's spring: peel > 0 pulls the faces apart."""
    d = R_loop.T @ (np.asarray(p_hook) - np.asarray(p_loop)) - np.asarray(target)
    f = k * d
    return float(f[2]), float(math.hypot(f[0], f[1]))


class Fastener:
    """Engages and breaks the pre-authored pair joints from contact forces and poses, once per control step."""

    def __init__(self, env, name: str, rule: FastenRule | None = None):
        self.env, self.name, self.spec = env, name, _SPECS[name]
        self.rule = rule or FastenRule()
        self.on = {t: {} for t in "ab"[:len(self.spec.straps)]}  # tag -> {(h, l): target}
        self.views = None
        self.events: list = []
        self.max_force: dict = {}
        FASTENERS[name] = self

    def _attach(self):
        if self.views is not None:
            return
        base = self.env.unwrapped
        sim_view = base.scene[self.name]._physics_sim_view
        root = PATHS[self.name]
        self.views = {}
        for t in self.on:
            loops = [f"{root}/strap_{t}_{lo:02d}" for lo in range(*self.spec.loop)]
            for h in range(*self.spec.hook):
                self.views[(t, h)] = sim_view.create_rigid_contact_view(f"{root}/strap_{t}_{h:02d}",
                                                                        filter_patterns=loops,
                                                                        max_contact_data_count=0)

    def forces(self) -> dict:
        """(tag, hook, loop) -> contact force magnitude (N) at the last physics step."""
        self._attach()
        out = {}
        dt = self.env.unwrapped.sim.get_physics_dt()
        for (t, h), v in self.views.items():
            m = as_np(v.get_contact_force_matrix(dt))[0]
            for k, lo in enumerate(range(*self.spec.loop)):
                f = float(np.linalg.norm(m[k]))
                if f > 0:
                    out[(t, h, lo)] = f
        return out

    def _set(self, t, h, lo, target):
        import omni.usd
        from pxr import UsdPhysics
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(f"{PATHS[self.name]}/pair_{t}_{h:02d}_{lo:02d}")
        if target is not None:
            for k, ax in enumerate(("transX", "transY", "transZ")):
                UsdPhysics.DriveAPI(prim, ax).GetTargetPositionAttr().Set(float(target[k]))
        UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(target is not None)

    def release_all(self):
        for t in self.on:
            for (h, lo) in list(self.on[t]):
                self._set(t, h, lo, None)
            self.on[t] = {}

    def update(self, step: int, raw: dict) -> dict:
        r = self.rule
        f = self.forces()
        for key, v in f.items():
            self.max_force[key] = max(self.max_force.get(key, 0.0), v)
        # breaks first
        for t in self.on:
            for (h, lo), tgt in list(self.on[t].items()):
                pl, ql = raw["pos"][f"strap_{t}_{lo:02d}"], raw["quat"][f"strap_{t}_{lo:02d}"]
                ph = raw["pos"][f"strap_{t}_{h:02d}"]
                peel, shear = pair_load(self.spec.pair_k, rotmat(ql[None])[0], pl, ph, tgt)
                if peel > r.peel_n or shear > r.shear_n:
                    self._set(t, h, lo, None)
                    del self.on[t][(h, lo)]
                    self.events.append((step, "break", t, h, lo, round(peel, 2), round(shear, 2)))
        best = {}
        for (t, h, lo), force in f.items():
            if lo < h and force > best.get((t, h), (None, 0.0))[1]:
                best[(t, h)] = (lo, force)
        for (t, h), (lo, force) in best.items():
            if any(hh == h for hh, _ in self.on[t]):
                continue
            pl, ql = raw["pos"][f"strap_{t}_{lo:02d}"], raw["quat"][f"strap_{t}_{lo:02d}"]
            ph, qh = raw["pos"][f"strap_{t}_{h:02d}"], raw["quat"][f"strap_{t}_{h:02d}"]
            Rl, Rh = rotmat(ql[None])[0], rotmat(qh[None])[0]
            if engage_ok(r, force, Rh[:, 2], Rl[:, 2], ph, pl):
                tgt = Rl.T @ (ph - pl)
                self._set(t, h, lo, tgt)
                self.on[t][(h, lo)] = tgt
                self.events.append((step, "engage", t, h, lo, round(force, 2)))
        return f


# -- yam_arena hooks -------------------------------------------------------------------------------------------------

def _patch_scene(scene) -> None:
    if getattr(scene, "_shoekit", False):
        return
    orig_obj, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Shoe):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_obj(spec)

    def object_states(env, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(env, plain) if plain else {}
        out.update({n: shoe_state(env, n) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._shoekit = True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig = spec.loader.exec_module

        def exec_module(module):
            orig(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
