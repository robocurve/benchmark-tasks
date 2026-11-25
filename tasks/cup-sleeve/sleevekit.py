"""sleevekit: the cup, the sleeve organizer and the sleeve (16 hinged strips in a closed loop) for the task file, and
the per-step model: the hinges' set (elastic-plastic paperboard) and the contact forces the scorer reads.

Bodies are bodykit Bodies (one rigid body each). The sleeve's 16 hinges are PhysX revolute joints between the strips,
authored once the last strip exists (a closed loop of joints between free bodies, so no articulation). Each hinge has
an angular spring whose rest angle is the board's set: it starts at the flat pack (0) and follows the hinge once the
hinge is bent more than a yield band past it (creased and bent corrugated board stays roughly where it is bent, and
springs back by the band).
"""

from __future__ import annotations

import math

import numpy as np

import bodykit
import sleevegeom as G
from bodykit import Body, BoxShape, ConvexShape, CylShape, MeshShape

# -- layout (world, m) ----------------------------------------------------------------------------------------------------
TOP = 0.7475  # the station table's top
CUP_XY = (0.370, -0.240)
ORG_XY = (0.250, 0.000)  # the organizer compartment's centre
ORG_IN = (0.020, 0.140)  # inside, x by y
ORG_WALL_H = 0.030
ORG_T = 0.003
SLEEVE_ORIGIN = np.array((ORG_XY[0], ORG_XY[1], TOP + ORG_T + 0.0004))  # crease ends' midpoint, on the compartment floor

# -- materials ------------------------------------------------------------------------------------------------------------
KRAFT = (0.60, 0.43, 0.27)
KRAFT_EDGE = (0.50, 0.35, 0.21)
CUP_WHITE = (0.93, 0.92, 0.89)
LID_BLACK = (0.08, 0.08, 0.09)
ACRYLIC = (0.78, 0.86, 0.92)
F_CUP = (1.2, 1.0)  # the harness's grip friction for handled objects (averaged with the fingers' 0.5: 0.85 static)
MU_INSIDE = 0.30  # the sleeve's fluted inner face on the cup's coated wall (the lower of the two is used)
MU_OUTSIDE = 1.00  # its outer kraft face under a rubber fingertip (the higher of the two is used)
SKIN = 0.0005  # the inner face's layer of the strip's thickness

# -- the board's hinges (estimates; see README) ---------------------------------------------------------------------------
K_BOARD = 4.0e-4  # N m per degree: a strip of the sleeve bent across its corrugations
K_CREASE = 2.0e-4  # a factory crease
CREASE_SET = -40.0  # degrees: a factory crease's own set is short of folded flat (negative opens it); the flat pack
# is pressed flat by the stack and springs a little open when free (the organizer's slot stops it)
BAND_BOARD = 6.0  # degrees of spring-back before the board takes a set
BAND_CREASE = 10.0
DAMP = 2.0e-5  # N m s per degree
STRIP_MASS = G.SLEEVE_MASS / G.N_FACETS
STRIP_INERTIA = 2.0e-7  # kg m^2, numerical (see README)

STRIPS = [f"sleeve_{k:02d}" for k in range(G.N_FACETS)]
_FLAT = G.flat_facets(SLEEVE_ORIGIN)
_HINGES = G.hinge_lines(SLEEVE_ORIGIN)
STRIP_POS = [p.mean(axis=0) for p in _FLAT]
STRIP_LOCAL = [p - c for p, c in zip(_FLAT, STRIP_POS)]


def _hex_faces(pts: np.ndarray):
    """Triangles of a strip's 8 corners (inner 0-3, outer 4-7), wound outward."""
    quads = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    c = pts.mean(axis=0)
    tris = []
    for a, b, cc, d in quads:
        for t in ((a, b, cc), (a, cc, d)):
            n = np.cross(pts[t[1]] - pts[t[0]], pts[t[2]] - pts[t[0]])
            if n @ (pts[list(t)].mean(axis=0) - c) < 0:
                t = (t[0], t[2], t[1])
            tris.append(t)
    return tuple(tris)


def _frustum_mesh(r0, r1, z0, z1, n=48, cap=True):
    pts, faces = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        pts.append((r0 * math.cos(a), r0 * math.sin(a), z0))
        pts.append((r1 * math.cos(a), r1 * math.sin(a), z1))
    for k in range(n):
        i, j = 2 * k, 2 * ((k + 1) % n)
        faces += [(i, j, j + 1), (i, j + 1, i + 1)]
    if cap:
        b = len(pts)
        pts += [(0.0, 0.0, z0), (0.0, 0.0, z1)]
        for k in range(n):
            i, j = 2 * k, 2 * ((k + 1) % n)
            faces += [(b, j, i), (b + 1, i + 1, j + 1)]
    return tuple(pts), tuple(faces)


def bodies() -> list:
    out = []
    # the organizer: one open-top clear acrylic compartment, fixed to the counter
    ix, iy = ORG_IN
    t, h = ORG_T, ORG_WALL_H
    sh = [BoxShape((0, 0, t / 2), (ix + 2 * t, iy + 2 * t, t), ACRYLIC)]
    for s in (-1, 1):
        sh.append(BoxShape((s * (ix / 2 + t / 2), 0, h / 2), (t, iy + 2 * t, h), ACRYLIC))
        sh.append(BoxShape((0, s * (iy / 2 + t / 2), h / 2), (ix, t, h), ACRYLIC))
    out.append(Body.build("organizer", (ORG_XY[0], ORG_XY[1], TOP), mass=0.3, shapes=sh, kinematic=True,
                          friction=(0.4, 0.35)))
    # the cup: a rigid paper cone (the collider) with its lid; origin at the middle of its base
    rb, rl = G.CUP_D_BOTTOM / 2, G.CUP_D_LIP / 2
    c24 = 1.0 / math.cos(math.pi / 24)  # a 24-sided hull (50 points: PhysX keeps it as given) whose faces touch the cone
    pts, faces = _frustum_mesh(rb * c24, rl * c24, 0.0, G.CUP_H, n=24)
    vis_pts, vis_faces = _frustum_mesh(rb, rl, 0.0, G.CUP_H, n=64)
    lid_z = G.CUP_H - 0.006
    cup_shapes = [
        ConvexShape((0, 0, G.CUP_H / 2), pts, faces, color=None),
        MeshShape((0, 0, G.CUP_H / 2), vis_pts, vis_faces, CUP_WHITE, smooth=False, roughness=0.6),
        CylShape((0, 0, lid_z + 0.004), G.LID_D / 2, 0.008, LID_BLACK),  # the lid's skirt over the rolled lip
        CylShape((0, 0, lid_z + 0.0105), 0.0385, 0.009, LID_BLACK),  # its dome
        CylShape((0, 0, G.CUP_H - 0.0015), rl + 0.0012, 0.003, CUP_WHITE, collide=False),  # rolled lip, just visible
        BoxShape((0.030, 0, lid_z + 0.0152), (0.010, 0.004, 0.0008), (0.02, 0.02, 0.02), collide=False),  # sip hole
    ]
    out.append(Body.build("cup", (CUP_XY[0], CUP_XY[1], TOP), mass=G.CUP_MASS, shapes=cup_shapes,
                          com=(0.0, 0.0, G.CUP_COM_Z), friction=F_CUP, contact_report=True, solver_iterations=24,
                          yaw=math.pi))  # sip hole toward the robot
    # the sleeve's strips
    for k, name in enumerate(STRIPS):
        loc = STRIP_LOCAL[k]
        fc = _hex_faces(loc)
        ign = []
        if k > 0:
            ign.append(STRIPS[k - 1])
        if k == G.N_FACETS - 1:
            ign.append(STRIPS[0])
        inner, outer = loc[:4], loc[4:]
        mid = inner + (outer - inner) * (SKIN / G.BOARD_T)
        skin_in = np.vstack([inner, mid])
        skin_out = np.vstack([mid, outer])
        col = KRAFT if k % 2 else KRAFT_EDGE
        out.append(Body.build(name, tuple(STRIP_POS[k]), mass=STRIP_MASS,
                              shapes=[ConvexShape((0, 0, 0), tuple(map(tuple, skin_in)), _hex_faces(skin_in), color=col),
                                      ConvexShape((0, 0, 0), tuple(map(tuple, skin_out)), _hex_faces(skin_out), color=col)],
                              friction=(MU_OUTSIDE, MU_OUTSIDE), contact_report=True, solver_iterations=32, inertia=STRIP_INERTIA,
                              damping=(0.5, 0.5), ignore=tuple(ign), contact_offset=0.001))
    return out


# -- the hinges (authored with the last strip) ----------------------------------------------------------------------------

HINGE_PATHS: list[str] = []
HINGE_IS_CREASE = [k in G.CREASES for k in range(G.N_FACETS)]


def _quat_x_to(d) -> tuple[float, float, float, float]:
    """(w, x, y, z) of the rotation turning +x onto unit vector d."""
    x = np.array((1.0, 0.0, 0.0))
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = float(x @ d)
    if c > 1 - 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    if c < -1 + 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax = np.cross(x, d)
    ax /= np.linalg.norm(ax)
    a = math.acos(c)
    s = math.sin(a / 2)
    return (math.cos(a / 2), ax[0] * s, ax[1] * s, ax[2] * s)


def _faces_material(stage, prim_path: str, spec) -> None:
    """The strip's two faces get their own friction: the inner face (on the cup) takes the lower of the pair, the
    outer face (under the fingers) the higher."""
    if spec.name not in STRIPS:
        return
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material

    for i, (mu, mode) in enumerate(((MU_INSIDE, "min"), (MU_OUTSIDE, "max"))):
        path = f"{prim_path}/Looks/face{i}"
        m = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, friction_combine_mode=mode)
        m.func(path, m)
        bind_physics_material(f"{prim_path}/body/s{i}", path, stage=stage)


bodykit.EXTRA_AUTHORS.append(_faces_material)

TORSION_R = 0.020  # m: a fingertip's contact patch on the cup wall (a rubber pad on a curved wall), so that a pinch
# resists the cup turning about it


def _cup_patch(stage, prim_path: str, spec) -> None:
    if spec.name != "cup":
        return
    from pxr import PhysxSchema

    cp = PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body/s0"))
    cp.CreateTorsionalPatchRadiusAttr(TORSION_R)
    cp.CreateMinTorsionalPatchRadiusAttr(0.8 * TORSION_R)


bodykit.EXTRA_AUTHORS.append(_cup_patch)


def _author(stage, prim_path: str, spec) -> None:
    if spec.name != STRIPS[-1]:
        return
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    HINGE_PATHS.clear()
    for k, (pt, d) in enumerate(_HINGES):
        b0, b1 = STRIPS[k - 1], STRIPS[k]
        path = f"{parent}/{b1}/hinge_{k:02d}"
        j = UsdPhysics.RevoluteJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([f"{parent}/{b0}/body"])
        j.CreateBody1Rel().SetTargets([f"{parent}/{b1}/body"])
        w, x, y, z = _quat_x_to(d)
        j.CreateLocalPos0Attr(Gf.Vec3f(*(pt - STRIP_POS[k - 1]).tolist()))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(pt - STRIP_POS[k]).tolist()))
        j.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        j.CreateLocalRot1Attr(Gf.Quatf(w, x, y, z))
        j.CreateAxisAttr("X")
        j.CreateCollisionEnabledAttr(False)
        dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        dr.CreateTypeAttr("force")
        dr.CreateStiffnessAttr(float(K_CREASE if HINGE_IS_CREASE[k] else K_BOARD))
        dr.CreateDampingAttr(float(DAMP))
        dr.CreateTargetPositionAttr(float(CREASE_SET if HINGE_IS_CREASE[k] else 0.0))
        dr.CreateMaxForceAttr(1.0e3)
        HINGE_PATHS.append(path)


bodykit.EXTRA_AUTHORS.append(_author)


# -- reading the scene ------------------------------------------------------------------------------------------------------

def _arr(x) -> np.ndarray:
    if hasattr(x, "torch"):
        x = x.torch
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array((aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz))


def _qinv(q):
    return np.array((-q[0], -q[1], -q[2], q[3]))


AXES = [d for _, d in _HINGES]  # each hinge's axis in its strips' (unrotated, start) frames


def hinge_angles(quats, prev=None) -> np.ndarray:
    """Each hinge's angle (deg): strip k turned relative to strip k-1 about the hinge line, 0 in the flat pack;
    unwrapped against ``prev``."""
    out = np.zeros(G.N_FACETS)
    for k in range(G.N_FACETS):
        qr = _qmul(_qinv(quats[k - 1]), quats[k])
        if qr[3] < 0:
            qr = -qr
        a = math.degrees(2 * math.atan2(float(qr[:3] @ AXES[k]), float(qr[3])))
        if prev is not None:
            a = prev[k] + ((a - prev[k] + 180.0) % 360.0) - 180.0
        out[k] = a
    return out


def follow_set(angle: np.ndarray, rest: np.ndarray) -> np.ndarray:
    """The board's set after bending to ``angle``: it moves only once a hinge is past its band."""
    band = np.where(HINGE_IS_CREASE, BAND_CREASE, BAND_BOARD)
    return np.clip(rest, angle - band, angle + band)


class Model:
    """Per control step: strip and cup poses, the hinges' set, and the contact forces the scorer needs."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.env = None
        self.rest = np.where(HINGE_IS_CREASE, CREASE_SET, 0.0)
        self.angle = np.zeros(G.N_FACETS)
        self.attrs = None
        self.views = None
        self.set_on_sim = np.zeros(G.N_FACETS)

    def attach(self, env):
        if self.env is env and self.views is not None:
            return
        self.env = env
        base = env.unwrapped
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        stage = get_current_stage()
        self.attrs = [UsdPhysics.DriveAPI(stage.GetPrimAtPath(p), "angular").GetTargetPositionAttr() for p in HINGE_PATHS]
        for a, r in zip(self.attrs, self.rest):
            a.Set(float(r))
        self.set_on_sim = self.rest.copy()
        psv = base.scene["cup"]._physics_sim_view
        cup = base.scene["cup"].root_view.prim_paths[0]
        strips = [base.scene[n].root_view.prim_paths[0] for n in STRIPS]
        self.links, self.link_side = [], []
        for side in ("left", "right"):
            for q in base.scene[f"{side}_robot"].root_view.link_paths[0]:
                if stage.GetPrimAtPath(q).HasAPI(UsdPhysics.RigidBodyAPI):
                    self.links.append(q)
                    self.link_side.append(side)
        self.link_side = np.array(self.link_side)
        self.views = (psv.create_rigid_contact_view([cup] + strips, filter_patterns=[self.links] * (1 + len(strips))),
                      psv.create_rigid_contact_view(strips, filter_patterns=[[cup]] * len(strips)))
        self.dt = base.sim.get_physics_dt()

    def poses(self):
        base = self.env.unwrapped
        origin = _arr(base.scene.env_origins)[0]
        out = {}
        for n in ["cup"] + STRIPS:
            p = _arr(base.scene[n].data.root_link_pose_w)[0]
            out[n] = (p[:3] - origin, p[3:7])
        return out

    def step(self) -> dict:
        """Update the set and read everything the scorer and the log need (world frame)."""
        P = self.poses()
        quats = [P[n][1] for n in STRIPS]
        self.angle = hinge_angles(quats, self.angle)
        self.rest = follow_set(self.angle, self.rest)
        for k in range(G.N_FACETS):
            if abs(self.rest[k] - self.set_on_sim[k]) > 0.2:
                self.attrs[k].Set(float(self.rest[k]))
                self.set_on_sim[k] = self.rest[k]
        robot_f = _arr(self.views[0].get_contact_force_matrix(self.dt))  # (17, links, 3)
        cup_f = _arr(self.views[1].get_contact_force_matrix(self.dt))[:, 0, :]  # (16, 3) strip <- cup
        return {"poses": P, "robot_f": robot_f, "strip_cup_f": cup_f, "link_side": self.link_side,
                "angle": self.angle.copy(), "rest": self.rest.copy()}


def squeeze_by_hand(robot_f, strip_cup_f, link_side, cup_quat, touch: float = 0.05) -> dict:
    """The force across the cup's wall from the hands: over every robot contact on the cup, and on strips pressed
    against the cup, the horizontal (in the cup's frame) forces' sum of sizes less the size of their sum, halved. A
    two-finger pinch of F reads F, and so do two hands pressing from opposite sides; a one-sided push reads 0.
    Returned as {"hands": N, "left": .., "right": ..} (each hand's own pinch, for the log)."""
    R = G.rotmat(cup_quat)
    on_cup = np.linalg.norm(strip_cup_f, axis=1) > touch

    def pinch(mask):
        fs = [robot_f[0, mask, :]]
        for k in range(len(strip_cup_f)):
            if on_cup[k]:
                fs.append(robot_f[1 + k, mask, :])
        fh = (np.vstack(fs) @ R)[:, :2]
        return float((np.linalg.norm(fh, axis=1).sum() - np.linalg.norm(fh.sum(axis=0))) / 2)

    out = {"hands": pinch(np.ones(len(link_side), bool))}
    for side in ("left", "right"):
        out[side] = pinch(link_side == side)
    return out
