"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/head.usda: the dental training head, one kinematic body whose origin is the 31/41 contact: the 28-tooth
  typodont (lower and upper arches), the soft gums with their papillae, the face mask with its mouth opening, the
  mouth's dark interior, and the post. Triangle-mesh colliders on the front teeth, the lower gum and the mask.
- assets/lip.usda: the lower lip, an articulation with a fixed base and the lip on a prismatic joint along x (a sprung
  slide, 0-10 mm forward). Its shape follows the arch in front of the lower front teeth: back face on the teeth's
  labial faces up to 3 mm below the incisal edges, a rounded top edge, 10 mm thick; convex pieces for the collider.
- assets/pick.usda: the F-shaped floss pick, one rigid body (handle and two arms with tapered tips; 1 g). The floss
  itself is task code (floss.py), drawn as a display-only curve.

All meshes are generated here from the dimensions in geometry.py; nothing is scanned or downloaded.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from scipy.spatial import ConvexHull

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
IVORY = (0.93, 0.91, 0.84)
GUM = (0.86, 0.47, 0.52)
SKIN = (0.80, 0.60, 0.50)
LIPC = (0.76, 0.46, 0.42)
CAVITY = (0.32, 0.07, 0.09)
METAL = (0.55, 0.56, 0.58)
PICK = (0.42, 0.80, 0.72)
MM = 0.001


# -- USD helpers ---------------------------------------------------------------------------------------------------------
def stage_for(name: str, ext: str = "usda"):
    path = OUT / f"{name}.{ext}"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def offsets(prim, contact=0.0005, rest=0.0):
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def mesh(stage, path, V, F, mat=None, collide=False, approx="none", visible=True, smooth=True, off=None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in V]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(F)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in np.asarray(F).ravel()]))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if smooth:
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, n)) for n in vertex_normals(V, F)]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if mat is not None:
        bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(approx)
        if off is not None:
            offsets(m.GetPrim(), *off)
    if not visible:
        m.CreatePurposeAttr("guide")
    return m


def vertex_normals(V, F):
    V, F = np.asarray(V, float), np.asarray(F)
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    vn = np.zeros_like(V)
    for k in range(3):
        np.add.at(vn, F[:, k], fn)
    return vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)


def grid_faces(nu, nv, closed_v=False):
    """Triangles of an nu x nv vertex grid (row-major, v fastest)."""
    F = []
    vmax = nv if closed_v else nv - 1
    for i in range(nu - 1):
        for j in range(vmax):
            a, b = i * nv + j, i * nv + (j + 1) % nv
            c, d = a + nv, b + nv
            F += [[a, b, d], [a, d, c]]
    return np.array(F)


def hull_mesh(P):
    h = ConvexHull(P)
    F = h.simplices.copy()
    c = P.mean(0)
    for f in F:                                   # outward winding
        n = np.cross(P[f[1]] - P[f[0]], P[f[2]] - P[f[0]])
        if n @ (P[f[0]] - c) < 0:
            f[1], f[2] = f[2], f[1]
    return P, F


# -- teeth -----------------------------------------------------------------------------------------------------------------
def profile(kind: str, w: float, neck: float, central: bool):
    """(a(z), b(z), xc(z), z_top, z_bottom) of a crown in its own frame (z: the station's, H mm; x lingual; y distal
    from its centre)."""
    if central:
        return g.a_of, g.b_of, g.xc_of, g.Z_EDGE, -9.0
    k = neck / g.A_CERV
    b0 = w / 2

    def neck_taper(z):
        u = np.maximum(g.ZONE[0] - np.asarray(z, float), 0.0)
        return np.maximum(1.0 - 0.05 * u, 0.6)

    if kind == "incisor":
        def b(z):
            return g.b_of(z) - g.B0 + b0

        return (lambda z: g.a_of(z) * k), b, (lambda z: g.xc_of(z) * k), g.Z_EDGE, -9.0
    top = {"canine": g.Z_EDGE + 1.0, "premolar": g.Z_EDGE - 0.5, "molar": g.Z_EDGE - 1.0}[kind]
    dome = {"canine": 3.0, "premolar": 2.0, "molar": 1.6}[kind]
    pw = {"canine": 2.0, "premolar": 3.0, "molar": 4.0}[kind]

    def f(z):
        u = np.clip((np.asarray(z, float) - (top - dome)) / dome, 0.0, 1.0)
        return np.maximum((1.0 - u ** pw), 0.0) ** (1.0 / pw)

    def a(z):
        return neck * neck_taper(z) * np.maximum(f(z), 0.15)

    def bb(z):
        return b0 * neck_taper(z) * np.maximum(f(z), 0.15)

    return a, bb, (lambda z: 0.0 * np.asarray(z, float)), top, -9.0


def crown_mesh(kind, w, neck, central, n_theta=None):
    a_f, b_f, xc_f, top, bot = profile(kind, w, neck, central)
    if central:
        zs = np.r_[np.linspace(bot, -3.0, 10), np.linspace(-2.8, 0.6, 18), np.linspace(0.7, top, 14)]
    else:
        zs = np.r_[np.linspace(bot, top - 3.0, 6), np.linspace(top - 2.6, top, 9)]
    n_theta = n_theta or (56 if kind == "incisor" else 32)
    th = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    ct, st = np.cos(th), np.sin(th)
    n = g.N_SE if kind in ("incisor",) else 2.6
    ex = 2.0 / n
    cx, cy = np.sign(ct) * np.abs(ct) ** ex, np.sign(st) * np.abs(st) ** ex
    V = []
    for z in zs:
        a, b, xc = float(a_f(z)), float(b_f(z)), float(xc_f(z))
        V += [(xc + a * cx[i], b * cy[i], z) for i in range(n_theta)]
    V = np.array(V)
    F = grid_faces(len(zs), n_theta, closed_v=True)
    # caps
    nb = len(V)
    V = np.vstack([V, [[float(xc_f(zs[0])), 0.0, zs[0]], [float(xc_f(zs[-1])), 0.0, zs[-1] + 0.05]]])
    top_ring = (len(zs) - 1) * n_theta
    caps = [[nb, (i + 1) % n_theta, i] for i in range(n_theta)]
    caps += [[nb + 1, top_ring + i, top_ring + (i + 1) % n_theta] for i in range(n_theta)]
    return V, np.vstack([F, np.array(caps)])


def place_tooth(V, s, upper=False, scale=1.0, xshift=0.0):
    """Tooth-frame vertices (x lingual, y distal from its centre, z) -> H frame at arc length s."""
    p, nrm, th = g.arch(s, scale)
    side = 1.0 if s >= 0 else -1.0
    tangent = np.array([math.sin(th), side * math.cos(th), 0.0])
    out = p + np.outer(V[:, 0], nrm) + np.outer(V[:, 1], tangent) + np.outer(V[:, 2], [0, 0, 1.0])
    out[:, 0] += xshift
    if upper:
        out[:, 2] = g.UPPER_EDGE_Z + (g.Z_EDGE - out[:, 2])
    return out


def flip(F):
    return F[:, [0, 2, 1]]


# -- gums --------------------------------------------------------------------------------------------------------------------
def margin_z(s, contacts):
    d = np.min(np.abs(np.asarray(s)[..., None] - contacts[None]), axis=-1)
    return g.Z_NECK + (g.PAP_Z - g.Z_NECK) * np.exp(-(d / 1.3) ** 2)


def gum_mesh(teeth, scale=1.0, upper=False, xshift=0.0):
    con = g.contacts(teeth)
    s_end = g.GAP_HALF + sum(t[1] for t in teeth) + 1.5
    s = np.unique(np.r_[np.linspace(-s_end, -8, 45), np.linspace(-8, 8, 129), np.linspace(8, s_end, 45)])
    t = np.unique(np.r_[np.linspace(-7.0, -3.0, 9), np.linspace(-3.0, 3.0, 41), np.linspace(3.0, 6.0, 7)])
    S, Tt = np.meshgrid(s, t, indexing="ij")
    m = margin_z(S, con)
    T0 = 2.5
    slope = 2 * g.PAP_K * T0
    zt = np.where(np.abs(Tt) <= T0, m - g.PAP_K * Tt ** 2, m - g.PAP_K * T0 ** 2 - slope * (np.abs(Tt) - T0))
    zt = np.maximum(zt, -20.0)
    p, nrm, _ = g.arch(S.ravel(), scale)
    top = p + Tt.ravel()[:, None] * nrm
    top[:, 2] = zt.ravel()
    top[:, 0] += xshift
    bot = top.copy()
    bot[:, 2] = -20.0
    nu, nv = len(s), len(t)
    V = np.vstack([top, bot])
    F = list(grid_faces(nu, nv))
    # side walls: around the grid's border
    border = [(0, j) for j in range(nv)] + [(i, nv - 1) for i in range(1, nu)] + \
             [(nu - 1, j) for j in range(nv - 2, -1, -1)] + [(i, 0) for i in range(nu - 2, 0, -1)]
    idx = [i * nv + j for i, j in border]
    for k in range(len(idx)):
        a, b = idx[k], idx[(k + 1) % len(idx)]
        F += [[a, b + nu * nv, b], [a, a + nu * nv, b + nu * nv]]
    F = np.array(F)
    if upper:
        V[:, 2] = g.UPPER_EDGE_Z + (g.Z_EDGE - V[:, 2])
        F = flip(F)
    return V, F


# -- the face mask -------------------------------------------------------------------------------------------------------------
HEAD_C = np.array([70.0, 0.0, 40.0])
HEAD_R = np.array([85.0, 76.0, 112.0])
MOUTH_C = np.array([0.0, (g.Z_NECK + 52.0) / 2])          # (y, z) of the opening's centre
MOUTH_R = np.array([g.MOUTH_W / 2, (52.0 - g.Z_NECK) / 2])  # half-width, half-height
RIM = 10.0                                                   # the lips' depth


def face_offset(y, z):
    """Toward the robot (-x) is negative: nose, chin, brow, lips; eye sockets recessed."""
    def gauss(cy, cz, sy, sz):
        return np.exp(-0.5 * (((y - cy) / sy) ** 2 + ((z - cz) / sz) ** 2))

    d = -16.0 * gauss(0.0, 76.0, 7.0, 11.0) - 6.0 * gauss(0.0, -42.0, 16.0, 13.0)
    d += -4.0 * gauss(0.0, 112.0, 30.0, 6.0) + 7.0 * gauss(31.0, 100.0, 10.0, 7.0) + 7.0 * gauss(-31.0, 100.0, 10.0, 7.0)
    rho = np.sqrt((y / MOUTH_R[0]) ** 2 + ((z - MOUTH_C[1]) / MOUTH_R[1]) ** 2)
    d += -3.0 * np.exp(-((rho - 1.0) / 0.35) ** 2)
    return d


def head_x(y, z, front=True):
    u = 1 - (y / HEAD_R[1]) ** 2 - ((z - HEAD_C[2]) / HEAD_R[2]) ** 2
    r = HEAD_R[0] * np.sqrt(np.clip(u, 0, 1))
    return HEAD_C[0] - r + face_offset(y, z) * np.clip(u * 4, 0, 1) if front else HEAD_C[0] + r


def mask_mesh(n_phi=160, n_front=40, n_back=30):
    phi = np.linspace(0, 2 * np.pi, n_phi, endpoint=False)
    sil = np.stack([HEAD_R[1] * np.cos(phi), HEAD_C[2] + HEAD_R[2] * np.sin(phi)], 1) * 0.999 + \
        np.array([0.0, HEAD_C[2] * 0.001])
    # the hole's boundary point on each ray from the mouth's centre to the silhouette point
    d = sil - MOUTH_C
    k = 1.0 / np.sqrt((d[:, 0] / MOUTH_R[0]) ** 2 + (d[:, 1] / MOUTH_R[1]) ** 2)
    hole = MOUTH_C + d * k[:, None]
    u = np.linspace(0, 1, n_front)[:, None, None] ** 1.4
    YZ = hole[None] + u * (sil - hole)[None]                       # (n_front, n_phi, 2)
    Xf = head_x(YZ[..., 0], YZ[..., 1], True)
    front = np.stack([Xf, YZ[..., 0], YZ[..., 1]], -1).reshape(-1, 3)
    ub = np.linspace(1, 0, n_back)[1:-1, None, None]
    YZb = np.array([0.0, HEAD_C[2]]) + ub * (sil - np.array([0.0, HEAD_C[2]]))[None]
    Xb = head_x(YZb[..., 0], YZb[..., 1], False)
    back = np.stack([Xb, YZb[..., 0], YZb[..., 1]], -1).reshape(-1, 3)
    pole = np.array([[HEAD_C[0] + HEAD_R[0], 0.0, HEAD_C[2]]])
    V = np.vstack([front, back, pole])
    F = list(flip(grid_faces(n_front, n_phi, closed_v=True)))
    off = n_front * n_phi
    rings = [np.arange(n_phi) + (n_front - 1) * n_phi] + [off + r * n_phi + np.arange(n_phi) for r in range(n_back - 2)]
    for r0, r1 in zip(rings[:-1], rings[1:]):
        for j in range(n_phi):
            a, b = r0[j], r0[(j + 1) % n_phi]
            c, dd = r1[j], r1[(j + 1) % n_phi]
            F += [[a, dd, b], [a, c, dd]]
    pi = len(V) - 1
    last = rings[-1]
    F += [[last[j], pi, last[(j + 1) % n_phi]] for j in range(n_phi)]
    # the lips: the hole's edge carried RIM mm into the head
    hole_ring = np.arange(n_phi)
    hv = front[hole_ring].copy()
    inner = hv.copy()
    inner[:, 0] += RIM
    base = len(V)
    V = np.vstack([V, inner])
    for j in range(n_phi):
        a, b = hole_ring[j], hole_ring[(j + 1) % n_phi]
        c, dd = base + j, base + (j + 1) % n_phi
        F += [[a, b, dd], [a, dd, c]]
    return V, np.array(F)


def cavity_mesh():
    """The mouth's dark interior: back wall, floor, roof and cheeks (visual only)."""
    xb, yw, zf, zr, xf = 58.0, 36.0, -24.0, 74.0, 2.0
    boxes = []
    V, F = [], []

    def quad(p0, p1, p2, p3):
        b = len(V)
        V.extend([p0, p1, p2, p3])
        F.extend([[b, b + 1, b + 2], [b, b + 2, b + 3]])

    quad((xb, -yw, zf), (xb, yw, zf), (xb, yw, zr), (xb, -yw, zr))
    quad((xf, -yw, zf), (xb, -yw, zf), (xb, yw, zf), (xf, yw, zf))
    quad((xf, -yw, zr), (xf, yw, zr), (xb, yw, zr), (xb, -yw, zr))
    quad((-6, -yw, zf), (-6, -yw, zr), (xb, -yw, zr), (xb, -yw, zf))
    quad((-6, yw, zf), (xb, yw, zf), (xb, yw, zr), (-6, yw, zr))
    del boxes
    return np.array(V, float), np.array(F)


def cylinder(r, z0, z1, n=32, c=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    bot = np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.full(n, z0)]
    top = bot.copy()
    top[:, 2] = z1
    V = np.vstack([bot, top, [[c[0], c[1], z0], [c[0], c[1], z1]]])
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    return V, np.array(F)


def build_head():
    stage, root = stage_for("head", "usdc")
    r = "/head"
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr(True)
    ivory = material(stage, f"{r}/Looks/ivory", IVORY, roughness=0.35)
    gum = material(stage, f"{r}/Looks/gum", GUM, roughness=0.55)
    skin = material(stage, f"{r}/Looks/skin", SKIN, roughness=0.6)
    cav = material(stage, f"{r}/Looks/cavity", CAVITY, roughness=0.8)
    metal = material(stage, f"{r}/Looks/metal", METAL, roughness=0.35, metallic=0.6)
    off = (0.0003, 0.0)
    # lower arch
    for name, side, s, w, neck, kind in g.tooth_layout(g.LOWER_TEETH):
        central = name == "1"
        V, F = crown_mesh(kind, w, neck, central)
        W = place_tooth(V, s)
        num = f"{3 if side < 0 else 4}{name}"
        collide = name in ("1", "2", "3")
        mesh(stage, f"{r}/lower/t{num}", W * MM, F, ivory, collide=collide, off=off)
    V, F = gum_mesh(g.LOWER_TEETH)
    mesh(stage, f"{r}/lower/gum", V * MM, F, gum, collide=True, off=off)
    # upper arch: wider, its incisors over the lower ones and UPPER_X in front, the jaw open 48 mm
    for name, side, s, w, neck, kind in g.tooth_layout(g.UPPER_TEETH, start=0.0):
        V, F = crown_mesh(kind, w, neck, False)
        W = place_tooth(V, s, upper=True, scale=1.12, xshift=g.UPPER_X)
        num = f"{2 if side < 0 else 1}{name}"
        mesh(stage, f"{r}/upper/t{num}", W * MM, flip(F), ivory, collide=name in ("1", "2"), off=off)
    V, F = gum_mesh(g.UPPER_TEETH, scale=1.12, upper=True, xshift=g.UPPER_X)
    mesh(stage, f"{r}/upper/gum", V * MM, F, gum)
    # mask, interior, post
    V, F = mask_mesh()
    mesh(stage, f"{r}/mask", V * MM, F, skin, collide=True, off=(0.001, 0.0))
    V, F = cavity_mesh()
    mesh(stage, f"{r}/cavity", V * MM, F, cav, smooth=False)
    post_bottom = (g.BOARD_TOP - (g.CONTACT[2] - g.TOP)) * 1000
    V, F = cylinder(16.0, post_bottom, -60.0, c=(80.0, 0.0))
    mesh(stage, f"{r}/post", V * MM, F, metal, collide=True, smooth=False)
    stage.Save()


# -- the lower lip (articulation) ----------------------------------------------------------------------------------------------
LIP_R = 6.0          # the top edge's rounding (mm), tangent to the back face at LIP_TOP
LIP_FRONT_R = 6.0
LIP_LOW = -12.0


def lip_section(n_arc=10):
    """The lip's cross-section (t forward from its back face, z), convex, counter-clockwise."""
    pts = [(0.0, LIP_LOW)]
    for a in np.linspace(math.pi, math.pi / 2, n_arc):                 # back face -> top
        pts.append((LIP_R + LIP_R * math.cos(a), g.LIP_TOP + LIP_R * math.sin(a)))
    zt = g.LIP_TOP + LIP_R
    rf = g.LIP_T - LIP_R
    for a in np.linspace(math.pi / 2, 0.0, n_arc)[1:]:                  # top -> front (quarter circle)
        pts.append((LIP_R + rf * math.cos(a), zt - rf + rf * math.sin(a)))
    pts.append((g.LIP_T, LIP_LOW))
    return np.array(pts)


def lip_back_offset(s, z):
    """The lip's back face along the arch's normal (negative = labial), LIP_GAP in front of the teeth."""
    k = np.clip((np.abs(s) - 9.0) / 4.0, 0.0, 1.0)
    inc = g.xc_of(z) - g.a_of(z)
    can = (g.xc_of(z) - g.a_of(z)) * (3.8 / 3.0) - 0.4
    return (1 - k) * inc + k * can - g.LIP_GAP


def lip_points(s_vals, sec):
    """(len(s), len(sec), 3) H-frame points of the lip's surface."""
    out = []
    for s in s_vals:
        p, nrm, _ = g.arch(s)
        ring = []
        for t, z in sec:
            back = lip_back_offset(s, min(z, g.LIP_TOP))
            ring.append(p + (back - t) * nrm + np.array([0.0, 0.0, z]))
        out.append(ring)
    return np.array(out)


def build_lip():
    stage, root = stage_for("lip")
    r = "/lip"
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    root.GetPrim().AddAppliedSchema("PhysxArticulationAPI")
    root.GetPrim().CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    skin = material(stage, f"{r}/Looks/lip", LIPC, roughness=0.55)
    base = f"{r}/base"
    xf = UsdGeom.Xform.Define(stage, base)
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(0.05)
    UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint").CreateBody1Rel().SetTargets([Sdf.Path(base)])
    body = f"{r}/body"
    bx = UsdGeom.Xform.Define(stage, body)
    UsdPhysics.RigidBodyAPI.Apply(bx.GetPrim())
    UsdPhysics.MassAPI.Apply(bx.GetPrim()).CreateMassAttr(g.LIP_MASS)
    sec = lip_section()
    # visual: smooth sweep
    s_vis = np.linspace(-g.LIP_LEN / 2, g.LIP_LEN / 2, 81)
    P = lip_points(s_vis, sec)
    ns, nc = P.shape[:2]
    V = P.reshape(-1, 3)
    F = list(grid_faces(ns, nc, closed_v=True))
    c0, c1 = len(V), len(V) + 1
    V = np.vstack([V, P[0].mean(0), P[-1].mean(0)])
    F += [[c0, (j + 1) % nc, j] for j in range(nc)]
    F += [[c1, (ns - 1) * nc + j, (ns - 1) * nc + (j + 1) % nc] for j in range(nc)]
    mesh(stage, f"{body}/visual", V * MM, np.array(F), skin)
    # collider: convex pieces 4 mm along the arch
    edges = np.linspace(-g.LIP_LEN / 2, g.LIP_LEN / 2, 11)
    for i in range(10):
        Q = lip_points(np.linspace(edges[i], edges[i + 1], 3), sec).reshape(-1, 3)
        Vh, Fh = hull_mesh(Q)
        mesh(stage, f"{body}/col{i}", Vh * MM, Fh, None, collide=True, approx="convexHull", visible=False,
                 smooth=False, off=(0.0005, 0.0))
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{r}/joints/slide")
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(base)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body)])
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-g.LIP_TRAVEL)
    j.CreateUpperLimitAttr(0.0)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    stage.Save()


# -- the floss pick ----------------------------------------------------------------------------------------------------------------
TIP_T = 0.6          # the arms' tips: 0.6 mm thick, tapering over the lowest TAPER_H
TAPER_H = 1.5
TIP_BELOW = 0.5      # the tip's end below the floss


def arm_points(inner_x: float, outward: float):
    """A tapered arm: inner face at x = inner_x (where the floss is tied), the outer face slanting in toward the tip."""
    y0, y1 = -g.ARM_T / 2, g.ARM_T / 2
    zb, zt = -TIP_BELOW, g.ARM_L + 0.5
    zk = zb + TAPER_H
    xi, xo, xtip = inner_x, inner_x + outward * g.ARM_T, inner_x + outward * TIP_T
    P = []
    for y in (y0, y1):
        P += [(xi, y, zb), (xtip, y, zb), (xo, y, zk), (xo, y, zt), (xi, y, zt)]
    return np.array(P, float)


def build_pick():
    stage, root = stage_for("pick")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(g.PICK_MASS)
    p.AddAppliedSchema("PhysxRigidBodyAPI")
    p.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
    mat = material(stage, "/pick/Looks/plastic", PICK, roughness=0.4)
    r = "/pick"
    hx = (g.X_REAR, g.X_TIP)
    hy = (-g.HANDLE_T / 2, g.HANDLE_T / 2)
    hz = g.Z_HANDLE
    corners = np.array([(x, y, z) for x in hx for y in hy for z in hz])
    V, F = hull_mesh(corners)
    mesh(stage, f"{r}/handle", V * MM, F, mat, collide=True, approx="convexHull", smooth=False, off=(0.0005, 0.0))
    for name, inner, out in (("front_arm", g.ANCHOR_A[0], -1.0), ("back_arm", g.ANCHOR_B[0], 1.0)):
        V, F = hull_mesh(arm_points(inner, out))
        mesh(stage, f"{r}/{name}", V * MM, F, mat, collide=True, approx="convexHull", smooth=False,
             off=(0.0005, 0.0))
    stage.Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    build_head()
    build_lip()
    build_pick()
    for f in sorted(OUT.glob("*.usda")):
        print(f.name, f.stat().st_size // 1024, "KB")
