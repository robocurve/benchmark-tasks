"""Write the scene's USD assets.

    python build_manikin.py      # first: the manikin's meshes and landmarks (build/manikin.npz, assets/landmarks.npz)
    python build_assets.py       # needs pxr (Isaac Sim's Python), not the simulator

- assets/manikin.usda: the infant CPR manikin, a floating articulation. The torso (root: CoACD convex pieces of the
  body) carries the head on a revolute neck about y (the hinge is the centre of the occiput's curve; a sphere about it
  is the occiput's collider, so the head rests on the mat; the face collider is the head's surface as an SDF) and the
  chest patch with the flag on a prismatic slide along z that only task code moves (no collider). Its origin is on the
  mat top under the crown, on the midline; axes as the station's.
- assets/bvm.usda: the bag-valve mask, a floating articulation rooted at the mask's dome (origin at the middle of the
  cushion's contact line, z up the stem, x toward the mask's wide (chin) end). 12 cushion segments slide along z
  (sprung); the valve link (swivel connector, valve housing, expiratory port and the bag's rigid core) turns about the
  stem (the swivel); the bag's 4 walls slide radially (sprung).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

import geometry as g

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets"
SKIN = (0.93, 0.76, 0.66)            # light skin
CUSHION = (0.62, 0.78, 0.92)         # the air cushion (tinted)
DOME = (0.86, 0.90, 0.93)            # clear PVC dome
VALVE = (0.95, 0.80, 0.20)           # the patient valve
BAG = (0.84, 0.90, 0.93)             # the bag (frosted, slightly translucent silicone)
CARD = (0.97, 0.97, 0.95)
INK = (0.03, 0.03, 0.03)


def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, opacity=None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    if opacity is not None:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def mesh(stage, path, V, F, mat=None, collide=False, approx="none", visible=True, smooth=False):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in V]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(F)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in np.asarray(F).ravel()]))
    m.CreateSubdivisionSchemeAttr("none")
    if smooth:
        import trimesh
        n = trimesh.Trimesh(np.asarray(V), np.asarray(F), process=False).vertex_normals
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in n]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if mat is not None:
        bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(approx)
    if not visible:
        m.CreatePurposeAttr("guide")
    return m


def hull_mesh(stage, path, P, visible=False, mat=None):
    from scipy.spatial import ConvexHull
    P = np.asarray(P, float)
    h = ConvexHull(P)
    F = h.simplices.copy()
    c = P.mean(0)
    for k, f in enumerate(F):                          # outward winding
        n = np.cross(P[f[1]] - P[f[0]], P[f[2]] - P[f[0]])
        if n @ (P[f[0]] - c) < 0:
            F[k] = f[[0, 2, 1]]
    return mesh(stage, path, P, F, mat, collide=True, approx="convexHull", visible=visible)


def body(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0), rot=None, inertia=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    if rot is not None:
        xf.AddOrientOp().Set(Gf.Quatf(*rot))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(mass)
    if inertia is not None:                             # links with no collider need their inertia given
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    return xf


def contact_report(prim):
    prim.AddAppliedSchema("PhysxContactReportAPI")
    prim.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.0)


def articulation_root(prim):
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)


def quat_wxyz(R) -> tuple:
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return (0.25 / s, (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return ((R[k, j] - R[j, k]) / s, q[0], q[1], q[2])


def joint(stage, path, kind, body0, body1, pos0, rot0=(1, 0, 0, 0), axis="Z", limits=None, stiffness=0.0, damping=0.0,
          target=0.0):
    j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if limits is not None:
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(target)
    return j


def capsule(stage, path, r, h, axis="X", pos=(0, 0, 0), mat=None):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(h)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    if mat is not None:
        bind(c, mat)
    return c


def cyl_points(r, x0, x1, n=24, axis="x", r1=None):
    r1 = r if r1 is None else r1
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    out = []
    for x, rr in ((x0, r), (x1, r1)):
        c, s = rr * np.cos(a), rr * np.sin(a)
        if axis == "x":
            out.append(np.c_[np.full(n, x), c, s])
        else:
            out.append(np.c_[c, s, np.full(n, x)])
    return np.vstack(out)


def tube_mesh(profile, n=36):
    """A closed surface of revolution about x: profile [(x, r), ...] (r 0 at the ends closes them)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    V, F = [], []
    for x, r in profile:
        V.extend(np.c_[np.full(n, x), r * np.cos(a), r * np.sin(a)])
    for k in range(len(profile) - 1):
        for i in range(n):
            j = (i + 1) % n
            F += [[k * n + i, (k + 1) * n + i, (k + 1) * n + j], [k * n + i, (k + 1) * n + j, k * n + j]]
    V = np.array(V)
    c0, c1 = len(V), len(V) + 1
    V = np.vstack([V, [profile[0][0], 0, 0], [profile[-1][0], 0, 0]])
    last = (len(profile) - 1) * n
    for i in range(n):
        j = (i + 1) % n
        F += [[c0, i, j], [c1, last + j, last + i]]
    return V, np.array(F)


# -- the manikin -------------------------------------------------------------------------------------------------------
def build_manikin():
    d = np.load(HERE / "build" / "manikin.npz", allow_pickle=True)
    L = np.load(OUT / "landmarks.npz")
    stage, root = stage_for("manikin")
    r = "/manikin"
    skin = material(stage, f"{r}/Looks/skin", SKIN, 0.6)
    card = material(stage, f"{r}/Looks/card", CARD, 0.7)
    ink = material(stage, f"{r}/Looks/ink", INK, 0.8)
    torso = body(stage, f"{r}/torso", g.TORSO_MASS)
    articulation_root(torso.GetPrim())
    contact_report(torso.GetPrim())
    mesh(stage, f"{r}/torso/skin", d["torso_V"], d["torso_F"], skin, smooth=True)
    for i, (V, F) in enumerate(zip(d["hull_V"], d["hull_F"])):
        hull_mesh(stage, f"{r}/torso/hull_{i:02d}", V)
    hinge = L["hinge"]
    head = body(stage, f"{r}/head", g.HEAD_MASS, hinge)
    contact_report(head.GetPrim())
    mesh(stage, f"{r}/head/skin", d["head_V"], d["head_F"], skin, smooth=True)
    face = mesh(stage, f"{r}/head/face", d["col_V"], d["col_F"], None, collide=True, approx="sdf", visible=False)
    face.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
    face.GetPrim().CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(300)
    s = UsdGeom.Sphere.Define(stage, f"{r}/head/occiput")
    s.CreateRadiusAttr(float(L["r_occ"]) + g.OCCIPUT_PRELOAD)     # a hair into the mat: it carries the head
    s.CreatePurposeAttr("guide")
    UsdPhysics.CollisionAPI.Apply(s.GetPrim())
    joint(stage, f"{r}/neck", "angular", f"{r}/torso", f"{r}/head", hinge, axis="Y",
          limits=(-g.NECK_LIMITS[1], -g.NECK_LIMITS[0]), stiffness=g.NECK_K * math.pi / 180,
          damping=g.NECK_D * math.pi / 180)
    # the chest patch and flag: no collider, moved only by task code (its slide's drive follows the lungs)
    st = L["sternum"]
    chest = body(stage, f"{r}/chest", 0.004, st, inertia=1e-6)
    mesh(stage, f"{r}/chest/patch", d["patch_V"], d["patch_F"], skin, smooth=True)
    f = UsdGeom.Cube.Define(stage, f"{r}/chest/flag")
    f.CreateSizeAttr(1.0)
    f.AddTranslateOp().Set(Gf.Vec3d(0, 0, g.FLAG_H / 2))
    f.AddScaleOp().Set(Gf.Vec3f(g.FLAG_W, g.FLAG_T, g.FLAG_H))
    bind(f, card)
    for side in (1, -1):
        c = UsdGeom.Cylinder.Define(stage, f"{r}/chest/dot_{'l' if side > 0 else 'r'}")
        c.CreateRadiusAttr(g.DOT_D / 2)
        c.CreateHeightAttr(0.0003)
        c.CreateAxisAttr("Y")
        c.AddTranslateOp().Set(Gf.Vec3d(0, side * (g.FLAG_T / 2 + 0.00016), g.DOT_UP))
        bind(c, ink)
    joint(stage, f"{r}/rise", "linear", f"{r}/torso", f"{r}/chest", st, limits=(0.0, 0.02), stiffness=2000.0,
          damping=20.0)
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{r}/head"))
    fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{r}/torso")])
    stage.GetRootLayer().Save()


# -- the bag-valve mask --------------------------------------------------------------------------------------------------
def dome_surface(rim_h):
    """The dome: from the outline (at the cushion's top, following the face) up to its flat top (thumb rest)."""
    out = g.mask_outline(96)
    rows = []
    levels = np.linspace(0, 1, 9)
    for t in levels:
        k = 1 - 0.55 * t ** 1.6                         # shrink toward the top plateau
        z = (1 - t ** 0.8) * rim_h + t ** 0.8 * g.DOME_TOP
        rows.append(np.c_[out * k, z])
    V = np.vstack(rows)
    n = len(out)
    F = []
    for a in range(len(rows) - 1):
        for i in range(n):
            j = (i + 1) % n
            F += [[a * n + i, a * n + j, (a + 1) * n + j], [a * n + i, (a + 1) * n + j, (a + 1) * n + i]]
    c = len(V)
    V = np.vstack([V, [[0.0, 0.0, g.DOME_TOP]]])
    last = (len(rows) - 1) * n
    F += [[last + i, last + (i + 1) % n, c] for i in range(n)]
    return V, np.array(F)


def build_bvm():
    L = np.load(OUT / "landmarks.npz")
    face_h, out_h = L["face_h"], L["out_h"]
    stage, root = stage_for("bvm")
    r = "/bvm"
    dome_m = material(stage, f"{r}/Looks/dome", DOME, 0.15, opacity=0.55)
    cush_m = material(stage, f"{r}/Looks/cushion", CUSHION, 0.3)
    valve_m = material(stage, f"{r}/Looks/valve", VALVE, 0.35)
    bag_m = material(stage, f"{r}/Looks/bag", BAG, 0.35, opacity=0.8)
    # the dome (root): the shell is shown; its collider is the cap above DOME_CAP_FROM, plus the stem
    dome = body(stage, f"{r}/dome", g.MASS_DOME)
    articulation_root(dome.GetPrim())
    contact_report(dome.GetPrim())
    rim_top = out_h - g.SEG_NOM + 2 * g.SEG_R           # the cushion's top along the outline
    V, F = dome_surface(rim_top)
    mesh(stage, f"{r}/dome/shell", V, F, dome_m)
    hull_mesh(stage, f"{r}/dome/cap", V[V[:, 2] >= g.DOME_CAP_FROM])
    hull_mesh(stage, f"{r}/dome/stem", cyl_points(g.STEM_R, g.DOME_TOP - 0.002, g.DOME_TOP + g.STEM_H, axis="z"),
              visible=True, mat=dome_m)
    # the cushion: 12 segments along the contact line, each a capsule along the rim, sprung along z
    cp, tan = g.contact_points()
    for i, (p, t) in enumerate(zip(cp, tan)):
        z = face_h[i] - g.SEG_NOM + g.SEG_R             # the capsule's axis: its tip 2 mm below the face, nominally
        a = math.atan2(t[1], t[0])
        rot = (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))
        b = body(stage, f"{r}/seg_{i:02d}", g.MASS_SEG, (p[0], p[1], z), rot, inertia=5e-8)
        contact_report(b.GetPrim())
        capsule(stage, f"{r}/seg_{i:02d}/cushion", g.SEG_R, g.SEG_LEN, "X", mat=cush_m)
        joint(stage, f"{r}/joints/seg_{i:02d}", "linear", f"{r}/dome", f"{r}/seg_{i:02d}", (p[0], p[1], z), rot0=rot,
              limits=(0.0, g.SEG_TRAVEL), stiffness=g.SEG_K, damping=g.SEG_D, target=-g.SEG_PRELOAD / g.SEG_K)
    # the valve link: swivel connector, valve housing along -x (toward the bag) .. +x (expiratory port), bag core
    valve = body(stage, f"{r}/valve", g.MASS_VALVE + g.MASS_BAG_CORE)
    contact_report(valve.GetPrim())
    z0 = g.DOME_TOP + g.STEM_H
    hull_mesh(stage, f"{r}/valve/swivel", cyl_points(g.SWIVEL_R, z0 - 0.003, z0 + g.SWIVEL_H + 0.004, axis="z"),
              visible=True, mat=valve_m)
    zc = g.VALVE_Z
    hv = cyl_points(g.VALVE_R, -g.VALVE_L / 2, g.VALVE_L / 2) + [0, 0, zc]
    hull_mesh(stage, f"{r}/valve/housing", hv, visible=True, mat=valve_m)
    hp = cyl_points(g.PORT_R, g.VALVE_L / 2, g.VALVE_L / 2 + g.PORT_L) + [0, 0, zc]
    hull_mesh(stage, f"{r}/valve/port", hp, visible=True, mat=valve_m)
    # the bag along -x from the valve: neck, cone up to 71 mm, (the walls), cone down to the inlet end
    u0, um = g.BAG_U0, g.BAG_MID
    w0, w1 = um - g.WALL_L / 2, um + g.WALL_L / 2
    pieces = [((u0, g.BAG_NECK_R), (u0 + 0.012, g.BAG_NECK_R)),
              ((u0 + 0.012, g.BAG_NECK_R), (w0, g.BAG_R)),
              ((w1, g.BAG_R), (g.BAG_U0 + g.BAG_L, 0.020))]
    flat = -(g.BAG_R - g.BAG_FLAT)                      # the bag's underside flattens where it rests
    for k, ((a0, r0), (a1, r1)) in enumerate(pieces):
        P = cyl_points(r0, -a0, -a1, r1=r1)
        P[:, 2] = np.maximum(P[:, 2], flat)
        P = P + [0, 0, zc]
        hull_mesh(stage, f"{r}/valve/bag_{k}", P, visible=False)          # collider (hidden)
    # drawn: the two ends as smooth surfaces of revolution (shoulders rounded, at most 2.5 mm off the colliders)
    for k, (xa, xb, ra, rb) in enumerate(((u0, w0, g.BAG_NECK_R, g.BAG_R), (w1, u0 + g.BAG_L, g.BAG_R, 0.020))):
        tt = np.linspace(0.0, 1.0, 17)
        if k == 0:
            prof = [(-u0, g.BAG_NECK_R)] + [(-(xa + 0.012 + (xb - xa - 0.012) * t),
                                             ra + (rb - ra) * (0.5 * t + 0.5 * math.sin(0.5 * math.pi * t))) for t in tt]
        else:
            prof = [(-(xa + (xb - xa) * t), ra + (rb - ra) * (0.5 * t + 0.5 * (1 - math.cos(0.5 * math.pi * t))))
                    for t in tt]
        V, F = tube_mesh(prof, n=72)
        V[:, 2] = np.maximum(V[:, 2], flat)
        mesh(stage, f"{r}/valve/bag_skin_{k}", V + [0, 0, zc], F, bag_m, smooth=True)
    # the 4 walls: 90-degree shells round the middle 100 mm, sliding inward along their middle radius
    for name, ang in (("top", 90.0), ("left", 0.0), ("bottom", -90.0), ("right", 180.0)):
        th = math.radians(ang)                          # direction in the bag's y-z plane: (y, z) = (cos, sin)
        e = np.array([0.0, math.cos(th), math.sin(th)])
        n_in = -e
        # the wall's body frame: origin on the bag axis at the middle; its z = inward
        zc_ = n_in
        xc = np.array([1.0, 0, 0])
        yc = np.cross(zc_, xc)
        Rw = np.column_stack([xc, yc, zc_])
        q = quat_wxyz(Rw)
        pos = (-um, 0.0, zc)
        wb = body(stage, f"{r}/wall_{name}", g.MASS_WALL, pos, q)
        contact_report(wb.GetPrim())
        # collider: a thick block under the wall's crown (a pad flattens the soft wall it presses), 24 mm wide and
        # 15 mm deep, 4 mm short of the core's cones; drawn as a 90-degree shell
        zf = g.BAG_R - (g.BAG_FLAT if name == "bottom" else 0.0)
        P = np.array([[x, y, -z] for x in (-g.WALL_L / 2 + 0.004, g.WALL_L / 2 - 0.004)
                      for y in (-g.WALL_FLAT / 2, g.WALL_FLAT / 2) for z in (zf, zf - g.WALL_DEPTH)])
        hull_mesh(stage, f"{r}/wall_{name}/block", P, visible=False)
        aa = np.linspace(-math.radians(46.0), math.radians(46.0), 33)
        V = np.array([[x, g.BAG_R * math.sin(t), -g.BAG_R * math.cos(t)] for x in (-g.WALL_L / 2, g.WALL_L / 2)
                      for t in aa])
        F = []
        for k in range(len(aa) - 1):
            F += [[k, k + 1, len(aa) + k + 1], [k, len(aa) + k + 1, len(aa) + k]]
        mesh(stage, f"{r}/wall_{name}/skin", V, np.array(F), bag_m, smooth=True)
        joint(stage, f"{r}/joints/wall_{name}", "linear", f"{r}/valve", f"{r}/wall_{name}", pos, rot0=q,
              limits=(0.0, g.WALL_TRAVEL), stiffness=g.WALL_K, damping=g.WALL_D, target=-g.WALL_PRELOAD / g.WALL_K)
    joint(stage, f"{r}/joints/swivel", "angular", f"{r}/dome", f"{r}/valve", (0, 0, 0), damping=g.SWIVEL_D)
    # no collisions between the BVM's own parts, whatever the articulation's self-collision setting
    walls = [f"{r}/wall_{n}" for n in ("top", "left", "bottom", "right")]
    segs = [f"{r}/seg_{i:02d}" for i in range(g.N_SEG)]
    parts = [f"{r}/dome", f"{r}/valve"] + walls + segs
    for path in parts:
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path))
        fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(o) for o in parts if o != path])
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    build_manikin()
    build_bvm()
    for p in sorted(OUT.glob("*.usda")):
        print(p.name, p.stat().st_size, "bytes")
