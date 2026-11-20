"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr, numpy, trimesh (Isaac Sim's Python), not the simulator

- assets/wheelchair.usda: the Drive Medical Blue Streak, open, one floating articulation in the chair frame:
  - frame (root): side frames, seat rails, the X cross-brace (held open), the sling seat and back, push handles with foam
    grips, desk-length arm pads with their bars, clothing guards, caster housings, the wheel-lock brackets;
  - wheel_l, wheel_r (revolute about y at the hubs): 24 in wheels, solid tyres, handrims, spokes;
  - fork_l, fork_r (revolute about z at the swivels) and caster_l, caster_r (revolute about y): 8 in casters, trailing
    toward the chair's front at joint 0;
  - lock_l, lock_r (revolute about y at the lock's pivot): each wheel lock's lever, knob and shoe, off at joint 0.
- assets/room.usda: one kinematic body holding everything that never moves: the twin platform bed frame, mattress, sheet
  and pillow, Mrs. Lee (the mannequin, lying on her back; visual mesh from the MakeHuman base mesh with its female-young
  targets, CC0; capsule colliders), the hip tick, the tape outline, the floor marks.

The mannequin mesh needs the MakeHuman files in build/dl_makehuman (download commands in build_mannequin's docstring).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets"
DL = HERE / "build" / "dl_makehuman"

BLUE = (0.05, 0.13, 0.42)           # powder-coated frame
CHROME = (0.80, 0.80, 0.82)
STEEL = (0.55, 0.56, 0.58)
TYRE = (0.03, 0.03, 0.032)
BLACK = (0.03, 0.03, 0.035)
UPHOL = (0.025, 0.025, 0.03)
GREY = (0.20, 0.20, 0.21)
WHITE = (0.93, 0.93, 0.91)
SHEET = (0.95, 0.95, 0.96)
BEDSTEEL = (0.12, 0.12, 0.13)
TAPE = (0.10, 0.35, 0.85)           # blue floor tape
TAPE_Y = (0.95, 0.80, 0.10)         # yellow tape (start ticks)
SKIN = (0.86, 0.68, 0.57)
GOWN = (0.30, 0.47, 0.58)
BLANKET = (0.74, 0.62, 0.46)
HEM = (0.93, 0.93, 0.92)
HAIR = (0.20, 0.12, 0.07)
IRIS = (0.42, 0.30, 0.19)


# -- meshes (numpy) ------------------------------------------------------------------------------------------------------
def join(*parts):
    V, F, n = [], [], 0
    for v, f in parts:
        V.append(np.asarray(v, float))
        F.append(np.asarray(f, int) + n)
        n += len(v)
    return np.vstack(V), np.vstack(F)


def tube(points, r, sides=12):
    """A closed tube along a polyline (densified so bends keep their radius)."""
    p = np.asarray(points, float)
    dense = [p[0]]
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(np.linalg.norm(b - a) / 0.03))
        dense += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    p = np.array(dense)
    d = np.gradient(p, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    u = np.cross(d[0], [0.0, 0.0, 1.0] if abs(d[0][2]) < 0.9 else [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    th = np.linspace(0.0, 2 * np.pi, sides, endpoint=False)
    rings = []
    for k in range(len(p)):
        u = u - u.dot(d[k]) * d[k]
        u /= np.linalg.norm(u)
        w = np.cross(d[k], u)
        rings.append(p[k] + r * (np.outer(np.cos(th), u) + np.outer(np.sin(th), w)))
    V = np.vstack(rings + [p[0][None], p[-1][None]])
    F = []
    for k in range(len(p) - 1):
        for i in range(sides):
            j = (i + 1) % sides
            a, b, c, e = k * sides + i, k * sides + j, (k + 1) * sides + j, (k + 1) * sides + i
            F += [(a, b, c), (a, c, e)]
    first, last = len(p) * sides, len(p) * sides + 1
    for i in range(sides):
        j = (i + 1) % sides
        F += [(first, j, i), (last, (len(p) - 1) * sides + i, (len(p) - 1) * sides + j)]
    return V, np.array(F)


def bend(a, corner, b, r=0.05, n=6):
    a, corner, b = (np.asarray(v, float) for v in (a, corner, b))
    u1 = (corner - a) / np.linalg.norm(corner - a)
    u2 = (b - corner) / np.linalg.norm(b - corner)
    p1, p2 = corner - u1 * r, corner + u2 * r
    return [a] + [(1 - t) ** 2 * p1 + 2 * (1 - t) * t * corner + t * t * p2 for t in np.linspace(0, 1, n + 1)] + [b]


def torus_y(major, minor, center, segments=64, sides=12):
    """A ring about the y axis."""
    V, F = [], []
    for a in np.linspace(0, 2 * np.pi, segments, endpoint=False):
        for b in np.linspace(0, 2 * np.pi, sides, endpoint=False):
            r = major + minor * math.cos(b)
            V.append((r * math.cos(a), minor * math.sin(b), r * math.sin(a)))
    for i in range(segments):
        i1 = (i + 1) % segments
        for k in range(sides):
            k1 = (k + 1) % sides
            F += [(i * sides + k, i1 * sides + k, i1 * sides + k1), (i * sides + k, i1 * sides + k1, i * sides + k1)]
    return np.array(V) + np.asarray(center, float), np.array(F)


def cyl(r, length, center, axis="z", segments=24):
    th = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = np.c_[r * np.cos(th), r * np.sin(th)]
    V = np.vstack([np.c_[ring, np.full(segments, -length / 2)], np.c_[ring, np.full(segments, length / 2)],
                   [[0, 0, -length / 2], [0, 0, length / 2]]])
    F = []
    for i in range(segments):
        j = (i + 1) % segments
        F += [(i, j, segments + j), (i, segments + j, segments + i), (2 * segments, j, i),
              (2 * segments + 1, segments + i, segments + j)]
    if axis == "x":
        V = V[:, [2, 0, 1]]
    elif axis == "y":
        V = V[:, [1, 2, 0]]
    return V + np.asarray(center, float), np.array(F)


def boxm(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    V = np.array([[x, y, z] for z in (lo[2], hi[2]) for y in (lo[1], hi[1]) for x in (lo[0], hi[0])])
    F = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6),
         (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    return V, np.array(F)


def rounded_box(lo, hi, r, n=4):
    """A box with rounded vertical edges and a rounded top edge (for the mattress and pillow), as one mesh."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = (lo + hi) / 2
    hx, hy = (hi[0] - lo[0]) / 2 - r, (hi[1] - lo[1]) / 2 - r
    ring = []
    for cx, cy, a0 in ((hx, -hy, -90.0), (hx, hy, 0.0), (-hx, hy, 90.0), (-hx, -hy, 180.0)):
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            ring.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    ring = np.array(ring)
    m = len(ring)
    off = ring - np.clip(ring, [-hx, -hy], [hx, hy])
    offn = off / np.maximum(np.linalg.norm(off, axis=1, keepdims=True), 1e-9)
    levels = [(0.0, lo[2])] + [(r * (1 - math.cos(math.radians(90.0 * kk / n))),
                                hi[2] - r + r * math.sin(math.radians(90.0 * kk / n))) for kk in range(n + 1)]
    V, F = [], []
    for shrink, z in levels:
        pts = ring - offn * shrink
        V += [(c[0] + p[0], c[1] + p[1], z) for p in pts]
    L = len(levels)
    for li in range(L - 1):
        for i in range(m):
            j = (i + 1) % m
            a, b, cc, d = li * m + i, li * m + j, (li + 1) * m + j, (li + 1) * m + i
            F += [(a, b, cc), (a, cc, d)]
    V += [(c[0], c[1], lo[2]), (c[0], c[1], hi[2])]
    bot, top = len(V) - 2, len(V) - 1
    for i in range(m):
        j = (i + 1) % m
        F += [(bot, j, i), (top, (L - 1) * m + i, (L - 1) * m + j)]
    return np.array(V), np.array(F)


def wheel_hull_points(center, radius, width, segments):
    """A cylinder about y as hull points; half a segment's turn puts a flat face at the bottom."""
    th = np.linspace(0, 2 * np.pi, segments, endpoint=False) + np.pi / segments
    ring = np.c_[radius * np.cos(th), np.zeros(segments), radius * np.sin(th)]
    return np.vstack([ring + center + (0, -width / 2, 0), ring + center + (0, width / 2, 0)])


def wheel_wedges(radius, width, segments, pieces=4):
    """A wheel (cylinder about y, a flat face at the bottom) as ``pieces`` convex sector wedges, each under the GPU's
    64-vertex hull limit, so the rim is a ``segments``-gon."""
    out = []
    per = segments // pieces
    for k in range(pieces):
        th = (np.arange(per + 1) + k * per) * 2 * np.pi / segments + np.pi / segments
        arc = np.c_[radius * np.cos(th), np.zeros(per + 1), radius * np.sin(th)]
        pts = np.vstack([arc, [[0.0, 0.0, 0.0]]])
        out.append(np.vstack([pts + (0, -width / 2, 0), pts + (0, width / 2, 0)]))
    return out


def hull_faces(P):
    from scipy.spatial import ConvexHull

    h = ConvexHull(P)
    c = P.mean(0)
    F = []
    for f in h.simplices:
        a, b, e = P[f]
        F.append(f if np.dot(np.cross(b - a, e - a), a - c) > 0 else f[[0, 2, 1]])
    return np.array(F)


# -- USD helpers -------------------------------------------------------------------------------------------------------------
def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path, color, roughness=0.5, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def phys_material(stage, path, static, dynamic):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(static))
    api.CreateDynamicFrictionAttr(float(dynamic))
    api.CreateRestitutionAttr(0.0)
    return mat


def bind(prim, mat, purpose=None) -> None:
    api = UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    if purpose:
        api.Bind(mat, UsdShade.Tokens.weakerThanDescendants, purpose)
    else:
        api.Bind(mat)


REC: list = []          # (body, kind, data) of every collider, for the scorer's distance checks


def _tag(path: str) -> str:
    if path.startswith("/room"):
        return "lee" if "/lee_col/" in path else "bed"
    return path.split("/")[2]


def collider(prim, pmat=None, contact=0.004, rest=0.0):
    prim = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)
    UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
    if pmat is not None:
        bind(prim, pmat, "physics")


def mesh(stage, path, V, F, mat=None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in F])
    m.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if mat is not None:
        bind(m, mat)
    return m


def hull(stage, path, P, pmat=None):
    m = mesh(stage, path, P, hull_faces(P))
    collider(m, pmat)
    m.GetPrim().AddAppliedSchema("PhysxConvexHullCollisionAPI")
    m.GetPrim().CreateAttribute("physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int).Set(64)
    REC.append((_tag(path), "hull", np.asarray(P, float)))
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def quat_from_z(d):
    d = np.asarray(d, float) / np.linalg.norm(d)
    axis = np.cross([0.0, 0.0, 1.0], d)
    s, c = np.linalg.norm(axis), d[2]
    if s < 1e-12:
        return Gf.Quatf(1, 0, 0, 0) if c > 0 else Gf.Quatf(0, 1, 0, 0)
    axis /= s
    a = math.atan2(s, c)
    return Gf.Quatf(math.cos(a / 2), *map(float, axis * math.sin(a / 2)))


def capsule(stage, path, a, b, r, pmat=None):
    """A capsule collider from a to b (cylinder ends), radius r."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(float(r))
    c.CreateHeightAttr(float(np.linalg.norm(b - a)))
    c.CreateAxisAttr("Z")
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2)))
    c.AddOrientOp().Set(quat_from_z(b - a))
    collider(c, pmat)
    REC.append((_tag(path), "capsule", np.r_[a, b, r]))
    return c


def box_col(stage, path, lo, hi, pmat=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
    collider(c, pmat)
    REC.append((_tag(path), "box", np.r_[lo, hi]))
    return c


def body(stage, path, mass, pos, com, inertia):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    prim = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(float(mass))
    m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    prim.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(1.0)
    return xf


def joint(stage, path, body0, body1, pos0, axis, limits_deg=None):
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if limits_deg is not None:
        j.CreateLowerLimitAttr(float(limits_deg[0]))
        j.CreateUpperLimitAttr(float(limits_deg[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


def slider(stage, path, body0, body1, pos0, axis, travel):
    """A prismatic joint along ``axis``, +-``travel`` m (its spring is the joint drive, set by the task file)."""
    j = UsdPhysics.PrismaticJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-float(travel))
    j.CreateUpperLimitAttr(float(travel))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


# -- the wheelchair ------------------------------------------------------------------------------------------------------------
def build_chair():
    stage, root = stage_for("wheelchair")
    r = "/wheelchair"
    look = {k: material(stage, f"{r}/Looks/{k}", *v) for k, v in dict(
        frame=(BLUE, 0.32, 0.55), chrome=(CHROME, 0.18, 1.0), steel=(STEEL, 0.35, 0.9), tyre=(TYRE, 0.85),
        black=(BLACK, 0.55), uphol=(UPHOL, 0.75), grey=(GREY, 0.5)).items()}
    pm_tyre = phys_material(stage, f"{r}/Physics/tyre", g.TYRE_MU, g.TYRE_MU)
    pm_grip = phys_material(stage, f"{r}/Physics/grip", *g.GRIP_MU)
    W, rt = g.W_RAIL, g.R_TUBE

    # frame (articulation root): both side frames, the X held open, seat, back, arms, guards, lock brackets
    fb = f"{r}/frame"
    I = g.FRAME_MASS / 12 * np.array([0.5 ** 2 + 0.9 ** 2, 0.65 ** 2 + 0.9 ** 2, 0.65 ** 2 + 0.5 ** 2])
    prim = body(stage, fb, g.FRAME_MASS, (0, 0, 0), g.FRAME_COM, I).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    prim.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    vis = {k: [] for k in look}
    k = 0

    def cap(a, b, rad, pm=pm_grip, show=None, sides=12):
        nonlocal k
        capsule(stage, f"{fb}/col{k}", a, b, rad, pm)
        k += 1
        if show:
            vis[show].append(tube([a, b], rad, sides))

    def bx(lo, hi, pm=pm_grip, show=None):
        nonlocal k
        box_col(stage, f"{fb}/col{k}", lo, hi, pm)
        k += 1
        if show:
            vis[show].append(boxm(lo, hi))

    for s in (1.0, -1.0):
        y = s * W
        out = s
        cap((g.RAIL_X[0], y, g.RAIL_Z), (g.RAIL_X[1], y, g.RAIL_Z), g.R_RAIL, show="frame", sides=14)       # seat rail
        vis["black"].append(cyl(g.R_RAIL + 0.001, 0.02, (g.RAIL_X[1] - 0.006, y, g.RAIL_Z), "x", 16))         # end caps
        vis["black"].append(cyl(g.R_RAIL + 0.001, 0.02, (g.RAIL_X[0] + 0.006, y, g.RAIL_Z), "x", 16))
        cap((g.X_FRONT, y, g.HOUSING_Z + 0.03), (g.X_FRONT, y, g.RAIL_Z - 0.012), rt, show="frame")         # front post
        drop = [(g.X_FRONT, y, g.HOUSING_Z), (0.27, y, g.LOWER_Z)]
        cap(*drop, rt, show="frame")
        cap((0.27, y, g.LOWER_Z), (g.X_CANE, y, g.LOWER_Z), rt, show="frame")                                 # lower rail
        cane = bend((g.X_CANE, y, g.LOWER_Z), (g.X_CANE, y, g.HANDLE_Z), (g.HANDLE_X[0], y, g.HANDLE_Z), r=0.05)
        vis["frame"].append(tube(cane, rt))
        cap((g.X_CANE, y, g.LOWER_Z), (g.X_CANE, y, g.HANDLE_Z - 0.04), rt)
        cap((g.X_CANE - 0.03, y, g.HANDLE_Z), (g.HANDLE_X[0], y, g.HANDLE_Z), 0.0145)                         # handle
        vis["black"].append(cyl(0.0145, g.HANDLE_X[1] - g.HANDLE_X[0], ((g.HANDLE_X[0] + g.HANDLE_X[1]) / 2, y,
                                                                        g.HANDLE_Z), "x", 16))              # foam grip
        cap((g.X_FRONT, y, g.SWIVEL_Z + 0.012), (g.X_FRONT, y, g.HOUSING_Z + 0.045), 0.019, show="steel", sides=16)
        bx((-0.038, y + out * 0.012 - 0.004, 0.21), (0.062, y + out * 0.012 + 0.004, 0.37), show="steel")    # axle plate
        vis["chrome"].append(tube([(0.0, y, g.R_WHEEL), (0.0, s * (g.HUB_Y - 0.01), g.R_WHEEL)], 0.008, 10))
        # rear saddle bracket under the rail, behind the seat
        vis["frame"].append(tube([(g.X_CANE, y, g.RAIL_Z - 0.02), (0.06, y, g.RAIL_Z - 0.02)], 0.009))
        # desk arm: pad, bar under it, post down to the cane
        ax, ay, az = g.ARM_PAD["x"], g.ARM_PAD["y"], g.ARM_PAD["z"]
        ylo, yhi = sorted((y + out * ay[0], y + out * ay[1]))
        bx((ax[0], ylo, az[0]), (ax[1], yhi, az[1]), show="black")
        ym = y + out * (ay[0] + ay[1]) / 2
        cap((ax[1] - 0.02, ym, az[0] - 0.012), (g.X_CANE + 0.005, ym, az[0] - 0.012), 0.010, show="frame")
        post = [(g.X_CANE + 0.005, ym, az[0] - 0.012), (g.X_CANE + 0.005, ym, az[0] - 0.12),
                (g.X_CANE + 0.005, y + out * 0.012, az[0] - 0.14)]
        vis["frame"].append(tube(post, rt))
        cap(post[0], post[2], rt)
        # clothing guard
        gy = y + out * g.GUARD["y"]
        bx((g.GUARD["x"][0], gy - g.GUARD["t"] / 2, g.GUARD["z"][0]), (g.GUARD["x"][1], gy + g.GUARD["t"] / 2,
                                                                         g.GUARD["z"][1]), show="black")
        # wheel-lock bracket: a flat bar clamped to the seat rail, down to the pivot inboard of the lever, and the boss
        by = s * (g.KNOB_Y - 0.0215)
        bx((g.PIVOT[0] - 0.012, by - 0.003, g.PIVOT[2] - 0.02), (g.PIVOT[0] + 0.012, by + 0.003, g.RAIL_Z - 0.005),
           show="steel")
        ylo, yhi = sorted((by, s * (g.KNOB_Y - 0.004)))
        bx((g.PIVOT[0] - 0.011, ylo, g.PIVOT[2] - 0.012), (g.PIVOT[0] + 0.011, yhi, g.PIVOT[2] + 0.012), show="black")
    # X cross-brace (held open), sling seat, back
    for s in (1.0, -1.0):
        xb = g.X_BRACE + s * 0.013
        cap((xb, -s * W, g.LOWER_Z), (xb, s * W, g.RAIL_Z - 0.015), rt, show="frame")
    bx((g.SLING_X[0], -W + g.R_RAIL, g.SEAT_TOP - 0.006), (g.SLING_X[1], W - g.R_RAIL, g.SEAT_TOP), show="uphol")
    vis["uphol"].append(tube([(g.SLING_X[0], W, g.RAIL_Z), (g.SLING_X[1], W, g.RAIL_Z)], g.R_RAIL + 0.003, 14))
    vis["uphol"].append(tube([(g.SLING_X[0], -W, g.RAIL_Z), (g.SLING_X[1], -W, g.RAIL_Z)], g.R_RAIL + 0.003, 14))
    bx((g.X_CANE - 0.018, -W, g.BACK_Z[0]), (g.X_CANE - 0.010, W, g.BACK_Z[1]), show="uphol")
    for name, parts in vis.items():
        if parts:
            mesh(stage, f"{fb}/visual/{name}", *join(*parts), look[name])

    # rear wheels
    for side, s in (("l", 1.0), ("r", -1.0)):
        hub = np.array([0.0, s * g.HUB_Y, g.R_WHEEL])
        wb = f"{r}/wheel_{side}"
        m = g.WHEEL_MASS
        body(stage, wb, m, hub, (0, 0, 0), (0.33 * m * g.R_WHEEL ** 2, 0.62 * m * g.R_WHEEL ** 2,
                                            0.33 * m * g.R_WHEEL ** 2))
        for kk, P in enumerate(wheel_wedges(g.R_WHEEL, g.TYRE_W, 96)):
            hull(stage, f"{wb}/tyre_col{kk}", P, pm_tyre)
        mesh(stage, f"{wb}/visual/tyre", *torus_y(g.R_WHEEL - g.TYRE_R, g.TYRE_R, (0, 0, 0), 72, 12), look["tyre"])
        rim = join(torus_y(g.R_WHEEL - 0.030, 0.007, (0, 0.007, 0), 64, 8), torus_y(g.R_WHEEL - 0.030, 0.007,
                                                                                      (0, -0.007, 0), 64, 8))
        mesh(stage, f"{wb}/visual/rim", *rim, look["chrome"])
        hr = np.array([0.0, s * g.HANDRIM_OUT, 0.0])
        mesh(stage, f"{wb}/visual/handrim", *torus_y(g.HANDRIM_R, g.HANDRIM_TUBE, hr, 64, 10), look["chrome"])
        spokes = []
        for kk in range(24):
            a = 2 * math.pi * kk / 24
            p0 = np.array([0.03 * math.cos(a), 0.022 if kk % 2 else -0.022, 0.03 * math.sin(a)])
            p1 = np.array([(g.R_WHEEL - 0.035) * math.cos(a + 0.25), 0.0, (g.R_WHEEL - 0.035) * math.sin(a + 0.25)])
            spokes.append(tube([p0, p1], 0.0012, 4))
        tabs = [tube([[(g.R_WHEEL - 0.035) * math.cos(a), s * 0.002, (g.R_WHEEL - 0.035) * math.sin(a)],
                      [g.HANDRIM_R * math.cos(a), s * g.HANDRIM_OUT, g.HANDRIM_R * math.sin(a)]], 0.004, 6)
                for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)]
        mesh(stage, f"{wb}/visual/spokes", *join(*spokes, *tabs), look["chrome"])
        mesh(stage, f"{wb}/visual/hub", *cyl(0.032, 0.06, (0, 0, 0), "y", 20), look["steel"])
        joint(stage, f"{r}/axle_{side}", fb, wb, hub, "Y")

    # casters: stem (on a stiff spring in the housing), fork (swivel about z) and wheel; at joint 0 the wheel trails
    # toward the chair's front
    for side, s in (("l", 1.0), ("r", -1.0)):
        sw = np.array([g.X_FRONT, s * g.W_RAIL, g.SWIVEL_Z])
        st = f"{r}/stem_{side}"
        body(stage, st, g.STRUT_MASS, sw, (0, 0, 0.02), (1e-4, 1e-4, 1e-4))
        slider(stage, f"{r}/spring_{side}", fb, st, sw, "Z", g.SPRING_TRAVEL)
        fk = f"{r}/fork_{side}"
        body(stage, fk, g.FORK_MASS, sw, (g.CASTER_TRAIL / 2, 0, -0.04), (4e-4, 4e-4, 2e-4))
        ax_local = np.array([g.CASTER_TRAIL, 0.0, g.R_CASTER - g.SWIVEL_Z])
        blades = [tube([(0.01, o * 0.024, -0.01), (ax_local[0], o * 0.024, ax_local[2])], 0.007, 8) for o in (-1, 1)]
        crown = boxm((-0.012, -0.028, -0.014), (g.CASTER_TRAIL / 2 + 0.03, 0.028, -0.002))
        stem = tube([(0, 0, -0.01), (0, 0, 0.06)], 0.008, 10)
        mesh(stage, f"{fk}/visual/fork", *join(*blades, crown), look["steel"])
        mesh(stage, f"{fk}/visual/stem", *stem, look["chrome"])
        joint(stage, f"{r}/swivel_{side}", st, fk, (0, 0, 0), "Z")
        cw = f"{r}/caster_{side}"
        m = g.CASTER_MASS
        body(stage, cw, m, sw + ax_local, (0, 0, 0), (0.25 * m * g.R_CASTER ** 2, 0.5 * m * g.R_CASTER ** 2,
                                                      0.25 * m * g.R_CASTER ** 2))
        for kk, P in enumerate(wheel_wedges(g.R_CASTER, g.CASTER_W, 64)):
            hull(stage, f"{cw}/tyre_col{kk}", P, pm_tyre)
        mesh(stage, f"{cw}/visual/tyre", *torus_y(g.R_CASTER - 0.014, 0.014, (0, 0, 0), 40, 10), look["grey"])
        mesh(stage, f"{cw}/visual/hub", *cyl(g.R_CASTER - 0.026, 0.026, (0, 0, 0), "y", 28), look["black"])
        joint(stage, f"{r}/caster_axle_{side}", fk, cw, ax_local, "Y")

    # wheel locks: lever, knob and shoe on a pivot; joint 0 = off (leaning back), forward = toward on
    for side, s in (("l", 1.0), ("r", -1.0)):
        pv = g.PIVOT * np.array([1.0, s, 1.0])
        lb = f"{r}/lock_{side}"
        knob = (g.KNOB_OFF - g.PIVOT) * np.array([1.0, 0.0, 1.0])
        com = knob * 0.6
        body(stage, lb, g.LEVER_MASS, pv, com, (2.5e-4, 2.5e-4, 2e-5))
        th_k = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        ring = np.c_[g.KNOB_R * np.cos(th_k), np.zeros(16), g.KNOB_R * np.sin(th_k)]
        hull(stage, f"{lb}/knob_col", np.vstack([ring + knob - (0, g.KNOB_LEN / 2, 0), ring + knob + (0, g.KNOB_LEN / 2, 0)]),
             pm_grip)
        capsule(stage, f"{lb}/shaft_col", (0, 0, 0), knob * 0.88, 0.005, pm_grip)
        mesh(stage, f"{lb}/visual/knob", *cyl(g.KNOB_R, g.KNOB_LEN, knob, "y", 20), look["black"])
        shaft = tube([(0, 0, 0), knob * 0.9], 0.0055, 10)
        # the shoe: on a short arm below and behind the pivot, it swings back into the tyre as the lever goes forward
        u = knob / np.linalg.norm(knob)
        back = -np.array([u[2], 0.0, -u[0]])            # perpendicular to the lever, toward the tyre
        tyre_gap_pt = np.array([math.sqrt(g.R_WHEEL ** 2 - (g.PIVOT[2] - 0.025 - g.R_WHEEL) ** 2), 0.0,
                                g.PIVOT[2] - 0.025]) - g.PIVOT * np.array([1, 0, 1])
        shoe_c = tyre_gap_pt + np.array([g.SHOE_GAP + 0.008, 0.0, 0.0])
        arm = tube([(0, 0, 0), shoe_c], 0.005, 8)
        shoe = boxm(shoe_c + (-0.008, -0.010, -0.012), shoe_c + (0.008, 0.010, 0.012))
        mesh(stage, f"{lb}/visual/lever", *join(shaft, arm), look["steel"])
        mesh(stage, f"{lb}/visual/shoe", *shoe, look["black"])
        lim = (0.0, math.degrees(g.TH_ON - g.TH_OFF))
        joint(stage, f"{r}/lock_joint_{side}", fb, lb, pv, "Y", lim)
        _ = back
    stage.Save()


# -- the mannequin (MakeHuman) ---------------------------------------------------------------------------------------------
def read_obj(path):
    V, groups, grp = [], {}, None
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                V.append([float(t) for t in line.split()[1:4]])
            elif line.startswith("g "):
                grp = line.split()[1]
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                groups.setdefault(grp, []).extend([idx[0], idx[kk], idx[kk + 1]] for kk in range(1, len(idx) - 1))
    return np.array(V), {kk: np.array(v) for kk, v in groups.items()}


def apply_target(V, path, w):
    off = np.zeros_like(V)
    for line in open(path):
        if line[:1].isdigit():
            t = line.split()
            off[int(t[0])] += np.array([float(x) for x in t[1:4]])
    return V + w * off


def build_mannequin():
    """Mrs. Lee from the MakeHuman base mesh (hm08) with its female-young macro targets (the three ethnic targets mixed
    equally, MakeHuman's default, plus the universal female-young average-muscle average-weight target), CC0:

        cd build/dl_makehuman
        curl -L -O https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/3dobjs/base.obj
        for e in african asian caucasian; do curl -L -O \\
          https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/targets/macrodetails/$e-female-young.target
        done
        curl -L -O https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/targets/macrodetails/universal-female-young-averagemuscle-averageweight.target

    Scaled to 178 cm, the A-pose arms lowered to her sides, laid on her back. Returns world-frame meshes by look and the
    capsule colliders."""
    V, groups = read_obj(DL / "base.obj")
    for e in ("african", "asian", "caucasian"):
        V = apply_target(V, DL / f"{e}-female-young.target", 1.0 / 3.0)
    V = apply_target(V, DL / "universal-female-young-averagemuscle-averageweight.target", 1.0)
    body_idx = np.unique(groups["body"])
    lo, hi = V[body_idx, 1].min(), V[body_idx, 1].max()
    V = (V - [0.0, lo, 0.0]) * (g.LEE_HEIGHT / (hi - lo))           # metres, feet at y 0

    def jpos(name):
        return V[np.unique(groups[f"joint-{name}"])].mean(0)

    # turn each arm about its shoulder from MakeHuman's A pose to lie along her side (7 degrees out, back to the bed),
    # blended over the shoulder; the arm is everything beyond the shoulder's side, above the hips
    for side, sgn in (("l", 1.0), ("r", -1.0)):
        sh, hand = jpos(f"{side}-shoulder"), jpos(f"{side}-hand")
        d = hand - sh
        n = np.linalg.norm(d)
        want = np.array([sgn * math.sin(math.radians(7.0)), -math.cos(math.radians(7.0)), 0.0]) * n
        want[2] = 0.02
        u0, u1 = d / n, want / np.linalg.norm(want)
        axis = np.cross(u0, u1)
        axis /= np.linalg.norm(axis)
        ang = math.acos(float(np.clip(u0 @ u1, -1, 1)))
        t = (V - sh) @ u0
        w = np.clip((t + 0.02) / 0.06, 0, 1)
        el = jpos(f"{side}-elbow")
        bones = [(sh, el), (el, hand), (hand, hand + 0.15 * (hand - el) / np.linalg.norm(hand - el))]
        dist = np.min([np.linalg.norm(V - (a + np.clip(((V - a) @ (b_ - a)) / ((b_ - a) @ (b_ - a)), 0, 1)[:, None]
                                            * (b_ - a)), axis=1) for a, b_ in bones], axis=0)
        f = np.clip((0.14 - dist) / 0.03, 0, 1)
        w = w * w * (3 - 2 * w) * f * f * (3 - 2 * f)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        P = V - sh
        a = (ang * w)[:, None]
        V = sh + P * np.cos(a) + (P @ K.T) * np.sin(a) + np.outer(P @ axis, axis) * (1 - np.cos(a))
    J = {n: jpos(n) for n in ("head", "neck", "pelvis", "l-shoulder", "r-shoulder", "l-elbow", "r-elbow", "l-hand",
                              "r-hand", "l-upper-leg", "r-upper-leg", "l-knee", "r-knee", "l-ankle", "r-ankle",
                              "spine-1", "spine-3")}
    hip_mh = (J["l-upper-leg"] + J["r-upper-leg"]) / 2
    # MakeHuman: x her left, y up, z forward -> world: up -> +x (head toward the bed's head), forward -> +z (face up),
    # her left -> -y
    M = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    tilt = math.radians(2.0)                                        # her head end resting 3 cm up on the pillow
    Rt = np.array([[math.cos(tilt), 0, -math.sin(tilt)], [0, 1, 0], [math.sin(tilt), 0, math.cos(tilt)]])
    W_ = (V - hip_mh) @ M.T @ Rt.T
    core = body_idx[np.abs(V[body_idx, 0]) < 0.13]                  # torso and legs, not the arms
    back = np.percentile(W_[core, 2], 0.5)
    hip_z_off = -back                                               # hip joint above her back and buttocks
    shift = np.array([g.HIP_X, g.LEE_Y, g.FZ + g.MATT_TOP_H + hip_z_off - 0.008])
    Wd = W_ + shift

    def tow(p):
        return (np.asarray(p) - hip_mh) @ M.T @ Rt.T + shift

    JW = {n: tow(p) for n, p in J.items()}
    crown = Wd[body_idx][np.argmax(Wd[body_idx, 0])]
    # looks per face
    F = groups["body"]
    Vb = V
    cen = Vb[F].mean(1)
    knee_y = (J["l-knee"][1] + J["r-knee"][1]) / 2
    sh_y = (J["l-shoulder"][1] + J["r-shoulder"][1]) / 2
    elbow_y = (J["l-elbow"][1] + J["r-elbow"][1]) / 2
    half_w = abs(J["l-shoulder"][0] - J["r-shoulder"][0]) / 2
    in_arm = np.abs(cen[:, 0]) > half_w * 0.80
    gown = (cen[:, 1] > knee_y + 0.08) & (cen[:, 1] < sh_y + 0.035) & (~in_arm | (cen[:, 1] > (elbow_y + sh_y) / 2))
    gown &= (Vb[F][:, :, 1] < sh_y + 0.05).all(1)              # a clean neckline: no face reaching up the neck
    Fh = groups["helper-hair"]                               # the wig: the long-hair helper cut to the scalp (above
    ch_h = V[Fh].mean(1)                                     # the ears, and behind the face)
    head_j = J["head"]
    scalp = (ch_h[:, 1] > head_j[1] + 0.035) | ((ch_h[:, 1] > head_j[1] - 0.06) & (ch_h[:, 2] < head_j[2] - 0.035))
    eye_c = V[np.unique(np.r_[groups["helper-l-eye"].ravel(), groups["helper-r-eye"].ravel()])].mean(0)
    back_of_brow = (V[:, 2] < eye_c[2] - 0.02) | (V[:, 1] > eye_c[1] + 0.06)    # no fringe over her face
    scalp &= back_of_brow[Fh].all(1)
    parts = {"skin": [F[~gown]], "gown": [F[gown]], "hair": [Fh[scalp]],
             "lash": [groups[k] for k in groups if "eyelashes" in k]}
    eyes = {"sclera": [], "iris": [], "pupil": []}
    for side in ("l", "r"):
        Fe = groups[f"helper-{side}-eye"]
        ve = np.unique(Fe)
        c = V[ve].mean(0)
        fc = V[Fe].mean(1) - c
        cosang = fc[:, 2] / np.linalg.norm(fc, axis=1)
        eyes["pupil"].append(Fe[cosang > 0.988])
        eyes["iris"].append(Fe[(cosang > 0.92) & (cosang <= 0.988)])
        eyes["sclera"].append(Fe[cosang <= 0.92])
    parts.update(eyes)
    gown_f = parts.pop("gown")[0]
    meshes = {kk: (Wd, np.vstack(v)) for kk, v in parts.items() if sum(len(x) for x in v)}
    meshes["gown"] = (loose(Wd, gown_f, F, 0.008, 6), gown_f)
    top_x = (JW["l-shoulder"][0] + JW["r-shoulder"][0]) / 2 - 0.085
    meshes.update(blanket(Wd[body_idx], top_x))
    caps = [("head", JW["head"] - (0.01, 0, 0), JW["head"] + (0.05, 0, 0), 0.095),
            ("torso", JW["pelvis"], (JW["l-shoulder"] + JW["r-shoulder"]) / 2, 0.135)]
    for s in ("l", "r"):
        caps += [(f"{s}_upper_arm", JW[f"{s}-shoulder"], JW[f"{s}-elbow"], 0.05),
                 (f"{s}_forearm", JW[f"{s}-elbow"], JW[f"{s}-hand"], 0.04),
                 (f"{s}_thigh", JW[f"{s}-upper-leg"], JW[f"{s}-knee"], 0.075),
                 (f"{s}_shin", JW[f"{s}-knee"], JW[f"{s}-ankle"], 0.055)]
    info = dict(crown=crown, hip=(JW["l-upper-leg"] + JW["r-upper-leg"]) / 2, joints=JW,
                lowest_z=float(Wd[body_idx, 2].min()), width_y=(float(Wd[body_idx, 1].min()), float(Wd[body_idx, 1].max())),
                feet_x=float(Wd[body_idx, 0].min()))
    return meshes, caps, info


def loose(V, F, F_all, offset, iters):
    """A garment over faces F of mesh V: the vertices those faces use, smoothed (Laplacian, ``iters`` passes, the
    opening edges kept) and moved ``offset`` m out along the body's normals (from all its faces, F_all)."""
    from collections import defaultdict

    W = V.copy()
    used = np.unique(F)
    nb = defaultdict(set)
    edges = defaultdict(int)
    for f in F:
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            nb[a].add(b)
            nb[b].add(a)
            edges[(min(a, b), max(a, b))] += 1
    rim = {v for e, n in edges.items() if n == 1 for v in e}
    inner = np.array([v for v in used if v not in rim])
    nbl = [np.array(sorted(nb[v])) for v in inner]
    for _ in range(iters):
        W[inner] = 0.5 * W[inner] + 0.5 * np.array([W[n].mean(0) for n in nbl])
    N = np.zeros_like(W)
    Ft = np.asarray(F_all)
    if Ft.shape[1] == 4:                                       # quads -> two triangles
        Ft = np.r_[Ft[:, [0, 1, 2]], Ft[:, [0, 2, 3]]]
    fn = np.cross(V[Ft[:, 1]] - V[Ft[:, 0]], V[Ft[:, 2]] - V[Ft[:, 0]])
    for k in range(3):
        np.add.at(N, Ft[:, k], fn)
    N[used] /= np.linalg.norm(N[used], axis=1, keepdims=True) + 1e-12
    wt = np.ones(len(W))
    ring = {n for v in rim for n in nb[v]}
    wt[list(ring)] = 0.5
    wt[list(rim)] = 0.0                                   # the openings lie on the skin: no lip
    W[used] += offset * (wt[used, None] * N[used])
    return W


def blanket(Vbody, top_x):
    """A blanket over Mrs. Lee from just below her shoulders to the bed's foot, across the mattress, its sides and foot
    hanging 8 cm down the mattress and a turned-down sheet hem along its top edge: a height field over her body
    (dilated 4 cm, softened 2.5 cm, 1.2 cm clear of her)."""
    from scipy.ndimage import gaussian_filter, maximum_filter

    h = 0.01
    x0, x1 = g.BED_X[0] - 0.005, top_x
    y0, y1 = g.BED_EDGE_Y - g.BED_W - 0.005, g.BED_EDGE_Y + 0.005
    xs, ys = np.arange(x0, x1 + 1e-9, h), np.arange(y0, y1 + 1e-9, h)
    mt = g.FZ + g.MATT_TOP_H
    raw = np.full((len(xs), len(ys)), mt)
    ix = np.clip(np.round((Vbody[:, 0] - x0) / h).astype(int), 0, len(xs) - 1)
    iy = np.clip(np.round((Vbody[:, 1] - y0) / h).astype(int), 0, len(ys) - 1)
    inside = (Vbody[:, 0] >= x0 - h) & (Vbody[:, 0] <= x1 + 0.04)
    np.maximum.at(raw, (ix[inside], iy[inside]), Vbody[inside, 2])
    soft = gaussian_filter(maximum_filter(raw, size=9), 2.5, mode="nearest") + 0.012
    Z = np.maximum(soft, maximum_filter(raw, size=3) + 0.006)
    Z = np.maximum(Z, mt + 0.004)
    Z[:, 0] = Z[:, -1] = mt + 0.004                       # down to the mattress at its long edges ...
    Z[0, :] = mt + 0.004                                  # ... and at its foot
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    V = np.c_[X.ravel(), Y.ravel(), Z.ravel()]
    nx, ny = len(xs), len(ys)
    idx = np.arange(nx * ny).reshape(nx, ny)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
    F = np.r_[np.c_[a, b, c], np.c_[a, c, d]]
    # the skirts: down 8 cm along both long edges and the foot
    skirts = []
    for line in (idx[:, 0], idx[:, -1], idx[0, :]):
        top = V[line]
        bot = top - (0, 0, 0.08)
        skirts.append((np.r_[top, bot], len(top)))
    out = {"blanket": (V, F)}
    SV, SF, base = [], [], 0
    for P, n in skirts:
        SV.append(P)
        k = np.arange(n - 1)
        SF.append(np.r_[np.c_[base + k, base + k + 1, base + n + k + 1], np.c_[base + k, base + n + k + 1, base + n + k]])
        base += 2 * n
    out["blanket_skirt"] = (np.vstack(SV), np.vstack(SF))
    # the hem: the top sheet turned down over the blanket's top 7 cm, 4 mm proud
    rows = xs >= x1 - 0.07
    Vh = V.reshape(nx, ny, 3)[rows].reshape(-1, 3) + (0, 0, 0.004)
    nh = int(rows.sum())
    idh = np.arange(nh * ny).reshape(nh, ny)
    a, b, c, d = idh[:-1, :-1].ravel(), idh[1:, :-1].ravel(), idh[1:, 1:].ravel(), idh[:-1, 1:].ravel()
    out["hem"] = (Vh, np.r_[np.c_[a, b, c], np.c_[a, c, d]])
    print(f"blanket: x {x0:.3f}..{x1:.3f}, under-cover check {float((raw + 0.004 - Z).max()) * 1000:.1f} mm "
          f"(<= 0: clear of her everywhere)")
    return out


# -- the room: bed, Mrs. Lee, tape ---------------------------------------------------------------------------------------
def build_room():
    stage, root = stage_for("room")
    r = "/room"
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    look = {k: material(stage, f"{r}/Looks/{k}", *v) for k, v in dict(
        sheet=(SHEET, 0.85), bedsteel=(BEDSTEEL, 0.5, 0.3), tape=(TAPE, 0.6), tape_y=(TAPE_Y, 0.6), skin=(SKIN, 0.45),
        gown=(GOWN, 0.95), hair=(HAIR, 0.8), lash=((0.16, 0.11, 0.08), 0.7), sclera=((0.90, 0.88, 0.85), 0.35),
        iris=(IRIS, 0.35), pupil=((0.03, 0.02, 0.02), 0.35), pillow=((0.97, 0.97, 0.98), 0.85),
        blanket=(BLANKET, 0.95), blanket_skirt=(BLANKET, 0.95), hem=(HEM, 0.9)).items()}
    pm = phys_material(stage, f"{r}/Physics/plain", 0.6, 0.5)
    F0 = g.FZ
    x0, x1 = g.BED_X
    y1, y0 = g.BED_EDGE_Y, g.BED_EDGE_Y - g.BED_W
    k = [0]

    def bx(lo, hi, lk=None, collide=True):
        if collide:
            box_col(stage, f"{r}/col{k[0]}", lo, hi, pm)
        if lk:
            mesh(stage, f"{r}/vis{k[0]}", *boxm(lo, hi), look[lk])
        k[0] += 1

    # platform frame: perimeter rails (steel angle), slat deck, legs at the corners and middles
    top = F0 + g.BED_FRAME_H
    t = 0.035
    bx((x0, y1 - t, top - 0.04), (x1, y1, top), "bedsteel")
    bx((x0, y0, top - 0.04), (x1, y0 + t, top), "bedsteel")
    bx((x0, y0, top - 0.04), (x0 + t, y1, top), "bedsteel")
    bx((x1 - t, y0, top - 0.04), (x1, y1, top), "bedsteel")
    bx((x0 + t, y0 + t, top - 0.012), (x1 - t, y1 - t, top - 0.004), "bedsteel")
    for xx in (x0, (x0 + x1) / 2 - 0.02, x1 - 0.04):
        for yy in (y0, y1 - 0.04):
            bx((xx, yy, F0), (xx + 0.04, yy + 0.04, top - 0.04), "bedsteel")
    # mattress with its fitted sheet, and the pillow
    mt = F0 + g.MATT_TOP_H
    box_col(stage, f"{r}/mattress_col", (x0, y0, top), (x1, y1, mt), pm)
    mesh(stage, f"{r}/mattress", *rounded_box((x0, y0, top), (x1, y1, mt), 0.03), look["sheet"])
    pl = g.PILLOW
    pc = g.LEE_Y
    box_col(stage, f"{r}/pillow_col", (pl["x"][0], pc - pl["w"] / 2, mt), (pl["x"][1], pc + pl["w"] / 2, mt + 0.045), pm)
    mesh(stage, f"{r}/pillow", *rounded_box((pl["x"][0], pc - pl["w"] / 2, mt - 0.01),
                                            (pl["x"][1], pc + pl["w"] / 2, mt + pl["t"] - 0.02), 0.035), look["pillow"])
    # Mrs. Lee
    meshes, caps, info = build_mannequin()
    for name, (Vw, Fw) in meshes.items():
        used = np.unique(Fw)
        remap = -np.ones(len(Vw), int)
        remap[used] = np.arange(len(used))
        mesh(stage, f"{r}/lee/{name}", Vw[used], remap[Fw], look[name])
    for name, a, b, rad in caps:
        capsule(stage, f"{r}/lee_col/{name}", a, b, rad, pm)
    # tape: the hip tick on the mattress's near edge (top and side), the outline round her, floor marks
    tw = 0.019
    hx = g.HIP_X
    mesh(stage, f"{r}/hip_tick", *join(boxm((hx - tw / 2, y1 - 0.06, mt), (hx + tw / 2, y1 + 0.0005, mt + 0.0015)),
                                        boxm((hx - tw / 2, y1, mt - 0.08), (hx + tw / 2, y1 + 0.0015, mt))), look["tape"])
    ox0, ox1 = info["feet_x"] - 0.04, info["crown"][0] + 0.03
    oy0, oy1 = info["width_y"][0] - 0.04, min(info["width_y"][1] + 0.04, y1 - 0.025)
    zt = mt + 0.001
    outline = [boxm((ox0, oy0, mt), (ox1, oy0 + 0.012, zt)), boxm((ox0, oy1 - 0.012, mt), (ox1, oy1, zt)),
               boxm((ox0, oy0, mt), (ox0 + 0.012, oy1, zt))]
    mesh(stage, f"{r}/outline", *join(*outline), look["tape"])
    floor = []
    for xx, yy, sx, sy in ((x0, y1, 1, 1), (x1, y1, -1, 1)):          # corner L marks on the near side
        floor += [boxm((min(xx, xx + sx * 0.15), yy, F0), (max(xx, xx + sx * 0.15), yy + sy * tw, F0 + 0.0008)),
                  boxm((xx, min(yy, yy + 0.15 * sy), F0), (xx + sx * tw, max(yy, yy + 0.15 * sy), F0 + 0.0008))]
    mesh(stage, f"{r}/floor_marks", *join(*floor), look["tape"])
    ticks = []
    for s in (1.0, -1.0):                                            # the start's hub plumb points
        p = g.chair_to_world((0.0, s * g.HUB_Y, 0.0), g.START_M, g.START_YAW)
        ticks.append(boxm((p[0] - 0.06, p[1] - tw / 2, F0), (p[0] + 0.06, p[1] + tw / 2, F0 + 0.0008)))
    mesh(stage, f"{r}/start_ticks", *join(*ticks), look["tape_y"])
    stage.Save()
    return info


def save_colliders(info) -> None:
    """assets/colliders.npz: every collider in its body's frame (chair bodies at their joint origins, the room's in
    world), and Mrs. Lee's capsules, for the scorer's distance checks."""
    out = {}
    for b in sorted({r_[0] for r_ in REC}):
        for kind in ("capsule", "box", "hull"):
            items = [d for bb, kk, d in REC if bb == b and kk == kind]
            if kind == "hull":
                for i, d in enumerate(items):
                    out[f"{b}|hull|{i}"] = d
            elif items:
                out[f"{b}|{kind}"] = np.array(items)
    out["lee_crown"] = np.asarray(info["crown"])
    out["lee_hip"] = np.asarray(info["hip"])
    np.savez(OUT / "colliders.npz", **out)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_chair()
    info = build_room()
    save_colliders(info)
    print("Mrs. Lee: crown", np.round(info["crown"], 3), "hip", np.round(info["hip"], 3), "lowest z",
          round(info["lowest_z"] - g.FZ, 3), "width y", np.round(info["width_y"], 3), "feet x", round(info["feet_x"], 3))
    print("lever: L", round(g.LEVER_L, 4), "off", round(math.degrees(g.TH_OFF), 2), "on", round(math.degrees(g.TH_ON), 2),
          "centre", round(math.degrees(g.TH_C), 1), "K_off", round(g.K_OFF, 3), "K_on", round(g.K_ON, 3),
          "frame mass", round(g.FRAME_MASS, 2))
    for p in sorted(OUT.rglob("*.usda")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
