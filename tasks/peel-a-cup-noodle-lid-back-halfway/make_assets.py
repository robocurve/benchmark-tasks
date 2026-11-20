"""Writes the task's USD file into assets/ (any Python with pxr; the Arena venv has it):

    python make_assets.py

cup.usda: a cup of instant noodles (Nissin Cup Noodles size: 9.4 cm across the rim, 6.6 cm across the base, 10.6 cm
tall, 75 g with the dry noodles) with its paper-foil lid, as one articulation with a free root. The lid is a chain of
19 strips, each 5 mm wide, lying across the cup from the tab side (-y) to the far side (+y):

    cup -> j18 (hinge on the far edge of the rim) -> s18 -> j17 -> s17 -> ... -> j0 -> s0 (with the tab)

Every hinge turns about the cup's x axis. The task file locks the hinges of the strips still sealed to the rim and frees
each one as the seal there lets go. Only the tab has a collider (the strips are drawn, not collided), so the gripper
pinches the tab and pulls the rest through it. Origin: the centre of the cup's base; +z up its axis.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
N_SEG = 48
R_TOP, R_BASE, H = 0.047, 0.033, 0.106       # outer rim radius, base radius, height (m)
RIM_W = 0.003                                # the sealed ring on the rim (m)
N_STRIP, W_STRIP = 19, 0.005                 # strips across the 95 mm lid (m)
LID_R = R_TOP + 0.0005
TAB_L, TAB_W, TAB_T = 0.020, 0.020, 0.0012   # the tab past the rim: length (-y), width (x), thickness (m)
TAB_UP = math.radians(30.0)                  # the tab sticks up from the rim edge, as a cup's tab does
CUP_MASS, STRIP_MASS, TAB_MASS = 0.055, 0.001, 0.001   # the cup + noodles + lid: 75 g
LID_T = 0.0008                               # drawn thickness of the lid (m)

CUP_RED = (0.78, 0.10, 0.10)
CUP_WHITE = (0.94, 0.93, 0.90)
LID_SILVER = (0.80, 0.80, 0.78)
LID_PRINT = (0.80, 0.12, 0.10)
LID_WHITE = (0.95, 0.95, 0.93)
NOODLE = (0.86, 0.72, 0.45)


def strip_y(k: int) -> tuple[float, float]:
    """The strip's extent along y in the cup frame (tab side first)."""
    y0 = -LID_R + k * (2 * LID_R / N_STRIP)
    return y0, y0 + 2 * LID_R / N_STRIP


def strip_width() -> float:
    return 2 * LID_R / N_STRIP


def chord(y: float) -> float:
    return math.sqrt(max(0.0, LID_R ** 2 - y * y))


def new_stage(name):
    from pxr import Usd, UsdGeom, UsdPhysics

    out = HERE / "assets" / f"{name}.usda"
    out.parent.mkdir(exist_ok=True)
    if out.exists():
        out.unlink()
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    return stage, out


def mesh(stage, path, pts, faces, color, collide=False):
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        m.CreatePurposeAttr("guide")
    return m


def revolve(profile, n=N_SEG):
    pts, faces = [], []
    m = len(profile)
    for k in range(n):
        a = 2 * math.pi * k / n
        for r, z in profile:
            pts.append((r * math.cos(a), r * math.sin(a), z))
    for k in range(n):
        k2 = (k + 1) % n
        for i in range(m - 1):
            faces.append([k * m + i, k2 * m + i, k2 * m + i + 1, k * m + i + 1])
    return pts, faces


def prism(poly, z0, z1):
    """A vertical prism over a convex polygon (list of (x, y)), from z0 to z1."""
    n = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    return pts, faces


def slice_poly(y0, y1, n=6):
    """The lid's outline between y0 and y1 (a slice of the disc), in the cup frame."""
    ys = np.linspace(y0, y1, n)
    right = [(chord(y), y) for y in ys]
    left = [(-chord(y), y) for y in ys[::-1]]
    return right + left


def link(stage, path, mass, at, inertia):
    from pxr import Gf, UsdGeom, UsdPhysics

    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, at)))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    m = UsdPhysics.MassAPI.Apply(x.GetPrim())
    m.CreateMassAttr(float(mass))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    return x


def hinge(stage, path, b0, b1, at0):
    from pxr import Gf, Sdf, UsdPhysics

    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, at0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    j.CreateLowerLimitAttr(-60.0)
    j.CreateUpperLimitAttr(200.0)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(0.0)
    return j


def cup():
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    stage, out = new_stage("cup")
    root = UsdGeom.Xform.Define(stage, "/Cup")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    prim = root.GetPrim()
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)

    body = link(stage, "/Cup/cup", CUP_MASS, (0, 0, 0), inertia=(6e-5, 6e-5, 4e-5))
    UsdPhysics.MassAPI(body.GetPrim()).CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.05))
    # outside: base, tapered wall, rolled rim; a red band round the middle and a white top and bottom
    p, f = revolve([(0.0, 0.0), (R_BASE, 0.0), (R_TOP - 0.0015, H - 0.004), (R_TOP, H - 0.003), (R_TOP, H),
                    (0.0, H)])
    hull = mesh(stage, "/Cup/cup/hull", p, f, CUP_WHITE, collide=True)
    # drawn: the cup open at the top (rim, inner wall) with the dry noodle block 12 mm below the rim
    p, f = revolve([(0.0, 0.0), (R_BASE, 0.0), (R_TOP - 0.0015, H - 0.004), (R_TOP, H - 0.003), (R_TOP, H),
                    (R_TOP - 0.003, H), (R_TOP - 0.004, H - 0.012), (0.0, H - 0.012)])
    mesh(stage, "/Cup/cup/shell", p, f, CUP_WHITE)
    p, f = revolve([(0.0, H - 0.0118), (R_TOP - 0.0042, H - 0.0118)])
    mesh(stage, "/Cup/cup/noodles", p, f, NOODLE)
    for i, (r, a0) in enumerate([(0.012, 0.3), (0.024, 2.0), (0.030, 4.1), (0.018, 5.2), (0.008, 3.3)]):
        bx, by = r * math.cos(a0), r * math.sin(a0)
        p, f = prism([(bx - 0.003, by - 0.003), (bx + 0.003, by - 0.003), (bx + 0.003, by + 0.003),
                      (bx - 0.003, by + 0.003)], H - 0.0118, H - 0.0108)
        mesh(stage, f"/Cup/cup/bit{i}", p, f, [(0.85, 0.55, 0.15), (0.25, 0.55, 0.25)][i % 2])
    def r_at(z):
        return R_BASE + (R_TOP - 0.0015 - R_BASE) * z / (H - 0.004)
    for i, (z0, z1, col) in enumerate([(0.0, 0.030, CUP_WHITE), (0.030, 0.082, CUP_RED), (0.082, H - 0.004, CUP_WHITE)]):
        p, f = revolve([(r_at(z0) + 0.0004, z0), (r_at(z1) + 0.0004, z1)])
        mesh(stage, f"/Cup/cup/band{i}", p, f, col)

    # the lid: strips from the tab side (k = 0) to the far side (k = N-1); each strip's frame sits on its far edge
    zl = H + 0.0002
    w = strip_width()
    for k in range(N_STRIP):
        y0, y1 = strip_y(k)
        path = f"/Cup/s{k}"
        mass = STRIP_MASS + (TAB_MASS if k == 0 else 0.0)
        link(stage, path, mass, (0, y1, zl), inertia=(1e-6, 1e-6, 1e-6))
        poly = [(x, y - y1) for x, y in slice_poly(y0, y1)]
        p, f = prism(poly, 0.0, LID_T * 0.5)
        mesh(stage, f"{path}/foil", p, f, LID_SILVER)          # the foil underside
        # the printed top: red with a white band across the middle of the lid
        col = LID_WHITE if abs((y0 + y1) / 2) < 0.012 else LID_PRINT
        p, f = prism(poly, LID_T * 0.5, LID_T)
        mesh(stage, f"{path}/print", p, f, col)
        if k == 0:
            # the tab: a paper tongue past the rim's outer edge on the -y side, its own link on a limp hinge there
            # (the paper bends at the rim), standing up TAB_UP; the tab's frame sits on that hinge
            e = y0 - y1 - 0.001
            link(stage, "/Cup/tab", TAB_MASS, (0, y1 + e, zl), inertia=(1e-6, 1e-6, 1e-6))
            c, s = math.cos(TAB_UP), math.sin(TAB_UP)
            pts = []
            for x in (-TAB_W / 2, TAB_W / 2):
                for (yy, zz) in ((0.0, 0.0), (-TAB_L * c, TAB_L * s)):
                    for dz in (0.0, TAB_T):
                        pts.append((x, yy + dz * s, zz + dz * c))
            faces = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]]
            mesh(stage, "/Cup/tab/paper", pts, faces, LID_PRINT)
            mesh(stage, "/Cup/tab/hull", pts, faces, LID_PRINT, collide=True)
            j = hinge(stage, "/Cup/joints/jt", path, "/Cup/tab", (0.0, e, 0.0))
            j.GetLowerLimitAttr().Set(-20.0)
            j.GetUpperLimitAttr().Set(90.0)
            # the strip's own part of the tab, flat over the rim edge (drawn)
            poly = [(-TAB_W / 2, e), (TAB_W / 2, e), (TAB_W / 2, e + 0.003), (-TAB_W / 2, e + 0.003)]
            p, f = prism(poly, 0.0, LID_T * 2)
            mesh(stage, f"{path}/tab_root", p, f, LID_PRINT)
        parent = "/Cup/cup" if k == N_STRIP - 1 else f"/Cup/s{k + 1}"
        at0 = (0.0, y1, zl) if k == N_STRIP - 1 else (0.0, -w, 0.0)
        hinge(stage, f"/Cup/joints/j{k}", parent, path, at0)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


if __name__ == "__main__":
    cup()
