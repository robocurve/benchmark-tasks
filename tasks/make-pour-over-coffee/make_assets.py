"""Writes the task's USD files into assets/ (any Python with pxr; the Arena venv has it):

    python make_assets.py

kettle.usda: a Hario V60 Drip Kettle Air (350 mL, clear resin, drawn opaque): body, lid, gooseneck spout and loop
handle. Origin: the centre of its base; +x towards the spout.
dripper.usda: a Hario V60 01 plastic dripper (white) with a paper filter and the coffee bed in it. Origin: the
centre of the underside of its base plate.
mug.usda: a clear glass mug drawn as its edges, with a dark line inside at 220 mL. Origin: the centre of its base.
Each is one rigid body; the task file gives the masses.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import brew

HERE = Path(__file__).resolve().parent
N_SEG = 40

# kettle (m, in its frame)
K_R, K_H = 0.039, 0.115
SPOUT = [(0.036, 0.0, 0.020), (0.058, 0.0, 0.032), (0.072, 0.0, 0.055), (0.083, 0.0, 0.080), (0.094, 0.0, 0.098),
         (0.100, 0.0, 0.105)]
SPOUT_R = 0.0045
HANDLE_X, HANDLE_Z0, HANDLE_Z1 = -0.060, 0.025, 0.105      # the handle's vertical bar
HANDLE_W = 0.012                                           # its thickness
# dripper
PLATE_R, PLATE_T = 0.048, 0.004
CONE_Z0 = 0.010                                            # the outlet above the plate's underside
WALL = 0.0025
# mug
MUG_R, MUG_H, MUG_BASE = 0.040, 0.085, 0.006
LINE_ML = 220.0

KETTLE_C = (0.93, 0.94, 0.95)
LID_C = (0.15, 0.15, 0.16)
DRIPPER_C = (0.97, 0.97, 0.96)
PAPER_C = (0.95, 0.93, 0.86)
COFFEE_C = (0.30, 0.19, 0.10)
GLASS_C = (0.62, 0.72, 0.78)
LINE_C = (0.10, 0.10, 0.12)
def revolve(profile, n=N_SEG, z_shift=0.0):
    pts, faces = [], []
    m = len(profile)
    for k in range(n):
        a = 2 * math.pi * k / n
        for r, z in profile:
            pts.append((r * math.cos(a), r * math.sin(a), z + z_shift))
    for k in range(n):
        k2 = (k + 1) % n
        for i in range(m - 1):
            faces.append([k * m + i, k2 * m + i, k2 * m + i + 1, k * m + i + 1])
    return pts, faces


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


def mesh(stage, path, pts, faces, color, collide=False, opacity=None, offset=(0.0, 0.0, 0.0)):
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    off = np.asarray(offset, float)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*(np.asarray(p) - off)) for p in pts]))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if opacity is not None:
        m.CreateDisplayOpacityAttr([opacity])
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        m.CreatePurposeAttr("guide")
    return m


def link(stage, path, mass, at, com=None, inertia=None):
    from pxr import Gf, UsdGeom, UsdPhysics

    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, at)))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    m = UsdPhysics.MassAPI.Apply(x.GetPrim())
    m.CreateMassAttr(float(mass))
    m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com if com is not None else (0, 0, 0))))
    if inertia is not None:
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    return x


def joint(stage, path, kind, b0, b1, at0, axis, lo=None, hi=None):
    from pxr import Gf, Sdf, UsdPhysics

    j = (UsdPhysics.PrismaticJoint if kind == "p" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, at0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    if lo is not None:
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if kind == "p" else "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(0.0)
    return j




def rigid_root(stage, name):
    from pxr import UsdGeom, UsdPhysics

    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim())
    return f"/{name}"


def tube(p0, p1, r, n=16):
    """A closed cylinder from p0 to p1 (points, faces)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = p1 - p0
    L = float(np.linalg.norm(a))
    a /= L
    t = np.cross(a, [0, 0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [1.0, 0, 0])
    t /= np.linalg.norm(t)
    b = np.cross(a, t)
    pts, faces = [], []
    for k in range(n):
        ang = 2 * math.pi * k / n
        d = r * (math.cos(ang) * t + math.sin(ang) * b)
        pts += [tuple(p0 + d), tuple(p1 + d)]
    pts += [tuple(p0), tuple(p1)]
    for k in range(n):
        k2 = (k + 1) % n
        faces.append([2 * k, 2 * k2, 2 * k2 + 1, 2 * k + 1])
        faces.append([2 * n, 2 * k2, 2 * k])
        faces.append([2 * n + 1, 2 * k + 1, 2 * k2 + 1])
    return pts, faces


def box(c, size):
    c, h = np.asarray(c, float), np.asarray(size, float) / 2
    pts = [tuple(c + h * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return pts, faces


def kettle():
    stage, out = new_stage("kettle")
    r = rigid_root(stage, "Kettle")
    p, f = revolve([(0.0, 0.0), (K_R - 0.004, 0.0), (K_R, 0.004), (K_R, K_H), (0.0, K_H)])
    mesh(stage, f"{r}/body", p, f, KETTLE_C, collide=False)
    mesh(stage, f"{r}/body_hull", p, f, KETTLE_C, collide=True)
    p, f = revolve([(0.0, K_H), (K_R - 0.002, K_H), (K_R - 0.006, K_H + 0.006), (0.010, K_H + 0.008),
                    (0.010, K_H + 0.016), (0.0, K_H + 0.016)])
    mesh(stage, f"{r}/lid", p, f, LID_C)
    for i, (a, b) in enumerate(zip(SPOUT[:-1], SPOUT[1:])):
        p, f = tube(a, b, SPOUT_R, n=12)
        mesh(stage, f"{r}/spout{i}", p, f, KETTLE_C)
        if i >= 1:
            mesh(stage, f"{r}/spout_hull{i}", p, f, KETTLE_C, collide=True)
    # the loop handle: a vertical bar joined to the body at its top and bottom
    zc = (HANDLE_Z0 + HANDLE_Z1) / 2
    parts = [box((HANDLE_X, 0.0, zc), (HANDLE_W, 0.016, HANDLE_Z1 - HANDLE_Z0)),
             box(((HANDLE_X - K_R) / 2 + 0.002, 0.0, HANDLE_Z1 - 0.006), (abs(HANDLE_X) - K_R + 0.012, 0.014, 0.012)),
             box(((HANDLE_X - K_R) / 2 + 0.002, 0.0, HANDLE_Z0 + 0.006), (abs(HANDLE_X) - K_R + 0.012, 0.014, 0.012))]
    for i, (p, f) in enumerate(parts):
        mesh(stage, f"{r}/handle{i}", p, f, LID_C)
        mesh(stage, f"{r}/handle_hull{i}", p, f, LID_C, collide=True)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def dripper():
    stage, out = new_stage("dripper")
    r = rigid_root(stage, "Dripper")
    z0, z1 = CONE_Z0, CONE_Z0 + brew.CONE_D
    ri = lambda h: brew.R_HOLE + (brew.R_RIM - brew.R_HOLE) * h / brew.CONE_D
    outer = [(0.0, 0.0), (PLATE_R, 0.0), (PLATE_R, PLATE_T), (brew.R_HOLE + WALL + 0.004, PLATE_T),
             (brew.R_HOLE + WALL, z0), (brew.R_RIM + WALL, z1), (brew.R_RIM + WALL + 0.003, z1 + 0.002), (0.0, z1 + 0.002)]
    hull = [(0.0, 0.0), (PLATE_R, 0.0), (PLATE_R, PLATE_T), (brew.R_HOLE + WALL + 0.004, PLATE_T),
            (brew.R_RIM + WALL + 0.003, z1 + 0.002), (0.0, z1 + 0.002)]
    p, f = revolve(hull)
    mesh(stage, f"{r}/hull", p, f, DRIPPER_C, collide=True)
    # drawn: the plate, the cone's outer wall and its rim (open at the top, so the water and grounds show)
    p, f = revolve([(0.0, 0.0), (PLATE_R, 0.0), (PLATE_R, PLATE_T), (0.0, PLATE_T)])
    mesh(stage, f"{r}/plate", p, f, DRIPPER_C)
    prof = [(brew.R_HOLE + WALL, z0)] + [(ri(h) + WALL, z0 + h) for h in np.linspace(0.0, brew.CONE_D, 8)[1:]] + \
           [(brew.R_RIM + WALL + 0.003, z1 + 0.002), (brew.R_RIM + 0.0005, z1 + 0.002)] + \
           [(ri(h) + 0.0005, z0 + h) for h in np.linspace(brew.CONE_D, 0.0, 8)]
    p, f = revolve(prof)
    mesh(stage, f"{r}/cone", p, f, DRIPPER_C)
    p, f = revolve([(ri(h), z0 + h) for h in np.linspace(brew.CONE_D, brew.H_BED, 6)])
    mesh(stage, f"{r}/paper", p, f, PAPER_C)
    p, f = revolve([(0.0, z0), (brew.R_HOLE, z0)] + [(ri(h), z0 + h) for h in np.linspace(0.0, brew.H_BED, 4)[1:]] +
                   [(0.0, z0 + brew.H_BED)])
    mesh(stage, f"{r}/grounds", p, f, COFFEE_C)
    p, f = box((0.0, brew.R_RIM + 0.020, z1 - 0.012), (0.012, 0.030, 0.020))      # the handle tab
    mesh(stage, f"{r}/tab", p, f, DRIPPER_C)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def mug():
    stage, out = new_stage("mug")
    r = rigid_root(stage, "Mug")
    p, f = revolve([(0.0, 0.0), (MUG_R, 0.0), (MUG_R, MUG_H), (0.0, MUG_H)])
    mesh(stage, f"{r}/hull", p, f, GLASS_C, collide=True)
    ri = brew.MUG_R
    for i, (z0, z1) in enumerate([(0.0, MUG_BASE), (MUG_H - 0.003, MUG_H)]):
        p, f = revolve([(ri, z0), (MUG_R, z0), (MUG_R, z1), (ri, z1)])
        mesh(stage, f"{r}/edge{i}", p, f, GLASS_C)
    z = MUG_BASE + LINE_ML * 1e-6 / (math.pi * ri * ri)
    p, f = revolve([(ri - 0.0004, z), (MUG_R + 0.0004, z), (MUG_R + 0.0004, z + 0.0015), (ri - 0.0004, z + 0.0015)])
    mesh(stage, f"{r}/line", p, f, LINE_C)
    for k in range(4):                                          # four thin uprights so the glass reads as a mug
        a = 2 * math.pi * k / 4 + math.pi / 4
        p, f = box((MUG_R * math.cos(a), MUG_R * math.sin(a), MUG_H / 2), (0.002, 0.002, MUG_H))
        mesh(stage, f"{r}/upright{k}", p, f, GLASS_C)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


if __name__ == "__main__":
    kettle()
    dripper()
    mug()
