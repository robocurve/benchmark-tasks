"""Writes the task's USD files into assets/ (any Python with pxr; the Arena venv has it):

    python make_assets.py

pot.usda: a 3 qt stainless pot without its lid (inside 21 cm across, 8.5 cm deep, two side handles). Origin: the
centre of the inside bottom. Its wall is 24 boxes round the circle, so the ladle can go in.
ladle.usda: an 8 oz stainless ladle: a spherical-cap bowl (10 cm across, 4.6 cm deep), a flat steel neck rising at 65 deg
from the rim plane and a soft 24 x 16 mm handle from 9 cm to its end, 20 cm along (about 31 cm overall). Origin: the centre of the bowl's rim,
+z out of the bowl, +x along the handle's direction.
bowl.usda: a white ceramic soup bowl (16.6 cm across the rim, 7.2 cm tall, about 700 mL to the brim). Origin: the
centre of the inside bottom (its foot is 8 mm under it).
Each is one rigid body; the task file gives the masses.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import soup

HERE = Path(__file__).resolve().parent
N_SEG = 40
STEEL = (0.74, 0.76, 0.78)
STEEL_D = (0.55, 0.57, 0.60)
WHITE = (0.95, 0.95, 0.93)
# pot
POT_WALL, POT_BASE_T = 0.004, 0.006
POT_N = 24
# ladle
HANDLE_DEG = 65.0
HANDLE_L, HANDLE_W, HANDLE_T = 0.20, 0.016, 0.003    # the steel neck
GRIP_S0, GRIP_W, GRIP_T = 0.09, 0.024, 0.016          # the soft handle from GRIP_S0 to the end: width, thickness
BLACK = (0.10, 0.10, 0.11)
GRIP_AT = 0.16                      # m along the handle from the rim: where it is held (4 cm from its end)
# bowl
BOWL_FOOT = 0.008
BOWL_T = 0.005


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


def rigid_root(stage, name):
    from pxr import UsdGeom, UsdPhysics

    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim())
    return f"/{name}"


def obox(c, axes, half):
    """An oriented box: centre c, unit axes (3 columns), half sizes."""
    c, Ax, h = np.asarray(c, float), np.asarray(axes, float), np.asarray(half, float)
    pts = [tuple(c + Ax @ (h * np.array([sx, sy, sz]))) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return pts, faces


def box(c, size):
    return obox(c, np.eye(3), np.asarray(size, float) / 2)


def pot():
    stage, out = new_stage("pot")
    r = rigid_root(stage, "Pot")
    R, H = soup.R_POT, soup.H_POT
    p, f = revolve([(0.0, -POT_BASE_T), (R + POT_WALL, -POT_BASE_T), (R + POT_WALL, H), (R, H), (R, 0.0), (0.0, 0.0)])
    mesh(stage, f"{r}/shell", p, f, STEEL)
    p, f = revolve([(R - 0.0005, 0.0), (R - 0.0005, H - 0.003), (R + 0.0005, H - 0.003), (R + 0.0005, 0.0)], n=N_SEG)
    mesh(stage, f"{r}/inside", p, f, STEEL_D)
    mesh(stage, f"{r}/floor_c", *box((0, 0, -POT_BASE_T / 2), (2 * R + 2 * POT_WALL, 2 * R + 2 * POT_WALL, POT_BASE_T)),
         STEEL, collide=True)
    for k in range(POT_N):
        a = 2 * math.pi * (k + 0.5) / POT_N
        u = np.array([math.cos(a), math.sin(a), 0.0])
        t = np.array([-u[1], u[0], 0.0])
        half = (POT_WALL / 2 + 0.002, R * math.tan(math.pi / POT_N) + 0.002, H / 2)
        mesh(stage, f"{r}/wall_c{k}", *obox(u * (R + POT_WALL / 2 + 0.002), np.stack([u, t, [0, 0, 1.0]], 1) , half)
             if False else obox(u * (R + POT_WALL / 2 + 0.002) + [0, 0, H / 2], np.stack([u, t, [0, 0, 1.0]], 1), half),
             STEEL, collide=True)
    for s in (-1, 1):                                    # side handles (along y)
        mesh(stage, f"{r}/handle{s + 1}", *box((0.0, s * (R + POT_WALL + 0.020), H - 0.020), (0.060, 0.036, 0.010)), STEEL_D)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def handle_frame():
    b = math.radians(HANDLE_DEG)
    h = np.array([math.cos(b), 0.0, math.sin(b)])      # along the handle
    f = np.array([-math.sin(b), 0.0, math.cos(b)])     # its broad face's normal (upper side)
    return h, f


def handle_point(s):
    h, _ = handle_frame()
    return np.array([soup.A_RIM, 0.0, 0.0]) + s * h


def ladle():
    stage, out = new_stage("ladle")
    r = rigid_root(stage, "Ladle")
    Rs, a, d = soup.R_SPH, soup.A_RIM, soup.H_CUP
    zc = Rs - d                                         # sphere centre above the cap's bottom; rim at z = 0
    t0 = math.asin(a / Rs)
    prof_out = [((Rs + 0.0015) * math.sin(t), zc - (Rs + 0.0015) * math.cos(t) - (zc - 0.0)) for t in np.linspace(0, t0, 10)]
    # profile in the ladle frame: z measured from the rim plane
    def cap(rr, n=10):
        return [(rr * math.sin(t), (Rs - d) - rr * math.cos(t)) for t in np.linspace(0, math.asin(min(1.0, a / rr)), n)]
    outer = cap(Rs + 0.0012)
    inner = cap(Rs)
    p, f = revolve(outer + [(a + 0.0012, 0.0), (a, 0.0)] + inner[::-1])
    mesh(stage, f"{r}/bowl", p, f, STEEL)
    p, f = revolve([(0.0, -d - 0.0012)] + outer[1:] + [(a + 0.0012, 0.0), (0.0, 0.0)])
    mesh(stage, f"{r}/bowl_hull", p, f, STEEL, collide=True)
    h, fn = handle_frame()
    y = np.array([0.0, 1.0, 0.0])
    Ax = np.stack([h, y, fn], 1)
    pts, fc = obox(handle_point(GRIP_S0 / 2), Ax, (GRIP_S0 / 2, HANDLE_W / 2, HANDLE_T / 2))
    mesh(stage, f"{r}/neck_strip", pts, fc, STEEL)
    mesh(stage, f"{r}/neck_strip_c", pts, fc, STEEL, collide=True)
    n_seg = 3
    for i in range(n_seg):
        s0 = GRIP_S0 + (HANDLE_L - GRIP_S0) * i / n_seg
        s1 = GRIP_S0 + (HANDLE_L - GRIP_S0) * (i + 1) / n_seg
        pts, fc = obox(handle_point((s0 + s1) / 2), Ax, ((s1 - s0) / 2, GRIP_W / 2, GRIP_T / 2))
        mesh(stage, f"{r}/handle{i}", pts, fc, BLACK)
        mesh(stage, f"{r}/handle_c{i}", pts, fc, BLACK, collide=True)
    # the neck joining the bowl's rim to the handle, and the hook at its end
    pts, fc = obox(np.array([a - 0.004, 0.0, 0.0]), np.eye(3), (0.008, HANDLE_W / 2 + 0.002, 0.002))
    mesh(stage, f"{r}/neck", pts, fc, STEEL)
    end = handle_point(HANDLE_L)
    pts, fc = obox(end - h * 0.012, Ax, (0.004, 0.003, GRIP_T / 2 + 0.0005))    # the hanging hole's grommet
    mesh(stage, f"{r}/hole", pts, fc, STEEL_D)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def bowl():
    stage, out = new_stage("bowl")
    r = rigid_root(stage, "Bowl")
    r0, r1, H = soup.BOWL_R0, soup.BOWL_R1, soup.BOWL_H
    prof = [(0.0, -BOWL_FOOT), (r0 - 0.002, -BOWL_FOOT), (r0 + BOWL_T, 0.0), (r1 + BOWL_T, H), (r1 + BOWL_T, H + 0.002),
            (r1, H + 0.002), (r1, H), (r0, 0.0), (0.0, 0.0)]
    p, f = revolve(prof)
    mesh(stage, f"{r}/body", p, f, WHITE)
    p, f = revolve([(0.0, -BOWL_FOOT), (r0 - 0.002, -BOWL_FOOT), (r0 + BOWL_T, 0.0), (r1 + BOWL_T, H + 0.002),
                    (0.0, H + 0.002)])
    mesh(stage, f"{r}/hull", p, f, WHITE, collide=True)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


if __name__ == "__main__":
    pot()
    ladle()
    bowl()
