"""The fixed scene as simulator bodies: the forearm trainer (lying across the table, palm down, the hand to the robot's
left), its underpad, the applicator tray with the opened pouch, the medicine cup on its tray with the dollop of cream,
the waste pot and the plaster's stand with its opened wrapper, all kinematic (fixed where they are put). And three loose
rigid bodies that the hands must really hold: the cotton-tipped applicator, and the two release liners' paper tabs
(each hinged at its root where it lies on the opened wrapper until a hand takes it).
"""

from __future__ import annotations

import math

import numpy as np

import scene as S
import bodykit
from bodykit import Body, BoxShape, ConvexShape, CylShape, MeshShape, SphereShape

TABLE = S.TABLE
SKIN = (0.93, 0.76, 0.64)

# (distance along the arm from the wound toward the hand, half-width, half-height, centre height): elbow to knuckles
_SECTIONS = [(-0.162, 0.034, 0.030, 0.042), (-0.155, 0.039, 0.035, 0.045), (-0.130, 0.041, 0.036, 0.045),
             (-0.090, S.ARM_A, S.ARM_B, S.ARM_ZC), (0.090, S.ARM_A, S.ARM_B, S.ARM_ZC), (0.130, 0.031, 0.021, 0.033),
             (0.160, 0.037, 0.016, 0.027), (0.210, 0.042, 0.014, 0.025), (0.240, 0.041, 0.012, 0.023), (0.245, 0.036, 0.008, 0.022)]
_SEG = 48


def _ring(v, a, b, zc, n=_SEG, off=0.0):
    """A section ring at distance v along the arm (world y = v), centred off the arm's axis by `off` (x)."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([S.XW + off + a * np.sin(t), np.full(n, v), zc + b * np.cos(t)], 1)


def _loft(rings, cap=True):
    pts = np.concatenate(rings)
    n = len(rings[0])
    faces = []
    for r in range(len(rings) - 1):
        for k in range(n):
            a, b = r * n + k, r * n + (k + 1) % n
            c, d = a + n, b + n
            faces += [(a, b, c), (b, d, c)]
    extra = []
    if cap:
        for r, flip in ((0, False), (len(rings) - 1, True)):
            ci = len(pts) + len(extra)
            extra.append(rings[r].mean(0))
            for k in range(n):
                a, b = r * n + k, r * n + (k + 1) % n
                faces.append((ci, a, b) if flip else (ci, b, a))
        pts = np.concatenate([pts, np.array(extra)])
    return pts, faces


def _merge(parts):
    pts, faces, off = [], [], 0
    for p, f in parts:
        pts.append(p)
        faces += [(a + off, b + off, c + off) for a, b, c in f]
        off += len(p)
    return np.concatenate(pts), faces


def _normals(pts, faces):
    n = np.zeros_like(pts)
    f = np.asarray(faces)
    fn = np.cross(pts[f[:, 1]] - pts[f[:, 0]], pts[f[:, 2]] - pts[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def arm_mesh():
    xs = []
    for (v0, a0, b0, z0), (v1, a1, b1, z1) in zip(_SECTIONS[:-1], _SECTIONS[1:]):
        for s in np.linspace(0, 1, 6, endpoint=False):
            f = 0.5 - 0.5 * math.cos(math.pi * s)
            xs.append((v0 + (v1 - v0) * s, a0 + (a1 - a0) * f, b0 + (b1 - b0) * f, z0 + (z1 - z0) * f))
    xs.append(_SECTIONS[-1])
    parts = [_loft([_ring(*s) for s in xs])]
    for off, ln, r in ((0.026, 0.060, 0.0075), (0.009, 0.072, 0.0080), (-0.009, 0.068, 0.0080), (-0.025, 0.055, 0.0072)):
        segs = []
        for s in np.linspace(0, 1, 6):
            v = 0.238 + ln * s * math.cos(0.35 * s)
            z = 0.021 - ln * 0.35 * s * s
            segs.append(_ring(v, r * (1 - 0.25 * s), r * 0.85 * (1 - 0.25 * s), z, 16, off))
        parts.append(_loft(segs))
    segs = [_ring(0.165 + 0.05 * s, 0.009 * (1 - 0.2 * s), 0.008, 0.022 - 0.006 * s, 16, -0.040 - 0.02 * s) for s in np.linspace(0, 1, 6)]
    parts.append(_loft(segs))
    return _merge(parts)


def arm_colliders():
    a, b = S.ARM_A - S.SKIN_GIVE, S.ARM_B - S.SKIN_GIVE
    return [np.concatenate([_ring(-0.090, a, b, S.ARM_ZC, 32), _ring(0.090, a, b, S.ARM_ZC, 32)]),
            np.concatenate([_ring(-0.162, 0.033, 0.029, 0.042, 16), _ring(-0.090, a, b, S.ARM_ZC, 16)]),
            np.concatenate([_ring(0.090, a, b, S.ARM_ZC, 16), _ring(0.140, 0.030, 0.019, 0.032, 16), _ring(0.250, 0.040, 0.010, 0.022, 16)])]


def _hull_faces(points):
    from scipy.spatial import ConvexHull

    return tuple(tuple(int(v) for v in s) for s in ConvexHull(points).simplices)


def _on_skin_mesh(curve_pts, half_w, lift, color, rough=0.6):
    """A thin ribbon on the skin along (y, s) points, half_w wide, `lift` above the skin (world frame, z above table)."""
    pts, faces = [], []
    for k, (y, s) in enumerate(curve_pts):
        for d in (-half_w, half_w):
            p, n = S.ARM.point(y, s + d)
            pts.append(p + n * lift)
    for k in range(len(curve_pts) - 1):
        a = 2 * k
        faces += [(a, a + 2, a + 1), (a + 1, a + 2, a + 3)]
    return np.array(pts), faces


def wound_shapes(origin):
    """The abrasion (a raw pink-red oval with a darker centre) and the margin's dotted outline in skin marker."""
    out = []
    (hy, hs) = S.WOUND_HALF
    for scale, color, lift in ((1.0, (0.78, 0.30, 0.28), 0.00010), (0.62, (0.62, 0.12, 0.12), 0.00016)):
        n = 28
        pts, faces = [], []
        c, _ = S.ARM.point(0.0, 0.0)
        ring = []
        for t in np.linspace(0, 2 * np.pi, n, endpoint=False):
            p, nn = S.ARM.point(hy * scale * math.cos(t), hs * scale * math.sin(t))
            ring.append(p + nn * lift)
        pc, nc = S.ARM.point(0.0, 0.0)
        pts = [pc + nc * lift] + ring
        faces = [(0, 1 + k, 1 + (k + 1) % n) for k in range(n)]
        p = np.array(pts) - origin
        out.append(MeshShape(centre=(0.0, 0.0, 0.0), points=tuple(map(tuple, p)), faces=tuple(faces), color=color, roughness=0.7))
    # the margin: dots 1.5 mm across every 3 mm round an outline MARGIN out from the wound's edge
    ry, rs = hy + S.MARGIN, hs + S.MARGIN
    per = 2 * math.pi * math.sqrt((ry ** 2 + rs ** 2) / 2)
    for t in np.linspace(0, 2 * np.pi, int(per / 0.003), endpoint=False):
        p, nn = S.ARM.point(ry * math.cos(t), rs * math.sin(t))
        q = p + nn * 0.0001 - origin
        out.append(BoxShape(centre=tuple(q.tolist()), size=(0.0015, 0.0015, 0.0002), color=(0.08, 0.12, 0.48), collide=False))
    return out


def lathe(profile, n=32):
    """A surface of revolution about z from (r, z) points."""
    pts, faces = [], []
    m = len(profile)
    for k in range(n):
        t = 2 * math.pi * k / n
        for r, z in profile:
            pts.append((r * math.cos(t), r * math.sin(t), z))
    for k in range(n):
        k2 = (k + 1) % n
        for j in range(m - 1):
            a, b, c, d = k * m + j, k * m + j + 1, k2 * m + j, k2 * m + j + 1
            faces += [(a, c, b), (b, c, d)]
    return pts, faces


def bodies() -> list:
    out = []
    # the trainer arm (kinematic; contact reports for the fingertip forces)
    origin = np.array([S.XW, 0.0, 0.0])
    pts, faces = arm_mesh()
    shapes = [MeshShape(centre=(0.0, 0.0, 0.0), points=tuple(map(tuple, pts - origin)), faces=tuple(faces), color=SKIN,
                        normals=tuple(map(tuple, _normals(pts, faces))), roughness=0.55)]
    for hull in arm_colliders():
        p = hull - origin
        shapes.append(ConvexShape(centre=tuple(p.mean(0).tolist()), points=tuple(map(tuple, p)), faces=_hull_faces(p),
                                  color=None, friction=(0.8, 0.7)))
    shapes += wound_shapes(origin)
    out.append(Body.build("underpad", (S.XW, S.PAD_YC, TABLE), 2.0, [
        BoxShape(centre=(0.0, 0.0, 0.003), size=(S.PAD_SIZE[0], S.PAD_SIZE[1], 0.006), color=(0.72, 0.58, 0.40)),
        BoxShape(centre=(0.0, 0.0, 0.0065), size=(S.PAD_SIZE[0] - 0.01, S.PAD_SIZE[1], 0.001), color=(0.70, 0.82, 0.93))],
        kinematic=True))
    out.append(Body.build("forearm", (S.XW, 0.0, TABLE), 1.2, shapes, kinematic=True, contact_report=True,
                          friction=(0.8, 0.7), contact_offset=0.002))
    # applicator tray: a 25 mm white HDPE board with the opened peel pouch (paper, a printed band)
    tx, ty = S.TRAY_C
    lx, ly, lz = S.TRAY_SIZE
    out.append(Body.build("applicator_tray", (tx, ty, TABLE), 0.3, [
        BoxShape(centre=(0.0, 0.0, lz / 2), size=(lx, ly, lz), color=(0.95, 0.95, 0.93)),
        BoxShape(centre=(0.0, 0.0, lz + 0.0002), size=(lx - 0.01, 0.030, 0.0004), color=(0.99, 0.99, 0.98), collide=False),
        BoxShape(centre=(-0.04, 0.0, lz + 0.0005), size=(0.012, 0.030, 0.0002), color=(0.25, 0.45, 0.75), collide=False)],
        kinematic=True))
    # the medicine cup on its tray, with the dollop of cream on its floor
    cx, cy = S.CUP_C
    cp, cf = lathe([(S.CUP_R_BOT - 0.0012, 0.0012), (S.CUP_R_BOT, 0.0), (S.CUP_R_TOP, S.CUP_H), (S.CUP_R_TOP - 0.0010, S.CUP_H),
                    (S.CUP_R_BOT - 0.0010, 0.0012)])
    cp = [(x, y, z + S.CUP_Z) for x, y, z in cp]
    dp, df = lathe([(0.0001, S.DOLLOP_H + 0.0012), (S.DOLLOP_R * 0.5, S.DOLLOP_H * 0.75 + 0.0012), (S.DOLLOP_R, 0.0012)], 24)
    dp = [(x, y, z + S.CUP_Z) for x, y, z in dp]
    walls = [BoxShape(centre=((S.CUP_R_BOT + 0.0015) * math.cos(t), (S.CUP_R_BOT + 0.0015) * math.sin(t), S.CUP_Z + S.CUP_H / 2),
                      size=(0.002, 0.007, S.CUP_H), yaw=t, color=None) for t in np.linspace(0, 2 * np.pi, 16, endpoint=False)]
    out.append(Body.build("cup", (cx, cy, TABLE), 0.3, [
        BoxShape(centre=(0.0, 0.0, S.CUP_Z / 2), size=(*S.CUP_TRAY, S.CUP_Z), color=(0.95, 0.95, 0.93)),
        MeshShape(centre=(0.0, 0.0, S.CUP_Z + S.CUP_H / 2), points=tuple(cp), faces=tuple(cf), color=(0.92, 0.94, 0.97), roughness=0.3),
        MeshShape(centre=(0.0, 0.0, S.CUP_Z + 0.003), points=tuple(dp), faces=tuple(df), color=(0.42, 0.20, 0.07), roughness=0.4),
        BoxShape(centre=(0.0, 0.0, S.CUP_Z + 0.0006), size=(0.026, 0.026, 0.0012), color=None), *walls], kinematic=True))
    # waste pot: a 1 L open bin
    px, py = S.POT_C
    wall = []
    for t in np.linspace(0, 2 * np.pi, 20, endpoint=False):
        wall.append(BoxShape(centre=(S.POT_R * math.cos(t), S.POT_R * math.sin(t), S.POT_H / 2), size=(0.003, 0.0185, S.POT_H),
                             yaw=t, color=(0.16, 0.17, 0.19)))
    out.append(Body.build("waste_pot", (px, py, TABLE), 0.4, [
        CylShape(centre=(0.0, 0.0, 0.002), radius=S.POT_R, height=0.004, color=(0.09, 0.09, 0.10)), *wall], kinematic=True))
    # the plaster's stand: a riser with the opened wrapper on it
    sx, sy = S.STAND_C
    a, b, h = S.STAND_SIZE
    out.append(Body.build("plaster_stand", (sx, sy, TABLE), 0.3, [
        BoxShape(centre=(0.0, 0.0, h / 2), size=(a, b, h), color=(0.95, 0.95, 0.93)),
        BoxShape(centre=(0.0, 0.0, h + 0.0002), size=(a - 0.004, b - 0.002, 0.0004), color=(0.96, 0.90, 0.86), collide=False)],
        kinematic=True))
    out += loose_parts()
    return out


APP_MASS = 0.002  # kg (a real 6 in wooden applicator is about 1 g; rounded up for the solver)
TAB_MASS = 0.001  # kg (a paper tab is a fraction of a gram; rounded up for the solver)


def loose_parts() -> list:
    """The applicator and the two liner tabs as loose PhysX bodies, plus a hinge at each tab's root (at the strip's end,
    across the strip, sprung to lie flat) that holds it where it lies until it is taken: a closing jaw turns the paper
    flap into line with its pads, as it would a real tab."""
    import firstaid as F

    stick, bud = (0.88, 0.76, 0.55), (0.98, 0.98, 0.97)
    L, r, rb, lb = S.APP_LEN, S.APP_STICK_R, S.APP_TIP_R, S.APP_TIP_LEN
    shaft = L - lb + 0.002
    shapes = [BoxShape(centre=(shaft / 2, 0.0, 0.0), size=(shaft, 2 * r, 2 * r), color=None),  # the shaft's collider
              CylShape(centre=(shaft / 2, 0.0, 0.0), radius=r, height=shaft, axis="X", color=stick, collide=False),
              CylShape(centre=(L - lb / 2, 0.0, 0.0), radius=rb, height=lb - 2 * rb, axis="X", color=bud, collide=False)]
    shapes += [SphereShape(centre=(x, 0.0, 0.0), radius=rb, color=bud) for x in (L - rb, L - lb / 2, L - lb + rb)]
    tx, ty = S.TRAY_C
    out = [Body.build("applicator", (tx + S.TRAY_SIZE[0] / 2 + S.APP_OVER, ty, TABLE + F.APP_REST_Z), APP_MASS, shapes,
                      yaw=math.pi, friction=(0.6, 0.5), contact_report=True, contact_offset=0.001, inertia=4e-6,
                      solver_iterations=32)]
    for k in F.SIDES:
        p, _ = F.tab_rest_pose(k)
        out.append(Body.build(f"tab_{k}", (p[0], p[1], TABLE + p[2]), TAB_MASS,
                              [BoxShape(centre=(0.0, F.SIGN[k] * S.TAB_OUT / 2, 0.0), size=(S.PL_WID, S.TAB_OUT, S.TAB_THICK),
                                        color=None)],
                              friction=(0.6, 0.5), contact_report=True, contact_offset=0.001, inertia=2e-7, solver_iterations=32))
        name = f"root_{k}"
        if name not in [h.name for h in bodykit._HINGES]:
            bodykit.hinge(name, "plaster_stand", f"tab_{k}", (p[0], p[1], TABLE + p[2]), axis="X", limits_deg=(-50.0, 50.0),
                          spring=2e-4, damping=2e-5)
    return out
