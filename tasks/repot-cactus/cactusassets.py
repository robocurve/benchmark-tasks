"""Shapes for the cactus repotting task: a golden barrel cactus in a 9 cm thin-wall nursery pot, with a foam collar
round its body, and a 4 in. pot with its firmed base layer. Colliders are boxes, cylinders and convex hulls; what the
cameras see are lathe and tube meshes in plain materials. Lengths in metres; each body's frame has z up and its origin
at the centre of its bottom (the cactus: at its base, on the soil face).
"""

from __future__ import annotations

import math

import numpy as np

from bodykit import BoxShape, ConvexShape, CylShape, MeshShape, plate

# -- sizes ------------------------------------------------------------------------------------------------------------
# Old pot: Teku VCG 9, 9.0 cm across the rim, 6.8 cm tall, thin wall
OLD_H = 0.068
OLD_RIM_R = 0.045
OLD_FLOOR = 0.001
OLD_MASS = 0.010         # in all: the base 6 g and four wall panels 1 g each
OLD_BASE_MASS = 0.006
PANEL_MASS = 0.001
OLD_TAPER = 0.109        # the wall's taper (radius per height), from 9.0 cm at the rim to 7.4 cm at the base
OLD_IN_RIM = 0.0444      # inner radius at the rim
OLD_WALL = 0.0006        # collider wall thickness (the real wall is 0.4 mm)
HINGE_Z = 0.003          # each wall panel flexes about a line round its base, this high
FLEX_Z = 0.045           # where the wall's inward deflection is measured, as on the real pot: 4.5 cm up
FLEX_AT_X = 0.003        # m: the wall moves in this far at that height ...
X_N = 20.0               # ... under a 20 N press there (X): a panel spring of X * L^2 / d
PANEL_K = X_N * (FLEX_Z - HINGE_Z) ** 2 / FLEX_AT_X     # N m/rad (11.8)
# Calibrated with known forces on the panels (20 N at 4.5 cm up, as the real X procedure): PhysX's D6 angular drive on
# these 1 g panels yields less stiffness than it is given, so each drive is scaled to give the measured 3 mm at 20 N.
PANEL_SPRING_GAIN = (14.3, 14.3, 10.8, 10.8)   # per panel (+x, +y, -x, -y): the D6 frame turned for -x, -y differs
PANEL_STOP = 0.004       # m: the wall stops after moving in 4 mm at 4.5 cm up, as the real wall bottoms on the soil ball
PANEL_LIMIT_DEG = math.degrees(PANEL_STOP / (FLEX_Z - HINGE_Z))   # 5.5 deg: 5.1 mm at the ball's top edge (5.6 clear)
PANEL_I = None           # the panels keep PhysX's own inertias

# Root ball: a frustum 6.8 cm across the base, 8.0 cm across the top, 5.5 cm tall; soil face 1.15 cm below the rim
BALL_H = 0.055
BALL_R0 = 0.0355         # the sim ball is a little narrower than the real 6.8/8.0 cm one at the top: the rigid ball
BALL_R1 = 0.0375         # must leave the wall room to flex in (the real ball's soil gives 3 mm)
BALL_MASS = 0.18
BALL_Z0 = OLD_FLOOR + 0.0005
TAPER = (BALL_R1 - BALL_R0) / BALL_H

# Cactus: golden barrel, 6.0 cm across the spine tips, 6.0 cm tall above the soil
CACTUS_H = 0.060
CACTUS_R = 0.030
CACTUS_MASS = 0.12
N_RIBS = 18

# Collar: a 30 mm band of 4 mm foam, 6.8 cm across outside, its lower edge 1.2 cm above the soil
COLLAR_Z0 = 0.012
COLLAR_Z1 = 0.042
COLLAR_T = 0.004
COLLAR_R_OUT = 0.034
COLLAR_R_IN = COLLAR_R_OUT - COLLAR_T
COLLAR_MASS = 0.004

# New pot: HC 4.00 in. Standard Thinwall, 10.2 cm rim outside, 8.9 cm tall
NEW_H = 0.089
NEW_RIM_R = 0.051
NEW_RIM_IN = 0.0495
NEW_BASE_R = 0.037
NEW_FLOOR = 0.002
LAYER_TOP = 0.019        # base layer of cactus mix firmed to 2.0 cm: a ball set on it has its top 1.5 cm below the rim
NEW_MASS = 0.015 + 0.040  # pot and its base layer

WALL_T = 0.002           # collider wall thickness (the real wall is 0.4-0.6 mm; nothing meets its inside but the ball)

BLACK = (0.035, 0.035, 0.038)
SOIL = (0.17, 0.12, 0.08)
MIX = (0.30, 0.24, 0.17)
ROOTS = (0.50, 0.40, 0.28)
CACTUS_C = (0.20, 0.42, 0.16)
SPINE_C = (0.86, 0.72, 0.30)
WOOL_C = (0.90, 0.82, 0.55)
FOAM_C = (0.12, 0.38, 0.80)


def old_inner_r(z: float) -> float:
    """The old pot's inner radius at height z above its bottom (at rest)."""
    return OLD_IN_RIM - OLD_TAPER * (OLD_H - z)


def old_outer_r(z: float) -> float:
    return old_inner_r(z) + OLD_WALL


def new_inner_r(z: float) -> float:
    return NEW_RIM_IN - (NEW_RIM_IN - (NEW_BASE_R - WALL_T)) * (NEW_H - z) / NEW_H


def cactus_r(z: float) -> float:
    """The cactus's radius to its spine tips at height z above the soil: a squat barrel."""
    u = min(max(z / CACTUS_H, 0.0), 1.0)
    return CACTUS_R * max(0.0, math.sqrt(max(0.0, 1.0 - ((u - 0.42) / 0.62) ** 2)) )


# -- meshes -----------------------------------------------------------------------------------------------------------

def lathe(profile, segments: int = 32, z_offset: float = 0.0, ribs: int = 0, rib_depth: float = 0.0,
          arc=(0.0, 2 * math.pi)):
    """A surface of revolution about z from an (r, z) profile, over ``arc`` (rad); ``ribs`` > 0 fluting the radius by
    ``rib_depth``."""
    pts, nrm, faces = [], [], []
    angles = [arc[0] + (arc[1] - arc[0]) * k / segments for k in range(segments + 1)]
    for (r0, z0), (r1, z1) in zip(profile[:-1], profile[1:]):
        dr, dz = r1 - r0, z1 - z0
        length = math.hypot(dr, dz)
        nr, nz = dz / length, -dr / length
        base = len(pts)
        for a in angles:
            c, s = math.cos(a), math.sin(a)
            f = 1.0 - rib_depth * (0.5 - 0.5 * math.cos(ribs * a)) if ribs else 1.0
            pts += [(r0 * f * c, r0 * f * s, z0 + z_offset), (r1 * f * c, r1 * f * s, z1 + z_offset)]
            nrm += [(nr * c, nr * s, nz)] * 2
        for k in range(segments):
            a0, b0, a1, b1 = base + 2 * k, base + 2 * k + 1, base + 2 * k + 2, base + 2 * k + 3
            faces += [(a0, a1, b1), (a0, b1, b0)]
    return tuple(pts), tuple(faces), tuple(nrm)


def tubes(segments, radius: float, sides: int = 4):
    """Thin tubes (spines) from (a, b) pairs, merged into one mesh."""
    pts, nrm, faces = [], [], []
    for p1, p2 in segments:
        a, b = np.asarray(p1, float), np.asarray(p2, float)
        d = (b - a) / np.linalg.norm(b - a)
        u = np.cross(d, [0.0, 0.0, 1.0] if abs(d[2]) < 0.9 else [1.0, 0.0, 0.0])
        u /= np.linalg.norm(u)
        v = np.cross(d, u)
        base = len(pts)
        for k in range(sides):
            ang = 2 * math.pi * k / sides
            r = math.cos(ang) * u + math.sin(ang) * v
            pts += [tuple(a + radius * r), tuple(b + 0.25 * radius * r)]
            nrm += [tuple(r), tuple(r)]
        for k in range(sides):
            i0, i1 = base + 2 * k, base + 2 * ((k + 1) % sides)
            faces += [(i0, i1, i1 + 1), (i0, i1 + 1, i0 + 1)]
    return tuple(pts), tuple(faces), tuple(nrm)


def mesh(data, color, roughness: float = 0.5) -> MeshShape:
    pts, faces, nrm = data
    return MeshShape((0.0, 0.0, 0.0), pts, faces, color, normals=nrm, roughness=roughness)


def _hull(profile, ring: int = 16, centre_z: float | None = None, friction=None) -> ConvexShape:
    """A convex hull through rings at each (r, z) of ``profile``."""
    pts = []
    for r, z in profile:
        for k in range(ring):
            a = 2 * math.pi * k / ring
            pts.append((r * math.cos(a), r * math.sin(a), z))
    n = len(profile)
    faces = []
    for i in range(n - 1):
        for k in range(ring):
            j = (k + 1) % ring
            a, b, c, d = i * ring + k, i * ring + j, (i + 1) * ring + j, (i + 1) * ring + k
            faces += [(a, b, c), (a, c, d)]
    for k in range(1, ring - 1):
        faces += [(0, k + 1, k), ((n - 1) * ring, (n - 1) * ring + k, (n - 1) * ring + k + 1)]
    zs = [z for _, z in profile]
    cz = (min(zs) + max(zs)) / 2 if centre_z is None else centre_z
    return ConvexShape((0.0, 0.0, cz), tuple(pts), tuple(faces), None, friction=friction)


def _wall(n: int, r_at, z0: float, z1: float) -> list:
    shapes = []
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        c, s = math.cos(a), math.sin(a)
        r0, r1 = r_at(z0) + WALL_T / 2, r_at(z1) + WALL_T / 2
        width = 2 * (r_at(z1) + WALL_T) * math.tan(math.pi / n) * 1.08
        shapes.append(plate((r0 * c, r0 * s, z0), (r1 * c, r1 * s, z1), width, WALL_T, (c, s, 0.0), None))
    return shapes


# -- bodies' shapes ---------------------------------------------------------------------------------------------------

PANEL_ANGLES = (0.0, 90.0, 180.0, 270.0)   # each wall panel's middle: +x, +y, -x, -y; each spans 90 degrees


def old_base_shapes() -> list:
    """The pot's floor and the stiff ring at its base, about which the wall panels flex."""
    r0 = old_outer_r(0.0)
    shapes = [CylShape((0.0, 0.0, OLD_FLOOR / 2), r0, OLD_FLOOR, None)]
    n = 16
    for k in range(n):  # a 3 mm lip round the floor: the base of the wall, below the panels' hinge line
        a = 2 * math.pi * (k + 0.5) / n
        rm = old_inner_r(0.0) + OLD_WALL / 2
        width = 2 * r0 * math.tan(math.pi / n) * 1.08
        shapes.append(BoxShape((rm * math.cos(a), rm * math.sin(a), HINGE_Z / 2), (OLD_WALL, width, HINGE_Z), None, yaw=a))
    profile = [(0.0, 0.0), (r0, 0.0), (old_outer_r(HINGE_Z), HINGE_Z), (old_inner_r(HINGE_Z), HINGE_Z),
               (old_inner_r(OLD_FLOOR), OLD_FLOOR), (0.0, OLD_FLOOR)]
    shapes.append(mesh(lathe(profile, 40), BLACK, 0.55))
    return shapes


def old_panel_shapes(i: int) -> list:
    """Wall panel ``i``: a quarter of the pot's wall with its rolled rim, from the hinge line up, in the pot's frame."""
    mid = math.radians(PANEL_ANGLES[i])
    shapes = []
    n = 4
    for k in range(n):
        a = mid + math.radians(-45.0 + 90.0 * (k + 0.5) / n)
        c, s = math.cos(a), math.sin(a)
        r0, r1 = old_inner_r(HINGE_Z) + OLD_WALL / 2, old_inner_r(OLD_H) + OLD_WALL / 2
        width = 2 * old_outer_r(OLD_H) * math.tan(math.pi / (4 * n)) * 1.04
        shapes.append(plate((r0 * c, r0 * s, HINGE_Z), (r1 * c, r1 * s, OLD_H), width, OLD_WALL, (c, s, 0.0), None,))
        r = OLD_RIM_R - 0.0015
        shapes.append(BoxShape((r * c, r * s, OLD_H - 0.002), (0.003, 2 * r * math.tan(math.pi / (4 * n)) * 1.04, 0.004),
                               None, yaw=a))
    profile = [(old_outer_r(HINGE_Z), HINGE_Z), (old_outer_r(OLD_H - 0.004), OLD_H - 0.004),
               (OLD_RIM_R, OLD_H - 0.004), (OLD_RIM_R, OLD_H - 0.0008), (old_inner_r(OLD_H), OLD_H),
               (old_inner_r(HINGE_Z), HINGE_Z)]
    shapes.append(mesh(lathe(profile, 12, arc=(mid - math.pi / 4, mid + math.pi / 4)), BLACK, 0.55))
    return shapes


def ball_shapes() -> list:
    """The root ball, its origin at the centre of its base."""
    hull = _hull([(BALL_R0, 0.0), (BALL_R1, BALL_H)], 16, friction=(0.6, 0.6))
    side = [(0.0, 0.0), (BALL_R0 - 0.003, 0.0), (BALL_R0, 0.003), (BALL_R1 - 0.0005, BALL_H - 0.002),
            (BALL_R1 - 0.002, BALL_H)]
    top = [(BALL_R1 - 0.002, BALL_H), (0.0, BALL_H)]
    return [hull, mesh(lathe(side, 32), ROOTS, 0.9), mesh(lathe(top, 32), SOIL, 1.0)]


def cactus_profile(n: int = 12) -> list[tuple[float, float]]:
    out = []
    for i in range(n + 1):
        z = CACTUS_H * i / n
        out.append((max(cactus_r(z), 0.004), z))
    return out


def cactus_shapes() -> list:
    """The cactus and its collar: one rigid body, its origin on the soil face at the centre of its base."""
    prof = cactus_profile(10)
    # the collider stays inside the spine tips; it is one hull, so the collar's ring below is what the fingers meet
    shapes = [_hull(prof, 16, friction=(0.6, 0.6))]
    # the collar: a ring of boxes, its outer face 6.8 cm across
    n = 20
    zc, h = (COLLAR_Z0 + COLLAR_Z1) / 2, COLLAR_Z1 - COLLAR_Z0
    rm = (COLLAR_R_OUT + COLLAR_R_IN) / 2
    width = 2 * COLLAR_R_OUT * math.tan(math.pi / n) * 1.05
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        shapes.append(BoxShape((rm * math.cos(a), rm * math.sin(a), zc), (COLLAR_T, width, h), None, yaw=a,
                               friction=(1.0, 0.9)))
    # what the cameras see: a ribbed green body, golden spines, a woolly crown, the blue foam band
    body = [(r * 0.86, z) for r, z in prof]
    body[-1] = (0.0, CACTUS_H - 0.002)
    shapes.append(mesh(lathe(body, 72, ribs=N_RIBS, rib_depth=0.16), CACTUS_C, 0.6))
    spines = []
    for k in range(N_RIBS):
        a = 2 * math.pi * k / N_RIBS
        for z in np.arange(0.004, CACTUS_H - 0.006, 0.0065):
            if COLLAR_Z0 - 0.002 < z < COLLAR_Z1 + 0.002:
                continue
            r = cactus_r(z) * 0.86
            p = np.array([r * math.cos(a), r * math.sin(a), z])
            out = np.array([math.cos(a), math.sin(a), 0.25])
            for da in (-0.35, 0.0, 0.35):
                ca, sa = math.cos(da), math.sin(da)
                d = np.array([out[0] * ca - out[1] * sa, out[0] * sa + out[1] * ca, out[2] + 0.3 * abs(da)])
                d /= np.linalg.norm(d)
                spines.append((p, p + d * (cactus_r(z) - r + 0.002)))
    shapes.append(mesh(tubes(spines, 0.0006), SPINE_C, 0.5))
    shapes.append(mesh(lathe([(0.0, CACTUS_H + 0.001), (0.009, CACTUS_H - 0.001), (0.012, CACTUS_H - 0.004)], 24),
                       WOOL_C, 0.9))
    ring = [(COLLAR_R_IN, COLLAR_Z0), (COLLAR_R_OUT, COLLAR_Z0), (COLLAR_R_OUT, COLLAR_Z1), (COLLAR_R_IN, COLLAR_Z1),
            (COLLAR_R_IN, COLLAR_Z0)]
    shapes.append(mesh(lathe(ring, 48), FOAM_C, 0.8))
    return shapes


def new_pot_shapes() -> list:
    shapes = [CylShape((0.0, 0.0, NEW_FLOOR / 2), NEW_BASE_R - WALL_T, NEW_FLOOR, None)]
    shapes += _wall(24, new_inner_r, 0.0, NEW_H)
    layer_r = new_inner_r(LAYER_TOP) - 0.001
    shapes.append(CylShape((0.0, 0.0, (NEW_FLOOR + LAYER_TOP) / 2), layer_r, LAYER_TOP - NEW_FLOOR, None,
                           friction=(0.8, 0.8)))
    profile = [(0.0, 0.0), (NEW_BASE_R, 0.0), (NEW_RIM_R - 0.0015, NEW_H - 0.007), (NEW_RIM_R, NEW_H - 0.005),
               (NEW_RIM_R, NEW_H - 0.001), (NEW_RIM_R - 0.0012, NEW_H), (NEW_RIM_IN, NEW_H),
               (new_inner_r(LAYER_TOP), LAYER_TOP)]
    shapes.append(mesh(lathe(profile, 48), BLACK, 0.55))
    top = [(layer_r + 0.001, LAYER_TOP), (0.0, LAYER_TOP)]
    shapes.append(mesh(lathe(top, 48), MIX, 1.0))
    return shapes
