"""Geometry and parts for hand-pulling a dandelion out of a potted planting (real size).

An 8 in (203 mm) black plastic bulb pan (half-depth nursery pot, 102 mm tall) of light peat/perlite potting mix
stands on the bench; a young dandelion has come up in it, 40 mm off the pot's axis: a rosette of six toothed leaves round a crown at the
soil surface, a yellow flower on a leaning stalk, and a 63 mm taproot down in the mix. A folded blue poly
tarp lies on the bench to the left, where pulled weeds go. The job: pull the weed out with its whole root and
put it on the tarp, leaving the pot standing.

Physics (all native PhysX joints and contacts; nothing is scripted):
  * The pot (0.80 kg with its mix) is a free rigid body: wall staves, rim, base and a soil layer with a 16 mm
    hole where the taproot goes down.
  * The lower taproot is bonded to the pot by a breakable fixed joint: the soil's grip on the root lets go at
    ROOT_GRIP_N (13 N), more than the pot and mix weigh (7.8 N): pulled alone, the weed lifts the whole pot, so
    the other hand has to hold the pot down by its rim while the weed is pulled.
  * The crown and upper taproot are one body, bonded to the lower taproot at its narrow neck 28 mm below
    the soil by a breakable joint that takes a straight pull (SNAP_N, well above ROOT_GRIP_N) but snaps
    under SNAP_NM of bending: a sideways yank breaks the root and leaves the lower part in the pot.
  * Each leaf is its own body, bonded to the crown by a breakable joint (LEAF_N): pulling on leaves tears
    them off and leaves the root where it is.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench.
"""
from __future__ import annotations

import math
from dataclasses import replace as _replace

import numpy as np

import meshlib as M
from simkit import Anchor, Part, Piece

TOP = 0.7475

# -- pot ----------------------------------------------------------------------------------------------
POT_C = (0.220, -0.030)
POT_H = 0.102
R_TOP, R_BOT = 0.1016, 0.0857        # 8 in top, 6.75 in base, 4 in deep bulb pan
POT_MASS = 0.80
SOIL_REL = 0.084                     # soil surface above the bench (18 mm below the rim)
SOIL = TOP + SOIL_REL
POT_Z = TOP + 0.003                  # pot body origin: centre of its 6 mm base
HOLE = 0.008                         # half-width of the soil hole round the taproot

# -- weed ---------------------------------------------------------------------------------------------
WEED_REL = (0.000, 0.040)            # crown, 40 mm off the pot axis (toward +y)
WX, WY = POT_C[0] + WEED_REL[0], POT_C[1] + WEED_REL[1]
CROWN_R, CROWN_H = 0.011, 0.015
CROWN_Z = SOIL + 0.001 + CROWN_H / 2  # crown body origin (its centre)
UP_ROOT_L, UP_ROOT_R = 0.028, 0.0055
LO_ROOT_L, LO_ROOT_R = 0.035, 0.0045
NECK_REL = -CROWN_H / 2 - UP_ROOT_L   # the neck (snap point), in the crown frame
ROOT_Z = CROWN_Z + NECK_REL - LO_ROOT_L / 2
WEED_MASS, ROOT_MASS, LEAF_MASS = 0.012, 0.004, 0.0015
MAX_DEPEN = 0.3                      # cap on contact push-out speed for the root and leaves (m/s)
MAX_DEPEN_CROWN = 4.0                # the gripped crown: a looser cap, or the pads slide off it
ROOT_GRIP_N = 10.0                   # soil lets go of the root (pot + mix weigh only 7.8 N)
ROOT_GRIP_NM = 3.0
SNAP_N = 40.0                        # root neck: strong in tension ...
SNAP_NM = 0.60                       # ... weak in bending (17 N sideways at the grip, 35 mm up)
LEAF_N = 4.5                         # a leaf tears off the crown (less than the 13 N soil grip)
LEAF_NM = 0.10
LEAF_ANGLES = (50.0, 90.0, 130.0, 230.0, 270.0, 310.0)   # clear on +-x, where the fingers go
LEAF_TILT = math.radians(20.0)
LEAF_L = 0.085
LEAF_BASE_Z = SOIL + 0.005
LEAVES = tuple(f"leaf{i}" for i in range(len(LEAF_ANGLES)))

# -- tarp ---------------------------------------------------------------------------------------------
TARP_C = (0.280, 0.250)
TARP_S = (0.320, 0.240, 0.006)

# colours
POT_BLACK = (0.045, 0.045, 0.05)
SOIL_C = (0.17, 0.115, 0.075)
SOIL_DARK = (0.11, 0.075, 0.05)
PERLITE = (0.86, 0.86, 0.82)
BARK = (0.33, 0.20, 0.12)
LEAF_G = (0.20, 0.40, 0.10)
RIB_G = (0.55, 0.62, 0.38)
CROWN_C = (0.36, 0.30, 0.18)
ROOT_C = (0.80, 0.70, 0.52)
STALK_G = (0.36, 0.52, 0.22)
YELLOW = (0.98, 0.78, 0.05)
TARP_BLUE = (0.07, 0.24, 0.62)
TARP_HEM = (0.85, 0.85, 0.88)
GROMMET = (0.75, 0.73, 0.68)


def _vm(name, mesh, color, material=None, opacity=1.0, offset=(0, 0, 0), R=None):
    p, t = M.transform(mesh, R, offset)
    return (name, p, t, color, material, opacity)


def _colliders(pieces):
    return tuple(_replace(p, hidden=True) for p in pieces if p.collide)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def wall_r(z):
    return R_BOT + (R_TOP - R_BOT) * z / POT_H


# ----------------------------------------------------------------------------------------------- pot
def soil_mesh(r, z0, rnd):
    """The soil surface: a slightly lumpy disc of radius r, a little low round the hole."""
    nr, na = 10, 64
    pts = [(0.0, 0.0, z0)]
    for i in range(1, nr + 1):
        rr = r * i / nr
        for j in range(na):
            a = 2 * math.pi * j / na
            z = z0 + 0.0012 * rnd.standard_normal() * (0.4 + 0.6 * i / nr)
            pts.append((rr * math.cos(a), rr * math.sin(a), z))
    tris = []
    for j in range(na):
        tris.append((0, 1 + j, 1 + (j + 1) % na))
    for i in range(nr - 1):
        b0, b1 = 1 + i * na, 1 + (i + 1) * na
        for j in range(na):
            j1 = (j + 1) % na
            tris += [(b0 + j, b1 + j, b1 + j1), (b0 + j, b1 + j1, b0 + j1)]
    return np.asarray(pts), np.asarray(tris)


def soil_boxes(rs, band=0.016):
    """The soil layer as boxes in y-bands across a disc of radius rs, leaving a square hole round the root."""
    hx, hy = WEED_REL
    ys = sorted(set([-rs] + [hy - HOLE - band * k for k in range(1, 20) if hy - HOLE - band * k > -rs]
                    + [hy - HOLE, hy + HOLE] + [hy + HOLE + band * k for k in range(1, 20) if hy + HOLE + band * k < rs]
                    + [rs]))
    out = []
    for y0, y1 in zip(ys[:-1], ys[1:]):
        ym = max(abs(y0), abs(y1)) if y0 * y1 > 0 else 0.0
        if ym >= rs:
            ym = min(abs(y0), abs(y1))
        c = math.sqrt(max(rs * rs - ym * ym, 0.0))
        if c < 0.004:
            continue
        if abs(y0 - (hy - HOLE)) < 1e-9 and abs(y1 - (hy + HOLE)) < 1e-9:
            out += [(-c, hx - HOLE, y0, y1), (hx + HOLE, c, y0, y1)]
        else:
            out.append((-c, c, y0, y1))
    return out


def pot_object() -> Part:
    V = []
    oz = -0.003                                   # bench level in the pot frame
    prof = [(0.0, 0.0), (R_BOT - 0.003, 0.0), (R_BOT - 0.001, 0.0015), (R_BOT, 0.004)]
    for k in range(1, 9):
        z = 0.004 + (POT_H - 0.012) * k / 8
        prof.append((wall_r(z), z))
    prof += [(R_TOP + 0.0015, POT_H - 0.007), (R_TOP + 0.0030, POT_H - 0.005), (R_TOP + 0.0032, POT_H - 0.001),
             (R_TOP + 0.0015, POT_H), (R_TOP - 0.0020, POT_H), (R_TOP - 0.0030, POT_H - 0.004),
             (wall_r(0.090) - 0.002, 0.090)]
    V.append(_vm("pot", M.revolve(prof, n=72, axis="Z"), POT_BLACK, "abs", offset=(0, 0, oz)))
    # moulded ribs round the outside of the wall
    for k in range(16):
        a = 2 * math.pi * (k + 0.5) / 16
        rib = M.loft([(0.010, 0.003, 0.0025, 2.0), (POT_H - 0.022, 0.003, 0.0025, 2.0)], axis="Z", n=10)
        p, t = rib
        p = p.copy()
        p[:, 0] += wall_r(0.045) + 0.0008
        V.append(_vm(f"rib{k}", (p, t), POT_BLACK, "abs", offset=(0, 0, oz), R=rz(a)))
    rnd = np.random.default_rng(7)
    V.append(_vm("soil", soil_mesh(wall_r(SOIL_REL) - 0.0025, 0.0, rnd), SOIL_C, None, offset=(0, 0, oz + SOIL_REL)))
    # perlite grains and bark bits in the mix
    for k in range(70):
        r = math.sqrt(rnd.uniform(0, 1)) * (wall_r(SOIL_REL) - 0.008)
        a = rnd.uniform(0, 2 * math.pi)
        x, y = r * math.cos(a), r * math.sin(a)
        if math.hypot(x - WEED_REL[0], y - WEED_REL[1]) < 0.014:
            continue
        if k % 4 == 0:
            g = M.rbox((rnd.uniform(0.004, 0.008), rnd.uniform(0.002, 0.004), 0.0015), e=3.0, n=10)
            V.append(_vm(f"bark{k}", g, BARK, None, offset=(x, y, oz + SOIL_REL + 0.0006), R=rz(rnd.uniform(0, 3.1))))
        else:
            s = rnd.uniform(0.0012, 0.0024)
            g = M.revolve([(0.0, -s * 0.6), (s, 0.0), (0.0, s * 0.7)], n=8, axis="Z")
            V.append(_vm(f"perl{k}", g, PERLITE, None, offset=(x, y, oz + SOIL_REL + 0.0005)))
    # colliders: wall staves (tilted with the taper), rim, soil layer with the root hole
    P = []
    n = 28
    alpha = math.atan2(R_TOP - R_BOT, POT_H)
    for k in range(n):
        a = 2 * math.pi * k / n
        rm = (R_TOP + R_BOT) / 2 - 0.001
        w = 2 * math.pi * rm / n * 1.15
        P.append(Piece("box", (0.004, w, POT_H / math.cos(alpha)), (rm * math.cos(a), rm * math.sin(a), oz + POT_H / 2),
                       rot=(0.0, alpha, a)))
        # the rolled rim: a 7 mm round bead (a capsule along the rim's tangent)
        P.append(Piece("capsule", (0.0035, w * 1.08, 0.0035), ((R_TOP + 0.0005) * math.cos(a),
                                                                (R_TOP + 0.0005) * math.sin(a), oz + POT_H - 0.0035),
                       rot=(0.0, 0.0, a), axis="Y"))
    sz = oz + SOIL_REL - 0.005
    for (x0, x1, y0, y1) in soil_boxes(wall_r(SOIL_REL - 0.005) - 0.004):
        P.append(Piece("box", (x1 - x0, y1 - y0, 0.010), ((x0 + x1) / 2, (y0 + y1) / 2, sz)))
    return Part("pot", pos=(POT_C[0], POT_C[1], POT_Z), size=(R_BOT - 0.002, 0.006, R_BOT - 0.002), shape="cylinder",
                exact_z=True, mass=POT_MASS, com=(0.0, 0.0, 0.040), color=POT_BLACK, friction=(0.7, 0.6),
                restitution=0.1, contact_offset=0.003, pieces=_colliders(P), hide_root=True, vmeshes=tuple(V),
                solver_iters=(32, 4))


# ---------------------------------------------------------------------------------------------- weed
def leaf_mesh(rnd, L=LEAF_L):
    """A dandelion leaf along +x from its base: narrow stalk, widening blade cut into backward-pointing
    triangular lobes (runcinate), a pointed end lobe; folded a little along the pale midrib; rising at
    LEAF_TILT and drooping toward the tip."""
    ns = 60
    lobes = 5
    rows = []
    for k in range(ns + 1):
        s = k / ns
        env = 0.0035 + 0.013 * min(1.0, s / 0.75) ** 0.8
        if s > 0.82:
            env = env * max(0.0, (1.0 - s) / 0.18) ** 0.9
        ph = (s * lobes) % 1.0
        tooth = 0.35 + 0.65 * ph ** 1.5 if s > 0.18 else 1.0
        h = env * tooth
        x = s * L
        z = x * math.tan(LEAF_TILT) - 0.10 * x * x / L * 0.9 * (s ** 1.2)
        rows.append([(x, -h * (1 + 0.05 * rnd.standard_normal()), z - 0.0012 * (h / 0.016)),
                     (x, 0.0, z + 0.0006),
                     (x, h * (1 + 0.05 * rnd.standard_normal()), z - 0.0012 * (h / 0.016))])
    pts = np.asarray([p for r in rows for p in r])
    tris = []
    for k in range(ns):
        a, b = 3 * k, 3 * (k + 1)
        tris += [(a, b, b + 1), (a, b + 1, a + 1), (a + 1, b + 1, b + 2), (a + 1, b + 2, a + 2)]
    rib = np.asarray([r[1] for r in rows]) + np.array([0, 0, 0.0002])
    return (pts, np.asarray(tris)), M.tube(rib[::3], 0.0007, n=6)


def weed_object() -> Part:
    """The crown and upper taproot (one body), with the flower on its stalk."""
    V = []
    crown = M.revolve([(0.0, -CROWN_H / 2), (CROWN_R * 0.85, -CROWN_H / 2), (CROWN_R, -CROWN_H / 4),
                       (CROWN_R * 0.95, CROWN_H / 4), (CROWN_R * 0.55, CROWN_H / 2), (0.0, CROWN_H / 2)], n=24, axis="Z")
    V.append(_vm("crown", crown, CROWN_C, None))
    # leaf bases (sheaths) hugging the crown, purple-green as on a real dandelion
    for k, a in enumerate(LEAF_ANGLES):
        sh = M.loft([(0.0, 0.006, 0.003, 2.0), (0.012, 0.004, 0.002, 2.0)], axis="X", n=10)
        V.append(_vm(f"sheath{k}", sh, (0.42, 0.30, 0.30), None, offset=(0, 0, -0.002),
                     R=rz(math.radians(a)) @ ry(-LEAF_TILT)))
    top = -CROWN_H / 2
    root = M.revolve([(0.0, top - UP_ROOT_L), (UP_ROOT_R * 0.9, top - UP_ROOT_L), (UP_ROOT_R, top - UP_ROOT_L * 0.6),
                      (UP_ROOT_R * 1.05, top - 0.004), (CROWN_R * 0.7, top + 0.001)], n=16, axis="Z")
    V.append(_vm("taproot_upper", root, ROOT_C, None))
    # flower: a stalk leaning out toward +y (clear of the fingers, away from the holding hand), the head on top
    pts = []
    for k in range(13):
        s = k / 12
        pts.append((-0.004 * s, 0.060 * s ** 1.15, CROWN_H / 2 - 0.002 + 0.070 * s - 0.012 * s * s))
    V.append(_vm("stalk", M.tube(np.asarray(pts), 0.0018, n=8), STALK_G, None))
    head_c = np.asarray(pts[-1]) + np.array([0.0, 0.002, 0.004])
    tilt = rz(-math.pi / 2) @ ry(-0.5)
    V.append(_vm("bracts", M.revolve([(0.0, -0.007), (0.006, -0.006), (0.008, -0.001), (0.0085, 0.001)], n=20, axis="Z"),
                 STALK_G, None, offset=head_c, R=tilt))
    rnd = np.random.default_rng(3)
    for ring, (r0, r1, z, n) in enumerate(((0.004, 0.019, 0.0015, 34), (0.003, 0.014, 0.0040, 28), (0.002, 0.009, 0.0062, 20))):
        for j in range(n):
            a = 2 * math.pi * (j + 0.5 * ring) / n
            pet = M.loft([(r0, 0.0022, 0.0006, 2.0, 0.0, 0.0), (r1, 0.0018 + 0.0004 * rnd.random(), 0.0006, 2.0, 0.0,
                                                                   0.002 * (r1 - r0) / 0.015)], axis="X", n=6)
            V.append(_vm(f"pet{ring}_{j}", M.transform(pet, rz(a) @ ry(-(0.12 + 0.30 * ring)), (0, 0, z)), YELLOW,
                         None, offset=head_c, R=tilt))
    P = [Piece("cylinder", (UP_ROOT_R, UP_ROOT_L, UP_ROOT_R), (0.0, 0.0, -CROWN_H / 2 - UP_ROOT_L / 2))]
    return Part("weed", pos=(WX, WY, CROWN_Z), size=(CROWN_R, CROWN_H, CROWN_R), shape="cylinder", exact_z=True,
                mass=WEED_MASS, color=CROWN_C, friction=(0.9, 0.8), restitution=0.05, contact_offset=0.002,
                pieces=_colliders(P), hide_root=True, vmeshes=tuple(V), solver_iters=(32, 4), max_depenetration=MAX_DEPEN_CROWN,
                anchor=Anchor(break_force=SNAP_N, break_torque=SNAP_NM, parent="root",
                              parent_pos=(WX, WY, ROOT_Z), local=(0.0, 0.0, NECK_REL)))


def root_object() -> Part:
    """The lower taproot, held in the mix (bonded to the pot) until the soil's grip lets go."""
    L = LO_ROOT_L
    prof = [(0.0, -L / 2 - 0.002), (0.0012, -L / 2), (LO_ROOT_R * 0.6, -L / 4), (LO_ROOT_R, L / 4), (LO_ROOT_R * 1.15, L / 2),
            (0.0, L / 2)]
    V = [_vm("taproot_lower", M.revolve(prof, n=16, axis="Z"), ROOT_C, None)]
    rnd = np.random.default_rng(11)
    for k in range(9):                           # fine side roots
        z = rnd.uniform(-L / 2, L / 3)
        a = rnd.uniform(0, 2 * math.pi)
        l = rnd.uniform(0.006, 0.014)
        path = np.asarray([(0.0, 0.0, 0.0), (l * 0.5, 0.0, -l * 0.25), (l, 0.0, -l * 0.7)])
        V.append(_vm(f"hair{k}", M.tube(path, 0.0004, n=4), ROOT_C, None, offset=(0, 0, z), R=rz(a)))
    return Part("root", pos=(WX, WY, ROOT_Z), size=(LO_ROOT_R, L, LO_ROOT_R), shape="cylinder", exact_z=True,
                mass=ROOT_MASS, color=ROOT_C, friction=(0.6, 0.5), restitution=0.05, contact_offset=0.002,
                hide_root=True, vmeshes=tuple(V), solver_iters=(32, 4), max_depenetration=MAX_DEPEN,
                anchor=Anchor(break_force=ROOT_GRIP_N, break_torque=ROOT_GRIP_NM, parent="pot",
                              parent_pos=(POT_C[0], POT_C[1], POT_Z)))


def leaf_objects():
    out = []
    for i, a in enumerate(LEAF_ANGLES):
        rnd = np.random.default_rng(20 + i)
        L = LEAF_L * (0.85 + 0.25 * rnd.random())
        blade, rib = leaf_mesh(rnd, L)
        V = (("blade", blade[0], blade[1], LEAF_G, None, 1.0), ("midrib", rib[0], rib[1], RIB_G, None, 1.0))
        r0, cl = 0.013, 0.050          # collider from 24 mm off the crown axis, 50 mm long, 9 mm wide
        cx = r0 + cl / 2
        P = [Piece("box", (cl, 0.009, 0.0015), (cx * math.cos(LEAF_TILT), 0.0, cx * math.sin(LEAF_TILT) - 0.001),
                   rot=(0.0, -LEAF_TILT, 0.0))]
        ar = math.radians(a)
        pos = (WX + CROWN_R * math.cos(ar), WY + CROWN_R * math.sin(ar), LEAF_BASE_Z)
        out.append(Part(LEAVES[i], pos=pos, size=(0.003, 0.003, 0.0015), exact_z=True, yaw=ar, mass=LEAF_MASS,
                        color=LEAF_G, friction=(0.8, 0.7), restitution=0.05, contact_offset=0.002,
                        pieces=_colliders(P), hide_root=True, vmeshes=V, solver_iters=(32, 4), max_depenetration=MAX_DEPEN,
                        anchor=Anchor(break_force=LEAF_N, break_torque=LEAF_NM, parent="weed",
                                      parent_pos=(WX, WY, CROWN_Z))))
    return out


# ---------------------------------------------------------------------------------------------- tarp
def tarp_object() -> Part:
    sx, sy, sz = TARP_S
    V = [_vm("tarp", M.rbox((sx, sy, sz), e=10, axis="Z"), TARP_BLUE, None)]
    # folded layers showing along two edges, the white hem, fold creases and grommets
    for k in range(3):
        V.append(_vm(f"layer{k}", M.rbox((sx - 0.004, 0.003, 0.0012), e=4), (0.05, 0.18, 0.50), None,
                     offset=(0.0, -sy / 2 + 0.0005, -sz / 2 + 0.0015 + 0.0016 * k)))
    V.append(_vm("hem", M.rbox((sx - 0.006, 0.016, 0.0008), e=4), TARP_HEM, None, offset=(0.0, sy / 2 - 0.010, sz / 2)))
    for k, x in enumerate((-sx / 4, 0.0, sx / 4)):
        V.append(_vm(f"crease{k}", M.rbox((0.0025, sy - 0.03, 0.0006), e=4), (0.04, 0.15, 0.42), None, offset=(x, -0.006, sz / 2)))
    for k, x in enumerate((-sx / 2 + 0.018, sx / 2 - 0.018)):
        ring = M.revolve([(0.0045, 0.0), (0.0065, 0.0004), (0.0068, 0.0012), (0.0045, 0.0012)], n=20, axis="Z")
        V.append(_vm(f"grommet{k}", ring, GROMMET, "aluminium", offset=(x, sy / 2 - 0.010, sz / 2)))
    return Part("tarp", pos=(TARP_C[0], TARP_C[1], TOP + sz / 2), size=(sx, sy, sz), exact_z=True, color=TARP_BLUE,
                friction=(0.6, 0.5), hide_root=True, vmeshes=tuple(V))


def scene_objects():
    return [tarp_object(), pot_object(), root_object(), weed_object()] + leaf_objects()
