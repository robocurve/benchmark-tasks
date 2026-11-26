"""Geometry and measurements for taking a tea bag out of a mug of tea.

On the bench: a mug of tea with a tea bag steeping in it and a teaspoon standing in it, the bag's
string taken once round the mug's handle so that its paper tag hangs outside, and a side plate
beyond it.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench top.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Hinge, Part, Piece, Tether

TOP = 0.7475

CERAMIC = (0.93, 0.92, 0.88)
MUG_COL = (0.16, 0.34, 0.52)
TEA = (0.42, 0.22, 0.06)
BAG_COL = (0.80, 0.70, 0.50)
TAG_COL = (0.92, 0.80, 0.20)
STEEL = (0.76, 0.77, 0.79)
MAT_COL = (0.97, 0.97, 0.96)

# -- mug (free) ---------------------------------------------------------------------------------
MUG_XY = (0.220, 0.000)
MUG_RI, MUG_RO = 0.043, 0.047          # inside and outside radius (86 mm bore, 4 mm wall)
MUG_BASE = 0.006
MUG_H = 0.078                          # outside height; 72 mm deep inside
MUG_MASS = 0.620                       # 400 g mug with 220 ml of tea
STAVES = 16
RIM = TOP + MUG_H
TEA_LEVEL = TOP + MUG_BASE + 0.038     # tea 38 mm deep, 34 mm below the rim
HANDLE_OUT = 0.036                     # the handle stands this far out from the wall, on the +y side

# -- tea bag, string and tag (free) -------------------------------------------------------------
BAG = (0.052, 0.009, 0.046)            # wet bag: width x thickness x height
BAG_MASS = 0.014
STRING = 0.105                         # bag's top to tag's top
TAG = (0.002, 0.026, 0.030)            # paper tag: thickness x width x height
TAG_MASS = 0.002
TAG_XY = (MUG_XY[0], MUG_XY[1] + MUG_RO + HANDLE_OUT + 0.016)
TAG_Z = TOP + 0.058                    # its centre, hanging beside the handle
TAG_HOLD = 1.0                         # N: the string's turn round the handle lets the tag go

# -- teaspoon (free), standing in the mug -------------------------------------------------------
SPOON_L = 0.130
BOWL = (0.030, 0.005, 0.042)           # width x thickness x length
HANDLE = (0.010, 0.004, 0.092)
SPOON_MASS = 0.028
LEAN = math.radians(34.7)              # it leans on the rim toward -y
SPOON_TIP = (MUG_XY[0], MUG_XY[1] + 0.004, TOP + MUG_BASE + 0.004)
SPOON_AXIS0 = (0.0, -math.sin(LEAN), math.cos(LEAN))       # along the handle, upwards, as spawned (body frame)
SPOON_FACE0 = (0.0, math.cos(LEAN), math.sin(LEAN))        # the bowl's face normal, as spawned
GRASP_S = 0.101                        # where the demo takes it, measured up the spoon from its tip: its fingertips
                                       # just above the rim, the handle's top 5 cm between the fingers
BOWL_S = 0.021                         # the bowl's centre

# -- side plate (fixed) and paper mat (drawn) ---------------------------------------------------
PLATE_XY = (0.395, 0.000)
PLATE_R = 0.095
PLATE_H = 0.006

# -- squeezing and dripping ---------------------------------------------------------------------
SQUEEZE_N = 1.5                        # the spoon presses the bag on the mug's wall at least this hard
WALL_N = 1.0                           # ... and the wall presses back at least this hard
SQUEEZE_S = 1.0                        # for this long in all: the bag is squeezed out
DRIP_EVERY = 0.5                       # s between drops from a bag that is out of the tea and not squeezed out
DRIPS = 40                             # drops in a bag
DROPS_DRAWN = 12
TOUCH_N = 0.2                          # a finger pad on the bag


def _c(x, y, z, ref):
    return (float(x - ref[0]), float(y - ref[1]), float(z - ref[2]))


def mug_object() -> Part:
    cx, cy = MUG_XY
    c = np.array([cx, cy, TOP + MUG_BASE / 2])
    rm = (MUG_RI + MUG_RO) / 2
    w = 2 * MUG_RO * math.tan(math.pi / STAVES) + 0.0005
    h = MUG_H - MUG_BASE
    pieces = []
    for k in range(STAVES):
        a = 2 * math.pi * k / STAVES
        pieces.append(Piece("box", (w, MUG_RO - MUG_RI, h), (rm * math.sin(a), rm * math.cos(a), MUG_BASE / 2 + h / 2),
                            rot=(0.0, 0.0, -a), color=MUG_COL))
    y0 = MUG_RO - 0.002
    pieces += [
        Piece("box", (0.012, HANDLE_OUT, 0.010), (0.0, y0 + HANDLE_OUT / 2, MUG_H - 0.016 - MUG_BASE / 2), color=MUG_COL),
        Piece("box", (0.012, HANDLE_OUT, 0.010), (0.0, y0 + HANDLE_OUT / 2, 0.022 - MUG_BASE / 2), color=MUG_COL),
        Piece("box", (0.012, 0.010, MUG_H - 0.028), (0.0, y0 + HANDLE_OUT - 0.005, (MUG_H + 0.006) / 2 - MUG_BASE / 2), color=MUG_COL),
        Piece("cylinder", (MUG_RI - 0.0005, 0.001, MUG_RI - 0.0005), (0.0, 0.0, TEA_LEVEL - c[2]), color=TEA, collide=False),
        Piece("cylinder", (MUG_RI - 0.0005, 0.001, MUG_RI - 0.0005), (0.0, 0.0, MUG_BASE / 2 + 0.001), color=TEA, collide=False),
    ]
    return Part("mug", pos=tuple(float(v) for v in c), size=(MUG_RO, MUG_BASE, MUG_RO), shape="cylinder", axis="Z",
                exact_z=True, mass=MUG_MASS, color=MUG_COL, friction=(0.5, 0.4), restitution=0.0, contact_offset=0.001,
                com=(0.0, 0.0, 0.035), pieces=tuple(pieces))


def bag_pos():
    return (MUG_XY[0], MUG_XY[1] + MUG_RI - BAG[1] / 2 - 0.004, TOP + MUG_BASE + BAG[2] / 2 + 0.002)


def bag_object() -> Part:
    staple = Piece("box", (0.006, BAG[1] + 0.001, 0.003), (0.0, 0.0, BAG[2] / 2 - 0.004), color=STEEL, collide=False)
    return Part("bag", pos=bag_pos(), size=BAG, exact_z=True, mass=BAG_MASS, color=BAG_COL, friction=(0.8, 0.6),
                restitution=0.0, contact_offset=0.001, max_depenetration=0.3, contact_report=True, pieces=(staple,))


def tag_object() -> Part:
    """The tag hangs beside the mug's handle from the string's turn round it: a hinge at its top edge that
    lets it swing and lets it go when it is pulled away."""
    top = (TAG_XY[0], TAG_XY[1], TAG_Z + TAG[2] / 2)
    return Part("tag", pos=(TAG_XY[0], TAG_XY[1], TAG_Z), size=TAG, exact_z=True, mass=TAG_MASS, color=TAG_COL,
                friction=(1.4, 1.2), restitution=0.0, contact_offset=0.001, max_depenetration=0.3,
                hinge=Hinge(pivot=top, axis="Y", damping=0.00005, lower_deg=-70.0, upper_deg=70.0, break_force=TAG_HOLD),
                tether=Tether(parent="bag", parent_local=(0.0, 0.0, BAG[2] / 2), local=(0.0, 0.0, TAG[2] / 2), length=STRING))


def spoon_object() -> Part:
    """Root: a small bead at the bowl's tip; bowl and handle are laid along the leaning axis from it."""
    a = np.array(SPOON_AXIS0)
    bowl_c = a * BOWL_S
    handle_c = a * (BOWL[2] + HANDLE[2] / 2 - 0.004)
    pieces = (Piece("box", BOWL, tuple(float(v) for v in bowl_c), rot=(LEAN, 0.0, 0.0), color=STEEL),
              Piece("box", HANDLE, tuple(float(v) for v in handle_c), rot=(LEAN, 0.0, 0.0), color=STEEL, friction=(1.4, 1.2)))
    return Part("spoon", pos=SPOON_TIP, size=(0.0025, 0.0025, 0.0025), shape="sphere", exact_z=True, mass=SPOON_MASS,
                color=STEEL, friction=(0.5, 0.4), restitution=0.0, contact_offset=0.001, max_depenetration=0.3,
                com=tuple(float(v) for v in a * 0.045), pieces=pieces)


def plate_object() -> Part:
    x, y = PLATE_XY
    ring = Piece("cylinder", (PLATE_R, 0.0012, PLATE_R), (0.0, 0.0, PLATE_H / 2 + 0.0002), color=(0.30, 0.45, 0.62), collide=False)
    well = Piece("cylinder", (PLATE_R - 0.012, 0.0014, PLATE_R - 0.012), (0.0, 0.0, PLATE_H / 2 + 0.0004), color=CERAMIC, collide=False)
    return Part("plate", pos=(x, y, TOP + PLATE_H / 2), size=(PLATE_R, PLATE_H, PLATE_R), shape="cylinder", axis="Z",
                exact_z=True, color=CERAMIC, friction=(0.6, 0.5), pieces=(ring, well))


def mat_object() -> Part:
    return Part("mat", pos=(0.290, 0.020, TOP + 0.0004), size=(0.460, 0.460, 0.0008), exact_z=True, color=MAT_COL,
                collision="none")


def drop_objects():
    """Drops of tea on the mat, as drawn: beads the task puts where each drop falls (no collider, no weight;
    they wait out of sight under the bench)."""
    return [Part(f"drop{i}", pos=(0.20 + 0.012 * i, -0.10, TOP - 0.30), size=(0.004, 0.004, 0.004), shape="sphere",
                 exact_z=True, mass=0.0005, color=TEA, collision="none", weightless=True) for i in range(DROPS_DRAWN)]


def string_objects(n=14):
    return [Part(f"str{i}", pos=(0.20 + 0.01 * i, 0.10, TOP - 0.30), size=(0.0012, 0.0012, 0.0012), shape="sphere",
                 exact_z=True, mass=0.0002, color=(0.95, 0.95, 0.93), collision="none", weightless=True) for i in range(n)]


# -- measurements ------------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def point(pose, local):
    p = np.asarray(pose, float)
    return p[:3] + quat_matrix(p[3:7]) @ np.asarray(local, float)


def tilt_deg(pose):
    v = quat_matrix(np.asarray(pose, float)[3:7]) @ np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(float(np.clip(v[2], -1, 1))))


def bag_top(pose):
    return point(pose, (0.0, 0.0, BAG[2] / 2))


def bag_lowest(pose):
    """Lowest corner of the bag (world)."""
    p = np.asarray(pose, float)
    r = quat_matrix(p[3:7])
    pts = [p[:3] + r @ np.array([sx * BAG[0] / 2, sy * BAG[1] / 2, sz * BAG[2] / 2])
           for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return min(pts, key=lambda v: v[2])


def tag_top(pose):
    return point(pose, (0.0, 0.0, TAG[2] / 2))


def spoon_axis(pose):
    return quat_matrix(np.asarray(pose, float)[3:7]) @ np.array(SPOON_AXIS0)


def spoon_face(pose):
    return quat_matrix(np.asarray(pose, float)[3:7]) @ np.array(SPOON_FACE0)


def spoon_at(pose, s):
    """Point on the spoon's axis ``s`` metres up from the bowl's tip."""
    return np.asarray(pose, float)[:3] + spoon_axis(pose) * s


def over_mug(p, mug_pose, margin=0.0):
    """A point over the mug's mouth (within its bore, plus ``margin``)."""
    m = np.asarray(mug_pose, float)
    return bool(math.hypot(p[0] - m[0], p[1] - m[1]) < MUG_RI + margin)


def over_plate(p, margin=0.0):
    return bool(math.hypot(p[0] - PLATE_XY[0], p[1] - PLATE_XY[1]) < PLATE_R + margin)


def bag_on_plate(pose):
    p = np.asarray(pose, float)
    return bool(over_plate(p, -0.010) and p[2] < TOP + PLATE_H + 0.035)


def spoon_on_plate(pose):
    """The spoon lying on the plate: its bowl over the plate, the whole spoon low."""
    bowl = spoon_at(pose, BOWL_S)
    top = spoon_at(pose, SPOON_L)
    return bool(over_plate(bowl) and max(bowl[2], top[2]) < TOP + 0.045)


def string_points(a, b, n=14):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = float(np.linalg.norm(a - b))
    sag = math.sqrt(3.0 * max(d, 0.01) * max(0.0, STRING - d) / 8.0)
    out = []
    for i in range(n):
        t = (i + 0.5) / n
        p = a + (b - a) * t
        p[2] -= sag * 4 * t * (1 - t)
        p[2] = max(p[2], TOP + 0.002)
        out.append(p)
    return out
