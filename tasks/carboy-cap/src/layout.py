"""Geometry, thread model constants and start states of the carboy-cap task (pure NumPy; no simulator).

Scene (station frame: x forward from the line between the arm bases, y to the robot's left of their midpoint, z up;
table top z = TOP): a 500 mL HDPE solvent-waste bottle with a polypropylene funnel standing in its neck, its 53 mm screw
cap lying top-up on the bench in front of it, a lab hot plate switched on behind-left of it, and a bench-top dedicated drip tray on the right.

Bottle frame B: origin on the table top under the bottle's axis, +z up along the axis, +x / +y rotated by the
bottle's yaw. The cap is carried by a chain of joints rooted at the bottle (x, y, z slides along B, then rotations
about x, y and the cap's own axis), all free until the thread engages (see ``isaac/capkit.py``); the chain origin is
the centre of the neck's lip, so the cap's chain coordinates are its position relative to the lip.

Outer dimensions follow Fisherbrand 02912030A. Internal geometry and friction are estimated; see README.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station TABLE_TOP)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ------------------------------------------------------------------------------------------------ bottle (root)
BODY_R = 0.0385  # Fisherbrand 02912030A, 77 mm body diameter
BODY_H = 0.112  # straight wall height
SHOULDER_H = 0.012  # shoulder rises to the neck
NECK_R_OUT = 0.0265  # 53 mm neck finish (thread major diameter)
NECK_R_IN = 0.0215  # 43 mm bore
NECK_H = 0.018
H_LIP = BODY_H + SHOULDER_H + NECK_H  # 0.142 m: neck lip above the table
WALL_T = 0.003
BOTTLE_EMPTY_KG = 0.075  # estimated dry assembly mass
WASTE_KG = 0.36  # inert ballast for the dry rig, kg
BOTTLE_MASS = BOTTLE_EMPTY_KG + WASTE_KG
BOTTLE_COM = (0.0, 0.0, 0.035)
LABEL_Z = (0.030, 0.098)  # hazardous-waste label band on the wall
LABEL_HALF_ANGLE = 55.0  # deg either side of the bottle's -x face (faces the robot at yaw 0)

# ------------------------------------------------------------------------------------------------ cap + thread
CAP_R_OUT = 0.0295  # 59 mm across the knurled skirt
CAP_R_IN = 0.0275  # bore: 1.0 mm radial clearance over the 53 mm thread crest
CAP_TOP_T = 0.003  # top plate (with the liner) thickness
CAP_DEPTH = 0.0165  # inner depth from the liner face to the skirt's bottom edge
CAP_H = CAP_TOP_T + CAP_DEPTH  # 19.5 mm
CAP_CHAMFER = 0.0025  # lead-in at the skirt's bottom inner edge
CAP_MASS = 0.018
PITCH = 0.00423  # 6 threads per inch
TURNS = 1.0  # from the thread start to the seat
TRAVEL = PITCH * TURNS  # 4.23 mm: cap rests this high on the thread start, unengaged
PHI_SEAT = 2 * math.pi * TURNS
T_RUN = 0.03  # N m: free-running thread torque (plastic on plastic, no load)
K_SEAT = 1.5  # N m / rad: torque rise past the seat (liner compression)
SEAT_STOP_DEG = 30.0  # past the seat the liner is fully compressed: hard stop
T_STOP = 4.0  # N m held at the hard stop
CAPTURE_R = 0.0016  # cap axis within this of the neck axis (the skirt over the neck keeps it within ~1 mm)
CAPTURE_TILT_DEG = 5.0
ENGAGE_DEG = 20.0  # past this clockwise turn from the thread start the flanks overlap (cap no longer lifts off)
SEALED_GAP = 0.00025  # m: liner within 0.25 mm of the lip = seated (turned to the seat)

# ------------------------------------------------------------------------------------------------ funnel
F_R_TOP = 0.0375  # 75 mm funnel
F_HALF_ANGLE = 30.0  # deg (60 deg funnel)
F_WALL = 0.002
F_STEM_R = 0.009  # 18 mm stem
F_STEM_L = 0.020
F_CONE_H = (F_R_TOP - F_STEM_R) / math.tan(math.radians(F_HALF_ANGLE))  # 49.4 mm
F_TAB = (0.018, 0.020, 0.003)  # hanging tab on the rim: length (radial), width, thickness
F_MASS = 0.022
F_COM_Z = 0.026  # above the stem top
# funnel frame: origin at the stem top (cone apex end), +z up along the axis; tab along +x at the rim
F_REST_Z = (NECK_R_IN - F_STEM_R) / math.tan(math.radians(F_HALF_ANGLE))  # stem top below the lip (cone on the bore edge)

# ------------------------------------------------------------------------------------------------ hot plate (static)
HP_CENTER = (0.490, 0.230)
HP_SIZE = (0.26, 0.18, 0.100)  # depth (x), width (y), height: lab stirring hot plate, 7 x 7 in class top
HP_TOP_SIZE = (0.18, 0.15)  # ceramic top plate

# Dedicated, four-sided drip tray on a fixed bench riser, inside dimensions in metres.
TRAY_X = (0.170, 0.350)
TRAY_Y = (-0.335, -0.145)
TRAY_Z = (0.105, 0.175)  # floor and rim heights above the table
TRAY_WALL = 0.006

FRICTION = dict(bottle=0.5, cap=1.0, funnel=0.6, table=0.5, tray=0.5)

# ------------------------------------------------------------------------------------------------ start states
NOMINAL = (0.262, 0.050)  # bottle axis x, y
BOTTLE_D = (0.010, 0.025)  # seeded bottle offset |d| from the nominal (any direction)
BOTTLE_YAW = (-15.0, 15.0)
FUNNEL_TAB_YAW = (-162.0, -128.0)  # station yaw of the funnel's tab (toward the right arm; the hand cannot reach a tab turned further left)
CAP_BOX = ((0.150, 0.210), (0.130, 0.200))  # cap centre region (x, y)
CAP_MIN_DIST = 0.090  # from the bottle's axis
NAMED_SETUPS = (
    dict(bx=0.274, by=0.064, byaw=5.0, tab=-130.0, cx=0.175, cy=0.180),
    dict(bx=0.249, by=0.040, byaw=-8.0, tab=-132.0, cx=0.190, cy=0.150),
    dict(bx=0.279, by=0.035, byaw=10.0, tab=-145.0, cx=0.160, cy=0.190),
)


OFFICIAL_SEED = 1415158438  # inspect-robots trial seed of eval seed 0 (the default start)
GUARD_BOTTLE = 0.015  # open-loop guard: a seeded start whose bottle is within 15 mm ...
GUARD_TAB_DEG = 25.0  # ... and whose tab is within 25 deg of the default start's ...
GUARD_CAP = 0.030  # ... gets its cap redrawn at least 30 mm from the default start's cap


def _seeded(n):
    rng = np.random.default_rng(int(n))
    d = float(rng.uniform(*BOTTLE_D))
    a = float(rng.uniform(0.0, 2 * math.pi))
    bx, by = NOMINAL[0] + d * math.cos(a), NOMINAL[1] + d * math.sin(a)
    byaw = float(rng.uniform(*BOTTLE_YAW))
    tab = float(rng.uniform(*FUNNEL_TAB_YAW))
    for _ in range(200):
        cx, cy = float(rng.uniform(*CAP_BOX[0])), float(rng.uniform(*CAP_BOX[1]))
        if math.hypot(cx - bx, cy - by) >= CAP_MIN_DIST:
            break
    return dict(bx=bx, by=by, byaw=byaw, tab=tab, cx=cx, cy=cy, name=f"seed {int(n)}")


def setup_spec(setup):
    """Start state {'bx', 'by', 'byaw', 'tab', 'cx', 'cy', 'name'}. ``setup`` is a named-setup index or ('seed', n).

    Seeded starts: bottle 10-25 mm from its nominal spot in any direction, yaw, funnel tab yaw and cap spot drawn from
    seed n. Open-loop guard: when the bottle is within GUARD_BOTTLE and the tab within GUARD_TAB_DEG of the default
    (official-seed) start, the cap is redrawn (from a second seeded stream) at least GUARD_CAP from the default start's
    cap, so that no start lies jointly near the default one (a replayed default trajectory then misses the cap)."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        n = int(setup[1])
        sp = _seeded(n)
        if n == OFFICIAL_SEED:
            return sp
        ref = _seeded(OFFICIAL_SEED)
        near = (math.hypot(sp["bx"] - ref["bx"], sp["by"] - ref["by"]) < GUARD_BOTTLE
                and abs(sp["tab"] - ref["tab"]) < GUARD_TAB_DEG)
        if near and math.hypot(sp["cx"] - ref["cx"], sp["cy"] - ref["cy"]) < GUARD_CAP:
            rng = np.random.default_rng([n, 7])
            for _ in range(2000):
                cx, cy = float(rng.uniform(*CAP_BOX[0])), float(rng.uniform(*CAP_BOX[1]))
                if (math.hypot(cx - sp["bx"], cy - sp["by"]) >= CAP_MIN_DIST
                        and math.hypot(cx - ref["cx"], cy - ref["cy"]) >= GUARD_CAP):
                    sp["cx"], sp["cy"] = cx, cy
                    break
        return sp
    k = int(setup) % len(NAMED_SETUPS)
    return dict(NAMED_SETUPS[k], name=f"setup {k}")


def bottle_pose(spec):
    return np.array([spec["bx"], spec["by"], TOP]), rz(spec["byaw"])


def lip_station(spec):
    p, r = bottle_pose(spec)
    return p + r @ np.array([0.0, 0.0, H_LIP])


def cap_chain_start(spec):
    """Cap chain coordinates (x, y, z in B relative to the lip; rx, ry, rz) for the cap lying top-up on the table."""
    p, r = bottle_pose(spec)
    rel = r.T @ (np.array([spec["cx"], spec["cy"], TOP + CAP_DEPTH]) - (p + r @ np.array([0.0, 0.0, H_LIP])))
    return np.array([rel[0], rel[1], rel[2], 0.0, 0.0, 0.0])


def funnel_start(spec):
    """Funnel pose (station) standing in the neck, its tab at the spec's yaw."""
    lip = lip_station(spec)
    return lip - np.array([0.0, 0.0, F_REST_Z - 0.0008]), rz(spec["tab"])


def h_thread(phi):
    """Height of the cap's liner face above the lip on the thread at clockwise turn ``phi`` (rad) from the start."""
    return float(np.clip(TRAVEL - PITCH * phi / (2 * math.pi), 0.0, TRAVEL))


def in_tray(pos, rotation):
    """The funnel centre is in the drip tray, with its actual lower surface at the floor."""
    centre = pos + rotation[:, 2] * F_COM_Z
    if not (TRAY_X[0] < centre[0] < TRAY_X[1] and TRAY_Y[0] < centre[1] < TRAY_Y[1]
            and centre[2] < TOP + TRAY_Z[1]):
        return False
    # The entire stem opening must drain into the basin, even when the rim
    # and the funnel's centre lie inside while the stem points over an edge.
    axis = rotation[:, 2]
    outlet = pos - F_STEM_L * axis
    radius_xy = F_STEM_R * np.sqrt(np.maximum(0.0, 1.0 - axis[:2]**2))
    for coordinate, radius, (low, high) in zip(outlet[:2], radius_xy, (TRAY_X, TRAY_Y)):
        if coordinate - radius < low or coordinate + radius > high:
            return False
    vertical = rotation[2]
    radial = np.linalg.norm(vertical[:2])
    stem = min(0.0, -F_STEM_L * vertical[2]) - F_STEM_R * radial
    rim = F_CONE_H * vertical[2] - F_R_TOP * radial
    tl, tw, tt = F_TAB
    tab = (vertical[0] * (F_R_TOP + tl / 2 - 0.002) + vertical[2] * (F_CONE_H - tt / 2)
           - np.abs(vertical) @ np.array([(tl + 0.004) / 2, tw / 2, tt / 2]))
    lowest = pos[2] + min(stem, rim, tab)
    return bool(abs(lowest - (TOP + TRAY_Z[0])) <= 0.003)


# ------------------------------------------------------------------------------------------------ shapes
def box(center, size):
    hx, hy, hz = (s / 2 for s in size)
    c = np.asarray(center, float)
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + c


def box_xyz(xr, yr, zr):
    return box(((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2),
               (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]))


def cyl_z(center, r, h, n=24):
    c = np.asarray(center, float)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.array([[r * math.cos(t), r * math.sin(t), z] for z in (-h / 2, h / 2) for t in a]) + c


def ring_segments(r_in, r_out, z0, z1, n=32, phase=0.0, r_in_bottom=None, chamfer=0.0):
    """Convex pieces of a tube (n sectors). ``r_in_bottom`` / ``chamfer``: the inner radius flares to r_in_bottom over
    the bottom ``chamfer`` height (lead-in)."""
    out = []
    for i in range(n):
        a0 = phase + 2 * math.pi * i / n
        a1 = phase + 2 * math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            pts += [[r_out * c, r_out * s, z0], [r_out * c, r_out * s, z1], [r_in * c, r_in * s, z1]]
            if chamfer > 0.0:
                rb = r_in_bottom
                pts += [[r_in * c, r_in * s, z0 + chamfer], [rb * c, rb * s, z0]]
            else:
                pts += [[r_in * c, r_in * s, z0]]
        out.append(np.array(pts))
    return out


def cone_segments(n=16):
    """Funnel cone wall pieces (funnel frame), outer surface from the stem top to the rim."""
    t = math.tan(math.radians(F_HALF_ANGLE))
    wn = F_WALL / math.cos(math.radians(F_HALF_ANGLE))
    out = []
    for i in range(n):
        a0 = 2 * math.pi * i / n
        a1 = 2 * math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            for z in (0.0, F_CONE_H):
                r = F_STEM_R + z * t
                pts += [[r * c, r * s, z], [(r - wn) * c, (r - wn) * s, z]]
        out.append(np.array(pts))
    return out
