"""Scene layout and geometry of the clock-dst task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

Clock frame K: origin at the centre of the clock's back plane (the plane that touches the wall), z_K out of the face,
y_K toward 12 o'clock, x_K toward 3 o'clock. Hanging level on the wall: z_K = -x (station), y_K = +z, x_K = -y.
Representative 10 in (254 mm) round plastic-case quartz wall clock with a 56 mm square quartz movement and a keyhole
hanger cut in its back (no specific product's drawing is copied; see README fidelity).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# K axes of a level clock hanging on the wall, as columns in the station frame
R_WALL = np.column_stack([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])

# ---------------------------------------------------------------- wall section and nail
WALL_X = 0.42  # front face of the wall board (1/2 in gypsum board on a bench stand)
WALL_T = 0.0127
WALL_W = 0.60  # along y
WALL_H = 0.50  # board height above the table
NAIL_SHANK_R = 0.0028702 / 2  # 6d common nail (ASTM F1667): shank 0.113 in
NAIL_HEAD_R = 0.0067564 / 2  # head 0.266 in
NAIL_HEAD_T = 0.0010
NAIL_PROUD = 0.0100  # head face 10 mm proud of the wall (driven slightly upward in practice; modelled level)
NAIL_Y0 = 0.0
NAIL_Z0 = TOP + 0.255  # nominal nail height (clock centre about 0.18 m above the bench)

# ---------------------------------------------------------------- clock (K frame)
CLOCK_R = 0.127  # bezel outer radius (10 in)
BEZEL_RI = 0.111  # bezel inner radius (dial window)
BEZEL_Z = (0.024, 0.042)  # bezel ring, 18 mm deep, 16 mm wide (r 111..127 mm)
SHELL_R = (0.1065, 0.1095)  # back shell side wall (3 mm), set 17.5 mm in from the bezel rim
SHELL_Z = (0.0, 0.024)
BACK_T = 0.0025  # back panel thickness
DIAL_Z = (0.0255, 0.0275)
CRYSTAL_Z = (0.031, 0.033)  # glass set 9 mm behind the bezel front (deep bezel)
CRYSTAL_R = 0.1115
MOVE_SIZE = (0.056, 0.056, 0.016)  # standard quartz movement body
MOVE_Z = (DIAL_Z[0] - 0.016, DIAL_Z[0])
OPENING = 0.060  # square knob-access opening in the back panel, centred
KNOB_R = 0.0075  # setting knob (thumb wheel), 15 mm
KNOB_Z = (0.0040, 0.0090)  # 4 mm inside the back plane, 0.5 mm clear of the movement
KEY_HOLE_C = np.array([0.0, 0.070])  # keyhole: entry hole centre (K x, y)
KEY_HOLE_R = 0.0055  # entry hole 11 mm (passes the 6.8 mm nail head with 2.1 mm play)
KEY_SLOT_W = 0.0040  # slot 4 mm wide (0.57 mm a side on the 2.87 mm shank)
KEY_SLOT_TOP = 0.0800  # slot top (round end, radius KEY_SLOT_W / 2)
MINUTE_LEN, HOUR_LEN = 0.098, 0.064
HAND_Z = {"hour": 0.0283, "minute": 0.0293, "second": 0.0303}
CLOCK_MASS = 0.50  # kg incl. glass crystal and an AA cell (typical 10 in plastic wall clock)
CLOCK_COM_Z = 0.022
KNOB_MASS = 0.004
# friction (static = dynamic): ABS case on the finger pads, steel nail, painted wall, bench top
CASE_FRICTION = 0.7
NAIL_FRICTION = 0.3
WALL_FRICTION = 0.4
KNOB_FRICTION = 0.9  # knurled knob rim

# setting-knob drag (the movement's friction clutch): breakaway torque about the knob axis (estimate; see README)
CLUTCH_NM = 0.008

# ---------------------------------------------------------------- time
TARGET_MIN = 10 * 60 + 10  # the instruction's time, 10:10
# Start time shown (standard time, drifted): 20..95 min behind 10:10, except within 6 min of exactly one hour behind.
# Wide so that no face-blind "always turn N minutes" policy reaches full credit (2 min window) on more than ~6 % of
# seeds; the band excludes the daylight-saving rule (60 min). Offsets up to 95 min fit the knob drive's range.
NEED_SEGMENTS = ((20.0, 54.0), (66.0, 95.0))  # minutes to add
START_SEGMENTS = tuple((TARGET_MIN - hi, TARGET_MIN - lo) for lo, hi in reversed(NEED_SEGMENTS))
START_RANGE = (START_SEGMENTS[0][0], START_SEGMENTS[-1][1])
NAMED_STARTS = (9 * 60 + 2, 8 * 60 + 41, 9 * 60 + 37, 9 * 60 + 18, 8 * 60 + 52)
# nail position jitter per seeded trial (m): the hang must be found from the nail seen on the wall
NAIL_JITTER_Y = 0.015
NAIL_JITTER_Z = 0.012

# knob joint coordinate: q = minute-hand angle clockwise from 12 (seen from the front) minus pi (the minute hand is
# authored pointing at 6 when q = 0), so a 45..75 min advance to 10:10 always ends at q = 60 deg - 180 deg + 360 deg
Q_OFFSET = math.pi


def minute_angle(minutes):
    """Minute-hand angle clockwise from 12 (rad), in [0, 2 pi)."""
    return math.radians((minutes % 60.0) * 6.0)


Q_END = math.radians(60.0) - Q_OFFSET + 2 * math.pi  # knob joint position when the hands show 10:10 (4.19 rad)


def q_start(start_min):
    """Knob joint position at the start: Q_END less the turn still needed (2 pi per hour), so the joint stays inside the
    drive's +-(2 pi - 0.2) range for any start 0..98 min behind 10:10 (the minute hand angle is pi + q)."""
    return Q_END - (TARGET_MIN - start_min) / 60.0 * 2 * math.pi


def hour_angle(minutes):
    """Hour-hand angle clockwise from 12 (rad) in (-pi, pi]."""
    a = math.radians((minutes / 60.0 % 12.0) * 30.0)
    return a - 2 * math.pi if a > math.pi else a


def shown_minutes(start_min, q, q0):
    """Time shown (minutes since 00:00) after the knob joint moved from q0 to q (positive = forward)."""
    return start_min + (q - q0) / (2 * math.pi) * 60.0


def fmt(minutes):
    m = minutes % (12 * 60)
    h = int(m // 60)
    return f"{h if h else 12}:{m - 60 * h:05.2f}"


# ---------------------------------------------------------------- setups
def setup_spec(setup):
    """Start state: {'start_min', 'nail_y', 'nail_z', 'name'}. ``setup`` is a named setup index (int) or ('seed', n):
    start time uniform over START_SEGMENTS (by length) and nail offset uniform in +-jitter, from seed n."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        rng.uniform(0.0, 1.0)  # (kept so the nail offsets below stay those of the earlier freeze)
        # start time from its own stream of the seed (stream 11)
        u = float(np.random.default_rng((int(setup[1]), 11)).uniform(0.0, sum(hi - lo for lo, hi in START_SEGMENTS)))
        for lo, hi in START_SEGMENTS:
            if u < hi - lo:
                start = lo + u
                break
            u -= hi - lo
        else:
            start = float(START_SEGMENTS[-1][1])
        dy = float(rng.uniform(-NAIL_JITTER_Y, NAIL_JITTER_Y))
        dz = float(rng.uniform(-NAIL_JITTER_Z, NAIL_JITTER_Z))
        return dict(
            start_min=start,
            nail_y=NAIL_Y0 + dy,
            nail_z=NAIL_Z0 + dz,
            name=f"seed {int(setup[1])}",
        )
    k = int(setup) % len(NAMED_STARTS)
    offs = ((0.0, 0.0), (0.010, -0.008), (-0.012, 0.006), (0.006, 0.010), (-0.008, -0.010))[k]
    return dict(
        start_min=float(NAMED_STARTS[k]),
        nail_y=NAIL_Y0 + offs[0],
        nail_z=NAIL_Z0 + offs[1],
        name=f"setup {k}",
    )


def nail_point(setup):
    """Nail axis point on the wall face (station frame)."""
    sp = setup_spec(setup)
    return np.array([WALL_X, sp["nail_y"], sp["nail_z"]])


# where the shank sits in the keyhole when the clock hangs: in the slot's round top
HANG_SHANK_K = np.array([0.0, KEY_SLOT_TOP - KEY_SLOT_W / 2 + (KEY_SLOT_W / 2 - NAIL_SHANK_R)])


def clock_hung_pose(setup):
    """(pos of K origin, rot) of the clock hanging level on the nail, back flat on the wall."""
    n = nail_point(setup)
    r = R_WALL
    p = n - r @ np.array([HANG_SHANK_K[0], HANG_SHANK_K[1], 0.0])
    return p, r


def nail_in_k(pos, rot, setup_nail):
    """Nail axis point (on the wall face) and head point expressed in the clock frame K."""
    n = np.asarray(setup_nail, float)
    head = n + np.array([-(NAIL_PROUD - NAIL_HEAD_T), 0.0, 0.0])  # head underside
    return rot.T @ (n - pos), rot.T @ (head - pos)


# ---------------------------------------------------------------- convex piece generation
def circle_pts(r, n, center=(0.0, 0.0), phase=0.0):
    return np.array(
        [
            [
                center[0] + r * math.cos(phase + 2 * math.pi * i / n),
                center[1] + r * math.sin(phase + 2 * math.pi * i / n),
            ]
            for i in range(n)
        ]
    )


def prism(poly2d, z0, z1):
    p = np.asarray(poly2d, float)
    return np.vstack([np.c_[p, np.full(len(p), z0)], np.c_[p, np.full(len(p), z1)]])


def annulus_cells(r0, r1, sector_deg, phase_deg=0.0):
    n = int(round(360.0 / sector_deg))
    cells = []
    for i in range(n):
        a0 = math.radians(phase_deg + i * 360.0 / n)
        a1 = math.radians(phase_deg + (i + 1) * 360.0 / n)
        cells.append(
            np.array(
                [
                    [r0 * math.cos(a0), r0 * math.sin(a0)],
                    [r1 * math.cos(a0), r1 * math.sin(a0)],
                    [r1 * math.cos(a1), r1 * math.sin(a1)],
                    [r0 * math.cos(a1), r0 * math.sin(a1)],
                ]
            )
        )
    return cells


def keyhole_half_width(y):
    """Half width (|x|) of the keyhole opening at height y (K), 0 where closed."""
    cy = KEY_HOLE_C[1]
    w = 0.0
    dy = y - cy
    if abs(dy) < KEY_HOLE_R:
        w = math.sqrt(KEY_HOLE_R**2 - dy * dy)
    top_c = KEY_SLOT_TOP - KEY_SLOT_W / 2
    if cy <= y <= top_c:
        w = max(w, KEY_SLOT_W / 2)
    elif top_c < y < KEY_SLOT_TOP:
        d = y - top_c
        w = max(w, math.sqrt(max(0.0, (KEY_SLOT_W / 2) ** 2 - d * d)))
    return w


def _disc_halfwidth(y, r):
    return math.sqrt(max(0.0, r * r - y * y))


def back_panel_cells():
    """Convex 2D cells (K x, y) covering the back panel disc (radius SHELL_R[1]) minus the square knob opening and the
    keyhole. Horizontal bands; each band's cells are quads between the disc edge / hole edges at the band's bottom
    and top (fine bands at the keyhole so its outline is followed within ~0.05 mm)."""
    R = SHELL_R[1]
    h = OPENING / 2
    ky0, ky1 = KEY_HOLE_C[1] - KEY_HOLE_R, KEY_SLOT_TOP
    edges = sorted(
        set(
            [-R, R, -h, h]
            + list(np.linspace(-h, h, 9))
            + list(-R + (R - h) * (1 - np.cos(np.linspace(0, np.pi / 2, 20))))
            + list(np.linspace(h, ky0 - 0.003, 4))
            + list(np.linspace(ky0 - 0.003, ky0, 2))
            + list(np.linspace(ky0, ky1, 31))
            + list(np.linspace(ky1, ky1 + 0.003, 2))
            + list(R - (R - ky1 - 0.003) * (1 - np.cos(np.linspace(0, np.pi / 2, 14))))
        )
    )
    edges = [e for e in edges if -R <= e <= R]
    cells = []
    for y0, y1 in zip(edges[:-1], edges[1:]):
        if y1 - y0 < 1e-6:
            continue
        ym = 0.5 * (y0 + y1)
        o0, o1 = _disc_halfwidth(y0, R), _disc_halfwidth(y1, R)
        if o0 < 1e-5 and o1 < 1e-5:
            continue
        if -h < ym < h:  # band through the knob opening: left and right of it
            for s in (-1.0, 1.0):
                cells.append(
                    np.array([[s * h, y0], [s * o0, y0], [s * o1, y1], [s * h, y1]])
                )
            continue
        if ky0 - 1e-9 <= y0 and y1 <= ky1 + 1e-9:  # keyhole band
            k0, k1 = keyhole_half_width(y0), keyhole_half_width(y1)
            km = keyhole_half_width(ym)
            if km > 0 or k0 > 0 or k1 > 0:
                for s in (-1.0, 1.0):
                    cells.append(
                        np.array(
                            [[s * k0, y0], [s * o0, y0], [s * o1, y1], [s * k1, y1]]
                        )
                    )
                continue
        cells.append(np.array([[-o0, y0], [o0, y0], [o1, y1], [-o1, y1]]))
    # drop degenerate cells
    out = []
    for c in cells:
        x, y = c[:, 0], c[:, 1]
        area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
        if area > 2e-8:
            out.append(c)
    return out


def clock_inertia():
    m, r, d = CLOCK_MASS, CLOCK_R, BEZEL_Z[1]
    ixx = m * (3 * r * r + d * d) / 12 * 0.8  # mass concentrated toward the rim and glass: approx
    return (ixx, ixx, m * r * r / 2 * 0.8)
