"""Every dimension and threshold of the task, in one place, and the clipper's geometry and the demo's paths as plain
functions. NumPy only.

A lever nail clipper of the large kind (82 to 92 mm long, 12.7 to 19 mm wide, 45 to 60 g: Trim 3.25 x 0.5 in,
Seki Edge SS-107 92 mm, Tweezerman 45 g). Sizes below are ours, solved so that the part behaves as such a clipper
does (see the README): a lever on a pin that swivels, a sprung upper blade, a heel on the lever that bears on it.

Clipper frame C (millimetres in this file unless a name says metres): origin at the cutting edge on the slot axis,
x toward the tail, y across, z up from the slot floor. It is mechkit's base frame of the clipper, and in
the world it is the tray's slot: x along world x (jaws toward the robot), z up. The tray (88 x 18 mm slot, walls 3 mm
thick and 7 mm high, floor 3 mm thick) is fixed to the table; the clipper lies loose in it.

Two mechkit assemblies:
  tray          static.
  clipper       free, ONE articulation. Base: the lower blade, the tail block and the upper blade (one rigid body:
                the blades are riveted at the tail and the jaws do not close in this build). Link ``pin`` (hinge
                about Z, the swivel). Link ``lever`` (child of ``pin``, hinge about -Y, positive lifts the tip over
                the top).
The kit switches self-collision off inside an articulation, so the lever's two rests on the upper blade are stops in
its hinge, set where the lever really meets the blade: LEVER_LOWER (0) where the heel stands on the blade's level
seat round the pin, LEVER_UPPER where the turned-over lever's tip lies on the sloping blade. The seat is level for
22 mm behind the jaws, so the heel's rest is the same at every swing angle, and the turned-over lever's stop is
within 0.1 mm of where it meets the jaw end when it points out past the jaws (tests).
All joints read 0 in the pose as drawn: the cutting position (the lever to the tail, raised, its heel on the blade).

Angles: the swivel psi is the pin's hinge (0: the lever points to the tail; plus or minus pi: out past the jaws). The
lever's elevation eps is measured in the pin's own frame from the pin's x axis (the lever's direction at psi = 0), up
and over the top, so the lever hinge reads eps - E0 (E0 = the drawn elevation).
"""

from __future__ import annotations

import math

import numpy as np

MM = 0.001
TOP = 0.7475  # the station bench top
CONTROL_HZ = 20.0
EPS = 1e-9

# -- where the tray stands (world, m) ------------------------------------------------------------------------------
OX, OY = 0.2575, -0.305  # the clipper frame's origin: the cutting edge on the slot's axis (m)
TRAY_FLOOR = 3.0  # mm
FLOOR_Z = TOP + TRAY_FLOOR * MM  # the slot floor: where the lower blade lies
ORIGIN = (OX, OY, FLOOR_Z)  # clipper frame origin in the world
TRAY_ORIGIN = (OX, OY, TOP)
SLOT_X = (-1.5, 86.5)
SLOT_Y = (-9.0, 9.0)
WALL, WALL_H = 3.0, 7.0

# -- the clipper (mm) ----------------------------------------------------------------------------------------------
LEN, WIDTH = 85.0, 15.0
LOW_T = 2.0  # lower blade thickness
LIP_H, LIP_T = 1.8, 2.0  # both lips: height and length along the blade
PIN_X, PIN_R, PIN_TOP = 14.0, 1.75, 14.0  # the pin stands from the lower blade's top (z 2) to z 14
ZH = 12.5  # the lever's hinge axis height
LEV, LEV_W, LEV_T = 57.0, 12.0, 2.2  # hinge to tip; width; thickness
LEV_X0 = 2.5  # the plate starts this far from the hinge
KNUCKLE_R = 1.1
HEEL_X, HEEL_C, HEEL_R, HEEL_L = 6.0, 3.1, 1.5, 8.0  # heel: along the lever, below the mid-plane, radius, length
TAIL = (77.0, 85.0, 3.0)  # tail block x0, x1, height above the lower blade's top (z 2..5)
PIV = (79.0, 3.5)  # the upper blade's pivot (x, z)
ALPHA_DEG = 5.5  # the upper blade rises this much from the tail ...
KINK_X = 22.0  # ... up to here; from here to the jaws it is level (the seat round the pin)
ALPHA = math.radians(ALPHA_DEG)
BLADE_T = 2.0
BLADE_S = (3.0, (PIV[0] - KINK_X) / math.cos(ALPHA))  # the sloping part, along it from the pivot: 3 to 57.26
LEVEL_C = PIV[1] + (PIV[0] - KINK_X) * math.tan(ALPHA)  # the level part's mid-plane height: 8.99
LEVEL_TOP = LEVEL_C + BLADE_T / 2  # the seat: 9.99 over the slot floor
HOLE_X = (10.5, 17.5)  # the pin's hole in the level part; across it is 7 mm (y -3.5..3.5)
HOLE_Y = 3.5
RIM_Y = (3.5, 7.5)

# -- joints --------------------------------------------------------------------------------------------------------
PIN_LIMIT = 3.67  # rad, either way round
PIN_DAMPING, PIN_FRICTION = 0.0002, 0.0003
LEVER_DAMPING, LEVER_FRICTION = 0.0001, 0.0
# LEVER_LOWER (0, the heel on its seat) and LEVER_UPPER (turned over onto the blade) are solved below.

# -- masses (kg) and surface ---------------------------------------------------------------------------------------
M_BASE, M_PIN, M_LEVER = 0.042, 0.002, 0.010  # base: lower blade 19, tail block 6, upper blade 17
FRICTION = (0.25, 0.2)  # steel on steel; the fingers are 1.2 and PhysX averages: about 0.7 on a finger
SOLVER_ITERATIONS = 32

STEEL = (0.62, 0.63, 0.66)
DARK_STEEL = (0.38, 0.40, 0.44)
TRAY_COLOUR = (0.45, 0.33, 0.24)

# -- scoring thresholds --------------------------------------------------------------------------------------------
TAIL_PLAN_DEG = 10.0  # TO_TAIL: within this of +x_C in plan
PAST_PLAN_DEG = 30.0  # PAST_JAWS: within this of -x_C in plan
PLAN_MIN_MM = 20.0  # the lever's plan length must be at least this (per LEV)
FLAT_GAP = 2.0 * MM  # the lever tip's blade-side face to the upper blade's top plane
SLOT_TOL = 1.5 * MM  # each bottom corner of the lower blade within this of the slot floor
LET_GO = 0.030  # every finger point at least 30 mm from every part of the clipper
STILL_WINDOW_S = 1.0
STILL_MOVE = 1.0 * MM
RESET_START_TOL = 0.005
RESET_JUMP = 0.020
RESET_TURN_DEG = 20.0
RESET_AT_START_DEG = 1.0


def bisect(f, lo: float, hi: float, n: int = 80) -> float:
    for _ in range(n):
        mid = (lo + hi) / 2
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


# -- the upper blade's top and the lever's two stops ---------------------------------------------------------------
def blade_top(x: float) -> float:
    """Height (mm) of the upper blade's top surface over clipper x: level from the jaws to KINK_X, then falling 5.5 deg
    to the tail."""
    if x <= KINK_X:
        return LEVEL_TOP
    return PIV[1] + (PIV[0] - x) * math.tan(ALPHA) + (BLADE_T / 2) / math.cos(ALPHA)


def heel_xz(th: float, cos_psi: float = 1.0) -> tuple[float, float]:
    """(x, z) of the heel's centre line (mm) with the lever at elevation ``th`` (rad, from the pin's own x axis, which
    points along cos_psi of the clipper's x)."""
    return (PIN_X + cos_psi * (HEEL_X * math.cos(th) + HEEL_C * math.sin(th)),
            ZH + HEEL_X * math.sin(th) - HEEL_C * math.cos(th))


def heel_gap(th: float, cos_psi: float = 1.0) -> float:
    """Gap (mm) from the heel's underside to the upper blade's top under it."""
    hx, hz = heel_xz(th, cos_psi)
    return hz - HEEL_R - blade_top(hx)


def rest_elevation(psi: float = 0.0) -> float:
    """The lever's resting elevation (rad, pin frame) on its heel. The heel stands within 6.7 mm of the pin, on the
    level seat, so the rest is the same at every swing angle (the argument is kept for the callers)."""
    return bisect(lambda t: heel_gap(t, math.cos(psi)), 0.0, math.radians(80.0))


TH0 = rest_elevation(0.0)  # 18.7 deg: the lever to the tail, on its heel
TH180 = rest_elevation(math.pi)  # the same, out past the jaws
E0 = TH0  # the drawn elevation: joint 0 of the lever, on its lower stop
LEVER_LOWER = 0.0  # the heel on its seat


def _stow_gap(beta: float) -> float:
    """Gap (mm) from the turned-over lever's tip (the corner of its blade-side face) to the sloping blade under it,
    the lever ``beta`` rad below level, pointing to the tail."""
    x = PIN_X + LEV * math.cos(beta) - (LEV_T / 2) * math.sin(beta)
    z = ZH - LEV * math.sin(beta) - (LEV_T / 2) * math.cos(beta)
    return z - blade_top(x)


BETA_STOW = bisect(_stow_gap, 0.0, math.radians(20.0))  # 6.13 deg below level, lever to the tail, turned over
EPS_STOW = math.pi + BETA_STOW  # pin frame at psi = pi
LEVER_UPPER = EPS_STOW - E0  # the turned-over stop: about 2.94 rad


def stow_kink_gap() -> float:
    """Gap (mm) from the stowed lever's blade-side face to the top of the kink (x = KINK_X): it rests on its tip."""
    b = BETA_STOW
    return ZH - (KINK_X - PIN_X) * math.tan(b) - (LEV_T / 2) / math.cos(b) - LEVEL_TOP


def _fwd_gap(beta: float) -> float:
    """Gap (mm) from the lever's blade-side face, turned over and pointing out past the jaws ``beta`` rad below level,
    to the jaw end's top front edge (x 0)."""
    return ZH - PIN_X * math.tan(beta) - (LEV_T / 2) / math.cos(beta) - LEVEL_TOP


BETA_FWD = bisect(_fwd_gap, math.radians(-10.0), math.radians(20.0))  # 5.73 deg below level, past the jaws
EPS_FWD = math.pi + BETA_FWD  # pin frame at psi = 0: within 0.4 deg of the stop (0.09 mm at the jaw edge)


def far_rest(psi: float = 0.0) -> float:
    """Where a lever let go beyond upright lies: on its turned-over stop (the argument is kept for the callers)."""
    return EPS_STOW


# the jaws: the level part's underside over the two lips
JAW_GAP = (LEVEL_C - BLADE_T / 2) - LIP_H - (LOW_T + LIP_H)  # 2.39 mm
LEVER_RATIO = LEV / HEEL_X


# -- the shapes of the clipper (builders return mechkit-free tuples; task.py wraps them) ---------------------------
def lower_shapes() -> list[tuple]:
    """(kind, centre, size or (radius, height, axis), ypr_deg, colour) in mm."""
    return [
        ("box", (LEN / 2, 0.0, LOW_T / 2), (LEN, WIDTH, LOW_T), None, STEEL),
        ("box", (LIP_T / 2, 0.0, LOW_T + LIP_H / 2), (LIP_T, WIDTH, LIP_H), None, STEEL),
    ]


def pin_shapes() -> list[tuple]:
    return [("cyl", (PIN_X, 0.0, (LOW_T + PIN_TOP) / 2), (PIN_R, PIN_TOP - LOW_T, "Z"), None, DARK_STEEL)]


def lever_shapes(elev: float = E0) -> list[tuple]:
    """The lever drawn at elevation ``elev`` (rad) about -Y through the hinge: plate, knuckle, heel."""
    e = math.degrees(elev)
    ux, uz = math.cos(elev), math.sin(elev)  # along the lever
    nx, nz = -math.sin(elev), math.cos(elev)  # the lever's own z (away from the heel)
    mid = (LEV_X0 + LEV) / 2

    def at(s, t):
        return (PIN_X + s * ux + t * nx, 0.0, ZH + s * uz + t * nz)

    return [
        ("box", at(mid, 0.0), (LEV - LEV_X0, LEV_W, LEV_T), (0.0, -e, 0.0), DARK_STEEL),
        ("cyl", (PIN_X, 0.0, ZH), (KNUCKLE_R, LEV_W, "Y"), None, DARK_STEEL),
        ("cyl", at(HEEL_X, -HEEL_C), (HEEL_R, HEEL_L, "Y"), None, DARK_STEEL),
    ]


def tail_shapes() -> list[tuple]:
    return [("box", ((TAIL[0] + TAIL[1]) / 2, 0.0, LOW_T + TAIL[2] / 2), (TAIL[1] - TAIL[0], WIDTH, TAIL[2]),
             None, STEEL)]


def _level_box(x0: float, x1: float, y0: float, y1: float) -> tuple:
    return ("box", ((x0 + x1) / 2, (y0 + y1) / 2, LEVEL_C), (x1 - x0, y1 - y0, BLADE_T), None, STEEL)


def blade_shapes() -> list[tuple]:
    """The upper blade: the sloping part (pitched 5.5 deg, from the tail block to the kink), the level part round the
    pin with its 7 x 7 mm hole, and its lip under the jaw end."""
    h = WIDTH / 2
    sc = (BLADE_S[0] + BLADE_S[1]) / 2
    slope = ("box", (PIV[0] - sc * math.cos(ALPHA), 0.0, PIV[1] + sc * math.sin(ALPHA)),
             (BLADE_S[1] - BLADE_S[0], WIDTH, BLADE_T), (0.0, ALPHA_DEG, 0.0), STEEL)
    return [
        slope,
        _level_box(HOLE_X[1], KINK_X, -h, h),
        _level_box(HOLE_X[0], HOLE_X[1], RIM_Y[0], RIM_Y[1]),
        _level_box(HOLE_X[0], HOLE_X[1], -RIM_Y[1], -RIM_Y[0]),
        _level_box(0.0, HOLE_X[0], -h, h),
        ("box", (LIP_T / 2, 0.0, LEVEL_C - BLADE_T / 2 - LIP_H / 2), (LIP_T, WIDTH, LIP_H), None, STEEL),
    ]


def base_shapes() -> list[tuple]:
    """Every shape of the clipper's base body: the lower blade and its lip, the tail block, the upper blade."""
    return lower_shapes() + tail_shapes() + blade_shapes()


def tray_outer() -> tuple:
    """(centre, outer size, wall, floor) of the tray in its base frame (origin on the bench top), mm."""
    sx = SLOT_X[1] - SLOT_X[0] + 2 * WALL
    sy = SLOT_Y[1] - SLOT_Y[0] + 2 * WALL
    sz = TRAY_FLOOR + WALL_H
    return ((SLOT_X[0] + SLOT_X[1]) / 2, 0.0, sz / 2), (sx, sy, sz), WALL, TRAY_FLOOR


def masses_g() -> dict:
    """Steel (7.9 g/cm3) masses of the plates, for the README's check on the declared masses."""
    v = lambda a, b, c: 7.9 * a * b * c / 1000.0  # noqa: E731
    return {"lower blade": v(LEN, WIDTH, LOW_T), "lever": v(LEV - LEV_X0, LEV_W, LEV_T),
            "upper blade": v(BLADE_S[1] - BLADE_S[0] + KINK_X, WIDTH, BLADE_T)}


# -- the demo's paths ----------------------------------------------------------------------------------------------
PAD_C, PAD_W, PAD_H = 8.0, 13.5, 38.0  # closed gripper: half the 16 mm across the pads, half the 27 mm width, depth
TIP_PAST_TCP = 13.0  # the pad tips are this far past the TCP along the approach axis
PITCH_DEG = 35.0  # the flip's gripper pitch about the clipper's y
SWING_SIDE = 1  # +1: the pusher starts on the +y side and sweeps clockwise (the lever goes to psi = -pi)
SWING_R = 45.0  # the pusher's circle about the pin
SWING_TIP_Z = 17.0  # the pads' bottom above the slot floor
SWING_STEP_DEG = 15.0
SWING_CLEAR = 1.5  # the pusher starts this far from the lever's edge
SWING_PRESS = 0.5  # and ends this far into it
BACKOFF = 5.0
FLIP_RIDE = 15.0  # phase 1: the lever's tip rides this far above the fingertip edge
FLIP_PHASE1_END = 55.0
FLIP_R = 42.0  # phase 2: the fingertip edge rides this far from the hinge
FLIP_R_END = 33.0
FLIP_DRAW_DEG = 30.0  # ... drawing in to FLIP_R_END over the last this-many degrees
FLIP_STEP_DEG = 10.0
FLIP_LIFT = 0.5  # the last point: this far above the stowed lever's upper face
FLIP_FLATTEN_FROM = 145.0  # deg of lever: past this a pad pitched 35 deg would dip its inner corner into the lever


def d_vec(pitch_deg: float) -> np.ndarray:
    """The approach axis (TCP toward the pad tips) of the pitched grip, clipper frame: tips toward the tail and down."""
    p = math.radians(pitch_deg)
    return np.array([math.sin(p), 0.0, -math.cos(p)])


def n_vec(pitch_deg: float) -> np.ndarray:
    """The outer face's normal of the pad that faces the tail and upward."""
    p = math.radians(pitch_deg)
    return np.array([math.cos(p), 0.0, math.sin(p)])


def tcp_for_edge(edge, pitch_deg: float) -> np.ndarray:
    """The TCP (mm, clipper frame) that puts the outer pad's fingertip edge at ``edge`` (closed gripper)."""
    tips = np.asarray(edge, dtype=float) - PAD_C * n_vec(pitch_deg)
    return tips - TIP_PAST_TCP * d_vec(pitch_deg)


def swing_circle(side: int = SWING_SIDE) -> list[np.ndarray]:
    """Pad-centre points (mm, xy) of the swing: the circle of radius SWING_R about the pin, from beside the lever's
    long edge to the lever lying out past the jaws, squared against the 16 mm side face of the pads."""
    lat0 = LEV_W / 2 + PAD_W + SWING_CLEAR
    lat1 = LEV_W / 2 + PAD_W - SWING_PRESS
    phi0 = side * math.asin(lat0 / SWING_R)
    phi1 = -side * math.pi + side * math.asin(lat1 / SWING_R)
    n = max(1, int(math.ceil(abs(phi1 - phi0) / math.radians(SWING_STEP_DEG) - 1e-9)))
    pts = []
    for k in range(n + 1):
        phi = phi0 + (phi1 - phi0) * k / n
        pts.append(np.array([PIN_X + SWING_R * math.cos(phi), SWING_R * math.sin(phi)]))
    return pts


def lever_dir(a: float) -> np.ndarray:
    """Unit vector (x, z) hinge to tip, the lever past the jaws raised ``a`` rad from -x (a = eps at psi = pi)."""
    return np.array([-math.cos(a), math.sin(a)])


def heel_normal(a: float) -> np.ndarray:
    """Unit vector (x, z) of the heel-side face's normal for the same lever."""
    return np.array([-math.sin(a), -math.cos(a)])


def flip_pitch(a: float, pitch_deg: float = PITCH_DEG) -> float:
    """The gripper's pitch (deg) when the lever is at ``a`` rad. A pad pitched 35 deg keeps its fingertip edge E on the
    lever's heel-side face only while the pad's other tip corner (16 mm behind E, 9 mm lower) stays above that face:
    that holds while the lever is at 145 deg or less. Past that the pitch eases to 180 deg less the lever's angle
    (the tip face parallel to the face: flat on the stowed lever, 6 deg the other way)."""
    return min(pitch_deg, 180.0 - math.degrees(a)) if math.degrees(a) > FLIP_FLATTEN_FROM else pitch_deg


def flip_edges(pitch_deg: float = PITCH_DEG) -> list[tuple]:
    """Fingertip-edge points (x, z) in mm in the clipper's xz plane, with a label and the gripper's pitch: phase 1 (the
    lever's tip rides the pad face) then phase 2 (the edge rides the lever's heel-side face over the top and down)."""
    d = d_vec(pitch_deg)
    out = []
    a0, a1 = TH180, math.radians(FLIP_PHASE1_END)
    n1 = max(1, int(math.ceil((a1 - a0) / math.radians(FLIP_STEP_DEG) - 1e-9)))
    for k in range(n1 + 1):
        a = a0 + (a1 - a0) * k / n1
        t = np.array([PIN_X, ZH]) + LEV * lever_dir(a)
        out.append((t[0] + FLIP_RIDE * d[0], t[1] + FLIP_RIDE * d[2], f"phase 1, lever {math.degrees(a):.0f} deg",
                    pitch_deg))
    a2 = EPS_STOW
    n2 = max(1, int(math.ceil((a2 - a1) / math.radians(FLIP_STEP_DEG) - 1e-9)))
    for k in range(1, n2 + 1):
        a = a1 + (a2 - a1) * k / n2
        late = max(0.0, math.degrees(a - (a2 - math.radians(FLIP_DRAW_DEG)))) / FLIP_DRAW_DEG
        r = FLIP_R + (FLIP_R_END - FLIP_R) * min(1.0, late)
        lift = FLIP_LIFT * (1.0 if k == n2 else 0.0)
        e = np.array([PIN_X, ZH]) + r * lever_dir(a) + (LEV_T / 2 + lift) * heel_normal(a)
        out.append((e[0], e[1], f"phase 2, lever {math.degrees(a):.0f} deg", flip_pitch(a, pitch_deg)))
    return out
