"""On the station table, two blue plastic totes stand side by side, one in front of each arm, fixed in place.
Each holds an orange parts box on its floor. Inside each blue tote, two yellow ledge blocks stick out from each
long side wall, 15 mm below the rim. In front of each blue tote, nearer the robot, an empty grey tote stands on
the table, open side up. Each grey tote's long side walls have two full-height slots cut in them, their edges
marked dark, and its floor is notched under each slot. The grey totes are loose and light; they are too wide to grip across, but their
5 mm walls can be pinched.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLUE_C, LEDGE_C, GREY_C, ORANGE_C = (0.15, 0.33, 0.75), (0.95, 0.80, 0.15), (0.58, 0.60, 0.62), (0.92, 0.45, 0.10)

# Blue (full) totes: 140 (x) x 120 (y) x 60 mm outside, 5 mm walls and floor, so 130 x 110 inside.
BLUE_OUT = (0.140, 0.120, 0.060)
WALL = 0.005
BLUE_IN = (BLUE_OUT[0] - 2 * WALL, BLUE_OUT[1] - 2 * WALL)
LEDGE = (0.012, 0.010, 0.008)  # x along the wall, inward from the wall, height
LEDGE_TOP = 0.045  # above the table: 15 mm below the blue rim
LEDGE_X = (-0.045, 0.020)  # ledge centres along the tote, from its centre (not symmetric: a turn moves the slots)
BLUE_XY = {"left": (0.330, 0.130), "right": (0.330, -0.130)}


def _blue(side: str) -> parts.Part:
    hx, hy = BLUE_OUT[0] / 2, BLUE_OUT[1] / 2
    h = BLUE_OUT[2]
    shapes = [
        parts.box((BLUE_OUT[0], BLUE_OUT[1], WALL), at=(0.0, 0.0, WALL / 2), color=BLUE_C),
        parts.box((BLUE_OUT[0], WALL, h), at=(0.0, hy - WALL / 2, h / 2), color=BLUE_C),
        parts.box((BLUE_OUT[0], WALL, h), at=(0.0, -hy + WALL / 2, h / 2), color=BLUE_C),
        parts.box((WALL, BLUE_IN[1], h), at=(hx - WALL / 2, 0.0, h / 2), color=BLUE_C),
        parts.box((WALL, BLUE_IN[1], h), at=(-hx + WALL / 2, 0.0, h / 2), color=BLUE_C),
    ]
    for sy in (1, -1):
        for lx in LEDGE_X:
            shapes.append(parts.box(LEDGE, at=(lx, sy * (BLUE_IN[1] / 2 - LEDGE[1] / 2), LEDGE_TOP - LEDGE[2] / 2),
                                    color=LEDGE_C))
    return parts.Part(f"blue_{side}", pos=(*BLUE_XY[side], TOP), shapes=shapes)


# Orange parts box on each blue tote's floor: top 35 mm above the table.
PARTS_BOX = (0.110, 0.080, 0.030)
PARTS_TOP = WALL + PARTS_BOX[2]
PARTS = {side: Box(f"parts_{side}", pos=(*BLUE_XY[side], TOP + WALL + PARTS_BOX[2] / 2 + 0.0005), size=PARTS_BOX,
                   mass=0.2, color=ORANGE_C) for side in BLUE_XY}

# Grey (empty) totes: 122 x 102 x 55 mm outside, 5 mm walls, 4 mm clearance per side inside a blue tote. The body
# origin is the centre of the tote's underside. Each long wall has two full-height slots 22 mm wide, centred where
# the ledges are when the tote is turned the same way as the blue tote (yaw 0); the floor is notched 10 mm under
# each slot so the ledges pass.
GREY_OUT = (0.122, 0.102, 0.055)
SLOT_W = 0.022
NOTCH = 0.010
GREY_MASS = 0.12
EDGE_C, EDGE_W = (0.12, 0.13, 0.15), 0.004  # dark strips marking each slot's edges


def _grey_shapes():
    hx, hy, h = GREY_OUT[0] / 2, GREY_OUT[1] / 2, GREY_OUT[2]
    ix = hx - WALL  # inner half length, 0.056
    # Wall segments along x between the slots: from the inner end face to the first slot, between, after.
    cuts = sorted((x - SLOT_W / 2, x + SLOT_W / 2) for x in LEDGE_X)
    segs, x0 = [], -ix
    for a, b in cuts:
        if a - x0 > 1e-6:
            segs.append((x0, a))
        x0 = b
    if ix - x0 > 1e-6:
        segs.append((x0, ix))
    shapes = [parts.box((2 * ix, 2 * (hy - WALL - NOTCH), WALL), at=(0.0, 0.0, WALL / 2), color=GREY_C)]
    # End walls. Where a slot runs into an end wall, the wall's two corner posts (the slot's far edge) are dark.
    for sx in (1, -1):
        touched = any(abs(e - sx * ix) < 1e-6 for c in cuts for e in c)
        x = sx * (hx - WALL / 2)
        shapes.append(parts.box((WALL, 2 * (hy - WALL), h), at=(x, 0.0, h / 2), color=GREY_C))
        for sy in (1, -1):
            shapes.append(parts.box((WALL, WALL, h), at=(x, sy * (hy - WALL / 2), h / 2),
                                    color=EDGE_C if touched else GREY_C))
    # Side-wall segments, each with a dark 4 mm strip at every end that borders a slot.
    edges = {e for c in cuts for e in c}
    for a, b in segs:
        pieces = []
        lo = a + EDGE_W if any(abs(a - e) < 1e-6 for e in edges) else a
        hi = b - EDGE_W if any(abs(b - e) < 1e-6 for e in edges) else b
        if lo > a:
            pieces.append((a, lo, EDGE_C))
        pieces.append((lo, hi, GREY_C))
        if hi < b:
            pieces.append((hi, b, EDGE_C))
        for sy in (1, -1):
            for p0, p1, col in pieces:
                shapes.append(parts.box((p1 - p0, WALL, h), at=((p0 + p1) / 2, sy * (hy - WALL / 2), h / 2), color=col))
            shapes.append(parts.box((b - a, NOTCH, WALL), at=((a + b) / 2, sy * (hy - WALL - NOTCH / 2), WALL / 2),
                                    color=GREY_C))
    return shapes


GREY_START = {"left": ((0.190, 0.130), 0.0), "right": ((0.190, -0.130), math.pi)}  # (xy, yaw)
GREY = {side: parts.Part(f"grey_{side}", pos=(*xy, TOP + 0.0005), yaw=yaw, mass=GREY_MASS, shapes=_grey_shapes())
        for side, (xy, yaw) in GREY_START.items()}

OBJECTS = [_blue("left"), _blue("right"), *PARTS.values(), *GREY.values()]

STACK_RIM = LEDGE_TOP + GREY_OUT[2]  # 0.100: rim height of a grey tote resting on the ledges
RIM_TOL = 0.0015  # every rim corner within 1.5 mm of that height
XY_TOL = 0.006  # grey tote's centre within 6 mm of the blue tote's centre (the walls allow 4 mm)
PARTS_TOL = 0.005  # parts box moved at most 5 mm
PRESS_Z = LEDGE_TOP - 0.002  # a grey tote's underside below 43 mm inside a blue tote: it went down past the ledges
RESET_TOL, RESET_JUMP = 0.0005, 0.020
_RIM = np.array([[sx * GREY_OUT[0] / 2, sy * GREY_OUT[1] / 2, GREY_OUT[2]] for sx in (1, -1) for sy in (1, -1)])


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rim_heights(st) -> np.ndarray:
    """Heights above the table of the grey tote's four outer rim corners, from its pose and true dimensions."""
    return (np.asarray(st.pos, float)[:, None] + _rot(st.quat_xyzw) @ _RIM.T)[2] - TOP


def _over(st, side: str) -> bool:
    """The grey tote's centre is over the blue tote's inside."""
    d = np.asarray(st.pos[:2], float) - BLUE_XY[side]
    return bool(abs(d[0]) < BLUE_IN[0] / 2 and abs(d[1]) < BLUE_IN[1] / 2)


def stacked(st, side: str) -> bool:
    """Resting on the blue tote's ledges: every rim corner 100 mm +/- 1.5 mm above the table, centred within 6 mm,
    and at rest."""
    d = np.asarray(st.pos[:2], float) - BLUE_XY[side]
    rims = rim_heights(st)
    return bool(
        np.all(np.abs(rims - STACK_RIM) <= RIM_TOL + 1e-6)
        and np.all(np.abs(d) <= XY_TOL + 1e-6)
        and st.resting()
    )


def _parts_ok(objects, side: str) -> bool:
    st = objects[f"parts_{side}"]
    return bool(np.linalg.norm(np.asarray(st.pos) - PARTS[side].pos) <= PARTS_TOL + 1e-6)


# Trial history, per blue tote: whether a grey tote has ever gone down past its ledges (toward its parts box), and
# whether its parts box has ever been more than 5 mm from its start. The scorer sees one step at a time, so both are
# kept here. They are cleared only at a new trial: every loose object back within 0.5 mm of its start and at rest,
# having got there by a jump of more than 20 mm in one step (a scene reset, not reachable by the robot).
_START = {o.name: np.asarray(o.pos, float) for o in OBJECTS if o.free}
_HISTORY = {"pressed": {"left": False, "right": False}, "moved": {"left": False, "right": False}, "prev": None}


def _update(objects) -> None:
    now = {n: np.asarray(objects[n].pos, float) for n in _START}
    home = all(np.linalg.norm(now[n] - p) < RESET_TOL and objects[n].resting() for n, p in _START.items())
    prev = _HISTORY["prev"]
    jumped = prev is None or any(np.linalg.norm(now[n] - prev[n]) > RESET_JUMP for n in _START)
    if home and jumped:
        _HISTORY["pressed"] = {"left": False, "right": False}
        _HISTORY["moved"] = {"left": False, "right": False}
    _HISTORY["prev"] = now
    for side in BLUE_XY:
        if not _parts_ok(objects, side):
            _HISTORY["moved"][side] = True
        for g in GREY:
            st = objects[f"grey_{g}"]
            if _over(st, side) and st.low[2] - TOP < PRESS_Z - 1e-6:
                _HISTORY["pressed"][side] = True


def pair_ok(objects, side: str) -> bool:
    """This blue tote qualifies: a grey tote rests on its ledges, and during the trial its parts box never moved more
    than 5 mm and no grey tote went down past its ledges."""
    if _HISTORY["pressed"][side] or _HISTORY["moved"][side] or not _parts_ok(objects, side):
        return False
    return any(stacked(objects[f"grey_{g}"], side) for g in GREY)


def score(objects):
    """0.5 for each blue tote that qualifies (see pair_ok), so 1.0 when both do. Each blue tote is scored on its
    own: a press or a moved parts box in one costs only that tote's 0.5."""
    _update(objects)
    return 0.5 * pair_ok(objects, "left") + 0.5 * pair_ok(objects, "right")


def success(objects):
    return score(objects) == 1.0


# -- scripted demo --------------------------------------------------------------------------------------------------
# Each arm pinches one end wall of its grey tote from above (fingers closing along x). The left tote starts turned
# the same way as its blue tote, so its slots line up with the ledges and it would nest onto the parts box; the left
# arm turns it end for end in the air, in two quarter turns, before lowering it. The right tote already has solid
# wall over the ledges and is carried straight across.
END_X = GREY_OUT[0] / 2 - WALL / 2  # 0.0585: end wall centre from the tote centre
GRIP_BELOW_RIM = 0.007  # TCP 7 mm below the wall top: fingertips 20 mm down the wall
TIP = 0.013
CARRY_BOTTOM = 0.085  # tote underside 25 mm above the blue rim while carried and turned
RELEASE_DROP = 0.007  # released with the underside 7 mm above the ledges
# Tracking error while carrying, cancelled by aiming off. Left arm: about 2.4 mm to the robot's left (+y) and 4 mm
# low (measured on earlier tasks; the tote landed within 0.5 mm of centre here). Right arm, holding the tote by its
# near end wall out at x 0.27 m: about 2 mm to the robot's right and 4.5 mm short in x (first GPU trace).
SAG_X = {"left": 0.0, "right": 0.0045}
SAG_Y = {"left": 0.0024, "right": -0.0021}
SAG_Z = 0.004


def _tcp_z(bottom: float) -> float:
    return TOP + bottom + GREY_OUT[2] - GRIP_BELOW_RIM


def _plan(side: str, grip_sign: float, turn: bool):
    """grip_sign: +1 grips the tote's far (+x) end wall, -1 its near one, in world terms at the start."""
    (sx, sy), _ = GREY_START[side]
    bx, by = BLUE_XY[side]
    gx = sx + grip_sign * END_X
    # After a half turn the gripped wall is on the other side of the tote centre.
    end_sign = -grip_sign if turn else grip_sign
    fx, fy = bx + end_sign * END_X + SAG_X[side], by - SAG_Y[side]
    # Left arm: a fingers-along-x grip at yaw -90 degrees near the robot, +90 degrees far out (both within the
    # wrist's range there), so the half turn goes from -90 to +90 degrees while the tote is carried out.
    y0 = -math.pi / 2 if turn else math.pi / 2
    y1 = y0 + math.pi if turn else y0
    carry, low = _tcp_z(CARRY_BOTTOM) + SAG_Z, _tcp_z(LEDGE_TOP + RELEASE_DROP) + SAG_Z
    grip_z = _tcp_z(0.0)
    w = [
        Waypoint(1.5, (gx, sy, grip_z + 0.06), FINGER_OPEN, "over the end wall", y0),
        Waypoint(1.2, (gx, sy, grip_z), FINGER_OPEN, "descend", y0),
        Waypoint(0.8, (gx, sy, grip_z), FINGER_CLOSED, "pinch", y0),
        Waypoint(1.5, (gx, sy, carry), FINGER_CLOSED, "lift", y0),
    ]
    if turn:
        mx = (gx + fx) / 2
        w += [
            Waypoint(1.5, (mx, sy, carry), FINGER_CLOSED, "quarter turn", y0 + math.pi / 2),
            Waypoint(1.5, (fx, fy, carry), FINGER_CLOSED, "half turn", y1),
        ]
    else:
        w.append(Waypoint(1.8, (fx, fy, carry), FINGER_CLOSED, "carry", y1))
    w += [
        Waypoint(0.8, (fx, fy, carry), FINGER_CLOSED, "settle", y1),
        Waypoint(1.5, (fx, fy, low), FINGER_CLOSED, "lower into the mouth", y1),
        Waypoint(0.8, (fx, fy, low), FINGER_CLOSED, "steady", y1),
        Waypoint(0.6, (fx, fy, low), FINGER_OPEN, "release", y1),
        Waypoint(1.0, (fx, fy, carry), FINGER_OPEN, "up", y1),
    ]
    return w


def demo():
    return {"left": _plan("left", -1.0, True), "right": _plan("right", -1.0, False)}
