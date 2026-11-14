"""On the station table: a white desktop bookshelf (a base, two end panels and a back panel, open above) with nine
cloth-bound hardcover books standing in a packed row, spines toward the robot, 0.4 mm apart. The red book, fifth
from the robot's left, is shallower than its neighbours, so its spine sits 12 and 14 mm behind the black and mustard
spines beside it and no finger fits at its sides; its head stands 8 and 6 mm above theirs. In front of the shelf lies
a brown cork desk mat. The books are loose; the shelf and the mat are fixed.

Requires -E object_jitter=0: the row is packed to 0.4 mm and the scorer measures from the starting layout.

Run it:  inspect-robots run --embodiment yam_arena -E task=tip_a_recessed_book_off_a_packed_shelf.py \\
           -E object_jitter=0 --instruction "Take the red book off the shelf and put it on the cork mat. Leave the
           other books standing on the shelf." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import mechkit  # noqa: E402
from belimits import Rect, inside, tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Mech, box, extrude, on_reset, released  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to open, back away and park after the first 1.0

WHITE, CORK, CREAM = (0.95, 0.95, 0.93), (0.42, 0.27, 0.15), (0.96, 0.93, 0.82)
GILT, INK = (0.86, 0.70, 0.25), (0.10, 0.08, 0.06)

# ---- the desktop bookshelf (fixed: clamped to the desk) ---------------------------------------------------------------
SHELF_X0 = 0.38  # front edge of the base
BASE_D, BASE_W, BASE_T = 0.200, 0.323, 0.015
PANEL_T, PANEL_H = 0.015, 0.250
INNER = 0.1465  # inner faces of the end panels at y +-0.1465: 293 mm between them
BACK_T = 0.006
BACK_X = SHELF_X0 + BASE_D  # front face of the back panel
SHELF = Mech(
    "bookshelf", pos=(SHELF_X0, 0.0, TOP), mass=None, friction=0.3,
    shapes=[
        box((BASE_D / 2, 0, BASE_T / 2), (BASE_D, BASE_W, BASE_T), WHITE),
        box((BASE_D / 2, INNER + PANEL_T / 2, BASE_T + PANEL_H / 2), (BASE_D, PANEL_T, PANEL_H), WHITE),
        box((BASE_D / 2, -INNER - PANEL_T / 2, BASE_T + PANEL_H / 2), (BASE_D, PANEL_T, PANEL_H), WHITE),
        box((BASE_D + BACK_T / 2, 0, (BASE_T + PANEL_H) / 2), (BACK_T, BASE_W, BASE_T + PANEL_H), WHITE),
    ],
)

# ---- the cork desk mat (fixed), abutting the shelf base ---------------------------------------------------------------
MAT_X, MAT_Y, MAT_T = (SHELF_X0 - 0.200, SHELF_X0), (-0.150, 0.150), 0.003
MAT = Mech(
    "cork_mat", pos=((MAT_X[0] + MAT_X[1]) / 2, 0.0, TOP), mass=None, friction=0.6,
    shapes=[box((0, 0, MAT_T / 2), (MAT_X[1] - MAT_X[0], MAT_Y[1] - MAT_Y[0], MAT_T), CORK)],
)

# ---- the books (free) -------------------------------------------------------------------------------------------------
# A book's frame: origin at the foot of its spine (mid thickness), x into the shelf (depth), y across its thickness,
# z up its height. A case of two 2.5 mm boards and a rounded spine piece 6 mm deep, and a page block that stops 3 mm
# short of the boards at the head, the tail and the fore-edge (the squares). So the head has a real recess, 3 mm
# deep, between the boards and behind the spine: a fingertip set in it hooks the spine's inner face.
BOARD, SQUARE, SPINE_D, SPINE_SAG = 0.0025, 0.003, 0.006, 0.0035
DENSITY = 600.0  # kg/m3, over the book's envelope
GAP = 0.0004  # between neighbours, and at both end panels and the back panel
BOOK_MU = 0.4  # cloth; the simulator averages it with the shelf's 0.3 (0.35) and the mat's 0.6 (0.5)

# name, thickness, depth, height (m), cloth colour, from the robot's left (+y) to its right (-y)
ROW = [
    ("navy_book", 0.028, 0.178, 0.235, (0.07, 0.11, 0.36)),
    ("green_book", 0.035, 0.176, 0.238, (0.05, 0.30, 0.13)),
    ("grey_book", 0.024, 0.170, 0.228, (0.52, 0.52, 0.54)),
    ("black_book", 0.040, 0.172, 0.232, (0.06, 0.06, 0.07)),
    ("red_book", 0.032, 0.160, 0.240, (0.92, 0.04, 0.04)),
    ("mustard_book", 0.030, 0.174, 0.234, (0.86, 0.64, 0.08)),
    ("brown_book", 0.038, 0.168, 0.230, (0.38, 0.20, 0.09)),
    ("teal_book", 0.026, 0.180, 0.236, (0.04, 0.48, 0.48)),
    ("slate_book", 0.036, 0.172, 0.228, (0.33, 0.43, 0.66)),
]


def _book(name, t, d, h, color, y):
    half = t / 2
    # The rounded spine as an eight-sided arc: a 0.375 t flat at the front, then two facets a side falling 3.5 mm.
    arc = [(SPINE_D, -half), (SPINE_SAG, -half), (0.0012, -0.7 * half), (0.0, -0.375 * half),
           (0.0, 0.375 * half), (0.0012, 0.7 * half), (SPINE_SAG, half), (SPINE_D, half)]
    board_d = d - SPINE_D
    band = INK if sum(color) > 1.5 else GILT
    look = [  # look only: a title panel, a rule and a head band on the spine
        box((0.0003, 0, 0.74 * h), (0.0012, 0.70 * half, 0.034), band),
        box((0.0003, 0, 0.14 * h), (0.0012, 0.70 * half, 0.005), band),
        box((0.0003, 0, h - 0.006), (0.0012, 0.70 * half, 0.004), band),
    ]
    return Mech(
        name, pos=(BACK_X - GAP - d, y, TOP + BASE_T), mass=round(DENSITY * t * d * h, 3), friction=BOOK_MU,
        shapes=[
            extrude((0, 0, h / 2), arc, h, color, axis="z"),
            box((SPINE_D + board_d / 2, half - BOARD / 2, h / 2), (board_d, BOARD, h), color),
            box((SPINE_D + board_d / 2, -half + BOARD / 2, h / 2), (board_d, BOARD, h), color),
            box((SPINE_D + (board_d - SQUARE) / 2, 0, h / 2), (board_d - SQUARE, t - 2 * BOARD, h - 2 * SQUARE), CREAM),
            *[dataclasses.replace(s, collide=False) for s in look],
        ],
    )


BOOKS, DIMS = [], {}
_y = INNER - GAP
for _name, _t, _d, _h, _c in ROW:
    BOOKS.append(_book(_name, _t, _d, _h, _c, _y - _t / 2))
    DIMS[_name] = (_t, _d, _h)
    _y -= _t + GAP
RED = next(b for b in BOOKS if b.name == "red_book")
OTHERS = [b for b in BOOKS if b is not RED]
RED_T, RED_D, RED_H = DIMS["red_book"]
RED_X, RED_Y = RED.pos[0], RED.pos[1]  # the red spine's foot: x 0.4196, y 0.0015

OBJECTS = [SHELF, MAT, *BOOKS]

# ---- scoring ----------------------------------------------------------------------------------------------------------
TIP_OUT = 0.040  # m: T, the red book's head front edge this far in front of its start
V_TILT, V_SLIDE, V_DROP = 35.0, 0.030, 0.005  # a neighbour toppled, pulled out, or off the shelf base
DROP_SPEED = 0.6  # m/s: D, any corner of the red book faster than this
REST_SPEED, REST_STEPS, STEP_HZ = 0.01, 20, 20.0  # under 1 cm/s for 1 s at 20 Hz: 0.5 mm or less per control step
RELEASE_CLEAR = 0.03
H_TILT = 10.0  # deg from upright on the tail, or from flat on a cover
ON_GAP = 0.005  # m: lowest point within this of the mat top
DESK_GAP = 0.010  # m: lowest point within this of the mat top or the desk
MAT_RECT = Rect(MAT_X, MAT_Y)
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it for a held book)

_L = {"tipped": False, "v": False, "dropped": False, "still": 0, "seen": None, "last": None}


@on_reset
def _clear(_env=None):
    _L.update(tipped=False, v=False, dropped=False, still=0, seen=None, last=None)


def envelope(state, name) -> np.ndarray:
    """The eight corners of a book's envelope (thickness x depth x height) in the world."""
    t, d, h = DIMS[name]
    return np.array([state.to_world((x, y, z)) for x in (0.0, d) for y in (-t / 2, t / 2) for z in (0.0, h)])


def lean_deg(state) -> float:
    """How far the book's head leans toward the robot (about its foot edge), in degrees."""
    z = mechkit._quat_xyzw_matrix(state.quat_xyzw)[:, 2]
    return math.degrees(math.atan2(-z[0], z[2]))


def _fingers_clear(red) -> bool:
    """Both grippers at least 3 cm from the red book. Only the simulator can tell; offline replays have no fingers to
    measure, so they use OFFLINE_FINGERS_CLEAR."""
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return released(red, RELEASE_CLEAR)


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    red = objects["red_book"]
    if red.to_world((0.0, 0.0, RED_H))[0] <= RED_X - TIP_OUT:
        _L["tipped"] = True
    for b in OTHERS:
        s = objects[b.name]
        if (tilt_deg(s) > V_TILT or s.to_world((0.0, 0.0, 0.0))[0] <= b.pos[0] - V_SLIDE
                or envelope(s, b.name)[:, 2].min() < TOP + BASE_T - V_DROP):
            _L["v"] = True
    # Speeds are what two video frames a control step apart show: the distance each corner moved, times 20.
    here, last = envelope(red, "red_book"), _L["last"]
    _L["last"] = here
    if last is None:
        _L["still"] = 0
        return
    step = float(np.linalg.norm(here - last, axis=1).max())
    if step * STEP_HZ > DROP_SPEED:
        _L["dropped"] = True
    _L["still"] = _L["still"] + 1 if step * STEP_HZ < REST_SPEED else 0


def on_mat(red) -> bool:
    """H: resting on the cork mat on its tail (within 10 degrees of upright) or on a cover (within 10 degrees of
    flat), every point of its outline over the mat."""
    rot = mechkit._quat_xyzw_matrix(red.quat_xyzw)
    low = float(envelope(red, "red_book")[:, 2].min())
    on_tail = tilt_deg(red) <= H_TILT
    on_cover = math.degrees(math.acos(min(1.0, abs(float(rot[2, 1]))))) <= H_TILT
    return bool((on_tail or on_cover) and abs(low - (TOP + MAT_T)) <= ON_GAP and inside(red, RED, MAT_RECT))


def off_shelf(red) -> bool:
    """Lying or standing on the desk or the mat with no part of it over the shelf."""
    pts = envelope(red, "red_book")
    return bool(pts[:, 0].max() < SHELF_X0 and TOP - 0.005 <= pts[:, 2].min() <= TOP + MAT_T + DESK_GAP)


def score(objects):
    """1.0: the red book released and at rest on the cork mat on its tail or a cover, no neighbour disturbed, never
    dropped. 0.5: on the mat after a neighbour was disturbed or the book was dropped, or at rest on the desk clear of
    the shelf with no neighbour disturbed. 0.25: its head was tipped 40 mm out. Else 0."""
    _update(objects)
    red = objects["red_book"]
    ended = _L["still"] >= REST_STEPS and _fingers_clear(red)
    if ended and on_mat(red):
        return 0.5 if (_L["v"] or _L["dropped"]) else 1.0
    if ended and off_shelf(red) and not _L["v"]:
        return 0.5
    return 0.25 if _L["tipped"] else 0.0


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps off the other books ------------------------------------------------------------------------------
# Over the eight other books, outside the red book's own slot (its thickness widened by 12 mm a side), no link origin
# and no point tracked along the links of either arm comes below 26.5 cm above the table, 1.2 cm over the tallest
# head: a hand reaching for one book does not press on or hook its neighbours. (The gate's envelope holds the two tool
# points to 28 cm there.)
SLOT_Y = (RED_Y - RED_T / 2 - 0.012, RED_Y + RED_T / 2 + 0.012)
KEEP_OFF_X, KEEP_OFF_Z = (SHELF_X0, BACK_X + BACK_T), 0.265
ARM_KEEP_OUT = [
    {"name": "books left of the red book", "x": KEEP_OFF_X, "y": (SLOT_Y[1], BASE_W / 2), "z_top": TOP + KEEP_OFF_Z},
    {"name": "books right of the red book", "x": KEEP_OFF_X, "y": (-BASE_W / 2, SLOT_Y[0]), "z_top": TOP + KEEP_OFF_Z},
]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
PIVOT = np.array([RED_X, RED_Y, TOP + BASE_T])  # the red book's foot edge at the start
# The right gripper works leaned 40 degrees to its own side and 20 degrees forward, jaws along the shelf's depth (a
# straight-down gripper cannot reach 26 cm). So leaned, the closed fingertips' lowest corner is on the finger nearer
# the robot, and that finger's outer face rises from the corner within 0.2 mm of vertical for 6 mm: set in the head
# recess, it meets the spine's inner face square, as a fingertip hooked behind the top of a spine does.
TIP_PITCH, TIP_SIDE = 0.35, 0.70  # rad
# The lowest corner of the closed fingertips from the tool point at that lean (from the gripper's collision hull), and
# how far the hooking face stands in front of that corner.
TIP_CORNER = np.array([0.0018, 0.0025, -0.0137])
HOOK_FACE = 0.0018
HOOK_BEHIND, HOOK_Y = 0.009, 0.003  # the corner lands this far behind the spine's inner face, this far left of centre
PRESS_FROM, PRESS_EXTRA, PRESS_MORE = 0.020, 0.008, 0.005  # hover height, commanded press past contact, more per retry
SEEK_SPEED, SEEK_PAST = 0.006, 0.004  # m/s toward the robot until the book stirs, at most this far past the lip
PRESS_KEEP, PRESS_STEP, PRESS_CAP = 0.006, 0.0004, 0.020  # hold the command this far under the measured fingertip
STALL_S, TIP_TRIES = 2.5, 3  # no gain in lean for this long with the fingertip leading: it slipped; land again
TIP_STOP, TIP_MAX, TIP_RATE, TIP_LEAD = 22.0, 27.0, 4.0, 7.0  # deg, deg, deg/s, deg
PINCH_Z = 0.170  # m above the table: where the left pads take the spine
PINCH_OPEN, PINCH_CLOSE, PINCH_FELT, RELEASE_OPEN = 0.075, 0.024, 0.028, 0.090  # pad gaps (m); the book is 32 mm
CARRIAGE_BACK, CARRIAGE_HALF = 0.0177, 0.034  # the finger carriages start 17.7 mm behind the tool point, 68 mm tall
TURN_X, STAND_X = 0.275, 0.200  # the foot edge's x where the book is stood up to 9 degrees, and where it ends
STAND_TOL, NUDGE_SPEED, NUDGE_S = 0.004, 0.008, 6.0  # m, m/s, s: the standing book is slid to within 4 mm of its place
SLIDE_LEAN, TURN_RATE = 9.0, 3.0  # deg, deg/s
PINCH_LIGHT = 0.030  # m: the loosened pinch (1 mm of squeeze a side), which lets the book turn between the pads
DRAW_SPEED, STEP_SPEED, SLIDE_SPEED = 0.02, 0.008, 0.015  # m/s: the draw out, the step down, the last slide
REACH_MIN_X = 0.183  # the level left gripper reaches no nearer the robot than x 0.18
RISE_Z = 0.290  # m above the table: the open left gripper rises here, 3 cm clear of the standing book's head


def _tip_rot() -> np.ndarray:
    """The right gripper's orientation for the tip: grip_rot pitched forward, jaws along x, then leaned to -y."""
    c, sn = math.cos(TIP_SIDE), math.sin(TIP_SIDE)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -sn], [0.0, sn, c]]) @ grip_rot(pitch=TIP_PITCH, roll=-math.pi / 2)


def _finger(gap: float) -> float:
    """Finger joint target for a pad gap in metres (94 mm is fully open)."""
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def _lean_rot(deg: float) -> np.ndarray:
    """Turns a vector from the foot edge by ``deg`` toward the robot (head toward -x)."""
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, -math.sin(a)], [0.0, 1.0, 0.0], [math.sin(a), 0.0, math.cos(a)]])


def _pinch_inset(lean: float) -> float:
    """How far inside the spine's front the tool point may sit before the finger carriages touch a spine leaning
    ``lean`` degrees toward a level gripper, less 1.5 mm."""
    return float(np.clip(CARRIAGE_BACK - CARRIAGE_HALF * math.tan(math.radians(max(lean, 0.0))) - 0.0015, 0.0, 0.012))


def _red(ctx, nominal):
    """(foot of the spine in the world, lean in degrees) of the red book: from the simulator in a live run; offline,
    where nothing moves, the ``nominal`` pose the step expects."""
    s = ctx.obj("red_book")
    if ctx.left.offline:
        return np.asarray(nominal[0], dtype=float), float(nominal[1])
    return np.asarray(s.to_world((0.0, 0.0, 0.0)), dtype=float), lean_deg(s)


class _Lag:
    """The command offset that cancels the lag the joint encoders report (the load makes the wrist sag), filtered."""

    def __init__(self, arm, gain=0.2, limit=0.03):
        self.arm, self.gain, self.limit = arm, gain, limit
        self.off = arm.pos - np.asarray(arm.measured_pos, dtype=float)

    def to(self, want, rot):
        """One control step toward having the measured tool point at ``want``."""
        arm = self.arm
        off = self.off + self.gain * (arm.pos - np.asarray(arm.measured_pos, dtype=float) - self.off)
        n = float(np.linalg.norm(off))
        self.off = off if n <= self.limit else off * (self.limit / n)
        yield from arm.move(np.asarray(want, dtype=float) + self.off, arm.dt, rot=rot)


def _line(arm, start, goal, speed, rot):
    """Move the measured tool point in a straight line from ``start`` to ``goal`` at ``speed`` (m/s)."""
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    lag = _Lag(arm)
    steps = max(1, round(float(np.linalg.norm(goal - start)) / speed / arm.dt))
    for k in range(1, steps + 1):
        yield from lag.to(start + (goal - start) * k / steps, rot)


demo = placeholder_demo


def demo_script(ctx):
    """Right fingertip into the recess at the red book's head, hook the spine's inner face, tip the book out about its
    foot edge to 22 degrees;
    left hand pinches the spine that now stands proud; right hand lets go and parks; left hand draws the book out over
    the base edge and down onto the mat, stands it up in two turns with a slide between, lets go and parks."""
    R, L = ctx.right, ctx.left
    hook, level = _tip_rot(), grip_rot(pitch=math.pi / 2)
    base_top, mat_top = TOP + BASE_T, TOP + MAT_T

    # 1) tip: the closed right fingertip goes down into the head recess 9 mm behind the spine's inner face, is drawn
    # toward the robot until it meets that face, and then leads the book round its foot edge
    floor_z = base_top + RED_H - SQUARE  # the page block's head
    corner = PIVOT + np.array([SPINE_D + HOOK_BEHIND, HOOK_Y, RED_H - SQUARE])
    touch = corner - TIP_CORNER  # tool point with the fingertip's lowest corner there
    hover = touch + np.array([0.0, 0.0, PRESS_FROM])
    cmd0 = hover.copy()

    def tip(extra):
        """Land in the recess, press ``extra`` past contact, find the lip and tip the book. True at 22 degrees."""
        nonlocal cmd0
        yield from R.move(hover, 2.0, rot=hook)
        yield from R.trim_to_measured(hover)
        yield from R.press_until(R.pos.copy(), max_depth=PRESS_FROM + 0.004, err_mm=4.0, min_depth=PRESS_FROM - 0.004,
                                 rot=hook)
        for _ in range(4):  # the encoders say whether the corner is down on the page block; if not, go on down
            high = float(R.measured_pos[2] + TIP_CORNER[2] - floor_z)
            if R.offline or high < 0.002:
                break
            yield from R.move(R.pos + np.array([0.0, 0.0, -high]), 0.4, rot=hook)
        yield from R.move(R.pos + np.array([0.0, 0.0, -extra]), 0.4, rot=hook)
        sink = 0.0

        def bear():
            """Pulling lifts the arm a little. Keep the commanded height 6 mm under the height the encoders report,
            so the fingertip keeps bearing down on the page block: lower the command 0.4 mm a step while it is not."""
            nonlocal sink
            if not R.offline and float(R.measured_pos[2] - R.pos[2]) < PRESS_KEEP and sink < PRESS_CAP:
                sink += PRESS_STEP
                return np.array([0.0, 0.0, -PRESS_STEP])
            return np.zeros(3)

        reach = HOOK_BEHIND - HOOK_FACE + SEEK_PAST
        for k in range(1, round(reach / SEEK_SPEED / R.dt) + 1):
            if not R.offline and _red(ctx, (PIVOT, 0.0))[1] >= 0.5:
                break
            yield from R.move(R.pos + np.array([-SEEK_SPEED * R.dt, 0.0, 0.0]) + bear(), R.dt, rot=hook)
        cmd0, arm0 = R.pos.copy(), np.asarray(R.measured_pos, dtype=float) + TIP_CORNER - PIVOT
        # servo the fingertip along the arc about the foot edge, leading the book's measured lean, to 22 degrees
        phi, best, since = 0.0, 0.0, 0.0
        for _ in range(round(14.0 / R.dt)):
            _, lean = _red(ctx, (PIVOT, max(0.0, phi - 3.0)))
            if lean >= TIP_STOP:
                return True
            if lean > best + 0.3:
                best, since = lean, 0.0
            since += R.dt
            if since > STALL_S and phi >= min(lean + TIP_LEAD, TIP_MAX) - 0.01:
                return False
            phi = min(phi + TIP_RATE * R.dt, lean + TIP_LEAD, TIP_MAX)
            cmd0 = cmd0 + bear()
            yield from R.move(cmd0 + (_lean_rot(phi) - np.eye(3)) @ arm0, R.dt, rot=hook)
        return _red(ctx, (PIVOT, TIP_STOP))[1] >= TIP_STOP - 4.0

    yield from R.move((SHELF_X0 - 0.08, RED_Y, hover[2] + 0.02), 3.5, FINGER_CLOSED, rot=hook)
    tipped = False
    for attempt in range(TIP_TRIES):
        tipped = yield from tip(PRESS_EXTRA + attempt * PRESS_MORE)
        if tipped or _red(ctx, (PIVOT, 0.0))[1] > 3.0:
            break
        yield from R.move(R.pos + np.array([0.004, 0.0, 0.03]), 1.0, rot=hook)  # slipped: up, and land again
    if not tipped:  # the book would not come: lift off and park
        yield from R.move(R.pos + np.array([0.004, 0.0, 0.05]), 1.5, rot=hook)
        yield from R.home(3.5)
        return
    yield from R.hold(0.6)

    # 2) take: the left gripper, level, jaws closing across the book, comes in from the front onto the proud spine
    foot, lean = _red(ctx, (PIVOT, TIP_STOP))
    up = (TOP + PINCH_Z - foot[2]) / math.cos(math.radians(lean))  # how far up the spine the pads take it
    spine = foot + _lean_rot(lean) @ np.array([0.0, 0.0, up])
    pinch = spine + np.array([_pinch_inset(lean), 0.0, 0.0])
    before = pinch - np.array([0.06, 0.0, 0.0])
    yield from L.move((before[0] - 0.04, 0.16, TOP + PINCH_Z), 3.5, _finger(PINCH_OPEN), rot=level)
    yield from L.move(before, 2.0, rot=level)
    yield from L.trim_to_measured(before)
    free_off = L.pos - np.asarray(L.measured_pos, dtype=float)  # the arm's own sag correction, with nothing in hand
    yield from _line(L, before, pinch, 0.03, level)
    yield from L.trim_to_measured(pinch, tol=0.0015)
    yield from L.grip(_finger(PINCH_CLOSE), 1.0)
    yield from L.hold(0.3)
    if not L.holding(PINCH_FELT):  # a miss: once more, 10 mm lower
        yield from L.grip(_finger(PINCH_OPEN), 0.5)
        yield from L.move(L.pos - np.array([0.0, 0.0, 0.010]), 0.6, rot=level)
        yield from L.grip(_finger(PINCH_CLOSE), 1.0)
        yield from L.hold(0.3)
        if not L.holding(PINCH_FELT):  # still nothing: let the book back and park both arms
            yield from L.grip(_finger(RELEASE_OPEN), 0.5)
            yield from L.move(before, 1.5, rot=level)
            yield from R.move(cmd0, 3.0, rot=hook)
            yield from R.move(hover, 1.0, rot=hook)
            yield from ctx.both(L.home(), R.home())
            return

    # 3) free: the right fingertip lifts straight up off the leaning head and parks
    yield from R.move(R.pos + np.array([0.006, 0.0, 0.06]), 1.5, rot=hook)
    yield from R.home(3.5)

    # From here the left hand moves the pinched point of the spine so that the book's foot edge is at x ``fx`` and
    # height ``fz`` with the book leaning ``ln`` degrees toward the robot. While it draws the book along, it holds
    # the hand at exactly the height that lean needs with the foot edge on its surface, so the book trails on its foot
    # edge whether the pads hold it like a clamp or like a hinge pin. To change the lean it loosens the pinch, so the
    # book turns between the pads about its planted foot edge. The pads keep one pitch throughout.
    y0, lean0 = pinch[1], lean
    held = _lean_rot(-lean0) @ (pinch - foot)  # the pinched point in the book's own frame
    held[1] = 0.0
    lag = _Lag(L)
    fx = float(foot[0])

    def read(nominal_ln):
        """(foot edge, lean, the pinched point in the book's frame): read from the simulator, or nominal offline."""
        s = ctx.obj("red_book")
        if L.offline:
            return np.array([fx, y0, mat_top]), nominal_ln, held
        f = np.asarray(s.to_world((0.0, 0.0, 0.0)), dtype=float)
        ln = lean_deg(s)
        grip = _lean_rot(-ln) @ (np.asarray(L.measured_pos, dtype=float) - f)
        grip[1] = 0.0
        return f, ln, grip

    def turn(to, nominal_from, anchor=None):
        """Loosen the pinch and turn the book about its foot edge on the mat to ``to`` degrees of lean, reading its
        lean each step; then pinch firmly again. The hand goes where the pinched point is with the foot edge at x
        ``anchor`` (where the foot edge is when the turn starts, unless given), not where the foot edge has got to:
        a book hanging in a loose pinch rights itself by its foot sliding toward the robot, and a hand that followed
        the foot would let it creep off the mat. Held to the anchor, the hand brings the foot edge back to it."""
        nonlocal held
        yield from L.grip(_finger(PINCH_LIGHT), 0.5)
        f, ln, held = read(nominal_from)
        ax = float(f[0]) if anchor is None else float(anchor)
        ln_t = ln
        for _ in range(round(8.0 / L.dt)):
            f, ln, grip = read(ln_t)
            if ln_t == to and abs(ln - to) <= 0.5:
                break
            ln_t = to + math.copysign(max(0.0, abs(ln_t - to) - TURN_RATE * L.dt), ln_t - to)
            held = held + 0.3 * (grip - held)
            yield from lag.to(np.array([ax, y0, mat_top]) + _lean_rot(ln_t) @ held, level)
        yield from L.hold(0.3)
        yield from L.grip(_finger(PINCH_CLOSE), 0.5)

    # draw the book out level at its 22 degrees: its tail slides on the base, then, past the base's front edge, the
    # foot edge comes down the 12 mm step onto the mat as fast as the tail's underside clears that edge (by 2 mm),
    # and trails on across the mat to x 0.275
    fz = base_top
    while fx > TURN_X:
        fx = max(TURN_X, fx - DRAW_SPEED * L.dt)
        clear = base_top - (SHELF_X0 - fx) * math.tan(math.radians(lean0)) + 0.002
        fz = min(base_top, max(mat_top, clear, fz - STEP_SPEED * L.dt))
        yield from lag.to(np.array([fx, y0, fz]) + _lean_rot(lean0) @ held, level)
    yield from L.hold(0.3)
    # stand it up to 9 degrees where it is (the tail's far end, still over the base, then clears its edge by 4 mm)
    yield from turn(SLIDE_LEAN, lean0)
    # trail it on at that lean until the foot edge is at x 0.200 and the whole tail is in front of the base
    f, ln, held = read(SLIDE_LEAN)
    fx = float(f[0])
    while fx > STAND_X:
        fx = max(STAND_X, fx - SLIDE_SPEED * L.dt)
        yield from lag.to(np.array([fx, y0, mat_top]) + _lean_rot(ln) @ held, level)
    for _ in range(round(3.0 / L.dt)):  # if the foot edge trails short of its place, pull on as far as the hand reaches
        f, _, _ = read(ln)
        want = np.array([fx - 0.005 * L.dt, y0, mat_top]) + _lean_rot(ln) @ held
        if L.offline or f[0] <= STAND_X + 0.003 or want[0] < REACH_MIN_X:
            break
        fx -= 0.005 * L.dt
        yield from lag.to(want, level)
    yield from L.hold(0.3)

    # 4) set down: bring the book upright about its foot edge, the pinch loosened, until it reads upright
    # (the hand is held to the place the book is to stand, so the book ends there even if its foot edge slid)
    f, _, _ = read(SLIDE_LEAN)
    yield from turn(0.0, SLIDE_LEAN, anchor=float(np.clip(f[0], STAND_X - 0.003, STAND_X + 0.005)))
    yield from L.hold(0.3)
    # check where it stands: if its spine is more than 4 mm from x 0.200, slide it there on its tail, firmly pinched
    want = np.asarray(L.measured_pos, dtype=float)
    for _ in range(round(NUDGE_S / L.dt)):
        f, _, _ = read(0.0)
        err = STAND_X - float(f[0])
        if L.offline or abs(err) <= STAND_TOL:
            break
        want = want + np.array([math.copysign(min(abs(err), NUDGE_SPEED * L.dt), err), 0.0, 0.0])
        yield from lag.to(want, level)
    yield from L.hold(0.3)
    # let go: take the load correction off the arm, open slowly, rise straight up clear of the book's head, park
    yield from L.move(np.asarray(L.measured_pos, dtype=float) + free_off, 1.0, rot=level)
    yield from L.grip(_finger(RELEASE_OPEN), 2.0)
    yield from L.hold(0.3)
    yield from L.move((L.pos[0], L.pos[1], TOP + RISE_Z), 2.0, rot=level)
    yield from L.move((L.pos[0], 0.17, TOP + RISE_Z), 1.5, rot=level)
    yield from L.home(3.0)
