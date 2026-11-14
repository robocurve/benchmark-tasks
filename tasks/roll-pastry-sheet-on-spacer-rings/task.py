"""A block of puff pastry on a floured pastry board, and a wooden rolling pin with 3 mm spacer rings on both ends of its
barrel, lying in its rest behind the pastry.

The pastry is rolled into a sheet 3 mm thick and at least 20 cm long (to line the floor of a 20 cm square pan), with
both hands on the pin (on its handles or palms on the barrel) so that both rings ride on the board: then the barrel
runs 3 mm above the board all the way across. A pin with one end lifted leaves a wedge (thick on that side), and the
dough springs back, so it takes several passes to come down to the rings.

Model (doughkit): the pastry has no collider; the pin rides on its rings. Each slice of pastry that the barrel rolls
across with its lowest line below the slice's top is pressed to gap + half its excess over the gap (the gap is the
barrel's height above the board where it crosses), the slice lengthens and the pastry beyond it moves out, volume
kept. Rolling past an end presses the end like any other slice (the rings hold the barrel 3 mm up there too).
The model gives the pin no rolling resistance (the pastry pushes back on nothing), so in the sim the rings keep the
pin level on the flat board whatever the hands do; a real trial checks the sheet's evenness directly (a needle gauge
across the width).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import guardkit as gk  # noqa: E402  (grip and contact checks)
import doughkit as dk  # noqa: E402  (the rolling model, updated every control step)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Roll this puff pastry out into a 3 mm sheet at least 20 cm long."

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BOARD = (0.295, 0.0)  # the pastry board's center (x, y)
BOARD_SIZE = (0.31, 0.28, 0.015)  # 31 cm deep, 28 cm across, 1.5 cm thick (maple)
BOARD_FRICTION = (0.15, 0.10)  # static, dynamic: a floured board under a wooden pin
BZ = TOP + BOARD_SIZE[2]  # the board's top

PASTRY = (0.295, 0.0)  # the block's center
PASTRY_SIZE = (0.065, 0.20, 0.010)  # 6.5 cm long (x), 20 cm wide (y), 10 mm thick: 130 mL

R_BARREL = 0.025  # the pin's barrel: 5 cm across ...
BARREL_HALF = 0.13  # ... 26 cm long
RING_GAP = 0.003  # the spacer rings stand 3 mm proud of the barrel ...
R_RING = R_BARREL + RING_GAP
RING_IN, RING_OUT = 0.115, 0.13  # ... and sit on the barrel's last 1.5 cm at each end
R_HANDLE = 0.0125  # handles 2.5 cm across ...
HANDLE_IN, HANDLE_OUT = 0.13, 0.215  # ... 8.5 cm long; 43 cm overall
PIN_MASS = 0.45  # kg
REST_X = 0.415  # the pin rest: two wooden cradles on the table, one under each handle's outer end
REST_Y = (0.195, 0.215)  # |y| span of each cradle
REST_AXIS = TOP + 0.050  # the pin's axis in its rest: the rings 7 mm above the board
LIP = (0.005, 0.010)  # each cradle's front and back lips: 5 mm thick, 1 cm above the floor that carries the handle
REST_PLAY = 0.005  # m: room between the handle and each lip (a set-down pin drops in)
PIN_START = (REST_X, 0.0, REST_AXIS)  # in its rest, behind the pastry (a round pin left on the board rolls away)

# --- The rule's numbers (reasons in score() and the README).
TARGET_T = 0.003  # m: the sheet's thickness (the rings' gap)
TOL_T = 0.0005  # m: a cell counts as on spec within 0.5 mm of 3 mm
SPEC_SHARE = 0.90  # share of the sheet's cells that must be on spec (the 1 cm trimmed off each end is ~10 %)
MIN_LEN = 0.20  # m: every strip of the sheet at least this long (the pan's floor)
PART_LEN = 0.18  # m: 90 % of that (about one more pass to go) earns half
START_LEN = 0.16  # m: 80 % of it, what about two even passes reach (two level passes make 17.2 cm, one makes 11.8 cm;
#                   1 cm to spare for strokes that stop short of the ends) earns a quarter
WEDGE_MAX = 0.0005  # m: the 1 cm strips' average thicknesses may differ by at most this across the width (the same
#                     0.5 mm a cell is allowed off 3 mm); a pin rolled with one end lifted leaves the sheet thicker on
#                     that side, and puff pastry rises higher and bakes slower where it is thicker, so a wedge can
#                     bake lopsided
PIN_DOWN = 0.005  # m: rings within 5 mm of the board, or axis within 5 mm of its rest seat, level within LEVEL_DEG: set down
LEVEL_DEG = 3.0

GRIP_Y = 0.165  # where the hands take the handles (|y|), inboard of the rest

WOOD = kit.Look((0.80, 0.62, 0.40), 0.6)
DARKWOOD = kit.Look((0.55, 0.36, 0.20), 0.5)
BOARDWOOD = kit.Look((0.66, 0.48, 0.30), 0.8)  # oiled maple, darker than the table so the pastry reads on it
RINGLOOK = kit.Look((0.20, 0.45, 0.80), 0.4)  # silicone spacer rings


def _board():
    top = ((0.0, 0.0, BOARD_SIZE[2] / 2), BOARD_SIZE)
    return kit.Prop("board", pos=(*BOARD, TOP), size=BOARD_SIZE, meshes=(kit.box(*top, BOARDWOOD),),
                    colliders=(kit.Collider(*top),), friction=BOARD_FRICTION)


def _pin():
    m = [kit.cylinder((0, 0, 0), R_BARREL, 2 * BARREL_HALF, WOOD, axis="y")]
    c = [kit.Collider((0, 0, 0), radius=R_BARREL, height=2 * BARREL_HALF, axis="Y")]
    for s in (-1, 1):
        ring = (0.0, s * (RING_IN + RING_OUT) / 2, 0.0)
        handle = (0.0, s * (HANDLE_IN + HANDLE_OUT) / 2, 0.0)
        m += [kit.cylinder(ring, R_RING, RING_OUT - RING_IN, RINGLOOK, axis="y"),
              kit.cylinder(handle, R_HANDLE, HANDLE_OUT - HANDLE_IN, DARKWOOD, axis="y")]
        c += [kit.Collider(ring, radius=R_RING, height=RING_OUT - RING_IN, axis="Y"),
              kit.Collider(handle, radius=R_HANDLE, height=HANDLE_OUT - HANDLE_IN, axis="Y")]
    return kit.Prop("pin", pos=PIN_START, size=(2 * R_RING, 2 * HANDLE_OUT, 2 * R_RING), mass=PIN_MASS,
                    meshes=tuple(m), colliders=tuple(c))


def _rest():
    floor = REST_AXIS - R_HANDLE - TOP
    m, c = [], []
    for s in (-1, 1):
        yc, wy = s * sum(REST_Y) / 2, REST_Y[1] - REST_Y[0]
        parts = [((0.0, yc, floor / 2), (0.045, wy, floor))]
        for k in (-1, 1):
            parts.append(((k * (R_HANDLE + REST_PLAY + LIP[0] / 2), yc, floor + LIP[1] / 2), (LIP[0], wy, LIP[1])))
        for center, size in parts:
            m.append(kit.box(center, size, DARKWOOD))
            c.append(kit.Collider(center, size=size))
    return kit.Prop("rest", pos=(REST_X, 0.0, TOP), size=(0.045, 2 * REST_Y[1], floor + LIP[1]), meshes=tuple(m),
                    colliders=tuple(c))


OBJECTS = [_board(), _rest(), _pin()]

# The pin's shape in its own frame (axis along y), as its colliders: the barrel, a spacer ring on each end of it, and
# the handles (left at +y, right at -y), as flat-ended cylinders.
PIN_SHAPE = (gk.Cyl((0, -BARREL_HALF, 0), (0, BARREL_HALF, 0), R_BARREL),
             gk.Cyl((0, RING_IN, 0), (0, RING_OUT, 0), R_RING),
             gk.Cyl((0, -RING_IN, 0), (0, -RING_OUT, 0), R_RING),
             gk.Cyl((0, HANDLE_IN, 0), (0, HANDLE_OUT, 0), R_HANDLE),
             gk.Cyl((0, -HANDLE_IN, 0), (0, -HANDLE_OUT, 0), R_HANDLE))

def _gate(objects):
    """True while both hands are on the pin: each gripper's own shape (palm and fingers at their present opening)
    touching it within 2 mm (gk.touching), anywhere on its handles, barrel or rings, gripping it or pressing on it with
    a closed hand. The sim's pin slides on its rings instead of turning, so the model cannot tell rolling from pushing
    the pin along; it counts presses only while two hands are on the pin, so that both ends can be kept down. A hand
    hovering off the pin does not count, and strokes made with one hand (or none) just do not press."""
    return len(set(gk.touching(objects, "pin", PIN_SHAPE))) >= 2


dk.setup(dk.Sheet(center=PASTRY, size=PASTRY_SIZE, board_z=BZ, x_range=(BOARD[0] - BOARD_SIZE[0] / 2,
                                                                         BOARD[0] + BOARD_SIZE[0] / 2)),
         dk.Pin("pin", radius=R_BARREL, length=2 * BARREL_HALF), gate=_gate)


def _rotm(q):
    return dk._rotm(q)


def pin_down(objects):
    """The pin set down, axis within 3 degrees of level: back in its rest (axis within 5 mm of the rest's seat height
    and within 1 cm of its x), or lying on its rings on the board (their bottoms within 5 mm of the board's top)."""
    o = objects["pin"]
    a = _rotm(o.quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    if math.degrees(math.asin(min(1.0, abs(a[2])))) > LEVEL_DEG:
        return False
    z = float(o.pos[2])
    in_rest = abs(z - REST_AXIS) <= PIN_DOWN and abs(float(o.pos[0]) - REST_X) <= 0.01
    return in_rest or abs(z - R_RING - BZ) <= PIN_DOWN


def released(objects):
    """Let go: no gripper touching the pin's barrel, rings or handles (gk.touching, the gripper's own shape within
    2 mm). An open hand still around a handle without touching it has let go; the pin rests on its own."""
    return not gk.touching(objects, "pin", PIN_SHAPE)


def sheet(objects=None):
    """(on-spec share, shortest strip length m, thinnest cell m) of the pastry now."""
    t = dk.grid()
    cov = t > 0
    on = (np.abs(t[cov] - TARGET_T) <= TOL_T + 1e-9).mean()
    return float(on), float(dk.lengths().min()), float(t[cov].min())


def wedge(objects=None):
    """How much thicker the thickest 1 cm strip is than the thinnest, each averaged over its length (m): 0 for a pin
    kept level, which presses every strip alike; a pin with one end lifted leaves that side thicker."""
    t = dk.grid()
    means = [row[row > 0].mean() for row in t if (row > 0).any()]
    return float(max(means) - min(means)) if means else 0.0


def _job(objects):
    on, length, thin = sheet(objects)
    even = wedge(objects) <= WEDGE_MAX + 1e-9
    if length >= MIN_LEN and on >= SPEC_SHARE and even:
        s = 1.0
    elif length >= PART_LEN:
        s = 0.5  # most of the way: at least 18 cm long but short of the pan or uneven (about one pass to go)
    elif length >= START_LEN and even:
        s = 0.25  # about two even passes: rolled level, but still thick and a few passes short
    else:
        return 0.0  # nothing done, one pass, or a strip left thick and short (a wedge from a tilted pin)
    if not released(objects) or not pin_down(objects):
        return min(s, 0.5)  # a hand still touching the pin, or the pin not set down (in its rest or on the board)
    return s


def _score(objects):
    return _job(objects)


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """The pastry is read on a 1 cm grid. Doing nothing scores 0. Only rolling with both hands on the pin counts (each
    gripper touching it, on its handles or barrel, within 2 mm; the sim's pin slides rather than turns, so this
    stands in for a pin rolled under two hands, which can keep both ends down). 1.0 if every 1 cm strip of the sheet
    is at least 20 cm long (the floor of a 20 cm square pan), at least 90 % of its cells are 3 mm thick within 0.5
    mm (the rings' gap; the remaining 10 % is about the 1 cm a baker trims off each end of a 21 cm sheet), the
    strips' average thicknesses differ by at most 0.5 mm across the width (a pin with one end lifted leaves a wedge,
    and a wedge can bake lopsided), and the pin is set down level (within 3 degrees) in its rest or on its rings on
    the board (within 5 mm) with no hand touching it (2 mm contact tolerance). 0.5 if every strip is at least 18 cm
    long (90 % of the length, about one pass to go) but the sheet is not finished, or if the sheet is done but a
    hand still touches the pin or it is not set down. 0.25 if every strip is at least 16 cm long (about two even
    passes) and the strips are within 0.5 mm of each other. 0 otherwise (nothing done, one pass, or a wedge left by
    a tilted pin). Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0."""
    return _score(objects)


# --- Demo: both arms take the pin by its handles, lift it over the middle of the block, set it down on its rings
# and roll from the middle out to each end and back, stopping short of the end, five passes; then lift it back into
# its rest, let go and go home.
CENTER_X = PASTRY[0]
AXIS_Z = BZ + R_RING  # tool point on the handle's axis with the pin on its rings
PRESS = 0.004  # m: the hands are commanded this far below the axis, so the rings stay on the board
LIFT = 0.05
SPEED = 0.04  # m/s along a stroke
YAW = math.pi / 2  # fingers close along x, across the handle
# Stroke ends (x of the barrel's line), five passes, worked out with doughkit: 4 mm short of the end at that pass (the
# pin trails its hands by about 1 cm at the turn, so it stops about 1.4 cm short)
STROKES = (0.332, 0.295, 0.249, 0.295, 0.361, 0.295, 0.219, 0.295, 0.385, 0.295, 0.201, 0.295, 0.394, 0.295, 0.194, 0.295,
           0.398, 0.295, 0.192, 0.295)


def _arm(y):
    half = FINGER_OPEN
    out = [Waypoint(2.5, (REST_X, y, REST_AXIS + LIFT), half, "over the handle", yaw=YAW),
           Waypoint(1.2, (REST_X, y, REST_AXIS), half, "around the handle", yaw=YAW),
           Waypoint(0.6, (REST_X, y, REST_AXIS), FINGER_CLOSED, "grip", yaw=YAW),
           Waypoint(1.0, (REST_X, y, REST_AXIS + LIFT), FINGER_CLOSED, "lift the pin out of its rest", yaw=YAW),
           Waypoint(1.5, (CENTER_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "over the middle of the block", yaw=YAW),
           Waypoint(1.2, (CENTER_X, y, AXIS_Z - PRESS), FINGER_CLOSED, "rings down on the board", yaw=YAW)]
    x = CENTER_X
    for k, end in enumerate(STROKES):
        out.append(Waypoint(abs(end - x) / SPEED + 0.3, (end, y, AXIS_Z - PRESS), FINGER_CLOSED, f"stroke {k + 1}",
                            yaw=YAW))
        x = end
    out += [Waypoint(1.0, (x, y, AXIS_Z + LIFT), FINGER_CLOSED, "lift the pin off the pastry", yaw=YAW),
            Waypoint(1.5, (REST_X, y, REST_AXIS + LIFT), FINGER_CLOSED, "over its rest", yaw=YAW),
            Waypoint(1.2, (REST_X, y, REST_AXIS), FINGER_CLOSED, "into its rest", yaw=YAW),
            Waypoint(0.6, (REST_X, y, REST_AXIS), FINGER_OPEN, "let go", yaw=YAW),
            Waypoint(1.0, (REST_X, y, REST_AXIS + LIFT), FINGER_OPEN, "clear of the pin", yaw=YAW)]
    return out


def demo():
    """Both arms on the pin's handles; strokes from the middle out to each end and back; let go; home."""
    left, right = _arm(GRIP_Y), _arm(-GRIP_Y)
    tail = [kit.Home(2.0), kit.Home(SUCCESS_HOLD_S + 1.0, label="hold home")]
    return {"left": left + tail, "right": right + list(tail)}
