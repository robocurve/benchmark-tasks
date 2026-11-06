"""On the station table, on a light grey anti-static mat: a small green programmer board (8.5 x 12.5 cm) with a blue
40-pin ZIF socket, fixed, its long side along y, 4 cm right of the table's centre line. The socket's lever (4.5 cm,
silver with a red tip) stands up, open, on the socket's near side, hinged at the socket's left end (+y). A black
40-pin DIP chip (5.2 x 1.5 cm body, 5 mm thick, with two 3 mm rows of pins under it, 6 g) lies on the mat on the
robot's right, at (0.24, -0.20), along y, its notch pointing to the right (-y), away from the lever.

The task: seat the chip in the socket with its notch toward the lever (the chip turned half a turn), then push the
lever down to close it.

The socket's top has two pin slots (6 x 53 mm, 3.5 mm deep) and a 2 mm guide boss at the lever end that only fits
the chip's notch: the right way round, the chip's pins drop into the slots and its body sits on the socket; the
wrong way round, its solid end lands on the boss and it sits 2 mm high at that end. The lever is a hinge with two
detents (``bonds.Hinged``): it stays open or closed, and it snaps shut once pushed past 45 degrees.

Run it:  inspect-robots run --embodiment yam_arena -E task=zif_socket_chip.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import Hinged, hinge_angle, on_reset, on_step  # noqa: E402
from looks import dress  # noqa: E402
from parts import Compound, compound  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the mat and the programmer board ---------------------------------------------------------------------------------
MAT_SIZE = (0.34, 0.40, 0.003)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("mat", pos=(0.30, -0.08, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.64, 0.67, 0.70))

SOCKET_XY = (0.32, -0.035)  # the seated chip's centre: 4 cm right of the centre line, so the lever end is near it
BOARD_SIZE = (0.085, 0.125, 0.0016)
BOARD_TOP = MAT_TOP + BOARD_SIZE[2]
BOARD = Box("board", pos=(SOCKET_XY[0] + 0.005, SOCKET_XY[1] + 0.008, MAT_TOP + BOARD_SIZE[2] / 2), size=BOARD_SIZE,
            color=(0.08, 0.32, 0.16))

# -- the chip: a body with a notch at one end, and two rows of pins under it, as one rigid body -----------------------
# Its own frame: long side along y, the notch at the +y end, z up from the pins' bottom (-4 mm) to the body's top.
CHIP_BODY = (0.015, 0.052, 0.005)
PIN_H = 0.003  # the pin rows stand 3 mm under the body
PIN_W, PIN_LEN = 0.002, 0.048  # each row is a bar, 2 mm thick, 48 mm long (20 pins at 2.54 mm)
PIN_X = 0.0065  # the rows' centres, from the chip's middle (flush with the body's sides)
NOTCH = (0.0065, 0.0075)  # m: the notch's width and depth, at the body's +y end
CHIP_SIZE = (CHIP_BODY[0], CHIP_BODY[1], CHIP_BODY[2] + PIN_H)
CHIP_MASS = 0.006
_bz0, _bz1 = -CHIP_SIZE[2] / 2 + PIN_H, CHIP_SIZE[2] / 2  # the body's bottom and top in the chip's frame
_hw, _hl = CHIP_BODY[0] / 2, CHIP_BODY[1] / 2
_cheek = _hw - NOTCH[0] / 2
CHIP_PARTS = (
    ("body", (0.0, -NOTCH[1] / 2, (_bz0 + _bz1) / 2), (CHIP_BODY[0], CHIP_BODY[1] - NOTCH[1], CHIP_BODY[2])),
    ("cheek_left", (-(NOTCH[0] / 2 + _cheek / 2), _hl - NOTCH[1] / 2, (_bz0 + _bz1) / 2),
     (_cheek, NOTCH[1], CHIP_BODY[2])),
    ("cheek_right", (NOTCH[0] / 2 + _cheek / 2, _hl - NOTCH[1] / 2, (_bz0 + _bz1) / 2),
     (_cheek, NOTCH[1], CHIP_BODY[2])),
    ("pins_left", (-PIN_X, 0.0, _bz0 - PIN_H / 2), (PIN_W, PIN_LEN, PIN_H)),
    ("pins_right", (PIN_X, 0.0, _bz0 - PIN_H / 2), (PIN_W, PIN_LEN, PIN_H)),
)
CHIP_START_XY = (0.24, -0.20)
CHIP_START_YAW = math.pi  # the notch points to -y, away from the lever
CHIP = compound(Compound(
    "chip",
    pos=(*CHIP_START_XY, MAT_TOP + CHIP_SIZE[2] / 2),  # standing on its pins
    size=CHIP_SIZE,
    yaw=CHIP_START_YAW,
    mass=CHIP_MASS,
    color=(0.05, 0.05, 0.055),
    parts=CHIP_PARTS,
))

# -- the socket: one fixed body of boxes, in its own frame (centre of its footprint at the seated chip, z up from the
# board): a base, two pin slots between rails and a middle ridge, end walls, the guide boss and the lever's cam block --
SOCKET_W = 0.025  # across (x)
SOCKET_TOP_H = 0.012  # the socket's top above the board: where the chip's body sits
SEAT_Z = BOARD_TOP + SOCKET_TOP_H  # world height of the socket's top (the seated chip's body bottom)
SLOT_DEPTH = 0.0035  # the pins (3 mm) don't touch the slots' floor: the body sits on the top
PLAY = 0.002  # m of play each side of a pin row across its slot
PLAY_Y = 0.0025  # and at each end of the rows
SLOT_W = PIN_W + 2 * PLAY  # 6 mm
SLOT_LEN = PIN_LEN + 2 * PLAY_Y  # 53 mm
SLOT_IN, SLOT_OUT = PIN_X - SLOT_W / 2, PIN_X + SLOT_W / 2  # 3.5 and 9.5 mm from the middle
END_Y = SLOT_LEN / 2 + 0.005  # the end walls are 5 mm long
BOSS = (0.002, 0.0015, 0.002)  # m: across, along, high
# The boss sits inside the notch wherever the pins' play puts the chip: the notch spans [26 - 7.5, 26] mm, shifted
# by up to 2.5 mm either way, so [21, 23.5] mm is always inside it, and the boss is the middle 1.5 mm of that.
BOSS_Y = _hl - NOTCH[1] / 2
CAM_Y = (END_Y, 0.046)  # the lever's cam block, past the lever end
CAM_H = SOCKET_TOP_H + 0.003
_f = SOCKET_TOP_H - SLOT_DEPTH  # the slots' floor


def _span(lo, hi):
    return ((lo + hi) / 2, hi - lo)


def _socket_part(name, x, y, z):
    (cx, sx), (cy, sy), (cz, sz) = _span(*x), _span(*y), _span(*z)
    return name, (cx, cy, cz), (sx, sy, sz)


_W = SOCKET_W / 2
_SOCKET_LOCAL = (  # (name, x range, y range, z range) in the socket's frame
    ("base", (-_W, _W), (-END_Y, END_Y), (0.0, _f)),
    ("rail_left", (-_W, -SLOT_OUT), (-END_Y, END_Y), (_f, SOCKET_TOP_H)),
    ("rail_right", (SLOT_OUT, _W), (-END_Y, END_Y), (_f, SOCKET_TOP_H)),
    ("ridge", (-SLOT_IN, SLOT_IN), (-END_Y, END_Y), (_f, SOCKET_TOP_H)),
    ("end_near", (-SLOT_OUT, SLOT_OUT), (-END_Y, -SLOT_LEN / 2), (_f, SOCKET_TOP_H)),
    ("end_far", (-SLOT_OUT, SLOT_OUT), (SLOT_LEN / 2, END_Y), (_f, SOCKET_TOP_H)),
    ("boss", (-BOSS[0] / 2, BOSS[0] / 2), (BOSS_Y - BOSS[1] / 2, BOSS_Y + BOSS[1] / 2),
     (SOCKET_TOP_H, SOCKET_TOP_H + BOSS[2])),
    ("cam", (-_W, _W), CAM_Y, (0.0, CAM_H)),
)
_lo = np.min([[x[0], y[0], z[0]] for _, x, y, z in _SOCKET_LOCAL], axis=0)
_hi = np.max([[x[1], y[1], z[1]] for _, x, y, z in _SOCKET_LOCAL], axis=0)
SOCKET_SIZE = tuple(float(v) for v in _hi - _lo)
_mid = (_lo + _hi) / 2
SOCKET_PARTS = tuple(
    _socket_part(name, tuple(v - _mid[0] for v in x), tuple(v - _mid[1] for v in y), tuple(v - _mid[2] for v in z))
    for name, x, y, z in _SOCKET_LOCAL
)
SOCKET = compound(Compound(
    "socket",
    pos=(SOCKET_XY[0] + float(_mid[0]), SOCKET_XY[1] + float(_mid[1]), BOARD_TOP + float(_mid[2])),
    size=SOCKET_SIZE,
    color=(0.10, 0.25, 0.75),
    parts=SOCKET_PARTS,
))

# -- the lever: a 4 x 4 mm bar hinged at its bottom end beside the socket's near side, at the lever end --------------
LEVER_LEN = 0.045
LEVER_T = 0.004
LEVER_X = SOCKET_XY[0] - _W - 0.006 - LEVER_T / 2  # 6 mm out from the socket's near side
LEVER_PIVOT = (LEVER_X, SOCKET_XY[1] + 0.040, SEAT_Z - 0.002)  # closed, it lies just under the socket's top
CLOSED_DEG = 90.0  # it turns about +x: its top goes toward -y, along the socket
LEVER = Hinged(
    "lever",
    pos=(LEVER_PIVOT[0], LEVER_PIVOT[1], LEVER_PIVOT[2] + LEVER_LEN / 2),
    size=(LEVER_T, LEVER_T, LEVER_LEN),
    mass=0.008,
    color=(0.75, 0.75, 0.78),
    axis="X",
    hinge=(0.0, 0.0, -LEVER_LEN / 2),
    lower=0.0,
    upper=CLOSED_DEG,
    detents=(0.0, CLOSED_DEG),
    rate=0.05,  # N m/rad: 0.04 N m (0.9 N on the tip) to push it to the over-centre point
    damping=0.0075,  # N m s/rad: it swings shut in about a third of a second
)

OBJECTS = [MAT, BOARD, SOCKET, LEVER, CHIP]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged.
LOOKS = {"chip": "chip.usdc", "lever": "lever.usdc", "socket": "socket.usdc", "board": "board.usdc",
         "mat": "mat.usdc"}
dress(LOOKS)

# -- scoring ---------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.0005  # m: each end of the chip's body within this of the socket's top
IN_XY = (PLAY + 0.0005, PLAY_Y + 0.0005)  # m: the chip's centre within this of the socket's: its pins are in the slots
IN_Z = 0.0035  # m: both ends of the body less than this above the socket's top: the pins are down in the slots
FACING = math.cos(math.radians(20))  # the notch within 20 degrees of +y (toward the lever) or of -y
CLOSED = CLOSED_DEG - 5.0  # degrees: the lever is closed
OFF_TABLE_Z = -0.02  # m above the table: the chip's centre lower than this has fallen off
_ENDS = np.array([[0.0, _hl - 0.002, _bz0], [0.0, -_hl + 0.002, _bz0]])  # the body's bottom, 2 mm in from each end

# What the episode remembers: the lever closed with the chip seated (the socket clamps it), or on something else.
STATE = {"closed_on_seated": False, "was_closed": False}


@on_reset
def reset_memory(env=None) -> None:
    STATE["closed_on_seated"] = STATE["was_closed"] = False


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) for s in objects.values())


def body_ends_z(objects) -> np.ndarray:
    """The heights of the chip body's bottom near its notch end and its other end."""
    s = objects["chip"]
    return (np.asarray(s.pos) + _ENDS @ _rot(s.quat_xyzw).T)[:, 2]


def notch_facing(objects) -> float:
    """The cosine between the chip's notch direction and +y (the lever): 1 the right way round, -1 reversed."""
    return float(_rot(objects["chip"].quat_xyzw)[1, 1])


def in_socket(objects) -> bool:
    """The chip's pins are down in the slots: its centre within 2.5 x 3 mm of the socket's middle and both ends of its
    body less than 3.5 mm above the socket's top."""
    c = objects["chip"].pos
    return bool(abs(c[0] - SOCKET_XY[0]) < IN_XY[0] and abs(c[1] - SOCKET_XY[1]) < IN_XY[1]
                and np.all(body_ends_z(objects) < SEAT_Z + IN_Z))


def seated(objects) -> bool:
    """In the socket, the notch toward the lever, and the body's bottom within 0.5 mm of the socket's top at both
    ends."""
    return bool(in_socket(objects) and notch_facing(objects) > FACING
                and np.all(np.abs(body_ends_z(objects) - SEAT_Z) < SEAT_TOL))


def reversed_in_socket(objects) -> bool:
    return bool(in_socket(objects) and notch_facing(objects) < -FACING)


def lever_angle(objects) -> float:
    """Degrees the lever has turned from open (90 is flat, closed)."""
    return hinge_angle(LEVER, objects["lever"].quat_xyzw)


def lever_closed(objects) -> bool:
    return lever_angle(objects) >= CLOSED


def _violation(objects) -> bool:
    return not _finite(objects) or bool(objects["chip"].pos[2] < TOP + OFF_TABLE_Z)


def _remember(objects) -> None:
    closed = lever_closed(objects)
    if closed and not STATE["was_closed"]:
        STATE["closed_on_seated"] = seated(objects)
    if not closed:
        STATE["closed_on_seated"] = False
    STATE["was_closed"] = closed


@on_step
def watch_the_lever(env) -> None:
    """Every step, so the moment the lever closes is caught even between scored steps."""
    from yam_arena.scene import object_states

    objects = object_states(env, ["chip", "lever"])
    if _finite(objects):
        _remember(objects)


def score(objects) -> float:
    """1.0: the chip is seated, notch toward the lever, and the lever was closed on it. 0.5: seated, lever open.
    0.25: the chip is in the socket the wrong way round, or the lever was closed while the chip was in the socket
    but not seated. 0 otherwise, and 0 if the chip fell off the table."""
    if _violation(objects):
        return 0.0
    _remember(objects)
    closed = lever_closed(objects)
    if seated(objects) and closed and STATE["closed_on_seated"]:
        return 1.0
    if seated(objects) and not closed:
        return 0.5
    if reversed_in_socket(objects) or (closed and in_socket(objects)):
        return 0.25
    return 0.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects["chip"].resting() and objects["lever"].resting())


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm, straight-down grip across the chip's width (the fingers close along x, a quarter turn of yaw), pad tips
# 1.5 mm above the body's bottom: pick, lift, turn the grip half a turn on the way (through yaw 0, so the turn goes
# about the vertical), lower the pins into the slots, let go, park. Left arm, fingers closed: wait for the right arm
# to park, then sweep the lever's top along its arc toward -y until it is past its over-centre point, so it snaps
# shut. Numbers are TCP positions.
GRIP_ABOVE = 0.0015  # m: the pad tips above the chip body's bottom
GRIP = -0.0065  # finger target: 1 mm tighter than the chip (15 mm), a partial grip of about 5 N: at 10 N the
# light chip jitters between the pads
PICK_YAW = -math.pi / 2
TURN = math.pi  # half a turn in the air: the notch ends toward the lever
PLACE_YAW = PICK_YAW + TURN
TRAVEL_Z = TOP + 0.10
PICK_Z = MAT_TOP + PIN_H + GRIP_ABOVE + PAD_TIP_PAST_TCP
PLACE_Z = SEAT_Z + GRIP_ABOVE + PAD_TIP_PAST_TCP
PARK_RIGHT = (0.22, -0.26, TOP + 0.16)
WAIT_LEFT = (0.24, 0.22, TOP + 0.16)
PUSH_R = 0.035  # m from the pivot: where the pads push on the lever (it is 45 mm long)
PUSH_GAP = 0.004  # m from the lever's axis to the pads' face
PAD_HALF = 0.0064  # m: half the closed fingers' thickness along y (their face toward -y)
LEVER_ARC = (-10.0, 20.0, 45.0, 70.0)  # degrees of the push
RIGHT_DONE = 14.0  # s: the left arm starts on the lever after the right arm has parked


def push_tcp(angle_deg: float) -> tuple[float, float, float]:
    """The TCP that puts the closed fingers' -y face on the lever's back at ``angle_deg``, PUSH_R from the pivot."""
    a = math.radians(angle_deg)
    px, py, pz = LEVER_PIVOT
    ey = py - PUSH_R * math.sin(a) + PUSH_GAP * math.cos(a)
    ez = pz + PUSH_R * math.cos(a) + PUSH_GAP * math.sin(a)
    return (px, ey + PAD_HALF, ez + PAD_TIP_PAST_TCP)


def demo():
    w = Waypoint
    cx, cy = CHIP_START_XY
    sx, sy = SOCKET_XY
    mid = ((cx + sx) / 2, (cy + sy) / 2)
    right = [
        w(1.5, (cx, cy, TRAVEL_Z), FINGER_OPEN, "over the chip", PICK_YAW),
        w(1.2, (cx, cy, PICK_Z), FINGER_OPEN, "down around it", PICK_YAW),
        w(0.4, (cx, cy, PICK_Z), FINGER_OPEN, "settle", PICK_YAW),
        w(0.6, (cx, cy, PICK_Z), GRIP, "close", PICK_YAW),
        w(0.4, (cx, cy, PICK_Z), GRIP, "hold", PICK_YAW),
        w(1.0, (cx, cy, TRAVEL_Z), GRIP, "lift", PICK_YAW),
        w(1.2, (*mid, TRAVEL_Z), GRIP, "carry, turning", (PICK_YAW + PLACE_YAW) / 2),
        w(1.2, (sx, sy, TRAVEL_Z), GRIP, "over the socket", PLACE_YAW),
        w(1.0, (sx, sy, PLACE_Z + 0.008), GRIP, "pins over the slots", PLACE_YAW),
        w(0.6, (sx, sy, PLACE_Z + 0.008), GRIP, "line up", PLACE_YAW),
        w(1.0, (sx, sy, PLACE_Z), GRIP, "seat", PLACE_YAW),
        w(0.6, (sx, sy, PLACE_Z), FINGER_OPEN, "let go", PLACE_YAW),
        w(1.0, (sx, sy, TRAVEL_Z), FINGER_OPEN, "up", PLACE_YAW),
        w(1.5, PARK_RIGHT, FINGER_OPEN, "park", 0.0),
    ]
    px, py, pz = LEVER_PIVOT
    over = (px, py + 0.02, pz + LEVER_LEN + 0.02 + PAD_TIP_PAST_TCP)
    t_right = sum(x.duration for x in right)
    left = [
        w(1.5, WAIT_LEFT, FINGER_CLOSED, "aside"),
        w(max(RIGHT_DONE, t_right) - 1.5, WAIT_LEFT, FINGER_CLOSED, "wait for the chip"),
        w(1.5, over, FINGER_CLOSED, "over the lever"),
        w(1.0, push_tcp(LEVER_ARC[0]), FINGER_CLOSED, "behind the lever"),
        *[w(0.8, push_tcp(a), FINGER_CLOSED, f"push to {a:.0f} deg") for a in LEVER_ARC[1:]],
        w(0.6, push_tcp(LEVER_ARC[-1]), FINGER_CLOSED, "hold"),
        w(1.0, (px, py - 0.03, TOP + 0.10), FINGER_CLOSED, "up"),
        w(1.5, WAIT_LEFT, FINGER_CLOSED, "back"),
    ]
    return {"right": right, "left": left}
