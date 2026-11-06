"""On the station table: three grading bays side by side across the table, a yellow flag pad in front of each,
and six loose tokens in a row nearest the arms.

- Bays 1, 2 and 3, from left to right as the arms see them, are centred 0.20 m left of the midline, on it, and
  0.20 m right of it, 0.40 m out from the arms. Each bay is a flat 16 x 16 cm card, 3 mm thick, fixed to the
  table: grey with a white cross 2 cm wide running edge to edge through its centre.
- One orange wooden cube, 5 cm on a side, stands on each bay card. Each shows where an earlier robot attempt to
  put the cube on the cross ended. The cubes are loose.
  - Bay 1: the cube stands on the grey corner nearest the arms and left, its centre 4.5 cm from the cross's
    centre line in both directions; the whole cross is visible.
  - Bay 2: the cube stands on the centre of the cross, turned 30 degrees.
  - Bay 3: the cube stands on the grey corner farthest from the arms and right, again 4.5 cm off in both
    directions; the whole cross is visible.
- In front of each bay, 0.25 m out from the arms, lies a yellow flag pad, a 10 x 10 cm card 3 mm thick, fixed.
- Six wooden tokens, cylinders 4 cm across and 4 cm tall, stand upright in one row across the table 0.13 m out
  from the arms. From left to right: green, red, green, red, green, red. They are loose.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- sizes (m) ---------------------------------------------------------------------------------------------
GRIPPER_MAX_OPENING = 0.094  # pad gap at FINGER_OPEN (yam_arena.constants; tests/test_constants.py)
WOOD_DENSITY = 600.0  # kg/m^3, painted craft wood
SPAWN_GAP = 0.001  # loose objects start this far above their support and settle onto it

CUBE = 0.050  # graded cubes: 50 mm on a side
CUBE_MASS = round(WOOD_DENSITY * CUBE**3, 3)  # 0.075 kg
TOKEN_R = 0.020  # tokens: 40 mm across
TOKEN_H = 0.040  # and 40 mm tall
TOKEN_MASS = round(WOOD_DENSITY * math.pi * TOKEN_R**2 * TOKEN_H, 3)  # 0.030 kg

CARD_H = 0.003  # bay cards and flag pads: 3 mm card
CARD_TOP = TOP + CARD_H
BAY = 0.160  # bay card: 16 x 16 cm
STRIP = 0.020  # the cross's arms are 2 cm wide and run edge to edge
ARM_LEN = (BAY - STRIP) / 2  # 0.07: one arm of the cross from the centre square to the card's edge
BAY_X = 0.40  # centre of every bay's cross
BAY_Y = {1: 0.20, 2: 0.0, 3: -0.20}  # bay 1 is on the left as the arms see it
BAYS = tuple(BAY_Y)
PAD = 0.100  # flag pad: 10 x 10 cm
PAD_X = 0.25

GREY = (0.45, 0.47, 0.50)
WHITE = (0.95, 0.95, 0.95)
ORANGE = (0.95, 0.55, 0.10)
YELLOW = (0.95, 0.85, 0.20)
GREEN = (0.15, 0.65, 0.25)
RED = (0.85, 0.15, 0.15)


def _bay_card(bay: int) -> list[Box]:
    """A flat card tiled from fixed boxes, all CARD_H thick and flush: a white strip along x through the
    centre, two white half-strips along y, and four grey corner squares."""
    cx, cy = BAY_X, BAY_Y[bay]
    z = TOP + CARD_H / 2
    off = STRIP / 2 + ARM_LEN / 2  # 0.045: centre of an arm or corner tile from the cross centre
    tiles = [
        Box(f"bay{bay}_cross_x", pos=(cx, cy, z), size=(BAY, STRIP, CARD_H), color=WHITE),
        Box(f"bay{bay}_cross_left", pos=(cx, cy + off, z), size=(STRIP, ARM_LEN, CARD_H), color=WHITE),
        Box(f"bay{bay}_cross_right", pos=(cx, cy - off, z), size=(STRIP, ARM_LEN, CARD_H), color=WHITE),
    ]
    for tag, sx, sy in (("near_left", -1, 1), ("near_right", -1, -1), ("far_left", 1, 1), ("far_right", 1, -1)):
        tiles.append(
            Box(f"bay{bay}_{tag}", pos=(cx + sx * off, cy + sy * off, z), size=(ARM_LEN, ARM_LEN, CARD_H), color=GREY)
        )
    return tiles


def _pad(bay: int) -> Box:
    return Box(f"pad{bay}", pos=(PAD_X, BAY_Y[bay], TOP + CARD_H / 2), size=(PAD, PAD, CARD_H), color=YELLOW)


# Where each earlier attempt left its cube: offset of the cube centre from the cross centre, and its yaw.
# A failure sits wholly on a grey corner (4.5 cm off along both axes, 6.4 cm in all), with 1 cm of grey between
# it and the cross; the success stands on the cross centre, turned 30 deg so that a grader who expects a
# square-on cube is tested too.
FAIL_OFFSET = 0.045
CUBE_PLACEMENT = {
    1: ((-FAIL_OFFSET, FAIL_OFFSET), 0.0),
    2: ((0.0, 0.0), math.radians(30.0)),
    3: ((FAIL_OFFSET, -FAIL_OFFSET), 0.0),
}
COVER = 0.010  # an attempt succeeded when the cube's centre is within 1 cm of the cross centre (it covers it)


def attempt_succeeded(bay: int) -> bool:
    (dx, dy), _ = CUBE_PLACEMENT[bay]
    return math.hypot(dx, dy) <= COVER


VERDICT = {b: ("green" if attempt_succeeded(b) else "red") for b in BAYS}  # 1 red, 2 green, 3 red
CUBE_START = {b: (BAY_X + CUBE_PLACEMENT[b][0][0], BAY_Y[b] + CUBE_PLACEMENT[b][0][1], CARD_TOP + CUBE / 2) for b in BAYS}
CUBES = [
    Box(f"cube{b}", pos=(x, y, z + SPAWN_GAP), size=(CUBE, CUBE, CUBE), mass=CUBE_MASS, color=ORANGE,
        yaw=CUBE_PLACEMENT[b][1])
    for b, (x, y, z) in CUBE_START.items()
]
CUBE_OF = {b: f"cube{b}" for b in BAYS}

# Six tokens in a row across the table, alternating colours, so each arm has both colours within reach and no
# token's position points to a bay's verdict.
TOKEN_X = 0.13
TOKEN_ROW = (
    ("token_g1", "green", 0.27),
    ("token_r1", "red", 0.16),
    ("token_g2", "green", 0.05),
    ("token_r2", "red", -0.05),
    ("token_g3", "green", -0.16),
    ("token_r3", "red", -0.27),
)
TOKEN_NAMES = tuple(n for n, _, _ in TOKEN_ROW)
TOKEN_COLOR = {n: c for n, c, _ in TOKEN_ROW}
RGB = {"green": GREEN, "red": RED}
TOKENS = [
    Cylinder(n, pos=(TOKEN_X, y, TOP + TOKEN_H / 2 + SPAWN_GAP), radius=TOKEN_R, height=TOKEN_H, mass=TOKEN_MASS,
             color=RGB[c])
    for n, c, y in TOKEN_ROW
]

OBJECTS = [*(t for b in BAYS for t in _bay_card(b)), *(_pad(b) for b in BAYS), *CUBES, *TOKENS]

# -- scoring -----------------------------------------------------------------------------------------------
MOVED = 0.010  # a cube whose centre has shifted more than 1 cm (horizontally or vertically) has been moved
TILTED_DEG = 10.0  # or whose top face is tipped more than 10 deg from level
ON_PAD = 0.010  # a token is on a pad when its bottom is within 1 cm of the pad top (not stacked, not held)
RIGHT = 1 / 3
WRONG = 2 / 3


def _up(state) -> float:
    """The z component of the body's own z axis (1 when its top face is level)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return 1 - 2 * (x * x + y * y)


def cube_moved(state, bay: int) -> bool:
    start = np.asarray(CUBE_START[bay])
    return bool(
        np.linalg.norm(np.asarray(state.pos[:2]) - start[:2]) > MOVED
        or abs(float(state.pos[2]) - start[2]) > MOVED
        or _up(state) < math.cos(math.radians(TILTED_DEG))
    )


def cubes_untouched(objects) -> bool:
    return not any(cube_moved(objects[CUBE_OF[b]], b) for b in BAYS)


def on_pad(state, bay: int) -> bool:
    """Resting with its centre over the pad and its bottom within 1 cm of the pad top, standing or lying."""
    return bool(
        abs(state.pos[0] - PAD_X) <= PAD / 2
        and abs(state.pos[1] - BAY_Y[bay]) <= PAD / 2
        and state.low[2] <= CARD_TOP + ON_PAD
        and state.resting()
    )


def flag(objects, bay: int):
    """The bay's flag: "green", "red", None (no token on its pad) or "mixed" (both colours)."""
    colours = {TOKEN_COLOR[n] for n in TOKEN_NAMES if on_pad(objects[n], bay)}
    if not colours:
        return None
    return colours.pop() if len(colours) == 1 else "mixed"


def _tally(objects):
    """(bays flagged with the right colour, bays flagged with the wrong colour or both colours)."""
    right = wrong = 0
    for b in BAYS:
        f = flag(objects, b)
        if f == VERDICT[b]:
            right += 1
        elif f is not None:
            wrong += 1
    return right, wrong


def score(objects):
    """+1/3 per bay whose pad holds only the right colour, -2/3 per bay whose pad holds a wrong colour (alone or
    with the right one), 0 for an empty pad; floored at 0. 0 whenever any cube has been moved or tipped."""
    if not cubes_untouched(objects):
        return 0.0
    right, wrong = _tally(objects)
    return round(max(0.0, RIGHT * right - WRONG * wrong), 6)


def success(objects):
    """Every pad flagged with the right colour only and no cube moved; must hold for 1 s."""
    return cubes_untouched(objects) and _tally(objects) == (len(BAYS), 0)


def distance(objects):
    """Sum over the bays of the metres from the nearest token of the right colour to its place on the pad."""
    total = 0.0
    for b in BAYS:
        lo = np.array([PAD_X - PAD / 2, BAY_Y[b] - PAD / 2, CARD_TOP + TOKEN_H / 2])
        hi = np.array([PAD_X + PAD / 2, BAY_Y[b] + PAD / 2, CARD_TOP + TOKEN_H / 2 + ON_PAD])
        total += min(
            float(np.linalg.norm(np.maximum(lo - objects[n].pos, 0) + np.maximum(objects[n].pos - hi, 0)))
            for n in TOKEN_NAMES
            if TOKEN_COLOR[n] == VERDICT[b]
        )
    return total


# -- scripted demo -----------------------------------------------------------------------------------------
GRIP_BELOW_TOP = 0.016  # the TCP grips this far below a token's top; the pads hold its upper 2.9 cm
GRASP_Z = TOP + TOKEN_H - GRIP_BELOW_TOP
RELEASE_GAP = 0.006  # a token is let go with its bottom this far above the pad
RELEASE_Z = CARD_TOP + RELEASE_GAP + TOKEN_H - GRIP_BELOW_TOP
PLACE_AIM_Y = 0.006  # aim +6 mm in y: the grip holds parts toward -y (measured on the GPU)
TRAVEL_Z = TOP + 0.12  # carry height of the TCP: a carried token's bottom is 9.6 cm above the table
REST = {"left": (0.20, 0.42, TOP + 0.12), "right": (0.20, -0.42, TOP + 0.12)}  # outboard, clear of every bay
# Which token each arm takes to which pad, in order. The left arm flags bays 1 and 2, the right arm bay 3.
DEMO = {"left": (("token_r1", 1), ("token_g2", 2)), "right": (("token_r3", 3),)}
HOLD_S = 2.0  # still hold at the end, so the 1 s success hold completes with everything resting


class _Track:
    """A waypoint list with its running end time."""

    def __init__(self):
        self.wps, self.t, self.pos, self.finger = [], 0.0, None, FINGER_OPEN

    def go(self, duration, pos=None, finger=None, label=""):
        if duration <= 1e-9:
            return
        self.pos = self.pos if pos is None else pos
        self.finger = self.finger if finger is None else finger
        self.wps.append(Waypoint(duration, self.pos, self.finger, label))
        self.t += duration

    def wait_until(self, t, label="wait"):
        self.go(t - self.t, label=label)


def demo():
    """The left arm puts a red token on bay 1's pad and then a green one on bay 2's; the right arm meanwhile puts a
    red token on bay 3's pad. Grips are straight down with the fingers closing along y, across the token row; the
    tokens are 11 cm apart, so the open pads never reach a neighbour. Each token is lowered to 6 mm above its
    pad, let go, and the pads rise 3 cm or more straight up before moving away. Neither arm goes past the pads,
    so the cubes are never approached. Both arms then move outboard, clear of every bay in the top camera's
    view, and hold still."""
    tracks = {"left": _Track(), "right": _Track()}
    xy = {o.name: o.pos[:2] for o in TOKENS}
    for side, jobs in DEMO.items():
        t = tracks[side]
        for name, bay in jobs:
            bx, by = xy[name]
            px, py = PAD_X, BAY_Y[bay] + PLACE_AIM_Y
            t.go(1.5, (bx, by, TRAVEL_Z), FINGER_OPEN, f"over {name}")
            t.go(1.2, (bx, by, GRASP_Z), label="descend")
            t.go(0.6, finger=FINGER_CLOSED, label="close")
            t.go(1.0, (bx, by, TRAVEL_Z), label="lift")
            t.go(1.5, (px, py, TRAVEL_Z), label=f"carry over pad{bay}")
            t.go(1.2, (px, py, RELEASE_Z), label="lower")
            t.go(0.5, finger=FINGER_OPEN, label="let go")
            t.go(1.0, (px, py, TRAVEL_Z), label="rise")
        t.go(1.5, REST[side], FINGER_CLOSED, "back off")
    end = max(t.t for t in tracks.values())
    for t in tracks.values():
        t.wait_until(end, "wait")
        t.go(HOLD_S, label="hold")
    return {side: t.wps for side, t in tracks.items()}
