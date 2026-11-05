"""On the station table, in front of the left arm: a stack of three small boxes (6 x 6 cm, 4 cm tall each), a green
one at the bottom and two brown ones on top of it. A yellow square (9 x 9 cm) is marked on the table between the stack
and the left arm. The table to the far side of the stack is clear. The boxes are loose; the yellow square is flat on the
table.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x forward from the arms, y to the left, z up (m); the arm bases stand at x 0, y +-0.305.
BOX_X, BOX_Y, BOX_H, BOX_KG = 0.06, 0.06, 0.04, 0.08
STACK_XY = (0.24, 0.20)  # where the stack stands, and where the two brown boxes must end up
ZONE_XY, ZONE_SIDE, ZONE_T = (0.14, 0.20), 0.09, 0.001  # the yellow square, a thin fixed plate
GREEN, BROWN, YELLOW = (0.15, 0.62, 0.25), (0.62, 0.45, 0.28), (0.95, 0.82, 0.15)

TARGET = "green_box"
OTHERS = ("brown_box_middle", "brown_box_top")


def _stacked_at(level):
    return (*STACK_XY, TOP + BOX_H * (level + 0.5) + 0.0005 * (level + 1))


OBJECTS = [
    Box("yellow_square", pos=(*ZONE_XY, TOP + ZONE_T / 2), size=(ZONE_SIDE, ZONE_SIDE, ZONE_T), color=YELLOW),
    Box(TARGET, pos=_stacked_at(0), size=(BOX_X, BOX_Y, BOX_H), mass=BOX_KG, color=GREEN),
    Box(OTHERS[0], pos=_stacked_at(1), size=(BOX_X, BOX_Y, BOX_H), mass=BOX_KG, color=BROWN),
    Box(OTHERS[1], pos=_stacked_at(2), size=(BOX_X, BOX_Y, BOX_H), mass=BOX_KG, color=BROWN),
]

# Limits. A box counts as flat when it lies on a 6 x 6 cm face, either way up (a plain box upside down looks the same),
# leaning at most 10 degrees; a box on its side is not flat. "On" a surface: its lowest corner within 4 mm of it.
# The green box is in the square when all four bottom corners are inside it. The brown boxes are restacked when one
# stands flat on the table with its centre within 10 mm of the stack's original spot and the other stands flat on top
# of it, its centre within 15 mm of the lower box's centre.
MAX_TILT_DEG = 10.0
MAX_GAP = 0.004
SPOT_TOL = 0.010
STACK_TOL = 0.015

# Box geometry comes from each box's centre and orientation, not from low/high (docs/robocurve/rules.md §3).


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _corners(box):
    """The box's 8 corners in the world frame."""
    r = _rot(box.quat_xyzw)
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = (sx * BOX_X / 2, sy * BOX_Y / 2, sz * BOX_H / 2)
                out.append(tuple(float(box.pos[i]) + sum(r[i][j] * local[j] for j in range(3)) for i in range(3)))
    return out


def _tilt_deg(box) -> float:
    """How far the box's 4 cm axis leans from vertical, either way up: upside down reads 0, on its side 90."""
    return math.degrees(math.acos(min(1.0, abs(_rot(box.quat_xyzw)[2][2]))))


def _bottom_z(box) -> float:
    return min(c[2] for c in _corners(box))


def _top_z(box) -> float:
    return max(c[2] for c in _corners(box))


def _flat_on(box, surface_z) -> bool:
    return _tilt_deg(box) <= MAX_TILT_DEG and abs(_bottom_z(box) - surface_z) <= MAX_GAP


def _in_square(box) -> bool:
    """Flat on the yellow square, all four bottom corners inside it."""
    if not _flat_on(box, TOP + ZONE_T):
        return False
    half = ZONE_SIDE / 2
    bottom = sorted(_corners(box), key=lambda c: c[2])[:4]
    return all(abs(c[0] - ZONE_XY[0]) <= half and abs(c[1] - ZONE_XY[1]) <= half for c in bottom)


def _stacked(lower, upper, spot=None) -> bool:
    """``upper`` flat on ``lower``, centres within STACK_TOL; ``lower`` flat on the table (at ``spot`` if given)."""
    if not (_flat_on(lower, TOP) and _flat_on(upper, _top_z(lower))):
        return False
    if math.dist(lower.pos[:2], upper.pos[:2]) > STACK_TOL:
        return False
    return spot is None or math.dist(lower.pos[:2], spot) <= SPOT_TOL


def _restacked(objects, spot=None) -> bool:
    a, b = (objects[name] for name in OTHERS)
    return _stacked(a, b, spot) or _stacked(b, a, spot)


def success(objects):
    """The green box in the square and the brown boxes restacked on the original spot, all still; held for 1 s."""
    return (
        _in_square(objects[TARGET])
        and _restacked(objects, STACK_XY)
        and all(objects[name].resting() for name in (TARGET, *OTHERS))
    )


def distance(objects):
    """The green box's centre from the square's centre (m)."""
    return math.dist(objects[TARGET].pos[:2], ZONE_XY)


def score(objects):
    """Partial credit (success scores 1.0): 1.0 with the brown boxes restacked on the original spot, 0.5 if they are
    stacked squarely somewhere else, 0 if the green box is not in the square or the brown boxes are not stacked (left
    scattered, knocked over or off the table): getting the green box out by sweeping the others aside earns nothing."""
    if not _in_square(objects[TARGET]):
        return 0.0
    if _restacked(objects, STACK_XY):
        return 1.0
    return 0.5 if _restacked(objects) else 0.0


GRIP_ABOVE_CENTRE = 0.005  # finger pads end 12 mm above a box's bottom, clear of the box below it
ASIDE_XY = (0.34, 0.20)  # the demo's spot to set the brown boxes aside: past the stack, along the line the fingers miss
_CLEAR_Z = TOP + 3 * BOX_H + 0.015 + BOX_H / 2 + GRIP_ABOVE_CENTRE  # a carried box's bottom 1.5 cm over a full stack


def _move(pick_xy, pick_level, place_xy, place_level, place_on=TOP, label=""):
    """Pick the box whose bottom is ``pick_level`` boxes up at pick_xy and set it ``place_level`` boxes up at place_xy
    (on ``place_on``), coming down onto it and leaving straight up."""
    grip_dz = BOX_H / 2 + GRIP_ABOVE_CENTRE
    pick_z = TOP + pick_level * BOX_H + grip_dz
    place_z = place_on + place_level * BOX_H + grip_dz + 0.001
    return [
        Waypoint(1.5, (*pick_xy, _CLEAR_Z), FINGER_OPEN, f"{label}: above"),
        Waypoint(1.5, (*pick_xy, pick_z), FINGER_OPEN, f"{label}: down"),
        Waypoint(0.8, (*pick_xy, pick_z), FINGER_CLOSED, f"{label}: grip"),
        Waypoint(1.5, (*pick_xy, _CLEAR_Z), FINGER_CLOSED, f"{label}: lift"),
        Waypoint(2.0, (*place_xy, _CLEAR_Z), FINGER_CLOSED, f"{label}: carry"),
        Waypoint(1.5, (*place_xy, place_z), FINGER_CLOSED, f"{label}: set down"),
        Waypoint(0.8, (*place_xy, place_z), FINGER_OPEN, f"{label}: let go"),
        Waypoint(1.0, (*place_xy, _CLEAR_Z), FINGER_OPEN, f"{label}: up"),
    ]


def demo():
    """The left arm alone: the top brown box goes aside past the stack, the middle one on top of it, the green box onto
    the yellow square, then the two brown boxes back onto the original spot, one on the other."""
    return {"left": [
        *_move(STACK_XY, 2, ASIDE_XY, 0, label="top box aside"),
        *_move(STACK_XY, 1, ASIDE_XY, 1, label="middle box aside, on it"),
        *_move(STACK_XY, 0, ZONE_XY, 0, place_on=TOP + ZONE_T, label="green box to the square"),
        *_move(ASIDE_XY, 1, STACK_XY, 0, label="a brown box back"),
        *_move(ASIDE_XY, 0, STACK_XY, 1, label="the other on top"),
    ]}
