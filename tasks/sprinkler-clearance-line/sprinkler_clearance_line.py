"""On the station table: a long grey storage shelf (10 x 48 cm, 5 cm tall, fixed) runs across the table 21 to 31 cm in
front of the arms. Behind its middle stands a white post (2 cm across, 20 cm tall) with a red band on it whose lower
edge is 11.5 cm above the table: the height limit for stored items. On the shelf, from left to right: an orange
carton stacked on a blue carton at y +19 (4.5 cm cubes), a green bottle (5 cm across, 10 cm tall) at +10, a blue carton
at +1, a blue carton at -10 and a green bottle at -19. Two dark-grey floor pads (8 x 8 cm, 4 mm thick, fixed) lie on
the table nearer the arms, at x 13, y +17 and -17. Three items stand taller than the band: the orange carton (top at
14 cm) and both bottles (tops at 15 cm). The cartons and bottles are loose.

Requires -E object_jitter=0: jitter can topple the two-carton stack.

Run it:  inspect-robots run --embodiment yam_arena -E task=sprinkler_clearance_line.py -E object_jitter=0 \\
           --instruction "Fire code: nothing stored here may stand taller than the red band on the white post, 11.5 cm
           above the table. Fix every item that breaks this rule; you may use the grey floor pads. Everything must
           end upright on the shelf or on a pad, not stacked." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import Rect, _rot, bottom, is_airborne, overlaps, tilt_deg, top  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
# success() only sees the objects, so the episode would end the moment the last item is let go, with the hand still
# around it. Holding success for 6 s keeps the run going while the arm rises and parks, so the release is seen.
SUCCESS_HOLD_S = 6.0

SHELF_GREY, PAD_GREY, WHITE, RED = (0.55, 0.56, 0.58), (0.22, 0.22, 0.24), (0.93, 0.93, 0.91), (0.85, 0.08, 0.08)
BLUE, ORANGE, GREEN = (0.15, 0.35, 0.75), (0.90, 0.40, 0.10), (0.15, 0.55, 0.25)

# The shelf: fixed, 10 x 48 x 5 cm, x 0.21..0.31, y -0.24..0.24, top at TOP + 0.05.
SHELF_H = 0.05
SHELF_TOP = TOP + SHELF_H
SHELF_AREA = Rect((0.21, 0.31), (-0.24, 0.24))
SHELF = Box("shelf", pos=(0.26, 0.0, TOP + SHELF_H / 2), size=(0.10, 0.48, SHELF_H), color=SHELF_GREY)

# The height limit: a red band on a white post behind the shelf. The band's lower edge is the limit.
LIMIT = TOP + 0.115
POST = Cylinder("post", pos=(0.36, 0.0, TOP + 0.10), radius=0.01, height=0.20, color=WHITE)
BAND = Cylinder("band", pos=(0.36, 0.0, LIMIT + 0.005), radius=0.014, height=0.01, color=RED)

# Two floor pads: fixed, 8 x 8 cm, 4 mm thick, at x 0.13, y +-0.17.
PAD_H = 0.004
PAD_TOP = TOP + PAD_H
PAD_XY = {"pad_l": (0.13, 0.17), "pad_r": (0.13, -0.17)}
PAD_AREA = {n: Rect((x - 0.04, x + 0.04), (y - 0.04, y + 0.04)) for n, (x, y) in PAD_XY.items()}
PADS = [Box(n, pos=(x, y, TOP + PAD_H / 2), size=(0.08, 0.08, PAD_H), color=PAD_GREY) for n, (x, y) in PAD_XY.items()]

# Cartons: 4.5 cm cubes, 80 g. box_top, the orange one, is stacked on box_base (its top at TOP + 0.14, over the limit);
# the other three are blue, so the carton that has to move can be told from the others at scoring.
CUBE = 0.045


def _carton(name, y, z, color=BLUE):
    return Box(name, pos=(0.26, y, z), size=(CUBE,) * 3, mass=0.08, color=color)


BOX_BASE = _carton("box_base", 0.19, SHELF_TOP + CUBE / 2)
BOX_TOP = _carton("box_top", 0.19, SHELF_TOP + CUBE * 1.5, ORANGE)
BOX_MID = _carton("box_mid", 0.01, SHELF_TOP + CUBE / 2)
BOX_END = _carton("box_end", -0.10, SHELF_TOP + CUBE / 2)

# Bottles: 5 cm across, 10 cm tall, 150 g, standing on the shelf (tops at TOP + 0.15, over the limit).
R, H = 0.025, 0.10
BOTTLE_L = Cylinder("bottle_l", pos=(0.26, 0.10, SHELF_TOP + H / 2), radius=R, height=H, mass=0.15, color=GREEN)
BOTTLE_R = Cylinder("bottle_r", pos=(0.26, -0.19, SHELF_TOP + H / 2), radius=R, height=H, mass=0.15, color=GREEN)

CARTONS = (BOX_BASE, BOX_TOP, BOX_MID, BOX_END)
BOTTLES = (BOTTLE_L, BOTTLE_R)
ITEMS = (*CARTONS, *BOTTLES)
VIOLATORS = (BOX_TOP, BOTTLE_L, BOTTLE_R)  # over the limit at the start
COMPLIANT = (BOX_BASE, BOX_MID, BOX_END)  # within the limit at the start (box_base only once nothing is on it)

OBJECTS = [SHELF, POST, BAND, *PADS, *ITEMS]

MAX_TILT_DEG = 15.0
SEAT = 0.01  # an item's underside within 1 cm above a storage surface (3 mm below allowed for contact) counts as on it
OFF_TABLE = TOP - 0.05  # an underside below this: the item left the table
SURFACES = [(SHELF_TOP, SHELF_AREA), *((PAD_TOP, PAD_AREA[n]) for n in PAD_XY)]


def _upright(s, spec) -> bool:
    """A bottle stands on its base, tilted 15 degrees or less. A carton may rest on any face: one of its three axes
    within 15 degrees of vertical."""
    if spec in BOTTLES:
        return tilt_deg(s) <= MAX_TILT_DEG
    return float(np.max(np.abs(_rot(s.quat_xyzw)[2, :]))) >= np.cos(np.radians(MAX_TILT_DEG))


def _on_storage(s, spec) -> bool:
    """Underside within 1 cm above the shelf top or a pad top, with the centre over that surface. Also not more than
    1 cm above every support, so an item still held in the air never counts."""
    low = bottom(s, spec)
    if is_airborne(s, spec, (TOP, PAD_TOP, SHELF_TOP), clearance=SEAT):
        return False
    return any(
        z - 0.003 <= low <= z + SEAT and area.x0 <= s.pos[0] <= area.x1 and area.y0 <= s.pos[1] <= area.y1
        for z, area in SURFACES
    )


def _under_something(objects, spec) -> bool:
    """Another item rests on this one: its outline overlaps this item's outline (seen from above, bounding box) and
    its underside is within 1 cm of this item's top."""
    s = objects[spec.name]
    area, t = Rect((s.low[0], s.high[0]), (s.low[1], s.high[1])), top(s, spec)
    for o in ITEMS:
        if o is spec:
            continue
        so = objects[o.name]
        if abs(bottom(so, o) - t) <= SEAT and overlaps(so, o, area):
            return True
    return False


def stored(objects, spec) -> bool:
    """Stored within the rule: upright, on the shelf or a pad (centre over it, underside within 1 cm of its top), top
    at or below the band (TOP + 0.115), and not stacked: not on another item and nothing on it."""
    s = objects[spec.name]
    return bool(
        _upright(s, spec)
        and _on_storage(s, spec)
        and top(s, spec) <= LIMIT
        and not _under_something(objects, spec)
    )


def score(objects):
    """0 if any item has left the table. Otherwise (violators stored / 3) x (compliant cartons stored / 3): the three
    items that start over the limit (box_top and both bottles) must be fixed, and the three cartons that start
    within it (box_base, box_mid, box_end) must still be stored at the end. An item still over the limit, tipped,
    stacked, under another item, held up, or off the shelf and pads is not stored."""
    if any(bottom(objects[i.name], i) < OFF_TABLE for i in ITEMS):
        return 0.0
    fixed = sum(stored(objects, i) for i in VIOLATORS)
    kept = sum(stored(objects, i) for i in COMPLIANT)
    return (fixed / 3.0) * (kept / 3.0)


def success(objects):
    """Every item stored within the rule, and every item still."""
    return score(objects) == 1.0 and all(objects[i.name].resting() for i in ITEMS)


# The robot itself keeps clear of the post and its band: no part of either arm comes within 4 cm of the band below
# 25 cm above the table during the demo (checked offline by replaying demo() through the arm's kinematics). The
# demo also never puts either hand in the stacking band (TOP + 0.11 to TOP + 0.15) over the shelf, except in the
# column of the two-carton stack to lift the orange carton off, nor in the band TOP + 0.10 to TOP + 0.15 over a pad.
ARM_KEEP_OUT = [{"name": "post and band, 4 cm around", "x": (0.306, 0.414), "y": (-0.054, 0.054), "z_top": TOP + 0.25}]

# Demo heights (TCP, world z). The TCP is between the fingertips' pads, 1.3 cm above their lower edge.
TRAVEL = TOP + 0.16  # above every stacking band
FRONT_X = 0.17  # in front of the shelf (its front edge is at x 0.21): where the hand rises and drops
GAP_X = 0.19  # between a pad (x up to 0.17) and the shelf: where the hand rises after leaving a pad
PAD_R_SIDE_Y = -0.24  # right of pad_r: bottle_r stands in line with pad_r, so it leaves the shelf outside the pad's band
BOX_OUT_X = 0.185  # where the open hand backs out to after setting the carton down: clear of the bottle on pad_l
BOTTLE_GRASP = BOTTLE_L.pos[2]  # TOP + 0.10: the bottle's middle, below the shelf's stacking band
BOTTLE_SLIDE = BOTTLE_GRASP + 0.004  # carried bottle's underside 4 mm above the shelf top
PAD_APPROACH = PAD_TOP + H / 2 + 0.010  # carried bottle's underside 1 cm above the pad
PAD_PLACE = PAD_TOP + H / 2 + 0.003  # TOP + 0.057
BOX_GRASP = BOX_TOP.pos[2]  # TOP + 0.1175: the fingertips stay 9.5 mm above box_base
BOX_DROP = (0.26, 0.10)  # where bottle_l stood
BOX_APPROACH = SHELF_TOP + CUBE / 2 + 0.0075  # carried carton's underside 7.5 mm above the shelf top
BOX_PLACE = SHELF_TOP + CUBE / 2 + 0.003  # TOP + 0.0755
PARK = (0.15, 0.25)  # where each arm finishes (y mirrored for the right arm), at travel height


def _wp(t, x, y, z, finger, label):
    return Waypoint(t, (x, y, z), finger, label, 0.0)


def _bottle_to_pad(bottle, pad, reach_x=FRONT_X, out_y=None):
    """Reach the bottle from the front, below the shelf's stacking band; slide it out; lower it in front of the
    shelf; carry it sideways onto the pad, below the pad's band; set it down; slide the open hand out toward the
    shelf; rise between the pad and the shelf. reach_x and out_y keep the hand out of the pad's band when the bottle
    stands in line with its pad: the open hand comes down at reach_x and the bottle leaves the shelf toward out_y."""
    bx, by, _ = bottle.pos
    out_y = by if out_y is None else out_y
    px, py = PAD_XY[pad]
    o, c = FINGER_OPEN, FINGER_CLOSED
    return [
        _wp(2.0, reach_x, by, TRAVEL, o, "in front of the bottle"),
        _wp(1.5, reach_x, by, BOTTLE_GRASP, o, "down, in front of the shelf"),
        _wp(1.5, bx, by, BOTTLE_GRASP, o, "in, around the bottle"),
        _wp(0.8, bx, by, BOTTLE_GRASP, c, "close"),
        _wp(0.5, bx, by, BOTTLE_SLIDE, c, "lift 4 mm"),
        _wp(1.5, FRONT_X, out_y, BOTTLE_SLIDE, c, "out, off the shelf"),
        _wp(1.0, FRONT_X, out_y, PAD_APPROACH, c, "down, in front of the shelf"),
        _wp(1.5, px, py, PAD_APPROACH, c, "across onto the pad"),
        _wp(0.6, px, py, PAD_PLACE, c, "lower"),
        _wp(0.6, px, py, PAD_PLACE, o, "release"),
        _wp(1.0, GAP_X, py, PAD_PLACE, o, "out, toward the shelf"),
        _wp(1.2, GAP_X, py, TRAVEL, o, "up, between the pad and the shelf"),
    ]


def _unstack():
    """Lift box_top off box_base from above (the one column where the hand enters the shelf's stacking band), carry
    it at travel height to the front of bottle_l's old spot, lower it there, slide it in onto the shelf, set it down
    and slide the open hand back out."""
    bx, by, _ = BOX_TOP.pos
    dx, dy = BOX_DROP
    o, c = FINGER_OPEN, FINGER_CLOSED
    return [
        _wp(1.5, bx, by, TRAVEL, o, "above the stack"),
        _wp(1.2, bx, by, BOX_GRASP, o, "down to the orange carton"),
        _wp(0.8, bx, by, BOX_GRASP, c, "close"),
        _wp(1.2, bx, by, TRAVEL, c, "lift"),
        _wp(1.5, FRONT_X, dy, TRAVEL, c, "to the front of the empty spot"),
        _wp(1.2, FRONT_X, dy, BOX_APPROACH, c, "down, in front of the shelf"),
        _wp(1.5, dx, dy, BOX_APPROACH, c, "in, over the empty spot"),
        _wp(0.5, dx, dy, BOX_PLACE, c, "lower"),
        _wp(0.6, dx, dy, BOX_PLACE, o, "release"),
        _wp(1.5, BOX_OUT_X, dy, BOX_PLACE, o, "out, off the shelf"),
        _wp(1.2, BOX_OUT_X, dy, TRAVEL, o, "up"),
    ]


def demo():
    """Left arm: bottle_l to pad_l, then box_top off the stack onto the shelf where bottle_l stood. Right arm, at the
    same time: bottle_r to pad_r. Every set-down and every grasp except the orange carton's is made from in front of
    the shelf or beside a pad, below the stacking bands, so no hand is ever raised over a stored item at a height
    where it could stack one. Neither arm comes within 4 cm of the post."""
    left = _bottle_to_pad(BOTTLE_L, "pad_l") + _unstack() + [_wp(1.5, *PARK, TRAVEL, FINGER_OPEN, "park")]
    right = _bottle_to_pad(BOTTLE_R, "pad_r", GAP_X, PAD_R_SIDE_Y) + [_wp(1.5, PARK[0], -PARK[1], TRAVEL, FINGER_OPEN, "park")]
    return {"left": left, "right": right}
