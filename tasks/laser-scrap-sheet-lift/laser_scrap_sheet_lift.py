"""On the station table, right of centre, a desktop CO2 laser cutter (52 x 40 x 22 cm, fixed) with its lid standing
open at its stop: a dark body with a 7 cm front wall, an aluminium honeycomb bed whose top is 14.5 cm above the
table, and the laser head on its gantry parked at the back. On the bed lies a clear, light blue tinted acrylic sheet
(22 x 20 cm, 5 mm, 219 g) that has just been cut: its near edge overhangs the honeycomb's near edge by 3 cm, and four
cut parts, their tops still covered by orange masking paper, sit loose in their holes (0.5 mm all round): a 6 cm disc
and a 3 x 5 cm rectangle cut 1.2 cm from the sheet's far edge, a 5 x 3 cm rectangle and a triangle of 4 cm sides in
the middle. Left of the machine stands an open black scrap bin (30 x 23.4 x 10 cm).

The task: lift the scrap sheet straight up off the bed with both hands so the cut parts stay on the bed, then put the
sheet in the scrap bin.

The sheet is one rigid body whose collider is boxes around the four holes (``plate.py``), and the parts are separate
bodies. Lifted level, the sheet rises off them. Lifted by its near edge first, it pivots on its far edge, which slides
toward the hands: the far holes' walls drag the far parts along until the sheet has tilted 25 degrees. Slid along the
bed, the walls of the holes push every part along.

Run it:  inspect-robots run --embodiment yam_arena -E task=laser_scrap_sheet_lift.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import wrist  # noqa: E402, F401  (lets ArmPlan play PinchWaypoint)
import cutter as C  # noqa: E402
from layout import (  # noqa: E402
    BED_TOP,
    BED_X,
    BED_Y,
    BIN_FLOOR,
    BIN_INSIDE,
    BIN_RIM,
    BIN_XY,
    DISC_R,
    DISC_SIDES,
    LID_OPEN_POS,
    LID_OPEN_SIZE,
    PART_SIZE,
    SHEET_EDGE_X,
    SHEET_POS,
    SHEET_SIZE,
    TOP,
    XF,
    YC,
    bin_parts,
    circle_outline,
    hole_outlines,
    open_lid_pieces,
    part_mass,
    part_pos,
    sheet_mass,
    triangle_outline,
)
from looks import dress  # noqa: E402
from plate import Plate, Prism, collider, pieces, plate, prism, surface  # noqa: E402
from wrist import PinchWaypoint  # noqa: E402
from yam_arena.demo import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
SUCCESS_HOLD_S = 1.0
ACRYLIC_FRICTION = 0.4  # acrylic on acrylic and on the bed's aluminium

# -- the laser cutter and the bin: fixed ---------------------------------------------------------------------------------
BODY_COLOR = (0.13, 0.14, 0.15)
CUTTER = [Box(name, pos=pos, size=size, color=BODY_COLOR) for name, pos, size in C.placed_body(XF, YC)]
surface("cutter_bed", ACRYLIC_FRICTION)
LID = Box("cutter_lid_open", pos=LID_OPEN_POS, size=LID_OPEN_SIZE, color=BODY_COLOR)
collider("cutter_lid_open", open_lid_pieces())  # the lid's boxes, turned 80 degrees open about its hinge
BIN = [Box(name, pos=pos, size=size, color=(0.06, 0.06, 0.07)) for name, pos, size in bin_parts()]

# -- the sheet and its parts --------------------------------------------------------------------------------------------
HOLES = hole_outlines()
SHEET = plate(Plate("sheet", pos=SHEET_POS, size=SHEET_SIZE, mass=round(sheet_mass(), 4), color=(0.75, 0.86, 0.95),
                    parts=tuple(pieces(SHEET_SIZE, list(HOLES.values())))))
surface("sheet", ACRYLIC_FRICTION)

PART_NAMES = ("disc", "triangle", "rect_long", "rect_wide")
PART_COLOR = (0.93, 0.47, 0.10)  # the orange masking paper on their tops
T = SHEET_SIZE[2]


def _part(name: str):
    size = (*PART_SIZE[name], T)
    if name == "disc":
        return prism(Prism(name, pos=part_pos(name), size=size, mass=round(part_mass(name), 4), color=PART_COLOR,
                           outline=tuple(circle_outline(0.0, 0.0, DISC_R * math.cos(math.pi / DISC_SIDES),
                                                        DISC_SIDES))))
    if name == "triangle":
        return prism(Prism(name, pos=part_pos(name), size=size, mass=round(part_mass(name), 4), color=PART_COLOR,
                           outline=tuple(triangle_outline())))
    return Box(name, pos=part_pos(name), size=size, mass=round(part_mass(name), 4), color=PART_COLOR)


PARTS = [_part(n) for n in PART_NAMES]
for _n in PART_NAMES:
    surface(_n, ACRYLIC_FRICTION)

OBJECTS = [*CUTTER, LID, *BIN, SHEET, *PARTS]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The machine's
# body mesh rides on its front wall and draws every fixed part of it, the bin's mesh rides on its floor.
LOOKS = {"cutter_front": "cutter_body.usdc", "cutter_lid_open": "cutter_lid_open.usdc", "bin_floor": "bin.usdc",
         "sheet": "sheet.usdc", "disc": "disc.usdc", "triangle": "triangle.usdc", "rect_long": "rect_long.usdc",
         "rect_wide": "rect_wide.usdc"}
dress(LOOKS, hide=[n for n, _, _ in C.body_boxes() if n != "cutter_front"]
      + [n for n, _, _ in bin_parts() if n != "bin_floor"])

# -- scoring -------------------------------------------------------------------------------------------------------------
PART_TOL = 0.006  # m: a part counts when its centre is within this of where it was cut, along the bed ...
PART_UP, PART_DOWN = 0.003, 0.005  # m: ... no higher or lower than this (lying on the bed, not on the sheet or held)
PART_TILT = 10.0  # degrees: ... and lying flat, within this
OFF_BED_UP = 0.02  # m: the sheet is off the bed when its centre is this much higher than at the start ...
BED_LO, BED_HI = (BED_X[0], BED_Y[0]), (BED_X[1], BED_Y[1])  # ... or its centre is not over the bed
OFF_TABLE_Z = -0.02  # m above the table: the sheet's centre lower than this has fallen off the table
START = {n: np.asarray(part_pos(n), float) for n in PART_NAMES}
SHEET_START = np.asarray(SHEET_POS, float)


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(objects[n].pos)) and np.all(np.isfinite(objects[n].quat_xyzw))
               for n in ("sheet", *PART_NAMES))


def tilt_deg(state) -> float:
    """How far the object's own up axis leans from vertical, in degrees."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def part_left(objects, name: str) -> bool:
    """The part lies flat on the bed where it was cut: its centre within 6 mm of its start along the bed, at most
    3 mm higher or 5 mm lower, and tilted 10 degrees or less."""
    s = objects[name]
    p, p0 = s.pos, START[name]
    return bool(math.hypot(p[0] - p0[0], p[1] - p0[1]) <= PART_TOL and p0[2] - PART_DOWN <= p[2] <= p0[2] + PART_UP
                and tilt_deg(s) <= PART_TILT)


def parts_left(objects) -> int:
    return sum(part_left(objects, n) for n in PART_NAMES)


def sheet_in_bin(objects) -> bool:
    """The sheet's centre is over the bin's inside (28.8 x 22.2 cm) and below its rim."""
    c = objects["sheet"].pos
    return bool(abs(c[0] - BIN_XY[0]) <= BIN_INSIDE[0] / 2 and abs(c[1] - BIN_XY[1]) <= BIN_INSIDE[1] / 2
                and TOP + BIN_FLOOR <= c[2] < BIN_RIM)


def sheet_off_bed(objects) -> bool:
    """The sheet's centre is not over the bed, or is 2 cm or more above where it lay."""
    c = objects["sheet"].pos
    over = BED_LO[0] <= c[0] <= BED_HI[0] and BED_LO[1] <= c[1] <= BED_HI[1]
    return bool(not over or c[2] >= SHEET_START[2] + OFF_BED_UP)


def _violation(objects) -> bool:
    return not _finite(objects) or objects["sheet"].pos[2] < TOP + OFF_TABLE_Z


def success(objects) -> bool:
    """The sheet lies still in the scrap bin and all four parts lie flat on the bed where they were cut."""
    if _violation(objects):
        return False
    return bool(sheet_in_bin(objects) and objects["sheet"].resting() and parts_left(objects) == 4)


def score(objects) -> float:
    """With the sheet in the bin: 1.0 for all four parts left flat on the bed (within 6 mm of where they were cut),
    0.75 for three, 0.5 for two. 0.25: the sheet off the bed but not in the bin, three or more parts left. 0
    otherwise, and 0 if the sheet fell off the table."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    n = parts_left(objects)
    if sheet_in_bin(objects):
        return {4: 1.0, 3: 0.75, 2: 0.5}.get(n, 0.0)
    if sheet_off_bed(objects) and n >= 3:
        return 0.25
    return 0.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Both grippers point forward along x, their jaws closing up and down, and pinch the sheet's near edge where it
# overhangs the bed, 8 cm either side of its centre: one pad under the sheet, one on it. They lift it 5 cm, level, then
# up over the machine's left side wall, carry it 27.7 cm left over the bin, tip its far edge down into the bin, let go
# and back out.
GRIP_V = 0.08  # m either side of the sheet's centre line
GRIP_DEPTH = 0.010  # m: the TCP this far in from the sheet's near edge (the pad tips 23 mm in, 7 mm short of the bed)
GRIP_X = SHEET_EDGE_X + GRIP_DEPTH
PINCH_Z = SHEET_POS[2]  # the TCP at the sheet's mid-plane
APPROACH_X = GRIP_X - 0.025  # the pads 2 mm out of the sheet before going in
OPEN = -0.012  # finger target before the pinch: the pads 24 mm apart, the sheet 5 mm
PINCH = FINGER_CLOSED  # the pads close on the 5 mm sheet and squeeze it
LIFT = 0.05  # m, level
CARRY_Z = TOP + 0.28  # the TCP while carrying: the lower fingers 7 cm over the side wall
CARRY_DY = BIN_XY[1] - SHEET_POS[1]  # 27.7 cm to the left
TIP_PITCH = math.radians(60)  # the fingers 30 degrees below the horizontal: the sheet's far edge down in the bin
TIP_Z = BIN_RIM + 0.07  # the TCP while the sheet is tipped (the lower finger 2 cm over the rim)
LEVEL = math.pi / 2
ROLL = {"right": -math.pi / 2, "left": -math.pi / 2}  # the same jaw on top for both; the right one keeps joint 6
# inside its limit when it tips the sheet (its wrist camera then faces the left gripper, 16 cm away)
HANDS = 2  # 1 in the one-handed ablation: the left arm stays at its ready pose (see README)
LIFT_TILT = 0.0  # rad the fingers turn down while lifting, so the far edge stays on the bed (the tilted ablation)


def demo():
    plans = {}
    for arm, side in (("right", -1), ("left", 1))[:HANDS]:
        y = SHEET_POS[1] + side * GRIP_V
        y_bin = y + CARRY_DY
        roll = ROLL[arm]

        def w(t, pos, finger, label, pitch=LEVEL):
            return PinchWaypoint(t, pos, finger, label, 0.0, pitch, roll)

        plans[arm] = [
            w(2.0, (APPROACH_X, y, PINCH_Z + 0.03), OPEN, "in front of the sheet's edge"),
            w(1.0, (APPROACH_X, y, PINCH_Z), OPEN, "down to the sheet"),
            w(1.2, (GRIP_X, y, PINCH_Z), OPEN, "pads over and under the edge"),
            w(0.8, (GRIP_X, y, PINCH_Z), PINCH, "pinch"),
            w(2.0, (GRIP_X, y, PINCH_Z + LIFT), PINCH, "lift level", LEVEL - LIFT_TILT),
            w(0.5, (GRIP_X, y, PINCH_Z + LIFT), PINCH, "hold", LEVEL - LIFT_TILT),
            w(2.0, (GRIP_X, y, CARRY_Z), PINCH, "up over the side wall"),
            w(3.5, (GRIP_X, y_bin, CARRY_Z), PINCH, "carry over the bin"),
            w(2.0, (GRIP_X, y_bin, TIP_Z), PINCH, "tip into the bin", TIP_PITCH),
            w(0.6, (GRIP_X, y_bin, TIP_Z), OPEN, "let go", TIP_PITCH),
            w(1.0, (GRIP_X - 0.05, y_bin, TIP_Z + 0.02), OPEN, "back out", TIP_PITCH),
            w(1.0, (GRIP_X - 0.06, y_bin, TIP_Z + 0.07), OPEN, "up"),
        ]
    return plans
