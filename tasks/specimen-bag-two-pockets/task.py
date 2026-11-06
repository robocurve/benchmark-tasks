"""A specimen transport bag mock at the back of the table: one open-top box split by a fixed divider into two
compartments side by side. The specimen compartment (orange biohazard walls) is on the robot's left; the
document pocket (white walls) is on the robot's right. In front of it: a capped blood tube (lavender, 1.6 x
10 cm) standing in a one-tube gray holder on the left, and a folded lab requisition form (white, 10 x 6 x
1.5 cm) lying on the table on the right.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Bag the blood sample for the lab."

T = 0.01  # wall thickness
BAG_X = 0.30
IX, IY = 0.12, 0.09  # each compartment's inside, x by y
WALL_H = 0.06
SPEC_Y, DOC_Y = 0.05, -0.05  # compartment centers; the divider sits at y 0
ORANGE, WHITE, GRAY = (0.95, 0.45, 0.10), (0.92, 0.92, 0.92), (0.6, 0.6, 0.62)

HOLDER = (0.18, 0.08)  # one-tube holder center
POCKET, HW, HD = 0.030, 0.004, 0.035  # holder pocket inside, wall, depth
R_TUBE, H_TUBE = 0.008, 0.10
FORM_SIZE = (0.10, 0.06, 0.015)
FORM_POS = (0.17, -0.08)


def _bag():
    hx = IX / 2 + T / 2
    z, h = TOP + WALL_H / 2, WALL_H
    span = 2 * IY + 3 * T
    return [
        # specimen compartment: front, back and outer side orange, plus the shared divider
        Box("spec_front", pos=(BAG_X - hx, SPEC_Y + T / 4, z), size=(T, IY + 1.5 * T, h), color=ORANGE),
        Box("spec_back", pos=(BAG_X + hx, SPEC_Y + T / 4, z), size=(T, IY + 1.5 * T, h), color=ORANGE),
        Box("spec_side", pos=(BAG_X, SPEC_Y + IY / 2 + T / 2, z), size=(IX, T, h), color=ORANGE),
        Box("divider", pos=(BAG_X, 0.0, z), size=(IX, T, h), color=ORANGE),
        # document pocket: white
        Box("doc_front", pos=(BAG_X - hx, DOC_Y - T / 4, z), size=(T, IY + 1.5 * T, h), color=WHITE),
        Box("doc_back", pos=(BAG_X + hx, DOC_Y - T / 4, z), size=(T, IY + 1.5 * T, h), color=WHITE),
        Box("doc_side", pos=(BAG_X, DOC_Y - IY / 2 - T / 2, z), size=(IX, T, h), color=WHITE),
    ]


def _holder():
    cx, cy = HOLDER
    off = POCKET / 2 + HW / 2
    z = TOP + HD / 2
    return [
        Box("holder_front", pos=(cx - off, cy, z), size=(HW, POCKET + 2 * HW, HD), color=GRAY),
        Box("holder_back", pos=(cx + off, cy, z), size=(HW, POCKET + 2 * HW, HD), color=GRAY),
        Box("holder_left", pos=(cx, cy + off, z), size=(POCKET, HW, HD), color=GRAY),
        Box("holder_right", pos=(cx, cy - off, z), size=(POCKET, HW, HD), color=GRAY),
    ]


TUBE = Cylinder("blood_tube", pos=(HOLDER[0], HOLDER[1], TOP + H_TUBE / 2), radius=R_TUBE, height=H_TUBE,
                mass=0.012, color=(0.70, 0.55, 0.85))
FORM = Box("requisition_form", pos=(FORM_POS[0], FORM_POS[1], TOP + FORM_SIZE[2] / 2), size=FORM_SIZE,
           mass=0.03, color=(0.98, 0.98, 0.98))
OBJECTS = [*_bag(), *_holder(), TUBE, FORM]


def _in(s, cy):
    """Center inside the compartment's walls, bottom within 4 cm of the floor (on the table, or resting on
    the other item, never up on a 6 cm rim), and still."""
    return bool(
        abs(s.pos[0] - BAG_X) < IX / 2 and abs(s.pos[1] - cy) < IY / 2
        and s.low[2] < TOP + 0.04 and s.resting()
    )


def success(objects):
    """1.0: the tube in the specimen compartment and the form in the document pocket."""
    return _in(objects["blood_tube"], SPEC_Y) and _in(objects["requisition_form"], DOC_Y)


def score(objects):
    """0 if the tube is not in the specimen compartment (in the document pocket, still in its holder, on the
    table, held). With the tube in: 0.25 if the form is in the specimen compartment with it (paperwork
    exposed to the specimen); 0.5 if the form is anywhere else outside the bag (left on the table, on a rim,
    held). The 1.0 case is success()."""
    if not _in(objects["blood_tube"], SPEC_Y):
        return 0.0
    return 0.25 if _in(objects["requisition_form"], SPEC_Y) else 0.5


# -- demo -------------------------------------------------------------------------------------------------

YAW = np.pi / 2  # fingers close along x


def _tube_moves():
    """Lift the tube out of its holder by its upper body and lower it into the specimen compartment until its
    bottom is 5 mm above the floor, so it stands up before release."""
    grip_z = TOP + H_TUBE - 0.025 + 0.013
    set_z = TOP + 0.005 + H_TUBE - 0.025 + 0.013
    travel = TOP + 0.20
    a, b = HOLDER, (BAG_X, SPEC_Y)
    return [
        Waypoint(1.2, (*a, travel), FINGER_OPEN, "over the tube", YAW),
        Waypoint(1.2, (*a, grip_z), FINGER_OPEN, "descend", YAW),
        Waypoint(0.8, (*a, grip_z), 0.0, "close", YAW),
        Waypoint(1.2, (*a, travel), 0.0, "lift", YAW),
        Waypoint(1.5, (*b, travel), 0.0, "carry", YAW),
        Waypoint(1.5, (*b, set_z), 0.0, "lower into the compartment", YAW),
        Waypoint(0.6, (*b, set_z), FINGER_OPEN, "release", YAW),
        Waypoint(1.0, (*b, travel), FINGER_OPEN, "up", YAW),
        Waypoint(1.5, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "park"),
    ]


def demo():
    """Left arm: the tube into the specimen compartment, then parks. Right arm, after it: the form, gripped
    across its 6 cm width, into the document pocket, released 1.5 cm above the rim."""
    left = _tube_moves()
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "wait")]
    grip = (FORM_POS[0], FORM_POS[1], TOP + FORM_SIZE[2] - 0.010 + 0.013)
    drop = (BAG_X, DOC_Y, TOP + WALL_H + 0.015 + FORM_SIZE[2] - 0.010 + 0.013)
    right += pick_place_waypoints(grip, drop, travel_z=TOP + 0.18)
    right.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left, "right": right}
