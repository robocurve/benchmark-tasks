"""On the station table: a white sample box, taped down, with a flat blue pack of samples on its floor. Its sliding
lid stands open towards the right arm, with a grey handle on the lid's end. A red bolt on the near side of the box
slides across the end of the shut lid and locks it. On a steel tray in front of the left arm are two blocks of dry ice.

Run it:  inspect-robots run --embodiment yam_arena -E task=pack_dry_ice_into_a_sample_box.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, tool  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 130.0  # longer than the run limit: shutting the lid must not end the trial before the bolt is in reach
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Put the two blocks of dry ice in the sample box, slide the lid shut and push the bolt across to lock it, so the "
    "samples stay cold."
)

WHITE = (0.93, 0.93, 0.95)
BLOCK_WHITE = (0.98, 0.98, 0.99)
STEEL = (0.62, 0.64, 0.67)
GREY = (0.42, 0.43, 0.46)
BLUE = (0.20, 0.35, 0.65)
RED = (0.75, 0.08, 0.08)

# -- the box, in its own frame: origin at its centre, y along the lid's travel ------------------------------------------
BOX_XY = (0.25, 0.06)
BOX_SIZE = (0.14, 0.20, 0.12)
WALL = 0.008
FLOOR = 0.006
INNER_X = BOX_SIZE[0] - 2 * WALL
INNER_Y = BOX_SIZE[1] - 2 * WALL
INNER_H = BOX_SIZE[2] - FLOOR
RIM = BOX_SIZE[2] / 2  # above the box's centre

LID_T = 0.006
LID_L = BOX_SIZE[1]  # covers the whole opening; the seal on the rim is what the bolt closes
LID_GAP = 0.001
LID_TRAVEL = 0.15
LID_Z = RIM + LID_GAP + LID_T / 2
LID_SHUT_Y = (BOX_SIZE[1] - LID_L) / 2
HANDLE = (0.10, 0.015, 0.030)
LID_CLOSED_WITHIN = 0.010

BOLT = (0.06, 0.018, 0.020)
BOLT_TRAVEL = 0.03
BOLT_BACK = 0.010  # its tip starts this far short of the lid, so a nudge doesn't reach it
BOLT_ACROSS_OVER = 0.015  # over the lid by 5 mm
BOLT_Y = -0.04  # clear of the handle
BOLT_X = -BOX_SIZE[0] / 2 - BOLT[0] / 2 - BOLT_BACK
BOLT_Z = RIM + LID_GAP + LID_T + 0.001 + BOLT[2] / 2  # slides over the top of the lid
GUIDE = (BOLT[0] + BOLT_BACK, 0.03, 0.018)
GUIDE_TOP = BOLT_Z - BOLT[2] / 2
STAND_Y = -0.19  # holds up the open lid's far end

SAMPLE_BOX = tool(
    "sample_box",
    pos=(*BOX_XY, TOP + BOX_SIZE[2] / 2),
    boxes=[
        ((0.0, 0.0, -BOX_SIZE[2] / 2 + FLOOR / 2), (INNER_X, INNER_Y, FLOOR), BLUE),
        ((BOX_SIZE[0] / 2 - WALL / 2, 0.0, 0.0), (WALL, BOX_SIZE[1], BOX_SIZE[2]), WHITE),
        ((-BOX_SIZE[0] / 2 + WALL / 2, 0.0, 0.0), (WALL, BOX_SIZE[1], BOX_SIZE[2]), WHITE),
        ((0.0, BOX_SIZE[1] / 2 - WALL / 2, 0.0), (INNER_X, WALL, BOX_SIZE[2]), WHITE),
        ((0.0, -BOX_SIZE[1] / 2 + WALL / 2, 0.0), (INNER_X, WALL, BOX_SIZE[2]), WHITE),
        ((BOLT_X + BOLT_BACK / 2, BOLT_Y, GUIDE_TOP - GUIDE[2] / 2), GUIDE, GREY),
        ((0.0, STAND_Y, 0.0), (BOX_SIZE[0], 0.02, BOX_SIZE[2]), GREY),
    ],
    sliders=[
        # No springs: the lid and the bolt stay where they are pushed. The end state is what's scored.
        Slider("lid", rest=(0.0, LID_SHUT_Y - LID_TRAVEL, LID_Z), size=(BOX_SIZE[0], LID_L, LID_T), axis="Y",
               lower=0.0, upper=LID_TRAVEL, color=WHITE, rate=0.0, damping=8.0, mass=0.10,
               parts=(((0.0, -LID_L / 2 + HANDLE[1] / 2, LID_T / 2 + HANDLE[2] / 2), HANDLE, GREY),)),
        Slider("bolt", rest=(BOLT_X, BOLT_Y, BOLT_Z), size=BOLT, axis="X", lower=0.0, upper=BOLT_TRAVEL, color=RED,
               rate=0.0, damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

# -- the blocks and the tray ---------------------------------------------------------------------------------------
BLOCK = 0.04
BLOCK_MASS = 0.03
TRAY_XY = (0.25, 0.25)
TRAY_SIZE = (0.18, 0.09, 0.004)
BLOCK_XY = ((0.21, 0.25), (0.29, 0.25))
DROP_XY = ((0.25, 0.05), (0.25, 0.11))  # side by side, so one never lands on the other

OBJECTS = [
    Box("steel_tray", pos=(*TRAY_XY, TOP + TRAY_SIZE[2] / 2), size=TRAY_SIZE, color=STEEL),
    *(Box(f"block{i}", pos=(x, y, TOP + TRAY_SIZE[2] + BLOCK / 2), size=(BLOCK,) * 3, mass=BLOCK_MASS, color=BLOCK_WHITE)
      for i, (x, y) in enumerate(BLOCK_XY)),
    SAMPLE_BOX,
]

# -- scoring: the end state only, nothing latches -------------------------------------------------------------------
PER_BLOCK = 0.3
LID_SHUT = 0.4
UPRIGHT_DEG = 20.0


def in_box(block, box) -> bool:
    """The block's centre is inside the walls and below the rim of a box still standing."""
    x, y, z = box.rotation().T @ (block.pos - box.pos)
    return bool(abs(x) <= INNER_X / 2 and abs(y) <= INNER_Y / 2 and z < RIM and box.tilt_deg() <= UPRIGHT_DEG)


def lid_shut(box) -> bool:
    return box.sliders["lid"] >= LID_TRAVEL - LID_CLOSED_WITHIN


def bolt_across(box) -> bool:
    return box.sliders["bolt"] > BOLT_ACROSS_OVER


def grade(objects) -> float:
    """The score as the outcome table states it."""
    box = objects["sample_box"]
    inside = sum(in_box(objects[f"block{i}"], box) for i in range(2))
    shut = lid_shut(box)
    if inside and shut and bolt_across(box):
        return 0.0
    return round(PER_BLOCK * inside + (LID_SHUT if inside and shut else 0.0), 6)


def success(objects) -> bool:
    return grade(objects) >= 1.0 and all(objects[f"block{i}"].resting(max_speed=0.02, max_spin=1.0) for i in range(2))


def score(objects) -> float:
    return grade(objects)


def distance(objects) -> float:
    """Metres from the farther block to the box, plus what's left of the lid's travel."""
    box = objects["sample_box"]
    apart = max(float(np.linalg.norm(objects[f"block{i}"].pos[:2] - box.pos[:2])) for i in range(2))
    return apart + max(LID_TRAVEL - box.sliders["lid"], 0.0)


# -- the scripted demo: left arm drops the blocks in, right arm pushes the lid shut. It never goes near the bolt --------
GRIP_Z = TOP + TRAY_SIZE[2] + 0.024
TRAVEL_Z = TOP + 0.185
DROP_Z = TOP + 0.170  # the block's underside about 2.5 cm above the rim
PUSH_Z = TOP + BOX_SIZE[2] + LID_GAP + LID_T + 0.005 + 0.013  # finger tips 5 mm above the lid, against the handle
HOVER_Z = TOP + 0.185
FINGER_Y = 0.0065  # half the closed fingers' width along y, near the tips
OVERSHOOT = 0.006  # commanded past the stop, so the lid still shuts if the arm falls a little short
LID_END_OPEN_Y = BOX_XY[1] + LID_SHUT_Y - LID_TRAVEL - LID_L / 2
LID_END_SHUT_Y = LID_END_OPEN_Y + LID_TRAVEL
PUSH_START_Y = LID_END_OPEN_Y - 0.030
PUSH_END_Y = LID_END_SHUT_Y - FINGER_Y + OVERSHOOT
TRIM = {"pick": (0.0, 0.0), "drop": (0.0, 0.0), "push": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    plan: list[Waypoint] = []
    px, py = TRIM["pick"]
    qx, qy = TRIM["drop"]
    for (x, y), (dx, dy) in zip(BLOCK_XY, DROP_XY, strict=True):
        plan += pick_place_waypoints((x + px, y + py, GRIP_Z), (dx + qx, dy + qy, DROP_Z), travel_z=TRAVEL_Z, speed=0.7)
    plan.append(Waypoint(1.5, (*TRAY_XY, TRAVEL_Z), FINGER_OPEN, "clear of the box"))
    return plan


def _right_plan(start_after: float) -> list[Waypoint]:
    tx, ty = TRIM["push"]
    x = BOX_XY[0] + tx
    start, end = PUSH_START_Y + ty, PUSH_END_Y + ty
    return [
        Waypoint(start_after, (x, start, HOVER_Z), FINGER_CLOSED, "wait for the blocks"),
        Waypoint(1.5, (x, start, PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.5, (x, start, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(4.0, (x, end, PUSH_Z), FINGER_CLOSED, "push the lid shut"),
        Waypoint(0.8, (x, end, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, end, HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(1.5, (x, PUSH_START_Y, HOVER_Z), FINGER_CLOSED, "return"),
    ]


def demo():
    """Left arm: both blocks into the box. Right arm: once they're in, push the lid shut by its handle and go back.
    The bolt stays open."""
    left = _left_plan()
    return {"left": left, "right": _right_plan(sum(w.duration for w in left))}
