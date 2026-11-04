"""On the station table, between the arms: a gravity-conveyor section, two grey steel side rails 30 cm apart with open
U-slots in their tops, holding two rollers (4.8 cm across, 26 cm long, on 12 mm axle stubs that rest in the slots): a
grey one nearer the arms and a worn red one beyond it. In front of the rails, a new grey roller rests in a spare stand with the
same U-slots; past the rails, a green mat lies on the table. The goal is to replace the worn roller: lift the red roller out, lay it on the
green mat, and drop the new roller into the red one's slots, leaving the other roller in place.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Replace the worn red roller: put it on the green mat and drop the new roller into its slots." \\
           --scorer task_score ...
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import rigid_part, to_world  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, GREY, RED, GREEN = (0.45, 0.47, 0.50), (0.68, 0.70, 0.72), (0.80, 0.15, 0.12), (0.20, 0.55, 0.25)

# The rails: along x, at y = +-RAIL_Y, with a U-slot for each roller position.
RAIL_Y, RAIL_T, RAIL_H = 0.15, 0.02, 0.08
RAIL_X0, RAIL_X1 = 0.16, 0.33
SLOT_W, SLOT_D = 0.022, 0.025  # 22 mm for a 12 mm axle stub: 5 mm of play each way
SLOTS = (0.20, 0.30)  # the grey roller's, then the worn roller's
SLOT_BOTTOM = TOP + RAIL_H - SLOT_D

# A roller: its frame's origin at its centre, axis along y.
ROLLER_R, ROLLER_L = 0.024, 0.26
STUB_R, STUB_L = 0.006, 0.042
STUB_Y = ROLLER_L / 2 + STUB_L / 2  # 0.151: the stubs' centres, resting in the rails' slots
SEATED_Z = SLOT_BOTTOM + STUB_R  # the roller's centre height with its stubs on the slot bottoms
ROLLER_MASS = 0.4


def roller(name, pos, color):
    return rigid_part(
        name,
        pos=pos,
        cylinders=[((0.0, 0.0, 0.0), ROLLER_R, ROLLER_L, color, "Y"),
                   ((0.0, STUB_Y, 0.0), STUB_R, STUB_L, STEEL, "Y"),
                   ((0.0, -STUB_Y, 0.0), STUB_R, STUB_L, STEEL, "Y")],
        mass=ROLLER_MASS,
    )


def rail_boxes(y, side):
    """One rail as fixed boxes: a solid lower part, and an upper part with a gap at each slot."""
    lower_h = RAIL_H - SLOT_D
    boxes = [Box(f"rail_{side}_base", pos=((RAIL_X0 + RAIL_X1) / 2, y, TOP + lower_h / 2),
                 size=(RAIL_X1 - RAIL_X0, RAIL_T, lower_h), color=STEEL)]
    edges = [RAIL_X0, *[e for x in SLOTS for e in (x - SLOT_W / 2, x + SLOT_W / 2)], RAIL_X1]
    for i, (x0, x1) in enumerate(zip(edges[::2], edges[1::2])):
        boxes.append(Box(f"rail_{side}_top_{i}", pos=((x0 + x1) / 2, y, TOP + lower_h + SLOT_D / 2),
                         size=(x1 - x0, RAIL_T, SLOT_D), color=STEEL))
    return boxes


SPARE_X = 0.11
MAT = Box("green_mat", pos=(0.40, 0.0, TOP + 0.0005), size=(0.12, 0.30, 0.001), color=GREEN)


def spare_stand(y, side):
    """A spare-roller stand in front of the rails: a short post with the same U-slot as the rails."""
    lower_h = RAIL_H - SLOT_D
    x0, x1 = SPARE_X - 0.025, SPARE_X + 0.025
    return [
        Box(f"stand_{side}_base", pos=(SPARE_X, y, TOP + lower_h / 2), size=(x1 - x0, RAIL_T, lower_h), color=STEEL),
        Box(f"stand_{side}_front", pos=((x0 + SPARE_X - SLOT_W / 2) / 2, y, TOP + lower_h + SLOT_D / 2),
            size=(SPARE_X - SLOT_W / 2 - x0, RAIL_T, SLOT_D), color=STEEL),
        Box(f"stand_{side}_back", pos=((x1 + SPARE_X + SLOT_W / 2) / 2, y, TOP + lower_h + SLOT_D / 2),
            size=(x1 - SPARE_X - SLOT_W / 2, RAIL_T, SLOT_D), color=STEEL),
    ]


OBJECTS = [
    *rail_boxes(RAIL_Y, "left"),
    *rail_boxes(-RAIL_Y, "right"),
    roller("grey_roller", (SLOTS[0], 0.0, SEATED_Z), GREY),
    roller("worn_roller", (SLOTS[1], 0.0, SEATED_Z), RED),
    roller("new_roller", (SPARE_X, 0.0, SEATED_Z), GREY),
    *spare_stand(RAIL_Y, "left"),
    *spare_stand(-RAIL_Y, "right"),
    MAT,
]


def seated(state, slot_x) -> bool:
    """Both axle stubs rest in the slots at ``slot_x``: within the slot's width and 6 mm of the slot bottom."""
    for y in (STUB_Y, -STUB_Y):
        stub = to_world(state, (0.0, y, 0.0))
        if not (abs(stub[0] - slot_x) < SLOT_W / 2 and abs(stub[2] - SEATED_Z) < 0.006 and abs(abs(stub[1]) - STUB_Y) < 0.02):
            return False
    return True


def on_mat(state) -> bool:
    """Lying on the green mat: its centre over the mat and at most 1.5 cm above lying flat on the table."""
    hx, hy = MAT.size[0] / 2, MAT.size[1] / 2
    return bool(abs(state.pos[0] - MAT.pos[0]) < hx and abs(state.pos[1] - MAT.pos[1]) < hy
                and state.pos[2] < TOP + ROLLER_R + 0.015)


def success(objects):
    """Replaced: the worn roller on the mat, the new one seated in its slots, the grey roller still in its own, all
    still; must hold for 1 s."""
    grey, worn, new = objects["grey_roller"], objects["worn_roller"], objects["new_roller"]
    return bool(on_mat(worn) and seated(new, SLOTS[1]) and seated(grey, SLOTS[0])
                and grey.resting() and worn.resting() and new.resting())


def score(objects):
    """Partial credit (success scores 1.0): 0.4 for the worn roller on the mat, plus 0.4 for the new roller seated in the
    worn one's slots; 0 if the grey roller is out of its slots."""
    if not seated(objects["grey_roller"], SLOTS[0]):
        return 0.0
    return 0.4 * on_mat(objects["worn_roller"]) + 0.4 * seated(objects["new_roller"], SLOTS[1])


# How far each roller sits from where the demo grips it, as measured in a traced run (closing the fingers shifts it a
# few millimetres, the same way every run with -E object_jitter=0).
GRIP_SHIFT = (0.0, 0.0)  # roller centre minus its intended position while held (x, z; m)


def _half(side: float) -> list[Waypoint]:
    """One arm's half of the two-arm lifts: ``side`` +1 for the left arm (gripping 9 cm left of the roller's centre) and
    -1 for the right; gripping far apart keeps the roller from twisting between the hands. The fingers close along x, across the roller; they open only 61 mm so they clear the next roller."""
    y = side * 0.09
    yaw = -side * np.pi / 2  # the hand turned this way keeps every waypoint well inside the arm's reach
    part = FINGER_OPEN * 0.65
    dx, dz = GRIP_SHIFT
    worn_x, spare_x, travel = SLOTS[1], SPARE_X, TOP + 0.12
    mat_z = TOP + ROLLER_R + 0.002
    return [
        Waypoint(1.5, (worn_x, y, TOP + 0.14), part, "above the worn roller", yaw),
        Waypoint(1.5, (worn_x, y, SEATED_Z), part, "down to it", yaw),
        Waypoint(0.8, (worn_x, y, SEATED_Z), FINGER_CLOSED, "grip", yaw),
        Waypoint(1.2, (worn_x, y, travel), FINGER_CLOSED, "lift it out of the slots", yaw),
        Waypoint(1.5, (RAIL_X1 + 0.04, y, travel), FINGER_CLOSED, "carry it past the rails' ends", yaw),
        Waypoint(1.2, (MAT.pos[0] - dx, y, mat_z - dz), FINGER_CLOSED, "lay it down", yaw),
        Waypoint(0.6, (MAT.pos[0] - dx, y, mat_z - dz), part, "let go", yaw),
        Waypoint(1.0, (MAT.pos[0], y, travel), part, "up", yaw),
        Waypoint(2.0, (spare_x, y, travel), part, "over the new roller", yaw),
        Waypoint(1.5, (spare_x, y, SEATED_Z), part, "down to it", yaw),
        Waypoint(0.8, (spare_x, y, SEATED_Z), FINGER_CLOSED, "grip", yaw),
        Waypoint(0.5, (spare_x, y, SEATED_Z), FINGER_CLOSED, "let the grip settle", yaw),
        Waypoint(2.0, (spare_x, y, travel), FINGER_CLOSED, "lift it out of its stand, slowly", yaw),
        Waypoint(2.5, (worn_x - dx, y, travel), FINGER_CLOSED, "carry it over the empty slots", yaw),
        Waypoint(1.5, (worn_x - dx, y, SEATED_Z + 0.004 - dz), FINGER_CLOSED, "lower it in", yaw),
        Waypoint(0.6, (worn_x - dx, y, SEATED_Z + 0.004 - dz), part, "let go", yaw),
        Waypoint(1.0, (worn_x, y, TOP + 0.14), part, "back up", yaw),
    ]


def demo():
    """Both arms together, one at each end of the roller: lift the worn roller out of its slots, lay it on the mat, then
    take the new roller out of its stand and lower its axle stubs into the empty slots. The arms move in step because
    both plans use the same waypoint timings."""
    return {"left": _half(+1.0), "right": _half(-1.0)}
