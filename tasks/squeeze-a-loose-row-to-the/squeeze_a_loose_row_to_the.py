"""Inventory racks in a data hall get pushed back together after a maintenance visit: the crates end up
spread out along the aisle and are jogged inward from both ends until they sit closed up and centred on the
taped mark. Here: five light grey crates (6 x 6 x 8 cm) stand in a loose row across the table, with a yellow
tape tick on each side of the row marking the centre. Push the two ends inward until the row is closed up
(every gap at most 1.5 cm) and its centre is within 1.5 cm of the mark. The left arm reaches only the left half
of the row and the right arm only the right half.
"""

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
GREY = (0.55, 0.57, 0.62)
TAPE = (0.95, 0.80, 0.10)
MAT = (0.20, 0.32, 0.48)

ROW_X = 0.35  # the row line, metres in front of the arm-base line
BOX = (0.06, 0.06, 0.08)
BOX_Y = [-0.25, -0.125, 0.0, 0.13, 0.25]  # starting centres along y (midline y = 0)
MARK_Y = 0.05  # centre of the taped mark
MASS = 0.1
GAP_FULL, CENTRE_FULL = 0.015, 0.015
GAP_HALF, CENTRE_HALF = 0.03, 0.04
LINE_TOL = 0.02

NAMES = [f"crate_{i + 1}" for i in range(5)]
TICK_H = 0.002


def _tick(name, x):
    return Box(name, pos=(x, MARK_Y, TOP + 0.003 + TICK_H / 2), size=(0.04, 0.012, TICK_H), color=TAPE)


OBJECTS = [
    Box("aisle_mat", pos=(ROW_X, 0.0, TOP + 0.0015), size=(0.22, 0.66, 0.003), color=MAT),
    _tick("tick_front", ROW_X - 0.085),
    _tick("tick_back", ROW_X + 0.085),
    *[Box(n, pos=(ROW_X, y, TOP + 0.003 + BOX[2] / 2), size=BOX, color=GREY, mass=MASS) for n, y in zip(NAMES, BOX_Y)],
]


def score(objects):
    """1.0: gaps <= 1.5 cm and centre within 1.5 cm of the mark; 0.5: gaps <= 3 cm and centre within 4 cm.
    Always needs every crate upright, on the table and within 2 cm of the row line."""
    boxes = sorted((objects[n] for n in NAMES), key=lambda o: float(o.pos[1]))
    for o in boxes:
        upright = 1 - 2 * (float(o.quat_xyzw[0]) ** 2 + float(o.quat_xyzw[1]) ** 2) > 0.99
        if not (upright and float(o.low[2]) < TOP + 0.01 and abs(float(o.pos[0]) - ROW_X) <= LINE_TOL):
            return 0.0
    gaps = [float(b.low[1]) - float(a.high[1]) for a, b in zip(boxes, boxes[1:])]
    centre = (float(boxes[0].low[1]) + float(boxes[-1].high[1])) / 2
    err = abs(centre - MARK_Y)
    if max(gaps) <= GAP_FULL and err <= CENTRE_FULL:
        return 1.0
    if max(gaps) <= GAP_HALF and err <= CENTRE_HALF:
        return 0.5
    return 0.0


PUSH_Z = TOP + 0.013 + 0.012  # pad tips 12 mm above the table
HOVER_Z = TOP + 0.16
YAW = 1.5708  # closed fingers side by side along x: a wide pushing face
PITCH = BOX[1]
END_L, END_R = MARK_Y + 2 * PITCH, MARK_Y - 2 * PITCH  # final centres of the two end crates
PARK = {"left": (0.16, 0.32, TOP + 0.20), "right": (0.12, -0.38, TOP + 0.20)}
FACE = 0.012  # TCP offset from the crate face while pushing (finger half thickness)


def _push(y_start, y_end, park):
    c = FINGER_CLOSED
    return [
        Waypoint(1.5, (ROW_X, y_start, HOVER_Z), c, "to the end", YAW),
        Waypoint(1.5, (ROW_X, y_start, PUSH_Z), c, "descend", YAW),
        Waypoint(0.6, (ROW_X, y_start, PUSH_Z), c, "settle", YAW),
        Waypoint(7.0, (ROW_X, y_end, PUSH_Z), c, "push inward", YAW),
        Waypoint(0.8, (ROW_X, y_end, PUSH_Z), c, "hold", YAW),
        Waypoint(1.0, (ROW_X, y_start, PUSH_Z), c, "back off", YAW),
        Waypoint(1.2, (ROW_X, y_start, HOVER_Z), c, "lift", YAW),
        Waypoint(2.5, park, FINGER_OPEN, "move clear"),
    ]


def demo():
    return {
        "left": _push(BOX_Y[-1] + 0.03 + FACE + 0.015, END_L + 0.03 + FACE - 0.022, PARK["left"]),
        "right": _push(BOX_Y[0] - 0.03 - FACE - 0.015, END_R - 0.03 - FACE + 0.037, PARK["right"]),
    }
