"""On the station table, two packing orders, each with its own open-topped box (fixed walls 34 mm tall, no lid)
and its own flag pad. Sizes are front to back by across. The blue order, in front of the left arm: three loose
blue wooden blocks, each 30 mm thick, lying flat (90 x 60 mm, 60 x 60 mm and 140 x 35 mm), and the blue box,
170 x 115 mm inside. The orange order, in front of the right arm: four loose orange wooden blocks, each 30 mm
thick, lying flat (two 60 x 80 mm, one 50 x 70 mm, one 60 x 60 mm), and the orange box, 150 x 100 mm inside. Between the boxes are a blue flag pad and an orange flag pad (80 mm squares), and nearer the robot a red
card (a 60 x 40 x 25 mm block).
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
WALL_T, WALL_H = 0.005, 0.034
RIM_Z = TOP + WALL_H
BLUE, ORANGE = (0.20, 0.38, 0.80), (0.95, 0.52, 0.12)
BLUE_WALL, ORANGE_WALL = (0.30, 0.45, 0.80), (0.93, 0.55, 0.20)
BOXES = {  # name: (centre x, centre y, inside x, inside y, colour)
    "blue": (0.330, 0.200, 0.170, 0.115, BLUE_WALL),
    "orange": (0.330, -0.200, 0.150, 0.100, ORANGE_WALL),
}


def _walls(name, cx, cy, ix, iy, colour):
    ox, oy = ix / 2 + WALL_T / 2, iy / 2 + WALL_T / 2
    lx, ly = ix + 2 * WALL_T, iy + 2 * WALL_T
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_box_near", pos=(cx - ox, cy, z), size=(WALL_T, ly, WALL_H), color=colour),
        Box(f"{name}_box_far", pos=(cx + ox, cy, z), size=(WALL_T, ly, WALL_H), color=colour),
        Box(f"{name}_box_right", pos=(cx, cy - oy, z), size=(lx, WALL_T, WALL_H), color=colour),
        Box(f"{name}_box_left", pos=(cx, cy + oy, z), size=(lx, WALL_T, WALL_H), color=colour),
    ]


WALLS = [w for n, (cx, cy, ix, iy, c) in BOXES.items() for w in _walls(n, cx, cy, ix, iy, c)]

T = 0.030  # every block is 30 mm thick
WOOD_DENSITY = 600.0  # kg/m^3
# name: (x size, y size, start centre x, start centre y)
BLUE_SPEC = {
    "blue_a": (0.090, 0.060, 0.170, 0.240),
    "blue_b": (0.060, 0.060, 0.170, 0.355),
    "blue_c": (0.140, 0.035, 0.175, 0.115),
}
ORANGE_SPEC = {
    "orange_a": (0.060, 0.080, 0.160, -0.100),
    "orange_b": (0.060, 0.080, 0.160, -0.205),
    "orange_c": (0.050, 0.070, 0.160, -0.300),
    "orange_d": (0.060, 0.060, 0.245, -0.355),
}


def _block(name, spec, colour):
    sx, sy, x, y = spec
    return Box(name, pos=(x, y, TOP + T / 2), size=(sx, sy, T), mass=round(WOOD_DENSITY * sx * sy * T, 4), color=colour)


BLOCKS = [_block(n, s, BLUE) for n, s in BLUE_SPEC.items()] + [_block(n, s, ORANGE) for n, s in ORANGE_SPEC.items()]
SIZE = {b.name: b.size for b in BLOCKS}

PAD_SIZE = (0.080, 0.080, 0.002)
PADS = {"blue": (0.340, 0.055), "orange": (0.340, -0.055)}
PAD_TOP = TOP + PAD_SIZE[2]
PAD_BOXES = [Box(f"{n}_flag_pad", pos=(x, y, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=BLUE if n == "blue" else ORANGE)
             for n, (x, y) in PADS.items()]
CARD_SIZE = (0.060, 0.040, 0.025)
CARD = Box("red_card", pos=(0.230, 0.0, TOP + CARD_SIZE[2] / 2), size=CARD_SIZE, mass=0.070,
           color=(0.85, 0.10, 0.10))  # solid plastic, about 1,170 kg/m^3

OBJECTS = [*WALLS, *BLOCKS, *PAD_BOXES, CARD]
NAMES = {"blue": list(BLUE_SPEC), "orange": list(ORANGE_SPEC)}

ON_TOL = 0.001  # "resting on" a surface: lowest point within 1 mm of it
FLAT_DEG = 5.0  # "lying flat": its 30 mm axis within 5 degrees of vertical
EPS = 1e-6  # stated limits are inclusive


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _flat(st) -> bool:
    """The block's own thickness axis (its local z) within 5 degrees of vertical, either way up."""
    return bool(np.degrees(np.arccos(min(1.0, abs(_rot(st)[2, 2])))) <= FLAT_DEG + EPS)


def _over_box(st, box) -> bool:
    """Centre over the box's outer footprint (walls included), seen from above."""
    cx, cy, ix, iy, _ = BOXES[box]
    return bool(abs(st.pos[0] - cx) <= ix / 2 + WALL_T + EPS and abs(st.pos[1] - cy) <= iy / 2 + WALL_T + EPS)


def _on_floor(st) -> bool:
    return bool(abs(st.low[2] - TOP) <= ON_TOL + EPS and st.resting())


def _packed(st, box) -> bool:
    """In the box, lying flat on the floor, below the rim, at rest."""
    return bool(_over_box(st, box) and _on_floor(st) and _flat(st) and st.high[2] <= RIM_Z + EPS)


def _outside_flat(st) -> bool:
    """Lying flat on the table (or on a flag pad, which counts as table), outside both boxes, at rest."""
    on = abs(st.low[2] - TOP) <= ON_TOL + EPS or abs(st.low[2] - PAD_TOP) <= ON_TOL + EPS
    return bool(on and st.resting() and _flat(st) and not _over_box(st, "blue") and not _over_box(st, "orange"))


def card_on(objects, pad: str) -> bool:
    c = objects["red_card"]
    px, py = PADS[pad]
    return bool(
        abs(c.pos[0] - px) <= PAD_SIZE[0] / 2 + EPS and abs(c.pos[1] - py) <= PAD_SIZE[1] / 2 + EPS
        and abs(c.low[2] - PAD_TOP) <= ON_TOL + EPS and c.resting()
    )


def blue_half(objects) -> float:
    """Up to 0.5: the blue order fits, so it must be packed. 0 if it was flagged."""
    if card_on(objects, "blue"):
        return 0.0
    blocks = [objects[n] for n in NAMES["blue"]]
    return 0.5 * sum(_packed(s, "blue") for s in blocks) / len(blocks)


def orange_half(objects) -> float:
    """Up to 0.5: the orange order cannot fit, so it must be flagged with none of it packed."""
    if not card_on(objects, "orange"):
        return 0.0
    blocks = [objects[n] for n in NAMES["orange"]]
    if all(_outside_flat(s) for s in blocks):
        return 0.5
    if all(_outside_flat(s) or _packed(s, "orange") for s in blocks):
        return 0.25
    return 0.0


def score(objects):
    return blue_half(objects) + orange_half(objects)


def success(objects):
    return score(objects) >= 1.0 - EPS


def distance(objects):
    return float(1.0 - score(objects))


# Demo. Left arm: the three blue blocks into the blue box, each let go with its finger tips above the rim (the
# block drops about 31 mm onto the box floor), so the opening fingers clear the walls. Right arm: the card onto
# the orange pad.
TIP_PAST_TCP = 0.013
TIP_ABOVE_BOTTOM = 0.006
GRIP_DZ = TIP_PAST_TCP + TIP_ABOVE_BOTTOM
TRAVEL = TOP + 0.11
_bx, _by, _bix, _biy, _ = BOXES["blue"]
_x0, _y0 = _bx - _bix / 2, _by - _biy / 2  # inside near-right corner of the blue box
TARGET = {  # packed centres: a and b side by side along the left wall, c along the right wall
    "blue_a": (_x0 + 0.007 + 0.045, _y0 + _biy - 0.006 - 0.030),
    "blue_b": (_x0 + _bix - 0.007 - 0.030, _y0 + _biy - 0.006 - 0.030),
    "blue_c": (_x0 + _bix / 2, _y0 + 0.007 + 0.0175),
}
RELEASE_BOTTOM = RIM_Z + 0.003 - TIP_ABOVE_BOTTOM  # finger tips 3 mm above the rim at release


def _move(px, py, qx, qy, pick_bottom, place_bottom, label):
    return [
        Waypoint(1.2, (px, py, TRAVEL), FINGER_OPEN, f"over {label}"),
        Waypoint(1.0, (px, py, pick_bottom + GRIP_DZ), FINGER_OPEN, "descend"),
        Waypoint(0.6, (px, py, pick_bottom + GRIP_DZ), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (px, py, TRAVEL), FINGER_CLOSED, "lift"),
        Waypoint(1.4, (qx, qy, TRAVEL), FINGER_CLOSED, "carry"),
        Waypoint(1.0, (qx, qy, place_bottom + GRIP_DZ), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (qx, qy, place_bottom + GRIP_DZ), FINGER_OPEN, "release"),
        Waypoint(0.8, (qx, qy, TRAVEL), FINGER_OPEN, "up"),
    ]


def demo():
    left = []
    for n in ("blue_c", "blue_a", "blue_b"):
        _, _, x, y = BLUE_SPEC[n]
        left += _move(x, y, *TARGET[n], TOP, RELEASE_BOTTOM, n)
    left.append(Waypoint(2.0, left[-1].pos, FINGER_OPEN, "wait"))
    cx, cy, _ = CARD.pos
    px, py = PADS["orange"]
    right = [Waypoint(4.0, (cx, cy, TRAVEL), FINGER_OPEN, "wait for the left arm to clear")]
    right += _move(cx, cy, px, py, TOP, PAD_TOP + 0.006, "the card")
    return {"left": left, "right": right}
