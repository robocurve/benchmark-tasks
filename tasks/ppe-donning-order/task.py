"""An isolation cart's gray top stands across the back of the table, divided by low rails into four slots in
a row; a red marker block at its left end (the robot's left) marks slot 1. In front of the cart, on the
table, lie four folded PPE items in mixed order, from the robot's left to right: a surgical mask (white,
6 x 4 x 2 cm), an isolation gown (yellow, folded, 7 x 7 x 3 cm), a box of nitrile gloves (purple,
6 x 4 x 3 cm) and safety goggles (dark gray, 3 x 7 x 3 cm).
"""

from itertools import combinations

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Lay out the PPE on the cart in the order it goes on."

CART_X = 0.31
CART_TOP = TOP + 0.01  # a 1 cm gray board
SLOT_IX, SLOT_IY = 0.10, 0.09  # slot inside, x by y
RAIL_W, RAIL_H = 0.01, 0.015
PITCH = SLOT_IY + RAIL_W
SLOT_Y = [1.5 * PITCH, 0.5 * PITCH, -0.5 * PITCH, -1.5 * PITCH]  # slot 1 (robot's left) .. slot 4
GRAY = (0.62, 0.62, 0.65)

ITEMS = {  # name: (size x, y, z; color)
    "gown": ((0.07, 0.07, 0.03), (0.95, 0.85, 0.20)),
    "mask": ((0.06, 0.04, 0.02), (0.96, 0.96, 0.96)),
    "goggles": ((0.03, 0.07, 0.03), (0.25, 0.25, 0.28)),
    "gloves": ((0.06, 0.04, 0.03), (0.55, 0.30, 0.80)),
}
ORDER = ["gown", "mask", "goggles", "gloves"]  # CDC donning sequence: slot 1 .. 4
START_X = 0.17
START = {"mask": SLOT_Y[0], "gown": SLOT_Y[1], "gloves": SLOT_Y[2], "goggles": SLOT_Y[3]}  # mixed order


def _cart():
    span = 4 * PITCH + RAIL_W
    parts = [Box("cart_top", pos=(CART_X, 0.0, TOP + 0.005), size=(SLOT_IX + 2 * RAIL_W, span, 0.01), color=GRAY)]
    off = SLOT_IX / 2 + RAIL_W / 2
    z = CART_TOP + RAIL_H / 2
    parts.append(Box("cart_rail_front", pos=(CART_X - off, 0.0, z), size=(RAIL_W, span, RAIL_H), color=GRAY))
    parts.append(Box("cart_rail_back", pos=(CART_X + off, 0.0, z), size=(RAIL_W, span, RAIL_H), color=GRAY))
    for i in range(5):
        y = 2 * PITCH - i * PITCH
        parts.append(Box(f"cart_div{i}", pos=(CART_X, y, z), size=(SLOT_IX, RAIL_W, RAIL_H), color=GRAY))
    return parts


MARKER = Box("slot1_marker", pos=(CART_X, 2 * PITCH + 0.02, TOP + 0.015), size=(0.03, 0.03, 0.03),
             color=(0.85, 0.10, 0.10))
PPE = [
    Box(name, pos=(START_X, START[name], TOP + size[2] / 2), size=size, mass=0.05, color=color)
    for name, (size, color) in ITEMS.items()
]
OBJECTS = [*_cart(), MARKER, *PPE]


def _slot(s):
    """Which slot (0..3) the item lies in, or None. In a slot: its center inside the slot's rails, its bottom
    on the cart top (not stacked on another item), and still."""
    if not s.resting() or s.low[2] > CART_TOP + 0.01 or abs(s.pos[0] - CART_X) > SLOT_IX / 2:
        return None
    for i, y in enumerate(SLOT_Y):
        if abs(s.pos[1] - y) < SLOT_IY / 2:
            return i
    return None


def _slots(objects):
    return {name: _slot(objects[name]) for name in ORDER}


def success(objects):
    """1.0: gown in slot 1, mask in 2, goggles in 3, gloves in 4 (slot 1 at the red marker, robot's left)."""
    slots = _slots(objects)
    return all(slots[name] == i for i, name in enumerate(ORDER))


def score(objects):
    """0 whenever any item is in the wrong slot (out of the donning order). Otherwise 0.25 for each item in
    its own slot; an item on the table, on a rail, stacked or held earns nothing. All four right is 1.0."""
    slots = _slots(objects)
    if any(slots[name] is not None and slots[name] != i for i, name in enumerate(ORDER)):
        return 0.0
    return 0.25 * sum(slots[name] == i for i, name in enumerate(ORDER))


# -- demo -------------------------------------------------------------------------------------------------

TRAVEL_Z = TOP + 0.15


def _move(name, dst):
    """Grip the item 1.2 cm below its top across y (fingers close along y; the goggles across x), release it
    1.5 cm above the cart top in its slot."""
    size = ITEMS[name][0]
    yaw = 1.5708 if size[1] > 0.06 else 0.0  # close across the narrower side
    grip = (START_X, START[name], TOP + size[2] - 0.012 + 0.013)
    place = (CART_X, SLOT_Y[dst], CART_TOP + 0.015 + size[2] - 0.012 + 0.013)
    return pick_place_waypoints(grip, place, travel_z=TRAVEL_Z, yaw=yaw)


def demo():
    """Left arm: gown to slot 1, mask to slot 2, then parks. Right arm, after it: goggles to slot 3, gloves
    to slot 4."""
    left = _move("gown", 0) + _move("mask", 1)
    left.append(Waypoint(1.5, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "park"))
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "wait")]
    right += _move("goggles", 2) + _move("gloves", 3)
    return {"left": left, "right": right}
