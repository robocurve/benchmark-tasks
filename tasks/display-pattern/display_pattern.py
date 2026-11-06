"""On the station table: a shop display shelf with a white back panel, a red header sign and a yellow price strip.
Seven product spots run along it from the robot's left to its right; the first five hold, in order, a soup can, a
ketchup bottle, a green beans can, a soup can and a ketchup bottle, and the last two are empty. In front of it, on a
cardboard stock mat, stand a ketchup bottle, a yogurt cup, a soup can, a mustard bottle and a green beans can.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SHELF_X, STOCK_X = 0.33, 0.15
SPOTS_Y = (0.27, 0.18, 0.09, 0.0, -0.09, -0.18, -0.27)

PRODUCTS = {  # type: (library asset, extent m, height m, mass kg)
    "soup": ("alphabet_soup_can_hope_robolab", 0.068, 0.080, 0.35),
    "ketchup": ("ketchup_bottle_hope_robolab", 0.075, 0.173, 0.45),
    "beans": ("green_beans_can_hope_robolab", 0.068, 0.055, 0.25),
    "mustard": ("mustard_bottle_hope_robolab", 0.07, 0.172, 0.40),
    "yogurt": ("yogurt_cup_hope_robolab", 0.07, 0.055, 0.15),
}
DISPLAY = {f"display_{i + 1}": (kind, (SHELF_X, SPOTS_Y[i])) for i, kind in enumerate(("soup", "ketchup", "beans", "soup", "ketchup"))}
STOCK = {
    "stock_ketchup": ("ketchup", (STOCK_X, 0.04)), "stock_yogurt": ("yogurt", (STOCK_X, -0.05)),
    "stock_soup": ("soup", (STOCK_X, -0.13)), "stock_mustard": ("mustard", (STOCK_X, -0.21)),
    "stock_beans": ("beans", (STOCK_X, -0.29)),
}
GAPS = {(SHELF_X, SPOTS_Y[5]): "beans", (SHELF_X, SPOTS_Y[6]): "soup"}
ITEMS = {**DISPLAY, **STOCK}

DRESSING = [
    Box("display_shelf", pos=(SHELF_X, 0.0, TOP + 0.0015), size=(0.11, 0.66, 0.003), color=(0.86, 0.76, 0.60)),
    Box("price_strip", pos=(SHELF_X - 0.0575, 0.0, TOP + 0.008), size=(0.004, 0.66, 0.016), color=(0.95, 0.85, 0.20)),
    Box("back_panel", pos=(SHELF_X + 0.065, 0.0, TOP + 0.11), size=(0.01, 0.66, 0.22), color=(0.92, 0.92, 0.90)),
    Box("header_sign", pos=(SHELF_X + 0.059, 0.0, TOP + 0.20), size=(0.004, 0.30, 0.05), color=(0.75, 0.10, 0.10)),
    Box("stock_mat", pos=(STOCK_X, -0.125, TOP + 0.0015), size=(0.09, 0.36, 0.003), color=(0.65, 0.50, 0.32)),
]
OBJECTS = [
    *DRESSING,
    *(Asset(n, PRODUCTS[k][0], xy=xy, extent=PRODUCTS[k][1], mass=PRODUCTS[k][3]) for n, (k, xy) in ITEMS.items()),
]
_disturbed = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _moved(objects, name):
    return float(np.linalg.norm(objects[name].pos[:2] - np.array(ITEMS[name][1])))


def _filled(objects, spot, kind):
    return any(
        ITEMS[n][0] == kind and np.linalg.norm(o.pos[:2] - spot) < 0.03 and _tilt_deg(o) < 15 and o.resting()
        for n, o in ((n, objects[n]) for n in ITEMS)
    )


def _track(objects):
    """Remember, for the whole trial, whether a product tipped over or the display was knocked."""
    global _disturbed
    if all(_moved(objects, n) < 0.005 for n in ITEMS):
        _disturbed = False
    tipped = any(_tilt_deg(objects[n]) > 45 for n in ITEMS)
    _disturbed = _disturbed or tipped or any(_moved(objects, n) > 0.02 for n in DISPLAY)


def score(objects):
    """0 if a product ever tipped over or a display product was ever moved more than 2 cm. Otherwise 0.5 for each
    empty spot holding the product that continues the pattern (green beans, then soup), upright and within 3 cm."""
    _track(objects)
    if _disturbed:
        return 0.0
    return 0.5 * sum(_filled(objects, np.array(spot), kind) for spot, kind in GAPS.items())


def success(objects):
    return score(objects) == 1.0


def demo():
    """Right arm, fingers closing front to back: green beans into the sixth spot, then soup into the seventh. Each
    item is lifted, moved out past the stock row, then carried across, so it never passes a tall bottle."""
    travel, clear_x, yaw = TOP + 0.16, 0.26, np.pi / 2

    def move(name, spot):
        kind, (x, y) = STOCK[name]
        grip = TOP + 0.013 + PRODUCTS[kind][2] / 2
        sx, sy = spot
        steps = [
            (1.5, (x, y, travel), FINGER_OPEN, "above the item"),
            (1.5, (x, y, grip), FINGER_OPEN, "descend"),
            (0.8, (x, y, grip), FINGER_CLOSED, "close"),
            (1.2, (x, y, travel), FINGER_CLOSED, "lift"),
            (1.0, (clear_x, y, travel), FINGER_CLOSED, "out past the stock"),
            (1.5, (sx, sy, travel), FINGER_CLOSED, "over the spot"),
            (1.2, (sx, sy, grip + 0.005), FINGER_CLOSED, "lower"),
            (0.6, (sx, sy, grip + 0.005), FINGER_OPEN, "release"),
            (1.0, (sx, sy, travel), FINGER_OPEN, "retreat"),
        ]
        return [Waypoint(d, p, f, label, yaw) for d, p, f, label in steps]

    (gap6, _), (gap7, _) = GAPS.items()
    return {"right": [*move("stock_beans", gap6), *move("stock_soup", gap7)]}
