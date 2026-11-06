"""Two bundles of opened low-hydrogen E7018 welding rods (each a banded bundle 35 cm long, 2.4 cm across,
gray flux coating) lie on the bench in front of the arms, along y. Behind them, also along y: an open
wooden storage crate (brown, 40 x 8 cm inside, 6 cm walls) nearest the rods, and beyond it a rod holding oven
(orange, 40 x 8 cm inside, 12 cm walls, open top). Everything but the bundles is fixed.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the welding rods away."  # read by our run script, not the framework

PAD = 0.013  # the TCP sits this far above the finger pad tips
T = 0.01  # wall thickness
IN_X, IN_Y = 0.08, 0.40  # inside of both containers
BOXES = {  # name: (center x, wall height, color)
    "crate": (0.29, 0.06, (0.55, 0.38, 0.20)),
    "oven": (0.40, 0.12, (0.95, 0.50, 0.10)),
}


def _box(prefix, cx, h, color):
    hx, hy = IN_X / 2 + T / 2, IN_Y / 2 + T / 2
    return [
        Box(f"{prefix}_front", pos=(cx - hx, 0.0, TOP + h / 2), size=(T, IN_Y + 2 * T, h), color=color),
        Box(f"{prefix}_back", pos=(cx + hx, 0.0, TOP + h / 2), size=(T, IN_Y + 2 * T, h), color=color),
        Box(f"{prefix}_left", pos=(cx, hy, TOP + h / 2), size=(IN_X, T, h), color=color),
        Box(f"{prefix}_right", pos=(cx, -hy, TOP + h / 2), size=(IN_X, T, h), color=color),
    ]


R, L = 0.012, 0.35
GRAY = (0.45, 0.45, 0.42)
# Shapes can only be turned about the vertical, so a cylinder cannot lie down: each bundle is a box of the
# same footprint (2.4 x 35 x 2.4 cm).
BUNDLE_A = Box("bundle_a", pos=(0.10, 0.0, TOP + R), size=(2 * R, L, 2 * R), mass=0.30, color=GRAY)
BUNDLE_B = Box("bundle_b", pos=(0.19, 0.0, TOP + R), size=(2 * R, L, 2 * R), mass=0.30, color=GRAY)
OBJECTS = [*(p for name, (cx, h, c) in BOXES.items() for p in _box(name, cx, h, c)), BUNDLE_A, BUNDLE_B]
BUNDLES = ("bundle_a", "bundle_b")


def _where(s):
    """The container a bundle lies in (its whole footprint inside the walls, below the rim, still), or None."""
    if not s.resting():
        return None
    for name, (cx, h, _c) in BOXES.items():
        if (s.low[0] > cx - IN_X / 2 - 0.002 and s.high[0] < cx + IN_X / 2 + 0.002
                and s.low[1] > -IN_Y / 2 - 0.002 and s.high[1] < IN_Y / 2 + 0.002 and s.high[2] < TOP + h):
            return name
    return None


def success(objects):
    """1.0: both bundles in the rod oven."""
    return all(_where(objects[b]) == "oven" for b in BUNDLES)


def score(objects):
    """0.5 for each bundle in the oven. A bundle in the crate, on the bench, across a rim or held earns
    nothing: opened low-hydrogen rods left in open air pick up moisture."""
    return 0.5 * sum(_where(objects[b]) == "oven" for b in BUNDLES)


def demo():
    """Right arm, gripper turned 90 degrees so the fingers close across each bundle: lift it 24 cm, carry it
    over the crate and release it 2 cm above the oven's rim."""
    travel = TOP + 0.24
    out = []
    for b, dx in ((BUNDLE_B, 0.015), (BUNDLE_A, -0.015)):
        grip = (b.pos[0], 0.0, TOP + 0.006 + PAD)
        drop = (BOXES["oven"][0] + dx, 0.0, TOP + 0.12 + 0.02 + 2 * R - 0.006 + PAD)
        out += pick_place_waypoints(grip, drop, travel_z=travel, yaw=math.pi / 2)
    out.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"right": out}
