"""A raised-floor data-centre model (about 1:8 scale). A row of server racks (a fixed black block, 26 cm long,
6 cm wide, 8 cm tall) runs away from the arms down the middle of the table. To its right is the cold aisle,
marked by a blue tape strip along its outer edge; to its left the hot aisle, marked by a red tape strip. Each
aisle is one column of three fixed grey floor tiles (8.6 x 8.6 x 1.5 cm on a 9.4 cm pitch); the middle tile
of each column is lifted out, leaving an open gap. On a grey cart at the near edge lie the two loose tiles: a
perforated tile (dark grey) on the left, in front of the hot aisle, and a solid tile (light grey) on the
right, in front of the cold aisle.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the two floor tiles back."  # read by our run script, not the framework

GREY = (0.62, 0.62, 0.64)
TILE, PITCH, TH = 0.086, 0.094, 0.015  # fixed floor tile edge, pitch, thickness
ROW_X = [0.21, 0.21 + PITCH, 0.21 + 2 * PITCH]  # tile centres along each aisle; the middle one is the gap
GAP_X = ROW_X[1]
COLD_Y, HOT_Y = -0.10, 0.10  # cold aisle on the robot's right, hot aisle on its left
GAP_HALF = PITCH - TILE / 2  # free half-span between the neighbouring tiles' edges (5.1 cm)

LOOSE = 0.070  # loose tiles: 7 x 7 x 1.5 cm (clearance for the gripper)
CART_X = 0.115
CART = Box("cart", pos=(CART_X, 0.0, TOP + 0.01), size=(0.10, 0.26, 0.02), color=(0.45, 0.45, 0.47))
PERF = Box("perforated_tile", pos=(CART_X, 0.065, TOP + 0.02 + TH / 2), size=(LOOSE, LOOSE, TH), mass=0.08,
           color=(0.22, 0.22, 0.24))
SOLID = Box("solid_tile", pos=(CART_X, -0.065, TOP + 0.02 + TH / 2), size=(LOOSE, LOOSE, TH), mass=0.08,
            color=(0.82, 0.82, 0.84))

FLOOR = [
    Box(f"{side}_tile{i}", pos=(x, y, TOP + TH / 2), size=(TILE, TILE, TH), color=GREY)
    for side, y in (("cold", COLD_Y), ("hot", HOT_Y))
    for i, x in enumerate(ROW_X) if i != 1
]
RACKS = Box("rack_row", pos=(0.31, 0.0, TOP + 0.04), size=(0.26, 0.06, 0.08), color=(0.08, 0.08, 0.09))
TAPE_COLD = Box("cold_aisle_tape", pos=(0.30, COLD_Y - 0.06, TOP + 0.0015), size=(0.28, 0.02, 0.003),
                color=(0.15, 0.35, 0.90))
TAPE_HOT = Box("hot_aisle_tape", pos=(0.30, HOT_Y + 0.06, TOP + 0.0015), size=(0.28, 0.02, 0.003),
               color=(0.85, 0.12, 0.10))
OBJECTS = [CART, *FLOOR, RACKS, TAPE_COLD, TAPE_HOT, PERF, SOLID]


def _flat(s, deg=10.0):
    qx, qy, _qz, _qw = s.quat_xyzw
    return 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(deg))


def _in_gap(s, gy):
    """Seated in the gap of the aisle at y = gy: lying flat and still on the floor (the table) with its whole
    footprint inside the gap square (between the neighbouring tiles' edges, 10.2 cm square)."""
    return bool(
        s.resting() and _flat(s) and s.low[2] < TOP + 0.006
        and s.low[0] > GAP_X - GAP_HALF and s.high[0] < GAP_X + GAP_HALF
        and s.low[1] > gy - GAP_HALF and s.high[1] < gy + GAP_HALF
    )


def success(objects):
    """1.0: the perforated tile in the cold-aisle gap and the solid tile in the hot-aisle gap."""
    return _in_gap(objects["perforated_tile"], COLD_Y) and _in_gap(objects["solid_tile"], HOT_Y)


def score(objects):
    """0 whenever the perforated tile is in the hot-aisle gap (it would blow cold air into the exhaust), and
    0 whenever the solid tile blocks the cold-aisle gap. Otherwise: 0.5 if the perforated tile is in the
    cold-aisle gap (the solid tile not in the hot-aisle gap); 0.25 if only the solid tile is in the hot-aisle
    gap; 0 if neither gap is filled."""
    perf, solid = objects["perforated_tile"], objects["solid_tile"]
    if _in_gap(perf, HOT_Y) or _in_gap(solid, COLD_Y):
        return 0.0
    if _in_gap(perf, COLD_Y):
        return 0.5
    if _in_gap(solid, HOT_Y):
        return 0.25
    return 0.0


def demo():
    """Right arm: perforated tile (left of the cart) to the cold-aisle gap, then parks. Left arm, after it:
    solid tile (right of the cart) to the hot-aisle gap. Fingers close across the tile's 7 cm width; each
    tile is released 1 cm above the floor."""
    travel = TOP + 0.16
    def grip(t):
        return (t.pos[0], t.pos[1], TOP + 0.02 + 0.005 + 0.013)  # pad tips 5 mm above the cart
    def place(gy):
        return (GAP_X, gy, TOP + 0.01 + 0.005 + 0.013)
    right = pick_place_waypoints(grip(PERF), place(COLD_Y), travel_z=travel)
    right.append(Waypoint(1.5, (0.12, -0.28, TOP + 0.25), FINGER_OPEN, "park"))
    wait = sum(w.duration for w in right)
    left = [Waypoint(wait, (0.12, 0.28, TOP + 0.25), FINGER_OPEN, "wait")]
    left += pick_place_waypoints(grip(SOLID), place(HOT_Y), travel_z=travel)
    left.append(Waypoint(1.5, (0.12, 0.28, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left, "right": right}
