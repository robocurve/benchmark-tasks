"""On the station table, in front of the right arm: five glazed ceramic wall tiles, 6 mm thick, stand on edge in a
row, 6 mm apart, with their faces towards the left and right arms. Tall and short tiles alternate: tall, short,
tall, short, tall. The tall tiles are 10 x 10 cm. The short tiles are 15 x 7.5 cm, standing on a long edge. To the
left of the row, on the midline, is an open box with 4 cm high walls and an inside of 19 x 12 cm.

Task: move all five tiles into the box in as few trips as possible. A trip starts with a lift: tiles that leave
the table together (within 0.5 s) count as one trip. The tiles are fragile: a tile that tips over or falls counts as
broken.

Score (task_score): 0 if any tile tips over (its face more than 60 degrees from vertical) or falls (its centre
drops faster than 0.5 m/s) at any time, or if not all five tiles rest in the box at the end. Otherwise
(5 - K + 1) / 5 for K trips: 1.0 for one trip, 0.2 for five.

Run it with -E object_jitter=0: the gaps between the tiles are smaller than the default jitter.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---- geometry (m) and masses (kg): ceramic at 2.0 g/cm^3 ----
THICK = 0.006
TALL = dict(size=(0.10, THICK, 0.10), mass=0.12)  # 10 x 10 cm wall tile
SHORT = dict(size=(0.15, THICK, 0.075), mass=0.135)  # 7.5 x 15 cm subway tile on its long edge
KINDS = [TALL, SHORT, TALL, SHORT, TALL]  # 0.63 kg in all, under the arm's 2 kg payload
N = len(KINDS)
GAP = 0.006  # between neighbouring tiles
PITCH = THICK + GAP
ROW_XY = (0.24, -0.14)  # centre of the row
BOX_XY = (0.24, 0.02)  # centre of the box
BOX_INNER = (0.19, 0.12)  # x, y
WALL_T, WALL_H = 0.01, 0.04

# ---- thresholds ----
LIFT = 0.01  # a tile is lifted when its lowest point is this far above the table
MOVE_WINDOW_STEPS = 10  # lifts within 10 control steps (0.5 s at 20 Hz) of a trip's first lift join that trip
MAX_TILT_DEG = 60.0  # a tile has tipped over (lies, not leans) when its face is further than this from vertical
MAX_FALL_SPEED = 0.5  # m/s: a tile whose centre drops faster than this has fallen (a free fall of about 1.3 cm)
AT_START = 0.005  # every tile within this distance of its start pose: the counter starts again

TILE_NAMES = [f"tile_{i}" for i in range(N)]
TILE_COLOR = (0.55, 0.75, 0.85)
BOX_COLOR = (0.62, 0.48, 0.32)

TILES = [
    Box(
        name,
        pos=(ROW_XY[0], ROW_XY[1] + (i - (N - 1) / 2) * PITCH, TOP + kind["size"][2] / 2),
        size=kind["size"],
        mass=kind["mass"],
        color=TILE_COLOR,
    )
    for i, (name, kind) in enumerate(zip(TILE_NAMES, KINDS, strict=True))
]

_bx, _by = BOX_XY
_ix, _iy = BOX_INNER[0] / 2, BOX_INNER[1] / 2
_wz = TOP + WALL_H / 2
_long, _short = (WALL_T, BOX_INNER[1] + 2 * WALL_T, WALL_H), (BOX_INNER[0], WALL_T, WALL_H)
WALLS = [
    Box("box_front", pos=(_bx + _ix + WALL_T / 2, _by, _wz), size=_long, color=BOX_COLOR),
    Box("box_back", pos=(_bx - _ix - WALL_T / 2, _by, _wz), size=_long, color=BOX_COLOR),
    Box("box_left", pos=(_bx, _by + _iy + WALL_T / 2, _wz), size=_short, color=BOX_COLOR),
    Box("box_right", pos=(_bx, _by - _iy - WALL_T / 2, _wz), size=_short, color=BOX_COLOR),
]
OBJECTS = TILES + WALLS


# ---- per-tile quantities ----
def _face_normal(quat_xyzw) -> np.ndarray:
    """The tile's local y axis (normal to its large faces) in the world."""
    x, y, z, w = quat_xyzw
    return np.array([2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)])


def tipped(tile) -> bool:
    """The face is more than MAX_TILT_DEG from vertical: its normal is that far from horizontal."""
    return bool(abs(_face_normal(tile.quat_xyzw)[2]) > np.sin(np.radians(MAX_TILT_DEG)))


def fell(tile) -> bool:
    return bool(tile.lin_vel[2] < -MAX_FALL_SPEED)


def lifted(tile) -> bool:
    return bool(tile.low[2] > TOP + LIFT)


def in_box(tile) -> bool:
    """The tile's centre is inside the box walls, and it rests on the table."""
    dx, dy = np.abs(tile.pos[:2] - np.array(BOX_XY))
    return bool(dx < _ix and dy < _iy and not lifted(tile) and tile.resting())


def at_start(objects) -> bool:
    return all(
        np.linalg.norm(objects[t.name].pos - np.array(t.pos)) < AT_START and not tipped(objects[t.name])
        for t in TILES
    )


# ---- trip counter ----
class Counter:
    """Counts trips from the tile poses after each control step. The hooks get no reset signal, so the count
    starts again whenever every tile stands at its start pose (which is true after each reset)."""

    def __init__(self):
        self.restart()
        self._last = None

    def restart(self):
        self.step = 0
        self.moves = 0
        self.move_start = None  # step of the current trip's first lift
        self.carried = dict.fromkeys(TILE_NAMES, False)  # lifted, and not yet set down at rest
        self.broken = False

    def update(self, objects):
        """Read one step. success() and score() get the same dict each step, so a step is read only once."""
        if objects is self._last:
            return
        self._last = objects
        if at_start(objects):
            self.restart()
            return
        self.step += 1
        for name in TILE_NAMES:
            tile = objects[name]
            self.broken |= tipped(tile) or fell(tile)
            if lifted(tile) and not self.carried[name]:
                self.carried[name] = True
                if self.move_start is None or self.step - self.move_start > MOVE_WINDOW_STEPS:
                    self.moves += 1
                    self.move_start = self.step
            elif self.carried[name] and not lifted(tile) and tile.resting():
                self.carried[name] = False  # set down: a dip below LIFT while still moving is not a new lift


COUNTER = Counter()


def _score(objects) -> float:
    COUNTER.update(objects)
    if COUNTER.broken or not all(in_box(objects[n]) for n in TILE_NAMES):
        return 0.0
    return min(max((N - COUNTER.moves + 1) / N, 0.0), 1.0)


# ---- hooks read by yam_arena (task_score) ----
def success(objects):
    """Score 1.0: all five tiles rest in the box after one trip, and none tipped over or fell."""
    return _score(objects) == 1.0


def score(objects):
    """0 if a tile tipped over or fell, or not all tiles are in the box; otherwise (N - K + 1) / N for K trips."""
    return _score(objects)


def demo():
    """Right arm, one trip: open fully around the whole row (54 mm wide; the fingers open to 94 mm), close on it
    with the pad tips 4 cm above the table, so the pads hold the short tiles too, lift, carry slowly above the box
    walls, set the tiles down in the box and let go."""
    grasp_z = TOP + 0.04 + PAD_TIP_PAST_TCP
    travel_z = grasp_z + WALL_H + 0.02  # tile bottoms 2 cm above the box walls
    place_z = grasp_z + 0.003
    release = -0.03  # pads 60 mm apart: clear of the box walls
    (rx, ry), (bx, by) = ROW_XY, BOX_XY
    return {
        "right": [
            Waypoint(1.5, (rx, ry, travel_z + 0.06), FINGER_OPEN, "above the row"),
            Waypoint(1.5, (rx, ry, grasp_z), FINGER_OPEN, "descend"),
            Waypoint(1.5, (rx, ry, grasp_z), FINGER_CLOSED, "close"),
            Waypoint(2.0, (rx, ry, travel_z), FINGER_CLOSED, "lift"),
            Waypoint(3.5, (bx, by, travel_z), FINGER_CLOSED, "carry"),
            Waypoint(1.5, (bx, by, travel_z), FINGER_CLOSED, "settle"),
            Waypoint(2.0, (bx, by, place_z), FINGER_CLOSED, "lower"),
            Waypoint(1.0, (bx, by, place_z), FINGER_CLOSED, "settle"),
            Waypoint(0.8, (bx, by, place_z), release, "release"),
            Waypoint(1.2, (bx, by, travel_z + 0.03), FINGER_OPEN, "retreat"),
        ]
    }
