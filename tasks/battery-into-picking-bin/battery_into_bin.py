"""On the station table, from the arms outward: a 9-volt battery standing upright midway between the arms,
with four 4 cm wooden blocks (red, blue, green, yellow) around it, one on each diagonal; then, behind them, a
cardboard picking bin (24 x 20 cm, open at the top, its low 6.5 cm front wall toward the robot and its back wall
13.5 cm tall) centred between the arms. The battery and the blocks are loose; the bin is fixed in place.

The battery and the bin are NVIDIA SimReady props (CC-BY 4.0), loaded by ``simready.py``.

Run it:  inspect-robots run --embodiment yam_arena -E task=battery_into_bin.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

REST = 0.001  # props spawn this far above the table and settle onto it

# -- the bin: SimReady's open cardboard box A06 (61 x 50.5 x 30.2 cm) scaled to 45 % --------------------------------
BIN_SCALE = 0.45
BIN_XY = (0.415, 0.0)  # behind the blocks: its front wall at x = 0.30
BIN_YAW = -math.pi / 2  # the box's low wall (its -y side) faces the robot (-x)
BIN_HALF = (0.2525 * BIN_SCALE, 0.3055 * BIN_SCALE)  # half depth (x) and half width (y) on the table: 11.4, 13.7 cm
BIN_FLOOR = 0.004 * BIN_SCALE  # floor thickness: 1.8 mm
BIN_BACK = 0.3024 * BIN_SCALE  # back wall: 13.6 cm (the front wall is 6.5 cm)
BIN = SimReady(
    "picking_bin",
    pos=(BIN_XY[0], BIN_XY[1], TOP + REST),
    size=(2 * BIN_HALF[0], 2 * BIN_HALF[1], BIN_BACK),
    yaw=BIN_YAW,
    package="box_cardboard_open_a06",
    usd="sm_box_cardboard_open_a06_01.usd",
    scale=BIN_SCALE,
)

# -- the battery: SimReady's 9 V battery A01, 17.5 x 27 x 53.4 mm with its snaps, standing on its base ------------------
BATTERY_SIZE = (0.0175, 0.027, 0.0534)
BATTERY_MASS = 0.045  # a real 9 V alkaline battery (the asset's own 0.2 kg is solid steel)
BATTERY_XY = (0.19, 0.0)
BATTERY = SimReady(
    "battery",
    pos=(BATTERY_XY[0], BATTERY_XY[1], TOP + REST),
    size=BATTERY_SIZE,
    mass=BATTERY_MASS,
    package="power_battery_9v_a01",
    usd="sm_power_battery_9v_a01_01.usd",
    convex=True,  # a box: collide as its hull, so the finger pads grip it cleanly
)

# -- four 4 cm painted wooden blocks (40 g) as distractors ----------------------------------------------------------
BLOCK = 0.04
BLOCK_MASS = 0.04
BLOCK_COLORS = {
    # around the battery, one per diagonal: 6 cm nearer or farther and 8 cm to either side, clear of the open fingers
    "red_block": ((0.13, 0.08), (0.85, 0.12, 0.12)),
    "blue_block": ((0.13, -0.08), (0.15, 0.30, 0.85)),
    "green_block": ((0.25, 0.08), (0.12, 0.65, 0.22)),
    "yellow_block": ((0.25, -0.08), (0.95, 0.80, 0.10)),
}
BLOCKS = [
    Box(name, pos=(x, y, TOP + BLOCK / 2), size=(BLOCK,) * 3, mass=BLOCK_MASS, color=color)
    for name, ((x, y), color) in BLOCK_COLORS.items()
]

OBJECTS = [BIN, BATTERY, *BLOCKS]

# -- scoring ---------------------------------------------------------------------------------------------------------
INSIDE_MARGIN = 0.012  # m: the battery's centre is at least this far inside the bin's outer walls
ON_FLOOR_Z = 0.040  # m above the table: the battery's centre is this low when it lies on the bin floor (upright: 2.9 cm)
LIFTED_Z = 0.060  # m above the table: the battery's centre this high is clear of the table (resting upright: 2.8 cm)
FALLEN_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table


def _over_bin(pos, margin: float = 0.0) -> bool:
    """The point is over the bin's footprint, at least ``margin`` inside its outer walls."""
    return bool(abs(pos[0] - BIN_XY[0]) < BIN_HALF[0] - margin and abs(pos[1] - BIN_XY[1]) < BIN_HALF[1] - margin)


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.lin_vel)) for s in objects.values())


def _violation(objects) -> bool:
    """Scores 0: a non-finite state, an object off the table, or a block over the bin (packed by mistake)."""
    if not _finite(objects):
        return True
    if any(objects[o.name].pos[2] < TOP + FALLEN_Z for o in OBJECTS if o.free):
        return True
    return any(_over_bin(objects[b.name].pos) for b in BLOCKS)


def _in_bin(objects) -> bool:
    battery = objects["battery"].pos
    return _over_bin(battery, INSIDE_MARGIN) and battery[2] < TOP + BIN_BACK


def success(objects) -> bool:
    """The battery lies on the bin floor, still, and no block is over the bin or any object off the table."""
    battery = objects["battery"]
    return bool(
        not _violation(objects)
        and _in_bin(objects)
        and battery.pos[2] < TOP + ON_FLOOR_Z
        and battery.resting()
    )



def score(objects) -> float:
    """Partial credit (success scores 1.0): 0.5 when the battery is inside the bin but not lying still on its floor
    (held in it, or still moving); 0.25 when it is lifted clear of the table elsewhere; 0 otherwise, and 0 for any
    violation."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    if _in_bin(objects):
        return 0.5
    return 0.25 if objects["battery"].pos[2] > TOP + LIFTED_Z else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
GRIP_Z = 0.032  # TCP height above the battery's base: the pad tips 1.9 cm up, the battery's top 5.3 cm up
TRAVEL_Z = TOP + 0.12  # the carried battery's base clears the bin's 6.5 cm front wall by 2.3 cm
# Inside the bin, 8 cm behind its front wall and 4 cm right of its middle: the open fingers (along y, 9.4 cm apart)
# stay 3.5 cm clear of the side wall. Nearer the wall, a finger catches the battery and lifts it back out.
PLACE_XY = (0.38, -0.04)
PLACE_Z = TOP + BIN_FLOOR + GRIP_Z + 0.012  # let go with the battery's base 1.2 cm above the bin floor


def demo():
    """Right arm: grip the battery across its 27 mm sides, carry it over the bin's low front wall, lower it into the
    bin and let go."""
    pick = (BATTERY_XY[0], BATTERY_XY[1], TOP + REST + GRIP_Z)
    place = (PLACE_XY[0], PLACE_XY[1], PLACE_Z)
    return {"right": pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)}
