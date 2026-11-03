"""On the station table: three gas cylinders (16 mm across, 65 mm tall) stand upright in a row 12 cm right of the
midline, running out from the arms, 12 to 24 cm out: maroon, green, maroon. Two fixed grey masonry walls, 20 mm thick, stand
on the table: a 91 mm tall one, 16 cm long, just beyond the row's far end and across it, and a 46 mm tall one, 16 cm
long, beside the row on its right, running out from the arms.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SCALE = 20
FT = 0.3048 / SCALE  # one foot at scale (m)

# WHY: 1.4 times too thick for 1:20 (a real cylinder is about 23 cm across) so the grippers can hold them.
RADIUS, HEIGHT = 0.008, 0.065
ROW_Y = -0.12
START = {"acetylene_a": 0.12, "oxygen": 0.18, "acetylene_b": 0.24}  # x along the row
GREEN, MAROON = (0.10, 0.50, 0.22), (0.45, 0.08, 0.12)
CYLINDERS = [Cylinder(n, pos=(x, ROW_Y, TOP + HEIGHT / 2 + 0.001), radius=RADIUS, height=HEIGHT, mass=0.10,
                      color=GREEN if n == "oxygen" else MAROON) for n, x in START.items()]
FUELS = [c.name for c in CYLINDERS if c.name != "oxygen"]

# Each wall's centre line on the table (two ends, m) and its height.
WALL_T = 0.020
WALLS = {
    "tall_wall": (((0.30, -0.20), (0.30, -0.04)), 6 * FT),  # 6 ft
    "low_wall": (((0.10, -0.22), (0.26, -0.22)), 3 * FT),  # 3 ft
}
WALL_BOXES = []
for _name, ((_a, _b), _h) in WALLS.items():
    _along = abs(_b[0] - _a[0]) + abs(_b[1] - _a[1])
    _size = (WALL_T, _along, _h) if _a[0] == _b[0] else (_along, WALL_T, _h)
    WALL_BOXES.append(Box(_name, pos=((_a[0] + _b[0]) / 2, (_a[1] + _b[1]) / 2, TOP + _h / 2), size=_size,
                          color=(0.55, 0.55, 0.55)))
OBJECTS = [*CYLINDERS, *WALL_BOXES]

# 29 CFR 1910.253(b)(4)(iii): "Oxygen cylinders in storage shall be separated from fuel-gas cylinders or combustible
# materials (especially oil or grease), a minimum distance of 20 feet (6.1 m) or by a noncombustible barrier at least
# 5 feet (1.5 m) high having a fire-resistance rating of at least one-half hour."
APART_FT = 20.0
BARRIER_FT = 5.0
# WHY: a line from the oxygen cylinder to a fuel cylinder must cross the barrier at least a cylinder's width in from
# its ends, so a barrier only grazed at its end doesn't count.
END_MARGIN = 2 * RADIUS
ON_MM = 2.0  # a cylinder's bottom within this of the table
UPRIGHT_DEG = 10.0


def _xy(c) -> np.ndarray:
    return np.asarray(c.pos, dtype=float)[:2]


def _upright(c) -> bool:
    """Standing on the table, its axis within UPRIGHT_DEG of vertical."""
    x, y, _, _ = (float(v) for v in c.quat_xyzw)
    return (abs(float(c.low[2]) - TOP) <= ON_MM / 1000
            and 1 - 2 * (x * x + y * y) >= math.cos(math.radians(UPRIGHT_DEG)))


def apart_ft(a, b) -> float:
    """The gap between two upright cylinders, in feet at scale."""
    return (float(np.linalg.norm(_xy(a) - _xy(b))) - 2 * RADIUS) / FT


def _cross(o, a, b) -> float:
    return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))


def behind_wall(a, b) -> str | None:
    """The barrier-height wall, if any, that the line between the two cylinders crosses away from its ends."""
    p, q = _xy(a), _xy(b)
    for name, ((w0, w1), h) in WALLS.items():
        if h < BARRIER_FT * FT:
            continue
        w0, w1 = np.array(w0), np.array(w1)
        d = (w1 - w0) / np.linalg.norm(w1 - w0)
        w0, w1 = w0 + END_MARGIN * d, w1 - END_MARGIN * d
        if _cross(p, q, w0) * _cross(p, q, w1) < 0 and _cross(w0, w1, p) * _cross(w0, w1, q) < 0:
            return name
    return None


def separated(objects, fuel) -> bool:
    o2, f = objects["oxygen"], objects[fuel]
    return apart_ft(o2, f) >= APART_FT or behind_wall(o2, f) is not None


def _all_upright(objects) -> bool:
    return all(_upright(objects[c.name]) for c in CYLINDERS)


def success(objects):
    """Every cylinder upright, the oxygen separated from both fuel cylinders, all still; holds 1 s."""
    return (_all_upright(objects) and all(separated(objects, f) for f in FUELS)
            and all(objects[c.name].resting() for c in CYLINDERS))


def distance(objects):
    """Fuel cylinders not yet separated from the oxygen, plus 3 if any cylinder is down."""
    return sum(not separated(objects, f) for f in FUELS) + (0 if _all_upright(objects) else 3)


def score(objects):
    """Partial credit (success scores 1.0): 0 if any cylinder is down, else 0.5 for each fuel cylinder separated from
    the oxygen."""
    if not _all_upright(objects):
        return 0.0
    return sum(0.5 for f in FUELS if separated(objects, f))


GRIP_ABOVE = 0.050  # TCP above a cylinder's bottom: pad tips 37 mm up its 65 mm
CARRY_Z = TOP + WALLS["tall_wall"][1] + 0.012 + GRIP_ABOVE  # the cylinder's bottom 12 mm above the tall wall
BEHIND = (0.345, ROW_Y)  # past the tall wall, on the row's line


def demo():
    """The right arm lifts the oxygen cylinder out of the row, carries it over the tall wall and stands it behind it."""
    px, py = START["oxygen"], ROW_Y
    qx, qy = BEHIND
    grip = TOP + 0.001 + GRIP_ABOVE
    set_down = TOP + 0.002 + GRIP_ABOVE
    right = [
        Waypoint(2.0, (px, py, CARRY_Z), FINGER_OPEN, "above the oxygen"),
        Waypoint(1.5, (px, py, grip), FINGER_OPEN, "down around it"),
        Waypoint(0.8, (px, py, grip), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (px, py, CARRY_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.5, (qx, qy, CARRY_Z), FINGER_CLOSED, "over the tall wall"),
        Waypoint(1.5, (qx, qy, set_down), FINGER_CLOSED, "stand it down"),
        Waypoint(0.8, (qx, qy, set_down), FINGER_CLOSED, "settle"),
        Waypoint(0.6, (qx, qy, set_down), FINGER_OPEN, "let go"),
        Waypoint(1.0, (qx, qy, CARRY_Z), FINGER_OPEN, "up"),
        Waypoint(1.5, (0.20, -0.25, CARRY_Z), FINGER_OPEN, "clear"),
    ]
    return {"right": right}
