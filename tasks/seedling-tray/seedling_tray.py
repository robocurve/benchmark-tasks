"""On the station table, in front of the left arm: a black plastic seedling plug tray fixed to the table, its centre 28
cm in front of the arm bases and 20 cm left of centre. It has 12 square cells in 3 rows of 4 (6 cm apart, with thin
walls 1.2 cm tall), its long side running straight out from the arms. Green seedling plugs (upright cylinders, 3 cm
across, 5 cm tall) stand in its cells, one per cell, but some cells are empty. Four spare plugs stand on the table in
a line further left (35 cm left of centre), 6 cm apart. The tray is fixed; the plugs are loose.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TRAY_XY = np.array([0.28, 0.20])
COLS, ROWS, PITCH = 4, 3, 0.06  # cells along x (straight out) and y
WALL_T, WALL_H, FLOOR = 0.004, 0.012, 0.005  # wall thickness and height above the floor; floor thickness
FLOOR_TOP = TOP + FLOOR


def _xyz(xy, z) -> tuple[float, float, float]:
    """A position as plain floats: the official stack can't turn numpy float64 values into a loose object's start
    pose (EmbodimentFault on the first official run, 2026-11-04)."""
    return float(xy[0]), float(xy[1]), float(z)


TRAY_BLACK = (0.10, 0.10, 0.11)
_LEN = (COLS * PITCH + WALL_T, ROWS * PITCH + WALL_T)
_WALL_Z = FLOOR_TOP + WALL_H / 2
TRAY = [Box("tray_floor", pos=_xyz(TRAY_XY, TOP + FLOOR / 2), size=(*_LEN, FLOOR), color=TRAY_BLACK)]
TRAY += [Box(f"tray_wall_x{i}", pos=_xyz((TRAY_XY[0] + (i - COLS / 2) * PITCH, TRAY_XY[1]), _WALL_Z),
             size=(WALL_T, _LEN[1], WALL_H), color=TRAY_BLACK) for i in range(COLS + 1)]
TRAY += [Box(f"tray_wall_y{j}", pos=_xyz((TRAY_XY[0], TRAY_XY[1] + (j - ROWS / 2) * PITCH), _WALL_Z),
             size=(_LEN[0], WALL_T, WALL_H), color=TRAY_BLACK) for j in range(ROWS + 1)]  # fixed: no mass

CELLS = {(c, r): TRAY_XY + np.array([(c - (COLS - 1) / 2) * PITCH, (r - (ROWS - 1) / 2) * PITCH])
         for c in range(COLS) for r in range(ROWS)}  # (column out from the arms, row to the left): centre

EMPTY = ((1, 0), (3, 1), (0, 2))  # one per row, no two in a column or next to each other

PLUG_R, PLUG_H, PLUG_KG = 0.015, 0.05, 0.05  # a 30 mm, 50 g cylinder holds still in the grip (rules section 3)
PLUG_GREEN = (0.15, 0.55, 0.20)
IN_TRAY = tuple(k for k in CELLS if k not in EMPTY)
TRAY_PLUGS = {f"plug_{i}": k for i, k in enumerate(IN_TRAY, start=1)}  # name: its cell
SPARE_X, SPARE_Y = (0.18, 0.24, 0.30, 0.36), 0.35
SPARES = tuple(f"spare_{i}" for i in range(1, len(SPARE_X) + 1))
PLUGS = [Cylinder(n, pos=_xyz(CELLS[k], FLOOR_TOP + PLUG_H / 2 + 0.001), radius=PLUG_R, height=PLUG_H, mass=PLUG_KG,
                  color=PLUG_GREEN) for n, k in TRAY_PLUGS.items()]
PLUGS += [Cylinder(n, pos=_xyz((x, SPARE_Y), TOP + PLUG_H / 2 + 0.001), radius=PLUG_R, height=PLUG_H, mass=PLUG_KG,
                   color=PLUG_GREEN) for n, x in zip(SPARES, SPARE_X, strict=True)]  # each starts 1 mm up, and settles
OBJECTS = [*TRAY, *PLUGS]
NAMES = (*TRAY_PLUGS, *SPARES)

CELL_HALF = (PITCH - WALL_T) / 2  # in a cell: the plug's centre inside the cell's walls, wherever it stands (review:
# with an 8 mm limit, a plug standing in its cell 9-12 mm off centre earned nothing)
FLOOR_TOL = 0.003  # its bottom within 3 mm of the tray floor
MAX_TILT_DEG = 10.0  # standing
UNTOUCHED = 0.005  # a plug already in the tray: still within 5 mm of where it stood
CELL_CREDIT = 1 / len(EMPTY)

# Where a plug is comes from its centre and axis, not from ``low``/``high``: Arena builds those from the cylinder's
# bounding box, turned with it, so they grow when a plug spins about its axis.


def _axis(plug) -> np.ndarray:
    x, y, z, w = (float(v) for v in plug.quat_xyzw)
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])  # the plug's axis, world


def _tilt_deg(plug) -> float:
    return float(np.degrees(np.arccos(min(1.0, abs(float(_axis(plug)[2]))))))


def _bottom(plug) -> float:
    """The plug's lowest point (m): exact for a cylinder, half its height times |a.z| plus its radius times
    sqrt(1 - a.z^2)."""
    az = abs(float(_axis(plug)[2]))
    return float(plug.pos[2]) - PLUG_H / 2 * az - PLUG_R * math.sqrt(max(0.0, 1 - az * az))


def _standing(plug) -> bool:
    return _tilt_deg(plug) <= MAX_TILT_DEG


def _in_cell(plug, cell) -> bool:
    """Standing on the tray floor with its centre inside ``cell``'s walls."""
    d = np.abs(np.asarray(plug.pos[:2], dtype=float) - CELLS[cell])
    return bool(_standing(plug) and abs(_bottom(plug) - FLOOR_TOP) <= FLOOR_TOL and np.all(d < CELL_HALF))


def _grade(objects) -> float:
    plugs = {n: objects[n] for n in NAMES}
    if any(_bottom(p) < TOP - 0.01 or not _standing(p) for p in plugs.values()):
        return 0.0  # a plug off the table or knocked over is a damaged seedling
    # The plugs already in the tray must be left alone: moving one, even within its cell, earns nothing.
    if any(np.linalg.norm(np.asarray(plugs[n].pos[:2]) - CELLS[k]) > UNTOUCHED or not _in_cell(plugs[n], k)
           for n, k in TRAY_PLUGS.items()):
        return 0.0
    filled = sum(any(_in_cell(plugs[s], cell) and plugs[s].resting() for s in SPARES) for cell in EMPTY)
    return round(CELL_CREDIT * filled, 6)


def success(objects):
    """Every empty cell holds a spare plug standing on its floor, the plugs already in the tray untouched, every plug
    standing and still. Must hold for 1 s."""
    return _grade(objects) == 1.0 and all(objects[n].resting() for n in NAMES)


def score(objects):
    """Partial credit (success scores 1.0): a third for each empty cell that now holds a spare plug standing on its
    floor, and still. 0 if a plug already in the tray was moved more than 5 mm, or any plug ends knocked over or off
    the table."""
    return _grade(objects)


GRIP_ABOVE = 0.038  # grip point above a plug's bottom: 1.2 cm below its top; the pad tips 1.4 cm above the walls
TRAVEL_Z = TOP + 0.12  # a carried plug's bottom 2.7 cm above every plug top
DROP = 0.001  # each plug is let go 1 mm above the tray floor
DEMO_FILL = dict(zip(SPARES, EMPTY))  # spare_1 to spare_3 go into the empty cells; spare_4 is left over
# Where each plug lands against where it is let go, in mm (x, y): no correction needed. The first official run
# (2026-11-04, replayed) scored 1.0 with the plugs 1.0, 0.4 and 0.4 mm from their cells' centres (the walls allow
# 13 mm) and every plug already in the tray unmoved (0.0 mm).
DEMO_LANDING = {cell: (0.0, 0.0) for cell in EMPTY}
# Every plug's aim moved this far across the tray (mm, + to the left), toward the plugs beside each gap, which the
# fingers pass. 0 for the demo; the margin runs set it to +4 and -4. Official runs (2026-11-04, replayed): demo, +4 and
# -4 mm all 1.0, the spares at most 4.8 mm from their cells' centres and every plug already in the tray unmoved.
DEMO_AIM_SHIFT_MM = 0.0
# The fingers come down only 44 mm apart, 7 mm clear of a 30 mm plug, so the fingers stay 13 mm from the plugs next
# to an empty cell; they grip aimed 22 mm apart, 4 mm inside the plug, so each presses about 20 N, under its 50 N
# limit, and the pair centres the plug (fully closed, both push at the limit and a part can slide across the grip:
# 17 mm on the official stack, in the snug-tray task); they let go slowly, opening to 40 mm.
APPROACH_FINGER = FINGER_OPEN * 44 / 94  # FINGER_OPEN is 94 mm apart
GRIP_FINGER = FINGER_OPEN * 22 / 94
RELEASE_FINGER = FINGER_OPEN * 40 / 94


def _move(pick, place) -> list[Waypoint]:
    """A top-down pick and place with the fingers only partly open: wait over the plug before gripping it, carry it,
    set it down slowly, open the fingers slowly to RELEASE_FINGER, then lift slowly."""
    (px, py, pz), (qx, qy, qz) = pick, place
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), APPROACH_FINGER, "to the object"),
        Waypoint(1.5, (px, py, pz), APPROACH_FINGER, "descend"),
        Waypoint(0.5, (px, py, pz), APPROACH_FINGER, "steady"),
        Waypoint(0.8, (px, py, pz), GRIP_FINGER, "close"),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP_FINGER, "lift"),
        Waypoint(1.8, (qx, qy, TRAVEL_Z), GRIP_FINGER, "carry"),
        Waypoint(0.5, (qx, qy, TRAVEL_Z), GRIP_FINGER, "hover"),
        Waypoint(2.0, (qx, qy, qz), GRIP_FINGER, "set down"),
        Waypoint(0.3, (qx, qy, qz), GRIP_FINGER, "hold"),
        Waypoint(1.5, (qx, qy, qz), RELEASE_FINGER, "release"),
        Waypoint(0.3, (qx, qy, qz), RELEASE_FINGER, "settle"),
        Waypoint(1.5, (qx, qy, TRAVEL_Z), RELEASE_FINGER, "retreat"),
    ]


def demo():
    """Left arm: spare_1 to spare_3, one at a time, each gripped across its middle 1.2 cm below its top (the fingers
    close along the table's width and come down only 44 mm apart), carried 2.7 cm above every plug top, and set
    down slowly in an empty cell (allowing for DEMO_LANDING), let go gently 1 mm above the tray floor. spare_4 and
    the plugs already in the tray are never touched."""
    waypoints = []
    for spare, cell in DEMO_FILL.items():
        x = SPARE_X[SPARES.index(spare)]
        dx, dy = DEMO_LANDING[cell]
        cx, cy = CELLS[cell]
        place = _xyz((cx - dx / 1000, cy + (DEMO_AIM_SHIFT_MM - dy) / 1000), FLOOR_TOP + DROP + GRIP_ABOVE)
        waypoints += _move((x, SPARE_Y, TOP + GRIP_ABOVE), place)
    return {"left": waypoints}
