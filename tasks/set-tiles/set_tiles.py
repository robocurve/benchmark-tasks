"""On the station table, in front of the left arm: a dark grey straightedge (30 cm long, 1.2 cm wide, 5 mm tall) fixed
to the table, running straight out from the arms 22 cm left of centre, from 12 to 42 cm in front of the arm bases.
Four blue tiles (6 x 6 cm, 2 cm thick) lie flat in a line on its left side, 30 cm left of centre, their centres 15,
23, 31 and 39 cm in front of the arm bases, turned from square by 25, 10, 30 and 15 degrees in the same order
(anticlockwise seen from above). The straightedge is fixed; the tiles are loose. The tiles are set when all four lie
flat in one row along the straightedge, each touching it or within 1 mm of it and square to it, with a joint of 3 mm
(2 to 4 mm) between neighbouring tiles.
"""

import math
from itertools import pairwise

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

EDGE = Box("straightedge", pos=(0.27, 0.226, TOP + 0.0025), size=(0.30, 0.012, 0.005), color=(0.30, 0.31, 0.33))
EDGE_IN = EDGE.pos[1] - EDGE.size[1] / 2  # the face toward the middle of the table (y = 0.22)
EDGE_OUT = EDGE.pos[1] + EDGE.size[1] / 2  # the face toward the left arm (y = 0.232)
EDGE_X = (EDGE.pos[0] - EDGE.size[0] / 2, EDGE.pos[0] + EDGE.size[0] / 2)

TILE, TILE_KG = (0.06, 0.06, 0.02), 0.17  # porcelain, 2 cm thick
SUPPLY_X, SUPPLY_Y = (0.39, 0.31, 0.23, 0.15), 0.30
# Tiles come out of the box any way round: each starts turned from square (anticlockwise seen from above), so it has to
# be squared to the straightedge. Turned, a tile reaches 35 to 41 mm from its centre along x; 8 cm apart, they don't
# touch.
SUPPLY_TURN_DEG = (15.0, 30.0, 10.0, 25.0)
NAMES = tuple(f"tile_{i}" for i in range(1, 5))
TILE_BLUE = (0.12, 0.35, 0.80)  # saturated: pale objects vanish from the top camera
TILES = [
    Box(name, pos=(x, SUPPLY_Y, TOP + TILE[2] / 2 + 0.001), size=TILE, mass=TILE_KG, color=TILE_BLUE,
        yaw=math.radians(turn))
    for name, x, turn in zip(NAMES, SUPPLY_X, SUPPLY_TURN_DEG, strict=True)
]  # each starts 1 mm above the table, and settles
OBJECTS = [EDGE, *TILES]

MAX_TILT_DEG = 2.0  # flat on the table
MAX_TURN_DEG = 2.0  # square to the straightedge
EDGE_GAP = (-0.0005, 0.001)  # touching the straightedge, or at most 1 mm from it (0.5 mm slack for contact)
FLAT_TOL = 0.002  # resting on the table: its lowest point within 2 mm of it
JOINT = (0.002, 0.004)  # 3 mm joints, +-1 mm, measured at the narrowest point
SET_CREDIT, JOINT_CREDIT = 0.1, 0.2  # 4 x 0.1 + 3 x 0.2 = 1.0

# Where a tile is comes from its centre and rotation, not from ``low``/``high``: Arena builds those from the tile's
# corners, turned with it, so they grow when a tile turns about the vertical.


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(tile) -> np.ndarray:
    """The tile's 8 corners in the world (8 x 3)."""
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return signs * (np.array(TILE) / 2) @ _rot(tile.quat_xyzw).T + np.asarray(tile.pos)


def _tilt_deg(tile) -> float:
    """Angle between the tile's face and the table (near 90: on an edge; over FACE_DOWN_DEG: face down)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, _rot(tile.quat_xyzw)[2, 2]))))


def _turn_deg(tile) -> float:
    """How far the tile is turned from square to the straightedge, about the vertical (0 to 45)."""
    r = _rot(tile.quat_xyzw)
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0])) % 90.0
    return min(yaw, 90.0 - yaw)


def _side(tile) -> str:
    return "in" if float(tile.pos[1]) < EDGE.pos[1] else "out"


def _edge_gap(tile) -> float:
    """From the straightedge to the tile's nearest point, across the straightedge (m)."""
    ys = _corners(tile)[:, 1]
    return EDGE_IN - float(ys.max()) if _side(tile) == "in" else float(ys.min()) - EDGE_OUT


FACE_DOWN_DEG = 135.0  # face down: tilted past 135 degrees (on an edge, near 90, is only not set)


def _intact(tile) -> bool:
    """Not face down, and not dropped off the table."""
    return _tilt_deg(tile) <= FACE_DOWN_DEG and float(_corners(tile)[:, 2].min()) >= TOP - 0.01


def _set(tile) -> bool:
    """Flat on the table, square to the straightedge, against it within its length, and still."""
    return bool(
        _tilt_deg(tile) <= MAX_TILT_DEG
        and abs(float(_corners(tile)[:, 2].min()) - TOP) <= FLAT_TOL
        and _turn_deg(tile) <= MAX_TURN_DEG
        and EDGE_GAP[0] <= _edge_gap(tile) <= EDGE_GAP[1]
        and EDGE_X[0] <= float(tile.pos[0]) <= EDGE_X[1]
        and tile.resting()
    )


def _joints(objects) -> list[float]:
    """The joints of the row, each at its narrowest point (m): between neighbouring set tiles on the side of the
    straightedge with the most set tiles (if the sides tie, the one with more good joints). The instruction asks for
    one row, so tiles set on the other side earn their own credit but none for their joints."""
    rows = []
    for side in ("in", "out"):
        row = sorted((objects[n] for n in NAMES if _set(objects[n]) and _side(objects[n]) == side),
                     key=lambda t: float(t.pos[0]))
        joints = [float(_corners(far)[:, 0].min() - _corners(near)[:, 0].max()) for near, far in pairwise(row)]
        rows.append((len(row), sum(JOINT[0] <= j <= JOINT[1] for j in joints), joints))
    return max(rows, key=lambda r: r[:2])[2]


def _counts(objects) -> tuple[int, int]:
    """(set tiles, good joints)."""
    good = sum(JOINT[0] <= j <= JOINT[1] for j in _joints(objects))
    return sum(_set(objects[n]) for n in NAMES), good


def success(objects):
    """All four tiles set in one row with three good joints; must hold for 1 s."""
    return _counts(objects) == (4, 3)


def score(objects):
    """Partial credit (success scores 1.0): 0.1 per set tile and 0.2 per good joint between neighbouring set tiles in
    the row (the side of the straightedge with the most set tiles). 0 if any tile is face down or off the table."""
    if not all(_intact(objects[n]) for n in NAMES):
        return 0.0
    tiles, joints = _counts(objects)
    return round(SET_CREDIT * tiles + JOINT_CREDIT * joints, 6)


# The demo: the left arm moves each tile straight across the straightedge into the row, farthest first.
GRIP_ABOVE = 0.022  # grip point above a tile's bottom: the pad tips 9 mm up, 4 mm clear of the straightedge's top
TRAVEL_Z = TOP + 0.10
DROP = 0.002  # each tile is let go 2 mm above the table
DEMO_EDGE_GAP = 0.0005  # aim each tile 0.5 mm off the straightedge
DEMO_JOINT = 0.003
FIRST_X = 0.37
# Where each tile lands against where it is let go, measured on the official stack (two traces on 2026-11-03, identical
# to 0.01 mm): (x, y) in mm, and its turn in degrees. Every tile came out 1.4-2.2 mm nearer the arms and turned 1.0-1.7
# degrees anticlockwise, which left two joints at 1.5 and 0.4 mm (score 0.6). The demo lets go of each tile that much
# the other way.
DEMO_LANDING = {"tile_1": (-2.23, -1.00, 1.72), "tile_2": (-1.43, -0.39, 1.72), "tile_3": (-1.37, -0.03, 1.03),
                "tile_4": (-1.93, 0.00, 0.96)}
# The official run (inspect-robots run, replayed with scripts/trace_task.py --replay) lands tiles a little differently
# from those traces, which use seed 0: with DEMO_LANDING alone it scored 1.0, but with joints of 3.84, 3.13 and 2.33 mm
# (tile_1 to tile_4), two of them within 0.4 mm of the 2-4 mm window's edges. These x shifts (mm) aim all three at
# 3.0 mm. Two official runs with them (identical to 0.01 mm) gave 1.0 with joints of 3.21, 2.38 and 3.31 mm: not 3.0
# each, since moving a tile changes how it lands, but every joint inside the window. The middle one sat 0.38 mm from
# the window's edge (review), so a second round of shifts aims all three at 3.0 again from those joints: tile_2 -0.21,
# tile_3 +0.41, tile_4 +0.10 mm on top of the first (-0.84, -0.97, -0.30).
DEMO_OFFICIAL_X = {"tile_1": 0.0, "tile_2": -1.05, "tile_3": -0.56, "tile_4": -0.20}
# Every tile's aim moved this far along the row (mm, + away from the arms). 0 for the demo; the margin runs set it to
# +0.5 and -0.5 to check that the joints hold if every tile lands half a millimetre off. Official runs (2026-11-03,
# replayed), joints in mm: demo 2.88, 3.49, 2.56; +0.5 mm 2.77, 3.60, 3.01; -0.5 mm 2.78, 3.25, 3.28. All three 1.0,
# every joint at least 0.40 mm inside the 2-4 mm window.
DEMO_AIM_SHIFT_MM = 0.0


def demo():
    """Left arm: tile by tile, from the one farthest out, grip it across its sides with the grip turned to match the
    tile, carry it 10 cm above the table over the straightedge while turning the grip back to square, and set it down
    on the middle side, 0.5 mm off the straightedge and 3 mm short of the tile before (allowing for DEMO_LANDING and
    DEMO_OFFICIAL_X). Set down, the fingers close sideways, along the straightedge's width, so no finger ever goes into
    a joint; the one next to the straightedge passes 4 mm above it."""
    row_y = EDGE_IN - TILE[1] / 2 - DEMO_EDGE_GAP
    waypoints = []
    for i, (name, x, turn) in enumerate(zip(NAMES, SUPPLY_X, SUPPLY_TURN_DEG, strict=True)):
        row_x = FIRST_X - i * (TILE[0] + DEMO_JOINT)
        dx, dy, dturn = DEMO_LANDING[name]
        dx += DEMO_OFFICIAL_X[name] - DEMO_AIM_SHIFT_MM
        waypoints += pick_place_waypoints(
            (x, SUPPLY_Y, TOP + GRIP_ABOVE), (row_x - dx / 1000, row_y - dy / 1000, TOP + GRIP_ABOVE + DROP),
            travel_z=TRAVEL_Z, yaw=math.radians(turn), place_yaw=-math.radians(dturn),
        )
    return {"left": waypoints}
