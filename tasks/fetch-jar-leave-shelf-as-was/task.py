"""On the station table, in front of the left arm: a low wooden pantry shelf with a back wall. Three tall green
bottles stand in a row along its front edge; behind them, against the back, stand three short spice jars: a blue
one in the middle between two white ones. The blue jar is hemmed in: the white jars stand 10 mm from it on each
side, the middle bottle 10 mm in front of it, and the bottles are too tall to lift the jar over. A yellow mat lies
on the table between the arms. Fetch the blue jar to the mat, and put back whatever was moved to reach it so the
shelf ends as it started.
"""

import itertools

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
R = 0.030  # every jar and bottle is 60 mm across
JAR_H, JAR_MASS = 0.100, 0.15
BOTTLE_H, BOTTLE_MASS = 0.180, 0.30
FRONT_X, BACK_X = 0.29, 0.36  # row centres: 10 mm between the rows
ROW_Y = 0.17  # the middle of both rows
BACK_Y = (ROW_Y + 0.07, ROW_Y, ROW_Y - 0.07)  # 70 mm pitch: 10 mm between back-row jars
FRONT_Y = (ROW_Y + 0.10, ROW_Y, ROW_Y - 0.10)  # 100 mm pitch: 40 mm between bottles

SHELF_H = 0.020  # a solid board standing on the table
SHELF_TOP = TOP + SHELF_H
SHELF_X0, WALL_X0 = 0.25, 0.425  # front edge of the board; front face of the back wall
WALL_T, WALL_H = 0.010, 0.120
SHELF_Y0, SHELF_Y1 = 0.02, 0.32
SHELF_W = SHELF_Y1 - SHELF_Y0

MAT_XY, MAT_SIZE, MAT_T = (0.18, 0.0), 0.10, 0.002
MAT_TOP = TOP + MAT_T

WOOD, WALL_C, YELLOW = (0.62, 0.45, 0.28), (0.55, 0.40, 0.25), (0.95, 0.80, 0.15)
WHITE, BLUE, GREEN = (0.92, 0.92, 0.90), (0.12, 0.30, 0.85), (0.10, 0.40, 0.18)

SHELF = [
    Box("shelf_board", pos=((SHELF_X0 + WALL_X0 + WALL_T) / 2, (SHELF_Y0 + SHELF_Y1) / 2, TOP + SHELF_H / 2),
        size=(WALL_X0 + WALL_T - SHELF_X0, SHELF_W, SHELF_H), color=WOOD),
    Box("back_wall", pos=(WALL_X0 + WALL_T / 2, (SHELF_Y0 + SHELF_Y1) / 2, SHELF_TOP + WALL_H / 2),
        size=(WALL_T, SHELF_W, WALL_H), color=WALL_C),
]
MAT = Box("yellow_mat", pos=(*MAT_XY, TOP + MAT_T / 2), size=(MAT_SIZE, MAT_SIZE, MAT_T), color=YELLOW)

# Home spots. Items of one kind look the same, so any of them may fill any spot of that kind.
JAR_HOMES = {"white_jar_left": (BACK_X, BACK_Y[0]), "white_jar_right": (BACK_X, BACK_Y[2])}
BOTTLE_HOMES = {"bottle_left": (FRONT_X, FRONT_Y[0]), "bottle_middle": (FRONT_X, FRONT_Y[1]),
                "bottle_right": (FRONT_X, FRONT_Y[2])}
GROUPS = (JAR_HOMES, BOTTLE_HOMES)
HEIGHT = {**{n: JAR_H for n in JAR_HOMES}, **{n: BOTTLE_H for n in BOTTLE_HOMES}, "blue_jar": JAR_H}
SHELF_ITEMS = (*JAR_HOMES, *BOTTLE_HOMES)
ALL_ITEMS = (*SHELF_ITEMS, "blue_jar")

WHITE_JARS = [Cylinder(n, pos=(x, y, SHELF_TOP + JAR_H / 2), radius=R, height=JAR_H, mass=JAR_MASS, color=WHITE)
              for n, (x, y) in JAR_HOMES.items()]
BOTTLES = [Cylinder(n, pos=(x, y, SHELF_TOP + BOTTLE_H / 2), radius=R, height=BOTTLE_H, mass=BOTTLE_MASS,
                    color=GREEN) for n, (x, y) in BOTTLE_HOMES.items()]
BLUE_JAR = Cylinder("blue_jar", pos=(BACK_X, BACK_Y[1], SHELF_TOP + JAR_H / 2), radius=R, height=JAR_H,
                    mass=JAR_MASS, color=BLUE)
ITEMS = [*WHITE_JARS, *BOTTLES, BLUE_JAR]

OBJECTS = [*SHELF, MAT, *ITEMS]

UPRIGHT_DEG = 10.0  # standing: axis within 10 degrees of vertical
FALLEN_DEG = 45.0  # knocked over: axis more than 45 degrees from vertical, at any moment
LYING_FRAC = 0.8  # knocked over: its top less than 80% of its length above its lowest point
BOTTOM_TOL = 0.004  # standing on a surface: the item's lowest point within 4 mm of it
HOME_TOL, NEAR_TOL = 0.010, 0.020  # an item's centre from its home spot (m)


def _tilt_deg(st) -> float:
    """Angle of the item's axis from vertical, from its orientation."""
    qx, qy = float(st.quat_xyzw[0]), float(st.quat_xyzw[1])
    c = float(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _tall(st) -> float:
    return float(st.high[2] - st.low[2])


def _standing(st, h) -> bool:
    """Upright within 10 degrees, and as tall as when standing (top at least h - 5 mm above its bottom)."""
    return _tilt_deg(st) <= UPRIGHT_DEG and _tall(st) >= h - 0.005


def _knocked_over(st, h) -> bool:
    return _tilt_deg(st) > FALLEN_DEG or _tall(st) < LYING_FRAC * h or st.low[2] < TOP - 0.02


def _on_shelf(st, h) -> bool:
    """Standing on the shelf board: centre over the board in front of the wall, bottom on the board's top."""
    x, y = st.pos[0], st.pos[1]
    return bool(
        _standing(st, h)
        and SHELF_X0 < x < WALL_X0 and SHELF_Y0 < y < SHELF_Y1
        and abs(st.low[2] - SHELF_TOP) <= BOTTOM_TOL
        and st.resting()
    )


def _on_mat(st) -> bool:
    """Standing on the mat, its whole outline seen from above inside the mat's edges, at rest."""
    mx, my = MAT_XY
    h = MAT_SIZE / 2
    return bool(
        _standing(st, JAR_H)
        and st.low[0] >= mx - h and st.high[0] <= mx + h and st.low[1] >= my - h and st.high[1] <= my + h
        and abs(st.low[2] - MAT_TOP) <= BOTTOM_TOL
        and st.resting()
    )


def _worst_home_offset(objects) -> float:
    """Largest distance from a home spot to the item filling it, matching identical items to spots of their own
    kind in the way that fits best (m, horizontal)."""
    worst = 0.0
    for homes in GROUPS:
        spots = np.array(list(homes.values()))
        items = np.array([objects[n].pos[:2] for n in homes])
        d = np.linalg.norm(items[:, None, :] - spots[None, :, :], axis=2)
        n = len(spots)
        worst = max(worst, min(max(d[i, p[i]] for i in range(n)) for p in itertools.permutations(range(n))))
    return float(worst)


# Trial history: whether anything has been knocked over since the trial started. The scorer sees one step at a
# time, so it is kept here and cleared when every item is back at its starting pose (a new trial).
_HISTORY = {"knocked_over": False}


def _at_start(objects) -> bool:
    for o in ITEMS:
        st = objects[o.name]
        if np.linalg.norm(st.pos - np.asarray(o.pos)) > 0.002 or _tilt_deg(st) > 1.0:
            return False
    return True


def _update(objects) -> None:
    if _at_start(objects):
        _HISTORY["knocked_over"] = False
    if any(_knocked_over(objects[n], HEIGHT[n]) for n in ALL_ITEMS):
        _HISTORY["knocked_over"] = True


def _tier(objects) -> float:
    _update(objects)
    if _HISTORY["knocked_over"] or not _on_mat(objects["blue_jar"]):
        return 0.0
    if not all(_on_shelf(objects[n], HEIGHT[n]) for n in SHELF_ITEMS):
        return 0.0
    worst = _worst_home_offset(objects)
    if worst <= HOME_TOL:
        return 1.0
    return 0.5 if worst <= NEAR_TOL else 0.0


def success(objects):
    """The blue jar standing on the mat; the two white jars and three bottles standing on the shelf, each home
    spot filled to within 10 mm; everything at rest; nothing knocked over during the trial."""
    return _tier(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(objects["blue_jar"].pos[:2] - np.asarray(MAT_XY)))


def score(objects):
    """1.0 as success; 0.5 if every home spot is filled to within 20 mm instead; 0 otherwise, and 0 for the whole
    trial once anything has been knocked over."""
    return _tier(objects)


# ---- scripted demo (left arm) ----------------------------------------------------------------------------
OPEN_PART = -0.038  # fingers part open: pads 76 mm apart, outer faces 110 mm apart; fits the 40 mm gaps
GRIP_BELOW_TOP = 0.035  # grip each item with the TCP 35 mm below its top
LIFT = 0.015  # carry an item with its bottom 15 mm above the surface it left
DROP = 0.003  # release an item with its bottom 3 mm above the surface
PARK = (0.18, 0.27)  # where the demo stands the middle bottle while it takes the blue jar out
OUT_X = 0.21  # in front of the shelf
BACK_OUT = (PARK[0] - 0.07, PARK[1])  # in front of the parked bottle, fingers clear of it
AISLE = (PARK[0] - 0.07, ROW_Y)  # beside that, so moves to and from the shelf pass clear of the parked bottle
YAW_X = -np.pi / 2  # fingers closing along x (front to back); +pi/2 needs a wrist past its limit at the mat
JAR_HOVER = SHELF_TOP + JAR_H + 0.03  # finger tips 17 mm above the jar tops
# Measured on the L40S with the trace wrapper: at the bottles' grip height (165 mm above the table) over the front
# of the shelf, the left arm's fingers settle about 15 mm to the robot's right (-y) of the target. The demo aims that
# much to the left whenever it grips or sets down the middle bottle on the shelf.
SAG_Y = 0.015
CLEAR, CLEAR_Z = (0.17, 0.26), TOP + 0.20  # in front of the shelf, above the bottle tops


def _wp(t, xy, z, f, label, yaw=0.0):
    return Waypoint(t, (float(xy[0]), float(xy[1]), float(z)), f, label, yaw)


def demo():
    """The bottles' tops are above the height a straight-down grip reaches over the shelf, so the demo takes the
    middle bottle from the front: it slides its open fingers in level with the bottle's top, either side of it."""
    hx, hy = BOTTLE_HOMES["bottle_middle"]
    bm = (hx, hy + SAG_Y)  # aimed off so the fingers actually close centred on the bottle's spot
    blue = (BACK_X, BACK_Y[1])
    b_shelf = SHELF_TOP + BOTTLE_H - GRIP_BELOW_TOP
    b_table = TOP + BOTTLE_H - GRIP_BELOW_TOP
    j_shelf = SHELF_TOP + JAR_H - GRIP_BELOW_TOP
    j_mat = MAT_TOP + JAR_H - GRIP_BELOW_TOP
    w = [
        # 1. Take the middle bottle off the shelf and stand it on the table. The arm starts above and just behind
        # the left bottle, so it first pulls back in front of the shelf before coming down.
        _wp(1.2, CLEAR, CLEAR_Z, OPEN_PART, "back from the start pose, clear of the bottles"),
        _wp(1.5, (OUT_X, bm[1]), b_shelf, OPEN_PART, "in front of the middle bottle"),
        _wp(1.5, bm, b_shelf, OPEN_PART, "slide the fingers either side of it"),
        _wp(0.8, bm, b_shelf, FINGER_CLOSED, "grip"),
        _wp(0.8, bm, b_shelf + LIFT, FINGER_CLOSED, "lift"),
        _wp(1.5, (OUT_X, bm[1]), b_shelf + LIFT, FINGER_CLOSED, "draw out"),
        _wp(1.5, PARK, b_table + LIFT, FINGER_CLOSED, "to the park spot"),
        _wp(0.8, PARK, b_table + DROP, FINGER_CLOSED, "set down"),
        _wp(0.6, PARK, b_table + DROP, OPEN_PART, "release"),
        _wp(1.0, BACK_OUT, b_table + DROP, OPEN_PART, "back out"),
        _wp(1.0, AISLE, b_table + DROP, OPEN_PART, "sideways, clear of the parked bottle"),
        # 2. Take the blue jar out through the gap and stand it on the mat.
        _wp(1.2, (OUT_X, blue[1]), JAR_HOVER, OPEN_PART, "in front of the gap", YAW_X),
        _wp(1.5, blue, JAR_HOVER, OPEN_PART, "over the blue jar", YAW_X),
        _wp(1.2, blue, j_shelf, OPEN_PART, "descend", YAW_X),
        _wp(0.8, blue, j_shelf, FINGER_CLOSED, "grip", YAW_X),
        _wp(0.8, blue, j_shelf + LIFT, FINGER_CLOSED, "lift", YAW_X),
        _wp(1.5, (OUT_X, blue[1]), j_shelf + LIFT, FINGER_CLOSED, "draw out", YAW_X),
        _wp(1.5, MAT_XY, j_mat + LIFT, FINGER_CLOSED, "to the mat", YAW_X),
        _wp(0.8, MAT_XY, j_mat + DROP, FINGER_CLOSED, "set down", YAW_X),
        _wp(0.6, MAT_XY, j_mat + DROP, OPEN_PART, "release", YAW_X),
        _wp(1.0, MAT_XY, JAR_HOVER, OPEN_PART, "up", YAW_X),
        # 3. Put the middle bottle back on its spot.
        _wp(1.5, AISLE, b_table, OPEN_PART, "round to the parked bottle"),
        _wp(1.0, BACK_OUT, b_table, OPEN_PART, "in front of the parked bottle"),
        _wp(1.2, PARK, b_table, OPEN_PART, "slide the fingers either side of it"),
        _wp(0.8, PARK, b_table, FINGER_CLOSED, "grip"),
        _wp(0.8, PARK, b_shelf + LIFT, FINGER_CLOSED, "lift"),
        _wp(1.5, (OUT_X, bm[1]), b_shelf + LIFT, FINGER_CLOSED, "in front of its spot"),
        _wp(1.5, bm, b_shelf + LIFT, FINGER_CLOSED, "slide over its spot"),
        _wp(0.8, bm, b_shelf + DROP, FINGER_CLOSED, "set down"),
        _wp(0.6, bm, b_shelf + DROP, OPEN_PART, "release"),
        _wp(1.2, (OUT_X, bm[1]), b_shelf + DROP, OPEN_PART, "slide the fingers back out"),
    ]
    return {"left": w}
