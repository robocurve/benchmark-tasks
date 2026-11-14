"""On the station table, between the two arms: a long narrow green board (48 x 6.4 x 1.9 cm, a cut length of
pine 1x3) lies along the forward axis on the midline. Its near end is 0.34 m in front of the arm bases; its far end
is 0.82 m out, near the table's far edge. A red spice jar (4.5 cm across, 6 cm tall) stands upright on the
board's far end, 0.78 m out. A fixed open blue basket (15 x 15 cm, 6 cm tall walls) sits on the table in front of
the left arm. The board and the jar are loose; the basket is fixed.

The jar is beyond either arm's reach in any gripper orientation. The board's near end is within reach of both
arms. Sliding the board toward the arms carries the jar with it, resting on the board.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \\
           --instruction "Put the spice jar in the basket." --scorer task_score ...
"""

import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# "end": the score is the jar's final state. With "best" a jar flung toward the arms and off the table (or dropped
# on the floor after reaching the zone) would keep 0.4, which contradicts "0 if the jar falls off the table".
SCORE_OVER = "end"

REST_GAP = 0.001  # loose objects spawn this far above their support and settle onto it
TABLE_X = (-0.19, 0.91)  # the station table's footprint (station.TABLE_CENTER +- TABLE_SIZE / 2)
TABLE_Y = (-0.65, 0.65)

# -- the board: a pine 1x3 (19 x 64 mm actual) cut to 48 cm, lying along x on the midline -------------------------
BOARD_LEN, BOARD_W, BOARD_H = 0.48, 0.064, 0.019
BOARD_NEAR_X = 0.34
BOARD = Box(
    "board",
    pos=(BOARD_NEAR_X + BOARD_LEN / 2, 0.0, TOP + REST_GAP + BOARD_H / 2),
    size=(BOARD_LEN, BOARD_W, BOARD_H),
    mass=0.27,
    color=(0.25, 0.55, 0.30),
)
BOARD_TOP = TOP + BOARD_H  # once settled

# -- the jar: a small glass spice jar, upright on the board's far end -------------------------------------------------
JAR_R, JAR_H = 0.0225, 0.06  # squat (h/d 1.3): an 8 cm jar rocks on its rim while dragged
JAR_X0 = 0.78
JAR = Cylinder(
    "spice_jar",
    pos=(JAR_X0, 0.0, BOARD.pos[2] + BOARD_H / 2 + REST_GAP + JAR_H / 2),
    radius=JAR_R,
    height=JAR_H,
    mass=0.10,
    color=(0.75, 0.12, 0.12),
)

# -- the basket: a fixed open plastic basket, inner 14 x 14 cm, rim 6 cm above the table, in front of the left arm --
BASKET_XY = (0.30, 0.25)
BASKET_INNER = 0.140
WALL_T = 0.006
FLOOR_T = 0.004
WALL_H = 0.056
OUTER = BASKET_INNER + 2 * WALL_T
FLOOR_TOP = TOP + FLOOR_T
RIM = FLOOR_TOP + WALL_H
BLUE = (0.20, 0.40, 0.80)
_bx, _by = BASKET_XY
_wz = FLOOR_TOP + WALL_H / 2
_off = BASKET_INNER / 2 + WALL_T / 2
BASKET = [
    Box("basket_floor", pos=(_bx, _by, TOP + FLOOR_T / 2), size=(OUTER, OUTER, FLOOR_T), color=BLUE),
    Box("basket_wall_near", pos=(_bx - _off, _by, _wz), size=(WALL_T, OUTER, WALL_H), color=BLUE),
    Box("basket_wall_far", pos=(_bx + _off, _by, _wz), size=(WALL_T, OUTER, WALL_H), color=BLUE),
    Box("basket_wall_right", pos=(_bx, _by - _off, _wz), size=(BASKET_INNER, WALL_T, WALL_H), color=BLUE),
    Box("basket_wall_left", pos=(_bx, _by + _off, _wz), size=(BASKET_INNER, WALL_T, WALL_H), color=BLUE),
]

OBJECTS = [BOARD, JAR, *BASKET]

# -- scoring -----------------------------------------------------------------------------------------------------------
REACH_X = 0.40  # "within reach": the jar's centre at most 0.40 m in front of the arm bases ...
REACH_Y = 0.40  # ... and at most 0.40 m to either side of the midline (both arms reach straight down there)
FALLEN_Z = TOP - 0.02  # the jar's bottom more than 2 cm below the table top: it has gone off the table
FLOOR_SLACK = 0.01  # "in the basket": the jar's bottom within 1 cm of the basket floor
GRIPPER_CLEAR = 0.03  # let go: both TCPs at least 3 cm from the jar's bounding box


def _in_basket(jar) -> bool:
    """The jar's centre over the basket's inner 14 x 14 cm and its bottom down on the basket floor."""
    dx, dy = jar.pos[0] - BASKET_XY[0], jar.pos[1] - BASKET_XY[1]
    half = BASKET_INNER / 2
    return bool(abs(dx) <= half and abs(dy) <= half and FLOOR_TOP - 0.005 <= jar.low[2] <= FLOOR_TOP + FLOOR_SLACK)


def _fallen(jar) -> bool:
    x, y = jar.pos[0], jar.pos[1]
    off_footprint = not (TABLE_X[0] <= x <= TABLE_X[1] and TABLE_Y[0] <= y <= TABLE_Y[1])
    return bool(jar.low[2] < FALLEN_Z or off_footprint)


def _within_reach(jar) -> bool:
    return bool(jar.pos[0] <= REACH_X and abs(jar.pos[1]) <= REACH_Y)


def tcp_points() -> list:
    """World TCP positions of both arms this step, read from the embodiment up the call stack; [] offline."""
    f = sys._getframe(1)
    while f is not None:
        s = f.f_locals.get("self")
        if s is not None and hasattr(s, "_held") and getattr(s, "_base", None) is not None:
            from yam_arena import ik, runtime, station

            scene = s._base.scene
            return [ik.tcp_pose(runtime.arm_q(scene[f"{a}_robot"]), station.BASES[a])[0] for a in ("left", "right")]
        f = f.f_back
    return []


def _released(jar, tcps) -> bool:
    """Both grippers have let go: each TCP at least GRIPPER_CLEAR from the jar. No TCPs to read: not released."""
    return bool(tcps) and all(
        np.linalg.norm(np.asarray(p, float) - np.clip(p, jar.low, jar.high)) >= GRIPPER_CLEAR for p in tcps
    )


def success(objects):
    """The jar rests in the basket: centre over the inner 14 x 14 cm, bottom within 1 cm of the floor, still,
    and both grippers have let go of it."""
    jar = objects["spice_jar"]
    return _in_basket(jar) and jar.resting() and _released(jar, tcp_points())


def distance(objects):
    """Metres from the jar's centre to where it stands upright in the middle of the basket."""
    goal = np.array([BASKET_XY[0], BASKET_XY[1], FLOOR_TOP + JAR_H / 2])
    return float(np.linalg.norm(objects["spice_jar"].pos - goal))


def score(objects):
    """Final-state credit when the run ends without success (1.0 comes only from the runner's success ending, after
    success() has held for 1 s): 0.4 on the board or the table (or held above it, the basket included) with its centre
    within 0.40 m in front of the arm bases and 0.40 m of the midline. 0: anywhere else, or off the table."""
    jar = objects["spice_jar"]
    if _fallen(jar):
        return 0.0
    return 0.4 if _within_reach(jar) else 0.0


# -- demo (left arm only; the right arm holds its ready pose) ---------------------------------------------------------
BOARD_GRIP_Z = TOP + 0.019  # the TCP level with the board's top: pad tips 6 mm above the table, 13 mm of pad on it
GRIP_X = 0.37  # where the left hand takes the board, 3 cm in from its near end at the start
PULL1_TO = 0.07  # first stroke 30 cm (jar 0.78 -> 0.48 m: still out of straight-down reach)
PULL2_TO = 0.19  # re-grip at GRIP_X, second stroke 18 cm (jar -> 0.30 m)
HOVER_Z = TOP + 0.10
TRAVEL_Z = TOP + 0.13  # carrying height: the jar's bottom 10 cm above the table, 4 cm over the basket rim
# planned JAR_X0 - (GRIP_X - PULL1_TO) - (GRIP_X - PULL2_TO) = 0.30; in sim the board trails the hand ~5 mm per stroke,
# so the jar ends at x 0.309-0.312
JAR_END_X = 0.31
JAR_GRIP_Z = BOARD_TOP + JAR_H / 2  # the jar's centre once settled on the board
PLACE_Z = FLOOR_TOP + 0.008 + JAR_H / 2  # set down 8 mm above the basket floor, fingers inside the walls


def _stroke(x_from: float, x_to: float, pull_s: float, label: str) -> list[Waypoint]:
    """Take the board at x_from (fingers closing across its width), slide it to x_to at a gentle pace, let go."""
    return [
        Waypoint(1.5, (x_from, 0.0, BOARD_GRIP_Z), FINGER_OPEN, f"{label}: descend onto the board"),
        Waypoint(1.0, (x_from, 0.0, BOARD_GRIP_Z), FINGER_CLOSED, f"{label}: grip the board"),
        Waypoint(pull_s, (x_to, 0.0, BOARD_GRIP_Z), FINGER_CLOSED, f"{label}: slide the board toward the arms"),
        Waypoint(0.5, (x_to, 0.0, BOARD_GRIP_Z), FINGER_CLOSED, f"{label}: settle"),
        Waypoint(0.8, (x_to, 0.0, BOARD_GRIP_Z), FINGER_OPEN, f"{label}: let go"),
        Waypoint(1.0, (x_to, 0.0, HOVER_Z), FINGER_OPEN, f"{label}: rise"),
    ]


def demo():
    """Left arm: two hand-over-hand strokes on the board (30 cm in 10 s, then 18 cm in 8 s; peak speed under
    5 cm/s, peak acceleration under 0.02 m/s^2, so the jar rides on friction), then a top-down pick of the jar
    into the basket. The right arm stays in its ready pose."""
    left = [Waypoint(2.5, (GRIP_X, 0.0, HOVER_Z), FINGER_OPEN, "over the board's near end")]
    left += _stroke(GRIP_X, PULL1_TO, 10.0, "stroke 1")
    left += [Waypoint(2.0, (GRIP_X, 0.0, HOVER_Z), FINGER_OPEN, "back over the board")]
    left += _stroke(GRIP_X, PULL2_TO, 8.0, "stroke 2")
    left += pick_place_waypoints((JAR_END_X, 0.0, JAR_GRIP_Z), (*BASKET_XY, PLACE_Z), travel_z=TRAVEL_Z)
    return {"left": left}
