"""On the station table, a small maze of fixed grey walls taller than a loose crate. It lies between 12 and
57 cm in front of the arm bases, and from 25 cm to the right of the midpoint between the arms to 14 cm to its
left, mostly in front of the right arm. A straight corridor runs away from the robot, open at its near end;
partway along, a side corridor branches off to the robot's left and ends at a wall. Green tape on the tops of the
side corridor's walls marks the goal: the last part of its floor, up to the end wall. The straight corridor carries
on past the branch and ends at a wall (a dead end). The crate, a wooden box wider than the open gripper, stands in
the straight corridor near its open end.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Inner faces of the walls:
X_OPEN = 0.12  # near (open) end of the straight corridor
Y_R, Y_L = -0.24, -0.02  # straight corridor: its right and left walls (220 mm wide)
XB0, XB1 = 0.30, 0.44  # side corridor: its near and far walls (140 mm wide)
YE = 0.13  # side corridor's end wall
XE = 0.56  # straight corridor's end wall; XB1..XE is the dead end
WALL_H, WALL_T = 0.070, 0.010
CRATE = (0.100, 0.100, 0.050)
CRATE_MASS = 0.6
CRATE_START = (0.20, -0.13)
GOAL_Y0 = -0.005  # the goal: the side corridor's floor from 15 mm past the straight corridor's left wall to its end
SQUARE = ((XB0, XB1), (GOAL_Y0, YE))  # the goal area's x and y extents

GREY, WOOD, GREEN = (0.55, 0.57, 0.60), (0.72, 0.55, 0.35), (0.15, 0.65, 0.25)


def _wall(name, x0, x1, y0, y1):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + WALL_H / 2), size=(x1 - x0, y1 - y0, WALL_H), color=GREY)


T = WALL_T
WALLS = [
    _wall("wall_right", X_OPEN, XE + T, Y_R - T, Y_R),
    _wall("wall_left_near", X_OPEN, XB0, Y_L, Y_L + T),
    _wall("wall_left_far", XB1, XE + T, Y_L, Y_L + T),
    _wall("wall_dead_end", XE, XE + T, Y_R, Y_L),
    _wall("wall_side_near", XB0 - T, XB0, Y_L + T, YE),
    _wall("wall_side_far", XB1, XB1 + T, Y_L + T, YE),
    _wall("wall_side_end", XB0 - T, XB1 + T, YE, YE + T),
]
(_sx0, _sx1), (_sy0, _sy1) = SQUARE
# The goal is marked by green tape on the tops of the side corridor's walls, from GOAL_Y0 to the end wall, and
# along the end wall's top. A mark on the floor would be a step the crate's leading edge catches on.
_cap_z = TOP + WALL_H + 0.0005
GOAL_MARKS = [
    Box("goal_mark_near", pos=(XB0 - T / 2, (GOAL_Y0 + YE) / 2, _cap_z), size=(T, YE - GOAL_Y0, 0.001), color=GREEN),
    Box("goal_mark_far", pos=(XB1 + T / 2, (GOAL_Y0 + YE) / 2, _cap_z), size=(T, YE - GOAL_Y0, 0.001), color=GREEN),
    Box("goal_mark_end", pos=((XB0 + XB1) / 2, YE + T / 2, _cap_z), size=(XB1 - XB0 + 2 * T, T, 0.001), color=GREEN),
]
CRATE_BOX = Box("crate", pos=(*CRATE_START, TOP + CRATE[2] / 2), size=CRATE, mass=CRATE_MASS, color=WOOD)
OBJECTS = [*WALLS, *GOAL_MARKS, CRATE_BOX]

FLOOR_TOL = 0.002  # on the floor: bottom within 2 mm of the table top
LIFT_TOL = 0.020  # lifted: lowest point more than 20 mm above the table top, at any moment of the trial
EPS = 1e-6  # comparisons at the stated limits include the limit
FLAT_DEG = 5.0
BEND = ((XB0 + CRATE[0] / 2, XB1 - CRATE[0] / 2))  # crate centre x range that lines it up with the side corridor


def _tilt_deg(st) -> float:
    x, y, z, w = st.quat_xyzw / np.linalg.norm(st.quat_xyzw)
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _yaw(st) -> float:
    x, y, z, w = st.quat_xyzw / np.linalg.norm(st.quat_xyzw)
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _corners(st) -> np.ndarray:
    """The crate's four bottom corners (x, y), from its true size and yaw."""
    c, s = np.cos(_yaw(st)), np.sin(_yaw(st))
    hx, hy = CRATE[0] / 2, CRATE[1] / 2
    local = np.array([(hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)])
    return st.pos[:2] + local @ np.array([[c, s], [-s, c]])


def _on_square(st) -> bool:
    p = _corners(st)
    return bool(np.all((p[:, 0] >= _sx0 - EPS) & (p[:, 0] <= _sx1 + EPS) & (p[:, 1] >= _sy0 - EPS)
                       & (p[:, 1] <= _sy1 + EPS)))


# Trial history: whether the crate has been lifted, and whether it has entered the dead end, since the trial
# started. The scorer sees one step at a time, so it is kept here. It is cleared only on a real episode reset: the
# crate appears within 0.5 mm of its start pose, at rest, by a jump of more than 20 mm from where it was at the
# previous call (or at the first call). A robot carrying the crate back to its start moves it gradually, so that
# does not clear the record.
_HISTORY = {"lifted": False, "dead_end": False, "last": None}
_START_POS = np.array((*CRATE_START, TOP + CRATE[2] / 2))
DEAD_END_X = XB1  # the crate's centre past the side corridor's far wall line: in the dead end


def _update(st) -> None:
    last = _HISTORY["last"]
    jumped = last is None or float(np.linalg.norm(st.pos - last)) > 0.020
    if jumped and np.linalg.norm(st.pos - _START_POS) <= 0.0005 and _tilt_deg(st) < 1.0 and st.resting():
        _HISTORY["lifted"] = _HISTORY["dead_end"] = False
    _HISTORY["last"] = np.array(st.pos, float)
    if st.low[2] > TOP + LIFT_TOL:
        _HISTORY["lifted"] = True
    if st.pos[0] > DEAD_END_X + EPS:
        _HISTORY["dead_end"] = True


def _tier(objects) -> float:
    st = objects["crate"]
    _update(st)
    if _HISTORY["lifted"] or _HISTORY["dead_end"]:
        return 0.0
    if not (st.low[2] <= TOP + FLOOR_TOL and _tilt_deg(st) <= FLAT_DEG and st.resting()):
        return 0.0
    x, y = st.pos[:2]
    in_side = XB0 < x < XB1 and Y_L - EPS <= y < YE
    if in_side and _on_square(st):
        return 1.0
    if in_side:
        return 0.5
    if BEND[0] - EPS <= x <= BEND[1] + EPS and Y_R < y < Y_L:
        return 0.25
    return 0.0


def success(objects):
    """The crate wholly inside the goal (all four bottom corners between the green marks), flat on the floor and at
    rest; never lifted and never in the dead end during the trial."""
    return _tier(objects) == 1.0


def distance(objects):
    goal = np.array(((_sx0 + _sx1) / 2, (YE - CRATE[1] / 2 + _sy0 + CRATE[1] / 2) / 2))
    return float(np.linalg.norm(objects["crate"].pos[:2] - goal))


def score(objects):
    """1.0 as success. 0.5: the crate's centre in the side corridor (past the line of the straight corridor's left
    wall) but not wholly inside the goal. 0.25: the crate in the straight corridor, lined up with the side corridor
    (centre 350 to 390 mm out). 0 otherwise (still near the start, out of the maze, tipped), and 0 for the whole
    trial once the crate has been lifted more than 20 mm or its centre has passed 440 mm out (into the dead end)."""
    return _tier(objects)


# ---- scripted demo (right arm) -----------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
PUSH_Z = TOP + 0.008 + TIP  # finger tips 8 mm above the table
CLEAR_Z = TOP + WALL_H + TIP + 0.020  # finger tips 20 mm above the wall tops
FRONT = 0.030  # finger front ahead of the TCP along x (yaw 0), at crate height
SIDE = 0.019  # finger half thickness along y (yaw 0)
TURN_X = (BEND[0] + BEND[1]) / 2  # 0.37: crate centre lined up with the side corridor
# Measured on the L40S with the trace wrapper: pushing the 0.6 kg crate along x, the right arm's finger stops about
# 40 mm short of its target (joint compliance under the push load). The first push aims that much further.
LAG_X = 0.040
LAG_Y = 0.030  # the second push ends against the end wall; aiming past it keeps the crate pressed home


def demo():
    cx, cy = CRATE_START
    behind = cx - CRATE[0] / 2 - FRONT - 0.012
    end_x = TURN_X - CRATE[0] / 2 - FRONT + LAG_X
    gap_y = (Y_R + (cy - CRATE[1] / 2)) / 2  # midway between the right wall and the crate's right face
    end_y = YE - CRATE[1] - SIDE + LAG_Y
    w = [
        Waypoint(1.5, (behind, cy, CLEAR_Z), FINGER_CLOSED, "behind the crate"),
        Waypoint(1.2, (behind, cy, PUSH_Z), FINGER_CLOSED, "down"),
        Waypoint(3.5, (end_x, cy, PUSH_Z), FINGER_CLOSED, "push to the bend"),
        Waypoint(0.8, (end_x - 0.01, cy, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.2, (end_x - 0.01, cy, CLEAR_Z), FINGER_CLOSED, "up"),
        Waypoint(1.5, (TURN_X, gap_y, CLEAR_Z), FINGER_CLOSED, "over the gap beside it"),
        Waypoint(1.5, (TURN_X, gap_y, PUSH_Z), FINGER_CLOSED, "down beside it"),
        Waypoint(4.0, (TURN_X, end_y, PUSH_Z), FINGER_CLOSED, "push into the side corridor"),
        Waypoint(0.8, (TURN_X, end_y - 0.04, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.2, (TURN_X - 0.04, end_y - 0.04, CLEAR_Z), FINGER_CLOSED, "up"),
    ]
    return {"right": w}
