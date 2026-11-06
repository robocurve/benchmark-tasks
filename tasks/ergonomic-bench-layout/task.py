"""On the station table, set up as an assembly bench: a worker (a fixed stand-in) sits on a stool just beyond the
table's far edge, facing the arms, in front of the right arm. The stand-in is a torso, a head and two forearms
resting flat on the table, pointing toward the arms. Between the worker's hands and the arms, a thin green pad
lies flat on the table: it marks the worker's reach zone, 310 mm along the worker's line of reach and 260 mm
across, 3 mm thick. Its far side faces the worker; its near side faces the arms.

Three closed parts bins stand loose on the table outside the pad: one large, one medium and one small. They are
the same blue plastic and filled with the same parts, so the bigger a bin, the heavier it is (300 g, 157 g and
69 g). The large one points toward the worker; the other two lie with their long sides across the line toward
the worker.

The goal: all three bins standing upright on the pad, the heaviest nearest the worker's body and the lightest
farthest from it, with at least 20 mm of clear space between neighbouring bins.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

TABLE_FAR_X = 0.91  # the station table's far edge (table 1.10 m deep, centred 0.36 m in front of the arm bases)
FLOOR_Z = -0.0125  # the station floor


def _box(name, x0, x1, y0, y1, z0, z1, color):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0), color=color)


# -- the worker stand-in (fixed), seated beyond the far edge, centred on the pad ------------------------------------
WORKER_Y = -0.21  # the worker's midline, in front of the right arm (the left arm's base is at y +0.305)
TORSO_X0, TORSO_X1 = 0.94, 1.16  # chest 30 mm clear of the table edge, torso 220 mm deep
TORSO_HALF_W = 0.18  # 360 mm across the shoulders
SEAT_Z = 0.45  # stool seat height above the floor
TORSO_TOP = TOP + 0.30  # shoulders 300 mm above the bench top
HEAD_R = 0.10
STOOL_R = 0.17
FOREARM_W, FOREARM_H = 0.07, 0.06
FOREARM_X0 = 0.53  # fingertips 380 mm from the table edge
FOREARM_DY = 0.14  # forearm centres this far either side of the midline
SHIRT = (0.35, 0.40, 0.45)
SKIN = (0.87, 0.70, 0.56)
STOOL = (0.15, 0.15, 0.16)
WORKER = [
    Cylinder("worker_stool", pos=(1.05, WORKER_Y, (FLOOR_Z + SEAT_Z) / 2), radius=STOOL_R, height=SEAT_Z - FLOOR_Z,
             color=STOOL),
    _box("worker_torso", TORSO_X0, TORSO_X1, WORKER_Y - TORSO_HALF_W, WORKER_Y + TORSO_HALF_W, SEAT_Z, TORSO_TOP,
         SHIRT),
    Sphere("worker_head", pos=((TORSO_X0 + TORSO_X1) / 2, WORKER_Y, TORSO_TOP + 0.005 + HEAD_R), radius=HEAD_R,
           color=SKIN),
    *(_box(f"worker_forearm_{side}", FOREARM_X0, TORSO_X0 - 0.005, WORKER_Y + s * FOREARM_DY - FOREARM_W / 2,
           WORKER_Y + s * FOREARM_DY + FOREARM_W / 2, TOP, TOP + FOREARM_H, SKIN)
      for side, s in (("left", -1), ("right", 1))),  # the worker's own left and right (facing -x)
]
# The point distances to the body are measured from: the middle of the chest, at the table edge's height.
BODY_XY = (TORSO_X0, WORKER_Y)

# -- the reach zone: a 3 mm foam pad, fixed ------------------------------------------------------------------------
ZONE_X0, ZONE_X1 = 0.17, 0.48  # 310 mm along the line of reach; ZONE_X1 is the side nearest the worker
ZONE_Y0, ZONE_Y1 = WORKER_Y - 0.13, WORKER_Y + 0.13  # 260 mm across
ZONE_T = 0.003
ZONE_TOP = TOP + ZONE_T
ZONE_COLOR = (0.25, 0.62, 0.35)
ZONE = _box("reach_pad", ZONE_X0, ZONE_X1, ZONE_Y0, ZONE_Y1, TOP, ZONE_TOP, ZONE_COLOR)

# -- the parts bins (loose): closed boxes of one material, so mass grows with size ---------------------------------
REST_GAP = 0.001  # loose objects spawn this far above their support and settle onto it
BIN_COLOR = (0.16, 0.36, 0.72)
FILL_DENSITY = 347.0  # kg/m^3 over the outside volume: the same parts and the same fill in every bin
# name -> (width across the grip, length, height): width along x and length along y at yaw 0
BIN_DIMS = {
    "bin_large": (0.064, 0.180, 0.075),
    "bin_medium": (0.054, 0.140, 0.060),
    "bin_small": (0.044, 0.100, 0.045),
}
BIN_NAMES = tuple(BIN_DIMS)  # heaviest first
BIN_MASS = {n: round(FILL_DENSITY * w * l * h, 3) for n, (w, l, h) in BIN_DIMS.items()}  # 0.300, 0.157, 0.069
# Starting spots (centre x, y) and yaw. None is on the pad; their order along the line of reach is not the goal's.
START = {
    "bin_large": ((0.30, 0.03), math.pi / 2),  # beside the pad toward the midline, pointing at the worker
    "bin_medium": ((0.12, WORKER_Y), 0.0),  # in front of the pad (toward the arms), lying across
    "bin_small": ((0.14, 0.02), 0.0),  # in front of the pad, toward the midline, lying across
}
BINS = [
    Box(n, pos=(*START[n][0], TOP + REST_GAP + BIN_DIMS[n][2] / 2), size=BIN_DIMS[n], mass=BIN_MASS[n],
        color=BIN_COLOR, yaw=START[n][1])
    for n in BIN_NAMES
]
START_XY = {n: START[n][0] for n in BIN_NAMES}

OBJECTS = [*WORKER, ZONE, *BINS]

# -- scoring -------------------------------------------------------------------------------------------------------
ON_PAD_TOL = 0.015  # "on the pad": lowest point at most 15 mm above the pad's top (not held, not on another bin)
MAX_TILT_DEG = 20.0  # "upright": the bin's vertical axis within 20 deg of vertical (a tipped bin spills its parts)
ORDER_MIN_GAP = 0.02  # a heavier bin counts as nearer only if it is at least 20 mm nearer the body point
SPACING = 0.02  # clear space between any two bins, seen from above
PLACE_CREDIT = 0.15  # per bin on the pad
ORDER_CREDIT = 0.15  # per correctly ordered pair, both on the pad (three pairs)
SPACING_CREDIT = 0.10  # all three on the pad and every pair at least 20 mm apart
PAIRS = tuple((a, b) for i, a in enumerate(BIN_NAMES) for b in BIN_NAMES[i + 1:])  # (heavier, lighter)


def tilt_deg(state) -> float:
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def off_table(state) -> bool:
    """Centre below the table top: the bin has fallen to the floor."""
    return float(state.pos[2]) < TOP


def over(state, other) -> bool:
    """``state``'s centre, seen from above, lies over ``other``'s footprint."""
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(other.low[0] <= x <= other.high[0] and other.low[1] <= y <= other.high[1])


def stacked(objects) -> bool:
    """Some bin rests on (or is held over and above) another bin."""
    return any(
        a != b and over(objects[a], objects[b]) and float(objects[a].low[2]) >= float(objects[b].high[2]) - 0.01
        for a in BIN_NAMES for b in BIN_NAMES
    )


def on_pad(state) -> bool:
    """Centre over the pad, lowest point within 15 mm of the pad's top, upright within 20 deg."""
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(
        ZONE_X0 <= x <= ZONE_X1 and ZONE_Y0 <= y <= ZONE_Y1
        and float(state.low[2]) <= ZONE_TOP + ON_PAD_TOL and tilt_deg(state) <= MAX_TILT_DEG
    )


def body_distance(state) -> float:
    """Horizontal distance (m) from the middle of the worker's chest to the bin's centre."""
    return math.hypot(float(state.pos[0]) - BODY_XY[0], float(state.pos[1]) - BODY_XY[1])


def gap(a, b) -> float:
    """Clear space between two bins seen from above (m); 0 or less if their footprints touch or overlap."""
    return float(max(a.low[0] - b.high[0], b.low[0] - a.high[0], a.low[1] - b.high[1], b.low[1] - a.high[1]))


def ordered(objects, heavier, lighter) -> bool:
    a, b = objects[heavier], objects[lighter]
    return on_pad(a) and on_pad(b) and body_distance(b) - body_distance(a) >= ORDER_MIN_GAP


def score(objects):
    """0 if a bin is on the floor or stacked on another. Otherwise 0.15 per bin on the pad, 0.15 per pair in the
    right order (heavier at least 20 mm nearer the worker's chest, both on the pad), and 0.10 if all three are on
    the pad with at least 20 mm between every pair."""
    if any(off_table(objects[n]) for n in BIN_NAMES) or stacked(objects):
        return 0.0
    placed = [n for n in BIN_NAMES if on_pad(objects[n])]
    s = PLACE_CREDIT * len(placed)
    s += ORDER_CREDIT * sum(ordered(objects, a, b) for a, b in PAIRS)
    if len(placed) == len(BIN_NAMES) and all(gap(objects[a], objects[b]) >= SPACING for a, b in PAIRS):
        s += SPACING_CREDIT
    return float(min(1.0, round(s, 6)))


def success(objects):
    """Full score with every bin still."""
    return bool(score(objects) == 1.0 and all(objects[n].resting() for n in BIN_NAMES))


def _gap1(value, lo, hi) -> float:
    return max(lo - value, 0.0, value - hi)


def distance(objects):
    """Metres: horizontal distance from the pad, summed over the bins, plus 0.1 m per pair out of order."""
    d = sum(
        math.hypot(_gap1(float(objects[n].pos[0]), ZONE_X0, ZONE_X1), _gap1(float(objects[n].pos[1]), ZONE_Y0, ZONE_Y1))
        for n in BIN_NAMES
    )
    return float(d + 0.1 * sum(not ordered(objects, a, b) for a, b in PAIRS))


# -- demo (right arm only) ------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.13  # TCP carry height: carried bins' bottoms 90-100 mm up, over the 75 mm large bin
APPROACH_CLEARANCE = 0.012  # each pad this far off the bin on the way down
LOOSEN = 0.010  # each pad backs off this far to let go
RELEASE_ABOVE = 0.003  # a bin is let go with its bottom 3 mm above the pad
LOWER_S = 1.6  # onto an open pad: no walls, default-like pace
# TCP height above a bin's bottom at the grip: fingertips 17-27 mm above the table, bin tops 15-35 mm above the TCP
# (19-39 mm under the finger carriage).
GRIP_ABOVE_BOTTOM = {"bin_large": 0.040, "bin_medium": 0.035, "bin_small": 0.030}
# Demo order and goal spots (centre x, y), nearest the worker first; every bin ends with its long side across the
# line of reach (yaw 0: width along x). Gaps along x: 56 mm and 61 mm, so a loosened finger (23 mm thick) beside a
# placed bin still clears it when that bin landed 20 mm off its aim.
GOAL = {"bin_large": (0.425, WORKER_Y), "bin_medium": (0.31, WORKER_Y), "bin_small": (0.20, WORKER_Y)}
PARK_XY = (0.15, -0.45)
PARK_Z = TOP + 0.20
PARK_YAW = 0.0  # joint 6 would pass its limit holding +90 deg out here
HOLD_S = 2.0


# Gripper yaw at the pick: the fingers close across the bin's width (at gripper yaw 0 they close along world y).
# +90 deg rather than -90 deg keeps joint 6 inside its +-120 deg range near the arm.
GRIP_YAW = {"bin_large": 0.0, "bin_medium": math.pi / 2, "bin_small": math.pi / 2}
PLACE_YAW = math.pi / 2  # at the goal every bin's width runs along x: the fingers close along x


def _finger(width: float, clearance: float) -> float:
    """Finger joint target that leaves ``clearance`` between each pad and a bin ``width`` wide."""
    return max(FINGER_OPEN, -(width / 2 + clearance))


def move_waypoints(name) -> list[Waypoint]:
    """Pick ``name`` straight down across its width, carry it at TRAVEL_Z turning it so its width runs along x,
    lower it onto the pad at its goal spot, open the pads a little and rise straight up."""
    (x, y), _ = START[name]
    qx, qy = GOAL[name]
    w = BIN_DIMS[name][0]
    g = GRIP_ABOVE_BOTTOM[name]
    approach, loose = _finger(w, APPROACH_CLEARANCE), _finger(w, LOOSEN)
    grasp_z = TOP + REST_GAP + g
    release_z = ZONE_TOP + RELEASE_ABOVE + g
    y0, y1 = GRIP_YAW[name], PLACE_YAW
    return [
        Waypoint(1.5, (x, y, TRAVEL_Z), approach, "to the bin", y0),
        Waypoint(1.5, (x, y, grasp_z), approach, "descend", y0),
        Waypoint(0.8, (x, y, grasp_z), FINGER_CLOSED, "close", y0),
        Waypoint(1.2, (x, y, TRAVEL_Z), FINGER_CLOSED, "lift", y0),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), FINGER_CLOSED, "carry", y1),
        Waypoint(LOWER_S, (qx, qy, release_z), FINGER_CLOSED, "lower", y1),
        Waypoint(0.6, (qx, qy, release_z), loose, "release", y1),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), loose, "retreat", y1),
    ]


def demo():
    """Right arm: the large bin to the pad's row nearest the worker, then the medium bin to the middle row, then
    the small bin to the row nearest the arms, each turned so its long side runs across the line of reach. No
    carried bin passes over a standing one. Then the arm parks off the pad to the right and holds still for 2 s.
    The left arm stays at rest."""
    plan = [w for name in BIN_NAMES for w in move_waypoints(name)]
    plan += [
        Waypoint(1.5, (*PARK_XY, PARK_Z), FINGER_OPEN, "park", PARK_YAW),
        Waypoint(HOLD_S, (*PARK_XY, PARK_Z), FINGER_OPEN, "hold still", PARK_YAW),
    ]
    return {"right": plan}
