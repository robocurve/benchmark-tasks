"""On the station table, on the midline between the arms, 27 cm in front of the arm bases: a newly planted tree, shown
as a fixed dark-brown trunk (4 cm across, 20 cm tall) standing in the centre of a fixed tan root-ball disc (18 cm
across, 1 cm tall). Further out on the left, a fixed blue wind-axis strip (22 x 2 cm, 3 mm thick) is taped to the
table, its long side pointing forward and to the left at 45 degrees; it marks an axis, not a direction. In front of
the right arm is a fixed dark grey nursery tray (inside 14 x 9 cm, 5 mm floor, 1.5 cm rim) holding two loose orange
stakes (upright cylinders 4 cm across, 12 cm tall, 100 g each). Staked, the two stakes stand upright on the table on
opposite sides of the trunk, outside the root ball (centres 11 to 16 cm from the trunk's axis), on a line at right
angles to the strip: one in front of and to the right of the trunk, the other nearer the arms and to its left, on the
left arm's side.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The tree: a fixed trunk standing through the centre of a fixed root-ball disc.
TRUNK_X, TRUNK_Y = 0.27, 0.0
TRUNK_R, TRUNK_H = 0.02, 0.20
ROOT_R, ROOT_H = 0.09, 0.01
TRUNK_BROWN = (0.30, 0.16, 0.06)
ROOT_TAN = (0.70, 0.50, 0.22)
TREE = [
    Cylinder("root_ball", pos=(TRUNK_X, TRUNK_Y, TOP + ROOT_H / 2), radius=ROOT_R, height=ROOT_H, color=ROOT_TAN),
    Cylinder("trunk", pos=(TRUNK_X, TRUNK_Y, TOP + TRUNK_H / 2), radius=TRUNK_R, height=TRUNK_H, color=TRUNK_BROWN),
]

# The wind-axis strip: its long side at 45 degrees (forward and to the left). Only its axis matters.
WIND_YAW_DEG = 45.0
STRIP_SIZE = (0.22, 0.02, 0.003)
STRIP_XY = (0.47, 0.22)  # its nearer end is 19 cm from the trunk's axis, clear of every stake spot
STRIP_BLUE = (0.10, 0.30, 0.90)
STRIP = Box("wind_strip", pos=(*STRIP_XY, TOP + STRIP_SIZE[2] / 2), size=STRIP_SIZE, color=STRIP_BLUE,
            yaw=math.radians(WIND_YAW_DEG))

# The nursery tray, in front of the right arm: a 5 mm floor and four 5 mm walls, 1.5 cm tall, around 14 x 9 cm.
TRAY_X, TRAY_Y = 0.18, -0.30
TRAY_IN = (0.14, 0.09)
TRAY_WALL_T, TRAY_WALL_H, TRAY_FLOOR_T = 0.005, 0.015, 0.005
TRAY_FLOOR = TOP + TRAY_FLOOR_T
TRAY_GREY = (0.22, 0.23, 0.26)
_TX = (TRAY_X - TRAY_IN[0] / 2, TRAY_X + TRAY_IN[0] / 2)
_TY = (TRAY_Y - TRAY_IN[1] / 2, TRAY_Y + TRAY_IN[1] / 2)
_TWZ = TOP + TRAY_WALL_H / 2
TRAY = [
    Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + TRAY_FLOOR_T / 2), size=(*TRAY_IN, TRAY_FLOOR_T), color=TRAY_GREY),
    Box("tray_wall_near", pos=(_TX[0] - TRAY_WALL_T / 2, TRAY_Y, _TWZ),
        size=(TRAY_WALL_T, TRAY_IN[1] + 2 * TRAY_WALL_T, TRAY_WALL_H), color=TRAY_GREY),
    Box("tray_wall_far", pos=(_TX[1] + TRAY_WALL_T / 2, TRAY_Y, _TWZ),
        size=(TRAY_WALL_T, TRAY_IN[1] + 2 * TRAY_WALL_T, TRAY_WALL_H), color=TRAY_GREY),
    Box("tray_wall_left", pos=(TRAY_X, _TY[1] + TRAY_WALL_T / 2, _TWZ), size=(TRAY_IN[0], TRAY_WALL_T, TRAY_WALL_H),
        color=TRAY_GREY),
    Box("tray_wall_right", pos=(TRAY_X, _TY[0] - TRAY_WALL_T / 2, _TWZ), size=(TRAY_IN[0], TRAY_WALL_T, TRAY_WALL_H),
        color=TRAY_GREY),
]

# The stakes: standing in the tray, one behind the other, 8 cm apart (4 cm between them).
STAKE_R, STAKE_H, STAKE_KG = 0.02, 0.12, 0.1
STAKE_ORANGE = (0.95, 0.42, 0.05)
STAKE_START = {"stake_a": (TRAY_X + 0.04, TRAY_Y), "stake_b": (TRAY_X - 0.04, TRAY_Y)}
STAKES = [
    Cylinder(name, pos=(x, y, TRAY_FLOOR + STAKE_H / 2 + 0.001), radius=STAKE_R, height=STAKE_H, mass=STAKE_KG,
             color=STAKE_ORANGE)
    for name, (x, y) in STAKE_START.items()
]  # each starts 1 mm above the tray floor, and settles
STAKE_NAMES = tuple(STAKE_START)

OBJECTS = [*TREE, STRIP, *TRAY, *STAKES]
SUCCESS_HOLD_S = 2.5  # staked and still for 2.5 s

R_MIN, R_MAX = 0.11, 0.16  # a stake's centre this far from the trunk's axis (m); R_MIN = ROOT_R + STAKE_R
ROOT_CONTACT_Z = 0.015  # the root-ball rule: any part of a stake within ROOT_R of the trunk's axis in plan, with its
# lowest point at most 1.5 cm above the table top
UPRIGHT_DEG = 10.0  # a stake tilted at most 10 degrees from vertical, either end up
ON_TABLE_TOL = 0.003  # a stake's bottom within 3 mm of the table top (not on the disc, the tray or the other stake)
OPPOSITE_TOL_DEG = 12.0  # the two stakes 180 +- 12 degrees apart around the trunk
ACROSS_TOL_DEG = 12.0  # the line between them 90 +- 12 degrees to the strip's long side
STAKE_CREDIT, OPPOSITE_CREDIT, ACROSS_CREDIT = 0.2, 0.2, 0.4


def _tilt_deg(state) -> float:
    """Angle between the stake's long axis and the vertical, either end up (a stake is the same at both ends)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _from_trunk(state) -> tuple[float, float]:
    """The centre's plan offset from the trunk's axis."""
    return float(state.pos[0]) - TRUNK_X, float(state.pos[1]) - TRUNK_Y


def _dist(state) -> float:
    return math.hypot(*_from_trunk(state))


def _seg_dist(p, q) -> float:
    """Plan distance from the trunk's axis to the segment p-q (plan points relative to the trunk)."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    n = dx * dx + dy * dy
    t = 0.0 if n < 1e-12 else max(0.0, min(1.0, -(p[0] * dx + p[1] * dy) / n))
    return math.hypot(p[0] + t * dx, p[1] + t * dy)


def _nearest_reach(state) -> float:
    """Plan distance from the trunk's axis to the nearest point of the stake, whatever its pose: the stake's side
    lines (one per point on its rim, plus the rim point facing the trunk), each from its bottom to its top end."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    u = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))  # the stake's own x axis in the world
    v = (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x))  # its y axis
    a = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))  # its long axis
    cx, cy = _from_trunk(state)
    to_trunk = (-cx, -cy, 0.0)
    ta = sum(t * s for t, s in zip(to_trunk, a))
    perp = [t - ta * s for t, s in zip(to_trunk, a)]  # the direction to the trunk, across the stake's axis
    dirs = [(math.cos(2 * math.pi * k / 64), math.sin(2 * math.pi * k / 64)) for k in range(64)]
    pu, pv = sum(p * s for p, s in zip(perp, u)), sum(p * s for p, s in zip(perp, v))
    if math.hypot(pu, pv) > 1e-9:
        dirs.append((pu / math.hypot(pu, pv), pv / math.hypot(pu, pv)))
    best = _dist(state)
    for cu, cv in dirs:
        ox, oy = STAKE_R * (cu * u[0] + cv * v[0]), STAKE_R * (cu * u[1] + cv * v[1])
        hx, hy = STAKE_H / 2 * a[0], STAKE_H / 2 * a[1]
        best = min(best, _seg_dist((cx + ox - hx, cy + oy - hy), (cx + ox + hx, cy + oy + hy)))
    return best


def _in_root_ball(state) -> bool:
    """On or against the root ball: some part of the stake less than ROOT_R from the trunk's axis in plan (over the
    disc; for an upright stake, its centre under R_MIN) while its lowest point is at most ROOT_CONTACT_Z above the
    table top. A stake carried higher over the tree touches no roots."""
    return _nearest_reach(state) < ROOT_R and float(state.low[2]) - TOP <= ROOT_CONTACT_Z


def _stake_ok(state) -> bool:
    """Upright, standing on the table, and its centre R_MIN to R_MAX from the trunk's axis."""
    return bool(
        _tilt_deg(state) <= UPRIGHT_DEG
        and abs(float(state.low[2]) - TOP) <= ON_TABLE_TOL
        and R_MIN <= _dist(state) <= R_MAX
    )


def _angle_diff_deg(a: float, b: float) -> float:
    """|a - b| folded into 0..180 degrees."""
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def _opposite(a, b) -> bool:
    """The two stakes' bearings from the trunk 180 +- OPPOSITE_TOL_DEG apart."""
    (ax, ay), (bx, by) = _from_trunk(a), _from_trunk(b)
    ba, bb = math.degrees(math.atan2(ay, ax)), math.degrees(math.atan2(by, bx))
    return _angle_diff_deg(ba, bb) >= 180.0 - OPPOSITE_TOL_DEG


def _across_wind(a, b) -> bool:
    """The line between the two centres 90 +- ACROSS_TOL_DEG to the strip's long side (axis only, either way)."""
    dx, dy = float(b.pos[0] - a.pos[0]), float(b.pos[1] - a.pos[1])
    d = _angle_diff_deg(math.degrees(math.atan2(dy, dx)), WIND_YAW_DEG)
    d = min(d, 180.0 - d)  # angle between two axes: 0..90
    return d >= 90.0 - ACROSS_TOL_DEG


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each stake upright on the table 11 to 16 cm from the trunk's axis;
    with both so, 0.2 more if they are opposite each other around the trunk (180 +- 12 degrees), and then 0.4 more if
    the line between them is at right angles to the wind strip (90 +- 12 degrees). 0 overall if any part of a stake,
    seen from above, is under 9 cm from the trunk's axis (over the root-ball disc) while its lowest point is at most
    1.5 cm above the table (on or against the root ball, standing or lying)."""
    stakes = [objects[n] for n in STAKE_NAMES]
    if any(_in_root_ball(s) for s in stakes):
        return 0.0
    ok = [_stake_ok(s) for s in stakes]
    total = STAKE_CREDIT * sum(ok)
    if all(ok) and _opposite(*stakes):
        total += OPPOSITE_CREDIT
        if _across_wind(*stakes):
            total += ACROSS_CREDIT
    return round(total, 6)


def success(objects):
    """Both stakes placed, opposite and across the wind, and still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in STAKE_NAMES)


# The demo: the right arm stands stake B at a hand-off spot on the midline nearer the arms (17 cm from the trunk's
# axis, outside the scored ring), then stands stake A in front of and to the right of the trunk. Once the right arm
# has left the hand-off spot, the left arm picks stake B up there and stands it nearer the arms, left of the trunk.
# Both stakes go 13 cm from the trunk's axis, on the line at -45 degrees (at right angles to the strip).
# Every grip is straight down, the fingers closing along world y.
TRAVEL_Z = TOP + 0.15  # a carried stake's bottom clears the tray rim and the root ball by 4.5 cm or more
STAKE_GRIP = 0.09  # TCP above a stake's bottom: the pads cover its top 3.3 cm, its top 3 cm below the palm
DROP = 0.006  # each stake is let go this far above the table
PLACE_R = 0.13
_LINE = math.radians(WIND_YAW_DEG - 90.0)
RIGHT_SPOT = (TRUNK_X + PLACE_R * math.cos(_LINE), TRUNK_Y + PLACE_R * math.sin(_LINE))  # in front, to the right
LEFT_SPOT = (TRUNK_X - PLACE_R * math.cos(_LINE), TRUNK_Y - PLACE_R * math.sin(_LINE))  # nearer the arms, to the left
HANDOFF = (0.10, 0.0)  # 17 cm from the trunk's axis, on the midline both arms reach
RIGHT_PARK = (TRAY_X, TRAY_Y, TRAVEL_Z)  # above the empty tray
LEFT_PARK = (0.15, 0.30, TRAVEL_Z)  # in front of the left base
LEFT_WAIT_XY = (0.15, 0.25)  # where the left arm waits, clear of the hand-off spot
LEFT_WAIT_S = 11.0  # until the right arm has let go of stake B and moved back to the tray


def demo():
    right = []
    for name, spot in (("stake_b", HANDOFF), ("stake_a", RIGHT_SPOT)):
        right += pick_place_waypoints(
            (*STAKE_START[name], TRAY_FLOOR + STAKE_GRIP), (*spot, TOP + STAKE_GRIP + DROP), travel_z=TRAVEL_Z,
        )
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    left = [Waypoint(LEFT_WAIT_S, (*LEFT_WAIT_XY, TRAVEL_Z), FINGER_OPEN, "wait for the hand-off")]
    left += pick_place_waypoints(
        (*HANDOFF, TOP + STAKE_GRIP), (*LEFT_SPOT, TOP + STAKE_GRIP + DROP), travel_z=TRAVEL_Z,
    )
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
