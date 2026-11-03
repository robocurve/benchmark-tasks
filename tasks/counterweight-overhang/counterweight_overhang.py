"""On the station table, in front of the left arm: a fixed grey ledge, 12 cm deep, 5 cm tall and 26.5 cm long, runs
left to right. A loose yellow wooden plank (40 cm long, 10 cm wide, 2 cm thick, 0.4 kg) lies along the top of the
ledge. Its left end almost touches a fixed dark grey stop (2 cm thick, 4 cm tall) on the ledge's left end. Its right
end, the far end, sticks 16 cm out past the ledge's right-hand end, 5 cm above the table, with nothing under it. On
the table, nearer the arm: a full blue paint tin (7 cm across, 9 cm tall, 0.6 kg) on the left, and a red steel block
(a 5 cm cube, 1 kg) on the right. The ledge and stop are fixed; the plank, tin and block are loose.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x forward from the arms, y to the left, z up (m); the arm bases stand at x 0, y +-0.305. Everything is
# in the left arm's reach (straight-down IK map of 2026-11-03, and the real arm's box x 0.15-0.48, |y - 0.305| <= 0.25).
ROW_X = 0.29  # the ledge and plank run left to right (along y) at this x
EDGE_Y = 0.27  # the ledge's right-hand end: the edge the plank tips over
LEDGE_LEFT_Y, LEDGE_DEPTH, LEDGE_H = 0.535, 0.12, 0.05
STOP_Y, STOP_H = (0.513, 0.533), 0.04  # on the ledge, 3 mm past the plank's left end, so sliding the plank back is blocked

PLANK_LEN, PLANK_W, PLANK_T, PLANK_KG = 0.40, 0.10, 0.02, 0.40  # pine, about 500 kg/m^3
PLANK_FAR_Y = 0.11  # the far (right) end: 16 cm past the edge
PLANK_Y = PLANK_FAR_Y + PLANK_LEN / 2  # its centre, 4 cm behind the edge: it holds itself up, with little to spare
PLANK_TOP = TOP + LEDGE_H + PLANK_T

TIN_R, TIN_H, TIN_KG = 0.035, 0.09, 0.6
TIN_XY = (0.16, 0.44)
BLOCK_SIZE, BLOCK_KG = 0.05, 1.0
BLOCK_XY = (0.16, 0.20)

YELLOW, BLUE, RED = (0.95, 0.75, 0.10), (0.10, 0.35, 0.90), (0.85, 0.12, 0.12)  # saturated: they show in the top view
CONCRETE, DARK = (0.55, 0.55, 0.57), (0.25, 0.25, 0.27)

LEDGE = Box("ledge", pos=(ROW_X, (EDGE_Y + LEDGE_LEFT_Y) / 2, TOP + LEDGE_H / 2),
            size=(LEDGE_DEPTH, LEDGE_LEFT_Y - EDGE_Y, LEDGE_H), color=CONCRETE)
STOP = Box("stop", pos=(ROW_X, sum(STOP_Y) / 2, TOP + LEDGE_H + STOP_H / 2),
           size=(LEDGE_DEPTH, STOP_Y[1] - STOP_Y[0], STOP_H), color=DARK)
PLANK = Box("plank", pos=(ROW_X, PLANK_Y, TOP + LEDGE_H + PLANK_T / 2 + 0.0005),
            size=(PLANK_W, PLANK_LEN, PLANK_T), mass=PLANK_KG, color=YELLOW)
TIN = Cylinder("tin", pos=(*TIN_XY, TOP + TIN_H / 2 + 0.001), radius=TIN_R, height=TIN_H, mass=TIN_KG, color=BLUE)
BLOCK = Box("block", pos=(*BLOCK_XY, TOP + BLOCK_SIZE / 2 + 0.001), size=(BLOCK_SIZE,) * 3, mass=BLOCK_KG, color=RED)
OBJECTS = [LEDGE, STOP, PLANK, TIN, BLOCK]
LOOSE = {"plank": PLANK, "tin": TIN, "block": BLOCK}

# The target: the centre of the tin's base 2.5 to 8 cm from the plank's far end, measured along the plank, and within
# 3 cm of its middle line. From every spot in it, the tin alone tips the plank (test_the_tin_alone_tips_the_plank).
ZONE_NEAR, ZONE_FAR, ZONE_SIDE = 0.025, 0.08, 0.03

# The plank stands on its own when the centre of mass of it and everything resting on it lies at least 1 cm behind the
# edge. Physics decides whether the plank tips; this check is for what physics can't see: the sim can't tell a gripped
# object from a released one, so a gripper propping the plank or holding the tin would otherwise pass as balanced.
BALANCE_MARGIN = 0.01

# A fall: the plank tilting more than 3 degrees (resting, it is level; tipped, it lands at 18 degrees, its far end on
# the table), the tin leaning past 45 degrees, or the tin or block moving down faster than 1 m/s (a free fall of 5 cm
# or more; a tin falling off the plank's end reaches 1.2 m/s). Once anything falls, the trial scores 0: on a real
# scaffold the tin would be on the ground, and lifting it off the tipped plank lets the plank level again by itself.
TIP_DEG, TIN_OVER_DEG, DROP_SPEED = 3.0, 45.0, 1.0
LEVEL_DEG, UPRIGHT_DEG = 1.5, 10.0  # for success: the plank level, the tin upright

# Resting on the plank means within 2 mm of its top face: a tin or block held just above it doesn't count.
REST_GAP = 0.002

# The plank must stay where it starts, for 0.5 and 1.0: its centre within 5 mm, its length within 2 degrees of left to
# right. Turned about 16 degrees and pulled toward the arms, its left end passes beside the stop and more of it sits on
# the ledge, so the tin alone balances (found in review).
PLACE_TOL, PLACE_DEG = 0.005, 2.0


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _plank_tilt_deg(plank) -> float:
    """How far the plank's top face is turned from level (degrees)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, _rot(plank.quat_xyzw)[2, 2]))))


def _tin_tilt_deg(tin) -> float:
    """How far the tin's axis leans from vertical (degrees); a tin on its lid counts as upright."""
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(_rot(tin.quat_xyzw)[2, 2])))))


# Where the tin is comes from its centre and axis, never from low/high: Arena builds those from the corners of the
# tin's local box, which swing out when it spins on its axis (docs/robocurve/rules.md §3).
def _tin_base(tin) -> tuple[float, float, float]:
    """x, y of the centre of the tin's lower end, and the z of its lowest point (exact for a cylinder)."""
    axis = _rot(tin.quat_xyzw)[:, 2]
    c = min(1.0, abs(float(axis[2])))
    sign = -1.0 if axis[2] >= 0 else 1.0  # the end that is lower
    lowest = float(tin.pos[2]) - (TIN_H / 2 * c + TIN_R * math.sqrt(1.0 - c * c))
    return float(tin.pos[0]) + sign * axis[0] * TIN_H / 2, float(tin.pos[1]) + sign * axis[1] * TIN_H / 2, lowest


def _on_plank(plank, x: float, y: float, lowest_z: float) -> tuple[float, float] | None:
    """If something whose lowest point is at lowest_z, over (x, y), rests on the plank's top face: its distance from
    the plank's far end along the plank, and its offset from the plank's middle line (m). Otherwise None."""
    rot = _rot(plank.quat_xyzw)
    along, across, normal = rot[:, 1], rot[:, 0], rot[:, 2]
    top = np.asarray(plank.pos, dtype=float) + normal * PLANK_T / 2
    top_z = top[2] - (normal[0] * (x - top[0]) + normal[1] * (y - top[1])) / normal[2]
    if not -0.003 <= lowest_z - top_z <= REST_GAP:
        return None
    offset = np.array([x, y, top_z]) - np.asarray(plank.pos, dtype=float)
    from_far_end, side = float(offset @ along) + PLANK_LEN / 2, float(offset @ across)
    if not (0.0 <= from_far_end <= PLANK_LEN and abs(side) <= PLANK_W / 2):
        return None
    return from_far_end, side


def _tin_on_plank(objects):
    return _on_plank(objects["plank"], *_tin_base(objects["tin"]))


def _block_on_plank(objects):
    block = objects["block"]
    return _on_plank(objects["plank"], float(block.pos[0]), float(block.pos[1]), float(block.low[2]))


def _balance(objects, tin_y: float | None = None) -> float:
    """How far behind the edge (m; negative: past it) the centre of mass of the plank and the loads resting on it
    lies. With ``tin_y``, the tin counts as standing on the plank at that y, wherever it really is."""
    mass, moment = PLANK_KG, PLANK_KG * float(objects["plank"].pos[1])
    if _block_on_plank(objects) is not None:
        mass, moment = mass + BLOCK_KG, moment + BLOCK_KG * float(objects["block"].pos[1])
    if tin_y is None and _tin_on_plank(objects) is not None:
        tin_y = float(objects["tin"].pos[1])
    if tin_y is not None:
        mass, moment = mass + TIN_KG, moment + TIN_KG * tin_y
    return moment / mass - EDGE_Y


def _zone_y(plank, from_far_end: float) -> float:
    """The world y of the point on the plank's middle line ``from_far_end`` from its far end."""
    return float(plank.pos[1] - _rot(plank.quat_xyzw)[1, 1] * (PLANK_LEN / 2 - from_far_end))


def _in_place(plank) -> bool:
    """The plank's centre within PLACE_TOL of its start, seen from above, and its length within PLACE_DEG of the
    start direction (world y)."""
    along = _rot(plank.quat_xyzw)[:, 1]
    turn = math.degrees(math.atan2(abs(float(along[0])), abs(float(along[1]))))
    shift = math.hypot(float(plank.pos[0]) - PLANK.pos[0], float(plank.pos[1]) - PLANK.pos[1])
    return shift <= PLACE_TOL and turn <= PLACE_DEG


def _counterweighted(objects) -> bool:
    """The plank, as loaded now, would hold the tin at the outermost spot of the target zone."""
    return _balance(objects, tin_y=_zone_y(objects["plank"], ZONE_NEAR)) >= BALANCE_MARGIN


def _fell(objects) -> bool:
    plank, tin, block = objects["plank"], objects["tin"], objects["block"]
    return (_plank_tilt_deg(plank) > TIP_DEG or _tin_tilt_deg(tin) > TIN_OVER_DEG
            or float(tin.lin_vel[2]) < -DROP_SPEED or float(block.lin_vel[2]) < -DROP_SPEED)


# What a reset leaves (measured on the official sim in review, 2026-11-03): every loose object within 0.05 mm of its
# spawn x and y, 0.5 to 1 mm below its spawn z (they are dropped in), and level. A tin or block put back by a gripper
# won't land within 1 mm of its mark, so a tip can't be undone by putting things back.
START_XY, START_Z, START_DEG = 0.001, 0.002, 0.5


def _at_start(objects) -> bool:
    """Every loose object where a reset puts it: the first step of a trial (the runs use -E object_jitter=0)."""
    if not (_plank_tilt_deg(objects["plank"]) < START_DEG and _tin_tilt_deg(objects["tin"]) < START_DEG):
        return False
    for name, spec in LOOSE.items():
        pos = np.asarray(objects[name].pos, dtype=float)
        if math.hypot(pos[0] - spec.pos[0], pos[1] - spec.pos[1]) > START_XY or abs(pos[2] - spec.pos[2]) > START_Z:
            return False
    return True


# Whether anything has fallen this trial. A module-level flag, because a fall must count even after the robot tidies
# up; it clears when every loose object is back at its start, which is how the next trial begins. success() and
# score() both call _watch() on the same step, so the second call must change nothing: it doesn't.
_fallen = False


def _watch(objects) -> bool:
    global _fallen
    if _at_start(objects):
        _fallen = False
    elif _fell(objects):
        _fallen = True
    return _fallen


def _delivered(objects) -> bool:
    """The tin stands upright in the target zone, the plank is level, in place and stands on its own, and nothing
    moves."""
    plank, tin = objects["plank"], objects["tin"]
    spot = _tin_on_plank(objects)
    return (spot is not None and ZONE_NEAR <= spot[0] <= ZONE_FAR and abs(spot[1]) <= ZONE_SIDE
            and _tin_tilt_deg(tin) <= UPRIGHT_DEG and _plank_tilt_deg(plank) <= LEVEL_DEG and _in_place(plank)
            and _balance(objects) >= BALANCE_MARGIN and all(objects[n].resting() for n in LOOSE))


def success(objects):
    """Nothing has fallen, and the tin is delivered; must hold for 1 s."""
    fallen = _watch(objects)
    return not fallen and _delivered(objects)


def distance(objects):
    """From the centre of the tin's base to the middle of the target zone, seen from above (m)."""
    plank = objects["plank"]
    rot = _rot(plank.quat_xyzw)
    middle = np.asarray(plank.pos, dtype=float) - rot[:, 1] * (PLANK_LEN / 2 - (ZONE_NEAR + ZONE_FAR) / 2)
    bx, by, _ = _tin_base(objects["tin"])
    return float(math.hypot(bx - middle[0], by - middle[1]))


def score(objects):
    """1.0 when delivered (the success state); 0.5 when nothing has fallen, the plank is level and in place, and the
    block rests on it far enough back to hold the tin anywhere in the target zone, but the tin is not delivered yet; 0
    otherwise, including any trial in which something fell."""
    if _watch(objects):
        return 0.0
    if _delivered(objects):
        return 1.0
    plank = objects["plank"]
    if _plank_tilt_deg(plank) <= LEVEL_DEG and _in_place(plank) and _counterweighted(objects):
        return 0.5
    return 0.0


BLOCK_SPOT_Y = 0.45  # the demo's block: 18 cm behind the edge, 4 cm clear of the stop
TIN_SPOT_Y = PLANK_FAR_Y + 0.045  # the demo's tin: 4.5 cm from the far end, inside the zone
GRIP_ABOVE_BASE = 0.055  # the gripper holds the tin 5.5 cm above its base
_TIN_GRIP_Z = TOP + GRIP_ABOVE_BASE
_TIN_CARRY_Z = PLANK_TOP + 0.03 + GRIP_ABOVE_BASE  # the tin's base 3 cm over the plank
_TIN_SET_Z = PLANK_TOP + GRIP_ABOVE_BASE + 0.001  # its base on the plank: set down, not dropped


def deliver_tin_waypoints():
    """Take the tin from the table and set it on the plank's far end: down, grip, lift, carry, pause, lower slowly,
    hold, open slowly, wait, go up."""
    spot = (ROW_X, TIN_SPOT_Y)
    return [
        Waypoint(1.8, (*TIN_XY, _TIN_CARRY_Z), FINGER_OPEN, "over the tin"),
        Waypoint(1.5, (*TIN_XY, _TIN_GRIP_Z), FINGER_OPEN, "down"),
        Waypoint(0.8, (*TIN_XY, _TIN_GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (*TIN_XY, _TIN_CARRY_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (*spot, _TIN_CARRY_Z), FINGER_CLOSED, "carry over the far end"),
        Waypoint(0.5, (*spot, _TIN_CARRY_Z), FINGER_CLOSED, "settle"),
        Waypoint(2.0, (*spot, _TIN_SET_Z), FINGER_CLOSED, "lower onto the plank"),
        Waypoint(0.5, (*spot, _TIN_SET_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (*spot, _TIN_SET_Z), FINGER_OPEN, "let go"),
        Waypoint(0.8, (*spot, _TIN_SET_Z), FINGER_OPEN, "wait, fingers open"),
        Waypoint(1.0, (*spot, _TIN_CARRY_Z + 0.02), FINGER_OPEN, "up"),
    ]


def demo():
    """The left arm first sets the block on the plank's other end, 18 cm behind the ledge edge, then sets the tin on
    the far end, 4.5 cm from it. The block carries 14 cm above the table: higher up, a straight-down grip can need
    joint4 past its limit. The right arm doesn't move."""
    weigh_down = pick_place_waypoints(
        BLOCK.pos, (ROW_X, BLOCK_SPOT_Y, PLANK_TOP + BLOCK_SIZE / 2 + 0.003), travel_z=TOP + 0.14)
    return {"left": [*weigh_down, *deliver_tin_waypoints()]}
