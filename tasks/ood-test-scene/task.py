"""On the station table: a dark grey rubber mat (44 x 34 cm, 3 mm thick) lies flat in front of the arms, fixed.
Three thin painted discs lie on the mat, fixed: they are the outlines of where three objects stood while a robot
policy was trained. Each outline is 1 cm wider all round than its object and painted a pale tint of that object's
colour. All three loose objects stand upright on their own outlines, so only a 1 cm ring of each outline shows:

- red tin (sealed steel food tin, 54 mm across, 55 mm tall): on the pale red outline, left of the midline
- yellow jar (lidded glass spice jar, 44 mm across, 65 mm tall): on the pale yellow outline, on the midline,
  furthest from the arms
- blue tub (lidded plastic tub, 48 mm across, 75 mm tall): on the pale blue outline, right of the midline

"Left" and "right" are as the arms face the table: left is toward the left arm. The mat has room for every
object to stand somewhere else: in front of the outlines (toward the arms), behind them, and at the sides.
The task is to set up a test scene unlike the training layout: every object moved so its centre is at least
10 cm from the centre of its own outline, standing upright, entirely on the mat, with no outline covered.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the mat (fixed): dark grey rubber, 3 mm --------------------------------------------------------------------------

MAT_X0, MAT_X1 = 0.12, 0.46  # 34 cm, from 12 cm in front of the arm bases
MAT_Y0, MAT_Y1 = -0.22, 0.22  # 44 cm, centred on the midline
MAT_T = 0.003
MAT_TOP = TOP + MAT_T
MAT_COLOR = (0.22, 0.23, 0.25)
MAT = Box(
    "mat",
    pos=((MAT_X0 + MAT_X1) / 2, (MAT_Y0 + MAT_Y1) / 2, TOP + MAT_T / 2),
    size=(MAT_X1 - MAT_X0, MAT_Y1 - MAT_Y0, MAT_T),
    color=MAT_COLOR,
)

# -- the objects (loose) and their outlines (fixed) -------------------------------------------------------------------

OUTLINE_T = 0.002  # painted vinyl disc, 2 mm
OUTLINE_TOP = MAT_TOP + OUTLINE_T
OUTLINE_BORDER = 0.010  # each outline is 1 cm wider all round than its object
REST_GAP = 0.001  # loose objects spawn this far above their outline and settle onto it

# name: (radius, height, mass, colour, outline colour, outline centre (x, y)) -- the training layout
SPEC = {
    "red_tin": (0.027, 0.055, 0.150, (0.80, 0.12, 0.12), (0.96, 0.62, 0.60), (0.30, 0.11)),
    "yellow_jar": (0.022, 0.065, 0.120, (0.95, 0.78, 0.08), (0.99, 0.93, 0.58), (0.36, 0.0)),
    "blue_tub": (0.024, 0.075, 0.060, (0.15, 0.30, 0.85), (0.62, 0.72, 0.97), (0.30, -0.11)),
}
NAMES = tuple(SPEC)


def radius(name: str) -> float:
    return SPEC[name][0]


def height(name: str) -> float:
    return SPEC[name][1]


def outline_xy(name: str) -> np.ndarray:
    return np.asarray(SPEC[name][5], dtype=float)


def outline_radius(name: str) -> float:
    return radius(name) + OUTLINE_BORDER


OUTLINES = [
    Cylinder(
        f"{n}_outline",
        pos=(*SPEC[n][5], MAT_TOP + OUTLINE_T / 2),
        radius=outline_radius(n),
        height=OUTLINE_T,
        color=SPEC[n][4],
    )
    for n in NAMES
]
ITEMS = {
    n: Cylinder(
        n,
        pos=(*SPEC[n][5], OUTLINE_TOP + REST_GAP + SPEC[n][1] / 2),
        radius=SPEC[n][0],
        height=SPEC[n][1],
        mass=SPEC[n][2],
        color=SPEC[n][3],
    )
    for n in NAMES
}
OBJECTS = [MAT, *OUTLINES, *ITEMS.values()]

# -- the instruction, as numbers --------------------------------------------------------------------------------------

FULL_SHIFT = 0.10  # centre at least 10 cm from the centre of its own outline: 1/3
PART_SHIFT = 0.05  # 5 to 10 cm: 1/6
UPRIGHT_DEG = 10.0  # axis within 10 deg of vertical (either flat end down: all three are closed)
ON_MAT = 0.004  # lowest point within 4 mm of the mat top: standing on the mat, not on an outline or another object

# -- geometry helpers -------------------------------------------------------------------------------------------------


def _axis(state) -> np.ndarray:
    """The object's own vertical axis (body z) in the world frame."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])


def xy(state) -> np.ndarray:
    return np.asarray(state.pos[:2], dtype=float)


def shift(objects, name: str) -> float:
    """Horizontal distance from the object's centre to the centre of its own outline (m)."""
    return float(np.linalg.norm(xy(objects[name]) - outline_xy(name)))


def upright(state) -> bool:
    return abs(float(_axis(state)[2])) >= math.cos(math.radians(UPRIGHT_DEG))


def off_table(state) -> bool:
    """Centre below the table top: fallen to the floor."""
    return float(state.pos[2]) < TOP


def on_mat(state, name: str) -> bool:
    """The whole base over the mat (centre at least one radius inside every edge) and standing on the mat itself."""
    x, y = xy(state)
    r = radius(name)
    inside = MAT_X0 + r <= x <= MAT_X1 - r and MAT_Y0 + r <= y <= MAT_Y1 - r
    return bool(inside and abs(float(state.low[2]) - MAT_TOP) <= ON_MAT)


def covered(objects, name: str) -> list[str]:
    """The outlines this object's base overlaps, seen from above (centre distance under the two radii summed)."""
    p = xy(objects[name])
    return [o for o in NAMES if float(np.linalg.norm(p - outline_xy(o))) < radius(name) + outline_radius(o)]


def placed_well(objects, name: str) -> bool:
    """Standing upright, still, entirely on the mat and over no outline (its own or another's)."""
    s = objects[name]
    return bool(upright(s) and s.resting() and on_mat(s, name) and not covered(objects, name))


# -- scoring ----------------------------------------------------------------------------------------------------------


def object_sixths(objects, name: str) -> int:
    """2 (= 1/3) if placed well and at least 10 cm from its own outline; 1 (= 1/6) if placed well and 5-10 cm from
    it; else 0."""
    if not placed_well(objects, name):
        return 0
    d = shift(objects, name)
    return 2 if d >= FULL_SHIFT else 1 if d >= PART_SHIFT else 0


def object_score(objects, name: str) -> float:
    return object_sixths(objects, name) / 6.0


def score(objects):
    """Sum of the three objects' credit (in sixths, 0..1), or 0 if any object has fallen off the table."""
    if any(off_table(objects[n]) for n in NAMES):
        return 0.0
    return sum(object_sixths(objects, n) for n in NAMES) / 6.0


def success(objects):
    """All three objects at least 10 cm from their own outlines, upright, still, on the mat, every outline
    uncovered (full score); must hold for 1 s."""
    return bool(score(objects) == 1.0)


def distance(objects):
    """Metres: how far each object is short of 10 cm from its own outline, summed, plus 0.05 for each object that
    is not standing upright entirely on the mat clear of every outline."""
    total = 0.0
    for n in NAMES:
        total += max(0.0, FULL_SHIFT - shift(objects, n))
        s = objects[n]
        total += 0.05 * (not (upright(s) and on_mat(s, n) and not covered(objects, n)))
    return float(total)


# -- demo -------------------------------------------------------------------------------------------------------------
# The right arm moves the blue tub to the front right of the mat while the left arm moves the red tin to the front
# left; then the left arm moves the yellow jar straight toward the arms. Each object is gripped straight down with
# the fingers closing along y (yaw 0), carried 3 cm above the mat, set down from 3 mm, let go, and the gripper rises
# 3 cm before moving anywhere. Both arms then park beside the mat, outside the top camera's view of every object.

GOAL_XY = {"red_tin": (0.17, 0.14), "yellow_jar": (0.22, 0.0), "blue_tub": (0.17, -0.14)}
DEMO_ARM = {"red_tin": "left", "yellow_jar": "left", "blue_tub": "right"}
DEMO_ORDER = {"left": ("red_tin", "yellow_jar"), "right": ("blue_tub",)}
PARK_XY = {"left": (0.14, 0.33), "right": (0.14, -0.33)}  # beside the mat, by each arm's own base

TIP_CLEARANCE = 0.020  # moving sideways, the pad tips pass at least this far above the object's top
PARK_Z = TOP + 0.120  # park and hold height of the TCP
# TCP height above an object's bottom at the grip: the pad tips (13 mm below the TCP) are 27-42 mm up the side.
GRIP_ABOVE_BOTTOM = {"red_tin": 0.040, "yellow_jar": 0.048, "blue_tub": 0.055}
CARRY = 0.030  # the object's bottom is carried this far above the mat (clears the 2 mm outlines)
RELEASE_GAP = 0.003  # and let go this far above the mat
CLEAR_RISE = 0.030  # after letting go, rise this far before any sideways move
APPROACH_CLEARANCE = 0.010  # each pad this far off the object on the way down
LOOSEN = 0.006  # each pad backs off this far to let go
GRIP_SHIFT_Y = 0.006  # pads hold an object 11-20 mm toward -y (GPU rounds): aim each place this much toward +y
FINAL_HOLD_S = 2.0  # still hold at the end (at least 1.5 s): success needs 1 s of rest
MOVE_S = 10.0  # one pick and place, see move_waypoints
PARK_S = 1.5


def _finger(name: str, clearance: float) -> float:
    """Finger joint target leaving ``clearance`` between each pad and the object."""
    return -(radius(name) + clearance)


def hover_z(name: str) -> float:
    """TCP height for the sideways moves to and from an object: the open pad tips 2 cm above its top as it stands
    on its outline (TOP + 93 / 103 / 113 mm for the tin / jar / tub; the jar's outline at x 0.36 allows up to
    about TOP + 0.10-0.11)."""
    return OUTLINE_TOP + height(name) + PAD_TIP_PAST_TCP + TIP_CLEARANCE


def move_waypoints(name: str) -> list[Waypoint]:
    """Grip the object on its outline, lift it 3 cm above the mat, carry it to its goal, set it down from 3 mm,
    let go, rise 3 cm, then go up to the hover height (10.0 s)."""
    px, py = outline_xy(name)
    qx, qy = GOAL_XY[name][0], GOAL_XY[name][1] + GRIP_SHIFT_Y
    g = GRIP_ABOVE_BOTTOM[name]
    grip_z = OUTLINE_TOP + REST_GAP + g
    lift_z = MAT_TOP + CARRY + g
    release_z = MAT_TOP + RELEASE_GAP + g
    rise_z = release_z + CLEAR_RISE
    hover = hover_z(name)
    approach, loose = _finger(name, APPROACH_CLEARANCE), _finger(name, LOOSEN)
    return [
        Waypoint(1.5, (px, py, hover), approach, "to the object"),
        Waypoint(1.5, (px, py, grip_z), approach, "descend"),
        Waypoint(0.8, (px, py, grip_z), FINGER_CLOSED, "close"),
        Waypoint(1.0, (px, py, lift_z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (qx, qy, lift_z), FINGER_CLOSED, "carry"),
        Waypoint(1.0, (qx, qy, release_z), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (qx, qy, release_z), loose, "release"),
        Waypoint(0.8, (qx, qy, rise_z), loose, "rise clear"),
        Waypoint(0.8, (qx, qy, hover), loose, "up"),
    ]


def arm_plan(side: str, total_s: float) -> list[Waypoint]:
    plan = [w for name in DEMO_ORDER[side] for w in move_waypoints(name)]
    park = Waypoint(PARK_S, (*PARK_XY[side], PARK_Z), plan[-1].finger, "park")
    hold_s = total_s - sum(w.duration for w in plan) - PARK_S
    return [*plan, park, Waypoint(hold_s, park.pos, park.finger, "hold still")]


def demo():
    """23.5 s per arm: the left arm moves the red tin and then the yellow jar, the right arm moves the blue tub and
    parks; both end parked beside the mat, the left with a 2 s still hold, the right holding still for 12 s."""
    total = max(len(v) for v in DEMO_ORDER.values()) * MOVE_S + PARK_S + FINAL_HOLD_S
    return {side: arm_plan(side, total) for side in ("left", "right")}
