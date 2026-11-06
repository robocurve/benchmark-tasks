"""On the station table: three painted wooden bars lie flat in a row across the table, 30 cm out from the arms.
Each bar is 8.0 x 2.5 x 2.5 cm (a square stick 8 cm long) with its long side running left to right (y), so the
bars also serve as a ruler: 5 cm is five eighths of a bar's length. All three are loose.

- red bar: on the left, in front of the left arm, its centre 16 cm left of the midline
- green bar: in the middle, on the midline between the arms
- blue bar: on the right, in front of the right arm, its centre 16 cm right of the midline

"Left" and "right" are as the arms face the table: left is toward the left arm. Nothing on the table marks where
any bar should end up; every amount in the instruction is measured from where that bar starts.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the bars: painted pine, 80 x 25 x 25 mm, lying along y ---------------------------------------------------------

BAR_SIZE = (0.025, 0.080, 0.025)  # x (across), y (along the bar), z; yaw 0 puts the long side along world y
BAR_MASS = 0.025  # kg: pine at about 500 kg/m^3
ROW_X = 0.30  # every bar's centre is this far out from the arm bases (m)
SPACING = 0.16  # centre to centre along y; 8 cm of bare table between neighbouring bars
SPAWN_GAP = 0.001  # bars spawn this far above the table and settle onto it


def _bar(name: str, y: float, color: tuple[float, float, float]) -> Box:
    return Box(name, pos=(ROW_X, y, TOP + BAR_SIZE[2] / 2 + SPAWN_GAP), size=BAR_SIZE, mass=BAR_MASS, color=color)


RED = _bar("red_bar", +SPACING, (0.80, 0.12, 0.12))
GREEN = _bar("green_bar", 0.0, (0.15, 0.62, 0.25))
BLUE = _bar("blue_bar", -SPACING, (0.15, 0.30, 0.85))
BARS = {b.name: b for b in (RED, GREEN, BLUE)}
OBJECTS = [RED, GREEN, BLUE]

# -- the instruction, as numbers -----------------------------------------------------------------------------------

SLIDE = 0.050  # red: 5 cm toward +y (the left arm's side)
TURN_DEG = 90.0  # blue: a quarter turn, either way round

# Red, 0.5: moved 35-65 mm toward +y, drifted at most 25 mm along x, and turned at most 15 deg.
# Red, 0.25: moved 25-75 mm toward +y, drifted at most 40 mm along x, and turned at most 30 deg.
RED_FULL = dict(slide=(0.035, 0.065), drift=0.025, turn_deg=15.0)
RED_PART = dict(slide=(0.025, 0.075), drift=0.040, turn_deg=30.0)
# Blue, 0.5: long axis turned 80-90 deg from where it started (90 +/- 10, either way), centre within 25 mm of start.
# Blue, 0.25: turned 65-90 deg (90 +/- 25), centre within 40 mm.
BLUE_FULL = dict(turn_deg=80.0, shift=0.025)
BLUE_PART = dict(turn_deg=65.0, shift=0.040)
# Green, a multiplier on the sum: x1 within 8 mm and 8 deg of where it started; x0.5 within 20 mm and 20 deg;
# x0 beyond that, or not lying flat on the table.
GREEN_STILL = dict(shift=0.008, turn_deg=8.0)
GREEN_BUMPED = dict(shift=0.020, turn_deg=20.0)
FULL, PART = 0.5, 0.25

FLAT_DEG = 10.0  # "lying flat": long axis within 10 deg of horizontal and a long face within 10 deg of down
ON_TABLE = 0.005  # bottom of the bounding box within 5 mm of the table top (not lifted, not on the floor)


# -- geometry helpers ------------------------------------------------------------------------------------------------


def _rotation(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def long_axis(state) -> np.ndarray:
    """The bar's long axis (its body y) in the world frame."""
    return _rotation(state)[:, 1]


def lying_flat(state) -> bool:
    """On one of its four long faces on the table: long axis within 10 deg of horizontal, one of the two short body
    axes within 10 deg of vertical (the cross-section is square, so any long face down counts), and the bottom of
    the bounding box within 5 mm of the table top (so not held up, not resting on another bar, not on the floor)."""
    r = _rotation(state)
    sin_flat = math.sin(math.radians(FLAT_DEG))
    cos_flat = math.cos(math.radians(FLAT_DEG))
    axis_level = abs(r[2, 1]) <= sin_flat
    face_down = max(abs(r[2, 0]), abs(r[2, 2])) >= cos_flat
    return bool(axis_level and face_down and abs(state.low[2] - TOP) <= ON_TABLE)


def axis_turn_deg(state, name: str) -> float:
    """How far the bar's long axis has turned about the vertical from its start, 0..90 deg. A bar looks the same
    end for end, so +90 and -90 are both a quarter turn, and 180 is no turn."""
    a = long_axis(state)[:2]
    norm = float(np.linalg.norm(a))
    if norm < 1e-9:  # standing on end: no horizontal axis (lying_flat already fails)
        return 90.0
    start = np.array([-math.sin(BARS[name].yaw), math.cos(BARS[name].yaw)])
    cos = abs(float(np.dot(a / norm, start)))
    return math.degrees(math.acos(min(1.0, cos)))


def start_xy(name: str) -> np.ndarray:
    return np.asarray(BARS[name].pos[:2], dtype=float)


def offset(state, name: str) -> np.ndarray:
    """Centre now minus centre at the start, in the table plane (m)."""
    return np.asarray(state.pos[:2], dtype=float) - start_xy(name)


# -- scoring -------------------------------------------------------------------------------------------------------


def red_score(objects) -> float:
    """0.5 / 0.25 / 0 for the red bar's 5 cm slide toward +y (see RED_FULL, RED_PART)."""
    s = objects["red_bar"]
    if not (lying_flat(s) and s.resting()):
        return 0.0
    dx, dy = offset(s, "red_bar")
    turn = axis_turn_deg(s, "red_bar")
    for tier, value in ((RED_FULL, FULL), (RED_PART, PART)):
        lo, hi = tier["slide"]
        if lo <= dy <= hi and abs(dx) <= tier["drift"] and turn <= tier["turn_deg"]:
            return value
    return 0.0


def blue_score(objects) -> float:
    """0.5 / 0.25 / 0 for the blue bar's quarter turn in place (see BLUE_FULL, BLUE_PART)."""
    s = objects["blue_bar"]
    if not (lying_flat(s) and s.resting()):
        return 0.0
    shift = float(np.linalg.norm(offset(s, "blue_bar")))
    turn = axis_turn_deg(s, "blue_bar")
    for tier, value in ((BLUE_FULL, FULL), (BLUE_PART, PART)):
        if turn >= tier["turn_deg"] and shift <= tier["shift"]:
            return value
    return 0.0


def green_factor(objects) -> float:
    """1 if the green bar was left where it was, 0.5 if it was bumped a little, 0 if it was moved or knocked over."""
    s = objects["green_bar"]
    if not lying_flat(s):
        return 0.0
    shift = float(np.linalg.norm(offset(s, "green_bar")))
    turn = axis_turn_deg(s, "green_bar")
    for tier, value in ((GREEN_STILL, 1.0), (GREEN_BUMPED, 0.5)):
        if shift <= tier["shift"] and turn <= tier["turn_deg"]:
            return value
    return 0.0


def score(objects):
    """(red + blue) x green, 0..1: red and blue each 0.5 / 0.25 / 0, green a multiplier of 1 / 0.5 / 0."""
    return float((red_score(objects) + blue_score(objects)) * green_factor(objects))


def success(objects):
    """Full score with all three bars still; must hold for 1 s."""
    return bool(score(objects) == 1.0 and all(objects[n].resting() for n in BARS))


def goal_xy(name: str) -> np.ndarray:
    return start_xy(name) + (np.array([0.0, SLIDE]) if name == "red_bar" else 0.0)


def distance(objects):
    """Metres, summed over the bars: each centre to where it should be in the table plane (red 5 cm toward +y of
    its start; blue and green at their starts), plus 0.1 m for each bar not yet in its full-credit state otherwise
    (blue not turned at least 80 deg, red turned more than 15 deg, green turned more than 8 deg)."""
    total = sum(float(np.linalg.norm(objects[n].pos[:2] - goal_xy(n))) for n in BARS)
    total += 0.1 * (axis_turn_deg(objects["blue_bar"], "blue_bar") < BLUE_FULL["turn_deg"])
    total += 0.1 * (axis_turn_deg(objects["red_bar"], "red_bar") > RED_FULL["turn_deg"])
    total += 0.1 * (axis_turn_deg(objects["green_bar"], "green_bar") > GREEN_STILL["turn_deg"])
    return float(total)


# -- demo ------------------------------------------------------------------------------------------------------------
# Left arm: picks the red bar, lifts it 15 mm, carries it 5 cm toward +y and sets it down. Right arm: picks the blue
# bar, lifts it 15 mm, turns it a quarter turn about the vertical and sets it down where it was. Neither arm comes
# near the green bar. Both grip each bar across its 25 mm width with the fingers closing along x (yaw +-90 deg), so
# any off-centre grip (12-20 mm along the closing direction in the first GPU round) shifts the red bar along x,
# not along the 5 cm slide.

HOVER_Z = TOP + 0.100  # approach and final height of the TCP
GRIP_Z = TOP + 0.019  # TCP height when gripping: pads hold the bar from 6 mm above the table to its top
LIFT = 0.015  # the bar is carried this far above the table
RELEASE_GAP = 0.003  # and let go this far above it
CLEAR_RISE = 0.030  # after letting go, rise this far before any sideways move
APPROACH_CLEARANCE = 0.010  # each pad this far off the bar on the way down (less travel, less off-centre grip)
LOOSEN = 0.006  # each pad backs off this far to let go
FINAL_HOLD_S = 2.0  # still hold at the end (at least 1.5 s): success needs 1 s of rest
PICK_YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along x, across the bar's width
PLACE_YAW = {"left": -math.pi / 2, "right": 0.0}  # red keeps its heading; blue turns -90 deg (clockwise from above)
DEMO_ARM = {"red_bar": "left", "blue_bar": "right"}


def _finger(clearance: float) -> float:
    """Finger joint target leaving ``clearance`` between each pad and the 25 mm bar."""
    return -(BAR_SIZE[0] / 2 + clearance)


def adjust_waypoints(name: str) -> list[Waypoint]:
    """Grip the bar, lift it 15 mm, move it to its goal (red: 5 cm toward +y; blue: same place, turned a quarter
    turn), set it down from 3 mm, let go, rise 3 cm, then go up and hold still."""
    side = DEMO_ARM[name]
    px, py, _ = BARS[name].pos
    qx, qy = goal_xy(name)
    pick_yaw, place_yaw = PICK_YAW[side], PLACE_YAW[side]
    approach, loose = _finger(APPROACH_CLEARANCE), _finger(LOOSEN)
    lift_z, release_z = GRIP_Z + LIFT, GRIP_Z + RELEASE_GAP
    rise_z = release_z + CLEAR_RISE
    return [
        Waypoint(1.5, (px, py, HOVER_Z), approach, "to the bar", pick_yaw),
        Waypoint(1.5, (px, py, GRIP_Z), approach, "descend", pick_yaw),
        Waypoint(0.8, (px, py, GRIP_Z), FINGER_CLOSED, "close", pick_yaw),
        Waypoint(0.8, (px, py, lift_z), FINGER_CLOSED, "lift", pick_yaw),
        Waypoint(2.0, (qx, qy, lift_z), FINGER_CLOSED, "move", place_yaw),
        Waypoint(0.8, (qx, qy, release_z), FINGER_CLOSED, "lower", place_yaw),
        Waypoint(0.6, (qx, qy, release_z), loose, "release", place_yaw),
        Waypoint(1.0, (qx, qy, rise_z), loose, "rise clear", place_yaw),
        Waypoint(1.2, (qx, qy, HOVER_Z), loose, "up", place_yaw),
        Waypoint(FINAL_HOLD_S, (qx, qy, HOVER_Z), loose, "hold still", place_yaw),
    ]


def demo():
    """Both arms at once, 12.2 s each: the left slides the red bar 5 cm toward +y (by lifting it 15 mm), the right
    turns the blue bar a quarter turn in place; the green bar is not touched."""
    return {side: adjust_waypoints(name) for name, side in DEMO_ARM.items()}
