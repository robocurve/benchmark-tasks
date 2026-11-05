"""On the station table, between the arms: a fixed dark grey shelf rack with one row of six open pockets side by side
from left to right, its centre 28 cm in front of the arm bases. Each pocket is 46 x 46 mm inside with a 4 mm floor
and 12 mm lips; the pockets are 52 mm apart centre to centre, numbered 1 to 6 from the robot's left. The rack's
front lip, the one nearest the arms, is black. Four vial cartons (cardboard boxes 40 mm wide, 24 mm deep, 60 mm
tall, 60 g each, the drug name printed on both wide faces) stand on the table with their wide faces turned to the
sides: a red one and a blue one in front of the left arm, 20 and 32 cm in front of the arm bases, and a green one
and a yellow one in front of the right arm at the same distances. The red and blue cartons are look-alike drugs.
Shelved, every carton stands upright in its own pocket with a wide face toward the black front lip, and the red
and blue cartons are not in the same or neighbouring pockets.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The rack: a 4 mm floor plate, front and back lips, two end walls and five dividers, 6 mm thick, rising 12 mm above
# the floor, around six 46 x 46 mm pockets in a row along y.
RACK_X = 0.28
POCKET_IN = 0.046  # inside of a pocket, front to back and side to side
WALL_T, LIP_H, FLOOR_T = 0.006, 0.012, 0.004
PITCH = POCKET_IN + WALL_T  # 52 mm centre to centre
N_POCKETS = 6
POCKET_Y = tuple((N_POCKETS - 1) / 2 * PITCH - i * PITCH for i in range(N_POCKETS))  # pocket 1 (left) to 6 (right)
FLOOR = TOP + FLOOR_T
RACK_LEN = N_POCKETS * POCKET_IN + (N_POCKETS + 1) * WALL_T  # 318 mm outside, left to right
RACK_OUT_X = (RACK_X - POCKET_IN / 2 - WALL_T, RACK_X + POCKET_IN / 2 + WALL_T)
RACK_GREY = (0.22, 0.23, 0.26)
LIP_BLACK = (0.03, 0.03, 0.03)
_INNER_LEN = RACK_LEN - 2 * WALL_T
_FULL_H = FLOOR_T + LIP_H  # walls standing on the table: 16 mm
RACK = [
    Box("rack_floor", pos=(RACK_X, 0.0, TOP + FLOOR_T / 2), size=(POCKET_IN, _INNER_LEN, FLOOR_T), color=RACK_GREY),
    Box("rack_front_lip", pos=(RACK_OUT_X[0] + WALL_T / 2, 0.0, TOP + _FULL_H / 2), size=(WALL_T, RACK_LEN, _FULL_H),
        color=LIP_BLACK),
    Box("rack_back_lip", pos=(RACK_OUT_X[1] - WALL_T / 2, 0.0, TOP + _FULL_H / 2), size=(WALL_T, RACK_LEN, _FULL_H),
        color=RACK_GREY),
    Box("rack_end_left", pos=(RACK_X, RACK_LEN / 2 - WALL_T / 2, TOP + _FULL_H / 2), size=(POCKET_IN, WALL_T, _FULL_H),
        color=RACK_GREY),
    Box("rack_end_right", pos=(RACK_X, -RACK_LEN / 2 + WALL_T / 2, TOP + _FULL_H / 2),
        size=(POCKET_IN, WALL_T, _FULL_H), color=RACK_GREY),
] + [
    Box(f"rack_divider_{i + 1}", pos=(RACK_X, (POCKET_Y[i] + POCKET_Y[i + 1]) / 2, FLOOR + LIP_H / 2),
        size=(POCKET_IN, WALL_T, LIP_H), color=RACK_GREY)
    for i in range(N_POCKETS - 1)
]

# A carton at yaw 0 has its wide faces (40 mm, side to side) toward the front lip; it starts turned 90 degrees.
CARTON = (0.024, 0.040, 0.060)  # depth (x), width (y), height (z) at yaw 0
CARTON_KG = 0.06
START_YAW = math.pi / 2
COLORS = {
    "red_carton": (0.80, 0.12, 0.10),
    "blue_carton": (0.12, 0.30, 0.85),
    "green_carton": (0.10, 0.60, 0.20),
    "yellow_carton": (0.95, 0.80, 0.10),
}
START = {
    "red_carton": (0.20, 0.30),
    "blue_carton": (0.32, 0.30),
    "green_carton": (0.20, -0.30),
    "yellow_carton": (0.32, -0.30),
}
CARTONS = [
    Box(name, pos=(x, y, TOP + CARTON[2] / 2 + 0.001), size=CARTON, mass=CARTON_KG, color=COLORS[name], yaw=START_YAW)
    for name, (x, y) in START.items()
]  # each starts 1 mm above the table, and settles
CARTON_NAMES = tuple(START)
LOOK_ALIKES = ("red_carton", "blue_carton")

OBJECTS = [*RACK, *CARTONS]
SUCCESS_HOLD_S = 3.0  # shelved and still for 3 s

SHELVED_CREDIT, FACING_CREDIT = 0.15, 0.10  # per carton: 0.25 at most, 1.0 for all four
UPRIGHT_DEG = 10.0  # shelved: tilted at most 10 degrees
FACING_DEG = 20.0  # a wide face within 20 degrees of facing the front lip
FLOOR_TOL = 0.005  # shelved: the carton's bottom within 5 mm of the pocket floor
# The look-alike rule counts a carton over the rack whose bottom is less than 70 mm above the pocket floor (anything in
# a pocket, on the lips or on a shelved carton, 60 mm tall), and any carton over the rack standing on another carton,
# however high the stack. Only a carton in the air, carried over the rack clear of the shelved cartons' tops with
# nothing under it, is not counted.
GATE_H = 0.070
STACK_TOL = 0.010  # standing on another carton: its bottom within 10 mm of that carton's top, footprints overlapping


def _tilt_deg(state) -> float:
    """Angle between the object's up axis and the vertical."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _facing_err_deg(state) -> float:
    """Angle about the vertical between the carton's wide-face normal and the front-to-back axis (0 to 90 degrees;
    either wide face toward the front counts, as both carry the name)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    a = abs(math.degrees(math.atan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z)))) % 180.0
    return min(a, 180.0 - a)


def _on_carton(objects, name) -> bool:
    """The carton stands on another carton: its bottom within STACK_TOL of the other's top, footprints overlapping."""
    s = objects[name]
    for other in CARTON_NAMES:
        if other == name:
            continue
        o = objects[other]
        overlap = all(float(s.low[k]) < float(o.high[k]) and float(o.low[k]) < float(s.high[k]) for k in (0, 1))
        if overlap and abs(float(s.low[2]) - float(o.high[2])) <= STACK_TOL:
            return True
    return False


def _slot(objects, name):
    """The pocket (0 to 5) whose 52 mm band of the rack (across its full 58 mm depth) holds the carton's centre, at
    any tilt, with its bottom less than GATE_H above the pocket floor or standing on another carton; None if the
    centre is not over the rack or the carton is in the air above GATE_H (carried over the shelved cartons)."""
    state = objects[name]
    x, y, _ = (float(v) for v in state.pos)
    if not RACK_OUT_X[0] <= x <= RACK_OUT_X[1]:
        return None
    if float(state.low[2]) - FLOOR >= GATE_H and not _on_carton(objects, name):
        return None
    for i, py in enumerate(POCKET_Y):
        if abs(y - py) <= PITCH / 2:
            return i
    return None


def _shelved(state):
    """The pocket (0 to 5) the carton stands in: centre inside its 46 x 46 mm inside, tilted at most UPRIGHT_DEG, its
    bottom within FLOOR_TOL of the floor. None otherwise."""
    x, y, _ = (float(v) for v in state.pos)
    if abs(x - RACK_X) > POCKET_IN / 2 or _tilt_deg(state) > UPRIGHT_DEG:
        return None
    if abs(float(state.low[2]) - FLOOR) > FLOOR_TOL:
        return None
    for i, py in enumerate(POCKET_Y):
        if abs(y - py) <= POCKET_IN / 2:
            return i
    return None


def _look_alikes_together(objects) -> bool:
    """The red and blue cartons in or on the same or neighbouring pockets (at any tilt): a wrong-drug pick waiting."""
    a, b = (_slot(objects, n) for n in LOOK_ALIKES)
    return a is not None and b is not None and abs(a - b) <= 1


def score(objects):
    """Partial credit (success scores 1.0): per carton, 0.15 for standing upright in a pocket of its own and 0.10 more
    for a wide face toward the front lip within 20 degrees. 0 overall if the red and blue cartons are in or on the
    same or neighbouring pockets (each with its bottom less than 70 mm above the pocket floor, or standing on another
    carton)."""
    if _look_alikes_together(objects):
        return 0.0
    pockets = {n: _shelved(objects[n]) for n in CARTON_NAMES}
    total = 0.0
    for name, pocket in pockets.items():
        if pocket is None or sum(p == pocket for p in pockets.values()) > 1:
            continue  # not shelved, or sharing a pocket
        total += SHELVED_CREDIT
        if _facing_err_deg(objects[name]) <= FACING_DEG:
            total += FACING_CREDIT
    return round(total, 6)


def success(objects):
    """All four shelved, facing the front, the look-alikes apart, and still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in CARTON_NAMES)


# The demo: the left arm shelves the red carton in pocket 1 and the blue one in pocket 3, while the right arm shelves
# the yellow carton in pocket 6, then waits until the left arm has parked and shelves the green carton in pocket 5
# (nearer its base than pocket 4, and a pocket clear of the blue carton).
# Each pick grips across the carton's 24 mm depth (fingers closing along world y, as it starts turned) and turns the
# grip 90 degrees on the way, so the carton arrives with its wide faces toward the front lip and the fingers closing
# front to back, clear of the neighbouring pockets.
TRAVEL_Z = TOP + 0.15  # a carried carton's bottom clears the rack and the shelved cartons' tops by 4.6 cm
GRIP = 0.040  # TCP above a carton's bottom: the pads cover its top 33 mm; the fingertips stay 15 mm above the lips
# Each carton is let go with its bottom 9 mm above the pocket floor: the TCP goes to DROP above where it would be with
# the carton on the floor, and the carton hangs 1 mm lower in the grip than that, because it starts 1 mm above the
# table and the grip closes before it has settled (the kinematic replay keeps that 1 mm; on a real table it is 0 to
# 1 mm). Held there, the carton is outside the 5 mm floor tolerance, so it is not shelved, and success cannot start,
# until it is let go.
DROP = 0.008
# A carton has 3 mm to spare side to side, so it must arrive over its pocket without the overshoot a long, fast carry
# leaves in the arm: the carry stops APPROACH_DY short of the pocket (on its own arm's side), the arm creeps the last
# stretch slowly and settles, then lowers until the carton's bottom is ENTRY_GAP above the divider tops, settles
# again, and only then slides it into the pocket.
APPROACH_DY = 0.03
APPROACH_S = 1.5  # the last 3 cm to above the pocket
SETTLE_S = 0.5  # the arm holds still above the pocket, and again just above the dividers
ENTRY_GAP = 0.006  # the carton's bottom above the divider tops before it goes in
LOWER_S = 1.2  # down to just above the dividers
INSERT_S = 1.2  # the last 14 mm into the pocket
STILL_S = 0.6  # the arm holds still, carton in the pocket, before the fingers open
OPEN_S = 0.5  # the fingers stay open, the arm still, before lifting
# The grip turns a quarter turn either way (a parallel gripper holds the same); the left arm turns clockwise seen
# from above, since its wrist runs out of travel turning the other way over pocket 3.
PLACE_YAW = {"left": -math.pi / 2, "right": math.pi / 2}
PLAN = {"left": (("red_carton", 0), ("blue_carton", 2)), "right": (("yellow_carton", 5), ("green_carton", 4))}
LEFT_PARK = (*START["red_carton"], TRAVEL_Z)
RIGHT_PARK = (*START["green_carton"], TRAVEL_Z)
PARK_S = 1.5
# The right arm's last park is quicker, so it is parked within 2.5 s of letting go of the green carton, before the
# 3 s success hold that starts when the carton lands can end the trial.
LAST_PARK_S = 1.0


def _shelve(side, name, pocket):
    out = []
    py = POCKET_Y[pocket]
    short_y = py + math.copysign(APPROACH_DY, START[name][1] - py)  # short of the pocket, toward the start
    entry_z = FLOOR + LIP_H + ENTRY_GAP + GRIP  # TCP with the carton's bottom ENTRY_GAP above the divider tops
    for w in pick_place_waypoints(
        (*START[name], TOP + GRIP), (RACK_X, py, FLOOR + GRIP + DROP), travel_z=TRAVEL_Z,
        yaw=0.0, place_yaw=PLACE_YAW[side],
    ):
        if w.label == "carry":
            out.append(Waypoint(w.duration, (RACK_X, short_y, TRAVEL_Z), w.finger, "carry, short of the pocket", w.yaw))
            out.append(Waypoint(APPROACH_S, w.pos, w.finger, "creep over the pocket", w.yaw))
            out.append(Waypoint(SETTLE_S, w.pos, w.finger, "settle over the pocket", w.yaw))
        elif w.label == "lower":
            out.append(Waypoint(LOWER_S, (RACK_X, py, entry_z), w.finger, "lower to just above the dividers", w.yaw))
            out.append(Waypoint(SETTLE_S, (RACK_X, py, entry_z), w.finger, "settle above the dividers", w.yaw))
            out.append(Waypoint(INSERT_S, w.pos, w.finger, "slide into the pocket", w.yaw))
            out.append(Waypoint(STILL_S, w.pos, w.finger, "hold still in the pocket", w.yaw))
        elif w.label == "release":
            out.append(w)
            out.append(Waypoint(OPEN_S, w.pos, w.finger, "fingers open, arm still", w.yaw))
        else:
            out.append(w)
    return out


def demo():
    left = []
    for name, pocket in PLAN["left"]:
        left += _shelve("left", name, pocket)
    left.append(Waypoint(PARK_S, LEFT_PARK, FINGER_OPEN, "park"))
    left_done = sum(w.duration for w in left)
    (first, first_pocket), (second, second_pocket) = PLAN["right"]
    right = _shelve("right", first, first_pocket)
    wait_s = left_done - sum(w.duration for w in right) + 0.5  # until half a second after the left arm has parked
    right.append(Waypoint(wait_s, (*START[second], TRAVEL_Z), FINGER_OPEN, "wait for the left arm"))
    right += _shelve("right", second, second_pocket)
    right.append(Waypoint(LAST_PARK_S, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
