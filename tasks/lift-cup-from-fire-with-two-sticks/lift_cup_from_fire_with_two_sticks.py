"""On the station table, in front of the two arms: a small campfire pit at the far end of the table, a ring of stones
round a bed of embers, and in it, at the back, a tin cup heating. It is out of both arms' reach (on the midline
between the arms they reach about 40 cm forward; the cup stands 60.5 cm forward). Two long wooden sticks lie on the
table, one in front of each arm, and a flat stone lies in front of the fire pit, within reach. The cup has to come out
of the fire and stand on the flat stone. One stick can only push the cup about inside the ring of stones, which is
taller than a stick can lift it; the two sticks used together like tongs, one in each gripper, can pinch the cup,
lift it over the stones and set it down. No gripper can hold both sticks as tongs.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- fire pit: fixed grey stone walls 1.5 cm thick and 4.5 cm tall round a 15 x 15 cm bed of embers (a fixed orange slab
  5 mm thick), centred at (0.58, 0).
- tin cup: a cylinder 6.6 cm across and 8 cm tall, 50 g, standing on the embers at (0.605, 0).
- sticks: two 34 x 1.2 x 1.2 cm wooden sticks, 25 g, lying front to back, centred at (0.36, 0.20) and (0.36, -0.20).
- flat stone: a fixed grey slab 10 x 10 x 2 cm centred at (0.42, 0).

Full score: the cup stands upright on the flat stone (tilted less than 10 degrees, its bottom within 5 mm of the
stone's top, its centre within 4 cm of the stone's centre), still for 2 s.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

STONE_GREY = (0.45, 0.44, 0.42)
EMBER = (0.85, 0.30, 0.06)
TIN = (0.72, 0.73, 0.75)
WOOD = (0.55, 0.36, 0.18)
FLAT_STONE = (0.62, 0.60, 0.56)

PIT_XY, PIT_IN, PIT_T, PIT_H = (0.58, 0.0), 0.15, 0.015, 0.045
EMBER_H = 0.005
CUP_R, CUP_H, CUP_KG = 0.033, 0.08, 0.05
CUP_XY = (0.605, 0.0)
STICK = (0.34, 0.012, 0.012)  # x (length), y, z
STICK_KG = 0.025
STICK_X, STICK_Y = 0.36, 0.20
ROCK_XY, ROCK = (0.42, 0.0), (0.10, 0.10, 0.02)


def _pit():
    cx, cy = PIT_XY
    off = PIT_IN / 2 + PIT_T / 2
    outer = PIT_IN + 2 * PIT_T
    z = TOP + PIT_H / 2
    return [
        Box("pit_front", pos=(cx - off, cy, z), size=(PIT_T, outer, PIT_H), color=STONE_GREY),
        Box("pit_back", pos=(cx + off, cy, z), size=(PIT_T, outer, PIT_H), color=STONE_GREY),
        Box("pit_left", pos=(cx, cy + off, z), size=(PIT_IN, PIT_T, PIT_H), color=STONE_GREY),
        Box("pit_right", pos=(cx, cy - off, z), size=(PIT_IN, PIT_T, PIT_H), color=STONE_GREY),
        Box("embers", pos=(cx, cy, TOP + EMBER_H / 2), size=(PIT_IN, PIT_IN, EMBER_H), color=EMBER),
    ]


OBJECTS = (
    _pit()
    + [
        Box("flat_stone", pos=(ROCK_XY[0], ROCK_XY[1], TOP + ROCK[2] / 2), size=ROCK, color=FLAT_STONE),
        Box("stick_left", pos=(STICK_X, STICK_Y, TOP + STICK[2] / 2 + 0.0005), size=STICK, mass=STICK_KG, color=WOOD),
        Box("stick_right", pos=(STICK_X, -STICK_Y, TOP + STICK[2] / 2 + 0.0005), size=STICK, mass=STICK_KG,
            color=WOOD),
        Cylinder("tin_cup", pos=(CUP_XY[0], CUP_XY[1], TOP + EMBER_H + 0.0005 + CUP_H / 2), radius=CUP_R,
                 height=CUP_H, mass=CUP_KG, color=TIN),
    ]
)

# Thresholds, as a reviewer would check them from the video and with a ruler.
ON_STONE_XY = 0.04
ON_STONE_Z = 0.005
UPRIGHT = np.radians(10.0)
PIT_X_MIN = PIT_XY[0] - PIT_IN / 2 - PIT_T  # the fire pit's front face

_DEBUG = os.environ.get("LC_DEBUG") == "1"
_state = {"step": 0}


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _on_stone(cup) -> bool:
    return bool(np.hypot(cup.pos[0] - ROCK_XY[0], cup.pos[1] - ROCK_XY[1]) <= ON_STONE_XY
                and abs(cup.low[2] - TOP - ROCK[2]) <= ON_STONE_Z and _tilt(cup.quat_xyzw) <= UPRIGHT)


def _out_of_fire(cup) -> bool:
    return bool(cup.high[0] < PIT_X_MIN)


def success(objects):
    cup = objects["tin_cup"]
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            sl, sr = objects["stick_left"], objects["stick_right"]
            print(f"DBG step={_state['step']} cup=({cup.pos[0]:.3f},{cup.pos[1]:.3f},b{cup.low[2] - TOP:.3f}) "
                  f"tilt={np.degrees(_tilt(cup.quat_xyzw)):.1f} sticks=({sl.pos[0]:.3f},{sl.pos[1]:.3f},"
                  f"{sl.pos[2] - TOP:.3f})({sr.pos[0]:.3f},{sr.pos[1]:.3f},{sr.pos[2] - TOP:.3f}) "
                  f"on_stone={_on_stone(cup)} out={_out_of_fire(cup)}", flush=True)
    return _on_stone(cup) and cup.resting()


def distance(objects):
    cup = objects["tin_cup"]
    return float(np.hypot(cup.pos[0] - ROCK_XY[0], cup.pos[1] - ROCK_XY[1]) + abs(cup.low[2] - TOP - ROCK[2]))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the cup is out of the fire pit (wholly in front of it) but not
    standing on the flat stone; 0 otherwise."""
    return 0.5 if _out_of_fire(objects["tin_cup"]) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# Each gripper picks up the stick in front of it by its back end and turns it inward, so the sticks point forward and
# converge like the two halves of a pair of tongs while the grippers stay 22-28 cm apart. Above the stones, the stick
# ends go in on either side of the cup; the grippers move inward so the stick ends pinch it, then lift it over the
# stones, carry it back and set it down on the flat stone, and move apart to let go.
TIP = 0.013
GRIP_BACK = 0.09  # grip point from the stick's back end (well in: the arm may stop short of the target)
AHEAD = STICK[0] - GRIP_BACK  # stick end ahead of the grip point (0.25)
THETA = float(os.environ.get("LC_THETA_DEG", "22")) * np.pi / 180  # each stick turned inward by this angle
OPEN_TIP_Y = float(os.environ.get("LC_OPEN_TIP_Y", "0.050"))  # stick-end centreline off the midline, going in
SQUEEZE_TIP_Y = float(os.environ.get("LC_SQUEEZE_TIP_Y", "0.020"))  # commanded: ~2 cm inside the cup's side
TIP_PAST = float(os.environ.get("LC_TIP_PAST", "0.0"))  # stick ends this far beyond the cup's centre (0: the ends
# press on the cup's sides square on, with no push towards the grippers)
STICK_LOW = float(os.environ.get("LC_STICK_LOW", "0.052"))  # stick bottom above the table inside the pit
LIFT = float(os.environ.get("LC_LIFT", "0.06"))
SET_DOWN = float(os.environ.get("LC_SET_DOWN", "0.006"))  # cup bottom above the stone's top when letting go
STICK_OPEN = -0.015  # fingers opened 3 cm round a 1.2 cm stick (the scripted gripper closes at full range per second)


def _dur(plan):
    return sum(w.duration for w in plan)


def _grip(tip_x, tip_y_abs, side):
    """Grip point (TCP xy) for a stick whose end is at (tip_x, side * tip_y_abs), turned inward by THETA."""
    return (tip_x - AHEAD * np.cos(THETA), side * (tip_y_abs + AHEAD * np.sin(THETA)))


def demo():
    tcp_pick = TOP + 0.002 + TIP
    tcp_carry = TOP + STICK_LOW + 0.002 + TIP  # stick bottom at STICK_LOW
    tcp_lift = tcp_carry + LIFT
    cup_bottom_on_embers = EMBER_H
    tcp_set = tcp_carry + (ROCK[2] + SET_DOWN - cup_bottom_on_embers)
    plans = {}
    for arm, side in (("left", 1.0), ("right", -1.0)):
        yaw_in = -side * THETA
        gx0 = STICK_X - STICK[0] / 2 + GRIP_BACK
        gy0 = side * STICK_Y
        tip_in = CUP_XY[0] + TIP_PAST
        g_open = _grip(tip_in, OPEN_TIP_Y, side)
        g_squeeze = _grip(tip_in, SQUEEZE_TIP_Y, side)
        g_front = _grip(PIT_X_MIN - 0.03, OPEN_TIP_Y, side)  # stick ends just in front of the stones
        g_set = _grip(ROCK_XY[0] + TIP_PAST, SQUEEZE_TIP_Y, side)
        g_free = _grip(ROCK_XY[0] + TIP_PAST, OPEN_TIP_Y + 0.02, side)
        plans[arm] = [
            Waypoint(2.0, (gx0, gy0, tcp_pick + 0.06), STICK_OPEN, "above the stick's back end", 0.0),
            Waypoint(1.0, (gx0, gy0, tcp_pick), STICK_OPEN, "down", 0.0),
            Waypoint(0.8, (gx0, gy0, tcp_pick), FINGER_CLOSED, "grip the stick", 0.0),
            Waypoint(0.3, (gx0, gy0, tcp_pick), FINGER_CLOSED, "steady", 0.0),
            Waypoint(1.0, (gx0, gy0, tcp_carry + 0.02), FINGER_CLOSED, "lift it", 0.0),
            Waypoint(2.0, (*g_front, tcp_carry + 0.02), FINGER_CLOSED, "turn it inward, ends in front of the fire",
                     yaw_in),
            Waypoint(2.5, (*g_open, tcp_carry), FINGER_CLOSED, "ends in, either side of the cup", yaw_in),
            Waypoint(1.5, (*g_squeeze, tcp_carry), FINGER_CLOSED, "pinch the cup", yaw_in),
            Waypoint(0.5, (*g_squeeze, tcp_carry), FINGER_CLOSED, "steady", yaw_in),
            Waypoint(1.5, (*g_squeeze, tcp_lift), FINGER_CLOSED, "lift it over the stones", yaw_in),
            Waypoint(3.0, (*g_set, tcp_lift), FINGER_CLOSED, "bring it over the flat stone", yaw_in),
            Waypoint(1.5, (*g_set, tcp_set), FINGER_CLOSED, "lower it onto the stone", yaw_in),
            Waypoint(0.5, (*g_set, tcp_set), FINGER_CLOSED, "steady", yaw_in),
            Waypoint(1.0, (*g_free, tcp_set), FINGER_CLOSED, "move apart: let go", yaw_in),
            Waypoint(1.0, (*g_free, tcp_set + 0.05), FINGER_CLOSED, "rise", yaw_in),
        ]
    return plans
