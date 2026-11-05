"""On the station table, in front of the two arms: a big blue ball lying on the table, two square wooden battens lying
on either side of it (one in front of it, one behind it, both running left to right), and a stand nearer the arms: a
dark plinth 6 cm tall with a brass holder on top, a short slot the ball sits in. The ball has to go into the holder.
It is 15 cm across, much wider than a gripper opens, and round, so it rolls out of a squeeze between the two grippers;
it cannot be rolled up onto the stand either. The two battens, one in each gripper, brought against the ball low down
from front and back, cradle it on their top edges when they rise together; carried level across the battens and
lowered on either side of the plinth, they leave the ball in the holder.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- ball: a sphere 15 cm across, 150 g, resting on the table at (0.33, 0).
- battens: two 2 x 40 x 2 cm wooden battens, 80 g, lying left to right, centred at (0.245, 0) (in front of the
  ball) and (0.415, 0) (behind it).
- stand: a fixed dark plinth 4 x 6 x 6 cm centred at (0.20, 0.062), with a fixed brass holder on top: a frame 6 mm
  tall, 4 x 6 cm outside, round a slot 2.4 cm wide (front to back) and 4 cm long (left to right).

Full score: the ball rests in the holder's slot (its centre within 1 cm of the slot's centre line front to back and
within 2 cm of the slot's middle left to right, its lowest point from 1 cm below to 5 mm above the holder's top),
still for 2 s.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "best"

BALL_COLOR = (0.12, 0.52, 0.62)
WOOD = (0.55, 0.36, 0.18)
PLINTH_GREY = (0.28, 0.28, 0.30)
BRASS = (0.78, 0.62, 0.26)

BALL_R = 0.075
BALL_KG = float(os.environ.get("BC_BALL_KG", "0.15"))
BALL_XY = (0.33, 0.0)
STICK = (0.02, 0.40, 0.02)  # x, y (length), z
STICK_KG = 0.08
STICK_GAP0 = 0.085  # each batten's centre line starts this far in front of / behind the ball's centre

STAND_XY = (0.20, 0.062)  # 6.2 cm left of the centre line, under where the carried ball rests
PLINTH = (0.04, 0.06, 0.06)
HOLDER_H = 0.006
SLOT = (0.024, 0.04)  # x (front to back), y (left to right)
BAR = (PLINTH[0] - SLOT[0]) / 2  # width of the holder's front and back bars (8 mm)
HOLDER_TOP = PLINTH[2] + HOLDER_H  # above the table


def _stand():
    cx, cy = STAND_XY
    zb = TOP + PLINTH[2] / 2
    zh = TOP + PLINTH[2] + HOLDER_H / 2
    end_w = (PLINTH[1] - SLOT[1]) / 2
    return [
        Box("plinth", pos=(cx, cy, zb), size=PLINTH, color=PLINTH_GREY),
        Box("holder_front", pos=(cx - SLOT[0] / 2 - BAR / 2, cy, zh), size=(BAR, PLINTH[1], HOLDER_H), color=BRASS),
        Box("holder_back", pos=(cx + SLOT[0] / 2 + BAR / 2, cy, zh), size=(BAR, PLINTH[1], HOLDER_H), color=BRASS),
        Box("holder_left", pos=(cx, cy + SLOT[1] / 2 + end_w / 2, zh), size=(SLOT[0], end_w, HOLDER_H), color=BRASS),
        Box("holder_right", pos=(cx, cy - SLOT[1] / 2 - end_w / 2, zh), size=(SLOT[0], end_w, HOLDER_H), color=BRASS),
    ]


OBJECTS = _stand() + [
    Box("batten_front", pos=(BALL_XY[0] - STICK_GAP0, 0.0, TOP + STICK[2] / 2 + 0.0005), size=STICK, mass=STICK_KG,
        color=WOOD),
    Box("batten_back", pos=(BALL_XY[0] + STICK_GAP0, 0.0, TOP + STICK[2] / 2 + 0.0005), size=STICK, mass=STICK_KG,
        color=WOOD),
    Sphere("ball", pos=(BALL_XY[0], BALL_XY[1], TOP + BALL_R + 0.0005), radius=BALL_R, mass=BALL_KG,
           color=BALL_COLOR),
]

# Thresholds, as a reviewer would check them from the video and with a ruler.
IN_SLOT_X = 0.01
IN_SLOT_Y = 0.02
SEAT_BELOW, SEAT_ABOVE = 0.01, 0.005

_DEBUG = os.environ.get("BC_DEBUG") == "1"
_state = {"step": 0}


def _bottom(ball) -> float:
    """Height of the ball's lowest point above the table (its bounding box grows when it turns)."""
    return float(ball.pos[2] - BALL_R - TOP)


def _in_holder(ball) -> bool:
    return bool(abs(ball.pos[0] - STAND_XY[0]) <= IN_SLOT_X and abs(ball.pos[1] - STAND_XY[1]) <= IN_SLOT_Y
                and HOLDER_TOP - SEAT_BELOW <= _bottom(ball) <= HOLDER_TOP + SEAT_ABOVE)


def _lifted(ball) -> bool:
    return _bottom(ball) > HOLDER_TOP + SEAT_ABOVE


def success(objects):
    ball = objects["ball"]
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            sf, sb = objects["batten_front"], objects["batten_back"]
            print(f"DBG step={_state['step']} ball=({ball.pos[0]:.3f},{ball.pos[1]:.3f},b{_bottom(ball):.3f}) "
                  f"sf=({sf.pos[0]:.3f},{sf.pos[1]:.3f},{sf.pos[2] - TOP:.3f}) "
                  f"sb=({sb.pos[0]:.3f},{sb.pos[1]:.3f},{sb.pos[2] - TOP:.3f}) "
                  f"in={_in_holder(ball)} lifted={_lifted(ball)}", flush=True)
    return _in_holder(ball) and ball.resting()


def distance(objects):
    ball = objects["ball"]
    return float(np.hypot(ball.pos[0] - STAND_XY[0], ball.pos[1] - STAND_XY[1]) + abs(_bottom(ball) - HOLDER_TOP))


def score(objects):
    """Partial credit (success scores 1.0), best over the trial: 0.5 once the ball has been lifted with its lowest
    point higher than the top of the holder; 0 otherwise."""
    return 0.5 if _lifted(objects["ball"]) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# The left gripper takes the front batten 4 cm from its left end, the right gripper the back batten 4 cm from its
# right end (fingers closing front to back). Both battens slide in together until their top inner edges touch the
# ball low down, then rise together with the ball resting on the two edges. Each batten, held at one end, sags a
# little, so the ball rolls along them until it rests against the left gripper, which then keeps it there; the stand
# stands under that spot. The battens carry the ball back to the stand (across the battens, so it cannot roll off
# them), lower on either side of the plinth so the ball settles in the holder, and let go: they drop to the table,
# clear of the ball.
TIP = 0.013
GRIP_Y = float(os.environ.get("BC_GRIP_Y", "0.16"))  # grip point left / right of the centre line
SQ = float(os.environ.get("BC_SQ", "0.059"))  # commanded batten centre line off the ball's centre at contact
SLIDE = float(os.environ.get("BC_SLIDE", "0.002"))  # batten bottom above the table while sliding in
LIFT_CLEAR = float(os.environ.get("BC_LIFT_CLEAR", "0.02"))  # ball bottom above the holder's top when carried
SET_DROP = float(os.environ.get("BC_SET_DROP", "0.025"))  # battens lowered this far below the holder's top
STICK_OPEN = -0.02  # fingers opened 4 cm round a 2 cm batten (the scripted gripper closes at full range per second)


def demo():
    tcp_pick = TOP + 0.002 + TIP
    tcp_slide = TOP + SLIDE + 0.002 + TIP
    # Resting on the two top edges, the ball's lowest point is about level with the battens' bottoms.
    tcp_carry = tcp_slide + HOLDER_TOP + LIFT_CLEAR
    tcp_set = TOP + HOLDER_TOP - SET_DROP + 0.002 + TIP
    bx = BALL_XY[0]
    sx = STAND_XY[0]
    plans = {}
    for arm, side, front in (("left", 1.0, -1.0), ("right", -1.0, 1.0)):
        yaw = -side * np.pi / 2  # fingers close front to back
        gy = side * GRIP_Y
        x0 = bx + front * STICK_GAP0
        xin = bx + front * SQ
        xst = sx + front * SQ
        plans[arm] = [
            Waypoint(2.0, (x0, gy, tcp_pick + 0.06), STICK_OPEN, "above the batten's end", yaw),
            Waypoint(1.0, (x0, gy, tcp_pick), STICK_OPEN, "down", yaw),
            Waypoint(0.8, (x0, gy, tcp_pick), FINGER_CLOSED, "grip the batten", yaw),
            Waypoint(0.4, (x0, gy, tcp_pick), FINGER_CLOSED, "steady", yaw),
            Waypoint(0.5, (x0, gy, tcp_slide), FINGER_CLOSED, "just off the table", yaw),
            Waypoint(2.0, (xin, gy, tcp_slide), FINGER_CLOSED, "slide in against the ball, low down", yaw),
            Waypoint(0.5, (xin, gy, tcp_slide), FINGER_CLOSED, "steady", yaw),
            Waypoint(3.0, (xin, gy, tcp_carry), FINGER_CLOSED, "lift it together, level", yaw),
            Waypoint(0.5, (xin, gy, tcp_carry), FINGER_CLOSED, "steady", yaw),
            Waypoint(3.0, (xst, gy, tcp_carry), FINGER_CLOSED, "carry it over the stand", yaw),
            Waypoint(1.0, (xst, gy, tcp_carry), FINGER_CLOSED, "settle", yaw),
            Waypoint(2.5, (xst, gy, tcp_set), FINGER_CLOSED, "lower on either side of the plinth", yaw),
            Waypoint(0.5, (xst, gy, tcp_set), FINGER_CLOSED, "steady", yaw),
            Waypoint(0.6, (xst, gy, tcp_set), STICK_OPEN, "let go: the batten drops to the table", yaw),
            Waypoint(0.5, (xst, gy, tcp_set), STICK_OPEN, "wait", yaw),
            Waypoint(1.5, (xst, gy, tcp_set + 0.08), STICK_OPEN, "rise", yaw),
        ]
    return plans
