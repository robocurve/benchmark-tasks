"""On the station table, in front of the two arms: a tall partition runs straight out from between the arms and splits
the table into a left and a right side. It has one narrow doorway, 8 cm wide. A long wooden board lies on the left
side, parallel to the partition; a grey mat on the right side marks where it has to lie, parallel to the partition
too. The board is much longer than the doorway is wide, so it only goes through end first: it has to be turned
across the partition, pushed through the doorway, taken on the other side and turned back. Neither arm reaches the
far side of the partition, which is too tall to lift the board over; the two meet at the doorway.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- partition: fixed, 1.2 cm thick and 35 cm tall, along x at y 0 from x 0.08 to 0.52, with a doorway from x 0.26
  to 0.34 (two panels, 18 cm long each).
- board: a 30 x 4 x 1.5 cm pine board, 100 g, lying flat along x, centred at (0.30, 0.20).
- mat: a fixed grey mat 34 x 8 cm, 1 mm thick, along x, centred at (0.30, -0.17).

Full score: the board lies flat on the mat along it (its centre within 2 cm of the mat's centre, turned less than
10 degrees from the mat's long side), still for 2 s.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

WALL_COLOR = (0.80, 0.80, 0.76)
WOOD = (0.76, 0.58, 0.36)
MAT_GREY = (0.42, 0.44, 0.47)

WALL_T, WALL_H = 0.012, 0.35
WALL_X0, WALL_X1 = 0.08, 0.52
DOOR_X0, DOOR_X1 = 0.26, 0.34
BOARD = (0.30, 0.04, 0.015)  # x (length), y (width), z (thickness), lying along x
BOARD_KG = 0.10
BOARD_XY = (0.30, 0.20)
MAT = (0.34, 0.08, 0.001)
MAT_XY = (0.30, -0.17)


def _wall():
    z = TOP + WALL_H / 2
    front_len, back_len = DOOR_X0 - WALL_X0, WALL_X1 - DOOR_X1
    return [
        Box("partition_front", pos=(WALL_X0 + front_len / 2, 0.0, z), size=(front_len, WALL_T, WALL_H), color=WALL_COLOR),
        Box("partition_back", pos=(DOOR_X1 + back_len / 2, 0.0, z), size=(back_len, WALL_T, WALL_H), color=WALL_COLOR),
    ]


OBJECTS = _wall() + [
    Box("mat", pos=(MAT_XY[0], MAT_XY[1], TOP + MAT[2] / 2), size=MAT, color=MAT_GREY),
    Box("board", pos=(BOARD_XY[0], BOARD_XY[1], TOP + BOARD[2] / 2 + 0.0005), size=BOARD, mass=BOARD_KG, color=WOOD),
]

# Thresholds, as a reviewer would check them from the video and with a ruler and a protractor.
ON_MAT_XY = 0.02
ALONG = np.radians(10.0)
FLAT = np.radians(5.0)
ON_MAT_Z = 0.005
RIGHT_OF_WALL = -WALL_T / 2  # every part of the board right of the partition's right face

_DEBUG = os.environ.get("PV_DEBUG") == "1"
_state = {"step": 0}


def _yaw(q) -> float:
    x, y, z, w = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _tilt(q) -> float:
    x, y, _, _ = q
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _on_mat(board) -> bool:
    return bool(np.hypot(board.pos[0] - MAT_XY[0], board.pos[1] - MAT_XY[1]) <= ON_MAT_XY
                and abs(np.sin(_yaw(board.quat_xyzw))) <= np.sin(ALONG) and _tilt(board.quat_xyzw) <= FLAT
                and abs(board.low[2] - TOP - MAT[2]) <= ON_MAT_Z)


def _through(board) -> bool:
    return bool(board.high[1] < RIGHT_OF_WALL)


def success(objects):
    board = objects["board"]
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            print(f"DBG step={_state['step']} board=({board.pos[0]:.3f},{board.pos[1]:.3f},{board.pos[2] - TOP:.3f}) "
                  f"yaw={np.degrees(_yaw(board.quat_xyzw)):.0f} tilt={np.degrees(_tilt(board.quat_xyzw)):.0f} "
                  f"y=[{board.low[1]:.3f},{board.high[1]:.3f}] on_mat={_on_mat(board)} through={_through(board)}",
                  flush=True)
    return _on_mat(board) and board.resting()


def distance(objects):
    board = objects["board"]
    return float(np.hypot(board.pos[0] - MAT_XY[0], board.pos[1] - MAT_XY[1]))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the whole board is through the doorway, right of the partition,
    but not lying on the mat as required; 0 otherwise."""
    return 0.5 if _through(objects["board"]) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# The left gripper picks the board up 5 cm from its front end, turns it across the partition while drawing it away
# from the partition, lines it up with the doorway and pushes it end first through until 14 cm stick out on the far
# side. The right gripper takes that end 4 cm from its tip (the two grippers stay 21 cm apart, either side of the
# partition); the left lets go; the right pulls the board through, lays it down, takes it again by its middle, turns
# it back along the partition and lays it on the mat.
TIP = 0.013
OPEN = -0.04  # fingers 8 cm apart round the 4 cm board
CARRY = float(os.environ.get("PV_CARRY", "0.04"))  # board bottom above the table while carried
GRIP_IN = 0.05  # left grip from the board's front end
TURN_Y = float(os.environ.get("PV_TURN_Y", "0.28"))  # left grip while the board is turned (clear of the partition)
PUSH_Y = float(os.environ.get("PV_PUSH_Y", "0.11"))  # left grip when the board is pushed through
TAKE_IN = 0.04  # right grip from the board's far tip
TAKE_LOW = float(os.environ.get("PV_TAKE_LOW", "0.004"))  # right pads above the table when taking the far end
PULL_Y = float(os.environ.get("PV_PULL_Y", "-0.27"))  # right grip once the board is through
DX = (DOOR_X0 + DOOR_X1) / 2  # the doorway's middle, front to back
DX_LEFT = DX + float(os.environ.get("PV_DX_LEFT", "0.017"))  # the board sits ~1.7 cm nearer than the left grip (turned -90 deg)
DX_RIGHT = DX + float(os.environ.get("PV_DX_RIGHT", "0.005"))


def demo():
    tcp_pick = TOP + 0.002 + TIP
    tcp_carry = tcp_pick + CARRY
    tcp_mat = tcp_pick + MAT[2]
    tcp_take = tcp_pick + TAKE_LOW  # the board held by one end droops: its far end is low
    high = tcp_pick + 0.15
    bx, by = BOARD_XY
    mx, my = MAT_XY
    gx = bx - BOARD[0] / 2 + GRIP_IN  # left grip, near the front end
    tip_y = PUSH_Y - (BOARD[0] - GRIP_IN)  # far tip when pushed through
    take_y = tip_y + TAKE_IN
    mid_through = PULL_Y - TAKE_IN + BOARD[0] / 2  # board middle once pulled through
    across = np.pi / 2  # gripper turned so the board lies across the partition (along y)
    turn = -across  # the left gripper turns clockwise (seen from above): the board's far end swings towards the doorway
    left = [
        Waypoint(2.0, (gx, by, tcp_pick + 0.06), OPEN, "above the board, near its back end", 0.0),
        Waypoint(1.0, (gx, by, tcp_pick), OPEN, "down", 0.0),
        Waypoint(0.8, (gx, by, tcp_pick), FINGER_CLOSED, "grip", 0.0),
        Waypoint(0.4, (gx, by, tcp_pick), FINGER_CLOSED, "steady", 0.0),
        Waypoint(1.0, (gx, by, tcp_carry), FINGER_CLOSED, "lift", 0.0),
        Waypoint(2.5, (gx, TURN_Y, tcp_carry), FINGER_CLOSED, "turn it across the partition, away from it", turn),
        Waypoint(1.5, (DX_LEFT, TURN_Y, tcp_carry), FINGER_CLOSED, "line it up with the doorway", turn),
        Waypoint(3.0, (DX_LEFT, PUSH_Y, tcp_carry), FINGER_CLOSED, "push it end first through the doorway", turn),
    ]
    t_push = sum(w.duration for w in left)
    right = [
        Waypoint(t_push - 2.0, (0.25, -0.22, high), OPEN, "wait out of the way", 0.0),
        Waypoint(2.0, (DX, take_y - 0.02, tcp_carry + 0.06), OPEN, "above the end coming through", across),
        Waypoint(1.5, (DX, take_y, tcp_take), OPEN, "down round it (its far end droops)", across),
        Waypoint(0.8, (DX, take_y, tcp_take), FINGER_CLOSED, "grip", across),
        Waypoint(0.6, (DX, take_y, tcp_take), FINGER_CLOSED, "steady", across),
    ]
    t_taken = sum(w.duration for w in right)
    left += [
        Waypoint(t_taken - t_push, (DX_LEFT, PUSH_Y, tcp_carry), FINGER_CLOSED, "hold it while the other gripper takes it",
                 turn),
        Waypoint(0.6, (DX_LEFT, PUSH_Y, tcp_carry), OPEN, "let go", turn),
        Waypoint(1.5, (DX_LEFT, PUSH_Y + 0.06, tcp_carry + 0.10), OPEN, "back away", turn),
        Waypoint(1.5, (0.20, 0.25, high), OPEN, "out of the way", 0.0),
    ]
    right += [
        Waypoint(1.2, (DX, take_y, tcp_take), FINGER_CLOSED, "hold while the other gripper lets go", across),
        Waypoint(3.0, (DX, PULL_Y, tcp_take + 0.01), FINGER_CLOSED, "pull it through", across),
        Waypoint(1.0, (DX, PULL_Y, tcp_pick), FINGER_CLOSED, "lay it down", across),
        Waypoint(0.6, (DX, PULL_Y, tcp_pick), OPEN, "let go", across),
        Waypoint(1.0, (DX, PULL_Y, tcp_pick + 0.06), OPEN, "rise", across),
        Waypoint(1.5, (DX_RIGHT, mid_through, tcp_pick + 0.06), OPEN, "above its middle", across),
        Waypoint(1.0, (DX_RIGHT, mid_through, tcp_pick), OPEN, "down", across),
        Waypoint(0.8, (DX_RIGHT, mid_through, tcp_pick), FINGER_CLOSED, "grip", across),
        Waypoint(0.4, (DX_RIGHT, mid_through, tcp_pick), FINGER_CLOSED, "steady", across),
        Waypoint(1.0, (DX_RIGHT, mid_through, tcp_carry), FINGER_CLOSED, "lift", across),
        Waypoint(2.0, (mx, my, tcp_carry), FINGER_CLOSED, "turn it back along the partition, over the mat", 0.0),
        Waypoint(1.0, (mx, my, tcp_mat), FINGER_CLOSED, "lay it on the mat", 0.0),
        Waypoint(0.6, (mx, my, tcp_mat), OPEN, "let go", 0.0),
        Waypoint(1.0, (mx, my, tcp_mat + 0.08), OPEN, "rise", 0.0),
    ]
    t_left, t_right = sum(w.duration for w in left), sum(w.duration for w in right)
    if t_right > t_left:
        left.append(Waypoint(t_right - t_left, (0.20, 0.25, high), OPEN, "wait", 0.0))
    return {"left": left, "right": right}
