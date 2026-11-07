"""On the station table, in front of the two arms: a section of decking, three boards side by side in a frame, running
from left to right. The middle board rocks: a pebble is trapped under it near its left end, so its left end sits a
little high and its right end on the floor of its slot. It has to be fixed: the board taken out, the pebble put in
the bucket, and the board laid back flat in its slot. The board fits its slot with only 7 mm to spare on each side and
the frame closes both ends, so no gripper can get round it to take hold. Pressed down at its left end, it pivots on
the pebble like a seesaw and its right end rises clear of the deck, but only while the left end is held down: one
gripper presses while the other takes hold of the risen end.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- frame: fixed dark wood end strips (1 cm thick, 2.5 cm tall) across both ends of the boards, at y +0.137 and -0.137.
- fixed boards: two light wood boards, 7 cm front to back x 25 cm side to side x 2.5 cm thick, centred at (0.183, 0)
  and (0.337, 0).
- loose board: the same size, 150 g, darker, in the 8.4 cm wide slot between them (centred at (0.26, 0) when flat).
- pebble: a grey stone, 2 x 2 cm and 8 mm thick, 20 g, under the loose board 4.5 cm from its left end.
- bucket: fixed black walls 1 cm thick and 3 cm tall round a 7 x 7 cm floor, centred at (0.15, 0.27).

Full score: the loose board lies flat in its slot, flush with the boards beside it (no part of it more than 3 mm above
their tops, its bottom within 3 mm of the slot floor, its centre within 1 cm of the slot's centre), and the pebble lies
in the bucket, everything still for 2 s. At the start the board's left end stands about 1 cm proud.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

LIGHT_WOOD = (0.76, 0.60, 0.40)
DARK_WOOD = (0.36, 0.22, 0.12)
LOOSE_WOOD = (0.58, 0.38, 0.22)
STONE = (0.48, 0.48, 0.46)
BLACK = (0.05, 0.05, 0.06)

BOARD = (0.07, 0.25, 0.025)  # x (width), y (length), z (thickness)
BOARD_KG = 0.15
GAP = 0.007
DECK_X = 0.26
NEIGHBOUR_DX = BOARD[0] + GAP  # 0.077
LEFT_Y = BOARD[1] / 2  # 0.125: the loose board's left end when flat
STRIP_T = 0.01
STRIP_Y = LEFT_Y + GAP + STRIP_T / 2  # 0.137
PEBBLE, PEBBLE_KG = (0.02, 0.02, 0.008), 0.02
PEBBLE_Y = LEFT_Y - 0.045  # 0.08
BUCKET_XY, BUCKET_IN, BUCKET_T, BUCKET_H = (0.15, 0.27), 0.07, 0.01, 0.03


def _bucket():
    cx, cy = BUCKET_XY
    off = BUCKET_IN / 2 + BUCKET_T / 2
    outer = BUCKET_IN + 2 * BUCKET_T
    z = TOP + BUCKET_H / 2
    return [
        Box("bucket_front", pos=(cx - off, cy, z), size=(BUCKET_T, outer, BUCKET_H), color=BLACK),
        Box("bucket_back", pos=(cx + off, cy, z), size=(BUCKET_T, outer, BUCKET_H), color=BLACK),
        Box("bucket_left", pos=(cx, cy + off, z), size=(BUCKET_IN, BUCKET_T, BUCKET_H), color=BLACK),
        Box("bucket_right", pos=(cx, cy - off, z), size=(BUCKET_IN, BUCKET_T, BUCKET_H), color=BLACK),
    ]


STRIP_LEN = 3 * BOARD[0] + 2 * GAP + 0.02
OBJECTS = (
    [
        Box("strip_left", pos=(DECK_X, STRIP_Y, TOP + BOARD[2] / 2), size=(STRIP_LEN, STRIP_T, BOARD[2]), color=DARK_WOOD),
        Box("strip_right", pos=(DECK_X, -STRIP_Y, TOP + BOARD[2] / 2), size=(STRIP_LEN, STRIP_T, BOARD[2]), color=DARK_WOOD),
        Box("board_front", pos=(DECK_X - NEIGHBOUR_DX, 0.0, TOP + BOARD[2] / 2), size=BOARD, color=LIGHT_WOOD),
        Box("board_back", pos=(DECK_X + NEIGHBOUR_DX, 0.0, TOP + BOARD[2] / 2), size=BOARD, color=LIGHT_WOOD),
    ]
    + _bucket()
    + [
        Box("pebble", pos=(DECK_X, PEBBLE_Y, TOP + 0.0005 + PEBBLE[2] / 2), size=PEBBLE, mass=PEBBLE_KG, color=STONE),
        # starts level, just above the pebble; it settles onto the pebble and its right end
        Box("loose_board", pos=(DECK_X, 0.0, TOP + PEBBLE[2] + 0.002 + BOARD[2] / 2), size=BOARD, mass=BOARD_KG,
            color=LOOSE_WOOD),
    ]
)

# Thresholds, as a reviewer would check them from the video and with a ruler.
FLUSH = 0.003  # no part of the board more than this above the deck's top, its bottom this close to the slot floor
IN_SLOT_XY = 0.01
IN_BUCKET_Z = 0.015

_DEBUG = os.environ.get("RB_DEBUG") == "1"
_state = {"step": 0}


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _bottom(obj) -> float:
    return float(obj.low[2] - TOP)


def _proud(board) -> float:
    return float(board.high[2] - TOP - BOARD[2])


def _board_flat_in_slot(board) -> bool:
    return bool(abs(board.pos[0] - DECK_X) <= IN_SLOT_XY and abs(board.pos[1]) <= IN_SLOT_XY
                and _bottom(board) <= FLUSH and _proud(board) <= FLUSH)


def _pebble_in_bucket(pebble) -> bool:
    return bool(abs(pebble.pos[0] - BUCKET_XY[0]) <= BUCKET_IN / 2 and abs(pebble.pos[1] - BUCKET_XY[1]) <= BUCKET_IN / 2
                and _bottom(pebble) <= IN_BUCKET_Z)


def success(objects):
    board, pebble = objects["loose_board"], objects["pebble"]
    ok = _board_flat_in_slot(board) and _pebble_in_bucket(pebble)
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            print(f"DBG step={_state['step']} board=({board.pos[0]:.3f},{board.pos[1]:.3f},{board.pos[2] - TOP:.3f}) "
                  f"low={_bottom(board):.3f} high={board.high[2] - TOP:.3f} tilt={np.degrees(_tilt(board.quat_xyzw)):.1f} "
                  f"pebble=({pebble.pos[0]:.3f},{pebble.pos[1]:.3f},b{_bottom(pebble):.3f}) "
                  f"flat_in_slot={_board_flat_in_slot(board)} pebble_in_bucket={_pebble_in_bucket(pebble)}", flush=True)
    return ok and board.resting() and pebble.resting()


def distance(objects):
    board, pebble = objects["loose_board"], objects["pebble"]
    d = float(np.hypot(board.pos[0] - DECK_X, board.pos[1])) + _bottom(board) + max(0.0, _proud(board))
    d += float(np.hypot(pebble.pos[0] - BUCKET_XY[0], pebble.pos[1] - BUCKET_XY[1]))
    return d


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the pebble is in the bucket but the board is not lying flat in
    its slot; 0 otherwise."""
    return 0.5 if _pebble_in_bucket(objects["pebble"]) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# The left gripper, closed, presses the board's high left end down; the board pivots on the pebble and its right end
# rises about 4.5 cm. The right gripper (turned a quarter turn: fingers closing front to back, across the board) takes
# the risen end, 21 cm from the left one, lifts the whole board clear and holds it out to the right. The left gripper
# picks the pebble out of the uncovered end of the slot and drops it in the bucket. The right gripper lowers the board
# back: its low left end goes into the slot first, and the board is let go just above the deck.
TIP = 0.013
PRESS_Y = LEFT_Y - 0.010
# commanded pad-tip height above the table when pressing: well below where the board stops (its top at about 2.7 cm
# there, once its left end is on the slot floor), so the arm presses with a few newtons
PRESS_TIPS = float(os.environ.get("RB_PRESS_TIPS", "0.016"))
GRAB_Y = -LEFT_Y + float(os.environ.get("RB_GRAB_IN", "0.025"))
GRAB_TIPS = float(os.environ.get("RB_GRAB_TIPS", "0.050"))  # pad-tip height when taking the risen end
LIFT = float(os.environ.get("RB_LIFT", "0.05"))
ASIDE_DY = float(os.environ.get("RB_ASIDE_DY", "-0.13"))
SEAT_DX = float(os.environ.get("RB_SEAT_DX", "0.0"))
RELEASE_TIPS = float(os.environ.get("RB_RELEASE_TIPS", "0.030"))  # pad tips this far above the deck when letting go
PEBBLE_OPEN = -0.02  # fingers opened just enough for the pebble, clear of the end strip
HOVER = TOP + 0.11


def _dur(plan):
    return sum(w.duration for w in plan)


def demo():
    yl, yr = 0.0, np.pi / 2
    press_hi = TOP + BOARD[2] + 0.03 + TIP
    press_lo = TOP + PRESS_TIPS + TIP  # the high end pushed down until it meets the slot floor
    grab_z = TOP + GRAB_TIPS + TIP
    x = DECK_X
    left = [
        Waypoint(2.0, (x, PRESS_Y, press_hi), FINGER_CLOSED, "above the high left end", yl),
        Waypoint(1.0, (x, PRESS_Y, TOP + BOARD[2] + 0.012 + TIP), FINGER_CLOSED, "onto it", yl),
        Waypoint(1.5, (x, PRESS_Y, press_lo), FINGER_CLOSED, "press it down: the right end rises", yl),
    ]
    right = [
        Waypoint(2.5, (x, GRAB_Y, HOVER), FINGER_OPEN, "above the right end", yr),
        Waypoint(2.0, (x, GRAB_Y, HOVER), FINGER_OPEN, "wait for it to rise", yr),
    ]
    t_up = _dur(left) + 0.5
    left.append(Waypoint(t_up - _dur(left), (x, PRESS_Y, press_lo), FINGER_CLOSED, "hold it down", yl))
    right.append(Waypoint(t_up - _dur(right), (x, GRAB_Y, HOVER), FINGER_OPEN, "wait", yr))
    right += [
        Waypoint(1.2, (x, GRAB_Y, grab_z), FINGER_OPEN, "down round the risen end", yr),
        Waypoint(0.8, (x, GRAB_Y, grab_z), FINGER_CLOSED, "take hold", yr),
        Waypoint(0.5, (x, GRAB_Y, grab_z), FINGER_CLOSED, "steady", yr),
    ]
    left.append(Waypoint(_dur(right) - _dur(left), (x, PRESS_Y, press_lo), FINGER_CLOSED, "keep holding", yl))
    # the left gripper lets go of the left end as the right one lifts the board out and holds it aside
    left += [
        Waypoint(0.8, (x, PRESS_Y, press_hi + 0.03), FINGER_CLOSED, "let go of the left end", yl),
        Waypoint(1.2, (0.18, 0.22, HOVER), PEBBLE_OPEN, "out of the way", yl),
    ]
    right += [
        Waypoint(1.5, (x, GRAB_Y, grab_z + LIFT), FINGER_CLOSED, "lift the board out", yr),
        Waypoint(1.5, (x, GRAB_Y + ASIDE_DY, grab_z + LIFT), FINGER_CLOSED, "hold it aside", yr),
    ]
    # the left gripper picks the pebble out of the uncovered end of the slot and drops it in the bucket
    left.append(Waypoint(max(0.1, _dur(right) - _dur(left)), (0.18, 0.22, HOVER), PEBBLE_OPEN, "wait", yl))
    pz = TOP + 0.002 + TIP
    left += [
        Waypoint(1.5, (x, PEBBLE_Y, TOP + 0.06 + TIP), PEBBLE_OPEN, "above the pebble", yl),
        Waypoint(1.0, (x, PEBBLE_Y, pz), PEBBLE_OPEN, "down into the slot", yl),
        Waypoint(0.8, (x, PEBBLE_Y, pz), FINGER_CLOSED, "pick the pebble", yl),
        Waypoint(1.0, (x, PEBBLE_Y, TOP + 0.07 + TIP), FINGER_CLOSED, "up", yl),
        Waypoint(1.5, (*BUCKET_XY, TOP + 0.07 + TIP), FINGER_CLOSED, "over the bucket", yl),
        Waypoint(0.8, (*BUCKET_XY, TOP + 0.03 + TIP), FINGER_CLOSED, "lower", yl),
        Waypoint(0.5, (*BUCKET_XY, TOP + 0.03 + TIP), PEBBLE_OPEN, "drop it in", yl),
        Waypoint(1.0, (0.18, 0.24, HOVER), PEBBLE_OPEN, "out of the way", yl),
    ]
    right.append(Waypoint(_dur(left) - _dur(right), (x, GRAB_Y + ASIDE_DY, grab_z + LIFT), FINGER_CLOSED, "hold", yr))
    # the right gripper lays the board back: low left end into the slot first, let go just above the deck
    rel_z = TOP + BOARD[2] + RELEASE_TIPS + TIP
    sx = x + SEAT_DX
    right += [
        Waypoint(1.5, (sx, GRAB_Y, grab_z + LIFT), FINGER_CLOSED, "back over the slot", yr),
        Waypoint(1.5, (sx, GRAB_Y, rel_z + 0.02), FINGER_CLOSED, "lower: left end into the slot", yr),
        Waypoint(1.0, (sx, GRAB_Y, rel_z), FINGER_CLOSED, "down to the deck", yr),
        Waypoint(0.6, (sx, GRAB_Y, rel_z), FINGER_OPEN, "let go", yr),
        Waypoint(1.2, (sx, GRAB_Y, HOVER), FINGER_OPEN, "rise", yr),
        Waypoint(1.2, (0.20, -0.24, HOVER), FINGER_OPEN, "out of the way", yr),
    ]
    end = max(_dur(left), _dur(right))
    for plan, y in ((left, yl), (right, yr)):
        if _dur(plan) < end:
            plan.append(Waypoint(end - _dur(plan), plan[-1].pos, plan[-1].finger, "wait", y))
    return {"left": left, "right": right}
