"""On the station table, in front of the arms: a long narrow oak board (84.8 x 8 x 1.6 cm, 0.81 kg) lies flat across
both arms' work areas, along the left-right axis, 0.28 m in front of the arm bases. Its left end is in front of the
left arm (0.40 m left of centre) and its right end is 0.448 m right of centre. A soft red foam ball, 10 cm across,
rests on the board's right part, 0.358 m right of centre.

The board lies in a fixed grey guide slot: two upright boards, 16 cm tall, 5 mm from each side of the board, run from
0.30 m left of centre to 0.19 m right of centre. There are two open gaps where the board can be held: its left end
(in front of the left arm, beyond the slot) and a 13 cm gap 0.25 m right of centre (in front of the right arm).
Right of that gap, two fixed blue guide rails, 14 cm tall and 12 mm from each side of the board, keep the ball on the
board. Past the board's right end stands a fixed open yellow cup: inner 14 x 14 cm, walls 7.5 cm tall, its near wall
1 cm from the end of the board. The board and the ball are loose; the slot, the rails and the cup are fixed.

The ball is wider than an open gripper. The cup's rim is higher than the centre of the ball while the board lies on
the table, so the ball goes in only if the board's right end is lifted above the rim and then tipped down.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \\
           --instruction "Hold the board with both grippers, one in each open gap of the guide slot, lift it clear of
           the table and tilt it so the ball rolls off the right end of the board into the cup." --scorer task_score ...
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# "end": the final state decides. With "best", a ball that rode the lifted board (0.3 or more) and was then dropped
# on the table or the floor would keep that credit, contradicting "0 if the ball leaves the board".
SCORE_OVER = "end"

REST_GAP = 0.001  # loose objects spawn this far above their support and settle onto it
TABLE_X = (-0.19, 0.91)  # the station table's footprint (station.TABLE_CENTER +- TABLE_SIZE / 2)
TABLE_Y = (-0.65, 0.65)

# -- layout along the board (world y: left +, right -) ----------------------------------------------------------------
X_BOARD = 0.28  # the board's centre line, metres in front of the arm bases
Y_LEFT_GRIP = 0.365  # where the left gripper holds the board, 3.5 cm in from its left end
Y_RIGHT_GRIP = -0.25  # where the right gripper holds the board, in front of the right arm base (y -0.305)
GRIP_HALF = 0.034  # a straight-down gripper's half-extent along the board at yaw 90 deg (finger bodies, URDF meshes)
GAP_HALF = GRIP_HALF + 0.030  # half-length of each open gap: the gripper plus 3 cm of clearance

# -- the board: an oak strip ripped to 80 x 16 mm -------------------------------------------------------------------------
BOARD_W, BOARD_T = 0.080, 0.016  # gripped across its 80 mm width (gripper opens 94 mm; 1.4 cm to spare)
BALL_R = 0.05  # 100 mm foam ball: wider than the 94 mm open gripper, so it cannot be picked up
Y_BALL0 = Y_RIGHT_GRIP - 0.108  # the ball starts clear of the right gripper (finger bodies end 0.083 from its centre)
EXIT_GAP = 0.040  # board surface left between the ball's start and the board's right end
Y_BOARD_RIGHT = Y_BALL0 - BALL_R - EXIT_GAP  # -0.448
Y_BOARD_LEFT = Y_LEFT_GRIP + 0.035  # 0.400
BOARD_LEN = Y_BOARD_LEFT - Y_BOARD_RIGHT  # 0.848
BOARD_MASS = 0.81  # oak, about 750 kg/m^3
BOARD = Box(
    "board",
    pos=(X_BOARD, (Y_BOARD_LEFT + Y_BOARD_RIGHT) / 2, TOP + REST_GAP + BOARD_T / 2),
    size=(BOARD_W, BOARD_LEN, BOARD_T),
    mass=BOARD_MASS,
    color=(0.62, 0.45, 0.27),
)
BALL = Sphere(
    "ball",
    pos=(X_BOARD, Y_BALL0, TOP + REST_GAP + BOARD_T + REST_GAP + BALL_R),
    radius=BALL_R,
    mass=0.05,
    color=(0.85, 0.12, 0.12),
)

# -- the guide slot: two upright boards 5 mm from the board's sides, between the two gaps -----------------------------
SLOT_GAP = 0.005  # thinner than a finger (9-10 mm just above the pad), so the board cannot be pinched inside the slot
SLOT_T, SLOT_H = 0.012, 0.16
SLOT_Y = (Y_RIGHT_GRIP + GAP_HALF, Y_LEFT_GRIP - GAP_HALF)  # (-0.186, 0.301)
GREY = (0.55, 0.55, 0.58)
_sx = BOARD_W / 2 + SLOT_GAP + SLOT_T / 2
_sy, _sl = (SLOT_Y[0] + SLOT_Y[1]) / 2, SLOT_Y[1] - SLOT_Y[0]
SLOT = [
    Box("slot_near", pos=(X_BOARD - _sx, _sy, TOP + SLOT_H / 2), size=(SLOT_T, _sl, SLOT_H), color=GREY),
    Box("slot_far", pos=(X_BOARD + _sx, _sy, TOP + SLOT_H / 2), size=(SLOT_T, _sl, SLOT_H), color=GREY),
]

# -- the cup: a fixed open box past the board's right end, rim above the ball's centre on a flat board -----------------
CUP_INNER, CUP_WALL, CUP_H = 0.140, 0.008, 0.075
CUP_GAP = 0.010  # between the board's right end and the cup's near wall
Y_CUP_NEAR = Y_BOARD_RIGHT - CUP_GAP  # outer face of the near wall, -0.458
Y_CUP_IN = (Y_CUP_NEAR - CUP_WALL - CUP_INNER, Y_CUP_NEAR - CUP_WALL)  # inner span (-0.606, -0.466)
CUP_XY = (X_BOARD, (Y_CUP_IN[0] + Y_CUP_IN[1]) / 2)  # (0.28, -0.536)
YELLOW = (0.92, 0.78, 0.15)
_cz, _co = TOP + CUP_H / 2, CUP_INNER + 2 * CUP_WALL
_cy_mid = Y_CUP_NEAR - _co / 2
CUP = [
    Box("cup_near", pos=(X_BOARD, Y_CUP_NEAR - CUP_WALL / 2, _cz), size=(_co, CUP_WALL, CUP_H), color=YELLOW),
    Box("cup_far", pos=(X_BOARD, Y_CUP_IN[0] - CUP_WALL / 2, _cz), size=(_co, CUP_WALL, CUP_H), color=YELLOW),
    Box("cup_side_near", pos=(X_BOARD - _co / 2 + CUP_WALL / 2, _cy_mid, _cz), size=(CUP_WALL, CUP_INNER, CUP_H),
        color=YELLOW),
    Box("cup_side_far", pos=(X_BOARD + _co / 2 - CUP_WALL / 2, _cy_mid, _cz), size=(CUP_WALL, CUP_INNER, CUP_H),
        color=YELLOW),
]

# -- the ball rails: right of the right gap up to the cup, 12 mm from the board's sides ------------------------------
RAIL_GAP = 0.012  # leaves 2 mm each side of the 100 mm ball; too narrow for a finger's 21 mm upper body
RAIL_T, RAIL_H = 0.008, 0.14
RAIL_Y = (Y_CUP_NEAR, Y_RIGHT_GRIP - GAP_HALF)  # (-0.458, -0.314)
BLUE = (0.20, 0.40, 0.80)
_rx = BOARD_W / 2 + RAIL_GAP + RAIL_T / 2
_ry, _rl = (RAIL_Y[0] + RAIL_Y[1]) / 2, RAIL_Y[1] - RAIL_Y[0]
RAILS = [
    Box("rail_near", pos=(X_BOARD - _rx, _ry, TOP + RAIL_H / 2), size=(RAIL_T, _rl, RAIL_H), color=BLUE),
    Box("rail_far", pos=(X_BOARD + _rx, _ry, TOP + RAIL_H / 2), size=(RAIL_T, _rl, RAIL_H), color=BLUE),
]

OBJECTS = [BOARD, BALL, *SLOT, *RAILS, *CUP]

# -- scoring -----------------------------------------------------------------------------------------------------------
FALLEN_Z = TOP - 0.02  # bottom more than 2 cm below the table top: off the table
CLEAR_Z = TOP + 0.010  # the board's lowest point at least 1 cm above the table: held clear of it
ON_BOARD_TOL = 0.012  # the ball's centre within 1.2 cm of where it sits on the board's top face
BALL_START_DIST = float(np.hypot(X_BOARD - CUP_XY[0], Y_BALL0 - CUP_XY[1]))  # 0.178 m


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _fallen(obj) -> bool:
    x, y = obj.pos[0], obj.pos[1]
    off_footprint = not (TABLE_X[0] <= x <= TABLE_X[1] and TABLE_Y[0] <= y <= TABLE_Y[1])
    return bool(obj.low[2] < FALLEN_Z or off_footprint)


def _in_cup(ball) -> bool:
    """The ball's centre over the cup's inner 14 x 14 cm and its bottom within 1 cm of the table (the cup's floor)."""
    dx, dy = ball.pos[0] - CUP_XY[0], ball.pos[1] - CUP_XY[1]
    half = CUP_INNER / 2
    return bool(abs(dx) <= half and abs(dy) <= half and ball.low[2] <= TOP + 0.010)


def _on_board(ball, board) -> bool:
    """The ball sits on the board's top face: in the board's frame its centre is over the board and one ball radius
    (within 1.2 cm) above the top face."""
    local = _rot(board.quat_xyzw).T @ (np.asarray(ball.pos, float) - np.asarray(board.pos, float))
    over = abs(local[0]) <= BOARD_W / 2 and abs(local[1]) <= BOARD_LEN / 2
    return bool(over and abs(local[2] - (BOARD_T / 2 + BALL_R)) <= ON_BOARD_TOL)


def _progress(ball) -> float:
    """0 at the ball's starting distance from the cup's centre (or farther), 1 at the cup's centre."""
    d = float(np.hypot(ball.pos[0] - CUP_XY[0], ball.pos[1] - CUP_XY[1]))
    return float(np.clip(1.0 - d / BALL_START_DIST, 0.0, 1.0))


def success(objects):
    """The ball rests in the cup (centre over the inner 14 x 14 cm, bottom on the floor, still) and the board is
    still on the table."""
    ball, board = objects["ball"], objects["board"]
    return _in_cup(ball) and ball.resting() and not _fallen(board)


def distance(objects):
    """Metres from the ball's centre to where it rests in the middle of the cup."""
    goal = np.array([CUP_XY[0], CUP_XY[1], TOP + BALL_R])
    return float(np.linalg.norm(np.asarray(objects["ball"].pos, float) - goal))


def score(objects):
    """1.0: the ball rests in the cup. 0.8: in the cup but still moving. 0.3 + 0.3 x progress: the ball is on the
    board and the whole board is held at least 1 cm clear of the table (progress = 1 - d / 0.178, d the ball's
    horizontal distance to the cup's centre, clipped to 0..1). 0: anything else: the ball on a board that still
    touches the table, the ball on the table or a rim, or the ball or the board off the table."""
    ball, board = objects["ball"], objects["board"]
    if _fallen(ball) or _fallen(board):
        return 0.0
    if _in_cup(ball):
        return 1.0 if ball.resting() else 0.8
    if _on_board(ball, board) and board.low[2] >= CLEAR_Z:
        return 0.3 + 0.3 * _progress(ball)
    return 0.0


# -- demo: both arms, straight-down grips turned 90 deg so the fingers close across the board's width ----------------
GRIP_YAW = np.pi / 2
GRIP_Z = TOP + BOARD_T  # TCP level with the board's top face: pad tips 3 mm above the table, 13 mm of pad on the board
APPROACH_Z = TOP + 0.20
HOVER_Z = TOP + 0.08
# board top-face heights at the two grips (the TCP rides at the top face)
LIFT_LEFT, LIFT_RIGHT = 0.116, 0.121  # lifted with the left end 5 mm low: the ball rolls back against the right hand
TILT_LEFT = 0.136  # then the left end goes up 2 cm: the right part slopes down 2.5 % and the ball rolls off the end
LIFT_S, TILT_S, HOLD_S = 6.0, 3.0, 5.0


def board_top_at(y: float, left: float, right: float) -> float:
    """Height above the table of the board's top face at y when the grips hold it at these top-face heights."""
    return right + (left - right) * (y - Y_RIGHT_GRIP) / (Y_LEFT_GRIP - Y_RIGHT_GRIP)


def _arm(y: float, lift: float, tilt: float) -> list[Waypoint]:
    p = (X_BOARD, y)
    return [
        # turn straight down well above the slot (16 cm) first: a direct move from the ready pose swept a finger
        # within 1 mm of the slot's end
        Waypoint(2.5, (*p, APPROACH_Z), FINGER_OPEN, "above the gap, turning straight down", GRIP_YAW),
        Waypoint(1.0, (*p, HOVER_Z), FINGER_OPEN, "over the gap", GRIP_YAW),
        Waypoint(2.0, (*p, GRIP_Z), FINGER_OPEN, "descend beside the board", GRIP_YAW),
        Waypoint(1.0, (*p, GRIP_Z), FINGER_CLOSED, "grip the board", GRIP_YAW),
        Waypoint(LIFT_S, (*p, TOP + lift), FINGER_CLOSED, "lift together", GRIP_YAW),
        Waypoint(1.5, (*p, TOP + lift), FINGER_CLOSED, "hold: ball parked against the right hand", GRIP_YAW),
        Waypoint(TILT_S, (*p, TOP + tilt), FINGER_CLOSED, "tilt: left end up", GRIP_YAW),
        Waypoint(HOLD_S, (*p, TOP + tilt), FINGER_CLOSED, "hold still while the ball drops and settles", GRIP_YAW),
    ]


def demo():
    """Both arms grip the board in their gaps, lift it 10 cm together with the left end 5 mm low (the ball rolls
    2.5 cm back against the right gripper), then the left arm raises its end 2 cm while the right arm holds: the
    right part of the board slopes down 2.5 % toward the cup with its end 2.5 cm above the rim, the ball rolls off
    into the cup, and both arms hold still for 5 s."""
    return {
        "left": _arm(Y_LEFT_GRIP, LIFT_LEFT, TILT_LEFT),
        "right": _arm(Y_RIGHT_GRIP, LIFT_RIGHT, LIFT_RIGHT),
    }


def one_arm_demo():
    """For the one-arm check only (not a hook): the right arm alone grips its gap and lifts 10.5 cm. Expected:
    the board pivots on its left end (the pads cannot hold its 1.74 N m moment), the ball rolls back against the
    gripper, and the score stays 0."""
    right = _arm(Y_RIGHT_GRIP, LIFT_RIGHT, LIFT_RIGHT)
    return {"right": right}
