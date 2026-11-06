"""On the station table, between the two arms: one loose flat wooden board, painted blue, lying on the table.
It is 24.0 x 14.0 x 1.9 cm (a length of 1x6 pine shelving board), about 0.32 kg. Its centre is 0.30 m in front of
the arm bases on the midline, and its long side runs forward, away from the arms (along x). Nothing else is on
the table.

The board is wider than the open gripper in both directions (24 and 14 cm; the fingers open to 9.4 cm), so it
cannot be picked up from above, and while it lies flat on the table its edges cannot be pinched from the side.
It can be pushed: it slides on the table.

Goal: the board lies flat with its long side across the table (along y, facing the arms), its centre still
where it started.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \\
           --instruction "Turn the board a quarter turn so its long side faces you, keeping it where it is." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# "end": the final state decides. With "best" a board that passed through the goal and was then pushed on (or off
# the table) would keep its best score.
SCORE_OVER = "end"

REST_GAP = 0.001  # the board spawns this far above the table and settles onto it
TABLE_X = (-0.19, 0.91)  # the station table's footprint (station.TABLE_CENTER +- TABLE_SIZE / 2)
TABLE_Y = (-0.65, 0.65)

# -- the board: 1x6 pine shelving (actual 19 x 140 mm) cut to 240 mm, lying flat, long side along x ---------------
BOARD_L, BOARD_W, BOARD_T = 0.240, 0.140, 0.019  # body x (long side), body y, thickness
BOARD_MASS = 0.32  # kg: pine at about 500 kg/m^3
START_XY = (0.30, 0.0)
START_YAW = 0.0  # long side along world x
GOAL_YAW = math.pi / 2  # long side along world y (either way round: the board looks the same turned 180 deg)
BOARD = Box(
    "board",
    pos=(START_XY[0], START_XY[1], TOP + REST_GAP + BOARD_T / 2),
    size=(BOARD_L, BOARD_W, BOARD_T),
    mass=BOARD_MASS,
    color=(0.20, 0.40, 0.80),
    yaw=START_YAW,
)
OBJECTS = [BOARD]

# -- scoring -------------------------------------------------------------------------------------------------------
# Full: long side within 5 deg of the y axis, centre within 20 mm of where it started.
# 0.5: within 15 deg and 40 mm. 0.2: within 15 deg, centre 40-100 mm away. 0: anything else.
FULL = dict(yaw_deg=5.0, drift=0.020)
HALF = dict(yaw_deg=15.0, drift=0.040)
TURNED_ONLY = dict(yaw_deg=15.0, drift=0.100)
FLAT_DEG = 5.0  # "lying flat": the board's faces within 5 deg of horizontal ...
ON_TABLE = 0.005  # ... and its bottom within 5 mm of the table top (not propped on a finger, not held up)
FALLEN_Z = TOP - 0.02  # bottom more than 2 cm below the table top: off the table
HALF_DIAG = math.hypot(BOARD_L, BOARD_W) / 2


def _rotation(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def tilt_deg(state) -> float:
    """Angle between the board's face normal and the vertical, 0..90 deg (upside down counts as flat)."""
    return math.degrees(math.acos(min(1.0, abs(float(_rotation(state)[2, 2])))))


def yaw_error_deg(state) -> float:
    """Angle between the board's long side and the y axis in the table plane, 0..90 deg. The board looks the same
    turned 180 deg, so the angle is taken mod 180: +90 and -90 from the start are both a quarter turn."""
    a = _rotation(state)[:2, 0]  # the long side (body x) projected onto the table
    norm = float(np.linalg.norm(a))
    if norm < 1e-9:  # standing on its end: no horizontal long side (lying_flat fails anyway)
        return 90.0
    return math.degrees(math.acos(min(1.0, abs(float(a[1]) / norm))))


def drift(state) -> float:
    """Metres between the board's centre now and at the start, in the table plane."""
    return float(np.linalg.norm(np.asarray(state.pos[:2], dtype=float) - np.asarray(START_XY)))


def off_table(state) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    over = TABLE_X[0] <= x <= TABLE_X[1] and TABLE_Y[0] <= y <= TABLE_Y[1]
    return bool(not over or state.low[2] < FALLEN_Z)


def lying_flat(state) -> bool:
    return bool(tilt_deg(state) <= FLAT_DEG and abs(float(state.low[2]) - TOP) <= ON_TABLE)


def score(objects):
    """1.0 / 0.5 / 0.2 / 0 by yaw error (mod 180) and centre drift; 0 unless the board lies flat on the table."""
    s = objects["board"]
    if off_table(s) or not lying_flat(s):
        return 0.0
    e, d = yaw_error_deg(s), drift(s)
    for tier, value in ((FULL, 1.0), (HALF, 0.5), (TURNED_ONLY, 0.2)):
        if e <= tier["yaw_deg"] and d <= tier["drift"]:
            return value
    return 0.0


def success(objects):
    """Full score and the board still (under 1 cm/s); must hold for 1 s."""
    return bool(score(objects) == 1.0 and objects["board"].resting())


def distance(objects):
    """Metres: the centre's drift plus how far a corner still has to swing (yaw error x half-diagonal)."""
    s = objects["board"]
    return float(drift(s) + math.radians(yaw_error_deg(s)) * HALF_DIAG)


# -- demo ----------------------------------------------------------------------------------------------------------
# Closed fingers push the board's edges; it is never grasped. Both arms push at once on opposite sides of the
# centre in opposite directions (a force couple), so the board turns about its centre with no net push.
#   Phase A (0 -> 45 deg): the right hand pushes the right long edge forward of the centre toward +y; the left hand
#     pushes the left long edge behind the centre toward -y.
#   Phase B (33 -> 92 deg): the right hand pushes the near (-x) end, right of the centre, along the board; the left
#     hand pushes the far (+x) end, left of the centre, the other way. It is planned from 33 deg, 12 deg before
#     phase A ends: the soft wrists make the board lag the plan, and a hand set down where a lagging board's edge
#     would be lands on the table, not on the board. If the board did reach 45 deg, the hand meets it in push 1.
#   Squaring: both hands, side by side, push the board's near long edge (now facing the arms) 8 mm forward. Two
#     contacts on one edge advancing together line that edge up with them, which takes out whatever yaw error the
#     turning left; the board then ends up at most 8 mm beyond where it started.
# Every push is a short smoothstep move (each 15 deg of turn or less); the board stops between them. Peak hand speed is
# at most 4.5 cm/s and peak acceleration at most 0.02 m/s^2 while touching the board.

PUSH_Z = TOP + 0.019  # TCP height while pushing: pad tips 6 mm above the table, 13 mm of finger against the edge
TRAVEL_Z = PUSH_Z + 0.030  # between pushes: rise 3 cm (pad tips 17 mm over the board's top) before moving sideways
HOVER_Z = TOP + 0.100  # start and end height
FINGER_HALF = 0.0064  # closed fingers: 12.8 mm across the closing direction at TCP height (URDF collision meshes)
CLEAR = 0.020  # each approach puts the finger 20 mm off the edge it will push
CLEAR_SQUARE = 0.035  # 35 mm for the squaring push: a board left 14 deg off brings its near edge's ends closer
BACK_OFF_S = 1.5  # backing away from an edge: 16 mm in 1.5 s, peak 1.6 cm/s
PRESS = 0.004  # commanded this far into the edge, against the soft wrist's lag
BACK_OFF = 0.012  # after a push, back away to this far off the edge before rising (beyond the wrist lag)
V_MAX, A_MAX = 0.045, 0.020  # contact moves: peak speed (m/s) and peak acceleration (m/s^2) of the smoothstep
TRAVEL_S = 2.5  # free moves between pushes
FINAL_HOLD_S = 2.0

PHASE_A = dict(theta=(0.0, 45.0), steps=3)
PHASE_B = dict(theta=(33.0, 92.0), steps=4)
# Contact points in the board's frame (body x along the long side) and the push direction there (into the board).
A_LEVER_R, A_LEVER_L = 0.060, 0.070  # phase A: how far along the long edge from the centre each hand pushes
B_LEVER_R, B_LEVER_L = 0.045, 0.045  # phase B: how far across the end from the centre each hand pushes
CONTACTS = {
    "A": {
        "right": ((A_LEVER_R, -BOARD_W / 2), (0.0, 1.0)),
        "left": ((-A_LEVER_L, BOARD_W / 2), (0.0, -1.0)),
    },
    "B": {
        "right": ((-BOARD_L / 2, -B_LEVER_R), (1.0, 0.0)),
        "left": ((BOARD_L / 2, B_LEVER_L), (-1.0, 0.0)),
    },
}
SQUARE_Y = {"left": 0.060, "right": -0.060}  # squaring: the two hands this far either side of the midline
SQUARE_OVER = 0.008  # the near edge is pushed to 8 mm beyond where it starts (a turned board's near edge)
END_XY = {"left": (0.22, 0.20), "right": (0.22, -0.20)}  # where each hand waits at the end, clear of the board


def _rot2(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def board_point(p_board, theta_deg: float) -> np.ndarray:
    """A point given in the board's frame, in the world, with the board turned theta about its starting centre."""
    return np.asarray(START_XY) + _rot2(theta_deg) @ np.asarray(p_board, dtype=float)


def finger_xy(phase: str, side: str, theta_deg: float, gap: float) -> np.ndarray:
    """The finger centre for a contact, ``gap`` m off the edge (negative: pressed into it)."""
    p, d = (np.asarray(v, dtype=float) for v in CONTACTS[phase][side])
    return board_point(p - d * (FINGER_HALF + gap), theta_deg)


def finger_yaw(phase: str, side: str, theta_deg: float) -> float:
    """Grip yaw (rad) that turns the fingers' closing direction onto the push direction, so the flat outer face of
    one finger meets the edge."""
    d = np.asarray(CONTACTS[phase][side][1], dtype=float)
    push_deg = math.degrees(math.atan2(d[1], d[0])) + theta_deg  # push direction in the world
    yaw = push_deg - 90.0  # the closing direction at yaw psi is (-sin psi, cos psi), i.e. at angle psi + 90
    return math.radians((yaw + 90.0) % 180.0 - 90.0)  # the closing direction is a line: keep yaw in [-90, 90)


def contact_duration(dist: float) -> float:
    """Shortest smoothstep duration (s, rounded up to 0.5 s) keeping peak speed and acceleration in bounds."""
    t = max(1.5 * dist / V_MAX, math.sqrt(6.0 * dist / A_MAX), 1.0)
    return math.ceil(t * 2.0) / 2.0


def _phase(phase: str, cfg: dict) -> dict[str, list[Waypoint]]:
    """Both hands: travel to the approach points, descend, close in, then push in short steps, back off, rise."""
    t0, t1 = cfg["theta"]
    thetas = np.linspace(t0, t1, cfg["steps"] + 1)
    out = {}
    for side in ("left", "right"):
        a = finger_xy(phase, side, t0, CLEAR)
        out[side] = [
            Waypoint(TRAVEL_S, (*a, TRAVEL_Z), FINGER_CLOSED, f"{phase}: over the approach point", finger_yaw(phase, side, t0)),
            Waypoint(1.5, (*a, PUSH_Z), FINGER_CLOSED, f"{phase}: down beside the edge", finger_yaw(phase, side, t0)),
        ]
    # every arm's segment k lasts as long as the slower of the two needs
    seq = [("close in", t0, CLEAR, t0, -PRESS)] + [
        (f"push {k + 1}", thetas[k], -PRESS, thetas[k + 1], -PRESS) for k in range(cfg["steps"])
    ]
    for label, ta, ga, tb, gb in seq:
        ends = {s: finger_xy(phase, s, tb, gb) for s in out}
        dist = max(float(np.linalg.norm(ends[s] - finger_xy(phase, s, ta, ga))) for s in out)
        dur = contact_duration(dist)
        for s in out:
            out[s].append(Waypoint(dur, (*ends[s], PUSH_Z), FINGER_CLOSED, f"{phase}: {label}", finger_yaw(phase, s, tb)))
    for s in out:
        back = finger_xy(phase, s, t1, BACK_OFF)
        yaw = finger_yaw(phase, s, t1)
        out[s] += [
            Waypoint(BACK_OFF_S, (*back, PUSH_Z), FINGER_CLOSED, f"{phase}: back off", yaw),
            Waypoint(1.0, (*back, TRAVEL_Z), FINGER_CLOSED, f"{phase}: rise", yaw),
        ]
    return out


SQUARE_YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # closing direction along x


def square_x(gap: float) -> float:
    """TCP x for the squaring push with the finger face ``gap`` m short of the line the near edge is pushed to."""
    return START_XY[0] - BOARD_W / 2 + SQUARE_OVER - FINGER_HALF - gap


def _square() -> dict[str, list[Waypoint]]:
    """Both hands side by side on the near long edge: approach 35 mm off, then push it to the line."""
    out = {}
    start = square_x(CLEAR_SQUARE + SQUARE_OVER)  # 35 mm off where a perfectly turned board's near edge would be
    end = square_x(-PRESS)
    dur = contact_duration(end - start)
    for s, y in SQUARE_Y.items():
        yaw = SQUARE_YAW[s]
        out[s] = [
            Waypoint(TRAVEL_S, (start, y, TRAVEL_Z), FINGER_CLOSED, "square: over the near edge", yaw),
            Waypoint(1.5, (start, y, PUSH_Z), FINGER_CLOSED, "square: down in front of the edge", yaw),
            Waypoint(dur, (end, y, PUSH_Z), FINGER_CLOSED, "square: push the near edge to the line", yaw),
            Waypoint(BACK_OFF_S, (square_x(BACK_OFF), y, PUSH_Z), FINGER_CLOSED, "square: back off", yaw),
            Waypoint(1.0, (square_x(BACK_OFF), y, TRAVEL_Z), FINGER_CLOSED, "square: rise", yaw),
            Waypoint(TRAVEL_S, (*END_XY[s], HOVER_Z), FINGER_CLOSED, "clear of the board", 0.0),
            Waypoint(FINAL_HOLD_S, (*END_XY[s], HOVER_Z), FINGER_CLOSED, "hold still", 0.0),
        ]
    return out


SIDE_Y = {"left": 0.20, "right": -0.20}  # between phases A and B the hands swap ends; they pass out at the sides


def _make_way(a: dict, b: dict) -> dict[str, list[Waypoint]]:
    """From the end of phase A out to each hand's own side, then along it to level with phase B's approach point.
    Moving straight there (GPU round 3) locked the two wrists together over the board for about 8 s."""
    out = {}
    for s in a:
        yaw = b[s][0].yaw
        out[s] = [
            Waypoint(1.5, (a[s][-1].pos[0], SIDE_Y[s], TRAVEL_Z), FINGER_CLOSED, "make way: out to the side", yaw),
            Waypoint(2.0, (b[s][0].pos[0], SIDE_Y[s], TRAVEL_Z), FINGER_CLOSED, "make way: along the side", yaw),
        ]
    return out


def demo():
    """Both arms, in step: phase A (turn 0 -> 45 deg), the hands pass each other out at the sides, phase B
    (33 -> 92 deg), then square the near edge."""
    plan = {"left": [], "right": []}
    a, b = _phase("A", PHASE_A), _phase("B", PHASE_B)
    for part in (a, _make_way(a, b), b, _square()):
        for s in plan:
            plan[s] += part[s]
    return plan
