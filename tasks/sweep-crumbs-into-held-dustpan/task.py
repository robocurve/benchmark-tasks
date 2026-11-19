"""On the station table, between the arms: a hand dustpan lying with its lip towards the right, a hand brush lying on
the right, and twenty croutons scattered on the table between them. Nothing else is on the table."""

# Sweeping spilled food and debris into a dustpan is a core, many-times-a-shift duty of janitors and building cleaners
# (about 2.1 million US workers, BLS OEWS May 2024, 37-2011: "sweep ... floors") and of maids and housekeeping
# cleaners (about 0.8 million, 37-2012). Done by hand it is a two-handed job: one hand holds the pan's lip down on the
# floor and keeps the pan from sliding, the other sweeps the debris up over the lip.
#
# Physical difficulty (bimanual): the pan is light (150 g) and slides on the table under the brush's push. One arm
# must grip it by its handle and hold its lip pressed to the table while the other drives a 28 cm hand brush through
# the scattered croutons and up over the lip; a pan that slides back or tips leaves a gap at the lip that the croutons
# fall through, and a brush pushed too fast scatters them round its ends. Both full-size objects: the brush and pan are
# a standard household hand-brush-and-dustpan set.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import part  # noqa: E402
from taskbase import Trial, fake_state  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Sweep all the croutons into the dustpan with the brush, holding the dustpan steady, then put the brush "
               "down and let go of both.")

# -- the scene -------------------------------------------------------------------------------------------------------

PAN_C = (0.35, 0.36, 0.40)  # grey plastic
BRUSH_C = (0.25, 0.30, 0.55)  # blue bristle block
WOOD = (0.62, 0.45, 0.28)
CRUMB_C = (0.80, 0.62, 0.32)

# Dustpan (pan frame: origin on the table at the middle of the lip's front edge; the pan opens towards -y).
PAN_W = 0.23  # outside width (forward)
PAN_D = 0.18  # lip front to back wall
LIP_L = 0.02  # the lip: a 2 cm ramp from the table up onto the floor
FLOOR_T = 0.002
WALL_T, SIDE_H, BACK_H = 0.004, 0.040, 0.070
HANDLE_L, HANDLE_H, HANDLE_T = 0.11, 0.030, 0.012  # a flat fin standing on edge behind the back wall
PAN_MASS = 0.15
PAN_FRICTION = 0.4  # plastic on the table
PAN_POS = (0.27, 0.0, TOP)

# Hand brush (brush frame: origin on the table under the middle of the bristle block).
HEAD_L, HEAD_D, HEAD_H = 0.18, 0.045, 0.035  # bristle block: 18 cm long (forward), 4.5 cm deep
STOCK_H = 0.012  # wooden stock on top of the bristles
GRIP_L, GRIP_S = 0.10, 0.022  # handle: 10 cm long, 22 mm square, off the forward end of the stock
BRUSH_MASS = 0.12
BRUSH_POS = (0.23, -0.21, TOP)

# Croutons: twenty 1 cm cubes, 1 g each, scattered in front of the pan's lip.
CRUMB = 0.010
CRUMB_MASS = 0.001
N_CRUMBS = 20


def _crumb_xy():
    rng = np.random.default_rng(7)
    pts = []
    while len(pts) < N_CRUMBS:
        p = (round(float(rng.uniform(0.20, 0.32)), 3), round(float(rng.uniform(-0.15, -0.05)), 3))
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.018 for q in pts):
            pts.append(p)
    return pts


CRUMB_XY = _crumb_xy()
CRUMB_NAMES = [f"crouton_{i:02d}" for i in range(N_CRUMBS)]


def _pan_boxes():
    w_in = PAN_W - 2 * WALL_T
    ramp = math.hypot(LIP_L, FLOOR_T)
    pitch = math.atan2(FLOOR_T, LIP_L)
    return [
        # the lip: a thin ramp rising from the table to the floor (box turned 90 degrees, then pitched)
        ((0, LIP_L / 2, FLOOR_T / 2 + 0.0004), (ramp, w_in, 0.0008), PAN_C, (math.pi / 2, -pitch)),
        ((0, LIP_L + (PAN_D - LIP_L) / 2, FLOOR_T / 2), (w_in, PAN_D - LIP_L, FLOOR_T), PAN_C),  # floor
        ((-PAN_W / 2 + WALL_T / 2, LIP_L + (PAN_D - LIP_L) / 2, SIDE_H / 2), (WALL_T, PAN_D - LIP_L, SIDE_H), PAN_C),
        ((PAN_W / 2 - WALL_T / 2, LIP_L + (PAN_D - LIP_L) / 2, SIDE_H / 2), (WALL_T, PAN_D - LIP_L, SIDE_H), PAN_C),
        ((0, PAN_D + WALL_T / 2, BACK_H / 2), (PAN_W, WALL_T, BACK_H), PAN_C),  # back wall
        ((0, PAN_D + WALL_T + HANDLE_L / 2, BACK_H - HANDLE_H / 2), (HANDLE_T, HANDLE_L, HANDLE_H), PAN_C),  # handle
    ]


PAN = part("dustpan", pos=PAN_POS, mass=PAN_MASS, color=PAN_C, friction=PAN_FRICTION, boxes=_pan_boxes())
BRUSH = part("brush", pos=BRUSH_POS, mass=BRUSH_MASS, color=BRUSH_C, friction=0.6, boxes=[
    ((0, 0, HEAD_H / 2), (HEAD_L, HEAD_D, HEAD_H), BRUSH_C),  # bristles
    ((0, 0, HEAD_H + STOCK_H / 2), (HEAD_L, HEAD_D - 0.006, STOCK_H), WOOD),  # stock
    ((HEAD_L / 2 + GRIP_L / 2, 0, HEAD_H + STOCK_H / 2), (GRIP_L, GRIP_S, GRIP_S), WOOD),  # handle
])
CRUMBS = [part(n, pos=(x, y, TOP + CRUMB / 2), mass=CRUMB_MASS, color=CRUMB_C, friction=0.6,
               boxes=[((0, 0, 0), (CRUMB, CRUMB, CRUMB), CRUMB_C)]) for n, (x, y) in zip(CRUMB_NAMES, CRUMB_XY)]
OBJECTS = [PAN, BRUSH, *CRUMBS]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A crouton is in the pan when its centre, in the pan's own frame, lies over the pan's floor (between the side walls,
# between the top of the lip and the back wall) and no more than 2.5 cm above the floor. Score at the end of the trial:
# 0.6 x (croutons in the pan) / 20 (0 to 0.6). 1.0 when at least SUCCESS_N of the 20 are in the pan, the pan stands
# upright (tilted under 10 degrees), the pan, the brush and every crouton have stopped moving (under 5 mm over the last
# 0.3 s), and no gripper tool point is within 5 cm of the pan's handle or the brush's handle (both hands off), held for
# 1 s. Completed less than 1 s before the run limit: the partial score.

SUCCESS_N = 18
IN_PAN_ABOVE = 0.025
UPRIGHT_DEG = 10.0
HANDS_OFF = 0.05
PAN_GRIP = (0, PAN_D + WALL_T + HANDLE_L * 0.6, BACK_H - HANDLE_H / 2)  # where the left hand holds the handle (pan frame)
BRUSH_GRIP = (HEAD_L / 2 + GRIP_L * 0.6, 0, HEAD_H + STOCK_H / 2)  # where the right hand holds the handle (brush frame)


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _local(state, p):
    return _rot(state.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(state.pos, float))


def _world(state, p):
    return np.asarray(state.pos, float) + _rot(state.quat_xyzw) @ np.asarray(p, float)


def tilt_deg(state) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 2])))))


def in_pan(pan, crumb) -> bool:
    x, y, z = _local(pan, crumb.pos)
    w_in = PAN_W / 2 - WALL_T
    return abs(x) <= w_in and LIP_L <= y <= PAN_D and FLOOR_T - 0.002 <= z <= FLOOR_T + IN_PAN_ABOVE


def evaluate(objects, trial):
    pan, brush = objects["dustpan"], objects["brush"]
    n_in = sum(in_pan(pan, objects[n]) for n in CRUMB_NAMES)
    trial.info["in_pan"] = n_in
    upright = tilt_deg(pan) < UPRIGHT_DEG
    done = (n_in >= SUCCESS_N and upright
            and all(trial.still(n) for n in ["dustpan", "brush", *CRUMB_NAMES])
            and trial.clear_of([_world(pan, PAN_GRIP), _world(brush, BRUSH_GRIP)], HANDS_OFF))
    return 0.6 * n_in / N_CRUMBS, done


TRIAL = Trial(evaluate, trace_env="DUSTPAN_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    pan = objects["dustpan"]
    mouth = _world(pan, (0, LIP_L + 0.03, 0))
    return float(sum(0.0 if in_pan(pan, objects[n]) else np.linalg.norm(np.asarray(objects[n].pos)[:2] - mouth[:2])
                     for n in CRUMB_NAMES))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. The left arm grips the pan's handle fin (pads closing across the 12 mm fin) and presses the pan
# down onto the table, holding it there throughout. The right arm grips the brush by its handle, lifts it, sets the
# bristles down behind the croutons and sweeps them forward over the lip onto the pan's floor, twice; then it sets the
# brush down where it found it and lets go, and the left arm lets go of the pan.

YAW_L = -math.pi / 2  # left pads close along x: across the handle fin
YAW_R = 0.0  # right pads close along y: across the brush handle
PRESS = 0.003  # commanded below contact, so the hands press the pan and the brush onto the table
SWEEP_Y0, SWEEP_Y1 = BRUSH_POS[1], PAN_POS[1] + LIP_L + 0.015  # brush centre: behind the croutons, to the bristles 3.75 cm past the lip


def _pan_grip_world():
    return np.asarray(PAN_POS) + np.asarray(PAN_GRIP)


def _brush_grip(y, z_up=0.0):
    return (BRUSH_POS[0] + BRUSH_GRIP[0], y, TOP + BRUSH_GRIP[2] + z_up)


def _sweep(label):
    g = _brush_grip
    floor_up = FLOOR_T + 0.002  # over the pan the bristles skim 2 mm above its floor
    return [Waypoint(1.0, g(SWEEP_Y0, 0.03), FINGER_CLOSED, f"brush down behind the croutons ({label})", yaw=YAW_R),
            Waypoint(0.6, g(SWEEP_Y0, -PRESS), FINGER_CLOSED, "bristles on the table", yaw=YAW_R),
            Waypoint(2.6, g(PAN_POS[1] - HEAD_D / 2 - 0.003, -PRESS), FINGER_CLOSED, "sweep up to the lip", yaw=YAW_R),
            Waypoint(0.9, g(SWEEP_Y1, floor_up), FINGER_CLOSED, "up the lip onto the floor", yaw=YAW_R),
            Waypoint(0.6, g(SWEEP_Y1 - 0.04, 0.03), FINGER_CLOSED, "lift the brush back off the pan", yaw=YAW_R)]


def demo():
    hp = _pan_grip_world()
    left = [Waypoint(1.4, tuple(hp + (0, 0, 0.05)), FINGER_OPEN * 0.5, "over the pan's handle", yaw=YAW_L),
            Waypoint(0.8, tuple(hp), FINGER_OPEN * 0.5, "around the handle fin", yaw=YAW_L),
            Waypoint(0.6, tuple(hp), FINGER_CLOSED, "grip the handle", yaw=YAW_L),
            Waypoint(0.4, tuple(hp - (0, 0, PRESS)), FINGER_CLOSED, "press the pan down", yaw=YAW_L)]
    g = _brush_grip
    right = [Waypoint(1.4 + 1.4, g(BRUSH_POS[1], 0.05), FINGER_OPEN * 0.5, "over the brush handle", yaw=YAW_R),
             Waypoint(0.8, g(BRUSH_POS[1]), FINGER_OPEN * 0.5, "around the handle", yaw=YAW_R),
             Waypoint(0.6, g(BRUSH_POS[1]), FINGER_CLOSED, "grip the brush", yaw=YAW_R)]
    right += _sweep("first pass") + _sweep("second pass")
    right += [Waypoint(1.4, g(BRUSH_POS[1], 0.03), FINGER_CLOSED, "carry the brush back", yaw=YAW_R),
              Waypoint(0.6, g(BRUSH_POS[1], 0.002), FINGER_CLOSED, "set it down", yaw=YAW_R),
              Waypoint(0.5, g(BRUSH_POS[1], 0.002), FINGER_OPEN * 0.5, "let go of the brush", yaw=YAW_R),
              Waypoint(0.7, g(BRUSH_POS[1], 0.08), FINGER_OPEN * 0.5, "lift clear", yaw=YAW_R),
              Waypoint(1.0, (0.12, -0.27, TOP + 0.16), FINGER_OPEN * 0.5, "clear", yaw=YAW_R)]
    t_right = sum(w.duration for w in right)
    t_left = sum(w.duration for w in left)
    left += [Waypoint(t_right - t_left, tuple(hp - (0, 0, PRESS)), FINGER_CLOSED, "hold the pan", yaw=YAW_L),
             Waypoint(0.5, tuple(hp), FINGER_OPEN * 0.5, "let go of the pan", yaw=YAW_L),
             Waypoint(0.7, tuple(hp + (0, 0, 0.07)), FINGER_OPEN * 0.5, "lift clear", yaw=YAW_L),
             Waypoint(1.0, (0.12, 0.27, TOP + 0.16), FINGER_OPEN * 0.5, "clear", yaw=YAW_L),
             Waypoint(3.0, (0.12, 0.27, TOP + 0.16), FINGER_OPEN * 0.5, "wait", yaw=YAW_L)]
    right += [Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right), (0.12, -0.27, TOP + 0.16),
                       FINGER_OPEN * 0.5, "wait", yaw=YAW_R)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    half = (CRUMB / 2,) * 3

    def pan_xy(i):  # a spot on the pan floor for crouton i
        return (PAN_POS[0] - 0.08 + 0.04 * (i % 5), PAN_POS[1] + LIP_L + 0.02 + 0.03 * (i // 5), TOP + FLOOR_T + CRUMB / 2)

    def st(n_in, pan_quat=(0, 0, 0, 1.0), crumb_dz=0.0):
        d = {"dustpan": fake_state(PAN_POS, (PAN_W / 2, PAN_D / 2, BACK_H / 2), quat=pan_quat),
             "brush": fake_state(BRUSH_POS, (HEAD_L / 2, HEAD_D / 2, HEAD_H / 2))}
        for i, n in enumerate(CRUMB_NAMES):
            p = pan_xy(i) if i < n_in else (CRUMB_XY[i][0], CRUMB_XY[i][1], TOP + CRUMB / 2)
            d[n] = fake_state((p[0], p[1], p[2] + (crumb_dz if i < n_in else 0.0)), half)
        return d
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    holding = lambda: [_pan_grip_world(), np.array((0.1, -0.3, 1.0))]
    s = math.sin(math.radians(15) / 2)
    tipped = (s, 0.0, 0.0, math.cos(math.radians(15) / 2))
    return [
        ("all croutons on the table", [st(0)] * 30, far, 0.0, False),
        ("10 croutons in the pan", [st(10)] * 30, far, 0.3, False),
        ("all 20 in, hands off, held 1 s", [st(20)] * 30, far, 1.0, True),
        ("18 in, hands off, held 1 s", [st(18)] * 30, far, 1.0, True),
        ("17 in", [st(17)] * 30, far, 0.51, False),
        ("all in but still holding the pan", [st(20)] * 30, holding, 0.6, False),
        ("all in, 0.75 s before the run limit", [st(20)] * 15, far, 0.6, True),
        ("pan tipped 15 degrees", [st(20, pan_quat=tipped)] * 30, far, None, False),
        ("croutons perched 3 cm above the floor", [st(20, crumb_dz=0.03)] * 30, far, 0.0, False),
    ]
