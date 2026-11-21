"""A dog's ball and a ball launcher lie on a green outdoor mat with a low edging board along its far edge.

The ball is a 51 mm rubber dog ball. The launcher is a 0.36 m plastic fetch stick with a handle at one end and a
cup at the other. It lies cup-down with its handle towards the robot's left. The cup has two flexible lips that
reach past the middle of the ball, one on the tip side and one on the shaft side.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
import lots  # noqa: E402
import observation  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 2 s hold is timed inside score()
TOP = TABLE_TOP[SCENE]

# -- the mat and edging board (fixed) -----------------------------------------------------------------------------
MAT_X, MAT_Y = (0.10, 0.40), (-0.20, 0.40)
MAT_T = 0.006
MAT_TOP = TOP + MAT_T
BOARD_H = 0.015  # above the mat
GRASS, BOARD_BROWN, ORANGE, BLUE, LIME, DARK = (0.24, 0.48, 0.20), (0.45, 0.30, 0.16), (0.95, 0.42, 0.08), \
    (0.12, 0.42, 0.85), (0.62, 0.84, 0.16), (0.20, 0.32, 0.12)
MAT = parts.Part("lawn_mat", pos=((MAT_X[0] + MAT_X[1]) / 2, (MAT_Y[0] + MAT_Y[1]) / 2, TOP), friction=(0.9, 0.8),
                 shapes=[parts.box((MAT_X[1] - MAT_X[0], MAT_Y[1] - MAT_Y[0], MAT_T), (0, 0, MAT_T / 2), GRASS)])
BOARD = parts.Part("edging_board", pos=(MAT_X[1] + 0.009, (MAT_Y[0] + MAT_Y[1]) / 2, TOP), friction=(0.8, 0.7),
                   shapes=[parts.box((0.018, MAT_Y[1] - MAT_Y[0], MAT_T + BOARD_H), (0, 0, (MAT_T + BOARD_H) / 2),
                                     BOARD_BROWN)])

# -- the ball -------------------------------------------------------------------------------------------------------
R = 0.0254  # 2 in Chuckit! small ball
BALL_MASS = 0.045  # estimate: the maker's small 2-pack weighs 0.23 lb (104 g) with packaging
BALL_DAMPING = 4.0  # 1/s: stands in for the mat's rolling resistance (PhysX has none)
BALL_Z = MAT_TOP + R
BALL_A = (MAT_X[1] - R, 0.04, BALL_Z)  # lot A: against the edging board
BALL_B = (0.27, 0.02, BALL_Z)  # lot B: in the open
BALL = parts.Part("ball", pos=BALL_A, mass=BALL_MASS, friction=(0.8, 0.7), angular_damping=BALL_DAMPING,
                  shapes=[parts.sphere(R, (0, 0, 0), ORANGE)])

# -- the launcher (launcher frame: origin at the cup centre, +x along the shaft towards the cup, +z towards the cup's
#    back, which is up while it lies cup-down) ---------------------------------------------------------------------
RI, T = 0.0265, 0.003  # cup inner radius (1.1 mm clearance on the ball), wall
TOOTH_Z = (-0.0105, -0.0065)  # each lip ends in a tooth below the cup centre
TOOTH_IN = 0.0035  # tooth reach inside the lip wall: inner edges 23.0 mm from the cup axis
LIP_W = 0.028
LIP_K, LIP_DAMP, LIP_MASS = 2.0, 0.002, 0.012  # N m/rad (estimate), N m s/rad, kg
SHAFT_Z, SHAFT_W, SHAFT_H = 0.016, 0.014, 0.010
SHAFT_X = (-0.336, -0.036)
GRIP_X = -0.100  # where the demo holds the shaft


def _lip(name: str, sign: int) -> parts.Hinge:
    """sign +1: the tip-side lip at +x; -1: the shaft-side lip. Opening swings the tooth outwards (positive angle)."""
    hx, hz = sign * (RI + T / 2), RI
    plate_h = hz - TOOTH_Z[0]
    return parts.Hinge(name, pivot=(hx, 0.0, hz), axis=(0.0, -sign, 0.0), lower=0.0, upper=0.5, mass=LIP_MASS,
                       spring=LIP_K, damping=LIP_DAMP, shapes=(
                           parts.box((T, LIP_W, plate_h), (hx, 0, hz - plate_h / 2), LIME),
                           parts.box((TOOTH_IN + T, LIP_W, TOOTH_Z[1] - TOOTH_Z[0]),
                                     (hx - sign * TOOTH_IN / 2, 0, (TOOTH_Z[0] + TOOTH_Z[1]) / 2), LIME)))


LAUNCHER_SHAPES = [
    parts.box((2 * RI + 2 * T - 0.004, 2 * RI + 2 * T, T), (0, 0, RI + T / 2), LIME),  # cup back
    parts.box((0.040, T, RI), (0, RI + T / 2, RI / 2), LIME),  # side walls, down to the ball's middle
    parts.box((0.040, T, RI), (0, -RI - T / 2, RI / 2), LIME),
    parts.box((0.020, 0.016, 0.006), (-0.036, 0, RI + 0.003), LIME),  # bridge over the shaft-side lip's hinge
    parts.box((0.010, 0.016, RI + 0.006 - SHAFT_Z), (-0.041, 0, (RI + 0.006 + SHAFT_Z) / 2), LIME),  # neck
    parts.box((SHAFT_X[1] - SHAFT_X[0], SHAFT_W, SHAFT_H), ((SHAFT_X[0] + SHAFT_X[1]) / 2, 0, SHAFT_Z), LIME),
    parts.box((0.070, 0.026, SHAFT_Z + SHAFT_H / 2 - TOOTH_Z[0]), (-0.300, 0, (SHAFT_Z + SHAFT_H / 2 + TOOTH_Z[0]) / 2),
              BLUE),  # handle loop
]
LAUNCHER_YAW = -math.pi / 2  # shaft along world y, cup at the -y end
LAUNCHER_POS = (0.16, 0.0, MAT_TOP - TOOTH_Z[0])  # lies cup-down on its two teeth and its handle
LAUNCHER = parts.Part("ball_launcher", pos=LAUNCHER_POS, yaw=LAUNCHER_YAW, mass=0.141 - 2 * LIP_MASS,
                      friction=(0.4, 0.3), shapes=LAUNCHER_SHAPES, joints=[_lip("lip_tip", 1), _lip("lip_shaft", -1)])

OBJECTS = [MAT, BOARD, LAUNCHER, BALL]
LOTS = lots.declare(OBJECTS,
                    lots.Lot("A", place={"ball": BALL_A}),  # zero move: forces the settle and a fresh first frame
                    lots.Lot("B", place={"ball": BALL_B}))

# -- scoring --------------------------------------------------------------------------------------------------------
SEAT_Z = RI - R  # ball centre above the cup centre when it sits against the cup's back
SEAT_TOL = (0.004, 0.006, (-0.003, 0.003))  # x, y, z band around the seat, launcher frame
LIP_SHUT = 0.12  # rad: both teeth back under the ball
LIFT = 0.060  # ball bottom above the mat
STILL_DRIFT = 0.002  # m: the held ball stays within this of one spot for the whole hold
HOLD_S = 2.0
TOUCH = 0.005  # any hand surface this close to the ball counts as touching it


def _finite(*xs) -> bool:
    return all(np.isfinite(np.asarray(x, float)).all() for x in xs)


def seated(ball, launcher) -> bool:
    """The ball sits in the cup against its back, and both lips have closed back under it."""
    if not _finite(ball.pos, launcher.pos, launcher.quat_xyzw) or np.linalg.norm(launcher.quat_xyzw) < 1e-6:
        return False
    rel = parts._quat_to_matrix(launcher.quat_xyzw).T @ (np.asarray(ball.pos) - np.asarray(launcher.pos))
    lips = [launcher.joints.get(n, math.inf) for n in ("lip_tip", "lip_shaft")]
    return bool(abs(rel[0]) <= SEAT_TOL[0] + 1e-9 and abs(rel[1]) <= SEAT_TOL[1] + 1e-9
                and SEAT_Z + SEAT_TOL[2][0] - 1e-9 <= rel[2] <= SEAT_Z + SEAT_TOL[2][1] + 1e-9
                and all(math.isfinite(q) and q <= LIP_SHUT for q in lips))


def on_mat(ball) -> bool:
    x, y, z = ball.pos
    return bool(MAT_X[0] <= x <= MAT_X[1] and MAT_Y[0] <= y <= MAT_Y[1] and z >= MAT_TOP + R - 0.005)


def lifted(ball) -> bool:
    return bool(ball.pos[2] - R >= MAT_TOP + LIFT - 1e-6)


def hand_gap(clouds, ball) -> float:
    """Smallest distance from any sampled gripper or fingertip surface point to the ball's surface (m)."""
    if not clouds or any(c.ndim != 2 or c.shape[1] != 3 or not len(c) or not np.isfinite(c).all() for c in clouds):
        return -math.inf
    c = np.asarray(ball.pos, float)
    return float(min(np.linalg.norm(p - c, axis=1).min() for p in clouds) - R)


def _stamp(*states):
    return tuple(float(v) for s in states for v in np.r_[s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel])


_known = False
_invalid = False
_touched = False
_left_mat = False
_held = 0.0
_time = 0.0
_prev_ok = False
_gap = math.inf
_stamped = ()
_anchor = None  # where the ball was when the current hold began


@parts.on_reset
def _reset(sim):
    global _known, _invalid, _touched, _left_mat, _held, _time, _prev_ok, _gap, _stamped, _anchor
    _known = _invalid = _touched = _left_mat = _prev_ok = False
    _held = _time = 0.0
    _gap = math.inf
    _stamped = ()
    _anchor = None
    observation.reset()


@parts.on_step
def _step(sim):
    global _known, _invalid, _touched, _left_mat, _held, _time, _prev_ok, _gap, _stamped, _anchor
    states = sim.states()
    ball, launcher = states["ball"], states["ball_launcher"]
    dt = float(sim.t) - _time
    try:
        clouds, force = observation.hand_state(sim)
        gap = hand_gap(clouds, ball)
        _known = math.isfinite(gap)
    except (ValueError, AttributeError, KeyError, IndexError):
        gap, force, _known = -math.inf, math.nan, False
    if not _known or not 0 < dt <= 0.100001 or not _finite(ball.pos, ball.lin_vel, launcher.pos):
        _invalid = True
    _gap = min(_gap, gap)
    _touched |= gap <= TOUCH
    _left_mat |= not on_mat(ball)
    ok = not (_invalid or _touched or _left_mat) and seated(ball, launcher) and lifted(ball)
    pos = np.asarray(ball.pos, float)
    if ok and _prev_ok and _anchor is not None and np.linalg.norm(pos - _anchor) <= STILL_DRIFT + 1e-9:
        _held += dt
    else:  # a hold starts afresh here
        _held = 0.0
        _anchor = pos.copy() if ok else None
    _prev_ok = ok
    _time = float(sim.t)
    _stamped = _stamp(ball, launcher)
    if os.environ.get("TRACE_OUT"):
        rel = parts._quat_to_matrix(launcher.quat_xyzw).T @ (np.asarray(ball.pos) - np.asarray(launcher.pos))
        row = dict(t=_time, lot=lots.current().name, ball=np.round(ball.pos, 4).tolist(),
                   ball_v=round(float(np.linalg.norm(ball.lin_vel)), 4), cup=np.round(launcher.pos, 4).tolist(),
                   rel_mm=np.round(rel * 1000, 1).tolist(), lips_deg=[round(math.degrees(v), 1) for v in
                                                                     launcher.joints.values()],
                   gap_mm=round(gap * 1000, 1), min_gap_mm=round(_gap * 1000, 1), finger_N=round(float(force), 2),
                   seated=seated(ball, launcher), lifted=lifted(ball), held=round(_held, 2), touched=_touched,
                   left_mat=_left_mat, invalid=_invalid, score=score(states))
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("ball.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


def score(objects) -> float:
    """1.0: ball held up in the cup within 2 mm of one spot for 2 s; 0.5: ball seated in the cup; 0 after any
    touch or exit."""
    try:
        ball, launcher = objects["ball"], objects["ball_launcher"]
        if not _known or _invalid or _touched or _left_mat or _stamped != _stamp(ball, launcher):
            return 0.0
        if not on_mat(ball) or not seated(ball, launcher):
            return 0.0
        if lifted(ball) and _held >= HOLD_S - 1e-9:
            return 1.0
        return 0.5
    except (KeyError, AttributeError, TypeError, ValueError):
        return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def demo():
    """Open loop from the published layout: the launcher's start and the ball's visible position."""
    bx, by, _ = lots.current().pos("ball")
    lx, ly, lz = LAUNCHER_POS
    grip_y = ly - GRIP_X  # shaft runs to +y from the cup
    shaft_z = lz + SHAFT_Z
    yaw = -math.pi / 2  # fingers close along world x, across the shaft
    shut = -0.001  # finger target well past the 14 mm shaft: a firm pinch
    seat_c = BALL_Z - SEAT_Z  # cup centre with the ball seated
    hi = seat_c + 0.075
    over = seat_c + 0.045
    # The pads let the shaft pivot, so the press uses the handle end as a fulcrum on the mat: with the ball seated
    # and the handle resting, the shaft at the grip sits a third of the way from the cup centre's height down to
    # the handle's. Commanded 20 mm lower than that: the arm is compliant and must push.
    handle_c = MAT_TOP - TOOTH_Z[0]
    press_c = seat_c + (handle_c - seat_c) * (-GRIP_X) / 0.30 - 0.020

    def w(t, x, y, cup_z, f, label):
        return Waypoint(t, (x, y, cup_z + SHAFT_Z), f, label, yaw)

    return {"left": [
        Waypoint(2.0, (lx, grip_y, shaft_z + 0.10), FINGER_OPEN, "above the shaft", yaw),
        Waypoint(2.0, (lx, grip_y, shaft_z), FINGER_OPEN, "down to the shaft", yaw),
        Waypoint(1.0, (lx, grip_y, shaft_z), shut, "grip the shaft", yaw),
        w(2.0, lx, grip_y, hi, shut, "lift the launcher"),
        w(3.0, bx, by - GRIP_X, hi, shut, "cup over the ball"),
        w(1.5, bx, by - GRIP_X, over, shut, "lower the cup"),
        w(3.0, bx, by - GRIP_X, press_c, shut, "press down, handle end on the mat"),
        w(1.0, bx, by - GRIP_X, press_c, shut, "hold the press"),
        w(2.0, bx, by - GRIP_X, hi, shut, "lift"),
        w(3.5, bx, by - GRIP_X, hi, shut, "hold up"),
    ]}
