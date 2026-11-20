"""The scripted reference demo, open loop from the fixed start: Cartesian waypoints for each arm, played by the
harness's ``yam_scripted`` policy with IK on the arm's URDF. It reads nothing from the simulator while it runs.

1. The right hand, fingers level and pointing along +y, comes in round the old pot's body and squeezes it across, so
   that its wall flexes in, then eases to a light hold that keeps the pot down.
2. The left hand, tilted 45 degrees down towards -y, then closes on the collar.
3. It lifts the cactus and its ball straight up out of the pot, carries them over the new pot and lowers them square
   onto the base layer; once the ball is clear of the rim the right hand lets the pot go.
4. The left hand opens, backs out along its own axis, and both arms go home.

TASK_DEMO=failure plays the careless variant: no squeeze, so the left hand pulls the plant out of the bound pot.
"""

from __future__ import annotations

import math

import numpy as np

import bodykit as bk
import cactusassets as ca
from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY
from yam_arena.demo import ArmPlan, Waypoint

TOP = station.TABLE_TOP
MAT_T = 0.003
OLD = np.array([0.35, -0.06])
NEW = np.array([0.35, 0.13])
S45 = math.sqrt(0.5)

# heights of the TCP above the table
SQUEEZE_Z = 0.045                                            # across the old pot's body
COLLAR_Z = MAT_T + ca.BALL_Z0 + ca.BALL_H + (ca.COLLAR_Z0 + ca.COLLAR_Z1) / 2   # the collar's middle, at the start
CARRY_Z = 0.195                                              # the ball's base 1.6 cm over the new pot's rim
SEAT_Z = MAT_T + ca.LAYER_TOP + ca.BALL_H + (ca.COLLAR_Z0 + ca.COLLAR_Z1) / 2   # the ball's base on the base layer
SEAT_PRESS = 0.002                                           # commanded this far below, against the arm's sag
# where the open-loop plan aims, allowing for the arms' steady offsets under load
SEAT_TRIM = np.array([0.0046, -0.0052, 0.0])
GRASP_TRIM = np.array([0.0, 0.0, 0.0])
CARRY_TILT = (-2.2, -2.7)  # degrees about world x and y


def rot_xy(ax_deg: float, ay_deg: float) -> np.ndarray:
    a, b = math.radians(ax_deg), math.radians(ay_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    return ry @ rx


def grip(x, z) -> np.ndarray:
    """A TCP rotation whose fingers point along ``z`` and close along ``x`` (world)."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    z = z / np.linalg.norm(z)
    x = x - np.dot(x, z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def finger_for(width: float, force: float = 0.0) -> float:
    """The finger joint target that closes the pads on something ``width`` across with about ``force`` N a finger."""
    return float(np.clip(-(width / 2 - force / GRIPPER["stiffness"]), FINGER_OPEN, 0.0))


LEVEL = grip((1, 0, 0), (0, 1, 0))          # right hand: fingers level along +y, closing along x
TILT = grip((-1, 0, 0), (0, -S45, -S45))     # left hand: 45 degrees down towards -y, closing along x
APPROACH = TILT[:, 2]

POT_W = 2 * ca.old_outer_r(SQUEEZE_Z - MAT_T + 0.016)   # the pot across, where the pads' top edges meet the tapered wall
SQUEEZE_IN = 0.015                                      # the pads close this much past the pot at rest: 7.5 mm a side
HOLD_IN = 0.004                                         # then ease to this: a light hold that keeps the pot down
SQUEEZE_F = finger_for(POT_W - SQUEEZE_IN, 25.0)
SQUEEZE_ROLL = 1.1                                      # degrees the right hand is turned back about its pointing
                                                        # axis while it squeezes: the wrist rolls this much under load
SQUEEZE_DX = 0.0036                                        # the right hand's offset along x when it squeezes
COLLAR_F = finger_for(2 * ca.COLLAR_R_OUT, 25.0)


def P(xy, z, trim=None) -> tuple:
    p = np.array([xy[0], xy[1], TOP + z]) + (0 if trim is None else trim)
    return tuple(float(v) for v in p)


def home(side: str, t: float = 2.5) -> Waypoint:
    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[side])
    return Waypoint(t, tuple(float(v) for v in pos), FINGER_OPEN, "home", rot)


def plan(mode: str = "success") -> dict:
    """The demo's waypoints. ``failure`` is the careless variant: the right hand only holds the pot, never squeezes it,
    and the left hand pulls the plant out of the bound pot."""
    bk.allow_matrix_grips()
    squeeze = mode != "failure"
    squeeze_f = finger_for(POT_W - SQUEEZE_IN, 25.0)
    hold_f = finger_for(POT_W - HOLD_IN, 5.0)
    pot_at = OLD + [SQUEEZE_DX, 0.0]
    level_sq = rot_xy(0.0, SQUEEZE_ROLL) @ LEVEL
    # the carried cactus leans under its weight (the soft wrist): the hand is turned back by this from the carry on
    lean = rot_xy(*CARRY_TILT) @ TILT
    collar_f = COLLAR_F
    grasp_trim, seat_trim, close_t = GRASP_TRIM, SEAT_TRIM, 1.0
    grasp_z = COLLAR_Z
    collar = np.array([OLD[0], OLD[1], TOP + grasp_z]) + grasp_trim
    back = -APPROACH * 0.06
    seat = np.array([NEW[0], NEW[1], TOP + SEAT_Z - SEAT_PRESS]) + seat_trim
    tp = lambda p: tuple(float(v) for v in p)  # noqa: E731

    right = [
        Waypoint(2.0, P(pot_at + [0, -0.075], SQUEEZE_Z), FINGER_OPEN, "right: beside the pot", LEVEL),
        Waypoint(1.5, P(pot_at, SQUEEZE_Z), FINGER_OPEN, "right: round the pot", level_sq),
        Waypoint(1.0, P(pot_at, SQUEEZE_Z), squeeze_f if squeeze else FINGER_OPEN, "right: squeeze", level_sq),
        Waypoint(2.5, P(pot_at, SQUEEZE_Z), squeeze_f if squeeze else FINGER_OPEN, "right: keep squeezing", level_sq),
        Waypoint(2.5, P(pot_at, SQUEEZE_Z), hold_f if squeeze else FINGER_OPEN, "right: ease to a hold", level_sq),
        Waypoint(6.0, P(pot_at, SQUEEZE_Z), hold_f if squeeze else FINGER_OPEN, "right: hold the pot", level_sq),
        Waypoint(0.6, P(pot_at, SQUEEZE_Z), FINGER_OPEN, "right: let go", level_sq),
        Waypoint(1.2, P(OLD + [0, -0.075], SQUEEZE_Z), FINGER_OPEN, "right: out", LEVEL),
        home("right"),
    ]
    left = [
        Waypoint(2.5, tp(collar + back), FINGER_OPEN, "left: over the collar", TILT),
        Waypoint(7.0, tp(collar + back), FINGER_OPEN, "left: wait: squeeze, ball freed, hold eased", TILT),
        Waypoint(1.0, tp(collar), FINGER_OPEN, "left: in", TILT),
        Waypoint(close_t, tp(collar), collar_f, "left: close on the collar", TILT),
        Waypoint(0.5, tp(collar), collar_f, "left: settle", TILT),
        Waypoint(3.0, tp(collar + [0, 0, CARRY_Z - COLLAR_Z]), collar_f, "left: lift", TILT),
        Waypoint(3.0, P(NEW, CARRY_Z, seat_trim * [1, 1, 0]), collar_f, "left: carry", lean),
        Waypoint(1.5, tp(seat + [0, 0, 0.03]), collar_f, "left: over the layer", lean),
        Waypoint(2.0, tp(seat), collar_f, "left: set down", lean),
        Waypoint(0.5, tp(seat), collar_f, "left: settle", lean),
        Waypoint(0.8, tp(seat), finger_for(2 * ca.COLLAR_R_OUT + 0.012), "left: let go", TILT),
        Waypoint(1.5, tp(seat + [0, 0, 0.05]), finger_for(2 * ca.COLLAR_R_OUT + 0.012), "left: up off the collar", TILT),
        Waypoint(1.0, tp(seat + [0, 0, 0.05] + back), FINGER_OPEN, "left: back out", TILT),
        home("left"),
    ]
    return {"left": left, "right": right}


def play(ctx, plans: dict):
    """Play the plan through demokit.py, one control step a yield, as ``yam_scripted`` does."""
    arms = {side: ArmPlan(list(w), station.BASES[side]) for side, w in plans.items()}
    end = max(p.duration for p in arms.values())
    dt = 1.0 / 20.0
    k = 0
    while k * dt <= end:
        k += 1
        for side, p in arms.items():
            ctx.arm(side).cmd = p.action7(k * dt)[0]
        yield
    for side, p in arms.items():
        print(f"[cactus] {side} IK worst {p.worst_ik_mm:.1f} mm {p.worst_ik_deg:.1f} deg", flush=True)
