"""On the station table: a tank's camlock outlet adapter standing between the arms, and on the right the delivery
hose's female camlock coupler parked on its dummy adapter, both cam arms raised open. Nothing else is on the table."""

# Coupling the delivery hose to a tank outlet with a camlock (cam-and-groove) fitting is a core operation for tanker
# drivers among heavy and tractor-trailer truck drivers (about 2.0 million US workers, BLS OEWS May 2023, 53-3032):
# lift the coupler off its dummy, seat it fully over the adapter, then swing both cam arms down. Arms closed on a
# coupler that is not seated leave the joint leaking.
#
# Physical difficulty (bimanual): the coupler's bore leaves 6 mm of play around the adapter and must be lowered
# straight over it until it seats on the collar; its two cam arms then have to be pushed down from about 10 degrees
# above horizontal through a 100-degree arc, one gripper on each side. Closed arms hang below the coupler,
# so an arm closed before seating stops the coupler going on.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, part  # noqa: E402
from taskbase import Trial, arc, fake_state, joint_point  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Couple the hose to the tank outlet: seat the coupler fully on the adapter, then close both cam arms."

# -- the scene -------------------------------------------------------------------------------------------------------

ALU = (0.78, 0.79, 0.80)
STEEL = (0.40, 0.41, 0.43)
HOSE = (0.12, 0.12, 0.13)
ARM_C = (0.85, 0.65, 0.15)

ADAPTER_R = 0.018  # male adapter, 36 mm
COLLAR_R, COLLAR_TOP = 0.032, 0.03
ADAPTER_TOP = 0.057
BORE = 0.048  # coupler bore: 6 mm of play on each side
WALL, SLEEVE_H, CAP_T = 0.006, 0.03, 0.004
STUB_R, STUB_H = 0.015, 0.04  # the hose tail's stub on top, where it is gripped
ARM_L, ARM_Y = 0.045, BORE / 2 + WALL + 0.015  # lugs stand 1.5 cm out: closed arms clear the collar
ARM_OPEN = math.radians(100)  # from hanging straight down: 10 degrees above horizontal, sticking out
OUTLET_XY = (0.27, 0.0)
PARK_XY = (0.15, -0.24)
MASS = 0.25


def _post(name, xy):
    return part(name, pos=(*xy, TOP), fixed=True, boxes=[((0, 0, 0.003), (0.07, 0.07, 0.006), STEEL)],
                cylinders=[((0, 0, ADAPTER_TOP / 2), ADAPTER_R, ADAPTER_TOP, ALU, "Z"),
                           ((0, 0, COLLAR_TOP - 0.003), COLLAR_R, 0.006, ALU, "Z")])


def _arm(side):
    s = 1 if side == "left" else -1
    lo, hi = (-0.02, ARM_OPEN + 0.05) if s > 0 else (-ARM_OPEN - 0.05, 0.02)
    init = s * ARM_OPEN
    return Joint(f"arm_{side}", kind="hinge", axis="X", anchor=(0, s * ARM_Y, SLEEVE_H), lower=lo, upper=hi,
                 initial=init, target=init, friction=0.03, damping=0.01, mass=0.02,
                 boxes=[((0, s * ARM_Y, SLEEVE_H - ARM_L / 2), (0.012, 0.006, ARM_L), ARM_C),
                        ((0, s * (ARM_Y + 0.004), SLEEVE_H - ARM_L + 0.008), (0.014, 0.012, 0.016), ARM_C)])


def _coupler():
    h = BORE / 2 + WALL / 2
    z = SLEEVE_H / 2
    outer = BORE + 2 * WALL
    return part("coupler", pos=(*PARK_XY, TOP + COLLAR_TOP), mass=MASS, color=ALU, joints=[_arm("left"), _arm("right")],
                boxes=[((h, 0, z), (WALL, outer, SLEEVE_H), ALU), ((-h, 0, z), (WALL, outer, SLEEVE_H), ALU),
                       ((0, h, z), (BORE, WALL, SLEEVE_H), ALU), ((0, -h, z), (BORE, WALL, SLEEVE_H), ALU),
                       ((0, 0, SLEEVE_H + CAP_T / 2), (outer, outer, CAP_T), ALU)],
                cylinders=[((0, 0, SLEEVE_H + CAP_T + STUB_H / 2), STUB_R, STUB_H, HOSE, "Z")])


OUTLET = _post("tank_outlet", OUTLET_XY)
DUMMY = _post("dummy_adapter", PARK_XY)
COUPLER = _coupler()
OBJECTS = [OUTLET, DUMMY, COUPLER]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# The coupler is seated when its axis is within 10 mm of the tank adapter's (seen from above; the bore's play is 6 mm and a coupler
# off the adapter cannot get down to the collar), it rests on the collar
# (within 4 mm of seated height) and stands upright (within 5 degrees). An arm is closed when it hangs within 10 degrees
# of straight down. Score at the end of the trial: 0.3 for the coupler seated and still, plus 0.15 per arm closed while
# it is seated (0 to 0.6). 1.0 when the coupler is seated with both arms closed, nothing moves and no gripper tool point
# is within 4 cm of the hose stub or an arm, held for 1 s. Completed less than 1 s before the run limit: 0.6.
# Defect that scores 0: the coupler falling off the table.

SEAT_Z = TOP + COLLAR_TOP


def _tilt(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def seated(s) -> bool:
    p = np.asarray(s.pos, float)
    return (math.hypot(p[0] - OUTLET_XY[0], p[1] - OUTLET_XY[1]) < 0.010 and abs(p[2] - SEAT_Z) < 0.004
            and _tilt(s.quat_xyzw) < 5)


def evaluate(objects, trial):
    c = objects["coupler"]
    if float(c.low[2]) < TOP - 0.03:
        trial.defect("coupler fell off the table")
    sit = seated(c) and trial.still("coupler")
    closed = [abs(c.joints.get(f"arm_{x}", ARM_OPEN)) < math.radians(10) for x in ("left", "right")]
    trial.info.update(seated=sit, arms=[round(c.joints.get(f"arm_{x}", 0.0), 3) for x in ("left", "right")])
    pts = [np.asarray(c.pos) + np.array((0, 0, SLEEVE_H + CAP_T + STUB_H / 2))]
    pts += [np.asarray(c.pos) + np.array((0, k * ARM_Y, SLEEVE_H - ARM_L / 2)) for k in (1, -1)]
    done = sit and all(closed) and trial.clear_of(pts, 0.04)
    return 0.3 * sit + (0.15 * sum(closed) if sit else 0.0), done


TRIAL = Trial(evaluate, trace_env="CAMLOCK_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    c = objects["coupler"]
    p = np.asarray(c.pos, float)
    return float(np.linalg.norm(p - np.array((*OUTLET_XY, SEAT_Z)))
                 + 0.02 * sum(abs(c.joints.get(f"arm_{x}", ARM_OPEN)) for x in ("left", "right")))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. The right arm grips the hose stub (pads closing along the forward axis, across the 30 mm stub),
# lifts the coupler straight off its dummy, carries it with its sleeve 3 cm above the tank adapter, lowers it straight
# down until it is 3 mm above the collar and lets go. Then each arm, fingers closed, pushes the cam arm on its side from
# outside, finger tips trailing 1.3 cm behind a point 3.5 cm out along it, down through 100 degrees until it hangs
# straight, and lifts clear: the left arm first, then the right, so the grippers never meet over the coupler.

YAW_L, YAW_R = -math.pi / 2, math.pi / 2
GRIP_UP = SLEEVE_H + CAP_T + STUB_H / 2  # tool point above the coupler's bottom when gripping the stub
N = 10


def _close_arm(side, yaw):
    k = 1 if side == "left" else -1
    anchor = (0, k * ARM_Y, SLEEVE_H)
    p0 = (0, k * (ARM_Y + 0.003), SLEEVE_H - 0.035)  # 3.5 cm out along the arm, its outer face (hanging)
    path = arc((*OUTLET_XY, SEAT_Z), anchor, "X", p0, k * ARM_OPEN, -k * 0.2, N, standoff=0.010, lift=0.0)
    pre = path[0] + np.array((0, k * 0.03, 0.03))
    w = [Waypoint(1.2, tuple(pre), FINGER_CLOSED, f"beside the {side} cam arm", yaw=yaw),
         Waypoint(0.6, tuple(path[0]), FINGER_CLOSED, "on the cam arm", yaw=yaw)]
    w += [Waypoint(0.22, tuple(p), FINGER_CLOSED, f"close the {side} cam arm", yaw=yaw) for p in path]
    w += [Waypoint(0.7, tuple(path[-1] + np.array((0, k * 0.04, 0.05))), FINGER_CLOSED, "lift clear", yaw=yaw)]
    return w


def demo():
    carry = TOP + ADAPTER_TOP + 0.03 + GRIP_UP
    park_g = TOP + COLLAR_TOP + GRIP_UP
    seat_g = SEAT_Z + 0.003 + GRIP_UP
    right = [Waypoint(1.5, (*PARK_XY, carry), FINGER_OPEN * 0.6, "over the parked coupler", yaw=YAW_R),
             Waypoint(1.0, (*PARK_XY, park_g), FINGER_OPEN * 0.6, "descend", yaw=YAW_R),
             Waypoint(0.7, (*PARK_XY, park_g), FINGER_CLOSED, "grip the hose stub", yaw=YAW_R),
             Waypoint(1.2, (*PARK_XY, carry), FINGER_CLOSED, "lift it off the dummy", yaw=YAW_R),
             Waypoint(1.8, (*OUTLET_XY, carry), FINGER_CLOSED, "over the tank adapter", yaw=YAW_R),
             Waypoint(1.4, (*OUTLET_XY, seat_g), FINGER_CLOSED, "lower it straight on", yaw=YAW_R),
             Waypoint(0.6, (*OUTLET_XY, seat_g), FINGER_OPEN * 0.6, "let go", yaw=YAW_R),
             Waypoint(0.7, (*OUTLET_XY, carry), FINGER_OPEN * 0.6, "rise", yaw=YAW_R)]
    t_seat = sum(w.duration for w in right)
    left = [Waypoint(t_seat, (0.14, 0.26, TOP + 0.16), FINGER_CLOSED, "wait for the coupler", yaw=YAW_L)]
    left += _close_arm("left", YAW_L)
    # the right arm closes its cam arm once the left gripper has lifted clear: the two never meet over the coupler
    right += [Waypoint(sum(w.duration for w in left) - t_seat, (0.14, -0.26, TOP + 0.16), FINGER_CLOSED,
                       "wait for the left arm", yaw=YAW_R)]
    right += _close_arm("right", YAW_R)
    rest_l, rest_r = (0.14, 0.26, TOP + 0.16), (0.14, -0.26, TOP + 0.16)
    left += [Waypoint(1.2, rest_l, FINGER_CLOSED, "clear", yaw=YAW_L), Waypoint(3.0, rest_l, FINGER_CLOSED, "wait", yaw=YAW_L)]
    right += [Waypoint(1.2, rest_r, FINGER_CLOSED, "clear", yaw=YAW_R), Waypoint(3.0, rest_r, FINGER_CLOSED, "wait", yaw=YAW_R)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    seat = (*OUTLET_XY, SEAT_Z)
    park = (*PARK_XY, SEAT_Z)

    def st(pos, l=ARM_OPEN, r=-ARM_OPEN, quat=(0, 0, 0, 1.0)):
        return {"coupler": fake_state(pos, (0.03, 0.03, 0.04), {"arm_left": l, "arm_right": r}, quat=quat)}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    on_stub = lambda: [np.asarray(seat) + np.array((0, 0, GRIP_UP)), np.array((0.1, -0.3, 1.0))]
    s5 = math.sin(math.radians(4))
    return [
        ("parked", [st(park)] * 30, far, 0.0, False),
        ("seated, arms open", [st(seat)] * 30, far, 0.3, False),
        ("seated, one arm closed", [st(seat, l=0.0)] * 30, far, 0.45, False),
        ("coupled, held 1 s", [st(seat, 0.0, 0.0)] * 30, far, 1.0, True),
        ("coupled 0.75 s before the run limit", [st(seat, 0.0, 0.0)] * 15, far, 0.6, True),
        ("coupled but still holding the stub", [st(seat, 0.0, 0.0)] * 30, on_stub, 0.6, False),
        ("arms closed on a coupler 14 mm off the adapter", [st((seat[0] + 0.014, seat[1], seat[2]), 0.0, 0.0)] * 30, far, 0.0, False),
        ("coupler perched 1 cm high", [st((seat[0], seat[1], seat[2] + 0.01), 0.0, 0.0)] * 30, far, 0.0, False),
        ("coupler dropped off the table", [st((0.3, 0.6, TOP - 0.3))] * 30, far, 0.0, False),
    ]
