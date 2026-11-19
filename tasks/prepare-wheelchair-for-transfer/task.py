"""On the station table, between the arms: a parked wheelchair facing the arms, its two swing-away legrests in place
in front of the seat and both wheel brakes off. Nothing else is on the table."""

# Before a patient moves from a wheelchair to a bed or toilet, nursing assistants (about 1.35 million US workers, BLS
# OEWS May 2023, 31-1131) lock both wheel brakes and swing both legrests out of the way, so the chair cannot roll and
# the patient's feet have a clear path. A chair with a brake off can roll away during the transfer.
#
# Physical difficulty: four separate mechanisms, two per arm: each legrest is a hinged hanger that must be swung out
# through a 90-degree arc about its vertical hinge by pushing its release handle round from the seat side; each
# brake is a lever pushed forward 30 degrees until it bears on the tyre. Both arms work at once, one per side.
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
INSTRUCTION = "Get the wheelchair ready for a transfer: swing both legrests away and put both brakes on."

# -- the scene -------------------------------------------------------------------------------------------------------

FRAME = (0.30, 0.31, 0.34)
SEAT = (0.10, 0.18, 0.40)
TYRE = (0.08, 0.08, 0.08)
RED = (0.75, 0.15, 0.12)
POS = (0.27, 0.0, TOP)  # part frame origin: centre of the chair's footprint, on the table

SEAT_FRONT = -0.07
WHEEL_X, WHEEL_Y, WHEEL_R = 0.045, 0.10, 0.06
LEG_HINGE_X, LEG_HINGE_Y = -0.085, 0.085  # each legrest swings about a vertical hinge at the seat's outer front corner
LEG_W, LEG_T, LEG_Z0, LEG_Z1 = 0.07, 0.01, 0.012, 0.07  # hanger panel: 7 cm wide, 1 cm thick, 1.2-7 cm up
BRAKE_X, BRAKE_Y, BRAKE_Z, BRAKE_L = -0.02, 0.112, 0.075, 0.05
BRAKE_ON = math.radians(25)
LEG_AWAY = math.radians(80)
POST_Y, POST_TOP = LEG_HINGE_Y - LEG_W + 0.008, 0.11  # release handle on the legrest's free end, standing above the seat
GRIP_C = (0.12, 0.12, 0.13)


def _leg(side):
    s = 1 if side == "left" else -1
    hinge = (LEG_HINGE_X, s * LEG_HINGE_Y, 0.04)
    centre = (LEG_HINGE_X, s * (LEG_HINGE_Y - LEG_W / 2), (LEG_Z0 + LEG_Z1) / 2)
    # left legrest swings its free end towards the arms for a negative angle, the right one for a positive angle
    lo, hi = (-1.85, 0.0) if s > 0 else (0.0, 1.85)
    return Joint(f"legrest_{side}", kind="hinge", axis="Z", anchor=hinge, lower=lo, upper=hi, initial=0.0, target=0.0,
                 friction=0.05, damping=0.02, mass=0.15,
                 boxes=[(centre, (LEG_T, LEG_W, LEG_Z1 - LEG_Z0), FRAME),
                        ((LEG_HINGE_X - 0.02, s * (LEG_HINGE_Y - LEG_W / 2), LEG_Z0 + 0.003), (0.04, LEG_W - 0.01, 0.006), FRAME),
                        ((LEG_HINGE_X, s * POST_Y, (LEG_Z1 + POST_TOP) / 2), (0.01, 0.01, POST_TOP - LEG_Z1), GRIP_C)])


def _brake(side):
    s = 1 if side == "left" else -1
    return Joint(f"brake_{side}", kind="hinge", axis="Y", anchor=(BRAKE_X, s * BRAKE_Y, BRAKE_Z), lower=0.0,
                 upper=BRAKE_ON + 0.08, initial=0.0, target=0.0, friction=0.08, damping=0.02, mass=0.03,
                 boxes=[((BRAKE_X, s * BRAKE_Y, BRAKE_Z + BRAKE_L / 2), (0.01, 0.01, BRAKE_L), RED),
                        ((BRAKE_X, s * BRAKE_Y, BRAKE_Z + BRAKE_L), (0.016, 0.016, 0.016), RED)])


JOINTS = [_leg("left"), _leg("right"), _brake("left"), _brake("right")]
CHAIR = part("wheelchair", pos=POS, fixed=True, joints=JOINTS, boxes=[
    ((0.0, 0, 0.07), (0.14, 0.17, 0.02), SEAT),  # seat
    ((0.07, 0, 0.11), (0.015, 0.17, 0.08), SEAT),  # back
    ((0.0, 0.088, 0.045), (0.14, 0.008, 0.05), FRAME), ((0.0, -0.088, 0.045), (0.14, 0.008, 0.05), FRAME),  # side frames
    ((-0.06, 0.06, 0.02), (0.02, 0.02, 0.04), FRAME), ((-0.06, -0.06, 0.02), (0.02, 0.02, 0.04), FRAME),  # casters
], cylinders=[((WHEEL_X, WHEEL_Y, WHEEL_R), WHEEL_R, 0.012, TYRE, "Y"), ((WHEEL_X, -WHEEL_Y, WHEEL_R), WHEEL_R, 0.012, TYRE, "Y")])
OBJECTS = [CHAIR]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A legrest is swung away when its hinge has turned at least 80 degrees out; a brake is on when its lever has been
# pushed at least 25 degrees forward (onto the tyre). Score at the end of the trial: 0.2 per legrest swung away and per
# brake on (0 to 0.8). 1.0 when all four are done, nothing moves, and no gripper tool point is within 4 cm of a brake
# lever's knob, held for 1 s. Completed less than 1 s before the run limit: 0.8.


def knob(s, side):
    k = 1 if side == "left" else -1
    return joint_point(s.pos, (BRAKE_X, k * BRAKE_Y, BRAKE_Z), "Y", s.joints.get(f"brake_{side}", 0.0),
                       (BRAKE_X, k * BRAKE_Y, BRAKE_Z + BRAKE_L))


def evaluate(objects, trial):
    s = objects["wheelchair"]
    j = s.joints
    away = [abs(j.get(f"legrest_{x}", 0.0)) >= LEG_AWAY for x in ("left", "right")]
    on = [j.get(f"brake_{x}", 0.0) >= BRAKE_ON for x in ("left", "right")]
    n = sum(away) + sum(on)
    trial.info.update(done=n)
    complete = n == 4 and trial.still("wheelchair") and trial.clear_of([knob(s, "left"), knob(s, "right")], 0.04)
    return 0.2 * n, complete


TRIAL = Trial(evaluate, trace_env="CHAIR_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    j = objects["wheelchair"].joints
    return float(sum(max(0.0, LEG_AWAY - abs(j.get(f"legrest_{x}", 0.0))) + max(0.0, BRAKE_ON - j.get(f"brake_{x}", 0.0))
                     for x in ("left", "right")))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, closed fingers. The left arm does its legrest then its brake; the right arm does its brake first
# and starts its legrest only once the left one is out of the way (the legrests meet in the middle). For a legrest the
# arm lowers its fingers behind the release handle, on the seat side and above the seat, and pushes it round 100 degrees
# about the hinge (finger tips trailing 1 cm behind it), then lifts clear. For a brake it pushes the lever's knob
# forward 32 degrees and lifts off.

N = 10
PUSH_TO = 100  # degrees: the handle is pushed 10 degrees past square; the legrest stops at its 106-degree limit


def _side(side, brake_first=False, wait=0.0):
    k = 1 if side == "left" else -1
    leg = JOINTS[0] if side == "left" else JOINTS[1]
    sgn = -k  # sign of the opening angle
    # closed fingers behind the legrest's release handle (the seat side, above the seat), pushing it round its hinge
    face_pt = (LEG_HINGE_X + 0.005, k * POST_Y, POST_TOP - 0.012)
    push = arc(POS, leg.anchor, "Z", face_pt, 0.0, sgn * math.radians(PUSH_TO), 12, standoff=0.010, lift=0.0)
    pyaw = math.pi / 2 if side == "right" else -math.pi / 2
    wps = [Waypoint(1.4, tuple(push[0] + np.array((0, 0, 0.05))), FINGER_CLOSED, f"over the {side} legrest's handle", yaw=pyaw),
           Waypoint(0.8, tuple(push[0]), FINGER_CLOSED, "behind the handle", yaw=pyaw)]
    wps += [Waypoint(0.3, tuple(p), FINGER_CLOSED, "swing the legrest away", yaw=pyaw) for p in push[1:]]
    wps += [Waypoint(0.6, tuple(push[-1] + np.array((0, 0, 0.06))), FINGER_CLOSED, "lift clear", yaw=pyaw)]
    leg_wps = wps
    b = JOINTS[2] if side == "left" else JOINTS[3]
    knob0 = (BRAKE_X, k * BRAKE_Y, BRAKE_Z + BRAKE_L - 0.004)
    path = arc(POS, b.anchor, "Y", knob0, 0.0, BRAKE_ON + 0.15 * (1 if BRAKE_ON > 0 else -1), 8, standoff=0.010, lift=0.0)
    byaw = math.pi / 2 if side == "right" else -math.pi / 2
    brk = [Waypoint(1.3, tuple(path[0] + np.array((-0.02, 0, 0.04))), FINGER_CLOSED, f"in front of the {side} brake", yaw=byaw),
           Waypoint(0.6, tuple(path[0]), FINGER_CLOSED, "behind the lever's knob", yaw=byaw)]
    brk += [Waypoint(0.2, tuple(p), FINGER_CLOSED, "push the brake on", yaw=byaw) for p in path]
    brk += [Waypoint(0.7, tuple(path[-1] + np.array((-0.03, 0, 0.05))), FINGER_CLOSED, "lift off", yaw=byaw)]
    rest = (0.12, k * 0.27, TOP + 0.16)
    if brake_first:
        park = (0.12, k * 0.2, TOP + 0.12)
        seq = brk + ([Waypoint(wait, park, FINGER_CLOSED, "wait for the other legrest", yaw=byaw)] if wait > 0 else []) + leg_wps
    else:
        seq = leg_wps + brk
    seq += [Waypoint(1.2, rest, FINGER_CLOSED, "clear", yaw=byaw), Waypoint(3.0, rest, FINGER_CLOSED, "wait", yaw=byaw)]
    return seq


def demo():
    left = _side("left")
    t_left_leg = sum(w.duration for w in left[:_LEG_STEPS])
    right0 = _side("right", brake_first=True)
    t_brake = sum(w.duration for w in right0[:_BRAKE_STEPS])
    # the right arm starts its legrest only once the left arm has finished its own (they meet in the gap between them)
    return {"left": left, "right": _side("right", brake_first=True, wait=max(0.0, t_left_leg - t_brake + 0.3))}


_LEG_STEPS = 2 + 12 + 1  # waypoints in one legrest sequence
_BRAKE_STEPS = 2 + 9 + 1  # waypoints in one brake sequence


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(ll=0.0, lr=0.0, bl=0.0, br=0.0):
        return {"wheelchair": fake_state(POS, (0.08, 0.1, 0.07),
                                         {"legrest_left": ll, "legrest_right": lr, "brake_left": bl, "brake_right": br})}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    done = dict(ll=-1.55, lr=1.55, bl=0.5, br=0.5)
    on_knob = lambda: [knob(st(**done)["wheelchair"], "left"), np.array((0.1, -0.3, 1.0))]
    return [
        ("untouched", [st()] * 30, far, 0.0, False),
        ("both legrests away", [st(ll=-1.55, lr=1.55)] * 30, far, 0.4, False),
        ("legrests away, one brake on", [st(ll=-1.55, lr=1.55, bl=0.5)] * 30, far, 0.6, False),
        ("all four, held 1 s", [st(**done)] * 30, far, 1.0, True),
        ("all four 0.75 s before the run limit", [st(**done)] * 15, far, 0.8, True),
        ("all four but a gripper on a brake knob", [st(**done)] * 30, on_knob, 0.8, False),
        ("a legrest only 60 degrees out", [st(**{**done, "ll": -1.05})] * 30, far, 0.6, False),
        ("a brake only 15 degrees on", [st(**{**done, "br": 0.26})] * 30, far, 0.6, False),
    ]
