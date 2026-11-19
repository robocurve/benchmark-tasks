"""On the station table, between the arms: a hospital bed (a table-scale training model) with both side rails down.
Each rail slides up in its frame; a sprung latch pin under it snaps out beneath the rail once the rail is lifted past
it. Nothing else is on the table."""

# Raising both side rails until they latch before leaving a patient in bed is a core safety operation for nursing
# assistants (about 1.35 million US workers, BLS OEWS May 2023, 31-1131). A rail that is lifted but not latched drops
# back down when let go.
#
# Physical difficulty: each 400 g rail must be lifted straight up 6.5 cm by its top bar, past a sprung latch pin whose
# bevelled underside it pushes back on the way up; only once the rail's bottom edge is above the pin does the pin
# snap out beneath it, and the rail must then be let down onto the pin rather than released early. Both arms, one rail
# each, at the same time.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, part  # noqa: E402
from taskbase import Trial, fake_state  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Raise both bed rails until they latch, then let go of them."

# -- the scene -------------------------------------------------------------------------------------------------------

FRAME = (0.55, 0.57, 0.60)
MATTRESS = (0.75, 0.82, 0.90)
RAIL_C = (0.85, 0.86, 0.88)
PIN_C = (0.85, 0.20, 0.15)
POS = (0.27, 0.0, TOP)

BED = (0.16, 0.18)  # bed footprint: 16 cm (forward) x 18 cm across
FRAME_H, MATTRESS_T = 0.05, 0.03
RAIL_Y = 0.10  # rail plane, either side
RAIL = (0.12, 0.01, 0.05)  # rail panel: 12 cm long, 1 cm thick, 5 cm tall
RAIL_Z0 = 0.03  # rail bottom when down (its top level with the mattress)
LIFT = 0.078  # travel to the latch: the lowered rail's bar (top 9.4 cm up) stays below the pin
BAR_R = 0.007
PIN_TOP = RAIL_Z0 + LIFT + 0.003  # the raised rail's bottom rests about here
PIN_H, PIN_W = 0.008, 0.014
PIN_OUT = RAIL_Y + RAIL[1] / 2 + 0.003  # pin tip y when sprung out (under the whole rail thickness)
PIN_TRAVEL = 0.016
SPRING = 30.0


def _rail(side):
    k = 1 if side == "left" else -1
    y = k * RAIL_Y
    return Joint(f"rail_{side}", kind="slide", axis="Z", anchor=(0, y, RAIL_Z0), lower=-0.002, upper=LIFT + 0.04,
                 initial=0.0, target=0.0, friction=0.2, damping=1.0, mass=0.4,
                 boxes=[((0, y, RAIL_Z0 + RAIL[2] / 2), RAIL, RAIL_C)],
                 cylinders=[((0, y, RAIL_Z0 + RAIL[2] + BAR_R), BAR_R, RAIL[0], RAIL_C, "X")])


def _pin(side):
    k = 1 if side == "left" else -1
    z = PIN_TOP - PIN_H / 2
    inner = k * (RAIL_Y - RAIL[1] / 2 - 0.012)  # pin body inside the rail's plane
    tip = k * PIN_OUT
    body_c = ((inner + k * (RAIL_Y - RAIL[1] / 2)) / 2)
    lo, hi = (-PIN_TRAVEL, 0.0) if k > 0 else (0.0, PIN_TRAVEL)
    bevel_c = k * (RAIL_Y + 0.0)  # bevelled tip under the rail: its underside slopes, so a rail rising pushes it in
    return Joint(f"latch_{side}", kind="slide", axis="Y", anchor=(0, k * RAIL_Y, z), lower=lo, upper=hi,
                 initial=0.0, target=0.0, stiffness=SPRING, damping=1.5, mass=0.02,
                 boxes=[((0, body_c, z), (PIN_W, abs(inner - k * (RAIL_Y - RAIL[1] / 2)), PIN_H), PIN_C),
                        ((0, bevel_c, z + 0.001), (PIN_W, abs(tip - k * (RAIL_Y - RAIL[1] / 2)) * 1.25, PIN_H * 1.1),
                         PIN_C, (math.pi / 2, -k * math.radians(35)))])  # underside rises outwards: a rising rail pushes it in


JOINTS = [_rail("left"), _rail("right"), _pin("left"), _pin("right")]
BEDPART = part("bed", pos=POS, fixed=True, self_collide=True, friction=0.3, joints=JOINTS, boxes=[
    ((0, 0, FRAME_H / 2), (BED[0], BED[1], FRAME_H), FRAME),
    ((0, 0, FRAME_H + MATTRESS_T / 2), (BED[0] - 0.01, BED[1] - 0.01, MATTRESS_T), MATTRESS),
    *[((sx * (RAIL[0] / 2 + 0.006), sy * RAIL_Y, 0.075), (0.008, 0.016, 0.15), FRAME) for sx in (1, -1) for sy in (1, -1)],
])
OBJECTS = [BEDPART]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A rail is latched when it is up (its bottom within 1.2 cm above the latch pin's top, never below it) and its latch
# pin is sprung out under it (within 3 mm of fully out). Score at the end of the trial: 0.3 per rail latched (0 to
# 0.6). 1.0 when both rails are latched, nothing moves and no gripper tool point is within 4 cm of either rail's top
# bar, so the pins alone hold the rails; held for 1 s. Completed less than 1 s before the run limit: 0.6.


def latched(j, side) -> bool:
    k = 1 if side == "left" else -1
    up = LIFT - 0.001 <= j.get(f"rail_{side}", 0.0) <= LIFT + 0.012
    pin_out = abs(j.get(f"latch_{side}", 0.0)) < 0.003
    return up and pin_out


def bar(s, side):
    k = 1 if side == "left" else -1
    return np.asarray(s.pos) + np.array((0, k * RAIL_Y, RAIL_Z0 + RAIL[2] + BAR_R + s.joints.get(f"rail_{side}", 0.0)))


def evaluate(objects, trial):
    s = objects["bed"]
    j = s.joints
    n = [latched(j, x) for x in ("left", "right")]
    trial.info.update(rails=[round(j.get(f"rail_{x}", 0.0), 4) for x in ("left", "right")],
                      pins=[round(j.get(f"latch_{x}", 0.0), 4) for x in ("left", "right")])
    done = all(n) and trial.still("bed") and trial.clear_of([bar(s, "left"), bar(s, "right")], 0.04)
    return 0.3 * sum(n), done


TRIAL = Trial(evaluate, trace_env="RAIL_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    j = objects["bed"].joints
    return float(sum(max(0.0, LIFT - j.get(f"rail_{x}", 0.0)) for x in ("left", "right")))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, both arms at once. Each arm grips its rail's top bar (pads closing across the 14 mm bar, along
# y), lifts its hand 3 cm past the latch height (the rail lags about 1 cm behind, so the pin has room to spring out), pauses, lowers it 2 cm so
# the rail settles onto the pin, lets go and lifts clear.


GRIP_X = -0.035  # grip the bar off centre, clear of the latch pin under its middle


def _side(side):
    k = 1 if side == "left" else -1
    b0 = np.asarray(POS) + np.array((GRIP_X, k * RAIL_Y, RAIL_Z0 + RAIL[2] + BAR_R))
    up = b0 + np.array((0, 0, LIFT + 0.03))  # the rail lags ~1 cm behind the hand
    rest = b0 + np.array((0, 0, LIFT - 0.005))
    yaw = 0.0
    return [Waypoint(1.5, tuple(b0 + np.array((0, 0, 0.05))), FINGER_OPEN * 0.45, f"over the {side} rail's bar", yaw=yaw),
            Waypoint(1.0, tuple(b0), FINGER_OPEN * 0.45, "descend", yaw=yaw),
            Waypoint(0.7, tuple(b0), FINGER_CLOSED, "grip the bar", yaw=yaw),
            Waypoint(2.6, tuple(up), FINGER_CLOSED, "lift the rail past its latch", yaw=yaw),
            Waypoint(1.0, tuple(up), FINGER_CLOSED, "let the pin spring out", yaw=yaw),
            Waypoint(0.8, tuple(rest), FINGER_CLOSED, "let the rail down onto the pin", yaw=yaw),
            Waypoint(0.6, tuple(rest), FINGER_OPEN * 0.45, "let go", yaw=yaw),
            Waypoint(0.7, tuple(rest + np.array((0, 0, 0.05))), FINGER_OPEN * 0.45, "lift clear", yaw=yaw),
            Waypoint(1.2, (0.12, k * 0.26, TOP + 0.16), FINGER_OPEN, "clear", yaw=yaw),
            Waypoint(3.0, (0.12, k * 0.26, TOP + 0.16), FINGER_OPEN, "wait", yaw=yaw)]


def demo():
    return {"left": _side("left"), "right": _side("right")}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(rl=0.0, rr=0.0, pl=0.0, pr=0.0):
        return {"bed": fake_state(POS, (0.08, 0.1, 0.06), {"rail_left": rl, "rail_right": rr, "latch_left": pl, "latch_right": pr})}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    on_bar = lambda: [bar(st(LIFT + 0.001, LIFT + 0.001)["bed"], "left"), np.array((0.1, -0.3, 1.0))]
    up = LIFT + 0.001
    return [
        ("both rails down", [st()] * 30, far, 0.0, False),
        ("left rail latched", [st(rl=up)] * 30, far, 0.3, False),
        ("both latched, let go, held 1 s", [st(up, up)] * 30, far, 1.0, True),
        ("both latched 0.75 s before the run limit", [st(up, up)] * 15, far, 0.6, True),
        ("both latched but still holding a bar", [st(up, up)] * 30, on_bar, 0.6, False),
        ("a rail held high with its pin pushed in", [st(up, LIFT + 0.02, 0.0, 0.012)] * 30, far, 0.3, False),
        ("a rail 1 cm short of the latch", [st(up, LIFT - 0.01)] * 30, far, 0.3, False),
    ]
