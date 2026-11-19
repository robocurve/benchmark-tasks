"""On the station table, between the arms: a water line running across the table with a filter housing in the
middle, a quarter-turn ball valve on each side of it (both open, levers in line with the pipe) and a drain valve on
the housing's branch (closed, lever across the branch). Nothing else is on the table."""

# Isolating and draining a filter housing before changing its cartridge is a core operation for general maintenance
# and repair workers (about 1.6 million US workers in 2024, BLS employment projections, 49-9071): close the isolation
# valves on both sides, then open the drain. Opening the drain while the line is still live sprays the room.
#
# Physical difficulty: three quarter-turn ball valves, each with a stiff lever (stem friction 0.15 N m) that must be
# pushed through a 90-degree arc about its vertical stem, in order: both isolation valves before the drain. Both arms
# close one isolation valve each at the same time; the left arm then opens the drain.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, part  # noqa: E402
from taskbase import Trial, arc, fake_state, joint_point  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Isolate the filter for a cartridge change: close both line valves, then open the drain valve."

# -- the scene -------------------------------------------------------------------------------------------------------

COPPER = (0.72, 0.45, 0.25)
BRASS = (0.78, 0.62, 0.25)
RED = (0.80, 0.12, 0.10)
BLUE = (0.15, 0.30, 0.70)
GREY = (0.55, 0.57, 0.60)
POS = (0.29, 0.0, TOP)  # part frame origin: below the filter housing, on the table

PIPE_Z, PIPE_R = 0.05, 0.011
VALVE_Y = 0.10  # isolation valves 10 cm either side of the housing
STEM_Z = PIPE_Z + 0.022
LEVER = 0.065
DRAIN_X = -0.055  # drain valve on a branch running from the housing towards the arms
DRAIN_Z = 0.03
QUARTER = math.pi / 2
FRICTION = 0.15  # N m: a ball valve's stem is stiff


def _lever(name, anchor, direction, lo, hi, colour):
    d = np.asarray(direction, float)
    c = np.asarray(anchor, float) + d * LEVER / 2 + np.array((0, 0, 0.004))
    size = (abs(d[0]) * LEVER + 0.012 * (1 - abs(d[0])), abs(d[1]) * LEVER + 0.012 * (1 - abs(d[1])), 0.006)
    grip = np.asarray(anchor, float) + d * (LEVER - 0.012) + np.array((0, 0, 0.004))
    return Joint(name, kind="hinge", axis="Z", anchor=anchor, lower=lo, upper=hi, initial=0.0, target=0.0,
                 friction=FRICTION, damping=0.05, mass=0.05,
                 boxes=[(tuple(c), size, BRASS), (tuple(grip), (0.022, 0.022, 0.009) if d[0] == 0 else (0.022, 0.022, 0.009), colour)])


# line valves: levers in line with the pipe (open), turning 90 degrees to point at the arms (closed)
V_LEFT = _lever("valve_left", (0.0, VALVE_Y, STEM_Z), (0, 1, 0), 0.0, QUARTER + 0.05, RED)
V_RIGHT = _lever("valve_right", (0.0, -VALVE_Y, STEM_Z), (0, -1, 0), -QUARTER - 0.05, 0.0, RED)
# drain valve: lever across its branch (closed), turning 90 degrees to lie along the branch, towards the arms (open)
DRAIN = _lever("drain", (DRAIN_X, 0.0, DRAIN_Z + 0.02), (0, 1, 0), 0.0, QUARTER + 0.05, BLUE)

LINE = part("filter_line", pos=POS, fixed=True, joints=[V_LEFT, V_RIGHT, DRAIN], boxes=[
    ((0, 0.13, PIPE_Z / 2 - 0.006), (0.03, 0.02, PIPE_Z - 0.012), GREY), ((0, -0.13, PIPE_Z / 2 - 0.006), (0.03, 0.02, PIPE_Z - 0.012), GREY),  # pipe stands
    ((0, VALVE_Y, PIPE_Z), (0.028, 0.034, 0.028), BRASS), ((0, -VALVE_Y, PIPE_Z), (0.028, 0.034, 0.028), BRASS),  # valve bodies
    ((0, VALVE_Y, PIPE_Z + 0.017), (0.01, 0.01, 0.01), BRASS), ((0, -VALVE_Y, PIPE_Z + 0.017), (0.01, 0.01, 0.01), BRASS),  # stems
    ((DRAIN_X / 2, 0, DRAIN_Z), (abs(DRAIN_X), 0.016, 0.016), COPPER),  # drain branch
    ((DRAIN_X, 0, DRAIN_Z), (0.022, 0.026, 0.022), BRASS), ((DRAIN_X, 0, DRAIN_Z + 0.015), (0.01, 0.01, 0.012), BRASS),
], cylinders=[((0, 0, PIPE_Z), PIPE_R, 0.30, COPPER, "Y"),
              ((0, 0, 0.032), 0.028, 0.06, BLUE, "Z")])  # filter housing
OBJECTS = [LINE]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A valve is turned when its lever has moved at least 80 degrees of its quarter turn: the line valves are then closed,
# the drain open. Score at the end of the trial: 0.2 per line valve closed plus 0.2 for the drain open (0 to 0.6).
# 1.0 when all three hold, nothing moves and no gripper tool point is within 4 cm of a lever's grip, held for 1 s.
# Completed less than 1 s before the run limit: 0.6. Defect that scores 0: the drain opened more than 30 degrees while
# either line valve is still less than 80 degrees closed (draining a live line).

TURNED = math.radians(80)
DRAIN_LIVE = math.radians(30)


def grip(s, j: Joint):
    th = s.joints.get(j.name, 0.0)
    box_c = j.boxes[1][0]
    return joint_point(s.pos, j.anchor, "Z", th, box_c)


def evaluate(objects, trial):
    s = objects["filter_line"]
    j = s.joints
    left = j.get("valve_left", 0.0) >= TURNED
    right = j.get("valve_right", 0.0) <= -TURNED
    drain = j.get("drain", 0.0)
    if drain > DRAIN_LIVE and not (left and right):
        trial.defect("drain opened on a live line")
    trial.info.update(left=left, right=right, drain=drain)
    done = left and right and drain >= TURNED and trial.still("filter_line") and trial.clear_of(
        [grip(s, V_LEFT), grip(s, V_RIGHT), grip(s, DRAIN)], 0.04)
    return 0.2 * (left + right + (drain >= TURNED)), done


TRIAL = Trial(evaluate, trace_env="VALVE_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    j = objects["filter_line"].joints
    return float(max(0.0, TURNED - j.get("valve_left", 0.0)) + max(0.0, TURNED + j.get("valve_right", 0.0))
                 + max(0.0, TURNED - j.get("drain", 0.0)))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, closed fingers. Each arm pushes its line valve's lever, 4.5 cm out from the stem, through 95
# degrees about the stem (finger tips trailing 1.3 cm behind it) so that it ends pointing at the arms, and lifts off.
# When both line valves are closed, the left arm pushes the drain lever through 95 degrees the same way.

N = 10


def _turn(j: Joint, to, label, yaw):
    d = np.asarray(j.boxes[0][0], float) - np.asarray(j.anchor, float)
    d[2] = 0.0
    d /= np.linalg.norm(d)
    p0 = np.asarray(j.anchor, float) + d * 0.045 + np.array((0, 0, 0.004))
    path = arc(POS, j.anchor, "Z", p0, 0.0, to, N, standoff=0.016, lift=0.009)
    pre = path[0] - (path[1] - path[0]) / (np.linalg.norm(path[1] - path[0]) + 1e-9) * 0.025
    wps = [Waypoint(1.3, tuple(pre + np.array((0, 0, 0.04))), FINGER_CLOSED, f"above the {label} lever", yaw=yaw),
           Waypoint(0.7, tuple(pre), FINGER_CLOSED, f"beside the {label} lever", yaw=yaw)]
    wps += [Waypoint(0.25, tuple(p), FINGER_CLOSED, f"turn the {label} valve") for p in path]
    wps = [Waypoint(w.duration, w.pos, w.finger, w.label, yaw=yaw) for w in wps]
    wps += [Waypoint(0.6, tuple(path[-1] + np.array((0, 0, 0.05))), FINGER_CLOSED, "lift off", yaw=yaw)]
    return wps


def demo():
    yl, yr = -math.pi / 2, math.pi / 2
    left = _turn(V_LEFT, QUARTER + 0.05, "left line", yl)
    right = _turn(V_RIGHT, -QUARTER - 0.05, "right line", yr)
    right += [Waypoint(1.2, (0.14, -0.27, TOP + 0.16), FINGER_CLOSED, "clear", yaw=yr)]
    t = max(sum(w.duration for w in left), sum(w.duration for w in right))
    left += [Waypoint(max(0.4, t - sum(w.duration for w in left) + 0.3), tuple(np.asarray(left[-1].pos)), FINGER_CLOSED,
                      "wait until both line valves are shut", yaw=yl)]
    left += _turn(DRAIN, QUARTER + 0.05, "drain", yl)
    left += [Waypoint(1.2, (0.14, 0.27, TOP + 0.16), FINGER_CLOSED, "clear", yaw=yl),
             Waypoint(3.0, (0.14, 0.27, TOP + 0.16), FINGER_CLOSED, "wait", yaw=yl)]
    right += [Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right), (0.14, -0.27, TOP + 0.16),
                       FINGER_CLOSED, "wait", yaw=yr)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(l=0.0, r=0.0, d=0.0):
        return {"filter_line": fake_state(POS, (0.04, 0.15, 0.05), {"valve_left": l, "valve_right": r, "drain": d})}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    shut = dict(l=1.6, r=-1.6)
    on_drain = lambda: [grip(st(**shut, d=1.6)["filter_line"], DRAIN), np.array((0.1, -0.3, 1.0))]
    return [
        ("untouched", [st()] * 30, far, 0.0, False),
        ("one line valve closed", [st(l=1.6)] * 30, far, 0.2, False),
        ("both line valves closed", [st(**shut)] * 30, far, 0.4, False),
        ("isolated and drained, held 1 s", [st(**shut)] * 3 + [st(**shut, d=1.6)] * 30, far, 1.0, True),
        ("done 0.75 s before the run limit", [st(**shut)] * 3 + [st(**shut, d=1.6)] * 15, far, 0.6, True),
        ("done but a gripper still on the drain lever", [st(**shut)] * 3 + [st(**shut, d=1.6)] * 30, on_drain, 0.6, False),
        ("drain opened with the right valve still open", [st(l=1.6, d=1.0)] * 3 + [st(**shut, d=1.6)] * 30, far, 0.0, False),
        ("drain cracked 20 degrees early (allowed)", [st(l=1.6, d=0.35)] * 3 + [st(**shut, d=1.6)] * 30, far, 1.0, True),
    ]
