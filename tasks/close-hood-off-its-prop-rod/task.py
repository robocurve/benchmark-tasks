"""On the station table, between the arms: a vehicle's front end (a service training mock-up) with its hood standing
30 degrees open, resting on its prop rod. Nothing else is on the table."""

# Closing the hood after service is a core, many-times-a-day operation for automotive service technicians and
# mechanics (about 0.8 million US workers in 2024, BLS employment projections, 49-3023): lift the hood off its prop
# rod and hold it, stow the rod in its clip, then lower the hood and press it shut. Dropping the hood on the rod, or
# leaving the rod up, bends the rod or the hood.
#
# Physical difficulty (bimanual): the hood rests on the rod, so the rod cannot be stowed until the hood is lifted off
# it and held. One arm grips the hood's front edge and raises it about 10 degrees; the other pushes the rod's top over
# and down through a 90-degree arc into its stowed position along the fender; then the hood is let down and pressed shut.
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
INSTRUCTION = "Close the hood: lift it off the prop rod, stow the rod, then let the hood down and press it shut."

# -- the scene -------------------------------------------------------------------------------------------------------

BODY = (0.20, 0.22, 0.26)
PAINT = (0.70, 0.10, 0.10)
ROD_C = (0.85, 0.75, 0.15)
CHROME = (0.82, 0.83, 0.86)
POS = (0.30, 0.0, TOP)  # part frame origin: centre of the engine bay's footprint, on the table

BAY = (0.18, 0.20, 0.04)  # engine bay: 18 cm (forward) x 20 cm x 4 cm tall
HOOD_LEN, HOOD_W, HOOD_T = 0.17, 0.19, 0.008
HINGE = (BAY[0] / 2, 0.0, BAY[2] + 0.008)  # hood hinge along the bay's far top edge (the windscreen end)
OPEN0 = math.radians(30)  # the hood stands 30 degrees open on its rod
ROD_X, ROD_Y = -0.02, -0.117  # rod pivot on the right fender ledge, outside the hood
ROD_T = 0.006
ROD_ANCHOR = (ROD_X, ROD_Y, BAY[2])
# the rod's top meets the underside of the hood's side tab when the hood is 30 degrees open
ROD_LEN = (HINGE[2] + (HINGE[0] - ROD_X) * math.tan(OPEN0)) - BAY[2] - 0.002
TAB_X = HINGE[0] - (HINGE[0] - ROD_X) / math.cos(OPEN0)  # the side tab's position on the hood (closed)
UP = 0.0  # rod angle when standing (propping)
STOWED = -math.pi / 2  # lying along the fender towards the arms
PEG_Y, PEG_LEN = ROD_Y - 0.020, 0.028  # a finger peg on the rod's top, sticking out beyond the hood's side tab

HOOD = Joint("hood", kind="hinge", axis="Y", anchor=HINGE, lower=0.0, upper=0.95, initial=OPEN0, target=OPEN0,
             friction=0.01, damping=0.05, mass=0.25,
             boxes=[((HINGE[0] - HOOD_LEN / 2, 0, HINGE[2] + HOOD_T / 2), (HOOD_LEN, HOOD_W, HOOD_T), PAINT),
                    ((TAB_X, -HOOD_W / 2 - 0.012, HINGE[2] + HOOD_T / 2), (0.022, 0.024, HOOD_T), PAINT)],  # side tab
             cylinders=[((HINGE[0] - HOOD_LEN + 0.012, 0.04, HINGE[2] + HOOD_T + 0.007), 0.007, 0.05, CHROME, "Y")])
# rod: standing straight up under the hood's side tab (angle 0); stowed lying along the fender towards the arms
ROD = Joint("prop_rod", kind="hinge", axis="Y", anchor=ROD_ANCHOR, lower=STOWED - 0.02, upper=0.05, initial=UP,
            target=UP, friction=0.004, damping=0.005, mass=0.03,
            boxes=[((ROD_X, ROD_Y, BAY[2] + ROD_LEN / 2), (ROD_T, ROD_T, ROD_LEN), ROD_C),
                   ((ROD_X, ROD_Y - PEG_LEN / 2, BAY[2] + ROD_LEN - 0.006), (ROD_T, PEG_LEN, ROD_T), ROD_C)])  # peg

CAR = part("front_end", pos=POS, fixed=True, self_collide=True, joints=[HOOD, ROD], boxes=[
    ((0, 0, 0.006), (BAY[0], BAY[1], 0.012), BODY),
    ((-BAY[0] / 2 + 0.005, 0, BAY[2] / 2), (0.01, BAY[1], BAY[2]), BODY),
    ((BAY[0] / 2 - 0.005, 0, BAY[2] / 2), (0.01, BAY[1], BAY[2]), BODY),
    ((0, BAY[1] / 2 - 0.005, BAY[2] / 2), (BAY[0] - 0.02, 0.01, BAY[2]), BODY),
    ((0, -BAY[1] / 2 + 0.005, BAY[2] / 2), (BAY[0] - 0.02, 0.01, BAY[2]), BODY),
    ((0, ROD_Y, BAY[2] / 2 - 0.003), (BAY[0], 0.024, BAY[2] - 0.006), BODY),  # right fender ledge
    ((0.0, 0.0, 0.024), (0.08, 0.10, 0.024), (0.15, 0.15, 0.16)),  # engine block
])
OBJECTS = [CAR]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# The rod is stowed when it lies within 9 degrees of flat along the fender; the hood is shut within 3 degrees of
# closed. Score at the end of the trial: 0.3 for the rod stowed plus 0.3 for the hood shut (0 to 0.6). 1.0 when both
# hold, nothing moves (under 0.03 rad over the last 0.3 s) and no gripper tool point is within 4 cm of the hood's
# front edge or middle, held for 1 s. Completed less than 1 s before the run limit: 0.6.


def hood_pts(s):
    th = s.joints.get("hood", OPEN0)
    return [joint_point(s.pos, HINGE, "Y", th, (HINGE[0] - d, 0, HINGE[2] + HOOD_T)) for d in (HOOD_LEN, HOOD_LEN / 2)]


def evaluate(objects, trial):
    s = objects["front_end"]
    rod, hood = s.joints.get("prop_rod", UP), s.joints.get("hood", OPEN0)
    stowed, shut = rod < STOWED + math.radians(9), abs(hood) < math.radians(3)
    trial.info.update(rod=rod, hood=hood)
    done = stowed and shut and trial.still("front_end") and trial.clear_of(hood_pts(s), 0.04)
    return 0.3 * stowed + 0.3 * shut, done


TRIAL = Trial(evaluate, trace_env="HOOD_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    j = objects["front_end"].joints
    return float(abs(j.get("hood", OPEN0)) + abs(j.get("prop_rod", UP) - STOWED))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. The left arm grips the hood's chrome handle at its front edge (pads closing along the forward
# axis, across the 14 mm bar, which can turn between them) and raises the hood 11 degrees off the rod, holding it. The
# right arm, fingers closed, pushes the peg on the rod's top (outside the hood's side tab, which sits right above the
# rod) from its far side towards the arms and down through 90 degrees
# (finger tips trailing 1.3 cm behind a point 1.2 cm below the top) until it lies along the fender, and moves clear. The left arm lowers the hood
# back to the 30-degree mark and lets go; the hood falls shut (hinge damping slows it), and the left arm presses its
# front edge down and lifts clear.

TIP = 0.013
YAW_L, YAW_R = -math.pi / 2, math.pi / 2
LIFT_TO = OPEN0 + math.radians(11)


def handle_at(th):
    return joint_point(POS, HINGE, "Y", th, (HINGE[0] - HOOD_LEN + 0.012, 0.04, HINGE[2] + HOOD_T + 0.007))


def demo():
    lift = [handle_at(OPEN0 + (LIFT_TO - OPEN0) * i / 6) for i in range(7)]
    left = [Waypoint(1.5, tuple(lift[0] + np.array((0, 0, 0.05))), FINGER_OPEN * 0.45, "over the hood handle", yaw=YAW_L),
            Waypoint(1.0, tuple(lift[0]), FINGER_OPEN * 0.45, "descend", yaw=YAW_L),
            Waypoint(0.6, tuple(lift[0]), FINGER_CLOSED, "grip the handle", yaw=YAW_L)]
    left += [Waypoint(0.25, tuple(p), FINGER_CLOSED, "raise the hood off the rod", yaw=YAW_L) for p in lift[1:]]
    rod_pt0 = (ROD_X + ROD_T / 2, PEG_Y - 0.004, BAY[2] + ROD_LEN - 0.006)  # the peg's far side, clear of the tab
    path = arc(POS, ROD_ANCHOR, "Y", rod_pt0, UP, STOWED + 0.15 * (1 if STOWED > UP else -1), 10, standoff=0.010, lift=0.0)
    t_lift = sum(w.duration for w in left)
    right = [Waypoint(t_lift + 0.3, (0.14, -0.26, TOP + 0.16), FINGER_CLOSED, "wait for the hood", yaw=YAW_R),
             Waypoint(1.3, tuple(path[0] + np.array((0.03, 0, 0.03))), FINGER_CLOSED, "beyond the rod's top", yaw=YAW_R),
             Waypoint(0.6, tuple(path[0]), FINGER_CLOSED, "on the rod", yaw=YAW_R)]
    right += [Waypoint(0.25, tuple(p), FINGER_CLOSED, "push the rod over and down", yaw=YAW_R) for p in path]
    right += [Waypoint(0.7, tuple(path[-1] + np.array((0, -0.03, 0.06))), FINGER_CLOSED, "lift clear", yaw=YAW_R),
              Waypoint(1.2, (0.14, -0.26, TOP + 0.16), FINGER_CLOSED, "clear", yaw=YAW_R)]
    t_rod = sum(w.duration for w in right)
    down = [handle_at(LIFT_TO - (LIFT_TO - OPEN0) * i / 4) for i in range(5)]
    left += [Waypoint(max(0.5, t_rod - t_lift), tuple(lift[-1]), FINGER_CLOSED, "hold the hood up", yaw=YAW_L)]
    left += [Waypoint(0.25, tuple(p), FINGER_CLOSED, "lower the hood", yaw=YAW_L) for p in down[1:]]
    shut_front = handle_at(0.0)
    left += [Waypoint(0.5, tuple(down[-1]), FINGER_OPEN * 0.45, "let go: the hood falls shut", yaw=YAW_L),
             Waypoint(0.8, tuple(down[-1] + np.array((-0.03, 0, 0.04))), FINGER_CLOSED, "back off", yaw=YAW_L),
             Waypoint(1.5, tuple(shut_front + np.array((0.02, -0.04, 0.03))), FINGER_CLOSED, "over the shut hood's front", yaw=YAW_L),
             Waypoint(0.6, tuple(shut_front + np.array((0.02, -0.04, 0.004))), FINGER_CLOSED, "press it down", yaw=YAW_L),
             Waypoint(0.6, tuple(shut_front + np.array((0.02, -0.04, 0.06))), FINGER_CLOSED, "lift clear", yaw=YAW_L),
             Waypoint(1.2, (0.14, 0.26, TOP + 0.16), FINGER_CLOSED, "clear", yaw=YAW_L),
             Waypoint(3.0, (0.14, 0.26, TOP + 0.16), FINGER_CLOSED, "wait", yaw=YAW_L)]
    right += [Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right), (0.14, -0.26, TOP + 0.16),
                       FINGER_CLOSED, "wait", yaw=YAW_R)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(hood, rod):
        return {"front_end": fake_state(POS, (0.09, 0.1, 0.05), {"hood": hood, "prop_rod": rod})}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    on_hood = lambda: [hood_pts(st(0.0, STOWED)["front_end"])[0] + np.array((0, 0, 0.01)), np.array((0.1, -0.3, 1.0))]
    return [
        ("untouched", [st(OPEN0, UP)] * 30, far, 0.0, False),
        ("hood held up, rod stowed", [st(LIFT_TO, STOWED)] * 30, far, 0.3, False),
        ("hood shut on the stowed rod, held 1 s", [st(0.0, STOWED)] * 30, far, 1.0, True),
        ("shut 0.75 s before the run limit", [st(0.0, STOWED)] * 15, far, 0.6, True),
        ("shut but a gripper still on the hood", [st(0.0, STOWED)] * 30, on_hood, 0.6, False),
        ("rod half stowed", [st(OPEN0 + 0.1, -0.8)] * 30, far, 0.0, False),
        ("hood resting 6 degrees open", [st(0.1, STOWED)] * 30, far, 0.3, False),
    ]
