"""On the station table, in front of the arms: a small hinged storage-room door in its frame, standing 29 degrees
open, pushed open by its compressed seal; a barrel bolt on its face and a keeper in the latch post. Nothing else is
on the table."""

# Bolting a sprung door shut, holding it closed against its seal while sliding the bolt into the keeper, is a core
# lock-up operation for janitors and building cleaners (about 2.4 million US workers in 2024, BLS employment projections, 37-2011).
#
# Physical difficulty (bimanual): the door springs back open unless held. One arm must push it shut and keep pushing,
# while the other slides the bolt 3 cm along the door into the keeper's slot; once the bolt is in, the keeper's lip
# holds the door and both arms can let go. A bolt thrown with the door open fouls the latch post and the door then
# cannot close.
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
INSTRUCTION = "Bolt the door shut: hold it closed and slide the bolt into the keeper, then let go."

# -- the scene -------------------------------------------------------------------------------------------------------

FRAME = (0.80, 0.78, 0.74)
DOOR = (0.30, 0.45, 0.62)
BRASS = (0.78, 0.62, 0.25)
POS = (0.30, 0.0, TOP)  # part frame origin: centre of the door's closed line, on the table

DOOR_T, DOOR_H = 0.012, 0.11
HINGE_Y, EDGE_Y = 0.14, -0.068  # door spans y from the latch edge to the hinge
POST_Y0, POST_Y1 = -0.09, -0.07  # latch post
BOLT_Z = 0.095
BOLT_X = -DOOR_T / 2 - 0.005  # bolt runs along the door's near face
BOLT_LEN, BOLT_THROW = 0.05, 0.03
BOLT_TIP0 = -0.06  # bolt tip y when withdrawn (inside the door's width)
OPEN0 = -0.5  # rad: the seal pushes the door 29 degrees open
LIP_X = BOLT_X - 0.009  # the keeper's lip, on the near side of the bolt

HINGE_A = (0, HINGE_Y, 0.065)  # hinge anchor in the table-aligned frame (part origin at POS) used by scoring and demo
DOOR_BOXES = [((0, (HINGE_Y + EDGE_Y) / 2, 0.014 + DOOR_H / 2), (DOOR_T, HINGE_Y - EDGE_Y - 0.002, DOOR_H), DOOR)]
BOLT_ANCHOR = (BOLT_X, BOLT_TIP0 + BOLT_LEN / 2, BOLT_Z)
BOLT_BOXES = [((BOLT_X, BOLT_TIP0 + BOLT_LEN / 2, BOLT_Z), (0.01, BOLT_LEN, 0.01), BRASS),
              ((BOLT_X - 0.012, BOLT_TIP0 + BOLT_LEN / 2, BOLT_Z), (0.014, 0.012, 0.012), BRASS)]  # knob

_post = []
for zlo, zhi in ((0.0, BOLT_Z - 0.0075), (BOLT_Z + 0.0075, 0.13)):
    _post.append(((0.0, (POST_Y0 + POST_Y1) / 2, (zlo + zhi) / 2), (0.04, POST_Y1 - POST_Y0, zhi - zlo), FRAME))
_post.append(((LIP_X + 0.0015, (POST_Y0 + POST_Y1) / 2, BOLT_Z), (0.003, POST_Y1 - POST_Y0, 0.015), FRAME))  # lip, 3 mm
_post.append((((BOLT_X + 0.006 + 0.02) / 2, (POST_Y0 + POST_Y1) / 2, BOLT_Z), (0.02 - BOLT_X - 0.006, POST_Y1 - POST_Y0, 0.015), FRAME))  # back

FRAME_BOXES = [
    ((0, 0.035, 0.006), (0.06, 0.30, 0.012), FRAME),  # sill
    ((0.0, HINGE_Y + 0.018, 0.035), (0.04, 0.016, 0.07), FRAME),  # hinge post (knuckle height), clear of the door's corner
    *_post,
]

# The part is built in a frame turned to the door's open position, with its origin on the hinge: the door's joint
# reads 0 when the door stands 29 degrees open (where its seal spring rests) and +OPEN_SPAN when shut against the post.
# Scoring and the demonstration work in the table-aligned frame: door angle there = joint + OPEN0.
OPEN_SPAN = -OPEN0
_H = np.array((0.0, HINGE_Y, 0.0))
_RZ = np.array(((math.cos(-OPEN0), -math.sin(-OPEN0), 0), (math.sin(-OPEN0), math.cos(-OPEN0), 0), (0, 0, 1)))


def _frame_box(bx):
    c, size, colour = bx
    return (tuple(_RZ @ (np.asarray(c, float) - _H)), size, colour, -OPEN0)


def _moving_box(bx):
    c, size, colour = bx
    return (tuple(np.asarray(c, float) - _H), size, colour)


DOOR_J = Joint("door", kind="hinge", axis="Z", anchor=(0, 0, HINGE_A[2]), lower=-1.2 - OPEN0, upper=OPEN_SPAN,
               initial=0.0, target=0.0, stiffness=0.6, damping=0.05, mass=0.3,
               boxes=[_moving_box(bx) for bx in DOOR_BOXES])
BOLT_J = Joint("bolt", kind="slide", axis="Y", parent="door", anchor=tuple(np.asarray(BOLT_ANCHOR) - _H),
               lower=-BOLT_THROW, upper=0.0, initial=0.0, target=0.0, friction=0.4, damping=0.5, mass=0.03,
               boxes=[_moving_box(bx) for bx in BOLT_BOXES])
DOORSET = part("doorset", pos=(POS[0], POS[1] + HINGE_Y, POS[2]), yaw=OPEN0, fixed=True, self_collide=True,
               joints=[DOOR_J, BOLT_J], boxes=[_frame_box(bx) for bx in FRAME_BOXES])


def door_angle(ds) -> float:
    """Door angle in the table-aligned frame: OPEN0 (29 degrees open) at rest, 0 when shut."""
    return ds.joints.get("door", 0.0) + OPEN0
OBJECTS = [DOORSET]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# The door is shut when its hinge is within 2 degrees of the frame stop; the bolt is thrown when it has slid at
# least 2.5 cm of its 3 cm travel. Score at the end of the trial: 0.3 for the door shut, plus 0.3 for the bolt thrown
# (0 to 0.6). 1.0 when the door is shut and bolted, nothing moves (under 0.03 rad / 5 mm over the last 0.3 s) and no
# gripper tool point is within 5 cm of the door's face or the bolt knob, so the bolt alone is holding the door; held
# for 1 s. Completed less than 1 s before the run limit: 0.6.

SHUT = math.radians(2)
THROWN = 0.025


def knob(ds):
    j = ds.joints
    p = joint_point(POS, HINGE_A, "Z", door_angle(ds),
                    (BOLT_X - 0.012, BOLT_TIP0 + BOLT_LEN / 2 + j.get("bolt", 0.0), BOLT_Z))
    return p


def door_face(ds, y):
    return joint_point(POS, HINGE_A, "Z", door_angle(ds), (-DOOR_T / 2, y, 0.05))


def evaluate(objects, trial):
    ds = objects["doorset"]
    th, b = door_angle(ds), ds.joints.get("bolt", 0.0)
    shut, thrown = th > -SHUT, b < -THROWN
    trial.info.update(door=th, bolt=b)
    done = (shut and thrown and trial.still("doorset")
            and trial.clear_of([knob(ds), door_face(ds, 0.10), door_face(ds, 0.05), door_face(ds, 0.0), door_face(ds, -0.05)], 0.05))
    return 0.3 * shut + 0.3 * thrown, done


TRIAL = Trial(evaluate, trace_env="DOOR_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    j = objects["doorset"].joints
    return float(abs(door_angle(objects["doorset"])) * 0.15 + max(0.0, THROWN + j.get("bolt", 0.0)))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, closed fingers. The left arm pushes the door shut along its arc (finger tips trailing 1.2 cm
# behind a point 4.5 cm in from the hinge, 9 cm up, near the door's top edge), pressing on 17 degrees past the stop (the hand lags about 1 cm under the spring), and holds it there. The right arm
# then pushes the bolt's knob 4.4 cm along the door into the keeper and backs off; the left arm lets go last.

T_CLOSE = 3.2
HOLD_Z = 0.092  # push the door near its top edge: the gripper's body stays above the door
MIN_TCP_SEP = 0.10  # the two hands work 11 cm apart along the door (checked by check_task.py)
YAW = math.pi / 2  # pads close along x: the grippers' housings (5.1 cm along the closing axis) stay apart along y


def demo():
    return _demo()


def _demo():
    hold_pt = (-DOOR_T / 2, HINGE_Y - 0.045, HOLD_Z)
    path = arc(POS, HINGE_A, "Z", hold_pt, OPEN0, 0.30, 10, standoff=0.010, lift=0.0)  # 17 degrees past the stop: the hand lags ~1 cm
    start = path[0] + (path[0] - path[1]) / (np.linalg.norm(path[0] - path[1]) + 1e-9) * 0.03
    left = [Waypoint(1.2, tuple(start + np.array((0, 0, 0.12))), FINGER_CLOSED, "high above the open door", yaw=-YAW),
            Waypoint(0.8, tuple(start + np.array((0, 0, 0.04))), FINGER_CLOSED, "above the open door", yaw=-YAW),
            Waypoint(0.8, tuple(start), FINGER_CLOSED, "behind the door", yaw=-YAW)]
    left += [Waypoint(T_CLOSE / 10, tuple(p), FINGER_CLOSED, "push the door shut", yaw=-YAW) for p in path]
    hold = path[-1]
    knob0 = np.asarray(POS) + np.array((BOLT_X - 0.012, BOLT_TIP0 + BOLT_LEN / 2, BOLT_Z))
    right = [Waypoint(2.8 + T_CLOSE + 0.6, (0.14, -0.22, TOP + 0.16), FINGER_CLOSED, "wait for the door", yaw=YAW),
             Waypoint(1.4, tuple(knob0 + np.array((-0.004, 0.035, 0.05))), FINGER_CLOSED, "above the bolt knob", yaw=YAW),
             Waypoint(0.8, tuple(knob0 + np.array((-0.004, 0.022, 0.002))), FINGER_CLOSED, "beside the knob", yaw=YAW),
             Waypoint(1.6, tuple(knob0 + np.array((-0.004, 0.022 - BOLT_THROW - 0.014, 0.002))), FINGER_CLOSED,
                      "slide the bolt into the keeper", yaw=YAW),
             Waypoint(0.5, tuple(knob0 + np.array((-0.004, 0.022 - BOLT_THROW - 0.014, 0.06))), FINGER_CLOSED, "lift off", yaw=YAW),
             Waypoint(1.2, (0.14, -0.22, TOP + 0.16), FINGER_CLOSED, "clear", yaw=YAW),
             Waypoint(4.0, (0.14, -0.22, TOP + 0.16), FINGER_CLOSED, "wait", yaw=YAW)]
    t_bolt = sum(w.duration for w in right[:5])
    t_left = sum(w.duration for w in left)
    left += [Waypoint(max(0.5, t_bolt - t_left + 0.4), tuple(hold), FINGER_CLOSED, "hold the door shut", yaw=-YAW),
             Waypoint(0.8, tuple(hold + np.array((-0.04, 0.0, 0.04))), FINGER_CLOSED, "let go of the door", yaw=-YAW),
             Waypoint(1.2, (0.14, 0.24, TOP + 0.16), FINGER_CLOSED, "clear", yaw=-YAW),
             Waypoint(3.0, (0.14, 0.24, TOP + 0.16), FINGER_CLOSED, "wait", yaw=-YAW)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(door, bolt):
        return {"doorset": fake_state(POS, (0.03, 0.12, 0.07), {"door": door - OPEN0, "bolt": bolt})}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    holding = lambda: [np.asarray(door_face(st(0.0, -0.03)["doorset"], 0.08)) + np.array((-0.012, 0, 0)),
                       np.array((0.1, -0.3, 1.0))]
    return [
        ("door open, bolt back", [st(OPEN0, 0.0)] * 30, far, 0.0, False),
        ("door held shut, bolt back", [st(0.0, 0.0)] * 30, holding, 0.3, False),
        ("door shut and bolted but still held", [st(0.0, -0.03)] * 30, holding, 0.6, False),
        ("door shut and bolted, let go, held 1 s", [st(0.0, -0.03)] * 30, far, 1.0, True),
        ("bolted 0.75 s before the run limit", [st(0.0, -0.03)] * 15, far, 0.6, True),
        ("bolt thrown with the door open", [st(-0.4, -0.03)] * 30, far, 0.3, False),
        ("bolt only half thrown", [st(0.0, -0.015)] * 30, far, 0.3, False),
    ]
