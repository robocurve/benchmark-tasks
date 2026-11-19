"""On the station table, between the arms: an assembly fixture plate with a part nest and a swing clamp whose arm lies
over a finished part in the nest. An output tray and a fresh blank stand on the right. Nothing else is on the
table."""

# Changing parts in a swing-clamp fixture is a core, repeated operation for miscellaneous assemblers and
# fabricators (about 1.46 million US workers, BLS OEWS May 2024, 51-2090): open the clamp, unload the finished
# part, load the next blank into the nest, and clamp it. Loading over the finished part, or leaving the blank
# unclamped, scraps the next part.
#
# Physical difficulty: the clamp is a swing arm on a stiff vertical pivot lying 3 mm over the part, so nothing can come
# out of or go into the nest until its knob has been pushed round 90 degrees, and it closes by pushing it back round
# over the new blank; the part and blank are lifted out of and lowered into a nest with 6 mm of play. The left arm
# works the clamp, the right arm the parts.
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
INSTRUCTION = ("Swing the clamp open, put the finished part in the output tray, load the blank into the nest and "
               "swing the clamp back over it.")

# -- the scene -------------------------------------------------------------------------------------------------------

STEEL = (0.45, 0.47, 0.50)
BLUE = (0.15, 0.30, 0.65)  # clamp
RED = (0.75, 0.12, 0.10)  # handle grip
BRIGHT = (0.85, 0.87, 0.90)  # machined finish
RAW = (0.55, 0.52, 0.47)  # saw-cut blank
GREEN = (0.20, 0.45, 0.30)

FIX_POS = (0.22, 0.0, TOP)  # part frame origin: fixture centre, on the table
PLATE = 0.015
PART = (0.04, 0.05, 0.03)  # finished part and blank: 40 (forward) x 50 x 30 mm tall, 120 g
PART_MASS = 0.12
PLAY = 0.006
NEST_IN = (PART[0] + 2 * PLAY, PART[1] + 2 * PLAY)
NEST_WALL, NEST_H = 0.006, 0.012
NEST_Y = -0.04  # nest centre, in the part frame
SWING_R = 0.075  # swing clamp: pivot to pad centre
ARM_Z = PLATE + PART[2] + 0.003  # the arm's underside: 3 mm over a seated part, so a clamped part cannot lift out
PIVOT = (0.0, NEST_Y + SWING_R, ARM_Z)  # anchor, part frame: the pivot post stands beside the nest
KNOB_R, KNOB_H = 0.05, 0.03  # knob on the arm, 5 cm out from the pivot, standing 3 cm above it
OPEN_ANGLE = math.radians(60)  # open: the arm swung at least 60 degrees off the nest (it swings to 90 at its stop)
OPEN_STOP = math.radians(92)

def world(p):
    return np.asarray(FIX_POS) + np.asarray(p, float)


NEST_C = world((0.0, NEST_Y, PLATE))
SEATED_Z = TOP + PLATE + PART[2] / 2

# The clamp arm swings about the vertical pivot post: at 0 it lies over the nest (pad over the part), swung +90 degrees
# it points forward, clear of the nest. A stiff pivot (0.12 N m) holds it wherever it is left.
CLAMP = Joint(
    "clamp", kind="hinge", axis="Z", anchor=PIVOT, lower=-0.03, upper=OPEN_STOP, initial=0.0, target=0.0,
    friction=0.12, damping=0.05, mass=0.08,
    boxes=[
        ((0, PIVOT[1] - SWING_R / 2, ARM_Z + 0.007), (0.022, SWING_R + 0.022, 0.012), BLUE),  # arm
        ((0, NEST_Y, ARM_Z + 0.003), (0.026, 0.026, 0.004), BLUE),  # pad under the arm's end
        ((0, PIVOT[1] - KNOB_R, ARM_Z + 0.013 + KNOB_H / 2), (0.012, 0.012, KNOB_H), RED),  # knob
    ])

FIXTURE = part("fixture", pos=FIX_POS, fixed=True, joints=[CLAMP], boxes=[
    ((0, 0, PLATE / 2), (0.12, 0.24, PLATE), STEEL),
    ((-NEST_IN[0] / 2 - NEST_WALL / 2, NEST_Y, PLATE + NEST_H / 2), (NEST_WALL, NEST_IN[1] + 2 * NEST_WALL, NEST_H), STEEL),
    ((NEST_IN[0] / 2 + NEST_WALL / 2, NEST_Y, PLATE + NEST_H / 2), (NEST_WALL, NEST_IN[1] + 2 * NEST_WALL, NEST_H), STEEL),
    ((0, NEST_Y - NEST_IN[1] / 2 - NEST_WALL / 2, PLATE + NEST_H / 2), (NEST_IN[0], NEST_WALL, NEST_H), STEEL),
    ((0, NEST_Y + NEST_IN[1] / 2 + NEST_WALL / 2, PLATE + NEST_H / 2), (NEST_IN[0], NEST_WALL, NEST_H), STEEL),
    ((0, PIVOT[1], (PLATE + ARM_Z) / 2 - 0.0005), (0.022, 0.022, ARM_Z - PLATE - 0.001), STEEL),  # pivot post
])

TRAY_XY, TRAY_IN, TRAY_LIP = (0.27, -0.25), 0.11, 0.015
BLANK_START = (0.13, -0.20)

FINISHED = part("finished_part", pos=(NEST_C[0], NEST_C[1], SEATED_Z), mass=PART_MASS, color=BRIGHT,
                boxes=[((0, 0, 0), PART, BRIGHT)])
BLANK = part("blank", pos=(*BLANK_START, TOP + PART[2] / 2), mass=PART_MASS, color=RAW, boxes=[((0, 0, 0), PART, RAW)])
_TW = TRAY_IN + 0.016
TRAY = part("output_tray", pos=(*TRAY_XY, TOP), fixed=True, color=GREEN, boxes=[
    ((0, 0, 0.005), (_TW, _TW, 0.01), GREEN),
    ((-TRAY_IN / 2 - 0.004, 0, 0.01 + TRAY_LIP / 2), (0.008, _TW, TRAY_LIP), GREEN),
    ((TRAY_IN / 2 + 0.004, 0, 0.01 + TRAY_LIP / 2), (0.008, _TW, TRAY_LIP), GREEN),
    ((0, TRAY_IN / 2 + 0.004, 0.01 + TRAY_LIP / 2), (TRAY_IN, 0.008, TRAY_LIP), GREEN),
    ((0, -TRAY_IN / 2 - 0.004, 0.01 + TRAY_LIP / 2), (TRAY_IN, 0.008, TRAY_LIP), GREEN),
])
OBJECTS = [FIXTURE, FINISHED, BLANK, TRAY]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A part is in the nest when its centre lies inside the nest's inside rectangle seen from above and less than 1.5 cm
# above where it rests on the plate; in the tray when its centre lies inside the tray's inside square, less than
# 1.5 cm above the tray floor. Still: moved under 5 mm over the last 0.3 s. The clamp is open once its arm has been
# swung more than 60 degrees off the nest, and closed when its arm is back within 5 degrees of lying over the nest.
#   0.2 [clamp opened, at any time] + 0.2 [finished part in the tray, still] + 0.2 [blank in the nest, still, with
#   the finished part out] + 0.2 [clamp closed on the blank] (0 to 0.8), at the end of the trial;
#   1.0 when all four hold, nothing moves and no tool point is within 4 cm of the blank, the finished part or the
#   pad, held for 1 s. Completed less than 1 s before the run limit: 0.8.
# Defects that score 0: the blank and the finished part in the nest at the same time (loading over the finished part),
# or either part falling off the table.


def in_nest(s) -> bool:
    x, y, z = (float(v) for v in s.pos)
    return abs(x - NEST_C[0]) <= NEST_IN[0] / 2 and abs(y - NEST_C[1]) <= NEST_IN[1] / 2 and z - SEATED_Z < 0.015


def in_tray(s) -> bool:
    x, y, z = (float(v) for v in s.pos)
    return (abs(x - TRAY_XY[0]) <= TRAY_IN / 2 and abs(y - TRAY_XY[1]) <= TRAY_IN / 2
            and z - (TOP + 0.01 + PART[2] / 2) < 0.015)


def pad_centre(fx):
    return joint_point(fx.pos, PIVOT, "Z", fx.joints.get("clamp", 0.0), (0, NEST_Y, ARM_Z))


def knob(fx):
    return joint_point(fx.pos, PIVOT, "Z", fx.joints.get("clamp", 0.0), (0, PIVOT[1] - KNOB_R, ARM_Z + 0.013 + KNOB_H / 2))


def evaluate(objects, trial):
    fx, fin, blank = objects["fixture"], objects["finished_part"], objects["blank"]
    th = fx.joints.get("clamp", 0.0)
    for s in (fin, blank):
        if float(s.low[2]) < TOP - 0.03:
            trial.defect("a part fell off the table")
    if in_nest(fin) and in_nest(blank):
        trial.defect("blank loaded over the finished part")
    opened = trial.once("opened", th > OPEN_ANGLE)
    tray = in_tray(fin) and trial.still("finished_part")
    loaded = in_nest(blank) and not in_nest(fin) and trial.still("blank")
    clamped = loaded and abs(th) < math.radians(5)
    trial.info.update(clamp=th, opened=opened, tray=tray, loaded=loaded, clamped=clamped)
    done = (opened and tray and loaded and clamped and trial.still("fixture")
            and trial.clear_of([blank.pos, fin.pos, pad_centre(fx), knob(fx)], 0.04))
    return 0.2 * (opened + tray + loaded + clamped), done


TRIAL = Trial(evaluate, trace_env="CLAMP_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    b = np.asarray(objects["blank"].pos, float)
    return float(np.linalg.norm(b - np.array((NEST_C[0], NEST_C[1], SEATED_Z))) + abs(objects["fixture"].joints.get("clamp", 0.0)))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. Left arm, closed fingers: push the clamp's knob round from behind (finger tips trailing 1 cm
# behind it, the path carried 20 degrees past each stop because the arm lags) until the arm stands forward against its stop; later, push it back round until the arm lies over the
# blank. Right arm, pads closing across the 40 mm width
# (yaw +90 degrees): lift the finished part out of the nest to the output tray, then the blank from the table into the
# nest, letting go 1 cm above the plate.

TIP = 0.013
GRIP_ABOVE_BOTTOM = 0.018 + TIP  # tool point 31 mm above the part's bottom: pads on its upper 12 mm
YAW_R = math.pi / 2
CARRY = TOP + PLATE + 0.075 + GRIP_ABOVE_BOTTOM  # part bottom 7.5 cm above the plate, clear of the nest walls


def _clamp_moves():
    kp = (0.0, PIVOT[1] - KNOB_R, ARM_Z + 0.013 + KNOB_H - 0.01)  # on the knob, 1 cm below its top
    # open: push the knob round, from the arms' side, until the arm stands forward against its stop
    out = arc(FIX_POS, PIVOT, "Z", kp, 0.0, OPEN_STOP + 0.35, 12, standoff=0.010, lift=0.0)
    opening = [Waypoint(1.4, tuple(out[0] + np.array((0, 0, 0.05))), FINGER_CLOSED, "over the clamp knob"),
               Waypoint(0.7, tuple(out[0]), FINGER_CLOSED, "behind the knob")]
    opening += [Waypoint(0.25, tuple(p), FINGER_CLOSED, "swing the clamp open") for p in out[1:]]
    opening += [Waypoint(0.6, tuple(out[-1] + np.array((0, 0, 0.05))), FINGER_CLOSED, "lift off")]
    # close: push the knob back round until the arm lies over the blank against its stop
    back = arc(FIX_POS, PIVOT, "Z", kp, OPEN_STOP, -0.35, 12, standoff=0.010, lift=0.0)
    closing = [Waypoint(1.4, tuple(back[0] + np.array((0, 0, 0.05))), FINGER_CLOSED, "over the swung-out knob"),
               Waypoint(0.7, tuple(back[0]), FINGER_CLOSED, "beside the knob")]
    closing += [Waypoint(0.25, tuple(p), FINGER_CLOSED, "swing the clamp over the blank") for p in back[1:]]
    closing += [Waypoint(0.7, tuple(back[-1] + np.array((0, 0, 0.05))), FINGER_CLOSED, "lift clear")]
    return opening, closing

def _carry(frm, frm_z, to, to_z, label):
    return [
        Waypoint(1.4, (*frm, CARRY), FINGER_OPEN, f"over the {label}", yaw=YAW_R),
        Waypoint(1.0, (*frm, frm_z), FINGER_OPEN, "descend", yaw=YAW_R),
        Waypoint(0.7, (*frm, frm_z), FINGER_CLOSED, "grip", yaw=YAW_R),
        Waypoint(1.0, (*frm, CARRY), FINGER_CLOSED, "lift", yaw=YAW_R),
        Waypoint(1.6, (*to, CARRY), FINGER_CLOSED, "carry", yaw=YAW_R),
        Waypoint(0.9, (*to, to_z), FINGER_CLOSED, "lower", yaw=YAW_R),
        Waypoint(0.6, (*to, to_z), FINGER_OPEN, "let go", yaw=YAW_R),
        Waypoint(0.7, (*to, CARRY), FINGER_OPEN, "rise", yaw=YAW_R),
    ]


def demo():
    opening, closing = _clamp_moves()
    nest = (float(NEST_C[0]), float(NEST_C[1]))
    in_nest_z = TOP + PLATE + GRIP_ABOVE_BOTTOM
    unload = _carry(nest, in_nest_z, TRAY_XY, TOP + 0.01 + 0.01 + TRAY_LIP + GRIP_ABOVE_BOTTOM, "finished part")
    load = _carry(BLANK_START, TOP + GRIP_ABOVE_BOTTOM, nest, in_nest_z + 0.01, "blank")
    rest_l, rest_r = (0.12, 0.25, TOP + 0.16), (0.12, -0.28, TOP + 0.16)
    t_open = sum(w.duration for w in opening)
    t_parts = sum(w.duration for w in unload + load)
    right = [Waypoint(t_open + 0.5, rest_r, FINGER_OPEN, "wait for the clamp to open", yaw=YAW_R)] + unload + load
    right += [Waypoint(1.2, rest_r, FINGER_OPEN, "clear", yaw=YAW_R),
              Waypoint(sum(w.duration for w in closing) + 3.0, rest_r, FINGER_OPEN, "wait", yaw=YAW_R)]
    left = opening + [Waypoint(1.0, rest_l, FINGER_CLOSED, "clear"),
                      Waypoint(t_parts + 1.5, rest_l, FINGER_CLOSED, "wait for the swap")]
    left += closing + [Waypoint(1.2, rest_l, FINGER_CLOSED, "clear"), Waypoint(2.5, rest_l, FINGER_CLOSED, "wait")]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    half = np.array(PART) / 2
    nest = (NEST_C[0], NEST_C[1], SEATED_Z)
    tray = (TRAY_XY[0], TRAY_XY[1], TOP + 0.01 + PART[2] / 2)
    table = (*BLANK_START, TOP + PART[2] / 2)

    def st(clamp, fin, blank):
        return {"fixture": fake_state(FIX_POS, (0.06, 0.12, 0.05), {"clamp": clamp}),
                "finished_part": fake_state(fin, half), "blank": fake_state(blank, half)}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    opened = [st(1.5, nest, table)] * 3
    return [
        ("nothing done", [st(0.0, nest, table)] * 30, far, 0.0, False),
        ("clamp opened, nothing moved", opened + [st(1.5, nest, table)] * 27, far, 0.2, False),
        ("opened, finished part in the tray", opened + [st(1.5, tray, table)] * 27, far, 0.4, False),
        ("opened, swapped, clamp still open", opened + [st(1.5, tray, nest)] * 27, far, 0.6, False),
        ("full changeover, held 1 s", opened + [st(0.0, tray, nest)] * 30, far, 1.0, True),
        ("full changeover 0.75 s before the limit", opened + [st(0.0, tray, nest)] * 15, far, 0.8, True),
        ("clamp swung back but 9 degrees short", opened + [st(0.16, tray, nest)] * 30, far, 0.6, False),
        ("blank loaded over the finished part", opened + [st(1.5, nest, (nest[0], nest[1], nest[2] + 0.004))] * 5
         + [st(0.0, tray, nest)] * 30, far, 0.0, False),
        ("finished part dropped off the table", opened + [st(0.0, (0.3, -0.6, TOP - 0.3), nest)] * 30, far, 0.0, False),
        ("never opened (blank forced in)", [st(0.0, tray, nest)] * 30, far, 0.6, False),
    ]
