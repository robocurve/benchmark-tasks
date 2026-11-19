"""On the station table, between the arms: a point-of-sale receipt printer with its lid standing open and the spent
paper core lying in its roll pocket. A new paper roll lies in a supply tray on the left, a waste tray stands on the
right. Nothing else is on the table."""

# Changing the receipt printer's paper roll is a core, repeated operation for cashiers (about 3.3 million US workers,
# BLS OEWS May 2023, 41-2011): take the spent core out, drop the new roll into the pocket, close the lid.
#
# Physical difficulty: the spent core is lifted out of a pocket only 1 cm wider on each side than a fresh roll; the
# 6 cm roll is lowered into it lying on its side; the hinged lid, standing 110 degrees open towards the arms, is pushed
# over and down through its arc. The lid only closes over a seated roll with the core gone. Both arms work: the right
# clears the core, the left loads the roll and shuts the lid.
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
INSTRUCTION = "Change the printer's paper roll: put the empty core in the waste tray, drop in the new roll and shut the lid."

# -- the scene -------------------------------------------------------------------------------------------------------

BODY = (0.15, 0.15, 0.17)
LID_C = (0.22, 0.22, 0.24)
PAPER = (0.95, 0.95, 0.92)
CORE_C = (0.70, 0.58, 0.42)
GREY = (0.55, 0.57, 0.60)

PR_POS = (0.26, 0.0, TOP)  # printer: part frame origin at its footprint centre, on the table
PR = (0.14, 0.12, 0.07)  # printer body: 14 cm deep x 12 cm wide x 7 cm tall
POCKET = (0.08, 0.075, 0.062)  # roll pocket: 8 cm (forward) x 7.5 cm (across) x 6.2 cm deep, under the lid
ROLL_R, ROLL_W, ROLL_MASS = 0.03, 0.058, 0.15  # 60 mm thermal roll, 58 mm wide
CORE_R, CORE_MASS = 0.011, 0.01
WALL = 0.008
POCKET_FLOOR = PR[2] - POCKET[2]
LID_LEN, LID_T = 0.13, 0.008
HINGE = (-PR[0] / 2, 0.0, PR[2])  # lid hinge along the printer's near top edge
LID_OPEN = -1.92  # rad: 110 degrees open, leaning back towards the arms


def _body_boxes():
    px, py, pz = POCKET
    h = PR[2]
    out = [((0, 0, POCKET_FLOOR / 2), (PR[0], PR[1], POCKET_FLOOR), BODY)]
    out.append(((-(px / 2 + (PR[0] / 2 - px / 2) / 2), 0, POCKET_FLOOR + pz / 2), (PR[0] / 2 - px / 2, PR[1], pz), BODY))
    out.append((((px / 2 + (PR[0] / 2 - px / 2) / 2), 0, POCKET_FLOOR + pz / 2), (PR[0] / 2 - px / 2, PR[1], pz), BODY))
    out.append(((0, py / 2 + (PR[1] / 2 - py / 2) / 2, POCKET_FLOOR + pz / 2), (px, PR[1] / 2 - py / 2, pz), BODY))
    out.append(((0, -(py / 2 + (PR[1] / 2 - py / 2) / 2), POCKET_FLOOR + pz / 2), (px, PR[1] / 2 - py / 2, pz), BODY))
    return out


LID = Joint("lid", kind="hinge", axis="Y", anchor=HINGE, lower=LID_OPEN - 0.05, upper=0.0, initial=LID_OPEN,
            target=LID_OPEN, friction=0.04, damping=0.01, mass=0.06,
            boxes=[((HINGE[0] + LID_LEN / 2, 0, PR[2] + LID_T / 2), (LID_LEN, PR[1] - 0.004, LID_T), LID_C)])
PRINTER = part("printer", pos=PR_POS, fixed=True, joints=[LID], boxes=_body_boxes())

CORE_POS = (PR_POS[0], PR_POS[1], TOP + POCKET_FLOOR + CORE_R)
CORE = part("spent_core", pos=CORE_POS, mass=CORE_MASS, color=CORE_C,
            cylinders=[((0, 0, 0), CORE_R, ROLL_W, CORE_C, "Y")])

SUPPLY_XY = (0.16, 0.24)
SUPPLY = part("supply_tray", pos=(*SUPPLY_XY, TOP), fixed=True, boxes=[
    ((0, 0, 0.004), (0.10, 0.09, 0.008), GREY),
    ((-0.047, 0, 0.018), (0.006, 0.09, 0.02), GREY), ((0.047, 0, 0.018), (0.006, 0.09, 0.02), GREY),
])
ROLL_POS = (SUPPLY_XY[0], SUPPLY_XY[1], TOP + 0.008 + ROLL_R)
ROLL = part("paper_roll", pos=ROLL_POS, mass=ROLL_MASS, color=PAPER,
            cylinders=[((0, 0, 0), ROLL_R, ROLL_W, PAPER, "Y"), ((0, 0, 0), CORE_R, ROLL_W + 0.002, CORE_C, "Y")])

WASTE_XY, WASTE_IN = (0.18, -0.25), 0.10
WASTE = part("waste_tray", pos=(*WASTE_XY, TOP), fixed=True, boxes=[
    ((0, 0, 0.004), (WASTE_IN + 0.012, WASTE_IN + 0.012, 0.008), GREY),
    ((-WASTE_IN / 2 - 0.003, 0, 0.018), (0.006, WASTE_IN + 0.012, 0.02), GREY),
    ((WASTE_IN / 2 + 0.003, 0, 0.018), (0.006, WASTE_IN + 0.012, 0.02), GREY),
    ((0, WASTE_IN / 2 + 0.003, 0.018), (WASTE_IN, 0.006, 0.02), GREY),
    ((0, -WASTE_IN / 2 - 0.003, 0.018), (WASTE_IN, 0.006, 0.02), GREY),
])
OBJECTS = [PRINTER, CORE, SUPPLY, ROLL, WASTE]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# The core is binned when its centre lies inside the waste tray's inside square less than 2 cm above its floor. The
# roll is loaded when its centre lies inside the pocket seen from above and less than 1 cm above where it rests on the
# pocket floor, lying on its side (its axis within 10 degrees of across). The lid is shut within 3 degrees of flat.
#   0.2 [core binned, still] + 0.2 [roll loaded, still] + 0.2 [lid shut] (0 to 0.6), at the end of the trial;
#   1.0 when all three hold, nothing moves, and no gripper tool point is within 4 cm of the lid's middle, held 1 s.
#   Completed less than 1 s before the run limit: 0.6.
# Defect that scores 0: the roll or the core falling off the table.


def _axis_y_dev(s) -> float:
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    ay = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])  # body y axis in world
    return math.degrees(math.acos(min(1.0, abs(ay[1]))))


def roll_loaded(s) -> bool:
    x, y, z = (float(v) for v in s.pos)
    return (abs(x - PR_POS[0]) <= POCKET[0] / 2 and abs(y - PR_POS[1]) <= POCKET[1] / 2
            and z - (TOP + POCKET_FLOOR + ROLL_R) < 0.01 and _axis_y_dev(s) < 10)


def core_binned(s) -> bool:
    x, y, z = (float(v) for v in s.pos)
    return abs(x - WASTE_XY[0]) <= WASTE_IN / 2 and abs(y - WASTE_XY[1]) <= WASTE_IN / 2 and z < TOP + 0.008 + 0.02 + CORE_R


def lid_mid(p):
    return joint_point(p.pos, HINGE, "Y", p.joints.get("lid", LID_OPEN), (HINGE[0] + LID_LEN / 2, 0, PR[2] + LID_T))


def evaluate(objects, trial):
    p, roll, core = objects["printer"], objects["paper_roll"], objects["spent_core"]
    for s in (roll, core):
        if float(s.low[2]) < TOP - 0.03:
            trial.defect("fell off the table")
    th = p.joints.get("lid", LID_OPEN)
    binned = core_binned(core) and trial.still("spent_core")
    loaded = roll_loaded(roll) and trial.still("paper_roll")
    shut = abs(th) < math.radians(3)
    trial.info.update(lid=th, binned=binned, loaded=loaded)
    done = binned and loaded and shut and trial.still("printer") and trial.clear_of([lid_mid(p)], 0.04)
    return 0.2 * (binned + loaded + shut), done


TRIAL = Trial(evaluate, trace_env="PRINTER_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    r = np.asarray(objects["paper_roll"].pos, float)
    return float(np.linalg.norm(r - np.array((PR_POS[0], PR_POS[1], TOP + POCKET_FLOOR + ROLL_R)))
                 + abs(objects["printer"].joints.get("lid", LID_OPEN)) * 0.05)


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. Pads close along the forward axis, across the core's and roll's diameters (yaw -90 degrees on
# the left arm, +90 on the right). The right arm reaches into the pocket, lifts the spent core out and drops it into
# the waste tray. The left arm lifts the new roll from the supply tray, carries it over the pocket with its bottom 3 cm
# clear of the printer, lowers it until the finger tips are 6 mm above the printer's top and lets it drop the last
# 1 cm; then, fingers closed, it pushes the open lid over and down through its arc (finger tips trailing 1.3 cm behind
# a point 9 cm out on the lid) and presses it shut.

TIP = 0.013
YAW_L, YAW_R = -math.pi / 2, math.pi / 2
TOP_Z = TOP + PR[2]


def demo():
    core_z = CORE_POS[2]
    right = [Waypoint(1.5, (CORE_POS[0], CORE_POS[1], TOP_Z + 0.05), FINGER_OPEN * 0.55, "over the spent core", yaw=YAW_R),
             Waypoint(1.2, (CORE_POS[0], CORE_POS[1], core_z), FINGER_OPEN * 0.55, "reach into the pocket", yaw=YAW_R),
             Waypoint(0.7, (CORE_POS[0], CORE_POS[1], core_z), FINGER_CLOSED, "grip the core", yaw=YAW_R),
             Waypoint(1.2, (CORE_POS[0], CORE_POS[1], TOP_Z + 0.05), FINGER_CLOSED, "lift it out", yaw=YAW_R),
             Waypoint(1.5, (*WASTE_XY, TOP + 0.08), FINGER_CLOSED, "over the waste tray", yaw=YAW_R),
             Waypoint(0.6, (*WASTE_XY, TOP + 0.08), FINGER_OPEN * 0.55, "drop it", yaw=YAW_R),
             Waypoint(1.2, (0.12, -0.27, TOP + 0.16), FINGER_OPEN, "clear", yaw=YAW_R)]
    t_core = sum(w.duration for w in right[:4])
    carry = TOP_Z + 0.03 + ROLL_R
    release = TOP_Z + 0.006 + TIP
    rx, ry = PR_POS[0], PR_POS[1]
    left = [Waypoint(1.5, (*SUPPLY_XY, carry), FINGER_OPEN * 0.85, "over the new roll", yaw=YAW_L),
            Waypoint(1.0, ROLL_POS, FINGER_OPEN * 0.85, "descend", yaw=YAW_L),
            Waypoint(0.7, ROLL_POS, FINGER_CLOSED, "grip the roll", yaw=YAW_L),
            Waypoint(1.0, (*SUPPLY_XY, carry), FINGER_CLOSED, "lift", yaw=YAW_L),
            Waypoint(max(0.5, t_core + 0.3 - 4.2), (*SUPPLY_XY, carry), FINGER_CLOSED, "wait for the core", yaw=YAW_L),
            Waypoint(1.6, (rx, ry, carry), FINGER_CLOSED, "over the pocket", yaw=YAW_L),
            Waypoint(0.8, (rx, ry, release), FINGER_CLOSED, "lower", yaw=YAW_L),
            Waypoint(0.6, (rx, ry, release), FINGER_OPEN * 0.85, "let go", yaw=YAW_L),
            Waypoint(0.7, (rx, ry, carry), FINGER_OPEN * 0.85, "rise", yaw=YAW_L)]
    path = arc(PR_POS, HINGE, "Y", (HINGE[0] + 0.09, 0.02, PR[2] + LID_T), LID_OPEN, 0.2 * (1 if LID_OPEN < 0 else -1), 12, standoff=0.010, lift=0.0)
    behind = path[0] + np.array((-0.03, 0, 0.03))
    side_y = PR[1] / 2 + 0.045  # beside the standing lid, clear of its edge
    left += [Waypoint(0.7, (rx, side_y, behind[2]), FINGER_CLOSED, "out to the side of the lid", yaw=YAW_L),
             Waypoint(1.0, (behind[0], side_y, behind[2]), FINGER_CLOSED, "round behind the lid", yaw=YAW_L),
             Waypoint(0.7, tuple(behind), FINGER_CLOSED, "behind the open lid", yaw=YAW_L),
             Waypoint(0.6, tuple(path[0]), FINGER_CLOSED, "on the lid", yaw=YAW_L)]
    left += [Waypoint(0.25, tuple(p), FINGER_CLOSED, "push the lid over and down", yaw=YAW_L) for p in path]
    left += [Waypoint(0.5, tuple(path[-1] + np.array((0, 0, -0.004))), FINGER_CLOSED, "press it shut", yaw=YAW_L),
             Waypoint(0.7, tuple(path[-1] + np.array((0, 0, 0.06))), FINGER_CLOSED, "lift off", yaw=YAW_L),
             Waypoint(1.2, (0.12, 0.27, TOP + 0.16), FINGER_CLOSED, "clear", yaw=YAW_L),
             Waypoint(3.0, (0.12, 0.27, TOP + 0.16), FINGER_CLOSED, "wait", yaw=YAW_L)]
    right += [Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right), (0.12, -0.27, TOP + 0.16),
                       FINGER_OPEN, "wait", yaw=YAW_R)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    loaded = (PR_POS[0], PR_POS[1], TOP + POCKET_FLOOR + ROLL_R)
    binned = (WASTE_XY[0], WASTE_XY[1], TOP + 0.008 + CORE_R)

    def st(lid, roll, core, roll_quat=(0, 0, 0, 1.0)):
        return {"printer": fake_state(PR_POS, (0.07, 0.06, 0.035), {"lid": lid}),
                "paper_roll": fake_state(roll, (ROLL_R, ROLL_W / 2, ROLL_R), quat=roll_quat),
                "spent_core": fake_state(core, (CORE_R, ROLL_W / 2, CORE_R))}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    s45 = math.sin(math.pi / 4)
    return [
        ("untouched", [st(LID_OPEN, ROLL_POS, CORE_POS)] * 30, far, 0.0, False),
        ("core binned", [st(LID_OPEN, ROLL_POS, binned)] * 30, far, 0.2, False),
        ("core binned, roll loaded, lid open", [st(LID_OPEN, loaded, binned)] * 30, far, 0.4, False),
        ("all done, held 1 s", [st(0.0, loaded, binned)] * 30, far, 1.0, True),
        ("all done 0.75 s before the run limit", [st(0.0, loaded, binned)] * 15, far, 0.6, True),
        ("roll standing on end in the pocket", [st(0.0, loaded, binned, (s45, 0, 0, s45))] * 30, far, 0.4, False),
        ("lid shut with the core still in", [st(0.0, ROLL_POS, CORE_POS)] * 30, far, 0.2, False),
        ("roll dropped off the table", [st(0.0, (0.3, 0.6, TOP - 0.3), binned)] * 30, far, 0.0, False),
    ]
