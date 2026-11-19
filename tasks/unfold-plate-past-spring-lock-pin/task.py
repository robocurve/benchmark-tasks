"""On the station table, between the arms: a hand truck's folding nose-plate extension, standing folded up on its
hinge and locked by a spring pop-pin that runs along the hinge line through a lug on its left edge; the pin's
T-handle is on the outside. Nothing else is on the table."""

# Unfolding a hand truck's nose-plate extension before moving a long load is a routine operation for laborers and
# freight, stock and material movers (about 3.0 million US workers, BLS OEWS May 2023, 53-7062): pull the spring pin
# out and hold it, fold the extension down flat, let the pin go.
#
# Physical difficulty (bimanual): the pin is spring-loaded back through the extension's lug, across the lug's swing.
# One arm must hook the pin's T-handle and hold it pulled against its spring while the other pushes the 17 cm-wide
# plate over and down through a 90-degree arc; released too early, the pin springs back into the lug's path.
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
INSTRUCTION = "Unfold the nose-plate extension: pull the pin out and hold it, fold the plate down flat, then let go."

# -- the scene -------------------------------------------------------------------------------------------------------

STEEL = (0.35, 0.36, 0.38)
RED = (0.72, 0.12, 0.10)
ZINC = (0.75, 0.76, 0.78)
POS = (0.24, 0.0, TOP)  # part frame origin: below the hinge, on the table
BASE_H = 0.03  # the truck's nose plate (fixed) the extension folds onto
HZ = BASE_H + 0.006  # hinge height
PL_LEN, PL_W, PL_T = 0.14, 0.17, 0.01
LUG_R = 0.07  # the side lug's pin hole: 7 cm along the plate from the hinge
LUG_H = 0.020  # ... and 2 cm off the plate's face (towards the arms when the plate stands up)
LUG_T, LUG_HOLE = 0.006, 0.014  # lug: 6 mm steel, 14 mm square hole
LUG_Y = PL_W / 2 + LUG_T / 2  # the lug stands on the plate's left edge
STOWED = -math.pi / 2
PIN_X = -LUG_H - PL_T / 2  # pin line (part frame) when the plate stands up: in front of its face ...
PIN_Z = HZ + LUG_R  # ... level with the lug's hole
PIN_HALF = 0.005  # 10 mm square pin
PIN_TIP = LUG_Y - LUG_T / 2 - 0.006  # pin tip y when pushed in: 6 mm inboard of the lug
GUIDE_Y = LUG_Y + 0.016  # guide bracket on the frame, just outboard of the lug
HANDLE_Y = GUIDE_Y + 0.044  # T-handle: a 3.3 cm gap outboard of the guide for the fingers
PULL = 0.025  # pin travel (outwards, +y)
SPRING = 40.0  # N/m: holding the pin out takes about 1 N


def _plate_boxes():
    """The plate at rest position (flat, reaching +x from the hinge), with the lug standing up from its left edge."""
    z = HZ
    out = [((PL_LEN / 2, 0, z), (PL_LEN, PL_W, PL_T), RED)]
    zc = z + PL_T / 2 + LUG_H  # lug hole centre, off the plate's top face
    h, w = LUG_HOLE / 2, 0.012  # hole half-size, ring width
    ring = [((LUG_R, LUG_Y, zc + h + w / 2), (2 * (h + w), LUG_T, w)),  # far from the plate
            ((LUG_R, LUG_Y, (z + PL_T / 2 + zc - h) / 2), (2 * (h + w), LUG_T, zc - h - z - PL_T / 2)),  # web to plate
            ((LUG_R - h - w / 2, LUG_Y, zc), (w, LUG_T, 2 * h)),
            ((LUG_R + h + w / 2, LUG_Y, zc), (w, LUG_T, 2 * h))]
    out += [(c, sz, RED) for c, sz in ring]
    return out


PLATE_J = Joint("plate", kind="hinge", axis="Y", anchor=(0, 0, HZ), lower=STOWED - 0.08, upper=0.0, initial=STOWED,
                target=STOWED, friction=0.03, damping=0.02, mass=0.5, boxes=_plate_boxes())
# the pin slides along y (along the hinge line), so the lug, which swings in the x-z plane, cannot pass it
_PIN_C = ((PIN_TIP + HANDLE_Y) / 2)
PIN_J = Joint("pin", kind="slide", axis="Y", anchor=(PIN_X, HANDLE_Y, PIN_Z), lower=0.0, upper=PULL, initial=0.0,
              target=0.0, stiffness=SPRING, damping=2.0, mass=0.04,
              boxes=[((PIN_X, _PIN_C, PIN_Z), (2 * PIN_HALF, HANDLE_Y - PIN_TIP, 2 * PIN_HALF), ZINC),
                     ((PIN_X, HANDLE_Y, PIN_Z), (0.035, 0.01, 0.012), ZINC)])  # T-handle, across the pin

_GW = 0.008  # guide bracket wall
TRUCK = part("nose_plate", pos=POS, fixed=True, self_collide=True, joints=[PLATE_J, PIN_J], boxes=[
    ((0.075, 0, BASE_H / 2), (0.17, 0.19, BASE_H), STEEL),  # the truck's own nose plate, under the extension
    ((PIN_X, GUIDE_Y, (PIN_Z - PIN_HALF - 0.002) / 2), (0.03, _GW, PIN_Z - PIN_HALF - 0.002), STEEL),  # bracket post
    ((PIN_X, GUIDE_Y, PIN_Z + PIN_HALF + 0.002 + 0.005), (0.03, _GW, 0.01), STEEL),  # bracket cap over the pin
    ((PIN_X - PIN_HALF - 0.002 - 0.006, GUIDE_Y, PIN_Z), (0.012, _GW, 2 * PIN_HALF + 0.004), STEEL),  # cheeks
    ((PIN_X + PIN_HALF + 0.002 + 0.006, GUIDE_Y, PIN_Z), (0.012, _GW, 2 * PIN_HALF + 0.004), STEEL),
])
OBJECTS = [TRUCK]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# The pin is pulled once it has been drawn at least 2 cm out of its 2.5 cm travel; the plate is down when its hinge is
# within 3 degrees of flat. Score at the end of the trial: 0.3 once the pin has been pulled (at any time) plus 0.3 for
# the plate down (0 to 0.6). 1.0 when the plate is down, the pin has been let go (back within 5 mm of home), nothing
# moves and no gripper tool point is within 4 cm of the plate's middle or the pin's handle; held for 1 s. Completed
# less than 1 s before the run limit: 0.6.

DOWN = math.radians(3)


def handle(s):
    return np.asarray(s.pos) + np.array((PIN_X, HANDLE_Y + s.joints.get("pin", 0.0), PIN_Z))


def plate_mid(s):
    return joint_point(s.pos, PLATE_J.anchor, "Y", s.joints.get("plate", STOWED), (PL_LEN / 2, 0, HZ))


def evaluate(objects, trial):
    s = objects["nose_plate"]
    th, pin = s.joints.get("plate", STOWED), s.joints.get("pin", 0.0)
    pulled = trial.once("pulled", pin > 0.02)
    down = abs(th) < DOWN
    trial.info.update(plate=th, pin=pin)
    done = (down and pin < 0.005 and trial.still("nose_plate")
            and trial.clear_of([plate_mid(s), handle(s)], 0.04))
    return 0.3 * pulled + 0.3 * down, done


TRIAL = Trial(evaluate, trace_env="PLATE_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    return float(abs(objects["nose_plate"].joints.get("plate", STOWED)))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, closed fingers. The left arm lowers its closed fingers into the gap between the guide bracket and
# the pin's T-handle and draws the handle 3.3 cm outwards (past the 2.5 cm stop: the arm lags under load),
# holding the pin out. The right arm then pushes the plate's near face 6.5 cm right of centre over and down through its
# arc (finger tips trailing 1.3 cm behind a point 11 cm up the plate) until it lies flat, and lifts off; then the left
# arm lifts away and the spring returns the pin.

YAW_L = 0.0  # left pads close along y: the closed fingers are thin along y, to fit the 3.5 cm gap


def demo():
    gap_y = (GUIDE_Y + 0.004 + HANDLE_Y - 0.005) / 2  # middle of the gap between the guide bracket and the T-handle
    hx = np.asarray(POS) + np.array((PIN_X, gap_y, PIN_Z))
    pulled = hx + np.array((0, PULL + 0.008, 0.004))
    left = [Waypoint(1.5, tuple(hx + np.array((0, 0, 0.06))), FINGER_CLOSED, "above the pin's T-handle", yaw=YAW_L),
            Waypoint(0.9, tuple(hx + np.array((0, 0, 0.004))), FINGER_CLOSED, "fingers inside the handle", yaw=YAW_L),
            Waypoint(1.2, tuple(pulled), FINGER_CLOSED, "pull the pin out", yaw=YAW_L)]
    path = arc(POS, PLATE_J.anchor, "Y", (0.11, -0.065, HZ + PL_T / 2), STOWED, 0.2 * (1 if STOWED < 0 else -1), 12, standoff=0.010, lift=0.0)
    start = path[0] + np.array((-0.03, 0, 0.02))
    t_pull = sum(w.duration for w in left)
    right = [Waypoint(t_pull - 0.2, (0.12, -0.25, TOP + 0.16), FINGER_CLOSED, "wait for the pin"),
             Waypoint(1.0, tuple(start), FINGER_CLOSED, "in front of the standing plate"),
             Waypoint(0.6, tuple(path[0]), FINGER_CLOSED, "on the plate's face")]
    right += [Waypoint(0.28, tuple(p), FINGER_CLOSED, "fold the plate over and down") for p in path]
    right += [Waypoint(0.5, tuple(path[-1] + np.array((0, 0, -0.004))), FINGER_CLOSED, "press it flat"),
              Waypoint(0.7, tuple(path[-1] + np.array((0, 0, 0.07))), FINGER_CLOSED, "lift off"),
              Waypoint(1.2, (0.12, -0.25, TOP + 0.16), FINGER_CLOSED, "clear"),
              Waypoint(4.0, (0.12, -0.25, TOP + 0.16), FINGER_CLOSED, "wait")]
    t_fold = sum(w.duration for w in right[:-2])
    left += [Waypoint(max(0.5, t_fold - t_pull), tuple(pulled), FINGER_CLOSED, "hold the pin out", yaw=YAW_L),
             Waypoint(0.6, tuple(pulled + np.array((0, 0, 0.066))), FINGER_CLOSED, "let the pin go", yaw=YAW_L),
             Waypoint(1.2, (0.12, 0.25, TOP + 0.16), FINGER_CLOSED, "clear", yaw=YAW_L),
             Waypoint(3.0, (0.12, 0.25, TOP + 0.16), FINGER_CLOSED, "wait", yaw=YAW_L)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(plate, pin):
        return {"nose_plate": fake_state(POS, (0.08, 0.08, 0.05), {"plate": plate, "pin": pin})}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    on_handle = lambda: [handle(st(0.0, 0.0)["nose_plate"]), np.array((0.1, -0.3, 1.0))]
    pulled = [st(STOWED, 0.024)] * 3
    return [
        ("untouched", [st(STOWED, 0.0)] * 30, far, 0.0, False),
        ("pin pulled, plate still up", pulled + [st(STOWED, 0.024)] * 27, far, 0.3, False),
        ("plate down, pin still held out", pulled + [st(0.0, 0.024)] * 30, far, 0.6, False),
        ("plate down, pin let go, held 1 s", pulled + [st(0.0, 0.0)] * 30, far, 1.0, True),
        ("done 0.75 s before the run limit", pulled + [st(0.0, 0.0)] * 15, far, 0.6, True),
        ("plate down but a gripper still on the handle", pulled + [st(0.0, 0.0)] * 30, on_handle, 0.6, False),
        ("plate stopped 10 degrees short", pulled + [st(-0.17, 0.0)] * 30, far, 0.3, False),
    ]
