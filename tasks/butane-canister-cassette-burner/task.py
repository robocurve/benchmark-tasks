"""Load a butane canister into a cassette burner. A portable cassette gas burner (34.5 x 28.2 x 8 cm, 1.6 kg) stands on
the table with its canister compartment on its right. A full 250 g cassette canister (66 mm across, 190 mm long,
0.36 kg) lies on the table to the right of it, its notch turned 35 degrees away from the top, toward the right.

The compartment holds the canister along two rails. At the back is the burner's gas inlet: a round socket with a guide
slot at its top, so the canister's notch (here a lug on its mounting cup) has to face straight up for the nozzle to go
in. Inside the socket a spring plunger stands in for the inlet's seal and magnet coupling: pushed home against it the
canister is connected, let go it is pushed back out of the seal. At the front, a locking lever (a bar slid across
behind the canister's base) holds a canister that is pushed home, and cannot be moved across one that is not.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from burnerkit import PartState, Slider, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Load the new gas canister into the burner so that the burner is ready to light."

BLACK = (0.12, 0.12, 0.13)
STEEL = (0.70, 0.71, 0.73)
GREY = (0.45, 0.46, 0.48)
ORANGE = (1.00, 0.45, 0.05)
CAN_RED = (0.80, 0.12, 0.10)
CAN_CUP = (0.82, 0.82, 0.84)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the canister, in its own frame: origin at its centre, axis along x, nozzle end toward +x ---------------------------
# A standard 250 g butane cassette canister: 66 mm across, 190 mm long (170 mm body and a 15 mm mounting cup with the
# nozzle), 0.36 kg full. The notch on the mounting cup's rim is modelled as a lug (10 x 10 mm, standing 8 mm off the
# cup) that must pass through the inlet's guide slot; in this frame it points toward -y.
CAN_R = 0.033
CAN_MASS = 0.36
CUP_R = 0.020
BASE_X = -0.095            # the canister's base, canister frame
NOSE_X = 0.090             # the end of the mounting cup
LUG_DIR = (0.0, -1.0, 0.0)  # the notch's direction, canister frame


def _can_cylinders():
    return [((-0.010, 0.0, 0.0), CAN_R, 0.170, CAN_RED, "X"),          # body
            ((0.0825, 0.0, 0.0), CUP_R, 0.015, CAN_CUP, "X")]          # mounting cup and nozzle


def _can_boxes():
    return [_mm(75, 87, -28, -18, -5, 5, CAN_CUP)]                      # the notch's lug


# -- the burner, in its own frame: origin on the table under its centre, x forward (away from the arms), y left -------
# Body 282 x 350 mm, 80 mm tall, 1.6 kg, on rubber feet (friction 1.0). The canister compartment runs front to back on the right:
# 100 mm wide, centred on y = COMP_Y, the canister's axis 45 mm above the table on two rails. At the back is the gas
# inlet socket (its face at x = 90 mm), at the front the locking bar.
COMP_Y = -0.125
AXIS_Z = 0.045
SOCKET_X = 0.090           # the inlet socket's face; the canister's shoulder stops against it
HOLE_R = 0.022             # the socket's round opening (the mounting cup is 20 mm in radius)
SLOT_HALF = 0.012          # the guide slot at the top of the opening: 24 mm wide, up to 31 mm above the axis
SLOT_TOP = 0.031
PLUG_REST_X = 0.099        # the plunger's face at rest; the mounting cup's end meets it 1 mm before the locked position
PLUG_TRAVEL = 0.0045
LOCK_TRAVEL = 0.040        # the locking bar slides 40 mm toward -y, across behind the canister's base
LOCK_BACK_X = -0.085       # the bar's back face; a locked canister's base rests against it


def _socket_boxes():
    """The inlet socket: a 51 mm deep block with a round opening of radius 22 mm (built from 4 mm slabs whose opening
    follows the circle) and the guide slot at its top. A canister whose lug is more than about 13 degrees off the top
    meets the socket face with its lug and stops 5 mm short of the nozzle entering the seal."""
    yc, zc = COMP_Y * 1000, AXIS_Z * 1000
    out = [_mm(90, 141, -180, -75, 5, zc - 22, GREY)]                   # below the opening
    for k in range(11):
        z0, z1 = -22 + 4 * k, -18 + 4 * k
        zmin = 0.0 if z0 < 0 < z1 else min(abs(z0), abs(z1))
        w = math.sqrt(22 ** 2 - zmin ** 2)
        if z1 > 18:
            w = max(w, SLOT_HALF * 1000)
        out += [_mm(90, 141, -180, yc - w, zc + z0, zc + z1, GREY), _mm(90, 141, yc + w, -75, zc + z0, zc + z1, GREY)]
    s = SLOT_HALF * 1000
    out += [_mm(90, 141, -180, yc - s, zc + 22, zc + SLOT_TOP * 1000, GREY),
            _mm(90, 141, yc + s, -75, zc + 22, zc + SLOT_TOP * 1000, GREY),
            _mm(90, 141, -180, -75, zc + SLOT_TOP * 1000, 80, GREY)]    # above the slot
    return out


def _stove_boxes():
    yc = COMP_Y * 1000
    return [
        _mm(-141, 141, -180, 170, 0, 5, BLACK),                         # base pan
        _mm(-70, 141, -75, 170, 5, 80, BLACK),                          # body round the burner
        _mm(-141, -96, -75, 170, 5, 45, BLACK),                         # control panel
        _mm(-96, -70, -30, 170, 5, 45, BLACK),                          # deck behind the panel
        _mm(-141, 141, -180, -175, 5, 25, BLACK),                       # compartment's outer wall (low)
        _mm(-141, -136, -175, -75, 5, 25, BLACK),                       # compartment's front wall (low)
        _mm(-130, 87, yc - 23, yc - 17, 5, 16.72, STEEL),               # rails the canister rests on
        _mm(-130, 87, yc + 17, yc + 23, 5, 16.72, STEEL),
        _mm(-25, 95, -10, 5, 80, 88, STEEL),                            # pan supports
        _mm(-25, 95, 89, 104, 80, 88, STEEL),
        _mm(28, 42, -10, 104, 80, 88, STEEL),
    ] + _socket_boxes()


def _stove_cylinders():
    return [((0.035, 0.047, 0.085), 0.040, 0.010, STEEL, "Z"),          # burner head
            ((-0.146, 0.060, 0.025), 0.012, 0.010, GREY, "X")]          # ignition knob


def _plug():
    """The inlet's seal and magnet coupling: a plunger (10 x 16 x 16 mm) in the socket, its face 9 mm inside. The
    mounting cup's end meets it 1 mm before the locked position; it gives 4.5 mm against a 1500 N/m spring, which is as
    far as a canister can be pushed."""
    return Slider("plug", rest=(0.104, COMP_Y, AXIS_Z), boxes=(_mm(99, 109, -133, -117, 37, 53, STEEL),),
                  travel=PLUG_TRAVEL, rate=1500.0, damping=20.0, axis="X", sign=1, mass=0.03)


def _lock():
    """The locking lever: a bar (10 x 50 x 35 mm) that slides 40 mm across behind the canister's base, with an arm
    running from it over the control panel to an orange handle (12 x 15 x 40 mm) standing up at the panel's front
    left. Unlocked the bar sits clear of the compartment; it stays where it is put. A canister that is not pushed home
    blocks it."""
    bar = _mm(-95, -85, -85, -35, 25, 60, STEEL)
    arm_y = _mm(-95, -85, -35, 10, 50, 60, STEEL)
    arm_x = _mm(-125, -95, -5, 10, 50, 60, STEEL)
    handle = _mm(-125, -113, -5, 10, 60, 100, ORANGE)
    return Slider("lock", rest=(-0.100, -0.030, 0.050), boxes=(bar, arm_y, arm_x, handle), travel=LOCK_TRAVEL,
                  rate=0.0, damping=30.0, axis="Y", sign=-1, mass=0.08)


# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
STOVE_XY = (0.33, 0.0)
CAN_XY = (0.30, -0.30)          # the canister lies on the table to the right of the burner, axis along x, nozzle away
CAN_ROLL = math.radians(-55)    # turned about its axis so that its notch points 35 degrees right of straight up
STOVE_MASS = 1.6

STOVE = part("burner", pos=(*STOVE_XY, TOP), mass=STOVE_MASS, boxes=_stove_boxes(), cylinders=_stove_cylinders(),
             sliders=(_plug(), _lock()), color=BLACK, friction=1.0)
CAN = part("canister", pos=(*CAN_XY, TOP + CAN_R), mass=CAN_MASS, boxes=_can_boxes(), cylinders=_can_cylinders(),
           color=CAN_RED, friction=0.35, roll=CAN_ROLL,
           com=(-0.005, 0.0, 0.0))   # on its axis (the lug's few grams don't turn it over)
OBJECTS = [STOVE, CAN]

# -- scoring -----------------------------------------------------------------------------------------------------------
LOCKED_X = LOCK_BACK_X - BASE_X                # 0.010: the canister's centre, locked, along the compartment
HOME_X = (LOCKED_X - 0.0015, LOCKED_X + 0.0040)  # pushed home: locked, or held against the plunger up to the stop
ENGAGED_X = 0.005                              # the lug 2 mm into the guide slot (only possible with the notch up)
PLACED_X = (-0.060, 0.016)
PLACED_YZ = (0.010, 0.006)                     # on the rails: within 10 mm of the compartment's line, 6 mm of its height
LOCKED = -0.035                                # the lever at least 35 of its 40 mm across
NOTCH_UP = 15.0                                # degrees
STOVE_MOVE_LIMIT = 0.030
STOVE_TILT_LIMIT = 10.0
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.pop("_this_step", None)
    _STATE.update(placed=False, engaged=False, home=False, failed=False, recent=[], held=0)


_reset_state()


def in_compartment(can: PartState, stove: PartState) -> np.ndarray:
    """The canister's centre in the burner's frame (m)."""
    return stove.rotation().T @ (np.asarray(can.pos, float) - np.asarray(stove.pos, float))


def _deg(a, b) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))


def notch_angle(can: PartState, stove: PartState) -> float:
    """Degrees between the canister's notch and straight up (the burner's vertical)."""
    return _deg(can.rotation() @ np.array(LUG_DIR), stove.rotation()[:, 2])


def axis_angle(can: PartState, stove: PartState) -> float:
    """Degrees between the canister's axis and the compartment's (the burner's x)."""
    return _deg(can.rotation()[:, 0], stove.rotation()[:, 0])


def stove_moved(stove: PartState) -> float:
    return float(np.linalg.norm(np.asarray(stove.pos[:2], float) - np.array(STOVE_XY)))


def stove_tilt(stove: PartState) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, stove.rotation()[2, 2]))))


def _update(objects) -> tuple[float, bool]:
    can, stove = objects["canister"], objects["burner"]
    _STATE["failed"] |= bool(stove_moved(stove) > STOVE_MOVE_LIMIT or stove_tilt(stove) > STOVE_TILT_LIMIT
                             or can.pos[2] < TOP - 0.05)
    if _STATE["failed"]:
        return 0.0, False
    c = in_compartment(can, stove)
    on_rails = bool(abs(c[1] - COMP_Y) < PLACED_YZ[0] and abs(c[2] - AXIS_Z) < PLACED_YZ[1]
                    and axis_angle(can, stove) < 10)
    _STATE["placed"] |= bool(on_rails and PLACED_X[0] <= c[0] <= PLACED_X[1])
    _STATE["engaged"] |= bool(on_rails and c[0] >= ENGAGED_X)
    home = bool(on_rails and HOME_X[0] <= c[0] <= HOME_X[1] and notch_angle(can, stove) < NOTCH_UP)
    _STATE["home"] |= home
    recent = _STATE["recent"]
    recent.append(c.copy())
    del recent[:-20]
    still = len(recent) == 20 and all(np.linalg.norm(p - c) < 0.001 for p in recent)
    done = bool(home and stove.sliders["lock"] <= LOCKED)
    _STATE["held"] = _STATE["held"] + 1 if done else 0
    if done:   # success once home and locked for 20 steps (1 s) in a row, the canister still over that second
        return 1.0, bool(still and _STATE["held"] >= 20)
    if _STATE["home"]:
        return 0.75, False
    if _STATE["engaged"]:
        return 0.5, False
    return (0.25 if _STATE["placed"] else 0.0), False


def success(objects) -> bool:
    """The canister home in the inlet with its notch up and the lever locked, held for 1 s with the canister still."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[1]                            # score() reuses this, so the step counts once


def score(objects) -> float:
    """0.25 canister on the compartment's rails; 0.5 nozzle into the inlet (notch through the guide); 0.75 pushed home;
    1.0 home and locked. 0 if the burner was knocked 3 cm or tipped 10 degrees, or the canister fell off the table."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
