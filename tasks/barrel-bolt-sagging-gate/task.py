"""Bolt a sagging garden gate shut with its barrel bolt. The latch end of a shut, boarded timber gate stands beside its
gate post. Its hinges have worn, so the gate has dropped 7 mm: the 150 mm barrel bolt on its lock rail no longer lines
up with the keeper on the post, and slid across it only butts against the keeper's lower strap.

The gate's free end has to be lifted and held while the bolt is slid across into the keeper, until it stops; then the
bolt's handle is turned down into the locking notch at the end of its slot, so the bolt cannot slide back.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from gatekit import Link, PartState, link_poses, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Bolt the gate shut."

CEDAR = (0.62, 0.42, 0.27)
CEDAR_DARK = (0.50, 0.33, 0.21)
POST_GREY = (0.47, 0.40, 0.33)
STEEL = (0.20, 0.21, 0.22)      # black-japanned steel bolt hardware
ZINC = (0.72, 0.73, 0.75)       # the bolt itself
KNOB = (0.10, 0.10, 0.11)


def _mm(x0, x1, y0, y1, z0, z1, color):
    """A box from world extents in mm (x forward, y left, z up from the table top)."""
    return (np.array([(x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000]),
            ((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000), color)


# -- layout, world frame: x forward from the arms, y to the robot's left, heights above the table top (m) -------------
# The gate stands across the table 300 mm in front of the arm bases, its face toward the robot. Its free (latch) end is
# on the robot's right, 10 mm from the gate post; its hinges are 900 mm away to the left, off the table.
GATE_FRONT_X = 0.300            # the gate's face (and the post's face) toward the robot
GATE_T = 0.045                  # frame thickness (45 x 70 mm cedar stile and lock rail)
GATE_EDGE_Y = -0.030            # the gate's free edge
GATE_END_Y = 0.330              # where the section shown ends (toward the hinges)
GATE_WIDTH = 0.900              # free edge to hinge line
PIVOT = np.array([GATE_FRONT_X + GATE_T / 2, GATE_EDGE_Y + GATE_WIDTH, 0.075])   # the bottom hinge, off the table
BOLT_H = 0.075                  # the keeper's centre height; the bolt's axis when the gate hangs true
SAG = 0.007                     # the gate has dropped 7 mm at the bolt
LIFT_STOP = 0.016               # the worn hinges let the free end be lifted up to 16 mm off where it sags to

# The bolt: a 150 mm black barrel bolt (backplate 150 x 40 x 2 mm) screwed to the lock rail, its 12 mm zinc bolt
# 9.5 mm in front of the plate. The bolt slides 45 mm toward the post (-y). Its handle (a 4 mm radius stem with a 14 mm
# knob) sticks out toward the robot through a slot in the barrel; at the end of the bolt's travel the slot's lower lip
# has a 10 mm notch the handle can be turned down into, which then stops the bolt sliding back.
PLATE_T = 0.002
BARREL_R = 0.0095
ROD_R = 0.006
ROD_LEN = 0.150
AXIS_X = GATE_FRONT_X - PLATE_T - BARREL_R          # 0.2885: the bolt's axis
ROD_TIP_Y = -0.027                                  # the bolt's end, drawn back
ROD_C_Y = ROD_TIP_Y + ROD_LEN / 2                   # 0.048
THROW = 0.045
STEM_Y = 0.105                                      # the handle, drawn back; thrown it is at 0.060
NOTCH_Y = STEM_Y - THROW                            # 0.060
STEM_R = 0.004
KNOB_R = 0.007
KNOB_C = 0.033                                      # the knob's centre, out from the bolt's axis

# The keeper: a 3 mm steel staple, 30 mm long, screwed to the post's face, its 18 mm square opening centred on BOLT_H
# (3 mm play all round the 12 mm bolt, front to back from the bolt's axis -8 mm .. +11.5 mm to the post).
KEEPER_Y = (-0.075, -0.045)                         # the bolt enters at y -0.045
KEEPER_HALF = 0.009
POST = (0.300, 0.390, -0.130, -0.040)               # 90 x 90 mm gate post (x0, x1, y0, y1)


def _post_boxes():
    a, h = AXIS_X * 1000, BOLT_H * 1000
    k0, k1 = KEEPER_Y[1] * 1000, KEEPER_Y[0] * 1000
    k = KEEPER_HALF * 1000
    return [
        _mm(300, 390, -130, -40, 0, 160, POST_GREY),                       # the post
        _mm(347, 367, -40, -10, 0, 160, POST_GREY),                        # the gate-stop batten on its side
        _mm(a - k - 3, a - k, k1, k0, h - k - 3, h + k + 3, STEEL),        # keeper: front strap
        _mm(a - k - 3, 300, k1, k0, h - k - 3, h - k, STEEL),              # lower strap
        _mm(a - k - 3, 300, k1, k0, h + k, h + k + 3, STEEL),              # upper strap
        _mm(298, 300, k1, k0, h + k + 3, h + k + 18, STEEL),               # screw flanges
        _mm(298, 300, k1, k0, h - k - 18, h - k - 3, STEEL),
    ]


def _gate_boxes():
    """The gate section and the bolt's fixed parts, in world mm as the gate hangs true."""
    a, h = AXIS_X * 1000, BOLT_H * 1000
    return [
        _mm(300, 345, -30, 40, 40, 180, CEDAR),                            # latch stile, 45 x 70 mm
        _mm(300, 345, 40, 330, 40, 110, CEDAR),                            # lock rail, 45 x 70 mm
        _mm(312.5, 332.5, 41, 135, 110, 180, CEDAR_DARK),                  # infill boards, 20 mm, set between
        _mm(312.5, 332.5, 137, 233, 110, 180, CEDAR_DARK),
        _mm(312.5, 332.5, 235, 330, 110, 180, CEDAR_DARK),
        _mm(298, 300, -25, 125, h - 20, h + 20, STEEL),                    # backplate 150 x 40 x 2 mm
        _mm(a - 10.5, 298, 40, 55, h - 11.5, h - 6.5, STEEL),              # slot's lower lip, before the notch
        _mm(a - 10.5, 298, 65, 113, h - 11.5, h - 6.5, STEEL),             # ... and after it (notch y 55..65)
    ]


def _gate_cylinders():
    a, h = AXIS_X, BOLT_H
    return [((a, 0.0075, h), BARREL_R, 0.065, STEEL, "Y"),                  # barrel, front of the slot
            ((a, 0.119, h), BARREL_R, 0.012, STEEL, "Y")]                   # rear guide


def _rot_x(angle):
    """(w, x, y, z) of a turn about x."""
    return (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)


SAG_ANGLE = math.asin(SAG / (PIVOT[1] - (ROD_TIP_Y - THROW)))   # the gate turned about its hinge so the bolt drops 7 mm
LIFT_ANGLE = math.asin(LIFT_STOP / (PIVOT[1] - (ROD_TIP_Y - THROW)))

# The mechanism's root is the post; its pose is ROOT (all shapes are given relative to it).
ROOT = np.array([0.345, -0.085, TOP])
_W = np.array([0.0, 0.0, TOP]) - ROOT          # world (heights from table top) -> root frame
_PIV = PIVOT + np.array([0, 0, TOP]) - ROOT     # the hinge in the root frame


def _shift(boxes, d):
    return [(b[0] + d, b[1], b[2]) for b in boxes]


def _shift_c(cyls, d):
    return [(np.asarray(c[0]) + d, *c[1:]) for c in cyls]


ROD_C = np.array([AXIS_X, ROD_C_Y, BOLT_H])                 # world (true-hanging gate)
GATE_MASS = 0.72        # 0.62 kg of western red cedar section (350 kg/m3), 0.10 kg of backplate, barrel and lip
ROD_MASS = 0.13         # 12 mm steel bolt, 150 mm long
HANDLE_MASS = 0.03

LINKS = (
    # The gate on its hinges: a hinge about x through the bottom hinge, 0.9 m from the free edge. Authored as it sags;
    # it can be lifted up to LIFT_STOP at the bolt, and rests by its own weight on the worn hinges' stop.
    Link("leaf", "hinge", None, at=tuple(_PIV), axis="X", rot=_rot_x(SAG_ANGLE), lower=-LIFT_ANGLE, upper=0.0,
         boxes=tuple(_shift(_gate_boxes(), _W - _PIV)), cylinders=tuple(_shift_c(_gate_cylinders(), _W - _PIV)),
         mass=GATE_MASS, damping=2.0, friction=0.6,
         com=tuple((np.array([0.3225, 0.15, 0.10]) - PIVOT).tolist())),
    # The bolt slides 45 mm toward the post.
    Link("bolt", "slide", "leaf", at=tuple(ROD_C - PIVOT), axis="Y", lower=-THROW, upper=0.0,
         cylinders=(((0.0, 0.0, 0.0), ROD_R, ROD_LEN, ZINC, "Y"),), mass=ROD_MASS, damping=10.0, friction=0.3),
    # The handle turns about the bolt's axis: 0 sticking straight out toward the robot, -90 degrees turned down.
    Link("handle", "hinge", "bolt", at=(0.0, STEM_Y - ROD_C_Y, 0.0), axis="Y", lower=math.radians(-92.0),
         upper=math.radians(3.0),
         cylinders=(((-(0.0055 + 0.028) / 2, 0.0, 0.0), STEM_R, 0.028 - 0.0055, ZINC, "X"),
                    ((-KNOB_C, 0.0, 0.0), KNOB_R, 0.010, KNOB, "X")),
         mass=HANDLE_MASS, damping=0.002, friction=0.6),
)

GATE = part("gate", pos=tuple(ROOT), boxes=_shift(_post_boxes(), _W), links=LINKS, color=POST_GREY, friction=0.6)
OBJECTS = [GATE]

# -- scoring -----------------------------------------------------------------------------------------------------------
LINED_UP = 0.003        # the bolt's end within 3 mm (up or down) of the keeper opening's centre: it fits
IN_PART = 0.005         # the bolt's end 5 mm into the keeper
IN_HOME = 0.025         # 25 mm into the keeper (it stops at 27 mm)
HANDLE_DOWN = 70.0      # degrees the handle is turned down (90 in the notch)
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.pop("_this_step", None)
    _STATE.update(lined=False, part=False, home=False, recent=[], held=0)


_reset_state()


def bolt_tip(joints: dict) -> np.ndarray:
    """The bolt's end (world, m) for the mechanism's joint readings."""
    poses = link_poses(GATE, joints)
    p, r = poses["bolt"]
    return ROOT + p + r @ np.array([0.0, -ROD_LEN / 2, 0.0])


def readings(gate: PartState) -> dict:
    """What a ruler and a photo would read: the bolt end's height off the keeper's centre (m, + above), how far it is
    past the keeper's mouth (m, + into it), the handle's turn down (degrees), and the lift at the bolt (m)."""
    j = gate.joints
    tip = bolt_tip(j)
    return {"dz": float(tip[2] - (TOP + BOLT_H)), "into": float(KEEPER_Y[1] - tip[1]),
            "handle_deg": float(-math.degrees(j.get("handle", 0.0))), "tip": tip}


def _update(objects) -> tuple[float, bool]:
    r = readings(objects["gate"])
    _STATE["lined"] |= bool(abs(r["dz"]) <= LINED_UP)
    _STATE["part"] |= bool(r["into"] >= IN_PART)
    home = bool(r["into"] >= IN_HOME)
    _STATE["home"] |= home
    recent = _STATE["recent"]
    recent.append((r["tip"].copy(), r["handle_deg"]))
    del recent[:-20]
    still = len(recent) == 20 and all(np.linalg.norm(p - r["tip"]) < 0.001 and abs(a - r["handle_deg"]) < 5.0
                                      for p, a in recent)
    done = bool(home and r["handle_deg"] >= HANDLE_DOWN)
    _STATE["held"] = _STATE["held"] + 1 if done else 0
    if done and still and _STATE["held"] >= 20:   # bolted for 20 steps (1 s) in a row, nothing moving over that second
        return 1.0, True
    if _STATE["home"]:
        return 0.75, False
    if _STATE["part"]:
        return 0.5, False
    return (0.25 if _STATE["lined"] else 0.0), False


def success(objects) -> bool:
    """The bolt at least 25 mm into the keeper with its handle turned down into the notch, held for 1 s, still."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[1]                            # score() reuses this, so the step counts once


def score(objects) -> float:
    """0.25 the bolt was lined up with the keeper (gate lifted); 0.5 it went 5 mm into the keeper; 0.75 25 mm (home);
    1.0 home with the handle turned down into its notch, held for 1 s."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
