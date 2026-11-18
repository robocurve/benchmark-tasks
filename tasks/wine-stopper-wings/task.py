"""On a bar mat stands an opened half bottle of sparkling wine, its cork gone. Beside it a two-clamp sparkling-wine
stopper sits on a stopper peg, both side clamps open.

The robot must reseal the bottle with the stopper: set the stopper's cap down over the bottle's mouth, so its skirt
centres it on the glass lip and its gasket seals the mouth, then squeeze the two side clamps in until they click shut.
Each clamp hangs from a pivot at the cap's edge; squeezed in, it swings its hook in under the ledge below the bottle's
lip, so the gas pressure cannot push the stopper off. A clamp that is not squeezed past its catch springs back open.

Everything is a real product at true size: a 375 ml sparkling-wine half bottle (80 mm across, 232 mm tall, 31 mm
crown lip, 0.95 kg with what is left in it) and a chrome two-clamp stopper (42 mm cap, 60 g).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import meshkit as mk  # noqa: E402
import numpy as np  # noqa: E402
from partkit import (Hinge, PartState, finger_points, on_reset, on_step, part, set_hinge_stiffness,  # noqa: E402
                     set_hinge_target)
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Reseal the opened sparkling wine with the stopper."

GLASS = (0.10, 0.26, 0.14)
FOIL = (0.80, 0.66, 0.25)
CHROME = (0.78, 0.79, 0.80)
GASKET = (0.15, 0.15, 0.16)
MAT = (0.12, 0.12, 0.12)
PEG = (0.45, 0.32, 0.20)

# -- the bottle (loose). Body frame: origin at the middle of its base.
MOUTH_Z = 0.232                  # the mouth's top face
LIP_R, LIP_Z0 = 0.0155, 0.222    # the crown lip: 31 mm across, its ledge underside 10 mm below the mouth
NECK_R = 0.0125
BOTTLE_PROFILE = [(0.0, 0.0), (0.040, 0.0), (0.040, 0.140), (0.037, 0.153), (0.029, 0.168), (0.019, 0.182),
                  (0.0135, 0.194), (NECK_R, 0.205), (NECK_R, LIP_Z0 - 0.001), (LIP_R, LIP_Z0), (LIP_R, MOUTH_Z - 0.001),
                  (LIP_R - 0.001, MOUTH_Z), (0.0, MOUTH_Z)]
BOTTLE_XY = (0.300, -0.080)
BOTTLE_MASS = 0.95
BOTTLE = mk.mesh_part("bottle", pos=(BOTTLE_XY[0], BOTTLE_XY[1], TOP + 0.004 + 0.002), mass=BOTTLE_MASS, friction=0.7,
                      shapes=[mk.Shape(mk.revolve(BOTTLE_PROFILE, segments=48), color=GLASS, roughness=0.15),
                              mk.Shape(mk.revolve([(0.0, 0.196), (0.0137, 0.196), (NECK_R + 0.0004, 0.212),
                                                   (0.0, 0.212)], segments=48), color=FOIL, density=0.0,
                                       collider="none")])

# -- the stopper (loose partkit part). Part frame: origin on the cap's axis at its gasket face (seated: the mouth).
CAP_R, CAP_WALL, CAP_TOP = 0.021, 0.003, 0.006
SKIRT = 0.008                    # the skirt reaches 8 mm down round the lip: 2.5 mm clear of it all round
PIVOT_Y, PIVOT_Z = 0.0255, -0.004   # each side clamp's pivot, out from the cap's axis and below the gasket face
HOOK_TOP = -0.014                # the hook's top face below the mouth (the ledge is 10 mm down: 4 mm play)
HOOK_IN = 0.0140                 # its inner end from the bottle's axis (the neck is 12.5 mm)
PAD = (0.0265, 0.0295, -0.016, -0.006)   # the knurled grip pad on each clamp's outer face: y from, y to, z from, z to
LEVER_OPEN = math.radians(18.0)  # each clamp's open stop: its hook swings 2 mm clear of the lip
LATCH = math.radians(2.0)        # squeezed to within 2 degrees of shut it clicks under its catch
STOPPER_MASS = 0.06


def _rot_x(angle):
    return (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)


def _lever_boxes(side: int):
    """One side clamp's boxes in its open pose (its hinge angle 0): side +1 at +y, -1 at -y. Shut, it is a 3 mm plate
    down the outside of the lip with a hook under the ledge and a knurled pad on its outer face to squeeze."""
    s = side
    shut = [
        ((0.0, s * (PIVOT_Y - 0.0005), HOOK_TOP / 2), (0.016, 0.003, 0.008 - HOOK_TOP)),                   # plate
        ((0.0, s * (HOOK_IN + PIVOT_Y - 0.002) / 2, HOOK_TOP - 0.002), (0.016, PIVOT_Y - 0.002 - HOOK_IN, 0.004)),  # hook
        ((0.0, s * (PAD[0] + PAD[1]) / 2, (PAD[2] + PAD[3]) / 2), (0.014, PAD[1] - PAD[0], PAD[3] - PAD[2])),  # pad
    ]
    a = s * LEVER_OPEN                                                   # open: the hook swings out
    c, sn = math.cos(a), math.sin(a)
    out = []
    for (x, y, z), size in shut:
        dy, dz = y - s * PIVOT_Y, z - PIVOT_Z
        out.append(((x, s * PIVOT_Y + dy * c - dz * sn, PIVOT_Z + dy * sn + dz * c), size, CHROME, _rot_x(a)))
    return tuple(out)


def lever_name(side: int) -> str:
    return "clamp_left" if side > 0 else "clamp_right"


def _shut_angle(side: int) -> float:
    return -side * LEVER_OPEN                                            # the joint angle when shut


def _lever(side: int) -> Hinge:
    lo, hi = sorted((0.0, _shut_angle(side) - side * math.radians(1.0)))
    return Hinge(lever_name(side), pivot=(0.0, side * PIVOT_Y, PIVOT_Z), boxes=_lever_boxes(side), lower=lo, upper=hi,
                 stiffness=0.05, damping=0.002, target=0.0, mass=0.006, axis="X")


PEG_XY, PEG_H, PEG_R = (0.200, -0.190), 0.080, 0.0125
def _yaw_q(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


_STAVES = 12                      # the skirt as 12 staves round the lip (2.5 mm clear of it all round)
_SKIRT_R = CAP_R - CAP_WALL / 2
CAP_BOXES = [((_SKIRT_R * math.cos(2 * math.pi * k / _STAVES), _SKIRT_R * math.sin(2 * math.pi * k / _STAVES),
               -SKIRT / 2 + 0.0005), (CAP_WALL, 2 * CAP_R * math.tan(math.pi / _STAVES) * 1.05, SKIRT + 0.001), CHROME,
              _yaw_q(2 * math.pi * k / _STAVES)) for k in range(_STAVES)]
STOPPER = part("stopper", pos=(PEG_XY[0], PEG_XY[1], TOP + PEG_H + 0.0005), mass=STOPPER_MASS, color=CHROME,
               friction=0.9, boxes=[((0.0, 0.0, CAP_TOP + 0.005), (0.016, 0.016, 0.010), CHROME), *CAP_BOXES],
               cylinders=[((0.0, 0.0, CAP_TOP / 2), CAP_R, CAP_TOP, CHROME, "Z"),
                          ((0.0, 0.0, 0.0002), CAP_R - CAP_WALL - 0.0005, 0.0004, GASKET, "Z")],
               hinges=(_lever(1), _lever(-1)))

SCENERY = [Box("bar_mat", pos=(BOTTLE_XY[0], BOTTLE_XY[1], TOP + 0.002), size=(0.150, 0.150, 0.004), color=MAT),
           Cylinder("stopper_peg", pos=(PEG_XY[0], PEG_XY[1], TOP + PEG_H / 2), radius=PEG_R, height=PEG_H, color=PEG)]
OBJECTS = [BOTTLE, STOPPER, *SCENERY]

# -- scoring ------------------------------------------------------------------------------------------------------------
SEAT_XY, SEAT_Z, SEAT_TILT = 0.0025, 0.002, math.radians(5.0)
TIPPED = math.radians(20.0)
CLEAR = 0.030
STILL_STEPS = 20
STILL_TOL = 0.001
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(failed=False, why="", seated_once=False, latched={1: False, -1: False}, clear=False, recent=[],
                  steps=0)


_reset_state()


def _axis_tilt(rot: np.ndarray) -> float:
    return math.acos(max(-1.0, min(1.0, float(rot[2, 2]))))


def cap_offset(cap: PartState, bottle) -> np.ndarray:
    """The cap's origin in the bottle's frame, relative to the middle of its mouth (m)."""
    return bottle.to_local(cap.pos) - np.array([0.0, 0.0, MOUTH_Z])


def seated(cap: PartState, bottle) -> bool:
    d = cap_offset(cap, bottle)
    rel = bottle.rotation().T @ cap.rotation()
    return math.hypot(float(d[0]), float(d[1])) <= SEAT_XY and abs(float(d[2])) <= SEAT_Z and _axis_tilt(rel) <= SEAT_TILT


def lever_shut_by(cap: PartState, side: int) -> float:
    """How far a side clamp still is from shut (rad): LEVER_OPEN on its open stop, 0 shut."""
    return abs(_shut_angle(side) - float(cap.hinges.get(lever_name(side), 0.0)))


def box_distance(point, cap: PartState) -> float:
    p = cap.to_local(point) - np.array([0.0, 0.0, 0.0])
    r = math.hypot(float(p[0]), float(p[1]))
    return math.hypot(max(0.0, r - PAD[1] - 0.003), max(0.0, abs(float(p[2])) - 0.025))


def observe(cap: PartState, bottle, fingers) -> None:
    s = _STATE
    if _axis_tilt(bottle.rotation()) > TIPPED or float(bottle.pos[2]) < TOP - 0.01:
        s["failed"], s["why"] = True, s["why"] or "bottle knocked over"
    if seated(cap, bottle):
        s["seated_once"] = True
    for side in (1, -1):
        gap = lever_shut_by(cap, side)
        if gap <= LATCH:
            s["latched"][side] = True               # it clicks under its catch
        elif gap > math.radians(6.0):
            s["latched"][side] = False              # prised back open
    s["clear"] = fingers is not None and all(box_distance(f, cap) >= CLEAR for f in fingers)
    s["recent"].append(np.concatenate([np.asarray(cap.pos, float), np.asarray(bottle.pos, float)]))
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _catch(environment) -> None:
    """Track the trial, and the clamps' catches: a clamp squeezed past its catch is held shut (a stiff drive at shut);
    otherwise its light spring holds it open."""
    from partkit import _part_state

    cap = _part_state(environment, STOPPER)
    bottle = mk._mesh_state(environment, BOTTLE)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"wine-stopper-wings: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(cap, bottle, fingers)
    for side in (1, -1):
        if _STATE["latched"][side]:
            set_hinge_target(environment, STOPPER.name, lever_name(side), _shut_angle(side))
            set_hinge_stiffness(environment, STOPPER.name, lever_name(side), 2.0)
        else:
            set_hinge_target(environment, STOPPER.name, lever_name(side), 0.0)
            set_hinge_stiffness(environment, STOPPER.name, lever_name(side), 0.05)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """Metres from the cap's seated place on the mouth, plus 1 cm per 10 degrees each clamp is from shut."""
    cap, bottle = objects["stopper"], objects["bottle"]
    return float(np.linalg.norm(cap_offset(cap, bottle))) + 0.057 * sum(lever_shut_by(cap, s) for s in (1, -1))


def _level(objects) -> tuple[float, bool]:
    if _STATE["failed"]:
        return 0.0, False
    cap, bottle = objects["stopper"], objects["bottle"]
    on = seated(cap, bottle)
    n = sum(1 for s in (1, -1) if _STATE["latched"][s])
    if on and n == 2:
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if on and n == 1 and _STATE["clear"]:
        return 0.5, False
    if _STATE["seated_once"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The stopper's cap seated on the bottle's mouth (within 2.5 mm of its middle, 2 mm of its top, 5 degrees square)
    with both side clamps clicked shut, both grippers 3 cm clear and nothing moving for 1 s; the bottle never tipped 20
    degrees."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 seated with both clamps shut but a gripper is near or something moves; 0.5 seated with one
    clamp shut and both grippers 3 cm clear; 0.25 the cap was seated on the mouth at some point; else 0. 0 once the
    bottle tipped over 20 degrees."""
    return _level(objects)[0]
