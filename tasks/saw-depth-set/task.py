"""On the bench lies a corded 7-1/4 in circular saw (DEWALT DWE575 class, 4.0 kg, unplugged) on its motor side, blade
side up, the way its manual says to put it down for adjustments. Its depth of cut is set to the full 65 mm: the blade
stands 65 mm past the foot plate. The next job is ripping 3/4 in (19 mm) boards, and a saw should cut no more than
about 1/4 in (6 mm) past the board.

The robot must set the saw to cut 25 mm deep (19 mm board plus 6 mm) and leave it locked there. The foot plate pivots
on the saw at its front; the depth lever at the rear clamps it. Freed, the plate swings on its pivot (held where it is
put by the clamp's friction); swung away from the blade at the rear, it leaves less blade past it. Clamped, it holds.

From the manual and product page: 184 mm blade, 65 mm depth of cut, 4.0 kg, depth lever at the rear. Estimates (as in
the saw model this task shares with the team's guard task): foot plate 300 x 160 x 3 mm, pivot 150 mm in front of the
arbor, guards, motor housing and handles.

Saw frame (the part's own axes, parallel to the world's at the start): origin at the arbor in the blade's plane;
x toward the foot plate (away from the robot), y toward the saw's front (robot's left), z up out of the blade's face.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import (Hinge, PartState, finger_points, on_reset, on_step, part, set_hinge_stiffness,  # noqa: E402
                     set_hinge_target)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Set this saw to cut 25 mm deep and lock it there."

STEEL = (0.70, 0.71, 0.74)
ALU = (0.80, 0.81, 0.82)
YELLOW = (0.93, 0.73, 0.08)
BLACK = (0.08, 0.08, 0.09)
CEDAR = (0.80, 0.58, 0.36)

# -- the saw (m, saw frame) --------------------------------------------------------------------------------------------
XA, YA = 0.300, 0.040              # the arbor on the bench
ZB = 0.1253                        # the blade's plane above the bench (motor end cap and foot plate edge on the bench)
BLADE_R = 0.092                    # 184 mm blade
SHOE_X = 0.024                     # the foot plate's top face: the blade stands 65 mm (depth of cut) past its far face
SHOE_T = 0.003
SHOE_Y = (-0.130, 0.170)           # estimate
SHOE_Z = (-0.125, 0.035)           # estimate (160 mm wide)
UG_R = (0.100, 0.105)              # upper guard rim (estimate)
UG_Z = 0.017                       # half its width
LG_R = (0.094, 0.097)              # lower guard rim (estimate)
LG_Z = 0.012
LG_SPAN = (math.radians(-88.0), math.radians(78.0))   # the lower guard's rim when shut, about the +x direction
OPEN = math.radians(26.0)          # the shim holds the guard this far from shut
RETRACT = math.radians(95.0)       # full retraction
LEVER_R = 0.118                    # lever pad from the arbor (estimate)
LEVER_SHUT = math.radians(-84.1)   # where the lever sits with the guard shut: 12 mm above the foot plate, at the rear
SPRING = 0.30                      # N m per rad (estimate)
SPRING_REST = OPEN + math.radians(70.0)   # the spring's free angle, past the shut stop: 0.50 N m open, 0.37 N m shut
SAW_MASS, GUARD_MASS = 3.85, 0.15
def _qz(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def box(x0, x1, y0, y1, z0, z1, color):
    return ((float((x0 + x1) / 2), float((y0 + y1) / 2), float((z0 + z1) / 2)),
            (float(abs(x1 - x0)), float(abs(y1 - y0)), float(abs(z1 - z0))), color)


def arc(r0, r1, a0, a1, z0, z1, color, n):
    """``n`` boxes along a ring segment from angle a0 to a1 (rad, about z from +x), radii r0 to r1."""
    out, step = [], (a1 - a0) / n
    for k in range(n):
        a = a0 + (k + 0.5) * step
        rm = (r0 + r1) / 2
        chord = 2 * r1 * math.tan(abs(step) / 2) * 1.02
        out.append(((float(rm * math.cos(a)), float(rm * math.sin(a)), float((z0 + z1) / 2)),
                    (float(r1 - r0), float(chord), float(z1 - z0)), color, _qz(a)))
    return out


def turned(boxes, angle):
    """Boxes turned by ``angle`` about the arbor."""
    ca, sa = math.cos(angle), math.sin(angle)
    out = []
    for b in boxes:
        c = (float(ca * b[0][0] - sa * b[0][1]), float(sa * b[0][0] + ca * b[0][1]), b[0][2])
        q0 = b[3] if len(b) > 3 else (1.0, 0.0, 0.0, 0.0)
        a0 = 2 * math.atan2(q0[3], q0[0])
        out.append((c, b[1], b[2], _qz(a0 + angle)))
    return out


_UG0 = math.acos(SHOE_X / UG_R[1])                         # where the upper guard's rim meets the foot plate
BODY = [((0.0, 0.0, 0.0), (2 * BLADE_R, 2 * BLADE_R * math.tan(math.pi / 8), 0.002), STEEL, _qz(k * math.pi / 4))
        for k in range(4)]                                 # the blade: an octagon 184 mm across
BODY += arc(UG_R[0], UG_R[1], _UG0, 2 * math.pi - _UG0, -UG_Z, UG_Z, YELLOW, 10)        # upper guard rim
for z0, z1 in ((UG_Z - 0.003, UG_Z), (-UG_Z, -UG_Z + 0.003)):                           # and its two side plates
    BODY += [box(-0.100, SHOE_X, -0.040, 0.040, z0, z1, YELLOW), box(-0.072, SHOE_X, -0.075, 0.075, z0, z1, YELLOW),
             box(-0.035, SHOE_X, -0.097, 0.097, z0, z1, YELLOW)]
BODY += [box(-0.040, SHOE_X, -0.125, -0.097, UG_Z - 0.003, UG_Z, YELLOW)]              # its squared rear corner (estimate)
BODY += [((-0.020, -0.010, -0.071), (0.082, 0.082, 0.108), YELLOW, _qz(a)) for a in (0.0, math.pi / 4)]   # motor housing
BODY += [box(-0.060, 0.020, -0.050, 0.030, -0.017, -0.010, BLACK),                      # gear case
         box(-0.135, -0.100, -0.110, 0.030, -0.045, -0.012, BLACK),                     # main handle
         box(-0.108, -0.060, 0.008, 0.032, -0.045, -0.012, BLACK),
         box(-0.112, -0.045, -0.110, -0.086, -0.045, -0.018, BLACK),
         box(-0.050, SHOE_X, 0.112, 0.150, -0.062, -0.020, BLACK)]                      # front handle and bevel pivot

_LG = arc(LG_R[0], LG_R[1], LG_SPAN[0], LG_SPAN[1], -LG_Z, LG_Z, BLACK, 9)
_LG += arc(0.060, LG_R[0], LG_SPAN[0], LG_SPAN[1], LG_Z - 0.002, LG_Z, BLACK, 9)
_LG += arc(0.060, LG_R[0], LG_SPAN[0], LG_SPAN[1], -LG_Z, -LG_Z + 0.002, BLACK, 9)
_LG += [((float(0.0625 * math.cos(LEVER_SHUT)), float(0.0625 * math.sin(LEVER_SHUT)), 0.01925), (0.075, 0.008, 0.0035),
         BLACK, _qz(LEVER_SHUT))]

GUARD = Hinge("guard", pivot=(0.0, 0.0, 0.0), boxes=tuple(_LG), lower=-RETRACT, upper=0.0, stiffness=SPRING,
              damping=0.004, target=math.radians(25.0), mass=GUARD_MASS, axis="Z")   # its spring holds it shut

# the saw rests on its motor housing, the back of its main handle and its front bevel bracket; the foot plate's edge
# clears the bench by 2 mm so it can swing
BODY += [box(-0.135, -0.105, -0.100, -0.060, -0.1253, -0.045, BLACK),                 # main handle's back, on the bench
         box(-0.030, 0.020, 0.120, 0.150, -0.1253, -0.062, BLACK)]                   # front bevel bracket, on the bench

# -- the foot plate on its front pivot, and the depth lever ------------------------------------------------------------
SHOE_Z = (-0.123, 0.035)
PIVOT_Y = 0.150                    # the foot plate's pivot, in front of the arbor (estimate)
SHOE_MAX = math.radians(35.0)      # how far the plate can swing out at the rear (the depth bracket's slot)
SHOE_BOXES = (box(SHOE_X, SHOE_X + SHOE_T, SHOE_Y[0], SHOE_Y[1], SHOE_Z[0], -0.015, ALU),   # round the blade slot
              box(SHOE_X, SHOE_X + SHOE_T, SHOE_Y[0], SHOE_Y[1], 0.015, SHOE_Z[1], ALU),
              box(SHOE_X, SHOE_X + SHOE_T, 0.100, SHOE_Y[1], -0.015, 0.015, ALU),
              box(SHOE_X, SHOE_X + SHOE_T, SHOE_Y[0], -0.100, -0.015, 0.015, ALU),
              box(SHOE_X - 0.006, SHOE_X, SHOE_Y[0], SHOE_Y[0] + 0.008, 0.012, SHOE_Z[1], BLACK))  # depth bracket
SHOE = Hinge("shoe", pivot=(SHOE_X, PIVOT_Y, 0.0), boxes=SHOE_BOXES, lower=0.0, upper=SHOE_MAX, stiffness=300.0,
             damping=2.0, target=0.0, mass=0.40, axis="Z")
LEVER_PIVOT = (SHOE_X - 0.034, -0.105, UG_Z + 0.006)   # on the upper guard's rim, 34 mm inboard of the plate
LEVER_L = 0.045
LEVER_FREE = math.radians(-70.0)   # swung out toward the robot: the plate is free
LOCKED_WITHIN = math.radians(8.0)  # the clamp holds once the lever is back within 8 degrees of home
_lx, _ly, _lz = LEVER_PIVOT
LEVER = Hinge("depth_lever", pivot=LEVER_PIVOT, boxes=(
    ((_lx, _ly - LEVER_L / 2, _lz), (0.010, LEVER_L, 0.006), BLACK),                       # the lever
    ((_lx, _ly - LEVER_L + 0.005, _lz + 0.009), (0.010, 0.010, 0.012), BLACK)),            # its thumb tab
    lower=LEVER_FREE, upper=0.0, stiffness=0.0, damping=0.05, mass=0.02, axis="Z")

SAW_ORIGIN = (XA, YA, float(TOP + ZB))
SAW = part("saw", pos=SAW_ORIGIN, mass=SAW_MASS, boxes=BODY, hinges=(GUARD, SHOE, LEVER), color=YELLOW, friction=0.6)
OBJECTS = [SAW]


def depth(saw: PartState) -> float:
    """How far the blade stands past the foot plate's outer face (m)."""
    b = float(saw.hinges.get("shoe", 0.0))
    return BLADE_R - (SHOE_X * math.cos(b) + PIVOT_Y * math.sin(b) + SHOE_T)


def shoe_for(depth_m: float) -> float:
    """The plate's swing that gives a depth (rad)."""
    lo, hi = 0.0, SHOE_MAX
    for _ in range(60):
        mid = (lo + hi) / 2
        if BLADE_R - (SHOE_X * math.cos(mid) + PIVOT_Y * math.sin(mid) + SHOE_T) > depth_m:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# -- scoring ------------------------------------------------------------------------------------------------------------
TARGET, TOL = 0.025, 0.0015
CHANGED = 0.010                    # partial credit once the depth was changed by 10 mm
SAW_MOVE, SAW_TURN = 0.030, math.radians(15.0)
CLEAR = 0.030
STILL_STEPS = 20
STILL_TOL = 0.001
SHOE_K, SHOE_F = 20.0, 0.30        # the clamp's friction when freed: holds until pushed with more than 0.3 N m
SHOE_LOCK_K = 300.0
SAW_BOX = ((-0.137, 0.094), (-0.132, 0.172), (-0.126, 0.036))
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(failed=False, why="", changed=False, saw0=None, clear=False, recent=[], steps=0, hold=0.0,
                  locked=True)


_reset_state()


def lever_locked(saw: PartState) -> bool:
    return float(saw.hinges.get("depth_lever", 0.0)) >= -LOCKED_WITHIN


def _box_distance(p, bounds) -> float:
    d = [max(lo - v, 0.0, v - hi) for v, (lo, hi) in zip(p, bounds)]
    return float(math.sqrt(sum(x * x for x in d)))


def saw_shift(saw: PartState) -> tuple[float, float]:
    if _STATE["saw0"] is None:
        return 0.0, 0.0
    p0, r0 = _STATE["saw0"]
    cos = (np.trace(r0.T @ saw.rotation()) - 1) / 2
    return float(np.linalg.norm(np.asarray(saw.pos) - p0)), float(math.acos(max(-1.0, min(1.0, cos))))


def observe(saw: PartState, fingers) -> None:
    s = _STATE
    if s["saw0"] is None:
        s["saw0"] = (np.array(saw.pos, float), saw.rotation())
    shift, turn = saw_shift(saw)
    if shift > SAW_MOVE or turn > SAW_TURN:
        s["failed"], s["why"] = True, s["why"] or "saw moved"
    if abs(depth(saw) - (BLADE_R - SHOE_X - SHOE_T)) >= CHANGED:
        s["changed"] = True
    s["clear"] = fingers is not None and all(_box_distance(saw.to_local(f), SAW_BOX) >= CLEAR for f in fingers)
    s["recent"].append(np.concatenate([np.asarray(saw.pos, float), [depth(saw), float(saw.hinges.get("depth_lever",
                                                                                                    0.0)) * 0.04]]))
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _clamp(environment) -> None:
    """Track the trial, and the depth clamp: locked, the plate is held hard where it was when the lever came home;
    freed, it sticks where it is put until pushed past the clamp's friction, then slides along with the push."""
    from partkit import _part_state

    saw = _part_state(environment, SAW)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"saw-depth-set: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(saw, fingers)
    pos = float(saw.hinges.get("shoe", 0.0))
    locked = lever_locked(saw)
    if locked:
        if not _STATE["locked"]:
            _STATE["hold"] = pos                       # clamped where it stands
        set_hinge_stiffness(environment, SAW.name, "shoe", SHOE_LOCK_K)
    else:
        slip, hold = SHOE_F / SHOE_K, _STATE["hold"]
        if pos < hold - slip:
            hold = pos + slip
        elif pos > hold + slip:
            hold = pos - slip
        _STATE["hold"] = hold
        set_hinge_stiffness(environment, SAW.name, "shoe", SHOE_K)
    _STATE["locked"] = locked
    set_hinge_target(environment, SAW.name, "shoe", _STATE["hold"])


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """Metres between the depth of cut and 25 mm, plus 1 cm while the lever is not locked."""
    saw = objects["saw"]
    return abs(depth(saw) - TARGET) + (0.0 if lever_locked(saw) else 0.01)


def _level(objects) -> tuple[float, bool]:
    if _STATE["failed"]:
        return 0.0, False
    saw = objects["saw"]
    on = abs(depth(saw) - TARGET) <= TOL
    if on and lever_locked(saw):
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if on:
        return 0.5, False
    if _STATE["changed"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The blade standing 25 mm (within 1.5 mm) past the foot plate with the depth lever home (locked), both grippers
    3 cm clear of the saw and nothing moving for 1 s; the saw never moved 30 mm or turned 15 degrees."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 set to 25 mm and locked but a gripper is near or something moves; 0.5 set to 25 mm with the
    lever not locked; 0.25 the depth was changed by 10 mm at some point; else 0. 0 once the saw moved 30 mm or turned
    15 degrees."""
    return _level(objects)[0]
