"""Tilt a window's lower sash back up into its frame after cleaning. A small single-hung tilt window (vinyl frame
47 cm wide and 65 cm tall, fixed upper sash) stands on the bench facing the arms. Its lower sash (38 x 30 x 3.5 cm,
1.8 kg, double glazed) has been tilted in for cleaning: it hangs on its two pivot bars in the balance shoes at the
bottom of the jamb tracks and lies flat toward the arms, its top rail resting on a padded block.

Two spring-loaded tilt latches are screwed to the top of the sash's top rail, one at each end, as on an ordinary
tilt window. Each is a low housing with a finger tab and a bolt whose nose stands 11 mm (7/16 in) out of the sash's
side into the slot of the jamb liner, which holds the sash upright. The nose is ramped on the side that meets the
jamb: pushed home, the sash cams both bolts in against their springs, and they click out into the slots once it is
upright. Makers tell owners to hold the top rail with both hands, push the sash all the way back into the frame and
make sure both tilt latches click. Stopped short, the sash rests on its ramped noses against the jamb liners about
10 degrees from upright, unlatched, and falls back in when it is let go. The job is done once the latched sash stands
in the frame with both hands taken away from it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from sashkit import PartState, Slider, gripper_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "The window's lower sash has been tilted in for cleaning. Put it back up in the window frame so that it latches."

WHITE = (0.93, 0.93, 0.91)
GLASS = (0.62, 0.74, 0.80)
GREY = (0.55, 0.56, 0.58)
BLUE = (0.18, 0.36, 0.72)
FOAM = (0.25, 0.25, 0.27)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the lower sash, in its own frame: origin on the pivot axis at the sash's centre; x through the sash (exterior +x),
# y along it (left +y), z up the sash when it stands in the frame. Vinyl, 380 x 300 x 35 mm, double glazed: 1.8 kg.
SASH_W, SASH_H, SASH_T = 0.380, 0.300, 0.035
SASH_MASS = 1.8
SASH_BOTTOM = -0.025          # the pivot bars are 25 mm above the sash's bottom edge
SASH_TOP = SASH_BOTTOM + SASH_H
LATCH_TRAVEL = 0.012          # a latch slides 12 mm toward the middle, which draws its nose 1 mm inside the sash's side
BOLT_OUT = 0.011              # the nose stands 11 mm (7/16 in) out of the sash's side at rest
LATCH_LEN, LATCH_WID = 0.0802, 0.0333   # the latch's housing on top of the rail: 3-5/32 x 1-5/16 in
LATCH_HOUSING = 0.004         # the housing stands 4 mm above the rail (an estimate)
TAB_HEIGHT = 0.008            # the finger tab's top above the rail: low, as on a real latch (an estimate)
TAB_Y = (0.150, 0.172)        # the finger tab along the rail, from the sash's centre
NOSE_X = 0.006                # the nose is 12 mm thick, through the sash
NOSE_RAMP_FROM = -0.001       # its ramp starts 1 mm inside the sash's side and runs out to its tip: 45 degrees
LATCH_RATE = 400.0            # the latch spring, N/m (an estimate): 3.2 N with the nose pushed in level with the liner
LATCH_FRICTION = 0.2          # plastic latch nose on a vinyl jamb liner (an estimate)


def _sash_boxes():
    return [
        _mm(-17.5, 17.5, 145, 190, -25, 275, WHITE),        # left stile
        _mm(-17.5, 17.5, -190, -145, -25, 275, WHITE),      # right stile
        _mm(-17.5, 17.5, -145, 145, -25, 30, WHITE),        # bottom rail
        _mm(-17.5, 17.5, -145, 145, 235, 275, WHITE),       # top (meeting) rail
        _mm(-9, 9, -145, 145, 30, 235, GLASS),              # insulated glass unit, 18 mm
    ] + _latch_housings()


def _pivots():
    """The pivot bars: 12 mm round, standing 11 mm out of each side at the bottom of the stiles, on the pivot axis."""
    return [((0.0, side * 0.1955, 0.0), 0.006, 0.011, GREY, "Y") for side in (1, -1)]


def _latch_housings():
    """The two latch housings, fixed on top of the top rail at its ends (part of the sash body)."""
    half = SASH_W / 2 * 1000
    return [_mm(-LATCH_WID * 500, LATCH_WID * 500, *sorted((s * (half - LATCH_LEN * 1000), s * half)),
                275, 275 + LATCH_HOUSING * 1000, WHITE) for s in (1, -1)]


def _latch(side):
    """A tilt latch's moving part, on a slider along the top rail: a low finger tab on top (22 x 12 mm, its top 8 mm
    above the rail) and, inside the rail, a bolt (12 x 12 mm) whose nose stands 11 mm out of the sash's side. The
    nose is a wedge, ramped on its exterior side: from the bolt's exterior face 1 mm inside the sash's side out to
    its tip at the interior face (12 mm of ramp over 12 mm of thickness, 45 degrees). Pushed against the jamb liner's
    edge the ramp draws the bolt in against its spring (400 N/m: 3.2 N with the nose level with the liner)."""
    s = side
    half = SASH_W / 2 * 1000
    nx = NOSE_X * 1000
    y0, y1 = TAB_Y[0] * 1000, TAB_Y[1] * 1000
    y_root, y_tip = half + NOSE_RAMP_FROM * 1000, half + BOLT_OUT * 1000
    tab = _mm(-6, 6, *sorted((s * y0, s * y1)), 275, 275 + TAB_HEIGHT * 1000, BLUE)
    bolt = _mm(-nx, nx, *sorted((s * y0, s * y_root)), 249, 261, GREY)
    # the nose: a wedge whose exterior face runs from the bolt's exterior face at the root to its interior face at
    # the tip (one convex piece)
    nose = (((nx / 1000, s * y_root / 1000), (-nx / 1000, s * y_root / 1000), (-nx / 1000, s * y_tip / 1000)),
            0.249, 0.261, GREY)
    return Slider(f"latch_{'l' if s > 0 else 'r'}", rest=(0.0, s * (TAB_Y[0] + TAB_Y[1]) / 2, 0.275 + TAB_HEIGHT / 2),
                  boxes=(tab, bolt), prisms=(nose,),
                  travel=LATCH_TRAVEL, rate=LATCH_RATE, damping=5.0, axis="Y", sign=-s, mass=0.02,
                  friction=LATCH_FRICTION)


# -- the window frame, in the same frame as the sash standing in it: origin on the pivot axis. Fixed to the bench, as a
# window is fixed in its wall. Jamb liners 3 mm clear of the sash's sides, each with a 14 mm slot (12 mm deep) that the
# pivot bar and the latch bolt run in; a balance shoe at the bottom of each slot carries the pivot bar. The fixed upper
# sash stands 3 mm outside the lower one and overlaps its top rail by 40 mm: the lower sash closes against it.
LINER_Y = 0.193
SLOT_X = 0.007
FRAME_X = (-0.040, 0.060)
SILL_TOP = -0.033
PIVOT_Z = 0.075               # the pivot axis above the bench top
HEAD_Z = 0.535


def _jamb(s):
    y = sorted((s * 193, s * 235))
    yb = sorted((s * 205, s * 235))
    ys = sorted((s * 193, s * 205))
    top = HEAD_Z * 1000
    return [_mm(-40, -7, *y, -33, top, WHITE),            # interior side of the liner (stops the bolt tilting in)
            _mm(7, 60, *y, -33, top, WHITE),              # exterior side
            _mm(-7, 7, *yb, -33, top, WHITE),             # back of the slot
            _mm(-7, 7, *ys, -33, -6, GREY)]               # balance shoe: the pivot bar rests on it


def _upper_sash():
    return [_mm(20.5, 55.5, -190, 190, 235, 275, WHITE),   # meeting rail
            _mm(20.5, 55.5, -190, 190, 495, 535, WHITE),   # top rail
            _mm(20.5, 55.5, 145, 190, 275, 495, WHITE),
            _mm(20.5, 55.5, -190, -145, 275, 495, WHITE),
            _mm(29, 47, -145, 145, 275, 495, GLASS)]


FRAME_BOXES = (
    [_mm(-40, 60, -235, 235, -75, -33, WHITE),            # sill (on the bench)
     _mm(-40, 60, -235, 235, 535, 575, WHITE)]            # head
    + _jamb(1) + _jamb(-1) + _upper_sash()
)

# -- layout (world frame: x forward from the arms, y left, metres). SASH_LOT picks one of two places for the window. ----
LAYOUTS = {"A": (0.42, 0.0), "B": (0.41, 0.02)}
LOT = os.environ.get("SASH_LOT", "A")
WINDOW_X, WINDOW_Y = LAYOUTS[LOT]   # the sash's mid-plane when upright, and the window's centre line; it faces the arms
PIVOT = (WINDOW_X, WINDOW_Y, TOP + PIVOT_Z)
START_TILT = math.radians(90)   # tilted in to horizontal, interior face down, resting on the block
BLOCK = ((WINDOW_X - 0.235, WINDOW_Y, TOP + 0.0285), (0.070, 0.240, 0.057))   # padded block under the top rail (centre, size)

FRAME = part("window_frame", pos=PIVOT, mass=None, boxes=FRAME_BOXES, color=WHITE, friction=0.4, fixed=True)
BLOCK_PART = part("block", pos=(BLOCK[0][0], BLOCK[0][1], TOP), mass=None,
                  boxes=[((0.0, 0.0, BLOCK[1][2] / 2), BLOCK[1], FOAM)], color=FOAM, friction=0.8, fixed=True)
SASH = part("sash", pos=(PIVOT[0], PIVOT[1], PIVOT[2] + 0.0003), mass=SASH_MASS, boxes=_sash_boxes(),
            cylinders=_pivots(), sliders=(_latch(1), _latch(-1)), color=WHITE, friction=0.4, pitch=-START_TILT)
OBJECTS = [FRAME, BLOCK_PART, SASH]

# -- scoring -----------------------------------------------------------------------------------------------------------
RAISED_1 = 60.0          # deg from upright: the top rail raised about 14 cm
RAISED_2 = 20.0          # nearly up
IN_FRAME = 3.0           # in the frame: both noses pushed in past the jamb liners (they first meet them at 10 degrees)
UPRIGHT = (-1.0, 1.5)    # closed: within this of upright (it closes against the upper sash)
LATCHED = 0.001          # a bolt is out in its slot: its latch within 1 mm of rest
OFF_PIVOT = 0.010        # the pivot axis more than 10 mm from the shoes: the sash came off its pivots
HANDS_CLEAR = 0.030      # let go: both grippers at least 3 cm from every part of the sash
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(best=0.0, failed=False, recent=[], near=None)
    _STATE["_held"], _STATE["_step"] = 0, None


_reset_state()


def _sash_solids() -> list:
    """Every solid piece of the sash in its own frame, as (centre, half size): stiles, rails, glass, latch housings,
    each latch's tab and bolt at rest (with the box round its nose), and the pivot bars."""
    out = [(np.array(c, float), np.array(size, float) / 2) for c, size, _ in SASH.boxes]
    for sl in SASH.sliders:
        out += [(np.array(c, float), np.array(size, float) / 2) for c, size, _ in sl.boxes]
        for tri, z0, z1, _ in sl.prisms:
            xy = np.array(tri, float)
            lo, hi = np.array([*xy.min(0), z0]), np.array([*xy.max(0), z1])
            out.append(((lo + hi) / 2, (hi - lo) / 2))
    out += [(np.array(c, float), np.array([r, half, r])) for c, r, half, _, _ in _pivots()]
    return out


_SOLIDS = _sash_solids()


def hands_near(points, sash: PartState) -> float:
    """The least distance (m) from any of the given points on the grippers to any part of the sash."""
    local = (np.asarray(points, float) - np.asarray(sash.pos, float)) @ sash.rotation()
    return float(min(np.linalg.norm(np.maximum(np.abs(local - c) - h, 0.0), axis=1).min() for c, h in _SOLIDS))


@on_step
def _watch(environment) -> None:
    """Each control step, before scoring: how near the grippers are to the sash."""
    from yam_arena.scene import object_states   # importable only once the simulator has started

    sash = object_states(environment, ["sash"])["sash"]
    _STATE["near"] = hands_near([p for side in gripper_points(environment) for p in side], sash)


def hands_clear() -> bool:
    """Both grippers at least 3 cm from every part of the sash (as last measured)."""
    return _STATE.get("near") is not None and _STATE["near"] >= HANDS_CLEAR


def tilt_deg(sash: PartState) -> float:
    """How far the sash leans in from upright (deg): 0 upright, 90 lying flat toward the arms."""
    r = sash.rotation()
    return math.degrees(math.atan2(-r[0, 2], r[2, 2]))


def pivot_offset(sash: PartState) -> float:
    """How far the sash's pivot axis (its origin) is from where the shoes hold it (m)."""
    return float(np.linalg.norm(np.asarray(sash.pos, float) - np.array(PIVOT)))


def latched(sash: PartState) -> bool:
    return all(abs(v) < LATCHED for v in sash.sliders.values())


def _assess(objects) -> tuple[float, bool]:
    sash = objects["sash"]
    _STATE["failed"] |= pivot_offset(sash) > OFF_PIVOT
    if _STATE["failed"]:
        return 0.0, False
    t = tilt_deg(sash)
    recent = _STATE["recent"]
    recent.append(t)
    del recent[:-20]
    still = len(recent) == 20 and max(recent) - min(recent) < 0.3
    milestone = 0.75 if t <= IN_FRAME else 0.5 if t <= RAISED_2 else 0.25 if t <= RAISED_1 else 0.0
    _STATE["best"] = max(_STATE["best"], milestone)
    if UPRIGHT[0] <= t <= UPRIGHT[1] and latched(sash) and still and hands_clear():
        return 1.0, True
    return _STATE["best"], False


FULL_HOLD_STEPS = 20        # the success condition must hold this many control steps in a row (1 s) for 1.0
UNHELD = 0.75                # the score while it has held for less


def _update(objects) -> tuple[float, bool]:
    """This step's (score, success). success() and score() are both called on each step's states; the step is
    assessed once. The score is 1.0 only once the success condition has held for 1 s; until then it is UNHELD."""
    last = _STATE.get("_step")
    if last is not None and last[0] is objects and last[2] < 2:
        _STATE["_step"] = (last[0], last[1], last[2] + 1)
        return last[1]
    raw, ok = _assess(objects)
    _STATE["_held"] = _STATE.get("_held", 0) + 1 if ok else 0
    out = (raw if raw < 1.0 else 1.0 if _STATE["_held"] >= FULL_HOLD_STEPS else UNHELD, ok)
    _STATE["_step"] = (objects, out, 1)
    return out


def success(objects) -> bool:
    """The sash upright in the frame with both tilt latches clicked out into their slots, still, and both grippers
    at least 3 cm from every part of it, all held for 1 s."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[0] >= 1.0                     # true only once the full score's 1 s hold is complete


def score(objects) -> float:
    """0.25 raised to 60 degrees from upright; 0.5 to 20; 0.75 pushed into the frame (3); 1.0 upright with both
    latches clicked out into their slots, still, and both grippers at least 3 cm from every part of the sash, held
    for 1 s. 0 if the sash ever came off its pivots."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
