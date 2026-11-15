"""Load a 2.5-inch SSD into a tool-less drive carrier, ready for a server's hot-swap bay.

A standard 2.5-inch SATA SSD (100 x 69.85 x 7 mm, 55 g) lies flat on the table in front of the right arm, label up,
connector toward the right. In the middle of the table lies an empty tool-less carrier for a server's 2.5-inch
hot-swap bay (140 x 76 x 15 mm, 0.2 kg), lengthwise across the table with its handle bezel toward the left. A drive
sits at the carrier's rear end (connector flush with the rear edge, where it meets the server's backplane) on two
cross ribs, and is held by four pins that go into its side mounting holes:

- On the rail nearer the arms two pins are fixed.
- The far rail is a separate, spring-loaded part (a real carrier's rail flexes outward; here it slides outward on a
  spring, 150 N/m, up to 7 mm). It carries the other two pins and has a pull tab standing at its front end.

The drive's side holes (two each side, 3 mm above its bottom face) are off-centre along it, as on a real drive, so a
drive turned end for end does not meet the pins. The drive can't simply be pushed down into the carrier: the pins
stand 2 mm into the space between the rails. The sprung rail has to be held open, the drive set down beside the fixed
pins and slid onto them, and the rail let go so its pins spring into the holes on the other side.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from ssdkit import PartState, Slider, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fit the SSD into the drive carrier so it is ready to go into the server."

ALU = (0.70, 0.71, 0.73)
LABEL = (0.92, 0.92, 0.88)
BLACK = (0.08, 0.08, 0.09)
STEEL = (0.50, 0.52, 0.55)
BEZEL = (0.15, 0.16, 0.18)
BLUE = (0.10, 0.35, 0.85)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the SSD, in its own frame: origin under its centre on its bottom face, long axis x, connector end toward +x ------
SSD_L, SSD_W, SSD_H = 0.100, 0.06985, 0.007
SSD_MASS = 0.055
HALF_W = SSD_W / 2 * 1000            # 34.925 mm
HOLE_X = (-36.0, 40.6)               # side holes 14.0 mm from the front end and 9.4 mm from the connector end
HOLE_Z = 3.0                         # hole centres 3 mm above the bottom face
HOLE = 4.0                           # holes modelled 4 mm square (a real one is M3, 3 mm round) ...
HOLE_DEPTH = 3.0                     # ... and 3 mm deep


def _ssd_side(sign):
    """One 3 mm side wall, built around its two holes."""
    y0, y1 = sorted((sign * (HALF_W - HOLE_DEPTH), sign * HALF_W))
    zl, zh = HOLE_Z - HOLE / 2, HOLE_Z + HOLE / 2
    (a0, a1), (b0, b1) = [(x - HOLE / 2, x + HOLE / 2) for x in HOLE_X]
    return [_mm(-50, 50, y0, y1, 0, zl, ALU), _mm(-50, 50, y0, y1, zh, 7, ALU),
            _mm(-50, a0, y0, y1, zl, zh, ALU), _mm(a1, b0, y0, y1, zl, zh, ALU), _mm(b1, 50, y0, y1, zl, zh, ALU)]


SSD_BOXES = (
    [_mm(-50, 44, -31.925, 31.925, 0, 7, ALU),          # aluminium case
     _mm(44, 50, -31.925, 0, 0, 7, BLACK),              # SATA connector, toward one side of the end face
     _mm(44, 50, 0, 31.925, 0, 7, ALU),
     _mm(-42, 30, -26, 26, 6.9, 7.02, LABEL)]           # the label on top
    + _ssd_side(1) + _ssd_side(-1)
)

# -- the carrier, in its own frame: origin under its centre on the table, long axis x, handle bezel toward -x --------
# A seated drive's origin sits at SEAT in this frame: connector end flush with the rear edge, on 4 mm ribs.
CAR_L, CAR_W, CAR_H = 0.140, 0.076, 0.015
CAR_MASS = 0.20
SEAT = np.array([0.020, 0.0, 0.004])
GAP = 0.5                            # rail inner faces stand 0.5 mm clear of the drive's sides
PIN = 2.5                            # pins 2.5 mm square ...
PIN_IN = 2.0                         # ... standing 2 mm in past the drive's side
RAIL_T = 3.0
RAIL_IN = HALF_W + GAP               # 35.425 mm: the rails' inner faces
RAIL_OUT = RAIL_IN + RAIL_T          # 38.425 mm
SX, SZ = SEAT[0] * 1000, SEAT[2] * 1000
PIN_X = tuple(SX + x for x in HOLE_X)          # -16.0, 60.6 mm in the carrier frame
PIN_Z = SZ + HOLE_Z                            # 7 mm above the table
RAIL_SEGS = ((-61.0, -4.0), (48.0, 70.0))      # the rails stand only around the pins; the gap is for fingers
RIB_X = ((-25.0, -10.0), (50.0, 65.0))
RAIL_TRAVEL = 0.007
RAIL_RATE = 150.0
TAB_X = (-61.0, -49.0)                         # the pull tab, at the sprung rail's front end ...
TAB_Y = (RAIL_OUT + 8.0, RAIL_OUT + 10.5)       # ... standing 8 mm out from the rail on a bracket, 2.5 mm thick ...
TAB_Z = (8.0, 34.0)                             # ... and 26 mm tall


def _pin(x, side):
    """A pin on the rail at y side*RAIL_IN, standing in toward the drive."""
    y0, y1 = sorted((side * RAIL_IN, side * (HALF_W - PIN_IN)))
    return _mm(x - PIN / 2, x + PIN / 2, y0, y1, PIN_Z - PIN / 2, PIN_Z + PIN / 2, STEEL)


CARRIER_BOXES = (
    [_mm(-70, -62, -38, 38, 0, 15, BEZEL),                            # handle bezel
     _mm(-74, -70, -25, 25, 3, 12, BLUE),                             # its release handle
     _mm(-62, -34, -33, 33, 0, 3, STEEL)]                             # front floor plate
    + [_mm(x0, x1, -RAIL_OUT, 33, 0, SZ, STEEL) for x0, x1 in RIB_X]  # cross ribs the drive rests on
    + [_mm(-70, 70, -RAIL_OUT, -RAIL_IN, 0, 3, STEEL)]                # fixed rail's base bar
    + [_mm(-62 if i == 0 else x0, x1, -RAIL_OUT, -RAIL_IN, 0, 12, STEEL) for i, (x0, x1) in enumerate(RAIL_SEGS)]
    + [_pin(x, -1) for x in PIN_X]                                    # fixed pins
)


def _latch_rail():
    """The sprung rail: one 30 g part sliding outward (+y) up to 7 mm against a 150 N/m spring, with two pins, and a pull
    tab (16 x 2.5 x 26 mm) standing up 8 mm outboard of its front end on a bracket."""
    boxes = ([_mm(x0, x1, RAIL_IN, RAIL_OUT, 1, 12, STEEL) for x0, x1 in RAIL_SEGS]
             + [_mm(-61, 70, RAIL_IN, RAIL_OUT, 1, 3, STEEL),
                _mm(TAB_X[0], TAB_X[1], RAIL_OUT, TAB_Y[1], 8, 12, BLUE),                   # the tab's bracket
                _mm(TAB_X[0], TAB_X[1], TAB_Y[0], TAB_Y[1], TAB_Z[0], TAB_Z[1], BLUE)]
             + [_pin(x, 1) for x in PIN_X])
    return Slider("rail", rest=(0.0, 0.0, 0.0), boxes=tuple(boxes), travel=RAIL_TRAVEL, rate=RAIL_RATE, damping=2.0,
                  axis="Y", sign=1, mass=0.03)


# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
CARRIER_XY = (0.300, 0.000)
CARRIER_YAW = -math.pi / 2     # long axis across the table: handle bezel toward the left, sprung rail away from the arms
SSD_XY = (0.300, -0.200)
SSD_YAW = -math.pi / 2         # lying label up, connector toward the right

CARRIER = part("carrier", pos=(*CARRIER_XY, TOP), yaw=CARRIER_YAW, mass=CAR_MASS, boxes=CARRIER_BOXES,
               sliders=(_latch_rail(),),
               color=STEEL, friction=0.8)
SSD = part("ssd", pos=(*SSD_XY, TOP), yaw=SSD_YAW, mass=SSD_MASS, boxes=SSD_BOXES, color=ALU, friction=0.5)
OBJECTS = [CARRIER, SSD]

# -- scoring -----------------------------------------------------------------------------------------------------------
IN_CARRIER_XY = 0.005      # the drive's centre within 5 mm of its seated spot (top view) ...
FLAT = 0.001               # ... and every bottom corner within 1 mm of the seated height
PIN_DEPTH = 0.001          # a pin is in its hole when its tip is inside the hole and at least 1 mm past the side
RAIL_CLOSED = -0.001       # the sprung rail within 1 mm of fully closed (its slider within 1 mm of rest)
STILL = 0.001              # moving less than 1 mm over the last second (20 control steps)
HOLD_STEPS = 20
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(recent=[], last=None)


_reset_state()


def relative(ssd: PartState, carrier: PartState):
    """The drive's pose in the carrier's frame: (rotation 3x3, origin offset from the seated origin, m)."""
    rc, rs = carrier.rotation(), ssd.rotation()
    rot = rc.T @ rs
    off = rc.T @ (np.asarray(ssd.pos, float) - np.asarray(carrier.pos, float)) - SEAT
    return rot, off


def corner_heights(ssd: PartState, carrier: PartState) -> np.ndarray:
    """The four bottom corners' heights relative to the seated bottom face (m)."""
    rot, off = relative(ssd, carrier)
    pts = [rot @ np.array([sx * SSD_L / 2, sy * SSD_W / 2, 0.0]) + off for sx in (-1, 1) for sy in (-1, 1)]
    return np.array([p[2] for p in pts])


def pins_in(ssd: PartState, carrier: PartState) -> tuple[int, int]:
    """How many pins are in their holes on the fixed side and on the sprung side."""
    rot, off = relative(ssd, carrier)
    rail = carrier.sliders.get("rail", 0.0)
    counts = []
    for side, shift in ((-1, 0.0), (1, rail)):
        n = 0
        for i, x in enumerate(PIN_X):
            tip = np.array([x / 1000, side * (HALF_W / 1000 - PIN_IN / 1000) + shift, PIN_Z / 1000]) - SEAT
            local = rot.T @ (tip - off)            # the pin tip in the drive's frame (origin under its centre)
            hx = HOLE_X[i] / 1000
            n += bool(abs(local[0] - hx) < HOLE / 2000 and abs(local[2] - HOLE_Z / 1000) < HOLE / 2000
                      and side * local[1] < HALF_W / 1000 - PIN_DEPTH
                      and side * local[1] > HALF_W / 1000 - HOLE_DEPTH / 1000 - 0.0005)
        counts.append(n)
    return counts[0], counts[1]


def _update(objects) -> tuple[float, bool]:
    if _STATE["last"] is not None and _STATE["last"][0] is objects:
        return _STATE["last"][1]
    ssd, carrier = objects["ssd"], objects["carrier"]
    rot, off = relative(ssd, carrier)
    recent = _STATE["recent"]
    recent.append(off.copy())
    del recent[:-HOLD_STEPS]
    lying = bool(np.hypot(off[0], off[1]) < IN_CARRIER_XY and np.all(np.abs(corner_heights(ssd, carrier)) < FLAT)
                 and rot[2, 2] > 0 and rot[0, 0] > 0)
    fixed, sprung = pins_in(ssd, carrier)
    closed = carrier.sliders.get("rail", 0.0) <= -RAIL_CLOSED
    still = len(recent) == HOLD_STEPS and all(np.linalg.norm(p - off) < STILL for p in recent)
    if lying and fixed == 2 and sprung == 2 and closed:
        out = (1.0, True) if still else (0.75, False)
    elif lying and fixed == 2 and sprung == 2:
        out = (0.75, False)
    elif lying and (fixed == 2 or sprung == 2):
        out = (0.5, False)
    elif lying:
        out = (0.25, False)
    else:
        out = (0.0, False)
    _STATE["last"] = (objects, out)
    return out


def success(objects) -> bool:
    """All four pins in the drive's holes, the sprung rail closed, the drive flat and still for 1 s."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[1]                            # score() reuses this, so the step counts once


def score(objects) -> float:
    """0.25 lying flat in the carrier; 0.5 both pins of one side in; 0.75 all four in; 1.0 all four in with the rail
    closed, held still for 1 s. Judged on the drive's state at the time: lying wrong way up or end for end scores 0."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
