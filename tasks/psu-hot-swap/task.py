"""Hot-swap a server's failed power supply. A two-bay power-supply cage, fixed to the table like a server in its rack,
faces the arms. The left bay holds the failed supply (amber light), the right bay the one still powering the server
(green light). Both are full-size CRPS-format supplies (185 x 73.5 x 39.5 mm, 0.9 kg).

Each supply has an orange vertical grip handle on its face. The handle and a hook on the supply's side are one
spring-loaded part that slides across the supply: at rest the hook stands out into a notch in the bay wall, and
moving the handle 5 mm sideways draws it in (a CRPS supply's release tab, pushed toward its handle, does the same),
so one grip on the handle releases the supply and pulls it. At the back of the bay the supply's edge connector is modelled as a spring plunger: the supply has to be
pushed home against it, and the hook, springing back out into its notch, then holds it there. A spare supply lies on
the table in front of the cage, to the left. The robot must take the failed supply out and seat the spare in its bay without disturbing the
live supply.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from psukit import PartState, Slider, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("The power supply with the amber light has failed. Replace it with the spare power supply on the table, "
               "without disturbing the power supply with the green light, which is keeping the server running.")

GREY = (0.55, 0.56, 0.58)
DARK = (0.16, 0.17, 0.19)
SILVER = (0.72, 0.73, 0.75)
ORANGE = (1.00, 0.50, 0.00)
AMBER = (1.00, 0.70, 0.05)
GREEN = (0.10, 0.85, 0.20)
UNLIT = (0.30, 0.30, 0.30)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the supply, in its own frame: origin under its centre on the table, long axis x, face (handle) toward -x -------
# CRPS form factor: 185 x 73.5 x 39.5 mm; 0.9 kg.
PSU_L, PSU_W, PSU_H = 0.185, 0.0735, 0.0395
PSU_MASS = 0.9
FACE_X = -0.0925
GRIP_X = -0.127            # the handle bar's centre, where a gripper pinches it across (y)
GRIP_Y = 0.0               # the bar is on the supply's centreline, so pushing on it doesn't turn the supply
LATCH_TRAVEL = 0.005       # sliding the handle 5 mm toward -y draws the hook fully into the supply
PLUG_TRAVEL = 0.0014       # the edge connector's mating stroke: the last 1.4 mm of insertion, against a spring


def _psu_boxes(led):
    return [
        _mm(-92.5, 92.5, -36.75, 36.75, 0, 39.5, DARK),            # body
        _mm(-94.5, -92.5, -36, 36, 1, 38.5, SILVER),               # face plate
        _mm(-122, -94.5, -4, -2.5, 2, 7, SILVER),                  # handle guide arms, bottom and top, below and above
        _mm(-122, -94.5, -4, -2.5, 45, 50, SILVER),                # where fingers hold the bar, on its far (-y) edge
        _mm(-95.5, -94.5, -30, -20, 28, 34, led),                  # status light
    ]


def _hook_ramp():
    """The hook's ramp: a 3 mm plate behind the hook's square catch, its outer face running from the catch's outer
    edge (x -76, y 41.75 mm) down to the supply's side 10 mm further back (x -66, y 36.75 mm), 27 degrees to the side.
    Pushed into a bay, the wall's edge rides up the ramp and cams the hook in."""
    x0, y0, x1, y1, thick = -76.0, 41.75, -66.0, 36.75, 3.0
    a = math.atan2(y1 - y0, x1 - x0)
    length = math.hypot(x1 - x0, y1 - y0)
    nx, ny = -math.sin(a), math.cos(a)            # unit normal on the ramp's outer side
    cx, cy = (x0 + x1) / 2 - nx * thick / 2, (y0 + y1) / 2 - ny * thick / 2
    return ((cx / 1000, cy / 1000, 0.020), (length / 1000, thick / 1000, 0.012), ORANGE, a)


def _latch():
    """The release handle, one part on a slider across the supply (y): an orange vertical grip bar (10 x 8 x 48 mm,
    standing 27.5 mm in front of the face) and a hook that stands 5 mm out of the supply's +y side into a notch in the
    bay wall: a square catch (4 x 5 x 12 mm, 12.5 mm behind the face) with a ramp behind it, like a CRPS supply's latch.
    Pulling, the catch holds against the notch's front edge until the bar is slid 5 mm toward -y (spring 300 N/m),
    which draws the hook in; pushing a supply in, the ramp rides on the bay wall's edge and draws the hook in by
    itself, and the spring snaps it out into the notch at home."""
    bar = _mm(-132, -122, -4, 4, 2, 50, ORANGE)
    catch = _mm(-80, -76, 36.75, 41.75, 14, 26, ORANGE)
    return Slider("latch", rest=(-0.100, 0.0, 0.020), boxes=(bar, catch, _hook_ramp()), travel=LATCH_TRAVEL,
                  rate=300.0, damping=10.0, axis="Y", sign=-1, mass=0.06)   # handle and latch, steel: 60 g


def _plug():
    """The edge connector's mating resistance: a plunger (10 x 40 x 20 mm) standing 1.4 mm proud of the back face. It
    just meets the back stop when the supply is latched home, so it holds no load there; pushing the supply the last
    1.4 mm to the stop presses it in against a 3000 N/m spring (4.2 N)."""
    plunger = _mm(85.9, 93.9, -20, 20, 10, 30, SILVER)
    return Slider("plug", rest=(0.0925, 0.0, 0.020), boxes=(plunger,), travel=PLUG_TRAVEL, rate=3000.0, damping=20.0,
                  axis="X", sign=-1, mass=0.03)


# -- the cage, in its own frame: origin on the table where a seated supply's origin sits in the left bay ---------
# Bays 77.5 mm wide (2 mm clear each side) and 40.5 mm tall under a 4 mm roof. The face of a seated supply stands
# 2 mm proud of the bay mouth. The back stop is 1.5 mm behind a seated supply's back face: the connector plunger
# just touches it there, and a supply pushed in presses the plunger over the last 1.4 mm. Latched home, the hook sits
# at the front of its notch with nothing pushing it. Left bay centred on y = 0, right bay on y = -BAY_PITCH.
BAY_W = 0.0775
BAY_PITCH = BAY_W + 0.015   # a 15 mm divider between the bays
MOUTH_X = -0.0905
BACK_X = 0.094
NOTCH_X = (-0.0815, -0.064)  # the hook's notch: 1.5 mm clear in front of the latched catch, 2 mm behind its ramp
NOTCH_Z = (0.013, 0.027)


def _notched_wall(y0, y1):
    """A 6 mm side wall from the mouth to the back stop, with the hook's notch cut through it."""
    nx0, nx1 = NOTCH_X[0] * 1000, NOTCH_X[1] * 1000
    nz0, nz1 = NOTCH_Z[0] * 1000, NOTCH_Z[1] * 1000
    return [_mm(-90.5, nx0, y0, y1, 0, 40.5, GREY), _mm(nx1, 94, y0, y1, 0, 40.5, GREY),
            _mm(nx0, nx1, y0, y1, 0, nz0, GREY), _mm(nx0, nx1, y0, y1, nz1, 40.5, GREY)]


def _lead_in(y_edge, outward):
    """A 15 mm lead-in at the bay mouth: a 2 mm plate angled 30 degrees, its inner edge meeting the wall's inner face
    (y_edge, mm) at the mouth and flaring ``outward`` (+1 or -1 in y) toward the arms, so a slightly skewed supply is
    steered in."""
    a = math.radians(30)
    length, thick = 15.0, 2.0
    # centre of the plate: from the mouth corner, half its length back along the flare, half its thickness outward
    cx = -90.5 - length / 2 * math.cos(a) - thick / 2 * math.sin(a)
    cy = y_edge + outward * (length / 2 * math.sin(a) + thick / 2 * math.cos(a))
    return ((cx / 1000, cy / 1000, 0.020), (length / 1000, thick / 1000, 0.040), GREY, -outward * a)


_R = -BAY_PITCH * 1000   # the right bay's centre (mm)
CAGE_BOXES = (
    _notched_wall(38.75, 44.75)                              # left outer wall, notched for the failed/spare hook
    + _notched_wall(-53.75, -38.75)                          # divider, notched for the live supply's hook
    + [_mm(-90.5, 94, _R - 44.75, _R - 38.75, 0, 40.5, GREY),   # right outer wall
       _mm(-90.5, 99, _R - 44.75, 44.75, 40.5, 44.5, GREY),     # roof
       _mm(94, 99, _R - 44.75, 44.75, 0, 40.5, GREY),           # back stop
       _mm(99, 230, _R - 44.75, 44.75, 0, 44.5, GREY)]          # the rest of the server behind the cage
    + [_lead_in(38.75, 1), _lead_in(-38.75, -1), _lead_in(_R + 38.75, 1), _lead_in(_R - 38.75, -1)]
)

# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
CAGE_XY = (0.46, 0.0)           # the left bay's seated position; the bay mouth is at x 0.3695
SPARE_XY = (0.235, 0.22)        # the spare's origin on the table, in front of the cage to the left, face toward the arms
PULL = 0.260                    # the demo pulls the failed supply this far out ...
CLEAR_DY = -0.150               # ... and slides it this far right (in front of the live supply), clear of the bay's line
SEAT_DX = 0.0                   # a supply's seated origin, relative to the left bay's (the hook bearing on its notch)

CAGE = part("cage", pos=(*CAGE_XY, TOP), mass=None, boxes=CAGE_BOXES, color=GREY, friction=0.15, fixed=True)   # sheet steel


def _psu(name, xy, led):
    return part(name, pos=(*xy, TOP), mass=PSU_MASS, boxes=_psu_boxes(led), sliders=(_latch(), _plug()), color=DARK,
                friction=0.25)


FAILED = _psu("failed_psu", CAGE_XY, AMBER)
LIVE = _psu("live_psu", (CAGE_XY[0], CAGE_XY[1] - BAY_PITCH), GREEN)
SPARE = _psu("spare_psu", SPARE_XY, UNLIT)
OBJECTS = [CAGE, FAILED, LIVE, SPARE]

# -- scoring -----------------------------------------------------------------------------------------------------------
LIVE_MOVE_LIMIT = 0.005   # the live supply may not move more than 5 mm (pulling it takes the server down)
STARTED = -0.020          # the failed supply has come 2 cm out (its latch must have been released)
OUT_X = MOUTH_X - PSU_L / 2 - 0.005    # origin this far forward: the back face is 5 mm clear of the mouth
IN_X = MOUTH_X - PSU_L / 2 + 0.100     # the spare's back face 10 cm inside the mouth
HOME_X = (-0.0015, 0.0015)             # seated: within 1.5 mm of home along the bay ...
HOME_YZ = 0.002                        # ... and 2 mm across it
LATCHED = -0.001                       # the hook is out in its notch (its slider within 1 mm of rest)
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(started=False, out=False, inserted=False, failed=False, recent=[])


_reset_state()


def in_bay(psu: PartState) -> np.ndarray:
    """A supply's origin relative to its seated position in the left bay (m)."""
    return np.asarray(psu.pos, float) - np.array([CAGE_XY[0] + SEAT_DX, CAGE_XY[1], TOP])


def tilt_deg(psu: PartState) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, psu.rotation()[2, 2]))))


def live_moved(live: PartState) -> float:
    return float(np.linalg.norm(np.asarray(live.pos, float) - np.array(LIVE.pos)))


def seated(psu: PartState) -> bool:
    """Home in the left bay (within 1.5 mm along it and 2 mm across it), level within 3 degrees, hook in its notch."""
    c = in_bay(psu)
    return bool(HOME_X[0] <= c[0] <= HOME_X[1] and abs(c[1]) < HOME_YZ and abs(c[2]) < HOME_YZ
                and tilt_deg(psu) < 3 and psu.sliders["latch"] >= LATCHED)


def _update(objects) -> tuple[float, bool]:
    failed, live, spare = objects["failed_psu"], objects["live_psu"], objects["spare_psu"]
    _STATE["failed"] |= live_moved(live) > LIVE_MOVE_LIMIT
    if _STATE["failed"]:
        return 0.0, False
    f = in_bay(failed)
    _STATE["started"] |= bool(f[0] <= STARTED)
    _STATE["out"] |= bool(f[0] <= OUT_X)
    failed_clear = bool(f[0] <= OUT_X or abs(f[1]) > PSU_W + 0.005)   # not back in (or in front of) the bay
    s = in_bay(spare)
    lined_up = bool(abs(s[1]) < 0.010 and abs(s[2]) < 0.010)
    _STATE["inserted"] |= bool(_STATE["out"] and lined_up and s[0] >= IN_X)
    recent = _STATE["recent"]
    recent.append(s.copy())
    del recent[:-20]
    still = len(recent) == 20 and all(np.linalg.norm(p - s) < 0.001 for p in recent)
    if _STATE["out"] and failed_clear and seated(spare):
        return 1.0, bool(still)
    if _STATE["inserted"]:
        return 0.75, False
    if _STATE["out"]:
        return 0.5, False
    return (0.25 if _STATE["started"] else 0.0), False


def success(objects) -> bool:
    """The spare seated and latched in the left bay, the failed supply out, the live supply untouched, for 1 s."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[1]                            # score() reuses this, so the step counts once


def score(objects) -> float:
    """0.25 failed supply 2 cm out; 0.5 all the way out; 0.75 spare 10 cm into the bay; 1.0 spare home and latched.
    0 if the live supply ever moved 5 mm."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
