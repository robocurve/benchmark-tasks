"""Hot-swap a server's failed power supply. A two-bay power-supply cage, fixed to the table like a server in its rack,
faces the arms. The left bay holds the failed supply (amber light), the right bay the one still powering the server
(green light). Both are full-size CRPS-format supplies (185 x 73.5 x 39.5 mm, 0.9 kg).

Each supply has an orange grip handle fixed to its face and, beside it, a separate spring-loaded green release tab
that drives a hook on the supply's side: at rest the hook stands out into a notch in the bay wall, and pushing the tab
5 mm toward the handle draws it in, as on a CRPS supply, so the hand that pulls has to squeeze tab and handle together.
At the back of the bay the supply's edge connector is modelled on a card-edge power connector: two sprung contacts in
the supply ride over a ridge on a blade in the cage during the last 8 mm of insertion (about 35 N to mate), and the
hook, springing back out into its notch at home, then holds the supply there. The live supply's power-cord plug sits
in its inlet. A spare supply lies on the table in front of the cage, to the left. The robot must take the failed supply
out and seat the spare in its bay without disturbing the live supply.

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
GREEN_TAB = (0.20, 0.65, 0.35)
BLACK = (0.05, 0.05, 0.05)
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
GRIP_X = -0.127            # handle bar's and release tab's centre along the supply, where a gripper pinches them
GRIP_Y = 0.003             # midway across handle bar and tab (y -8 to +14 mm), close to the supply's centreline
HANDLE_Y = (-0.008, 0.0)   # the fixed handle bar, across the supply
TAB_Y = (0.006, 0.014)     # the release tab at rest; pushed 5 mm toward the handle it stops 1 mm short of it
LATCH_TRAVEL = 0.005       # pushing the tab 5 mm toward the handle (-y) draws the hook fully into the supply
LATCH_RATE = 1000.0        # the latch spring, N/m (5 N with the tab pushed to its stop)
CONTACT_RATE = 5000.0      # each connector contact's spring, N/m
CONTACT_TRAVEL = 0.0045    # how far each contact can be spread


def _psu_boxes(led, plug=False):
    """The supply's rigid body: its case with a slot at the back for the connector, the face plate, the fixed loop
    handle (bar and two arms), the status light and the AC inlet (with the live supply's power-cord plug in it)."""
    boxes = [
        _mm(-92.5, 72, -36.75, 36.75, 0, 39.5, DARK),              # body, up to the connector slot
        _mm(72, 92.5, -36.75, -8, 0, 39.5, DARK),                  # beside the slot (16 x 30 mm at the back, low)
        _mm(72, 92.5, 8, 36.75, 0, 39.5, DARK),
        _mm(72, 92.5, -8, 8, 30, 39.5, DARK),                      # over the slot
        _mm(-94.5, -92.5, -36, 36, 1, 38.5, SILVER),               # face plate
        _mm(-132, -122, -8, 0, 2, 50, ORANGE),                     # fixed handle: grip bar ...
        _mm(-122, -94.5, -8, 0, 2, 7, ORANGE),                     # ... and its arms to the face, below and above
        _mm(-122, -94.5, -8, 0, 45, 50, ORANGE),                   #     where fingers hold the bar
        _mm(-95.5, -94.5, -30, -20, 28, 34, led),                  # status light
        _mm(-95.0, -94.5, -35, -14, 4, 22, BLACK),                 # AC inlet (C14)
    ]
    if plug:   # the live supply's power-cord plug (C13), seated in its inlet: part of the supply, so it moves with it
        boxes.append(_mm(-132, -94.5, -33, -15, 5, 21, BLACK))
    return boxes


RAMP_TOP = (-76.0, 42.75)   # the ramp's crest (mm): 1 mm higher than the catch's outer face
RAMP_DEG = 27.0


def _hook_ramp():
    """The hook's ramp: a 3 mm plate behind the hook's square catch, its outer face running from a crest 1 mm above
    the catch's outer face (x -76, y 42.75 mm) down to the supply's side (y 36.75 mm, 11.8 mm further back), 27
    degrees to the side. Pushed into a bay, the mouth's edge rides up the ramp and cams the hook in; the higher crest
    leaves the catch 1 mm inside the wall's face as it passes the mouth, so no flat face meets the corner flush."""
    x0, y0 = RAMP_TOP
    y1 = PSU_W * 1000 / 2
    x1, thick = x0 + (y0 - y1) / math.tan(math.radians(RAMP_DEG)), 3.0
    a = math.atan2(y1 - y0, x1 - x0)
    length = math.hypot(x1 - x0, y1 - y0)
    nx, ny = -math.sin(a), math.cos(a)            # unit normal on the ramp's outer side
    cx, cy = (x0 + x1) / 2 - nx * thick / 2, (y0 + y1) / 2 - ny * thick / 2
    return ((cx / 1000, cy / 1000, 0.020), (length / 1000, thick / 1000, 0.012), ORANGE, a)


def _latch():
    """The release latch, one part on a slider across the supply (y), separate from the fixed handle: an orange
    release tab (10 x 8 x 48 mm) standing 6 mm to the left of the handle bar, linked through the face to a hook that
    stands 5 mm out of the supply's +y side into a notch in the bay wall: a square catch (4 x 5 x 12 mm, 12.5 mm
    behind the face) with a ramp behind it, like a CRPS supply's latch. Pulling, the catch holds against the notch's
    front edge until the tab is pushed 5 mm toward the handle (spring 1000 N/m), which draws the hook in; pushing a
    supply in, the ramp rides on the bay wall's edge and draws the hook in by itself, and the spring snaps it out into
    the notch at home."""
    tab = _mm(-132, -122, TAB_Y[0] * 1000, TAB_Y[1] * 1000, 2, 50, GREEN_TAB)
    link = _mm(-122, -95.5, TAB_Y[0] * 1000, TAB_Y[1] * 1000, 2, 7, GREEN_TAB)
    catch = _mm(-80, -76, 36.75, 41.75, 14, 26, GREEN_TAB)
    return Slider("latch", rest=(-0.100, 0.010, 0.020), boxes=(tab, link, catch, _hook_ramp()), travel=LATCH_TRAVEL,
                  rate=LATCH_RATE, damping=10.0, axis="Y", sign=-1, mass=0.06)   # tab, latch arm and hook: 60 g


CONTACT_X = (89.0, 92.0)      # each contact's 3 mm tip, along the supply (mm), ...
CONTACT_Y = (3.0, 7.0)        # ... its inner face 3 mm off the centreline at rest (mirrored on the right)
CONTACT_Z = (6.0, 26.0)


def _contacts():
    """The edge connector's spring contacts, in the slot at the supply's back: two 3 x 4 x 20 mm blocks, each on a
    slider across the supply (spread outward up to 4.5 mm against a 5000 N/m spring). Their inner faces stand 6 mm
    apart; the cage's connector blade has a 10 mm ridge, so mating spreads each contact 2 mm. (On a real supply the
    gold-finger blade is on the supply and the sprung contacts in the server; here they are swapped, which gives the
    same forces.)"""
    out = []
    for name, side in (("contact_l", 1), ("contact_r", -1)):
        y0, y1 = sorted((side * CONTACT_Y[0], side * CONTACT_Y[1]))
        box = _mm(*CONTACT_X, y0, y1, *CONTACT_Z, SILVER)
        out.append(Slider(name, rest=(0.0905, side * 0.005, 0.016), boxes=(box,), travel=CONTACT_TRAVEL,
                          rate=CONTACT_RATE, damping=20.0, axis="Y", sign=side, mass=0.03))
    return tuple(out)


# -- the cage, in its own frame: origin on the table where a seated supply's origin sits in the left bay ---------
# Bays 77.5 mm wide (2 mm clear each side) and 40.5 mm tall under a 4 mm roof. The face of a seated supply stands
# 2 mm proud of the bay mouth. The back stop is 1 mm behind a seated supply's back face. Latched home, the hook sits
# in its notch 1.5 mm behind the notch's front edge. Left bay centred on y = 0, right bay on y = -BAY_PITCH.
BAY_W = 0.0775
BAY_PITCH = BAY_W + 0.015   # a 15 mm divider between the bays
MOUTH_X = -0.0905
BACK_X = 0.0935
NOTCH_X = (-0.0815, -0.064)  # the hook's notch: 1.5 mm clear in front of the latched catch, 2 mm behind its ramp
NOTCH_Z = (0.013, 0.027)
# The connector blade (cage side): a 3 mm core from the back stop, with a ridge on each side whose apex stands 5 mm
# off the centreline (2 mm into the contacts' path) at x 85.6 mm. Its front ramp rises at 50 degrees and its back ramp
# falls at 40 degrees, so pushing a supply home the contacts climb the steep ramp (the mating force peak), slide over
# the apex, and drop down the back ramp, which then holds them (the unmating force). They clear the ridge 1 mm short
# of home, so a supply at home rests on its latch, not on the connector.
RIDGE_APEX = (85.6, 5.0)
RIDGE_DEG = 50.0
RIDGE_SIDES = (4.6, 5.5)     # the ridge block's sides along its front and back ramps (mm)
BLADE_X = (78.0, BACK_X * 1000)
BLADE_Z = (6.0, 26.0)


def _blade(yc):
    """The connector blade of the bay centred on ``yc`` (mm): a core and two ridges (yawed blocks)."""
    out = [_mm(*BLADE_X, yc - 1.5, yc + 1.5, *BLADE_Z, SILVER)]
    lu, lv = RIDGE_SIDES
    for side in (1, -1):
        a = side * math.radians(RIDGE_DEG)
        u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
        ax, ay = RIDGE_APEX[0], side * RIDGE_APEX[1]
        # the block's apex corner is its +u, +side*v corner
        cx = ax - lu / 2 * u[0] - side * lv / 2 * v[0]
        cy = ay - lu / 2 * u[1] - side * lv / 2 * v[1]
        out.append((((cx) / 1000, (yc + cy) / 1000, sum(BLADE_Z) / 2000), (lu / 1000, lv / 1000,
                                                                             (BLADE_Z[1] - BLADE_Z[0]) / 1000),
                    SILVER, a))
    return out


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
_B = BACK_X * 1000
CAGE_BOXES = (
    _notched_wall(38.75, 44.75)                              # left outer wall, notched for the failed/spare hook
    + _notched_wall(-53.75, -38.75)                          # divider, notched for the live supply's hook
    + [_mm(-90.5, _B, _R - 44.75, _R - 38.75, 0, 40.5, GREY),   # right outer wall
       _mm(-90.5, 99, _R - 44.75, 44.75, 40.5, 44.5, GREY),     # roof
       _mm(_B, 99, _R - 44.75, 44.75, 0, 40.5, GREY),           # back stop
       _mm(99, 230, _R - 44.75, 44.75, 0, 44.5, GREY)]          # the rest of the server behind the cage
    + [_lead_in(38.75, 1), _lead_in(-38.75, -1), _lead_in(_R + 38.75, 1), _lead_in(_R - 38.75, -1)]
    + _blade(0.0) + _blade(_R)                               # the two bays' connector blades
)

# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
CAGE_XY = (0.46, 0.0)           # the left bay's seated position; the bay mouth is at x 0.3695
SPARE_XY = (0.235, 0.22)        # the spare's origin on the table, in front of the cage to the left, face toward the arms
PULL = 0.260                    # the demo pulls the failed supply this far out ...
CLEAR_DY = -0.150               # ... and slides it this far right (in front of the live supply), clear of the bay's line
SEAT_DX = 0.0                   # a supply's seated origin, relative to the left bay's (the hook bearing on its notch)

CAGE = part("cage", pos=(*CAGE_XY, TOP), mass=None, boxes=CAGE_BOXES, color=GREY, friction=0.15, fixed=True)   # sheet steel


def _psu(name, xy, led, plug=False):
    return part(name, pos=(*xy, TOP), mass=PSU_MASS, boxes=_psu_boxes(led, plug), sliders=(_latch(), *_contacts()),
                color=DARK, friction=0.25)


FAILED = _psu("failed_psu", CAGE_XY, AMBER)
LIVE = _psu("live_psu", (CAGE_XY[0], CAGE_XY[1] - BAY_PITCH), GREEN, plug=True)
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
HOLD_STEPS = 20                        # 1 s at the 20 Hz control rate
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(started=False, out=False, inserted=False, failed=False, recent=[], held=0)
    _STATE.pop("_this_step", None)


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
    """One control step: (score, success condition). The success condition is the spare seated and latched, the
    failed supply out and clear of the bay, the live supply untouched, and the spare still (it moved less than 1 mm
    over the last 1 s). The score is 1.0 only once that condition has held for 1 s (HOLD_STEPS steps in a row), the
    same step the runtime ends the episode with success; until then a seated spare scores 0.75."""
    failed, live, spare = objects["failed_psu"], objects["live_psu"], objects["spare_psu"]
    _STATE["failed"] |= live_moved(live) > LIVE_MOVE_LIMIT
    if _STATE["failed"]:
        _STATE["held"] = 0
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
    del recent[:-(HOLD_STEPS + 1)]            # positions over the last 1 s (HOLD_STEPS intervals)
    still = len(recent) == HOLD_STEPS + 1 and all(np.linalg.norm(p - s) < 0.001 for p in recent)
    done = bool(_STATE["out"] and failed_clear and seated(spare) and still)
    _STATE["held"] = _STATE["held"] + 1 if done else 0
    if _STATE["held"] >= HOLD_STEPS:
        return 1.0, done
    if _STATE["inserted"]:
        return 0.75, done
    if _STATE["out"]:
        return 0.5, done
    return (0.25 if _STATE["started"] else 0.0), done


def success(objects) -> bool:
    """True only once the full score has been reached: the spare seated, latched and still in the left bay, the
    failed supply out and the live supply untouched, held for 1 s (score() is 1.0 from the same step). The runtime
    then ends the episode after this has stayed true for its own 1 s, so it can only end at 1.0."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[0] >= 1.0                     # score() reuses this, so the step counts once


def score(objects) -> float:
    """0.25 failed supply 2 cm out; 0.5 all the way out; 0.75 spare 10 cm into the bay; 1.0 once the spare has been
    home, latched and still, with the failed supply clear, for 1 s. 0 if the live supply ever moved 5 mm."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
