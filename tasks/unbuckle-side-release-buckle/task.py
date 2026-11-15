"""Unbuckle a side-release buckle. A 38 mm side-release buckle, the kind on backpack hip belts, luggage straps and
pet harnesses, lies buckled on the table, each half on a 12 cm tail of nylon webbing: the female half on the robot's
left, the male half on its right.

The male half's two sprung prongs reach into the female half, and the tab at each prong's tip stands 3 mm out of a
window in the female's side, hooked behind the lip of the female's mouth. The halves come apart only while both tabs
are pressed in (by at least 1.5 mm; flush with the female's sides is 3 mm) and the male half is drawn out; one tab pressed is
not enough, and pulling with neither pressed only drags the whole buckle.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bucklekit import PartState, Slider, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Unbuckle the strap: take the two halves of its buckle apart."

FEMALE_GREY = (0.20, 0.20, 0.22)
MALE_BLACK = (0.06, 0.06, 0.07)
WEBBING = (0.10, 0.20, 0.45)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# Both halves are described in one frame, the buckle's: origin on the table under the centre of the female's mouth
# (its open end), x across the buckle (world x, away from the arms), y along it (world y: the female toward +y, the
# male toward -y), z up. Buckled, each half's body origin is this point. Sizes are in mm.
#
# Female half (dark grey): a hollow shell 54 mm wide at its mouth (y 0..10) and 38 mm wide behind it, 14 mm thick,
# 61 mm long with its webbing bar. Its mouth opening is 41 mm wide (the lips' inner faces at x = +-20.5); behind the
# mouth each side has a window (y 10..42) where the shell is only its top and bottom plates.
MOUTH_HALF = 27.0    # the mouth's outer half-width
LIP_IN = 20.5        # the lips' inner faces
BODY_HALF = 19.0     # the shell's outer half-width behind the mouth (the faces a pinch closes on)
WINDOW = (10.0, 42.0)
FEMALE_MASS = 0.022  # shell 18 g and its webbing tail 4 g


def _female_boxes():
    g, w = FEMALE_GREY, WEBBING
    m, li, b = MOUTH_HALF, LIP_IN, BODY_HALF
    return [
        _mm(-m, m, 0, 10, 0, 3, g), _mm(-m, m, 0, 10, 11, 14, g),        # mouth: bottom and top plates
        _mm(li, m, 0, 10, 3, 11, g), _mm(-m, -li, 0, 10, 3, 11, g),      # the two lips the tabs hook behind
        _mm(-b, b, 10, 55, 0, 3, g), _mm(-b, b, 10, 55, 11, 14, g),      # body: bottom and top plates
        _mm(7.75, 9.25, 12, 50, 3, 5, g), _mm(-9.25, -7.75, 12, 50, 3, 5, g),   # ribs guiding the male's tongue
        _mm(16.5, b, 42, 52, 3, 11, g), _mm(-b, -16.5, 42, 52, 3, 11, g),       # side walls behind the windows
        _mm(-b, b, 52, 55, 3, 11, g),                                    # end wall
        _mm(-21, 21, 55, 61, 0, 8, g),                                   # webbing bar
        _mm(-19, 19, 61, 181, 0, 1.6, w),                                # 12 cm of 38 mm webbing
    ]


# Male half (black): a base 48 mm wide and 40 mm long (y -42..-2) carrying the webbing, a centre tongue (14 mm wide,
# y -2..45) that runs between the female's ribs, and two prongs. Each prong is a sprung slider across the buckle (a
# stand-in for the flexing plastic arm): an arm (x 13..16) through the mouth and a tab (x 13..22, y 11..27) in the
# female's window, standing 3 mm out of its side. The tab's front face is 1 mm behind the lip's back face (y 10): pulled
# with the tab out, the tab stops on the lip, 1.5 mm deep. Pressed in 1.5 mm it passes inside the lip.
TAB_OUT = 22.0
PRONG_TRAVEL = 0.0035   # a prong presses in up to 3.5 mm (the female's sides stop a pinch at 3 mm)
PRONG_RATE = 1500.0     # N/m: 4.5 N per tab pressed flush
PRONG_MASS = 0.010
MALE_MASS = 0.016       # base, tongue and webbing tail


def _male_boxes():
    k, w = MALE_BLACK, WEBBING
    return [
        _mm(-24, 24, -42, -2, 0, 12, k),                                 # base with its webbing bar
        _mm(-7, 7, -2, 45, 4, 10, k),                                    # tongue
        _mm(-19, 19, -162, -42, 0, 1.6, w),                              # 12 cm of 38 mm webbing
    ]


def _prong(name: str, side: int) -> Slider:
    """``side`` +1: the prong on +x (pressed toward -x); -1: on -x."""
    def mx(a, b):
        return (a, b) if side > 0 else (-b, -a)
    boxes = (_mm(*mx(13, 16), -2, 11, 4, 10, MALE_BLACK),                # arm
             _mm(*mx(13, TAB_OUT), 11, 27, 4, 10, MALE_BLACK))           # tab
    return Slider(name, rest=(side * 0.0175, 0.012, 0.007), boxes=boxes, travel=PRONG_TRAVEL, rate=PRONG_RATE,
                  damping=5.0, mass=PRONG_MASS, axis="X", sign=-side)


# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
BUCKLE_XY = (0.30, 0.0)   # the centre of the female's mouth; the buckle runs along y, the female on the left (+y)

FEMALE = part("female", pos=(*BUCKLE_XY, TOP), mass=FEMALE_MASS, boxes=_female_boxes(), color=FEMALE_GREY,
              friction=0.4)
MALE = part("male", pos=(*BUCKLE_XY, TOP), mass=MALE_MASS, boxes=_male_boxes(),
            sliders=(_prong("prong_a", 1), _prong("prong_b", -1)), color=MALE_BLACK, friction=0.4)
OBJECTS = [FEMALE, MALE]

# -- scoring -----------------------------------------------------------------------------------------------------------
MOUTH_PT = (0.0, 0.0, 0.007)       # centre of the female's mouth face (female frame)
SHOULDER_PT = (0.0, -0.002, 0.007)  # centre of the male base's front face (male frame); 2 mm from the mouth buckled
PRESSED = 0.0015                   # a tab pressed in at least 1.5 mm (clear of its lip; flush with the side is 3 mm)
RELEASED_GAP = 0.008               # drawn back 6 mm from buckled: only possible with both tabs past the lips
APART_GAP = 0.052                  # the 47 mm tongue clear of the mouth by at least 5 mm
DROP = 0.05                        # a half more than 5 cm below the table top has fallen off the table
HOLD = 20                          # control steps (1 s at 20 Hz)
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.pop("_this_step", None)
    _STATE.update(pressed=False, released=False, apart=False, failed=False, held=0)


_reset_state()


def gap(male: PartState, female: PartState) -> float:
    """Distance (m) from the centre of the female's mouth to the centre of the male base's front face."""
    return float(np.linalg.norm(male.to_world(SHOULDER_PT) - female.to_world(MOUTH_PT)))


def pressed_in(male: PartState) -> tuple[float, float]:
    """How far (m) each tab is pressed in from its rest."""
    return abs(male.sliders.get("prong_a", 0.0)), abs(male.sliders.get("prong_b", 0.0))


def _update(objects) -> tuple[float, bool]:
    male, female = objects["male"], objects["female"]
    _STATE["failed"] |= bool(min(male.pos[2], female.pos[2]) < TOP - DROP)
    if _STATE["failed"]:
        return 0.0, False
    g = gap(male, female)
    _STATE["pressed"] |= bool(min(pressed_in(male)) >= PRESSED)
    _STATE["released"] |= bool(g >= RELEASED_GAP)
    apart = bool(g >= APART_GAP)
    _STATE["apart"] |= apart
    _STATE["held"] = _STATE["held"] + 1 if apart else 0
    if apart and _STATE["held"] >= HOLD:   # success: apart for 20 control steps (1 s) in a row
        return 1.0, True
    if _STATE["apart"]:
        return 0.75, False
    if _STATE["released"]:
        return 0.5, False
    return (0.25 if _STATE["pressed"] else 0.0), False


def success(objects) -> bool:
    """The two halves apart (the male's tongue at least 5 mm clear of the female's mouth), held for 1 s."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[1]                            # score() reuses this, so the step counts once


def score(objects) -> float:
    """1.0 apart for 1 s; 0.75 apart at some point; 0.5 drawn 6 mm out (released); 0.25 both tabs pressed in at once;
    0 if either half fell off the table."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
