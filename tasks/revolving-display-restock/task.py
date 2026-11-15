"""Restock a revolving countertop display. A four-sided revolving merchandiser stands on the counter in front of the
arms: a 30 cm square turntable on a weighted base, with two pockets on each side around a central column and a short
steel upright at each corner. Every pocket holds one boxed USB charger except the two on the side facing away from the
arms, which are empty. Two boxed chargers stand on the counter, one to each side of the display.

The turntable turns freely on its bearing. The empty pockets are out of the arms' reach (and the column and the
stocked boxes are in the way), so the display has to be turned round to bring them to the front, and a display that
turns this easily also turns away when a box is pushed against one of its pockets.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from spinkit import PartState, Slider, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Restock the two empty pockets of the revolving display with the two boxed chargers on the counter."

BLACK = (0.10, 0.10, 0.11)
STEEL = (0.20, 0.20, 0.22)
SILVER = (0.72, 0.73, 0.75)
RED = (0.80, 0.10, 0.10)
CARTON = (0.92, 0.93, 0.95)
BLUE = (0.10, 0.35, 0.80)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the product: a boxed USB wall charger, printed card carton, 70 x 35 x 110 mm, 150 g. Its frame: origin at the
#    middle of its bottom face, 70 mm along y, 35 mm along x.
BOX_W, BOX_D, BOX_H = 0.070, 0.035, 0.110
BOX_MASS = 0.15


def _box_shapes(cx=0.0, cy=0.0, z0=0.0, along_x=False):
    """A boxed charger standing upright, centred at (cx, cy) mm with its bottom at z0 mm; 70 mm along y (or x)."""
    w, d = (BOX_D * 1000, BOX_W * 1000) if not along_x else (BOX_W * 1000, BOX_D * 1000)
    body = _mm(cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2, z0, z0 + 110, CARTON)
    band = _mm(cx - w / 2 - 0.5, cx + w / 2 + 0.5, cy - d / 2 - 0.5, cy + d / 2 + 0.5, z0 + 60, z0 + 90, BLUE)
    return [body, band]


# -- the display, in its own frame: origin at the middle of the base's underside, on the counter. Base 320 x 320 x
#    20 mm (weighted steel, 3.0 kg, rubber feet). The turntable turns on a bearing on top of the base, about the
#    vertical through the origin. Turntable, in its own frame (origin on the axis at the bearing, 20 mm up):
#    a 300 x 300 x 10 mm deck (its top 30 mm above the counter), a 170 x 170 mm central column 95 mm tall, and on
#    each of the four sides a row of two pockets, each 80 mm wide (tangential) and 45 mm deep (radial) inside
#    3 mm walls 60 mm tall, so a 70 x 35 mm box has 5 mm to spare all round. Four 16 mm steel uprights 60 mm
#    tall stand at the deck's corners. Powder-coated black steel; 1.75 kg with its six boxes.
BASE_H = 0.020
DECK_TOP = 0.010            # the deck's top, above the bearing (turntable frame)
COL = 85.0                  # the column's half-width (mm)
POCKET_OUT = 130.0          # the pockets' inner face of the outer wall (mm from the axis)
POCKET_MID = (COL + POCKET_OUT) / 2000   # a pocket's centre, radially (m): 0.1075
POCKET_Y = (1.5 + 81.5) / 2000           # ... and tangentially, either side of the divider (m): 0.0415
POST_XY = 0.135             # the uprights' centres, at (+-0.135, +-0.135)
POST_R = 0.008
POST_TOP = 0.070            # above the bearing: 90 mm above the counter
TRAY_DAMPING = 0.05         # bearing drag, N m s/rad: a push turns it, and it coasts only a few degrees


def _side(nx, ny, stocked):
    """One side's two pockets (walls), and its boxes if it is stocked. (nx, ny): the side's outward normal."""
    tx, ty = -ny, nx                                       # tangential direction
    out = []

    def box(r0, r1, t0, t1, z0, z1, color):                # radial/tangential extents (mm) -> a box in the frame
        xs = sorted([nx * r0 + tx * t0, nx * r1 + tx * t1])
        ys = sorted([ny * r0 + ty * t0, ny * r1 + ty * t1])
        return _mm(xs[0], xs[1], ys[0], ys[1], z0, z1, color)

    z0, z1 = DECK_TOP * 1000, DECK_TOP * 1000 + 60
    out.append(box(POCKET_OUT, POCKET_OUT + 3, -84.5, 84.5, z0, z1, STEEL))  # outer wall
    out.append(box(COL, POCKET_OUT, -1.5, 1.5, z0, z1, STEEL))               # divider
    out.append(box(COL, POCKET_OUT, 81.5, 84.5, z0, z1, STEEL))              # end walls
    out.append(box(COL, POCKET_OUT, -84.5, -81.5, z0, z1, STEEL))
    if stocked:
        for t in (-POCKET_Y * 1000, POCKET_Y * 1000):
            cx, cy = nx * POCKET_MID * 1000 + tx * t, ny * POCKET_MID * 1000 + ty * t
            out += _box_shapes(cx, cy, z0, along_x=abs(nx) < 0.5)
    return out


TRAY_BOXES = (
    [_mm(-150, 150, -150, 150, 0, DECK_TOP * 1000, BLACK),                 # deck
     _mm(-COL, COL, -COL, COL, DECK_TOP * 1000, 105, BLACK),               # central column
     _mm(-COL - 0.5, COL + 0.5, -COL - 0.5, COL + 0.5, 80, 100, RED)]      # header band round the column
    + _side(1, 0, stocked=False)          # the empty side, facing +x (away from the arms) at the start
    + _side(-1, 0, stocked=True) + _side(0, 1, stocked=True) + _side(0, -1, stocked=True)
)
TRAY_POSTS = tuple(((sx * POST_XY, sy * POST_XY, (DECK_TOP + POST_TOP) / 2), POST_R, POST_TOP - DECK_TOP, SILVER)
                   for sx in (-1, 1) for sy in (-1, 1))
EMPTY_POCKETS = ((POCKET_MID, POCKET_Y), (POCKET_MID, -POCKET_Y))   # in the turntable frame


def _tray():
    """The turntable: a free hinge about z through the bearing, damped."""
    shifted = [((c[0], c[1], c[2] + BASE_H), s, col) for c, s, col in TRAY_BOXES]       # part frame: + base height
    posts = [((c[0], c[1], c[2] + BASE_H), r, h, col) for c, r, h, col in TRAY_POSTS]
    return Slider("tray", rest=(0.0, 0.0, BASE_H), boxes=tuple(shifted), cylinders=tuple(posts), travel=None,
                  rate=0.0, damping=TRAY_DAMPING, mass=1.75, axis="Z", kind="hinge")


# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
DISPLAY_XY = (0.35, 0.0)
BOX_L_XY = (0.15, 0.22)     # the two boxed chargers, standing on the counter, 70 mm side along y
BOX_R_XY = (0.15, -0.22)

DISPLAY = part("display", pos=(*DISPLAY_XY, TOP), mass=3.0, boxes=[_mm(-160, 160, -160, 160, 0, 20, BLACK)],
               sliders=(_tray(),), color=BLACK, friction=0.8)
BOX_L = part("charger_left", pos=(*BOX_L_XY, TOP), mass=BOX_MASS, boxes=_box_shapes(), color=CARTON, friction=0.5)
BOX_R = part("charger_right", pos=(*BOX_R_XY, TOP), mass=BOX_MASS, boxes=_box_shapes(), color=CARTON, friction=0.5)
OBJECTS = [DISPLAY, BOX_L, BOX_R]
BOXES = ("charger_left", "charger_right")

# -- scoring -----------------------------------------------------------------------------------------------------------
SEAT_XY = 0.006       # a box's centre within 6 mm of its pocket's centre, radially and tangentially ...
SEAT_Z = 0.008        # ... its bottom within 8 mm of the deck ...
SEAT_TILT = 10.0      # ... and upright within 10 degrees
FACING = 20.0         # the empty side faces the arms: its outward normal within 20 degrees of pointing at them
TIPPED = 20.0         # the display tipped more than 20 degrees: knocked over (0 for the trial)
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(failed=False, recent=[])
    _STATE.pop("_this_step", None)


_reset_state()


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tilt_deg(p: PartState) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, p.rotation()[2, 2]))))


def tray_frame(display: PartState) -> tuple[np.ndarray, np.ndarray]:
    """The turntable's origin (on the axis at the bearing) and rotation in the world."""
    rot = display.rotation() @ _rz(display.sliders["tray"])
    return display.to_world((0.0, 0.0, BASE_H)), rot


def in_tray(display: PartState, p: PartState) -> np.ndarray:
    origin, rot = tray_frame(display)
    return rot.T @ (np.asarray(p.pos, float) - origin)


def facing_deg(display: PartState) -> float:
    """Angle (degrees) between the empty side's outward normal and the direction toward the arms (-x)."""
    n = tray_frame(display)[1] @ np.array([1.0, 0.0, 0.0])
    return math.degrees(math.atan2(abs(n[1]), -n[0]))


def seated(display: PartState, box: PartState, pocket) -> bool:
    c = in_tray(display, box)
    return bool(abs(c[0] - pocket[0]) <= SEAT_XY and abs(c[1] - pocket[1]) <= SEAT_XY
                and abs(c[2] - DECK_TOP) <= SEAT_Z and tilt_deg(box) <= SEAT_TILT)


def filled(objects) -> int:
    """How many of the two empty pockets now hold a seated box."""
    d = objects["display"]
    return sum(any(seated(d, objects[b], pk) for b in BOXES) for pk in EMPTY_POCKETS)


def _update(objects) -> tuple[float, bool]:
    d = objects["display"]
    _STATE["failed"] |= tilt_deg(d) > TIPPED
    if _STATE["failed"]:
        return 0.0, False
    n = filled(objects)
    recent = _STATE["recent"]
    recent.append(np.array([objects[b].pos for b in BOXES], float))
    del recent[:-20]
    still = len(recent) == 20 and all(np.abs(p - recent[-1]).max() < 0.001 for p in recent)
    if n == 2:
        return 1.0, bool(still)
    if n == 1:
        return 0.5, False
    return (0.25 if facing_deg(d) <= FACING else 0.0), False


def success(objects) -> bool:
    """Both empty pockets hold a seated box, the display upright, both boxes still for 1 s."""
    out = _update(objects)
    _STATE["_this_step"] = (objects, out)   # the runtime calls success() then score() on each step's states:
    return out[1]                            # score() reuses this, so the step counts once


def score(objects) -> float:
    """1.0 both pockets filled; 0.5 one; 0.25 the empty side turned to face the arms; 0 if the display was tipped."""
    pending = _STATE.pop("_this_step", None)
    return (pending[1] if pending and pending[0] is objects else _update(objects))[0]
