"""On the counter stands a bamboo roll-top bread bin with a loaf inside, its roll top pushed open. The roll top is a
tambour: thirteen bamboo slats joined edge to edge, each with a pin at both ends that runs in a curved groove in the
bin's side panels. Open, the tambour lies flat under the bin's top board with its handle slat at the front of the top;
shut, it is drawn forward over the rounded front corner and down the front to the bottom of the grooves.

The robot must close the bread bin. The tambour only moves along its grooves: pulled any other way its pins bind, so
the handle has to be drawn forward and round the curve, then down the front until the tambour is shut.

Everything is a real product at true size: a bamboo roll-top bread bin (393 mm wide, 295 mm deep, 185 mm tall) with a
tambour of 16 mm slats (0.33 kg in all).

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
from partkit import Hinge, PartState, finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Close the bread bin."

BAMBOO = (0.80, 0.64, 0.42)
SLAT_A, SLAT_B = (0.76, 0.58, 0.36), (0.84, 0.69, 0.47)
CRUST = (0.62, 0.38, 0.17)

# -- the grooves' centreline in the bin's side profile (x forward, z up; metres above the counter)
GROOVE_X = 0.205                 # the front run
GROOVE_BOTTOM = 0.025            # its lower end
ARC_C = (0.255, 0.115)           # the rounded front corner's centre
ARC_R = 0.050
GROOVE_TOP = ARC_C[1] + ARC_R    # the top run (0.165)
GROOVE_BACK = 0.465
GROOVE_W = 0.009                 # 9 mm groove for a 6 mm pin
PANEL_IN, PANEL_SLOT, PANEL_OUT = 0.172, 0.182, 0.192   # side panel: inner face, groove depth, outer face (|y|)
BIN_X = (0.185, 0.480)
BIN_H = 0.185
TOP_BOARD_X = 0.300              # the fixed top board over the stored tambour starts here


def _centreline(n_arc: int = 16):
    pts = [(GROOVE_X, GROOVE_BOTTOM), (GROOVE_X, ARC_C[1])]
    for k in range(1, n_arc):
        a = math.pi - 0.5 * math.pi * k / n_arc
        pts.append((ARC_C[0] + ARC_R * math.cos(a), ARC_C[1] + ARC_R * math.sin(a)))
    pts += [(ARC_C[0], GROOVE_TOP), (GROOVE_BACK, GROOVE_TOP)]
    return pts


def _offset(pts, d):
    """The polyline offset by ``d`` to its left: walking up the front and back along the top, that is outward."""
    out = []
    for i, (x, z) in enumerate(pts):
        a = pts[max(0, i - 1)]
        b = pts[min(len(pts) - 1, i + 1)]
        tx, tz = b[0] - a[0], b[1] - a[1]
        n = math.hypot(tx, tz)
        out.append((x - d * tz / n, z + d * tx / n))
    return out


def slot_outline():
    c = _centreline()
    outer, inner = _offset(c, GROOVE_W / 2), _offset(c, -GROOVE_W / 2)
    r = GROOVE_W / 2
    back = [(GROOVE_BACK + r * math.cos(a), GROOVE_TOP + r * math.sin(a))
            for a in np.linspace(0.5 * math.pi, -0.5 * math.pi, 7)[1:-1]]
    bottom = [(GROOVE_X + r * math.cos(a), GROOVE_BOTTOM + r * math.sin(a))
              for a in np.linspace(0.0, -math.pi, 7)[1:-1]]
    # up the outer side, round the back end, back along the inner side, round the bottom end
    return outer + back + inner[::-1] + bottom


def panel_outline():
    """The side panel's profile: square at the back and bottom, rounded over the front corner."""
    r = ARC_R + GROOVE_W / 2 + 0.0155
    pts = [(BIN_X[0], 0.0), (BIN_X[1], 0.0), (BIN_X[1], BIN_H), (ARC_C[0], BIN_H)]
    for k in range(1, 16):
        a = 0.5 * math.pi + 0.5 * math.pi * k / 16
        pts.append((ARC_C[0] + r * math.cos(a), ARC_C[1] + r * math.sin(a)))
    pts.append((BIN_X[0], ARC_C[1]))
    return pts


def _panel(side: int):
    """A fixed side panel (side +1: left, y > 0): the grooved inner layer and the solid outer layer."""
    xz_hole = slot_outline()
    outline = panel_outline()
    if side > 0:
        inner = mk.extrude(outline, PANEL_SLOT - PANEL_IN, holes=[xz_hole], z0=-PANEL_SLOT).rotated("X", 90)
        outer = mk.extrude(outline, PANEL_OUT - PANEL_SLOT, z0=-PANEL_OUT).rotated("X", 90)
    else:
        inner = mk.extrude(outline, PANEL_SLOT - PANEL_IN, holes=[xz_hole], z0=PANEL_IN).rotated("X", 90)
        outer = mk.extrude(outline, PANEL_OUT - PANEL_SLOT, z0=PANEL_SLOT).rotated("X", 90)
    name = "panel_left" if side > 0 else "panel_right"
    return mk.mesh_part(name, pos=(0.0, 0.0, TOP), friction=0.35,       # waxed grooves
                        shapes=[mk.Shape(inner, color=BAMBOO), mk.Shape(outer, color=BAMBOO)])


_bx = (0.215 + BIN_X[1]) / 2
BIN = [
    _panel(1), _panel(-1),
    Box("bin_bottom", pos=(_bx, 0.0, TOP + 0.0075), size=(BIN_X[1] - 0.215, 2 * PANEL_IN, 0.015), color=BAMBOO),
    Box("bin_back", pos=(BIN_X[1] - 0.005, 0.0, TOP + BIN_H / 2), size=(0.010, 2 * PANEL_IN, BIN_H), color=BAMBOO),
    Box("bin_top", pos=((TOP_BOARD_X + BIN_X[1]) / 2, 0.0, TOP + BIN_H - 0.005),
        size=(BIN_X[1] - TOP_BOARD_X, 2 * PANEL_IN, 0.010), color=BAMBOO),
    Box("loaf", pos=(0.340, 0.0, TOP + 0.015 + 0.040), size=(0.160, 0.180, 0.080), color=CRUST),
]

# -- the tambour (loose partkit part). Part frame: the handle slat's middle with the tambour open (flat, along +x).
N_SLATS = 13
PITCH = 0.016
SLAT_T = 0.008
SLAT_L = 2 * (PANEL_IN - 0.002)  # 2 mm clear of each panel
PIN_R, PIN_L = 0.003, 0.010
BEND = math.radians(40.0)        # how far one joint can fold (the curve needs 18 degrees)
SLAT_MASS = 0.025
HANDLE = (0.008, 0.120, 0.012)   # the handle ridge on the first slat: along the tambour, across, standing out


def _slat_shapes(k: int):
    x = k * PITCH
    color = SLAT_A if k % 2 else SLAT_B
    boxes = (((x, 0.0, 0.0), (PITCH - 0.0005, SLAT_L, SLAT_T), color),)
    cyl = tuple(((x, sy * (SLAT_L / 2 + PIN_L / 2 - 0.002), 0.0), PIN_R, PIN_L, BAMBOO, "Y") for sy in (1, -1))
    return boxes, cyl


def _hinges():
    out = []
    for k in range(1, N_SLATS):
        boxes, cyl = _slat_shapes(k)
        out.append(Hinge(f"joint_{k}", pivot=((k - 0.5) * PITCH, 0.0, 0.0), boxes=boxes, cylinders=cyl,
                         lower=-math.radians(3.0), upper=BEND, stiffness=0.0, damping=0.004, mass=SLAT_MASS, axis="Y",
                         parent="" if k == 1 else f"joint_{k - 1}"))
    return tuple(out)


START = (ARC_C[0] + PITCH / 2, 0.0, GROOVE_TOP - (GROOVE_W / 2 - PIN_R) + 0.0002)   # pins resting in the top run
_b0, _c0 = _slat_shapes(0)
# the handle slat is the leading rail: two pins at each end (5 mm either side of its middle), so it cannot pitch in the
# grooves; the other slats have one pin at each end
_c0 = [((dx, sy * (SLAT_L / 2 + PIN_L / 2 - 0.002), 0.0), PIN_R, PIN_L, BAMBOO, "Y") for dx in (-0.005, 0.005)
       for sy in (1, -1)]
TAMBOUR = part("tambour", pos=(START[0], START[1], TOP + START[2]), mass=SLAT_MASS + 0.005, color=SLAT_B, friction=0.35,
               boxes=[*_b0, ((0.0, 0.0, SLAT_T / 2 + HANDLE[2] / 2), HANDLE, BAMBOO)], cylinders=list(_c0),
               hinges=_hinges())
OBJECTS = [*BIN, TAMBOUR]

# -- scoring ------------------------------------------------------------------------------------------------------------
SHUT = np.array([GROOVE_X, 0.0, GROOVE_BOTTOM])
SHUT_TOL = 0.004                 # the handle slat's pins within 4 mm of the grooves' lower ends
MOVED = 0.030
CLEAR = 0.030
STILL_STEPS = 20
STILL_TOL = 0.001
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(moved_once=False, clear=False, recent=[], steps=0)


_reset_state()


def lead(tam: PartState) -> np.ndarray:
    """The handle slat's pin axis (world, relative to the counter)."""
    return np.asarray(tam.pos, float) - np.array([0.0, 0.0, TOP])


def shut(tam: PartState) -> bool:
    p = lead(tam)
    return abs(float(p[0]) - GROOVE_X) <= SHUT_TOL and float(p[2]) <= GROOVE_BOTTOM + SHUT_TOL


def on_front(tam: PartState) -> bool:
    """The handle slat past the curve, on the front run."""
    p = lead(tam)
    return abs(float(p[0]) - GROOVE_X) <= 0.006 and float(p[2]) < ARC_C[1]


def box_distance(point, tam: PartState) -> float:
    """Distance from a world point to the handle slat and its ridge (a 3 cm clearance check)."""
    p = tam.to_local(point)
    half = np.array([PITCH / 2, SLAT_L / 2, (SLAT_T + HANDLE[2]) / 2])
    c = np.array([0.0, 0.0, HANDLE[2] / 2])
    return float(np.linalg.norm(np.maximum(0.0, np.abs(p - c) - half)))


def observe(tam: PartState, fingers) -> None:
    s = _STATE
    if float(np.linalg.norm(lead(tam) - np.array(START))) > MOVED:
        s["moved_once"] = True
    s["clear"] = fingers is not None and all(box_distance(f, tam) >= CLEAR for f in fingers)
    s["recent"].append(lead(tam))
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _track(environment) -> None:
    from partkit import _part_state

    tam = _part_state(environment, TAMBOUR)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"breadbin-rolltop-close: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(tam, fingers)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """Metres the handle slat still has to travel along the grooves to shut."""
    p = lead(objects["tambour"])
    x, z = float(p[0]), float(p[2])
    if z < ARC_C[1] and x < ARC_C[0]:
        return max(0.0, z - GROOVE_BOTTOM)
    if x >= ARC_C[0]:
        return (x - ARC_C[0]) + 0.5 * math.pi * ARC_R + (ARC_C[1] - GROOVE_BOTTOM)
    a = math.atan2(z - ARC_C[1], x - ARC_C[0])                      # pi/2 at the top, pi at the front
    return ARC_R * (math.pi - a) + (ARC_C[1] - GROOVE_BOTTOM)


def _level(objects) -> tuple[float, bool]:
    tam = objects["tambour"]
    if shut(tam):
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if on_front(tam):
        return 0.5, False
    if _STATE["moved_once"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The roll top shut: its handle slat's pins within 4 mm of the grooves' lower ends, both grippers 3 cm clear of
    the handle slat and nothing moving for 1 s."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 shut but a gripper is near or it still moves; 0.5 the handle slat drawn past the curve onto
    the front run but not shut; 0.25 the handle slat was moved 3 cm along the grooves at some point; else 0."""
    return _level(objects)[0]
