"""On the station table: a countertop straw dispenser and, to its right, five rocks glasses with drinks in them.

The dispenser holds six short straws (146 mm long, 8 mm across) lying side by side on a sloped floor behind its
front wall, where a gripper cannot get around one. A lifter runs along the front: raising its red handle, at the
dispenser's left end, lifts the front straw 3 cm, clear of the walls and of the straws behind it. The lifter falls
back under its own weight when the handle is let go, taking the straw down with it, so a straw can only be taken
while the handle is held up. Each time the lifter comes down the next straw rolls onto it.

The glasses are 85 mm tall and 84 mm across. A straw has to end up standing in each glass: put in at a slant, lower
end first, so that it comes to rest leaning on the rim. The glasses are loose; one that is knocked over is a spilled
drink and no longer counts.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable STRAWS_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from strawkit import finger_points, on_reset, on_step, part, quat_about  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Put one straw from the dispenser in each of the five glasses."

PAPER = (0.95, 0.94, 0.90)
SMOKE = (0.22, 0.24, 0.27)
STEEL = (0.62, 0.64, 0.67)
RED = (0.80, 0.12, 0.10)
GLASS = (0.55, 0.75, 0.85)

# -- straw: a short wide paper straw -------------------------------------------------------------------------------
STRAW_R = 0.004
STRAW_LEN = 0.146
STRAW_CYL = STRAW_LEN - 2 * STRAW_R
STRAW_MASS = 0.0010
N_STRAWS = 6                                   # five glasses and one spare
STRAW_FRICTION = dict(static_friction=0.5, dynamic_friction=0.4, combine="average")

# -- dispenser, in its own frame: x from the front (toward the arms) to the back, y along the straws, z up from the
#    table. x = 0 is the line the front straw lies on; y = 0 is the middle of the straws. ---------------------------
HALF_IN = 0.075                                # half the inside length: 2 mm of play at each end of a straw
WALL = 0.003
GROOVE_Z = 0.045                               # the seat on top of the lifter, with the lifter down
LIFTER_X = (-0.0065, 0.0030)                   # the lifter's front and back faces
LIP = 0.0015                                   # front lip of the seat: this thick and LIP_H tall
LIP_H = 0.0030
STEP_X, STEP_H = 0.0019, 0.0010                # behind the straw the seat steps up by STEP_H, to stop it rolling back
STRAW_X = LIFTER_X[0] + LIP + STRAW_R          # where the front straw's axis lies: -1 mm
SLOT = (-0.0080, 0.0040)                       # the slot the lifter runs in: front wall's inner face, sloped floor's end
LIFT = 0.030                                   # the lifter's travel
RAMP_DEG = 12.0
RAMP_X0, RAMP_X1 = SLOT[1], 0.091              # the sloped floor, from its low end at the slot to the back wall
RAMP_Z0 = GROOVE_Z + STEP_H + 0.0005           # its low end is just above the back of the lifter's seat
FRONT_H = GROOVE_Z + 0.010                     # top of the front wall
BOX_H = 0.085                                  # top of the back and end walls
LIFTER_MASS = 0.040
GRIP_FROM_END = 0.029                          # where a straw is pinched, measured from its left end
GAP_Y = (0.024, 0.062)                         # the lifter's groove is cut away here to let two fingers in
ARM_Y1 = HALF_IN - 0.0005 + 0.075              # the lifter's arm reaches this far out through the left end wall
HANDLE_W, HANDLE_T, HANDLE_H = 0.030, 0.003, 0.039   # the handle: a plate standing on the end of the arm
HANDLE_Z0 = 0.031
HOUSING_FRICTION = dict(static_friction=0.4, dynamic_friction=0.3, combine="average")

# -- glass: a rocks glass with a drink in it ------------------------------------------------------------------------
GLASS_R_OUT = 0.042
GLASS_WALL = 0.003
GLASS_R_IN = GLASS_R_OUT - GLASS_WALL
GLASS_H = 0.085
GLASS_BASE = 0.006
GLASS_MASS = 0.33                              # 130 g of glass and about 200 ml of drink
N_GLASSES = 5
GLASS_FRICTION = dict(static_friction=0.5, dynamic_friction=0.4, combine="average")

# -- layout ----------------------------------------------------------------------------------------------------------
LOT = os.environ.get("STRAWS_LOT", "A")
_COLS = ((-0.135, (0.210, 0.300, 0.390)), (-0.225, (0.255, 0.345)))   # (y of a column, x of its glasses)
LOTS = {"A": dict(dispenser=(0.240, 0.002), shift=(0.000, 0.000)),
        "B": dict(dispenser=(0.232, 0.012), shift=(-0.010, 0.008))}
if LOT not in LOTS:
    raise ValueError(f"STRAWS_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
DISPENSER_XY = LOTS[LOT]["dispenser"]
GLASS_XY = tuple((x + LOTS[LOT]["shift"][0], y + LOTS[LOT]["shift"][1]) for y, xs in _COLS for x in xs)
GLASSES = tuple(f"glass_{i + 1}" for i in range(N_GLASSES))
STRAWS = tuple(f"straw_{i + 1}" for i in range(N_STRAWS))
SPAWN_GAP = 0.0005

# -- scoring constants ---------------------------------------------------------------------------------------------
IN_DEPTH = 0.030                               # a straw is in a glass if its lower end is this far below the rim
REST_SPEED = 0.05                              # m/s
HELD_RADIUS = 0.035                            # a straw with a gripper's grasp point this close to it is held
UPRIGHT_DEG = 10.0                             # a glass stands if it is within this of upright
SPILL_DEG = 30.0                               # and it has spilled if it was ever tipped further than this


def dispenser_point(x: float, y: float, z: float = 0.0) -> tuple:
    """World position of a point given in the dispenser's frame (z up from the table)."""
    return (DISPENSER_XY[0] + x, DISPENSER_XY[1] + y, TOP + z)


def ramp_z(x: float) -> float:
    """Height of the sloped floor's surface at x, in the dispenser's frame."""
    return RAMP_Z0 + (x - RAMP_X0) * math.tan(math.radians(RAMP_DEG))


def _span(x0, x1, y0, y1, z0, z1, color) -> tuple:
    return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _housing():
    out, slope = HALF_IN + WALL, math.radians(RAMP_DEG)
    run = RAMP_X1 - RAMP_X0
    mid = np.array([(RAMP_X0 + RAMP_X1) / 2, 0.0, ramp_z((RAMP_X0 + RAMP_X1) / 2)])
    plate = mid - 0.0015 * np.array([-math.sin(slope), 0.0, math.cos(slope)])       # 3 mm plate under that surface
    boxes = [
        _span(SLOT[0] - 0.004, RAMP_X1 + 0.004, -out, out, 0.0, WALL, SMOKE),                        # floor
        _span(SLOT[0] - 0.004, SLOT[0], -out, out, WALL, FRONT_H, SMOKE),                            # front wall
        _span(SLOT[1], SLOT[1] + 0.003, -HALF_IN, HALF_IN, WALL, RAMP_Z0 - 0.002, SMOKE),            # back of the slot
        (tuple(plate), (run / math.cos(slope), 2 * HALF_IN, 0.003), SMOKE, quat_about("Y", -RAMP_DEG)),  # sloped floor
        _span(RAMP_X1, RAMP_X1 + 0.004, -out, out, WALL, BOX_H, SMOKE),                              # back wall
        _span(SLOT[0], RAMP_X1, -out, -HALF_IN, WALL, BOX_H, SMOKE),                                 # right end wall
        _span(SLOT[1], RAMP_X1, HALF_IN, out, WALL, BOX_H, SMOKE),                                   # left end wall
    ]
    return part("dispenser", pos=dispenser_point(0.0, 0.0), boxes=boxes, color=SMOKE, contact_offset=0.001,
                **HOUSING_FRICTION)


def _lifter():
    """The lifter with its handle, as it sits with the handle down. It slides straight up by LIFT.

    Its seat holds one straw between a front lip and a 1 mm step. Its back face is a plain wall only 4 mm behind
    that straw's axis, so the next straw in the queue leans on the front straw and stands on the sloped floor, never
    on the lifter, and is not carried up with it."""
    x0, x1 = LIFTER_X
    end, mid = HALF_IN - 0.0005, (LIFTER_X[0] + LIFTER_X[1]) / 2
    boxes = [_span(x0, x1, -end, end, 0.004, 0.016, STEEL),                            # bar, lower part
             _span(x0, x1, -end, ARM_Y1, 0.016, 0.031, STEEL),                          # bar and the arm out to the left
             _span(x0, x1, end, end + 0.003, GROOVE_Z, GROOVE_Z + 0.008, STEEL),        # stop at the straw's left end
             _span(mid - HANDLE_T / 2, mid + HANDLE_T / 2, ARM_Y1 - HANDLE_W, ARM_Y1, HANDLE_Z0, HANDLE_Z0 + HANDLE_H, RED)]
    for y0, y1 in ((-end, GAP_Y[0]), (GAP_Y[1], end)):                                 # two posts carrying the seat
        boxes += [_span(x0, x1, y0, y1, 0.031, GROOVE_Z, STEEL),
                  _span(x0, x0 + LIP, y0, y1, GROOVE_Z, GROOVE_Z + LIP_H, STEEL),
                  _span(STEP_X, x1, y0, y1, GROOVE_Z, GROOVE_Z + STEP_H, STEEL)]
    return part("lifter", pos=dispenser_point(0.0, 0.0), mass=LIFTER_MASS, boxes=boxes, color=STEEL,
                rail=("Z", 0.0, LIFT), rail_damping=3.0, contact_offset=0.001, **HOUSING_FRICTION)


def straw_start(i: int) -> tuple:
    """World position of straw i's centre at the start: the first in the lifter's groove, the rest queued behind."""
    if i == 0:
        return dispenser_point(STRAW_X, 0.0, GROOVE_Z + STRAW_R + SPAWN_GAP)
    x = RAMP_X0 + 0.0030 + (i - 1) * (2 * STRAW_R + 0.0005)
    return dispenser_point(x, 0.0, ramp_z(x) + STRAW_R / math.cos(math.radians(RAMP_DEG)) + SPAWN_GAP)


def _straw(i: int):
    return part(STRAWS[i], pos=straw_start(i), mass=STRAW_MASS, capsules=[((0.0, 0.0, 0.0), STRAW_R, STRAW_CYL, PAPER, "Y")],
                color=PAPER, contact_offset=0.001, angular_damping=3.0, gyroscopic=False, solver_iterations=8,
                **STRAW_FRICTION)


def _glass(i: int):
    """An eight-sided glass: eight wall plates on a base made of four crossed bars, which together fill the octagon."""
    r_mid = GLASS_R_OUT - GLASS_WALL / 2
    width = 2 * GLASS_R_OUT * math.tan(math.pi / 8)
    boxes = [((0.0, 0.0, GLASS_BASE / 2), (2 * GLASS_R_OUT, width, GLASS_BASE), GLASS, quat_about("Z", 45.0 * k))
             for k in range(4)]
    for k in range(8):
        a = k * math.pi / 4
        boxes.append(((r_mid * math.cos(a), r_mid * math.sin(a), GLASS_H / 2), (GLASS_WALL, width + 0.001, GLASS_H),
                      GLASS, quat_about("Z", math.degrees(a))))
    x, y = GLASS_XY[i]
    return part(GLASSES[i], pos=(x, y, TOP + SPAWN_GAP), mass=GLASS_MASS, boxes=boxes, color=GLASS, contact_offset=0.001,
                **GLASS_FRICTION)


DISPENSER, LIFTER = _housing(), _lifter()
OBJECTS = [DISPENSER, LIFTER, *(_glass(i) for i in range(N_GLASSES)), *(_straw(i) for i in range(N_STRAWS))]
NAMES = ["lifter", *GLASSES, *STRAWS]          # the parts that move


# -- what happened during the trial --------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(spilled=set(), held=set())


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(state) -> float:
    """Angle between an object's own vertical axis and the vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 2])))))


def straw_ends(state) -> tuple:
    """World positions of the two ends of a straw's axis, lower end first."""
    axis = _rot(state.quat_xyzw)[:, 1] * (STRAW_LEN / 2)
    a, b = np.asarray(state.pos, float) - axis, np.asarray(state.pos, float) + axis
    return (a, b) if a[2] <= b[2] else (b, a)


def _distance_to_segment(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


@on_step
def _watch(environment) -> None:
    """Runs after every control step: notes glasses that were tipped over and straws that are being held."""
    from yam_arena.scene import object_states   # importable only once the simulator has started

    states = object_states(environment, NAMES)
    for name in GLASSES:
        if tilt_deg(states[name]) > SPILL_DEG:
            _STATE["spilled"].add(name)
    hands = [np.asarray(h, float) for h in finger_points(environment)]
    _STATE["held"] = {n for n in STRAWS
                      if any(_distance_to_segment(h, *straw_ends(states[n])) < HELD_RADIUS for h in hands)}


# -- scoring -------------------------------------------------------------------------------------------------------
def glass_of(objects, straw: str) -> str | None:
    """The glass a straw stands in: its lower end is inside that glass, at least IN_DEPTH below the rim."""
    low, _ = straw_ends(objects[straw])
    for name in GLASSES:
        g = objects[name]
        p = _rot(g.quat_xyzw).T @ (low - np.asarray(g.pos, float))
        if math.hypot(p[0], p[1]) <= GLASS_R_IN and p[2] <= GLASS_H - IN_DEPTH:
            return name
    return None


def served(objects, glass: str) -> bool:
    """A glass counts when it stands upright, was never tipped over, and holds exactly one straw that is still and
    has been let go."""
    if glass in _STATE["spilled"] or tilt_deg(objects[glass]) > UPRIGHT_DEG:
        return False
    inside = [s for s in STRAWS if glass_of(objects, s) == glass]
    if len(inside) != 1:
        return False
    s = inside[0]
    return s not in _STATE["held"] and float(np.linalg.norm(objects[s].lin_vel)) < REST_SPEED


def score(objects) -> float:
    return sum(served(objects, g) for g in GLASSES) / N_GLASSES


def success(objects) -> bool:
    return all(served(objects, g) for g in GLASSES)


def report(objects) -> dict:
    """Where everything is, for result files."""
    straws = {}
    for n in STRAWS:
        low, high = straw_ends(objects[n])
        straws[n] = {"in": glass_of(objects, n), "held": n in _STATE["held"],
                     "low_cm": [round(100 * float(v), 2) for v in low], "high_cm": [round(100 * float(v), 2) for v in high],
                     "slant_deg": round(math.degrees(math.asin(min(1.0, (high[2] - low[2]) / STRAW_LEN))), 1)}
    glasses = {g: {"served": served(objects, g), "spilled": g in _STATE["spilled"], "tilt_deg": round(tilt_deg(objects[g]), 1),
                   "pos_cm": [round(100 * float(v), 2) for v in objects[g].pos]} for g in GLASSES}
    return {"straws": straws, "glasses": glasses,
            "lifter_up_mm": round(1000 * (float(objects["lifter"].pos[2]) - TOP), 1)}


def demo() -> dict:
    """yam_scripted plays straight-down pick-and-place only; the two-handed demo is demo.py. Returning no waypoints
    holds both arms still, which is a check of the scene by itself."""
    return {}
