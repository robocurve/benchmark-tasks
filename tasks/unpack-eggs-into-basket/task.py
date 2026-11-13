"""On the station table: a closed half-dozen egg carton holding six eggs, and to its right a cloth-lined basket,
which is fixed to the table.

The carton is a moulded two-by-three carton with a hinged lid, 146 x 110 x 69 mm closed, and it weighs 32 g empty.
It stands with its hinge toward the arms, so the lid opens toward them. On the far side the lid has a pull tab
(right of centre) and the base a thumb tab (near the left end). The lid snaps shut: the snap takes a 3 N pull at
the lid's tab to open, twice what it takes to lift that side of the loaded carton off the table (1.5 N), so
pulling the lid's tab alone lifts the carton instead of opening it. Something has to hold the base down. Each egg
(43 x 57 mm, 58 g) stands in its own cup with 1 mm of clearance and only its top 22 mm showing above the cup walls,
with 18 mm between the eggs of the two rows at the height a gripper can reach.

The eggs have to end up in the basket, set down gently on its cloth lining: an egg that falls more than 3 cm counts
as broken and earns nothing.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable EGGS_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from eggkit import Flap, Slick, body, finger_points, on_reset, on_step, set_flap_hold  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Open the egg carton and put all six eggs in the basket, setting each one down gently on the cloth. "
               "A dropped egg is a broken egg.")

SHELL = (0.96, 0.95, 0.91)
PULP = (0.55, 0.52, 0.47)
WICKER = (0.60, 0.42, 0.22)
CLOTH = (0.20, 0.35, 0.60)
G = 9.81

# -- egg: a large hen's egg as a capsule, 43 mm across and 57 mm long ---------------------------------------------
EGG_R = 0.0215
EGG_LEN = 0.057
EGG_CYL = EGG_LEN - 2 * EGG_R                  # the straight part between the two round ends
EGG_MASS = 0.058
# Eggshell against a rubber finger pad, the cloth or the carton: 0.7 static, 0.6 sliding. "max" makes PhysX use
# these values in every contact an egg makes (the robot's fingers carry no material of their own).
EGG_FRICTION = dict(static_friction=0.7, dynamic_friction=0.6, combine="max")
EGG_ROLL_DAMPING = 12.0                        # 1/s

# -- carton, in its own frame: x is its depth (two rows; hinge at +x, tabs at -x), y its length (three columns) ----
CELL = 0.045                                   # square cup opening: 1 mm of clearance around a 43 mm egg
SEAT = 0.040                                   # narrower opening at the bottom of each cup that the egg's end sits in
COL_PITCH = 0.048                              # cup spacing along the carton
ROW_PITCH = 0.060                              # cup spacing across it
WALL = 0.0025
FLOOR_T = 0.003
SEAT_H = 0.0175                                # height of the seat above the floor
BASE_H = 0.042                                 # top of the base's walls: the line where the lid meets the base
LID_H = 0.027
CARTON_L = 2 * COL_PITCH + CELL + 2 * WALL     # 0.146
CARTON_D = ROW_PITCH + CELL + 2 * WALL         # 0.110
BASE_MASS, LID_MASS = 0.020, 0.012
CARTON_FRICTION = dict(static_friction=0.6, dynamic_friction=0.5, combine="average")

TAB_T = 0.003
LID_TAB_DEPTH, LID_TAB_WIDTH, LID_TAB_Y = 0.035, 0.030, 0.025      # pull tab on the lid's edge opposite the hinge
BASE_TAB_DEPTH, BASE_TAB_WIDTH, BASE_TAB_Y = 0.025, 0.030, -0.055  # thumb tab on the base, same side, 8 cm along
LID_PINCH_OUT = 0.024                          # where the demo pinches the lid's tab, measured out from the lid's wall

# The snap: two press-fit buttons hold the lid shut until its tab is pulled with SNAP_PULL. Here that is a spring at
# the hinge that lets go for good once the lid has turned SNAP_ANGLE (see _watch below).
SNAP_PULL = 3.0                                # N at the pinch point of the lid's tab
SNAP_ANGLE = 4.0                               # degrees
SNAP_ARM = CARTON_D + LID_PINCH_OUT            # 0.134 m from the hinge
SNAP_HOLD = SNAP_PULL * SNAP_ARM / math.radians(SNAP_ANGLE)        # N m/rad
HINGE_DAMPING = 0.01                           # N m s/rad: the crease's friction, so the freed lid swings open slowly

# An egg sits with its round end in the seat's square opening.
_SAG = EGG_R - math.sqrt(EGG_R**2 - (SEAT / 2) ** 2)
EGG_BOTTOM = FLOOR_T + SEAT_H - _SAG           # 6.9 mm above the table
EGG_TOP = EGG_BOTTOM + EGG_LEN                 # 63.9 mm
CELLS = tuple((sx * ROW_PITCH / 2, cy * COL_PITCH) for cy in (1, 0, -1) for sx in (-1, 1))   # (x, y) in the carton
EGGS = tuple(f"egg_{i + 1}" for i in range(len(CELLS)))
N = len(EGGS)

# -- basket: wicker with a cloth lining, fixed to the table --------------------------------------------------------
BASKET_IN_X, BASKET_IN_Y = 0.200, 0.250        # inside, wall to wall
BASKET_WALL = 0.006
BASKET_H = 0.045                               # rim above the table
BASKET_FLOOR = 0.010                           # top of the cloth lining above the table
BASKET_FRICTION = dict(static_friction=0.9, dynamic_friction=0.8, combine="max")
SLOT_DX, SLOT_DY = 0.105, 0.070                # where the demo lays the eggs: two rows of three, on their sides

# -- layout ----------------------------------------------------------------------------------------------------------
LOT = os.environ.get("EGGS_LOT", "A")
LOTS = {"A": dict(carton=(0.280, -0.040), basket=(0.300, -0.262)),
        "B": dict(carton=(0.265, -0.025), basket=(0.285, -0.252))}
if LOT not in LOTS:
    raise ValueError(f"EGGS_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
CARTON_XY = LOTS[LOT]["carton"]
CARTON_YAW = math.pi                           # hinge toward the arms, tabs on the far side
BASKET_XY = LOTS[LOT]["basket"]
SPAWN_GAP = 0.0003


def carton_point(x: float, y: float, z: float = 0.0) -> tuple:
    """World position, at the start, of a point given in the carton's own frame (z up from the table)."""
    c, s_ = math.cos(CARTON_YAW), math.sin(CARTON_YAW)
    return (CARTON_XY[0] + c * x - s_ * y, CARTON_XY[1] + s_ * x + c * y, TOP + z)

# -- scoring constants ---------------------------------------------------------------------------------------------
DROP_LIMIT = 0.03                              # an egg that falls further than this is broken
BREAK_SPEED = math.sqrt(2 * G * DROP_LIMIT)    # 0.77 m/s: the speed a 3 cm fall ends at
ON_CLOTH = 0.004                               # an egg rests on the lining if its lowest point is this close to it
REST_SPEED = 0.05                              # m/s
HELD_RADIUS = 0.04                             # an egg with a gripper's grasp point this close to its centre is held


def _plate_with_holes(z0, z1, xs, ys, hole, color) -> list:
    """Boxes filling the carton's footprint from z0 to z1 except square holes of side ``hole`` centred on xs x ys."""
    out, zc, h = [], (z0 + z1) / 2, z1 - z0
    x_edges = [-CARTON_D / 2] + [e for x in sorted(xs) for e in (x - hole / 2, x + hole / 2)] + [CARTON_D / 2]
    y_edges = [-CARTON_L / 2] + [e for y in sorted(ys) for e in (y - hole / 2, y + hole / 2)] + [CARTON_L / 2]
    for a, b in zip(x_edges[0::2], x_edges[1::2]):                 # bars along the carton's whole length
        out.append((((a + b) / 2, 0.0, zc), (b - a, CARTON_L, h), color))
    for xa, xb in zip(x_edges[1::2], x_edges[2::2]):               # bars across, between the holes of each row
        for a, b in zip(y_edges[0::2], y_edges[1::2]):
            out.append((((xa + xb) / 2, (a + b) / 2, zc), (xb - xa, b - a, h), color))
    return out


def _carton():
    xs, ys = sorted({c[0] for c in CELLS}), sorted({c[1] for c in CELLS})
    front = -CARTON_D / 2
    base = [((0.0, 0.0, FLOOR_T / 2), (CARTON_D, CARTON_L, FLOOR_T), PULP)]
    base += _plate_with_holes(FLOOR_T, FLOOR_T + SEAT_H, xs, ys, SEAT, PULP)
    base += _plate_with_holes(FLOOR_T + SEAT_H, BASE_H, xs, ys, CELL, PULP)
    # thumb tab, level with the top of the base, on a rib that stands on the table
    base.append(((front - BASE_TAB_DEPTH / 2, BASE_TAB_Y, BASE_H - TAB_T / 2), (BASE_TAB_DEPTH, BASE_TAB_WIDTH, TAB_T),
                 PULP))
    base.append(((front - BASE_TAB_DEPTH / 2, BASE_TAB_Y, (BASE_H - TAB_T) / 2), (BASE_TAB_DEPTH, TAB_T, BASE_H - TAB_T),
                 PULP))
    top = BASE_H + LID_H
    zc, h = BASE_H + (LID_H - WALL) / 2, LID_H - WALL
    lid = [((0.0, 0.0, top - WALL / 2), (CARTON_D, CARTON_L, WALL), PULP),
           ((front + WALL / 2, 0.0, zc), (WALL, CARTON_L, h), PULP),
           ((-front - WALL / 2, 0.0, zc), (WALL, CARTON_L, h), PULP),
           ((0.0, CARTON_L / 2 - WALL / 2, zc), (CARTON_D, WALL, h), PULP),
           ((0.0, -CARTON_L / 2 + WALL / 2, zc), (CARTON_D, WALL, h), PULP),
           ((front - LID_TAB_DEPTH / 2, LID_TAB_Y, BASE_H + TAB_T / 2), (LID_TAB_DEPTH, LID_TAB_WIDTH, TAB_T), PULP)]
    flap = Flap("lid", pivot=(CARTON_D / 2, 0.0, BASE_H), axis="Y", lower=0.0, upper=180.0, mass=LID_MASS,
                hold=SNAP_HOLD, damping=HINGE_DAMPING, boxes=tuple(lid))
    return body("carton", pos=(CARTON_XY[0], CARTON_XY[1], TOP + SPAWN_GAP), yaw=CARTON_YAW, mass=BASE_MASS,
                boxes=base, flaps=[flap], color=PULP, angular_damping=0.5, **CARTON_FRICTION)


def _basket() -> list:
    """The basket as fixed boxes: a wicker floor and four walls, and the cloth lining lying on the floor."""
    bx, by = BASKET_XY
    ox, oy = BASKET_IN_X + 2 * BASKET_WALL, BASKET_IN_Y + 2 * BASKET_WALL

    def fixed(name, dx, dy, z0, z1, sx, sy, color):
        return Slick(name, pos=(bx + dx, by + dy, TOP + (z0 + z1) / 2), size=(sx, sy, z1 - z0), color=color,
                     **BASKET_FRICTION)

    return [fixed("basket_floor", 0.0, 0.0, 0.0, 0.006, ox, oy, WICKER),
            fixed("basket_cloth", 0.0, 0.0, 0.006, BASKET_FLOOR, BASKET_IN_X, BASKET_IN_Y, CLOTH),
            fixed("basket_far", ox / 2 - BASKET_WALL / 2, 0.0, 0.006, BASKET_H, BASKET_WALL, oy, WICKER),
            fixed("basket_near", -ox / 2 + BASKET_WALL / 2, 0.0, 0.006, BASKET_H, BASKET_WALL, oy, WICKER),
            fixed("basket_left", 0.0, oy / 2 - BASKET_WALL / 2, 0.006, BASKET_H, BASKET_IN_X, BASKET_WALL, WICKER),
            fixed("basket_right", 0.0, -oy / 2 + BASKET_WALL / 2, 0.006, BASKET_H, BASKET_IN_X, BASKET_WALL, WICKER)]


def egg_start(i: int) -> tuple:
    """World position of egg i's centre at the start, standing in its cup."""
    return carton_point(*CELLS[i], EGG_BOTTOM + EGG_LEN / 2 + SPAWN_GAP)


def _egg(i: int):
    # Angular damping stands in for the rolling resistance of a real egg on cloth, which PhysX does not model.
    # With PhysX's gyroscopic forces on and 32 solver iterations a lying capsule kept rolling by itself.
    return body(EGGS[i], pos=egg_start(i), mass=EGG_MASS, capsules=[((0.0, 0.0, 0.0), EGG_R, EGG_CYL, SHELL)],
                color=SHELL, linear_damping=0.05, angular_damping=EGG_ROLL_DAMPING, contact_offset=0.002,
                gyroscopic=False, solver_iterations=8, **EGG_FRICTION)


CARTON, BASKET = _carton(), _basket()
OBJECTS = [CARTON, *BASKET, *(_egg(i) for i in range(N))]
NAMES = ["carton", *EGGS]                      # the loose objects


# -- what happened during the trial --------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(latched=True, broken=set(), held=set(), last={}, worst={})


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()
    set_flap_hold(environment, "carton", "lid", SNAP_HOLD)


def impact_speed(z_before: float, speed_before: float, z_now: float) -> float:
    """The speed an egg reaches by the end of a control step: its speed at the start of the step plus what the
    height it lost in the step adds in free fall."""
    return math.sqrt(speed_before**2 + 2 * G * max(0.0, z_before - z_now))


@on_step
def _watch(environment) -> None:
    """Runs after every control step: lets the lid's snap go, and notes eggs that fell or are being held."""
    from yam_arena.scene import object_states   # importable only once the simulator has started

    states = object_states(environment, NAMES)
    if _STATE["latched"] and states["carton"].flaps["lid"] > SNAP_ANGLE:
        _STATE["latched"] = False
        set_flap_hold(environment, "carton", "lid", 0.0)
    hands = finger_points(environment)
    held = set()
    for name in EGGS:
        s = states[name]
        z, speed = float(s.pos[2]), float(np.linalg.norm(s.lin_vel))
        if name in _STATE["last"]:
            hit = impact_speed(*_STATE["last"][name], z)
            _STATE["worst"][name] = max(_STATE["worst"].get(name, 0.0), hit)
            if hit > BREAK_SPEED:
                _STATE["broken"].add(name)
        _STATE["last"][name] = (z, speed)
        if any(np.linalg.norm(np.asarray(h) - s.pos) < HELD_RADIUS for h in hands):
            held.add(name)
    _STATE["held"] = held


# -- scoring -------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(state) -> float:
    """Angle between an object's own vertical axis and the vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 2])))))


def egg_lowest(state) -> np.ndarray:
    """World position of the lowest point of an egg's shell."""
    axis = _rot(state.quat_xyzw)[:, 2]
    end = np.asarray(state.pos, float) - axis * (EGG_CYL / 2) * (1.0 if axis[2] >= 0 else -1.0)
    return end - np.array([0.0, 0.0, EGG_R])


def where(objects, name: str) -> str:
    """Where an egg is: "basket" (resting on the basket's lining), "carton" (still in a cup) or "loose"."""
    egg, carton = objects[name], objects["carton"]
    centre = np.asarray(egg.pos, float) - np.array([*BASKET_XY, TOP])
    low = egg_lowest(egg) - np.array([*BASKET_XY, TOP])
    c = _rot(carton.quat_xyzw).T @ (np.asarray(egg.pos, float) - np.asarray(carton.pos, float))
    if any(abs(c[0] - x) <= CELL / 2 and abs(c[1] - y) <= CELL / 2 for x, y in CELLS) and 0.0 < c[2] < BASE_H:
        return "carton"
    if abs(centre[0]) <= BASKET_IN_X / 2 and abs(centre[1]) <= BASKET_IN_Y / 2 and low[2] <= BASKET_FLOOR + ON_CLOTH:
        return "basket"
    return "loose"


def counts(objects, name: str) -> bool:
    """An egg counts when it is unbroken, rests on the basket's lining, is still and no gripper holds it."""
    return (name not in _STATE["broken"] and name not in _STATE["held"] and where(objects, name) == "basket"
            and float(np.linalg.norm(objects[name].lin_vel)) < REST_SPEED)


def score(objects) -> float:
    return sum(counts(objects, n) for n in EGGS) / N


def success(objects) -> bool:
    return all(counts(objects, n) for n in EGGS)


def report(objects) -> dict:
    """Where everything is, for result files."""
    eggs = {n: {"where": where(objects, n), "broken": n in _STATE["broken"], "held": n in _STATE["held"],
                "counts": counts(objects, n), "hardest_hit_m_s": round(_STATE["worst"].get(n, 0.0), 3),
                "pos_cm": [round(100 * float(v), 2) for v in objects[n].pos],
                "lean_deg": round(tilt_deg(objects[n]), 1)} for n in EGGS}
    carton = objects["carton"]
    return {"eggs": eggs, "lid_deg": round(float(getattr(carton, "flaps", {}).get("lid", float("nan"))), 1),
            "snap_let_go": not _STATE["latched"],
            "carton_pos_cm": [round(100 * float(v), 2) for v in carton.pos], "carton_tilt_deg": round(tilt_deg(carton), 1)}


def demo() -> dict:
    """yam_scripted plays straight-down pick-and-place only; the two-handed demo is demo.py. Returning no waypoints
    holds both arms still, which is a check of the scene by itself."""
    return {}
