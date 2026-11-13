"""On the station table: a slice of soft bread (10 x 10 cm) lies on a white plate on the table, a butter dish
holds a block of butter, and a butter knife lies on the table. The robot must butter the whole slice evenly, then put
the knife down off the bread. The bread is fragile and slides easily on the glazed plate, so an unheld slice is
dragged along by the butter and the knife and nothing is spread: one hand has to steady it while the other spreads, as
people do. Squeezing or crushing it, poking the crumb, pressing hard or dragging the knife fast tears it, which scores
0.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/butter-bread/butter_bread.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<INSTRUCTION>" --scorer task_score --max-steps 2400
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Contacts, PartState, Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
CONTROL_DT = 1.0 / 20  # the scorer runs after every control step; run with -E control_hz=20

INSTRUCTION = ("Butter the whole slice of bread on the plate evenly with the knife, without tearing it: the bread is "
               "soft. Then put the knife down off the bread, and leave the plate where it is.")

CRUST = (0.62, 0.40, 0.20)
CRUMB = (0.97, 0.94, 0.86)
BUTTER = (0.97, 0.72, 0.04)
LUMP = (0.86, 0.55, 0.02)
PLATE_WHITE = (0.95, 0.95, 0.93)
DISH_WHITE = (0.88, 0.90, 0.92)
HANDLE = (0.15, 0.15, 0.17)
STEEL = (0.72, 0.74, 0.77)

# -- the plate ---------------------------------------------------------------------------------------------------------
PLATE_XY = (0.30, 0.05)
PLATE_FRICTION = 0.40                  # glazed plate and bread crust: the bread on the plate 0.4
PLATE_R, PLATE_H = 0.095, 0.012
PLATE_TOP = TOP + PLATE_H
PLATE = part("plate", pos=(*PLATE_XY, TOP), boxes=(), mass=0.35, color=PLATE_WHITE, friction=PLATE_FRICTION,
             cylinders=[((0.0, 0.0, PLATE_H / 2), PLATE_R, PLATE_H, PLATE_WHITE)])

# -- the bread ---------------------------------------------------------------------------------------------------------
# Frame: origin at the centre of the slice's underside. A crust frame on a base slab, and the crumb inside the frame on
# three springs in series: sideways along x and along y (the crumb shearing against the crust) and down. The forces on
# the crumb show as its displacement: pressed 0.4 mm (0.2 N) is the knife touching it, 5 mm (2.5 N) down or 1.5 mm
# (1.5 N) sideways tears it. (The arms press with about 1 N when commanded a few millimetres into a surface, so the
# springs are scaled to what they can do.) The robot's own links are felt by contact sensors: more than 3 N on the
# crust (a squeeze, a pinch, a crushing press) or 0.5 N on the crumb (a finger poking it) tears the bread too.
BREAD_SIDE, BREAD_H = 0.100, 0.012
BASE_H = 0.008
CRUST_W = 0.010
CRUMB_SIDE = BREAD_SIDE - 2 * CRUST_W  # 80 mm
CRUMB_H = BREAD_H - BASE_H
CRUMB_K = 500.0                         # N/m, down
SHEAR_K = 1000.0                        # N/m, sideways
PRESSED = 0.0004                        # m (0.2 N): the crumb pressed at least this far counts as the blade on it
TEAR_DOWN = 0.005                       # m (2.5 N)
TEAR_SHEAR = 0.0015                     # m (1.5 N)
CRUST_TEAR_N = 3.0                      # N from the robot's links on the crust
CRUMB_POKE_N = 0.5                      # N from the robot's links on the crumb
GRID = 5                                # the crumb is scored as 5 x 5 squares, 16 mm on a side
CELL = CRUMB_SIDE / GRID
BREAD_MASS = 0.030
BREAD_XY = PLATE_XY
CRUMB_Z = BASE_H + CRUMB_H / 2


def _cell_centre(i: int, j: int) -> tuple[float, float]:
    return (-CRUMB_SIDE / 2 + (i + 0.5) * CELL, -CRUMB_SIDE / 2 + (j + 0.5) * CELL)


CELL_DECALS = tuple((f"c{i}{j}", (*_cell_centre(i, j), BREAD_H + 0.0006), (CELL - 0.0006, CELL - 0.0006, 0.0010),
                     BUTTER, False) for i in range(GRID) for j in range(GRID))
LUMP_DECALS = tuple((f"l{i}{j}", (*_cell_centre(i, j), BREAD_H + 0.0015), (0.010, 0.010, 0.0020), LUMP, False)
                    for i in range(GRID) for j in range(GRID))
SHEAR_X = Slider("shear_x", rest=(0.0, 0.0, CRUMB_Z), axis="X", limits=(-0.004, 0.004), rate=SHEAR_K, damping=3.0,
                 mass=0.002)
SHEAR_Y = Slider("shear_y", rest=(0.0, 0.0, CRUMB_Z), axis="Y", limits=(-0.004, 0.004), rate=SHEAR_K, damping=3.0,
                 mass=0.002, parent="shear_x")
# Friction: bread on the glazed plate 0.4 (the crust and plate both 0.4); the steel blade on the buttered crumb 0.2,
# butter acting as a lubricant (the crumb's own material, combined by "min" so it holds against the knife's 1.0).
CRUMB_FRICTION = 0.20
CRUMB_LINK = Slider("crumb", rest=(0.0, 0.0, CRUMB_Z), parent="shear_y", surface_friction=CRUMB_FRICTION,
                    surface_combine="min",
                    boxes=(((0.0, 0.0, CRUMB_Z), (CRUMB_SIDE, CRUMB_SIDE, CRUMB_H), CRUMB),),
                    axis="Z", limits=(-0.008, 0.0), rate=CRUMB_K, damping=5.0, mass=0.005,
                    decals=CELL_DECALS + LUMP_DECALS)
_half = BREAD_SIDE / 2 - CRUST_W / 2
BREAD = part("bread", pos=(*BREAD_XY, PLATE_TOP), mass=BREAD_MASS, color=CRUST, friction=PLATE_FRICTION, contact_report=True,
             boxes=[((0.0, 0.0, BASE_H / 2), (BREAD_SIDE, BREAD_SIDE, BASE_H), CRUST),
                    ((0.0, _half, BASE_H + CRUMB_H / 2), (BREAD_SIDE, CRUST_W, CRUMB_H), CRUST),
                    ((0.0, -_half, BASE_H + CRUMB_H / 2), (BREAD_SIDE, CRUST_W, CRUMB_H), CRUST),
                    ((_half, 0.0, BASE_H + CRUMB_H / 2), (CRUST_W, CRUMB_SIDE, CRUMB_H), CRUST),
                    ((-_half, 0.0, BASE_H + CRUMB_H / 2), (CRUST_W, CRUMB_SIDE, CRUMB_H), CRUST)],
             sliders=(SHEAR_X, SHEAR_Y, CRUMB_LINK))

# -- the butter dish: a block of soft butter on a spring, so scraping it needs real pressure --------------------------
DISH_XY = (0.27, -0.17)
DISH_SIZE = (0.130, 0.090, 0.008)
PAT = (0.075, 0.045, 0.025)            # the block of butter
PAT_TOP = DISH_SIZE[2] + PAT[2]        # 33 mm above the table
BUTTER_K = 800.0
SCRAPE_PRESSED = 0.0004                # m (0.3 N)
BUTTER_LINK = Slider("block", rest=(0.0, 0.0, DISH_SIZE[2] + PAT[2] / 2),
                     boxes=(((0.0, 0.0, DISH_SIZE[2] + PAT[2] / 2), PAT, BUTTER),),
                     axis="Z", limits=(-0.006, 0.0), rate=BUTTER_K, damping=5.0, mass=0.005)
DISH = part("butter_dish", pos=(*DISH_XY, TOP), mass=0.45, color=DISH_WHITE, friction=0.8,
            boxes=[((0.0, 0.0, DISH_SIZE[2] / 2), DISH_SIZE, DISH_WHITE)], sliders=(BUTTER_LINK,))

# -- the knife: an offset butter spreader. Origin under the handle's centre at the blade's underside; the blade runs
# along +x, 8 mm below the handle (a neck steps down to it), so it lies flat on the bread with the handle clear of the
# crust. The blade (72 mm) fits inside the crumb (80 mm). Lying on the table it rests on the blade and the handle's end.
HANDLE_SIZE = (0.100, 0.018, 0.016)
HANDLE_Z0 = 0.008                      # the handle's underside above the blade's
BLADE_LEN, BLADE_W, BLADE_T = 0.072, 0.016, 0.002
BLADE_X0 = HANDLE_SIZE[0] / 2 + 0.006  # the blade starts 6 mm past the handle, under the neck
NECK = ((HANDLE_SIZE[0] / 2 + 0.004, 0.0, (BLADE_T + HANDLE_Z0 + HANDLE_SIZE[2]) / 2),
        (0.016, 0.012, HANDLE_Z0 + HANDLE_SIZE[2] - BLADE_T))
KNIFE_XY = (0.17, -0.12)
KNIFE_YAW = math.pi / 2                # the blade points left (+y), the handle toward the right arm
KNIFE = part("knife", pos=(*KNIFE_XY, TOP), yaw=KNIFE_YAW, mass=0.040, color=HANDLE, friction=1.0, contact_report=True,
             boxes=[((0.0, 0.0, HANDLE_Z0 + HANDLE_SIZE[2] / 2), HANDLE_SIZE, HANDLE),
                    (NECK[0], NECK[1], STEEL),
                    ((BLADE_X0 + BLADE_LEN / 2, 0.0, BLADE_T / 2), (BLADE_LEN, BLADE_W, BLADE_T), STEEL)],
             decals=(("load", (BLADE_X0 + 0.040, 0.0, BLADE_T + 0.001), (0.036, 0.014, 0.002), BUTTER, False),))
# Points across the blade's mid-plane that scrape and spread (knife frame), each standing for 1/15 of the blade.
BLADE_PTS = np.array([(BLADE_X0 + BLADE_LEN * f, BLADE_W * g, BLADE_T / 2)
                      for f in (0.1, 0.3, 0.5, 0.7, 0.9) for g in (-0.45, 0.0, 0.45)])
FLAT = math.radians(30.0)              # the blade within 30 degrees of flat
CLOSE = 0.004                          # a blade point within 4 mm of the surface it is on
SLIDING = 0.005                        # m/s: the blade moving over the surface, relative to it

OBJECTS = [PLATE, BREAD, DISH, KNIFE]

# -- butter, as an amount: millimetres and cubic millimetres -----------------------------------------------------------
A_BLADE = BLADE_LEN * BLADE_W * 1e6    # 1152 mm2
A_POINT = A_BLADE / len(BLADE_PTS)     # 77 mm2 of blade per point
A_CELL = (CELL * 1000) ** 2            # 256 mm2
SCRAPE_RATE = 2.7                      # mm3 picked up per point per mm scraped with the block pressed 1 mm or more
SCRAPE_FULL = 0.001                    # m of block pressing for the full rate (less pressing, proportionally less)
BLADE_CAP = 1500.0                     # mm3 the blade can carry (a 1.3 mm layer)
TRANSFER = 0.04                        # per mm slid: the share of the difference between the blade's butter layer and
                                       # the square's that moves across (thicker gives to thinner, both ways)
COVERED_MM = 0.2                       # a square is buttered at 0.2 mm or more
LUMP_MM = 1.2                          # and lumpy above 1.2 mm
LOADED_MM3 = 100.0                     # the knife counts as loaded with this much
VISCOUS = 16.0                         # N per m/s: butter's drag on the crumb (and back on the knife), full blade

# -- scoring -----------------------------------------------------------------------------------------------------------
HALF, MOST, ALL = 10, 20, 23           # squares buttered for 0.5, 0.75 and 1.0
FLAT_TILT = math.radians(10.0)
LEVEL_TOL = 0.004
PLATE_SLACK = 0.030                    # the plate's centre moved less than 3 cm from where it started
ON_PLATE = PLATE_R - 0.035             # 60 mm: the bread's centre within this of the plate's centre
KNIFE_DOWN_Z = 0.040                   # the knife's lowest point within 4 cm of the table: lying on it, the dish or plate
CLEAR = BREAD_SIDE / 2 + 0.005         # no knife point over the bread or within 5 mm of its edge

_STATE: dict = {}
_SHOWN: dict = {"cells": set(), "lumps": set(), "load": False}  # what is drawn now (kept across resets)
_CONTACTS: dict = {}


def _fresh() -> dict:
    return dict(h=np.zeros((GRID, GRID)), volume=0.0, loaded=False, torn=None, max_shear=0.0,
                max_press=0.0)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.clear()
    _STATE.update(_fresh())
    if environment is not None:  # hide every decal, whatever the last run left showing, and stop any butter drag
        from partkit import push, set_visible

        set_visible("bread", [d[0] for d in CELL_DECALS + LUMP_DECALS], False)
        set_visible("knife", "load", False)
        _SHOWN["cells"].clear()
        _SHOWN["lumps"].clear()
        _SHOWN["load"] = False
        push(environment, "bread", "crumb", (0.0, 0.0, 0.0))
        push(environment, "knife", "body", (0.0, 0.0, 0.0))


_reset_state()


def tilt(s: PartState) -> float:
    return math.acos(max(-1.0, min(1.0, float(s.rotation()[2, 2]))))


def _blade_flat(knife: PartState) -> bool:
    return abs(float(knife.rotation()[2, 2])) >= math.cos(FLAT)


def _point_vel(s: PartState, world_pt) -> np.ndarray:
    return np.asarray(s.lin_vel, float) + np.cross(np.asarray(s.ang_vel, float), np.asarray(world_pt) - s.pos)


def _on_surface(knife: PartState, under: PartState, top_z: float, half_x: float, half_y: float):
    """Blade points lying on ``under``'s top face (its frame: within the footprint and ``CLOSE`` of ``top_z``):
    yields each point's (x, y) in ``under``'s frame and its velocity relative to ``under`` (world)."""
    for p in BLADE_PTS:
        w = knife.to_world(p)
        local = under.to_local(w)
        if abs(local[0]) > half_x or abs(local[1]) > half_y or abs(local[2] - top_z) > CLOSE:
            continue
        yield local[0], local[1], _point_vel(knife, w) - _point_vel(under, w)


def _tear(reason: str) -> None:
    if _STATE["torn"] is None:
        _STATE["torn"] = reason


def advance(knife: PartState, bread: PartState, dish: PartState, contacts=None, dt: float = CONTROL_DT):
    """One control step: tearing, scraping butter off the block, and spreading it. Returns the
    butter's drag on the crumb (N, world), to be applied to the crumb and, reversed, to the knife."""
    c = contacts or {}
    _STATE["steps"] = _STATE.get("steps", 0) + 1
    crumb_z = bread.sliders.get("crumb", 0.0)
    shear = math.hypot(bread.sliders.get("shear_x", 0.0), bread.sliders.get("shear_y", 0.0))
    _STATE["max_press"] = max(_STATE["max_press"], -crumb_z)
    _STATE["max_shear"] = max(_STATE["max_shear"], shear)
    if crumb_z <= -TEAR_DOWN:
        _tear("pressed too hard")
    if shear >= TEAR_SHEAR:
        _tear("dragged too hard")
    if c.get("crust_n", 0.0) > CRUST_TEAR_N:
        _tear("squeezed or crushed")
    if c.get("crumb_n", 0.0) > CRUMB_POKE_N:
        _tear("poked")
    drag = np.zeros(3)
    if not _blade_flat(knife):
        return drag
    # Scraping: points sliding over the pressed block pick butter up, in proportion to how far they slide and how
    # hard the block is pressed (up to 1 mm).
    block = dish.sliders.get("block", 0.0)
    if block <= -SCRAPE_PRESSED:
        depth = min(1.0, -block / SCRAPE_FULL)
        for _, _, rel in _on_surface(knife, dish, PAT_TOP + block, PAT[0] / 2, PAT[1] / 2):
            v = float(np.linalg.norm(rel))
            if v >= SLIDING:
                _STATE["volume"] = min(BLADE_CAP, _STATE["volume"] + SCRAPE_RATE * depth * v * dt * 1000)
        if _STATE["volume"] >= LOADED_MM3:
            _STATE["loaded"] = True
    # Spreading: each point sliding over the pressed crumb evens out the butter between the blade and the square under
    # it; butter between the blade and the crumb drags on the crumb.
    if crumb_z <= -PRESSED:
        on, rel_sum = 0, np.zeros(3)
        for x, y, rel in _on_surface(knife, bread, BREAD_H + crumb_z, CRUMB_SIDE / 2, CRUMB_SIDE / 2):
            v = float(np.linalg.norm(rel))
            i = min(GRID - 1, int((x + CRUMB_SIDE / 2) // CELL))
            j = min(GRID - 1, int((y + CRUMB_SIDE / 2) // CELL))
            layer = _STATE["volume"] / A_BLADE
            if v >= SLIDING:
                moved = (layer - _STATE["h"][i, j]) * A_POINT * min(1.0, TRANSFER * v * dt * 1000)
                moved = min(moved, _STATE["volume"])
                _STATE["h"][i, j] += moved / A_CELL
                _STATE["volume"] -= moved
            if layer > 0 or _STATE["h"][i, j] > 0:
                on += 1
                drag += VISCOUS / len(BLADE_PTS) * rel
                rel_sum += rel
        drag[2] = 0.0
        drag = _stable(drag, rel_sum / max(on, 1), crumb_z, dt, float(np.linalg.norm(np.asarray(bread.lin_vel)[:2])))
    return drag


def _stable(drag: np.ndarray, rel: np.ndarray, crumb_z: float, dt: float, bread_speed: float = 0.0) -> np.ndarray:
    """Cap the butter's drag so it cannot throw the slice. The drag is held for a whole control step (50 ms) on a 30 g
    slice, so left alone it would keep pushing a sliding slice long after the slice had caught up with the knife. While
    the slice slides, the drag is capped at what the plate's friction resists (the slice's weight plus the knife's
    press) plus what brings the slice up to the knife's speed within the step, an implicit step: the slice creeps along
    slower than the knife but never overtakes it. A slice that is not sliding (held, or not yet moving) feels the full
    drag, so a fast stroke still shears and tears its crumb."""
    if bread_speed < SLIDING:
        _STATE["max_drag"] = max(_STATE.get("max_drag", 0.0), float(np.linalg.norm(drag[:2])))
        return drag
    resist = PLATE_FRICTION * (BREAD_MASS * 9.81 + CRUMB_K * max(0.0, -crumb_z))
    cap = resist + BREAD_MASS * float(np.linalg.norm(rel[:2])) / dt
    mag = float(np.linalg.norm(drag[:2]))
    _STATE["max_drag"] = max(_STATE.get("max_drag", 0.0), min(mag, cap))
    return drag * (cap / mag) if mag > cap else drag


def covered() -> int:
    return int((_STATE["h"] >= COVERED_MM).sum())


def lumps() -> int:
    return int((_STATE["h"] > LUMP_MM).sum())


def _show(environment) -> None:
    """Draw the butter: the buttered squares, any lumps, and the load on the blade."""
    from partkit import set_visible

    h = _STATE["h"]
    cells = {(i, j) for i in range(GRID) for j in range(GRID) if h[i, j] >= COVERED_MM}
    lump = {(i, j) for i in range(GRID) for j in range(GRID) if h[i, j] > LUMP_MM}
    for key, now, prefix in (("cells", cells, "c"), ("lumps", lump, "l")):
        on, off = now - _SHOWN[key], _SHOWN[key] - now
        if on:
            set_visible("bread", [f"{prefix}{i}{j}" for i, j in on], True)
        if off:
            set_visible("bread", [f"{prefix}{i}{j}" for i, j in off], False)
        _SHOWN[key] = set(now)
    loaded = _STATE["volume"] >= LOADED_MM3
    if loaded != _SHOWN["load"]:
        set_visible("knife", "load", loaded)
        _SHOWN["load"] = loaded


def _read_contacts(environment) -> dict:
    """Forces from the robot's links on the bread's crust and crumb."""
    if not _CONTACTS:
        _CONTACTS["bread"] = Contacts("bread", ["body", "crumb"], ["left_robot", "right_robot"])
    fm, _ = _CONTACTS["bread"].read(environment, float(environment.unwrapped.physics_dt))
    crust_n, crumb_n = (float(np.linalg.norm(fm[k].sum(axis=0))) for k in (0, 1))
    return {"crust_n": crust_n, "crumb_n": crumb_n}


@on_step
def _butter(environment) -> None:
    from partkit import _SPECS, _part_state, push

    s = {n: _part_state(environment, _SPECS[n]) for n in ("knife", "bread", "butter_dish")}
    drag = advance(s["knife"], s["bread"], s["butter_dish"], _read_contacts(environment))
    push(environment, "bread", "crumb", tuple(drag))
    push(environment, "knife", "body", tuple(-drag))
    _show(environment)


def bread_on_plate(bread: PartState, plate: PartState) -> bool:
    """The slice lies flat on the plate, butter side up, inside its rim."""
    d = np.hypot(*(np.asarray(bread.pos[:2]) - np.asarray(plate.pos[:2])))
    return bool(d <= ON_PLATE and tilt(bread) <= FLAT_TILT
                and abs(float(bread.pos[2]) - (float(plate.pos[2]) + PLATE_H)) <= LEVEL_TOL)


def plate_in_place(plate: PartState) -> bool:
    """The plate lies flat on the table, its centre less than 3 cm from where it started."""
    d = float(np.hypot(*(np.asarray(plate.pos[:2]) - np.asarray(PLATE_XY))))
    return bool(d < PLATE_SLACK and tilt(plate) <= FLAT_TILT and abs(float(plate.pos[2]) - TOP) <= LEVEL_TOL)


def knife_down(knife: PartState, bread: PartState) -> bool:
    """The knife lies still, low (on the table, dish or plate), with no part of it over the bread."""
    # Off the table (dropped): the knife's origin, at the blade's underside, more than 2 cm below the table top. (low is
    # a box around the origin, not the knife's shape, so it reads about 1.5 cm low for a knife lying on the table.)
    if not knife.resting() or float(knife.low[2]) > TOP + KNIFE_DOWN_Z or float(knife.pos[2]) < TOP - 0.02:
        return False
    pts = [(x, y, HANDLE_Z0) for x in (-HANDLE_SIZE[0] / 2, 0.0, HANDLE_SIZE[0] / 2) for y in (-0.009, 0.009)]
    pts += [tuple(p) for p in BLADE_PTS]
    for p in pts:
        local = bread.to_local(knife.to_world(p))
        if abs(local[0]) <= CLEAR and abs(local[1]) <= CLEAR:
            return False
    return True


def _update(objects) -> tuple[float, bool]:
    bread, plate, knife = objects["bread"], objects["plate"], objects["knife"]
    if _STATE["torn"] or not bread_on_plate(bread, plate):
        return 0.0, False
    n = covered()
    if n >= ALL and lumps() == 0 and plate_in_place(plate) and bread.resting() and knife_down(knife, bread):
        s = 1.0
    elif n >= MOST:
        s = 0.75
    elif n >= HALF:
        s = 0.5
    else:
        s = 0.25 if _STATE["loaded"] else 0.0
    return s, s >= 1.0


def success(objects) -> bool:
    """At least 23 of 25 squares buttered and none lumpy, the bread flat and still on the plate, the plate in place,
    the knife put down off the bread, and the bread never torn; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if the bread was torn or is not flat on the plate. Otherwise 0.25 once the knife has been loaded with butter;
    0.5 at 10 squares buttered; 0.75 at 20; 1.0 at 23 with no lumps, the plate in place and the knife down off the
    bread."""
    return _update(objects)[0]


def distance(objects) -> float:
    """Squares still to butter (of the 23 needed), as a fraction."""
    return max(0, ALL - covered()) / ALL


# -- offline checks ----------------------------------------------------------------------------------------------------
def _quat(roll=0.0, pitch=0.0, yaw=0.0):  # yaw, then pitch (about y), then roll (about x), all world axes
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return np.array([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])


def _state(spec, pos, q=None, vel=(0, 0, 0), sliders=None):
    q = _quat() if q is None else q
    probe = PartState(pos=np.asarray(pos, float), quat_xyzw=q, lin_vel=np.asarray(vel, float),
                      ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (0, 1)]) * \
        np.array(spec.size) * np.array([0.5, 0.5, 1.0])
    pts = np.array([probe.to_world(c) for c in corners])
    return PartState(pos=probe.pos, quat_xyzw=q, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), sliders=sliders or {})


def _knife_at(x, y, z_mid, vel=(0, 0, 0), roll_deg=0.0, yaw_deg=0.0):
    """The knife placed so the blade's mid-plane is at height ``z_mid`` with the blade's middle at (x, y)."""
    q = _quat(roll=math.radians(roll_deg), yaw=math.radians(yaw_deg))
    probe = PartState(pos=np.zeros(3), quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3),
                      high=np.zeros(3))
    mid = probe.to_world((BLADE_X0 + BLADE_LEN / 2, 0.0, BLADE_T / 2))
    return _state(KNIFE, (x - mid[0], y - mid[1], z_mid - mid[2]), q, vel)


def _bread(dx=0.0, dy=0.0, crumb=-0.001, vel=(0, 0, 0), tilt_deg=0.0, dz=0.0, shear=(0.0, 0.0)):
    return _state(BREAD, (PLATE_XY[0] + dx, PLATE_XY[1] + dy, PLATE_TOP + dz), _quat(roll=math.radians(tilt_deg)), vel,
                  {"crumb": crumb, "shear_x": shear[0], "shear_y": shear[1]})


def _dish(block=-0.0012):
    return _state(DISH, (*DISH_XY, TOP), sliders={"block": block})


def scrape(stroke=0.040, speed=0.02, block=-0.0012):
    """A scrape across the block's width (along y) with the blade along x, flat and pressed."""
    n = round(stroke / speed / CONTROL_DT)
    for k in range(n):
        y = DISH_XY[1] - stroke / 2 + speed * CONTROL_DT * k
        advance(_knife_at(DISH_XY[0], y, TOP + PAT_TOP + block, (0, speed, 0)), _bread(), _dish(block))


def stroke(y0=-0.031, y1=0.031, speed=0.02, bread_vel=(0, 0, 0), crumb=-0.001, roll_deg=0.0):
    """A spreading stroke across the bread along its y, the blade along its x over all five rows, pressed."""
    n = max(1, round(abs(y1 - y0) / speed / CONTROL_DT))
    drags = []
    for k in range(n):
        y = PLATE_XY[1] + y0 + (y1 - y0) * k / n
        v = (0, math.copysign(speed, y1 - y0), 0)
        drags.append(advance(_knife_at(PLATE_XY[0], y, PLATE_TOP + BREAD_H + crumb + BLADE_T / 2, v, roll_deg),
                             _bread(crumb=crumb, vel=bread_vel), _dish(0.0)))
    return drags


def self_test() -> int:
    """The butter and tearing models and the scoring rules, on hand-built states."""
    checks = 0

    def check(cond, msg):
        nonlocal checks
        assert cond, (msg, {k: v for k, v in _STATE.items() if k != "h"}, np.round(_STATE["h"], 2))
        checks += 1

    # Scraping loads in proportion to travel and pressing.
    _reset_state(); scrape()
    full = _STATE["volume"]
    check(1300 <= full <= BLADE_CAP and _STATE["loaded"], "a 4 cm scrape pressed 1.2 mm loads about 1.6 cm3")
    _reset_state(); scrape(block=-0.0005)
    check(abs(_STATE["volume"] - full * 0.5) < 0.1 * full, "pressed 0.5 mm loads half as much")
    _reset_state(); scrape(block=-0.0002)
    check(_STATE["volume"] == 0 and not _STATE["loaded"], "not pressed: nothing")
    _reset_state()
    for _ in range(40):
        advance(_knife_at(DISH_XY[0], DISH_XY[1], TOP + PAT_TOP - 0.0012), _bread(), _dish())
    check(_STATE["volume"] == 0, "pressed but not moving: nothing")
    _reset_state()
    for k in range(40):
        advance(_knife_at(DISH_XY[0], DISH_XY[1] - 0.02 + 0.001 * k, TOP + PAT_TOP - 0.0012, (0, 0.02, 0), roll_deg=60),
                _bread(), _dish())
    check(_STATE["volume"] == 0, "the blade on its edge: nothing")
    _reset_state(); scrape(); scrape(); scrape()
    check(_STATE["volume"] == BLADE_CAP, "the blade carries at most 2 cm3")

    # Spreading: conserved, front squares thicker than far ones, a second load and stroke even it out.
    _reset_state(); scrape()
    v0 = _STATE["volume"]
    drags = stroke()
    total = _STATE["h"].sum() * A_CELL + _STATE["volume"]
    check(abs(total - v0) < 1e-6 * v0, "butter is conserved")
    check(10 <= covered() < 20 and _STATE["h"][:, 1].mean() > _STATE["h"][:, 4].mean(), "one stroke: thinner further on")
    check(max(np.linalg.norm(d) for d in drags) < 0.4, "butter drag at 2 cm/s is about 0.3 N")
    scrape(); stroke()
    check(covered() < 25, "two loads leave the far edge thin")
    scrape(); stroke()
    check(covered() == 25 and lumps() == 0, "three loads and strokes butter the slice evenly")
    check(not _STATE["torn"], "gently done")
    # Back and forth over butter evens a lump.
    _reset_state()
    _STATE["h"][2, 2] = 2.0
    _STATE["volume"] = 300.0
    check(lumps() == 1, "a lump")
    stroke(); stroke(0.031, -0.031)
    check(lumps() == 0 and _STATE["h"][2, 2] < LUMP_MM, "spreading over a lump flattens it")
    # The unheld bread moving with the knife: no relative motion, no butter.
    _reset_state(); scrape(); stroke(bread_vel=(0, 0.02, 0))
    check(covered() == 0, "the bread carried along with the knife: nothing")
    _reset_state(); scrape(); stroke(crumb=0.0)
    check(covered() == 0, "hovering without pressing: nothing")
    _reset_state(); stroke()
    check(covered() == 0, "an unloaded knife: nothing")
    _reset_state(); scrape(); stroke(roll_deg=50)
    check(covered() == 0, "the blade on its edge: nothing")
    # Viscous drag grows with speed: fast strokes drag the crumb hard.
    _reset_state(); scrape()
    d = stroke(speed=0.15)
    check(max(np.linalg.norm(x) for x in d) > 1.0, "a 15 cm/s stroke drags over 1 N")

    # The drag cap: a held slice feels the full drag, an unheld slice is never pushed faster than the knife.
    _reset_state()
    d = _stable(np.array([0.3, 0.0, 0.0]), np.array([0.02, 0.0, 0.0]), -0.001, CONTROL_DT)
    check(abs(d[0] - 0.3) < 1e-9, "a 0.3 N drag at 2 cm/s is under the cap")
    d = _stable(np.array([3.0, 0.0, 0.0]), np.array([0.02, 0.0, 0.0]), -0.001, CONTROL_DT, bread_speed=0.01)
    check(d[0] < 0.4, "on a sliding slice a large drag is capped near the plate's friction")
    d = _stable(np.array([3.0, 0.0, 0.0]), np.array([0.02, 0.0, 0.0]), -0.001, CONTROL_DT)
    check(d[0] == 3.0, "on a still (held) slice it is not")

    # Tearing.
    for name, kw, contacts in (("pressed 6 mm", dict(crumb=-0.006), {}),
                               ("sheared 2 mm", dict(shear=(0.0014, 0.0014)), {}),
                               ("crust squeezed 4 N", {}, {"crust_n": 4.0}),
                               ("crumb poked 1 N", {}, {"crumb_n": 1.0})):
        _reset_state()
        advance(_knife_at(0.17, -0.12, TOP + 0.1), _bread(**kw), _dish(0.0), contacts)
        check(_STATE["torn"], name)
    _reset_state()
    advance(_knife_at(0.17, -0.12, TOP + 0.1), _bread(crumb=-0.002, shear=(0.001, 0.0)), _dish(0.0),
            {"crust_n": 2.0, "crumb_n": 0.3})
    check(not _STATE["torn"], "gentle contact does not tear")

    # Scoring.
    plate = _state(PLATE, (*PLATE_XY, TOP))
    knife_table = _state(KNIFE, (*KNIFE_XY, TOP), _quat(yaw=KNIFE_YAW))
    knife_on_bread = _knife_at(*PLATE_XY, PLATE_TOP + BREAD_H + BLADE_T / 2)
    knife_held = _state(KNIFE, (*KNIFE_XY, TOP + 0.10), _quat(yaw=KNIFE_YAW))
    order = [(i, j) for i in range(GRID) for j in range(GRID)]

    def hist(n=0, loaded=False, lumpy=0, torn=None):
        h = _fresh()
        for k, (i, j) in enumerate(order[:n]):
            h["h"][i, j] = 1.5 if k < lumpy else 0.6
        h.update(loaded=loaded or n > 0, torn=torn)
        return h

    cases = {
        "start": (hist(), _bread(crumb=0.0), plate, knife_table, 0.0, False),
        "knife loaded": (hist(loaded=True), _bread(), plate, knife_table, 0.25, False),
        "9 squares": (hist(9), _bread(), plate, knife_on_bread, 0.25, False),
        "10 squares": (hist(10), _bread(), plate, knife_on_bread, 0.5, False),
        "20 squares, knife down": (hist(20), _bread(), plate, knife_table, 0.75, False),
        "23, knife still on the bread": (hist(23), _bread(), plate, knife_on_bread, 0.75, False),
        "23, knife held up": (hist(23), _bread(), plate, knife_held, 0.75, False),
        "23, knife down": (hist(23), _bread(), plate, knife_table, 1.0, True),
        "25, knife down": (hist(25), _bread(), plate, knife_table, 1.0, True),
        "25, knife dropped off the table": (hist(25), _bread(), plate, _state(KNIFE, (0.20, -0.40, TOP - 0.10)), 0.75, False),
        "25 with a lump": (hist(25, lumpy=1), _bread(), plate, knife_table, 0.75, False),
        "plate pushed 4 cm": (hist(25), _bread(dx=0.04), _state(PLATE, (PLATE_XY[0] + 0.04, PLATE_XY[1], TOP)), knife_table, 0.75, False),
        "plate nudged 2 cm": (hist(25), _bread(dx=0.02), _state(PLATE, (PLATE_XY[0] + 0.02, PLATE_XY[1], TOP)), knife_table, 1.0, True),
        "bread at the plate's edge": (hist(25), _bread(dy=0.07), plate, knife_table, 0.0, False),
        "bread off the plate": (hist(25), _bread(dy=0.12, dz=-PLATE_H), plate, knife_table, 0.0, False),
        "bread tipped up": (hist(25), _bread(tilt_deg=20, dz=0.015), plate, knife_table, 0.0, False),
        "bread still moving": (hist(25), _bread(vel=(0.02, 0, 0)), plate, knife_table, 0.75, False),
        "torn": (hist(25, torn="poked"), _bread(), plate, knife_table, 0.0, False),
    }
    for name, (h, b, p, kn, want, want_success) in cases.items():
        _STATE.clear()
        _STATE.update(h)
        objects = {"bread": b, "plate": p, "knife": kn, "butter_dish": _dish(0.0)}
        got, got_success = score(objects), success(objects)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    _reset_state()
    return checks + len(cases)


if __name__ == "__main__":
    from yam_arena.task_file import load

    print(f"butter, tearing and scoring: {self_test()} checks pass")
    task = load(__file__)
    print(f"loaded {task.name}: objects {task.names}, free {task.free_objects}")
