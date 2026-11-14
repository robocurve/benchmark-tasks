"""On the station table, in front of the arms: a range-top griddle (a 7-gauge stainless plate, 56.8 x 31.1 cm cooking
surface, with a 7.6 cm grease tray across its near end, toward the robot). The near end of the plate, where the last
order was cooked, is covered with charred debris stuck to the plate: 80 small carbon flakes over a 12 x 16 cm area and
eight larger crusts on two 5 x 5 cm spots where the patties sat. A 4 in. (10.2 cm) griddle scraper lies on the bench to
the right.

The job is to scrape the griddle clean: break the debris free and push it along the plate with the scraper, over the
lip into the grease tray, without flinging it off the griddle or gouging the plate, then set the scraper down off the
griddle.

Every flake and crust is its own rigid body (debriskit), bonded to the plate by a breakable joint that the physics
solver breaks when the blade pushes the piece harder than its bond; from then on it is a free body on the greased plate
with real friction. The scraper's blade has to meet the pieces, break them free and push them; they fall into the tray
under gravity when pushed over the lip. The score reads where they really are, by mass: in the tray, still on the
griddle, or off it. Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import debriskit as dk  # noqa: E402
import guardkit as gk  # noqa: E402
import scrapekit as sk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Scrape the griddle clean into the grease tray with the scraper."

# -- the griddle (world frame: x away from the arms, y to the robot's left, z up; m) ----------------------------------
# A range-top add-on griddle (Avantco AOGT1227 class: 27 x 12-1/4 in. overall, 22-3/8 x 12-1/4 in. cooking surface,
# 7-gauge type 430 stainless, a 3 x 12-1/4 x 1-1/4 in. grease tray across one end), set on the bench on its own frame
# with the plate's top 5.0 cm above the bench, tray end toward the robot. Fixed (clamped to the bench in the real build).
PLATE_Z = TOP + 0.050  # the plate's top
PLATE_T = 0.0046  # 7 gauge
LIP_X = 0.150  # the lip: the plate's near edge, over the tray
PLATE_LX = 0.568  # 22-3/8 in. along x
PLATE_W = 0.311  # 12-1/4 in. along y
PLATE_YC = -0.020
PLATE_Y0 = PLATE_YC - PLATE_W / 2
TRAY_X0 = LIP_X - 0.076  # 3 in. tray
TRAY_DEPTH = 0.032  # 1-1/4 in. below the plate's top
GUARD_H = 0.050  # splash guards 5.0 cm above the plate on both sides and the far end
GUARD_T = 0.003
STEEL = (0.72, 0.73, 0.74)
DARK_STEEL = (0.45, 0.46, 0.47)

TRAY = Box("tray", pos=((TRAY_X0 + LIP_X) / 2, PLATE_YC, (TOP + PLATE_Z - TRAY_DEPTH) / 2),
           size=(LIP_X - TRAY_X0, PLATE_W, PLATE_Z - TRAY_DEPTH - TOP), color=DARK_STEEL)  # tray floor on the frame
TRAY_WALL = Box("tray_wall", pos=(TRAY_X0 - GUARD_T / 2, PLATE_YC, (TOP + PLATE_Z) / 2),
                size=(GUARD_T, PLATE_W, PLATE_Z - TOP), color=STEEL)  # the tray's near wall, level with the plate
_GUARD_X0 = TRAY_X0 - GUARD_T
_GUARD_LEN = LIP_X + PLATE_LX + GUARD_T - _GUARD_X0
GUARD_R = Box("guard_right", pos=(_GUARD_X0 + _GUARD_LEN / 2, PLATE_Y0 - GUARD_T / 2, (TOP + PLATE_Z + GUARD_H) / 2),
              size=(_GUARD_LEN, GUARD_T, PLATE_Z + GUARD_H - TOP), color=STEEL)
GUARD_L = Box("guard_left", pos=(_GUARD_X0 + _GUARD_LEN / 2, PLATE_Y0 + PLATE_W + GUARD_T / 2,
                                 (TOP + PLATE_Z + GUARD_H) / 2),
              size=(_GUARD_LEN, GUARD_T, PLATE_Z + GUARD_H - TOP), color=STEEL)
GUARD_B = Box("guard_back", pos=(LIP_X + PLATE_LX + GUARD_T / 2, PLATE_YC, (TOP + PLATE_Z + GUARD_H) / 2),
              size=(GUARD_T, PLATE_W, PLATE_Z + GUARD_H - TOP), color=STEEL)

# -- the debris: charred grease and carbon, stuck to the plate (each piece a rigid body: debriskit) -------------------
# Small flakes on a 1.2 cm grid over a 12 x 16 cm area (7-9.5 mm across, 2.0-3.0 mm thick, turned up to 25 degrees),
# and four crusts on each of two 5 x 5 cm spots where the patties sat (16-19 mm across, 3 mm thick). Sizes, turns and
# offsets come from a fixed seed, so the layout is the same every trial. Density 1.1 g/cm^3 (compact char).
SOIL_AREA = (0.160, -0.100, 0.120, 0.160)  # x, y, size x, size y
PATTY_SPOTS = ((0.190, -0.080, 0.050, 0.050), (0.210, 0.000, 0.050, 0.050))
CHAR_DENSITY = 1100.0  # kg/m^3
FLAKE_PITCH = 0.012
CHAR_COLORS = ((0.10, 0.08, 0.07), (0.16, 0.11, 0.08), (0.22, 0.15, 0.10))
CHIP_LIFT = 0.0002  # m: spawned this far above the plate (the bond holds it there; contact offset 2 mm)
# Each piece is stuck to the plate by a breakable bond at the middle of its underside (a PhysX fixed joint that the
# solver breaks for good the first time holding the piece takes more than its break force or torque). Break force =
# BOND_SHEAR x the piece's footprint: 0.15 N per cm^2, 0.08-0.13 N for a flake and 0.41-0.50 N for a crust. Break torque
# = break force x BOND_PEEL_ARM (a blade getting under an edge peels a piece off with less push than a flat shove).
BOND_SHEAR = 1500.0  # Pa
BOND_PEEL_ARM = 0.002  # m
# PhysX tests a joint's break force against its own internal measure of the joint's load, not the push on the piece:
# pushed with a slowly rising force in the simulator, a piece came free at about 1/32 of the value written (flakes
# 1/36, crusts 1/26-1/29). So the value written is the bond x BREAK_CAL, and the pieces come free at about their bond.
BREAK_CAL = 32.0


def _corners2(cx, cy, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in
            ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


def overlap2(a, b, margin=0.0) -> bool:
    """Two chips' footprints (oriented rectangles) overlap, or come closer than `margin` (separating-axis test)."""
    pa, pb = _corners2(a.pos[0], a.pos[1], a.size[0], a.size[1], a.yaw), _corners2(b.pos[0], b.pos[1], b.size[0],
                                                                                   b.size[1], b.yaw)
    for poly in (pa, pb):
        for i in range(4):
            (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % 4]
            nx, ny = y0 - y1, x1 - x0
            n = math.hypot(nx, ny)
            nx, ny = nx / n, ny / n
            da = [nx * x + ny * y for x, y in pa]
            db = [nx * x + ny * y for x, y in pb]
            if max(da) + margin <= min(db) or max(db) + margin <= min(da):
                return False
    return True


def _layout():
    rng = np.random.default_rng(20261114)
    chips = []

    def add(name, x, y, sx, sy, t, yaw):
        col = CHAR_COLORS[int(rng.integers(len(CHAR_COLORS)))]
        f = float(BOND_SHEAR * sx * sy)
        chips.append(dk.Chip(name, (float(x), float(y), float(PLATE_Z + t / 2 + CHIP_LIFT)), (float(sx), float(sy),
                             float(t)), float(yaw), float(CHAR_DENSITY * sx * sy * t), col,
                             (f, f * BOND_PEEL_ARM)))

    for k, (x, y, sx, sy) in enumerate(PATTY_SPOTS):  # four crusts per spot, 2 x 2
        for i in (-1, 1):
            for j in (-1, 1):
                s1, s2 = rng.uniform(0.016, 0.019, 2)
                add(f"crust_{k}_{(i + 1) // 2}{(j + 1) // 2}", x + sx / 2 + i * 0.0125 + rng.uniform(-0.001, 0.001),
                    y + sy / 2 + j * 0.0125 + rng.uniform(-0.001, 0.001), s1, s2, 0.003, rng.uniform(-0.15, 0.15))
    x0, y0, lx, ly = SOIL_AREA
    nx, ny = int(lx / FLAKE_PITCH), int(ly / FLAKE_PITCH)
    for i in range(nx):
        for j in range(ny):
            x = x0 + (lx - (nx - 1) * FLAKE_PITCH) / 2 + i * FLAKE_PITCH
            y = y0 + (ly - (ny - 1) * FLAKE_PITCH) / 2 + j * FLAKE_PITCH
            if any(px - 0.006 < x < px + sx + 0.006 and py - 0.006 < y < py + sy + 0.006
                   for px, py, sx, sy in PATTY_SPOTS):
                continue
            for _ in range(50):  # draw until it clears its neighbors by 1 mm
                s1, s2 = rng.uniform(0.007, 0.0095, 2)
                c = dk.Chip("probe", (x + rng.uniform(-0.0015, 0.0015), y + rng.uniform(-0.0015, 0.0015), 0.0),
                            (s1, s2, 0.0), float(rng.uniform(-0.44, 0.44)), 0.0, (0, 0, 0))
                if not any(overlap2(c, o, 0.001) for o in chips):
                    break
            else:
                continue
            add(f"flake_{i}_{j}", c.pos[0], c.pos[1], s1, s2, rng.uniform(0.002, 0.003), c.yaw)
    return tuple(chips)


CHIPS = _layout()
CHIP_MASS = np.array([c.mass for c in CHIPS])
TOTAL_MASS = float(CHIP_MASS.sum())
CHIP_BOND = np.array([c.bond[0] for c in CHIPS])  # break force per piece, N

# -- the scraper (scraper frame: origin where the blade's neck meets the handle, +x toward the handle's end, y along the
# edge, z square to the blade). Dexter-Russell Sani-Safe S294 class: 4 in. (10.2 cm) stiff stainless blade, 8-1/2 in.
# (21.6 cm) overall, polypropylene handle, 150 g (0.33 lb).
BLADE_W = 0.102
BLADE_L = 0.060  # the blade's flat, edge to neck
NECK_L = 0.035
BLADE_T = 0.0015
EDGE_LOCAL = np.array([-(BLADE_L + NECK_L), 0.0, -BLADE_T / 2])  # the middle of the edge, on the blade's underside
# (the face toward the plate when the edge is down: the line that meets the plate, 0.6 mm below the mid-thickness at 36
# degrees, so the 0.6 mm contact band is measured from the steel that touches)
HANDLE = ((0.065, 0.0, 0.0), (0.120, 0.030, 0.024))  # center, size: 12 cm long, 30 x 24 mm
BLADE_BOX = ((-(NECK_L + BLADE_L / 2), 0.0, 0.0), (BLADE_L, BLADE_W, BLADE_T))
NECK_BOX = ((-NECK_L / 2, 0.0, 0.0), (NECK_L, 0.030, BLADE_T))
SCRAPER_MASS = 0.150
GRIP_LOCAL = np.array([0.050, 0.0, 0.0])  # where the demo holds the handle: at its balance point, 14.5 cm behind the edge
# At rest the scraper lies on the bench on its edge and the handle's far end: the edge dips by REST_TILT.
REST_TILT = math.atan2(HANDLE[1][2] / 2 - BLADE_T / 2, HANDLE[0][0] + HANDLE[1][0] / 2 - EDGE_LOCAL[0])
SCRAPER_XY = (0.250, -0.255)
R_REST = np.array([[math.cos(-REST_TILT), 0.0, math.sin(-REST_TILT)], [0.0, 1.0, 0.0],
                   [-math.sin(-REST_TILT), 0.0, math.cos(-REST_TILT)]])  # Ry(-tilt): the edge end down
_rest_low = R_REST @ np.array([HANDLE[0][0] + HANDLE[1][0] / 2, 0.0, -HANDLE[1][2] / 2])
SCRAPER_START = (SCRAPER_XY[0], SCRAPER_XY[1], TOP - float(_rest_low[2]) + 0.0005)
WHITE = kit.Look((0.94, 0.94, 0.92), 0.5)
BLADE_LOOK = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=0.9)
SCRAPER = kit.Prop(
    "scraper", pos=SCRAPER_START, mass=SCRAPER_MASS, size=(0.216, BLADE_W, HANDLE[1][2]),
    tilt_xyzw=tuple(float(v) for v in sk.quat(R_REST)),
    meshes=(kit.box(*HANDLE, WHITE), kit.box(*BLADE_BOX, BLADE_LOOK), kit.box(*NECK_BOX, BLADE_LOOK)),
    colliders=(kit.Collider(*HANDLE), kit.Collider(*BLADE_BOX), kit.Collider(*NECK_BOX)))
HANDLE_PARTS = (gk.Block(*HANDLE),)  # what a hand holds


# -- the gouge check (scrapekit's blade-angle rule, no soil) and thresholds (each with its reason in the README) --------
# contact 0.6 mm (PhysX rest jitter; 1.0 mm once a stroke is on); slide 1.5 mm in 0.15 s; gouge > 70 deg or edge tilted
# > 10 deg over 1 cm
LAW = sk.Law(gap=0.0006, gap_hyst=0.0004)
BLADE = sk.Blade(center=tuple(EDGE_LOCAL), along=(0, 1, 0), normal=(0, 0, 1), width=BLADE_W, lanes=10)
GRID = sk.Plate(x0=LIP_X, y0=PLATE_Y0, lx=0.57, ly=0.31, z=PLATE_Z, cell=0.01, trough_x0=TRAY_X0,
                trough_y=(PLATE_Y0, PLATE_Y0 + PLATE_W))
TIER_TROUGH = (0.25, 0.60, 0.90)  # share of the debris (by mass) in the tray for 0.25, 0.5, 0.75
RESIDUE_MAX = 0.05  # 0.75 also needs at most 5 % of the debris left on the griddle
FULL_TROUGH = 0.98  # 1.0 needs at least 98 % in the tray ...
FULL_RESIDUE = 0.01  # ... and at most 1 % left on the griddle (the plate clean), plus the scraper set down

# The plate's surface: seasoned steel with a film of grease on it, slippery to a steel blade and to carbon (greased steel
# on steel, about 0.15). The simulator's default contact friction (1.2) would let the edge stick and chatter.
PLATE_FRICTION = (0.20, 0.15)  # static, dynamic
_PLATE_SIZE = (PLATE_LX, PLATE_W, PLATE_Z - TOP)
PLATE = kit.Prop("plate", pos=(LIP_X + PLATE_LX / 2, PLATE_YC, (TOP + PLATE_Z) / 2), size=_PLATE_SIZE,
                 meshes=(kit.box((0.0, 0.0, 0.0), _PLATE_SIZE, kit.Look(STEEL, 0.4, metallic=0.8)),),
                 colliders=(kit.Collider((0.0, 0.0, 0.0), _PLATE_SIZE),),
                 friction=PLATE_FRICTION)  # plate on its frame, one fixed block

# Carbon on steel: 0.5 static, 0.4 sliding; on the greased plate the plate's lower friction applies (combine "min").
# Every piece is bonded to the plate (the anchor) until the blade breaks it free.
dk.setup(CHIPS, dk.Physics(static_friction=0.5, dynamic_friction=0.4, break_scale=BREAK_CAL),
         anchor=(PLATE.name, PLATE.pos))

OBJECTS = [PLATE, TRAY, TRAY_WALL, GUARD_R, GUARD_L, GUARD_B, SCRAPER]

SPILL_CAP_SHARE = 0.02  # more than 2 % of the debris off the griddle caps the score at 0.25
SPILL_CAP = 0.25
GOUGE_CAP = 0.5
SET_DOWN_Z = TOP + 0.030  # the scraper's origin below this: lying on the bench, not on the griddle (plate at 5.0 cm)
STILL_STEP = 0.0005  # m per control step (1 cm/s) ...
STILL_TURN = math.radians(0.5)  # ... and under 0.5 degree a step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
GRIPPERS = ("left_gripper", "right_gripper")


def _pose(o):
    return np.asarray(o.pos, float), np.asarray(o.quat_xyzw, float)


def in_footprint(x, y) -> np.ndarray:
    """Over the griddle (plate, tray and splash guards), in plan."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    return ((x >= _GUARD_X0) & (x <= LIP_X + PLATE_LX + GUARD_T)
            & (y >= PLATE_Y0 - GUARD_T) & (y <= PLATE_Y0 + PLATE_W + GUARD_T))


def off_griddle(p) -> bool:
    """The scraper's origin outside the griddle's footprint (plate, tray and guards): on the bench, not in the grease
    tray, whose floor (1.8 cm up) is below SET_DOWN_Z."""
    return not bool(in_footprint(p[0], p[1]))


def where(debris) -> tuple:
    """Each piece's place from its real pose (center): in the tray (inside the tray's walls, below the plate's top),
    off the griddle (outside its footprint: on the bench, the floor, or carried off on the scraper), or else still on
    the griddle (on the plate, against a guard, on the blade above the plate, or falling)."""
    d = np.asarray(debris, float).reshape(-1, 7)
    x, y, z = d[:, 0], d[:, 1], d[:, 2]
    tray = (x >= TRAY_X0) & (x <= LIP_X) & (y >= PLATE_Y0) & (y <= PLATE_Y0 + PLATE_W) & (z < PLATE_Z)
    off = ~in_footprint(x, y)
    return tray, ~tray & ~off, off


def shares(debris) -> tuple:
    """Shares of the debris by mass: in the tray, left on the griddle, off it."""
    tray, left, off = where(debris)
    return (float(CHIP_MASS[tray].sum()) / TOTAL_MASS, float(CHIP_MASS[left].sum()) / TOTAL_MASS,
            float(CHIP_MASS[off].sum()) / TOTAL_MASS)


def scraper_held(objects) -> bool:
    """A gripper's own shape (palm and fingers, opened as they are) touches the scraper's handle (guardkit's contact
    check, within 2 mm): contact, not nearness."""
    return bool(gk.touching(objects, "scraper", HANDLE_PARTS, grippers=GRIPPERS))


def scraper_touched(objects) -> bool:
    """Any gripper touching any part of the scraper (blade included): it is not released."""
    parts = (gk.Block(*HANDLE), gk.Block(*BLADE_BOX), gk.Block(*NECK_BOX))
    return bool(gk.touching(objects, "scraper", parts, grippers=GRIPPERS))


START_DEBRIS = np.array([[*c.pos, 0.0, 0.0, math.sin(c.yaw / 2), math.cos(c.yaw / 2)] for c in CHIPS])


class Ledger:
    """One trial's history, advanced once per control step (success() and score() share it): the debris as last read,
    the gouge latch (scrapekit's blade-angle rule on the scraper's pose; no soil), and the set-down hold."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.s = sk.Scrape(GRID, BLADE, LAW, ())
        self.debris = START_DEBRIS.copy()
        self.last = None
        self.good = 0  # consecutive steps with the plate clean (the full shares) and the scraper set down and released

    def shares(self):
        return shares(self.debris)

    def tier(self) -> float:
        """The ladder by debris in the tray (and left on the griddle for 0.75), then the caps."""
        tr, res, sp = self.shares()
        t = 0.0
        if tr >= TIER_TROUGH[0] - 1e-12:
            t = 0.25
        if tr >= TIER_TROUGH[1] - 1e-12:
            t = 0.5
        if tr >= TIER_TROUGH[2] - 1e-12 and res <= RESIDUE_MAX + 1e-12:
            t = 0.75
        if self.s.gouged:
            t = min(t, GOUGE_CAP)
        if sp > SPILL_CAP_SHARE + 1e-12:
            t = min(t, SPILL_CAP)
        return t

    def clean(self) -> bool:
        """The full state: at least 98 % in the tray, at most 1 % left on the griddle, and no cap."""
        tr, res, sp = self.shares()
        return (tr >= FULL_TROUGH - 1e-12 and res <= FULL_RESIDUE + 1e-12 and not self.s.gouged
                and sp <= SPILL_CAP_SHARE + 1e-12)


_L = Ledger()


def _update(objects):
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    if dk.KEY in objects:
        _L.debris = np.asarray(objects[dk.KEY], float).reshape(-1, 7)
    held = scraper_held(objects)
    sp, sq = _pose(objects["scraper"])
    _L.s.step(sp, sq, held)
    r = sk.rotm(sq)
    still = False
    if _L.last is not None:
        dp = float(np.linalg.norm(sp - _L.last[0]))
        dr = math.acos(max(-1.0, min(1.0, (float(np.trace(_L.last[1].T @ r)) - 1) / 2)))
        still = dp <= STILL_STEP and dr <= STILL_TURN
    _L.last = (sp, r)
    down = (not scraper_touched(objects) and still and float(sp[2]) <= SET_DOWN_Z and off_griddle(sp)
            and not _L.s.touching)
    _L.good = _L.good + 1 if (_L.clean() and down) else 0


def success(objects):
    """At least 98 % of the debris (by mass) in the tray, at most 1 % left on the griddle, nothing gouged, at most 2 %
    off the griddle, and the scraper set down on the bench, released and still: this step counts toward the 1 s
    hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """By mass of the debris, from where the pieces really are: 0.25 with at least 25 % in the grease tray; 0.5 with at
    least 60 %; 0.75 with at least 90 % in the tray and at most 5 % left on the griddle; 1.0 with at least 98 % in the
    tray, at most 1 % left on the griddle and the scraper set down on the bench (off the griddle), released and still
    for 1 s. Caps: a gouge (blade over 70 degrees, or the edge tilted over 10 degrees onto a corner, slid 1 cm) caps
    at 0.5; more than 2 % of the debris off the griddle caps at 0.25. Doing nothing scores 0."""
    _update(objects)
    s = _L.tier()
    if _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
# The right hand takes the scraper by the handle from above (yaw 0: fingers close along world y, across the handle,
# which runs along x), lifts it and tilts it 40 degrees edge-down (Tilted pitch below zero: the approach leans forward),
# so the edge points at the robot and the handle rises away from it. Each stroke sets the edge on the plate beyond
# the debris, draws it toward the robot at 4 cm/s over the lip, lifts it slowly (5 cm/s) over the tray and moves
# back. Three lanes 5 cm apart (each stroke overlaps the last by half the blade), three times over, so the plate ends
# clean (at most 1 % left), then one short stroke along the left end of the lip. The scraper crosses the splash guard only while level and high (tilted, its edge would hang
# below the guard's top), so it is tilted over the plate and levelled over the tray. Then it goes back to where it lay, level, and is let go.
SCRAPE_DEG = 40.0  # blade-to-plate angle of the demo's strokes (inside the 20-60 degree band)
PRESS = 0.010  # m: the edge is aimed this far into the plate (the collider stops it). Holding the scraper, the hand
# sits 3.5-6.5 mm above its command along the stroke (more toward the robot; measured in the simulator), so a shallower
# aim lets the edge rise off the plate halfway down the stroke and the debris slip under it.
LANES_Y = (-0.070, -0.020, 0.030)  # stroke centers: the blade covers y -0.121 to 0.081
PASSES = 3
# A last short stroke along the left end of the lip: pushed flakes spread sideways, and one can come to rest just beyond
# the left lane's end (y about 10 cm, by the lip). Centered at y 7.5, the blade covers y 2.4 to 12.6, 9.5 mm inside the
# left splash guard (13.55).
FINAL_LANE_Y = 0.075
FINAL_X_START = 0.220  # it starts 7 cm before the lip: the plate behind it is already clean
X_START, X_END = 0.325, 0.135  # edge from 4.5 cm beyond the debris to 1.5 cm past the lip. The hand lags about 1.5 cm,
# so the edge lands about 3 cm beyond the debris: an edge that lands on a bonded piece presses it down and slides over
# it, and pieces knocked back when a bond lets go can come to rest a few cm beyond the debris
STROKE_SPEED = 0.04  # m/s
LIFT = 0.03  # m above the plate between strokes
LIFT_S = 0.6  # s for the lift at the end of a stroke (0.05 m/s: what rides on the blade slides off into the tray)
CARRY_Z = TOP + 0.15  # tool point height when carrying the scraper level (it clears the 5 cm splash guards)
GRIP_YAW = 0.0  # yaw 0: the fingers close along world y, across the handle (which lies along x)
# The handle turns about 6 degrees in the fingers as they close and lift it (measured in the simulator: the edge end swings
# toward -y); tilted 36-40 degrees, that turn would roll the edge 3.5 degrees, one end 6 mm above the plate. The tilted
# hand is turned back by the same amount, so the edge meets the plate along its length.
YAW_TRIM = math.radians(-6.0)
GRIP_DZ = 0.004  # m: the tool point 4 mm above the handle's axis, so the pad tips (13 mm past the tool point) stop
# 1 mm above the handle's underside and clear the bench
GRASP_LOCAL = GRIP_LOCAL + R_REST.T @ np.array([0.0, 0.0, GRIP_DZ])  # the tool point in the scraper's frame


def scraper_rot(pitch: float) -> np.ndarray:
    """The scraper's rotation once the hand that took it at rest pitches by `pitch` about world y (negative: the
    approach leans forward and the edge, pointing at the robot, goes down)."""
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]) @ R_REST


def pitch_for(theta_deg: float) -> float:
    """Hand pitch that puts the blade at theta_deg to the plate (the rest tilt is already in the grasp)."""
    return REST_TILT - math.radians(theta_deg)


def blade_angle(pitch: float) -> float:
    return sk.blade_angle_deg(scraper_rot(pitch), BLADE)


def tcp_for_edge(edge, pitch: float) -> tuple:
    """The tool point that puts the middle of the scraper's edge at `edge`."""
    r = scraper_rot(pitch)
    return tuple(float(v) for v in np.asarray(edge, float) - r @ (EDGE_LOCAL - GRASP_LOCAL))


def _stroke(plan, y, label, x0=X_START):
    p = pitch_for(SCRAPE_DEG)
    E = lambda x, dz: tcp_for_edge((x, y, PLATE_Z + dz), p)  # noqa: E731
    run = (x0 - X_END) / STROKE_SPEED
    plan += [kit.Tilted(1.5, E(x0, LIFT), FINGER_CLOSED, f"over the start: {label}", GRIP_YAW + YAW_TRIM, pitch=p),
             kit.Tilted(0.8, E(x0, -PRESS), FINGER_CLOSED, f"edge on: {label}", GRIP_YAW + YAW_TRIM, pitch=p),
             kit.Tilted(run, E(X_END, -PRESS), FINGER_CLOSED, label, GRIP_YAW + YAW_TRIM, pitch=p),
             kit.Tilted(LIFT_S, E(X_END, LIFT), FINGER_CLOSED, f"lift over the tray: {label}", GRIP_YAW + YAW_TRIM, pitch=p)]


def demo():
    """Right hand: take the scraper, scrape three overlapping lanes three times over into the tray, lay the scraper back down."""
    g0 = np.asarray(SCRAPER_START) + R_REST @ GRIP_LOCAL
    gx, gy, gz = (float(v) for v in g0)
    gz += GRIP_DZ
    p = pitch_for(SCRAPE_DEG)
    s0 = tcp_for_edge((X_START, LANES_Y[0], PLATE_Z + LIFT), p)
    plan = [Waypoint(2.0, (gx, gy, gz + 0.08), FINGER_OPEN, "above the handle", GRIP_YAW),
            Waypoint(1.5, (gx, gy, gz), FINGER_OPEN, "around the handle", GRIP_YAW),
            Waypoint(0.8, (gx, gy, gz), FINGER_CLOSED, "grip", GRIP_YAW),
            Waypoint(1.2, (gx, gy, CARRY_Z), FINGER_CLOSED, "lift", GRIP_YAW),
            Waypoint(1.5, (0.300, LANES_Y[0] - 0.030, CARRY_Z), FINGER_CLOSED, "level over the plate", GRIP_YAW),
            kit.Tilted(2.0, s0, FINGER_CLOSED, "tilt the edge down over the plate", GRIP_YAW + YAW_TRIM, pitch=p)]
    for k in range(PASSES):
        for i, y in enumerate(LANES_Y):
            _stroke(plan, y, f"pass {k + 1} lane {i + 1}")
    _stroke(plan, FINAL_LANE_Y, "left end of the lip", x0=FINAL_X_START)
    e = tcp_for_edge((X_END, LANES_Y[-1], PLATE_Z + LIFT), p)  # levelled where the arm reaches CARRY_Z easily
    plan += [Waypoint(1.5, (e[0], e[1], CARRY_Z), FINGER_CLOSED, "level it over the tray", GRIP_YAW),
             Waypoint(2.0, (gx, gy, CARRY_Z), FINGER_CLOSED, "back over the bench", GRIP_YAW),
             Waypoint(1.5, (gx, gy, gz + 0.002), FINGER_CLOSED, "lay it down", GRIP_YAW),
             Waypoint(0.8, (gx, gy, gz + 0.002), FINGER_OPEN, "let go", GRIP_YAW),
             Waypoint(1.2, (gx, gy, gz + 0.08), FINGER_OPEN, "lift clear", GRIP_YAW),
             Waypoint(2.0, (gx, gy, gz + 0.08), FINGER_OPEN, "wait", GRIP_YAW)]
    return {"right": plan}
