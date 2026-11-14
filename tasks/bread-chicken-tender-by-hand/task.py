"""On the station table, three eighth-size aluminum sheet pans stand side by side: on the left a pan holding one raw
chicken tender, wet from its marinade; in the middle a pan of seasoned flour 1 cm deep (the breading); on the right a
pan with a wire rack in it, where breaded tenders wait for the fryer.

The job is a fry cook's hand-breading: press the breading onto every face of the tender (lay it in the flour and
press it, turn it over and press it again), handle it as little as possible once it is coated, because every grip
on a breaded piece wipes the coat off under the fingers, and leave it on the rack.

The tender is loose, of its real mass, and soft: three rigid parts (together an 11.5 x 3.5 x 1.4 cm box) on two
sprung hinges across its width, so it droops a little when held by one end. It is drawn as a tenderloin (13.5 cm tip
to tip, 3.7 cm wide, 1.8 cm thick, rounded and tapered) over those parts. The flour has no collider:
the tender lies on the breading pan's floor with its lower 1 cm in the flour. The coat is coatkit's model on the box's
surface (2.5 mm cells, drawn as crumbs on the tender's shape): breading sticks only where it is pressed on, read from PhysX contact forces (the pan pushing up at least
2.0 N while a gripper pushes down, for 25 ms), and a grip (both fingers squeezing from opposite sides, 0.2 N each)
wipes it off under the finger pads. Every threshold below appears once, with the same value in task.json and the
README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402
import coatkit as ck  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Bread the chicken tender and put it on the rack for the fryer."

# -- the pans: three eighth-size aluminum sheet pans, 24.4 x 16.8 x 2.5 cm (9.6 x 6.6 x 1 in.), long side along x ---
PAN_L, PAN_W, PAN_H = 0.244, 0.168, 0.025
SHEET = 0.001  # the floor's thickness: its top is at TOP + 1 mm
WALL = 0.002  # wall colliders (the real rolled rim is about 5 mm across; the sheet is 1 mm)
PAN_X = 0.280  # all three centered at x 28.0 cm
Y_MARINADE, Y_BREADING, Y_RACK = 0.195, 0.0, -0.195  # left, middle, right
FLOOR = TOP + SHEET
INNER = (PAN_L - 2 * WALL, PAN_W - 2 * WALL)  # 24.0 x 16.4 cm inside

# -- the breading: seasoned flour (all-purpose flour with 2 % paprika by weight), 1.0 cm deep, about 400 mL --------
FLOUR_DEPTH = 0.010
FLOUR_RGB = (0.95, 0.90, 0.80)
CRUMB_RGB = ((0.87, 0.75, 0.55), (0.76, 0.45, 0.28))  # toasted crumbs and paprika flecks on the flour's surface

# -- the rack: an eighth-size wire cooling rack, 22.0 x 14.5 cm, its grid top 1.6 cm above the table ---------------
RACK_SIZE = (0.220, 0.145)
GRATE_TOP = TOP + 0.016
WIRE = 0.003  # 3 mm wires
WIRE_PITCH = 0.012  # cross wires every 1.2 cm

# -- the chicken tender (body frame: x along its length, z through its thickness, origin at its middle) -------------
TENDER_SIZE = (0.115, 0.035, 0.014)  # its body: an 11.5 cm long, 3.5 cm wide, 1.4 cm thick box (collider and coat cells)
TENDER_MASS = 0.050  # kg: a raw breast tenderloin (weigh the real one; tenderloins run about 40 to 60 g)
TENDER_START = (PAN_X, Y_MARINADE, FLOOR + TENDER_SIZE[2] / 2)  # lying flat in the marinade pan, long side along x
RAW_RGB = (0.93, 0.68, 0.64)
# What the cameras see: a tenderloin 13.5 cm tip to tip, 3.7 cm across at its widest and 1.8 cm thick along its middle,
# rounded in section, blunt at the near end and tapering to a narrower tail at the far end. Its body is the box above: the drawn shape
# covers the box's sides where the fingers hold it, and its rounded middle lies about 2 mm into a surface it rests on,
# as soft meat flattens on a pan.
TENDER_LONG, TENDER_WIDE, TENDER_THICK = 0.135, 0.037, 0.018
SECTION_POWER = 3.5  # the cross-section is a superellipse: flattish faces, rounded edges
# Soft: three 3.8 cm parts on two hinges across its width, 1.9 cm either side of its middle, so it bends through its
# thickness as raw meat does. Each hinge's stiffness: held level by one end part, the rest droops about 14 degrees at
# the first hinge and 18 at the second (the 33 g beyond the first hinge, 3.8 cm out, against 0.05 N*m/rad).
CUT = TENDER_SIZE[0] / 6  # m: the hinges' x
BEND_K = 0.05  # N*m/rad per hinge, toward straight
BEND_C = 0.003  # N*m*s/rad per hinge (about critical for the parts beyond it)
BEND_LIMIT = math.radians(60)  # each hinge bends at most this far either way
PART_MASS = TENDER_MASS / 3
PART_LEN = TENDER_SIZE[0] / 2 - CUT  # 3.83 cm: each end part's length (the middle one is 2 x CUT, the same)
PARTS = (("near", (-CUT, 0.0, 0.0)), ("base", (0.0, 0.0, 0.0)), ("far", (CUT, 0.0, 0.0)))  # bodies, lowest x first
PART_KEYS = ("tender/near", "tender", "tender/far")  # their keys in `objects`
# each part's box in its own frame (center, size): the near and far parts' frames sit at their hinges
PART_BOXES = (((-PART_LEN / 2, 0.0, 0.0), (PART_LEN, TENDER_SIZE[1], TENDER_SIZE[2])),
              ((0.0, 0.0, 0.0), (2 * CUT, TENDER_SIZE[1], TENDER_SIZE[2])),
              ((PART_LEN / 2, 0.0, 0.0), (PART_LEN, TENDER_SIZE[1], TENDER_SIZE[2])))

# -- the coat model (coatkit) and the thresholds (each with its reason in the README and task.json) ----------------
CELL = 0.0025  # m: 2.5 mm coat cells, 2,008 on the tender
PRESS_N = 2.0  # N: the pan's upward push on the tender while a gripper pushes it down (a resting tender gives 0.49)
PRESS_STEPS = 6  # physics steps (25 ms at 240 Hz) in a row: a press, not the impact of a drop
PRESS_HALF = 0.030  # m: a press sets the breading within 3 cm either side of where it is pressed, along the length
WIPE_N = 0.2  # N on each finger, from opposite sides: a grip, which wipes the coat under the pads. Carrying the
# 0.49 N tender by friction takes about 0.25 N per finger at friction 1.0, so every grip that can lift it wipes.
LAW = ck.Law(press_n=PRESS_N, press_steps=PRESS_STEPS, press_half=PRESS_HALF, wipe_n=WIPE_N)
SLAB = ck.Slab(TENDER_SIZE, CELL)
BED = ck.Bed(center=(PAN_X, Y_BREADING), size=INNER, floor_z=FLOOR, depth=FLOUR_DEPTH)
ck.setup("tender", SLAB, BED, LAW, pan="breading_pan", parts=PARTS, cuts=(-CUT, CUT))

TIERS = ((10e-4, 1.0), (25e-4, 0.75), (50e-4, 0.5), (85e-4, 0.25))  # bare area at most (m^2) -> score
# of the tender's 122.5 cm^2: at most 10 cm^2 bare (91.8 % coated) 1.0 (also needs release), 25 cm^2 (79.6 %) 0.75,
# 50 cm^2 (59.2 %) 0.5, 85 cm^2 (30.6 %) 0.25; more bare than that: 0
HELD_CAP = 0.5  # a hand still on the tender at the end caps the score at 0.5
ON_RACK_GAP = 0.010  # m: the tender's lowest corner within 1 cm above the grate (resting on it, not held over it)
TABLE_TOUCH = 0.003  # m: the tender's lowest corner this close to the table top, off every pan: dropped
STILL_SPEED = 0.01  # m/s: at rest ...
STILL_SPIN = math.radians(10.0)  # ... and turning under 10 degrees/s
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s on the rack and still
GRIPPERS = ("left_gripper", "right_gripper")
TENDER_PARTS = tuple((key, (gk.Block(c, size),)) for key, (c, size) in zip(PART_KEYS, PART_BOXES))

# -- the scene ---------------------------------------------------------------------------------------------------------
ALU = kit.Look((0.80, 0.81, 0.82), 0.35, metallic=0.8)
STEEL = kit.Look((0.74, 0.75, 0.77), 0.3, metallic=0.9)
RAW = kit.Look(RAW_RGB, 0.25)  # wet raw chicken: glossy pink
FLOUR = kit.Look(FLOUR_RGB, 0.95)
CRUMBS = tuple(kit.Look(c, 0.95) for c in CRUMB_RGB)


def tender_half_width(x):
    """Half the drawn tender's width at x along it (m): a broad, rounded near end (-x) and a longer taper to a narrower
    far end (+x), as a tenderloin narrows from its thick head to its tail."""
    x = np.asarray(x, float)
    s = np.clip(np.abs(x) / (TENDER_LONG / 2), 0.0, 1.0)
    return TENDER_WIDE / 2 * np.where(x < 0, np.sqrt(1 - s ** 4), (1 - s ** 3.5) ** 0.7)


def tender_half_thick(x):
    """Half the drawn tender's thickness at x along it (m): thickest along its middle, thinner toward the tips."""
    x = np.asarray(x, float)
    s = np.clip(np.abs(x) / (TENDER_LONG / 2), 0.0, 1.0)
    return TENDER_THICK / 2 * np.where(x < 0, np.sqrt(1 - s ** 4), np.sqrt(1 - s ** 3))


def _inside(p):
    """Below 0 inside the drawn tender, above 0 outside (p: (n, 3) in its frame)."""
    p = np.atleast_2d(p)
    a = np.maximum(tender_half_width(p[:, 0]), 1e-6)
    b = np.maximum(tender_half_thick(p[:, 0]), 1e-6)
    f = (np.abs(p[:, 1]) / a) ** SECTION_POWER + (np.abs(p[:, 2]) / b) ** SECTION_POWER - 1.0
    return np.where(np.abs(p[:, 0]) >= TENDER_LONG / 2, 1.0, f)


def tender_surface(points, normals):
    """Where the coat's cells (on the body's box faces) sit on the drawn tender: each cell is moved along a ray from
    the tender's middle line through it onto the drawn surface; returns the surface points and outward normals."""
    p = np.asarray(points, float)
    core = TENDER_SIZE[0] / 2 - 0.012
    a = np.stack([np.clip(p[:, 0], -core, core), np.zeros(len(p)), np.zeros(len(p))], axis=1)
    d = p - a
    lo, hi = np.zeros(len(p)), np.full(len(p), 4.0)
    for _ in range(40):  # bisection along each ray for the surface
        mid = (lo + hi) / 2
        out = _inside(a + d * mid[:, None]) > 0
        hi, lo = np.where(out, mid, hi), np.where(out, lo, mid)
    s = a + d * lo[:, None]
    e = 1e-5
    g = np.stack([(_inside(s + e * np.eye(3)[k]) - _inside(s - e * np.eye(3)[k])) / (2 * e) for k in range(3)], axis=1)
    g /= np.linalg.norm(g, axis=1, keepdims=True)
    return s, g


def _tender_mesh(look, x0: float, x1: float, origin, rings: int = 30, around: int = 36):
    """The drawn tender between x0 and x1 (its own frame), as rings of its superellipse section, moved into a part's
    frame (`origin`: the part's frame origin in the tender's frame). An end that is not a tip is closed by a flat cap
    (the cut between two parts, hidden inside the neighbor's overlap)."""
    u = np.linspace(0.0, 1.0, rings)
    half = TENDER_LONG / 2
    lo, hi = max(x0, -half), min(x1, half)
    xs = lo + (hi - lo) * u  # evenly spaced, except that rings bunch up toward a tip
    if lo <= -half + 1e-9:
        xs = -half + (hi + half) * np.sin(u * np.pi / 2)
    if hi >= half - 1e-9:
        xs = half - (half - lo) * np.sin((1 - u) * np.pi / 2)
    t = 2 * np.pi * np.arange(around) / around
    cy = np.sign(np.cos(t)) * np.abs(np.cos(t)) ** (2 / SECTION_POWER)
    cz = np.sign(np.sin(t)) * np.abs(np.sin(t)) ** (2 / SECTION_POWER)
    o = np.asarray(origin, float)
    pts = [np.array([x, a * y, b * z]) - o
           for x, a, b in zip(xs, tender_half_width(xs), tender_half_thick(xs)) for y, z in zip(cy, cz)]
    counts, idx = [], []
    for i in range(rings - 1):
        for k in range(around):
            j = (k + 1) % around
            counts.append(4)
            idx += [i * around + k, i * around + j, (i + 1) * around + j, (i + 1) * around + k]
    if lo > -half + 1e-9:
        counts.append(around)
        idx += list(range(around))[::-1]
    if hi < half - 1e-9:
        counts.append(around)
        idx += [(rings - 1) * around + k for k in range(around)]
    return kit._mesh(pts, counts, idx, look)


def _crumbs(rng, n: int, look, z0: float):
    """`n` small crumbs (0.8 to 2 mm, turned at random) scattered over the flour's surface, as one mesh."""
    pts, counts, idx = [], [], []
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    for _ in range(n):
        c = np.array([rng.uniform(-1, 1) * (INNER[0] / 2 - 0.003), rng.uniform(-1, 1) * (INNER[1] / 2 - 0.003), z0])
        w, d, h = rng.uniform(0.0008, 0.002), rng.uniform(0.0008, 0.002), rng.uniform(0.0003, 0.0008)
        ang = rng.uniform(0, np.pi)
        R = np.array([[np.cos(ang), -np.sin(ang), 0.0], [np.sin(ang), np.cos(ang), 0.0], [0.0, 0.0, 1.0]])
        base = len(pts)
        pts += [c + R @ (np.array([sx * w, sy * d, (sz + 1) * h]) / 2) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
        counts += [4] * 6
        idx += [base + i for f in faces for i in f]
    return kit._mesh(pts, counts, idx, look)


def _pan_parts():
    """An eighth sheet pan's floor and four walls, in the pan's frame (origin on the table under its middle)."""
    return [((0.0, 0.0, SHEET / 2), (PAN_L, PAN_W, SHEET)),
            ((-(PAN_L - WALL) / 2, 0.0, PAN_H / 2), (WALL, PAN_W, PAN_H)),
            (((PAN_L - WALL) / 2, 0.0, PAN_H / 2), (WALL, PAN_W, PAN_H)),
            ((0.0, -(PAN_W - WALL) / 2, PAN_H / 2), (PAN_L, WALL, PAN_H)),
            ((0.0, (PAN_W - WALL) / 2, PAN_H / 2), (PAN_L, WALL, PAN_H))]


def _pan(name, y, extra_meshes=(), extra_colliders=()):
    parts = _pan_parts()
    return kit.Prop(name, pos=(PAN_X, y, TOP), mass=None, size=(PAN_L, PAN_W, PAN_H),
                    meshes=tuple(kit.box(c, s, ALU) for c, s in parts) + tuple(extra_meshes),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in parts) + tuple(extra_colliders))


_rz = GRATE_TOP - TOP - WIRE / 2  # the grid's mid-height in the pan's frame
_rack_meshes = [kit.box((0.0, s * (RACK_SIZE[1] - WIRE) / 2, _rz), (RACK_SIZE[0], WIRE, WIRE), STEEL) for s in (-1, 1)]
_rack_meshes += [kit.box((x, 0.0, _rz), (WIRE, RACK_SIZE[1], WIRE), STEEL)
                 for x in np.arange(-RACK_SIZE[0] / 2 + WIRE / 2, RACK_SIZE[0] / 2, WIRE_PITCH)]
_rack_meshes += [kit.box((sx * 0.09, sy * 0.06, (_rz - WIRE / 2 + SHEET) / 2), (WIRE, WIRE, _rz - WIRE / 2 - SHEET),
                         STEEL) for sx in (-1, 1) for sy in (-1, 1)]  # four feet
RACK_COLLIDER = kit.Collider((0.0, 0.0, _rz), size=(RACK_SIZE[0], RACK_SIZE[1], WIRE))  # the grid as one flat plate

MARINADE_PAN = _pan("marinade_pan", Y_MARINADE)
_RNG = np.random.default_rng(5)  # the crumbs' fixed scatter
BREADING_PAN = _pan("breading_pan", Y_BREADING, extra_meshes=(
    kit.box((0.0, 0.0, SHEET + FLOUR_DEPTH / 2), (INNER[0], INNER[1], FLOUR_DEPTH), FLOUR),  # seen, not collided
    _crumbs(_RNG, 2400, CRUMBS[0], SHEET + FLOUR_DEPTH - 0.0002), _crumbs(_RNG, 350, CRUMBS[1], SHEET + FLOUR_DEPTH - 0.0002)))
RACK_PAN = _pan("rack_pan", Y_RACK, extra_meshes=_rack_meshes, extra_colliders=(RACK_COLLIDER,))
class _Bend:
    """A hinge's own stiffness and damping toward straight, as PhysX's implicit joint drive (yamkit law)."""

    def drive(self, key, q, qd, joints, mem):
        return 0.0, BEND_K, 0.0, BEND_C


_LAP = 0.003  # m: each part's drawn piece reaches this far past its hinge, so a bent hinge shows no gap
_BEND = _Bend()
TENDER = kit.Mechanism(
    "tender", pos=TENDER_START, mass=PART_MASS, size=TENDER_SIZE,
    meshes=(_tender_mesh(RAW, -CUT - _LAP, CUT + _LAP, PARTS[1][1]),),
    colliders=(kit.Collider(PART_BOXES[1][0], size=PART_BOXES[1][1]),),
    links=tuple(kit.Link(name, "revolute", "Y", anchor=origin, lower=-BEND_LIMIT, upper=BEND_LIMIT, mass=PART_MASS,
                         meshes=(_tender_mesh(RAW, *span, origin),), colliders=(kit.Collider(box[0], size=box[1]),),
                         damping=BEND_C, law=_BEND)
                for (name, origin), box, span in ((PARTS[0], PART_BOXES[0], (-1.0, -CUT + _LAP)),
                                                  (PARTS[2], PART_BOXES[2], (CUT - _LAP, 1.0)))))
OBJECTS = [MARINADE_PAN, BREADING_PAN, RACK_PAN, TENDER]

PANS = {"marinade_pan": Y_MARINADE, "breading_pan": Y_BREADING, "rack_pan": Y_RACK}
_UNIT = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) / 2


# -- checks -----------------------------------------------------------------------------------------------------------
def corners(objects) -> np.ndarray:
    """The corners of the tender's three parts (24 points, world)."""
    out = []
    for key, (c, size) in zip(PART_KEYS, PART_BOXES):
        o = objects[key]
        out.append((np.asarray(c) + _UNIT * np.asarray(size)) @ ck.rotm(o.quat_xyzw).T + np.asarray(o.pos, float))
    return np.concatenate(out)


def over_pan(xy, name: str) -> bool:
    return abs(xy[0] - PAN_X) <= PAN_L / 2 and abs(xy[1] - PANS[name]) <= PAN_W / 2


def dropped_now(objects) -> bool:
    """The tender touches the table (its lowest corner within 3 mm of the table top) away from every pan, or has
    fallen below it. Raw chicken that has touched the bench or the floor is thrown away."""
    c = corners(objects)
    p = np.asarray(objects["tender"].pos, float)
    if p[2] < TOP - 0.02:
        return True
    low = c[np.argmin(c[:, 2])]
    return float(low[2]) <= TOP + TABLE_TOUCH and not any(over_pan(low[:2], n) for n in PANS)


def on_rack(objects) -> bool:
    """Resting on the rack's grid: the tender's middle over the grid and its lowest corner within 1 cm above it."""
    p = np.asarray(objects["tender"].pos, float)
    low = float(corners(objects)[:, 2].min())
    inside = abs(p[0] - PAN_X) <= RACK_SIZE[0] / 2 and abs(p[1] - Y_RACK) <= RACK_SIZE[1] / 2
    return inside and GRATE_TOP - 0.003 <= low <= GRATE_TOP + ON_RACK_GAP


def motion(objects, last) -> tuple:
    """The fastest any of the tender's parts moves (m/s) and turns (rad/s): its reported velocity or, as the
    articulation's parts report none, how far it moved since the last control step's poses `last` (None at the
    first step: unknown, so not still)."""
    if last is None:
        return math.inf, math.inf
    v = w = 0.0
    for key, (p0, q0) in zip(PART_KEYS, last):
        o = objects[key]
        d = abs(float(np.dot(np.asarray(o.quat_xyzw, float), q0)))
        v = max(v, float(np.linalg.norm(np.asarray(o.pos, float) - p0)) / DT, float(np.linalg.norm(o.lin_vel)))
        w = max(w, 2 * math.acos(min(1.0, d)) / DT, float(np.linalg.norm(o.ang_vel)))
    return v, w


def touching(objects) -> tuple:
    """Grippers in contact with the tender (their palm or fingers within 2 mm of any of its parts): contact, not
    nearness."""
    hit = set()
    for key, parts in TENDER_PARTS:
        hit.update(gk.touching(objects, key, parts, grippers=GRIPPERS))
    return tuple(g for g in GRIPPERS if g in hit)


def coverage() -> float:
    """Coated area over the tender's whole surface (0..1), from coatkit."""
    return ck.state().coverage()


def bare_area() -> float:
    """Uncoated area of the tender's surface (m^2): what a real trial counts as bare 5 mm squares on its photos."""
    c = ck.state()
    return c.total_area() * (1.0 - c.coverage())


def tier(bare: float) -> float:
    for at, s in TIERS:
        if bare <= at + 1e-12:
            return s
    return 0.0


# -- one trial's record ------------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.drop_run = 0
        self.dropped = False  # latched
        self.good = 0  # consecutive control steps on the rack and still
        self.held = False
        self.last = None  # the parts' (pos, quat) at the last control step
        self.still = False


_L = Ledger()


def _update(objects):
    """Advance the trial's record once per control step (success() and score() share it)."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
        ck.reset()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.drop_run = _L.drop_run + 1 if dropped_now(objects) else 0
    if _L.drop_run >= 2:  # two control steps in a row: not a one-step physics pop
        _L.dropped = True
    _L.held = bool(touching(objects))
    v, w = motion(objects, _L.last)
    _L.last = [(np.asarray(objects[k].pos, float).copy(), np.asarray(objects[k].quat_xyzw, float).copy())
               for k in PART_KEYS]
    _L.still = v < STILL_SPEED and w < STILL_SPIN
    _L.good = _L.good + 1 if (on_rack(objects) and _L.still) else 0
    _TILES.draw(objects)


def success(objects):
    """Full coverage tier, on the rack, released and still: this step counts toward the 1 s hold."""
    _update(objects)
    return (not _L.dropped and _L.good >= 1 and not _L.held and tier(bare_area()) == 1.0)


def score(objects):
    """By the bare (uncoated) area of the tender's 122.5 cm^2 surface, once it rests on the rack, still for 1 s: 0.25
    at 85 cm^2 or less, 0.5 at 50, 0.75 at 25, 1.0 at 10 cm^2 or less with no hand on it. A hand still on the tender caps the score at 0.5. Off
    the rack at the end, or dropped on the table at any time (latched): 0. Doing nothing scores 0."""
    _update(objects)
    if _L.dropped or _L.good < HOLD_STEPS:
        return 0.0
    s = tier(bare_area())
    if _L.held:
        s = min(s, HELD_CAP)
    return s


# -- the coat the cameras see ------------------------------------------------------------------------------------------
class _Tiles:
    """coatkit.Tiles on the tender's parts, placed in the env's frame (the tender's prim, where it was spawned)."""

    def __init__(self):
        self.tiles, self.origin = ck.Tiles(surface=tender_surface), None

    def draw(self, objects):
        if self.tiles.off:
            return
        if self.origin is None:
            try:
                import omni.usd
                from pxr import UsdGeom

                stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
                self.origin = np.zeros(3)
                for prim in stage.Traverse():
                    if prim.GetName() == "tender":
                        w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                        self.origin = w - np.asarray(TENDER_START, float)
                        break
            except Exception:  # noqa: BLE001 (no simulator: unit tests)
                self.tiles.off = True
                return
        pos = np.array([objects[k].pos for k in PART_KEYS], float)
        q = np.array([objects[k].quat_xyzw for k in PART_KEYS], float)
        self.tiles.draw(ck.state(), pos, q, self.origin)


_TILES = _Tiles()

# -- scripted demo: open loop from the fixed layout ------------------------------------------------------------------
# Left hand: takes the tender out of the marinade (pads on its long sides, across its 3.5 cm width), lays it flat in
# the breading and presses it with a closed fist at three spots along it. Right hand: takes it near one end, lifts it
# and rolls it 90 degrees onto its edge; the left hand, rolled the other way, takes it by its far half and the right
# lets go; the left rolls back level, which turns the tender over, lays it back in the breading and presses three
# spots again. Right hand: takes it at the same near end as before (so the second grip wipes the same spots as the
# first), carries it to the rack and lets go.
ACROSS = 0.0  # grip yaw: fingers close along world y (the simulator's yaw 0), across a tender lying along x (right hand)
ACROSS_L = 0.0  # the same for the left hand (real IK: yaw 180 runs its joint 6 into -120 during the presses)
GRASP_Z = FLOOR + 0.015  # tool point for a grip on the tender lying on a floor: fingertips 2 mm above the floor
CENTER_TO_TCP = GRASP_Z - (FLOOR + TENDER_SIZE[2] / 2)  # 8 mm: the tool point above the tender's middle in a grip
PRESS_DEPTH = 0.008  # m: the fist is commanded this far below the tender's top (the arm's lag sets the force)
FIST_TIP = 0.0128  # m: fingertips past the tool point (guardkit's finger fit)
PRESS_Z = FLOOR + TENDER_SIZE[2] - PRESS_DEPTH + FIST_TIP
STATIONS = (-0.040, 0.0, 0.040)  # m along the tender from its middle: three presses cover its 11.5 cm
NEAR_GRIP = -0.0375  # m: the right hand's grip, on the end toward the robot (fingertips 1.5 to 5.1 cm from the end)
FAR_GRIP = 0.035  # m: the left hand's grip during the turn, on the far half
FLIP_Z = TOP + 0.150  # height of the turn (the roll needs the hand at least 9 to 11 cm up at this reach; at 12 cm the
# left hand, coming in rolled, sagged and caught its lower finger on the breading pan's wall in the simulator)
LIFT = TOP + 0.100
LEFT_PARK = (0.200, 0.200, TOP + 0.160)
RIGHT_PARK = (0.200, -0.200, TOP + 0.160)
HAND_Y = 0.013  # left tool point's y during the turn: its fingertips span the tender's 1.4 cm thickness
R_APPROACH = 0.6 * FINGER_OPEN  # the right hand comes down on the tender in the breading pan 60 % open: each finger
# 2.8 cm from the tool point (the tender is 1.75 cm each side) keeps the fingers' outsides 2.5 cm inside the pan's
# walls (fully open, the simulator stopped the hand on them 2.4 cm short of its grip)


class _Plan:
    """One arm's waypoints with a running clock, so the two arms can wait for each other."""

    def __init__(self, start, yaw=ACROSS):
        self.across = yaw
        self.wps, self.t, self.pos, self.finger, self.yaw, self.roll = [], 0.0, tuple(start), FINGER_OPEN, yaw, 0.0
        self.go(2.5, start, FINGER_OPEN, "park")

    def go(self, dur, pos, finger=None, label="", yaw=None, roll=None):
        self.pos = tuple(float(v) for v in pos)
        self.finger = self.finger if finger is None else finger
        self.yaw = self.yaw if yaw is None else yaw
        self.roll = self.roll if roll is None else roll
        self.wps.append(kit.Tilted(dur, self.pos, self.finger, label, self.yaw, self.roll, 0.0))
        self.t += dur
        return self

    def until(self, t, label="wait"):
        if t > self.t + 1e-9:
            self.go(t - self.t, self.pos, label=label)
        return self

    def presses(self, x0, y):
        """Close the hand and press the tender at the three stations with the fingertips."""
        self.go(0.8, (x0 + STATIONS[0], y, PRESS_Z + 0.03), FINGER_CLOSED, "close the hand over the tender", self.across)
        for s in STATIONS:
            self.go(0.8, (x0 + s, y, PRESS_Z + 0.03), label="over the next spot")
            self.go(1.0, (x0 + s, y, PRESS_Z), label="press it into the breading")
            self.go(1.0, (x0 + s, y, PRESS_Z), label="hold the press")
            self.go(0.8, (x0 + s, y, PRESS_Z + 0.03), label="lift")
        return self


def demo():
    L, R = _Plan(LEFT_PARK, ACROSS_L), _Plan(RIGHT_PARK, ACROSS)
    x0, yb = PAN_X, Y_BREADING
    # 1. left: out of the marinade, into the breading, first presses
    L.go(2.0, (x0, Y_MARINADE, GRASP_Z + 0.08), FINGER_OPEN, "over the tender")
    L.go(1.5, (x0, Y_MARINADE, GRASP_Z), label="fingers around it")
    L.go(0.8, (x0, Y_MARINADE, GRASP_Z), FINGER_CLOSED, "pick it up")
    L.go(1.2, (x0, Y_MARINADE, LIFT), label="lift out of the marinade")
    L.go(2.0, (x0, yb, LIFT), label="over the breading")
    L.go(1.5, (x0, yb, GRASP_Z + 0.003), label="lay it flat in the flour")
    L.go(0.8, (x0, yb, GRASP_Z + 0.003), FINGER_OPEN, "let go")
    L.go(1.0, (x0, yb, GRASP_Z + 0.05), label="up")
    L.presses(x0, yb)
    # parked while the right hand takes the near end: waiting over the far half, the left hand stopped the right one
    # 2.4 cm short of its grip in the simulator (the right hand's later grip, with the left parked, takes it)
    L.go(2.0, LEFT_PARK, FINGER_OPEN, "out of the way")
    # 2. right: take the near end, lift, roll onto its edge
    R.until(L.t - 1.5)
    xr = x0 + NEAR_GRIP
    R.go(2.0, (xr, yb, GRASP_Z + 0.08), R_APPROACH, "over the near end")
    R.go(1.5, (xr, yb, GRASP_Z), label="fingers around it")
    R.go(0.8, (xr, yb, GRASP_Z), FINGER_CLOSED, "grip the near end")
    R.go(1.5, (xr, yb, FLIP_Z), label="lift")
    R.go(2.0, (xr, yb, FLIP_Z), label="roll it onto its edge", roll=math.pi / 2)
    # 3. left: rolled the other way, take the far half from the +y side
    L.until(R.t)
    xl = x0 + FAR_GRIP
    L.go(3.0, (xl, HAND_Y + 0.06, FLIP_Z), FINGER_OPEN, "roll the hand, beside the far half", roll=-math.pi / 2)
    L.go(1.5, (xl, HAND_Y, FLIP_Z), label="fingers over and under it")
    L.go(0.8, (xl, HAND_Y, FLIP_Z), FINGER_CLOSED, "grip the far half")
    R.until(L.t)
    R.go(0.8, (xr, yb, FLIP_Z), FINGER_OPEN, "let go")
    R.go(1.5, (xr, yb - 0.06, FLIP_Z), label="back out")
    R.go(2.0, RIGHT_PARK, label="park", roll=0.0)
    # 4. left: roll level (the tender turns over), lay it back, press the other face
    L.until(R.t - 2.0)
    L.go(2.0, (xl, HAND_Y, FLIP_Z), label="roll level: the tender is turned over", roll=0.0)
    L.go(1.5, (xl, yb, GRASP_Z + 0.06), label="over the breading")
    L.go(1.5, (xl, yb, GRASP_Z + 0.004), label="lay it in the flour")
    L.go(0.8, (xl, yb, GRASP_Z + 0.004), FINGER_OPEN, "let go")
    L.go(1.0, (xl, yb, GRASP_Z + 0.05), label="up")
    L.presses(x0, yb)
    L.go(2.0, LEFT_PARK, FINGER_OPEN, "park")
    # 5. right: take the same near end again, carry it to the rack, let go
    R.until(L.t - 1.0)
    R.go(2.0, (xr, yb, GRASP_Z + 0.08), R_APPROACH, "over the same near end")
    R.go(1.5, (xr, yb, GRASP_Z), label="fingers around it")
    R.go(0.8, (xr, yb, GRASP_Z), FINGER_CLOSED, "grip the near end")
    R.go(1.2, (xr, yb, LIFT), label="lift")
    R.go(2.0, (xr, Y_RACK, LIFT), label="over the rack")
    rack_tcp = GRATE_TOP + 0.004 + TENDER_SIZE[2] / 2 + CENTER_TO_TCP
    R.go(1.5, (xr, Y_RACK, rack_tcp), label="lay it on the rack")
    R.go(0.8, (xr, Y_RACK, rack_tcp), FINGER_OPEN, "let go")
    R.go(1.2, (xr, Y_RACK, LIFT), label="lift clear")
    R.go(2.5, (xr, Y_RACK, LIFT), label="wait")
    L.until(R.t)
    return {"left": L.wps, "right": R.wps}


DEMO_SECONDS = None  # filled below for the README check


def _demo_seconds() -> float:
    d = demo()
    return max(sum(w.duration for w in d[a]) for a in d)


DEMO_SECONDS = _demo_seconds()
