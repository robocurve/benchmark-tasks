"""dosekit: measuring dry solids (flour, sugar, powders, sand, grout, concrete, fertilizer) and a scale, for a
yam_arena task file (built on yamkit and pourkit; edits neither).

The simulator has no particles, so a solid here is a scored bulk model, like pourkit's liquids. A `Bin` gives a task
object a solid-holding interior (stacked cylinder sections, optionally around a central core such as a post in a
hole); a scoop or spoon is a `Bin` on a tool, offset from its origin and opening along any axis. After every control
step the kit reads the poses and moves material:

- Bulk density. Each `Material` has a loose (poured or spooned) density, a dipped density (what a container pushed
  into the material comes out holding), a tapped density and a fully packed density, in g/mL. A bin's contents are a
  stack of layers, bottom to top, each with its material, grams, density, provenance and the rod/tamp strokes it got.
  Material that falls (a pour, a strike-off, a drain) arrives loose; material picked up by dipping arrives at the
  dipped density. So a cup dipped into flour and leveled holds about 20% more flour than one spooned full and leveled,
  with nothing but poses as input.
- Heap. A bin fills to its brim and then heaps above it: the heap is kept on a grid of cells over the mouth, each at
  most tan(angle of repose) x its distance to the rim (a cone for an untouched heap). More than that rolls off over
  the rim. Tilting a bin first sheds its heap (the allowed slope is repose - tilt), and past the angle of repose the
  body pours like a liquid tilted by (tilt - repose) (pourkit's tilt table), from the top layer down, at most
  `Material.flow` mL/s per cm^2 of mouth. A caked layer (`cake` > 0.3) does not flow, nor does anything beneath it,
  until jolts break it.
- Falling material drops straight down from the lip and lands in the first bin whose mouth (tilted at most
  `receive_tilt`) the plumb line crosses (or that it starts inside, up to LAND_BELOW = 2 cm under its rim), else
  on the table as a spill pile at that x, y (on a scale's pan, if over
  one). Piles merge within 1.5 cm.
- Dipping (scooping). (Start scoops clear of the material: one that starts buried fills at the first step.) A bin
  with `scoop=True` whose mouth center goes more than 4 mm below another (upright) bin's
  material surface, inside its mouth, fills from that bin's top layers up to its own brim plus a full heap at its
  current tilt, within `dip_s` seconds. That is how a scoop picks up powder, and also how a measuring cup gets packed
  when it is dipped into the canister.
- Pressing (tamping, rodding, the back of a spoon). A `Presser` is a tool tip. A stroke counts when the tip goes more
  than 3 mm below a bin's surface inside its mouth and then comes back up. Each stroke moves the material in a band
  toward the packed density by `gain` of the remaining gap and adds a stroke to the layers there: for a rod
  (`depth=None`) the band runs from 6 mm below the deepest point the tip reached up to the surface; for a flat foot (`depth`
  set) it is the top `depth` of the material, wherever the tip went (the material isn't solid, so a foot could pass
  through it in the sim; a real one stops on the surface). Lifts: material that arrives after a stroke starts a new
  lift, so a bin knows whether it was filled in layers and how many strokes each got.
- Striking off. An `Edge` is a straight edge on a tool (a knife's back, a leveler, a rod, a screed). Where the edge
  sweeps over a bin's heap cells (between its segment at the last step and now, or within 4 mm of it) at a height
  between 8 mm below the rim and the heap's top, the heap above the edge is cut to the edge. Only an edge held
  within about 27 degrees of level strikes (a rod held upright for rodding does not). The cut material falls
  off the rim on the side the edge is moving toward (into whatever bin is under it, else spilled).
- Jolts. A bin whose object's velocity changes by more than `jolt_dv` m/s in one control step (tapped on the
  counter, shaken, dropped) settles: each uncaked layer moves `jolt_gain` toward the tapped density, and caked
  layers lose `cake_break` of their cake. A tap snapped down in one step and stopped by the table is two jolts (the
  start and the stop); shaking is one per reversal.
- Drains. `drain(objects)` gives mL/s leaving through the bottom (a spreader's open gate, a funnel), from the bottom
  layer, landing below `drain_at`.
- Scales. A `Scale` is a fixed pan with a readout. Its gross reading is the grams of every listed loose object resting
  on the pan (low face within 8 mm of the pan's top or of another object already on it, center over it, not lifted),
  plus the material in bins on those objects, plus spill piles on the pan. Pressing its tare (a yamkit push-button
  link past `press_at`, or a tool point at `touch`) zeroes it: net = gross - tare, latched on each press's rising
  edge, logged with what was on the pan. The display shows the net, rounded to `resolution`.

Real numbers (all approximate; the presets in `MATERIALS` use them): all-purpose flour 120 g per spooned, leveled US
cup (236.6 mL), 0.51 g/mL (King Arthur Baking ingredient weight chart), about 145 g when the cup is dipped into the
bag (0.61 g/mL), tapped about 0.70; granulated sugar 200 g/cup (0.85 g/mL); brown sugar 213 g per firmly packed cup
(0.90 g/mL) against about 0.60 loose; powdered infant formula about 0.50 g/mL loose (a level 8.7 g scoop holds about
17.5 mL; the label's scoop weight is for a scoop dipped into the tin and leveled, so its dipped density is the
loose one), 0.64 packed; dry sand 1.45 loose to 1.70 compacted, repose 34 deg; fresh concrete about 2.20
g/mL unconsolidated to 2.40 consolidated (ASTM C138 unit weights near 2.4); copper(II) sulfate pentahydrate crystals
about 1.10 loose; granular lawn fertilizer about 1.0 g/mL, repose 30 deg. Angles of repose: free-flowing granules
30-35 deg, cohesive powders (flour, formula, cement) 40-50 deg.

Scorer API (after the kit has updated for the step, which it does inside the scene's object_states):
`amount(objects, bin, material=None)` grams; `volume(objects, bin)` mL; `heap(objects, bin)` highest heap point
above the rim, m; `heaped(objects, bin, tol)`; `level_struck(objects, bin, tol, fill_tol)` full to the brim and flat;
`packing(objects, bin)` 0 loose .. 1 packed (mass-weighted); `density(objects, bin)`; `lifts(objects, bin)` list of
`Lift`; `strokes(objects, bin)`; `stroke_log(objects, bin)` (each stroke: its depth, the lift it was made in and
how far below that lift's bottom it reached, and in which direction from the axis); `taps(objects, bin)` (each
`Tapper` tap on a bin's outside, e.g. mallet taps on a mold, tagged with the lift on top); `struck(objects, bin)` strike-off steps and `struck_grams`;
`swept_since(objects, bin, step)` (share of the mouth an edge was drawn across since a step); `jolts(objects, bin)`;
`caked(objects, bin)`; `transfers()` (every move: pour, dip, strike, drain, overflow) and `transferred(objects, src,
dst, material, how)`; `dipped(objects, scoop, source)`; `dips()` (each with what the scoop had held before);
`touched(objects, bin)` (every bin whose material has been in it); `received(objects, bin, via)` grams now in `bin`
that passed through `via`; `spilled(objects, material, inside)`; `piles()`; `surface_z(objects, bin)`;
`gross_grams / net_grams / reading(objects, scale)`; `tares(objects, scale)`; `tared_with(objects, scale, names,
empty=True)`; `on_scale(objects, scale)`.

Visuals (Isaac Lab VisualizationMarkers, guarded so the model imports and tests offline): each bin's body as a
column per section, the heap as one short column per cell, spill piles as flat disks, a seven-segment readout per
`Display` (digits 3 cm tall are about 13 px at 320x176 from the top camera) and an optional bar gauge with a target
tick. Packed material is drawn darker. Visuals never affect scoring. Drawn right after every scene reset (the
embodiment's env.reset), before the first rendered frame, at the starting contents, and after every step.

Limits: bins are stacked cylinders; the material has no weight in the physics (only the scale counts it); it is not
solid (tools and fingers pass through it; nothing piles against a tool); a heap's shape is kept per cell but a pour
lands at the bin's center; a dip does not displace the source's surface; a strike-off drops the cut material at the
rim, not carried along in front of the edge; packing is per layer, uniform across the bin; a jolt is read from
the object's positions (velocity change per step), so a hard contact in PhysX could register as one (set
`jolt_dv=None` on a bin that should never settle).

Usage in task.py (next to yamkit.py, pourkit.py and dosekit.py):

    import yamkit as kit
    import dosekit as dk
    dk.setup(materials=[dk.MATERIALS["flour"]],
             bins=[dk.Bin("canister", sections=((0.06, 0.10),), floor=-0.05, fill={"flour": 500.0}, scoop=False),
                   dk.Bin("cup", sections=((0.0415, 0.0437),), floor=-0.02),
                   dk.Bin("spoon", obj="spoon", at=(0.07, 0, 0), sections=((0.022, 0.012),), floor=-0.006)],
             tools=[dk.Edge("knife", a=(-0.06, 0, 0.0), b=(0.06, 0, 0.0))],
             floor_z=TOP)
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

import pourkit as pk
import yamkit as kit

ML = 1e6  # mL per cubic metre
SPILL = "spill"
TILT_SLACK = 3.0  # degrees: a heap stands this far past its repose angle before it slides (sim jitter margin)
DIP_DEPTH = 0.004  # m: a scoop's mouth this far below a surface picks up
PRESS_MIN = 0.003  # m: a tip this far below the surface is pressing
EDGE_REACH = 0.004  # m: cells this close to the edge's segment are under it
EDGE_BELOW = 0.008  # m: an edge deeper than this below the rim strikes nothing (it is cutting into the body)
CAKE_HOLD = 0.3  # a layer caked more than this does not flow
PILE_MERGE = 0.015  # m
CELLS = 9  # heap grid across the mouth's diameter
DRAW_MIN = 0.001  # m: a body fill thinner than this (what a tipped scoop keeps on its lower wall) is drawn as a lump
# against that wall, on the side the bin was last tipped toward, not as a film over the whole floor
SWEEP_STEP = 0.03  # m: a bin's rim moving up to this far in a step, and up to SWEEP_DZ in height, sweeps
SWEEP_DZ = 0.002  # the whole band its mouth crossed under an edge between steps (a fast draw is not skipped over)
SWEEP_ABOVE = 0.002  # m: an edge this close above a cell's top still counts as drawn across it (swept_since)
LAND_BELOW = 0.02  # m: material let go up to this far below a bin's rim, inside its mouth, still lands in it (a
#                   scoop struck level with the tin's rim, a lip dipped into a neck); it used to count as a spill


# -- configuration ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Material:
    """A dry bulk solid. Densities in g/mL: `loose` as poured or spooned, `dipped` as picked up by pushing a container
    into it (default half way to packed), `tapped` after jolting (default 0.6 of the way), `packed` fully pressed.
    `repose` is its angle of repose (degrees); `flow` its pour rate, mL/s per cm^2 of mouth; `press_gain` and
    `jolt_gain` the fraction of the gap to packed/tapped closed per stroke/jolt."""

    name: str
    loose: float
    packed: float
    dipped: float | None = None
    tapped: float | None = None
    repose: float = 35.0
    color: tuple = (0.92, 0.90, 0.84)
    flow: float = 4.0
    press_gain: float = 0.35
    jolt_gain: float = 0.2

    @property
    def rho_dip(self) -> float:
        return float(self.dipped if self.dipped is not None else self.loose + 0.5 * (self.packed - self.loose))

    @property
    def rho_tap(self) -> float:
        return float(self.tapped if self.tapped is not None else self.loose + 0.6 * (self.packed - self.loose))


MATERIALS = {
    "flour": Material("flour", loose=0.507, dipped=0.613, tapped=0.70, packed=0.75, repose=45.0,
                      color=(0.95, 0.93, 0.86), flow=2.0),
    "sugar": Material("sugar", loose=0.845, dipped=0.86, tapped=0.89, packed=0.92, repose=34.0,
                      color=(0.98, 0.98, 0.97), flow=6.0),
    "brown_sugar": Material("brown_sugar", loose=0.60, dipped=0.70, tapped=0.72, packed=0.90, repose=50.0,
                            color=(0.55, 0.36, 0.18), flow=1.5, press_gain=0.45),
    "formula": Material("formula", loose=0.497, dipped=0.50, tapped=0.62, packed=0.64, repose=42.0,
                        color=(0.97, 0.94, 0.80), flow=2.5),
    "sand": Material("sand", loose=1.45, dipped=1.52, tapped=1.60, packed=1.70, repose=34.0,
                     color=(0.80, 0.68, 0.45), flow=6.0),
    "concrete": Material("concrete", loose=2.20, dipped=2.22, tapped=2.30, packed=2.40, repose=38.0,
                         color=(0.58, 0.58, 0.56), flow=3.0, press_gain=0.12),
    "grout": Material("grout", loose=1.30, dipped=1.40, tapped=1.48, packed=1.55, repose=42.0,
                      color=(0.62, 0.60, 0.57), flow=3.0),
    "copper_sulfate": Material("copper_sulfate", loose=1.10, dipped=1.14, tapped=1.20, packed=1.25, repose=35.0,
                               color=(0.10, 0.45, 0.95), flow=5.0),
    "fertilizer": Material("fertilizer", loose=1.00, dipped=1.02, tapped=1.06, packed=1.10, repose=30.0,
                           color=(0.30, 0.55, 0.85), flow=8.0),
    "silica": Material("silica", loose=0.45, dipped=0.50, tapped=0.58, packed=0.62, repose=36.0,
                       color=(0.97, 0.97, 0.97), flow=4.0),
}


@dataclass(frozen=True)
class Bin:
    """A solid-holding interior. `name` is the bin's key; `obj` the task object it moves with (default `name`).
    The bin's frame is the object's, moved by `at` (object frame) and turned so its +z is `axis` (object frame).
    `sections` are (inner radius, height) stacked along that +z from `floor`; the top one's top is the rim. `core` is
    a central obstruction's radius (a post in a hole). `fill` is grams per material at the start, at packing `pack`
    (0 loose .. 1 packed) and caked by `cake` (0..1). `scoop` lets it pick material up when dipped; `dip_into` lets
    others dip into it. `jolt_dv` (m/s per control step, None = never) is the velocity change that counts as a jolt.
    `drain` is None or a callable(objects) -> mL/s leaving through the bottom, landing below `drain_at` (bin frame).
    `max_flow` overrides the pour rate (mL/s). `heap_color` (sRGB) draws the material standing above the rim in that
    color instead of the material's (a few px at 320x176 are otherwise the same shade as a level fill seen from
    above, so heaped and struck-level look alike); visuals only."""

    name: str
    sections: tuple = ((0.04, 0.05),)
    floor: float = 0.0
    obj: str | None = None
    at: tuple = (0.0, 0.0, 0.0)
    axis: tuple = (0.0, 0.0, 1.0)
    fill: dict = field(default_factory=dict)
    pack: float = 0.0
    cake: float = 0.0
    core: float = 0.0
    mouth: float | None = None
    receive_tilt: float = 60.0
    max_flow: float | None = None
    scoop: bool = True
    dip_into: bool = True
    dip_s: float = 0.4
    jolt_dv: float | None = 0.15
    cake_break: float = 0.25
    drain: object = None
    drain_at: tuple = (0.0, 0.0, 0.0)
    heap_color: tuple | None = None

    @property
    def body(self) -> str:
        return self.obj or self.name

    @property
    def height(self) -> float:
        return float(sum(h for _, h in self.sections))

    @property
    def mouth_r(self) -> float:
        return float(self.mouth if self.mouth is not None else self.sections[-1][0])

    @property
    def brim(self) -> float:
        """mL to the brim."""
        return float(sum(math.pi * (r * r - self.core * self.core) * h for r, h in self.sections) * ML)


@dataclass(frozen=True)
class Presser:
    """A pressing tip on task object `obj` (or a gripper key), at `tip` in its frame. `depth=None` is a rod (it
    consolidates down to where it reached, and `below` m past its tip, so a rod stopped short of a mold's floor, as
    ASTM C31 asks, still reaches the whole layer); a float is a flat foot that compacts the top `depth` m. `gain`
    overrides the material's press_gain."""

    obj: str
    tip: tuple = (0.0, 0.0, 0.0)
    depth: float | None = None
    gain: float | None = None
    below: float = 0.006


@dataclass(frozen=True)
class Tapper:
    """A point on task object `obj` (or a gripper key), at `tip` in its frame, that taps the outside of bin `bin`
    (a mallet's head, the side of a hand). A tap counts when the point comes within `radius` + `reach` of the bin's
    axis (set `radius` to the bin's outer radius plus the tool's half-thickness at `tip`) between `lo` and `hi` m above
    the bin's floor; it counts again only after the point has moved `release` m further out (a bounce is one tap). Taps change nothing in the model (no settling);
    they are only counted and logged (`taps`), each tagged with the lift on top at the time."""

    obj: str
    bin: str
    radius: float
    tip: tuple = (0.0, 0.0, 0.0)
    reach: float = 0.005
    release: float = 0.010
    lo: float = 0.0
    hi: float = 1.0
    min_r: float = 0.0  # a point closer to the axis than this is inside the bin (rodding, scooping), not tapping


@dataclass(frozen=True)
class Tap:
    """One tap on the outside of `bin` at control step `step` by tapper `tapper` (its index among the tappers), at
    height `h` m above the bin's floor, while lift `lift` was on top (0 if the bin was empty)."""

    step: int
    bin: str
    tapper: int
    h: float
    lift: int


@dataclass(frozen=True)
class Edge:
    """A straight edge on task object `obj` from `a` to `b` (its frame). `below` (m) is how far the edge may sit below
    a bin's rim and still strike (default EDGE_BELOW, 8 mm: a hand-held knife can be pushed down into the bin's mouth).
    For a fixed bar with a collider, which a scoop can only be drawn up against, set it to the contact slop (2 mm) so a
    scoop carried over the top of the bar strikes nothing."""

    obj: str
    a: tuple
    b: tuple
    below: float | None = None


@dataclass(frozen=True)
class Scale:
    """A fixed scale. `pan` is the pan's top center (world, m) and `half` its half size in x, y. `masses` are the grams
    of each loose task object that may be weighed. The tare is pressed when yamkit joint `button` is past `press_at`
    (sign included: -0.003 means pressed at 3 mm down), or when a gripper's tool point (or any object named in
    `touch_by`) is within `touch[3]` of `touch[:3]`. `capacity` g; over it the reading is None."""

    name: str
    pan: tuple
    half: tuple
    masses: dict = field(default_factory=dict)
    resolution: float = 0.1
    capacity: float = 2000.0
    button: str | None = None
    press_at: float = -0.003
    touch: tuple | None = None
    touch_by: tuple = ("left_gripper", "right_gripper")
    start_tare: float = 0.0


@dataclass(frozen=True)
class Display:
    """A seven-segment readout of scale `scale` drawn flat at world `at` (center), digits `digit_h` tall, reading
    left to right along `right` (xy) with `decimals` places; and, with `bar_len` > 0, a bar gauge beside it from 0 to
    `bar_full` g with a tick at `bar_target`."""

    scale: str
    at: tuple
    right: tuple = (0.0, -1.0)
    digits: int = 4
    decimals: int = 1
    digit_h: float = 0.03
    color: tuple = (1.0, 0.25, 0.08)
    bar_len: float = 0.0
    bar_full: float = 100.0
    bar_target: float | None = None


@dataclass(frozen=True)
class Transfer:
    """One continuous move of material from `source` to `dest` (a bin or "spill") by `how` ("pour", "dip", "strike",
    "drain", "overflow") over control steps `start`..`end`; `grams` per material."""

    source: str
    dest: str
    how: str
    start: int
    end: int
    grams: dict

    @property
    def total(self) -> float:
        return float(sum(self.grams.values()))


@dataclass(frozen=True)
class Dip:
    """A scoop (bin `scoop`) dipped into `source` from step `start` to `end`, picking up `grams`; `carried` is every
    bin whose material the scoop had held before this dip began (its `touched` set then)."""

    scoop: str
    source: str
    start: int
    end: int
    grams: float
    carried: frozenset


@dataclass(frozen=True)
class Lift:
    """A layer of filling put in between strokes: its `index` (0 = the starting fill), grams, bottom and top heights
    (m above the bin's floor, now), the fewest and most strokes any part of it got, and its packing (0..1)."""

    index: int
    grams: float
    bottom: float
    top: float
    strokes_min: int
    strokes_max: int
    packing: float


@dataclass(frozen=True)
class Stroke:
    """One finished press stroke into `bin` (a rod's or tamper's down-and-up) at control step `step` by presser
    `presser` (its index in setup's tools, among pressers): `deepest` is the lowest the tip reached and `band_lo` the
    bottom of the band it consolidated, both in m above the bin's floor; `lift` is the index of the lift on top when
    the stroke was made and `lift_bottom` that lift's bottom height then (before this stroke compacted it), so a
    scorer can tell which layer a stroke was made in and how far it reached into the one below
    (`lift_bottom - deepest`). `r` is the tip's distance from the bin's axis at its deepest (m)."""

    step: int
    bin: str
    presser: int
    deepest: float
    band_lo: float
    lift: int
    lift_bottom: float
    r: float
    ang: float = 0.0  # the tip's direction from the bin's axis at its deepest, radians in the bin's frame (x = 0)


@dataclass(frozen=True)
class TareEvent:
    step: int
    gross: float
    on: frozenset
    contents: float  # grams of material in bins on the pan at the press


_CFG = {"materials": {}, "bins": {}, "pressers": (), "edges": (), "tappers": (), "scales": {}, "displays": (), "dt": 0.05,
        "floor_z": 0.0}
_GEO: dict = {}


class _Layer:
    __slots__ = ("mat", "mass", "rho", "via", "cake", "lift", "strokes")

    def __init__(self, mat, mass, rho, via, cake=0.0, lift=0, strokes=0):
        self.mat, self.mass, self.rho, self.via, self.cake, self.lift, self.strokes = (
            mat, float(mass), float(rho), frozenset(via), float(cake), int(lift), int(strokes))

    @property
    def vol(self) -> float:
        return self.mass / self.rho

    def copy(self, mass=None):
        return _Layer(self.mat, self.mass if mass is None else mass, self.rho, self.via, self.cake, self.lift,
                      self.strokes)


class _Bin:
    def __init__(self, b: Bin):
        mats = _CFG["materials"]
        self.layers = []
        for m, g in b.fill.items():
            mat = mats[m]
            self.layers.append(_Layer(m, g, mat.loose + b.pack * (mat.packed - mat.loose), {b.name}, b.cake, 0, 0))
        self.heap = np.zeros(len(_GEO[b.name]["A"]))
        self.touched = {b.name} if b.fill else set()
        self.lift = 1
        self.stroked = False
        self.strokes = 0
        self.jolts = 0
        self.struck = 0
        self.struck_g = 0.0
        self.swept_at = np.full(len(_GEO[b.name]["A"]), -1)  # per heap cell: last step an edge passed over it


class _State:
    def __init__(self):
        self.episode = kit.episode()
        self.step = 0
        self.bins = {n: _Bin(b) for n, b in _CFG["bins"].items()}
        for n, b in _CFG["bins"].items():
            _sync(self.bins[n], b)
        self.piles: list = []  # [x, y, z, material, grams, scale name or None]
        self.transfers: list = []
        self.open: dict = {}
        self.dips: list = []
        self.stroke_log: list = []  # Stroke, in order
        self.tap_log: list = []  # Tap, in order
        self.tap_in: dict = {}  # tapper index -> True while it is against its bin
        self.dip_open: dict = {}
        self.press: dict = {}  # (presser index, bin) -> (deepest height (bin frame), radial offset there) while in
        self.edge_prev: dict = {}  # edge index -> (a, b) world at the last step
        self.rim_prev: dict = {}  # (edge index, bin) -> the bin's rim center (world) at the last step
        self.motion: dict = {}  # object -> (pos, vel)
        self.frames: dict = {}
        self.tare = {n: s.start_tare for n, s in _CFG["scales"].items()}
        self.tares = {n: [] for n in _CFG["scales"]}
        self.pressed = {n: False for n in _CFG["scales"]}
        self.gross = {n: 0.0 for n in _CFG["scales"]}
        self.on = {n: frozenset() for n in _CFG["scales"]}


_S = None


def setup(materials=(), bins=(), tools=(), scales=(), displays=(), control_hz: float = 20.0,
          floor_z: float = 0.0) -> None:
    """Declare the task's materials (`Material`s; presets in MATERIALS), bins, tools (`Presser`s, `Edge`s and
    `Tapper`s), scales
    and displays. `floor_z` is the table top's height (where spills land)."""
    global _S
    _CFG["materials"] = {m.name: m for m in materials}
    _CFG["bins"] = {b.name: b for b in bins}
    _CFG["pressers"] = tuple(t for t in tools if isinstance(t, Presser))
    _CFG["edges"] = tuple(t for t in tools if isinstance(t, Edge))
    _CFG["tappers"] = tuple(t for t in tools if isinstance(t, Tapper))
    _CFG["scales"] = {s.name: s for s in scales}
    _CFG["displays"] = tuple(displays)
    _CFG["dt"], _CFG["floor_z"] = 1.0 / control_hz, float(floor_z)
    for b in bins:
        for m in b.fill:
            if m not in _CFG["materials"]:
                raise ValueError(f"bin {b.name}: unknown material {m!r}")
    _GEO.clear()
    for b in bins:
        _GEO[b.name] = _geometry(b)
    _S = _State()


def reset() -> None:
    """Back to the starting contents, no history."""
    global _S
    _S = _State()


# -- geometry ----------------------------------------------------------------------------------


def _geometry(b: Bin) -> dict:
    r, c = b.mouth_r, b.core
    step = 2 * r / CELLS
    xs = -r + step * (np.arange(CELLS) + 0.5)
    X, Y = np.meshgrid(xs, xs, indexing="ij")
    rho = np.hypot(X, Y)
    keep = (rho < r) & (rho > c)
    X, Y, rho = X[keep], Y[keep], rho[keep]
    A = np.full(X.size, math.pi * (r * r - c * c) / X.size)  # equal cells that add up to the mouth's area
    d = r - rho
    if c > 0:
        d = np.minimum(d, rho - c)
    return {"x": X, "y": Y, "A": A, "d": np.maximum(d, 0.25 * step)}


def _rotm(q) -> np.ndarray:
    return pk._rotm(q)


def _area(b: Bin, i: int) -> float:
    r = b.sections[i][0]
    return math.pi * (r * r - b.core * b.core)


def _v_of_h(b: Bin, h: float) -> float:
    """mL below height h (from the floor), above the brim continuing at the top section's area."""
    v, z = 0.0, 0.0
    for i, (_, hh) in enumerate(b.sections):
        part = min(max(h - z, 0.0), hh)
        v += _area(b, i) * part
        z += hh
    if h > z:
        v += _area(b, len(b.sections) - 1) * (h - z)
    return v * ML


def _h_of_v(b: Bin, v: float) -> float:
    """Height from the floor of `v` mL (past the brim, at the top section's area)."""
    v = v / ML
    z = 0.0
    for i, (_, hh) in enumerate(b.sections):
        a = _area(b, i)
        if v <= a * hh:
            return z + v / a
        v -= a * hh
        z += hh
    return z + v / _area(b, len(b.sections) - 1)


def _held_body(b: Bin, tilt: float) -> float:
    """mL of the body (below the rim) a bin holds tilted `tilt` degrees past the slope its material can stand at."""
    if tilt <= 0:
        return b.brim
    v = pk.Vessel(f"dosekit:{b.name}", sections=tuple(b.sections), floor=0.0, mouth=b.mouth)
    return pk.held_at(v, tilt) * b.brim / v.capacity


def _heap_cap(b: Bin, slope_deg: float) -> np.ndarray:
    """Each cell's highest heap (m) at this slope (degrees)."""
    if slope_deg <= 0:
        return np.zeros(len(_GEO[b.name]["A"]))
    return math.tan(math.radians(min(slope_deg, 80.0))) * _GEO[b.name]["d"]


def _frame(objects, b: Bin):
    s = objects.get(b.body)
    if s is None:
        return None
    ro = _rotm(s.quat_xyzw)
    rot = ro @ kit._rot(np.asarray(b.axis, float))
    origin = np.asarray(s.pos, float) + ro @ np.asarray(b.at, float)
    axis = rot[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, axis[2]))))
    rim = origin + axis * (b.floor + b.height)
    down = np.array([0.0, 0.0, -1.0]) - axis * (-axis[2])
    n = np.linalg.norm(down)
    lip = rim + (down / n * b.mouth_r if n > 1e-6 else np.array([b.mouth_r, 0.0, 0.0]))
    return {"rot": rot, "origin": origin, "axis": axis, "tilt": tilt, "rim": rim, "lip": lip,
            "floor": origin + axis * b.floor}


# -- contents ------------------------------------------------------------------------------------


def _vol(st: _Bin) -> float:
    return float(sum(L.vol for L in st.layers))


def _heap_ml(st: _Bin, b: Bin) -> float:
    return float(np.sum(st.heap * _GEO[b.name]["A"]) * ML)


def _sync(st: _Bin, b: Bin) -> None:
    """Make the heap cells hold what is above the brim: scale them down, or add a cone-shaped share."""
    want = max(0.0, _vol(st) - b.brim)
    have = _heap_ml(st, b)
    if want <= 1e-9:
        st.heap[:] = 0.0
    elif want < have:
        st.heap *= want / have
    elif want > have + 1e-12:
        w = _GEO[b.name]["d"]
        st.heap += (want - have) / ML * w / float(np.sum(w * _GEO[b.name]["A"]))


def _flowable(st: _Bin, from_top=True) -> float:
    """mL that can leave from the top (or the bottom): down (up) to the first caked layer."""
    v = 0.0
    for L in (reversed(st.layers) if from_top else st.layers):
        if L.cake > CAKE_HOLD:
            break
        v += L.vol
    return v


def _take(st: _Bin, ml: float, from_top=True) -> list:
    """Remove `ml` of flowable material from the top (or bottom); returns the parcels (layer copies)."""
    out, left = [], ml
    seq = list(range(len(st.layers)))[::-1] if from_top else list(range(len(st.layers)))
    dead = set()
    for i in seq:
        if left <= 1e-12:
            break
        L = st.layers[i]
        if L.cake > CAKE_HOLD:
            break
        v = L.vol
        part = min(v, left)
        g = L.mass * part / v
        out.append(L.copy(g))
        L.mass -= g
        left -= part
        if L.mass <= 1e-9:
            dead.add(i)
    st.layers = [L for i, L in enumerate(st.layers) if i not in dead]
    return out


def _put(st: _Bin, b: Bin, parcels, how: str) -> None:
    """Material arriving in bin `b`: loose if it fell in, at the dipped density if picked up by dipping."""
    if not parcels:
        return
    if st.stroked:
        st.lift += 1
        st.stroked = False
    for p in parcels:
        mat = _CFG["materials"][p.mat]
        rho = mat.rho_dip if how == "dip" else mat.loose
        via = p.via | {b.name}
        st.touched |= set(via)
        top = st.layers[-1] if st.layers else None
        if (top is not None and top.mat == p.mat and top.lift == st.lift and abs(top.rho - rho) < 1e-9
                and top.via == via and top.cake <= 0 and top.strokes == 0):
            top.mass += p.mass
        else:
            st.layers.append(_Layer(p.mat, p.mass, rho, via, 0.0, st.lift, 0))


def _grams(parcels) -> dict:
    g = {}
    for p in parcels:
        g[p.mat] = g.get(p.mat, 0.0) + p.mass
    return g


def _log(src: str, dst: str, how: str, parcels) -> None:
    g = _grams(parcels)
    if sum(g.values()) <= 1e-12:
        return
    key = (src, dst, how)
    i = _S.open.get(key)
    if i is not None and _S.transfers[i].end >= _S.step - 1:
        t = _S.transfers[i]
        _S.transfers[i] = Transfer(src, dst, how, t.start, _S.step,
                                   {k: t.grams.get(k, 0.0) + g.get(k, 0.0) for k in set(t.grams) | set(g)})
    else:
        _S.transfers.append(Transfer(src, dst, how, _S.step, _S.step, g))
        _S.open[key] = len(_S.transfers) - 1


def _landing(objects, source_obj: str | None, point) -> str | None:
    """The bin whose mouth the plumb line from `point` crosses first below it (not one on `source_obj`)."""
    best, best_z = None, None
    for name, b in _CFG["bins"].items():
        if b.body == source_obj:
            continue
        fr = _S.frames.get(name)
        if fr is None or fr["tilt"] > b.receive_tilt or fr["axis"][2] <= 1e-6:
            continue
        axis, rim = fr["axis"], fr["rim"]
        t = float(np.dot(point - rim, axis) / axis[2])
        if t <= -LAND_BELOW:
            # below the rim: inside the bin's walls (a scoop's lip shedding under the powder) it stays in this bin
            if b.core <= _radial(b, fr, point) < b.mouth_r and float(np.dot(point - fr["floor"], axis)) >= 0.0:
                return name
            continue
        hit = point - np.array([0.0, 0.0, t])
        dist = float(np.linalg.norm(hit - rim))
        if b.core <= dist < b.mouth_r and (best is None or hit[2] > best_z):
            best, best_z = name, float(hit[2])
    return best


def _drop(objects, src: str, source_obj: str | None, point, parcels, how: str) -> str:
    """Parcels falling from `point`: into the bin below it, else a spill pile there. Returns where they went."""
    if not parcels:
        return SPILL
    dest = _landing(objects, source_obj, np.asarray(point, float))
    if dest is not None:
        _put(_S.bins[dest], _CFG["bins"][dest], parcels, how)
        _log(src, dest, how, parcels)
        return dest
    x, y = float(point[0]), float(point[1])
    on, z = None, _CFG["floor_z"]
    for n, sc in _CFG["scales"].items():
        if abs(x - sc.pan[0]) <= sc.half[0] and abs(y - sc.pan[1]) <= sc.half[1]:
            on, z = n, float(sc.pan[2])
    for p in parcels:
        for pile in _S.piles:
            if pile[3] == p.mat and pile[5] == on and math.hypot(pile[0] - x, pile[1] - y) < PILE_MERGE:
                tot = pile[4] + p.mass
                pile[0], pile[1] = (pile[0] * pile[4] + x * p.mass) / tot, (pile[1] * pile[4] + y * p.mass) / tot
                pile[4] = tot
                break
        else:
            _S.piles.append([x, y, z, p.mat, p.mass, on])
    _log(src, SPILL, how, parcels)
    return SPILL


def _surface(st: _Bin, b: Bin, fr, xy=None) -> float:
    """World z of the material's top along the bin's axis (plus the heap at the cell nearest `xy`, if heaped)."""
    v = _vol(st)
    if v < b.brim - 1e-9 or not np.any(st.heap > 0):
        return float((fr["floor"] + fr["axis"] * _h_of_v(b, min(v, b.brim)))[2])
    h = float(np.max(st.heap))
    if xy is not None:
        g = _GEO[b.name]
        rel = np.asarray(xy, float) - fr["rim"][:2]
        i = int(np.argmin((g["x"] - rel[0]) ** 2 + (g["y"] - rel[1]) ** 2))
        h = float(st.heap[i])
    return float(fr["rim"][2] + h)


def _radial(b: Bin, fr, p) -> float:
    rel = np.asarray(p, float) - fr["rim"]
    return float(np.linalg.norm(rel - np.dot(rel, fr["axis"]) * fr["axis"]))


# -- the step ----------------------------------------------------------------------------------


def update(objects) -> None:
    """Advance the solids by one control step from the poses in `objects`. The scene hook calls this once per step;
    tests call it directly."""
    global _S
    if _S is None or _S.episode != kit.episode():
        _S = _State()
    _S.step += 1
    dt = _CFG["dt"]
    bins = _CFG["bins"]
    _S.frames = {n: _frame(objects, b) for n, b in bins.items()}
    _jolts(objects, dt)
    for n, b in bins.items():  # drains
        fr = _S.frames[n]
        if fr is None or b.drain is None:
            continue
        rate = float(b.drain(objects))
        st = _S.bins[n]
        ml = min(max(rate, 0.0) * dt, _flowable(st, from_top=False))
        if ml > 1e-9:
            parcels = _take(st, ml, from_top=False)
            _sync(st, b)
            _drop(objects, n, b.body, fr["origin"] + fr["rot"] @ np.asarray(b.drain_at, float), parcels, "drain")
    for n, b in bins.items():  # tilt: shed the heap, then pour
        fr = _S.frames[n]
        if fr is not None:
            _pour(objects, n, b, fr, dt)
    _dips(objects, dt)
    _presses(objects)
    _taps(objects)
    _strikes(objects)
    for n, b in bins.items():
        _sync(_S.bins[n], b)
    _scales(objects)


def _pour(objects, n: str, b: Bin, fr, dt: float) -> None:
    st = _S.bins[n]
    if not st.layers:
        return
    top = _CFG["materials"][st.layers[-1].mat]
    eff = max(0.0, fr["tilt"] - TILT_SLACK)
    rate = (b.max_flow if b.max_flow is not None else top.flow * math.pi * (b.mouth_r * 100) ** 2) * dt
    flow = _flowable(st)
    if eff < top.repose:
        cap = _heap_cap(b, top.repose - eff)
        over = np.maximum(st.heap - cap, 0.0)
        excess = float(np.sum(over * _GEO[b.name]["A"]) * ML)
        out = min(excess, rate, flow)
        if out <= 1e-9:
            return
        st.heap -= over * (out / excess)
        how = "overflow" if fr["tilt"] < 10 else "pour"
    else:
        out = min(_heap_ml(st, b) + max(0.0, min(_vol(st), b.brim) - _held_body(b, eff - top.repose)), rate, flow)
        if out <= 1e-9:
            return
        how = "pour"
    parcels = _take(st, out)
    _sync(st, b)
    side = (fr["rot"].T @ (fr["lip"] - fr["rim"]))[:2]  # the lip's side, in the bin's own frame (for the draw)
    if np.linalg.norm(side) > 1e-6:
        st.tip_dir = side / np.linalg.norm(side)
    _drop(objects, n, b.body, fr["lip"], parcels, how)


def _jolts(objects, dt: float) -> None:
    seen = {}
    for n, b in _CFG["bins"].items():
        s = objects.get(b.body)
        if s is None:
            continue
        if b.body not in seen:
            p = np.asarray(s.pos, float)
            prev = _S.motion.get(b.body)
            v = (p - prev[0]) / dt if prev is not None else np.zeros(3)
            dv = float(np.linalg.norm(v - prev[1])) if prev is not None and prev[1] is not None else 0.0
            _S.motion[b.body] = (p, v if prev is not None else None)
            seen[b.body] = dv
        dv = seen[b.body]
        st = _S.bins[n]
        if b.jolt_dv is None or dv < b.jolt_dv or not st.layers:
            continue
        st.jolts += 1
        for L in st.layers:
            mat = _CFG["materials"][L.mat]
            if L.cake > CAKE_HOLD:
                L.cake = max(0.0, L.cake - b.cake_break)
            elif L.rho < mat.rho_tap:
                L.rho += mat.jolt_gain * (mat.rho_tap - L.rho)
        _sync(st, b)


def _dips(objects, dt: float) -> None:
    active = set()
    for sn, sb in _CFG["bins"].items():
        sf = _S.frames.get(sn)
        if not sb.scoop or sf is None or sf["tilt"] > 100:
            continue
        mouth = sf["rim"]
        for n, b in _CFG["bins"].items():
            fr = _S.frames.get(n)
            if n == sn or b.body == sb.body or not b.dip_into or fr is None or fr["tilt"] > 30:
                continue
            st = _S.bins[n]
            if not st.layers:
                continue
            r = _radial(b, fr, mouth)
            if not (b.core <= r < b.mouth_r) or mouth[2] < fr["floor"][2] - 0.01:
                continue
            if _surface(st, b, fr, mouth[:2]) - mouth[2] < DIP_DEPTH:
                continue
            ss = _S.bins[sn]
            mat = _CFG["materials"][st.layers[-1].mat]
            room = sb.brim + float(np.sum(_heap_cap(sb, mat.repose - max(0.0, sf["tilt"] - TILT_SLACK))
                                          * _GEO[sn]["A"]) * ML) - _vol(ss)
            # room and rate are in the scoop's (dipped) volume; take the same grams from the source's top layer
            ml = min(min(room, sb.brim / max(sb.dip_s, dt) * dt) * mat.rho_dip / st.layers[-1].rho, _flowable(st))
            if ml <= 1e-9:
                continue
            key = (sn, n)
            active.add(key)
            carried = frozenset(ss.touched)
            parcels = _take(st, ml)
            _sync(st, b)
            _put(ss, sb, parcels, "dip")
            _sync(ss, sb)
            _log(n, sn, "dip", parcels)
            g = sum(p.mass for p in parcels)
            i = _S.dip_open.get(key)
            if i is not None and _S.dips[i].end >= _S.step - 1:
                d = _S.dips[i]
                _S.dips[i] = Dip(sn, n, d.start, _S.step, d.grams + g, d.carried)
            else:
                _S.dips.append(Dip(sn, n, _S.step, _S.step, g, carried))
                _S.dip_open[key] = len(_S.dips) - 1


def _tool_point(objects, name: str, local) -> np.ndarray | None:
    s = objects.get(name)
    if s is None:
        return None
    return np.asarray(s.pos, float) + _rotm(s.quat_xyzw) @ np.asarray(local, float)


def _presses(objects) -> None:
    for k, P in enumerate(_CFG["pressers"]):
        tip = _tool_point(objects, P.obj, P.tip)
        for n, b in _CFG["bins"].items():
            fr = _S.frames.get(n)
            key = (k, n)
            if tip is None or fr is None or fr["tilt"] > 30 or b.body == P.obj:
                _S.press.pop(key, None)
                continue
            st = _S.bins[n]
            inside = b.core <= _radial(b, fr, tip) < b.mouth_r and tip[2] > fr["floor"][2] - 0.02
            surf = _surface(st, b, fr, tip[:2]) if st.layers else fr["floor"][2]
            if inside and st.layers and tip[2] < surf - PRESS_MIN:
                h = float(np.dot(tip - fr["floor"], fr["axis"]))
                if key not in _S.press or h < _S.press[key][0]:
                    rel = tip - fr["rim"]
                    ang = math.atan2(float(rel @ fr["rot"][:, 1]), float(rel @ fr["rot"][:, 0]))
                    _S.press[key] = (h, _radial(b, fr, tip), ang)
            elif key in _S.press and (not inside or tip[2] > surf - PRESS_MIN / 2):
                deepest, r, ang = _S.press.pop(key)
                _stroke(st, b, P, deepest, n, k, r, ang)


def _taps(objects) -> None:
    for k, T in enumerate(_CFG.get("tappers", ())):
        p = _tool_point(objects, T.obj, T.tip)
        b = _CFG["bins"].get(T.bin)
        fr = _S.frames.get(T.bin)
        if p is None or b is None or fr is None:
            continue
        r = _radial(b, fr, p)
        h = float(np.dot(p - fr["floor"], fr["axis"]))
        if not _S.tap_in.get(k):
            if T.min_r <= r <= T.radius + T.reach and T.lo <= h <= T.hi:
                _S.tap_in[k] = True
                st = _S.bins[T.bin]
                _S.tap_log.append(Tap(_S.step, T.bin, k, h, st.layers[-1].lift if st.layers else 0))
        elif r > T.radius + T.reach + T.release or not T.lo - T.release <= h <= T.hi + T.release:
            _S.tap_in[k] = False


def _stroke(st: _Bin, b: Bin, P: Presser, deepest: float, name: str = "", k: int = 0, r: float = 0.0,
            ang: float = 0.0) -> None:
    """One finished stroke: compact the band, count it and log it."""
    total = _vol(st)
    surf_h = _h_of_v(b, total)
    lo = max(0.0, surf_h - P.depth) if P.depth is not None else max(0.0, deepest - P.below)
    vlo = _v_of_h(b, lo)
    if st.layers:  # the top lift and its bottom before this stroke compacts anything
        top_lift, cum, bottom_v = st.layers[-1].lift, 0.0, None
        for L in st.layers:
            if L.lift == top_lift and bottom_v is None:
                bottom_v = cum
            cum += L.vol
        _S.stroke_log.append(Stroke(_S.step, name, k, float(deepest), float(lo), int(top_lift),
                                    float(_h_of_v(b, bottom_v or 0.0)), float(r), float(ang)))
    out, cum = [], 0.0
    for L in st.layers:  # split the layer that straddles the band's bottom
        v = L.vol
        if cum + 1e-12 < vlo < cum + v - 1e-12:
            below = (vlo - cum) / v
            out += [L.copy(L.mass * below), L.copy(L.mass * (1 - below))]
        else:
            out.append(L)
        cum += v
    cum = 0.0
    for L in out:
        v = L.vol
        if cum >= vlo - 1e-9:
            mat = _CFG["materials"][L.mat]
            g = P.gain if P.gain is not None else mat.press_gain
            L.rho += g * (mat.packed - L.rho)
            L.strokes += 1
        cum += v
    st.layers = out
    st.strokes += 1
    st.stroked = True
    _sync(st, b)


def _strikes(objects) -> None:
    for k, E in enumerate(_CFG["edges"]):
        a, bb = _tool_point(objects, E.obj, E.a), _tool_point(objects, E.obj, E.b)
        if a is None:
            continue
        prev = _S.edge_prev.get(k)
        _S.edge_prev[k] = (a, bb)
        pa, pb = prev if prev is not None else (a, bb)
        seg = bb - a
        if abs(seg[2]) > 0.5 * math.hypot(seg[0], seg[1]):
            continue  # not held level: no strike
        L2 = float(seg[0] ** 2 + seg[1] ** 2)
        for n, b in _CFG["bins"].items():
            fr = _S.frames.get(n)
            st = _S.bins[n]
            if fr is None or fr["tilt"] > 20 or b.body == E.obj:
                _S.rim_prev.pop((k, n), None)
                continue
            g = _GEO[n]
            cx, cy = fr["rim"][0] + g["x"], fr["rim"][1] + g["y"]  # cells, upright approximation
            # The edge's motion relative to the bin's mouth since the last step: the bin moved by `mv`, so in the
            # mouth's current frame the edge came from pa + mv, and a fast draw sweeps the whole band it crossed.
            rp = _S.rim_prev.get((k, n))
            _S.rim_prev[(k, n)] = np.array(fr["rim"], float)
            mv = np.zeros(3) if rp is None else np.array(fr["rim"], float) - rp
            if np.linalg.norm(mv[:2]) > SWEEP_STEP or abs(mv[2]) > SWEEP_DZ:
                mv = np.zeros(3)  # a jump or a change of height: only where the edge is now counts
            hit = _swept(np.stack([cx, cy], 1), pa[:2] + mv[:2], pb[:2] + mv[:2], a[:2], bb[:2])
            if not np.any(hit):
                continue
            t = np.clip(((cx - a[0]) * seg[0] + (cy - a[1]) * seg[1]) / L2, 0.0, 1.0) if L2 > 1e-12 else 0.0
            e = a[2] + t * seg[2] - fr["rim"][2]  # the edge's height above the rim at each cell
            below = EDGE_BELOW if E.below is None else E.below
            # swept: the edge passed over the cell at striking height, level with the material's top or below it
            # (within SWEEP_ABOVE), whether or not there was a heap to cut (see swept_since)
            st.swept_at[hit & (e > -below) & (e <= st.heap + SWEEP_ABOVE)] = _S.step
            if not np.any(st.heap > 1e-6):
                continue
            cut = hit & (e > -below) & (st.heap > np.maximum(e, 0.0))
            if not np.any(cut):
                continue
            new = np.where(cut, np.maximum(e, 0.0), st.heap)
            ml = float(np.sum((st.heap - new) * g["A"]) * ML)
            st.heap = new
            parcels = _take(st, ml)
            st.struck += 1
            st.struck_g += sum(p.mass for p in parcels)
            mid, pmid = (a + bb) / 2, (pa + pb) / 2
            d = (mid - pmid)[:2]
            if np.linalg.norm(d) < 1e-4:
                d = mid[:2] - fr["rim"][:2]
            d = d / (np.linalg.norm(d) or 1.0)
            drop = np.array([fr["rim"][0] + d[0] * (b.mouth_r + 0.012), fr["rim"][1] + d[1] * (b.mouth_r + 0.012),
                             fr["rim"][2] - 0.002])
            _sync(st, b)
            _drop(objects, n, b.body, drop, parcels, "strike")


def _swept(pts, pa, pb, a, b) -> np.ndarray:
    """Points inside the quad swept by segment pa-pb moving to a-b, or within EDGE_REACH of a-b."""
    def tri(p, u, v, w):
        d = (v[1] - w[1]) * (u[0] - w[0]) + (w[0] - v[0]) * (u[1] - w[1])
        if abs(d) < 1e-14:
            return np.zeros(len(p), bool)
        l1 = ((v[1] - w[1]) * (p[:, 0] - w[0]) + (w[0] - v[0]) * (p[:, 1] - w[1])) / d
        l2 = ((w[1] - u[1]) * (p[:, 0] - w[0]) + (u[0] - w[0]) * (p[:, 1] - w[1])) / d
        return (l1 >= 0) & (l2 >= 0) & (1 - l1 - l2 >= 0)
    inside = tri(pts, pa, pb, b) | tri(pts, pa, b, a)
    seg = np.asarray(b) - np.asarray(a)
    L2 = float(np.dot(seg, seg))
    t = np.clip(((pts - a) @ seg) / L2, 0, 1) if L2 > 1e-12 else np.zeros(len(pts))
    near = np.linalg.norm(pts - (np.asarray(a) + t[:, None] * seg), axis=1) <= EDGE_REACH
    return inside | near


def _on_pan(objects, sc: Scale) -> frozenset:
    on = set()
    pz = sc.pan[2]
    for name in sc.masses:
        s = objects.get(name)
        if s is None or s.low is None:
            continue
        if (pz - 0.006 <= s.low[2] <= pz + 0.008 and abs(s.pos[0] - sc.pan[0]) <= sc.half[0]
                and abs(s.pos[1] - sc.pan[1]) <= sc.half[1]):
            on.add(name)
    for _ in range(2):  # things standing on things on the pan
        for name in sc.masses:
            s = objects.get(name)
            if name in on or s is None or s.low is None:
                continue
            for o in list(on):
                u = objects[o]
                if (abs(s.low[2] - u.high[2]) <= 0.008 and u.low[0] <= s.pos[0] <= u.high[0]
                        and u.low[1] <= s.pos[1] <= u.high[1]):
                    on.add(name)
                    break
    return frozenset(on)


def _scales(objects) -> None:
    for n, sc in _CFG["scales"].items():
        on = _on_pan(objects, sc)
        gross = sum(sc.masses[o] for o in on)
        contents = sum(_mass(_S.bins[bn]) for bn, b in _CFG["bins"].items() if b.body in on)
        gross += contents + sum(p[4] for p in _S.piles if p[5] == n)
        _S.gross[n], _S.on[n] = float(gross), on
        pressed = False
        if sc.button is not None and sc.button in kit._JOINTS:
            q = kit._JOINTS[sc.button].pos
            pressed = q <= sc.press_at if sc.press_at < 0 else q >= sc.press_at
        if sc.touch is not None:
            for t in sc.touch_by:
                s = objects.get(t)
                if s is not None and np.linalg.norm(np.asarray(s.pos, float) - np.asarray(sc.touch[:3])) <= sc.touch[3]:
                    pressed = True
        if pressed and not _S.pressed[n]:
            _S.tare[n] = float(gross)
            _S.tares[n].append(TareEvent(_S.step, float(gross), on, float(contents)))
        _S.pressed[n] = pressed


def _mass(st: _Bin) -> float:
    return float(sum(L.mass for L in st.layers))


# -- scorer API ------------------------------------------------------------------------------


def _st(name: str) -> _Bin:
    return _S.bins[name]


def amount(objects, bin: str, material: str | None = None) -> float:
    """Grams in `bin` (of one material, or all)."""
    return float(sum(L.mass for L in _st(bin).layers if material is None or L.mat == material))


def volume(objects, bin: str) -> float:
    """mL the contents take up (including the heap)."""
    return _vol(_st(bin))


def heap(objects, bin: str) -> float:
    """The heap's highest point above the rim, m (0 if not heaped)."""
    h = _st(bin).heap
    return float(np.max(h)) if h.size else 0.0


def heaped(objects, bin: str, tol: float = 0.002) -> bool:
    """Material stands more than `tol` m above the rim somewhere."""
    return heap(objects, bin) > tol


def level_struck(objects, bin: str, tol: float = 0.002, fill_tol: float = 0.02) -> bool:
    """Full to the brim (within `fill_tol` of the brim volume) and flat (no heap above `tol` m)."""
    b = _CFG["bins"][bin]
    return volume(objects, bin) >= b.brim * (1 - fill_tol) and heap(objects, bin) <= tol


def packing(objects, bin: str) -> float:
    """0 (all loose) .. 1 (all fully packed), mass-weighted."""
    st = _st(bin)
    m = _mass(st)
    if m <= 0:
        return 0.0
    s = 0.0
    for L in st.layers:
        mat = _CFG["materials"][L.mat]
        s += L.mass * max(0.0, (L.rho - mat.loose) / max(mat.packed - mat.loose, 1e-9))
    return float(s / m)


def density(objects, bin: str) -> float:
    """Mean bulk density, g/mL."""
    v = volume(objects, bin)
    return amount(objects, bin) / v if v > 0 else 0.0


def lifts(objects, bin: str) -> list:
    """The layers of filling put in between strokes, bottom up (`Lift`s)."""
    b, st = _CFG["bins"][bin], _st(bin)
    out, cum = {}, 0.0
    for L in st.layers:
        v0, cum = cum, cum + L.vol
        d = out.setdefault(L.lift, {"g": 0.0, "lo": _h_of_v(b, v0), "hi": 0.0, "smin": 10 ** 9, "smax": 0, "p": 0.0})
        mat = _CFG["materials"][L.mat]
        d["g"] += L.mass
        d["hi"] = _h_of_v(b, cum)
        d["smin"], d["smax"] = min(d["smin"], L.strokes), max(d["smax"], L.strokes)
        d["p"] += L.mass * (L.rho - mat.loose) / max(mat.packed - mat.loose, 1e-9)
    return [Lift(i, d["g"], d["lo"], d["hi"], d["smin"], d["smax"], d["p"] / d["g"] if d["g"] else 0.0)
            for i, d in sorted(out.items())]


def taps(objects, bin: str | None = None) -> tuple:
    """Every tap on the outside of a bin this trial (`Tap`s, in order), on `bin` or any bin."""
    return tuple(t for t in _S.tap_log if bin is None or t.bin == bin)


def stroke_log(objects, bin: str | None = None) -> tuple:
    """Every finished press stroke this trial (`Stroke`s, in order), into `bin` or any bin."""
    return tuple(s for s in _S.stroke_log if bin is None or s.bin == bin)


def swept_since(objects, bin: str, step: int) -> float:
    """Share (0..1) of `bin`'s mouth cells an edge has passed over, at striking height, after control step `step`
    (e.g. the end of the scoop's last dip: was this load drawn across a leveler?)."""
    sw = _st(bin).swept_at
    return float(np.mean(sw > step)) if sw.size else 0.0


def strokes(objects, bin: str) -> int:
    """Strokes (rod, tamper, spoon back) the bin's material got this trial."""
    return _st(bin).strokes


def struck(objects, bin: str) -> int:
    """Control steps in which an edge cut material off the bin's heap."""
    return _st(bin).struck


def struck_grams(objects, bin: str) -> float:
    return _st(bin).struck_g


def jolts(objects, bin: str) -> int:
    return _st(bin).jolts


def caked(objects, bin: str) -> float:
    """Mass-weighted cake, 0 (free-flowing) .. 1."""
    st = _st(bin)
    m = _mass(st)
    return float(sum(L.mass * L.cake for L in st.layers) / m) if m > 0 else 0.0


def transfers(objects=None) -> tuple:
    return tuple(_S.transfers)


def transferred(objects, source: str, dest: str, material: str | None = None, how: str | None = None) -> float:
    """Grams moved from `source` to `dest` ("spill" for spills) this trial."""
    return float(sum(t.grams.get(material, 0.0) if material else t.total for t in _S.transfers
                     if t.source == source and t.dest == dest and (how is None or t.how == how)))


def dips(objects=None) -> tuple:
    return tuple(_S.dips)


def dipped(objects, scoop: str, source: str | None = None) -> float:
    """Grams `scoop` picked up by dipping (from `source`, or anywhere)."""
    return float(sum(d.grams for d in _S.dips if d.scoop == scoop and (source is None or d.source == source)))


def touched(objects, bin: str) -> frozenset:
    """Every bin whose material has been in `bin` this trial (its own start contents included)."""
    return frozenset(_st(bin).touched)


def received(objects, bin: str, via: str) -> float:
    """Grams now in `bin` that have passed through bin `via` (e.g. reagent returned to its stock jar)."""
    return float(sum(L.mass for L in _st(bin).layers if via in L.via))


def spilled(objects=None, material: str | None = None, inside=None) -> float:
    """Grams spilled this trial (of one material), optionally only piles inside (x0, x1, y0, y1)."""
    tot = 0.0
    for x, y, _, m, g, _ in _S.piles:
        if material is not None and m != material:
            continue
        if inside is not None and not (inside[0] <= x <= inside[1] and inside[2] <= y <= inside[3]):
            continue
        tot += g
    return float(tot)


def piles(objects=None) -> tuple:
    """Spill piles: (x, y, z, material, grams, scale name or None)."""
    return tuple(tuple(p) for p in _S.piles)


def surface_z(objects, bin: str) -> float:
    """World z of the material's top (the highest heap point if heaped)."""
    b = _CFG["bins"][bin]
    fr = _S.frames.get(bin) or _frame(objects, b)
    return _surface(_st(bin), b, fr)


def gross_grams(objects, scale: str) -> float:
    return float(_S.gross[scale])


def net_grams(objects, scale: str) -> float:
    """Gross minus tare, unrounded."""
    return float(_S.gross[scale] - _S.tare[scale])


def reading(objects, scale: str):
    """What the display shows: the net rounded to the resolution, or None over capacity."""
    sc = _CFG["scales"][scale]
    if _S.gross[scale] > sc.capacity:
        return None
    return round(round(net_grams(objects, scale) / sc.resolution) * sc.resolution, 6)


def tares(objects, scale: str) -> tuple:
    return tuple(_S.tares[scale])


def tared_with(objects, scale: str, names, empty: bool = True) -> bool:
    """The last tare press was made with exactly `names` on the pan (and, with `empty`, their bins holding less than
    half the resolution)."""
    t = _S.tares[scale]
    if not t:
        return False
    last = t[-1]
    return last.on == frozenset(names) and (not empty or last.contents < _CFG["scales"][scale].resolution / 2)


def on_scale(objects, scale: str) -> frozenset:
    return _S.on[scale]


def step() -> int:
    return _S.step if _S is not None else 0


# -- visuals -----------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
SEGMENTS = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc",
            "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", " ": ""}
MAX_PILES = 16


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def surf(rgb):
            return sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in rgb), roughness=0.8)

        protos, index = {}, {}
        for m in _CFG["materials"].values():
            for shade, f in (("", 1.0), ("_p", 0.72)):
                index[m.name + shade] = len(protos)
                protos[f"mat_{m.name}{shade}"] = sim_utils.CylinderCfg(
                    radius=1.0, height=1.0, axis="Z", visual_material=surf(tuple(c * f for c in m.color)))
        for d in _CFG["displays"]:
            if f"seg{d.color}" not in index:
                index[f"seg{d.color}"] = len(protos)
                protos[f"seg_{len(protos)}"] = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=surf(d.color))
        for name, rgb in marker_colors().items():
            if name.startswith("heap_"):
                index[name] = len(protos)
                protos[name] = sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=surf(rgb))
        index["dark"] = len(protos)
        protos["dark"] = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=surf((0.05, 0.05, 0.05)))
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/dosekit",
                                                                       markers=protos))
        _VIS["index"] = index
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"dosekit: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def marker_colors() -> dict:
    """The marker colors the visuals use: each material's (and its packed shade), and `heap_<bin>` for every bin with
    a `heap_color`. Pure Python, so tests can check it without Isaac Lab."""
    out = {}
    for m in _CFG["materials"].values():
        out[f"mat_{m.name}"] = tuple(m.color)
        out[f"mat_{m.name}_p"] = tuple(c * 0.72 for c in m.color)
    for n, b in _CFG["bins"].items():
        if b.heap_color is not None:
            out[f"heap_{n}"] = tuple(b.heap_color)
    return out


def _quat_from_rot(R) -> tuple:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    return (x, (R[0, 1] + R[1, 0]) / (4 * x), (R[0, 2] + R[2, 0]) / (4 * x), (R[2, 1] - R[1, 2]) / (4 * x))


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    idx = _VIS["index"]
    any_mat = next(iter(_CFG["materials"]))
    tr, qs, sc, ids = [], [], [], []

    def put(c, q, s, i):
        tr.append(tuple(np.asarray(c, float) + origin)), qs.append(tuple(q)), sc.append(tuple(s)), ids.append(i)

    def hide():
        put((0.0, 0.0, -10.0), (0, 0, 0, 1), (1e-5,) * 3, idx[any_mat])

    for n, b in _CFG["bins"].items():
        st, fr = _S.bins[n], _S.frames.get(n)
        g = _GEO[n]
        if fr is None or not st.layers:
            for _ in range(len(b.sections) + len(g["A"])):
                hide()
            continue
        shade = "_p" if packing(objects, n) > 0.5 else ""
        key = idx[st.layers[-1].mat + shade]
        q = _quat_from_rot(fr["rot"])
        h_body = _h_of_v(b, min(_vol(st), b.brim))
        z = 0.0
        for r, hh in b.sections:
            part = min(max(h_body - z, 0.0), hh)
            side = getattr(st, "tip_dir", None)
            if part > 1e-4 and h_body < DRAW_MIN and side is not None and z == 0.0:
                rc = 0.35 * r  # the residue lies in the corner of the floor and the wall it was tipped onto
                hc = _vol(st) / ML / (math.pi * rc * rc)
                off = side * (r * 0.97 - rc)
                put(fr["floor"] + fr["rot"] @ np.array([off[0], off[1], hc / 2]), q, (rc, rc, hc), key)
            elif part > 1e-4:
                put(fr["floor"] + fr["axis"] * (z + part / 2), q, (r * 0.97, r * 0.97, part), key)
            else:
                hide()
            z += hh
        cell_r = b.mouth_r / CELLS * 1.13
        hkey = idx.get(f"heap_{n}", key)
        for i in range(len(g["A"])):
            h = float(st.heap[i])
            if h < 5e-4:
                hide()
                continue
            c = fr["rim"] + fr["rot"] @ np.array([g["x"][i], g["y"][i], h / 2])
            ri = min(cell_r, b.mouth_r - math.hypot(g["x"][i], g["y"][i]))  # kept inside the mouth, off the rim
            put(c, q, (ri, ri, h), hkey)
    for k in range(MAX_PILES):
        if k < len(_S.piles):
            x, y, z, mat, grams, _ = _S.piles[k]
            r = min(0.04, math.sqrt(grams / _CFG["materials"][mat].loose / ML / (math.pi * 0.003)))
            put((x, y, z + 0.0015), (0, 0, 0, 1), (r, r, 0.003), idx[mat])
        else:
            hide()
    for d in _CFG["displays"]:
        _draw_display(objects, d, put, hide, idx)
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


def _draw_display(objects, d: Display, put, hide, idx) -> None:
    """Seven segments per digit, lit ones in the display's color and the rest hidden; then the bar and its tick."""
    val = reading(objects, d.scale)
    text = "-" * d.digits if val is None else f"{val:{d.digits + (d.decimals + 1 if d.decimals else 0)}.{d.decimals}f}"
    digits = [c for c in text if c != "."][-d.digits:]
    digits = [" "] * (d.digits - len(digits)) + digits
    rx, ry = d.right
    ux, uy = -ry, rx  # "up" on the readout, 90 degrees left of right
    yaw = math.atan2(ry, rx)
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    h, w, t = d.digit_h, d.digit_h * 0.55, d.digit_h * 0.15
    pitch = w + t * 2.2
    seg_key = idx[f"seg{d.color}"]
    # segment centers (along right, along up) and sizes (along right, along up) in digit units
    seg = {"a": ((0, h / 2), (w, t)), "g": ((0, 0), (w, t)), "d": ((0, -h / 2), (w, t)),
           "f": ((-w / 2, h / 4), (t, h / 2)), "b": ((w / 2, h / 4), (t, h / 2)),
           "e": ((-w / 2, -h / 4), (t, h / 2)), "c": ((w / 2, -h / 4), (t, h / 2))}
    x0 = -(d.digits - 1) * pitch / 2
    for k, ch in enumerate(digits):
        lit = SEGMENTS.get(ch, "")
        for s, ((cr, cu), (sr, su)) in seg.items():
            if s not in lit:
                hide()
                continue
            ar, au = x0 + k * pitch + cr, cu
            c = (d.at[0] + ar * rx + au * ux, d.at[1] + ar * ry + au * uy, d.at[2] + 0.0008)
            put(c, q, (sr, su, 0.0015), seg_key)
    if d.decimals:  # the decimal point before the last `decimals` digits
        ar = x0 + (d.digits - d.decimals) * pitch - pitch / 2
        au = -h / 2
        put((d.at[0] + ar * rx + au * ux, d.at[1] + ar * ry + au * uy, d.at[2] + 0.0008), q, (t, t, 0.0015), seg_key)
    if d.bar_len > 0:
        frac = 0.0 if val is None else min(max(val / d.bar_full, 0.0), 1.0)
        base = np.array(d.at[:2]) + np.array([ux, uy]) * (-h)  # the bar runs along `right` below the digits
        start = base - np.array([rx, ry]) * d.bar_len / 2
        c = start + np.array([rx, ry]) * d.bar_len * frac / 2
        put((c[0], c[1], d.at[2] + 0.0008), q, (max(d.bar_len * frac, 1e-5), t * 1.5, 0.0015), seg_key)
        if d.bar_target is not None:
            c = start + np.array([rx, ry]) * d.bar_len * min(d.bar_target / d.bar_full, 1.0)
            put((c[0], c[1], d.at[2] + 0.0012), q, (t * 0.6, t * 4, 0.002), idx["dark"])
        else:
            hide()


# -- hook into yam_arena.scene (after yamkit's and pourkit's) ------------------------------------


def _patch(scene) -> None:
    if getattr(scene, "_dosekit", False):
        return
    scene._dosekit = True
    inner = scene.object_states
    last = {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if not (_CFG["bins"] or _CFG["scales"]):
            return out
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
            update(out)
            if os.environ.get("DOSEKIT_DEBUG"):
                print("dosekit", _S.step, {n: round(amount(out, n), 2) for n in _CFG["bins"]},
                      {n: round(packing(out, n), 2) for n in _CFG["bins"]},
                      {n: reading(out, n) for n in _CFG["scales"]}, round(spilled(), 2), flush=True)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"dosekit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


# -- the reset draw (the embodiment's env.reset) ---------------------------------------------------
# object_states runs only after a step, so without this a trial's first rendered frames show empty bins and a blank
# readout. Like pourkit (and levelhose/dispersekit/siphonkit), the starting contents, heaps and scale readings are
# drawn right after every scene reset, before the first rendered frame, without touching the trial's state.


def _draw_start(env) -> None:
    global _S
    if not (_CFG["bins"] or _CFG["scales"]):
        return
    names = [b.body for b in _CFG["bins"].values()] + [n for sc in _CFG["scales"].values() for n in sc.masses]
    names += [t for sc in _CFG["scales"].values() for t in sc.touch_by]
    objs, origin = pk.start_objects(env, list(dict.fromkeys(names)))
    keep, _S = _S, _State()
    try:
        _S.frames = {n: _frame(objs, b) for n, b in _CFG["bins"].items()}
        try:
            _scales(objs)
        except Exception:  # noqa: BLE001  (the readout then shows the start tare)
            pass
        _draw(objs, origin)
    finally:
        _S = keep


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_dosekit", False):
        return
    cls._dosekit = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_dosekit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            if not _VIS["failed"]:
                try:
                    _draw_start(env)
                except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                    print(f"dosekit: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset, env._dosekit_reset = reset_and_draw, True

    cls._ensure_sim = _ensure_sim


class _Hook(pk._Hook):
    def exec_module(self, module):
        super().exec_module(module)
        _patch(module)


_patch_embodiment()
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's/pourkit's import hook with one that also adds the dose step after theirs
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- demo helpers --------------------------------------------------------------------------------

pour = pk.pour  # tip a held bin about its lip (see pourkit.pour); granular material starts to run at repose + tilt


def rim_point(bin: Bin | str, obj_pos, toward=(1.0, 0.0, 0.0)) -> np.ndarray:
    """World position of the rim point on the `toward` side of `bin`, its object upright and unturned at `obj_pos`
    (for pour()'s `lip_at_grip`)."""
    b = _CFG["bins"][bin] if isinstance(bin, str) else bin
    d = np.array([toward[0], toward[1], 0.0], float)
    d /= np.linalg.norm(d)
    rot = kit._rot(np.asarray(b.axis, float))
    rim = np.asarray(obj_pos, float) + np.asarray(b.at, float) + rot @ np.array([0.0, 0.0, b.floor + b.height])
    return rim + d * b.mouth_r


def mouth_center(bin: Bin | str, obj_pos) -> np.ndarray:
    """World position of the rim's center, the object upright and unturned at `obj_pos`."""
    b = _CFG["bins"][bin] if isinstance(bin, str) else bin
    rot = kit._rot(np.asarray(b.axis, float))
    return np.asarray(obj_pos, float) + np.asarray(b.at, float) + rot @ np.array([0.0, 0.0, b.floor + b.height])


def _offset_plan(tcp_at_grip, point_at_grip, roll, pitch, yaw, finger, label):
    d = np.asarray(point_at_grip, float) - np.asarray(tcp_at_grip, float)

    def at(target, dur, what):
        return kit.Tilted(dur, tuple(np.asarray(target, float) - d), finger, f"{label}: {what}", yaw, roll, pitch)
    return at


def dip(tcp_at_grip, mouth_at_grip, into, depth: float, *, drag=(0.0, 0.0), above: float = 0.05, roll=0.0,
        pitch=0.0, yaw=0.0, finger=0.0, down_s=1.0, drag_s=1.0, label="dip"):
    """Waypoints that dip a held scoop: its mouth (at `mouth_at_grip` when the tool point was at `tcp_at_grip`) over
    `into` (x, y, surface z), `above` higher, down to `depth` below it, along `drag` (m, x and y), and back up. The
    grip's orientation stays as given."""
    at = _offset_plan(tcp_at_grip, mouth_at_grip, roll, pitch, yaw, finger, label)
    x, y, z = into
    return [at((x, y, z + above), 1.2, "over the material"),
            at((x, y, z - depth), down_s, "into it"),
            at((x + drag[0], y + drag[1], z - depth), drag_s, "through it"),
            at((x + drag[0], y + drag[1], z + above), 1.0, "lift out")]


def strike(tcp_at_grip, edge_at_grip, rim_center, *, across=(0.0, 1.0), span: float = 0.1, lift: float = 0.03,
           roll=0.0, pitch=0.0, yaw=0.0, finger=0.0, slow_s=1.5, label="strike off"):
    """Waypoints that draw a held straight edge (its middle at `edge_at_grip` when the tool point was at
    `tcp_at_grip`) across a rim at `rim_center`: from `span`/2 before it along `across`, at rim height, to `span`/2
    past it, then up `lift`. Draw it toward the bin the excess should fall into."""
    at = _offset_plan(tcp_at_grip, edge_at_grip, roll, pitch, yaw, finger, label)
    c = np.asarray(rim_center, float)
    u = np.array([across[0], across[1], 0.0]) / math.hypot(*across)
    p0, p1 = c - u * span / 2, c + u * span / 2
    return [at(p0 + [0, 0, lift], 1.2, "over the rim"), at(p0, 0.8, "down to the rim"),
            at(p1, slow_s, "draw across"), at(p1 + [0, 0, lift], 0.8, "lift")]


def tamp(tcp_at_grip, tip_at_grip, points, z_top: float, z_bottom: float, *, strokes: int = 25, down_s=0.35,
         up_s=0.35, roll=0.0, pitch=0.0, yaw=0.0, finger=0.0, label="tamp"):
    """`strokes` up-and-down strokes of a held tool's tip (at `tip_at_grip` when the tool point was at `tcp_at_grip`)
    from `z_top` to `z_bottom`, cycling over the (x, y) `points` (spread them over the cross-section)."""
    at = _offset_plan(tcp_at_grip, tip_at_grip, roll, pitch, yaw, finger, label)
    out = []
    for k in range(strokes):
        x, y = points[k % len(points)]
        out += [at((x, y, z_top), up_s, f"stroke {k + 1} up"), at((x, y, z_bottom), down_s, f"stroke {k + 1} down")]
    x, y = points[(strokes - 1) % len(points)]
    out.append(at((x, y, z_top), up_s, "out"))
    return out


def jolt(tcp, *, drop: float = 0.02, n: int = 3, down_s: float = 0.08, up_s: float = 0.4, roll=0.0, pitch=0.0,
         yaw=0.0, finger=0.0, label="tap"):
    """`n` taps of a held container on the table: from `tcp` quickly down `drop` (aim it below where the container
    meets the table, so the table stops it short) and back up."""
    x, y, z = tcp
    out = []
    for k in range(n):
        out += [kit.Tilted(down_s, (x, y, z - drop), finger, f"{label} {k + 1}", yaw, roll, pitch),
                kit.Tilted(up_s, (x, y, z), finger, f"{label} {k + 1} up", yaw, roll, pitch)]
    return out


# -- a scale prop ----------------------------------------------------------------------------------


def scale_prop(name: str, pos, *, pan=(0.12, 0.12), pan_h=0.006, body=(0.16, 0.20, 0.03), button_at=(-0.055, 0.0),
               button=(0.024, 0.024, 0.008), travel=0.004, spring=60.0, mass=0.06,
               body_look=None, pan_look=None, button_look=None, window_look=None, window=(0.04, 0.10)):
    """A fixed kitchen/top-loading scale as a yamkit Mechanism: a housing `body` (x, y, z m) with its pan on top
    (its top at pos z + body z + pan_h), a dark display window toward -x (the robot) and a tare push button at
    `button_at` (x, y from the center) that travels `travel` down against a force spring. Returns (mechanism, pan top
    center, button joint key). Pair it with `Scale(..., pan=..., button=key, press_at=-travel * 0.6)` and a `Display`
    over the window."""
    bl = body_look or kit.Look((0.85, 0.85, 0.86), 0.4)
    pl = pan_look or kit.Look((0.80, 0.81, 0.83), 0.25, metallic=0.8)
    btl = button_look or kit.Look((0.10, 0.55, 0.20), 0.5)
    wl = window_look or kit.Look((0.04, 0.05, 0.05), 0.3)
    bx, by, bz = body
    meshes = (kit.box((0, 0, bz / 2), body, bl),
              kit.box((0.02, 0, bz + pan_h / 2), (pan[0], pan[1], pan_h), pl),
              kit.box((-bx / 2 + window[0] / 2 + 0.005, 0, bz + 0.0005), (window[0], window[1], 0.001), wl))
    colliders = (kit.Collider((0, 0, bz / 2), size=body),
                 kit.Collider((0.02, 0, bz + pan_h / 2), size=(pan[0], pan[1], pan_h)))
    cap = (0.0, 0.0, button[2] / 2)
    link = kit.Link("tare", "prismatic", "Z", anchor=(button_at[0], button_at[1], bz), lower=-travel, upper=0.0,
                    start=0.0, mass=mass, meshes=(kit.box(cap, button, btl),),
                    colliders=(kit.Collider(cap, size=button),), spring=spring, rest=0.0, damping=0.5)
    mech = kit.Mechanism(name, pos=tuple(pos), size=(bx, by, bz + pan_h), meshes=meshes, colliders=colliders,
                         links=(link,))
    pan_top = (pos[0] + 0.02, pos[1], pos[2] + bz + pan_h)
    return mech, pan_top, f"{name}/tare"
