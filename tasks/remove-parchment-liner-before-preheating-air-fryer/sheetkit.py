"""sheetkit: thin sheets (paper) as a fine row of rigid strips on elastic-plastic hinges, for a
yam_arena task file (import after yamkit). Version 2.1.

    import yamkit as kit
    import sheetkit as sk
    SHEET = sk.Sheet.uniform("sheet", width, length, paper, pos)   # or sk.Sheet(name, width, length, hinges, ...)
    OBJECTS = [..., SHEET.mechanism()]
    st = sk.state(objects, SHEET)          # in score(): strip frames, hinge angles and each hinge's permanent set

THE MODEL
- The sheet is cut across its length into equal strips (`Sheet.uniform`, default pitch 6.7 mm), rigid thin boxes joined edge to edge by revolute hinges whose axes run along the width. Every hinge is
  identical, so no fold line is pre-placed: a fold made anywhere lands on the hinge(s) nearest to it, within half a
  pitch. The sheet bends about lines parallel to its width only (no diagonal folds, no corner lifts): pick the sheet's
  orientation so the task's folds run that way.
- Elastic bending: hinge stiffness k = D W / l from the material's measured bending stiffness D (flexural rigidity per
  unit width, N*m), W the width and l the strip pitch. `cantilever()` checks k against the Peirce cantilever test
  (ASTM D1388) offline: the chain droops as the real sheet does.
- Plastic bending (creases emerge): each hinge is elastic-perfectly-plastic. It keeps a rest angle (its permanent set)
  and may move at most an elastic range e away from it; bent further, the rest angle follows (the excess becomes set).
  The elastic range starts at e0 = pitch / yield radius (a free bend that curves the strip tighter than the radius at
  which the material starts to yield, R_y = t / (2 eps_y), takes a set, as paper rolled round a pencil keeps a curl)
  and narrows toward the material's crease springback as the fold gets sharper (a fold pressed flat keeps nearly all of
  its angle). Bending back past the elastic range reduces the set (a crease can
  be flattened, as in real paper). Nothing in the scene holds a hinge at a fold line: the set appears where, and only
  where, the robot (or a fixture) bends the sheet past yield. Readout: the set angles per hinge (`state().sets`).
- Tearing (`Perforation`): a perforated edge is a row of ties, each a stiff elastic PhysX D6 joint from a strip to
  the world (the part it is torn from): rotations free (a small land of paper carries no moment), translations on an
  implicit spring that stretches well
  under a millimeter before the tie's strength = the paper's tensile strength x the tie's land (the uncut length of
  the perforation per tie). Every control step the kit reads each tie's stretch from the strip's pose, so its load is
  spring x stretch, and switches the joint off for good (USD physics:jointEnabled) once that load passes the
  strength; the tear then runs strip by strip as the robot peels. PhysX's own breakForce is NOT used: on GPU it broke
  at a small and varying fraction of the authored force on GPU, not in proportion.
  Readouts: `tear_length()` (geometry) and `tie_report()` (the simulator's broken flags and peak loads).
- Contact and grip: real colliders (boxes `collider_t` thick) with the material's own friction; the fingertip links
  get a rubber pad material (combine "max") so a pad pressing on the sheet grips it and the sheet slides on the table.
  No attachments, no snap: the sheet moves only by contact.
- Stability: every hinge gets a
  PhysX joint armature k (dt / omega_dt)^2 (extra joint-space inertia, no weight), a joint speed cap, a fold stop that
  keeps non-adjacent strips a collider apart when folded flat, a 2 mm strip contact offset and a capped
  depenetration speed. `stability()` reports the numbers the PhysX articulation guide asks about.

WHY STRIPS AND NOT A PHYSX FEM SURFACE (femclothkit): the FEM surface has no plasticity (its rest shape is fixed, so a
fold springs back to a loop and never creases), it cannot be gripped
except through a thick contact offset, and it has no tearing. The strip chain creases, tears, is gripped like any thin
rigid part, costs n - 1 joints in one articulation (<= 64 links), and costs about as much as a cable of the same number of links.

LIMITS (state them in a task's fidelity text): one bending direction; strips are rigid across the width (a sheet
cannot twist or lift at one corner); a crease lies on a hinge, so a fold made between hinges while both halves are
held (a fixture, two flat hands) lands on the nearer hinge or splits over two (a line 3.7 mm off a 13 mm-pitch
hinge split over two hinges); keep the pitch at or under ~7 mm where crease position matters; the
sheet is `collider_t` thick for contact (a stack of n layers is n mm, not n x 0.1 mm); a sheet is at most 64 strips;
ties are checked at the 20 Hz control rate.

SOURCES: see the SOURCES dict below (reference and figure for each paper constant the kit supplies).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

import yamkit as kit

for _attr in ("_SHEET_NAMES", "_SHEET_JOINTS", "_SHEET_SPEED", "_SHEET_PAPER", "_SHEET_TIES", "_SHEET_TIE_MASS",
              "_SHEET_TIE_PATHS", "_SHEET_TIE_STATE"):
    if not hasattr(kit, _attr):
        setattr(kit, _attr, set() if _attr == "_SHEET_NAMES" else {})

G = 9.81
MAX_LINKS = 64  # links per PhysX articulation (PxArticulationReducedCoordinate; DY_ARTICULATION_MAX_SIZE)
try:
    from yam_arena.constants import PHYSICS_DT  # 1/240 s: 240 Hz physics, 12 substeps per 20 Hz control step
except ImportError:  # pragma: no cover
    PHYSICS_DT = 1 / 240
CONTROL_DT = 1 / 20  # the control step the sheet tasks run at (-E control_hz=20)

# Ties: PhysX's own physics:breakForce does not break in proportion to the authored force on GPU, so the kit breaks
# ties itself from their
# measured stretch. TIE_BREAK_STRETCH: how far a tie stretches at its strength (sets the spring, k = F / stretch).
TIE_BREAK_STRETCH = 0.0005

SOURCES = {
    "paper: yield strain (curl set)": (
        "Niskanen (ed.), Paper Physics, 2nd ed., Paperi ja Puu 2008, ch. 5: the linear part of a copy paper's tensile "
        "curve ends near 0.3-0.5 % strain", "eps_y 0.4 %"),
    "paper: crease springback": (
        "Thiria and Adda-Bedia, Phys. Rev. Lett. 107, 025506 (2011): a fold pressed flat and let go opens to a rest "
        "angle and then creeps only logarithmically", "35 deg"),
    "paper: friction": (
        "TAPPI T 815 (inclined plane) / T 549 (horizontal plane): office paper on paper, static 0.4-0.6, kinetic 0.3-0.5",
        "0.45 static / 0.35 kinetic, the middle of those ranges"),
    "pad friction": (
        "Derler and Gerhardt, Tribol. Lett. 45 (2012) 1-27 (skin 0.4-1.5 on smooth surfaces); rubber on paper 0.8-1.2",
        "pads 1.0 static / 0.8 kinetic, combine max"),
    "articulation stability": (
        "https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/dev_guide/guides/articulation_stability_guide.html",
        "'Avoid large mass and inertia ratios'; 'joint armature can help with stability'"),
}


@dataclass(frozen=True)
class Paper:
    """A sheet material. `areal_mass` kg/m^2, `thickness` m, `bending_stiffness` D in N*m (flexural rigidity per unit
    width), `yield_strain` (the surface strain at which bending starts to leave a set), `springback` rad (how far a fold
    pressed flat opens when let go), `friction` / `kinetic_friction`, `tensile_strength` N/m (for perforation ties),
    `damping_ratio` of each hinge (1 = critical), `source` where the numbers come from (see SOURCES)."""

    name: str
    areal_mass: float
    thickness: float
    bending_stiffness: float
    yield_strain: float
    springback: float
    friction: float
    kinetic_friction: float = 0.35
    tensile_strength: float = 3500.0
    damping_ratio: float = 1.0
    source: str = ""

    @property
    def bending_length(self) -> float:
        """Peirce bending length c = (D / (w g))^(1/3) in m (ASTM D1388: a strip pushed out flat over a table edge
        droops to 41.5 degrees at an overhang of 2c). A ruler check of D on the real sheet."""
        return (self.bending_stiffness / (self.areal_mass * G)) ** (1 / 3)

    @property
    def yield_radius(self) -> float:
        """The bend radius below which the material starts to keep a set: R_y = t / (2 eps_y) (surface strain t / 2R)."""
        return self.thickness / (2 * self.yield_strain)

    def elastic_range(self, pitch: float) -> float:
        """A hinge's elastic range for a free bend (rad): the angle at which a strip of length `pitch` bent uniformly
        reaches the yield radius, pitch / R_y (capped at 90 degrees)."""
        return float(min(pitch / self.yield_radius, math.pi / 2))


# Default strip pitch: 6.7 mm. At 13.3 mm a fold pressed by a fixture 3.7 mm
# from a hinge split over two hinges with a 13 mm flat between (a box fold). At 6.7 mm the worst split is a 6.7 mm
# flat, which reads as one line on video and `fold_lines` merges it. 42 bodies is still under a 40-60 link cable.
PITCH = 0.0067
CREASE_SET = math.radians(90.0)  # a hinge whose permanent set is at least this is a crease (a fold pressed well past 90)
SET_MIN = math.radians(10.0)     # sets below this are curl, not folds (fold_lines ignores them)


def elastic_range(peak: float, e0: float, springback: float, stop: float) -> float:
    """The hinge's elastic range after a largest bend `peak` (rad): e0 (free-bend yield, `Paper.elastic_range`) until
    the bend passes e0, then moving linearly toward the crease springback as the fold approaches the stop (pressed
    flat). Sharper folds keep more of their angle, as real creases do."""
    a = abs(peak)
    if a <= e0 or stop <= e0:
        return e0
    f = min(1.0, (a - e0) / (stop - e0))
    return e0 + (springback - e0) * f


def crease_update(mem: dict, q: float, e0: float, springback: float, stop: float) -> dict:
    """One control step of a hinge's plastic memory (elastic-perfectly-plastic, both directions). `mem` holds "rest"
    (the permanent set, rad, signed), "peak" (the signed largest bend so far) and "creased" (|rest| >= CREASE_SET at
    any time this trial). Returns mem."""
    rest = float(mem.get("rest", 0.0))
    if abs(q) > abs(mem.get("peak", 0.0)):
        mem["peak"] = float(q)
    e = elastic_range(mem.get("peak", 0.0), e0, springback, stop)
    if q - rest > e:
        rest = q - e
    elif rest - q > e:
        rest = q + e
    mem["rest"] = float(rest)
    mem["creased"] = bool(mem.get("creased", False) or abs(rest) >= CREASE_SET)
    return mem


@dataclass(frozen=True)
class Crease:
    """A yamkit law for one hinge: an implicit spring of stiffness `k` (N*m/rad) and damping `c` (N*m*s/rad) toward
    the hinge's permanent set, which `crease_update` moves when the hinge is bent past its elastic range."""

    k: float
    c: float
    e0: float
    springback: float
    stop: float

    def fresh(self, lk) -> dict:
        return {"peak": 0.0, "creased": False, "rest": 0.0}

    def drive(self, key, q, qd, jq, mem):
        crease_update(mem, q, self.e0, self.springback, self.stop)
        return mem["rest"], self.k, 0.0, self.c


WHITE = kit.Look((0.95, 0.95, 0.93), 0.85)
INK = kit.Look((0.20, 0.20, 0.22), 0.7)


@dataclass(frozen=True)
class Perforation:
    """A perforated edge: ties along the strips' `edge` ("-x" or "+x", the sheet's long side) holding them to the
    world (the part the sheet is torn from, fixed), `ties_per_strip` per strip. `land` is the uncut fraction of the
    perforation (a "3 x 1" rule: 1 mm tie per 4 mm, 0.25), so each tie carries tensile strength x land x its share of
    the edge: the perforation's strength per meter is the same at any pitch. `strips`: which are tied (None = all)."""

    edge: str = "-x"
    ties_per_strip: int = 1
    land: float = 0.25
    strips: tuple | None = None


@dataclass(frozen=True)
class Sheet:
    """A sheet `width` (along the hinge axes, the base's x) by `length` (along y), lying flat with its mid-plane
    center at `pos`. `hinges` are the hinge lines' distances (m) from the sheet's -y edge, strictly increasing
    (`Sheet.uniform` spaces them evenly; a hand-placed tuple is only for a sheet that really is scored there).
    `collider_t` is each strip's box thickness; `fold_limit` caps each hinge (rad; None: `fold_stop(j)`).
    `omega_dt` and `physics_dt` set each hinge's armature; `max_joint_speed` (rad/s) is PhysX's joint velocity clamp.
    `lines`: printed text lines on the top face (printed paper reads as printed on video). `perforation`: tear ties."""

    name: str
    width: float
    length: float
    hinges: tuple
    paper: Paper
    pos: tuple
    collider_t: float = 0.001
    fold_limit: float | None = None
    omega_dt: float = 1.0
    physics_dt: float = PHYSICS_DT
    max_joint_speed: float = 40.0
    gap: float = 0.0004
    lines: bool = True
    line_pitch: float = 0.006
    margin: float = 0.025
    look: kit.Look = field(default=WHITE)
    perforation: Perforation | None = None

    @classmethod
    def uniform(cls, name: str, width: float, length: float, paper: Paper, pos, pitch: float = PITCH, **kw) -> "Sheet":
        """A sheet with a hinge every `pitch` (rounded so the strips are equal and span the length): no fold line is
        favored, and a fold anywhere lands within pitch / 2 of a hinge."""
        n = max(1, int(round(length / pitch)))
        if n > MAX_LINKS:
            raise ValueError(f"{name}: {n} strips is past PhysX's {MAX_LINKS} links per articulation")
        return cls(name, width, length, tuple(length * k / n for k in range(1, n)), paper, pos, **kw)

    # -- geometry ---------------------------------------------------------------------------------------------------

    @property
    def edges(self) -> np.ndarray:
        """The strips' boundaries from the -y edge: 0, the hinges, length."""
        return np.array([0.0, *self.hinges, self.length], float)

    @property
    def n(self) -> int:
        return len(self.hinges) + 1

    @property
    def widths(self) -> np.ndarray:
        return np.diff(self.edges)

    @property
    def root(self) -> int:
        """The strip holding the sheet's middle (the articulation's root)."""
        e = self.edges
        return int(np.searchsorted(e, self.length / 2, side="right") - 1)

    def depth(self, i: int) -> int:
        """Hinges between strip i and the root."""
        return abs(i - self.root)

    @property
    def max_depth(self) -> int:
        return max(self.depth(i) for i in range(self.n))

    def parent(self, i: int) -> int | None:
        r = self.root
        return None if i == r else (i - 1 if i > r else i + 1)

    def hinge_of(self, i: int) -> int:
        """The hinge (index into `hinges`) strip i turns on (i != root): hinge j joins strips j and j + 1."""
        return i - 1 if i > self.root else i

    def strip_of_hinge(self, j: int) -> int:
        """The strip that turns on hinge j (the one farther from the root)."""
        return j + 1 if j >= self.root else j

    def side(self, i: int) -> int:
        """+1 for strips on the +y side of the root, -1 on the -y side (0 for the root). A hinge angle times the side
        of its strip is positive when the sheet folds up (top faces closing, a valley fold seen from above)."""
        return int(np.sign(i - self.root))

    def anchor(self, i: int) -> tuple:
        """Strip i's hinge in the base frame with the sheet flat (the base origin is the sheet's center)."""
        e, L = self.edges, self.length
        return (0.0, float((e[i] if i > self.root else e[i + 1]) - L / 2), 0.0)

    def box_local(self, i: int) -> tuple:
        """(center, size) of strip i's collider in its own frame (the base frame for the root, else the frame at its
        hinge). Each strip is `gap` short at a hinge, so neighbors don't overlap when flat."""
        e, L, w, t = self.edges, self.length, self.widths[i], self.collider_t
        g = self.gap / 2
        if i == self.root:
            lo, hi = e[i] - L / 2 + (g if i > 0 else 0.0), e[i + 1] - L / 2 - (g if i < self.n - 1 else 0.0)
        elif i > self.root:
            lo, hi = g, w - (g if i < self.n - 1 else 0.0)
        else:
            lo, hi = -w + (g if i > 0 else 0.0), -g
        return (0.0, float((lo + hi) / 2), 0.0), (self.width, float(hi - lo), t)

    def mass(self, i: int) -> float:
        return float(self.paper.areal_mass * self.width * self.widths[i])

    @property
    def total_mass(self) -> float:
        return float(sum(self.mass(i) for i in range(self.n)))

    def stiffness(self, j: int) -> float:
        """Hinge j's spring, k = D W / l (N*m/rad), l the distance between the two strips' centers."""
        w = self.widths
        return float(self.paper.bending_stiffness * self.width / ((w[j] + w[j + 1]) / 2))

    def inertia(self, i: int) -> float:
        """Strip i's moment of inertia about its hinge edge (m l^2 / 3, kg*m^2)."""
        return float(self.mass(i) * self.widths[i] ** 2 / 3)

    def armature(self, j: int) -> float:
        """Hinge j's joint armature (kg*m^2): k (dt / omega_dt)^2, so that sqrt(k / armature) * dt = omega_dt. Joint
        armature is extra joint-space inertia (no mass, no weight: the sheet's drape is unchanged); NVIDIA's
        articulation stability guide: it "can help with stability by adding joint-space inertia"."""
        return float(self.stiffness(j) * (self.physics_dt / self.omega_dt) ** 2)

    def damping(self, j: int) -> float:
        """Hinge j's damping, 2 zeta sqrt(k (I + A)), I the turning strip's inertia about the hinge, A the armature."""
        i = self.strip_of_hinge(j)
        return float(2 * self.paper.damping_ratio * math.sqrt(self.stiffness(j) * (self.inertia(i) + self.armature(j))))

    def fold_stop(self, j: int) -> float:
        """Hinge j's stop (rad): folded this far at one hinge, the strips on either side of the two that meet there
        (which start a strip's width from the hinge) are a collider thickness apart: pi - asin(t / w), w the shorter
        of the two strips, capped at 177 degrees. Sharper and those non-adjacent strips would start inside each other
        (unintended self-collision, an instability the stability guide names)."""
        if self.fold_limit is not None:
            return float(self.fold_limit)
        w = min(self.widths[j], self.widths[j + 1])
        return float(min(math.radians(177.0), math.pi - math.asin(min(1.0, self.collider_t / w))))

    def link_name(self, i: int) -> str:
        return f"s{i}"

    def key(self, i: int) -> str:
        """objects[] key of strip i."""
        return self.name if i == self.root else f"{self.name}/s{i}"


    def crease_law(self, j: int) -> Crease:
        """Hinge j's elastic-plastic law (see crease_update)."""
        e0 = self.paper.elastic_range(float((self.widths[j] + self.widths[j + 1]) / 2))
        return Crease(self.stiffness(j), self.damping(j), e0, self.paper.springback, self.fold_stop(j))

    def hinge_y(self, j: int) -> float:
        """Hinge j's position along the sheet from its -y edge (m): sheet coordinates of fold lines."""
        return float(self.hinges[j])

    def ties(self) -> list:
        """The perforation's ties: (link name, point in the link's frame, world-anchored break force N), [] if none."""
        p = self.perforation
        if p is None:
            return []
        out = []
        sx = -1.0 if p.edge == "-x" else 1.0
        for i in (range(self.n) if p.strips is None else p.strips):
            c, s = self.box_local(i)
            force = self.paper.tensile_strength * p.land * float(self.widths[i]) / p.ties_per_strip
            for k in range(p.ties_per_strip):
                u = (k + 0.5) / p.ties_per_strip - 0.5
                pt = (sx * self.width / 2, float(c[1] + u * s[1]), 0.0)
                out.append(("base" if i == self.root else self.link_name(i), pt, float(force)))
        return out

    def tie_points_flat(self) -> np.ndarray:
        """World points of every tie with the sheet where it starts (the anchors the ties hold to)."""
        fr = self.forward(np.zeros(self.n - 1))
        names = {("base" if i == self.root else self.link_name(i)): i for i in range(self.n)}
        return np.array([fr[names[ln]][0] + fr[names[ln]][1] @ np.asarray(pt) for ln, pt, _ in self.ties()]
                        ).reshape(-1, 3)

    # -- the yamkit mechanism -------------------------------------------------------------------------------------

    def _meshes(self, i: int) -> tuple:
        c, s = self.box_local(i)
        # the visible face spans the hinge gaps (visual only; the collider keeps them), so no seams show
        vs = (s[0], s[1] + (self.gap if 0 < i < self.n - 1 else self.gap / 2), s[2])
        vc = (c[0], c[1] + (0.0 if 0 < i < self.n - 1 else (self.gap / 4 if i == 0 else -self.gap / 4)), c[2])
        out = [kit.box(vc, vs, self.look)]
        if self.lines:
            y0, y1 = c[1] - s[1] / 2, c[1] + s[1] / 2
            ys = np.arange(self.margin, self.length - self.margin, self.line_pitch)  # sheet coordinates
            e = self.edges
            off = self.length / 2 if i == self.root else (e[i] if i > self.root else e[i + 1])
            for k, yy in enumerate(ys):
                ly = yy - off
                if y0 + 0.001 < ly < y1 - 0.001:
                    span = self.width - 2 * self.margin - (0.04 if k % 5 == 4 else 0.0)
                    out.append(kit.box((-(self.width - 2 * self.margin - span) / 2, ly, s[2] / 2 + 0.00005),
                                       (span, 0.0012, 0.0001), INK))
        return tuple(out)

    def mechanism(self) -> kit.Mechanism:
        """The sheet as one floating yamkit Mechanism (the root strip is its base)."""
        r = self.root
        links = []
        for i in sorted(range(self.n), key=self.depth):
            if i == r:
                continue
            j = self.hinge_of(i)
            c, s = self.box_local(i)
            p = self.parent(i)
            lim = self.fold_stop(j)
            links.append(kit.Link(
                self.link_name(i), "revolute", "X", anchor=self.anchor(i), lower=-lim, upper=lim, start=0.0,
                mass=self.mass(i), meshes=self._meshes(i), colliders=(kit.Collider(c, size=s),), damping=0.0,
                law=self.crease_law(j),
                parent=None if p == r else self.link_name(p)))
        if self.n > MAX_LINKS:
            raise ValueError(f"{self.name}: {self.n} strips is past PhysX's {MAX_LINKS} links per articulation")
        c, s = self.box_local(r)
        kit._SHEET_NAMES.add(self.name)
        kit._SHEET_PAPER[self.name] = self.paper
        kit._SHEET_JOINTS[self.name] = {self.link_name(self.strip_of_hinge(j)): self.armature(j)
                                        for j in range(self.n - 1)}
        kit._SHEET_SPEED[self.name] = self.max_joint_speed
        kit._SHEET_TIE_MASS[self.name] = float(min(self.mass(i) for i in range(self.n)))
        kit._SHEET_TIES[self.name] = [(*t, tuple(float(v) for v in a)) for t, a in zip(self.ties(),
                                                                                         self.tie_points_flat())]
        return kit.Mechanism(self.name, pos=tuple(float(v) for v in self.pos), size=(self.width, self.length,
                             self.collider_t), mass=self.mass(r), meshes=self._meshes(r),
                             colliders=(kit.Collider(c, size=s),), links=tuple(links))

    # -- poses (pure numpy) ---------------------------------------------------------------------------------------

    def forward(self, q, base_pos=None, base_R=None) -> list:
        """Each strip's frame (pos, R) for hinge angles q (one per hinge, rad), with the root at base_pos/base_R
        (default: where the sheet starts, flat). Frames as the simulator reports them: the root's at the sheet center,
        every other strip's at its hinge."""
        q = np.asarray(q, float)
        P = np.asarray(self.pos if base_pos is None else base_pos, float)
        R = np.eye(3) if base_R is None else np.asarray(base_R, float)
        out = [None] * self.n
        out[self.root] = (P, R)
        for i in sorted(range(self.n), key=self.depth):
            if i == self.root:
                continue
            p = self.parent(i)
            pp, pr = out[p]
            rel = np.asarray(self.anchor(i)) - (np.zeros(3) if p == self.root else np.asarray(self.anchor(p)))
            out[i] = (pp + pr @ rel, pr @ rot_x(q[self.hinge_of(i)]))
        return out

    def corners(self, frames) -> np.ndarray:
        """All strips' collider corners in world, shape (n, 8, 3)."""
        return np.stack([strip_corners(self, i, *frames[i]) for i in range(self.n)])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def strip_corners(sheet: Sheet, i: int, pos, R) -> np.ndarray:
    c, s = sheet.box_local(i)
    signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], float)
    local = np.asarray(c, float) + signs * np.asarray(s, float) / 2
    return np.asarray(pos, float) + local @ np.asarray(R, float).T


def quat_R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_quat(R) -> tuple:
    """Rotation matrix to (x, y, z, w)."""
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    return (x, y, z, w)


# -- offline checks (pure numpy) ------------------------------------------------------------------------------------


def planar_model(sheet: Sheet, armature: bool = True) -> dict:
    """The sheet flat, as a planar chain in the y-z plane: generalized coordinates (root y, root z, root tilt, hinge
    angles), joint-space mass matrix H (strip masses, their inertia about their centers, and the hinge armature) and
    stiffness K (the hinge springs). Also each strip's center velocity Jacobian Jv (n x 2 x dof)."""
    n, r, wid, e = sheet.n, sheet.root, sheet.widths, sheet.edges
    m = np.array([sheet.mass(i) for i in range(n)])
    Ic = m * (wid ** 2 + sheet.collider_t ** 2) / 12
    dof = 3 + n - 1
    Jv, Jw = np.zeros((n, 2, dof)), np.zeros((n, dof))
    c = (e[:-1] + e[1:]) / 2  # strip centers along the sheet (flat: y = c - L/2, z = 0)
    for i in range(n):
        Jv[i, 0, 0], Jv[i, 1, 1], Jw[i, 2] = 1.0, 1.0, 1.0
        Jv[i, 1, 2] = c[i] - c[r]  # a root tilt lifts strip i by its lever arm
        # hinges between the root and strip i: turning hinge j lifts strip i's center by its distance from the hinge
        js = range(r, i) if i > r else range(i, r)
        for j in js:
            hinge_y = e[j + 1]
            Jw[i, 3 + j] = sheet.side(i) if i != r else 0.0
            Jv[i, 1, 3 + j] = (c[i] - hinge_y) * (1.0 if i > r else -1.0) * sheet.side(i)
    H = sum(m[i] * Jv[i].T @ Jv[i] + Ic[i] * np.outer(Jw[i], Jw[i]) for i in range(n))
    K = np.zeros((dof, dof))
    for j in range(n - 1):
        K[3 + j, 3 + j] = sheet.stiffness(j)
        if armature:
            H[3 + j, 3 + j] += sheet.armature(j)
    return {"H": H, "K": K, "Jv": Jv, "Jw": Jw, "m": m}


def stability(sheet: Sheet, armature: bool = True, control_dt: float = CONTROL_DT) -> dict:
    """Numbers the PhysX articulation guidance asks about, for the flat sheet:
    - omega_dt: the highest hinge mode's natural frequency (root free) times the physics step (want <= ~1);
    - inertia_ratio: largest / smallest eigenvalue of the hinge block of H (the joint-space inertia spread);
    - min_contact_mass: the smallest effective mass (kg) a normal push on a strip's center meets (root free);
    - slow_modes: the natural periods (s) longer than two control steps (what a 20 Hz scorer can see, Nyquist), which
      the armature must leave almost alone;
    - slow_shift: the largest relative change of those periods against the same sheet without armature."""
    out = {}
    for arm in ((False, True) if armature else (False,)):
        M = planar_model(sheet, arm)
        H, K, Jv = M["H"], M["K"], M["Jv"]
        w2 = np.linalg.eigvals(np.linalg.solve(H, K)).real
        w = np.sqrt(np.sort(w2[w2 > 1e-9]))
        hb = np.linalg.eigvalsh(H[3:, 3:])
        Hi = np.linalg.inv(H)
        meff = [1.0 / float(Jv[i, 1] @ Hi @ Jv[i, 1]) for i in range(sheet.n)]
        out[arm] = {"omega_dt": float(w.max() * sheet.physics_dt), "inertia_ratio": float(hb.max() / hb.min()),
                    "min_contact_mass": float(min(meff)), "periods": 2 * math.pi / w}
    res = dict(out[armature])
    base = out[False]["periods"]
    slow = base[base > 2 * control_dt]
    res["slow_modes"] = slow
    if armature:
        cur = out[True]["periods"]
        res["slow_shift"] = float(np.max(np.abs(cur[:len(slow)] - slow) / slow)) if len(slow) else 0.0
    else:
        res["slow_shift"] = 0.0
    del res["periods"]
    return res


def cantilever(sheet: Sheet, overhang: float, iters: int = 2000) -> tuple:
    """The sheet lying on a table with its +y edge pushed `overhang` m past the table's edge, at rest under gravity:
    the strip straddling the edge stays flat (the table and the sheet behind hold it), and each hinge past the edge
    bends until its spring carries the weight hanging beyond it. Returns (hinge indices past the edge, their angles in
    rad (drooping positive, relative), tip drop below the table top in m, chord angle from the edge to the tip in
    degrees: Peirce's theta)."""
    e = sheet.edges
    edge_y = sheet.length - overhang
    hinges = [j for j in range(sheet.n - 1) if e[j + 1] > edge_y + 1e-12]
    cuts = [edge_y] + [e[j + 1] for j in hinges] + [sheet.length]  # segment k runs cuts[k]..cuts[k+1]
    lens = np.diff(cuts)
    w = sheet.paper.areal_mass * sheet.width * G  # weight per m of sheet length, N/m
    k = np.array([sheet.stiffness(j) for j in hinges])
    th = np.zeros(len(hinges))
    for _ in range(iters):
        ang = np.concatenate([[0.0], np.cumsum(th)])  # segment k's droop angle
        y = np.concatenate([[0.0], np.cumsum(lens * np.cos(ang))])  # horizontal positions of the cut points
        mid = y[:-1] + lens / 2 * np.cos(ang)
        new = np.array([np.sum(w * lens[i + 1:] * (mid[i + 1:] - y[i + 1])) for i in range(len(hinges))]) / k
        new = np.minimum(new, math.pi / 2)
        if np.max(np.abs(new - th), initial=0.0) < 1e-12:
            th = new
            break
        th = 0.7 * th + 0.3 * new
    ang = np.concatenate([[0.0], np.cumsum(th)])
    tip_y = float(np.sum(lens * np.cos(ang)))
    drop = float(np.sum(lens * np.sin(ang)))
    return hinges, th, drop, math.degrees(math.atan2(drop, tip_y))


def peirce_chord(overhang: float, bending_length: float) -> float:
    """Peirce's heavy-cantilever chord angle (degrees) for a strip of bending length c pushed out `overhang` flat:
    c = l (cos(theta / 2) / (8 tan theta))^(1/3) (Peirce 1930; ASTM D1388 uses theta = 41.5 deg at l = 2c)."""
    target = (bending_length / overhang) ** 3
    lo, hi = 1e-6, math.radians(89.9)
    for _ in range(100):
        mid = (lo + hi) / 2
        if math.cos(mid / 2) / (8 * math.tan(mid)) > target:
            lo = mid
        else:
            hi = mid
    return math.degrees((lo + hi) / 2)


def slide_press_min(paper: Paper, sheet_mass: float, pad=(1.0, 0.8), table=(0.5, 0.5), pad_combine="max") -> float:
    """The smallest normal force (N) a fingertip pressing on the sheet needs to drag the whole sheet over the table
    without slipping on it: mu_pad N >= mu_table (N + m g), with PhysX's combine rules (pad "max", table "average").
    Any larger press also drags it: friction alone sets no upper limit (stability does)."""
    comb = max if pad_combine == "max" else (lambda a, b: (a + b) / 2)
    mu_pad = comb(pad[0], paper.friction)  # the pad grips the paper (static) while the sheet slides on the table
    mu_tab_s = (table[0] + paper.friction) / 2
    if mu_pad <= mu_tab_s:
        return float("inf")
    return float(mu_tab_s * sheet_mass * G / (mu_pad - mu_tab_s))


def fold_clearance(sheet: Sheet, j: int, angle: float | None = None) -> float:
    """With hinge j alone folded to `angle` (default its stop, upward), the smallest distance (m) between the
    mid-planes of the strips beyond the fold (j - 1 and j + 2, which do collide) wherever they overlap; inf if
    they don't overlap. Want >= collider_t."""
    q = np.zeros(sheet.n - 1)
    a = sheet.fold_stop(j) if angle is None else angle
    q[j] = a * sheet.side(sheet.strip_of_hinge(j))
    fr = sheet.forward(q, base_pos=(0.0, 0.0, 0.0))
    if j - 1 < 0 or j + 2 >= sheet.n:
        return float("inf")
    pa, Ra = fr[j - 1]
    pb, Rb = fr[j + 2]
    ca, sa = sheet.box_local(j - 1)
    cb, sb = sheet.box_local(j + 2)
    na = Ra[:, 2]
    best = float("inf")
    for u in np.linspace(-0.5, 0.5, 41):
        pt = pb + Rb @ (np.asarray(cb) + np.array([0.0, u * sb[1], 0.0]))
        loc = Ra.T @ (pt - pa) - np.asarray(ca)
        if abs(loc[1]) <= sa[1] / 2:
            best = min(best, abs(float((pt - pa - Ra @ np.asarray(ca)) @ na)))
    return best




# -- readouts (pure numpy on strip frames; the simulator wrappers are `frames` / `state`) -----------------------------


@dataclass
class SheetState:
    """One control step of a sheet: each strip's (pos, R), each hinge's angle q (rad) and permanent set (rad, signed),
    and whether each hinge has ever held a crease this trial."""

    frames: list
    q: np.ndarray
    sets: np.ndarray
    creased: np.ndarray


def frames(objects, sheet: Sheet) -> list:
    """Each strip's (pos, R) from the simulator's object states."""
    return [(np.asarray(objects[sheet.key(i)].pos, float), quat_R(objects[sheet.key(i)].quat_xyzw))
            for i in range(sheet.n)]


def hinge_angles_from_frames(sheet: Sheet, fr) -> np.ndarray:
    """Each hinge's angle (rad) from the strips' frames: the turn about the hinge axis from the parent strip to the
    child (the same sign convention as the joint)."""
    out = np.zeros(sheet.n - 1)
    for j in range(sheet.n - 1):
        i = sheet.strip_of_hinge(j)
        Rp, Rc = fr[sheet.parent(i)][1], fr[i][1]
        Rr = np.asarray(Rp).T @ np.asarray(Rc)
        out[j] = math.atan2(Rr[2, 1], Rr[1, 1])
    return out


def hinge_angles(sheet: Sheet) -> np.ndarray:
    """Each hinge's angle (rad) at this control step (simulator)."""
    return np.array([kit.joint(None, f"{sheet.name}/s{sheet.strip_of_hinge(j)}").pos for j in range(sheet.n - 1)])


def sets_now(sheet: Sheet) -> np.ndarray:
    """Each hinge's permanent set (rad, signed) this trial (simulator: the crease laws' memory)."""
    return np.array([float(kit.law_state(f"{sheet.name}/s{sheet.strip_of_hinge(j)}").get("rest", 0.0))
                     for j in range(sheet.n - 1)])


def creased(sheet: Sheet) -> np.ndarray:
    """Each hinge's crease flag this trial (|set| reached CREASE_SET at some time)."""
    return np.array([bool(kit.law_state(f"{sheet.name}/s{sheet.strip_of_hinge(j)}").get("creased", False))
                     for j in range(sheet.n - 1)])


def state(objects, sheet: Sheet) -> SheetState:
    """The sheet's state at this control step (simulator)."""
    return SheetState(frames(objects, sheet), hinge_angles(sheet), sets_now(sheet), creased(sheet))


def offline_state(sheet: Sheet, q, sets=None, base_pos=None, base_R=None) -> SheetState:
    """A state built from hinge angles (offline tests and replicas). `sets` default: what crease_update leaves after
    bending each hinge straight to q from flat."""
    q = np.asarray(q, float)
    if sets is None:
        sets = np.zeros_like(q)
        for j in range(len(q)):
            law = sheet.crease_law(j)
            sets[j] = crease_update({}, float(q[j]), law.e0, law.springback, law.stop)["rest"]
    sets = np.asarray(sets, float)
    return SheetState(sheet.forward(q, base_pos, base_R), q, sets, np.abs(sets) >= CREASE_SET)


@dataclass(frozen=True)
class FoldLine:
    """A fold: consecutive hinges with a set of one sign. `pos` m from the sheet's -y edge (set-weighted), `angle`
    the summed set (rad, signed: + = valley seen from the sheet's top face), `hinges` their indices, `width` m from
    the first to the last."""

    pos: float
    angle: float
    hinges: tuple
    width: float

    @property
    def sharp(self) -> bool:
        """A crease, not a roll: at least CREASE_SET of set within two hinges."""
        return abs(self.angle) >= CREASE_SET and len(self.hinges) <= 2

    @property
    def sharpness(self) -> float:
        """0..1: the fold's set in degrees per hinge it spans, over 180 (a flat-pressed one-hinge crease is ~0.8)."""
        return float(min(1.0, abs(math.degrees(self.angle)) / len(self.hinges) / 180.0))


def fold_lines(sheet: Sheet, sets, min_set: float = SET_MIN) -> list:
    """The sheet's fold lines from its hinges' permanent sets (raw joint signs, as `state().sets`), in sheet
    coordinates, valley-positive (`FoldLine.angle` > 0: the top faces close)."""
    raw = np.asarray(sets, float)
    # valley-positive: a set times its strip's side is + when the top faces close (see Sheet.side)
    sets = raw * np.array([sheet.side(sheet.strip_of_hinge(j)) for j in range(len(raw))], float)
    out, cur = [], []

    def close():
        if cur:
            w = np.abs(sets[cur])
            ys = np.array([sheet.hinge_y(j) for j in cur])
            out.append(FoldLine(float(np.sum(w * ys) / np.sum(w)), float(np.sum(sets[cur])), tuple(cur),
                                float(ys[-1] - ys[0])))

    for j in range(len(sets)):
        if abs(sets[j]) >= min_set and (not cur or np.sign(sets[j]) == np.sign(sets[cur[-1]])):
            cur.append(j)
        else:
            close()
            cur = [j] if abs(sets[j]) >= min_set else []
    close()
    return out


def folds_match(sheet: Sheet, sets, targets, tol: float = 0.015) -> tuple:
    """Match fold lines to wanted ones. `targets`: [(pos m from the -y edge, sign +1/-1/0 for either)]. Returns
    (sharp folds matched, stray folds: any fold line, sharp or not, that matches no target)."""
    lines = fold_lines(sheet, sets)
    used, matched = set(), 0
    for pos, sign in targets:
        best = None
        for k, ln in enumerate(lines):
            if k in used or abs(ln.pos - pos) > tol or (sign and np.sign(ln.angle) != sign):
                continue
            if best is None or abs(ln.pos - pos) < abs(lines[best].pos - pos):
                best = k
        if best is not None:
            used.add(best)
            matched += int(lines[best].sharp)
    return matched, len(lines) - len(used)


def crumpled(sheet: Sheet, sets, expected: int) -> bool:
    """Anti-gaming: a sheet with more fold lines than the task asks for (+1 tolerated), or with more total set than
    `expected` flat folds could hold, is crumpled, not folded."""
    lines = fold_lines(sheet, sets)
    total = float(np.sum(np.abs(np.asarray(sets, float))))
    return len(lines) > expected + 1 or total > (expected + 0.5) * math.pi


def _strip_hit(sheet: Sheet, i: int, pos, R, xy):
    """Where the vertical line through table point xy meets strip i's mid-plane: (z, inside) or None."""
    c, s = sheet.box_local(i)
    R = np.asarray(R, float)
    n = R[:, 2]
    if abs(n[2]) < 0.2:  # strip nearly on edge: it covers no area seen from above
        return None
    o = np.asarray(pos, float) + R @ np.asarray(c, float)
    z = o[2] - (n[0] * (xy[0] - o[0]) + n[1] * (xy[1] - o[1])) / n[2]
    loc = R.T @ (np.array([xy[0], xy[1], z]) - o)
    return z, bool(abs(loc[0]) <= s[0] / 2 and abs(loc[1]) <= s[1] / 2 + sheet.gap / 2)  # gaps count as sheet


def layer_count(sheet: Sheet, fr, xy) -> int:
    """How many sheet layers lie over table point xy (strips whose face covers it, seen from above)."""
    zs = []
    for i in range(sheet.n):
        h = _strip_hit(sheet, i, fr[i][0], fr[i][1], xy)
        if h and h[1]:
            zs.append(h[0])
    # neighbors meeting over a hinge are one layer: merge hits closer than half a collider thickness
    zs = sorted(zs)
    return sum(1 for k, z in enumerate(zs) if k == 0 or z - zs[k - 1] > sheet.collider_t / 2)


def coverage(sheet: Sheet, fr, rect, n: int = 12) -> float:
    """Fraction of rectangle (x0, x1, y0, y1) the sheet covers (n x n sample points, any number of layers)."""
    x0, x1, y0, y1 = rect
    xs = x0 + (np.arange(n) + 0.5) / n * (x1 - x0)
    ys = y0 + (np.arange(n) + 0.5) / n * (y1 - y0)
    return float(np.mean([layer_count(sheet, fr, (x, y)) > 0 for x in xs for y in ys]))


def inside_box(sheet: Sheet, fr, center, R, inner, margin: float = 0.0) -> float:
    """Fraction of the sheet's strip corners inside an oriented box (a pocket, an envelope, a sleeve): `center` world,
    `R` its rotation, `inner` its inside size (m), shrunk by `margin` on every side."""
    pts = sheet.corners(fr).reshape(-1, 3)
    loc = (pts - np.asarray(center, float)) @ np.asarray(R, float)
    half = np.asarray(inner, float) / 2 - margin
    return float(np.mean(np.all(np.abs(loc) <= half, axis=1)))


def tear_length(sheet: Sheet, fr, anchors=None, tol: float = 0.002) -> float:
    """Length of perforation torn (m): ties whose strip point has left its anchor by more than `tol`, times the tie
    spacing. `anchors` default: the tie points with the sheet where it starts."""
    ties = sheet.ties()
    if not ties:
        return 0.0
    anchors = sheet.tie_points_flat() if anchors is None else np.asarray(anchors, float)
    names = {("base" if i == sheet.root else sheet.link_name(i)): i for i in range(sheet.n)}
    spacing = sheet.widths / sheet.perforation.ties_per_strip
    torn = 0.0
    for (ln, pt, _), a in zip(ties, anchors):
        i = names[ln]
        p = fr[i][0] + np.asarray(fr[i][1]) @ np.asarray(pt)
        if np.linalg.norm(p - a) > tol:
            torn += float(spacing[i])
    return torn


def sheet_com(sheet: Sheet, fr) -> np.ndarray:
    """The sheet's center of mass (world)."""
    m = np.array([sheet.mass(i) for i in range(sheet.n)])
    cs = [np.asarray(fr[i][0]) + np.asarray(fr[i][1]) @ np.asarray(sheet.box_local(i)[0]) for i in range(sheet.n)]
    return np.sum(m[:, None] * np.array(cs), axis=0) / m.sum()


def distance_to_sheet(sheet: Sheet, fr, p) -> float:
    """Distance (m) from point p to the nearest strip box."""
    best = float("inf")
    for i in range(sheet.n):
        c, s = sheet.box_local(i)
        loc = np.asarray(fr[i][1]).T @ (np.asarray(p, float) - np.asarray(fr[i][0])) - np.asarray(c)
        d = np.maximum(np.abs(loc) - np.asarray(s) / 2, 0.0)
        best = min(best, float(np.linalg.norm(d)))
    return best


class Monitor:
    """Anti-gaming watch over a trial, fed once per control step: NaN, hinges driven past their stop (a blown
    solver), the sheet thrown (center of mass faster than `throw_speed` while no gripper is within `near` of it, or
    in free flight: every strip above `table_z + air` for longer than `air_s` with no gripper near)."""

    def __init__(self, sheet: Sheet, table_z: float, throw_speed: float = 1.0, near: float = 0.04,
                 air: float = 0.02, air_s: float = 0.3, dt: float = CONTROL_DT):
        self.sheet, self.table_z = sheet, table_z
        self.throw_speed, self.near, self.air, self.air_s, self.dt = throw_speed, near, air, air_s, dt
        self.reset()

    def reset(self) -> None:
        self.prev, self.air_t = None, 0.0
        self.nan = self.overrun = self.thrown = False
        self.peak_speed = 0.0

    def update(self, st: SheetState, grippers=()) -> None:
        pts = self.sheet.corners(st.frames).reshape(-1, 3)
        if not np.all(np.isfinite(pts)) or not np.all(np.isfinite(st.q)):
            self.nan = True
            return
        stops = np.array([self.sheet.fold_stop(j) for j in range(self.sheet.n - 1)])
        self.overrun |= bool(np.any(np.abs(st.q) > stops + math.radians(3.0)))
        com = sheet_com(self.sheet, st.frames)
        held = any(distance_to_sheet(self.sheet, st.frames, g) <= self.near for g in grippers)
        if self.prev is not None:
            v = float(np.linalg.norm(com - self.prev) / self.dt)
            self.peak_speed = max(self.peak_speed, v)
            if v > self.throw_speed and not held:
                self.thrown = True
        self.prev = com
        if pts[:, 2].min() > self.table_z + self.air and not held:
            self.air_t += self.dt
            self.thrown |= self.air_t > self.air_s
        else:
            self.air_t = 0.0

    @property
    def void(self) -> bool:
        """The trial scores 0: NaN, a blown hinge, or a throw."""
        return self.nan or self.overrun or self.thrown


def start_satisfies(rule, st: SheetState) -> bool:
    """Anti-gaming: True if the start state already meets an end rule (the scene would hand out free credit)."""
    return bool(rule(st))


def tier(value: float, table, higher_is_better: bool = True) -> float:
    """Partial credit from a [(threshold, credit)] table: the best credit whose threshold `value` meets."""
    best = 0.0
    for thr, cr in table:
        if (value >= thr) if higher_is_better else (value <= thr):
            best = max(best, cr)
    return best


# -- the offline replica of a fixture fold (quasi-static) -----------------------------------------------------


def replica_fold(sheet: Sheet, fold_y: float, panel: str = "+y") -> dict:
    """What a flat sheet folded over a line `fold_y` (m from its -y edge) by a flap pressed flat should end as:
    the crease forms on the hinge nearest the line (the strip across the line is rigid, so the pressed U collapses
    onto the nearer hinge); it reaches the fold stop; on release it keeps stop - springback of set, and the top panel
    (`panel` side, lying on the bottom one) opens until the hinge spring balances the panel's weight (the panel taken
    as rigid). Returns the crease hinge, its position, set, opening angle and the layers over two test points."""
    hy = np.asarray(sheet.hinges, float)
    j = int(np.argmin(np.abs(hy - fold_y)))
    law = sheet.crease_law(j)
    mem = {}
    for a in np.linspace(0.0, law.stop, 60):  # the flap's sweep, bending only hinge j
        crease_update(mem, float(a), law.e0, law.springback, law.stop)
    rest = mem["rest"]
    k = sheet.stiffness(j)
    top_len = (sheet.length - hy[j]) if panel == "+y" else hy[j]
    m_top = sheet.paper.areal_mass * sheet.width * top_len
    th = law.stop
    for _ in range(4000):  # k (th - rest) = m g (L/2) cos(pi - th): the opening where spring and weight balance
        f = k * (th - rest) - m_top * G * top_len / 2 * math.cos(math.pi - th)
        if abs(f) < 1e-9:
            break
        th = min(law.stop, th - 0.2 * f / k)
    q = np.zeros(sheet.n - 1)
    side = sheet.side(sheet.strip_of_hinge(j)) if sheet.strip_of_hinge(j) != sheet.root else 1
    q[j] = side * th * (1 if panel == "+y" else 1)
    sets = np.zeros(sheet.n - 1)
    sets[j] = side * rest
    lines = fold_lines(sheet, sets)
    return {"hinge": j, "hinge_y": float(hy[j]), "offset": float(hy[j] - fold_y), "peak_deg": math.degrees(law.stop),
            "set_deg": math.degrees(rest), "springback_deg": math.degrees(law.stop - rest),
            "released_fold_deg": math.degrees(th), "opening_deg": math.degrees(math.pi - th),
            "e0_deg": math.degrees(law.e0), "k": k, "top_mass_g": m_top * 1000,
            "lines": lines, "sharp": bool(lines and lines[0].sharp)}


# -- turn on self-collision for sheet articulations (yamkit authors every mechanism without it) -------------------
if not getattr(kit, "_sheetkit_patched", False):
    kit._SHEET_NAMES = set()
    kit._SHEET_JOINTS = {}
    kit._SHEET_SPEED = {}
    _plain_mech_usda = kit._mech_usda

    # A pad pressed deep into a 1 mm strip can throw light links apart. Contacts that start 2 mm out and rest at zero
    # separation (as yam_arena gives its props, 5 mm there) let the solver push back before a pad is inside a strip,
    # and a 0.3 m/s depenetration cap keeps a strip that is penetrated anyway from being fired out of it.
    SHEET_CONTACT_OFFSET = 0.002
    SHEET_MAX_DEPENETRATION = 0.3

    def _mech_usda(mech):
        text = _plain_mech_usda(mech)
        if mech.name in kit._SHEET_NAMES:
            text = text.replace("physxArticulation:enabledSelfCollisions = 0",
                                "physxArticulation:enabledSelfCollisions = 1")
            text = text.replace('prepend apiSchemas = ["PhysicsCollisionAPI"]',
                                'prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]')
            text = text.replace('uniform token purpose = "guide"\n',
                                'uniform token purpose = "guide"\n'
                                f"            float physxCollision:contactOffset = {SHEET_CONTACT_OFFSET}\n"
                                "            float physxCollision:restOffset = 0\n")
            text = text.replace("bool physxRigidBody:disableGravity = 0\n",
                                "bool physxRigidBody:disableGravity = 0\n"
                                f"        float physxRigidBody:maxDepenetrationVelocity = {SHEET_MAX_DEPENETRATION}\n")
            text = _joint_extras(text, kit._SHEET_JOINTS.get(mech.name, {}), kit._SHEET_SPEED.get(mech.name))
        return text

    def _joint_extras(text: str, armature: dict, speed) -> str:
        """Author PhysxJointAPI's armature (kg*m^2) and maxJointVelocity (deg/s for a hinge) on each sheet joint.
        Isaac Lab keeps both: yamkit's ImplicitActuatorCfg leaves armature and the velocity limit None, which reads
        them from the USD joint prim."""
        out = []
        for block in text.split('    def PhysicsRevoluteJoint "'):
            name = block.split('_joint"', 1)[0] if out else None
            if name in armature:
                extra = f"        float physxJoint:armature = {armature[name]:.6g}\n"
                if speed:
                    extra += f"        float physxJoint:maxJointVelocity = {math.degrees(speed):.6g}\n"
                block = block.replace("        float physxJoint:jointFriction", extra + "        float physxJoint:jointFriction",
                                      1)
            out.append(block)
        return '    def PhysicsRevoluteJoint "'.join(out)

    kit._mech_usda = _mech_usda

    # Friction. yamkit gives every mechanism the arena's rubber-like 1.2 / 1.0 (meant for objects the pads grip) and
    # the pads and the table both have the scene default 0.5 / 0.5, so a pad pressing on a sheet could never drag it
    # over the table: pad-on-sheet and sheet-on-table friction came out equal (so a pad could not drag a sheet).
    # A sheet gets its paper's own friction, and the fingertip links get rubber pads that win any pairing ("max"):
    # 1.0 static, 0.8 kinetic, within the range for gripper pads and a fingertip on paper (0.6 to 1.2;
    # Derler and Gerhardt 2012 for skin, 0.8 to 1.2 for rubber on paper).
    PAD_FRICTION = (1.0, 0.8)
    PAD_LINKS = ("tip_left", "tip_right")
    kit._SHEET_PAPER = {}
    _plain_mech_object = kit._mech_object

    _PAD_STATE = {"bound": set(), "listener": None, "callback": None, "busy": False, "colliders": 0}

    def _pad_material(stage):
        from pxr import Sdf, UsdPhysics, UsdShade

        path = "/World/sheetkit_pad_material"
        if stage.GetPrimAtPath(path).IsValid():
            return UsdShade.Material(stage.GetPrimAtPath(path))
        mat = UsdShade.Material.Define(stage, path)
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(PAD_FRICTION[0])
        api.CreateDynamicFrictionAttr(PAD_FRICTION[1])
        api.CreateRestitutionAttr(0.0)
        # PhysxMaterialAPI's combine mode, authored by name (the same USD as PhysxSchema writes; pxr without Kit's
        # PhysX schemas, as in the offline test, has no PhysxSchema module)
        mat.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
        mat.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("max")
        return mat

    def _bind_under(stage, roots) -> list:
        """Bind the pad material to every fingertip link under `roots` (prims) not bound yet."""
        from pxr import Usd, UsdPhysics, UsdShade

        links = []  # find first, then edit: un-instancing recomposes the stage and expires any running PrimRange
        for root in roots:
            for prim in Usd.PrimRange(root):
                key = str(prim.GetPath())
                # the link itself (a rigid body), not its instanceable visual child of the same name
                if (prim.GetName() in PAD_LINKS and key not in _PAD_STATE["bound"]
                        and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                    links.append(key)
        new = []
        for key in links:
            mat = _pad_material(stage)
            prim = stage.GetPrimAtPath(key)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
            # The link's collider meshes sit inside instanceable prims (instance proxies): whether PhysX's parser
            # honors a binding on an ancestor outside the instance is not documented, and a binding on the link
            # alone did not take effect. So un-instance the fingertip's sub-prims and bind each collider
            # directly as well (a few extra prims per fingertip).
            for sub in [str(p.GetPath()) for p in list(Usd.PrimRange(prim))[1:] if p.IsInstanceable()]:
                stage.GetPrimAtPath(sub).SetInstanceable(False)
            cols = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(key))
                    if p.HasAPI(UsdPhysics.CollisionAPI)]
            for c in cols:
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(c)).Bind(
                    mat, UsdShade.Tokens.strongerThanDescendants, "physics")
            _PAD_STATE["colliders"] += len(cols)
            _PAD_STATE["bound"].add(key)
            new.append(key)
        return new

    def _bind_pads(stage) -> None:
        """Bind the pads now if the arms exist, and again whenever fingertip links appear later (the arms are
        spawned after the task's objects): a USD change listener, so the binding is in place before physics starts."""
        from pxr import Tf, Usd

        def report(new):
            if new:
                if os.environ.get("SHEETKIT_VERBOSE"):
                    print(f"sheetkit: pad material {PAD_FRICTION} bound to {len(_PAD_STATE['bound'])} fingertip links: "
                          + ", ".join(sorted(new)), flush=True)

        report(_bind_under(stage, [stage.GetPseudoRoot()]))

        def changed(notice, sender):
            if _PAD_STATE["busy"]:
                return
            _PAD_STATE["busy"] = True
            try:
                roots = [stage.GetPrimAtPath(p.GetPrimPath()) for p in notice.GetResyncedPaths()]
                roots = [r for r in roots if r and r.IsValid() and "sheetkit_pad_material" not in str(r.GetPath())]
                report(_bind_under(stage, roots))
            finally:
                _PAD_STATE["busy"] = False

        if _PAD_STATE["listener"] is None:
            # Tf keeps only a weak reference to the callback: hold it here too, or it expires when this function
            # returns and the listener never fires ("expired python callback")
            _PAD_STATE["callback"] = changed
            _PAD_STATE["listener"] = Tf.Notice.Register(Usd.Notice.ObjectsChanged, changed, stage)
        _hook_sim_reset()
        if os.environ.get("SHEETKIT_VERBOSE"):
            print(f"sheetkit: pad material listener on; {len(_PAD_STATE['bound'])} fingertip links bound so far",
                  flush=True)

    def _bind_all(stage, when: str) -> int:
        """Bind every fingertip link on the stage not bound yet; print the count. Returns how many are bound."""
        new = _bind_under(stage, [stage.GetPseudoRoot()])
        if os.environ.get("SHEETKIT_VERBOSE"):
            print(f"sheetkit: pad material {PAD_FRICTION} {when}: {len(new)} new, {len(_PAD_STATE['bound'])} fingertip "
                  f"links bound, {_PAD_STATE['colliders']} colliders bound directly"
                  + (": " + ", ".join(sorted(new)) if new else ""), flush=True)
        return len(_PAD_STATE["bound"])

    def _hook_sim_reset(cls=None) -> None:
        """Bind the pads once more right before the simulator starts physics (SimulationContext.reset: every prim of
        the scene exists by then, whatever order the arms and the sheet were spawned in). The change
        listener alone never saw the arms (no notice reached it: 0 bound)."""
        if cls is None:
            try:
                from isaaclab.sim import SimulationContext as cls
            except Exception:  # noqa: BLE001  (offline: no simulator)
                return
        if getattr(cls.reset, "_sheetkit", False):
            return
        plain_reset = cls.reset

        def reset(self, *args, **kwargs):
            try:
                _bind_all(self.stage, "at physics start")
            except Exception as e:  # noqa: BLE001  (never block the run; the log shows the count)
                if os.environ.get("SHEETKIT_VERBOSE"):
                    print(f"sheetkit: pad binding at physics start failed: {e!r}", flush=True)
            return plain_reset(self, *args, **kwargs)

        reset._sheetkit = True
        cls.reset = reset

    def _author_ties(prim, ties) -> int:
        """A perforation's ties: one PhysX D6 joint per tie from the world (where the strip point is with the sheet
        flat at its spawn pose) to the strip, outside the articulation: rotations free, translations on an implicit
        spring of strength / TIE_BREAK_STRETCH. Broken by `_check_ties` (jointEnabled off). Authored before physics."""
        if not ties:
            return 0
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = prim.GetStage()
        cache = UsdGeom.XformCache()
        root = str(prim.GetPath())
        name = prim.GetName()
        # anchors in the env's frame (the task's coordinates), carried to world by the env prim's transform (not by
        # the spawned prim's, which Isaac Lab may only pose at reset)
        env_M = cache.GetLocalToWorldTransform(prim.GetParent())
        rot = env_M.ExtractRotationQuat()
        paths = []
        for k, (ln, pt, force, anchor) in enumerate(ties):
            body = stage.GetPrimAtPath(f"{root}/{ln}")
            world = env_M.Transform(Gf.Vec3d(*anchor))
            jt = UsdPhysics.Joint.Define(stage, f"{root}/tie_{k}")
            jt.CreateBody1Rel().SetTargets([body.GetPath()])
            jt.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in world)))
            jt.CreateLocalRot0Attr(Gf.Quatf(float(rot.GetReal()), *(float(v) for v in rot.GetImaginary())))
            jt.CreateLocalPos1Attr(Gf.Vec3f(*(float(v) for v in pt)))
            jt.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            jt.CreateExcludeFromArticulationAttr(True)
            jt.CreateCollisionEnabledAttr(False)
            jt.CreateJointEnabledAttr(True)
            jp = jt.GetPrim()
            # rotations free: a 1-2 mm land of paper carries no bending moment (locked rotations would make the
            # tied strips one rigid plate that cannot peel)
            kt = force / TIE_BREAK_STRETCH
            mass = kit._SHEET_TIE_MASS.get(name, 1e-4)
            for ax in ("transX", "transY", "transZ"):
                dr = UsdPhysics.DriveAPI.Apply(jp, ax)
                dr.CreateTypeAttr("force")
                dr.CreateStiffnessAttr(float(kt))
                dr.CreateDampingAttr(float(2 * math.sqrt(kt * mass)))
                dr.CreateTargetPositionAttr(0.0)
                dr.CreateMaxForceAttr(1e6)
            paths.append(str(jp.GetPath()))
        kit._SHEET_TIE_PATHS[name] = paths
        if os.environ.get("SHEETKIT_VERBOSE"):
            print(f"sheetkit: {len(ties)} perforation ties on {root} (strength {ties[0][2]:.3g} N each, spring "
                  f"{ties[0][2] / TIE_BREAK_STRETCH:.3g} N/m)", flush=True)
        return len(ties)

    def _mech_object(scene, spec):
        paper = kit._SHEET_PAPER.get(spec.name)
        if paper is None:
            return _plain_mech_object(scene, spec)
        from isaaclab.sim.spawners.from_files import spawn_from_usd
        from isaaclab.sim.utils import clone

        @clone
        def _spawn_sheet(prim_path, cfg, *args, **kwargs):
            prim = spawn_from_usd.__wrapped__(prim_path, cfg, *args, **kwargs)
            _bind_pads(prim.GetStage())
            _author_ties(prim, kit._SHEET_TIES.get(spec.name, []))
            return prim

        plain_grippy, plain_object = scene._grippy, scene.Object
        mat = type(plain_grippy())(static_friction=paper.friction, dynamic_friction=paper.kinetic_friction)

        class _SheetObject(plain_object):
            def __init__(self, *args, spawn_cfg_addon=None, **kwargs):
                addon = dict(spawn_cfg_addon or {}, physics_material=mat, func=_spawn_sheet)
                super().__init__(*args, spawn_cfg_addon=addon, **kwargs)

        scene._grippy, scene.Object = (lambda: mat), _SheetObject
        try:
            return _plain_mech_object(scene, spec)
        finally:
            scene._grippy, scene.Object = plain_grippy, plain_object

    kit._mech_object = _mech_object
    kit._sheetkit_patched = True


# -- perforation ties: read each tie's stretch every control step and break it past its strength ---------------------


def tie_loads(sheet_name: str, poses: dict) -> np.ndarray:
    """Each tie's load (N) = spring x stretch, from strip poses {objects key: (pos, R)} in the task's frame."""
    ties = kit._SHEET_TIES.get(sheet_name, [])
    out = np.zeros(len(ties))
    for k, (ln, pt, force, anchor) in enumerate(ties):
        key = sheet_name if ln == "base" else f"{sheet_name}/{ln}"
        pos, R = poses[key]
        p = np.asarray(pos, float) + np.asarray(R, float) @ np.asarray(pt, float)
        out[k] = force / TIE_BREAK_STRETCH * float(np.linalg.norm(p - np.asarray(anchor, float)))
    return out


def update_ties(sheet_name: str, loads, episode: int, disable=None) -> dict:
    """One control step of a perforation: ties whose load passes their strength break for good this trial
    (`disable(k)` switches tie k's joint off in the simulator). Returns the state (broken, peak, step broken)."""
    ties = kit._SHEET_TIES.get(sheet_name, [])
    st = kit._SHEET_TIE_STATE.get(sheet_name)
    if st is None or st["ep"] != episode:
        st = {"ep": episode, "n": 0, "broken": [False] * len(ties), "peak": [0.0] * len(ties),
              "at": [None] * len(ties), "fresh": True}
        kit._SHEET_TIE_STATE[sheet_name] = st
    st["n"] += 1
    for k, f in enumerate(loads):
        if st["broken"][k]:
            continue
        st["peak"][k] = max(st["peak"][k], float(f))
        if f > ties[k][2]:
            st["broken"][k], st["at"][k] = True, st["n"]
            if disable is not None:
                disable(k)
    return st


def tie_report(sheet) -> dict:
    """The simulator's tie state this trial: broken flags, peak loads (N), the control step each broke, torn length."""
    st = kit._SHEET_TIE_STATE.get(sheet.name, {"broken": [], "peak": [], "at": []})
    spacing = sheet.widths[0] / sheet.perforation.ties_per_strip if sheet.perforation else 0.0
    return dict(st, torn=float(sum(st["broken"]) * spacing))


if not getattr(kit, "_sheetkit_ties_patched", False):
    _plain_states_ties = kit._mech_states

    def _set_tie_enabled(name: str, k: int, on: bool) -> None:
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        UsdPhysics.Joint(stage.GetPrimAtPath(kit._SHEET_TIE_PATHS[name][k])).GetJointEnabledAttr().Set(bool(on))

    def _mech_states_ties(base, name, origin):
        out = _plain_states_ties(base, name, origin)
        if kit._SHEET_TIES.get(name) and kit._SHEET_TIE_PATHS.get(name):
            try:
                old = kit._SHEET_TIE_STATE.get(name)
                if old is not None and old["ep"] != kit.episode():  # a new trial: rejoin every broken tie
                    for k, b in enumerate(old["broken"]):
                        if b:
                            _set_tie_enabled(name, k, True)
                poses = {key: (np.asarray(o.pos, float), quat_R(o.quat_xyzw)) for key, o in out.items()}
                loads = tie_loads(name, poses)

                def off(k, _n=name, _l=loads):
                    _set_tie_enabled(_n, k, False)
                    if os.environ.get("SHEETKIT_VERBOSE"):
                        print(f"sheetkit: tie {k} of {_n} broke at {_l[k]:.3g} N", flush=True)

                update_ties(name, loads, kit.episode(), off)
            except Exception as e:  # noqa: BLE001  (never break a run; the log shows it)
                if os.environ.get("SHEETKIT_VERBOSE"):
                    print(f"sheetkit: tie check failed: {e!r}", flush=True)
        return out

    kit._mech_states = _mech_states_ties
    kit._sheetkit_ties_patched = True


# -- diagnostics: SHEETKIT_DEBUG_TIPS=<every n steps> prints each fingertip link's height and contact force -------
if not getattr(kit, "_sheetkit_tips_patched", False):
    import os as _os

    _TIPS_EVERY = int(_os.environ.get("SHEETKIT_DEBUG_TIPS", "0") or 0)
    if _TIPS_EVERY:
        _plain_mech_states = kit._mech_states
        _TIPS = {"n": 0}

        def _mech_states(base, name, origin):
            out = _plain_mech_states(base, name, origin)
            if name in kit._SHEET_NAMES:
                _TIPS["n"] += 1
                if _TIPS["n"] % _TIPS_EVERY == 0:
                    try:
                        _print_tips(base, origin, _TIPS["n"])
                    except Exception as e:  # noqa: BLE001  (diagnostics never break a run)
                        if os.environ.get("SHEETKIT_VERBOSE"):
                            print(f"sheetkit tips: {e!r}", flush=True)
            return out

        def _print_tips(base, origin, n):
            parts = []
            for robot in ("left_robot", "right_robot"):
                art = base.scene[robot]
                for tip in PAD_LINKS:
                    i = art.body_names.index(tip)
                    z = float(art.data.body_pos_w.torch[0, i, 2]) - float(origin[2])
                    parts.append(f"{robot[0]}.{tip[4]} z {z - TOP_Z:+.4f}")
            for sname, s in getattr(base.scene, "sensors", {}).items():
                f = getattr(s.data, "net_forces_w", None)
                names = getattr(s, "body_names", None)
                if f is None or names is None:
                    continue
                f = f.torch if hasattr(f, "torch") else f
                for j, b in enumerate(names):
                    if b in PAD_LINKS:
                        v = f[0, j].detach().cpu().numpy()
                        parts.append(f"{sname}:{b[4]} F ({v[0]:+.2f},{v[1]:+.2f},{v[2]:+.2f})")
            if os.environ.get("SHEETKIT_VERBOSE"):
                print(f"sheetkit tips step {n}: " + "; ".join(parts), flush=True)

        kit._mech_states = _mech_states
    kit._sheetkit_tips_patched = True

try:
    from yam_arena.task_file import TABLE_TOP as _TT

    TOP_Z = _TT["station"]  # heights printed above the station's table top
except Exception:  # noqa: BLE001
    TOP_Z = 0.0
