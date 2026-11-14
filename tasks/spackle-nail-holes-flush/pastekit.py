"""pastekit: a paste (spackle, wood filler, butter) worked into and across a surface with a rigid blade (a putty knife,
a butter knife), for a yam_arena task file. Plain Python and NumPy.

The simulator has no paste, so the paste is a scored model computed from object poses. Its parts:

- `Blade`: the knife's working edge, in the knife's own frame: a straight edge at x = `edge_x`, `width` across (y),
  on a blade `thick` thick whose mid-plane is z = `z`. The kit samples `n_pts` points along the edge on both faces of
  the blade (so a blade lying flat on a surface touches with its lower face, one held on edge touches with its tip).
- `Surface`: a flat face on a task object (a drywall panel, a slice of toast), a rectangle `size` = (lu, lv) from
  `origin` along the unit axes `u` and `v` (object frame); its outward normal is u x v. The face is a height field of
  square `cell`s: `top` is the height of the paste's top above the face (m; 0 = bare face, negative = an open recess).
  `Recess`es (nail holes, a dent) are disks with a floor `depth` below the face; their cells start at top = -depth.
  `start` adds paste already on the surface (a pat of butter): ((u, v, su, sv), thickness) rectangles.
- `Pot`: a reservoir of paste (the inside of an open tub): a disk of `radius` at height `level` (object frame, z up),
  holding paste down to `depth` below its surface. Never runs out.
- `Law`: the numbers below. One `Paste` (the state of one trial) takes the knife's pose every control step.

Physical model (each number is a `Law` field; the task states its reason):

1. Contact, not proximity. An edge sample touches a surface when it lies over the face and no more than `gap` (0.6 mm,
   about PhysX's rest jitter and contact offset) in front of it, or up to `below` (5 mm) behind it (the sim lets a
   pressed collider sink a little). A rigid blade's collider is stopped by the face, so "touching" here is the blade
   resting on or pressed against the face; a blade held 1 mm off does nothing. Once a stroke has begun the band widens
   by `gap_hyst` (0.4 mm) so contact chatter does not end it. Pressure itself is not readable from poses (the knife
   is rigid); the demo presses by commanding the edge a few mm into the face, and the collider stops it.
2. Only a held, sliding blade works paste. The task says whether the knife is held (a gripper's own shape touching the
   handle). Sliding: the edge's middle moved at least `slide_min` (1.5 mm) along the face over the last `slide_steps`
   (3 steps, 0.15 s, so 1 cm/s); motion square to the face (pressing in, pulling off) does not count. A dab, or a blade
   pressed and held still, moves no paste.
3. Each surface cell is worked once per entry: when it comes under a touching edge sample (moving away and coming back,
   as in a second pass, is a new entry). Between control steps the knife's pose is interpolated in sub-steps of at most
   half a cell, so a fast stroke still works every cell it crossed.
4. The blade's angle to the face sets what it leaves behind. theta = the angle between the blade's plane and the face
   (0 = lying flat on it, 90 = square to it). A blade held low rides up on the paste (it hydroplanes on the wedge of
   paste under it) and leaves a coat; held steep it scrapes the face nearly clean. Drywall finishers use exactly this:
   more angle removes material, less angle lays it on. The kit's level left behind is
       film(theta) = `film_low` (0.8 mm) at theta <= `low_deg` (30 deg), `film_high` (0.1 mm) at theta >= `high_deg`
       (60 deg), linear between.
   On each entered cell: paste standing above film(theta) is scraped off into the knife's load (top := film); a cell
   whose top is below film(theta) is filled up to it from the load, if the knife carries paste.
5. Packing a recess takes a low angle. Below `pack_max_deg` (50 deg) the paste rolled ahead of the blade is pressed
   into a recess and fills it from its floor. At a steeper angle the blade bridges a recess: it tops up only cells
   whose top is within `bridge` (1 mm) of the face (a shallow dent or the last skim), and passes over a deeper hole,
   leaving it open. So a steep pass alone never fills a nail hole, and a low pass alone leaves a coat standing proud.
6. Loading: a held blade sliding with edge or face samples under a pot's paste surface (within its radius and depth)
   picks up `load_rate` (0.5 mL per cm of travel with the whole edge under) until it carries `load_cap` (2 mL).
   Scraped paste also goes into the load (no cap: a knife can carry what it scrapes). Volume is conserved: what is
   laid comes out of the load and laying stops when the load is used up (the last cell gets what is left).
7. Gouging: the blade's corner digging in. When the edge is turned more than `gouge_tilt_deg` (15 deg) out of the
   face's plane, only one end touches; if that corner slides along a touching face for `gouge_len` (5 mm) in one
   stroke the surface is gouged (latched for the trial; where it happened is recorded). A real knife's corner dragged
   like this tears drywall's paper face or scores a soft surface.

State the scorer reads (all in SI units; helpers in mm where named so):
    P = Paste([surfaces], [pots], blade, law)      one trial; P.step(knife_pos, knife_quat, held, poses) per control
                                                  step, with poses = {object name: (pos, quat_xyzw)} for every
                                                  surface's and pot's object
    P.remaining_depth(surf, i)   deepest point of recess i below the face (m; 0 = filled flush or proud)
    P.fill_fraction(surf, i)     paste volume inside recess i / its volume (0..1)
    P.proud_area(surf, above)    area (m^2) of cells whose paste stands more than `above` over the face (a ridge,
                                 a coat left on), recesses included; P.film_area(surf, above) the same for any film
    P.smear_volume(surf)         paste on the face outside the recesses (m^3)
    P.profile(surf)              a copy of the height field `top` (m), rows along u, columns along v
    P.load                       paste on the knife now (m^3); P.loaded_total: picked up from pots this trial
    P.gouged(surf)               latched: the corner dug in (P.gouge_at[surf]: surface (u, v) where it did)
    P.touching(surf)             the edge touches the surface at this step (whether or not it slides)
    P.strokes                    list of Stroke records (surface, first/last step, min/max angle, cells worked)
    P.blocks(surf, k, ...)       drawable blocks [(u, v, su, sv, key)] of k x k cells, key in "fill", "part",
                                 "film", "proud" (see `Tiles`)

Visuals: `Tiles` draws the blocks as thin colored tiles just in front of the face with Isaac Lab's
VisualizationMarkers (imported lazily; off with one warning if unavailable; scoring never depends on it).

Simplifications (say them in the task's fidelity): the paste is bookkeeping on a height field, not a fluid (no flow,
slump, shrinkage or drying); the blade is rigid (a real flexible putty knife bends, which is how a worker feels
pressure); the paste does not push back on the knife; stroke direction (handle leading or edge leading) does not
matter, only the angle; a knife lifted off does not leave a ridge at the end of its stroke.
"""

from __future__ import annotations

import math
import sys
import warnings
from dataclasses import dataclass, field

import numpy as np

ML = 1e-6  # m^3 in a millilitre


# -- configuration ------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Law:
    gap: float = 0.0006  # m in front of the face that still touches
    gap_hyst: float = 0.0004  # m extra once a stroke has begun
    below: float = 0.005  # m behind the face (a pressed collider sinking in)
    slide_min: float = 0.0015  # m the edge's middle must move ...
    slide_steps: int = 3  # ... over this many control steps
    lift_steps: int = 3  # steps without contact that end a stroke
    low_deg: float = 30.0  # at or below: the low-angle coat
    high_deg: float = 60.0  # at or above: the steep scrape
    film_low: float = 0.0008  # m left by a low blade
    film_high: float = 0.0001  # m left by a steep blade
    pack_max_deg: float = 50.0  # at or below, a blade packs recesses from the floor
    bridge: float = 0.001  # m: above pack_max_deg, only cells this close below the face are topped up
    load_rate: float = 0.5 * ML / 0.01  # m^3 per m of travel with the whole edge in the paste
    load_cap: float = 2.0 * ML  # m^3 a blade picks up from a pot
    gouge_tilt_deg: float = 15.0  # edge turned this far out of the face's plane: only a corner touches
    gouge_len: float = 0.005  # m a corner may slide before the face is gouged

    def film(self, theta_deg: float) -> float:
        """The paste level a blade at `theta_deg` leaves above the face (m)."""
        if theta_deg <= self.low_deg:
            return self.film_low
        if theta_deg >= self.high_deg:
            return self.film_high
        s = (theta_deg - self.low_deg) / (self.high_deg - self.low_deg)
        return self.film_low + s * (self.film_high - self.film_low)


@dataclass(frozen=True)
class Blade:
    edge_x: float  # m: the edge line, in the knife's frame
    width: float  # m across the edge (knife y)
    thick: float = 0.001  # m: the blade's collider thickness
    z: float = 0.0  # m: the blade's mid-plane (knife z)
    n_pts: int = 39  # samples along the edge
    face_len: float = 0.02  # m of blade face back from the edge sampled for loading from a pot


@dataclass(frozen=True)
class Recess:
    u: float  # m: center on the surface
    v: float
    radius: float
    depth: float
    name: str = ""


@dataclass(frozen=True)
class Surface:
    name: str
    target: str  # the task object it is on
    origin: tuple  # m, object frame: the (u, v) = (0, 0) corner on the face
    u: tuple  # unit axes in the object frame; outward normal = u x v
    v: tuple
    size: tuple  # (lu, lv) m
    cell: float = 0.001
    recesses: tuple = ()
    start: tuple = ()  # ((u0, v0, su, sv), thickness m) paste already there


@dataclass(frozen=True)
class Pot:
    name: str
    target: str
    center: tuple  # m, object frame: the middle of the paste's surface
    radius: float
    depth: float  # m of paste under the surface that counts as "in the paste"


@dataclass
class Stroke:
    surface: str
    first: int
    last: int
    theta_min: float
    theta_max: float
    cells: int = 0


# -- geometry -----------------------------------------------------------------------------------------------------------


def rotm(q) -> np.ndarray:
    """Rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat(r: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w) from a rotation matrix."""
    r = np.asarray(r, float)
    t = float(np.trace(r))
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return q if q[3] >= 0 else -q


def _nlerp(a, b, s: float) -> np.ndarray:
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.dot(a, b) < 0:
        b = -b
    q = (1 - s) * a + s * b
    return q / (np.linalg.norm(q) or 1.0)


def _unit(a) -> np.ndarray:
    a = np.asarray(a, float)
    return a / (np.linalg.norm(a) or 1.0)


def edge_points(b: Blade) -> np.ndarray:
    """Edge samples in the knife frame: (2, n, 3), on the blade's two faces."""
    ys = np.linspace(-b.width / 2, b.width / 2, b.n_pts)
    out = []
    for dz in (-b.thick / 2, b.thick / 2):
        out.append(np.stack([np.full_like(ys, b.edge_x), ys, np.full_like(ys, b.z + dz)], axis=1))
    return np.array(out)


def face_points(b: Blade, step: float = 0.002) -> np.ndarray:
    """Samples over the blade's last `face_len` (both faces), knife frame, (m, 3): what dips into a pot."""
    xs = np.arange(b.edge_x, b.edge_x - b.face_len - 1e-12, -step)
    ys = np.linspace(-b.width / 2, b.width / 2, 9)
    g = [(x, y, b.z + dz) for x in xs for y in ys for dz in (-b.thick / 2, b.thick / 2)]
    return np.array(g, float)


def blade_angle_deg(rot: np.ndarray, normal: np.ndarray) -> float:
    """Angle between the blade's plane (knife x-y) and a face with unit `normal` (deg, 0..90)."""
    nb = rot[:, 2]
    return math.degrees(math.acos(min(1.0, abs(float(np.dot(nb, normal))))))


def edge_tilt_deg(rot: np.ndarray, normal: np.ndarray) -> float:
    """How far the edge line (knife y) is turned out of a face's plane (deg, 0..90)."""
    return math.degrees(math.asin(min(1.0, abs(float(np.dot(rot[:, 1], normal))))))


# -- per-surface state --------------------------------------------------------------------------------------------------


class _Field:
    def __init__(self, s: Surface):
        self.s = s
        self.nu, self.nv = round(s.size[0] / s.cell), round(s.size[1] / s.cell)
        uc = (np.arange(self.nu) + 0.5) * s.cell
        vc = (np.arange(self.nv) + 0.5) * s.cell
        self.U, self.V = np.meshgrid(uc, vc, indexing="ij")
        self.floor = np.zeros((self.nu, self.nv))
        self.rid = np.full((self.nu, self.nv), -1, dtype=int)  # which recess a cell is in
        for i, r in enumerate(s.recesses):
            m = (self.U - r.u) ** 2 + (self.V - r.v) ** 2 <= r.radius ** 2
            self.floor[m] = -r.depth
            self.rid[m] = i
        self.top = self.floor.copy()
        for (u0, v0, su, sv), t in s.start:
            m = (self.U >= u0) & (self.U < u0 + su) & (self.V >= v0) & (self.V < v0 + sv)
            self.top[m] = np.maximum(self.top[m], t)
        self.under = set()  # flat indices under a touching edge at the last sub-step
        self.stroke: Stroke | None = None
        self.miss = 0
        self.touch = False
        self.gouged = False
        self.gouge_at = None
        self.corner_run = 0.0  # m a corner has slid in this stroke
        self.worked = np.zeros((self.nu, self.nv), dtype=int)

    @property
    def area(self) -> float:
        return self.s.cell ** 2


class Paste:
    """One trial of paste on `surfaces`, loaded from `pots`, worked by one knife with `blade`."""

    def __init__(self, surfaces, pots=(), blade: Blade | None = None, law: Law = Law()):
        self.law, self.blade = law, blade
        self.surfaces = {s.name: s for s in surfaces}
        self.pots = {p.name: p for p in pots}
        self.f = {s.name: _Field(s) for s in surfaces}
        self._edge = edge_points(blade)
        self._face = face_points(blade)
        self.load = 0.0
        self.loaded_total = 0.0
        self.n = 0  # control steps seen
        self.prev = None  # (pos, quat) last step
        self.mids = []  # the edge's middle (world) at recent steps
        self.strokes: list[Stroke] = []
        self.gouge_at = {}

    # ---- the step ----
    def _frame(self, poses, s: Surface):
        p, q = poses[s.target]
        r = rotm(q)
        o = np.asarray(p, float) + r @ np.asarray(s.origin, float)
        u, v = r @ _unit(s.u), r @ _unit(s.v)
        n = np.cross(u, v)
        return o, u, v, n / np.linalg.norm(n)

    def step(self, pos, q, held: bool, poses: dict) -> None:
        """One control step: the knife at (pos, q), `held` or not; poses = {object: (pos, quat)}."""
        L = self.law
        self.n += 1
        pos, q = np.asarray(pos, float), np.asarray(q, float)
        r_now = rotm(q)
        mid_local = np.array([self.blade.edge_x, 0.0, self.blade.z])
        mid = pos + r_now @ mid_local
        self.mids = (self.mids + [mid])[-(L.slide_steps + 1):]
        disp = self.mids[-1] - self.mids[0] if len(self.mids) > L.slide_steps else None
        prev = self.prev if self.prev is not None else (pos, q)
        self.prev = (pos, q)
        prev_mid = prev[0] + rotm(prev[1]) @ mid_local
        span = float(np.linalg.norm(mid - prev_mid))
        rot_span = math.acos(max(-1.0, min(1.0, (float(np.trace(rotm(prev[1]).T @ r_now)) - 1) / 2)))
        cell = min((s.cell for s in self.surfaces.values()), default=0.001)
        reach = self.blade.width / 2 + abs(self.blade.edge_x)
        n_sub = int(min(200, max(1, math.ceil(max(span, rot_span * reach) / (cell / 2)))))
        frames = {name: self._frame(poses, s) for name, s in self.surfaces.items() if s.target in poses}
        pots = {}
        for name, p in self.pots.items():
            if p.target in poses:
                pp, pq = poses[p.target]
                pots[name] = np.asarray(pp, float) + rotm(pq) @ np.asarray(p.center, float)
        for f in self.f.values():
            f.touch = False
        for k in range(1, n_sub + 1):
            sk = k / n_sub
            p_k = prev[0] + sk * (pos - prev[0])
            r_k = r_now if n_sub == 1 else rotm(_nlerp(prev[1], q, sk))
            travel = span / n_sub
            for name, fr in frames.items():
                self._work(self.f[name], fr, p_k, r_k, held, self._slides(disp, fr[3]), travel)
            for name, c in pots.items():
                self._dip(self.pots[name], c, p_k, r_k, held, self._slides(disp, np.array([0.0, 0.0, 1.0])), travel)
        for name, f in self.f.items():  # strokes end after lift_steps without contact
            if f.touch:
                f.miss = 0
                if f.stroke is not None:
                    f.stroke.last = self.n
            elif f.stroke is not None:
                f.miss += 1
                if f.miss > L.lift_steps:
                    f.stroke, f.corner_run, f.under = None, 0.0, set()

    def _slides(self, disp, normal) -> bool:
        """The edge's middle moved at least slide_min along a face (normal motion, pressing in, does not count)."""
        if disp is None:
            return False
        t = disp - float(np.dot(disp, normal)) * normal
        return float(np.linalg.norm(t)) >= self.law.slide_min

    def _work(self, f: _Field, fr, p, r, held, sliding, travel) -> None:
        L, s = self.law, f.s
        o, eu, ev, n = fr
        pts = self._edge @ r.T + p  # (2, m, 3)
        d = (pts - o) @ n  # (2, m): height in front of the face
        k = np.argmin(np.abs(d), axis=0)
        cols = np.arange(d.shape[1])
        dd = d[k, cols]
        pp = pts[k, cols]
        uu, vv = (pp - o) @ eu, (pp - o) @ ev
        gap = L.gap + (L.gap_hyst if f.stroke is not None else 0.0)
        on = (dd <= gap) & (dd >= -L.below) & (uu >= 0) & (uu < s.size[0]) & (vv >= 0) & (vv < s.size[1])
        if not on.any():
            f.under = set()
            return
        f.touch = True
        theta = blade_angle_deg(r, n)
        if f.stroke is None:
            f.stroke = Stroke(s.name, self.n, self.n, theta, theta)
            self.strokes.append(f.stroke)
            f.miss = 0
        f.stroke.theta_min = min(f.stroke.theta_min, theta)
        f.stroke.theta_max = max(f.stroke.theta_max, theta)
        iu = np.clip((uu[on] / s.cell).astype(int), 0, f.nu - 1)
        iv = np.clip((vv[on] / s.cell).astype(int), 0, f.nv - 1)
        now = set((iu * f.nv + iv).tolist())
        new = now - f.under
        f.under = now
        if not (held and sliding):
            return
        tilt = edge_tilt_deg(r, n)
        if tilt > L.gouge_tilt_deg:
            f.corner_run += travel
            if f.corner_run >= L.gouge_len and not f.gouged:
                f.gouged = True
                f.gouge_at = (float(np.mean(uu[on])), float(np.mean(vv[on])))
                self.gouge_at[s.name] = f.gouge_at
        if not new:
            return
        film = L.film(theta)
        pack = theta <= L.pack_max_deg
        idx = np.array(sorted(new))
        top = f.top.ravel()
        f.worked.ravel()[idx] += 1
        f.stroke.cells += len(idx)
        hi = idx[top[idx] > film]
        if len(hi):  # scrape the excess into the load
            self.load += float(np.sum(top[hi] - film)) * f.area
            top[hi] = film
        lo = idx[top[idx] < film]
        if not pack:
            lo = lo[top[lo] >= -L.bridge - 1e-12]
        for c in lo:  # fill from the load, in order, until it runs out
            if self.load <= 1e-15:
                break
            need = (film - top[c]) * f.area
            give = min(need, self.load)
            top[c] += give / f.area
            self.load -= give

    def _dip(self, pot: Pot, c, p, r, held, sliding, travel) -> None:
        if not (held and sliding) or self.load >= self.law.load_cap:
            return
        e = self._edge[0] @ r.T + p
        dz = c[2] - e[:, 2]
        rad = np.linalg.norm((e - c)[:, :2], axis=1)
        under = (dz >= 0) & (dz <= pot.depth) & (rad <= pot.radius)
        share = float(under.mean())
        if share <= 0:
            fp = self._face @ r.T + p
            dzf = c[2] - fp[:, 2]
            radf = np.linalg.norm((fp - c)[:, :2], axis=1)
            share = 0.5 * float(((dzf >= 0) & (dzf <= pot.depth) & (radf <= pot.radius)).mean())
        if share <= 0:
            return
        add = min(self.law.load_rate * share * travel, self.law.load_cap - self.load)
        self.load += add
        self.loaded_total += add

    # ---- what the scorer reads ----
    def _recess_cells(self, surf: str, i: int):
        f = self.f[surf]
        return f, f.rid == i

    def remaining_depth(self, surf: str, i: int) -> float:
        f, m = self._recess_cells(surf, i)
        return float(max(0.0, -float(f.top[m].min()))) if m.any() else 0.0

    def fill_fraction(self, surf: str, i: int) -> float:
        f, m = self._recess_cells(surf, i)
        if not m.any():
            return 1.0
        inside = np.clip(f.top[m], f.floor[m], 0.0) - f.floor[m]
        return float(inside.sum() / (-f.floor[m]).sum())

    def proud_area(self, surf: str, above: float) -> float:
        f = self.f[surf]
        return float((f.top > above).sum()) * f.area

    def film_area(self, surf: str, above: float = 0.00005) -> float:
        f = self.f[surf]
        return float(((f.top > above) & (f.rid < 0)).sum()) * f.area

    def smear_volume(self, surf: str) -> float:
        f = self.f[surf]
        m = f.rid < 0
        return float(np.clip(f.top[m], 0.0, None).sum()) * f.area

    def profile(self, surf: str) -> np.ndarray:
        return self.f[surf].top.copy()

    def gouged(self, surf: str) -> bool:
        return self.f[surf].gouged

    def touching(self, surf: str) -> bool:
        return self.f[surf].touch

    def total_volume(self) -> float:
        """Paste on every surface above its floor, plus the knife's load (m^3): conservation checks."""
        out = self.load
        for f in self.f.values():
            out += float((f.top - f.floor).clip(0, None).sum()) * f.area
        return out

    def blocks(self, surf: str, k: int = 2, flush: float = 0.0003, proud: float = 0.0003, film: float = 0.00005):
        """Drawable blocks of k x k cells with paste: [(u, v, su, sv, key)], key "proud" if any cell stands more than
        `proud` over the face, else "fill" for a block of a recess filled to within `flush` of the face, "part" for a
        recess holding some paste but not filled, "film" for a coat thicker than `film`. Empty blocks are left out."""
        f = self.f[surf]
        nu, nv = f.nu, f.nv
        pu, pv = (-nu) % k, (-nv) % k

        def pad(a, val):
            return np.pad(a, ((0, pu), (0, pv)), constant_values=val).reshape((nu + pu) // k, k, (nv + pv) // k, k)

        t, fl, rec = pad(f.top, -1.0), pad(f.floor, -1.0), pad(f.rid >= 0, False)
        any_proud = (t > proud).any(axis=(1, 3))
        has_rec = rec.any(axis=(1, 3))
        rec_ok = np.where(rec, t >= -flush, True).all(axis=(1, 3))
        rec_some = (rec & (t > fl + 1e-9)).any(axis=(1, 3))
        any_film = (t > film).any(axis=(1, 3))
        key = np.full(any_proud.shape, "", dtype=object)
        key[any_film & ~has_rec] = "film"
        key[has_rec & rec_some] = "part"
        key[has_rec & rec_ok] = "fill"
        key[any_proud] = "proud"
        c = f.s.cell
        out = []
        for bi, bj in zip(*np.nonzero(key != "")):
            su, sv = min(k, nu - bi * k) * c, min(k, nv - bj * k) * c
            out.append((bi * k * c + su / 2, bj * k * c + sv / 2, su, sv, str(key[bi, bj])))
        return out


# -- visuals ------------------------------------------------------------------------------------------------------------

COLORS = {"fill": (0.95, 0.55, 0.70), "part": (0.80, 0.35, 0.55), "film": (0.98, 0.78, 0.86),
          "proud": (0.90, 0.20, 0.55)}  # pink spackle (the kind that goes on pink and dries white)


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class Tiles:
    """Paste blocks drawn as thin tiles `lift` in front of a surface's face. Call draw(paste, surf, frame, origin)
    after each step with the surface's world frame (o, u, v, n) and the env origin. Imported lazily; turns itself off
    with one warning on any error (outside the simulator it does nothing)."""

    def __init__(self, k: int = 2, lift: float = 0.0004, thick: float = 0.0003, max_tiles: int = 6000,
                 path: str = "/Visuals/pastekit"):
        self.k, self.lift, self.thick, self.max = k, lift, thick, max_tiles
        self.path, self.markers, self.off, self.shown, self.keys = path, None, False, None, list(COLORS)

    def draw(self, paste: Paste, surf: str, frame, origin=(0.0, 0.0, 0.0)) -> None:
        if self.off:
            return
        blocks = paste.blocks(surf, self.k)[: self.max]
        sig = hash(tuple((round(a, 5), round(b, 5), key) for a, b, _, _, key in blocks))
        if sig == self.shown:
            return
        try:
            if self.markers is None:
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                protos = {key: sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(_linear(x) for x in COLORS[key]), roughness=0.6)) for key in self.keys}
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path, markers=protos))
            self.shown = sig
            if not blocks:
                self.markers.set_visibility(False)
                return
            o, eu, ev, n = (np.asarray(x, float) for x in frame)
            qr = quat(np.stack([eu, ev, n], axis=1))
            tr = [o + a * eu + b * ev + n * (self.lift + self.thick / 2) + np.asarray(origin, float)
                  for a, b, _, _, _ in blocks]
            sc = [(su * 0.95, sv * 0.95, self.thick) for _, _, su, sv, _ in blocks]
            ids = [self.keys.index(key) for *_, key in blocks]
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32),
                                   orientations=np.tile(qr, (len(blocks), 1)).astype(np.float32),
                                   scales=np.array(sc, np.float32), marker_indices=np.array(ids, int))
        except ImportError:
            self.off = True  # no simulator: offline tests
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            warnings.warn(f"pastekit tiles off ({type(e).__name__}: {e}); scoring is unaffected", RuntimeWarning)
            print(f"pastekit: tiles off ({e!r})", file=sys.stderr, flush=True)
