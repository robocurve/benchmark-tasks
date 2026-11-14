"""scrapekit: a bonded film (carbonized grease on a griddle, dried soil on a flat plate) scraped off a fixed flat plate
with a rigid blade and pushed into a trough along one edge, for a yam_arena task file. Plain Python and NumPy;
offline checks: `python3 test_scrapekit.py`.

The simulator has no grease, carbon or debris, so the film is a scored model computed from object poses (the same
approach as pastekit and wipekit, which it borrows from: contact band, held, sliding, sub-stepping, angle law).

- `Plate`: a horizontal rectangle of square `cell`s at height `z` (world frame), from (x0, y0) along +x (length lx)
  and +y (width ly). Each cell holds `bonded` film (soil units still stuck to the plate) and `loose` debris (scraped
  off but left lying on the plate: a ridge where a loaded blade was lifted, or a windrow that spilled round a blade
  end). The plate's edge at x = x0 is the lip of the `trough`, which runs below it from x = trough_x0 to x0 across
  the trough's (y0, y1). The other three edges have splash guards: debris is never pushed past them.
- `Blade`: the scraper's straight edge in the scraper's own frame: its middle `center`, the unit direction along the
  edge `along`, the unit normal of the blade's plane `normal`, and the edge's `width`. The edge is divided into
  `lanes` equal lanes; each lane carries its own load of debris (what has piled up in front of that part of the
  blade).

Physical model (each number is a `Law` field; the task states its reason):

1. Contact, not proximity. An edge sample touches the plate when it lies over a plate cell no more than `gap`
   (1 mm) above the plate's top or up to `below` (5 mm) under it (a pressed collider sinks a little). The band widens
   by `gap_hyst` once a stroke has begun. A blade held off the plate does nothing. Pressure is not read (the scraper
   is rigid and poses carry no force); the demo presses by aiming the edge a few mm into the plate.
2. Only a held, sliding blade works. The task says whether the scraper is held (a gripper's own shape touching its
   handle). Sliding: the edge's middle moved at least `slide_min` (1.5 mm) along the plate over `slide_steps` (3
   steps, 0.15 s). A dab, a press held still, or a blade nudged along without a hand does nothing.
3. Angle band. theta is the angle between the blade's plane and the plate (0 = flat, 90 = square to it). Inside
   [`bite_lo`, `bite_hi`] (20 to 60 deg) the edge bites under the film: each plate cell newly entered by a touching
   edge sample loses up to `bite` (1 soil unit) of bonded film into that lane's load (a level-2 cell needs two passes).
   Flatter than `bite_lo`, the blade rides up over the film and removes none; steeper than `bite_hi`, it chatters
   and removes none. Any touching, sliding edge (any angle) gathers the loose debris of the cells it enters.
4. Gouging (latched). Steeper than `gouge_deg` (70 deg) and sliding `gouge_len` (1 cm) in one stroke, or with the
   edge turned more than `tilt_max` (10 deg) out of the plate's plane (only a corner touching) and sliding 1 cm: the
   plate is gouged (chatter marks or a scored line through the seasoning). Where it happened is recorded.
5. Transport. Debris in front of the blade moves with it only square to the edge. When the blade slides along its
   own edge (a skewed stroke), the debris stays put in the world and so shifts from lane to lane; what passes the
   blade's end is left as a windrow of loose debris on the plate just outside that end. Even on a square stroke the
   two end lanes leak `leak` (4 % of their load per cm of travel) round the blade's ends into the cells just outside.
   So material escapes round the ends unless the next stroke overlaps the last one and picks the windrow up.
6. Delivery. A lane whose edge sample is past the lip (x < x0) and over the trough while the stroke is on drops its
   whole load into the trough. A windrow or ridge that would land past the lip also goes into the trough.
7. Lift-off. A stroke ends `lift_steps` (3 steps) after the last touch. A load still on the blade then is either
   flung (counted as `spilled`, latched) if the edge's middle moved faster than `fling_speed` (0.3 m/s) during the
   lift, or left as a ridge of loose debris under each lane's last touching sample.
Soil is conserved: bonded + loose (plate) + load (blade) + trough + spilled = the soil laid down (tested).

State the scorer reads: S = Scrape(plate, blade, law, soil); S.step(pos, quat, held) per control step with the
scraper's pose. S.applied (units laid down), S.trough, S.spilled, S.load_total(), S.bonded_total(),
S.loose_total(), S.residue() (bonded + loose on the plate), S.gouged, S.gouge_at, S.touching (this step),
S.strokes (list of Stroke), S.cells_view() (the per-cell arrays), S.blocks(k) (drawable soil blocks).

Simplifications (say them in the task's fidelity): the film is bookkeeping on a 1 cm grid, not a material: no
heat, no grease flow, no sticking to the blade, no friction change; one pass removes up to one unit whatever the
force (pressure is not read); debris is carried in lanes, not as particles; the blade is rigid.
"""

from __future__ import annotations

import math
import sys
import warnings
from dataclasses import dataclass, field

import numpy as np


# -- configuration ------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Law:
    gap: float = 0.001  # m above the plate that still touches
    gap_hyst: float = 0.0005  # m extra once a stroke has begun
    below: float = 0.005  # m under the plate's top (a pressed collider sinking in)
    slide_min: float = 0.0015  # m the edge's middle must move along the plate ...
    slide_steps: int = 3  # ... over this many control steps
    lift_steps: int = 3  # steps without contact that end a stroke
    bite_lo: float = 20.0  # deg: flatter rides over the film
    bite_hi: float = 60.0  # deg: steeper chatters
    bite: float = 1.0  # soil units removed per cell entry inside the band
    gouge_deg: float = 70.0  # deg: steeper, sliding gouge_len, gouges
    tilt_max: float = 10.0  # deg the edge may turn out of the plate's plane (a corner digging in beyond it)
    gouge_len: float = 0.01  # m slid steep or on a corner in one stroke before the plate is gouged
    leak: float = 4.0  # fraction of an end lane's load per m of travel (4 % per cm)
    fling_speed: float = 0.3  # m/s: a lift faster than this flings the load


@dataclass(frozen=True)
class Plate:
    x0: float  # m: the lip (trough side)
    y0: float
    lx: float  # m along +x (away from the lip)
    ly: float  # m along +y
    z: float  # m: the plate's top
    cell: float = 0.01
    trough_x0: float = -1.0  # m: the trough's far wall (x < x0)
    trough_y: tuple = (-1.0, 1.0)  # m: the trough's extent across


@dataclass(frozen=True)
class Blade:
    center: tuple  # m, scraper frame: the middle of the edge
    along: tuple  # unit, scraper frame: along the edge
    normal: tuple  # unit, scraper frame: the blade's plane normal
    width: float  # m
    lanes: int = 10


@dataclass
class Stroke:
    first: int
    last: int
    theta_min: float
    theta_max: float
    cells: int = 0  # cell entries
    removed: float = 0.0  # bonded units removed
    delivered: float = 0.0  # units dropped in the trough
    end: str = ""  # "ridge", "flung", "" (open or empty)


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


def blade_angle_deg(r: np.ndarray, blade: Blade) -> float:
    """Angle between the blade's plane and a horizontal plate (deg, 0..90)."""
    nb = r @ np.asarray(blade.normal, float)
    return math.degrees(math.acos(min(1.0, abs(float(nb[2])))))


def edge_tilt_deg(r: np.ndarray, blade: Blade) -> float:
    """How far the edge line is turned out of the plate's (horizontal) plane (deg, 0..90)."""
    a = r @ np.asarray(blade.along, float)
    return math.degrees(math.asin(min(1.0, abs(float(a[2])))))


# -- the model ----------------------------------------------------------------------------------------------------------


class Scrape:
    """One trial: `soil` = [((x, y, sx, sy), units)] rectangles of bonded film (world, m), added up where they
    overlap."""

    def __init__(self, plate: Plate, blade: Blade, law: Law = Law(), soil=()):
        self.p, self.b, self.law = plate, blade, law
        self.nu, self.nv = round(plate.lx / plate.cell), round(plate.ly / plate.cell)
        xc = plate.x0 + (np.arange(self.nu) + 0.5) * plate.cell
        yc = plate.y0 + (np.arange(self.nv) + 0.5) * plate.cell
        self.X, self.Y = np.meshgrid(xc, yc, indexing="ij")
        self.bonded = np.zeros((self.nu, self.nv))
        for (x, y, sx, sy), units in soil:
            m = (self.X >= x) & (self.X < x + sx) & (self.Y >= y) & (self.Y < y + sy)
            self.bonded[m] += units
        self.start = self.bonded.copy()
        self.applied = float(self.bonded.sum())
        self.loose = np.zeros((self.nu, self.nv))
        self.passes = np.zeros((self.nu, self.nv), dtype=int)
        self.lane_load = np.zeros(blade.lanes)
        self.trough = 0.0
        self.spilled = 0.0
        self.gouged = False
        self.gouge_at = None
        self.touching = False
        self.n = 0
        self.prev = None
        self.mids = []
        self.stroke: Stroke | None = None
        self.strokes: list[Stroke] = []
        self.miss = 0
        self.lift_speed = 0.0
        self.steep_run = 0.0
        self.corner_run = 0.0
        self.shift = 0.0  # fractional lane offset of the load (material stays in the world)
        self.under = set()  # plate cells under a touching edge sample at the last sub-step
        self.last_cells = [None] * blade.lanes  # each lane's last touching cell (u, v), for a ridge
        s = (np.arange(2 * blade.lanes) + 0.5) / (2 * blade.lanes) - 0.5
        self._s = s * blade.width  # sample offsets along the edge, two per lane
        self._lane_of = np.arange(2 * blade.lanes) // 2

    # ---- helpers ----
    def _cell(self, x: float, y: float):
        """(iu, iv) of the plate cell at a world point, or None off the plate."""
        p = self.p
        iu, iv = math.floor((x - p.x0) / p.cell), math.floor((y - p.y0) / p.cell)
        if 0 <= iu < self.nu and 0 <= iv < self.nv:
            return iu, iv
        return None

    def _over_trough(self, x: float, y: float) -> bool:
        p = self.p
        return p.trough_x0 <= x < p.x0 and p.trough_y[0] <= y <= p.trough_y[1]

    def _drop(self, x: float, y: float, amount: float) -> None:
        """Leave `amount` of loose debris at a world point: in the trough past the lip, else on the nearest plate cell
        (splash guards stop it at the other three edges)."""
        if amount <= 0:
            return
        if x < self.p.x0 and self._over_trough(x, y):
            self.trough += amount
            if self.stroke is not None:
                self.stroke.delivered += amount
            return
        p = self.p
        iu = min(max(math.floor((x - p.x0) / p.cell), 0), self.nu - 1)
        iv = min(max(math.floor((y - p.y0) / p.cell), 0), self.nv - 1)
        self.loose[iu, iv] += amount

    # ---- the step ----
    def step(self, pos, q, held: bool) -> None:
        """One control step with the scraper at (pos, q) (world), `held` or not."""
        L, b = self.law, self.b
        self.n += 1
        pos, q = np.asarray(pos, float), np.asarray(q, float)
        r_now = rotm(q)
        c_loc = np.asarray(b.center, float)
        mid = pos + r_now @ c_loc
        self.mids = (self.mids + [mid])[-(L.slide_steps + 1):]
        disp = self.mids[-1] - self.mids[0] if len(self.mids) > L.slide_steps else None
        sliding = disp is not None and math.hypot(disp[0], disp[1]) >= L.slide_min
        prev = self.prev if self.prev is not None else (pos, q)
        self.prev = (pos, q)
        prev_mid = prev[0] + rotm(prev[1]) @ c_loc
        span = float(np.linalg.norm(mid - prev_mid))
        rot_span = math.acos(max(-1.0, min(1.0, (float(np.trace(rotm(prev[1]).T @ r_now)) - 1) / 2)))
        n_sub = int(min(200, max(1, math.ceil(max(span, rot_span * b.width / 2) / (self.p.cell / 2)))))
        touched = False
        last_mid = prev_mid
        for k in range(1, n_sub + 1):
            sk = k / n_sub
            p_k = prev[0] + sk * (pos - prev[0])
            r_k = r_now if n_sub == 1 else rotm(_nlerp(prev[1], q, sk))
            m_k = p_k + r_k @ c_loc
            touched |= self._sub(p_k, r_k, m_k - last_mid, held, sliding)
            last_mid = m_k
        self.touching = touched
        speed = span / 0.05  # m/s at 20 Hz
        if touched:
            self.miss, self.lift_speed = 0, 0.0
            if self.stroke is not None:
                self.stroke.last = self.n
        elif self.stroke is not None:
            self.miss += 1
            self.lift_speed = max(self.lift_speed, speed)
            if self.miss > L.lift_steps:
                self._end_stroke()

    def _sub(self, p, r, dmid, held, sliding) -> bool:
        L, b, P = self.law, self.b, self.p
        along = r @ np.asarray(b.along, float)
        c = p + r @ np.asarray(b.center, float)
        pts = c[None, :] + self._s[:, None] * along[None, :]  # (2 lanes, 3)
        h = pts[:, 2] - P.z
        gap = L.gap + (L.gap_hyst if self.stroke is not None else 0.0)
        on_band = (h <= gap) & (h >= -L.below)
        cells = [self._cell(x, y) for x, y in pts[:, :2]]
        on = np.array([bool(on_band[i]) and cells[i] is not None for i in range(len(pts))])
        # delivery: lanes past the lip over the trough drop their load while a stroke is on
        if self.stroke is not None or on.any():
            for i in range(len(pts)):
                ln = self._lane_of[i]
                if self.lane_load[ln] > 0 and pts[i, 0] < P.x0 and self._over_trough(pts[i, 0], pts[i, 1]) \
                        and h[i] <= 0.03:
                    self.trough += self.lane_load[ln]
                    if self.stroke is not None:
                        self.stroke.delivered += self.lane_load[ln]
                    self.lane_load[ln] = 0.0
        if not on.any():
            self.under = set()
            return False
        theta = blade_angle_deg(r, b)
        tilt = edge_tilt_deg(r, b)
        if self.stroke is None:
            self.stroke = Stroke(self.n, self.n, theta, theta)
            self.strokes.append(self.stroke)
            self.miss, self.steep_run, self.corner_run, self.shift = 0, 0.0, 0.0, 0.0
        st = self.stroke
        st.theta_min, st.theta_max = min(st.theta_min, theta), max(st.theta_max, theta)
        now, owner = set(), {}
        for i in np.nonzero(on)[0]:  # a cell under two lanes' samples belongs to the first (worked once)
            now.add(cells[i])
            owner.setdefault(cells[i], self._lane_of[i])
            self.last_cells[self._lane_of[i]] = cells[i]
        new = [set() for _ in range(b.lanes)]
        for c_ in now - self.under:
            new[owner[c_]].add(c_)
        self.under = now
        if not (held and sliding):
            return True
        # in-plane travel of the edge's middle this sub-step, split along the edge and square to it
        d = np.array([dmid[0], dmid[1], 0.0])
        a2 = np.array([along[0], along[1], 0.0])
        a2 /= (np.linalg.norm(a2) or 1.0)
        d_al = float(np.dot(d, a2))
        d_sq = float(np.linalg.norm(d - d_al * a2))
        travel = float(np.linalg.norm(d))
        # gouging
        if theta > L.gouge_deg:
            self.steep_run += travel
        if tilt > L.tilt_max:
            self.corner_run += travel
        if not self.gouged and (self.steep_run >= L.gouge_len or self.corner_run >= L.gouge_len):
            self.gouged = True
            on_pts = pts[on]
            self.gouge_at = (float(on_pts[:, 0].mean()), float(on_pts[:, 1].mean()))
        bite = L.bite_lo <= theta <= L.bite_hi and tilt <= L.tilt_max
        for j in range(b.lanes):
            for (iu, iv) in new[j]:
                st.cells += 1
                self.passes[iu, iv] += 1
                if bite and self.bonded[iu, iv] > 0:
                    take = min(L.bite, self.bonded[iu, iv])
                    self.bonded[iu, iv] -= take
                    self.lane_load[j] += take
                    st.removed += take
                if self.loose[iu, iv] > 0:
                    self.lane_load[j] += self.loose[iu, iv]
                    self.loose[iu, iv] = 0.0
        # transport: sliding along the edge shifts the load between lanes; what passes an end is a windrow
        lane_w = b.width / b.lanes
        self.shift -= d_al / lane_w
        ends = (c - (b.width / 2 + lane_w / 2) * a2, c + (b.width / 2 + lane_w / 2) * a2)
        while self.shift <= -1.0:
            self._drop(ends[0][0], ends[0][1], self.lane_load[0])
            self.lane_load[:-1] = self.lane_load[1:].copy()
            self.lane_load[-1] = 0.0
            self.shift += 1.0
        while self.shift >= 1.0:
            self._drop(ends[1][0], ends[1][1], self.lane_load[-1])
            self.lane_load[1:] = self.lane_load[:-1].copy()
            self.lane_load[0] = 0.0
            self.shift -= 1.0
        # leak round the ends on any travel square to the edge
        f = min(1.0, L.leak * d_sq)
        for j, e in ((0, ends[0]), (b.lanes - 1, ends[1])):
            out = self.lane_load[j] * f
            if out > 0:
                self.lane_load[j] -= out
                self._drop(e[0], e[1], out)
        return True

    def _end_stroke(self) -> None:
        st, L = self.stroke, self.law
        total = float(self.lane_load.sum())
        if total > 1e-12:
            if self.lift_speed > L.fling_speed:
                self.spilled += total
                st.end = "flung"
            else:
                for j in range(self.b.lanes):
                    c = self.last_cells[j]
                    if c is not None and self.lane_load[j] > 0:
                        self.loose[c] += self.lane_load[j]
                    elif self.lane_load[j] > 0:  # never touched with this lane: the nearest lane's cell
                        cs = [x for x in self.last_cells if x is not None]
                        self.loose[cs[0]] += self.lane_load[j]
                st.end = "ridge"
            self.lane_load[:] = 0.0
        self.stroke, self.miss, self.lift_speed = None, 0, 0.0
        self.under = set()
        self.last_cells = [None] * self.b.lanes

    # ---- what the scorer reads ----
    def bonded_total(self) -> float:
        return float(self.bonded.sum())

    def loose_total(self) -> float:
        return float(self.loose.sum())

    def load_total(self) -> float:
        return float(self.lane_load.sum())

    def residue(self) -> float:
        """Soil still on the plate: bonded film plus loose debris (units)."""
        return self.bonded_total() + self.loose_total()

    def total(self) -> float:
        """Conservation check: everything accounted for (units); equals `applied`."""
        return self.residue() + self.load_total() + self.trough + self.spilled

    def blocks(self, k: int = 1):
        """Drawable cells with soil: [(x, y, size, key)], key "film" (bonded, any loose on top) or "loose"."""
        out = []
        c = self.p.cell
        for iu, iv in zip(*np.nonzero((self.bonded > 1e-9) | (self.loose > 1e-9))):
            key = "heavy" if self.bonded[iu, iv] > 1.0 + 1e-9 else ("film" if self.bonded[iu, iv] > 1e-9 else "loose")
            out.append((float(self.X[iu, iv]), float(self.Y[iu, iv]), c, key))
        return out


# -- visuals ------------------------------------------------------------------------------------------------------------

COLORS = {"film": (0.30, 0.22, 0.14), "heavy": (0.12, 0.09, 0.07), "loose": (0.45, 0.33, 0.20)}


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class Tiles:
    """Soil cells drawn as thin dark squares just above the plate with Isaac Lab's VisualizationMarkers. Call
    draw(scrape, origin) after each step. Imported lazily; off with one warning on any error (offline: nothing)."""

    def __init__(self, lift: float = 0.0004, thick: float = 0.0004, path: str = "/Visuals/scrapekit"):
        self.lift, self.thick, self.path = lift, thick, path
        self.markers, self.off, self.shown, self.keys = None, False, None, list(COLORS)

    def draw(self, s: Scrape, origin=(0.0, 0.0, 0.0)) -> None:
        if self.off:
            return
        blocks = s.blocks()
        sig = hash(tuple((round(x, 4), round(y, 4), key) for x, y, _, key in blocks))
        if sig == self.shown:
            return
        try:
            if self.markers is None:
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                protos = {key: sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(_linear(x) for x in COLORS[key]), roughness=0.7)) for key in self.keys}
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path, markers=protos))
            self.shown = sig
            if not blocks:
                self.markers.set_visibility(False)
                return
            o = np.asarray(origin, float)
            tr = [(x + o[0], y + o[1], s.p.z + self.lift + self.thick / 2 + o[2]) for x, y, _, _ in blocks]
            sc = [(sz * 0.96, sz * 0.96, self.thick) for _, _, sz, _ in blocks]
            ids = [self.keys.index(key) for *_, key in blocks]
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32),
                                   orientations=np.tile(np.array([1.0, 0, 0, 0]), (len(blocks), 1)).astype(np.float32),
                                   scales=np.array(sc, np.float32), marker_indices=np.array(ids, int))
        except ImportError:
            self.off = True
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            warnings.warn(f"scrapekit tiles off ({type(e).__name__}: {e}); scoring is unaffected", RuntimeWarning)
            print(f"scrapekit: tiles off ({e!r})", file=sys.stderr, flush=True)
