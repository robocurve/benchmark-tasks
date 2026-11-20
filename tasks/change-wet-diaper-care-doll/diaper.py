"""Reduced position-based (PBD) diaper sheets for the diaper-change task. Pure numpy, no simulator imports.

A diaper is a grid of nodes (rows along its length from the back waist, columns across) joined by distance
constraints: stretch constraints resist extension only (cloth buckles freely), and a few bilateral "bend" links give
the absorbent core its stiffness. The rigid simulator supplies, every control step, the measured gripper TCP poses and
jaw openings and the doll's body shapes; the sheets never push back on the rigid bodies (uncalibrated surrogate).

Rules (all driven by measured gripper state, never by a script phase):
- pinch: a closed jaw (opening < PINCH_CLOSE) whose pad zone holds a boundary or tab node pins up to 3 such nodes at
  their offsets in the jaw frame; the pinch lets go when the jaw opens past PINCH_OPEN or a node is dragged > PINCH_TEAR.
- tab: a pinched tab node held pressed on the landing zone of the same diaper (touching it, within TAB_CONTACT of its
  surface, and pushed PRESS_DEPTH into it toward the body, for PRESS_STEPS control steps in a row) attaches there (hook-and-loop), and stays attached after the jaw lets go;
  released anywhere else it stays loose; a pinch that pulls an attached tab node > TAB_PEEL away peels it.
- load: any node lying on the mat under the doll's pelvis (the part carrying its weight) cannot slide while the pelvis
  rests on it; under the lighter legs and torso it slides with friction.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import os
import numpy as np

PINCH_CLOSE = 0.012  # m jaw opening below which the jaw holds cloth
PINCH_OPEN = 0.022
PINCH_TEAR = 0.035
PINCH_ZONE = np.array([0.014, 0.013, 0.020])  # half extents in the jaw frame (closing axis, finger width, approach)
PINCH_Z0 = 0.004  # pad zone centre past the TCP along the approach axis (fingertips end 13 mm past the TCP)
TAB_CONTACT = 0.004  # m: tab touching the landing zone (two sheet half-thicknesses of 2 mm each meet)
PRESS_STEPS = 5  # 0.25 s at 20 Hz
# The jaw is position-controlled and the sheets push no force back on it, so the press is measured as depth: the
# held tab must be driven at least PRESS_DEPTH past touching (into the soft diaper-on-vinyl stack, toward the body).
# Stand-in for a firm fingertip press of a few newtons, not calibrated (standard hook-and-loop peel tests close the
# sample under an 11 lb / ~49 N roller, ASTM D5170).
PRESS_DEPTH = 0.0015
STATIC_SLIP = 0.0004  # m per substep (about 5 cm/s): below this a touching node does not slide
TAB_PEEL = 0.030
GRAVITY = 9.81


def _rows_cols(nu, nv):
    return np.arange(nu * nv).reshape(nu, nv)


@dataclass
class Sheet:
    name: str
    x: np.ndarray  # (N,3) node positions
    faces: np.ndarray  # (F,3) triangles, wound so the normal points to the OUTER side
    edges: np.ndarray  # (E,2)
    rest: np.ndarray  # (E,)
    bilateral: np.ndarray  # (E,) bool: resists compression too (core bending)
    stiff: np.ndarray  # (E,)
    graspable: np.ndarray  # (N,) bool boundary / tab nodes
    tab: np.ndarray  # (N,) int: 0 none, 1 left tab (+x side when laid inner-up, back half toward +y), 2 right tab
    lz_faces: np.ndarray  # faces of the landing zone (outer side of the front panel)
    uv: np.ndarray  # (N,2) flat material coordinates (u along length from the back waist, v across, m)
    color_in: np.ndarray  # (N,3) inner-side colours
    color_out: np.ndarray  # (N,3) outer-side colours
    thick: float = 0.004
    v: np.ndarray = None
    pins: dict = field(default_factory=dict)  # arm -> list of (node, offset in jaw frame)
    stuck: dict = field(default_factory=dict)  # tab node -> (face, bary)
    press: dict = field(default_factory=dict)  # pinned tab node -> consecutive control steps pressed on the zone
    torn: bool = False
    blocked: set = field(default_factory=set)  # arms already holding another (upper) sheet: they cannot pinch this one
    peak_strain: float = 1.0

    def __post_init__(self):
        self.v = np.zeros_like(self.x)
        self.inv_m = np.ones(len(self.x))
        e = self.edges
        self._deg = np.bincount(e.ravel(), minlength=len(self.x)).astype(float)

    # -- rules ----------------------------------------------------------------------------------------------------
    def pinned_nodes(self):
        return {n for lst in self.pins.values() for n, _ in lst}

    def update_pinches(self, tcp, rot, aperture):
        """tcp (2,3), rot (2,3,3) jaw frames (x closing, y finger width, z approach), aperture (2,) m."""
        for arm in range(len(tcp)):
            if arm in self.pins:
                lst = self.pins[arm]
                tgt = [tcp[arm] + rot[arm] @ off for _, off in lst]
                far = max(np.linalg.norm(self.x[n] - t) for (n, _), t in zip(lst, tgt))
                if aperture[arm] > PINCH_OPEN or far > PINCH_TEAR:
                    released = [n for n, _ in lst]
                    if os.environ.get("DIAPER_DEBUG"):
                        print(f"[pinch] {self.name} arm{arm} release nodes={released} ap={aperture[arm]:.3f} far={far:.3f}", flush=True)
                    del self.pins[arm]
                    self._try_stick(released)
                continue
            if aperture[arm] > PINCH_CLOSE or arm in self.blocked:
                continue
            cand = np.flatnonzero(self.graspable)
            taken = self.pinned_nodes()
            local = (self.x[cand] - tcp[arm]) @ rot[arm]
            local[:, 2] -= PINCH_Z0
            ok = np.all(np.abs(local) < PINCH_ZONE, axis=1)
            if not ok.any():
                continue
            ids = cand[ok]
            d = np.linalg.norm(local[ok], axis=1)
            chosen = [int(n) for n in ids[np.argsort(d)] if int(n) not in taken][:3]
            if not chosen:
                continue
            self.pins[arm] = [(n, (self.x[n] - tcp[arm]) @ rot[arm]) for n in chosen]
            if os.environ.get("DIAPER_DEBUG"):
                print(f"[pinch] {self.name} arm{arm} grab nodes={chosen}", flush=True)

    def _try_stick(self, nodes):
        for n in nodes:
            if self.tab[n] == 0 or n in self.stuck or not len(self.lz_faces):
                continue
            if self.press.get(n, 0) < PRESS_STEPS:
                continue
            f, bary, dist = self._closest_on_faces(self.x[n], self.lz_faces)
            if dist < TAB_CONTACT * 1.5:
                self.stuck[n] = (f, bary)
        for n in nodes:
            self.press.pop(n, None)

    def _pressed(self, n, f, bary, dist, world):
        """Tab node n is on the zone (touching it) and pushed PRESS_DEPTH into it, measured along the body normal:
        its distance to the doll's surface is at most the zone's own distance plus TAB_CONTACT - PRESS_DEPTH."""
        if dist >= TAB_CONTACT or not hasattr(world, "doll_sd"):
            return False
        q = bary @ self.x[self.faces[f]]
        s_tab, s_zone = world.doll_sd(np.vstack([self.x[n], q]))
        return bool(s_tab - s_zone <= TAB_CONTACT - PRESS_DEPTH)

    def _closest_on_faces(self, p, faces):
        best = (None, None, 1e9)
        tri = self.x[self.faces[faces]]  # (K,3,3)
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        # project p onto each triangle plane, clamp barycentrics (approximate closest point)
        v0, v1, v2 = b - a, c - a, p - a
        d00 = (v0 * v0).sum(1); d01 = (v0 * v1).sum(1); d11 = (v1 * v1).sum(1)
        d20 = (v2 * v0).sum(1); d21 = (v2 * v1).sum(1)
        den = np.maximum(d00 * d11 - d01 * d01, 1e-12)
        vv = (d11 * d20 - d01 * d21) / den
        ww = (d00 * d21 - d01 * d20) / den
        uu = 1 - vv - ww
        bary = np.clip(np.stack([uu, vv, ww], 1), 0, 1)
        bary /= bary.sum(1, keepdims=True)
        q = bary[:, :1] * a + bary[:, 1:2] * b + bary[:, 2:] * c
        d = np.linalg.norm(q - p, axis=1)
        k = int(np.argmin(d))
        return int(faces[k]), bary[k], float(d[k])

    def stuck_point(self, n):
        f, bary = self.stuck[n]
        return bary @ self.x[self.faces[f]]

    # -- dynamics ---------------------------------------------------------------------------------------------------
    def step(self, tcp, rot, aperture, world, dt=0.05, substeps=6, iters=12):
        """Advance one control period. ``world`` supplies collide(x, prev, thick) -> (x, contact mask, locked mask)."""
        self.update_pinches(tcp, rot, aperture)
        # peel: a pinched, attached tab node dragged away from its spot comes off
        for arm, lst in self.pins.items():
            for n, off in lst:
                if n in self.stuck and np.linalg.norm(tcp[arm] + rot[arm] @ off - self.stuck_point(n)) > TAB_PEEL:
                    del self.stuck[n]
        h = dt / substeps
        pin_t = {n: tcp[a] + rot[a] @ off for a, lst in self.pins.items() for n, off in lst}
        pin_ids = np.array(sorted(pin_t), dtype=int)
        pin_pos = np.array([pin_t[n] for n in pin_ids]) if len(pin_ids) else np.zeros((0, 3))
        w = self.inv_m.copy()
        w[pin_ids] = 0.0
        e0, e1 = self.edges[:, 0], self.edges[:, 1]
        we = w[e0] + w[e1]
        live = we > 0
        for _ in range(substeps):
            prev = self.x.copy()
            self.v[:, 2] -= GRAVITY * h
            self.v *= 0.985
            self.x = self.x + self.v * h
            if len(pin_ids):
                self.x[pin_ids] = pin_pos
            for it in range(iters):
                d = self.x[e1] - self.x[e0]
                ln = np.linalg.norm(d, axis=1)
                c = ln - self.rest
                act = live & ((c > 0) | self.bilateral)
                s = np.where(act, c / np.maximum(ln, 1e-9) * self.stiff / np.maximum(we, 1e-9), 0.0)
                corr = d * s[:, None]
                n = len(self.x)
                c0, c1 = corr * w[e0, None], -corr * w[e1, None]
                acc = np.stack([np.bincount(e0, c0[:, k], n) + np.bincount(e1, c1[:, k], n) for k in range(3)], 1)
                self.x += acc * (1.6 / np.maximum(self._deg, 1.0))[:, None]
                for n, (f, bary) in self.stuck.items():
                    tri = self.faces[f]
                    q = bary @ self.x[tri]
                    mid = 0.5 * (q + self.x[n])
                    self.x[n] = mid
                    self.x[tri] += (mid - q)[None, :]
                if len(pin_ids):
                    self.x[pin_ids] = pin_pos
                if it % 3 == 2 or it == iters - 1:
                    self.x, contact, locked = world.collide(self.x, prev, self.thick, pin_ids)
            # static friction: a touching node that moved less than STATIC_SLIP this substep stays where it was
            # (cloth on vinyl, mu ~0.6); then collide once more so a moving doll still pushes it out
            stick = contact & (np.linalg.norm(self.x - prev, axis=1) < STATIC_SLIP)
            if len(pin_ids):
                stick[pin_ids] = False
            if stick.any():
                self.x[stick] = prev[stick]
                self.x, contact, locked = world.collide(self.x, prev, self.thick, pin_ids)
            # kinetic friction: contact nodes lose most tangential motion; locked nodes (under the loaded pelvis) keep x,y
            vel = (self.x - prev) / h
            vel[contact] *= 0.25
            self.v = vel
            sp = np.linalg.norm(self.v, axis=1)
            self.v *= np.minimum(1.0, 1.5 / np.maximum(sp, 1e-9))[:, None]
        for n in self.pinned_nodes():
            if self.tab[n] and len(self.lz_faces) and n not in self.stuck:
                f, bary, dist = self._closest_on_faces(self.x[n], self.lz_faces)
                self.press[n] = self.press.get(n, 0) + 1 if self._pressed(n, f, bary, dist, world) else 0
                if self.press[n] >= PRESS_STEPS:  # the hooks engage the loops: attached, even while still held
                    self.stuck[n] = (f, bary)
                    self.press.pop(n, None)
        lens = np.linalg.norm(self.x[e1] - self.x[e0], axis=1)
        strain = float(np.max(lens / self.rest))
        self.peak_strain = max(self.peak_strain, strain)
        if not np.isfinite(self.x).all() or strain > 3.0:
            self.torn = True
        return self.x

    def normals(self):
        tri = self.x[self.faces]
        fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        vn = np.zeros_like(self.x)
        for k in range(3):
            np.add.at(vn, self.faces[:, k], fn)
        return vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)


# -- building diapers -----------------------------------------------------------------------------------------------
WHITE = np.array([0.95, 0.95, 0.93])
CORE = np.array([0.90, 0.92, 0.95])
STRIPE_DRY = np.array([0.95, 0.80, 0.10])  # wetness indicator, dry: yellow
STRIPE_WET = np.array([0.15, 0.40, 0.90])  # turned blue
TAB_COL = np.array([0.20, 0.55, 0.95])
LZ_COL = np.array([0.55, 0.85, 0.55])  # printed landing zone (green band)
CUFF_COL = np.array([0.85, 0.88, 0.92])


def _grid_sheet(name, us, width_fn, nv, place, tab_rows=(), tab_extra=0.0, lz=None, stripe=STRIPE_DRY,
                stiff_core=0.6, thick=0.004, outer=(0.0, 0.0, -1.0), band=0.0, band_k=0.2):
    """Flat sheet: rows at material u values ``us`` (from the back waist), ``nv`` columns spread across width_fn(u).
    ``place(uv) -> xyz`` lays the flat material coordinates into the world."""
    nu = len(us)
    ids = _rows_cols(nu, nv)
    uv = []
    for i, u in enumerate(us):
        half = width_fn(u) / 2 + (tab_extra if i in tab_rows else 0.0)
        for j in range(nv):
            t = -1 + 2 * j / (nv - 1)
            vv = t * half if j in (0, nv - 1) else t * width_fn(u) / 2
            uv.append((u, vv))
    uv = np.array(uv)
    x = np.array([place(p) for p in uv])
    edges, bil, stf = [], [], []

    def add(a, b, bilateral=False, k=1.0):
        edges.append((a, b)); bil.append(bilateral); stf.append(k)

    for i in range(nu):
        for j in range(nv):
            if j + 1 < nv:
                add(ids[i, j], ids[i, j + 1])
            if i + 1 < nu:
                add(ids[i, j], ids[i + 1, j])
            if i + 1 < nu and j + 1 < nv:
                add(ids[i, j], ids[i + 1, j + 1], k=0.7)
                add(ids[i, j + 1], ids[i + 1, j], k=0.7)
            # core stiffness: skip-one bilateral links along the length in the central columns (absorbent pad)
            core = abs(uv[ids[i, j], 1]) < 0.035
            if core and i + 2 < nu:
                add(ids[i, j], ids[i + 2, j], bilateral=True, k=stiff_core)
            if core and j + 2 < nv and abs(uv[ids[i, j + 2], 1]) < 0.035:
                add(ids[i, j], ids[i, j + 2], bilateral=True, k=stiff_core)
            elif band and uv[ids[i, j], 0] <= band and j + 2 < nv:
                add(ids[i, j], ids[i, j + 2], bilateral=True, k=band_k)  # elastic waistband keeps the back's width
    edges = np.array(edges)
    rest = np.linalg.norm(np.array([np.r_[uv[a], 0] - np.r_[uv[b], 0] for a, b in edges]), axis=1)
    faces = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, d = ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]
            faces += [(a, b, c), (a, c, d)]
    faces = np.array(faces)
    tri = x[faces]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]).sum(0)
    if fn @ np.asarray(outer) < 0:  # wind faces so the normal points to the outer side as laid
        faces = faces[:, ::-1].copy()
    graspable = np.zeros(len(uv), bool)
    graspable[ids[0, :]] = graspable[ids[-1, :]] = graspable[ids[:, 0]] = graspable[ids[:, -1]] = True
    tab = np.zeros(len(uv), int)
    for i in tab_rows:
        tab[ids[i, -1]] = 1
        tab[ids[i, 0]] = 2
    cin = np.tile(WHITE, (len(uv), 1))
    cout = np.tile(WHITE, (len(uv), 1))
    core = np.abs(uv[:, 1]) < 0.035
    cin[core] = CORE
    stripe_m = np.abs(uv[:, 1]) < 0.006
    cout[stripe_m] = stripe
    cin[tab > 0] = TAB_COL
    cout[tab > 0] = TAB_COL
    lz_faces = np.zeros(0, int)
    if lz is not None:
        u0, u1, vh = lz
        fu = uv[faces].mean(1)
        lz_faces = np.flatnonzero((fu[:, 0] >= u0) & (fu[:, 0] <= u1) & (np.abs(fu[:, 1]) <= vh))
        m = (uv[:, 0] >= u0 - 1e-6) & (uv[:, 0] <= u1 + 1e-6) & (np.abs(uv[:, 1]) <= vh + 1e-6)
        cout[m] = LZ_COL
    return Sheet(name=name, x=x, faces=faces, edges=edges, rest=rest, bilateral=np.array(bil), stiff=np.array(stf),
                 graspable=graspable, tab=tab, lz_faces=lz_faces, uv=uv, color_in=cin, color_out=cout, thick=thick)


# Newborn diaper, open: 35.5 cm long, back ears 24 cm wide (+2.5 cm hook tabs each side), crotch 10 cm, front 17 cm.
LENGTH = 0.355
TAB_ROWS = (1, 2)  # rows at u = 2 and 4 cm carry the tabs
TAB_EXTRA = 0.025  # m each tab reaches past the 24 cm back


def width_fn(u):
    pts = [(0.0, 0.24), (0.08, 0.24), (0.15, 0.10), (0.22, 0.10), (0.29, 0.17), (LENGTH, 0.17)]
    us, ws = zip(*pts)
    return float(np.interp(u, us, ws))


def clean_diaper(place, stiff_core=0.6):
    us = np.linspace(0.0, LENGTH, 18)
    return _grid_sheet("clean", us, width_fn, 11, place, tab_rows=TAB_ROWS, tab_extra=TAB_EXTRA, band=0.081,
                       lz=(LENGTH - 0.075, LENGTH - 0.01, 0.065), stripe=STRIPE_DRY, stiff_core=stiff_core)


def wet_bundle(place):
    """The used diaper folded in half (front over back, as it was folded under the bottom): one double-weight layer
    17 x 20 cm, wetness stripe blue."""
    us = np.linspace(0.0, 0.17, 9)
    sh = _grid_sheet("wet", us, lambda u: 0.20, 11, place, stripe=STRIPE_WET, stiff_core=0.8, thick=0.008)
    sh.inv_m[:] = 0.5
    sh.graspable[:] = True  # a folded, bunched used diaper can be pinched anywhere, not only at an edge
    # the folded bundle shows its outer (printed) side on both faces
    sh.color_in = sh.color_out.copy()
    return sh
