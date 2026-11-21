"""Tea towel and gel pack as reduced position-based (XPBD) sheets. NumPy only, no task-phase inputs.

The rigid sim supplies, every control step: both TCP poses, both jaw openings and the gripper collision boxes. The
towel and the pack only react to those measured quantities:
  * a free-edge node is pinched only while a jaw is closed on it (aperture < PINCH_CLOSE, node in a 3 x 3 x 4 cm box
    at the TCP); it lets go when the jaw opens (> PINCH_OPEN) or the node is dragged > PINCH_SLIP away.
  * Towel: the two-ply strip of a 50 x 70 cm cotton tea towel folded once (50 x 35 cm, 70 g), on a 2.5 cm lattice,
    tension-only, light bending. Pack: mid-surface of a 15.2 x 29.2 x 2.5 cm, 510 g gel pack on a 2.5 cm lattice,
    stiffer and heavier; it keeps half its thickness (1.25 cm) from everything it touches.
  * Contacts: leg form (capsules), cushion, shoebox, loaf pan, table plane and gripper boxes; the pack pushes the
    towel and the towel pushes back where the towel is backed by the leg, cushion or table. Coulomb-style friction on
    contact. No towel self-collision.
Pack-skin contact is measured, never prevented: any pack node within half its thickness (+1 mm) of the leg's skin
with no towel lying on the skin beneath it.
"""
from __future__ import annotations

import numpy as np

TABLE_Z = 0.7475
TOP = TABLE_Z

# -- scene geometry (world, m; x forward from the arm-base midpoint, y left) ---------------------------------------
ANKLE = np.array([0.40, -0.04, TOP + 0.14])    # centre of the ankle at the outer ankle bone
# leg form (right lower leg) lying on its inner side along y, outer ankle bone facing up: shin and calf run to the
# knee at +y, the foot lies on its inner edge, toes towards the robot (-x)
LEG = [
    (ANKLE + [0.0, 0.0, 0.0], ANKLE + [0.0, 0.16, 0.0], 0.036),        # ankle to lower shin
    (ANKLE + [0.0, 0.16, 0.0], ANKLE + [0.0, 0.36, 0.01], 0.050),      # calf
    (ANKLE + [0.0, -0.035, -0.01], ANKLE + [-0.14, -0.05, -0.015], 0.032),  # heel to toes
]
# bruise over the outer ankle bone, on top
BRUISE_C = ANKLE + [0.0, 0.0, 0.036]
BRUISE_N = np.array([0.0, 0.0, 1.0])
BRUISE_R = 0.025
CUSHION = (np.array([0.25, -0.15, TOP]), np.array([0.55, 0.25, TOP + 0.10]))    # firm foam block 30 x 40 x 10 cm
SHOEBOX = (np.array([0.25, -0.53, TOP]), np.array([0.50, -0.17, TOP + 0.12]))    # closed boot box 25 x 36 x 12 cm
TIN = (np.array([0.1135, 0.2125, TOP]), np.array([0.2465, 0.4475, TOP + 0.07]))   # upturned 9 x 5 in loaf pan

# -- materials -----------------------------------------------------------------------------------------------------
TOWEL_L, TOWEL_W = 0.50, 0.35       # m (local x = 50 cm length, local y = 35 cm edge with the two corners)
TOWEL_MASS = 0.070
TOWEL_H = 0.025
TOWEL_THICK = 0.003                 # two plies of cotton
PACK_L, PACK_W = 0.292, 0.152       # m (local y = 29.2 cm long side, local x = 15.2 cm)
PACK_MASS = 0.510
PACK_H = 0.0253
PACK_HALF = 0.0125                  # half the 2.5 cm thickness
FLANGE = 0.0125                     # heat-sealed end seam width (thin)
MU = 0.8
PINCH_CLOSE, PINCH_OPEN, PINCH_SLIP = 0.012, 0.023, 0.035
PAD_HALF = 0.030                    # cloth within 3 cm of the pad centre line is held (pads are 4.8 cm wide)
COVER_R = 0.8 * TOWEL_H              # towel lattice node stands for the cloth within this radius
TOUCH_TOL = 0.001                   # pack within PACK_HALF + 1 mm of skin = touching


def cap_sdf(p, a, b, r):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    d = p - (a + t[:, None] * ab)
    n = np.linalg.norm(d, axis=1)
    return n - r, d / np.maximum(n, 1e-9)[:, None]


def leg_sdf(p):
    """Signed distance (m) and outward normal from world points to the leg form's skin."""
    p = np.atleast_2d(p)
    best_d, best_n = None, None
    for a, b, r in LEG:
        d, n = cap_sdf(p, a, b, r)
        if best_d is None:
            best_d, best_n = d, n
        else:
            m = d < best_d
            best_d = np.where(m, d, best_d)
            best_n = np.where(m[:, None], n, best_n)
    return best_d, best_n


def box_dist(p, lo, hi):
    p = np.atleast_2d(p)
    return np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0), axis=1)


def bruise_points(n_ring=(1, 6, 12)):
    """Sample points on the bruise (a 5 cm disc on the skin, centre + rings), projected onto the skin."""
    u = np.array([1.0, 0.0, 0.0]); v = np.cross(BRUISE_N, u)
    pts = [BRUISE_C]
    for k, n in enumerate(n_ring[1:], 1):
        r = BRUISE_R * k / (len(n_ring) - 1)
        for i in range(n):
            a = 2 * np.pi * i / n
            pts.append(BRUISE_C + r * (np.cos(a) * u + np.sin(a) * v))
    pts = np.array(pts)
    d, nrm = leg_sdf(pts)
    return pts - d[:, None] * nrm


class Sheet:
    def __init__(self, L, W, mass, h, k, shear, bend, thick, damp=0.995):
        nx, ny = int(round(L / h)), int(round(W / h))
        self.nx, self.ny = nx, ny
        xs, ys = np.linspace(0, L, nx + 1), np.linspace(0, W, ny + 1)
        self.rest_local = np.array([[x, y, 0.0] for x in xs for y in ys])
        tid = np.arange(len(self.rest_local)).reshape(nx + 1, ny + 1)
        self.tid = tid
        e, kd = [], []
        for i in range(nx + 1):
            for j in range(ny + 1):
                if i < nx: e.append((tid[i, j], tid[i + 1, j])); kd.append(0)
                if j < ny: e.append((tid[i, j], tid[i, j + 1])); kd.append(0)
                if i < nx and j < ny:
                    e += [(tid[i, j], tid[i + 1, j + 1]), (tid[i + 1, j], tid[i, j + 1])]; kd += [1, 1]
                if i < nx - 1: e.append((tid[i, j], tid[i + 2, j])); kd.append(2)
                if j < ny - 1: e.append((tid[i, j], tid[i, j + 2])); kd.append(2)
        self.E = np.array(e); kd = np.array(kd); self.kind = kd
        self.n = len(self.rest_local)
        self.R = np.linalg.norm(self.rest_local[self.E[:, 1]] - self.rest_local[self.E[:, 0]], axis=1)
        kk = np.where(kd == 0, k, np.where(kd == 1, shear, bend))
        self.comp = 1.0 / kk
        self.tens = kd == 0
        self.w = np.full(self.n, self.n / mass)
        self.thick = thick
        self.damp = damp
        col = -np.ones(len(e), int); used = [set() for _ in range(self.n)]
        for i, (a, b) in enumerate(e):
            c = 0
            while c in used[a] or c in used[b]:
                c += 1
            col[i] = c; used[a].add(c); used[b].add(c)
        self.colors = [np.flatnonzero(col == c) for c in range(col.max() + 1)]
        f = []
        for i in range(nx):
            for j in range(ny):
                a, b, c, d = tid[i, j], tid[i + 1, j], tid[i + 1, j + 1], tid[i, j + 1]
                f += [(a, b, c), (a, c, d)]
        self.faces = np.array(f, np.int32)
        self.x = np.zeros((self.n, 3)); self.v = np.zeros((self.n, 3))
        self.pinchable = np.zeros(self.n, bool)
        self.rigid_rows = []
        self.pins = {}
        self.contact = np.zeros(self.n, bool)
        self._dn = np.zeros(self.n); self._nrm = np.zeros((self.n, 3))

    def place(self, x):
        self.x = np.array(x, float); self.v[:] = 0; self.pins = {}

    def update_pins(self, tcp, rot, aperture, taken):
        """A closed jaw welds every free-edge node between its pads (pads 4.8 cm wide, so a seam or hem is held across
        2-3 lattice nodes and keeps its direction). It lets go when the jaw opens or the cloth is dragged off."""
        for arm in range(len(tcp)):
            if arm in self.pins:
                held = self.pins[arm]
                tgt = [tcp[arm] + rot[arm] @ off for _, off in held]
                if aperture[arm] > PINCH_OPEN or max(np.linalg.norm(self.x[nd] - t) for (nd, _), t in zip(held, tgt)) > PINCH_SLIP:
                    del self.pins[arm]
                continue
            if aperture[arm] > PINCH_CLOSE or arm in taken:
                continue
            idx = np.flatnonzero(self.pinchable)
            local = (self.x[idx] - tcp[arm]) @ rot[arm]
            ok = (np.abs(local[:, 0]) < 0.015) & (np.abs(local[:, 1]) < PAD_HALF) & (np.abs(local[:, 2]) < 0.020)
            others = {nd for v in self.pins.values() for nd, _ in v}
            cand = [int(c) for c in idx[ok] if int(c) not in others]
            if cand:
                # the pads hold a 5 cm stretch of the edge centred on the point nearest the TCP
                c0 = min(cand, key=lambda c: float(np.linalg.norm(self.x[c] - tcp[arm])))
                near = np.linalg.norm(self.x[idx] - self.x[c0], axis=1) < 0.03
                cand = sorted({c0} | {int(c) for c in idx[near] if int(c) not in others})
                for row in self.rigid_rows:          # a stiff sealed seam: holding it holds all of it
                    if c0 in row:
                        cand = sorted(int(c) for c in row if int(c) not in others)
                self.pins[arm] = [(c, (self.x[c] - tcp[arm]) @ rot[arm]) for c in cand]

    def constraints(self, w, a_e):
        E, R = self.E, self.R
        for c in self.colors:
            a, b = E[c, 0], E[c, 1]
            d = self.x[b] - self.x[a]
            ln = np.linalg.norm(d, axis=1)
            C = ln - R[c]
            dl = (-C - a_e[c] * self.lam[c]) / np.maximum(w[a] + w[b] + a_e[c], 1e-12)
            dl = np.where(self.tens[c], np.minimum(self.lam[c] + dl, 0.0) - self.lam[c], dl)
            self.lam[c] += dl
            g = d / np.maximum(ln, 1e-12)[:, None]
            self.x[a] -= (w[a] * dl)[:, None] * g
            self.x[b] += (w[b] * dl)[:, None] * g


def _push_box(x, lo, hi, th, free, dn, nrm):
    ins = np.all((x > lo - th) & (x < hi + th), axis=1) & free
    if not ins.any():
        return ins
    idx = np.flatnonzero(ins)
    a, b = x[idx] - (lo - th), (hi + th) - x[idx]
    m = np.minimum(a, b); ax = np.argmin(m, axis=1); r = np.arange(len(idx))
    low = a[r, ax] < b[r, ax]
    new = np.where(low, lo[ax] - th, hi[ax] + th)
    dn[idx] += np.abs(new - x[idx, ax])
    nn = np.zeros((len(idx), 3)); nn[r, ax] = np.where(low, -1.0, 1.0)
    nrm[idx] = nn
    x[idx, ax] = new
    return ins


def collide_env(s, th, free, boxes, leg_th=None):
    """Push free nodes of sheet s out of the leg (skin offset leg_th, per node), cushion, boxes, table and gripper
    boxes (contact skin th)."""
    x = s.x
    hit = np.zeros(s.n, bool)
    d, n = leg_sdf(x)
    lt = th if leg_th is None else leg_th
    m = (d < lt) & free
    if m.any():
        lt = np.broadcast_to(lt, d.shape)
        s._dn[m] += lt[m] - d[m]; s._nrm[m] = n[m]
        x[m] += (lt[m] - d[m])[:, None] * n[m]
        hit |= m
    for lo, hi in (CUSHION, SHOEBOX, TIN):
        hit |= _push_box(x, lo, hi, th, free, s._dn, s._nrm)
    low = (x[:, 2] < TOP + th) & free
    s._dn[low] += TOP + th - x[low, 2]; s._nrm[low] = [0, 0, 1]; x[low, 2] = TOP + th
    hit |= low
    for world, lo, hi in boxes:
        loc = (x - world[:3, 3]) @ world[:3, :3]
        ins = np.all((loc > lo - th) & (loc < hi + th), axis=1) & free
        if ins.any():
            idx = np.flatnonzero(ins)
            a, b = loc[idx] - (lo - th), (hi + th) - loc[idx]
            mm = np.minimum(a, b); ax = np.argmin(mm, axis=1); r = np.arange(len(idx))
            loc[idx, ax] = np.where(a[r, ax] < b[r, ax], lo[ax] - th, hi[ax] + th)
            x[idx] = loc[idx] @ world[:3, :3].T + world[:3, 3]
    return hit


def friction(s, prev):
    k = np.flatnonzero(s._dn > 0)
    if not len(k):
        return
    dx = s.x[k] - prev[k]
    n = s._nrm[k]
    t = dx - (dx * n).sum(axis=1, keepdims=True) * n
    tl = np.linalg.norm(t, axis=1)
    lim = MU * s._dn[k]
    f = np.where(tl <= lim, 1.0, lim / np.maximum(tl, 1e-12))
    s.x[k] -= t * f[:, None]


class Drape:
    """Towel + pack, stepped together once per control step."""

    def __init__(self):
        self.towel = Sheet(TOWEL_L, TOWEL_W, TOWEL_MASS, TOWEL_H, k=4e4, shear=300.0, bend=25.0, thick=TOWEL_THICK)
        self.pack = Sheet(PACK_W, PACK_L, PACK_MASS, PACK_H, k=1e5, shear=1e4, bend=40.0, thick=PACK_HALF)
        t, p = self.towel, self.pack
        # towel: the proud free edge (local x = L row, the top layer of the folded pad) is what a jaw can take
        t.pinchable[t.tid[-1, :]] = True
        # pack: the two heat-sealed end seams (local y = 0 and y = L rows)
        p.pinchable[p.tid[:, 0]] = True; p.pinchable[p.tid[:, -1]] = True
        p.rigid_rows = [set(map(int, p.tid[:, 0])), set(map(int, p.tid[:, -1]))]
        self.touched = False          # pack ever on bare skin
        self.touch_where = None
        self.min_pack_skin = 1.0
        self.bad = False

    def place(self, towel_x, pack_x):
        self.towel.place(towel_x); self.pack.place(pack_x)
        self.touched = False; self.touch_where = None; self.min_pack_skin = 1.0; self.bad = False

    def _couple(self, rad):
        t, p = self.towel, self.pack
        lo = p.x.min(axis=0) - rad; hi = p.x.max(axis=0) + rad
        tin = np.flatnonzero(np.all((t.x > lo) & (t.x < hi), axis=1))
        if not len(tin):
            return
        d = p.x[:, None, :] - t.x[None, tin, :]
        dist = np.linalg.norm(d, axis=2)
        ii, jj = np.nonzero(dist < rad)
        if not len(ii):
            return
        tj = tin[jj]
        nn = d[ii, jj] / np.maximum(dist[ii, jj], 1e-9)[:, None]
        corr = rad - dist[ii, jj]
        wt = t.w_eff[tj] * (~t.contact[tj]).astype(float)   # towel backed by the leg/cushion/table does not give
        wp = p.w_eff[ii]
        ws = wp + wt + 1e-12
        np.add.at(p.x, ii, (corr * wp / ws)[:, None] * nn)
        np.add.at(t.x, tj, -(corr * wt / ws)[:, None] * nn)
        np.add.at(p._dn, ii, corr * wp / ws)
        p._nrm[ii] = nn

    def step(self, tcp, rot, aperture, boxes=(), dt=0.05, substeps=10, iters=4):
        tcp = np.asarray(tcp, float); rot = np.asarray(rot, float)
        t, p = self.towel, self.pack
        p.update_pins(tcp, rot, aperture, taken=set())
        t.update_pins(tcp, rot, aperture, taken=set(p.pins))
        h = dt / substeps
        sheets = (t, p)
        for s in sheets:
            s.a_e = s.comp / (h * h)
            s.w_eff = s.w.copy()
            s.free = np.ones(s.n, bool)
            s.targets = {}
            for arm, held in s.pins.items():
                for node, off in held:
                    s.targets[node] = tcp[arm] + rot[arm] @ off
                    s.w_eff[node] = 0.0; s.free[node] = False
        rad = PACK_HALF + TOWEL_THICK + 0.005   # towel nodes 2.5 cm apart: each stands for a 1 cm disc of cloth
        for _ in range(substeps):
            prevs = []
            for s in sheets:
                prevs.append(s.x.copy())
                s.v *= s.damp
                s.v[:, 2] -= 9.81 * h
                s.x += s.v * h
                for node, tg in s.targets.items():
                    s.x[node] = tg
                s.lam = np.zeros(len(s.E))
                s._dn = np.zeros(s.n); s._nrm = np.zeros((s.n, 3))
                s.contact = np.zeros(s.n, bool)
            for _it in range(iters):
                for s in sheets:
                    s.constraints(s.w_eff, s.a_e)
                t.contact = collide_env(t, t.thick, t.free, boxes)
                self._couple(rad)
                cov = self.towel_under(p.x)
                p.contact = collide_env(p, PACK_HALF, p.free, boxes,
                                        leg_th=np.where(cov, PACK_HALF + 2 * TOWEL_THICK, PACK_HALF))
            d, _ = leg_sdf(p.x)
            bare = ~self.towel_under(p.x)
            db = np.where(bare, d, 1.0)
            self.min_pack_skin = min(self.min_pack_skin, float(db.min()))
            if db.min() < PACK_HALF + TOUCH_TOL and not self.touched:
                self.touched = True
                self.touch_where = p.x[int(np.argmin(db))].round(3).tolist()
            for s, pv in zip(sheets, prevs):
                friction(s, pv)
                s.v = (s.x - pv) / h
                # resting contact: a free node touching something and creeping under 2 cm/s stops (static friction)
                rest = (s._dn > 0) & s.free & (np.linalg.norm(s.v, axis=1) < 0.02)
                s.v[rest] = 0.0
                sp = np.linalg.norm(s.v, axis=1)
                s.v *= np.minimum(1.0, 3.0 / np.maximum(sp, 1e-9))[:, None]
        for s in sheets:
            if not np.isfinite(s.x).all():
                self.bad = True
                s.x = np.nan_to_num(s.x)

    def towel_under(self, pts):
        """For each point: is there towel cloth on the skin right beneath it? (a towel node within COVER_R of the
        point's foot on the skin, and itself lying on the skin). The lattice is 2.5 cm, so the cloth between nodes
        counts: COVER_R is 0.8 of the lattice spacing."""
        d, n = leg_sdf(pts)
        foot = pts - d[:, None] * n
        t = self.towel
        dt, _ = leg_sdf(t.x)
        on = np.flatnonzero(dt < 3 * TOWEL_THICK)
        if not len(on):
            return np.zeros(len(pts), bool)
        dd = np.linalg.norm(foot[:, None, :] - t.x[None, on, :], axis=2)
        return dd.min(axis=1) < COVER_R

    # -- measurements --------------------------------------------------------------------------------------------
    def coverage(self, pts=None, layer="towel", reach=0.02):
        """Fraction of bruise sample points covered by the sheet resting over the skin (towel lying on the skin, or
        pack resting on the towel over the skin)."""
        pts = bruise_points() if pts is None else pts
        if layer == "towel":
            dt, _ = leg_sdf(self.towel.x)
            on = self.towel.x[dt < 3 * TOWEL_THICK]
            if not len(on):
                return 0.0
            d = np.linalg.norm(on[None, :, :] - pts[:, None, :], axis=2)
            return float((d.min(axis=1) <= reach).mean())
        dp_skin, _ = leg_sdf(self.pack.x)
        on = self.pack.x[(dp_skin < PACK_HALF + 2 * TOWEL_THICK + 0.008) & self.towel_under(self.pack.x)]
        if not len(on):
            return 0.0
        d = np.linalg.norm(on[None, :, :] - pts[:, None, :], axis=2)
        return float((d.min(axis=1) <= 0.035).mean())
