"""Sterilization wrap (one 45 x 45 cm SMS sheet) folded envelope-style round an instrument tray, as a reduced
position-based (XPBD) sheet. NumPy/SciPy only, no task-phase inputs.

The rigid sim supplies, every control step: both TCP poses, both jaw openings, the gripper collision boxes and the
tray's pose. The sheet only reacts to those measured quantities:
  * contact: every gripper collision box pushes sheet points out of itself and carries touching points along by
    friction; the points a gripper touches are recorded (for the border-only rule);
  * a pinch starts when a jaw closes (aperture falls below PINCH_CLOSE, having been open within PINCH_RECENT) with a
    point of the sheet's 2.5 cm border between its pad tips; the top layer's points there are then held;
  * it lets go when the jaw is commanded open or the held cloth is dragged > PINCH_SLIP off the pads.
Material coordinates (a, b) in [-L/2, L/2]^2 along the sheet's hems. The sheet lies as a diamond: its corners point
at the robot (near, a = b = -L/2), away (far, +,+), left (-,+) and right (+,-).
Contacts: table, the tray (rounded box at its measured pose), gripper boxes; layer-on-layer contact keeps the side
two points met on (along the sheet normal) and LAYER apart. Coulomb friction on the table, the tray and the plies.
"""
from __future__ import annotations

import math
import os

import numpy as np
from scipy.spatial import cKDTree

L = 0.45                 # Halyard H300 18 x 18 in (45 x 45 cm)
H = 0.015                # lattice spacing
GSM = 0.060              # kg/m2, SMS polypropylene wrap (uncalibrated)
FABRIC_K = 3.0e3         # N/m per lattice edge, tension only
SHEAR_K = 60.0
BEND_K = 0.05
CREASE_TAU = 1.5        # s: bending rest shape relaxes toward the current shape (creases set, then ease)
THICK = 0.002            # contact skin
LAYER = 0.003            # plies kept this far apart
R_PAIR = 0.016
MU_TABLE, MU_TRAY, MU_PLY, MU_PAD = 0.5, 0.5, 0.6, 0.9
MU_UNDER, DN_UNDER = 0.9, 0.0015   # cloth under the tray (its weight on it)
BORDER = 0.025           # the outer 2.5 cm of the wrap is not sterile
PINCH_CLOSE, PINCH_OPEN, PINCH_SLIP = 0.012, 0.023, 0.04
PINCH_RECENT = 0.8
PIN_BOX = np.array([0.012, 0.013, 0.0085])     # common pad patch, in the gripper frame
PIN_C = np.array([0.0, 0.0, 0.138])            # common pad height; TCP at 0.134
TRAY = np.array([0.15, 0.20, 0.04])            # instrument tray (x depth, y width, height)
TRAY_R = 0.006
UNDER_GAP = 0.005
DAMP = 0.98             # per substep (air and internal damping of the nonwoven)

FLAPS = ("near", "left", "right", "far")
CORNER_AB = {"near": (-1, -1), "left": (-1, 1), "right": (1, -1), "far": (1, 1)}
DIRS = {"near": np.array([-1.0, 0.0]), "far": np.array([1.0, 0.0]), "left": np.array([0.0, 1.0]),
        "right": np.array([0.0, -1.0])}


def _rounded_box_sdf(p, half, r):
    q = np.abs(p) - (half - r)
    outside = np.maximum(q, 0.0)
    d = np.linalg.norm(outside, axis=1) + np.minimum(np.max(q, axis=1), 0.0) - r
    n = np.where(np.linalg.norm(outside, axis=1, keepdims=True) > 1e-9, outside, 0.0)
    inner = np.max(q, axis=1) <= 0.0
    if inner.any():
        k = np.argmax(q[inner], axis=1)
        n_in = np.zeros((inner.sum(), 3))
        n_in[np.arange(len(k)), k] = 1.0
        n[inner] = n_in
    n = n * np.where(p >= 0, 1.0, -1.0)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    return d, n


def flat_xy(ab):
    """Material (a, b) -> flat diamond offsets (x forward, y left) from the sheet centre."""
    a, b = ab[:, 0], ab[:, 1]
    return np.c_[(a + b) / math.sqrt(2), (b - a) / math.sqrt(2)]


class Wrap:
    def __init__(self, table_z):
        self.table_z = table_z
        m = int(round(L / H)) + 1
        self.m = m
        g = -L / 2 + np.arange(m) * H
        A, B = np.meshgrid(g, g, indexing="ij")
        self.ab = np.c_[A.ravel(), B.ravel()]
        n = m * m
        self.n = n
        idx = np.arange(n).reshape(m, m)
        self.ii, self.jj = np.divmod(np.arange(n), m)
        e, kind = [], []
        for i in range(m):
            for j in range(m):
                if i + 1 < m: e.append((idx[i, j], idx[i + 1, j])); kind.append(0)
                if j + 1 < m: e.append((idx[i, j], idx[i, j + 1])); kind.append(0)
                if i + 1 < m and j + 1 < m:
                    e.append((idx[i, j], idx[i + 1, j + 1])); kind.append(1)
                    e.append((idx[i + 1, j], idx[i, j + 1])); kind.append(1)
                if i + 2 < m: e.append((idx[i, j], idx[i + 2, j])); kind.append(2)
                if j + 2 < m: e.append((idx[i, j], idx[i, j + 2])); kind.append(2)
        self.edges = np.array(e)
        self.kind = np.array(kind)
        flat = np.c_[self.ab, np.zeros(n)]
        self.rest = np.linalg.norm(flat[self.edges[:, 1]] - flat[self.edges[:, 0]], axis=1)
        self.rest_flat = self.rest.copy()
        self._bend = self.kind == 2
        k = np.where(self.kind == 0, FABRIC_K, np.where(self.kind == 1, SHEAR_K, BEND_K))
        self.compliance = 1.0 / k
        self._tension_only = self.kind <= 1          # stretch and shear lattice edges resist only tension
        self.inv_m = np.full(n, 1.0 / (GSM * H * H))
        self.colors = self._color()
        f = []
        for i in range(m - 1):
            for j in range(m - 1):
                p, q, r, s = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
                f += [(p, q, r), (p, r, s)]
        self.faces = np.array(f, np.int32)
        self.edge_dist = np.minimum(L / 2 - np.abs(self.ab[:, 0]), L / 2 - np.abs(self.ab[:, 1]))
        self.border = self.edge_dist <= BORDER + 1e-6
        # flap membership (by the tray face the point lies beyond, in the flat layout)
        fx = flat_xy(self.ab)
        hx, hy = TRAY[0] / 2, TRAY[1] / 2
        beyond = np.c_[-fx[:, 0] - hx, fx[:, 1] - hy, -fx[:, 1] - hy, fx[:, 0] - hx]
        self.flap = np.where(beyond.max(axis=1) > 0, np.argmax(beyond, axis=1), -1)   # -1 = base under the tray
        self.corner = {}
        for name, (sa, sb) in CORNER_AB.items():
            self.corner[name] = int(idx[0 if sa < 0 else m - 1, 0 if sb < 0 else m - 1])
        self.x = np.zeros((n, 3)); self.v = np.zeros((n, 3))
        self.fixed = np.zeros(n, bool); self.x_fixed = np.zeros((0, 3))
        self.pins = {}
        self.pair_n = {}
        self.prev_boxes = None
        self.time = 0.0
        self.open_t = np.zeros(2)
        self.prev_cmd = [None, None]
        self.pin_cmd = [None, None]
        self.debug_pinch = [False, False]
        self.tray = (np.zeros(3), 0.0)
        self.touched = np.zeros(n, bool)          # ever touched by a gripper (contact or pinch)
        self.touch_arm = np.zeros((2, n), bool)
        self.peak_strain = 1.0
        self.bad = False

    def _color(self):
        col = -np.ones(len(self.edges), int)
        used = [set() for _ in range(self.n)]
        for i, (a, b) in enumerate(self.edges):
            c = 0
            while c in used[a] or c in used[b]:
                c += 1
            col[i] = c; used[a].add(c); used[b].add(c)
        return [np.flatnonzero(col == c) for c in range(col.max() + 1)]

    # -- state -----------------------------------------------------------------------------------------------------
    def place(self, x, tray_pos, tray_yaw=0.0):
        self.x = np.array(x, float); self.v[:] = 0; self.pins = {}; self.pair_n = {}; self.prev_boxes = None
        self.rest = self.rest_flat.copy()
        self.rest[self._bend] = self._cur_len(self._bend)      # the folded wrap keeps its creases
        self.tray = (np.asarray(tray_pos, float), float(tray_yaw))
        self.touched[:] = False; self.touch_arm[:] = False
        self.peak_strain = 1.0; self.bad = False; self.time = 0.0; self.open_t[:] = 0.0
        self.prev_cmd = [None, None]; self.pin_cmd = [None, None]
        self.debug_pinch = [False, False]

    def to_tray(self, x):
        c, yaw = self.tray
        cs, sn = math.cos(yaw), math.sin(yaw)
        d = x - c
        return np.c_[cs * d[:, 0] + sn * d[:, 1], -sn * d[:, 0] + cs * d[:, 1], d[:, 2]]

    def _tray_sdf(self, x):
        c, yaw = self.tray
        # the tray stands on the wrap's base: its body starts UNDER_GAP above the table
        half = np.array([TRAY[0] / 2, TRAY[1] / 2, (TRAY[2] - UNDER_GAP) / 2])
        p = self.to_tray(x) - [0, 0, UNDER_GAP + half[2]]
        d, n = _rounded_box_sdf(p, half, TRAY_R)
        cs, sn = math.cos(yaw), math.sin(yaw)
        nw = np.c_[cs * n[:, 0] - sn * n[:, 1], sn * n[:, 0] + cs * n[:, 1], n[:, 2]]
        return d, nw

    # -- pinches ---------------------------------------------------------------------------------------------------
    def _update_pins(self, tcp, rot, aperture, jaw_command=None):
        for arm in range(len(tcp)):
            cmd = aperture[arm] if jaw_command is None else jaw_command[arm]
            widening = self.prev_cmd[arm] is not None and cmd > self.prev_cmd[arm] + 1e-5
            self.prev_cmd[arm] = cmd
            if aperture[arm] > PINCH_CLOSE:
                self.open_t[arm] = self.time
                self.debug_pinch[arm] = False
            if arm in self.pins:
                nodes, off = self.pins[arm]
                target = tcp[arm] + off @ rot[arm].T
                slip = np.linalg.norm(self.x[nodes] - target, axis=1).mean()
                if widening or cmd > self.pin_cmd[arm] + 1e-5 or aperture[arm] > PINCH_OPEN or slip > PINCH_SLIP:
                    del self.pins[arm]
                    self.pin_cmd[arm] = None
                continue
            if widening or aperture[arm] > PINCH_CLOSE or self.time - self.open_t[arm] > PINCH_RECENT:
                continue
            local = (self.x - tcp[arm]) @ rot[arm] - (PIN_C - [0, 0, 0.134])
            ok = np.all(np.abs(local) < PIN_BOX, axis=1)
            ok &= np.abs(local[:, 0]) <= aperture[arm] / 2
            for other in self.pins.values():
                ok[other[0]] = False
            if not (ok & self.border).any():
                if os.environ.get("WRAP_LOG") and not self.debug_pinch[arm]:
                    ids = np.flatnonzero(self.border)
                    ids = ids[np.argsort(np.linalg.norm(local[ids], axis=1))[:5]]
                    print(f"[pin-candidates] arm {arm} aperture {aperture[arm]:.4f} "
                          f"nodes {ids.tolist()} local {local[ids].round(4).tolist()}", flush=True)
                    self.debug_pinch[arm] = True
                continue
            # the jaw takes the top ply where it closes: the flap of the highest border point between the pads
            cand = np.flatnonzero(ok & self.border)
            top = cand[np.argmax(self.x[cand, 2])]
            fl = self.flap[top]
            nodes = np.flatnonzero(ok & self.border & (self.flap == fl) & (self.x[:, 2] > self.x[top, 2] - 0.012))
            self.pins[arm] = (nodes, (self.x[nodes] - tcp[arm]) @ rot[arm])
            self.pin_cmd[arm] = cmd
            self.touched[nodes] = True; self.touch_arm[arm, nodes] = True

    # -- contacts --------------------------------------------------------------------------------------------------
    def _normals(self):
        f = self.faces
        fn = np.cross(self.x[f[:, 1]] - self.x[f[:, 0]], self.x[f[:, 2]] - self.x[f[:, 0]])
        nn = np.zeros((self.n, 3))
        for i in range(3):
            np.add.at(nn, f[:, i], fn)
        return nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-12)

    def _layer_normals(self, p):
        """Layer direction at points p: the tray surface's outward normal near the tray, straight up elsewhere."""
        d, n = self._tray_sdf(p)
        far = d > 0.03
        n[far] = [0.0, 0.0, 1.0]
        mid = (d > 0.015) & ~far
        if mid.any():
            s = ((d[mid] - 0.015) / 0.015)[:, None]
            n[mid] = (1 - s) * n[mid] + s * np.array([0.0, 0.0, 1.0])
            n[mid] /= np.linalg.norm(n[mid], axis=1, keepdims=True)
        return n

    def _pairs(self, nrm=None):
        """Sheet points over one another (not lattice neighbours); each pair keeps the order, along the layer
        direction, in which it met."""
        tree = cKDTree(self.x)
        p = tree.query_pairs(R_PAIR * 1.6, output_type="ndarray")
        if len(p):
            far = np.maximum(np.abs(self.ii[p[:, 0]] - self.ii[p[:, 1]]), np.abs(self.jj[p[:, 0]] - self.jj[p[:, 1]]))
            p = p[far > 2]
        if not len(p):
            self.pair_n = {}
            return np.zeros((0, 2), int), np.zeros((0, 3))
        n = self._layer_normals(0.5 * (self.x[p[:, 0]] + self.x[p[:, 1]]))
        dn = ((self.x[p[:, 0]] - self.x[p[:, 1]]) * n).sum(1)
        keys = (p[:, 0].astype(np.int64) * self.n + p[:, 1]).tolist()
        sign = np.where(dn >= 0, 1.0, -1.0)
        old = self.pair_n
        for i, k in enumerate(keys):
            if k in old:
                sign[i] = old[k]
        self.pair_n = dict(zip(keys, sign.tolist()))
        return p, n * sign[:, None]

    def _add(self, idx, dn, nrm, mu, mv=None):
        self._c.append((np.asarray(idx), np.asarray(dn, float), np.asarray(nrm, float), np.full(len(idx), mu),
                        np.zeros((len(idx), 3)) if mv is None else mv))

    def _collide(self, pinned, pairs, pn, boxes, prevb):
        x = self.x
        free = ~pinned
        # table
        low = (x[:, 2] < self.table_z + THICK) & free
        if low.any():
            i = np.flatnonzero(low)
            self._add(i, self.table_z + THICK - x[i, 2], np.tile([0, 0, 1.0], (len(i), 1)), MU_TABLE)
            x[i, 2] = self.table_z + THICK
        # under the tray: the tray's weight holds that cloth (it lies between the tray's feet and the table)
        u = self.to_tray(x)
        under = (np.abs(u[:, 0]) < TRAY[0] / 2) & (np.abs(u[:, 1]) < TRAY[1] / 2) & \
                (u[:, 2] < UNDER_GAP + 0.001) & free
        if under.any():
            i = np.flatnonzero(under)
            self._add(i, np.full(len(i), DN_UNDER), np.tile([0, 0, 1.0], (len(i), 1)), MU_UNDER)
        # tray
        d, nrm = self._tray_sdf(x)
        hit = (d < THICK) & free & ~under
        if hit.any():
            i = np.flatnonzero(hit)
            dep = THICK - d[i]
            x[i] += dep[:, None] * nrm[i]
            self._add(i, dep, nrm[i], MU_TRAY)
        # ply on ply
        if len(pairs):
            a, b = pairs[:, 0], pairs[:, 1]
            dd = x[a] - x[b]
            dn = (dd * pn).sum(1)
            plan = np.linalg.norm(dd - dn[:, None] * pn, axis=1)
            m = (plan < R_PAIR) & (dn < LAYER)
            if m.any():
                a, b, n_, dn = a[m], b[m], pn[m], dn[m]
                wa = (~pinned[a]).astype(float); wb = (~pinned[b]).astype(float)
                ws = np.maximum(wa + wb, 1e-12)
                corr = np.minimum(LAYER - dn, LAYER)
                acc = np.zeros((self.n, 3)); cnt = np.zeros(self.n)
                np.add.at(acc, a, (corr * wa / ws)[:, None] * n_); np.add.at(cnt, a, 1)
                np.add.at(acc, b, -(corr * wb / ws)[:, None] * n_); np.add.at(cnt, b, 1)
                k = cnt > 0
                x[k] += acc[k] / cnt[k][:, None]
                self._add(a, corr * 0.5 * wa / ws, n_, MU_PLY)
                self._add(b, corr * 0.5 * wb / ws, -n_, MU_PLY)
        # gripper boxes
        for bi, (world, lo, hi, arm) in enumerate(boxes):
            Rm, t = world[:3, :3], world[:3, 3]
            l = (x - t) @ Rm
            ins = np.all((l > lo - THICK) & (l < hi + THICK), axis=1) & free
            if not ins.any():
                continue
            i = np.flatnonzero(ins)
            li = l[i]
            pw = prevb[bi] if prevb is not None else world
            pl = (x[i] - pw[:3, 3]) @ pw[:3, :3]
            out_lo = (lo - THICK) - pl; out_hi = pl - (hi + THICK)
            outv = np.maximum(out_lo, out_hi)
            aa, bb = li - (lo - THICK), (hi + THICK) - li
            mm = np.minimum(aa, bb); ax = np.argmin(mm, axis=1); r = np.arange(len(i))
            lower = aa[r, ax] < bb[r, ax]
            was = outv.max(axis=1) > 0
            if was.any():
                k2 = np.argmax(outv[was], axis=1)
                ax[was] = k2
                rr = np.flatnonzero(was)
                lower[was] = out_lo[rr, k2] > out_hi[rr, k2]
            newl = li.copy()
            newl[r, ax] = np.where(lower, lo[ax] - THICK, hi[ax] + THICK)
            delta = (newl - li) @ Rm.T
            x[i] += delta
            nloc = np.zeros_like(newl); nloc[r, ax] = np.where(lower, -1.0, 1.0)
            mv = (newl @ Rm.T + t) - (newl @ pw[:3, :3].T + pw[:3, 3])
            self._add(i, np.linalg.norm(delta, axis=1) + 1e-4, nloc @ Rm.T, MU_PAD, mv / self._sub)
            self.touched[i] = True
            self.touch_arm[arm, i] = True

    def _friction(self, prev, pinned):
        if not self._c:
            return
        idx = np.concatenate([c[0] for c in self._c]); dn = np.concatenate([c[1] for c in self._c])
        n = np.concatenate([c[2] for c in self._c]); mu = np.concatenate([c[3] for c in self._c])
        mv = np.concatenate([c[4] for c in self._c])
        keep = ~pinned[idx]
        idx, dn, n, mu, mv = idx[keep], dn[keep], n[keep], mu[keep], mv[keep]
        if not len(idx):
            return
        order = np.lexsort((-(mu * dn), idx))
        idx, dn, n, mu, mv = idx[order], dn[order], n[order], mu[order], mv[order]
        first = np.r_[True, idx[1:] != idx[:-1]]
        tot = np.zeros(self.n); np.add.at(tot, idx, dn)
        idx, n, mu, mv = idx[first], n[first], mu[first], mv[first]
        dn = tot[idx]
        dx = (self.x[idx] - prev[idx]) - mv
        t = dx - (dx * n).sum(1, keepdims=True) * n
        tl = np.linalg.norm(t, axis=1)
        lim = mu * dn
        f = np.where(tl <= lim, 1.0, lim / np.maximum(tl, 1e-12))
        self.x[idx] -= t * f[:, None]

    # -- main step -------------------------------------------------------------------------------------------------
    def step(self, tcp, rot, aperture, boxes=(), tray=None, dt=0.05, substeps=8, iters=4, jaw_command=None):
        """boxes: list of (world 4x4, lo, hi, arm index)."""
        tcp = np.asarray(tcp, float); rot = np.asarray(rot, float)
        if tray is not None:
            self.tray = (np.asarray(tray[0], float), float(tray[1]))
        self.time += dt
        self._update_pins(tcp, rot, aperture, jaw_command)
        h = dt / substeps
        self._sub = substeps
        a_edge = self.compliance / (h * h)
        E, R = self.edges, self.rest
        w = self.inv_m.copy()
        pinned = self.fixed.copy()
        w[pinned] = 0.0
        targets = []
        for arm, (nodes, off) in self.pins.items():
            targets.append((nodes, self.x[nodes].copy(), tcp[arm] + off @ rot[arm].T))
            pinned[nodes] = True; w[nodes] = 0.0
        cur = [bx[0] for bx in boxes]
        prevb = self.prev_boxes if (self.prev_boxes is not None and len(self.prev_boxes) == len(cur)) else None
        anchors = np.array([p for nodes, _s, _t in targets for p in nodes[:: max(1, len(nodes) // 3)]], int)
        geo = np.linalg.norm(self.ab[None, :, :] - self.ab[anchors][:, None, :], axis=2) * 1.01 + 1e-4 \
            if len(anchors) else None
        x_fixed = self.x[self.fixed].copy()
        for s in range(substeps):
            fr = (s + 1) / substeps
            prev = self.x.copy()
            pairs, pn = self._pairs()
            self.v *= DAMP
            self.v[:, 2] -= 9.81 * h
            self.x += self.v * h
            self.x[self.fixed] = x_fixed
            for nodes, start, tgt in targets:
                self.x[nodes] = start + fr * (tgt - start)
            bx = []
            for bi, (world, lo, hi, arm) in enumerate(boxes):
                wd = world
                if prevb is not None:
                    wd = world.copy(); wd[:3, 3] = prevb[bi][:3, 3] + fr * (world[:3, 3] - prevb[bi][:3, 3])
                bx.append((wd, lo, hi, arm))
            last = prevb if s == 0 else last_bx
            last_bx = [b[0] for b in bx]
            lam = np.zeros(len(E))
            self._c = []            # contacts of all the iterations (their total push sets the friction limit)
            for it in range(iters):
                for c in self.colors:
                    a, b = E[c, 0], E[c, 1]
                    d = self.x[b] - self.x[a]
                    ln = np.linalg.norm(d, axis=1)
                    C = ln - R[c]
                    dl = (-C - a_edge[c] * lam[c]) / np.maximum(w[a] + w[b] + a_edge[c], 1e-12)
                    tens = self._tension_only[c]
                    dl = np.where(tens, np.minimum(lam[c] + dl, 0.0) - lam[c], dl)
                    lam[c] += dl
                    g = d / np.maximum(ln, 1e-12)[:, None]
                    self.x[a] -= (w[a] * dl)[:, None] * g
                    self.x[b] += (w[b] * dl)[:, None] * g
                if geo is not None:
                    for k, p in enumerate(anchors):
                        d = self.x - self.x[p]
                        ln = np.linalg.norm(d, axis=1)
                        over = (ln > geo[k]) & ~pinned
                        if over.any():
                            self.x[over] -= d[over] * (1 - geo[k][over] / ln[over])[:, None]
                self._collide(pinned, pairs, pn, bx, last)
            self._friction(prev, pinned)
            self.v = (self.x - prev) / h
            sp = np.linalg.norm(self.v, axis=1)
            self.v *= np.minimum(1.0, 2.5 / np.maximum(sp, 1e-9))[:, None]
        self.prev_boxes = cur
        b = self._bend
        self.rest[b] += (np.minimum(self._cur_len(b), self.rest_flat[b]) - self.rest[b]) * min(1.0, dt / CREASE_TAU)
        st = self.strain()
        self.peak_strain = max(self.peak_strain, st)
        if not np.isfinite(self.x).all() or st > 3.0:
            self.bad = True
            self.x = np.nan_to_num(self.x)
        return self.x

    def _cur_len(self, m):
        return np.linalg.norm(self.x[self.edges[m, 1]] - self.x[self.edges[m, 0]], axis=1)

    def strain(self, pct=None):
        s = self._tension_only
        ln = np.linalg.norm(self.x[self.edges[s, 1]] - self.x[self.edges[s, 0]], axis=1)
        r = ln / self.rest[s]
        return float(np.max(r) if pct is None else np.percentile(r, pct))
