"""The floss as a task-code string, and the contact rule. Pure NumPy (head frame H, mm, N, s).

The floss is a taut, frictionless string of N_NODES nodes between the pick's two arm tips (its anchors), solved
quasi-statically every physics step: straight wherever it is free, wrapped round whatever it touches (the two central
incisors, the papilla, and the contact's plug). Its tension is T0 plus K_AX per mm of stretch. The string's pull on the
two anchors is applied to the pick as a force and torque, so the robot feels it through the grasp.

The contact between the incisors is a plug that only the floss touches, filling the 0.6 mm gap over the contact zone
(ZONE, |x| <= XS). Coming from above, the floss rests on the plug's top; once it presses down the plug grips it at its
depth z_s. The plug holds while the floss's push F is below F_STATIC and the floss does not slide along itself; past
that, or while it slides along itself at v_s (sawing), the plug gives way at the descent speed v_z that balances
F = F_W + F_F v_z / sqrt(v_z^2 + v_s^2) (friction opposes the total slip). It lets go at the zone's bottom: the floss
has passed. From below it is a one-way hold: the floss goes back up only when it pushes up F_STATIC_UP or more
(sliding along itself does not let it creep up).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

import geometry as g

R = g.FLOSS_R
EPS = 0.01                  # mm: a node this close to an obstacle (beyond the floss's radius) touches it
MAX_STEP = 0.15             # mm: the anchors' largest move per solve (larger jumps are sub-stepped)
UP_DZ_MAX = 0.02            # mm per physics step (4.8 mm/s): the floss's rise through the contact once pulled out
RELEASE_F = 0.05            # N: the floss resting on the contact's edge leaves it when lifted (pulled) this hard
H_FD = 1e-3                 # mm, finite-difference step for normals


# -- obstacles -----------------------------------------------------------------------------------------------------------
_ZT = np.arange(-20.0, 5.0, 0.002)
_AT, _BT, _XT = g.a_of(_ZT), g.b_of(_ZT), g.xc_of(_ZT)
_DAT, _DBT, _DXT = (np.gradient(v, _ZT) for v in (_AT, _BT, _XT))
_NE = g.N_SE


def _tooth41(q):
    """41 as seen by the floss: its crown with the distal half extended to +y (the closed 41/42 contact and the teeth
    beyond it form one wall), so the floss can only go down through the 31/41 gap. Returns the implicit function
    (0 on the side surface, < 0 inside) and its gradient, at points q (M, 3) mm."""
    z = q[:, 2]
    a, b, xc = np.interp(z, _ZT, _AT), np.interp(z, _ZT, _BT), np.interp(z, _ZT, _XT)
    da, db, dxc = np.interp(z, _ZT, _DAT), np.interp(z, _ZT, _DBT), np.interp(z, _ZT, _DXT)
    xr = q[:, 0] - xc
    sx = np.sign(xr)
    ax = np.abs(xr)
    X = ax / a
    yr = np.maximum(g.Y_C - q[:, 1], 0.0)
    Y = yr / b
    Xn1, Yn1 = X ** (_NE - 1), Y ** (_NE - 1)
    S = X * Xn1 + Y * Yn1 + 1e-12
    phi = S ** (1.0 / _NE) - 1.0
    c = S ** (1.0 / _NE - 1.0)
    gx = c * Xn1 * sx / a
    gy = -c * Yn1 / b
    gz = c * (Xn1 * (-sx * dxc / a - ax * da / (a * a)) + Yn1 * (-yr * db / (b * b)))
    return phi, np.stack([gx, gy, gz], axis=1)


def _tooth_sd(q):
    """Signed distance estimate (mm) and outward normal for 41 at q: the side surface (implicit function over its
    gradient) and the incisal edge's plane; inside, the nearer."""
    phi, grad = _tooth41(q)
    gn = np.linalg.norm(grad, axis=1) + 1e-12
    d_side = phi / gn
    d_top = q[:, 2] - g.Z_EDGE
    top = d_top > d_side
    d = np.where(top, d_top, d_side)
    n = np.where(top[:, None], np.array([0.0, 0.0, 1.0]), grad / gn[:, None])
    return d, n


def tooth_sd(p, side: int):
    q = np.array(p, float)
    q[:, 1] *= side
    d, n = _tooth_sd(q)
    n[:, 1] *= side
    return d, n


def gum_sd(p):
    x = p[:, 0]
    s = 2 * g.PAP_K * x
    norm = np.sqrt(1 + s * s)
    d = (p[:, 2] - g.pap_z(x)) / norm
    n = np.stack([s / norm, np.zeros_like(x), 1 / norm], axis=1)
    return d, n


NAMES = ("t41", "t31", "gum")


BOX = ((-8.0, 8.0), (-16.0, 16.0), (-14.0, 4.0))   # mm: the floss meets the teeth and gum only in here (the lower
                                                  # front teeth); elsewhere (the pick in the air) it is free


def sd_all(p):
    """Distances (M, 3) and normals (M, 3, 3) to 41, 31 and the gum (far for points outside BOX)."""
    m = len(p)
    q = np.concatenate([p, p * np.array([1.0, -1.0, 1.0])])
    d, n = _tooth_sd(q)
    n2 = n[m:].copy()
    n2[:, 1] *= -1
    dg, ng = gum_sd(p)
    D = np.stack([d[:m], d[m:], dg], axis=1)
    out = np.zeros(m, bool)
    for i, (lo, hi) in enumerate(BOX):
        out |= (p[:, i] < lo) | (p[:, i] > hi)
    D[out] = 1e3
    return D, np.stack([n[:m], n2, ng], axis=1)


def in_region(P):
    return (np.abs(P[:, 0]) <= g.XS) & (np.abs(P[:, 1]) <= g.YS)


def seg_dist(pts, P):
    """Distance from each point (M, 3) to the polyline P (N, 3), and the index of the nearest segment."""
    a, b = P[:-1], P[1:]
    ab = b - a
    L2 = np.maximum((ab * ab).sum(1), 1e-12)
    ap = pts[:, None, :] - a[None]
    t = np.clip((ap * ab[None]).sum(2) / L2[None], 0.0, 1.0)
    c = a[None] + t[..., None] * ab[None]
    d = np.linalg.norm(pts[:, None, :] - c, axis=2)
    k = d.argmin(1)
    return d[np.arange(len(pts)), k], k, c[np.arange(len(pts)), k]


class Floss:
    def __init__(self, n: int = g.N_NODES):
        self.n = n
        self.reset()

    def reset(self) -> None:
        self.P = None
        self.mode = "above"         # above / in / below the contact zone
        self.zs = g.ZONE[1]         # the plug's grip height while "in"
        self.passed = False
        self.T = g.T0
        self.F_plug = 0.0           # N, + = the floss pushing down on the plug
        self.F_A = np.zeros(3)
        self.F_B = np.zeros(3)
        self.on = {k: 0.0 for k in NAMES + ("plug",)}   # N, normal force on each obstacle
        self.touch = {k: False for k in NAMES}
        self.v_z = 0.0
        self.v_s = 0.0
        self.unbounded = False

    # -- the string -------------------------------------------------------------------------------------------------
    @staticmethod
    def _plug(inner, pin):
        """Apply the plug to the interior nodes in place; returns the mask of nodes it holds."""
        if pin is None:
            return np.zeros(len(inner), bool)
        reg = in_region(inner)
        kind, z = pin
        if kind == "floor":
            hit = reg & (inner[:, 2] <= z + EPS)
        elif kind == "ceil":
            hit = reg & (inner[:, 2] >= z - EPS)
        else:
            hit = reg
        inner[hit, 2] = z
        return hit

    @staticmethod
    def _push(inner):
        """Push the interior nodes out of the obstacles in place; returns which obstacles each touches."""
        d, n = sd_all(inner)
        pen = R - d
        touch = pen >= -EPS
        for k in range(3):
            m = pen[:, k] > 0
            if m.any():
                inner[m] += pen[m, k, None] * n[m, k]
        return touch

    def _straighten(self, P, fixed):
        """Free runs of nodes on straight lines between the nearest fixed nodes (anchors and contacts)."""
        n = len(P)
        idx = np.arange(n)
        prev = np.maximum.accumulate(np.where(fixed, idx, 0))
        nxt = np.minimum.accumulate(np.where(fixed, idx, n - 1)[::-1])[::-1]
        span = np.maximum(nxt - prev, 1)
        t = (idx - prev) / span
        out = P[prev] + t[:, None] * (P[nxt] - P[prev])
        return np.where(fixed[:, None], P, out)

    def solve(self, A, B, pin, iters: int = 5):
        n = self.n
        if self.P is None:
            self.P = np.linspace(A, B, n)
        else:
            # a large jump of the anchors (the pick knocked) is followed in sub-steps of at most MAX_STEP, so the string
            # slides round the teeth rather than tunnelling through them
            jump = max(np.linalg.norm(A - self.P[0]), np.linalg.norm(B - self.P[-1]))
            if jump > MAX_STEP:
                A0, B0 = self.P[0].copy(), self.P[-1].copy()
                k = int(np.ceil(jump / MAX_STEP))
                for i in range(1, k):
                    self.solve(A0 + (A - A0) * i / k, B0 + (B - B0) * i / k, pin, iters)
            s = np.linspace(0.0, 1.0, n)[:, None]
            self.P = self.P + (1 - s) * (A - self.P[0]) + s * (B - self.P[-1])
        P = self.P
        P[0], P[-1] = A, B
        for _ in range(iters):
            Q = P.copy()
            Q[1:-1] = 0.5 * (P[:-2] + P[2:])
            inner = Q[1:-1]
            touch = self._push(inner)
            held = self._plug(inner, pin)
            fixed = np.r_[True, touch.any(1) | held, True]
            Q = self._straighten(Q, fixed)
            inner = Q[1:-1]
            self._push(inner)
            self._plug(inner, pin)
            P = Q
        self.P = P
        return P

    def forces(self, P, pin):
        """Tension, the pull on each anchor, the normal force on each obstacle and on the plug. At each interior node
        the string turns by u_in - u_out; T times that is what the obstacles there push on the floss."""
        seg = np.diff(P, axis=0)
        L = np.linalg.norm(seg, axis=1)
        u = seg / np.maximum(L, 1e-9)[:, None]
        rest = np.linalg.norm(P[-1] - P[0])
        T = g.T0 + g.K_AX * max(0.0, L.sum() - max(rest, g.SPAN))
        f = T * (u[:-1] - u[1:])
        inner = P[1:-1]
        d, nrm = sd_all(inner)
        c = d - R <= EPS
        plug = np.zeros(len(inner), bool)
        if pin is not None:
            plug = in_region(inner) & (np.abs(inner[:, 2] - pin[1]) <= EPS)
        tot = np.zeros(3)
        pap = 0.0                       # the gum's push on the floss over the pressure film strip
        disc = (np.abs(inner[:, 1]) <= g.STRIP_W / 2) & (np.abs(inner[:, 0]) <= g.STRIP_L / 2)
        F_plug = 0.0
        k = c.sum(1) + plug
        one = k == 1
        # one contact: the force along its normal
        for j in range(3):
            m = one & c[:, j]
            if m.any():
                fn = np.maximum((f[m] * nrm[m, j]).sum(1), 0.0)
                tot[j] += fn.sum()
                if j == 2:
                    pap += fn[disc[m]].sum()
        mp = one & plug
        if mp.any():
            F_plug += f[mp, 2].sum()
        # several: least squares on their normals
        for i in np.flatnonzero(k > 1):
            who = list(np.flatnonzero(c[i]))
            basis = [nrm[i, j] for j in who] + ([np.array([0.0, 0.0, 1.0])] if plug[i] else [])
            lam = np.linalg.lstsq(np.stack(basis, axis=1), f[i], rcond=None)[0]
            for q, j in enumerate(who):
                tot[j] += max(lam[q], 0.0)
                if j == 2 and disc[i]:
                    pap += max(lam[q], 0.0)
            if plug[i]:
                F_plug += lam[-1]
        if pin is not None and pin[0] == "floor":
            F_plug = max(F_plug, 0.0)
        elif pin is not None and pin[0] == "ceil":
            F_plug = min(F_plug, 0.0)
        on = {NAMES[j]: float(tot[j]) for j in range(3)}
        on["pap"] = float(pap)
        touch = {NAMES[j]: bool(c[:, j].any()) for j in range(3)}
        return T, T * u[0], -T * u[-1], on, touch, float(F_plug)

    # -- one physics step ---------------------------------------------------------------------------------------------
    def _pin(self):
        if self.mode == "above":
            return ("floor", g.ZONE[1])
        if self.mode == "below":
            return ("ceil", g.ZONE[0])
        return ("pin", self.zs)

    def _plug_stiffness(self, P) -> float:
        """dF/dz of the floss's push on the plug: the two spans' geometric stiffness plus the stretch."""
        reg = in_region(P)
        if not reg.any():
            return 1.0
        xs = P[reg, 0]
        LA = max(xs.min() - P[0, 0], 0.5)
        LB = max(P[-1, 0] - xs.max(), 0.5)
        return self.T * (1 / LA + 1 / LB) + 0.5

    def step(self, A, B, v_slide: float, dt: float):
        """Advance one physics step. A, B: the anchors (mm); v_slide: the floss's speed along itself at the contact
        (mm/s). Returns the solved nodes."""
        pin = self._pin()
        P = self.solve(A, B, pin)
        T, F_A, F_B, on, touch, F = self.forces(P, pin)
        self.T, self.F_A, self.F_B, self.touch = T, F_A, F_B, touch
        self.v_s = v_slide
        self.unbounded = False
        self.v_z = 0.0
        if self.mode == "above" and F > 1e-6:
            self.mode, self.zs = "in", g.ZONE[1]
        elif self.mode == "below" and F < -1e-6:
            self.mode, self.zs = "in", g.ZONE[0]
        if self.mode == "in":
            at_top, at_bot = self.zs >= g.ZONE[1] - 1e-9, self.zs <= g.ZONE[0] + 1e-9
            if (at_top and F < -RELEASE_F) or (at_bot and F > RELEASE_F):
                self.mode = "above" if at_top else "below"
            else:
                down = F > 0
                Fm = abs(F)
                F_static, F_w, F_f = ((g.F_STATIC_DN, g.F_W_DN, g.F_F_DN) if down
                                      else (g.F_STATIC_UP, g.F_W_UP, g.F_F_UP))
                vs = abs(v_slide) if abs(v_slide) >= g.VS_DEADBAND else 0.0
                if not down:
                    vs = 0.0        # from below the contact holds one way: sliding does not let the floss creep back up
                vz, unb = 0.0, False
                if Fm <= F_w:
                    vz = 0.0
                elif vs > 0:
                    r = (Fm - F_w) / F_f
                    if r < 1.0:
                        vz = vs * r / math.sqrt(1 - r * r)
                    else:
                        unb = True
                elif Fm >= F_static:
                    unb = True
                if unb:
                    dz = (Fm - (F_w + F_f)) / self._plug_stiffness(P) + 1e-4
                    if not down:
                        dz = min(dz, UP_DZ_MAX)     # pulled out: the floss rides up through the contact, not in a jump
                else:
                    dz = vz * dt
                self.unbounded, self.v_z = unb, vz
                if dz > 0:
                    self.zs += -dz if down else dz
                    if down and self.zs <= g.ZONE[0]:
                        self.mode, self.zs = "below", g.ZONE[0]
                    elif not down and self.zs >= g.ZONE[1]:
                        self.mode, self.zs = "above", g.ZONE[1]
        if self.mode == "below":
            self.passed = True
            # the floss can only leave the gap back up through the contact; if it is no longer between the teeth at
            # all (dragged clear), it is not below the contact
            between = (np.abs(P[:, 0]) <= 3.5) & (np.abs(P[:, 1]) <= 1.5) & (P[:, 2] < g.ZONE[0] + 0.1)
            if not between.any():
                self.mode = "above"
        on["plug"] = F
        self.F_plug = F
        self.on = on
        return P


LAND_X, LAND_Y = 1.5, 1.5   # mm: the floss "between the teeth", over the papilla (where it lands on the gum)


class Rules:
    """The floss-side scoring state, advanced every physics step: snap, wipe, the floss's share of the press, the
    floss's load."""

    def __init__(self, substeps_snap: int = 6):
        self.k = substeps_snap
        from geometry import band41
        pts, nrm, cell = band41()
        self.band = {"t41": pts, "t31": pts * np.array([1.0, -1.0, 1.0])}
        self.band_n = {"t41": nrm, "t31": nrm * np.array([1.0, -1.0, 1.0])}
        self.cell = cell
        self.n_cells = int(cell.max()) + 1
        self.reset()

    ALT = (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2)   # diagnostic: coverage at other F_wipe values (does not affect the score)

    def reset(self) -> None:
        self.wiped = {k: np.zeros(len(v), bool) for k, v in self.band.items()}
        self.wiped_alt = {f: {k: np.zeros(len(v), bool) for k, v in self.band.items()} for f in self.ALT}
        self.low = deque(maxlen=self.k + 1)
        self.gum_touch = False
        self.snap = None              # (the floss's force on the film strip, N) once it reaches F_MARK: a mark
        self.pap_peak = 0.0           # N: the floss's largest force on the film strip
        self.landings = []            # (descent mm over the 25 ms before each touch of the gum)
        self.load_max = 0.0
        self.failed = False
        self.f_down = 0.0
        self.f_down_max_below = 0.0
        self.peak_drop = 0.0          # mm: the largest 25 ms descent of the floss between the teeth while it is in or
                                      # below the contact zone (the landing-speed margin, even with no gum touch)

    def cells(self, w: np.ndarray) -> np.ndarray:
        """Which of the band's cells count as wiped: at least 3 of a cell's 4 points."""
        return np.bincount(self.cell, weights=w.astype(float), minlength=self.n_cells) >= 3

    def coverage(self) -> dict:
        return {k: float(self.cells(v).mean()) for k, v in self.wiped.items()}

    def step(self, fl: Floss, P, vel_at):
        """vel_at(points (M, 3) mm) -> the floss material's velocity there (mm/s): the pick's rigid motion."""
        between = (np.abs(P[:, 0]) <= LAND_X) & (np.abs(P[:, 1]) <= LAND_Y)
        z_low = float(P[between, 2].min()) if between.any() else float("nan")
        if not between.any():
            self.low.clear()
        else:
            self.low.append(z_low)
        touch = fl.touch["gum"]
        if touch and not self.gum_touch and len(self.low) == self.low.maxlen and not math.isnan(z_low):
            drop = self.low[0] - z_low
            self.landings.append(drop)
            # (diagnostic: the landing speed; the snap rule is the floss's force on the film strip, below)
        self.gum_touch = touch
        pap = fl.on.get("pap", 0.0)     # the floss's force on the film strip this physics step
        self.pap_peak = max(self.pap_peak, pap)
        if pap >= g.F_MARK and self.snap is None:
            self.snap = pap
        if fl.mode != "above" and len(self.low) == self.low.maxlen and not math.isnan(z_low):
            self.peak_drop = max(self.peak_drop, self.low[0] - z_low)
        load = float(np.linalg.norm(fl.F_A + fl.F_B))
        self.load_max = max(self.load_max, load)
        if load > g.FLOSS_FAIL:
            self.failed = True
        self.f_down = float((fl.F_A + fl.F_B)[2])       # the floss pushes the head down as hard as it lifts the pick
        for k in ("t41", "t31"):
            f_on = fl.on.get(k, 0.0)
            if f_on < min(g.F_WIPE, min(self.ALT)):
                continue
            pts = self.band[k]
            d, _, c = seg_dist(pts, P)
            near = d - R <= g.WIPE_TOL
            if not near.any():
                continue
            v = vel_at(c[near])
            nrm = self.band_n[k][near]
            vt = v - (v * nrm).sum(1, keepdims=True) * nrm
            ok = np.linalg.norm(vt, axis=1) >= g.WIPE_SPEED
            idx = np.flatnonzero(near)[ok]
            if f_on >= g.F_WIPE:
                self.wiped[k][idx] = True
            for thr, w in self.wiped_alt.items():
                if f_on >= thr:
                    w[k][idx] = True
