"""Soup model for the ladle (no fluids in the simulator). Water's state is kept here and drawn as shapes.

Ladle (Winco LDI-8SH, 8 oz): its bowl is a spherical cap (rim radius A_RIM, depth H_CUP) in the ladle frame, origin at
the centre of its rim, +z out of the bowl, +x towards the handle. The water in it is level, perpendicular to an
effective vertical that leans with the ladle's acceleration through one slosh mode (natural frequency OMEGA,
damping ZETA). The water's volume below a level is counted on a grid of points filling the bowl.

Over the lip: water above the rim's lowest point runs over it at Q = K_LIP * head^2, from that point; it falls under
gravity and lands where it falls. Dipped in the pot with the lip below the soup's surface, the bowl fills (or empties)
at the same law towards the pot's level. When the bowl comes out of the soup its underside carries FILM of soup
(soup running over the lip adds to it), which drips off its lowest point over the next few seconds (TAU_DRIP), landing
straight below.

Pot: a cylinder of inner radius R_POT, the soup level from its volume. Bowl (the soup bowl): a cone frustum.
"""
from __future__ import annotations

import math

import numpy as np

G = 9.81
# ladle
A_RIM, H_CUP = 0.050, 0.046
R_SPH = (A_RIM ** 2 + H_CUP ** 2) / (2 * H_CUP)
K_LIP = 0.5                     # m3/s per m^2 of head, over the lip
K_FILL = 4.0                    # the same, the bowl's mouth dipped under the soup
FILM = 3.0e-6                   # m3 carried out on the underside
TAU_DRIP = 1.6                  # s
CLING = 0.2                     # of what runs over the lip, the share that clings to the outside (up to FILM / 2)
OMEGA, ZETA, SLOSH_GAIN = 18.0, 0.12, 0.8
# pot (stockpot) and bowl (soup bowl), in their own frames: origin at the centre of the inside bottom
R_POT, H_POT = 0.105, 0.085
V_POT0 = 2.4e-3
BOWL_R0, BOWL_R1, BOWL_H = 0.040, 0.078, 0.062   # inside: bottom radius, rim radius, depth
SPLASH_H = 0.15                 # m of free fall into the bowl before splashes reach its rim

_STEP = 0.0025
_g = np.arange(-A_RIM, A_RIM + 1e-9, _STEP)
_X, _Y, _Z = np.meshgrid(_g, _g, np.arange(-H_CUP, 0.0, _STEP) + _STEP / 2, indexing="ij")
_P = np.stack([_X.ravel(), _Y.ravel(), _Z.ravel()], 1)
_c = np.array([0.0, 0.0, R_SPH - H_CUP])          # sphere centre: the cap's bottom is at z = -H_CUP
_P = _P[np.linalg.norm(_P - _c, axis=1) <= R_SPH]
V_CAP = math.pi * H_CUP * (3 * A_RIM ** 2 + H_CUP ** 2) / 6    # the cap's volume to its rim
DV = V_CAP / len(_P)          # each grid point's share
HALF = _STEP / 2
CAPACITY = V_CAP
_RIM = np.array([[A_RIM * math.cos(a), A_RIM * math.sin(a), 0.0] for a in np.linspace(0, 2 * math.pi, 72, False)])
# outside of the bowl, for the lowest point that drips: the cap's surface plus the rim
_OUT = np.concatenate([[(_c + R_SPH * np.array([math.sin(t) * math.cos(p), math.sin(t) * math.sin(p), -math.cos(t)]))
                        for t in np.linspace(0, math.acos((R_SPH - H_CUP) / R_SPH), 8) for p in np.linspace(0, 2 * math.pi, 16, False)],
                       _RIM])


def bowl_r(z):
    return BOWL_R0 + (BOWL_R1 - BOWL_R0) * min(max(z / BOWL_H, 0.0), 1.0)


def bowl_vol(z):
    r = bowl_r(z)
    return math.pi * z / 3 * (BOWL_R0 ** 2 + BOWL_R0 * r + r * r)


def bowl_level(w):
    lo, hi = 0.0, BOWL_H * 1.5
    for _ in range(40):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if bowl_vol(m) < w else (lo, m)
    return (lo + hi) / 2


class Soup:
    def __init__(self):
        self.V = V_POT0          # in the pot
        self.L = 0.0             # in the ladle
        self.W = 0.0             # in the soup bowl
        self.F = 0.0             # film on the ladle's underside
        self.spilt = 0.0         # on the pad, the table, the bowl's rim or its outside
        self.rim = 0.0           # of that, on the bowl's rim (splashes and a stream on the rim)
        self.th = np.zeros(2)    # slosh: lean of the effective vertical
        self.thd = np.zeros(2)
        self.q = 0.0             # flow over the lip now (m3/s)
        self.into = 0.0          # of it, into the soup bowl
        self.pours = 0           # separate pours into the soup bowl (each more than 20 mL)
        self._pour_v, self._pour_gap = 0.0, 99.0
        self.max_tilt = 0.0
        self.was_in = False
        self.cause = {"stream": 0.0, "drip": 0.0, "splash": 0.0}
        self.clock = 0.0
        self.log = {}

    # -- geometry ------------------------------------------------------------------------------------------------
    def up(self):
        n = np.array([self.th[0] * SLOSH_GAIN, self.th[1] * SLOSH_GAIN, 1.0])
        return n / np.linalg.norm(n)

    def level(self, R, p, L):
        """Height (along the effective vertical) of the water surface for a volume L, and of the lowest rim point,
        with that point (world)."""
        n = self.up()
        s = (_P @ R.T + p) @ n
        sr = (_RIM @ R.T + p) @ n
        j = int(np.argmin(sr))
        k = int(L / DV)
        if k <= 0:
            lvl = float(s.min()) - HALF
        elif k >= len(s):
            lvl = float(s.max()) + HALF + (L - CAPACITY) / (math.pi * A_RIM ** 2)
        else:
            lvl = float(np.partition(s, k)[k]) + HALF * (1.0 - 2.0 * (k % 1))
        return lvl, float(sr[j]), R @ _RIM[j] + p

    def capacity(self, R, p):
        n = self.up()
        s = (_P @ R.T + p) @ n
        sr = float(((_RIM @ R.T + p) @ n).min())
        return float((s + HALF < sr).sum() * DV)

    # -- one step ------------------------------------------------------------------------------------------------
    def step(self, R, p, acc, pot_c, bowl_c, bowl_R, dt):
        """R, p: the ladle frame (world); acc: its bowl's acceleration (world); pot_c / bowl_c: the centres of the
        inside bottoms of the pot and the soup bowl (world, both upright); bowl_R its rotation. Returns the stream's
        start point (or None) and the drip point (or None)."""
        self.clock += dt
        # slosh
        th_eq = -np.asarray(acc[:2], float) / G
        thdd = OMEGA ** 2 * (th_eq - self.th) - 2 * ZETA * OMEGA * self.thd
        self.thd = self.thd + thdd * dt
        self.th = self.th + self.thd * dt
        self.max_tilt = max(self.max_tilt, math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))))

        z_pot = pot_c[2] + self.V / (math.pi * R_POT ** 2)
        lvl, lip, lip_p = self.level(R, p, self.L)
        n = self.up()
        cup_bottom = float(min(((_OUT @ R.T + p) @ np.array([0, 0, 1.0]))))
        in_pot = float(np.hypot(*(p[:2] - pot_c[:2]))) < R_POT - 0.01 and cup_bottom < z_pot
        self.q, self.into, stream = 0.0, 0.0, None
        # filling or emptying towards the pot's level while the lip is under it
        lip_z = float(lip_p[2])
        if in_pot and lip_z < z_pot:
            self.F = FILM
            self.was_in = True
            hi, lo = max(z_pot - lip_z, 0.0), max(lvl - lip, 0.0)
            dq = K_FILL * (hi * hi - lo * lo) * dt if self.L < CAPACITY * 1.5 else 0.0
            dq = max(-self.L, min(dq, self.V))
            self.L += dq
            self.V -= dq
            lvl, lip, lip_p = self.level(R, p, self.L)
        else:
            if self.was_in and cup_bottom > z_pot:
                self.was_in = False
            # over the lip
            hh = lvl - lip
            if hh > 0 and self.L > 0:
                q = min(K_LIP * hh * hh, self.L / dt)
                self.q = q
                self.L -= q * dt
                stream = lip_p
                where, land, fall = self.land(lip_p, R, p, pot_c, bowl_c)
                self._put(where, q * dt, fall, "stream")
                self.F = min(max(self.F, 0.5 * FILM), self.F + CLING * q * dt)   # what runs over wets the lip
        # drips off the underside
        drip = None
        if self.F > 0.05e-6 and not (in_pot and lip_z < z_pot):
            dd = self.F * min(1.0, dt / TAU_DRIP)
            self.F -= dd
            pts = _OUT @ R.T + p
            low = pts[int(np.argmin(pts[:, 2]))]
            drip = low
            where, _, fall = self._below(low, pot_c, bowl_c)
            if not (where == "pot" and low[2] < z_pot):
                self._put(where, dd, fall, "drip")
        # count pours into the soup bowl
        if self.into > 0:
            self._pour_v += self.into
            self._pour_gap = 0.0
        else:
            self._pour_gap += dt
            if self._pour_gap > 1.0:
                if self._pour_v > 20e-6:
                    self.pours += 1
                self._pour_v = 0.0
        return stream, drip

    def pours_now(self):
        return self.pours + (1 if self._pour_v > 20e-6 else 0)

    def _below(self, pt, pot_c, bowl_c):
        rp = float(np.hypot(*(pt[:2] - pot_c[:2])))
        rb = float(np.hypot(*(pt[:2] - bowl_c[:2])))
        if rp < R_POT - 0.002:
            return "pot", pt, 0.0
        surf = bowl_c[2] + bowl_level(self.W)
        if rb < BOWL_R1 - 0.003:
            return "bowl", pt, float(pt[2] - surf)
        if rb < BOWL_R1 + 0.006:
            return "rim", pt, 0.0
        return "out", pt, 0.0

    def land(self, lip_p, R, p, pot_c, bowl_c):
        """Where the stream from the lip lands: it leaves the lip outwards (away from the bowl's centre) at 0.6 sqrt(2 g head)
        and falls; judged where it crosses the height of the bowl's rim (or the pot's rim)."""
        out = lip_p - p
        out[2] = 0.0
        nrm = float(np.linalg.norm(out))
        hh = max(0.0, self.level(R, p, self.L)[0] - self.level(R, p, self.L)[1])
        v = (0.6 * math.sqrt(2 * G * hh) * out / nrm) if nrm > 1e-6 else np.zeros(3)
        x = np.asarray(lip_p, float).copy()
        rim_b, rim_p = bowl_c[2] + BOWL_H, pot_c[2] + H_POT
        for _ in range(300):
            xn = x + v * 0.003
            v = v + np.array([0.0, 0.0, -G * 0.003])
            for zz in (rim_p, rim_b):
                if x[2] >= zz > xn[2]:
                    w, pt, f = self._below(xn, pot_c, bowl_c)
                    if (zz == rim_p and w == "pot") or (zz == rim_b and w in ("bowl", "rim")):
                        return w, pt, f + (lip_p[2] - xn[2])
            x = xn
            if x[2] < min(rim_b, rim_p) - 0.02:
                break
        return self._below(x, pot_c, bowl_c)

    def _put(self, where, dv, fall, cause="stream"):
        if where == "pot":
            self.V += dv
        elif where == "bowl":
            sp = dv * min(0.3, max(0.0, 0.15 * (fall - SPLASH_H) / 0.10))
            self.W += dv - sp
            self.into += dv - sp
            self.spilt += sp
            self.rim += sp
            self.cause["splash"] += sp
        else:
            self.spilt += dv
            self.cause[cause] += dv
            key = (cause, where, round(self.clock * 2) / 2)
            self.log[key] = self.log.get(key, 0.0) + dv
            if where == "rim":
                self.rim += dv
