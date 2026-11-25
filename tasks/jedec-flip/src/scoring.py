"""Scorer of the jedec-flip task: pure functions on measured poses (numpy only).

Trays: A = the loaded tray (parts lid up in its top pockets at the start), B = the empty tray. Correct procedure: B is
stacked on A (its walls over A's stacking lip, chamfers together), the two-tray stack is squeezed at both ends and rolled
180 deg about its long axis, set down, and A (now on top, upside down) is lifted off and set aside. Every package then
lies dead-bug (ball side up) in a cell on B's underside.

Levels (latched in order; read at the end of the trial):
* 0.25  B nested on A: walls over the lip, edges flush (B within 1.0 mm / 0.4 deg of its stacked pose on A, seated
        within 0.6 mm), both trays upright.
* 0.5   the nested stack turned over (B upside down below A) with all 21 packages captive between the trays.
* 1.0   every package flat (< 2 deg), ball side up, in a cell of the upside-down tray B; A lies clear of B (centres
        >= 14 cm apart, A down on the table); both hands >= 3 cm clear of B and of every package; everything still
        for 1 s (ends the trial).
* 0 (veto, at any time): a package outside both trays (fell out of the stack, onto the bench or the floor).
"""

from __future__ import annotations

import math

import numpy as np

import layout as L

NEST_XY = 0.0010
NEST_DZ = 0.0006
NEST_YAW = 0.4
NEST_TILT = 0.5
INVERTED = -0.98  # B's z axis . world z
CELL_XY = 0.0012  # package centre within 1.2 mm of a cell centre (0.5 mm play a side)
REST_DZ = 0.0006  # package resting on the cell floor within 0.6 mm
FLAT_DEG = 2.0
ASIDE_DIST = 0.14
ASIDE_Z = L.TOP + 0.030
HAND_CLEAR = 0.030
STILL_LIN = 0.005  # m/s
STILL_ANG = 3.0  # deg/s
HOLD_S = 1.0
OUT_MARGIN = 0.001
CTRL_DT = 0.05


def chip_centroid(p, r):
    return np.asarray(p) + np.asarray(r) @ np.array([0.0, 0.0, L.CHIP_H / 2])


def in_tray_volume(c, pt, rt, margin=OUT_MARGIN):
    """Point c inside the box spanned by a tray (outline, from below its walls to above its lip)."""
    q = rt.T @ (np.asarray(c) - pt)
    return (
        abs(q[0]) <= L.TRAY_L / 2 + margin
        and abs(q[1]) <= L.TRAY_W / 2 + margin
        and -0.004 <= q[2] <= L.TRAY_H + L.LIP_H + 0.004
    )


def captive(c, pa, ra):
    """Package centroid c inside the cavity between A's pocket floor and B's cell floor of a nested stack, inside the
    pocket grid (A frame)."""
    q = ra.T @ (np.asarray(c) - pa)
    gx, gy = L.GRID_HALF
    return (
        abs(q[0]) < gx
        and abs(q[1]) < gy
        and L.WEB_Z[1] - 0.001 < q[2] < L.STACK_DZ + L.WEB_Z[0] + 0.001
    )


def in_cell(c, rc, pb, rb):
    """Package (centroid c, rotation rc) dead-bug in a cell of the upside-down tray B: (ok, dxy_mm, dz_mm, tilt_deg).
    Tilt is measured against the tray (flat in its pocket); that B itself is upside down is checked by the caller."""
    q = rb.T @ (np.asarray(c) - pb)
    best = min(L.pocket_centers(), key=lambda p: float(np.hypot(q[0] - p[0], q[1] - p[1])))
    dxy = float(np.hypot(q[0] - best[0], q[1] - best[1]))
    # lid down on B's web underside (B frame z = WEB_Z[0], B upside down so the package lies at lower B-z)
    dz = float(q[2] - (L.WEB_Z[0] - L.CHIP_H / 2))
    up = np.asarray(rb).T @ np.asarray(rc)[:, 2]  # package lid axis in B's frame (B upside down: lid along +B z)
    tilt = math.degrees(math.acos(float(np.clip(up[2], -1.0, 1.0))))  # 0 = flat in the cell, lid on the cell floor
    ok = dxy <= CELL_XY and abs(dz) <= REST_DZ and tilt <= FLAT_DEG
    return ok, dxy * 1e3, dz * 1e3, tilt


def box_distance(p, pt, rt, half, z0, z1):
    """Distance from point p to an oriented box (tray frame half-extents in x/y, z from z0 to z1)."""
    q = rt.T @ (np.asarray(p) - pt)
    d = np.array(
        [
            max(abs(q[0]) - half[0], 0.0),
            max(abs(q[1]) - half[1], 0.0),
            max(z0 - q[2], q[2] - z1, 0.0),
        ]
    )
    return float(np.linalg.norm(d))


def measure(poses, tips):
    """poses: {'A': (p, R), 'B': (p, R), 'chips': [(p, R)] * 21}; tips: finger pad tip points (world)."""
    pa, ra = poses["A"]
    pb, rb = poses["B"]
    nx, ny, nz, nyaw, ntilt = L.nest_error(pa, ra, pb, rb)
    nested = (
        abs(nx) <= NEST_XY
        and abs(ny) <= NEST_XY
        and abs(nz) <= NEST_DZ
        and abs(nyaw) <= NEST_YAW
        and ntilt <= NEST_TILT
    )
    cents = [chip_centroid(p, r) for p, r in poses["chips"]]
    out = [
        i
        for i, c in enumerate(cents)
        if not (in_tray_volume(c, pa, ra) or in_tray_volume(c, pb, rb))
    ]
    n_captive = sum(captive(c, pa, ra) for c in cents)
    cells = [in_cell(c, r, pb, rb) for c, (p, r) in zip(cents, poses["chips"])]
    n_cell = sum(1 for c in cells if c[0])
    a_up = float(ra[2, 2])
    b_up = float(rb[2, 2])
    d_ab = float(np.hypot(*(pa - pb)[:2]))
    half = (L.TRAY_L / 2, L.TRAY_W / 2)
    hand = min(
        [box_distance(t, pb, rb, half, -0.003, L.TRAY_H + L.LIP_H + 0.003) for t in tips]
        + [box_distance(t, pa, ra, half, -0.003, L.TRAY_H + L.LIP_H + 0.003) for t in tips]
        + [float(np.linalg.norm(np.asarray(t) - c)) - L.CHIP_SUB[0] / 2 for t in tips for c in cents]
    )
    return dict(
        nest_mm=[round(nx * 1e3, 3), round(ny * 1e3, 3), round(nz * 1e3, 3)],
        nest_yaw_deg=round(nyaw, 3),
        nest_tilt_deg=round(ntilt, 3),
        nested=bool(nested),
        a_up=round(a_up, 4),
        b_up=round(b_up, 4),
        out=out,
        n_captive=int(n_captive),
        n_cell=int(n_cell),
        worst_cell=max(cells, key=lambda c: (not c[0], c[1]))[1:] if cells else None,
        d_ab_mm=round(d_ab * 1e3, 1),
        a_z_mm=round(float(pa[2]) * 1e3, 2),
        hand_mm=round(hand * 1e3, 1),
    )


class Scorer:
    """Latched levels, veto, and the 1 s final hold (one call per control step)."""

    def __init__(self, dt=CTRL_DT):
        self.dt = dt
        self.level = 0.0
        self.veto = None
        self.hold = 0.0
        self.done = False
        self.last = {}
        self.nest_n = 0

    def step(self, m, max_lin, max_ang_deg):
        if m["out"] and self.veto is None:
            self.veto = f"package(s) {m['out']} outside both trays"
        stacked = m["nested"] and m["a_up"] > 0.99 and m["b_up"] > 0.99
        self.nest_n = self.nest_n + 1 if stacked else 0
        if self.level < 0.25 and self.nest_n >= 10:
            self.level = 0.25
        if (
            self.level >= 0.25
            and self.level < 0.5
            and m["nested"]
            and m["b_up"] < INVERTED
            and m["n_captive"] == 21
        ):
            self.level = 0.5
        final = (
            self.level >= 0.5
            and m["n_cell"] == 21
            and m["b_up"] < INVERTED
            and m["d_ab_mm"] >= ASIDE_DIST * 1e3
            and m["a_z_mm"] <= ASIDE_Z * 1e3
            and m["hand_mm"] >= HAND_CLEAR * 1e3
            and max_lin <= STILL_LIN
            and max_ang_deg <= STILL_ANG
        )
        self.hold = self.hold + self.dt if final else 0.0
        if final and self.hold >= HOLD_S - 1e-9:
            self.done = True
        score = 0.0 if self.veto else (1.0 if self.done else self.level)
        self.last = dict(measure=m, level=self.level, score=score, veto=self.veto, hold_s=round(self.hold, 2))
        return score
