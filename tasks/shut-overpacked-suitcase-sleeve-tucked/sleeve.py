"""Reduced position-based cloth sheet for the jumper sleeve (pure NumPy, no simulator imports).

The rigid simulator supplies, every 20 Hz control step, the measured TCP poses and finger openings of both arms and the
poses of the colliding rigid parts (case walls, packing cubes, jumper, lid, latches). The sheet does not push back on
them (one-way coupling). A closed jaw holds a sleeve EDGE node that lies between its fingers when it closes (contact-
triggered pinch, the jaw's measured opening only); opening the jaw or pulling the node 3.5 cm off releases it.

Uncalibrated surrogate: distance + shear constraints on a 1.25 cm grid, no bending stiffness, no self-collision.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

DX = 0.0125
CLOSED = 0.006  # |joint7| below this = jaw closed (fingers meet: nothing in PhysX stops them on the cloth)
OPENED = 0.0115  # |joint7| above this = jaw opened again
PULL_OFF = 0.035
MARGIN = 0.003  # half thickness of the knit
GATHER = 0.040  # |closing-axis offset| of cloth a closing jaw gathers (open pads ~4.5 cm from the centre)


@dataclass
class Pinch:
    node: np.ndarray  # ids of the nodes held between the pads
    offset: np.ndarray  # their positions in the TCP frame when pinched


def obb(center, rot, half):
    return (np.asarray(center, float), np.asarray(rot, float), np.asarray(half, float))


class Sleeve:
    def __init__(self, path_pts, y0: float, ncol: int, root_rows: int = 2):
        """path_pts: (K,3) polyline the sleeve's centre line starts on, from the root (sewn into the jumper) to the cuff;
        the sheet spans y0 .. y0 + (ncol-1)*DX across it."""
        p = np.asarray(path_pts, float)
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(seg)]
        self.nr = int(s[-1] // DX) + 1
        self.nc = ncol
        rows = []
        for i in range(self.nr):
            si = i * DX
            k = min(np.searchsorted(s, si, side="right") - 1, len(seg) - 1)
            t = (si - s[k]) / max(seg[k], 1e-9)
            rows.append(p[k] + t * (p[k + 1] - p[k]))
        rows = np.array(rows)
        ys = y0 + DX * np.arange(ncol)
        self.x = np.zeros((self.nr, ncol, 3))
        self.x[:, :, 0] = rows[:, None, 0]
        self.x[:, :, 1] = ys[None, :]
        self.x[:, :, 2] = rows[:, None, 2]
        self.x = self.x.reshape(-1, 3)
        self.v = np.zeros_like(self.x)
        self.root_rows = root_rows
        self.root_ids = np.arange(root_rows * ncol)
        self.root_local = None  # set by attach_root
        self.pins: dict[int, Pinch] = {}
        self.pinch_events = 0
        self.bad = False
        ids = np.arange(self.nr * ncol).reshape(self.nr, ncol)
        groups = []
        for par in (0, 1):
            groups.append((ids[par:-1:2, :].ravel(), ids[par + 1::2, :].ravel(), DX, 1.0))
            groups.append((ids[:, par:-1:2].ravel(), ids[:, par + 1::2].ravel(), DX, 1.0))
        for par in (0, 1):
            for col in (0, 1):
                a = ids[par:-1:2, col:-1:2]
                b = ids[par + 1::2, col + 1::2]
                groups.append((a.ravel(), b.ravel(), DX * np.sqrt(2), 0.5))
                a = ids[par:-1:2, col + 1::2]
                b = ids[par + 1::2, col:-1:2]
                n = min(a.shape[1], b.shape[1])
                groups.append((a[:, :n].ravel(), b[:, :n].ravel(), DX * np.sqrt(2), 0.5))
        self.groups = [g for g in groups if len(g[0])]
        edge = np.unique(np.r_[ids[:, 0], ids[:, -1], ids[-1, :]])
        self.edge = edge[edge >= root_rows * ncol]
        f = []
        for i in range(self.nr - 1):
            for j in range(ncol - 1):
                a, b, c, d = ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]
                f += [(a, b, c), (a, c, d)]
        self.faces = np.asarray(f, np.int32)

    # -- root sewn into the jumper body -------------------------------------------------------------------------------
    def attach_root(self, body_pos, body_rot=np.eye(3)):
        self.root_local = (self.x[self.root_ids] - body_pos) @ body_rot

    def root_targets(self, body_pos, body_rot=np.eye(3)):
        return self.root_local @ body_rot.T + body_pos

    # -- pinches --------------------------------------------------------------------------------------------------------
    def update_pinch(self, tcp, rot, opening):
        for arm in range(len(tcp)):
            if arm in self.pins:
                p = self.pins[arm]
                tgt = tcp[arm] + p.offset @ rot[arm].T
                if opening[arm] > OPENED or np.max(np.linalg.norm(self.x[p.node] - tgt, axis=1)) > PULL_OFF:
                    del self.pins[arm]
                continue
            if opening[arm] > CLOSED:
                continue
            # the cloth the closing jaw swept between its pads (TCP frame: x closing axis, y pad width, z approach;
            # pads end 13 mm past the TCP; open pads 4.5 cm either side). The jaw must close on a free edge of the
            # sleeve; it then holds that cloth gathered between its pads.
            loc = (self.x - tcp[arm]) @ rot[arm]
            good = (np.abs(loc[:, 0]) < GATHER) & (np.abs(loc[:, 1]) < 0.014) & (loc[:, 2] > -0.006) & (loc[:, 2] < 0.022)
            good[self.root_ids] = False
            for q in self.pins.values():
                good[q.node] = False
            if not good[self.edge].any():
                continue
            ids = np.flatnonzero(good)
            off = loc[ids].copy()
            off[:, 0] *= 0.004 / GATHER  # bunched between the closed pads
            self.pins[arm] = Pinch(ids, off)
            self.pinch_events += 1

    # -- collisions -----------------------------------------------------------------------------------------------------
    @staticmethod
    def _push_out(x, prev, boxes, skip, touched):
        """Keep nodes out of the boxes. A node that got inside is put back on the face it came in through (the side
        its previous position was on), so thin walls are not tunnelled by a pulled sheet."""
        for c, R, h in boxes:
            loc = (x - c) @ R
            hh = h + MARGIN
            inside = np.all(np.abs(loc) < hh, axis=1)
            inside[skip] = False
            idx = np.flatnonzero(inside)
            if not len(idx):
                continue
            l = loc[idx]
            lp = (prev[idx] - c) @ R
            pen = hh - np.abs(l)
            was_out = np.abs(lp) >= hh - 1e-6
            pen_sel = np.where(was_out, pen, pen + 1.0)  # prefer axes it was outside of before
            ax = np.argmin(pen_sel, axis=1)
            r = np.arange(len(idx))
            side = np.where(was_out[r, ax], np.sign(lp[r, ax] + 1e-12), np.sign(l[r, ax] + 1e-12))
            l[r, ax] = side * hh[ax]
            x[idx] = l @ R.T + c
            touched[idx] = True

    def step(self, tcp, rot, opening, boxes, root_pos, root_rot=np.eye(3), floor=0.0, dt=0.05, substeps=8, iters=10):
        """boxes: list of obb(center, rot(3x3, columns = box axes in world), half extents)."""
        self.update_pinch(tcp, rot, opening)
        h = dt / substeps
        fixed = np.zeros(len(self.x), bool)
        fixed[self.root_ids] = True
        pin_ids = np.zeros(0, int)
        pin_t = np.zeros((0, 3))
        for a, p in self.pins.items():
            pin_ids = np.r_[pin_ids, p.node]
            pin_t = np.vstack([pin_t, tcp[a] + p.offset @ rot[a].T])
        fixed[pin_ids] = True
        w = (~fixed).astype(float)
        root_t = self.root_targets(root_pos, root_rot)
        lo_c = self.x.min(axis=0) - 0.04
        hi_c = self.x.max(axis=0) + 0.04
        near = []
        for c, R, hb in boxes:  # broad phase: only boxes near the sheet this step
            e = np.abs(R) @ hb
            if np.all(c + e >= lo_c) and np.all(c - e <= hi_c):
                near.append((c, R, hb))
        boxes = near
        for _ in range(substeps):
            old = self.x.copy()
            self.v *= 0.985
            self.v[:, 2] -= 9.81 * h
            self.x += self.v * h
            x = self.x
            x[self.root_ids] = root_t
            if len(pin_ids):
                x[pin_ids] = pin_t
            touched = np.zeros(len(x), bool)
            for _it in range(iters):
                for a, b, rest, k in self.groups:
                    d = x[b] - x[a]
                    ln = np.linalg.norm(d, axis=1)
                    ws = w[a] + w[b]
                    c = d * ((ln - rest) / np.maximum(ln, 1e-9) * k / np.maximum(ws, 1e-9))[:, None]
                    x[a] += c * w[a, None]
                    x[b] -= c * w[b, None]
                self._push_out(x, old, boxes, fixed, touched)
                low = x[:, 2] < floor + MARGIN
                x[low, 2] = floor + MARGIN
                touched |= low
            # friction on contact: keep 30 % of the tangential slip
            if touched.any():
                d = x[touched] - old[touched]
                x[touched] = old[touched] + 0.15 * d
            v = (x - old) / h
            sp = np.linalg.norm(v, axis=1)
            v *= np.minimum(1.0, 1.5 / np.maximum(sp, 1e-9))[:, None]
            self.v = v
        if not np.isfinite(self.x).all():
            self.bad = True
            self.x = np.nan_to_num(self.x)
        return self.x

    def max_strain(self):
        e = []
        for a, b, rest, _k in self.groups:
            e.append(np.linalg.norm(self.x[b] - self.x[a], axis=1) / rest)
        return float(np.max(np.concatenate(e)))
