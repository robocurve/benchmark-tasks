"""Kinematics of the clipper for use ONLY where there is no simulator: tools/offline/check_task.py (which by itself can
carry and drop a gripped box, nothing else) and the tests. The simulator never takes a result from here: there the
swivel and the lever's hinge are PhysX joints of one articulation, the lever's two rests on the blade are the stops of
its hinge, and nothing is moved by code.

What this file assumes is what the design claims, so the offline prediction is the design's prediction, not a
measurement. It moves the lever from the arms' COMMANDED pad-tip points and openings (rigstate's offline feed):

- The clipper stays in its slot. The pads are two 8 x 27 x 38 mm slabs closing along the axis that is perpendicular to
  the approach axis and lies in the plane of that axis and world x (every grip of this task's demo closes that way:
  straight down closing along x, or pitched about y), the tip end at the pad-tip point.
- The swivel psi is moved by the pads' outline in plan: a pad point that lies in the lever's plate (its height within
  the plate's at that radius) forbids the lever the angles (its bearing from the pin) plus or minus the plate's
  half-width as seen from that radius; a lever inside the forbidden set is moved the shortest way out. The plate's
  edge is thus pushed round, and a pad face parallel to the blade squares the lever to it.
- The lever's elevation eps is moved by the pads' outline in the lever's own plane (side view): a pad point within the
  plate's width of that plane forbids the plate (its bearing from the hinge) plus or minus the plate's half-thickness
  as seen from that radius, the same way. Of the two moves (swivel, elevation) the smaller is taken.
- Gravity: a lever let go below upright settles on its heel (its lower stop, layout.TH0, the same at every swivel); a
  lever let go beyond upright falls onto the blade (its upper stop, layout.EPS_STOW). Neither stop can be passed: a
  pad pressing on the lever there moves nothing (the jaws do not close in this build).
  A pad holds it where it is by the same push.
- Nothing slides, tips or lifts, and a gripper holds nothing.
"""

from __future__ import annotations

import math

import numpy as np

import layout as L
import rigstate

TOL = 1.0  # mm: half the sampling pitch, added to every half-width (a pad point stands for its surroundings)
EDGE = 3.0  # mm: a pad over the plate's middle 6 mm presses its face (the elevation), not its edge
SUBSTEPS = 8  # per control step
FALL_RATE = 2.5  # rad/s: how fast a free lever settles
REACH_SKIP = 160.0  # mm: a gripper tip farther than this from the pin cannot touch the lever


def pad_points(tcp, tips, opening: float, origin) -> np.ndarray:
    """Points (N, 3), mm in the clipper frame, filling the two pads (two slabs, a finger's thickness each, apart by the
    gripper's gap) of a gripper with its TCP and pad tips at world positions ``tcp`` and ``tips``."""
    tcp, tips = np.asarray(tcp, dtype=float), np.asarray(tips, dtype=float)
    a = tips - tcp
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        return np.empty((0, 3))
    a = a / n
    c = np.array([-a[2], 0.0, a[0]])
    nc = float(np.linalg.norm(c))
    c = np.array([1.0, 0.0, 0.0]) if nc < 1e-9 else c / nc
    w = np.cross(a, c)
    gap = float(np.clip(opening, 0.0, 1.0)) * rigstate.FULL_GAP / L.MM
    ci = np.concatenate([np.linspace(gap / 2 + 0.5, gap / 2 + L.PAD_C - 0.5, 3),
                         -np.linspace(gap / 2 + 0.5, gap / 2 + L.PAD_C - 0.5, 3)])
    wi = np.linspace(-L.PAD_W, L.PAD_W, 12)
    ai = np.linspace(0.0, L.PAD_H, 16)
    cc, ww, aa = np.meshgrid(ci, wi, ai, indexing="ij")
    base = (tips - np.asarray(origin, dtype=float)) / L.MM
    return base + cc.reshape(-1, 1) * c + ww.reshape(-1, 1) * w - aa.reshape(-1, 1) * a


def _clear_interval(intervals: list[tuple[float, float]]) -> float:
    """The shortest signed move that takes the origin out of the union of ``intervals`` ((lo, hi) relative to it).
    0 when it is not inside any."""
    inside = sorted(iv for iv in intervals if iv[0] < 0.0 < iv[1])
    if not inside:
        return 0.0
    lo, hi = inside[0]
    grown = True
    while grown:  # grow the component with every interval that overlaps it
        grown = False
        for a, b in intervals:
            if a < hi and b > lo and (a < lo or b > hi):
                lo, hi = min(lo, a), max(hi, b)
                grown = True
    return hi if abs(hi) <= abs(lo) else lo


class OfflineClipper:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.psi = 0.0
        self.eps = L.TH0  # on its heel: the lower stop, the pose as drawn
        self.moved = {"psi": 0.0, "eps": 0.0}
        self.last = None  # the grippers at the last control step

    # -- the two pushes ------------------------------------------------------------------------------------------
    def _swing(self, pts: np.ndarray) -> tuple[float, bool]:
        """(the move (rad) of the swivel that takes the lever out of the pads' outline in plan, whether a pad is over or
        under the plate itself). A pad over or under the plate cannot push its edge: the move is then 0."""
        ce = math.cos(self.eps)
        if abs(ce) < 0.05:
            return 0.0, False  # a standing lever has no plan length to push
        theta = self.psi + (0.0 if ce >= 0.0 else math.pi)  # the lever's bearing in plan
        v = pts[:, :2] - np.array([L.PIN_X, 0.0])
        r = np.hypot(v[:, 0], v[:, 1])
        phi = np.arctan2(v[:, 1], v[:, 0])
        rel_all = np.remainder(phi - theta + math.pi, 2.0 * math.pi) - math.pi
        along, lateral = r * np.cos(rel_all), r * np.sin(rel_all)
        on_range = (along >= L.LEV_X0 * abs(ce)) & (along <= L.LEV * abs(ce) + TOL)
        if bool((on_range & (np.abs(lateral) <= EDGE)).any()):
            return 0.0, True
        zc = L.ZH + r * math.sin(self.eps) / abs(ce)
        half_v = (L.LEV_T / 2) / abs(ce) + TOL
        ok = on_range & (np.abs(pts[:, 2] - zc) <= half_v)
        if not ok.any():
            return 0.0, False
        half = np.arcsin(np.minimum(1.0, (L.LEV_W / 2 + TOL) / np.maximum(r[ok], 1e-9)))
        return _clear_interval(list(zip(rel_all[ok] - half, rel_all[ok] + half))), False

    def _flip(self, pts: np.ndarray) -> float:
        """The move (rad) of the elevation that takes the lever out of the pads' outline in its own plane."""
        c, s = math.cos(self.psi), math.sin(self.psi)
        d = pts[:, :2] - np.array([L.PIN_X, 0.0])
        xp = d[:, 0] * c + d[:, 1] * s
        yp = -d[:, 0] * s + d[:, 1] * c
        zp = pts[:, 2] - L.ZH
        rho = np.hypot(xp, zp)
        ok = (np.abs(yp) <= L.LEV_W / 2 - EDGE) & (rho >= L.LEV_X0) & (rho <= L.LEV + TOL)
        if not ok.any():
            return 0.0
        th = np.arctan2(zp[ok], xp[ok])
        half = np.arcsin(np.minimum(1.0, (L.LEV_T / 2 + TOL) / rho[ok]))
        rel = np.remainder(th - self.eps + math.pi, 2.0 * math.pi) - math.pi
        return _clear_interval(list(zip(rel - half, rel + half)))

    def _gravity(self, dt: float) -> None:
        target = L.TH0 if self.eps < math.pi / 2 else L.EPS_STOW
        step = FALL_RATE * dt
        self.eps += float(np.clip(target - self.eps, -step, step))

    # -- one control step ----------------------------------------------------------------------------------------
    def step(self, arms, origin=L.ORIGIN, dt: float = 1.0 / L.CONTROL_HZ) -> None:
        """Advance one control step. The grippers move from where they were at the last step to where they are now in
        SUBSTEPS small moves, so that neither a gripper nor gravity can carry the lever through a pad."""
        read = rigstate.read_arms(arms)
        if read is None:
            self.last = None
            return  # nothing can be judged from unreadable arms: the clipper stays as it is
        now = [(np.asarray(tcp, dtype=float), np.asarray(tips, dtype=float), float(op)) for tcp, tips, op in read]
        before = self.last if self.last is not None else now
        self.last = now
        for j in range(1, SUBSTEPS + 1):
            f = j / SUBSTEPS
            pts = []
            for (t0, p0, o0), (t1, p1, o1) in zip(before, now):
                tcp, tips, opening = t0 + f * (t1 - t0), p0 + f * (p1 - p0), o0 + f * (o1 - o0)
                near = (tips - np.asarray(origin, dtype=float)) / L.MM
                if float(np.hypot(near[0] - L.PIN_X, near[1])) <= REACH_SKIP and near[2] <= 150.0:
                    pts.append(pad_points(tcp, tips, opening, origin))
            cloud = np.vstack(pts) if pts else np.empty((0, 3))
            self._substep(cloud, dt / SUBSTEPS)

    def _substep(self, cloud: np.ndarray, dt: float) -> None:
        if len(cloud):
            d_psi, over = self._swing(cloud)
            if d_psi != 0.0 and not over:
                self.psi = float(np.clip(self.psi + d_psi, -L.PIN_LIMIT, L.PIN_LIMIT))
                self.moved["psi"] += abs(d_psi)
        self._gravity(dt)
        if len(cloud):
            d_eps = self._flip(cloud)
            if d_eps != 0.0:
                self.eps += d_eps
                self.moved["eps"] += abs(d_eps)
        self.eps = float(np.clip(self.eps, L.TH0, L.EPS_STOW))  # the two stops of the lever's hinge

    def joints(self) -> dict:
        return {"pin": self.psi, "lever": self.eps - L.E0}

    def pose(self) -> tuple[np.ndarray, float]:
        return np.array(L.ORIGIN, dtype=float), 0.0
