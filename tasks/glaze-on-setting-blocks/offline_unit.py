"""Offline only: a quasi-static model of the glass unit for tools/offline/check_task.py, which has no physics.

The checker carries plain boxes upright with a straight-down grip. This task's demo tilts the unit 40 degrees,
leans it on a fingertip and pushes it upright, so offline the unit's state comes from here instead, driven only by
what the scorer itself reads (rigstate's TCP, pad-tip point and opening for each arm) and the blocks' states. In the
simulator the physics moves the unit and none of this runs.

The model, each control step:
- Held: a gripper whose fingers close past the unit's 24 mm with its pad-tip point inside the unit (3 mm slack)
  takes it; the unit then moves rigidly with that hand (any tilt) until the fingers open past 25 mm. Held tilted, it
  also sags with the hand by the payload sag the demo compensates for (task.py SAG), as far as the blocks let it.
- Free: the unit keeps its long axis along y and moves in the x-z plane (its cross-section, against the cross-
  sections of whatever overlaps it along y: the frame's boxes, the marks, the blocks, the table, and the pads of a
  hand not holding it; the hand that has just let go is left out until its pads are clear of it). Let go pressed
  into something, it first rises out of it. A finger pressing into it turns it about its lowest bottom corner, or
  slides it, whichever moves it least, until clear; it drops straight down onto what is under it; then gravity turns
  it about its lowest bottom corner, at most 2.5 degrees per step, until something stops it (both bottom corners
  down, a finger, the outside stop). It lands 0.02 mm into what it lands on, as a solver leaves a resting body.
- No friction, sliding, bouncing or toppling sideways is modelled. A hand's rotation is rebuilt from its approach
  axis as the demo builds it (tiltgrip.rotation: the left hand yaw -pi/2, the right +pi/2, pitched about y and
  rolled about x), so the model is right for this demo's hands and approximate for anything else.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

import layout as L
import rigstate
import tiltgrip

_UH = np.array(L.UNIT_SIZE) / 2
_FULL_GAP = rigstate.FULL_GAP
_GRIP_OPEN = L.UNIT_SIZE[0] / _FULL_GAP  # opening (0..1) at which the pads are 24 mm apart
_RELEASE_OPEN = (L.UNIT_SIZE[0] + 0.001) / _FULL_GAP
_YAWS = {"left": -math.pi / 2, "right": math.pi / 2}
_PAD_BACK, _PAD_TIP, _PAD_T, _PAD_HW = 0.025, 0.013, 0.008, 0.0135
_LAND = 2e-5  # m: how far a body comes to rest inside what it lands on
_EPS = 1e-7  # m: a move is blocked by anything it would press in deeper than it already is by more than this
_STEP = math.radians(2.5)  # most gravity turns the unit in one control step


def _quat(r: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w) of a rotation matrix."""
    t = float(np.trace(r))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q


def _rotation(v) -> np.ndarray:
    """Rotation matrix of a rotation vector (rad)."""
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def hand(arm, side: str):
    """(tcp, rotation) of a hand as rigstate reports it, its rotation rebuilt as the demo builds it."""
    tcp, tips = np.asarray(arm.tcp, dtype=float), np.asarray(arm.tips, dtype=float)
    pitch, roll = tiltgrip.pitch_roll(tips - tcp)
    return tcp, tiltgrip.rotation(_YAWS[side], pitch, roll)


def pad_points(tcp, rot, opening: float) -> np.ndarray:
    """The 16 corners of both finger pads (world): the first 8 one pad's, the last 8 the other's."""
    gap = float(np.clip(opening, 0.0, 1.0)) * _FULL_GAP
    out = []
    for side in (-1.0, 1.0):
        for t in (gap / 2, gap / 2 + _PAD_T):
            for w in (-_PAD_HW, _PAD_HW):
                for a in (-_PAD_BACK, _PAD_TIP):
                    out.append(tcp + rot[:, 0] * side * t + rot[:, 1] * w + rot[:, 2] * a)
    return np.array(out)


def _hull(p: np.ndarray) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2-D points."""
    pts = sorted(set(map(tuple, np.round(p, 9))))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def _overlap(a: np.ndarray, b: np.ndarray) -> float:
    """How deep two convex polygons (2-D vertex arrays) overlap (<= 0: apart or touching)."""
    best = math.inf
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            e = poly[(i + 1) % n] - poly[i]
            axis = np.array([-e[1], e[0]])
            norm = float(np.linalg.norm(axis))
            if norm < 1e-12:
                continue
            axis /= norm
            pa, pb = a @ axis, b @ axis
            best = min(best, min(pa.max(), pb.max()) - max(pa.min(), pb.min()))
    return best


def _rect(lo_x, hi_x, lo_z, hi_z) -> np.ndarray:
    return np.array([[lo_x, lo_z], [hi_x, lo_z], [hi_x, hi_z], [lo_x, hi_z]])


class UnitProxy:
    def __init__(self, sag=None) -> None:
        """``sag``: (TCP shift (m, 3), turn (degrees about world x, y, z), the least pitch (rad) at which it applies)
        of a hand holding the unit tilted: the payload sag the demo compensates for. It is applied in full while the
        unit hangs from the hand, and only as far as the blocks (or anything else) let it once the unit rests on
        them."""
        self.sag = None
        if sag is not None:
            vec, rot_deg, min_pitch = sag
            self.sag = (np.asarray(vec, dtype=float), np.radians(np.asarray(rot_deg, dtype=float)), float(min_pitch))
        self.reset()

    def reset(self) -> None:
        self.c = np.array([L.UNIT_X, L.UNIT_Y, L.TOP + _UH[2]])
        self.r = np.eye(3)
        self.holder: str | None = None
        self.rel = None
        self.letting_go: set[str] = set()
        self.prev_open = {"left": 1.0, "right": 1.0}
        self.prev = (self.c.copy(), self.r.copy())
        self.vel = np.zeros(3)
        self.spin = np.zeros(3)
        self.events: list[tuple] = []
        self.steps = 0

    # -- the unit's state ---------------------------------------------------------------------------------------
    def state(self):
        ext = np.abs(self.r) @ _UH
        return SimpleNamespace(pos=self.c.copy(), quat_xyzw=_quat(self.r), lin_vel=self.vel.copy(),
                               ang_vel=self.spin.copy(), low=self.c - ext, high=self.c + ext)

    def _section(self, c=None, r=None) -> np.ndarray:
        """The unit's cross-section in the x-z plane (its four edges along y), as a polygon."""
        c = self.c if c is None else c
        r = self.r if r is None else r
        pts = [c + r @ (_UH * np.array([i, 0.0, k])) for i, k in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        return _hull(np.array([[p[0], p[2]] for p in pts]))

    # -- what it can run into -------------------------------------------------------------------------------------
    def _obstacles(self, arms, blocks) -> tuple[list, list]:
        """(fixed and loose solids, fingers) overlapping the unit along y, as x-z polygons."""
        y0, y1 = self.c[1] - _UH[1], self.c[1] + _UH[1]
        solids = [_rect(-1.0, 2.0, L.TOP - 1.0, L.TOP)]  # the table
        boxes = list(L.FRAME_BOXES.values()) + [((L.MARK_X, my, L.SILL_TOP + L.MARK_SIZE[2] / 2), L.MARK_SIZE)
                                                for my in L.MARK_YS]
        for (cx, cy, cz), (sx, sy, sz) in boxes:
            if cy + sy / 2 > y0 and cy - sy / 2 < y1:
                solids.append(_rect(cx - sx / 2, cx + sx / 2, cz - sz / 2, cz + sz / 2))
        for b in blocks:
            lo, hi = np.asarray(b.low, dtype=float), np.asarray(b.high, dtype=float)
            if np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)) and hi[1] > y0 and lo[1] < y1:
                solids.append(_rect(lo[0], hi[0], lo[2], hi[2]))
        fingers = []
        if arms is not None:
            sec = self._section()
            for side in ("left", "right"):
                if side == self.holder or side not in arms:
                    continue
                tcp, rot = hand(arms[side], side)
                pts = pad_points(tcp, rot, float(arms[side].opening))
                pads = [_hull(pad[:, [0, 2]]) for pad in (pts[:8], pts[8:])  # each pad on its own
                        if pad[:, 1].max() > y0 and pad[:, 1].min() < y1]
                if side in self.letting_go:  # the hand that just let go opens away from it: it pushes nothing
                    if any(_overlap(sec, q) > 0.0 for q in pads):
                        continue
                    self.letting_go.discard(side)
                fingers += pads
        return solids, fingers

    @staticmethod
    def _depths(poly, polys) -> list[float]:
        return [_overlap(poly, q) for q in polys]

    @staticmethod
    def _deeper(poly, polys, base) -> bool:
        """``poly`` presses into one of ``polys`` deeper than ``base`` (the overlaps now; never less than touching)."""
        return any(_overlap(poly, q) > max(b, 0.0) + _EPS for q, b in zip(polys, base))

    # -- moves ---------------------------------------------------------------------------------------------------
    def _turned(self, about_xz, angle: float):
        """(centre, rotation) turned ``angle`` rad about the y axis through the point (x, z): + turns +z toward +x."""
        rot = tiltgrip.ry(angle)
        p = np.array([about_xz[0], self.c[1], about_xz[1]])
        return p + rot @ (self.c - p), rot @ self.r

    def _bottom_corners(self) -> list[np.ndarray]:
        """The two lowest corners of the cross-section (x, z), lowest first."""
        sec = self._section()
        order = np.argsort(sec[:, 1])
        return [sec[order[0]], sec[order[1]]]

    @staticmethod
    def _largest(ok, top: float, step: float) -> float:
        """The largest t in [0, top] such that ok holds all the way from 0 to t (ok(0) True): a sweep in steps of
        ``step``, then bisection inside the first step that fails."""
        t = 0.0
        while t < top:
            t2 = min(t + step, top)
            if not ok(t2):
                lo, hi = t, t2
                for _ in range(30):
                    mid = (lo + hi) / 2
                    lo, hi = (mid, hi) if ok(mid) else (lo, mid)
                return lo
            t = t2
        return top

    def _push_clear(self, solids, fingers) -> None:
        """A finger pressing into the unit turns it about its lowest bottom corner, or slides it along x, whichever
        moves it least, until clear of every finger (never deeper into a solid)."""
        sec = self._section()
        if not any(d > _EPS for d in self._depths(sec, fingers)):
            return
        base = self._depths(sec, solids)
        clear_f = [0.0] * len(fingers)
        best = None
        pivot = self._bottom_corners()[0]
        for sign in (1.0, -1.0):
            def clear(a, sign=sign):
                c, r = self._turned(pivot, sign * a)
                s2 = self._section(c, r)
                return not self._deeper(s2, fingers, clear_f) and not self._deeper(s2, solids, base)
            a = self._smallest(clear, math.radians(45.0))
            if a is not None and (best is None or a * _UH[2] < best[0]):
                best = (a * _UH[2], "turn", sign * a)
        for sign in (1.0, -1.0):
            def clear(d, sign=sign):
                s2 = self._section(self.c + np.array([sign * d, 0.0, 0.0]))
                return not self._deeper(s2, fingers, clear_f) and not self._deeper(s2, solids, base)
            d = self._smallest(clear, 0.02)
            if d is not None and (best is None or d < best[0]):
                best = (d, "slide", sign * d)
        if best is None:
            return
        if best[1] == "turn":
            self.c, self.r = self._turned(pivot, best[2])
        else:
            self.c = self.c + np.array([best[2], 0.0, 0.0])

    @staticmethod
    def _smallest(ok, top: float) -> float | None:
        """The smallest t in (0, top] with ok(t), by a scan then bisection; None if there is none."""
        n = 24
        prev = 0.0
        for i in range(1, n + 1):
            t = top * i / n
            if ok(t):
                lo, hi = prev, t
                for _ in range(25):
                    mid = (lo + hi) / 2
                    lo, hi = (lo, mid) if ok(mid) else (mid, hi)
                return hi
            prev = t
        return None

    def _lift_out(self, solids) -> None:
        """Let go pressed into something (a hand can set it down below where it can be), it rises straight up until
        it rests only _LAND into it, as a solver pushes a body out."""
        if max(self._depths(self._section(), solids), default=0.0) <= _LAND + 1e-6:
            return

        def deep(u):
            return max(self._depths(self._section(self.c + np.array([0.0, 0.0, u])), solids)) > _LAND + 1e-6

        lo, hi = 0.0, 0.03
        if deep(hi):
            return
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if deep(mid) else (lo, mid)
        self.c = self.c + np.array([0.0, 0.0, hi])

    def _drop(self, solids) -> None:
        """Straight down onto whatever is under it (it comes to rest _LAND into it)."""
        base = self._depths(self._section(), solids)

        def ok(d):
            return not self._deeper(self._section(self.c - np.array([0.0, 0.0, d])), solids, base)

        d = self._largest(ok, 0.3, 0.0005)
        if d > 1e-6:
            self.c = self.c - np.array([0.0, 0.0, d + _LAND])

    def _fall(self, solids, fingers) -> None:
        """Gravity turns the unit about its lowest bottom corner, at most _STEP, until something stops it."""
        low, other = self._bottom_corners()
        if other[1] - low[1] < 1e-7:
            return  # flat on its bottom face
        sign = 1.0 if self.c[0] > low[0] else -1.0  # +: the centre is outside of the corner, it falls toward +x
        obstacles = solids + fingers
        base = self._depths(self._section(), obstacles)

        def ok(a):
            c, r = self._turned(low, sign * a)
            return not self._deeper(self._section(c, r), obstacles, base)

        a = self._largest(ok, _STEP, math.radians(0.25))
        if a > 1e-6:
            self.c, self.r = self._turned(low, sign * a)

    def _sag(self, blocks, tcp, rot) -> None:
        """The held unit sags with the hand, as far as what is under it lets it (see __init__); a hand that sets it
        down lower than it can go (into the blocks) gives way, as the arm's compliance does: the unit stays on them."""
        solids, _ = self._obstacles(None, blocks)
        if self.sag is not None and tiltgrip.pitch_roll(rot[:, 2])[0] >= self.sag[2]:
            vec, turn, _ = self.sag
            c0, r0 = self.c.copy(), self.r.copy()

            def pose(f):
                t = _rotation(f * turn)
                return tcp + f * vec + t @ (c0 - tcp), t @ r0

            base = self._depths(self._section(c0, r0), solids)

            def ok(f):
                c, r = pose(f)
                return not self._deeper(self._section(c, r), solids, base)

            self.c, self.r = pose(self._largest(ok, 1.0, 0.05))
        self._lift_out(solids)

    # -- one control step -----------------------------------------------------------------------------------------
    def step(self, arms, block_1, block_2) -> None:
        self.steps += 1
        readable = rigstate.readable(arms)
        if self.holder is not None:
            a = arms.get(self.holder) if readable else None
            if a is None or float(a.opening) > _RELEASE_OPEN:
                self.events.append(("release", self.holder, self.steps))
                self.letting_go.add(self.holder)
                self.holder, self.rel = None, None
            else:
                tcp, rot = hand(a, self.holder)
                self.c, self.r = tcp + rot @ self.rel[0], rot @ self.rel[1]
                self._sag((block_1, block_2), tcp, rot)
        elif readable:
            for side in ("left", "right"):
                a = arms[side]
                if float(a.opening) < _GRIP_OPEN <= self.prev_open[side]:
                    local = (np.asarray(a.tips, dtype=float) - self.c) @ self.r
                    tcp, rot = hand(a, side)
                    if np.all(np.abs(local) <= _UH + 0.003) and abs(float(rot[:, 0] @ self.r[:, 0])) > 0.9:
                        self.holder = side
                        self.rel = (rot.T @ (self.c - tcp), rot.T @ self.r)
                        self.events.append(("grasp", side, self.steps))
                        break
        if readable:
            self.prev_open = {s: float(arms[s].opening) for s in ("left", "right")}
        if self.holder is None and abs(float(self.r[1, 1])) > 0.999:
            solids, fingers = self._obstacles(arms if readable else None, (block_1, block_2))
            self._lift_out(solids)
            self._push_clear(solids, fingers)
            self._drop(solids)
            self._fall(solids, fingers)
        dt = 1.0 / L.CONTROL_HZ
        c0, r0 = self.prev
        self.vel = (self.c - c0) / dt
        w = self.r @ r0.T
        angle = math.acos(float(np.clip((np.trace(w) - 1) / 2, -1.0, 1.0)))
        self.spin = np.array([0.0, angle / dt, 0.0])
        self.prev = (self.c.copy(), self.r.copy())
