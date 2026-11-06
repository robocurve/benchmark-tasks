"""Declared-limit checks for yam_arena task files: keep-out and keep-in areas, height bands and speed limits, scored from
object states. Copy this file next to a task file and import it (``from belimits import ...``); it needs only NumPy.

Every check reads the object's real footprint from its pose (a turned or tipped box is judged by its outline seen from
above, not its axis-aligned bounding box). Units are metres and the station frame (x forward, y left, z up).

Trackers (highest point reached, fastest speed reached) keep state across control steps, because ``score()`` only
sees the current step. A tracker's ``clear()`` is meant to be called from an ``on_reset`` callback (mechkit); as a fallback they
also reset themselves when a real trial reset is seen from inside ``score()`` (every tracked object back at its spawn pose and still).
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "Rect", "corners", "footprint", "overlaps", "inside", "bottom", "top", "tilt_deg", "is_flat_on", "is_airborne",
    "PeakTracker", "ResetOnJumpTracker",
]


class Rect:
    """An axis-aligned rectangle seen from above: x in (x0, x1), y in (y0, y1)."""

    def __init__(self, x, y):
        self.x0, self.x1 = float(min(x)), float(max(x))
        self.y0, self.y1 = float(min(y)), float(max(y))

    def grown(self, m: float) -> "Rect":
        return Rect((self.x0 - m, self.x1 + m), (self.y0 - m, self.y1 + m))

    def poly(self) -> np.ndarray:
        return np.array([[self.x0, self.y0], [self.x1, self.y0], [self.x1, self.y1], [self.x0, self.y1]])


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _half_extents(spec) -> tuple[str, np.ndarray]:
    if hasattr(spec, "size"):
        return "box", np.asarray(spec.size, float) / 2
    if hasattr(spec, "height"):
        return "cylinder", np.array([spec.radius, spec.radius, spec.height / 2], float)
    if hasattr(spec, "radius"):
        return "sphere", np.array([spec.radius] * 3, float)
    raise TypeError(f"unsupported object spec {spec!r}")


def corners(state, spec) -> np.ndarray:
    """3-D points on the object's surface, in the world: box corners; cylinder rims (16 points each); sphere extremes."""
    kind, h = _half_extents(spec)
    rot, c = _rot(state.quat_xyzw), np.asarray(state.pos, float)
    if kind == "box":
        local = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float) * h
    elif kind == "cylinder":
        a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        ring = np.stack([np.cos(a) * h[0], np.sin(a) * h[1]], axis=1)
        local = np.vstack([np.c_[ring, np.full(16, -h[2])], np.c_[ring, np.full(16, h[2])]])
    else:
        r = h[0]
        local = np.array([[r, 0, 0], [-r, 0, 0], [0, r, 0], [0, -r, 0], [0, 0, r], [0, 0, -r]], float)
        return c + local  # a sphere's outline does not turn
    return c + local @ rot.T


def footprint(state, spec) -> np.ndarray:
    """The object's outline seen from above: its surface points projected onto the table (x, y)."""
    return corners(state, spec)[:, :2]


def _sat_overlap(pts: np.ndarray, poly: np.ndarray) -> bool:
    """Separating-axis test between the convex hull of ``pts`` and a convex polygon."""
    axes = []
    for shape in (poly, pts):
        n = len(shape)
        for i in range(n):
            e = shape[(i + 1) % n] - shape[i]
            if np.hypot(*e) > 1e-9:
                axes.append(np.array([-e[1], e[0]]) / np.hypot(*e))
    if len(pts) > 2:  # hull edges of the point set: use all pairwise directions (small sets)
        d = pts[:, None, :] - pts[None, :, :]
        for e in d.reshape(-1, 2)[:: max(1, len(pts) // 4)]:
            if np.hypot(*e) > 1e-9:
                axes.append(np.array([-e[1], e[0]]) / np.hypot(*e))
    for a in axes:
        p, r = pts @ a, poly @ a
        if p.max() <= r.min() or r.max() <= p.min():
            return False
    return True


def overlaps(state, spec, area: Rect, z_top: float | None = None) -> bool:
    """Any part of the object's outline over ``area`` (and, if ``z_top`` is given, its bottom below ``z_top``)."""
    if z_top is not None and bottom(state, spec) >= z_top:
        return False
    return _sat_overlap(footprint(state, spec), area.poly())


def inside(state, spec, area: Rect) -> bool:
    """The whole outline inside ``area``."""
    fp = footprint(state, spec)
    return bool(np.all(fp[:, 0] >= area.x0) and np.all(fp[:, 0] <= area.x1) and np.all(fp[:, 1] >= area.y0)
                and np.all(fp[:, 1] <= area.y1))


def bottom(state, spec) -> float:
    return float(corners(state, spec)[:, 2].min())


def top(state, spec) -> float:
    return float(corners(state, spec)[:, 2].max())


def tilt_deg(state) -> float:
    """Angle between the object's own up axis and the world's up axis."""
    return float(np.degrees(np.arccos(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0))))


def is_flat_on(state, spec, surface_z: float, tol: float = 0.006, max_tilt: float = 15.0) -> bool:
    """Resting upright on a surface at height ``surface_z``: bottom within ``tol`` of it and tilted at most ``max_tilt``."""
    return abs(bottom(state, spec) - surface_z) <= tol and tilt_deg(state) <= max_tilt


def is_airborne(state, spec, supports_z, clearance: float = 0.01) -> bool:
    """More than ``clearance`` above every support height: still held, or in flight. The simulator scores the last step
    without opening the grippers, so a held object must never count as placed."""
    b = bottom(state, spec)
    return all(b - z > clearance for z in supports_z)


class PeakTracker:
    """Highest value a per-object quantity reaches over a trial (height of an object's top, speed, ...).

    ``update(objects, values)`` records ``values`` ({name: number}) and returns the peaks so far. It resets when every
    object in ``home`` ({name: (x, y, z)}) is within ``home_tol`` of home and still: the state a trial starts from.
    """

    def __init__(self, home: dict, home_tol: float = 0.005, still_speed: float = 0.01):
        self.home = {k: np.asarray(v, float) for k, v in home.items()}
        self.tol, self.still = home_tol, still_speed
        self.peaks: dict = {}
        self._left_home = False

    def _at_home(self, objects) -> bool:
        for name, p in self.home.items():
            s = objects[name]
            if np.linalg.norm(np.asarray(s.pos, float) - p) > self.tol or np.linalg.norm(s.lin_vel) > self.still:
                return False
        return True

    def update(self, objects, values: dict) -> dict:
        home = self._at_home(objects)
        if home and self._left_home:
            self.peaks, self._left_home = {}, False
        if not home:
            self._left_home = True
        for k, v in values.items():
            self.peaks[k] = max(self.peaks.get(k, -np.inf), float(v))
        return dict(self.peaks)


class ResetOnJumpTracker(PeakTracker):
    """A PeakTracker that clears only on a real trial reset. A reset is a teleport: a tracked object that was more than
    ``jump`` (3 cm) from its home on the previous step is home (within ``home_tol``) on this step, and every tracked
    object is home and still. No placement moves 3 cm in one 50 ms step, so putting an object back by hand or by the
    robot never clears the latch."""

    def __init__(self, home: dict, home_tol: float = 0.005, still_speed: float = 0.01, jump: float = 0.03):
        super().__init__(home, home_tol, still_speed)
        self.jump = jump
        self._prev = None

    def clear(self):
        self.peaks, self._prev, self._left_home = {}, None, False

    def _dists(self, objects) -> dict:
        return {k: float(np.linalg.norm(np.asarray(objects[k].pos, float) - p)) for k, p in self.home.items()}

    def update(self, objects, values: dict) -> dict:
        dist = self._dists(objects)
        if (self._prev is not None and self._at_home(objects)
                and any(self._prev[k] > self.jump and dist[k] <= self.tol for k in dist)):
            self.peaks = {}
        self._prev = dist
        for k, v in values.items():
            self.peaks[k] = max(self.peaks.get(k, -np.inf), float(v))
        return dict(self.peaks)
