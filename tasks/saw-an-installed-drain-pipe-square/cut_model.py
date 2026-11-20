"""The saw's cut through the pipe, as a model (PhysX cannot cut a solid).

The pipe is a fixed tube along AXIS (through CENTER; radii R_OUT and R_IN). The blade's collider does not collide with
the pipe; instead it rests on a kinematic support plate the model places under its teeth, so PhysX itself takes the
blade's push (its contacts are stable where a spring the model applied would chatter against the hand's grip):

- A kerf is a slot WIDTH wide in a plane through the pipe. It starts where the running blade (trigger pulled) presses
  on the pipe's outside (touching it with F_START), in the blade's plane as it is then; its floor starts at the outside
  surface and is kept per strip across the pipe (BINS strips). With the blade in a kerf's slot, a small pad stands in
  that kerf under the uncut pipe the straight blade meets first; where the kerf is through under the blade, there is
  none, and the blade drops.
- The floor sinks where the blade bears (the strips whose uncut top is the support's) while the trigger is pulled,
  the blade lies within ALIGN of the kerf's plane and the plate pushes back on the blade with at least F_MIN (the
  feed, from PhysX contacts): faster with more feed, V_MAX from F_FULL up. Where a wall is cut through, the floor
  drops through the bore to the far wall.
- With the blade out of every slot, the plate stands under it at the pipe's outside surface (the blade rests on the
  uncut pipe, sliding on it with friction); a running blade touching there starts a kerf.
- A blade pushed sideways out of its slot (by less than NOTCH) is pushed back by a spring: the kerf's sides.
- While it cuts, the work drags the blade toward its tip with PULL times the feed: the teeth face the saw and cut on
  the pull stroke (averaged over strokes); the saw's shoe, against the pipe, takes it.

Pure NumPy: the simulator hook (cut_runtime) and the offline tests both drive it.
"""

from __future__ import annotations

import math

import numpy as np

# Geometry (set by the task file).
CENTER = np.zeros(3)  # a point on the pipe's axis
AXIS = np.array((1.0, 0.0, 0.0))
UP = np.array((0.0, 0.0, 1.0))
R_OUT = 0.0603 / 2
R_IN = R_OUT - 0.0039
X_RANGE = (-1.0, 1.0)  # the pipe's ends along AXIS, from CENTER

# The cut.
WIDTH = 0.0015  # the kerf (a 0.9 mm blade with set teeth cuts ~1.5 mm)
SLOT_HALF = WIDTH / 2 + 0.0004  # the blade's plane this close to a kerf's mid-plane is in its slot
BINS = 49
F_START = 0.5  # N: a running blade touching the outside this hard bites: a kerf starts there
F_MIN = 3.0  # N: below this feed the teeth only rub
F_FULL = 10.0  # N: the feed for full speed (a light push: 'let the saw do the work' on plastic pipe)
V_MAX = 0.015  # m/s: 2-inch PVC in about 4 s
ALIGN = math.radians(5.0)  # the blade's plane within this of its kerf's, or it binds and cuts nothing
BEARING = 0.0003  # strips whose uncut top is this close to the support's bear on the blade
THROUGH = 0.0005  # a strip is cut through when its floor is this far past the wall's far side
SURFACE_SLIDE = 0.002  # the blade slid this far along the outside takes the pad with it
PULL = 0.4  # the average drag toward the blade's tip while cutting, per newton of feed
K_SIDE = 8000.0  # N/m: the kerf's sides on a blade pushed out of its slot
C_SIDE = 40.0  # N s/m
NOTCH = 0.004  # a blade this far beside a kerf's slot is still in it (its sides push it back); further, it is out


def _strips():
    return np.linspace(-R_OUT, R_OUT, BINS)


def _walls(y):
    """The tube's material along a vertical line at ``y`` across it (heights from the axis): (top interval, bottom
    interval); a single interval (as top) where the line misses the bore; (None, None) outside the pipe."""
    if abs(y) >= R_OUT:
        return None, None
    zo = math.sqrt(R_OUT * R_OUT - y * y)
    if abs(y) >= R_IN:
        return (-zo, zo), None
    zi = math.sqrt(R_IN * R_IN - y * y)
    return (zi, zo), (-zo, -zi)


def _side():
    return np.cross(UP, AXIS)


def _uncut_top(y, floor):
    """The highest pipe left uncut below the floor ``floor`` along the line at ``y``, and the interval it is in; or
    (None, None) where it is cut through (or there is no pipe)."""
    top_i, bottom_i = _walls(y)
    if top_i is None:
        return None, None
    for interval in (top_i, bottom_i):
        if interval is None:
            continue
        if floor > interval[0] + 1e-9:
            return min(floor, interval[1]), interval
    return None, None


class Kerf:
    def __init__(self, point, normal):
        self.point = np.asarray(point, float)
        n = np.asarray(normal, float)
        n = n / np.linalg.norm(n)
        self.normal = n if n @ AXIS >= 0 else -n
        self.floor = np.array([(_walls(y)[0] or (0.0, -R_OUT))[1] for y in _strips()])

    def x(self) -> float:
        """Where the kerf's plane crosses the axis (m from CENTER along AXIS)."""
        return float((self.point - CENTER) @ self.normal / (AXIS @ self.normal))

    def squareness(self) -> float:
        """The angle (rad) between the kerf's plane and a plane square to the pipe."""
        return math.acos(min(1.0, abs(float(self.normal @ AXIS))))

    def depth(self) -> float:
        """How far below the pipe's top the kerf reaches (m)."""
        lows = [self.floor[j] for j, y in enumerate(_strips()) if _walls(y)[0] is not None]
        return float(R_OUT - max(min(lows), -R_OUT)) if lows else 0.0

    def support(self, span):
        """The highest uncut pipe under the blade (height from the axis) over the strips in ``span``, or None."""
        tops = [_uncut_top(y, self.floor[j])[0] for j, y in enumerate(_strips()) if span[0] <= y <= span[1]]
        tops = [t for t in tops if t is not None]
        return max(tops) if tops else None

    def round_share(self, samples: int = 144) -> float:
        """The share of the pipe's outside circumference the kerf has cut (a flexible tape round the kerf)."""
        strips = _strips()
        step = strips[1] - strips[0]
        cut = 0
        for i in range(samples):
            t = 2 * math.pi * (i + 0.5) / samples
            j = min(BINS - 2, max(1, int(round((R_OUT * math.sin(t) + R_OUT) / step))))  # the edge strips are empty
            top_i, bottom_i = _walls(strips[j])
            if math.cos(t) >= 0.0:
                cut += self.floor[j] < top_i[1] - 1e-4
            else:
                cut += self.floor[j] <= (bottom_i or top_i)[0] - THROUGH / 2
        return cut / samples

    def through(self) -> bool:
        for j, y in enumerate(_strips()):
            top_i, bottom_i = _walls(y)
            if top_i is None:
                continue
            if abs(y) < R_OUT - 0.001 and self.floor[j] > (bottom_i or top_i)[0] - THROUGH / 2:
                return False
        return True


class Model:
    def __init__(self):
        self.clear()

    def clear(self):
        self.kerfs: list[Kerf] = []
        self.feed = 0.0
        self.cutting = False
        self.plate = None  # (centre of its top face, rotation (x across the kerf, z up), mode), or None (no support)
        self._surface_x = None  # where the pad stands on the outside while the blade stays on it

    def _local(self, p):
        q = np.asarray(p, float) - CENTER
        return float(q @ AXIS), float(q @ _side()), float(q @ UP)

    def step(self, edge, blade_normal, feed: float, trigger: bool, dt: float, edge_vel=None):
        """One physics substep. edge: the two ends of the blade's cutting edge (world); blade_normal: its side
        normal; feed: the plate's push on the blade (N, from PhysX). Updates the kerfs and ``self.plate``; returns the
        forces (force, point) to apply to the saw besides PhysX's."""
        e0, e1 = (np.asarray(e, float) for e in edge)
        n_b = np.asarray(blade_normal, float)
        out = []
        self.cutting = False
        self.feed = float(feed)
        x0, y0, z0 = self._local(e0)
        x1, y1, z1 = self._local(e1)
        lo, hi = min(y0, y1), max(y0, y1)
        if not (hi > -R_OUT and lo < R_OUT and abs(y1 - y0) > 1e-6 and X_RANGE[0] <= 0.5 * (x0 + x1) <= X_RANGE[1]):
            self.plate = None
            self._surface_x = None
            return out
        strips = _strips()

        def edge_at(y):
            s = (y - y0) / (y1 - y0)
            return e0 + s * (e1 - e0), z0 + s * (z1 - z0)

        mid, _ = edge_at(0.5 * (max(lo, -R_OUT) + min(hi, R_OUT)))
        x_mid = self._local(mid)[0]
        kerf, d = None, None
        for k in self.kerfs:
            dk = float((mid - k.point) @ k.normal)
            if d is None or abs(dk) < abs(d):
                kerf, d = k, dk
        in_slot = kerf is not None and abs(d) <= SLOT_HALF
        teeth_z = self._local(mid)[2]
        if kerf is not None and not in_slot and teeth_z < R_OUT + 0.002 and abs(d) < SLOT_HALF + NOTCH:
            # pushed sideways out of its slot at or in the pipe: the kerf's side pushes it back (the teeth sit in their
            # notch from the first stroke), and it is still in it
            direction = -math.copysign(1.0, d) * kerf.normal
            v = 0.0 if edge_vel is None else float(np.asarray(edge_vel[0]) @ direction)
            out.append(((K_SIDE * (abs(d) - SLOT_HALF) - C_SIDE * v) * direction, mid))
            in_slot = True

        def bearing(tops):
            """(the strip the straight edge meets first, its gap to it (m), strips within BEARING of that gap)."""
            gaps = {j: edge_at(strips[j])[1] - t for j, t in tops.items()}
            if not gaps:
                return None, None, []
            j0 = min(gaps, key=gaps.get)
            return j0, gaps[j0], [j for j, g in gaps.items() if g <= gaps[j0] + BEARING]

        if in_slot:
            tops = {}
            for j, y in enumerate(strips):
                if lo <= y <= hi:
                    t, _ = _uncut_top(y, kerf.floor[j])
                    if t is not None:
                        tops[j] = t
            j0, gap, touching = bearing(tops)
            aligned = math.acos(min(1.0, abs(float(n_b @ kerf.normal)))) <= ALIGN
            if j0 is not None and trigger and aligned and feed > F_MIN:
                self.cutting = True
                speed = V_MAX * min(1.0, (feed - F_MIN) / (F_FULL - F_MIN))
                for j in touching:
                    t, interval = _uncut_top(strips[j], kerf.floor[j])
                    kerf.floor[j] = t - speed * dt
                    if kerf.floor[j] <= interval[0]:  # through this wall here
                        kerf.floor[j] = interval[0] - THROUGH
                along = e1 - e0
                out.append((PULL * feed * along / (np.linalg.norm(along) or 1.0), mid))
                tops = {j: _uncut_top(strips[j], kerf.floor[j])[0] for j in tops}
                tops = {j: t for j, t in tops.items() if t is not None}
                j0, gap, touching = bearing(tops)
            self.plate = None if j0 is None else (self._centre(kerf.x(), strips[j0], tops[j0]),
                                                  self._rotation(kerf.normal), "kerf")
        else:
            # On the uncut outside: the plate stands under the blade at the pipe's surface.
            tops = {j: _walls(y)[0][1] for j, y in enumerate(strips) if lo <= y <= hi and _walls(y)[0] is not None}
            j0, gap, _ = bearing(tops)
            if j0 is None:
                self.plate = None
                return out
            # The pad stays where the blade came down on the pipe (so the blade slides on it with friction, as on the
            # pipe), and moves only if the blade slides off it.
            if self._surface_x is None or abs(x_mid - self._surface_x) > SURFACE_SLIDE:
                self._surface_x = x_mid
            self.plate = (self._centre(self._surface_x, strips[j0], tops[j0]), self._rotation(n_b), "surface")
            if trigger and feed >= F_START:
                self.kerfs.append(Kerf(self._centre(x_mid, strips[j0], tops[j0]), n_b))
                self._surface_x = None
        return out

    def _centre(self, x, y, top):
        return CENTER + x * AXIS + y * _side() + top * UP

    def _rotation(self, normal):
        n = np.asarray(normal, float)
        n = n if n @ AXIS >= 0 else -n
        z = UP - (UP @ n) * n
        z /= np.linalg.norm(z)
        return np.column_stack((n, np.cross(z, n), z))

    def main_kerf(self):
        return max(self.kerfs, key=lambda k: k.depth()) if self.kerfs else None


MODEL = Model()
