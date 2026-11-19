"""The water: how much the nozzle passes, where its stream lands and how full the bucket gets. NumPy only.

The nozzle's valve opens with the trigger. The open valve passes ``flow(angle)`` litres a second through the
nozzle's "full" (flood) setting, a 10 mm round outlet, so the stream leaves the outlet at
``speed = flow / outlet area`` along the barrel and then falls under gravity. Every control step the stream's
trajectory is traced from the outlet until it meets something:

- the water in the bucket, or the bucket's floor or inner wall below the rim (it then runs down into it): the water
  goes into the bucket;
- the bucket's outer wall or the rim's top, or the ground: it is spilled.

The bucket is a straight-sided tapered pail: inner radius ``R_BOTTOM`` at its inner floor rising linearly to
``R_TOP`` at the brim, ``DEPTH`` deep. The volume below level ``h`` is the frustum's. Water above the brim runs
over it and counts as spilled (overflow).

The model leaves out splashing, the stream breaking up, its hitting the robot, and sloshing; the bucket is scored
upright (a tipped bucket fails the trial outright).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81

# The pail (inside, m). 6.3 L to the brim.
R_BOTTOM = 0.090
R_TOP = 0.110
DEPTH = 0.200
WALL = 0.003  # wall thickness
LIP = 0.010  # the rolled rim's width beyond the outer wall

# The nozzle's flood setting and valve.
OUTLET_D = 0.010  # m
OUTLET_AREA = math.pi * OUTLET_D**2 / 4
FLOW_MAX = 12.0 / 60.0  # L/s at a full squeeze (12 L/min, a garden tap on household pressure)
VALVE_OPENS_DEG = 6.0  # the trigger's travel before the valve starts to open
VALVE_FULL_DEG = 20.0  # travel at which it is fully open (the trigger stops at 22 deg)


def flow(trigger_deg: float) -> float:
    """Litres a second through the nozzle for a trigger pulled ``trigger_deg`` from rest."""
    open_frac = (trigger_deg - VALVE_OPENS_DEG) / (VALVE_FULL_DEG - VALVE_OPENS_DEG)
    return FLOW_MAX * min(1.0, max(0.0, open_frac))


def exit_speed(litres_per_s: float) -> float:
    return litres_per_s / 1000.0 / OUTLET_AREA


def radius_at(h: float) -> float:
    """Inner radius (m) at height ``h`` above the inner floor."""
    return R_BOTTOM + (R_TOP - R_BOTTOM) * min(max(h, 0.0), DEPTH) / DEPTH


def volume_at(h: float) -> float:
    """Litres below a level ``h`` (m above the inner floor)."""
    h = min(max(h, 0.0), DEPTH)
    r = radius_at(h)
    return math.pi * h / 3.0 * (R_BOTTOM**2 + R_BOTTOM * r + r * r) * 1000.0


BRIM_L = volume_at(DEPTH)
_H = np.linspace(0.0, DEPTH, 2001)
_V = np.array([volume_at(h) for h in _H])


def level_at(litres: float) -> float:
    """Level (m above the inner floor) of ``litres`` of water in the bucket."""
    return float(np.interp(litres, _V, _H))


@dataclass
class Landing:
    into_bucket: bool
    point: np.ndarray  # world, where the stream ends
    path: np.ndarray  # world, K x 3, from the outlet to ``point``
    what: str  # "water", "inner wall", "outer wall", "rim", "ground", "none"


def trace(outlet, direction, speed, bucket_floor, bucket_rot, level: float, ground_z: float,
          step_m: float = 0.004, max_t: float = 2.0) -> Landing:
    """Trace a stream leaving ``outlet`` (world, m) along ``direction`` at ``speed`` (m/s).

    ``bucket_floor`` is the centre of the bucket's inner floor (world) and ``bucket_rot`` its rotation (world from
    bucket, its z up the bucket's axis); ``level`` the water's height above that floor (m)."""
    p0 = np.asarray(outlet, float)
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    v0 = speed * d
    rot = np.asarray(bucket_rot, float)
    floor = np.asarray(bucket_floor, float)

    def local(p):
        q = rot.T @ (p - floor)
        return math.hypot(q[0], q[1]), q[2]

    def inside(r, z):
        return 0.0 <= z <= DEPTH and r < radius_at(z)

    pts = [p0]
    r, z = local(p0)
    was_inside = inside(r, z)
    if was_inside and z <= level:
        return Landing(True, p0, np.array(pts), "water")
    t = 0.0
    while t < max_t:
        v = np.linalg.norm(v0 + np.array([0.0, 0.0, -G * t]))
        t += step_m / max(v, 0.2)
        p = p0 + v0 * t + np.array([0.0, 0.0, -0.5 * G * t * t])
        pts.append(p)
        r, z = local(p)
        now_inside = inside(r, z)
        if now_inside and z <= level:
            return Landing(True, p, np.array(pts), "water")
        if was_inside and not now_inside and z < DEPTH:
            # out through the inner wall below the brim: it hits the wall and runs down inside
            return Landing(True, p, np.array(pts), "inner wall" if z > 0.0 else "water")
        if not was_inside and not now_inside:
            if -0.012 <= z <= DEPTH and radius_at(z) <= r <= radius_at(z) + WALL + 0.004:
                return Landing(False, p, np.array(pts), "outer wall")
            if DEPTH <= z <= DEPTH + 0.008 and radius_at(DEPTH) <= r <= radius_at(DEPTH) + WALL + LIP:
                return Landing(False, p, np.array(pts), "rim")
        if p[2] <= ground_z:
            return Landing(False, p, np.array(pts), "ground")
        was_inside = now_inside
    return Landing(False, pts[-1], np.array(pts), "none")


@dataclass
class Tank:
    """What happened to the water over an episode (litres)."""

    in_bucket: float = 0.0
    missed: float = 0.0  # landed outside the bucket
    overflow: float = 0.0  # ran over the brim
    flowing: bool = False
    off_s: float = 0.0  # time since the water last flowed
    history: list = field(default_factory=list)

    @property
    def spilled(self) -> float:
        return self.missed + self.overflow

    @property
    def level(self) -> float:
        return level_at(self.in_bucket)

    def add(self, litres: float, into_bucket: bool) -> None:
        if litres <= 0.0:
            return
        if into_bucket:
            self.in_bucket += litres
            if self.in_bucket > BRIM_L:
                self.overflow += self.in_bucket - BRIM_L
                self.in_bucket = BRIM_L
        else:
            self.missed += litres
