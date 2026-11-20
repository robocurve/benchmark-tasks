"""Kinematic timelines for a task's observe phase: an operator's prop or hand-moved object follows keyframes.

    keys = [(t, (x, y, z)), (t, (x, y, z), yaw), ...]   # seconds since reset, world metres, yaw rad

``drive(world, name, t, keys)`` writes the interpolated pose while ``t`` is inside the keyframes (plus a short tail
so the last pose is written at least once at 10 or 20 Hz). Outside them the object is left to physics: end a
timeline on a stable rest pose (on the table, in a container, or parked on the floor under the table).
"""

from __future__ import annotations

import math

from yam_arena import station

FLOOR_Z = station.FLOOR_Z
TAIL_S = 0.12


def _key(k):
    t, pos, *rest = k
    return float(t), tuple(float(v) for v in pos), float(rest[0]) if rest else 0.0


def pose_at(t: float, keys) -> tuple[tuple[float, float, float], float]:
    ks = [_key(k) for k in keys]
    if t <= ks[0][0]:
        return ks[0][1], ks[0][2]
    for (t0, p0, y0), (t1, p1, y1) in zip(ks, ks[1:]):
        if t <= t1:
            a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            a = a * a * (3 - 2 * a)  # ease in/out: hand motion, no velocity jumps at the keys
            return tuple(u + a * (v - u) for u, v in zip(p0, p1)), y0 + a * (y1 - y0)
    return ks[-1][1], ks[-1][2]


def active(t: float, keys) -> bool:
    return _key(keys[0])[0] <= t <= _key(keys[-1])[0] + TAIL_S


def drive(world, name: str, t: float, keys) -> None:
    if active(t, keys):
        pos, yaw = pose_at(t, keys)
        world.set_pose(name, pos, yaw)


def carry(t0: float, a, b, *, lift: float = 0.10, dur: float = 1.6, yaw: float = 0.0, yaw_b: float | None = None) -> list:
    """Pick at ``a``, lift, move over, lower at ``b`` (centres): a hand moving an object."""
    yb = yaw if yaw_b is None else yaw_b
    up_a = (a[0], a[1], max(a[2], b[2]) + lift)
    up_b = (b[0], b[1], max(a[2], b[2]) + lift)
    return [(t0, a, yaw), (t0 + 0.3 * dur, up_a, yaw), (t0 + 0.7 * dur, up_b, yb), (t0 + dur, b, yb)]


def park(i: int, height: float, row: int = 0) -> tuple[float, float, float]:
    """A resting spot on the floor under the table (out of every camera), one per prop index."""
    return (0.05 + 0.12 * (i % 7), 0.50 + 0.10 * (row + i // 7), FLOOR_Z + height / 2 + 0.002)


def yaw_deg(d: float) -> float:
    return math.radians(d)
