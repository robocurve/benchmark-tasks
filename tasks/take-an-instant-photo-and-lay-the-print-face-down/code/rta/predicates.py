"""Scoring vocabulary over the privileged snapshot. Pure NumPy: compose these in a task's milestones(),
success() and failure(); test them on synthetic states without the simulator.

Every function takes the :class:`rta.spec.Snapshot` (``objects``) first. Positions are world metres; ``low``/``high``
are each object's world-aligned bounding box. Arms are ``"left"`` / ``"right"``; ``objects.tcp[arm]`` is the
tool centre between the finger pads and ``objects.finger[arm]`` the finger joint (0 closed .. -0.047 fully open).
"""

from __future__ import annotations

import numpy as np

FINGER_OPEN = -0.047
PAD_TIP_PAST_TCP = 0.013  # the pad tips reach this far below the TCP


# -- single objects ---------------------------------------------------------------------


def lifted(objects, name: str, rest_z: float, by: float = 0.03) -> bool:
    """The object's centre is at least ``by`` above ``rest_z`` (its resting centre height)."""
    return bool(objects[name].pos[2] > rest_z + by)


def resting_on(objects, name: str, surface_z: float, tol: float = 0.015, max_speed: float = 0.01) -> bool:
    """The object's bottom is within ``tol`` of ``surface_z`` and it is still."""
    o = objects[name]
    return bool(abs(o.low[2] - surface_z) < tol and o.resting(max_speed=max_speed))


def fell_off(objects, name: str, table_top: float, below: float = 0.2) -> bool:
    return bool(objects[name].pos[2] < table_top - below)


def within_xy(objects, name: str, centre, half: float) -> bool:
    """The object's centre is inside the square of half-width ``half`` around ``centre`` (x, y)."""
    p = objects[name].pos
    return bool(abs(p[0] - centre[0]) <= half and abs(p[1] - centre[1]) <= half)


def moved_from(objects, name: str, start, more_than: float) -> bool:
    """The object's centre is more than ``more_than`` from ``start`` (x, y)."""
    p = objects[name].pos
    return bool(np.hypot(p[0] - start[0], p[1] - start[1]) > more_than)


def distance(objects, a: str, b: str) -> float:
    return float(np.linalg.norm(objects[a].pos - objects[b].pos))


# -- containers --------------------------------------------------------------------------


def inside(objects, name: str, container: str, wall: float = 0.01, above_floor: float = 0.06, below_floor: float = 0.015) -> bool:
    """``name``'s centre is over ``container``'s footprint and its bottom sits between ``below_floor`` under and
    ``above_floor`` over the container's floor (the container's bounding-box bottom plus ``wall``)."""
    o, c = objects[name], objects[container]
    floor_top = c.low[2] + wall
    return bool(
        c.low[0] < o.pos[0] < c.high[0]
        and c.low[1] < o.pos[1] < c.high[1]
        and floor_top - below_floor < o.low[2] < floor_top + above_floor
    )


def count_inside(objects, names, container: str, **kw) -> int:
    return sum(inside(objects, n, container, **kw) for n in names)


def only_inside(objects, wanted, candidates, container: str, **kw) -> bool:
    """Exactly the ``wanted`` objects (and no other ``candidates``) are inside the container."""
    wanted = set(wanted)
    return all(inside(objects, n, container, **kw) == (n in wanted) for n in candidates)


def on_top_of(objects, name: str, base: str, xy_tol: float = 0.025, z_tol: float = 0.01) -> bool:
    """``name`` rests squarely on ``base``: centres within ``xy_tol``, bottom within ``z_tol`` of the base's top."""
    o, b = objects[name], objects[base]
    return bool(np.hypot(o.pos[0] - b.pos[0], o.pos[1] - b.pos[1]) < xy_tol and abs(o.low[2] - b.high[2]) < z_tol)


# -- grippers -----------------------------------------------------------------------------


def tcp_near(objects, arm: str, point, xy: float = 0.025, z_below: float | None = None) -> bool:
    """The arm's TCP is within ``xy`` of ``point`` horizontally (and below ``z_below`` if given)."""
    t = objects.tcp[arm]
    ok = abs(t[0] - point[0]) <= xy and abs(t[1] - point[1]) <= xy
    return bool(ok and (z_below is None or t[2] < z_below))


def gripper_closed_on(objects, arm: str, width: float, tol: float = 0.012) -> bool:
    """The fingers are closed to about ``width`` metres apart (an object of that width between the pads)."""
    opening = 2 * abs(objects.finger[arm])  # metres between the pads (0.094 fully open)
    return bool(abs(opening - width) < tol and opening > 0.004)


def holding(objects, arm: str, name: str, width: float, xy: float = 0.03, z: float = 0.04) -> bool:
    """The arm's TCP is on the object and the fingers are closed to the object's width."""
    t, o = objects.tcp[arm], objects[name]
    near = abs(t[0] - o.pos[0]) <= xy and abs(t[1] - o.pos[1]) <= xy and abs(t[2] - o.pos[2]) <= z
    return bool(near and gripper_closed_on(objects, arm, width))


def touching_pad(objects, arm: str, centre, pad_top_z: float, xy: float = 0.025, reach: float = 0.03) -> bool:
    """The fingertips are on a flat pad: TCP over it and no higher than ``reach`` above the pad's top."""
    return tcp_near(objects, arm, centre, xy=xy, z_below=pad_top_z + reach)


def any_tcp_within(objects, point, radius: float) -> list[str]:
    """Arms whose TCP is within ``radius`` (3-D) of ``point``."""
    return [arm for arm, t in objects.tcp.items() if np.linalg.norm(np.asarray(t) - np.asarray(point)) <= radius]


# -- latched bookkeeping ------------------------------------------------------------------


def first_time(state: dict, key: str, condition: bool, t: float) -> bool:
    """Record in ``state[key]`` the first time ``condition`` holds; return whether it has ever held."""
    if condition and key not in state:
        state[key] = float(t)
    return key in state


def ordered(state: dict, keys) -> bool:
    """The recorded first-times of ``keys`` are all present and increasing."""
    times = [state.get(k) for k in keys]
    return all(t is not None for t in times) and all(a < b for a, b in zip(times, times[1:]))
