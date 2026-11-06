"""Scripted motions for the demo and the checks, played by ``--policy yam_scripted``.

- :func:`pick_place`: a straight-down pick-and-place whose grip closes over 1.2 s and opens over 1.0 s, slow enough
  for the gripper limit of ``inspect-robots run``'s default guardrails (5 % of its range per step).
- :func:`freeze`: the same sequence, stopped where the arm is at a given time and held until another.
- :func:`back_off`: the same sequence of moves; the move under way at a given time is abandoned, the arm lifts and
  moves aside, waits, then starts that move again.

``yam_scripted`` follows the waypoints in time (smoothstep between them), so a plan knows where the arm is at any
moment. NumPy only.
"""

from __future__ import annotations

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint


def start_tcp(arm: str) -> tuple:
    """Where the arm's TCP is in the ready pose."""
    return tuple(float(v) for v in ik.tcp_pose(np.asarray(READY, float), np.asarray(station.BASES[arm]))[0])


def pick_place(pick, place, travel_z: float, speed: float = 1.25, yaw: float = 0.0, fingers: float = FINGER_OPEN,
               label: str = "") -> list[Waypoint]:
    (px, py, pz), (qx, qy, qz), k = pick, place, 1.0 / speed
    return [
        Waypoint(1.5 * k, (px, py, travel_z), fingers, f"to {label}".strip(), yaw),
        Waypoint(1.5 * k, (px, py, pz), fingers, "descend", yaw),
        Waypoint(1.2, (px, py, pz), FINGER_CLOSED, "close", yaw),
        Waypoint(1.2 * k, (px, py, travel_z), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8 * k, (qx, qy, travel_z), FINGER_CLOSED, "carry", yaw),
        Waypoint(1.2 * k, (qx, qy, qz), FINGER_CLOSED, "lower", yaw),
        Waypoint(1.0, (qx, qy, qz), fingers, "release", yaw),
        Waypoint(1.0 * k, (qx, qy, travel_z), fingers, "retreat", yaw),
    ]


def duration(waypoints) -> float:
    return float(sum(w.duration for w in waypoints))


def _cut(waypoints, t: float, start):
    """Split ``waypoints`` at time ``t``: those before, the one under way cut short to end where the arm is at ``t``,
    the rest of that one (from there to its end) and the ones after. Returns (before, rest, where, finger)."""
    elapsed, prev = 0.0, tuple(start)
    for i, w in enumerate(waypoints):
        if elapsed + w.duration > t:
            a = (t - elapsed) / w.duration
            s = a * a * (3 - 2 * a)  # the smoothstep yam_scripted follows
            where = tuple(p + s * (q - p) for p, q in zip(prev, w.pos))
            finger = w.finger if a > 0.5 else (waypoints[i - 1].finger if i else FINGER_OPEN)
            head = [Waypoint(t - elapsed, where, finger, w.label, w.yaw)] if t > elapsed else []
            tail = [Waypoint(elapsed + w.duration - t, w.pos, w.finger, w.label, w.yaw)]
            return waypoints[:i] + head, tail + waypoints[i + 1:], where, finger
        elapsed += w.duration
        prev = w.pos
    return list(waypoints), [], tuple(prev), waypoints[-1].finger if waypoints else FINGER_OPEN


def freeze(waypoints, stop_at: float, resume_at: float, start) -> list[Waypoint]:
    """``waypoints`` with the arm held still where it is from ``stop_at`` to ``resume_at``."""
    before, rest, where, finger = _cut(waypoints, stop_at, start)
    yaw = (rest or before)[0].yaw if (rest or before) else 0.0
    return before + [Waypoint(resume_at - stop_at, where, finger, "hold still", yaw)] + rest


def back_off(moves, stop_at: float, resume_at: float, start, aside, travel_z: float) -> list[Waypoint]:
    """Play ``moves`` (lists of waypoints, each from above an object back to above its place) one after another; at
    ``stop_at`` give up the move under way (it must not be holding anything yet), lift, move to ``aside`` (x, y), wait
    until ``resume_at``, then start that move again and carry on."""
    out, t = [], 0.0
    for k, move in enumerate(moves):
        d = duration(move)
        if t <= stop_at < t + d:
            here = out[-1].pos if out else tuple(start)
            before, _, where, finger = _cut(move, stop_at - t, here)
            if finger == FINGER_CLOSED:
                raise ValueError("back_off would let go of a held object; choose another time")
            yaw = move[0].yaw
            out += before + [
                Waypoint(1.0, (where[0], where[1], travel_z), FINGER_OPEN, "back off: up", yaw),
                Waypoint(1.2, (aside[0], aside[1], travel_z), FINGER_OPEN, "back off: aside", yaw),
            ]
            out.append(Waypoint(max(0.1, resume_at - stop_at - 2.2), (aside[0], aside[1], travel_z), FINGER_OPEN,
                                "wait", yaw))
            return out + [w for m in moves[k:] for w in m]
        out += move
        t += d
    return out
