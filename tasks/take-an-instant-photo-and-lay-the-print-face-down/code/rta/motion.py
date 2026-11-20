"""Scripted object motion for events: smooth straight segments between poses. NumPy only."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def smoothstep(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


@dataclass(frozen=True)
class Segment:
    """``name`` moves from ``start`` to ``end`` (world, m) between ``t0`` and ``t1`` seconds."""

    name: str
    t0: float
    t1: float
    start: tuple[float, float, float]
    end: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.t1 > self.t0:
            raise ValueError(f"segment {self.name}: t1 must be > t0, got {self.t0} .. {self.t1}")

    def pos(self, t: float) -> np.ndarray:
        s = smoothstep((t - self.t0) / (self.t1 - self.t0))
        return np.asarray(self.start, float) + s * (np.asarray(self.end, float) - np.asarray(self.start, float))


def carry(name: str, t0: float, start, end, *, lift_z: float, rise_s: float, move_s: float, drop_s: float) -> list[Segment]:
    """Lift ``name`` from ``start`` straight up to ``lift_z``, move over ``end``, lower onto it."""
    sx, sy, sz = start
    ex, ey, ez = end
    t1, t2 = t0 + rise_s, t0 + rise_s + move_s
    return [
        Segment(name, t0, t1, (sx, sy, sz), (sx, sy, lift_z)),
        Segment(name, t1, t2, (sx, sy, lift_z), (ex, ey, lift_z)),
        Segment(name, t2, t2 + drop_s, (ex, ey, lift_z), (ex, ey, ez)),
    ]


def play(segments: list[Segment], t: float, world, *, dt: float) -> None:
    """Apply every segment active at ``t``; a segment also writes its exact end pose on the first control step at
    or past ``t1`` (the window is 1.5 dt wide so float accumulation of ``t`` cannot skip it)."""
    for seg in segments:
        if seg.t0 <= t < seg.t1 + 1.5 * dt:
            world.set_pose(seg.name, tuple(seg.pos(min(t, seg.t1))))


def end_time(segments: list[Segment]) -> float:
    return max((s.t1 for s in segments), default=0.0)
