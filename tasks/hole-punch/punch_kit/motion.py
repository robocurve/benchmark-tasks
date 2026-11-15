"""Reactive Cartesian motion for scripted demos (reusable kit module). NumPy only.

:class:`Arm` plays a queue of :class:`Move` s for one YAM arm through the stock adapter's IK and returns ordinary
14-D-action halves (6 joints + normalized gripper). Moves can be appended at any time, so a demo can react to the
measured state (a stall, a reading) instead of following a fixed timeline.

- a ``goto`` move (default) interpolates (smoothstep) from where the last move ended to its target, evaluated once
  when the move starts;
- a ``track`` move follows ``target(sim)`` every step with no interpolation (the caller makes the target smooth,
  e.g. a rate-limited crank angle); it runs until :meth:`Arm.interrupt` or its duration ends.

:class:`AngleServo` is the measured-angle correction for a worked part (crank, wheel, screw): the YAM arm is
compliant under the stock PD gains, so the commanded angle is the desired angle plus an integral of the measured
error, rate-limited and capped, and frozen while the error is large (the part is blocked: no wind-up).

The segment interpolation and the integral correction are adapted from the ratchet-lowering task plugin
(robocurve/benchmark-tasks PR #538, MIT, Copyright (c) 2026 YAMbition contributors; full notice in
THIRD_PARTY_NOTICES.txt).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY


@dataclass
class Move:
    duration: float
    target: Callable | None  # sim -> (pos, rot matrix); None holds the last pose
    finger: float  # finger joint target (m): FINGER_OPEN (-0.047) .. 0 closed
    label: str
    track: bool = False


class Arm:
    def __init__(self, base, q0=READY):
        self.base = np.asarray(base, float)
        self.q = np.array(q0, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.finger = FINGER_OPEN
        self.queue: list[Move] = []
        self.cur: Move | None = None
        self.t0 = 0.0
        self._from = (self.pos, self.rot, self.finger)
        self._goal = None
        self.worst_mm = 0.0
        self.label = "-"

    def push(self, *moves: Move) -> None:
        self.queue.extend(moves)

    @property
    def idle(self) -> bool:
        return self.cur is None and not self.queue

    def interrupt(self) -> None:
        """End the current move where it is now (its next pose is the last commanded one)."""
        self.cur = None

    def _next(self, sim, t):
        self.cur = self.queue.pop(0) if self.queue else None
        self.t0 = t
        self._from = (self.pos, self.rot, self.finger)
        if self.cur is not None and not self.cur.track:
            self._goal = self._from[:2] if self.cur.target is None else self.cur.target(sim)

    def act(self, sim, t) -> np.ndarray:
        if self.cur is not None and not self.cur.track and t - self.t0 >= self.cur.duration:
            self.cur = None
        if self.cur is not None and self.cur.track and t - self.t0 >= self.cur.duration:
            self.cur = None
        if self.cur is None and self.queue:
            self._next(sim, t)
        m = self.cur
        if m is not None:
            u = ik.smoothstep((t - self.t0) / m.duration) if m.duration > 0 else 1.0
            self.finger = self._from[2] + u * (m.finger - self._from[2])
            if m.track:
                self.pos, self.rot = m.target(sim)
            else:
                g_pos, g_rot = self._goal
                self.pos = self._from[0] + u * (np.asarray(g_pos) - self._from[0])
                self.rot = ik.slerp_matrix(self._from[1], g_rot, u)
            self.label = m.label
        else:
            self.label = "hold"
        q, pe, _ = ik.solve(np.asarray(self.pos, float), ik.matrix_to_quat_xyzw(self.rot), self.q,
                            base_pos=self.base, iters=60)
        self.q = q
        self.worst_mm = max(self.worst_mm, pe * 1000.0)
        return np.r_[q, float(np.clip(self.finger / FINGER_OPEN, 0.0, 1.0))]


class AngleServo:
    """command = desired + bias; d(bias)/dt = gain * (desired - measured), limited to ``rate`` deg/s, |bias| <= cap.
    Integration pauses while |error| > ``freeze_deg`` (blocked part) or while ``active`` is False."""

    def __init__(self, gain: float = 1.5, rate_dps: float = 8.0, cap_deg: float = 8.0, freeze_deg: float = 1.5):
        self.gain, self.rate, self.cap, self.freeze = gain, rate_dps, cap_deg, freeze_deg
        self.bias = 0.0

    def command(self, desired: float, measured: float, dt: float, active: bool = True) -> float:
        err = desired - measured
        if active and abs(err) <= self.freeze and math.isfinite(err):
            step = float(np.clip(self.gain * err, -self.rate, self.rate)) * dt
            self.bias = float(np.clip(self.bias + step, -self.cap, self.cap))
        return desired + self.bias
