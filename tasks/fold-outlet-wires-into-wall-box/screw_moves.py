"""The scripted demo's screw-driving part: the left hand takes the screwdriver off its rest, puts its bit on each
screw and turns it home with ratchet strokes of the wrist. Closed loop: every control step it reads where the
screwdriver's tip and the screw's head are and moves the hand's target by the error. Pure numpy."""
from __future__ import annotations

import math

import numpy as np

import outlet_logic as L
import screws as S

HZ = L.CONTROL_HZ
GRIP_BACK = 0.010  # the grip centre this far behind the middle of the handle
GRIP_FINGER = -0.006  # fingers told to close to 6 mm from centre (the flats are 13 mm out): a firm hold
OPEN_FINGER = -0.030
OVER = 0.070  # come down on the handle from this far above it
LIFT = 0.050
STAND_OFF = 0.030  # the bit waits this far in front of a screw's head
PRESS = 0.0015  # and then aims this far past the head's top
LEAD_MAX = 0.020  # the hand's target is kept within this of the hand in the servo stages (the arm sags 5-12 mm)
ROLL_BACK, ROLL_FWD = -85.0, 85.0  # the wrist's stroke (degrees about the screwdriver)
FWD_RATE, BACK_RATE = 45.0, 80.0  # degrees per second


def _toward(now, goal, step):
    d = np.asarray(goal, float) - now
    n = float(np.linalg.norm(d))
    return np.asarray(goal, float).copy() if n <= step else now + d * (step / n)


class ScrewMoves:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.stage, self.since, self.target, self.finger, self.roll = 0, 0, None, OPEN_FINGER, 0.0
        self.screw, self.forward, self.done, self.label, self.ok = 0, True, False, "over the handle", 0

    def _next(self, label: str) -> None:
        self.stage, self.since, self.label, self.ok = self.stage + 1, 0, label, 0

    def step(self, tcp, handle, tip, tops, engaged, home):
        """One control step. ``tcp``: the left hand's grip centre; ``handle``: the middle of the screwdriver's handle;
        ``tip``: its bit's tip; ``tops``: the middle of each screw head's top; ``engaged``/``home``: per screw.
        Returns (target position, roll in degrees about x, finger, label)."""
        tcp, handle, tip = (np.asarray(v, float) for v in (tcp, handle, tip))
        if self.target is None:
            self.target = tcp.copy()
        self.since += 1
        s = self.stage
        if self.done:
            return self.target, self.roll, self.finger, "hold"
        grip_at = handle + np.array([-GRIP_BACK, 0.0, 0.0])
        if s == 0:  # over the handle, fingers open
            goal = grip_at + np.array([0.0, 0.0, OVER])
            self.target = _toward(self.target, goal, 0.08 / HZ)
            if np.linalg.norm(self.target - goal) < 1e-4 and np.linalg.norm(tcp - goal) < 0.010 or self.since > 8 * HZ:
                self._next("down onto the handle")
        elif s == 1:
            self.target = _toward(self.target, grip_at + (self.target - tcp) * np.array([1.0, 1.0, 0.0]), 0.03 / HZ)
            if abs(tcp[2] - grip_at[2]) < 0.003 or self.since > 6 * HZ:
                self._next("close on the handle")
        elif s == 2:
            self.finger = max(GRIP_FINGER, self.finger + 0.02 / HZ) if self.finger < GRIP_FINGER else GRIP_FINGER
            self.finger = GRIP_FINGER if self.since > 0.6 * HZ else OPEN_FINGER + (GRIP_FINGER - OPEN_FINGER) * self.since / (0.6 * HZ)
            if self.since > 1.4 * HZ:
                self.lift_to = self.target + np.array([0.0, 0.0, LIFT])
                self._next("lift the screwdriver")
        elif s == 3:
            self.target = _toward(self.target, self.lift_to, 0.03 / HZ)
            if np.linalg.norm(self.target - self.lift_to) < 1e-4 and self.since > 2.5 * HZ:
                self._next("to the top screw")
        elif s in (4, 8):  # carry the bit to just in front of the screw's head, wrist rolled back
            i = 0 if s == 4 else 1
            self.screw = i
            goal_tip = np.asarray(tops[i], float) + np.array([-STAND_OFF, 0.0, 0.0])
            self.roll = max(ROLL_BACK, self.roll - BACK_RATE / HZ)
            err = goal_tip - tip
            self.target = self.target + np.clip(0.15 * err, -0.06 / HZ, 0.06 / HZ)
            self.ok = self.ok + 1 if np.linalg.norm(err) < 0.002 else 0
            if (self.ok >= 8 and self.roll <= ROLL_BACK) or self.since > 14 * HZ:
                self._next("bit onto the screw")
        elif s in (5, 9):
            i = self.screw
            goal_tip = np.asarray(tops[i], float) + np.array([PRESS, 0.0, 0.0])
            err = goal_tip - tip
            self.target = self.target + np.clip(0.15 * err, -0.012 / HZ, 0.012 / HZ)
            self.ok = self.ok + 1 if engaged[i] else 0
            if self.ok >= 6 or self.since > 9 * HZ:
                self.forward = True
                self._next("drive the screw")
        elif s in (6, 10):
            i = self.screw
            goal_tip = np.asarray(tops[i], float) + np.array([PRESS, 0.0, 0.0])
            self.target = self.target + np.clip(0.15 * (goal_tip - tip), -0.012 / HZ, 0.012 / HZ)
            if self.forward:
                self.roll = min(ROLL_FWD, self.roll + FWD_RATE / HZ)
                self.forward = self.roll < ROLL_FWD
            else:
                self.roll = max(ROLL_BACK, self.roll - BACK_RATE / HZ)
                self.forward = self.roll <= ROLL_BACK
            if home[i] or self.since > 60 * HZ:
                self._next("back off the screw")
        elif s in (7, 11):
            i = self.screw
            goal_tip = np.asarray(tops[i], float) + np.array([-STAND_OFF - 0.01, 0.0, 0.0])
            err = goal_tip - tip
            self.target = self.target + np.clip(0.15 * err, -0.04 / HZ, 0.04 / HZ)
            if np.linalg.norm(err) < 0.004 or self.since > 5 * HZ:
                self._next("to the bottom screw" if s == 7 else "away from the outlet")
        elif s == 12:
            if self.since == 1:
                self.away = self.target + np.array([0.0, 0.10, 0.03])  # sideways: nearer the arm's base it swings round
            self.target = _toward(self.target, self.away, 0.06 / HZ)
            self.roll = min(0.0, self.roll + BACK_RATE / HZ) if self.roll < 0 else max(0.0, self.roll - BACK_RATE / HZ)
            if np.linalg.norm(self.target - self.away) < 1e-4 or self.since > 6 * HZ:
                self.done, self.label = True, "hold"
        if s >= 4:  # servo stages: a blocked hand must not wind the target up (the tip is steered, not forced)
            lead = self.target - tcp
            n = float(np.linalg.norm(lead))
            if n > LEAD_MAX:
                self.target = tcp + lead * (LEAD_MAX / n)
        return self.target, self.roll, self.finger, self.label
