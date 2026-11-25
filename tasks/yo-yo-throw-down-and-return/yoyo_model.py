"""The yo-yo on its string: a reduced model, pure Python, shared by the scene hook and offline checks.

The yo-yo is rigid (mass M, moment of inertia I about its axle). The string is inextensible. In the hand's frame (the
loop on the fingertip), s is the string between the loop and the axle:

- Wound (rolling): the string rolls on the axle without slipping at effective radius R_EFF, so
  (M + I / R_EFF^2) s'' = M (g + a_hand_up) - (TAU_ROLL / R_EFF) sign(s'): the hand's own vertical acceleration acts
  like extra gravity, and string rub on the halves costs a small constant torque.
- Turnaround: at the string's end (s = L) the yo-yo goes round the bottom of the loop and starts winding up the other
  way; the loop's slip on the axle there keeps TURN_KEEP of its energy (relative to the hand). Arriving slower than
  DIE_SPEED it stops there and hangs, spinning down: it is dead.
- Top: wound fully (s = S_WOUND) it meets the hand and stops, then rolls down again from rest.
- Slack: if the string is longer than the distance from the loop to the yo-yo (after a release with slack), the
  yo-yo flies freely until the string is taut, then the jerk sets it rolling: s' = (v + K s'_spin) / (1 + K), the
  momentum balance of a rolling yo-yo (K = I / (M R_EFF^2)).
- Sideways: the yo-yo swings below the loop as a damped pendulum of length s.

Everything here is in SI units. No Isaac Sim import.
"""

import math

import numpy as np

G = 9.81
M = 0.0488  # kg: Duncan Butterfly (classic, plastic), 48.8 g
DIAM, WIDTH = 0.0572, 0.0324  # m: 57.2 mm across, 32.4 mm wide
R_OUT = DIAM / 2
I_AXLE = 0.6 * M * R_OUT**2  # kg m^2: the mass is mostly in the rims (estimate, 0.6 m R^2)
R_EFF = 0.0050  # m: the axle (about 3 mm radius) plus the string wound on it
K = I_AXLE / (M * R_EFF**2)  # about 19.6: a dropped yo-yo falls at g / (1 + K), about 0.48 m/s^2
L = 0.40  # m: string from the loop to the axle, unwound (the string is cut to length, as users do)
S_WOUND = R_OUT + 0.022  # m: fully wound, the yo-yo's rim just below the finger the loop is on (the loop point is mid-finger)
S_START = 0.060  # m: string out at the start, from the loop on the open finger to the yo-yo in its cradle
TAU_ROLL = 1.5e-4  # N m: string rub on the halves while rolling (estimate)
TURN_KEEP = 0.92  # energy kept through a turnaround at the bottom (estimate)
DIE_SPEED = 0.06  # m/s: slower than this at the bottom, the loop slips and the yo-yo hangs dead
SWING_DAMP = 0.6  # 1/s: sideways swing damping
SUB = 40  # integration sub-steps per control step


class YoYo:
    """State in the hand's frame. `phase`: 'held' (a physical body, not on the model), 'roll', 'slack', 'dead'."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase = "held"
        self.s = S_START  # string length out (loop to axle)
        self.u = 0.0  # ds/dt (positive: unwinding, the yo-yo going down relative to the hand)
        self.d = np.zeros(2)  # horizontal offset of the axle from the loop
        self.dv = np.zeros(2)
        self.rel = np.zeros(3)  # slack flight: axle position relative to the loop
        self.relv = np.zeros(3)
        self.turns = 0  # turnarounds at the bottom
        self.tops = 0  # climbs that ended (an apex, or meeting the hand)
        self.events = []  # (kind, value) since last read

    # -- the rolling phase ------------------------------------------------------------------------------------
    def _acc(self, u, a_up):
        f = M * (G + a_up)
        if abs(u) > 1e-6:
            f -= (TAU_ROLL / R_EFF) * math.copysign(1.0, u)
        return f / (M * (1.0 + K))

    def release(self, rel_pos, rel_vel):
        """The yo-yo leaves the hand (or a support) at rel_pos from the loop, moving at rel_vel relative to it."""
        self.rel = np.asarray(rel_pos, float).copy()
        self.relv = np.asarray(rel_vel, float).copy()
        dist = float(np.linalg.norm(self.rel))
        if dist < self.s - 1e-4:
            self.phase = "slack"
        else:
            self._taut(dist)

    def _taut(self, dist):
        dirn = self.rel / max(dist, 1e-9)
        v_out = float(self.relv @ dirn)  # the axle moving away from the loop, along the string
        spin_u = self.u  # zero after a release from the hand; the spin it kept through slack flight otherwise
        self.s = min(max(dist, S_WOUND), L)
        self.u = (v_out + K * spin_u) / (1.0 + K) if v_out > 0 else spin_u
        horiz = self.rel[:2]
        self.d = horiz.copy()
        self.dv = self.relv[:2].copy()
        self.phase = "roll"

    def step(self, dt, a_hand):
        """Advance by dt with the hand (loop) accelerating at a_hand (world, 3-vector) throughout."""
        a_hand = np.asarray(a_hand, float)
        h = dt / SUB
        for _ in range(SUB):
            if self.phase == "slack":
                acc = np.array([0.0, 0.0, -G]) - a_hand
                self.relv += acc * h
                self.rel += self.relv * h
                dist = float(np.linalg.norm(self.rel))
                if dist >= self.s:
                    self._taut(dist)
                continue
            if self.phase not in ("roll", "dead"):
                continue
            a_up = float(a_hand[2])
            if self.phase == "roll" and G + a_up < 0.0:  # the hand drops faster than the yo-yo falls: the string goes slack
                self.rel, self.relv = self.offset(), self.offset_vel()
                self.phase = "slack"
                continue
            if self.phase == "dead":
                self.u = 0.0
            else:
                self.u += self._acc(self.u, a_up) * h
            self.s += self.u * h
            if self.phase == "dead":
                pass
            elif self.s >= L and self.u > 0:
                self.s = L
                self.turns += 1
                if self.u < DIE_SPEED:
                    self.u = 0.0
                    self.phase = "dead"
                    self.events.append(("dead", 0.0))
                else:
                    self.u = -self.u * math.sqrt(TURN_KEEP)
                    self.events.append(("turn", self.u))
            elif self.s <= S_WOUND and self.u < 0:
                self.s = S_WOUND
                self.u = 0.0
                self.tops += 1
                self.events.append(("top", S_WOUND))
            # sideways pendulum of length s
            ell = max(self.s, 0.02)
            g_eff = max(G + a_up, 0.5)
            dd = -(g_eff / ell) * self.d - a_hand[:2] - SWING_DAMP * self.dv
            self.dv += dd * h
            self.d += self.dv * h
            lim = 0.6 * self.s
            n = float(np.linalg.norm(self.d))
            if n > lim:
                self.d *= lim / n
        return self.offset()

    def offset(self):
        """The axle relative to the loop (world axes)."""
        if self.phase == "slack":
            return self.rel.copy()
        dz = math.sqrt(max(self.s**2 - float(self.d @ self.d), 1e-8))
        return np.array([self.d[0], self.d[1], -dz])

    def offset_vel(self):
        if self.phase == "slack":
            return self.relv.copy()
        off = self.offset()
        # d/dt of -sqrt(s^2 - |d|^2)
        dz = -off[2]
        vz = -(self.s * self.u - float(self.d @ self.dv)) / max(dz, 1e-6)
        return np.array([self.dv[0], self.dv[1], vz])
