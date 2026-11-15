"""Hole-punch scoring and the disc-release rule. NumPy only, so it is tested without the simulator.

Each hole is a 6 mm disc pre-scored in the card at its mark, held there by a fixed joint. A disc is released (its
joint switched off) only when, at the same moment:
  - its punch's tip has come down to the card's top face (the head pressed through its spring to the card),
  - the disc's centre is within 1.5 mm of that punch's axis, and
  - the card lies flat on the bed there (the disc's centre within 0.5 mm of its seated height).
A released disc must then be driven through the die by the punch: a hole counts once its disc is more than 3 mm from
its place in the card and below the bed's top face by more than 4 mm (in the die / chip cavity).
"""

from __future__ import annotations

import numpy as np

from . import geometry as g

SEAT_TOL = 0.0005
CONTACT_TOL = 0.0002
OUT_OF_CARD = 0.003
BELOW_BED = 0.004


def quat_rotate(q_xyzw, v):
    q = np.asarray(q_xyzw, float)
    u, w = q[:3], q[3]
    v = np.asarray(v, float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def disc_home(card_pose, k):
    """Where disc k belongs, in the card as it lies now (env frame)."""
    card_pose = np.asarray(card_pose, float)
    return card_pose[:3] + quat_rotate(card_pose[3:], g.slug_offset(k))


def release_ok(card_pose, head_tip_z, k):
    """True when disc k may be released this step (see the module docstring)."""
    home = disc_home(card_pose, k)
    pressed = head_tip_z <= g.BED + g.T + CONTACT_TOL
    on_axis = float(np.linalg.norm(home[:2] - g.pin_xy(k))) <= g.TOL
    seated = abs(home[2] - (g.BED + g.T / 2)) <= SEAT_TOL
    return bool(pressed and on_axis and seated)


def punched(card_pose, slug_pos, k, released):
    """Disc k has left the card through the die."""
    if not released:
        return False
    home = disc_home(card_pose, k)
    return bool(np.linalg.norm(np.asarray(slug_pos, float) - home) > OUT_OF_CARD and slug_pos[2] < g.BED - BELOW_BED)


class PunchScore:
    """Score = holes / 2. Success (1.0) once both holes are punched and stay so for ``hold_s`` seconds of sim time."""

    def __init__(self, hold_s: float = 1.0):
        self.hold_s = float(hold_s)
        self.reset()

    def reset(self):
        self.released = [False, False]
        self.released_at = [None, None]
        self.best = 0.0
        self.both_since = None
        self.success = False

    def update(self, t, card_pose, head_tip_z, slug_positions):
        newly = []
        for k in (0, 1):
            if not self.released[k] and release_ok(card_pose, head_tip_z, k):
                self.released[k], self.released_at[k] = True, round(float(t), 3)
                newly.append(k)
        holes = sum(punched(card_pose, slug_positions[k], k, self.released[k]) for k in (0, 1))
        score = holes / 2.0
        self.best = max(self.best, score)
        if holes == 2:
            self.both_since = float(t) if self.both_since is None else self.both_since
        else:
            self.both_since = None
        self.success = self.success or (self.both_since is not None and float(t) - self.both_since >= self.hold_s - 1e-9)
        return score, newly
