"""Hole-punch scoring and the disc-release rule. NumPy only, so it is tested without the simulator.

Each hole is a 6 mm disc pre-scored in the card at its mark. It sits on a slide along the card's normal, held by a
force drive that resists the punch with up to 30 N (geometry.CUT_FORCE_N), so the punch must push it down through
the card. A disc is cut free (its slide switched off) only when, at the same moment:
  - the punch has pushed it the card's full thickness and on by PAST (0.5 mm), and the punch tip is that far below
    the card's underside, so below the bed's top face;
  - the disc's centre is within 1.5 mm of that punch's axis; and
  - the card lies on the bed there: the disc's place in the card is no more than 0.5 mm above its seated height.
    (Below it is allowed, up to 2 mm: under the full press the simulated card sinks into the bed by up to about
    1.5 mm at the punches, which a real card on a steel bed cannot do; a card held up off the bed is still refused.)
A hole counts once its disc has been cut free, is more than 3 mm from its place in the card and more than 4 mm below
the bed's top face (in the die / chip cavity).
"""

from __future__ import annotations

import numpy as np

from . import geometry as g

SEAT_TOL = 0.0005    # raised above the bed: refused beyond this
SINK_TOL = 0.0020    # pressed into the bed under the punch load (simulation only): tolerated up to this
OUT_OF_CARD = 0.003
BELOW_BED = 0.004


def quat_rotate(q_xyzw, v):
    q = np.asarray(q_xyzw, float)
    u, w = q[:3], q[3]
    v = np.asarray(v, float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def quat_conj(q_xyzw):
    q = np.asarray(q_xyzw, float)
    return np.r_[-q[:3], q[3]]


def disc_home(card_pose, k):
    """Where disc k belongs, in the card as it lies now (env frame)."""
    card_pose = np.asarray(card_pose, float)
    return card_pose[:3] + quat_rotate(card_pose[3:], g.slug_offset(k))


def disc_depth(card_pose, slug_pos, k):
    """How far disc k has been pushed out of the card along the card's normal (m, positive toward the bed)."""
    card_pose = np.asarray(card_pose, float)
    local = quat_rotate(quat_conj(card_pose[3:]), np.asarray(slug_pos, float) - card_pose[:3])
    return float(g.slug_offset(k)[2] - local[2])


def release_ok(card_pose, head_tip_z, slug_pos, k):
    """True when disc k may be cut free this step (see the module docstring)."""
    home = disc_home(card_pose, k)
    through = disc_depth(card_pose, slug_pos, k) >= g.T + g.PAST
    tip_below = head_tip_z <= home[2] - g.T / 2 - g.PAST
    on_axis = float(np.linalg.norm(home[:2] - g.pin_xy(k))) <= g.TOL
    dz = home[2] - (g.BED + g.T / 2)
    seated = -SINK_TOL <= dz <= SEAT_TOL
    return bool(through and tip_below and on_axis and seated)


def punched(card_pose, slug_pos, k, released):
    """Disc k has left the card through the die."""
    if not released:
        return False
    home = disc_home(card_pose, k)
    return bool(np.linalg.norm(np.asarray(slug_pos, float) - home) > OUT_OF_CARD and slug_pos[2] < g.BED - BELOW_BED)


class PunchScore:
    """0.5 per hole up to 0.5; 1.0 (success) only once both holes are punched and stay so for ``hold_s`` seconds of
    sim time. Before the hold is complete two holes score 0.5."""

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
            if not self.released[k] and release_ok(card_pose, head_tip_z, slug_positions[k], k):
                self.released[k], self.released_at[k] = True, round(float(t), 3)
                newly.append(k)
        holes = sum(punched(card_pose, slug_positions[k], k, self.released[k]) for k in (0, 1))
        if holes == 2:
            self.both_since = float(t) if self.both_since is None else self.both_since
        else:
            self.both_since = None
        self.success = self.success or (self.both_since is not None and float(t) - self.both_since >= self.hold_s - 1e-9)
        score = 1.0 if self.success else (0.5 if holes else 0.0)
        self.best = max(self.best, score)
        return score, newly
