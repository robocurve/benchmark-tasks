"""Where each stick meets its pad, and the stroke joint that swings it there (yam_arena's inverse kinematics on the
YAM's URDF; NumPy and yam_arena, no Isaac Sim).

The stick lies across the gripper's finger pads at its tape mark (the grip point is the gripper's centre point,
``GRIP_DEPTH`` toward the palm): along the gripper frame's y axis, the fingers closing along x and the gripper
pointing along z. Each arm plays with its gripper pointing sideways toward the robot's centre line (wrist outside),
so the finger pads are on the stick's upper and lower sides and joint 6, the wrist roll, whose axis passes through the
grip point, swings the stick up and down about it.
"""

import math
from functools import lru_cache

import numpy as np

from . import fixed_scene as F
from . import layout as L

GRIP_DEPTH = .004                # the stick's axis this far past the gripper's centre point, toward the palm
MARGIN_DEG = 6.                  # every joint at least this far from its limits at the contact pose ...
THROW_ROOM = 1.4                 # ... with this much of joint 6 past it (the throw) ...
LIFT_ROOM = .5                   # ... and this much above it (the lift)
HAND_Y = .07                     # each grip point at least this far on its own side of the centre line
SEEDS = [np.array(s, dtype=float) for s in (
    (0, 1, 1, -.6, 0, 0), (0, 1.6, 1.3, -1.2, 0, 0), (0, 2.1, 1.0, -.4, 1.5, -.1), (0, 2.1, 1.0, -.4, -1.5, .1),
    (.35, 2.2, .8, 1.4, -1.2, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.4, 2.5, 1.4, -1.6, 1.1, 0))]
# The gripper turned so the finger pads close up and down: toward the centre line for each arm.
GAMMA = {"left": -math.pi / 2, "right": math.pi / 2}


def stick_dir(heading, pitch):
    return np.array((math.cos(pitch) * math.cos(heading), math.cos(pitch) * math.sin(heading), -math.sin(pitch)))


def grip_rotation(s, gamma):
    """Gripper rotation (columns: x closing, y along the stick, z pointing) holding the stick along ``s``, the
    fingers closing along cos(gamma) h + sin(gamma) u (h horizontal across the stick, u up across it)."""
    s = s / np.linalg.norm(s)
    h = np.cross((0., 0., 1.), s)
    h /= np.linalg.norm(h)
    u = np.cross(s, h)
    c = math.cos(gamma) * h + math.sin(gamma) * u
    return np.column_stack((c, s, np.cross(c, s)))


def stick_in_gripper():
    """The stick body's rotation in the gripper frame: +x (toward the tip) along +y_g, +y along the closing axis."""
    return np.array(((0., 1., 0.), (1., 0., 0.), (0., 0., -1.)))


def bases():
    return {arm: np.asarray(F.BASES[arm], dtype=float) for arm in F.ARMS}


def solve(point, R, base, seeds, iters=200):
    from yam_arena import ik

    quat = ik.matrix_to_quat_xyzw(R)
    best = None
    for seed in seeds:
        q, err, rerr = ik.solve(point, quat, seed, base, iters=iters)
        margin = math.degrees(float(np.minimum(q - ik.LOWER, ik.UPPER - q).min()))
        ok = err < 1e-3 and rerr < math.radians(1.)
        key = (ok, margin if ok else -err)
        if best is None or key > best[0]:
            best = (key, q, err, rerr, margin)
    return best[1], best[2], best[3], best[4]


def tcp_of_tip(tip_centre, s):
    """The grip point for a tip sphere centre and stick direction."""
    return tip_centre - (L.TIP_OUT - L.TIP_R) * s


class Stroke:
    """One arm on one pad: the joints at which its stick's tip touches the aim point, and joint 6's sense."""

    def __init__(self, arm, pad, q_contact, heading, pitch):
        self.arm, self.pad = arm, pad
        self.base = bases()[arm]
        self.q_c = np.asarray(q_contact, dtype=float)
        self.heading, self.pitch = heading, pitch
        self.s_g = np.array((0., 1., 0.))
        c, n = F.pad_frame(pad)
        self.n = np.asarray(n)
        self.contact_tip = self.tip(self.q_c)
        eps = 1e-4
        dq = self.q_c.copy()
        dq[5] += eps
        g = (self.tip(dq) - self.contact_tip) @ self.n / eps
        self.sign = 1. if g > 0 else -1.
        self.lever = abs(g)

    def tip(self, q):
        from yam_arena import ik

        p, R = ik.tcp_pose(q, self.base)
        return p - GRIP_DEPTH * R[:, 2] + R @ self.s_g * (L.TIP_OUT - L.TIP_R)

    def q_at(self, phi):
        """The joints with joint 6 ``phi`` rad above (positive) or past (negative) the contact angle."""
        from yam_arena import ik

        q = self.q_c.copy()
        q[5] = float(np.clip(q[5] + self.sign * phi, ik.LOWER[5] + .01, ik.UPPER[5] - .01))
        return q

    def gain(self, phi):
        return float((self.tip(self.q_at(phi)) - self.contact_tip) @ self.n)

    def phi_for(self, lift):
        """Joint 6's lift (rad) that raises the tip ``lift`` metres along the pad's normal."""
        lo, hi = 0., 1.2
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if self.gain(mid) < lift else (lo, mid)
        return (lo + hi) / 2


def aim_point(pad, heading, pitch):
    """The tip sphere's centre at contact: the snare's and tom's head centres; on the hi-hat, its bow 60 mm in from the
    centre toward the robot."""
    c, n = F.pad_frame(pad)
    c, n = np.asarray(c, dtype=float), np.asarray(n)
    if pad == "hihat":
        s = stick_dir(heading, pitch)
        hor = np.array((s[0], s[1], 0.)) / math.hypot(s[0], s[1])
        r = L.HIHAT_AIM_IN
        rise = L.HIHAT_BOW_RISE * (L.HIHAT_R - r) / (L.HIHAT_R - L.HIHAT_BELL_R)
        c = c - r * hor + rise * n
    return c + L.TIP_R * n


def find_contact(arm, pad, headings_deg=(0., -15., 15., -30., 30., -45., -60., -75.), pitches_deg=(12., 18., 24.)):
    """Search headings (from the arm's own direction to the pad) and pitches for a contact pose meeting the margins;
    the first good one in this order, or the best margin."""
    from yam_arena import ik

    base = bases()[arm]
    c, _ = F.pad_frame(pad)
    natural = math.atan2(c[1] - base[1], c[0] - base[0])
    side = 1. if arm == "left" else -1.
    best = None
    for p in pitches_deg:
        for d in headings_deg:
            h = natural + math.radians(d)
            s = stick_dir(h, math.radians(p))
            R = grip_rotation(s, GAMMA[arm])
            tip = aim_point(pad, h, math.radians(p))
            tcp = tcp_of_tip(tip, s) + GRIP_DEPTH * R[:, 2]
            q, err, rerr, margin = solve(tcp, R, base, SEEDS)
            if not (err < 1e-3 and rerr < math.radians(1.)) or margin < MARGIN_DEG or side * tcp[1] < HAND_Y:
                continue
            st = Stroke(arm, pad, q, h, math.radians(p))
            up = (ik.UPPER[5] - q[5]) if st.sign > 0 else (q[5] - ik.LOWER[5])
            down = (q[5] - ik.LOWER[5]) if st.sign > 0 else (ik.UPPER[5] - q[5])
            if up < LIFT_ROOM or down < THROW_ROOM:
                continue
            if best is None or margin > best[0]:
                best = (margin, st)
            if margin > 12.:
                return st, margin
    if best is None:
        raise RuntimeError(f"no contact pose for the {arm} stick on the {pad}")
    return best[1], best[0]


# Found by ``find_contact`` (about a minute of inverse kinematics) and frozen: (joints at contact, heading and pitch of
# the stick in degrees, the smallest joint margin in degrees). ``tests/test_scene.py`` checks them on the GPU machine.
CONTACT = {
    ("left", "hihat"): ((-0.32717, 0.93901, 0.21786, 0.72071, 0.96495, 1.25701), 16.0, 18.0, 12.5),
    ("right", "snare"): ((0.25913, 1.56991, 0.27403, 1.29533, -1.10679, -1.36186), -11.7, 12.0, 15.7),
    ("right", "tom"): ((0.31256, 1.8157, 0.50198, 1.31359, 0.01505, -1.36135), -73.0, 12.0, 14.7),
}


@lru_cache(maxsize=None)
def contact_poses():
    """{(arm, pad): (Stroke, joint margin in degrees)} for the left stick on the hi-hat and the right stick on the
    snare and the tom (the frozen ``CONTACT``)."""
    return {key: (Stroke(key[0], key[1], q, math.radians(h), math.radians(p)), m)
            for key, (q, h, p, m) in CONTACT.items()}
