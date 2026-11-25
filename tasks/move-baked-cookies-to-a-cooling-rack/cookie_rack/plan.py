"""Open-loop joint plans from gripper poses. NumPy and yam_arena's URDF kinematics only (no Isaac Sim).

Created 11_15_26 18:30 EST. A `Path` is a list of legs for one arm; each leg takes the fingertip midpoint to a position
with the gripper pointing straight down, turned `pitch` about the world's y axis (positive tips the turner's blade down),
and the fingers at `finger` (m). `Path.joints()` solves every control step's pose with yam_arena's IK and returns a
`JointPlan`, which the stock `yam_scripted` policy plays once `install()` has wrapped its planner. Nothing here reads
the simulator.
"""

import math

import numpy as np


class JointPlan:
    """Joint targets per control step: q (n, 6) rad and grip (n,) from 0 closed to 1 open, one row every dt seconds."""

    def __init__(self, q, grip, dt, labels, worst_mm=0.0, worst_deg=0.0):
        self.q, self.grip, self.dt, self.labels = np.asarray(q, float), np.asarray(grip, float), float(dt), list(labels)
        self.worst_ik_mm, self.worst_ik_deg = worst_mm, worst_deg

    @property
    def duration(self):
        return len(self.q) * self.dt

    def action7(self, t):
        k = int(min(max(round(t / self.dt) - 1, 0), len(self.q) - 1))
        return np.r_[self.q[k], self.grip[k]], self.labels[k]


def install():
    """Let yam_scripted play a JointPlan: demo() returns {"right": [plan]} and the policy's planner hands it back."""
    from yam_arena import scripted

    stock = scripted.ArmPlan
    if getattr(stock, "_cookie_rack", False):
        return

    def arm_plan(waypoints, base_pos, *args, **kwargs):
        if len(waypoints) == 1 and isinstance(waypoints[0], JointPlan):
            return waypoints[0]
        return stock(waypoints, base_pos, *args, **kwargs)

    arm_plan._cookie_rack = True
    scripted.ArmPlan = arm_plan


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


class Path:
    def __init__(self, base, dt):
        from yam_arena import ik
        from yam_arena.constants import FINGER_OPEN, READY
        from yam_arena.demo import grip_rotation

        self.ik, self.base, self.dt = ik, np.asarray(base, float), float(dt)
        self.down = grip_rotation(0.0)
        self.open = FINGER_OPEN
        self.q0 = np.array(READY, float)
        p, r = ik.tcp_pose(self.q0, self.base)
        self.legs = []  # (duration, pos, pitch, finger, label)
        self.start = (p, r)
        self.t = 0.0

    def to(self, dur, pos, pitch=0.0, finger=None, label=""):
        finger = self.legs[-1][3] if finger is None and self.legs else (self.open if finger is None else finger)
        self.legs.append((float(dur), np.asarray(pos, float), float(pitch), float(finger), label))
        self.t += float(dur)

    def home(self, dur, label="home"):
        """Back to the ready pose the arm started in, moving joint by joint."""
        self.legs.append((float(dur), None, 0.0, self.legs[-1][3], label))
        self.t += float(dur)

    def wait(self, dur, label="wait", finger=None):
        _, pos, pitch, f, _ = self.legs[-1]
        self.to(dur, pos, pitch, f if finger is None else finger, label)

    def until(self, t):
        if t > self.t + 1e-9:
            self.wait(t - self.t)

    def joints(self):
        ik = self.ik
        q = self.q0.copy()
        p0, r0 = self.start
        f0 = self.open
        qs, grips, labels = [], [], []
        worst_mm = worst_deg = 0.0
        for dur, pos, pitch, finger, label in self.legs:
            n = max(1, round(dur / self.dt))
            if pos is None:
                q_from = q.copy()
                for k in range(1, n + 1):
                    q = q_from + ik.smoothstep(k / n) * (self.q0 - q_from)
                    qs.append(q.copy())
                    grips.append(min(max(finger / self.open, 0.0), 1.0))
                    labels.append(label)
                p0, r0 = self.start
                continue
            r1 = ry(pitch) @ self.down
            for k in range(1, n + 1):
                s = ik.smoothstep(k / n)
                p = p0 + s * (pos - p0)
                r = ik.slerp_matrix(r0, r1, s)
                q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q, base_pos=self.base, iters=80, rot_weight=0.5)
                worst_mm, worst_deg = max(worst_mm, 1000 * pe), max(worst_deg, math.degrees(re))
                qs.append(q.copy())
                grips.append(min(max((f0 + s * (finger - f0)) / self.open, 0.0), 1.0))
                labels.append(label)
            p0, r0, f0 = pos, r1, finger
        return JointPlan(qs, grips, self.dt, labels, worst_mm, worst_deg)
