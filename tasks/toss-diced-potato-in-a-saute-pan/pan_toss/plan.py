"""Open-loop joint plans for the pan arm. NumPy and yam_arena's URDF kinematics only (no Isaac Sim).

The pan's handle points at the arm's base, so the whole job lies in one vertical plane through the base: the plane's
coordinates are (d, z, a): the fingertip midpoint's horizontal distance from the base axis, its height, and how far the
gripper points below horizontal. Joint 1 holds the plane's heading, joints 2-4 move in it, joint 5 stays at zero and
joint 6 is turned a quarter turn so that the fingers close on the handle's flat top and bottom faces.

`JointPlan` is what the stock `yam_scripted` policy plays: one joint target per control step. The policy builds its plans
with `yam_arena.scripted.ArmPlan(waypoints, base)`; `install()` wraps that name so a demo() that returns a ready
`JointPlan` is played as it is, and any other demo still gets the stock planner.
"""

import math

import numpy as np

ROLL = math.pi / 2  # joint 6: fingers one above the other
STIFFNESS = (80.0, 80.0, 10.0)  # joints 2, 3, 4 (i2rt's position gains, N m/rad), for the payload's sag
G = 9.81
STEP_FRACTION = 0.05  # Inspect Robots' default guard lets a joint target change by 5% of its range per control step


class JointPlan:
    """Joint targets per control step: q (n, 6) rad and grip (n,) from 0 closed to 1 open, one row every dt seconds."""

    def __init__(self, q, grip, dt, labels=None):
        self.q, self.grip, self.dt = np.asarray(q, float), np.asarray(grip, float), float(dt)
        self.labels = list(labels) if labels is not None else [""] * len(self.q)
        self.worst_ik_mm = self.worst_ik_deg = 0.0

    @property
    def duration(self):
        return len(self.q) * self.dt

    def action7(self, t):
        """[j1..j6, grip] for the control step that ends at time t."""
        k = int(min(max(round(t / self.dt) - 1, 0), len(self.q) - 1))
        return np.r_[self.q[k], self.grip[k]], self.labels[k]


def install():
    """Let yam_scripted play a JointPlan: demo() returns {"left": [plan]} and the policy's planner hands it back."""
    from yam_arena import scripted

    stock = scripted.ArmPlan
    if getattr(stock, "_pan_toss", False):
        return

    def arm_plan(waypoints, base_pos, *args, **kwargs):
        if len(waypoints) == 1 and isinstance(waypoints[0], JointPlan):
            return waypoints[0]
        return stock(waypoints, base_pos, *args, **kwargs)

    arm_plan._pan_toss = True
    scripted.ArmPlan = arm_plan


class Planar:
    """The arm's kinematics in the plane through its base axis at ``heading`` (rad, world yaw)."""

    def __init__(self, base, heading, roll=ROLL):
        from yam_arena import ik

        self.roll = float(roll)  # joint 6: the default puts one finger above the other
        self.ik = ik
        self.base = np.asarray(base, float)
        self.heading = float(heading)
        self.lo, self.hi = ik.LOWER[1:4], ik.UPPER[1:4]

    def fk(self, q3):
        """(d, z, a) for joints 2-4."""
        p, r = self.ik.tcp_pose(np.array((0.0, q3[0], q3[1], q3[2], 0.0, self.roll)))
        return np.array((p[0], p[2] + self.base[2], math.atan2(-r[2, 2], r[0, 2])))

    def solve(self, target, guess):
        """Joints 2-4 reaching (d, z, a), by Newton steps from ``guess``; raises if out of reach."""
        q = np.array(guess, float)
        target = np.asarray(target, float)
        for _ in range(60):
            f = self.fk(q)
            err = target - f
            if abs(err[0]) < 2e-5 and abs(err[1]) < 2e-5 and abs(err[2]) < 2e-4:
                return q
            jac = np.zeros((3, 3))
            for i in range(3):
                dq = q.copy()
                dq[i] += 1e-6
                jac[:, i] = (self.fk(dq) - f) / 1e-6
            step = np.linalg.solve(jac.T @ jac + 1e-8 * np.eye(3), jac.T @ err)
            q = np.clip(q + np.clip(step, -0.3, 0.3), self.lo, self.hi)
        raise ValueError(f"out of reach: (d, z, a) = {np.round(target, 4).tolist()}, got {np.round(self.fk(q), 4).tolist()}")

    def joints(self, q3):
        return np.array((self.heading, q3[0], q3[1], q3[2], 0.0, self.roll))

    def sag(self, q3, mass, com_from_tcp, level_a):
        """Joint-target offsets that cancel a held payload's static sag: the arm's own weight is compensated by the
        controller, the payload's is not. ``com_from_tcp`` is its centre of mass from the fingertip midpoint in the pan's
        (x, z) axes, and ``level_a`` the gripper angle at which the pan is level."""

        def com_z(q):
            _, z, a = self.fk(q)
            phi = a - level_a  # the pan's nose-down pitch
            return z - com_from_tcp[0] * math.sin(phi) + com_from_tcp[1] * math.cos(phi)

        z0 = com_z(q3)
        out = np.zeros(3)
        for i in range(3):
            dq = np.array(q3, float)
            dq[i] += 1e-6
            out[i] = mass * G * (com_z(dq) - z0) / 1e-6 / STIFFNESS[i]
        return out


def smooth(s):
    return s * s * (3.0 - 2.0 * s)


class Builder:
    """Collects joint targets step by step. Each move is given in plane coordinates and sampled once per control step."""

    def __init__(self, planar, dt, q_start, payload=None):
        self.p, self.dt = planar, float(dt)
        self.q, self.grip, self.labels = [], [], []
        self.q_now = np.array(q_start, float)  # all six joints
        self.pose = None  # (d, z, a) once the arm is in the plane
        self.q3 = np.array(q_start[1:4], float)
        self.grip_now = 1.0
        self.payload = payload  # (mass, com_from_tcp, level_a) or None
        self.loaded = False

    def _push(self, q6, label):
        self.q.append(np.array(q6, float))
        self.grip.append(self.grip_now)
        self.labels.append(label)
        self.q_now = np.array(q6, float)

    def _target(self, pose):
        self.q3 = self.p.solve(pose, self.q3)
        q3 = self.q3
        if self.loaded and self.payload is not None:
            q3 = np.clip(q3 + self.p.sag(q3, *self.payload), self.p.lo, self.p.hi)
        return self.p.joints(q3)

    def joint_move(self, seconds, pose, label):
        """From wherever the arm is to a plane pose, interpolating the joints."""
        goal = self._target(pose)
        start = self.q_now.copy()
        n = max(1, round(seconds / self.dt))
        for k in range(1, n + 1):
            self._push(start + smooth(k / n) * (goal - start), label)
        self.pose = np.array(pose, float)

    def move(self, seconds, pose, label, grip=None, ease=True):
        """A straight move in the plane to (d, z, a); ``grip`` ramps the fingers to that opening over the move."""
        start, goal = self.pose.copy(), np.array(pose, float)
        g0 = self.grip_now
        n = max(1, round(seconds / self.dt))
        for k in range(1, n + 1):
            s = k / n
            if grip is not None:
                self.grip_now = g0 + s * (grip - g0)
            self._push(self._target(start + (smooth(s) if ease else s) * (goal - start)), label)
        self.pose = goal

    def hold(self, seconds, label, grip=None):
        self.move(seconds, self.pose, label, grip)

    def knots(self, knots, label):
        """Timed offsets from the current pose: rows of (t seconds, dd, dz, da), joined by straight lines and sampled
        once per control step; the arm ends at the last row's offset."""
        base = self.pose.copy()
        t = np.array([k[0] for k in knots], float)
        v = np.array([k[1:4] for k in knots], float)
        n = round(t[-1] / self.dt)
        for k in range(1, n + 1):
            off = np.array([np.interp(k * self.dt, t, v[:, i]) for i in range(3)])
            self._push(self._target(base + off), label)
        self.pose = base + v[-1]

    def joint_knots(self, knots, label):
        """Timed joint-target offsets from the current pose's targets: rows of (t seconds, shoulder, elbow, wrist) in rad
        (joints 2, 3, 4), joined by straight lines and sampled once per control step. The targets are clipped to the joint
        limits, so a row may ask for more than the arm can reach in order to drive a joint at its full torque. The last
        row should be all zeros: the arm is left at the pose it started from."""
        base = self._target(self.pose)
        t = np.array([k[0] for k in knots], float)
        v = np.array([k[1:4] for k in knots], float)
        for k in range(1, round(t[-1] / self.dt) + 1):
            q = base.copy()
            q[1:4] = np.clip(q[1:4] + [np.interp(k * self.dt, t, v[:, i]) for i in range(3)], self.p.lo, self.p.hi)
            self._push(q, label)

    def to_joints(self, seconds, q6, label):
        start = self.q_now.copy()
        n = max(1, round(seconds / self.dt))
        for k in range(1, n + 1):
            self._push(start + smooth(k / n) * (np.asarray(q6, float) - start), label)
        self.pose = None

    def plan(self):
        """The targets, held to the harness's per-step limit (5% of each joint's range, and of the gripper's travel, per
        control step): the default delta-limit guard would clip them to the same values, so what is planned here is
        what the arm is sent."""
        ik = self.p.ik
        step = STEP_FRACTION * (ik.UPPER - ik.LOWER) - 1e-9
        q, grip = np.array(self.q), np.array(self.grip)
        for k in range(1, len(q)):
            q[k] = np.clip(q[k], q[k - 1] - step, q[k - 1] + step)
            grip[k] = min(max(grip[k], grip[k - 1] - STEP_FRACTION), grip[k - 1] + STEP_FRACTION)
        return JointPlan(q, grip, self.dt, self.labels)
