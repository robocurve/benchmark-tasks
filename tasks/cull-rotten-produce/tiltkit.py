"""Any gripper orientation, and joint-space moves, for a task's scripted demo with stock yam-arena (copy this file
into the task folder).

yam_arena's ``Waypoint`` only knows a straight-down grip turned by ``yaw``, and every move is a straight Cartesian
line tracked by warm-started IK. The arm itself can tilt (see ``work/reach_map.py``), but a tilted pose often has a
single IK solution on a different arm configuration than the last one, which Cartesian tracking cannot cross.

- ``grip(approach, closing)`` gives a ``yaw`` value that carries a full TCP rotation;
- ``grip(..., joint=True)`` (or ``Tilt(rot, joint=True)``; a plain float yaw: ``Tilt(None, joint=True)`` keeps the
  straight-down grip) makes that waypoint a joint-space move: its pose is solved once (from many seeds, the solution
  nearest the arm's current joints) and the joints go there smoothly; the next waypoints track from it as usual.
- ``Tilt(rot, load=(mass, com))`` marks a waypoint where the hand carries ``mass`` kg whose centre of mass is at
  ``com`` in the TCP frame. yam-arena's arm compensates gravity for its own links only (as i2rt's driver does) and
  its joints are soft PD drives (80 N m/rad shoulder, 10 N m/rad wrist), so a heavy load sags the hand by centimetres;
  the plan then commands each joint past its target by the load's torque over the joint's stiffness.
- ``install()``, called once from ``demo()``, hooks yam_arena.demo for these. A plain float yaw works as before.

    from tiltkit import grip, install
    install()
    W(2.0, (x, y, z), O, "behind the handle", grip(approach=(1, 0, 0), closing=(0, -1, 0), joint=True))

TCP frame (from DOWN_GRIP): its z axis is the approach direction (fingers point along it), its x axis the direction
the fingers close along. The closing direction's sign matters to the arm though not to the grasp: straight-down
yam_scripted closes along -y; +y is the same grip turned 180 degrees, past the wrist's +-120 degree limit. Check a
new orientation with work/reach_map.py and try the other sign when IK fails. A joint move's path in space is not a
straight line: check it for collisions in the video.
"""

from __future__ import annotations

import numpy as np


class Tilt(float):
    """A yaw of 0.0 that carries a full TCP rotation matrix (``rot``; None: the straight-down grip) and whether the
    waypoint is a joint-space move."""

    def __new__(cls, rot, joint: bool = False, load=None):
        obj = super().__new__(cls, 0.0)
        obj.rot = None if rot is None else np.asarray(rot, dtype=float)
        obj.joint = joint
        obj.load = None if load is None else (float(load[0]), np.asarray(load[1], dtype=float))
        return obj


def rotation(approach, closing) -> np.ndarray:
    """The TCP rotation whose fingers point along ``approach`` and close along ``closing`` (made square to it)."""
    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, dtype=float)
    x = x - z * (x @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def grip(approach=(0, 0, -1), closing=(0, -1, 0), joint: bool = False) -> Tilt:
    return Tilt(rotation(approach, closing), joint)


def solve_near(pos, rot, q_now, base_pos, tries: int = 80):
    """The IK solution for (pos, rot) nearest ``q_now`` (joint distance), from ``q_now`` and fixed random seeds."""
    from yam_arena import ik

    quat = ik.matrix_to_quat_xyzw(rot)
    rng = np.random.default_rng(0)
    seeds = [np.asarray(q_now, float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(tries)]
    best, best_d = None, np.inf
    for q0 in seeds:
        q, pe, re = ik.solve(pos, quat, q0, base_pos=base_pos, iters=200)
        if pe < 0.001 and np.degrees(re) < 1:
            d = float(np.abs(q - q_now).sum())
            if d < best_d:
                best, best_d = q, d
    if best is None:
        raise ValueError(f"tiltkit: no IK solution for {np.round(pos, 3)}")
    return best


def sag(q, base_pos, mass: float, com) -> np.ndarray:
    """How far each joint gives under a load of ``mass`` kg at ``com`` (TCP frame), at joints ``q`` (rad)."""
    from yam_arena import ik
    from yam_arena.constants import SHOULDER, WRIST

    def point(qq):
        pos, rot = ik.tcp_pose(qq, base_pos)
        return np.asarray(pos) + np.asarray(rot) @ np.asarray(com)

    q = np.asarray(q, float)
    p0, eps = point(q), 1e-6
    jac = np.column_stack([(point(q + eps * np.eye(6)[i]) - p0) / eps for i in range(6)])
    torque = jac.T @ np.array([0.0, 0.0, -9.81 * mass])
    stiff = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    effort = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
    if np.any(np.abs(torque) > 0.9 * effort):
        print(f"tiltkit: load torque {np.round(torque, 2)} N m near the joints' limits", flush=True)
    return torque / stiff


def install() -> None:
    """Hook yam_arena.demo: waypoints may carry a Tilt, and a joint Tilt makes a joint-space move (idempotent)."""
    import yam_arena.demo as d
    from yam_arena import ik

    if getattr(d.grip_rotation, "tilt", False):
        return
    plain = d.grip_rotation

    def grip_rotation(yaw: float = 0.0) -> np.ndarray:
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else plain(float(yaw))

    grip_rotation.tilt = True
    d.grip_rotation = grip_rotation

    cartesian = d.ArmPlan.action7

    def action7(self, t: float):
        start = 0.0
        for k, w in enumerate(self.waypoints):
            if t < start + w.duration:
                break
            start += w.duration
        else:
            return cartesian(self, t)
        if not getattr(w.yaw, "joint", False):
            return cartesian(self, t)
        moves = self.__dict__.setdefault("_joint_moves", {})
        if k not in moves:
            goal = solve_near(np.asarray(w.pos, float), grip_rotation(w.yaw), self.q, self.base_pos)
            moves[k] = (self.q.copy(), goal)
        q0, q1 = moves[k]
        s = ik.smoothstep((t - start) / w.duration)
        self.q = q0 + s * (q1 - q0)
        _, _, finger, label = self.target(t)
        grip = float(np.clip(finger / d.FINGER_OPEN, 0.0, 1.0)) if d.FINGER_OPEN != d.FINGER_CLOSED else 1.0
        return np.r_[self.q, grip], label

    def loaded(self, t: float):
        cmd, label = joint_or_cartesian(self, t)
        start, prev = 0.0, None
        for w in self.waypoints:
            if t < start + w.duration:
                break
            start, prev = start + w.duration, w
        load = getattr(w.yaw, "load", None)
        if load is not None:
            # taken up smoothly over the first waypoint that carries it
            k = 1.0 if getattr(getattr(prev, "yaw", None), "load", None) is not None else ik.smoothstep(
                min((t - start) / w.duration, 1.0))
            cmd = np.array(cmd, dtype=float)
            cmd[:6] = np.clip(cmd[:6] - k * sag(cmd[:6], self.base_pos, *load), ik.LOWER, ik.UPPER)
        return cmd, label

    joint_or_cartesian = action7
    d.ArmPlan.action7 = loaded
