"""grip: scripted-demo waypoints with the gripper pointing any way, for side grips on a spool's rims.

``yam_arena.demo.Waypoint`` grips straight down, turned ``yaw`` about the vertical. :class:`HandWaypoint` names the
grip by two directions in the world: ``approach``, where the fingers point (the gripper's +z), and ``closing``, the
line the fingers close along (the gripper's x). A gripper coming at the spool from the robot's left, level, with its
fingers closing up and down is ``approach=(0, -1, 0), closing=(0, 0, 1)``. Turning ``closing`` about ``approach``
between two waypoints rolls the wrist, and whatever the fingers hold turns about the line through the fingertips.

Importing this module extends ``ArmPlan.target`` (which ``yam_scripted`` uses to play ``demo()``) to read these grips.
Plans without a ``HandWaypoint`` behave exactly as before. NumPy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from yam_arena import demo, ik

__all__ = ["HandWaypoint", "hand_rotation", "joint_moves", "solve"]


@dataclass
class HandWaypoint(demo.Waypoint):
    approach: tuple[float, float, float] = (0.0, 0.0, -1.0)  # where the fingers point
    closing: tuple[float, float, float] = (0.0, 1.0, 0.0)  # the line the fingers close along


def hand_rotation(approach, closing) -> np.ndarray:
    """The TCP rotation whose +z is ``approach`` and whose x is ``closing`` (made square to ``approach``)."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    f = np.asarray(closing, dtype=float)
    f = f - a * float(f @ a)
    n = np.linalg.norm(f)
    if n < 1e-9:
        raise ValueError("closing must not lie along approach")
    f = f / n
    return np.column_stack([f, np.cross(a, f), a])


def _rotation(w: demo.Waypoint) -> np.ndarray:
    return hand_rotation(w.approach, w.closing) if isinstance(w, HandWaypoint) else demo.grip_rotation(w.yaw)


def _hand_target(self: demo.ArmPlan, t: float):
    """(TCP position, rotation matrix, finger, label) at time t: ``ArmPlan.target`` with hand grips."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), _rotation(w)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


def solve(pos, approach, closing, q_init, base) -> np.ndarray:
    """Joint angles putting the TCP at ``pos`` with this grip, solved from ``q_init`` (which picks the arm's
    posture: elbow, wrist up or down)."""
    q, _, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(hand_rotation(approach, closing)), q_init, base_pos=base, iters=300)
    return q


def joint_moves(q_from, q_to, base, duration: float, steps: int, finger: float, label: str) -> list[HandWaypoint]:
    """Waypoints that take the arm from ``q_from`` to ``q_to`` along a straight line in joint space: the TCP poses
    at ``steps`` evenly spaced points on it. The IK, warm-started from each, then keeps the posture of ``q_to``
    instead of finding another one on the way (a big turn of the hand can otherwise end in a posture at a joint
    limit)."""
    q_from, q_to = np.asarray(q_from, dtype=float), np.asarray(q_to, dtype=float)
    out = []
    for k in range(1, steps + 1):
        pos, rot = ik.tcp_pose(q_from + (q_to - q_from) * k / steps, base)
        out.append(HandWaypoint(duration / steps, tuple(pos), finger, label, approach=tuple(rot[:, 2]),
                                closing=tuple(rot[:, 0])))
    return out


if getattr(demo.ArmPlan.target, "__name__", "") != "_hand_target":
    demo.ArmPlan.target = _hand_target
