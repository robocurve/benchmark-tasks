"""wrist: scripted-demo waypoints whose grip tilts forward, for reaching into a vertical board.

``yam_arena.demo.Waypoint`` grips straight down, turned ``yaw`` about the vertical. :class:`TiltWaypoint` adds
``pitch``: the grip is tilted about the fingers' closing axis (world y at ``yaw=0``), from straight down
(``pitch=0``) to pointing forward into the board (``pitch=pi/2``, the approach along world +x). The fingers keep
closing along y.

Importing this module extends ``ArmPlan.target`` (which ``yam_scripted`` uses to play ``demo()``) to read the pitch.
Plans without a ``TiltWaypoint`` behave exactly as before. NumPy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from yam_arena import demo, ik
from yam_arena.urdf import axis_angle_matrix

__all__ = ["TiltWaypoint", "tilt_rotation"]


@dataclass
class TiltWaypoint(demo.Waypoint):
    pitch: float = 0.0  # rad: 0 points straight down, pi/2 points forward along world +x


def tilt_rotation(yaw: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    """The TCP rotation of the down grip tilted ``pitch`` toward +x (about world y), then turned ``yaw`` about the
    vertical."""
    rot = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -pitch) @ demo.grip_rotation(0.0)
    return rot if yaw == 0.0 else axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ rot


def _rotation(w: demo.Waypoint) -> np.ndarray:
    return tilt_rotation(w.yaw, w.pitch) if isinstance(w, TiltWaypoint) else demo.grip_rotation(w.yaw)


def _target(self: demo.ArmPlan, t: float):
    """(TCP position, rotation matrix, finger, label) at time t: ``ArmPlan.target`` with tilted grips."""
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


if getattr(demo.ArmPlan.target, "__name__", "") != "_target":
    demo.ArmPlan.target = _target
