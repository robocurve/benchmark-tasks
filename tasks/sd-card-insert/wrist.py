"""wrist: scripted-demo waypoints whose grip tilts forward and rolls, for holding a flat card level.

``yam_arena.demo.Waypoint`` grips straight down, turned ``yaw`` about the vertical. :class:`TiltWaypoint` adds
``pitch`` and ``roll``. The down grip is first turned ``roll`` about its approach axis (the vertical), then tilted
``pitch`` about world y, from straight down (``pitch=0``) to pointing forward (``pitch=pi/2``, the approach along world
+x), then turned ``yaw`` about the vertical. With ``roll=0`` the fingers close along world y. A forward grip with
``roll=pi/2`` closes them vertically, one pad above a flat card and one below it.

Importing this module extends ``ArmPlan.target`` (which ``yam_scripted`` uses to play ``demo()``) to read the pitch
and roll. Plans without a ``TiltWaypoint`` behave exactly as before. NumPy only.

Adapted from the wrist module of the cutters-off-pegboard task (the roll is new).
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
    roll: float = 0.0  # rad about the approach axis, applied before the pitch: pi/2 closes a forward grip vertically


def tilt_rotation(yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """The TCP rotation of the down grip turned ``roll`` about the vertical, tilted ``pitch`` toward +x (about world
    y), then turned ``yaw`` about the vertical."""
    rot = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -pitch) @ demo.grip_rotation(roll)
    return rot if yaw == 0.0 else axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ rot


def _rotation(w: demo.Waypoint) -> np.ndarray:
    return tilt_rotation(w.yaw, w.pitch, w.roll) if isinstance(w, TiltWaypoint) else demo.grip_rotation(w.yaw)


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
