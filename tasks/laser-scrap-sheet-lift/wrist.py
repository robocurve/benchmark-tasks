"""wrist: scripted-demo waypoints whose grip points forward, its jaws closing up and down, to pinch a flat sheet.

``yam_arena.demo.Waypoint`` grips straight down, turned ``yaw`` about the vertical, the fingers closing along world y
(at ``yaw`` 0). :class:`PinchWaypoint` adds ``pitch`` and ``roll``: the down grip is first turned ``roll`` about its
own approach axis, then tilted ``pitch`` toward +x about world y (0 points straight down, pi/2 points forward along
+x), then turned ``yaw``. At ``pitch`` pi/2 and ``roll`` +-pi/2 the fingers point forward and close vertically, one
pad above a sheet and one below it. Less ``pitch`` points the fingers down past the horizontal, which tips a pinched
sheet's far edge down.

Importing this module extends ``ArmPlan.target`` (which ``yam_scripted`` uses to play ``demo()``) to read the tilt.
Plans without a ``PinchWaypoint`` behave exactly as before. NumPy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from yam_arena import demo, ik
from yam_arena.urdf import axis_angle_matrix

__all__ = ["PinchWaypoint", "pinch_rotation"]

X, Y, Z = np.eye(3)


@dataclass
class PinchWaypoint(demo.Waypoint):
    pitch: float = 0.0  # rad: 0 points straight down, pi/2 forward along world +x
    roll: float = 0.0  # rad about the grip's own approach axis, before the pitch


def pinch_rotation(yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """The TCP rotation: the down grip rolled ``roll`` about its approach axis, tilted ``pitch`` toward +x about
    world y, then turned ``yaw`` about the vertical."""
    rot = axis_angle_matrix(Y, -pitch) @ demo.grip_rotation(0.0) @ axis_angle_matrix(Z, roll)
    return rot if yaw == 0.0 else axis_angle_matrix(Z, yaw) @ rot


def _rotation(w: demo.Waypoint) -> np.ndarray:
    return pinch_rotation(w.yaw, w.pitch, w.roll) if isinstance(w, PinchWaypoint) else demo.grip_rotation(w.yaw)


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
