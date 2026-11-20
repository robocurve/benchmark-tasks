"""tiltgrip: tilted and horizontal grips for yam_arena demos, without changing yam_arena. NumPy only.

yam_arena's scripted demos give every waypoint a grip pointing straight down, turned ``yaw`` about the vertical:
``ArmPlan.target`` builds each waypoint's rotation with ``demo.grip_rotation(w.yaw)``. The 6-DOF YAM arm and
``ik.solve`` accept any orientation, so the limit is only in how a waypoint names its grip. This module adds
:class:`Grip`, a ``float`` that *is* the waypoint's yaw and also carries a full TCP rotation:

    from tiltgrip import Grip
    Waypoint(1.5, (0.32, 0.30, top + 0.30), FINGER_OPEN, "reach the rim", Grip.deg(pitch=60))

Importing the module wraps ``yam_arena.demo.grip_rotation`` so that a Grip yields its own rotation and every other
yaw yields exactly what it did before. ``Waypoint``, ``ArmPlan``, ``pick_place_waypoints`` and the ``yam_scripted``
policy are untouched and need nothing new: a task file's ``demo()`` returns ordinary Waypoints whose ``yaw`` happens
to be a Grip. Copy this file into a task folder next to the task file and import it there (``sys.path`` as for
``benchkit``).

Why this is a legitimate demo, not a simulator edit:

- It only chooses the *target orientation* of the gripper. ``ArmPlan.action7`` still solves IK on the URDF within
  the joint limits and sends joint targets through the embodiment's normal 14-D joint action. Tilting the wrist is
  a motion the real arm makes with the same six joints; nothing is teleported, no object, joint limit, gain,
  contact or physics setting is changed, and no simulator source is edited.
- An unreachable orientation is not faked: the IK error grows and ``ArmPlan.worst_ik_mm`` / ``worst_ik_deg`` report
  it. :func:`ik_check` plays a plan the way the policy does and returns those numbers; keep them under 2 mm and 2 deg.

Orientation convention (world frame: +x forward from the arm bases, +z up):

- ``yaw`` turns the straight-down grip about the vertical, as before (0 closes the fingers along world y).
- ``pitch`` then turns it about the gripper's own closing axis: positive pitch swings the fingertips forward along
  the yawed heading (+x at yaw 0, i.e. away from the robot); 90 deg points the grip horizontally forward, negative
  values tilt it back toward the robot. The fingers still close horizontally.
- ``roll`` then turns it about the palm normal (perpendicular to approach and closing): positive roll swings the
  fingertips toward +y of the yawed heading (left at yaw 0). The closing axis tilts with it.
- ``flip=True`` adds 180 deg about the approach axis: the same grasp for a symmetric parallel gripper, but a
  different wrist (joint 6) solution; try it when one way is past a joint limit.
- ``Grip.from_matrix(R)`` takes any TCP rotation (columns: closing axis, palm normal, approach axis).
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena import demo as _demo
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

_DOWN = quat_xyzw_to_matrix(np.asarray(_demo.DOWN_GRIP, dtype=float))  # columns: closing, palm normal, approach
_X, _Y, _Z = np.eye(3)


def _orthonormal(r) -> np.ndarray:
    r = np.asarray(r, dtype=float)
    if r.shape != (3, 3):
        raise ValueError(f"a grip rotation is a 3x3 matrix, got shape {r.shape}")
    u, _, vt = np.linalg.svd(r)
    out = u @ vt
    if np.linalg.det(out) < 0 or np.abs(out - r).max() > 1e-3:
        raise ValueError("a grip rotation must be a proper rotation matrix")
    return out


class Grip(float):
    """A waypoint yaw (rad) that also carries the full TCP rotation of the grip.

    Its float value is the yaw, so anything that reads ``Waypoint.yaw`` as a number still sees the heading. Arithmetic
    on it returns a plain float and drops the tilt: build a new Grip instead.
    """

    def __new__(cls, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0, *, flip: bool = False):
        rot = axis_angle_matrix(_Z, float(yaw)) @ _DOWN @ axis_angle_matrix(_X, float(pitch))
        rot = rot @ axis_angle_matrix(_Y, -float(roll))
        if flip:
            rot = rot @ np.diag([-1.0, -1.0, 1.0])
        return cls._make(float(yaw), rot, (float(yaw), float(pitch), float(roll), bool(flip)))

    @classmethod
    def _make(cls, yaw: float, rot: np.ndarray, angles):
        self = float.__new__(cls, yaw)
        rot = np.array(rot, dtype=float)
        rot.setflags(write=False)
        self._rot, self._angles = rot, angles
        return self

    @classmethod
    def deg(cls, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0, *, flip: bool = False) -> Grip:
        """The same grip with angles in degrees."""
        return cls(math.radians(yaw), math.radians(pitch), math.radians(roll), flip=flip)

    @classmethod
    def from_matrix(cls, rot) -> Grip:
        """Any TCP rotation (world frame). Its float value is the grip's heading: the yaw that built it, for a grip
        whose pitch is within +-90 deg (read off the palm normal, which roll leaves alone)."""
        rot = _orthonormal(rot)
        palm, ahead = rot[:, 1], rot[:, 2]
        if math.hypot(palm[0], palm[1]) > 1e-6:
            yaw = math.atan2(-palm[1], -palm[0])
        elif math.hypot(ahead[0], ahead[1]) > 1e-6:  # horizontal, pointing along the heading
            yaw = math.atan2(ahead[1], ahead[0])
        else:
            yaw = 0.0
        return cls._make(yaw, rot, None)

    # yam_arena.demo.grip_rotation looks for this, so a Grip from any copy of this file works with any copy's hook.
    def __tiltgrip_rotation__(self) -> np.ndarray:
        return self._rot

    @property
    def rotation(self) -> np.ndarray:
        """TCP rotation in the world (read-only 3x3)."""
        return self._rot

    @property
    def approach(self) -> np.ndarray:
        """Unit vector the fingers point along (the pad tips are 13 mm past the TCP this way)."""
        return self._rot[:, 2].copy()

    @property
    def closing(self) -> np.ndarray:
        """Unit vector the fingers close along."""
        return self._rot[:, 0].copy()

    @property
    def tilt_deg(self) -> float:
        """Angle of the approach axis from straight down (0 down, 90 horizontal)."""
        return float(np.degrees(np.arccos(np.clip(-self._rot[2, 2], -1.0, 1.0))))

    def __reduce__(self):  # pickling and copy.deepcopy keep the rotation
        return (_rebuild, (float(self), self._rot.tolist(), self._angles))

    def __repr__(self) -> str:
        if self._angles is None:
            return f"Grip.from_matrix({np.round(self._rot, 4).tolist()})"
        yaw, pitch, roll, flip = self._angles
        parts = [f"yaw={math.degrees(yaw):.1f}", f"pitch={math.degrees(pitch):.1f}", f"roll={math.degrees(roll):.1f}"]
        return f"Grip.deg({', '.join(parts)}{', flip=True' if flip else ''})"


def _rebuild(yaw, rot, angles):
    return Grip._make(yaw, np.array(rot), tuple(angles) if angles is not None else None)


def grip_rotation(yaw: float = 0.0) -> np.ndarray:
    """``yam_arena.demo.grip_rotation`` that also understands a :class:`Grip`; any other yaw is passed through."""
    full = getattr(yaw, "__tiltgrip_rotation__", None)
    if full is not None:
        return np.array(full())
    return grip_rotation.plain(yaw)


def install() -> None:
    """Wrap ``yam_arena.demo.grip_rotation`` (once, however many copies of this file are imported)."""
    current = _demo.grip_rotation
    if getattr(current, "_tiltgrip", False):
        grip_rotation.plain = current.plain  # another copy is installed: share its original
        return
    grip_rotation.plain = current
    grip_rotation._tiltgrip = True
    _demo.grip_rotation = grip_rotation


def ik_check(waypoints, base_pos, control_hz: float | None = None):
    """Play ``waypoints`` as the ``yam_scripted`` policy does; return (worst IK mm, worst IK deg, joint actions)."""
    from yam_arena.constants import CONTROL_HZ

    hz = control_hz or CONTROL_HZ
    plan = _demo.ArmPlan(list(waypoints), base_pos)
    steps = round(plan.duration * hz) + 5
    actions = np.array([plan.action7((k + 1) / hz)[0] for k in range(steps)])
    return plan.worst_ik_mm, plan.worst_ik_deg, actions


install()
