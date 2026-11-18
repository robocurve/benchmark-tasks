"""Full-orientation waypoints for ``yam_scripted`` demos (yaw + pitch + roll, or any rotation). NumPy only.

Ship this file next to the task file and import it there (the accepted pattern of PR #115 ``wrist.py`` and
PR #1880 ``tilt.py``)::

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from pitch_demo import Waypoint  # noqa: E402

    Waypoint(1.5, (x, y, z), FINGER_OPEN, "side approach", yaw=0.0, pitch=math.pi / 2)

``pitch_demo.Waypoint`` is the stock :class:`yam_arena.demo.Waypoint` plus three optional fields:

- ``pitch`` (rad): leans the approach axis off straight down, about the jaw (finger-closing) axis. At ``yaw=0``
  positive pitch swings the fingertips toward world +x (away from the arm bases); ``pi/2`` is a level side grip
  pointing +x with the fingers closing along world y.
- ``roll`` (rad): then turns the grip about its own (leaned) approach axis. At ``pitch=pi/2, roll=pi/2`` the
  fingers close vertically.
- ``rot``: a full TCP rotation (3x3 matrix or XYZW quaternion) that overrides yaw/pitch/roll.

So the TCP rotation is ``grip_rotation(yaw) @ Rx(pitch) @ Rz(roll)`` in the gripper frame (x = closing axis,
z = approach). ``yaw`` keeps its stock meaning. With ``pitch == roll == 0`` and no ``rot`` the waypoint is exactly a
stock Waypoint (its ``yaw`` stays a plain float), so stock and full-orientation waypoints mix in one plan; the stock
ArmPlan slerps between them.

How it works: the stock ``ArmPlan.target`` builds each rotation with ``yam_arena.demo.grip_rotation(w.yaw)``. An
oriented waypoint stores its rotation on its ``yaw`` value (a float subclass equal to the yaw) and importing this
module wraps ``grip_rotation`` to return that rotation when present and to call the original otherwise. The patch is
in-process only (nothing on disk changes), idempotent, and checked at import: it raises if the installed yam_arena
no longer builds rotations this way. Accepted precedents for this exact mechanism: PR #1880 (``tilt.py``) and
PR #282 (``bench-vise/task.py`` ``Orient``).

Check every plan offline with :func:`check_plan` (worst IK position / rotation error per arm) before a GPU run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import yam_arena.demo as _demo
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

__all__ = ["Waypoint", "rotation", "hinge_turn", "check_plan"]

_X = np.array([1.0, 0.0, 0.0])
_Z = np.array([0.0, 0.0, 1.0])


class _Oriented(float):
    """A yaw (rad) that also carries the full TCP rotation of its waypoint."""

    rotation: np.ndarray

    def __new__(cls, yaw: float, rotation: np.ndarray):
        obj = super().__new__(cls, yaw)
        obj.rotation = np.array(rotation, dtype=float)
        return obj

    def __reduce__(self):
        return (_Oriented, (float(self), self.rotation))


def _as_matrix(rot) -> np.ndarray:
    r = np.asarray(rot, dtype=float)
    if r.shape == (4,):
        return quat_xyzw_to_matrix(r)
    if r.shape != (3, 3) or not np.allclose(r @ r.T, np.eye(3), atol=1e-6) or np.linalg.det(r) < 0:
        raise ValueError("rot must be an XYZW quaternion or a proper 3x3 rotation matrix")
    return r


def rotation(yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """TCP rotation: the stock straight-down grip turned ``yaw`` about the vertical, leaned ``pitch`` about its jaw
    axis, then turned ``roll`` about its approach axis (all rad)."""
    r = _ORIGINAL(float(yaw))
    if pitch:
        r = r @ axis_angle_matrix(_X, pitch)
    if roll:
        r = r @ axis_angle_matrix(_Z, roll)
    return r


@dataclass
class Waypoint(_demo.Waypoint):
    pitch: float = 0.0  # rad: approach axis leaned off straight down about the jaw axis (+ = toward +x at yaw 0)
    roll: float = 0.0  # rad: then turned about the leaned approach axis
    rot: np.ndarray | None = field(default=None, repr=False)  # full TCP rotation (3x3 or XYZW); overrides the angles

    def __post_init__(self) -> None:
        yaw = float(self.yaw)
        if self.rot is not None:
            self.yaw = _Oriented(yaw, _as_matrix(self.rot))
        elif self.pitch or self.roll:
            self.yaw = _Oriented(yaw, rotation(yaw, self.pitch, self.roll))
        else:
            self.yaw = yaw


def hinge_turn(pos, rot, hinge, axis, angle: float, *, steps: int, duration: float, finger: float,
               label: str = "turn") -> list[Waypoint]:
    """Waypoints that swing a held part ``angle`` rad about the line through ``hinge`` along ``axis`` (world).
    ``pos``/``rot`` is the TCP pose at the start of the swing; the TCP moves rigidly with the part. ``steps`` cuts
    the arc into that many waypoints (the stock ArmPlan interpolates each one in a straight line)."""
    pos, hinge = np.asarray(pos, float), np.asarray(hinge, float)
    r0 = _as_matrix(rot)
    out = []
    for k in range(1, steps + 1):
        turn = axis_angle_matrix(np.asarray(axis, float), angle * k / steps)
        p = hinge + turn @ (pos - hinge)
        out.append(Waypoint(duration / steps, tuple(p), finger, f"{label} {k}/{steps}", rot=turn @ r0))
    return out


def check_plan(plan: dict, bases: dict, dt: float = 0.05) -> dict[str, tuple[float, float]]:
    """Play a demo() plan through the stock ArmPlan IK offline: worst (position mm, rotation deg) error per arm.
    Keep both within a few mm / degrees; larger means a pose is out of reach."""
    out = {}
    for arm, waypoints in plan.items():
        p = _demo.ArmPlan(list(waypoints), bases[arm])
        for k in range(int(p.duration / dt) + 2):
            p.action7(k * dt)
        out[arm] = (round(p.worst_ik_mm, 2), round(p.worst_ik_deg, 2))
    return out


# -- the patch: grip_rotation honours a rotation carried on the yaw ----------------------------------------------

_ORIGINAL = getattr(_demo.grip_rotation, "_full_orient_original", _demo.grip_rotation)


def _grip_rotation(yaw: float = 0.0) -> np.ndarray:
    rot = getattr(yaw, "rotation", None)
    return rot.copy() if rot is not None else _ORIGINAL(yaw)


_grip_rotation._full_orient_original = _ORIGINAL  # type: ignore[attr-defined]
_demo.grip_rotation = _grip_rotation


def _self_check() -> None:
    """Fail loudly at import if the stock ArmPlan no longer takes its rotations from grip_rotation(w.yaw)."""
    w = Waypoint(1.0, (0.3, 0.3, 0.9), 0.0, "check", pitch=math.pi / 6, roll=0.2)
    _, rot, _, _ = _demo.ArmPlan([w], (0.0, 0.305, 0.75)).target(10.0)
    if not np.allclose(rot, w.yaw.rotation, atol=1e-9):
        raise RuntimeError("pitch_demo: yam_arena.demo.ArmPlan ignores oriented waypoints on this stack")


_self_check()
