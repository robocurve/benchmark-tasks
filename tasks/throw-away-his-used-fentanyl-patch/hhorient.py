"""hhorient: tilted and side-on grips for this task's scripted plans, kept inside this folder.

yam_arena's ``Waypoint`` carries one ``yaw`` (a straight-down grip turned about the vertical) and ``ArmPlan.target``
turns it into the TCP rotation with ``yam_arena.demo.grip_rotation``. ``Orient`` is a float subclass that also
carries a whole TCP rotation matrix; importing this module wraps ``demo.grip_rotation`` (once) so that an ``Orient``
yaw returns its matrix and every plain float yaw behaves exactly as before. Plans then pass ``yaw=tilted(...)``.
yam_arena itself is not modified (the same wrap-from-the-task pattern as merged upstream tasks that add their own
grips, e.g. #1436's allow_matrix_grips).
"""
from __future__ import annotations

import math

import numpy as np
from yam_arena import demo as _demo
from yam_arena.urdf import axis_angle_matrix


class Orient(float):
    """A waypoint yaw that also carries a full TCP rotation matrix (``rot``)."""

    def __new__(cls, yaw, rot):
        o = float.__new__(cls, yaw)
        o.rot = np.asarray(rot, float)
        return o


_plain = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _plain(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain
    _demo.grip_rotation = _grip_rotation


def plain(yaw: float = 0.0) -> np.ndarray:
    """The unmodified straight-down grip turned ``yaw`` about the vertical."""
    return _plain(float(yaw))


def tilted(yaw: float, axis, deg: float) -> Orient:
    """The straight-down grip at ``yaw``, then turned ``deg`` degrees about the world ``axis`` (through the TCP)."""
    return Orient(yaw, axis_angle_matrix(np.asarray(axis, float), math.radians(deg)) @ plain(yaw))


def approach(o) -> np.ndarray:
    """The world direction the fingers point along (the TCP frame's z axis) for a yaw or Orient."""
    m = o.rot if isinstance(o, Orient) else plain(o)
    return np.asarray(m, float)[:, 2]
