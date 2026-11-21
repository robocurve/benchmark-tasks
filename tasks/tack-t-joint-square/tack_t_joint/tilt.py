"""Tilted grips for the scripted demo. The stock yam_scripted policy builds every waypoint's gripper rotation from
``yam_arena.demo.grip_rotation(w.yaw)``: straight down, turned about the vertical. Standing a flat plate on its edge and
holding a torch at a fillet's 45 degree work angle both need the gripper leaned over, so a waypoint here carries its full
rotation on its ``yaw`` (a float that still equals the yaw), and importing this module wraps ``grip_rotation`` to return
that rotation when there is one. Plain floats behave as before; nothing is changed on disk; the import fails loudly if
the stock planner stops building rotations this way."""

import math

import numpy as np
import yam_arena.demo as _demo
from yam_arena.urdf import axis_angle_matrix

_STOCK = getattr(_demo.grip_rotation, "_stock", _demo.grip_rotation)


class Lean(float):
    """A yaw (rad) that also carries the gripper's full rotation."""

    def __new__(cls, yaw, rotation):
        obj = super().__new__(cls, yaw)
        obj.rotation = np.array(rotation, float)
        return obj


def rotation(yaw=0.0, tilt=0.0, heading=0.0):
    """The straight-down grip turned ``yaw`` about the vertical, then leaned ``tilt`` (rad) from straight down so the
    fingertips swing toward the horizontal direction ``heading`` (rad, world; 0 is +x). A negative tilt leans the other way."""
    down = _STOCK(float(yaw))
    if abs(tilt) < 1e-12:
        return down
    toward = np.array((math.cos(heading), math.sin(heading), 0.0))
    return axis_angle_matrix(np.cross((0.0, 0.0, -1.0), toward), tilt) @ down


def leaned(yaw=0.0, tilt=0.0, heading=0.0):
    """The value to put in a Waypoint's ``yaw``."""
    return float(yaw) if abs(tilt) < 1e-12 else Lean(yaw, rotation(yaw, tilt, heading))


def _grip_rotation(yaw=0.0):
    rot = getattr(yaw, "rotation", None)
    return rot.copy() if rot is not None else _STOCK(yaw)


_grip_rotation._stock = _STOCK
_demo.grip_rotation = _grip_rotation

_w = _demo.Waypoint(1.0, (0.3, 0.3, 0.9), 0.0, "check", leaned(0.0, 0.5, 0.0))
if not np.allclose(_demo.ArmPlan([_w], (0.0, 0.305, 0.75)).target(10.0)[1], _w.yaw.rotation, atol=1e-9):
    raise RuntimeError("tack_t_joint.tilt: the stock ArmPlan ignores leaned waypoints on this stack")
