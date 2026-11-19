"""Full gripper rotations for the scripted demo. Created 11_16_26 (America/New_York).

The stock yam_scripted policy turns each waypoint's ``yaw`` into a straight-down gripper through
``yam_arena.demo.grip_rotation``. Holding a bottle by its side and tilting it to the down tube's angle needs other
rotations, so a waypoint here may carry a whole rotation matrix on its ``yaw``: a float subclass (still equal to a yaw)
with a ``rotation`` attribute. Importing this module wraps ``grip_rotation`` to hand that rotation back when present;
a plain float behaves as before. Nothing on disk changes, and the import raises if the stock planner stops building
rotations this way."""

import numpy as np
import yam_arena.demo as _demo

_STOCK = getattr(_demo.grip_rotation, "_stock", _demo.grip_rotation)


class Turned(float):
    """A yaw (rad) that also carries the TCP's full rotation matrix."""

    def __new__(cls, yaw, rotation):
        obj = super().__new__(cls, yaw)
        obj.rotation = np.array(rotation, float)
        return obj


def frame(tips, closing):
    """TCP rotation with the fingertips along `tips` and the fingers closing along `closing` (made square to `tips`).
    In the stock straight-down grip the TCP z axis is the fingertip direction and its x axis the closing direction."""
    f = np.asarray(tips, float)
    f = f / np.linalg.norm(f)
    c = np.asarray(closing, float)
    c = c - f * float(np.dot(c, f))
    c = c / np.linalg.norm(c)
    return np.column_stack((c, np.cross(f, c), f))


def turned(rotation):
    r = np.asarray(rotation, float)
    return Turned(float(np.arctan2(r[1, 0], r[0, 0])), r)


def _grip_rotation(yaw=0.0):
    rot = getattr(yaw, "rotation", None)
    return rot.copy() if rot is not None else _STOCK(yaw)


_grip_rotation._stock = _STOCK
_demo.grip_rotation = _grip_rotation

if not np.allclose(frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0)), _STOCK(0.0), atol=1e-9):
    raise RuntimeError("bike_cage.grip: the stock straight-down grip is not the frame this module assumes")
_w = _demo.Waypoint(1.0, (0.3, -0.3, 0.9), 0.0, "probe", turned(frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))))
if not np.allclose(_demo.ArmPlan([_w], (0.0, -0.305, 0.75)).target(10.0)[1], _w.yaw.rotation, atol=1e-9):
    raise RuntimeError("bike_cage.grip: the stock ArmPlan ignores turned waypoints on this stack")
