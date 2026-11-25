"""Expose the exact starting orientation to the task's stock waypoint player."""

from yam_arena import demo, ik, station
from yam_arena.constants import READY


class GripRotation(float):
    def __new__(cls, rotation):
        yaw = super().__new__(cls, 0.)
        yaw.rotation = rotation
        return yaw


def as_waypoint_rotation(rotation):
    stock_rotation = demo.grip_rotation
    if not getattr(stock_rotation, "_a2_grip_pose", False):
        def grip_rotation(yaw=0.):
            if isinstance(yaw, GripRotation):
                return yaw.rotation
            return stock_rotation(yaw)

        grip_rotation._a2_grip_pose = True
        demo.grip_rotation = grip_rotation
    return GripRotation(rotation)


def ready_pose(arm):
    position, rotation = ik.tcp_pose(READY, station.BASES[arm])
    return tuple(position), as_waypoint_rotation(rotation)
