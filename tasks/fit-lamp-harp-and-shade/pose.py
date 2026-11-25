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
    if not getattr(stock_rotation, "_harp_grip_pose", False):
        def grip_rotation(yaw=0.):
            if isinstance(yaw, GripRotation):
                return yaw.rotation
            return stock_rotation(yaw)

        grip_rotation._harp_grip_pose = True
        demo.grip_rotation = grip_rotation
    return GripRotation(rotation)


def ready_pose(arm):
    position, rotation = ik.tcp_pose(READY, station.BASES[arm])
    return tuple(position), as_waypoint_rotation(rotation)


def install_expert(task_path, stages, replan, steer=None):
    """Let this task's scripted policy retarget waypoints and refine right-arm targets."""
    from yam_arena.scripted import YamScriptedPolicy

    if getattr(YamScriptedPolicy,"_harp_expert_installed",False):
        return
    YamScriptedPolicy._harp_expert_installed = True
    reset,act = YamScriptedPolicy.reset,YamScriptedPolicy.act

    def reset_with_calibration(policy,scene):
        reset(policy,scene)
        policy._harp_calibrations = []
        if policy._task is None or policy._task.path.resolve() != task_path:
            return
        elapsed = 0.
        for index,waypoint in enumerate(policy._plans["left"].waypoints):
            if waypoint.label in stages:
                policy._harp_calibrations.append((elapsed,index,stages[waypoint.label]))
            elapsed += waypoint.duration

    def act_with_calibration(policy,observation):
        pending = getattr(policy,"_harp_calibrations",())
        if pending and observation.state_time+policy._dt >= pending[0][0]:
            _,index,stage = pending.pop(0)
            replacements = replan(stage)
            for side,plan in policy._plans.items():
                plan.waypoints[index:] = replacements[side][index:]
        if steer is None or policy._task is None or policy._task.path.resolve() != task_path:
            return act(policy,observation)
        right = policy._plans["right"]
        target = right.target
        right.target = lambda time: steer(target(time))
        try:
            return act(policy,observation)
        finally:
            del right.target

    YamScriptedPolicy.reset = reset_with_calibration
    YamScriptedPolicy.act = act_with_calibration
