"""YAM pickup-and-roll demonstration, with an offline IK reach check.

The v3 candidate uses a faster release for a visible marked-face turnover.
The separate offline check establishes only kinematic feasibility. This policy
applies only joint targets: it never moves the die directly or changes the scorer.

The factory ``roll_die_policy()`` returns an Inspect Robots policy. Import it in
the task runner and pass that object to ``inspect_robots.eval``. Inspect Robots
does not resolve ``file.py:function`` policy names; ``register_policy()`` is an
optional helper for a runner that invokes its CLI in the same Python process.

Offline: PYTHONPATH=/path/to/yam-isaac-lab-arena python demo_policy.py --check
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY, SHOULDER, WRIST
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

# Tunable motion settings. Keep the task's object layout synchronized with these.
TOP = TABLE_TOP["station"]
DIE_XY = (0.20, 0.16)
DIE_EDGE = 0.016
GRASP_TCP_HEIGHT = 0.016  # Pad tips extend 13 mm below the TCP: 3 mm table clearance.
GRASP_FINGER_TARGET = -0.0075  # 15 mm target gap: gentle preload on the 16 mm cube.
LIFT_HEIGHT = 0.11
LIFT_HOLD_SECONDS = 0.60
RELEASE_START = (0.24, 0.14, TOP + 0.18)
RELEASE_END = (0.255, 0.14, TOP + 0.35)
# Roll about world X, perpendicular to the initial world-Y jaw closing axis.
# Moving the opposed pads around the die transfers tumble through their normal
# forces; turning around the closing axis relies on pad torsional friction.
WINDUP_ROLL_DEG = 10.0
FOLLOWTHROUGH_ROLL_DEG = -80.0
FLICK_SECONDS = 0.22
# Hold the gentle grasp through the first 35% of the stroke, then open smoothly.
# This delays release until the wrist has built speed without changing its path.
RELEASE_GRIP_DELAY_FRACTION = 0.35
# Each finger ends 22 mm from its closed position: a 44 mm final gap.
RELEASE_FINGER_TARGET = -0.022
FINAL_HOLD_SECONDS = 3.0
POLICY_NAME = "roll_die_demo"


@dataclass
class TiltWaypoint(Waypoint):
    """Upstream Cartesian waypoint plus roll about world X, in degrees."""

    roll_deg: float = 0.0
    grip_delay_fraction: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.grip_delay_fraction) or not 0.0 <= self.grip_delay_fraction < 1.0:
            raise ValueError("grip_delay_fraction must be finite and in [0, 1)")

    def rotation(self) -> np.ndarray:
        return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(self.roll_deg)) @ grip_rotation(
            self.yaw
        )


class TiltArmPlan(ArmPlan):
    """Keep upstream IK/action packing; interpolate complete target rotations."""

    def target(self, t: float):
        previous_position = self.start_pos
        previous_rotation = self.start_rot
        previous_finger = self.finger_start
        for waypoint in self.waypoints:
            position = np.asarray(waypoint.pos, dtype=float)
            rotation = waypoint.rotation()
            if t < waypoint.duration:
                fraction = t / waypoint.duration
                blend = ik.smoothstep(fraction)
                grip_fraction = max(0.0, (fraction - waypoint.grip_delay_fraction) / (1.0 - waypoint.grip_delay_fraction))
                grip_blend = ik.smoothstep(grip_fraction)
                return (
                    previous_position + blend * (position - previous_position),
                    ik.slerp_matrix(previous_rotation, rotation, blend),
                    previous_finger + grip_blend * (waypoint.finger - previous_finger),
                    waypoint.label,
                )
            t -= waypoint.duration
            previous_position, previous_rotation, previous_finger = position, rotation, waypoint.finger
        return previous_position, previous_rotation, previous_finger, "done"


def make_waypoints(flick_seconds: float = FLICK_SECONDS) -> list[TiltWaypoint]:
    """Lift, hold, then roll the wrist while moving upward and opening the jaws.

    The upward stroke increases free-flight time. The roll is about a horizontal
    axis, so this aims for end-over-end tumbling rather than a yaw-only spin.
    Actual release timing and tumble depend on contact dynamics and need GPU QA.
    """
    if not math.isfinite(flick_seconds) or flick_seconds <= 0:
        raise ValueError("flick_seconds must be a finite positive duration")
    hover = (*DIE_XY, TOP + LIFT_HEIGHT)
    pick = (*DIE_XY, TOP + GRASP_TCP_HEIGHT)
    return [
        TiltWaypoint(1.50, hover, FINGER_OPEN, "approach"),
        TiltWaypoint(1.50, pick, FINGER_OPEN, "descend"),
        TiltWaypoint(0.80, pick, GRASP_FINGER_TARGET, "grasp"),
        TiltWaypoint(1.40, hover, GRASP_FINGER_TARGET, "lift"),
        TiltWaypoint(LIFT_HOLD_SECONDS, hover, GRASP_FINGER_TARGET, "hold lifted"),
        TiltWaypoint(1.20, RELEASE_START, GRASP_FINGER_TARGET, "carry"),
        TiltWaypoint(0.70, RELEASE_START, GRASP_FINGER_TARGET, "wind up", roll_deg=WINDUP_ROLL_DEG),
        TiltWaypoint(
            flick_seconds,
            RELEASE_END,
            RELEASE_FINGER_TARGET,
            "roll upward and release",
            roll_deg=FOLLOWTHROUGH_ROLL_DEG,
            grip_delay_fraction=RELEASE_GRIP_DELAY_FRACTION,
        ),
        TiltWaypoint(0.50, RELEASE_END, FINGER_OPEN, "open clear", roll_deg=FOLLOWTHROUGH_ROLL_DEG),
        TiltWaypoint(1.00, RELEASE_END, FINGER_OPEN, "wait for landing", roll_deg=FOLLOWTHROUGH_ROLL_DEG),
    ]


def make_plan(flick_seconds: float = FLICK_SECONDS) -> TiltArmPlan:
    return TiltArmPlan(make_waypoints(flick_seconds), station.BASES["left"])


def check_reach(control_hz: float = 20.0, flick_seconds: float = FLICK_SECONDS) -> dict[str, Any]:
    """Check sampled IK errors, joint limits and target speed against arm limits.

    This checks the desired trajectory, not collisions, tracking, grasp, release,
    or task score. It intentionally imports neither Inspect Robots nor Isaac Sim.
    """
    if not math.isfinite(control_hz) or control_hz <= 0:
        raise ValueError("control_hz must be a finite positive frequency")
    plan = make_plan(flick_seconds)
    actions = [np.r_[READY, 1.0]]
    for step in range(1, math.ceil(plan.duration * control_hz) + 1):
        action, _ = plan.action7(step / control_hz)
        actions.append(action)
    actions = np.asarray(actions)
    speeds = np.max(np.abs(np.diff(actions[:, :6], axis=0)) * control_hz, axis=0)
    speed_limits = np.array([SHOULDER["velocity"]] * 3 + [WRIST["velocity"]] * 3)
    finite = bool(np.isfinite(actions).all())
    within_limits = bool(np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER))
    normalized_grip = bool(np.all((actions[:, 6] >= 0.0) & (actions[:, 6] <= 1.0)))
    grip_step = float(np.max(np.abs(np.diff(actions[:, 6]))))
    grip_step_limit = GRIPPER["velocity"] / abs(FINGER_OPEN) / control_hz
    result = {
        "validation": "offline kinematics only; contact physics and full score are unverified",
        "control_hz": control_hz,
        "plan_seconds": plan.duration,
        "additional_policy_hold_seconds": FINAL_HOLD_SECONDS,
        "worst_ik_position_mm": plan.worst_ik_mm,
        "worst_ik_rotation_deg": plan.worst_ik_deg,
        "maximum_joint_target_speeds_rad_s": speeds.tolist(),
        "joint_speed_limits_rad_s": speed_limits.tolist(),
        "finite_actions": finite,
        "within_joint_limits": within_limits,
        "normalized_grip": normalized_grip,
        "maximum_normalized_gripper_target_step": grip_step,
        "normalized_gripper_step_limit": grip_step_limit,
        "within_gripper_target_speed_limit": grip_step <= grip_step_limit,
        "within_joint_target_speed_limits": bool(np.all(speeds <= speed_limits)),
    }
    result["passed"] = bool(
        finite
        and within_limits
        and normalized_grip
        and plan.worst_ik_mm < 0.5
        and plan.worst_ik_deg < 0.1
        and np.all(speeds <= speed_limits)
        and grip_step <= grip_step_limit
    )
    return result


def roll_die_policy(*, hold_s: float = FINAL_HOLD_SECONDS, flick_seconds: float = FLICK_SECONDS):
    """Build the custom simulated YAM policy, keeping offline imports lightweight."""
    from yam_arena.scripted import YamScriptedPolicy

    if not math.isfinite(hold_s) or hold_s < 0:
        raise ValueError("hold_s must be a finite nonnegative duration")
    make_waypoints(flick_seconds)  # Validate the tunable duration before a trial starts.

    class RollDiePolicy(YamScriptedPolicy):
        def __init__(self):
            super().__init__(hold_s=hold_s)
            self.info = replace(self.info, name=POLICY_NAME)

        def bind(self, embodiment_info):
            super().bind(embodiment_info)
            if not embodiment_info.is_simulated or "-station-" not in (embodiment_info.environment_id or ""):
                raise ValueError("roll_die_demo requires the simulated YAM station with a task file")

        def reset(self, scene):
            plan = make_plan(flick_seconds)
            self._plans = {"left": plan}
            self._end = plan.duration

    return RollDiePolicy()


def register_policy() -> str:
    """Register this factory before invoking the CLI in this same process."""
    from inspect_robots.registry import register

    register("policy", POLICY_NAME)(roll_die_policy)
    return POLICY_NAME


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", required=True, help="run the offline reachability check")
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--flick-seconds", type=float, default=FLICK_SECONDS)
    args = parser.parse_args()
    report = check_reach(args.control_hz, args.flick_seconds)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)
