"""Joint-command demonstration with one measured grasp-offset correction.

After pickup, this privileged script reads the upper/lower body roots and the
measured left TCP once. It shifts the remaining Cartesian waypoints to account
for where the upper brick actually sits in the fingers. This is feedforward
alignment, not visual inference or a claim that Dream has object-pose access.
The grasp offset is assumed to stay fixed during the remaining motion; no later
slip or brick rotation is corrected. Motion timing and grip commands stay fixed.
No object pose, force, attachment state or scoring state is changed by the policy.
"""
from __future__ import annotations
import argparse
from dataclasses import replace
import json
import math
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY, SHOULDER, WRIST
from yam_arena.demo import ArmPlan, Waypoint
from config import CONFIG
import telemetry

POLICY_NAME = 'brick_join_demo'
HOLD_SECONDS = 3.
ENABLE_POSE_CORRECTION = True
MAX_ALIGNMENT_CORRECTION_M = .015


def correct_after_pickup(plan, sample, observation_time):
    """Shift only future TCP targets, returning an auditable correction event.

    The place target puts the upper body centre one body height above the
    measured lower body centre, with the task's configured horizontal offset.
    The upper-to-TCP displacement is measured in the hand frame and rotated
    into the unchanged target grip orientation. All later targets receive the
    same translation, preserving insertion depth, proof lift and final place.
    Earlier targets stay fixed, so the carry segment smoothly approaches its
    corrected endpoint without a target jump at the pickup/carry boundary.
    """
    event = {'kind': 'measured_grasp_offset', 'observation_time': float(observation_time),
             'applied': False, 'privileged_demo_only': True}
    if not math.isfinite(event['observation_time']):
        return dict(event, observation_time=None, reason='observation timestamp is malformed')
    if sample is None:
        return dict(event, reason='no measured sample available')
    try:
        sample_time = float(sample['time'])
        age = float(observation_time)-sample_time
        if not math.isfinite(sample_time) or age < -1e-9 or age > .075:
            return dict(event, reason='measured sample is stale or newer than the observation')
        hand = sample['hands'][0]  # Runner records left, then right.
        upper = np.asarray(sample['upper']['pos'], dtype=float)
        lower = np.asarray(sample['lower']['pos'], dtype=float)
        tcp = np.asarray(hand['tcp'], dtype=float)
        rotation = np.asarray(hand['tcp_rotation'], dtype=float)
        if (any(v.shape != (3,) or not np.isfinite(v).all() for v in (upper, lower, tcp))
                or rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5)
                or abs(np.linalg.det(rotation)-1.) > 1e-5):
            return dict(event, reason='measured pose is malformed')
        if np.linalg.norm(upper-tcp) > .035:
            return dict(event, reason='upper brick is outside the grasp neighbourhood')
        first = next(i for i, w in enumerate(plan.waypoints) if w.label == 'align over lower block')
        place = next(w for w in plan.waypoints if w.label == 'set joined pair down')
        # All post-pickup waypoints use the same downward grip orientation.
        from yam_arena.demo import grip_rotation
        target_rotation = grip_rotation(place.yaw)
        measured_offset_local = rotation.T@(upper-tcp)
        upper_target = lower + np.r_[CONFIG['target_offset'], CONFIG['body_height']]
        corrected_place_tcp = upper_target-target_rotation@measured_offset_local
        delta = corrected_place_tcp-np.asarray(place.pos)
        event.update(sample_time=sample_time, measured_upper_root=upper.tolist(),
                     measured_lower_root=lower.tolist(), measured_tcp=tcp.tolist(),
                     measured_upper_in_tcp=measured_offset_local.tolist(),
                     desired_upper_root=upper_target.tolist(), correction_m=delta.tolist(),
                     assumption='The measured grasp offset remains fixed; no later slip or object rotation is corrected.')
        if not np.isfinite(delta).all() or np.linalg.norm(delta) > MAX_ALIGNMENT_CORRECTION_M:
            return dict(event, reason='required translation exceeds the 15 mm correction limit')
        for i in range(first, len(plan.waypoints)):
            waypoint = plan.waypoints[i]
            plan.waypoints[i] = replace(waypoint, pos=tuple(np.asarray(waypoint.pos)+delta))
        return dict(event, applied=True, reason='shifted remaining TCP waypoints once')
    except (KeyError, TypeError, ValueError, IndexError, StopIteration) as exc:
        return dict(event, reason=f'measured correction unavailable: {type(exc).__name__}')

def make_waypoints():
    c = CONFIG
    top, height = c['table_top'], c['body_height']
    x, y = c['upper_xy']
    tx, ty = np.asarray(c['lower_xy']) + c['target_offset']
    four_stud_join = c['required_studs'] == 4
    # The four-stud 3 mm press trial slipped upward about 1.85 mm in the pads.
    # Grip that piece 1 mm higher and command 1 mm compression per side.
    # Keep the earlier 0.5 mm squeeze/22 mm pickup for one- and two-stud joins.
    # Actuator gains and limits remain identical to the evaluated policy.
    grasp_z = .023 if four_stud_join else .022
    squeeze = .002 if four_stud_join else .001  # Total gap reduction, metres.
    finger = -(c['upper_shape'][1] * c['pitch'] - .0002 - squeeze) / 2
    pick = (x, y, top + grasp_z)
    hover = (x, y, top + .11)
    carry = (tx, ty, top + .11)
    above = (tx, ty, top + height + grasp_z + .007)
    # Loaded joint tracking and grasp slip left four studs partly seated.
    # Additional commanded overtravel supplies insertion force through the
    # ordinary joint controller. It never moves a brick directly.
    press_overtravel = .006 if four_stud_join else .003
    seat = (tx, ty, top + height + grasp_z - press_overtravel)
    proof = (tx, ty, top + height + grasp_z + .045)
    place = (tx, ty, top + height + grasp_z)
    return [
        Waypoint(1.8, hover, FINGER_OPEN, 'approach'),
        Waypoint(1.8, pick, FINGER_OPEN, 'descend'),
        Waypoint(.8, pick, finger, 'grasp'),
        Waypoint(1.6, hover, finger, 'lift upper block'),
        Waypoint(.4, hover, finger, 'hold pickup'),
        Waypoint(1.8, carry, finger, 'align over lower block'),
        Waypoint(1.6, above, finger, 'approach studs'),
        Waypoint(2., seat, finger, 'press sockets onto studs'),
        Waypoint(.8, seat, finger, 'hold seated'),
        Waypoint(1.8, proof, finger, 'lift by upper block only'),
        Waypoint(1.4, proof, finger, 'show unsupported connection'),
        Waypoint(1.8, place, finger, 'set joined pair down'),
        Waypoint(.8, place, FINGER_OPEN, 'release'),
        Waypoint(1.5, carry, FINGER_OPEN, 'withdraw'),
    ]

def make_plan():
    return ArmPlan(make_waypoints(), station.BASES['left'])

def check_reach(control_hz=20.):
    plan = make_plan()
    actions = [np.r_[READY, 1.]]
    for step in range(1, math.ceil(plan.duration * control_hz) + 1):
        actions.append(plan.action7(step / control_hz)[0])
    a = np.asarray(actions)
    speeds = np.max(np.abs(np.diff(a[:, :6], axis=0)) * control_hz, axis=0)
    limits = np.array([SHOULDER['velocity']] * 3 + [WRIST['velocity']] * 3)
    grip_speed = float(np.max(np.abs(np.diff(a[:, 6]))) * control_hz)
    passed = (np.isfinite(a).all() and np.all(a[:,:6] >= ik.LOWER) and np.all(a[:,:6] <= ik.UPPER)
              and np.all((a[:,6] >= 0) & (a[:,6] <= 1)) and np.all(speeds <= limits)
              and grip_speed <= GRIPPER['velocity'] / abs(FINGER_OPEN)
              and plan.worst_ik_mm < .5 and plan.worst_ik_deg < .1)
    return dict(passed=bool(passed), validation='Kinematics only; grasp and connection require physics trials',
                plan_seconds=plan.duration, worst_ik_position_mm=plan.worst_ik_mm,
                worst_ik_rotation_deg=plan.worst_ik_deg, maximum_joint_speeds=speeds.tolist(),
                joint_speed_limits=limits.tolist(), maximum_normalized_grip_speed=grip_speed)

def brick_policy(enable_pose_correction: bool = ENABLE_POSE_CORRECTION):
    from yam_arena.scripted import YamScriptedPolicy
    class BrickPolicy(YamScriptedPolicy):
        def __init__(self):
            super().__init__(hold_s=HOLD_SECONDS)
            self.info = replace(self.info, name=POLICY_NAME)
        def bind(self, info):
            super().bind(info)
            if not info.is_simulated or '-station-' not in (info.environment_id or ''):
                raise ValueError('This demonstration requires the simulated YAM station')
        def reset(self, scene):
            plan = make_plan()
            self._plans, self._end = {'left': plan}, plan.duration
            self._correction_done = False
            self._correction_start = 0.
            for waypoint in plan.waypoints:
                if waypoint.label == 'align over lower block':
                    break
                self._correction_start += waypoint.duration
            telemetry.demo_events.clear()
        def act(self, observation):
            state_time = float(observation.state_time)
            if not self._correction_done and state_time >= self._correction_start-1e-9:
                self._correction_done = True
                if enable_pose_correction:
                    event = correct_after_pickup(self._plans['left'], telemetry.latest_sample, state_time)
                else:
                    event = dict(kind='measured_grasp_offset', observation_time=state_time,
                                 applied=False, reason='pose correction disabled', privileged_demo_only=True)
                telemetry.demo_events.append(event)
            return super().act(observation)
    return BrickPolicy()

def register_policy():
    from inspect_robots.registry import register
    register('policy', POLICY_NAME)(brick_policy)
    return POLICY_NAME

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', required=True, action='store_true')
    parser.parse_args()
    result = check_reach()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
