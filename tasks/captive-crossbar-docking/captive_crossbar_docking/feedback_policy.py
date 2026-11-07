# SPDX-License-Identifier: MIT
"""Privileged measured-pose feasibility demo; output is ordinary joint actions only.

The passive carrier and crossbar are never attached, teleported or actuated.
The extra measured-pose stream is explicitly gated by the demo runtime.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import json
import math
import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import STATE_KEY, pack, split
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN
from yam_arena.demo import grip_rotation
from yam_arena.urdf import quat_xyzw_to_matrix
from . import contract as c

# Right jaws open across x, perpendicular to the captive y slide. The raised
# grasp keeps pad tips 5mm above guide cheeks, with about 23mm of tab overlap.
RIGHT_GRIP_YAW = math.pi / 2.
RIGHT_GRASP_TCP_Z_OFFSET = .004


def _bounded(vector, limit):
    return vector * min(1., limit / max(float(np.linalg.norm(vector)), 1e-12))


def _angle(rotation):
    return math.acos(float(np.clip((np.trace(rotation) - 1.) / 2., -1., 1.)))


@dataclass
class _Servo:
    q: np.ndarray
    pos: np.ndarray
    rot: np.ndarray
    grasp_pos: np.ndarray
    grasp_rot: np.ndarray
    rotation_offset: np.ndarray
    saturated_s: float = 0.
    slipped_s: float = 0.


class CaptiveCrossbarDockingDemo(PolicyBase):
    def __init__(self, mode='demo', debug_path=None):
        if mode != 'demo':
            raise ValueError('Only mode=demo is implemented')
        self._debug_path = Path(debug_path) if debug_path else None
        self._dt = .05
        self._bases = station.BASES
        low, high = np.r_[ik.LOWER, 0.], np.r_[ik.UPPER, 1.]
        self.info = PolicyInfo(name='captive_crossbar_docking_demo',
                               action_space=action_box(pack(low, low), pack(high, high)))

    def bind(self, info):
        if info.control_hz:
            self._dt = 1. / info.control_hz

    def reset(self, scene):
        self._last = (np.r_[READY, 1.], np.r_[READY, 1.])
        self._phase = 0
        self._start = self._phase_start = self._previous_time = None
        self._hold = 0.
        self._motion = None
        self._tool_paths_checked = set()
        self._servo = {}
        self._captured = {}
        self._previous_poses = {}
        self._previous_tcp = {}
        self._previous_grippers = {}
        self._release = {}
        self._debug_observation = {}

    def _next(self, now):
        self._phase += 1
        self._phase_start = now
        self._hold = 0.
        self._motion = None

    def _stable(self, condition, duration):
        self._hold = self._hold + self._dt if condition else 0.
        return self._hold >= duration - 1e-9

    def _chunk(self, meta):
        data = pack(*self._last)
        if self._debug_path is not None:
            with self._debug_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps({**self._debug_observation, **meta,
                                         'commanded_joint_state': data.tolist()}, allow_nan=False) + '\n')
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1. / self._dt)

    def _tcp(self, arm, measured, goal, opening, elapsed, duration, meta):
        """Check the entire Cartesian interpolation before executing its first step."""
        idx = 0 if arm == 'left' else 1
        if self._motion is None:
            start, start_rot = ik.tcp_pose(measured[:6], self._bases[arm])
            end_rot = grip_rotation(0. if arm == 'left' else RIGHT_GRIP_YAW)
            seed = self._last[idx][:6].copy()
            worst_pos = worst_ang = 0.
            for fraction in np.linspace(0., 1., 33)[1:]:
                s = ik.smoothstep(float(fraction))
                point = start + s * (goal - start)
                rot = ik.slerp_matrix(start_rot, end_rot, s)
                seed, error, angular = ik.solve(point, ik.matrix_to_quat_xyzw(rot), seed,
                                                base_pos=self._bases[arm], iters=96, tol=5e-5)
                worst_pos, worst_ang = max(worst_pos, error), max(worst_ang, angular)
            meta.update(path_ik_max_error_mm=worst_pos * 1000.,
                        path_ik_max_error_deg=math.degrees(worst_ang))
            if worst_pos > .003 or worst_ang > math.radians(3.):
                meta['feedback_fault'] = arm + '_full_path_ik_failed'
                return self._last[idx].copy(), False
            if self._phase in (1, 4):
                duration = max(duration, 1.5 * float(np.linalg.norm(goal - start)) / .020)
            self._motion = (start, start_rot, end_rot, duration)
        start, start_rot, end_rot, duration = self._motion
        s = ik.smoothstep(elapsed / duration)
        desired = start + s * (goal - start)
        rot = ik.slerp_matrix(start_rot, end_rot, s)
        target, error, angular = ik.solve(desired, ik.matrix_to_quat_xyzw(rot),
                                         self._last[idx][:6], base_pos=self._bases[arm],
                                         iters=64, tol=5e-5)
        actual, actual_rot = ik.tcp_pose(measured[:6], self._bases[arm])
        meta.update(tcp_error_mm=float(np.linalg.norm(actual - goal)) * 1000.,
                    ik_error_mm=error * 1000.)
        if error > .003 or angular > math.radians(3.):
            meta['feedback_fault'] = arm + '_tcp_ik_failed'
        reached = np.linalg.norm(actual - goal) < .004 and _angle(actual_rot @ end_rot.T) < math.radians(3.)
        return np.r_[target, opening], bool(reached)

    def _capture_servo(self, arm, measured, pose):
        """Capture once after contact settles, retaining commanded preload."""
        idx = 0 if arm == 'left' else 1
        tcp, tcp_rot = ik.tcp_pose(measured[:6], self._bases[arm])
        tool_rot = quat_xyzw_to_matrix(pose[3:])
        commanded, commanded_rot = ik.tcp_pose(self._last[idx][:6], self._bases[arm])
        self._servo[arm] = _Servo(
            self._last[idx][:6].copy(), commanded, commanded_rot,
            tcp_rot.T @ (pose[:3] - tcp), tcp_rot.T @ tool_rot,
            tool_rot.T @ commanded_rot,
        )

    def _track(self, arm, measured, pose, goal, rotation, speed_limit, meta):
        """Measured rigid-body servo with bounded lead, load retention and slip gates."""
        idx = 0 if arm == 'left' else 1
        tcp, tcp_rot = ik.tcp_pose(measured[:6], self._bases[arm])
        tool_rot = quat_xyzw_to_matrix(pose[3:])
        grasp = tcp_rot.T @ (pose[:3] - tcp)
        grasp_rot = tcp_rot.T @ tool_rot
        if arm not in self._servo:
            self._capture_servo(arm, measured, pose)
        state = self._servo[arm]
        if (self._phase, arm) not in self._tool_paths_checked:
            # Feasibility of the complete intended slide, before contact motion.
            seed = state.q.copy()
            worst_pos = worst_ang = 0.
            for fraction in np.linspace(0., 1., 33)[1:]:
                desired_tool = pose[:3] + fraction * (goal - pose[:3])
                desired_tool_rot = ik.slerp_matrix(tool_rot, rotation, float(fraction))
                desired_tcp_rot = desired_tool_rot @ state.grasp_rot.T
                desired_tcp = desired_tool - desired_tcp_rot @ state.grasp_pos
                seed, e, a = ik.solve(desired_tcp, ik.matrix_to_quat_xyzw(desired_tcp_rot), seed,
                                      base_pos=self._bases[arm], iters=96, tol=5e-5)
                worst_pos, worst_ang = max(worst_pos, e), max(worst_ang, a)
            meta.update({arm + '_slide_path_ik_max_error_mm': worst_pos * 1000.,
                         arm + '_slide_path_ik_max_error_deg': math.degrees(worst_ang)})
            if worst_pos > .003 or worst_ang > math.radians(3.):
                meta['feedback_fault'] = arm + '_slide_path_ik_failed'
                return self._last[idx].copy()
            self._tool_paths_checked.add((self._phase, arm))
        error = goal - pose[:3]
        # Absolute target anchored at the captured loaded command. Integrating
        # measured object yaw on every tick caused the supporting hand to creep.
        absolute_rot = rotation @ state.rotation_offset
        rot_angle = _angle(absolute_rot @ state.rot.T)
        fraction = min(1., math.radians(10.) * self._dt / max(rot_angle, 1e-12))
        commanded_rot = ik.slerp_matrix(state.rot, absolute_rot, fraction)
        velocity = _bounded(error * 3., speed_limit)
        # Both operations are on-table slides. No lift or downward windup.
        velocity[2] = np.clip(velocity[2], -.004, .004)
        # Never adopt a changed contact transform into the commanded mapping:
        # live transforms are measurements for slip detection only.
        tool_command = state.pos + state.rot @ state.grasp_pos + velocity * self._dt
        lead = tool_command - commanded_rot @ state.grasp_pos - tcp
        position_lead = float(np.linalg.norm(lead))
        angular_lead = _angle(commanded_rot @ tcp_rot.T)
        saturated = position_lead > .020 or angular_lead > math.radians(12.)
        state.saturated_s = state.saturated_s + self._dt if saturated else 0.
        drift = float(np.linalg.norm(grasp - state.grasp_pos))
        drift_angle = _angle(grasp_rot @ state.grasp_rot.T)
        slipped = drift > .008 or drift_angle > math.radians(15.)
        state.slipped_s = state.slipped_s + self._dt if slipped else 0.
        blend = min(1., .020 / max(position_lead, 1e-12),
                    math.radians(12.) / max(angular_lead, 1e-12))
        commanded_rot = ik.slerp_matrix(tcp_rot, commanded_rot, blend)
        tool_command = pose[:3] + blend * (tool_command - pose[:3])
        lead = _bounded(tool_command - commanded_rot @ state.grasp_pos - tcp, .020)
        lead[2] = np.clip(lead[2], -.003, .003)
        commanded = tcp + lead
        target, error_ik, angular_ik = ik.solve(commanded, ik.matrix_to_quat_xyzw(commanded_rot),
                                               state.q, base_pos=self._bases[arm], iters=64, tol=5e-5)
        if state.saturated_s >= 1.5:
            meta['feedback_fault'] = arm + '_servo_lead_saturated'
        elif state.slipped_s >= .30:
            meta['feedback_fault'] = arm + '_grasp_slipped'
        elif error_ik > .003 or angular_ik > math.radians(3.):
            meta['feedback_fault'] = arm + '_servo_ik_failed'
        state.q, state.pos, state.rot = target, commanded, commanded_rot
        meta.update({arm + '_tool_error_mm': (error * 1000.).tolist(),
                     arm + '_lead_mm': position_lead * 1000.,
                     arm + '_grasp_drift_mm': drift * 1000.,
                     arm + '_ik_error_mm': error_ik * 1000.})
        return np.r_[target, 0.]

    def act(self, observation):
        try:
            tools = observation.extra['demo_tools']
            poses = {name: np.asarray(tools[name + '_pose'], dtype=float) for name in c.DYNAMIC_NAMES}
            if any(p.shape != (7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:]) - 1.) > .01
                   for p in poses.values()):
                raise ValueError('invalid measured pose')
            left, right = split(np.asarray(observation.state[STATE_KEY], dtype=float))
            if any(q.shape != (7,) or not np.isfinite(q).all() for q in (left, right)):
                raise ValueError('invalid measured joints')
            now = float(observation.state_time)
            bar_q = float(tools['crossbar_q'])
            if not math.isfinite(now) or not math.isfinite(bar_q):
                raise ValueError('invalid time or joint measurement')
            if self._previous_time is not None and not 0. < now - self._previous_time <= .25:
                raise ValueError('stale or discontinuous measured feedback')
        except (KeyError, TypeError, ValueError) as exc:
            return self._chunk(dict(request_stop=True, stop_reason='demo_feedback_unavailable', detail=str(exc)))
        if self._start is None:
            self._start = self._phase_start = now
            self._last = (left.copy(), right.copy())
            self._initial_carrier = poses['carrier'].copy()
        elapsed = now - self._phase_start
        carrier, bar = poses['carrier'], poses['crossbar']
        carrier_rot = quat_xyzw_to_matrix(carrier[3:])
        handle = carrier[:3] + carrier_rot @ np.asarray(c.CARRIER_GRIP_LOCAL)
        tab = bar[:3] + quat_xyzw_to_matrix(bar[3:]) @ np.asarray(c.CROSSBAR_GRIP_LOCAL)
        right_grasp_tcp = tab + np.array([0., 0., RIGHT_GRASP_TCP_Z_OFFSET])
        initial = self._initial_carrier[:3]
        phases = [
            ('left_hover', 'tcp', 'left', handle + [0., 0., .055], 1., 1.2),
            ('left_descend', 'tcp', 'left', handle, 1., 4.2),
            ('left_close', 'grip', 'left', None, 0., .6),
            ('right_hover', 'tcp', 'right', right_grasp_tcp + [0., 0., .065], 1., 1.3),
            ('right_descend', 'tcp', 'right', right_grasp_tcp, 1., 4.9),
            ('right_close', 'grip', 'right', None, 0., .6),
            ('crossbar_center', 'bar', 'right', carrier[:3] + carrier_rot @ np.asarray(c.CROSSBAR_ORIGIN_LOCAL), 0., .3),
            ('right_release_prepare', 'prepare', 'right', None, 0., .35),
            ('right_release', 'grip', 'right', None, 1., .4),
            ('right_vertical_clear', 'tcp', 'right', tab + [0., 0., .085], 1., 1.2),
            ('right_clear', 'tcp', 'right', np.array([.23, -.20, .89]), 1., 1.2),
            ('carrier_slide', 'carrier', 'left', np.array([.440, 0., c.TABLE_TOP + .008]), 0., .3),
            ('carrier_dock', 'carrier_precise', 'left', np.asarray(c.CARRIER_FINAL_POS), 0., .3),
            ('left_release_prepare', 'prepare', 'left', None, 0., .35),
            ('left_release', 'grip', 'left', None, 1., .4),
            ('left_vertical_clear', 'tcp', 'left', handle + [0., 0., .044], 1., 1.2),
            ('left_clear', 'tcp', 'left', np.array([.24, .22, .87]), 1., 1.2),
            ('settle', 'wait', 'left', None, 1., 2.),
        ]
        if self._phase >= len(phases):
            return self._chunk(dict(phase='complete', request_stop=True, stop_reason='demo_motion_complete'))
        name, kind, arm, goal, opening, duration = phases[self._phase]
        meta = dict(phase=name, phase_s=elapsed, crossbar_q=bar_q,
                    carrier_error_mm=float(np.linalg.norm(carrier[:3] - c.CARRIER_FINAL_POS)) * 1000.)
        self._debug_observation = dict(t=now, poses={k: v.tolist() for k, v in poses.items()},
                                       joints=pack(left, right).tolist())
        if now - self._start > 115. or elapsed > 20.:
            return self._chunk({**meta, 'request_stop': True, 'stop_reason': 'demo_timeout_' + name})
        if abs(carrier[2] - initial[2]) > .012 or _angle(carrier_rot) > math.radians(8.):
            return self._chunk({**meta, 'request_stop': True, 'stop_reason': 'carrier_lift_or_tilt'})
        if self._phase <= 10 and np.linalg.norm(carrier[:2] - initial[:2]) > .012:
            return self._chunk({**meta, 'request_stop': True, 'stop_reason': 'carrier_stabilization_lost'})
        if self._phase >= 7 and abs(bar_q) > .004:
            return self._chunk({**meta, 'request_stop': True, 'stop_reason': 'crossbar_alignment_lost'})
        if kind == 'bar':
            # The target is q=0 in the current carrier frame, including yaw.
            # Right closure can displace the carrier before left restores it.
            goal = carrier[:3] + carrier_rot @ np.asarray(c.CROSSBAR_ORIGIN_LOCAL)
        else:
            if self._phase not in self._captured:
                self._captured[self._phase] = None if goal is None else np.asarray(goal).copy()
            goal = self._captured[self._phase]
        commands = [q.copy() for q in self._last]
        # Hold the measured carrier against right-hand crossbar reaction forces.
        if 3 <= self._phase <= 6:
            commands[0] = self._track('left', left, carrier, initial, np.eye(3), .020, meta)
        idx = 0 if arm == 'left' else 1
        measured = left if idx == 0 else right
        if kind == 'tcp':
            commands[idx], reached = self._tcp(arm, measured, goal, opening, elapsed, duration, meta)
            if elapsed >= duration and self._stable(reached, .15):
                self._next(now)
        elif kind in ('bar', 'carrier', 'carrier_precise'):
            pose = bar if kind == 'bar' else carrier
            speed_limit = .10 if kind == 'carrier' else .020
            goal_rotation = carrier_rot if kind == 'bar' else np.eye(3)
            commands[idx] = self._track(arm, measured, pose, goal, goal_rotation, speed_limit, meta)
            speed = 0.
            if arm in self._previous_poses and self._previous_time is not None:
                speed = float(np.linalg.norm(pose[:3] - self._previous_poses[arm][:3]) / (now - self._previous_time))
            error = float(np.linalg.norm(pose[:3] - goal))
            tolerance = .003 if kind == 'carrier' else .001
            reached = error < tolerance and speed < .006 and _angle(quat_xyzw_to_matrix(pose[3:]) @ goal_rotation.T) < math.radians(1.)
            if kind == 'bar':
                reached = reached and abs(bar_q) < .001
            meta.update(measured_tool_speed_m_s=speed, measured_tool_error_mm=error * 1000.)
            if self._stable(reached, .35):
                self._next(now)
        elif kind == 'grip':
            previous = self._previous_grippers.get(arm, float(measured[6]))
            gripper_speed = abs(float(measured[6]) - previous) / self._dt
            if opening == 0.:
                commands[idx][6] = 0.
                width = c.CARRIER_GRIP_SIZE[1] if arm == 'left' else c.CROSSBAR_GRIP_SIZE[0]
                expected = width / (2. * abs(FINGER_OPEN))
                ready = .65 * expected <= measured[6] <= expected + .04 and gripper_speed < .10
                pose = carrier if arm == 'left' else bar
                local = c.CARRIER_GRIP_LOCAL if arm == 'left' else c.CROSSBAR_GRIP_LOCAL
                tcp, tcp_rot = ik.tcp_pose(measured[:6], self._bases[arm])
                tool_rot = quat_xyzw_to_matrix(pose[3:])
                contact = pose[:3] + tool_rot @ np.asarray(local)
                contact_offset = tcp_rot.T @ (contact - tcp)
                grasp_rot = tcp_rot.T @ tool_rot
                relative_speed = object_speed = relative_angular_speed = math.inf
                if arm in self._previous_tcp and self._previous_time is not None:
                    previous_tcp, previous_tcp_rot = self._previous_tcp[arm]
                    previous_pose = self._previous_poses[arm]
                    previous_rot = quat_xyzw_to_matrix(previous_pose[3:])
                    previous_contact = previous_pose[:3] + previous_rot @ np.asarray(local)
                    previous_offset = previous_tcp_rot.T @ (previous_contact - previous_tcp)
                    measurement_dt = now - self._previous_time
                    relative_speed = float(np.linalg.norm(contact_offset - previous_offset) / measurement_dt)
                    object_speed = float(np.linalg.norm(pose[:3] - previous_pose[:3]) / measurement_dt)
                    relative_angular_speed = _angle(grasp_rot @ (previous_tcp_rot.T @ previous_rot).T) / measurement_dt
                settled = relative_speed < .002 and object_speed < .003 and relative_angular_speed < .05
                ready = ready and settled
                meta.update(expected_grasp_open=expected, grasp_contact_band=bool(
                                .65 * expected <= measured[6] <= expected + .04),
                            pickup_relative_speed_m_s=relative_speed,
                            pickup_object_speed_m_s=object_speed,
                            pickup_relative_angular_speed_rad_s=relative_angular_speed,
                            pickup_settled=bool(settled))
            else:
                # Start from the commanded jaw preload, never the measured gap.
                if self._phase not in self._release:
                    self._release[self._phase] = (now, self._last[idx].copy())
                started, loaded = self._release[self._phase]
                commands[idx] = loaded.copy()
                commands[idx][6] = min(1., loaded[6] + .25 * (now - started))
                ready = measured[6] >= .98 and gripper_speed < .10
                meta.update(release_preserves_arm_targets=True, release_initial_commanded_open=float(loaded[6]),
                            commanded_gripper_open=float(commands[idx][6]), release_opening_rate_per_s=.25)
            meta.update(measured_gripper_open=float(measured[6]), gripper_open_speed=gripper_speed)
            if elapsed >= duration and self._stable(bool(ready), .40 if opening == 0. else .20):
                if opening == 0.:
                    self._capture_servo(arm, measured, carrier if arm == 'left' else bar)
                else:
                    self._servo.pop(arm, None)
                self._next(now)
        elif kind == 'prepare':
            # Hold every commanded arm/jaw target, including load-bearing lead.
            meta.update(release_prepare_preserves_arm_targets=True, release_prepare_preserves_jaw_target=True)
            if elapsed >= duration:
                self._next(now)
        elif elapsed >= duration:
            if np.linalg.norm(carrier[:3] - c.CARRIER_FINAL_POS) > .004 or abs(bar_q) > .002:
                meta['feedback_fault'] = 'released_assembly_drifted'
            else:
                self._next(now)
        self._previous_poses = {'left': carrier.copy(), 'right': bar.copy()}
        self._previous_tcp = {name: ik.tcp_pose(q[:6], self._bases[name])
                              for name, q in (('left', left), ('right', right))}
        self._previous_grippers = dict(left=float(left[6]), right=float(right[6]))
        self._previous_time = now
        if 'feedback_fault' in meta:
            # Keep the last valid command when any physical/IK gate rejects a step.
            meta.update(request_stop=True, stop_reason=meta['feedback_fault'])
        else:
            self._last = tuple(commands)
        return self._chunk(meta)


def make_demo_policy(**kwargs):
    return CaptiveCrossbarDockingDemo(**kwargs)
