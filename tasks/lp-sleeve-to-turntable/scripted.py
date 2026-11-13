"""Privileged demonstration controller: read native poses, command real robot joints.

No object pose writes, constraints, attachments, or contact modifications. This is
not Dream-1. Native trials must establish whether the physical grasps work.
"""
import math
import numpy as np
from yam_arena import ik, station
from yam_arena.runtime import arm_q, tcp_world
from yam_arena.urdf import quat_xyzw_to_matrix, axis_angle_matrix


def limited(vector, maximum):
    norm = np.linalg.norm(vector)
    return vector * min(1., maximum / max(norm, 1e-12))


def withdrawal(fraction):
    # Clear the entire 27 mm rim/pad overlap before any vertical retreat.
    later = max(0., (fraction - .5) / .5)
    distance = .030 * min(1., fraction / .5) + .035 * later
    return distance, .08 * later, 1.0


class ScriptedLP:
    def __init__(self, task):
        self.task = task
        import json
        stages = json.loads((task.folder / 'trajectory.json').read_text())['recovery']
        self.duration = {s['phase']: max(1, round(s['seconds'] * 20)) for s in stages}
        self.phase = None
        self.tick = 0
        self.last = {}
        self.rim_shift = None
        self.rim_tick = 0
        self.translation_bias = np.zeros(3)
        import importlib.util
        spec = importlib.util.spec_from_file_location('_lp_playback', task.folder / 'playback.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.playback = module.Playback(self)
        self.playback_phases = {s['phase'] for s in stages[23:]}

    def choose_goal_rotation(self, hands):
        """Yaw is unscored; choose a reachable orientation for the measured grips."""
        relative_right = self.relative['right'][1]
        preferred = math.pi / 4 - math.atan2(relative_right[1, 2], relative_right[0, 2])
        for deviation in (0, 10, -10, 20, -20, 30, -30, 45, -45, 60, -60):
            yaw = preferred + math.radians(deviation)
            goal_r = axis_angle_matrix(np.array([0., 0., 1.]), yaw)
            valid = True
            for side, (offset, relative_r) in self.relative.items():
                seed = hands[side][2].copy()
                grip = np.array([*self.task.cfg['spindle_xy'], .93]) + goal_r @ offset
                outward = goal_r @ offset
                outward[2] = 0.
                outward /= np.linalg.norm(outward)
                positions = [grip + [0., 0., .015], grip]
                for fraction in (.25, .5, .75, .85):
                    distance, rise, _ = withdrawal(fraction)
                    positions.append(grip + distance * outward + [0., 0., rise])
                for position in positions:
                    seed, pe, re = ik.solve(position, ik.matrix_to_quat_xyzw(goal_r @ relative_r),
                                            seed, station.BASES[side], iters=250)
                    if pe > .001 or re > .01:
                        valid = False
                        break
                if not valid:
                    break
            if valid:
                return goal_r, yaw
        raise RuntimeError('No reachable flat LP placement for measured grips; regrasp required')

    def observed_hands(self, base):
        hands = {}
        for side in ('left', 'right'):
            robot = base.scene[side + '_robot']
            q = arm_q(robot)
            _, rotation = ik.tcp_pose(q, station.BASES[side])
            hands[side] = (np.asarray(tcp_world(robot)), rotation, q)
        return hands

    def action(self, nominal, phase, base):
        if phase in self.playback_phases:
            return self.playback.action(nominal, phase, base)
        # The free jacket can slide on its stand during the first grasp. Locate
        # its exposed LP rim before inserting the opposite fingers in the notch.
        if phase == 'approach exposed LP rim' and self.rim_shift is None:
            self.rim_shift = self.task.states(base)['record'][0] - np.asarray(self.task.cfg['poses']['record'])
            if np.linalg.norm(self.rim_shift) > .06:
                raise RuntimeError('LP shifted beyond the safe notch-grasp correction')
        early = phase in ('approach exposed LP rim', 'enter jacket thumb cutout',
            'pinch LP rim', 'lift jacket and enclosed record', 'extract LP clear of jacket',
            'set empty jacket aside', 'release jacket', 'retreat left hand',
            'approach other LP rim', 'enter other LP rim', 'grip other LP rim')
        if early and self.rim_shift is not None:
            self.rim_tick += 1
            blend = ik.smoothstep(min(1., self.rim_tick / self.duration['approach exposed LP rim']))
            position, orientation = ik.tcp_pose(nominal[7:13], station.BASES['right'])
            seed = arm_q(base.scene['right_robot'])
            q, pe, re = ik.solve(position + blend * self.rim_shift,
                ik.matrix_to_quat_xyzw(orientation), seed, station.BASES['right'], iters=150)
            if pe > .003 or re > .04:
                raise RuntimeError(f'Observed LP rim target unreachable: {phase}, {pe}, {re}')
            nominal = nominal.copy()
            nominal[7:13] = q
        adaptive_grasp = phase in ('approach other LP rim', 'enter other LP rim', 'grip other LP rim')
        transport = phase in ('carry LP above platter', 'align LP hole above spindle', 'settle over spindle')
        adjust = phase == 'square rim grips'
        release = phase in ('release LP', 'retreat from platter', 'hold endpoint')
        if not (adaptive_grasp or adjust or transport or (release and self.last)):
            return nominal, {'adaptive': False}
        hands = self.observed_hands(base)
        p, rotation, velocity = self.task.states(base)['record']
        if phase != self.phase:
            self.phase = phase
            self.tick = 0
            self.start_hands = hands
            if phase in ('square rim grips', 'carry LP above platter'):
                self.relative = {side: (rotation.T @ (hp - p), rotation.T @ hr)
                                 for side, (hp, hr, _) in hands.items()}
                self.nominal_p = p.copy()
                self.nominal_r = rotation.copy()
                if adjust:
                    self.adjust_p = p.copy()
                    self.adjust_r = rotation.copy()
                else:
                    self.goal_r, self.goal_yaw = self.choose_goal_rotation(hands)
            if transport:
                self.start_p = self.nominal_p.copy()
                self.start_r = self.nominal_r.copy()
            if phase == 'retreat from platter':
                self.retreat_start = {side: (hp.copy(), hr.copy()) for side, (hp, hr) in self.last.items()}
        self.tick += 1
        fraction = ik.smoothstep(min(1., self.tick / self.duration.get(phase, 60)))
        targets = {}
        if adaptive_grasp:
            offset = np.array([-.142, 0., 0.])
            if phase == 'approach other LP rim':
                offset += [-.04, 0., 0.]
            destination = p + rotation @ offset
            if phase == 'approach other LP rim':
                destination[2] += .04
            grasp_rotation = rotation @ quat_xyzw_to_matrix(np.array([
                0., 2**-.5, 0., 2**-.5]))
            start_p, start_r, _ = self.start_hands['left']
            targets['left'] = (start_p + fraction * (destination - start_p),
                               ik.slerp_matrix(start_r, grasp_rotation, fraction))
        elif adjust:
            for side, (offset, relative_r) in self.relative.items():
                approach = -offset.copy()
                approach[2] = 0.
                approach /= np.linalg.norm(approach)
                closing = np.array([0., 0., -1.])
                radial = np.column_stack((closing, np.cross(approach, closing), approach))
                targets[side] = (self.adjust_p + self.adjust_r @ offset,
                    self.adjust_r @ ik.slerp_matrix(relative_r, radial, fraction))
        elif transport:
            height = {'carry LP above platter': .945,
                      'align LP hole above spindle': .93,
                      'settle over spindle': .93}[phase]
            goal = np.array([*self.task.cfg['spindle_xy'], height])
            goal_r = self.goal_r
            self.nominal_p = self.start_p + fraction * (goal - self.start_p)
            self.nominal_r = ik.slerp_matrix(self.start_r, goal_r, fraction)
            if phase != 'carry LP above platter':
                self.translation_bias += limited(.15 * (self.nominal_p - p), .001)
                self.translation_bias = limited(self.translation_bias, .03)
            commanded_p = self.nominal_p + self.translation_bias
            # The contact grasp can slip in orientation. Keep the reachable hand
            # attitude and align the hole by translation; the native spindle/mat
            # must seat the released LP. A flat held LP is not a scoring condition.
            commanded_r = self.nominal_r
            for side, (offset, relative_r) in self.relative.items():
                targets[side] = (commanded_p + commanded_r @ offset, commanded_r @ relative_r)
        elif phase == 'retreat from platter':
            for side, (hp, hr) in self.retreat_start.items():
                outward = hp - p
                outward[2] = 0.
                outward /= max(np.linalg.norm(outward), 1e-6)
                distance, rise, _ = withdrawal(min(fraction, .85))
                targets[side] = (hp + distance * outward + [0., 0., rise], hr)
        else:
            targets = self.last.copy()
        command = nominal.copy()
        if phase == 'retreat from platter':
            _, _, opening = withdrawal(min(fraction, .85))
            command[6] = opening
            command[13] = opening
        errors = {}
        for side, (position, orientation) in targets.items():
            offset = 0 if side == 'left' else 7
            seed = hands[side][2]
            q, pe, re = ik.solve(position, ik.matrix_to_quat_xyzw(orientation), seed,
                                 station.BASES[side], iters=150)
            if pe > .003 or re > .04:
                raise RuntimeError(f'Native-pose LP target unreachable: {phase}, {side}, {pe} m, {re} rad')
            command[offset:offset + 6] = q
            errors[side] = {'position_m': pe, 'rotation_rad': re}
        self.last.update(targets)
        return command, {'adaptive': True, 'corrected_ik_errors': errors,
                         'record_position_m': p.tolist(),
                         'translation_bias_m': self.translation_bias.tolist(),
                         'selected_record_yaw_rad': getattr(self, 'goal_yaw', None)}
