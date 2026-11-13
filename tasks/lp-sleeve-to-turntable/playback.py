"""Contact-only scripted tonearm handling and button press, using native poses."""
import math
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_JOINTS, FINGER_OPEN
from yam_arena.urdf import axis_angle_matrix

DOWN = np.array([[0., -2**-.5, 2**-.5], [-1., 0., 0.], [0., -2**-.5, -2**-.5]])
ANGLE = math.radians(30)
GRASP = np.array([[0., -math.sin(ANGLE), math.cos(ANGLE)], [-1., 0., 0.], [0., -math.cos(ANGLE), -math.sin(ANGLE)]])


def rotation(yaw, pitch):
    return axis_angle_matrix(np.array([0., 0., 1.]), yaw) @ axis_angle_matrix(np.array([0., 1., 0.]), pitch)


class Playback:
    def __init__(self, parent):
        self.parent = parent
        self.phase = None
        self.last = None
        self.press_bias = 0.
        self.seat_bias = 0.

    def action(self, nominal, phase, base):
        hands = self.parent.observed_hands(base)
        from resist_isaac.state import tensor
        robot = base.scene['right_robot']
        finger_ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        measured_opening = float(np.clip((tensor(robot.data.joint_pos)[0, finger_ids] / FINGER_OPEN).max().cpu().item(), 0., 1.))
        # A 0.4 mm closing offset corresponds to roughly 2 N with the native 5000 N/m drive.
        gentle_grip = max(0., measured_opening - .0004 / abs(FINGER_OPEN))
        state = self.parent.task.states(base)
        ap, ar, _ = state['player/tonearm']
        if phase != self.phase:
            self.phase = phase
            self.tick = 0
            self.begin = {s: (p.copy(), r.copy(), 1.) for s, (p, r, q) in hands.items()}
            if self.last:
                for s in self.begin:
                    self.begin[s] = (*self.begin[s][:2], self.last[s][2])
            if phase == 'release headshell':
                self.begin['right'] = (*self.begin['right'][:2], measured_opening)
            self.start_arm_r = ar.copy()
            self.start_q = {s: q.copy() for s, (_, _, q) in hands.items()}
            if phase == 'lift stylus clear':
                hp, hr, _ = hands['right']
                self.grasp = (ar.T @ (hp - ap), ar.T @ hr)
        self.tick += 1
        fraction = ik.smoothstep(min(1., self.tick / self.parent.duration[phase]))
        targets = dict(self.begin)
        hp, hr, opening = self.begin['right']
        headshell = ap + ar @ np.asarray(self.parent.task.cfg['headshell_local'])
        if phase == 'clear hands above player':
            command = nominal.copy()
            for side, offset in [('left', 0), ('right', 7)]:
                command[offset:offset + 6] = self.start_q[side] + fraction * (np.asarray(READY) - self.start_q[side])
                command[offset + 6] = 1.
            self.last = None
            return command, dict(adaptive=True, playback_controller=True)
        elif phase in ('approach LP label', 'press LP onto spindle', 'withdraw from LP'):
            centre, disc_r, _ = state['record']
            spindle = np.asarray(self.parent.task.cfg['spindle_xy'])
            if np.linalg.norm(centre[:2] - spindle) > .005:
                raise RuntimeError('LP is outside the safe spindle seating capture radius')
            seated_height = self.parent.task.cfg['mat_top'] + self.parent.task.cfg['record_half_thickness']
            if phase == 'press LP onto spindle':
                if fraction > .7 and centre[2] > seated_height + .0006:
                    self.seat_bias = min(.004, self.seat_bias + .00025)
                height = .890 - self.seat_bias
            else:
                height = 1.00
            # Contact only the centre label, 20 mm from the spindle. The actual
            # closed finger tip is about 14 mm ahead of and below this TCP.
            destination = np.array([spindle[0] + .006, spindle[1], height])
            desired_r = DOWN
            opening = 0.
        elif phase == 'approach headshell':
            destination = headshell + [-.06, 0., .06]
            desired_r = GRASP
            opening = .5
        elif phase in ('lower to headshell', 'grip headshell'):
            destination = headshell
            desired_r = GRASP
            opening = .5 if phase == 'lower to headshell' else gentle_grip
        elif phase in ('lift stylus clear', 'swing stylus above record', 'lower stylus onto record'):
            current_yaw = math.atan2(self.start_arm_r[1, 0], self.start_arm_r[0, 0])
            yaw = current_yaw if phase == 'lift stylus clear' else -.385
            pitch = 0. if phase == 'lower stylus onto record' else .18
            desired_arm = ik.slerp_matrix(self.start_arm_r, rotation(yaw, pitch), fraction)
            offset, relative_r = self.grasp
            pivot = ap + ar @ np.array([.0025, 0., 0.])
            desired_origin = pivot - desired_arm @ np.array([.0025, 0., 0.])
            targets['right'] = (desired_origin + desired_arm @ offset, desired_arm @ relative_r, gentle_grip)
            destination = None
        elif phase == 'release headshell':
            destination = hp
            desired_r = hr
            opening = .65
        elif phase == 'withdraw from stylus':
            destination = hp + [0., 0., .12]
            desired_r = hr
            opening = .65
        elif phase == 'approach start button':
            destination = np.array([self.parent.task.cfg['button_xy'][0] - .014, self.parent.task.cfg['button_xy'][1], 1.00])
            desired_r = DOWN
            opening = 0.
        elif phase == 'press start button':
            # Actual closed finger mesh reaches 13.46 mm below TCP at this 45-degree approach.
            travel = self.parent.task.cfg['button_rest_z'] - state['player/button'][0][2]
            if fraction > .7 and travel < .00325:
                self.press_bias = min(.015, self.press_bias + .0005)
            destination = np.array([self.parent.task.cfg['button_xy'][0] - .014, self.parent.task.cfg['button_xy'][1], .8785 - self.press_bias])
            desired_r = DOWN
            opening = 0.
        elif phase == 'release start button':
            destination = np.array([self.parent.task.cfg['button_xy'][0] - .014, self.parent.task.cfg['button_xy'][1], 1.00])
            desired_r = DOWN
            opening = 0.
        elif phase == 'listen with hands clear':
            destination = hp
            desired_r = hr
        else:
            raise ValueError(phase)
        if destination is not None:
            targets['right'] = (hp + fraction * (destination - hp), ik.slerp_matrix(hr, desired_r, fraction),
                                opening if phase == 'grip headshell' else self.begin['right'][2] + fraction * (opening - self.begin['right'][2]))
        command = nominal.copy()
        errors = {}
        for side, (p, r, g) in targets.items():
            q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), hands[side][2], station.BASES[side], iters=200)
            if pe > .003 or re > .04:
                raise RuntimeError(f'Playback IK unreachable: {phase}, {side}, {pe}, {re}')
            offset = 0 if side == 'left' else 7
            command[offset:offset + 6] = q
            command[offset + 6] = g
            errors[side] = dict(position_m=pe, rotation_rad=re)
        self.last = targets
        return command, dict(adaptive=True, playback_controller=True, corrected_ik_errors=errors)
