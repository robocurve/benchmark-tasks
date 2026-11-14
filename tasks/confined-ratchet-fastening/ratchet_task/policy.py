"""Rest, pickup, socket placement and ratcheting through normal robot actions."""
from dataclasses import replace
import numpy as np
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.constants import READY
from yam_arena import ik, station
from yam_arena.urdf import axis_angle_matrix
from inspect_robots.types import Action, ActionChunk
from ratchet_task.config import *
from ratchet_task.motion import Plan, Target, rz


class FastenPolicy(YamScriptedPolicy):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.info = replace(self.info, name='confined_ratchet_scripted')

    def reset(self, scene):
        self.phase = 'start'
        self.integral = np.zeros(14)
        self.stop = False
        self.last_report = -100.
        self.pickup_verified = False
        self.socket_fit_verified = False
        self.withdrawal_verified = False
        self.settle_waits = 0

    def begin_hand(self, phase, targets, t, f):
        hand = f['hands']['right']
        grip = float(np.clip(-np.mean(hand['fingers'])/.04695, 0, 1))
        self.plan = Plan(targets, 'right', hand['joints'], grip)
        self.phase, self.start = phase, t
        self.end = t + self.plan.duration
        self.integral[:] = 0.
        print('ROBOT_POLICY', phase, t, flush=True)

    def begin_body(self, phase, targets, t, f):
        self.body_p = f['poses']['handle'].copy()
        self.body_r = f['rotations']['handle'].copy()
        self.body_targets = targets
        self.phase, self.start = phase, t
        self.end = t + sum(x.seconds for x in targets)
        self.settle_waits = 0
        print('ROBOT_POLICY', phase, t, flush=True)

    def body_goal(self, elapsed):
        p0, r0 = self.body_p, self.body_r
        for target in self.body_targets:
            if elapsed < target.seconds:
                s = ik.smoothstep(elapsed/target.seconds)
                return p0+s*(target.pos-p0), ik.slerp_matrix(r0, target.rot, s), target.label
            elapsed -= target.seconds
            p0, r0 = target.pos, target.rot
        return p0, r0, self.body_targets[-1].label

    def pack(self, a, label, observation):
        target = np.r_[READY, 1., a]
        actual = np.asarray(observation.state['joint_pos'])
        self.integral += self._dt*(target-actual)
        self.integral[[6, 13]] = 0.
        self.integral = np.clip(self.integral, -.1, .1)
        return ActionChunk(actions=[Action(data=target+self.integral,
                            meta={'waypoint': label})], control_hz=1/self._dt)

    def servo_body(self, p, r, f):
        """Command a fixed measured grasp; contacts supply all tool motion."""
        self.transport_q, pe, re = ik.solve(p+r@self.transport_offset,
            ik.matrix_to_quat_xyzw(r@self.transport_relative.T), self.transport_q,
            base_pos=station.BASES['right'], iters=100)
        if pe > .002 or re > .02:
            raise RuntimeError(f'Placement IK: {pe}, {re}')
        return np.r_[self.transport_q, 0.]

    def servo_withdrawal(self, p, r, f):
        """Correct measured tool pose as the physical grasp unloads."""
        h, hr = f['poses']['handle'], f['rotations']['handle']
        hand = f['hands']['right']
        error = ik._rot_error(r, hr)
        n = np.linalg.norm(error)
        delta = np.eye(3) if n < 1e-9 else axis_angle_matrix(error/n, min(.03, .4*n))
        shift = .4*(p-h)
        length = np.linalg.norm(shift)
        if length > .003:shift *= .003/length
        tcp = hand['tcp']+shift+(delta-np.eye(3))@(hand['tcp']-h)
        rotation = delta@hand['rotation']
        q, pe, re = ik.solve(tcp, ik.matrix_to_quat_xyzw(rotation),
            hand['joints'], base_pos=station.BASES['right'], iters=100)
        if pe > .002 or re > .02:
            raise RuntimeError(f'Withdrawal IK: {pe}, {re}')
        return np.r_[q,0.]

    def act(self, observation):
        from ratchet_task.embodiment import FEEDBACK as f
        t = float(observation.state_time)+self._dt
        hand, h, b = f['hands']['right'], f['poses']['handle'], f['poses']['bolt']
        hw = f['hardware']
        if self.phase == 'start':
            rp, rr = ik.tcp_pose(np.array(READY), station.BASES['right'])
            self.begin_hand('rest', [Target(1., rp, rr, 1., 'both arms at rest')], t, f)
        if self.phase == 'rest' and t >= self.end:
            hr = f['rotations']['handle']
            self.initial_tool = h.copy()
            p = h+hr@np.array([-.115, 0, .0132])
            # A small approach tilt keeps the wrist away from its vertical singularity.
            a = np.deg2rad(-5.)
            tilt = np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
            r = hr@tilt@np.array([[0,-1,0],[-1,0,0],[0,0,-1.]])
            self.begin_hand('pickup', [
                Target(3., p+[0,0,.08], r, .65, 'above ratchet'),
                Target(2.5, p, r, .65, 'descend around grip'),
                Target(1.5, p, r, 0., 'close on grip pads'),
                Target(5., p+[0,0,.10], r, 0., 'lift ratchet'),
                Target(1., p+[0,0,.10], r, 0., 'verify physical lift')], t, f)
        if self.phase == 'pickup' and t >= self.end:
            if h[2]-self.initial_tool[2] < .06:
                raise RuntimeError('Physical pickup did not lift the ratchet')
            self.pickup_verified = True
            hr = f['rotations']['handle']
            self.transport_offset = hr.T@(hand['tcp']-h)
            self.transport_relative = hand['rotation'].T@hr
            self.transport_q = hand['joints'].copy()
            self.entry_height = float(b[2]+.047)
            self.begin_body('carry', [
                Target(4., np.array([.20,-.12,self.entry_height]), np.eye(3), 0., 'carry outside wall'),
                Target(3., np.array([.20,0,self.entry_height]), np.eye(3), 0., 'line up with opening'),
                Target(2., np.array([.20,0,self.entry_height]), np.eye(3), 0., 'settle level tool')], t, f)
        if self.phase in ['carry', 'enter', 'align']:
            p, r, label = self.body_goal(t-self.start)
            if t >= self.end:
                pe = np.linalg.norm(p-h)
                re = np.linalg.norm(ik._rot_error(r, f['rotations']['handle']))
                if pe > .002 or re > .02:
                    self.settle_waits += 1
                    if self.settle_waits > 8:
                        raise RuntimeError(f'Tool pose failed to settle in {self.phase}: {pe}, {re}')
                    self.end = t+1.
                elif self.phase == 'carry':
                    self.begin_body('enter', [Target(6., np.array([AXIS[0],0,self.entry_height]), np.eye(3), 0., 'slide through opening'), Target(2., np.array([AXIS[0],0,self.entry_height]), np.eye(3), 0., 'settle over bolt')], t, f)
                elif self.phase == 'enter':
                    ha, da, ba = (hw['angles'][k] for k in ['handle','drive','bolt'])
                    mismatch = (ba-da+np.pi/6) % (np.pi/3)-np.pi/6
                    self.align_angle = float(np.clip(ha+mismatch, -np.deg2rad(12), np.deg2rad(12)))
                    self.begin_body('align', [Target(2., b+[0,0,.047], rz(self.align_angle), 0., 'align socket hex'), Target(1.5, b+[0,0,.047], rz(self.align_angle), 0., 'settle socket phase')], t, f)
                else:
                    self.phase, self.start = 'seat', t
                    bottom = f['poses']['drive']-.028*f['rotations']['drive'][:,2]
                    self.seat_mouth_start = float(bottom[2]-b[2])
                    self.engaged_seconds = 0.
                    print('ROBOT_POLICY', self.phase, t, flush=True)
            if self.phase in ['carry','enter','align']:
                p, r, label = self.body_goal(t-self.start)
                return self.pack(self.servo_body(p, r, f), label, observation)
        if self.phase == 'seat':
            elapsed = t-self.start
            mouth = self.seat_mouth_start + ik.smoothstep(elapsed/4.)*(.0005-self.seat_mouth_start)
            theta = self.align_angle if elapsed < 4. else np.deg2rad(12)*np.sin(2*np.pi*(elapsed-4.)/3.)
            root = b+np.array([0,0,mouth])+.028*f['rotations']['drive'][:,2]
            self.engaged_seconds = self.engaged_seconds+self._dt if hw['socket_engaged'] else 0.
            if self.engaged_seconds >= .4:
                self.socket_fit_verified = True
                hr = f['rotations']['handle']
                self.offset = hr.T@(hand['tcp']-h)
                self.relative = hand['rotation'].T@hr
                self.phase = 'ratchet'
                self.stroke_start = t
                self.stroke_from = float(hw['angles']['handle'])
                self.stroke_to = np.deg2rad(14)
                self.stroke_duration = 1.
                self.command_q = hand['joints'].copy()
                self.missing = 0.
                print('ROBOT_POLICY', self.phase, t, flush=True)
            else:
                if elapsed > 20.:
                    raise RuntimeError('Socket placement did not establish stable hex engagement')
                return self.pack(self.servo_body(root, rz(theta), f), 'lower and rock socket onto bolt', observation)
        if self.phase == 'hold' and f.get('tightened', False):
            self.phase, self.start = 'unload', t
            self.unload_from = float(hw['angles']['handle'])
            print('ROBOT_POLICY', self.phase, t, flush=True)
        if self.phase == 'unload':
            elapsed = t-self.start
            if elapsed >= 5. and f.get('score') == 1. and hw['socket_engaged']:
                self.phase = 'withdraw_start'
            else:
                if elapsed > 12.:
                    raise RuntimeError('Seat did not qualify after the freewheel return')
                theta = self.unload_from+(np.deg2rad(14)-self.unload_from)*ik.smoothstep(elapsed/3.)
                r = rz(theta)
                root = b+np.array([0,0,.0005])+.028*f['rotations']['drive'][:,2]
                self.command_q, pe, re = ik.solve(root+r@self.offset,
                    ik.matrix_to_quat_xyzw(r@self.relative.T), self.command_q,
                    base_pos=station.BASES['right'], iters=100)
                if pe > .002 or re > .02:
                    raise RuntimeError(f'Freewheel return IK: {pe}, {re}')
                return self.pack(np.r_[self.command_q,0.], 'unload socket before extraction', observation)
        if self.phase == 'withdraw_start':
            # Reverse the insertion path. Clear the hex before neutralizing yaw,
            # then settle each physical pose before moving through the opening.
            hr = f['rotations']['handle']
            self.integral[:] = 0.
            # Eight millimetres clears the 5.3 mm head. Slow extraction
            # limits the disturbance as the physical grasp unloads.
            self.withdraw_height = float(h[2]+.008)
            self.begin_body('withdraw_lift', [
                Target(2., h.copy(), hr.copy(), 0., 'settle unloaded grasp'),
                Target(20.,
                np.array([AXIS[0], 0., self.withdraw_height]),
                f['rotations']['handle'].copy(), 0., 'lift socket clear of head')], t, f)
        if self.phase in ['withdraw_lift','withdraw_level','withdraw_exit','withdraw_expose']:
            p, r, label = self.body_goal(t-self.start)
            if t >= self.end:
                pe = np.linalg.norm(p-h)
                re = np.linalg.norm(ik._rot_error(r, f['rotations']['handle']))
                bottom = f['poses']['drive']-.028*f['rotations']['drive'][:,2]
                clear = bool(not hw['socket_engaged'] and bottom[2] > b[2]+HEAD_HEIGHT+.001)
                settled = clear if self.phase == 'withdraw_lift' else pe <= .003 and re <= .03
                if not settled:
                    self.settle_waits += 1
                    if self.settle_waits > 12:
                        raise RuntimeError(f'Withdrawal failed to settle in {self.phase}: {pe}, {re}')
                    self.end = t+1.
                elif self.phase == 'withdraw_lift':
                    self.withdraw_height = float(h[2])
                    self.begin_body('withdraw_level', [Target(3., h.copy(), np.eye(3), 0., 'level tool for exit')], t, f)
                elif self.phase == 'withdraw_level':
                    self.begin_body('withdraw_exit', [Target(6.,
                        np.array([.20,0,self.withdraw_height]),np.eye(3),0.,'withdraw through opening')], t, f)
                elif self.phase == 'withdraw_exit':
                    self.begin_body('withdraw_expose', [Target(3.,
                        np.array([.20,-.12,self.withdraw_height]),np.eye(3),0.,'expose seated interface'),
                        Target(2.,np.array([.20,-.12,self.withdraw_height]),np.eye(3),0.,'inspect exposed seat')], t, f)
                else:
                    self.withdrawal_verified = bool(not hw['socket_engaged'] and
                        np.linalg.norm(h[:2]-b[:2]) > .10 and f.get('score') == 1.)
                    if not self.withdrawal_verified:
                        raise RuntimeError('Withdrawal did not preserve and expose the seated bolt')
                    self.stop = True
            p, r, label = self.body_goal(t-self.start)
            return self.pack(self.servo_withdrawal(p, r, f), label, observation)
        if self.phase in ['ratchet','hold']:
            from ratchet_task.scoring import State, seat_confirmed
            seated = seat_confirmed(State(gap=hw.get('seat_gap_m',1.),
                seat_contact=hw.get('seat_contact',False),seat_contact_fraction=hw.get('seat_contact_fraction',0.),
                seat_force_n=hw.get('seat_force_mean_n',0.),seat_gap_min_m=hw.get('seat_gap_min_m',1.),
                seat_gap_max_m=hw.get('seat_gap_max_m',1.)))
            if seated and self.phase == 'ratchet':
                self.phase, self.hold_angle = 'hold', float(hw['angles']['handle'])
            if self.phase == 'hold' and not seated:
                self.phase='ratchet';self.stroke_start=t
                self.stroke_from=float(hw['angles']['handle']);self.stroke_to=-np.deg2rad(14);self.stroke_duration=2.
            if self.phase == 'hold':
                theta = self.hold_angle
            else:
                elapsed = t-self.stroke_start
                if elapsed >= self.stroke_duration:
                    self.stroke_from, self.stroke_to = self.stroke_to, -self.stroke_to
                    self.stroke_start, elapsed = t, 0.
                    self.stroke_duration = 2. if self.stroke_to < 0 and hw['gap'] < .0005 else 1.
                theta = self.stroke_from+(self.stroke_to-self.stroke_from)*ik.smoothstep(elapsed/self.stroke_duration)
            root = b+np.array([0,0,.0005])+.028*f['rotations']['drive'][:,2]
            r = rz(theta)
            self.command_q, pe, re = ik.solve(root+r@self.offset, ik.matrix_to_quat_xyzw(r@self.relative.T), self.command_q, base_pos=station.BASES['right'], iters=100)
            if pe > .002 or re > .02:
                raise RuntimeError(f'Ratchet IK: {pe}, {re}')
            self.missing = self.missing+self._dt if not hw['socket_engaged'] else 0.
            if self.missing > 3.:
                raise RuntimeError('Socket lost engagement')
            if t-self.last_report >= 10.:
                print('RATCHET_PROGRESS', t, hw['turns'], hw['gap'], hw.get('socket_torque_nm',0.), hw['ratchet_cycles'], flush=True)
                self.last_report = t
            return self.pack(np.r_[self.command_q,0.], self.phase, observation)
        a, label = self.plan.action7(t-self.start)
        return self.pack(a, label, observation)


def make_fasten_policy(**kwargs):
    return FastenPolicy(**kwargs)
