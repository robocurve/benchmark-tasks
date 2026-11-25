"""Opposed fingertip grip on the exposed shell and spring latch."""
import json
from pathlib import Path

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix
from collision_geometry import FingerGeometry
import pinch_ik
from config import CLIP_Z, RELEASE_TRAVEL, SERVICE, TARGET, port
from feedback import retaining_face


class Pinch:
    def __init__(self, env, plan):
        self.env = env
        self.geometry = FingerGeometry()
        spec = plan if isinstance(plan,dict) else {'plan':plan}
        self.plan = json.loads(Path(__file__).with_name('pinch_plans.json').read_text())[spec['plan']]
        self.offset = (np.array(self.plan['tcp_mm']) + spec.get('offset_mm',[0,0,0])) / 1000
        self.orientation = np.array(self.plan['rotation'])
        self.half = (self.plan['half_aperture_mm'] + spec.get('half_trim_mm',0.)) / 1000
        self.opening = self.half + spec.get('open_extra_mm',self.plan.get('open_extra_mm',3.4))/1000
        self.entry_shift = self.plan.get('entry_shift_mm',0.)/1000
        self.ready = env._ready.detach().cpu().numpy().reshape(14).copy()
        self.base = env.rig.bases['right']
        self.seed = self.ready[7:13].copy()
        self.initial_pos, self.initial_rot = ik.tcp_pose(self.seed, self.base)
        self.previous = self.ready.copy()
        self.bias = np.zeros(6)
        self.phase = 'settle'
        self.begin = 0.
        self.streak = 0
        self.failure = None
        self.hold_opening = self.opening
        self.target = np.array(port(TARGET)) + [.001, 0., 0.]
        self.control_ticks = 0
        self.preload = float(spec.get('preload_n',2.8))
        self.trim_rot = np.eye(3)

    def transition(self, phase, t):
        self.phase, self.begin, self.streak = phase, t, 0

    def hold(self, condition, count):
        self.streak = self.streak + 1 if condition else 0
        return self.streak >= count

    def pose(self, name):
        p = self.env._base.arena_world.get_pose_w(name)
        return (p.torch if hasattr(p, 'torch') else p).detach().cpu().numpy()[0]

    def adjust_grip(self, fingers, travel):
        # Opposing forces from the two finger drives. The estimate is only a control signal.
        effort=5000*(-float(np.mean(fingers))-self.hold_opening)
        change=np.clip(.18*(self.preload-effort)/5000,-.000045,.000045)
        if travel>.00242:
            change=min(change, -.000025)
        self.hold_opening=float(np.clip(self.hold_opening-change,.0008,self.opening))
        return self.hold_opening

    def command(self, t, observations=None):
        if observations is None:
            joint = self.env._base.scene['right_robot'].data.joint_pos
            joint = (joint.torch if hasattr(joint, 'torch') else joint).detach().cpu().numpy()[0]
            plug = self.pose(f'plug_{SERVICE}')
            clip = self.pose(f'clip_{SERVICE}')
        else:
            joint, plug, clip = observations
        ids, fingers = self.env._ids['right_robot']
        actual_q = joint[ids]
        actual, actual_rot, _ = pinch_ik.geometry(actual_q, self.base)
        rot = quat_xyzw_to_matrix(plug[3:])
        travel = CLIP_Z - (rot.T @ (clip[:3] - plug[:3]))[2]
        phase, age = self.phase, t - self.begin
        p, r, opening = actual, actual_rot, self.opening
        self.control_ticks += 1
        if phase == 'settle':
            p, r = self.initial_pos, self.initial_rot
            if age >= 1.7:
                self.plug0 = plug[:3].copy()
                self.rot0 = rot.copy()
                self.grasp = self.plug0 + rot @ self.offset
                self.grasp_rot = rot @ self.orientation
                self.entry = self.grasp + self.entry_shift*self.grasp_rot[:,0]
                self.front = self.entry - self.grasp_rot[:, 2] * .055
                self.transition('approach', t)
        elif phase == 'approach':
            s = ik.smoothstep(age / 3.5)
            p = self.initial_pos + (self.front - self.initial_pos) * s
            r = ik.slerp_matrix(self.initial_rot, self.grasp_rot, s)
            if age > 3.6 and np.linalg.norm(actual - p) < .002:
                self.transition('enter', t)
        elif phase in ('enter', 'close', 'preload'):
            r = self.grasp_rot
            p = self.front + (self.entry - self.front) * ik.smoothstep(age / 3.5) if phase == 'enter' else self.grasp + self.entry_shift * self.grasp_rot[:,0] * np.clip((self.hold_opening-self.half)/(self.opening-self.half),0.,1.)
            if phase == 'enter':
                if age >= 3.5 and self.hold(np.linalg.norm(p - actual) < .0007, 4):
                    self.transition('close', t)
            elif phase == 'close':
                if travel < .00202:
                    self.hold_opening = max(self.half - .003, self.hold_opening - .000028)
                elif travel > .00227:
                    self.hold_opening += .000012
                opening = self.hold_opening
                if self.hold(travel >= RELEASE_TRAVEL, 5):
                    self.grasp=p.copy()
                    self.transition('preload', t)
            else:
                opening = self.adjust_grip(joint[fingers],travel)
                if age>=.8 and self.hold(travel>=RELEASE_TRAVEL,4):
                    # Preserve the motor target that produced release through the first pull.
                    self.rel=rot.T@(p-plug[:3])
                    self.rel_rot=rot.T@r
                    self.withdraw0=plug[:3].copy()
                    self.withdraw_rot=rot.copy()
                    self.pull=0.
                    self.correction=np.zeros(3)
                    self.transition('withdraw',t)
        elif phase in ('withdraw','transfer','insert'):
            opening = self.adjust_grip(joint[fingers],travel)
            if phase == 'withdraw':
                if plug[0]<.396 or travel >= RELEASE_TRAVEL-.00012:
                    self.pull = min(.034, self.pull + .00013)
                elif self.hold_opening > self.half - .003:
                    self.hold_opening -= .000015
                goal = self.withdraw0 - [self.pull, 0., 0.]
                goal_rot = self.withdraw_rot
                if self.pull >= .034 and plug[0] < .378:
                    self.transfer0 = plug[:3].copy()
                    self.transfer_rot = rot.copy()
                    self.transition('transfer', t)
            else:
                pre = self.target - [.034, 0., 0.]
                if phase == 'transfer':
                    s = ik.smoothstep(age / 4.)
                    goal = self.transfer0 + (pre - self.transfer0) * s
                    goal_rot = ik.slerp_matrix(self.transfer_rot, np.eye(3), s)
                    if age >= 4. and self.hold(np.linalg.norm(plug[:3] - pre) < .00065 and rot[0,0] > .998, 4):
                        self.insert = 0.
                        self.transition('insert', t)
                else:
                    if np.linalg.norm(plug[1:3] - self.target[1:3]) < .00085 and pre[0]+self.insert-plug[0] < .001:
                        self.insert = min(.034, self.insert + .00013)
                    goal = pre + [self.insert, 0., 0.]
                    goal_rot = np.eye(3)
                    if self.hold(plug[0] >= self.target[0]-.00035 and np.linalg.norm(plug[1:3] - self.target[1:3]) < .0009, 4):
                        self.seat, self.seat_rot = actual.copy(), actual_rot.copy()
                        self.transition('open', t)
            # Correct slow grip compliance and slip from measured plug pose.
            err = goal - plug[:3]
            self.correction = np.clip(self.correction + .035 * err, -.0035, .0035)
            p = goal + goal_rot @ self.rel + self.correction
            if phase!='withdraw':
                # Correct angular slip under the cable load. Rate-limit the wrist correction.
                from yam_arena.urdf import axis_angle_matrix
                error_r=ik._rot_error(goal_rot,rot)
                angle=np.linalg.norm(error_r)
                if angle>1e-6:
                    delta=axis_angle_matrix(error_r/angle,min(.007,.05*angle))
                    proposed=delta@self.trim_rot
                    if np.linalg.norm(ik._rot_error(proposed,np.eye(3)))<.65:
                        self.trim_rot=proposed
                # Rotate about the plug as the wrist changes orientation.
                p=goal + self.trim_rot@goal_rot@self.rel + self.correction
            r = self.trim_rot @ goal_rot @ self.rel_rot
            if np.linalg.norm(err) > .012:
                self.failure = 'grasp slipped during transport'
        elif phase == 'open':
            p, r = self.seat, self.seat_rot
            opening = self.hold_opening + (self.opening-self.hold_opening)*ik.smoothstep(age/1.8)
            if age >= 1.85 and self.hold(travel < .0003 and retaining_face(clip,TARGET)['retained'], 4):
                self.transition('retreat', t)
        elif phase in ('retreat','rest'):
            r = self.seat_rot
            delta = -.13*r[:,2] + np.array([-.035,0.,.018])
            p = self.seat + delta * (ik.smoothstep(age/4.) if phase=='retreat' else 1.)
            if phase == 'retreat' and age >= 4.25:
                self.transition('rest', t)
            if phase == 'rest':
                opening = self.opening + (abs(FINGER_OPEN) - self.opening)*ik.smoothstep(age)
        timeouts = {'approach': 10, 'enter': 12, 'close': 18, 'preload': 7, 'withdraw': 24, 'transfer': 15, 'insert': 32, 'open': 10}
        if age > timeouts.get(phase, 200):
            self.failure = 'timed out in ' + phase
        self.seed = pinch_ik.solve(p, r, self.seed, self.base)
        self.bias = np.clip(self.bias + np.clip(.06*(self.seed-actual_q), -.00022,.00022), -.024,.024)
        out = self.ready.copy()
        out[7:13] = np.clip(self.seed+self.bias, self.previous[7:13]-.04, self.previous[7:13]+.04)
        out[13] = np.clip(opening/abs(FINGER_OPEN),self.previous[13]-.05,self.previous[13]+.05)
        self.previous = np.clip(out,self.env._low,self.env._high)
        return self.previous, dict(phase=phase, failure=self.failure, complete=phase=='rest' and age>=3.5,
            clip_travel_m=float(travel), opening_command_mm=1000*opening, jaws_mm=(1000*joint[fingers]).tolist(),
            target=p.tolist(), tcp=actual.tolist(), plug=plug.tolist(), elapsed_in_phase=age,
            arms=[dict(side='left',position_error=0.),dict(side='right',position_error=float(np.linalg.norm(p-actual)))])


def make_policy(env,spec):
    if isinstance(spec,dict) and spec.get('style')=='boot_press':
        from boot_press import BootPress
        return BootPress(env,spec)
    return Pinch(env,spec)
