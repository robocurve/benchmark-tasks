"""Scripted Cartesian poses with explicit orientations for side grips."""
from dataclasses import dataclass
import numpy as np
from yam_arena import ik
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix
from inspect_robots_yam.packing import split


@dataclass
class PoseWaypoint(Waypoint):
    rotation: tuple | None = None  # 3x3 matrix, row-major, or None for downward grip


class PosePlan(ArmPlan):
    def target(self, t):
        original_t = t
        def adjusted(pos, rot, finger, label):
            callback = getattr(self, "adjust", None)
            return callback(original_t, pos, rot, finger, label) if callback else (pos, rot, finger, label)
        prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal = np.asarray(w.pos, dtype=float)
            rot = np.asarray(w.rotation).reshape(3, 3) if getattr(w, 'rotation', None) is not None else grip_rotation(w.yaw)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return adjusted(prev_pos+s*(goal-prev_pos), ik.slerp_matrix(prev_rot,rot,s),
                                prev_finger+s*(w.finger-prev_finger), w.label)
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return adjusted(prev_pos, prev_rot, prev_finger, 'done')


class PanelPolicy(YamScriptedPolicy):
    def act(self, observation):
        left, right = split(observation.state['joint_pos'])
        self._observed_joints = {'left': left[:6], 'right': right[:6]}
        return super().act(observation)

    def reset(self, scene):
        plan = self.plan_for(scene.instruction, scene.target)
        self._plans = {arm: PosePlan(list(waypoints), self._bases[arm]) for arm, waypoints in plan.items()}
        self._end = max((p.duration for p in self._plans.values()), default=0.)
        self._offsets = {}
        self._upper_yaw = 0.
        self._pin_holes = {}
        self._align_offset = np.zeros(3)
        self._align_rot = np.zeros(3)
        self._insertion_offsets = {'second': np.zeros(3)}
        for side, p in self._plans.items():
            p.adjust = lambda t, pos, rot, finger, label, side=side: self._feedback(
                side, t, pos, rot, finger, label)

    def _feedback(self, side, t, pos, rot, finger, label):
        from flange_assembly.embodiment import demo_states
        states = demo_states()
        if side == 'right' and label == 'done':
            return pos + self._insertion_offsets['second'], rot, finger, label
        if side == 'right' and states:
            for name, pin_name, hole_y, hole_z in (
                ('second', 'pin_1', .06, .225),
            ):
                if name not in label or not any(action in label for action in
                        ('approach', 'insert', 'seat', 'release', 'back off', 'push', 'clear', 'withdraw')):
                    continue
                pin = states[pin_name]
                desired = np.array([hole_y, self._task.module.TOP + hole_z])
                error = desired - np.asarray(pin.pos)[1:]
                if ('hole' in label or 'insert' in label or 'seat' in label) and pin.pos[0] > .19 and np.linalg.norm(error) < .008:
                    self._insertion_offsets[name][1:] += np.clip(.01 * error, -.00005, .00005)
                    np.clip(self._insertion_offsets[name], -.004, .004, out=self._insertion_offsets[name])
                return pos + self._insertion_offsets[name], rot, finger, label
        if side == 'left' and label in ('release flange', 'withdraw flange arm', 'done'):
            angle = np.linalg.norm(self._align_rot)
            if angle > 1e-8:
                rot = axis_angle_matrix(self._align_rot / angle, angle) @ rot
            return pos + self._align_offset, rot, finger, label
        if side != 'left' or label != 'hold flange' or t < 13.5 or not states:
            return pos, rot, finger, label
        # The side lug extends the loose object's bounding box by 40 mm in +y.
        target = np.array([.2495, .08, self._task.module.TOP + .17])
        error = target - states['upper_flange'].pos
        obj_rotation = quat_xyzw_to_matrix(states['upper_flange'].quat_xyzw)
        desired = axis_angle_matrix(np.array([0., 1., 0.]), -np.pi/2)
        angular_error = ik._rot_error(desired, obj_rotation)
        self._align_rot += np.clip(.01 * angular_error, -.00015, .00015)
        np.clip(self._align_rot, -.15, .15, out=self._align_rot)
        angle = np.linalg.norm(self._align_rot)
        if angle > 1e-8:
            rot = axis_angle_matrix(self._align_rot / angle, angle) @ rot
        # Keep a 4.5 mm axial gap while correcting the bolt-circle offset.
        # A 14 mm pin in a 16 mm hole allows only 1 mm radial clearance.
        pin = states['pin_2']
        pin_engaged = (pin.pos[0] > .295 and abs(pin.pos[1] - .06) < .012
                       and abs(pin.pos[2] - (self._task.module.TOP + .115)) < .012)
        if not pin_engaged and (np.linalg.norm(error[1:]) > .001 or np.linalg.norm(angular_error) > .02):
            error[0] = .245 - states['upper_flange'].pos[0]
        correction = np.clip(.01 * error, -.00005, .00005)
        self._align_offset += correction
        np.clip(self._align_offset, -.04, .04, out=self._align_offset)
        return pos + self._align_offset, rot, finger, label


def make_policy(**kwargs):
    return PanelPolicy(**kwargs)
