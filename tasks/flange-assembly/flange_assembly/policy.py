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
        for side, p in self._plans.items():
            p.adjust = lambda t, pos, rot, finger, label, side=side: self._feedback(
                side, t, pos, rot, finger, label)

    def _feedback(self, side, t, pos, rot, finger, label):
        # This oracle changes robot targets only. Object poses remain governed
        # by rigid-body contacts; the Dream policy does not call this helper.
        from flange_assembly.embodiment import demo_states
        states = demo_states()
        if not states or self._task is None:
            return pos, rot, finger, label
        module = self._task.module
        key = None
        error = None
        if label in ('lower into holder', 'release lower flange',
                     'withdraw lower horizontally', 'lift clear lower'):
            key = 'lower_flange'
            if label == 'lower into holder':
                # Release/withdrawal moves this rim grip slightly north.
                error = np.asarray(module.HOLDER) + (0., -.006) - states[key].pos[:2]
        elif label in ('align upper flange', 'release upper flange',
                       'clear upper neck', 'withdraw above flange'):
            key = 'upper_flange'
            if label == 'align upper flange':
                # The neck settles east as the loaded jaw opens. Compensate
                # this repeatable release motion rather than relaxing scoring.
                error = states['lower_flange'].pos[:2] + (-.004, 0.) - states[key].pos[:2]
                a = quat_xyzw_to_matrix(states['lower_flange'].quat_xyzw)
                b = quat_xyzw_to_matrix(states[key].quat_xyzw)
                delta = np.arctan2(a[1, 0], a[0, 0]) + .022 - np.arctan2(b[1, 0], b[0, 0])
                delta = (delta + np.pi/4) % (np.pi/2) - np.pi/4
                self._upper_yaw += float(np.clip(.15 * delta, -.015, .015))
            elif label in ('clear upper neck', 'withdraw above flange'):
                # Once open, centre the jaw opening on the neck before lifting.
                # This avoids dragging the released flange with an inner pad.
                tcp, _ = ik.tcp_pose(self._observed_joints[side], self._bases[side])
                error = states[key].pos[:2] - tcp[:2]
            rot = axis_angle_matrix(np.array([0., 0., 1.]), self._upper_yaw) @ rot
        elif label.startswith('pin_'):
            key, phase = label.split(' ', 1)
            if key not in self._pin_holes:
                # Choose the matched pair nearest this pin's nominal goal.
                nominal = next(w.pos[:2] for w in self._plans[side].waypoints
                               if w.label == key + ' lower')
                lower = module._holes(states['lower_flange'])
                upper = module._holes(states['upper_flange'])
                hole = min(lower, key=lambda h: np.linalg.norm(h - nominal))
                other = min(upper, key=lambda h: np.linalg.norm(h - hole))
                self._pin_holes[key] = (hole + other) / 2
            if phase in ('settle above hole', 'lower', 'settle pin in hole'):
                error = self._pin_holes[key] - states[key].pos[:2]
            if phase not in ('carry', 'settle above hole', 'lower',
                             'settle pin in hole', 'release', 'retreat'):
                key = None
        if key:
            offset = self._offsets.setdefault(key, np.zeros(2))
            if error is not None:
                offset += np.clip(.15 * error, -.0015, .0015)
                np.clip(offset, -.08, .08, out=offset)
            pos = pos + np.r_[offset, 0.]
        return pos, rot, finger, label


def make_policy(**kwargs):
    return PanelPolicy(**kwargs)
