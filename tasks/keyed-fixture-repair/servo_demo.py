"""Feedback demo: use body poses to drive ordinary YAM joint actions.

This commissioning policy can see simulated poses. It never writes a body
pose, changes contact, or supplies poses to Dream-1.
"""

from collections import deque
from math import atan2, pi

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena import ik
from yam_arena.constants import READY, TCP_OFFSET
from yam_arena.demo import grip_rotation
from yam_arena.runtime import link_poses
from yam_arena.urdf import axis_angle_matrix

import task


def yaw_rot(yaw):
    return axis_angle_matrix(np.array([0., 0., 1.]), yaw)


MOVES = (
    ("test_part", task.FIX, task.SEAT, pi, pi, True),
    ("test_part", task.CRADLE, .006, pi, pi, False),
    ("original_guide", (task.REJECT[0] - .015, task.REJECT[1]), .006, 0, 0, False),
    ("good_spare", task.FIX, task.FLOOR, 0, 0, False),
    ("test_part", task.FIX, task.SEAT, pi, 0, True),
    ("test_part", task.FIX, task.SEAT + task.BLOCK, 0, pi, True),
    ("test_part", task.CRADLE, .006, pi, pi, False),
)


class Servo:
    def __init__(self, dt, base_pos, measure):
        self.dt, self.base_pos, self.measure = dt, base_pos, measure
        self.moves = deque(MOVES)
        self.phases = deque()
        self.phase = None
        self.q = np.array(READY)
        self.integral = np.zeros(3)
        self.done = False

    def tcp(self, base):
        frame = link_poses(base.scene["right_robot"])["gripper"]
        return frame[:3, 3] + frame[:3, :3] @ TCP_OFFSET, frame[:3, :3]

    def setup_move(self, base):
        name, xy, z, source_yaw, dest_yaw, prove = self.moves.popleft()
        p = self.measure(base, name)
        coupon = name == "test_part"
        yaw = (pi / 2 if source_yaw == pi else -pi / 2) if coupon else 0
        yaw_error = p.rot @ yaw_rot(-source_yaw)
        yaw += atan2(yaw_error[1, 0], yaw_error[0, 0])
        anchor = np.array((0., 0., .070) if coupon else (.049, 0., .050))
        grasp = p.pos + p.rot @ anchor
        down = grip_rotation(yaw)
        above = np.array([grasp[0], grasp[1], task.TOP + .170])
        hover = task.TOP + (.097 if coupon else .120)

        def fixed(pos, rot):
            return lambda _: (pos, rot)

        def vertical_lift(env):
            tcp, rot = self.tcp(env)
            return np.r_[tcp[:2], task.TOP + .170], rot

        def held(goal_height):
            def target(env):
                # Lock the part-in-gripper transform at the start of this
                # phase, after the preceding lift or carry has settled.
                actual_pos, actual_rot = self.tcp(env)
                obj = self.measure(env, name)
                rel = actual_rot.T @ (obj.pos - actual_pos)
                rel_rot = actual_rot.T @ obj.rot
                desired_rot = yaw_rot(dest_yaw) @ rel_rot.T
                desired_pos = np.array([*xy, goal_height]) - desired_rot @ rel
                return desired_pos, desired_rot
            return target

        self.phases.extend([
            (1.15, fixed(above, down), 1.),
            (1.25, fixed(grasp, down), 1.),
            (.65, fixed(grasp, down), 0.),
            (1.15, vertical_lift, 0.),
        ])
        if source_yaw != dest_yaw:
            self.phases.append((1.10, fixed(above, grip_rotation(0)), 0.))
            # The second quarter turn below then stays away from joint limits.
        self.phases.extend([
            (1.6, held(hover), 0.),
            (1.9, held(task.TOP + z + .0025), 0.),
            (.65, lambda _: (self.goal, self.goal_rot), 1.),
            (1.1, lambda _: (np.r_[self.goal[:2], task.TOP + .170], self.goal_rot), 1.),
        ])
        if prove:
            self.phases.append((1.35, lambda _: (self.goal, self.goal_rot), 1.))

    def action(self, base):
        actual_pos, actual_rot = self.tcp(base)
        if self.phase is None or self.elapsed >= self.duration - 1e-8:
            if not self.phases and self.moves:
                self.setup_move(base)
            if not self.phases:
                self.done = True
                return np.r_[self.q, 1.]
            self.phase = self.phases.popleft()
            self.duration, target, self.finger = self.phase
            self.start, self.start_rot = actual_pos, actual_rot
            self.goal, self.goal_rot = target(base)
            self.elapsed = 0.
        self.elapsed += self.dt
        fraction = min(1., self.elapsed / max(self.dt, self.duration - .22))
        alpha = ik.smoothstep(fraction)
        pos = (1. - alpha) * self.start + alpha * self.goal
        rot = ik.slerp_matrix(self.start_rot, self.goal_rot, alpha)
        # Correct steady tracking error without touching the simulator state.
        error = pos - actual_pos
        self.integral = np.clip(.98 * self.integral + .26 * error, (-.018, -.018, -.007), (.018, .018, .012))
        correction = ik.slerp_matrix(np.eye(3), rot @ actual_rot.T, .35)
        self.q, _, _ = ik.solve(pos + self.integral, ik.matrix_to_quat_xyzw(correction @ rot),
                                self.q, base_pos=self.base_pos, iters=40)
        return np.r_[self.q, self.finger]


def install(get_embodiment, measure):
    from yam_arena.scripted import YamScriptedPolicy

    def reset(self, scene):
        self.servo = Servo(self._dt, self._bases["right"], measure)

    def act(self, observation):
        right = self.servo.action(get_embodiment()._base)
        meta = {"request_stop": True} if self.servo.done and observation.state_time > 85 else {}
        return ActionChunk(actions=[Action(data=pack(np.r_[READY, 1.], right), meta=meta)],
                           control_hz=1. / self._dt)

    YamScriptedPolicy.reset, YamScriptedPolicy.act = reset, act
