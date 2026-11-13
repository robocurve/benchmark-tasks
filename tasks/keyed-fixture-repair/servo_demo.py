"""Feedback demo: use body poses to drive ordinary YAM joint actions.

This commissioning policy can see simulated poses. It never writes a body
pose, changes contact, or supplies poses to Dream-1. The right arm proves the
fault, pulls the lock pin, slides the faulty guide out, installs the 427 spare
and pins it, then retests the part both ways.
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

ABOVE = task.TOP + .170
PART_ANCHOR = (0., 0., .070)
GUIDE_ANCHOR = (.049, 0., .050)
PIN_ANCHOR = (0., 0., .047)              # fingertips stay above the guide walls
FIX_X, FIX_Y = task.FIX
ENTRY_XY = (FIX_X - task.ENTRY, FIX_Y)
HOME_XY = (FIX_X + .0005, FIX_Y)        # pressed lightly against the stop
TRAY_XY = (task.REJECT[0] - .0015, task.REJECT[1])
SLIDE_Z = task.FLOOR                    # slide on the base; lugs clear the lips by 1.5 mm


def yaw_rot(yaw):
    return axis_angle_matrix(np.array([0., 0., 1.]), yaw)


def wrap(angle):
    return atan2(np.sin(angle), np.cos(angle))


def obj(duration, xy, z, yaw):
    return ("obj", duration, xy, z, yaw)


def part_move(dest_xy, dest_z, source_yaw, dest_yaw, *, prove=False):
    path = [("lift",)]
    if source_yaw != dest_yaw:
        path.append(("turn",))
    path += [obj(1.6, dest_xy, task.TOP + .097, dest_yaw),
             obj(1.9, dest_xy, task.TOP + dest_z + .0025, dest_yaw)]
    return ("test_part", PART_ANCHOR, path, prove)


def pin_move(source_xy, source_top, dest_xy, dest_top):
    # Draw the pin out along its own axis (the grasp may sit off-centre after
    # the jaws close), then carry it.
    return ("lock_pin", PIN_ANCHOR, [
        ("draw", 1.4, source_xy, task.TOP + source_top + .030, 0.),
        ("lift",),
        obj(1.6, dest_xy, task.TOP + .090, 0.),
        obj(1.4, dest_xy, task.TOP + dest_top + .012, 0.),
        obj(1.2, dest_xy, task.TOP + dest_top + .001, 0.),
    ], False)


MOVES = (
    part_move(task.FIX, task.SEAT, pi, pi, prove=True),       # fault: backward part seats
    part_move(task.CRADLE, .006, pi, pi),
    pin_move(task.PIN_HOLE, task.TAB_TOP, task.PARK, task.PARK_TOP),   # unlock
    ("original_guide", GUIDE_ANCHOR, [                         # slide out, lift, quarantine
        obj(.8, task.FIX, task.TOP + SLIDE_Z, 0.),
        obj(1.6, ENTRY_XY, task.TOP + SLIDE_Z, 0.),
        obj(1.2, ENTRY_XY, task.TOP + task.FLOOR + .030, 0.),
        obj(1.6, TRAY_XY, task.TOP + .120, 0.),
        obj(1.9, TRAY_XY, task.TOP + .006 + .0025, 0.),
    ], False),
    ("matching_spare", GUIDE_ANCHOR, [                         # drop in, slide to stop
        ("lift",),
        obj(1.6, ENTRY_XY, task.TOP + .120, 0.),
        obj(1.6, ENTRY_XY, task.TOP + task.FLOOR + .008, 0.),
        obj(1.0, ENTRY_XY, task.TOP + SLIDE_Z, 0.),
        obj(1.8, HOME_XY, task.TOP + SLIDE_Z, 0.),
        obj(1.0, HOME_XY, task.TOP + SLIDE_Z, 0.),          # re-measured push to the stop
    ], False),
    pin_move(task.PARK, task.PARK_TOP, "slot", task.TAB_TOP),          # lock
    part_move(task.FIX, task.SEAT, pi, 0., prove=True),       # aligned: seats
    part_move(task.FIX, task.SEAT + task.BLOCK, 0., pi, prove=True),  # opposed: blocked
    part_move(task.CRADLE, .006, pi, pi),
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
        self.parked = False

    def tcp(self, base):
        frame = link_poses(base.scene["right_robot"])["gripper"]
        return frame[:3, 3] + frame[:3, :3] @ TCP_OFFSET, frame[:3, :3]

    def setup_move(self, base):
        name, anchor, path, prove = self.moves.popleft()
        p = self.measure(base, name)
        object_yaw = atan2(p.rot[1, 0], p.rot[0, 0])
        yaw = wrap(object_yaw - pi / 2) if name == "test_part" else (
            wrap(object_yaw) if name != "lock_pin" else pi / 2)   # pin: fingers close along Y
        grasp = p.pos + p.rot @ np.asarray(anchor)
        # Aim the pin between the installed guide's tab slot and the base slot.
        g = self.measure(base, "matching_spare")
        tab = (g.pos + g.rot @ np.array([task.PIN_OFFSET, 0., 0.]))[:2]
        slot = tuple((tab + np.asarray(task.PIN_HOLE)) / 2)
        path = [(s[0], s[1], slot, *s[3:]) if len(s) > 2 and isinstance(s[2], str) else s for s in path]
        down = grip_rotation(yaw)
        above = np.array([grasp[0], grasp[1], ABOVE])

        def fixed(pos, rot):
            return lambda _: (pos, rot)

        def vertical_lift(env):
            tcp, rot = self.tcp(env)
            return np.r_[tcp[:2], ABOVE], rot

        def held(xy, goal_height, dest_yaw):
            def target(env):
                # Lock the object-in-gripper transform at the start of this
                # phase, after the preceding motion has settled.
                actual_pos, actual_rot = self.tcp(env)
                o = self.measure(env, name)
                rel = actual_rot.T @ (o.pos - actual_pos)
                rel_rot = actual_rot.T @ o.rot
                desired_rot = yaw_rot(dest_yaw) @ rel_rot.T
                desired_pos = np.array([*xy, goal_height]) - desired_rot @ rel
                return desired_pos, desired_rot
            return target

        # The pin is captive in its bore: hold the joints still while the jaws
        # close on it, so the servo cannot push it sideways.
        captive = "freeze" if name == "lock_pin" else False
        self.phases.extend([
            (1.15, fixed(above, down), 1.),
            (1.25, fixed(grasp, down), 1.),
            (.65, fixed(grasp, down), 0., captive),
        ])
        for step in path:
            if step[0] == "lift":
                self.phases.append((1.15, vertical_lift, 0.))
            elif step[0] == "turn":
                # The second quarter turn below then stays away from joint limits.
                self.phases.append((1.10, fixed(above, grip_rotation(0)), 0.))
            else:
                kind, duration, xy, z, dest_yaw = step
                # "draw" starts from a fresh integral so wind-up from closing
                # the jaws cannot press the pin sideways in its bore.
                self.phases.append((duration, held(xy, z, dest_yaw), 0., kind == "draw"))
        self.phases.extend([
            (.65, lambda _: (self.goal, self.goal_rot), 1.),
            (1.1, lambda _: (np.r_[self.goal[:2], ABOVE], self.goal_rot), 1.),
        ])
        if prove:
            self.phases.append((1.35, lambda _: (self.goal, self.goal_rot), 1.))

    def action(self, base):
        actual_pos, actual_rot = self.tcp(base)
        if self.phase is None or self.elapsed >= self.duration - 1e-8:
            if not self.phases and self.moves:
                self.setup_move(base)
            elif not self.phases and not self.parked:
                self.parked = True
                park = np.array([.23, -.257, ABOVE])
                self.phases.append((1.5, lambda _: (park, grip_rotation(pi / 2)), 1.))
            if not self.phases:
                self.done = True
                return np.r_[self.q, 1.]
            self.phase = self.phases.popleft()
            self.duration, target, self.finger, *mode = self.phase
            self.mode = mode[0] if mode else False
            if self.mode:
                self.integral[:] = 0.
            self.start, self.start_rot = actual_pos, actual_rot
            self.goal, self.goal_rot = target(base)
            self.elapsed = 0.
        self.elapsed += self.dt
        if self.mode == "freeze":
            return np.r_[self.q, self.finger]
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
        meta = {"request_stop": True} if self.servo.done else {}
        return ActionChunk(actions=[Action(data=pack(np.r_[READY, 1.], right), meta=meta)],
                           control_hz=1. / self._dt)

    YamScriptedPolicy.reset, YamScriptedPolicy.act = reset, act
