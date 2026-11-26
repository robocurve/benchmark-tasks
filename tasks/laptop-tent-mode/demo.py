"""Bimanual YAM script using fixed reset geometry and public joint position."""

import math

import numpy as np
from inspect_robots import Action, ActionChunk

from yam_arena import ik
from geometry import (ARM_BASES, CHASSIS_Z, CONTROL_HZ, DISPLAY_Z, FRONT_X, RESET_ANGLE, RESET_HINGE,
                      WIDTH, contact_height, pitch)


BASES = ARM_BASES
OPEN = 0.32
HOVER = 0.040
CHASSIS_PITCH = -50.0
DISPLAY_PITCH = 231.25
DISPLAY_PLACEMENT_APPROACH = 33.0
PRE_FOLD = 155.0
IK_SEEDS = np.deg2rad(((-26, 145, 60, 85, 84, -90), (3, 122, 67, 56, -87, 22)))


def target(p, r, opening, joints=None):
    return np.asarray(p), r, opening, joints


def grip(hinge, angle, index, *, opening=None, back=0., lift=0., roll=0., yaw=0.):
    """Pinch each shell across its thickness, approaching a port-free side edge."""
    chassis = index == 0
    bounds = CHASSIS_Z if chassis else DISPLAY_Z
    rim = WIDTH / 2 - (0.002 if chassis else 0.007)
    theta = math.radians(-20 if chassis else yaw)
    along_edge = np.array((math.sin(theta), -math.cos(theta), 0.))
    if not chassis:
        along_edge *= -1
    closer = np.array((0., 0., 1.))
    rot = pitch(angle)
    hand = rot @ np.column_stack((closer, np.cross(along_edge, closer), along_edge))
    c, s = math.cos(roll), math.sin(roll)
    hand = hand @ np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.)))
    point = np.array((-0.080 if chassis else -0.158, rim if chassis else -rim, sum(bounds) / 2))
    tcp = hinge + rot @ point - back * hand[:, 2] + [0., 0., lift]
    if opening is None:
        opening = 0.0
    return target(tcp, hand, opening)


def push_display_rim(hinge, angle, lift=0.):
    """Top-side fingertip contact for the flat control."""
    down = np.array((0.64, 0., -0.7683749))
    across = np.array((0., -1., 0.))
    orientation = np.column_stack((across, np.cross(down, across), down))
    bezel = hinge + pitch(angle) @ [FRONT_X + 0.004, -0.107, DISPLAY_Z[0]]
    return target(bezel - 0.013 * down + [0., 0., lift], orientation, 0.)


def blend(start, end, u):
    p, r, g, _ = start
    pe, re, ge, _ = end
    return target(p + u * (pe - p), ik.slerp_matrix(r, re, u), g + u * (ge - g))


def plan(start, start_joints, control):
    """Plan the case and panel motion in desk coordinates and turn it into two TCP paths."""
    current = start

    def segment(label, steps, endpoint=None, path=None):
        nonlocal current
        initial = current
        for frame in range(1, steps + 1):
            u = ik.smoothstep(frame / steps)
            current = path(u) if path else tuple(blend(a, b, u) for a, b in zip(initial, endpoint, strict=True))
            yield label, current

    def preposition(label, steps, arm, goal):
        pos, rot, opening, _ = goal
        q0 = start_joints[arm]
        q1, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), IK_SEEDS[arm], base_pos=BASES[arm])
        if pe > 0.002 or re > 0.025:
            raise RuntimeError(f'Unreachable pregrasp for arm {arm}: {pe:.4f} m, {re:.4f} rad')
        initial = current

        def path(u):
            q = q0 + (q1 - q0) * u
            g = initial[arm][2] + (opening - initial[arm][2]) * u
            pair = list(initial)
            pair[arm] = target(*ik.tcp_pose(q, BASES[arm]), g, q)
            return tuple(pair)

        yield from segment(label, steps, path=path)

    hinge = np.array(RESET_HINGE)
    left = current[0]
    yield from preposition('approach the display rim', 36, 1, grip(hinge, RESET_ANGLE, 1, back=HOVER, lift=0.010, opening=OPEN))
    for label, steps, back, lift, opening in (
        ('align to the display rim', 12, HOVER, 0., OPEN),
        ('straddle the display rim', 22, 0., 0., OPEN),
        ('grip the display rim', 16, 0., 0., None),
    ):
        right = grip(hinge, RESET_ANGLE, 1, back=back, lift=lift, opening=opening)
        yield from segment(label, steps, endpoint=(left, right))

    right = current[1]
    yield from preposition('approach the chassis side', 35, 0, grip(hinge, 0., 0, back=HOVER, lift=0.018, opening=OPEN))
    for label, steps, back, lift, opening in (
        ('align to the chassis side', 12, HOVER, 0., OPEN),
        ('straddle the chassis rim', 24, 0., 0., OPEN),
        ('grip the chassis rim', 17, 0., 0., None),
    ):
        left = grip(hinge, 0., 0, back=back, lift=lift, opening=opening)
        yield from segment(label, steps, endpoint=(left, right))

    def pair(h, a, b, opening=None, back=0., lift=0., roll=0., yaw=0.):
        return (grip(h, a, 0, opening=opening, back=back, lift=lift),
                grip(h, b, 1, opening=opening, back=back, lift=lift, roll=roll, yaw=yaw))

    if control == 'flat':
        partial_open = 158.
        yield from segment('open the braced display', 66, path=lambda u: pair(
            hinge, 0., RESET_ANGLE + u * (partial_open - RESET_ANGLE),
        ))
        left = current[0]
        yield from segment('release the display side grip', 15, endpoint=(
            left, grip(hinge, partial_open, 1, opening=OPEN),
        ))
        yield from segment('clear the display side', 25, endpoint=(
            left, grip(hinge, partial_open, 1, opening=OPEN, back=HOVER, lift=0.04),
        ))
        yield from segment('approach above the display rim', 40, endpoint=(
            left, push_display_rim(hinge, partial_open, 0.06),
        ))
        flat_angle = 182.3
        yield from segment('press the display rim flat', 65, path=lambda u: (
            left, push_display_rim(hinge, partial_open + u * (flat_angle - partial_open), 0.06 * (1. - u) ** 4),
        ))
        yield from segment('settle flat against the desk', 18, endpoint=current)
        yield from segment('lift the right gripper clear', 24, endpoint=(left, push_display_rim(hinge, flat_angle, 0.065)))
        right = current[1]
        yield from segment('release the chassis rim', 14, endpoint=(grip(hinge, 0., 0, opening=OPEN), right))
        yield from segment('withdraw the left hand', 27, endpoint=(
            grip(hinge, 0., 0, opening=OPEN, back=HOVER, lift=0.035), right,
        ))
        yield from segment('observe the flat laptop', 50, endpoint=current)
        return

    yield from segment('open toward flat with both panels supported', 60, path=lambda u: pair(
        hinge, 0., RESET_ANGLE + u * (PRE_FOLD - RESET_ANGLE),
    ))
    yield from segment('settle the display before regrasp', 20, endpoint=current)
    left = current[0]
    for label, steps, back, opening in (
        ('release display while bracing the chassis', 14, 0., OPEN),
        ('clear the display rim to regrasp', 22, HOVER, OPEN),
    ):
        yield from segment(label, steps, endpoint=(left, grip(hinge, PRE_FOLD, 1, opening=opening, back=back)))
    # The wrist can roll 240 degrees in total. The screen is held by hinge
    # friction while the chassis remains braced on the desk during this change.
    yield from segment('roll the open right gripper', 32, path=lambda u: (
        left, grip(hinge, PRE_FOLD, 1, opening=OPEN, back=HOVER, roll=-math.pi * u),
    ))
    yield from segment('straddle the display after regrasp', 23, endpoint=(left, grip(hinge, PRE_FOLD, 1, opening=OPEN, roll=-math.pi)))
    yield from segment('close the new display grip', 16, endpoint=(left, grip(hinge, PRE_FOLD, 1, roll=-math.pi)))

    raised = hinge + [0., 0., 0.125]
    yield from segment('lift the two supported panels', 40, endpoint=pair(raised, 0., PRE_FOLD, roll=-math.pi))
    inside = raised + [0., -0.046, 0.]
    yield from segment('move the laptop inboard', 25, endpoint=pair(inside, 0., PRE_FOLD, roll=-math.pi))
    yield from segment('pass the hinge through flat', 70, path=lambda u: pair(
        inside, 0., PRE_FOLD + u * (203. - PRE_FOLD), roll=-math.pi,
    ))

    def fold(u):
        a = u * CHASSIS_PITCH
        b = 203. + u * (DISPLAY_PITCH - 203.)
        h = inside.copy()
        h[2] = max(inside[2], contact_height(a, b) + 0.032)
        return pair(h, a, b, roll=-math.pi, yaw=DISPLAY_PLACEMENT_APPROACH * u)

    yield from segment('form the inverted V', 63, path=fold)
    placed = inside.copy()
    placed[2] = contact_height(CHASSIS_PITCH, DISPLAY_PITCH) - 0.0005
    yield from segment('lower the chassis edge', 39, endpoint=pair(
        placed, CHASSIS_PITCH, DISPLAY_PITCH, roll=-math.pi, yaw=DISPLAY_PLACEMENT_APPROACH,
    ))
    # Place the display with a small downward lead to take up compliance
    # in the grip. The chassis edge remains on the desk during the transfer.
    screen_seat = placed + [0., 0., 0.003 if control == 'held' else -0.013]

    def seat(opening=None, back=0., lift=0.):
        return (grip(placed, CHASSIS_PITCH, 0, opening=opening, back=back, lift=lift),
                grip(screen_seat, DISPLAY_PITCH, 1, opening=opening, back=back, lift=lift,
                     roll=-math.pi, yaw=DISPLAY_PLACEMENT_APPROACH))

    yield from segment('lower the display edge', 25, endpoint=seat())
    if control == 'held':
        yield from segment('keep the tent supported with the grippers', 60, endpoint=seat())
        return

    yield from segment('transfer the weight to the desk', 16, endpoint=seat())
    yield from segment('release both rims', 14, endpoint=seat(opening=OPEN))
    yield from segment('withdraw both hands', 29, endpoint=seat(opening=OPEN, back=HOVER, lift=0.045))
    yield from segment('observe the unsupported tent', 48, endpoint=current)


class LaptopController:
    """Track the declared TCP path using IK and feedback from measured arm joints."""

    def __init__(self, control):
        self.control = control
        self.path = None
        self.trim = np.zeros((2, 6))

    def act(self, observation):
        measured = np.asarray(observation.state['joint_pos']).reshape(2, 7)
        if self.path is None:
            self.q = measured[:, :6].copy()
            start = tuple(target(*ik.tcp_pose(q[:6], base), q[6]) for q, base in zip(measured, BASES, strict=True))
            self.path = plan(start, self.q.copy(), self.control)
        waypoint = next(self.path, None)
        if waypoint is None:
            meta = {'request_stop': True}
        else:
            label, goals = waypoint
            command = []
            for i, (pos, rot, opening, joints) in enumerate(goals):
                if joints is None:
                    joints, _, _ = ik.solve(
                        pos, ik.matrix_to_quat_xyzw(rot), self.q[i], base_pos=BASES[i], iters=28, damping=0.022, tol=2e-6,
                    )
                self.q[i] = joints
                self.trim[i] = np.clip(self.trim[i] + 0.22 * (self.q[i] - measured[i, :6]), -0.065, 0.065)
                command.extend(np.r_[np.clip(self.q[i] + self.trim[i], ik.LOWER, ik.UPPER), opening])
            self.last = np.array(command)
            meta = {'stage': label}
        return ActionChunk(actions=[Action(data=self.last, meta=meta)], control_hz=CONTROL_HZ)
