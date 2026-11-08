"""Audited joint-command rope manipulation. No rope poses or contacts are edited.

The privileged demonstration can read measured rope geometry. Dream receives
only the normal cameras and joint state. Idle/grasp modes are development probes.
"""
from dataclasses import replace
import math
import numpy as np
import telemetry
from config import CONFIG


class ArmMotion:
    def __init__(self, side):
        from yam_arena.constants import READY, FINGER_OPEN
        from yam_arena import station
        self.side, self.base = side, station.BASES[side]
        self.q = np.array(READY, float)
        self.finger = FINGER_OPEN
        self.action = np.r_[self.q, 1.]
        self.yaw = -math.pi / 2 if side == 'left' else math.pi / 2

    def move(self, position, seconds=1., *, opening=None, yaw=None, label='move'):
        from yam_arena.demo import ArmPlan, Waypoint
        opening = self.finger if opening is None else opening
        yaw = self.yaw if yaw is None else yaw
        plan = ArmPlan([Waypoint(seconds, tuple(position), opening, label, yaw)],
                       self.base, q_start=self.q, finger_start=self.finger)
        event = dict(label=label, side=self.side,
            target=list(map(float, position)), seconds=seconds, yaw=yaw,
            finger_target=opening, privileged_demo_only=True)
        telemetry.demo_events.append(event)
        for i in range(1, round(seconds * 20) + 1):
            self.action, _ = plan.action7(i / 20)
            self.q = plan.q.copy()
            yield
        self.finger, self.yaw = opening, yaw
        event.update(worst_ik_mm=plan.worst_ik_mm, worst_ik_deg=plan.worst_ik_deg)

    def grip(self, opening, seconds=.8):
        from yam_arena import ik
        pos, rotation = ik.tcp_pose(self.q, self.base)
        yield from self.move_pose(pos, rotation, seconds, opening=opening, label='grip')

    def move_pose(self, position, rotation, seconds=1., *, opening=None,
                  label='pose move'):
        """Arbitrary TCP orientation through the same normal joint action path."""
        from yam_arena import ik
        from yam_arena.demo import ArmPlan, Waypoint
        opening = self.finger if opening is None else opening
        goal_pos, goal_rot = np.asarray(position, float), np.asarray(rotation, float)
        if goal_pos.shape != (3,) or goal_rot.shape != (3, 3):
            raise ValueError('Pose needs xyz and a 3×3 rotation')
        if not np.allclose(goal_rot.T @ goal_rot, np.eye(3), atol=1e-6) or np.linalg.det(goal_rot) < .999:
            raise ValueError('Invalid target rotation')

        class PosePlan(ArmPlan):
            def target(plan, t):
                s = ik.smoothstep(t / plan.duration)
                return (plan.start_pos + s * (goal_pos - plan.start_pos),
                        ik.slerp_matrix(plan.start_rot, goal_rot, s),
                        plan.finger_start + s * (opening - plan.finger_start), label)

        plan = PosePlan([Waypoint(seconds, tuple(position), opening, label)],
                        self.base, q_start=self.q, finger_start=self.finger)
        event = dict(label=label, side=self.side, target=goal_pos.tolist(),
                     rotation=goal_rot.tolist(), seconds=seconds,
                     finger_target=opening, privileged_demo_only=True)
        telemetry.demo_events.append(event)
        for i in range(1, round(seconds * 20) + 1):
            self.action, _ = plan.action7(i / 20)
            self.q = plan.q.copy()
            if plan.worst_ik_mm > 3. or plan.worst_ik_deg > 2.:
                raise RuntimeError(f'Unreachable pose during {label}: '
                                   f'{plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.2f} degrees')
            yield
        self.finger = opening
        event.update(worst_ik_mm=plan.worst_ik_mm, worst_ik_deg=plan.worst_ik_deg)


def wait(seconds):
    for _ in range(round(seconds * 20)):
        yield


def both(*motions):
    active = list(map(iter, motions))
    while active:
        for motion in active.copy():
            try:
                next(motion)
            except StopIteration:
                active.remove(motion)
        if active:
            yield


def script(arms, mode):
    from yam_arena.constants import FINGER_OPEN
    if mode == 'idle':
        yield from wait(10. if CONFIG.get('proof_probe') else 5.)
        return
    if mode in ('demo', 'loop', 'thread'):
        from table_policy import tie_overhand
        yield from tie_overhand(arms, stage=mode)
        return
    if mode == 'jig':
        from jig_policy import jig_thread
        yield from jig_thread(arms)
        return
    right = arms[1]
    p = np.asarray(telemetry.latest_sample['cord']['points'][-4], float)
    hover = p + [0, 0, .10]
    pick = p + [0, 0, .009]
    yield from right.move(hover, 2., opening=FINGER_OPEN, yaw=0., label='approach free end')
    yield from right.move(pick, 1.5, label='descend to cord')
    yield from right.grip(-(CONFIG['radius']-.0005))
    yield from right.move(hover, 2., label='lift free end')
    yield from wait(2.)
    yield from right.move(pick, 2., label='return cord')
    yield from right.grip(FINGER_OPEN)
    yield from right.move(hover, 1.5, label='withdraw')
    yield from wait(2.)


def form_loop(arms):
    """Development sequence: form one crossing and lift it; not a knot demo."""
    from yam_arena.constants import FINGER_OPEN
    left, right = arms
    top = CONFIG['table_top']
    nodes = np.asarray(telemetry.latest_sample['cord']['points'])
    a, b = nodes[3].copy(), nodes[-12].copy()
    yield from right.move(b + [0, 0, .10], 2., opening=FINGER_OPEN, yaw=0.)
    yield from right.move(b + [0, 0, .009], 1.5)
    yield from right.grip(-.0035)
    yield from right.move(b + [0, 0, .085], 2., label='lift working end')
    yield from right.move((.28, .05, top + .09), 2.5, label='cross above standing part')
    yield from right.move((.28, .05, top + .020), 1.8, label='lay single crossing')
    yield from right.grip(FINGER_OPEN)
    yield from right.move((.16, -.20, top + .12), 2., label='clear crossing')
    yield from left.move((.18, .09, top + .12), 1.5, opening=FINGER_OPEN, yaw=-math.pi/4)
    crossing = find_crossing(telemetry.latest_sample['cord']['polyline'])
    telemetry.demo_events.append(dict(label='crossing detection', crossing=crossing,
                                      privileged_demo_only=True))
    if crossing is None:
        yield from wait(2.)
        return
    cx, cy, cz = crossing['position']
    yield from left.move((cx, cy, top + .12), 1.5, label='approach measured crossing')
    yield from left.move((cx, cy, top + .014), 1.5, label='capture crossing')
    yield from left.grip(-.0035)
    yield from left.move((.20, .015, top + .150), 3., label='lift loose loop without stretching standing end')
    yield from wait(3.)
    return crossing


def thread_tail(arms):
    """Bounded contact-only threading probe, using measured cord geometry."""
    from yam_arena.constants import FINGER_OPEN
    left, right = arms
    top = CONFIG['table_top']
    sample = telemetry.latest_sample
    points = np.asarray(sample['cord']['points'])
    tcp_left = np.asarray(sample['hands'][0]['tcp'])
    distances = np.linalg.norm(points - tcp_left, axis=1)
    i = int(np.argmin(distances[:len(points)//2]))
    j = int(len(points)//2 + np.argmin(distances[len(points)//2:]))
    if max(distances[i], distances[j]) > .025 or (len(points)-j)*.01 < .14:
        telemetry.demo_events.append(dict(label='threading aborted: crossing or free tail insufficient',
            held_indices=[i,j], distances=[float(distances[i]), float(distances[j])]))
        return
    loop = points[i+1:j]
    center = loop.mean(axis=0)
    _, singular, vh = np.linalg.svd(loop-center)
    normal = vh[-1]
    if normal[1] < 0:
        normal = -normal
    telemetry.demo_events.append(dict(label='measured loop', indices=[i,j],
        center=center.tolist(), normal=normal.tolist(), singular_values=singular.tolist()))
    if normal[1] < .7:
        telemetry.demo_events.append(dict(label='threading aborted: loop plane outside validated approach range'))
        return
    angle = math.radians(20)
    rotation = np.array([[1.,0.,0.], [0.,-math.sin(angle),math.cos(angle)],
                         [0.,-math.cos(angle),-math.sin(angle)]])
    approach = rotation[:,2]
    pick = points[-8].copy()
    # The working section hangs below the left hand. Approach it from the
    # right side with the palm behind the TCP, then pinch normally.
    yield from right.move_pose(pick-approach*.10, rotation, 3., opening=FINGER_OPEN,
                              label='approach hanging working tail')
    pick = np.asarray(telemetry.latest_sample['cord']['points'][-8])
    yield from right.move_pose(pick-approach*.006, rotation, 2., label='capture working tail')
    yield from right.grip(-.0035, 1.)
    yield from wait(.5)
    # A short dangling tail passes through; the wrist is not dragged through
    # the loop. Tip feedback only changes joint targets, never the cord.
    desired = center.copy()
    desired[2] = max(top+.035, min(center[2], top+.065))
    for feed_index, offset in enumerate(np.linspace(-.04, .025, 14)):
        sample = telemetry.latest_sample
        tcp = np.asarray(sample['hands'][1]['tcp'])
        tip = np.asarray(sample['cord']['end_tip'])
        held = np.asarray(sample['cord']['points'][-8])
        if np.linalg.norm(held-tcp) > .03:
            telemetry.demo_events.append(dict(label='threading aborted: working tail slipped',
                                              distance_m=float(np.linalg.norm(held-tcp))))
            return
        target = desired + normal*offset - (tip-tcp)
        if target[2] < top+.04 or target[2] > top+.16:
            telemetry.demo_events.append(dict(label='threading aborted: tip-feedback height outside validated range',
                                              target=target.tolist()))
            return
        yield from right.move_pose(target, rotation, 3. if feed_index == 0 else .6,
                                   label='feed measured tail through loop')
    yield from wait(1.)
    tip = np.asarray(telemetry.latest_sample['cord']['end_tip'])
    signed = float((tip-center) @ normal)
    telemetry.demo_events.append(dict(label='tail insertion measurement', signed_plane_distance_m=signed))
    if signed < .01:
        return
    yield from right.grip(FINGER_OPEN)
    tcp = np.asarray(telemetry.latest_sample['hands'][1]['tcp'])
    yield from right.move_pose(tcp-normal*.13, rotation, 3., label='withdraw on entry side')
    yield from wait(1.)
    yield from left.grip(FINGER_OPEN)
    yield from left.move((.17,.15,top+.16), 2., opening=FINGER_OPEN,
                         label='release loop and withdraw')
    yield from wait(3.)


def find_crossing(polyline):
    """Measured XY crossing of well-separated standing/working rope segments."""
    p = np.asarray(polyline, float)
    found = []
    for i in range(min(len(p)//2, len(p)-1)):
        for j in range(max(i + 12, len(p)-25), len(p)-1):
            u, v = p[i+1]-p[i], p[j+1]-p[j]
            matrix = np.column_stack((u[:2], -v[:2]))
            if abs(np.linalg.det(matrix)) < 1e-9:
                continue
            s, t = np.linalg.solve(matrix, p[j,:2]-p[i,:2])
            if 0 <= s <= 1 and 0 <= t <= 1:
                a, b = p[i]+s*u, p[j]+t*v
                if .005 <= abs(a[2]-b[2]) <= .025:
                    found.append(dict(position=((a+b)/2).tolist(),
                                      segments=[i,j], vertical_gap=float(abs(a[2]-b[2]))))
    return min(found, key=lambda c: np.linalg.norm(np.array(c['position'][:2])-[.29,0])) if found else None


def register_policy(mode='demo'):
    from inspect_robots.registry import register
    from inspect_robots.types import Action, ActionChunk
    from inspect_robots_yam.packing import pack
    from yam_arena.scripted import YamScriptedPolicy
    name = 'rope_' + mode

    class RopeDemo(YamScriptedPolicy):
        def __init__(self):
            super().__init__()
            self.info = replace(self.info, name=name)

        def reset(self, scene):
            self.arms = [ArmMotion('left'), ArmMotion('right')]
            self.sequence = script(self.arms, mode)
            self.finished = False

        def act(self, observation):
            if not self.finished:
                try:
                    next(self.sequence)
                except StopIteration:
                    self.finished = True
            action = Action(data=pack(*(arm.action for arm in self.arms)),
                            meta={'request_stop': True} if self.finished else {})
            return ActionChunk(actions=[action], control_hz=20.)

    register('policy', name)(RopeDemo)
    return name
