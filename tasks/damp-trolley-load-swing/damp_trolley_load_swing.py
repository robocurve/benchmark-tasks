"""Damp a hanging test weight by moving its passive monorail trolley.

All dimensions are metres. The open-loop demo uses the fixed reset and time.
No task actuator, runtime pose write, attachment to the robot or readout is used.
"""
import math
import sys, pathlib
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import box, cyl
from yam_joints import Mechanism, Link, Hinge, Slide
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = 'station'
SUCCESS_HOLD_S = 1.0

# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


TOP = TABLE_TOP['station']
YELLOW, BLUE, BLACK = (1., .75, .02), (.03, .22, .95), (.025, .025, .025)
GREY, RED, WHITE, ORANGE = (.48, .50, .53), (.95, .035, .025), (1., 1., 1.), (1., .30, .015)
BOB = (0., 0., .050)  # 27 mm bottom clearance at zero angle
KNOB = (0., .120, .150)

# Fixed build dimensions; centres and sizes lie on a 0.5 mm grid.
PIVOT = (0., 0., .305)
ROD_LENGTH = .220
BEAM_BOTTOM_Z = .328
BEAM_WEB_Z = .356
BEAM_TOP_Z = .384
POST_HEIGHT = .325
BRACKET_LEG_Z = .2075
BRACKET_LEG_LENGTH = .185
HANGER = (box((0, 0, .299), (.012, .014, .012), color=GREY),
          cyl((0, 0, .183), .004, ROD_LENGTH, color=GREY),
          cyl(BOB, .022, .046, color=RED))


def mass_properties(shapes, pivot):
    """Uniform collider-volume density, exactly as the unchanged kit."""
    volumes, central = [], []
    for s in shapes:
        if hasattr(s, 'size'):
            x, y, z = s.size
            volumes.append(x*y*z)
            central.append((x*x+z*z)/12)
        else:
            volumes.append(math.pi*s.radius**2*s.height)
            central.append((3*s.radius**2+s.height**2)/12)
    masses = .50*np.asarray(volumes)/sum(volumes)
    offsets = np.asarray([s.center for s in shapes])-pivot
    first = -float(masses @ offsets[:, 2])
    inertia = float(masses @ (np.asarray(central)+offsets[:, 0]**2+offsets[:, 2]**2))
    return dict(m=.50, d=first/.50, I=inertia, L=inertia/first,
                masses=masses.tolist(), C60=2*inertia/60)


PROPS = mass_properties(HANGER, PIVOT)
C60 = PROPS['C60']

ORIGIN = (.320, .120, TOP)
MARK_X = .200
POINTER = (0., -.050, .3355)
TROLLEY_MASS = 1.0
HINGE_ARMATURE = 0.0
HINGE_DAMPING = C60 + PROPS['m']*9.81*PROPS['d']/480
SLIDE_STATIC_FRICTION = 3.0
SLIDE_DYNAMIC_FRICTION = 2.5
HAND_RADIUS = .080  # around the tool axis: open fingers reach 73 mm, the 68 x 142 mm palm 78.7 mm, at any yaw
HAND_TIP = .013  # fingertips past the TCP along the approach axis
HAND_BACK = .100  # back of the palm behind the TCP
ARM_RADIUS = .045  # around the line through the arm's joint centres, base to wrist
BODY_CLEAR = .005  # no robot envelope may come this close to the rod or weight box
SAMPLE_STEP = .001
MAX_SWING_DEG = 30.0
CONTACT_TOL = .002
AT_MARK_TOL = .005
SETTLED_DEG = 1.5
SWAY_DEG = 5.0


def _monorail():
    h = PIVOT[2]
    base = [box((-.060, 0, BEAM_BOTTOM_Z), (.450, .060, .006), color=YELLOW),
            box((-.060, 0, BEAM_WEB_Z), (.450, .006, .050), color=YELLOW),
            box((-.060, 0, BEAM_TOP_Z), (.450, .060, .006), color=YELLOW),
            box((-.270, 0, POST_HEIGHT/2), (.030, .030, POST_HEIGHT), color=YELLOW),
            box((+.150, 0, POST_HEIGHT/2), (.030, .030, POST_HEIGHT), color=YELLOW),
            box((-.120, 0, .3875), (.010, .060, .001), color=WHITE),
            box((-.120, -.0305, BEAM_WEB_Z), (.010, .001, .062), color=WHITE)]
    trolley = [box((0, -.037, h+.013), (.044, .008, .034), color=BLUE),
               box((0, +.037, h+.013), (.044, .008, .034), color=BLUE),
               box((0, 0, h-.003), (.044, .082, .006), color=BLUE)]
    for xw in (-.014, .014):
        for yw in (-.019, .019):
            trolley.append(cyl((xw, yw, h+.036), .010, .010, axis='y', color=BLUE))
    # High arm goes behind the wrist; low return supports the knob below pads.
    leg_y = .055
    trolley += [box(POINTER, (.004, .020, .015), color=ORANGE),
                box((0, (leg_y+.026)/2, h-.005), (.012, leg_y-.026+.004, .012), color=BLUE),
                box((0, leg_y, BRACKET_LEG_Z), (.012, .004, BRACKET_LEG_LENGTH), color=BLUE),
                box((0, (leg_y+KNOB[1])/2, .115), (.012, KNOB[1]-leg_y+.008, .006), color=BLUE),
                cyl((0, KNOB[1], .1215), .004, .007, color=BLUE),
                cyl(KNOB, .010, .050, color=BLACK)]
    return Mechanism('monorail', ORIGIN, shapes=base,
        links=[Link('trolley', trolley, Slide('x', PIVOT, -.170, .050,
                    friction=0, damping=0, initial=0, armature=0,
                    static_friction=SLIDE_STATIC_FRICTION,
                    dynamic_friction=SLIDE_DYNAMIC_FRICTION), TROLLEY_MASS),
               Link('hanger', HANGER, Hinge('y', PIVOT, -60, 60,
                    damping=HINGE_DAMPING, initial=-12, armature=HINGE_ARMATURE), .50, parent='trolley')],
        mass=None, color=BLUE, yaw=0, friction=(.8, .6), contact_offset=.0004,
        self_collide=True, grip=len(base)+len(trolley)-1)


OBJECTS = [_monorail()]


def cylinder_box(state, shape):
    """Exact rotated-cylinder AABB at the hanger's reported body pose."""
    center = state.link_to_world('hanger', shape.center)
    axis = state.link_rotation('hanger')[:, 2]
    half = shape.radius*np.sqrt(np.maximum(0., 1.-axis*axis)) + shape.height/2*np.abs(axis)
    return center-half, center+half


def point_box_distance(point, bounds):
    low, high = bounds
    point = np.asarray(point)
    return float(np.linalg.norm(np.maximum(np.maximum(low-point, point-high), 0.)))


def box_distance(a, b):
    return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1], b[0]-a[1]), 0.)))


def post_boxes(state):
    rot = state.rotation()
    result = []
    for shape in OBJECTS[0].base_shapes[3:5]:
        center = state.to_world(shape.center)
        half = np.abs(rot) @ (np.asarray(shape.size)/2)
        result.append((center-half, center+half))
    return result


def _segment_points(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(math.ceil(np.linalg.norm(b-a)/SAMPLE_STEP))+1)
    return a+np.linspace(0., 1., n)[:, None]*(b-a)


def _points_box_distance(points, bounds):
    if len(points) == 0:
        return math.inf
    low, high = bounds
    gap = np.maximum(np.maximum(low-points, points-high), 0.)
    return float(np.min(np.linalg.norm(gap, axis=1)))


def _hand_axis(tcp, up):
    tcp, up = np.asarray(tcp, float), np.asarray(up, float)
    return _segment_points(tcp-HAND_TIP*up, tcp+HAND_BACK*up)


def robot_bodies():
    """Per arm: points on the hand envelope's axis and on the arm envelope's axis, from joint-state FK.

    Offline test hands (TEST_GRIPPERS) point straight down and have no arm links. Offline without
    test hands the robot is unknown and None is returned, as for release.
    """
    if TEST_GRIPPERS is not None:
        return [(_hand_axis(tcp, (0., 0., 1.)), np.zeros((0, 3))) for tcp, _ in TEST_GRIPPERS.values()]
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import YAM_URDF, station
    from yam_arena.urdf import forward_kinematics, parse_joints
    from yam_arena.constants import ARM_JOINTS, TCP_OFFSET, LEFT_BASE_POS, RIGHT_BASE_POS

    parent = {j.child: j.parent for j in parse_joints(YAM_URDF).values()}
    chain = ['gripper']
    while chain[-1] in parent:
        chain.append(parent[chain[-1]])
    chain.reverse()
    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = []
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)[arm_ids]
        poses = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, q)})
        base = np.asarray(bases[arm], float)
        g = poses['gripper']
        tcp = g[:3, :3] @ np.asarray(TCP_OFFSET) + g[:3, 3] + base
        up = -g[:3, 2]  # the approach axis is the gripper frame's +z
        centres = [poses[link][:3, 3]+base for link in chain]+[tcp+HAND_BACK*up]
        arm_points = np.concatenate([_segment_points(a, b) for a, b in zip(centres[:-1], centres[1:])])
        out.append((_hand_axis(tcp, up), arm_points))
    return out


def body_near_hanger(bodies, boxes):
    """True if a hand or arm envelope comes within BODY_CLEAR of any of the boxes."""
    if bodies is None:
        return False
    return any(_points_box_distance(hand, b) <= HAND_RADIUS+BODY_CLEAR or
               _points_box_distance(arm, b) <= ARM_RADIUS+BODY_CLEAR
               for hand, arm in bodies for b in boxes)


def score(objects):
    state = objects['monorail']
    weight = cylinder_box(state, HANGER[2])
    rod = cylinder_box(state, HANGER[1])
    near_hanger = body_near_hanger(robot_bodies(), (weight, rod))
    theta = abs(state.angle_deg('hanger'))
    contact = weight[0][2] <= TOP+CONTACT_TOL or any(
        box_distance(weight, post) <= CONTACT_TOL for post in post_boxes(state))
    # Evaluate every sticky flag and both timers every call, including after a violation.
    body_violation = ever('body_keepout', near_hanger)
    swing_violation = ever('excess_swing', theta > MAX_SWING_DEG)
    contact_violation = ever('weight_contact', contact)
    pointer_x = float(state.link_to_world('trolley', POINTER)[0])
    at_mark = abs(pointer_x-MARK_X) <= AT_MARK_TOL
    free_at_mark = at_mark and released(state.link_view('trolley'))
    settled_s = held_for('settled', free_at_mark and theta <= SETTLED_DEG)
    sway_s = held_for('sway', free_at_mark and theta <= SWAY_DEG)
    if body_violation or swing_violation or contact_violation:
        return 0.0
    if settled_s >= STILL_S:
        return 1.0
    if sway_s >= STILL_S:
        return .5
    return .25 if free_at_mark else 0.0


def success(objects):
    return score(objects) >= 1.0


ARM_LAG_S = .04243  # calibrated on the simulator from the known start and time; least-squares TCP shift.
CANCEL_T = 5.000  # calibrated on the simulator from the known start and time; five pinned periods.
CANCEL_S = .30
NET_DECAY_RATE = (HINGE_DAMPING-PROPS['m']*9.81*PROPS['d']/480)/(2*PROPS['I'])
OMEGA = math.sqrt(PROPS['m']*9.81*PROPS['d']/PROPS['I']-NET_DECAY_RATE**2)
SMOOTHSTEP_A = OMEGA*CANCEL_S/2
SMOOTHSTEP_G = 3*(math.sin(SMOOTHSTEP_A)-SMOOTHSTEP_A*math.cos(SMOOTHSTEP_A))/SMOOTHSTEP_A**3
CANCEL_THETA = math.radians(-12)*math.exp(-NET_DECAY_RATE*(CANCEL_T+.5))*(
    math.cos(OMEGA*(CANCEL_T+.5))+NET_DECAY_RATE/OMEGA*math.sin(OMEGA*(CANCEL_T+.5)))
CANCEL_STEP = PROPS['L']*abs(CANCEL_THETA)/SMOOTHSTEP_G  # calibrated on the simulator from the known start and time; model magnitude.
MARK_COMP = .0007  # calibrated on the simulator from the known start and time; transport undershoot.
TRANSPORT_S = 3.0
TRANSPORT_START = 7.0
RELEASE_T = 11.0
RUN_LIMIT_S = 30.0
PLAN_S = 15.8
KNOB_TCP = (ORIGIN[0], ORIGIN[1]+KNOB[1], TOP+KNOB[2])
MARK_TCP = (MARK_X-MARK_COMP, KNOB_TCP[1], KNOB_TCP[2])
PARK_TCP = (.150, .300, TOP+.200)


def _waypoint(duration, pos, finger, label):
    return Waypoint(duration, tuple(pos), finger, label, math.pi/2)


def _approach():
    hover = (KNOB_TCP[0], KNOB_TCP[1], KNOB_TCP[2]+.060)
    return [_waypoint(1., (.260, .320, hover[2]), FINGER_OPEN, 'orient clear of beam'),
            _waypoint(1., hover, FINGER_OPEN, 'hover'),
            _waypoint(1., KNOB_TCP, FINGER_OPEN, 'descend'),
            _waypoint(.6, KNOB_TCP, FINGER_CLOSED, 'pinch')]


def _transport(start):
    result = []
    for i in range(1, 61):
        s = i/60
        blend = 10*s**3-15*s**4+6*s**5
        pos = (start[0]+(MARK_TCP[0]-start[0])*blend, KNOB_TCP[1], KNOB_TCP[2])
        result.append(_waypoint(TRANSPORT_S/60, pos, FINGER_CLOSED, f'transport {i:02d}'))
    return result


def _finish(hold_to=PLAN_S):
    raised = (MARK_TCP[0], MARK_TCP[1], MARK_TCP[2]+.040)
    return [_waypoint(.5, MARK_TCP, FINGER_OPEN, 'release'),
            _waypoint(.6, raised, FINGER_OPEN, 'rise'),
            _waypoint(1., PARK_TCP, FINGER_OPEN, 'park'),
            _waypoint(hold_to-(RELEASE_T+2.1), PARK_TCP, FINGER_OPEN, 'scored hold')]


def demo():
    cancelled = (KNOB_TCP[0]-CANCEL_STEP, KNOB_TCP[1], KNOB_TCP[2])
    begin = CANCEL_T-ARM_LAG_S-CANCEL_S/2
    left = _approach()+[
        _waypoint(begin-3.6, KNOB_TCP, FINGER_CLOSED, 'hold'),
        _waypoint(CANCEL_S, cancelled, FINGER_CLOSED, 'cancel step'),
        _waypoint(TRANSPORT_START-(begin+CANCEL_S), cancelled, FINGER_CLOSED, 'hold after cancel')]
    left += _transport(cancelled)
    left += [_waypoint(RELEASE_T-TRANSPORT_START-TRANSPORT_S, MARK_TCP, FINGER_CLOSED, 'hold at mark')]
    left += _finish()
    return {'left': left, 'right': []}
