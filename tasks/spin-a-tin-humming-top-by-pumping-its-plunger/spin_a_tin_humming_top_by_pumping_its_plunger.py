"""A Schylling Little Tin Tops (MTA) humming top, 127 mm across, lies stopped on an IKEA TILLGANG tray
(370 x 290 mm, gray), leaning on its red plastic point and its rim. Its red knob sits on a twisted rod: pushing
the knob down turns the tin body through a one-way screw, lifting it freewheels. Stand the top up by its knob,
pump the knob down and up until the top spins fast, then let go and keep both grippers clear so the top spins on
its point by itself for ten seconds. The tin body has red, blue, green, yellow and orange bands and a white dot.
"""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import pump_top_kit as PK  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena import ik  # noqa: E402

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
FLOOR = TOP + .003          # tray deck surface
SCORE_OVER = 'end'
SUCCESS_HOLD_S = .05
RUN_LIMIT_STEPS = 1200
TEST_TRACK = None
TEST_HANDS = None
DIAG = False
PRESSED_PARTS = ('top',)
CLEAR_M = .050               # both grippers at least 5 cm from the toy
MIN_SPIN = 2 * math.pi       # one turn per second
SCREW_J = .001                # J the screw must have passed into the body (one turn per second holds .0043 J)
RIM_Z, RIM_R = .063, .0635   # the tin body's rim: height along the axis above the point, radius


def context():
    c = {'env': None, 't': None, 'episode': 0, 'step': 0}
    try:
        from yam_arena import inspect_embodiment as ie
    except ImportError:
        return c
    cls = ie.YamArenaEmbodiment
    if hasattr(cls, '_pump_context'):
        return cls._pump_context
    step, reset = cls._task_step, cls.reset

    def wrapped_step(self, *a, **kw):
        c.update(env=self._env, t=self._t, step=c['step'] + 1)
        return step(self, *a, **kw)

    def wrapped_reset(self, *a, **kw):
        if getattr(self, '_env', None) is not None:
            PK.clear(self._env)
        r = reset(self, *a, **kw)
        PK.clear(self._env)
        c.update(env=self._env, t=0., episode=c['episode'] + 1)
        PK.install(self._env)
        for hook in AFTER_RESET:
            hook(self._env)
        return r
    cls._pump_context = c
    cls._task_step, cls.reset = wrapped_step, wrapped_reset
    return c


AFTER_RESET = []
C = context()
PK.spawn_settings()
TOY = PK.top((*PK.TOP_XY, FLOOR + .002))
OBJECTS = [PK.tray(TOP), TOY]


# ---------------------------------------------------------------- scoring ----
def hands():
    if TEST_HANDS is not None:
        return TEST_HANDS
    if C['env'] is None:
        return {}
    from yam_arena import station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    out = {}
    for arm in ('left', 'right'):
        a = C['env'].unwrapped.scene[arm + '_robot']
        j = a.find_joints(ARM_JOINTS, preserve_order=True)[0]
        f = a.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = PK.array(a.data.joint_pos)[0]
        p, R = ik.tcp_pose(q[j], station.BASES[arm])
        out[arm] = (p, R, float(abs(q[f].mean()) * 2), np.array(q[j], float))
    return out


_HULL = {}
_ROBOT_HULLS = None


def gripper_points(gap):
    """Gripper and fingertip hull vertices in the TCP frame at this opening (1 mm cache)."""
    key = round(gap, 3)
    if key in _HULL:
        return _HULL[key]
    from yam_arena.analysis import link_hulls
    from yam_arena import YAM_URDF
    from yam_arena.urdf import forward_kinematics
    from yam_arena.constants import TCP_OFFSET
    global _ROBOT_HULLS
    if _ROBOT_HULLS is None:
        _ROBOT_HULLS = link_hulls()
    q = {f'joint{i}': 0. for i in range(1, 9)}
    q['joint7'] = q['joint8'] = -key / 2
    poses = forward_kinematics(YAM_URDF, q)
    inv = np.linalg.inv(poses['gripper'])
    pts = []
    for name in ('gripper', 'tip_left', 'tip_right'):
        T = inv @ poses[name]
        for hull in _ROBOT_HULLS[name]:
            pts.extend(hull.vertices @ T[:3, :3].T + T[:3, 3] - np.array(TCP_OFFSET))
    _HULL[key] = np.array(pts)
    return _HULL[key]


MEM = {'episode': -1}
ROD_OUT_MM = 10.     # the rod's parts this far from where its guide puts them: pulled out or bent


def failure(s):
    """(off the table: the point more than 5 cm below the table top; mm the rod is out of its guide)."""
    return float(s.origin[2]) < TOP - .05, float(s.fk_error_mm())


def track(objects):
    """Whole-video bookkeeping, as a person with the trial video and a stopwatch would do it."""
    if TEST_TRACK is not None:      # offline checks: the stopwatch record is given, the failure rules read the pose
        m = dict(TEST_TRACK)
        if m.get('bad') is None:
            off, out = failure(objects['top'])
            m['bad'] = off or m.get('rod_out_mm', out) > ROD_OUT_MM
        return m
    if C['t'] is None:
        return {'spun': False, 'best': 0., 'bad': False}
    t = C['t']
    s = objects['top']
    if MEM['episode'] != C['episode'] or t < MEM.get('last', -1):
        MEM.clear()
        MEM.update(episode=C['episode'], last=-1, spun=False, best=0., time=0., bad=False, since=None, t20=None,
                   let_go=None)
    if t == MEM['last']:
        return MEM
    R = s.rotation()
    axis = R[:, 2]
    spin = float(s.ang_vel @ axis)
    # Spun by the plunger: at least a turn per second, with the screw having done work.
    MEM['spun'] |= abs(spin) >= MIN_SPIN and PK.CLUTCH.work_out > SCREW_J
    off, fk = failure(s)
    torn = fk > ROD_OUT_MM
    if (off or torn) and not MEM['bad'] and DIAG:
        print(f'PUMP_BAD t={t:.2f} off_table={int(off)} rod_out_mm={fk:.1f}', flush=True)
    MEM['bad'] |= off or torn
    MEM['fk'] = max(MEM.get('fk', 0.), fk)
    clear = True
    for p, Rh, gap, _ in hands().values():
        P = gripper_points(gap) @ Rh.T + p
        d = np.maximum(np.maximum(s.low - P.max(0), P.min(0) - s.high), 0.)
        if float(np.linalg.norm(d)) < CLEAR_M:
            clear = False
    tilt = math.degrees(math.acos(float(np.clip(axis[2], -1, 1))))
    # In the tray's frame: the point on the flat deck at least 10 mm in from the rim's inner face; the tin body's
    # rim off the deck (the top has not fallen onto its rim).
    tip = s.origin + R @ np.array([0, 0, PK.TIP])
    tray = objects.get('tray')
    if tray is not None:
        local = tray.rotation().T @ (tip - tray.origin)
        deck = float(tray.origin[2]) + .0015
    else:
        local = tip - np.array([.195, .050, TOP + .0015])
        deck = FLOOR
    inside = abs(float(local[0])) < .165 and abs(float(local[1])) < .125
    rim_low = float(s.origin[2] + RIM_Z * axis[2] - RIM_R * math.sqrt(max(0., 1 - axis[2] ** 2)))
    standing = rim_low - deck > .0015 and float(tip[2]) - deck < PK.TIP + .006
    free = MEM['spun'] and clear and standing and inside and abs(spin) >= MIN_SPIN
    if free:
        if MEM['since'] is None:
            MEM['since'] = t
            MEM['let_go'] = dict(t=t, spin=spin, tilt=tilt)
            MEM['t20'] = None
            print(f'PUMP_RELEASE t={t:.3f} spin={spin:.2f} tilt={tilt:.2f} xy={tip[0]:.4f},{tip[1]:.4f}', flush=True)
        MEM['time'] = t - MEM['since']
        MEM['best'] = max(MEM['best'], MEM['time'])
        if tilt > 20 and MEM['t20'] is None:
            MEM['t20'] = MEM['time']
            if DIAG:
                print(f'PUMP_LEAN20 t={t:.3f} free={MEM["time"]:.3f} spin={spin:.2f}', flush=True)
    elif MEM['since'] is not None:
        print(f'PUMP_END t={t:.3f} free={MEM["time"]:.3f} tilt={tilt:.2f} spin={spin:.2f} clear={int(clear)} '
              f'standing={int(standing)} inside={int(inside)}', flush=True)
        MEM['since'] = None
        MEM['time'] = 0.
    MEM['last'] = t
    if DIAG:
        h = hands().get('left')
        hp = h[0] if h else np.zeros(3)
        print('PUMP_DIAG', f't={t:.2f} q={s.joints["rod"] * 1000:.1f} u={-s.joint_vel["rod"]:.3f} spin={spin:.2f} '
              f'tilt={tilt:.2f} lean={axis[0]:.3f},{axis[1]:.3f} tip={tip[0] * 1000:.1f},{tip[1] * 1000:.1f},'
              f'{(tip[2] - deck) * 1000:.1f} hand={hp[0] * 1000:.1f},{hp[1] * 1000:.1f},{(hp[2] - deck) * 1000:.1f} '
              f'clear={int(clear)} free={MEM["time"]:.2f} best={MEM["best"]:.2f} F={PK.CLUTCH.last_force:.1f} '
              f'Win={PK.CLUTCH.work_in:.3f} Wout={PK.CLUTCH.work_out:.3f} fk={MEM["fk"]:.1f} bad={int(MEM["bad"])}', flush=True)
    return MEM


def score(objects):
    """Free-spin time t: the longest stretch with both grippers at least 5 cm clear while the top turns on its
    point alone (at least one turn per second, rim off the tray, point on the tray's flat deck).
    0: never spun by the plunger, off the table, or the rod torn out. 0.25: spun, t < 2 s (or never let go).
    0.5: 2 s <= t < 5 s. 0.75: 5 s <= t < 10 s. 1.0: t >= 10 s."""
    m = track(objects)
    if m['bad'] or not m['spun']:
        return 0.
    if m['best'] < 2:
        return .25
    if m['best'] < 5:
        return .5
    if m['best'] < 10:
        return .75
    return 1.


def success(objects):
    return score(objects) == 1.


# ------------------------------------------------------------------- demo ----
import top_demo as TD  # noqa: E402
from yam_arena import demo as AD  # noqa: E402


class Grip(float):
    def __new__(cls, R):
        o = super().__new__(cls, 0.)
        o.matrix = np.array(R)
        return o


_original_grip = getattr(AD.grip_rotation, '_pump_original', AD.grip_rotation)


def grip_rotation(yaw=0.):
    return yaw.matrix.copy() if hasattr(yaw, 'matrix') else _original_grip(yaw)


grip_rotation._pump_original = _original_grip
AD.grip_rotation = grip_rotation
DOWN = _original_grip(0.)


def now():
    if C['env'] is None:
        return None
    from yam_arena import scene
    return scene.object_states(C['env'], ['top'])['top']


def _rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


READ = {}
HOME = np.array([*PK.TOP_XY, FLOOR + .001])


class ReadWaypoint(Waypoint):
    """A waypoint placed relative to the top as read once per episode (key), or fixed (key None). With trim, the
    pose is corrected once by the hand's own error (wanted minus where the hand settled)."""
    def __init__(self, duration, p, finger, label, R=None, key=None, local=False, trim=None, arc=None):
        self.duration = duration
        self.p = np.array(p, float)
        self.finger = finger
        self.label = label
        self.R = np.array(DOWN if R is None else R)
        self.key = key
        self.local = local
        self.trim = trim
        self.arc = arc      # fraction of the read lean still left: the knob swung up on an arc about the point

    def frame(self):
        if self.key is None:
            return self.p, self.R
        k = (C['episode'], self.key)
        if k not in READ:
            s = now()
            if s is None:  # offline: the nominal settled pose
                if self.arc is not None:
                    return self.swing(HOME, _rx(-PK.TILT_DEG))
                if self.local:
                    r = _rx(-PK.TILT_DEG)
                    return HOME + r @ self.p, r @ self.R
                return self.p, self.R
            READ[k] = (s.origin.copy(), s.rotation().copy())
            print('PUMP_READ', self.key, 'origin', np.round(s.origin, 4), 'axis', np.round(s.rotation()[:, 2], 3),
                  flush=True)
        o, R = READ[k]
        if self.arc is not None:
            return self.swing(o, R)
        if self.local:
            p, Rw = o + R @ self.p, R @ self.R
        else:
            shift = o - HOME
            shift[2] = 0
            p, Rw = self.p + shift, self.R
        if self.trim is not None:
            kt = (C['episode'], self.trim)
            if kt not in READ:
                h = hands().get('left')
                if h is None:
                    return p, Rw
                READ[kt] = (np.clip(p - h[0], -.008, .008), Rw @ h[1].T)
                print('PUMP_TRIM', self.trim, 'mm', np.round(READ[kt][0] * 1000, 1), 'deg',
                      round(math.degrees(np.linalg.norm(ik._rot_error(Rw, h[1]))), 2), flush=True)
            dp, dR = READ[kt]
            p, Rw = p + dp, dR @ Rw
        return p, Rw

    def swing(self, o, R):
        axis = R[:, 2]
        lean = math.acos(float(np.clip(axis[2], -1, 1)))
        n = np.cross([0., 0., 1.], axis)
        n = n / np.linalg.norm(n) if np.linalg.norm(n) > 1e-9 else np.array([1., 0., 0.])
        Rf = TD.expm(-n * lean * (1 - self.arc)) @ R
        centre = o + R @ np.array([0, 0, PK.TIP])
        return centre + Rf @ np.array([0, 0, GRIP - PK.TIP]) + self.p, Rf @ self.R

    @property
    def pos(self):
        return tuple(self.frame()[0])

    @property
    def yaw(self):
        return Grip(self.frame()[1])


KNOB = .134          # knob grip band centre above the point with the rod down (band 128-140 mm)
GRIP_UP = .011       # TCP this far above the band centre: pad tips 4 mm above the cone, pads on the band only
GRIP = KNOB + GRIP_UP  # TCP height above the point when the knob is bottomed
PINCH = -.0085       # 17 mm commanded on the 28 mm knob: about 27 N per pad, friction carries the 14 N push
SOFT = PINCH         # the same firm pinch from the pick-up on
STROKES = 8          # at most; pumping stops earlier once the spin passes ENOUGH
ENOUGH = 115.        # rad/s
PUMP_STEPS = 300     # control steps handed to the pumping controller (15 s)
WATCH = (.150, .270, FLOOR + .001 + .190)
RIGHTING = (.75, .5, .25)


class PumpRun:
    """One pumping controller per episode; offline, the planned path."""
    def __init__(self, start, t0):
        self.start, self.t0 = start, t0
        self.episode = None
        self.ctl = None
        self.offline = None
        self.done = -1

    def _new(self, q_arm, start_pos, log):
        ctl = TD.Pumper(_base(), q_arm, start_pos, self.start.frame()[1], PK.K, PK.TAU_MAX / PK.I3 - PK.DRAG, PK.FSCREW + PK.FBEAR,
                        strokes=STROKES, pinch=PINCH, open_g=-.026, watch=WATCH, grip_above_point=GRIP, log=log)
        ctl.stroke_kw = dict(a_up=10., a_brake=12.)
        ctl.enough, ctl.N_nominal = ENOUGH, 6
        ctl.t_top, ctl.t_ret, ctl.t_bottom = .15, .45, .05
        return ctl

    def get(self, i):
        if C['env'] is None or C['t'] is None:
            if self.offline is None:
                self.offline = (self._new(np.array([-1.13, 1.61, 1.41, -1.37, 0., -1.13]),
                                          np.array(self.start.pos), lambda *a: None), {})
            ctl, cache = self.offline
            for k in range(len(cache), i + 1):
                cache[k] = ctl.step(k, None, planned=True)
            return cache[i]
        if self.episode != C['episode']:
            self.episode, self.ctl, self.done = C['episode'], None, -1
        k_live = int(round((C['t'] - self.t0) / TD.DT))
        if k_live < 0:
            return np.array(self.start.pos), self.start.frame()[1], self.start.finger
        if self.ctl is None:
            h = hands()['left']
            self.ctl = self._new(h[3], np.array(self.start.pos), lambda *a: print(*a, flush=True))
        while self.done < min(i, k_live):
            self.done += 1
            try:
                self.ctl.step(self.done, observe())
            except Exception as e:  # noqa: BLE001
                print('PUMP_CTL error', self.done, repr(e), flush=True)
                self.ctl.cmd[self.done] = self.ctl.cmd.get(self.done - 1, (np.array(self.start.pos),
                                                                           self.start.frame()[1], PINCH))
        return self.ctl.cmd[min(i, self.done)]


def _base():
    from yam_arena import station
    return station.BASES['left']


def observe():
    s = now()
    h = hands()['left']
    R = s.rotation()
    axis = R[:, 2]
    tip = s.origin + R @ np.array([0, 0, PK.TIP])
    return TD.Obs(s.joints['rod'], float(s.ang_vel @ axis), tip[:2], axis, h[0], h[1])


class PumpWaypoint(Waypoint):
    """One 50 ms control step of the pumping controller."""
    def __init__(self, run, i):
        self.duration = TD.DT
        self.run = run
        self.i = i
        self.label = 'pump, stand upright, let go'

    @property
    def pos(self):
        return tuple(self.run.get(self.i)[0])

    @property
    def yaw(self):
        return Grip(self.run.get(self.i)[1])

    @property
    def finger(self):
        return float(self.run.get(self.i)[2])


def demo():
    x, y = PK.TOP_XY
    z = FLOOR + .001
    L = []

    def add(t, p, g, label, rot=None, key='upright', local=False, trim=None):
        L.append(ReadWaypoint(t, p, g, label, rot, key, local, trim))
    add(1.0, (x, y + .06, z + .21), -.025, 'over the knob', key=None)
    add(1.2, (0, 0, GRIP + .035), -.025, 'line up with the tilted knob', key='pick', local=True)
    add(.9, (0, 0, GRIP), -.025, 'down onto the knob', key='pick', local=True)
    add(.3, (0, 0, GRIP), -.025, 'settle on the knob', key='pick', local=True)
    add(.4, (0, 0, GRIP), -.025, 'square up to the knob', key='pick', local=True, trim='knob')
    add(.6, (0, 0, GRIP), SOFT, 'pinch the knob', key='pick', local=True, trim='knob')
    # Swing the knob up on an arc about the point (as read once the knob is held) so the point stays on the tray.
    for i, frac in enumerate(RIGHTING):
        L.append(ReadWaypoint(.35, (0, 0, .0005 * (i + 1)), SOFT, 'stand the top up', None, 'upright', arc=frac))
    L.append(ReadWaypoint(.5, (0, 0, .002), SOFT, 'steady upright', None, 'upright', arc=0.))
    run = PumpRun(L[-1], sum(w.duration for w in L))
    for i in range(PUMP_STEPS):
        L.append(PumpWaypoint(run, i))
    L.append(ReadWaypoint(12.0, WATCH, -.026, 'watch it spin', None, None))
    return {'left': L}
