"""Get a bare-tang hand file ready for use: push its handle home before leaving the file in the tool tray.

Scene: a full-size 200 mm flat hand file lies on the bench with its pointed tang bare; its push-on handle lies
just beyond the tang's point, mouth toward it, and a steel bench block stands a little beyond the handle's butt;
an open tool tray stands in front of the right arm. The request only says to get the file ready and leave it in
the tray. Letting go of the bare-tang file in the tray scores 0 for the rest of the episode, even if the handle
is fitted afterwards.

Scoring (end of the run, judged in a real trial from the top-camera video, an end photo and a ruler):
- 0 if at any moment the file lay in the tray, no gripper within 2.5 cm of it, while its handle was not on
  the tang (gap g over 30 mm, or off the tang's line), or if the file or handle fell off the bench.
- otherwise the lower of two numbers.
  Handle: g is the distance from the handle's front face to the line marked on the file where that face sits
  when the handle is fully home, in whole millimetres. 1.0 if g <= 2 mm (seated); 0.25 if 3 <= g <= 30 mm
  (part-way on); 0 otherwise (not on the tang).
  Place: 1.0 if the file and handle both lie inside the tray's walls, both grippers are at least 3 cm clear of
  them and they stay still for the final 1 s; 0.75 if they lie inside the tray but a gripper is closer than
  3 cm or they are still moving; 0.5 if they are anywhere else.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box
from filehaft_kit import (make_file, make_handle, TANG, BODY_GRIP, CARRY_GRIP, H_LEN, RECESS, AXIS_H)
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
SUCCESS_HOLD_S, SCORE_OVER = 1.0, 'end'
MM = .001
DIAG = False         # print a diagnostic line every 10 control steps (read-only; never part of the score)
UNSAFE_DEMO = False  # wrong-way copy: carry the bare file straight to the tray
TEST_GRIPPERS = None
TEST_UNSAFE = False

# Layout (m, from the midpoint between the arm bases; x forward, y to the robot's left). Everything lies on one
# line at x = 0.13, in front of the right arm's base, where a top-down pinch pushing along +y meets the arm's
# stiffest sideways direction (about 320 N/m by the joint stiffness, estimate).
X0 = .130
STOP_Y = -.1185                                         # steel bench block's face toward the handle
HANDLE_POS = np.array([X0, STOP_Y - .013 - H_LEN, TOP + AXIS_H])  # front face; butt 13 mm short of the block
FILE_POS = np.array([X0, HANDLE_POS[1] - .025 - TANG, TOP + .0025])  # shoulder; tang point 25 mm short of the handle
TRAY_POS = np.array([.330, -.270, TOP])
FILE = make_file(tuple(FILE_POS))
HANDLE = make_handle(tuple(HANDLE_POS))
# Steel bench block, 4 x 4 x 3/4 in (101.6 x 101.6 x 19.05 mm), hardened steel, 3 lb 2 oz (retailer listing).
# On the real bench it sits on rubber non-slip matting; here it is fixed in place (no mass).
BLOCK = Part('bench_block', (X0, STOP_Y + .0508, TOP), [box((0, 0, .009525), (.1016, .1016, .01905))],
             color=(.30, .32, .35), friction=(.4, .35), contact_offset=.0005)
# Tool tray: Really Useful Products 4 litre box, open, long side along y. 395 x 255 x 88 mm outside,
# 348 x 220 x 68 mm inside (maker); floor 20 mm, walls drawn solid to the outer size; 0.45 kg (estimate).
TRAY_IN = np.array([.110, .174, .020, .088])           # inner half-length x, half-length y, floor, rim
TRAY = Part('tray', tuple(TRAY_POS), [
    box((0, 0, .010), (.255, .395, .020)),
    box((-.11875, 0, .054), (.0175, .395, .068)),
    box((.11875, 0, .054), (.0175, .395, .068)),
    box((0, -.18575, .054), (.220, .0235, .068)),
    box((0, .18575, .054), (.220, .0235, .068))], mass=.45, color=(.60, .70, .78), friction=(.6, .5),
    contact_offset=.001)
OBJECTS = [FILE, HANDLE, BLOCK, TRAY]
PRESSED_PARTS = ('file', 'handle')


# ---------------------------------------------------------------- run context (read-only)
def _run_context():
    context = {'env': None, 't': None, 'step': 0, 'episode': 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, '_rc_run'):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context['env'], context['t'] = self._env, self._t
        context['step'] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context['episode'] += 1
        context['t'] = 0.0
        context['env'] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def _np(x):
    x = getattr(x, 'torch', x)
    if hasattr(x, 'detach'):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def grippers():
    """{arm: (TCP position, opening 0..1)} from the robot's joint readings, or None offline."""
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN['env']
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    out = {}
    for arm in ('left', 'right'):
        art = env.unwrapped.scene[arm + '_robot']
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = _np(art.data.joint_pos)[0]
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        out[arm] = (tcp, float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0)))
    return out


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN['episode']:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN['episode']


def held_for(key, cond):
    """Seconds a condition has held over consecutive control steps; offline, inf if it holds."""
    _episode_memory()
    t, step = _RUN['t'], _RUN['step']
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
    """Remember a condition once met, until the next episode; offline each call stands alone."""
    if _RUN['t'] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ---------------------------------------------------------------- what a person checks
def _clear(st, hands, margin):
    """True when every gripper's TCP is at least `margin` from the part's bounding box."""
    if hands is None:
        return True
    lo, hi = np.array(st.low), np.array(st.high)
    return all(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0)) >= margin for p, _ in hands.values())


def _inside_tray(st, tray, tol=.002):
    """The part's box lies within the tray's inner walls (in the tray's frame) and below its rim."""
    r, o = tray.rotation(), np.array(tray.origin)
    lo, hi = np.array(st.low), np.array(st.high)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    loc = (corners - o) @ r
    return bool(np.all(np.abs(loc[:, 0]) <= TRAY_IN[0] + tol) and np.all(np.abs(loc[:, 1]) <= TRAY_IN[1] + tol)
                and loc[:, 2].min() >= TRAY_IN[2] - .004 and loc[:, 2].min() <= TRAY_IN[3])


def _in_tray_footprint(st, tray):
    """The part is within the tray's inner walls and has some of it below the rim (lying in the tray)."""
    r, o = tray.rotation(), np.array(tray.origin)
    lo, hi = np.array(st.low), np.array(st.high)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    loc = (corners - o) @ r
    return bool(np.all(np.abs(loc[:, 0]) <= TRAY_IN[0] + .002) and np.all(np.abs(loc[:, 1]) <= TRAY_IN[1] + .002)
                and loc[:, 2].min() <= TRAY_IN[3])


def handle_gap(objects):
    """(gap g in metres from the handle's front face to its home mark, handle on the tang's line?)."""
    f, h = objects['file'], objects['handle']
    rf, rh = f.rotation(), h.rotation()
    rel = rf.T @ (np.array(h.origin) - np.array(f.origin))
    aligned = float(np.dot(rf[:, 1], rh[:, 1])) >= math.cos(math.radians(8))
    in_line = aligned and abs(rel[0]) <= .004 and abs(rel[2]) <= .004
    return float(rel[1] + RECESS), bool(in_line)


def score_terms(objects):
    f, h, tray = objects['file'], objects['handle'], objects['tray']
    hands = grippers()
    gap, in_line = handle_gap(objects)
    seated = in_line and gap < .0025          # rounds to 2 mm or less
    partway = in_line and .0025 <= gap < .0305  # 3 to 30 mm
    on = seated or partway
    # Irreversible: the bare-tang file left lying in the tray (no gripper within 2.5 cm), handle not on.
    bare_in_tray = _in_tray_footprint(f, tray) and _clear(f, hands, .025) and not on
    unsafe = ever('bare_in_tray', bare_in_tray or TEST_UNSAFE)
    fallen = ever('fallen', any(float(s.low[2]) < TOP - .020 for s in (f, h)))
    in_tray = _inside_tray(f, tray) and _inside_tray(h, tray)
    clear = _clear(f, hands, .030) and _clear(h, hands, .030)
    slow = all(np.linalg.norm(np.array(s.lin_vel)) < .005 for s in (f, h))
    still = held_for('still', in_tray and clear and slow) >= 1.0
    return dict(gap=gap, in_line=in_line, seated=seated, partway=partway, unsafe=unsafe, fallen=fallen,
                in_tray=in_tray, clear=clear, still=still, bare_in_tray=bare_in_tray)


def score(objects):
    t = score_terms(objects)
    _diag(objects, t)
    if t['unsafe'] or t['fallen']:
        return 0.0
    handle = 1.0 if t['seated'] else .25 if t['partway'] else 0.0
    place = 1.0 if (t['in_tray'] and t['clear'] and t['still']) else .75 if t['in_tray'] else .5
    return min(handle, place)


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (read-only)
def _force(arm):
    """Rough TCP force from the drive targets' lead (stiffness x error through the Jacobian); diagnostic only."""
    env = _RUN['env']
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, SHOULDER, WRIST
    art = env.unwrapped.scene[arm + '_robot']
    ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
    q = _np(art.data.joint_pos)[0][ids]
    qt = _np(art.data.joint_pos_target)[0][ids]
    tau = np.array([SHOULDER['stiffness']] * 3 + [WRIST['stiffness']] * 3) * (qt - q)
    p0, r0 = ik.tcp_pose(q, station.BASES[arm]); J = np.zeros((6, 6))
    for j in range(6):
        dq = q.copy(); dq[j] += 1e-5
        p1, r1 = ik.tcp_pose(dq, station.BASES[arm]); d = r1 @ r0.T
        J[:3, j] = (p1 - p0) / 1e-5
        J[3:, j] = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]]) / 2e-5
    return np.round(np.linalg.lstsq(J.T, tau, rcond=None)[0][:3], 1).tolist()


def _diag(objects, t):
    if not DIAG or _RUN['t'] is None or _RUN['step'] % 10 or _EVER.get('_printed') == _RUN['step']:
        return
    _EVER['_printed'] = _RUN['step']
    f, h = objects['file'], objects['handle']
    pitch = math.degrees(math.asin(float(np.clip(f.rotation()[2, 1], -1, 1))))
    hp = math.degrees(math.asin(float(np.clip(h.rotation()[2, 1], -1, 1))))
    try:
        forces = (_force('right'), _force('left'))
    except Exception as e:  # noqa: BLE001 - diagnostics must never break a run
        forces = repr(e)[:60]
    ins = getattr(h, 'joints', None)
    print('HAFT', round(_RUN['t'], 2), 'gap_mm', round(t['gap'] * 1000, 1), 'line', t['in_line'],
          'tray', t['in_tray'], 'clear', t['clear'], 'unsafe', t['unsafe'], 'pitch', round(pitch, 1), round(hp, 1),
          'F', forces, 'file', np.round(np.array(f.origin) * 1000, 1).tolist(),
          'handle', np.round(np.array(h.origin) * 1000, 1).tolist(), 'ins', ins, flush=True)


# ---------------------------------------------------------------- scripted demonstration
_READ = {'ep': None, 'data': {}}
R_YAW = -math.pi / 2      # right hand closes across the file's width (-90 deg keeps joint 6 inside its limits)
L_YAW = 0.0
OPEN40, OPEN30, F_SHUT = -.020, -.015, 0.0
F_FILE = -.0085           # 17 mm between the pads on the 20 mm file: 3 mm inside its width
PARK = {'right': np.array([.10, -.45, TOP + .16]), 'left': np.array([.15, .25, TOP + .18])}
CARRY_Z = TOP + .135
HOME_AIM = .0015          # the hand aims at most 1.5 mm past the seated position
TAP_BACKS = (.025, .030, .035, .040, .045, .050, .050, .050)  # up to eight taps, each from a little further back
TAPS, TAP_S = len(TAP_BACKS), .15  # each tap driven in 0.15 s


def state_now(name):
    if _RUN['env'] is None:
        return None
    from yam_arena import scene
    return scene.object_states(_RUN['env'], [name])[name]


def read(key, fn):
    """Evaluate once per episode, when the plan first reaches the waypoint that needs it."""
    if _READ['ep'] != _RUN['episode']:
        _READ.update(ep=_RUN['episode'], data={})
    if key not in _READ['data']:
        _READ['data'][key] = np.asarray(fn(), dtype=float)
    return _READ['data'][key]


def _pose(name):
    st = state_now(name)
    if st is None:
        nominal = {'file': FILE_POS, 'handle': HANDLE_POS, 'tray': TRAY_POS}[name]
        return np.array(nominal, float), np.eye(3)
    return np.array(st.origin, float), np.array(st.rotation(), float)


class Step(Waypoint):
    """A waypoint whose position is computed when the plan reaches it (demonstration sensing)."""

    def __init__(self, secs, fn, finger, label, yaw):
        self.duration = secs; self.fn = fn; self.finger = finger; self.label = label; self._yaw = yaw

    @property
    def pos(self):
        return tuple(self.fn())

    @property
    def yaw(self):
        return self._yaw


def demo():
    """The right hand does the work; the left hand stays parked. The steel block, not an arm, takes the push:
    the hand drives the file's tang into the handle, whose butt comes to rest on the block, and seats it with
    short taps (TM 9-867: "Tap back end of handle on a flat surface until file is properly seated"), here with
    the handle's butt striking the block."""
    right, left = [], []

    def add(secs, fr, gr, label):
        right.append(Step(secs, fr, gr, label, R_YAW))
        left.append(Step(secs, lambda: PARK['left'], F_SHUT, label, L_YAW))

    def axis_dir():
        return read('axis', lambda: _pose('handle')[1] @ np.array([0, 1., 0]))

    def handle_at():
        """The handle's front face as the file is lined up (offline: where it starts)."""
        return read('h0', lambda: _pose('handle')[0])

    def on_file(key, along, offline=None):
        """A point on the file's axis `along` metres from the shoulder toward the point, read once.
        Offline (no simulator) the file is where the plan has put it: `offline()` gives its shoulder."""
        def fn():
            if state_now('file') is None and offline is not None:
                return offline() + np.array([0, -along, 0])
            o, r = _pose('file')
            return o + r @ np.array([0, -along, 0])
        return read(key, fn)

    def tcp_on(key, along, dz=0., offline=None):
        return lambda: on_file(key, along, offline) + np.array([0, 0, .0115 + dz])

    def over_tray(z, dy=0.):
        def fn():
            o, _ = _pose('tray')
            return np.array([o[0], o[1] + dy, z])
        return lambda: read('tray%.4f' % z, fn)

    park_r = lambda: PARK['right']
    lift_to = lambda key, along: (lambda: on_file(key, along) * [1, 1, 0] + [0, 0, CARRY_Z])
    if UNSAFE_DEMO:
        add(2.0, tcp_on('g0', BODY_GRIP, .06), OPEN40, 'over the file')
        add(1.5, tcp_on('g0', BODY_GRIP), OPEN40, 'down to the file')
        add(1.0, tcp_on('g0', BODY_GRIP), F_FILE, 'pinch the bare file')
        add(2.0, lift_to('g0', BODY_GRIP), F_FILE, 'lift')
        add(2.5, over_tray(CARRY_Z, -.022), F_FILE, 'carry the bare file over the tray')
        add(2.0, over_tray(TOP + .039, -.022), F_FILE, 'lower into the tray')
        add(1.0, over_tray(TOP + .039, -.022), OPEN30, 'let go')
        add(1.5, over_tray(CARRY_Z, -.022), OPEN30, 'rise')
        add(2.0, park_r, OPEN40, 'park')
        add(3.0, park_r, OPEN40, 'hold clear')
        return {'right': right, 'left': left}

    # 1. Take the file by its body at its own mass centre (pads on its smooth edges), lift it to the bore's
    # height and line its tang up with the handle's bore. The handle is never gripped: it stays on its flat.
    add(2.0, tcp_on('g0', BODY_GRIP, .06), OPEN40, 'over the file')
    add(1.5, tcp_on('g0', BODY_GRIP), OPEN40, 'down to the file')
    add(1.0, tcp_on('g0', BODY_GRIP), F_FILE, 'pinch the file')

    def grip0():
        return on_file('g0', BODY_GRIP) + np.array([0, 0, .0115])

    def lifted():
        def fn():
            p = grip0()
            p[2] = handle_at()[2] + .012
            return p
        return read('lift', fn)

    def lined_up():
        def fn():
            if state_now('file') is None:
                fo = _pose('file')[0] + (lifted() - grip0())
            else:
                fo = _pose('file')[0]
            want = handle_at() + axis_dir() * -(TANG + .008)
            return lifted() + (want - fo)
        return read('line', fn)

    add(1.5, lifted, F_FILE, 'lift to the bore')
    add(1.5, lined_up, F_FILE, 'line the tang up')

    # 2. Push the tang in. As its edges catch the insert the handle slides back until its butt meets the block.
    home = lambda: lined_up() + axis_dir() * (TANG + .008 + RECESS)
    aim = lambda: home() + axis_dir() * HOME_AIM
    add(3.0, aim, F_FILE, 'push the tang in')
    add(0.5, aim, F_FILE, 'settle')

    # 3. Seat the handle with short taps (TM 9-867: tap the handle's back end on a flat surface). Each tap reads
    # where file and handle lie now, draws them back together 25-50 mm, then drives the file to 1.5 mm past home
    # on the handle as it lies, so the handle's butt strikes the block and the file runs on into the handle.
    # The hand never aims more than 1.5 mm past home. Once g < 1.5 mm the hand stays put.
    def tap_plan(i):
        def fn():
            a = axis_dir()
            if state_now('file') is None:  # offline: the parts are where the plan left them; replay every tap
                hp, shoulder = handle_at(), handle_at() + a * (RECESS + HOME_AIM)
                off, gap = aim() - shoulder, math.inf
            else:
                now = {n: state_now(n) for n in ('file', 'handle')}
                gap, _ = handle_gap(now)
                hp, shoulder = np.array(now['handle'].origin), np.array(now['file'].origin)
                off = grippers()['right'][0] - shoulder      # where the pads hold the file now
            drive = hp + a * (RECESS + HOME_AIM) + off
            if gap < .0015:
                return np.concatenate([drive, drive])
            return np.concatenate([shoulder - a * TAP_BACKS[i] + off, drive])
        return read('tap%d' % i, fn)

    for i in range(TAPS):
        add(.5, (lambda i=i: tap_plan(i)[:3]), F_FILE, 'draw back %d' % i)
        add(TAP_S, (lambda i=i: tap_plan(i)[3:]), F_FILE, 'tap %d' % i)
        add(.3, (lambda i=i: tap_plan(i)[3:]), F_FILE, 'dwell %d' % i)
    last = lambda: tap_plan(TAPS - 1)[3:] - axis_dir() * HOME_AIM
    add(0.5, last, F_FILE, 'ease off')

    # 4. Let go, take the handled file at the pair's mass centre, draw it off the block, lift and lay it in the tray.
    def seated_shoulder():
        """Offline only (the replay cannot seat): the shoulder where the plan left it."""
        return handle_at() + axis_dir() * RECESS

    add(1.0, last, OPEN40, 'let go')
    add(1.2, lambda: last() + np.array([0, 0, .035]), OPEN40, 'rise')
    add(1.5, tcp_on('g1', CARRY_GRIP, .032, seated_shoulder), OPEN40, 'over the handled file')
    add(1.5, tcp_on('g1', CARRY_GRIP, -.003, seated_shoulder), OPEN40, 'down')
    add(1.0, tcp_on('g1', CARRY_GRIP, -.003, seated_shoulder), F_FILE, 'pinch')
    add(1.0, tcp_on('g1', CARRY_GRIP, -.0015, seated_shoulder), F_FILE, 'raise 1.5 mm')
    drawn = lambda: on_file('g1', CARRY_GRIP) - axis_dir() * .080 + np.array([0, 0, .0115 - .0015])
    add(1.5, drawn, F_FILE, 'draw it off the block')
    add(2.0, lambda: drawn() * [1, 1, 0] + [0, 0, CARRY_Z], F_FILE, 'lift')
    add(3.0, over_tray(CARRY_Z), F_FILE, 'carry over the tray')
    add(2.0, over_tray(TOP + .045), F_FILE, 'lower into the tray')
    add(1.0, over_tray(TOP + .045), OPEN30, 'let go')
    add(1.5, over_tray(CARRY_Z), OPEN30, 'rise')
    add(2.0, park_r, OPEN40, 'park')
    add(3.0, park_r, OPEN40, 'hold clear')
    return {'right': right, 'left': left}
