"""A taped paper tic-tac-toe game, a red felt-tip pen and a steel fork stand.
Rows run from far to near, columns from left to right. X starts and turns alternate.
Use the printed game to decide what to write. The free marker has a passive sprung
nib; writing appears as permanent near-black ink. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl, ball
from yam_joints import Mechanism, Link, Slide
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
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


INK_MODE = "instancer"
DEMO_PACE = 1.0
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1800
BOARD_XY = np.array([.195, -.155])
CELL_SIZE, BOARD_SIZE, SHEET_SIZE = .045, .135, .165
PAPER_TOP = TOP + .001
CELLS = tuple((r, c) for r in range(3) for c in range(3))
INK_Q, INK_Z, DOT_PITCH, DOT_R = .001, .002, .0025, .0022
STAND_XY = np.array([.155, -.310])
FORK_TOP = TOP + .108
PEN_R = np.eye(3)
NIB_TIP = np.array([0., 0., -.135])
COLLAR_UNDER = np.array([0., 0., -.039])
GRIP_LOCAL = np.array([0., 0., -.0175])
RIDE_LOW, SHORTFALL = .0035, .0014
ROOT_ABOVE_TCP = .0155
GRASP_TCP = FORK_TOP + .001 + .039 - ROOT_ABOVE_TCP
TRAVEL_TCP = TOP + .170
DRAW_TCP = PAPER_TOP + .1195 + RIDE_LOW - SHORTFALL - .0055
FORK_TCP = GRASP_TCP + .008 + RIDE_LOW
RELEASE_TCP = FORK_TOP + .039 - ROOT_ABOVE_TCP + RIDE_LOW + .0025
EXIT_XY = STAND_XY + [0, .070]


def cell_center(cell):
    row, col = cell
    if row not in range(3) or col not in range(3):
        raise ValueError(cell)
    return BOARD_XY + [(1-row)*CELL_SIZE, (1-col)*CELL_SIZE]


def nib_tip(pen_state):
    """Use the reported nib body pose, including base tilt and passive slide."""
    return pen_state.link_to_world("nib", NIB_TIP)


def writing(pen_state):
    tip = nib_tip(pen_state)
    return bool(pen_state.joints["nib"] >= INK_Q and
                abs(tip[2] - PAPER_TOP) <= INK_Z and
                np.all(np.abs(tip[:2] - BOARD_XY) <= SHEET_SIZE/2))


SHEET = Part("sheet", (*BOARD_XY, TOP), [box((0, 0, .0005), (SHEET_SIZE, SHEET_SIZE, .001))],
             color=(.97, .97, .94), contact_offset=.0002)
# Part-frame +z is UP. Positive q retracts the nib upward; negative rest
# preloads it down onto the lower stop. Every shape is authored at q=0.
PEN = Mechanism("pen", (*STAND_XY, FORK_TOP + .001 + .039),
    shapes=[box((0, 0, -.0175), (.014, .014, .035), color=(.12, .13, .16)),
            cyl((0, 0, -.037), .015, .004),
            cyl((0, 0, -.074), .007, .070),
            cyl((0, 0, -.112), .006, .006),
            cyl((0, 0, -.118), .005, .006),
            cyl((0, 0, -.124), .004, .006)],
    links=[Link("nib", [cyl((0, 0, -.1265), .0025, .011), ball((0, 0, -.132), .003)],
                Slide("z", (0, 0, -.121), 0, .010, spring=120., rest=-.002, damping=1.),
                .003, color=(.025, .025, .03), friction=(.15, .10))],
    mass=.020, color=(.95, .18, .035), friction=(1.2, 1.), contact_offset=.0003, grip=0)
# Fork opens toward +y; 20 mm slot vs 14 mm barrel gives 3 mm per side.
STAND = Part("stand", (*STAND_XY, TOP),
    [box((0, -.034, .052), (.038, .008, .104)),
     box((0, -.029, .106), (.038, .018, .004))] +
    [box((s*.013, .0025, .106), (.006, .045, .004)) for s in (-1, 1)],
    color=(.25, .35, .42), contact_offset=.0003)
OBJECTS = [SHEET, PEN, STAND]



# Board strings use far-to-near rows, left-to-right columns.
LAYOUTS = (
    '..X.OX.XO',
    '...XOXOX.',
    'X.OX..OX.',
    'OX.XOX...',
    '..X.OO.XX',
    'OXXO....X',
    '.OXXO.X..',
    'XX...XO.O',
    'X.XO..OXO',
    '..OXX..O.',
    '.XOOX....',
    '.OX..XO..',
    'X.XXO.O.O',
    '.O...X.OX',
    'X.O.OXXO.',
    '....OOXX.',
)
LINES = ((0,1,2), (3,4,5), (6,7,8), (0,3,6), (1,4,7), (2,5,8), (0,4,8), (2,4,6))
LINE_NAMES = ("row0", "row1", "row2", "col0", "col1", "col2", "diag_back", "diag_forward")
TEST_BOARD = None
TEST_STROKES = None
_CURRENT_BOARD = LAYOUTS[0]


def current_board():
    return TEST_BOARD if TEST_BOARD is not None else _CURRENT_BOARD


def winning_cells(board, symbol):
    return [divmod(i, 3) for i, c in enumerate(board) if c == "." and
            any(i in line and all(k == i or board[k] == symbol for k in line) for line in LINES)]


def winning_move(board):
    """Infer the turn and the unique win from the visible printed game."""
    if len(board) != 9 or set(board)-set("XO."):
        raise ValueError("board must contain nine X/O/dot characters")
    x, o = board.count("X"), board.count("O")
    if x-o not in (0, 1) or any(all(board[k] == s for k in l) for l in LINES for s in "XO"):
        raise ValueError("game must be legal and unfinished")
    symbol = "X" if x == o else "O"
    cells = winning_cells(board, symbol)
    if len(cells) != 1:
        raise ValueError("game must have exactly one winning move")
    return symbol, cells[0]


def layout_self_check():
    assert len(LAYOUTS) == len(set(LAYOUTS)) == 16
    counts = {(s, t): 0 for s in "XO" for t in (False, True)}
    cells, lines, rows = set(), [0]*8, []
    for seed, board in enumerate(LAYOUTS):
        symbol, cell = winning_move(board)
        assert 4 <= board.count("X")+board.count("O") <= 6
        threat = winning_cells(board, "O" if symbol == "X" else "X")
        # A blocking move must be distinguishable from the winning move.
        assert cell not in threat
        counts[symbol, bool(threat)] += 1
        cells.add(cell)
        completed = [j for j,l in enumerate(LINES) if cell[0]*3+cell[1] in l and
                     all(k == cell[0]*3+cell[1] or board[k] == symbol for k in l)]
        for j in completed: lines[j] += 1
        rows.append((seed, board, symbol, cell, completed, threat))
    assert set(counts.values()) == {4} and cells == set(CELLS) and min(lines) >= 2
    return rows, lines


layout_self_check()
ZONE_EXPAND, OUTSIDE_MAX = .0025, .001
# Glyph shape tests read the ink centreline only (what a top photo shows):
# stroke count, order, direction and retracing never change them. The two
# length allowances (OUTSIDE_MAX, STRAY_MAX) count centreline path, and every
# separate nib touch counts at least INK_DOT, one ink-disc diameter, so a dab
# of ink is never free.
INK_DOT = 2*DOT_R
RESAMPLE_PITCH, STRAY_MAX, INK_GAP_MAX = .0005, .005, INK_DOT
X_DEVIATION, X_ANGLE, CENTRE_MAX = .003, 25., .008
X_LENGTH_MIN, X_ARM_MIN = .022, .008
O_RADIUS_MIN, O_BAND, O_GAP_DEG = .010, .18, 30.
HANG_Z, HANG_TILT = .002, 5.
# Derive support limits from the unchanged physical shapes. The collar is round:
# use its chord at the farther prong's inner edge, rather than its full radius,
# so at least 3 mm along BOTH prongs remains underneath it even off-centre.
COLLAR_RADIUS = PEN.base_shapes[1].radius
BARREL_RADIUS = PEN.base_shapes[2].radius
FORK_SLOT_HALF = abs(STAND.shapes[2].center[0]) - STAND.shapes[2].size[0]/2
FORK_REAR_FACE = STAND.shapes[1].center[1] + STAND.shapes[1].size[1]/2
FORK_TIP_Y = STAND.shapes[2].center[1] + STAND.shapes[2].size[1]/2
HANG_SUPPORT_MIN, HANG_X_SLOP = .003, .0005
HANG_X = FORK_SLOT_HALF - BARREL_RADIUS + HANG_X_SLOP
HANG_Y_MIN = FORK_REAR_FACE + BARREL_RADIUS


def hang_y_max(x_offset):
    """Last supported +y axis position relative to the stand (metres)."""
    chord = math.sqrt(max(0., COLLAR_RADIUS**2 - (FORK_SLOT_HALF + abs(x_offset))**2))
    return FORK_TIP_Y + chord - HANG_SUPPORT_MIN


HANG_Y_MAX = hang_y_max(0.)
STILL_SPEED, STILL_JOINT_SPEED = .010, .050
AIM_CORRECTION = np.array([.0008, .0018])
DRAW_SPEED, O_COMMAND_RADIUS = .020, .0165

# Pure-NumPy path analysis. Clip segments before resampling, so crossing a cell
# never bridges unrelated outside points and outside lengths remain measurable.
def clip_paths(strokes, low, high):
    low, high = np.asarray(low), np.asarray(high)
    out = []
    for stroke in strokes:
        p = np.asarray(stroke, dtype=float).reshape(-1, 2)
        run = []
        if len(p) == 1 and np.all(p[0] >= low) and np.all(p[0] <= high):
            out.append(p.copy())
        for a, b in zip(p[:-1], p[1:]):
            delta, lo, hi = b-a, 0., 1.
            for k in range(2):
                if abs(delta[k]) < 1e-15:
                    if a[k] < low[k] or a[k] > high[k]: lo, hi = 1., 0.
                else:
                    u, v = sorted(((low[k]-a[k])/delta[k], (high[k]-a[k])/delta[k]))
                    lo, hi = max(lo, u), min(hi, v)
            if lo <= hi:
                aa, bb = a+lo*delta, a+hi*delta
                if run and np.linalg.norm(run[-1]-aa) > 1e-9:
                    out.append(np.asarray(run)); run = []
                if not run: run.append(aa)
                run.append(bb)
            elif run:
                out.append(np.asarray(run)); run = []
        if run: out.append(np.asarray(run))
    return out


def path_length(p):
    return float(np.linalg.norm(np.diff(np.asarray(p).reshape(-1, 2), axis=0), axis=1).sum())


def resample(p, pitch=.0005):
    p = np.asarray(p).reshape(-1, 2)
    if len(p) < 2: return p.copy()
    ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
    p = p[np.r_[True, ds > 1e-12]]
    if len(p) < 2: return p.copy()
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    u = np.r_[np.arange(0, s[-1], pitch), s[-1]]
    return np.column_stack([np.interp(u, s, p[:, k]) for k in range(2)])


def clip_touches(strokes, low, high):
    """Clip every nib touch (one stroke) to a box; returns (path, counted length).
    A touch counts its centreline length but at least INK_DOT: a shorter touch is
    scaled up evenly along its path (each clipped part keeps its share), and a
    stationary touch counts INK_DOT at its point."""
    out = []
    for stroke in strokes:
        p = np.asarray(stroke, dtype=float).reshape(-1, 2)
        total = path_length(p)
        for part in clip_paths([p], low, high):
            out.append((part, path_length(part)*max(1., INK_DOT/total) if total > 1e-12 else INK_DOT))
    return out


def ink_samples(parts):
    """Unordered centreline samples: 0.5 mm segment midpoints sharing their part's
    counted length in proportion to segment length; a dot is one sample."""
    pts, wts = [], []
    for p, weight in parts:
        q = resample(p, RESAMPLE_PITCH)
        if len(q) < 2:
            pts.append(q.reshape(-1, 2)[:1]); wts.append(np.full(len(q[:1]), weight))
        else:
            seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
            pts.append((q[1:]+q[:-1])/2); wts.append(seg*(weight/seg.sum()))
    if not pts: return np.zeros((0, 2)), np.zeros(0)
    return np.vstack(pts), np.concatenate(wts)


def _line(p):
    c = p.mean(axis=0)
    return c, np.linalg.svd(p-c, full_matrices=False)[2][0]


def _line_dist(p, line):
    c, v = line
    return np.abs((p-c) @ np.array([-v[1], v[0]]))


def x_test(P, W, centre):
    """Two straight lines fit the ink, cross near the centre, and the ink along
    each continues at least X_ARM_MIN past the crossing on both sides."""
    if len(P) < 4: return False, {}
    c0 = P.mean(axis=0)
    lines = [(c0, np.array([1., 1.])/math.sqrt(2)), (c0, np.array([1., -1.])/math.sqrt(2))]
    for it in range(12):
        d = np.stack([_line_dist(P, l) for l in lines])
        near = d.argmin(axis=0)
        new = []
        for k in range(2):
            sel = near == k
            if it >= 4: sel &= d[k] <= X_DEVIATION
            if sel.sum() < 2: return False, {}
            new.append(_line(P[sel]))
        lines = new
    d = np.stack([_line_dist(P, l) for l in lines])
    stray = float(W[d.min(axis=0) > X_DEVIATION].sum())
    angles = sorted(math.degrees(math.atan2(v[1], v[0])) % 180 for _, v in lines)
    (ca, va), (cb, vb) = lines
    mat = np.column_stack((va, -vb))
    if abs(np.linalg.det(mat)) < .1: return False, {}
    cross = ca + np.linalg.solve(mat, cb-ca)[0]*va
    offset = float(np.linalg.norm(cross-centre))
    ok = (stray <= STRAY_MAX and abs(angles[0]-45) <= X_ANGLE and abs(angles[1]-135) <= X_ANGLE
          and offset <= CENTRE_MAX)
    arms = []
    for k, (_, v) in enumerate(lines):
        t = np.sort((P[d[k] <= X_DEVIATION]-cross) @ v)
        gap = float(np.max(np.diff(t))) if len(t) > 1 else math.inf
        arm = min(-t[0], t[-1]) if len(t) else -math.inf
        arms.append(arm)
        ok = ok and gap <= INK_GAP_MAX and t[-1]-t[0] >= X_LENGTH_MIN and arm >= X_ARM_MIN
    return ok, {"x_stray_mm": round(stray*1000, 3), "x_angles_deg": [round(a, 2) for a in angles],
                "crossing_mm": round(offset*1000, 3), "short_arm_mm": [round(a*1000, 3) for a in arms]}


def _circle(p):
    """Least-squares circle: x^2+y^2 = 2ax + 2by + c."""
    sol = np.linalg.lstsq(np.column_stack((2*p, np.ones(len(p)))), (p**2).sum(axis=1), rcond=None)[0]
    c = sol[:2]
    return c, math.sqrt(max(float(sol[2] + c @ c), 0.))


def o_test(P, W, centre):
    """Fit a circle to all samples, refit it to the samples within O_BAND of its
    radius; for the refitted circle: all but STRAY_MAX of the ink within O_BAND,
    no angular gap over O_GAP_DEG, centre near the cell centre, radius >= minimum."""
    if len(P) < 3: return False, {}
    c, R = _circle(P)
    inband = np.abs(np.linalg.norm(P-c, axis=1)-R) <= O_BAND*R
    if inband.sum() >= 3:
        c, R = _circle(P[inband])
        inband = np.abs(np.linalg.norm(P-c, axis=1)-R) <= O_BAND*R
    stray = float(W[~inband].sum())
    ang = np.sort(np.degrees(np.arctan2(P[inband, 1]-c[1], P[inband, 0]-c[0])) % 360)
    gap = float(max(np.max(np.diff(ang), initial=0.), 360-ang[-1]+ang[0])) if len(ang) else 360.
    offset = float(np.linalg.norm(c-centre))
    ok = (R >= O_RADIUS_MIN and stray <= STRAY_MAX and gap <= O_GAP_DEG and offset <= CENTRE_MAX)
    return ok, {"o_radius_mm": round(R*1000, 3), "o_stray_mm": round(stray*1000, 3),
                "o_gap_deg": round(gap, 2), "o_centre_mm": round(offset*1000, 3)}


def classify_cell(strokes, cell):
    centre = cell_center(cell)
    half = CELL_SIZE/2+ZONE_EXPAND
    P, W = ink_samples(clip_touches(strokes, centre-half, centre+half))
    details = {"length_mm": round(float(W.sum())*1000, 3)}
    if not len(P): return "none", details
    x_ok, xd = x_test(P, W, centre)
    o_ok, od = o_test(P, W, centre)
    details.update(xd); details.update(od)
    return ("unclear" if x_ok and o_ok else "X" if x_ok else "O" if o_ok else "unclear"), details


def outside_length(strokes, cell):
    half = CELL_SIZE/2+ZONE_EXPAND
    c = cell_center(cell)
    # Path travelled, so retracing outside the zone counts twice.
    sheet = sum(w for _, w in clip_touches(strokes, BOARD_XY-SHEET_SIZE/2, BOARD_XY+SHEET_SIZE/2))
    inside = sum(w for _, w in clip_touches(strokes, c-half, c+half))
    return max(0., sheet-inside)

_PATH_EPISODE = None
_LAST_PATH_TOKEN = None
_PREV_WRITING = False
_STROKES = []
_START_ORIGIN = None
_LAST_CLASSIFY = None


def reset_path():
    global _PATH_EPISODE, _LAST_PATH_TOKEN, _PREV_WRITING, _START_ORIGIN, _LAST_CLASSIFY
    _PATH_EPISODE = _RUN["episode"]
    _LAST_PATH_TOKEN = _START_ORIGIN = _LAST_CLASSIFY = None
    _PREV_WRITING = False
    _STROKES.clear()


def record_path(pen):
    global _LAST_PATH_TOKEN, _PREV_WRITING, _START_ORIGIN
    if _PATH_EPISODE != _RUN["episode"]: reset_path()
    if _START_ORIGIN is None: _START_ORIGIN = np.asarray(pen.origin).copy()
    token = (_RUN["episode"], _RUN["step"])
    if token == _LAST_PATH_TOKEN: return
    if _LAST_PATH_TOKEN is not None and token[1] != _LAST_PATH_TOKEN[1]+1: _PREV_WRITING = False
    now = writing(pen)
    if now:
        if not _PREV_WRITING: _STROKES.append([])
        _STROKES[-1].append(nib_tip(pen)[:2].copy())
    _PREV_WRITING, _LAST_PATH_TOKEN = now, token


def score(objects):
    global _LAST_CLASSIFY
    pen = objects["pen"]
    record_path(pen)
    strokes = TEST_STROKES if TEST_STROKES is not None else _STROKES
    symbol, cell = winning_move(current_board())
    glyph, details = classify_cell(strokes, cell)
    collar = pen.to_world(COLLAR_UNDER)
    dx, dy = collar[:2] - STAND_XY
    hanging = (abs(collar[2]-FORK_TOP) <= HANG_Z and
               abs(dx) <= HANG_X+1e-12 and
               HANG_Y_MIN-1e-12 <= dy <= hang_y_max(dx)+1e-12 and
               pen.tilt_deg() <= HANG_TILT)
    outside = outside_length(strokes, cell)
    # Permanent ink violation and stillness timer run on EVERY call, before returns.
    violation = ever("outside_winning_zone", outside > OUTSIDE_MAX+1e-12)
    end = (not violation and glyph == symbol and hanging and released(pen) and
           pen.still(max_speed=STILL_SPEED, max_joint_speed=STILL_JOINT_SPEED))
    end_s = held_for("written_hung_released_still", end)
    token = (_RUN["episode"], _RUN["step"])
    if _RUN["env"] is not None and _RUN["t"] is not None and token != _LAST_CLASSIFY and token[1] % 40 == 0:
        print(f"CLASSIFY t={_RUN['t']:.3f} board={LAYOUTS.index(current_board())} "
              f"symbol={symbol} cell={cell} glyph={glyph} details={details} "
              f"outside_mm={outside*1000:.3f}", flush=True)
        _LAST_CLASSIFY = token
    if violation or glyph != symbol: return 0.0
    if not hanging: return .6
    return 1.0 if end and end_s >= STILL_S else .8


def success(objects):
    return score(objects) == 1.0


def strokes_x(cell, span=.030):
    c, d = cell_center(cell), span/2
    return [np.linspace(c+[-d,-d], c+[d,d], 90), np.linspace(c+[-d,d], c+[d,-d], 90)]


def strokes_o(cell, radius=.015, sweep=390.):
    a = np.radians(np.linspace(0, sweep, 105))
    return [cell_center(cell)+radius*np.column_stack((np.cos(a),np.sin(a)))]


def nominal_strokes(board_index):
    symbol, cell = winning_move(LAYOUTS[board_index])
    return strokes_x(cell) if symbol == "X" else strokes_o(cell)


def wrong_cell(board_index):
    board = LAYOUTS[board_index]
    _, answer = winning_move(board)
    return next(c for c in CELLS if board[c[0]*3+c[1]] == "." and c != answer)


def scribble(cell):
    c = cell_center(cell)
    return [c+np.array([(x,y) for y in np.linspace(-.020,.020,21)
                     for x in ((-.020,.020) if round((y+.020)/.002)%2 == 0 else (.020,-.020))])]


def stray(cell):
    return [np.array([cell_center(cell), cell_center(cell)+[.005,0]])]


def polygon(cell, radius, sides, phase=90.):
    a = np.radians(phase+np.arange(sides+1)*360./sides)
    return [cell_center(cell)+radius*np.column_stack((np.cos(a), np.sin(a)))]


def strokes_v(cell, arm=.018, overrun=0.):
    """Two 45/135-degree strokes that meet (overrun 0) or cross by `overrun` metres."""
    c, e = cell_center(cell), overrun/math.sqrt(2)
    return [np.linspace(c+[-arm,-arm], c+[e,e], 60), np.linspace(c+[-arm,arm], c+[e,-e], 60)]


def strokes_lambda(cell, arm=.018):
    c = cell_center(cell)
    return [np.linspace(c+[-arm,-arm], c+[arm,arm], 90), np.linspace(c, c+[arm,-arm], 60)]


# The policy builds plans BEFORE embodiment.reset chooses the seed. Only positions
# are deferred; counts and durations are fixed. ArmPlan reads pos when playing.
# The resolver uses the printed starting board, never poses, joints or score().
class _DrawingPlan:
    def __init__(self):
        self.board, self.points = None, None

    def resolve(self):
        board = current_board()
        if board != self.board:
            self.points = _drawing_points(board)
            self.board = board
        return self.points


class _DrawingWaypoint(Waypoint):
    def __init__(self, duration, resolver, index, label):
        self.duration = duration*DEMO_PACE
        self.resolver, self.index = resolver, index
        self.finger, self.label, self.yaw = FINGER_CLOSED, label, 0.

    @property
    def pos(self):
        return self.resolver.resolve()[self.index]


def _drawing_spec(board):
    symbol, cell = winning_move(board)
    c = cell_center(cell)
    # Measured arm offset, the same (+0.8,+1.8) mm for EVERY layout.
    c = c+AIM_CORRECTION
    xpaths = [c+np.array([[-.015,-.015],[.015,.015]]), c+np.array([[-.015,.015],[.015,-.015]])]
    angles = np.arange(53)*2*np.pi/48
    circle = c+O_COMMAND_RADIUS*np.column_stack((np.cos(angles),np.sin(angles)))
    hover = DRAW_TCP+.010
    anchor = xpaths[0][0] if symbol == "X" else circle[0]
    rows = []
    def add(dt, xy, z, label): rows.append((dt, (*xy,z), label))
    add(1.5, anchor, TRAVEL_TCP, "draw first hover")
    add(.8, anchor, TRAVEL_TCP, "draw first settle")
    # Both symbol plans reserve the identical X and O slots. Unused slots
    # hold the nib 10 mm above drawing height; no extra ink is produced.
    for i,path in enumerate(xpaths):
        active = symbol == "X"
        a,b = path if active else (anchor,anchor)
        add(.7, a, TRAVEL_TCP if i == 0 else hover, f"X{i} hover")
        add(.8 if i == 0 else .3, a, TRAVEL_TCP if i == 0 else hover, f"X{i} settle")
        add(.7, a, DRAW_TCP if active else hover, f"X{i} lower")
        add(math.sqrt(2)*.030/DRAW_SPEED, b, DRAW_TCP if active else hover, f"X{i} stroke")
        add(.3, b, DRAW_TCP if active else hover, f"X{i} dwell")
        add(.6, b, hover, f"X{i} lift")
    active = symbol == "O"
    if not active: circle = np.repeat(xpaths[-1][-1][None,:],53,axis=0)
    add(.7, circle[0], hover, "O hover")
    add(.8, circle[0], hover, "O settle")
    add(.7, circle[0], DRAW_TCP if active else hover, "O lower")
    dt = 2*O_COMMAND_RADIUS*math.sin(math.pi/48)/DRAW_SPEED
    for i,xy in enumerate(circle[1:]): add(dt, xy, DRAW_TCP if active else hover, f"O stroke {i:02}")
    add(.3, circle[-1], DRAW_TCP if active else hover, "O dwell")
    add(.6, circle[-1], hover, "O lift")
    add(.7, circle[-1], TRAVEL_TCP, "draw travel lift")
    return rows


def _drawing_points(board):
    return [pos for _,pos,_ in _drawing_spec(board)]


def demo():
    def w(dt, xy, z, label, finger=FINGER_CLOSED):
        return Waypoint(dt*DEMO_PACE, (*xy,z), finger, label)
    plan = [w(1.8, STAND_XY, TOP+.170, "grip hover", FINGER_OPEN),
            w(1.2, STAND_XY, GRASP_TCP, "grip descend", FINGER_OPEN),
            w(.8, STAND_XY, GRASP_TCP, "grip close"),
            w(.8, STAND_XY, FORK_TCP, "fork lift"),
            w(1.3, EXIT_XY, FORK_TCP, "fork withdraw"),
            w(.8, EXIT_XY, TRAVEL_TCP, "travel rise")]
    resolver = _DrawingPlan()
    # Only timing/labels are taken from board 0 here; all positions resolve later.
    plan += [_DrawingWaypoint(dt,resolver,i,label) for i,(dt,_,label) in enumerate(_drawing_spec(LAYOUTS[0]))]
    plan += [w(1.4, EXIT_XY, TRAVEL_TCP, "park travel"),
             w(.8, EXIT_XY, FORK_TCP, "park descend"),
             w(1.3, STAND_XY, FORK_TCP, "park enter"),
             w(.8, STAND_XY, RELEASE_TCP, "park lower"),
             w(.8, STAND_XY, RELEASE_TCP, "park release", FINGER_OPEN),
             w(1., STAND_XY, TOP+.170, "pads clear", FINGER_OPEN),
             w(1., STAND_XY+[0,-.075], TOP+.170, "back off", FINGER_OPEN),
             w(3.5, STAND_XY+[0,-.075], TOP+.170, "stillness tail", FINGER_OPEN)]
    return {"right":plan}


def dabs(points):
    """Separate stationary nib touches, one ink disc each."""
    return [np.asarray(p, dtype=float).reshape(1, 2) for p in points]


def ring_points(cell, radius, step_deg):
    a = np.radians(np.arange(0., 360., step_deg))
    return cell_center(cell)+radius*np.column_stack((np.cos(a), np.sin(a)))


def x_points(cell, half=.015, pitch=.003):
    t = np.arange(-half, half+1e-9, pitch)
    return cell_center(cell)+np.vstack((np.column_stack((t, t)), np.column_stack((t, -t))))


def symbol_strokes(symbol, cell):
    return strokes_x(cell) if symbol == "X" else strokes_o(cell)


def cheat_cases():
    """Synthetic permanent-path fixtures; never used by the demo."""
    b = 0
    symbol, cell = winning_move(LAYOUTS[b])
    other = wrong_cell(b)
    threat_board = next(i for i,b in enumerate(LAYOUTS) if winning_cells(b, "O" if winning_move(b)[0] == "X" else "X"))
    ts, _ = winning_move(LAYOUTS[threat_board])
    block = winning_cells(LAYOUTS[threat_board], "O" if ts == "X" else "X")[0]
    existing_o = divmod(LAYOUTS[0].index("O"), 3)
    # The 30 mm glyph extends 5 mm past the actual grid line into a neighbour.
    shift = np.array([.0125, 0.])
    xb = 8  # board 8: X to play in (0, 1)
    xcell = winning_move(LAYOUTS[xb])[1]
    return [
        ("wrong symbol in winning cell", b, symbol_strokes("X" if symbol == "O" else "O", cell)),
        ("right symbol in another empty cell", b, symbol_strokes(symbol, other)),
        ("block instead of win", threat_board, symbol_strokes(ts, block)),
        ("scribble filling winning cell", b, scribble(cell)),
        ("X over an existing O", b, strokes_x(existing_o)),
        ("X and O together", b, strokes_x(cell)+strokes_o(cell)),
        ("right mark plus 5 mm stray", b, nominal_strokes(b)+stray(other)),
        ("square instead of O", b, polygon(cell, .018, 4, 45.)),
        ("6 mm mark", b, strokes_x(cell,.006) if symbol == "X" else strokes_o(cell,.003)),
        ("right mark 5 mm over grid", b, [p+shift for p in nominal_strokes(b)]),
        ("pen hung no mark", b, []),
        ("V instead of X", xb, strokes_v(xcell)),
        ("lambda instead of X", xb, strokes_lambda(xcell)),
        ("X arms crossing by only 5 mm", xb, strokes_v(xcell, .018, .005)),
        ("triangle instead of O", b, polygon(cell, .018, 3)),
        ("small square instead of O", b, polygon(cell, .0123, 4, 45.)),
        ("dabbed O around the winning X", xb, strokes_x(xcell)+dabs(ring_points(xcell, .015, 15.))),
        ("dabbed X over a printed O", b, nominal_strokes(b)+dabs(x_points(existing_o))),
        ("dotted 24 mm line into another square", b, nominal_strokes(b)+
         dabs(cell_center(cell)+np.column_stack((np.zeros(9), -np.arange(.020, .0441, .003))))),
        ("right mark plus one dab in another square", b, nominal_strokes(b)+dabs([cell_center(other)])),
        ("right mark plus two dabs off the glyph", b, nominal_strokes(b)+
         dabs([cell_center(cell)+[.020, .020], cell_center(cell)+[-.020, -.020]])),
    ]


import ink
ink.install(sys.modules.get(__name__) or globals())
