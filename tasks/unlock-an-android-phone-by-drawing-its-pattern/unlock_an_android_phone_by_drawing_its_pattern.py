"""An Android phone (Google Pixel 7a: 152.0 x 72.9 x 9.0 mm, 193.5 g) lies loose and screen up on the desk, in
portrait, its top pointing away from the robot. Its lock screen shows stock Android's 3 x 3 pattern pad. The robot
unlocks it by drawing the phone's pattern, a Z through seven dots (top row left to right, the diagonal down to the
bottom left, the bottom row left to right), with one fingertip that never leaves the glass and touches no wrong dot.

The screen is live. Every control step the task reads where the robot's fingertip end faces are (a face touches when
it rests on the glass, within 0.2 mm), runs Android's own pattern rules on the touch (a port of the open-source
LockPatternView: hit radius 0.3 x the dot pitch, a dot is used once, a skipped middle dot is filled in, lifting the
finger submits the pattern, under 4 dots is refused) and redraws the screen: lit dots and the drawn line while the
finger moves, a red pattern for a wrong one, the home screen once unlocked.

Screen, touch detection, per-step loop and episode memory are adapted from the task folder
pinch-zoom-map-on-tablet-to-fit-route (its tablet kit); the phone, the pattern rules, the drawing, the scorer and the
demo are new.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts as _yp  # noqa: E402
from yam_parts import Part, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False
DEMO_WRONG = ""            # GPU-only copies: "tap" taps the seven dots one at a time, lifting between them
DEMO_PACE = 1.0


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
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

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
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


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
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ the phone
TOP = TABLE_TOP[SCENE]
MM = 0.001
# Google Pixel 7a (2023): 152.0 x 72.9 x 9.0 mm, 193.5 g, 6.1-inch 1080 x 2400 OLED (20:9), glass front, plastic
# back. Screen 63.6 x 141.3 mm from the 6.1-inch diagonal at 20:9, centred on the front (estimate).
PH_L, PH_W, PH_T = 0.152, 0.0729, 0.0090     # long side along x (top away from the robot), width along y
PH_MASS = 0.1935
PH_R = 0.009                                 # corner radius in plan (estimate from photos)
PH_X, PH_Y = 0.235, -0.135                   # centre on the desk
SCREEN_W, SCREEN_H = 63.6, 141.3             # mm; screen u to the right (world -y), v to the top (world +x)
GLASS_Z = PH_T                               # glass top above the phone's underside
DP_MM = SCREEN_W / (1080 / 2.625)            # one Android dp: 0.1545 mm (1080 px at density 2.625)
FRAME = (0.06, 0.06, 0.07)


def _phone():
    cx, cy = PH_L / 2 - PH_R, PH_W / 2 - PH_R
    shapes = [box((0.0, 0.0, PH_T / 2), (PH_L, PH_W - 2 * PH_R, PH_T)),
              box((0.0, 0.0, PH_T / 2), (PH_L - 2 * PH_R, PH_W, PH_T))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            shapes.append(cyl((sx * cx, sy * cy, PH_T / 2), PH_R, PH_T, "z"))
    # Plastic back on a laminate desk; the same material is the glass the fingertip slides on (estimate, as the
    # tablet kit): static 0.4, dynamic 0.35.
    return Part("phone", (PH_X, PH_Y, TOP), shapes, mass=PH_MASS, color=FRAME, friction=(0.4, 0.35),
                contact_offset=0.001)


PHONE = _phone()
OBJECTS = [PHONE]
PRESSED_PARTS = ("phone",)            # the fingertip presses on the glass and slides on it; nothing is grasped

# ------------------------------------------------------------------ the pattern pad (AOSP LockPatternView)
# The keyguard's pattern view is square, about 57 mm on this screen (estimate from Android's layout values), so the
# cells are 19 mm: dot pitch 19 mm, hit radius min(cell)/2 x 0.6 = 5.7 mm (lock_pattern_dot_hit_factor 0.6).
PITCH = 19.0
HIT_R = PITCH / 2 * 0.6
PAD_V = -30.0                                 # pattern centre, mm below the screen centre (estimate)
DOT_R = 14.0 * DP_MM / 2                      # lock_pattern_dot_size 14 dp
ACTIVE_R = 30.0 * DP_MM / 2                   # lock_pattern_dot_size_activated 30 dp
LINE_HW = 22.0 * DP_MM / 2                    # lock_pattern_dot_line_width 22 dp
MIN_DOTS = 4                                  # LockPatternUtils.MIN_LOCK_PATTERN_SIZE / MIN_PATTERN_REGISTER_FAIL
TRIES_BEFORE_WAIT, WAIT_S = 5, 30.0           # 5 failed attempts -> 30 s lockout
CLEAR_S = 2.0                                 # a wrong pattern stays drawn 2 s (PATTERN_CLEAR_TIMEOUT_MS)
PATTERN = (1, 2, 3, 5, 7, 8, 9)              # dots numbered 1-9 from the top left, row by row: a Z
FILL_MM = 1.0                                 # moves are checked every 1 mm along the step (panel history)


def dot_uv(n):
    """Screen point (u, v) mm of dot n (1-9)."""
    r, c = divmod(n - 1, 3)
    return np.array([(c - 1) * PITCH, PAD_V - (r - 1) * PITCH])


class PatternPad:
    """Android's pattern lock (LockPatternView + the keyguard's check), restated in screen millimetres. Touches come
    in as {id: (u, v)} once per control step; the first touch down is followed (Android reads pointer 0)."""

    def __init__(self):
        self.pattern = []            # dots of the pattern in progress or last submitted
        self.drawing = False
        self.finger = None           # (id, last point)
        self.unlocked = False
        self.unlocked_at = None
        self.attempts = []           # every submitted pattern, in order
        self.failed = 0              # counted failed attempts (4 dots or more)
        self.wait_until = -1.0
        self.wrong_until = -1.0
        self.wrong_shown = False     # the header says "Wrong pattern" (until the next touch)
        self.t = 0.0

    def _log(self, text):
        print(f"[pattern] {self.t:6.2f} s {text}", flush=True)

    def _hit(self, p):
        """detectAndAddHit: a dot within the hit radius, not used yet; a skipped middle dot is added first."""
        for n in range(1, 10):
            if n in self.pattern:
                continue
            if float(np.sum((np.asarray(p) - dot_uv(n)) ** 2)) < HIT_R ** 2:
                if self.pattern:
                    lr, lc = divmod(self.pattern[-1] - 1, 3)
                    r, c = divmod(n - 1, 3)
                    fr, fc = lr, lc
                    if abs(r - lr) == 2 and abs(c - lc) != 1:
                        fr = lr + (1 if r > lr else -1)
                    if abs(c - lc) == 2 and abs(r - lr) != 1:
                        fc = lc + (1 if c > lc else -1)
                    gap = 3 * fr + fc + 1
                    if gap not in self.pattern:
                        self.pattern.append(gap)
                        self._log(f"dot {gap} added (skipped middle dot)")
                self.pattern.append(n)
                self._log(f"dot {n} lit at ({p[0]:.1f}, {p[1]:.1f}) mm, {math.hypot(*(np.asarray(p) - dot_uv(n))):.1f} mm "
                          f"from its centre; pattern {self.pattern}")
                return n
        return None

    def update(self, t, points):
        self.t = float(t)
        if self.wrong_until >= 0 and self.t >= self.wrong_until and not self.drawing:
            self.pattern, self.wrong_until = [], -1.0
        if self.unlocked:
            return
        pts = {k: np.asarray(p, float) for k, p in points.items()}
        if self.finger is not None:
            k, last = self.finger
            if k not in pts:
                others = sorted(pts)
                if others:                       # pointer 0 lifted; the remaining touch becomes pointer 0
                    self.finger = (others[0], last)
                    k = others[0]
                else:
                    self._up()
                    return
            p = pts[k]
            n = max(1, int(math.ceil(float(np.linalg.norm(p - last)) / FILL_MM)))
            if self.drawing:
                for i in range(1, n + 1):
                    self._hit(last + (p - last) * i / n)
            self.finger = (k, p)
            return
        if pts:
            k = sorted(pts)[0]
            self._down(k, pts[k])

    def _down(self, k, p):
        self.finger = (k, p)
        if self.t < self.wait_until:
            self._log(f"touch at ({p[0]:.1f}, {p[1]:.1f}) ignored: pattern pad locked for {self.wait_until - self.t:.0f} s")
            self.drawing = False
            return
        self.pattern, self.wrong_until, self.wrong_shown = [], -1.0, False
        self.drawing = True
        self._log(f"finger down at ({p[0]:.1f}, {p[1]:.1f}) mm")
        self._hit(p)

    def _up(self):
        self.finger = None
        if not self.drawing:
            return
        self.drawing = False
        if not self.pattern:
            self._log("finger up, no dot touched")
            return
        got = list(self.pattern)
        self.attempts.append(got)
        if tuple(got) == PATTERN:
            self.unlocked, self.unlocked_at = True, self.t
            self._log(f"finger up: pattern {got} correct: UNLOCKED")
            return
        self.wrong_until, self.wrong_shown = self.t + CLEAR_S, True
        if len(got) < MIN_DOTS:
            self._log(f"finger up: pattern {got} too short (under {MIN_DOTS} dots): wrong, not counted")
            return
        self.failed += 1
        self._log(f"finger up: pattern {got} wrong (failed attempt {self.failed})")
        if self.failed % TRIES_BEFORE_WAIT == 0:
            self.wait_until = self.t + WAIT_S
            self._log(f"{TRIES_BEFORE_WAIT} failed attempts: try again in {WAIT_S:.0f} s")


def leading(seq):
    """How many dots of `seq` match the pattern from its start."""
    k = 0
    for a, b in zip(seq, PATTERN):
        if a != b:
            break
        k += 1
    return k


# ------------------------------------------------------------------ fingertips on the glass (tablet kit)
# The fingertip end faces in each finger link's frame (from the robot's collision meshes): 14 x 3.2 mm flat faces,
# 12.8 mm past the tool point, square to the approach axis.
NIB_FACE = {
    "tip_left": [(x, y, -0.0901) for x in (0.0170, 0.0310) for y in (-0.0476, -0.0444)] + [(0.0240, -0.0460, -0.0901)],
    "tip_right": [(x, y, -0.0901) for x in (-0.0310, -0.0170) for y in (0.0444, 0.0476)] + [(-0.0240, 0.0460, -0.0901)],
}
TOUCH_GAP = 0.0002                            # touching: end face on the glass
CLEAR_MM = 20.0                               # hands off: fingertips 20 mm above the glass or off the phone
TEST_NIBS = None                              # offline tests: {(arm, link): (5, 3) array of world points}


def nib_points():
    """{(arm, finger): 5 x 3 world points (4 face corners, then the centre)} from the arms' joint readings."""
    if TEST_NIBS is not None:
        return {k: np.asarray(v, float) for k, v in TEST_NIBS.items()}
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import YAM_URDF, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.urdf import forward_kinematics

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        names = list(ARM_JOINTS) + list(FINGER_JOINTS)
        ids = art.find_joints(names, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q, float)[ids]
        fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(names, q)})
        base = np.asarray(station.BASES[arm], float)[:3]
        for link, pts in NIB_FACE.items():
            m = fk[link]
            out[(arm, link)] = np.asarray(pts) @ m[:3, :3].T + m[:3, 3] + base
    return out


def _local(ph, pts):
    return (np.asarray(pts, float) - np.asarray(ph.origin, float)) @ ph.rotation()


def touches(ph, nibs):
    """Screen points {arm: (u, v)}: the touching end faces of one hand make one touch at their mean centre (the closed
    fingertips, taped together, are one contact patch for the panel)."""
    out = {}
    for arm in ("left", "right"):
        cs = []
        for k, pts in sorted((nibs or {}).items()):
            if k[0] != arm:
                continue
            loc = _local(ph, pts)
            lowest = float(loc[:4, 2].min())
            u, v = -loc[4, 1] / MM, loc[4, 0] / MM
            if GLASS_Z - 0.006 <= lowest <= GLASS_Z + TOUCH_GAP and abs(u) <= SCREEN_W / 2 and abs(v) <= SCREEN_H / 2:
                cs.append((u, v))
        if cs:
            out[arm] = tuple(np.mean(np.asarray(cs), axis=0))
    return out


def hands_off(ph, nibs):
    """Every fingertip face 20 mm or more above the glass, or 10 mm or more outside the phone's outline."""
    if not nibs:
        return True
    for pts in nibs.values():
        loc = _local(ph, pts)
        above = float(loc[:4, 2].min()) - GLASS_Z
        outside = max(float(np.abs(loc[:4, 0]).min()) - PH_L / 2, float(np.abs(loc[:4, 1]).min()) - PH_W / 2)
        if above < CLEAR_MM * MM and outside < 0.010:
            return False
    return True


def nib_at(u, v, height_mm):
    """Offline tests: the right hand closed, its two end faces side by side `height_mm` above the glass at (u, v)."""
    out = {}
    for name, side in (("tip_left", 1.0), ("tip_right", -1.0)):
        c = screen_to_world(u, v, TOP + GLASS_Z + height_mm * MM) + np.array([0.0, side * 0.0016, 0.0])
        corners = [c + np.array([dx, dy, 0.0]) for dx in (-0.007, 0.007) for dy in (-0.0016, 0.0016)]
        out[("right", name)] = np.array(corners + [c])
    return out


def screen_to_world(u, v, z):
    return np.array([PH_X + v * MM, PH_Y - u * MM, z])


# ------------------------------------------------------------------ drawing the screen
def _quad(a, b, hw):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    n = np.linalg.norm(d)
    if n < 1e-9:
        return None
    w = np.array([-d[1], d[0]]) / n * hw
    return np.array([a - w, b - w, b + w, a + w])


def _rect(u0, v0, u1, v1):
    return np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]], float)


def _disc(c, r, n=16):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


def _rounded(u0, v0, u1, v1, r, n=4):
    pts = []
    for cu, cv, a0 in ((u1 - r, v0 + r, -90), (u1 - r, v1 - r, 0), (u0 + r, v1 - r, 90), (u0 + r, v0 + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cu + r * math.cos(a), cv + r * math.sin(a)))
    return np.array(pts)


SEGS = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc",
        "8": "abcdefg", "9": "abcdfg"}


def _digit(ch, u0, v0, w, h, t):
    """Seven-segment digit with its lower left corner at (u0, v0)."""
    seg = {"a": _rect(u0, v0 + h - t, u0 + w, v0 + h), "d": _rect(u0, v0, u0 + w, v0 + t),
           "g": _rect(u0, v0 + h / 2 - t / 2, u0 + w, v0 + h / 2 + t / 2),
           "f": _rect(u0, v0 + h / 2, u0 + t, v0 + h), "b": _rect(u0 + w - t, v0 + h / 2, u0 + w, v0 + h),
           "e": _rect(u0, v0, u0 + t, v0 + h / 2), "c": _rect(u0 + w - t, v0, u0 + w, v0 + h / 2)}
    return [seg[s] for s in SEGS[ch]]


LAYERS = (  # name, sRGB colour, height above the glass (mm)
    ("wall", (0.07, 0.09, 0.16), 0.10), ("home", (0.55, 0.78, 0.86), 0.12), ("clock", (0.95, 0.95, 0.97), 0.20),
    ("dots", (0.80, 0.82, 0.88), 0.20), ("line", (0.97, 0.97, 1.00), 0.30), ("lit", (1.0, 1.0, 1.0), 0.40),
    ("red_line", (0.92, 0.20, 0.18), 0.30), ("red_lit", (0.96, 0.26, 0.22), 0.40), ("bar", (0.92, 0.20, 0.18), 0.20),
    ("icon_a", (0.93, 0.33, 0.27), 0.25), ("icon_b", (0.25, 0.55, 0.95), 0.25), ("icon_c", (0.98, 0.76, 0.18), 0.25),
    ("icon_d", (0.26, 0.70, 0.40), 0.25), ("icon_e", (0.97, 0.97, 0.97), 0.25),
)
ICON_COLOURS = ("icon_a", "icon_b", "icon_c", "icon_d", "icon_e")


def screen_polys(pad, finger=None):
    """{layer: [polygons in screen mm]}: what the phone shows for the pad's state (finger: the touch point now)."""
    polys = {name: [] for name, _, _ in LAYERS}
    full = _rounded(-SCREEN_W / 2, -SCREEN_H / 2, SCREEN_W / 2, SCREEN_H / 2, 6.0)
    if pad.unlocked:
        polys["home"].append(full)
        polys["icon_e"].append(_rounded(-26.0, -60.0, 26.0, -53.0, 3.5))        # search bar
        k = 0
        for r in range(5):
            for c in range(4):
                u, v = -22.5 + 15.0 * c, 50.0 - 17.0 * r
                polys[ICON_COLOURS[k % 4]].append(_disc((u, v), 4.8, 16))
                k += 1
        for c in range(4):                                                       # dock
            polys[ICON_COLOURS[(c + 1) % 4]].append(_disc((-22.5 + 15.0 * c, -42.0), 4.8, 16))
        for i, ch in enumerate("1009"):
            polys["clock"] += _digit(ch, -14.0 + 7.5 * i + (2.0 if i > 1 else 0.0), 60.0, 4.5, 6.0, 0.9)
        return polys
    polys["wall"].append(full)
    for i, ch in enumerate("1009"):                                              # the lock screen clock
        polys["clock"] += _digit(ch, -27.0 + 13.0 * i + (4.0 if i > 1 else 0.0), 40.0, 9.0, 18.0, 2.0)
    polys["clock"] += [_disc((0.0, 45.0), 1.1, 8), _disc((0.0, 53.0), 1.1, 8)]
    polys["clock"].append(_rounded(-3.0, 63.0, 3.0, 68.0, 0.8))                  # padlock body
    polys["clock"] += [_rect(-2.6, 67.5, -1.6, 70.5), _rect(1.6, 67.5, 2.6, 70.5), _rect(-2.6, 70.0, 2.6, 71.0)]
    wrong = pad.wrong_until >= 0 and not pad.drawing
    if pad.wrong_shown:
        polys["bar"].append(_rounded(-20.0, 4.0, 20.0, 8.0, 2.0))               # where "Wrong pattern" is written
    line, lit = ("red_line", "red_lit") if wrong else ("line", "lit")
    for n in range(1, 10):
        polys["dots"].append(_disc(dot_uv(n), DOT_R, 12))
    seq = [dot_uv(n) for n in pad.pattern]
    for a, b in zip(seq[:-1], seq[1:]):
        polys[line].append(_quad(a, b, LINE_HW))
    if pad.drawing and seq and finger is not None:
        polys[line].append(_quad(seq[-1], finger, LINE_HW))
    for p in seq:
        polys[lit].append(_disc(p, ACTIVE_R, 16))
    return {k: [p for p in v if p is not None] for k, v in polys.items()}


def _mesh_arrays(polys, height_mm):
    """USD mesh arrays in the phone's frame (screen u -> -y, v -> +x), each polygon one counter-clockwise face."""
    pts, counts, idx = [], [], []
    for p in polys:
        if np.cross(p[1] - p[0], p[2] - p[0]) < 0:
            p = p[::-1]
        base = len(pts)
        for u, v in p:
            pts.append((v * MM, -u * MM, GLASS_Z + height_mm * MM))
        counts.append(len(p))
        idx += list(range(base, base + len(p)))
    return pts, counts, idx


_SCREEN = _RUN.setdefault("patlock_screen", {"paths": [], "drawn": None})
_original_spawn = getattr(_yp._spawn_part, "_patlock_original", _yp._spawn_part)


def _spawn_part(prim_path, cfg, *args, **kwargs):
    """The kit's part spawner, plus the phone's screen: one drawn-only mesh per layer under the phone's body (no
    collider, no mass), with an emissive look, as a lit screen."""
    root = _original_spawn(prim_path, cfg, *args, **kwargs)
    if getattr(cfg, "part_name", "") != "phone":
        return root
    try:
        import isaaclab.sim as sim_utils
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        from yam_arena.station import linear_rgb

        stage = kwargs.get("stage") or sim_utils.get_current_stage()
        UsdGeom.Xform.Define(stage, f"{prim_path}/screen")
        for name, rgb, _ in LAYERS:
            path = f"{prim_path}/screen/{name}"
            mesh = UsdGeom.Mesh.Define(stage, path)
            mesh.CreateDoubleSidedAttr().Set(True)
            mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            mat = UsdShade.Material.Define(stage, f"{path}_look")
            shader = UsdShade.Shader.Define(stage, f"{path}_look/Shader")
            shader.CreateIdAttr().Set("UsdPreviewSurface")
            lin = linear_rgb(rgb)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*lin))
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(0.45 * c for c in lin)))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        _SCREEN["paths"].append(prim_path)
        _SCREEN["drawn"] = None
        draw(PatternPad(), None)
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"patlock: screen not authored: {exc!r}", flush=True)
    return root


_spawn_part._patlock_original = _original_spawn
_yp._spawn_part = _spawn_part


def draw(pad, finger):
    """Redraw the screen for the pad's state (only when it changed)."""
    f = None if finger is None or not pad.drawing else (round(float(finger[0]), 1), round(float(finger[1]), 1))
    key = (pad.unlocked, tuple(pad.pattern), pad.drawing, pad.wrong_until >= 0, pad.wrong_shown, f)
    if not _SCREEN["paths"] or _SCREEN["drawn"] == key:
        return
    import isaaclab.sim as sim_utils
    from pxr import Gf, UsdGeom, Vt

    stage = sim_utils.get_current_stage()
    polys = screen_polys(pad, finger)
    for prim_path in _SCREEN["paths"]:
        for name, _, height in LAYERS:
            pts, counts, idx = _mesh_arrays(polys[name], height)
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"{prim_path}/screen/{name}"))
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(counts))
            mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(idx))
    _SCREEN["drawn"] = key


# ------------------------------------------------------------------ the trial: touches in, screen out, every step
STEP_M = 0.0005
TEST_PAD = None           # offline tests: a PatternPad
TEST_START = None         # offline tests: (x, y, yaw_deg) of the phone at the start


def _yaw_deg(ph):
    r = ph.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _pad():
    v = _RUN.get("patlock_pad")
    if v is None or v[0] != _RUN["episode"]:
        v = (_RUN["episode"], PatternPad(), {"step": None, "touching": False, "off": True, "pose": None,
                                              "quiet": False, "start": None})
        _RUN["patlock_pad"] = v
    return v[1], v[2]


def tick(objects):
    """Once per control step: read the fingertips, run the pattern rules, redraw the screen."""
    ph = objects["phone"]
    if _RUN["t"] is None:
        nibs = nib_points()
        return TEST_PAD or PatternPad(), {"touching": bool(touches(ph, nibs)), "off": hands_off(ph, nibs),
                                          "quiet": True, "start": TEST_START or (PH_X, PH_Y, 0.0)}
    pad, rec = _pad()
    token = _RUN["step"]
    if rec["step"] == token:
        return pad, rec
    rec["step"] = token
    if rec["start"] is None:
        rec["start"] = (float(ph.origin[0]), float(ph.origin[1]), _yaw_deg(ph))
    nibs = nib_points()
    pts = touches(ph, nibs)
    pad.update(float(_RUN["t"]), pts)
    try:
        correct(pts)
    except Exception as exc:  # noqa: BLE001
        print(f"patlock: correction failed: {exc!r}", flush=True)
    rec["touching"] = bool(pts)
    rec["off"] = hands_off(ph, nibs)
    pose = np.r_[ph.to_world((0, 0, 0)), ph.to_world((0.1, 0, 0))]
    rec["quiet"] = rec["pose"] is not None and bool(np.all(np.abs(pose - rec["pose"]) < STEP_M))
    rec["pose"] = pose
    finger = pts.get(pad.finger[0]) if pad.finger is not None else None
    try:
        draw(pad, finger)
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"patlock: draw failed: {exc!r}", flush=True)
    if token % 10 == 0 or pts:
        hts = {f"{k[0][0]}{k[1][4]}": round((float(_local(ph, p)[:4, 2].min()) - GLASS_Z) / MM, 2)
               for k, p in (nibs or {}).items()}
        d = math.hypot(ph.origin[0] - rec["start"][0], ph.origin[1] - rec["start"][1]) / MM
        print(f"PLLOG {token} {float(_RUN['t']):.2f} touches={ {k: (round(float(p[0]), 1), round(float(p[1]), 1)) for k, p in pts.items()} } "
              f"nib_mm={hts} pattern={pad.pattern} unlocked={pad.unlocked} phone=({ph.origin[0]:.4f},{ph.origin[1]:.4f}) "
              f"moved_mm={d:.2f} yaw={_yaw_deg(ph) - rec['start'][2]:.2f}", flush=True)
    return pad, rec


# ------------------------------------------------------------------ scoring
MOVED_MM, TURNED_DEG = 20.0, 10.0
TIP_DEG, DROP_M = 45.0, 0.020


def made_pad(attempts=(), drawing=()):
    """Offline tests: a pad after these submitted attempts (lists of dots), then `drawing` in progress."""
    pad = PatternPad()
    for i, a in enumerate(list(attempts) + ([list(drawing)] if drawing else [])):
        pad.update(10.0 * i, {"t": dot_uv(a[0])})
        for k, (p, q) in enumerate(zip(a[:-1], a[1:])):
            pad.update(10.0 * i + 0.1 * (k + 1), {"t": dot_uv(q)})
        if i < len(attempts):
            pad.update(10.0 * i + 5.0, {})
    return pad


def status(objects):
    ph = objects["phone"]
    pad, rec = tick(objects)
    x0, y0, yaw0 = rec["start"]
    moved = math.hypot(ph.origin[0] - x0, ph.origin[1] - y0) / MM
    turned = abs((_yaw_deg(ph) - yaw0 + 180.0) % 360.0 - 180.0)
    tries = list(pad.attempts) + ([list(pad.pattern)] if pad.drawing and pad.pattern else [])
    best = max([leading(a) for a in tries] + [0])
    miss = any(tuple(a) != PATTERN for a in pad.attempts)
    return dict(unlocked=pad.unlocked, miss=miss, best=best, moved_mm=moved, turned_deg=turned,
                touching=rec["touching"], off=rec["off"], quiet=rec["quiet"])


def screen_number(st):
    """From the phone's screen: 1.0 unlocked at the first try; 0.75 unlocked after a wrong try; 0.5 not unlocked but
    an attempt began with 4 or more of the pattern's dots in order; 0.25 with 1-3; 0 none."""
    if st["unlocked"]:
        return 0.75 if st["miss"] else 1.0
    if st["best"] >= MIN_DOTS:
        return 0.5
    return 0.25 if st["best"] >= 1 else 0.0


def phone_number(st, settled_s):
    """From where the phone is: 1.0 within 20 mm and 10 degrees of its start with the hand off it (1 s); 0.75 in place
    but the hand still on it; 0.5 moved or turned further (still flat on the desk)."""
    if st["moved_mm"] > MOVED_MM or st["turned_deg"] > TURNED_DEG:
        return 0.5
    return 1.0 if settled_s + 1e-9 >= STILL_S else 0.75


def score(objects):
    ph = objects["phone"]
    st = status(objects)
    turned_over = ever("phone turned over", ph.tilt_deg() > TIP_DEG)
    fell = ever("phone off the desk", ph.origin[2] < TOP - DROP_M)
    still = float(np.linalg.norm(np.asarray(getattr(ph, "lin_vel", np.zeros(3)), float))) < 0.01
    settled = (not st["touching"]) and st["off"] and st["quiet"] and still
    seconds = held_for("hand off, phone still", settled)
    if turned_over or fell:
        return 0.0
    return min(screen_number(st), phone_number(st, seconds))


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (reads the phone's pose once)
NIB_PAST_TCP = 0.0128      # fingertip end faces past the tool point (robot collision meshes)
PRESS = 0.0035             # commanded this far below the glass: a light press (about 1.5-2 N on this soft arm)
SEG_S = 1.6                # seconds per 19 mm side of the Z
DIAG_S = 2.2               # seconds for the diagonal (27 mm)
DWELL_S = 0.8              # pause on each corner dot
# The arm is compliant and the pressed fingertip's friction holds it back: open loop it lagged up to 10 mm behind the
# commanded path. The demo watches its own fingertip (end faces from the joint readings, as the scorer reads them)
# and shifts the whole path on the phone by an integral correction, as a person corrects a finger they see.
GAIN = 0.25                # share of the remaining error corrected per control step
STEP_MAX_M = 0.001         # at most 1 mm of correction change per step
CORR_MAX_M = 0.015
PARK = {"left": np.array([0.12, 0.22, TOP + 0.13]), "right": np.array([0.10, -0.22, TOP + 0.13])}
_PHONE_AT = {}


def _phone_frame():
    """(origin, rotation) of the phone, read once per episode at the demo's first use; nominal offline."""
    ep = _RUN["episode"]
    if ep in _PHONE_AT:
        return _PHONE_AT[ep]
    env = _RUN["env"]
    frame = (np.array([PH_X, PH_Y, TOP]), np.eye(3))
    if env is not None:
        try:
            from yam_arena import scene as arena_scene
            s = arena_scene.object_states(env, ["phone"])["phone"]
            frame = (np.asarray(s.origin, float), np.asarray(s.rotation(), float))
            print(f"PHONE_READ origin=({frame[0][0]:.4f},{frame[0][1]:.4f},{frame[0][2]:.4f}) "
                  f"yaw={math.degrees(math.atan2(frame[1][1, 0], frame[1][0, 0])):.2f}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"PHONE_READ failed: {e!r}", flush=True)
    _PHONE_AT[ep] = frame
    return frame


_CTRL = {}


def _ctrl():
    c = _CTRL.get("state")
    if c is None or c["episode"] != _RUN["episode"]:
        c = {"episode": _RUN["episode"], "c": np.zeros(3), "target": None, "step": None}
        _CTRL["state"] = c
    return c


def _install_target_log():
    """Record the right arm's commanded TCP position each time the scripted plan computes it."""
    from yam_arena import demo as arena_demo
    cls = arena_demo.ArmPlan
    if getattr(cls.target, "_patlock", False):
        return
    original = cls.target

    def target(self, t):
        out = original(self, t)
        if float(self.base_pos[1]) < 0:
            c = _ctrl()
            if str(out[3]).startswith(("right: settle", "right: slide", "right: hold")):
                c["target"] = np.asarray(out[0], float) - c["c"]
            else:
                c["target"] = None
        return out

    target._patlock = True
    cls.target = target


_install_target_log()


def correct(pts):
    """Once per control step while the right fingertip touches: move the correction toward the error between the
    commanded (uncorrected) point and the touch point."""
    c = _ctrl()
    if c["step"] == _RUN["step"] or c["target"] is None or "right" not in pts:
        return
    c["step"] = _RUN["step"]
    o, r = _phone_frame()
    u, v = pts["right"]
    act = o + r @ np.array([v * MM, -u * MM, 0.0])
    err = c["target"] - act
    err[2] = 0.0
    d = GAIN * err
    n = float(np.linalg.norm(d))
    if n > STEP_MAX_M:
        d *= STEP_MAX_M / n
    c["c"] = c["c"] + d
    n = float(np.linalg.norm(c["c"]))
    if n > CORR_MAX_M:
        c["c"] *= CORR_MAX_M / n
    if DIAG and _RUN["step"] % 5 == 0:
        print(f"PLCTRL {_RUN['step']} err_mm=({err[0] / MM:.1f},{err[1] / MM:.1f}) corr_mm=({c['c'][0] / MM:.1f},"
              f"{c['c'][1] / MM:.1f})", flush=True)


class OnPhone(Waypoint):
    """A right-hand waypoint at a screen point (u, v) mm, `h` m above the glass (negative: pressed past it), placed on
    the phone where it lies when the waypoint is first played."""

    def __init__(self, duration, u, v, h, label):
        self.duration, self.finger, self.label = duration * DEMO_PACE, 0.0, label
        self.u, self.v, self.h = u, v, h

    @property
    def pos(self):
        o, r = _phone_frame()
        p = o + r @ np.array([self.v * MM, -self.u * MM, GLASS_Z + NIB_PAST_TCP + self.h]) + _ctrl()["c"]
        return tuple(float(x) for x in p)

    @property
    def yaw(self):
        r = _phone_frame()[1]
        return math.atan2(r[1, 0], r[0, 0])


def demo():
    right = [Waypoint(1.5 * DEMO_PACE, tuple(PARK["right"]), 0.0, "right: close the fingertips together", 0.0)]
    u, v = dot_uv(PATTERN[0])
    right += [OnPhone(2.0, u, v, 0.03, "right: over dot 1, closed fingertips 30 mm up"),
              OnPhone(1.0, u, v, 0.006, "right: down to 6 mm above dot 1")]
    if DEMO_WRONG == "tap":
        for n in PATTERN:
            u, v = dot_uv(n)
            right += [OnPhone(0.8, u, v, 0.006, f"right: over dot {n}"),
                      OnPhone(0.6, u, v, -PRESS, f"right: tap dot {n}"),
                      OnPhone(0.3, u, v, -PRESS, f"right: on dot {n}"),
                      OnPhone(0.5, u, v, 0.006, f"right: lift off dot {n}")]
    else:
        right += [OnPhone(0.8, u, v, -PRESS, "right: press onto dot 1"), OnPhone(0.4, u, v, -PRESS, "right: settle")]
        for a, b in zip(PATTERN[:-1], PATTERN[1:]):
            u, v = dot_uv(b)
            corner = b in (3, 7)
            right.append(OnPhone(DIAG_S if {a, b} & {5} else SEG_S, u, v, -PRESS, f"right: slide to dot {b}"))
            if corner:
                right.append(OnPhone(DWELL_S, u, v, -PRESS, f"right: hold on dot {b} (corner)"))
        right.append(OnPhone(0.4, u, v, -PRESS, "right: hold on dot 9"))
    right += [OnPhone(0.8, u, v, 0.04, "right: lift straight off"),
              Waypoint(1.5 * DEMO_PACE, tuple(PARK["right"]), 0.0, "right: away", 0.0),
              Waypoint(3.0, tuple(PARK["right"]), 0.0, "stillness tail", 0.0)]
    total = sum(w.duration for w in right)
    left = [Waypoint(total, tuple(PARK["left"]), FINGER_OPEN, "left: parked clear", 0.0)]
    return {"left": left, "right": right}


def demo_duration():
    return sum(w.duration for w in demo()["right"])
