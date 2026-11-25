"""An LG 24MR400-B desktop monitor (23.8 in, 540 x 200 x 414 mm with its stand, 2.9 kg) stands free on the desk in
front of the robot, switched on and showing a desktop picture. Its only control is the joystick button under the
middle of its bottom edge: one 9 mm nub pointing down, 89 mm above the desk, that is both a push button and a
four-way stick. The robot turns the monitor off the way the maker's manual gives: slide a level fingertip in under
the bezel, press the nub straight up once to open the main menu, then push it toward the back of the monitor (UP),
which is [Power Off]. The menu may already be open when the trial starts (the main menu, or the Settings menu on one
of its pages); then the screen shows it, and the robot must first move LEFT out of Settings as the manual says.

The joystick is modelled as its switch is built: the nub on a stem that rocks on two sprung axes (a direction
registers when the stem tips 3.3 degrees, about 0.5 N at the nub's tip) and slides on a sprung axial button (a press
registers at 0.15 mm, 2.35 N; figures of a typical four-way + centre-push switch, estimates for this model). Whatever
registers first is the event, as on the real control: a press that leans or slips sideways moves the cursor or
changes the volume instead. The task reads the joints every control step, runs the monitor's menu rules from the
manual and redraws the screen. The screen, touch-free here, is drawn as in the tablet kit (pinch-zoom-map-on-tablet-
to-fit-route); the joystick, the menu rules, the scorer and the demo are new.

Reusable part: `joystick_links()` + `JoystickReader` model any under-edge four-way + push nub (tuned numbers above).
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_joints as _yam_joints  # noqa: E402
import yam_parts as _yp  # noqa: E402
from yam_joints import Hinge, Link, Mechanism, Slide  # noqa: E402
from yam_parts import ball, box, cyl  # noqa: E402
from yam_arena import demo as _arena_demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
STILL_S = 1.0
TEST_GRIPPERS = None       # offline tests: {arm: (tcp world point, opening 0..1)}
TEST_MENU = None           # offline tests: a Menu
DIAG = False
WRONG = ""                 # GPU-only copies: "diag" the first press is a diagonal jab (all episodes);
                           # "diag4" only episode 4 jabs; "shove" a hard upward shove under the bezel after the off
START_CYCLE = ("idle", "main", "settings_page")   # start menu state by episode (1, 2, 3, 4 ...)


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
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


def grippers():
    """{arm: (TCP world position from the joint readings, opening 0..1)}."""
    if TEST_GRIPPERS is not None:
        return {k: (np.asarray(v[0], float), float(v[1])) for k, v in TEST_GRIPPERS.items()}
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
        out[arm] = (np.asarray(tcp, float), float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0)))
    return out


# ------------------------------------------------------------------ the monitor (monitor frame, metres)
# x toward the back of the monitor (away from the robot), y to the left of a person facing the screen, z up from the
# desk; the origin is on the desk straight below the joystick's axis.
TOP = TABLE_TOP[SCENE]
MM = 0.001
# LG 24MR400-B, maker's manual: with stand 540 x 200 x 414 mm (W x D x H), without stand 540 x 50.5 x 320.6 mm;
# 2.9 kg with stand; tilt -5 to 20 degrees. ESTIMATES (ruler): bottom edge 414 - 320.6 = 93.4 mm above the desk;
# joystick nub 9 mm across, domed, 4.5 mm below the bezel, its axis 12 mm behind the front face; stand neck 57 mm
# behind the face; base plate 210 x 190 x 10 mm from 10 mm behind the face.
W, H_PANEL = 0.540, 0.3206
FACE_X = -0.012                  # front face of the panel
BEZEL_Z = 0.0934                 # underside of the bottom bezel
NUB_R = 0.0045                   # nub radius (domed end, a half ball)
APEX_Z = BEZEL_Z - NUB_R         # lowest point of the nub: 88.9 mm above the desk
PIVOT_Z = BEZEL_Z + 0.006        # the switch's rocking point inside the bezel: nub tip 10.5 mm below it
MON_X, MON_Y, MON_YAW_DEG = 0.410, 0.000, -10.0   # the joystick in front of the robot, screen turned 10 deg
MON_MASS = 2.9
SCREEN_W, SCREEN_H = 527.0, 296.5     # 23.8 in 16:9 active area, mm
SCREEN_V0 = 16.0                      # active area starts 16 mm above the panel's bottom edge (estimate)
PANEL_GREY, BASE_GREY, NUB_GREY = (0.07, 0.07, 0.08), (0.10, 0.10, 0.11), (0.16, 0.16, 0.17)

# The switch (estimates from a typical 4-way + centre-push multi-control switch datasheet: direction 1.23 N at
# 0.25 mm measured 4.3 mm up the stem = 3.3 deg; push 2.35 N at 0.15 mm).
TILT_REG_DEG = 3.3
TILT_LIMIT_DEG = 5.0
TILT_K = 1.23 * 0.0043 / math.radians(TILT_REG_DEG)    # 0.092 N m/rad
PUSH_REG = 0.00015
PUSH_LIMIT = 0.0004
PUSH_K = 2.35 / PUSH_REG                               # 15,667 N/m
NEUTRAL_DEG, NEUTRAL_PUSH = 1.5, 0.00007
LONG_PRESS_S = 5.0


def joystick_links(pivot, nub_r, bezel_z, color):
    """Rocker A (tips the nub toward +x/-x: UP/DOWN), rocker B (toward +y/-y: LEFT/RIGHT), then the axial button
    with the cap: a 9 mm round nub with a domed end. Anchors at the rocking point inside the bezel."""
    p = tuple(pivot)
    stub = (box(p, (0.002, 0.002, 0.002)),)
    cap = (cyl((p[0], p[1], bezel_z + 0.0015), nub_r, 0.003),
           cyl((p[0], p[1], (bezel_z + 0.003 + p[2]) / 2), 0.0015, p[2] - bezel_z - 0.003 + 0.001),
           ball((p[0], p[1], bezel_z), nub_r))
    tilt = dict(lower=-TILT_LIMIT_DEG, upper=TILT_LIMIT_DEG, spring=TILT_K, damping=6e-4, armature=1e-6)
    return [
        Link("rock_a", stub, Hinge("y", p, **tilt), mass=0.001, color=color),
        Link("rock_b", stub, Hinge("x", p, **tilt), mass=0.001, parent="rock_a", color=color),
        Link("cap", cap, Slide("z", p, 0.0, PUSH_LIMIT, spring=PUSH_K, damping=18.0, armature=0.005),
             mass=0.002, parent="rock_b", color=color, friction=(0.3, 0.25)),
    ]


def _root_on_base():
    """Put the articulation's root on the monitor's base body. Rooted on the Xform, PhysX picks the middle link of
    the chain base - rocker A - rocker B - cap as the floating root, and the harness then places the monitor by that
    link's frame (the rocking point), 99 mm off, so it spawns into the desk and is thrown (first GPU probe). The fix
    other tasks proved on the GPU; sleeping is switched off so the light switch links always respond."""
    original = _yam_joints._spawn_mechanism
    if getattr(original, "_monoff_root", False):
        return

    def spawn(prim_path, cfg, *args, **kwargs):
        prim = original(prim_path, cfg, *args, **kwargs)
        from pxr import PhysxSchema, UsdPhysics
        mech = _yam_joints._SPECS[cfg.mech_name]
        if mech.mass is None:
            return prim
        stage = prim.GetStage()
        base = stage.GetPrimAtPath(f"{prim_path}/base")
        prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        UsdPhysics.ArticulationRootAPI.Apply(base)
        art = PhysxSchema.PhysxArticulationAPI.Apply(base)
        art.CreateEnabledSelfCollisionsAttr().Set(mech.self_collide)
        art.CreateSolverPositionIterationCountAttr().Set(32)
        art.CreateSolverVelocityIterationCountAttr().Set(4)
        art.CreateSleepThresholdAttr().Set(0.0)
        art.CreateStabilizationThresholdAttr().Set(0.0)
        return prim

    spawn._monoff_root = True
    _yam_joints._spawn_mechanism = spawn


_root_on_base()
SPAWN_LIFT = 0.002      # spawned 2 mm above the desk: a collider touching it at spawn throws a free articulation


def _monitor():
    fx = FACE_X
    shapes = [
        box((fx + 0.009, 0.0, BEZEL_Z + H_PANEL / 2), (0.018, W, H_PANEL), PANEL_GREY),           # panel
        box((0.022, 0.0, 0.255), (0.032, 0.36, 0.23), PANEL_GREY),                              # rear housing
        box((0.057, 0.0, 0.205), (0.040, 0.07, 0.060), BASE_GREY),                              # hinge block
        box((0.057, 0.0, 0.098), (0.024, 0.06, 0.176), BASE_GREY),                              # neck
        box((0.093, 0.0, 0.005), (0.190, 0.21, 0.010), BASE_GREY),                              # base plate
    ]
    yaw = math.radians(MON_YAW_DEG)
    c, s = math.cos(yaw), math.sin(yaw)
    pos = (MON_X, MON_Y, TOP + SPAWN_LIFT)
    return Mechanism("monitor", pos, shapes, joystick_links((0.0, 0.0, PIVOT_Z), NUB_R, BEZEL_Z, NUB_GREY),
                     mass=MON_MASS, color=PANEL_GREY, yaw=yaw, friction=(0.5, 0.45), contact_offset=0.001)


MONITOR = _monitor()
OBJECTS = [MONITOR]
PRESSED_PARTS = ("monitor",)           # the fingertip presses the nub; nothing is grasped


# ------------------------------------------------------------------ the monitor's menu rules (maker's manual)
INPUTS = ("HDMI", "D-SUB")
PICTURE_MODES = ("Custom", "Reader", "Photo", "Cinema", "Color Weakness", "Game")
SETTINGS_TABS = ("Quick Settings", "Input", "Picture", "General")


class Menu:
    """The 24MR400's joystick logic as the manual gives it. Screen states: "off", "idle" (picture, no menu), "main"
    (main menu: UP [Power Off], LEFT [Input], RIGHT [Settings], DOWN [Picture Mode], press [Exit]), "settings"
    (tab list), "settings_page" (one tab's page), "input_list", "picture_list". From the picture: press opens the
    main menu, LEFT/RIGHT change the volume, UP/DOWN show the input information; holding the press 5 s turns the
    monitor off. In Settings LEFT goes back up one level ("move the joystick to LEFT until you exit")."""

    def __init__(self, start="idle"):
        self.on = True
        self.screen = "idle" if start == "off" else start
        if start == "off":
            self.on, self.screen = False, "off"
        self.volume, self.input, self.picture = 30, 0, 0
        self.start_settings = (30, 0, 0)
        self.cursor = 0                  # settings tab / list highlight
        self.banner, self.banner_until = None, -1.0
        self.events = []                 # (t, event, screen before, screen after)
        self.t = 0.0

    def changed(self):
        return (self.volume, self.input, self.picture) != self.start_settings

    def _show(self, what, t):
        self.banner, self.banner_until = what, t + 3.0

    def event(self, ev, t):
        self.t = float(t)
        before = self.screen
        s = self.screen
        if not self.on:
            if ev == "press":
                self.on, self.screen = True, "idle"
        elif ev == "long":
            self.on, self.screen, self.banner = False, "off", None
        elif s == "idle":
            if ev == "press":
                self.screen, self.banner = "main", None
            elif ev in ("left", "right"):
                self.volume = int(np.clip(self.volume + (-1 if ev == "left" else 1), 0, 100))
                self._show("volume", t)
            elif ev in ("up", "down"):
                self._show("input_info", t)
        elif s == "main":
            if ev == "up":
                self.on, self.screen, self.banner = False, "off", None
            elif ev == "press":
                self.screen = "idle"
            elif ev == "left":
                self.screen, self.cursor = "input_list", self.input
            elif ev == "right":
                self.screen, self.cursor = "settings", 0
            elif ev == "down":
                self.screen, self.cursor = "picture_list", self.picture
        elif s == "settings":
            if ev == "left":
                self.screen = "idle"
            elif ev in ("right", "press"):
                self.screen = "settings_page"
            elif ev in ("up", "down"):
                self.cursor = (self.cursor + (-1 if ev == "up" else 1)) % len(SETTINGS_TABS)
        elif s == "settings_page":
            if ev in ("left", "press"):
                self.screen = "settings"
        elif s in ("input_list", "picture_list"):
            n = len(INPUTS) if s == "input_list" else len(PICTURE_MODES)
            if ev == "left":
                self.screen = "main"
            elif ev in ("up", "down"):
                self.cursor = (self.cursor + (-1 if ev == "up" else 1)) % n
            elif ev == "press":
                if s == "input_list":
                    self.input = self.cursor
                else:
                    self.picture = self.cursor
                self.screen = "idle"
        self.events.append((round(self.t, 2), ev, before, self.screen))
        print(f"[joystick] {self.t:6.2f} s {ev}: {before} -> {self.screen} (volume {self.volume}, input "
              f"{INPUTS[self.input]}, picture {PICTURE_MODES[self.picture]})", flush=True)


class JoystickReader:
    """Turns the nub's joint readings into events, first come first served: a direction when a rocker passes
    3.3 deg, a press when the button passes 0.15 mm; nothing more until the nub is back near its centre. A short
    press acts when it is let go; held 5 s it is the long press."""

    def __init__(self):
        self.armed = True
        self.pressed_at = None
        self.long_sent = False
        self.peak = {"tilt_deg": 0.0, "push_mm": 0.0}
        self.at_register = []            # (event, tilt_a_deg, tilt_b_deg, push_mm)

    def update(self, t, ta, tb, push, menu):
        a, b = math.degrees(ta), math.degrees(tb)
        self.peak["tilt_deg"] = max(self.peak["tilt_deg"], abs(a), abs(b))
        self.peak["push_mm"] = max(self.peak["push_mm"], push / MM)
        neutral = abs(a) < NEUTRAL_DEG and abs(b) < NEUTRAL_DEG and push < NEUTRAL_PUSH
        if self.pressed_at is not None:
            if push >= NEUTRAL_PUSH:
                if not self.long_sent and t - self.pressed_at >= LONG_PRESS_S:
                    self.long_sent = True
                    menu.event("long", t)
                return
            if not self.long_sent:
                menu.event("press", t)
            self.pressed_at, self.long_sent = None, False
        if not self.armed:
            self.armed = neutral
            return
        if push >= PUSH_REG:
            self.armed, self.pressed_at = False, t
            self.at_register.append(("press", round(a, 2), round(b, 2), round(push / MM, 3)))
            print(f"[joystick] {t:6.2f} s button down (tilt A {a:+.2f} deg, B {b:+.2f} deg, push "
                  f"{push / MM:.3f} mm)", flush=True)
            return
        if max(abs(a), abs(b)) >= TILT_REG_DEG:
            # rocker A positive tips the nub's tip toward -x (toward the user): DOWN; rocker B positive tips it +y:
            # LEFT (as seen by a person facing the screen)
            ev = ("down" if a > 0 else "up") if abs(a) >= abs(b) else ("left" if b > 0 else "right")
            self.armed = False
            self.at_register.append((ev, round(a, 2), round(b, 2), round(push / MM, 3)))
            menu.event(ev, t)


# ------------------------------------------------------------------ drawing the screen
def _rect(u0, v0, u1, v1):
    return np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]], float)


def _disc(c, r, n=24):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


def _ring(c, r0, r1, n=24):
    out = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        out.append(np.array([[c[0] + r0 * math.cos(a0), c[1] + r0 * math.sin(a0)],
                             [c[0] + r1 * math.cos(a0), c[1] + r1 * math.sin(a0)],
                             [c[0] + r1 * math.cos(a1), c[1] + r1 * math.sin(a1)],
                             [c[0] + r0 * math.cos(a1), c[1] + r0 * math.sin(a1)]]))
    return out


SEGS = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc",
        "8": "abcdefg", "9": "abcdfg"}


def _digit(ch, u0, v0, w, h, t):
    seg = {"a": _rect(u0, v0 + h - t, u0 + w, v0 + h), "d": _rect(u0, v0, u0 + w, v0 + t),
           "g": _rect(u0, v0 + h / 2 - t / 2, u0 + w, v0 + h / 2 + t / 2),
           "f": _rect(u0, v0 + h / 2, u0 + t, v0 + h), "b": _rect(u0 + w - t, v0 + h / 2, u0 + w, v0 + h),
           "e": _rect(u0, v0, u0 + t, v0 + h / 2), "c": _rect(u0 + w - t, v0, u0 + w, v0 + h / 2)}
    return [seg[s] for s in SEGS[ch]]


LAYERS = (  # name, sRGB colour, height in front of the panel face (mm)
    ("wall", (0.16, 0.38, 0.66), 0.10), ("wall2", (0.36, 0.62, 0.84), 0.15), ("window", (0.95, 0.95, 0.96), 0.20),
    ("titlebar", (0.22, 0.24, 0.30), 0.25), ("text", (0.55, 0.57, 0.62), 0.30), ("taskbar", (0.10, 0.11, 0.14), 0.20),
    ("osd", (0.06, 0.06, 0.07), 0.40), ("osd_line", (0.55, 0.55, 0.58), 0.45), ("osd_hi", (0.80, 0.10, 0.30), 0.50),
    ("osd_icon", (0.92, 0.92, 0.94), 0.55), ("bar_fill", (0.92, 0.92, 0.94), 0.55), ("black", (0.005, 0.005, 0.006), 0.70),
)
MENU_C = (0.0, 62.0)          # main menu centre on the screen (u, v mm from the screen's bottom centre)


def _icon_power(c, r):
    return _ring(c, r * 0.62, r, 16)[2:15] + [_rect(c[0] - r * 0.12, c[1] + r * 0.1, c[0] + r * 0.12, c[1] + r * 1.05)]


def screen_polys(menu):
    """{layer: [polygons]} in screen mm, u to the right of a person facing it, v up from the active area's bottom."""
    P = {name: [] for name, _, _ in LAYERS}
    hw = SCREEN_W / 2
    if not menu.on:
        P["black"].append(_rect(-hw, 0.0, hw, SCREEN_H))
        return P
    P["wall"].append(_rect(-hw, 0.0, hw, SCREEN_H))
    P["wall2"] += [np.array([[-hw, 120.0], [hw, 60.0], [hw, 120.0], [-hw, 200.0]])]
    P["window"].append(_rect(-170.0, 70.0, 120.0, 250.0))
    P["titlebar"].append(_rect(-170.0, 238.0, 120.0, 250.0))
    for i in range(7):
        P["text"].append(_rect(-155.0, 215.0 - 19.0 * i, -155.0 + (220.0 if i % 3 else 150.0), 222.0 - 19.0 * i))
    P["taskbar"].append(_rect(-hw, 0.0, hw, 14.0))
    s = menu.screen
    if s == "main":
        c = MENU_C
        P["osd"].append(_disc(c, 42.0, 32))
        P["osd_line"] += _ring(c, 13.0, 14.5, 24)
        P["osd_icon"] += _icon_power((c[0], c[1] + 28.0), 6.0)                                          # UP
        P["osd_icon"] += [_rect(c[0] - 34.0, c[1] - 4.0, c[0] - 22.0, c[1] + 4.0)]                       # LEFT input
        P["osd_icon"] += _ring((c[0] + 28.0, c[1]), 3.0, 6.0, 12)                                         # RIGHT gear
        P["osd_icon"] += [_rect(c[0] - 6.0, c[1] - 33.0, c[0] + 6.0, c[1] - 24.0)]                       # DOWN picture
        P["osd_icon"] += [np.array([[c[0] - 5.0, c[1] - 4.0], [c[0] - 3.5, c[1] - 5.5], [c[0] + 5.0, c[1] + 4.0],
                                    [c[0] + 3.5, c[1] + 5.5]]),
                          np.array([[c[0] + 3.5, c[1] - 5.5], [c[0] + 5.0, c[1] - 4.0], [c[0] - 3.5, c[1] + 5.5],
                                    [c[0] - 5.0, c[1] + 4.0]])]                                         # centre X
    elif s in ("settings", "settings_page"):
        P["osd"].append(_rect(-60.0, 30.0, 200.0, 170.0))
        for i in range(len(SETTINGS_TABS)):
            v = 150.0 - 30.0 * i
            (P["osd_hi"] if i == menu.cursor else P["osd_line"]).append(_rect(-52.0, v - 10.0, 20.0, v + 10.0))
        if s == "settings_page":
            P["osd_line"].append(_rect(28.0, 38.0, 30.0, 162.0))
            for i in range(4):
                v = 148.0 - 28.0 * i
                P["osd_icon"].append(_rect(40.0, v - 3.0, 120.0, v + 3.0))
                P["bar_fill"].append(_rect(130.0, v - 2.0, 190.0, v + 2.0))
    elif s in ("input_list", "picture_list"):
        n = len(INPUTS) if s == "input_list" else len(PICTURE_MODES)
        P["osd"].append(_rect(-140.0, 30.0, -50.0, 40.0 + 22.0 * n))
        for i in range(n):
            v = 40.0 + 22.0 * (n - 1 - i) + 6.0
            (P["osd_hi"] if i == menu.cursor else P["osd_line"]).append(_rect(-132.0, v - 7.0, -58.0, v + 7.0))
    if menu.banner == "volume" and menu.t <= menu.banner_until:
        P["osd"].append(_rect(-120.0, 40.0, 120.0, 70.0))
        P["osd_line"].append(_rect(-80.0, 52.0, 80.0, 58.0))
        P["bar_fill"].append(_rect(-80.0, 52.0, -80.0 + 1.6 * menu.volume + 0.01, 58.0))
        for i, ch in enumerate(f"{menu.volume:02d}"):
            P["osd_icon"] += _digit(ch, 88.0 + 9.0 * i, 47.0, 6.0, 16.0, 1.6)
    elif menu.banner == "input_info" and menu.t <= menu.banner_until:
        P["osd"].append(_rect(-hw + 20.0, SCREEN_H - 60.0, -hw + 140.0, SCREEN_H - 20.0))
        P["osd_icon"].append(_rect(-hw + 30.0, SCREEN_H - 45.0, -hw + 60.0, SCREEN_H - 35.0))
    return P


def _mesh_arrays(polys, height_mm):
    """Mesh arrays in the monitor frame: screen u -> -y, v -> z, drawn just in front of the panel face."""
    pts, counts, idx = [], [], []
    for p in polys:
        p = np.asarray(p, float)
        if np.cross(p[1] - p[0], p[2] - p[0]) < 0:
            p = p[::-1]
        base = len(pts)
        for u, v in p:
            pts.append((FACE_X - height_mm * MM, -u * MM, BEZEL_Z + (SCREEN_V0 + v) * MM))
        counts.append(len(p))
        idx += list(range(base, base + len(p)))
    return pts, counts, idx


_SCREEN = _RUN.setdefault("monoff_screen", {"paths": [], "drawn": None})
_original_spawn = getattr(_yp._spawn_part, "_monoff_original", _yp._spawn_part)


def _spawn_part(prim_path, cfg, *args, **kwargs):
    """The kit's spawner, plus the monitor's screen: one drawn-only mesh per layer under the monitor's base body (no
    collider, no mass), lit, as an emitting panel."""
    root = _original_spawn(prim_path, cfg, *args, **kwargs)
    name = str(getattr(cfg, "part_name", ""))
    if not (name.endswith("/base") and "monitor" in name):
        return root
    try:
        import isaaclab.sim as sim_utils
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        from yam_arena.station import linear_rgb

        stage = kwargs.get("stage") or sim_utils.get_current_stage()
        UsdGeom.Xform.Define(stage, f"{prim_path}/screen")
        for lname, rgb, _ in LAYERS:
            path = f"{prim_path}/screen/{lname}"
            mesh = UsdGeom.Mesh.Define(stage, path)
            mesh.CreateDoubleSidedAttr().Set(True)
            mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            mat = UsdShade.Material.Define(stage, f"{path}_look")
            shader = UsdShade.Shader.Define(stage, f"{path}_look/Shader")
            shader.CreateIdAttr().Set("UsdPreviewSurface")
            lin = linear_rgb(rgb)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*lin))
            glow = 0.0 if lname == "black" else 0.5
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(glow * c for c in lin)))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3 if lname != "black" else 0.15)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        _SCREEN["paths"].append(prim_path)
        _SCREEN["drawn"] = None
        draw(Menu(START_CYCLE[0]))
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"monoff: screen not authored: {exc!r}", flush=True)
    return root


_spawn_part._monoff_original = _original_spawn
_yp._spawn_part = _spawn_part


def draw(menu):
    """Redraw the screen for the menu's state (only when it changed)."""
    banner = menu.banner if menu.t <= menu.banner_until else None
    key = (menu.on, menu.screen, menu.cursor, banner, menu.volume)
    if not _SCREEN["paths"] or _SCREEN["drawn"] == key:
        return
    import isaaclab.sim as sim_utils
    from pxr import Gf, UsdGeom, Vt

    stage = sim_utils.get_current_stage()
    polys = screen_polys(menu)
    for prim_path in _SCREEN["paths"]:
        for lname, _, height in LAYERS:
            pts, counts, idx = _mesh_arrays(polys[lname], height)
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"{prim_path}/screen/{lname}"))
            if not mesh:
                continue
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(counts))
            mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(idx))
    _SCREEN["drawn"] = key


# ------------------------------------------------------------------ the trial: joints in, screen out, every step
def start_state(episode=None):
    ep = _RUN["episode"] if episode is None else episode
    return START_CYCLE[(max(ep, 1) - 1) % len(START_CYCLE)]


def _yaw_deg(st):
    r = st.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _rec():
    v = _RUN.get("monoff_rec")
    if v is None or v["episode"] != _RUN["episode"]:
        v = {"episode": _RUN["episode"], "menu": Menu(start_state()), "reader": JoystickReader(), "step": None,
             "start": None, "pose": None, "quiet": False, "away": True}
        _RUN["monoff_rec"] = v
    return v


def hands_away(st, g):
    """Every fingertip out from under the monitor: each TCP 40 mm or more in front of the panel face (its fingertip
    end 27 mm or more), or 60 mm or more from the joystick."""
    if not g:
        return True
    nub = st.to_world((0.0, 0.0, APEX_Z))
    for tcp, _ in g.values():
        loc = st.to_local(tcp)
        if loc[0] > FACE_X - 0.040 and float(np.linalg.norm(np.asarray(tcp) - nub)) < 0.060 + 0.0128:
            return False
    return True


def tick(objects):
    """Once per control step: read the nub's joints, run the menu rules, redraw the screen."""
    st = objects["monitor"]
    if _RUN["t"] is None:
        return (TEST_MENU or Menu("idle")), {"away": hands_away(st, grippers()), "quiet": True,
                                             "start": (MON_X, MON_Y, MON_YAW_DEG)}
    rec = _rec()
    menu = rec["menu"]
    if rec["step"] == _RUN["step"]:
        return menu, rec
    rec["step"] = _RUN["step"]
    t = float(_RUN["t"])
    if rec["start"] is None:
        rec["start"] = (float(st.origin[0]), float(st.origin[1]), _yaw_deg(st))
        print(f"MONOFF start state {menu.screen}, monitor at ({st.origin[0]:.4f}, {st.origin[1]:.4f}) yaw "
              f"{_yaw_deg(st):.2f}", flush=True)
    j = st.joints
    rec["reader"].update(t, j["rock_a"], j["rock_b"], j["cap"], menu)
    menu.t = t
    g = grippers()
    rec["away"] = hands_away(st, g)
    pose = np.r_[st.to_world((0, 0, 0)), st.to_world((0.1, 0, 0))]
    rec["quiet"] = rec["pose"] is not None and bool(np.all(np.abs(pose - rec["pose"]) < 0.0005))
    rec["pose"] = pose
    try:
        draw(menu)
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"monoff: draw failed: {exc!r}", flush=True)
    if DIAG and (rec["step"] % 5 == 0 or abs(j["rock_a"]) > 0.02 or abs(j["rock_b"]) > 0.02 or j["cap"] > 3e-5):
        d = math.hypot(st.origin[0] - rec["start"][0], st.origin[1] - rec["start"][1]) / MM
        tcp = g["right"][0] if g else np.zeros(3)
        loc = st.to_local(tcp) if g else np.zeros(3)
        print(f"MOLOG {rec['step']} {t:.2f} tiltA={math.degrees(j['rock_a']):+.2f} tiltB={math.degrees(j['rock_b']):+.2f} "
              f"push_mm={j['cap'] / MM:.3f} screen={menu.screen} on={menu.on} vol={menu.volume} "
              f"tcp_mon_mm=({loc[0] / MM:.1f},{loc[1] / MM:.1f},{loc[2] / MM:.1f}) moved_mm={d:.2f} "
              f"yaw={_yaw_deg(st) - rec['start'][2]:.2f} tilt={st.tilt_deg():.2f} away={rec['away']}", flush=True)
    return menu, rec


# ------------------------------------------------------------------ scoring
MOVE_OK_MM, TURN_OK_DEG = 10.0, 3.0
MOVE_MAX_MM, TURN_MAX_DEG = 25.0, 8.0
TOPPLE_DEG = 30.0


def made_menu(start="idle", events=()):
    """Offline tests: the menu after these joystick events from this start state."""
    m = Menu(start)
    for e in events:
        m.event(e, 0.0)
    return m


def status(objects):
    st = objects["monitor"]
    menu, rec = tick(objects)
    x0, y0, yaw0 = rec["start"]
    moved = math.hypot(st.origin[0] - x0, st.origin[1] - y0) / MM
    turned = abs((_yaw_deg(st) - yaw0 + 180.0) % 360.0 - 180.0)
    return dict(on=menu.on, changed=menu.changed(), moved_mm=moved, turned_deg=turned, tilt_deg=st.tilt_deg(),
                away=rec["away"], quiet=rec["quiet"])


def score(objects):
    """0 if the screen is still on (any menu, any picture) or the monitor toppled (tilted over 30 degrees);
    otherwise the lowest of: 1.0; 0.5 if the volume, input or picture mode differs from the start; 0.25 if the
    monitor moved more than 25 mm or turned more than 8 degrees; 0.75 if it moved more than 10 mm or turned more than
    3 degrees; 0.75 until the hand is out from under the monitor for 1 s with the monitor still."""
    st = objects["monitor"]
    s = status(objects)
    toppled = ever("monitor toppled", s["tilt_deg"] > TOPPLE_DEG)
    still = float(np.linalg.norm(np.asarray(getattr(st, "lin_vel", np.zeros(3)), float))) < 0.01
    settled = held_for("hand away, monitor still", s["away"] and s["quiet"] and still)
    if toppled or s["on"]:
        return 0.0
    n = 1.0
    if s["changed"]:
        n = min(n, 0.5)
    if s["moved_mm"] > MOVE_MAX_MM or s["turned_deg"] > TURN_MAX_DEG:
        n = min(n, 0.25)
    elif s["moved_mm"] > MOVE_OK_MM or s["turned_deg"] > TURN_OK_DEG:
        n = min(n, 0.75)
    if settled + 1e-9 < STILL_S:
        n = min(n, 0.75)
    return n


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo: a level fingertip, closed loop
# The right hand comes in level, pointing toward the back of the monitor, fingers closed and stacked (pads closing
# vertically): the upper finger's back is a 15 mm wide flat face 4 mm above the tool point over the last 13 mm.
# Positions are in the joystick frame N: origin at the nub's lowest point, a toward the back of the monitor, l to
# the left of a person facing the screen, z up. Tool point (TCP) offsets in mm.
TOP_FACE = 4.0          # upper finger's back above the TCP (tip region)
END_FACE = 12.8         # fingertip end past the TCP
SIDE = 7.7              # fingertip half width
DOME_AT2 = math.sqrt(NUB_R ** 2 - (NUB_R - 0.002) ** 2) / MM   # nub radius 2 mm above its lowest point: 3.74 mm
UNDER = (-20.5, 0.0, -7.0)                      # parked under the bezel, finger top 3 mm below the nub
ENTRY = (-62.0, 0.0, -7.0)                      # in front of the face, at the same height
APPROACH = (-110.0, 0.0, 60.0)                  # where the hand turns level, in front of the screen
PRESS_PRE = (-10.5, 0.0, -7.0)                  # under the nub, over the middle of the flat fingertip (10.5 mm past the TCP)
UP_PRE = (-(DOME_AT2 + END_FACE + 4.0), 0.0, -1.5)    # end face 4 mm short of the nub's front
LEFT_PRE = (-10.0, -(DOME_AT2 + SIDE + 4.0), -1.5)    # side face 4 mm short of the nub's right side
RIGHT_PRE = (-10.0, DOME_AT2 + SIDE + 4.0, -1.5)
SPEED = 0.035           # m/s for moves between poses
ACT_SPEED = 0.005       # m/s while pressing or pushing
PRESS_MAX = 20.0        # mm of rise past the PRESS_PRE height: the level arm is soft (about 200 N/m up)
PUSH_MAX = 15.0         # mm of travel past the pre pose for a push (up to about 11 mm past contact)
DT = 0.05
T_TOTAL = 40.0
PARK = {"left": np.array([0.12, 0.22, TOP + 0.13]), "right": np.array([0.16, -0.20, TOP + 0.16])}
GAIN, STEP_MAX, CORR_MAX = 0.3, 0.0008, 0.015
HOLD_GAIN, HOLD_STEP = 0.1, 0.0002      # gentle: at 0.4 the hold rang +-1.5 mm on the soft arm (GPU)


def _rot_level(R):
    """TCP rotation: approach along the monitor's +x (horizontal), the TCP x axis (pads' closing axis) up."""
    d = np.array([R[0, 0], R[1, 0], 0.0])
    d /= np.linalg.norm(d)
    up = np.array([0.0, 0.0, 1.0])
    return np.c_[up, np.cross(d, up), d]


LEVEL_CODE = 10.0       # a waypoint yaw of 10 + heading (rad) asks for the level hand pointing along that heading


def level_code(R):
    """Waypoint yaw code for the level hand pointing along the monitor's +x (horizontal heading)."""
    return LEVEL_CODE + math.atan2(R[1, 0], R[0, 0])


_orig_grip = getattr(_arena_demo.grip_rotation, "_monoff_original", _arena_demo.grip_rotation)


def _grip_rotation(yaw=0.0):
    """The arena's straight-down grip, plus the level hand: yaw codes 10 +- pi give it (the plan stays floats)."""
    y = float(yaw)
    if y > LEVEL_CODE - 4.0:
        h = y - LEVEL_CODE
        return _rot_level(np.array([[math.cos(h), -math.sin(h), 0.0], [math.sin(h), math.cos(h), 0.0], [0, 0, 1.0]]))
    return _orig_grip(yaw)


_grip_rotation._monoff_original = _orig_grip
_arena_demo.grip_rotation = _grip_rotation


def _monitor_frame():
    """(nub lowest point in the world, monitor rotation) read now; nominal offline."""
    env = _RUN["env"]
    yaw = math.radians(MON_YAW_DEG)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    o = np.array([MON_X, MON_Y, TOP])
    if env is not None:
        try:
            from yam_arena import scene as arena_scene
            st = arena_scene.object_states(env, ["monitor"])["monitor"]
            o, R = np.asarray(st.origin, float), np.asarray(st.rotation(), float)
        except Exception as e:  # noqa: BLE001
            print(f"MONITOR_READ failed: {e!r}", flush=True)
    return o + R @ np.array([0.0, 0.0, APEX_Z]), R


class Driver:
    """Plays the right hand one 0.05 s waypoint at a time. Between actions it reads the screen (the menu) and picks
    the next: picture -> press; main menu -> UP; Settings, a list -> LEFT; dark -> take the hand out."""

    def __init__(self, episode):
        self.episode = episode
        self.queue = []                  # segments: (goal N mm, speed m/s, stop condition or None, label)
        self.pos = None                  # current commanded point, N mm
        self.frame = None
        self.c = np.zeros(3)
        self.cache = []
        self.phase = "start"
        self.offline_menu = Menu(start_state(episode)) if _RUN["env"] is None else None
        self.actions = 0
        self.wrong_done = False
        self.label = ""
        self.hold = 0
        self._seg_start = None

    # -- geometry
    def world(self, n):
        o, R = self.frame
        a, l = np.array([R[0, 0], R[1, 0], 0.0]), np.array([R[0, 1], R[1, 1], 0.0])
        a, l = a / np.linalg.norm(a), l / np.linalg.norm(l)
        return o + (a * n[0] + l * n[1] + np.array([0.0, 0.0, 1.0]) * n[2]) * MM

    def menu(self):
        if self.offline_menu is not None:
            return self.offline_menu
        return _rec()["menu"]

    def reader(self):
        return None if self.offline_menu is not None else _rec()["reader"]

    # -- planning
    def go(self, goal, speed=SPEED, until=None, label="", dwell=0.4):
        self.queue.append((np.asarray(goal, float), speed, until, label))
        if until is None and dwell > 0:
            self.queue.append((np.asarray(goal, float), dwell, "dwell", label + " (settle)"))

    def wrong_now(self):
        if self.wrong_done:
            return False
        return WRONG == "diag" or (WRONG == "diag4" and self.episode == 4)

    def plan_next(self):
        m = self.menu()
        if self.phase == "start":
            self.frame = _monitor_frame()
            self.go(ENTRY, 0.08, label="right: to the front of the monitor, level, under the screen")
            self.go(UNDER, SPEED, label="right: slide in under the bezel")
            self.phase = "act"
            return
        if self.phase == "out":
            self.queue.append((None, 0.0, None, "right: hold away"))
            return
        if not m.on or self.actions >= 6:
            if WRONG == "shove" and not self.wrong_done:
                self.wrong_done = True
                shove = (-14.0, 40.0, -7.0)
                self.go(UNDER, label="right: (wrong way) under the bezel")
                self.go(shove, label="right: (wrong way) under the bezel 40 mm to the left of the nub")
                self.go((shove[0], shove[1], shove[2] + 32.0), 0.03, label="right: (wrong way) shove up hard")
                self.go(shove, label="right: (wrong way) back down")
                self.go(UNDER, label="right: (wrong way) back under")
                return
            self.go(UNDER, label="right: back under the bezel")
            self.go(ENTRY, label="right: slide out from under the monitor")
            self.go(APPROACH, 0.06, label="right: away from the monitor", dwell=0.0)
            self.phase = "out"
            return
        self.actions += 1
        self.frame = _monitor_frame()
        s = m.screen
        if s == "idle":
            if self.wrong_now():
                self.wrong_done = True
                self.go(PRESS_PRE, label="right: under the nub (wrong way: a diagonal jab)")
                tgt = np.array(PRESS_PRE) + np.array([0.0, 7.0, PRESS_MAX])
                self.go(tgt, ACT_SPEED * 1.4, until="any", label="right: jab up and sideways")
                self.go(PRESS_PRE, SPEED, until="neutral", label="right: back down")
                self.go(UNDER, label="right: back under")
                return
            self.go(PRESS_PRE, label="right: under the nub")
            self.go(np.array(PRESS_PRE) + np.array([0.0, 0.0, PRESS_MAX]), ACT_SPEED, until="any",
                    label="right: press the nub straight up")
            self.go(PRESS_PRE, SPEED, until="neutral", label="right: let go")
            self.go(UNDER, label="right: back under")
        elif s == "main":
            self.go(UP_PRE, label="right: behind the nub's front, finger top 2 mm up the nub")
            self.go(np.array(UP_PRE) + np.array([PUSH_MAX, 0.0, 0.0]), ACT_SPEED, until="any",
                    label="right: push the nub toward the back (UP: Power Off)")
            self.go(UP_PRE, SPEED, until="neutral", label="right: let go")
            self.go(UNDER, label="right: back under")
        else:   # Settings, a page, a list: LEFT backs out (manual: "move the joystick to LEFT until you exit")
            self.go((LEFT_PRE[0], LEFT_PRE[1], UNDER[2]), label="right: beside the nub, low")
            self.go(LEFT_PRE, label="right: beside the nub, finger top 2 mm up the nub")
            self.go(np.array(LEFT_PRE) + np.array([0.0, PUSH_MAX, 0.0]), ACT_SPEED, until="any",
                    label="right: push the nub to the left (LEFT: back)")
            self.go(LEFT_PRE, SPEED, until="neutral", label="right: let go")
            self.go((LEFT_PRE[0], LEFT_PRE[1], UNDER[2]), label="right: down")
            self.go(UNDER, label="right: back under")

    def _event_count(self):
        if self.offline_menu is not None:
            return len(self.offline_menu.events)
        return len(_rec()["menu"].events) + len(_rec()["reader"].at_register)

    def _stop(self, until, start_count):
        if until == "any":
            if self.offline_menu is not None:
                return False
            r = self.reader()
            return len(r.at_register) > start_count[1] or not r.armed
        if until == "neutral":
            if self.offline_menu is not None:
                return True
            r = self.reader()
            return r.pressed_at is None and r.armed
        return True

    def _offline_event(self, label):
        m = self.offline_menu
        if m is None:
            return
        if "press the nub" in label:
            m.event("press", 0.0)
        elif "UP: Power Off" in label:
            m.event("up", 0.0)
        elif "LEFT: back" in label:
            m.event("left", 0.0)

    def step(self):
        """Next commanded TCP (world) and its label."""
        guard = 0
        while not self.queue and guard < 3:
            self.plan_next()
            guard += 1
        goal, speed, until, label = self.queue[0]
        if goal is None:                                  # parked: stay
            return self.world(np.array(APPROACH, float)), label, None
        if isinstance(goal, str):                         # to the park point (world)
            if self.pos_w is None:
                self.pos_w = self.world(self.pos)
            d = PARK["right"] - self.pos_w
            n = float(np.linalg.norm(d))
            stepm = speed * DT
            self.pos_w = PARK["right"].copy() if n <= stepm else self.pos_w + d / n * stepm
            if n <= stepm:
                self.queue.pop(0)
            return self.pos_w.copy(), label, None
        if self.pos is None:
            self.pos = np.array(APPROACH, float)
        if until == "dwell":
            if self._seg_start is None:
                self._seg_start = speed
            self._seg_start -= DT
            if self._seg_start <= 1e-9:
                self.queue.pop(0)
                self._seg_start = None
            return self.world(self.pos), label, True
        if self._seg_start is None:
            r = self.reader()
            self._seg_start = (len(self.menu().events), len(r.at_register) if r else 0)
        d = goal - self.pos
        n = float(np.linalg.norm(d))
        stepmm = speed * DT / MM
        arrived = n <= stepmm
        self.pos = goal.copy() if arrived else self.pos + d / n * stepmm
        done = False
        mode = False
        if until == "any":
            # hold the two axes the action does not drive where the hand was when it started (proprioception)
            o, R = self.frame
            ax = [np.array([R[0, 0], R[1, 0], 0.0]), np.array([R[0, 1], R[1, 1], 0.0]), np.array([0.0, 0.0, 1.0])]
            ax = [v / np.linalg.norm(v) for v in ax]
            drive = int(np.argmax(np.abs(d))) if n > 1e-9 else 2
            if self.hold_ref is None:
                g = grippers()
                self.hold_ref = g["right"][0].copy() if g else None
                self.hold_c0 = self.c.copy()
            self.hold_axes = [ax[i] for i in range(3) if i != drive]
            mode = "free" if "jab" in label else "hold"
            done = self._stop("any", self._seg_start) or arrived
            if arrived and self.offline_menu is not None:
                self._offline_event(label)
        elif until == "neutral":
            done = arrived and self._stop("neutral", self._seg_start)
        else:
            done = arrived
        if done:
            if self.hold_ref is not None and self.hold_c0 is not None:
                self.c = self.hold_c0      # the hold only offsets the contact force: drop it once the action ends
            self.hold_ref, self.hold_c0 = None, None
            if until == "any":
                self.queue.pop(0)
                # stop where it is: let go from here
            else:
                self.queue.pop(0)
            self._seg_start = None
        return self.world(self.pos), label, mode

    pos_w = None
    hold_ref = None
    hold_c0 = None
    hold_axes = ()


_DRIVERS = {}


def _driver():
    ep = _RUN["episode"]
    d = _DRIVERS.get(ep)
    if d is None:
        _DRIVERS.clear()
        d = Driver(ep)
        _DRIVERS[ep] = d
    return d


def _correct(target, free):
    """While the hand moves freely, shift the plan by the error between where it was sent and where the joint
    readings put it (the arm sags under its own weight when reaching level and low)."""
    drv = _driver()
    if free is None or free == "free" or _RUN["env"] is None:
        return
    g = grippers()
    if not g or drv.last_target is None:
        return
    err = drv.last_target - g["right"][0]
    if free == "hold" and drv.hold_ref is not None:
        # pressing or pushing: keep the hand where it started along the two axes the action does not drive, so the
        # soft arm neither drifts sideways on the nub nor is pushed off it (only the joint readings are used)
        e = drv.hold_ref - g["right"][0]
        dc = sum((HOLD_GAIN * float(e @ v)) * v for v in drv.hold_axes)
        n = float(np.linalg.norm(dc))
        if n > HOLD_STEP:
            dc *= HOLD_STEP / n
        drv.c = drv.c + dc
        n = float(np.linalg.norm(drv.c))
        if n > CORR_MAX:
            drv.c *= CORR_MAX / n
    elif free is True and np.linalg.norm(err) < 0.03:   # only while settling on a pose
        dc = GAIN * err
        n = float(np.linalg.norm(dc))
        if n > STEP_MAX:
            dc *= STEP_MAX / n
        drv.c = drv.c + dc
        n = float(np.linalg.norm(drv.c))
        if n > CORR_MAX:
            drv.c *= CORR_MAX / n
    if DIAG and _RUN["step"] % 5 == 0:
        print(f"MOCTRL {_RUN['step']} err_mm=({err[0] / MM:.1f},{err[1] / MM:.1f},{err[2] / MM:.1f}) corr_mm="
              f"({drv.c[0] / MM:.1f},{drv.c[1] / MM:.1f},{drv.c[2] / MM:.1f}) {drv.label}", flush=True)


Driver.last_target = None


class Live(Waypoint):
    """One 0.05 s right-hand waypoint, computed when first played from the screen and the hand's own readings."""

    def __init__(self, k):
        self.k, self.duration, self.finger = k, DT, 0.0
        self._pos, self._rot, self._label = None, None, ""

    def _compute(self):
        drv = _driver()
        if self._pos is None:
            target, label, free = drv.step()
            _correct(target, free)
            drv.last_target = np.asarray(target, float)
            drv.label = label
            self._pos = tuple(float(x) for x in np.asarray(target) + drv.c)
            self._rot = level_code(drv.frame[1] if drv.frame is not None else np.eye(3))
            self._label = label
            drv.cache.append(self)

    @property
    def pos(self):
        self._compute()
        return self._pos

    @property
    def yaw(self):
        self._compute()
        return self._rot

    @property
    def label(self):
        self._compute()
        return self._label


def _approach_point():
    yaw = math.radians(MON_YAW_DEG)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    o = np.array([MON_X, MON_Y, TOP]) + R @ np.array([0.0, 0.0, APEX_Z])
    a, l = R[:, 0], R[:, 1]
    return o + (a * APPROACH[0] + l * APPROACH[1] + np.array([0.0, 0.0, 1.0]) * APPROACH[2]) * MM, level_code(R)


TURN_S = 3.0


def demo():
    n = int(round(T_TOTAL / DT))
    waypoints = [Live(k) for k in range(n)]
    p, rot = _approach_point()
    right = [Waypoint(TURN_S, tuple(p), 0.0, "right: close the fingers, turn the hand level in front of the screen",
                      rot)] + waypoints
    total = TURN_S + n * DT
    left = [Waypoint(total, tuple(PARK["left"]), FINGER_OPEN, "left: parked clear", 0.0)]
    return {"left": left, "right": right}


def demo_duration():
    return TURN_S + T_TOTAL
