"""Proving a stick-mounted voltage detector before and after testing a bare bar. A red proximity voltage
detector (28 x 7.6 x 5 cm, 0.31 kg) on a 61.5 cm orange fibreglass hot stick (0.95 kg together) lies level on two
grey posts, 13 cm above the table, its sensing (top) end pointing to the robot's right. To the right of that end a
black proof tester lies on a block, its metal end plate facing the detector 2 cm away and a red TEST button on
top. Beyond the tester a bare aluminium bar (9.35 mm, 18 cm long) runs away from the robot on two posts.
The detector has a white ON button (press: 3 s self-test, then on) and two red lights on its face; they flash
while its top end is on the tester's end plate and TEST is held, unless the tester's battery is flat (odd seeds).
Zero object jitter; 20 Hz.
"""
import collections
import math
import pathlib
import sys
import zlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Lamp, Link, Mechanism, Slide
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
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

# ------------------------------------------------------------------ layout (metres; z above the table top)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

X0 = 0.130                      # the proving line: detector axis and tester centre line
Y0 = -0.075                     # detector part origin (the body-to-head joint is 5 cm left of it)
AXIS_Z = 0.130                  # detector axis, tester centre and bar axis above the table
RED, ORANGE, HEADC = (0.80, 0.08, 0.08), (0.95, 0.55, 0.10), (0.30, 0.30, 0.32)
GREYC, ALU, BLACK, WHITE = (0.45, 0.45, 0.47), (0.82, 0.83, 0.85), (0.07, 0.07, 0.07), (0.95, 0.95, 0.95)

# Detector on its stick. Part frame: origin on the stick axis, y along the stick (top end toward -y), z out of
# the face. HD Electric PRX: 11 x 3 x 2 in, 11 oz. Stick: 1-1/4 in fibreglass, 61.5 cm, universal spline head.
BODY_L, BODY_W, BODY_T = 0.280, 0.076, 0.050
M_DETECTOR = 0.31
M_STICK = 0.64                  # pole 0.40 + head 0.20 + end cap 0.04 kg (estimate)
M_BUTTON = 0.002
TIP = np.array([0.0, -0.230, 0.0])           # centre of the top (sensing) end face
HEAD_L, HEAD_R, POLE_L, POLE_R = 0.055, 0.020, 0.615, 0.016
BODY = box((0.0, 0.05 - BODY_L / 2, 0.0), (BODY_W, BODY_L, BODY_T), RED)
HEAD = cyl((0.0, 0.05 + HEAD_L / 2, 0.0), HEAD_R, HEAD_L, axis="y", color=HEADC)
_vh, _vp = math.pi * HEAD_R ** 2 * HEAD_L, math.pi * POLE_R ** 2 * POLE_L
STICK_COM = (_vh * (0.05 + HEAD_L / 2) + _vp * (0.105 + POLE_L / 2)) / (_vh + _vp)
BAL = (M_DETECTOR * (0.05 - BODY_L / 2) + M_STICK * STICK_COM) / (M_DETECTOR + M_STICK)   # balance point (y)
FACE = BODY_T / 2
GRIP_HALF = 0.040               # the pole is authored as three abutting cylinders; the middle one is where it balances
_p0, _p1, _p2, _p3 = 0.105, BAL - GRIP_HALF, BAL + GRIP_HALF, 0.105 + POLE_L
POLE = [cyl((0.0, (a + b) / 2, 0.0), POLE_R, b - a, axis="y", color=ORANGE) for a, b in ((_p1, _p2), (_p0, _p1), (_p2, _p3))]
TIP_Y = Y0 + TIP[1]             # world y of the top end at the start
ON_Y, ON_R, ON_PROUD, ON_TRAVEL = -0.054, 0.007, 0.002, 0.0015
ON_K, ON_REST = 300.0, 0.006                 # 1.8 N at rest, 2.25 N bottomed
RED_LIGHTS = ((-0.018, -0.208), (0.018, -0.208))   # 22 mm from the top end (estimate)
RED_R = 0.005                                # 10 mm lenses (estimate)
GREEN_Y = tuple(-0.160 + 0.010 * i for i in range(7))
LAMP_OFF = (0.16, 0.03, 0.03)
LAMP_RED = (1.0, 0.10, 0.05)
GREEN_OFF, GREEN_ON = (0.05, 0.14, 0.06), (0.15, 0.95, 0.25)
BLINK_STEPS = 4                 # the red lights flash 4 control steps on, 4 off (2.5 Hz)


def _sim():
    return _LIVE["sim"]


def _detector_lamps(state):
    s = _sim()
    blink = (s["step"] // BLINK_STEPS) % 2 == 0
    red = bool((s["selftest"] or s["flashing"]) and blink)
    out = {"red_a": red, "red_b": red}
    for i in range(7):
        out[f"range_{i}"] = bool((s["selftest"] and int(s["selftest_frac"] * 7) == i)
                                 or (s["on"] and not s["selftest"] and i == 0))
    return out


def _tester_lamps(state):
    return {"led": bool(_sim()["tester_led"])}


DETECTOR = Mechanism(
    "detector", (X0, Y0, TOP + AXIS_Z), [BODY],
    [Link("stick", [HEAD] + POLE, Slide("y", (0.0, 0.05, 0.0), -2e-5, 2e-5, damping=200.0, spring=2e5), mass=M_STICK),
     Link("on_button", [cyl((0.0, ON_Y, FACE - 0.001 + (ON_PROUD + 0.001) / 2), ON_R, ON_PROUD + 0.001, color=WHITE)],
          Slide("z", (0.0, ON_Y, FACE), -ON_TRAVEL, 0.0, damping=0.5, spring=ON_K, rest=ON_REST),
          mass=M_BUTTON, color=WHITE)],
    mass=M_DETECTOR - M_BUTTON, color=RED, friction=(0.8, 0.6), contact_offset=0.001, grip=2,
    lamps=tuple(Lamp(n, (x, y, FACE - 0.003), RED_R, on=LAMP_RED, off=LAMP_OFF)
                for n, (x, y) in zip(("red_a", "red_b"), RED_LIGHTS))
    + tuple(Lamp(f"range_{i}", (0.0, y, FACE - 0.0012), 0.0025, on=GREEN_ON, off=GREEN_OFF)
            for i, y in enumerate(GREEN_Y)),
    lamp_logic=_detector_lamps)

# Proof tester (HD Electric PT-PRX, 4.9 x 2.6 x 1.3 in, 199 g) lying face up on a block, bolted down.
PLATE_Y = -0.325                              # outer face of the metal end plate (world y)
T_L, T_W, T_T = 0.125, 0.066, 0.033
TESTER_Y = PLATE_Y - T_L / 2
TEST_DY, TEST_R, TEST_PROUD, TEST_TRAVEL = 0.0225, 0.007, 0.002, 0.002
TEST_Y = TESTER_Y + TEST_DY
TEST_K, TEST_REST = 400.0, 0.005              # 2.0 N at rest, 2.8 N bottomed
TESTER = Mechanism(
    "tester", (X0, TESTER_Y, TOP + AXIS_Z),
    [box((0.0, -0.0005, 0.0), (T_W, T_L - 0.001, T_T), BLACK),
     box((0.0, T_L / 2 - 0.0005, 0.0), (0.050, 0.001, 0.025), ALU),                       # metal end plate
     box((0.0, 0.0, -(AXIS_Z + T_T / 2) / 2), (T_W, T_L, AXIS_Z - T_T / 2), GREYC)],      # support block
    [Link("test_button", [cyl((0.0, TEST_DY, T_T / 2 - 0.001 + (TEST_PROUD + 0.001) / 2), TEST_R,
                              TEST_PROUD + 0.001, color=RED)],
          Slide("z", (0.0, TEST_DY, T_T / 2), -TEST_TRAVEL, 0.0, damping=0.5, spring=TEST_K, rest=TEST_REST),
          mass=0.002, color=RED)],
    mass=None, color=BLACK, friction=(0.6, 0.5), contact_offset=0.001,
    lamps=(Lamp("led", (0.018, -0.020, T_T / 2 - 0.001), 0.0025, on=GREEN_ON, off=GREEN_OFF),),
    lamp_logic=_tester_lamps)

# Cradle posts and the bar on its two posts (all bolted down).
POST_BODY_Y, POST_STICK_Y = -0.165, 0.425
POST_A = Part("post_body", (X0, POST_BODY_Y, TOP),
              [box((0, 0, (AXIS_Z - FACE) / 2), (0.090, 0.030, AXIS_Z - FACE))], None, GREYC)
POST_B = Part("post_stick", (X0, POST_STICK_Y, TOP),
              [box((0, 0, (AXIS_Z - POLE_R) / 2), (0.060, 0.030, AXIS_Z - POLE_R)),
               box((-0.024, 0, AXIS_Z - POLE_R + 0.006), (0.006, 0.030, 0.012)),
               box((0.024, 0, AXIS_Z - POLE_R + 0.006), (0.006, 0.030, 0.012))], None, GREYC)
BAR_Y, BAR_R, BAR_X0, BAR_X1 = -0.345, 0.004675, 0.200, 0.380
BAR_POSTS = (0.220, 0.360)
BAR = Part("bar", ((BAR_X0 + BAR_X1) / 2, BAR_Y, TOP),
           [cyl((0, 0, AXIS_Z), BAR_R, BAR_X1 - BAR_X0, axis="x", color=ALU)]
           + [box((px - (BAR_X0 + BAR_X1) / 2, 0, (AXIS_Z - BAR_R) / 2), (0.022, 0.022, AXIS_Z - BAR_R))
              for px in BAR_POSTS],
           None, GREYC)
OBJECTS = [DETECTOR, TESTER, POST_A, POST_B, BAR]
PRESSED_PARTS = ("detector", "tester")


# ------------------------------------------------------------------ live state shared across imports
def _new_live():
    return {"episode": -1, "seed_raw": None, "flat": False,
            "sim": {"step": 0, "on": False, "selftest": False, "selftest_frac": 0.0, "flashing": False,
                    "tester_led": False},
            "rec": {}, "token": None, "on_t": None, "btn_down": False, "hist": collections.deque(), "logged": None,
            "obs": {"frames": collections.deque(maxlen=BLINK_STEPS + 1), "loc": None, "flash_seen": False,
                    "win": {}}}


_SEED_TABLE = {}


def seed_is_flat(reset_seed):
    """Odd --seed: the tester's battery is flat. The harness hands reset() crc32('<seed>:None:0'), not the seed."""
    if reset_seed is None:
        return False
    if not _SEED_TABLE:
        _SEED_TABLE[zlib.crc32(b"None:None:0") & 0xFFFFFFFF] = 0
        for k in range(4096):
            _SEED_TABLE[zlib.crc32(f"{k}:None:0".encode()) & 0xFFFFFFFF] = k
    return _SEED_TABLE.get(int(reset_seed), int(reset_seed)) % 2 == 1


def _install():
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return _new_live()
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_pvd_live"):
        return cls._pvd_live
    live = _new_live()
    original_reset, original_obs = cls.reset, cls._observation

    def reset(self, scene, *args, **kwargs):
        fresh = _new_live()
        fresh["episode"] = live["episode"] + 1
        fresh["seed_raw"] = kwargs.get("seed")
        fresh["flat"] = seed_is_flat(kwargs.get("seed"))
        live.update(fresh)
        print(f"[detector] reset seed={kwargs.get('seed')}: tester battery {'FLAT' if live['flat'] else 'good'}",
              flush=True)
        return original_reset(self, scene, *args, **kwargs)

    def observation(self):
        obs = original_obs(self)
        try:
            _watch(obs)
        except Exception as exc:  # the camera reading must never break a rollout
            print(f"[detector] camera reading failed: {exc!r}", flush=True)
        return obs

    cls._pvd_live = live
    cls.reset, cls._observation = reset, observation
    return live


TEST_FLAT = None                # offline tests: True = the tester's battery is flat
TEST_RECORD = None              # offline tests: the episode record, e.g. {"selftest": True, "proved": True}
RECORD_KEYS = ("selftest", "proved", "touched", "reproved", "attempt", "bad_touch", "dropped")
LOG_EVENTS = True


def battery_flat():
    return bool(_LIVE["flat"]) if TEST_FLAT is None else bool(TEST_FLAT)


# ------------------------------------------------------------------ geometry the scorer measures
TOUCH_GAP = 0.003               # "touching": the top end within 3 mm of the surface
PLATE_HALF = 0.025              # top-end centre within 25 mm of the plate centre, across and in height
CLEAR_BAR = 0.080               # finish with the top end at least 8 cm from the bar
OFF_PLATE = 0.010               # and at least 1 cm off the tester's end plate
SELFTEST_S = 3.0
FLASH_S = 1.0
TOUCH_S = 1.0
ATTEMPT_S = 1.5
ON_PRESSED, ON_RELEASED = -0.0010, -0.0004
TEST_PRESSED = -0.0014
DROP_Z = TOP - 0.020
STILL_MM = 0.002                # at rest: top end and stick end each move under 2 mm over 1 s
_FACE_PTS = np.array([(x, TIP[1], z) for x in np.linspace(-BODY_W / 2, BODY_W / 2, 7)
                      for z in np.linspace(-BODY_T / 2, BODY_T / 2, 5)])
STICK_END = np.array([0.0, 0.105 + POLE_L, 0.0])
PLATE_C = np.array([X0, PLATE_Y, TOP + AXIS_Z])


def bar_gap(det):
    """Smallest distance from the detector's top end face to the bar's surface."""
    R, o = det.rotation(), np.asarray(det.origin, float)
    P = o + _FACE_PTS @ R.T
    x = np.clip(P[:, 0], BAR_X0, BAR_X1)
    d = np.sqrt((P[:, 0] - x) ** 2 + (P[:, 1] - BAR_Y) ** 2 + (P[:, 2] - (TOP + AXIS_Z)) ** 2) - BAR_R
    return float(d.min())


def plate_gap(det):
    """(gap from the top end face to the plate, offset of its centre across the plate)."""
    c = det.to_world(TIP)
    return float(c[1] - PLATE_Y), float(max(abs(c[0] - PLATE_C[0]), abs(c[2] - PLATE_C[2])))


def on_plate(det):
    gap, off = plate_gap(det)
    return bool(-0.006 <= gap <= TOUCH_GAP and off <= PLATE_HALF)


def _still(det):
    """Top end and stick end within STILL_MM of where they were, over the last STILL_S (true offline)."""
    t = _RUN["t"]
    if t is None:
        return True
    hist = _LIVE["hist"]
    pts = np.concatenate([det.to_world(TIP), det.to_world(STICK_END)])
    if not hist or hist[-1][0] != t:
        hist.append((t, pts))
    while hist and hist[0][0] < t - STILL_S - 1e-6:
        hist.popleft()
    if t - hist[0][0] < STILL_S - 0.051:
        return False
    arr = np.array([p for _, p in hist])
    return bool((arr.max(0) - arr.min(0)).max() <= STILL_MM)


def _advance(objects):
    """The instruments' rules, advanced once per control step from button travel and poses only."""
    det, tst = objects["detector"], objects["tester"]
    if _RUN["t"] is None:
        rec = dict.fromkeys(RECORD_KEYS, False)
        rec.update(TEST_RECORD or {})
        return rec
    live, sim = _LIVE, _LIVE["sim"]
    token = (_RUN["step"], float(_RUN["t"]), _RUN["episode"])
    if not live["rec"]:
        live["rec"] = dict.fromkeys(RECORD_KEYS, False)
    rec = live["rec"]
    if token == live["token"]:
        return rec
    live["token"] = token
    t = float(_RUN["t"])
    sim["step"] += 1
    q_on, q_test = float(det.joints["on_button"]), float(tst.joints["test_button"])
    if not live["btn_down"] and q_on <= ON_PRESSED:           # a press of ON toggles the unit
        live["btn_down"] = True
        sim["on"] = not sim["on"]
        live["on_t"] = t if sim["on"] else None
    elif live["btn_down"] and q_on >= ON_RELEASED:
        live["btn_down"] = False
    since = (t - live["on_t"]) if sim["on"] else 0.0
    sim["selftest"] = bool(sim["on"] and since < SELFTEST_S)
    sim["selftest_frac"] = min(since / SELFTEST_S, 0.999)
    ready = bool(sim["on"] and since >= SELFTEST_S)
    held = q_test <= TEST_PRESSED
    flat = battery_flat()
    sim["tester_led"] = bool(held and not flat)
    contact = on_plate(det)
    flashing = bool(ready and held and contact and not flat)
    sim["flashing"] = flashing
    rec["selftest"] = rec["selftest"] or ready
    was_touched = rec["touched"]
    rec["proved"] = rec["proved"] or held_for("flash", flashing) + 1e-9 >= FLASH_S
    gap = bar_gap(det)
    touching = gap <= TOUCH_GAP
    trusted = bool(rec["proved"] and sim["on"])
    rec["bad_touch"] = rec["bad_touch"] or bool(touching and not trusted)
    rec["touched"] = rec["touched"] or held_for("touch", touching and trusted) + 1e-9 >= TOUCH_S
    rec["reproved"] = rec["reproved"] or held_for("flash again", flashing and was_touched) + 1e-9 >= FLASH_S
    rec["attempt"] = rec["attempt"] or held_for("attempt", ready and held and contact) + 1e-9 >= ATTEMPT_S
    rec["dropped"] = rec["dropped"] or bool(det.low[2] < DROP_Z)
    if LOG_EVENTS:
        state = (sim["on"], sim["selftest"], flashing, held, contact, touching) + tuple(rec[k] for k in RECORD_KEYS)
        if state != live["logged"]:
            live["logged"] = state
            pg, off = plate_gap(det)
            print(f"[detector] t={t:.2f} on={sim['on']} selftest={sim['selftest']} flashing={flashing} "
                  f"test_held={held} on_plate={contact} (gap {pg*1000:.1f}, off {off*1000:.1f} mm) "
                  f"bar_gap={gap*1000:.1f} mm q_on={q_on*1000:.2f} q_test={q_test*1000:.2f} | "
                  + " ".join(k for k in RECORD_KEYS if rec[k]), flush=True)
    return rec


def score(objects):
    det = objects["detector"]
    rec = _advance(objects)
    gap = bar_gap(det)
    flat = battery_flat()
    down = bool(released(det) and _still(det) and det.low[2] >= DROP_Z)
    away = bool(gap >= CLEAR_BAR and not on_plate(det) and plate_gap(det)[0] >= OFF_PLATE)
    away_s = held_for("backed off", away)
    done_s = held_for("set down clear", away and down)
    if rec["bad_touch"] or rec["dropped"] or det.low[2] < DROP_Z:
        return 0.0
    if flat:
        if rec["attempt"] and done_s + 1e-9 >= STILL_S:
            return 1.0
        if rec["attempt"] and away_s + 1e-9 >= STILL_S:
            return 0.6
        if rec["attempt"]:
            return 0.3
        return 0.1 if rec["selftest"] else 0.0
    if rec["reproved"] and done_s + 1e-9 >= STILL_S:
        return 1.0
    if rec["reproved"]:
        return 0.8
    if rec["touched"]:
        return 0.6
    if rec["proved"]:
        return 0.2
    return 0.1 if rec["selftest"] else 0.0


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ what the demo sees (camera images only)
# The scripted demo never reads simulator state. It keeps the camera frames the policy is given
# (YamArenaEmbodiment._observation -> Observation.images) and takes two things from them:
#  1. where the detector and the TEST button are: red blobs in the top camera's first frame, compared with a
#     reference frame taken with everything at its drawing position (REF_PX, M_PER_PX);
#  2. whether the two red lights come on while TEST is held: lit-lens pixels (bright pinkish red) counted in
#     the right wrist camera's frames while both arms hold still.
REF_PX = {"detector": (190.55, 120.36), "button": None}   # blob centres in the reference frame (320 x 176)
M_PER_PX = (0.0043, 0.0060)     # table metres per pixel along image u (-y) and v (-x) near the detector
FLASH_CAMERA = "right"          # the camera whose frames decide ("right" wrist camera or "top")
FLASH_DELTA = 40                # redness change (0..255) between frames BLINK_STEPS apart (logged only)
LIT_PIXELS = 15                 # lit-lens pixels in one frame that count as "the red lights are on"
FLASH_FRAMES = 6                # frames with the lights on needed in the watch window
DEMO_BRANCH = None              # offline replay: "touch" or "clear"; None = decide from the cameras
_WINDOWS = {}


def _redness(img):
    a = img.astype(np.int16)
    return a[..., 0] - np.maximum(a[..., 1], a[..., 2])


def _lit(img):
    """Pixels that look like a lit red lens: bright red washed toward pink (the unlit lens and the body are darker)."""
    a = img.astype(np.int16)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return int(((r > 200) & (g > 70) & (g < 0.62 * r) & (b < 0.62 * r)).sum())


def _locate(top):
    import cv2
    a = top.astype(np.int16)
    mask = ((a[..., 0] > 110) & (a[..., 1] < 0.45 * a[..., 0]) & (a[..., 2] < 0.45 * a[..., 0])).astype(np.uint8)
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = sorted(((int(stats[i, cv2.CC_STAT_AREA]), round(float(cent[i][0]), 2), round(float(cent[i][1]), 2))
                    for i in range(1, n)), reverse=True)
    out = {"detector": None, "button": None, "blobs": blobs[:6]}
    if blobs and blobs[0][0] >= 60:
        out["detector"] = blobs[0][1:]
        right = [b for b in blobs[1:] if 2 <= b[0] <= 60 and b[1] > blobs[0][1] and abs(b[2] - blobs[0][2]) < 25]
        if right:
            out["button"] = min(right, key=lambda b: b[1])[1:]
    return out


def _offset(name):
    """Table offset (dx, dy, 0) of a located object from its drawing position; zero until the reference is set."""
    loc = _LIVE["obs"]["loc"]
    if not loc or REF_PX is None or loc.get(name) is None or REF_PX.get(name) is None:
        return np.zeros(3)
    du, dv = loc[name][0] - REF_PX[name][0], loc[name][1] - REF_PX[name][1]
    return np.array([float(np.clip(-dv * M_PER_PX[1], -0.03, 0.03)),
                     float(np.clip(-du * M_PER_PX[0], -0.03, 0.03)), 0.0])


def _watch(obs):
    o = _LIVE["obs"]
    t = float(obs.state_time)
    images = {("top" if "top" in k else "right" if "right" in k else "left"): v for k, v in obs.images.items()}
    if o["loc"] is None and "top" in images:
        o["loc"] = _locate(images["top"])
        print(f"[demo-eyes] top camera {images['top'].shape}: detector at {o['loc']['detector']}, TEST button at "
              f"{o['loc']['button']} px; blobs {o['loc']['blobs']}; offsets detector {_offset('detector')[:2]} "
              f"button {_offset('button')[:2]}", flush=True)
    frames = o["frames"]
    frames.append({k: _redness(v) for k, v in images.items()})
    lit = {k: _lit(v) for k, v in images.items()}
    for name, (t0, t1) in _WINDOWS.items():
        w = o["win"].setdefault(name, {"n": 0, "hits": {}, "peak": {}, "lit": {}, "done": False})
        if w["done"] or t < t0:
            continue
        if t <= t1 and len(frames) == frames.maxlen:
            w["n"] += 1
            for cam in frames[-1]:
                changed = int((np.abs(frames[-1][cam] - frames[0][cam]) > FLASH_DELTA).sum())
                w["peak"][cam] = max(w["peak"].get(cam, 0), changed)
                w["lit"][cam] = max(w["lit"].get(cam, 0), lit[cam])
                w["hits"][cam] = w["hits"].get(cam, 0) + int(lit[cam] >= LIT_PIXELS)
        if t > t1:
            w["done"] = True
            seen = w["hits"].get(FLASH_CAMERA, 0) >= FLASH_FRAMES
            if name == "first":
                o["flash_seen"] = bool(seen)
            print(f"[demo-eyes] watch '{name}' t={t0:.2f}-{t1:.2f}: frames {w['n']}, frames with the lights on {w['hits']}, "
                  f"most lit pixels {w['lit']}, most blink-changed pixels {w['peak']} -> {'FLASH seen' if seen else 'no flash'} ({FLASH_CAMERA} camera)"
                  + (f"; decision: {'touch the bar' if seen else 'keep clear'}" if name == "first" else ""),
                  flush=True)


def _touch_branch():
    if DEMO_BRANCH is not None:
        return DEMO_BRANCH == "touch"
    return bool(_LIVE["obs"]["flash_seen"])


_LIVE = _install()

# ------------------------------------------------------------------ scripted demo
DEMO_PACE = 1.0
YAW_L, YAW_ON, YAW_TEST = -math.pi / 2, 0.0, math.pi / 2
PAD = 0.013                      # pad tips below the gripper's centre point
LIFT = 0.020                     # commanded lift; the loaded arm rides about 10 mm low
AWAY = 0.060                     # both seeds: laid back on its posts 6 cm further left (top end 10 cm from the bar)
TOUCH_X = 0.282                  # commanded; the top end lands about 8 mm further forward


class _Live:
    """A waypoint whose target is read when the arm plan asks for it (after the cameras have been read)."""

    def __init__(self, duration, label, fn):
        self.duration, self.label, self._fn = float(duration), label, fn

    pos = property(lambda self: tuple(float(v) for v in self._fn()[0]))
    finger = property(lambda self: self._fn()[1])
    yaw = property(lambda self: self._fn()[2])


def _z(h):
    return np.array([0.0, 0.0, TOP + h])


def _poses():
    d, b = _offset("detector"), _offset("button")
    grip = np.array([X0, Y0 + BAL, 0.0]) + d
    on = np.array([X0, Y0 + ON_Y, 0.0]) + d
    test = np.array([X0, TEST_Y, 0.0]) + b
    plate_dy = (PLATE_Y - TIP_Y) + b[1] - d[1] - 0.003          # top end onto the plate, 3 mm past contact
    away = np.array([0.0, AWAY, 0.0])
    side = np.array([0.090, 0.0, 0.0])
    return {
        "on_hover": on + _z(0.195), "on_press": on + _z(AXIS_Z + FACE + ON_PROUD + PAD - ON_TRAVEL - 0.006),
        "test_hover": test + _z(0.190),
        "test_press": test + _z(AXIS_Z + T_T / 2 + TEST_PROUD + PAD - TEST_TRAVEL - 0.006),
        "grip_hover": grip + _z(0.185), "grip": grip + _z(AXIS_Z), "lift": grip + _z(AXIS_Z + LIFT),
        "plate": grip + np.array([0.0, plate_dy, 0.0]) + _z(AXIS_Z + LIFT),
        "pre": np.array([TOUCH_X, grip[1], 0.0]) + _z(AXIS_Z + 0.030),
        "touch": np.array([TOUCH_X, grip[1] + (BAR_Y + BAR_R - TIP_Y) - 0.002, 0.0]) + _z(AXIS_Z + 0.025),
        "away": grip + away + _z(AXIS_Z + LIFT), "away_down": grip + away + _z(AXIS_Z + 0.004),
        "away_up": grip + away + _z(0.175), "away_side": grip + away + side + _z(0.175),
    }


def demo():
    O, C = FINGER_OPEN, FINGER_CLOSED
    plan = {"left": [], "right": []}
    clock = {"t": 0.0}

    def row(dt, label, right, left):
        """right/left: (pose, finger, yaw), or a pair of them: (if a flash was seen, if not)."""
        for arm, spec in (("right", right), ("left", left)):
            a, b = (spec, spec) if isinstance(spec[0], str) else spec

            def fn(a=a, b=b):
                name, finger, yaw = a if _touch_branch() else b
                return _poses()[name], finger, yaw
            plan[arm].append(_Live(dt * DEMO_PACE, label, fn))
        clock["t"] += dt * DEMO_PACE
        return clock["t"]

    def R(name):
        return (name, C, YAW_ON if name.startswith("on") else YAW_TEST)

    def L(name, f):
        return (name, f, YAW_L)

    row(2.5, "right over ON, left over the stick's balance point", R("on_hover"), L("grip_hover", O))
    row(1.0, "press ON", R("on_press"), L("grip_hover", O))
    row(0.4, "hold", R("on_press"), L("grip_hover", O))
    row(0.8, "release ON: the self-test runs", R("on_hover"), L("grip_hover", O))
    row(1.5, "left down to the stick; right to the tester", R("test_hover"), L("grip", O))
    row(1.0, "pinch the stick at its balance point", R("test_hover"), L("grip", C))
    row(1.5, "lift the detector level", R("test_hover"), L("lift", C))
    row(1.5, "top end onto the tester's end plate", R("test_hover"), L("plate", C))
    row(0.5, "settle", R("test_hover"), L("plate", C))
    t0 = row(1.0, "press TEST", R("test_press"), L("plate", C))
    t1 = row(3.0, "hold TEST and watch the two red lights", R("test_press"), L("plate", C))
    _WINDOWS["first"] = (t0 + 0.5 * DEMO_PACE, t1 - 0.25 * DEMO_PACE)
    row(0.8, "release TEST", R("test_hover"), L("plate", C))
    row(1.0, "back off the plate", R("test_hover"), L("lift", C))
    # from here: (flash seen: test the bar, prove again, put it back | no flash: put it straight back)
    row(3.0, "to the bar | hold off the plate", R("test_hover"), (L("pre", C), L("away", C)))
    row(1.5, "top end to the bar's side | lower", R("test_hover"), (L("touch", C), L("away_down", C)))
    row(2.0, "hold on the bar: no lights | let go", R("test_hover"), (L("touch", C), L("away_down", O)))
    row(1.5, "back off the bar | rise", R("test_hover"), (L("pre", C), L("away_up", O)))
    row(3.0, "back to the tester | move clear", R("test_hover"), (L("lift", C), L("away_side", O)))
    row(1.5, "top end onto the plate again | wait", R("test_hover"), (L("plate", C), L("away_side", O)))
    row(0.5, "settle", R("test_hover"), (L("plate", C), L("away_side", O)))
    t0 = row(1.0, "press TEST again | wait", (R("test_press"), R("test_hover")), (L("plate", C), L("away_side", O)))
    t1 = row(2.5, "hold TEST: the lights flash again | wait", (R("test_press"), R("test_hover")),
             (L("plate", C), L("away_side", O)))
    _WINDOWS["second"] = (t0 + 0.5 * DEMO_PACE, t1 - 0.25 * DEMO_PACE)
    row(0.8, "release TEST", R("test_hover"), (L("plate", C), L("away_side", O)))
    row(1.0, "back off the plate", R("test_hover"), (L("lift", C), L("away_side", O)))
    row(2.0, "slide it 6 cm left, away from the bar", R("test_hover"), (L("away", C), L("away_side", O)))
    row(1.5, "lower onto the posts", R("test_hover"), (L("away_down", C), L("away_side", O)))
    row(1.0, "let go", R("test_hover"), (L("away_down", O), L("away_side", O)))
    row(1.2, "rise", R("test_hover"), (L("away_up", O), L("away_side", O)))
    row(1.5, "move clear", R("test_hover"), (L("away_side", O), L("away_side", O)))
    row(3.5, "stillness tail", R("test_hover"), (L("away_side", O), L("away_side", O)))
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
