"""A light-up tabletop whack-a-mole game: five sprung mole heads in the deck of the game unit (four round a middle one),
and the game's plastic mallet lying on the desk on the robot's right. The robot picks up the mallet and plays one
round: the unit lights one mole at a time at random, and the robot bops each lit mole down with the mallet's head
before its light goes out, without bopping moles that are not lit and without touching a mole with its fingers.

The unit is an articulated mechanism (yam_joints): a free base on rubber feet and five slide joints, each mole held up
against its top stop by a preloaded spring. The game's electronics (which mole is lit, when, and what a press counts
as) are the state machine in whack_kit.py; a press registers only when a mole is physically pushed past its switch.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import whack_kit as wk  # noqa: E402
from yam_joints import Lamp, Link, Mechanism, Slide  # noqa: E402
from yam_parts import Part, ball, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TEST_GRIPPERS = None       # offline tests: {arm: [world points of the gripper]}
TEST_GAME = None           # offline tests: (hits, wrong bops, finger presses) as the game counted them
DIAG = False
DEMO_MODE = ""             # GPU-only copies: "blind" bops every mole in turn without looking, "late" waits too long,
                           # "bare" leaves the mallet and presses lit moles with the closed fingers, "seq" the probe
DEMO_SEQ = ("", "", "", "late", "bare")


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
_EVER = {}


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def ever(key, cond):
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ the game unit (x forward, away from the robot)
TOP = TABLE_TOP[SCENE]
MM = 0.001
# Unit 10.5 x 10.5 x 4 in (267 x 267 x 102 mm, retail listing), 1.3 lb (0.59 kg) with its moles. Deck height, mole
# layout and sizes, travel and spring are estimates.
UX, UY = 0.210, -0.100                    # unit origin: middle of its underside, on the desk
SIDE = 0.267
DECK = 0.070                              # deck top above the desk
PITCH = 0.075                             # corner moles this far from the middle one along x and y
MOLES = {"m0": (0.0, 0.0), "m1": (PITCH, PITCH), "m2": (PITCH, -PITCH), "m3": (-PITCH, PITCH), "m4": (-PITCH, -PITCH)}
DOME_R = 0.0185                           # mole's hard-hat dome (collider); lamp sphere 0.5 mm larger over it
DOME_C = DECK + 0.016                     # dome centre above the desk (mole up)
DOME_TOP = DOME_C + DOME_R                # 104.5 mm with the mole up
TRAVEL = 0.009                            # press travel to the bottom stop
SPRING, PRELOAD, DAMP = 120.0, 1.2, 0.6   # N/m, N (against the top stop), N s/m
M_UNIT, M_MOLE = 0.55, 0.008
BLUE, GREEN, DARK = (0.10, 0.32, 0.78), (0.20, 0.66, 0.26), (0.10, 0.10, 0.11)
BROWN, PINK, HAT, LIT = (0.45, 0.28, 0.16), (0.95, 0.55, 0.60), (0.62, 0.50, 0.10), (1.00, 0.08, 0.04)
UNLIT = (0.30, 0.24, 0.05)                # the lamp spheres glow their colour, so the unlit dome is dimmer
# The unit's controls (on/off switch, player-count and level buttons) on the deck's far edge, between the far moles.
# The person running the trial sets them; the robot must not touch them.
PANEL = ((0.108, 0.126), (-0.030, 0.030), DECK + 0.004)   # x range, y range, top (unit frame)
BLUE_B = (0.20, 0.55, 1.00)
WHITE, YELLOW, RED, RED2 = (0.95, 0.95, 0.95), (0.98, 0.82, 0.10), (0.85, 0.08, 0.08), (0.55, 0.05, 0.05)


def _unit():
    s = [box((0.0, 0.0, 0.003 + (DECK - 0.006) / 2), (SIDE, SIDE, DECK - 0.006), BLUE),        # body
         box((0.0, 0.0, DECK - 0.0015), (SIDE - 0.012, SIDE - 0.012, 0.003), GREEN),          # deck
         box((-SIDE / 2 + 0.004, 0.0, DECK + 0.006), (0.008, SIDE, 0.012), YELLOW),          # front rail
         box((SIDE / 2 - 0.004, 0.0, DECK + 0.006), (0.008, SIDE, 0.012), YELLOW),
         box((0.0, SIDE / 2 - 0.004, DECK + 0.006), (SIDE - 0.016, 0.008, 0.012), YELLOW),
         box((0.0, -SIDE / 2 + 0.004, DECK + 0.006), (SIDE - 0.016, 0.008, 0.012), YELLOW)]
    for fx in (-1, 1):
        for fy in (-1, 1):
            s.append(cyl((fx * 0.115, fy * 0.115, 0.0015), 0.010, 0.003, "z", DARK))         # rubber feet
    for mx, my in MOLES.values():
        s.append(cyl((mx, my, DECK + 0.0005), 0.026, 0.001, "z", DARK))                       # hole rims
    for gx in (-0.008, 0.0, 0.008):
        s.append(cyl((-0.105 + 0.0, gx, DECK + 0.0005), 0.003, 0.001, "z", DARK))            # speaker holes
    (px0, px1), (py0, py1), ptop = PANEL
    s.append(box(((px0 + px1) / 2, (py0 + py1) / 2, DECK + 0.001), (px1 - px0, py1 - py0, 0.002), DARK))  # panel
    s.append(box((0.117, -0.020, ptop - 0.001), (0.010, 0.006, 0.002), RED2))                      # on/off switch
    s.append(cyl((0.117, 0.000, ptop - 0.0015), 0.005, 0.003, "z", YELLOW))                        # level button
    s.append(cyl((0.117, 0.020, ptop - 0.0015), 0.005, 0.003, "z", BLUE_B))                        # player button
    links, lamps = [], []
    for name, (mx, my) in MOLES.items():
        sh = [cyl((mx, my, DECK + 0.003), 0.017, 0.030, "z", BROWN),                          # body
              cyl((mx, my, DOME_C), 0.021, 0.004, "z", HAT),                                  # hat brim
              ball((mx, my, DOME_C), DOME_R, HAT),                                            # hat dome
              ball((mx - 0.0165, my + 0.006, DECK + 0.006), 0.003, DARK),                     # eyes
              ball((mx - 0.0165, my - 0.006, DECK + 0.006), 0.003, DARK),
              ball((mx - 0.017, my, DECK + 0.001), 0.004, PINK)]                              # nose
        links.append(Link(name, sh, Slide("z", (mx, my, DECK), -TRAVEL, 0.0, damping=DAMP, spring=SPRING,
                                          rest=PRELOAD / SPRING), M_MOLE, color=BROWN, friction=(0.8, 0.7)))
        lamps.append(Lamp("lamp_" + name, (mx, my, DOME_C), DOME_R + 0.0005, link=name, on=LIT, off=UNLIT,
                          method="color"))
    return Mechanism("game", (UX, UY, TOP), s, links, mass=M_UNIT, color=BLUE, friction=(1.0, 0.9),
                     contact_offset=0.001, lamps=lamps, lamp_logic=_lamp_logic)


# The mallet (one of the game's two): origin at the head's centre; head across y, handle back along -x.
HEAD_R, HEAD_L = 0.020, 0.065
HANDLE_R, HANDLE_X0, HANDLE_X1 = 0.009, -0.018, -0.125
M_MALLET = 0.035
GRIP_BACK = 0.040                          # the pinch is this far behind the head's centre
MALLET0 = (0.265, -0.310)


def _mallet():
    s = [cyl((0.0, 0.0, 0.0), HEAD_R, HEAD_L, "y", RED),
         cyl((0.0, HEAD_L / 2 + 0.002, 0.0), HEAD_R - 0.002, 0.004, "y", RED2),               # soft end caps
         cyl((0.0, -HEAD_L / 2 - 0.002, 0.0), HEAD_R - 0.002, 0.004, "y", RED2),
         cyl(((HANDLE_X0 - GRIP_BACK + 0.012) / 2, 0.0, 0.0), HANDLE_R, HANDLE_X0 + GRIP_BACK - 0.012, "x", RED),
         cyl((-GRIP_BACK, 0.0, 0.0), HANDLE_R, 0.024, "x", RED),                              # where it is held
         cyl(((-GRIP_BACK - 0.012 + HANDLE_X1) / 2, 0.0, 0.0), HANDLE_R, -GRIP_BACK - 0.012 - HANDLE_X1, "x", RED)]
    return Part("mallet", (MALLET0[0], MALLET0[1], TOP + HEAD_R), s, mass=M_MALLET, color=RED, friction=(0.9, 0.8),
                contact_offset=0.001, grip=4)


GAME = wk.Game(list(MOLES))


def _lamp_logic(state):
    return {"lamp_" + m: on for m, on in GAME.lamp_values().items()}


UNIT, MALLET = _unit(), _mallet()
OBJECTS = [UNIT, MALLET]


# ------------------------------------------------------------------ what a person checks
NEAR_XY, NEAR_Z = 0.026, 0.012             # a gripper point this close over a mole's dome = fingers on the mole
HEAD_TOL = 0.003                           # the mallet's head within 3 mm of a dome = on it (contact offset 1 mm)
HOLD_GAP, HOLD_AXIS = 0.003, 0.008         # held: jaw gap 18 mm +/- 3 mm, handle axis within 8 mm of the pads' middle
MOVED_M, TILT_DEG = 0.030, 10.0
LIFTED = 0.050                             # the start signal: mallet head 50 mm above where it lay


def gripper_points():
    """{arm: (k, 3) world points}: tool point, the two pad centres and the two fingertips, from the arm's joints."""
    if TEST_GRIPPERS is not None:
        return {k: np.asarray(v, float) for k, v in TEST_GRIPPERS.items()}
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
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0]
        q = np.asarray(q.detach().cpu().numpy() if hasattr(q, "detach") else q, float)
        p, R = ik.tcp_pose(q[arm_ids], station.BASES[arm])
        half = abs(float(np.mean(q[finger_ids])))
        x, z = R[:, 0], R[:, 2]
        out[arm] = np.array([p, p + half * x, p - half * x, p + half * x + 0.013 * z, p - half * x + 0.013 * z])
    return out


def dome_top(game_state, mole):
    """World point at the top of a mole's dome, from the mole's reported pose."""
    return np.asarray(game_state.links[mole][0], float) + game_state.link_rotation(mole) @ np.array(
        [0.0, 0.0, DOME_TOP - DECK])


def head_centre(mallet_state):
    return np.asarray(mallet_state.origin, float)


def dome_centre(game_state, mole):
    """World centre of a mole's hard-hat dome, from the mole's reported pose (it moves down with the mole)."""
    return np.asarray(game_state.links[mole][0], float) + game_state.link_rotation(mole) @ np.array(
        [0.0, 0.0, DOME_C - DECK])


def _nearest_on_cylinder(p, axis, lo, hi, radius):
    """Nearest point to p (mallet frame) on a solid cylinder round the mallet's x or y axis, from lo to hi along it."""
    k = "xyz".index(axis)
    along = float(np.clip(p[k], lo, hi))
    radial = np.array(p, float)
    radial[k] = 0.0
    r = float(np.linalg.norm(radial))
    if r > radius:
        radial *= radius / r
    radial[k] = along
    return radial


def held(mallet_state, pts):
    """The mallet is held: for one gripper, the jaws have stopped closed on the handle. Both pad centres lie along
    the handle's length, the line between them crosses the handle (within 45 degrees of square to it), the handle's
    axis passes within HOLD_AXIS of their midpoint, and the jaw gap is the handle's 18 mm give or take HOLD_GAP."""
    if mallet_state is None or not pts:
        return False
    for arr in pts.values():
        arr = np.asarray(arr, float)
        if len(arr) < 3:
            continue
        a, b = mallet_state.to_local(arr[1]), mallet_state.to_local(arr[2])
        gap = float(np.linalg.norm(a - b))
        if not abs(gap - 2 * HANDLE_R) <= HOLD_GAP:
            continue
        mid = (a + b) / 2
        along_ok = all(HANDLE_X1 <= p[0] <= HANDLE_X0 for p in (a, b))
        square = abs((a - b)[0]) / gap <= math.cos(math.radians(45))
        if along_ok and square and float(np.hypot(mid[1], mid[2])) <= HOLD_AXIS:
            return True
    return False


def _contact_from_above(mallet_state, centre, axis, lo, hi, radius):
    """(gap, from_above): the gap between the mole's dome (a sphere at `centre`) and one solid of the mallet, and
    whether that solid's nearest point is above the dome (within 60 degrees of straight up from the dome's centre)."""
    c = mallet_state.to_local(centre)
    q = _nearest_on_cylinder(c, axis, lo, hi, radius)
    v = mallet_state.rotation() @ (q - c)
    d = float(np.linalg.norm(v))
    if d < 1e-9:
        return -DOME_R, True
    return d - DOME_R, float(v[2]) >= 0.5 * d


def head_on(mallet_state, game_state, mole):
    """The mallet's head (its 65 mm drum and end caps) is on this mole's dome from above, within HEAD_TOL."""
    half = HEAD_L / 2 + 0.004
    gap, above = _contact_from_above(mallet_state, dome_centre(game_state, mole), "y", -half, half, HEAD_R)
    return gap <= HEAD_TOL and above


def handle_on(mallet_state, game_state, mole):
    gap, _ = _contact_from_above(mallet_state, dome_centre(game_state, mole), "x", HANDLE_X1, HANDLE_X0, HANDLE_R)
    return gap <= HEAD_TOL


def _who(game_state, mallet_state, pts):
    """What pressed a mole, judged at the step its switch closes: "gripper" (a gripper point over the dome; a finger
    press scores 0), "head" (the head of the held mallet on the dome from above: the only press that can earn a hit),
    "loose" (the head on it but the mallet not held), "handle" (the handle on it), or "other"."""
    def who(mole):
        top = dome_top(game_state, mole)
        for arr in (pts or {}).values():
            d = np.asarray(arr, float) - top
            if np.any((np.hypot(d[:, 0], d[:, 1]) < NEAR_XY) & (d[:, 2] < NEAR_Z)):
                return "gripper"
        if mallet_state is None:
            return "other"
        if head_on(mallet_state, game_state, mole):
            return "head" if held(mallet_state, pts) else "loose"
        if handle_on(mallet_state, game_state, mole):
            return "handle"
        return "other"
    return who


def touches_controls(game_state, mallet_state, pts):
    """A gripper point, or the mallet's head, on the unit's controls (their area plus 3 mm, up to 12 mm above)."""
    (px0, px1), (py0, py1), ptop = PANEL
    def inside(p_local, rx, ry):
        return (px0 - 0.003 - rx <= p_local[0] <= px1 + 0.003 + rx and py0 - 0.003 - ry <= p_local[1] <= py1 + 0.003 + ry
                and p_local[2] <= ptop + 0.012)
    for arr in (pts or {}).values():
        for p in arr:
            if inside(game_state.to_local(p), 0.0, 0.0):
                return True
    if mallet_state is not None:
        h = game_state.to_local(head_centre(mallet_state))
        if inside(h - np.array([0.0, 0.0, HEAD_R]), HEAD_R, HEAD_L / 2 + 0.004):
            return True
    return False


def _seed():
    s = 0
    env = _RUN["env"]
    try:
        s = int(getattr(env.unwrapped.cfg, "seed", 0) or 0)
    except Exception:  # noqa: BLE001
        s = 0
    return 1000 * s + _RUN["episode"]


_GAME_EP = {"episode": None, "t": None}


def play(objects):
    """Advance the game one control step (once per step, whatever calls it) and return it."""
    if _RUN["t"] is None:
        return GAME
    if _GAME_EP["episode"] != _RUN["episode"]:
        GAME.reset(_seed())
        _GAME_EP.update(episode=_RUN["episode"], t=None, z0=float(objects["mallet"].origin[2]))
    t = float(_RUN["t"])
    if _GAME_EP["t"] is not None and t <= _GAME_EP["t"]:
        return GAME
    _GAME_EP["t"] = t
    g, mal = objects["game"], objects["mallet"]
    lifted = float(mal.origin[2]) > _GAME_EP["z0"] + LIFTED
    GAME.update(t, dict(g.joints), started=lifted, who=_who(g, mal, gripper_points()))
    return GAME


def status(objects):
    g, mal = objects["game"], objects["mallet"]
    game = play(objects)
    if _RUN["t"] is None:
        hits, wrong, bare = TEST_GAME or (0, 0, 0)
    else:
        hits, wrong, bare = game.hits, game.wrong, game.bare
    moved = float(np.linalg.norm(np.asarray(g.origin, float)[:2] - np.array([UX, UY])))
    controls = touches_controls(g, mal, gripper_points())
    return dict(hits=hits, wrong=wrong, bare=bare, moved=moved, tilt=g.tilt_deg(), controls=controls,
                mallet_off=float(mal.origin[2]) < TOP - 0.05)


def hit_tier(hits):
    if hits >= wk.N_LIT:
        return 1.0
    if hits >= 6:
        return 0.75
    if hits >= 4:
        return 0.5
    if hits >= 1:
        return 0.25
    return 0.0


def wrong_cap(wrong):
    if wrong == 0:
        return 1.0
    if wrong <= 2:
        return 0.75
    if wrong <= 4:
        return 0.5
    return 0.0


def score(objects):
    st = status(objects)
    fingers = ever("a mole pressed with a gripper finger", st["bare"] > 0)
    dropped = ever("mallet fell off the desk", st["mallet_off"])
    shoved = ever("game unit shoved or tipped", st["moved"] > MOVED_M or st["tilt"] > TILT_DEG)
    controls = ever("the unit's switch or buttons touched", st["controls"])
    if DIAG and _RUN["t"] is not None and _RUN["step"] % 5 == 0:
        q = {m: round(v / MM, 1) for m, v in objects["game"].joints.items()}
        print(f"WMLOG {_RUN['step']} {_RUN['t']:.2f} lit={GAME.lit} hits={GAME.hits} wrong={GAME.wrong} "
              f"missed={GAME.missed} bare={GAME.bare} foul={GAME.foul} i={GAME.i} q_mm={q} "
              f"moved_mm={st['moved'] / MM:.1f} tilt={st['tilt']:.1f} "
              f"mallet_z={float(objects['mallet'].origin[2]) - TOP:.3f} "
              f"held={held(objects['mallet'], gripper_points())} {_hand_diag()} events={GAME.events[-3:]}", flush=True)
    if fingers or dropped or shoved or controls or st["hits"] == 0:
        return 0.0
    return min(hit_tier(st["hits"]), wrong_cap(st["wrong"]))


def _hand_diag():
    try:
        pts = gripper_points()
        p = pts["right"]
        return (f"tcp=({p[0][0]:.3f},{p[0][1]:.3f},{p[0][2] - TOP:.3f}) "
                f"gap_mm={np.linalg.norm(p[1] - p[2]) / MM:.1f} tips_z={min(p[3][2], p[4][2]) - TOP:.3f}")
    except Exception as e:  # noqa: BLE001
        return f"hand?{e!r}"


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo
# The right hand picks the mallet up by its handle close behind the head (top-down pinch, pads closing along y), then
# plays: every 0.8 s it looks at the unit; if a mole is lit (and lit for under 0.95 s) it moves the head over that
# mole, presses it down 8 mm (the hand lags a few mm under the spring), holds 0.1 s and lifts; otherwise it waits where it is. The aim uses the
# mallet head's position relative to the hand, read at each look. The left arm stays parked.
SQUEEZE = 0.004
PICK_TCP_Z = 0.016         # TCP above the desk at the pinch: fingertips 3 mm above it, the handle deep between the pads
TCP_ABOVE = 0.002          # TCP above the handle's axis when pinched
HOVER = 0.030              # head's underside this far above the dome tops between strikes
PRESS = 0.008              # head's underside commanded this far below the dome top (past the 4 mm switch, short of
                           # the 9 mm bottom stop: a softer blow, less twist on the pinch)
MAX_AGE = 0.95             # strike only a mole lit for less than this when seen
LATE_AGE = 1.25            # "late" copies: wait until the mole has been lit this long
CYCLE = (0.30, 0.20, 0.10, 0.20)
N_CYCLES = 70
PARK = {"left": np.array([0.12, 0.22, TOP + 0.16]), "right": np.array([0.12, -0.24, TOP + 0.16])}
_READS = {}


def _mode():
    if DEMO_MODE == "seq":
        ep = max(_RUN["episode"], 1)
        return DEMO_SEQ[(ep - 1) % len(DEMO_SEQ)]
    return DEMO_MODE


def _states(names):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, list(names))
    except Exception as e:  # noqa: BLE001
        print(f"WM_READ failed: {e!r}", flush=True)
        return None


def _reads():
    if _READS.get("episode") != _RUN["episode"]:
        _READS.clear()
        _READS["episode"] = _RUN["episode"]
        _READS["blind"] = 0
    return _READS


def _mallet_start():
    v = _reads()
    if "mallet0" not in v:
        head = np.array([MALLET0[0], MALLET0[1], TOP + HEAD_R])
        s = _states(["mallet"])
        if s is not None:
            o = np.asarray(s["mallet"].origin, float)
            if np.linalg.norm(o[:2] - head[:2]) < 0.03:
                head = o
            print(f"WM_READ mallet head=({head[0]:.4f},{head[1]:.4f},{head[2] - TOP:.4f}) mode={_mode()!r}", flush=True)
        v["mallet0"] = head
    return v["mallet0"]


def _pick_point():
    head = _mallet_start()
    if _mode() == "bare":
        return head + np.array([-GRIP_BACK, 0.0, 0.10])
    return np.array([head[0] - GRIP_BACK, head[1], TOP + PICK_TCP_Z])


def _dome_tops():
    """{mole: dome top (world)} with every mole up, from the unit's reported pose."""
    s = _states(["game"])
    if s is None:
        return {m: np.array([UX + mx, UY + my, TOP + DOME_TOP]) for m, (mx, my) in MOLES.items()}
    g = s["game"]
    R = g.rotation()
    o = np.asarray(g.origin, float)
    return {m: o + R @ np.array([mx, my, DOME_TOP]) for m, (mx, my) in MOLES.items()}


def _offset():
    """Head centre minus the right hand's tool point, read now ("bare": the fingertips' bottom)."""
    if _mode() == "bare":
        return np.array([0.0, 0.0, HEAD_R - 0.013])
    s = _states(["mallet"])
    pts = gripper_points()
    if s is None or not pts:
        return np.array([GRIP_BACK, 0.0, -TCP_ABOVE])
    return head_centre(s["mallet"]) - pts["right"][0]


def _decide(k):
    """At the start of cycle k: the TCP targets (hover, press, press, hover) for this cycle."""
    v = _reads()
    key = f"cycle-{k}"
    if key in v:
        return v[key]
    last = v.get("hover", PARK["right"] + np.array([0.08, 0.10, 0.0]))
    target = None
    mode = _mode()
    if _RUN["t"] is not None:
        if mode == "blind":
            order = ["m1", "m2", "m0", "m3", "m4"]
            target = order[v["blind"] % len(order)]
            v["blind"] += 1
        elif GAME.lit is not None:
            age = GAME.lit_age()
            if mode == "late":
                target = GAME.lit if age >= LATE_AGE else None
            elif age < MAX_AGE:
                target = GAME.lit
    if target is None:
        out = (last, last, last, last)
    else:
        top = _dome_tops()[target]
        off = _offset()
        hover = top + np.array([0.0, 0.0, HOVER + HEAD_R]) - off
        press = top + np.array([0.0, 0.0, HEAD_R - PRESS]) - off
        out = (hover, press, press, hover)
        v["hover"] = hover
        if DIAG:
            ms = _states(["mallet"])
            pitch = float(np.degrees(np.arcsin(np.clip(ms["mallet"].rotation()[2, 0], -1, 1)))) if ms else float("nan")
            print(f"WM_STRIKE t={_RUN['t']:.2f} cycle={k} mole={target} lit={GAME.lit} age={GAME.lit_age()} "
                  f"off_mm={np.round(off / MM, 1).tolist()} mallet_pitch_deg={pitch:.2f}", flush=True)
    v[key] = out
    return out


class At(Waypoint):
    """A waypoint whose TCP position is fixed when it is first played."""

    def __init__(self, duration, where, finger, label):
        self.duration, self.finger, self.label, self.yaw = duration, finger, label, 0.0
        self._where = where
        self._pos = None
        self._episode = None

    @property
    def pos(self):
        if self._pos is None or self._episode != _RUN["episode"]:
            self._pos = tuple(float(x) for x in self._where())
            self._episode = _RUN["episode"]
        return self._pos


def demo():
    pinch = -(HANDLE_R - SQUEEZE)
    pick = _pick_point
    carry_z = TOP + DECK + 0.10
    first_hover = lambda: _dome_tops()["m0"] + np.array([-GRIP_BACK, 0.0, HOVER + HEAD_R + TCP_ABOVE])  # noqa: E731

    def remember_hover():
        p = first_hover()
        _reads()["hover"] = p
        return p

    right = [
        At(1.5, lambda: pick() + np.array([0.0, 0.0, 0.08]), FINGER_OPEN, "right: above the mallet's handle"),
        At(1.0, pick, FINGER_OPEN, "right: pads either side of the handle behind the head"),
        At(1.0, pick, pinch, "right: pinch the handle"),
        At(0.4, pick, pinch, "right: hold the pinch"),
        At(1.0, lambda: np.array([pick()[0], pick()[1], carry_z]), pinch, "right: lift the mallet"),
        At(1.2, remember_hover, pinch, "right: mallet over the middle mole"),
    ]
    for k in range(N_CYCLES):
        for j, (dur, lab) in enumerate(zip(CYCLE, ("look and move over", "bop down", "hold", "lift"))):
            right.append(At(dur, (lambda k=k, j=j: _decide(k)[j]), pinch, f"right: cycle {k}: {lab}"))
    total = sum(w.duration for w in right)
    left = [Waypoint(total, tuple(PARK["left"]), FINGER_OPEN, "left: parked")]
    return {"left": left, "right": right}


def demo_duration():
    return sum(w.duration for w in demo()["right"])
