"""On the station table: a full-size retractable dog leash (white round reel housing with a blue handle loop and
an orange thumb button, 173 x 109 x 33 mm, 330 g) lies flat on a dark slate mat on the left half of the table. Its
snap hook is clipped to the chrome D-ring of a red nylon dog collar buckled round a short steel post at the far left
end, with the tape fully wound in. A white ruler along the far edge of the mat reads 0 cm at the leash's snout and
has a green band from 23 to 27 cm. Pulling the housing toward the right arm pays out the orange tape against the
reel's recoil spring (2.5 N at 0 cm, 5 N at 25 cm); letting go reels it back in unless the reel is locked. The
thumb button locks the reel when it is pressed in (toward the far side of the table) and then slid forward (toward
the post) behind its catch; pressed but not slid, it springs back out when released.

Requires -E object_jitter=0: the ruler's 0 cm mark is the leash's fixed start position.

Run it:  inspect-robots run --embodiment yam_arena -E task=lock_retractable_dog_leash_at_25cm.py -E object_jitter=0 \\
           --instruction "Pull the dog leash out to the green 25 cm band, press and slide its thumb lock to lock
           the reel there, then let go." --scorer task_score ...
"""

from __future__ import annotations

import collections
import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

import belimits  # noqa: F401
from closedloop import grip_rot, placeholder_demo
import mechkit
from mechkit import (
    Joint,
    Mech,
    arc,
    body_poses,
    box,
    cyl,
    lamp,
    lathe,
    on_reset,
    on_step,
    released,
    rod,
    set_hold,
    set_lamp,
    set_rest,
    tube,
)

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SUCCESS_HOLD_S = 8.0
SCORE_OVER = "end"

INSTRUCTION = (
    "Pull the dog leash out to the green 25 cm band, press and slide its thumb lock to lock the reel there, then let go."
)

# -- palette (high contrast on 320x176 top_cam and wrist cams) ----------------------------------------------------------
MAT_SLATE = (0.16, 0.19, 0.24)
RULER_WHITE = (0.95, 0.95, 0.94)
TICK_BLACK = (0.08, 0.08, 0.10)
TARGET_GREEN = (0.12, 0.78, 0.28)
STEEL = (0.68, 0.71, 0.75)
CHROME = (0.82, 0.85, 0.88)
COLLAR_RED = (0.86, 0.12, 0.12)
TAPE_ORANGE = (0.98, 0.45, 0.05)
DRUM_WHITE = (0.95, 0.96, 0.97)
HANDLE_BLUE = (0.12, 0.36, 0.82)
SNOUT_GREY = (0.28, 0.30, 0.34)
BUTTON_ORANGE = (0.98, 0.42, 0.04)
LUG_ORANGE = (0.85, 0.32, 0.04)

# -- bench and leash geometry (station frame, metres; x forward, y left, z up) -----------------------------------------
MAT_T = 0.004
MAT_TOP = TOP + MAT_T
POST_X = 0.265
HOUSING_X = 0.270
Y_ANCHOR = 0.383  # steel post centre; 95 mm floor flange clears retracted snout (y=+0.330) by 5.5 mm
Y_START = 0.330   # retracted snout front face (0 cm mark)
TAPE_Z = 0.0205   # Mech-local z of tape / housing midplane (16.5 mm above mat top)
RULER_X = 0.350

# Recoil spring, brake clutch, and L-track lock constants
HOUSING_MASS = 0.308       # kg: ABS shell, internal reel, steel clock spring, wound nylon tape
BRAKE_CARRIER_MASS = 0.008 # kg: internal brake shoe and carrier plunger
THUMB_BUTTON_MASS = 0.014  # kg: external orange thumb paddle and L-track catch lug
TOTAL_LEASH_MASS = HOUSING_MASS + BRAKE_CARRIER_MASS + THUMB_BUTTON_MASS  # 0.330 kg (TUG 360 Small: 11.3 to 12 oz)

RECOIL_SPRING = 10.0       # N/m
RECOIL_PRELOAD_M = 0.250   # m -> 2.5 N at 0 cm, 5.0 N at 25 cm, 5.9 N at 34 cm stop
RECOIL_DAMPING = 2.5       # N s/m
RECOIL_HOLD = 0.15         # N static slide friction when unlocked
RECOIL_HOLD_MOVING = 0.12  # N dynamic slide friction when unlocked
BRAKE_HOLD = 60.0          # N static clutch hold while brake is depressed
BRAKE_HOLD_MOVING = 50.0   # N dynamic clutch hold while brake is depressed
BRAKE_ENGAGE_M = 0.0055    # brake_carrier >= 5.5 mm engages reel clutch
BRAKE_RELEASE_M = 0.0045   # brake_carrier < 4.5 mm releases reel clutch (hysteresis)
BRAKE_REST_FOLLOW_M = 0.0005  # while braked, the spring rest follows the housing (no recoil through a locked reel)

BRAKE_TRAVEL = 0.011       # m (+x press travel)
BRAKE_SPRING = 220.0       # N/m (2.42 N at full press, 1.54 N seated against catch lip at 7 mm)
BRAKE_DAMPING = 3.0

BUTTON_TRAVEL = 0.011      # m (+y slide travel)
BUTTON_SPRING = 150.0      # N/m detent spring toward nearest of (0.0, 0.011)
BUTTON_DAMPING = 2.0
BUTTON_HOLD = 0.18         # N (deadband = 1.2 mm)
BUTTON_HOLD_MOVING = 0.144

EXT_MAX = 0.340            # m max housing slide travel (-0.340..0.0)
TARGET_EXT = 0.250         # m (25 cm target)
TARGET_FULL_TOL = 0.020    # +-2 cm -> 23..27 cm (1.00)
TARGET_MID_TOL = 0.050     # +-5 cm -> 20..30 cm (0.50)
MIN_LOCK_EXT = 0.100       # >=10 cm (0.20)

# Scoring thresholds
LOCK_BC_MIN = 0.0050       # m: brake carrier physically trapped behind catch lip (~7.0 mm)
LOCK_TB_MIN = 0.0085       # m: thumb button seated in forward detent (~10.5..11.0 mm)
RELEASE_CLEAR = 0.030      # m: every finger pad point >= 30 mm from housing and thumb_button
MIN_OPEN_GAP = 0.070       # m: both grippers opened >= 70 mm
HOLD_WINDOW_STEPS = 60     # 3.0 s continuous at 20 Hz
MAX_CREEP_M = 0.0020       # m: snout shifts < 2.0 mm across the final 3.0 s window
TOOL_SPEED_LIMIT = 0.14    # m/s declared tool-point speed limit

# Tape visual reveal (14 look-only 25 mm flat orange nylon webbing pieces)
N_TAPE_SEGS = 14
TAPE_SEG_LEN = 0.025
TAPE_SEG_W = 0.0095
TAPE_SEG_H = 0.0020
TAPE_REVEAL_LEAD = 0.002   # show segment k once ext >= k * 25 mm + 2 mm (extra length hides inside opaque snout/drum)

# -- shapes -------------------------------------------------------------------------------------------------------------
_BASE_SOLID = [
    # Dark-slate non-slip silicone bench mat: 220 x 545 x 4 mm, x 0.160..0.380, y -0.210..+0.335 (beside flange rim)
    box((0.270, 0.0625, MAT_T / 2.0), (0.220, 0.545, MAT_T), MAT_SLATE),
    # Turned galvanized steel bench anchor post: 1-in. NPT pipe nipple (33.4 mm OD) in 95 mm OD floor flange (70 mm tall)
    lathe(
        (POST_X, Y_ANCHOR, 0.0),
        [
            (0.0475, 0.0),
            (0.0475, 0.006),
            (0.0230, 0.0065),
            (0.0230, 0.009),
            (0.0167, 0.0095),
            (0.0167, 0.068),
            (0.0140, 0.070),
        ],
        STEEL,
        sides=24,
    ),
    # Standard 1-inch (25.4 mm) red flat nylon dog collar strapped snugly around the 33.4 mm OD pipe above the flange hub
    tube((POST_X, Y_ANCHOR, 0.0225), r_in=0.0172, r_out=0.0212, height=0.0254, color=COLLAR_RED, sides=24),
    # Chrome-plated steel D-ring projecting toward -y from the collar
    arc((POST_X, 0.360, TAPE_Z), radius=0.010, a0=-math.pi, a1=0.0, color=CHROME, section=0.0025, axis="z", sides=12),
]

_BASE_LOOK = [
    # Chrome swivel snap hook connecting D-ring (y=+0.350) to retracted snout zero mark (y=+0.330)
    dataclasses.replace(
        rod((POST_X, Y_START, TAPE_Z), (POST_X, 0.350, TAPE_Z), radius=0.0038, color=CHROME),
        collide=False,
    ),
    # Adhesive white metric ruler strip (24 x 370 x 1 mm, 0 cm at y=+0.330 to 35 cm at y=-0.020)
    dataclasses.replace(
        box((RULER_X, Y_START - 0.165, MAT_T + 0.0005), (0.024, 0.370, 0.001), RULER_WHITE),
        collide=False,
    ),
    # Bold bright-green 25 cm target band (23..27 cm -> y=+0.060..+0.100, 40 mm wide)
    dataclasses.replace(
        box((RULER_X, Y_START - TARGET_EXT, MAT_T + 0.0009), (0.028, 0.040, 0.0012), TARGET_GREEN),
        collide=False,
    ),
    # Narrow green alignment strip at mat edge beside snout (x=0.326..0.338, y=+0.060..+0.100)
    dataclasses.replace(
        box((0.332, Y_START - TARGET_EXT, MAT_T + 0.0006), (0.012, 0.040, 0.0008), TARGET_GREEN),
        collide=False,
    ),
]

# Crisp 5 cm graduation tick marks along the white ruler (0, 5, 10, 15, 20, 25, 30, 35 cm)
for _cm in range(0, 36, 5):
    _ty = Y_START - _cm * 0.01
    _tick_c = RULER_WHITE if _cm == 25 else (COLLAR_RED if _cm == 0 else TICK_BLACK)
    _tick_z = MAT_T + (0.0016 if _cm == 25 else 0.0012)
    _BASE_LOOK.append(
        dataclasses.replace(
            box((RULER_X - 0.004, _ty, _tick_z), (0.014, 0.0022, 0.0006), _tick_c),
            collide=False,
        )
    )

def _tape_piece(k: int) -> tuple:
    """Paid-out tape piece k (25 mm of 9.5 mm flat orange webbing, look-only) as a mechkit lamp: its shown lens
    starts hidden and the on_step hook shows it once the snout has passed it. The unlit lens is a 1 mm cube buried
    inside the solid steel post, so it never shows."""
    at = (POST_X, Y_START - (k + 0.5) * TAPE_SEG_LEN, TAPE_Z)
    unlit, lit = lamp(f"tape{k}", at, 0.004, {"out": TAPE_ORANGE}, off=STEEL, height=TAPE_SEG_H, square=True)
    hidden = dataclasses.replace(unlit, at=(POST_X, Y_ANCHOR, 0.030 + 0.002 * k), size=(0.001, 0.001, 0.001))
    return (hidden, dataclasses.replace(lit, size=(TAPE_SEG_W, TAPE_SEG_LEN, TAPE_SEG_H)))


_TAPE_SHAPES = [s for k in range(N_TAPE_SEGS) for s in _tape_piece(k)]

BASE_SHAPES = tuple([*_BASE_SOLID, *_BASE_LOOK, *_TAPE_SHAPES])

# TUG 360 Small ABS/TPE housing (173 x 109 x 33 mm flat on mat)
HOUSING_SHAPES = (
    # Dark-grey 360-degree anti-tangle tape snout (mouth at y_local=0.0)
    box((-0.005, -0.008, 0.0165), (0.026, 0.016, 0.024), SNOUT_GREY),
    # Glossy white round spool drum (96 mm OD x 33 mm thick)
    cyl((0.006, -0.048, 0.0165), 0.048, 0.033, DRUM_WHITE, axis="z", sides=24),
    # Decorative royal-blue center hub cap (look-only visual detail)
    dataclasses.replace(cyl((0.006, -0.048, 0.0165), 0.020, 0.0342, HANDLE_BLUE, axis="z", sides=24), collide=False),
    # Royal-blue ergonomic TPE bottom handle rail (20 mm wide x 72 mm long x 33 mm tall)
    box((0.044, -0.125, 0.0165), (0.020, 0.072, 0.033), HANDLE_BLUE),
    # Royal-blue ergonomic TPE rear handle bar (84 mm wide x 22 mm thick x 33 mm tall)
    box((0.0, -0.162, 0.0165), (0.084, 0.022, 0.033), HANDLE_BLUE),
    # White upper/rear spine behind the thumb button slot
    box((-0.046, -0.146, 0.0165), (0.016, 0.030, 0.033), DRUM_WHITE),
    # Dark-grey internal L-track catch lip (blocks forward slide at bc=0 after 2 mm; traps lug at bc=7 mm)
    box((-0.050, -0.084, 0.0165), (0.008, 0.012, 0.026), SNOUT_GREY),
)

BRAKE_CARRIER_SHAPES = (
    # Internal brake plunger guide riding inside housing void (10.5 mm above mat)
    box((-0.022, -0.110, 0.0165), (0.008, 0.010, 0.012), SNOUT_GREY),
)

THUMB_BUTTON_SHAPES = (
    # Bright-orange unified thumb brake/lock paddle (14 x 24 x 18 mm, 7.5 mm above mat)
    box((-0.063, -0.116, 0.0165), (0.014, 0.024, 0.018), BUTTON_ORANGE),
    # Raised 5 mm ergonomic front thumb-grip ridge along +y edge of paddle for positive +y push engagement
    box((-0.0725, -0.1065, 0.0165), (0.005, 0.005, 0.018), BUTTON_ORANGE),
    # Internal L-track catch lug (2 mm clear of lip at rest; 4 mm behind lip at bc=11 mm; seats on lip at bc=7 mm)
    box((-0.050, -0.098, 0.0165), (0.006, 0.012, 0.018), LUG_ORANGE),
)

HOUSING_JOINT = Joint(
    "housing",
    "slide",
    HOUSING_SHAPES,
    axis="y",
    at=(HOUSING_X, Y_START, MAT_T),
    low=-EXT_MAX,
    high=0.0,
    spring=RECOIL_SPRING,
    damping=RECOIL_DAMPING,
    mass=HOUSING_MASS,
    friction=0.85,
    hold=RECOIL_HOLD,
    hold_moving=RECOIL_HOLD_MOVING,
)

BRAKE_CARRIER_JOINT = Joint(
    "brake_carrier",
    "slide",
    BRAKE_CARRIER_SHAPES,
    axis="x",
    at=(0.0, 0.0, 0.0),
    low=0.0,
    high=BRAKE_TRAVEL,
    spring=BRAKE_SPRING,
    damping=BRAKE_DAMPING,
    mass=BRAKE_CARRIER_MASS,
    friction=0.50,
    parent="housing",
)

THUMB_BUTTON_JOINT = Joint(
    "thumb_button",
    "slide",
    THUMB_BUTTON_SHAPES,
    axis="y",
    at=(0.0, 0.0, 0.0),
    low=0.0,
    high=BUTTON_TRAVEL,
    spring=BUTTON_SPRING,
    damping=BUTTON_DAMPING,
    mass=THUMB_BUTTON_MASS,
    friction=0.95,
    hold=BUTTON_HOLD,
    hold_moving=BUTTON_HOLD_MOVING,
    detents=(0.0, BUTTON_TRAVEL),
    parent="brake_carrier",
)

LEASH_BENCH = Mech(
    "leash_bench",
    pos=(0.0, 0.0, TOP),
    mass=None,
    shapes=BASE_SHAPES,
    joints=(HOUSING_JOINT, BRAKE_CARRIER_JOINT, THUMB_BUTTON_JOINT),
    friction=0.90,
)

OBJECTS = [LEASH_BENCH]

# Speed-limit task: no forbidden spatial keep-out zones on the bench.
ARM_KEEP_OUT: list[dict] = []

# -- scorer-side runtime state, clutch hook, and tape visibility --------------------------------------------------------


def _fresh_state() -> dict:
    return {
        "braked": False,
        "y_braked": None,
        "preload_set": False,
        "tape_out": 0,
        "nonfinite": False,
        "window": collections.deque(maxlen=HOLD_WINDOW_STEPS),
        "last_step": None,
        "last_objs": None,
    }


_STATE = _fresh_state()


def _clear() -> None:
    """Reset scorer latches (for offline unit tests in check.py)."""
    _STATE.clear()
    _STATE.update(_fresh_state())


def _show_tape(ext: float) -> None:
    """Show the paid-out tape pieces (scorer side, mechkit lamps): piece k once the snout is past it."""
    n = sum(1 for k in range(N_TAPE_SEGS) if ext >= k * TAPE_SEG_LEN + TAPE_REVEAL_LEAD)
    if n == _STATE["tape_out"]:
        return
    for k in range(N_TAPE_SEGS):
        set_lamp(LEASH_BENCH, f"tape{k}", "out" if k < n else "off")
    _STATE["tape_out"] = n


def _warn_once(what: str, exc: Exception) -> None:
    if not _STATE.get("warned"):
        _STATE["warned"] = True
        print(f"leash task: {what} failed: {exc!r}", flush=True)


def _set_recoil(env) -> None:
    """Brake off: the recoil spring pulls the housing back toward 0 cm (rest 25 cm past the 0 stop)."""
    set_rest(env, "leash_bench", "housing", RECOIL_PRELOAD_M)
    set_hold(env, "leash_bench", "housing", RECOIL_HOLD, RECOIL_HOLD_MOVING)


@on_reset
def _on_reset(env) -> None:
    _clear()
    if env is None:
        return
    try:
        mechkit._write_joint_rest(env, "leash_bench", ("housing", "brake_carrier", "thumb_button"))
        _set_recoil(env)
        _STATE["preload_set"] = True
    except Exception as exc:  # noqa: BLE001
        _warn_once("reset", exc)


@on_step
def _on_step(env) -> None:
    """Reel brake (scorer side): while the brake carrier is pressed 5.5 mm or more, the reel is held (spring rest
    moved to where the housing is, 60 N slide friction); below 4.5 mm the recoil spring is back. Then show the tape."""
    if env is None:
        return
    try:
        asset = env.unwrapped.scene["leash_bench"]
        q = mechkit._to_numpy(asset.data.joint_pos)[0]
        vals = {n.removesuffix("_joint"): float(v) for n, v in zip(asset.joint_names, q, strict=True)}
        hy, bc = vals["housing"], vals["brake_carrier"]
        if not (math.isfinite(hy) and math.isfinite(bc)):
            return
        if not _STATE["braked"] and not _STATE["preload_set"]:
            _set_recoil(env)
            _STATE["preload_set"] = True
        if not _STATE["braked"] and bc >= BRAKE_ENGAGE_M:
            _STATE["braked"] = True
            _STATE["y_braked"] = hy
            set_rest(env, "leash_bench", "housing", hy)
            set_hold(env, "leash_bench", "housing", BRAKE_HOLD, BRAKE_HOLD_MOVING)
        elif _STATE["braked"] and bc >= BRAKE_RELEASE_M and abs(hy - _STATE["y_braked"]) > BRAKE_REST_FOLLOW_M:
            # a locked reel pulls neither way: keep the spring's rest on the housing so only the clutch acts
            _STATE["y_braked"] = hy
            set_rest(env, "leash_bench", "housing", hy)
        elif _STATE["braked"] and bc < BRAKE_RELEASE_M:
            _STATE["braked"] = False
            _STATE["y_braked"] = None
            _set_recoil(env)
        _show_tape(max(0.0, -hy))
    except Exception as exc:  # noqa: BLE001
        _warn_once("brake hook", exc)


def _moving_leash_local_aabb(st) -> tuple[np.ndarray, np.ndarray]:
    """Tight bounding box (lo, hi) of moving housing and thumb_button in LEASH_BENCH's frame from body_poses()."""
    poses = body_poses(LEASH_BENCH, st)
    pts = []
    for joint_name, shapes in (("housing", HOUSING_SHAPES), ("thumb_button", THUMB_BUTTON_SHAPES)):
        rw, ow = poses[joint_name]
        for s in shapes:
            if not s.collide:
                continue
            w_pts = mechkit._shape_corners(s) @ rw.T + ow
            pts.append(np.array([st.to_local(p) for p in w_pts]))
    all_pts = np.vstack(pts)
    return all_pts.min(axis=0), all_pts.max(axis=0)


def _min_gripper_gap_live(env) -> float:
    """Smaller of left and right measured pad gaps (m) from live simulator encoders."""
    base = env.unwrapped
    gaps = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        jp = mechkit._to_numpy(robot.data.joint_pos)[0]
        jn = list(robot.joint_names)
        halves = [abs(float(jp[jn.index(n)])) for n in ("joint7", "joint8") if n in jn]
        if not halves:
            return 0.0
        gaps.append(2.0 * min(halves))
    return float(min(gaps))


def _release_and_gap_ok(st) -> tuple[bool, bool]:
    """(both arms >= 30 mm clear of moving housing/thumb_button, both gripper gaps >= 70 mm)."""
    env = mechkit._ENV["env"]
    if env is None:
        return False, False
    lo, hi = _moving_leash_local_aabb(st)
    rel = released(st, clearance=RELEASE_CLEAR, lo=lo, hi=hi, environment=env)
    try:
        gap_ok = _min_gripper_gap_live(env) >= MIN_OPEN_GAP
    except Exception:  # noqa: BLE001
        gap_ok = False
    return bool(rel), bool(gap_ok)


def _is_finite_state(st) -> bool:
    vals = [
        *np.asarray(st.origin, dtype=float).ravel(),
        *np.asarray(st.quat_xyzw, dtype=float).ravel(),
        *(float(st.joints[k]) for k in ("housing", "brake_carrier", "thumb_button")),
    ]
    return all(math.isfinite(v) for v in vals)


def _observe(objects) -> None:
    st = objects["leash_bench"]
    step = mechkit.step_count()
    if step > 0:
        if _STATE["last_step"] == step:
            return
        _STATE["last_step"] = step
    else:
        if _STATE["last_objs"] is objects:
            return
        _STATE["last_objs"] = objects
    if not _is_finite_state(st):
        _STATE["nonfinite"] = True
        return
    ext = -float(st.joints["housing"])
    bc = float(st.joints["brake_carrier"])
    tb = float(st.joints["thumb_button"])
    locked = (bc >= LOCK_BC_MIN) and (tb >= LOCK_TB_MIN)
    rel_ok, gap_ok = _release_and_gap_ok(st)
    _STATE["window"].append((ext, locked, rel_ok, gap_ok))


def score(objects) -> float:
    """Scored at end of run (SCORE_OVER='end'):
    - 0.0 if any nonfinite state occurred during the run, or if fewer than 3.0 s (60 steps) of continuous
      locked + released + open-gripper + still (<2 mm snout shift) state hold at the end.
    - 1.00 if locked continuously for final 3.0 s with ext in [0.230, 0.270] m (25 +- 2 cm, green band).
    - 0.50 if locked continuously for final 3.0 s with ext in [0.200, 0.300] m (25 +- 5 cm).
    - 0.20 if locked continuously for final 3.0 s with ext >= 0.100 m (outside 20..30 cm).
    - 0.00 otherwise.
    """
    _observe(objects)
    if _STATE["nonfinite"]:
        return 0.0
    win = _STATE["window"]
    if len(win) < HOLD_WINDOW_STEPS:
        return 0.0
    if not all(locked and rel_ok and gap_ok for _, locked, rel_ok, gap_ok in win):
        return 0.0
    exts = [e for e, _, _, _ in win]
    if (max(exts) - min(exts)) >= MAX_CREEP_M:
        return 0.0
    ext = exts[-1]
    if TARGET_EXT - TARGET_FULL_TOL <= ext <= TARGET_EXT + TARGET_FULL_TOL:
        return 1.0
    if TARGET_EXT - TARGET_MID_TOL <= ext <= TARGET_EXT + TARGET_MID_TOL:
        return 0.5
    if ext >= MIN_LOCK_EXT:
        return 0.2
    return 0.0


def success(objects) -> bool:
    """Full score (1.0) achieved and held continuously for SUCCESS_HOLD_S (8.0 s)."""
    return score(objects) >= 1.0


# -- scripted closed-loop bimanual demo --------------------------------------------------------------------------------
demo = placeholder_demo

# Speed-capped motion parameters (every tool-point move stays well under 0.14 m/s declared limit)
TRANSFER_MPS = 0.105
DRAW_MPS = 0.035
ACTION_MPS = 0.022
HOME_DUR_S = 5.4

# Fixed bench grasp coordinates (object_jitter=0)
# Left grasps bottom rail forward at y_local=-0.098 (width 20 mm along x), with 18 deg outward wrist lean
BOT_RAIL_X = HOUSING_X + 0.044          # 0.314 m
BOT_RAIL_Y_LOCAL = -0.098               # m in housing frame (9 mm inside +y rail end; maximizes handoff separation)
GRASP_Z = TOP + 0.024                   # pad tips at TOP+11 mm, clearing 4 mm mat by 7 mm
HOVER_Z = TOP + 0.105
HOVER_Z_R = TOP + 0.078                 # lower Right hover keeps right_link4 clear of left_link3 at handoff
LEFT_RELEASE_OPEN = -0.018              # 36 mm compact release (8 mm/side on 20 mm rail) before lifting clear of Right

# Right grasps rear handle bar toward its -x side at x=0.246, y_local=-0.162 (width 22 mm along y), compact 48 mm opening
REAR_BAR_X = HOUSING_X - 0.024          # 0.246 m (68 mm x-separation from Left at x=0.314 during handoff)
REAR_BAR_Y_LOCAL = -0.162               # m in housing frame
RIGHT_WINDOW_OPEN = -0.024              # 48 mm pad gap -> clears drum back cleanly inside 55 mm handle window

EXT_HANDOFF = 0.155                     # Left draws 0 -> 15.5 cm (bot_rail y=+0.077, rear_bar y=+0.013 -> >=9 mm arm gap)
EXT_GOAL = 0.250                        # Right draws 15.5 -> 25 cm (snout at y=+0.080, center of green band)
PULL_LEAD_COMP = 0.010                  # small PD compliance offset along -y against 5.0 N recoil pull

# Step 4 Left tilted closed-finger button push (pitch=1.05 rad, z=TOP+0.028; clears mat by 11.8 mm and spine by 5 mm)
LOCK_PITCH = 1.05
LOCK_Z = TOP + 0.028
LOCK_Y_LOCAL_START = -0.1200            # closed 19.8 mm front pads sit 1.1 mm behind front thumb ridge (-0.109)
LOCK_Y_LOCAL_END = -0.0990              # +21.0 mm commanded +y sweep drives thumb_button onto +11 mm stop past PD lag
LOCK_X_STAGE = 0.172
LOCK_X_TOUCH = 0.186
LOCK_X_PRESS = 0.2020                   # +5 mm PD preload into bc=11 mm stop; paddle -x face at x=0.211 clears spine/lip (0.216)


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


# Left bottom-rail grasp: pads closing along x (yaw=-pi/2), wrist leaned 18 deg outward toward +y
ROT_LEFT_RAIL = _rx(-math.radians(18.0)) @ grip_rot(yaw=-math.pi / 2.0)
# Right rear-bar grasp: pads closing along y (yaw=0), wrist leaned 12 deg outward toward -y
ROT_RIGHT_BAR = _rx(math.radians(12.0)) @ grip_rot(yaw=0.0)
# Left button press-and-slide orientation: tilted 1.05 rad forward (+x), jaw axis along y
ROT_LEFT_LOCK = grip_rot(yaw=0.0, pitch=LOCK_PITCH, roll=0.0)


def _move_capped(arm, pos, max_mps: float, finger: float | None = None, *, rot=None, min_dur: float = 0.45):
    """Smoothstep Cartesian move sized so peak tool-point speed (1.5 * dist / dur) stays below max_mps."""
    p = np.asarray(pos, dtype=float)
    dist = float(np.linalg.norm(p - arm.pos))
    dur = max(float(min_dur), 1.60 * dist / float(max_mps))
    yield from arm.move(p, duration=dur, finger=finger, rot=rot)


def _live_ext(ctx, fallback: float) -> float:
    """Read current leash extension (-housing joint, m) from simulator state (falls back offline)."""
    if ctx.offline:
        return float(fallback)
    st = ctx.obj("leash_bench")
    return max(0.0, -float(st.joints["housing"]))


def demo_script(ctx):
    """Bimanual closed-loop demo:
    1) Left arm grips the blue bottom rail and draws the leash to 15.5 cm against recoil tension.
    2) Right arm descends into the handle window (48 mm opening) to grip the rear bar; Left releases and lifts clear.
    3) Right arm draws the housing smoothly to 25.0 cm (snout centered in green band) and holds steady.
    4) Left arm approaches the orange thumb paddle with closed fingers tilted at pitch 1.05 rad, presses +x
       to depress brake_carrier onto its 11 mm stop, holds +x preload while sweeping +y to push the catch lug
       behind the catch lip into the forward detent, then retracts clear.
    5) Right arm opens compactly and lifts straight up; both arms park at READY and hold while
       the locked leash stays stationary at 25.0 cm.
    """
    L, R = ctx.left, ctx.right

    # -- Step 1: Left grasps bottom handle rail and draws 0 -> 15.5 cm -------------------------------------------------
    y_l0 = Y_START + BOT_RAIL_Y_LOCAL
    y_l1 = Y_START - EXT_HANDOFF + BOT_RAIL_Y_LOCAL

    yield from _move_capped(L, (BOT_RAIL_X, y_l0, HOVER_Z), TRANSFER_MPS, FINGER_OPEN, rot=ROT_LEFT_RAIL)
    yield from L.trim_to_measured((BOT_RAIL_X, y_l0, HOVER_Z))
    yield from _move_capped(L, (BOT_RAIL_X, y_l0, GRASP_Z), TRANSFER_MPS, FINGER_OPEN, rot=ROT_LEFT_RAIL)
    yield from L.grip(FINGER_CLOSED, 0.7)
    yield from L.hold(0.3)
    if not L.holding(0.014):
        # Retry branch (exercised in dry_run pessimistic pass)
        yield from L.grip(FINGER_OPEN, 0.5)
        yield from _move_capped(L, (BOT_RAIL_X, y_l0, GRASP_Z - 0.002), ACTION_MPS, FINGER_OPEN, rot=ROT_LEFT_RAIL)
        yield from L.grip(FINGER_CLOSED, 0.7)
        yield from L.hold(0.3)

    # Smooth draw from 0 to 15.5 cm against 2.5..4.05 N recoil tension
    yield from _move_capped(L, (BOT_RAIL_X, y_l1 - 0.006, GRASP_Z), DRAW_MPS, FINGER_CLOSED, rot=ROT_LEFT_RAIL)
    yield from L.hold(0.3)

    # -- Step 2: Tension handoff to Right arm on rear handle bar -------------------------------------------------------
    ext_mid = _live_ext(ctx, EXT_HANDOFF)
    y_r0 = Y_START - ext_mid + REAR_BAR_Y_LOCAL

    yield from _move_capped(R, (REAR_BAR_X, y_r0, HOVER_Z_R), TRANSFER_MPS, RIGHT_WINDOW_OPEN, rot=ROT_RIGHT_BAR)
    yield from R.trim_to_measured((REAR_BAR_X, y_r0, HOVER_Z_R))
    yield from _move_capped(R, (REAR_BAR_X, y_r0, GRASP_Z), TRANSFER_MPS, RIGHT_WINDOW_OPEN, rot=ROT_RIGHT_BAR)
    yield from R.grip(FINGER_CLOSED, 0.7)
    yield from R.hold(0.3)
    if not R.holding(0.016):
        yield from R.grip(RIGHT_WINDOW_OPEN, 0.5)
        yield from _move_capped(R, (REAR_BAR_X, y_r0, GRASP_Z - 0.002), ACTION_MPS, RIGHT_WINDOW_OPEN, rot=ROT_RIGHT_BAR)
        yield from R.grip(FINGER_CLOSED, 0.7)
        yield from R.hold(0.3)

    # Left releases bottom rail compactly and lifts diagonally up+left cleanly away from Right
    yield from L.grip(LEFT_RELEASE_OPEN, 0.45)
    yield from _move_capped(L, (BOT_RAIL_X, L.pos[1] + 0.055, HOVER_Z + 0.020), TRANSFER_MPS, FINGER_OPEN, rot=ROT_LEFT_RAIL)

    # -- Step 3: Right draws smoothly to 25.0 cm target (snout in green band) ------------------------------------------
    y_r_goal = Y_START - EXT_GOAL + REAR_BAR_Y_LOCAL - PULL_LEAD_COMP
    yield from _move_capped(R, (REAR_BAR_X, y_r_goal, GRASP_Z), DRAW_MPS, FINGER_CLOSED, rot=ROT_RIGHT_BAR)
    yield from R.hold(0.4)

    # Closed-loop fine trim using live housing extension so snout lands squarely at 25.0 cm
    for _ in range(2):
        ext_now = _live_ext(ctx, EXT_GOAL)
        err_ext = EXT_GOAL - ext_now
        if abs(err_ext) <= 0.003:
            break
        err_clamped = float(np.clip(err_ext, -0.025, 0.025))
        yield from _move_capped(
            R, (REAR_BAR_X, R.pos[1] - err_clamped, GRASP_Z), ACTION_MPS, FINGER_CLOSED, rot=ROT_RIGHT_BAR
        )
        yield from R.hold(0.3)

    # -- Step 4: Left executes coupled press-in (+x) then slide-forward (+y) L-track lock ------------------------------
    ext_lock = _live_ext(ctx, EXT_GOAL)
    y_pad_start = Y_START - ext_lock + LOCK_Y_LOCAL_START
    y_pad_end = Y_START - ext_lock + LOCK_Y_LOCAL_END

    # Move high above staging, close fingers to compact 19.7 mm push tip, then descend to staging behind paddle
    yield from _move_capped(L, (LOCK_X_STAGE, y_pad_start, HOVER_Z), TRANSFER_MPS, FINGER_CLOSED, rot=ROT_LEFT_LOCK)
    yield from _move_capped(L, (LOCK_X_STAGE, y_pad_start, LOCK_Z), TRANSFER_MPS, FINGER_CLOSED, rot=ROT_LEFT_LOCK)
    yield from L.trim_to_measured((LOCK_X_STAGE, y_pad_start, LOCK_Z))

    # Advance +x to contact paddle -x face, then press +x to depress brake_carrier to 11 mm stop
    yield from _move_capped(L, (LOCK_X_TOUCH, y_pad_start, LOCK_Z), ACTION_MPS, FINGER_CLOSED, rot=ROT_LEFT_LOCK)
    yield from _move_capped(L, (LOCK_X_PRESS, y_pad_start, LOCK_Z), ACTION_MPS, FINGER_CLOSED, rot=ROT_LEFT_LOCK)
    yield from L.hold(0.35)

    # Brake on: the reel no longer pulls, so ease the right hand's pull lead off (command to where the hand is)
    yield from _move_capped(
        R, (REAR_BAR_X, float(R.measured_pos[1]), GRASP_Z), ACTION_MPS, FINGER_CLOSED, rot=ROT_RIGHT_BAR, min_dur=0.6
    )

    # Hold +x press preload while sweeping +y to push catch lug behind catch lip into forward detent
    yield from _move_capped(L, (LOCK_X_PRESS, y_pad_end, LOCK_Z), ACTION_MPS, FINGER_CLOSED, rot=ROT_LEFT_LOCK)
    yield from L.hold(0.45)

    # Retract cleanly -x so brake_carrier return spring seats catch lug firmly against catch lip at ~7 mm
    yield from _move_capped(L, (LOCK_X_STAGE, y_pad_end, LOCK_Z), ACTION_MPS, FINGER_CLOSED, rot=ROT_LEFT_LOCK)
    yield from _move_capped(L, (LOCK_X_STAGE, y_pad_end, HOVER_Z), TRANSFER_MPS, FINGER_OPEN, rot=ROT_LEFT_LOCK)

    # -- Step 5: Right opens cleanly off rear bar and lifts straight up; both arms park at READY -----------------------
    yield from R.grip(RIGHT_WINDOW_OPEN, 0.6)
    yield from _move_capped(R, (REAR_BAR_X, R.pos[1], HOVER_Z + 0.025), TRANSFER_MPS, FINGER_OPEN, rot=ROT_RIGHT_BAR)

    # Park both arms smoothly at READY (slow enough to stay under 0.14 m/s tool-point speed cap)
    yield from ctx.both(L.home(duration=HOME_DUR_S), R.home(duration=HOME_DUR_S))

    # Hold parked at READY with grippers open so 3.0 s stillness window + 8.0 s SUCCESS_HOLD_S complete cleanly
    yield from ctx.wait(12.5)
