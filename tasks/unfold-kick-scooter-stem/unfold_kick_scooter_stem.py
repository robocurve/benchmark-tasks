"""On the station table, in front of the arms, a full-size personal kick scooter (Razor A type) lies with its
deck resting on the bench and its steering stem folded flat against the deck, as a scooter is stored or carried.
The deck is an aluminium extrusion (44 x 9.5 cm) with a rear urethane wheel and a front folding head bracket.
The steering column (stem) is an aluminium tube (28 mm OD, 40 cm long) ending in a T-bar handlebar (24 cm wide),
hinged on a horizontal pivot at the front folding bracket. Folded, the stem lies flat along the deck (+x),
supported over the deck on rubber rests.

To make the scooter safe to ride, two distinct locking mechanisms must be engaged:
1. Joint Latch Pin: As the stem is swung up to vertical (90 deg from folded), a spring-loaded cylindrical
   steel locking pin on the stem bracket drops into the head bracket's locking socket. This positive mechanical
   latch stops the stem from folding under riding loads.
2. Quick-Release Collar Clamp: At the folding pivot, an over-centre quick-release cam lever clamps the split
   bracket around the hinge to eliminate steering wobble. Left open, the stem retains noticeable angular play;
   clamped flat against the column, the joint is fully rigid.

Then the verification check: once the robot has let go and stood clear for 2.0 s (40 control steps), a padded
test rod pushes horizontally against the upright stem 16 cm above the pivot with 12 N for 1.5 s, then retracts.
- An unlatched stem folds right back down (> 3 deg fold deflection) -> 0.0 score.
- A latched stem with the quick-release lever left open wobbles > 2.0 deg -> partial score (0.50).
- A latched stem with the quick-release lever fully closed deflects < 0.8 deg -> full score (1.00).

The job: swing the stem up out of its folded position until it stands vertical and its latch pin clicks into
the locking hole, close the quick-release clamp lever flat against the column, let go and stand clear.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/unfold-kick-scooter-stem/unfold_kick_scooter_stem.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import os
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print mechanism readouts
VARIANT = os.getenv("DEMO_VARIANT", "demo")  # "demo", "no_clamp" (unfolded, QR left open), "unlatched" (fails push test)

# Frame: x forward from the arm bases, y left, z up, metres.
# Left arm base: (0, +0.305, 0.75), Right arm base: (0, -0.305, 0.75).
# Origin of the scooter folding hinge H:
XH = 0.220
YH = 0.000
HZ = 0.055  # height of hinge above tabletop
H = np.array([XH, YH, TOP + HZ])  # hinge centre

# Scooter geometry:
DECK_L, DECK_W, DECK_H = 0.440, 0.095, 0.028
DECK_X = XH + 0.025 + DECK_L / 2  # centred along x
WHEEL_R, WHEEL_W = 0.0475, 0.024
STEM_L, STEM_R = 0.400, 0.014  # 28 mm diameter tube
BAR_W, BAR_R = 0.240, 0.011    # 24 cm wide handlebar

# Hinge angles: 0 deg = upright (vertical), 90 deg = folded flat along deck (+x)
START_DEG = 90.0
UPRIGHT_DEG = 0.0
STEM_FRICTION = 0.10  # N*m friction in folding hinge
STEM_KG = 0.40
DECK_KG = 1.35

# Latch pin specifications:
PIN_STROKE = 0.008  # 8 mm travel into socket
PIN_LOCKED_S = 0.005  # seated >= 5 mm counts as latched
PIN_KG = 0.03
PIN_K = 150.0  # spring stiffness

# Quick-Release collar clamp lever:
LEVER_L = 0.050  # 50 mm long paddle
LEVER_KG = 0.03
# Angle from closed: 0 deg = clamped flat against bracket, 80 deg = open
QR_OPEN_DEG = 80.0
QR_CLOSED_DEG = 0.0
QR_LOCKED_DEG = 12.0  # within 12 deg of closed counts as clamped

# Push test specifications:
PUSH_R = 0.160  # 16 cm up the stem from hinge
PUSH_Z = H[2] + PUSH_R
PUSH_REST_X = 0.660  # 66 cm from arm base midpoint (clear of folded stem and handlebar)
PAD = (0.024, 0.050, 0.050)
PAD_KG = 0.20
PUSH_TRAVEL = 0.450  # 45 cm forward travel towards stem
PUSH_TARGET = 0.428  # 42.8 cm stroke, contacts upright stem at 0.426 m with compliant deflection
K_PUSH = 1500.0

# Materials and colours:
ALU = (0.75, 0.76, 0.78)
DARK_ALU = (0.35, 0.36, 0.38)
BLACK_RUBBER = (0.12, 0.12, 0.12)
URETHANE_RED = (0.85, 0.15, 0.15)
STEEL_CHROME = (0.88, 0.89, 0.90)
RIG = (0.20, 0.36, 0.62)

kit = mk.Kit()

# 1. Base / Deck (fixed to table with mounting feet/pads)
kit.part("deck", (DECK_X, YH, TOP + DECK_H / 2), [
    mk.box((DECK_L, DECK_W, DECK_H), color=ALU),
    # Grip tape on deck:
    mk.box((DECK_L - 0.060, DECK_W - 0.015, 0.002), at=(0, 0, DECK_H / 2 + 0.001), color=BLACK_RUBBER),
    # Rear fork and 95 mm rear wheel:
    mk.box((0.080, 0.010, 0.050), at=(DECK_L / 2 + 0.010, 0.025, 0.015), color=ALU),
    mk.box((0.080, 0.010, 0.050), at=(DECK_L / 2 + 0.010, -0.025, 0.015), color=ALU),
    mk.box((2 * WHEEL_R, WHEEL_W, 2 * WHEEL_R), at=(DECK_L / 2 + 0.020, 0, 0.015), color=URETHANE_RED),
    # Rear spring fender brake:
    mk.box((0.090, 0.032, 0.004), at=(DECK_L / 2 - 0.020, 0, 0.035), color=DARK_ALU),
    # Front folding bracket blocks (spaced along y so stem hinge fits in between):
    mk.box((0.040, 0.012, HZ + 0.015), at=(-DECK_L / 2 - 0.010, 0.024, (HZ + 0.015 - DECK_H) / 2), color=DARK_ALU),
    mk.box((0.040, 0.012, HZ + 0.015), at=(-DECK_L / 2 - 0.010, -0.024, (HZ + 0.015 - DECK_H) / 2), color=DARK_ALU),
    # Folded rest bumper:
    mk.box((0.030, 0.050, 0.010), at=(0.080, 0, DECK_H / 2 + 0.005), color=BLACK_RUBBER),
], color=ALU)

# 2. Steering Column / Stem (hinged at H about y-axis)
_stem_boxes = [
    # Lower clamp collar & hinge sleeve:
    mk.box((0.036, 0.034, 0.050), at=(0, 0, 0.025), color=DARK_ALU),
    # Main column tube:
    mk.box((2 * STEM_R, 2 * STEM_R, STEM_L), at=(0, 0, STEM_L / 2), color=ALU),
    # Handlebar crossbar T-junction:
    mk.box((0.030, BAR_W, 0.024), at=(0, 0, STEM_L), color=DARK_ALU),
    # Rubber grips:
    mk.box((0.026, 0.080, 0.026), at=(0, BAR_W / 2 - 0.040, STEM_L), color=BLACK_RUBBER),
    mk.box((0.026, 0.080, 0.026), at=(0, -BAR_W / 2 + 0.040, STEM_L), color=BLACK_RUBBER),
]
kit.part("stem", tuple(H), _stem_boxes, mass=STEM_KG, color=ALU)
kit.hinge("stem_hinge", "deck", "stem", anchor=tuple(H), axis=(0, 1, 0),
          lower_deg=UPRIGHT_DEG, upper_deg=START_DEG, start_deg=START_DEG, friction=STEM_FRICTION)

# 3. Latch Pin (slides along the stem collar radially/axially into deck socket)
kit.part("latch_pin", (XH - 0.012, YH, H[2] + 0.025), [
    mk.box((0.018, 0.008, 0.008), color=STEEL_CHROME),
    # Thumb release knob:
    mk.box((0.008, 0.020, 0.010), at=(0.004, 0, 0), color=URETHANE_RED),
], mass=PIN_KG, color=STEEL_CHROME)
kit.slider("pin_slide", "stem", "latch_pin", axis=(-1, 0, 0), lower=0.0, upper=PIN_STROKE,
           start=0.0, stiffness=PIN_K, damping=5.0, rest=0.0)

# 4. Quick-Release Collar Clamp Lever
# Mounted on stem collar bracket outside face at y = -0.038 m
QR_ANCHOR = (XH, YH - 0.038, H[2] + 0.035)
kit.part("qr_lever", QR_ANCHOR, [
    # Cam hub:
    mk.box((0.014, 0.010, 0.014), color=DARK_ALU),
    # Paddle handle:
    mk.box((0.008, 0.012, LEVER_L), at=(0, 0, LEVER_L / 2), color=URETHANE_RED),
], mass=LEVER_KG, color=URETHANE_RED)
kit.hinge("qr_hinge", "stem", "qr_lever", anchor=QR_ANCHOR, axis=(0, -1, 0),
          lower_deg=QR_CLOSED_DEG, upper_deg=QR_OPEN_DEG, start_deg=QR_OPEN_DEG, friction=0.08)

# 5. Push-Test Rig (applies verification load to the upright stem)
kit.part("pusher", (PUSH_REST_X, YH, PUSH_Z), [
    mk.box(PAD, color=BLACK_RUBBER),
    mk.box((0.240, 0.018, 0.018), at=(0.120 + PAD[0] / 2, 0, 0), color=STEEL_CHROME),
], mass=PAD_KG, color=RIG)
kit.slider("push", None, "pusher", axis=(-1, 0, 0), lower=0.0, upper=PUSH_TRAVEL,
           start=0.0, stiffness=K_PUSH, damping=20.0, rest=0.0)

# Fixed test rig mount on bench (gantry arch clear of scooter parts across full swing arc):
H_TOWER = PUSH_Z - TOP + 0.070
kit.part("rig_mount", (PUSH_REST_X, YH, TOP), [
    # Left upright post:
    mk.box((0.060, 0.050, H_TOWER), at=(0, 0.180, H_TOWER / 2)),
    # Right upright post:
    mk.box((0.060, 0.050, H_TOWER), at=(0, -0.180, H_TOWER / 2)),
    # Overhead crossbeam:
    mk.box((0.060, 0.410, 0.040), at=(0, 0, H_TOWER - 0.020)),
    # Slider side guide cheeks (spaced across rod/pad with clearance):
    mk.box((0.060, 0.010, 0.050), at=(0, 0.035, PUSH_Z - TOP)),
    mk.box((0.060, 0.010, 0.050), at=(0, -0.035, PUSH_Z - TOP)),
    # Base plates on bench:
    mk.box((0.100, 0.100, 0.015), at=(0, 0.180, 0.0075)),
    mk.box((0.100, 0.100, 0.015), at=(0, -0.180, 0.0075)),
], color=RIG)

OBJECTS = [*kit.objects()]
PARTS = ("stem", "latch_pin", "qr_lever")

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 0.05
CLEAR_MARGIN = 0.12  # 12 cm clear of all parts (prevents premature trigger before arms are parked)
QUIET_STEPS = 40     # 2.0 s still before push test
RAMP_STEPS, HOLD_STEPS, BACK_STEPS, SETTLE_STEPS = 10, 30, 10, 20
M = "ukss"  # per-episode memory key
_TEST_TCPS = None


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {
            "n": 0, "init": False, "worked": False, "quiet": 0, "phase": None, "k": 0,
            "base_deg": None, "fold_max": 0.0, "failed": False, "prev": {}, "still": {},
            "armed_ok": False, "touched": False, "qr_ok": False, "contacted": False
        }
    return m


def stem_deg(objects) -> float:
    """Degrees from upright (0 = upright, 90 = folded flat)."""
    return kit.q_deg(objects, "stem_hinge")


def is_upright(objects) -> bool:
    """Within 2.0 deg of vertical."""
    return stem_deg(objects) <= 2.0


def pin_seated(objects) -> bool:
    """Latch pin seated >= 5 mm into the locking socket."""
    return kit.q(objects, "pin_slide") >= PIN_LOCKED_S


def qr_deg(objects) -> float:
    """Degrees from closed position."""
    return kit.q_deg(objects, "qr_hinge")


def is_qr_closed(objects) -> bool:
    """Quick-release lever folded within 12 deg of bracket."""
    return qr_deg(objects) <= QR_LOCKED_DEG


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _box_dist(objects, part, pt) -> float:
    t = kit._pose_T(objects, part)
    best = 1e9
    for b in kit.parts[part].boxes:
        tb = t @ mk._T(mk.quat_to_mat(b.quat), b.at)
        local = mk._inv(tb) @ np.r_[np.asarray(pt, float), 1.0]
        d = np.maximum(np.abs(local[:3]) - np.asarray(b.size) / 2, 0.0)
        best = min(best, float(np.linalg.norm(d)))
    return best


def clear_of(objects, margin=CLEAR_MARGIN) -> bool:
    """Both gripper TCPs at least margin from stem, latch pin, and QR lever."""
    tc = _tcps()
    if not tc:
        return True
    return all(_box_dist(objects, p, t) >= margin for t in tc for p in PARTS)


def _set_drive(name, target, stiffness=None) -> None:
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics
    prim = mk._SIM["stage"].GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI(prim, "angular" if "hinge" in name else "linear")
    drive.GetTargetPositionAttr().Set(float(target))
    if stiffness is not None:
        drive.GetStiffnessAttr().Set(float(stiffness))


def _rules(objects) -> None:
    """Per-step rule: stillness, latch engagement, clamp stiffness, automated push test."""
    m = _mem()
    m["n"] += 1
    if not m["init"]:
        m["init"] = True
        _set_drive("push", 0.0, stiffness=K_PUSH)

    for p in (*PARTS, "pusher"):
        t = kit._pose_T(objects, p)
        t0 = m["prev"].get(p)
        m["still"][p] = t0 is None or bool(
            np.linalg.norm(t[:3, 3] - t0[:3, 3]) < 0.001
            and mk._rot_angle(t[:3, :3] @ t0[:3, :3].T) < 0.5 * DT
        )
        m["prev"][p] = t

    deg = stem_deg(objects)
    pin_q = kit.q(objects, "pin_slide")
    qr_d = qr_deg(objects)
    push_q = kit.q(objects, "push")
    if DEBUG and (m["n"] % 25 == 0 or m["n"] >= 250):
        print(f"[DBG step {m['n']}] deg={deg:.2f} pin_q={pin_q:.4f} qr_d={qr_d:.1f} push_q={push_q:.4f} phase={m['phase']} quiet={m['quiet']} failed={m['failed']} clear={clear_of(objects)} still={all(m['still'].values())} parts_still={m['still']}")
    if abs(deg - START_DEG) >= 5.0 or qr_deg(objects) <= 60.0:
        m["worked"] = True

    # Spring-loaded latch pin snaps into socket when stem is swung upright
    if deg <= 4.0 and VARIANT != "unlatched":
        m["pin_engaged"] = True
        _set_drive("pin_slide", PIN_STROKE, stiffness=200.0)
    elif not m.get("pin_engaged"):
        _set_drive("pin_slide", 0.0, stiffness=PIN_K)

    # Over-center cam mechanism on QR clamp lever:
    # When the operator/robot pushes the lever paddle past the over-center cam threshold (<= 75 deg),
    # the internal cam spring snaps and drives the lever flat into the locked closed position (0 deg).
    if qr_d <= 75.0 and VARIANT not in ("no_clamp", "unlatched"):
        m["qr_cam_snapped"] = True
        _set_drive("qr_hinge", 0.0, stiffness=80.0)
    elif not m.get("qr_cam_snapped"):
        _set_drive("qr_hinge", math.radians(QR_OPEN_DEG), stiffness=0.0)

    # Physical clamping effect: closed QR collar stiffens the hinge against deflection
    if m.get("pin_engaged") and pin_seated(objects):
        if is_qr_closed(objects):
            _set_drive("stem_hinge", 0.0, stiffness=500.0)  # clamped rigid: < 0.8 deg deflection under 12 N
        else:
            _set_drive("stem_hinge", 0.0, stiffness=50.0)   # unclamped play: ~2.2 deg wobble under 12 N

    ph = m["phase"]
    if ph is None:
        quiet = m["worked"] and clear_of(objects) and all(m["still"].values())
        m["quiet"] = m["quiet"] + 1 if quiet else 0
        if m["quiet"] >= QUIET_STEPS:
            m["phase"], m["k"], m["base_deg"] = "ramp", 0, deg
            m["armed_ok"] = is_upright(objects) and pin_seated(objects)
            m["qr_ok"] = is_qr_closed(objects)
            m["touched"] = False
            m["contacted"] = False
    else:
        m["k"] += 1
        k = m["k"]
        if not clear_of(objects):
            m["touched"] = True

        # Track pusher contact with stem
        push_travel = kit.q(objects, "push")
        if push_travel >= 0.410:
            m["contacted"] = True

        if ph == "ramp":
            _set_drive("push", PUSH_TARGET * min(1.0, k / RAMP_STEPS))
            if k >= RAMP_STEPS:
                m["phase"], m["k"] = "hold", 0
        elif ph == "hold" and k >= HOLD_STEPS:
            m["phase"], m["k"] = "back", 0
        elif ph == "back":
            _set_drive("push", PUSH_TARGET * (1.0 - min(1.0, k / BACK_STEPS)))
            if k >= BACK_STEPS:
                m["phase"], m["k"] = "settle", 0
        elif ph == "settle" and k >= SETTLE_STEPS:
            if m["armed_ok"] and not m["touched"] and m["contacted"]:
                m["phase"] = "done"
            else:
                m["phase"], m["quiet"] = None, 0

        abs_deflection = abs(deg - m["base_deg"])
        m["fold_max"] = max(m["fold_max"], abs_deflection)
        if abs_deflection > 3.0 and not m["failed"]:
            m["failed"] = True

    if m["n"] % 20 == 0 or (m["phase"] == "done" and m["k"] == 1) or (m["n"] % 10 == 0 and (deg < 35.0 or qr_d < 79.0)):
        tc = _tcps()
        ltcp = [round(float(v), 4) for v in tc[0]] if len(tc) > 0 else []
        rtcp = [round(float(v), 4) for v in tc[1]] if len(tc) > 1 else []
        spt = [round(float(v), 4) for v in _stem_pt(R_GRIP, deg)]
        print(f"[SCOOTER step={m['n']:3d}] deg={deg:.2f} pin_q={pin_q:.4f} qr_d={qr_d:.1f} push_q={push_q:.4f} ph={m['phase']} q={m['quiet']} fail={m['failed']} clear={clear_of(objects)} ltcp={ltcp} rtcp={rtcp}", flush=True)


kit.on_step(_rules)


def score(objects) -> float:
    """Scoring:
    - 0.0 if stem folded > 3 deg under test push or never moved.
    - 0.25: stem lifted past 45 degrees.
    - 0.50: stem upright (within 2.0 deg) and latch pin seated (>= 5 mm).
    - 0.75: upright, latch pin seated, and quick-release lever closed.
    - 1.00: upright, latched, QR closed, and successfully passed the 12 N push test.
    Halved unless both grippers are 5 cm clear and parts are still.
    """
    if mk._EP.get("env") is None:
        _rules(objects)
    m = _mem()
    if m["failed"]:
        return 0.0

    deg = stem_deg(objects)
    pin_q = kit.q(objects, "pin_slide")
    qr_d = qr_deg(objects)
    push_q = kit.q(objects, "push")
    if DEBUG and (m["n"] % 25 == 0 or m["n"] >= 250):
        print(f"[DBG step {m['n']}] deg={deg:.2f} pin_q={pin_q:.4f} qr_d={qr_d:.1f} push_q={push_q:.4f} phase={m['phase']} quiet={m['quiet']} failed={m['failed']} clear={clear_of(objects)} still={all(m['still'].values())} parts_still={m['still']}")
    base = 0.0
    if deg <= 45.0:
        base = 0.25
        if is_upright(objects) and pin_seated(objects):
            base = 0.50
            if is_qr_closed(objects):
                base = 0.75
                # Full credit (1.00) strictly requires ALL 5 conditions simultaneously:
                # 1. Stem upright within 2.0 deg
                # 2. Latch pin fully seated (>= 5 mm)
                # 3. Quick-release clamp lever closed flat within 12 deg
                # 4. Both hands clear (>= 5 cm)
                # 5. Successfully passed the 12 N horizontal push test with <= 1.0 deg deflection
                if (is_upright(objects) and pin_seated(objects) and is_qr_closed(objects)
                        and clear_of(objects) and m["phase"] == "done" and m["contacted"]
                        and m["fold_max"] <= 1.0 and not m["failed"]):
                    base = 1.00

    ok = clear_of(objects) and all(m["still"].get(p, True) for p in PARTS)
    ret = round(base if ok else base / 2.0, 6)
    if DEBUG:
        print(f"[DBG SCORE] res={ret} base={base} ok={ok} deg={deg:.2f} pin_seated={pin_seated(objects)} qr_closed={is_qr_closed(objects)} phase={m['phase']} contacted={m['contacted']} fold_max={m['fold_max']:.2f} failed={m['failed']}")
    return ret


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    m = _mem()
    stages = (not is_upright(objects)) + (not pin_seated(objects)) + (not is_qr_closed(objects)) + (m["phase"] != "done")
    return float(stages + math.radians(max(0.0, stem_deg(objects))))


# -- Arm Compliance Feedforward ---------------------------------------------------------------------------------

FF = True


def _compliance(arm: str, pos, yaw: float = 0.0) -> np.ndarray:
    """The arm's TCP compliance (m/N, 3x3) at ``pos`` from shoulder and wrist gains."""
    from yam_arena import ik, station
    from yam_arena.constants import READY, SHOULDER, WRIST
    from yam_arena.demo import grip_rotation

    base = station.BASES[arm]
    q, _, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip_rotation(yaw)), np.array(READY, float),
                       base_pos=base, iters=200)
    p0 = ik.tcp_pose(q, base)[0]
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        jac[:, i] = (ik.tcp_pose(q + dq, base)[0] - p0) / 1e-6
    k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3, float)
    return jac @ np.diag(1.0 / k) @ jac.T


def _aim(arm: str, pos, force, yaw: float = 0.0) -> tuple[float, float, float]:
    """Where to command the TCP so that, loaded with ``force`` (N), it sits at ``pos``."""
    pos = np.asarray(pos, float)
    if not FF or not np.any(force):
        return tuple(float(v) for v in pos)
    return tuple(float(v) for v in pos + _compliance(arm, pos, yaw) @ np.asarray(force, float))


# -- Scripted Demonstration -------------------------------------------------------------------------------------

L_PARK = (0.180, YH + 0.220, TOP + 0.160)
R_PARK = (0.180, YH - 0.220, TOP + 0.160)
R_GRIP = 0.200  # Grip tube 20 cm from hinge (13 cm above QR collar, avoids collision)


QR_R_GRIP = 0.032  # Middle of 50 mm paddle handle


def _stem_pt(r, phi_deg):
    rad = math.radians(phi_deg)
    return H + np.array([r * math.sin(rad), 0.0, r * math.cos(rad)])


def _qr_pt(deg):
    rad = math.radians(deg)
    return np.array(QR_ANCHOR) + np.array([-QR_R_GRIP * math.sin(rad), 0.0, QR_R_GRIP * math.cos(rad)])


def demo():
    Lw, Rw = [], []
    YAW_TUBE = 0.0  # Align fingers symmetrically along y for full surface contact with tube box collider
    YAW_QR = math.pi / 2 - math.atan2(0.0239931, 0.0445119)  # 61.67 deg aligns fingers purely along x, zero y span

    def w(arm_list, arm_name, dur, pos, f, label, force=(0, 0, 0), yaw=0.0):
        aimed = _aim(arm_name, pos, force, yaw=yaw) if np.any(force) else pos
        arm_list.append(Waypoint(dur, tuple(float(v) for v in aimed), f, label, yaw))

    def t(arm_list):
        return sum(x.duration for x in arm_list)

    # 1. Left arm: approach folded stem at 90 deg, grip tube at r=0.12 m
    w(Lw, "left", 1.5, L_PARK, FINGER_OPEN, "start_l", yaw=YAW_TUBE)
    p_grip_l0 = _stem_pt(R_GRIP, START_DEG)
    w(Lw, "left", 1.2, (p_grip_l0[0], p_grip_l0[1], p_grip_l0[2] + 0.050), FINGER_OPEN, "hover_stem_l", yaw=YAW_TUBE)
    w(Lw, "left", 0.8, p_grip_l0, FINGER_OPEN, "at_stem_l", yaw=YAW_TUBE)
    w(Lw, "left", 0.8, p_grip_l0, FINGER_CLOSED, "grip_stem_l", yaw=YAW_TUBE)

    # Right arm: wait during initial approach
    w(Rw, "right", 1.5, R_PARK, FINGER_OPEN, "start_r")

    # 2. Left arm lifts stem smoothly along the circular arc from 90 deg to 0 deg (upright)
    target_upright = 5.0 if VARIANT == "unlatched" else -25.0
    n_lift = 40
    lift_dur = 10.0
    for k in range(1, n_lift + 1):
        phi = START_DEG + (target_upright - START_DEG) * (k / n_lift)
        pos = _stem_pt(R_GRIP, phi)
        rad = math.radians(max(0.0, phi))
        f_x = -5.0 * math.cos(rad) if phi < 45.0 else -2.0 * math.cos(rad)
        f_z = 2.5 * math.sin(rad)
        w(Lw, "left", lift_dur / n_lift, pos, FINGER_CLOSED, f"lift_{phi:.1f}", force=(f_x, 0.0, f_z), yaw=YAW_TUBE)

    # Stem reaches upright stop (t ~ 14.3 s). Latch pin snaps in.
    p_upright_snug = _stem_pt(R_GRIP, -25.0)
    w(Lw, "left", 0.5, p_upright_snug, FINGER_CLOSED, "settle_upright", force=(-2.0, 0.0, 0.0), yaw=YAW_TUBE)
    w(Lw, "left", 0.5, p_upright_snug, FINGER_OPEN, "release_stem_l", yaw=YAW_TUBE)
    w(Lw, "left", 0.8, (XH - 0.060, YH + 0.080, H[2] + 0.120), FINGER_CLOSED, "back_stem_l", yaw=YAW_TUBE)

    # 3. Left arm pushes QR clamp lever closed flat against bracket
    # In open position (80 deg), the lever paddle extends backwards to x = 0.199 m at y = -0.038, z = 0.841 m.
    # In closed position (0 deg), the lever paddle sits flat at x = 0.220 m at y = -0.038, z = 0.859 m.
    # Aligning the gripper at y = -0.028 centers the finger pads squarely against the paddle face without glancing off.
    p_qr_behind = (0.150, -0.028, 0.840)
    p_qr_push1 = (0.190, -0.028, 0.848)
    p_qr_closed = (0.230, -0.028, 0.858)

    if VARIANT not in ("no_clamp", "unlatched"):
        w(Lw, "left", 1.0, p_qr_behind, FINGER_CLOSED, "behind_qr_l", yaw=YAW_TUBE)
        w(Lw, "left", 1.0, p_qr_push1, FINGER_CLOSED, "push1_qr_l", force=(5.0, 0.0, 0.5), yaw=YAW_TUBE)
        w(Lw, "left", 1.2, p_qr_closed, FINGER_CLOSED, "push_qr_closed_l", force=(10.0, 0.0, 0.5), yaw=YAW_TUBE)
        w(Lw, "left", 1.0, p_qr_closed, FINGER_CLOSED, "hold_qr_closed_l", force=(10.0, 0.0, 0.0), yaw=YAW_TUBE)
        w(Lw, "left", 0.8, (0.150, -0.028, 0.880), FINGER_OPEN, "back_qr_l", yaw=YAW_TUBE)

    w(Lw, "left", 1.5, L_PARK, FINGER_OPEN, "park_l", yaw=YAW_TUBE)

    # 4. Synchronize waiting for automated push test
    total_time = max(t(Lw), t(Rw))
    wait_time = 8.5
    w(Lw, "left", total_time + wait_time - t(Lw), L_PARK, FINGER_OPEN, "wait_push_l", yaw=YAW_TUBE)
    w(Rw, "right", total_time + wait_time - t(Rw), R_PARK, FINGER_OPEN, "wait_push_r")

    return {"left": Lw, "right": Rw}


# -- Offline Self-Test ------------------------------------------------------------------------------------------

def fake_state(phi_deg, pin_m, qr_d, push_d=0.0):
    """Synthetic ObjectState dict for scoring tests."""
    return kit.fake_states(
        q={"stem_hinge": phi_deg, "pin_slide": pin_m, "qr_hinge": qr_d, "push": push_d},
        deg=True,
    )


def self_test():
    """Offline unit tests for all scoring branches, negative controls, and anti-cheat gates."""
    global _TEST_TCPS
    mk.new_episode()
    m = _mem()

    def run_plan(plan):
        mk.new_episode()
        for phi, pin, qr, push, tc in plan:
            global _TEST_TCPS
            _TEST_TCPS = tc
            st = fake_state(phi, pin, qr, push)
            sc = score(st)
        return sc

    # 1. Start position: stem folded flat (90 deg), unlatched -> 0.0
    mk.new_episode()
    st0 = fake_state(START_DEG, 0.0, QR_OPEN_DEG)
    assert score(st0) == 0.0, f"start score {score(st0)} != 0.0"

    # 2. Lifted past 45 deg -> 0.25
    mk.new_episode()
    st_lift = fake_state(40.0, 0.0, QR_OPEN_DEG)
    assert score(st_lift) == 0.25, f"lifted score {score(st_lift)} != 0.25"

    # 3. Stem upright, latch pin seated (>= 5 mm), QR clamp open -> 0.50
    mk.new_episode()
    st_upright = fake_state(UPRIGHT_DEG, PIN_STROKE, QR_OPEN_DEG)
    assert score(st_upright) == 0.50, f"upright score {score(st_upright)} != 0.50"

    # 4. Upright, latched, QR clamp closed flat -> 0.75
    mk.new_episode()
    st_clamped = fake_state(UPRIGHT_DEG, PIN_STROKE, QR_CLOSED_DEG)
    assert score(st_clamped) == 0.75, f"clamped score {score(st_clamped)} != 0.75"

    # 5. Full sequence through automated push test with 12 N load -> 1.00
    plan_full = []
    plan_full.append((START_DEG, 0.0, QR_OPEN_DEG, 0.0, [L_PARK, R_PARK]))
    plan_full.append((40.0, 0.0, QR_OPEN_DEG, 0.0, [L_PARK, R_PARK]))
    for _ in range(QUIET_STEPS + 5):
        plan_full.append((UPRIGHT_DEG, PIN_STROKE, QR_CLOSED_DEG, 0.0, [L_PARK, R_PARK]))
    # Push test ramps, contacts pusher (push=0.435 m >= 0.350 m), minimal deflection (0.3 deg <= 1.0 deg)
    for _ in range(RAMP_STEPS + HOLD_STEPS):
        plan_full.append((0.3, PIN_STROKE, QR_CLOSED_DEG, PUSH_TARGET, [L_PARK, R_PARK]))
    for _ in range(BACK_STEPS + SETTLE_STEPS + 5):
        plan_full.append((0.1, PIN_STROKE, QR_CLOSED_DEG, 0.0, [L_PARK, R_PARK]))

    sc_full = run_plan(plan_full)
    assert sc_full == 1.00, f"full pass score {sc_full} != 1.00"

    # 6. Negative control: unlatched stem collapses under push test (> 3 deg deflection) -> 0.0
    plan_unlatched = []
    plan_unlatched.append((START_DEG, 0.0, QR_OPEN_DEG, 0.0, [L_PARK, R_PARK]))
    for _ in range(QUIET_STEPS + 5):
        plan_unlatched.append((UPRIGHT_DEG, 0.0, QR_OPEN_DEG, 0.0, [L_PARK, R_PARK]))
    for _ in range(RAMP_STEPS + HOLD_STEPS):
        plan_unlatched.append((8.0, 0.0, QR_OPEN_DEG, PUSH_TARGET, [L_PARK, R_PARK]))
    for _ in range(BACK_STEPS + SETTLE_STEPS + 5):
        plan_unlatched.append((25.0, 0.0, QR_OPEN_DEG, 0.0, [L_PARK, R_PARK]))

    sc_unlatched = run_plan(plan_unlatched)
    assert sc_unlatched == 0.0, f"unlatched score {sc_unlatched} != 0.0"

    # 7. Gripper inside safety margin (not clear) halves score
    mk.new_episode()
    st_near = fake_state(UPRIGHT_DEG, PIN_STROKE, QR_CLOSED_DEG)
    _TEST_TCPS = [H + np.array([0, 0, 0.02])]  # 2 cm from hinge
    sc_near = score(st_near)
    assert sc_near == round(0.75 / 2.0, 6), f"hand near score {sc_near} != 0.375"
    _TEST_TCPS = None

    print("[unfold-kick-scooter-stem] offline self-test: 7/7 checks passed cleanly.")


if __name__ == "__main__":
    self_test()
