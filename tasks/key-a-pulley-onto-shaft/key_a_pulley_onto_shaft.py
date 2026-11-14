"""Fitting a keyed pulley to a shaft on an assembly bench. A 20 mm steel shaft is held horizontally on a support
block, its axis 40 mm above the bench and pointing at the robot, with a shoulder collar at the back. The top of the
shaft has a keyseat 5 mm wide, 25.6 mm long, by the shoulder. A 5 x 5 x 25 mm parallel key lies loose on the bench.
A 60 mm aluminium pulley (12 mm wide rim, 36 mm boss) is started on the free end of the shaft; its bore has a
matching keyway, turned 45 deg away from the key's line. A set screw in the boss sits right over the keyway (here a
10 mm knob with 3 detent clicks, 25 deg apart).

The job: press the key into the keyseat (an interference fit: it needs about 10 N), turn the pulley so its keyway
lines up with the key, slide it over the key up to the shoulder, and turn the set screw in three clicks.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/key-a-pulley-onto-shaft/key_a_pulley_onto_shaft.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1800 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL = (0.62, 0.63, 0.66)
KEYC = (0.30, 0.30, 0.32)
ALU = (0.80, 0.80, 0.82)
DARK = (0.14, 0.14, 0.15)
BLOCK = (0.20, 0.30, 0.55)
SCREW = (0.10, 0.10, 0.11)
MAT = (0.35, 0.50, 0.38)

# -- shaft (axis along x, y = 0) ---------------------------------------------------------------------------------
AXZ = TOP + 0.040  # shaft centreline
R_SHAFT = 0.010
X_END = 0.205  # free end
X_SH = 0.335  # shoulder face
X_CL = 0.375  # end over the support block
KEY_L, KEY_W = 0.025, 0.005
KEY_XC = X_SH - 0.015  # keyseat centre
SLOT_L, SLOT_HW = 0.0256, 0.0027  # keyseat (sim fit: 0.3 mm long, 0.4 mm wide clearance)
SLOT0, SLOT1 = KEY_XC - SLOT_L / 2, KEY_XC + SLOT_L / 2
RMAX = 0.0104  # the stepped section stays inside this radius
SLABS = [(-0.010, -0.0085), (-0.0085, -0.007), (-0.007, -0.005), (-0.005, 0.005), (0.005, 0.007), (0.007, 0.0085),
         (0.0085, 0.010)]


def _shaft_boxes():
    out = []
    for x0, x1, keyed in ((X_END, SLOT0, False), (SLOT0, SLOT1, True), (SLOT1, X_CL, False)):
        L, xc = x1 - x0, (x0 + x1) / 2
        for a, b in SLABS:
            w = 2 * math.sqrt(RMAX ** 2 - max(a * a, b * b))
            zc = AXZ + (a + b) / 2
            if keyed and a >= 0.0085:
                continue  # the keyseat opens the top
            if keyed and a >= 0.005:  # split around the slot
                hw = w / 2
                side = hw - SLOT_HW
                for s in (1, -1):
                    out.append(mk.box((L, side, b - a), at=(xc, s * (SLOT_HW + side / 2), zc), color=STEEL))
                continue
            out.append(mk.box((L, w, b - a), at=(xc, 0, zc), color=STEEL))
    # shoulder collar, 30 mm, 8 mm long
    side = 2 * 0.015 / math.sqrt(2) * 1.08
    for i in range(3):
        out.append(mk.box((0.008, side, side), at=(X_SH + 0.0045, 0, AXZ), color=STEEL,
                          quat=mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi / 2 * i / 3))))
    return out


kit = mk.Kit()
kit.part("shaft", (0, 0, 0), _shaft_boxes(), mass=None, color=STEEL)

# Interference fit: the keyseat floor is a pad that the key pushes 1 mm down against 10 N of Coulomb friction.
SEAT_TOP0 = AXZ + 0.008  # pad top at the start (the key enters 0.5 mm below the slot walls' tops)
SEAT_TRAVEL = 0.001
PRESS_N = 10.0  # interference: the key needs this much push to go home
kit.part("seat_pad", (KEY_XC, 0, SEAT_TOP0 - 0.001), [mk.box((SLOT_L - 0.0002, 2 * SLOT_HW - 0.0001, 0.002))],
         mass=0.02, color=STEEL)
kit.slider("seat_press", "shaft", "seat_pad", axis=(0, 0, -1), lower=0.0, upper=SEAT_TRAVEL, friction=PRESS_N,
           damping=1.0)

# The key: loose on the bench mat, long side along x.
KEY_START = (0.270, -0.130, TOP + 0.001 + 0.0025)
_c45 = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi / 4))
_KEY = [mk.box((KEY_L, KEY_W, 0.004), at=(0, 0, 0.0005)), mk.box((KEY_L, 0.003, 0.001), at=(0, 0, -0.002)),
        *[mk.box((KEY_L, 0.001414, 0.001414), at=(0, s * 0.0015, -0.0015), quat=_c45) for s in (1, -1)]]  # 1 mm chamfers
kit.part("key", KEY_START, _KEY, mass=0.02, color=KEYC)
KEY_SEATED = np.array([KEY_XC, 0.0, SEAT_TOP0 - SEAT_TRAVEL + 0.0025])  # bottom 7 mm, top 12 mm above the axis
_T_seat = np.eye(4)
_T_seat[:3, 3] = KEY_SEATED
kit.catch("key_seat", "key", _T_seat)  # the interference fit holds it once fully in

# -- pulley ---------------------------------------------------------------------------------------------------
HUB_L, BOSS_L = 0.036, 0.024  # boss toward the robot (-x), rim at the shoulder side
R_BORE, R_KEYWAY, R_BOSS, R_RIM = 0.0108, 0.0126, 0.018, 0.030
KW_HW = 0.0042  # keyway half-width (key 2.5): about +-8 deg of phase
HUB_X0 = X_END - 0.004  # hub's front face at the start
TRAVEL = (X_SH - HUB_L) - HUB_X0  # to the shoulder
KNOB_DX = 0.010  # set screw, from the hub's front face
HUB_START_DEG = 45.0


def _ring(x0, x1, ri, ro, n, color, skip_deg=0.0):
    out = []
    w = 2 * ro * math.tan(math.pi / n)
    for i in range(n):
        a = 2 * math.pi * i / n
        if skip_deg and abs((math.degrees(a) + 180) % 360 - 180) < skip_deg:
            continue
        r = (ri + ro) / 2
        out.append(mk.box((x1 - x0, w, ro - ri), at=((x0 + x1) / 2, -r * math.sin(a), r * math.cos(a)), color=color,
                          quat=mk.mat_to_quat(mk.axis_angle((1, 0, 0), a))))
    return out


def _hub_boxes():
    b = _ring(0, HUB_L, R_BORE, R_KEYWAY, 24, ALU, skip_deg=20.0)  # bore, keyway at the top (phase 0)
    for s in (1, -1):  # keyway walls
        b.append(mk.box((HUB_L, 0.002, 0.0020), at=(HUB_L / 2, s * (KW_HW + 0.001), 0.0116), color=ALU))
    b += _ring(0, BOSS_L, R_KEYWAY, R_BOSS, 16, ALU)
    b += _ring(BOSS_L, HUB_L, R_KEYWAY, R_RIM, 20, ALU)
    b.append(mk.box((0.001, 0.004, 0.003), at=(-0.0005, 0, R_BOSS - 0.0015), color=DARK))  # keyway mark
    return b


# The pulley is one body on a cylindrical joint about the shaft axis: it slides 0..TRAVEL and turns freely. mechkit
# declares it as a slider (so the readouts and the self-test work) and the joint is re-authored as a D6 (x free
# within the stroke, rotation about x free, the rest locked) right after mechkit builds the scene; see _author_d6.
_R45 = mk.axis_angle((1, 0, 0), math.radians(HUB_START_DEG))
HUB_Q0 = mk.mat_to_quat(_R45)
kit.part("hub", (HUB_X0, 0, AXZ), _hub_boxes(), mass=0.15, color=ALU, quat=HUB_Q0)
kit.slider("hub_slide", "shaft", "hub", axis=(1, 0, 0), lower=0.0, upper=TRAVEL)
SLIDE_FRICTION_N = 2.0  # a slip fit
TURN_FRICTION_NM = 0.01

# Set screw stand-in: a 10 mm knob, 6 mm tall, on the boss over the keyway; 3 detent clicks of 25 deg.
KNOB_D, KNOB_H = 0.010, 0.006
CLICK_DEG = 25.0
KNOB_LOCAL = np.array([KNOB_DX, 0.0, R_BOSS])
KNOB_AT = tuple(np.array([HUB_X0, 0, AXZ]) + _R45 @ KNOB_LOCAL)  # at the start
KNOB_TOP0 = (HUB_X0 + KNOB_DX, 0.0, AXZ + R_BOSS)  # with the keyway at the top
_knob = [mk.box((0.0098, 0.0098, KNOB_H), at=(0, 0, KNOB_H / 2), color=SCREW,
                quat=mk.mat_to_quat(mk.axis_angle((0, 0, 1), math.pi / 2 * i / 3))) for i in range(3)]
_knob.append(mk.box((0.004, 0.0012, 0.0005), at=(0.002, 0, KNOB_H + 0.00025), color=(0.85, 0.15, 0.12)))
kit.part("screw", KNOB_AT, _knob, mass=0.02, color=SCREW, quat=HUB_Q0)
kit.hinge("screw_turn", "hub", "screw", anchor=KNOB_AT, axis=tuple(_R45 @ np.array([0, 0, 1.0])), lower_deg=-2.0,
          upper_deg=3 * CLICK_DEG + 5.0, stiffness=0.05, damping=0.002)

_T_lock = np.eye(4)
_T_lock[:3, 3] = (HUB_X0 + TRAVEL, 0, AXZ)
kit.catch("hub_lock", "hub", _T_lock)  # the set screw bites: the pulley is fixed

_STICK_M, _STICK_DEG = 0.0005, 0.5
_author0 = mk._author_all


def _author_d6(stage, env_path):
    """Replace the pulley's prismatic joint with a D6 of the same frames (sim only, at scene build)."""
    _author0(stage, env_path)
    path = mk._SIM["joint_paths"].get((kit.id, "hub_slide"))
    if not path:
        return
    from pxr import UsdPhysics

    old = UsdPhysics.Joint(stage.GetPrimAtPath(path))
    b0, b1 = old.GetBody0Rel().GetTargets(), old.GetBody1Rel().GetTargets()
    lp0, lr0 = old.GetLocalPos0Attr().Get(), old.GetLocalRot0Attr().Get()
    lp1, lr1 = old.GetLocalPos1Attr().Get(), old.GetLocalRot1Attr().Get()
    stage.RemovePrim(path)
    j = UsdPhysics.Joint.Define(stage, path)
    if b0:
        j.CreateBody0Rel().SetTargets(b0)
    j.CreateBody1Rel().SetTargets(b1)
    j.CreateLocalPos0Attr().Set(lp0)
    j.CreateLocalRot0Attr().Set(lr0)
    j.CreateLocalPos1Attr().Set(lp1)
    j.CreateLocalRot1Attr().Set(lr1)
    j.CreateCollisionEnabledAttr().Set(False)
    j.CreateExcludeFromArticulationAttr().Set(True)
    prim = j.GetPrim()
    for dof in ("transY", "transZ", "rotY", "rotZ"):
        lim = UsdPhysics.LimitAPI.Apply(prim, dof)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)  # low > high: locked
    lim = UsdPhysics.LimitAPI.Apply(prim, "transX")
    lim.CreateLowAttr().Set(0.0)
    lim.CreateHighAttr().Set(float(TRAVEL))
    for dof, k, d in (("transX", SLIDE_FRICTION_N / _STICK_M, 2.0),
                      ("rotX", TURN_FRICTION_NM / math.radians(_STICK_DEG) * math.pi / 180, 0.0005)):
        dr = UsdPhysics.DriveAPI.Apply(prim, dof)
        dr.CreateTypeAttr().Set("force")
        dr.CreateStiffnessAttr().Set(float(k))
        dr.CreateDampingAttr().Set(float(d))
        dr.CreateTargetPositionAttr().Set(0.0)
        dr.CreateTargetVelocityAttr().Set(0.0)
    print(f"[keypulley] pulley joint {path} re-authored as a D6", flush=True)


mk._author_all = _author_d6


def _unwrap_phase(p):
    prev = mk.MEMORY.get("phase_unwrapped", HUB_START_DEG)
    v = prev + (p - prev + 180.0) % 360.0 - 180.0
    mk.MEMORY["phase_unwrapped"] = v
    return v


def _stick(key, q, lim, dof, scale=1.0):
    """Coulomb friction on a D6 axis: the drive's anchor follows the joint once it has slipped past `lim`."""
    M = mk.MEMORY
    s = M.get(key, 0.0)
    new = min(max(s, q - lim), q + lim)
    if new != s or key not in M:
        M[key] = new
        path = mk._SIM["joint_paths"].get((kit.id, "hub_slide"))
        if path and mk._SIM["stage"] is not None:
            from pxr import UsdPhysics

            UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), dof).GetTargetPositionAttr().Set(float(new * scale))


FIXTURES = [
    Box("v_block", pos=(0.360, 0.0, (TOP + AXZ - R_SHAFT) / 2), size=(0.030, 0.050, AXZ - R_SHAFT - TOP), color=BLOCK),
    Box("bench_mat", pos=(0.260, -0.130, TOP + 0.0005), size=(0.080, 0.080, 0.001), color=MAT),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- state ------------------------------------------------------------------------------------------------------


def _tilt(q) -> float:
    z = mk.quat_to_mat(q)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def _key_cock(q) -> float:
    """Angle of the key's long side from the shaft axis (its square section may lie on any face)."""
    x = mk.quat_to_mat(q)[:, 0]
    return math.degrees(math.acos(min(1.0, abs(float(x[0])))))


def key_near_seat(objects, tol_xy=0.0006, tol_z=0.0004, tol_deg=3.0) -> bool:
    p, q = kit.pose(objects, "key")
    d = p - KEY_SEATED
    return abs(float(d[0])) <= tol_xy and abs(float(d[1])) <= tol_xy and abs(float(d[2])) <= tol_z and _key_cock(q) <= tol_deg


def key_in_slot(objects) -> bool:
    p, _ = kit.pose(objects, "key")
    return abs(float(p[1])) < 0.004 and SLOT0 - 0.004 < float(p[0]) < SLOT1 + 0.004 and float(p[2]) < AXZ + 0.016


def hub_lead(objects) -> float:
    """x of the pulley's rear face (the one that meets the key first)."""
    return HUB_X0 + HUB_L + kit.q(objects, "hub_slide")


def hub_home(objects) -> bool:
    return kit.q(objects, "hub_slide") >= TRAVEL - 0.0015


def hub_phase(objects) -> float:
    """Angle (deg) of the keyway from the top of the bore, about the shaft axis."""
    _, q = kit.pose(objects, "hub")
    r = mk.quat_to_mat(q)
    return math.degrees(math.atan2(float(r[2, 1]), float(r[1, 1])))


def screw_deg(objects) -> float:
    return kit.q_deg(objects, "screw_turn")


def _mechanism_rule(objects):
    M = mk.MEMORY
    # pulley slip-fit friction (sliding and turning)
    rot = _unwrap_phase(hub_phase(objects)) - HUB_START_DEG
    _stick("stick_slide", kit.q(objects, "hub_slide"), _STICK_M, "transX")
    _stick("stick_rot", rot, _STICK_DEG, "rotX")
    # set screw detents
    s = screw_deg(objects)
    det = min(3 * CLICK_DEG, max(0.0, round(s / CLICK_DEG) * CLICK_DEG))
    if M.get("detent") != det:
        M["detent"] = det
        kit.drive("screw_turn", det)
    if s > CLICK_DEG / 2 and not hub_home(objects):
        M["screw_early"] = True  # tightened off the shoulder: it marks the shaft in the wrong place
    # the key seats when pressed fully home
    if not M.get("seated") and kit.q(objects, "seat_press") >= SEAT_TRAVEL - 0.0001 and key_near_seat(objects):
        M["seated"] = True
        kit.engage("key_seat")
    kp, kq = kit.pose(objects, "key")
    lead = hub_lead(objects)
    kmin = float(kp[0]) - KEY_L / 2
    if key_in_slot(objects) and not M.get("seated") and lead > kmin + 0.002:
        M["jam"] = True  # the pulley forced over a key that is not home
    if key_in_slot(objects) and _key_cock(kq) > 5.0 and lead > kmin:
        M["jam"] = True
    if float(kp[2]) < TOP - 0.020:
        M["dropped"] = True
    if (not M.get("locked") and M.get("seated") and hub_home(objects) and s >= 3 * CLICK_DEG - 5.0
            and not M.get("screw_early") and abs(hub_phase(objects)) < 4.0):
        M["locked"] = True
        kit.engage("hub_lock")
    if os.environ.get("KEYPULLEY_DEBUG") and mk._EP["step"] % int(os.environ.get("KEYPULLEY_EVERY", "20")) == 0:
        print(f"[keypulley] step {mk._EP['step']} tcp {[np.round(t, 4).tolist() for t in mk.tcps()]} key {np.round(kp, 4)} cock {_key_cock(kq):.1f} press "
              f"{kit.q(objects, 'seat_press') * 1000:.2f} mm slide {kit.q(objects, 'hub_slide') * 1000:.1f} mm "
              f"phase {hub_phase(objects):.1f} screw {s:.1f} "
              f"flags { {k: v for k, v in M.items() if k in ('seated', 'jam', 'dropped', 'screw_early', 'locked')} }"
              f" score {score(objects)}", flush=True)


kit.on_step(_mechanism_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _released(st, margin) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects) -> bool:
    """Latched: the key fell off the bench, or the pulley was forced over a key that was not fully home."""
    M = mk.MEMORY
    return bool(M.get("jam") or M.get("dropped"))


def key_ok(objects) -> bool:
    return (bool(mk.MEMORY.get("seated")) and key_near_seat(objects, 0.0008, 0.0006, 3.0)
            and _released(objects["key"], 0.010))


def hub_ok(objects) -> bool:
    return (key_ok(objects) and hub_home(objects) and _released(objects["hub"], 0.010)
            and _released(objects["screw"], 0.015) and objects["hub"].resting(max_speed=0.01, max_spin=0.2))


def screw_ok(objects) -> bool:
    return (hub_ok(objects) and not mk.MEMORY.get("screw_early") and screw_deg(objects) >= 3 * CLICK_DEG - 5.0
            and objects["screw"].resting(max_speed=0.01, max_spin=0.2))


def score(objects) -> float:
    """0 if failed(). Key fully seated and let go 0.3; with it, pulley at the shoulder (so over the key), let go and
    still 0.4; with both, set screw at 3 clicks (and not turned before the pulley reached the shoulder) 0.3."""
    if failed(objects):
        return 0.0
    s = 0.0
    if key_ok(objects):
        s += 0.3
        if hub_ok(objects):
            s += 0.4
            if screw_ok(objects):
                s += 0.3
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    p, _ = kit.pose(objects, "key")
    return float(np.linalg.norm(p - KEY_SEATED) + (TRAVEL - kit.q(objects, "hub_slide"))
                 + abs(3 * CLICK_DEG - screw_deg(objects)) / 1000.0)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm: picks the key off the mat, lowers it into the keyseat, lets go and presses it home with closed fingers.
# Left arm: pushes the set screw round with its closed fingertips, turning the pulley until the keyway is near the
# top, pinches the screw (which centres the keyway over the key), slides the pulley over the key to the shoulder,
# lets go, takes a new grip with the wrist at -40 deg and turns the screw three clicks.

TIP = 0.013
PARK_L = (0.18, 0.24, TOP + 0.09)
PARK_R = (0.18, -0.26, TOP + 0.09)
KEY_OPEN = -0.010
KEY_FIX = (0.0009, 0.0008)  # where the key sits in the closed grip (measured in sim), corrected at the keyseat
KNOB_OPEN = -0.012
WIDE_OPEN = -0.022
PINCH_DY = 0.002  # the left arm sits ~2 mm toward -y here (measured in sim)  # wide enough for a set screw up to ~25 deg off the top


def _dur(plan):
    return sum(w.duration for w in plan)


def _wait(plan, until, label="wait"):
    t = _dur(plan)
    if until > t:
        last = plan[-1]
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))
    return plan


def _key_plan(target="seat"):
    kx, ky, _ = KEY_START
    grip_z = TOP + 0.001 + 0.0025 + TIP  # pad tips 2.5 mm above the mat: the upper half of the key
    hi = AXZ + 0.050
    plan = [Waypoint(2.0, (kx, ky, TOP + 0.06), KEY_OPEN, "over the key", 0.0),
            Waypoint(1.5, (kx, ky, grip_z), KEY_OPEN, "down round the key", 0.0),
            Waypoint(0.8, (kx, ky, grip_z), FINGER_CLOSED, "pinch the key", 0.0),
            Waypoint(1.5, (kx, ky, hi), FINGER_CLOSED, "lift", 0.0)]
    if target == "hub":  # set it on top of the pulley's boss instead
        x, z = HUB_X0 + 0.004 + KEY_L / 2, AXZ + R_BOSS + 0.0008 + 0.0025 + TIP
        return plan + [Waypoint(2.5, (x, 0.0, z + 0.02), FINGER_CLOSED, "over the pulley", 0.0),
                       Waypoint(1.2, (x, 0.0, z), FINGER_CLOSED, "down onto the boss", 0.0),
                       Waypoint(0.6, (x, 0.0, z), KEY_OPEN, "let go", 0.0),
                       Waypoint(1.0, (x, 0.0, hi + 0.01), KEY_OPEN, "up", 0.0),
                       Waypoint(2.0, PARK_R, FINGER_CLOSED, "park", 0.0)]
    kb = 0.0025 + TIP  # TCP above the key's underside while it is pinched
    z_in = SEAT_TOP0 + 0.0006 + kb  # underside 0.6 mm over the pad: the chamfers find the slot
    z_home = KEY_SEATED[2] - 0.0025 - 0.006 + kb  # 6 mm past home: the arm leans on it (the pads stop on the shaft)
    px, py = KEY_XC + KEY_FIX[0], KEY_FIX[1]
    return plan + [Waypoint(2.5, (px, py, z_in + 0.02), FINGER_CLOSED, "over the keyseat", 0.0),
                   Waypoint(1.2, (px, py, z_in), FINGER_CLOSED, "down to the keyseat", 0.0),
                   Waypoint(2.0, (px, py, z_home), FINGER_CLOSED, "press the key home", 0.0),
                   Waypoint(0.8, (px, py, z_home), FINGER_CLOSED, "hold", 0.0),
                   Waypoint(0.6, (px, py, z_home), KEY_OPEN, "let go", 0.0),
                   Waypoint(1.0, (px, py, hi), KEY_OPEN, "up", 0.0),
                   Waypoint(2.0, PARK_R, FINGER_CLOSED, "park", 0.0)]


def _rim_point(phi_deg, press=0.002):
    """TCP with the closed pad tips `press` into the top of the rim at angle phi (deg, + toward -y)."""
    f = math.radians(phi_deg)
    r = R_RIM - press
    return (HUB_X0 + BOSS_L + (HUB_L - BOSS_L) / 2, -r * math.sin(f), AXZ + r * math.cos(f) + TIP)


def _hub_plan(slide=True, screw=True):
    x = KNOB_TOP0[0]
    a0, a1 = 30.0, -30.0  # the fingertips drag the rim top over 60 deg (they slip some): the keyway comes round to the top
    up0 = _rim_point(a0, -0.012)
    plan = [Waypoint(2.0, up0, FINGER_CLOSED, "over the rim", 0.0),
            Waypoint(1.2, _rim_point(a0, 0.005), FINGER_CLOSED, "fingertips onto the rim", 0.0)]
    n = 8
    for i in range(1, n + 1):
        plan.append(Waypoint(0.45, _rim_point(a0 + (a1 - a0) * i / n, 0.005), FINGER_CLOSED, "drag the rim round", 0.0))
    end = plan[-1].pos
    pz = AXZ + R_BOSS + 0.001 + TIP  # pad tips 1 mm above the boss
    zp = AXZ + R_SHAFT + 0.0015 + TIP  # pad tips just over the shaft

    def align(xk):  # pinch the set screw: the pads centre it, which puts the keyway on top; then let go
        return [Waypoint(1.0, (xk, PINCH_DY, pz + 0.025), WIDE_OPEN, "over the set screw", 0.0),
                Waypoint(1.2, (xk, PINCH_DY, pz), WIDE_OPEN, "down round the set screw", 0.0),
                Waypoint(0.8, (xk, PINCH_DY, pz), FINGER_CLOSED, "pinch it (lines up the keyway)", 0.0),
                Waypoint(0.6, (xk, PINCH_DY, pz), KNOB_OPEN, "let go", 0.0),
                Waypoint(0.8, (xk, 0.0, pz + 0.03), FINGER_CLOSED, "up", 0.0)]

    def push(d0, d1, t):  # closed fingertips against the boss's front face, from slide d0 to d1
        f0 = HUB_X0 + d0 - 0.025
        f1 = HUB_X0 + d1 - 0.0095
        return [Waypoint(1.2, (f0, 0.0, pz + 0.03), FINGER_CLOSED, "in front of the pulley", 0.0),
                Waypoint(1.0, (f0, 0.0, zp), FINGER_CLOSED, "down", 0.0),
                Waypoint(t, (f1, 0.0, zp), FINGER_CLOSED, "push the pulley along the shaft", 0.0),
                Waypoint(1.0, (f1, 0.0, zp), FINGER_CLOSED, "hold", 0.0),
                Waypoint(0.6, (f1 - 0.012, 0.0, zp), FINGER_CLOSED, "back off", 0.0),
                Waypoint(0.8, (f1 - 0.012, 0.0, pz + 0.03), FINGER_CLOSED, "up", 0.0)]

    plan += [Waypoint(0.6, (end[0], end[1], end[2] + 0.012), FINGER_CLOSED, "lift off the rim", 0.0)]
    plan += align(x)
    if slide:  # push it over the key to the shoulder
        plan += push(0.0, TRAVEL + 0.032, 5.5)  # 32 mm past the shoulder: the arm gives ~23 mm under this load
        x = x + TRAVEL
    plan += [Waypoint(1.0, (x, 0.0, pz + 0.025), KNOB_OPEN, "over the set screw", 0.0),
             Waypoint(1.0, (x, 0.0, pz), KNOB_OPEN, "down round the set screw", 0.0)]
    if screw:
        y0, y1 = math.radians(-40.0), math.radians(40.0)
        plan += [Waypoint(0.6, (x, 0.0, pz), KNOB_OPEN, "let go", 0.0),
                 Waypoint(0.8, (x, 0.0, pz), KNOB_OPEN, "turn the wrist back", y0),
                 Waypoint(0.6, (x, 0.0, pz), FINGER_CLOSED, "grip the set screw", y0),
                 Waypoint(2.0, (x, 0.0, pz), FINGER_CLOSED, "turn it three clicks", y1),
                 Waypoint(0.6, (x, 0.0, pz), KNOB_OPEN, "let go", y1)]
    else:
        plan += [Waypoint(0.6, (x, 0.0, pz), KNOB_OPEN, "let go", 0.0)]
    last = plan[-1]
    return plan + [Waypoint(1.0, (x, 0.0, pz + 0.035), KNOB_OPEN, "up", last.yaw),
                   Waypoint(2.0, PARK_L, FINGER_CLOSED, "park", 0.0)]


def demo():
    # DEMO_VARIANT selects a degenerate check (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    idle_r = [Waypoint(2.0, PARK_R, FINGER_CLOSED, "wait", 0.0)]
    idle_l = [Waypoint(2.0, PARK_L, FINGER_CLOSED, "wait", 0.0)]
    if v == "no_key":  # pulley slid home and screwed with no key
        return {"left": _hub_plan(), "right": idle_r}
    if v == "key_on_hub":  # the key set on top of the pulley, nothing else
        return {"left": idle_l, "right": _key_plan("hub")}
    if v == "screw_only":  # the set screw turned with the pulley still at the shaft end
        return {"left": _hub_plan(slide=False), "right": idle_r}
    right = _key_plan()
    left = _wait([Waypoint(1.0, PARK_L, FINGER_CLOSED, "wait", 0.0)], _dur(right) + 0.3) + _hub_plan()
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = mk.mat_to_quat(np.eye(3))
    seated = (KEY_SEATED.copy(), up)
    in_slot = (KEY_SEATED + np.array([0, 0, SEAT_TRAVEL]), up)
    on_hub = (np.array([HUB_X0 + 0.017, 0.0, AXZ + R_BOSS + 0.003]), up)
    home = {"hub_slide": TRAVEL}
    tight = {**home, "screw_turn": math.radians(75.0)}

    def st(q=None, key=seated, slide=None):
        q = dict(q or {})
        d = q.get("hub_slide", 0.0) if slide is None else slide
        hub = (np.array([HUB_X0 + d, 0, AXZ]), up)  # keyway at the top
        return kit.fake_states(q=q, moved={"key": key, "hub": hub})

    seat_mem = {"seated": True}
    key_tcp = [KEY_SEATED + np.array([0, 0, 0.0025 + 0.006])]
    knob_tcp = [np.array([KNOB_TOP0[0] + TRAVEL, 0, AXZ + R_BOSS + 0.012])]
    cases = {  # name: (states, tcps, MEMORY, score)
        "start": (kit.fake_states(), [], {}, 0.0),
        "key in the slot, not pressed": (st(key=in_slot), [], {}, 0.0),
        "key seated, held down": (st(), key_tcp, seat_mem, 0.0),
        "key seated, let go": (st(), [], seat_mem, 0.3),
        "key seated, pulley half way": (st({"hub_slide": TRAVEL / 2}), [], seat_mem, 0.3),
        "pulley home, still pinched": (st(home), knob_tcp, seat_mem, 0.3),
        "pulley home, let go": (st(home), [], seat_mem, 0.7),
        "pulley home, screw 2 clicks": (st({**home, "screw_turn": math.radians(50.0)}), [], seat_mem, 0.7),
        "pulley home, screw 3 clicks": (st(tight), [], seat_mem, 1.0),
        "screw turned before the shoulder": (st(tight), [], {**seat_mem, "screw_early": True}, 0.7),
        "pulley home and screwed, no key": (st(tight, key=(np.array(KEY_START), up)), [], {}, 0.0),
        "key on top of the pulley": (st(tight, key=on_hub), [], {}, 0.0),
        "forced over an unseated key": (st(tight), [], {**seat_mem, "jam": True}, 0.0),
        "key dropped": (st(tight), [], {**seat_mem, "dropped": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        s0 = kit.fake_states()
        assert key_in_slot(st(key=in_slot)) and not key_in_slot(s0)
        assert abs(hub_phase(s0) - HUB_START_DEG) < 1e-6 and abs(hub_phase(st(home))) < 1e-6
        assert abs(screw_deg(st(tight)) - 75.0) < 1e-6 and abs(screw_deg(s0)) < 1e-6
        assert abs(hub_lead(st(home)) - X_SH) < 1e-9
        # shaft section stays inside the bore; the seated key clears the keyway, an unseated one does not
        assert RMAX < R_BORE and KEY_SEATED[2] + 0.0025 - AXZ < R_KEYWAY - 0.0004
        assert KEY_SEATED[2] + 0.0025 + SEAT_TRAVEL - AXZ > R_KEYWAY
        assert KW_HW - KEY_W / 2 > 0.0006
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
