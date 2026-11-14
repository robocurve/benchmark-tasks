"""On the station table, in front of the arms, a folding banquet table lies upside down: its moulded top rests on the
bench (the end that carries one leg pair is modelled, from 10 cm before the leg's hinge to the bench's far edge) and
the leg pair is folded flat against it, as a table is stored. The leg pair is a U of two aluminium tubes 60 cm apart,
70 cm long, hinged to brackets on the top's underside 10 cm from its end; folded, it lies along the top, away from the
robot, on two moulded rests. A folding brace runs from a bracket on the top to the leg's cross bar: two steel strips
riveted together at a knee. Folded, the brace's knee stands up between the leg's tubes; its upper strip passes through
a slot in the cross bar to the clevis pin under it. Swung right out (upright), the leg reaches its stop and the brace
is then almost straight (the knee 4 deg short of straight, on the side it folds to).

The lock is a steel ring that slides on the brace's lower strip. Slid up over the knee and onto the upper strip, its
walls stop the knee bending, so the brace becomes a strut and the leg cannot fold. The ring only passes the knee
when the brace is straight (a bent knee's upper strip meets the ring's mouth). Opening the leg alone does not lock
it: an unlocked brace is held only by its own weight near straight and the hinge friction, and any real side load
on the leg folds the knee and the leg with it.

Then the check: once the robot has let go and stood clear for 2 s, a padded rod pushes on one leg tube 40 cm up, as
the leg is pushed when the table is turned over and stood on its feet (about 6 N, 1.5 s, then it retracts). A
locked leg does not move. An unlocked leg folds.

The job: swing the leg up out of its folded position until it stands on its stop and the brace is straight, slide the
ring up over the knee, let go and stand clear.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/folding-table-leg-ring-lock/folding_table_leg_ring_lock.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges, sliders, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the mechanism's readout every 5 steps
VARIANT = "demo"  # sim smoke tests: "demo", "open" (leg opened, ring left), "half" (ring pushed only part way)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# Everything below lies in the vertical plane y = YC (the leg pair's centre line), except the leg tubes.
YC = -0.120
SLAB = 0.040  # the table top's thickness; it lies upside down, its underside facing up
TS = TOP + SLAB  # the underside (now the upper face)
TOP_W = 0.760
# -- the leg pair: hinged along y at H, upright in the reference pose, folds away from the robot (+x) -------------
XH = 0.140
HZ = 0.027  # the hinge axis above the underside (the brackets lift it so the folded leg's clevis clears the top)
H = np.array([XH, YC, TS + HZ])  # hinge axis
TOP_X = (XH - 0.100, 0.910)  # the modelled part of the top: its end 10 cm before the hinge, to the bench's edge
TUBE, TUBE_L, TUBE_Y = 0.025, 0.700, 0.300  # square tube, length, its offset from the centre line
LEG_KG = 0.60
LEG_FRICTION = 0.5  # N*m: the hinge's friction
R_B = 0.130  # the brace bar's height up the leg
P_REF = np.array([XH + 0.018, YC, H[2] + R_B])  # brace pin on the leg (clevis under the brace bar), leg upright
START_DEG = 90.0  # the leg starts folded flat against the top (stowed)
FOLD_STOP_DEG = 90.0  # its fold stop: lying flat along the top
SLOT = 0.004  # half-width of the cross bar's slot for the upper strip
# -- the brace: two steel strips (20 x 6 mm) from the top's bracket A to the leg's pin P, a knee between ---------------
A = np.array([XH + 0.200, YC, TS + HZ - 0.001])
BETA_S = 4.0  # knee bend (deg) with the leg upright on its stop
_AP0 = P_REF - A
L = float(np.linalg.norm(_AP0) / (2 * math.cos(math.radians(BETA_S / 2))))  # each strip, pin to pin
STRIP_H, STRIP_T = 0.020, 0.006
LINK_KG = 0.05
# -- the lock ring: a steel collar on the lower strip, 40 mm long, 2.5 mm play in the bending plane ----------------
RING_LEN, RING_WALL = 0.040, 0.004
RING_IN_H, RING_IN_T = STRIP_H + 0.005, STRIP_T + 0.004
RING_KG = 0.04
RING_FRICTION = 1.0  # N: it stays where it is slid
S0 = 0.032  # the ring's centre this far up the lower strip from A at the start (on its stop)
RING_TOP = L + 0.005  # its centre's highest point: the leading end 25 mm past the knee
LOCK_S = L - RING_LEN / 2 + 0.012  # locked: the ring's leading end at least 12 mm past the knee
# -- the side-load rig: a padded rod on a slide, pushing the right tube 40 cm up, toward folded (+x) ---------------
PUSH_R = 0.400
PUSH_TRAVEL = 0.140
PUSH_KG = 0.20
K_PUSH, FAR = 1.2, 5.0  # drive: target -5 m holds it on its rear stop with 6 N; +5 m pushes with 6 N
PAD = (0.024, 0.050, 0.050)
PAD_X = XH - TUBE / 2 - 0.003 - PAD[0] / 2  # on the robot's side of the upright tube
PAD_Z = H[2] + PUSH_R
ROD_DY = 0.120  # the pad's arm runs out past the table's side to the rod
PARTS = ("leg", "link_a", "link_b", "ring")

HDPE, ALU, STEEL = (0.86, 0.86, 0.83), (0.62, 0.64, 0.67), (0.25, 0.26, 0.28)
ZINC, RIG, RUBBER = (0.80, 0.78, 0.55), (0.20, 0.36, 0.62), (0.10, 0.10, 0.10)


def _rot_y(deg):
    """Rotation about +y by ``deg``."""
    return mk.axis_angle((0, 1, 0), math.radians(deg))


def _slope(v):
    return math.degrees(math.atan2(v[2], v[0]))


def loop(phi_deg):
    """The brace's pose for the leg folded ``phi_deg`` from upright (planar four-bar, knee bent up and away from the
    robot): pin P, knee K (world), the strips' slopes (deg), and the joint values for the kit's tree."""
    r = mk.axis_angle((0, 1, 0), math.radians(phi_deg))  # folding turns the leg about +y (top toward +x)
    p = H + r @ (P_REF - H)
    ap = p - A
    d = float(np.linalg.norm(ap))
    e = ap / d
    n = np.array([e[2], 0.0, -e[0]])  # in-plane normal on the knee's side (up, and away from the robot)
    h = math.sqrt(max(L * L - (d / 2) ** 2, 0.0))
    k = A + e * d / 2 + n * h
    sa, sb = _slope(k - A), _slope(p - k)
    beta = 2 * math.degrees(math.asin(min(1.0, h / L)))
    return {"P": p, "K": k, "n": n, "slope_a": sa, "slope_b": sb, "beta": beta,
            "q_a": sa - SLOPE_A0, "q_knee": beta - BETA_S, "q_pin": (sb - SLOPE_B0 + phi_deg) % 360.0}


SLOPE_A0 = SLOPE_B0 = 0.0
_ref = loop(0.0)
SLOPE_A0, SLOPE_B0 = _ref["slope_a"], _ref["slope_b"]
K0 = _ref["K"]
E_A0 = (K0 - A) / L
_START = loop(START_DEG)

kit = mk.Kit()
_tx = (TOP_X[0] + TOP_X[1]) / 2
_REST_H = HZ - TUBE / 2 - 0.0015  # the moulded leg rests: 1.5 mm under the folded tubes
kit.part("tabletop", (_tx, YC, TOP), [
    mk.box((TOP_X[1] - TOP_X[0], TOP_W, SLAB), at=(0, 0, SLAB / 2)),  # the moulded top, upside down
    mk.box((0.010, TOP_W, 0.030), at=(TOP_X[0] - _tx + 0.005, 0, SLAB + 0.015)),  # its end rim
    *[mk.box((0.034, 0.008, HZ + 0.013), at=(XH - _tx, s * (TUBE_Y + TUBE / 2 + 0.005), SLAB + (HZ + 0.013) / 2),
             color=STEEL) for s in (-1, 1)],  # leg hinge brackets
    *[mk.box((0.040, 0.040, _REST_H), at=(XH + 0.560 - _tx, s * TUBE_Y, SLAB + _REST_H / 2)) for s in (-1, 1)],
    mk.box((0.016, 0.022, HZ), at=(A[0] - _tx + 0.004, 0, SLAB + HZ / 2), color=STEEL),  # brace bracket
], color=HDPE)

_BAR_HALF = (TUBE_Y - SLOT) / 2
_leg_boxes = [mk.box((TUBE, TUBE, TUBE_L), at=(0, s * TUBE_Y, TUBE_L / 2 - TUBE / 2)) for s in (-1, 1)]
_leg_boxes += [
    mk.box((0.020, 2 * TUBE_Y, 0.020), at=(0, 0, 0)),  # hinge rod
    # the brace bar, slotted at the middle for the upper strip, with the clevis's two cheeks under it
    *[mk.box((0.020, _BAR_HALF, 0.020), at=(0, s * (SLOT + _BAR_HALF / 2), R_B)) for s in (-1, 1)],
    *[mk.box((0.024, 0.003, 0.024), at=(0.012, s * (STRIP_T / 2 + 0.0035), R_B), color=STEEL) for s in (-1, 1)],
    mk.box((0.020, 2 * TUBE_Y + 0.02, 0.020), at=(0, 0, 0.660)),  # foot bar
    *[mk.box((0.030, 0.030, 0.012), at=(0, s * TUBE_Y, TUBE_L - TUBE / 2), color=RUBBER) for s in (-1, 1)],
]
kit.part("leg", tuple(H), _leg_boxes, mass=LEG_KG, color=ALU)
kit.hinge("leg_hinge", "tabletop", "leg", anchor=tuple(H), axis=(0, 1, 0), lower_deg=0.0, upper_deg=FOLD_STOP_DEG,
          start_deg=START_DEG, friction=LEG_FRICTION)

_qa = tuple(mk.mat_to_quat(_rot_y(-SLOPE_A0)))
kit.part("link_a", tuple(A), [mk.box((L + 0.008, STRIP_T, STRIP_H), at=((L - 0.008) / 2, 0, 0))], mass=LINK_KG,
         color=STEEL, quat=_qa)
kit.hinge("brace_a", "tabletop", "link_a", anchor=tuple(A), axis=(0, -1, 0), lower_deg=-75.0, upper_deg=10.0,
          start_deg=_START["q_a"])
kit.part("link_b", tuple(K0), [mk.box((L + 0.010, STRIP_T, STRIP_H), at=(L / 2, 0, 0))], mass=LINK_KG,
         color=STEEL, quat=tuple(mk.mat_to_quat(_rot_y(-SLOPE_B0))))
kit.hinge("knee", "link_a", "link_b", anchor=tuple(K0), axis=(0, -1, 0), lower_deg=-3.0, upper_deg=155.0,
          start_deg=_START["q_knee"])
_rw = (RING_IN_H + RING_WALL) / 2
kit.part("ring", tuple(A + E_A0 * S0), [
    mk.box((RING_LEN, RING_IN_T + 2 * RING_WALL, RING_WALL), at=(0, 0, _rw)),
    mk.box((RING_LEN, RING_IN_T + 2 * RING_WALL, RING_WALL), at=(0, 0, -_rw)),
    mk.box((RING_LEN, RING_WALL, RING_IN_H), at=(0, (RING_IN_T + RING_WALL) / 2, 0)),
    mk.box((RING_LEN, RING_WALL, RING_IN_H), at=(0, -(RING_IN_T + RING_WALL) / 2, 0)),
], mass=RING_KG, color=ZINC, quat=_qa)
kit.slider("ring_slide", "link_a", "ring", axis=tuple(E_A0), lower=-0.004, upper=RING_TOP - S0, start=0.0,
           friction=RING_FRICTION)

ROD = 0.300

kit.part("pusher", (PAD_X, YC - TUBE_Y, PAD_Z), [
    mk.box(PAD, color=RUBBER),
    mk.box((0.016, ROD_DY, 0.016), at=(-0.004, -ROD_DY / 2 - PAD[1] / 2 + 0.008, 0), color=STEEL),  # its arm
    mk.box((ROD, 0.016, 0.016), at=(ROD / 2 - 0.012, -ROD_DY - PAD[1] / 2 + 0.008, 0), color=STEEL),  # its rod
], mass=PUSH_KG, color=RIG)
kit.slider("push", None, "pusher", axis=(1, 0, 0), lower=0.0, upper=PUSH_TRAVEL, start=0.0, stiffness=K_PUSH,
           rest=-FAR)
# the rig (fixed): a column on the bench beside the table, and a guide rail beside the rod (clear of it)
_ROD_Y = YC - TUBE_Y - ROD_DY - PAD[1] / 2 + 0.008
_RIG_X, _RIG_Y = PAD_X + 0.250, _ROD_Y - 0.040
kit.part("rig", (_RIG_X, _RIG_Y, TOP), [
    mk.box((0.040, 0.040, PAD_Z - TOP + 0.03), at=(0, 0, (PAD_Z - TOP + 0.03) / 2)),
    mk.box((0.100, 0.100, 0.012), at=(0, 0, 0.006)),
    mk.box((0.200, 0.016, 0.016), at=(-0.090, 0.020, PAD_Z - TOP)),  # guide rail, beside the rod
], color=RIG)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the loop: the brace's upper pin (leg <-> upper strip) closes the four-bar. The kit builds trees only, so this
# one joint is authored here, with the kit's own authoring code, right after the kit has authored its joints. -------
_PIN = mk._Joint("brace_pin", "hinge", "leg", "link_b", np.array(P_REF, float), np.array([0.0, 1.0, 0.0]),
                 -math.inf, math.inf, 0.0)  # free: the loop sets it (it turns ~210 deg)
_author_all0 = mk._author_all


def _author_all(stage, env_path):
    _author_all0(stage, env_path)
    if (kit.id, "brace_pin") in mk._SIM["joint_paths"]:
        return
    p = mk._author_joint(stage, kit, _PIN, mk._SIM["paths"], mk._SIM["env_T"])
    if p:
        mk._SIM["joint_paths"][(kit.id, "brace_pin")] = p
    print(f"[folding-table] brace pin joint: {p}", flush=True)


if not getattr(mk._author_all, "_brace_pin", False):
    _author_all._brace_pin = True
    mk._author_all = _author_all

# -- the per-step rule (sim): the side load once the robot has let go; and the readouts and scoring ----------------
DT = 0.05
CLEAR = 0.05  # "stood clear": both TCPs 5 cm from the leg, both strips and the ring
QUIET_STEPS = 40  # 2 s let go, clear and still before the load
RAMP_STEPS, HOLD_STEPS, BACK_STEPS, SETTLE_STEPS = 10, 30, 10, 20
OPEN_DEG = 1.0  # open: the leg within 1 deg of upright
FOLD_DEG = 3.0  # the load test fails if the leg folds this much from where it stood
WORKED_DEG, WORKED_S = 3.0, 0.010
M = "ftlr"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "init": False, "worked": False, "quiet": 0, "phase": None, "k": 0,
                            "base": None, "fold_max": 0.0, "failed": False, "prev": {}, "still": {}}
    return m


def leg_deg(objects) -> float:
    """How far the leg is folded from upright (deg; 0 = on its open stop)."""
    return kit.q_deg(objects, "leg_hinge")


def knee_deg(objects) -> float:
    """The brace's bend at the knee (deg; 0 = straight), from the two strips' poses."""
    return kit.q_deg(objects, "knee") + BETA_S


def ring_s(objects) -> float:
    """How far up the lower strip the ring's centre is from the bracket pin A (m), read in the strip's own frame
    (not from the slider's world-frame value, which picks up any small tilt of the strip times its distance from
    the world origin)."""
    t = mk._inv(kit._pose_T(objects, "link_a")) @ kit._pose_T(objects, "ring")
    return float(t[0, 3])


def is_open(objects) -> bool:
    return leg_deg(objects) <= OPEN_DEG


def is_locked(objects) -> bool:
    """The ring over the knee: its leading end at least 12 mm onto the upper strip."""
    return ring_s(objects) >= LOCK_S


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _box_dist(objects, part, pt) -> float:
    """Distance (m) from a point to the nearest of ``part``'s boxes (0 inside)."""
    t = kit._pose_T(objects, part)
    best = 1e9
    for b in kit.parts[part].boxes:
        tb = t @ mk._T(mk.quat_to_mat(b.quat), b.at)
        local = mk._inv(tb) @ np.r_[np.asarray(pt, float), 1.0]
        d = np.maximum(np.abs(local[:3]) - np.asarray(b.size) / 2, 0.0)
        best = min(best, float(np.linalg.norm(d)))
    return best


def clear_of(objects, margin=CLEAR) -> bool:
    """Both grippers' TCPs at least ``margin`` from the leg, both strips and the ring."""
    return all(_box_dist(objects, p, t) >= margin for t in _tcps() for p in PARTS)


def _set_drive(name, target) -> None:
    """Sim only: set joint ``name``'s drive target (m) now."""
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear").GetTargetPositionAttr().Set(float(target))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit ftlr] step {mk._EP.get('step')}: {msg}", flush=True)


def _rules(objects) -> None:
    """One control step: stillness, the "worked" latch, and the side-load sequence with its fold check."""
    m = _mem()
    sim = mk._EP.get("env") is not None
    m["n"] += 1
    if not m["init"]:
        m["init"] = True
        _set_drive("push", -FAR)
    for p in (*PARTS, "pusher"):  # still: under 0.5 mm and 1.4 deg of motion in a control step
        t = kit._pose_T(objects, p)
        t0 = m["prev"].get(p)
        m["still"][p] = t0 is None or bool(np.linalg.norm(t[:3, 3] - t0[:3, 3]) < 0.0005
                                           and mk._rot_angle(t[:3, :3] @ t0[:3, :3].T) < 0.5 * DT)
        m["prev"][p] = t
    a = leg_deg(objects)
    if abs(a - START_DEG) >= WORKED_DEG or abs(ring_s(objects) - S0) >= WORKED_S:
        m["worked"] = True
    ph = m["phase"]
    if ph is None:
        quiet = m["worked"] and clear_of(objects) and all(m["still"].values())
        m["quiet"] = m["quiet"] + 1 if quiet else 0
        if m["quiet"] >= QUIET_STEPS:
            m["phase"], m["k"], m["base"] = "ramp", 0, a
            _say(f"LOAD: leg {a:.2f} deg, knee {knee_deg(objects):.1f} deg, ring {ring_s(objects) * 1000:.1f} mm")
    else:
        m["k"] += 1
        k = m["k"]
        if ph == "ramp":
            _set_drive("push", -FAR + 2 * FAR * min(1.0, k / RAMP_STEPS))
            if k >= RAMP_STEPS:
                m["phase"], m["k"] = "hold", 0
        elif ph == "hold" and k >= HOLD_STEPS:
            m["phase"], m["k"] = "back", 0
        elif ph == "back":
            _set_drive("push", FAR - 2 * FAR * min(1.0, k / BACK_STEPS))
            if k >= BACK_STEPS:
                m["phase"], m["k"] = "settle", 0
        elif ph == "settle" and k >= SETTLE_STEPS:
            m["phase"] = "done"
            _say(f"load over: leg folded at most {m['fold_max']:.2f} deg; failed {m['failed']}")
        m["fold_max"] = max(m["fold_max"], a - m["base"])
        if a - m["base"] > FOLD_DEG and not m["failed"]:
            m["failed"] = True
            _say(f"the leg FOLDED under the load: {a:.1f} deg")
    if DEBUG and sim and (m["n"] % 5 == 0 or m["phase"] in ("ramp", "back")):
        tc = mk.tcps()
        tz = (f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - TOP:.3f}) L ({tc[0][0]:.3f},{tc[0][1]:.3f},"
              f"{tc[0][2] - TOP:.3f})") if tc else "-"
        pin = np.linalg.norm(kit.point(objects, "leg", P_REF - H) - kit.point(objects, "link_b", (L, 0, 0)))
        lp, lq = kit.pose(objects, "leg")
        _say(f"R tcp {tz}; leg {a:.2f} deg, knee {knee_deg(objects):.1f} deg, ring {ring_s(objects) * 1000:.1f} mm "
             f"(lock at {LOCK_S * 1000:.0f}), push {kit.q(objects, 'push') * 1000:.0f} mm, pin gap {pin * 1000:.2f} mm, "
             f"phase {m['phase']}, quiet {m['quiet']}, worked {m['worked']}, still "
             f"{''.join(str(int(v)) for v in m['still'].values())}, clear {clear_of(objects)}")


kit.on_step(_rules)


def score(objects) -> float:
    """0 if the leg folded more than 3 deg under the side load. Otherwise 0.25 for the leg open (within 1 deg of
    upright); with that, +0.25 for the ring over the knee (its leading end 12 mm or more onto the upper strip);
    with both, +0.5 once the side load is over. Halved unless both grippers are 5 cm clear of the leg, the brace and
    the ring and all of them are still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["failed"]:
        return 0.0
    base = 0.0
    if is_open(objects):
        base = 0.25
        if is_locked(objects):
            base += 0.25
            if m["phase"] == "done":
                base += 0.5
    ok = clear_of(objects) and all(m["still"].get(p, True) for p in PARTS)
    return round(base if ok else base / 2, 6)


def success(objects) -> bool:
    """Leg open, ring over the knee, the side load held; let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do (open, locked, the load) plus the leg's angle from upright (rad) and the ring's distance
    from its locked position (m)."""
    m = _mem()
    return float((not is_open(objects)) + (not is_locked(objects)) + (m["phase"] != "done")
                 + math.radians(max(0.0, leg_deg(objects))) + max(0.0, LOCK_S - ring_s(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; both arms. The leg starts folded flat, where its brace is near a dead point
# (pushing the knee only presses the leg onto its fold stop), so the left hand lifts it by its left tube, 10 cm from
# the hinge (the fingers close on the tube's sides, so the tube turns between the pads), up to 33 deg from upright:
# past that the tube would run up between the fingers into the palm. The right hand meanwhile comes in from the side,
# fingers closed, onto the outer edge of the brace's lower strip just under the bent knee; the left lets go and backs
# off, and the right pushes the strip along its own path (computed from the four-bar's geometry) until the brace is
# straight and the leg on its stop, then 15 mm further into the stop. It backs off, opens, comes down astride the ring (still on its stop by the
# bracket), grips it and slides it straight up the lower strip, over the knee, onto its upper stop. Then it opens,
# lifts off, and both arms park and wait while the leg is pushed. Every TCP waypoint stays at x 0.12-0.40 and at most
# 185 mm above the bench (the arms' reach there), checked offline.

TIP_C = 0.0065  # the finger-tip block's centre below the TCP (tips 13 mm below)
TIP_R = 0.010  # its reach along the knee's outward normal (half-sizes 7.8 x 6.5 mm)
GRIP = -0.005  # 10 mm between the pads (the ring is 18 mm across): 8 mm squeeze
GRIP_TUBE = -0.008  # 16 mm between the pads on the 25 mm tube: 9 mm squeeze
RELEASE = -0.016  # let go of the tube only this far open (32 mm): wider, the fingers' bodies swing in toward it
R_GRIP = 0.100  # the left hand's grip on its tube, this far from the hinge
PHI_HAND = 33.0  # the leg angle (deg from upright) at which the right hand takes over at the knee
SIDE = 0.050  # the right hand comes onto the knee from this far out along -y
KNEE_BELOW = 0.015  # the right hand pushes the lower strip this far under the knee pin
SLIDE_OVER = 0.004  # commanded past the ring's upper stop
SAG = 0.0015  # the slide path is held this far toward straight (a hand that sags pushes the knee onto its stop)
PRESS_OVER = 0.015  # the knee pushed this far past straight-and-on-stop
R_PARK = (0.240, -0.330, TOP + 0.150)
L_PARK = (0.220, 0.430, TOP + 0.150)
PUSH_CAP = 25.0  # N: the most the plan asks of the right hand on the strip
REACH_X, REACH_Z = (0.120, 0.400), 0.185  # every waypoint's TCP inside this (x range; height above the bench)
YL = YC + TUBE_Y  # the left tube


def _knee_tcp(phi, back=0.0, dy=0.0):
    """TCP pushing the lower strip's outer edge KNEE_BELOW under the knee pin at leg angle ``phi`` (deg), ``back`` m
    further out along the strip's outward normal (the knee's side). Pushed there, the strip turns about its bracket
    and the brace straightens; it keeps the hand lower than a push on the knee's corner."""
    c = loop(phi)
    ea = (c["K"] - A) / L
    na = np.array([ea[2], 0.0, -ea[0]])
    na = na if na @ c["n"] > 0 else -na
    p = A + ea * (L - KNEE_BELOW) + na * (STRIP_H / 2 + TIP_R + back) + np.array([0, dy, TIP_C])
    return tuple(float(v) for v in p)


def _ring_tcp(s, out=0.0):
    """TCP gripping the ring with its centre ``s`` up the lower strip (leg open), ``out`` m along the knee normal."""
    p = A + E_A0 * s + _ref["n"] * out + np.array([0, 0, TIP_C])
    return tuple(float(v) for v in p)


def _tube_tcp(phi, off=(0.0, 0.0)):
    """TCP on the left tube's axis R_GRIP from the hinge at leg angle ``phi``; ``off`` (m) along the tube's fold-side
    normal and up."""
    a = math.radians(phi)
    f = np.array([math.cos(a), 0.0, -math.sin(a)])
    p = np.array([XH + R_GRIP * math.sin(a), YL, H[2] + R_GRIP * math.cos(a)]) + f * off[0] + np.array([0, 0, off[1]])
    return tuple(float(v) for v in p)


LEG_COM = 0.30  # m up the leg: its centre of mass (the tubes and three bars, by volume)
FF = True  # feed the arms' compliance forward (the offline clash check replays the plan with FF off)


def _hinge_torque(phi, moving=True):
    """N*m the robot must supply to hold (``moving``: to swing up) the leg at ``phi`` deg from upright."""
    return LEG_KG * 9.81 * LEG_COM * math.sin(math.radians(phi)) + (LEG_FRICTION if moving else 0.0)


def _strip_push(phi):
    """Unit push direction on the lower strip and its lever (m of contact travel per rad of unfolding)."""
    def contact(ph):
        c = loop(ph)
        ea = (c["K"] - A) / L
        na = np.array([ea[2], 0.0, -ea[0]])
        return A + ea * (L - KNEE_BELOW), (na if na @ c["n"] > 0 else -na)
    p, na = contact(phi)
    p2, _ = contact(phi - 0.2)
    return -na, float(-na @ (p2 - p)) / math.radians(0.2)


def _compliance(arm, pos):
    """The arm's TCP compliance (m/N, 3x3) at ``pos``: its joints are position servos of finite stiffness (the
    station's shoulder and wrist gains), so a hand that pushes or lifts sags back by C @ F. The plan aims that far
    beyond the path, as a stiffer or force-controlled arm would hold it."""
    from yam_arena import ik, station
    from yam_arena.constants import READY, SHOULDER, WRIST
    from yam_arena.demo import grip_rotation

    base = station.BASES[arm]
    q, _, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip_rotation(0.0)), np.array(READY, float),
                       base_pos=base, iters=200)
    p0 = ik.tcp_pose(q, base)[0]
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        jac[:, i] = (ik.tcp_pose(q + dq, base)[0] - p0) / 1e-6
    k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3, float)
    return jac @ np.diag(1.0 / k) @ jac.T


def _aim(arm, pos, force):
    """Where to command the TCP so that, loaded with ``force`` (N, applied by the hand), it sits at ``pos``."""
    pos = np.asarray(pos, float)
    if not FF or not np.any(force):
        return tuple(float(v) for v in pos)
    return tuple(float(v) for v in pos + _compliance(arm, pos) @ np.asarray(force, float))


def _lift_force(phi, moving=True):
    a = math.radians(phi)
    return np.array([-math.cos(a), 0.0, math.sin(a)]) * _hinge_torque(phi, moving) / R_GRIP


def _push_force(phi, moving=True):
    d, lever = _strip_push(max(phi, 1.0))
    return d * min(PUSH_CAP, _hinge_torque(phi, moving) / lever)


DEMO_PHI = []  # (time s, leg angle deg) the plan expects: for the offline clash check


def demo():
    """Lift the folded leg by its tube (left), hand over to a push on the brace's knee (right), push it straight (the
    leg swings upright onto its stop), then grip the ring and slide it up over the knee; let go, park and wait while
    the leg is pushed."""
    c, o = FINGER_CLOSED, FINGER_OPEN
    Lw, R = [], []
    DEMO_PHI.clear()

    def w(arm, dur, pos, f, label):
        arm.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, 0.0))

    def t(arm):
        return sum(x.duration for x in arm)

    # left: down astride its tube by the hinge, grip, lift the leg to PHI_HAND
    w(Lw, 2.0, L_PARK, o, "start")
    w(Lw, 2.0, _tube_tcp(START_DEG, (0.0, 0.040)), o, "over the left tube by its hinge")
    w(Lw, 1.0, _tube_tcp(START_DEG), o, "down astride the tube")
    w(Lw, 0.8, _tube_tcp(START_DEG), GRIP_TUBE, "grip the tube")
    w(Lw, 0.3, _tube_tcp(START_DEG), GRIP_TUBE, "settle closed")
    DEMO_PHI.append((t(Lw), START_DEG))
    w(Lw, 1.0, _aim("left", _tube_tcp(START_DEG), _lift_force(START_DEG)), GRIP_TUBE, "take the leg's weight")
    DEMO_PHI.append((t(Lw), START_DEG))
    n = 11
    for k in range(1, n + 1):
        phi = START_DEG + (PHI_HAND - START_DEG) * k / n
        w(Lw, 6.0 / n, _aim("left", _tube_tcp(phi), _lift_force(phi)), GRIP_TUBE, "lift the leg by its tube")
        DEMO_PHI.append((t(Lw), phi))
    hold = _aim("left", _tube_tcp(PHI_HAND), _lift_force(PHI_HAND, moving=False))
    w(Lw, 0.5, hold, GRIP_TUBE, "hold the leg up")
    t_up = t(Lw)
    # right: wait, then in from the side onto the knee's outer corner
    w(R, 2.0, R_PARK, c, "start")
    w(R, t_up - 2.0, R_PARK, c, "wait while the leg is lifted")
    w(R, 1.5, _knee_tcp(PHI_HAND, 0.010, -SIDE), c, "beside the bent knee, fingers closed")
    w(R, 1.0, _knee_tcp(PHI_HAND, 0.010), c, "in, just off the lower strip's outer edge")
    w(R, 0.6, _knee_tcp(PHI_HAND, 0.0), c, "onto the strip under the knee")
    w(Lw, t(R) - t_up, hold, GRIP_TUBE, "hold the leg up")
    # hand the leg's weight over: the right takes it on the strip while the left eases off, then the left lets go
    w(R, 1.2, _aim("right", _knee_tcp(PHI_HAND), _push_force(PHI_HAND, moving=False)), c, "take the leg's weight")
    w(Lw, 1.2, _tube_tcp(PHI_HAND), GRIP_TUBE, "ease off")
    w(Lw, 0.5, _tube_tcp(PHI_HAND), RELEASE, "let go of the tube")
    w(Lw, 0.8, _tube_tcp(PHI_HAND, (-0.040, 0.0)), RELEASE, "back off, square to the tube")
    w(Lw, 1.5, L_PARK, o, "park")
    w(R, t(Lw) - 1.5 - t(R), _aim("right", _knee_tcp(PHI_HAND), _push_force(PHI_HAND, moving=False)), c,
      "hold the leg up while the left lets go")
    DEMO_PHI.append((t(R), PHI_HAND))
    n = 10
    for k in range(1, n + 1):
        phi = PHI_HAND * (1 - k / n)
        w(R, 6.0 / n, _aim("right", _knee_tcp(phi), _push_force(phi)), c,
          "push the strip along its path: the leg swings upright")
        DEMO_PHI.append((t(R), phi))
    w(R, 1.0, _knee_tcp(0.0, -PRESS_OVER), c, "press it onto the stop")
    w(R, 0.5, _knee_tcp(0.0, -PRESS_OVER), c, "hold")
    w(R, 0.6, _knee_tcp(0.0, 0.015), c, "back off")
    if VARIANT == "open":
        w(R, 1.5, R_PARK, c, "park")
    else:  # along the brace to the ring, never more than about 4 cm from it (so it does not count as let go)
        up = np.array([0, 0, 0.030])
        w(R, 0.4, _knee_tcp(0.0, 0.015), o, "open the fingers")
        w(R, 1.2, np.array(_ring_tcp(S0)) + up, o, "over the ring")
        w(R, 1.2, _ring_tcp(S0, -SAG), o, "down astride the ring")
        w(R, 0.4, _ring_tcp(S0, -SAG), o, "settle")
        w(R, 0.8, _ring_tcp(S0, -SAG), GRIP, "grip the ring")
        w(R, 0.3, _ring_tcp(S0, -SAG), GRIP, "settle closed")
        s_end = S0 + 0.035 if VARIANT == "half" else RING_TOP + SLIDE_OVER
        n = 6
        for k in range(1, n + 1):
            w(R, 3.0 / n, _ring_tcp(S0 + (s_end - S0) * k / n, -SAG), GRIP, "slide the ring up over the knee")
        w(R, 0.5, _ring_tcp(s_end, -SAG), GRIP, "hold on its stop")
        w(R, 0.6, _ring_tcp(s_end, -SAG), o, "let go")
        w(R, 1.0, np.array(_ring_tcp(s_end)) + np.array([0.0, 0, 0.037]), o, "lift off")
        w(R, 0.8, np.array(_ring_tcp(s_end)) + np.array([0.0, -0.070, 0.037]), o, "out to the side")
        w(R, 2.0, R_PARK, o, "park")
    w(R, 9.0, R_PARK, R[-1].finger, "wait: the side load")
    w(Lw, t(R) - t(Lw), L_PARK, o, "park")
    return {"left": Lw, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed one control step at a time). The
    brace is posed by the four-bar's geometry for each leg angle, so every state is one the linkage can reach."""
    global _TEST_TCPS
    for phi in np.linspace(0.0, FOLD_STOP_DEG, 19):  # the kit's tree, posed from loop(), closes at the brace pin
        st = state(phi, 0.0)
        gap = np.linalg.norm(kit.point(st, "leg", P_REF - H) - kit.point(st, "link_b", (L, 0, 0)))
        assert gap < 1e-9, (phi, gap)
        assert abs(knee_deg(st) - loop(phi)["beta"]) < 1e-6, phi
        c = loop(phi)
        for jn, q in (("brace_a", c["q_a"]), ("knee", c["q_knee"])):
            j = kit.joints_[jn]
            assert math.radians(q) > j.lower + 0.02 or phi == 0.0, (jn, phi, q)
            assert math.radians(q) < j.upper - 0.02, (jn, phi, q)
    assert abs(ring_s(state(START_DEG, 0.0)) - S0) < 1e-9
    knee_hand = [_knee_tcp(0.0)]
    lift_hand = [_tube_tcp(START_DEG)]
    ring_hand = [_ring_tcp(RING_TOP)]

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def run(plan):
        """plan: list of (leg deg, ring travel m, push travel m, tcps); returns the last score and state."""
        global _TEST_TCPS
        got = st = None
        for phi, s, d, tc in plan:
            _TEST_TCPS = list(tc)
            st = state(phi, s, d)
            got = score(st)
        return got, st

    def opening(hand=True):
        return ([(a, 0.0, 0.0, lift_hand if hand else []) for a in lin(START_DEG, PHI_HAND)]
                + [(a, 0.0, 0.0, knee_hand if hand else []) for a in lin(PHI_HAND, 0.0)])

    def sliding(to=RING_TOP - S0, phi=0.0):
        return [(phi, s, 0.0, ring_hand) for s in lin(0.0, to)]

    def away(phi, s, n=QUIET_STEPS + 2):
        return [(phi, s, 0.0, [])] * n

    def load(phi, s, fold=0.0, n=RAMP_STEPS + HOLD_STEPS + BACK_STEPS + SETTLE_STEPS + 3):
        """The side load: the pad comes in and pushes; the leg folds by up to ``fold`` deg and stays there."""
        out = []
        for k in range(n):
            f = min(1.0, k / RAMP_STEPS) * fold
            d = min(PUSH_TRAVEL, 0.003 + math.radians(f) * PUSH_R) if k < RAMP_STEPS + HOLD_STEPS else 0.0
            out.append((min(FOLD_STOP_DEG, phi + f), s, d, []))
        return out

    full = RING_TOP - S0
    good = opening() + sliding() + away(0.0, full)
    cases = {  # name: (plan, score)
        "start (folded)": ([(START_DEG, 0.0, 0.0, [])], 0.0),
        "nothing touched for 3 s (no load is applied)": (away(START_DEG, 0.0, 60), 0.0),
        "open, ring over the knee, let go, the load held (demo)": (good + load(0.0, full, 0.3), 1.0),
        "the same, scored before the load": (good, 0.5),
        "open and locked, hand left on the ring (no load)": (opening() + sliding() + [(0.0, full, 0.0, ring_hand)] * 30,
                                                             0.25),
        "naive: leg opened only, let go; the load folds it": (opening() + away(0.0, 0.0) + load(0.0, 0.0, 19.0), 0.0),
        "leg opened only, scored before the load": (opening() + away(0.0, 0.0), 0.25),
        "leg lifted half way and let go: it falls back onto its fold stop (load into air)":
            ([(a, 0.0, 0.0, lift_hand) for a in lin(START_DEG, 45.0)] + [(a, 0.0, 0.0, []) for a in lin(45.0, 90.0)]
             + away(START_DEG, 0.0) + load(START_DEG, 0.0, 0.0), 0.0),
        "ring slid only part way (its end short of the knee); the load folds it":
            (opening() + sliding(0.035) + away(0.0, 0.035) + load(0.0, 0.035, 19.0), 0.0),
        "ring 2 mm short of locked; the load folds the leg": (opening() + sliding(LOCK_S - S0 - 0.002)
                                                              + away(0.0, LOCK_S - S0 - 0.002)
                                                              + load(0.0, LOCK_S - S0 - 0.002, 12.0), 0.0),
        "ring just locked (12 mm onto the upper strip), held": (opening() + sliding(LOCK_S - S0 + 0.0005)
                                                                + away(0.0, LOCK_S - S0 + 0.0005)
                                                                + load(0.0, LOCK_S - S0 + 0.0005, 0.5), 1.0),
        "locked but the leg gives 3.5 deg under the load": (good + load(0.0, full, 3.5), 0.0),
        "locked, the leg gives 2 deg and springs back (held)": (good + load(0.0, full, 2.0)[:45]
                                                                + [(0.0, full, 0.0, [])] * 30, 1.0),
        "leg left 2 deg short of upright, ring stopped short at the bent knee; the load folds it":
            (opening()[:-1] + [(2.0, 0.0, 0.0, knee_hand)] + sliding(0.050, 2.0) + away(2.0, 0.050)
             + load(2.0, 0.050, 17.0), 0.0),
        "ring slid up, then pulled back off the knee before letting go": (opening() + sliding()
                                                                          + sliding()[::-1] + away(0.0, 0.0)
                                                                          + load(0.0, 0.0, 19.0), 0.0),
        "demo, but the arm parks 3 cm over the ring (not clear: no load)": (opening() + sliding() + [(0.0, full, 0.0, [
            tuple(np.array(_ring_tcp(RING_TOP)) + np.array([0, 0, 0.030]))])] * 30, 0.25),
    }
    try:
        for name, (plan, want) in cases.items():
            mk.new_episode()
            got, st = run(plan)
            assert abs(got - want) < 1e-9, (name, got, want, {k: v for k, v in _mem().items() if k not in ("prev",)})
            assert success(st) == (want >= 1.0), name
        mk.new_episode()
        run(good + load(0.0, full, 19.0))
        assert _mem()["failed"]
        mk.new_episode()
        _TEST_TCPS = []
        assert score(state(START_DEG, 0.0)) == 0.0 and not _mem()["failed"], "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


def state(phi, s, d=0.0):
    """Every part's state for the leg folded ``phi`` deg, the ring ``s`` m up from its start, the pad in ``d`` m."""
    c = loop(phi)
    return kit.fake_states(q={"leg_hinge": math.radians(phi), "brace_a": math.radians(c["q_a"]),
                              "knee": math.radians(c["q_knee"]), "ring_slide": s, "push": d})


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm, and every waypoint
    # inside the arms' reach (x 0.12-0.40, at most 185 mm above the bench).
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n = self_test()
    print(f"scoring: {n} cases pass (strips {L * 1000:.1f} mm; knee bent {_START['beta']:.1f} deg folded, "
          f"{BETA_S:.0f} deg on the stop; locked from ring travel {(LOCK_S - S0) * 1000:.0f} mm of "
          f"{(RING_TOP - S0) * 1000:.0f})")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        for wp in waypoints:
            assert REACH_X[0] - 1e-9 <= wp.pos[0] <= REACH_X[1] + 1e-9 and wp.pos[2] - TOP <= REACH_Z + 1e-9, (arm, wp)
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        top = max(wp.pos[2] for wp in waypoints) - TOP
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg, "
              f"TCP at most {top * 1000:.0f} mm above the bench")
