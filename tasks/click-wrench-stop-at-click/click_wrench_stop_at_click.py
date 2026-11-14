"""On the station table, in front of the right arm, a copper busbar stands on two insulators with a cable lug on it.
The lug's bolt has been run down snug by hand; a click-type torque wrench, already set to the bolt's torque, sits
on it with its socket on the bolt head. The wrench lies flat, its handle pointing toward the right arm, its rubber
grip 21.5 cm from the bolt.

The wrench works as a real click wrench does. Inside it a sprung detent holds the handle in line with the head;
the torque you put through the grip bends that detent a little (1.5 deg at the set torque). At the set torque it
breaks over: the handle swings 6 deg on its pivot to a stop, a visible kink, and the load drops for a moment (the
click). Then nothing stops you: on the stop, the handle drives the head again, and pulling on turns the bolt
further. Let go, and the detent pulls the handle back in line and re-arms.

The bolt: its turning torque rises with every degree past snug (the joint clamping up), and it stays where it is
left (thread friction). At the set torque it is about 5 deg past snug. Pulled on to twice the set torque, about
15 deg past snug, it yields (the stud stretches): it then turns on under a steady pull and is ruined. Backing it
off undoes the torque.

The job: tighten the bolt to its torque with the wrench, then let go. Nothing about the bolt says when to stop;
the wrench does, once, and pulling on is still possible. Full credit needs the peak torque inside the joint's
tolerance, 90-110 % of the set torque (the bolt 3.9-5.9 deg past snug); a pull on past 110 % scores 0.

The real joint this models (README, "Real hardware and calibration"): an M5 x 0.8 class 8.8 hex bolt (8 mm
across flats) with a flat washer, through a 16 mm2 tinned-copper tube lug and a 20 x 6 mm copper bar into a
self-clinching M5 nut pressed into the bar; marked 3 N*m, snug 1.5 N*m; a 1/4-inch drive click wrench of 2-10 N*m
range, an inline torque transducer under its drive. At the grip's load mark 3 N*m is 14 N. The angles are
estimated from the joint's stiffness; the sim's torques are 1/7 of the real ones (0.43 N*m set, 2 N at the grip)
so the soft simulated arm can apply them.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges with springs and friction, per-step rules, per-episode memory)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the wrench's events and a readout every control step while it is loaded
VARIANT = "demo"  # sim runs: "demo"; "through" (pulls on to 45 deg); "short" (stops at 6 deg)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the bolted joint: busbar on two insulators, a cable lug on it, the bolt's axis vertical -----------------------
BX, BY = 0.330, -0.090  # bolt axis
ZLUG = TOP + 0.033  # lug top (insulators 24 mm, bar 6 mm, lug palm 3 mm)
SOCKET_H, SOCKET_W, HEAD_T = 0.016, 0.012, 0.014  # 8 mm socket, 1/4-inch drive
XDCR = (0.026, 0.040)  # inline rotary torque transducer between the drive and the socket: width, height
ZH = ZLUG + 0.001 + SOCKET_H + XDCR[1] + HEAD_T / 2  # the wrench's centre plane (socket 1 mm clear of the lug)
# -- the wrench: head (on the bolt) and handle (on the head's pivot), both turning about vertical axes -------------
R_PIVOT = 0.045  # handle pivot, from the bolt axis
R_GRIP = 0.215  # grip centre, from the bolt axis
GRIP = (0.018, 0.070, 0.018)  # rubber grip: across (x at the start), along the handle, height
TUBE = (0.014, 0.012)  # handle tube: width, height
HEAD_KG, HANDLE_KG = 0.30, 0.10  # head with the transducer and socket; handle
# -- the mechanism (forces at the grip are 1/7 of the real joint's, for the soft simulated arm) -------------------
F0, F_CLICK, F_OVER = 1.0, 2.0, 4.6  # N at the grip: bolt starts to turn; the wrench's set torque; the bolt yields
T0, T_CLICK, T_OVER = F0 * R_GRIP, F_CLICK * R_GRIP, F_OVER * R_GRIP  # N*m
SLOPE = 2.5  # N*m/rad: the bolt's turning torque rises this much per radian past snug
CLICK_DEG = 1.5  # the detent breaks over at this deflection (= the set torque, pulled at the grip's centre)
K_DETENT = F_CLICK * (R_GRIP - R_PIVOT) / math.radians(CLICK_DEG)  # N*m/rad about the pivot, the armed detent
K_RETURN = 0.3  # N*m/rad, the reset spring once it has broken over
BREAK_DEG = 6.0  # the handle's stop: how far it kinks at the click
REARM_DEG = 0.5  # back within this of in line (load off): the detent re-arms
SPEC_DEG = math.degrees((T_CLICK - T0) / SLOPE)  # bolt angle at the set torque (4.9 deg)
YIELD_DEG = math.degrees((T_OVER - T0) / SLOPE)  # bolt angle at the yield torque (17.7 deg, 230 %)
# The real joint (documentation; the sim's torques are these times TORQUE_SCALE, its angles are the real ones):
REAL_SET_NM = 3.0  # M5 class 8.8 steel bolt on copper, the marked torque (14 N at the load mark); snug = half
TORQUE_SCALE = T_CLICK / REAL_SET_NM  # 0.143 = 1/7
# The joint's documented tolerance on the peak torque: 90-110 % of the specified torque.
TOL = 0.10
TOL_LO_DEG = math.degrees(((1 - TOL) * T_CLICK - T0) / SLOPE)  # 3.9 deg past snug: 90 % of the set torque
TOL_HI_DEG = math.degrees(((1 + TOL) * T_CLICK - T0) / SLOPE)  # 5.9 deg past snug: 110 %, above it over-torqued
DAMP = 0.02  # N*m*s/rad on both hinges (a quiet solver)

STEEL, CHROME, RUBBER, RED = (0.30, 0.31, 0.33), (0.70, 0.72, 0.75), (0.08, 0.08, 0.09), (0.75, 0.12, 0.10)
COPPER, TIN, EPOXY, PVC = (0.78, 0.46, 0.25), (0.80, 0.80, 0.78), (0.85, 0.80, 0.20), (0.10, 0.10, 0.10)

kit = mk.Kit()
_hy = R_PIVOT  # head frame: origin on the bolt axis at the centre plane, the handle along -y
kit.part("wrench_head", (BX, BY, ZH), [
    mk.box((0.032, 0.032, HEAD_T)),  # ratchet head
    mk.box((XDCR[0], XDCR[0], XDCR[1]), at=(0, 0, -HEAD_T / 2 - XDCR[1] / 2), color=(0.15, 0.25, 0.45)),  # transducer
    mk.box((SOCKET_W, SOCKET_W, SOCKET_H), at=(0, 0, -HEAD_T / 2 - XDCR[1] - SOCKET_H / 2), color=CHROME),  # socket
    mk.box((0.016, _hy - 0.016, 0.012), at=(0, -(_hy + 0.016) / 2, 0)),  # neck to the pivot
    mk.box((0.010, 0.004, 0.003), at=(0, 0.004, HEAD_T / 2 + 0.0015), color=RED),  # direction lever
], mass=HEAD_KG, color=CHROME)
# handle frame: origin on the pivot, the tube running out along -y
_tube_len = R_GRIP - GRIP[1] / 2 - R_PIVOT
kit.part("handle", (BX, BY - R_PIVOT, ZH), [
    mk.box((TUBE[0], _tube_len, TUBE[1]), at=(0, -_tube_len / 2, 0)),
    mk.box(GRIP, at=(0, -(R_GRIP - R_PIVOT), 0), color=RUBBER),
    mk.box((0.012, 0.006, 0.012), at=(0, -(R_GRIP + GRIP[1] / 2 + 0.003 - R_PIVOT), 0), color=RED),  # lock knob
], mass=HANDLE_KG, color=STEEL)
# the bolt: the head turns with it (socket on); +angle = clockwise seen from above = tightening
kit.hinge("bolt", None, "wrench_head", anchor=(BX, BY, ZH), axis=(0, 0, -1), lower_deg=-30.0, upper_deg=90.0,
          friction=T0, damping=DAMP)
# the detent pivot: handle on the head, held in line by the armed detent spring, its stop at 6 deg
kit.hinge("breakover", "wrench_head", "handle", anchor=(BX, BY - R_PIVOT, ZH), axis=(0, 0, -1), lower_deg=-1.0,
          upper_deg=BREAK_DEG, stiffness=K_DETENT, rest_deg=0.0, damping=DAMP)

FIXTURES = [
    Box("insulator_a", pos=(BX - 0.020, BY, TOP + 0.012), size=(0.020, 0.020, 0.024), color=EPOXY),
    Box("clinch_nut", pos=(BX, BY, TOP + 0.0215), size=(0.008, 0.008, 0.005), color=CHROME),  # M5, under the bar
    Box("insulator_b", pos=(BX + 0.075, BY, TOP + 0.012), size=(0.020, 0.020, 0.024), color=EPOXY),
    Box("busbar", pos=(BX + 0.030, BY, TOP + 0.027), size=(0.120, 0.020, 0.006), color=COPPER),
    Box("lug_palm", pos=(BX + 0.003, BY, ZLUG - 0.0015), size=(0.018, 0.012, 0.003), color=TIN),
    Box("lug_barrel", pos=(BX + 0.023, BY, ZLUG + 0.001), size=(0.022, 0.008, 0.008), color=TIN),
    Box("cable", pos=(BX + 0.074, BY, ZLUG + 0.001), size=(0.080, 0.008, 0.008), color=PVC),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the mechanism's per-step rules (sim) and the scoring ---------------------------------------------------------
TURNED_DEG = 1.5  # partial credit: the bolt turned on from snug (under-torqued; 65 % of the set torque)
SETBACK_DEG = 1.0  # torqued: the bolt still within this of its tightest (it springs back up to 0.5 deg when the
# load comes off)
BACKOFF_DEG = 3.0  # turned back more than this from its tightest: loosened (latched fail)
REARM_STEPS = 4  # the detent re-arms no sooner than this many control steps after a click
STILL_DEG, STILL_M = 0.05, 0.0003  # still: the bolt and the grip moved under this in a control step
M = "cwsc"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
_STICK = mk.STICK_RAD


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "init": False, "armed": True, "clicks": 0, "t_click": None, "peak": 0.0,
                            "over": False, "yielded": False, "loose": False, "still": True, "prev": None,
                            "k_bolt": None}
    return m


def _cw_deg(r) -> float:
    """Clockwise turn (deg, seen from above) of a rotation matrix about the vertical."""
    return -math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))


def bolt_deg(objects) -> float:
    """How far the bolt has turned past snug (deg, + = tightened): the wrench head's turn in the world."""
    return _cw_deg(kit._pose_T(objects, "wrench_head")[:3, :3])


def break_deg(objects) -> float:
    """How far the handle is kinked on its pivot (deg, + = the tightening way), in the head's own frame."""
    rel = mk._inv(kit._pose_T(objects, "wrench_head")) @ kit._pose_T(objects, "handle")
    return _cw_deg(rel[:3, :3])


def grip_point(objects) -> np.ndarray:
    """The grip's centre (world)."""
    return kit.point(objects, "handle", (0.0, -(R_GRIP - R_PIVOT), 0.0))


def bolt_torque(deg: float) -> float:
    """The torque (N*m) that turns the bolt on at ``deg`` past snug: rising, flat once it yields."""
    return T0 + SLOPE * math.radians(min(max(deg, 0.0), YIELD_DEG))


def peak_pct(deg: float) -> float:
    """The torque that turned the bolt to ``deg`` past snug, in % of the set torque (what a peak-hold torque
    transducer between the wrench and the socket would read on the real joint)."""
    return 100.0 * bolt_torque(deg) / T_CLICK


def _set_drive(name, target=None, stiffness=None) -> None:
    """Sim only: set joint ``name``'s drive target (rad) and/or stiffness (N*m/rad) now."""
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    scale = 180.0 / math.pi
    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular")
    if target is not None:
        d.GetTargetPositionAttr().Set(float(target * scale))
    if stiffness is not None:
        d.GetStiffnessAttr().Set(float(stiffness / scale))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit cwsc] step {mk._EP.get('step')}: {msg}", flush=True)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


CLEAR_M = 0.10  # let go: both grippers this far outside the wrench's bounding box


def _released(st, margin=CLEAR_M) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _rules(objects) -> None:
    """One control step of the bolt (its rising turning torque) and the wrench's detent (break over, re-arm)."""
    m = _mem()
    sim = mk._EP.get("env") is not None
    m["n"] += 1
    if not m["init"]:  # a new episode: the detent armed, the bolt at snug
        m["init"] = True
        _set_drive("breakover", target=0.0, stiffness=K_DETENT)
    th, d = bolt_deg(objects), break_deg(objects)
    # the bolt: thread friction rising with the turn (the kit's friction hinge, its limit set from the angle)
    k = bolt_torque(th) / _STICK
    if m["k_bolt"] is None or abs(k - m["k_bolt"]) > 0.002 * k:
        m["k_bolt"] = k
        _set_drive("bolt", stiffness=k)
    m["peak"] = max(m["peak"], th)
    if th > TOL_HI_DEG and not m["over"]:
        m["over"] = True
        _say(f"OVER-TORQUED: bolt {th:.2f} deg past snug, {peak_pct(th):.0f} % of the set torque "
             f"(tolerance ends at 110 %, {TOL_HI_DEG:.1f} deg)")
    if th >= YIELD_DEG and not m["yielded"]:
        m["yielded"] = True
        _say(f"YIELDED: bolt {th:.2f} deg past snug (yields at {YIELD_DEG:.1f})")
    if th < m["peak"] - BACKOFF_DEG and not m["loose"]:
        m["loose"] = True
        _say(f"LOOSENED: bolt {th:.2f} deg, its tightest was {m['peak']:.2f}")
    # the detent: breaks over at the set torque, re-arms once the handle is back in line
    if m["armed"] and d >= CLICK_DEG:
        m["armed"], m["t_click"] = False, m["n"]
        m["clicks"] += 1
        _set_drive("breakover", target=0.0, stiffness=K_RETURN)
        _say(f"CLICK {m['clicks']}: handle {d:.2f} deg, bolt {th:.2f} deg past snug")
    elif not m["armed"] and d <= REARM_DEG and m["n"] - m["t_click"] >= REARM_STEPS:
        m["armed"] = True
        _set_drive("breakover", target=0.0, stiffness=K_DETENT)
        _say(f"re-armed: handle {d:.2f} deg, bolt {th:.2f} deg")
    g = grip_point(objects)
    if m["prev"] is not None:
        th0, g0 = m["prev"]
        m["still"] = bool(abs(th - th0) < STILL_DEG and np.linalg.norm(g - g0) < STILL_M)
    m["prev"] = (th, g)
    if DEBUG and sim:
        tc = mk.tcps()
        if tc:
            t = np.asarray(tc[1])
            a = math.radians(th)  # the hand's lead on the grip, along the grip's path (+ = pulling)
            tang = np.array([-math.cos(a), math.sin(a), 0.0])
            lead = float((t - g) @ tang)
            _say(f"R tcp ({t[0]:.4f},{t[1]:.4f},{t[2] - ZH:.4f}) lead {lead * 1000:.1f} mm; bolt {th:.2f} deg, "
                 f"handle {d:.2f} deg, armed {m['armed']}, clicks {m['clicks']}, peak {m['peak']:.2f} "
                 f"({peak_pct(m['peak']):.0f} %), over {m['over']} loose {m['loose']} still {m['still']}")


kit.on_step(_rules)


def torqued(objects) -> bool:
    """In tolerance: the wrench clicked, the peak torque was 90-110 % of the set torque (the bolt's tightest 3.9-5.9
    deg past snug) and the bolt is still within 1 deg of its tightest."""
    m = _mem()
    return bool(m["clicks"] > 0 and TOL_LO_DEG <= m["peak"] <= TOL_HI_DEG
                and bolt_deg(objects) >= m["peak"] - SETBACK_DEG)


def score(objects) -> float:
    """0 (latched) if the peak torque ever went over 110 % of the set torque (the bolt more than 5.9 deg past snug;
    it yields at 230 %, 17.7 deg) or the bolt was turned back more than 3 deg from its tightest. Otherwise 1.0 for
    torqued (clicked, peak 90-110 %, the bolt still within 1 deg of its tightest), else 0.25 for turned on at least
    1.5 deg (under-torqued). Halved unless both grippers are 10 cm outside the wrench's bounding box and the bolt
    and grip are still."""
    if mk._EP.get("env") is None and _mem().get("last") is not objects:  # offline (once per state); in sim the
        _mem()["last"] = objects  # per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["over"] or m["loose"]:
        return 0.0
    th = bolt_deg(objects)
    base = 1.0 if torqued(objects) else (0.25 if th >= TURNED_DEG else 0.0)
    clear = _released(objects["wrench_head"]) and _released(objects["handle"]) and m["still"]
    return round(base if clear else base / 2, 6)


def success(objects) -> bool:
    """Torqued in tolerance (clicked, peak 90-110 %, not backed off), let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """2 if over-torqued or backed off; else the click still to come plus the turn still missing as a fraction."""
    m = _mem()
    if m["over"] or m["loose"]:
        return 2.0
    return float((m["clicks"] == 0) + max(0.0, TOL_LO_DEG - bolt_deg(objects)) / TOL_LO_DEG)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry and the wrench's set torque; right arm only, the left stays parked. The
# fingers, opened 36 mm, straddle the 18 mm grip (turned so they close across the handle) and the hand pulls the
# grip round the bolt toward the robot, its far pad on the grip (9 mm play). Under load the arm lags its command
# (sim runs: about 3 deg, 11 mm, at 2 N), so the wrench clicks with the hand commanded about 11.3 deg round; the
# handle's 6 deg of free swing then takes up the hand's travel and the lag it lets go of, and the bolt would not
# turn on again until about 14 deg. The hand stops at 13.5 deg, eases back 1.5 deg, opens, lifts 7 cm and moves
# away to park, over 10 cm from the wrench, before the success hold starts.

TIP = 0.013  # pad tips below the TCP
PAD_DOWN = 0.006  # pad tips this far below the grip's centre (3 mm above its underside)
STRADDLE = -0.018  # finger target: pads 36 mm apart round the 18 mm grip (9 mm play each side)
WIDE = -0.030
PULL_DEG = {"demo": 13.5, "through": 45.0, "short": 6.0}
PULL_S = {"demo": 5.0, "through": 12.0, "short": 3.0}
EASE_DEG = 1.5
STEP_DEG = 1.5
PARK = (0.150, -0.300, TOP + 0.200)
LIFT = 0.070  # straight up off the grip after letting go, before moving away to PARK


def _grip_xy(deg):
    a = math.radians(deg)
    return (BX - R_GRIP * math.sin(a), BY - R_GRIP * math.cos(a))


def demo():
    """Straddle the grip, pull it round the bolt until just past where the wrench clicks, ease off, let go, lift
    clear and park."""
    tz = ZH - PAD_DOWN + TIP
    R = [Waypoint(2.0, PARK, WIDE, "start", math.pi / 2)]

    def w(dur, deg, z, f, label):
        x, y = _grip_xy(deg)
        R.append(Waypoint(dur, (x, y, z), f, label, math.pi / 2 - math.radians(deg)))

    def arc(a, b, dur, label):
        n = max(1, math.ceil(abs(b - a) / STEP_DEG))
        for k in range(1, n + 1):
            w(dur / n, a + (b - a) * k / n, tz, STRADDLE, label)

    pull = PULL_DEG[VARIANT]
    w(2.0, 0.0, tz + 0.060, STRADDLE, "over the grip, fingers part open")
    w(1.5, 0.0, tz, STRADDLE, "down round the grip")
    w(0.5, 0.0, tz, STRADDLE, "settle")
    if VARIANT == "through":  # the naive ablation: a pause where the demo stops, then on round to 45 deg
        arc(0.0, PULL_DEG["demo"], PULL_S["demo"], "pull the grip round, steadily")
        w(1.0, PULL_DEG["demo"], tz, STRADDLE, "pause where the demo stops")
        arc(PULL_DEG["demo"], pull, PULL_S[VARIANT], "pull on through the click")
    else:
        arc(0.0, pull, PULL_S[VARIANT], "pull the grip round, steadily")
    w(0.6, pull, tz, STRADDLE, "stop: it clicked")
    w(0.6, pull - EASE_DEG, tz, STRADDLE, "ease off")
    w(0.5, pull - EASE_DEG, tz, WIDE, "open the fingers")
    w(1.0, pull - EASE_DEG, tz + LIFT, WIDE, "lift clear")
    R.append(Waypoint(2.0, PARK, WIDE, "move away (over 10 cm from the wrench)", math.pi / 2))
    R.append(Waypoint(2.5, PARK, WIDE, "wait", math.pi / 2))
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), -0.04695, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), -0.04695, "park")]
    return {"left": left, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed one control step at a time)."""
    global _TEST_TCPS
    on = (*_grip_xy(0.0), ZH - PAD_DOWN + TIP)  # a gripper still round the grip

    def state(th, d):
        return kit.fake_states(q={"bolt": math.radians(th), "breakover": math.radians(d)})

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def load(th):  # the detent's bend while the bolt turns at ``th`` (armed, pulled at the grip's centre)
        return math.degrees(bolt_torque(th) * (R_GRIP - R_PIVOT) / R_GRIP / K_DETENT)

    def pull(a, b):  # the bolt turned on from a to b through the armed detent (no click on the way)
        return [(t, min(load(t), CLICK_DEG - 0.01)) for t in lin(a, b)]

    def click(th):  # the detent breaks over at the set torque, the handle swings to its stop
        return [(th, CLICK_DEG + 0.05), (th, 4.0), (th, BREAK_DEG), (th, BREAK_DEG)]

    def let_go(th, d0=BREAK_DEG, n=6):  # load off: the handle springs back in line from d0, then all still
        return [(th, d) for d in lin(d0, 0.0, 4)[1:]] + [(th, 0.0)] * n

    s = SPEC_DEG
    good = pull(0.0, s) + click(s + 0.1) + let_go(s + 0.1)
    cases = {  # name: (frames (bolt deg, handle deg), TCPs at the end, score)
        "start": ([(0.0, 0.0)], [], 0.0),
        "pulled until it clicked, stopped, let go (demo)": (good, [], 1.0),
        "the same, the hand still on the grip": (good, [on], 0.5),
        "the same, the open hand 5 cm above the grip": (good, [(on[0], on[1], on[2] + 0.05)], 0.5),
        "the same, the grip still swinging back at the end": (pull(0.0, s) + click(s + 0.1) + [(s + 0.1, 3.0)],
                                                              [], 0.5),
        "stopped short: bolt 3.5 deg on, no click (under-torqued)": (pull(0.0, 3.5) + let_go(3.5, load(3.5)), [], 0.25),
        "pulled on through the click to 20 deg (yielded)": (pull(0.0, s) + click(s + 0.1)
                                                            + [(t, BREAK_DEG) for t in lin(s + 0.1, 20.0)]
                                                            + let_go(20.0), [], 0.0),
        "naive: swung the handle round 80 deg": (pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, 80.0)]
                                                 + let_go(80.0), [], 0.0),
        "clicked, springs back 0.5 deg as the hand lets go": (pull(0.0, s) + click(s + 0.1) + let_go(s - 0.4), [],
                                                               1.0),
        "pulled on 0.8 deg past the click (106 %, inside the tolerance)": (pull(0.0, s) + click(s)
                                                                           + [(t, BREAK_DEG) for t in lin(s, s + 0.8)]
                                                                           + let_go(s + 0.8), [], 1.0),
        "pulled on 1.2 deg past the click (112 %, over the tolerance)": (pull(0.0, s) + click(s)
                                                                         + [(t, BREAK_DEG) for t in lin(s, s + 1.2)]
                                                                         + let_go(s + 1.2), [], 0.0),
        "pulled on 4 deg past the click (128 %)": (pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, s + 4.0)]
                                                   + let_go(s + 4.0), [], 0.0),
        "pulled on to 14.5 deg, just under the yield (197 %)": (pull(0.0, s) + click(s)
                                                                + [(t, BREAK_DEG) for t in lin(s, 14.5)]
                                                                + let_go(14.5), [], 0.0),
        "over by 1.2 deg, then eased back into the band (latched)": (pull(0.0, s) + click(s)
                                                                     + [(t, BREAK_DEG) for t in lin(s, s + 1.2)]
                                                                     + [(t, -1.0) for t in lin(s + 1.2, s + 0.4)]
                                                                     + let_go(s + 0.4, -1.0), [], 0.0),
        "clicked, then backed off to 1.5 deg (torque undone)": (good + [(t, -1.0) for t in lin(s + 0.1, 1.5)]
                                                                + let_go(1.5, -1.0), [], 0.0),
        "clicked, then eased back 2 deg (now under the torque)": (good + [(t, -1.0) for t in lin(s + 0.1, s - 1.9)]
                                                                  + let_go(s - 1.9, -1.0), [], 0.25),
        "loosened from snug: turned 4 deg the wrong way": ([(t, -1.0) for t in lin(0.0, -4.0)] + let_go(-4.0, -1.0),
                                                           [], 0.0),
        "turned 4.5 deg by the head, no click (torque not read)": ([(t, 0.0) for t in lin(0.0, 4.5)]
                                                                   + let_go(4.5, 0.0), [], 0.25),
        "turned 8 deg by the head, no click (126 %, over)": ([(t, 0.0) for t in lin(0.0, 8.0)] + let_go(8.0, 0.0),
                                                             [], 0.0),
        "clicked, let go, clicked again to check (bolt 5.3 deg)": (good + pull(s + 0.1, s + 0.4) + click(s + 0.4)
                                                                   + let_go(s + 0.4), [], 1.0),
        "a jerk kinks the handle with the bolt only 2 deg on": (pull(0.0, 2.0) + click(2.0) + let_go(2.0), [],
                                                                0.25),
        "untouched, hand resting on the grip": ([(0.0, 0.0)] * 5, [on], 0.0),
    }
    try:
        assert abs(break_deg(state(3.0, 2.0)) - 2.0) < 1e-6 and abs(bolt_deg(state(3.0, 2.0)) - 3.0) < 1e-6
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for th, d in frames[:-1]:
                score(state(th, d))
            _TEST_TCPS = list(tcps)
            st = state(*frames[-1])
            got = score(st)
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
            assert success(st) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        for th, d in good:
            score(state(th, d))
        clicks = _mem()["clicks"]
        mk.new_episode()
        assert score(state(0.0, 0.0)) == 0.0 and _mem()["clicks"] == 0, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, clicks


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, clicks = self_test()
    print(f"scoring: {n} cases pass (set torque {T_CLICK:.2f} N*m = {F_CLICK:.1f} N at the grip, reached "
          f"{SPEC_DEG:.1f} deg past snug; tolerance 90-110 % = {TOL_LO_DEG:.1f}-{TOL_HI_DEG:.1f} deg; yield "
          f"{T_OVER:.2f} N*m = {F_OVER:.1f} N at {YIELD_DEG:.1f} deg; real joint x{1 / TORQUE_SCALE:.1f}: "
          f"{REAL_SET_NM:.0f} N*m set)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
