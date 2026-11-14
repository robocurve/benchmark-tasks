"""On the station table, in front of the right arm, a section of a delivery vehicle's load deck carries a small wooden
crate (14 cm tall on a 5.5 x 10 cm base, so it tips easily front to back). A ratchet tie-down is already routed over
it the way loads are tied down on a flatbed: from a floor D-ring on the deck's left edge, up the crate's left side,
across its top and down its right side to a lever ratchet whose hook is in a second D-ring on the deck's right edge.
The ratchet lies along that right leg; its mandrel turns about an axis across the webbing (along x), and its handle,
a steel lever with a red grip bar across its end, swings up and out from the leg. The strap is slack: 12 mm more
webbing than the path over the crate needs. The handle stands part open, 40 deg.

The strap is a chain of rigid webbing links hinged across their width (it bends over the crate's edges, it cannot
stretch): the left link on the left D-ring, the top link lying on the crate, the right link into the ratchet, where a
strap end slides into the mandrel. The take-up is a spring of 8 kN/m (the webbing's stretch) from that strap end to
the wound length, so the strap's tension is that spring's force, and the crate is held down by the links' contact
with its top. The ratchet works as a real one does. Each full stroke of the handle (swung out past 75 deg and back in
under 35 deg) turns the mandrel one tooth and winds 5 mm of strap; a pawl holds what has been wound. Closing the
handle flat needs a firm push past its detent (a ridge pushes it back out to 12 deg until it is pressed within 1 deg
of shut); there it latches and stays. A handle opened right out to its release position (130 deg) lifts the pawls
and the strap pays out.

Then the test, the jolt of a vehicle braking. Once the robot has let go and stood clear for 1 s, the deck (a 20 kg
body on a slide along x) is pulled forward gently (0.5 m/s^2) and runs 15 cm into its end stop, a hard stop at about
0.38 m/s. A strap at 50 N or more presses the crate down hard enough that it neither slides nor tips. A slack strap
lets the crate pitch forward over its front edge until the webbing catches it.

The job: tension the strap with full strokes until it is taut, then close the handle flat into its latch, let go
and stand clear.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py \\
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
DEBUG = False  # sim only: print the mechanism's events and a readout every 5 steps
# Sim controls: "demo"; "slack" (1 stroke, latched: the strap is slack); "overswing" (the handle swung out past its
# release position on the first stroke); "knock" (the crate shoved before strapping); "open" (4 strokes, handle left up)
VARIANT = "demo"

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the deck section: a 20 kg body on a slide along x (the jolt), hovering 2 mm over the table ----------------------
HOVER, DECK_T = 0.002, 0.010
DZ = TOP + HOVER + DECK_T  # deck top
DECK_O = (0.290, -0.110, TOP + HOVER)  # deck part origin (plate bottom centre)
DECK_SIZE = (0.240, 0.320)
DECK_KG = 20.0
RUN = 0.150  # travel to the end stop
K_DECK, FAR = 2.0, 5.0  # drive: target -5 m holds it on its rear stop with 10 N; +5 m pulls it forward with 10 N
# -- the crate (free, standing on the deck at its marks) -------------------------------------------------------------
CRATE = (0.055, 0.100, 0.140)  # x (front to back), y, z
CX, CY = 0.283, -0.110  # bottom centre
CRATE_KG = 1.0
# -- the tie-down: 3 cm webbing over the crate's top at x = CX, D-rings at the deck's edges ---------------------------
BAND_W, STRAP_T = 0.030, 0.003  # webbing width (along x) and the links' thickness
GAP = 0.012  # slack: this much more webbing than the taut path over the crate
PITCH = 0.005  # strap wound per tooth (one full stroke)
K_S = 8000.0  # webbing stiffness, N/m (40 N per tooth once the strap is tight on the crate)
MAX_TEETH = 6
P_MIN = 50.0  # taut: at least 50 N in the strap
RING_Z = 0.008  # the D-rings' bar above the deck top
A_L = np.array([0.025, DZ + RING_Z])  # left D-ring (y, z): the strap's fixed end
A_R = np.array([-0.245, DZ + RING_Z])  # right D-ring: the ratchet's hook
_E = 0.0015  # the strap's centre line runs this far off the crate's faces
E_R = np.array([CY - CRATE[1] / 2 - _E, DZ + CRATE[2] + _E])  # right top edge (taut path)
E_L = np.array([CY + CRATE[1] / 2 + _E, DZ + CRATE[2] + _E])  # left top edge
U = (E_R - A_R) / np.linalg.norm(E_R - A_R)  # the right leg's line, D-ring to edge (y, z)
N = np.array([-U[1], U[0]])  # its outward normal, up and away from the crate
M_UP = 0.085  # the mandrel: this far up the leg from the right D-ring
M = A_R + M_UP * U  # mandrel axis (y, z); the strap leaves the mandrel here when taut
TAKE_MIN, TAKE_MAX = -0.030, 0.040
LINK_KG = {"strap_r": 0.020, "strap_t": 0.030, "strap_l": 0.035, "strap_end": 0.015}  # 100 g of webbing
LINK_ITERS = (32, 4)
# -- the ratchet handle: on the mandrel axis; 0 deg = shut, lying back down the leg; opens up and away ------------
R_ARM, CRANK = 0.064, 0.025  # grip bar centre: 6.4 cm back along the leg and 2.5 cm out from it when shut
BAR_L, BAR_R = 0.046, 0.005  # the red grip bar across the handle's end: 4.6 cm long (along x), 1 cm round
PLATE_X = 0.025  # the handle's two side plates, either side of the webbing
HANDLE_KG = 0.06
H_FRICTION = 0.04  # N*m: the pawl springs hold the handle where it is left (its weight gives under 0.025 N*m)
START_DEG = 40.0
STROKE_OPEN, STROKE_BACK = 75.0, 35.0  # a stroke: out past 75 deg, then back under 35 deg
RIDGE_DEG = 12.0  # the detent's ridge pushes an unlatched handle back out to here
RIDGE_TORQUE = 0.04  # N*m, its push (0.6 N at the grip bar)
LATCH_DEG = 1.0  # pressed this far shut, the latch engages
SHUT_DEG = 1.5  # latched and shut
HOLD_DEG = -8.0  # latched: the catch holds it on its stop (drive target past the stop)
RELEASE_DEG = 130.0  # handle opened right out (standing up over the leg): pawls lifted, strap pays out
PARTS = ("crate", "strap_l", "strap_t", "strap_r", "handle")

WOOD, STEEL, WEB = (0.72, 0.55, 0.34), (0.30, 0.31, 0.33), (0.95, 0.62, 0.10)
DECK_C, GRIP_C, ZINC = (0.32, 0.33, 0.35), (0.80, 0.12, 0.10), (0.62, 0.64, 0.66)


def _circles(c1, r1, c2, r2):
    """The upper intersection of two circles in the (y, z) plane."""
    d = np.linalg.norm(c2 - c1)
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(max(r1 * r1 - a * a, 0.0))
    m = c1 + a * (c2 - c1) / d
    perp = np.array([-(c2 - c1)[1], (c2 - c1)[0]]) / d
    p, q = m + h * perp, m - h * perp
    return p if p[1] >= q[1] else q


# The strap's links (lengths of the taut path) and its slack spawn pose: the strap end sits GAP further out of the
# mandrel, so the webbing over the top stands up off the crate, most on the ratchet's side (it settles onto the crate).
L_R = float(np.linalg.norm(E_R - M))
L_T = float(np.linalg.norm(E_L - E_R))
L_L = float(np.linalg.norm(A_L - E_L))
STRAP_LEN = L_R + L_T + L_L  # anchor to mandrel, taut
M0 = M + GAP * U  # the strap end's hinge at spawn
_SLACK_DY = 0.0035  # spawn: the top link's right end this far in from the taut path (all links >= 2 mm off the crate)
P_R = np.array([E_R[0] + _SLACK_DY, M0[1] + math.sqrt(L_R ** 2 - (E_R[0] + _SLACK_DY - M0[0]) ** 2)])
P_L = _circles(P_R, L_T, A_L, L_L)  # left end of the top link at spawn
LIFT = float(P_R[1] - E_R[1])


def _w(yz, x=CX):
    return (float(x), float(yz[0]), float(yz[1]))


def _xrot(angle):
    """xyzw quaternion of a turn ``angle`` (rad) about +x."""
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def _link_box(a, b, width=BAND_W, thick=STRAP_T, color=None, at_mid=True):
    """A box from (y, z) point a to b in the strap's plane: its length along a->b, ``width`` along x."""
    d = np.asarray(b, float) - np.asarray(a, float)
    ang = math.atan2(d[1], d[0])
    return ang, float(np.linalg.norm(d)), (width, float(np.linalg.norm(d)), thick), color


kit = mk.Kit()
_dz = DZ - DECK_O[2]  # deck top in the deck frame


def _d(x, y, z):  # world -> deck frame
    return (x - DECK_O[0], y - DECK_O[1], z - DECK_O[2])


def _deck_boxes():
    out = [mk.box((DECK_SIZE[0], DECK_SIZE[1], DECK_T), at=(0, 0, DECK_T / 2))]  # chequer-plate steel deck section
    for ay in (A_L[0], A_R[0]):  # floor D-rings: a base plate, two uprights, the ring's bar along x
        out.append(mk.box((0.044, 0.018, 0.003), at=_d(CX, ay, DZ + 0.0015), color=ZINC))
        for sx in (-1, 1):
            out.append(mk.box((0.004, 0.006, RING_Z + 0.003), at=_d(CX + sx * 0.020, ay, DZ + (RING_Z + 0.003) / 2),
                              color=ZINC))
        out.append(mk.box((0.044, 0.005, 0.005), at=_d(CX, ay, DZ + RING_Z), color=ZINC))
    # the ratchet's frame, lying along the right leg: hook in the D-ring, two side plates up to the mandrel's cheeks
    ang = math.atan2(U[1], U[0])
    hook = A_R + 0.010 * U
    out.append(mk.box((0.024, 0.020, 0.004), at=_d(CX, *hook), quat=_xrot(ang), color=STEEL))  # hook
    mid = A_R + (0.020 + (M_UP - 0.020) / 2) * U - 0.004 * N
    for sx in (-1, 1):
        out.append(mk.box((0.003, M_UP - 0.014, 0.018), at=_d(CX + sx * 0.0185, *mid), quat=_xrot(ang), color=STEEL))
        out.append(mk.box((0.003, 0.024, 0.024), at=_d(CX + sx * 0.0185, *M), quat=_xrot(ang), color=STEEL))  # cheeks
    fixed_bar = A_R + 0.022 * U - 0.004 * N
    out.append(mk.box((0.040, 0.006, 0.006), at=_d(CX, *fixed_bar), quat=_xrot(ang), color=STEEL))
    # marks painted on the deck at the crate's corners
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(mk.box((0.012, 0.012, 0.0004), at=_d(CX + sx * (CRATE[0] / 2 + 0.004),
                                                             CY + sy * (CRATE[1] / 2 + 0.004), DZ + 0.0002),
                              color=(0.95, 0.85, 0.15)))
    return out


kit.part("deck", DECK_O, _deck_boxes(), mass=DECK_KG, color=DECK_C)
kit.slider("deck_run", None, "deck", axis=(1, 0, 0), lower=0.0, upper=RUN, start=0.0, stiffness=K_DECK, rest=-FAR)

kit.part("crate", (CX, CY, DZ), [
    mk.box(CRATE, at=(0, 0, CRATE[2] / 2)),
    mk.box((CRATE[0] + 0.002, 0.012, 0.010), at=(0, -0.030, CRATE[2] - 0.020), color=(0.55, 0.40, 0.24)),  # slats
    mk.box((CRATE[0] + 0.002, 0.012, 0.010), at=(0, 0.030, CRATE[2] - 0.020), color=(0.55, 0.40, 0.24)),
], mass=CRATE_KG, color=WOOD, iters=LINK_ITERS)


def _link(name, a, b, extra=()):
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    ln = float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
    mid = (np.asarray(a) + np.asarray(b)) / 2
    kit.part(name, _w(mid), [mk.box((BAND_W, ln, STRAP_T)), *extra], mass=LINK_KG[name], color=WEB,
             quat=_xrot(ang), iters=LINK_ITERS)


# the strap end in the ratchet (slides into the mandrel), then the three links back to the left D-ring
_link("strap_end", M0 - 0.024 * U, M0)
_link("strap_r", M0, P_R)
_link("strap_t", P_R, P_L)
_link("strap_l", P_L, A_L)
# take-up: positive winds the strap end down into the mandrel (along -U)
kit.slider("take_up", "deck", "strap_end", axis=(0, -U[0], -U[1]), lower=TAKE_MIN, upper=TAKE_MAX, start=0.0,
           stiffness=K_S, rest=0.0)
kit.hinge("strap_mandrel", "strap_end", "strap_r", anchor=_w(M0), axis=(1, 0, 0), lower_deg=-120, upper_deg=120)
kit.hinge("strap_edge_r", "strap_r", "strap_t", anchor=_w(P_R), axis=(1, 0, 0), lower_deg=-150, upper_deg=150)
kit.hinge("strap_edge_l", "strap_t", "strap_l", anchor=_w(P_L), axis=(1, 0, 0), lower_deg=-150, upper_deg=150)
kit.hinge("strap_anchor", "strap_l", "deck", anchor=_w(A_L), axis=(1, 0, 0), lower_deg=-150, upper_deg=150, loop=True)


def _handle_boxes():
    """The handle in its shut pose, about its pin on the mandrel axis (part frame at the pin, world-aligned)."""
    bar = R_ARM * (-U) + CRANK * N
    ang = math.atan2(bar[1], bar[0])
    mid = bar / 2
    out = []
    for sx in (-1, 1):
        out.append(mk.box((0.004, float(np.linalg.norm(bar)) + 0.008, 0.012), at=(sx * PLATE_X, *mid),
                          quat=_xrot(ang), color=STEEL))  # side plates, pin to grip
        out.append(mk.box((0.004, 0.016, 0.016), at=(sx * PLATE_X, 0.0, 0.0), color=STEEL))  # knuckles on the pin
    for k in range(6):  # red PVC-sleeved grip bar, round: six 10 x 3 mm slats about its axis (radius 5.0-5.2 mm)
        out.append(mk.box((BAR_L, 2 * BAR_R, 0.003), at=(0.0, *bar), quat=_xrot(math.radians(30 * k)), color=GRIP_C))
    return out


kit.part("handle", _w(M), _handle_boxes(), mass=HANDLE_KG, color=STEEL)
# the hinge's +angle opens the handle: from lying back down the leg (-U) toward N (a turn about -x)
kit.hinge("handle_hinge", "deck", "handle", anchor=_w(M), axis=(-1, 0, 0), lower_deg=0.0, upper_deg=170.0,
          start_deg=START_DEG, friction=H_FRICTION)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def bar_yz(deg) -> np.ndarray:
    """The grip bar's centre (y, z) with the handle ``deg`` open (deck at its start)."""
    p0 = R_ARM * (-U) + CRANK * N
    t = math.radians(deg)
    x1, x2 = float(p0 @ -U), float(p0 @ N)
    return M + (math.cos(t) * x1 - math.sin(t) * x2) * (-U) + (math.sin(t) * x1 + math.cos(t) * x2) * N


# -- the mechanism's per-step rules (sim) and the scoring ---------------------------------------------------------
DT = 0.05  # control step
QUIET_STEPS = 20  # 1 s let go and still before the jolt
CLEAR = 0.03  # "stood clear": both TCPs 3 cm from the crate, strap and handle
SETTLE_STEPS = 20  # the jolt is over 1 s after the stop
KNOCK_MM, KNOCK_DEG = 3.0, 2.0  # before the jolt: the crate must not have been moved more than this
SHIFT_MM, TILT_DEG = 2.5, 2.0  # through the jolt: the crate must stay within this of where it stood
WIND_STEP = 0.001  # the mandrel winds a tooth's strap in over 5 control steps
M_KEY = "rslt"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M_KEY)
    if m is None:
        m = mk.MEMORY[M_KEY] = {"n": 0, "init": False, "teeth": 0, "armed": False, "cmd": 0.0, "latched": False,
                                "released": False, "worked": False, "knocked": False, "quiet": 0, "jolt": None,
                                "t_hit": None, "d_prev": 0.0, "base": None, "shift": 0.0, "tilt": 0.0,
                                "shifted": False, "t_trip": 0.0, "t_max": 0.0, "prev": {}, "still": {}}
    return m


def handle_deg(objects) -> float:
    """How far the handle is open (deg; 0 = shut flat)."""
    return kit.q_deg(objects, "handle_hinge")


def take_up(objects) -> float:
    """How far the strap end has been drawn into the mandrel from where it started (m), from the poses of the strap
    end and the deck."""
    return kit.q(objects, "take_up")


def deck_travel(objects) -> float:
    """How far the deck has run forward from its rear stop (m), at its own origin."""
    return float(kit._pose_T(objects, "deck")[0, 3] - DECK_O[0])


def wound(m=None) -> float:
    """Strap wound onto the mandrel (m): 5 mm per tooth, as wound in so far."""
    return (m or _mem())["cmd"]


def tension(objects) -> float:
    """Strap tension (N): the take-up spring's force, i.e. the webbing's stretch (wound minus how far the strap end
    has actually come in, which the strap over the crate resists) times its stiffness."""
    m = _mem()
    if m["released"]:
        return 0.0
    return max(0.0, K_S * (wound(m) - take_up(objects)))


_DECK_REF = kit.parts["deck"].ref
_CRATE_REF = kit.parts["crate"].ref
_REL0 = mk._inv(_DECK_REF) @ _CRATE_REF  # the crate in the deck's frame at the start
_PTS = [np.array([0, 0, 0.0]), np.array([0, 0, CRATE[2]])]  # bottom and top centre (crate frame)


def _crate_rel(objects) -> np.ndarray:
    return mk._inv(kit._pose_T(objects, "deck")) @ kit._pose_T(objects, "crate")


def _moved(rel, ref) -> tuple:
    """(mm, deg) the crate has moved in the deck's frame, from ``ref``: the larger move of its bottom and top
    centres, and its turn."""
    d = max(float(np.linalg.norm(rel[:3, :3] @ p + rel[:3, 3] - ref[:3, :3] @ p - ref[:3, 3])) for p in _PTS)
    return d * 1000.0, math.degrees(mk._rot_angle(rel[:3, :3] @ ref[:3, :3].T))


def _set_drive(name, target=None, stiffness=None) -> None:
    """Sim only: set joint ``name``'s drive target (rad or m) and/or stiffness (SI) now."""
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    j = kit.joints_[name]
    hinge = j.kind == "hinge"
    scale = 180.0 / math.pi if hinge else 1.0
    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular" if hinge else "linear")
    if target is not None:
        if j.friction > 0:
            mk._SIM["stick"][(kit.id, name)] = target
        d.GetTargetPositionAttr().Set(float(target * scale))
    if stiffness is not None:
        d.GetStiffnessAttr().Set(float(stiffness / scale))


_K_STICK = mk._k_stick(kit.joints_["handle_hinge"])


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit rslt] step {mk._EP.get('step')}: {msg}", flush=True)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _rules(objects) -> None:
    """One control step of the ratchet (strokes, pawl, detent, latch, release) and of the trip (the jolt)."""
    m = _mem()
    sim = mk._EP.get("env") is not None
    m["n"] += 1
    if not m["init"]:  # a new episode: deck on its rear stop, strap slack and held, handle on its friction
        m["init"] = True
        _set_drive("deck_run", target=-FAR)
        _set_drive("take_up", target=0.0, stiffness=K_S)
        _set_drive("handle_hinge", stiffness=_K_STICK)
    for p in (*PARTS, "deck"):  # still: under 0.5 mm and 1.4 deg of motion in a control step (10 mm/s, 0.5 rad/s)
        t = kit._pose_T(objects, p)
        t0 = m["prev"].get(p)
        m["still"][p] = t0 is None or bool(np.linalg.norm(t[:3, 3] - t0[:3, 3]) < 0.0005
                                           and mk._rot_angle(t[:3, :3] @ t0[:3, :3].T) < 0.5 * DT)
        m["prev"][p] = t
    a = handle_deg(objects)
    if abs(a - START_DEG) >= 15.0:
        m["worked"] = True
    # -- the handle: release, strokes, detent and latch
    if not m["released"] and a >= RELEASE_DEG:
        m["released"] = True
        _set_drive("take_up", stiffness=0.0)
        _say(f"LATCH released: handle opened to {a:.1f} deg, pawls lifted, strap paying out")
    if not m["latched"] and not m["released"]:
        if a >= STROKE_OPEN:
            m["armed"] = True
        elif m["armed"] and a <= STROKE_BACK:
            m["armed"] = False
            if m["teeth"] < MAX_TEETH:
                m["teeth"] += 1
                _say(f"stroke: tooth {m['teeth']}")
        if a <= LATCH_DEG:
            m["latched"] = True
            _say(f"handle LATCHED at {a:.2f} deg, {m['teeth']} teeth, tension {tension(objects):.0f} N")
    if m["latched"]:
        _set_drive("handle_hinge", target=math.radians(HOLD_DEG))
    elif sim and a < RIDGE_DEG:  # the ridge: a steady push out (the drive's target just past where the handle is)
        _set_drive("handle_hinge", target=math.radians(a) + RIDGE_TORQUE / _K_STICK)
    # -- the pawl: the mandrel winds the wound strap in, a tooth over 5 steps
    if not m["released"]:
        goal = m["teeth"] * PITCH
        if m["cmd"] < goal:
            m["cmd"] = min(goal, m["cmd"] + WIND_STEP)
            _set_drive("take_up", target=m["cmd"])
    # -- the crate before the trip: nobody may shove it about
    if m["jolt"] is None:
        mm, deg = _moved(_crate_rel(objects), _REL0)
        if mm > KNOCK_MM or deg > KNOCK_DEG:
            if not m["knocked"]:
                _say(f"LATCH knocked: crate moved {mm:.1f} mm / {deg:.1f} deg before the trip")
            m["knocked"] = True
    # -- the trip: 1 s after the robot has let go and stood clear, the deck runs into its stop
    d = deck_travel(objects)
    if m["jolt"] is None:
        quiet = (m["worked"] and all(_released(objects[p], CLEAR) for p in PARTS)
                 and all(m["still"][p] for p in (*PARTS, "deck")))
        m["quiet"] = m["quiet"] + 1 if quiet else 0
        if m["quiet"] >= QUIET_STEPS:
            m["jolt"], m["base"] = "go", _crate_rel(objects)
            m["t_trip"] = tension(objects)
            _set_drive("deck_run", target=FAR)
            _say(f"TRIP: deck pulled forward; tension {m['t_trip']:.0f} N, handle {a:.1f} deg, "
                 f"latched {m['latched']}")
    elif m["jolt"] == "go":
        v = (d - m["d_prev"]) / DT
        if d >= RUN - 0.002:
            m["jolt"], m["t_hit"] = "hit", m["n"]
            _say(f"deck on its stop at about {v:.2f} m/s")
    if m["jolt"] == "hit" and m["n"] - m["t_hit"] >= SETTLE_STEPS:
        m["jolt"] = "done"
        _say(f"trip over: crate moved at most {m['shift']:.1f} mm / {m['tilt']:.1f} deg, peak tension "
             f"{m['t_max']:.0f} N")
    if m["jolt"] is not None:
        mm, deg = _moved(_crate_rel(objects), m["base"])
        m["shift"], m["tilt"] = max(m["shift"], mm), max(m["tilt"], deg)
        m["t_max"] = max(m["t_max"], tension(objects))
        if (mm > SHIFT_MM or deg > TILT_DEG) and not m["shifted"]:
            m["shifted"] = True
            _say(f"LATCH shifted: the crate moved {mm:.1f} mm / {deg:.1f} deg on the deck in the jolt")
    m["d_prev"] = d
    if DEBUG and sim and (m["n"] % 5 == 0 or m["jolt"] in ("go", "hit")):
        tc = mk.tcps()
        tz = f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - TOP:.3f})" if tc else "-"
        top = kit.point(objects, "strap_t", (0, 0, -STRAP_T / 2))[2] - (DZ + CRATE[2])
        _say(f"R tcp {tz}; handle {a:.1f} deg, teeth {m['teeth']}, wound {m['cmd'] * 1000:.1f} mm, in "
             f"{take_up(objects) * 1000:.1f} mm, T {tension(objects):.0f} N, top link {top * 1000:.1f} mm over the "
             f"crate, deck {d * 1000:.1f} mm, crate {m['shift']:.1f} mm/{m['tilt']:.1f} deg, jolt {m['jolt']}, "
             f"latched {m['latched']}, released {m['released']}")
        sp = " ".join(f"{p}:{int(m['still'][p])}" for p in (*PARTS, "deck"))
        mm0, deg0 = _moved(_crate_rel(objects), _REL0)
        _say(f"   still {sp}; crate from start {mm0:.2f} mm/{deg0:.2f} deg; quiet {m['quiet']}")


kit.on_step(_rules)


def taut(objects) -> bool:
    """At least 50 N in the strap."""
    return tension(objects) >= P_MIN


def latched(objects) -> bool:
    """The handle pressed past its detent into the latch, and shut flat now."""
    return bool(_mem()["latched"] and handle_deg(objects) <= SHUT_DEG)


def score(objects) -> float:
    """0 if the strap was let out (handle opened to its release position), the crate was shoved before the trip, or
    the crate moved more than 2.5 mm / 2 deg on the deck through the jolt. Otherwise 0.25 for a taut strap (50 N or
    more); with that, +0.25 for the handle latched shut; with both, +0.5 once the jolt is over. Halved unless both
    grippers are 2.5 cm clear of the crate, strap and handle and everything is still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["released"] or m["knocked"] or m["shifted"]:
        return 0.0
    base = 0.0
    if taut(objects):
        base = 0.25
        if latched(objects):
            base += 0.25
            if m["jolt"] == "done":
                base += 0.5
    clear = all(_released(objects[p]) and m["still"].get(p, True) for p in PARTS)
    return round(base if clear else base / 2, 6)


def success(objects) -> bool:
    """Strap taut, handle latched, the crate held down through the jolt; let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do (taut, latched, the trip) plus the tension still missing as a fraction of 50 N."""
    m = _mem()
    return float((not taut(objects)) + (not latched(objects)) + (m["jolt"] != "done")
                 + max(0.0, P_MIN - tension(objects)) / P_MIN)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; right arm only, the left stays parked. The gripper points straight down with
# its fingers closing along y. It comes down round the round 10 mm grip bar (which lies along x, between the
# handle's side plates), closes on it lightly (about 5 N a pad, so the bar can roll between the pads as the handle
# turns) and follows the bar's circle about the mandrel axis: out to 100 deg and back to 22 deg four times, then in
# past shut to press the handle onto its stop through the detent. Then it opens, centres on the bar, lifts clear and
# the arm parks and waits for the jolt.

TIP = 0.013  # pad tips below the TCP
DEEP, SHALLOW = 0.013, 0.006  # pad tips this far below the bar's centre: on the strokes; near shut (over the frame)
STRADDLE = 0.022  # finger opening round the bar
GRIP = -0.003  # finger target on the bar: pads 2 mm inside its surface each side (5000 N/m: about 10 N a pad)
GRIP_SHUT = -0.0045  # closing it: 0.5 mm (about 2.5 N a pad), so the bar rolls freely in the pads
OUT_DEG, BACK_DEG, SHUT_CMD = 100.0, 22.0, -6.0
PRESS_DY = 0.020  # the press aims this much further in (+y): the arm gives way sideways under the handle's push
STEP_DEG = 10.0
PARK = (0.200, -0.300, TOP + 0.200)


def _tip_below(deg) -> float:
    """How far the pad tips reach below the bar's centre: deep on the strokes, shallower near shut, where the
    ratchet's frame lies under the bar."""
    s = min(1.0, max(0.0, (deg - 8.0) / 22.0))
    return SHALLOW + (DEEP - SHALLOW) * s


def _bar_tcp(deg, up=0.0):
    y, z = bar_yz(deg)
    return (CX, float(y), float(z - _tip_below(deg) + TIP + up))


def demo():
    """Straddle the grip bar; four full strokes (out to 100 deg, back to 22 deg); press the handle shut through its
    detent; let go, lift off, park and wait while the deck is jolted."""
    sq = -STRADDLE / 2
    wide = -0.016
    R = [Waypoint(2.0, PARK, FINGER_OPEN * 0.5, "start", 0.0)]

    def w(dur, deg, f, label, up=0.0, dy=0.0):
        x, y, z = _bar_tcp(deg, up)
        R.append(Waypoint(dur, (x, y + dy, z), f, label, 0.0))

    def arc(a, b, dur, label, f=GRIP):
        n = max(1, math.ceil(abs(b - a) / STEP_DEG))
        for k in range(1, n + 1):
            w(dur / n, a + (b - a) * k / n, f, label)

    if VARIANT == "knock":  # control: shove the crate back near the top of its front face (pads reach 17.9 mm -x;
        # aimed 30 mm in, as the arm gives about 7 mm per N sideways and tipping the crate takes about 2 N up there)
        x0, xp, zk = CX + CRATE[0] / 2 + 0.035, CX + CRATE[0] / 2 + 0.0179 - 0.030, DZ + 0.135
        R.append(Waypoint(2.0, (x0, CY, DZ + 0.150), FINGER_CLOSED, "control: over the crate's front", 0.0))
        R.append(Waypoint(1.0, (x0, CY, zk), FINGER_CLOSED, "control: down in front of it", 0.0))
        R.append(Waypoint(1.0, (xp, CY, zk), FINGER_CLOSED, "control: shove it back", 0.0))
        R.append(Waypoint(1.0, (x0, CY, zk), FINGER_CLOSED, "control: back off", 0.0))
        R.append(Waypoint(1.0, (x0, CY, DZ + 0.150), FINGER_CLOSED, "control: up", 0.0))
    w(2.0, START_DEG, sq, "over the grip bar, fingers part open", up=0.060)
    w(1.5, START_DEG, sq, "down round the bar")
    w(0.4, START_DEG, sq, "settle")
    w(0.6, START_DEG, GRIP, "grip the bar lightly")
    strokes = {"demo": 4, "slack": 1, "overswing": 1, "knock": 4, "open": 4}[VARIANT]
    a = START_DEG
    for i in range(strokes):
        out = 148.0 if VARIANT == "overswing" else OUT_DEG
        arc(a, out, 1.6 * out / OUT_DEG, f"stroke {i + 1}: out")
        w(0.2, out, GRIP, "turn")
        arc(out, BACK_DEG, 1.6, f"stroke {i + 1}: back")
        w(0.2, BACK_DEG, GRIP, "turn")
        a = BACK_DEG
    last = BACK_DEG
    if VARIANT != "open":
        arc(BACK_DEG, SHUT_CMD, 1.5, "close the handle flat, through the detent", f=GRIP_SHUT)
        w(0.8, SHUT_CMD, GRIP_SHUT, "press it home", dy=PRESS_DY)
        w(0.5, 0.0, GRIP_SHUT, "ease off onto the shut bar")
        w(0.4, 0.0, sq, "let go of the bar")
        last = 0.0
    w(0.4, last, sq, "centre on the bar")
    w(1.0, last, sq, "lift straight up off the bar", up=0.060)
    w(0.5, last, wide, "open the fingers", up=0.060)
    R.append(Waypoint(2.0, PARK, wide, "park", 0.0))
    R.append(Waypoint(8.0, PARK, wide, "wait: the jolt", 0.0))
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park")]
    return {"left": left, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed one control step at a time). The
    strap end's travel is what the physics gives: it follows the wound strap until the strap is tight on the crate
    (GAP), then the webbing stretches."""
    global _TEST_TCPS
    up = (0.0, 0.0, 0.0, 1.0)
    near = _bar_tcp(0.0)  # a gripper still round the shut handle's grip bar

    def state(h, d=0.0, crate=(0.0, 0.0, 0.0), tilt=0.0, band=None):
        m = _mem()
        if band is None:
            band = TAKE_MIN if m["released"] else min(wound(m), GAP + 0.0005)
        pos = np.asarray(_CRATE_REF[:3, 3]) + np.array([d, 0, 0]) + np.asarray(crate)
        q = up if not tilt else tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(tilt))))
        return kit.fake_states(q={"handle_hinge": math.radians(h), "deck_run": d, "take_up": band},
                               moved={"crate": (tuple(pos), q)})

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def strokes(k, a0=START_DEG, end=BACK_DEG):  # handle angles of k full strokes (out to 96, back to 26)
        out, a = [], a0
        for _ in range(k):
            out += lin(a, 96.0) + lin(96.0, 26.0)
            a = 26.0
        return out + (lin(26.0, end) if end != 26.0 else [])

    def run(plan, tcps_end=()):
        """plan: a list of steps, each a handle angle (hand on the grip bar) or a dict for state() with "hand";
        returns the last score and state."""
        global _TEST_TCPS
        got = st = None
        for i, s in enumerate(plan):
            kw = dict(s) if isinstance(s, dict) else {"h": s}
            hand = kw.pop("hand", True)
            if i == len(plan) - 1:
                _TEST_TCPS = list(tcps_end)
            else:
                _TEST_TCPS = [_bar_tcp(kw["h"])] if hand else []
            st = state(**kw)
            got = score(st)
        return got, st

    def away(h, n=QUIET_STEPS + 2):  # let go and stand clear
        return [{"h": h, "hand": False}] * n

    def trip(h, shift=0.0, tilt=0.0, n_run=16, settle=SETTLE_STEPS + 3, band=None):
        """The deck's run to its stop, the crate shifted by ``shift`` m (and tilted) at its worst, then back."""
        out = []
        for k in range(1, n_run + 1):
            out.append({"h": h, "d": RUN * (k / n_run) ** 2, "hand": False, "band": band})
        for k in range(settle):
            s = shift if k < 4 else shift * 0.25
            out.append({"h": h, "d": RUN, "crate": (s, 0, 0), "tilt": tilt if k < 4 else 0.0, "hand": False,
                        "band": band})
        return out

    good = strokes(4, end=0.0) + away(0.0)
    cases = {  # name: (plan, TCPs at the last step, score)
        "start": ([START_DEG], [], 0.0),
        "four strokes, latched, let go, the jolt, the crate held down (demo)": (good + trip(0.0, 0.001), [], 1.0),
        "the same, the score read before the jolt": (good, [], 0.5),
        "four strokes, latched, hand kept on the handle (no jolt)":
            (strokes(4, end=0.0) + [0.0] * 30, [near], 0.25),
        "two strokes then latched (slack): the jolt pitches the crate 9 deg over its front edge":
            (strokes(2, end=0.0) + away(0.0) + trip(0.0, 0.012, tilt=9.0), [], 0.0),
        "three strokes then latched (24 N, not taut), held": (strokes(3, end=0.0) + away(0.0) + trip(0.0, 0.002),
                                                               [], 0.0),
        "four strokes then latched, but the crate jolted 3 mm on the deck": (good + trip(0.0, 0.003), [], 0.0),
        "latched and taut, but the crate tips 3 deg in the jolt": (good + trip(0.0, 0.001, tilt=3.0), [], 0.0),
        "four strokes, handle left 26 deg open (not latched), the crate held": (strokes(4) + away(26.0)
                                                                              + trip(26.0, 0.001), [], 0.25),
        "four strokes, handle left up at 96 deg, the crate held": (strokes(4)[:-9] + away(96.0)
                                                                  + trip(96.0, 0.001), [], 0.25),
        "over-swung to 155 deg mid-stroke (pawls lifted), then four strokes and latched":
            (lin(START_DEG, 155.0) + strokes(4, a0=155.0, end=0.0) + away(0.0) + trip(0.0, 0.001), [], 0.0),
        "the crate shoved 5 mm by the robot before strapping":
            ([{"h": START_DEG, "crate": (0, -0.005, 0)}] + strokes(4, end=0.0) + away(0.0) + trip(0.0, 0.001),
             [], 0.0),
        "six strokes (the most it winds), latched, the jolt, held": (strokes(8, end=0.0) + away(0.0)
                                                                     + trip(0.0, 0.0005), [], 1.0),
        "short strokes (out to 60 deg only, never counted), latched": (
            lin(START_DEG, 60.0) + (lin(60.0, 26.0) + lin(26.0, 60.0)) * 4 + lin(60.0, 0.0) + away(0.0)
            + trip(0.0, 0.0), [], 0.0),
        "demo, but a gripper still on the bar at the end": (good + trip(0.0, 0.001), [(near[0] + RUN, *near[1:])],
                                                             0.5),
        "four strokes, latched; the strap end pulled 4 mm back out by the jolt (88 N), held":
            (good + trip(0.0, 0.001, band=GAP + 0.0005 - 0.004), [], 1.0),
    }
    try:
        for name, (plan, tcps, want) in cases.items():
            mk.new_episode()
            got, st = run(plan, tcps)
            assert abs(got - want) < 1e-9, (name, got, want, {k: v for k, v in _mem().items() if k != "base"})
            assert success(st) == (want >= 1.0), name
        mk.new_episode()
        run(good + trip(0.0, 0.001))
        teeth = _mem()["teeth"]
        mk.new_episode()
        assert score(state(START_DEG)) == 0.0 and _mem()["teeth"] == 0, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, teeth


def _geometry_check():
    """The rig as built: the strap's lengths, the spawn pose closing the chain, the handle's reach."""
    # the slack spawn pose closes the chain: same link lengths, the strap end GAP out of the mandrel
    assert abs(np.linalg.norm(P_R - M0) - L_R) < 1e-9 and abs(np.linalg.norm(P_L - A_L) - L_L) < 1e-9
    assert abs(np.linalg.norm(P_L - P_R) - L_T) < 1e-9
    assert abs(np.linalg.norm(M0 - M) - GAP) < 1e-12
    assert LIFT > 0.0, "slack: the top link starts above the crate"
    sp = kit._spawn_poses()
    for name, (a, b) in {"strap_r": (M0, P_R), "strap_t": (P_R, P_L), "strap_l": (P_L, A_L)}.items():
        assert np.linalg.norm(sp[name][:3, 3] - np.array(_w((a + b) / 2))) < 1e-9, name
    zs = [_bar_tcp(a)[2] - TOP for a in np.linspace(SHUT_CMD, OUT_DEG, 30)]
    assert max(zs) <= 0.18, max(zs)
    return zs


if __name__ == "__main__":
    # Offline checks, no GPU: the rig's geometry, the scoring rules on hand-built states, then both arms' plans
    # replayed through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    zs = _geometry_check()
    print(f"strap: {STRAP_LEN * 100:.1f} cm D-ring to mandrel when taut (legs {L_L * 100:.1f} / {L_R * 100:.1f} cm "
          f"at {math.degrees(math.atan2(U[1], U[0])):.0f} deg, {L_T * 100:.1f} cm over the top), "
          f"{GAP * 1000:.0f} mm slack (top link's ratchet end {LIFT * 1000:.1f} mm up at spawn); "
          f"grip TCP {min(zs) * 100:.1f}-{max(zs) * 100:.1f} cm above the table")
    n, teeth = self_test()
    print(f"scoring: {n} cases pass (demo winds {teeth} teeth: {K_S * (teeth * PITCH - GAP - 0.0005):.0f} N in the "
          f"strap)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
