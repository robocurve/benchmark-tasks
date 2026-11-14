"""A handheld carton-sealing tape gun lies on its side on the bench, its empty hub up. A new 48 mm roll of packing
tape lies on the bench beside it, its leader wound round it with a folded tab at the end.

The job: push the roll's 76 mm core onto the spring hub until it clicks past the detent ball (12 N), peel the leader,
lead it round the outside of the sprung tension roller, swing the blade guard open, lay the leader along the nose so
its end sticks out past the blade, and let the guard close on it. One hand holds the gun by its handle while the
other works; the gun is free on the bench and can be pushed around, tipped or knocked off it.

The tape leader is eight rigid links joined by hinges (the adhesive side is each link's local -y face). Mechanism
parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/reload-handheld-tape-gun/reload_handheld_tape_gun.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 2000 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
ZP = TOP + 0.006  # the gun's side plate face

GUN_C = (0.065, 0.068, 0.072)  # dark grey body
STEEL = (0.62, 0.64, 0.66)
BLACK = (0.08, 0.08, 0.09)
RED = (0.80, 0.14, 0.10)
TAPE = (0.78, 0.60, 0.33)  # tan packing tape
CORE = (0.55, 0.42, 0.28)
LEAD = (0.86, 0.70, 0.42)
TABC = (0.20, 0.45, 0.85)

# -- geometry (world, the gun at its spawn pose; the gun frame is world shifted to GUN_O) ---------------------------
GUN_O = (0.280, 0.020, TOP)
HUB = (0.280, 0.022)
HUB_R, HUB_H = 0.030, 0.046  # hub body 60 mm across, 46 mm tall
COL_R0, COL_R1 = 0.033, 0.045  # detent collar tabs (the 74 mm sprung leaves)
COL_TOP0, COL_TRAVEL = 0.024, 0.020  # collar tab top above the plate at rest, and its travel down to the seat
DETENT_N = 12.0
COL_K = 600.0  # N/m; preloaded so the core has to push 12 N to start it
DETENT_AT = 0.003  # collar pushed this far = past the ball
R_I, R_O, R_H = 0.038, 0.060, 0.048  # tape roll: 76 mm core, 120 mm across, 48 mm wide
SEAT_Z = ZP + 0.004 + R_H / 2  # roll centre height when seated on the dropped collar
ROLL0 = (0.200, -0.240, TOP + R_H / 2)
ROLLER = (0.235, -0.050)  # tension roller axis
ROLLER_PIVOT = (0.247, -0.075)
ROLLER_H = 0.034
NOSE_X = 0.225  # the nose's outer face, where the leader lies
BLADE_Y = -0.118  # the blade's cutting edge (gun front)
GUARD_HINGE = (0.2185, -0.086)
GUARD_LEN = 0.044
LINK_L, LINK_T, LINK_H = 0.020, 0.002, 0.044
N_LINKS = 8
RV = 0.065  # chain vertex radius on the roll
THETA = 2 * math.asin(LINK_L / 2 / RV)
A0 = math.radians(88.0)  # the leader's root on the roll; it runs clockwise to its end


def W(x, y, z):
    return (x - GUN_O[0], y - GUN_O[1], z - GUN_O[2])


def _ring(ri, ro, h, z0, color, n=12, cx=0.0, cy=0.0):
    t = ro - ri
    w = 2 * ro * math.tan(math.pi / n)
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = (ro + ri) / 2
        out.append(mk.box((t, w, h), at=(cx + rc * math.cos(a), cy + rc * math.sin(a), z0 + h / 2), yaw=a,
                          color=color))
    return out


kit = mk.Kit()

# gun body: side plate, hub, nose with the blade, handle
_gun = [
    mk.box((0.160, 0.250, 0.006), at=W(0.280, 0.005, TOP + 0.003)),
    *_ring(0.020, HUB_R, HUB_H, ZP - TOP, STEEL, cx=HUB[0] - GUN_O[0], cy=HUB[1] - GUN_O[1]),
    mk.box((0.025, 0.027, 0.034), at=W(0.2375, -0.1015, ZP + 0.017)),
    mk.box((0.025, 0.003, 0.034), at=W(0.2375, -0.1165, ZP + 0.017), color=STEEL),
    mk.box((0.100, 0.030, 0.035), at=W(0.290, 0.160, ZP + 0.0175), color=BLACK),
    mk.box((0.008, 0.008, 0.006), at=W(*ROLLER_PIVOT, ZP + 0.003), color=STEEL),
]
kit.part("gun", GUN_O, _gun, mass=0.32, color=GUN_C)  # 0.40 kg with collar, roller arm and drum, guard

# detent collar: three tabs round the hub on a slider; preloaded up against its stop at 12 N
kit.part("collar", (*HUB, ZP + COL_TOP0 - 0.002),
         [mk.box((COL_R1 - COL_R0, 0.008, 0.004), at=((COL_R0 + COL_R1) / 2 * math.cos(a), (COL_R0 + COL_R1) / 2
                                                         * math.sin(a), 0.0), yaw=a, color=STEEL)
          for a in (math.radians(90), math.radians(210), math.radians(330))], mass=0.02, color=STEEL)
kit.slider("collar_slide", "gun", "collar", axis=(0, 0, -1), lower=0.0, upper=COL_TRAVEL, stiffness=COL_K,
           rest=-DETENT_N / COL_K, damping=2.0)

# tension roller on a sprung arm (6 N at the roller, toward the leader path); the roller turns freely on its axle
_ax, _ay = ROLLER[0] - ROLLER_PIVOT[0], ROLLER[1] - ROLLER_PIVOT[1]
_arm_len, _arm_yaw = math.hypot(_ax, _ay), math.atan2(_ay, _ax)
kit.part("roller", (*ROLLER_PIVOT, ZP),
         [mk.box((_arm_len + 0.008, 0.008, 0.004), at=(_ax / 2, _ay / 2, 0.003), yaw=_arm_yaw)], mass=0.02,
         color=STEEL)
ROLLER_K = 6.0 * _arm_len / math.radians(15.0)
kit.hinge("roller_arm", "gun", "roller", anchor=(*ROLLER_PIVOT, ZP), axis=(0, 0, -1), lower_deg=0.0, upper_deg=25.0,
          stiffness=ROLLER_K, rest_deg=-15.0, damping=0.01)
kit.part("drum", (*ROLLER, ZP),
         [mk.box((0.018, 0.018, ROLLER_H - 0.005), at=(0, 0, 0.005 + (ROLLER_H - 0.005) / 2)),
          mk.box((0.018, 0.018, ROLLER_H - 0.005), at=(0, 0, 0.005 + (ROLLER_H - 0.005) / 2), yaw=math.pi / 4)],
         mass=0.02, color=BLACK)
kit.hinge("drum_spin", "roller", "drum", anchor=(*ROLLER, ZP), axis=(0, 0, 1), lower_deg=-math.inf,
          upper_deg=math.inf, damping=0.0002)

# blade guard: a red plate over the nose face, hinged at its root, sprung shut, detent at open (90 deg)
kit.part("guard", (*GUARD_HINGE, ZP),
         [mk.box((0.002, GUARD_LEN, 0.030), at=(0.0, -GUARD_LEN / 2 - 0.002, 0.019))], mass=0.02, color=RED)
kit.hinge("guard_turn", "gun", "guard", anchor=(*GUARD_HINGE, ZP), axis=(0, 0, -1), lower_deg=-9.0,
          upper_deg=92.0, stiffness=0.05, rest_deg=-20.0, damping=0.005)

# the roll, frame at its centre
kit.part("roll", ROLL0, [*_ring(0.0365, 0.0395, R_H, -R_H / 2, CORE), *_ring(0.0395, R_O, R_H - 0.001,
                                                                             -R_H / 2 + 0.0005, TAPE)],
         mass=0.30, color=TAPE)


def _vertex(k):
    a = A0 - k * THETA
    return np.array([ROLL0[0] + RV * math.cos(a), ROLL0[1] + RV * math.sin(a)])


TAB_LOCAL = (0.012, 0.004, LINK_H / 2 + 0.012)  # the folded tab's centre on link 8 (pads pinch it)
TAB_YAW = math.radians(-60.0)  # tab faces: pinch normal at +30 deg from the link's direction
for k in range(1, N_LINKS + 1):
    b, e = _vertex(k - 1), _vertex(k)
    yaw = math.atan2(e[1] - b[1], e[0] - b[0])
    boxes = [mk.box((LINK_L - 0.001, LINK_T, LINK_H), at=(LINK_L / 2, 0.0, 0.0))]
    if k == N_LINKS:
        boxes.append(mk.box((0.014, 0.006, 0.032), at=TAB_LOCAL, yaw=TAB_YAW, color=TABC))
    kit.part(f"lk{k}", (b[0], b[1], ROLL0[2]), boxes, mass=0.02, color=LEAD, yaw=yaw)
    kit.hinge(f"lk{k}_hinge", "roll" if k == 1 else f"lk{k - 1}", f"lk{k}", anchor=(b[0], b[1], ROLL0[2]),
              axis=(0, 0, 1), lower_deg=-170.0, upper_deg=170.0, damping=0.0005)

# the folded tab end sticks to the roll until a hand takes the tab
kit.catch("tab_stuck", "lk8", kit.parts["lk8"].ref, parent="roll", parent_T=kit.parts["roll"].ref)
# past the detent the roll turns freely on its hub (two ways round: tape face out or face in)
_seat = mk._T(None, (*HUB, SEAT_Z))
_flip = mk._T(mk.axis_angle((1, 0, 0), math.pi), (*HUB, SEAT_Z))
kit.catch("seat_up", "roll", _seat, parent="gun", parent_T=kit.parts["gun"].ref, pivot=(0, 0, 1), pivot_damping=0.02)
kit.catch("seat_dn", "roll", _flip, parent="gun", parent_T=kit.parts["gun"].ref, pivot=(0, 0, 1),
          pivot_damping=0.02)

OBJECTS = [*kit.objects()]
LINKS = [f"lk{k}" for k in range(1, N_LINKS + 1)]

# -- gun-frame readouts ---------------------------------------------------------------------------------------------
_GUN_REF_INV = mk._inv(kit.parts["gun"].ref)


def _gun_T(objects):
    """World <- reference: maps a point given in spawn-pose world coordinates to where it is now."""
    return kit._pose_T(objects, "gun") @ _GUN_REF_INV


def to_gun(objects, p):
    """A world point in spawn-pose world coordinates of the gun (so it can be compared with the constants)."""
    t = mk._inv(_gun_T(objects))
    return t[:3, :3] @ np.asarray(p, float) + t[:3, 3]


def _dir_gun(objects, v):
    return mk._inv(_gun_T(objects))[:3, :3] @ np.asarray(v, float)


def gun_tilt(objects) -> float:
    _, q = kit.pose(objects, "gun")
    z = mk.quat_to_mat(q)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def roll_face(objects) -> float:
    """Cosine between the roll's axis and the gun's: +1 tape face out (as packed), -1 turned over."""
    _, q = kit.pose(objects, "roll")
    return float(_dir_gun(objects, mk.quat_to_mat(q)[:, 2])[2])


def seated_geom(objects) -> bool:
    c = to_gun(objects, kit.point(objects, "roll", (0, 0, 0)))
    return (math.hypot(c[0] - HUB[0], c[1] - HUB[1]) <= 0.008 and abs(c[2] - SEAT_Z) <= 0.004
            and abs(roll_face(objects)) >= math.cos(math.radians(5.0)))


def chain_points(objects):
    """The leader's nine joint points (root .. end), in gun coordinates."""
    pts = [to_gun(objects, kit.point(objects, n, (0, 0, 0))) for n in LINKS]
    pts.append(to_gun(objects, kit.point(objects, "lk8", (LINK_L, 0, 0))))
    return pts


def _seg_closest(a, b, c):
    ab = b[:2] - a[:2]
    t = float(np.clip(np.dot(c - a[:2], ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return a[:2] + t * ab


ROLLER_TOUCH = 0.016  # roller 12.7 mm to its corners + half the tape + 2.3 mm
UNDER = np.array([-1.0, 1.0]) / math.sqrt(2.0)  # the roller side facing away from the roll and the nose
OFF_ROLL = R_O + 0.010  # a leader point 70 mm or more from the roll's axis is off the roll (wound links lie at 64-65)
WRAP_DEG = 120.0  # how far the leader must wind round the roller's axis, root to end (a wound leader: under 100)
PAST_ROLLER = 0.016  # the end must be this far past the roller's axis toward the nose (gun -y)


def roller_axis(objects):
    return to_gun(objects, kit.point(objects, "drum", (0, 0, 0)))[:2]


def roller_touch(objects) -> tuple[bool, bool]:
    """(touching, touching off the roll): a segment of links 2-6 has its centreline within 16 mm of the roller's
    axis on the roller's outer side; off the roll if that touching point is also 70 mm or more from the roll's
    axis (a leader still wound on the roll lies 64-65 mm from it)."""
    c = roller_axis(objects)
    rc = to_gun(objects, kit.point(objects, "roll", (0, 0, 0)))[:2]
    pts = chain_points(objects)
    touch = off = False
    for k in range(1, 6):  # link k+1 runs from pts[k] to pts[k+1]
        q = _seg_closest(pts[k], pts[k + 1], c)
        if float(np.linalg.norm(q - c)) <= ROLLER_TOUCH and float(np.dot(q - c, UNDER)) > 0.002:
            touch = True
            off = off or float(np.linalg.norm(q - rc)) >= OFF_ROLL
    return touch, off


def roller_wrap(objects) -> float:
    """Degrees the leader winds round the roller's axis from its root on the roll to its end, counter-clockwise
    (the way round the roller's outside, from the roll toward the nose) positive. Each link subtends less than 180
    deg at the axis (the roller body keeps the leader 9 mm or more from it), so summing per link is exact."""
    c = roller_axis(objects)
    pts = [p[:2] - c for p in chain_points(objects)]
    return math.degrees(sum(math.atan2(a[0] * b[1] - a[1] * b[0], float(np.dot(a, b))) for a, b in zip(pts, pts[1:])))


def under_roller(objects) -> bool:
    """The leader is threaded round the roller: paid off the roll and touching the roller's outer side there,
    wound 120 deg or more round the roller's axis the right way, its end past the roller toward the nose."""
    _, off = roller_touch(objects)
    tip = chain_points(objects)[-1]
    return off and roller_wrap(objects) >= WRAP_DEG and float(tip[1]) <= float(roller_axis(objects)[1]) - PAST_ROLLER


def adhesive_out(objects) -> bool:
    """Links 7 and 8 have their adhesive face (local -y) facing away from the nose (gun -x) within 37 deg."""
    for n in ("lk7", "lk8"):
        _, q = kit.pose(objects, n)
        v = _dir_gun(objects, mk.quat_to_mat(q) @ np.array([0.0, -1.0, 0.0]))
        if float(v[0]) > -0.8:
            return False
    return True


def end_past_blade(objects) -> float:
    """How far (m) the leader's end is past the blade edge, lying along the nose face; -1 if it is not there."""
    tip = chain_points(objects)[-1]
    mid = to_gun(objects, kit.point(objects, "lk8", (LINK_L / 2, 0, 0)))
    if not (NOSE_X - 0.0045 <= mid[0] <= NOSE_X + 0.0035 and NOSE_X - 0.0045 <= tip[0] <= NOSE_X + 0.0035):
        return -1.0
    return float(BLADE_Y - tip[1])


def guard_closed(objects) -> bool:
    return kit.q_deg(objects, "guard_turn") <= 6.0


# -- rules ---------------------------------------------------------------------------------------------------------


def _disengage(name):
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


def _drive_once(key, joint, target):
    if mk.MEMORY.get(("drv", key)) != target:
        mk.MEMORY[("drv", key)] = target
        kit.drive(joint, target)


def _rules(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    if m["n"] == 1:
        kit.engage("tab_stuck")
    # detent: once the collar has been pushed 3 mm against its 12 N preload, the ball is past and it drops
    if not m.get("detent") and kit.q(objects, "collar_slide") >= DETENT_AT:
        m["detent"] = True
    _drive_once("collar", "collar_slide", COL_TRAVEL + 0.010 if m.get("detent") else -DETENT_N / COL_K)
    # the roll clicks onto the hub (turns freely on it) once it is down on the dropped collar
    if m.get("detent") and not m.get("seat") and seated_geom(objects):
        m["seat"] = "up" if roll_face(objects) > 0 else "dn"
        kit.engage("seat_up" if m["seat"] == "up" else "seat_dn")
    # the tab is peeled when a hand takes it
    if not m.get("peeled"):
        tab = kit.point(objects, "lk8", TAB_LOCAL)
        if any(float(np.linalg.norm(np.asarray(t) - tab)) < 0.015 for t in mk.tcps()):
            m["near_tab"] = m.get("near_tab", 0) + 1
            if m["near_tab"] >= 5:
                m["peeled"] = True
                _disengage("tab_stuck")
    # guard detents: past 45 deg it stays open; below, the spring shuts it
    q = kit.q_deg(objects, "guard_turn")
    _drive_once("guard", "guard_turn", 100.0 if q >= 45.0 else -20.0)
    # off the bench: the gun, the roll or the leader end dropped 5 cm below the bench top (out of reach, on the floor)
    for n in ("gun", "roll", "lk8"):
        if float(kit.point(objects, n, (0, 0, 0))[2]) < TOP - 0.05:
            m["floor"] = True
    if os.environ.get("TAPE_DEBUG") and m["n"] % 20 == 0:
        f = lambda v: "(" + ",".join(f"{float(c):.4f}" for c in v) + ")"
        pts = chain_points(objects)
        _, rq = kit.pose(objects, "roll")
        ryaw = math.degrees(math.atan2(mk.quat_to_mat(rq)[1, 0], mk.quat_to_mat(rq)[0, 0]))
        print(f"[tape] n={m['n']} roll={f(to_gun(objects, kit.point(objects, 'roll', (0, 0, 0))))} yaw={ryaw:.1f} "
              f"collar={kit.q(objects, 'collar_slide'):.4f} guard={q:.1f} roller={kit.q_deg(objects, 'roller_arm'):.1f} "
              f"tab={f(kit.point(objects, 'lk8', TAB_LOCAL))} tip={f(pts[-1])} gun={f(kit.point(objects, 'gun', (0, 0, 0)))} "
              f"tilt={gun_tilt(objects):.1f} gyaw={math.degrees(math.atan2(_gun_T(objects)[1, 0], _gun_T(objects)[0, 0])):.2f} "
              f"touch={roller_touch(objects)} wrap={roller_wrap(objects):.1f} spin={kit.q_deg(objects, 'drum_spin'):.1f} "
              f"under={under_roller(objects)} adh={adhesive_out(objects)} "
              f"past={end_past_blade(objects):.4f} mem={ {k: v for k, v in m.items() if isinstance(k, str) and k != 'n'} } "
              f"score={score(objects)}", flush=True)


kit.on_step(_rules)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring -------------------------------------------------------------------------------------------------------
_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def settled(objects) -> bool:
    return (all(_released(objects[n]) for n in ("roll", "lk8", "guard", "gun"))
            and all(objects[n].resting(max_speed=0.01, max_spin=0.25) for n in ("roll", "lk6", "lk8", "gun")))


def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("floor"))


def seated(objects) -> bool:
    return bool(mk.MEMORY.get("detent")) and mk.MEMORY.get("seat") in ("up", "dn") and seated_geom(objects)


def score(objects) -> float:
    """0 if failed() or the roll is not seated face out past the detent. Seated face out 0.3; leader round the
    outside of the roller 0.3; with that, adhesive out on the last two links 0.2; with that, end 5-25 mm past the
    blade on the nose face, guard shut on it, hands off and still 0.2. Only failed() latches; the rest is read off
    the state at the last step, so a misfed or short leader can still be put right."""
    if failed(objects) or not seated(objects) or roll_face(objects) <= 0:
        return 0.0
    s = 0.30
    if under_roller(objects):
        s += 0.30
        if adhesive_out(objects):
            s += 0.20
            past = end_past_blade(objects)
            if 0.005 <= past <= 0.025 and guard_closed(objects) and settled(objects):
                s += 0.20
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    c = kit.point(objects, "roll", (0, 0, 0))
    return float(math.dist([float(v) for v in c], [HUB[0], HUB[1], SEAT_Z]))


# -- the scripted demo ---------------------------------------------------------------------------------------------
TIP = 0.013
TRAVEL = ZP + 0.150
F_IN, F_GRIP = -0.020, -0.040
F_PRESS = -0.044  # pads 44-50 mm out from the axis: on the tape face, clear of the core and hub
CORE_DEPTH = 0.025  # pad tips this far down inside the core (outside the 60 mm hub body)
F_TAB_OPEN, F_TAB = -0.022, 0.0
F_HANDLE_OPEN, F_HANDLE = -0.026, -0.006
HANDLE = (0.290, 0.160)
CORE_YAW = -1.0  # fingers spread mostly along x inside the core, clear of the other hand
TAB_TCP_Z = ZP + 0.056 + TIP  # pad tips 4 mm over the seated roll's top


class _Seq:
    def __init__(self):
        self.w, self.t = [], 0.0
        self.last = None

    def go(self, d, pos, f, label, yaw=0.0):
        self.w.append(Waypoint(d, tuple(float(v) for v in pos), f, label, float(yaw)))
        self.t += d
        self.last = (tuple(pos), f, yaw)
        return self

    def until(self, t, label="wait"):
        pos, f, yaw = self.last
        return self.go(max(0.1, t - self.t), pos, f, label, yaw)


def _tab_seated(yaw_deg=0.0):
    """Where the tab centre is once the roll sits on the hub turned yaw_deg from how it lay on the bench."""
    st = kit.fake_states()
    t = kit.point(st, "lk8", TAB_LOCAL) - np.asarray(ROLL0)
    r = mk.axis_angle((0, 0, 1), math.radians(yaw_deg))
    return np.array([HUB[0], HUB[1], SEAT_Z]) + r @ t


def _tab_for(tip_xy, link_dir_deg):
    """Tab centre for link 8 lying with its end at tip_xy, pointing link_dir_deg."""
    a = math.radians(link_dir_deg)
    d, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    base = np.asarray(tip_xy) - LINK_L * d
    return base + TAB_LOCAL[0] * d + TAB_LOCAL[1] * n


def _yaw_for(link_dir_deg):
    """Gripper yaw that pinches the tab: fingers close along the tab normal (link direction + 30 deg)."""
    y = (link_dir_deg + 30.0 - 90.0 + 90.0) % 180.0 - 90.0
    return math.radians(y)


AIM = (0.0, 0.0)
SEAT_YAW = float(os.environ.get("TAPE_SEAT_YAW", "0"))
TIP_END = (NOSE_X - 0.0015, BLADE_Y - 0.015)


def _plans(variant=""):
    L, R = _Seq(), _Seq()
    hx, hy = HANDLE
    hz = ZP + 0.035 - 0.020 + TIP
    # L: hold the gun by its handle
    L.go(2.0, (hx, hy, hz + 0.06), F_HANDLE_OPEN, "over the handle")
    L.go(1.5, (hx, hy, hz), F_HANDLE_OPEN, "down round the handle")
    L.go(1.0, (hx, hy, hz), F_HANDLE, "hold the gun")
    if variant == "tipped":
        L.go(2.0, (hx, hy - 0.05, hz + 0.16), F_HANDLE, "lift the handle end")
        L.go(0.8, (hx, hy - 0.05, hz + 0.16), F_HANDLE_OPEN, "let go")
        L.go(2.0, (0.20, 0.30, TRAVEL), F_HANDLE_OPEN, "away")
        L.go(3.0, (0.20, 0.30, TRAVEL), F_HANDLE_OPEN, "wait")
        R.go(1.0, (0.20, -0.30, TRAVEL), 0.0, "park")
        R.until(L.t)
        return {"left": L.w, "right": R.w}
    # R: pick the roll by its core
    x0, y0, z0 = ROLL0
    zc = z0 + R_H / 2 - CORE_DEPTH + TIP
    R.go(2.0, (x0, y0, zc + 0.08), F_IN, "over the roll", CORE_YAW)
    R.go(1.5, (x0, y0, zc), F_IN, "fingers into the core", CORE_YAW)
    R.go(1.0, (x0, y0, zc), F_GRIP, "open against the core", CORE_YAW)
    R.go(0.5, (x0, y0, zc), F_GRIP, "settle", CORE_YAW)
    carry = ZP + HUB_H + 0.015 + R_H - CORE_DEPTH + TIP  # roll bottom 15 mm over the hub top
    R.go(1.5, (x0, y0, carry), F_GRIP, "lift", CORE_YAW)
    if variant == "beside":
        bx, by = 0.135, -0.110
        R.go(2.5, (bx, by, carry), F_GRIP, "over the bench beside the gun", CORE_YAW)
        R.go(1.5, (bx, by, zc + 0.001), F_GRIP, "down on the bench", CORE_YAW)
        R.go(0.8, (bx, by, zc + 0.001), F_IN, "let go", CORE_YAW)
        R.go(1.0, (bx, by, TRAVEL), F_IN, "up", CORE_YAW)
        R.go(2.0, (0.20, -0.30, TRAVEL), F_IN, "park", CORE_YAW)
        L.until(R.t)
        L.go(0.8, (hx, hy, hz), F_HANDLE_OPEN, "let go of the gun")
        L.go(1.5, (0.20, 0.30, TRAVEL), F_HANDLE_OPEN, "park")
        R.until(L.t + 2.0)
        return {"left": L.w, "right": R.w}
    ax, ay = HUB[0] + AIM[0], HUB[1] + AIM[1]  # the roll hangs a little behind the hand
    R.go(2.5, (ax, ay, carry), F_GRIP, "over the hub", CORE_YAW)
    R.go(1.0, (ax, ay, carry), F_GRIP, "settle over it", CORE_YAW)
    z_rest = ZP + COL_TOP0 + R_H - CORE_DEPTH + TIP  # roll bottom on the collar
    R.go(1.5, (ax, ay, z_rest + 0.002), F_GRIP, "onto the hub, down to the detent", CORE_YAW)
    R.go(0.8, (ax, ay, z_rest + 0.002), F_IN, "let go, the roll resting on the detent", CORE_YAW)
    R.go(1.0, (ax, ay, z_rest + 0.040), F_IN, "up out of the core", CORE_YAW)
    top = ZP + COL_TOP0 + R_H  # the roll's top face while it rests on the detent
    R.go(1.0, (HUB[0], HUB[1], top + 0.010 + TIP), F_PRESS, "pads over the tape face, either side", CORE_YAW)
    R.go(1.0, (HUB[0], HUB[1], top + 0.001 + TIP), F_PRESS, "on the roll's face", CORE_YAW)
    R.go(1.5, (HUB[0], HUB[1], top - 0.018 + TIP), F_PRESS, "push it past the detent ball", CORE_YAW)
    R.go(1.0, (HUB[0], HUB[1], top - 0.018 + TIP), F_PRESS, "hold it down", CORE_YAW)
    R.go(1.0, (ax, ay, TRAVEL), F_IN, "up", CORE_YAW)
    if variant == "wound_pushed":
        # leader left wound on the roll: turn the seated roll (two strokes of the pads on its face) until the wound
        # leader faces the roller, then push the roller's arm in onto it and hold it there to the end
        face = SEAT_Z + R_H / 2 - 0.002 + TIP
        for _ in range(2):
            R.go(1.5, (HUB[0], HUB[1], face + 0.020), F_PRESS, "over the roll's face", 0.6)
            R.go(1.0, (HUB[0], HUB[1], face), F_PRESS, "pads on the face", 0.6)
            for k in range(1, 8):
                R.go(0.4, (HUB[0], HUB[1], face), F_PRESS, "turn the roll on its hub", 0.6 - 1.2 * k / 7)
            R.go(1.0, (HUB[0], HUB[1], face + 0.020), F_PRESS, "up off it", -0.6)
        px, py, pz = ROLLER[0] - 0.030, ROLLER[1] + 0.002, ZP + 0.016 + TIP
        R.go(2.0, (px, py, TRAVEL), 0.0, "over the roller's outside")
        R.go(1.5, (px, py, pz), 0.0, "down beside it")
        R.go(2.0, (ROLLER[0] - 0.009, py, pz), 0.0, "push the roller arm in onto the wound leader")
        R.until(79.0, "hold it there")
        L.until(79.0, "hold the gun")
        return {"left": L.w, "right": R.w}
    # R: swing the blade guard open (pads against its inside face beyond the blade, swept round its hinge)
    gx, gy = GUARD_HINGE
    gz = ZP + 0.010 + TIP

    def gat(q, r=0.034, off=0.0095):
        a = math.radians(270.0 - q)
        d, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        return np.array([gx, gy]) + r * d + off * n

    p = gat(-4)
    R.go(2.0, (p[0] + 0.016, p[1] - 0.002, TRAVEL), 0.0, "over the nose end, beyond the blade")
    R.go(1.5, (p[0] + 0.016, p[1] - 0.002, gz), 0.0, "down beside the guard's tip")
    R.go(1.0, (p[0] + 0.002, p[1], gz), 0.0, "against its inside face")
    for q in range(5, 101, 12):
        p = gat(q)
        R.go(0.8, (p[0], p[1], gz), 0.0, f"swing the guard to {q} deg")
    p = gat(85, off=0.020)
    R.go(1.0, (p[0], p[1], TRAVEL), 0.0, "up off the guard")
    # R: take the tab, peel the leader and lead it round the roller and along the nose
    tab = _tab_seated(SEAT_YAW)
    d8 = 225.3 + SEAT_YAW
    R.go(2.0, (tab[0], tab[1], TAB_TCP_Z + 0.04), F_TAB_OPEN, "over the tab", _yaw_for(d8))
    R.go(1.2, (tab[0], tab[1], TAB_TCP_Z), F_TAB_OPEN, "down round the tab", _yaw_for(d8))
    R.go(0.8, (tab[0], tab[1], TAB_TCP_Z), F_TAB, "pinch the tab", _yaw_for(d8))
    R.go(0.5, (tab[0], tab[1], TAB_TCP_Z), F_TAB, "settle", _yaw_for(d8))
    c = np.array(HUB)
    a_t = math.atan2(tab[1] - c[1], tab[0] - c[0])
    for i, frac in enumerate((0.25, 0.5, 0.75, 1.0)):
        a = a_t + (math.radians(-95.0) - a_t) * frac
        r = 0.075 + 0.012 * min(1.0, 2 * frac) - 0.010 * max(0.0, frac - 0.5) * 2
        dd = math.degrees(a) - 90.0
        p = c + r * np.array([math.cos(a), math.sin(a)])
        R.go(1.0, (p[0], p[1], TAB_TCP_Z), F_TAB, "peel the leader back round the roll", _yaw_for(dd))
    y_line = ROLLER[1] + 0.020  # drawn off the roll clear of the roller's top side
    t1 = _tab_for((0.255, y_line), 180.0)
    R.go(1.5, (t1[0], t1[1], TAB_TCP_Z), F_TAB, "draw it off the roll toward the roller", _yaw_for(180.0))
    if variant == "over":
        t2 = _tab_for((NOSE_X - 0.0015, -0.080), 250.0)
        R.go(2.0, (t2[0], t2[1], TAB_TCP_Z), F_TAB, "straight across to the nose", _yaw_for(250.0))
    else:
        t2 = _tab_for((ROLLER[0] - 0.021, y_line), 180.0)
        R.go(1.5, (t2[0], t2[1], TAB_TCP_Z), F_TAB, "past the roller", _yaw_for(180.0))
        t3 = _tab_for((ROLLER[0] - 0.018, ROLLER[1] - 0.022), 270.0)
        R.go(2.0, (t3[0], t3[1], TAB_TCP_Z), F_TAB, "turn it down round the roller's outside", _yaw_for(270.0))
        t3 = _tab_for((NOSE_X - 0.0045, ROLLER[1] - 0.028), 270.0)
        R.go(1.0, (t3[0], t3[1], TAB_TCP_Z), F_TAB, "between the open guard's root and the nose", _yaw_for(270.0))
        t3 = _tab_for((NOSE_X - 0.0045, BLADE_Y + 0.010), 270.0)
        R.go(1.5, (t3[0], t3[1], TAB_TCP_Z), F_TAB, "down past the guard's root", _yaw_for(270.0))
    t4 = _tab_for(TIP_END, 270.0)
    R.go(1.5, (t4[0], t4[1], TAB_TCP_Z), F_TAB, "along the nose face past the blade", _yaw_for(270.0))
    R.go(0.8, (t4[0], t4[1], TAB_TCP_Z), F_TAB, "hold it there", _yaw_for(270.0))
    t_laid = R.t
    # R: let go of the tab and swing the guard shut onto the leader; then the left hand lets go of the gun
    R.go(0.8, (t4[0], t4[1], TAB_TCP_Z), F_TAB_OPEN, "let go of the tab", _yaw_for(270.0))
    R.go(1.0, (t4[0], t4[1], TRAVEL), F_TAB_OPEN, "up", _yaw_for(270.0))

    def gclose(q, r=0.034, off=0.0105):
        a = math.radians(270.0 - q)
        d, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        return np.array([gx, gy]) + r * d - off * n

    p = gclose(98)
    R.go(2.0, (p[0], p[1], TRAVEL), 0.0, "over the open guard")
    R.go(1.5, (p[0], p[1], gz), 0.0, "down beside it")
    # over: the leader lies across the guard's path, so push it right down onto the nose
    for q in range(86, (-1 if variant == "over" else 29), -12):
        p = gclose(q)
        R.go(0.8, (p[0], p[1], gz), 0.0, f"swing the guard back to {q} deg")
    R.go(1.0, (p[0] - 0.015, p[1] + 0.010, gz + 0.04), 0.0, "off it; the spring shuts it on the leader")
    R.go(2.0, (0.20, -0.30, TRAVEL), 0.0, "park")
    L.until(R.t)
    L.go(0.8, (hx, hy, hz), F_HANDLE_OPEN, "let go of the handle")
    L.go(1.0, (hx, hy, TRAVEL), F_HANDLE_OPEN, "up")
    L.go(1.5, (0.20, 0.30, TRAVEL), F_HANDLE_OPEN, "park")
    L.go(3.0, (0.20, 0.30, TRAVEL), F_HANDLE_OPEN, "wait")
    R.until(L.t)
    return {"left": L.w, "right": R.w}


def demo():
    # DEMO_VARIANT: beside | over | tipped | wound_pushed (degenerate checks: 0, 0.3 seated only, 0, 0.3 seated only)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ---------------------------------------------------------------------------------------------


def _chain_moved(verts, z=SEAT_Z, flip=False):
    out = {}
    for k in range(N_LINKS):
        b, e = np.asarray(verts[k]), np.asarray(verts[k + 1])
        yaw = math.atan2(e[1] - b[1], e[0] - b[0])
        r = mk.axis_angle((0, 0, 1), yaw)
        if flip:
            r = r @ mk.axis_angle((1, 0, 0), math.pi)
        out[f"lk{k + 1}"] = (np.array([b[0], b[1], z]), mk.mat_to_quat(r))
    return out


def _route(kind="good", past=0.015):
    """Nine leader points for a routed leader: from the roll tangent, round the roller's outside, down the nose."""
    y_line = ROLLER[1] + 0.011
    x_line = NOSE_X - 0.001
    tip = (x_line, BLADE_Y - past)
    if kind == "good":
        corner = (x_line, y_line)
        path = [(0.290, y_line), (0.270, y_line), (0.250, y_line), (0.230, y_line), corner]
        rest = np.linspace(corner[1], tip[1], 5)[1:]
        pts = path + [(x_line, y) for y in rest]
    elif kind == "past":  # drawn straight off the roll past the roller's outside, not turned down round it
        a, b = np.array([0.300, y_line]), np.array([0.300 - 8 * LINK_L, y_line])
        pts = [tuple(a + (b - a) * t) for t in np.linspace(0, 1, 9)]
    else:  # draped straight across, roller on the inside
        a, b = np.array([0.300, y_line]), np.array(tip)
        pts = [tuple(a + (b - a) * t) for t in np.linspace(0, 1, 9)]
    pts = [np.array(p) for p in pts]
    # respace to 20 mm links along the polyline, ending at the tip
    seg = [pts[-1]]
    i = len(pts) - 1
    cur = pts[-1]
    while len(seg) < 9 and i > 0:
        nxt = pts[i - 1]
        d = np.linalg.norm(nxt - cur)
        need = LINK_L
        if d >= need:
            cur = cur + (nxt - cur) / d * need
            seg.append(cur.copy())
        else:
            # walk on to the next segment
            rem = need - d
            cur = nxt
            i -= 1
            while i > 0:
                n2 = pts[i - 1]
                d2 = np.linalg.norm(n2 - cur)
                if d2 >= rem:
                    cur = cur + (n2 - cur) / d2 * rem
                    break
                rem -= d2
                cur = n2
                i -= 1
            seg.append(cur.copy())
            continue
        pts[i] = cur
    while len(seg) < 9:
        seg.append(seg[-1] + np.array([0.0, 0.0]))
    return seg[::-1]


def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])
    seat = (np.array([*HUB, SEAT_Z]), I)
    flip = (np.array([*HUB, SEAT_Z]), mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi)))

    def st(roll=seat, chain=None, guard=0.0, flipped=False, arm=0.0):
        moved = {"roll": roll}
        if chain is not None:
            moved.update(_chain_moved(chain, flip=flipped))
        return kit.fake_states(q={"guard_turn": math.radians(guard), "collar_slide": COL_TRAVEL,
                                  "roller_arm": math.radians(arm)}, moved=moved)

    def turned(yaw_deg):  # the seated roll turned on its hub, leader still wound on it
        return (np.array([*HUB, SEAT_Z]), mk.mat_to_quat(mk.axis_angle((0, 0, 1), math.radians(yaw_deg))))

    S = {"detent": True, "seat": "up"}
    good = _route("good")
    cases = {
        "start": (kit.fake_states(), [], {}, 0.0),
        "roll on the hub, detent not passed": (st(), [], {"seat": "up"}, 0.0),
        "seated, leader still wound": (st(), [], S, 0.3),
        "seated, leader draped over the roller": (st(chain=_route("over")), [], S, 0.3),
        "seated, leader wound, turned to face the roller, arm pushed 15 deg": (st(roll=turned(220), arm=15), [], S, 0.3),
        "seated, leader wound, turned to face the roller, arm pushed 25 deg": (st(roll=turned(200), arm=25), [], S, 0.3),
        "leader drawn past the roller's outside, not turned round it": (st(chain=_route("past")), [], S, 0.3),
        "routed, guard open": (st(chain=good, guard=90.0), [], S, 0.8),
        "routed, guard shut, hand on the tab": (st(chain=good), [kit.point(st(chain=good), "lk8", TAB_LOCAL)], S, 0.8),
        "everything done": (st(chain=good), [], S, 1.0),
        "end 3 mm past the blade": (st(chain=_route("good", 0.003)), [], S, 0.8),
        "end 30 mm past the blade": (st(chain=_route("good", 0.030)), [], S, 0.8),
        "turned-over roll": (st(roll=flip), [], {"detent": True, "seat": "dn"}, 0.0),
        "turned-over roll, routed": (st(roll=flip, chain=good), [], {"detent": True, "seat": "dn"}, 0.0),
        "roll beside the hub": (st(roll=(np.array([0.16, -0.08, TOP + R_H / 2]), I)), [], {"detent": True}, 0.0),
        "everything done, something fell off the bench earlier": (st(chain=good), [], {**S, "floor": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        assert under_roller(st(chain=good)) and not under_roller(st(chain=_route("over")))
        for yaw in range(0, 360, 10):  # a wound leader never counts, at any roll angle or arm position
            for arm in (0.0, 15.0, 25.0):
                assert not under_roller(st(roll=turned(yaw), arm=arm)), (yaw, arm)
        assert roller_touch(st(roll=turned(220), arm=15))[0]  # (it touches the roller's outside, but on the roll)
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
    for v in ("", "beside", "over", "tipped", "wound_pushed"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
