"""On the station table, in front of the left arm, stands a steel clevis joint: a yoke with two horizontal cheeks
on a stand, a yellow linkage rod end between them, and a vertical 18 mm square clevis pin through all three. The
pin's drilled end stands up above the top cheek; its cross-hole (13 mm wide, 12 mm tall) runs along x, toward and
away from the robot. A 10 mm steel cotter pin lies on a white holder 10 cm nearer the robot: a flat eye loop, then
two legs side by side, one short (22 mm past the shank) and one long (40 mm, the extended-prong type). The job is
to secure the clevis pin with the cotter. Scored from the cotter's and its legs' poses.

Each leg is a rigid bar on a hinge at its root (mechkit.py, in this folder), the axis vertical, with Coulomb
friction (0.01 N*m): a leg stays where it is pushed, like soft steel bent past its yield point. The legs bend
only outward, one each way (the short leg toward +y, the long leg toward -y), and they collide with each other
and with the pin.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/cotter-pin-spread-legs/cotter_pin_spread_legs.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges; installs its yam_arena hooks in memory, incl. per-episode mk.MEMORY)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PX, PY = 0.330, 0.235  # clevis pin axis
PIN_W = 0.018  # square clevis pin
FACE_X = PX - PIN_W / 2  # the pin's near face, where the cotter goes in (the eye seats here)
FAR_X = PX + PIN_W / 2  # the far face, where the legs come out
HOLE_W, HOLE_H = 0.013, 0.012  # cross-hole, along x
HOLE_Z = TOP + 0.100  # cross-hole centre height
HOLE_FLOOR = HOLE_Z - HOLE_H / 2
PIN_BOTTOM, PIN_TOP = TOP + 0.030, TOP + 0.115
CHEEK_TOP = TOP + 0.074

# Cotter (part frame: x along the cotter toward the legs, origin at the centre of the eye's seat face, at the legs'
# mid-height). Eye loop 16 x 24 x 8 mm; shank (the two legs' roots, inside the hole) 18 mm; legs 5 mm wide, 8 mm tall.
EYE_L, EYE_W, BAR = 0.016, 0.024, 0.004
LEG_W, LEG_H, LEG_GAP = 0.005, 0.008, 0.0004
SHANK = PIN_W  # the legs' roots: the hinge line sits at the far face once the eye is seated
SHORT, LONG = 0.022, 0.040  # legs past the shank
COTTER_KG, SHORT_KG, LONG_KG = 0.026, 0.007, 0.012
LEG_FRICTION = 0.010  # N*m at each root: about 0.3-0.6 N at the leg bends it, and it stays bent
LEG_Y = LEG_GAP / 2 + LEG_W  # each leg's outer edge, where its bend axis runs

HOLDER = (0.070, 0.014, 0.030)  # white ABS holder the cotter lies on
HX, HY = 0.240, 0.235
COTTER_Z0 = TOP + HOLDER[2] + LEG_H / 2  # the cotter's mid-height lying on the holder
COTTER_X0 = HX - (SHANK + LONG - EYE_L) / 2  # its origin, so it lies centred on the holder

STEEL = (0.62, 0.63, 0.66)
DARK = (0.30, 0.31, 0.33)
YELLOW = (0.85, 0.68, 0.10)
ZINC = (0.80, 0.80, 0.74)
WHITE = (0.92, 0.92, 0.90)

kit = mk.Kit()
_x0 = np.array([COTTER_X0, HY, COTTER_Z0])
kit.part("cotter", tuple(_x0), [
    mk.box((BAR, EYE_W, LEG_H), at=(-BAR / 2, 0, 0)),  # eye: front bar (the seat face)
    mk.box((BAR, EYE_W, LEG_H), at=(-EYE_L + BAR / 2, 0, 0)),  # back bar
    *(mk.box((EYE_L - 2 * BAR, BAR, LEG_H), at=(-EYE_L / 2, s * (EYE_W - BAR) / 2, 0)) for s in (1, -1)),
    *(mk.box((SHANK, LEG_W, LEG_H), at=(SHANK / 2, s * (LEG_GAP + LEG_W) / 2, 0)) for s in (1, -1)),  # shank
], mass=COTTER_KG, color=ZINC)
kit.part("leg_short", tuple(_x0), [mk.box((SHORT, LEG_W, LEG_H), at=(SHANK + SHORT / 2, (LEG_GAP + LEG_W) / 2, 0))],
         mass=SHORT_KG, color=ZINC)
kit.part("leg_long", tuple(_x0), [mk.box((LONG, LEG_W, LEG_H), at=(SHANK + LONG / 2, -(LEG_GAP + LEG_W) / 2, 0))],
         mass=LONG_KG, color=ZINC)
# Each leg bends about the vertical through its outer edge at the root; positive angle = outward.
kit.hinge("bend_short", "cotter", "leg_short", anchor=tuple(_x0 + (SHANK, LEG_Y, 0)), axis=(0, 0, 1),
          lower_deg=0.0, upper_deg=120.0, friction=LEG_FRICTION)
kit.hinge("bend_long", "cotter", "leg_long", anchor=tuple(_x0 + (SHANK, -LEG_Y, 0)), axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=120.0, friction=LEG_FRICTION)

LEGS = {"leg_short": "bend_short", "leg_long": "bend_long"}
PARTS = ("cotter", "leg_short", "leg_long")


def _clevis():
    hole_z = HOLE_Z
    wall = (PIN_W - HOLE_W) / 2
    return [
        Box("stand", pos=(PX, PY, TOP + 0.015), size=(0.070, 0.056, 0.030), color=DARK),
        Box("cheek_lower", pos=(PX, PY, TOP + 0.035), size=(0.050, 0.044, 0.010), color=STEEL),
        Box("cheek_upper", pos=(PX, PY, CHEEK_TOP - 0.005), size=(0.050, 0.044, 0.010), color=STEEL),
        Box("yoke_web", pos=(PX - 0.028, PY, TOP + 0.052), size=(0.006, 0.044, 0.044), color=STEEL),
        Box("rod_end", pos=(PX + 0.039, PY, TOP + 0.052), size=(0.102, 0.022, 0.016), color=YELLOW),
        Box("pin_lower", pos=(PX, PY, (PIN_BOTTOM + HOLE_FLOOR) / 2), size=(PIN_W, PIN_W, HOLE_FLOOR - PIN_BOTTOM),
            color=STEEL),
        Box("pin_wall_left", pos=(PX, PY + (HOLE_W + wall) / 2, hole_z), size=(PIN_W, wall, HOLE_H), color=STEEL),
        Box("pin_wall_right", pos=(PX, PY - (HOLE_W + wall) / 2, hole_z), size=(PIN_W, wall, HOLE_H), color=STEEL),
        Box("pin_end", pos=(PX, PY, (HOLE_Z + HOLE_H / 2 + PIN_TOP) / 2), size=(PIN_W, PIN_W, PIN_TOP - HOLE_Z - HOLE_H / 2),
            color=STEEL),
        Box("holder", pos=(HX, HY, TOP + HOLDER[2] / 2), size=HOLDER, color=WHITE),
    ]


OBJECTS = [*kit.objects(), *_clevis()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# A cotter pin retains only once its legs are bent: a straight cotter shakes out of the hole and the clevis pin
# walks out of the linkage. Measured from the parts' poses: where the eye sits on the pin, and each leg's bend angle
# relative to the cotter (the angle between the leg bar and the shank, as on a photo).

SEAT_TOL = 0.003  # the eye's seat face within 3 mm of the pin's near face
CENTRE_TOL = 0.003  # the shank's axis within 3 mm of the hole's centre line (y), 6 mm (z)
SHANK_TILT = 10.0  # ... and within 10 deg of the hole's axis
BENT_DEG = 45.0  # a leg counts as bent at 45 deg
REBEND_DEG = 30.0  # a leg bent back 30 deg from its furthest bend after passing 45 deg: re-bent (latched)
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the part's bounding box."""
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _seated(objects) -> bool:
    """The cotter in the cross-hole: eye against the pin's near face, shank on the hole's axis."""
    o = kit.point(objects, "cotter", (0.0, 0.0, 0.0))
    a = kit.point(objects, "cotter", (SHANK, 0.0, 0.0)) - o
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(a[0]) / float(np.linalg.norm(a))))))
    return bool(FACE_X - SEAT_TOL <= o[0] <= FACE_X + 0.002 and abs(o[1] - PY) <= CENTRE_TOL
                and abs(o[2] - HOLE_Z) <= 0.006 and tilt <= SHANK_TILT)


def bends(objects) -> dict:
    """Each leg's bend from straight, degrees (outward positive)."""
    return {leg: kit.q_deg(objects, hinge) for leg, hinge in LEGS.items()}


def _rebent(objects) -> bool:
    """Latched for the trial: a leg bent past 45 deg and then bent back 30 deg or more (a reused cotter)."""
    for leg, deg in bends(objects).items():
        key = ("cps_max", leg)
        top = max(mk.MEMORY.get(key, 0.0), deg)
        mk.MEMORY[key] = top
        if top >= BENT_DEG and deg <= top - REBEND_DEG:
            mk.MEMORY["cps_rebent"] = True
    return bool(mk.MEMORY.get("cps_rebent"))


def _still(objects) -> bool:
    """Nothing moving: from one control step to the next the eye's seat face moves under 0.3 mm and each leg's bend
    changes under 0.5 deg (6 mm/s, 10 deg/s). Measured from poses because the friction-held legs and the cotter
    chatter in sim (spin readings up to ~1.5 rad/s with the poses unchanged to 0.1 mm)."""
    o = kit.point(objects, "cotter", (0.0, 0.0, 0.0))
    b = bends(objects)
    frame = mk._EP.get("step") if mk._EP.get("env") is not None else id(objects)  # offline: one dict = one step
    prev = mk.MEMORY.get("cps_prev")
    if prev is not None and prev[0] == frame:  # score() and success() in the same step
        prev = mk.MEMORY.get("cps_prev_before")
    else:
        mk.MEMORY["cps_prev_before"] = prev
        mk.MEMORY["cps_prev"] = (frame, o, b)
    if prev is None:
        return False
    return bool(np.linalg.norm(o - prev[1]) < 0.0003 and all(abs(b[g] - prev[2][g]) < 0.5 for g in b))


def score(objects) -> float:
    """0.25: the cotter fully in the cross-hole (eye within 3 mm of the pin's face, shank centred and within 10 deg
    of the hole axis) and let go of. 0.5: also one leg bent 45 deg or more. 1.0: both legs bent 45 deg or more (one
    each way), everything still and let go of. A leg bent back 30 deg after passing 45 deg caps the trial at 0.25."""
    rebent = _rebent(objects)
    if not (_seated(objects) and all(_released(objects[p]) for p in PARTS)):
        return 0.0
    if rebent:
        return 0.25
    n = sum(d >= BENT_DEG for d in bends(objects).values())
    if n == 2 and _still(objects):
        return 1.0
    return 0.5 if n >= 1 else 0.25


def success(objects) -> bool:
    """Cotter in, both legs bent, let go; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the eye's seat face to the pin's face, plus the arc the legs' tips still have to travel."""
    o = kit.point(objects, "cotter", (0.0, 0.0, 0.0))
    arc = sum(max(0.0, math.radians(BENT_DEG - d)) * (SHORT if leg == "leg_short" else LONG)
              for leg, d in bends(objects).items())
    return float(abs(FACE_X - o[0]) + abs(PY - o[1]) + abs(HOLE_Z - o[2]) + arc)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm only, open loop from the scene's fixed geometry (it reads no joint or simulator state). Pick the cotter
# by its eye (fingers closing along y on the eye's sides, which pass beside the 18 mm pin), slide it level through
# the cross-hole until the eye stops on the pin, let go. Then, with the fingers closed, push the long leg round the
# far face of the pin from beyond the short leg's tip, and the short leg round the other way, each along an arc.

TIP = 0.013  # pad tips below the TCP
SQUEEZE = 0.006
TRAVEL = TOP + 0.170
C, O = FINGER_CLOSED, FINGER_OPEN
GRIP = -(EYE_W - SQUEEZE) / 2
EYE_C = -EYE_L / 2  # grip at the eye's centre (part frame x)
GRIP_DZ = -LEG_H / 2 + 0.001 + TIP  # TCP above the cotter's mid-height: pad tips 1 mm above the eye's bottom
PUSH_Z = HOLE_FLOOR - 0.003 + TIP  # closed pads' tips 3 mm below the legs
HALF_W = 0.019  # closed pad pair: half width (across the closing axis)
PRESS, CLEAR = -0.005, 0.006  # pusher face into / off the leg's inner face
R_LONG, R_SHORT = 0.035, 0.020  # pusher centre's distance along each leg from its root
ARC = (0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 105)  # past 90: pressed flat on the pin


def _long_tcp(deg, gap):
    """Pusher on the long leg's inner (+y) side, closing axis along the leg (so only the part past the short leg's
    tip is touched)."""
    t = math.radians(deg)
    d, n = np.array([math.cos(t), -math.sin(t)]), np.array([math.sin(t), math.cos(t)])
    c = np.array([FAR_X, PY - LEG_Y]) + R_LONG * d + (LEG_W + gap + HALF_W) * n
    return (float(c[0]), float(c[1]), PUSH_Z), math.pi / 2 - t


def _short_tcp(deg, gap):
    """Pusher on the short leg's inner (-y) side, closing axis along the leg (the pads' edges push, as for the long
    leg); the long leg, already round the pin, is clear of it."""
    t = math.radians(deg)
    d, n = np.array([math.cos(t), math.sin(t)]), np.array([math.sin(t), -math.cos(t)])
    c = np.array([FAR_X, PY + LEG_Y]) + R_SHORT * d + (LEG_W + gap + HALF_W) * n
    return (float(c[0]), float(c[1]), PUSH_Z), t - math.pi / 2


def _bend(tcp_fn, label):
    p0, y0 = tcp_fn(0, CLEAR)
    out = [Waypoint(2.0, (p0[0], p0[1], TRAVEL), C, f"{label}: over the leg", y0),
           Waypoint(1.5, p0, C, f"{label}: down beside it", y0),
           Waypoint(0.4, p0, C, f"{label}: settle", y0)]
    for deg in ARC:
        p, y = tcp_fn(deg, PRESS)
        out.append(Waypoint(0.7, p, C, f"{label}: bend to {deg} deg", y))
    p, y = tcp_fn(ARC[-1], PRESS)
    q, _ = tcp_fn(ARC[-1], 0.012)
    out += [Waypoint(0.4, p, C, f"{label}: hold", y),
            Waypoint(0.8, q, C, f"{label}: back off", y),
            Waypoint(1.2, (q[0], q[1], TRAVEL), C, f"{label}: lift clear", y)]
    return out


def demo():
    """Pick the cotter, slide it through the cross-hole and let go; bend the long leg round the pin, then the short
    leg the other way; park. About 41 s."""
    pick = (COTTER_X0 + EYE_C, HY, COTTER_Z0 + GRIP_DZ)
    z_in = HOLE_Z + GRIP_DZ
    start_x = FACE_X + EYE_C - SHANK - LONG - 0.008  # the long leg's tip 8 mm short of the pin's face
    home_x = FACE_X + EYE_C + 0.0015  # the eye 1.5 mm past the face: pressed against it
    left = [
        Waypoint(2.0, (pick[0], pick[1], TRAVEL), O, "over the cotter", 0.0),
        Waypoint(1.5, (pick[0], pick[1], pick[2] + 0.020), O, "descend", 0.0),
        Waypoint(1.0, pick, O, "down to the eye", 0.0),
        Waypoint(0.5, pick, O, "settle", 0.0),
        Waypoint(1.0, pick, GRIP, "close on the eye", 0.0),
        Waypoint(0.5, pick, GRIP, "settle closed", 0.0),
        Waypoint(1.5, (pick[0], pick[1], TRAVEL), GRIP, "lift", 0.0),
        Waypoint(2.0, (start_x, PY, TRAVEL), GRIP, "carry behind the pin", 0.0),
        Waypoint(1.5, (start_x, PY, z_in + 0.015), GRIP, "lower", 0.0),
        Waypoint(1.0, (start_x, PY, z_in), GRIP, "level with the cross-hole", 0.0),
        Waypoint(0.6, (start_x, PY, z_in), GRIP, "settle", 0.0),
        Waypoint(4.0, (home_x, PY, z_in), GRIP, "slide it through", 0.0),
        Waypoint(0.5, (home_x, PY, z_in), GRIP, "hold", 0.0),
        Waypoint(1.0, (home_x, PY, z_in), O, "let go", 0.0),
        Waypoint(1.2, (home_x, PY, TRAVEL), O, "lift clear", 0.0),
    ]
    left += _bend(_long_tcp, "long leg") + _bend(_short_tcp, "short leg")
    left += [Waypoint(1.5, (0.220, 0.300, TOP + 0.200), C, "park", 0.0)]
    return {"left": left}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built poses (single states and short sequences, one trial each)."""
    global _TEST_TCPS
    seat = (np.array([FACE_X, PY, HOLE_FLOOR + LEG_H / 2]), mk.yaw_quat(0.0))

    def st(short=0.0, long=0.0, at=None):
        return kit.fake_states(q={"bend_short": short, "bend_long": long}, deg=True,
                               moved={"cotter": at} if at is not None else {})

    start = st()
    held = st(at=(np.array([FACE_X - 0.030, PY, HOLE_Z]), mk.yaw_quat(0.0)))
    inserted = st(at=seat)
    one = st(long=90.0, at=seat)
    both = st(short=90.0, long=90.0, at=seat)
    both_50 = st(short=50.0, long=50.0, at=seat)
    short_40 = st(short=40.0, long=90.0, at=seat)
    backed_out = st(short=90.0, long=90.0, at=(np.array([FACE_X - 0.006, PY, HOLE_FLOOR + LEG_H / 2]), mk.yaw_quat(0.0)))
    cocked = st(short=90.0, long=90.0, at=(seat[0], mk.yaw_quat(math.radians(15.0))))
    bent_in_holder = st(short=90.0, long=90.0)
    rebent = st(short=90.0, long=55.0, at=seat)
    both_again = st(short=90.0, long=90.0, at=seat)  # the next step, nothing moved
    still_50 = st(short=50.0, long=50.0, at=seat)
    swinging = st(short=88.0, long=90.0, at=seat)  # the short leg still moving 2 deg per step
    sliding = st(short=90.0, long=90.0, at=(seat[0] + np.array([0.0, 0.0005, 0.0]), seat[1]))
    eye_tcp = kit.point(both, "cotter", (EYE_C, 0.0, GRIP_DZ))

    cases = {  # name: (sequence of states in one trial, gripper TCPs, final score, success)
        "start (cotter on the holder)": ([start], [], 0.0, False),
        "cotter held behind the pin": ([start, held], [], 0.0, False),
        "naive: cotter pushed in straight and let go": ([start, held, inserted], [], 0.25, False),
        "inserted but still gripped by the eye": ([start, held, inserted], [eye_tcp], 0.0, False),
        "long leg bent round the pin": ([start, inserted, one], [], 0.5, False),
        "both legs bent round the pin, still": ([start, inserted, one, both, both_again], [], 1.0, True),
        "both legs bent round the pin, first step there": ([start, inserted, one, both], [], 0.5, False),
        "both legs bent 50 deg": ([start, inserted, both_50, still_50], [], 1.0, True),
        "short leg only 40 deg": ([start, inserted, short_40], [], 0.5, False),
        "both bent, still being held": ([start, inserted, both], [eye_tcp], 0.0, False),
        "both bent, short leg still swinging": ([start, inserted, swinging, both], [], 0.5, False),
        "both bent, cotter still sliding": ([start, inserted, sliding, both], [], 0.5, False),
        "legs bent but the cotter backed 6 mm out of the hole": ([start, backed_out], [], 0.0, False),
        "legs bent, shank cocked 15 deg in the hole": ([start, cocked], [], 0.0, False),
        "legs bent with the cotter still on the holder": ([start, bent_in_holder], [], 0.0, False),
        "long leg bent back 35 deg (re-bent)": ([start, inserted, both, rebent], [], 0.25, False),
        "re-bent, then bent round again": ([start, inserted, both, rebent, both, both_again], [], 0.25, False),
        "end state right, two steps, no other history": ([both, both_again], [], 1.0, True),
    }
    try:
        for name, (seq, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            for s in seq[:-1]:
                score(s)
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        # a new trial forgets the last one's re-bend
        mk.new_episode()
        _TEST_TCPS = []
        score(both)
        score(rebent)
        mk.new_episode()
        score(both)
        assert score(both_again) == 1.0, "reset"
        # the readouts match the hand-set bends
        b = bends(both_50)
        assert all(abs(v - 50.0) < 1e-6 for v in b.values()), b
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1




if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
