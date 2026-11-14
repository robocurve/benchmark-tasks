"""A hospital overbed table stands across a bed, its 380 x 760 mm top cantilevered from a single column at its left
end, over the patient's shins. The column is a locking gas spring: the top is held where it is until the release
paddle under the top's front edge is lifted; then the spring pushes the top up with a nearly constant net force,
so lowering it means holding the paddle up with one hand while pressing the top down near its middle with the
other, and letting go of the paddle first (the column locks) and only then of the top. A measuring post with a
red tape tab stands beside the top's right-hand end; the tab marks the height the top is to be set to.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/overbed-table-lower/overbed_table_lower.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 900 \\
           --instruction "Lower the overbed table to the marked height. Its single column is a locking gas spring, released by the paddle under the front edge of the top." --scorer task_score ...
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
TOP = TABLE_TOP[SCENE]  # the bench top stands in for the mattress surface

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
LAMINATE = (0.86, 0.80, 0.68)
STEEL = (0.62, 0.64, 0.66)
DARK = (0.18, 0.19, 0.21)
BLANKET = (0.55, 0.68, 0.80)
WHITE = (0.93, 0.93, 0.92)
RED = (0.85, 0.12, 0.10)

# Table top: 380 mm deep (x) x 760 mm long (y) x 18 mm laminated board, column under its left end.
TOP_SIZE = (0.380, 0.760, 0.018)
TOP_C = (0.330, 0.080)
FRONT_X = TOP_C[0] - TOP_SIZE[0] / 2  # 0.14
RIGHT_Y = TOP_C[1] - TOP_SIZE[1] / 2  # -0.30
COL_XY = (0.330, 0.400)
S0 = TOP + 0.140  # top surface at the start
TARGET = TOP + 0.100  # the marked height: 40 mm lower
US0 = S0 - TOP_SIZE[2]  # underside at the start
TOP_KG = 5.5
CARRIAGE_KG = 2.5
PADDLE_KG = 0.05
G = 9.81
WEIGHT = (TOP_KG + CARRIAGE_KG + PADDLE_KG) * G  # 79 N carried by the column

# Gas spring column: 86 N push at the start (about 7 N net up on the 8 kg it carries), falling 50 N/m with travel.
GAS_N = 86.0
GAS_K = 50.0
GAS_REST = GAS_N / GAS_K  # the spring's rest point, metres above the start
LOCK_K = 80000.0  # the locked column: very stiff
LIFT_DAMP = 60.0  # N*s/m, the gas spring's oil damping
TRAVEL_DOWN, TRAVEL_UP = 0.064, 0.060
POST_L = TRAVEL_DOWN - 0.003  # inner post under the bracket: never reaches the bench
TILT_K = 250.0  # N*m/rad: the column and bracket's give when the top is leaned on
TILT_DEG = 4.0
TILT_OK_DEG = 1.0  # the top must end up level to within this to earn any height credit

kit = mk.Kit()
# fixed outer column on its foot (the H-base on castors on the floor in a ward; here bolted to the bench)
kit.part("column", (COL_XY[0], COL_XY[1], TOP),
         [mk.box((0.300, 0.080, 0.010), at=(0, 0, 0.005), color=DARK),
          mk.box((0.050, 0.050, US0 - TRAVEL_DOWN - 0.001 - TOP - 0.010),
                 at=(0, 0, 0.010 + (US0 - TRAVEL_DOWN - 0.001 - TOP - 0.010) / 2))],
         mass=None, color=STEEL)
# inner post with the bracket under the top
kit.part("carriage", (COL_XY[0], COL_XY[1], US0),
         [mk.box((0.036, 0.036, POST_L), at=(0, 0, -POST_L / 2)),
          mk.box((0.060, 0.100, 0.006), at=(0, 0, -0.003), color=DARK)],
         mass=CARRIAGE_KG, color=STEEL)
kit.slider("lift", "column", "carriage", axis=(0, 0, 1), lower=-TRAVEL_DOWN, upper=TRAVEL_UP, stiffness=LOCK_K,
           rest=WEIGHT / LOCK_K, damping=LIFT_DAMP)
# The top: board plus the paddle's hanger. Its weight on the cantilever is preloaded out of the tilt spring.
PIVOT = (FRONT_X + 0.030, 0.250, US0 - 0.012)  # paddle pivot, 30 mm in from the front edge
_tilt_tau = TOP_KG * G * (COL_XY[1] - TOP_C[1])
kit.part("top", (TOP_C[0], TOP_C[1], US0),
         [mk.box(TOP_SIZE, at=(0, 0, TOP_SIZE[2] / 2)),
          mk.box((0.014, 0.030, 0.008), at=(PIVOT[0] + 0.012 - TOP_C[0], PIVOT[1] - TOP_C[1], -0.004), color=DARK),
          mk.box((0.012, 0.012, 0.0015), at=(PIVOT[0] - 0.060 - TOP_C[0], PIVOT[1] - TOP_C[1], TOP_SIZE[2] + 0.0007),
                 color=RED)],  # a red dot on the top over the paddle
         mass=TOP_KG, color=LAMINATE)
kit.hinge("tilt", "carriage", "top", anchor=(COL_XY[0], COL_XY[1], US0), axis=(1, 0, 0), lower_deg=-12.0,
          upper_deg=12.0, stiffness=TILT_K, rest_deg=-math.degrees(_tilt_tau / TILT_K), damping=8.0)
# Release paddle: a 105 mm black bar 20 mm wide, pivoted under the top, sticking 65 mm out past the front edge.
# Lifting its tip 7 deg (8 mm at the fingers) unlatches the column; a spring returns it.
PADDLE_L = 0.105
PADDLE = (PADDLE_L, 0.020, 0.008)
RELEASE_DEG = 6.0
RELOCK_DEG = 4.5
kit.part("paddle", PIVOT, [mk.box(PADDLE, at=(-PADDLE_L / 2 + 0.010, 0, 0))], mass=PADDLE_KG, color=DARK)
kit.hinge("paddle_pull", "top", "paddle", anchor=PIVOT, axis=(0, 1, 0), lower_deg=0.0, upper_deg=20.0,
          stiffness=0.3, rest_deg=-8.0, damping=0.002)

# The patient's shins under a blanket, 68 mm proud of the mattress, under the top's right half. They give like
# flesh on a mattress: a 4000 N/m spring (30 N at 6.5 mm), preloaded 4 N onto their stop.
LEG = (0.300, 0.240, 0.068)
LEG_XY = (0.360, -0.090)
LEG_KG = 2.0
LEG_K = 4000.0
LEG_PRELOAD = 4.0
LEG_LIMIT_N = 30.0
kit.part("legs", (LEG_XY[0], LEG_XY[1], TOP + LEG[2]), [mk.box(LEG, at=(0, 0, -LEG[2] / 2 + 0.001))],
         mass=LEG_KG, color=BLANKET)
kit.slider("leg_give", None, "legs", axis=(0, 0, -1), lower=0.0, upper=0.020, stiffness=LEG_K,
           rest=-(LEG_KG * G + LEG_PRELOAD) / LEG_K, damping=40.0)

# Measuring post beside the top's right end; the red tab's top face is the marked height.
POST_XY = (0.170, RIGHT_Y - 0.025)
MEASURE = (0.170, RIGHT_Y + 0.020)  # top-surface point measured: by the post, 20 mm in from the right end

FIXTURES = [
    Box("post_foot", pos=(POST_XY[0], POST_XY[1], TOP + 0.004), size=(0.060, 0.060, 0.008), color=DARK),
    Box("post", pos=(POST_XY[0], POST_XY[1], (TOP + S0 + 0.030) / 2), size=(0.016, 0.016, S0 + 0.030 - TOP),
        color=WHITE),
    Box("mark", pos=(POST_XY[0], POST_XY[1] + 0.010, TARGET - 0.004), size=(0.020, 0.012, 0.008), color=RED),
]
OBJECTS = [*kit.objects(), *FIXTURES]

DT = 0.05  # one control step at 20 Hz
HOLD_S = 2.0  # the locked top must stay put this long after it is let go
DRIFT = 0.003
SURGE = 0.040
TOL = 0.010
GATE = 0.020


def _drive(stiffness, target):
    path = mk._SIM["joint_paths"].get((kit.id, "lift"))
    if not path or mk._SIM["stage"] is None:
        return
    from pxr import UsdPhysics

    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear")
    d.GetStiffnessAttr().Set(float(stiffness))
    d.GetTargetPositionAttr().Set(float(target))


def _lock(q):
    _drive(LOCK_K, q + WEIGHT / LOCK_K)
    mk.MEMORY["mode"] = "locked"


def _column_rule(objects):
    """The gas spring locks unless the paddle is lifted; with it lifted the spring pushes the top up."""
    q, pull = kit.q(objects, "lift"), kit.q_deg(objects, "paddle_pull")
    mode = mk.MEMORY.get("mode")
    if mode is None:
        _lock(q)
    elif mode == "locked" and pull >= RELEASE_DEG:
        _drive(GAS_K, GAS_REST)
        mk.MEMORY["mode"] = "free"
    elif mode == "free" and pull < RELOCK_DEG:
        _lock(q)


def _top_released(objects):
    return mk.released(objects["top"], 0.020) and mk.released(objects["paddle"], 0.020)


def _watch_rule(objects):
    """Latches, and how long the top has stood locked and untouched."""
    m = mk.MEMORY
    m["t"] = m.get("t", 0.0) + DT
    q = kit.q(objects, "lift")
    m["q_min"] = min(m.get("q_min", q), q)
    if q > m["q_min"] + SURGE:
        m["surge"] = True
    if abs(kit.q_deg(objects, "tilt")) > TILT_DEG:
        m["tilted"] = True
    if leg_force(objects) > LEG_LIMIT_N:
        m["leg_hit"] = True
    if m.get("mode") == "locked" and _top_released(objects):
        if m.get("since") is None:
            m["since"], m["q0"], m["drift"] = m["t"], q, 0.0
        m["drift"] = max(m["drift"], abs(q - m["q0"]))
    else:
        m["since"] = None
    if os.environ.get("OVERBED_DEBUG") and round(m["t"] / DT) % 5 == 0:
        print(f"[overbed] t={m['t']:.2f} q={q*1000:.1f}mm h_err={(height(objects)-TARGET)*1000:.1f}mm "
              f"corner_err={(corner_height(objects)-TARGET)*1000:.1f}mm "
              f"pull={kit.q_deg(objects, 'paddle_pull'):.1f} tilt={kit.q_deg(objects, 'tilt'):.2f} "
              f"leg={leg_force(objects):.1f}N mode={m.get('mode')} since={m.get('since')} "
              f"drift={m.get('drift')} surge={m.get('surge')} tilted={m.get('tilted')} "
              f"leg_hit={m.get('leg_hit')} score={score(objects):.3f}", flush=True)


kit.on_step(_column_rule)
kit.on_step(_watch_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def height(objects) -> float:
    """The top's set height: its surface height at the start plus the column's travel. Leaning the top over on the
    column's give does not count."""
    return S0 + kit.q(objects, "lift")


def corner_height(objects) -> float:
    """Top-surface height beside the measuring post (for the debug print only)."""
    lx, ly = MEASURE[0] - TOP_C[0], MEASURE[1] - TOP_C[1]
    return float(kit.point(objects, "top", (lx, ly, TOP_SIZE[2]))[2])


def leg_force(objects) -> float:
    """Force on the shins from their spring's compression (0 while they rest on their stop)."""
    d = kit.q(objects, "leg_give")
    return 0.0 if d <= 0.0005 else LEG_K * d + LEG_PRELOAD


def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("surge") or m.get("tilted") or m.get("leg_hit"))


def score(objects) -> float:
    """0, latched, if at any time the column sprang up 40 mm above the lowest point it had reached, the top tilted
    more than 4 deg, or the shins were pressed with more than 30 N. 0 also if the top now tilts more than 1 deg.
    Otherwise, with e = |column height - mark| (start height plus the column's travel, not a corner of the top):
    0.6 for e of 10 mm or less, falling linearly to 0 at 33 mm (the start is 40 mm off); with e of 20 mm or less,
    +0.3 if the column is locked and the top, let go (no gripper within 2 cm of top or paddle), has stayed within
    3 mm for 2 s, and +0.1 if the top and paddle are let go and the top is still."""
    if failed(objects) or abs(kit.q_deg(objects, "tilt")) > TILT_OK_DEG:
        return 0.0
    e = abs(height(objects) - TARGET)
    s = 0.6 * min(1.0, max(0.0, (0.033 - e) / (0.033 - TOL)))
    if e <= GATE:
        m = mk.MEMORY
        free = _released(objects["top"]) and _released(objects["paddle"])
        if (m.get("mode") == "locked" and m.get("since") is not None and m["t"] - m["since"] >= HOLD_S - 1e-9
                and m.get("drift", 1.0) < DRIFT and free):
            s += 0.3
        if free and objects["top"].resting():
            s += 0.1
    return round(s, 6)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return abs(height(objects) - TARGET)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm: comes down over the paddle's tip, 35 mm out from the front edge, closes on it (the fingers close along
# y, across the 20 mm bar) and lifts it 9 mm. Right arm: closed fingers on the top near its middle. The two then
# go down together 40 mm, the left still holding the paddle lifted; the left lets the paddle go (the column locks)
# and backs away, then the right lifts off the top.

TIP = 0.013
DROP = S0 - TARGET
GRIP_X = FRONT_X - 0.050
PADDLE_Z = PIVOT[2]
PRESS_XY = (0.280, 0.000)
LIFT_PULL = float(os.environ.get("OVERBED_PULL", "0.030"))
PRESS_IN = float(os.environ.get("OVERBED_PRESS_IN", "0.030"))  # commanded below the surface: the arm's give
PARK_L, PARK_R = (0.15, 0.30, S0 + 0.04), (0.15, -0.30, S0 + 0.04)
OPEN = -0.022


T_DOWN = 6.0  # both arms start down together
DESCENT = 4.0


def _left(variant):
    gz = PADDLE_Z - PADDLE[2] / 2 - 0.002 + TIP  # pad tips 2 mm below the bar
    up = S0 + 0.030
    pz = gz + LIFT_PULL
    plan = [
        Waypoint(2.0, (GRIP_X, PIVOT[1], up), OPEN, "over the paddle's tip", 0.0),
        Waypoint(1.5, (GRIP_X, PIVOT[1], gz), OPEN, "fingers either side of the paddle", 0.0),
        Waypoint(1.0, (GRIP_X, PIVOT[1], gz), FINGER_CLOSED, "grip the paddle", 0.0),
        Waypoint(1.0, (GRIP_X, PIVOT[1], pz), FINGER_CLOSED, "lift the paddle", 0.0),
    ]
    if variant == "lever_only":
        return plan + [Waypoint(3.0, (GRIP_X, PIVOT[1], pz), FINGER_CLOSED, "hold it", 0.0),
                       Waypoint(1.0, (GRIP_X, PIVOT[1], pz), OPEN, "let go", 0.0),
                       Waypoint(1.0, (GRIP_X, PIVOT[1], up), OPEN, "up", 0.0),
                       Waypoint(2.0, PARK_L, OPEN, "clear", 0.0)]
    if variant in ("press_only", "press_far_end"):
        return [Waypoint(16.0, PARK_L, OPEN, "stay clear", 0.0)]
    if variant == "release_top_first":  # still holding the paddle up after the right hand has let the top go
        return plan + [
            Waypoint(T_DOWN - 5.5, (GRIP_X, PIVOT[1], pz), FINGER_CLOSED, "hold it", 0.0),
            Waypoint(DESCENT, (GRIP_X, PIVOT[1], pz - DROP), FINGER_CLOSED, "follow the top down", 0.0),
            Waypoint(1.0, (GRIP_X, PIVOT[1], pz - DROP), FINGER_CLOSED, "hold", 0.0),
            Waypoint(1.5, (GRIP_X, PIVOT[1], pz + 0.020), FINGER_CLOSED, "let the top ride up, paddle held", 0.0),
            Waypoint(1.0, (GRIP_X, PIVOT[1], pz + 0.020), OPEN, "let the paddle go", 0.0),
            Waypoint(1.0, (GRIP_X, PIVOT[1], pz + 0.023), OPEN, "up", 0.0),
            Waypoint(1.5, PARK_L, OPEN, "clear", 0.0),
        ]
    return plan + [
        Waypoint(T_DOWN - 5.5, (GRIP_X, PIVOT[1], pz), FINGER_CLOSED, "hold it", 0.0),
        Waypoint(DESCENT, (GRIP_X, PIVOT[1], pz - DROP), FINGER_CLOSED, "follow the top down", 0.0),
        Waypoint(1.0, (GRIP_X, PIVOT[1], pz - DROP), FINGER_CLOSED, "hold", 0.0),
        Waypoint(1.0, (GRIP_X, PIVOT[1], pz - DROP), OPEN, "let the paddle go: the column locks", 0.0),
        Waypoint(1.0, (GRIP_X, PIVOT[1], pz - DROP + 0.040), OPEN, "up", 0.0),
        Waypoint(1.5, PARK_L, OPEN, "clear", 0.0),
    ]


def _right(variant):
    x, y = PRESS_XY
    on = S0 + TIP
    if variant == "lever_only":
        return [Waypoint(9.0, PARK_R, FINGER_CLOSED, "stay clear", 0.0)]
    plan = [
        Waypoint(2.0, (x, y, on + 0.015), FINGER_CLOSED, "over the middle of the top", 0.0),
        Waypoint(1.5, (x, y, on + 0.002), FINGER_CLOSED, "fingers on the top", 0.0),
        Waypoint(1.0, (x, y, on - 0.010), FINGER_CLOSED, "lean on it", 0.0),
        Waypoint(T_DOWN - 4.5, (x, y, on - 0.010), FINGER_CLOSED, "wait for the paddle", 0.0),
        Waypoint(DESCENT, (x, y, on - DROP - PRESS_IN), FINGER_CLOSED, "press the top down", 0.0),
        Waypoint(3.5, (x, y, on - DROP - PRESS_IN), FINGER_CLOSED, "hold while the paddle is let go", 0.0),
        Waypoint(1.5, (x, y, on - DROP + 0.030), FINGER_CLOSED, "lift off", 0.0),
        Waypoint(1.5, PARK_R, FINGER_CLOSED, "clear", 0.0),
    ]
    if variant == "press_only":
        plan[3:4] = []
    if variant == "release_top_first":  # lift off as soon as the top is down, before the paddle is let go
        plan[5] = Waypoint(0.5, (x, y, on - DROP - PRESS_IN), FINGER_CLOSED, "brief hold", 0.0)
    if variant == "press_far_end":  # one hand on the far (right) end, column still locked
        fx, fy = 0.250, RIGHT_Y + 0.040
        plan = [
            Waypoint(2.0, (fx, fy, on + 0.015), FINGER_CLOSED, "over the far end of the top", 0.0),
            Waypoint(1.5, (fx, fy, on + 0.002), FINGER_CLOSED, "fingers on the top", 0.0),
            Waypoint(DESCENT, (fx, fy, on - DROP - PRESS_IN), FINGER_CLOSED, "press the far end down", 0.0),
            Waypoint(3.0, (fx, fy, on - DROP - PRESS_IN), FINGER_CLOSED, "hold", 0.0),
            Waypoint(1.5, (fx, fy, on + 0.030), FINGER_CLOSED, "lift off", 0.0),
            Waypoint(1.5, PARK_R, FINGER_CLOSED, "clear", 0.0),
        ]
    return plan


def demo():
    variant = os.environ.get("OVERBED_DEMO", "")
    left, right = _left(variant), _right(variant)
    for plan in (left, right):  # both arms finish, then wait in the clear for the lock check
        plan.append(Waypoint(4.0, plan[-1].pos, plan[-1].finger, "wait", 0.0))
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    D = TARGET - S0  # slider travel to the mark

    def st(**q):
        return kit.fake_states(q=q, deg=True)

    good = {"mode": "locked", "since": 1.0, "t": 4.0, "drift": 0.0005}
    near = [np.array([PRESS_XY[0], PRESS_XY[1], TARGET + TIP])]
    cases = {  # name: (state, tcps, MEMORY, score)
        "start, untouched": (st(), [], {"mode": "locked", "since": 0.0, "t": 5.0, "drift": 0.0}, 0.0),
        "at the mark, locked, let go 3 s": (st(lift=D), [], good, 1.0),
        "8 mm high, locked, let go": (st(lift=D + 0.008), [], good, 1.0),
        "15 mm high, locked, let go": (st(lift=D + 0.015), [], good, 0.869565),
        "25 mm high, locked, let go": (st(lift=D + 0.025), [], good, 0.208696),
        "at the mark, still pressed": (st(lift=D), near, good, 0.6),
        "at the mark, let go only 1 s ago": (st(lift=D), [], {**good, "since": 3.0}, 0.7),
        "at the mark, drifted 4 mm": (st(lift=D), [], {**good, "drift": 0.004}, 0.7),
        "at the mark, column free (paddle held up)": (st(lift=D), [], {**good, "mode": "free"}, 0.7),
        "sprang up 40 mm": (st(lift=D), [], {**good, "surge": True}, 0.0),
        "tilted": (st(lift=D), [], {**good, "tilted": True}, 0.0),
        "pressed onto the shins": (st(lift=D), [], {**good, "leg_hit": True}, 0.0),
        "column at the mark, top tilted 3 deg": (st(lift=D, tilt=3.0), [], good, 0.0),
        "column at the mark, top tilted 0.8 deg": (st(lift=D, tilt=0.8), [], good, 1.0),
        "column locked at start, far end leaned 1 deg down": (st(tilt=-1.0), [], good, 0.0),
    }
    try:
        n = 0
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            if want is not None:
                assert abs(got - want) < 1e-6, (name, got, want)
            n += 1
        s_ = st(leg_give=0.007)
        assert leg_force(s_) > LEG_LIMIT_N, leg_force(s_)
        assert abs(height(st()) - S0) < 1e-6 and abs(height(st(lift=D)) - TARGET) < 1e-6
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return n


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
