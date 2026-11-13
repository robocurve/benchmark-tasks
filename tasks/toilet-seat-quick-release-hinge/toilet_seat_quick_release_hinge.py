"""A toilet seat with quick-release hinges, taken off for cleaning and left lying on the bowl rim 45 mm forward of
its hinge posts. The seat (0.38 m long, 1.2 kg) has two hinge sockets 0.16 m apart at its back edge. Each socket
must drop over its post, and each post carries a spring button that the socket has to push down about 10 mm
(about 12 N) before it clicks and the socket is held. The seat is rigid, so both sockets go down together; cocked,
one socket goes on and the other sits on its post top or jams.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/toilet-seat-quick-release-hinge/toilet_seat_quick_release_hinge.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1200 \\
           --instruction "Put the toilet seat back on." --scorer task_score ...
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

WHITE = (0.94, 0.94, 0.92)
SEAT_C = (0.97, 0.97, 0.95)
CHROME = (0.75, 0.76, 0.78)
BLUE = (0.20, 0.40, 0.80)
GREY_C = (0.55, 0.56, 0.58)

# Frame: x forward from the arm bases, y left, z up, metres.
R = TOP + 0.020  # bowl rim top
PX = 0.380  # hinge posts, x
PY = 0.080  # posts at y = +-PY (0.16 m apart)
SIDES = {"L": 1, "R": -1}
START_DX = -0.045  # the seat starts this far forward of home

# -- posts and buttons -----------------------------------------------------------------------------------------
POST = (0.012, 0.012, 0.020)
BTN = (0.012, 0.012, 0.006)
BTN_K = 600.0  # N/m
BTN_PRELOAD = 0.010  # 6 N preload against the top stop
BTN_LATCH = 0.0085  # pressed this far, it clicks (about 11 N)
BTN_LOCK = 0.010  # held here once clicked
kit = mk.Kit()
for s, sg in SIDES.items():
    kit.part(f"post_{s}", (PX, sg * PY, R), [mk.box(POST, at=(0, 0, POST[2] / 2), color=CHROME)], mass=None,
             color=CHROME)
    kit.part(f"button_{s}", (PX, sg * PY, R), [mk.box(BTN, at=(0, 0, 0.033), color=BLUE)], mass=0.030, color=BLUE)
    kit.slider(f"button_{s}", f"post_{s}", f"button_{s}", axis=(0, 0, 1), lower=-0.012, upper=0.0, start=0.0,
               stiffness=BTN_K, rest=BTN_PRELOAD, damping=2.0)
    t = np.eye(4)
    t[:3, 3] = (PX, sg * PY, R - BTN_LOCK)
    kit.catch(f"lock_{s}", f"button_{s}", t)

# -- seat --------------------------------------------------------------------------------------------------------
# Seat frame: origin on the hinge line between the sockets, at the seat's underside. Home = origin at (PX, 0, R).
SEAT_H = 0.020
CAV = 0.026  # socket bore (posts 12 mm)
WALL = 0.003
HOUS = CAV + 2 * WALL  # 32 mm housing
ROOF_IN = 0.026  # bore ceiling above the seat underside
ROOF = 0.006
_SEAT = [mk.box((0.050, 0.340, SEAT_H), at=(-0.345, 0, SEAT_H / 2)),  # front
         mk.box((0.048, 0.340, SEAT_H), at=(-0.036, 0, SEAT_H / 2)),  # back
         mk.box((0.260, 0.050, SEAT_H), at=(-0.190, 0.145, SEAT_H / 2)),
         mk.box((0.260, 0.050, SEAT_H), at=(-0.190, -0.145, SEAT_H / 2))]
for sg in SIDES.values():
    y = sg * PY
    _SEAT += [mk.box((WALL, HOUS, ROOF_IN), at=(sx * (CAV + WALL) / 2, y, ROOF_IN / 2), color=GREY_C) for sx in (1, -1)]
    _SEAT += [mk.box((CAV, WALL, ROOF_IN), at=(0, y + sy * (CAV + WALL) / 2, ROOF_IN / 2), color=GREY_C)
              for sy in (1, -1)]
    _SEAT.append(mk.box((HOUS, HOUS, ROOF), at=(0, y, ROOF_IN + ROOF / 2), color=GREY_C))
SEAT_KG = 1.2
kit.part("seat", (PX + START_DX, 0.0, R + 0.0005), _SEAT, mass=SEAT_KG, color=SEAT_C)
HOME = np.eye(4)
HOME[:3, 3] = (PX, 0.0, R)
kit.catch("seat_home", "seat", HOME)

BOWL = (0.435, 0.360, R - TOP)
FIXTURES = [
    Box("bowl", pos=(0.395 - BOWL[0] / 2, 0.0, TOP + BOWL[2] / 2), size=BOWL, color=WHITE),
]
OBJECTS = [*kit.objects(), *FIXTURES]

DROP_V = 0.95  # m/s down at a socket: a free fall of about 4.6 cm
OVER_XY = 0.008
OVER_Z = 0.003


def socket(objects, s):
    """World point at the bottom centre of socket s."""
    return kit.point(objects, "seat", (0.0, SIDES[s] * PY, 0.0))


def aligned(objects, s) -> bool:
    c = socket(objects, s)
    return math.hypot(float(c[0]) - PX, float(c[1]) - SIDES[s] * PY) <= OVER_XY


def over(objects, s) -> bool:
    """Socket s down over its post: centred within 4 mm and its bottom within 3 mm of the rim."""
    return aligned(objects, s) and abs(float(socket(objects, s)[2]) - R) <= OVER_Z


def socket_vz(objects, s) -> float:
    """Vertical speed of socket s (m/s, from the seat's linear and angular velocity)."""
    st = objects["seat"]
    r = socket(objects, s) - np.asarray(st.pos, float)
    return float((np.asarray(st.lin_vel, float) + np.cross(np.asarray(st.ang_vel, float), r))[2])


def _rule(objects):
    M = mk.MEMORY
    M["n"] = M.get("n", 0) + 1
    st = objects["seat"]
    if M["n"] > 5 and not M.get("home") and min(socket_vz(objects, s) for s in SIDES) < -DROP_V:
        M["dropped"] = True
    for s in SIDES:
        if not M.get(f"lock_{s}") and kit.q(objects, f"button_{s}") <= -BTN_LATCH and aligned(objects, s):
            kit.engage(f"lock_{s}")
            M[f"lock_{s}"] = True
    if M.get("lock_L") and M.get("lock_R") and not M.get("home") and over(objects, "L") and over(objects, "R"):
        kit.engage("seat_home")
        M["home"] = True
    if float(st.pos[2]) < TOP - 0.050:
        M["fell"] = True
    if os.environ.get("TOILET_DEBUG") and M["n"] % 20 == 0:
        print(f"[toilet] step {M['n']} btn L {kit.q(objects, 'button_L') * 1000:.1f} R "
              f"{kit.q(objects, 'button_R') * 1000:.1f} mm sock L {np.round(socket(objects, 'L'), 4)} "
              f"R {np.round(socket(objects, 'R'), 4)} vz {min(socket_vz(objects, s) for s in SIDES):.2f} "
              f"flags { {k: v for k, v in M.items() if k in ('lock_L', 'lock_R', 'home', 'dropped', 'fell')} } "
              f"score {score(objects)}", flush=True)


kit.on_step(_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects) -> bool:
    M = mk.MEMORY
    return bool(M.get("dropped") or M.get("fell"))


def score(objects) -> float:
    """0 if failed, if a gripper is within 2 cm of the seat, if the seat is moving, or unless both sockets are
    down over their posts. Then 0.3, plus 0.35 for each button clicked."""
    st = objects["seat"]
    if failed(objects) or not _released(st) or not st.resting(max_speed=0.01, max_spin=0.1):
        return 0.0
    if not (over(objects, "L") and over(objects, "R")):
        return 0.0
    return round(0.3 + 0.35 * sum(bool(mk.MEMORY.get(f"lock_{s}")) for s in SIDES), 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(sum(np.linalg.norm(socket(objects, s) - np.array([PX, SIDES[s] * PY, R])) for s in SIDES))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Each arm pinches one socket housing across its sides (fingers along y), lifts the back of the seat 45 mm (the
# front stays on the rim and the seat turns in the pinch), carries it 45 mm back over the posts, lowers both sockets
# onto the buttons together, pushes both down 14 mm past the free button height, lets go and backs off.

TIP = 0.013
GRIP = 0.012 + TIP  # pad tips 12 mm above the seat underside
OPEN = -0.030
LIFT = 0.045
X_TRIM = {"L": 0.017, "R": -0.008}
Y_TRIM = {"L": 0.003, "R": 0.003}  # open-loop correction for where each arm lands its socket
PRESS = -0.020  # TCP target below home (the arm's position error pushes the button)


def _arm(s, press=True, wait=0.0):
    y = SIDES[s] * PY + Y_TRIM[s]
    x0, x1 = PX + START_DX, PX + X_TRIM[s]
    z = R + GRIP
    plan = [Waypoint(2.0, (x0, y, z + 0.050), OPEN, "over the socket", 0.0),
            Waypoint(1.5, (x0, y, z), OPEN, "down round the socket", 0.0),
            Waypoint(1.0, (x0, y, z), FINGER_CLOSED, "pinch the socket", 0.0),
            Waypoint(2.0, (x0, y, z + LIFT), FINGER_CLOSED, "lift the back of the seat", 0.0),
            Waypoint(2.5, (x1, y, z + LIFT), FINGER_CLOSED, "carry it back over the post", 0.0),
            Waypoint(0.5, (x1, y, z + LIFT), FINGER_CLOSED, "settle", 0.0),
            Waypoint(2.0, (x1, y, z + 0.012), FINGER_CLOSED, "lower onto the button", 0.0)]
    if press:
        if wait:
            plan.append(Waypoint(wait, plan[-1].pos, FINGER_CLOSED, "hold while the other arm lets go", 0.0))
        plan += [Waypoint(1.5, (x1, y, z + PRESS), FINGER_CLOSED, "push the socket down", 0.0),
                 Waypoint(1.0, (x1, y, z + PRESS), FINGER_CLOSED, "hold", 0.0)]
    else:
        plan += [Waypoint(2.5, (x1, y, z + 0.012), FINGER_CLOSED, "hold", 0.0)]
    zl = plan[-1].pos[2]
    return plan + [Waypoint(1.0, (x1, y, zl), OPEN, "let go", 0.0),
                   Waypoint(1.5, (x1, y, z + 0.070), OPEN, "up", 0.0),
                   Waypoint(1.5, (x1 - 0.06, y + SIDES[s] * 0.06, z + 0.080), OPEN, "clear", 0.0)]


def demo():
    # DEMO_VARIANT selects a degenerate check (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    if v == "unlatched":  # sockets set on the posts, not pushed down
        return {"left": _arm("L", press=False), "right": _arm("R", press=False)}
    if v == "one_side":  # the right arm lets go of its socket on the button; only the left one is pushed down
        return {"left": _arm("L", wait=4.0), "right": _arm("R", press=False)}
    return {"left": _arm("L"), "right": _arm("R")}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = mk.mat_to_quat(np.eye(3))
    home = (np.array([PX, 0.0, R]), up)
    tilt = mk.mat_to_quat(mk.axis_angle((0, 1, 0), -math.atan2(0.010, 0.37)))
    rest = (np.array([PX, 0.0, R + 0.010]), tilt)
    roll = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.atan2(0.010, 2 * PY)))
    cocked = (np.array([PX, 0.0, R + 0.005]), roll)
    off = (np.array([PX - 0.010, 0.0, R]), up)

    def st(p):
        return kit.fake_states(moved={"seat": p})

    near = [np.array([PX, PY, R + GRIP])]
    both = {"lock_L": True, "lock_R": True}
    cases = {
        "start": (kit.fake_states(), [], {}, 0.0),
        "home, both clicked": (st(home), [], both, 1.0),
        "home, both clicked, still pinched": (st(home), near, both, 0.0),
        "home, one clicked": (st(home), [], {"lock_L": True}, 0.65),
        "home, none clicked": (st(home), [], {}, 0.3),
        "resting on the buttons": (st(rest), [], {}, 0.0),
        "cocked: left down, right up": (st(cocked), [], {"lock_L": True}, 0.0),
        "10 mm off the posts": (st(off), [], both, 0.0),
        "home after a drop": (st(home), [], {**both, "dropped": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        s0 = kit.fake_states()
        for s in SIDES:  # at the start each socket is clear of its post, ahead of it
            assert socket(s0, s)[0] + HOUS / 2 < PX - POST[0] / 2
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
