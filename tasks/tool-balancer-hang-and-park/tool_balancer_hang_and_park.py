"""An assembly station's spring balancer, with its air nutrunner lying on the bench, its air hose uncoupled. The
balancer housing hangs from a boom over the bench; its cable ends in a swivel and a steel hook (a short shank with
a level hook bar and an upturned tip). The constant-force spring pulls the cable up with 12.1 N, 3% over the weight
of the hook and the 1.1 kg tool together, so an empty hook is pulled up hard against the cable stop and a hung tool
drifts up until the park detent in the cable reel holds it, 1.5 cm below the stop.

The nutrunner (a straight air nutrunner, 28 cm long) lies on the bench. It has two loops on top: the hang bail at
its balance point, and the trigger guard around the start lever near the front.

The job: lift the tool to the hook, put the bail (not the trigger guard) over the hook bar, let the spring take the
weight and leave the tool hanging at the park detent. Letting go of an empty hook pulled down lets it whip up into
the stop; dropping the tool is a failure too.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/tool-balancer-hang-and-park/tool_balancer_hang_and_park.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1200 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
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
G = 9.81

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
STEEL = (0.62, 0.64, 0.67)
YELLOW = (0.93, 0.72, 0.10)  # balancer housing
DARK = (0.15, 0.15, 0.16)
TOOL_C = (0.20, 0.32, 0.62)  # nutrunner body
RED = (0.78, 0.14, 0.12)
ORANGE = (0.90, 0.45, 0.10)

# -- balancer ----------------------------------------------------------------------------------------------------
HX, HY = 0.260, 0.050  # the hook's shank (x, y)
STOP_Z = TOP + 0.087  # underside of the hook bar with the cable fully in (against the housing stop)
BAR = (0.006, 0.048, 0.005)  # hook bar along -y from the shank
BAR_Y0, BAR_Y1 = 0.003, -0.045  # its ends (hook frame)
TIP_H = 0.007  # the hook's upturned tip
SHANK_H = 0.025
HOUSING_Z = TOP + 0.700  # underside of the balancer housing
CABLE_TOP = HOUSING_Z - STOP_Z + 0.100  # cable end in the housing (hook frame)
HOOK_KG = 0.10
TOOL_KG = 1.10
SPRING_N = 1.03 * (HOOK_KG + TOOL_KG) * G  # 12.1 N: set 3% over hook and tool, so a hung tool rises into the detent
K = 5.0  # N/m: a soft drive spring aimed 2.4 m up, re-aimed every step so its pull stays SPRING_N (constant force)
TRAVEL_LO = -0.070
DETENT_K = 150.0  # N/m: the park detent, a native spring switched in within DETENT_W of the park point
DETENT_W = 0.008
DETENT_OUT = 0.010  # ...and out again past this (hysteresis)
DAMP_FREE, DAMP_DETENT = 1.0, 8.0  # N*s/m: reel drag, and the detent's damping

kit = mk.Kit()
POST_DX = 0.250
kit.part("balancer", (HX, HY, HOUSING_Z), [
    mk.box((0.090, 0.070, 0.120), at=(0, 0, 0.060), color=YELLOW),
    mk.box((0.030, 0.030, 0.030), at=(0, 0, 0.135), color=DARK),  # suspension eye
    mk.box((POST_DX, 0.030, 0.030), at=(POST_DX / 2, 0, 0.165), color=DARK),  # boom to the post
    mk.box((0.040, 0.040, HOUSING_Z - TOP + 0.180), at=(POST_DX, 0, 0.180 - (HOUSING_Z - TOP + 0.180) / 2), color=DARK),
], mass=None, color=YELLOW)

_HOOK = [
    mk.box(BAR, at=(0, (BAR_Y0 + BAR_Y1) / 2, BAR[2] / 2)),
    mk.box((0.006, 0.006, TIP_H), at=(0, BAR_Y1 + 0.003, BAR[2] + TIP_H / 2)),
    mk.box((0.008, 0.008, SHANK_H), at=(0, 0, SHANK_H / 2)),
    mk.box((0.014, 0.014, 0.016), at=(0, 0, SHANK_H + 0.008), color=DARK),  # swivel
    mk.box((0.003, 0.003, CABLE_TOP - SHANK_H - 0.016), at=(0, 0, (CABLE_TOP + SHANK_H + 0.016) / 2), color=DARK),
]
kit.part("hook", (HX, HY, STOP_Z), _HOOK, mass=HOOK_KG, color=STEEL)
kit.slider("cable", "balancer", "hook", axis=(0, 0, 1), lower=TRAVEL_LO, upper=0.0, start=0.0,
           stiffness=K, rest=SPRING_N / K, damping=DAMP_FREE)

# -- nutrunner ---------------------------------------------------------------------------------------------------
BODY = (0.200, 0.036, 0.036)
WIRE = 0.006
LOOP_IN_W = 0.018  # loop opening across (x)
LOOP_IN_H = 0.024  # loop opening height
GUARD_X = 0.060  # trigger guard, toward the front (+x)
TX = HX
TY0 = HY - 0.024 - 0.090  # start: 9 cm to -y of where the bail goes onto the hook (2.4 cm along the bar)
TZ = TOP + BODY[2] / 2


def _loop(cx, color):
    h = BODY[2] / 2
    return [mk.box((WIRE, WIRE, LOOP_IN_H), at=(cx - LOOP_IN_W / 2 - WIRE / 2, 0, h + LOOP_IN_H / 2), color=color),
            mk.box((WIRE, WIRE, LOOP_IN_H), at=(cx + LOOP_IN_W / 2 + WIRE / 2, 0, h + LOOP_IN_H / 2), color=color),
            mk.box((LOOP_IN_W + 2 * WIRE, WIRE, WIRE), at=(cx, 0, h + LOOP_IN_H + WIRE / 2), color=color)]


_TOOL = [mk.box(BODY),
         mk.box((0.030, 0.030, 0.030), at=(0.115, 0, 0.001), color=DARK),  # angle-free gear head
         mk.box((0.014, 0.014, 0.014), at=(0.137, 0, 0.001), color=STEEL),  # square drive
         mk.box((0.034, 0.030, 0.030), at=(-0.117, 0, 0.001), color=DARK),  # air inlet
         *_loop(0.0, STEEL),  # hang bail at the balance point
         *_loop(GUARD_X, ORANGE),  # trigger guard
         mk.box((0.012, 0.010, 0.008), at=(GUARD_X, 0, BODY[2] / 2 + 0.004), color=RED)]  # start lever
kit.part("tool", (TX, TY0, TZ), _TOOL, mass=TOOL_KG, color=TOOL_C)

OBJECTS = [*kit.objects()]

# The cable and the overhead balancer are drawn but do not collide: on a station the arms work in front of them, and
# here a collider 3 mm thick hanging through the arms' workspace would only snag the arm links.
GHOSTS = {"hook": [4], "balancer": [0, 1, 2, 3]}
_author_all0 = mk._author_all


def _author_all(stage, env_path):
    _author_all0(stage, env_path)
    from pxr import Usd, UsdPhysics

    for part, idx in GHOSTS.items():
        for i in idx:
            root = stage.GetPrimAtPath(f"{env_path}/{part}/box{i}")
            for prim in Usd.PrimRange(root) if root.IsValid() else []:
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)


mk._author_all = _author_all

PARK_Q = -0.015  # park detent: hook bar 1.5 cm below the stop, the hanging tool's underside 1.7 cm over the bench
LOOP_TOP = BODY[2] / 2 + LOOP_IN_H  # inner top of a loop (tool frame)


# -- the balancer spring, detent and latches (every control step) ------------------------------------------------


def _spring(q: float, in_detent: bool):
    """(stiffness, target, damping) of the cable drive: a constant SPRING_N pull, plus the detent spring about
    the park point while in the detent. Inside the detent the drive is a plain PhysX spring (no per-step feedback)."""
    if in_detent:
        return DETENT_K, PARK_Q + SPRING_N / DETENT_K, DAMP_DETENT
    return K, q + SPRING_N / K, DAMP_FREE


def _set_drive(k, target, damping):
    path = mk._SIM["joint_paths"].get((kit.id, "cable"))
    if not path or mk._SIM["stage"] is None:
        return
    from pxr import UsdPhysics

    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear")
    d.GetStiffnessAttr().Set(float(k))
    d.GetDampingAttr().Set(float(damping))
    d.GetTargetPositionAttr().Set(float(target))


def _mechanism_rule(objects):
    M = mk.MEMORY
    q = kit.q(objects, "cable")
    det = M.get("detent", False)
    det = abs(q - PARK_Q) < (DETENT_OUT if det else DETENT_W)
    if det != M.get("detent", False) or not det:
        _set_drive(*_spring(q, det))
    M["detent"] = det
    h, t = objects["hook"], objects["tool"]
    prev = M.get("q_prev", q)
    M["q_prev"] = q
    if not hanging(objects) and (float(h.lin_vel[2]) > 1.5 or q - prev > 0.012):  # an empty hook whipped up
        M["whip"] = True
    if float(t.lin_vel[2]) < -0.95 or float(t.low[2]) < TOP - 0.050:  # dropped (a free fall of 5 cm reaches 0.99 m/s)
        M["dropped"] = True
    if os.environ.get("BAL_DEBUG") and (mk._EP["step"] % 20 == 0 or 290 <= mk._EP["step"] <= 360):
        print(f"[balancer] step {mk._EP['step']} q {q * 1000:.1f} mm hook vz {float(h.lin_vel[2]):.2f} tool low "
              f"{(float(t.low[2]) - TOP) * 1000:.1f} mm vz {float(t.lin_vel[2]):.2f} through {_through(objects, 0.0)} "
              f"flags { {k: v for k, v in M.items() if k in ('whip', 'dropped')} } score {score(objects)} tcps {[list(np.round(p, 3)) for p in mk.tcps()]} tool {np.round(t.pos, 3)}", flush=True)


kit.on_step(_mechanism_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring -----------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _through(objects, loop_x):
    """Where the hook bar crosses the loop's plane (tool frame), if it passes through the opening of the loop at
    tool-frame x ``loop_x``: returns the bar's top height in the tool frame there, else None."""
    th = kit._pose_T(objects, "tool")
    hk = kit._pose_T(objects, "hook")
    inv = mk._inv(th)
    a = (inv @ hk @ np.array([0.0, BAR_Y0 - 0.003, BAR[2] / 2, 1.0]))[:3]
    b = (inv @ hk @ np.array([0.0, BAR_Y1 + 0.006, BAR[2] / 2, 1.0]))[:3]
    if a[1] * b[1] > 0 or abs(a[1] - b[1]) < 1e-9:
        return None
    p = a + (b - a) * (a[1] / (a[1] - b[1]))
    if abs(p[0] - loop_x) > LOOP_IN_W / 2 + 0.002:
        return None
    if not (BODY[2] / 2 - 0.002 <= p[2] - BAR[2] / 2 and p[2] + BAR[2] / 2 <= LOOP_TOP + 0.002):
        return None
    return float(p[2] + BAR[2] / 2)


def hooked(objects) -> bool:
    return _through(objects, 0.0) is not None


def hanging(objects) -> bool:
    """Through the bail, the bail's inner top resting on the hook bar (within 4 mm), the tool's underside 8 mm or
    more above the bench."""
    top = _through(objects, 0.0)
    return top is not None and LOOP_TOP - top <= 0.004 and float(objects["tool"].low[2]) >= TOP + 0.008


HOOK_LO = np.array([-0.007, BAR_Y1, 0.0])
HOOK_HI = np.array([0.007, 0.007, SHANK_H + 0.016])


def _hands_off(objects) -> bool:
    inv = mk._inv(kit._pose_T(objects, "hook"))
    t = objects["tool"]
    lo, hi = np.asarray(t.low) - 0.02, np.asarray(t.high) + 0.02
    for p in _tcps():
        p = np.asarray(p, float)
        if np.all(p >= lo) and np.all(p <= hi):
            return False
        h = (inv @ np.append(p, 1.0))[:3]  # hook frame: handle, shank, swivel and bar
        if np.all(h >= HOOK_LO - 0.02) and np.all(h <= HOOK_HI + 0.02):
            return False
    return True


def parked(objects) -> bool:
    q = kit.q(objects, "cable")
    return (abs(q - PARK_Q) <= 0.006 and _hands_off(objects)
            and objects["tool"].resting(max_speed=0.01, max_spin=0.2)
            and objects["hook"].resting(max_speed=0.01, max_spin=0.2))


def failed(objects) -> bool:
    """Latched: the hook whipped up (over 1.5 m/s, or 12 mm in one 50 ms step), or the tool fell (faster than
    0.95 m/s down, i.e. a free drop of about 5 cm, or 5 cm below the bench top)."""
    M = mk.MEMORY
    return bool(M.get("whip") or M.get("dropped"))


def score(objects) -> float:
    """0 if failed() or the hook is not through the hang bail. Then 0.4; plus 0.3 hanging free from the hook;
    plus 0.3 (with that) the hook at the park detent, both grippers clear and everything still."""
    if failed(objects) or not hooked(objects):
        return 0.0
    s = 0.4
    if hanging(objects):
        s += 0.3
        if parked(objects):
            s += 0.3
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    hk = kit.point(objects, "hook", (0, (BAR_Y0 + BAR_Y1) / 2, 0))
    bl = kit.point(objects, "tool", (0, 0, LOOP_TOP))
    return float(np.linalg.norm(hk - bl) + abs(kit.q(objects, "cable") - PARK_Q))


# -- the scripted demo -------------------------------------------------------------------------------------------
# Both arms pinch the tool (left behind the bail, right on the gear head), lift it until its back is just under the
# hook bar, carry it 9 cm to +y so the bail passes over the hook's tip onto the bar, then lower it: the bail settles
# on the bar, the spring takes the weight, and the arms lower tool and hook together to the park detent and let go.

TIP = 0.013
OPEN_TOOL = -0.032
PARK_L = (0.18, 0.24, TOP + 0.10)
PARK_R = (0.20, -0.24, TOP + 0.10)
REAR_X = TX - 0.070
FRONT_X = TX + 0.090
GRIP_DZ = 0.004 + TIP - BODY[2] / 2  # TCP over the tool's origin: pad tips 4 mm over its underside
CARRY_DZ = STOP_Z - 0.004 - (TOP + BODY[2])  # tool lifted so its back is 4 mm under the hook bar
HANG_DZ = STOP_Z + PARK_Q + BAR[2] - LOOP_TOP - TZ  # tool origin hanging at park, over its start height
SLIDE = 0.090
SAG = 0.015  # the arms sag this much carrying the tool between them (measured in sim)


def _swivel_tcp(q):
    """Closed fingertips on top of the swivel."""
    return (HX, HY, STOP_Z + q + SHANK_H + 0.016 + TIP)


def _dur(plan):
    return sum(w.duration for w in plan)


def _wait(plan, until, label="wait"):
    t = _dur(plan)
    if until > t:
        last = plan[-1]
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))
    return plan


def _carry(x, dx=0.0, final="hang", park=PARK_L):
    """One arm's part: pinch the tool at x (tool frame), lift, carry onto the hook, lower and let go."""
    y0, z0 = TY0, TZ + GRIP_DZ
    x0 = TX + x
    lift = (CARRY_DZ if final != "bench" else 0.060) + SAG
    high = TOP + (0.160 if park[1] > 0 else 0.090)  # the left arm passes over the hook
    plan = [Waypoint(1.5, (x0, (y0 + park[1]) / 2, high), OPEN_TOOL, "up over the hook", 0.0),
            Waypoint(2.0, (x0, y0, z0 + 0.050), OPEN_TOOL, "over the tool", 0.0),
            Waypoint(1.5, (x0, y0, z0), OPEN_TOOL, "down round the tool", 0.0),
            Waypoint(0.8, (x0, y0, z0), FINGER_CLOSED, "pinch the tool", 0.0),
            Waypoint(2.0, (x0, y0, z0 + lift), FINGER_CLOSED, "lift it", 0.0)]
    if dx:
        plan.append(Waypoint(2.0, (x0 + dx, y0, z0 + lift), FINGER_CLOSED, "shift it along", 0.0))
    x1, y1 = x0 + dx, y0 + SLIDE
    plan.append(Waypoint(3.0, (x1, y1, z0 + lift), FINGER_CLOSED, "carry it across onto the hook", 0.0))
    z1 = z0 + (HANG_DZ if final == "hang" else 0.0)
    plan += [Waypoint(3.0, (x1, y1, z1), FINGER_CLOSED, "lower it", 0.0),
             Waypoint(1.0, (x1, y1, z1), FINGER_CLOSED, "settle", 0.0),
             Waypoint(1.5, (x1, y1, z1), FINGER_OPEN, "let go", 0.0),
             Waypoint(3.0, (x1, y1, z1 + 0.060), FINGER_OPEN, "up", 0.0),
             Waypoint(1.5, (x1 - 0.080 if park[1] > 0 else x1, y1 if park[1] > 0 else y1 - 0.080, z1 + 0.060),
                      FINGER_OPEN, "back away from the hook", 0.0),
             Waypoint(1.5, park, FINGER_OPEN, "clear", 0.0),
             Waypoint(4.0, park, FINGER_OPEN, "wait", 0.0)]
    return plan


def demo():
    # DEMO_VARIANT selects a degenerate check (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    if v == "on_tool":  # carry the tool under the hook and set it on the bench, then pull the hook down onto it
        left = _carry(REAR_X - TX, final="bench")
        right = _carry(FRONT_X - TX, final="bench", park=PARK_R)
        rest = TOP + BODY[2] + LOOP_IN_H + WIRE + 0.001 - STOP_Z  # bar on the bail's top
        x, y, z = _swivel_tcp(0.0)
        left += [Waypoint(2.0, (x, y, z + 0.030), FINGER_CLOSED, "over the hook's swivel", 0.0),
                 Waypoint(1.5, (x, y, z), FINGER_CLOSED, "fingertips on the swivel", 0.0),
                 Waypoint(2.0, _swivel_tcp(rest), FINGER_CLOSED, "push the hook down onto the tool", 0.0),
                 Waypoint(8.0, _swivel_tcp(rest), FINGER_CLOSED, "hold it there", 0.0)]
        return {"left": left, "right": right}
    if v == "guard":  # the trigger guard, not the bail, lined up with the hook
        return {"left": _carry(REAR_X - TX, dx=-GUARD_X),
                "right": _carry(FRONT_X - TX, dx=-GUARD_X, park=PARK_R)}
    return {"left": _carry(REAR_X - TX), "right": _carry(FRONT_X - TX, park=PARK_R)}


# -- offline self-test -------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    flat = mk.mat_to_quat(np.eye(3))
    bail_y = HY - 0.024
    on_hook = np.array([TX, bail_y, TZ])

    def st(q, tool_pos=None, quat=flat):
        moved = {} if tool_pos is None else {"tool": (np.asarray(tool_pos, float), quat)}
        return kit.fake_states(q={"cable": q}, moved=moved)

    # the bail's inner top resting on the bar with the hook at q
    def hang(q):
        return np.array([TX, bail_y, STOP_Z + q + BAR[2] - LOOP_TOP])

    park = hang(PARK_Q)
    guard = hang(PARK_Q) + np.array([-GUARD_X, 0, 0])
    grip = [np.array(_swivel_tcp(PARK_Q))]
    tool_grip = [np.array([REAR_X, bail_y, park[2] + GRIP_DZ])]
    cases = {  # name: (state, tcps, MEMORY, score)
        "start": (st(0.0), [], {}, 0.0),
        "hook pulled down, tool not moved": (st(-0.040), grip, {}, 0.0),
        "bail carried onto the hook, held under it": (st(0.0, on_hook + np.array([0, 0, CARRY_DZ])), tool_grip, {}, 0.4),
        "hanging at park, hook still held": (st(PARK_Q, park), grip, {}, 0.7),
        "hanging at park, let go": (st(PARK_Q, park), [], {}, 1.0),
        "hanging 8 mm under park, let go": (st(PARK_Q - 0.008, hang(PARK_Q - 0.008)), [], {}, 0.7),
        "hanging at the stop, let go": (st(0.0, hang(0.0)), [], {}, 0.7),
        "hanging at park, tool held up by the other hand": (st(PARK_Q, park + np.array([0, 0, 0.006])), tool_grip, {}, 0.4),
        "hook in the trigger guard at park": (st(PARK_Q, guard), [], {}, 0.0),
        "hook laid on top of the tool": (st(TOP + BODY[2] + LOOP_IN_H + WIRE + 0.001 - STOP_Z, on_hook), grip, {}, 0.0),
        "hook whipped": (st(PARK_Q, park), [], {"whip": True}, 0.0),
        "tool dropped": (st(PARK_Q, park), [], {"dropped": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        # geometry: tool on the bench clear of the hook; the hook's tip clears the bail's inner top when low
        t = kit.fake_states()["tool"]
        assert abs(float(t.low[2]) - TOP) < 1e-6, t.low
        assert STOP_Z + BAR[2] + TIP_H < TOP + BODY[2] + CARRY_DZ + LOOP_IN_H - 0.006
        assert float(t.low[2]) + HANG_DZ > TOP + 0.015
        k, t, _ = _spring(PARK_Q + 0.003, True)
        assert abs(k * (t - PARK_Q - 0.003) - (SPRING_N - DETENT_K * 0.003)) < 1e-9
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
    for variant in ("", "on_tool", "guard"):
        os.environ["DEMO_VARIANT"] = variant
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
            print(f"{variant or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm")
