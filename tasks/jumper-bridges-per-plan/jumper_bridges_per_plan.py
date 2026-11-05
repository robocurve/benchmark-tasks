"""On the station table, in front of the left arm, a finished strip of eight grey terminal blocks stands on a DIN rail
lying flat, running left to right, numbered 1 to 8 from the left end clamp. The terminal plan commons two pairs of
neighbouring terminals with plug-in bridges: terminals 2 and 3, and terminals 5 and 6. Three red two-pole jumper bars
lie in a white tray nearer the arms; one is a spare. A bar bridges a pair when it lies along the strip on top of both
blocks, centred over the gap between them. A bar on any other pair is a wiring fault. Scored from the bars' poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/jumper-bridges-per-plan/jumper_bridges_per_plan.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
RAIL_X = 0.380
RAIL = (0.035, 0.200, 0.0075)
RAIL_Y = 0.330
RAIL_TOP = TOP + RAIL[2]
BLOCK = (0.055, 0.0122, 0.050)
PITCH = BLOCK[1] + 0.0005
CLAMP = (0.045, 0.010, 0.035)
LEFT_CLAMP_Y = RAIL_Y + RAIL[1] / 2 - 0.010 - CLAMP[1] / 2
N = 8
TERM_Y = {k: LEFT_CLAMP_Y - CLAMP[1] / 2 - 0.0005 - BLOCK[1] / 2 - (k - 1) * PITCH for k in range(1, N + 1)}
STRIP_TOP = RAIL_TOP + BLOCK[2]
PAIRS = ((2, 3), (5, 6))  # the plan's bridges

BAR = (0.010, 0.023, 0.012)  # two-pole jumper: 1 cm across the strip, 2.3 cm along it, 1.2 cm tall
BAR_KG = 0.005
TRAY = (0.240, 0.300)
TRAY_INNER = (0.060, 0.140)
TRAY_WALL = (0.006, 0.010)
BAR_SPOTS = [(TRAY[0], TRAY[1] + dy) for dy in (0.045, 0.0, -0.045)]  # lying along y, 4.5 cm apart

GREY = (0.62, 0.63, 0.65)
RED = (0.80, 0.12, 0.10)
DARK = (0.12, 0.12, 0.13)
STEEL = (0.75, 0.76, 0.78)
WHITE = (0.92, 0.92, 0.92)


def _tray():
    cx, cy = TRAY
    ix, iy = TRAY_INNER
    t, h = TRAY_WALL
    z = TOP + h / 2
    return [
        Box("tray_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=WHITE),
        Box("tray_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=WHITE),
        Box("tray_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=WHITE),
        Box("tray_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=WHITE),
    ]


BARS = [Box(f"jumper_{k + 1}", pos=(x, y, TOP + BAR[2] / 2), size=BAR, mass=BAR_KG, color=RED)
        for k, (x, y) in enumerate(BAR_SPOTS)]
FIXTURES = [
    Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("left_clamp", pos=(RAIL_X, LEFT_CLAMP_Y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, color=DARK),
    *(Box(f"terminal_{k}", pos=(RAIL_X, y, RAIL_TOP + BLOCK[2] / 2), size=BLOCK, color=GREY) for k, y in TERM_Y.items()),
    Box("right_clamp", pos=(RAIL_X, TERM_Y[N] - BLOCK[1] / 2 - 0.0005 - CLAMP[1] / 2, RAIL_TOP + CLAMP[2] / 2),
        size=CLAMP, color=DARK),
    *_tray(),
]
OBJECTS = [*BARS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
ON_Y = 0.003  # a bar bridges a pair with its centre within 3 mm of the pair's midpoint along the strip
ON_X = 0.004  # and within 4 mm of the strip's centreline
ON_Z = 0.002  # its bottom within 2 mm of the blocks' top
ON_DEG = 8.0  # its long side along the strip


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def bridged(objects):
    """{(a, b): bar names} for every pair of neighbouring terminals that a bar bridges."""
    out = {}
    for bar in BARS:
        st = objects[bar.name]
        yaw, tilt = _yaw_tilt(st.quat_xyzw)
        if not (abs(float(st.pos[0]) - RAIL_X) <= ON_X and abs(float(st.low[2]) - STRIP_TOP) <= ON_Z
                and _square(yaw) <= ON_DEG and tilt <= ON_DEG and st.resting()):
            continue
        for k in range(1, N):
            mid = (TERM_Y[k] + TERM_Y[k + 1]) / 2
            if abs(float(st.pos[1]) - mid) <= ON_Y:
                out.setdefault((k, k + 1), []).append(bar.name)
    return out


def _on_strip_elsewhere(objects, b) -> int:
    """Bars lying on top of the strip (bottom on the blocks' top, over the strip) that bridge no plan pair."""
    n = 0
    plan = {name for p in PAIRS for name in b.get(p, [])}
    for bar in BARS:
        st = objects[bar.name]
        over = abs(float(st.pos[0]) - RAIL_X) <= BLOCK[0] / 2 and TERM_Y[N] - PITCH / 2 <= float(st.pos[1]) <= TERM_Y[1] + PITCH / 2
        if over and abs(float(st.low[2]) - STRIP_TOP) <= ON_Z and st.resting() and bar.name not in plan:
            n += 1
    return n


def success(objects) -> bool:
    """Terminals 2-3 and 5-6 bridged, and no bar on any other part of the strip; holds for 1 s."""
    b = bridged(objects)
    return all(p in b for p in PAIRS) and _on_strip_elsewhere(objects, b) == 0


def score(objects) -> float:
    """0.5 for each plan pair bridged; 0.5 off for each bar lying on top of the strip anywhere else (a bridge between
    the wrong terminals, or a bar left across one); floored at 0."""
    b = bridged(objects)
    return max(0.0, 0.5 * sum(p in b for p in PAIRS) - 0.5 * _on_strip_elsewhere(objects, b))


def distance(objects) -> float:
    b = bridged(objects)
    return float(sum(p not in b for p in PAIRS) + _on_strip_elsewhere(objects, b))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, across the bar's 10 mm width
TIP = 0.013
TRAVEL = TOP + 0.130
SQUEEZE = 0.006
OPEN = -(BAR[0] + 0.030) / 2
DEPTH = BAR[2] - 0.003  # the pad tips 3 mm above the bar's bottom


def _move(pick, place_y, label):
    (px, py) = pick
    gz = TOP + BAR[2] - DEPTH + TIP
    qz = STRIP_TOP + BAR[2] - DEPTH + TIP + 0.0015
    grip = -(BAR[0] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), OPEN, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), OPEN, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), OPEN, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), OPEN, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (RAIL_X, place_y, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (RAIL_X, place_y, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (RAIL_X, place_y, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (RAIL_X, place_y, qz), grip, f"{label}: set on slowly", YAW),
        Waypoint(0.6, (RAIL_X, place_y, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (RAIL_X, place_y, qz), OPEN, f"{label}: release", YAW),
        Waypoint(1.0, (RAIL_X, place_y, TRAVEL), OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: one bar onto terminals 2-3, another onto 5-6, each set down along the strip centred over the gap;
    the third bar stays in the tray. 33.0 s."""
    moves = []
    for k, (a, b) in enumerate(PAIRS):
        moves += _move(BAR_SPOTS[k], (TERM_Y[a] + TERM_Y[b]) / 2, f"bridge {a}-{b}")
    return {"left": moves}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def bar(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])
        half = np.array(BAR) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {b.name: bar(b.pos[0], b.pos[1], TOP) for b in BARS}

    def on(a, **kw):
        return bar(RAIL_X, (TERM_Y[a] + TERM_Y[a + 1]) / 2, STRIP_TOP, **kw)

    cases = {
        "start": (start, 0.0, False),
        "done": (start | {"jumper_1": on(2), "jumper_2": on(5)}, 1.0, True),
        "done, a bar turned 180 degrees": (start | {"jumper_1": on(2, yaw_deg=180), "jumper_2": on(5)}, 1.0, True),
        "one pair bridged": (start | {"jumper_1": on(2)}, 0.5, False),
        "wrong pair 3-4 instead of 2-3": (start | {"jumper_1": on(3), "jumper_2": on(5)}, 0.0, False),
        "both pairs plus the spare on 7-8": (start | {"jumper_1": on(2), "jumper_2": on(5), "jumper_3": on(7)}, 0.5, False),
        "spare bar left across terminal 1 only": (
            start | {"jumper_1": on(2), "jumper_2": on(5), "jumper_3": bar(RAIL_X, TERM_Y[1], STRIP_TOP)}, 0.5, False),
        "bar 5 mm off the gap": (start | {"jumper_1": on(2), "jumper_2": bar(RAIL_X, (TERM_Y[5] + TERM_Y[6]) / 2 + 0.005, STRIP_TOP)}, 0.0, False),
        "bar turned across the strip": (start | {"jumper_1": on(2), "jumper_2": on(5, yaw_deg=90)}, 0.0, False),
        "bar still sliding": (start | {"jumper_1": on(2), "jumper_2": on(5, vel=(0.0, 0.02, 0.0))}, 0.5, False),
        "bar on its side over the gap": (start | {"jumper_1": on(2), "jumper_2": on(5, tilt_deg=90)}, 0.0, False),
        "bar on the table by the strip": (start | {"jumper_1": on(2), "jumper_2": bar(0.30, 0.25, TOP)}, 0.5, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success, bridged(st))
    return len(cases)


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
