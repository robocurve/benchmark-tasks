"""On the station table, in front of the left arm, a DIN rail lies flat, running left to right, with three grey I/O
modules packed against its fixed left end clamp and a fixed end clamp near its right end. A white DIN-rail power
supply stands on the table nearer the arms. Its datasheet asks for 20 to 30 mm of free air beside it for convection
cooling, so it has to go on the rail to the right of the modules with a gap of that size: not touching them, and not
so far away that it blocks the next slot. Scored from the power supply's pose at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/power-supply-air-gap/power_supply_air_gap.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
RAIL_X = 0.300
RAIL = (0.035, 0.300, 0.0075)
RAIL_Y = 0.270
RAIL_TOP = TOP + RAIL[2]
MOD = (0.080, 0.045, 0.060)
CLAMP = (0.040, 0.010, 0.030)
LEFT_CLAMP_Y = RAIL_Y + RAIL[1] / 2 - 0.010 - CLAMP[1] / 2
MOD_Y = [LEFT_CLAMP_Y - CLAMP[1] / 2 - MOD[1] / 2 - k * MOD[1] for k in range(3)]  # packed, no gaps
MODULES_EDGE = MOD_Y[-1] - MOD[1] / 2  # the last module's right face
RIGHT_CLAMP_Y = RAIL_Y - RAIL[1] / 2 + 0.010 + CLAMP[1] / 2

PSU = (0.080, 0.060, 0.070)  # 80 mm across the rail, 60 mm wide, 70 mm tall
PSU_KG = 0.400
PSU_START = (0.160, 0.250)
GAP_MIN, GAP_MAX = 0.020, 0.030

GREY = (0.62, 0.63, 0.65)
WHITE = (0.93, 0.93, 0.92)
DARK = (0.12, 0.12, 0.13)
STEEL = (0.75, 0.76, 0.78)

SUPPLY = Box("power_supply", pos=(*PSU_START, TOP + PSU[2] / 2), size=PSU, mass=PSU_KG, color=WHITE)
FIXTURES = [
    Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("left_clamp", pos=(RAIL_X, LEFT_CLAMP_Y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, color=DARK),
    Box("right_clamp", pos=(RAIL_X, RIGHT_CLAMP_Y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, color=DARK),
    *(Box(f"module_{k + 1}", pos=(RAIL_X, y, RAIL_TOP + MOD[2] / 2), size=MOD, color=GREY) for k, y in enumerate(MOD_Y)),
]
OBJECTS = [SUPPLY, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
SEAT_X = 0.004
SEAT_Z = 0.002
SEAT_DEG = 4.0


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _seated(state) -> bool:
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[0]) - RAIL_X) <= SEAT_X
        and abs(float(state.low[2]) - RAIL_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= SEAT_DEG
        and state.resting()
    )


def gap(state) -> float:
    """Free air between the last module's right face and the power supply's left face, metres (negative if they
    overlap, which a seated supply cannot), rounded to 0.1 mm."""
    return round(MODULES_EDGE - (float(state.pos[1]) + PSU[1] / 2), 4)


def success(objects) -> bool:
    """Seated on the rail right of the modules with 20 to 30 mm of air; holds for 1 s."""
    st = objects["power_supply"]
    return _seated(st) and GAP_MIN <= gap(st) <= GAP_MAX


def score(objects) -> float:
    """0.25 lifted 3 cm off the table; 0.5 seated on the rail right of the modules (centred across the rail within 4 mm,
    bottom within 2 mm of the rail top, square and upright within 4 degrees, still); 0.75 seated with 10 to 20 mm or 30
    to 40 mm of air; 1.0 seated with 20 to 30 mm. Seated left of the modules is impossible (they fill the rail to the
    left clamp)."""
    st = objects["power_supply"]
    if _seated(st):
        g = gap(st)
        if GAP_MIN <= g <= GAP_MAX:
            return 1.0
        if GAP_MIN - 0.010 <= g <= GAP_MAX + 0.010:
            return 0.75
        return 0.5
    return 0.25 if float(st.low[2]) >= TOP + 0.030 else 0.0


def distance(objects) -> float:
    st = objects["power_supply"]
    target = MODULES_EDGE - (GAP_MIN + GAP_MAX) / 2 - PSU[1] / 2
    return float(math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (RAIL_X, target, RAIL_TOP)))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto the supply's front and back
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
DEPTH = 0.025
X_TRIM = 0.006  # the held supply trails the commanded x by about 6 mm on the way down (measured in sim)


def demo():
    """Left arm only: lift the supply across its 80 mm depth with the fingers fully open, carry it over the rail and set
    it down with 25 mm of air beside the last module. 16.5 s."""
    px, py = PSU_START
    qx, qy = RAIL_X + X_TRIM, MODULES_EDGE - 0.025 - PSU[1] / 2
    gz = TOP + PSU[2] - DEPTH + TIP
    qz = RAIL_TOP + PSU[2] - DEPTH + TIP + 0.0015
    grip = -(PSU[0] - SQUEEZE) / 2
    return {"left": [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, "over the supply", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, "descend", YAW),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, "down slowly", YAW),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, "settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, "close", YAW),
        Waypoint(1.5, (px, py, TRAVEL), grip, "lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, "carry over the rail", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, "settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, "lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, "set on slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, "settle", YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "retreat", YAW),
    ]}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def psu(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])
        half = np.array(PSU) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def at_gap(g, **kw):
        return {"power_supply": psu(RAIL_X, MODULES_EDGE - g - PSU[1] / 2, RAIL_TOP, **kw)}

    cases = {
        "start": ({"power_supply": psu(*PSU_START, TOP)}, 0.0, False),
        "lifted 4 cm": ({"power_supply": psu(0.2, 0.25, TOP + 0.04, vel=(0.05, 0, 0))}, 0.25, False),
        "25 mm": (at_gap(0.025), 1.0, True),
        "20 mm": (at_gap(0.020), 1.0, True),
        "30 mm": (at_gap(0.030), 1.0, True),
        "15 mm": (at_gap(0.015), 0.75, False),
        "35 mm": (at_gap(0.035), 0.75, False),
        "touching the modules": (at_gap(0.0), 0.5, False),
        "45 mm": (at_gap(0.045), 0.5, False),
        "25 mm, turned 180 degrees": (at_gap(0.025, yaw_deg=180), 1.0, True),
        "25 mm, turned 6 degrees": (at_gap(0.025, yaw_deg=6), 0.0, False),
        "25 mm, still sliding": (at_gap(0.025, vel=(0, -0.02, 0)), 0.0, False),
        "on its side on the table": ({"power_supply": psu(0.2, 0.2, TOP, tilt_deg=90)}, 0.0, False),
        "on the rail 6 mm off its centreline": ({"power_supply": psu(RAIL_X + 0.006, MODULES_EDGE - 0.025 - PSU[1] / 2, RAIL_TOP)}, 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
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
