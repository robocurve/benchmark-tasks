"""On the station table, in front of the left arm, a rack power shelf lies on its back so its bays open upward: six PSU
bays in a row along y between fixed walls, on a fixed base, with an orange end plate at the bay-1 end. PSUs already
stand in bays 2 and 5. Four identical spare PSUs stand on a spares mat nearer the arms. The instruction gives the
rack's load (8.4 kW), each PSU's rating (3 kW) and the redundancy rule (N+1), so the shelf needs ceil(8.4 / 3) + 1 = 4
PSUs: exactly two spares must be added, in any empty bays. Scored from PSU poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/psu-shelf-n-plus-one/psu_shelf_n_plus_one.py \\
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
SH_X, SH_Y = 0.300, 0.250  # shelf centre; bays run along y
BAYS = 6
BASE_T = 0.010
FLOOR = TOP + BASE_T  # bay floor
PSU = (0.080, 0.032, 0.050)  # PSU: 80 mm long (x), 32 mm wide (y), 50 mm tall (z)
PSU_KG = 0.250
GAP_W = 0.038  # clear width of a bay: 3 mm each side of a PSU
WALL = (0.050, 0.006, 0.035)  # wall between bays: 50 mm long, 6 mm thick, 35 mm tall
PITCH = GAP_W + WALL[1]
END = (0.060, 0.010, 0.050)  # end plates
BAY_Y = {k: SH_Y + (2 * k - BAYS - 1) * PITCH / 2 for k in range(1, BAYS + 1)}  # bay 1 at the orange end (low y)
END_Y = {"orange": BAY_Y[1] - GAP_W / 2 - END[1] / 2, "grey": BAY_Y[BAYS] + GAP_W / 2 + END[1] / 2}
BASE = (0.100, END_Y["grey"] - END_Y["orange"] + END[1], BASE_T)

LOAD_KW, PSU_KW = 8.4, 3.0
NEED_TOTAL = math.ceil(LOAD_KW / PSU_KW) + 1  # N+1 = 4
INSTALLED = (2, 5)  # bays already populated
NEED_ADD = NEED_TOTAL - len(INSTALLED)  # 2

MAT_X = 0.180
MAT = (0.110, 0.260, 0.002)
MAT_Y = 0.250
SPARE_Y = {"spare_1": 0.160, "spare_2": 0.220, "spare_3": 0.280, "spare_4": 0.340}

DARK = (0.16, 0.17, 0.19)
STEEL = (0.62, 0.63, 0.66)
ORANGE = (0.93, 0.45, 0.08)
PSU_C = (0.86, 0.86, 0.83)
MATC = (0.30, 0.32, 0.36)


def _psu(name, x, y, bottom):
    return Box(name, pos=(x, y, bottom + PSU[2] / 2), size=PSU, mass=PSU_KG, color=PSU_C)


INSTALLED_NAMES = [f"psu_bay_{k}" for k in INSTALLED]
SPARES = list(SPARE_Y)
PSUS = [
    *(_psu(f"psu_bay_{k}", SH_X, BAY_Y[k], FLOOR) for k in INSTALLED),
    *(_psu(n, MAT_X, y, TOP + MAT[2]) for n, y in SPARE_Y.items()),
]
FIXTURES = [
    Box("shelf_base", pos=(SH_X, SH_Y, TOP + BASE_T / 2), size=BASE, color=DARK),
    *(Box(f"bay_wall_{k}", pos=(SH_X, (BAY_Y[k] + BAY_Y[k + 1]) / 2, FLOOR + WALL[2] / 2), size=WALL, color=STEEL)
      for k in range(1, BAYS)),
    Box("end_plate_orange", pos=(SH_X, END_Y["orange"], FLOOR + END[2] / 2), size=END, color=ORANGE),
    Box("end_plate_grey", pos=(SH_X, END_Y["grey"], FLOOR + END[2] / 2), size=END, color=STEEL),
    Box("spares_mat", pos=(MAT_X, MAT_Y, TOP + MAT[2] / 2), size=MAT, color=MATC),
]
OBJECTS = [*PSUS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

IN_X = 0.020  # in a bay: centre within 2 cm of the shelf centreline across the bays,
IN_Z = 0.010  # inside the bay's clear width along y, bottom within 1 cm of the bay floor
SEAT_X = 0.010  # seated: centre within 1 cm across,
SEAT_Y = 0.004  # within 4 mm of the bay centre along y,
SEAT_Z = 0.003  # bottom within 3 mm of the floor,
SEAT_DEG = 8.0  # upright and square within 8 degrees, still


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _bay_of(st):
    """The bay the PSU is in (centre inside a bay's width, near the centreline, bottom near the floor), or None."""
    if abs(float(st.pos[0]) - SH_X) >= IN_X or float(st.low[2]) >= FLOOR + IN_Z:
        return None
    for k, y in BAY_Y.items():
        if abs(float(st.pos[1]) - y) < GAP_W / 2:
            return k
    return None


def _seated(st, k) -> bool:
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        _bay_of(st) == k
        and abs(float(st.pos[1]) - BAY_Y[k]) <= SEAT_Y
        and abs(float(st.pos[0]) - SH_X) <= SEAT_X
        and abs(float(st.low[2]) - FLOOR) <= SEAT_Z
        and tilt <= SEAT_DEG
        and _square(yaw) <= SEAT_DEG
        and st.resting()
    )


def score(objects) -> float:
    """0.5 for each spare PSU seated in a bay, up to the two that N+1 needs. Capped at 0.25 if more than two spares
    end up in bays (over-populated: N+2 or more) or if either installed PSU is no longer seated in bay 2 or 5."""
    in_bays = [n for n in SPARES if _bay_of(objects[n]) is not None]
    seated = [n for n in in_bays if _seated(objects[n], _bay_of(objects[n]))]
    cap = 1.0
    if not all(_seated(objects[f"psu_bay_{k}"], k) for k in INSTALLED):
        cap = 0.25
    if len(in_bays) > NEED_ADD:
        cap = 0.25
    return float(min(cap, 0.5 * min(len(seated), NEED_ADD)))


def success(objects) -> bool:
    """Exactly two spares seated in empty bays (four PSUs in the shelf), installed ones still seated; holds 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres to go: the two spares nearest the shelf to the nearest empty bays, each to its own bay."""
    empty = [k for k in BAY_Y if k not in INSTALLED]

    def d(n, k):
        st = objects[n]
        return math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (SH_X, BAY_Y[k], FLOOR))

    return float(min(d(a, i) + d(b, j) for a in SPARES for b in SPARES if a != b for i in empty for j in empty if i != j))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = -math.pi / 2  # the fingers close along x, onto the PSU's two end faces; the bay walls sit between the pads
TIP = 0.013
TRAVEL = TOP + 0.160
SQUEEZE = 0.008
GRIP_DEPTH = 0.025  # pad tips 25 mm below the PSU's top
X_TRIM = 0.0  # none needed: in sim the 250 g PSUs land ~5 mm short in x (seat tolerance 1 cm)


def _grip_z(bottom):
    return bottom + PSU[2] - GRIP_DEPTH + TIP


def _move(pick, place, label):
    """Top-down pick and place across the PSU's 80 mm length with the fingers fully open (94 mm) to clear it,
    settling before every close and release, and lowering the last 4 cm into the bay slowly. 18.2 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += X_TRIM
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.0015  # set down 1.5 mm above the floor
    grip = -(PSU[0] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.040), grip, f"{label}: lower over the bay", YAW),
        Waypoint(2.0, (qx, qy, qz), grip, f"{label}: down into the bay slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: two spares into the empty bays 3 and 4; the other two spares stay on the mat. 36.4 s."""
    bottom = TOP + MAT[2]
    plan = []
    for name, k in (("spare_2", 3), ("spare_3", 4)):
        plan += _move((MAT_X, SPARE_Y[name], bottom), (SH_X, BAY_Y[k], FLOOR), f"{name} to bay {k}")
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(PSU) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: box(p.pos[0], p.pos[1], p.pos[2] - PSU[2] / 2) for p in PSUS}

    def at(name, k, dy=0.0, **kw):
        return {name: box(SH_X, BAY_Y[k] + dy, FLOOR, **kw)}

    def scene(*parts):
        out = dict(start)
        for p in parts:
            out |= p
        return out

    cases = {
        "start": (scene(), 0.0),
        "done, bays 3 and 4": (scene(at("spare_2", 3), at("spare_3", 4)), 1.0),
        "done, bays 1 and 6 with other spares": (scene(at("spare_1", 1), at("spare_4", 6)), 1.0),
        "one spare added (N, no redundancy)": (scene(at("spare_1", 3)), 0.5),
        "three spares added (N+2)": (scene(at("spare_1", 1), at("spare_2", 3), at("spare_3", 4)), 0.25),
        "every bay filled": (scene(at("spare_1", 1), at("spare_2", 3), at("spare_3", 4), at("spare_4", 6)), 0.25),
        "two seated, a third leaning in bay 6": (scene(at("spare_2", 3), at("spare_3", 4), at("spare_4", 6, tilt_deg=15)), 0.25),
        "one seated, one leaning": (scene(at("spare_2", 3), at("spare_3", 4, tilt_deg=12)), 0.5),
        "one seated, one 6 mm off centre": (scene(at("spare_2", 3), at("spare_3", 4, dy=0.006)), 0.5),
        "one seated, one still moving": (scene(at("spare_2", 3), at("spare_3", 4, vel=(0, 0, -0.05))), 0.5),
        "one seated, one held 3 cm above its bay": (scene(at("spare_2", 3), {"spare_3": box(SH_X, BAY_Y[4], FLOOR + 0.03)}), 0.5),
        "done but installed PSU in bay 5 pulled": (scene(at("spare_2", 3), at("spare_3", 4), {"psu_bay_5": box(0.20, 0.10, TOP)}), 0.25),
        "installed PSU moved to bay 6, spares in 3 and 4": (scene(at("spare_2", 3), at("spare_3", 4), at("psu_bay_5", 6)), 0.25),
        "done, one turned 180 degrees": (scene(at("spare_2", 3, yaw_deg=180), at("spare_3", 4)), 1.0),
    }
    for name, (st, want) in cases.items():
        got = score(st)
        assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
    assert NEED_TOTAL == 4 and NEED_ADD == 2
    assert distance(scene(at("spare_2", 3), at("spare_3", 4))) < 1e-9
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
