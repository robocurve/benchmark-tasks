"""On the station table, in front of the left arm, a pipe flange lies flat, fixed to the table: a steel disc 22 cm
across with six studs standing up from it on a 15 cm bolt circle. Stud 1 is painted orange, the other five are bare
steel; the studs are numbered clockwise seen from above, starting at stud 1, which is the one furthest from the arms.
Six identical hex nuts (plain steel discs here) stand in a row on a black parts mat further left. A flange is bolted up in the
cross (star) pattern so the gasket is pressed evenly: stud 1, the stud across from it, the next stud clockwise from
stud 1, the one across from that, and so on, so the order for six studs is 1-4-2-5-3-6. A nut is started on each stud
by setting it on the stud's end. The order is checked from the moment each stud first gets its nut.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/flange-caps-star-order/flange_caps_star_order.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
FLANGE_XY = (0.300, 0.200)
FLANGE_R = 0.110
FLANGE_H = 0.020
FLANGE_TOP = TOP + FLANGE_H
BOLT_R = 0.075  # bolt circle radius: neighbouring studs 7.5 cm apart
STUD_R = 0.012  # M24 studs
STUD_H = 0.035  # standing 3.5 cm proud of the flange face
STUD_TOP = FLANGE_TOP + STUD_H
N = 6
# Stud k (1..6) at angle -(k-1)*60 degrees from +x: clockwise seen from above (z up), stud 1 furthest from the arms.
STUD_ANGLE = {k: -math.radians(60.0 * (k - 1)) for k in range(1, N + 1)}
STUD_XY = {k: (FLANGE_XY[0] + BOLT_R * math.cos(a), FLANGE_XY[1] + BOLT_R * math.sin(a)) for k, a in STUD_ANGLE.items()}
STAR = (1, 4, 2, 5, 3, 6)  # each stud, then the one across; step clockwise from stud 1

NUT_R = 0.018  # M24 hex nut, 36 mm across flats, modelled as a disc
NUT_H = 0.020
NUT_KG = 0.100
NUT_START = [(0.150 + 0.050 * k, 0.420) for k in range(N)]  # a row along x, 5 cm apart, on the parts mat
MAT = (0.320, 0.070, 0.003)  # a fixed rubber parts mat under the row of nuts
MAT_XY = (0.275, 0.420)
MAT_TOP = TOP + MAT[2]
ON_STUD_XY = 0.008  # a nut is on a stud when its centre is within 8 mm of the stud's axis

STEEL = (0.62, 0.63, 0.66)
FLANGE_GREY = (0.45, 0.46, 0.48)
ORANGE = (0.92, 0.48, 0.10)
NUT_STEEL = (0.78, 0.78, 0.76)

NUTS = [Cylinder(f"nut_{k + 1}", pos=(x, y, MAT_TOP + NUT_H / 2), radius=NUT_R, height=NUT_H, mass=NUT_KG, color=NUT_STEEL)
        for k, (x, y) in enumerate(NUT_START)]
FIXTURES = [Cylinder("flange", pos=(*FLANGE_XY, TOP + FLANGE_H / 2), radius=FLANGE_R, height=FLANGE_H, color=FLANGE_GREY)]
FIXTURES += [Box("parts_mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=(0.10, 0.10, 0.10))]
FIXTURES += [Cylinder(f"stud_{k}", pos=(*STUD_XY[k], FLANGE_TOP + STUD_H / 2), radius=STUD_R, height=STUD_H,
                      color=ORANGE if k == 1 else STEEL) for k in range(1, N + 1)]
OBJECTS = [*NUTS, *FIXTURES]
NUT_NAMES = [n.name for n in NUTS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _on(st, k) -> bool:
    """The nut at rest on stud k's end: centre within 8 mm of its axis, bottom on the stud top."""
    sx, sy = STUD_XY[k]
    return bool(
        math.hypot(float(st.pos[0]) - sx, float(st.pos[1]) - sy) <= ON_STUD_XY
        and STUD_TOP - 0.002 < float(st.low[2]) < STUD_TOP + 0.004
        and st.resting()
    )


def nutted(objects) -> dict:
    """For each stud, whether some nut sits on it."""
    return {k: any(_on(objects[n], k) for n in NUT_NAMES) for k in STUD_XY}


# The trial's history: the studs in the order they first got a nut. The task file only sees object states, so the
# history resets whenever every nut is back on its start mark (each new trial).
_HISTORY = {"order": []}


def _at_start(objects) -> bool:
    return all(math.hypot(float(objects[n].pos[0]) - x, float(objects[n].pos[1]) - y) < 0.005
               and float(objects[n].low[2]) < MAT_TOP + 0.003 for n, (x, y) in zip(NUT_NAMES, NUT_START))


def _update(objects) -> dict:
    if _at_start(objects):
        _HISTORY["order"] = []
    now = nutted(objects)
    for k in sorted(now):
        if now[k] and k not in _HISTORY["order"]:
            _HISTORY["order"].append(k)
    return now


def in_sequence(objects) -> int:
    """How many studs, from the first, got their nut in the star order and still have it. A stud nutted out of turn
    ends the count there, even if the rest follow in order."""
    now = _update(objects)
    order = _HISTORY["order"]
    k = 0
    while k < min(len(order), N) and order[k] == STAR[k] and now[STAR[k]]:
        k += 1
    return k


def order() -> list:
    """The studs in the order this trial first nutted them (for the log)."""
    return list(_HISTORY["order"])


def success(objects) -> bool:
    """All six studs nutted in the order 1-4-2-5-3-6; holds for 1 s."""
    return in_sequence(objects) == N


def score(objects) -> float:
    """1/6 for each stud nutted in turn, counted from the first: the count stops at the first stud that got its nut out
    of the star order (or lost it). Possible scores: 0, 1/6, ..., 1."""
    return in_sequence(objects) / N


def distance(objects) -> float:
    return float(N - in_sequence(objects))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.130
SQUEEZE = 0.008
DEPTH = 0.012  # pad tips 8 mm above the nut's bottom
OPEN = -(2 * NUT_R + 0.024) / 2
GRIP = -(2 * NUT_R - SQUEEZE) / 2
X_TRIM = 0.004  # a carried part trails the commanded x by about 5 mm on the way down (measured in sim)


def _radial_yaw(k) -> float:
    """The grip yaw that closes the fingers along the radius through stud k, so the pads' 38 mm width runs along the
    bolt circle's tangent, away from the neighbouring studs; folded into [-90, 90] degrees (the grip is symmetric)."""
    yaw = STUD_ANGLE[k] - math.pi / 2
    while yaw > math.pi / 2 + 1e-9:
        yaw -= math.pi
    while yaw < -math.pi / 2 - 1e-9:
        yaw += math.pi
    return yaw


def _move(pick, stud, label):
    (px, py), (qx, qy) = pick, STUD_XY[stud]
    qx += X_TRIM
    gz = MAT_TOP + NUT_H - DEPTH + TIP
    qz = STUD_TOP + 0.002 + NUT_H - DEPTH + TIP  # let go 2 mm above the stud top
    yaw = _radial_yaw(stud)
    return [
        Waypoint(1.8, (px, py, TRAVEL), OPEN, f"{label}: over", 0.0),
        Waypoint(1.5, (px, py, gz + 0.020), OPEN, f"{label}: descend", 0.0),
        Waypoint(1.0, (px, py, gz), OPEN, f"{label}: down slowly", 0.0),
        Waypoint(0.5, (px, py, gz), OPEN, f"{label}: settle", 0.0),
        Waypoint(1.5, (px, py, gz), GRIP, f"{label}: close", 0.0),
        Waypoint(1.5, (px, py, TRAVEL), GRIP, f"{label}: lift", 0.0),
        Waypoint(2.0, (qx, qy, TRAVEL), GRIP, f"{label}: carry", yaw),
        Waypoint(0.6, (qx, qy, TRAVEL), GRIP, f"{label}: settle over", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.020), GRIP, f"{label}: lower", yaw),
        Waypoint(1.5, (qx, qy, qz), GRIP, f"{label}: onto the stud slowly", yaw),
        Waypoint(0.6, (qx, qy, qz), GRIP, f"{label}: settle", yaw),
        Waypoint(1.5, (qx, qy, qz), OPEN, f"{label}: release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL), OPEN, f"{label}: retreat", yaw),
    ]


def demo():
    """Left arm only: the nuts from the row (fingers across each along y, clear of the neighbours 5 cm away), one onto
    each stud in the star order 1-4-2-5-3-6, the fingers turned to close along the stud's radius. 99.0 s."""
    plan = []
    for nut, stud in zip(range(N), STAR):
        plan += _move(NUT_START[nut], stud, f"nut {nut + 1} to stud {stud}")
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the order history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def nut(x, y, bottom, vel=(0.0, 0.0, 0.0)):
        half = np.array([NUT_R, NUT_R, NUT_H / 2])
        pos = np.array([x, y, bottom + NUT_H / 2])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: nut(x, y, MAT_TOP) for n, (x, y) in zip(NUT_NAMES, NUT_START)}

    def placing(*studs, base=None):
        """Frames: from ``base`` (the start), put nut_1, nut_2, ... on ``studs`` one at a time."""
        frames, f = [], dict(base or start)
        for i, k in enumerate(studs):
            f = f | {NUT_NAMES[i]: nut(*STUD_XY[k], STUD_TOP)}
            frames.append(f)
        return frames

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    full = placing(*STAR)
    cases = {
        "start": ((start,), 0.0, False),
        "stud 1 only": ((start, *placing(1)), 1 / 6, False),
        "star order, all six": ((start, *full), 1.0, True),
        "star order, first three": ((start, *placing(1, 4, 2)), 0.5, False),
        "round the circle 1-2-3-4-5-6": ((start, *placing(1, 2, 3, 4, 5, 6)), 1 / 6, False),
        "anticlockwise star 1-4-6-3-5-2": ((start, *placing(1, 4, 6, 3, 5, 2)), 2 / 6, False),
        "started on the wrong stud": ((start, *placing(4, 1, 2, 5, 3, 6)), 0.0, False),
        "star order but one nut fell off": ((start, *full, full[-1] | {NUT_NAMES[2]: nut(0.30, 0.31, TOP)}), 2 / 6, False),
        "nut 9 mm off the stud axis": ((start, start | {NUT_NAMES[0]: nut(STUD_XY[1][0] - 0.009, STUD_XY[1][1], STUD_TOP)}), 0.0,
                                       False),
        "nut still sliding on the stud": ((start, start | {NUT_NAMES[0]: nut(*STUD_XY[1], STUD_TOP, vel=(0.02, 0, 0))}), 0.0,
                                          False),
        "nut on the flange face, not a stud": ((start, start | {NUT_NAMES[0]: nut(*FLANGE_XY, FLANGE_TOP)}), 0.0, False),
        "a new trial after a wrong order starts clean": ((start, *placing(2), start, *full), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
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
