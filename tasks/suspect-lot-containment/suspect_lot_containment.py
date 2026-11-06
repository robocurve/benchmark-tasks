"""On the station table, in front of the left arm, the outfeed rack of a machining cell: two low steel rails running
forward from the arms to the machine (a grey block at the far end). The machine pushes each finished part onto the
rack, so the five aluminium blocks on it stand in the order they were made: part 1, the oldest, nearest the arms,
part 5, the newest, against the machine. Two fixed tags lie beside the rails: a white tag at part 2, the last part
that passed the operator's in-process check, and a red tag at part 5, where the next check found a defect. Nearer
the middle of the table is a red hold bin. Containment rule: every part made after the last good check, up to the
defective one, is suspect and goes on hold; the parts at or before the last good check stay on the rack. Scored from
the part poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/suspect-lot-containment/suspect_lot_containment.py \\
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
PART = (0.030, 0.030, 0.040)  # machined aluminium block: 30 x 30 mm, 40 mm tall
PART_KG = 0.090
N_PARTS = 5
RACK_Y = 0.330
SLOT_X = {k: 0.170 + 0.060 * (k - 1) for k in range(1, N_PARTS + 1)}  # part 1 (oldest) nearest the arms
LAST_GOOD = 2  # white tag: the last part that passed the in-process check
DEFECT = 5  # red tag: where the defect was found
SUSPECT = tuple(range(LAST_GOOD + 1, DEFECT + 1))  # 3, 4, 5
KEEP = tuple(range(1, LAST_GOOD + 1))  # 1, 2

RAIL = (0.300, 0.006, 0.010)
RAIL_X = 0.290
RAIL_DY = PART[1] / 2 + 0.012 + RAIL[1] / 2  # 12 mm of air each side of a part
MACHINE = (0.060, 0.140, 0.100)
MACHINE_X = SLOT_X[N_PARTS] + PART[0] / 2 + 0.040 + MACHINE[0] / 2
TAG = (0.025, 0.020, 0.003)
TAG_Y = RACK_Y - RAIL_DY - 0.025

HOLD = (0.300, 0.170)  # hold bin centre
HOLD_INNER = (0.160, 0.090)
WALL_T, WALL_H = 0.006, 0.020
HOLD_SPOT = {3: (0.250, 0.170), 4: (0.300, 0.170), 5: (0.350, 0.170)}  # where the demo sets each suspect part

KEEP_MOVE = 0.005  # a kept part counts as undisturbed within 5 mm and 5 degrees of its start

ALU = (0.78, 0.79, 0.81)
STEEL = (0.45, 0.47, 0.50)
GREY = (0.35, 0.37, 0.40)
WHITE = (0.95, 0.95, 0.95)
RED = (0.85, 0.12, 0.10)


def _walls(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


PARTS = [Box(f"part_{k}", pos=(SLOT_X[k], RACK_Y, TOP + PART[2] / 2), size=PART, mass=PART_KG, color=ALU)
         for k in range(1, N_PARTS + 1)]
FIXTURES = [
    Box("rail_left", pos=(RAIL_X, RACK_Y + RAIL_DY, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("rail_right", pos=(RAIL_X, RACK_Y - RAIL_DY, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("machine", pos=(MACHINE_X, RACK_Y, TOP + MACHINE[2] / 2), size=MACHINE, color=GREY),
    Box("check_tag", pos=(SLOT_X[LAST_GOOD], TAG_Y, TOP + TAG[2] / 2), size=TAG, color=WHITE),
    Box("defect_tag", pos=(SLOT_X[DEFECT], TAG_Y, TOP + TAG[2] / 2), size=TAG, color=RED),
    *_walls("hold_bin", HOLD, HOLD_INNER, RED),
]
OBJECTS = [*PARTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _in_hold(st) -> bool:
    """At rest, the centre inside the hold bin's inner walls and the lowest point within 2 cm of the table."""
    return bool(
        abs(float(st.pos[0]) - HOLD[0]) < HOLD_INNER[0] / 2
        and abs(float(st.pos[1]) - HOLD[1]) < HOLD_INNER[1] / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.020
        and st.resting()
    )


def _kept(objects, k) -> bool:
    st = objects[f"part_{k}"]
    start = (SLOT_X[k], RACK_Y, TOP + PART[2] / 2)
    return bool(math.dist([float(v) for v in st.pos], start) <= KEEP_MOVE and _tilt(st.quat_xyzw) <= 5.0)


def held(objects) -> int:
    return sum(_in_hold(objects[f"part_{k}"]) for k in SUSPECT)


def score(objects) -> float:
    """0.25 for each suspect part (3, 4, 5) at rest in the hold bin, and the last 0.25 when all three are there and
    parts 1 and 2 are still on the rack where they were (within 5 mm and 5 degrees). Putting a good part on hold or
    moving it off its place caps the score at 0.25."""
    n = held(objects)
    keep_ok = all(_kept(objects, k) for k in KEEP)
    s = 0.25 * n + 0.25 * (n == len(SUSPECT) and keep_ok)
    if not keep_ok:
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Parts 3, 4 and 5 in the hold bin, parts 1 and 2 undisturbed on the rack; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: each suspect part's centre to the hold bin's centre (in the plane)."""
    d = 0.0
    for k in SUSPECT:
        st = objects[f"part_{k}"]
        d += math.hypot(float(st.pos[0]) - HOLD[0], float(st.pos[1]) - HOLD[1])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.120
SQUEEZE = 0.008
CLEAR = 0.020
TIP = 0.013
GRIP_DEPTH = 0.022  # the pad tips 22 mm below the part's top: 18 mm above the table, clear of the 10 mm rails


def _move(pick, place, label):
    """Top-down pick across the block's y faces and place, settling before every close and release. 14.2 s."""
    (px, py), (qx, qy) = pick, place
    gz = TOP + PART[2] - GRIP_DEPTH + TIP
    qz = gz + 0.002  # let go 2 mm above the table
    open_ = -(PART[1] + CLEAR) / 2
    grip = -(PART[1] - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), open_, f"{label}: over"),
        Waypoint(1.5, (px, py, gz), open_, f"{label}: descend"),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{label}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{label}: settle over the bin"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{label}: set down"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat"),
    ]


def demo():
    """Left arm only: parts 3, 4 and 5 off the rack into the hold bin; parts 1 and 2 untouched. 39.6 s."""
    plan = []
    for k in SUSPECT:
        plan += _move((SLOT_X[k], RACK_Y), HOLD_SPOT[k], f"part {k}")
    return {"left": plan}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom=TOP, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(PART) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {f"part_{k}": box(SLOT_X[k], RACK_Y) for k in range(1, N_PARTS + 1)}

    def hold(*ks):
        return {f"part_{k}": box(*HOLD_SPOT.get(k, (0.30, 0.19))) for k in ks}

    cases = {
        "start": (start, 0.0, False),
        "part 5 on hold": (start | hold(5), 0.25, False),
        "parts 4 and 5 on hold": (start | hold(4, 5), 0.5, False),
        "done": (start | hold(3, 4, 5), 1.0, True),
        "done but part 2 on hold too": (start | hold(2, 3, 4, 5), 0.25, False),
        "only the defective part held, then part 2": (start | hold(2, 5), 0.25, False),
        "done but part 1 nudged 8 mm": (start | hold(3, 4, 5) | {"part_1": box(SLOT_X[1] + 0.008, RACK_Y)}, 0.25, False),
        "done but part 2 knocked over": (start | hold(3, 4, 5) | {"part_2": box(SLOT_X[2], RACK_Y, tilt_deg=90.0)}, 0.25, False),
        "part 3 set on the table beside the bin": (start | hold(4, 5) | {"part_3": box(0.30, 0.25)}, 0.5, False),
        "part 3 still falling into the bin": (start | hold(4, 5) | {"part_3": box(*HOLD_SPOT[3], vel=(0, 0, -0.2))}, 0.5, False),
        "part 3 on top of the bin wall": (start | hold(4, 5) | {"part_3": box(0.30, HOLD[1] + 0.048, TOP + WALL_H)}, 0.5, False),
        "part 1 held instead of part 3": (start | hold(1, 4, 5), 0.25, False),
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
