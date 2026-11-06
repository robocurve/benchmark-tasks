"""On the station table, in front of the left arm, the outfeed slide of a machining cell that has just been changed
over to a new job: two low steel rails running across the table (along y), with the machine, a grey block, at their
left end. The machine pushes each finished part out onto the slide, so the three aluminium blocks on it stand in the
order they were made: part 1, the first part off the new setup, has travelled furthest from the machine (nearest the
middle of the table); part 3, the newest, is nearest the machine. Around the slide are three containers: a white
first-article tray (nearer the arms, on the left), a green good-parts bin (nearer the arms, in the middle) and a red
hold tray (further right). First-article rule: after a changeover, the first part off the new setup goes to the
inspector in the first-article tray, and every part made after it waits in the hold tray until the first article is
signed off; nothing from the new setup goes into the good-parts bin yet. Scored from the part poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/first-article-hold/first_article_hold.py \\
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
N_PARTS = 3
SLIDE_X = 0.300
SLOT_Y = {k: 0.250 + 0.060 * (k - 1) for k in range(1, N_PARTS + 1)}  # part 1 (first made) furthest from the machine
FIRST = 1
FOLLOWERS = (2, 3)

RAIL = (0.006, 0.215, 0.010)
RAIL_Y = 0.3175
RAIL_DX = PART[0] / 2 + 0.012 + RAIL[0] / 2  # 12 mm of air each side of a part
MACHINE = (0.120, 0.060, 0.100)
MACHINE_Y = SLOT_Y[N_PARTS] + PART[1] / 2 + 0.040 + MACHINE[1] / 2

FAI = (0.170, 0.420)  # first-article tray centre
FAI_INNER = (0.080, 0.080)
GOOD = (0.165, 0.250)  # good-parts bin centre
GOOD_INNER = (0.100, 0.100)
HOLD = (0.300, 0.140)  # hold tray centre
HOLD_INNER = (0.160, 0.080)
WALL_T, WALL_H = 0.006, 0.020
SPOT = {1: FAI, 2: (0.260, 0.140), 3: (0.340, 0.140)}  # where the demo sets each part
X_TRIM = 0.003  # carried 90 g blocks land a few mm short in x: aim this far beyond the spot

ALU = (0.78, 0.79, 0.81)
STEEL = (0.45, 0.47, 0.50)
GREY = (0.35, 0.37, 0.40)
WHITE = (0.94, 0.94, 0.92)
GREEN = (0.15, 0.60, 0.25)
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


PARTS = [Box(f"part_{k}", pos=(SLIDE_X, SLOT_Y[k], TOP + PART[2] / 2), size=PART, mass=PART_KG, color=ALU)
         for k in range(1, N_PARTS + 1)]
FIXTURES = [
    Box("rail_near", pos=(SLIDE_X - RAIL_DX, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("rail_far", pos=(SLIDE_X + RAIL_DX, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("machine", pos=(SLIDE_X, MACHINE_Y, TOP + MACHINE[2] / 2), size=MACHINE, color=GREY),
    *_walls("fai_tray", FAI, FAI_INNER, WHITE),
    *_walls("good_bin", GOOD, GOOD_INNER, GREEN),
    *_walls("hold_tray", HOLD, HOLD_INNER, RED),
]
OBJECTS = [*PARTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _inside(st, centre, inner) -> bool:
    """At rest, the centre inside the container's inner walls and the lowest point within 2 cm of the table."""
    return bool(
        abs(float(st.pos[0]) - centre[0]) < inner[0] / 2
        and abs(float(st.pos[1]) - centre[1]) < inner[1] / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.020
        and st.resting()
    )


def in_fai(st) -> bool:
    return _inside(st, FAI, FAI_INNER)


def in_hold(st) -> bool:
    return _inside(st, HOLD, HOLD_INNER)


def in_good(st) -> bool:
    """Anywhere over the good-parts bin's opening (resting or not): released into the bin."""
    return bool(abs(float(st.pos[0]) - GOOD[0]) < GOOD_INNER[0] / 2 + WALL_T
                and abs(float(st.pos[1]) - GOOD[1]) < GOOD_INNER[1] / 2 + WALL_T
                and float(st.low[2]) < TOP + 0.030)


def _routed(objects, k) -> bool:
    st = objects[f"part_{k}"]
    return in_fai(st) if k == FIRST else in_hold(st)


def score(objects) -> float:
    """0.25 for part 1 (the first article) at rest in the first-article tray, 0.25 for each of parts 2 and 3 at rest
    in the hold tray, and the last 0.25 when all three are routed so. A part in the wrong container, or in the
    good-parts bin, earns nothing for itself."""
    n = sum(_routed(objects, k) and not in_good(objects[f"part_{k}"]) for k in range(1, N_PARTS + 1))
    return float(0.25 * n + 0.25 * (n == N_PARTS))


def success(objects) -> bool:
    """Part 1 in the first-article tray, parts 2 and 3 in the hold tray; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: each part's centre to its container's centre (in the plane)."""
    d = 0.0
    for k in range(1, N_PARTS + 1):
        st = objects[f"part_{k}"]
        goal = FAI if k == FIRST else HOLD
        d += math.hypot(float(st.pos[0]) - goal[0], float(st.pos[1]) - goal[1])
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
    qx += X_TRIM
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
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{label}: settle over the tray"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{label}: set down"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat"),
    ]


def demo():
    """Left arm only: part 1 to the first-article tray, then parts 2 and 3 to the hold tray. 39.6 s."""
    plan = []
    for k in range(1, N_PARTS + 1):
        plan += _move((SLIDE_X, SLOT_Y[k]), SPOT[k], f"part {k}")
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

    start = {f"part_{k}": box(SLIDE_X, SLOT_Y[k]) for k in range(1, N_PARTS + 1)}

    def at(k, xy):
        return {f"part_{k}": box(*xy)}

    done = start | at(1, FAI) | at(2, SPOT[2]) | at(3, SPOT[3])
    cases = {
        "start": (start, 0.0, False),
        "first article in its tray": (start | at(1, FAI), 0.25, False),
        "part 2 on hold": (start | at(2, SPOT[2]), 0.25, False),
        "done": (done, 1.0, True),
        "all three on hold (no first article)": (start | at(1, (0.30, 0.14)) | at(2, SPOT[2]) | at(3, SPOT[3]), 0.5, False),
        "newest part sent for inspection instead": (start | at(3, FAI) | at(1, SPOT[3]) | at(2, SPOT[2]), 0.25, False),
        "followers released to the good-parts bin": (start | at(1, FAI) | at(2, GOOD) | at(3, (0.19, 0.25)), 0.25, False),
        "all three into the good-parts bin": (start | at(1, GOOD) | at(2, (0.15, 0.22)) | at(3, (0.19, 0.26)), 0.0, False),
        "part 3 still falling into the hold tray": (done | {"part_3": box(*SPOT[3], vel=(0, 0, -0.2))}, 0.5, False),
        "part 3 on the hold tray's wall": (done | {"part_3": box(0.34, HOLD[1] + 0.043, TOP + WALL_H)}, 0.5, False),
        "part 2 beside the hold tray": (done | at(2, (0.30, 0.20)), 0.5, False),
        "first article on the table next to its tray": (done | at(1, (0.17, 0.48)), 0.5, False),
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
