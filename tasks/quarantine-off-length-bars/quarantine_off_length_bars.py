"""On the station table, in front of the left arm, an in-process inspection bench. A black incoming tray holds four cut
steel bars lying flat along x, side by side, their near ends lined up against a common stop line. Just left of the tray
a white inspection plate carries the master sample: a fixed bar of the correct 80 mm length, its near end on the same
line. Two of the four bars come from a wrong-cut lot, 20 mm short (60 mm); side by side with the master that is
visible from above, but nothing else marks them. Further out stand a grey line tray (good parts go on down the line)
and a red quarantine bin (nonconforming parts are held there). Each bar is judged against the master and moved to
the line tray or the quarantine bin. Scored per bar from the poses at the end; a short bar in the line tray is an
escape and caps the score.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/quarantine-off-length-bars/quarantine_off_length_bars.py \\
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
GOOD_L, SHORT_L = 0.080, 0.060  # the master's length, and the wrong-cut lot (20 mm short)
BAR_W, BAR_H = 0.016, 0.012  # cross-section: 16 mm wide, 12 mm tall, lying flat
STEEL_KG_PER_M = 1.5  # 16 x 12 mm steel bar
IN_TRAY = (0.200, 0.220)
IN_INNER = (0.120, 0.230)
STOP_X = IN_TRAY[0] - GOOD_L / 2  # the near ends' common line
BAR_Y = [IN_TRAY[1] + d for d in (-0.075, -0.025, 0.025, 0.075)]  # 5 cm apart: clear of the 38 mm pads
LENGTHS = {"bar_1": GOOD_L, "bar_2": SHORT_L, "bar_3": GOOD_L, "bar_4": SHORT_L}
GOOD = {n: abs(L - GOOD_L) < 1e-9 for n, L in LENGTHS.items()}
START = {n: (STOP_X + L / 2, y) for (n, L), y in zip(LENGTHS.items(), BAR_Y)}

PLATE = (0.120, 0.050, 0.004)
PLATE_XY = (IN_TRAY[0], 0.385)
PLATE_TOP = TOP + PLATE[2]
LINE_TRAY = (0.370, 0.170)
QUARANTINE = (0.370, 0.370)
BIN_INNER = (0.120, 0.140)
WALL_T, WALL_H = 0.006, 0.015
SPOTS = (-0.025, 0.025)  # two bars side by side in each, along y

STEEL = (0.70, 0.71, 0.73)
BLACK = (0.08, 0.08, 0.08)
WHITE = (0.93, 0.93, 0.93)
LINE_GREY = (0.45, 0.47, 0.50)
RED = (0.80, 0.10, 0.10)


def _frame(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


BARS = [Box(n, pos=(*START[n], TOP + BAR_H / 2), size=(L, BAR_W, BAR_H), mass=round(STEEL_KG_PER_M * L, 3), color=STEEL)
        for n, L in LENGTHS.items()]
FIXTURES = [
    Box("inspection_plate", pos=(*PLATE_XY, TOP + PLATE[2] / 2), size=PLATE, color=WHITE),
    Box("master_bar", pos=(STOP_X + GOOD_L / 2, PLATE_XY[1], PLATE_TOP + BAR_H / 2), size=(GOOD_L, BAR_W, BAR_H), color=STEEL),
    *_frame("incoming_tray", IN_TRAY, IN_INNER, BLACK),
    *_frame("line_tray", LINE_TRAY, BIN_INNER, LINE_GREY),
    *_frame("quarantine_bin", QUARANTINE, BIN_INNER, RED),
]
OBJECTS = [*BARS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _in(st, centre) -> bool:
    """At rest with its centre inside the inner walls and lying low (bottom under 2.5 cm above the table)."""
    return bool(
        abs(float(st.pos[0]) - centre[0]) < BIN_INNER[0] / 2
        and abs(float(st.pos[1]) - centre[1]) < BIN_INNER[1] / 2
        and float(st.low[2]) < TOP + 0.025
        and st.resting()
    )


def placed(objects) -> dict:
    """For each bar: 'line', 'quarantine' or None."""
    out = {}
    for n in LENGTHS:
        st = objects[n]
        out[n] = "line" if _in(st, LINE_TRAY) else "quarantine" if _in(st, QUARANTINE) else None
    return out


def success(objects) -> bool:
    """Both full-length bars in the line tray and both short ones in the quarantine bin; holds for 1 s."""
    p = placed(objects)
    return all(p[n] == ("line" if GOOD[n] else "quarantine") for n in LENGTHS)


def score(objects) -> float:
    """0.25 per bar where it belongs: a full-length bar in the line tray, a short bar in the quarantine bin. A short
    bar in the line tray is an escape to the customer and caps the score at 0.25. Possible scores: 0, 0.25, 0.5, 0.75,
    1.0."""
    p = placed(objects)
    s = 0.25 * sum(p[n] == ("line" if GOOD[n] else "quarantine") for n in LENGTHS)
    if any(p[n] == "line" and not GOOD[n] for n in LENGTHS):
        s = min(s, 0.25)
    return float(s)


def distance(objects) -> float:
    total = 0.0
    for n in LENGTHS:
        goal = LINE_TRAY if GOOD[n] else QUARANTINE
        total += math.hypot(float(objects[n].pos[0]) - goal[0], float(objects[n].pos[1]) - goal[1])
    return float(total)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.110
SQUEEZE = 0.008
DEPTH = 0.008  # pad tips 4 mm above the bar's bottom
OPEN = -(BAR_W + 0.024) / 2
GRIP = -(BAR_W - SQUEEZE) / 2
X_TRIM = 0.005  # a carried part trails the commanded x by about 5 mm on the way down (measured in sim)


def _move(pick, place, label):
    (px, py), (qx, qy) = pick, place
    qx += X_TRIM
    gz = TOP + BAR_H - DEPTH + TIP
    qz = gz + 0.002  # let go 2 mm above the floor
    return [
        Waypoint(1.8, (px, py, TRAVEL), OPEN, f"{label}: over", 0.0),
        Waypoint(1.5, (px, py, gz + 0.020), OPEN, f"{label}: descend", 0.0),
        Waypoint(1.0, (px, py, gz), OPEN, f"{label}: down slowly", 0.0),
        Waypoint(0.5, (px, py, gz), OPEN, f"{label}: settle", 0.0),
        Waypoint(1.5, (px, py, gz), GRIP, f"{label}: close", 0.0),
        Waypoint(1.5, (px, py, TRAVEL), GRIP, f"{label}: lift", 0.0),
        Waypoint(2.0, (qx, qy, TRAVEL), GRIP, f"{label}: carry", 0.0),
        Waypoint(0.6, (qx, qy, TRAVEL), GRIP, f"{label}: settle over", 0.0),
        Waypoint(1.5, (qx, qy, qz + 0.020), GRIP, f"{label}: lower", 0.0),
        Waypoint(1.0, (qx, qy, qz), GRIP, f"{label}: set down slowly", 0.0),
        Waypoint(0.5, (qx, qy, qz), GRIP, f"{label}: settle", 0.0),
        Waypoint(1.2, (qx, qy, qz), OPEN, f"{label}: release", 0.0),
        Waypoint(1.0, (qx, qy, TRAVEL), OPEN, f"{label}: retreat", 0.0),
    ]


def demo():
    """Left arm only: each bar picked at its centre (fingers across its 16 mm width, the pads clear of the bars 5 cm
    either side) and set down flat, the full-length ones side by side in the line tray, the short ones in the
    quarantine bin. 62.4 s."""
    plan, slot = [], {"line": 0, "quarantine": 0}
    for n in LENGTHS:
        dest = "line" if GOOD[n] else "quarantine"
        c = LINE_TRAY if GOOD[n] else QUARANTINE
        plan += _move(START[n], (c[0], c[1] + SPOTS[slot[dest]]), f"{n} to {dest}")
        slot[dest] += 1
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def bar(n, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0)):
        half = np.array([LENGTHS[n] / 2, BAR_W / 2, BAR_H / 2])
        pos = np.array([x, y, bottom + BAR_H / 2])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: bar(n, *START[n]) for n in LENGTHS}
    L = {n: bar(n, LINE_TRAY[0], LINE_TRAY[1] + d) for n, d in zip(LENGTHS, (-0.025, -0.025, 0.025, 0.025))}
    Q = {n: bar(n, QUARANTINE[0], QUARANTINE[1] + d) for n, d in zip(LENGTHS, (-0.025, -0.025, 0.025, 0.025))}
    right = {"bar_1": L["bar_1"], "bar_2": Q["bar_2"], "bar_3": L["bar_3"], "bar_4": Q["bar_4"]}
    cases = {
        "start": (start, 0.0, False),
        "sorted right": (right, 1.0, True),
        "one good bar in the line tray": (start | {"bar_1": L["bar_1"]}, 0.25, False),
        "both short bars quarantined only": (start | {"bar_2": Q["bar_2"], "bar_4": Q["bar_4"]}, 0.5, False),
        "everything quarantined": (Q, 0.5, False),
        "everything to the line": (L, 0.25, False),
        "three right, one short bar escaped to the line": (right | {"bar_4": L["bar_4"]}, 0.25, False),
        "good bar quarantined by mistake": (right | {"bar_3": Q["bar_3"]}, 0.75, False),
        "good bar still sliding in the line tray": (right | {"bar_1": bar("bar_1", *LINE_TRAY, vel=(0.02, 0, 0))}, 0.75,
                                                    False),
        "short bar resting on the quarantine wall, centre outside": (
            right | {"bar_2": bar("bar_2", QUARANTINE[0] + 0.065, QUARANTINE[1], TOP + WALL_H)}, 0.75, False),
        "short bar dropped on the table": (right | {"bar_4": bar("bar_4", 0.30, 0.30)}, 0.75, False),
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
