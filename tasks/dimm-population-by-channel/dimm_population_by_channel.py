"""On the station table, between the arms, a section of a server board: a fixed processor block (CPU and heatsink) in
the middle and a row of eight memory slots, two channels on each side of it. Each channel is a pair of neighbouring
slots: slot 1 is the one farther from the processor and has white latches at both ends, slot 2 is the one nearer the
processor and has black latches. Every slot is a pair of low fixed guide walls on the board. Nearer the arms, four
DIMMs (thin boxes standing on their long edge) stand in the guide channels of an anti-static tray. The population
rule for one DIMM per channel: each channel's slot 1 (farthest from the processor) first, no slot 2 used until every
slot 1 is full. Scored from the DIMM poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/dimm-population-by-channel/dimm_population_by_channel.py \\
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

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
DIMM = (0.078, 0.010, 0.032)  # 78 mm long (x), 10 mm thick (y), 32 mm tall, standing on its long contact edge
DIMM_KG = 0.035

BOARD = (0.150, 0.350, 0.003)  # the board section, centred at (SLOT_X, 0)
SLOT_X = 0.380
BOARD_TOP = TOP + BOARD[2]
CPU = (0.070, 0.060, 0.025)  # processor and heatsink, fixed, centred at (SLOT_X, 0)

# Slot centres along y. Channels A and B on the left of the processor, C and D on the right. Slot 1 of each channel
# is the one farther from the processor.
SLOTS = {
    "A2": 0.060, "A1": 0.085, "B2": 0.120, "B1": 0.145,
    "C2": -0.060, "C1": -0.085, "D2": -0.120, "D1": -0.145,
}
FIRST = ("A1", "B1", "C1", "D1")
SECOND = ("A2", "B2", "C2", "D2")

WALL_T = 0.002  # guide walls either side of each slot and each tray channel
WALL_H = 0.008
CLEAR = 0.002  # air between the DIMM and each guide wall
LATCH = (0.010, 0.012, 0.012)  # a latch block at each end of each slot, 65 mm from the slot centre
LATCH_DX = 0.065

TRAY = (0.110, 0.130, 0.004)  # one anti-static tray per side, centred at (TRAY_X, +-0.125)
TRAY_X = 0.200
TRAY_TOP = TOP + TRAY[2]
TRAY_Y = {"dimm_1": 0.100, "dimm_2": 0.150, "dimm_3": -0.100, "dimm_4": -0.150}

DIMM_GREEN = (0.12, 0.48, 0.24)
BOARD_GREEN = (0.05, 0.22, 0.14)
TRAY_BLUE = (0.18, 0.22, 0.32)
STEEL = (0.75, 0.76, 0.78)
DARK = (0.12, 0.12, 0.13)
WHITE = (0.94, 0.94, 0.94)
BLACK = (0.06, 0.06, 0.06)


def _guides(name, x, y, z_bottom, color):
    """Two guide walls along x either side of a channel for one DIMM."""
    off = DIMM[1] / 2 + CLEAR + WALL_T / 2
    size = (DIMM[0], WALL_T, WALL_H)
    return [Box(f"{name}_guide_{s}", pos=(x, y + sgn * off, z_bottom + WALL_H / 2), size=size, color=color)
            for s, sgn in (("ypos", 1), ("yneg", -1))]


def _latches(slot, y):
    color = WHITE if slot in FIRST else BLACK
    return [Box(f"{slot}_latch_{s}", pos=(SLOT_X + sgn * LATCH_DX, y, BOARD_TOP + LATCH[2] / 2), size=LATCH, color=color)
            for s, sgn in (("near", -1), ("far", 1))]


DIMMS = [Box(n, pos=(TRAY_X, y, TRAY_TOP + DIMM[2] / 2), size=DIMM, mass=DIMM_KG, color=DIMM_GREEN)
         for n, y in TRAY_Y.items()]
FIXTURES = [
    Box("board", pos=(SLOT_X, 0.0, TOP + BOARD[2] / 2), size=BOARD, color=BOARD_GREEN),
    Box("cpu", pos=(SLOT_X, 0.0, BOARD_TOP + CPU[2] / 2), size=CPU, color=STEEL),
    *(w for s, y in SLOTS.items() for w in _guides(s, SLOT_X, y, BOARD_TOP, DARK)),
    *(w for s, y in SLOTS.items() for w in _latches(s, y)),
    *(Box(f"tray_{s}", pos=(TRAY_X, sgn * 0.125, TOP + TRAY[2] / 2), size=TRAY, color=TRAY_BLUE)
      for s, sgn in (("left", 1), ("right", -1))),
    *(w for n, y in TRAY_Y.items() for w in _guides(f"tray_{n}", TRAY_X, y, TRAY_TOP, DARK)),
]
OBJECTS = [*DIMMS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_Y = 0.003  # seated: centre within 3 mm of the slot's centreline (the guides allow 2 mm)
SEAT_X = 0.008  # within 8 mm of the slot centre along the slot
SEAT_Z = 0.003  # bottom within 3 mm of the board top (inside the guides, not on them)
SEAT_DEG = 6.0  # square to the slot
UPRIGHT_DEG = 8.0
IN_Y, IN_X, IN_Z = 0.008, 0.040, 0.015  # loosely in a slot: enough to count a second slot as used


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the DIMM's up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """How far the DIMM is from square to the slots; a DIMM turned 180 degrees is square again."""
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _seated(state, slot) -> bool:
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[1]) - SLOTS[slot]) <= SEAT_Y
        and abs(float(state.pos[0]) - SLOT_X) <= SEAT_X
        and abs(float(state.low[2]) - BOARD_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= UPRIGHT_DEG
        and state.resting()
    )


def _in_slot(state, slot) -> bool:
    return bool(
        abs(float(state.pos[1]) - SLOTS[slot]) <= IN_Y
        and abs(float(state.pos[0]) - SLOT_X) <= IN_X
        and float(state.low[2]) < BOARD_TOP + IN_Z
    )


def populated(objects):
    """The first slots holding a seated DIMM, and the second slots with a DIMM in them at all."""
    firsts = {s for s in FIRST if any(_seated(objects[d.name], s) for d in DIMMS)}
    seconds = {s for s in SECOND if any(_in_slot(objects[d.name], s) for d in DIMMS)}
    return firsts, seconds


def success(objects) -> bool:
    """A DIMM seated in A1, B1, C1 and D1, and nothing in any second slot; holds for 1 s."""
    firsts, seconds = populated(objects)
    return len(firsts) == 4 and not seconds


def score(objects) -> float:
    """0.25 for each first slot (A1, B1, C1, D1) holding a seated DIMM. Any DIMM in a second slot (A2, B2, C2, D2)
    while a first slot is still empty breaks the population rule and caps the score at 0.25; with four DIMMs and four
    first slots, a DIMM in a second slot always leaves a first slot empty."""
    firsts, seconds = populated(objects)
    s = 0.25 * len(firsts)
    if seconds and len(firsts) < 4:
        return min(s, 0.25)
    return s


def distance(objects) -> float:
    """Metres to go: each first slot to the nearest DIMM not already counted for another slot (greedy)."""
    free = [objects[d.name] for d in DIMMS]
    total = 0.0
    for s in FIRST:
        goal = (SLOT_X, SLOTS[s], BOARD_TOP)
        dists = [math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), goal) for st in free]
        k = int(np.argmin(dists))
        total += dists[k]
        free.pop(k)
    return float(total)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the fingers close along x, onto the DIMM's two short ends, as a
# technician holds one by its ends; each arm turns its wrist the way its joint limits allow
TIP = 0.013
TRAVEL = TOP + 0.130
SQUEEZE = 0.008
DEPTH = 0.014  # the pad tips 14 mm below the DIMM's top edge, 18 mm above the board, clear of guides and latches
X_TRIM = 0.0  # a light part lands about 1.5 mm short in x; the slot tolerance along x is 8 mm


def _grip_z(bottom):
    return bottom + DIMM[2] - DEPTH + TIP


def _move(arm, pick, place, label):
    """Top-down pick and place across the DIMM's 78 mm length with the fingers fully open (94 mm), settling before
    every close and release and making the last 2 cm slowly. 17.7 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += X_TRIM
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.0015  # set down 1.5 mm above the board
    grip = -(DIMM[0] - SQUEEZE) / 2
    yaw = YAW[arm]
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{label}: over", yaw),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, f"{label}: descend", yaw),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{label}: down slowly", yaw),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{label}: settle", yaw),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", yaw),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift straight up", yaw),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", yaw),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", yaw),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", yaw),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set in slowly", yaw),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", yaw),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat", yaw),
    ]


def demo():
    """Both arms at once, mirrored, each on its own side of the processor (the grippers stay at least 17 cm apart):
    the left arm fills A1 then B1 from the left tray, the right arm C1 then D1 from the right tray. 35.4 s."""
    plan = {"left": [], "right": []}
    for arm, jobs in (("left", (("dimm_1", "A1"), ("dimm_2", "B1"))), ("right", (("dimm_3", "C1"), ("dimm_4", "D1")))):
        for dimm, slot in jobs:
            plan[arm] += _move(arm, (TRAY_X, TRAY_Y[dimm], TRAY_TOP), (SLOT_X, SLOTS[slot], BOARD_TOP), f"{dimm} to {slot}")
    return plan


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def dimm(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(DIMM) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: dimm(TRAY_X, y, TRAY_TOP) for n, y in TRAY_Y.items()}

    def at(slot, dx=0.0, dy=0.0, dz=0.0, **kw):
        return dimm(SLOT_X + dx, SLOTS[slot] + dy, BOARD_TOP + dz, **kw)

    def scene(**kw):
        return start | kw

    done = scene(dimm_1=at("A1"), dimm_2=at("B1"), dimm_3=at("C1"), dimm_4=at("D1"))
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, DIMMs in another order and turned 180 degrees":
            (scene(dimm_1=at("D1", yaw_deg=180), dimm_2=at("C1"), dimm_3=at("B1"), dimm_4=at("A1")), 1.0, True),
        "A1 only": (scene(dimm_1=at("A1")), 0.25, False),
        "A1 and C1": (scene(dimm_1=at("A1"), dimm_3=at("C1")), 0.5, False),
        "three first slots": (scene(dimm_1=at("A1"), dimm_2=at("B1"), dimm_3=at("C1")), 0.75, False),
        "three first slots and one in D2": (scene(dimm_1=at("A1"), dimm_2=at("B1"), dimm_3=at("C1"), dimm_4=at("D2")), 0.25, False),
        "outermost four slots (B1, B2 order wrong side)":
            (scene(dimm_1=at("B1"), dimm_2=at("B2"), dimm_3=at("D1"), dimm_4=at("D2")), 0.25, False),
        "nearest the processor (all second slots)":
            (scene(dimm_1=at("A2"), dimm_2=at("B2"), dimm_3=at("C2"), dimm_4=at("D2")), 0.0, False),
        "A1 landed 6 mm short along the slot": (scene(dimm_1=at("A1", dx=-0.006)), 0.25, False),
        "A1 12 mm off along the slot": (scene(dimm_1=at("A1", dx=-0.012)), 0.0, False),
        "A1 resting on top of its guides": (scene(dimm_1=at("A1", dz=WALL_H)), 0.0, False),
        "A1 leaning 12 degrees": (scene(dimm_1=at("A1", dz=0.002, tilt_deg=12)), 0.0, False),
        "A1 turned 10 degrees": (scene(dimm_1=at("A1", yaw_deg=10)), 0.0, False),
        "A1 still moving": (scene(dimm_1=at("A1", vel=(0.0, 0.0, -0.05))), 0.0, False),
        "held 3 cm over A1": (scene(dimm_1=at("A1", dz=0.030)), 0.0, False),
        "one lying flat on the board by B1": (scene(dimm_2=dimm(SLOT_X, 0.165, BOARD_TOP, 0, 90)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(done) < 1e-9 and distance(start) > 0.5
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then each arm's plan replayed through the
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
