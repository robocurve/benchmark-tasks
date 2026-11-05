"""On the station table, in front of the left arm, the fan wall of a running server chassis lies flat: a steel sill
with five fixed divider walls forming four fan bays in a row along y, each holding a dark grey hot-swap fan module.
A status tile on the table at each bay's front lip shows its fan's health: green for bays 1 and 3, red for bays 2 and 4
(failed). Two spare fan modules stand in a white spares tray nearer the arms, and a black return bin lies further out.
The servers stay running, so cooling redundancy allows only one empty fan bay at a time: pull failed fan A, seat a
spare in bay A, and only then do bay B. The rule is checked every step: if two bays are ever empty at once, the
trial's score is capped at 0.25. Scored from fan poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/fan-wall-swap-one-at-a-time/fan_wall_swap_one_at_a_time.py \\
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
WALL_X = 0.300  # the fan wall's centreline, running along y
WALL_Y = 0.250
FAN = (0.080, 0.045, 0.060)  # fan module: 80 mm deep (x), 45 mm wide (y), 60 mm tall
FAN_KG = 0.150
DIVIDER = (0.080, 0.006, 0.030)
BAY_GAP = 0.003  # air between a seated fan and each divider
PITCH = FAN[1] + 2 * BAY_GAP + DIVIDER[1]
BAYS = 4
BAY_Y = {k: WALL_Y + (BAYS + 1 - 2 * k) * PITCH / 2 for k in range(1, BAYS + 1)}  # bay 1 at the left (largest y)
DIVIDER_Y = [WALL_Y + (BAYS / 2 - j) * PITCH for j in range(BAYS + 1)]
SILL = (0.060, BAYS * PITCH + DIVIDER[1] + 0.010, 0.0075)
SILL_TOP = TOP + SILL[2]
FAILED = (2, 4)  # the bays with a red status tile, swapped in this order
TILE = (0.020, 0.030, 0.003)
TILE_X = WALL_X - FAN[0] / 2 - 0.035

TRAY_INNER = (0.110, 0.130)
WALL_T, WALL_H = 0.006, 0.015
SPARES_XY = (0.150, 0.250)  # spares tray centre
SPARE_START = {"spare_1": (0.150, 0.280), "spare_2": (0.150, 0.220)}
RETURN_XY = (0.420, 0.250)  # return bin centre
RETURN_SPOT = {2: (0.420, 0.280), 4: (0.420, 0.220)}  # where the demo sets each failed fan down

GOOD_MOVE = 0.003  # a working fan counts as undisturbed within 3 mm and 4 degrees of its start
SEAT_Y, SEAT_X, SEAT_Z, SEAT_DEG = 0.004, 0.006, 0.002, 5.0

DARK = (0.18, 0.19, 0.21)
STEEL = (0.75, 0.76, 0.78)
GREEN = (0.15, 0.70, 0.25)
RED = (0.85, 0.12, 0.10)
WHITE = (0.92, 0.92, 0.92)
BLACK = (0.06, 0.06, 0.06)


def _frame(name, centre, color):
    cx, cy = centre
    ix, iy = TRAY_INNER
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


FANS = [Box(f"fan_{k}", pos=(WALL_X, BAY_Y[k], SILL_TOP + FAN[2] / 2), size=FAN, mass=FAN_KG, color=DARK)
        for k in range(1, BAYS + 1)]
SPARES = [Box(n, pos=(*xy, TOP + FAN[2] / 2), size=FAN, mass=FAN_KG, color=DARK) for n, xy in SPARE_START.items()]
GOOD = [f for f in FANS if int(f.name.split("_")[1]) not in FAILED]
FIXTURES = [
    Box("sill", pos=(WALL_X, WALL_Y, TOP + SILL[2] / 2), size=SILL, color=STEEL),
    *(Box(f"divider_{j}", pos=(WALL_X, y, SILL_TOP + DIVIDER[2] / 2), size=DIVIDER, color=STEEL) for j, y in enumerate(DIVIDER_Y)),
    *(Box(f"status_{k}", pos=(TILE_X, BAY_Y[k], TOP + TILE[2] / 2), size=TILE, color=RED if k in FAILED else GREEN)
      for k in range(1, BAYS + 1)),
    *_frame("spares_tray", SPARES_XY, WHITE),
    *_frame("return_bin", RETURN_XY, BLACK),
]
OBJECTS = [*FANS, *SPARES, *FIXTURES]
ALL_FANS = [o.name for o in (*FANS, *SPARES)]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """How far a fan is from square to the wall; a fan turned 180 degrees is square again."""
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _occupies(st, bay) -> bool:
    """Any fan module down in the bay: centre within 2 cm of the bay centre, bottom within 1.5 cm of the sill."""
    return bool(
        abs(float(st.pos[0]) - WALL_X) < 0.020
        and abs(float(st.pos[1]) - BAY_Y[bay]) < 0.020
        and float(st.low[2]) < SILL_TOP + 0.015
    )


def _empty_bays(objects) -> int:
    return sum(not any(_occupies(objects[n], k) for n in ALL_FANS) for k in range(1, BAYS + 1))


def _seated(st, bay) -> bool:
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        abs(float(st.pos[1]) - BAY_Y[bay]) <= SEAT_Y
        and abs(float(st.pos[0]) - WALL_X) <= SEAT_X
        and abs(float(st.low[2]) - SILL_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= SEAT_DEG
        and st.resting()
    )


def _in_bin(st) -> bool:
    """At rest, the centre inside the return bin's inner walls, the bottom within 2 cm of the table."""
    return bool(
        abs(float(st.pos[0]) - RETURN_XY[0]) < TRAY_INNER[0] / 2
        and abs(float(st.pos[1]) - RETURN_XY[1]) < TRAY_INNER[1] / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.020
        and st.resting()
    )


def _undisturbed(objects) -> bool:
    for f in GOOD:
        st = objects[f.name]
        yaw, tilt = _yaw_tilt(st.quat_xyzw)
        if math.dist([float(v) for v in st.pos], f.pos) > GOOD_MOVE or _square(yaw) > 4.0 or tilt > 4.0:
            return False
    return True


# The trial's history: whether two fan bays were ever empty at the same time. The task file only sees object states,
# so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"violated": False}


def _at_start(objects) -> bool:
    return all(math.dist([float(v) for v in objects[o.name].pos], o.pos) < 0.005 for o in (*FANS, *SPARES))


def _update(objects) -> bool:
    if _at_start(objects):
        _HISTORY["violated"] = False
    elif _empty_bays(objects) >= 2:
        _HISTORY["violated"] = True
    return _HISTORY["violated"]


def score(objects) -> float:
    """0.25 for each failed fan at rest in the return bin and 0.25 for each vacated bay with a spare seated in it.
    Two bays ever empty at once, or a working fan moved more than 3 mm or 4 degrees, caps the score at 0.25."""
    violated = _update(objects)
    s = 0.25 * sum(_in_bin(objects[f"fan_{k}"]) for k in FAILED)
    s += 0.25 * sum(any(_seated(objects[sp.name], k) for sp in SPARES) for k in FAILED)
    if violated or not _undisturbed(objects):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Both failed fans in the return bin, a spare seated in each of their bays, the working fans undisturbed and
    never two bays empty at once; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def violated() -> bool:
    """Whether this trial has had two fan bays empty at once (for the log)."""
    return _HISTORY["violated"]


def distance(objects) -> float:
    """Metres to go: each failed fan to the return bin's centre plus each spare to its bay."""
    d = 0.0
    for k in FAILED:
        st = objects[f"fan_{k}"]
        d += math.hypot(float(st.pos[0]) - RETURN_XY[0], float(st.pos[1]) - RETURN_XY[1])
    for sp, k in zip(SPARES, FAILED):
        st = objects[sp.name]
        d += math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (WALL_X, BAY_Y[k], SILL_TOP))
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto the fan's front and back faces; the dividers sit beside the pads
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
GRIP_DEPTH = 0.025  # the pad tips 25 mm below the fan's top
X_TRIM = 0.003  # a carried 150 g module lands about 3 mm short in x (measured in sim on the same geometry)


def _grip_z(bottom):
    return bottom + FAN[2] - GRIP_DEPTH + TIP


def _move(pick, place, label, trim=0.0):
    """Top-down pick and place across the fan's 80 mm depth, the fingers fully open (94 mm) to clear it, settling
    before every close and release and making the last 2 cm slowly. 17.7 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += trim
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.0015  # set down 1.5 mm above the surface
    grip = -(FAN[0] - SQUEEZE) / 2
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
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set in slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only, one bay at a time: failed fan 2 straight up out of its bay into the return bin, spare 1 straight
    down into bay 2, then failed fan 4 into the bin and spare 2 into bay 4. 70.8 s."""
    plan = []
    for k, (sp, xy) in zip(FAILED, SPARE_START.items()):
        plan += _move((WALL_X, BAY_Y[k], SILL_TOP), (*RETURN_SPOT[k], TOP), f"failed fan {k}")
        plan += _move((*xy, TOP), (WALL_X, BAY_Y[k], SILL_TOP), f"{sp} into bay {k}", trim=X_TRIM)
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(FAN) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {f.name: box(WALL_X, BAY_Y[int(f.name[-1])], SILL_TOP) for f in FANS}
    start |= {n: box(*xy, TOP) for n, xy in SPARE_START.items()}
    lift2 = {"fan_2": box(WALL_X, BAY_Y[2], SILL_TOP + 0.05, vel=(0, 0, 0.05))}
    bin2 = {"fan_2": box(*RETURN_SPOT[2], TOP)}
    lift4 = {"fan_4": box(WALL_X, BAY_Y[4], SILL_TOP + 0.05, vel=(0, 0, 0.05))}
    bin4 = {"fan_4": box(*RETURN_SPOT[4], TOP)}
    in2 = {"spare_1": box(WALL_X, BAY_Y[2], SILL_TOP)}
    in4 = {"spare_2": box(WALL_X - 0.004, BAY_Y[4] + 0.002, SILL_TOP, yaw_deg=2.0)}
    a = start | bin2
    b = a | in2
    c = b | bin4
    d = c | in4

    def run(*frames):
        """Feed the states in order (as the simulator would, step by step); the last score and success."""
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "failed fan 2 lifted out": ((start, start | lift2), 0.0, False),
        "failed fan 2 in the bin": ((start, start | lift2, a), 0.25, False),
        "bay 2 refilled": ((start, a, b), 0.5, False),
        "failed fan 4 in the bin too": ((start, a, b, b | lift4, c), 0.75, False),
        "one at a time, done": ((start, start | lift2, a, b, b | lift4, c, d), 1.0, True),
        "both failed fans out before any spare goes in": ((start, a, a | bin4, a | bin4 | in2, d), 0.25, False),
        "fan 4 lifted while bay 2 still empty, then put back": ((start, a, a | lift4, a, b, c, d), 0.25, False),
        "spare 1 held over bay 2 while fan 4 comes out": ((start, a, a | {"spare_1": box(WALL_X, BAY_Y[2], SILL_TOP + 0.03)} | lift4, d), 0.25, False),
        "a new trial after a violation starts clean": ((start, a | bin4, start, a, b, c, d), 1.0, True),
        "done but working fan 3 pushed 4 mm": ((start, a, b, c, d | {"fan_3": box(WALL_X, BAY_Y[3] - 0.004, SILL_TOP)}), 0.25, False),
        "done but working fan 1 knocked over": ((start, a, b, c, d | {"fan_1": box(0.25, 0.36, TOP, 0, 90)}), 0.25, False),
        "spare 7 mm off across the wall": ((start, a, start | bin2 | {"spare_1": box(WALL_X - 0.007, BAY_Y[2], SILL_TOP)}), 0.25, False),
        "spare turned 10 degrees in its bay": ((start, a, start | bin2 | {"spare_1": box(WALL_X, BAY_Y[2], SILL_TOP, 10.0)}), 0.25, False),
        "spare still moving in its bay": ((start, a, start | bin2 | {"spare_1": box(WALL_X, BAY_Y[2], SILL_TOP, vel=(0, 0, -0.05))}), 0.25, False),
        "good fan 1 pulled into the bin instead": ((start, start | {"fan_1": box(*RETURN_SPOT[2], TOP)}), 0.0, False),
        "spare turned 180 degrees still seats": ((start, a, start | bin2 | {"spare_1": box(WALL_X, BAY_Y[2], SILL_TOP, 180.0)}), 0.5, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not violated()
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
