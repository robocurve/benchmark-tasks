"""On the station table, in front of the left arm, a DIN rail lies flat, running left to right, fitted with a full row
of five grey 45 mm I/O modules between two fixed end clamps, 1.5 mm apart. The module in slot 3 (the middle one,
counting from the left clamp) has failed and is red. A blue spare stands in a white spares pocket nearer the arms, and
a black return pocket lies further out. The job is a module swap: the failed module goes into the return pocket and
the spare is seated in slot 3, without shifting the four working modules. Scored from module poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/io-module-swap/io_module_swap.py \\
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
RAIL_X = 0.300  # the rail's centreline, running along y
RAIL = (0.035, 0.290, 0.0075)  # TS35 x 7.5 top-hat rail, 29 cm long, modelled as its outline
RAIL_Y = 0.250
RAIL_TOP = TOP + RAIL[2]

MOD = (0.080, 0.045, 0.060)  # module: 80 mm across the rail (x), 45 mm wide along it (y), 60 mm tall
MOD_KG = 0.150
GAP = 0.0015  # air between neighbouring modules
PITCH = MOD[1] + GAP
SLOTS = 5
FAILED = 3  # slot number, counting from the left end clamp (largest y)
SLOT_Y = {k: RAIL_Y + (SLOTS + 1 - 2 * k) * PITCH / 2 for k in range(1, SLOTS + 1)}  # slot 1 at the left
CLAMP = (0.040, 0.010, 0.030)  # fixed end clamps on the rail, 1.5 mm outside the outer modules
CLAMP_Y = {
    "left": SLOT_Y[1] + MOD[1] / 2 + GAP + CLAMP[1] / 2,
    "right": SLOT_Y[SLOTS] - MOD[1] / 2 - GAP - CLAMP[1] / 2,
}

POCKET_INNER = (0.110, 0.075)
WALL_T = 0.006
WALL_H = 0.015
SPARES = (0.150, 0.250)  # pocket centres
RETURN = (0.420, 0.250)

GREY = (0.62, 0.63, 0.65)
RED = (0.78, 0.12, 0.10)
BLUE = (0.15, 0.32, 0.80)
DARK = (0.12, 0.12, 0.13)
STEEL = (0.75, 0.76, 0.78)
WHITE = (0.92, 0.92, 0.92)
BLACK = (0.08, 0.08, 0.08)


def _walls(name, centre, color):
    cx, cy = centre
    ix, iy = POCKET_INNER
    z = TOP + WALL_H / 2
    side = (ix + 2 * WALL_T, WALL_T, WALL_H)
    end = (WALL_T, iy, WALL_H)
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=end, color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=end, color=color),
    ]


def _module(name, x, y, z_bottom, color):
    return Box(name, pos=(x, y, z_bottom + MOD[2] / 2), size=MOD, mass=MOD_KG, color=color)


MODULES = [
    _module(f"module_{k}", RAIL_X, SLOT_Y[k], RAIL_TOP, RED if k == FAILED else GREY) for k in range(1, SLOTS + 1)
]
SPARE = _module("spare", SPARES[0], SPARES[1], TOP, BLUE)
NEIGHBOURS = [m for m in MODULES if m.name != f"module_{FAILED}"]
FIXTURES = [
    Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    *(Box(f"end_clamp_{s}", pos=(RAIL_X, y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, color=DARK) for s, y in CLAMP_Y.items()),
    *_walls("spares", SPARES, WHITE),
    *_walls("return", RETURN, BLACK),
]
OBJECTS = [*MODULES, SPARE, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_Y = 0.003  # seated: centre within 3 mm of the slot centre along the rail
SEAT_X = 0.004  # and within 4 mm of the rail's centreline across it
SEAT_Z = 0.002  # bottom within 2 mm of the rail top
SEAT_DEG = 4.0  # square to the rail
UPRIGHT_DEG = 10.0
NEIGHBOUR_MOVE = 0.003  # a working module counts as undisturbed within 3 mm and 4 degrees of its start


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the module's up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """How far the module is from square to the rail; a box turned 180 degrees is square again."""
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _bottom(state) -> float:
    return float(state.low[2])


def _seated(state, slot) -> bool:
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[1]) - SLOT_Y[slot]) <= SEAT_Y
        and abs(float(state.pos[0]) - RAIL_X) <= SEAT_X
        and abs(_bottom(state) - RAIL_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= UPRIGHT_DEG
        and state.resting()
    )


def _in_pocket(state, centre) -> bool:
    """At rest, the centre inside the pocket's inner walls, the bottom within 2 cm of the table (standing or lying)."""
    return bool(
        abs(float(state.pos[0]) - centre[0]) < POCKET_INNER[0] / 2
        and abs(float(state.pos[1]) - centre[1]) < POCKET_INNER[1] / 2
        and TOP - 0.005 < _bottom(state) < TOP + 0.020
        and state.resting()
    )


def _off_rail(state) -> bool:
    """Clear of slot 3: its centre more than a module width from the slot, or its bottom 2 cm above the rail."""
    return bool(
        math.hypot(float(state.pos[0]) - RAIL_X, float(state.pos[1]) - SLOT_Y[FAILED]) > MOD[1]
        or _bottom(state) > RAIL_TOP + 0.020
    )


def _undisturbed(objects) -> bool:
    for m in NEIGHBOURS:
        st = objects[m.name]
        yaw, tilt = _yaw_tilt(st.quat_xyzw)
        moved = math.dist((float(st.pos[0]), float(st.pos[1]), float(st.pos[2])), m.pos)
        if moved > NEIGHBOUR_MOVE or _square(yaw) > SEAT_DEG or tilt > SEAT_DEG:
            return False
    return True


def success(objects) -> bool:
    """The failed module in the return pocket, the spare seated in slot 3, the four working modules undisturbed;
    holds for 1 s."""
    return (
        _in_pocket(objects[f"module_{FAILED}"], RETURN)
        and _seated(objects["spare"], FAILED)
        and _undisturbed(objects)
    )


def score(objects) -> float:
    """The removal comes first. 0.25 once the failed module is clear of slot 3; 0.5 once it is in the return pocket;
    then 0.75 if the spare also stands on the rail in slot 3 (centre within 1 cm of the slot, bottom within 5 mm of
    the rail top) and 1.0 when it is seated. Any working module moved more than 3 mm or 4 degrees caps the score at
    0.25. A spare put in slot 3 while the failed module is still there cannot happen without moving modules."""
    failed, spare = objects[f"module_{FAILED}"], objects["spare"]
    if not _off_rail(failed):
        return 0.0
    if not _undisturbed(objects):
        return 0.25
    if not _in_pocket(failed, RETURN):
        return 0.25
    if _seated(spare, FAILED):
        return 1.0
    near = abs(float(spare.pos[1]) - SLOT_Y[FAILED]) <= 0.010 and abs(float(spare.pos[0]) - RAIL_X) <= 0.010
    if near and abs(_bottom(spare) - RAIL_TOP) <= 0.005:
        return 0.75
    return 0.5


def distance(objects) -> float:
    """Metres to go: the failed module to the return pocket's centre plus the spare to slot 3."""
    f, s = objects[f"module_{FAILED}"], objects["spare"]
    return float(
        math.hypot(float(f.pos[0]) - RETURN[0], float(f.pos[1]) - RETURN[1])
        + math.dist((float(s.pos[0]), float(s.pos[1]), _bottom(s)), (RAIL_X, SLOT_Y[FAILED], RAIL_TOP))
    )


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto the module's front and back faces; neighbours sit beside the pads
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
GRIP_DEPTH = 0.025  # the pad tips 25 mm below the module's top


def _grip_z(bottom):
    return bottom + MOD[2] - GRIP_DEPTH + TIP


def _move(pick, place, label):
    """Top-down pick and place across the module's 80 mm depth, with the fingers fully open (94 mm) to clear it,
    settling before every close and release and making the last 2 cm slowly. 19.0 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.0015  # set down 1.5 mm above the surface
    grip = -(MOD[0] - SQUEEZE) / 2
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
    """Left arm only: the failed module straight up out of slot 3 and into the return pocket, then the spare from its
    pocket straight down into slot 3. 38.2 s."""
    out = _move((RAIL_X, SLOT_Y[FAILED], RAIL_TOP), (RETURN[0], RETURN[1], TOP), "failed module")
    into = _move((SPARES[0], SPARES[1], TOP), (RAIL_X, SLOT_Y[FAILED], RAIL_TOP), "spare")
    return {"left": out + into}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        qyaw = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        qtilt = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
        x1, y1, z1, w1 = qyaw
        x2, y2, z2, w2 = qtilt
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(MOD) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {m.name: box(m.pos[0], m.pos[1], RAIL_TOP) for m in MODULES} | {"spare": box(*SPARES, TOP)}
    failed = f"module_{FAILED}"
    in_return = box(RETURN[0], RETURN[1], TOP)
    seated = box(RAIL_X, SLOT_Y[FAILED], RAIL_TOP)

    def scene(**kw):
        return start | kw

    shifted = box(RAIL_X, SLOT_Y[2] + 0.004, RAIL_TOP)
    cases = {
        "start": (start, 0.0, False),
        "done": (scene(**{failed: in_return, "spare": seated}), 1.0, True),
        "done, spare turned 180 degrees": (scene(**{failed: in_return, "spare": box(RAIL_X, SLOT_Y[3], RAIL_TOP, 180)}), 1.0, True),
        "failed module lifted 3 cm": (scene(**{failed: box(RAIL_X, SLOT_Y[3], RAIL_TOP + 0.03)}), 0.25, False),
        "failed module on the table": (scene(**{failed: box(0.20, 0.10, TOP, 30)}), 0.25, False),
        "failed module in the return pocket": (scene(**{failed: in_return}), 0.5, False),
        "failed module lying in the return pocket": (scene(**{failed: box(RETURN[0], RETURN[1], TOP, 0, 90)}), 0.5, False),
        "failed module in the spares pocket": (scene(**{failed: box(SPARES[0], SPARES[1] + 0.01, TOP)}), 0.25, False),
        "spare on the rail 6 mm off the slot": (scene(**{failed: in_return, "spare": box(RAIL_X, SLOT_Y[3] - 0.006, RAIL_TOP)}), 0.75, False),
        "spare leaning in the slot": (scene(**{failed: in_return, "spare": box(RAIL_X, SLOT_Y[3], RAIL_TOP + 0.003, 0, 6)}), 0.75, False),
        "spare turned 10 degrees in the slot": (scene(**{failed: in_return, "spare": box(RAIL_X, SLOT_Y[3], RAIL_TOP, 10)}), 0.75, False),
        "spare still moving in the slot": (scene(**{failed: in_return, "spare": box(RAIL_X, SLOT_Y[3], RAIL_TOP, vel=(0, 0, -0.05))}), 0.75, False),
        "spare held 3 cm over the slot": (scene(**{failed: in_return, "spare": box(RAIL_X, SLOT_Y[3], RAIL_TOP + 0.03)}), 0.5, False),
        "done but slot 2 pushed 4 mm": (scene(**{failed: in_return, "spare": seated, "module_2": shifted}), 0.25, False),
        "done but slot 4 knocked over": (scene(**{failed: in_return, "spare": seated, "module_4": box(0.30, 0.15, TOP, 0, 90)}), 0.25, False),
        "slot 2 pulled out instead": (scene(module_2=in_return), 0.0, False),
        "failed module still in its slot, spare beside the rail": (scene(spare=box(0.36, 0.25, TOP)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(scene(**{failed: box(RETURN[0], RETURN[1], TOP), "spare": seated})) < 1e-9
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
