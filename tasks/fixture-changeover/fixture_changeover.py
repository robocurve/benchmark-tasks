"""On the station table, in front of the left arm, a machine changeover. Straight ahead is the machine table: a fixed
steel slab carrying a fixed locating arrangement for fixture plates, two locating blocks along its far edge (76 mm
apart) and an end stop on its right edge. The current job's fixture plate (an aluminium plate 80 mm deep, 120 mm long,
25 mm thick) sits seated against both blocks and the stop. To the left, a fixed rack bay (a shelf with two end walls
128 mm apart) is the current plate's storage spot. Nearer the arms, on a black staging mat, wait two plates of the
same depth and thickness: the next job's plate, 100 mm long, and a 70 mm plate for another machine, which is too short
to bear on both locating blocks. The changeover: current plate into the rack bay, the next job's plate onto the
machine table, pushed home against both blocks and the stop. Scored from the plates' poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/fixture-changeover/fixture_changeover.py \\
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
PLATE_X = 0.080  # every plate is 80 mm deep (x) and 25 mm thick
PLATE_T = 0.025
LENGTH = {"current_plate": 0.120, "next_plate": 0.100, "other_plate": 0.070}  # along y
MASS = {"current_plate": 0.450, "next_plate": 0.380, "other_plate": 0.270}

DATUM_X = 0.380  # the locating blocks' near faces: a seated plate's far edge
STOP_Y = 0.130  # the end stop's left face: a seated plate's right edge
LOCATOR = (0.012, 0.012, 0.010)
LOCATOR_Y = (STOP_Y + 0.012, STOP_Y + 0.088)  # block centres: 76 mm apart
STOP = (0.030, 0.012, 0.010)
STOP_X = DATUM_X - PLATE_X / 2
MACHINE = (0.110, 0.150, 0.010)  # the machine table slab
MACHINE_C = (0.340, STOP_Y + 0.060)
MT = TOP + MACHINE[2]

RACK = (0.340, 0.420)  # centre of the rack bay
SHELF = (0.100, 0.150, 0.010)
RT = TOP + SHELF[2]
BAY_INNER = 0.128  # between the end walls: the 120 mm plate with 4 mm each end
END_WALL = (0.060, 0.006, 0.015)

STAGE = (0.170, 0.285)
STAGE_MAT = (0.110, 0.230, 0.002)
ST = TOP + STAGE_MAT[2]
STAGED_Y = {"next_plate": 0.230, "other_plate": 0.360}

ALU = (0.78, 0.79, 0.81)
STEEL = (0.45, 0.47, 0.50)
DARK = (0.15, 0.15, 0.16)
ORANGE = (0.90, 0.45, 0.10)
BLACK = (0.08, 0.08, 0.08)


def _seated_centre(name):
    return DATUM_X - PLATE_X / 2, STOP_Y + LENGTH[name] / 2


def _plate(name, x, y, bottom):
    return Box(name, pos=(x, y, bottom + PLATE_T / 2), size=(PLATE_X, LENGTH[name], PLATE_T), mass=MASS[name], color=ALU)


_cx, _cy = _seated_centre("current_plate")
PLATES = [
    _plate("current_plate", _cx - 0.0005, _cy + 0.0005, MT),  # seated, 0.5 mm off the blocks and the stop
    _plate("next_plate", STAGE[0], STAGED_Y["next_plate"], ST),
    _plate("other_plate", STAGE[0], STAGED_Y["other_plate"], ST),
]
FIXTURES = [
    Box("machine_table", pos=(*MACHINE_C, TOP + MACHINE[2] / 2), size=MACHINE, color=STEEL),
    *(Box(f"locator_{k + 1}", pos=(DATUM_X + LOCATOR[0] / 2, y, MT + LOCATOR[2] / 2), size=LOCATOR, color=DARK)
      for k, y in enumerate(LOCATOR_Y)),
    Box("end_stop", pos=(STOP_X, STOP_Y - STOP[1] / 2, MT + STOP[2] / 2), size=STOP, color=DARK),
    Box("rack_shelf", pos=(*RACK, TOP + SHELF[2] / 2), size=SHELF, color=ORANGE),
    *(Box(f"rack_wall_{s}", pos=(RACK[0], RACK[1] + sign * (BAY_INNER + END_WALL[1]) / 2, RT + END_WALL[2] / 2),
          size=END_WALL, color=ORANGE) for s, sign in (("left", 1), ("right", -1))),
    Box("staging_mat", pos=(*STAGE, TOP + STAGE_MAT[2] / 2), size=STAGE_MAT, color=BLACK),
]
OBJECTS = [*PLATES, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_GAP = 0.0025  # seated: within 2.5 mm of both locating blocks and the end stop
SEAT_DEG = 2.0  # and square to the datum within 2 degrees
REST_Z = 0.003


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _fold(yaw_deg) -> float:
    """A plate turned 180 degrees is the same plate: yaw folded into -90..90."""
    return (yaw_deg + 90.0) % 180.0 - 90.0


def _over_machine(state) -> bool:
    return bool(abs(float(state.pos[0]) - MACHINE_C[0]) < MACHINE[0] / 2
                and abs(float(state.pos[1]) - MACHINE_C[1]) < MACHINE[1] / 2)


def _on_machine(state) -> bool:
    """At rest flat on the machine table, its centre over the slab."""
    _, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(_over_machine(state) and abs(float(state.low[2]) - MT) <= REST_Z and tilt <= 5.0 and state.resting())


def datum_gaps(state, name):
    """(gap at locating block 1, at block 2, at the end stop), metres, measured square to the plate's own edges; a
    block the plate's edge does not reach (beyond its length) reads as an infinite gap."""
    yaw, _ = _yaw_tilt(state.quat_xyzw)
    a = math.radians(_fold(yaw))
    c, s = math.cos(a), math.sin(a)
    cx, cy = float(state.pos[0]), float(state.pos[1])
    half_x, half_y = PLATE_X / 2, LENGTH[name] / 2

    def local(px, py):
        dx, dy = px - cx, py - cy
        return c * dx + s * dy, -s * dx + c * dy

    gaps = []
    for y in LOCATOR_Y:
        lx, ly = local(DATUM_X, y)
        gaps.append(lx - half_x if abs(ly) <= half_y else math.inf)
    lx, ly = local(STOP_X, STOP_Y)
    gaps.append(-ly - half_y if abs(lx) <= half_x else math.inf)
    return tuple(gaps)


def _seated(state, name) -> bool:
    yaw, _ = _yaw_tilt(state.quat_xyzw)
    return bool(_on_machine(state) and abs(_fold(yaw)) <= SEAT_DEG
                and all(-0.001 <= g <= SEAT_GAP for g in datum_gaps(state, name)))


def _in_rack(state) -> bool:
    """At rest flat on the rack shelf between its end walls, square to the bay within 8 degrees."""
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[0]) - RACK[0]) <= 0.015
        and abs(float(state.pos[1]) - RACK[1]) <= 0.006
        and abs(float(state.low[2]) - RT) <= REST_Z
        and abs(_fold(yaw)) <= 8.0
        and tilt <= 5.0
        and state.resting()
    )


def _steps(objects):
    cur, nxt, oth = objects["current_plate"], objects["next_plate"], objects["other_plate"]
    return (not _over_machine(cur), _in_rack(cur), _on_machine(nxt), _seated(nxt, "next_plate"),
            _over_machine(oth) and float(oth.low[2]) < MT + PLATE_T)


def success(objects) -> bool:
    """Current plate in the rack bay, next plate seated against both blocks and the stop; holds for 1 s."""
    _, racked, _, seated, wrong = _steps(objects)
    return racked and seated and not wrong


def score(objects) -> float:
    """In order, each step needing the ones before: 0.25 the current plate is off the machine table; 0.5 it rests in
    the rack bay; 0.75 the next job's plate rests flat on the machine table; 1.0 it is seated: within 2.5 mm of both
    locating blocks and the end stop and square within 2 degrees. The short plate on the machine table caps the score
    at 0.5."""
    off, racked, on, seated, wrong = _steps(objects)
    s = 0.0
    if off:
        s = 0.25
        if racked:
            s = 0.5
            if on:
                s = 1.0 if seated else 0.75
    return min(s, 0.5) if wrong else s


def distance(objects) -> float:
    cur, nxt = objects["current_plate"], objects["next_plate"]
    sx, sy = _seated_centre("next_plate")
    return float(math.hypot(float(cur.pos[0]) - RACK[0], float(cur.pos[1]) - RACK[1])
                 + math.hypot(float(nxt.pos[0]) - sx, float(nxt.pos[1]) - sy))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto a plate's near and far edges
TIP = 0.013
TRAVEL = TOP + 0.130
SQUEEZE = 0.008
DEPTH = 0.012  # pad tips 13 mm above a plate's bottom: above the 10 mm blocks and stop
HOVER = 0.0015  # let go 1.5 mm above the resting height
# A held plate hangs about 10 mm below the commanded height, trails the commanded x by 5.4 mm and y by 1.4 mm, and sits
# turned 1.6 degrees from the commanded yaw (all measured in sim, repeatable between runs).
Z_SAG = 0.010
X_LAG = 0.0054
Y_LAG = 0.0014
YAW_TRIM = -0.028
SLIDE = 0.002 + Z_SAG  # the next plate is slid home hovering about 2 mm above the slab, below the 10 mm blocks
APPROACH = 0.002  # it comes down this far clear of the blocks and the stop
PUSH = 0.004  # then is pushed this far past contact, first into the blocks, then into the stop


def _grip_z(bottom):
    return bottom + PLATE_T - DEPTH + TIP


def _pick(x, y, bottom, label, yaw=YAW):
    gz = _grip_z(bottom)
    grip = -(PLATE_X - SQUEEZE) / 2
    return grip, [
        Waypoint(1.8, (x, y, TRAVEL), FINGER_OPEN, f"{label}: over", yaw),
        Waypoint(1.5, (x, y, gz + 0.020), FINGER_OPEN, f"{label}: descend", yaw),
        Waypoint(1.0, (x, y, gz), FINGER_OPEN, f"{label}: down slowly", yaw),
        Waypoint(0.5, (x, y, gz), FINGER_OPEN, f"{label}: settle", yaw),
        Waypoint(1.5, (x, y, gz), grip, f"{label}: close", yaw),
        Waypoint(1.5, (x, y, TRAVEL), grip, f"{label}: lift", yaw),
    ]


def _lower(x, y, bottom, grip, label):
    qz = _grip_z(bottom) + HOVER
    return [
        Waypoint(2.0, (x, y, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (x, y, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (x, y, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (x, y, qz), grip, f"{label}: set down slowly", YAW),
        Waypoint(0.6, (x, y, qz), grip, f"{label}: settle", YAW),
    ]


def _release(x, y, bottom, label):
    qz = _grip_z(bottom) + HOVER
    return [
        Waypoint(1.5, (x, y, qz), FINGER_OPEN, f"{label}: release", YAW),
        Waypoint(1.0, (x, y, TRAVEL), FINGER_OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the current plate lifted off the locators by its near and far edges and set on the rack shelf
    between the end walls; the 100 mm plate from the staging mat lowered 2 mm clear of the datum, slid home hovering
    just above the slab into both locating blocks, then sideways into the end stop, lowered and let go. 37.6 s."""
    cx, cy = _seated_centre("current_plate")
    grip, out = _pick(cx, cy, MT, "current plate")
    out += _lower(*RACK, RT, grip, "current plate") + _release(*RACK, RT, "current plate")
    sx, sy = _seated_centre("next_plate")
    grip, load = _pick(STAGE[0], STAGED_Y["next_plate"], ST, "next plate", YAW + YAW_TRIM)
    ax, ay = sx - APPROACH + X_LAG, sy + APPROACH - Y_LAG  # commanded, so the plate itself comes down 2 mm clear
    hx, hy = sx + PUSH + X_LAG, sy - PUSH - Y_LAG  # commanded push past contact
    qz, rz = _grip_z(MT) + SLIDE, _grip_z(MT) + HOVER
    NYAW = YAW + YAW_TRIM
    load += [
        Waypoint(2.0, (ax, ay, TRAVEL), grip, "next plate: carry", NYAW),
        Waypoint(0.6, (ax, ay, TRAVEL), grip, "next plate: settle over", NYAW),
        Waypoint(1.5, (ax, ay, qz + 0.020), grip, "next plate: lower", NYAW),
        Waypoint(1.5, (ax, ay, qz), grip, "next plate: down to hover just above the slab", NYAW),
        Waypoint(0.6, (ax, ay, qz), grip, "next plate: settle", NYAW),
        Waypoint(1.5, (hx, ay, qz), grip, "next plate: slide into the locating blocks", NYAW),
        Waypoint(1.5, (hx, hy, qz), grip, "next plate: slide into the end stop", NYAW),
        Waypoint(1.0, (hx, hy, rz), grip, "next plate: lower onto the slab", NYAW),
        Waypoint(0.6, (hx, hy, rz), grip, "next plate: settle", NYAW),
    ]
    load += [
        Waypoint(1.5, (hx, hy, rz), FINGER_OPEN, "next plate: release", NYAW),
        Waypoint(1.0, (hx, hy, TRAVEL), FINGER_OPEN, "next plate: retreat", NYAW),
    ]
    return {"left": out + load}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def plate(name, x, y, bottom, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw = math.radians(yaw_deg)
        q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        L = LENGTH[name]
        half = np.array([c * PLATE_X / 2 + s * L / 2, s * PLATE_X / 2 + c * L / 2, PLATE_T / 2])
        pos = np.array([x, y, bottom + PLATE_T / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: plate(p.name, p.pos[0], p.pos[1], p.pos[2] - PLATE_T / 2) for p in PLATES}
    sx, sy = _seated_centre("next_plate")
    racked = {"current_plate": plate("current_plate", *RACK, RT)}
    done = start | racked | {"next_plate": plate("next_plate", sx - 0.001, sy + 0.001, MT)}

    def nxt(dx=0.0, dy=0.0, **kw):
        return start | racked | {"next_plate": plate("next_plate", sx + dx, sy + dy, MT, **kw)}

    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, plate turned end for end": (nxt(-0.001, 0.001, yaw_deg=180), 1.0, True),
        "current plate lifted off onto the table": (start | {"current_plate": plate("current_plate", 0.25, 0.40, TOP)}, 0.25, False),
        "current plate in the rack": (start | racked, 0.5, False),
        "next plate 4 mm short of the blocks": (nxt(-0.004), 0.75, False),
        "next plate 4 mm off the end stop": (nxt(0.0, 0.004), 0.75, False),
        "next plate on one block only, turned 3 degrees": (nxt(-0.002, 0.0, yaw_deg=3), 0.75, False),
        "next plate still sliding": (nxt(vel=(0.02, 0, 0)), 0.5, False),
        "short plate seated instead": (start | racked | {"other_plate": plate("other_plate", sx, STOP_Y + 0.035, MT)}, 0.5, False),
        "next plate seated, current plate left on the staging mat": (
            start | {"current_plate": plate("current_plate", 0.17, 0.40, ST), "next_plate": plate("next_plate", sx, sy, MT)}, 0.25, False),
        "current plate on top of the rack walls": (start | {"current_plate": plate("current_plate", *RACK, RT + END_WALL[2], yaw_deg=90)}, 0.25, False),
        "next plate in the rack, current plate back on the machine": (
            start | {"next_plate": plate("next_plate", *RACK, RT)}, 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert all(abs(g) < 1e-9 for g in datum_gaps(plate("next_plate", sx, sy, MT), "next_plate"))
    assert datum_gaps(plate("other_plate", sx, STOP_Y + 0.035, MT), "other_plate")[1] == math.inf
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
