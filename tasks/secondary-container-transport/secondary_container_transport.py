"""On the station table, in front of the left arm, two labs on either side of a corridor (scaled down): the sending
lab's bench nearest the arm (x below 25 cm), a corridor strip across the table (x 25 to 33 cm, grey floor with a
yellow tape line along each edge) and the receiving lab's bench beyond it, where a green receiving mat lies. On the
sending bench stand two amber sample bottles and, beside them, an empty white bottle carrier: the rigid secondary
container that must hold any biological sample taken out of a lab. Rule: a sample may cross the corridor only inside
the carrier, so the bottles go into the carrier on the sending bench and the loaded carrier is carried across to the
receiving mat. The route is checked every step: a bottle seen over the corridor while not inside the carrier fails,
even if it ends up in the carrier on the mat. Scored from the poses of the bottles and the carrier.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/secondary-container-transport/secondary_container_transport.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
CORRIDOR_X = (0.250, 0.330)  # the corridor, along y across the table
CORRIDOR_Y = (0.020, 0.560)
TAPE_W = 0.010
FLOOR_T = 0.001
BOTTLE_R, BOTTLE_H, BOTTLE_KG = 0.016, 0.060, 0.080
BOTTLE_START = {"bottle_1": (0.180, 0.400), "bottle_2": (0.180, 0.460)}
CARRIER = (0.070, 0.200, 0.022)  # 7 cm deep (x), 20 cm long (y), 2.2 cm tall
CARRIER_KG = 0.250
CARRIER_START = (0.170, 0.220)
SLOT_DY = 0.065  # the demo stands the bottles 6.5 cm either side of the carrier's middle, clear of the gripper
MAT = (0.120, 0.240, 0.002)
MAT_XY = (0.420, 0.220)
DEST_X = CORRIDOR_X[1] + 0.010  # the receiving bench starts 1 cm past the corridor's far tape

AMBER = (0.70, 0.40, 0.10)
WHITE = (0.93, 0.93, 0.93)
GREY = (0.55, 0.56, 0.58)
YELLOW = (0.95, 0.80, 0.10)
GREEN = (0.20, 0.55, 0.30)

BOTTLES = [Cylinder(n, pos=(*xy, TOP + BOTTLE_H / 2), radius=BOTTLE_R, height=BOTTLE_H, mass=BOTTLE_KG, color=AMBER)
           for n, xy in BOTTLE_START.items()]
CARRIER_OBJ = Box("carrier", pos=(*CARRIER_START, TOP + CARRIER[2] / 2), size=CARRIER, mass=CARRIER_KG, color=WHITE)
_cy, _ly = sum(CORRIDOR_Y) / 2, CORRIDOR_Y[1] - CORRIDOR_Y[0]
FIXTURES = [
    Box("corridor_floor", pos=(sum(CORRIDOR_X) / 2, _cy, TOP + FLOOR_T / 2),
        size=(CORRIDOR_X[1] - CORRIDOR_X[0] - 2 * TAPE_W, _ly, FLOOR_T), color=GREY),
    Box("corridor_tape_near", pos=(CORRIDOR_X[0] + TAPE_W / 2, _cy, TOP + FLOOR_T / 2), size=(TAPE_W, _ly, FLOOR_T),
        color=YELLOW),
    Box("corridor_tape_far", pos=(CORRIDOR_X[1] - TAPE_W / 2, _cy, TOP + FLOOR_T / 2), size=(TAPE_W, _ly, FLOOR_T),
        color=YELLOW),
    Box("receiving_mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=GREEN),
]
OBJECTS = [*BOTTLES, CARRIER_OBJ, *FIXTURES]
FREE = [*BOTTLES, CARRIER_OBJ]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical in radians, tilt of the up axis from the vertical in degrees)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw, tilt


def inside(bottle, carrier) -> bool:
    """The bottle stands upright on the carrier's floor within its outline (in the carrier's own frame, 4 mm in from
    its edges), moving or not: a bottle riding in the carrier is inside it."""
    yaw, ctilt = _yaw_tilt(carrier.quat_xyzw)
    _, btilt = _yaw_tilt(bottle.quat_xyzw)
    dx, dy = float(bottle.pos[0] - carrier.pos[0]), float(bottle.pos[1] - carrier.pos[1])
    lx, ly = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    floor = float(carrier.high[2])
    return bool(
        abs(lx) <= CARRIER[0] / 2 - 0.004
        and abs(ly) <= CARRIER[1] / 2 - 0.004
        and -0.004 <= float(bottle.low[2]) - floor <= 0.010
        and btilt < 15.0
        and ctilt < 15.0
    )


def over_corridor(st) -> bool:
    """The bottle's centre is over the corridor strip, at any height."""
    return bool(CORRIDOR_X[0] < float(st.pos[0]) < CORRIDOR_X[1] and CORRIDOR_Y[0] < float(st.pos[1]) < CORRIDOR_Y[1])


def delivered(carrier) -> bool:
    """The carrier at rest, flat, on the receiving bench: its whole footprint past the corridor and on the table."""
    _, tilt = _yaw_tilt(carrier.quat_xyzw)
    return bool(float(carrier.low[0]) >= DEST_X and float(carrier.low[2]) < TOP + MAT[2] + 0.004 and tilt < 5.0
                and carrier.resting())


# The trial's history: which bottles have been over the corridor outside the carrier. The task file only sees object
# states, so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"breached": set()}


def _at_start(objects) -> bool:
    return all(math.dist([float(v) for v in objects[o.name].pos], o.pos) < 0.005 for o in FREE)


def _update(objects) -> set:
    if _at_start(objects):
        _HISTORY["breached"] = set()
    for b in BOTTLES:
        st = objects[b.name]
        if over_corridor(st) and not inside(st, objects["carrier"]):
            _HISTORY["breached"].add(b.name)
    return _HISTORY["breached"]


def score(objects) -> float:
    """Per bottle that has never been over the corridor outside the carrier: 0.25 while it is inside the carrier, and
    0.25 more when that carrier is at rest on the receiving bench. A bottle that crossed outside the carrier earns
    nothing, wherever it ends."""
    breached = _update(objects)
    carrier = objects["carrier"]
    there = delivered(carrier)
    s = 0.0
    for b in BOTTLES:
        if b.name not in breached and inside(objects[b.name], carrier):
            s += 0.25 + 0.25 * (there and objects[b.name].resting())
    return float(s)


def success(objects) -> bool:
    """Both bottles inside the carrier, the carrier at rest on the receiving bench, and neither bottle ever over the
    corridor outside the carrier; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def breached() -> list:
    """The bottles that crossed outside the carrier this trial (for the log)."""
    return sorted(_HISTORY["breached"])


def distance(objects) -> float:
    """Metres to go: each bottle to the carrier plus the carrier to the receiving mat's centre."""
    c = objects["carrier"]
    d = math.hypot(float(c.pos[0]) - MAT_XY[0], float(c.pos[1]) - MAT_XY[1])
    for b in BOTTLES:
        d += 0.0 if inside(objects[b.name], c) else math.dist([float(v) for v in objects[b.name].pos], c.pos)
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.140
SQUEEZE = 0.008
CARRIER_YAW = -math.pi / 2  # the fingers close along x, across the carrier's 7 cm depth, at its middle
GRIP_TIPS = 0.004  # carrier grip: pad tips 4 mm above the table
X_TRIM = 0.004  # a carried 400 g load lands a few mm short in x


def _bottle_move(name, place):
    (px, py), (qx, qy) = BOTTLE_START[name], place
    gz = TOP + BOTTLE_H - 0.030 + TIP  # pad tips 30 mm below the bottle's top
    qz = gz + CARRIER[2] + 0.002
    open_ = -(2 * BOTTLE_R + 0.024) / 2
    grip = -(2 * BOTTLE_R - SQUEEZE) / 2
    lab = name.replace("_", " ")
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{lab}: over"),
        Waypoint(1.5, (px, py, gz), open_, f"{lab}: descend"),
        Waypoint(0.5, (px, py, gz), open_, f"{lab}: settle"),
        Waypoint(1.0, (px, py, gz), grip, f"{lab}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{lab}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{lab}: over the carrier (sending bench)"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{lab}: settle over"),
        Waypoint(1.5, (qx, qy, qz + 0.015), grip, f"{lab}: lower"),
        Waypoint(1.0, (qx, qy, qz), grip, f"{lab}: set in slowly"),
        Waypoint(0.5, (qx, qy, qz), grip, f"{lab}: settle"),
        Waypoint(1.0, (qx, qy, qz), open_, f"{lab}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{lab}: retreat"),
    ]


def demo():
    """Left arm only: both bottles into the carrier on the sending bench, then the loaded carrier gripped at its
    middle and carried slowly across the corridor to the receiving mat. 49.0 s."""
    cx, cy = CARRIER_START
    plan = _bottle_move("bottle_1", (cx, cy + SLOT_DY)) + _bottle_move("bottle_2", (cx, cy - SLOT_DY))
    gz = TOP + GRIP_TIPS + TIP
    qx, qy = MAT_XY[0] + X_TRIM, MAT_XY[1]
    qz = gz + MAT[2] + 0.002
    grip = -(CARRIER[0] - SQUEEZE) / 2
    lift = TOP + 0.090
    plan += [
        Waypoint(1.8, (cx, cy, TRAVEL), FINGER_OPEN, "carrier: over its middle", CARRIER_YAW),
        Waypoint(1.5, (cx, cy, gz + 0.020), FINGER_OPEN, "carrier: descend", CARRIER_YAW),
        Waypoint(1.0, (cx, cy, gz), FINGER_OPEN, "carrier: down slowly", CARRIER_YAW),
        Waypoint(0.5, (cx, cy, gz), FINGER_OPEN, "carrier: settle", CARRIER_YAW),
        Waypoint(1.5, (cx, cy, gz), grip, "carrier: close", CARRIER_YAW),
        Waypoint(2.0, (cx, cy, lift), grip, "carrier: lift slowly", CARRIER_YAW),
        Waypoint(4.0, (qx, qy, lift), grip, "carrier: across the corridor, slowly", CARRIER_YAW),
        Waypoint(0.8, (qx, qy, lift), grip, "carrier: settle over the mat", CARRIER_YAW),
        Waypoint(2.0, (qx, qy, qz + 0.015), grip, "carrier: lower", CARRIER_YAW),
        Waypoint(1.0, (qx, qy, qz), grip, "carrier: set down slowly", CARRIER_YAW),
        Waypoint(0.6, (qx, qy, qz), grip, "carrier: settle", CARRIER_YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "carrier: release", CARRIER_YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "carrier: retreat", CARRIER_YAW),
    ]
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the route history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
        a = math.radians(yaw_deg)
        q = np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])
        hx = abs(math.cos(a)) * CARRIER[0] / 2 + abs(math.sin(a)) * CARRIER[1] / 2
        hy = abs(math.sin(a)) * CARRIER[0] / 2 + abs(math.cos(a)) * CARRIER[1] / 2
        half = np.array([hx, hy, CARRIER[2] / 2])
        pos = np.array([x, y, bottom + CARRIER[2] / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def bottle(x, y, bottom=TOP, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array([BOTTLE_R, BOTTLE_R, BOTTLE_H / 2])
        pos = np.array([x, y, bottom + BOTTLE_H / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    cx, cy = CARRIER_START
    mx, my = MAT_XY
    v = (0.10, 0.0, 0.0)
    start = {n: bottle(*xy) for n, xy in BOTTLE_START.items()} | {"carrier": box(cx, cy, TOP)}
    loaded = {"bottle_1": bottle(cx, cy + SLOT_DY, TOP + CARRIER[2]), "bottle_2": bottle(cx, cy - SLOT_DY, TOP + CARRIER[2]),
              "carrier": box(cx, cy, TOP)}

    def riding(x, z=TOP + 0.07, vel=v):
        return {"bottle_1": bottle(x, cy + SLOT_DY, z + CARRIER[2], vel=vel), "bottle_2": bottle(x, cy - SLOT_DY, z + CARRIER[2], vel=vel),
                "carrier": box(x, cy, z, vel=vel)}

    mat_top = TOP + MAT[2]
    done = {"bottle_1": bottle(mx, my + SLOT_DY, mat_top + CARRIER[2]), "bottle_2": bottle(mx, my - SLOT_DY, mat_top + CARRIER[2]),
            "carrier": box(mx, my, mat_top)}
    hand_carried = start | {"bottle_1": bottle(0.29, 0.40, TOP + 0.06, vel=v)}
    one_in_mat = done | {"bottle_2": bottle(mx, my - SLOT_DY, mat_top + CARRIER[2])}

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "both loaded on the sending bench": ((start, loaded), 0.5, False),
        "carried across inside the carrier, done": ((start, loaded, riding(0.29), riding(0.40), done), 1.0, True),
        "done, carrier turned 90 degrees": ((start, loaded, riding(0.29), {
            "bottle_1": bottle(mx + 0.03 + SLOT_DY, my, mat_top + CARRIER[2]),
            "bottle_2": bottle(mx + 0.03 - SLOT_DY, my, mat_top + CARRIER[2]),
            "carrier": box(mx + 0.03, my, mat_top, yaw_deg=90)}), 1.0, True),
        "carrier turned 90 degrees, its end over the far tape": ((start, loaded, riding(0.29), {
            "bottle_1": bottle(mx + SLOT_DY, my, mat_top + CARRIER[2]), "bottle_2": bottle(mx - SLOT_DY, my, mat_top + CARRIER[2]),
            "carrier": box(mx, my, mat_top, yaw_deg=90)}), 0.5, False),
        "bottle 1 hand-carried over the corridor, then loaded": ((start, hand_carried, done), 0.5, False),
        "both hand-carried across, loaded on the mat": ((start, start | {"bottle_1": bottle(0.29, 0.4, TOP + 0.06, vel=v),
                                                                         "bottle_2": bottle(0.30, 0.46, TOP + 0.06, vel=v)}, done), 0.0, False),
        "bottle 2 fell out over the corridor": ((start, loaded, riding(0.29) | {"bottle_2": bottle(0.29, 0.155, TOP, tilt_deg=90)},
                                                  one_in_mat | {"bottle_2": bottle(0.29, 0.155, TOP, tilt_deg=90)}), 0.5, False),
        "carrier left in the corridor": ((start, loaded, riding(0.29, z=TOP, vel=(0, 0, 0))), 0.5, False),
        "carrier on the mat, empty": ((start, start | {"carrier": box(mx, my, mat_top)}), 0.0, False),
        "a new trial after a breach starts clean": ((start, hand_carried, start, loaded, riding(0.3), done), 1.0, True),
        "bottle tipped over in the carrier": ((start, loaded, done | {"bottle_1": bottle(mx, my + SLOT_DY, mat_top + CARRIER[2], 40)}),
                                              0.5, False),
        "bottle beside the carrier on the mat": ((start, loaded, done | {"bottle_1": bottle(mx, my + 0.11, mat_top)}), 0.5, False),
        "carrier still sliding": ((start, loaded, {k: (bottle if k != "carrier" else box)(*(
            (mx, my + (SLOT_DY if k == "bottle_1" else -SLOT_DY), mat_top + CARRIER[2]) if k != "carrier" else (mx, my, mat_top)),
            vel=(0.05, 0, 0)) for k in done}), 0.5, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not breached()
    return len(cases)


if __name__ == "__main__":
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
