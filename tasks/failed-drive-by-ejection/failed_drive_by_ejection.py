"""On the station table, in front of the left arm, a top-open server drive cage lies with its four bays in a row along
y, each a pair of low dark guide rails on a grey floor plate running along x to a green backplane bar at the far end.
Every bay holds a 2.5-inch drive in its carrier, standing on its long edge; all four carriers look the same. The
storage controller has failed one drive and released its latch, so that carrier stands 30 mm proud of the others
(nearer the arms, out past the front ends of the guides); the seated drives sit with their back ends against the
backplane. A white spares tray nearer the arms holds one spare drive, and a black return bin stands beside it. The
robot must find the released drive by its offset alone, put it in the return bin, seat the spare fully home in that
same bay, and leave the three seated drives untouched. Scored from the drive poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/failed-drive-by-ejection/failed_drive_by_ejection.py \\
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
DRIVE = (0.090, 0.024, 0.070)  # 90 mm long (x, the insertion axis), 24 mm thick with its carrier (y), 70 mm tall
DRIVE_KG = 0.150
BAYS = 4
PITCH = 0.052
CAGE_Y = 0.250
BAY_Y = {k: CAGE_Y + (BAYS + 1 - 2 * k) * PITCH / 2 for k in range(1, BAYS + 1)}  # bay 1 at the left (largest y)
HOME_X = 0.330  # a seated drive's centre: its back end against the backplane
EJECT = 0.030  # the released drive stands 30 mm proud, towards the arms
FAILED = 3  # the bay whose drive the controller released; nothing but the offset marks it

FLOOR = (0.150, 0.210, 0.004)
FLOOR_X = HOME_X - 0.018
FLOOR_TOP = TOP + FLOOR[2]
RAIL_T, RAIL_H = 0.003, 0.020  # guide rails either side of each bay, as long as a seated drive
CLEAR = 0.0045  # air between the carrier and each rail
BACKPLANE = (0.006, 0.200, 0.030)
BACK_X = HOME_X + DRIVE[0] / 2 + BACKPLANE[0] / 2 + 0.0005  # 0.5 mm behind a seated drive's back end

TRAY_T, TRAY_H = 0.006, 0.015
SPARE_TRAY = ((0.170, 0.310), (0.120, 0.070))  # (centre, inner size)
RETURN_BIN = ((0.170, 0.200), (0.130, 0.080))

METAL = (0.55, 0.57, 0.60)
DARK = (0.12, 0.12, 0.13)
LIGHT = (0.74, 0.75, 0.77)
PCB = (0.10, 0.42, 0.22)
WHITE = (0.92, 0.92, 0.92)
BLACK = (0.06, 0.06, 0.06)


def _start_x(k):
    return HOME_X - EJECT if k == FAILED else HOME_X


def _frame(name, centre, inner, color):
    (cx, cy), (ix, iy) = centre, inner
    z = TOP + TRAY_H / 2
    t = TRAY_T
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, TRAY_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, TRAY_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, TRAY_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, TRAY_H), color=color),
    ]


def _cage():
    off = DRIVE[1] / 2 + CLEAR + RAIL_T / 2
    parts = [Box("cage_floor", pos=(FLOOR_X, CAGE_Y, TOP + FLOOR[2] / 2), size=FLOOR, color=LIGHT),
             Box("backplane", pos=(BACK_X, CAGE_Y, FLOOR_TOP + BACKPLANE[2] / 2), size=BACKPLANE, color=PCB)]
    for k, y in BAY_Y.items():
        for s, sgn in (("ypos", 1), ("yneg", -1)):
            parts.append(Box(f"bay{k}_rail_{s}", pos=(HOME_X, y + sgn * off, FLOOR_TOP + RAIL_H / 2),
                             size=(DRIVE[0], RAIL_T, RAIL_H), color=DARK))
    return parts


DRIVES = [Box(f"drive_{k}", pos=(_start_x(k), BAY_Y[k], FLOOR_TOP + DRIVE[2] / 2), size=DRIVE, mass=DRIVE_KG,
              color=METAL) for k in range(1, BAYS + 1)]
SPARE = Box("spare_drive", pos=(*SPARE_TRAY[0], TOP + DRIVE[2] / 2), size=DRIVE, mass=DRIVE_KG, color=METAL)
FIXTURES = [*_cage(), *_frame("spare_tray", *SPARE_TRAY, WHITE), *_frame("return_bin", *RETURN_BIN, BLACK)]
OBJECTS = [*DRIVES, SPARE, *FIXTURES]
HEALTHY = [d for d in DRIVES if d.name != f"drive_{FAILED}"]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_Y = 0.005  # seated: centre within 5 mm of the bay's centreline (the rails allow 4.5 mm)
SEAT_X = 0.005  # and within 5 mm of fully home (the released drive starts 30 mm out)
SEAT_Z = 0.003  # bottom within 3 mm of the floor plate
SEAT_DEG = 6.0
UPRIGHT_DEG = 8.0
KEEP_MOVE, KEEP_DEG = 0.004, 5.0  # a seated healthy drive counts as untouched within 4 mm and 5 degrees of its start


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """How far a carrier is from square to the bays; one turned 180 degrees is square again."""
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _seated(st, bay) -> bool:
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        abs(float(st.pos[1]) - BAY_Y[bay]) <= SEAT_Y
        and abs(float(st.pos[0]) - HOME_X) <= SEAT_X
        and abs(float(st.low[2]) - FLOOR_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= UPRIGHT_DEG
        and st.resting()
    )


def _in_bin(st) -> bool:
    """At rest, the centre inside the return bin's inner walls, the bottom within 2 cm of the table."""
    (cx, cy), (ix, iy) = RETURN_BIN
    return bool(
        abs(float(st.pos[0]) - cx) < ix / 2
        and abs(float(st.pos[1]) - cy) < iy / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.020
        and st.resting()
    )


def _untouched(objects) -> bool:
    for d in HEALTHY:
        st = objects[d.name]
        yaw, tilt = _yaw_tilt(st.quat_xyzw)
        if math.dist([float(v) for v in st.pos], d.pos) > KEEP_MOVE or _square(yaw) > KEEP_DEG or tilt > KEEP_DEG:
            return False
    return True


def score(objects) -> float:
    """0.5 for the released (failed) drive at rest in the return bin, 0.5 for the spare seated fully home in the bay it
    came from. Any of the three seated drives moved more than 4 mm or turned or tipped more than 5 degrees caps the
    score at 0.25: pulling a healthy array member degrades the array."""
    s = 0.5 * _in_bin(objects[f"drive_{FAILED}"]) + 0.5 * _seated(objects[SPARE.name], FAILED)
    if not _untouched(objects):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """The failed drive in the return bin, the spare seated home in its bay, the healthy drives untouched; holds 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the failed drive to the return bin's centre plus the spare to its seat."""
    f, sp = objects[f"drive_{FAILED}"], objects[SPARE.name]
    d = math.hypot(float(f.pos[0]) - RETURN_BIN[0][0], float(f.pos[1]) - RETURN_BIN[0][1])
    d += math.dist((float(sp.pos[0]), float(sp.pos[1]), float(sp.low[2])), (HOME_X, BAY_Y[FAILED], FLOOR_TOP))
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = 0.0  # the fingers close along y onto the carrier's two broad faces; the pads come down in the gaps between drives
TIP = 0.013
TRAVEL = TOP + 0.170  # a held drive's bottom 3.8 cm above the other drives' tops
SQUEEZE = 0.008
GRIP_DEPTH = 0.025  # the pad tips 25 mm below the carrier's top, 25 mm above the rails
NARROW = -0.020  # open to 40 mm only: the pads clear the carrier by 8 mm and the neighbouring drives by 10 mm
SET_GAP = 0.0015  # set the spare down 1.5 mm short of the backplane
X_TRIM = 0.002  # a carried 150 g drive trails the commanded x by about 3 mm on the way down: trim towards the backplane


def _grip_z(bottom):
    return bottom + DRIVE[2] - GRIP_DEPTH + TIP


def _move(pick, place, label, trim=0.0):
    """Straight up, carry high, straight down, the fingers never wider than 40 mm, settling before every close and
    release and making the last 2 cm slowly. 18.9 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += trim
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.0015  # set down 1.5 mm above the surface
    grip = -(DRIVE[1] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), NARROW, f"{label}: over", YAW),
        Waypoint(1.8, (px, py, gz + 0.020), NARROW, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), NARROW, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), NARROW, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.040), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.8, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set in slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), NARROW, f"{label}: release", YAW),
        Waypoint(1.6, (qx, qy, TRAVEL), NARROW, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the released drive in bay 3 (30 mm proud) straight up and into the return bin, then the spare
    from its tray straight down into bay 3, home against the backplane. 37.8 s."""
    fx = _start_x(FAILED)
    plan = _move((fx, BAY_Y[FAILED], FLOOR_TOP), (*RETURN_BIN[0], TOP), "released drive to the return bin")
    plan += _move((*SPARE_TRAY[0], TOP), (HOME_X - SET_GAP, BAY_Y[FAILED], FLOOR_TOP), "spare into bay 3", trim=X_TRIM)
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
        half = np.array(DRIVE) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {f"drive_{k}": box(_start_x(k), BAY_Y[k], FLOOR_TOP) for k in BAY_Y}
    start[SPARE.name] = box(*SPARE_TRAY[0], TOP)
    binned = {f"drive_{FAILED}": box(*RETURN_BIN[0], TOP)}
    seated = {SPARE.name: box(HOME_X - 0.003, BAY_Y[FAILED] + 0.002, FLOOR_TOP, yaw_deg=2.0)}
    done = start | binned | seated
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, spare turned 180 degrees": (done | {SPARE.name: box(HOME_X, BAY_Y[FAILED], FLOOR_TOP, 180.0)}, 1.0, True),
        "failed drive binned only": (start | binned, 0.5, False),
        "failed drive lying in the bin": (start | {f"drive_{FAILED}": box(0.17, 0.20, TOP, 0, 90)}, 0.5, False),
        "failed drive pushed home instead of replaced": (start | {f"drive_{FAILED}": box(HOME_X, BAY_Y[FAILED], FLOOR_TOP)}, 0.0, False),
        "spare seated but the failed drive left on the table": (start | seated | {f"drive_{FAILED}": box(0.20, 0.10, TOP)}, 0.5, False),
        "spare left 8 mm proud": (start | binned | {SPARE.name: box(HOME_X - 0.008, BAY_Y[FAILED], FLOOR_TOP)}, 0.5, False),
        "spare resting on the rails": (start | binned | {SPARE.name: box(HOME_X, BAY_Y[FAILED], FLOOR_TOP + RAIL_H)}, 0.5, False),
        "spare leaning 12 degrees": (start | binned | {SPARE.name: box(HOME_X, BAY_Y[FAILED], FLOOR_TOP + 0.003, tilt_deg=12)}, 0.5, False),
        "spare still moving": (start | binned | {SPARE.name: box(HOME_X, BAY_Y[FAILED], FLOOR_TOP, vel=(0, 0, -0.05))}, 0.5, False),
        "healthy drive 2 pulled into the bin instead": (start | {"drive_2": box(*RETURN_BIN[0], TOP)}, 0.0, False),
        "done but drive 4 knocked 5 mm out": (done | {"drive_4": box(HOME_X - 0.005, BAY_Y[4], FLOOR_TOP)}, 0.25, False),
        "done but drive 2 tipped over": (done | {"drive_2": box(HOME_X, BAY_Y[2] - 0.03, FLOOR_TOP, 0, 90)}, 0.25, False),
        "done, drive 1 nudged 2 mm": (done | {"drive_1": box(HOME_X - 0.002, BAY_Y[1], FLOOR_TOP)}, 1.0, True),
        "failed drive still held over the bin": (start | {f"drive_{FAILED}": box(*RETURN_BIN[0], TOP + 0.05)}, 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(done) < 0.01 and distance(start) > 0.2
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
