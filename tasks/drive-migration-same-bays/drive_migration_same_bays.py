"""On the station table, in front of the left arm, two top-loading server drive cages. The failed chassis, nearer the
arms, has four drive bays in a row along y, each holding a 2.5-inch drive in its carrier, standing on its connector
end; the drives carry four different label colours. The replacement chassis, further out, has the same four bays, all
empty, but it sits turned 180 degrees and shifted 2 cm: its front bezel is on its far side and its bay 1 is at the
opposite end. In both chassis the bays are numbered 1 to 4 from the end with the orange end wall. A RAID array
expects each member drive back in the same bay number, so each drive must go to the same-numbered bay of the new
chassis. Scored from the drive poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/drive-migration-same-bays/drive_migration_same_bays.py \\
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
DRIVE = (0.070, 0.018, 0.100)  # 70 mm wide (x), 18 mm thick with its carrier (y), 100 mm tall: standing on its end
DRIVE_KG = 0.120
BAYS = 4
PITCH = 0.045

FLOOR = (0.150, 0.230, 0.004)  # each chassis' floor plate
FLOOR_TOP = TOP + FLOOR[2]
OLD = (0.200, 0.215)  # chassis centres
NEW = (0.380, 0.235)
OLD_BAY_Y = {k: OLD[1] + (BAYS + 1 - 2 * k) * PITCH / 2 for k in range(1, BAYS + 1)}  # old bay 1 at the +y end
NEW_BAY_Y = {k: NEW[1] - (BAYS + 1 - 2 * k) * PITCH / 2 for k in range(1, BAYS + 1)}  # turned 180: bay 1 at the -y end

RAIL_T = 0.003  # guide walls either side of each bay
RAIL_H = 0.035
CLEAR = 0.0035  # air between the carrier and each guide wall
END = (0.070, 0.006, 0.040)  # end walls of the row, 35 mm outside the end bays' centres; orange at the bay-1 end
END_OFF = 0.035
BEZEL = (0.008, 0.200, 0.030)  # the front bezel, 65 mm in front of the bays' centreline
BEZEL_OFF = 0.065

LABELS = {1: (0.80, 0.15, 0.12), 2: (0.92, 0.78, 0.12), 3: (0.15, 0.35, 0.85), 4: (0.15, 0.62, 0.25)}  # red, yellow,
# blue, green carrier labels: they only tell the drives apart; the bay number is what matters
NAMES = {1: "drive_red", 2: "drive_yellow", 3: "drive_blue", 4: "drive_green"}  # by the old bay each starts in
GREY = (0.45, 0.46, 0.48)
LIGHT = (0.68, 0.69, 0.71)
DARK = (0.12, 0.12, 0.13)
ORANGE = (0.95, 0.45, 0.05)


def _chassis(name, centre, bay_y, front_sign, floor_color):
    """Floor, guides, end walls (orange beyond bay 1) and front bezel; ``front_sign`` -1 puts the bezel on the near
    side (x smaller), +1 on the far side."""
    cx, cy = centre
    off = DRIVE[1] / 2 + CLEAR + RAIL_T / 2
    parts = [Box(f"{name}_floor", pos=(cx, cy, TOP + FLOOR[2] / 2), size=FLOOR, color=floor_color)]
    for k, y in bay_y.items():
        for s, sgn in (("ypos", 1), ("yneg", -1)):
            parts.append(Box(f"{name}_bay{k}_guide_{s}", pos=(cx, y + sgn * off, FLOOR_TOP + RAIL_H / 2),
                             size=(DRIVE[0], RAIL_T, RAIL_H), color=DARK))
    out1 = 1.0 if bay_y[1] > bay_y[BAYS] else -1.0  # which way along y is "beyond bay 1"
    parts.append(Box(f"{name}_end_bay1", pos=(cx, bay_y[1] + out1 * END_OFF, FLOOR_TOP + END[2] / 2), size=END,
                     color=ORANGE))
    parts.append(Box(f"{name}_end_bay{BAYS}", pos=(cx, bay_y[BAYS] - out1 * END_OFF, FLOOR_TOP + END[2] / 2), size=END,
                     color=GREY))
    parts.append(Box(f"{name}_bezel", pos=(cx + front_sign * BEZEL_OFF, cy, FLOOR_TOP + BEZEL[2] / 2), size=BEZEL,
                     color=DARK))
    return parts


DRIVES = [Box(NAMES[k], pos=(OLD[0], OLD_BAY_Y[k], FLOOR_TOP + DRIVE[2] / 2), size=DRIVE, mass=DRIVE_KG,
              color=LABELS[k]) for k in range(1, BAYS + 1)]
FIXTURES = [*_chassis("old", OLD, OLD_BAY_Y, -1, GREY), *_chassis("new", NEW, NEW_BAY_Y, +1, LIGHT)]
OBJECTS = [*DRIVES, *FIXTURES]
BAY_OF = {NAMES[k]: k for k in NAMES}

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_Y = 0.005  # seated: centre within 5 mm of the bay's centreline (the guides allow 3.5 mm)
SEAT_X = 0.008  # within 8 mm of the bay centre along it
SEAT_Z = 0.003  # bottom within 3 mm of the floor (inside the guides, not on them)
SEAT_DEG = 6.0
UPRIGHT_DEG = 8.0
IN_Y, IN_X, IN_Z = 0.010, 0.040, 0.020  # loosely in a bay: enough to count a drive as put in the wrong bay


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the drive's up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """How far the drive is from square to the bays; a drive turned 180 degrees is square again."""
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _seated(state, bay) -> bool:
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[1]) - NEW_BAY_Y[bay]) <= SEAT_Y
        and abs(float(state.pos[0]) - NEW[0]) <= SEAT_X
        and abs(float(state.low[2]) - FLOOR_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= UPRIGHT_DEG
        and state.resting()
    )


def _in_bay(state, bay) -> bool:
    return bool(
        abs(float(state.pos[1]) - NEW_BAY_Y[bay]) <= IN_Y
        and abs(float(state.pos[0]) - NEW[0]) <= IN_X
        and float(state.low[2]) < FLOOR_TOP + IN_Z
    )


def placed(objects):
    """Drives seated in their own bay number, and drives in a new-chassis bay with another number."""
    right = [n for n, k in BAY_OF.items() if _seated(objects[n], k)]
    wrong = [n for n, k in BAY_OF.items() if any(_in_bay(objects[n], j) for j in NEW_BAY_Y if j != k)]
    return right, wrong


def success(objects) -> bool:
    """Every drive seated in the new chassis' bay with its old bay number; holds for 1 s."""
    right, wrong = placed(objects)
    return len(right) == BAYS and not wrong


def score(objects) -> float:
    """0.25 for each drive seated in the new chassis' bay with the same number as the bay it came from. A drive in a
    bay with another number earns nothing and caps the score at 0.25: the array would not assemble."""
    right, wrong = placed(objects)
    s = 0.25 * len(right)
    return min(s, 0.25) if wrong else s


def distance(objects) -> float:
    """Metres to go: each drive's centre to its own bay in the new chassis."""
    return float(sum(math.dist((float(objects[n].pos[0]), float(objects[n].pos[1]), float(objects[n].low[2])),
                               (NEW[0], NEW_BAY_Y[k], FLOOR_TOP)) for n, k in BAY_OF.items()))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = -math.pi / 2  # the fingers close along x onto the carrier's two narrow sides; this wrist turn is in reach
TIP = 0.013
TRAVEL = TOP + 0.150  # a held drive's bottom 5.8 cm above the floor: over the 3.5 cm guides, not over other drives
MID_X = 0.290  # between the two chassis: the held drive changes lane here, clear of every bay
SQUEEZE = 0.008
DEPTH = 0.025  # the pad tips 25 mm below the drive's top, 79 mm above the floor, far above the guides
X_TRIM = 0.003  # the held drive trails the commanded x by about 3.5 mm on the way down (measured in sim)
Y_TRIM_PER_M = 0.030  # and lands 3 mm towards the arm in y at the bay farthest across (measured): trim in proportion
Y_TRIM_ZERO = 0.290  # to how far across from y 0.29 the bay is


def _grip_z(bottom):
    return bottom + DRIVE[2] - DEPTH + TIP


def _move(pick, place, label):
    """Straight up out of the old bay with the fingers fully open (94 mm) across the 70 mm carrier, out along its own
    lane (so it never passes over another drive), across between the chassis, in along the new bay's lane and straight
    down, settling before every close and release and making the last 2 cm slowly. 20.7 s."""
    (px, py), (qx, qy) = pick, place
    qx, qy = qx + X_TRIM, qy + Y_TRIM_PER_M * (qy - Y_TRIM_ZERO)
    gz, qz = _grip_z(FLOOR_TOP), _grip_z(FLOOR_TOP) + 0.0015  # set down 1.5 mm above the floor
    grip = -(DRIVE[0] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{label}: over", YAW),
        Waypoint(1.8, (px, py, gz + 0.020), FINGER_OPEN, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.040), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(1.2, (MID_X, py, TRAVEL), grip, f"{label}: out along its lane", YAW),
        Waypoint(1.5, (MID_X, qy, TRAVEL), grip, f"{label}: across between the chassis", YAW),
        Waypoint(1.2, (qx, qy, TRAVEL), grip, f"{label}: in along the new lane", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.8, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set in slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release", YAW),
        Waypoint(1.2, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the drives in bay order, each from its old bay to the same-numbered bay of the turned chassis
    (old bay 1 at the +y end to new bay 1 at the -y end, and so on). 82.8 s."""
    plan = []
    for k in range(1, BAYS + 1):
        plan += _move((OLD[0], OLD_BAY_Y[k]), (NEW[0], NEW_BAY_Y[k]), f"bay {k} ({NAMES[k]})")
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def drive(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(DRIVE) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {NAMES[k]: drive(OLD[0], OLD_BAY_Y[k], FLOOR_TOP) for k in NAMES}

    def bay(k, dx=0.0, dy=0.0, dz=0.0, **kw):
        return drive(NEW[0] + dx, NEW_BAY_Y[k] + dy, FLOOR_TOP + dz, **kw)

    def scene(**kw):
        return start | kw

    r, yel, b, g = NAMES[1], NAMES[2], NAMES[3], NAMES[4]
    done = scene(**{r: bay(1), yel: bay(2), b: bay(3), g: bay(4)})
    mirrored = scene(**{r: bay(4), yel: bay(3), b: bay(2), g: bay(1)})  # copied as a straight translation
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, carriers turned 180 degrees": (scene(**{r: bay(1, yaw_deg=180), yel: bay(2), b: bay(3), g: bay(4, yaw_deg=180)}), 1.0, True),
        "bay 1 only": (scene(**{r: bay(1)}), 0.25, False),
        "bays 1 and 3": (scene(**{r: bay(1), b: bay(3)}), 0.5, False),
        "three right": (scene(**{r: bay(1), yel: bay(2), b: bay(3)}), 0.75, False),
        "copied by position, not bay number (all reversed)": (mirrored, 0.0, False),
        "two right, two swapped": (scene(**{r: bay(1), yel: bay(2), b: bay(4), g: bay(3)}), 0.25, False),
        "three right, the fourth in bay 4 of the old chassis still": (scene(**{r: bay(1), yel: bay(2), b: bay(3)}), 0.75, False),
        "bay 1 landed 6 mm short along the bay": (scene(**{r: bay(1, dx=-0.006)}), 0.25, False),
        "bay 1 12 mm off along the bay": (scene(**{r: bay(1, dx=-0.012)}), 0.0, False),
        "resting on top of the guides": (scene(**{r: bay(1, dz=RAIL_H)}), 0.0, False),
        "leaning 12 degrees in bay 1": (scene(**{r: bay(1, dz=0.003, tilt_deg=12)}), 0.0, False),
        "turned 10 degrees in bay 1": (scene(**{r: bay(1, yaw_deg=10)}), 0.0, False),
        "still moving in bay 1": (scene(**{r: bay(1, vel=(0.0, 0.0, -0.05))}), 0.0, False),
        "held 3 cm over bay 1": (scene(**{r: bay(1, dz=0.030)}), 0.0, False),
        "one lying on the new chassis floor": (scene(**{r: bay(1), yel: drive(NEW[0], 0.33, FLOOR_TOP, 0, 90)}), 0.25, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(done) < 1e-9 and distance(start) > 0.5
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
