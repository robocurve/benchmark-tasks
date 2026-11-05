"""On the station table, a loose 50 cm DIN rail lies flat across the front of both arms, running left to right. Further
out stand two fixed rail standoffs, 46 cm apart, each topped by a cradle: two short walls that leave a channel 6 mm
wider than the rail. The rail must be lifted with both arms, kept level, and laid in both cradles at once, centred
between them. The rail is too long for one gripper to carry level. Scored from the rail's pose at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/din-rail-onto-standoffs/din_rail_onto_standoffs.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
RAIL = (0.035, 0.500, 0.0075)  # TS35 x 7.5, 50 cm long, modelled as its outline
RAIL_KG = 0.240
RAIL_START = (0.220, 0.0)
MOUNT_X = 0.340
CRADLE_Y = 0.230  # each cradle's centre, either side of the midline
POST = (0.030, 0.030, 0.040)  # standoff: 3 x 3 cm, 4 cm tall
POST_TOP = TOP + POST[2]
CHANNEL = RAIL[0] + 0.006  # 41 mm between the cradle walls
WALL = (0.006, 0.030, 0.008)  # cradle wall: along y, 8 mm above the standoff top

STEEL = (0.75, 0.76, 0.78)
DARK = (0.20, 0.20, 0.22)
ORANGE = (0.92, 0.48, 0.10)

THE_RAIL = Box("rail", pos=(*RAIL_START, TOP + RAIL[2] / 2), size=RAIL, mass=RAIL_KG, color=STEEL)
FIXTURES = []
for _side, _sy in (("left", 1.0), ("right", -1.0)):
    _y = _sy * CRADLE_Y
    FIXTURES += [
        Box(f"{_side}_standoff", pos=(MOUNT_X, _y, TOP + POST[2] / 2), size=POST, color=DARK),
        Box(f"{_side}_cradle_near", pos=(MOUNT_X - CHANNEL / 2 - WALL[0] / 2, _y, POST_TOP + WALL[2] / 2), size=WALL,
            color=ORANGE),
        Box(f"{_side}_cradle_far", pos=(MOUNT_X + CHANNEL / 2 + WALL[0] / 2, _y, POST_TOP + WALL[2] / 2), size=WALL,
            color=ORANGE),
    ]
OBJECTS = [THE_RAIL, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "best"  # credit for the furthest stage reached, so a rail that slides out late keeps its lift credit

SEAT_X = 0.003  # seated: centre within 3 mm of the cradles' line across x
SEAT_Y = 0.010  # and within 1 cm of the midline along the rail
SEAT_Z = 0.002  # both ends' bottoms within 2 mm of the standoff tops
SEAT_DEG = 3.0  # long side along y, and level


def _axis(quat_xyzw):
    """The rail's long axis (its local y) in the world frame."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])


def _ends(state):
    """World positions of the rail's two ends (the centres of its end faces)."""
    a = _axis(state.quat_xyzw)
    c = np.asarray(state.pos, float)
    return c + a * RAIL[1] / 2, c - a * RAIL[1] / 2


def _end_in_cradle(p, tol=0.003) -> bool:
    """An end over a standoff (within 3.5 cm of its centre along the rail, inside the channel across it), at the
    standoff's height: the end face's centre within ``tol`` of half a rail thickness above the standoff top."""
    return any(
        abs(p[0] - MOUNT_X) <= CHANNEL / 2
        and abs(p[1] - sy * CRADLE_Y) <= 0.035
        and abs(p[2] - (POST_TOP + RAIL[2] / 2)) <= tol
        for sy in (1.0, -1.0)
    )


def _seated(state) -> bool:
    a = _axis(state.quat_xyzw)
    off = math.degrees(math.acos(min(1.0, abs(float(a[1])))))
    return bool(
        abs(float(state.pos[0]) - MOUNT_X) <= SEAT_X
        and abs(float(state.pos[1])) <= SEAT_Y
        and all(_end_in_cradle(p, SEAT_Z) for p in _ends(state))
        and off <= SEAT_DEG
        and state.resting()
    )


def success(objects) -> bool:
    """The rail lies in both cradles, centred between them and square to them, still; holds for 1 s."""
    return _seated(objects["rail"])


def score(objects) -> float:
    """0.25 once the whole rail is 3 cm off the table; 0.5 with one end resting on a standoff in its cradle; 0.75 with
    both ends; 1.0 seated (centre within 3 mm of the cradle line and 1 cm of the midline, bottom within 2 mm of the
    standoff tops, square and level within 3 degrees, still). The best stage reached counts."""
    st = objects["rail"]
    if _seated(st):
        return 1.0
    n = sum(_end_in_cradle(p) for p in _ends(st))
    if n == 2 and st.resting():
        return 0.75
    if n >= 1 and st.resting():
        return 0.5
    if float(st.low[2]) >= TOP + 0.030:
        return 0.25
    return 0.0


def distance(objects) -> float:
    st = objects["rail"]
    return float(math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (MOUNT_X, 0.0, POST_TOP)))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, across the rail's 35 mm width
TIP = 0.013
TRAVEL = TOP + 0.110
SQUEEZE = 0.008
GRIP_Y = 0.190  # each arm holds the rail 19 cm from its centre, clear of the cradles
OPEN = -(RAIL[0] + 0.030) / 2
X_TRIM = 0.0055  # the held rail trails the commanded x by about 5.5 mm on the way down (measured in sim)


def demo():
    """Both arms at once, mirrored and in step so the rail stays level: grip it 19 cm either side of its centre, lift
    together to 11 cm, carry it out over the standoffs, lower it into both cradles, let go together. 18.2 s."""
    gz = TOP + 0.002 + TIP  # pad tips 2 mm above the table
    qz = POST_TOP + 0.0035 + TIP  # the rail let go 1.5 mm above the standoff tops
    grip = -(RAIL[0] - SQUEEZE) / 2
    plans = {}
    for side, sy in (("left", 1.0), ("right", -1.0)):
        px, py = RAIL_START[0], sy * GRIP_Y
        qx, qy = MOUNT_X + X_TRIM, sy * GRIP_Y
        plans[side] = [
            Waypoint(1.8, (px, py, TRAVEL), OPEN, "over the rail", YAW),
            Waypoint(1.5, (px, py, gz + 0.020), OPEN, "descend", YAW),
            Waypoint(1.0, (px, py, gz), OPEN, "down slowly", YAW),
            Waypoint(0.5, (px, py, gz), OPEN, "settle", YAW),
            Waypoint(1.5, (px, py, gz), grip, "close", YAW),
            Waypoint(2.0, (px, py, TRAVEL), grip, "lift together", YAW),
            Waypoint(2.5, (qx, qy, TRAVEL), grip, "carry out", YAW),
            Waypoint(0.8, (qx, qy, TRAVEL), grip, "settle over the cradles", YAW),
            Waypoint(2.0, (qx, qy, qz + 0.020), grip, "lower", YAW),
            Waypoint(1.5, (qx, qy, qz), grip, "into the cradles slowly", YAW),
            Waypoint(0.6, (qx, qy, qz), grip, "settle", YAW),
            Waypoint(1.5, (qx, qy, qz), OPEN, "let go together", YAW),
            Waypoint(1.0, (qx, qy, TRAVEL), OPEN, "retreat", YAW),
        ]
    return plans


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def rail(x, y, bottom, yaw_deg=0.0, roll_deg=0.0, vel=(0.0, 0.0, 0.0)):
        """Yawed about z, then rolled about x (one end up)."""
        yaw, roll = math.radians(yaw_deg), math.radians(roll_deg)
        cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
        q = np.array([cy * sr, sy * sr, sy * cr, cy * cr])
        half_z = RAIL[2] / 2 * math.cos(roll) + RAIL[1] / 2 * abs(math.sin(roll))
        pos = np.array([x, y, bottom + half_z])
        half = np.array([RAIL[0] / 2, RAIL[1] / 2, half_z])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def one_end_up():
        """The right end in its cradle, the rail tilted up over the left cradle's wall, resting."""
        roll = math.degrees(math.asin(0.012 / RAIL[1]))
        st = rail(MOUNT_X, 0.0, POST_TOP, roll_deg=roll)
        return st

    cases = {
        "start": ({"rail": rail(*RAIL_START, TOP)}, 0.0, False),
        "lifted 4 cm": ({"rail": rail(0.28, 0.0, TOP + 0.04, vel=(0.05, 0, 0))}, 0.25, False),
        "lifted 2 cm": ({"rail": rail(0.28, 0.0, TOP + 0.02, vel=(0.05, 0, 0))}, 0.0, False),
        "seated": ({"rail": rail(MOUNT_X, 0.0, POST_TOP)}, 1.0, True),
        "seated, 8 mm along": ({"rail": rail(MOUNT_X, 0.008, POST_TOP)}, 1.0, True),
        "both ends in, 1.5 cm along": ({"rail": rail(MOUNT_X, 0.015, POST_TOP)}, 0.75, False),
        "both ends in, turned 2 degrees": ({"rail": rail(MOUNT_X, 0.0, POST_TOP, yaw_deg=2)}, 1.0, True),
        "both ends in, turned 4 degrees": ({"rail": rail(MOUNT_X + 0.002, 0.0, POST_TOP, yaw_deg=4)}, 0.75, False),
        "one end in, the other up on the wall": ({"rail": one_end_up()}, 0.5, False),
        "seated but still sliding (lifted only)": ({"rail": rail(MOUNT_X, 0.0, POST_TOP, vel=(0.0, 0.03, 0.0))}, 0.25, False),
        "dropped on the table under the standoffs": ({"rail": rail(MOUNT_X - 0.05, 0.0, TOP)}, 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
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
