"""On the station table, a 1 g bench analogue of passing flight hardware between two work envelopes around a delicate
instrument. A grey strut lies on a rack within only the left arm's top-down reach; its seat, a node pair with saddle walls
and end posts, is within only the right arm's. A transfer cradle on the midline is the handoff point. A free-standing
white housing, a star-tracker stand-in too wide to grasp, blocks the left arm's straight carry at the arms' carrying height,
and a fixed radiator wall blocks the right arm's. The housing records a push that slides it more than 10 mm or tips
it more than 5 degrees, and the score caps a seat won by shoving it.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/strut-handoff-keepout/strut_handoff_keepout.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Move the grey strut from the left rack to the right node pair, seating both ends between the saddle walls, without moving the white housing more than 10 mm." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
STRUT = (0.140, 0.020, 0.020)
RACK = (0.160, 0.360)
HOUSING = (0.130, 0.160)
HOUSING_SIZE = (0.100, 0.100, 0.180)  # wider than the 94 mm gripper opening, taller than the travel band
CRADLE = (0.280, 0.000)
SEAT = (0.260, -0.300)
WALL = (0.260, -0.150)

GREY = (0.75, 0.75, 0.78)
WHITE = (0.95, 0.95, 0.93)
DARK = (0.25, 0.25, 0.27)
SILVER = (0.80, 0.82, 0.85)
BLACK = (0.08, 0.08, 0.08)

RACK_MARK = Box("rack_mark", pos=(RACK[0], RACK[1], TOP + 0.0005), size=(0.160, 0.040, 0.001), color=BLACK)
STRUT_BOX = Box("strut", pos=(RACK[0], RACK[1], TOP + 0.001 + STRUT[2] / 2), size=STRUT, mass=0.05, color=GREY)
HOUSING_BOX = Box("housing", pos=(HOUSING[0], HOUSING[1], TOP + HOUSING_SIZE[2] / 2), size=HOUSING_SIZE, mass=0.40, color=WHITE)
CRADLE_PARTS = [
    Box("cradle_block", pos=(CRADLE[0], CRADLE[1], TOP + 0.010), size=(0.060, 0.040, 0.020), color=DARK),
    *[
        Box(f"cradle_post_{k}", pos=(CRADLE[0] + dx, CRADLE[1] + dy, TOP + 0.0175), size=(0.010, 0.006, 0.035), color=DARK)
        for k, (dx, dy) in enumerate(((0.055, 0.016), (0.055, -0.016), (-0.055, 0.016), (-0.055, -0.016)))
    ],
]
RADIATOR = Box("radiator_wall", pos=(WALL[0], WALL[1], TOP + 0.080), size=(0.010, 0.120, 0.160), color=SILVER)
NODES = [
    Cylinder("node_disc_near", pos=(0.190, SEAT[1], TOP + 0.005), radius=0.035, height=0.010, color=DARK),
    Cylinder("node_disc_far", pos=(0.330, SEAT[1], TOP + 0.005), radius=0.035, height=0.010, color=DARK),
    Cylinder("node_post_near", pos=(0.174, SEAT[1], TOP + 0.020), radius=0.012, height=0.040, color=DARK),
    Cylinder("node_post_far", pos=(0.346, SEAT[1], TOP + 0.020), radius=0.012, height=0.040, color=DARK),
    *[
        Box(f"saddle_{end}_{side}", pos=(x, SEAT[1] + dy, TOP + 0.025), size=(0.020, 0.010, 0.030), color=DARK)
        for end, x in (("near", 0.198), ("far", 0.322))
        for side, dy in (("l", 0.018), ("r", -0.018))
    ],
]
OBJECTS = [STRUT_BOX, HOUSING_BOX, RACK_MARK, *CRADLE_PARTS, RADIATOR, *NODES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

HOUSING_MOVE_TOL = 0.010
HOUSING_TILT_TOL = math.radians(5.0)
HOUSING_YAW_TOL = math.radians(5.0)
SEAT_XY_TOL = (0.005, 0.004)  # along x, across: 1 mm past the 4 mm end play and 3 mm side play
SEAT_BOTTOM = TOP + 0.010  # on the discs
CRADLE_XY_TOL = (0.020, 0.004)  # along x the strut stays on the block between the posts up to 20 mm
CRADLE_BOTTOM = TOP + 0.020  # on the block
BOTTOM_TOL = 0.003


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _along_x(state, tol) -> bool:
    """The strut's long axis horizontal and within ``tol`` of world x, either way round, from its quaternion."""
    ex = _rot(state.quat_xyzw)[:, 0]
    if abs(float(ex[2])) > math.sin(tol):
        return False
    off = abs(math.atan2(float(ex[1]), float(ex[0])))
    return min(off, math.pi - off) <= tol


def _resting_at(state, centre, xy_tol, bottom, yaw_tol) -> bool:
    dx, dy = abs(float(state.pos[0]) - centre[0]), abs(float(state.pos[1]) - centre[1])
    near = dx <= xy_tol[0] + 1e-9 and dy <= xy_tol[1] + 1e-9
    return bool(state.resting() and near and abs(float(state.low[2]) - bottom) <= BOTTOM_TOL and _along_x(state, yaw_tol))


def _seated(strut) -> bool:
    """Both ends on the discs, between the saddle walls and inside the end posts."""
    return _resting_at(strut, SEAT, SEAT_XY_TOL, SEAT_BOTTOM, math.radians(5.0))


def _cradled(strut) -> bool:
    """On the cradle block between its four posts."""
    return _resting_at(strut, CRADLE, CRADLE_XY_TOL, CRADLE_BOTTOM, math.radians(10.0))


def _undisturbed(housing) -> bool:
    """Centre within 10 mm of its start, vertical axis within 5 degrees of upright, turned less than 5 degrees about
    the vertical (a square footprint looks the same every 90 degrees)."""
    moved = math.hypot(float(housing.pos[0]) - HOUSING[0], float(housing.pos[1]) - HOUSING[1])
    r = _rot(housing.quat_xyzw)
    yaw = abs(math.atan2(float(r[1, 0]), float(r[0, 0]))) % (math.pi / 2)
    turned = min(yaw, math.pi / 2 - yaw) > HOUSING_YAW_TOL
    return moved <= HOUSING_MOVE_TOL and float(r[2, 2]) >= math.cos(HOUSING_TILT_TOL) and not turned


def success(objects) -> bool:
    """The strut seated and the housing undisturbed, at rest; holds for 1 s."""
    return _seated(objects["strut"]) and _undisturbed(objects["housing"])


def distance(objects) -> float:
    """Metres from the strut's centre to its seated centre."""
    s = objects["strut"]
    return math.hypot(float(s.pos[0]) - SEAT[0], float(s.pos[1]) - SEAT[1])


def score(objects) -> float:
    """1.0 seated with the housing undisturbed; 0.5 resting on the cradle with the housing undisturbed; 0.25 seated or
    cradled with the housing moved more than 10 mm or tilted more than 5 degrees; 0 for everything else."""
    strut, housing = objects["strut"], objects["housing"]
    seated, cradled, calm = _seated(strut), _cradled(strut), _undisturbed(housing)
    if seated and calm:
        return 1.0
    if cradled and calm:
        return 0.5
    if seated or cradled:
        return 0.25
    return 0.0


TRAVEL = TOP + 0.150
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the strut
GRIP = -(STRUT[1] - SQUEEZE) / 2  # finger joint target: each finger pushes 4 mm x 5000 N/m = 20 N, under its 50 N cap
RACK_GRIP_Z = TOP + 0.019  # TCP 8 mm above the strut centre, pad tips 5 mm above the rack mark
CRADLE_PLACE_Z = TOP + 0.044  # strut bottom 6 mm above the block
CRADLE_GRIP_Z = TOP + 0.038  # TCP 8 mm above the cradled strut's centre, pad tips 5 mm above the block
SEAT_PLACE_Z = TOP + 0.033  # strut bottom 5 mm above the discs
CRADLE_ENTRY = 0.020  # the pause above the cradle: strut bottom 11 mm above the 35 mm posts
SEAT_ENTRY = 0.035  # the pause above the seat: strut bottom 10 mm above the 40 mm saddle walls and end posts
CORRIDOR_X = 0.140  # the right arm's route around the radiator wall
DETOUR = (0.300, 0.360)  # the left arm's route around the housing
PARK_LEFT = (0.260, 0.330)


def _w(duration, xy, z, finger, label):
    return Waypoint(duration, (xy[0], xy[1], z), finger, label)


def _set_down(xy, z, entry, label):
    """Lower to ``entry`` above the place, settle, set in slowly, settle, release: the simulated arm's soft wrist
    trails a moving target by several millimetres, and the fixtures leave 3 mm per side."""
    return [
        _w(0.6, xy, TRAVEL, GRIP, f"settle over the {label}"),
        _w(1.2, xy, z + entry, GRIP, f"lower to the {label} entry"),
        _w(0.8, xy, z + entry, GRIP, "settle at the entry"),
        _w(2.0, xy, z, GRIP, f"set onto the {label}"),
        _w(0.8, xy, z, GRIP, "settle"),
        _w(1.5, xy, z, FINGER_OPEN, "release"),
    ]


def demo():
    """Left arm: rack to cradle around the housing, then park. Right arm: waits for the left arm to clear the
    midline, takes the strut off the cradle, carries it round the radiator wall along a corridor at x = 0.14 m and
    seats it. Every grip is top-down at yaw 0 (fingers across the strut's width) and closes only to 8 mm narrower
    than the strut, so it centres the strut instead of saturating; the gripper moves over 1.5 s, inside the default
    0.05 per step limiter. Left 18.5 s, right 38.4 s, then the 3 s hold: 828 steps at 20 Hz."""
    left = [
        _w(1.5, RACK, TRAVEL, FINGER_OPEN, "to the rack"),
        _w(1.5, RACK, RACK_GRIP_Z, FINGER_OPEN, "descend"),
        _w(0.6, RACK, RACK_GRIP_Z, FINGER_OPEN, "settle"),
        _w(1.5, RACK, RACK_GRIP_Z, GRIP, "close"),
        _w(1.2, RACK, TRAVEL, GRIP, "lift"),
        _w(1.0, DETOUR, TRAVEL, GRIP, "step out from the housing"),
        _w(1.8, CRADLE, TRAVEL, GRIP, "carry to the cradle"),
        *_set_down(CRADLE, CRADLE_PLACE_Z, CRADLE_ENTRY, "cradle"),
        _w(1.0, CRADLE, TRAVEL, FINGER_OPEN, "retreat"),
        _w(1.5, PARK_LEFT, TRAVEL, FINGER_OPEN, "park"),
    ]
    hover = (CORRIDOR_X, SEAT[1])
    corridor = (CORRIDOR_X, 0.000)
    right = [
        _w(1.5, hover, TRAVEL, FINGER_OPEN, "hover"),
        _w(17.0, hover, TRAVEL, FINGER_OPEN, "wait for the left arm to park"),
        _w(1.5, corridor, TRAVEL, FINGER_OPEN, "corridor in"),
        _w(1.2, CRADLE, TRAVEL, FINGER_OPEN, "over the cradle"),
        _w(0.6, CRADLE, TRAVEL, FINGER_OPEN, "settle"),
        _w(1.5, CRADLE, CRADLE_GRIP_Z, FINGER_OPEN, "descend"),
        _w(0.6, CRADLE, CRADLE_GRIP_Z, FINGER_OPEN, "settle"),
        _w(1.5, CRADLE, CRADLE_GRIP_Z, GRIP, "close"),
        _w(1.2, CRADLE, TRAVEL, GRIP, "lift"),
        _w(1.2, corridor, TRAVEL, GRIP, "back to the corridor"),
        _w(1.5, hover, TRAVEL, GRIP, "corridor out, clear of the wall"),
        _w(1.2, SEAT, TRAVEL, GRIP, "over the seat"),
        *_set_down(SEAT, SEAT_PLACE_Z, SEAT_ENTRY, "seat"),
        _w(1.0, SEAT, TRAVEL, FINGER_OPEN, "retreat"),
    ]
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, roll=0.0, pitch=0.0):  # yaw about z, then roll about the body's x, then pitch about its y
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        r = r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def body(size, x, y, bottom, q, vel=(0.0, 0.0, 0.0)):
        half = np.abs(_rot(q)) @ (np.asarray(size) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def strut(x, y, bottom, yaw=0.0, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        return body(STRUT, x, y, bottom, quat(yaw, roll, pitch), vel)

    def housing(dx=0.0, dy=0.0, tilt=0.0):
        return body(HOUSING_SIZE, HOUSING[0] + dx, HOUSING[1] + dy, TOP, quat(0.0, tilt))

    on_rack = strut(*RACK, TOP + 0.001)
    seated = strut(*SEAT, SEAT_BOTTOM)
    cradled = strut(*CRADLE, CRADLE_BOTTOM)
    cases = {
        "start": ({"strut": on_rack, "housing": housing()}, 0.0, False),
        "seated, housing untouched": ({"strut": seated, "housing": housing()}, 1.0, True),
        "cradled, housing untouched": ({"strut": cradled, "housing": housing()}, 0.5, False),
        "seated after pushing the housing 15 mm": ({"strut": seated, "housing": housing(dy=-0.015)}, 0.25, False),
        "cradled with the housing knocked over": ({"strut": cradled, "housing": housing(tilt=math.pi / 2)}, 0.25, False),
        "seated, housing nudged 8 mm and leaning 3 degrees": ({"strut": seated, "housing": housing(dx=0.008, tilt=math.radians(3))}, 1.0, True),
        "seated, housing leaning 7 degrees": ({"strut": seated, "housing": housing(tilt=math.radians(7))}, 0.25, False),
        "on the rack with the housing pushed": ({"strut": on_rack, "housing": housing(dx=0.03)}, 0.0, False),
        "seated 3 mm off along x": ({"strut": strut(SEAT[0] + 0.003, SEAT[1], SEAT_BOTTOM), "housing": housing()}, 1.0, True),
        "resting on the saddle wall tops": ({"strut": strut(*SEAT, TOP + 0.040), "housing": housing()}, 0.0, False),
        "held 10 mm above the discs, moving": ({"strut": strut(*SEAT, SEAT_BOTTOM + 0.010, vel=(0, 0, -0.03)), "housing": housing()}, 0.0, False),
        "over the seat, yawed 8 degrees": ({"strut": strut(*SEAT, SEAT_BOTTOM, yaw=math.radians(8)), "housing": housing()}, 0.0, False),
        "seated, rolled 90 degrees about its long axis": ({"strut": strut(*SEAT, SEAT_BOTTOM, roll=math.pi / 2), "housing": housing()}, 1.0, True),
        "seated, reversed end for end": ({"strut": strut(*SEAT, SEAT_BOTTOM, yaw=math.pi), "housing": housing()}, 1.0, True),
        "on the cradle, yawed 8 degrees": ({"strut": strut(*CRADLE, CRADLE_BOTTOM, yaw=math.radians(8)), "housing": housing()}, 0.5, False),
        "on the cradle, yawed 12 degrees": ({"strut": strut(*CRADLE, CRADLE_BOTTOM, yaw=math.radians(12)), "housing": housing()}, 0.0, False),
        "on the cradle, 12 mm along x": ({"strut": strut(CRADLE[0] + 0.012, CRADLE[1], CRADLE_BOTTOM), "housing": housing()}, 0.5, False),
        "on the cradle block, 25 mm along x, past a post pair": ({"strut": strut(CRADLE[0] + 0.025, CRADLE[1], CRADLE_BOTTOM), "housing": housing()}, 0.0, False),
        "seated against an end post": ({"strut": strut(SEAT[0] + 0.004, SEAT[1], SEAT_BOTTOM), "housing": housing()}, 1.0, True),
        "seated in a corner": ({"strut": strut(SEAT[0] + 0.004, SEAT[1] - 0.003, SEAT_BOTTOM), "housing": housing()}, 1.0, True),
        "seated, housing turned 20 degrees in place": ({"strut": seated, "housing": body(HOUSING_SIZE, HOUSING[0], HOUSING[1], TOP, quat(math.radians(20)))}, 0.25, False),
        "seated, housing turned 90 degrees in place": ({"strut": seated, "housing": body(HOUSING_SIZE, HOUSING[0], HOUSING[1], TOP, quat(math.pi / 2))}, 1.0, True),
        "on the table beside the cradle": ({"strut": strut(CRADLE[0], CRADLE[1] + 0.06, TOP), "housing": housing()}, 0.0, False),
        "on the table under the seat line, outside the node pair": ({"strut": strut(SEAT[0], SEAT[1] + 0.06, TOP), "housing": housing()}, 0.0, False),
        "standing on its end on a disc": ({"strut": strut(0.190, SEAT[1], SEAT_BOTTOM, pitch=math.pi / 2), "housing": housing()}, 0.0, False),
        "one end on a disc, the other on the table": ({"strut": strut(*SEAT, TOP, pitch=math.atan2(0.010, 0.140)), "housing": housing()}, 0.0, False),
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
