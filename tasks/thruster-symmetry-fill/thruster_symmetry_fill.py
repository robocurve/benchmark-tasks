"""On the station table, in the left arm's lane, a 1 g bench analogue of laying out reaction control thrusters:
thruster sets are laid out symmetric about both vehicle planes, so every thruster has a twin across each plane with
the same lever arm and either axis can be commanded without coupling into the other. A white jig plate carries
sixteen square nests in a 4 x 4 grid, split by two black centre lines, standing for those planes, that run between
the rows and the columns. Three gold thrusters are already fitted: a diagonal corner pair, which is point-symmetric
and looks balanced but is symmetric about neither centre line, and one inner thruster. Six loose thrusters of the
same kind stand on the table. Making the layout symmetric about both lines forces exactly five nests, the closure
of the fitted three under both reflections less the fitted three; no larger symmetric layout fits six loose
thrusters, as the next needs nine. Mirroring about either line alone, or completing only the inner set, fills three
of the five; mirroring each thruster once about each line fills four but leaves the layout symmetric about neither
line. The score counts the five forced nests that hold a seated thruster and subtracts every thruster seated
anywhere else, from each thruster's centre and axis.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/thruster-symmetry-fill/thruster_symmetry_fill.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Fit thrusters into empty nests so the layout becomes symmetric about both black centre lines, using as few thrusters as possible." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PLATE_CENTRE = (0.225, 0.300)  # the two centre lines cross here
PLATE = (0.250, 0.250, 0.006)  # spans x 0.100 to 0.350 and y 0.175 to 0.425
PLATE_TOP = TOP + PLATE[2]
NEST_XS = (0.1275, 0.1925, 0.2575, 0.3225)  # 65 mm pitch, mirror pairs about x = 0.225: index i and 3 - i
NEST_YS = (0.2025, 0.2675, 0.3325, 0.3975)  # mirror pairs about y = 0.300: index j and 3 - j
POCKET = 0.034  # square pocket inside the walls, 5 mm per side around a 24 mm thruster
WALL_T = 0.005
WALL_H = 0.008  # standing on the plate, so the wall tops are 14 mm above the table
WALL_OFF = POCKET / 2 + WALL_T / 2  # 19.5 mm, wall centre from the nest centre; 44 mm outside, 21 mm between nests
LINE_W = 0.015  # the centre lines, in the 21 mm gaps between nests, 3 mm clear of the walls each side
LINE_T = 0.001

THRUSTER_R = 0.012
THRUSTER_H = 0.040
THRUSTER_MASS = 0.025

FITTED = ((0, 0), (3, 3), (1, 1))  # (column i along x, row j along y): the point-symmetric corner pair and one inner thruster
REQUIRED = ((3, 0), (2, 1), (2, 2), (1, 2), (0, 3))  # the closure of FITTED under both reflections, less FITTED; the demo's order
OUTBOARD_Y = 0.490  # 65 mm left of the plate's left edge
INBOARD_Y = 0.120  # 55 mm right of the plate's right edge
LOOSE_START = (
    (0.260, INBOARD_Y), (0.200, INBOARD_Y), (0.140, INBOARD_Y),
    (0.250, OUTBOARD_Y), (0.190, OUTBOARD_Y), (0.130, OUTBOARD_Y),
)
DEMO_PICKS = (0, 1, 3, 4, 5)  # thruster_2 is the spare and stays on its mark

WHITE = (0.92, 0.92, 0.90)
WALL_GREY = (0.60, 0.60, 0.62)  # mid grey, so the walls cannot be mistaken for the black lines
BLACK = (0.05, 0.05, 0.05)
GOLD = (0.85, 0.65, 0.15)


def _nest_walls(i, j):
    cx, cy = NEST_XS[i], NEST_YS[j]
    z = PLATE_TOP + WALL_H / 2
    along_x = (POCKET + 2 * WALL_T, WALL_T, WALL_H)
    along_y = (WALL_T, POCKET, WALL_H)
    return [
        Box(f"nest_{i}{j}_wall_left", pos=(cx, cy + WALL_OFF, z), size=along_x, color=WALL_GREY),
        Box(f"nest_{i}{j}_wall_right", pos=(cx, cy - WALL_OFF, z), size=along_x, color=WALL_GREY),
        Box(f"nest_{i}{j}_wall_near", pos=(cx - WALL_OFF, cy, z), size=along_y, color=WALL_GREY),
        Box(f"nest_{i}{j}_wall_far", pos=(cx + WALL_OFF, cy, z), size=along_y, color=WALL_GREY),
    ]


JIG = [
    Box("jig_plate", pos=(*PLATE_CENTRE, TOP + PLATE[2] / 2), size=PLATE, color=WHITE),
    Box("centre_line_x", pos=(*PLATE_CENTRE, PLATE_TOP + LINE_T / 2), size=(PLATE[0], LINE_W, LINE_T), color=BLACK),
    Box("centre_line_y", pos=(*PLATE_CENTRE, PLATE_TOP + LINE_T / 2), size=(LINE_W, PLATE[1], LINE_T), color=BLACK),
    *[w for i in range(4) for j in range(4) for w in _nest_walls(i, j)],
]
FITTED_THRUSTERS = [
    Cylinder(f"fitted_{i}{j}", pos=(NEST_XS[i], NEST_YS[j], PLATE_TOP + THRUSTER_H / 2), radius=THRUSTER_R, height=THRUSTER_H, color=GOLD)
    for i, j in FITTED
]
THRUSTERS = [
    Cylinder(f"thruster_{k}", pos=(x, y, TOP + THRUSTER_H / 2), radius=THRUSTER_R, height=THRUSTER_H, mass=THRUSTER_MASS, color=GOLD)
    for k, (x, y) in enumerate(LOOSE_START)
]
OBJECTS = [*THRUSTERS, *FITTED_THRUSTERS, *JIG]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)  # a thruster leaning on a nest wall rests at more than 30 degrees
BOTTOM_TOL = 0.003  # bottom to the plate top
EPS = 1e-4


def _closure(cells):
    """Every cell together with its mirror images about both centre lines."""
    return {c for i, j in cells for c in ((i, j), (3 - i, j), (i, 3 - j), (3 - i, 3 - j))}


def _axis_z(state) -> float:
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)  # world z of the thruster's axis


def _bottom(state) -> float:
    """The thruster's lowest point from its centre and axis. The axis-aligned bounds of a tilted cylinder come from
    its box corners and read too low, so they are not used."""
    uz = _axis_z(state)
    return float(state.pos[2]) - (THRUSTER_H / 2 * abs(uz) + THRUSTER_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _seated_nest(state):
    """The (i, j) of the nest a thruster is seated in, or None: at rest, its axis within 10 degrees of vertical
    (either end up), its bottom within 3 mm of the plate top and its centre inside that nest's 34 x 34 mm pocket."""
    if not state.resting() or abs(_axis_z(state)) < math.cos(TILT_TOL):
        return None
    if abs(_bottom(state) - PLATE_TOP) > BOTTOM_TOL + EPS:
        return None
    x, y = float(state.pos[0]), float(state.pos[1])
    for i, cx in enumerate(NEST_XS):
        for j, cy in enumerate(NEST_YS):
            if abs(x - cx) <= POCKET / 2 + EPS and abs(y - cy) <= POCKET / 2 + EPS:
                return (i, j)
    return None


def _tally(objects):
    """R: the forced nests holding a seated thruster. X: thrusters seated in any other nest."""
    seated = [_seated_nest(objects[t.name]) for t in THRUSTERS]
    held = {n for n in seated if n is not None}
    r = sum(1 for n in REQUIRED if n in held)
    x = sum(1 for n in seated if n is not None and n not in REQUIRED)
    return r, x


def success(objects) -> bool:
    """All five forced nests filled and no thruster seated anywhere else; holds for 1 s."""
    r, x = _tally(objects)
    return r == len(REQUIRED) and x == 0


def distance(objects) -> float:
    """Metres still to go: for each forced nest, the nearest loose thruster's centre to the nest centre."""
    total = 0.0
    for i, j in REQUIRED:
        total += min(math.hypot(float(objects[t.name].pos[0]) - NEST_XS[i], float(objects[t.name].pos[1]) - NEST_YS[j]) for t in THRUSTERS)
    return total


def score(objects) -> float:
    """max(0, R - X) / 5: R forced nests filled, X thrusters seated in nests the symmetry does not force. 0, 0.2,
    0.4, 0.6, 0.8 or 1.0. The start state scores 0."""
    r, x = _tally(objects)
    return max(0, r - x) / len(REQUIRED)


TRAVEL = TOP + 0.145
GRIP_Z = TOP + 0.025  # TCP 15 mm under the thruster top; pad tips 12 mm above the table
PLACE_Z = PLATE_TOP + 0.005 + 0.025  # thruster bottom 5 mm above the plate; pad tips 9 mm above the nest walls
ENTRY = 0.015  # the pause above a nest: thruster bottom 12 mm above the wall tops
PLACE_YAW = math.pi / 4  # open fingers along the nest diagonal, 46 mm off every orthogonal neighbour
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Left arm only. Five top-down picks at yaw 0, the fingers closing along y so each thruster's row neighbours
    sit beside the pads; each place at yaw 45 degrees, so the open fingers lie along the nest diagonal, clear of the
    fitted thrusters; a carry at 14.5 cm, a pause with the thruster 12 mm above the walls and a drop of 5 mm onto the
    plate. The far corner first, the spare left on its mark. 81.0 s of plan; the episode ends on success about 1 s
    after the last release (the scripted policy would otherwise stop after a 3 s hold, at step 1,680 at 20 Hz)."""
    left = []
    for k, (i, j) in zip(DEMO_PICKS, REQUIRED):
        sx, sy = LOOSE_START[k]
        left += _pick_place((sx, sy, GRIP_Z), (NEST_XS[i], NEST_YS[j], PLACE_Z), width=2 * THRUSTER_R, entry=ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=PLACE_YAW)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    assert set(REQUIRED) == _closure(FITTED) - set(FITTED) and len(REQUIRED) == 5
    # Every nest's mirror orbit has four nests, so the next symmetric layout adds a whole orbit: 5 + 4 = 9 > 6 loose.
    assert all(len(_closure([(i, j)])) == 4 for i in range(4) for j in range(4))
    assert len(REQUIRED) + 4 > len(THRUSTERS)
    once = {(3, 0), (0, 3), (2, 1), (1, 2)}  # each fitted thruster mirrored once about each line, no double mirror
    layout = set(FITTED) | once
    assert {(3 - i, j) for i, j in layout} != layout and {(i, 3 - j) for i, j in layout} != layout
    assert all(abs(2 * PLATE_CENTRE[0] - NEST_XS[i] - NEST_XS[3 - i]) < 1e-12 for i in range(4))
    assert all(abs(2 * PLATE_CENTRE[1] - NEST_YS[j] - NEST_YS[3 - j]) < 1e-12 for j in range(4))

    def quat(yaw, tilt):  # tilt about the thruster's own x axis, then yaw about the vertical
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def thruster(x, y, bottom=TOP, tilt=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0)):
        """A thruster with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator
        reports them."""
        q = quat(yaw, tilt)
        xq, yq, zq, wq = q
        rot = np.array([[1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
                        [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
                        [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)]])
        half = np.abs(rot) @ np.array([THRUSTER_R, THRUSTER_R, THRUSTER_H / 2])
        pos = np.array([x, y, bottom + THRUSTER_H / 2 * abs(math.cos(tilt)) + THRUSTER_R * abs(math.sin(tilt))])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def nest(i, j, dx=0.0, dy=0.0, bottom=PLATE_TOP, **kw):
        return thruster(NEST_XS[i] + dx, NEST_YS[j] + dy, bottom=bottom, **kw)

    start = [thruster(x, y) for x, y in LOOSE_START]

    def scene(*placed):
        """Thrusters by index to the given states; the rest stay on their marks."""
        states = list(start)
        for k, st in enumerate(placed):
            states[k] = st
        return {t.name: st for t, st in zip(THRUSTERS, states)}

    done = [nest(i, j) for i, j in REQUIRED]
    gap_x = (NEST_XS[1] + NEST_XS[2]) / 2  # the gap the line along y runs in
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(*done), 1.0, True),
        "done with the spare used and another left on its mark": (scene(*done[:2], start[2], *done[2:4], done[4]), 1.0, True),
        "the corner pair taken as balanced, the inner set completed": (scene(nest(2, 1), nest(1, 2), nest(2, 2)), 0.6, False),
        "everything mirrored about the line at x 0.225 only": (scene(nest(3, 0), nest(0, 3), nest(2, 1)), 0.6, False),
        "everything mirrored about the line at y 0.300 only": (scene(nest(0, 3), nest(3, 0), nest(1, 2)), 0.6, False),
        "the corner pair taken as balanced, the inner one mirrored once": (scene(nest(2, 1)), 0.2, False),
        "each mirrored once about each line, the inner one's double mirror missed": (scene(nest(3, 0), nest(0, 3), nest(2, 1), nest(1, 2)), 0.8, False),
        "all six used, the spare in an unforced nest": (scene(*done, nest(0, 1)), 0.8, False),
        "four forced and the spare elsewhere": (scene(*done[:4], nest(1, 0)), 0.6, False),
        "three forced and two elsewhere": (scene(*done[:3], nest(3, 1), nest(3, 2)), 0.2, False),
        "two forced and three elsewhere": (scene(*done[:2], nest(0, 1), nest(0, 2), nest(1, 0)), 0.0, False),
        "five unforced nests": (scene(nest(0, 1), nest(0, 2), nest(1, 0), nest(2, 0), nest(3, 1)), 0.0, False),
        "six unforced nests": (scene(nest(0, 1), nest(0, 2), nest(1, 0), nest(2, 0), nest(3, 1), nest(1, 3)), 0.0, False),
        "done but one still moving": (scene(*done[:4], nest(0, 3, vel=(0.0, 0.0, -0.03))), 0.8, False),
        "done but one held 30 mm over its nest": (scene(*done[:4], nest(0, 3, bottom=PLATE_TOP + 0.030)), 0.8, False),
        "a thruster 4 mm off its nest centre counts": (scene(*done[:4], nest(0, 3, dx=0.004, dy=-0.004)), 1.0, True),
        "8 degrees of tilt still counts": (scene(*done[:4], nest(0, 3, dx=0.003, tilt=math.radians(8))), 1.0, True),
        "upside down counts": (scene(*done[:4], nest(0, 3, tilt=math.pi)), 1.0, True),
        "leaning 35 degrees on a wall in the pocket does not": (scene(*done[:4], nest(0, 3, dx=0.004, tilt=math.radians(35))), 0.8, False),
        "bottom 2.9 mm above the plate counts": (scene(*done[:4], nest(0, 3, bottom=PLATE_TOP + 0.0029)), 1.0, True),
        "bottom 3.5 mm above the plate does not": (scene(*done[:4], nest(0, 3, bottom=PLATE_TOP + 0.0035)), 0.8, False),
        "a thruster standing on the walls between two nests": (scene(*done[:4], thruster(gap_x, NEST_YS[3], bottom=PLATE_TOP + WALL_H)), 0.8, False),
        "a thruster lying across a forced nest": (scene(*done[:4], nest(0, 3, bottom=PLATE_TOP + WALL_H, tilt=math.pi / 2)), 0.8, False),
        "a thruster stood on a fitted one": (scene(*done[:4], nest(1, 1, bottom=PLATE_TOP + THRUSTER_H)), 0.8, False),
        "a thruster stood on a seated one does not add": (scene(*done, nest(3, 0, bottom=PLATE_TOP + THRUSTER_H)), 1.0, True),
        "a thruster on the table beside the plate": (scene(*done[:4], thruster(0.200, 0.160)), 0.8, False),
        "every loose thruster knocked over where it stood": (scene(*[thruster(x, y, tilt=math.pi / 2, yaw=0.5) for x, y in LOOSE_START]), 0.0, False),
        "a thruster off the table": (scene(*done[:4], thruster(0.950, 0.300, bottom=0.0, tilt=math.pi / 2)), 0.8, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not want_success or got == 1.0, name
    assert distance(scene(*done)) < 1e-9 and distance(scene()) > 0.0
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
