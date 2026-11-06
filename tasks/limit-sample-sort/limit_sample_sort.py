"""On the station table, in the left arm's lane, a 1 g bench analogue of inspection against a limit-sample board: a
board of accepted and rejected examples shows the criterion instead of stating it, and the inspector sorts the lot
by the same rule. A fixed board, its outboard half green and its inboard half red, carries five fixed examples: three
on the green half and two on the red half, grey and blue, boxes and cylinders. Four loose parts in white and yellow,
colours that never appear on the board, stand in a row. A green tray and a red tray, each four low walls round an
open pocket, wait either side of the row. The score counts each loose part that rests inside the tray the board's
rule calls for and subtracts each part in the other tray, so sorting by colour, by shape, by nearest tray, by
alternating trays or all into one tray earns nothing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/limit-sample-sort/limit_sample_sort.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "The sample board shows accepted parts on its green half and rejected parts on its red half. Sort the four loose parts by the same rule: accepted into the green tray, rejected into the red tray." \\
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
WIDTH = 0.030  # every part and example: a 30 mm square box or a 30 mm diameter cylinder
RADIUS = WIDTH / 2
SHORT = 0.035  # accepted on the board
TALL = 0.070  # rejected on the board; height is the only attribute that splits the five examples

BOARD_X = 0.340  # board x 0.300 to 0.380, 14 mm beyond the trays' far walls
HALF = (0.080, 0.150, 0.006)  # each half of the board, along x, along y, thick
GREEN_HALF_Y = 0.375  # the outboard half, y 0.300 to 0.450
RED_HALF_Y = 0.225  # the inboard half, y 0.150 to 0.300
BOARD_TOP = TOP + HALF[2]

INNER = (0.080, 0.120)  # each tray's open pocket, along x and along y; the floor is the table
WALL_T = 0.006
WALL_H = 0.010
GREEN_TRAY = (0.240, 0.470)  # pocket x 0.200 to 0.280, y 0.410 to 0.530
RED_TRAY = (0.240, 0.140)  # pocket x 0.200 to 0.280, y 0.080 to 0.200
LOW = 0.004  # a part counts with its lowest point under 4 mm above the table: on the pocket floor, not on a wall, on a
# part or still in the fingers at the 5 mm release height
SINK = 0.005  # and no more than 5 mm under the table top, so a part on the floor under a pocket does not count

ROW_X = 0.150

GREY = (0.62, 0.62, 0.64)
BLUE = (0.15, 0.35, 0.85)
WHITE = (0.92, 0.92, 0.90)
YELLOW = (0.95, 0.80, 0.10)
GREEN = (0.15, 0.65, 0.25)
RED = (0.85, 0.12, 0.12)


def _part(name, kind, y, height, color, x=ROW_X, base=TOP, mass=None):
    pos = (x, y, base + height / 2)
    if kind == "box":
        return Box(name, pos=pos, size=(WIDTH, WIDTH, height), mass=mass, color=color)
    return Cylinder(name, pos=pos, radius=RADIUS, height=height, mass=mass, color=color)


def _tray(name, centre, color):
    cx, cy = centre
    ix, iy = INNER
    z = TOP + WALL_H / 2
    side = (ix + 2 * WALL_T, WALL_T, WALL_H)
    end = (WALL_T, iy, WALL_H)
    return [
        Box(f"{name}_wall_out", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_in", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=end, color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=end, color=color),
    ]


# The loose parts, outboard to inboard: accept, reject, reject, accept. Each colour and each shape has one of each,
# and neither the nearest tray, nor alternating trays from either end, nor the board half straight ahead sorts them.
# One of a row's three two-and-two splits must match: here the two end parts against the middle two.
PARTS = [
    _part("part_white_box", "box", 0.395, SHORT, WHITE, mass=0.030),
    _part("part_yellow_box", "box", 0.335, TALL, YELLOW, mass=0.060),
    _part("part_white_cyl", "cyl", 0.275, TALL, WHITE, mass=0.050),
    _part("part_yellow_cyl", "cyl", 0.215, SHORT, YELLOW, mass=0.025),
]
HEIGHT = {p.name: (p.size[2] if isinstance(p, Box) else p.height) for p in PARTS}
KIND = {p.name: ("box" if isinstance(p, Box) else "cyl") for p in PARTS}
STARTS = {p.name: (p.pos[0], p.pos[1]) for p in PARTS}

BOARD = [
    Box("board_green", pos=(BOARD_X, GREEN_HALF_Y, TOP + HALF[2] / 2), size=HALF, color=GREEN),
    Box("board_red", pos=(BOARD_X, RED_HALF_Y, TOP + HALF[2] / 2), size=HALF, color=RED),
    # Accepted, on the green half: both colours, both shapes, all 35 mm tall.
    _part("sample_grey_cyl_short", "cyl", 0.330, SHORT, GREY, x=BOARD_X, base=BOARD_TOP),
    _part("sample_blue_box_short", "box", 0.375, SHORT, BLUE, x=BOARD_X, base=BOARD_TOP),
    _part("sample_grey_box_short", "box", 0.420, SHORT, GREY, x=BOARD_X, base=BOARD_TOP),
    # Rejected, on the red half: both colours, both shapes, both 70 mm tall.
    _part("sample_blue_box_tall", "box", 0.195, TALL, BLUE, x=BOARD_X, base=BOARD_TOP),
    _part("sample_grey_cyl_tall", "cyl", 0.255, TALL, GREY, x=BOARD_X, base=BOARD_TOP),
]
TRAYS = [*_tray("green_tray", GREEN_TRAY, GREEN), *_tray("red_tray", RED_TRAY, RED)]
OBJECTS = [*PARTS, *BOARD, *TRAYS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _target(name):
    """The tray the board's rule calls for: 35 mm parts were accepted, 70 mm parts rejected."""
    return GREEN_TRAY if HEIGHT[name] < (SHORT + TALL) / 2 else RED_TRAY


def _other(name):
    return RED_TRAY if _target(name) is GREEN_TRAY else GREEN_TRAY


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _bottom(state, name) -> float:
    """The part's lowest point from its centre and quaternion, standing, lying or leaning. A box's lowest corner;
    a cylinder's lowest rim point. The axis-aligned bounds of a lying cylinder come from its box corners and read up
    to r(sqrt 2 - 1) too low, so they are not used."""
    rz = _rot(state.quat_xyzw)[2]  # world z of each body axis
    h = HEIGHT[name]
    if KIND[name] == "box":
        drop = (abs(rz[0]) + abs(rz[1])) * RADIUS + abs(rz[2]) * h / 2
    else:
        drop = abs(rz[2]) * h / 2 + RADIUS * math.sqrt(max(0.0, 1.0 - rz[2] * rz[2]))
    return float(state.pos[2]) - drop


def _gap(state, centre) -> float:
    """Horizontal distance from the part's centre to a tray's pocket outline, 0 when the centre is inside it."""
    dx = max(0.0, abs(float(state.pos[0]) - centre[0]) - INNER[0] / 2)
    dy = max(0.0, abs(float(state.pos[1]) - centre[1]) - INNER[1] / 2)
    return math.hypot(dx, dy)


def _in_tray(state, name, centre) -> bool:
    """At rest, the centre strictly inside the pocket outline seen from above, and the lowest point from 5 mm under
    the table top to under 4 mm above it: standing, lying or leaning on the pocket floor, but not on a wall top, on
    another part, held above the pocket (the demo releases 5 mm up), still moving, or on the floor under the open
    table."""
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - centre[0]) < INNER[0] / 2 and abs(y - centre[1]) < INNER[1] / 2
    return bool(inside and state.resting() and TOP - SINK < _bottom(state, name) < TOP + LOW)


def _tally(objects):
    """C, parts in the tray their height calls for; W, parts in the other tray."""
    correct = wrong = 0
    for p in PARTS:
        st = objects[p.name]
        if _in_tray(st, p.name, _target(p.name)):
            correct += 1
        elif _in_tray(st, p.name, _other(p.name)):
            wrong += 1
    return correct, wrong


def success(objects) -> bool:
    """All four parts in the right trays, at rest; holds for 1 s."""
    correct, _ = _tally(objects)
    return correct == len(PARTS)


def distance(objects) -> float:
    """Metres still to go: each part's centre to the pocket outline of the tray its height calls for."""
    return sum(_gap(objects[p.name], _target(p.name)) for p in PARTS)


def score(objects) -> float:
    """Net sort: max(0, C - W) / 4, so 0, 0.25, 0.5, 0.75 or 1.0. Sorting by colour or by shape, either way round,
    gives C = 2 and W = 2, and so does every part in one tray: 0."""
    correct, wrong = _tally(objects)
    return max(0, correct - wrong) / len(PARTS)


TRAVEL = TOP + 0.145
GRIP_Z = {SHORT: TOP + 0.027, TALL: TOP + 0.055}  # pad tips 14 mm up on a short part; 15 mm under a tall part's top
PLACE_Z = {SHORT: TOP + 0.032, TALL: TOP + 0.060}  # part bottom 5 mm above the pocket floor; short: pad tips 9 mm over the walls
ENTRY = 0.015  # the pause above a pocket: part bottom 10 mm above the 10 mm walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part

# Each move: part, place, yaw at the pick, yaw at the place. Yaw +-90 degrees closes the fingers along x, so a
# neighbour standing along y sits beside the pads; yaw 0 closes them along y, used only where no neighbour stands
# along y. Joint 6 runs at about joint 1 minus the yaw and must stay inside +-120 degrees, so -90 works only inboard
# of the base and +90 only outboard; no turn within or between moves reaches 180 degrees.
Q = math.pi / 2
ORDER = (  # tall parts first, so no short part is gripped beside a taller neighbour
    ("part_white_cyl", (0.240, 0.110), -Q, -Q),
    ("part_yellow_box", (0.240, 0.170), -Q, -Q),  # beside the white cylinder along y
    ("part_white_box", (0.240, 0.500), 0.0, 0.0),  # nothing along y at the pick or in the green tray yet
    ("part_yellow_cyl", (0.240, 0.440), 0.0, Q),  # the last part in the row; beside the white box along y at the place
)


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
    """Left arm only. The two 70 mm parts go standing into the red tray, then the two 35 mm parts into the green
    tray, each a top-down pick, a carry at 14.5 cm, a pause 10 mm above the walls and a drop of 5 mm onto the pocket
    floor. 64.8 s of plan, then the 3 s hold: 1,356 steps at 20 Hz."""
    left = []
    for name, (qx, qy), yaw, place_yaw in ORDER:
        sx, sy = STARTS[name]
        h = HEIGHT[name]
        left += _pick_place((sx, sy, GRIP_Z[h]), (qx, qy, PLACE_Z[h]), width=WIDTH, entry=ENTRY, travel_z=TRAVEL, yaw=yaw, place_yaw=place_yaw)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw, tilt):  # tilt about the part's own x axis, then yaw about the vertical
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def part(name, x, y, bottom=TOP, tilt=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0)):
        """A part with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator
        reports them."""
        q = quat(yaw, tilt)
        rot = _rot(q)
        h = HEIGHT[name]
        half = np.array([RADIUS, RADIUS, h / 2])
        drop = RADIUS * abs(math.sin(tilt)) + h / 2 * abs(math.cos(tilt))  # box or cylinder, tilted about one body axis
        pos = np.array([x, y, bottom + drop])
        ext = np.abs(rot) @ half
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - ext, pos + ext)

    names = [p.name for p in PARTS]
    WB, YB, WC, YC = names
    start = {n: part(n, *STARTS[n]) for n in names}
    green_spots = ((0.240, 0.440), (0.240, 0.500), (0.220, 0.470), (0.260, 0.470))
    red_spots = ((0.240, 0.110), (0.240, 0.170), (0.220, 0.140), (0.260, 0.140))

    def scene(green=(), red=(), extra=()):
        """``green`` and ``red`` list the parts standing in each tray; the rest stay on their marks; ``extra``
        replaces single parts by name."""
        states = dict(start)
        for n, spot in zip(green, green_spots):
            states[n] = part(n, *spot)
        for n, spot in zip(red, red_spots):
            states[n] = part(n, *spot)
        for n, st in extra:
            states[n] = st
        return states

    lying_in_green = part(YC, 0.240, 0.470, tilt=math.pi / 2, yaw=0.4)
    leaning_in_red = part(YB, 0.268, 0.140, tilt=math.radians(20), yaw=math.pi / 2)  # top against the far wall
    lying_tall_in_red = part(WC, 0.240, 0.140, tilt=math.pi / 2, yaw=math.pi / 2)  # 70 mm along x in the 80 mm pocket
    on_green_wall = part(WB, 0.240, GREEN_TRAY[1] + INNER[1] / 2 + WALL_T / 2, bottom=TOP + WALL_H)
    on_part_in_green = part(YC, 0.240, 0.440, bottom=TOP + SHORT)  # on top of the white box
    on_part_lying = part(YC, 0.240, 0.470, bottom=TOP + 2 * RADIUS)  # on a part lying in the pocket
    held_over_green = part(WB, 0.240, 0.500, bottom=TOP + 0.030, vel=(0.0, 0.0, 0.03))
    held_still_low = part(WB, 0.240, 0.500, bottom=TOP + 0.012)  # motionless, but 12 mm above the floor
    held_at_release = part(WB, 0.240, 0.500, bottom=TOP + 0.005)  # motionless in the fingers at the release height
    on_board = part(WB, BOARD_X, 0.440, bottom=BOARD_TOP)
    beside_green = part(WB, 0.240, GREEN_TRAY[1] + INNER[1] / 2 + WALL_T + 0.025)
    under_green = part(WB, 0.240, 0.470, bottom=0.0, tilt=math.pi / 2)
    knocked_over = part(WB, ROW_X, 0.395, tilt=math.pi / 2)
    edge_in = part(YC, GREEN_TRAY[0] + INNER[0] / 2 - 0.001, 0.470, tilt=math.pi / 2, yaw=math.pi / 2)
    edge_out = part(YC, GREEN_TRAY[0] + INNER[0] / 2 + 0.001, 0.470, tilt=math.pi / 2, yaw=math.pi / 2)  # on the floor, as edge_in
    still_rolling = part(YC, 0.240, 0.440, tilt=math.pi / 2, vel=(0.0, 0.02, 0.0))

    done = dict(green=(WB, YC), red=(WC, YB))
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(**done), 1.0, True),
        "sorted by colour, white to green": (scene(green=(WB, WC), red=(YB, YC)), 0.0, False),
        "sorted by colour, yellow to green": (scene(green=(YB, YC), red=(WB, WC)), 0.0, False),
        "sorted by shape, boxes to green": (scene(green=(WB, YB), red=(WC, YC)), 0.0, False),
        "sorted by shape, cylinders to green": (scene(green=(WC, YC), red=(WB, YB)), 0.0, False),
        "sorted by height the wrong way round": (scene(green=(WC, YB), red=(WB, YC)), 0.0, False),
        "all four in the green tray": (scene(green=(WB, YB, WC, YC)), 0.0, False),
        "all four in the red tray": (scene(red=(WB, YB, WC, YC)), 0.0, False),
        "nearest tray: the outboard two to green": (scene(green=(WB, YB), red=(WC, YC)), 0.0, False),
        "alternating trays from the outboard end, green first": (scene(green=(WB, WC), red=(YB, YC)), 0.0, False),
        "alternating trays from the outboard end, red first": (scene(green=(YB, YC), red=(WB, WC)), 0.0, False),
        "the board half straight ahead of each part": (scene(green=(WB, YB), red=(WC, YC)), 0.0, False),
        "only the two short parts in green": (scene(green=(WB, YC)), 0.5, False),
        "only the two tall parts in red": (scene(red=(WC, YB)), 0.5, False),
        "three right, the fourth on its mark": (scene(green=(WB, YC), red=(WC,)), 0.75, False),
        "three right, the fourth in the wrong tray": (scene(green=(WB, YC, YB), red=(WC,)), 0.5, False),
        "two right, two wrong": (scene(green=(WB, WC), red=(YB, YC)), 0.0, False),
        "one right, one wrong, two on their marks": (scene(green=(WB, YB)), 0.0, False),
        "one right": (scene(red=(YB,)), 0.25, False),
        "a short part lying in the green tray counts": (scene(green=(WB,), red=(WC, YB), extra=[(YC, lying_in_green)]), 1.0, True),
        "a tall part leaning on the far wall inside counts": (scene(green=(WB, YC), red=(WC,), extra=[(YB, leaning_in_red)]), 1.0, True),
        "a tall part lying along x in the red tray counts": (scene(green=(WB, YC), red=(YB,), extra=[(WC, lying_tall_in_red)]), 1.0, True),
        "a part on a green wall top does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, on_green_wall)]), 0.75, False),
        "a part standing on another part does not count": (scene(green=(WB,), red=(WC, YB), extra=[(YC, on_part_in_green)]), 0.75, False),
        "a part resting on a lying part does not count": (scene(green=(WB,), red=(WC, YB), extra=[(YC, on_part_lying)]), 0.75, False),
        "a part held moving over the green tray does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, held_over_green)]), 0.75, False),
        "a part held still 12 mm above the floor does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, held_still_low)]), 0.75, False),
        "a part held still 5 mm above the floor does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, held_at_release)]), 0.75, False),
        "a part put on the board does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, on_board)]), 0.75, False),
        "a part beside the green tray does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, beside_green)]), 0.75, False),
        "a part on the floor under the green tray does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, under_green)]), 0.75, False),
        "a part knocked over on its mark does not count": (scene(green=(YC,), red=(WC, YB), extra=[(WB, knocked_over)]), 0.75, False),
        "centre 1 mm inside the pocket outline counts": (scene(green=(WB,), red=(WC, YB), extra=[(YC, edge_in)]), 1.0, True),
        "centre 1 mm outside the pocket outline does not": (scene(green=(WB,), red=(WC, YB), extra=[(YC, edge_out)]), 0.75, False),
        "done but a part still rolling": (scene(green=(WB,), red=(WC, YB), extra=[(YC, still_rolling)]), 0.75, False),
        "every part knocked over where it stood": (scene(extra=[(n, part(n, *STARTS[n], tilt=math.pi / 2, yaw=1.0)) for n in names]), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not got_success or got == 1.0, name
    assert distance(scene(**done)) == 0.0 and distance(scene()) > 0.0
    # Height is the only attribute that splits the five examples: both colours and both shapes sit on both halves.
    samples = [o for o in BOARD if o.name.startswith("sample_")]
    accepted = [o for o in samples if o.pos[1] > RED_HALF_Y + HALF[1] / 2]
    rejected = [o for o in samples if o.pos[1] < RED_HALF_Y + HALF[1] / 2]
    for group in (accepted, rejected):
        assert {o.color for o in group} == {GREY, BLUE} and {type(o) for o in group} == {Box, Cylinder}
    assert all(_target(p.name) is GREEN_TRAY for p in PARTS if HEIGHT[p.name] == SHORT)
    assert {p.color for p in PARTS}.isdisjoint({o.color for o in samples})
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
