"""On the station table, in the left arm's lane, a 1 g bench analogue of filling a three-day medication dose tray from
a written regimen. A fixed dark grey tray carries six light grey cell pads in a grid of three day columns, day 1 to
day 3 from left to right as seen from behind the arms, and two time rows, 08:00 nearer the arms and 20:00 farther
out. Six loose dose cups, three blue and three red, stand in two supply columns either side of the tray. The
instruction gives each colour a first dose and an interval; working the schedule out says which cells take which
colour, which cells stay empty and how many cups are spare. A dose in the wrong cell, a cup in a cell that must stay
empty, two cups in one cell, or a cup spilled or left loose on the tray scores 0; otherwise each correctly filled
cell earns a quarter.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/dose-tray-regimen-fill/dose_tray_regimen_fill.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Fill the dose tray with one cup per due dose and nothing else. Days 1 to 3 run left to right; the near row is 08:00, the far row 20:00. Blue is first due day 2 08:00, then every 24 h. Red is first due day 1 20:00, then every 48 h." \\
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
BASE_XY = (0.240, 0.230)  # the tray base's centre
BASE_SIZE = (0.156, 0.236, 0.003)
PAD_SIZE = (0.056, 0.056, 0.002)
PAD_TOP = TOP + BASE_SIZE[2] + PAD_SIZE[2]  # the cell pads' top faces, 5 mm above the table
ROW_X = (0.200, 0.280)  # near row, far row
ROW_HOUR = (8, 20)  # near row 08:00, far row 20:00
DAY_Y = (0.310, 0.230, 0.150)  # day 1, day 2, day 3: left to right as seen from behind the arms and in the top camera
CELLS = {(row, day): (ROW_X[row], DAY_Y[day]) for day in range(3) for row in range(2)}

# The regimen in the instruction: first dose in hours from day 1 00:00, and the interval in hours.
REGIMEN = {"blue": (24 + 8, 24), "red": (20, 48)}


def _due(colour, hour) -> bool:
    first, every = REGIMEN[colour]
    return hour >= first and (hour - first) % every == 0


DUE = {cell: next((c for c in REGIMEN if _due(c, 24 * cell[1] + ROW_HOUR[cell[0]])), None) for cell in CELLS}
N_DUE = sum(1 for c in DUE.values() if c)  # four doses fall inside the three days; two cells stay empty

CUP_R = 0.015  # 30 mm across
CUP_H = 0.045
CUP_MASS = 0.025
BLUE = (0.20, 0.40, 0.85)
RED = (0.85, 0.20, 0.20)
TRAY = (0.25, 0.25, 0.27)
PAD_GREY = (0.80, 0.80, 0.80)
COLOURS = {"blue": BLUE, "red": RED}

# Supply columns either side of the tray, 80 mm apart along x, each cup 37 mm clear of the tray base's edge. The
# left column stands at y 0.400 because the left arm at its ready pose hides y 0.46 and beyond from the top camera.
SUPPLY = (
    ("blue_cup_a", "blue", (0.140, 0.400)),
    ("red_cup_a", "red", (0.220, 0.400)),
    ("blue_cup_b", "blue", (0.300, 0.400)),
    ("red_cup_b", "red", (0.140, 0.060)),
    ("blue_cup_c", "blue", (0.220, 0.060)),
    ("red_cup_c", "red", (0.300, 0.060)),
)
CUP_COLOUR = {name: colour for name, colour, _ in SUPPLY}
CUPS = [Cylinder(name, pos=(x, y, TOP + CUP_H / 2), radius=CUP_R, height=CUP_H, mass=CUP_MASS, color=COLOURS[colour]) for name, colour, (x, y) in SUPPLY]

TRAY_BASE = Box("tray_base", pos=(BASE_XY[0], BASE_XY[1], TOP + BASE_SIZE[2] / 2), size=BASE_SIZE, color=TRAY)
CELL_PADS = [
    Box(f"cell_day{day + 1}_{ROW_HOUR[row]:02d}00", pos=(x, y, TOP + BASE_SIZE[2] + PAD_SIZE[2] / 2), size=PAD_SIZE, color=PAD_GREY)
    for (row, day), (x, y) in CELLS.items()
]
OBJECTS = [*CUPS, TRAY_BASE, *CELL_PADS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

CENTRE_TOL = 0.020  # a cup's centre within 20 mm of its cell's centre
BOTTOM_TOL = 0.004  # its lowest point within 4 mm of the pad top
TILT_MAX = math.radians(10.0)  # its axis within 10 degrees of vertical
EPS = 1e-4  # values exactly on a limit pass


def _axis_z(state) -> float:
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)  # world z of the cup's axis


def _bottom(state) -> float:
    """The cup's lowest point from its centre and axis, standing, tilted or lying. The axis-aligned bounds of a lying
    cylinder come from its box corners and read up to r(sqrt 2 - 1) too low, so they are not used."""
    uz = _axis_z(state)
    return float(state.pos[2]) - (CUP_H / 2 * abs(uz) + CUP_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _cell_of(state):
    """The cell a cup stands in: at rest, upright either way up within 10 degrees, its lowest point within 4 mm of the
    pad top and its centre within 20 mm of that cell's centre; otherwise None."""
    if not state.resting() or abs(_axis_z(state)) < math.cos(TILT_MAX) - 1e-9:
        return None
    if abs(_bottom(state) - PAD_TOP) > BOTTOM_TOL + EPS:
        return None
    x, y = float(state.pos[0]), float(state.pos[1])
    for cell, (cx, cy) in CELLS.items():
        if math.hypot(x - cx, y - cy) <= CENTRE_TOL + EPS:
            return cell
    return None


def _over_tray(state) -> bool:
    """The cup's centre lies over the tray base's 156 x 236 mm footprint, seen from above."""
    return abs(float(state.pos[0]) - BASE_XY[0]) < BASE_SIZE[0] / 2 and abs(float(state.pos[1]) - BASE_XY[1]) < BASE_SIZE[1] / 2


def _fill(objects):
    """Cups per cell, by colour, and whether any cup rests on the tray without standing in a cell."""
    filled, loose = {}, False
    for cup in CUPS:
        st = objects[cup.name]
        cell = _cell_of(st)
        if cell is not None:
            filled.setdefault(cell, []).append(CUP_COLOUR[cup.name])
        elif st.resting() and _over_tray(st):
            loose = True
    return filled, loose


def score(objects) -> float:
    """Gate, then count. The score is 0 when any cup rests on the tray without standing in a cell (lying, tilted,
    between cells, or on another cup), when two cups stand in one cell, when a cell holds the wrong colour, or when a
    cup stands in a cell that must stay empty. Otherwise it is the number of correctly filled cells over four, so 0,
    0.25, 0.5, 0.75 or 1.0. Cups anywhere off the tray count neither way."""
    filled, loose = _fill(objects)
    if loose:
        return 0.0
    for cell, colours in filled.items():
        if len(colours) > 1 or DUE[cell] != colours[0]:
            return 0.0
    return len(filled) / N_DUE


def success(objects) -> bool:
    """Every due cell holds one cup of its colour, the empty cells are empty, nothing else is on the tray, and every
    cup whose centre is over the tray is at rest, so a cup still rolling or sliding across the tray holds success off
    (score() ignores it, as a moving cup counts neither way); holds 1 s. success() implies score() == 1.0."""
    return score(objects) >= 1.0 - 1e-9 and all(objects[c.name].resting() for c in CUPS if _over_tray(objects[c.name]))


def distance(objects) -> float:
    """Metres still to go: for each colour, the best assignment of its cups to its due cells, summed horizontally."""
    total = 0.0
    for colour in REGIMEN:
        cells = [CELLS[c] for c, due in DUE.items() if due == colour]
        cups = [objects[c.name] for c in CUPS if CUP_COLOUR[c.name] == colour]
        costs = [[math.hypot(float(s.pos[0]) - cx, float(s.pos[1]) - cy) for (cx, cy) in cells] for s in cups]
        best = math.inf
        for i in range(len(cups)):
            for j in range(len(cups)):
                if i != j:
                    best = min(best, costs[i][0] + costs[j][1])
        total += best
    return total


TRAVEL = TOP + 0.140
GRIP_Z = TOP + 0.025  # TCP 2.5 mm over the cup's mid height, pad tips 12 mm above the table, 33 mm of cup above them
PLACE_Z = PAD_TOP + 0.005 + 0.025  # the cup's bottom 5 mm above the pad; pad tips 17 mm above the pad top
ENTRY = 0.025  # the pause above a cell: the cup's bottom 30 mm above the pad
# Every open finger is about 46 mm wide and reaches 56 mm out along the closing axis, so a cup 80 mm away along the
# closing axis is only 9 mm from it. Yaw 0 closes the fingers along y: across a supply column, where the cups are 80 mm
# apart along x, and toward the tray base, which the pad tips clear by 9 mm. Each place closes them along the axis on
# which the cell has no filled neighbour: yaw 0 (along y) except day 3 08:00, whose day 2 neighbour is filled, which
# takes yaw -90 degrees (along x).
ALONG_Y = 0.0
ALONG_X = -math.pi / 2
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part

MOVES = (  # cup, cell, pick yaw, place yaw
    ("blue_cup_a", (0, 1), ALONG_Y, ALONG_Y),  # day 2 08:00
    ("red_cup_a", (1, 0), ALONG_Y, ALONG_Y),  # day 1 20:00
    ("blue_cup_c", (0, 2), ALONG_Y, ALONG_X),  # day 3 08:00
    ("red_cup_b", (1, 2), ALONG_Y, ALONG_Y),  # day 3 20:00
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
    """Left arm only. Four top-down pick and place moves at 14 cm travel height, the two from the left supply column
    first: blue to day 2 08:00, red to day 1 20:00, then blue to day 3 08:00 and red to day 3 20:00, each cup released
    5 mm above its pad. Every pick closes the fingers across its supply column; every place closes them along an axis
    with no filled cell on it. The two far supply cups stay where they are. 64.8 s of plan; in the simulator the
    episode ends on success at step 1,272 at 20 Hz (63.6 s), as the fingers finish opening on the last cup (the
    scripted policy would otherwise stop after a 3 s hold, at step 1,356)."""
    start = {name: xy for name, _, xy in SUPPLY}
    left = []
    for name, cell, yaw, place_yaw in MOVES:
        (px, py), (qx, qy) = start[name], CELLS[cell]
        left += _pick_place((px, py, GRIP_Z), (qx, qy, PLACE_Z), width=2 * CUP_R, entry=ENTRY, travel_z=TRAVEL, yaw=yaw, place_yaw=place_yaw)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    assert DUE == {(0, 0): None, (0, 1): "blue", (0, 2): "blue", (1, 0): "red", (1, 1): None, (1, 2): "red"}, DUE
    assert [CUP_COLOUR[n] == DUE[c] for n, c, _, _ in MOVES] == [True] * 4

    def quat(yaw, tilt):  # tilt about the cup's own x axis, then yaw about the vertical
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def cup(x, y, bottom=TOP, tilt=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0)):
        """A cup with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator reports
        them."""
        q = quat(yaw, tilt)
        xq, yq, zq, wq = q
        rot = np.array([[1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
                        [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
                        [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)]])
        half = np.abs(rot) @ np.array([CUP_R, CUP_R, CUP_H / 2])
        pos = np.array([x, y, bottom + CUP_H / 2 * abs(math.cos(tilt)) + CUP_R * abs(math.sin(tilt))])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def in_cell(cell, dx=0.0, dy=0.0, **kw):
        cx, cy = CELLS[cell]
        return cup(cx + dx, cy + dy, bottom=PAD_TOP, **kw)

    start = {name: cup(x, y) for name, _, (x, y) in SUPPLY}

    def scene(**placed):
        """The start state with some cups replaced: a cell key stands the cup in that cell, a state is used as is."""
        states = dict(start)
        for name, where in placed.items():
            states[name] = in_cell(where) if isinstance(where, tuple) else where
        return states

    right = dict(blue_cup_a=(0, 1), blue_cup_c=(0, 2), red_cup_a=(1, 0), red_cup_b=(1, 2))
    lying = math.pi / 2
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(**right), 1.0, True),
        "done with the other cups of each colour": (scene(blue_cup_b=(0, 1), blue_cup_a=(0, 2), red_cup_c=(1, 0), red_cup_a=(1, 2)), 1.0, True),
        "done, the spares moved elsewhere on the table": (scene(**right, blue_cup_b=cup(0.420, 0.420), red_cup_c=cup(0.100, 0.300, tilt=lying)), 1.0, True),
        "done, a spare off the table": (scene(**right, red_cup_c=cup(0.950, 0.100, bottom=0.0, tilt=lying)), 1.0, True),
        "one dose placed": (scene(red_cup_a=(1, 0)), 0.25, False),
        "the two red doses only": (scene(red_cup_a=(1, 0), red_cup_b=(1, 2)), 0.5, False),
        "every 48 h for both colours (one dose missed)": (scene(blue_cup_a=(0, 1), red_cup_a=(1, 0), red_cup_b=(1, 2)), 0.75, False),
        "every 24 h for both colours (red on day 2 20:00)": (scene(**right, red_cup_c=(1, 1)), 0.0, False),
        "blue from day 1 08:00 (an empty cell filled)": (scene(**right, blue_cup_b=(0, 0)), 0.0, False),
        "all six cells filled": (scene(blue_cup_a=(0, 0), blue_cup_b=(0, 1), blue_cup_c=(0, 2), red_cup_a=(1, 0), red_cup_b=(1, 1), red_cup_c=(1, 2)), 0.0, False),
        "one colour in every due cell": (scene(blue_cup_a=(0, 1), blue_cup_b=(0, 2), blue_cup_c=(1, 0)), 0.0, False),
        "days read right to left": (scene(blue_cup_a=(0, 1), blue_cup_c=(0, 0), red_cup_a=(1, 2), red_cup_b=(1, 0)), 0.0, False),
        "rows swapped": (scene(blue_cup_a=(1, 1), blue_cup_c=(1, 2), red_cup_a=(0, 0), red_cup_b=(0, 2)), 0.0, False),
        "colours swapped": (scene(red_cup_a=(0, 1), red_cup_b=(0, 2), blue_cup_a=(1, 0), blue_cup_c=(1, 2)), 0.0, False),
        "three right and one wrong colour": (scene(blue_cup_a=(0, 1), red_cup_c=(0, 2), red_cup_a=(1, 0), red_cup_b=(1, 2)), 0.0, False),
        "two blue cups in one cell": (scene(**{**right, "blue_cup_a": in_cell((0, 1), dx=-0.016)}, blue_cup_b=in_cell((0, 1), dx=0.016)), 0.0, False),
        "done, a spare standing on a filled cup": (scene(**right, blue_cup_b=cup(ROW_X[0], DAY_Y[1], bottom=PAD_TOP + CUP_H)), 0.0, False),
        "done, a spare lying on the tray between cells": (scene(**right, red_cup_c=cup(0.240, 0.270, bottom=TOP + BASE_SIZE[2], tilt=lying)), 0.0, False),
        "done, a spare lying on an empty cell": (scene(**right, blue_cup_b=in_cell((0, 0), tilt=lying, yaw=0.4)), 0.0, False),
        "a dose cup spilled in its own cell": (scene(**{**right, "blue_cup_c": in_cell((0, 2), tilt=lying)}), 0.0, False),
        "a dose cup tilted 8 degrees in its cell counts": (scene(**{**right, "red_cup_b": in_cell((1, 2), tilt=math.radians(8))}), 1.0, True),
        "a dose cup tilted 12 degrees in its cell trips the gate": (scene(**{**right, "red_cup_b": in_cell((1, 2), tilt=math.radians(12))}), 0.0, False),
        "a dose cup upside down counts": (scene(**{**right, "red_cup_b": in_cell((1, 2), tilt=math.pi)}), 1.0, True),
        "a dose cup 20 mm off its cell centre counts": (scene(**{**right, "blue_cup_a": in_cell((0, 1), dx=0.012, dy=-0.016)}), 1.0, True),
        "a dose cup 22 mm off its cell centre, on the pad edge, trips the gate": (scene(**{**right, "blue_cup_a": in_cell((0, 1), dy=0.022)}), 0.0, False),
        "a dose cup standing on the tray base between cells trips the gate": (scene(**{**right, "blue_cup_a": cup(0.240, 0.270, bottom=TOP + BASE_SIZE[2])}), 0.0, False),
        "three doses placed, the fourth still moving in its cell": (scene(**{**right, "red_cup_b": in_cell((1, 2), vel=(0.0, 0.02, 0.0))}), 0.75, False),
        "three doses placed, the fourth carried over the tray": (scene(**{**right, "red_cup_b": cup(0.280, 0.230, bottom=PAD_TOP + 0.020, vel=(0.0, 0.0, -0.03))}), 0.75, False),
        "every cup knocked over where it stood": (scene(**{n: cup(x, y, tilt=lying, yaw=1.0) for n, _, (x, y) in SUPPLY}), 0.0, False),
        "done, then a spare knocked onto the tray": (scene(**right, red_cup_c=cup(0.300, 0.120, bottom=TOP + BASE_SIZE[2], tilt=lying, yaw=0.5)), 0.0, False),
        "a spare standing on the table beside the tray": (scene(**right, blue_cup_b=cup(0.240, 0.365)), 1.0, True),
        "done, a spare still rolling across the tray: 1.0 without success": (scene(**right, blue_cup_b=cup(0.240, 0.270, bottom=TOP + BASE_SIZE[2], tilt=lying, vel=(0.02, 0.0, 0.0))), 1.0, False),
        "done, a spare creeping through a must-stay-empty cell: 1.0 without success": (scene(**right, red_cup_c=in_cell((1, 1), vel=(0.011, 0.0, 0.0))), 1.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not got_success or got == 1.0, name
    assert distance(scene(**right)) < 1e-9 and distance(scene()) > 0.0
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
