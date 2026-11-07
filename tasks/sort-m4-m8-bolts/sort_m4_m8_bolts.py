"""On the station table, in the right arm's half: a green cutting mat (22 x 30 cm) with eight loose hex bolts on it,
scattered over a 15 x 15 cm patch at various angles: four M8 x 30 bolts, bright zinc plated (silver, 15 g), and four
M4 x 20 bolts, black oxide (black, 3 g), each lying on a flat of its head. Nearer the robot and between the arms
stands a blue two-compartment parts organizer (two 9 x 9 cm compartments side by side, 3 cm deep), fixed to the
table. The task: put the M8 bolts in the organizer's left compartment and the M4 bolts in its right one (the robot's
left and right).

Each bolt is one rigid body of two box colliders (head and shank, ``bolts.py``) under a render-only Blender mesh of
a threaded hex bolt (``looks.py``, ``looks/make_looks.py``). The organizer is fixed boxes under a moulded look.

Run it:  inspect-robots run --embodiment yam_arena -E task=sort_m4_m8_bolts.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bolts import Bolt, bolt  # noqa: E402
from looks import dress  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the cutting mat: fixed, 2 mm thick ------------------------------------------------------------------------------
MAT_SIZE = (0.22, 0.30, 0.002)
MAT_XY = (0.355, -0.20)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("cutting_mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.06, 0.30, 0.20))

# -- the organizer: a 3 mm floor, four outer walls and a divider, all fixed --------------------------------------------
ORG_XY = (0.18, -0.02)  # centre
CELL = 0.090  # m: each compartment's inside, square
WALL = 0.004  # m: wall and divider thickness
FLOOR = 0.003  # m
ORG_H = 0.030  # m: the walls' top above the table
ORG_OUT = (CELL + 2 * WALL, 2 * CELL + 3 * WALL)  # 9.8 x 19.2 cm
WALL_H = ORG_H - FLOOR
CELL_Y = {"left": ORG_XY[1] + (WALL + CELL) / 2, "right": ORG_XY[1] - (WALL + CELL) / 2}  # compartment centres
BLUE = (0.10, 0.36, 0.78)
_wz = TOP + FLOOR + WALL_H / 2
ORGANIZER = [
    Box("organizer", pos=(*ORG_XY, TOP + FLOOR / 2), size=(*ORG_OUT, FLOOR), color=BLUE),
    Box("organizer_front", pos=(ORG_XY[0] - (CELL + WALL) / 2, ORG_XY[1], _wz), size=(WALL, ORG_OUT[1], WALL_H),
        color=BLUE),
    Box("organizer_back", pos=(ORG_XY[0] + (CELL + WALL) / 2, ORG_XY[1], _wz), size=(WALL, ORG_OUT[1], WALL_H),
        color=BLUE),
    Box("organizer_left", pos=(ORG_XY[0], ORG_XY[1] + ORG_OUT[1] / 2 - WALL / 2, _wz), size=(CELL, WALL, WALL_H),
        color=BLUE),
    Box("organizer_right", pos=(ORG_XY[0], ORG_XY[1] - ORG_OUT[1] / 2 + WALL / 2, _wz), size=(CELL, WALL, WALL_H),
        color=BLUE),
    Box("organizer_divider", pos=(*ORG_XY, _wz), size=(CELL, WALL, WALL_H), color=BLUE),
]

# -- the bolts: ISO 4017 hex bolts, lying on a flat of the head ------------------------------------------------------
SIZES = {
    # thread d, head height k, across flats s, across corners e, shank length l (m), mass (kg)
    "m8": dict(d=0.008, k=0.0053, s=0.013, e=0.0145, l=0.030, mass=0.015),
    "m4": dict(d=0.004, k=0.0028, s=0.007, e=0.0077, l=0.020, mass=0.003),
}
COLORS = {"m8": (0.78, 0.80, 0.84), "m4": (0.03, 0.03, 0.035)}  # bright zinc, black oxide (the looks draw them)
REST = 0.0005  # bolts spawn this far above the mat and settle onto it
# name: (x, y, yaw in degrees); the bolt's axis points along yaw from its head to its tip
LAYOUT = {
    "m8_1": (0.29, -0.15, 30.0),
    "m4_1": (0.30, -0.22, -60.0),
    "m8_2": (0.28, -0.27, 80.0),
    "m4_2": (0.35, -0.17, 10.0),
    "m8_3": (0.36, -0.245, -35.0),
    "m4_3": (0.40, -0.14, 70.0),
    "m4_4": (0.34, -0.29, -15.0),
    "m8_4": (0.41, -0.21, 50.0),
}


def _bolt(name: str, x: float, y: float, yaw_deg: float) -> Bolt:
    kind = SIZES[name[:2]]
    return bolt(Bolt(
        name,
        pos=(x, y, MAT_TOP + kind["s"] / 2 + REST),
        size=(kind["k"] + kind["l"], kind["e"], kind["s"]),
        mass=kind["mass"],
        yaw=math.radians(yaw_deg),
        color=COLORS[name[:2]],
        d=kind["d"],
        k=kind["k"],
        s=kind["s"],
        l=kind["l"],
    ))


BOLTS = [_bolt(name, *spec) for name, spec in LAYOUT.items()]
M8 = [b.name for b in BOLTS if b.name.startswith("m8")]
M4 = [b.name for b in BOLTS if b.name.startswith("m4")]
TARGET = {**{n: "left" for n in M8}, **{n: "right" for n in M4}}

OBJECTS = [MAT, *ORGANIZER, *BOLTS]

dress(
    {"cutting_mat": "mat.usdc", "organizer": "organizer.usdc",
     **{b.name: f"{b.name[:2]}.usdc" for b in BOLTS}},
    hide=[o.name for o in ORGANIZER[1:]],
)

# -- scoring ---------------------------------------------------------------------------------------------------------
FALLEN_Z = -0.02  # m above the table: a bolt whose centre is lower has fallen off the table


def compartment(pos) -> str | None:
    """"left" or "right" when the point is inside that compartment (over its 9 x 9 cm floor, below the walls'
    top), else None."""
    x, y, z = pos
    if not (abs(x - ORG_XY[0]) < CELL / 2 and TOP < z < TOP + ORG_H):
        return None
    for side, cy in CELL_Y.items():
        if abs(y - cy) < CELL / 2:
            return side
    return None


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.lin_vel)) for s in objects.values())


def _void(objects) -> bool:
    """Scores 0: a non-finite state, or a bolt fallen off the table."""
    return not _finite(objects) or any(objects[n].pos[2] < TOP + FALLEN_Z for n in TARGET)


def tally(objects) -> tuple[int, int]:
    """(bolts at rest in their own compartment, bolts in the other compartment, at rest or not)."""
    right = wrong = 0
    for name, side in TARGET.items():
        state = objects[name]
        where = compartment(state.pos)
        if where == side and state.resting():
            right += 1
        elif where is not None and where != side:
            wrong += 1
    return right, wrong


def success(objects) -> bool:
    """All eight bolts at rest in their own compartment."""
    return not _void(objects) and tally(objects) == (len(TARGET), 0)


def score(objects) -> float:
    """1/8 for each bolt at rest in its own compartment, minus 1/8 for each bolt in the other one, at least 0. 0 if
    any bolt has fallen off the table."""
    if _void(objects):
        return 0.0
    right, wrong = tally(objects)
    return max(0, right - wrong) / len(TARGET)


def distance(objects) -> float:
    """Sum of the bolts' horizontal distances to their compartment's centre (m)."""
    return float(sum(math.dist(objects[n].pos[:2], (ORG_XY[0], CELL_Y[side])) for n, side in TARGET.items()))


# -- scripted demo ---------------------------------------------------------------------------------------------------
PAD_TIP = 0.013  # m: the finger pads' tips are this far below the TCP (yam_arena's PAD_TIP_PAST_TCP)
TIP_ABOVE = {"m8": 0.003, "m4": 0.0012}  # m: the pad tips this far above the mat when they close on the shank
GRIP_ALONG = {"m8": 0.0, "m4": 0.0015}  # m from the box's centre toward the tip: the pads clear the head
# the pinch: the fingers are sent 1 mm past each side of the shank, so the soft finger drive (5,000 N/m) squeezes about
# 5 N instead of the full 50 N, which shoves and spins a 3 g bolt as the pads land
PINCH = {"m8": -0.003, "m4": -0.001}
OPEN_PICK = -0.016  # finger joint: a 32 mm gap to come down round a bolt, clear of its neighbours
OPEN_PLACE = -0.012  # a 24 mm gap to let go inside a compartment, clear of its walls
TRAVEL_Z = TOP + 0.075  # the carried bolt clears the organizer's 3 cm walls
# where each bolt is let go: its grip point over a 2 x 2 grid in its compartment, the bolt along x
SLOTS = [(-0.022, -0.02), (0.022, -0.02), (-0.022, 0.02), (0.022, 0.02)]
RELEASE_GAP = 0.003  # m: the bolt's underside is this far above the compartment floor when let go
ORDER = ["m8_1", "m4_2", "m4_3", "m8_4", "m8_3", "m4_1", "m4_4", "m8_2"]
PACE = 1.0
DEMO_TO = {"m8": "left", "m4": "right"}  # where the demo takes each size (ablations change it to show shortcuts score 0)
RETREAT = (0.15, -0.30, TOP + 0.18)  # the hand ends back beside its base, off the organizer and the mat


def _grip_yaw(yaw: float) -> float:
    """The grip's yaw for a bolt at ``yaw``: closing across the shank, turned into -90..90 degrees."""
    return (yaw + math.pi / 2) % math.pi - math.pi / 2


def bolt_waypoints(spec: Bolt, slot: tuple[float, float], pace: float = PACE) -> list[Waypoint]:
    kind = spec.name[:2]
    gx, gy = spec.grip_point(GRIP_ALONG[kind])
    yaw = _grip_yaw(spec.yaw)
    pick_z = MAT_TOP + TIP_ABOVE[kind] + PAD_TIP
    cx, cy = ORG_XY[0] + slot[0] + GRIP_ALONG[kind], CELL_Y[DEMO_TO[kind]] + slot[1]
    # held as gripped: the bolt's underside hangs about TIP_ABOVE below the pad tips
    place_z = TOP + FLOOR + RELEASE_GAP + TIP_ABOVE[kind] + PAD_TIP
    k = 1.0 / pace
    return [
        Waypoint(1.2 * k, (gx, gy, TRAVEL_Z), OPEN_PICK, f"over {spec.name}", yaw),
        Waypoint(1.0 * k, (gx, gy, pick_z), OPEN_PICK, "descend", yaw),
        Waypoint(0.6 * k, (gx, gy, pick_z), PINCH[kind], "pinch", yaw),
        Waypoint(0.8 * k, (gx, gy, TRAVEL_Z), PINCH[kind], "lift", yaw),
        Waypoint(1.4 * k, (cx, cy, TRAVEL_Z), PINCH[kind], "carry", 0.0),
        Waypoint(0.8 * k, (cx, cy, place_z), PINCH[kind], "lower", 0.0),
        Waypoint(0.4 * k, (cx, cy, place_z), OPEN_PLACE, "let go", 0.0),
        Waypoint(0.6 * k, (cx, cy, TRAVEL_Z), OPEN_PLACE, "up", 0.0),
    ]


def demo():
    """Right arm: pinch each bolt across its shank from above, carry it to its compartment (M8 left, M4 right) and
    let it go on the floor, at its own spot of a 2 x 2 grid, then pull the hand back. The left arm holds still."""
    specs = {b.name: b for b in BOLTS}
    used = {"left": 0, "right": 0}
    plan = []
    for name in ORDER:
        side = DEMO_TO[name[:2]]
        plan += bolt_waypoints(specs[name], SLOTS[used[side] % len(SLOTS)])
        used[side] += 1
    plan.append(Waypoint(1.5, RETREAT, OPEN_PLACE, "clear the view", 0.0))
    return {"right": plan}
