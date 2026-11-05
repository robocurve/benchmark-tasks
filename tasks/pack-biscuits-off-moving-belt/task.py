"""On the station table: a moving belt runs from right to left across the far side of the table, carrying eight biscuit
packs in line; every third one is cracked and marked with a red band. Two open three-cell cases sit nearer the arms, one on
each side. The goal is to take the six good packs off the belt before they cross the red line, put one in each cell, and let
the cracked packs ride past. A pack passes every 1.7 s, faster than one arm's pick-and-place cycle.

Run it (the belt needs CPU physics):  inspect-robots run --embodiment yam_arena -E task=task.py -E device=cpu \\
           --instruction "Take the six good biscuit packs off the moving belt before they cross the red line and put one in each cell of the two cases. Let the cracked packs with the red band ride past." \\
           --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import axis_world, rigid_part  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK, RED, GOLD, CARD = ((0.45, 0.47, 0.50), (0.14, 0.14, 0.16), (0.80, 0.12, 0.10), (0.85, 0.62, 0.25), (0.75, 0.68, 0.55))

BELT_X, BELT_W, BELT_T = 0.33, 0.12, 0.02
BELT_TOP = TOP + 0.03
BELT_Y0, BELT_Y1 = -1.08, 0.45  # its ends; it runs in from past the edge of the table
SPEED = 0.06  # m/s, right to left
PACK = (0.04, 0.07, 0.035)  # across the belt, along it, tall
PACK_MASS = 0.05
SPACING = 0.10  # a pack every 1.7 s
FIRST_Y = -0.30  # the leading pack starts here; the rest follow behind it
N_PACKS, CRACKED = 8, (2, 5)  # every third pack in line is cracked
LINE_Y = 0.24  # the red line across the belt
PICK_Y = {"right": -0.08, "left": 0.18}  # where each hand waits

CASE_X, CASE_Y = 0.15, (-0.17, 0.17)  # the two cases, right and left
CELL = (0.075, 0.105)  # front to back, left to right
ROWS = 3
WALL, WALL_H, FLOOR = 0.004, 0.02, 0.004


def cell_centre(row, col):
    return (CASE_X + (row - 1) * CELL[0], CASE_Y[col])


CELLS = [(r, c) for r in range(ROWS) for c in range(2)]


def _case(name, y):
    w, d = ROWS * CELL[0], CELL[1]
    boxes = [((0.0, 0.0, FLOOR / 2), (w + WALL, d + WALL, FLOOR), CARD)]
    for r in range(ROWS + 1):
        boxes.append((((r - ROWS / 2) * CELL[0], 0.0, FLOOR + WALL_H / 2), (WALL, d + WALL, WALL_H), STEEL))
    for s in (-1, 1):
        boxes.append(((0.0, s * d / 2, FLOOR + WALL_H / 2), (w + WALL, WALL, WALL_H), STEEL))
    return rigid_part(name, pos=(CASE_X, y, TOP), boxes=boxes)


def pack(i):
    boxes = [((0.0, 0.0, PACK[2] / 2), PACK, GOLD)]
    if i in CRACKED:
        boxes = [((0.0, 0.0, PACK[2] / 2), PACK, GOLD), ((0.0, 0.0, PACK[2] + 0.001), (PACK[0], 0.03, 0.002), RED)]
    return rigid_part(f"pack_{i}", pos=(BELT_X, FIRST_Y - i * SPACING, BELT_TOP + 0.0005), boxes=boxes, mass=PACK_MASS)


BELT = rigid_part("belt", pos=(BELT_X, (BELT_Y0 + BELT_Y1) / 2, BELT_TOP - BELT_T / 2),
                  boxes=[((0.0, 0.0, 0.0), (BELT_W, BELT_Y1 - BELT_Y0, BELT_T), DARK)], surface_speed=(0.0, SPEED, 0.0))
SIDE_H = 0.008
SIDES = [Box(f"side_{i}", pos=(BELT_X + s * (BELT_W / 2 + 0.004), (BELT_Y0 + BELT_Y1) / 2, BELT_TOP + SIDE_H / 2 - 0.004),
             size=(0.008, BELT_Y1 - BELT_Y0, SIDE_H + 0.008), color=STEEL) for i, s in enumerate((-1, 1))]
END_STOP = Box("end_stop", pos=(BELT_X, BELT_Y1 + 0.005, BELT_TOP + 0.015), size=(BELT_W + 0.016, 0.01, 0.05), color=STEEL)
LEGS = [Box(f"leg_{i}", pos=(BELT_X, y, (TOP + BELT_TOP - BELT_T) / 2), size=(0.10, 0.03, BELT_TOP - BELT_T - TOP), color=STEEL)
        for i, y in enumerate((-0.45, 0.0, 0.38))]
LINE = Box("red_line", pos=(BELT_X + BELT_W / 2 + 0.03, LINE_Y, TOP + 0.0005), size=(0.04, 0.008, 0.001), color=RED)
PACKS = [pack(i) for i in range(N_PACKS)]
OBJECTS = [BELT, *SIDES, END_STOP, *LEGS, LINE, _case("case_right", CASE_Y[0]), _case("case_left", CASE_Y[1]), *PACKS]

NAMES = [f"pack_{i}" for i in range(N_PACKS)]
GOOD = [n for i, n in enumerate(NAMES) if i not in CRACKED]
BAD = [NAMES[i] for i in CRACKED]
ON_BELT_Z = BELT_TOP + 0.01  # a pack's frame below this, over the belt, is on the belt
STALL_SPEED, STALL_S = 0.02, 3.0  # a pack on the belt moving slower than this for this long has been held back
CONTROL_DT = 0.05


IN_CELL = 0.006  # m: a pack lies in a cell with its underside this close to the case's floor, not up on a divider


def on_belt(state) -> bool:
    return bool(abs(float(state.pos[0]) - BELT_X) < BELT_W / 2 + 0.01 and float(state.pos[2]) < ON_BELT_Z and float(state.pos[1]) < LINE_Y)


def cell_of(state):
    """The (row, case) of the cell a pack lies flat in, or None."""
    x, y, z = (float(v) for v in state.pos)
    upright = axis_world(state)[2] > math.cos(math.radians(10))
    row = int((x - CASE_X + ROWS * CELL[0] / 2) // CELL[0]) if abs(x - CASE_X) < ROWS * CELL[0] / 2 else None
    col = next((c for c, cy in enumerate(CASE_Y) if abs(y - cy) < CELL[1] / 2), None)
    return (row, col) if row is not None and col is not None and z < TOP + FLOOR + IN_CELL and upright else None


def in_case(state) -> bool:
    """Anywhere inside a case's walls, even if not lying flat."""
    x, y, z = (float(v) for v in state.pos)
    return bool(abs(x - CASE_X) < ROWS * CELL[0] / 2 and any(abs(y - cy) < CELL[1] / 2 for cy in CASE_Y) and z < TOP + FLOOR + WALL_H + 0.03)


class Watch:
    """Which packs are lost: past the red line, or held back on the belt. A new trial puts the last pack back at its start in
    one control step, faster than anything moves on its own, and only that clears the record."""

    def __init__(self):
        self.last_y, self.lost, self.stalled = None, set(), {}

    def update(self, objects):
        y = float(objects[NAMES[-1]].pos[1])
        if self.last_y is not None and y < self.last_y - 0.05:
            self.lost, self.stalled = set(), {}
        self.last_y = y
        for n in NAMES:
            s = objects[n]
            if float(s.pos[1]) >= LINE_Y and float(s.pos[2]) < ON_BELT_Z + 0.02:
                self.lost.add(n)
            slow = on_belt(s) and float(np.linalg.norm(s.lin_vel)) < STALL_SPEED
            self.stalled[n] = self.stalled.get(n, 0.0) + CONTROL_DT if slow else 0.0
            if self.stalled[n] >= STALL_S:
                self.lost.add(n)


WATCH = Watch()


def packed(objects) -> int:
    """How many cells hold exactly one good pack that was never lost, lying flat."""
    cells = [cell_of(objects[n]) for n in NAMES]
    good = [cell_of(objects[n]) for n in GOOD if n not in WATCH.lost]
    return sum(1 for c in set(good) if c is not None and cells.count(c) == 1)


def cracked_in_case(objects) -> bool:
    return any(in_case(objects[n]) for n in BAD)


def success(objects):
    """All six cells hold one good pack, both cracked packs are past the red line, and the packed packs are still; must hold
    for 1 s. Runs first in every control step, so it also keeps the record."""
    WATCH.update(objects)
    past = all(float(objects[n].pos[1]) >= LINE_Y for n in BAD)
    return bool(packed(objects) == 6 and past and all(objects[n].resting() for n in GOOD))


def score(objects):
    """Partial credit (success scores 1.0): 0.15 for each cell holding one good pack, less 0.3 if a cracked pack is in the
    case, never below 0."""
    return max(0.0, 0.15 * min(packed(objects), 6) - (0.3 if cracked_in_case(objects) else 0.0))


def arrival(i, side) -> float:
    """When pack ``i`` reaches a hand's waiting point (s)."""
    return (PICK_Y[side] - (FIRST_Y - i * SPACING)) / SPEED


# Which arm takes which pack, and which cell it goes in; each arm fills its own side of the case.
PLAN = {"right": [(0, (0, 0)), (3, (1, 0)), (6, (2, 0))], "left": [(1, (0, 1)), (4, (1, 1)), (7, (2, 1))]}
PLACE_SHIFT = {"right": (-0.01, 0.0), "left": (-0.01, 0.0)}  # the pack's place in the hand, measured in traced runs
CATCH_LEAD = 1.05  # s before a pack's centre reaches the hand that the fingers start to close


def demo():
    """Each arm comes down with its fingers open either side of the belt, so a pack rides in between them, then closes,
    lifts, carries the pack to its cell, lets go and comes back. An arm that is behind takes its pack further along the
    belt. Both arms let the cracked packs ride through."""
    grip_z = BELT_TOP + 0.036
    hover = BELT_TOP + 0.08
    place_z = TOP + FLOOR + WALL_H + 0.006 + 0.036
    back, settle, down, catch, lift, carry, lower, release, up = 0.9, 0.4, 0.6, 0.5, 0.4, 0.9, 0.35, 0.3, 0.2

    def arm(side, yaw):
        out, free, move = [], 0.0, 2.0
        for i, cell in PLAN[side]:
            due = arrival(i, side) - CATCH_LEAD
            close = max(due, free + move + settle + down)
            y = PICK_Y[side] + (close - due) * SPEED
            out.append(Waypoint(move, (BELT_X, y, hover), FINGER_OPEN, "over the belt", yaw))
            if close - down - free - move > 0.01:
                out.append(Waypoint(close - down - free - move, (BELT_X, y, hover), FINGER_OPEN, "wait", yaw))
            cx, cy = np.array(cell_centre(*cell)) + PLACE_SHIFT[side]
            out += [
                Waypoint(down, (BELT_X, y, grip_z), FINGER_OPEN, "down", yaw),
                Waypoint(catch, (BELT_X, y + 0.01, grip_z), FINGER_CLOSED, f"catch pack {i}", yaw),
                Waypoint(lift, (BELT_X, y + 0.02, hover), FINGER_CLOSED, "lift", yaw),
                Waypoint(carry, (cx, cy, place_z + 0.02), FINGER_CLOSED, "carry", yaw),
                Waypoint(lower, (cx, cy, place_z), FINGER_CLOSED, "lower", yaw),
                Waypoint(release, (cx, cy, place_z), FINGER_OPEN, "let go", yaw),
                Waypoint(up, (cx, cy, place_z + 0.05), FINGER_OPEN, "up", yaw),
            ]
            free, move = close + catch + lift + carry + lower + release + up, back
        out.append(Waypoint(1.0, (BELT_X - 0.1, 0.3 if side == "left" else -0.3, hover + 0.1), FINGER_OPEN, "clear", yaw))
        return out

    return {"left": arm("left", -np.pi / 2), "right": arm("right", np.pi / 2)}
