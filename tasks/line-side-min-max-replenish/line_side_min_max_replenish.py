"""On the station table, in front of the left arm, a line-side parts point under a min-max rule. Nearer the middle stand
three blue line-side bins in a row (square nuts, hex bolts, bronze bushings, front to back), each bin for one part
type and each with a max level of three. They hold one square nut, two bolts and three bushings. Further out, along
the outer side, stand three grey supermarket bins holding three spares each, in a different order (bushings,
square nuts, bolts). The replenisher tops every line-side bin up to its max with its own part type, counting each bin's
shortfall: one bolt, no bushing and two square nuts. Scored from the parts in each line-side bin at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/line-side-min-max-replenish/line_side_min_max_replenish.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
# Part types: (shape, size, mass, colour). Cylinders stand on end; size is (radius, height) or a box's (x, y, z).
PARTS = {
    "bolt": ("cyl", (0.006, 0.045), 0.035, (0.62, 0.62, 0.64)),  # M12 x 45 hex bolt, a plain steel rod
    "bushing": ("cyl", (0.012, 0.020), 0.030, (0.72, 0.52, 0.25)),  # bronze bushing, 24 mm across, 20 mm long
    "sqnut": ("box", (0.022, 0.022, 0.012), 0.020, (0.22, 0.22, 0.24)),  # M12 square nut, 22 mm, black oxide
}
MAX_LEVEL = 3
BIN_INNER = (0.120, 0.090)
WALL_T = 0.006
WALL_H = 0.015
SPOTS = [-0.037, 0.0, 0.037]  # three places in a bin along x, 37 mm apart: clear of the 38 mm pads
LINE_Y = 0.200
LINE_X = {"sqnut": 0.165, "bolt": 0.297, "bushing": 0.429}  # front to back
STOCK_Y = 0.420
STOCK_X = {"bushing": 0.165, "sqnut": 0.297, "bolt": 0.429}  # the supermarket's order differs from the line's
START_LEVEL = {"bolt": 2, "bushing": 3, "sqnut": 1}
STOCK_LEVEL = {"bolt": 3, "bushing": 3, "sqnut": 3}
DEFICIT = sum(MAX_LEVEL - n for n in START_LEVEL.values())  # 3

BLUE = (0.10, 0.20, 0.45)
GREY = (0.55, 0.56, 0.58)


def _walls(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


def _height(kind):
    shape, size, _, _ = PARTS[kind]
    return size[1] if shape == "cyl" else size[2]


def _part(name, kind, x, y):
    shape, size, mass, color = PARTS[kind]
    z = TOP + _height(kind) / 2
    if shape == "cyl":
        return Cylinder(name, pos=(x, y, z), radius=size[0], height=size[1], mass=mass, color=color)
    return Box(name, pos=(x, y, z), size=size, mass=mass, color=color)


PART_OBJECTS = []
for _kind, _n in START_LEVEL.items():
    PART_OBJECTS += [_part(f"{_kind}_line_{k}", _kind, LINE_X[_kind] + SPOTS[k], LINE_Y) for k in range(_n)]
for _kind, _n in STOCK_LEVEL.items():
    PART_OBJECTS += [_part(f"{_kind}_spare_{k}", _kind, STOCK_X[_kind] + SPOTS[k], STOCK_Y) for k in range(_n)]
KIND_OF = {p.name: p.name.split("_")[0] for p in PART_OBJECTS}
FIXTURES = [w for k, x in LINE_X.items() for w in _walls(f"line_{k}_bin", (x, LINE_Y), BIN_INNER, BLUE)]
FIXTURES += [w for k, x in STOCK_X.items() for w in _walls(f"stock_{k}_bin", (x, STOCK_Y), BIN_INNER, GREY)]
OBJECTS = [*PART_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _in_bin(state, centre) -> bool:
    """At rest, its centre inside the bin's inner walls, its lowest point less than 5 cm above the table."""
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - centre[0]) < BIN_INNER[0] / 2 and abs(y - centre[1]) < BIN_INNER[1] / 2
    return bool(inside and state.resting() and TOP - 0.005 < float(state.low[2]) < TOP + 0.050)


def line_contents(objects):
    """Line-side bin -> {part type: count}."""
    out = {b: dict.fromkeys(PARTS, 0) for b in LINE_X}
    for part in PART_OBJECTS:
        for b, x in LINE_X.items():
            if _in_bin(objects[part.name], (x, LINE_Y)):
                out[b][KIND_OF[part.name]] += 1
    return out


def _bin_state(counts, b):
    """(level of the bin's own type, number of other parts in it)."""
    return counts[b][b], sum(n for k, n in counts[b].items() if k != b)


def success(objects) -> bool:
    """Every line-side bin holds exactly three parts of its own type and nothing else; holds for 1 s."""
    c = line_contents(objects)
    return all(_bin_state(c, b) == (MAX_LEVEL, 0) for b in LINE_X)


def score(objects) -> float:
    """1/3 per part of the shortfall filled (one bolt, two square nuts; parts above a bin's max do not count, and a part
    taken out of a line-side bin counts back). Each line-side bin that is overfilled (more than three of its type) or
    holds a part of another type costs 1/3. Clipped to 0..1, so the start scores 0."""
    c = line_contents(objects)
    filled = 0
    bad = 0
    for b in LINE_X:
        level, wrong = _bin_state(c, b)
        filled += min(level, MAX_LEVEL) - START_LEVEL[b]
        bad += int(level > MAX_LEVEL or wrong > 0)
    return float(min(1.0, max(0.0, (filled - bad) / DEFICIT)))


def distance(objects) -> float:
    c = line_contents(objects)
    return float(sum(abs(_bin_state(c, b)[0] - MAX_LEVEL) + _bin_state(c, b)[1] for b in LINE_X))


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.110
SQUEEZE = 0.008
CLEAR = 0.020
TIP = 0.013


def _width(kind):
    shape, size, _, _ = PARTS[kind]
    return 2 * size[0] if shape == "cyl" else size[1]


def _grip_z(kind):
    return TOP + _height(kind) - 0.022 + TIP if kind == "bolt" else TOP + 0.002 + TIP


def _move(pick, place, kind):
    """Top-down pick and place, the fingers closing along y. 13.2 s."""
    (px, py), (qx, qy) = pick, place
    w = _width(kind)
    gz, qz = _grip_z(kind), _grip_z(kind) + 0.003
    open_ = -(w + CLEAR) / 2
    grip = -(w - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), open_, f"{kind}: over the supermarket bin"),
        Waypoint(1.5, (px, py, gz), open_, f"{kind}: descend"),
        Waypoint(0.5, (px, py, gz), open_, f"{kind}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{kind}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{kind}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{kind}: carry"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{kind}: settle over the line-side bin"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{kind}: set in"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{kind}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{kind}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{kind}: retreat"),
    ]


def demo():
    """Left arm only: one bolt into the bolt bin's empty place, two square nuts into the square-nut bin's two empty
    places; the full bushing bin is left alone. 39.6 s."""
    moves = []
    for kind in LINE_X:
        for k in range(START_LEVEL[kind], MAX_LEVEL):
            pick = (STOCK_X[kind] + SPOTS[k - START_LEVEL[kind]], STOCK_Y)
            moves += _move(pick, (LINE_X[kind] + SPOTS[k], LINE_Y), kind)
    return {"left": moves}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def part(kind, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0)):
        shape, size, _, _ = PARTS[kind]
        half = np.array([size[0], size[0], size[1] / 2]) if shape == "cyl" else np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: part(KIND_OF[p.name], p.pos[0], p.pos[1]) for p in PART_OBJECTS}

    def put(*moves, **kw):
        """(name, line-side bin, offset along x from the bin centre)"""
        st = dict(start)
        for name, b, dx in moves:
            st[name] = part(KIND_OF[name], LINE_X[b] + dx, LINE_Y, **kw)
        return st

    bolt = ("bolt_spare_0", "bolt", SPOTS[2])
    nut1, nut2 = ("sqnut_spare_0", "sqnut", SPOTS[1]), ("sqnut_spare_1", "sqnut", SPOTS[2])
    cases = {
        "start": (start, 0.0, False),
        "done": (put(bolt, nut1, nut2), 1.0, True),
        "bolt only": (put(bolt), 1 / 3, False),
        "square nuts only": (put(nut1, nut2), 2 / 3, False),
        "one square nut short": (put(bolt, nut1), 2 / 3, False),
        "done, plus a bushing in the full bushing bin": (put(bolt, nut1, nut2, ("bushing_spare_0", "bushing", 0.045)), 2 / 3, False),
        "done, plus a fourth bolt": (put(bolt, nut1, nut2, ("bolt_spare_1", "bolt", -0.018)), 2 / 3, False),
        "all three square nuts tipped in": (put(nut1, nut2, ("sqnut_spare_2", "sqnut", 0.018)), 1 / 3, False),
        "a square nut in the bolt bin instead of a bolt": (put(("sqnut_spare_2", "bolt", SPOTS[2]), nut1, nut2), 1 / 3, False),
        "done, but a bushing taken out of its bin": (put(bolt, nut1, nut2) | {"bushing_line_2": part("bushing", 0.30, 0.30)}, 2 / 3, False),
        "done, one square nut still sliding": (put(bolt, nut1) | {"sqnut_spare_1": part("sqnut", LINE_X["sqnut"] + SPOTS[2], LINE_Y, vel=(0.05, 0, 0))}, 2 / 3, False),
        "every spare tipped into the line": (put(*((f"{k}_spare_{i}", k, 0.01 * i - 0.01) for k in PARTS for i in range(3))), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(cases["done"][0]) == 0.0
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
