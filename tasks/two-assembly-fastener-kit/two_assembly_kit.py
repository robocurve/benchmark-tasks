"""On the station table, a line-side kitting bench for two bracket assemblies, one kit per arm. Each arm has three
dark-blue parts bins in a row along its outer side (bolts, nuts, washers, front to back; the washer bin is deeper) and, nearer the middle, its
half of a white kit tray: three open pockets in the same order. The bins hold more parts than one kit needs (3 bolts,
3 nuts, 5 washers each), so a full bin does not give the count away. Each assembly needs two bolts, a nut on every bolt
and a washer under every bolt head and every nut: 2 bolts, 2 nuts and 4 washers per kit, every part type in its own
pocket. The score counts the parts in each of the six pockets from their centres and axes.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/two-assembly-fastener-kit/two_assembly_kit.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
PARTS = {  # radius, height, mass, colour; plain cylinders standing on end
    "bolt": (0.006, 0.045, 0.035, (0.62, 0.62, 0.64)),  # M10 x 45 shank, 12 mm across (head omitted)
    "nut": (0.009, 0.009, 0.010, (0.30, 0.30, 0.32)),  # M10 hex nut as an 18 mm disc, 9 mm tall
    "washer": (0.012, 0.006, 0.006, (0.80, 0.80, 0.78)),  # 24 mm fender washer, thickened to 6 mm, no hole
}
NEED = {"bolt": 2, "nut": 2, "washer": 4}  # per kit: 2 bolts, a nut per bolt, a washer under every head and nut
STOCK = {"bolt": 3, "nut": 3, "washer": 5}  # per side, in the bins
ORDER = ("bolt", "nut", "washer")  # front to back, in the bins and in the tray
ROW_X = {"bolt": 0.165, "nut": 0.300, "washer": 0.435}

BIN_INNER = {"bolt": (0.120, 0.100), "nut": (0.120, 0.100), "washer": (0.120, 0.130)}  # along x, along y
POCKET_INNER = (0.090, 0.070)
WALL_T = 0.006
WALL_H = 0.015
BIN_Y = 0.400  # each side's bins, mirrored for the right arm
POCKET_Y = 0.200  # each side's half of the kit tray
SIDES = {"left": 1.0, "right": -1.0}
KITS = {"left": "A", "right": "B"}

BLUE = (0.10, 0.20, 0.45)
WHITE = (0.92, 0.92, 0.92)

# Where each part stands in its bin, relative to the bin's centre (+y away from the middle): a row along x, 35 mm
# apart, and a second row 60 mm further out for the washers. The fingers close along y and each pad face is about
# 38 mm wide along x, so a neighbour 35 mm away along x stays clear of the pads, none stands between them, and the
# pads stay inside the 120 mm bins.
BIN_SPOTS = {
    "bolt": [(-0.035, 0.0), (0.0, 0.0), (0.035, 0.0)],
    "nut": [(-0.035, 0.0), (0.0, 0.0), (0.035, 0.0)],
    "washer": [(-0.035, -0.030), (0.0, -0.030), (0.035, -0.030), (-0.0175, 0.030), (0.0175, 0.030)],
}
# Where the demo sets each part down in its pocket, relative to the pocket's centre, and on which layer: the washers
# go in two stacks of two, so the open pads stay clear of the pocket walls.
POCKET_SPOTS = {
    "bolt": [(-0.020, 0.0, 0), (0.020, 0.0, 0)],
    "nut": [(-0.020, 0.0, 0), (0.020, 0.0, 0)],
    "washer": [(-0.020, 0.0, 0), (0.020, 0.0, 0), (-0.020, 0.0, 1), (0.020, 0.0, 1)],
}


def _walls(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    side = (ix + 2 * WALL_T, WALL_T, WALL_H)
    end = (WALL_T, iy, WALL_H)
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=end, color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=end, color=color),
    ]


def bin_centre(side, kind):
    """The washer bin is deeper (130 mm across) and reaches further out than the other two."""
    out = (BIN_INNER[kind][1] - BIN_INNER["bolt"][1]) / 2
    return (ROW_X[kind], SIDES[side] * (BIN_Y + out))


def pocket_centre(side, kind):
    return (ROW_X[kind], SIDES[side] * POCKET_Y)


def _part_name(side, kind, k):
    return f"{side}_{kind}_{k}"


PART_OBJECTS = []
for _side in SIDES:
    for _kind in ORDER:
        _r, _h, _m, _c = PARTS[_kind]
        _bx, _by = bin_centre(_side, _kind)
        for _k, (_dx, _dy) in enumerate(BIN_SPOTS[_kind][: STOCK[_kind]]):
            PART_OBJECTS.append(
                Cylinder(_part_name(_side, _kind, _k), pos=(_bx + _dx, _by + SIDES[_side] * _dy, TOP + _h / 2),
                         radius=_r, height=_h, mass=_m, color=_c)
            )
KIND_OF = {p.name: p.name.split("_")[1] for p in PART_OBJECTS}

FIXTURES = []
for _side in SIDES:
    for _kind in ORDER:
        FIXTURES += _walls(f"{_side}_{_kind}_bin", bin_centre(_side, _kind), BIN_INNER[_kind], BLUE)
        FIXTURES += _walls(f"{_side}_{_kind}_pocket", pocket_centre(_side, _kind), POCKET_INNER, WHITE)

OBJECTS = [*PART_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
BAND = 0.050  # a part counts with its lowest point under 50 mm: in the pocket, or resting on parts that are
SINK = 0.005  # and no more than 5 mm under the table top (not on the floor under the table)


def _axis_z(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)  # world z of the part's axis


def _bottom(state, kind) -> float:
    """The part's lowest point from its centre and axis, standing, lying or leaning (the axis-aligned bounds of a
    turned cylinder come from its box corners, so they are not used)."""
    r, h, _, _ = PARTS[kind]
    uz = _axis_z(state.quat_xyzw)
    return float(state.pos[2]) - (h / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _in_pocket(state, kind, centre) -> bool:
    """At rest, the centre strictly inside the pocket's inner walls seen from above, the lowest point from 5 mm under
    the table top to 50 mm over it."""
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - centre[0]) < POCKET_INNER[0] / 2 and abs(y - centre[1]) < POCKET_INNER[1] / 2
    return bool(inside and state.resting() and TOP - SINK < _bottom(state, kind) < TOP + BAND)


def pocket_contents(objects):
    """{(side, pocket kind): {part kind: count}} for the six pockets."""
    out = {(s, k): dict.fromkeys(ORDER, 0) for s in SIDES for k in ORDER}
    for p in PART_OBJECTS:
        kind = KIND_OF[p.name]
        st = objects[p.name]
        for s in SIDES:
            for k in ORDER:
                if _in_pocket(st, kind, pocket_centre(s, k)):
                    out[(s, k)][kind] += 1
    return out


def _pocket_right(counts, kind) -> bool:
    return all(counts[k] == (NEED[kind] if k == kind else 0) for k in ORDER)


def success(objects) -> bool:
    """All six pockets hold exactly their part type and count, everything at rest; holds for 1 s."""
    c = pocket_contents(objects)
    return all(_pocket_right(c[(s, k)], k) for s in SIDES for k in ORDER)


def score(objects) -> float:
    """One sixth for each pocket that holds exactly the required count of its own part type and nothing else."""
    c = pocket_contents(objects)
    return sum(_pocket_right(c[(s, k)], k) for s in SIDES for k in ORDER) / 6.0


def distance(objects) -> float:
    """Metres still to go: for each pocket, the nearest required parts of its type to the pocket."""

    def gap(st, centre):
        dx = max(0.0, abs(float(st.pos[0]) - centre[0]) - POCKET_INNER[0] / 2)
        dy = max(0.0, abs(float(st.pos[1]) - centre[1]) - POCKET_INNER[1] / 2)
        return math.hypot(dx, dy)

    total = 0.0
    for s in SIDES:
        for k in ORDER:
            mine = [objects[p.name] for p in PART_OBJECTS if p.name.startswith(f"{s}_{k}_")]
            total += sum(sorted(gap(st, pocket_centre(s, k)) for st in mine)[: NEED[k]])
    return total


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.085  # low enough for the far washer bin, still clear of the 15 mm walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part, so the pads centre it under the 50 N cap
CLEAR = {"bolt": 0.030, "nut": 0.030, "washer": 0.020}  # the fingers open only this much wider than the part, clear of the walls
TIP = 0.013  # the pad tips sit this far below the TCP


def _grip_z(kind) -> float:
    """TCP height for a grip: the pads over the upper part of a bolt, or around the whole of a nut or washer with the
    tips 2 mm above the table."""
    _, h, _, _ = PARTS[kind]
    if kind == "bolt":
        return TOP + h - 0.010 + TIP - 0.012
    return TOP + 0.002 + TIP


def _place_z(kind) -> float:
    """TCP height for a release: the part's bottom 3 mm above the table."""
    return _grip_z(kind) + 0.003


def _move(pick, place, kind, layer=0):
    """A top-down pick and place paced for the simulated arm: settle before every close and release, open only
    ``CLEAR`` wider than the part inside the bin walls, close ``SQUEEZE`` narrower than it. 15.6 s."""
    (px, py), (qx, qy) = pick, place
    r = PARTS[kind][0]
    gz, qz = _grip_z(kind), _place_z(kind) + layer * PARTS[kind][1]
    open_ = -(2 * r + CLEAR[kind]) / 2
    grip = -(2 * r - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), open_, f"{kind}: over the bin"),
        Waypoint(1.5, (px, py, gz), open_, f"{kind}: descend"),
        Waypoint(0.5, (px, py, gz), open_, f"{kind}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{kind}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{kind}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{kind}: carry"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{kind}: settle over the pocket"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{kind}: set in"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{kind}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{kind}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{kind}: retreat"),
    ]


def demo():
    """Each arm kits its own half at the same time: two bolts, two nuts, then four washers (in two stacks of two), each
    from its bin to its pocket, the nearest stock first. 8 moves, 106 s per arm."""
    plans = {}
    for side, sy in SIDES.items():
        moves = []
        for kind in ORDER:
            bx, by = bin_centre(side, kind)
            qx, qy = pocket_centre(side, kind)
            for k in range(NEED[kind]):
                dx, dy = BIN_SPOTS[kind][k]
                ex, ey, layer = POCKET_SPOTS[kind][k]
                moves += _move((bx + dx, by + sy * dy), (qx + ex, qy + sy * ey), kind, layer)
        plans[side] = moves
    return plans


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat_tilt(tilt):  # tilt about world x
        return np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])

    def part(kind, x, y, bottom=TOP, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        r, h, _, _ = PARTS[kind]
        q = quat_tilt(tilt)
        z = bottom + h / 2 * abs(math.cos(tilt)) + r * abs(math.sin(tilt))
        pos = np.array([x, y, z])
        half = np.array([r, r, h / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: part(KIND_OF[p.name], p.pos[0], p.pos[1]) for p in PART_OBJECTS}

    def kitted(skip=(), extra=()):
        """The demo's end state: NEED[k] parts of each kind in each pocket; ``skip`` leaves (side, kind, k) in its
        bin, ``extra`` moves (name, state) anywhere."""
        st = dict(start)
        for side, sy in SIDES.items():
            for kind in ORDER:
                qx, qy = pocket_centre(side, kind)
                for k in range(NEED[kind]):
                    if (side, kind, k) in skip:
                        continue
                    ex, ey, layer = POCKET_SPOTS[kind][k]
                    st[_part_name(side, kind, k)] = part(kind, qx + ex, qy + sy * ey, bottom=TOP + layer * PARTS[kind][1])
        st.update(dict(extra))
        return st

    lp = pocket_centre("left", "bolt")
    ln = pocket_centre("left", "nut")
    rw = pocket_centre("right", "washer")
    cases = {
        "start": (start, 0.0, False),
        "done": (kitted(), 1.0, True),
        "one washer short on the right": (kitted(skip={("right", "washer", 3)}), 5 / 6, False),
        "a third bolt in the left bolt pocket": (kitted(extra=[("left_bolt_2", part("bolt", lp[0], lp[1] + 0.02))]), 5 / 6, False),
        "a nut in the left bolt pocket": (kitted(extra=[("left_nut_2", part("nut", lp[0], lp[1] - 0.02))]), 5 / 6, False),
        "a fifth washer in the right washer pocket": (kitted(extra=[("right_washer_4", part("washer", rw[0], rw[1] - 0.02))]), 5 / 6, False),
        "both nuts swapped into the bolt pocket": (
            kitted(extra=[("left_nut_0", part("nut", lp[0] - 0.03, lp[1])), ("left_nut_1", part("nut", lp[0] + 0.03, lp[1]))]), 4 / 6, False),
        "left kit only": (kitted(skip={("right", k, i) for k in ORDER for i in range(NEED[k])}), 0.5, False),
        "a bolt lying in its pocket counts": (kitted(extra=[("left_bolt_0", part("bolt", lp[0] - 0.02, lp[1], tilt=math.pi / 2))]), 1.0, True),
        "a nut resting on the other nut counts": (kitted(extra=[("left_nut_1", part("nut", ln[0] - 0.02, ln[1], bottom=TOP + 0.009))]), 1.0, True),
        "a nut on the pocket wall does not count": (
            kitted(extra=[("left_nut_1", part("nut", ln[0], ln[1] + POCKET_INNER[1] / 2 + WALL_T / 2, bottom=TOP + WALL_H))]), 5 / 6, False),
        "a nut held 60 mm up over its pocket does not count": (
            kitted(extra=[("left_nut_1", part("nut", ln[0], ln[1], bottom=TOP + 0.060))]), 5 / 6, False),
        "a washer still sliding does not count": (
            kitted(extra=[("right_washer_0", part("washer", rw[0] - 0.03, rw[1], vel=(0.02, 0.0, 0.0)))]), 5 / 6, False),
        "a part on the floor under a pocket does not count": (
            kitted(extra=[("right_washer_0", part("washer", rw[0], rw[1], bottom=0.0))]), 5 / 6, False),
        "every part dumped into one pocket": (
            {**start, **{p.name: part(KIND_OF[p.name], lp[0], lp[1], bottom=TOP) for p in PART_OBJECTS if p.name.startswith("left_")}},
            0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(kitted()) == 0.0 and distance(start) > 0.0
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
