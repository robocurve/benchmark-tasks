"""On the station table, in front of the left arm, a kitting station for one bracket assembly: four dark-blue parts
bins in a row along the outer side (bolts, washers, flange nuts, hex nuts, front to back) and, nearer the middle,
three white kit pockets (bolts, nuts, washers). The hex-nut bin is empty. Engineering has approved the M10 flange nut
as a substitute: a flange nut has its own washer face, so one flange nut replaces a hex nut and the washer under it.
The assembly needs two bolts, each with a nut and a washer under the nut; with the substitute the kit is two bolts,
two flange nuts in the nut pocket and no washers. Scored from the parts in each pocket at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/approved-substitute-kit/approved_substitute_kit.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PARTS = {  # radius, height, mass, colour; plain cylinders standing on end
    "bolt": (0.006, 0.045, 0.035, (0.62, 0.62, 0.64)),
    "washer": (0.012, 0.006, 0.006, (0.80, 0.80, 0.78)),
    "flange": (0.0115, 0.010, 0.014, (0.80, 0.68, 0.20)),  # M10 flange nut, 23 mm flange, zinc-yellow
}
BIN_X = {"bolt": 0.150, "washer": 0.275, "flange": 0.400, "hexnut": 0.525}  # front to back; the hex-nut bin is empty
BIN_Y = 0.410
BIN_INNER = (0.115, 0.090)
STOCK = {"bolt": 3, "washer": 3, "flange": 3}
POCKETS = {"bolt": (0.180, 0.200), "nut": (0.300, 0.200), "washer": (0.420, 0.200)}
POCKET_INNER = (0.090, 0.070)
WALL_T = 0.006
WALL_H = 0.015
WANT = {"bolt": {"bolt": 2}, "nut": {"flange": 2}, "washer": {}}  # an empty washer pocket is part of the right kit

BLUE = (0.10, 0.20, 0.45)
WHITE = (0.92, 0.92, 0.92)
SPOTS = [(-0.035, 0.0), (0.0, 0.0), (0.035, 0.0)]  # in each bin, along x: clear of the 38 mm pads


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


PART_OBJECTS = []
for _kind, _n in STOCK.items():
    _r, _h, _m, _c = PARTS[_kind]
    for _k in range(_n):
        _dx, _dy = SPOTS[_k]
        PART_OBJECTS.append(Cylinder(f"{_kind}_{_k}", pos=(BIN_X[_kind] + _dx, BIN_Y + _dy, TOP + _h / 2), radius=_r,
                                     height=_h, mass=_m, color=_c))
KIND_OF = {p.name: p.name.split("_")[0] for p in PART_OBJECTS}
FIXTURES = [w for k, x in BIN_X.items() for w in _walls(f"{k}_bin", (x, BIN_Y), BIN_INNER, BLUE)]
FIXTURES += [w for k, c in POCKETS.items() for w in _walls(f"{k}_pocket", c, POCKET_INNER, WHITE)]
OBJECTS = [*PART_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _bottom(state, kind) -> float:
    r, h, _, _ = PARTS[kind]
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    uz = 1.0 - 2.0 * (x * x + y * y)
    return float(state.pos[2]) - (h / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _in_pocket(state, kind, centre) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - centre[0]) < POCKET_INNER[0] / 2 and abs(y - centre[1]) < POCKET_INNER[1] / 2
    return bool(inside and state.resting() and TOP - 0.005 < _bottom(state, kind) < TOP + 0.050)


def contents(objects):
    out = {p: dict.fromkeys(PARTS, 0) for p in POCKETS}
    for part in PART_OBJECTS:
        kind = KIND_OF[part.name]
        for p, c in POCKETS.items():
            if _in_pocket(objects[part.name], kind, c):
                out[p][kind] += 1
    return out


def _right(counts, want) -> bool:
    return all(counts[k] == want.get(k, 0) for k in PARTS)


def success(objects) -> bool:
    """Two bolts in the bolt pocket, two flange nuts in the nut pocket, nothing in the washer pocket; holds for 1 s."""
    c = contents(objects)
    return all(_right(c[p], WANT[p]) for p in POCKETS)


def score(objects) -> float:
    """0.5 for the bolt pocket right and 0.5 for the nut pocket right (exactly two flange nuts, nothing else), but only
    while the washer pocket is empty: a washer kitted for a flange nut is a wrong kit, so any part in the washer
    pocket caps the score at 0.25. The empty washer pocket earns nothing by itself, so the start scores 0."""
    c = contents(objects)
    s = 0.5 * _right(c["bolt"], WANT["bolt"]) + 0.5 * _right(c["nut"], WANT["nut"])
    if any(c["washer"].values()):
        return min(s, 0.25)
    return s


def distance(objects) -> float:
    c = contents(objects)
    return float(abs(c["bolt"]["bolt"] - 2) + abs(c["nut"]["flange"] - 2) + sum(c["washer"].values()))


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.110
SQUEEZE = 0.008
CLEAR = 0.020
TIP = 0.013


def _grip_z(kind):
    _, h, _, _ = PARTS[kind]
    return TOP + h - 0.022 + TIP if kind == "bolt" else TOP + 0.002 + TIP


def _move(pick, place, kind):
    (px, py), (qx, qy) = pick, place
    r = PARTS[kind][0]
    gz, qz = _grip_z(kind), _grip_z(kind) + 0.003
    open_ = -(2 * r + CLEAR) / 2
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
    """Left arm only: two bolts to the bolt pocket and two flange nuts to the nut pocket; no washers. 53 s."""
    moves = []
    for kind, pocket in (("bolt", "bolt"), ("flange", "nut")):
        for k, ex in enumerate((-0.020, 0.020)):
            dx, dy = SPOTS[k]
            qx, qy = POCKETS[pocket]
            moves += _move((BIN_X[kind] + dx, BIN_Y + dy), (qx + ex, qy), kind)
    return {"left": moves}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def part(kind, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0)):
        r, h, _, _ = PARTS[kind]
        pos = np.array([x, y, bottom + h / 2])
        half = np.array([r, r, h / 2])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: part(KIND_OF[p.name], p.pos[0], p.pos[1]) for p in PART_OBJECTS}

    def put(**where):
        """name -> pocket"""
        st = dict(start)
        used = dict.fromkeys(POCKETS, 0)
        for name, pocket in where.items():
            qx, qy = POCKETS[pocket]
            st[name] = part(KIND_OF[name], qx - 0.025 + 0.025 * used[pocket], qy)
            used[pocket] += 1
        return st

    kit = dict(bolt_0="bolt", bolt_1="bolt", flange_0="nut", flange_1="nut")
    cases = {
        "start": (start, 0.0, False),
        "done": (put(**kit), 1.0, True),
        "done plus two washers (no substitution logic)": (put(**kit, washer_0="washer", washer_1="washer"), 0.25, False),
        "bolts only": (put(bolt_0="bolt", bolt_1="bolt"), 0.5, False),
        "flange nuts in the washer pocket": (put(bolt_0="bolt", bolt_1="bolt", flange_0="washer", flange_1="washer"), 0.25, False),
        "three flange nuts": (put(**kit, flange_2="nut"), 0.5, False),
        "a washer in the nut pocket": (put(**kit, washer_0="nut"), 0.5, False),
        "one flange nut short": (put(bolt_0="bolt", bolt_1="bolt", flange_0="nut"), 0.5, False),
        "bolts and washers, no nuts": (put(bolt_0="bolt", bolt_1="bolt", washer_0="washer", washer_1="washer"), 0.25, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
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
