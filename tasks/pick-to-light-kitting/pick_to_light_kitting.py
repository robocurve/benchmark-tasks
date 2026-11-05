"""On the station table, in front of the left arm, a pick-to-light kitting station: four dark-blue parts bins in a row
along the outer side (bolts, spacers, nuts, washers, front to back), each stocked with three parts standing on end.
On the table along each bin's inner lip, facing the kit tray, sit two pick-light segments: a lit segment is bright
green, an unlit one dark grey. A bin with no lit segment is not on this order; a lit bin's quantity is its number of
lit segments. This order lights one segment on the bolt bin, two on the spacer bin, none on the nut bin and one on the
washer bin. Nearer the middle lies one white kit tray. The kit is one bolt, two spacers and one washer, and nothing
from the nut bin. Scored from the parts in the kit tray at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/pick-to-light-kitting/pick_to_light_kitting.py \\
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
    "spacer": (0.009, 0.020, 0.012, (0.78, 0.80, 0.84)),  # aluminium round standoff spacer
    "nut": (0.0095, 0.008, 0.010, (0.55, 0.56, 0.58)),
    "washer": (0.012, 0.006, 0.006, (0.80, 0.80, 0.78)),
}
BINS = ["bolt", "spacer", "nut", "washer"]
BIN_X = {"bolt": 0.125, "spacer": 0.250, "nut": 0.375, "washer": 0.500}  # front to back
BIN_Y = 0.400
BIN_INNER = (0.115, 0.090)
STOCK = 3
LIT = {"bolt": 1, "spacer": 2, "nut": 0, "washer": 1}  # lit segments = quantity to pick; 0 = not on this order
TRAY = (0.300, 0.200)
TRAY_INNER = (0.170, 0.080)
WALL_T = 0.006
WALL_H = 0.015
LIGHT = (0.030, 0.012, 0.010)  # one pick-light segment
LIGHT_DX = (-0.020, 0.020)  # the two segments of a bin, side by side along x
LIGHT_Y = BIN_Y - BIN_INNER[1] / 2 - WALL_T - 0.002 - LIGHT[1] / 2  # on the table, 2 mm off the inner lip

BLUE = (0.10, 0.20, 0.45)
WHITE = (0.92, 0.92, 0.92)
LIT_GREEN = (0.30, 1.00, 0.35)
UNLIT = (0.16, 0.17, 0.18)
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
for _kind in BINS:
    _r, _h, _m, _c = PARTS[_kind]
    for _k in range(STOCK):
        _dx, _dy = SPOTS[_k]
        PART_OBJECTS.append(Cylinder(f"{_kind}_{_k}", pos=(BIN_X[_kind] + _dx, BIN_Y + _dy, TOP + _h / 2), radius=_r,
                                     height=_h, mass=_m, color=_c))
KIND_OF = {p.name: p.name.split("_")[0] for p in PART_OBJECTS}
FIXTURES = [w for k in BINS for w in _walls(f"{k}_bin", (BIN_X[k], BIN_Y), BIN_INNER, BLUE)]
FIXTURES += [
    Box(f"{k}_light_{j + 1}", pos=(BIN_X[k] + dx, LIGHT_Y, TOP + LIGHT[2] / 2), size=LIGHT,
        color=LIT_GREEN if j < LIT[k] else UNLIT)
    for k in BINS for j, dx in enumerate(LIGHT_DX)
]
FIXTURES += _walls("kit_tray", TRAY, TRAY_INNER, WHITE)
OBJECTS = [*PART_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
UNLIT_PENALTY = 0.25  # per part from a bin that is not lit


def _bottom(state, kind) -> float:
    r, h, _, _ = PARTS[kind]
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    uz = 1.0 - 2.0 * (x * x + y * y)
    return float(state.pos[2]) - (h / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _in_tray(state, kind) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - TRAY[0]) < TRAY_INNER[0] / 2 and abs(y - TRAY[1]) < TRAY_INNER[1] / 2
    return bool(inside and state.resting() and TOP - 0.005 < _bottom(state, kind) < TOP + 0.050)


def contents(objects):
    """Parts resting in the kit tray, counted by the bin they came from."""
    out = dict.fromkeys(BINS, 0)
    for part in PART_OBJECTS:
        kind = KIND_OF[part.name]
        if _in_tray(objects[part.name], kind):
            out[kind] += 1
    return out


def success(objects) -> bool:
    """Exactly the lit quantity from every lit bin in the kit tray, nothing from the unlit bin; holds for 1 s."""
    c = contents(objects)
    return all(c[k] == LIT[k] for k in BINS)


def score(objects) -> float:
    """Each lit bin is a third of the score: full when the tray holds exactly its lit quantity, a share for a short
    pick (one of two spacers earns half its third), nothing for an over-pick. Every part in the tray from an unlit bin
    takes 0.25 off (not below 0). The empty tray scores 0."""
    c = contents(objects)
    lit = [k for k in BINS if LIT[k]]
    s = sum((c[k] / LIT[k] if c[k] <= LIT[k] else 0.0) for k in lit) / len(lit)
    s -= UNLIT_PENALTY * sum(c[k] for k in BINS if not LIT[k])
    return max(0.0, s)


def distance(objects) -> float:
    c = contents(objects)
    return float(sum(abs(c[k] - LIT[k]) for k in BINS))


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.100
SQUEEZE = 0.008
CLEAR = 0.020
TIP = 0.013
TRAY_SPOTS = [(-0.057, 0.0), (-0.019, 0.0), (0.019, 0.0), (0.057, 0.0)]  # 38 mm apart along x, clear of the pads


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
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{kind}: settle over the tray"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{kind}: set in"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{kind}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{kind}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{kind}: retreat"),
    ]


def demo():
    """Left arm only: one bolt, two spacers and one washer into the kit tray, nothing from the nut bin. 52.8 s."""
    picks = [("bolt", 1), ("spacer", 0), ("spacer", 1), ("washer", 0)]  # nearest-the-tray spots first
    moves = []
    for n, (kind, spot) in enumerate(picks):
        dx, dy = SPOTS[spot]
        tx, ty = TRAY_SPOTS[n]
        moves += _move((BIN_X[kind] + dx, BIN_Y + dy), (TRAY[0] + tx, TRAY[1] + ty), kind)
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

    def put(*names, vel=(0.0, 0.0, 0.0)):
        st = dict(start)
        for k, name in enumerate(names):
            st[name] = part(KIND_OF[name], TRAY[0] - 0.070 + 0.025 * k, TRAY[1], vel=vel)
        return st

    third = 1.0 / 3.0
    cases = {
        "start": (start, 0.0, False),
        "done": (put("bolt_1", "spacer_0", "spacer_1", "washer_0"), 1.0, True),
        "done but still moving": (put("bolt_1", "spacer_0", "spacer_1", "washer_0", vel=(0.0, 0.0, -0.05)), 0.0, False),
        "done plus a nut from the unlit bin": (put("bolt_1", "spacer_0", "spacer_1", "washer_0", "nut_0"), 0.75, False),
        "one of each lit bin (one spacer short)": (put("bolt_1", "spacer_0", "washer_0"), 2.5 * third, False),
        "two bolts (over-pick)": (put("bolt_0", "bolt_1", "spacer_0", "spacer_1", "washer_0"), 2 * third, False),
        "spacers only": (put("spacer_0", "spacer_1"), third, False),
        "one of every bin": (put("bolt_0", "spacer_0", "nut_0", "washer_0"), 2.5 * third - 0.25, False),
        "nuts only": (put("nut_0", "nut_1"), 0.0, False),
        "three spacers": (put("bolt_1", "spacer_0", "spacer_1", "spacer_2", "washer_0"), 2 * third, False),
        "lit bins right, a washer dropped on the table": (put("bolt_1", "spacer_0", "spacer_1") | {"washer_0": part("washer", 0.30, 0.30)}, 2 * third, False),
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
