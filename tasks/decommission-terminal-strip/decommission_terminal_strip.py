"""On the station table, in front of the left arm, a DIN rail lies flat, running left to right, with a fixed end clamp
at its left end and a finished terminal strip against it: grey, green, grey, grey, green, grey blocks, closed by a loose
dark end clamp. The strip is being decommissioned. Grey feed-through blocks are scrap; the green protective-earth
blocks and the end clamp are kept for reuse. Nearer the arms stand a red scrap bin and a white reuse tray. Blocks can
only come off from the strip's open end, since a parallel gripper cannot reach between packed blocks. Scored from the
items' poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/decommission-terminal-strip/decommission_terminal_strip.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
RAIL_X = 0.380
RAIL = (0.035, 0.200, 0.0075)
RAIL_Y = 0.330
RAIL_TOP = TOP + RAIL[2]
BLOCK = (0.055, 0.0122, 0.050)
BLOCK_KG = 0.020
CLAMP = (0.045, 0.010, 0.035)
CLAMP_KG = 0.015
LEFT_CLAMP_Y = RAIL_Y + RAIL[1] / 2 - 0.010 - CLAMP[1] / 2
STRIP_START = LEFT_CLAMP_Y - CLAMP[1] / 2
GAP = 0.0005  # between items at the start

STRIP = ("grey", "earth", "grey", "grey", "earth", "grey")  # from the fixed clamp
COLORS = {"grey": (0.62, 0.63, 0.65), "earth": (0.20, 0.62, 0.20)}
DARK = (0.12, 0.12, 0.13)
STEEL = (0.75, 0.76, 0.78)
RED = (0.75, 0.12, 0.10)
WHITE = (0.92, 0.92, 0.92)

SCRAP = (0.250, 0.390)  # bin centres
REUSE = (0.250, 0.225)
BIN_INNER = (0.070, 0.130)
BIN_WALL = (0.006, 0.020)


def _block_y(k):
    return STRIP_START - (k + 1) * GAP - (k + 0.5) * BLOCK[1]


def _bin(name, centre, color):
    cx, cy = centre
    ix, iy = BIN_INNER
    t, h = BIN_WALL
    z = TOP + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


BLOCKS = [
    Box(f"block_{k + 1}_{kind}", pos=(RAIL_X, _block_y(k), RAIL_TOP + BLOCK[2] / 2), size=BLOCK, mass=BLOCK_KG,
        color=COLORS[kind])
    for k, kind in enumerate(STRIP)
]
END_Y = _block_y(len(STRIP) - 1) - BLOCK[1] / 2 - GAP - CLAMP[1] / 2
END_CLAMP = Box("end_clamp", pos=(RAIL_X, END_Y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, mass=CLAMP_KG, color=DARK)
ITEMS = [*BLOCKS, END_CLAMP]
DEST = {b.name: ("reuse" if b.name.endswith("earth") else "scrap") for b in BLOCKS} | {"end_clamp": "reuse"}
FIXTURES = [
    Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("left_clamp", pos=(RAIL_X, LEFT_CLAMP_Y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, color=DARK),
    *_bin("scrap_bin", SCRAP, RED),
    *_bin("reuse_tray", REUSE, WHITE),
]
OBJECTS = [*ITEMS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _in_bin(state, centre) -> bool:
    """At rest, the centre inside the bin's inner walls, the lowest point below the walls' top (standing, lying or
    leaning on other items in the bin)."""
    return bool(
        abs(float(state.pos[0]) - centre[0]) < BIN_INNER[0] / 2
        and abs(float(state.pos[1]) - centre[1]) < BIN_INNER[1] / 2
        and TOP - 0.005 < float(state.low[2]) < TOP + BIN_WALL[1]
        and state.resting()
    )


def where(objects):
    out = {}
    for it in ITEMS:
        st = objects[it.name]
        out[it.name] = "scrap" if _in_bin(st, SCRAP) else "reuse" if _in_bin(st, REUSE) else None
    return out


def success(objects) -> bool:
    """Every grey block in the scrap bin, both green blocks and the end clamp in the reuse tray; holds for 1 s."""
    w = where(objects)
    return all(w[n] == d for n, d in DEST.items())


def score(objects) -> float:
    """1/7 for each of the seven items (six blocks and the end clamp) in its right container. A green block or the end
    clamp in the scrap bin is lost hardware and also takes 1/7 off; floored at 0."""
    w = where(objects)
    right = sum(w[n] == d for n, d in DEST.items())
    lost = sum(w[n] == "scrap" and d == "reuse" for n, d in DEST.items())
    return max(0.0, (right - lost) / len(ITEMS))


def distance(objects) -> float:
    w = where(objects)
    return float(sum(w[n] != d for n, d in DEST.items()))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto the item's two ends
TIP = 0.013
TRAVEL = TOP + 0.120
SQUEEZE = 0.008
DEPTH = 0.015
PAD_HALF = 0.020  # half the pad's width along y, with margin
DROP = TOP + 0.005  # items are set down with their bottom 5 mm above the bin floor
SPOTS = {"scrap": [0.048, 0.016, -0.016, -0.048], "reuse": [0.032, 0.0, -0.032]}  # along y in each bin, 32 mm apart


def _move(pick_xy, pick_bottom, place_xy, size, label, pick_shift):
    """Lift an item off the open end of the strip, the grip moved ``pick_shift`` along y off its centre so the pads
    (about 38 mm wide) stay clear of the item behind it, then stand it in a bin. 19.6 s."""
    length, _, height = size
    (px, py), (qx, qy) = pick_xy, place_xy
    py += pick_shift
    gz = pick_bottom + height - DEPTH + TIP
    qz = DROP + height - DEPTH + TIP
    open_ = -min(length + 0.030, 0.0939) / 2
    grip = -(length - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), open_, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set in slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), open_, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only, from the open end: the end clamp to the reuse tray, then blocks 6 to 1, each lifted straight up
    with the grip shifted towards the open side, greys to the scrap bin and greens to the reuse tray. 124 s."""
    used = {"scrap": 0, "reuse": 0}

    def spot(dest):
        y = SPOTS[dest][used[dest]]
        used[dest] += 1
        centre = SCRAP if dest == "scrap" else REUSE
        return (centre[0], centre[1] + y)

    moves = _move(END_CLAMP.pos[:2], RAIL_TOP, spot("reuse"), CLAMP, "end clamp", -(PAD_HALF - CLAMP[1] / 2 + 0.0015))
    for b in reversed(BLOCKS):
        moves += _move(b.pos[:2], RAIL_TOP, spot(DEST[b.name]), BLOCK, b.name, -(PAD_HALF - BLOCK[1] / 2 + 0.0015))
    return {"left": moves}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    size = {it.name: (CLAMP if it.name == "end_clamp" else BLOCK) for it in ITEMS}
    start = {it.name: box(size[it.name], it.pos[0], it.pos[1], RAIL_TOP) for it in ITEMS}

    def put(**dest):
        st = dict(start)
        used = {"scrap": 0, "reuse": 0}
        for name, d in dest.items():
            centre = SCRAP if d == "scrap" else REUSE
            ys = [0.048, 0.016, -0.016, -0.048, 0.032, 0.0, -0.032]  # room for all seven in either bin
            st[name] = box(size[name], centre[0], centre[1] + ys[used[d]] * (0.9 if d == "reuse" else 1.0), TOP)
            used[d] += 1
        return st

    grey = [b.name for b in BLOCKS if b.name.endswith("grey")]
    earth = [b.name for b in BLOCKS if b.name.endswith("earth")]
    all_right = {**dict.fromkeys(grey, "scrap"), **dict.fromkeys(earth, "reuse"), "end_clamp": "reuse"}
    cases = {
        "start": (start, 0.0, False),
        "done": (put(**all_right), 1.0, True),
        "end clamp only": (put(end_clamp="reuse"), 1 / 7, False),
        "everything in the scrap bin": (put(**dict.fromkeys(all_right, "scrap")), max(0.0, (4 - 3) / 7), False),
        "everything in the reuse tray": (put(**dict.fromkeys(all_right, "reuse")), 3 / 7, False),
        "one green block scrapped": (put(**(all_right | {earth[0]: "scrap"})), (6 - 1) / 7, False),
        "greys scrapped, greens left on the rail": (put(end_clamp="reuse", **dict.fromkeys(grey[-2:], "scrap")), 3 / 7, False),
    }
    st = put(**all_right)
    st[grey[0]] = box(BLOCK, SCRAP[0], SCRAP[1], TOP + 0.030, vel=(0.0, 0.0, -0.2))  # still falling into the bin
    cases["done but a grey block still falling"] = (st, 6 / 7, False)
    st = put(**all_right)
    st[grey[1]] = box(BLOCK, SCRAP[0], SCRAP[1], TOP + 0.004, tilt_deg=90)  # lying in the bin
    cases["a grey block lying in the scrap bin counts"] = (st, 1.0, True)
    st = put(**all_right)
    st[earth[1]] = box(BLOCK, REUSE[0], REUSE[1] + BIN_INNER[1] / 2 + BIN_WALL[0] / 2, TOP + BIN_WALL[1])  # on the wall
    cases["a green block on the reuse tray wall does not count"] = (st, 6 / 7, False)
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
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
