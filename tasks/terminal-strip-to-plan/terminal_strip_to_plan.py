"""On the station table, in front of the left arm, a DIN rail lies flat, running left to right, with a fixed end clamp at
its left end. Someone started the terminal strip wrongly: a blue (neutral) terminal block sits against the clamp where
the terminal plan wants a grey one. The plan for the strip, from the clamp: grey, grey, blue, green-yellow, grey
(three feed-through blocks for line conductors, one neutral, one protective earth), then a dark end clamp to lock the
row. Spare blocks of each type stand in two rows on the table nearer the arms. The wrong block must come off first,
then the strip is built in order, each block within 3 mm of the one before. Scored from the blocks' poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/terminal-strip-to-plan/terminal_strip_to_plan.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
RAIL_X = 0.380
RAIL = (0.035, 0.200, 0.0075)  # TS35 x 7.5 top-hat rail, 20 cm long, modelled as its outline
RAIL_Y = 0.330
RAIL_TOP = TOP + RAIL[2]

BLOCK = (0.055, 0.0122, 0.050)  # a 16 mm2 feed-through terminal block: 55 mm across the rail, 12.2 mm pitch, 50 mm tall
BLOCK_KG = 0.020
CLAMP = (0.045, 0.010, 0.035)  # end clamp: the fixed one at the left, a loose one to lock the right end
CLAMP_KG = 0.015
LEFT_CLAMP_Y = RAIL_Y + RAIL[1] / 2 - 0.010 - CLAMP[1] / 2  # 1 cm in from the rail's left end
STRIP_START = LEFT_CLAMP_Y - CLAMP[1] / 2  # the clamp's right face: the strip grows from here towards -y

PLAN = ("grey", "grey", "blue", "earth", "grey")  # from the clamp
COLORS = {"grey": (0.62, 0.63, 0.65), "blue": (0.15, 0.32, 0.80), "earth": (0.20, 0.62, 0.20)}
DARK = (0.12, 0.12, 0.13)
STEEL = (0.75, 0.76, 0.78)

STOCK = {  # the spares on the table, standing, 50 mm apart in two rows along y
    "grey": [(0.200, 0.400), (0.200, 0.350), (0.200, 0.300), (0.200, 0.250)],
    "blue": [(0.290, 0.400), (0.290, 0.350)],
    "earth": [(0.290, 0.300), (0.290, 0.250)],
}
LOOSE_CLAMP_AT = (0.290, 0.200)
RETURN_SPOT = (0.200, 0.210)  # where the demo stands the wrong block, in the free spare spot clear of the end clamp
WRONG = "blue"  # the block fitted against the clamp at the start
GAP_START = 0.0005


def _block(name, kind, x, y, bottom):
    return Box(name, pos=(x, y, bottom + BLOCK[2] / 2), size=BLOCK, mass=BLOCK_KG, color=COLORS[kind])


BLOCKS = [_block(f"{kind}_{k}", kind, x, y, TOP) for kind, spots in STOCK.items() for k, (x, y) in enumerate(spots)]
WRONG_BLOCK = _block("fitted_blue", WRONG, RAIL_X, STRIP_START - GAP_START - BLOCK[1] / 2, RAIL_TOP)
END_CLAMP = Box("end_clamp", pos=(*LOOSE_CLAMP_AT, TOP + CLAMP[2] / 2), size=CLAMP, mass=CLAMP_KG, color=DARK)
KIND_OF = {b.name: b.name.split("_")[0] if b.name != "fitted_blue" else WRONG for b in [*BLOCKS, WRONG_BLOCK]}
FIXTURES = [
    Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("left_clamp", pos=(RAIL_X, LEFT_CLAMP_Y, RAIL_TOP + CLAMP[2] / 2), size=CLAMP, color=DARK),
]
OBJECTS = [WRONG_BLOCK, *BLOCKS, END_CLAMP, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MAX_GAP = 0.003  # a block (or the end clamp) continues the strip within 3 mm of the one before it
SEAT_X = 0.004  # centred on the rail within 4 mm
SEAT_Z = 0.002  # bottom within 2 mm of the rail top
SEAT_DEG = 5.0  # square to the rail and upright


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _on_rail(state) -> bool:
    """Standing square on the rail's top face, centred across it, still."""
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[0]) - RAIL_X) <= SEAT_X
        and abs(float(state.low[2]) - RAIL_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= SEAT_DEG
        and state.resting()
    )


def strip(objects):
    """The kinds along the strip, from the fixed clamp: each next block standing on the rail whose left face is within
    3 mm of the previous right face (and does not overlap it by more than 1 mm). Ends at the first gap. A trailing
    'clamp' entry means the loose end clamp closes the row."""
    on = [(o.name, objects[o.name]) for o in [WRONG_BLOCK, *BLOCKS, END_CLAMP] if _on_rail(objects[o.name])]
    on.sort(key=lambda t: -float(t[1].pos[1]))
    out, edge = [], STRIP_START
    for name, st in on:
        width = CLAMP[1] if name == "end_clamp" else BLOCK[1]
        left = float(st.pos[1]) + width / 2
        if left > edge + 0.001:
            continue  # sits further left than the strip's open end: overlapping, ignore
        if edge - left > MAX_GAP:
            break
        out.append("clamp" if name == "end_clamp" else KIND_OF[name])
        edge = left - width
        if name == "end_clamp":
            break
    return out


def _matched(objects) -> int:
    """How many of the plan's six items (five blocks, then the end clamp) are in place, in order from the clamp."""
    want = [*PLAN, "clamp"]
    got = strip(objects)
    n = 0
    for w, g in zip(want, got):
        if w != g:
            break
        n += 1
    return n


def success(objects) -> bool:
    """The strip reads grey, grey, blue, green-yellow, grey from the fixed clamp, closed by the end clamp, every item
    within 3 mm of the one before; holds for 1 s."""
    return _matched(objects) == len(PLAN) + 1


def score(objects) -> float:
    """1/6 for each item of the plan in place, counted in order from the fixed clamp, so a wrong block stops the count
    there (the start, with the blue block against the clamp, scores 0)."""
    return _matched(objects) / (len(PLAN) + 1)


def distance(objects) -> float:
    """Items of the plan still to place (0 to 6), as a stand-in for metres."""
    return float(len(PLAN) + 1 - _matched(objects))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto the block's two ends
TIP = 0.013
TRAVEL = TOP + 0.120
SQUEEZE = 0.008
GRIP_DEPTH = 0.015  # the pad tips 15 mm under the block's top
PAD_HALF = 0.020  # half the pad's width along y, with margin
PLACE_GAP = 0.0012  # each item is set down this far from the one before


def _slot_y(k):
    """Centre of plan item k (0-based) when every item is PLACE_GAP from the one before."""
    return STRIP_START - (k + 1) * PLACE_GAP - (k + 0.5) * BLOCK[1]


def _move(pick, place, height, length, label, place_shift=0.0, pick_shift=0.0):
    """Top-down pick and place across the part's length. ``place_shift`` moves the grip along y off the part's centre
    towards the open end of the strip, so the pads stay clear of the item already in place. 19.6 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    gz = pb + height - GRIP_DEPTH + TIP
    qz = qb + height - GRIP_DEPTH + TIP + 0.0015
    open_ = -min(length + 0.030, 0.0939) / 2
    grip = -(length - SQUEEZE) / 2
    gy, qy2 = py + pick_shift, qy + place_shift
    return [
        Waypoint(1.8, (px, gy, TRAVEL), open_, f"{label}: over", YAW),
        Waypoint(1.5, (px, gy, gz + 0.020), open_, f"{label}: descend", YAW),
        Waypoint(1.0, (px, gy, gz), open_, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, gy, gz), open_, f"{label}: settle", YAW),
        Waypoint(1.5, (px, gy, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, gy, gz + 0.030), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, gy, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy2, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy2, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy2, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy2, qz), grip, f"{label}: set on slowly", YAW),
        Waypoint(0.6, (qx, qy2, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy2, qz), open_, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy2, TRAVEL), open_, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only. The wrong blue block comes off first, gripped off-centre towards the open side so the pads stay
    clear of the fixed clamp, and goes to the table beside the spares. Then the five plan blocks and the end clamp
    go on in order from the clamp, each gripped off-centre towards the open end (at the pick and at the place, so
    the part lands where planned) so the pads (about 38 mm wide) never touch the block before it. 137 s."""
    off = PAD_HALF - BLOCK[1] / 2 + 0.0015  # grip centre shift towards -y: the pad's +y edge 1.5 mm inside the block
    w = WRONG_BLOCK.pos
    moves = _move((w[0], w[1], RAIL_TOP), (*RETURN_SPOT, TOP), BLOCK[2], BLOCK[0], "wrong block off",
                  pick_shift=-off, place_shift=-off)
    used = dict.fromkeys(STOCK, 0)
    for k, kind in enumerate(PLAN):
        sx, sy = STOCK[kind][used[kind]]
        used[kind] += 1
        moves += _move((sx, sy, TOP), (RAIL_X, _slot_y(k), RAIL_TOP), BLOCK[2], BLOCK[0], f"{k + 1}: {kind}",
                       pick_shift=-off, place_shift=-off)
    clamp_y = _slot_y(len(PLAN) - 1) - BLOCK[1] / 2 - PLACE_GAP - CLAMP[1] / 2
    coff = PAD_HALF - CLAMP[1] / 2 + 0.0015
    moves += _move((*LOOSE_CLAMP_AT, TOP), (RAIL_X, clamp_y, RAIL_TOP), CLAMP[2], CLAMP[0], "end clamp",
                   pick_shift=-coff, place_shift=-coff)
    return {"left": moves}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])  # tilt about x, then yaw
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def blk(x, y, bottom, **kw):
        return box(BLOCK, x, y, bottom, **kw)

    start = {o.name: blk(o.pos[0], o.pos[1], o.pos[2] - BLOCK[2] / 2) for o in [WRONG_BLOCK, *BLOCKS]}
    start["end_clamp"] = box(CLAMP, *LOOSE_CLAMP_AT, TOP)
    off_rail = blk(*RETURN_SPOT, TOP)

    def built(n, gap=0.0012, kinds=PLAN, clamp=True, clamp_gap=0.0012):
        """The wrong block moved off, the first n items of ``kinds`` on the rail ``gap`` apart, then maybe the clamp."""
        st = dict(start)
        st["fitted_blue"] = off_rail
        used = dict.fromkeys(STOCK, 0)
        edge = STRIP_START
        for kind in kinds[:n]:
            name = f"{kind}_{used[kind]}"
            used[kind] += 1
            y = edge - gap - BLOCK[1] / 2
            st[name] = blk(RAIL_X, y, RAIL_TOP)
            edge = y - BLOCK[1] / 2
        if clamp:
            st["end_clamp"] = box(CLAMP, RAIL_X, edge - clamp_gap - CLAMP[1] / 2, RAIL_TOP)
        return st

    wrong_order = ("grey", "blue", "grey", "earth", "grey")
    cases = {
        "start (wrong blue block against the clamp)": (start, 0.0, False),
        "wrong block off, nothing fitted": (built(0, clamp=False), 0.0, False),
        "done": (built(5), 1.0, True),
        "done with 2.9 mm gaps": (built(5, gap=0.0029, clamp_gap=0.0029), 1.0, True),
        "done, no gaps": (built(5, gap=0.0, clamp_gap=0.0), 1.0, True),
        "three blocks right, no clamp": (built(3, clamp=False), 3 / 6, False),
        "five blocks right, no clamp": (built(5, clamp=False), 5 / 6, False),
        "five blocks, clamp 5 mm away": (built(5, clamp_gap=0.005), 5 / 6, False),
        "blue and grey swapped at 2 and 3": (built(5, kinds=wrong_order), 1 / 6, False),
        "wrong block left on, plan fitted after it": (
            {**built(5), "fitted_blue": blk(RAIL_X, WRONG_BLOCK.pos[1], RAIL_TOP)}, 0.0, False),
    }
    # the 4 mm gap case: rebuild explicitly, items 3..5 and the clamp shifted 4 mm further away
    st = built(5)
    for name in ("blue_0", "earth_0", "grey_2", "end_clamp"):
        s = st[name]
        st[name] = box(CLAMP if name == "end_clamp" else BLOCK, s.pos[0], s.pos[1] - 0.004, RAIL_TOP)
    cases["a 4 mm gap after block 2 stops the count"] = (st, 2 / 6, False)
    st = built(5)
    s = st["blue_0"]
    st["blue_0"] = blk(s.pos[0], s.pos[1], RAIL_TOP, tilt_deg=8)
    cases["block 3 leaning stops the count at 2"] = (st, 2 / 6, False)
    st = built(5)
    s = st["earth_0"]
    st["earth_0"] = blk(s.pos[0] + 0.006, s.pos[1], RAIL_TOP)
    cases["block 4 off the rail centre by 6 mm stops at 3"] = (st, 3 / 6, False)
    st = built(5)
    s = st["grey_2"]
    st["grey_2"] = blk(s.pos[0], s.pos[1], RAIL_TOP, vel=(0.0, -0.03, 0.0))
    cases["block 5 still sliding stops at 4"] = (st, 4 / 6, False)
    st = built(5)
    st["grey_3"] = blk(RAIL_X, 0.25, RAIL_TOP + BLOCK[2])  # an extra block standing on top of the strip
    cases["an extra block on top of the strip does not count"] = (st, 1.0, True)

    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success, strip(st))
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
