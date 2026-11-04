"""On the station table, in the left arm's lane, a 1 g bench analogue of cargo load planning: four cargo blocks go
into a four-slot rack heaviest first, slot 1 being the one beside a red bar. The masses are not shown or measured.
The instruction gives each colour's material and density and says all four blocks are 30 mm tall, and the
footprints are read from the scene, so each mass is footprint times height times density: white nylon 115 g, dark
grey steel 77 g, silver aluminium 49 g, blue foam 26 g. With one height for all, the footprint seen from above
carries the whole volume. The set is built so that ordering by density alone or by size alone scores 0. The score
counts, of the six pairs of blocks, those whose slot order matches their mass order, once all four rest flat
(within 4 degrees) in four different slots: six pairs 1.0, five 0.5, else 0. Every horizontal test and the bottom
height use the block centre and its quaternion.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/cargo-mass-from-material/cargo_mass_from_material.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Load the four cargo blocks into the rack in order of mass, one per slot, heaviest in slot 1 by the red bar and lightest in slot 4. Dark grey is steel (7.9 g/cm3), silver aluminium (2.7), white nylon (1.15), blue foam (0.25). All are 30 mm tall." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
POCKET = (0.074, 0.064)  # each slot's open pocket inside its walls, x by y; the floor is the table
WALL_T = 0.006
WALL_H = 0.008
PITCH = POCKET[1] + WALL_T  # 70 mm, slot centre to slot centre; neighbouring slots share a wall
RACK_X = 0.280
RACK_Y = 0.305  # the rack is centred in front of the left base
SLOT_Y = tuple(round(RACK_Y + (k - 1.5) * PITCH, 4) for k in range(4))  # slot 1 inboard: 0.200, 0.270, 0.340, 0.410
RACK_LEN = 4 * POCKET[1] + 5 * WALL_T  # 286 mm along y
RED_BAR = (0.086, 0.012, 0.003)  # flat on the table along the outside of slot 1's end wall
ROW_X = 0.140

DARK_GREY = (0.30, 0.31, 0.33)
SILVER = (0.72, 0.74, 0.77)
WHITE = (0.93, 0.93, 0.91)
BLUE = (0.20, 0.40, 0.80)
RACK_GREEN = (0.18, 0.42, 0.25)
RED = (0.80, 0.12, 0.10)

# name: size (x, y, z) in m as it stands at the start, density in g/cm3, colour, start y on the row at ROW_X. All
# four are HEIGHT tall, so the footprint seen from above carries the whole volume (the instruction says so).
HEIGHT = 0.030
CARGO = {
    "steel": ((0.018, 0.018, HEIGHT), 7.9, DARK_GREY, 0.195),
    "foam": ((0.064, 0.054, HEIGHT), 0.25, BLUE, 0.265),
    "aluminium": ((0.024, 0.025, HEIGHT), 2.7, SILVER, 0.338),
    "nylon": ((0.064, 0.052, HEIGHT), 1.15, WHITE, 0.410),
}
SIZE = {n: c[0] for n, c in CARGO.items()}
DENSITY = {n: c[1] for n, c in CARGO.items()}
MASS = {n: s[0] * s[1] * s[2] * d * 1000.0 for n, (s, d, _, _) in CARGO.items()}  # kg: 76.8, 25.9, 48.6, 114.8 g
START_Y = {n: c[3] for n, c in CARGO.items()}
ORDER = tuple(sorted(MASS, key=lambda n: -MASS[n]))  # heaviest first: nylon, steel, aluminium, foam

BLOCKS = [
    Box(f"cargo_{n}", pos=(ROW_X, START_Y[n], TOP + s[2] / 2), size=s, mass=MASS[n], color=col)
    for n, (s, _, col, _) in CARGO.items()
]
_wz = TOP + WALL_H / 2
_wx = POCKET[0] / 2 + WALL_T / 2
RACK = [
    Box("rack_wall_near", pos=(RACK_X - _wx, RACK_Y, _wz), size=(WALL_T, RACK_LEN, WALL_H), color=RACK_GREEN),
    Box("rack_wall_far", pos=(RACK_X + _wx, RACK_Y, _wz), size=(WALL_T, RACK_LEN, WALL_H), color=RACK_GREEN),
    *[
        Box(f"rack_divider_{k}", pos=(RACK_X, round(SLOT_Y[0] + (k - 0.5) * PITCH, 4), _wz), size=(POCKET[0], WALL_T, WALL_H), color=RACK_GREEN)
        for k in range(5)
    ],
]
SLOT1_BAR = Box("slot1_red_bar", pos=(RACK_X, RACK_Y - RACK_LEN / 2 - RED_BAR[1] / 2, TOP + RED_BAR[2] / 2), size=RED_BAR, color=RED)
OBJECTS = [*BLOCKS, *RACK, SLOT1_BAR]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# One face down: some body axis within 4 degrees of vertical. A box flat on the floor tilts about 0 degrees, and
# propping one on an 8 mm wall top tilts it at least asin(8 / 83.7) = 5.5 degrees even across the largest face
# diagonal (the foam's 64 x 54 mm), so no propped block passes.
FLAT_TOL = math.radians(4.0)
BOTTOM_TOL = 0.005  # the block's lowest point to the table, which is the pocket floor; the walls are 8 mm
EPS = 1e-4  # 0.1 mm, so a value exactly on a limit passes


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _slot(state, size):
    """The slot (0 to 3) the block is loaded in, or None. Loaded: at rest, flat on one of its faces (any face, any
    yaw), its lowest point within 5 mm of the pocket floor and its centre inside one pocket's outline. The bottom
    comes from the centre and the quaternion, since a yawed box reports enlarged axis-aligned bounds."""
    if not state.resting():
        return None
    rot = _rot(state.quat_xyzw)
    if max(abs(float(rot[2, i])) for i in range(3)) < math.cos(FLAT_TOL):
        return None
    bottom = float(state.pos[2]) - sum(abs(float(rot[2, i])) * size[i] / 2 for i in range(3))
    if abs(bottom - TOP) > BOTTOM_TOL + EPS:
        return None
    if abs(float(state.pos[0]) - RACK_X) > POCKET[0] / 2 + EPS:
        return None
    for k, sy in enumerate(SLOT_Y):
        if abs(float(state.pos[1]) - sy) <= POCKET[1] / 2 + EPS:
            return k
    return None


def _pairs(slots) -> int:
    """Of the six pairs of blocks, those with the heavier block in the lower-numbered slot."""
    n = 0
    for i, a in enumerate(ORDER):
        for b in ORDER[i + 1 :]:
            n += slots[a] < slots[b]
    return n


def _level(objects) -> float:
    slots = {n: _slot(objects[f"cargo_{n}"], SIZE[n]) for n in CARGO}
    if any(s is None for s in slots.values()) or len(set(slots.values())) < len(slots):
        return 0.0  # fewer than four loaded, or two in one slot
    pairs = _pairs(slots)
    return 1.0 if pairs == 6 else 0.5 if pairs == 5 else 0.0


def success(objects) -> bool:
    """All four loaded, one per slot, in mass order from slot 1; holds for 1 s."""
    return _level(objects) == 1.0


def distance(objects) -> float:
    """Metres still to go: each block's centre to the centre of the slot its mass order calls for, summed."""
    return sum(
        math.hypot(float(objects[f"cargo_{n}"].pos[0]) - RACK_X, float(objects[f"cargo_{n}"].pos[1]) - SLOT_Y[k])
        for k, n in enumerate(ORDER)
    )


def score(objects) -> float:
    """With all four blocks loaded, one per slot (at rest, flat, lowest point within 5 mm of the pocket floor, centre
    inside the pocket), the pairs whose slot order matches their mass order: six 1.0, five (one neighbouring swap)
    0.5, four or fewer 0. Fewer than four loaded, or two in one slot, 0."""
    return _level(objects)


TRAVEL = TOP + 0.145
TIP = 0.013  # the pad tips sit 13 mm below the TCP
GRIP_Z = TOP + 0.010 + TIP  # pad tips 10 mm above the table: 20 mm of each block's side between the pads
PLACE_Z = GRIP_Z + 0.005  # block bottom 5 mm above the pocket floor; pad tips 15 mm up, 7 mm over the 8 mm walls
ENTRY = 0.013  # the pause above a slot: block bottom 18 mm up, 10 mm above the walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part

# Each move: block, slot index (0 is slot 1), wrist yaw. Yaw +-90 degrees closes the fingers along x, so the
# neighbours along y on the row and in the rack sit beside the pads. Joint 6 runs at joint 1 minus the yaw and must
# stay inside +-120 degrees, so the inboard steel pick (joint 1 -38 degrees) takes -90 and every other move +90; the
# steel goes first, and one quarter-turn pause above its slot splits the half turn between the two signs.
Q = math.pi / 2
MOVES = (("steel", 1, -Q), ("nylon", 0, Q), ("aluminium", 2, Q), ("foam", 3, Q))


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
    """Left arm only. Steel from the inboard end of the row into slot 2, then nylon from the outboard end into slot 1,
    aluminium into slot 3 and foam into slot 4. Every grip is top-down with the fingers closing along x across the
    block's x side (18, 64, 24 and 64 mm), so the neighbours along y stay beside the pads, and every block keeps its
    start orientation. Each move carries at 14.5 cm, pauses with the block 10 mm above the walls and releases it
    5 mm above the pocket floor. After the steel, the open wrist turns to yaw 0 above slot 2 for 1 s before the
    next move turns it on to +90. 65.8 s of plan, then the 3 s hold: 1,376 steps at 20 Hz."""
    left, prev = [], None
    for n, k, yaw in MOVES:
        if prev is not None and abs(yaw - prev) > Q + 1e-9:  # a half turn goes as two quarter turns
            left.append(Waypoint(1.0, left[-1].pos, FINGER_OPEN, "turn the wrist", (yaw + prev) / 2))
        left += _pick_place(
            (ROW_X, START_Y[n], GRIP_Z), (RACK_X, SLOT_Y[k], PLACE_Z), width=SIZE[n][0], entry=ENTRY, travel_z=TRAVEL, yaw=yaw
        )
        prev = yaw
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, tilt=0.0, axis=0.0):  # yaw about the vertical, then tilt about the block's own horizontal axis
        # at angle `axis` from its x axis
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        ca, sa = math.cos(axis), math.sin(axis)
        return np.array([st * (cy * ca - sy * sa), st * (cy * sa + sy * ca), sy * ct, cy * ct])

    def block(n, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0), **kw):
        q = quat(**kw)
        half = np.abs(_rot(q)) @ (np.asarray(SIZE[n]) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def loaded(n, k, dx=0.0, dy=0.0, **kw):
        return block(n, RACK_X + dx, SLOT_Y[k] + dy, **kw)

    def scene(order=(), **over):
        states = {n: block(n, ROW_X, START_Y[n]) for n in CARGO}
        for k, n in enumerate(order):
            if n is not None:
                states[n] = loaded(n, k)
        states.update(over)
        return {f"cargo_{n}": st for n, st in states.items()}

    N, S, A, F = "nylon", "steel", "aluminium", "foam"
    TRUE = (N, S, A, F)
    assert ORDER == TRUE, ORDER
    assert {n: k for n, k, _ in MOVES} == {n: k for k, n in enumerate(ORDER)}  # the demo loads the true order
    assert all(SIZE[n][2] == HEIGHT == 0.030 for n in CARGO)  # one height, as the instruction says
    grams = [round(MASS[n] * 1000, 1) for n in ORDER]
    assert grams == [114.8, 76.8, 48.6, 25.9], grams  # volume times density, the grams the text states
    assert all(a / b > 1.45 for a, b in zip(grams, grams[1:])), grams  # each step 1.5 to 1.9 times
    by_density = tuple(sorted(CARGO, key=lambda n: -DENSITY[n]))
    by_size = tuple(sorted(CARGO, key=lambda n: -SIZE[n][0] * SIZE[n][1] * SIZE[n][2]))
    row = tuple(sorted(CARGO, key=lambda n: START_Y[n]))  # inboard to outboard, the block in front of each slot
    assert by_density == (S, A, N, F) and by_size == (F, N, A, S) and row == (S, F, A, N)
    # The top view: footprint area alone is the size order, and footprint times density is the mass order.
    by_area = tuple(sorted(CARGO, key=lambda n: -SIZE[n][0] * SIZE[n][1]))
    by_area_density = tuple(sorted(CARGO, key=lambda n: -SIZE[n][0] * SIZE[n][1] * DENSITY[n]))
    assert by_area == by_size and by_area_density == TRUE

    def perm(order):
        return _pairs({n: k for k, n in enumerate(order)})

    assert perm(by_density) == 4 and perm(by_size) == 2 and perm(row) == 2 and perm(row[::-1]) == 4
    tally = {}
    orders = [(a, b, c, d) for a in CARGO for b in CARGO for c in CARGO for d in CARGO if len({a, b, c, d}) == 4]
    for order in orders:
        lvl = score(scene(order))
        tally[lvl] = tally.get(lvl, 0) + 1
    assert len(orders) == 24 and tally == {1.0: 1, 0.5: 3, 0.0: 20}, tally  # loading at random

    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(TRUE), 1.0, True),
        "density order, steel aluminium nylon foam, four pairs": (scene(by_density), 0.0, False),
        "size order, foam nylon aluminium steel, two pairs": (scene(by_size), 0.0, False),
        "size order with nylon and foam swapped, three pairs": (scene((N, F, A, S)), 0.0, False),
        "the reverse order": (scene(TRUE[::-1]), 0.0, False),
        "each block into the slot in front of it, two pairs": (scene(row), 0.0, False),
        "the row reversed into the slots, four pairs": (scene(row[::-1]), 0.0, False),
        "nylon and steel swapped, five pairs": (scene((S, N, A, F)), 0.5, False),
        "steel and aluminium swapped, five pairs": (scene((N, A, S, F)), 0.5, False),
        "aluminium and foam swapped, five pairs": (scene((N, S, F, A)), 0.5, False),
        "slots 1 and 3 swapped, three pairs": (scene((A, S, N, F)), 0.0, False),
        "two neighbouring swaps, four pairs": (scene((S, N, F, A)), 0.0, False),
        "three right, foam left on the row": (scene((N, S, A, None)), 0.0, False),
        "three right, foam on the table beyond slot 4": (scene((N, S, A), foam=block(F, RACK_X, 0.490)), 0.0, False),
        "steel and aluminium both in slot 2, slot 3 empty": (
            scene((N, None, None, F), steel=loaded(S, 1, dy=-0.020), aluminium=loaded(A, 1, dy=0.016)),
            0.0,
            False,
        ),
        "steel stacked on the nylon in slot 1": (scene((N, None, A, F), steel=loaded(S, 0, bottom=TOP + SIZE[N][2])), 0.0, False),
        "aluminium on the divider between slots 2 and 3": (
            scene((N, S, None, F), aluminium=block(A, RACK_X, (SLOT_Y[1] + SLOT_Y[2]) / 2, bottom=TOP + WALL_H)),
            0.0,
            False,
        ),
        "aluminium held still 10 mm above slot 3": (scene((N, S, None, F), aluminium=loaded(A, 2, bottom=TOP + 0.010)), 0.0, False),
        "foam still moving in slot 4": (scene((N, S, A), foam=loaded(F, 3, vel=(0.0, 0.0, -0.02))), 0.0, False),
        "steel lying on its side in slot 2 counts": (scene((N, None, A, F), steel=loaded(S, 1, tilt=math.pi / 2)), 1.0, True),
        "aluminium yawed 30 degrees in slot 3 counts": (scene((N, S, None, F), aluminium=loaded(A, 2, yaw=math.radians(30))), 1.0, True),
        "nylon upside down in slot 1 counts": (scene((None, S, A, F), nylon=loaded(N, 0, tilt=math.pi)), 1.0, True),
        "foam tilted 3 degrees counts": (scene((N, S, A), foam=loaded(F, 3, tilt=math.radians(3))), 1.0, True),
        "foam tilted 6 degrees does not": (scene((N, S, A), foam=loaded(F, 3, tilt=math.radians(6))), 0.0, False),
        # Yawed 45 degrees the nylon spans 82 mm along x, too long for the 74 mm pocket; its smallest supported
        # resting pose leans on a wall top at 8.75 degrees (found by a search over tilt axes and offsets).
        "nylon yawed 45 degrees, propped on a wall top at 8.75 degrees, does not": (
            scene(
                (None, S, A, F),
                nylon=loaded(N, 0, dx=-0.004, dy=-0.008, yaw=math.radians(45), tilt=math.radians(8.75), axis=math.radians(105)),
            ),
            0.0,
            False,
        ),
        "steel's centre exactly on its pocket's edge counts": (scene((N, None, A, F), steel=loaded(S, 1, dy=POCKET[1] / 2)), 1.0, True),
        "steel's centre 1 mm past the edge does not": (scene((N, None, A, F), steel=loaded(S, 1, dx=POCKET[0] / 2 + 0.001)), 0.0, False),
        "a bottom exactly 5.0 mm up counts": (scene((N, S, None, F), aluminium=loaded(A, 2, bottom=TOP + 0.005)), 1.0, True),
        "a bottom 5.5 mm up does not": (scene((N, S, None, F), aluminium=loaded(A, 2, bottom=TOP + 0.0055)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(scene(TRUE)) < 1e-9 and distance(scene()) > 0.5
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
