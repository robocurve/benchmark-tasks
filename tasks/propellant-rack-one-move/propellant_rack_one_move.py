"""On the station table, in the left arm's lane, a 1 g bench mock-up of a compartment segregation rule for propellant
sample canisters: fuel and oxidizer that ignite on contact are kept apart, so no fuel canister may stand in a bay that
shares a wall with a bay holding oxidizer. A fixed dark grey rack of low dividers forms two rows of four square bays.
Six loose canisters stand in six bays, two blue fuel, two orange oxidizer and two white inert, and two bays are empty.
The start layout breaks the rule twice, and both breaches run through one fuel canister. Moving that canister to the
one empty bay with no oxidizer neighbour fixes the rack in one move. The nearer empty bay sits next to oxidizer, and
moving either oxidizer canister alone leaves a breach. The score is 0 for any breach, any canister out of a bay or two
in one bay; otherwise it falls with the canisters moved, those that end in a bay other than their own start bay: one
scores 1.0, two 0.6, more 0.3. Every test uses each canister's centre and axis, not its bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/propellant-rack-one-move/propellant_rack_one_move.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "No blue fuel canister may stand in a bay that shares a wall with a bay holding an orange oxidizer canister. Fix the rack by moving as few canisters as possible, and leave every canister upright in a bay." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
CAN_R = 0.020  # every canister: 40 mm across, 60 mm tall, 90 g
CAN_H = 0.060
CAN_MASS = 0.090
PITCH = 0.090  # bay centre to bay centre, both ways
WALL_T = 0.006  # every divider: 6 mm thick, 15 mm tall
WALL_H = 0.015
INNER = PITCH - WALL_T  # 84 mm, each bay's inside between the dividers
ROW_X = {"a": 0.190, "b": 0.280}  # row A near the arm, row B far
COL_Y = {1: 0.380, 2: 0.290, 3: 0.200, 4: 0.110}  # column 1 outboard to column 4 inboard
# Column 1 sits only 0.075 m outboard of the left base: with the rack 0.06 m further out, the left arm at its ready
# pose hid bays B1 and A1 from the top camera at frame 0.
BAYS = {f"{r}{c}": (x, y) for r, x in ROW_X.items() for c, y in COL_Y.items()}

START = {  # canister name: its start bay
    "fuel_a1": "a1",
    "fuel_a2": "a2",
    "oxidizer_a3": "a3",
    "oxidizer_b2": "b2",
    "inert_a4": "a4",
    "inert_b1": "b1",
}

RACK_GREY = (0.12, 0.12, 0.12)  # dark, so the 6 mm dividers stand out against the light wood table
FUEL_BLUE = (0.15, 0.35, 0.85)
OXIDIZER_ORANGE = (0.95, 0.50, 0.10)
INERT_WHITE = (0.92, 0.92, 0.92)
COLOURS = {"fuel": FUEL_BLUE, "oxidizer": OXIDIZER_ORANGE, "inert": INERT_WHITE}

# Three long dividers run across the table (along y), one in front of each row and one behind row B; ten short ones
# run forward (along x) between them, five per row, closing the bays' sides. Short ones butt onto the long ones.
X_LINES = (ROW_X["a"] - PITCH / 2, ROW_X["a"] + PITCH / 2, ROW_X["b"] + PITCH / 2)  # 0.145, 0.235, 0.325
Y_LINES = (COL_Y[1] + PITCH / 2, *(COL_Y[c] - PITCH / 2 for c in (1, 2, 3, 4)))  # 0.425, 0.335, 0.245, 0.155, 0.065
RACK_Y = (Y_LINES[0] + Y_LINES[-1]) / 2  # 0.245
LONG = (WALL_T, Y_LINES[0] - Y_LINES[-1] + WALL_T, WALL_H)  # 6 x 366 x 15 mm
SHORT = (INNER, WALL_T, WALL_H)  # 84 x 6 x 15 mm

RACK = [
    *[Box(f"rack_long_{k}", pos=(x, RACK_Y, TOP + WALL_H / 2), size=LONG, color=RACK_GREY) for k, x in enumerate(X_LINES)],
    *[Box(f"rack_short_{r}{k}", pos=(ROW_X[r], y, TOP + WALL_H / 2), size=SHORT, color=RACK_GREY) for r in ROW_X for k, y in enumerate(Y_LINES)],
]
CANISTERS = [
    Cylinder(name, pos=(*BAYS[bay], TOP + CAN_H / 2), radius=CAN_R, height=CAN_H, mass=CAN_MASS, color=COLOURS[name.split("_")[0]])
    for name, bay in START.items()
]
OBJECTS = [*CANISTERS, *RACK]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)  # the canister's axis to vertical, either end down
BOTTOM_TOL = 0.003  # lowest point to the table; a canister still held at the demo's release height reads 5 to 6 mm
EPS = 1e-4  # 0.1 mm, so an exact 3.0 mm reads as within 3 mm
FIX_BAY = "b4"  # the only one-move fix: fuel_a2 from a2 to b4
FIX_CANISTER = "fuel_a2"


def _axis_z(state) -> float:
    """World z component of the canister's axis."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)


def _bottom(state) -> float:
    """The canister's lowest point from its centre and axis. The axis-aligned bounds of a tilted cylinder come from
    its box corners and read too low, so they are not used."""
    uz = _axis_z(state)
    return float(state.pos[2]) - (CAN_H / 2 * abs(uz) + CAN_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _bay(state):
    """The bay a canister stands in, or None: at rest, axis within 10 degrees of vertical, lowest point within 3 mm
    of the table, centre inside the bay's 84 x 84 mm inner outline. With its bottom on the table between 15 mm walls,
    a canister whose centre lies inside a bay's outline is wholly inside that bay."""
    if not state.resting():
        return None
    if abs(_axis_z(state)) < math.cos(TILT_TOL) or abs(_bottom(state) - TOP) > BOTTOM_TOL + EPS:
        return None
    x, y = float(state.pos[0]), float(state.pos[1])
    for name, (cx, cy) in BAYS.items():
        if abs(x - cx) < INNER / 2 and abs(y - cy) < INNER / 2:
            return name
    return None


def _share_wall(p, q) -> bool:
    """Side by side in a row or front to back in a column; diagonal bays meet only at a corner."""
    (rp, cp), (rq, cq) = (p[0], int(p[1])), (q[0], int(q[1]))
    return (rp == rq and abs(cp - cq) == 1) or (cp == cq and rp != rq)


def _breaches(layout):
    """Every (fuel bay, oxidizer bay) pair that shares a wall."""
    fuel = [b for n, b in layout.items() if n.startswith("fuel")]
    oxidizer = [b for n, b in layout.items() if n.startswith("oxidizer")]
    return [(f, o) for f in fuel for o in oxidizer if _share_wall(f, o)]


def _moved(layout) -> int:
    """Canisters moved: those that end in a bay other than their own start bay. A canister nudged, or lifted and set
    back, inside its own bay is no move, and two canisters of one colour that trade bays are two moves. The simulator
    knows each canister by name; on the real rig each canister's top carries its start bay's label, so a top photo
    gives the same count."""
    return sum(layout[name] != bay for name, bay in START.items())


def _judge(layout) -> float:
    """``layout`` maps every canister to its bay or None. Gate: 0 if a canister is in no bay, two share a bay or a
    fuel bay shares a wall with an oxidizer bay. Otherwise by the canisters moved (``_moved``): one 1.0, two 0.6,
    three or more 0.3 (none is the start layout, which breaches)."""
    bays = list(layout.values())
    if None in bays or len(set(bays)) < len(bays) or _breaches(layout):
        return 0.0
    moved = _moved(layout)
    return 1.0 if moved == 1 else 0.6 if moved == 2 else 0.3


def _layout(objects):
    return {name: _bay(objects[name]) for name in START}


def success(objects) -> bool:
    """Compliant with exactly one canister in a new bay, every canister at rest in a bay; holds for 1 s."""
    return _judge(_layout(objects)) == 1.0


def distance(objects) -> float:
    """Metres still to go: the fuel canister that starts in bay A2 to the centre of bay B4, the one-move fix. A
    progress signal only."""
    st = objects[FIX_CANISTER]
    cx, cy = BAYS[FIX_BAY]
    return math.hypot(float(st.pos[0]) - cx, float(st.pos[1]) - cy)


def score(objects) -> float:
    """0 for any breach, any canister out of a bay or two in one bay; otherwise 1.0, 0.6 or 0.3 for one, two, or
    three or more canisters moved, those that end in a bay other than their own start bay."""
    return _judge(_layout(objects))


TRAVEL = TOP + 0.140
GRIP_Z = TOP + 0.040  # pad tips 27 mm up, 12 mm over the 15 mm dividers; the canister top 20 mm above the TCP
PLACE_Z = TOP + 0.047  # release with the canister's bottom 7 mm above the table, so a held canister never scores
ENTRY = 0.018  # the pause above the bay: the canister's bottom 25 mm up, 10 mm above the dividers
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
VIA = BAYS["b3"]  # the carry goes by way of the empty bay B3, 63.6 mm in plan from both oxidizer centres


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
    """Left arm only, yaw 0, so the fingers close across the table (along y). One move: the fuel canister in A2 to
    bay B4, carried by way of the empty bay B3 so it never passes over oxidizer. A straight carry from A2 to B4 would
    pass 40 mm in plan from the centre of A3, footprints touching; through B3 both oxidizer centres stay 63.6 mm
    away, 23.6 mm between footprints. At A2 the row neighbours stand 90 mm away along y, 70 mm from the grasp centre
    to their surface; at B4 the inert canister in A4 stands 90 mm away along x, beside the pads. The pad tips stay
    12 mm over the dividers, and the canister is let go 7 mm above the table, so it never reads as in a bay while
    still held. 17.7 s of plan, then the 3 s hold: 414 steps at 20 Hz."""
    steps = _pick_place((*BAYS["a2"], GRIP_Z), (*BAYS[FIX_BAY], PLACE_Z), width=2 * CAN_R, entry=ENTRY, travel_z=TRAVEL)
    k = next(i for i, w in enumerate(steps) if w.label == "carry")
    steps.insert(k, Waypoint(1.5, (*VIA, TRAVEL), steps[k - 1].finger, "carry over the empty bay B3", 0.0))
    return {"left": steps}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states, and the one-move fix confirmed by enumeration; runs with the real
    ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def can(x, y, bottom=TOP, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        """A canister tilted ``tilt`` about the world x axis with its lowest point at ``bottom``; low/high from the
        rotated box corners, as the simulator reports them."""
        q = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
        c, s = math.cos(tilt), math.sin(tilt)
        rot = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        half = np.abs(rot) @ np.array([CAN_R, CAN_R, CAN_H / 2])
        pos = np.array([x, y, bottom + CAN_H / 2 * abs(c) + CAN_R * abs(s)])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def at(bay, dx=0.0, dy=0.0, **kw):
        cx, cy = BAYS[bay]
        return can(cx + dx, cy + dy, **kw)

    def scene(**changes):
        """Every canister on its start mark, except those named, which go to a bay name or a given state."""
        states = {name: at(bay) for name, bay in START.items()}
        for name, where in changes.items():
            states[name] = at(where) if isinstance(where, str) else where
        return states

    fix = {FIX_CANISTER: FIX_BAY}
    corner = INNER / 2 - CAN_R  # 22 mm: a canister touching two walls of its bay
    cases = {
        "start": (scene(), 0.0, False),
        "the one-move fix": (scene(**fix), 1.0, True),
        "the nearer empty bay, b3, next to both oxidizer canisters": (scene(fuel_a2="b3"), 0.0, False),
        "oxidizer a3 to b4 leaves a2 beside b2": (scene(oxidizer_a3="b4"), 0.0, False),
        "oxidizer a3 to b3 leaves a2 beside b2": (scene(oxidizer_a3="b3"), 0.0, False),
        "oxidizer b2 to b4 leaves a2 beside a3": (scene(oxidizer_b2="b4"), 0.0, False),
        "oxidizer b2 to b3 leaves a2 beside a3": (scene(oxidizer_b2="b3"), 0.0, False),
        "fuel a1 to b4 leaves a2 as it was": (scene(fuel_a1="b4"), 0.0, False),
        "an inert canister moved alone": (scene(inert_a4="b4"), 0.0, False),
        "both oxidizer canisters moved, a3 to b4 and b2 to b3": (scene(oxidizer_a3="b4", oxidizer_b2="b3"), 0.6, False),
        "the fix plus an inert canister moved": (scene(**fix, inert_b1="b3"), 0.6, False),
        "the fix then fuel a1 into the freed a2, beside a3": (scene(**fix, fuel_a1="a2"), 0.0, False),
        "three moved, compliant": (scene(**fix, inert_a4="a2", inert_b1="b3"), 0.3, False),
        "the fix, white b1 to b3 and white a4 into b1: three moved": (scene(**fix, inert_b1="b3", inert_a4="b1"), 0.3, False),
        "the whole rack rebuilt, six moved": (scene(fuel_a1="b3", fuel_a2="b4", oxidizer_a3="a1", oxidizer_b2="b1", inert_a4="a2", inert_b1="a3"), 0.3, False),
        "blue a1 to b4, then blue a2 into the freed a1: two moved": (scene(fuel_a1="b4", fuel_a2="a1"), 0.6, False),
        "orange a3 to b4, then blue a2 into the freed a3: two moved": (scene(oxidizer_a3="b4", fuel_a2="a3"), 0.6, False),
        "the fix with the two orange canisters traded: three moved": (scene(**fix, oxidizer_a3="b2", oxidizer_b2="a3"), 0.3, False),
        "the fix with the two white canisters traded: three moved": (scene(**fix, inert_a4="b1", inert_b1="a4"), 0.3, False),
        "the two blue canisters traded: a2 still breaches": (scene(fuel_a1="a2", fuel_a2="a1"), 0.0, False),
        "the fix with fuel a1 nudged 15 mm inside its own bay": (scene(**fix, fuel_a1=at("a1", dx=0.015, dy=-0.015)), 1.0, True),
        "the fix touching two walls of b4": (scene(**{FIX_CANISTER: at(FIX_BAY, dx=corner, dy=-corner)}), 1.0, True),
        "the fix leaning 8 degrees": (scene(**{FIX_CANISTER: at(FIX_BAY, tilt=math.radians(8))}), 1.0, True),
        "the fix leaning 12 degrees": (scene(**{FIX_CANISTER: at(FIX_BAY, tilt=math.radians(12))}), 0.0, False),
        "the fix standing on its other end": (scene(**{FIX_CANISTER: at(FIX_BAY, tilt=math.pi)}), 1.0, True),
        "the fix lying in b4": (scene(**{FIX_CANISTER: at(FIX_BAY, tilt=math.pi / 2)}), 0.0, False),
        "the fix with inert a4 knocked over in its bay": (scene(**fix, inert_a4=at("a4", tilt=math.pi / 2)), 0.0, False),
        "the fix still moving": (scene(**{FIX_CANISTER: at(FIX_BAY, vel=(0.0, 0.0, 0.02))}), 0.0, False),
        "the fix held still 3 mm up counts": (scene(**{FIX_CANISTER: at(FIX_BAY, bottom=TOP + 0.003)}), 1.0, True),
        "the fix held still 4 mm up does not": (scene(**{FIX_CANISTER: at(FIX_BAY, bottom=TOP + 0.004)}), 0.0, False),
        "the fix held still at the release height, drooping 1.5 mm, does not": (scene(**{FIX_CANISTER: at(FIX_BAY, bottom=PLACE_Z - GRIP_Z + TOP - 0.0015)}), 0.0, False),
        "the fix with inert a4 squeezed into b4 too": (scene(**{FIX_CANISTER: at(FIX_BAY, dx=corner, dy=corner), "inert_a4": at(FIX_BAY, dx=-corner, dy=-corner)}), 0.0, False),
        "fuel a2 on the divider between a2 and b2": (scene(fuel_a2=can(X_LINES[1], COL_Y[2], bottom=TOP + WALL_H)), 0.0, False),
        "fuel a2 outside the rack on the table": (scene(fuel_a2=can(0.380, 0.300)), 0.0, False),
        "fuel a2 stacked on inert a4": (scene(fuel_a2=at("a4", bottom=TOP + CAN_H)), 0.0, False),
        "fuel a2 off the table": (scene(fuel_a2=can(0.300, 0.300, bottom=0.0)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    # Every single move of one canister into an empty bay: only fuel a2 to b4 is compliant.
    empty = [b for b in BAYS if b not in START.values()]
    fixes = [(n, b) for n in START for b in empty if _judge({**START, n: b}) == 1.0]
    assert fixes == [(FIX_CANISTER, FIX_BAY)], fixes
    # Every end layout with one canister per bay, 20,160 of them: one scores 1.0 (the fix), nine 0.6, 2,390 0.3
    # and 17,760 0.
    def layouts(names, free):
        if not names:
            yield {}
            return
        for b in free:
            for rest in layouts(names[1:], [f for f in free if f != b]):
                yield {names[0]: b, **rest}

    tiers, full = {}, []
    for layout in layouts(list(START), list(BAYS)):
        v = _judge(layout)
        tiers[v] = tiers.get(v, 0) + 1
        if v == 1.0:
            full.append(layout)
    assert sum(tiers.values()) == 20160 and tiers == {1.0: 1, 0.6: 9, 0.3: 2390, 0.0: 17760}, tiers
    assert full == [{**START, FIX_CANISTER: FIX_BAY}], full
    return len(cases) + 3


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
