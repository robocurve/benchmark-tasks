"""On the station table, in the left arm's lane, a 1 g bench analogue of stowing hazardous chemicals on a station: a
canister's hazard level sets how many layers of containment it must be stowed inside, and a canister may go into more
containment than its level asks for, never less. Four loose canisters stand in a row on a transfer plate: one red
(level 2), two yellow (level 1) and one green (level 0). Four spots wait across the table: two locker pockets, each
inside its own grey wall ring and one shared dark outer wall ring (two barriers each), one bay inside a single grey
ring (one barrier) and an open white shelf (none). There are two yellow canisters but only one single bay, so one
yellow must move up into the locker beside the red one, and that leaves the shelf to the green one. The score
is 0 if any canister is placed in a spot with fewer barriers than its level, two are placed in one spot, or any
canister ends off the table, tipped over outside a spot, or perched on a wall top or another canister; otherwise each
canister stowed upright in a spot earns a quarter. Every test uses each canister's centre and axis, not its bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/containment-level-stow/containment_level_stow.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Stow each canister upright in its own spot, a ring pocket or the white shelf, with at least as many wall rings around it as its hazard level: red needs two, yellow one, green none." \\
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
PLATE = (0.060, 0.300, 0.003)  # the transfer plate the canisters start on
PLATE_XY = (0.130, 0.245)
PLATE_TOP = TOP + PLATE[2]
ROW_X = 0.130
START_Y = {"can_red": 0.140, "can_yellow_a": 0.210, "can_yellow_b": 0.280, "can_green": 0.350}
LEVEL = {"can_red": 2, "can_yellow_a": 1, "can_yellow_b": 1, "can_green": 0}

POCKET = 0.050  # each ring's square pocket, open to the table: 5 mm of clearance per side around a canister
RING_T = 0.006
RING_H = 0.015
RING_OFF = POCKET / 2 + RING_T / 2  # 28 mm, a ring wall's centre from its pocket's centre
ENCL_T = 0.006  # the dark outer wall ring round both locker rings (its walls are the enclosure_* boxes)
ENCL_H = 0.025
ENCL_IN = (0.102, 0.187)  # the outer ring's inside: its inner faces stand 20 mm outside the two locker rings
ENCL_XY = (0.270, 0.3175)
SHELF = (0.070, 0.070, 0.020)
SHELF_TOP = TOP + SHELF[2]
SPOTS = {  # name: centre, barriers (wall rings round the spot)
    "locker_outboard": ((0.270, 0.360), 2),
    "locker_inboard": ((0.270, 0.275), 2),
    "single_bay": ((0.270, 0.175), 1),
    "shelf": ((0.270, 0.090), 0),
}
# The layout is kept inboard: 60 mm further out, the left arm at its ready pose hid the outer ring's outboard wall
# from the top camera.

PLATE_GREY = (0.80, 0.80, 0.80)
RING_GREY = (0.45, 0.45, 0.45)
ENCL_GREY = (0.25, 0.25, 0.28)
SHELF_WHITE = (0.92, 0.92, 0.90)
COLOURS = {"red": (0.85, 0.15, 0.15), "yellow": (0.95, 0.80, 0.15), "green": (0.20, 0.65, 0.30)}


def _ring(spot):
    (cx, cy), _ = SPOTS[spot]
    z = TOP + RING_H / 2
    across, along = (RING_T, POCKET, RING_H), (POCKET + 2 * RING_T, RING_T, RING_H)
    return [
        Box(f"{spot}_ring_near", pos=(cx - RING_OFF, cy, z), size=across, color=RING_GREY),
        Box(f"{spot}_ring_far", pos=(cx + RING_OFF, cy, z), size=across, color=RING_GREY),
        Box(f"{spot}_ring_inboard", pos=(cx, cy - RING_OFF, z), size=along, color=RING_GREY),
        Box(f"{spot}_ring_outboard", pos=(cx, cy + RING_OFF, z), size=along, color=RING_GREY),
    ]


_ex, _ey = ENCL_IN[0] / 2 + ENCL_T / 2, ENCL_IN[1] / 2 + ENCL_T / 2  # 54 and 96.5 mm, wall centres from the middle
_ez = TOP + ENCL_H / 2
ENCLOSURE = [
    Box("enclosure_near", pos=(ENCL_XY[0] - _ex, ENCL_XY[1], _ez), size=(ENCL_T, ENCL_IN[1] + 2 * ENCL_T, ENCL_H), color=ENCL_GREY),
    Box("enclosure_far", pos=(ENCL_XY[0] + _ex, ENCL_XY[1], _ez), size=(ENCL_T, ENCL_IN[1] + 2 * ENCL_T, ENCL_H), color=ENCL_GREY),
    Box("enclosure_inboard", pos=(ENCL_XY[0], ENCL_XY[1] - _ey, _ez), size=(ENCL_IN[0], ENCL_T, ENCL_H), color=ENCL_GREY),
    Box("enclosure_outboard", pos=(ENCL_XY[0], ENCL_XY[1] + _ey, _ez), size=(ENCL_IN[0], ENCL_T, ENCL_H), color=ENCL_GREY),
]
FIXTURES = [
    Box("transfer_plate", pos=(*PLATE_XY, TOP + PLATE[2] / 2), size=PLATE, color=PLATE_GREY),
    *[w for spot in ("locker_outboard", "locker_inboard", "single_bay") for w in _ring(spot)],
    *ENCLOSURE,
    Box("open_shelf", pos=(*SPOTS["shelf"][0], TOP + SHELF[2] / 2), size=SHELF, color=SHELF_WHITE),
]
CANISTERS = [
    Cylinder(name, pos=(ROW_X, y, PLATE_TOP + CAN_H / 2), radius=CAN_R, height=CAN_H, mass=CAN_MASS, color=COLOURS[name.split("_")[1]])
    for name, y in START_Y.items()
]
OBJECTS = [*CANISTERS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)  # stowed: the canister's axis to vertical, either end down
BOTTOM_TOL = 0.004  # stowed: lowest point to the spot's floor; a canister still held at the demo's release reads 6 mm
PLACED_RISE = 0.010  # placed: lowest point less than this above the spot's floor, so inside a ring's 15 mm walls
PERCH_TOL = 0.004  # perched: lowest point within this of a wall's or another canister's top, its underside over it
TIP_TOL = math.radians(45.0)  # tipped over: the axis more than this from vertical, outside every spot
WALLS = [  # (x, y, half x, half y, top z) of every ring wall: 15 mm tall in the grey rings, 25 mm in the outer ring
    (b.pos[0], b.pos[1], b.size[0] / 2, b.size[1] / 2, b.pos[2] + b.size[2] / 2)
    for b in FIXTURES
    if "_ring_" in b.name or b.name.startswith("enclosure_")
]
_ANG = np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False)
MARGIN = 0.002  # a stowed footprint may reach this far past the outline (solver penetration into a wall)
EPS = 1e-4  # 0.1 mm, so exact boundary values count


def _floor(spot) -> float:
    return SHELF_TOP if spot == "shelf" else TOP


def _half(spot) -> float:
    """Half the side of the spot's square outline: the 50 mm pocket or the 70 mm shelf top."""
    return SHELF[0] / 2 if spot == "shelf" else POCKET / 2


def _axis(state) -> np.ndarray:
    """The canister's axis in the world, from its quaternion."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])


def _bottom(state) -> float:
    """The canister's lowest point from its centre and axis. The axis-aligned bounds of a tilted cylinder come from
    its box corners and read too low, so they are not used."""
    uz = float(_axis(state)[2])
    return float(state.pos[2]) - (CAN_H / 2 * abs(uz) + CAN_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _placed(state):
    """The spot a canister has been put in, in any pose, or None: at rest, its centre over the spot's outline and its
    lowest point less than 10 mm above the spot's floor (inside the walls of a ring, or on the shelf top). A canister
    standing, leaning or lying in a spot counts; one on a wall top, stacked on another or held higher does not."""
    if not state.resting():
        return None
    x, y = float(state.pos[0]), float(state.pos[1])
    for spot, ((cx, cy), _) in SPOTS.items():
        rise = _bottom(state) - _floor(spot)
        if abs(x - cx) <= _half(spot) + EPS and abs(y - cy) <= _half(spot) + EPS and -BOTTOM_TOL <= rise < PLACED_RISE:
            return spot
    return None


def _stowed(state):
    """The spot a canister is stowed in, or None: placed there, axis within 10 degrees of vertical, lowest point
    within 4 mm of the spot's floor, and its footprint (both end discs seen from above) inside the spot's outline
    grown by 2 mm."""
    spot = _placed(state)
    if spot is None:
        return None
    u = _axis(state)
    if abs(float(u[2])) < math.cos(TILT_TOL) or abs(_bottom(state) - _floor(spot)) > BOTTOM_TOL + EPS:
        return None
    (cx, cy), _ = SPOTS[spot]
    reach = _half(spot) - CAN_R + MARGIN + EPS
    for sign in (-1.0, 1.0):
        ex, ey = float(state.pos[0]) + sign * CAN_H / 2 * float(u[0]), float(state.pos[1]) + sign * CAN_H / 2 * float(u[1])
        if abs(ex - cx) > reach or abs(ey - cy) > reach:
            return None
    return spot


def _top(state) -> float:
    """The canister's highest point, from its centre and axis."""
    return 2.0 * float(state.pos[2]) - _bottom(state)


def _underside(state) -> np.ndarray:
    """Points on the canister's surface within 2 mm of its lowest point, which is what it rests on: sampled on both
    end discs (rim, half radius, centre) and on 13 rings along its side, 72 points round each."""
    u = _axis(state)
    a = np.array([1.0, 0.0, 0.0]) if abs(float(u[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(u, a)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    e2 /= np.linalg.norm(e2)
    ring = np.outer(np.cos(_ANG), e1) + np.outer(np.sin(_ANG), e2)
    c = np.asarray(state.pos, float)
    pts = [c + t * CAN_H / 2 * u + r * ring for t in (-1.0, 1.0) for r in (CAN_R, CAN_R / 2)]
    pts += [(c + t * CAN_H / 2 * u)[None, :] for t in (-1.0, 1.0)]
    pts += [c + t * CAN_H / 2 * u + CAN_R * ring for t in np.linspace(-1.0, 1.0, 13)]
    pts = np.vstack(pts)
    return pts[pts[:, 2] <= _bottom(state) + 0.002]


def _over_can(xy, other) -> bool:
    """Whether any of the plan points ``xy`` lies over the other canister: within 20 mm of its axis seen from
    above."""
    u = _axis(other)
    a = np.asarray(other.pos[:2], float) - CAN_H / 2 * u[:2]
    d = CAN_H * u[:2]
    span = float(d @ d)
    t = np.clip(((xy - a) @ d) / span, 0.0, 1.0) if span > 1e-12 else np.zeros(len(xy))
    return bool(np.any(np.linalg.norm(xy - (a + t[:, None] * d), axis=1) <= CAN_R + EPS))


def _perched(name, objects) -> bool:
    """At rest on a wall top or on another canister: its lowest point within 4 mm of the top of a ring wall (15 mm
    up for a grey ring, 25 mm for the outer ring) or of another canister's highest point, with part of its underside
    over that wall or canister. A canister held still over an empty pocket at any height is not perched."""
    state = objects[name]
    if not state.resting():
        return False
    low, under = _bottom(state), _underside(state)[:, :2]
    for x, y, hx, hy, top in WALLS:
        if abs(low - top) <= PERCH_TOL + EPS and np.any((np.abs(under[:, 0] - x) <= hx) & (np.abs(under[:, 1] - y) <= hy)):
            return True
    return any(
        other != name and abs(low - _top(objects[other])) <= PERCH_TOL + EPS and _over_can(under, objects[other])
        for other in LEVEL
    )


def _mishandled(name, objects) -> bool:
    """Dropped, tipped or perched, wherever it is: its centre below the table top (off the table); its axis more
    than 45 degrees from vertical while it is not placed in a spot (tipped over or lying outside a spot); or
    perched on a wall top or on another canister."""
    state = objects[name]
    if float(state.pos[2]) < TOP:
        return True
    if _placed(state) is None and abs(float(_axis(state)[2])) < math.cos(TIP_TOL):
        return True
    return _perched(name, objects)


def _judge(placed, stowed, mishandled=()) -> float:
    """``placed`` and ``stowed`` map every canister to a spot or None; ``mishandled`` lists the canisters dropped,
    tipped or perched. Gate: 0 if any canister is placed in a spot with fewer barriers than its level, two canisters
    are placed in one spot, or any canister is mishandled. Otherwise a quarter per canister stowed."""
    spots = [s for s in placed.values() if s]
    if mishandled or len(set(spots)) < len(spots) or any(s and SPOTS[s][1] < LEVEL[n] for n, s in placed.items()):
        return 0.0
    return sum(1 for s in stowed.values() if s) / 4.0


def _states(objects):
    return ({n: _placed(objects[n]) for n in LEVEL}, {n: _stowed(objects[n]) for n in LEVEL},
            [n for n in LEVEL if _mishandled(n, objects)])


def success(objects) -> bool:
    """All four canisters stowed, one per spot, none with fewer barriers than its level and none mishandled, all at
    rest; holds 1 s."""
    return _judge(*_states(objects)) == 1.0


def _orders(items):
    """Every ordering of ``items``."""
    if not items:
        yield ()
        return
    for k, first in enumerate(items):
        for rest in _orders(items[:k] + items[k + 1:]):
            yield (first, *rest)


FULL = [  # every end layout that scores 1.0: red in a locker pocket, the yellows in the other pocket and the single bay
    dict(zip(LEVEL, spots))
    for spots in _orders(list(SPOTS))
    if all(SPOTS[s][1] >= LEVEL[n] for n, s in zip(LEVEL, spots))
]


def distance(objects) -> float:
    """Metres still to go: the least summed plan distance from each canister's centre to its spot over the four
    layouts that score 1.0. A progress signal only."""
    def far(layout):
        return sum(math.hypot(float(objects[n].pos[0]) - SPOTS[s][0][0], float(objects[n].pos[1]) - SPOTS[s][0][1]) for n, s in layout.items())

    return min(far(layout) for layout in FULL)


def score(objects) -> float:
    """0 if any canister is placed in a spot with fewer barriers than its level (red 2, yellow 1, green 0; locker
    pockets 2, single bay 1, shelf 0), two share a spot, or any ends off the table, tipped over outside a spot or
    perched on a wall top or another canister; otherwise 0.25 per canister stowed: 0, 0.25, 0.5, 0.75 or 1.0."""
    return _judge(*_states(objects))


TRAVEL = TOP + 0.140
GRIP_H = 0.038  # the TCP 38 mm above a canister's bottom: pad tips 25 mm up it, 22 mm of canister above the TCP
PICK_Z = PLATE_TOP + GRIP_H  # pad tips 25 mm above the plate
DROP = 0.007  # released with its bottom 7 mm above the floor: held, it droops to about 6 mm and never reads as stowed
RING_PLACE_Z = TOP + DROP + GRIP_H  # pad tips 32 mm up: 7 mm over the 25 mm outer ring under the open fingers
SHELF_PLACE_Z = SHELF_TOP + DROP + GRIP_H
ENTRY = 0.018  # the pause above a spot: the canister's bottom 10 mm over the 15 mm rings, 25 mm over the shelf
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
# (canister, spot, pick yaw, place yaw), degrees. At yaw -90 the fingers close along x, so plate neighbours 70 mm away
# along y sit beside the pads; +90 grips the same way but runs joint 6 past its 120 degree stop on the inboard picks.
# Green goes last with no neighbour left and is picked at -60, which keeps joint 6 at 79 degrees rather than 109.
MOVES = (
    ("can_red", "locker_inboard", -90.0, -90.0),
    ("can_yellow_a", "single_bay", -90.0, -90.0),
    ("can_yellow_b", "locker_outboard", -90.0, -90.0),
    ("can_green", "shelf", -60.0, -90.0),
)


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
    """Left arm only. Red into the inboard locker pocket, the inboard yellow into the single bay, the outboard yellow
    up into the outboard locker pocket, then green onto the shelf; each a top-down pick, a carry at 14 cm, a pause
    with the canister 10 mm over the rings and a release 7 mm above the floor. 64.8 s of plan, then the 3 s hold:
    1,356 steps at 20 Hz."""
    left = []
    for name, spot, pick_yaw, place_yaw in MOVES:
        (cx, cy), _ = SPOTS[spot]
        z = SHELF_PLACE_Z if spot == "shelf" else RING_PLACE_Z
        left += _pick_place((ROW_X, START_Y[name], PICK_Z), (cx, cy, z), width=2 * CAN_R, entry=ENTRY, travel_z=TRAVEL,
                            yaw=math.radians(pick_yaw), place_yaw=math.radians(place_yaw))
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states, and the end layouts by enumeration; runs with the real ObjectState
    or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def can(x, y, bottom=PLATE_TOP, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        """A canister tilted ``tilt`` about the world x axis with its lowest point at ``bottom``; low/high from the
        rotated box corners, as the simulator reports them."""
        q = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
        c, s = math.cos(tilt), math.sin(tilt)
        rot = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        half = np.abs(rot) @ np.array([CAN_R, CAN_R, CAN_H / 2])
        pos = np.array([x, y, bottom + CAN_H / 2 * abs(c) + CAN_R * abs(s)])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def at(spot, dx=0.0, dy=0.0, rise=0.0, **kw):
        (cx, cy), _ = SPOTS[spot]
        return can(cx + dx, cy + dy, bottom=_floor(spot) + rise, **kw)

    def scene(**changes):
        """Every canister on its start mark, except those named, which go to a spot name or a given state."""
        states = {name: can(ROW_X, y) for name, y in START_Y.items()}
        for short, where in changes.items():
            states[f"can_{short}"] = at(where) if isinstance(where, str) else where
        return states

    demo_layout = dict(red="locker_inboard", yellow_a="single_bay", yellow_b="locker_outboard", green="shelf")
    nearest = {n[4:]: min(SPOTS, key=lambda s: math.hypot(ROW_X - SPOTS[s][0][0], START_Y[n] - SPOTS[s][0][1])) for n in LEVEL}
    assert nearest == dict(red="single_bay", yellow_a="single_bay", yellow_b="locker_inboard", green="locker_outboard"), nearest
    wall = POCKET / 2 - CAN_R  # 5 mm: a canister touching a ring wall
    lie = math.pi / 2
    cases = {
        "start": (scene(), 0.0, False),
        "the demo layout": (scene(**demo_layout), 1.0, True),
        "red outboard, the yellows the other way round": (scene(red="locker_outboard", yellow_a="locker_inboard", yellow_b="single_bay", green="shelf"), 1.0, True),
        "each canister to its nearest spot": (scene(**nearest), 0.0, False),
        "nearest free spot from the inboard end: red in the single bay": (scene(red="single_bay", yellow_a="locker_inboard", yellow_b="locker_outboard", green="shelf"), 0.0, False),
        "nearest free spot from the outboard end: red on the shelf": (scene(green="locker_outboard", yellow_b="locker_inboard", yellow_a="single_bay", red="shelf"), 0.0, False),
        "one level per spot: the second yellow on the shelf, green in the locker": (scene(red="locker_inboard", yellow_a="single_bay", yellow_b="shelf", green="locker_outboard"), 0.0, False),
        "an exact match of level to spot, the second yellow left on the plate": (scene(red="locker_inboard", yellow_a="single_bay", green="shelf"), 0.75, False),
        "green in the locker, the second yellow left on the plate": (scene(red="locker_inboard", yellow_a="single_bay", green="locker_outboard"), 0.75, False),
        "green in the locker, the second yellow on the table beside the shelf": (scene(red="locker_inboard", yellow_a="single_bay", green="locker_outboard", yellow_b=can(0.200, 0.090, bottom=TOP)), 0.75, False),
        "red and both yellows right, green left on the plate": (scene(red="locker_inboard", yellow_a="single_bay", yellow_b="locker_outboard"), 0.75, False),
        "the rest stowed, red left standing on the plate": (scene(yellow_a="single_bay", yellow_b="locker_outboard", green="shelf"), 0.75, False),
        "the rest stowed, red set upright on the table beside the plate": (scene(red=can(0.200, 0.140, bottom=TOP), yellow_a="single_bay", yellow_b="locker_outboard", green="shelf"), 0.75, False),
        "only red, in a locker pocket": (scene(red="locker_outboard"), 0.25, False),
        "only green, on the shelf": (scene(green="shelf"), 0.25, False),
        "only green, in a locker pocket": (scene(green="locker_inboard"), 0.25, False),
        "only a yellow, in the single bay": (scene(yellow_b="single_bay"), 0.25, False),
        "only red, in the single bay": (scene(red="single_bay"), 0.0, False),
        "only red, on the shelf": (scene(red="shelf"), 0.0, False),
        "only a yellow, on the shelf": (scene(yellow_a="shelf"), 0.0, False),
        "red lying on the shelf, the rest right": (scene(red=at("shelf", tilt=lie), yellow_a="single_bay", yellow_b="locker_outboard", green="locker_inboard"), 0.0, False),
        "red leaning 40 degrees in the single bay": (scene(red=at("single_bay", dy=0.004, tilt=math.radians(40))), 0.0, False),
        "red held still 3 mm above the single bay floor": (scene(red=at("single_bay", rise=0.003)), 0.0, False),
        "red held still 12 mm above the shelf": (scene(red=at("shelf", rise=0.012), yellow_a="single_bay"), 0.25, False),
        "the rest stowed, red held still over its pocket at the demo's entry height": (scene(**{**demo_layout, "red": at("locker_inboard", rise=0.025)}), 0.75, False),
        "red carried over the shelf": (scene(red=at("shelf", rise=0.004, vel=(0.0, 0.05, 0.0)), yellow_a="single_bay"), 0.25, False),
        "green and a yellow both on the shelf, footprints inside": (scene(red="locker_inboard", yellow_a="single_bay", yellow_b=at("shelf", dx=-0.015, dy=-0.015), green=at("shelf", dx=0.015, dy=0.015)), 0.0, False),
        "the rest stowed, red on the single bay's wall top, its centre over the pocket": (scene(red=can(0.270, SPOTS["single_bay"][0][1] + 0.022, bottom=TOP + RING_H), yellow_a="locker_inboard", yellow_b="locker_outboard", green="shelf"), 0.0, False),
        "the rest stowed, red lying across the single bay's walls": (scene(red=at("single_bay", rise=RING_H, tilt=lie), yellow_a="locker_inboard", yellow_b="locker_outboard", green="shelf"), 0.0, False),
        "the rest stowed, red standing on the outer ring's wall top": (scene(**{**demo_layout, "red": can(ENCL_XY[0] - _ex, SPOTS["locker_inboard"][0][1], bottom=TOP + ENCL_H)}), 0.0, False),
        "the rest stowed, red stacked on the yellow in the single bay": (scene(**{**demo_layout, "red": at("single_bay", rise=CAN_H)}), 0.0, False),
        "the rest stowed, red knocked off the table onto the floor": (scene(**{**demo_layout, "red": can(0.300, 0.200, bottom=0.0, tilt=lie)}), 0.0, False),
        "the rest stowed, red tipped over on the table": (scene(**{**demo_layout, "red": can(0.180, 0.140, bottom=TOP, tilt=lie)}), 0.0, False),
        "the rest stowed, green tipped over on the plate": (scene(**{**demo_layout, "green": can(ROW_X, START_Y["can_green"], bottom=PLATE_TOP, tilt=lie)}), 0.0, False),
        "the rest stowed, green knocked off the table": (scene(**{**demo_layout, "green": can(0.300, 0.100, bottom=0.0)}), 0.0, False),
        "a yellow stacked on red in a locker pocket": (scene(red="locker_inboard", yellow_a=at("locker_inboard", rise=CAN_H)), 0.0, False),
        "done but green still moving": (scene(**{**demo_layout, "green": at("shelf", vel=(0.02, 0.0, 0.0))}), 0.75, False),
        "done but green held still at the release height, drooped to 6 mm": (scene(**{**demo_layout, "green": at("shelf", rise=0.006)}), 0.75, False),
        "done with green 4 mm up counts": (scene(**{**demo_layout, "green": at("shelf", rise=0.004)}), 1.0, True),
        "done with red standing on its other end": (scene(**{**demo_layout, "red": at("locker_inboard", tilt=math.pi)}), 1.0, True),
        "done with red tilted 8 degrees": (scene(**{**demo_layout, "red": at("locker_inboard", tilt=math.radians(8))}), 1.0, True),
        "done with red tilted 12 degrees": (scene(**{**demo_layout, "red": at("locker_inboard", tilt=math.radians(12))}), 0.75, False),
        "done with red touching a ring wall": (scene(**{**demo_layout, "red": at("locker_inboard", dx=wall, dy=-wall)}), 1.0, True),
        "done with red 1 mm into a ring wall": (scene(**{**demo_layout, "red": at("locker_inboard", dx=wall + 0.001)}), 1.0, True),
        "done with green 17 mm off the shelf centre": (scene(**{**demo_layout, "green": at("shelf", dx=0.017)}), 1.0, True),
        "done with green 18 mm off, overhanging the shelf 3 mm": (scene(**{**demo_layout, "green": at("shelf", dx=0.018)}), 0.75, False),
        "done with green lying on the shelf": (scene(**{**demo_layout, "green": at("shelf", tilt=lie)}), 0.75, False),
        "green knocked off the table": (scene(green=can(0.300, 0.100, bottom=0.0)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    # Every end layout with each canister stowed in one spot or none, 625 of them: four score 1.0, all with red in a
    # locker pocket, a yellow in the other pocket, the other yellow in the single bay and green on the shelf; 32 score
    # 0.75, 38 score 0.5, 12 score 0.25 and 539 score 0.
    tiers, full, options = {}, [], [*SPOTS, None]
    for code in range(len(options) ** 4):
        layout = {n: options[code // len(options) ** k % len(options)] for k, n in enumerate(LEVEL)}
        v = _judge(layout, layout)
        tiers[v] = tiers.get(v, 0) + 1
        if v == 1.0:
            full.append(layout)
    assert sum(tiers.values()) == 625 and tiers == {0.0: 539, 0.25: 12, 0.5: 38, 0.75: 32, 1.0: 4}, tiers
    assert sorted(map(str, full)) == sorted(map(str, FULL)) and all(f["can_green"] == "shelf" and f["can_red"].startswith("locker") for f in full), full
    return len(cases) + 2


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
