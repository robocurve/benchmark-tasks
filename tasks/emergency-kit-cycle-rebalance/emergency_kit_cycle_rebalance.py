"""On the station table, in the left arm's lane, a 1 g bench analogue of rebalancing the emergency sets that each
module of a station keeps, as each hospital ward keeps a crash cart. Three identical grey mats lie side by side, and
each mat is a kit. Nine stand-in items are spread over them: three white masks (boxes), three yellow torches (upright
cylinders) and three red dressing packs (flatter boxes). Every kit holds three items, but each has one item twice and
lacks another, and each surplus sits in a different kit: kit 1 has two masks and no torch, kit 2 two torches and no
dressing, kit 3 two dressings and no mask. There are no spares, so items must move between kits, and the items that
already belong must stay where they are, so the only end state that scores in full is the one a three-way cycle of
the surpluses reaches; no swap between two kits gets there. A kit counts when it holds exactly one of each, each
resting on a face or an end (a torch may lie down), nothing else over its mat, and the items it kept still within
15 mm of their start places; the score is the number of complete kits over three, read from each item's centre and
axis.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/emergency-kit-cycle-rebalance/emergency_kit_cycle_rebalance.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Each grey mat is an emergency kit. Make each kit hold exactly one white mask, one yellow torch and one red dressing pack. There are no spares, so move items between the kits. Leave items that already belong where they are." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
MAT = (0.240, 0.090, 0.003)  # each kit mat, along x, along y, thick
MAT_X = 0.215  # the mats span x 0.095 to 0.335
MAT_YS = (0.420, 0.300, 0.180)  # kits 1, 2 and 3; 30 mm of bare table between neighbouring mats
MAT_TOP = TOP + MAT[2]
SPOT_XS = (0.125, 0.185, 0.245, 0.305)  # four unmarked places along each mat, 60 mm apart

MASK = (0.040, 0.050, 0.035)  # box, along x, along y, tall
DRESSING = (0.045, 0.050, 0.025)  # box
TORCH_R, TORCH_H = 0.015, 0.060  # upright cylinder
MASS = {"mask": 0.040, "torch": 0.050, "dressing": 0.030}
KINDS = ("mask", "torch", "dressing")

GREY = (0.55, 0.55, 0.58)
WHITE = (0.92, 0.92, 0.92)
YELLOW = (0.95, 0.80, 0.15)
RED = (0.80, 0.15, 0.15)

START = {  # kit index (0, 1, 2 for kits 1, 2, 3), spot index along the mat; spot 3 (x = 0.305) is free on every mat
    "mask_1": (0, 0), "mask_2": (0, 1), "dressing_1": (0, 2),
    "torch_1": (1, 0), "torch_2": (1, 1), "mask_3": (1, 2),
    "dressing_2": (2, 0), "dressing_3": (2, 1), "torch_3": (2, 2),
}


def _kind(name) -> str:
    return name.split("_")[0]


def _item(name, kit, spot):
    x, y = SPOT_XS[spot], MAT_YS[kit]
    kind = _kind(name)
    if kind == "torch":
        return Cylinder(name, pos=(x, y, MAT_TOP + TORCH_H / 2), radius=TORCH_R, height=TORCH_H, mass=MASS[kind], color=YELLOW)
    size, color = (MASK, WHITE) if kind == "mask" else (DRESSING, RED)
    return Box(name, pos=(x, y, MAT_TOP + size[2] / 2), size=size, mass=MASS[kind], color=color)


ITEMS = [_item(name, kit, spot) for name, (kit, spot) in START.items()]
MATS = [Box(f"kit_{k + 1}_mat", pos=(MAT_X, y, TOP + MAT[2] / 2), size=MAT, color=GREY) for k, y in enumerate(MAT_YS)]
OBJECTS = [*ITEMS, *MATS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)  # a box on any face, a torch on an end or lying: an own axis this near vertical or level
BOTTOM_TOL = 0.004  # lowest point to the mat top
BAND = 0.100  # an item over a mat with its lowest point under this height above the mat top is on that mat
EPS = 1e-4  # 0.1 mm, so a value exactly on a limit passes
KEEP_TOL = 0.015  # an item a kit keeps: its centre to a start place of its kind in that kit, seen from above
PERMS = ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0))


def _keep(k):
    """Kit k's start places by kind: the two places of the kind it holds twice and the one place of its single item.
    The kind it lacks has no place, so an item moved in may sit anywhere on the mat."""
    places = {}
    for name, (kit, spot) in START.items():
        if kit == k:
            places.setdefault(_kind(name), []).append((SPOT_XS[spot], MAT_YS[k]))
    return places


KEEP = [_keep(k) for k in range(3)]  # kit 1: a mask at x 0.125 or 0.185 and the dressing at 0.245; and so on


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _bottom(state, kind) -> float:
    """The item's lowest point from its centre and orientation, standing, lying or tipped. The axis-aligned bounds of
    a lying cylinder come from its box corners and read too low, so they are not used."""
    rz = _rot(state.quat_xyzw)[2]  # world z of the body's x, y and z axes
    if kind == "torch":
        uz = float(rz[2])
        return float(state.pos[2]) - (TORCH_H / 2 * abs(uz) + TORCH_R * math.sqrt(max(0.0, 1.0 - uz * uz)))
    half = np.asarray(MASK if kind == "mask" else DRESSING) / 2
    return float(state.pos[2]) - float(np.abs(rz) @ half)


def _mat_of(state):
    """The index of the mat whose outline holds the item's centre seen from above, or None."""
    x, y = float(state.pos[0]), float(state.pos[1])
    for k, my in enumerate(MAT_YS):
        if abs(x - MAT_X) <= MAT[0] / 2 + EPS and abs(y - my) <= MAT[1] / 2 + EPS:
            return k
    return None


def _set_down(state, kind) -> bool:
    """Resting on a face or an end, not leaning: a box with one of its own axes within 10 degrees of the vertical (any
    face, either way up), a torch with its axis within 10 degrees of the vertical (on an end) or of the horizontal
    (lying), and the lowest point within 4 mm of the mat top. A kit check reads contents, not pose."""
    rz = np.abs(_rot(state.quat_xyzw)[2])  # |world z| of the body's x, y and z axes
    if kind == "torch":
        posed = rz[2] >= math.cos(TILT_TOL) - 1e-9 or rz[2] <= math.sin(TILT_TOL) + 1e-9
    else:
        posed = float(rz.max()) >= math.cos(TILT_TOL) - 1e-9
    return posed and abs(_bottom(state, kind) - MAT_TOP) <= BOTTOM_TOL + EPS


def _kits(objects):
    """For each mat: the items set down in it, as (kind, x, y), and whether anything else lies over it within the band
    (still moving, leaning, tipped onto a corner or edge, on another item, or raised). Items above the band or off
    every mat count for no kit."""
    placed, spoiled = [[], [], []], [False, False, False]
    for it in ITEMS:
        st = objects[it.name]
        k = _mat_of(st)
        kind = _kind(it.name)
        if k is None or _bottom(st, kind) > MAT_TOP + BAND:
            continue
        if st.resting() and _set_down(st, kind):
            placed[k].append((kind, float(st.pos[0]), float(st.pos[1])))
        else:
            spoiled[k] = True
    return placed, spoiled


def _kept(k, items) -> bool:
    """For each kind kit k started with, its one item of that kind has its centre within 15 mm of a start place of
    that kind in kit k. Items of a kind are interchangeable, so which mask stays does not matter."""
    for kind, places in KEEP[k].items():
        x, y = next((x, y) for kd, x, y in items if kd == kind)
        if min(math.hypot(x - px, y - py) for px, py in places) > KEEP_TOL + EPS:
            return False
    return True


def _complete(objects):
    placed, spoiled = _kits(objects)
    return [
        not spoiled[k] and sorted(kd for kd, _, _ in placed[k]) == sorted(KINDS) and _kept(k, placed[k])
        for k in range(3)
    ]


def success(objects) -> bool:
    """Every kit holds exactly one mask, one torch and one dressing pack, each on a face or an end, nothing else, and
    the items it kept on their start places; holds for 1 s."""
    return all(_complete(objects))


def _gap(state, k) -> float:
    """Horizontal distance from the item's centre to mat k's outline, 0 when inside."""
    dx = max(0.0, abs(float(state.pos[0]) - MAT_X) - MAT[0] / 2)
    dy = max(0.0, abs(float(state.pos[1]) - MAT_YS[k]) - MAT[1] / 2)
    return math.hypot(dx, dy)


def _to_target(state, kind, k) -> float:
    """Horizontal distance from the item's centre to where kit k needs its kind: within 15 mm of a start place for a
    kind the kit started with, anywhere on the mat for the kind it lacks."""
    if kind in KEEP[k]:
        x, y = float(state.pos[0]), float(state.pos[1])
        return max(0.0, min(math.hypot(x - px, y - py) for px, py in KEEP[k][kind]) - KEEP_TOL)
    return _gap(state, k)


def distance(objects) -> float:
    """Metres still to go: for each kind, the cheapest one-per-kit assignment of its three items, each item's centre
    to where that kit needs it (a start place of that kind for an item the kit keeps, the mat outline otherwise)."""
    total = 0.0
    for kind in KINDS:
        names = [it.name for it in ITEMS if _kind(it.name) == kind]
        total += min(sum(_to_target(objects[n], kind, k) for n, k in zip(names, p)) for p in PERMS)
    return total


def score(objects) -> float:
    """Complete kits over three: 0, 1/3, 2/3 or 1."""
    return sum(_complete(objects)) / 3.0


TRAVEL = TOP + 0.145
GRIP_Z = {  # TCP heights; the pad tips sit 13 mm lower
    "mask": MAT_TOP + 0.021,  # 14 mm under the 35 mm mask's top, pad tips 8 mm above the mat
    "torch": MAT_TOP + 0.042,  # 18 mm under the 60 mm torch's top, pad tips 29 mm above the mat
    "dressing": MAT_TOP + 0.018,  # 7 mm under the 25 mm pack's top, pad tips 5 mm above the mat
}
DROP = 0.005  # release with the item's bottom 5 mm above the mat
ENTRY = 0.015  # the pause above a place: the item's bottom 20 mm above the mat
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
WIDTH = {"mask": MASK[1], "torch": 2 * TORCH_R, "dressing": DRESSING[1]}  # across the pads, which close along y
MOVES = (  # the three-way cycle: item, destination kit, destination spot
    ("mask_2", 2, 3),  # kit 1's extra mask to kit 3
    ("dressing_2", 1, 3),  # kit 3's extra dressing to kit 2
    ("torch_1", 0, 3),  # kit 2's extra torch to kit 1
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
    """Left arm only, yaw 0, the fingers closing along y, so the next mat's items sit 120 mm off the grasp centre and
    the same mat's neighbours sit beside the pads, not in front of them. Kit 1's extra mask to kit 3's free place,
    kit 3's extra dressing to kit 2's, kit 2's extra torch to kit 1's: three top-down pick and place moves, each
    carried at 14.5 cm and released 5 mm above the mat. 48.6 s of plan, then the 3 s hold: 1,032 steps at 20 Hz."""
    left = []
    for name, kit, spot in MOVES:
        k0, s0 = START[name]
        kind = _kind(name)
        pick = (SPOT_XS[s0], MAT_YS[k0], GRIP_Z[kind])
        place = (SPOT_XS[spot], MAT_YS[kit], GRIP_Z[kind] + DROP)
        left += _pick_place(pick, place, width=WIDTH[kind], entry=ENTRY, travel_z=TRAVEL)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw, tilt, about="x"):  # tilt about the body's own x (or y) axis, then yaw about the vertical
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        if about == "x":
            return np.array([cy * st, sy * st, sy * ct, cy * ct])
        return np.array([-sy * st, cy * st, sy * ct, cy * ct])

    def state(name, x, y, bottom=MAT_TOP, tilt=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0), about="x"):
        """An item with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator
        reports them."""
        kind = _kind(name)
        q = quat(yaw, tilt, about)
        half = np.array([TORCH_R, TORCH_R, TORCH_H / 2]) if kind == "torch" else np.asarray(MASK if kind == "mask" else DRESSING) / 2
        ext = np.abs(_rot(q)) @ half
        if kind == "torch":
            up = TORCH_H / 2 * abs(math.cos(tilt)) + TORCH_R * abs(math.sin(tilt))
        else:
            up = float(ext[2])
        pos = np.array([x, y, bottom + up])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - ext, pos + ext)

    def at(name, kit, spot, **kw):
        return state(name, SPOT_XS[spot], MAT_YS[kit], **kw)

    start = {name: at(name, kit, spot) for name, (kit, spot) in START.items()}

    def scene(*changes):
        states = dict(start)
        for name, st in changes:
            states[name] = st
        return states

    move1 = ("mask_2", at("mask_2", 2, 3))
    move2 = ("dressing_2", at("dressing_2", 1, 3))
    move3 = ("torch_1", at("torch_1", 0, 3))
    done = scene(move1, move2, move3)
    end = {**START, **{name: (kit, spot) for name, kit, spot in MOVES}}
    regroup = ["mask_1", "mask_2", "mask_3", "torch_1", "torch_2", "torch_3", "dressing_1", "dressing_2", "dressing_3"]
    off = 0.600  # on the table beyond the mats
    cases = {
        "start": (scene(), 0.0, False),
        "done by the three-way cycle": (done, 1.0, True),
        "done keeping kit 1's other mask": (scene(("mask_1", at("mask_1", 2, 3)), move2, move3), 1.0, True),
        "first cycle move only": (scene(move1), 0.0, False),
        "two cycle moves complete kit 3": (scene(move1, move2), 1 / 3, False),
        "the last two cycle moves complete kit 2": (scene(move2, move3), 1 / 3, False),
        "the first and last cycle moves complete kit 1": (scene(move1, move3), 1 / 3, False),
        "a swap of kit 1's extra mask and kit 2's extra torch completes only kit 1": (scene(move3, ("mask_2", at("mask_2", 1, 3))), 1 / 3, False),
        "the gap in kit 3 filled from kit 2, the nearest kit with a mask": (scene(move3, ("mask_3", at("mask_3", 2, 3))), 0.0, False),
        "the nearest-kit fill, then kit 3's extra dressing to kit 2, completes kit 3": (scene(("mask_3", at("mask_3", 2, 3)), move2), 1 / 3, False),
        "the nearest-kit fill repaired with a mask back on kit 2's mask place": (scene(move3, ("mask_3", at("mask_3", 2, 3)), ("mask_2", at("mask_2", 1, 2)), move2), 1.0, True),
        "the nearest-kit fill repaired with kit 2's new mask on another place": (scene(move3, ("mask_3", at("mask_3", 2, 3)), ("mask_2", at("mask_2", 1, 0)), move2), 2 / 3, False),
        "a regrouping one of each per mat that moves items which belonged completes only kit 1": (scene(*[(n, at(n, i % 3, i // 3)) for i, n in enumerate(regroup)]), 1 / 3, False),
        "done but kit 1's dressing moved to the vacated mask place": (scene(move1, move2, move3, ("dressing_1", at("dressing_1", 0, 1))), 2 / 3, False),
        "done but kit 1's remaining mask moved off both mask places": (scene(("mask_1", at("mask_1", 2, 3)), ("mask_2", at("mask_2", 0, 3)), ("torch_1", at("torch_1", 0, 0)), move2), 2 / 3, False),
        "a kept item 15 mm from its start place counts": (scene(move1, move2, move3, ("dressing_1", state("dressing_1", SPOT_XS[2], MAT_YS[0] + 0.015))), 1.0, True),
        "a kept item 16 mm from its start place spoils its kit": (scene(move1, move2, move3, ("dressing_1", state("dressing_1", SPOT_XS[2], MAT_YS[0] + 0.016))), 2 / 3, False),
        "a kept item turned and tipped where it stands counts": (scene(move1, move2, move3, ("dressing_1", at("dressing_1", 0, 2, yaw=0.6, tilt=math.pi / 2))), 1.0, True),
        "each surplus taken out to the table": (scene(("mask_2", state("mask_2", off, 0.42)), ("torch_2", state("torch_2", off, 0.30)), ("dressing_3", state("dressing_3", off, 0.18))), 0.0, False),
        "moved items anywhere along the mat count": (scene(("mask_2", state("mask_2", 0.110, 0.200)), move2, ("torch_1", state("torch_1", 0.190, 0.440))), 1.0, True),
        "a surplus left between two mats counts nowhere": (scene(move1, move2, ("torch_1", state("torch_1", 0.200, 0.360, bottom=TOP))), 2 / 3, False),
        "done with kit 1's torch lying on its mat counts": (scene(move1, move2, ("torch_1", at("torch_1", 0, 3, tilt=math.pi / 2))), 1.0, True),
        "a torch lying 8 degrees off the horizontal counts": (scene(move1, move2, ("torch_1", at("torch_1", 0, 3, tilt=math.radians(82)))), 1.0, True),
        "a torch leaning 30 degrees above the horizontal spoils kit 1": (scene(move1, move2, ("torch_1", at("torch_1", 0, 3, tilt=math.radians(60)))), 2 / 3, False),
        "done with kit 3's mask on its 50 x 35 side counts": (scene(move2, move3, ("mask_2", at("mask_2", 2, 3, tilt=math.pi / 2, about="y"))), 1.0, True),
        "a mask leaning 45 degrees on an edge spoils kit 3": (scene(move2, move3, ("mask_2", at("mask_2", 2, 3, tilt=math.pi / 4))), 2 / 3, False),
        "done but kit 3's mask stacked on its dressing": (scene(move1, move2, move3, ("mask_2", at("mask_2", 2, 1, bottom=MAT_TOP + DRESSING[2]))), 2 / 3, False),
        "done with kit 2's pack on its narrow 45 x 25 side counts": (scene(move1, move3, ("dressing_2", at("dressing_2", 1, 3, tilt=math.pi / 2))), 1.0, True),
        "a torch tilted 8 degrees counts": (scene(move1, move2, ("torch_1", at("torch_1", 0, 3, tilt=math.radians(8)))), 1.0, True),
        "a torch tilted 12 degrees does not": (scene(move1, move2, ("torch_1", at("torch_1", 0, 3, tilt=math.radians(12)))), 2 / 3, False),
        "an upside-down mask and a torch on its other end count": (scene(("mask_2", at("mask_2", 2, 3, tilt=math.pi)), move2, ("torch_1", at("torch_1", 0, 3, tilt=math.pi))), 1.0, True),
        "items turned about the vertical count": (scene(("mask_2", at("mask_2", 2, 3, yaw=1.0)), ("dressing_2", at("dressing_2", 1, 3, yaw=-0.7)), move3), 1.0, True),
        "a mask 5 mm above the mat (on a rim or a part) does not count": (scene(move2, move3, ("mask_2", at("mask_2", 2, 3, bottom=MAT_TOP + 0.005))), 2 / 3, False),
        "a mask 4 mm into the solver's contact counts": (scene(move2, move3, ("mask_2", at("mask_2", 2, 3, bottom=MAT_TOP - 0.004))), 1.0, True),
        "a mask held motionless 150 mm over kit 3 counts nowhere": (scene(move2, move3, ("mask_2", at("mask_2", 2, 3, bottom=MAT_TOP + 0.150))), 2 / 3, False),
        "a mask still moving over kit 3 does not count": (scene(move2, move3, ("mask_2", at("mask_2", 2, 3, vel=(0.0, 0.0, -0.05)))), 2 / 3, False),
        "a torch still rolling on kit 1 does not count": (scene(move1, move2, ("torch_1", at("torch_1", 0, 3, vel=(0.02, 0.0, 0.0)))), 2 / 3, False),
        "kit 1's extra mask still sliding on it spoils kit 1": (scene(move3, ("mask_2", state("mask_2", 0.200, 0.430, vel=(0.02, 0.0, 0.0)))), 0.0, False),
        "kit 1's extra mask still being lowered onto it spoils kit 1": (scene(move3, ("mask_2", at("mask_2", 0, 1, bottom=MAT_TOP + 0.005, vel=(0.0, 0.0, -0.011)))), 0.0, False),
        "a torch still sliding across complete kit 3 spoils it": (scene(move1, move2, ("torch_1", state("torch_1", 0.280, 0.150, vel=(0.02, 0.0, 0.0)))), 1 / 3, False),
        "a torch carried 120 mm over complete kit 3 spoils nothing": (scene(move1, move2, ("torch_1", state("torch_1", 0.280, 0.180, bottom=MAT_TOP + 0.120, vel=(0.05, 0.0, 0.0)))), 2 / 3, False),
        "centre 1 mm inside the mat edge counts": (scene(move1, move2, ("torch_1", state("torch_1", MAT_X + MAT[0] / 2 - 0.001, MAT_YS[0]))), 1.0, True),
        "centre 1 mm outside the mat edge does not": (scene(move1, move2, ("torch_1", state("torch_1", MAT_X + MAT[0] / 2 + 0.001, MAT_YS[0], bottom=TOP))), 2 / 3, False),
        "done but a torch knocked off the table": (scene(move1, move2, ("torch_1", state("torch_1", 0.95, 0.30, bottom=0.0, tilt=math.pi / 2))), 2 / 3, False),
        "every item tipped onto its side where it stood leaves the kits unbalanced": (scene(*[(n, at(n, k, s, tilt=math.pi / 2)) for n, (k, s) in START.items()]), 0.0, False),
        "the cycle done with every item on its side counts": (scene(*[(n, at(n, k, s, tilt=math.pi / 2)) for n, (k, s) in end.items()]), 1.0, True),
        "every item swept to one mat": (scene(*[(n, state(n, 0.100 + 0.026 * i, 0.420, bottom=MAT_TOP)) for i, n in enumerate(START)]), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not want_success or got == 1.0, name
    assert distance(done) == 0.0 and distance(start) > 0.0
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
