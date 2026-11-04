"""On the station table, in the left arm's lane, a 1 g bench analogue of sample-return curation: one sample
container has been flagged as possibly carrying biology, and curators treat every container touching it in the
layout as found, directly or through another container, as contaminated. Six upright sample containers, 30 mm
across, stand in one row across a dark bench mat: one red and five grey, in two sizes, 70 mm and 30 mm tall. The red
container and two grey ones stand pressed together in a line; one grey container stands 29 mm clear of the red
one, and a second touching grey pair stands 45 mm clear of the far end of the line. Two fixed racks wait beyond the
mat, each a block with three square pockets 40 mm wide and 12 mm deep: a yellow isolation box and a white curation
rack. The red container and the containers connected to it by contact go into the isolation box, the others into
the white rack. A contaminated container in, on or against the white rack scores 0; otherwise each container
standing in the right kind of pocket scores 1/6.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/contact-chain-quarantine/contact_chain_quarantine.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "The red sample may be biologically contaminated. Put it in the yellow isolation box with every sample that touches it now or touches one that does, and put the other samples in the white rack." \\
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
R = 0.015  # every sample is 30 mm across
TALL, SHORT = 0.070, 0.030
MASS = {TALL: 0.060, SHORT: 0.025}
ROW_X = 0.180
STARTS = {  # one row along y on the bench mat, inboard to outboard: (y, height)
    "sample_grey_1": (0.2260, SHORT),  # 29 mm clear of the red sample, still nearer to it than grey 3 is
    "sample_red": (0.2850, TALL),
    "sample_grey_2": (0.3155, SHORT),  # touches the red sample
    "sample_grey_3": (0.3460, TALL),  # touches grey 2
    "sample_grey_4": (0.4210, TALL),  # 45 mm clear of grey 3
    "sample_grey_5": (0.4515, SHORT),  # touches grey 4
}
HEIGHT = {name: h for name, (_, h) in STARTS.items()}
RED_NAME = "sample_red"
TOUCH = 2 * R + 0.001  # centre spacing that counts as touching; the touching samples spawn 0.5 mm apart
MAT = (0.080, 0.270, 0.003)  # bench mat, x by y by height
MAT_Y = 0.340

POCKET = 0.040  # square, 5 mm of clearance per side around a 30 mm sample
WALL = 0.006
WALL_H = 0.012
PITCH = 0.065
RACK_X = 0.280
ISO_YS = (0.355, 0.420, 0.485)  # yellow isolation box, outboard
CUR_YS = (0.140, 0.205, 0.270)  # white curation rack, inboard

GREY = (0.62, 0.62, 0.64)
RED = (0.85, 0.15, 0.15)
SLATE = (0.22, 0.24, 0.28)
YELLOW = (0.95, 0.80, 0.10)
WHITE = (0.92, 0.92, 0.90)


def _rack(prefix, ys, color):
    """A fixed block with three square pockets open to the table: two long walls along y, two end walls and two
    25 mm dividers, all 12 mm tall."""
    z = TOP + WALL_H / 2
    off = POCKET / 2 + WALL / 2
    span = (ys[-1] - ys[0]) + POCKET + 2 * WALL
    mid = (ys[0] + ys[-1]) / 2
    parts = [
        Box(f"{prefix}_wall_near", pos=(RACK_X - off, mid, z), size=(WALL, span, WALL_H), color=color),
        Box(f"{prefix}_wall_far", pos=(RACK_X + off, mid, z), size=(WALL, span, WALL_H), color=color),
        Box(f"{prefix}_end_in", pos=(RACK_X, ys[0] - off, z), size=(POCKET, WALL, WALL_H), color=color),
        Box(f"{prefix}_end_out", pos=(RACK_X, ys[-1] + off, z), size=(POCKET, WALL, WALL_H), color=color),
    ]
    for k in range(len(ys) - 1):
        parts.append(Box(f"{prefix}_divider_{k}", pos=(RACK_X, (ys[k] + ys[k + 1]) / 2, z), size=(POCKET, PITCH - POCKET, WALL_H), color=color))
    return parts


def _outline(ys):
    """The rack's outer outline seen from above: x0, x1, y0, y1."""
    half = POCKET / 2 + WALL
    return (RACK_X - half, RACK_X + half, ys[0] - half, ys[-1] + half)


MAT_BOX = Box("bench_mat", pos=(ROW_X, MAT_Y, TOP + MAT[2] / 2), size=MAT, color=SLATE)
SAMPLES = [
    Cylinder(name, pos=(ROW_X, y, TOP + MAT[2] + h / 2), radius=R, height=h, mass=MASS[h], color=RED if name == RED_NAME else GREY)
    for name, (y, h) in STARTS.items()
]
ISO_RACK = _rack("isolation_box", ISO_YS, YELLOW)
CUR_RACK = _rack("curation_rack", CUR_YS, WHITE)
OBJECTS = [*SAMPLES, MAT_BOX, *ISO_RACK, *CUR_RACK]


def _contact_chain(ys, seed):
    """Every sample connected to ``seed`` through a chain of touching samples in the start layout."""
    chain, frontier = {seed}, [seed]
    while frontier:
        a = frontier.pop()
        for b, yb in ys.items():
            if b not in chain and abs(yb - ys[a]) <= TOUCH:
                chain.add(b)
                frontier.append(b)
    return chain


CONTAMINATED = frozenset(_contact_chain({n: y for n, (y, _) in STARTS.items()}, RED_NAME))  # red, grey 2, grey 3
CLEAN = frozenset(STARTS) - CONTAMINATED
POCKETS = [("iso", (RACK_X, y)) for y in ISO_YS] + [("cur", (RACK_X, y)) for y in CUR_YS]
CUR_OUTLINE = _outline(CUR_YS)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)
FLOOR_TOL = 0.004  # lowest point to the pocket floor (the table)
NEAR_RACK = 0.001  # any part of a sample's footprint this close to the white rack's outline is against it
GATE_HEIGHT = 0.080  # a moving sample lower than this over or against the white rack counts; one at rest at any height
FOOT_N = 64  # chords per end circle of a sample's footprint
EPS = 1e-4


def _rotation(q):
    """Rotation matrix of a quaternion given as x, y, z, w."""
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _axis_z(state) -> float:
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)  # world z of the sample's axis


def _bottom(state, h) -> float:
    """The sample's lowest point from its centre and axis, standing, tilted or lying. The axis-aligned bounds of a
    tilted cylinder come from its box corners and read too low, so they are not used."""
    uz = _axis_z(state)
    return float(state.pos[2]) - (h / 2 * abs(uz) + R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _pocket(state, h):
    """The index into POCKETS of the pocket this sample stands in, or None. Standing in a pocket: at rest, axis
    within 10 degrees of vertical, lowest point within 4 mm of the table, centre inside the 40 x 40 mm opening;
    the walls then hold the whole footprint inside."""
    if not state.resting() or abs(_axis_z(state)) < math.cos(TILT_TOL) or abs(_bottom(state, h) - TOP) > FLOOR_TOL + EPS:
        return None
    x, y = float(state.pos[0]), float(state.pos[1])
    for k, (_, (px, py)) in enumerate(POCKETS):
        if abs(x - px) < POCKET / 2 and abs(y - py) < POCKET / 2:
            return k
    return None


def _segments_to_box(a, b, box) -> float:
    """Smallest distance in the plane from the segments a[i]-b[i] to the box (x0, x1, y0, y1), 0 where one touches
    or crosses it. Two convex shapes that do not meet are closest at a corner of one of them, so the segments' ends
    and the box's corners are the only candidates."""
    x0, x1, y0, y1 = box

    def to_box(p):
        return np.hypot(np.maximum(np.maximum(x0 - p[:, 0], p[:, 0] - x1), 0.0), np.maximum(np.maximum(y0 - p[:, 1], p[:, 1] - y1), 0.0))

    d = np.minimum(to_box(a), to_box(b))
    ab = b - a
    n2 = np.maximum((ab * ab).sum(1), 1e-18)
    side = []
    for c in ((x0, y0), (x0, y1), (x1, y0), (x1, y1)):
        ac = np.asarray(c) - a
        t = np.clip((ac * ab).sum(1) / n2, 0.0, 1.0)
        d = np.minimum(d, np.hypot(ac[:, 0] - t * ab[:, 0], ac[:, 1] - t * ab[:, 1]))
        side.append(ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0])
    side = np.array(side)
    # A segment crosses the box when their bounding boxes overlap and the box's corners are not all on one side of it.
    crosses = ((np.minimum(a[:, 0], b[:, 0]) <= x1) & (np.maximum(a[:, 0], b[:, 0]) >= x0)
               & (np.minimum(a[:, 1], b[:, 1]) <= y1) & (np.maximum(a[:, 1], b[:, 1]) >= y0)
               & (side.min(0) <= 0.0) & (side.max(0) >= 0.0))
    return float(np.where(crosses, 0.0, d).min())


def _footprint_gap(state, h) -> float:
    """Distance seen from above from the sample's footprint to the white rack's outline, 0 if they overlap. The
    footprint is the cylinder's shadow from its centre and quaternion, standing, tilted or lying: its two end circles
    as 64 chords each and the 64 side lines joining them. Chords sit at most 0.02 mm inside the true circle."""
    t = np.linspace(0.0, 2 * math.pi, FOOT_N, endpoint=False)
    rot = _rotation(state.quat_xyzw)
    centre = np.asarray(state.pos, float)[:2]
    lo, hi = [(np.stack([R * np.cos(t), R * np.sin(t), np.full_like(t, s * h / 2)], 1) @ rot.T)[:, :2] + centre for s in (-1, 1)]
    a = np.concatenate([lo, hi, lo])
    b = np.concatenate([np.roll(lo, -1, 0), np.roll(hi, -1, 0), hi])
    return _segments_to_box(a, b, CUR_OUTLINE)


def _against_curation_rack(state, h) -> bool:
    """In, on or against the white rack: some part of the sample's footprint seen from above inside the rack's
    outline or within 1 mm of it, while the sample is at rest (at any height) or its lowest point is under 80 mm.
    Standing in a pocket, lying on or across the walls, tipped onto a wall, stacked on samples in a pocket, or
    standing or lying against the outside all count; one moving high above the rack, as when carried, does not."""
    return _footprint_gap(state, h) <= NEAR_RACK + EPS and (state.resting() or _bottom(state, h) < TOP + GATE_HEIGHT)


def _tally(objects):
    """(gate, correct): gate is True when a contaminated sample is in, on or against the white rack; correct counts
    contaminated samples standing in isolation pockets and clean samples standing in curation pockets, one per
    pocket."""
    gate = any(_against_curation_rack(objects[n], HEIGHT[n]) for n in CONTAMINATED)
    used, correct = set(), 0
    for s in SAMPLES:
        k = _pocket(objects[s.name], HEIGHT[s.name])
        if k is None or k in used:
            continue
        used.add(k)
        kind = POCKETS[k][0]
        if (s.name in CONTAMINATED and kind == "iso") or (s.name in CLEAN and kind == "cur"):
            correct += 1
    return gate, correct


def success(objects) -> bool:
    """All six samples standing in the right kind of pocket, at rest, nothing contaminated at the white rack;
    holds for 1 s."""
    gate, correct = _tally(objects)
    return not gate and correct == len(SAMPLES)


def distance(objects) -> float:
    """Metres still to go: each sample's centre to the nearest pocket of its kind."""
    total = 0.0
    for s in SAMPLES:
        st = objects[s.name]
        kind = "iso" if s.name in CONTAMINATED else "cur"
        total += min(math.hypot(float(st.pos[0]) - px, float(st.pos[1]) - py) for pk, (px, py) in POCKETS if pk == kind)
    return total


def score(objects) -> float:
    """0 if a contaminated sample is in, on or against the white rack. Otherwise 1/6 per sample standing in the
    right kind of pocket: contaminated in the isolation box, clean in the white rack. A clean sample in the isolation
    box earns nothing and takes a pocket the chain needs; a sample left on the mat, lying, on a wall, on another
    sample or held earns nothing."""
    gate, correct = _tally(objects)
    return 0.0 if gate else correct / len(SAMPLES)


TRAVEL = TOP + 0.150
# TCP height above a sample's bottom at the grip. A tall sample is gripped 15 mm under its top, so the pad tips pass
# 12 mm over a short neighbour; a short one with the pad tips 14 mm up, 16 mm of it between the pads.
GRIP = {TALL: 0.055, SHORT: 0.027}
DROP = 0.005  # sample bottom above the pocket floor at release; short: pad tips 7 mm over the 12 mm walls
ENTRY = 0.017  # the pause above a pocket: sample bottom 10 mm above the walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


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


# Each move: sample, pocket y, yaw at the pick, yaw at the place. Yaw +-90 degrees closes the fingers along x, so a
# neighbour along y sits beside the pads, not in front of them; yaw 0 closes them along y, used only for grey 5,
# which stands alone by then. A tall sample is always taken while its touching neighbours are short, and a short one
# only once it stands alone, because the finger bodies are wider than a sample. Joint 6 runs at joint 1 minus the yaw
# and must stay inside +-120 degrees, so +90 fails at the inboard pockets and -90 at the outboard ones.
Q = math.pi / 2
ORDER = (  # clean samples first, as a curator would, then the chain
    ("sample_grey_4", CUR_YS[2], Q, Q),  # tall, beside short grey 5; joint 6 at -57 and -97 degrees
    ("sample_grey_5", CUR_YS[1], 0.0, -Q),  # short, alone; grey 4 in the next pocket; joint 6 at +39 and +70
    ("sample_grey_1", CUR_YS[0], -Q, -Q),  # short, 29 mm from the red sample; grey 5 in the next pocket; +66, +60
    ("sample_red", ISO_YS[0], -Q, -Q),  # tall, beside short grey 2; joint 6 at +84 and +100
    ("sample_grey_3", ISO_YS[1], Q, Q),  # tall, beside short grey 2; the red sample in the next pocket; -77, -68
    ("sample_grey_2", ISO_YS[2], Q, Q),  # short, alone; grey 3 in the next pocket; joint 6 at -87 and -57
)
TURN_AFTER = "sample_red"  # the wrist turns from -90 to +90 degrees in two steps, never 180 at once


def demo():
    """Left arm only. The three clean samples into the white rack, then the red sample, grey 3 and grey 2 into the
    isolation box. Each is a top-down pick, a carry at 15 cm, a pause 10 mm above the walls and a drop of 5 mm onto
    the pocket floor; after the red sample the wrist turns to yaw 0 above its pocket. 98.2 s of plan, then the 3 s
    hold: 2,024 steps at 20 Hz."""
    left = []
    for name, qy, yaw, place_yaw in ORDER:
        y, h = STARTS[name]
        pick = (ROW_X, y, TOP + MAT[2] + GRIP[h])
        place = (RACK_X, qy, TOP + DROP + GRIP[h])
        left += _pick_place(pick, place, width=2 * R, entry=ENTRY, travel_z=TRAVEL, yaw=yaw, place_yaw=place_yaw)
        if name == TURN_AFTER:
            left.append(Waypoint(1.0, (RACK_X, qy, TRAVEL), FINGER_OPEN, "turn the wrist", 0.0))
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def sample(name, x, y, bottom=TOP, tilt=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0)):
        """Sample ``name`` tilted ``tilt`` about its own x axis, then turned ``yaw``; low/high from the rotated box
        corners, as the simulator reports them."""
        h = HEIGHT[name]
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])
        uz = math.cos(tilt)
        pos = np.array([x, y, bottom + h / 2 * abs(uz) + R * math.sqrt(max(0.0, 1.0 - uz * uz))])
        half = np.abs(_rotation(q)) @ np.array([R, R, h / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    G1, RD, G2, G3, G4, G5 = STARTS  # row order, inboard to outboard

    def iso(name, k, **kw):
        return {name: sample(name, RACK_X, ISO_YS[k], **kw)}

    def cur(name, k, **kw):
        return {name: sample(name, RACK_X, CUR_YS[k], **kw)}

    start = {n: sample(n, ROW_X, y, bottom=TOP + MAT[2]) for n, (y, _) in STARTS.items()}

    def scene(*parts):
        out = dict(start)
        for p in parts:
            out.update(p)
        return out

    assert CONTAMINATED == {RD, G2, G3} and CLEAN == {G1, G4, G5}, CONTAMINATED
    names = list(STARTS)
    gaps = [STARTS[b][0] - STARTS[a][0] - 2 * R for a, b in zip(names, names[1:])]
    assert [round(g * 1000, 1) for g in gaps] == [29.0, 0.5, 0.5, 45.0, 0.5], gaps
    red_y = STARTS[RD][0]
    assert abs(STARTS[G1][0] - red_y) < abs(STARTS[G3][0] - red_y)  # clean grey 1 is nearer the red sample than grey 3
    assert {HEIGHT[n] for n in CONTAMINATED} == {HEIGHT[n] for n in CLEAN} == {TALL, SHORT}  # size does not tell
    x0, x1, y0, y1 = CUR_OUTLINE
    iso_y0 = ISO_YS[0] - POCKET / 2 - WALL
    tip = math.asin((WALL_H - MAT[2]) / TALL)  # a tall sample tipped from the mat until its top rim rests on a 12 mm wall
    done = [cur(G1, 0), cur(G5, 1), cur(G4, 2), iso(RD, 0), iso(G3, 1), iso(G2, 2)]
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(*done), 1.0, True),
        "done in another pocket order": (scene(cur(G4, 0), cur(G1, 1), cur(G5, 2), iso(G2, 0), iso(RD, 1), iso(G3, 2)), 1.0, True),
        "the three nearest the red sample isolated, the rest curated": (scene(iso(G1, 0), iso(RD, 1), iso(G2, 2), cur(G3, 0), cur(G4, 1), cur(G5, 2)), 0.0, False),
        "only the red sample and its direct neighbour isolated": (scene(iso(RD, 0), iso(G2, 1), cur(G3, 0), cur(G1, 1), cur(G4, 2)), 0.0, False),
        "only the red sample isolated, three curated": (scene(iso(RD, 0), cur(G1, 0), cur(G2, 1), cur(G4, 2)), 0.0, False),
        "the row split in half, inboard half curated": (scene(cur(G1, 0), cur(RD, 1), cur(G2, 2), iso(G3, 0), iso(G4, 1), iso(G5, 2)), 0.0, False),
        "the inboard three into the white rack, nothing isolated": (scene(cur(G1, 0), cur(RD, 1), cur(G2, 2)), 0.0, False),
        "the tall samples isolated": (scene(iso(RD, 0), iso(G3, 1), iso(G4, 2), cur(G1, 0), cur(G2, 1), cur(G5, 2)), 0.0, False),
        "the short samples isolated": (scene(iso(G1, 0), iso(G2, 1), iso(G5, 2), cur(RD, 0), cur(G3, 1), cur(G4, 2)), 0.0, False),
        "the chain isolated, the clean samples left on the mat": (scene(iso(RD, 0), iso(G2, 1), iso(G3, 2)), 0.5, False),
        "the clean samples curated, the chain left on the mat": (scene(cur(G1, 0), cur(G4, 1), cur(G5, 2)), 0.5, False),
        "touching treated as contaminated: chain isolated, the pair left out": (scene(iso(RD, 0), iso(G2, 1), iso(G3, 2), cur(G1, 0)), 4 / 6, False),
        "the touching pair isolated with the red sample": (scene(iso(G4, 0), iso(G5, 1), iso(RD, 2), cur(G1, 0)), 2 / 6, False),
        "a clean sample isolated, one of the chain left out": (scene(iso(G1, 0), iso(RD, 1), iso(G2, 2), cur(G4, 0), cur(G5, 1)), 4 / 6, False),
        "done but a contaminated sample on top of a tall clean one in the white rack": (scene(*done, cur(G2, 2, bottom=TOP + TALL)), 0.0, False),
        "done but a contaminated sample lying across the white rack": (scene(*done, {G3: sample(G3, RACK_X, 0.240, bottom=TOP + WALL_H, tilt=Q)}), 0.0, False),
        "done but a contaminated sample standing against the white rack": (scene(*done, {G3: sample(G3, x0 - R, 0.205)}), 0.0, False),
        "done but a contaminated sample standing 3 mm clear of the white rack": (scene(*done, {G3: sample(G3, x0 - R - 0.003, 0.205)}), 5 / 6, False),
        "done but a contaminated sample lying beyond the white rack's inboard end": (scene(*done, {RD: sample(RD, RACK_X, y0 - R, tilt=Q, yaw=Q)}), 0.0, False),
        "done but a contaminated sample lying with its flat end against the white rack's near wall": (scene(*done, {RD: sample(RD, x0 - TALL / 2, CUR_YS[0], tilt=Q, yaw=Q)}), 0.0, False),
        "done but a contaminated sample lying end on 3 mm clear of the white rack's near wall": (scene(*done, {RD: sample(RD, x0 - TALL / 2 - 0.003, CUR_YS[0], tilt=Q, yaw=Q)}), 5 / 6, False),
        "done but a contaminated sample lying with its flat end against the white rack's inboard end": (scene(*done, {G3: sample(G3, RACK_X, y0 - TALL / 2, tilt=Q)}), 0.0, False),
        "done but the red sample tipped from its spot, top rim 3 mm onto the white rack's near wall": (scene(*done, {RD: sample(RD, x0 + 0.003 - TALL / 2 * math.cos(tip) - R * math.sin(tip), STARTS[RD][0], bottom=TOP + MAT[2], tilt=Q - tip, yaw=Q)}), 0.0, False),
        "done but a contaminated sample lying across the gap between the racks on both end walls": (scene(*done, {G3: sample(G3, RACK_X, (y1 + iso_y0) / 2, bottom=TOP + WALL_H, tilt=Q)}), 0.0, False),
        "done but a contaminated sample on top of two samples stacked in a white pocket": (scene(*done, cur(G1, 2, bottom=TOP + TALL), cur(G2, 2, bottom=TOP + TALL + SHORT)), 0.0, False),
        "done but a contaminated sample held still high over the white rack": (scene(*done, {G3: sample(G3, RACK_X, 0.205, bottom=TOP + 0.095)}), 0.0, False),
        "done but a contaminated sample carried high over the white rack": (scene(*done, {G3: sample(G3, RACK_X, 0.205, bottom=TOP + 0.095, vel=(0.05, 0, 0))}), 5 / 6, False),
        "done but a contaminated sample on the table between the two racks": (scene(*done, {RD: sample(RD, RACK_X, y1 + R + 0.0015)}), 5 / 6, False),
        "done but a clean sample still moving in its pocket": (scene(*done, cur(G4, 2, vel=(0, 0, 0.02))), 5 / 6, False),
        "done but a clean sample held still 5 mm above its pocket floor": (scene(*done, cur(G4, 2, bottom=TOP + 0.005)), 5 / 6, False),
        "a sample 3.5 mm above the floor counts": (scene(*done, cur(G4, 2, bottom=TOP + 0.0035)), 1.0, True),
        "done but a sample tilted 12 degrees in its pocket": (scene(*done, cur(G5, 1, tilt=math.radians(12))), 5 / 6, False),
        "a sample tilted 8 degrees in its pocket counts": (scene(*done, cur(G4, 2, tilt=math.radians(8))), 1.0, True),
        "done but a contaminated sample on an isolation box divider": (scene(*done, {RD: sample(RD, RACK_X, (ISO_YS[0] + ISO_YS[1]) / 2, bottom=TOP + WALL_H)}), 5 / 6, False),
        "done but a contaminated sample lying on the isolation box": (scene(*done, iso(G3, 1, bottom=TOP + WALL_H, tilt=Q)), 5 / 6, False),
        "every sample knocked over where it stood": (scene({n: sample(n, ROW_X, y, bottom=TOP + MAT[2], tilt=Q, yaw=0.3) for n, (y, _) in STARTS.items()}), 0.0, False),
        "a clean sample 19 mm off its pocket centre still counts": (scene(*done, {G1: sample(G1, RACK_X + 0.019, CUR_YS[0])}), 1.0, True),
        "two samples counted in one pocket count once": (scene(*done, cur(G5, 2)), 5 / 6, False),
        "a short sample on the mat 3 mm up is not in a pocket": (scene(*done, {G5: sample(G5, ROW_X, 0.44, bottom=TOP + MAT[2])}), 5 / 6, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not got_success or got == 1.0, name
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
