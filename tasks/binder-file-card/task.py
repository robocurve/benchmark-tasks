"""File a punched card in a ring binder. A standard 1 inch round-ring binder (Letter size) lies open flat on the table,
taped down, its spine running away from the arms and its three rings closed. On the right, beyond the binder's back
cover, a three-hole-punched Letter-size card waits in the front slot of a wire step file (a desktop incline
sorter), standing on its punched edge and leaning back on the slot's divider.

Task: file the card in the binder. The rings open only when the two halves of a ring are pulled apart past the snap of
the ring mechanism (two hinged steel plates under a sprung cover: an over-centre toggle that holds the rings either
shut or open). The card's three holes, 108 mm apart, must then go over the three open ring halves at once, and the
rings must be pressed shut again, tips meeting, with the card on all three.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from binderkit import (Body, BoxShape, CapShape, Hinge, Slide, as_numpy, joint, on_reset, on_step,  # noqa: E402
                       quat_x, quat_z, rotation)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "File the punched card in the binder: put it on all three rings and close the rings."

# -- colours ---------------------------------------------------------------------------------------------------------
VINYL = (0.10, 0.22, 0.55)
VINYL_IN = (0.13, 0.27, 0.62)
STEEL = (0.78, 0.79, 0.81)
RING = (0.86, 0.87, 0.89)
CARD = (0.96, 0.95, 0.90)
INK = (0.25, 0.25, 0.30)
MESH = (0.12, 0.12, 0.13)

_DEBUG_CONTACT = bool(os.environ.get("BINDER_DEBUG"))

# -- the binder ------------------------------------------------------------------------------------------------------
# A standard US 1 inch round-ring binder, Letter size (for example the Avery Economy 1" round-ring binder): covers
# 292 x 257 mm (11.5 x 10.1 in) of 3 mm vinyl-covered board, a 36 mm (1.4 in) spine, opened flat (550 mm across) with
# its spine along x. The ring mechanism is riveted to the back cover, its centre line 25 mm from the spine.
XC = 0.27                  # the binder's middle (and the middle ring) along x (world)
YM = -0.03                 # the ring mechanism's centre line (world y)
BOARD_T = 0.003
COVER_L, COVER_W, SPINE_W = 0.292, 0.257, 0.036
SPINE_GAP = 0.025          # from the mechanism's centre line to the back cover's spine edge
RING_X = (XC - 0.108, XC, XC + 0.108)   # the three rings, 4.25 in (108 mm) apart: the North American standard
WIRE_R = 0.002             # ring wire 4.0 mm across (estimate; measure with calipers)
RING_ID = 0.0254           # 1 in inside diameter (the binder's size)
RC = RING_ID / 2 + WIRE_R  # radius of the wire's centre line: 14.7 mm
BASE_H = 0.003             # the plates' hinge line stands 3 mm above the board when the rings are shut
ZH0 = TOP + BOARD_T + BASE_H            # the hinge line, rings shut (world z)
ZC = ZH0 + RC                           # the rings' centre, rings shut
HOUSING_W, HOUSING_H, HOUSING_L = 0.032, 0.007, 0.250   # the mechanism's spring-steel cover

# The mechanism: two steel plates side by side under the cover, hinged to each other along the centre line. The cover
# pivots the left plate at its outer edge and pushes the right plate's outer edge toward it with its spring (modelled
# as a preloaded spring of SPRING_FORCE). Because the plates are a little wider than the room the cover gives them,
# they can never lie flat: their shared hinge sits either below the outer edges (each ring half leans in: the rings
# are shut) or above them (each half leans out: the rings are open), and passing between the two compresses the
# spring further -- an over-centre toggle.
PLATE_W = 0.013            # each plate, from the hinge line to its outer edge
PLATE_T = 0.0008
SHUT_DEG = 11.0            # the plates' tilt from flat when the rings are shut (hinge line down)
OPEN_DEG = 22.0            # the left plate turns this far from shut to open (stopped by the cover): tips 22 mm apart
_A = math.radians(SHUT_DEG)
PIVOT_L = np.array([XC, YM + PLATE_W * math.cos(_A), ZH0 + PLATE_W * math.sin(_A)])   # the left plate's outer edge
EDGE_R = np.array([XC, YM - PLATE_W * math.cos(_A), ZH0 + PLATE_W * math.sin(_A)])    # the right plate's outer edge
H0 = np.array([XC, YM, ZH0])            # the hinge line (both plates' body origin), rings shut
# OPEN_FORCE is an estimate, TO BE REPLACED BY THE MEASURED VALUE: the force that pulls a shut ring's tips apart
# (README, "Measuring the snap force"). The cover's spring force is set from it (SPRING_FORCE, below: with this
# geometry the force at the tips is about 0.061 x the spring's push at its peak, when a ring starts to open, and
# again when it starts to close).
OPEN_FORCE = 12.0          # N, estimate
SPRING_K = 2000.0          # N/m: soft, so the cover's push stays within 1% of SPRING_FORCE over the plates' travel
PLATE_MASS = 0.034         # kg: a 250 x 13 x 0.8 mm steel plate (20 g) and three ring halves (4.6 g each)
N_ARC = 12                 # capsules per ring half


def _arc(x: float, side: int) -> list[np.ndarray]:
    """The wire's centre line of one ring half (side +1: the left half, at +y; -1: the right), rings shut, from its
    foot (inside the cover) to its tip (at the top), world frame."""
    th = np.linspace(-math.pi / 2, math.pi / 2, N_ARC + 1)
    return [np.array([x, YM + side * RC * math.cos(t), ZC + RC * math.sin(t)]) for t in th]


ARC_POINTS = {side: [[p - H0 for p in _arc(x, side)] for x in RING_X] for side in (1, -1)}   # in the plates' frame


def _plate(name: str, side: int) -> Body:
    edge = (PIVOT_L if side > 0 else EDGE_R) - H0
    mid = edge / 2
    tilt = math.atan2(edge[2], edge[1]) if side > 0 else math.atan2(-edge[2], -edge[1])
    shapes = [BoxShape(tuple(mid), (HOUSING_L - 0.004, PLATE_W, PLATE_T), STEEL, quat=quat_x(tilt), collide=False)]
    for pts in ARC_POINTS[side]:
        for a, b in zip(pts[:-1], pts[1:]):
            shapes.append(CapShape(tuple(a), tuple(b), WIRE_R, RING))
    return Body.build(name, pos=tuple(H0), mass=PLATE_MASS, shapes=shapes, color=RING, friction=(1.0, 0.9), friction_combine="max",
                      contact_offset=0.001, solver_iterations=64, ignore=("binder",), damping=(0.0, 0.0), report=_DEBUG_CONTACT,
                      com=(0.0, side * PLATE_W / 2, 0.006))


def _binder() -> Body:
    o = np.array([XC, YM, TOP])        # the body origin: on the table under the mechanism's centre line
    z = BOARD_T / 2
    back_y = (SPINE_GAP - COVER_W / 2)
    spine_y = SPINE_GAP + SPINE_W / 2
    front_y = SPINE_GAP + SPINE_W + COVER_W / 2
    shapes = [BoxShape((0.0, back_y, z), (COVER_L, COVER_W, BOARD_T), VINYL_IN),
              BoxShape((0.0, spine_y, z), (COVER_L, SPINE_W, BOARD_T), VINYL),
              BoxShape((0.0, front_y, z), (COVER_L, COVER_W, BOARD_T), VINYL_IN)]
    # The cover: a flat top 22 mm wide, 7 mm above the hinge line, and two sloping sides down to the board.
    top_z = ZH0 + HOUSING_H - TOP
    shapes.append(BoxShape((0.0, 0.0, top_z), (HOUSING_L, 0.022, 0.0008), STEEL))
    for s in (1, -1):
        y0, z0, y1, z1 = s * 0.011, top_z, s * HOUSING_W / 2, BOARD_T
        ang = math.atan2(z1 - z0, y1 - y0)
        ln = math.hypot(y1 - y0, z1 - z0)
        shapes.append(BoxShape((0.0, (y0 + y1) / 2, (z0 + z1) / 2), (HOUSING_L, ln, 0.0008), STEEL,
                               quat=quat_x(ang)))
    for x in (-HOUSING_L / 2 + 0.008, HOUSING_L / 2 - 0.008):      # the two rivets that hold it to the back cover
        shapes.append(BoxShape((x, 0.0, top_z + 0.0006), (0.006, 0.006, 0.0012), STEEL, collide=False))
    return Body.build("binder", pos=tuple(o), mass=0.4, kinematic=True, color=VINYL, shapes=shapes,
                      friction=(0.5, 0.45))


# -- the card --------------------------------------------------------------------------------------------------------
# A Letter-size (279 x 216 mm) sheet of 110 lb index card stock (199 g/m2, 12 g), three-hole punched: 9/32 in (7.1 mm)
# holes 108 mm apart, their centres 9.5 mm from the long edge. Modelled as a rigid 0.5 mm plate whose holes are real
# gaps in its collision shapes (octagons 7.1 mm across the flats). Card frame: x along the punched edge, y across the
# card from the punched edge (-108 mm) to the far edge, z its face normal.
CARD_L, CARD_W, CARD_T = 0.2794, 0.2159, 0.0005
CARD_MASS = 0.012
HOLE_R = 0.00355
HOLE_IN = 0.0095           # hole centre to the punched edge
HOLE_Y = -CARD_W / 2 + HOLE_IN
HOLE_X = (-0.108, 0.0, 0.108)
BAND = 2 * HOLE_IN         # the strip along the punched edge that holds the holes


def card_shapes() -> list:
    t, r = CARD_T, HOLE_R
    y0, y1 = -CARD_W / 2, -CARD_W / 2 + BAND
    s = [BoxShape((0.0, (y1 + CARD_W / 2) / 2, 0.0), (CARD_L, CARD_W / 2 - y1, t), CARD)]
    xs = [-CARD_L / 2] + [v for h in HOLE_X for v in (h - r, h + r)] + [CARD_L / 2]
    for a, b in zip(xs[0::2], xs[1::2]):                       # the strip between the holes
        s.append(BoxShape(((a + b) / 2, (y0 + y1) / 2, 0.0), (b - a, BAND, t), CARD))
    a = 2 * r * (math.sqrt(2) - 1)
    for h in HOLE_X:
        s.append(BoxShape((h, (y0 + HOLE_Y - r) / 2, 0.0), (2 * r, HOLE_Y - r - y0, t), CARD))
        s.append(BoxShape((h, (y1 + HOLE_Y + r) / 2, 0.0), (2 * r, y1 - HOLE_Y - r, t), CARD))
        for sx in (-1, 1):                                     # corner fillers: an octagonal hole
            for sy in (-1, 1):
                s.append(BoxShape((h + sx * r, HOLE_Y + sy * r, 0.0), (a, a, t), CARD, quat=quat_z(math.pi / 4)))
    s.append(BoxShape((0.0, 0.03, t / 2 + 0.00005), (0.16, 0.004, 0.0001), INK, collide=False))   # printed lines
    s.append(BoxShape((0.0, 0.05, t / 2 + 0.00005), (0.16, 0.004, 0.0001), INK, collide=False))
    return s


# The sorter: a wire step file (desktop incline sorter) for Letter files, for example the Fellowes Wire Step File
# 72614 (about 308 x 257 mm, 300 mm tall), standing on the right beyond the binder's back cover with its slots along x
# and its steps rising away from the binder (-y). Modelled as 2 mm plates: a base, a 10 mm front lip, the front slot's
# divider 160 mm tall, a second divider 230 mm tall and a 300 mm back. The card stands in the front slot on its punched
# edge, against the lip, leaning back 15 degrees on the first divider's top edge (its top 50 mm above it).
LEAN_DEG = 15.0
SORTER_FRONT = -0.270      # the sorter's front edge (world y), 8 mm clear of the binder's back cover
SORTER_DEPTH, SORTER_LEN = 0.257, 0.308
WIRE_T = 0.002             # the model's plates (the real sorter is steel mesh on wire frames)
BASE_T = 0.003
LIP_H = 0.010
DIV_H = (0.160, 0.230, 0.300)
_L = math.radians(LEAN_DEG)
CARD_UP = np.array([0.0, -math.sin(_L), math.cos(_L)])         # up the card, from its punched edge
CARD_BACK = np.cross([1.0, 0.0, 0.0], CARD_UP)                 # the card's z axis: out of its back, toward the divider
CARD_START_R = np.column_stack([[1.0, 0.0, 0.0], CARD_UP, CARD_BACK])
LIP_Y = SORTER_FRONT - 0.003                                   # the lip's centre line
CARD_BOTTOM = np.array([XC, LIP_Y - WIRE_T / 2 - 0.0012 - CARD_T / 2 * math.cos(_L), TOP + BASE_T + 0.0003])
CARD_START = CARD_BOTTOM + CARD_W / 2 * CARD_UP
# The first divider's front face meets the card's back where the card reaches DIV_H[0] above the table.
_S = (TOP + DIV_H[0] - CARD_BOTTOM[2]) / math.cos(_L)
DIV1_Y = float((CARD_BOTTOM + _S * CARD_UP + CARD_T / 2 * CARD_BACK)[1]) - 0.0003 - WIRE_T / 2


def _quat_of(r: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    return (float((r[2, 1] - r[1, 2]) / (4 * w)), float((r[0, 2] - r[2, 0]) / (4 * w)),
            float((r[1, 0] - r[0, 1]) / (4 * w)), float(w))


CARD_START_XYZW = _quat_of(CARD_START_R)


def _card() -> Body:
    return Body.build("card", pos=tuple(CARD_START), mass=CARD_MASS, shapes=card_shapes(), color=CARD,
                      friction=(0.45, 0.4), contact_offset=0.001, solver_iterations=32, damping=(0.05, 0.1),
                      start_xyzw=CARD_START_XYZW)


def _sorter() -> Body:
    o = np.array([XC, SORTER_FRONT, TOP])
    back_y = -SORTER_DEPTH
    ys = (DIV1_Y - SORTER_FRONT, (DIV1_Y - SORTER_FRONT + back_y) / 2, back_y + WIRE_T / 2)
    shapes = [BoxShape((0.0, -SORTER_DEPTH / 2, BASE_T / 2), (SORTER_LEN, SORTER_DEPTH, BASE_T), MESH),
              BoxShape((0.0, LIP_Y - SORTER_FRONT, (BASE_T + LIP_H) / 2), (SORTER_LEN, WIRE_T, BASE_T + LIP_H), MESH)]
    for y, h in zip(ys, DIV_H):
        shapes.append(BoxShape((0.0, y, h / 2), (SORTER_LEN, WIRE_T, h), MESH))
    for sx in (-1, 1):                           # the end frames, as low rails
        shapes.append(BoxShape((sx * (SORTER_LEN / 2 - WIRE_T / 2), -SORTER_DEPTH / 2, 0.02),
                               (WIRE_T, SORTER_DEPTH, 0.04), MESH))
    return Body.build("sorter", pos=tuple(o), mass=1.2, kinematic=True, color=MESH, shapes=shapes,
                      friction=(0.4, 0.35))


OBJECTS = [_binder(), _plate("plate_l", 1), _plate("plate_r", -1), _card(), _sorter()]

# -- scoring numbers ---------------------------------------------------------------------------------------------------
OPEN_GAP = 0.010           # a ring is open when its tips are at least 10 mm apart
SHUT_GAP = 0.001           # ... and shut when they are within 1 mm
LINK_R = 0.0045            # the wire crosses the card's plane within this of a hole's centre: it is through the hole
CLEAR = 0.02               # let go: every part of both grippers at least 2 cm from the card and the rings
STILL = 0.001              # still: nothing moved more than 1 mm over the last 1 s
HOLD_STEPS = 20            # 1 s at 20 Hz


# -- geometry (pure; used both in simulation and by the offline tests) ------------------------------------------------

def wire_points(plate_l, plate_r) -> dict:
    """The wire's centre line of every ring half, world frame: {(ring, side): (N_ARC+1, 3) array}, from foot to tip.
    ``plate_l``/``plate_r`` are (pos, rotation matrix) of the plates' body origins."""
    out = {}
    for side, (p, r) in ((1, plate_l), (-1, plate_r)):
        p, r = np.asarray(p, float), np.asarray(r, float)
        for i, pts in enumerate(ARC_POINTS[side]):
            out[(i, side)] = p + np.asarray(pts) @ r.T
    return out


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def plate_poses(turn_deg: float):
    """The plates' poses ((pos, rotation), (pos, rotation)) with the left plate turned ``turn_deg`` from shut about its
    pivot (negative: toward open; -OPEN_DEG is fully open), the right plate following through the hinge with its
    outer edge kept at the cover's height (the kinematics of the mechanism, for the demo's plan and the tests)."""
    b = math.radians(turn_deg)
    rl = _rx(b)
    h = PIVOT_L + rl @ (H0 - PIVOT_L)
    v = EDGE_R - H0
    g = -b
    for _ in range(40):
        f = (h + _rx(g) @ v)[2] - EDGE_R[2]
        df = ((h + _rx(g + 1e-7) @ v)[2] - EDGE_R[2] - f) / 1e-7
        g -= f / df
    return (h, rl), (h, _rx(g))


def _tip_force_ratio() -> float:
    """The force at the tips (pulling them apart) per newton of the cover's spring, as a ring starts to open: the
    spring's work over the tips' parting, from the mechanism's kinematics."""
    def state(b):
        pl, pr = plate_poses(b)
        edge = pr[0] + pr[1] @ (EDGE_R - H0)
        return -edge[1], tip_gaps(wire_points(pl, pr))[0]
    (u0, g0), (u1, g1) = state(0.0), state(-0.01)
    return (u1 - u0) / (g1 - g0)


def tip_gaps(wires: dict) -> list[float]:
    """Distance (m) between the two tips of each ring (the wire's centre-line ends)."""
    return [float(np.linalg.norm(wires[(i, 1)][-1] - wires[(i, -1)][-1])) for i in range(len(RING_X))]


SPRING_FORCE = OPEN_FORCE / _tip_force_ratio()    # N: about 131 N for 8 N at the tips
SPRING_PRELOAD = SPRING_FORCE / SPRING_K

# The joints of the mechanism (always on; the bodies they join do not collide with each other).
joint(Hinge("pivot_l", "binder", "plate_l", anchor0=tuple(PIVOT_L - np.array([XC, YM, TOP])),
            anchor1=tuple(PIVOT_L - H0), limits=(-OPEN_DEG, 0.0)))
joint(Hinge("centre", "plate_l", "plate_r", anchor0=(0.0, 0.0, 0.0), anchor1=(0.0, 0.0, 0.0)))
joint(Slide("cover_spring", "binder", "plate_r", anchor0=tuple(EDGE_R - np.array([XC, YM, TOP])),
            anchor1=tuple(EDGE_R - H0), locked=("transZ",),
            spring=("transY", SPRING_K, 20.0, SPRING_PRELOAD), span=0.004))

def holes_world(card_pos, card_r) -> np.ndarray:
    return np.asarray(card_pos, float) + np.array([[h, HOLE_Y, 0.0] for h in HOLE_X]) @ np.asarray(card_r, float).T


def rings_through_holes(card_pos, card_r, wires: dict) -> list[bool]:
    """For each hole: whether a ring's wire passes through the card's plane within LINK_R of that hole's centre."""
    card_pos, card_r = np.asarray(card_pos, float), np.asarray(card_r, float)
    out = []
    for k, h in enumerate(HOLE_X):
        centre = np.array([h, HOLE_Y, 0.0])
        hit = False
        for pts in wires.values():
            loc = (np.asarray(pts) - card_pos) @ card_r - centre      # in the card frame, about the hole
            for a, b in zip(loc[:-1], loc[1:]):
                if a[2] * b[2] > 0.0 or a[2] == b[2]:
                    continue                                       # this piece of wire does not cross the card
                s = a[2] / (a[2] - b[2])
                q = a + s * (b - a)
                if math.hypot(q[0], q[1]) <= LINK_R:
                    hit = True
                    break
            if hit:
                break
        out.append(hit)
    return out


def card_corners(card_pos, card_r) -> np.ndarray:
    return np.array([np.asarray(card_pos, float) + np.asarray(card_r, float) @ np.array([sx * CARD_L / 2,
                                                                                          sy * CARD_W / 2, 0.0])
                     for sx in (-1, 1) for sy in (-1, 1)])


def _box_distance(points, centre, r, half) -> np.ndarray:
    local = (np.asarray(points, float) - centre) @ r
    return np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)


def _wire_distance(points, wires: dict) -> np.ndarray:
    pts = np.asarray(points, float)
    best = np.full(len(pts), np.inf)
    for w in wires.values():
        w = np.asarray(w)
        for a, b in zip(w[:-1], w[1:]):
            d = b - a
            t = np.clip((pts - a) @ d / (d @ d), 0.0, 1.0)
            best = np.minimum(best, np.linalg.norm(pts - (a + t[:, None] * d), axis=1) - WIRE_R)
    return best


def gripper_clearance(points, card_pos, card_r, wires: dict) -> float:
    """The least distance (m) from the grippers' points to the card and to the rings' wire."""
    if points is None or len(points) == 0:
        return float("inf")
    pts = np.asarray(points, float)
    d_card = _box_distance(pts, np.asarray(card_pos, float), np.asarray(card_r, float),
                           np.array([CARD_L / 2, CARD_W / 2, CARD_T / 2]))
    return float(min(d_card.min(), _wire_distance(pts, wires).min()))


# -- the score, step by step ---------------------------------------------------------------------------------------------

_EP: dict = {}
_DEBUG = bool(os.environ.get("BINDER_DEBUG"))


@on_reset
def _reset_state() -> None:
    _EP.clear()
    _EP.update(opened=False, on3=False, filed=False, held=0, recent=[], score=0.0, steps=0, info={})


_reset_state()


def observe(plate_l, plate_r, card, gripper_points) -> None:
    """One control step: work out the score from the plates' and card's poses ((pos, rotation matrix) each) and the
    grippers' points (world, m)."""
    wires = wire_points(plate_l, plate_r)
    gaps = tip_gaps(wires)
    card_pos, card_r = np.asarray(card[0], float), np.asarray(card[1], float)
    through = rings_through_holes(card_pos, card_r, wires)
    opened = all(g >= OPEN_GAP for g in gaps)
    shut = all(g <= SHUT_GAP for g in gaps)
    on3 = all(through)
    _EP["opened"] |= opened
    _EP["on3"] |= on3
    _EP["filed"] |= on3 and shut
    snap = np.vstack([card_corners(card_pos, card_r)] + [w[-1] for w in wires.values()])
    recent = _EP["recent"]
    recent.append(snap)
    del recent[:-(HOLD_STEPS + 1)]
    still = len(recent) == HOLD_STEPS + 1 and all(float(np.abs(r - snap).max()) <= STILL for r in recent)
    clear = gripper_clearance(gripper_points, card_pos, card_r, wires) >= CLEAR
    done = bool(on3 and shut and clear and still)
    _EP["held"] = _EP["held"] + 1 if done else 0
    if _EP["held"] >= HOLD_STEPS:
        score = 1.0
    elif _EP["filed"]:
        score = 0.75
    elif _EP["on3"]:
        score = 0.5
    elif _EP["opened"]:
        score = 0.25
    else:
        score = 0.0
    _EP.update(score=score, steps=_EP["steps"] + 1,
               info={"gaps": gaps, "through": through, "shut": shut, "clear": clear, "still": still})


def success(objects) -> bool:
    """True only on steps where score() is 1.0: the card has been on all three rings, the rings shut (tips within
    1 mm), both grippers 2 cm clear and everything still, for 1 s. The runtime then ends the episode after its own 1 s
    of this."""
    return _EP["score"] >= 1.0


def score(objects) -> float:
    """1.0 once the card has been on all three shut rings, let go (grippers 2 cm clear of the card and rings) and
    still, for 1 s (20 steps in a row); 0.75 if at some point the card was on all three rings with the rings shut;
    0.5 if at some point the card was on all three rings; 0.25 if at some point the rings were open (every ring's tips
    10 mm or more apart); else 0."""
    return float(_EP["score"])


# -- in the simulator: read the state and score it --------------------------------------------------------------------

_HULLS: dict = {}


def _gripper_points(base, origin) -> np.ndarray:
    """Points on both grippers (the vertices of their collision hulls, world frame)."""
    if not _HULLS:
        from yam_arena.analysis import link_hulls

        hulls = link_hulls()
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in hulls[link]])
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        for j, n in enumerate(robot.body_names):
            link = n.rsplit("/", 1)[-1]
            if link in _HULLS:
                out.append((_HULLS[link] @ rotation(poses[j][3:7]).T) + poses[j][:3] - origin)
    return np.vstack(out) if out else np.zeros((0, 3))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    pl, pr, card = pose("plate_l"), pose("plate_r"), pose("card")
    observe(pl, pr, card, _gripper_points(base, origin))
    if _DEBUG and _EP["steps"] % 10 == 0:
        inf = _EP["info"]
        print(f"[binder] step {_EP['steps']} score {_EP['score']} held {_EP['held']} "
              f"gaps {[round(1000 * g, 2) for g in inf['gaps']]} through {inf['through']} shut {inf['shut']} "
              f"clear {inf['clear']} still {inf['still']} card {np.round(1000 * card[0], 1).tolist()} "
              f"hinge {np.round(1000 * (pl[0] - H0), 2).tolist()}", file=sys.stderr, flush=True)
    if _DEBUG:
        from binderkit import net_contact_force
        fl, fr = net_contact_force("plate_l"), net_contact_force("plate_r")
        if max(np.abs(fl).max(), np.abs(fr).max()) > 0.05:
            print(f"[binder] step {_EP['steps']} contact on plates (N): L {np.round(fl, 2).tolist()} "
                  f"R {np.round(fr, 2).tolist()}", file=sys.stderr, flush=True)
