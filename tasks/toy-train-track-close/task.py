"""On a play table: a loop of wooden toy train track, eight large curves of which one is missing, and three loose
pieces beside it: a large curve, a straight and a short curve. The robot must close the loop with the piece that fits,
each of its two connectors joined to its neighbour's, and let go of it.

The physical catch: wooden track joins by a peg (a round knob on a neck) dropped into the next piece's keyhole socket
from above. A loop missing one piece leaves a gap with a peg sticking into it on one side and a socket waiting on the
other, so the new piece has to come down with its socket over that peg and its own peg over the far socket at the same
time. Only a large curve spans the gap. It lies turned a quarter turn from the way it has to go in, and a curve laid in
the other way round, or set down off its line, rests on the pegs instead of joining them.

Sizes (BRIO-compatible wooden track, from the woodenrailway.info track guide): 40 mm wide and 12 mm thick; peg 11.5 mm
across on a 7 mm neck; socket 15-17 mm across behind a 5 mm throat. Large curve (BRIO E, sold as BRIO 33342 Large Curved
Tracks, 4 pieces): an eighth of a circle, inner edge 182 mm and outer edge 222 mm from its centre. Short curve (E1):
inner edge 90 mm. Straight (A): 144 mm. Each piece has a peg at one end and a socket at the other. Masses are from beech
(about 0.72 g/cm3): large curve 50 g, straight 46 g, short curve 28 g. The neck's and throat's widths (6 and 9 mm) and the
peg's height are estimates; measure the bought pieces.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close the gap in the wooden train track with the piece that fits."

BEECH = (0.80, 0.63, 0.42)

# -- one piece of track ------------------------------------------------------------------------------------------------
# Part frame: origin on the piece's centre line halfway along it, on its underside; x along the track from its socket
# end (A) to its peg end (B); +y to the left (a curve bends toward +y, its circle's centre at (0, R)).
W, H = 0.040, 0.012
NECK_W, NECK_L = 0.006, 0.007
KNOB_R = 0.00575
PEG_Z = (0.0005, 0.0115)                 # the peg's underside and top (the track is 0-12 mm)
HOLE, THROAT_W, THROAT_L = 0.016, 0.009, 0.005
FEM_L = THROAT_L + HOLE                  # the socket end's length, 21 mm
HOLE_C = THROAT_L + HOLE / 2             # the socket's centre, 13 mm in from the end face
KNOB_C = NECK_L + KNOB_R                 # the peg's centre, 12.75 mm out from the end face
END_GAP = 0.0006                         # the peg end's face stops this short of the joint line (contact offset 0.5 mm)

R_LARGE, R_SHORT = 0.202, 0.110          # centre-line radii (edges 182/222 and 90/130 mm)
ARC = math.pi / 4                        # each curve is an eighth of a circle
L_STRAIGHT = 0.144


def _yawq(a: float):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def centre_line(kind: str, s: float):
    """Point and unit tangent on the centre line at fraction ``s`` (0 = end A, 1 = end B), part frame (z = 0)."""
    if kind == "straight":
        return np.array([(s - 0.5) * L_STRAIGHT, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    r = R_LARGE if kind == "large" else R_SHORT
    phi = (s - 0.5) * ARC
    return np.array([r * math.sin(phi), r * (1 - math.cos(phi)), 0.0]), np.array([math.cos(phi), math.sin(phi), 0.0])


def ends(kind: str):
    """(socket centre, peg centre) in the part frame."""
    a, ta = centre_line(kind, 0.0)
    b, tb = centre_line(kind, 1.0)
    return a + HOLE_C * ta, b + KNOB_C * tb


def piece_shapes(kind: str):
    """Boxes and cylinders of one piece: the socket end (two cheeks either side of a keyhole), the solid body (box
    segments along the centre line) and the peg (a neck box and a knob cylinder)."""
    boxes, cyls = [], []
    a, ta = centre_line(kind, 0.0)
    na = np.array([-ta[1], ta[0], 0.0])
    yaw_a = math.atan2(ta[1], ta[0])

    def at(origin, u, v, cu, cv):
        return origin + cu * u + cv * v

    half = W / 2
    for sv in (1.0, -1.0):
        # cheeks along the whole socket end, outside the hole
        c = at(a, ta, na, FEM_L / 2, sv * (HOLE / 2 + half) / 2)
        boxes.append(((c[0], c[1], H / 2), (FEM_L, half - HOLE / 2, H), BEECH, _yawq(yaw_a)))
        # the throat's lips, between the throat and the hole's width
        c = at(a, ta, na, THROAT_L / 2, sv * (THROAT_W / 2 + HOLE / 2) / 2)
        boxes.append(((c[0], c[1], H / 2), (THROAT_L, HOLE / 2 - THROAT_W / 2, H), BEECH, _yawq(yaw_a)))
    # the body, from the back of the socket to end B
    if kind == "straight":
        x0 = -L_STRAIGHT / 2 + FEM_L
        x1 = L_STRAIGHT / 2 - END_GAP
        boxes.append((((x0 + x1) / 2, 0.0, H / 2), (x1 - x0, W, H), BEECH))
    else:
        r = R_LARGE if kind == "large" else R_SHORT
        n = 9 if kind == "large" else 5
        step = ARC / n                    # segments centred on the piece's middle (one lies square across it there,
        phi0 = -ARC / 2 + FEM_L / r       # so a pinch at the middle meets a face along the tangent), from the back
        for k in range(n):                # of the socket to the peg end
            fa, fb = max(phi0, -ARC / 2 + k * step), -ARC / 2 + (k + 1) * step
            if fb <= fa:
                continue
            if k == n - 1:
                # the last segment lies along the end's own tangent, its face END_GAP short of the end, so the face is
                # square to the track and clear of the next piece's socket end
                pe, te = centre_line(kind, 1.0)
                length = r * (fb - fa)
                pm, tm = pe - (length / 2 + END_GAP) * te, te
            else:
                pm, tm = centre_line(kind, 0.5 + (fa + fb) / 2 / ARC)
                length = 2 * (r + half) * math.sin((fb - fa) / 2) + 0.0015
            boxes.append(((pm[0], pm[1], H / 2), (length, W, H), BEECH, _yawq(math.atan2(tm[1], tm[0]))))
    # the peg at end B
    b, tb = centre_line(kind, 1.0)
    yaw_b = math.atan2(tb[1], tb[0])
    zc, zh = (PEG_Z[0] + PEG_Z[1]) / 2, PEG_Z[1] - PEG_Z[0]
    c = b + (NECK_L / 2 - 0.0005) * tb
    boxes.append(((c[0], c[1], zc), (NECK_L + 0.001, NECK_W, zh), BEECH, _yawq(yaw_b)))
    k = b + KNOB_C * tb
    # the knob, its top edge rounded: modelled as a 9 mm step on its top 2.5 mm
    z_step = PEG_Z[1] - 0.0025
    cyls.append(((k[0], k[1], (PEG_Z[0] + z_step) / 2), KNOB_R, z_step - PEG_Z[0], BEECH))
    cyls.append(((k[0], k[1], (z_step + PEG_Z[1]) / 2), 0.0045, PEG_Z[1] - z_step, BEECH))
    return boxes, cyls


# -- layout ------------------------------------------------------------------------------------------------------------
LOOP_C = np.array([0.30, 0.0])           # the loop's centre on the table
GAP_K = 0                                # the missing piece's place: k = 0 is the one nearest the right arm


def loop_pose(k: int):
    """Mid point and yaw of the k-th large curve round the loop (counter-clockwise from the right-hand side)."""
    th = -math.pi / 2 + k * ARC
    p = LOOP_C + R_LARGE * np.array([math.cos(th), math.sin(th)])
    return float(p[0]), float(p[1]), th + math.pi / 2


MASS = {"large": 0.050, "straight": 0.046, "short": 0.028}
FRICTION = 0.45                          # varnished beech on beech and on the table's laminate


def _piece(name, kind, x, y, yaw):
    boxes, cyls = piece_shapes(kind)
    return part(name, pos=(float(x), float(y), float(TOP + 0.0005)), yaw=float(yaw), mass=MASS[kind], boxes=boxes,
                cylinders=cyls, color=BEECH, friction=FRICTION, meta={"kind": kind})


LOOP = [_piece(f"loop_{k}", "large", *loop_pose(k)) for k in range(8) if k != GAP_K]
# the loose pieces, in front of the right arm: the large curve turned a quarter turn from its place in the gap
LOOSE_POSE = {"track_a": ("large", 0.16, -0.40, math.pi / 2),
              "track_b": ("straight", 0.40, -0.40, 0.0),
              "track_c": ("short", 0.12, -0.25, math.pi / 2)}
LOOSE = [_piece(n, kind, x, y, yaw) for n, (kind, x, y, yaw) in LOOSE_POSE.items()]
FITS = "track_a"
OBJECTS = [*LOOP, *LOOSE]
PIECES = {p.name: p for p in OBJECTS}
RING = [f"loop_{k}" if k != GAP_K else FITS for k in range(8)]   # round the loop, each piece's peg in the next's socket

# -- scoring -----------------------------------------------------------------------------------------------------------
JOIN_MM = 3.5            # a peg is in its socket: its centre within 3.5 mm of the socket's (the socket is 2.25 mm wider)
FLAT_MM, FLAT_DEG = 2.0, 3.0
NEAR_MM, NEAR_DEG = 20.0, 10.0
RELEASE = 0.03
STILL_STEPS, STILL_MM = 21, 1.0
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.clear()
    _STATE.update(step=0, free=True, violation="", hist=[], updated_at=None, last=(0.0, False))


_reset_state()


def world(st, p):
    return np.asarray(st.to_world(np.asarray(p, float)), float)


def yaw_of(st) -> float:
    R = st.rotation()
    return math.atan2(float(R[1, 0]), float(R[0, 0]))


def tilt_deg(st) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(st.rotation()[2, 2])))))


def flat(st) -> bool:
    return abs(float(st.pos[2]) - TOP) * 1000 <= FLAT_MM + 0.5 and tilt_deg(st) <= FLAT_DEG


def joined(objects, a: str, b: str) -> bool:
    """Piece ``a``'s peg is in piece ``b``'s socket: both flat on the table and the peg's centre within 3.5 mm of the
    socket's."""
    sa, sb = objects[a], objects[b]
    if not (flat(sa) and flat(sb)):
        return False
    peg = world(sa, ends(PIECES[a].meta["kind"])[1])
    sock = world(sb, ends(PIECES[b].meta["kind"])[0])
    return float(np.hypot(*(peg - sock)[:2])) * 1000 <= JOIN_MM


def joints(objects) -> list[bool]:
    return [joined(objects, RING[k], RING[(k + 1) % 8]) for k in range(8)]


def near_gap(st) -> bool:
    """Lying in the gap: its middle within 20 mm of the gap's, heading within 10 degrees of the gap's."""
    x, y, yaw = loop_pose(GAP_K)
    d = math.hypot(float(st.pos[0]) - x, float(st.pos[1]) - y) * 1000
    dy = abs((yaw_of(st) - yaw + math.pi) % (2 * math.pi) - math.pi)
    return d <= NEAR_MM and math.degrees(dy) <= NEAR_DEG


def _near_piece(pt, st, kind) -> bool:
    """A finger point within 30 mm of the piece (its top, sides and ends, peg included)."""
    loc = np.asarray(st.to_local(np.asarray(pt, float)), float)
    if not -RELEASE < loc[2] < H + RELEASE:
        return False
    return any(math.hypot(loc[0] - c[0], loc[1] - c[1]) < W / 2 + RELEASE
               for c, _ in (centre_line(kind, s) for s in np.linspace(0.0, 1.0, 17)))


@on_step
def _step(environment) -> None:
    from partkit import _part_state   # noqa: PLC0415

    _STATE["step"] += 1
    pts = finger_points(environment)
    states = {n: _part_state(environment, p) for n, p in PIECES.items()}
    _STATE["free"] = not any(_near_piece(pt, states[n], PIECES[n].meta["kind"]) for n in PIECES for pt in pts)


def _update(objects) -> tuple[float, bool]:
    for n in PIECES:
        if float(objects[n].pos[2]) < TOP - 0.05 and not _STATE["violation"]:
            _STATE["violation"] = f"{n} fell off the table"
    hist = _STATE["hist"]
    hist.append((np.array([np.asarray(objects[n].pos, float) for n in RING]), _STATE["free"]))
    del hist[:-STILL_STEPS]
    if _STATE["violation"] or not _STATE["free"]:
        return 0.0, False
    a = objects[FITS]
    j = joints(objects)
    own = (j[GAP_K - 1 if GAP_K else 7], j[GAP_K])          # the peg into its socket, and its peg into the next socket
    if all(j):
        still = (len(hist) == STILL_STEPS and all(f for _, f in hist)
                 and max(float(np.linalg.norm(h - hist[-1][0], axis=1).max()) for h, _ in hist) <= STILL_MM / 1000)
        if still:
            return 1.0, True
        return 0.5, False
    if any(own) and flat(a):
        return 0.5, False
    if near_gap(a):
        return 0.25, False
    return 0.0, False


def _once(objects) -> tuple[float, bool]:
    """``_update`` once per control step (success() and score() are both called each step)."""
    if _STATE.get("updated_at") != _STATE["step"]:
        _STATE["updated_at"], _STATE["last"] = _STATE["step"], _update(objects)
    return _STATE["last"]


def success(objects) -> bool:
    return _once(objects)[1]


def score(objects) -> float:
    return _once(objects)[0]


def distance(objects) -> float:
    x, y, _ = loop_pose(GAP_K)
    p = objects[FITS].pos
    return float(math.hypot(float(p[0]) - x, float(p[1]) - y))


def demo():
    """The scripted demonstration is in demo.py."""
    return []
