"""A 165 mm Japanese kitchen knife (a santoku with a round "wa" handle) lies on an end-grain cutting board next to a
1000-grit whetstone held in a non-slip rubber holder. Sharpening it the trained way means holding the blade at 15 to
20 degrees to the stone and drawing it across in edge-leading strokes, a few on each side, then laying the knife back
down flat on the board with its edge facing away.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled knife, stone and board; nothing on disk changes)
import guardkit as gk  # noqa: E402  (the gripper's own shape, for holding and letting go of the knife)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Sharpen the knife on the whetstone, then put it back on the chopping block."


def _dirs(deg):
    """(u, v) for a heading: v along the knife from handle to tip, u across it towards the edge (both horizontal)."""
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])


# --- The knife, in its own frame: lying flat, handle axis along +y (handle at -y, tip at +y), edge towards +x,
# origin on the handle axis at the bolster (where the blade meets the handle).
HANDLE_Y = (-0.095, 0.0)
HANDLE_W, HANDLE_T = 0.020, 0.018  # across (in the blade's plane) and thickness of the octagonal handle
BLADE_Y = (0.0, 0.165)  # 165 mm blade
SPINE_X, EDGE_X = -0.005, 0.040  # at the heel the blade is 45 mm tall; its straight edge 40 mm from the handle axis
SPINE_CURVE = 0.07  # m past the bolster: the spine runs straight to here, then curves down to the tip (santoku shape)
TIP_RISE = (0.135, 0.006)  # the edge is straight to 13.5 cm past the bolster, then rises 6 mm over the last 3 cm
BEVEL_W = 0.006  # the edge bevel's width (the lighter strip along the edge)
BLADE_T = 0.003  # collider thickness (the steel is 2 mm at the spine)
EDGE_KNOTS = np.linspace(TIP_RISE[0], BLADE_Y[1], 5)  # the rising edge as 4 straight pieces (each its own collider)
EDGE_KNOTS_X = EDGE_X - TIP_RISE[1] * ((EDGE_KNOTS - TIP_RISE[0]) / (BLADE_Y[1] - TIP_RISE[0])) ** 2


def edge_x(y):
    """The edge line in the knife's frame: its distance from the handle axis `y` m past the bolster."""
    return np.interp(y, np.r_[0.0, EDGE_KNOTS], np.r_[EDGE_X, EDGE_KNOTS_X])


def spine_x(y):
    """The spine line: straight to SPINE_CURVE, then curving down to meet the edge at the tip."""
    t = np.clip((np.asarray(y, float) - SPINE_CURVE) / (BLADE_Y[1] - SPINE_CURVE), 0.0, 1.0)
    return SPINE_X + (EDGE_KNOTS_X[-1] - SPINE_X) * (1.0 - np.sqrt(1.0 - t ** 1.6))


GRIP_Y = -0.060  # where the demo holds the handle (6 cm behind the bolster)
KNIFE_MASS = 0.10  # kg (a real 165 mm santoku with a wa handle is about 140 g; see fidelity)
EDGE_SAMPLES = np.linspace(0.005, 0.162, 16)  # points along the edge the scorer checks: about 1 cm apart, from 0.5 cm
#                                                past the heel to 16.2 cm, 3 mm short of the point, where the edge
#                                                curves up to meet the spine (the last one 4.9 mm up the tip's rise)

# --- The board and the knife's starting place: the knife lies flat on the board, edge away from the robot, turned
# 15 degrees, its handle over the board's edge so a hand can close round it.
PARK_HEADING = 15.0
PARK_TCP = np.array([0.20, -0.05])  # the handle's grip point at the start
BOARD = (0.12, 0.22, 0.08)  # an end-grain maple butcher block: across x along x height
P_U, P_V = _dirs(PARK_HEADING)
BOARD_POS = PARK_TCP + P_V[:2] * (-GRIP_Y - 0.03 + BOARD[1] / 2) + P_U[:2] * 0.0  # its edge under the handle, 3 cm
#                                                                               behind the bolster
KNIFE_START = np.r_[PARK_TCP - GRIP_Y * P_V[:2], TOP + BOARD[2] + BLADE_T / 2]

# --- The stone, fixed in its holder; its long side along the stroke (u at STONE_HEADING).
STONE_HEADING = -15.0
STONE = (0.18, 0.06, 0.025)  # a standard 180 x 60 x 25 mm combination stone
RISER = 0.06  # the wooden sharpening stand the holder sits on
HOLDER_H = 0.015  # the rubber holder under the stone
STONE_TOP = RISER + HOLDER_H + STONE[2]  # 10 cm above the bench
S_U, S_V = _dirs(STONE_HEADING)
STROKE_TCP_Y = -0.20  # where the hand works: the stone sits under the blade's middle
STONE_V = 0.15  # the stone's middle is under the blade 9 cm past the bolster
STONE_POS = np.array([0.355, STROKE_TCP_Y]) + STONE_V * S_V[:2]

# --- Scoring
BAND = (13.0, 22.0)  # deg: the 15-20 degree sharpening angle, plus 2 degrees each way for a hand-held angle
STEEP = 22.0  # deg: steeper than this the stroke works a steeper, more obtuse micro-bevel instead of the 15-20 degree
#               edge: it is not counted (a real trial sees it on the angle card)
TOUCH = 0.0005  # m: contact: the lowest edge point over the stone within 0.5 mm of its top. The knife's collider
#                 rests on the stone's (rest offset 0), so a knife really on the stone reads 0 give or take the
#                 solver's jitter; a knife held even 1 mm up does not touch
CONTACT = 0.001  # m: with the knife touching, an edge point within 1 mm of the stone's top counts as on it (that part
#                  of the edge bears on the stone; 1 mm covers the jitter and the blade's flex under a light press)
EDGE_ON = 3  # edge points (about 1 cm apart, each standing for about 1 cm of edge) on the stone for a stroke to count:
#               3 cm of edge, half the stone's 6 cm width; a tip or heel dragged alone sharpens nothing
COVER = 12  # of the 16 edge points (each about 1 cm of the 16.5 cm edge): full credit needs each face to have worked
#             at least 12 (three quarters of the edge, "most of it"), so strokes on the same few cm do not count as a
#             sharpened knife
COVER_TRAVEL = 0.10  # m: a point counts as worked once it has been on the stone for 10 cm of in-band, edge-leading
#                      travel on that face: two strokes' worth (raised from one, 5 cm; still a floor, see PER_SIDE)
LEVEL = 5.0  # deg: the blade's length (handle to tip) within this of level while stroking; a whetstone stroke keeps
#               the whole edge on the stone, and a pitch above 5 deg lifts the far end of a 6 cm contact 5 mm off it
STROKE = 0.05  # m: edge-leading travel on the stone, in band, that makes a stroke (most of the stone's 6 cm width
#                     crossed by each part of the edge; the demo strokes 7 cm)
STEEP_TRAVEL = 0.02  # m: a touch with 2 cm of travel steeper than STEEP is a too-steep stroke: none of it counts
PER_SIDE = 5  # strokes on each face, a floor: sharpening guides for a dull edge on a 1000-grit stone work each section
#              of the edge until a burr runs along it, then alternate faces in 10, 8, 6, 4, 2 and 1 strokes per side,
#              far more than 5; the sim's strokes are a lower bound and a real trial's edge tester decides
OFF_END = 0.003  # m: an edge point this far below the stone's top just past an end, within 0.5 s of the edge leaving
#                  the stone: the stroke rolled off the end instead of being lifted, and it is not counted
OFF_END_STEPS = 10  # control steps (0.5 s at 20 Hz) after lift-off in which a roll-off cancels the stroke just ended
# Held: a gripper with both fingers touching the handle (gk.gripping: the gripper's own shape, palm and fingers at
# their present opening, within 2 mm of it, the solver's resting gap plus the fit of its boxes to the meshes) and
# their opening stopped by it: from its 1.8 cm thickness less 15 % of full opening to its 2.0 cm width plus 3 %
# (0.10 to 0.24; 0 closed, 1 open). An empty fist, an open hand round it or a hand merely near it does not hold it.
HANDLE_GRIP = (gk.band(HANDLE_T)[0], gk.band(HANDLE_W)[1])
FLAT = 20.0  # deg: lying flat on a face (not on its spine or edge)
EDGE_AWAY = 0.5  # the edge's horizontal direction has at least this much +x (facing away from the robot, < 60 deg)
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.58, 0.60, 0.63), 0.35, metallic=0.85)  # satin-finished blade
BEVEL = kit.Look((0.95, 0.96, 0.97), 0.2, metallic=0.3)  # the freshly ground edge bevel: a bright strip
WOOD = kit.Look((0.55, 0.36, 0.20), 0.6)  # magnolia handle
HORN = kit.Look((0.10, 0.09, 0.09), 0.4)  # buffalo-horn ferrule
STONE_LOOK = kit.Look((0.85, 0.45, 0.22), 0.9)  # orange 1000-grit stone
RUBBER = kit.Look((0.08, 0.08, 0.09), 0.9)
MAPLE = kit.Look((0.86, 0.72, 0.52), 0.7)
MAPLE_DARK = kit.Look((0.74, 0.58, 0.40), 0.7)


def _strip(rows, look):
    """A closed strip along the blade from (y, x0, x1, half-thickness at x0, at x1) cross-sections."""
    pts, counts, idx = [], [], []
    for y, x0, x1, t0, t1 in rows:
        pts += [(x0, y, t0), (x1, y, t1), (x1, y, -t1), (x0, y, -t0)]
    for i in range(len(rows) - 1):
        a, b = 4 * i, 4 * (i + 1)
        for e0, e1 in ((0, 1), (1, 2), (2, 3), (3, 0)):
            counts.append(4)
            idx += [a + e0, a + e1, b + e1, b + e0]
    for end, ring in ((0, (3, 2, 1, 0)), (len(rows) - 1, (0, 1, 2, 3))):
        counts.append(4)
        idx += [4 * end + k for k in ring]
    return kit._mesh(pts, counts, idx, look)


def _blade_colliders():
    """Boxes following the outline: three from the heel to where the edge starts to rise, each from the spine (at its
    middle) to the edge; one turned box per straight piece of the rising edge, 8 mm deep; two filling the tip."""
    out = []
    for y0, y1 in ((0.0, SPINE_CURVE), (SPINE_CURVE, 0.105), (0.105, TIP_RISE[0])):
        x0 = float(spine_x((y0 + y1) / 2))
        out.append(kit.Collider(((x0 + EDGE_X) / 2, (y0 + y1) / 2, 0.0), size=(EDGE_X - x0, y1 - y0, BLADE_T)))
    deep = 0.008
    for i in range(len(EDGE_KNOTS) - 1):
        (y0, y1), (x0, x1) = EDGE_KNOTS[i:i + 2], EDGE_KNOTS_X[i:i + 2]
        n = math.hypot(x1 - x0, y1 - y0)
        inward = np.array([-(y1 - y0), x1 - x0]) / n  # (x, y) unit normal pointing from the edge towards the spine
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2]) + inward * deep / 2
        out.append(kit.Collider((float(c[0]), float(c[1]), 0.0), size=(deep, n, BLADE_T),
                                rot_z=-math.degrees(math.atan2(x1 - x0, y1 - y0))))
    for y0, y1 in ((TIP_RISE[0], 0.15), (0.15, 0.16)):
        x0, x1 = float(spine_x((y0 + y1) / 2)), float(edge_x(y1)) - deep
        out.append(kit.Collider(((x0 + x1) / 2, (y0 + y1) / 2, 0.0), size=(x1 - x0, y1 - y0, BLADE_T)))
    return tuple(out)


def _knife():
    h0, h1 = HANDLE_Y
    ys = np.unique(np.r_[np.linspace(0.0, BLADE_Y[1], 34), EDGE_KNOTS, BLADE_Y[1] - np.geomspace(0.0005, 0.02, 8)])
    sp, ed = spine_x(ys), edge_x(ys)
    inner = np.maximum(ed - BEVEL_W, sp)  # where the bevel starts (it takes the whole tip where the blade narrows)
    meshes = (
        kit.cylinder((0.0, (h0 + h1 - 0.015) / 2, 0.0), HANDLE_T / 2, h1 - h0 - 0.015, WOOD, axis="y", n=8),
        kit.cylinder((0.0, -0.0075, 0.0), HANDLE_T / 2 + 0.0005, 0.015, HORN, axis="y", n=8),
        _strip([(y, a, b, 0.001, 0.001) for y, a, b in zip(ys, sp, inner)], STEEL),  # 2 mm flat steel
        _strip([(y, a, b, 0.001, 0.00015) for y, a, b in zip(ys, inner, ed)], BEVEL),  # ground to 0.3 mm at the edge
    )
    colliders = (kit.Collider((0.0, (h0 + h1) / 2, 0.0), size=(HANDLE_W, h1 - h0, HANDLE_T)),) + _blade_colliders()
    return kit.Prop("knife", pos=tuple(float(c) for c in KNIFE_START), size=(0.05, 0.26, HANDLE_T), mass=KNIFE_MASS,
                    yaw=math.radians(PARK_HEADING), meshes=meshes, colliders=colliders, com=(0.012, 0.02, 0.0))


HANDLE_SHAPE = gk.Block((0.0, sum(HANDLE_Y) / 2, 0.0), (HANDLE_W, HANDLE_Y[1] - HANDLE_Y[0], HANDLE_T))


def _knife_shape():
    """The knife's colliders as guardkit boxes in its frame; each turned box along the tip's rise is replaced by the box
    that bounds it (at most 2.2 mm longer and wider), which only makes the let-go check a little stricter there."""
    out = [HANDLE_SHAPE]
    for c in _blade_colliders():
        a = math.radians(c.rot_z)
        w = abs(math.cos(a)) * c.size[0] + abs(math.sin(a)) * c.size[1]
        h = abs(math.sin(a)) * c.size[0] + abs(math.cos(a)) * c.size[1]
        out.append(gk.Block(c.center, (w, h, c.size[2])))
    return tuple(out)


KNIFE_SHAPE = _knife_shape()


def _stone():
    hx, hy = STONE[0] + 0.03, STONE[1] + 0.02
    lip = 0.03  # the holder's end clamps reach 3 cm up (1 cm below the stone's top)
    parts = [  # (centre, size, look)
        ((0, 0, RISER / 2), (hx, hy, RISER), WOOD),
        ((0, 0, RISER + HOLDER_H / 2), (hx, hy, HOLDER_H), RUBBER),
        ((0, 0, RISER + HOLDER_H + STONE[2] / 2), STONE, STONE_LOOK),
        (((hx - 0.012) / 2, 0, RISER + lip / 2), (0.012, hy, lip), RUBBER),
        ((-(hx - 0.012) / 2, 0, RISER + lip / 2), (0.012, hy, lip), RUBBER),
    ]
    meshes = tuple(kit.box(c, sz, lk) for c, sz, lk in parts)
    colliders = tuple(kit.Collider(c, size=sz) for c, sz, _ in parts)
    return kit.Prop("stone", pos=(float(STONE_POS[0]), float(STONE_POS[1]), TOP), size=(hx, hy, STONE_TOP),
                    yaw=math.radians(STONE_HEADING), meshes=meshes, colliders=colliders)


def _board():
    meshes = (kit.box((0, 0, BOARD[2] / 2), BOARD, MAPLE),
              kit.box((0, 0, BOARD[2] + 0.0002), (BOARD[0] - 0.02, BOARD[1] - 0.02, 0.0004), MAPLE_DARK))
    return kit.Prop("board", pos=(float(BOARD_POS[0]), float(BOARD_POS[1]), TOP), size=BOARD, yaw=math.radians(PARK_HEADING),
                    meshes=meshes, colliders=(kit.Collider((0, 0, BOARD[2] / 2), size=BOARD),))


OBJECTS = [_board(), _stone(), _knife()]


# --- Geometry from the knife's pose.
def _rot(q):
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _pose(objects):
    k = objects["knife"]
    return np.asarray(k.pos, float), _rot(k.quat_xyzw)


def blade_angle(objects):
    """Degrees between the blade's faces and the horizontal stone top (0 = lying flat, 90 = on its edge)."""
    _, r = _pose(objects)
    return math.degrees(math.acos(min(1.0, abs(r[2, 2]))))


def sharpen_angle(objects):
    """The sharpening angle: degrees the blade's width (spine to edge, knife +x) tilts below the level stone top,
    edge down. Measured across the blade, so pitching the knife tip-down about its width does not count as angle."""
    _, r = _pose(objects)
    return math.degrees(math.asin(max(-1.0, min(1.0, -r[2, 0]))))


def level(objects):
    """The blade's length axis (knife +y, handle to tip) within LEVEL degrees of horizontal."""
    _, r = _pose(objects)
    return abs(r[2, 1]) < math.sin(math.radians(LEVEL))


def side(objects):
    """Which face is down: 'A' (the face that lies on the board at the start) or 'B'."""
    _, r = _pose(objects)
    return "A" if r[2, 2] > 0 else "B"


def _edge_points(objects):
    """World points along the edge's lower corner (both corners of the collider; the lower one is kept per point)."""
    p, r = _pose(objects)
    pts = []
    for y in EDGE_SAMPLES:
        a = p + r @ np.array([edge_x(y), y, BLADE_T / 2])
        b = p + r @ np.array([edge_x(y), y, -BLADE_T / 2])
        pts.append(a if a[2] < b[2] else b)
    return np.array(pts)


def _stone_local(pts):
    """(along the stone's length, across it, height above its top) for world points."""
    d = pts[:, :2] - STONE_POS
    return d @ S_U[:2], d @ S_V[:2], pts[:, 2] - (TOP + STONE_TOP)


def on_stone(objects):
    """The edge touching the stone: its lowest point over the stone's top within TOUCH of it (0.5 mm: resting on it,
    not hovering), with the edge lower than the spine."""
    _, r = _pose(objects)
    if r[2, 0] >= 0:  # the edge (knife +x) is not the low side: the spine or the face is what touches
        return False
    a, c, h = _stone_local(_edge_points(objects))
    over = (np.abs(a) <= STONE[0] / 2) & (np.abs(c) <= STONE[1] / 2)
    return bool(np.any(over)) and float(h[over].min()) <= TOUCH


def edge_mask(objects):
    """Which EDGE_SAMPLES points are over the stone's top and within CONTACT of it."""
    a, c, h = _stone_local(_edge_points(objects))
    return (np.abs(a) <= STONE[0] / 2) & (np.abs(c) <= STONE[1] / 2) & (h <= CONTACT)


def edge_on_stone(objects):
    """How many of the EDGE_SAMPLES points are over the stone's top and within CONTACT of it."""
    return int(np.sum(edge_mask(objects)))


def _off_end(objects):
    """An edge point just past an end of the stone (within 4 cm), across its width, below its top by OFF_END."""
    a, c, h = _stone_local(_edge_points(objects))
    past = (np.abs(a) > STONE[0] / 2) & (np.abs(a) < STONE[0] / 2 + 0.04) & (np.abs(c) <= STONE[1] / 2)
    return bool(np.any(past & (h < -OFF_END)))


def _edge_dir(objects):
    """The horizontal unit direction the edge faces (knife +x projected on the table)."""
    _, r = _pose(objects)
    e = r[:, 0].copy()
    e[2] = 0.0
    n = np.linalg.norm(e)
    return e / n if n > 1e-6 else e


def held(objects):
    """A gripper with both fingers touching the handle and their opening stopped by it (see HANDLE_GRIP)."""
    return bool(gk.gripping(objects, "knife", HANDLE_SHAPE, HANDLE_GRIP))


def _released(objects):
    """Let go: no part of either gripper touching the knife, handle or blade (gk.touching, within 2 mm)."""
    return not gk.touching(objects, "knife", KNIFE_SHAPE)


def on_board(objects):
    """The knife lying flat on the board: its middle over the board, its lowest edge point within 1 cm of the board's
    top, its faces within FLAT of level."""
    p, r = _pose(objects)
    mid = p + r @ np.array([0.0, 0.05, 0.0])
    d = mid[:2] - BOARD_POS
    over = abs(d @ P_U[:2]) <= BOARD[0] / 2 and abs(d @ P_V[:2]) <= BOARD[1] / 2
    low = _edge_points(objects)[:, 2].min() - (TOP + BOARD[2])
    return over and -0.005 <= low <= 0.01 and blade_angle(objects) <= FLAT


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.strokes = {"A": 0, "B": 0}  # strokes counted per face (kept: not too steep, not rolled off the end)
        self.worked = {"A": np.zeros(len(EDGE_SAMPLES)), "B": np.zeros(len(EDGE_SAMPLES))}  # per face, per edge
        #                                       point: in-band edge-leading travel (m) while that point was on the stone
        self.run = None  # the touch in progress: dict(side, band, steep, worked) or None
        self.ended = None  # the touch that just ended, kept until OFF_END_STEPS show it did not roll off the end
        self.since_contact = 99  # control steps since the edge was last on the stone
        self.last_pos = None


_H = _History()


def _new_run(s):
    return {"side": s, "band": 0.0, "steep": 0.0, "worked": np.zeros(len(EDGE_SAMPLES))}


def _keeps(run):
    """A touch that counts: not too steep (under STEEP_TRAVEL of travel steeper than STEEP)."""
    return run is not None and run["steep"] < STEEP_TRAVEL


def _commit(run):
    """Add a finished touch's stroke and worked travel to its face's totals, unless it was too steep."""
    if _keeps(run):
        _H.strokes[run["side"]] += int(run["band"] >= STROKE)
        _H.worked[run["side"]] += run["worked"]


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    p, _ = _pose(objects)
    move = np.zeros(3) if _H.last_pos is None else p - _H.last_pos
    _H.last_pos = p
    contact = on_stone(objects) and held(objects)  # a knife pushed or dragged without a hand on it is not a stroke
    if contact:
        if _H.ended is not None:  # back on the stone before the roll-off window closed: the last touch stands
            _commit(_H.ended)
            _H.ended = None
        ang, s = sharpen_angle(objects), side(objects)
        lead = float(move @ _edge_dir(objects))  # this step's travel in the direction the edge faces
        if _H.run is not None and _H.run["side"] != s:
            _commit(_H.run)
            _H.run = None
        if _H.run is None:
            _H.run = _new_run(s)
        if ang > STEEP:
            _H.run["steep"] += abs(lead)
        elif BAND[0] <= ang and lead > 0 and level(objects) and edge_on_stone(objects) >= EDGE_ON:
            _H.run["band"] += lead
            _H.run["worked"][edge_mask(objects)] += lead
        _H.since_contact = 0
    else:
        if _H.run is not None:  # lifting off ends the touch; it is held back until it did not roll off the end
            _H.ended, _H.run = _H.run, None
        _H.since_contact += 1
        if _H.ended is not None:
            if held(objects) and _off_end(objects):
                _H.ended = None  # rolled off the end: that stroke does not count
            elif _H.since_contact > OFF_END_STEPS:
                _commit(_H.ended)
                _H.ended = None


def _totals():
    """Strokes and worked travel per face so far: the committed touches plus the one in progress or just ended (each
    counted only while it still stands)."""
    strokes = dict(_H.strokes)
    worked = {f: w.copy() for f, w in _H.worked.items()}
    for run in (_H.run, _H.ended):
        if _keeps(run):
            strokes[run["side"]] += int(run["band"] >= STROKE)
            worked[run["side"]] += run["worked"]
    return strokes, worked


def coverage(face):
    """How many of the 16 edge points (about 1 cm each, the last 3 mm short of the point) have been worked on `face`:
    on the stone for 10 cm of in-band, edge-leading travel, in strokes that count."""
    return int(np.sum(_totals()[1][face] >= COVER_TRAVEL))


def put_down_safely(objects):
    """Let go of, lying flat on the board, edge facing away from the robot."""
    return (on_board(objects) and _released(objects) and not held(objects)
            and float(_edge_dir(objects) @ np.array([1.0, 0.0, 0.0])) >= EDGE_AWAY)


def _job(objects):
    strokes = _totals()[0]
    a, b = strokes["A"], strokes["B"]
    if min(a, b) >= PER_SIDE:
        if min(coverage("A"), coverage("B")) < COVER:
            return 0.5  # both faces stroked, but on a face less than three quarters of the edge was worked
        return 1.0 if put_down_safely(objects) else 0.75
    if max(a, b) >= PER_SIDE:
        return 0.25  # one side only: the edge is pushed over to one side (a wire edge), not sharp
    return 0.0


def _score(objects):
    return _job(objects)


def success(objects):
    """Both faces sharpened along most of the edge, and the knife put down safely and still (where the
    arms end is not scored). Called first each step, so it also records the history."""
    _update(objects)
    still = np.linalg.norm(np.asarray(objects["knife"].lin_vel, float)) < 0.01
    return _job(objects) == 1.0 and still


def score(objects):
    """A stroke: the knife held, its edge touching the stone (its lowest point over the stone within 0.5 mm of its top,
    edge lower than spine) with at least 3 cm of edge within 1 mm of it, and moved 5 cm edge-leading with its faces 13
    to 22 degrees to the stone and its length level, in one touch (lifting off ends it). An edge point is worked on a
    face once it has been on the stone for 10 cm of such travel. A touch that does not count at all: one with 2 cm of
    travel steeper than 22 degrees (it works a steeper micro-bevel), or one whose edge drops 3 mm below the stone's top
    just past an end within 0.5 s of leaving it (rolled off the end). 0 with under 5 strokes on both faces; 0.25 for
    5+ strokes on one face only; 0.5 for 5+ on each face with under 12 of the 16 edge points (about 1 cm apart, 0.5 to
    16.2 cm along the 16.5 cm edge) worked on a face; 0.75 for 5+ strokes and 12+ points worked on each face; 1.0
    for that and the knife let go, lying flat on the board, edge facing away. Where the arms end is not scored.
    Possible scores: 0, 0.25, 0.5, 0.75, 1."""
    return _score(objects)


# --- Scripted demo (right arm only; the left arm stays home).
def _axrot(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def _hand(heading, spin):
    """The right hand's rotation for a side grip on the handle: fingers closing across the handle, approach along the
    knife (v at `heading`), turned `spin` degrees about the handle; spin -90 holds the knife flat (face A down),
    -90 + a tilts the edge down by a degrees, +90 - a turns it over (face B down) with the edge down by a."""
    import yam_arena.demo as demo
    r0 = _axrot((0, 0, 1), math.radians(heading)) @ kit._rot_xyz(math.pi / 2, 0.0) @ demo.grip_rotation(math.pi / 2)
    return tuple((_axrot(r0 @ np.array([0.0, 0.0, 1.0]), math.radians(spin)) @ r0).ravel())


DEMO_ANGLE = 17.0  # deg: the angle the demo aims for on the stone
DEMO_TRIM = {"A": 2.5, "B": -2.5}  # deg: the knife sits about 2.5 deg low in the grip on face A and high on B (sim)
GRIP_OPEN = 0.45 * FINGER_OPEN  # part open: room round the 18 mm handle without the lower finger hitting the bench
DEMO_PRESS = 0.001  # m: the edge aimed this far into the stone
DEMO_SECTIONS = (0.035, 0.0825, 0.140)  # m past the bolster: the heel, middle and tip of the blade over the stone's
#                                         middle in turn (the stone's 6 cm covers about 6 cm of edge each time; the
#                                         first and last scorer points, 0.5 and 16.2 cm, sit 2 to 3 cm inside it)
DEMO_PER = 2  # strokes with each section on the stone
DEMO_TIP_LIFT = {"A": 2.5, "B": 3.5}  # deg: on the tip strokes the handle is lifted (tip down) so the edge's 6 mm rise
#                                       lies on the stone up to the last scorer point, as a sharpener does for the tip.
#                                       Face B's pull strokes turn the knife back about 2 deg in the grip, so it
#                                       starts higher (sim: face B's blade 1.4 to 3.9 deg off level on the stone,
#                                       inside the 5 deg LEVEL rule)
DEMO_TIP_YAW = -10.0  # deg: for the tip strokes the knife is turned 10 deg on the stone (tip further from the robot).
#                       Square to the stone, the tip strokes put the wrist (joints 5 and 4) at its limit: the hand
#                       fell 1 to 1.5 cm short and the handle lift was lost, so the last 1 cm of edge never reached
#                       the stone. Turned, every stroke is reached with the wrist at least 4.6 deg from its limit.
DEMO_RAISE = {("B", 2): 0.001}  # m: face B's tip strokes aimed at the stone's top, not DEMO_PRESS into it: a press
#                                 there turns the knife in the grip (up to 2 deg) and lifts the tip off the stone
DEMO_EXTRA = {("B", 2): 1}  # one more stroke with face B's tip on the stone: margin for its last 1 cm, the point the
#                             hardest for the arm to keep on the stone


def demo():
    """Right arm: slide the open hand sideways round the handle (fingers above and below it), lift, sharpen face A with
    push strokes (edge leading, away from the robot), two each with the heel, middle and tip of the blade over the
    stone (for the tip, the knife turned 10 deg on the stone and the handle lifted a little), lift straight up, turn
    the knife over and sharpen face B the same way with pull strokes (three with the tip), lay it back on the board
    edge-away, let go, back out and go home."""
    from yam_arena.constants import FINGER_CLOSED
    T = kit.Tilted
    z_board = TOP + BOARD[2] + BLADE_T / 2
    pu, pv = P_U, P_V
    grip = np.r_[PARK_TCP, z_board]
    beside = grip + 0.06 * pu  # past the spine-to-edge line, clear of the blade (which starts 6 cm further along)
    flat = _hand(PARK_HEADING, -90.0)
    right = [T(2.0, tuple(beside + [0, 0, 0.03]), GRIP_OPEN, "beside the handle", rot=flat),
             T(1.0, tuple(beside), GRIP_OPEN, "down beside it", rot=flat),
             T(1.2, tuple(grip), GRIP_OPEN, "slide round the handle", rot=flat),
             T(0.8, tuple(grip), FINGER_CLOSED, "grip", rot=flat),
             T(1.2, tuple(grip + [0, 0, 0.06]), FINGER_CLOSED, "lift", rot=flat)]
    centre = np.r_[STONE_POS, 0.0] - STONE_V * np.r_[S_V[:2], 0.0]  # the grip point when the blade's middle is on the stone

    def strokes(sign, n=DEMO_PER):
        """For each section of the blade in turn, n strokes edge-leading, each from the stone's middle to 6.5 cm past
        it: sign +1 pushes along +u (face A, edge towards +u, the far half of the stone), -1 pulls (face B, the near
        half)."""
        ang = DEMO_ANGLE + DEMO_TRIM["A" if sign > 0 else "B"]
        reach = math.cos(math.radians(ang)) * EDGE_X
        z = TOP + STONE_TOP + math.sin(math.radians(ang)) * EDGE_X - DEMO_PRESS
        out = []
        for i, v in enumerate(DEMO_SECTIONS):
            tip = i == len(DEMO_SECTIONS) - 1
            heading = STONE_HEADING + (DEMO_TIP_YAW if tip else 0.0)
            su, sv = _dirs(heading)
            rot0 = np.array(_hand(heading, -90.0 + ang if sign > 0 else 90.0 - ang)).reshape(3, 3)
            c = np.r_[STONE_POS, 0.0] - (v - GRIP_Y) * np.r_[sv[:2], 0.0]  # the grip point with section v on the stone
            # face A starts with the edge 5 mm short of the stone's middle; face B 8 mm past it, so the pull stroke's
            # start stays within the arm's 40 cm forward reach with the heel on the stone (it ends 7.3 cm past the
            # middle, inside the stone's 9 cm half-length)
            a = c - sign * su * (reach + (0.005 if sign > 0 else -0.008))
            a[2] = z + DEMO_RAISE.get(("A" if sign > 0 else "B", i), 0.0)
            lift = math.radians(DEMO_TIP_LIFT["A" if sign > 0 else "B"]) if tip else 0.0
            # tip down: turn about the stone's long axis (the knife points along +v), and raise the hand so the
            # section over the stone stays at the same height
            rot = tuple((_axrot(su, -lift) @ rot0).ravel())
            a[2] += (v - GRIP_Y) * math.sin(lift)
            b = a + sign * su * 0.065
            out += [T(1.2, tuple(a + [0, 0, 0.03]), FINGER_CLOSED, "over the stone", rot=rot)]
            for _ in range(n + DEMO_EXTRA.get(("A" if sign > 0 else "B", i), 0)):
                out += [T(0.6, tuple(a), FINGER_CLOSED, "edge on the stone", rot=rot),
                        T(1.4, tuple(b), FINGER_CLOSED, "stroke", rot=rot),
                        T(0.5, tuple(b + [0, 0, 0.02]), FINGER_CLOSED, "lift", rot=rot),
                        T(0.8, tuple(a + [0, 0, 0.02]), FINGER_CLOSED, "back", rot=rot)]
        return out

    right += strokes(1.0)
    right += [T(0.8, tuple(np.array(right[-1].pos) + [0, 0, 0.04]), FINGER_CLOSED, "lift clear before turning",
                rot=right[-1].rot),  # straight up first: turning from the tip strokes' place, the edge caught the stone
              T(1.5, tuple(centre + [0, 0, TOP + STONE_TOP + 0.07]), FINGER_CLOSED, "turn the knife over",
                rot=_hand(STONE_HEADING, 0.0))]
    right += strokes(-1.0)
    right += [T(1.0, tuple(centre + [0, 0, TOP + STONE_TOP + 0.07]), FINGER_CLOSED, "lift clear of the stone",
                rot=_hand(STONE_HEADING, 90.0 - DEMO_ANGLE - DEMO_TRIM["B"])),
              T(1.2, tuple(centre + [0, 0, TOP + STONE_TOP + 0.07]), FINGER_CLOSED, "turn back upright",
                rot=_hand(STONE_HEADING, 0.0))]
    right += [T(2.0, tuple(grip + [0, 0, 0.06]), FINGER_CLOSED, "back over the board", rot=flat),
              T(1.2, tuple(grip + [0, 0, 0.004]), FINGER_CLOSED, "lay it down", rot=flat),
              T(0.6, tuple(grip + [0, 0, 0.004]), GRIP_OPEN, "let go", rot=flat),
              T(1.0, tuple(beside + [0, 0, 0.004]), GRIP_OPEN, "slide out", rot=flat),
              T(0.8, tuple(beside + [0, 0, 0.06]), GRIP_OPEN, "up", rot=flat),
              kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    total = sum(w.duration for w in right)
    left = [kit.Home(total - 1.5), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
